/**
  * @file       app_bt.c
  * @brief      蓝牙（UART5）：$B 行协议、Beam 参数读写、机械臂编辑模式。
  *
  *             设计要点（与 CLAUDE.md 的铁律一一对应）：
  *              1. vBtTask 是 UART5 的**唯一读者**，编辑模式期间也是——脚本任务
  *                 从不读串口，只在握手阶段 1 里通过 Beam 标志让位（vBtEditPump）。
  *                 于是「脚本不会碰舵机」是结构性成立的，点动与路线抢不了舵机。
 *              2. 全部发送走 vUartSendArray（2026-09 起 = TX DMA 环形缓冲, 拷进 512B
 *                 环即返回; 环满阻塞背压 ≤500ms, 超时丢弃并计 usTxDrop——所以 LIST 这类
 *                 ~17KB 的多行响应天然被背压消化, 不再需要老版"忙等 HAL_UART_Transmit
 *                 会拖累 vPidTask"的顾虑; 逐行发 + 行间 vTaskDelay(1) 的节奏保留,
 *                 让对端的行缓冲跟得上）。
  *              3. **绝不在 taskENTER_CRITICAL 里发串口**：APP_TIM_PRIO_SERVO(5) 恰等于
  *                 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，临界区会屏蔽 TIM7
  *                 舵机中断与 TIM6 时基 → HAL_MAX_DELAY 超时永不触发 → 真死锁 + 舵机卡顿。
  *                 本模块读 Beam 参数一律单字直读，不加锁（Beam 规则第 4 条）。
  *              4. 不用 vUartPrintf：它把 vsnprintf 的**应写长度**直接喂给
  *                 HAL_UART_Transmit（app_uart.c），行长超过栈缓冲会越界读。
  *                 这里自己 snprintf 进栈缓冲并显式取长度。
  */

#include "app_bt.h"
#include "app_beam.h"
#include "app_arm.h"
#include "app_arm_action.h"
#include "app_chassis.h"     /* vChassisMove(0,0,0): 让位前先停车 */
#include "app_chassis_action.h"  /* ucChActPreviewExists + 预览请求槽（$B,CPLAY/CSTOP） */
#include "app_rpi.h"         /* vRpiKickPiClient（退出编辑模式立刻踢 Pi 重发握手） */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 常量 ==================== */

#define BT_RX_LINE_MAX            128u   /* 单行接收缓冲（含 '\0'）*/
#define BT_TX_LINE_MAX            256u   /* 单行发送缓冲（含 CRLF 与 '\0'）*/
#define BT_MAX_TOK                14u    /* 一行最多切出的 token 数（ARM 8 个 / 底盘动作组 CIT 12 个）*/

/* 编辑模式看门狗：这么久没收到任何 $B 指令就认为 PC 掉线/拔线，主动退出让位。
 * 没有它的话 PC 崩了会把脚本永久卡在握手等待里（机器人不动、不响、也不显示）。 */
#define BT_EDIT_IDLE_TIMEOUT_MS   8000u

#define BT_JOG_DEFAULT_MS         250u   /* 点动默认 S 曲线时长（比赛语义, 走完整 S 曲线） */
#define BT_JOG_MIN_MS             50u
#define BT_JOG_MAX_MS             2000u

/* ms=0 的跟手模式**没有额外阈值**: 速度由驱动里的 ARM_SERVO_MAX_SPEED_DPS 统一管
 * （app_config.h, 可在「调参」Tab 调）。早期版本用过「超过 8° 就降级短 S 曲线」那种
 * 分档, 现在不需要了 —— 恒速趋近本身就是削过速的。 */

/* ==================== 模块内状态（只有 vBtTask 访问） ==================== */

static char     s_acLine[BT_RX_LINE_MAX];
static uint16_t s_usLineLen      = 0;
static uint8_t  s_ucLineOverflow = 0;   /* 1 = 正在丢弃一条超长行（丢到下一个换行） */
static uint32_t s_ulSyncErrTick  = 0;   /* ERR,SYNC 限速用 */

/* ==================== 数值格式化 / 解析 ==================== */
/* 都不用 %f / atof：既省掉完整 printf 库的体积，也避开 MicroLib 下 %f 输出空白的坑。 */

/* 定点格式化，等价 "%.2f"（四舍五入到两位小数）。公开（app_bt.h）供 app_rpi 复用 */
void bt_fmt_f2(char *pcOut, int iSize, float fVal)
{
    long    lInt;
    long    lFrac;
    uint8_t ucNeg = (fVal < 0.0f) ? 1u : 0u;

    if (fVal != fVal) { snprintf(pcOut, (size_t)iSize, "nan"); return; }   /* NaN */
    if (fVal > 1.0e9f)  { snprintf(pcOut, (size_t)iSize, "inf"); return; }
    if (fVal < -1.0e9f) { snprintf(pcOut, (size_t)iSize, "-inf"); return; }
    if (ucNeg) { fVal = -fVal; }

    lInt  = (long)fVal;
    lFrac = (long)((fVal - (float)lInt) * 100.0f + 0.5f);
    if (lFrac >= 100L) { lFrac -= 100L; lInt += 1L; }   /* 进位（如 0.999 → 1.00）*/
    if ((lInt == 0L) && (lFrac == 0L)) { ucNeg = 0u; }  /* 别输出 "-0.00" */

    snprintf(pcOut, (size_t)iSize, "%s%ld.%02ld", ucNeg ? "-" : "", lInt, lFrac);
}

/* 解析十进制浮点（可选 +/-、可选小数部分）。成功置 *pbOk=1。
 * 尾部有多余字符即判失败（严格），免得 "12abc" 被悄悄当 12。 */
static float bt_parse_float(const char *pcStr, uint8_t *pbOk)
{
    float   fVal  = 0.0f;
    float   fFrac = 0.1f;
    uint8_t ucNeg = 0u;
    uint8_t ucAny = 0u;

    *pbOk = 0u;
    if (pcStr == NULL) { return 0.0f; }

    if (*pcStr == '-')      { ucNeg = 1u; pcStr++; }
    else if (*pcStr == '+') { pcStr++; }

    while ((*pcStr >= '0') && (*pcStr <= '9'))
    {
        fVal = fVal * 10.0f + (float)(*pcStr - '0');
        pcStr++;
        ucAny = 1u;
    }
    if (*pcStr == '.')
    {
        pcStr++;
        while ((*pcStr >= '0') && (*pcStr <= '9'))
        {
            fVal += (float)(*pcStr - '0') * fFrac;
            fFrac *= 0.1f;
            pcStr++;
            ucAny = 1u;
        }
    }
    if ((ucAny == 0u) || (*pcStr != '\0')) { return 0.0f; }

    *pbOk = 1u;
    return ucNeg ? -fVal : fVal;
}

/* ==================== 发送 ==================== */

/* 组装并发送一行（自动补 CRLF，一次 HAL_UART_Transmit 发完）。 */
static void bt_sendf(const char *pcFormat, ...)
{
    char    acBuf[BT_TX_LINE_MAX];
    va_list stArg;
    int     lLen;

    va_start(stArg, pcFormat);
    lLen = vsnprintf(acBuf, sizeof(acBuf) - 3u, pcFormat, stArg);
    va_end(stArg);

    if (lLen < 0) { return; }
    /* ⚠️ 判据必须是 `>=`: vsnprintf 返回的是「**本应**写入的长度」(不含 '\0')。
     * 它恰好等于 cap-3（=253）时, 实际只写了 252 个字符 + acBuf[252]='\0',
     * 而 lLen 仍是 253 —— 用 `>` 就漏掉这一格, 于是那个 NUL 被当成正文发出去,
     * PC 端拿到一行中间带 0x00 的破行（$B,LIST 少一条参数, 且不报错）。
     * 当前最长的一条 $B,P 约 100B, 属潜伏缺陷 —— 但给某个 label/unit 加长就会踩到。 */
    if (lLen >= (int)(sizeof(acBuf) - 3u)) { lLen = (int)(sizeof(acBuf) - 3u) - 1; }

    acBuf[lLen]      = '\r';
    acBuf[lLen + 1]  = '\n';
    acBuf[lLen + 2]  = '\0';

    vUartSendArray(UART_BT, (const uint8_t *)acBuf, (uint32_t)lLen + 2u);
}

/* 每条响应的终止符：PC 端「发一条 → 读到 $B,OK 或 $B,ERR 或超时」 */
static void bt_ok(void)             { bt_sendf("$B,OK"); }
static void bt_err(const char *pcCode) { bt_sendf("$B,ERR,%s", pcCode); }

/* ==================== 参数表辅助 ==================== */

static const char *bt_type_name(uint8_t ucType)
{
    switch (ucType)
    {
    case BEAM_VAL_I32: return "I32";
    case BEAM_VAL_U8:  return "U8";
    case BEAM_VAL_I8:  return "I8";
    default:           return "F32";
    }
}

/* 按类型取值并格式化成字符串（整数型走整数格式，免得出现 "230.00"） */
static void bt_fmt_param_value(char *pcOut, int iSize, const stBeamParamTdf *pstParam)
{
    float fVal = pstParam->pfGet();

    switch (pstParam->ucType)
    {
    case BEAM_VAL_F32:
        bt_fmt_f2(pcOut, iSize, fVal);
        break;
    case BEAM_VAL_U8:
        snprintf(pcOut, (size_t)iSize, "%d", (int)(uint8_t)fVal);
        break;
    case BEAM_VAL_I8:
        snprintf(pcOut, (size_t)iSize, "%d", (int)(int8_t)fVal);
        break;
    default:
        {
            int32_t iVal = (fVal >= 0.0f) ? (int32_t)(fVal + 0.5f) : (int32_t)(fVal - 0.5f);
            snprintf(pcOut, (size_t)iSize, "%ld", (long)iVal);
        }
        break;
    }
}

/* ==================== 编辑模式 ==================== */

/* 编辑模式是否**真正生效**。
 * ⚠️ 判据必须与 bt_cmd_mode 的**两条入场路径**一一对应（只改一边就是「进得去、动不了」）：
 *   ① 让位窗: bt_hs_wait=1（脚本停在握手阶段 1, 或路线里的 STEP_ARM_EDIT_WAIT）;
 *   ② 脚本根本没在跑（vScriptRun 已返回 / 任务停车）—— 那时路线动不了舵机,
 *      让操作员点动是安全的, 而这正是台架调臂最常用的姿态。
 * ⚠️ 原来这里**只认 ①** —— 于是从 ② 进来时: `$B,MODE,1` 被放行、控制台 toast 报
 *    「已进入机械臂编辑模式」并放开滑杆与「获取当前位姿」, 而随后的每一个
 *    JOG/GRIP/ARM/POS/PLAY 全回 `NOTEDIT`。现场表现就是「拖了没反应」。 */
static uint8_t bt_edit_active(void)
{
    if (ucGetBtEditMode() == 0u) { return 0u; }
    if (ucGetBtHsWait() != 0u)   { return 1u; }
    return (ucGetScriptRunning() == 0u) ? 1u : 0u;
}

/* 编辑模式的空闲看门狗 —— **只服务入场路径 ②**（脚本没在跑）。
 * 让位窗（①）那条由 vBtEditPump 在脚本任务的循环里逐轮喂（每 10ms 判一次）;
 * 而脚本空闲时**没有任何人在跑那个泵**, 于是 PC 掉线/拔线后 edit_mode 会一直留着 1:
 * 后到的 `$B,JOG`（没经 MODE,1）照样能动人, 控制台/OLED 的所见与设备也不一致。
 * 放在 vBtTask 取帧超时的那一拍兜底 —— 那正是"没人说话"的那一拍。
 * ⚠️ 刻意**不看链路状态**: 「链路 OK 就退出」是让位窗专有的语义（Pi 上线了脚本该继续走）,
 *    脚本空闲时链路本来就是 OK 的, 照搬会把 ② 当场掐死。 */
static void bt_edit_idle_check(void)
{
    if (ucGetBtEditMode() == 0u)    { return; }
    if (ucGetBtHsWait() != 0u)      { return; }   /* 让位窗: 交 vBtEditPump 管, 别抢 */
    if (ucGetScriptRunning() != 0u) { return; }   /* 脚本在跑: vScriptRun 进场已收回 */
    if ((xTaskGetTickCount() - ulGetBtCmdTick())
        > (TickType_t)pdMS_TO_TICKS(BT_EDIT_IDLE_TIMEOUT_MS))
    {
        vSetBtEditMode(0u);
    }
}

uint8_t vBtEditPump(void)
{
    if (ucGetBtEditMode() == 0u) { return 0u; }

    /* 看门狗：PC 掉线/拔线时主动退出，别把脚本永久卡在握手等待里 */
    if ((xTaskGetTickCount() - ulGetBtCmdTick()) > (TickType_t)pdMS_TO_TICKS(BT_EDIT_IDLE_TIMEOUT_MS))
    {
        vSetBtEditMode(0u);
        return 0u;
    }

    /* 树莓派上线了 → 立刻交还脚本。
     * ⚠️ 这里用**非破坏性**的链路状态判断，绝不能调读后清零的 ucTakeHandshakeSeen()
     *    —— 那会把握手标志吃掉，阶段 1 就永远等不到了。标志是 latched 的，
     *    阶段 1 的 while 会在本函数返回后立刻看到它并放行。 */
    if (emGetRpiLinkState() == RPI_LINK_OK)
    {
        vSetBtEditMode(0u);
        return 0u;
    }

    vTaskDelay(pdMS_TO_TICKS(10));   /* 停住 10ms；调用方本轮不要再延时 */
    return 1u;
}

/*
 * 强制让位给机械臂编辑模式并停住, 直到操作员按 MAIN 键（或 PC 侧退出编辑模式）。
 * 路线中途插一块「我要手动操纵机械臂」用（脚本动作 STEP_ARM_EDIT_WAIT → 本函数）。
 *
 * 三件事, 一件都不能少:
 *   ① **链路状态钉在 WAITING**。编辑模式的**退出门**是「链路 OK」(见上面 vBtEditPump) ——
 *      所以不能让状态变 OK; 而**也不能设 DISCONNECTED**: vRpiTask 有「Disconnected 态收到
 *      任何有效帧即恢复 OK」的规则（给 Pi 忙完/重连发心跳用的）, Pi 心跳 2s 一拍, 窗口转眼
 *      就被闩掉。只有 WAITING 是稳定的（它只被握手或「已握手过再进来」改）。
 *      ⚠️ 本函数**不真的断开 Pi**、也不给它发指令: Pi 照常发心跳/回结果, 只是本端不认它上线。
 *      要真让 Pi 停手得另发 RPI_TASK_END（"end"）。
 *   ② **bt_hs_wait 拉高**: `$B,MODE` 的入场条件是它（否则回 NOHANDSHAKE）—— 于是
 *      「机械臂」Tab 能进编辑模式点动六个关节。
 *   ③ **先停车**: 让位期间车不能还在跑（前一步可能是 NO_WAIT 速度模式）。
 *
 * 退出时把链路状态**恢复成 OK**: Pi 本来就是连着的（只是本端刚才不认）, 而留在 WAITING
 * 会让后续「等待握手」步骤去等一个**永远不会再来的** SYNC_HS_1（Pi 只在开机阶段发）——
 * 那是本项目最经典的卡死之一。真没连 Pi 的话, 8s 无帧的看门狗会把它自动翻成 DISCONNECTED。
 * ⚠️ 别在这里动 ucTakeHandshakeSeen(): 读后清零, 会破坏真正的握手等待（vBtEditPump 里
 *    专门有一条注释记这个坑）。
 * 仅任务上下文调用（内部用 vTaskDelay）。
 */
void vBtArmEditHold(void)
{
    uint8_t ucWasActive = 0u;

    vChassisMove(0, 0, 0);              /* ③ 让位前先停车 */
    vSetRpiLinkState(RPI_LINK_WAITING); /* ① 开窗: 链路不能是 OK */
    vSetBtHsWait(1u);                   /* ② 允许 PC 进编辑模式 */

    for (;;)
    {
        if (vBtEditPump() == 0u)
        {
            /* 没停住 = 编辑模式没开（PC 还没点「进入编辑模式」）或刚从编辑模式退出。
             * 后者说明 PC 那边收工了 → 交还脚本, 不用再等按键。 */
            if (ucWasActive != 0u) { break; }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            ucWasActive = 1u;           /* 编辑模式已生效（vBtEditPump 内已延时 10ms） */
        }

        /* ⚠️ 中止请求也是出口 —— 与 app_script.c::vScriptWaitKeyConfirm 同一个理由:
         *    路线停在这一步等 MAIN 时, 脚本任务没人去取菜单投的请求, 菜单就「点了没反应」。
         *    出场走**同一个收尾**（下面 break 之后的 bt_hs_wait 归零 + 链路回 OK）,
         *    所以这里是干净退出, 不是半途而废。 */
        if (ucGetScriptAbortReq() != 0u) { break; }

        if (ucTakeMenuConfirm() != 0u) { break; }   /* 操作员按 MAIN: 收工 */
    }

    vSetBtHsWait(0u);                   /* 关窗: 脚本继续跑, 不再让位 */
    vSetRpiLinkState(RPI_LINK_OK);      /* Pi 一直是连着的（见函数头注释） */
}

/* ==================== 指令处理 ==================== */

/* 把一行按 ',' 切分成 token（原地把 ',' 换成 '\0'），返回 token 数 */
static uint8_t bt_split(char *pcLine, char *apcTok[], uint8_t ucMax)
{
    uint8_t ucN = 0u;
    char   *pc   = pcLine;

    if (ucMax == 0u) { return 0u; }
    apcTok[ucN++] = pc;

    while (*pc != '\0')
    {
        if (*pc == ',')
        {
            *pc = '\0';
            if (ucN >= ucMax) { break; }
            apcTok[ucN++] = pc + 1;
        }
        pc++;
    }
    return ucN;
}

/* ---- PING ---- */
static void bt_cmd_ping(void)
{
    bt_sendf("$B,PONG,1");
    bt_ok();
}

/* ---- LIST ----
 * $B,LIST,<n>,<m>
 *   n × $B,P,<name>,<label>,<group>,<type>,<rw>,<warn>,<lo>,<hi>,<unit>,<value>[,<options>]
 *   m × $B,J,<idx>,<name>,<min>,<max>,<angle>
 * 逐行发 + 行间让出 1 tick：单行 ~6ms 的 HAL_UART_Transmit 是忙等，整块一次发会
 * 让 50ms 的 vPidTask 累积抖上百毫秒。⚠️ 别在路线正跑 PID 移动时拉 LIST。
 * ⚠️ `<options>`（"0=主口|1=副口"）**只有该参数填了 pcOptions 才追加** —— 没填的行
 *    一个字节都不变（老客户端的 `len(parts)==12` 判据照旧成立）。 */
static void bt_cmd_list(void)
{
    const stBeamParamTdf *pstTab;
    stArmJointTdf         astJoint[ARM_GROUP_NUM];
    uint16_t              usNum = 0u;
    uint16_t              usI;
    uint8_t               ucJnum;
    uint8_t               ucJ;
    char                  acVal[24];

    pstTab = cpstGetBeamParamTable(&usNum);
    ucJnum = ucArmFillJointTable(astJoint, (uint8_t)ARM_GROUP_NUM);

    bt_sendf("$B,LIST,%u,%u", (unsigned)usNum, (unsigned)ucJnum);

    for (usI = 0u; usI < usNum; usI++)
    {
        char acLo[16];
        char acHi[16];

        bt_fmt_param_value(acVal, (int)sizeof(acVal), &pstTab[usI]);
        bt_fmt_f2(acLo, (int)sizeof(acLo), pstTab[usI].fMin);
        bt_fmt_f2(acHi, (int)sizeof(acHi), pstTab[usI].fMax);

        /* 字段顺序: name,label,group,type,rw,warn,lo,hi,unit,value[,options] */
        if (pstTab[usI].pcOptions != NULL)
        {
            bt_sendf("$B,P,%s,%s,%s,%s,%u,%u,%s,%s,%s,%s,%s",
                     pstTab[usI].pcName,
                     pstTab[usI].pcLabel,
                     pstTab[usI].pcGroup,
                     bt_type_name(pstTab[usI].ucType),
                     (unsigned)pstTab[usI].ucRW,
                     (unsigned)pstTab[usI].ucWarn,
                     acLo,
                     acHi,
                     pstTab[usI].pcUnit,
                     acVal,
                     pstTab[usI].pcOptions);
        }
        else
        {
            bt_sendf("$B,P,%s,%s,%s,%s,%u,%u,%s,%s,%s,%s",
                     pstTab[usI].pcName,
                     pstTab[usI].pcLabel,
                     pstTab[usI].pcGroup,
                     bt_type_name(pstTab[usI].ucType),
                     (unsigned)pstTab[usI].ucRW,
                     (unsigned)pstTab[usI].ucWarn,
                     acLo,
                     acHi,
                     pstTab[usI].pcUnit,
                     acVal);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for (ucJ = 0u; ucJ < ucJnum; ucJ++)
    {
        char acLo[16];
        char acHi[16];
        char acCur[16];

        bt_fmt_f2(acLo, (int)sizeof(acLo), astJoint[ucJ].fMin);
        bt_fmt_f2(acHi, (int)sizeof(acHi), astJoint[ucJ].fMax);
        bt_fmt_f2(acCur, (int)sizeof(acCur),
                  c_pstGetServoDeviceParam(astJoint[ucJ].emDev)->stRunningParam.fCurrentValue);

        bt_sendf("$B,J,%u,%s,%s,%s,%s", (unsigned)ucJ, astJoint[ucJ].pcName, acLo, acHi, acCur);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    bt_ok();
}

/* ---- GET,<name> ---- */
static void bt_cmd_get(const char *pcName)
{
    const stBeamParamTdf *pstParam = cpstFindBeamParam(pcName);
    char                  acVal[24];

    if (pstParam == NULL) { bt_err("NOTFOUND"); return; }

    bt_fmt_param_value(acVal, (int)sizeof(acVal), pstParam);
    bt_sendf("$B,GET,%s,%s", pstParam->pcName, acVal);
    bt_ok();
}

/* ---- SET,<name>,<val> ---- */
static void bt_cmd_set(const char *pcName, const char *pcVal)
{
    const stBeamParamTdf *pstParam;
    char                  acVal[24];
    float                 fVal;
    uint8_t               ucOk = 0u;
    uint8_t               ucRet;

    fVal = bt_parse_float(pcVal, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }

    pstParam = cpstFindBeamParam(pcName);
    if (pstParam == NULL) { bt_err("NOTFOUND"); return; }

    ucRet = ucBeamApply(pcName, fVal);
    if (ucRet == 1u) { bt_err("NOTFOUND"); return; }
    if (ucRet == 2u) { bt_err("READONLY"); return; }
    if (ucRet == 3u)
    {
        char acLo[16];
        char acHi[16];
        bt_fmt_f2(acLo, (int)sizeof(acLo), pstParam->fMin);
        bt_fmt_f2(acHi, (int)sizeof(acHi), pstParam->fMax);
        bt_sendf("$B,ERR,RANGE,%s,%s", acLo, acHi);
        return;
    }

    bt_fmt_param_value(acVal, (int)sizeof(acVal), pstParam);
    bt_sendf("$B,SET,%s,%s", pstParam->pcName, acVal);
    bt_ok();
}

/* ---- MODE,<0|1> ----
 * 进入编辑模式只在「脚本正停在握手阶段 1」时允许——这样脚本确实不会碰舵机，
 * 无需 vTaskSuspend，也不需要第二个串口读者。
 * ⚠️ 退出时**什么都不动舵机**：脚本一离开让位状态就自己接着跑路线，这里再插一条
 *    回位姿会把路线那一步的臂动作盖掉（后写者胜）。要回上电默认位姿就自己写一条
 *    组动作积木排在路线里。 */
static void bt_cmd_mode(const char *pcOn)
{
    uint8_t ucOk = 0u;
    float   fOn  = bt_parse_float(pcOn, &ucOk);

    if (ucOk == 0u) { bt_err("BADVAL"); return; }

    if (fOn >= 0.5f)
    {
        /* 入场条件（两者取或）:
         *   ① 脚本正停在握手等待阶段 / 让位积木里（bt_hs_wait=1）—— 原逻辑, 结构性保证
         *      「点动与路线抢不了舵机」;
         *   ② **脚本根本没在跑**（vScriptRun 已返回、任务已停车）—— 那时路线不可能动舵机,
         *      让操作员进编辑模式点动是安全的, 而这正是台架调臂最常用的姿态。
         *      少了这一条就会「车明明停着、人围着调臂, 却只回 NOHANDSHAKE」（现场踩过）。
         * 路线正在跑**且不在让位窗里** → 仍然拒绝: 那一刻臂是脚本的。
         * ⚠️ 配套: vScriptRun 一进函数就 vSetBtEditMode(0) 收回所有权, 所以「脚本停了
         *    进编辑模式」不会被随后开跑的路线继承（否则保证就漏了）。
         * ⚠️ 2026-10: 判据加「请求槽非空」—— 菜单投递请求后、脚本任务把 running 置 1
         *    之前有个 ≤50ms 的窗口, 只看 running 会让 PC 恰好在这窗口进编辑模式、
         *    随后路线照样开跑（点动与脚本抢舵机）。 */
        if ((ucGetBtHsWait() == 0u) &&
            ((ucGetScriptRunning() != 0u) || (ucGetScriptReqPending() != 0u)))
        {
            bt_err("NOHANDSHAKE");
            return;
        }
        vSetBtEditMode(1u);
        bt_sendf("$B,MODE,1");
        bt_ok();
    }
    else
    {
        vSetBtEditMode(0u);
        bt_sendf("$B,MODE,0");
        bt_ok();

        /* 2026-10 加: 退出时正处**让位窗**（脚本停在握手阶段 1 等SYNC_HS_1）→ 立刻踢
         * Pi 重启客户端, 不等 ucRpiHandshake 里那个 6s 重试闸 —— 否则"退出之后恢复
         * 到和 Pi 连接的行为"要干等 6s + Pi 重启 20s+, 体感就是"退出了却没恢复,
         * 得自己再点一次握手"。Pi 只在启动阶段发 SYNC_HS_1, 这一脚就是让它重发。
         * ⚠️ 带内部 500ms 让出式延时（"r" 双发间隔）, 蓝牙任务可承受;
         *    Pi 没上电时两帧白发, 无害。 */
        if (ucGetBtHsWait() != 0u)
        {
            vRpiKickPiClient();
        }
    }
}

/* ---- HS ----
 * 投一次「握手让出」（SCRIPT_REQ_HANDSHAKE —— 与 OLED 第 3 页「握手让出」同一入口）:
 * 脚本任务在 ≤50ms 内取走, 正在跑的路线在**步骤边界**中止, ucRpiHandshake 落回
 * WAITING 进阶段 1 → bt_hs_wait 拉高 → PC 的 `$B,MODE,1` 就能进来。
 * 给「机械臂」Tab 的「连接 → 自动让位」用（2026-10 加）: Pi 在线时脚本多半在跑,
 * 不让位就 NOHANDSHAKE; 有了这条, PC 一条指令就能开出编辑窗, 不用去 OLED 上点。
 * ⚠️ 只投递不等待: 让位真正生效要等路线当前这一步跑完（大多数 <1s; 巡线那种长
 *    步骤有自己的闭环中止检查, 也快）。PC 侧 MODE,1 拿到 NOHANDSHAKE 就重试。 */
static void bt_cmd_hs(void)
{
    if (vScriptPostReq(SCRIPT_REQ_HANDSHAKE, 0u) == 0u)
    {
        bt_err("BUSY");               /* 请求槽被占（上一条还没被脚本任务取走）: 重发即可 */
        return;
    }
    bt_ok();
}

/* ---- JOG,<0..4>,<angle>[,<ms>] ---- */
static void bt_cmd_jog(const char *pcIdx, const char *pcAngle, const char *pcMs)
{
    stArmJointTdf astJoint[ARM_GROUP_NUM];
    uint8_t       ucJnum;
    uint8_t       ucOk = 0u;
    float         fIdx;
    float         fAngle;
    float         fMs = (float)BT_JOG_DEFAULT_MS;
    uint8_t       ucIdx;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    fIdx = bt_parse_float(pcIdx, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    fAngle = bt_parse_float(pcAngle, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    if ((pcMs != NULL) && (pcMs[0] != '\0'))
    {
        fMs = bt_parse_float(pcMs, &ucOk);
        if (ucOk == 0u) { bt_err("BADVAL"); return; }
        /* ms=0 是**跟手模式**（见下）, 不能被最小时长抬走; 负数按 0 处理。 */
        if (fMs < 0.0f) { fMs = 0.0f; }
        else if ((fMs > 0.0f) && (fMs < (float)BT_JOG_MIN_MS)) { fMs = (float)BT_JOG_MIN_MS; }
        if (fMs > (float)BT_JOG_MAX_MS) { fMs = (float)BT_JOG_MAX_MS; }
    }

    ucJnum = ucArmFillJointTable(astJoint, (uint8_t)ARM_GROUP_NUM);
    if (ucJnum == 0u) { bt_err("NOJOINT"); return; }

    if (fIdx < 0.0f) { ucIdx = (uint8_t)0u; }
    else             { ucIdx = (uint8_t)fIdx; }
    if ((ucIdx >= ucJnum) || (fIdx != (float)ucIdx)) { bt_err("NOJOINT"); return; }

    /* 范围钳位由固件把关：宁可回 ERR 也绝不把舵机写到量程外 */
    if ((fAngle < astJoint[ucIdx].fMin) || (fAngle > astJoint[ucIdx].fMax))
    {
        char acLo[16];
        char acHi[16];
        bt_fmt_f2(acLo, (int)sizeof(acLo), astJoint[ucIdx].fMin);
        bt_fmt_f2(acHi, (int)sizeof(acHi), astJoint[ucIdx].fMax);
        bt_sendf("$B,ERR,RANGE,%s,%s", acLo, acHi);
        return;
    }

    vArmSeqAbort();     /* 手动了就掐掉后台的臂序列, 免得两边抢舵机 */

    /* ms=0 = **跟手模式**（机械臂 Tab 拖滑杆时用）: 走**恒速趋近**, 不是 S 曲线。
     *
     * 为什么不能用 S 曲线: vServoSetTargetValueTimed 每次都把曲线的 t 打回 0, 而五次
     * 赫米特在 t≈0 附近速度近乎 0 —— PC 每 ~50ms 推一个新目标, 等于每次都让关节从缓入段
     * 重新爬, 于是「拖不跟手」（真正的瓶颈; 链路只要 4~6ms）。恒速趋近**不重启任何曲线**,
     * 是一条干净的一阶跟随, 手指移动慢于速度上限时就是跟手的。
     *
     * 速度由驱动里的**总速度闸**（ARM_SERVO_MAX_SPEED_DPS）管, 所以这里的「跟手」=
     * 「以设定转速跟到滑杆位置」, 不会瞬跳砸齿轮 —— 早期版本曾用零时长旁路直接写目标,
     * 结果是舵机以自身最高速甩过去, 太快。 */
    if (fMs <= 0.0f)
    {
        vServoSetTargetValueSlew(astJoint[ucIdx].emDev, fAngle);
    }
    else
    {
        vServoSetTargetValueTimed(astJoint[ucIdx].emDev, fAngle, (uint32_t)fMs);
    }

    {
        char acAngle[16];
        bt_fmt_f2(acAngle, (int)sizeof(acAngle), fAngle);
        bt_sendf("$B,JOG,%u,%s", (unsigned)ucIdx, acAngle);
    }
    bt_ok();
}

/* ---- GRIP,<kai_he>,<degree> ---- */
static void bt_cmd_grip(const char *pcKaiHe, const char *pcDegree)
{
    uint8_t ucOk = 0u;
    float   fKai;
    float   fDeg;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    fKai = bt_parse_float(pcKaiHe, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    fDeg = bt_parse_float(pcDegree, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }

    if (fDeg < 0.0f)   { fDeg = 0.0f; }
    if (fDeg > 100.0f) { fDeg = 100.0f; }

    /* NoWait 版：拖滑块时不能把 vBtTask 卡住 ~300ms */
    vArmSeqAbort();     /* 手动开合夹爪 = 新的臂指令, 掐掉后台序列 */
    vArmSetZhuaShouNoWait((fKai >= 0.5f) ? 1u : 0u, (uint8_t)(fDeg + 0.5f));

    bt_sendf("$B,GRIP,%u,%u", (fKai >= 0.5f) ? 1u : 0u, (unsigned)((uint8_t)(fDeg + 0.5f)));
    bt_ok();
}

/* ---- ARM,<a0>,<a1>,<a2>,<a3>,<a4>,<kai>,<deg> ---- */
/* 播放**一个任意位姿**：5 轴同时启动 S 曲线 + 夹爪, 立即返回。
 * 机械臂 Tab 的「单条播放」与 PC 编排的整序列播放都用它 —— 一条指令 5 轴同时启动,
 * 比 5 条 $B,JOG 少 4 次往返, 也不会串行启动（各轴起始时刻差几毫秒, 肉眼看是同步的）。
 * 不阻塞: 给 vBtTask 不引入长等待, 由 PC 用现成的 $B,POS 轮询 done。 */
static void bt_cmd_arm(char *const *apc, uint8_t ucN)
{
    stArmJointTdf astJoint[ARM_GROUP_NUM];
    float         afAngle[ARM_GROUP_NUM];
    char          acVal[ARM_GROUP_NUM][16];
    uint8_t       ucJnum;
    uint8_t       ucOk = 0u;
    uint8_t       ucI;
    float         fKai;
    float         fDeg;
    uint8_t       ucKai;
    uint8_t       ucDeg;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    ucJnum = ucArmFillJointTable(astJoint, (uint8_t)ARM_GROUP_NUM);
    if (ucJnum == 0u) { bt_err("NOJOINT"); return; }

    /* 5 个角度 + 开合 + 程度 = 7 个参数 */
    if (ucN < (uint8_t)(1u + ARM_GROUP_NUM + 2u)) { bt_err("ARGS"); return; }

    for (ucI = 0u; ucI < ucJnum; ucI++)
    {
        afAngle[ucI] = bt_parse_float(apc[1u + ucI], &ucOk);
        if (ucOk == 0u) { bt_err("BADVAL"); return; }

        /* 范围钳位由固件把关: 宁可回 ERR 也绝不把舵机写到量程外 */
        if ((afAngle[ucI] < astJoint[ucI].fMin) || (afAngle[ucI] > astJoint[ucI].fMax))
        {
            char acLo[16];
            char acHi[16];
            bt_fmt_f2(acLo, (int)sizeof(acLo), astJoint[ucI].fMin);
            bt_fmt_f2(acHi, (int)sizeof(acHi), astJoint[ucI].fMax);
            bt_sendf("$B,ERR,RANGE,%s,%s", acLo, acHi);
            return;
        }
        bt_fmt_f2(acVal[ucI], (int)sizeof(acVal[0]), afAngle[ucI]);
    }

    fKai = bt_parse_float(apc[1u + ucJnum], &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    fDeg = bt_parse_float(apc[2u + ucJnum], &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }

    if (fDeg < 0.0f)   { fDeg = 0.0f; }
    if (fDeg > 100.0f) { fDeg = 100.0f; }
    ucKai = (fKai >= 0.5f) ? 1u : 0u;
    ucDeg = (uint8_t)(fDeg + 0.5f);

    vArmSeqAbort();
    vArmGroupMoveNoWait(afAngle);
    vArmSetZhuaShouNoWait(ucKai, ucDeg);

    bt_sendf("$B,ARM,%s,%s,%s,%s,%s,%u,%u",
             acVal[0], acVal[1], acVal[2], acVal[3], acVal[4],
             (unsigned)ucKai, (unsigned)ucDeg);
    bt_ok();
}

/* ---- PLAY,<id> ---- */
/* 播放**已编译的动作预设整段**（含表内 STEP_DELAY）, 阻塞到走完。
 *
 * 它直接调 vArmPlayWait —— 与路线里的 STEP_ARM_PLAY_WAIT **同一个函数**, 所以
 * 「台架上试跑看到的」与「比赛跑路线时发生的」是逐字节同一条路径。这是这个指令
 * 存在的唯一理由（PC 自己编排也能动臂, 但那验证的是 PC 的数据, 不是固件里编译进去的那份）。
 *
 * ⚠️ 它**刻意不跟随**预设自己的「播放方式」（用的是 vArmPlayWait 不是 ucArmPlayByMode）:
 *    这个指令的语义就是「播给我看、播完再回 ACK」, 台架上标着「启动即返回」的动作也要
 *    完整播一遍才看得出录得对不对; 而且 PC 侧的 PLAY_TIMEOUT(15s) 正是按阻塞设计的。
 *    跟随模式的是排爆抓取/反恐任务里那两处调用 —— 别把这里也"顺手"改过去。
 *
 * 阻塞期间 vBtTask 收不到新指令, 所以:
 *   · PC 端 send_command 超时要给足（15s）, 且不要同时轮询 $B,POS（它也得不到应答）;
 *   · vArmPlayWait 内部逐条喂编辑模式看门狗, 免得 8s 空闲超时把编辑模式踢掉。 */
static void bt_cmd_play(const char *pcId)
{
    uint8_t ucOk = 0u;
    float   fId;
    uint8_t ucId;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    fId = bt_parse_float(pcId, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    if ((fId < 0.0f) || (fId > 255.0f)) { bt_err("BADVAL"); return; }

    ucId = (uint8_t)fId;
    if (fId != (float)ucId) { bt_err("BADVAL"); return; }   /* 与 bt_cmd_jog 同判据: 2.5 不许静默截成 2 */
    if (ucArmActionExists((emArmActionIdTdf)ucId) == 0u) { bt_err("NOACTION"); return; }

    /* 先回显再播: PC 立刻知道指令已被受理, 然后在同一个响应里等 $B,OK（15s） */
    bt_sendf("$B,PLAY,%u", (unsigned)ucId);
    vArmPlayWait((emArmActionIdTdf)ucId);
    bt_ok();
}

/* ---- CPLAY,<id> / CSTOP（底盘动作组预览, 2026-10 加） ---- */
/* 与机械臂 $B,PLAY 的关键差别: **非阻塞**。播放丢给 vChassisActionTask（见
 * freertos.c）, 本任务立刻返回 —— 车在跑的时候 $B,CSTOP 必须随时能进来;
 * 机械臂阻塞无所谓（$B,PLAY 期间臂没有"撞人"风险）, 底盘不行。
 * PC 端发完 CPLAY 后轮询 $B,GET,ch_busy 判断播完（1=在播）。
 *
 * ⚠️ **刻意不查 ucGetScriptRunning()**: 路线表尾带 STEP_SCRIPT_RESET 自动循环 /
 *    一上电就跑活动预设的机器, 这个标志从上电起几乎恒为 1（停在「等握手/按主
 *    按键」时也是 1）—— 拿它当闸门 = 台架上永远 SCRIPTRUN。预览的真正安全链在
 *    bt_edit_active()（只有「让位窗开着」或「脚本真没跑」两种状态放行）+ 播放器
 *    逐拍的让位判据（app_chassis_action.c ①: 编辑模式被清 或 让位窗关闭且路线
 *    活着 → ≤20ms 自停 —— 覆盖 Pi 重连交还 / MAIN 恢复路线 / 新路线开机收回
 *    编辑模式三种「路线要动了」的时刻）。 */
static void bt_cmd_cplay(const char *pcId)
{
    uint8_t ucOk = 0u;
    float   fId;
    uint8_t ucId;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    fId = bt_parse_float(pcId, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    if ((fId < 0.0f) || (fId > 255.0f)) { bt_err("BADVAL"); return; }

    ucId = (uint8_t)fId;
    if (fId != (float)ucId) { bt_err("BADVAL"); return; }   /* 与 bt_cmd_jog 同判据 */
    if (ucChActPreviewExists(ucId) == 0u) { bt_err("NOACTION"); return; }

    /* 播放器忙 → 拒绝（第二条速度流插不进来; 具体的蜂鸣/日志在任务侧） */
    if (ucGetChActBusy() != 0u) { bt_err("BUSY"); return; }

    /* 先排掉可能残留的 STOP 请求再投: 空闲期的 CSTOP 由预览任务排涝, 但投递与
     * 任务取走之间仍有竞窗 —— 这里再排一次, 保证不会一开播就被旧请求掐停。 */
    (void)ucTakeChActStopReq();
    vSetChActPreviewReq(ucId);
    bt_sendf("$B,CPLAY,%u", (unsigned)ucId);
    bt_ok();
}

/* ---- CPUSH,<id>,<count>（在线推送草稿: 开窗; 2026-10 加） ----
 * 控制台在预览前把**当前编辑的这一份**推过来: 本命令开窗 → count 条 $B,CIT 行 →
 * 收满自动武装 → 随后的 $B,CPLAY,<id> 命中草稿播出（Resolve 草稿优先）。
 * 于是「在线编辑的立即能预览」, 不用每次 Keil 重编译; 而路线/比赛仍只读 FLASH
 * 编译表（铁律见 app_chassis_action.h）—— 草稿断电即失, 绝不渗进比赛。 */
static void bt_cmd_cpush(const char *pcId, const char *pcCount)
{
    uint8_t ucOk = 0u;
    float   fId, fN;
    uint8_t ucId, ucN;

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    fId = bt_parse_float(pcId, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    if ((fId < 1.0f) || (fId > 255.0f)) { bt_err("BADVAL"); return; }
    ucId = (uint8_t)fId;
    if (fId != (float)ucId) { bt_err("BADVAL"); return; }

    fN = bt_parse_float(pcCount, &ucOk);
    if (ucOk == 0u) { bt_err("BADVAL"); return; }
    if ((fN < 1.0f) || (fN > (float)CH_ACT_ITEMS_MAX)) { bt_err("BADVAL"); return; }
    ucN = (uint8_t)fN;
    if (fN != (float)ucN) { bt_err("BADVAL"); return; }

    /* 播放器忙 → 拒收（草稿不许在播放途中被改写, 见 app_chassis_action.c） */
    if (ucChActTryBegin(ucId, ucN) == 0u) { bt_err("BUSY"); return; }

    bt_sendf("$B,CPW,%u", (unsigned)ucN);
    bt_ok();
}

/* ---- CIT,<kind>,<start>,<dur>,<wx>,<wy>,<speed>,<accel>,<delta>,<armId>,<hold>,<abs> ----
 * 草稿一行。字段顺序 = stChActItemTdf 的声明顺序（wx/wy 单位 mm, 浮点;
 * kind: 0平移 1转弯 2臂动作 3等臂 4支点（与 CH_ACT_KIND_* 一一对应）。任一字段越界 → CITBAD（窗口不关,
 * PC 端会整批重推; 没 CPUSH 就来 CIT → NOWIN）。 */
static void bt_cmd_cit(char *const *apc, uint8_t ucN)
{
    stChActItemTdf stItem;
    float afV[11];
    uint8_t ucOk = 0u;
    uint8_t ucI;

    if (ucN < 12u) { bt_err("ARGS"); return; }

    for (ucI = 0u; ucI < 11u; ucI++)
    {
        afV[ucI] = bt_parse_float(apc[ucI + 1u], &ucOk);
        if (ucOk == 0u) { bt_err("CITBAD"); return; }
        /* 超范围/NaN 先挡: 对 1e30 这类值做 (int) 转换按 C 标准是 UB, 而下面的
         * 整型判据会先做这个转换（ARM VCVT 实际饱和成 INT_MAX 也能被拒收, 这里
         * 前置只是把 UB 从路径上拿掉; 2^24 内 float 能精确表达整数）。 */
        if ((afV[ucI] != afV[ucI]) || (afV[ucI] > 8388607.0f) || (afV[ucI] < -8388607.0f))
        {
            bt_err("CITBAD");
            return;
        }
    }

    /* 整数字段的「2.5 不许静默截成 2」判据（同 bt_cmd_jog）+ 范围: 越界拒绝整行,
     * 宁可 PC 重推也不收一条会被固件静默回绕的脏数据。
     * kind: 0平移 1转弯 2臂动作 3等臂 4支点（与 CH_ACT_KIND_* 一一对应）。 */
    if ((afV[0] != (float)(int)afV[0]) || (afV[0] < 0.0f) || (afV[0] > 4.0f) ||
        (afV[1] != (float)(int)afV[1]) || (afV[1] < 0.0f) || (afV[1] > 65535.0f) ||
        (afV[2] != (float)(int)afV[2]) || (afV[2] < 0.0f) || (afV[2] > 65535.0f) ||
        (afV[3] < -10000.0f) || (afV[3] > 10000.0f) ||
        (afV[4] < -10000.0f) || (afV[4] > 10000.0f) ||
        (afV[5] != (float)(int)afV[5]) || (afV[5] < -2000.0f) || (afV[5] > 2000.0f) ||
        (afV[6] != (float)(int)afV[6]) || (afV[6] < 0.0f) || (afV[6] > 255.0f) ||
        (afV[7] != (float)(int)afV[7]) || (afV[7] < -360.0f) || (afV[7] > 360.0f) ||
        (afV[8] != (float)(int)afV[8]) || (afV[8] < 0.0f) || (afV[8] > 255.0f) ||
        (afV[9] != (float)(int)afV[9]) || (afV[9] < 0.0f) || (afV[9] > 1.0f) ||
        (afV[10] != (float)(int)afV[10]) || (afV[10] < 0.0f) || (afV[10] > 1.0f))
    {
        bt_err("CITBAD");
        return;
    }

    stItem.ucKind  = (uint8_t)afV[0];
    stItem.usStart = (uint16_t)afV[1];
    stItem.usDur   = (uint16_t)afV[2];
    stItem.fWx     = afV[3];
    stItem.fWy     = afV[4];
    stItem.nSpeed  = (int16_t)afV[5];
    stItem.ucAccel = (uint8_t)afV[6];
    stItem.nDelta  = (int16_t)afV[7];
    stItem.ucArmId = (uint8_t)afV[8];
    stItem.ucHold  = (uint8_t)afV[9];
    stItem.ucAbs   = (uint8_t)afV[10];

    /* 没开窗（NOWIN）/ 播放中（BUSY）都在这里被拒 —— bt 层不区分, PC 端整批重推 */
    if (ucChActTryStore(&stItem) == 0u) { bt_err("NOWIN"); return; }

    bt_ok();
}

/* 停底盘动作组预览: 置停止位（播放器下一拍, ≤20ms 生效）+ 掐臂序列（预览里
 * 已投出的臂动作一并停 —— 按了停臂不许事后自己动, 同 vArmSeqAbort 的纪律）。
 * **不设编辑模式门槛**: 停车指令永远合法（预览的入场有 NOTEDIT 门, 刹车没有）。 */
static void bt_cmd_cstop(void)
{
    vSetChActStopReq(1u);
    vArmSeqAbort();
    bt_sendf("$B,CSTOP");
    bt_ok();
}

/* ---- CHR0（航向关系重置, 2026-10 加） ----
 * 把世界航向**累加器**对齐到当前 IMU 工作航向 —— 重建「上电那一刻」的账面/物理
 * 关系。台架常态: 上电 → 手动把车搬回出发区（物理朝向已经转过任意角度）→ 此时
 * 账面还是 0°, 相对转弯的目标=账面±delta 会转错、保持层会拿 0° 跟物理朝向打架。
 * 摆好车按一下, 当前车头方向就重新成为 0° 基准, 相对转弯/保持/世界系平移查表
 * 全部自洽（查表按 ±45° 取整, 摆放残差天然被吸收, 同上电口径）。
 * ⚠️ 不动车、不碰电机; **不设编辑模式门槛**（它不产生运动, 同 CSTOP 的
 *    "刹车永远合法"逻辑）。但运行中重置会改写后续世界系动作的基准 —— PC 端
 *    按钮的语义就是「摆好车、确认朝向、再按」。 */
static void bt_cmd_chr0(void)
{
    char acYaw[12];

    bt_fmt_f2(acYaw, (int)sizeof(acYaw), fGetAttitudeYaw());
    vSetWorldHeading(fGetAttitudeYaw());
    bt_sendf("$B,CHR0,%s", acYaw);
    bt_ok();
}

/* ---- POS ---- */
/* 回读 5 个组关节 + 夹爪的当前角度与「全部到位」标志。
 * ⚠️ ucServoIsAllSmoothStepDone() 是遍历全部 6 轴（含夹爪），所以别拿它单独判
 *    「夹爪到位」；这里用它判「整体停稳」，夹爪的当前值另行回读。
 *    它现在是**真的"到位"**（被速度闸削速后的恒速收尾也算没到, 见 servo_device.c）——
 *    控制台的「播放整个序列」正是靠这一位决定下一条什么时候发, 所以它必须诚实。 */
static void bt_cmd_pos(void)
{
    stArmJointTdf astJoint[ARM_GROUP_NUM];
    uint8_t       ucJnum;
    uint8_t       ucJ;
    char          acVal[ARM_GROUP_NUM + 1u][16];   /* 末位留给夹爪 */

    if (bt_edit_active() == 0u) { bt_err("NOTEDIT"); return; }

    ucJnum = ucArmFillJointTable(astJoint, (uint8_t)ARM_GROUP_NUM);
    if (ucJnum == 0u) { bt_err("NOJOINT"); return; }

    for (ucJ = 0u; ucJ < ucJnum; ucJ++)
    {
        bt_fmt_f2(acVal[ucJ], (int)sizeof(acVal[0]),
                  c_pstGetServoDeviceParam(astJoint[ucJ].emDev)->stRunningParam.fCurrentValue);
    }
    bt_fmt_f2(acVal[ARM_GROUP_NUM], (int)sizeof(acVal[0]),
              c_pstGetServoDeviceParam(ARM_GRIPPER)->stRunningParam.fCurrentValue);

    bt_sendf("$B,POS,%s,%s,%s,%s,%s,%s,%u",
             acVal[0], acVal[1], acVal[2], acVal[3], acVal[4], acVal[5],
             (unsigned)(ucServoIsAllSmoothStepDone() ? 1u : 0u));
    bt_ok();
}

/* ---- 分发 ---- */
static void bt_handle_line(char *pcLine)
{
    char   *apcTok[BT_MAX_TOK];
    uint8_t ucN;

    /* 非 "$B," 开头的行 = 垃圾行（PC 侧粘贴错误、非本协议设备挂上来、行溢出后的残留）:
     * 回一条 SYNC 让 PC 重发, 而不是静默丢弃。限速 1 条/秒防刷屏。
     * 注意: 这里不再承担「DMA 回绕掉头」——取帧层已按**新增字节**交付, 帧头不会再被切掉
     * （见 app_uart.h 的契约; 2026-09 修）。 */
    if (strncmp(pcLine, "$B,", 3u) != 0)
    {
        uint32_t ulNow = xTaskGetTickCount();
        if ((ulNow - s_ulSyncErrTick) > (TickType_t)pdMS_TO_TICKS(1000u))
        {
            s_ulSyncErrTick = ulNow;
            bt_err("SYNC");
        }
        return;
    }

    vSetBtCmdTick(xTaskGetTickCount());   /* 给握手泵的看门狗喂狗 */

    ucN = bt_split(pcLine + 3, apcTok, BT_MAX_TOK);
    if (ucN == 0u) { bt_err("UNKNOWN"); return; }

    if (strcmp(apcTok[0], "PING") == 0)      { bt_cmd_ping(); return; }
    if (strcmp(apcTok[0], "LIST") == 0)      { bt_cmd_list(); return; }
    if (strcmp(apcTok[0], "POS") == 0)       { bt_cmd_pos();  return; }

    if (strcmp(apcTok[0], "GET") == 0)
    {
        if (ucN < 2u) { bt_err("ARGS"); return; }
        bt_cmd_get(apcTok[1]);
        return;
    }
    if (strcmp(apcTok[0], "SET") == 0)
    {
        if (ucN < 3u) { bt_err("ARGS"); return; }
        bt_cmd_set(apcTok[1], apcTok[2]);
        return;
    }
    if (strcmp(apcTok[0], "MODE") == 0)
    {
        if (ucN < 2u) { bt_err("ARGS"); return; }
        bt_cmd_mode(apcTok[1]);         /* 多余的 token 直接忽略（老客户端会多发一个） */
        return;
    }
    if (strcmp(apcTok[0], "HS") == 0)    { bt_cmd_hs(); return; }
    if (strcmp(apcTok[0], "JOG") == 0)
    {
        if (ucN < 3u) { bt_err("ARGS"); return; }
        bt_cmd_jog(apcTok[1], apcTok[2], (ucN >= 4u) ? apcTok[3] : NULL);
        return;
    }
    if (strcmp(apcTok[0], "GRIP") == 0)
    {
        if (ucN < 3u) { bt_err("ARGS"); return; }
        bt_cmd_grip(apcTok[1], apcTok[2]);
        return;
    }
    if (strcmp(apcTok[0], "ARM") == 0)
    {
        bt_cmd_arm(apcTok, ucN);        /* 参数个数在函数内校验（5 轴 + 开合 + 程度） */
        return;
    }
    if (strcmp(apcTok[0], "PLAY") == 0)
    {
        if (ucN < 2u) { bt_err("ARGS"); return; }
        bt_cmd_play(apcTok[1]);
        return;
    }
    if (strcmp(apcTok[0], "CPLAY") == 0)
    {
        if (ucN < 2u) { bt_err("ARGS"); return; }
        bt_cmd_cplay(apcTok[1]);
        return;
    }
    if (strcmp(apcTok[0], "CPUSH") == 0)
    {
        if (ucN < 3u) { bt_err("ARGS"); return; }
        bt_cmd_cpush(apcTok[1], apcTok[2]);
        return;
    }
    if (strcmp(apcTok[0], "CIT") == 0)
    {
        bt_cmd_cit(apcTok, ucN);        /* 字段个数在函数内校验（11 字段 + 命令字） */
        return;
    }
    if (strcmp(apcTok[0], "CSTOP") == 0)    { bt_cmd_cstop(); return; }
    if (strcmp(apcTok[0], "CHR0") == 0)     { bt_cmd_chr0(); return; }

    bt_err("UNKNOWN");
}

/* 把一帧字节喂进行累积器，凑满一行就处理一行。
 * 必须有这一层：DMA+IDLE 给的是「一帧」，BT 链路可能把一行拆成几帧、也可能几行并一帧。 */
static void bt_feed(const uint8_t *pucData, uint16_t usLen)
{
    uint16_t usI;

    for (usI = 0u; usI < usLen; usI++)
    {
        char c = (char)pucData[usI];

        if ((c == '\n') || (c == '\r'))
        {
            if (s_ucLineOverflow != 0u)
            {
                /* 超长行到此结束: 整行丢掉, 回到正常态（这才是注释里说的"丢弃整行"） */
                s_ucLineOverflow = 0u;
                s_usLineLen      = 0u;
                continue;
            }
            if (s_usLineLen > 0u)
            {
                s_acLine[s_usLineLen] = '\0';
                bt_handle_line(s_acLine);
                s_usLineLen = 0u;
            }
            continue;
        }

        if (s_ucLineOverflow != 0u)
        {
            continue;               /* 溢出态里一个字节都不收, 等换行重新同步 */
        }

        if (s_usLineLen < (BT_RX_LINE_MAX - 1u))
        {
            s_acLine[s_usLineLen++] = c;
        }
        else
        {
            /* 溢出: 进"丢到下一个换行"态。
             * ⚠️ 老写法只把长度清零就完事 —— 于是这一行的**后半截**照样被收进缓冲、
             * 到下一个换行时当成一整行交给 bt_handle_line（大概率回一条 ERR,SYNC）。
             * 注释一直写着"丢弃整行", 实现只丢了前半截。 */
            s_ucLineOverflow = 1u;
            s_usLineLen      = 0u;
        }
    }
}

/* ==================== 任务 ==================== */

void vBtTask(void *argument)
{
    /* 本次新增字节暂存: 取满一圈, 任何一次 take 都放得下; static 不占任务栈 */
    static uint8_t aucRx[UART_DMA_BUF_SIZE];

    (void)argument;

    for (;;)
    {
        /* ⚠️ 取帧超时从 portMAX_DELAY 改成有界: 本任务原来只在有字节时才醒,
         * 于是"脚本空闲时进编辑模式"那条路径**没有任何一拍**能跑空闲看门狗
         * （让位窗那条由 vBtEditPump 逐轮喂）。超时那一拍顺手兜一次即可 ——
         * 500ms 的唤醒对 4 号优先级的本任务可忽略。 */
        uint16_t usLen = usUartDmaTakeCopy(UART_BT, aucRx, (uint16_t)sizeof(aucRx),
                                          pdMS_TO_TICKS(500));

        if (usLen > 0u)
        {
            bt_feed(aucRx, usLen);
        }
        else
        {
            bt_edit_idle_check();
        }
    }
}
