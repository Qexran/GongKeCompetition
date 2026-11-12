/**
  * @file       app_rpi.c
  * @brief      树莓派通信模块实现：帧收发 + 握手 + 任务指令 + 结果等待。
  *
  *             RX: vRpiTask 是 UART_RPI 的唯一消费者（DMA+IDLE 整帧），
  *                 解析后经 Beam 发布，其他任务不碰该串口信号量。
  *             TX: 互斥保护（脚本任务发指令, vRpiTask 自动应答也会发）。
  *
  *             握手为 Pi 主动（对齐 Pi 端 handler.py, 见 app_rpi.h 时序图）:
  *             vRpiTask 收到 "SYNC_HS_1" 自动回 ACK 0xCC —— 即使脚本没在等
  *             握手也应答, 保证 Pi 任何时候上电/重连都能完成握手。
  *             pro 项目握手失败的三个坑（裸字节应答被吞 / 只认魔数不校验 /
  *             单向确认）分别以标准帧 / 完整帧校验 / 静默确认窗规避。
  */

#include "app_rpi.h"
#include "app_uart.h"
#include "app_beam.h"
#include "app_bt.h"           /* vSetBtHsWait / vBtEditPump（阶段1 蓝牙机械臂编辑让位） */
#include "app_script.h"       /* ucScriptWaitKeyConfirm（单步确定的阶段边界等 MAIN; 与 app_script.h 的声明配套） */
#include "app_config.h"       /* UART_RPI, RPI_HANDSHAKE_CONFIRM_MS, COLOR_* 色球标定 */
#include "app_chassis.h"      /* vChassisMoveCameraLateral / vChassisMoveCameraForward（色球/小球对位平移） */
#include "app_pid.h"          /* vPidSpinTurn（小球对位 yaw 转正, 陀螺仪闭环） */
#include "app_arm.h"          /* vArmAimByPixels（标靶校准微调舵机6/3）
                               * ⚠️ 少了本行 vArmAimByPixels 会被**隐式声明**成 int(int,int),
                               *    而实参是 float —— 默认实参提升按 double 传, 函数里按 float 解,
                               *    舵机角度直接是垃圾值（AC5 只给一条 #223-D 警告, 不报错）。 */
#include "app_arm_action.h"   /* vArmPlayWait（排爆抓取: 按目标色播对应动作预设） */
#include "app_line.h"         /* vLineFollowConverge（巡线闭环横移） */
#include "emm_v5_device.h"    /* ucEmmV5Crc8（与 Pi protocol.py 同表, 帧校验复用） */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>           /* memcmp / strlen */
#include <stdio.h>            /* snprintf（[STM-DBG] 调试串, 只用整数格式化不链 %f） */
#include <math.h>             /* NAN（射靶粗残差的"已打"哨兵; armcc math.h 两分支都有定义） */

/* RPI 发送互斥（vAppRpiInit 创建; 判空防御初始化前误调用） */
static SemaphoreHandle_t s_xRpiTxMutex = NULL;

/* 最近一次收到 Pi 有效帧的时刻（心跳超时判定用） */
static TickType_t s_xLastRpiFrameTick = 0;

/* 最近一次「RX DMA 兜底重装」的时刻（限速用, 见 vRpiTask 的掉线自愈） */
static TickType_t s_xRxRearmTick = 0;

/* ==================== 待发任务单槽（2026-09 加） ====================
 * 链路不是 OK 时任务不再白丢: 挂进这个**单槽**等下一次链路成功再发。
 * 后写者胜、不排队 —— 目标是「等待期间单片机只推送一个任务, 树莓派端保持只做一个任务」;
 * 排队会让链路恢复那一刻 Pi 收到一串陈旧任务, 而其中大部分早就没意义了。
 *
 * ⚠️ **单写者 = 脚本任务**（它是唯一会阻塞在 ucRpiSendTaskCmdWait 里的任务）。
 *    vBtTask 只读遥测 → 单字直读、不加锁（同 app_beam.c::vScriptPostReq 的理由:
 *    Cortex-M4 上 32 位对齐读写在单核无 cache 下本来就是原子的）。
 * ⚠️ 发送动作由**等待者自己**做, 不由 vRpiTask 代发 —— 理由见 ucRpiSendTaskCmdWait 的头注释。 */
static uint32_t s_ulPendSeq    = 0u;   /* 投递代次: 每个「任务发起」入口都 +1 */
static uint8_t  s_ucPendTask   = 0u;   /* 当前挂起的任务码（0=无）; 遥测/诊断用 */
static uint32_t s_ulPendTick   = 0u;   /* 本次等待开始的 tick */
static uint32_t s_ulPendRetry  = 0u;   /* 上次发 "r" 对的 tick（重连往复计时） */
static uint16_t s_usLinkRetry  = 0u;   /* 累计发出的 "r" **帧**数（2 帧 = 一对） */
static uint16_t s_usPendDrop   = 0u;   /* 累计「挂起中被顶掉」次数 */
static uint32_t s_ulSysCmdTick = 0u;   /* 最近一次本地发 sd/rb 的 tick（自动重启的互锁） */

/* 顶掉槽里那个（如果有）= 「后写者胜」。**任何「任务发起」入口都调一次**。
 * ⚠️ vRpiSendDebug 与握手 ACK **不调** —— 它们不是"任务", 不能顶掉别人。
 * （2026-10-09: 三字段 + 丢计数进短临界区 —— 调用方不止脚本任务, 跨任务并发时
 *   读-改-写会互相交织。） */
static void vRpiPendSupersede(void)
{
    taskENTER_CRITICAL();
    if (s_ucPendTask != 0u)
    {
        s_usPendDrop++;                 /* 槽里那个还没发出去 = 它被顶掉了 */
    }
    s_ucPendTask = 0u;
    s_ulPendSeq++;
    taskEXIT_CRITICAL();
}

void vAppRpiInit(void)
{
    if (s_xRpiTxMutex == NULL)
    {
        s_xRpiTxMutex = xSemaphoreCreateMutex();
        configASSERT(s_xRpiTxMutex != NULL);
    }
    vSetRpiLinkState(RPI_LINK_WAITING);   /* 上电初始: 等待握手 */
    s_xLastRpiFrameTick = xTaskGetTickCount();
    /* ⚠️ s_ulSysCmdTick 是「最近一次本地发 sd/rb 的 tick」, 那条互锁（vRpiRestartPiClient
     *    开头）**必须**在开机时就是"早就过了抑制窗"。
     *    本来这里是 0 —— 于是上电后前 RPI_SYSCMD_SUPPRESS_MS(60s) 内
     *    `(now - 0) < 60s` 恒真, 自动重启 Pi 客户端的通道被静默关掉整整一分钟
     *    （而这一分钟恰是比赛开跑那段; 两条自愈路径全走它, 握手那条还会每 6s 打一条
     *    `HS kick: peer alive but no SYNC_HS_1` 的**假日志**）。
     *    回退一个抑制窗即可让判据开机即成立。本函数在调度器启动前调用,
     *    xTaskGetTickCount() 此时为 0, 相减后回绕成大数 —— 语义正确。 */
    s_ulSysCmdTick = (uint32_t)xTaskGetTickCount()
                     - (uint32_t)pdMS_TO_TICKS(RPI_SYSCMD_SUPPRESS_MS);
}

/* ==================== 发送 ==================== */

/* 发一帧: SOF|TYPE|LEN|DATA|CRC8|EOF（CRC8 覆盖 TYPE+LEN+DATA） */
void vRpiSendFrame(uint8_t ucType, const uint8_t *pucData, uint8_t ucLen)
{
    uint8_t aucFrame[RPI_FRAME_OVERHEAD + RPI_DATA_MAX];
    uint8_t ucI;

    if (ucLen > RPI_DATA_MAX) ucLen = RPI_DATA_MAX;

    aucFrame[0] = RPI_SOF;
    aucFrame[1] = ucType;
    aucFrame[2] = ucLen;
    for (ucI = 0; ucI < ucLen; ucI++)
    {
        aucFrame[3 + ucI] = pucData[ucI];
    }
    aucFrame[3 + ucLen] = ucEmmV5Crc8(&aucFrame[1], (uint8_t)(2 + ucLen));
    aucFrame[4 + ucLen] = RPI_EOF;

    if (s_xRpiTxMutex != NULL) { xSemaphoreTake(s_xRpiTxMutex, portMAX_DELAY); }
    vUartSendArray(UART_RPI, aucFrame, RPI_FRAME_OVERHEAD + ucLen);
    if (s_xRpiTxMutex != NULL) { xSemaphoreGive(s_xRpiTxMutex); }
}

/* 任务码 → ASCII 命令串（下标 = emRpiTaskTdf 枚举值, 0 占位） */
static const char *s_apcTaskCmd[] = {
    NULL,       /* 0 占位 */
    "1",        /* RPI_TASK_QR */
    "2",        /* RPI_TASK_LINE_START */
    "3",        /* RPI_TASK_LINE_STOP */
    "r",        /* RPI_TASK_RESTART */
    "end",      /* RPI_TASK_END */
    "4",        /* RPI_TASK_HOSTAGE */
    "5",        /* RPI_TASK_COLORBALL */
    "6",        /* RPI_TASK_TARGET */
    "sd",       /* RPI_TASK_SHUTDOWN: 系统关机 */
    "rb",       /* RPI_TASK_REBOOT:   系统重启 */
    "7",        /* RPI_TASK_AIM_START: 靶标流跟踪开始(独立线程, 回传 0x05 流帧 ~20Hz) */
    "8",        /* RPI_TASK_AIM_STOP:  靶标流跟踪停止 */
    "9",        /* RPI_TASK_PHOTO:     拍照(Pi 转告 OpenMV 存一张到它盘上的 photos/) */
    "A",        /* RPI_TASK_RECT:      矩形地板识别(阻塞, 回传 0x07 帧: 色球区板心) */
    "B",        /* RPI_TASK_WARM_QR:   预热扫码相机(只发不等, 无结果帧) */
    "C",        /* RPI_TASK_WARM_MAIN: 预热主摄(只发不等, 无结果帧) */
    "D",        /* RPI_TASK_QR_MAIN:   主摄二维码识别(阻塞, 回传 QR 帧, 格式同 "1") */
    "E",        /* RPI_TASK_QR_DIFF:     二维码识别(自研算法, 扫码副摄; 回传同一套 QR 帧) */
    "F",        /* RPI_TASK_QR_MAIN_DIFF: 二维码识别(自研算法, 主摄; 同上) */
    "G",        /* RPI_TASK_ROAD:      看路(主摄测地板弯道角点→交点+右切点, 阻塞, 回传 0x08 帧) */
    "x",        /* RPI_TASK_CANCEL:    取消当前任务(置Pi侧取消事件+停巡线流/靶标流, 只发不等) */
    "T",        /* RPI_TASK_LED_TOGGLE: OpenMV 补光临时开关(亮↔灭由 Pi 定, 只发不等) */
    "H",        /* RPI_TASK_ROAD_YAW: 看路校准(主摄拟合黄区直边图像方向角, 阻塞, 回传 0x09 帧; 只测不改) */
    "RM",       /* RPI_TASK_PI_MAIN_RESTART: 重启主程序(单帧直达, 不走 "r" 双发门; OLED「重启main」) */
    "RS",       /* RPI_TASK_PI_SVC_RESTART:  重启 gks 服务(systemctl restart; 重启中串口断属预期) */
    "CL",       /* RPI_TASK_PI_LOG_CLEAR:    清屏日志(GKS_Web 清屏按钮同一条通道) */
    "W",        /* RPI_TASK_WARM_PROF:       预热+指定档位("W,<cam>,<uses>,<prof>"; 只发不等) */
    "J",        /* RPI_TASK_BUCKET_START:    爆爆桶坐标流开始(独立线程, 回传 0x0B 流帧 ~20Hz; "J,<ms>"=预算) */
    "K",        /* RPI_TASK_BUCKET_STOP:     爆爆桶坐标流停止 */
    "M",        /* RPI_TASK_ROAD_REV:        看路(反向, 2026-10-31 加; Pi 侧整帧镜像后量黄区右边缘, 同 "G" 的 0x08 帧; "M,<ms>"=预算) —— 对齐 Pi handler.py::_COMMANDS 的 b"M"(_MS_COMMANDS/互斥元组/_FAIL_FRAME_SENDERS 三处同挂) */
    "Z",        /* RPI_TASK_HEALTH:          四路相机健康检查(2026-11 加, 阻塞, 回传 0x0D 帧; "Z,<ms>"=预算): 主摄/扫码副摄/CSI/OpenMV 依次确认能出帧, 查完主动释放占用 —— 对齐 Pi handler.py::_COMMANDS 的 b"Z"(四处同挂) */
};

/* ==================== 树莓派系统级操作（OLED 菜单用） ====================
 * ⚠️ 与 RPI_TASK_RESTART("r", 只重启 GKS 主程序) 是**两件事**: 这两个停/重启整台树莓派。
 * ⚠️ 只发一条指令帧、**不等回应**（对方正在关机, 也没有结果帧可回）:
 *    发完之后本端会看到心跳停 → 8s 后链路自动判 DISCONNECTED, 这是预期现象;
 *    要恢复得给 Pi 重新上电（关机）或等它起来后 gks.service 自启再重新握手（重启）。
 * ⚠️⚠️ **防误触是「连发两次」——但两次必须是两次操作**:
 *    Pi 端（handler.py::_require_double_send）要求同一条指令在窗口内到达两次才执行,
 *    所以菜单的写法是**每按一次键调一次本函数**（发一帧）, 第一次只算"武装";
 *    **别在一次按键里连发两帧** —— 那样等于没有确认（误触也会连发）。
 *    窗口在 Pi 上可调（config.py 的 SYSTEM_CMD_CONFIRM_S, 默认 10s, 执行后还有等长冷却）。
 * ⚠️ 单片机侧**当前没有任何调用点**（等 OLED 菜单接）——控制台「地图」Tab 的
 *    STEP_RPI_SEND_TASK 任务码下拉里也能选到这两项, 台架上可先用它验证 Pi 端。 */
void vRpiShutdownPi(void)
{
    /* 打点: 之后 RPI_SYSCMD_SUPPRESS_MS 内不让"等链路自动重启"去发 "r" ——
     * 两条危险指令走的是 Pi 侧同一道双发确认门, 凑单会把等着的关机变成立刻关机。 */
    s_ulSysCmdTick = xTaskGetTickCount();
    vRpiSendTaskCmd(RPI_TASK_SHUTDOWN);
}

void vRpiRebootPi(void)
{
    s_ulSysCmdTick = xTaskGetTickCount();   /* 同上（重启整机比关机更该守） */
    vRpiSendTaskCmd(RPI_TASK_REBOOT);
}

/* ==================== 拍照（OLED 菜单用） ====================
 * **链路是三跳**, 别找错地方: 本端只会把 "9" 发给 Pi（UART_RPI 帧协议）;
 * 真正拍照的是 **OpenMV** —— 由 Pi 的 handler.py::_cmd_photo 调
 * `openmv_serial.openmv_reader.send_command(b'P')` 把控制字节写给相机。
 *
 * ⚠️ **单片机没有直达相机的路**: 相机只有一路串口(UART3), 被 Pi 占着
 *    （Pi 的 config.py: `OPENMV_PORT = "/dev/ttyUSB_OpenMV"`, 一个 CH340 USB 转串口）。
 *    本端的 USART2(PD5/PD6, 板上丝印 VICE_GYRO) 现接 M0 副陀螺, 与相机无关 —— 别试图从这儿发。
 *
 * ⚠️⚠️ **只发不回**: 相机对 'P' **不应答**（原协议的 'G'/'S' 也不应答）,
 *    所以调用方**无法确认真的拍到了**（结果看 TAG=0x06 拍照回执帧 / OLED 闪一句）。
 *    唯一判据是回头去相机盘的 photos/ 里看。而且按 OpenMV 官方行为:
 *    **保存的图片要重启相机之后, PC 上才看得见** —— "拍完立刻刷新看不到"是正常的。
 *
 * ⚠️ 好处是它**不影响巡线**: 相机那边只写文件, 采样循环与 'G'/'S' 发送开关都归别人管,
 *    所以拍照与巡线可以同时进行（不像 "1"/"4"/"5"/"6" 要先用主摄）。
 * ⚠️ 非阻塞（同 vRpiShutdownPi）: 只是发一帧, 任务上下文（含 OLED 任务）可直接调。 */
void vRpiPhotoTake(void)
{
    vRpiSendTaskCmd(RPI_TASK_PHOTO);
}

/* OpenMV 补光亮度: 发 "L,<0~100>"（Pi 的 handler.py::_cmd_openmv_led 收, 转发相机）。
 * ⚠️ **刻意不走 s_apcTaskCmd 那套**: 那套的数值版（vRpiSendTaskCmdNumRaw）对 0 会
 *    退化成裸命令码, 而 0=灯灭对补光是合法值 —— 裸 "L" 会被 Pi 当未知指令丢掉。
 *    专用发送器保证任何值（含 0）都带 ",<n>" 后缀。
 * ⚠️ 只发不等（无结果帧, 同 vRpiPhotoTake）; 任务上下文（含 OLED 任务）可直接调。 */
void vRpiOpenmvLed(uint32_t ulLevel)
{
    char acBuf[12];                         /* "L,100" 最长 6 字节, 12 绰绰有余 */

    if (ulLevel > 100u)                     /* OpenMV 端 _param_apply 同样钳 0~100, 两边对齐 */
    {
        ulLevel = 100u;
    }
    (void)snprintf(acBuf, sizeof(acBuf), "L,%lu", (unsigned long)ulLevel);
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)strlen(acBuf));
}

/* OpenMV 补光**恢复成配置亮度**: 发**裸** "L"（Pi 的 _cmd_openmv_led 收到后重发
 * config.OPENMV_LED_BRIGHTNESS 那个值, 见 docs/串口协议.md 的 "L" 行）。
 *
 * ⚠️ 为什么本端不自己记住一个亮度、直接 `vRpiOpenmvLed(记住的值)`:
 *    「点亮时用多少」的唯一真相在 **Pi**（OLED 调参行 / 积木 / Pi Web 都在改它,
 *    本端**没有**任何一个影子副本）。发裸 "L" 就是"按你那边记的点亮" —— 幂等、不怕脱节。
 * ⚠️ 与 vRpiOpenmvLed(0) 配对用: 那个是"临时灭", 这个是"点回来"; 两者都不写 Pi 的 config。
 * ⚠️ 只发不等（无结果帧）, 任务上下文（含 OLED 任务）可直接调。 */
void vRpiOpenmvLedRestore(void)
{
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)"L", 1u);
}

/* ---- 真正的发送实现。⚠️ 不再对外: 外面只有下面两个公开入口 + 阻塞版 ---- */
static void vRpiSendTaskCmdRaw(emRpiTaskTdf emTask)
{
    uint8_t ucIdx = (uint8_t)emTask;

    if (ucIdx == 0 || ucIdx >= (uint8_t)(sizeof(s_apcTaskCmd) / sizeof(s_apcTaskCmd[0])))
    {
        return;                             /* 未知任务码: 不发 */
    }
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)s_apcTaskCmd[ucIdx],
                  (uint8_t)strlen(s_apcTaskCmd[ucIdx]));
}

/* 发 "<命令码>,<数值>" 的**唯一实现**。数值的含义由调用方定, 线上只差一张表:
 *   阻塞类指令 → **毫秒**（Pi 拿这个数当自己的超时, 留一点余量后）见 vRpiSendTaskCmdMsRaw;
 *   预热类指令 → **使用次数**（Pi 拿它放宽相机租约的空闲窗口）见 vRpiWarmCamera。
 * ⚠️ 两边共用同一个格式, 所以 Pi 端那个"数字字段"的含义**按命令码而不同** ——
 *    加新命令族时先想清楚它属于哪一类（handler.py 里那个局部变量因此叫 ms, 见其注释）。
 * ulNum == 0 → 退化成裸发, 与不带后缀**一字不差**。
 * ⚠️ 24 字节够: 最长命令串 3 字符 + 逗号 + 10 位十进制的数。 */
static void vRpiSendTaskCmdNumRaw(emRpiTaskTdf emTask, uint32_t ulNum)
{
    uint8_t ucIdx = (uint8_t)emTask;
    char    acBuf[24];

    if (ucIdx == 0 || ucIdx >= (uint8_t)(sizeof(s_apcTaskCmd) / sizeof(s_apcTaskCmd[0])))
    {
        return;                             /* 未知任务码: 不发 */
    }
    if (ulNum == 0u)
    {
        /* ⚠️ 必须调 **Raw** 版: 调公开版会去顶代次, 等于自己顶掉自己。 */
        vRpiSendTaskCmdRaw(emTask);         /* 不带后缀: 与旧行为一字不差 */
        return;
    }
    (void)snprintf(acBuf, sizeof(acBuf), "%s,%lu",
                   s_apcTaskCmd[ucIdx], (unsigned long)ulNum);
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)strlen(acBuf));
}

/* 发 "<命令码>,<n1>,<n2>" —— **三字段形状（2026-10-16 加）**, 目前只有"带 is_use_picam
 * 标志"的那三条用: "6,<ms>,<pi>" / "7,0,<pi>" / "C,<uses>,<pi>"。
 * ⚠️ **ulN2 == 0 时整段退化成两字段/裸发**（直接委托 vRpiSendTaskCmdNumRaw）——
 *    这是"只用主摄时线上与改造前一字不差"的全部实现: 老 Pi 收到带第三段的形状会
 *    判"形状不认识"而**响亮拒收**, 但老形状它照旧认得。
 * ⚠️ 第三个字段的含义对这三条是**同一个**: is_use_picam（1=优先 pi/CSI 摄像头,
 *    Pi 内部失败自动退回主摄）; 第二字段的含义仍按命令码而定（毫秒/使用次数/"7"的占位 0）。
 * ⚠️ 缓冲区要装 最长命令串(3) + 逗号 + 10 位 + 逗号 + 10 位 + NUL = 26, 取 32。 */
static void vRpiSendTaskCmd2NumRaw(emRpiTaskTdf emTask, uint32_t ulN1, uint32_t ulN2)
{
    uint8_t ucIdx = (uint8_t)emTask;
    char    acBuf[32];

    if (ulN2 == 0u)
    {
        vRpiSendTaskCmdNumRaw(emTask, ulN1);    /* 不带标志: 与改造前逐字节相同 */
        return;
    }
    if (ucIdx == 0 || ucIdx >= (uint8_t)(sizeof(s_apcTaskCmd) / sizeof(s_apcTaskCmd[0])))
    {
        return;                             /* 未知任务码: 不发（同上面几个的口径） */
    }
    (void)snprintf(acBuf, sizeof(acBuf), "%s,%lu,%lu",
                   s_apcTaskCmd[ucIdx], (unsigned long)ulN1, (unsigned long)ulN2);
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)strlen(acBuf));
}

/* 发 "<命令码>,<n1>,<n2>,<n3>" —— **四字段形状（2026-10-23 加）**, 目前只有档位预热
 * "W,<cam>,<uses>,<prof>" 用它（见 vRpiWarmOne）。
 * ⚠️ 与上面那条两段版同一条纪律: 老 Pi 收到这个形状会判"形状不认识"而**响亮拒收**，
 *    所以只有当用户**显式选了非默认档**时才发它 —— 默认档线上与改造前逐字节相同。
 * ⚠️ 缓冲区要装 最长命令串(3) + 3 个逗号 + 3×10 位 + NUL = 37, 取 40。 */
static void vRpiSendTaskCmd3NumRaw(emRpiTaskTdf emTask, uint32_t ulN1,
                                   uint32_t ulN2, uint32_t ulN3)
{
    uint8_t ucIdx = (uint8_t)emTask;
    char    acBuf[40];

    if (ucIdx == 0u || ucIdx >= (uint8_t)(sizeof(s_apcTaskCmd) / sizeof(s_apcTaskCmd[0])))
    {
        return;                             /* 未知任务码: 不发（同上面几个的口径） */
    }
    (void)snprintf(acBuf, sizeof(acBuf), "%s,%lu,%lu,%lu",
                   s_apcTaskCmd[ucIdx], (unsigned long)ulN1,
                   (unsigned long)ulN2, (unsigned long)ulN3);
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)strlen(acBuf));
}

static void vRpiSendTaskCmdMsRaw(emRpiTaskTdf emTask, uint32_t ulTimeoutMs)
{
    /* 发 "<命令码>,<毫秒>" —— Pi 拿**这个数**当自己的超时(留一点余量后)。
     * 于是「谁先放弃」不再靠两个仓库各记一个数字互相迁就: 路线里那一步的 I(0)
     * 就是唯一的时间权威, Pi 跟着走(Pi 侧那两个 QR_*_TIMEOUT 退化成没给时限时的
     * 兜底, 供网页就地扫码用)。详见 docs/串口协议.md §6。
     * ⚠️ 只有 Pi 端登记过能收时限的指令可以这么发; 其余指令多带一个逗号会被判为
     *    未知指令。ulTimeoutMs == 0 → 退化成 vRpiSendTaskCmd。 */
    vRpiSendTaskCmdNumRaw(emTask, ulTimeoutMs);
}

/* ---- 公开入口: 照发、**不等链路**（「只发不等」的那几条 + 收尾帧）----
 * 用它们的有: OLED 的拍照"9" / 关机"sd" / 重启"rb"、STEP_RPI_SEND_TASK、
 * STEP_RPI_WARM_CAM（经 vRpiWarmCamera, 见下）、
 * SCRIPT_REQ_TASK 里的 "C" 预热主摄、以及**收尾帧** LINE_STOP("3")/AIM_STOP("8")。
 * ⚠️ 先顶掉挂起的那个 —— 「以新的任务为准」: 此刻若脚本正挂在 ucRpiSendTaskCmdWait
 *    里等链路, 它会看到代次变了并返回 0（本步按失败走, 不响 —— 被顶掉=有新任务要来）。
 * ⚠️ 这几条**刻意不挂起**（用户确认过的取舍）: 挂起的话链路恢复后会在几分钟后突然
 *    开火 —— 几分钟前按的"拍照"、路线早就过了那一步却发出一条起巡线流、而 "C" 的
 *    预热窗口（后面那次 vArmPlayWait）也早过了。收尾帧更是**绝不能**被门控: 它们最
 *    需要发出去的时刻恰恰是链路不可靠时（起流失败的兜底路径）, 一挂起就会让 Pi 一直
 *    跑着 30Hz 流。 */
void vRpiSendTaskCmd(emRpiTaskTdf emTask)
{
    vRpiPendSupersede();
    vRpiSendTaskCmdRaw(emTask);
}

void vRpiSendTaskCmdMs(emRpiTaskTdf emTask, uint32_t ulTimeoutMs)
{
    vRpiPendSupersede();
    vRpiSendTaskCmdMsRaw(emTask, ulTimeoutMs);
}

/* 只发不等的**带 is_use_picam 标志**版（2026-10-16 加）。
 * 用途 = 射靶保持档的起流自愈: 它必须和"当初那条起流"用**同一台相机** ——
 * 否则 Pi 侧 12s 让流自己退之后, 这一次重发会把环悄悄切回主摄, 而瞄准列还是 CSI 那一把
 * （静默跑偏; 见 ucTargetShootCore 的"相机变了就换尺"）。 */
void vRpiSendTaskCmdFlag(emRpiTaskTdf emTask, uint32_t ulUsePicam)
{
    vRpiPendSupersede();
    if (ulUsePicam != 0u)
    {
        vRpiSendTaskCmd2NumRaw(emTask, 0u, 1u);     /* 第二字段占位 0（"7" 不用它） */
    }
    else
    {
        vRpiSendTaskCmdRaw(emTask);                 /* 与改造前逐字节相同 */
    }
}

/* ---- 预热摄像头（脚本积木 STEP_RPI_WARM_CAM 的原语）----
 * 相机 → 任务码: 识别主摄 "C" / 扫码副摄 "B"。两张都只开相机、不发结果帧。
 * ⚠️ **pi 摄像头（WARM_CAM_PI, 2026-10-16 加）复用 "C"** —— 它在 Pi 侧就是"识别主摄"
 *    的另一种实现（CSI 那颗）, 只多带一个 is_use_picam 字段 → "C,<uses>,1"
 *    （见 vRpiSendTaskCmd2NumRaw）。因此"预热哪台"与"随后那次识别用哪台"永远一致,
 *    也不会再多出一个任务码/下拉项。
 * ⚠️ **WARM_CAM_MAIN_PI（2026-10-18 加）= 一次预热两颗**: 背靠背发两条帧
 *    （"C,<uses>" + "C,<uses>,1"）。两条都是"只发不等", 不会在串口接收线程里互相阻塞;
 *    Pi 端两个持有器（USB 租约 / CSI 子进程）互不相干, 同时热着是设计内的事。
 *    用途见 app_rpi.h 的 emWarmCamTdf: 平移射靶「粗主/细 pi」前把 pi 提前热好。
 * ⚠️ 越界（手写 map.c / 老参数）一律**不发** —— 绝不猜一台相机去开。
 * ⚠️ **只发不等**（与 vRpiSendTaskCmd 同一套: 顶掉挂起的、不等链路）: 它只是开相机,
 *    链路恢复后迟到的预热窗口早过了（那时后面那次识别早已退回现开现扫）;
 *    而它又恰恰常常排在巡线/移动**之前**, 挂起等链路会让整个预热窗口落空。 */
static const emRpiTaskTdf c_aemWarmTask[4] = {
    RPI_TASK_WARM_MAIN,   /* WARM_CAM_MAIN: 主摄 /dev/camera_main */
    RPI_TASK_WARM_QR,     /* WARM_CAM_QR:   扫码副摄 /dev/camera_Qr */
    RPI_TASK_WARM_MAIN,   /* WARM_CAM_PI:   pi(CSI) —— 同一个 "C", 靠下面的标志区分 */
    RPI_TASK_WARM_MAIN,   /* WARM_CAM_MAIN_PI: 只作占位（本项走上面的双发分支, 不会查到这行） */
};
static const uint8_t c_aucWarmPi[4] = { 0u, 0u, 1u, 0u };

/* 发**一路**的预热帧（内部助手; ucCam 必须已校验在表内）
 * ucProfPlan = 每次使用各自的档位**打包计划**（见 app_rpi.h 的 WARM_PROF_*）:
 *   每**次使用**占 2 bit, 第 i 次在 bit 2i 上（0=默认 1=看路 2=人质）。
 *   · 0（全默认）或 PI（CSI 没这套档位）→ 走老路: 裸 "B"/"C" 或 "C,<uses>[,1]",
 *     与加档位之前**逐字节相同**;
 *   · 非 0 → 发 "W,<cam>,<uses>,<plan>"（四字段带相机码, Pi 端拆包见
 *     handler._dispatch_warm_w / _unpack_profiles）。
 *     这段只在用户**显式设了档位**时才出现 —— 老路线/老 map.c 条目（第三槽补 0）
 *     线上流量一点没变。 */
static void vRpiWarmOne(uint8_t ucCam, uint32_t ulUses, uint32_t ulProfPlan)
{
    /* ulUses = 这台相机接下来要被**几个任务**消费（脚本积木的 uses 参数）:
     *   ≤1 → 发**裸** "B"/"C" —— 与加这个参数之前一字不差, 线上流量都没变;
     *   ≥2 → 发 "C,2" 这种带次数后缀的形状, Pi 把相机租约的空闲窗口放宽到 N 倍
     *        （于是隔了十几秒的第二、第三个任务还能命中同一份租约, 不再白花 ~2s 重开）。
     *        次数用完后窗口自动回到 1 倍 —— 给少了只是退回老行为, 不会更糟。
     * ⚠️ pi 那档**必须把第二字段materialise 出来**（不能用"0 → 裸发"那条捷径）:
     *    发了标志就得发全三字段 → 用 "C,1,1"（1 = 窗口 1 倍, 同 "C" 的含义）。 */
    uint32_t ulUsesWire = (ulUses >= 2u) ? ulUses : 1u;

    vRpiPendSupersede();                    /* 与 vRpiSendTaskCmd 同口径 */
    if ((ulProfPlan != 0u) && (c_aucWarmPi[ucCam] == 0u))
    {
        /* 档位预热: "W,<cam>,<uses>,<plan>" —— 头是 **W**(RPI_TASK_WARM_PROF),
         * 第二个数才是相机码（c_aemWarmTask 那张表在这里用不上: 它给的是 B/C 的老码）。 */
        vRpiSendTaskCmd3NumRaw(RPI_TASK_WARM_PROF, (uint32_t)ucCam,
                               ulUsesWire, ulProfPlan);
    }
    else if (c_aucWarmPi[ucCam] != 0u)
    {
        vRpiSendTaskCmd2NumRaw(c_aemWarmTask[ucCam], ulUsesWire, 1u);
    }
    else
    {
        vRpiSendTaskCmdNumRaw(c_aemWarmTask[ucCam], (ulUses >= 2u) ? ulUses : 0u);
    }
}

void vRpiWarmCamera(emWarmCamTdf emCam, uint32_t ulUses, uint32_t ulProfPlan)
{
    uint8_t ucCam = (uint8_t)emCam;

    /* 越界的一律按 0 处理（不猜）: 计划比 10 次使用还大 → 当"全默认"（= 老行为）;
     * 相机认不出则不预热。⚠️ 别把这里写成 `> 2` —— 计划是**打包值**, 3 以上是合法的
     * （例如 uses=2、第 2 次看路 = 4）。 */
    if (ulProfPlan > WARM_PROF_PLAN_MAX)
    {
        ulProfPlan = 0u;
    }

    /* 「两颗一起」= 两条帧背靠背（先主摄后 pi, 次序无所谓: 两个持有器互不干扰,
     * Pi 端两条预热都只开相机、不回帧）。
     * ⚠️ uses 对**两颗同时**生效（各算各的窗口）—— 主摄那格别开太大: 它还常被网页预览抢。
     * ⚠️ 档位只给主摄那一条: CSI 没有这套档位（见 app_rpi.h 的 WARM_PROF_* 说明）。 */
    if (emCam == WARM_CAM_MAIN_PI)
    {
        vRpiWarmOne((uint8_t)WARM_CAM_MAIN, ulUses, ulProfPlan);
        vRpiWarmOne((uint8_t)WARM_CAM_PI, ulUses, 0u);
        return;
    }
    if (ucCam >= (uint8_t)(sizeof(c_aemWarmTask) / sizeof(c_aemWarmTask[0])))
    {
        return;                             /* 未知相机: 不发 */
    }
    vRpiWarmOne(ucCam, ulUses, ulProfPlan);
}

/* ---- 自动重启 Pi 客户端（"r"）----
 * ⚠️⚠️ **这是全工程唯一一处自动发出的、绕过 Pi 侧人工确认门的危险指令**（有意为之:
 *    「重连超时就自动重启」）。Pi 的 handler.py::_require_double_send 要求同一条指令
 *    **两次操作各一帧**才执行, 所以必须**分两帧、中间隔一段**发 —— 一次调用里连发
 *    两帧会被只登记不执行（实测踩过, 见上面「树莓派系统级操作」那段注释）。
 * ⚠️ 两帧之间查一次中止位: 宁可留一条 Pi 侧"已登记未执行"的 r 让它 10s 过期,
 *    也不要在操作员已经按停之后真把 Pi 重启掉。
 * ⚠️ 只能在**等待者自己的任务上下文**里调（它带 vTaskDelay）。**绝不能在 vRpiTask 里调**:
 *    那是 UART_RPI 唯一的 RX 消费者, 在它里面等 500ms 会覆盖约 11 圈 RX 单缓冲,
 *    正在等的那一帧必丢 —— 现象正是「Pi 说识别成功、单片机超时」。 */
static void vRpiRestartPiClient(void)
{
    /* 互锁: 操作员刚在 OLED 上发过 sd/rb（同样走 Pi 那道确认门的危险指令）→
     * 这段时间内一律不自动重启。
     * ✅ **2026-09 已到 Pi 侧源码核实: 这道门当前是冗余的** —— handler.py 的门状态是
     *    `_system_cmd_arm` / `_system_cmd_last` 两个 **以 payload 为键**的 dict
     *    （`_system_cmd_arm.get(data)`）, `"sd"`/`"rb"`/`"r"` 各记一份, 所以"两条指令在
     *    确认门上凑单"**不可能发生**。保留它是防哪天那边把门改成**全局单槽** —— 那时这条
     *    互锁就从"冗余"变成"必需"（否则自动 "r" 可能把等着的关机变成立刻关机, 不可逆）。
     *    代价很小: 只在操作员刚发过 sd/rb 时生效, 而那正是 Pi 本来就要关/重启、
     *    自动重启也救不回来的场景。要精简可以删掉这一块 + RPI_SYSCMD_SUPPRESS_MS。 */
    if ((xTaskGetTickCount() - s_ulSysCmdTick) < (TickType_t)pdMS_TO_TICKS(RPI_SYSCMD_SUPPRESS_MS))
    {
        return;
    }

    vRpiSendTaskCmdRaw(RPI_TASK_RESTART);
    s_usLinkRetry++;

    vTaskDelay(pdMS_TO_TICKS(RPI_RESTART_CONFIRM_GAP_MS));

    /* ⚠️⚠️ 第二帧之前必须**重查链路**, 不只是中止位。
     *    Pi 侧收到 "r" 是「关串口 → time.sleep(20) → os.execl」（serial_comm.py::restart_program,
     *    阻塞在 handler 主循环里）, 而重启完成的**那一刻**它就发 SYNC_HS_1
     *    （handler.py::_handshake 是 run() 的第一步, 进循环立刻首发）—— 本端在几毫秒内
     *    就把链路置回 OK。若此时还照发第二帧, Pi 的确认门会**再确认一次** → 20s 后它又
     *    重启一遍, 把我们刚发出去的那个任务掐掉（而且全程没有报错）。
     *    只发第一帧是**安全**的: 它在 Pi 侧只是"登记", 10s 窗口一过自动作废
     *    （超窗口再发 = 重新登记, 见 handler.py::_require_double_send）。 */
    if ((ucGetScriptAbortReq() == 0u) && (emGetRpiLinkState() != RPI_LINK_OK))
    {
        vRpiSendTaskCmdRaw(RPI_TASK_RESTART);
        s_usLinkRetry++;
    }
}

/* 公开壳（2026-10 加, 见 .h 注释）: `$B,MODE,0` 退出编辑模式且正处让位窗时立刻踢,
 * 不等 ucRpiHandshake 里那个 6s 重试闸。链路判活/中止位那些护栏都在
 * vRpiRestartPiClient 内部（心跳判活不在此处 —— 调用方场景里"踢一脚"本身就是要的）。 */
void vRpiKickPiClient(void)
{
    vRpiRestartPiClient();
}

/* ---- 等链路 OK 再发任务（**唯一会等链路的入口**）----
 * 返回 1 = 帧已发出（调用方接着 ucRpiWaitXxx 等结果即可）; 0 = 被中止 / 被顶掉（**没发**）。
 *
 * ⚠️ 只能被 **vScriptTask** 调（唯一会阻塞等链路的任务）。**但「任务发起」入口不止它一个**
 *    （OLED 的拍照/关机经 vRpiSendTaskCmd 族、vRpiWarmCamera 也会顶槽, 见上面那批
 *    「只发不等」入口）—— 所以 2026-10-09 起槽字段与代次的读-改-写都进了短临界区:
 *    ① vRpiPendSupersede 整体原子（遥测 usPendDrop 不再丢计数）;
 *    ② 本函数「顶掉旧 + 登记自己」一次临界区完成（杜绝第三者在两步之间插队）;
 *    ③ 「最后一查 + 清槽」原子 —— 老写法两行之间被顶掉时, 等待者仍把被顶掉的帧发出,
 *    且随后的 s_ucPendTask=0 会把**新等待者刚登记的槽**抹掉。发送动作仍在临界区外
 *    （vUartSendArray 环满会阻塞背压, 不能进临界区）; 清槽到发送之间被顶掉的残窗
 *    最多表现为「两帧都上了线」, 槽与代次记账则始终正确。
 *
 * ⚠️⚠️ **发送动作刻意由等待者自己做, 不由 vRpiTask 代发。** 那样要跨任务交代「我这条
 *    发出去了没有」, 而「派发我的 → 10ms 内又来一个新投递并被派发」会把那个信息覆盖 →
 *    等待者把自己**已经发出去**的任务误判成"被顶掉" → 本步按失败走, 可那一帧真的发出去了,
 *    Pi 几秒后回来的结果帧会落进**下一步**的等待窗口、被当成那一步的结果 ——
 *    从这一步起同类型步骤的结果整体错位一格, **全程无任何报错**。
 *    自己发就没这回事: 判据退化成纯局部的「发了就是 1、没发就是 0」。
 *
 * ⚠️ **等待重连的时间不计入积木自己的超时**: 本函数在 ucRpiWaitXxx 之**前**返回, 所以
 *    积木的 I(0) 天然只覆盖「帧真正发出去之后」那一段 —— 一行代码都不用为它写。
 *
 * ⚠️ 判据是「链路 != OK」, 所以 WAITING(0) 与 DISCONNECTED(2) **都等**。WAITING 也等是
 *    刻意的: 那是「Pi 从没上过电 / 本端比 Pi 晚启动」的稳态 —— DISCONNECTED 只能从 OK
 *    进入, 而心跳救不了 WAITING（只有 Pi 重发握手才行）。发 "r" 重启 Pi 客户端正是唯一
 *    能把它救回来的手段: 客户端重启 → 重发 SYNC_HS_1 → 本端回 ACK → OK。 */
/* ⚠️ 本函数是**上面那段契约的唯一实现**, 参数化了"带不带 is_use_picam 标志"
 *    （2026-10-16 加, 只为平移射靶优先 pi 摄像头）:
 *      ulUsePicam == 0 → 末句发的是与改造前**逐字节相同**的 "<码>[,<ms>]";
 *      ulUsePicam != 0 → 发 "<码>,<ulNum1>,1"（三字段形状, Pi 侧 _split_payload 认它）。
 *    ⚠️ ulNum1 == 0 且带标志时**不能**走"0 → 裸发"那条捷径（发了标志就得发全字段）——
 *       所以 "7" 起流带标志时线上是 "7,0,1", 第二字段只是个占位, Pi 对 "7" 忽略它。
 *    ⚠️ 挂起槽(单槽)只记任务码: 标志是**等待者自己的局部量**, 重连后由它自己重发
 *       （见上面"发送动作刻意由等待者自己做"）, 槽里不需要也不该多存一份。 */
static uint8_t ucRpiSendTaskCmdWaitEx(emRpiTaskTdf emTask, uint32_t ulNum1, uint32_t ulUsePicam)
{
    uint32_t ulMySeq;

    if ((uint8_t)emTask == 0u)
    {
        return 0u;                      /* 未知任务码: 与 vRpiSendTaskCmd 同口径, 不发 */
    }

    taskENTER_CRITICAL();               /* 顶掉旧 + 登记自己一次做完（2026-10-09） */
    vRpiPendSupersede();
    ulMySeq       = s_ulPendSeq;        /* 记下"我是谁" */
    s_ucPendTask  = (uint8_t)emTask;
    s_ulPendTick  = xTaskGetTickCount();
    s_ulPendRetry = s_ulPendTick;
    taskEXIT_CRITICAL();

    while (emGetRpiLinkState() != RPI_LINK_OK)      /* 快路径: 链路正常时不进循环 */
    {
        if (ucGetScriptAbortReq() != 0u)            /* 中止出口（工程铁律: 等外部事件的循环都要带） */
        {
            s_ucPendTask = 0u;
            vRpiSendDebug("LINKWAIT abort");
            return 0u;
        }
        if (s_ulPendSeq != ulMySeq)                 /* 被新任务顶掉: 不发 */
        {
            vRpiSendDebug("LINKWAIT superseded");
            return 0u;
        }
        /* 让位窗（台架调臂）里链路是被**故意**钉在 WAITING 的, 绝不能因此去重启 Pi ——
         * 没有这道守卫就是"调一次臂重启一次 Pi"。 */
        if ((ucGetBtHsWait() == 0u) &&
            ((xTaskGetTickCount() - s_ulPendRetry)
                > (TickType_t)pdMS_TO_TICKS(RPI_LINK_RETRY_TIMEOUT_MS)))
        {
            vRpiRestartPiClient();
            s_ulPendRetry = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));              /* 与工程里所有等待循环同节拍 */
    }

    /* 最后一查 + 清槽做成原子（2026-10-09）: 老写法「查完 → 发送 → 清槽」三步分离,
     * 中途被顶掉时照样发送、且清槽会把新等待者的登记抹掉。现在: 查到代次没变就先
     * 抢占式清槽再出区发送; 发送在区外（vUartSendArray 可能阻塞背压, 不能进临界区）。 */
    {
        uint8_t ucSend = 0u;

        taskENTER_CRITICAL();
        if (s_ulPendSeq == ulMySeq)
        {
            s_ucPendTask = 0u;
            ucSend = 1u;
        }
        taskEXIT_CRITICAL();

        if (ucSend == 0u)
        {
            vRpiSendDebug("LINKWAIT superseded");
            return 0u;
        }
    }

    /* 自己发 —— 不等别人派发 */
    if (ulUsePicam != 0u)
    {
        vRpiSendTaskCmd2NumRaw(emTask, ulNum1, 1u);     /* "<码>,<n1>,1" */
    }
    else
    {
        vRpiSendTaskCmdMsRaw(emTask, ulNum1);           /* 与改造前逐字节相同 */
    }
    return 1u;
}

uint8_t ucRpiSendTaskCmdWait(emRpiTaskTdf emTask, uint32_t ulTimeoutMs)
{
    return ucRpiSendTaskCmdWaitEx(emTask, ulTimeoutMs, 0u);
}

uint8_t ucRpiSendTaskCmdWaitPi(emRpiTaskTdf emTask, uint32_t ulTimeoutMs, uint8_t ucUsePicam)
{
    return ucRpiSendTaskCmdWaitEx(emTask, ulTimeoutMs, (ucUsePicam != 0u) ? 1u : 0u);
}

/* ---- 遥测（只读, 蓝牙 Tab「通信」组）---- */
uint8_t  ucGetRpiPendTask(void) { return s_ucPendTask; }
uint16_t usGetRpiLinkRetry(void) { return s_usLinkRetry; }
uint16_t usGetRpiPendDrop(void)  { return s_usPendDrop; }
uint32_t ulGetRpiPendMs(void)
{
    return (s_ucPendTask == 0u)
        ? 0u
        : (uint32_t)((xTaskGetTickCount() - s_ulPendTick) * (TickType_t)portTICK_PERIOD_MS);
}

void vRpiSendDebug(const char *pcStr)
{
    if (pcStr == NULL) { return; }
    vRpiSendFrame(RPI_TYPE_DEBUG, (const uint8_t *)pcStr, (uint8_t)strlen(pcStr));
}

/* ---- 取消 Pi 侧当前任务（"x", 2026-10 加）----
 * 只发一帧、不等链路（链路不通就白丢 —— 那时 Pi 上多半也没有正在跑的任务）。
 * 治两件事:
 *   ① 迟到结果帧错位: 被中止的旧任务会在 Pi 上跑完并回帧, 落进下一轮同类型步骤的
 *      等待窗口被当成新结果（见 ucRpiSendTaskCmdWait 头注释里那条结构性风险）;
 *   ② 旧巡线流/靶标流残留: "3"/"8" 发不出去时（断电/中止）它们继续占主摄并以
 *      20~30Hz 发 COORD 帧, 与新任务的结果帧交错。
 * 调用点: vScriptPostReq（本端中止脚本时）与 ucRpiHandshake 的「断开重连」分支。
 * ⚠️ 非阻塞, 任意任务上下文（含 OLED/BT）可调; 重复调用幂等（Pi 侧 set 同一个事件）。 */
void vRpiCancelPiTask(void)
{
    vRpiSendTaskCmd(RPI_TASK_CANCEL);
}

/* ==================== 握手 ==================== */

/*
 * 等待握手（Pi 主动发起, 我方永不放弃）:
 *   阶段1: 等 Pi 的首个 "SYNC_HS_1"（vRpiTask 收到即已自动回 ACK 0xCC）,
 *          期间 OLED 状态页显示 RPI:Waiting（等待期不响）;
 *   阶段2: 静默确认窗 RPI_HANDSHAKE_CONFIRM_MS(> Pi 2.5s 重发周期)内无新
 *          握手请求 = Pi 已收到我们的应答, 双向链路皆通。
 *          窗口内又来请求 = 应答丢了, vRpiTask 已自动补答, 重开窗口。
 *          操作员按 MAIN 可**提前放行**（吃掉再原样放回, 不丢按键; ACK 本就由
 *          vRpiTask 自动回+自动补, 提前放行无链路风险 —— 见窗内注释）。
 *   阶段1 额外承担「蓝牙机械臂编辑模式」的让位: 期间置 bt_hs_wait, 并每轮调
 *   vBtEditPump()——编辑模式开着时脚本就停在这儿, 舵机交给蓝牙点动（app_bt.c）。
 * 全程 vTaskDelay 让出, 不卡其他任务。仅任务上下文调用。
 *
 * ★ 「**已经握手成功之后**又执行一次本步骤」= 操作者要求把当前连接判为断开、重新
 *   握手（路线在握手成功之后再排一步握手就是这个语义: 新一轮比赛/换了对手端要重新
 *   走一遍握手）。做法是落回 RPI_LINK_WAITING 再照常落进阶段1, 于是:
 *     · 树莓派——等它重发 SYNC_HS_1（它只在开机阶段发、ACK 到手即停, 所以本端
 *       **不主动请求**, 要么重启 Pi 主程序、要么给 Pi 重新上电）;
 *     · 蓝牙机械臂编辑模式——阶段1 会把 bt_hs_wait 重新拉高, PC 那边又能进编辑
 *       模式了（这就是「与蓝牙重新握手」）;
 *     · 模拟器模式同树莓派（同一个帧协议, 本端不区分对端是谁）。
 *   ⚠️ 判据是 `s_bHsDone`（本次上电脚本**走到过**阶段2 末尾）而**不是**「链路是 OK」:
 *      vRpiTask 是独立任务, 对端先上电时它会在脚本还没跑到这一步时就应答、置 OK
 *      （Pi 比我们先开的机 —— 正常顺序之一）。那是「对端先握好了」, 不是「又要握
 *      一次」, 必须照旧直接放行; 否则每次开机都有大约「一次 SYNC_HS_1 落在启动
 *      窗口里」的概率卡死在等一个再也不会来的握手请求上。
 *   ⚠️ 这里**不能**用 RPI_LINK_DISCONNECTED: vRpiTask 有一条「Disconnected 态收到
 *      任何有效帧即恢复 OK」的规则（给「Pi 忙完/重连后发心跳」用的), 对端下一个
 *      心跳(2s)就会把状态翻回 OK; 而 vBtEditPump 一见 OK 立刻把 PC 赶出编辑模式
 *      —— 上面那条让位窗就闩不上了。WAITING 是唯一既能如实表示「正在等握手」、
 *      又不会被心跳改写的状态（OLED 状态页显示 RPI:Waiting）。
 */
static uint8_t s_bHsDone = 0;               /* 本次上电以来成功握手过（只有脚本任务写/读） */

uint8_t ucRpiHandshake(void)
{
    uint32_t ulWaited;
    uint32_t ulKickTick;                   /* 阶段1 自动 "r" 的限速计时（见循环内注释） */

    if (emGetRpiLinkState() == RPI_LINK_OK)
    {
        if (s_bHsDone == 0u)
        {
            s_bHsDone = 1u;
            /* 本端刚上电、对端却已应答（Pi 断线重连后先发 SYNC_HS_1）→ 直接放行。
             * 但 Pi 侧可能还残留**上一会话**的任务/流（它没跟着我们断电）: 先取消一遍,
             * 干净起步。Pi 空闲时这只是它日志里一行「已取消当前任务」, 无副作用。 */
            vRpiCancelPiTask();
            return 1u;
        }
        vRpiCancelPiTask();                 /* 同上: 旧会话的巡线流/靶标流可能还活着占主摄 */
        vSetRpiLinkState(RPI_LINK_WAITING); /* 已经握过又执行一次 → 视为断开, 重来 */
    }

    (void)ucTakeHandshakeSeen();            /* 清残留（上次握手 / 途中对端重连 / 上面的重连） */

    /* 阶段1: 等首个握手请求
     * 期间若蓝牙请求机械臂编辑模式, 就在这里让位——脚本停在原地不动舵机, 六个关节
     * 交给蓝牙点动（见 app_bt.c）。这是「点动与路线抢不了舵机」的结构性保证:
     * 编辑模式只在 bt_hs_wait 置位（= 本循环里）时才被允许进入。
     * ⚠️ 阶段2 不能调 vBtEditPump: 那里的语义是「连续 RPI_HANDSHAKE_CONFIRM_MS
     *    内没有新握手请求」, 停住会让 ulWaited 不涨 → 窗口永不过期而卡死。 */
    ulWaited = 0;
    ulKickTick = xTaskGetTickCount();
    vSetBtHsWait(1u);
    while (!ucTakeHandshakeSeen())
    {
        /* 中止请求出口 —— ⚠️ 没有它, 由菜单发起的「重新握手」在 Pi 不上电时是**死等**:
         * 阶段 1 本来就设计成无限等（那是台架调臂的工作态: bt_hs_wait=1, 蓝牙可进
         * 机械臂编辑模式）, 但操作员总得有条路退出去。菜单随后发的任何请求都会把
         * 中止位置 1（见 vScriptPostReq）, 于是这里返回 0、臂收回、菜单接着办新事。
         * ⚠️ 返回 0 时**不动** s_bHsDone: 本次确实没握上手, 下次进来照旧重走一遍。
         * ⚠️ 路线里的 STEP_WAIT_HANDSHAKE 忽略返回值, 但它后面紧跟的 vScriptRun
         * 循环会看到同一个中止位并以「整条路线结束」收尾 —— 不会带着半截状态往下跑。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vSetBtHsWait(0u);            /* 出阶段1: 臂收回（与正常出口一致） */
            return 0u;
        }

        if (vBtEditPump() == 0u)
        {
            vTaskDelay(pdMS_TO_TICKS(10));   /* 让位时泵内部已延时, 别再叠加成 20ms */
        }
        ulWaited += 10;                      /* 让位期间 ulWaited 照常累计 */

        /* ---- 「对端活着、却永远不再发握手」的自愈（2026-10 加）----
         * 死锁场景: Pi 没断电、串口也活着, 但它**只在启动阶段**发 SYNC_HS_1 ——
         * 本端断电重启（或路中间的重握手步）后链路停在 WAITING, 而心跳帧每 2s 照来,
         * 两边互相干等（本端等握手, Pi 等指令）, 之前唯一的出路是手动重启 Pi。
         * 判据 = 心跳还在来（s_xLastRpiFrameTick 距今 < 断链阈值）却迟迟等不到握手
         * 请求 → 超过 RPI_LINK_RETRY_TIMEOUT_MS 就发一对 "r" 让 Pi 客户端重启,
         * 它起来第一件事就是发 SYNC_HS_1, 死锁解除。
         * ⚠️ Pi 真没上电/串口不在时 s_xLastRpiFrameTick 不更新 → 判据恒假,
         *    台架「先调臂再给 Pi 上电」的工作态**一字不变**（不会反复白发 "r"）。
         * ⚠️ 蓝牙机械臂编辑模式**开着时不踢**（编辑窗口要在阶段1 停多久是操作员说了算,
         *    重启 Pi 不影响编辑, 但握手一成窗口就关了）; 编辑一退出, 若死锁仍在,
         *    下一轮就照常踢。 */
        if ((ucGetBtEditMode() == 0u) &&
            (s_xLastRpiFrameTick != 0u) &&  /* 开机头几秒 tick 差会假装「最近有帧」, 显式排除 */
            ((xTaskGetTickCount() - s_xLastRpiFrameTick)
                < (TickType_t)pdMS_TO_TICKS(RPI_DISCONNECT_TIMEOUT_MS)) &&
            ((xTaskGetTickCount() - ulKickTick)
                > (TickType_t)pdMS_TO_TICKS(RPI_LINK_RETRY_TIMEOUT_MS)))
        {
            vRpiSendDebug("HS kick: peer alive but no SYNC_HS_1");
            vRpiRestartPiClient();
            ulKickTick = xTaskGetTickCount();
        }
    }
    vSetBtHsWait(0u);                        /* 出阶段1: 臂收回（循环无早退出口） */

    /* 阶段2: 静默确认窗（操作员按 MAIN 可提前放行, 见循环内注释） */
    ulWaited = 0;
    while (ulWaited < RPI_HANDSHAKE_CONFIRM_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;
        if (ucTakeHandshakeSeen()) { ulWaited = 0; }    /* 应答丢了, 已补答, 重开窗口 */

        /* ⚠️ 操作员按 MAIN = 「别等了, 赶紧放行」: 提前结束确认窗。
         *    窗口的本意只是「3s 内没有重发的 SYNC_HS_1 = ACK 送达」, 而 ACK 是
         *    vRpiTask 收到 SYNC_HS_1 就自动回的, 对端重发也会被再次自动补答 ——
         *    提前放行最坏也只是「ACK 曾丢 → 对端 2.5s 后重发 → 又被自动补答」,
         *    链路状态自愈, 不值得让人干等满 3s（2026-10-05: 路线表头是
         *    等握手 → 预热 → 按 MAIN, Pi 侧握手成功到能启动之间这 3s 正是
         *    操作员「握手成功后要等一会才能按 MAIN 启动」的来源）。
         *    ⚠️ MAIN 是读后清零的一次性信号: 这里**吃掉再原样放回** —— 既结束
         *       窗口, 又不丢操作员这一按（后面 STEP_DEAD/STEP_STOP 的等键还要消费它）。
         *       阶段 1 里按下的 MAIN 也走这里生效: 标志在进窗前就已置位, 首轮
         *       10ms 就命中 → 窗口名存实亡, 路线在握手链路刚通时就放行。 */
        if (ucTakeMenuConfirm() != 0u)
        {
            vSetMenuConfirm();              /* 原样放回: 窗口提前结束, 这一按不丢 */
            break;
        }
    }

    s_bHsDone = 1u;                         /* 走完静默确认窗才算本次「握手成功」 */
    return 1;
}

/* ==================== 相机健康检查（2026-11 加） ==================== */

/*
 * 四路相机体检（STEP_WAIT_HANDSHAKE 的 health 参数勾选时, 握手成功后调用）:
 *   发 "Z"（等链路）→ 等 Pi 回 TAG=0x0D 帧 → 写 Beam → 返回。
 *   四项: M=主摄 / Q=扫码副摄 / Pi=CSI / MV=OpenMV, 1=通过 0=失败。
 *
 * ⚠️ **阻塞原语**: 脚本任务停在等待循环里, 后面任何"发车"步骤都执行不到 ——
 *    这正是需求「检查过程中不能突然发车」的结构性保证。等待循环查中止位。
 * ⚠️ 发起帧用 ucRpiSendTaskCmdWait（**别用 vRpiSendTaskCmd**）: 链路不是 OK 时
 *    挂起等重连, 被顶掉/被中止返回 0; 照发不等的话断了就是白丢 + 空等超时。
 * ⚠️ 超时/链路失败都按**四项全失败**写 Beam（显示 ×）—— 检查没做完就不是"通过",
 *    不能让 H: 行停在 `?` 上让人误以为车还能跑; 失败音**不响**（用户定:
 *    只在屏上显示, 是否中止由路线里的 IF 条件决定）。
 * ⚠️ Pi 侧检查完会主动释放四路占用（USB 租约/CSI/OpenMV 'S'）—— 本端不再发任何
 *    收尾指令, 也就不会把刚预热好的相机收掉。
 */
uint8_t ucRpiHealthCheck(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0u;
    uint8_t  ucM  = RPI_HEALTH_UNKNOWN;
    uint8_t  ucQ  = RPI_HEALTH_UNKNOWN;
    uint8_t  ucPi = RPI_HEALTH_UNKNOWN;
    uint8_t  ucMv = RPI_HEALTH_UNKNOWN;

    (void)ucTakeHealthReady();              /* 清残留（上一次超时后迟到的结果帧） */
    vSetHealthResult(RPI_HEALTH_UNKNOWN, RPI_HEALTH_UNKNOWN,
                     RPI_HEALTH_UNKNOWN, RPI_HEALTH_UNKNOWN);
    vSetHealthBusy(1u);

    if (ucRpiSendTaskCmdWait(RPI_TASK_HEALTH, ulTimeoutMs) == 0u)
    {
        /* 链路没通 / 被顶掉 / 被中止 —— 没发出去, 四项都按失败显示 */
        vSetHealthResult(RPI_HEALTH_FAIL, RPI_HEALTH_FAIL,
                         RPI_HEALTH_FAIL, RPI_HEALTH_FAIL);
        vSetHealthBusy(0u);
        vRpiSendDebug("HEALTH fail: link down");
        return 0u;
    }

    while (ulWaited < ulTimeoutMs)
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            /* 操作员按停: 结果保持"未检查", 别把一次没做完的体检伪装成失败 */
            vSetHealthResult(RPI_HEALTH_UNKNOWN, RPI_HEALTH_UNKNOWN,
                             RPI_HEALTH_UNKNOWN, RPI_HEALTH_UNKNOWN);
            vSetHealthBusy(0u);
            vRpiSendDebug("HEALTH abort: stop by request");
            return 0u;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10u;

        if (ucTakeHealthReady() == 0u) { continue; }

        vSetHealthBusy(0u);
        vGetHealthResult(&ucM, &ucQ, &ucPi, &ucMv);
        {
            char acDbg[44];
            (void)snprintf(acDbg, sizeof(acDbg),
                           "HEALTH: M=%u Q=%u Pi=%u MV=%u",
                           (unsigned)ucM, (unsigned)ucQ,
                           (unsigned)ucPi, (unsigned)ucMv);
            vRpiSendDebug(acDbg);
        }

        return ((ucM  == RPI_HEALTH_OK) && (ucQ  == RPI_HEALTH_OK)
             && (ucPi == RPI_HEALTH_OK) && (ucMv == RPI_HEALTH_OK)) ? 1u : 0u;
    }

    /* 超时: 结果帧没来 —— 检查没完成, 按失败显示（不响, 调试串留痕） */
    vSetHealthResult(RPI_HEALTH_FAIL, RPI_HEALTH_FAIL,
                     RPI_HEALTH_FAIL, RPI_HEALTH_FAIL);
    vSetHealthBusy(0u);
    {
        char acDbg[40];
        (void)snprintf(acDbg, sizeof(acDbg), "HEALTH fail: wait(t=%lums)",
                       (unsigned long)ulTimeoutMs);
        vRpiSendDebug(acDbg);
    }
    return 0u;
}

/* ==================== 二维码结果等待 ==================== */

/*
 * 等二维码结果帧（Pi handler._cmd_qr 回传, 9B ASCII COORD）:
 *   成功 = "QR"+三位数字（补零到 9B）; 失败 = "QRF"。
 *   期间其他 COORD 帧（坐标等）到达则忽略继续等; 巡线帧走独立通道不影响。
 *   结果写入 Beam（iGetQrCode, 三位数; 失败/超时置 -1）。
 *   失败/超时 → 投失败音（非阻塞; 中止不响 —— 停是操作员的意图）。
 *   返回 1=成功, 0=Pi 报失败或超时。仅任务上下文调用。
 */
uint8_t ucRpiWaitQrResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t  aucPkt[9];                     /* ⚠️ 拷贝式读（见下面注释）, 不再拿内部指针 */

    (void)ucTakeRpiFrameReady();            /* 清残留旧帧 */
    vSetQrCode(-1);

    while (ulWaited < ulTimeoutMs)
    {
        /* ⚠️ 查中止位（与 ucRpiWaitColorResult 同一条纪律）: 本函数最长阻塞 I(0) 那么多
         * （扫码默认 6~7.5s）, 不查的话「停止 / 菜单改跑别的预设」要等满它才生效。
         * 中止按"本次没结果"返回 0（与超时同一条出口, 调用方不必分辨）。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("QR abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeRpiFrameReady()) { continue; }
        /* ⚠️ **拷贝式读**: 原来拿 pucGetLastRpiPacket() 返回的内部指针又逐字节读 5 个,
         * 而 vRpiTask（prio 4）随时能整包覆盖它 —— 撕裂后 "QR"+3 位数字**仍可能凑出来**
         * （前缀照样对上、某一位来自新帧）, 于是产出一个看着合法的错二维码, 后面排爆
         * 选色/标靶选靶全按它走, 全程不报错。一次拷走即可闭合。 */
        vGetLastRpiPacket(aucPkt);

        if (aucPkt[0] == 'Q' && aucPkt[1] == 'R')
        {
            if (aucPkt[2] == 'F')
            {
                vSetBuzzerReq(BUZZER_PAT_FAIL);     /* Pi 报失败: 失败音 */
                return 0;
            }

            if (aucPkt[2] >= '0' && aucPkt[2] <= '9' &&
                aucPkt[3] >= '0' && aucPkt[3] <= '9' &&
                aucPkt[4] >= '0' && aucPkt[4] <= '9')
            {
                vSetQrCode((aucPkt[2] - '0') * 100
                         + (aucPkt[3] - '0') * 10
                         + (aucPkt[4] - '0'));
                return 1;
            }
        }
        /* 非 QR 帧（坐标等）: 忽略, 继续等 */
    }
    vSetBuzzerReq(BUZZER_PAT_FAIL);                 /* 超时（Pi 没回帧/断链）: 同样响 */
    return 0;
}

/* ==================== 二维码扫描（组合动作） ==================== */

/*
 * 二维码扫描的**唯一实现**（两个原语共用）:
 *   发 emTask 指定的指令 → 阻塞等结果帧 → 失败音（失败/超时各投一声, 成功不响）。
 *   结果在 Beam iGetQrCode（三位数, 失败/超时 -1）。仅任务上下文调用。
 *
 * ⚠️ 两条路（RPI_TASK_QR = 扫码副摄 / RPI_TASK_QR_MAIN = 主摄）**只差这一条任务码**:
 *    结果帧格式相同（§6a 的 9B ASCII "QR<ddd>"/"QRF"）, 所以等待与下游消费
 *    （IF 条件 / 排爆 / 反恐 读的都是 iGetQrCode）一个字都不用变。
 *    换相机是 Pi 侧的事 —— 别在这边加任何"按相机区分"的分支。
 */
static uint8_t ucQrScanCommon(emRpiTaskTdf emTask, uint32_t ulTimeoutMs)
{
    uint8_t ucOk;

    /* 二维码是**人工预设**的（OLED 第 2 页编辑 / 调试积木）→ 直接返回、**不问 Pi**,
     * 沿用那个值（见 app_beam.h 的 ucGetQrManual）。
     *
     * ⚠️ 这道门必须在 vRpiSendTaskCmd **之前**: ucRpiWaitQrResult 一进来就
     *    vSetQrCode(-1) 把上一次的结果清掉 —— 指令一发出去, 手工设的值就已经被
     *    「没扫到」覆盖了, 再判就晚了。
     * ⚠️ 返回 1（= 有结果）: 下游按「拿到了二维码」走, 它们读的是 iGetQrCode(),
     *    手工值与扫来的值在那里没有区别。
     * ⚠️ 这条门**不问 Pi 也不响**: 跳没跳过看 Pi 日志的 `QR preset: skip scan`
     *    与 OLED 第 1 页的 `QR:xxx*`（带星号 = 人工预设）。
     * ⚠️ 「人工预设」与用哪台相机无关, 所以两个原语共用这一道门。 */
    if (ucGetQrManual() != 0u)
    {
        vRpiSendDebug("QR preset: skip scan");
        return 1u;
    }

    /* 超时**随指令一起发给 Pi**（"<命令码>,<毫秒>"）: 那一边留一点余量后拿它当自己的
     * 预算, 于是本函数下面那个 ucRpiWaitQrResult(ulTimeoutMs) 一定**后**到期 ——
     * 「谁先放弃」不再靠两个仓库各记一个数字互相迁就。 */
    /* ⚠️ 链路不通时在这里等重连 —— **等多久都不计入下面的 ulTimeoutMs**:
     *    时限是随帧一起发给 Pi 的, Pi 的预算从**收到**那一刻起算, 与本端等了多久无关。
     * 返回 0 = 被新任务顶掉 / 被中止（帧没发出去）, 与"扫失败"共用下面那个失败出口。 */
    if (ucRpiSendTaskCmdWait(emTask, ulTimeoutMs) == 0u)
    {
        return 0u;
    }
    ucOk = ucRpiWaitQrResult(ulTimeoutMs);
    return ucOk;
}

/*
 * 相机 × 算法 → 任务码。
 * ⚠️ 越界（手写 map.c 或老参数）一律**回落自动档**（= 一直以来最稳的那条）, 绝不发错命令。
 */
static const emRpiTaskTdf c_aemQrTask[2][2] = {
    /*                自动                    自研 */
    /* 扫码副摄 */ { RPI_TASK_QR,          RPI_TASK_QR_DIFF      },
    /* 主摄     */ { RPI_TASK_QR_MAIN,     RPI_TASK_QR_MAIN_DIFF },
};

static emRpiTaskTdf c_emQrTaskOf(uint8_t ucUseMain, emQrAlgoTdf emAlgo)
{
    uint8_t ucCam  = (ucUseMain != 0u) ? 1u : 0u;
    uint8_t ucAlgo = (uint8_t)emAlgo;

    if (ucAlgo > 1u) { ucAlgo = 0u; }
    return c_aemQrTask[ucCam][ucAlgo];
}

/*
 * 二维码扫描一步到位（脚本动作 STEP_QR_SCAN_WAIT 的原语）: 用**扫码副摄**（/dev/camera_Qr）。
 *   emAlgo 选解码算法（自动/自研）, 与相机一起决定发哪条任务码。
 *   自动档对应命令 "1" —— Pi 端 2026-10 起走「全帧定位→紧裁→强放大」级联,
 *   命中实测 60~75ms, 且不再依赖"码在画面中心"（旧固定裁剪的致命前提）。
 */
uint8_t ucRpiQrScan(uint32_t ulTimeoutMs, emQrAlgoTdf emAlgo)
{
    return ucQrScanCommon(c_emQrTaskOf(0u, emAlgo), ulTimeoutMs);
}

/*
 * 主摄二维码扫描（脚本动作 STEP_QR_SCAN_MAIN_WAIT 的原语）: 用**主摄**（/dev/camera_main）。
 *   见 ucQrScanCommon 的注释 —— 与 ucRpiQrScan 是同一个实现, 只换任务码。
 *   ⚠️ 不新增任何等待/发布逻辑: 结果帧与 "1" 同格式, 照样落进 iGetQrCode。
 */
uint8_t ucRpiQrScanMain(uint32_t ulTimeoutMs, emQrAlgoTdf emAlgo)
{
    return ucQrScanCommon(c_emQrTaskOf(1u, emAlgo), ulTimeoutMs);
}

/* ==================== 随机二维码（扫码无果时的兜底） ==================== */

/*
 * 随机源: 文件内 xorshift32（唯一调用点是 vSetQrCodeRandom）。
 *
 * ⚠️ 刻意**不引 libc `rand()`**: 工程至今一处没用过它（体积/风格上一直自己写格式化,
 *    见 app_bt.c 的 bt_fmt_f2）, 而且 `rand` 的状态是个全局量、非可重入 —— 谁都能从
 *    别的任务里把它搅乱。本用途只是「兜底猜一个二维码」, 不需要密码学强度。
 *
 * 熵: 每次调用都把**当前 tick + 调用序号 + 芯片 UID** 混进状态, 三者各管一件事 ——
 *   · tick（1000Hz, 本文件既有惯用法）: 同一块板子上电后的不同时机取到不同的值;
 *   · 调用序号(乘 0x9E3779B9): 保证**同一毫秒内**连着调两次也不同
 *     （只靠 tick 的话那种情况会退化成同一个值）;
 *   · UID: 两块板子不会因为「同时上电 + 操作员手法相似」而撞出同一个值。
 */
static uint32_t c_ulQrRandNext(void)
{
    static uint32_t s_ulState = 0u;
    static uint32_t s_ulCalls = 0u;

    s_ulCalls++;
    s_ulState ^= ((uint32_t)xTaskGetTickCount()
                  ^ (s_ulCalls * 0x9E3779B9u)
                  ^ HAL_GetUIDw0());
    if (s_ulState == 0u)
    {
        s_ulState = 0x2468ACE0u;   /* xorshift 的状态**不能全 0**（会永远输出 0） */
    }

    s_ulState ^= s_ulState << 13;
    s_ulState ^= s_ulState >> 17;
    s_ulState ^= s_ulState << 5;
    return s_ulState;
}

/*
 * 「随机二维码」(脚本动作 STEP_QR_SET_RANDOM 的原语): 把二维码结果写成一个**随机的
 * 合法三位数**（每位 1~3, 共 27 种）, 并打上「人工预设」标记 —— 效果等同于有人拿
 * OLED 第 2 页随手设了一个值。用途: 多次扫码识别无果之后让路线继续往下走 ——
 * 排爆抓取（首位=目标色）/ 反恐任务（第三位=形状）/ 标靶校准（二位=靶色）都按二维码
 * 选目标, 拿不到值（-1）就整步不动（调试串留痕）, 于是后面整条链都跑不下去。
 *
 * ⚠️ `vSetQrManual(1u)` 必须写在 `vSetQrCode()` **之后**: 控制台取积木原语名读的是
 *    case 体里**第一个**函数调用（同 STEP_QR_SET_TEST 那条注意事项）。
 * ⚠️ 打了标记 = **后面所有二维码扫描步骤都会跳过、不再问 Pi**（ucQrScanCommon 那道门）,
 *    只能复位/断电清除 —— 这是**有意**的: 否则随后一次失败的重扫会把它覆盖成 -1,
 *    下游又变成「没数据」（与「调试二维码」同一性质）。
 * ⚠️ **不保证与上一次的值不同**: 27 种组合里撞上同一个是正常的, 别拿它当「必定换一个」。
 * ⚠️ 它**不碰串口协议**（不发任何任务码、不等结果帧）, 只发一行调试串 —— 所以 Pi 端
 *    一个字都不用改, 也不占相机。
 *
 * 「随机值已生效」的听感判据已随蜂鸣撤除, 看屏显/日志（与 ucQrScanCommon 的人工预设门同口径）;
 * 到底随机到了哪个值: OLED 第 1 页 `QR:213*`、蓝牙遥测 iGetQrCode, 或 Pi 日志里
 * `QR random: 213` 那一行。仅任务上下文调用（发帧）。
 */
void vSetQrCodeRandom(void)
{
    char     acDbg[32];
    uint32_t ulR   = c_ulQrRandNext();
    int32_t  iCode;

    /* 一次抽签切三段: 27 = 3^3, 三段各取模 → 三位各自 1~3 且等概率。
     * 百位 = 目标色(1红/2绿/3蓝), 二位 = 靶色, 三位 = 序号/人质形状 —— 与
     * STEP_QR_SET_TEST 的取值域同构（那里是手填, 这里随机）。 */
    iCode = (int32_t)(1u + ((ulR / 9u) % 3u)) * 100
          + (int32_t)(1u + ((ulR / 3u) % 3u)) * 10
          + (int32_t)(1u + (ulR % 3u));

    vSetQrCode(iCode);
    vSetQrManual(1u);

    (void)snprintf(acDbg, sizeof(acDbg), "QR random: %d", (int)iCode);
    vRpiSendDebug(acDbg);
}

/* ==================== 人质识别结果等待 ==================== */

/*
 * 等人质识别结果帧（Pi tasks/_hostage.py 回传, TAG=0x02 二进制 COORD）:
 *   成功 = status=1（目标已由 Pi 按中心 x 升序排序, 结果经 vGetHostageResult 读）;
 *   失败 = status=0（相机失败/超时未凑满目标）。返回 1=成功, 0=失败/超时。
 *   走独立 ready 标志（不碰 last packet / RpiFrameReady）, 故不受坐标/QR 帧干扰。
 *   失败/超时 → 投一个失败音（非阻塞请求, 每次失败一声; 中止不响 —— 停是意图）。
 *   仅任务上下文调用。
 */
uint8_t ucRpiWaitHostageResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeHostageReady();             /* 清残留旧结果 */
    vSetHostageResult(0, 0, NULL);

    while (ulWaited < ulTimeoutMs)
    {
        /* ⚠️ 查中止位（同 ucRpiWaitColorResult）: 人质识别默认等 7500ms, 不查的话
         * 「停止 / 菜单请求」要等满它; 中止按"本次没结果"返回 0。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("HSTG abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeHostageReady()) { continue; }
        vGetHostageResult(&ucStatus, NULL, NULL);
        if (ucStatus != 1u)
        {
            vSetBuzzerReq(BUZZER_PAT_FAIL);     /* 识别失败: 失败音（蜂鸣任务异步响） */
            return 0u;
        }
        return 1u;
    }
    vSetBuzzerReq(BUZZER_PAT_FAIL);             /* 超时（Pi 没回帧/断链）: 同样响 */
    return 0;
}

/* ==================== 人质识别（组合动作） ==================== */

/*
 * 人质形状识别一步到位（脚本动作 STEP_HOSTAGE_SCAN_WAIT 的原语）:
 *   发 RPI_TASK_HOSTAGE 指令 → 阻塞等结果帧 → 失败音（失败/超时各投一声, 成功不响）。
 *   结果在 Beam（vGetHostageResult 读：状态 + 目标数组, 已按图像从左到右排序）。
 *   仅任务上下文调用。
 */
uint8_t ucRpiHostageScan(uint32_t ulTimeoutMs)
{
    uint8_t ucOk;

    /* 2026-10 起把本端超时随指令下发（"4,ms"）: Pi 拿它倒推自己的预算, 保证「Pi 先答」
     * —— 否则冷开相机 + 6s 检测可能压到本端 8s 断链阈值之后才回帧。ulTimeoutMs=0
     * 才是不带时限（旧行为）。返回值 0 = 被顶掉/被中止, 走失败出口。 */
    if (ucRpiSendTaskCmdWait(RPI_TASK_HOSTAGE, ulTimeoutMs) == 0u)
    {
        return 0u;
    }
    ucOk = ucRpiWaitHostageResult(ulTimeoutMs);
    return ucOk;
}

/* ==================== 色球识别结果等待 ==================== */

/*
 * 色球/矩形识别的**失败蜂鸣压制**标志（2026-10-27 加）:
 * 色球对位走 ucRpiAlignChain 多方案链（三球/矩形/拉回逐槽重试）—— 用户定的口径是
 * 「不到赌的时候不响」（见 ucRpiAlignChain 段头）: 链里每次扫描失败都不响,
 * 只在**进拉回（赌）**与**整链终失败**两个点由链自己响。链执行期间置 1,
 * ucRpiWaitColorResult / ucRpiWaitRectResult 看它压住失败音; 链外的色球失败
 * （STEP_COLOR_SCAN_WAIT、BALL1 的拉回重扫、排爆抓取的扫描）照常每失败响一次。
 * 单任务读写 = 脚本任务（这些等待函数都只能在脚本任务里跑）, 无并发问题。
 */
static uint8_t s_ucAlignBeepHold = 0u;

/*
 * 等色球识别结果帧（Pi tasks/_color.py 回传, TAG=0x03 二进制 COORD）:
 *   成功 = status=1（目标已由 Pi 按中心 x 升序排序, 结果经 vGetColorResult 读）;
 *   失败 = status=0（相机失败/超时未凑满三色）。返回 1=成功, 0=失败/超时。
 *   走独立 ready 标志（不碰 last packet / RpiFrameReady）, 故不受坐标/QR/巡线帧干扰。
 *   失败/超时 → 投失败音（非阻塞）; ⚠️ 对位链执行中（s_ucAlignBeepHold）不响,
 *   响不响归 ucRpiAlignChain 统一管。中止不响（停是操作员的意图）。
 *   仅任务上下文调用。
 */
uint8_t ucRpiWaitColorResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeColorReady();               /* 清残留旧结果 */
    vSetColorResult(0, 0, NULL);

    while (ulWaited < ulTimeoutMs)
    {
        /* ⚠️ 查中止位: 本函数最长阻塞 timeout（6s 级）, 不查的话「停止/菜单请求」要
         * 等满它才生效 —— 在多方案链里这个代价还会乘以方案数（CLAUDE.md「凡是等外部
         * 事件的循环都必须带上它」那条, 已踩过三次）。中止按"本次没结果"返回。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("BALL abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeColorReady()) { continue; }
        vGetColorResult(&ucStatus, NULL, NULL);
        if (ucStatus != 1u)
        {
            if (s_ucAlignBeepHold == 0u) { vSetBuzzerReq(BUZZER_PAT_FAIL); }
            return 0u;
        }
        return 1u;
    }
    if (s_ucAlignBeepHold == 0u) { vSetBuzzerReq(BUZZER_PAT_FAIL); }  /* 超时同样响 */
    return 0;
}

/* ==================== 矩形地板识别结果等待 ==================== */

/*
 * 等矩形地板结果帧（Pi tasks/_rect.py 回传, TAG=0x07 二进制 COORD）:
 *   成功 = status=1（板心像素经 vGetRectResult 读, 全分辨率）;
 *   失败 = status=0（板子没识别出来 / 超时）。返回 1=成功, 0=失败/超时/被中止。
 *   走独立 ready 标志（不碰 last packet / RpiFrameReady），故不受坐标/QR/巡线帧干扰。
 *   ⚠️ 失败帧也置 ready（见 vRpiTask 那个分支）—— 所以失败时**立刻**返回,
 *      不会白等满 timeout。
 *   ⚠️ 本函数**不投失败音**: 它唯一消费者是 ucRpiAlignChain（色球对位链）,
 *      链里「不到赌不响」归链统一管（见 s_ucAlignBeepHold 与 ucRpiAlignChain 段头）。
 *   ⚠️ 循环里查中止位, 理由同 ucRpiWaitColorResult。仅任务上下文调用。
 */
uint8_t ucRpiWaitRectResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeRectReady();                /* 清残留旧结果 */
    vSetRectResult(0u, 0u, 0u, 0u, 0u);

    while (ulWaited < ulTimeoutMs)
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("RECT abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeRectReady()) { continue; }
        vGetRectResult(&ucStatus, NULL, NULL, NULL, NULL);
        return (ucStatus == 1u) ? 1u : 0u;
    }
    return 0u;
}

/* ==================== 看路（主摄看地面黄色区 → 车体纵移 / 可选横移） ====================
 * 见 docs/串口协议.md §6i 与 app_config.h 的「看路」段。 */

/* 等看路结果帧（TAG=0x08）。
 *   成功 = status=1（Δx/Δy/左边缘经 vGetRoadResult 读; Δx/Δy 单位 0.1px）;
 *   失败 = status=0（没找到黄区 / 超时）。返回 1=成功, 0=失败/超时/被中止。
 *   与 ucRpiWaitRectResult 同构: 独立 ready 标志（不碰 last packet / RpiFrameReady）,
 *   ⚠️ 失败帧也置 ready（见 vRpiTask 那个分支）→ 失败时**立刻**返回, 不白等满 timeout。
 *   失败/超时 → 投失败音（非阻塞; 中止不响 —— 停是操作员的意图）。
 *   ⚠️ 循环里查中止位（等外部事件的循环必须带它 —— 这个工程已踩过三次）。
 *   仅任务上下文调用。 */
uint8_t ucRpiWaitRoadResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeRoadReady();                /* 清残留旧结果 */
    vSetRoadResult(0u, 0, 0, 0, 0u, 0u, 0u);
    vSetRoadDxMeas(0.0f);
    vSetRoadDyMeas(0.0f);

    while (ulWaited < ulTimeoutMs)
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("ROAD abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeRoadReady()) { continue; }
        vGetRoadResult(&ucStatus, NULL, NULL, NULL, NULL, NULL, NULL);
        if (ucStatus != 1u)
        {
            vSetBuzzerReq(BUZZER_PAT_FAIL);     /* 看路失败: 失败音 */
            return 0u;
        }
        return 1u;
    }
    vSetBuzzerReq(BUZZER_PAT_FAIL);             /* 超时（Pi 没回帧/断链）: 同样响 */
    return 0u;
}

/* 看路理想值: 把"车该停在哪儿"的两个读数写进 Beam 运行时量（与蓝牙「看路」组同一对量）。
 *
 * 用途: 理想Δx / 理想Δy 是**场地相关的标定量**（同一个停车位、不同地图/光照读数不同），
 *      做成一个积木以后，**每条路线预设可以带自己的一套** —— 换场地只改路线里的这一步，
 *      不用重编译、也不用现场掏手机开蓝牙 Tab 改。
 * ⚠️ **0（或负）= 这一项不动**：0 是"未标定"的哨兵值（`ucRpiRoadAlign` 靠它判"跳过"），
 *    所以直接写进去会把那一轴关掉；路线里只想改 Δx 时不该顺手把 Δy 也废了，故当"不改"。
 *    真要把某一轴设回"未标定"，用蓝牙 Tab 或「调参」Tab，或者让 `use_y=0`（那才是横向的开关）。
 * ⚠️ 只写 Beam、不发串口、不动底盘 → **瞬时**（设计规则第 4 条：免 _WAIT 后缀）。
 * ⚠️ 与蓝牙 Tab 是同一对量（易失，断电回 app_config.h 的 `ROAD_DX_IDEAL_PX` / `ROAD_DY_IDEAL_PX`），
 *    **谁后写谁生效** —— 现场用蓝牙调过、之后路线又跑到这一步，会以路线里的值为准。 */
void vRoadSetIdeal(float fDxIdeal, float fDyIdeal)
{
    if (fDxIdeal > 0.0f) { vSetRoadDxIdeal(fDxIdeal); }
    if (fDyIdeal > 0.0f) { vSetRoadDyIdeal(fDyIdeal); }
}

/* 看路对位: 发 "G"（ucRev≠0 时发 **"M" 反向看路**, RPI_TASK_ROAD_REV, 2026-10-31 加）
 * → 等 **Δx（黄色区左边缘的 x, 0.1px）** → 修一次**车体纵向**;
 *   ulUseY≠0 时**再用同一帧的 Δy（黄区下边缘的 y, 0.1px）** 修一次**车体横向**（2026-09-30 用户指定:**先横后纵**）。
 *
 * 物理: 车越往前开, 地上的黄区越大、左边缘越往左跑 ⇒ Δx 是一把**越近越小**的测距尺。
 *      本车相机相对车体转了 90° ⇒ 图像 y 那条轴 = 车体横向, 而"黄区下边缘的 y"就是它的尺。
 * 位移[mm] = (理想 − 实测) × 比例系数 × 方向(±1)，纵/横各一套标定量（ROAD_DX_* / ROAD_DY_*）。
 * ⚠️ **反向模式（ucRev=1; 2026-11-01 定稿"方向分轴处理"）**: Pi 侧整帧水平镜像后走同一套
 *    检测（见 Pi tasks/_road.py 文件头「反向看路」节）—— 镜像系里黄区回到右上角、量的仍是
 *    "左边缘", "越近越小"的尺子语义不变; 但车是倒着放/换边接近的, 车体补正方向要分轴:
 *    · **纵移与饱和救援乘 fRevDir(−1)**: Δx 量的是镜像边(真实右边缘), 尺子斜率相对车体
 *      前进轴翻了号 —— 同一误差该往反方向修（车倒着放, "沿路线前进" = 车体后退）。
 *    · **横移刻意不乘**: 图像竖直轴不镜像, "车体横移 → Δy 读数"的映射只由相机安装决定、
 *      与放置朝向无关; ideal 与 meas 又同取反向机位的读数, 几何自相抵消 —— 照正向公式
 *      发令恰好收敛, 乘 −1 反而往错误方向修（两帧样例 dy 正/反模式几乎同值可佐证）。
 *    位移大小/死区/钳位/road_sign/road_dy_sign 标定值都与正向**共用一份**;
 *    两把尺的理想值都按方向**分两把**——正向 road_dx_ideal/road_dy_ideal, 反向
 *    **road_dx_ideal_rev/road_dy_ideal_rev**(独立标定量, 反向停车位各标各的, 不与正向
 *    通用; 0=未标定哨兵同语义: 纵向未标→整步拒动+失败音, 横向未标→该轴跳过纵向照常)。
 *    STEP_ROAD_SET_IDEAL 写的仍是正向那对(反向那两把只走蓝牙/宏/标定Tab, 不进路线)。
 * ⚠️ **单次开环**: 本函数一次只走一步, 内部**不**做闭环循环; 要迭代就在 map.c 连排两块。
 * ⚠️ 它只动车、**不碰舵机** → 不进 c_bIsArmStep。
 * ⚠️ **纵向未标定（road_dx_ideal = 0）时仍然测量并上报 road_dx, 但一步不动 + 失败音** ——
 *    标定本身就是"读这个数", 提前 return 会没法标（鸡生蛋）。横向那一套同理（读 road_dy）。
 * ⚠️ **横向的一切异常都只打日志、不蜂鸣、也不拦纵向**（用户 2026-09-30 定）:
 *    帧里没 bit4(旧 Pi)/road_dy_ideal 未标定/已在死区内 → 纵向照做、横向跳过。
 *    失败音只留给"这一步彻底失败"（纵向自己失败）, 否则 Y 侧没标好时会一直响,
 *    操作员反而听不出真正出事的那一次。 */
static uint8_t ucRpiRoadAlignInner(uint32_t ulTimeoutMs, uint32_t ulUseY, uint32_t ulRev)
{
    uint8_t  ucStatus = 0u;
    uint8_t  ucFlags  = 0u;
    uint8_t  ucSpread = 0u;
    int16_t  nDx10 = 0, nEx = 0, nEy = 0;
    uint16_t usDy10 = 0u;
    float    fDxMeas, fErr, fMm, fMaxMm;
    float    fDyMeas, fErrY, fMmY;
    char     acDbg[RPI_DATA_MAX];
    char     acDx[12], acIdeal[12], acMm[12], acMmY[12];
    char     acDy[12], acIdealY[12];
    const emRpiTaskTdf emTask = (ulRev != 0u) ? RPI_TASK_ROAD_REV : RPI_TASK_ROAD;
    /* 反向时的车体补正方向因子（见函数头「反向模式」段）: 只进**纵向**位移与饱和救援,
     * 横移刻意不乘。日志里 ROAD fix/fixy 的 F/B、L/R 标签跟着 fMm/fMmY 的实际符号走。 */
    const float fRevDir = (ulRev != 0u) ? -1.0f : 1.0f;
    /* 纵向理想值按方向分两把: 正向 road_dx_ideal / 反向 road_dx_ideal_rev（独立标定量,
     * 读数是镜像系的, 不通用; 0 = 未标定哨兵, 语义同正向）。横向同理分两把: 反向那把
     * road_dy_ideal_rev 未标(0) → 该轴跳过、纵向照常（与正向的哨兵语义逐字一致）。 */
    const float fDxIdeal = (ulRev != 0u) ? fGetRoadDxIdealRev() : fGetRoadDxIdeal();
    const float fDyIdeal = (ulRev != 0u) ? fGetRoadDyIdealRev() : fGetRoadDyIdeal();

    uint8_t  ucCalibrated = 1u;
    if (fDxIdeal <= 0.5f) { ucCalibrated = 0u; }

    /* 发起帧用**等链路**的版本（链路不通就挂起等重连, 不在本步超时里空耗）;
     * ⚠️ 时限随指令下发（ulTimeoutMs）—— "G"/"M" 都在 Pi 的 _MS_COMMANDS 里,
     *    于是"谁先放弃"只由路线里那一步的一个数决定。 */
    if (ucRpiSendTaskCmdWait(emTask, ulTimeoutMs) == 0u)
    {
        vRpiSendDebug("ROAD fail: link down");
        return 0u;
    }

    if (ucRpiWaitRoadResult(ulTimeoutMs) == 0u)
    {
        vGetRoadResult(&ucStatus, NULL, NULL, NULL, NULL, NULL, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "ROAD fail: wait(t=%lums) st=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    vGetRoadResult(&ucStatus, &nDx10, &nEx, &nEy, &ucFlags, &ucSpread, &usDy10);
    if ((ucFlags & RPI_ROAD_FLAG_EDGE) == 0u)
    {
        /* 左边缘没量到 —— Δx 全靠它（Pi 侧正常不会发这种帧） */
        vRpiSendDebug("ROAD fail: no left edge");
        vSetBuzzerReq(BUZZER_PAT_FAIL);
        return 0u;
    }

    /* Δx 是 **Pi 量好的**（黄色区左边缘的 x, 单位 0.1px）—— 本端直接用, 不算距离。
     * 物理: 车越往前开, 黄区越大、左边缘越往左 → Δx 越小 ⇒ 它是一把测距尺。 */
    fDxMeas = (float)nDx10 / 10.0f;
    vSetRoadDxMeas(fDxMeas);            /* 蓝牙遥测 road_dx（与 Pi 日志那行应对得上） */

    /* ---------- 饱和救援（2026-10-30 加, 用户定）----------
     * 两种"尺子没数"的帧都不许拿去算位移, 改成**车体系开环后退一段 → 重测一次**:
     *   ① bit1 CLIPX = 黄区贴画面左缘（"横向全黄", Δx≈0 饱和, 量的是窗框不是黄区）。
     *      ⚠️ 这种帧是 **Pi 侧放行**来的: 老行为里 dx<ROAD_DX_MIN_PX 在 Pi 就被拒成
     *      失败帧（固件只见 status=0, 没机会救援）, 现在 _road.py 把它带旗标放行。
     *   ② bit4 没置 = 黄区下边缘被 ROI 下沿裁掉（蓝牙遥测 road_dy=0, 横向没数）。
     *      只在 use_y=1 时管 —— use_y=0 的老路线行为一字不变。
     * 两种饱和都是"车冲过头"的症状, 救援方向相同: **后退**（前进→左边缘往左跑 ⇒
     * 后退→左边缘往右移回画面; 即用户说的"向画面左移"/"退"）——
     * 车体系开环, **不叠加世界坐标系**（不碰世界航向/世界系平移）。
     * ⚠️ 方向 = **fGetRoadSign()**（与正常位移同一条：标定成能增大 Δx 的那个符号）。
     *    2026-10-30 实测**不能写死负值**：本车 road_sign=+1 时 `-fRescue` 朝篷布走
     *    （dx 304→288，越救越近）；旧注释里 "vChassisMoveCameraForward(负值)=后退"
     *    只对 road_sign=-1 成立。饱和帧的**数值**不可信，但方向符号仍由标定确定。
     * ⚠️ **单次救援**: 退一段→重测一次, 内部不循环; 还饱和就整步失败, 要迭代路线连排。
     * ⚠️ **标定态不救援**（零运动纪律）: 纵向未标定（road_dx_ideal=0, 标定就是"摆好车读数"）
     *    时不触发 —— 否则标定时车自己动了; 横向未标定（road_dy_ideal=0）时 ② 不触发 ——
     *    救援完了横向照样 skip, 白扰动纵向。
     * ⚠️ 救援关闭（road_rescue_mm=0）时: CLIPX 帧判失败（饱和值绝不进位移算式 ——
     *    老行为里这种帧根本到不了固件, 是 Pi 的 dx_min 闸门替它拒的, 语义等同）;
     *    bit4 没置则落回下面原有的 "skip (no y edge)" 老路。
     * ⚠️ 重测用**整份** ulTimeoutMs: 整步最坏 ≈ 2×超时 + 救援位移时间, 路线超时要留够。 */
    {
        uint8_t ucDxSat  = ((ucFlags & RPI_ROAD_FLAG_CLIPX) != 0u);
        uint8_t ucDyLost = ((ulUseY != 0u) && (ucCalibrated != 0u) &&
                            (fDyIdeal > 0.5f) &&
                            ((ucFlags & RPI_ROAD_FLAG_YEDGE) == 0u));
        float   fRescue  = fGetRoadRescueMm();

        if (ucDxSat && (fRescue <= 0.5f))
        {
            vRpiSendDebug("ROAD fail: dx clipped & rescue off");
            vSetBuzzerReq(BUZZER_PAT_FAIL);
            return 0u;
        }
        if ((ucDxSat || ucDyLost) && (fRescue > 0.5f) && (ucCalibrated != 0u))
        {
            bt_fmt_f2(acMm, (int)sizeof(acMm), fRescue);
            (void)snprintf(acDbg, sizeof(acDbg), "ROAD rescue: %s back %smm",
                           ucDxSat ? "dx clipped" : "dy lost", acMm);
            vRpiSendDebug(acDbg);
            /* 方向跟正常位移同一条符号（见上, 反向时再乘 fRevDir）；不要再写死负值。 */
            vChassisMoveCameraForward(fRescue * fGetRoadSign() * fRevDir,
                                      (int)iGetRoadAccel(), (int)iGetRoadSpeed());
            /* fChassisMoveDelay 只是**估时**，EMM 停稳/车身摇晃还要时间：不沉淀就重发
             * "G"，Pi 会在运动中途取帧 → 重测还是 CLIP_X（04:04 现场 40/100mm 两次
             * 都是 dx 只变 10~20px 就判 still clipped）。 */
            vTaskDelay(pdMS_TO_TICKS(ROAD_RESCUE_SETTLE_MS));

            /* 重发同一条指令再测一次（正向 "G"/反向 "M", 发起帧照旧用等链路的版本; 时限整份重给） */
            if (ucRpiSendTaskCmdWait(emTask, ulTimeoutMs) == 0u)
            {
                vRpiSendDebug("ROAD rescue fail: link down");
                return 0u;
            }
            if (ucRpiWaitRoadResult(ulTimeoutMs) == 0u)
            {
                vGetRoadResult(&ucStatus, NULL, NULL, NULL, NULL, NULL, NULL);
                (void)snprintf(acDbg, sizeof(acDbg),
                               "ROAD rescue fail: wait(t=%lums) st=%u",
                               (unsigned long)ulTimeoutMs, (unsigned)ucStatus);
                vRpiSendDebug(acDbg);
                return 0u;
            }
            vGetRoadResult(&ucStatus, &nDx10, &nEx, &nEy, &ucFlags, &ucSpread, &usDy10);
            if ((ucFlags & RPI_ROAD_FLAG_EDGE) == 0u)
            {
                vRpiSendDebug("ROAD rescue fail: no left edge");
                vSetBuzzerReq(BUZZER_PAT_FAIL);
                return 0u;
            }
            if ((ucFlags & RPI_ROAD_FLAG_CLIPX) != 0u)
            {
                /* 退了一段还贴左缘 —— 单次救援到此为止, 饱和值照旧不进算式 */
                vRpiSendDebug("ROAD fail: dx still clipped after rescue");
                vSetBuzzerReq(BUZZER_PAT_FAIL);
                return 0u;
            }
            fDxMeas = (float)nDx10 / 10.0f;
            vSetRoadDxMeas(fDxMeas);    /* 覆盖遥测: 蓝牙看到的是救援后那次 */
        }
    }

    fErr = fDxIdeal - fDxMeas;
    fMm  = fErr * fGetRoadScale() * fGetRoadSign() * fRevDir;   /* 反向: 车体补正方向取反 */

    /* 安全钳位: 误检出一个离谱的 Δx 时不许把车一次推出去撞墙（同 TARGET_MAX_STEP_DEG 的教训） */
    fMaxMm = fGetRoadMaxMm();
    if (fMm >  fMaxMm) { fMm =  fMaxMm; }
    if (fMm < -fMaxMm) { fMm = -fMaxMm; }

    /* 调试串: 浮点一律走 bt_fmt_f2（全工程不用 %f）; **每个裸数字都带单位**
     * （曾经把像素当角度读出一整轮排障事故）。三条串都 < RPI_DATA_MAX(64)。 */
    bt_fmt_f2(acDx,    (int)sizeof(acDx),    fDxMeas);
    bt_fmt_f2(acIdeal, (int)sizeof(acIdeal), fDxIdeal);
    bt_fmt_f2(acMm,    (int)sizeof(acMm),    fMm);
    (void)snprintf(acDbg, sizeof(acDbg), "ROAD meas: edge=(%d,%d) fl=0x%02X sp=%upx",
                   (int)nEx, (int)nEy, (unsigned)ucFlags, (unsigned)ucSpread);
    vRpiSendDebug(acDbg);
    (void)snprintf(acDbg, sizeof(acDbg), "ROAD dx: meas=%spx ideal=%spx", acDx, acIdeal);
    vRpiSendDebug(acDbg);

    /* 未标定（road_dx_ideal = 0）: **测量照样做、路遥测照样发, 但一步不动 + 失败音**。
     * ⚠️ 为什么要先测量再判标定（而不是提前 return）: 否则**没法标定** ——
     *    标定就是"把车摆理想停车位、读 road_dx 填进 road_dx_ideal",
     *    而提前 return 就没有 road_dx 可读（鸡生蛋）。现在台架上的用法是:
     *    摆好车 → 跑这一步 → 蓝牙 Tab 读 road_dx → 填进 road_dx_ideal → 再跑就动车了。
     * ⚠️ 仍然**一步不动**: 0 只是"没标过"的哨兵, 照它算出来的位移是凭空的位置。 */
    if (ucCalibrated == 0u)
    {
        /* 反向步点的是 road_dx_ideal_rev 那把（没标就拒绝动 + 失败音, 同"鸡生蛋"纪律:
         * 摆好车跑一步 → 读上面的 meas → 填进对应的 ideal）。 */
        vRpiSendDebug((ulRev != 0u)
                      ? "ROAD fail: road_dx_ideal_rev not calibrated (meas above)"
                      : "ROAD fail: road_dx_ideal not calibrated (meas above)");
        vSetBuzzerReq(BUZZER_PAT_FAIL);
        return 0u;
    }

    /* ---------- 先**横向**（2026-09-30 用户指定换序: 先横移再前后）----------
     * 积木参数 use_y=1 才动车; 默认 0 ⇒ 这一段整段跳过（老行为一字不变）。
     * Δy = **黄区下边缘的 y**（Pi 量好的, 单位 0.1px）, 图像 y ↔ 车体横向。
     * ⚠️ **有效判据是 flags 的 bit4（RPI_ROAD_FLAG_YEDGE）, 不是"值非 0"**:
     *    旧 Pi 把帧尾那个字段当"保留 u16"恒写 0 → 本端必须把 0 读成"没有这个量",
     *    而不是"下边缘在 y=0"（后者等于黄区贴在画面顶边, 与"黄区总在右上角"矛盾）。
     * ⚠️ **bit2（贴画面上/下边）刻意不当门**: 黄区固定在画面右上角 ⇒ 它其实是**上**边缘
     *    被顶边裁掉触发的（四个现场帧 flags 全是 0x05）, 与下边缘无关 —— 拿它当门 =
     *    横向永远不生效。
     * ⚠️ 换序只改**执行先后**（两段的位移量都是同一帧算出来的开环量, 与顺序无关）——
     *    日志顺序也跟着变成 `ROAD y:` → `ROAD fixy:` → `ROAD fix:`，排障时按这个读。 */
    if ((ucFlags & RPI_ROAD_FLAG_YEDGE) != 0u)
    {
        /* **不管有没有开 use_y 都发布** road_dy: 它是标定用的读数, 而标定横向时正好
         * 应该让 use_y=0 —— 那样车一步都不会横移（标定全程零运动 = 最安全的标定态）。
         * 这也与纵向"测量照做、只是不动车"是同一条纪律。 */
        fDyMeas = (float)usDy10 / 10.0f;
        vSetRoadDyMeas(fDyMeas);
    }

    if (ulUseY != 0u)
    {
        if ((ucFlags & RPI_ROAD_FLAG_YEDGE) == 0u)
        {
            vRpiSendDebug("ROAD fixy: skip (no y edge)");
        }
        else
        {
            bt_fmt_f2(acDy,     (int)sizeof(acDy),     fDyMeas);
            bt_fmt_f2(acIdealY, (int)sizeof(acIdealY), fDyIdeal);
            (void)snprintf(acDbg, sizeof(acDbg), "ROAD y: meas=%spx ideal=%spx",
                           acDy, acIdealY);
            vRpiSendDebug(acDbg);

            if (fDyIdeal <= 0.5f)
            {
                /* 与纵向那条同一条纪律: 0 只是"没标过"的哨兵, 照它算出来的位移是凭空的位置。
                 * ⚠️ 这里**不响**（横向没标好是常态, 一响就盖住了真正该管的失败音）。
                 * 反向步点的是 road_dy_ideal_rev 那把（没标只跳过横向, 纵向照常）。 */
                vRpiSendDebug((ulRev != 0u)
                              ? "ROAD fixy: skip (road_dy_ideal_rev not calibrated)"
                              : "ROAD fixy: skip (road_dy_ideal not calibrated)");
            }
            else
            {
                fErrY = fDyIdeal - fDyMeas;
                fMmY  = fErrY * fGetRoadScaleY() * fGetRoadDySign();   /* 反向也**不**翻(见函数头) */

                /* 与纵向**共用**同一个安全钳位（road_max_mm）—— 误检出一个离谱的 Δy 时
                 * 不许把车一次横推到撞墙（同 TARGET_MAX_STEP_DEG 的教训）。 */
                if (fMmY >  fMaxMm) { fMmY =  fMaxMm; }
                if (fMmY < -fMaxMm) { fMmY = -fMaxMm; }

                if ((fMmY <= fGetRoadDyDeadband()) && (fMmY >= -fGetRoadDyDeadband()))
                {
                    vRpiSendDebug("ROAD fixy: skip (within deadband)");
                }
                else
                {
                    /* ⚠️ **用独立缓冲 acMmY**: acMm 早在上面按 fMm 格式化过, 这里复用它会把
                     * 纵移那条日志（后面 "ROAD fix:"）的值悄悄换成横移值 —— 实测出现过
                     * "fixy: 14.10mm(R)" 与 "fix: 14.10mm(B)" 数字完全相同（2026-11-02 现场）。
                     * 动车用的是 fMm 本身、不受影响, 但日志是标定/排障的读数来源, 不能骗人。 */
                    bt_fmt_f2(acMmY, (int)sizeof(acMmY), fMmY);
                    (void)snprintf(acDbg, sizeof(acDbg), "ROAD fixy: %smm(%s)", acMmY,
                                   (fMmY > 0.0f) ? "R" : "L");
                    vRpiSendDebug(acDbg);
                    /* **车体系横移**（AXIS_X）—— 与纵向正交, 位置模式单轴走一步。
                     * ⚠️ 别改成 vChassisMoveCameraDistance: 那是"先 X 后 Y 两段"的**复合**
                     *    入口, 本步要的是**两段各自独立、可分开看日志**的单轴动作。 */
                    vChassisMoveCameraLateral(fMmY, (int)iGetRoadAccel(),
                                              (int)iGetRoadSpeed());
                }
            }
        }
    }

    /* ---------- 再**纵向**（先横后纵, 见上面那段注释）---------- */
    if ((fMm <= fGetRoadDeadband()) && (fMm >= -fGetRoadDeadband()))
    {
        vRpiSendDebug("ROAD fix: skip (within deadband)");
    }
    else
    {
        (void)snprintf(acDbg, sizeof(acDbg), "ROAD fix: %smm(%s)", acMm,
                       (fMm > 0.0f) ? "F" : "B");
        vRpiSendDebug(acDbg);
        /* **车体系纵移**（AXIS_Y, 就是 vChassisMoveCameraForward）—— 不是世界系,
         * 也不要用 vChassisMoveCameraDistance（那个的 XY 交换是反的旧约定）。 */
        vChassisMoveCameraForward(fMm, (int)iGetRoadAccel(), (int)iGetRoadSpeed());
    }
    return 1u;
}

/* 看路对位 —— **公开入口**, 只是给上面那具躯壳包一层「临时关补光」（见 app_config.h
 * 的 ROAD_LED_OFF_EN; 0 = 逐字节等同加这个开关之前）。
 *
 * ⚠️ 为什么是"包一层"而不是在 ucRpiRoadAlignInner 里逐处补一句恢复: 关灯必须在发 "G"
 *    **之前**, 而恢复必须走**唯一出口** —— 那具躯壳的提前 return 有**一大把**（链路不通 /
 *    等帧超时 / 没左边缘 / 未标定 / 饱和救援的各条失败路, 2026-10-30 起又多了几条）外加
 *    正常结束。逐处补 = 将来谁加一条出口就漏一处,
 *    症状是"补光一直灭着"（还自愈得慢: 要等下一次巡线起流才被 Pi 对齐回来）。
 *    包一层是**结构上**不漏, 且躯壳一行都不用改。
 * ⚠️ 关灯用 vRpiOpenmvLed(0)（发 "L,0"）、恢复用 vRpiOpenmvLedRestore()（发裸 "L"）
 *    —— 两个都**幂等**、都**不写** Pi 的 config, 所以"看路期间灭"不会动到配置亮度。
 *    刻意**不用** "T"（临时开关）: 那是有状态的翻转, 灯若已经是灭的, 它反而会点亮。
 * ⚠️ 关灯到发 "G" 之间可以再插一段 `ROAD_LED_OFF_DELAY_MS`（关灯后延时）: 看路档是
 *    **自动曝光**, 灯一灭 AE 要追; 不等它收敛, 那 9 帧投票窗就压在收敛瞬态里, Δx 会漂
 *    （而 road_dx_ideal 是在**开灯**下标定的）。热租约下这段等待最有用 —— 预热线程一直
 *    在读帧, 等这几百毫秒 AE 就适应完了。详见 app_config.h 那两行的备注。
 * ⚠️ 2026-10-30 加 `ROAD_SETTLE_MS` = **车体沉淀**（发 "G" 前先等，与关灯那段独立）:
 *    车刚停/刚转完弯还在晃时取帧 = 运动模糊 + Δx 漂（06:36:20 road_0007 实锤），
 *    放在本函数最前面, 两条分支（开/不开 OpenMV 灯）都吃到。 */
uint8_t ucRpiRoadAlign(uint32_t ulTimeoutMs, uint32_t ulUseY, uint32_t ulRev)
{
    /* ---- 看路前车体沉淀（2026-10-30 加, 旋钮 ROAD_SETTLE_MS）----
     * 车刚巡线停 / 刚转完弯时底盘还在晃, 相机在运动中途取帧 → 画面拖影 + Δx 逐帧漂,
     * Pi 的散布闸会把整步判失败（06:36:20 现场: road_0007 运动模糊、最近 3 帧 2px
     * 但全程 28px > 25px; 下一次 G 就成功）。等一段再发 "G"。
     * ⚠️ vTaskDelay 让出式（脚本任务上下文, 铁律 2）; **有界固定时长**, 不查中止位
     *    （中止点仍只在步骤边界, 同 STEP_DELAY）。
     * ⚠️ 与下面 ROAD_LED_OFF_DELAY_MS 是两段独立等待: 总前置 ≈ ROAD_SETTLE_MS + 400ms。 */
    if ((uint32_t)ROAD_SETTLE_MS > 0u)
    {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)ROAD_SETTLE_MS));
    }

#if (ROAD_LED_OFF_EN != 0)
    uint8_t ucRet;

    vRpiOpenmvLed(0u);              /* 关: 主摄判黄区靠 R−B 色差, OpenMV 的 LED 热点会冲淡它 */
    if ((uint32_t)ROAD_LED_OFF_DELAY_MS > 0u)
    {
        /* 让出式等待（同 STEP_DELAY 的口径, 不饿死按键/OLED/传感器任务）。
         * ⚠️ 刻意**不**查中止位: 这是**有界**的固定时长, 没有"等外部事件"的死结风险
         *    （全工程加中止位的那几处都是无界循环）—— 代价是中止点仍只在步骤边界。 */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)ROAD_LED_OFF_DELAY_MS));
    }
    ucRet = ucRpiRoadAlignInner(ulTimeoutMs, ulUseY, ulRev);
    vRpiOpenmvLedRestore();         /* 开: 唯一出口, 四条失败路径也照恢复 */
    return ucRet;
#else
    return ucRpiRoadAlignInner(ulTimeoutMs, ulUseY, ulRev);
#endif
}

/* ==================== 看路校准（黄区直边方向角 → 陀螺漂移偏置, 2026-10 加） ====================
 * 见 docs/串口协议.md §6j 与 app_config.h 的「看路校准」段。
 *
 * 物理: 黄区（比赛黄胶带 / 练习地图米黄篷布）**固定在场上**, 相机**固定在车上**
 * ⇒ 黄区直边在图像里的方向**只随车航向变**（直线方向对平移不敏感 —— 比看路那两把尺
 * Δx/Δy 干净: 那两把尺把平移和航向混在一起）。err = (ang − ideal)×方向 = 开机以来
 * 「真实航向变了多少」的视觉读数; 陀螺读数变化 = 真实变化 + 漂移 ⇒ 两者一减就是漂移。
 *
 * **每次跑都对齐到相机真值（无 mode 槽, 首次也不豁免）**:
 *   err = wrap180((ang − ideal)×方向) = 相机口径的航向真值（**"放直"= 0**）;
 *   bias = wrap180(原始工作航向 − err) ⇒ 发布 yaw = 原始 − bias ≡ err —— **每次校准后
 *   全场航向零点 = "放直"方向**, 上电摆放误差与累计漂移一次消掉。ROADYAW_APPLY_EN=1
 *   （默认）时 app_sensors 发布 yaw 统一扣偏置, PID/巡线陀螺腿/OLED 全生效。
 *
 * ⚠️ **劫持范围 = 闭环航向与显示; 世界坐标系不受影响**: vChassisMoveWorld 查表的
 *    world_heading 由 app_pid 记录时换算回原始值（fPidWorldYaw）—— 跨校准的「路径
 *    残留偏移」不会错轴。转弯 absolute 档/巡线陀螺腿的 gyro_target 在校准后坐标系里
 *    教/排。保险 = 首次定标不限幅（初始对齐量合法地大）, 之后每拍限幅
 *    ±ROADYAW_BIAS_MAX_DEG —— 某一次看错不许把闭环坐标系转飞（拒绝+保留旧值）。
 * ⚠️ 修正为何是"到点离散修"而不是"连续互补": 修偏置的时机 = 看路校准步骤（车停着,
 *    没有闭环咬航向）, 一次一步的阶跃无害; 拖成慢速斜坡反而把修正摊进行车过程里。
 * ⚠️ 基准方向角 roadyaw_ideal 出厂默认 **+90°**、方向 dir 默认 **−1**（均台架实测:
 *    放直读 +90; 陀螺正漂时读数减小 90→88）; 换机位/换边蓝牙现场改。
 * ⚠️ 未标定（roadyaw_ideal 哨兵 |值|>90, **不是 0** —— 竖直的直边 ang 本来就=0°）
 *    时**照测照报**（不响 —— 校准读数是台架常态）, 与 road_dx_ideal 同一条"鸡生蛋"纪律: 标定就是"读这个数"。
 * ⚠️ 它只读数、不碰舵机/底盘 → 不进 c_bIsArmStep。 */

/* 把角度折到 [−180, +180)（C 版 wrap180, 对齐偏置及其帧间跳变都用它） */
static float fWrap180(float fDeg)
{
    fDeg = fmodf(fDeg + 180.0f, 360.0f);
    if (fDeg < 0.0f) { fDeg += 360.0f; }
    return fDeg - 180.0f;
}

/* 等看路校准结果帧（TAG=0x09, 8B）。
 *   与 ucRpiWaitRoadResult 同构: 独立 ready 标志, 失败帧也置 ready（立刻返回, 不烧满超时）,
 *   循环里查中止位。仅任务上下文调用。 */
uint8_t ucRpiWaitRoadYawResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeRoadYawReady();             /* 清残留旧结果 */
    vSetRoadYawResult(0u, 0, 0u, 0u, 0u, 0u);
    vSetRoadYawMeas(0.0f);
    vSetRoadYawErr(0.0f);

    while (ulWaited < ulTimeoutMs)
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("ROADYAW abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeRoadYawReady()) { continue; }
        vGetRoadYawResult(&ucStatus, NULL, NULL, NULL, NULL, NULL);
        return (ucStatus == 1u) ? 1u : 0u;
    }
    return 0u;
}

/* 看路校准一步: 发 "H" → 等 ang（黄区直边图像方向角, 0.01°）→ 遥测 + 调试串 + 自动分序
 * （上电首次=记基准, 之后=校准漂移, 见段头）。返回 1=成功; 0=失败/超时/未标定/钳位拒绝
 * （都不响, 调试串留痕 —— 校准/标定流程静音）。 */
static uint8_t ucRpiRoadYawInner(uint32_t ulTimeoutMs)
{
    uint8_t  ucStatus = 0u;
    uint8_t  ucCols = 0u, ucRms10 = 0u, ucSpread = 0u, ucFlags = 0u;
    int16_t  nAng100 = 0;
    float    fAng, fErr, fRaw, fNewBias, fDelta;
    uint8_t  ucFirst;
    char     acDbg[RPI_DATA_MAX];
    char     acAng[12], acIdeal[12], acErr[12], acDrift[12], acBias[12];

    /* 发起帧用**等链路**的版本（时限随指令下发, "H" 在 Pi 的 _MS_COMMANDS 里） */
    if (ucRpiSendTaskCmdWait(RPI_TASK_ROAD_YAW, ulTimeoutMs) == 0u)
    {
        vRpiSendDebug("ROADYAW fail: link down");
        return 0u;
    }

    if (ucRpiWaitRoadYawResult(ulTimeoutMs) == 0u)
    {
        vGetRoadYawResult(&ucStatus, NULL, NULL, NULL, NULL, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "ROADYAW fail: wait(t=%lums) st=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    vGetRoadYawResult(&ucStatus, &nAng100, &ucCols, &ucRms10, &ucSpread, &ucFlags);
    if ((ucFlags & RPI_ROADYAW_FLAG_FIT) == 0u)
    {
        /* 方向角没拟合出来 —— ang 全靠它（Pi 侧正常不会发这种成功帧） */
        vRpiSendDebug("ROADYAW fail: no fit");
        return 0u;
    }

    fAng = (float)nAng100 / 100.0f;
    vSetRoadYawMeas(fAng);              /* 蓝牙遥测 roadyaw_ang（与 Pi 日志那行应对得上） */

    bt_fmt_f2(acAng,    (int)sizeof(acAng),    fAng);
    (void)snprintf(acDbg, sizeof(acDbg), "ROADYAW meas: ang=%s cols=%u rms=%u.%upx sp=%u.%02u fl=0x%02X",
                   acAng, (unsigned)ucCols, (unsigned)(ucRms10 / 10u),
                   (unsigned)(ucRms10 % 10u), (unsigned)(ucSpread / 100u),
                   (unsigned)(ucSpread % 100u), (unsigned)ucFlags);
    vRpiSendDebug(acDbg);

    /* **原始**工作航向 = 发布值 (+ 已生效偏置): ROADYAW_APPLY_EN=1 时发布值 = 原始 − 偏置。
     * 偏置数学全部锚在原始航向上。 */
#if (ROADYAW_APPLY_EN != 0)
    fRaw = fGetAttitudeYaw() + fGetRoadYawBias();
#else
    fRaw = fGetAttitudeYaw();
#endif

    /* 未标定（roadyaw_ideal 哨兵 |值|>90 —— **不是 0**, 0° 是合法角）:
     * 测量照做、遥测照发, 但不算 err（标定就是"把车摆基准位、读 roadyaw_ang 填进去"） */
    if (fGetRoadYawIdeal() > 90.0f || fGetRoadYawIdeal() < -90.0f)
    {
        vRpiSendDebug("ROADYAW fail: roadyaw_ideal not calibrated (meas above)");
        return 0u;
    }

    fErr = (fAng - fGetRoadYawIdeal()) * fGetRoadYawDir();
    /* ⚠️ 方向角是 **mod 180°** 的量（±90° 是同一朝向 —— 贴着边界的近水平直边, Pi 侧
     *    相邻两帧读数会在 +89.9 与 −89.9 之间跳）: 差值按 mod 180 折回 (−90,+90]。
     *    不折的话, 读数抖过边界一次 err 就跳 ±180°, 视觉上像"翻了半圈"、
     *    实际只有零点几度（2026-10-08 台架实测踩过投票侧的同一个坑）。 */
    fErr = fmodf(fErr + 90.0f, 180.0f);     /* → (−90, +270) */
    if (fErr < 0.0f) { fErr += 180.0f; }    /* → [0, 180) */
    fErr -= 90.0f;                          /* → [−90, +90) */
    vSetRoadYawErr(fErr);               /* 蓝牙遥测 roadyaw_err = 「真实航向变了多少」 */
    bt_fmt_f2(acIdeal,  (int)sizeof(acIdeal),  fGetRoadYawIdeal());
    bt_fmt_f2(acErr,    (int)sizeof(acErr),    fErr);
    (void)snprintf(acDbg, sizeof(acDbg), "ROADYAW err: ang=%s ideal=%s err=%sdeg",
                   acAng, acIdeal, acErr);
    vRpiSendDebug(acDbg);

    /* ---- 校准: bias = wrap180(原始工作航向 − err) ⇒ 发布 yaw = 原始 − bias ≡ err ----
     * 即**每次校准把全场航向零点对到相机基准（"放直"=0）** —— 上电摆放误差与累计
     * 漂移一并消掉, 首次也不豁免（用户 2026-10-08 拍板）。首次定标（calib 未记）
     * 不限幅（初始对齐量合法地大）; 之后每拍限幅 ±ROADYAW_BIAS_MAX_DEG: 某一次
     * 看错不许把整个坐标系转飞（拒绝 + 保留旧偏置）。mod 180 折回的理由
     * 同 err 那段（±90 边界）。 */
    fNewBias = fRaw - fErr;
    fNewBias = fmodf(fNewBias + 180.0f, 360.0f);   /* → (0, 360) 同 err 的折法 */
    if (fNewBias < 0.0f) { fNewBias += 360.0f; }
    fNewBias -= 180.0f;                            /* → [−180, +180) */
    ucFirst = (ucGetRoadYawCalibOk() == 0u);
    fDelta  = fWrap180(fNewBias - fGetRoadYawBias());
    if ((!ucFirst) &&
        ((fDelta > ROADYAW_BIAS_MAX_DEG) || (fDelta < -ROADYAW_BIAS_MAX_DEG)))
    {
        bt_fmt_f2(acDrift, (int)sizeof(acDrift), fDelta);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "ROADYAW fix: frame jump %sdeg over +/-%ddeg clamp, keep old",
                       acDrift, (int)ROADYAW_BIAS_MAX_DEG);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    vSetRoadYawBias(fNewBias);
    vSetRoadYawCalibOk(1u);
    bt_fmt_f2(acBias, (int)sizeof(acBias), fNewBias);
    (void)snprintf(acDbg, sizeof(acDbg), "ROADYAW fix: yaw->%sdeg bias=%sdeg%s",
                   acErr, acBias, (ucFirst != 0u) ? " (init)" : "");
    vRpiSendDebug(acDbg);
    return 1u;
}

/* 看路校准 —— 公开入口, 与 ucRpiRoadAlign 同一层「临时关补光」包装:
 * OpenMV 的 LED 热点同样会冲淡主摄判黄区的 Y−B 色差, 且 roadyaw_ideal 也要在**同一
 * 光照口径**下标定（与看路那步一致）, 所以照搬同一对宏（ROAD_LED_OFF_EN / 延时）。 */
uint8_t ucRpiRoadYaw(uint32_t ulTimeoutMs)
{
#if (ROAD_LED_OFF_EN != 0)
    uint8_t ucRet;

    vRpiOpenmvLed(0u);              /* 关: 主摄判黄区靠 Y−B 色差, LED 热点会冲淡它 */
    if ((uint32_t)ROAD_LED_OFF_DELAY_MS > 0u)
    {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)ROAD_LED_OFF_DELAY_MS));
    }
    ucRet = ucRpiRoadYawInner(ulTimeoutMs);
    vRpiOpenmvLedRestore();         /* 开: 唯一出口, 各失败路径也照恢复 */
    return ucRet;
#else
    return ucRpiRoadYawInner(ulTimeoutMs);
#endif
}

/* ==================== 色球识别（组合动作） ==================== */

/*
 * 色球顺序识别一步到位（脚本动作 STEP_COLOR_SCAN_WAIT 的原语）:
 *   发 RPI_TASK_COLORBALL 指令 → 阻塞等结果帧 → 失败音（失败/超时各投一声, 成功不响）。
 *   结果在 Beam（vGetColorResult 读: 状态 + 三球颜色/中心坐标, 已按图像从左到右排序）。
 *   ⚠️ 色球失败音在链内会被 ucRpiAlignChain 压制（s_ucAlignBeepHold, 链统一管）。
 *   ⚠️ 识别期间小车必须静止（Pi 端多帧投票要求同一机位）。仅任务上下文调用。
 */
uint8_t ucRpiColorScan(uint32_t ulTimeoutMs)
{
    uint8_t ucOk;

    if (ucRpiSendTaskCmdWait(RPI_TASK_COLORBALL, ulTimeoutMs) == 0u)   /* 时限随指令下发: Pi 减余量后当自己的预算 */
    {
        return 0u;
    }
    ucOk = ucRpiWaitColorResult(ulTimeoutMs);
    return ucOk;
}

/* ==================== 巡线收敛（组合动作） ==================== */

/*
 * 巡线收敛闭环一步到位（脚本动作 STEP_LINE_CONVERGE_WAIT 的原语）:
 *   发 RPI_TASK_LINE_START → 等 Pi 起流 → 闭环横移收敛 → 发 RPI_TASK_LINE_STOP 收尾
 *   → （成败都不响 —— 巡线这套不在 2026-10-27 的失败音清单里, 调试串留痕）。
 *   "起流"判据用**巡线帧计数变化**而不是 valid: 帧在流但丢线时 valid=0,
 *   那属于闭环正常会遇到的状态（vLineFollowConverge 会停车保持等恢复）,
 *   不能当"没起来"处理。
 *   ⚠️ Pi 没起流（没上电 / 没收到 LINE_START）时**不进闭环直接返回** ——
 *     否则 vLineFollowConverge 拿着 valid=0 的陈旧快照空跑满 converge_ms,
 *     表现为"车不动也不报错", 是最难查的失效模式。
 *   返回 1=Pi 已起流且闭环跑完, 0=Pi 未起流（车未动）。仅任务上下文调用。
 */
/* 公共实现: ucSrc = 这一步的**航向来源**（常量见 app_rpi.h 的 LINE_ANG_SRC_*:
 * 单源三选一, 或 3=融合; 单源档仍是"一刀切只认一个, 不换源不补权重"）。起流/等帧/收尾三段与原实现完全一致; ucSrc 只是透传给内层闭环。
 * iFwdRpm / iAccel = 内层闭环的前进分量与斜坡加速度（2026-10 巡线直走加）:
 * 巡线收敛走 (0, iGetChassisAccel()) = 老行为逐字节不变; 直走走 (积木速度, 积木加速度)。
 * ucEarlyRet = 内层闭环的「启动即返回」开关（2026-10 加, 判据见 app_config.h
 * 「巡线收敛增强」区）: 巡线收敛按积木参数透传; 直走恒 0 —— 它的时长=要走多远,
 * 提前退出=少走（vLineFollowConverge 函数头有记）。
 * ulSettleMs = **起步对准段**（2026-10-05 加, 照搬支点转弯 ④ 的结构）: 起流门过后、
 * 沿线走之前, 先原地收敛 ulSettleMs（iFwdRpm=0 → 用「原地对准」增益组）—— 把起步时
 * 的横向偏差与歪头先修掉, 沿线段从"已摆正"的状态起跑。⚠️ 直走没有这段时, 车从摆歪
 * 状态直接以全速前进, 修正追不上发散（实测「直走不纠偏」的根因; 支点接管好看正是因为
 * 转弯+settle 已把车摆正）。0=不对准（老行为）。⚠️ 对准段结束的 vChassisMoveA(0,0,0)
 * 会被沿线段第一拍立刻覆盖（微秒级, 同支点转弯 settle→line 的衔接, 不停车）。 */
static uint8_t ucRpiLineRun(uint32_t ulConvergeMs, uint8_t ucSrc, int iFwdRpm, int iAccel,
                            uint8_t ucEarlyRet, uint32_t ulSettleMs)
{
    uint32_t ulFrame0 = ulGetLineFrameCount();      /* 起流前基线（帧计数） */
    uint32_t ulWaited = 0;

    /* ⚠️ **起流帧要等链路**（等不到就没必要往下走 —— 下面那道起流门会白等
     * LINE_FIRST_FRAME_TIMEOUT_MS）; 而收尾的 LINE_STOP 恰恰相反, 用不带链路的
     * vRpiSendTaskCmd: 它最需要发出去的时刻就是链路不可靠时, 门控它会让 Pi
     * 一直跑着 30Hz 巡线流。 */
    if (ucRpiSendTaskCmdWait(RPI_TASK_LINE_START, 0u) == 0u)
    {
        return 0u;
    }

    while ((ulGetLineFrameCount() == ulFrame0) && (ulWaited < LINE_FIRST_FRAME_TIMEOUT_MS))
    {
        /* ⚠️ 查中止位: 这是"等外部事件"的循环（最长 2s）, 铁律要求带上它 ——
         * 本步的主闭环本来就有, 这里是脚本路径上最后一处漏网。
         * 中止按"起流失败"同一条出口收尾（补发 LINE_STOP 让 Pi 收流, 不蜂鸣: 按停是意图）。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
            vRpiSendDebug("LINE abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;
    }

    /* 仍未起流: 兜底补发停止指令, 不动车（不响, 调试串留痕） */
    if ((ulGetLineFrameCount() == ulFrame0) && (LINE_FIRST_FRAME_TIMEOUT_MS != 0u))
    {
        vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
        return 0;
    }

    /* 起步对准段（ulSettleMs>0 才跑, 照搬支点转弯 ④ 的 settle→line 结构, 见函数头）:
     * 原地收敛（iFwdRpm=0 → 用「原地对准」增益组）, 把起步偏差/歪头先修掉再沿线走。
     * ⚠️ 放在起流门**之后**: 对准要吃新鲜帧, 门只等帧计数、可能放进盲期帧 —— 盲期里
     *    对准拍会空转（v=0, vx=0）, OpenMV 起流的固有开销, 与支点转弯"转弯吃掉盲期"同理。 */
    if (ulSettleMs != 0u)
    {
        vLineFollowConverge(ulSettleMs, ucSrc, 0, (int)iGetChassisAccel(), 0u);
    }

    /* iFwdRpm=0: 老语义「对准」—— 只横移+转正, 不往前走（`STEP_LINE_CONVERGE_WAIT`
     * 的行为与加这个参数之前逐字节一致, 别给它加前进分量）。 */
    vLineFollowConverge(ulConvergeMs, ucSrc, iFwdRpm, iAccel, ucEarlyRet);   /* 内部阻塞, 结束/超时自会停车 */
    vRpiSendTaskCmd(RPI_TASK_LINE_STOP);            /* 收尾: 免得 Pi 继续 30Hz 空发 */
    return 1;
}

/* 校验「航向来源 + 陀螺目标角 + 融合占比」（共用: 巡线收敛 / 支点转弯接管 / 巡线直走）。
 * 返回**实际使用**的档位, 并把合法的目标角/占比写进 Beam。
 *
 * ⚠️ 口径（改这里就是改所有带 src 的积木）:
 *   · 认不出来的档位（手改 map.c）或陀螺目标角越界(±180 之外) → 返回 NONE(不修航向),
 *     且**不把脏值写进 Beam**。刻意**不**退回相机: 「笔误就偷偷换个源」正是要消灭的行为 ——
 *     可见性就是调试串 + 自述串里的 `src=none`。
 *   · **src=陀螺/融合 两档都写 `line_gyro_target`**（2026-10-08 融合档定案: 融合的陀螺腿
 *     咬的就是积木给的 angle, 不写的话它咬的是蓝牙组当前值/上电默认 —— 完全无关的航向会把
 *     40% 的修正拉去错误方向）。CAM/NONE 档里那个角仍完全不参与, 写了离谱值不生效也不污染。
 *   · **只在 src=融合 时才碰 `line_cam_pct`**（2026-10-08 加）: ucCamPct=KEEP(0xFF) = 不写
 *     （没有占比槽的调用方都传它）; 0~100 = 钳位后写进 Beam 当这一步的起点 —— 钳位不响:
 *     越界的占比钳回端点仍是合法档位（0=全陀螺/100=全相机）, 与蓝牙 $B,SET 走 ucBeamApply
 *     的钳位同一口径; 这与目标角越界(±180 外是无处可钳的废值)必须拒绝是两回事。
 *   · 目标角/占比都写一次当这一步的起点: 之后的蓝牙覆盖能当场生效, 而下一块积木又会把
 *     自己的值写回来 —— 于是既没有"上一段漏到下一段"的粘性, 又能现场标定。
 * `pcTag` = 调试串前缀（"LINE" 巡线收敛 / "PVL" 支点转弯接管 / "LW" 巡线直走）, 日志里好区分。
 * ⚠️ 不响"降级成功"这类静默路径 —— 档位改坏也只是拒绝+留痕（vRpiSendDebug）。 */
static uint8_t ucRpiLineSrcResolve(uint8_t ucSrc, float fGyroTarget, uint8_t ucCamPct,
                                   const char *pcTag)
{
    char acDbg[RPI_DATA_MAX];

    if ((ucSrc != LINE_ANG_SRC_CAM) && (ucSrc != LINE_ANG_SRC_GYRO)
        && (ucSrc != LINE_ANG_SRC_NONE) && (ucSrc != LINE_ANG_SRC_FUSE))
    {
        (void)snprintf(acDbg, sizeof(acDbg), "%s src: unknown, heading loop off", pcTag);
        vRpiSendDebug(acDbg);
        return LINE_ANG_SRC_NONE;
    }

    if ((ucSrc == LINE_ANG_SRC_GYRO) || (ucSrc == LINE_ANG_SRC_FUSE))
    {
        /* 陀螺/融合两档的陀螺腿都咬这个目标角; 越界(±180 外是世界航向的物理量程之外)
         * = 笔误 → 整步不修航向（融合档也一刀切掐掉, 刻意不"只用相机腿":
         * 静默降级正是要消灭的行为, 可见性就是调试串 + 自述串 src=none）。 */
        if ((fGyroTarget > 180.0f) || (fGyroTarget < -180.0f))
        {
            (void)snprintf(acDbg, sizeof(acDbg), "%s tgt: out of range, heading loop off", pcTag);
            vRpiSendDebug(acDbg);
            return LINE_ANG_SRC_NONE;
        }
        vSetLineGyroTarget(fGyroTarget);
    }

    if ((ucSrc == LINE_ANG_SRC_FUSE) && (ucCamPct != LINE_CAM_PCT_KEEP))
    {
        float fPct = (float)ucCamPct;

        if (fPct > 100.0f) { fPct = 100.0f; }      /* uint8_t 不会 <0, 只钳上界 */
        vSetLineCamPct(fPct);
    }

    return ucSrc;
}

/* 巡线类步骤收尾: 更新**世界航向累加器**（90° 逻辑网格）。
 *   陀螺档 → 写这一步的**指令目标**（fGyroTarget）—— 世界系是排路线用的账本
 *            （只决定 X/Y 与正负），不是误差处理通道；写实测会把航向环的残差
 *            渗进网格（同 app_pid.c「永远按指令目标写」的原则）。
 *   相机/融合 → 保持实测：线可能把车转到任意朝向，目标不是世界航向。
 *   不修 → 不写（本来就没修航向，保持上一个指令目标）。
 * ⚠️ fGyroTarget 只在 GYRO 档被采信；CAM/FUSE/NONE 档传了也不参与。
 * ⚠️ 只在真跑完巡线时调用（中止 / 没起流不写，与各调用方的收尾同口径）。 */
static void vLineStepSetWorldHeading(uint8_t ucUse, float fGyroTarget)
{
    if (ucUse == LINE_ANG_SRC_GYRO)
    {
        float fHeading = fGyroTarget;

        while (fHeading > 180.0f)       { fHeading -= 360.0f; }
        while (fHeading < -180.0f)      { fHeading += 360.0f; }
        vSetWorldHeading(fHeading);
    }
    else if (ucUse != LINE_ANG_SRC_NONE)
    {
        vSetWorldHeading(fGetAttitudeYaw());    /* CAM/FUSE: 线决定朝向 */
    }
}

/* 位置 + 航向（常规巡线）。ucSrc = 这一步**听谁**（常量见 app_rpi.h 的 LINE_ANG_SRC_*,
 * 含融合档）; fGyroTarget 在选陀螺/融合时参与（融合档 = 陀螺腿咬的目标）,
 * ucCamPct 只在选融合时参与（KEEP=沿用当前值）。
 * 取值校验见上面的 ucRpiLineSrcResolve。
 * ucEarlyRet = 积木的「启动即返回」: 1 = 按「巡线收敛增强」判据达标即提前返回 + 没达标给
 * LINE_CONV_OVERTIME_MS 超时收敛; 0（老行缺槽的默认值）= 跑满 converge_ms, 逐字节老行为。
 * ⚠️ 前进分量恒 0（「对准」不前进）+ 斜坡加速度取 Beam 全局量 = 老行为（2026-10 拆参数前）。 */
uint8_t ucRpiLineConverge(uint32_t ulConvergeMs, uint8_t ucSrc, float fGyroTarget,
                          uint8_t ucEarlyRet, uint8_t ucCamPct)
{
    uint8_t ucUse = ucRpiLineSrcResolve(ucSrc, fGyroTarget, ucCamPct, "LINE");
    uint8_t ucR   = ucRpiLineRun(ulConvergeMs, ucUse,
                                 0, (int)iGetChassisAccel(), ucEarlyRet, 0u);

    /* 世界航向账本收尾: **只补陀螺档**（写指令目标, 不写实测, 见 vLineStepSetWorldHeading）。
     * 本函数原来完全不写世界航向 —— CAM/FUSE/NONE 保持老行为, 不在这里新增实测写入。
     * 中止/没起流（ucR=0）不写 —— 这一步没跑完, 账本保持上一个指令目标。 */
    if ((ucR != 0u) && (ucUse == LINE_ANG_SRC_GYRO))
    {
        vLineStepSetWorldHeading(ucUse, fGetLineGyroTarget());
    }
    return ucR;
}

/* ==================== 巡线直走（组合动作, 2026-10 加） ==================== */

/* 步末自述串（每步一条, 进 Pi 日志的 [STM-DBG]）:
 *   d = 实际执行的距离[mm]（use_res 吃过残留之后）  t = 闭环时长[ms]（估时 × k）
 *   k = 时间系数（Beam line_time_scale, 蓝牙可改）  r = 0 没走 | 1 走完
 * ⚠️ 与内层闭环的 `LINE:` 互补: 那条管"这一拍在修什么", 这条管"为什么走了这么远" ——
 *    落点偏了先看 k= 该不该调（走过了调小、没走到调大）, 再谈巡线环。
 * ⚠️ RPI_DATA_MAX=64 是**静默截断**, 这条最长约 38 字符, 别再加字段。 */
static void vLineWalkDbg(float fDistMm, float fScale, uint32_t ulMs, uint8_t ucR)
{
    char acDbg[RPI_DATA_MAX];
    char acD[12];
    char acK[12];

    bt_fmt_f2(acD, (int)sizeof(acD), fDistMm);
    bt_fmt_f2(acK, (int)sizeof(acK), fScale);
    (void)snprintf(acDbg, sizeof(acDbg), "LW: d=%smm t=%ums k=%s r=%u",
                   acD, (unsigned)ulMs, acK, (unsigned)ucR);
    vRpiSendDebug(acDbg);
}

/* use_res 吃残留的对账串（只在该档发）: 一次性, 排障先对 path_res_x/y 遥测。
 *   res = 消费到的世界系残留[mm]   body = 折到车体的 (side, fwd)
 * ⚠️ 本积木只执行 fwd: side 分量**丢弃并在这里报出来**（路线上那步抓取若横向挪过,
 *    横向本就该由后面的巡线收敛修, 拿到这里重复补偿反而错）。 */
static void vLineWalkResDbg(float fResWx, float fResWy, float fSide, float fFwd)
{
    char acDbg[RPI_DATA_MAX];
    char acX[12];
    char acY[12];
    char acS[12];
    char acF[12];

    bt_fmt_f2(acX, (int)sizeof(acX), fResWx);
    bt_fmt_f2(acY, (int)sizeof(acY), fResWy);
    bt_fmt_f2(acS, (int)sizeof(acS), fSide);
    bt_fmt_f2(acF, (int)sizeof(acF), fFwd);
    (void)snprintf(acDbg, sizeof(acDbg), "LW2: res=(%s,%s) body=(%s,%s)",
                   acX, acY, acS, acF);
    vRpiSendDebug(acDbg);
}

/*
 * STEP_LINE_STRAIGHT_WAIT（巡线直走）的原语: 沿线**边前进边修**走固定距离 ——
 * 「巡线收敛」只对准不前进、「支点转弯接管」的沿线段时长靠人拍脑袋, 这一步把
 * 时长换成**位置模式同款估时**从 mm 反推, 且前进速度/加速度都由积木参数给。
 *
 *   ① 参数防呆: dist_mm / speed / accel 任一 ≤ 0（新拖出的积木参数全是 0 = 没配齐）
 *      → 不动车 + 调试串留痕, **连巡线流都不起**（别静默跑一段没意义的动作;
 *      accel=0 若放过去会被 EMM 当"直接启动"、而估时把它当 256, 两者劈开）。
 *      ⚠️ 防呆在 use_res **之前**: 没配齐的积木不去消费残留（吃了白吃, 残留被清零）。
 *   ② accel 钳到 1~255: EMM 的加速度是**单字节档位**, >255 截断会让"估时的加速度"
 *      与"实际斜坡的加速度"不再是同一个值。
 *   ③ use_res=1 时吃「路径残留偏移」（人质单动作抓取留下的世界系 mm, 读出即清零）:
 *      残留经 vChassisWorldToBodyEx 折到车体（与 vChassisMoveWorld 的 use_res **同一张
 *      象限表**）, **fwd 分量并入 fDistMm**（计划 − 残留: 车已经多走多少就少走多少;
 *      抓取时后撤过则表现为多走）—— 与世界系平移积木的"叠加上次残留偏移"同一语义,
 *      区别只在折算后的 side 分量本积木不走（丢弃并在 LW2 串里报出来, 横向留给巡线环）。
 *      ⚠️ 折算读 fGetWorldHeading()（航位推算量, 与世界系平移同口径）; 本步收尾又会
 *      按**源**写航向（陀螺=指令目标, 相机/融合=实测）—— 先折残留、后写航向, 顺序别反。
 *   ④ ucRpiLineSrcResolve: src 档位同「巡线收敛」（相机/陀螺/不修/融合）, 越界/笔误拒绝。
 *   ⑤ mm → ms: fChassisMoveTimeEst(dist, accel, speed, AXIS_Y) × **时间系数 k**
 *      （Beam line_time_scale, 蓝牙「巡线」组可改）—— 与位置模式**同一条梯形估时公式**,
 *      轴取 AXIS_Y（体坐标系前进, 同轴同脉冲系数）。⚠️ 公式搬到速度模式斜坡上带系统
 *      偏差（实测偏长）, k 就是那把现场标定的尺: 走过了调小(<1)、没走到调大(>1)。
 *      ⚠️ 速度模式斜坡与位置模式共用同一个 EMM 加速度档位, 公式才迁移得过来 ——
 *      这也是 ② 必须钳位、vChassisMoveA 必须存在的原因。
 *   ⑥ ucRpiLineRun(ulMs, src, speed, accel): 起流 → 等起流 → 沿线走 → 收流,
 *      与「巡线收敛」同一条龙、同一套收尾。
 *   ⑦ 走完按**实测**补写世界航向（w 环可能把车转了, 同 ucRpiPivotTurnLine 的收尾;
 *      只有真跑过才写 —— 跳过的路径车没动过）。
 *
 * ⚠️ **实际时长取整到 LINE_LOOP_PERIOD_MS(20ms) 的整数倍**（闭环逐拍重发, 每拍多走
 *    一段）, 平均比(估时×k)多约半拍 —— 落点偏差全折进 k 现场标。dist ≤ 2mm 会吃估时
 *    函数的 200ms 地板（那点距离全耗在启动上, 位置模式同样如此）—— 别拿这块走毫米级小步。
 * ⚠️ 前提是**这一段看得到线**: 看不到时 Pi 按它的定向找线策略大横移（不是本步要的）;
 *    LINE_CONVERGE_SAMPLES 必须保持 0, 它的提前退出在这里就是**截断行程**。
 * ⚠️ **起步对准段（2026-10-05 加, 照搬支点转弯 ④ 的结构）**: ulSettleMs>0 时起流门过后
 *    先原地收敛再沿线走 —— 直走没有对准段时, 车从摆歪状态直接全速前进, 修正追不上发散
 *    （实测「直走不纠偏」; 支点接管好看正是因为转弯+settle 已把车摆正）。对准段用
 *    「原地对准」增益组（iFwdRpm=0）, 时长独立于沿线窗。
 * 返回 1 = 走完; 0 = 没走（参数没填 / 链路不通 / 没起流）。仅任务上下文调用。
 */
uint8_t ucRpiLineStraightWalk(float fDistMm, int iAccel, int iSpeed,
                              uint8_t ucSrc, float fGyroTarget, uint8_t ucUseRes,
                              uint32_t ulSettleMs, uint8_t ucCamPct)
{
    uint32_t ulMs;
    float    fScale;
    uint8_t  ucUse;
    uint8_t  ucR;

    if ((fDistMm <= 0.0f) || (iSpeed <= 0) || (iAccel <= 0))
    {
        vLineWalkDbg(fDistMm, fGetLineTimeScale(), 0u, 0u);
        return 0u;
    }
    if (iAccel > 255) { iAccel = 255; }               /* EMM 单字节档位, 见函数头 ② */

    /* use_res: 计划 − 残留（车体前向分量）, 见函数头 ③。折算要在估时**之前** ——
     * 距离变了估时才对; 读的航向是"本步执行前"的累加值, 与世界系平移同口径。 */
    if (ucUseRes != 0u)
    {
        float fResWx, fResWy;
        float fSide,  fFwd;

        vConsumePathResidual(&fResWx, &fResWy);
        vChassisWorldToBodyEx(fResWx, fResWy, &fSide, &fFwd);
        vLineWalkResDbg(fResWx, fResWy, fSide, fFwd);
        fDistMm -= fFwd;

        /* ⚠️ 吃完残留后计划距离可能 ≤0: 残留的前向分量已经把本步计划**吃光甚至超过**
         * —— 车现在的位置已经在/超过了本步要到的位置, 本步**一步都不该再走**。
         * 原来直接把这个负数喂给估时, 而 fChassisMoveTimeEst 内部取 ABS → 得到一个**正的
         * 时长** → 闭环按 speed 朝**前**再走 |d|（本该后退 |d|）→ 落点偏差翻倍,
         * 而 LW: 里那个负数不明显。
         * 只对 use_res 这条生效: use_res=0 时距离仍是入参的那个正数, 行为逐字节不变。
         * 残留是"读出即清零"的一次性量（vConsumePathResidual 已消费）, 不必回写。 */
        if (fDistMm <= 2.0f)
        {
            vRpiSendDebug("LW skip: residual covers dist");
            vLineWalkDbg(fDistMm, fGetLineTimeScale(), 0u, 1u);
            return 1u;      /* 「走完了」—— 位置已由残留带到, 调用方按完成处理 */
        }
    }

    ucUse = ucRpiLineSrcResolve(ucSrc, fGyroTarget, ucCamPct, "LW");

    fScale = fGetLineTimeScale();
    ulMs   = (uint32_t)(fChassisMoveTimeEst(fDistMm, (uint16_t)iAccel,
                                            (uint16_t)iSpeed, AXIS_Y) * fScale);

    ucR = ucRpiLineRun(ulMs, ucUse, iSpeed, iAccel, 0u, ulSettleMs);   /* 直走不吃「启动即返回」; ulSettleMs=起步对准段（照搬支点转弯 settle→line 结构） */

    if (ucR != 0u)
    {
        /* 世界航向账本收尾: 陀螺档写指令目标, 不写实测（见 vLineStepSetWorldHeading 的注释）。
         * 传 fGetLineGyroTarget() 而不是入参 fGyroTarget: 蓝牙可能在跑动中覆盖目标,
         * 账本要跟**闭环最后一拍实际咬的目标**一致。 */
        vLineStepSetWorldHeading(ucUse, fGetLineGyroTarget());
    }

    vLineWalkDbg(fDistMm, fScale, ulMs, ucR);
    return ucR;
}

/* ==================== 支点转弯 → 巡线接管（组合动作） ==================== */

/* 自 xT0 起的毫秒数（configTICK_RATE_HZ=1000 时 1 tick = 1ms; 乘 portTICK_PERIOD_MS 与配置解耦） */
static uint32_t ulElapsedMs(TickType_t xT0)
{
    return (uint32_t)((xTaskGetTickCount() - xT0) * portTICK_PERIOD_MS);
}

/*
 * 步末自述串（每步一条, 进 Pi 日志的 [STM-DBG]）:
 *   r = 0 链路不通跳过巡线 | 1 没起流跳过巡线 | 2 交接成功 | 3 转完再巡线 | 4 被中止
 *   t = 本步总耗时[ms]（与老的两块积木比这个数, 就是"到底省没省"的证据）
 * ⚠️ 本串**必须留着**（与 app_line.c 的 `LINE:` 串互补, 不是重复）: 本步的巡线段一律强制
 *    听谁由积木的 `src` 决定（`LINE:` 会照实打那个档位）; 而"**为什么没走巡线**"（链路不通 / 没起流 /
 *    被中止）**只有**本串的 `r=` 说得清, 总耗时也只有 `t=` 有。
 * ⚠️ RPI_DATA_MAX=64 是**静默截断**, 这条最长约 22 字符, 别再加字段。
 * ⚠️ 不在这里带巡线段的 `line_err`/`line_valid`: 那些用蓝牙 Tab 的只读遥测看。
 * ⚠️ pcTag 分流: 支点版打 "PVL"、自转版打 "SPIN" —— Pi 日志里一眼分得清是谁在跑
 *    （与 HSTG/BALL 分前缀的同一原则）。
 */
static void vTurnLineDbg(const char *pcTag, uint8_t ucR, uint32_t ulMs)
{
    char acDbg[RPI_DATA_MAX];

    (void)snprintf(acDbg, sizeof(acDbg), "%s: r=%u t=%ums", pcTag,
                   (unsigned)ucR, (unsigned)ulMs);
    vRpiSendDebug(acDbg);
}

/*
 * 体坐标系**盲走** ulMs（交接前直行 / 尾段盲走共用; 支点与自转两条组合动作都走它）。
 *
 * ⚠️ 走 `vChassisMove` 的 **v_y（体坐标系）**, **不走 `vChassisMoveWorld` 的变换** ——
 *    那要求航向累加器是准的, 而交接那一刻它刚按"实测"写过, 本就不该被进一步依赖。
 * ⚠️ 每拍**重发**一次速度指令: 防被别的路径的停车指令覆盖（与支点转弯 / 巡线环同一种写法）。
 *    节拍沿用 PIVOT_TURN_PERIOD_MS（同一套底盘闭环节奏, 50ms）→ **实际时长是它的整数倍**,
 *    比 ulMs 略长一点是正常的（与转弯/巡线两段同性质）。
 * ⚠️ 结束/中止**都必须停车**: 速度模式指令是持久的, 不停就会一直往前跑。
 * ⚠️ 中止检查是必需的（「凡是等外部事件的循环都要带中止位」那条铁的纪律）——
 *    没有它, 长尾段期间按停止键会按不动。
 * 返回 1 = 走完; 0 = 被中止（两个出口都已停车, 调用方只管收尾）。
 * 仅任务上下文调用（内部 vTaskDelay）。
 */
static uint8_t ucTurnLineBlindDrive(uint32_t ulMs)
{
    uint32_t ulElapsed = 0;

    while (ulElapsed < ulMs)
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            vChassisMove(0, 0, 0);
            return 0u;
        }
        vChassisMove(0, (int)PIVOT_LINE_FWD_SPEED, 0);
        vTaskDelay(pdMS_TO_TICKS(PIVOT_TURN_PERIOD_MS));
        ulElapsed += PIVOT_TURN_PERIOD_MS;
    }
    vChassisMove(0, 0, 0);
    return 1u;
}

/*
 * **转弯之后的整条尾段**（②′盲走 → ③补等起流 → ④巡线两段 → ⑤尾段盲走 → 世界航向收尾）
 * —— 2026-10-30 从 ucRpiPivotTurnLine 里抽出来: 支点版与自转版（ucRpiSpinTurnLine）的
 * 交接后流程**逐字节相同**（巡线接管后底盘归巡线闭环管, 与"刚才谁转的弯"无关）,
 * 抽成一份才不会两条路改着改着漂移。转弯本体（两个入口/武装门/交接判据）仍在 app_pid.c。
 *
 * 入口约定: 调用方已发 LINE_START、已跑完转弯（vPidPivotTurnUntil / vPidSpinTurnUntil）、
 * 已查过中止位。ulFrame0 = 发 LINE_START 前取的帧计数基线（③判"起流没有"用）。
 * 返回 1 = 巡线段跑过（收尾已按源写世界航向: 陀螺=指令目标, 相机/融合=实测）;
 * 0 = 中止/没起流（r=4/1）。
 * r 码: 1 没起流 | 2 交接成功 | 3 转完再巡线 | 4 被中止（0 只出现在调用方的链路不通分支）。
 */
static uint8_t s_ucTurnLineTail(uint32_t ulTick0, uint32_t ulFrame0, uint8_t ucHandoff,
                                uint32_t ulFwdMs, uint32_t ulSettleMs, uint32_t ulExtraLineMs,
                                uint32_t ulTailMs, uint8_t ucUse, uint8_t ucSettleEarly,
                                const char *pcTag)
{
    /* ②′ 交接前**先按体坐标系直行一段**（ulFwdMs=0 则整段跳过）。
     * 为什么走体坐标系、为什么结束要停车 —— 两条都在 ucTurnLineBlindDrive 的函数头, 别改。
     * 循环体（含中止位）与 ⑤ 尾段**共用** `ucTurnLineBlindDrive`。 */
    if (ulFwdMs != 0u)
    {
        if (ucTurnLineBlindDrive(ulFwdMs) == 0u)  /* 中止: 收尾帧照发（Pi 可能正在推流） */
        {
            vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
            vTurnLineDbg(pcTag, 4u, ulElapsedMs(ulTick0));
            return 0u;
        }
    }

    /* ③ 转弯跑完还没起流 → 补等到**截止时刻**（不是再等一个满窗）。
     *    于是总时长上界 = max(转弯, 起流超时), 而不是两者相加。 */
    while ((ulGetLineFrameCount() == ulFrame0)
           && ((xTaskGetTickCount() - ulTick0)
               < (TickType_t)pdMS_TO_TICKS(LINE_FIRST_FRAME_TIMEOUT_MS)))
    {
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
            vTurnLineDbg(pcTag, 4u, ulElapsedMs(ulTick0));
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* 仍没起流: 收尾帧照发, 车**不动**（转弯已经做完了, 不重复转; 不响, 留痕）。
     * 与 ucRpiLineRun 同形, 含 LINE_FIRST_FRAME_TIMEOUT_MS=0 的旁路（=不等, 直接进闭环）。 */
    if ((ulGetLineFrameCount() == ulFrame0) && (LINE_FIRST_FRAME_TIMEOUT_MS != 0u))
    {
        vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
        vTurnLineDbg(pcTag, 1u, ulElapsedMs(ulTick0));
        return 0u;
    }

    /* ④ 巡线接管: 位置 + 航向（交接的语义就是"陀螺退出、听线", 而转弯刚结束时也正是
     *    最需要修航向的时候）。
     * **分两段,**与积木的两个时长参数一一对应:
     *   ulSettleMs   = 只对准（横移 + 转正, **不前进**）—— 把交接那一刻的横/纵偏差收掉;
     *   ulExtraLineMs= 沿线走（再叠加前进分量, 边修边走）。
     * 两段是同一个内层闭环连调两次: 第一段结尾的 vChassisMove(0,0,0) 会被第二段进入时的
     * **第一拍立刻覆盖**（中间只隔几次读操作, 微秒级）, 所以看不出停顿, 也没有"停稳再起步"。
     * ⚠️ 两段之和为 0 时**也要跑一拍**: 交接是**不停车**的, 不给巡线一次机会就等于把转弯的
     *    残留转速留在那儿（没人覆盖它）。所以这里把 settle 兜底成 LINE_LOOP_PERIOD_MS。
     * ⚠️ 别把全局的 LINE_CONVERGE_SAMPLES 调大: 它的提前退出会按"对准了"**截断沿线走的行程**。 */
    if ((ulSettleMs + ulExtraLineMs) == 0u) { ulSettleMs = LINE_LOOP_PERIOD_MS; }

    if (ulSettleMs != 0u)
    {
        vLineFollowConverge(ulSettleMs, ucUse, 0,
                            (int)iGetChassisAccel(), ucSettleEarly);   /* 只对准, 不往前走; settle_early=1 时按「巡线收敛增强」判据提前收段 */
    }
    if (ulExtraLineMs != 0u)
    {
        vLineFollowConverge(ulExtraLineMs, ucUse,
                            (int)PIVOT_LINE_FWD_SPEED,
                            (int)iGetChassisAccel(), 0u);       /* 沿线走; 时长=要走多远, 恒不吃提前退出 */
    }

    vRpiSendTaskCmd(RPI_TASK_LINE_STOP);       /* 收尾: 免得 Pi 继续 30Hz 空发 */

    /* ⑤ **尾段盲走**（ulTailMs=0 则整段跳过）: 沿线走跑完、收流之后再往前顶一段
     * （到点 / 进区这种"线到头了还得再走一点"的场合）。与 ②′ 是同一种动作、同一个速度宏,
     * 细节（为什么体坐标系 / 为什么停车 / 为什么排在这里）见 ucTurnLineBlindDrive 的函数头。
     * ⚠️ 中止时**不必**再发 LINE_STOP —— 上面那一帧刚发过。 */
    if (ulTailMs != 0u)
    {
        if (ucTurnLineBlindDrive(ulTailMs) == 0u)
        {
            vTurnLineDbg(pcTag, 4u, ulElapsedMs(ulTick0));
            return 0u;
        }
    }

    /* 巡线段的 w 环可能把车转了, 而 vLineFollowConverge **自己不更新**世界航向（那是给它
     * 自己那条老路径保持的既有行为）→ 这里按**源**收尾:
     *   陀螺档写这一步的指令目标（fGetLineGyroTarget(), ucRpiLineSrcResolve 入口已写入）;
     *   相机/融合保持实测（线可能把车转到任意朝向）; 不修不写。
     * ⚠️ 只有真跑过巡线才写: 跳过巡线的三条路径上面都已经 return 了。 */
    vLineStepSetWorldHeading(ucUse, fGetLineGyroTarget());

    vTurnLineDbg(pcTag, (ucHandoff != 0u) ? 2u : 3u, ulElapsedMs(ulTick0));
    return 1u;
}

/*
 * STEP_PIVOT_LINE_WAIT 的原语: **巡线起流与转弯重叠**, 转弯误差进 PIVOT_LINE_HANDOFF_DEG
 * 就松手交给巡线（陀螺退出, 位置与航向都改由线说了算）。
 *
 * ⚠️⚠️ **"重叠"是这一步唯一存在的理由, 别写成串行**: "先等满起流、再开始转"看着一样,
 *    其实总时长 = 起流等待 + 转弯, 与原来的两块积木相比**一点没省**（这份串行版真写过,
 *    是这一版改回来的）。做法是把**两个判据都塞进支点转弯的闭环**里 —— 转弯照转, 每 50ms
 *    顺带看一眼帧计数, **误差进阈值 且 帧已起流**的当拍才交（见 app_pid.c 的内核）。
 *    Pi 起流要几百毫秒（ucRpiLineProbe 的注释专门记过这件事）, 那几百毫秒正好被转弯吃掉。
 *    代价: Pi 一直不起流时车照常转完, 然后**补等到"起流超时的截止时刻"**（不是再等一个满窗）
 *    → 本步总时长上界 = max(转弯时长, LINE_FIRST_FRAME_TIMEOUT_MS), **不是两者相加**。
 *
 * 流程:
 *   ① 查链路 → OK 才发 LINE_START（**先取帧计数基线**）
 *   ② vPidPivotTurnUntil(..., ulFrame0) —— 转 + 顺带等起流, 两个条件齐了就松手（不停车）
 *   ②′~⑤ 交接后的全部段（盲走/补等/巡线两段/尾段/收尾）在 `s_ucTurnLineTail`
 *      —— 2026-10-30 起与自转版（ucRpiSpinTurnLine）**共用同一份**（交接后流程与谁转的弯
 *      无关; 不抽的话两条路必然改着改着漂移）。四个时长参数、src/settle_early 的语义、
 *      "settle 与 line 都为 0 仍跑一拍"的兜底、"为什么体坐标系/为什么停车"等全部细节,
 *      见 s_ucTurnLineTail 与 ucTurnLineBlindDrive 的函数头 —— 那里是唯一事实源。
 *
 * ⚠️ **巡线段听谁 = 积木参数 `src`**（档位同「巡线收敛」: 相机/陀螺/不修/融合; **不填就是相机**）。
 *    2026-09 之前它写死"强制纯相机"（`vSetLineGyroW(0)`）—— 现在那条粘性副作用连同融合一起
 *    消失了, 而且**交接后的最终航向也能选陀螺**（要绝对航向就用 `src=陀螺` + `gyro_target`）。
 *    ⚠️ 两段巡线共用这个档位: 交接之后就是一个连续的"听线"段, 没有分段的语义。
 * ⚠️⚠️ **链路不通就直接转完、跳过巡线**, 且**刻意不用 ucRpiSendTaskCmdWait**（与其余视觉
 *    步骤不同）: 转弯是纯本地动作、Pi 不在也干得了; 而阻塞版会不封顶地等重连、超时还反复发
 *    "r" 重启 Pi 客户端 —— 那期间**车停在原地不转**, 路线直接卡住。同理起流失败要补做
 *    **完整**转弯: 把降级做成"车不动也不报错"是这个工程反复踩过的最难查的失效。
 * ⚠️ 前提是**转弯终点能看到线**（由排表的人保证）。看不到线时 Pi 会按它的定向找线策略
 *    做大横移 —— 那不是这一步想要的。
 * 返回 1 = 巡线段跑过; 0 = 跳过巡线（链路不通 / 没起流）或被中止。仅任务上下文调用。
 */
uint8_t ucRpiPivotTurnLine(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                           uint32_t ulFwdMs, uint32_t ulSettleMs, uint32_t ulExtraLineMs,
                           uint32_t ulTailMs, uint8_t ucSrc, float fGyroTarget,
                           uint8_t ucSettleEarly, uint8_t ucCamPct)
{
    uint32_t ulTick0 = xTaskGetTickCount();   /* 起流超时的截止时刻也从这里起算 */
    uint32_t ulFrame0;
    uint8_t  ucHandoff;
    uint8_t  ucUse;                           /* 校验后的交接后航向来源 */

    /* 交接后的航向来源: 校验口径与「巡线收敛」**完全同一份代码**（ucRpiLineSrcResolve）,
     * 越界/认不出来的档位 → 本步当成 NONE(不修航向), 不写脏值进 Beam。 */
    ucUse = ucRpiLineSrcResolve(ucSrc, fGyroTarget, ucCamPct, "PVL");

    /* 链路不通: 连 LINE_START 都不发, 直接做完完整转弯（PIVOT_LINE 暂不带反向, ucRev=0） */
    if (emGetRpiLinkState() != RPI_LINK_OK)
    {
        vPidPivotTurn(iDir, fAngleDeg, ucAbsolute, 0u);
        vTurnLineDbg("PVL", 0u, ulElapsedMs(ulTick0));
        return 0u;
    }

    /* ⚠️ 基线必须在发 LINE_START **之前**取 —— 取晚了会把"已经起流"判成"没起流" */
    ulFrame0 = ulGetLineFrameCount();
    vRpiSendTaskCmd(RPI_TASK_LINE_START);      /* 照发不等链路（决策 3） */

    /* ② 转弯 + **顺带等起流**（重叠就在这个调用里: 帧没来就继续转, 不干等） */
    ucHandoff = vPidPivotTurnUntil(iDir, fAngleDeg, ucAbsolute,
                                   PIVOT_LINE_HANDOFF_DEG, ulFrame0);

    /* 被中止: 立刻收尾。**不接巡线、也不补鸣**（按停是操作员的意图, 不是故障） */
    if (ucGetScriptAbortReq() != 0u)
    {
        vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
        vTurnLineDbg("PVL", 4u, ulElapsedMs(ulTick0));
        return 0u;
    }

    return s_ucTurnLineTail(ulTick0, ulFrame0, ucHandoff,
                            ulFwdMs, ulSettleMs, ulExtraLineMs, ulTailMs,
                            ucUse, ucSettleEarly, "PVL");
}

/*
 * STEP_SPIN_LINE_WAIT 的原语（2026-10-30 加）: **原地转弯版**的"转弯→巡线接管" ——
 * 与 ucRpiPivotTurnLine **同构**, 只有两处不同:
 *   ① 转弯本体换 vPidSpinTurn / vPidSpinTurnUntil（四轮对称反向自转, 车心不动 —— 支点版
 *      是单侧驱动、车心会平移。哪种合适看站位: 交接后要**原地对准**的场合自转更准,
 *      车心不许漂的场合也是）;
 *   ② 调试串前缀 "SPIN"（支点版 "PVL"）—— Pi 日志里一眼分清。
 * 其余一切（起流重叠、交接判据、交接后的盲走/巡线两段/尾段/世界航向收尾、链路不通的降级、
 * 参数语义）**共用同一份代码**: 尾段 = s_ucTurnLineTail, 参数表与支点版**逐位同形**（F2I7）,
 * 排表经验可互相套用。⚠️ 自转角速度约支点的 2 倍 → 同一交接阈值下交接瞬间的残留自转也约
 * 2 倍, 巡线第一拍要多修一些（阈值共用 PIVOT_LINE_HANDOFF_DEG, 见 app_pid.h）。
 * ⚠️ 前提**转弯终点能看到线**、链路不通降级为完整转弯 —— 两条与支点版同一纪律。
 * 返回 1 = 巡线段跑过; 0 = 跳过巡线（链路不通 / 没起流）或被中止。仅任务上下文调用。
 */
uint8_t ucRpiSpinTurnLine(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                          uint32_t ulFwdMs, uint32_t ulSettleMs, uint32_t ulExtraLineMs,
                          uint32_t ulTailMs, uint8_t ucSrc, float fGyroTarget,
                          uint8_t ucSettleEarly, uint8_t ucCamPct)
{
    uint32_t ulTick0 = xTaskGetTickCount();
    uint32_t ulFrame0;
    uint8_t  ucHandoff;
    uint8_t  ucUse;

    ucUse = ucRpiLineSrcResolve(ucSrc, fGyroTarget, ucCamPct, "SPIN");

    /* 链路不通: 连 LINE_START 都不发, 直接做完完整自转（同支点版的降级纪律） */
    if (emGetRpiLinkState() != RPI_LINK_OK)
    {
        vPidSpinTurn(iDir, fAngleDeg, ucAbsolute);
        vTurnLineDbg("SPIN", 0u, ulElapsedMs(ulTick0));
        return 0u;
    }

    /* 基线必须在发 LINE_START **之前**取（取晚了会把"已经起流"判成"没起流"） */
    ulFrame0 = ulGetLineFrameCount();
    vRpiSendTaskCmd(RPI_TASK_LINE_START);

    /* 自转 + 顺带等起流（重叠就在这个调用里; 交接判据/武装门见 s_ucSpinTurnCore） */
    ucHandoff = vPidSpinTurnUntil(iDir, fAngleDeg, ucAbsolute,
                                  PIVOT_LINE_HANDOFF_DEG, ulFrame0);

    /* 被中止: 立刻收尾, 不接巡线（按停是操作员的意图, 不是故障） */
    if (ucGetScriptAbortReq() != 0u)
    {
        vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
        vTurnLineDbg("SPIN", 4u, ulElapsedMs(ulTick0));
        return 0u;
    }

    return s_ucTurnLineTail(ulTick0, ulFrame0, ucHandoff,
                            ulFwdMs, ulSettleMs, ulExtraLineMs, ulTailMs,
                            ucUse, ucSettleEarly, "SPIN");
}

/* ==================== 巡线主动探测（IF 条件用） ==================== */

/*
 * 巡线主动探测: 短暂起流 → 等它真的起流 → 数窗口内的有效帧 → 收流。
 * **不动底盘、不进闭环** —— 它只是替 IF 条件问 Pi 一句「现在还能不能看见线」。
 * 返回 1=窗口内见到的有效帧够数。
 *
 * ⚠️⚠️ **必须先等起流, 再开计时窗口**（两段式）。Pi 收到 LINE_START 后要相机重新取流 /
 *     任务重入才发得出第一帧（几百毫秒量级）, 一上来就开窗口的话, 那一整窗一帧都不会来
 *     → 被判成「丢线」→ IF 静默走失败分支。而「Pi 还没起流」和「丢线」是**两个完全不同的
 *     状态**: ucRpiLineRun 的注释专门记过, 把它们混起来是「车不动也不报错」那类最难查的失效。
 *     起流门用同一个 LINE_FIRST_FRAME_TIMEOUT_MS —— 问的是同一个问题「多久算 Pi 没起流」,
 *     两处必须一致, 否则会出现「STEP_LINE_CONVERGE_WAIT 成功, 紧跟的 IF 却说没线」。
 * ⚠️ 判「数据在流」必须看**帧计数有没有推进**, 不能用 valid —— 丢线时 Pi 照样发帧
 *    (valid=0), 而「一帧都没来」才是起流/链路失败。这个坑 ucRpiLineRun 已经踩过
 *    （见它的注释）, 所以这里先观察计数变化, 计数动了才读那一帧的 valid。
 * ⚠️ 无论如何都要发 LINE_STOP（起流失败也要发）—— 否则 Pi 那边一直跑线检测。
 * ⚠️ 耗时是**两段硬上界**: 起流段 ≤ LINE_FIRST_FRAME_TIMEOUT_MS,
 *    窗口段 ≤ ulProbeMs。Pi 不理人也不会无限等（最坏 2s + 150ms 左右）。
 * ⚠️ LINE_PROBE_MIN_VALID 计的是**帧数**、不是采样次数: 只有帧计数推进时才累加一次
 *    （轮询 10ms, Pi 30Hz → 150ms 窗口大约 5 帧）。@30Hz 150ms 想数到 6 帧以上是不可能的,
 *    设了会永远失败且不报错 —— 调参 Tab 的备注里写了这条。
 * ⚠️ 「起流失败/链路没响应」和「丢线」在这里都返回 0。要区分就另加一个 IF 条件
 *    （emScriptCondTdf 加一个成员 + app_script.c 的条件注册表加一行）, 别在这里
 *    搞第三种返回值 —— IF 只认成立/不成立。
 * ⚠️ vGetLineData **不认 NULL**（与 vGetHostageResult / vGetColorResult 不同）, 四个出参都得给。
 * 仅任务上下文调用（内部用 vTaskDelay）。
 */
uint8_t ucRpiLineProbe(uint32_t ulProbeMs)
{
    uint32_t ulBase;
    uint32_t ulLast;
    uint32_t ulValid  = 0;
    uint32_t ulWaited = 0;

    /* 起流帧等链路（同 ucRpiLineRun）; 被顶掉/被中止 → 判"没看见线"（本函数语义就是
     * 这个, IF 条件走失败分支）, 与「丢线」同码。本函数**不蜂鸣**（探测本身是静默的）。 */
    if (ucRpiSendTaskCmdWait(RPI_TASK_LINE_START, 0u) == 0u)
    {
        return 0u;
    }
    ulBase = ulGetLineFrameCount();             /* 起流基线 */

    /* 第一段: 等起流（与 ucRpiLineRun 完全同形, 含 LINE_FIRST_FRAME_TIMEOUT_MS=0 的旁路）。 */
    while ((ulGetLineFrameCount() == ulBase) && (ulWaited < LINE_FIRST_FRAME_TIMEOUT_MS))
    {
        /* ⚠️ 查中止位（等外部事件的循环都得带）: 中止 = 收尾停流 + 判"没答案",
         * 与「起流失败 / 丢线」同一个返回码（调用方的 IF 走失败分支, 脚本随即在
         * 步骤边界退出, 所以这里不必再区分）。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendTaskCmd(RPI_TASK_LINE_STOP);
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;
    }

    if ((ulGetLineFrameCount() == ulBase) && (LINE_FIRST_FRAME_TIMEOUT_MS != 0u))
    {
        vRpiSendTaskCmd(RPI_TASK_LINE_STOP);    /* 没起流也要收尾 */
        return 0u;                              /* 问不出答案 → 判失败（与「丢线」同码） */
    }

    /* 第二段: 起流了, 在窗口内数有效帧。 */
    ulLast   = ulGetLineFrameCount();
    ulWaited = 0;

    while (ulWaited < ulProbeMs)
    {
        uint32_t ulNow;

        /* ⚠️ 查中止位: 同第一段（+ 让下面的收尾 LINE_STOP 走同一条路）。
         * 中止把已累计的 ulValid 清零 —— 中止时不该"判在线"让 IF 走真分支。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            ulValid = 0u;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        ulNow = ulGetLineFrameCount();
        if (ulNow != ulLast)                    /* 计数推进 = 拿到一帧新的, 它的 valid 才算数 */
        {
            float   fErr;
            float   fSteer;
            float   fAngle;
            uint8_t ucValid = 0u;

            vGetLineData(&fErr, &fSteer, &fAngle, &ucValid);
            if (ucValid != 0u) {
                ulValid++;
            }
            ulLast = ulNow;
        }
    }

    vRpiSendTaskCmd(RPI_TASK_LINE_STOP);        /* ← 无条件发 */
    return (uint8_t)(ulValid >= (uint32_t)LINE_PROBE_MIN_VALID);
}

/* ==================== 小球对位（识别 → 转正 → 复测 → 平移） ==================== */

/*
 * 发一次 "5" 并等一帧合格结果: **status=1 且 count=3，就这两条**。
 * ⚠️⚠️ 2026-10-01 起**不再拿 fit_ok（三球共线质检）当门槛**（需求：别因为"共线 NG"
 *    整步不纠正 —— 原有行为是 NG 自动重扫一次、仍 NG 就长鸣+**一步不动**）。
 *    现在 fit_ok 只进日志（下面 BALL meas 的 fit=0/1），**不参与任何决策**：
 *      · 连线角不可信时 vPidSpinTurn 照转（yaw_en=1 时）；
 *      · 中球可疑时横/纵移照做。
 *    ⚠️ 这是**有意的取舍**，不是漏了：三球在板上共线 ⇒ 透视保直线 ⇒ 残差大 = 至少
 *    一个质心认错了，那时平移用的中球也可能就是错的那颗（"照着错位置把车开过去"）。
 *    宁可动错也不拒动的口径由使用者定下（2026-10-01）。**别"顺手"把门加回来**。
 *    Pi 端 fit_ok 的判据/日志/帧字段**一个字没动**（照发照打），本端只是不看。
 *    （`ball_fit_ok` 蓝牙遥测仍照常上报，仍是判断"这帧识别可不可信"的唯一入口。）
 * status/count 不对不重试 —— Pi 端本就是 5 帧投票, 它说没有就真没有。
 * 返回 1 = astT 已填三球（按 cx 升序 = 左/中/右）。仅任务上下文调用。
 */
static uint8_t ucBallScanOnce(stColorTgtTdf astT[COLOR_MAX_COUNT], uint32_t ulTimeoutMs)
{
    uint8_t ucStatus = 0;
    uint8_t ucCnt    = 0;
    char    acDbg[RPI_DATA_MAX];

    /* 等链路才发; 被顶掉/被中止 → 本次没结果（**不响**: 本函数是链里的内层
     * 助手, 失败音归 ucRpiAlignChain 统一管 —— 见 s_ucAlignBeepHold 的说明）。 */
    /* 先清方向字节（2026-10-26 加）: 它由**本次识别的失败帧**写入——超时（Pi 没回帧）
     * 时 Beam 里躺的是**上一轮**的旧方向, 不清就会被拉回逻辑当成本轮信号乱拉。 */
    vSetBallPullDir(0u);
    if (ucRpiSendTaskCmdWait(RPI_TASK_COLORBALL, ulTimeoutMs) == 0u)
    {
        return 0u;
    }

    if (!ucRpiWaitColorResult(ulTimeoutMs))
    {
        /* 失败要说清「为什么」—— 上报超时值 + 等到的状态/数量。
         * ⚠️ 超时值本身为 0（参数没配到, 见 vScriptTestLoop 的注释）时现象是
         *    「发完立刻判失败」, 与链路问题长得一模一样, 只有把这个数打出来才分得清。 */
        vGetColorResult(&ucStatus, &ucCnt, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "BALL fail: wait(t=%lums) st=%u cnt=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus, (unsigned)ucCnt);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    vGetColorResult(&ucStatus, &ucCnt, astT);
    if ((ucStatus != 1u) || (ucCnt != (uint8_t)COLOR_MAX_COUNT))
    {
        (void)snprintf(acDbg, sizeof(acDbg), "BALL fail: st=%u cnt=%u (need 1/%u)",
                       (unsigned)ucStatus, (unsigned)ucCnt, (unsigned)COLOR_MAX_COUNT);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    /* 测量结果进 Pi 日志（[STM-DBG], serial_comm 打印; 短串无碍 —— 发送早已是
     * TX DMA 环入队即返回, 且对位期间 PID 任务空闲）。
     * fit= 照实打（0 = 共线 NG）—— 现在它只是**已知情标记**，不再拒动（见函数头）。 */
    {
        char acYaw[12];
        bt_fmt_f2(acYaw, (int)sizeof(acYaw), fGetBallYawLine());
        (void)snprintf(acDbg, sizeof(acDbg),
                       "BALL meas: mid(%u,%u) yaw=%s fit=%u",
                       (unsigned)astT[1].usCx, (unsigned)astT[1].usCy,
                       acYaw, (unsigned)ucGetBallFitOk());
        vRpiSendDebug(acDbg);
    }
    return 1u;
}

/*
 * 小球对位一步到位（STEP_BALL_ALIGN_WAIT 的原语）:
 *   ① 发 "5" 拿三球中位数坐标 + 连线角（**只看 status/count**；fit_ok 不再拒动,
 *      见 ucBallScanOnce 函数头 —— 共线 NG 时连线角照用、平移照做）
 *   ② 蓝牙开关 ball_yaw_en=1 且 |车体角|>死区 → vPidSpinTurn 原地转正
 *      （陀螺仪闭环, 收敛带 ±1°, 自带超时/航向累加）
 *   ③ ②真的转了 → 再发一次 "5" 复测 —— 转正带来的像素视差**已含在新坐标里**,
 *      于是不需要 BALL_ROT_CX_PER_DEG 那类视差补偿常数（少标定一个耦合量,
 *      也避开它随距离变化的坑）
 *   ④ 中球(index 1)的 cx/cy 偏差 → 车体横移/纵移（各超死区才动, X→Y 顺序）
 *   对位基准 = **中间球**, 与动作组 ARM_ACT_3/4/5（右/中/左）的录制位姿一致。
 *   全部标定量走 Beam 运行时量（蓝牙 Tab「小球对位」组当场可调, 断电回
 *   app_config.h 的 BALL_* 默认）——车体角 = 图像连线角 × ball_yaw_scale
 *   （≈Z/h≈3.3; 图像角≠车体角, 别改回 1.0）。
 *   成功返回 1; 识别不全/超时返回 0 且**绝不动车**。
 *   ⚠️ 失败**不直接响**: 本函数现在由 ucRpiAlignChain 当 ALIGN_BALL_MID 方案调用,
 *      链里「不到赌不响」—— 失败音归链统一管（见 ucRpiAlignChain 段头）。
 *   ⚠️ 平移是开环, 精度全压在标定上; ③的复测是唯一的免费"闭环"。
 *   ⚠️ 失败后脚本照常走下一步（本执行器无失败分支） —— 排路线时对位步后面
 *      紧跟抓取, 失败就等于按没对位的老状态去抓, 不比现状差, 但要心里有数。
 *   仅任务上下文调用。
 */
uint8_t ucRpiBallAlign(uint32_t ulTimeoutMs)
{
    stColorTgtTdf astT[COLOR_MAX_COUNT];
    float fDxMm;
    float fDyMm;

    if (!ucBallScanOnce(astT, ulTimeoutMs))
    {
        vRpiSendDebug("BALL fail: scan");   /* 具体原因由 ucBallScanOnce 内部那条串说明 */
        return 0u;
    }

    if (ucGetBallYawEn() != 0u)
    {
        /* 与 cx/cy 同理: 转正的目标**不是 0°** 而是「理想停止位下那个读数」——
         * 相机装歪、地面不平、三球连线本身在世界里不平行于车体横轴, 都会让理想位姿下
         * ball_yaw 不是 0; 死按 0 就是把车往一个它不该在的角度上拽。
         * 标定: 车摆理想停止位跑一轮, 把遥测 ball_yaw 的数填进 ball_yaw_target。
         * 差 → 车体角 → 转正; 符号与倍率仍走 ball_yaw_sign / ball_yaw_scale。 */
        float fYawVeh = (fGetBallYawLine() - fGetBallYawTarget())
                        * fGetBallYawScale() * fGetBallYawSign();

        if ((fYawVeh > fGetBallYawDeadband()) || (fYawVeh < -fGetBallYawDeadband()))
        {
            char acYaw[12];
            char acDbg[RPI_DATA_MAX + 8u];
            bt_fmt_f2(acYaw, (int)sizeof(acYaw), fYawVeh);
            (void)snprintf(acDbg, sizeof(acDbg), "BALL turn: %sdeg(veh)", acYaw);
            vRpiSendDebug(acDbg);

            vPidSpinTurn((fYawVeh > 0.0f) ? 1 : 0,
                         (fYawVeh >= 0.0f) ? fYawVeh : -fYawVeh, 0u);   /* 0=相对: 修正量本就是转角 */
            /* 转过了 → 复测一次, 拿转正后的新坐标（视差已含在新 cx/cy 里） */
            if (!ucBallScanOnce(astT, ulTimeoutMs))
            {
                vRpiSendDebug("BALL fail: rescan");
                return 0u;
            }
        }
    }

    /* 中球偏差 → 车体 mm。默认: 图像 X→横移(cx 驱动)、图像 Y→纵移(cy 驱动)。
     * 横向复用 BALL_SIDE_SIGN（同一物理语义: 横移方向翻转）;
     * 纵向: 球偏画面下方(cy 大) = 车停得太近 → 该后退, 故取 (中心 − cy)。
     * ⚠️ BALL_XY_SWAP / 蓝牙 ball_xy_swap = 1 时**连轴带系数整套对调**
     *    （相机相对车体装了 90° 的情形）: 横移改由 cy 驱动、纵移改由 cx 驱动,
     *    两路 mm/px 增益也跟着换过来用; 方向符号仍各走 BALL_SIDE_SIGN / BALL_FWD_SIGN
     *    （对调后可能要翻它们 —— 现场试出来是哪个就翻哪个）。 */
    if (ucGetBallXySwap() != 0u)
    {
        fDxMm = ((float)astT[1].usCy - fGetBallCyCenter()) * fGetBallPxToMmY() * BALL_SIDE_SIGN;
        fDyMm = (fGetBallCxCenter() - (float)astT[1].usCx) * fGetBallPxToMmX() * fGetBallFwdSign();
    }
    else
    {
        fDxMm = ((float)astT[1].usCx - fGetBallCxCenter()) * fGetBallPxToMmX() * BALL_SIDE_SIGN;
        fDyMm = (fGetBallCyCenter() - (float)astT[1].usCy) * fGetBallPxToMmY() * fGetBallFwdSign();
    }

    /* 修正量进 Pi 日志, 再动车（日志先发免得被 1s 级的平移阻塞垫后）。
     * 方向字母 = 实际要做的动作: R/L=右/左横移, F/B=前进/后退, skip=死区内不动。 */
    {
        char    acDx[12];
        char    acDy[12];
        char    acDbg[RPI_DATA_MAX + 8u];
        uint8_t ucDxMove = ((fDxMm > fGetBallDxDeadband())
                            || (fDxMm < -fGetBallDxDeadband())) ? 1u : 0u;
        uint8_t ucDyMove = ((fDyMm > fGetBallDyDeadband())
                            || (fDyMm < -fGetBallDyDeadband())) ? 1u : 0u;

        bt_fmt_f2(acDx, (int)sizeof(acDx), fDxMm);
        bt_fmt_f2(acDy, (int)sizeof(acDy), fDyMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "BALL fix: dx=%smm(%s) dy=%smm(%s) yaw_en=%u",
                       acDx, ucDxMove ? ((fDxMm > 0.0f) ? "R" : "L") : "skip",
                       acDy, ucDyMove ? ((fDyMm > 0.0f) ? "F" : "B") : "skip",
                       (unsigned)ucGetBallYawEn());
        vRpiSendDebug(acDbg);

        if (ucDxMove != 0u)
        {
            vChassisMoveCameraLateral(fDxMm, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed());
        }
        if (ucDyMove != 0u)
        {
            vChassisMoveCameraForward(fDyMm, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed());
        }
    }
    return 1u;
}

/* ==================== 色球区对位：矩形板心方案 + 多方案链 ==================== */

/*
 * 矩形板心对位一步到位（脚本方案 ALIGN_RECT → ucRpiAlignChain 调）:
 *   发 RPI_TASK_RECT("A") → 等 0x07 帧（板心像素）→ 板心偏差 → 车体横/纵移
 *   （各超死区才动, X→Y 顺序）。
 *
 * ⚠️ **偏移换算整套复用小球那批标定量**（同一台相机、同一个停车位、同一个平面
 *    ⇒ mm/px 与方向符号都可迁移）: ball_px_to_mm_x/y、BALL_SIDE_SIGN、ball_fwd_sign、
 *    ball_xy_swap、ball_dx/dy_db_mm、ball_speed/accel —— 本方案**只**新增两个中心量
 *    rect_cx_center / rect_cy_center（理想停车位下板心该停的像素位置）。
 * ⚠️ 两个中心量必须与 ball_cx/cy_center **在同一停车位姿**下标定; 两种方案的测量定义
 *    不同（量胶带内沿 vs 中线, 实测差 ~15px≈5mm）, 那点差会被各自中心量天然吸收,
 *    **别再加补偿量**（补偿反而引入 5mm 系统偏移）。
 * ⚠️ 不做转正: 矩形没有三球连线角。要用板子上下边的倾角得另标一个"理想角",
 *    且一旦转正, "方案1失败→方案2成功"的落点会与方案1成功差一个朝向 —— 见 CLAUDE.md。
 * ⚠️ 失败**不响**（链内统一管, 见 s_ucAlignBeepHold 与 ucRpiAlignChain 段头）。
 * 成功返回 1; 失败/超时/中止返回 0 且**绝不动车**。仅任务上下文调用。
 */
uint8_t ucRpiRectAlign(uint32_t ulTimeoutMs)
{
    uint8_t  ucStatus = 0u;
    uint16_t usCx = 0u;
    uint16_t usCy = 0u;
    uint8_t  ucFlags = 0u;
    uint8_t  ucSpread = 0u;
    float    fDxMm;
    float    fDyMm;

    /* ⚠️ 未标定就**拒绝执行**（链内不响, 调试串留痕）: 板心中心量默认是 0（未标定），
     * 若照着一个占位值去动，车会朝一个凭空的位置横移 —— 宁可不动也不照占位值跑。
     * 标定法见 CLAUDE.md「色球区多方案对位链」: 车摆理想停止位读一次 rect_cx/rect_cy 填进蓝牙参数。 */
    if ((fGetRectCxCenter() <= 0.5f) || (fGetRectCyCenter() <= 0.5f))
    {
        vRpiSendDebug("RECT fail: rect_cx/cy_center not calibrated");
        return 0u;
    }

    /* 等链路才发; 被顶掉/被中止 → 本方案失败（不响, 归链统一管） */
    if (ucRpiSendTaskCmdWait(RPI_TASK_RECT, ulTimeoutMs) == 0u)
    {
        return 0u;
    }

    if (ucRpiWaitRectResult(ulTimeoutMs) == 0u)
    {
        /* 失败要说清"为什么"—— 上报超时值 + 等到的状态（与色球那边同一个习惯） */
        char acDbg[RPI_DATA_MAX];
        vGetRectResult(&ucStatus, NULL, NULL, NULL, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "RECT fail: wait(t=%lums) st=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    vGetRectResult(&ucStatus, &usCx, &usCy, &ucFlags, &ucSpread);

    /* 板心偏差 → 车体 mm（与小球对位同一套量; XY对调用同一个开关, 语义一致） */
    if (ucGetBallXySwap() != 0u)
    {
        fDxMm = ((float)usCy - fGetRectCyCenter()) * fGetBallPxToMmY() * BALL_SIDE_SIGN;
        fDyMm = (fGetRectCxCenter() - (float)usCx) * fGetBallPxToMmX() * fGetBallFwdSign();
    }
    else
    {
        fDxMm = ((float)usCx - fGetRectCxCenter()) * fGetBallPxToMmX() * BALL_SIDE_SIGN;
        fDyMm = (fGetRectCyCenter() - (float)usCy) * fGetBallPxToMmY() * fGetBallFwdSign();
    }

    /* 测量值 + 修正量进 Pi 日志, 再动车（日志先发免得被 1s 级的平移阻塞垫后） */
    {
        char    acDx[12];
        char    acDy[12];
        char    acDbg[RPI_DATA_MAX + 8u];
        uint8_t ucDxMove = ((fDxMm > fGetBallDxDeadband())
                            || (fDxMm < -fGetBallDxDeadband())) ? 1u : 0u;
        uint8_t ucDyMove = ((fDyMm > fGetBallDyDeadband())
                            || (fDyMm < -fGetBallDyDeadband())) ? 1u : 0u;

        bt_fmt_f2(acDx, (int)sizeof(acDx), fDxMm);
        bt_fmt_f2(acDy, (int)sizeof(acDy), fDyMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "RECT meas: cx=%u cy=%u fl=0x%02X sp=%u",
                       (unsigned)usCx, (unsigned)usCy, (unsigned)ucFlags, (unsigned)ucSpread);
        vRpiSendDebug(acDbg);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "RECT fix: dx=%smm(%s) dy=%smm(%s)",
                       acDx, ucDxMove ? ((fDxMm > 0.0f) ? "R" : "L") : "skip",
                       acDy, ucDyMove ? ((fDyMm > 0.0f) ? "F" : "B") : "skip");
        vRpiSendDebug(acDbg);

        if (ucDxMove != 0u)
        {
            vChassisMoveCameraLateral(fDxMm, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed());
        }
        if (ucDyMove != 0u)
        {
            vChassisMoveCameraForward(fDyMm, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed());
        }
    }
    return 1u;
}

/*
 * 拉回一步的共享助手（2026-10-26 加）: 读 Pi 失败帧的方向字节 → 沿排开环倒回
 * ball_pullback_mm → 消费掉方向字节。返回 1=拉了, 0=无方向（不动车）。
 * 被 ALIGN_PULLBACK 方案码（ucRpiPullbackAlign）和色球单动作抓取的识别失败
 * 分支（ucRpiBallGrabOne ②）共用 —— 两处的触发面相同: 发过 "5" 且 Pi 回了
 * 失败帧（剩两球、中点偏出阈值 → 缺端球在画面外哪侧是明确的）。
 * ⚠️ ucRev（2026-10-30 加）= 调用方是否「反向执行」: 道具换到路另一边、相机朝另一边
 *    看时, 方向标定按镜像取 —— 拉回方向跟着翻（老调用方传 0 = 原行为）。
 * ⚠️ 本函数不响（进「赌」的那一声由 ucRpiAlignChain 在进 ALIGN_PULLBACK 槽时投;
 *    除那一声外拉回/重扫保持安静 —— "不替用户做主"沿用 2026-10-26 的口径）。
 * 仅任务上下文调用。
 */
static uint8_t ucBallPullbackOnce(uint8_t ucRev)
{
    uint8_t ucDir = ucGetBallPullDir();
    float   fMm;
    float   fFwdSign;
    char    acMm[12];
    char    acDbg[RPI_DATA_MAX + 8u];

    if (ucDir == 0u)
    {
        return 0u;             /* 无信息: 盲走不如不走 */
    }

    /* 符号链与对位纵移同构: dir=1(缺球左出界) = 内容左移 = cx 偏小 → Forward(+),
     * 把对位公式里的"偏差量"换成固定的拉回距离 → ±pullback × ball_fwd_sign。
     * 方向现场反了翻一次 ball_fwd_sign, 对位和拉回两处同时改对;
     * rev=1（抓取积木勾了反向）在传进来之前已把对位符号翻了, 这里同步翻。 */
    fFwdSign = fGetBallFwdSign();
    if (ucRev != 0u) { fFwdSign = -fFwdSign; }
    fMm = ((ucDir == 1u) ? 1.0f : -1.0f) * fGetBallPullbackMm() * fFwdSign;
    bt_fmt_f2(acMm, (int)sizeof(acMm), fMm);
    (void)snprintf(acDbg, sizeof(acDbg), "PULL move: dir=%u(1=缺球左出界) %smm",
                   (unsigned)ucDir, acMm);
    vRpiSendDebug(acDbg);

    /* 平移复用对位的加减速档; 阻塞等走完再重扫（识别要求车静止） */
    vChassisMoveCameraForward(fMm, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed());

    /* 消费掉: 同一方向的旧字节不能喂给下一个槽/下一轮任务（快照语义, 用完即清） */
    vSetBallPullDir(0u);
    return 1u;
}

/*
 * 缺球开环拉回（ALIGN_PULLBACK 方案, 2026-10-26 加; 用户定的思路:
 * "根据偏左偏右运动一段固定距离再识别球" —— 最粗暴直接最快, 不做闭环）。
 *   触发面: 三球方案失败且缺球推断也没成（= 车沿排方向偏出太多、端球出画面,
 *   Pi 连第三颗都推不回来）→ 借 Pi 失败帧的方向字节倒回一步, 重跑三球对位。
 *   实现 = ucBallPullbackOnce（拉一步, 与 ucRpiBallGrabOne 共享）+ ucRpiBallAlign。
 * ⚠️ 本函数内部全程不响（拉回/重扫无论成败都安静）—— 「到赌了」的那一声由
 *    ucRpiAlignChain 在**进本槽时**投（2026-10-27 起, 用户口径「不到赌不响」）。
 * ⚠️ 只拉一次不循环（链的"同一方案只试一次"保证; 想多拉几步就多排几个槽）。
 * 返回 1 = 拉回后三球对位成功; 0 = 无方向 / 重扫仍失败。仅任务上下文调用。
 */
uint8_t ucRpiPullbackAlign(uint32_t ulTimeoutMs)
{
    if (ucBallPullbackOnce(0u) == 0u)   /* 老对位链没有「反向」概念: 恒正向（全局标定管方向） */
    {
        vRpiSendDebug("PULL fail: no dir (old Pi / balls < 2)");
        return 0u;
    }
    return ucRpiBallAlign(ulTimeoutMs);
}

/*
 * 色球区对位**多方案链**（脚本积木 STEP_BALL_ALIGN_WAIT 的原语）:
 *   按 m1 → m2 → m3 的顺序逐个试, **第一个成功就停**; ALIGN_NONE 跳过;
 *   同一方案重复写只试一次（排表错误, 免得白烧一个超时）。
 *
 * ⚠️ **每个方案各拿满 ulTimeoutMs**（用户定的语义: 宁可慢, 也别把下一个方案的时间砍短）:
 *    最坏阻塞 = 方案数 × (超时 + 平移时长)。3 槽 6000 就是 ~18s —— 它会跨过
 *    RPI_DISCONNECT_TIMEOUT_MS(8s), 链路中途闪一次 Disconnected（**不是故障**,
 *    Pi 一应答即自愈, 但会关掉蓝牙臂编辑窗）。想不闪别调小超时（那会低于 Pi 自己的
 *    识别耗时）, 正解是"一次识别带回两种测量"（见 CLAUDE.md 的 v2 备注）。
 * ⚠️ 每槽开始前查中止位, 中止就直接返回 0（不响 —— 停是操作员的意图）。
 * ⚠️ 蜂鸣分工（2026-10-27 起, 用户口径「**不到赌的时候不响**」）:
 *      · 链执行期间置 s_ucAlignBeepHold=1, **压住**色球/矩形等待函数自己的失败音
 *        —— 中间方案的失败不响（下一槽自己接上, 逐槽长鸣只会糊成一片）;
 *      · **进 ALIGN_PULLBACK 槽（赌）→ 失败音一声**: 到这一槽说明常规方案都没成、
 *        要靠开环倒回碰运气了, 操作员应当场听见;
 *      · 收尾: 全失败或一个方案都没配 → 失败音一声（链终失败）。
 *    于是整条链最多两声（进赌 + 终失败）, 成功全程静音。
 *    想分清是哪个方案赢的, 看 Pi 日志的 `ALIGN ok: ball|rect slot=N` 或蓝牙遥测
 *    align_ok / align_method。
 * ⚠️ 一个方案都没配也**响 + 调试串**，绝不静默空转（map.c 里老行没补全方案槽时
 *    就是这种情况 —— 见 app_script.h 的参数说明）。
 * 返回 1=链里某个方案成功; 0=全失败/没配/被中止。仅任务上下文调用。
 */
uint8_t ucRpiAlignChain(uint32_t ulTimeoutMs, emAlignMethodTdf emM1,
                        emAlignMethodTdf emM2, emAlignMethodTdf emM3)
{
    const emAlignMethodTdf aemSlot[3] = { emM1, emM2, emM3 };
    uint8_t ucTried = 0u;          /* 已试过的方案位掩码（同一方案只试一次） */
    uint8_t ucSlotTried = 0u;      /* 实际试了几个（0 = 一个都没配） */
    uint8_t ucIdx;
    char    acDbg[RPI_DATA_MAX];

    vSetAlignOk(0u);
    vSetAlignMethod(0u);
    s_ucAlignBeepHold = 1u;        /* 链内压住色球/矩形等待的失败音（所有出口都要清） */

    for (ucIdx = 0u; ucIdx < 3u; ucIdx++)
    {
        emAlignMethodTdf emMethod = aemSlot[ucIdx];
        uint8_t        ucBit;
        uint8_t        ucOk = 0u;

        if (emMethod == ALIGN_NONE) { continue; }              /* 空槽: 跳过 */

        /* ⚠️ 移位计数先夹住: emMethod 来自 map.c 的槽位（手写 / 老参数可能越界）,
         *    而 `1u << n` 在 n ≥ 32 时是 C **未定义行为**（ARM 上实际得 0）。
         *    越界的一律并到同一个"未知"位, 于是重复的未知方案照样会被去重跳过。
         *    范围校验本来就在下面（unknown 分支）, 这里只是别在它之前先踩 UB。 */
        ucBit = (uint8_t)((emMethod <= ALIGN_PULLBACK) ? (1u << (uint8_t)emMethod) : 0x80u);
        if ((ucTried & ucBit) != 0u)
        {
            (void)snprintf(acDbg, sizeof(acDbg), "ALIGN skip: dup method %u",
                           (unsigned)emMethod);
            vRpiSendDebug(acDbg);
            continue;
        }
        ucTried |= ucBit;

        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("ALIGN abort: stop by request");
            s_ucAlignBeepHold = 0u;
            return 0u;
        }
        ucSlotTried++;

        if (emMethod == ALIGN_BALL_MID)
        {
            ucOk = ucRpiBallAlign(ulTimeoutMs);               /* 失败音被 s_ucAlignBeepHold 压住 */
        }
        else if (emMethod == ALIGN_RECT)
        {
            ucOk = ucRpiRectAlign(ulTimeoutMs);               /* 本来就不响（见 ucRpiWaitRectResult） */
        }
        else if (emMethod == ALIGN_PULLBACK)
        {
            /* 到「赌」这一槽了: 先响一声再动（拉回/重扫本身保持安静, 见 ucRpiPullbackAlign） */
            vSetBuzzerReq(BUZZER_PAT_FAIL);
            ucOk = ucRpiPullbackAlign(ulTimeoutMs);
        }
        else
        {
            /* 认不出的方案码（手改 map.c / 新旧固件不匹配）: 当没配;
             * 全链都认不出会走到收尾的失败音, 不会静默空转 */
            (void)snprintf(acDbg, sizeof(acDbg), "ALIGN skip: unknown method %u",
                           (unsigned)emMethod);
            vRpiSendDebug(acDbg);
            continue;
        }

        if (ucOk != 0u)
        {
            (void)snprintf(acDbg, sizeof(acDbg), "ALIGN ok: %s slot=%u",
                           (emMethod == ALIGN_RECT) ? "rect"
                           : ((emMethod == ALIGN_PULLBACK) ? "pull" : "ball"),
                           (unsigned)(ucIdx + 1u));
            vRpiSendDebug(acDbg);
            vSetAlignOk(1u);
            vSetAlignMethod((uint8_t)emMethod);
            s_ucAlignBeepHold = 0u;
            return 1u;
        }
    }

    if (ucSlotTried == 0u)
    {
        vRpiSendDebug("ALIGN fail: no method configured (check map.c slots)");
    }
    else
    {
        (void)snprintf(acDbg, sizeof(acDbg), "ALIGN fail: tried=%u", (unsigned)ucSlotTried);
        vRpiSendDebug(acDbg);
    }
    vSetBuzzerReq(BUZZER_PAT_FAIL);            /* 链终失败: 失败音一声（进过赌就是第二声） */
    s_ucAlignBeepHold = 0u;
    return 0u;
}


/*
 * 发一次 "4" 并等一帧合格结果: status=1 且 count=HOSTAGE_MAX_COUNT（三只齐）。
 * ⚠️ 为什么"少一只就算失败"而不退而求其次: 基准是**按 cx 升序**数的第 ucIdx 只,
 *    缺一只时这个序号指向的物理人质就变了（少的是左边的, 那"中间"就落到原来右边的
 *    身上）—— 照着错位的人质对位, 比根本不对位更糟（会稳稳地停错位置）。与小球那边
 *    "count 必须 = 3 才算数"是同一条纪律。
 *    Pi 侧的正常配置下这条判据**永远不会误伤**: 它的成功条件就是凑满 HOSTAGE_MIN_TARGETS(3)
 *    且回传上限 HOSTAGE_MAX_TARGETS(3), 所以"Pi 报成功" ⟺ count=3。
 * ⚠️ 但 HOSTAGE_MIN_TARGETS 在 Pi 的 Web 控制端是**可调项**（GKS_Competition/config.py）:
 *    一旦被人调到 < 3, Pi 就会以 count<3 报成功, 而本步照样拒绝、一步不动 ——
 *    现场现象是「Pi 日志说识别成功、单片机就是不对位」, 查这里之前先去看 Pi 那个参数。
 * 不重试: 0x02 帧没有 fit_ok 那种"可疑但可用"的中间态, Pi 说没有就是真没有。
 * 返回 1 = pastT 已填三只（按 cx 升序 = 左/中/右）。仅任务上下文调用。
 */
static uint8_t ucHostageScanOnce(stHostageTgtTdf *pastT, uint32_t ulTimeoutMs)
{
    uint8_t ucStatus = 0;
    uint8_t ucCnt    = 0;
    char    acDbg[RPI_DATA_MAX];

    /* 等链路才发; 被顶掉/被中止 → 本次没结果（不响 —— 识别失败音由
     * ucRpiWaitHostageResult 投, 每次失败一声）。超时随指令下发（同 ucRpiHostageScan 的理由）。 */
    /* 先清快照（2026-10-26 加）: 单目标拉回读"失败帧里带的路标"——超时（Pi 没回帧）
     * 时快照里躺的是**上一轮**的旧目标, 不清就会被当成本轮路标乱拉。 */
    vSetHostageResult(0u, 0u, NULL);
    if (ucRpiSendTaskCmdWait(RPI_TASK_HOSTAGE, ulTimeoutMs) == 0u)
    {
        return 0u;
    }

    if (!ucRpiWaitHostageResult(ulTimeoutMs))
    {
        /* 失败要说清「为什么」—— 上报超时值 + 等到的状态/数量。
         * ⚠️ 超时值本身为 0（参数没配到, 见 vScriptTestLoop 的注释）时现象是
         *    「发完立刻判失败」, 与链路问题长得一模一样, 只有把这个数打出来才分得清。 */
        vGetHostageResult(&ucStatus, &ucCnt, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "HSTG fail: wait(t=%lums) st=%u cnt=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus, (unsigned)ucCnt);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    vGetHostageResult(&ucStatus, &ucCnt, pastT);
    if ((ucStatus != 1u) || (ucCnt != (uint8_t)HOSTAGE_MAX_COUNT))
    {
        (void)snprintf(acDbg, sizeof(acDbg), "HSTG fail: st=%u cnt=%u (need 1/%u)",
                       (unsigned)ucStatus, (unsigned)ucCnt, (unsigned)HOSTAGE_MAX_COUNT);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    return 1u;
}

/*
 * 单目标拉回（2026-10-26 加, 用户定的方案: "剩一只时拉回画面中间再识别"）。
 * 读失败帧快照（Pi 失败帧带"最好成绩", 0x02 协议零改动）: 恰剩 1 只路标 →
 * 沿排方向**把它平移到理想列**（hs_cx_center, 对位的 cx 路同一条符号链）→
 * 返回 1 由调用方重扫。
 * 为什么不需要知道它是谁: 是中间则车已对齐; 是端部则车只偏 150mm, 三只照样
 * 全在画面里（出画需 ~175mm+）, 重识别必然全出、后续对位自修——比色球的
 * ±固定距离开环准（按实测偏差定向平移）。
 * 路标 0 只（无路标）/ ≥2 只（不该到这, Pi 侧几何补齐兜过）→ 不动车返回 0。
 * ⚠️ 只修列方向（cx 路）, 远近（cy 路）留给对位——"拉回画面中间"的最小动作。
 * ⚠️ ucRev（2026-10-30 加）= 调用方是否「反向执行」: 道具换到路另一边、相机朝另一边看,
 *    偏差→车体平移的映射镜像 —— 拉回方向跟着翻（唯一调用方是人质单动作抓取,
 *    勾了 rev 才翻）。
 * ⚠️ 全程无蜂鸣（提示声用户自排, 同色球拉回纪律）。
 * 仅任务上下文调用。
 */
static uint8_t ucHstgPullToCenterOnce(uint8_t ucRev)
{
    uint8_t          ucStatus = 0u;
    uint8_t          ucCnt = 0u;
    stHostageTgtTdf  astT[HOSTAGE_MAX_COUNT];
    float            fMm;
    float            fSign;
    char             acMm[12];
    char             acDbg[RPI_DATA_MAX + 8u];

    vGetHostageResult(&ucStatus, &ucCnt, astT);
    if (ucCnt != 1u)
    {
        return 0u;
    }

    if (ucGetHsXySwap() != 0u)
    {
        fSign = fGetHsFwdSign();
        if (ucRev != 0u) { fSign = -fSign; }
        fMm = (fGetHsCxCenter() - (float)astT[0].usCx)
              * fGetHsPxToMmX() * fSign;
        bt_fmt_f2(acMm, (int)sizeof(acMm), fMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "HSTG1 pull: 路标(%u,%u)→理想列, 纵移%smm",
                       (unsigned)astT[0].usCx, (unsigned)astT[0].usCy, acMm);
        vRpiSendDebug(acDbg);
        vChassisMoveCameraForward(fMm, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed());
    }
    else
    {
        fSign = fGetHsSideSign();
        if (ucRev != 0u) { fSign = -fSign; }
        fMm = ((float)astT[0].usCx - fGetHsCxCenter())
              * fGetHsPxToMmX() * fSign;
        bt_fmt_f2(acMm, (int)sizeof(acMm), fMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "HSTG1 pull: 路标(%u,%u)→理想列, 横移%smm",
                       (unsigned)astT[0].usCx, (unsigned)astT[0].usCy, acMm);
        vRpiSendDebug(acDbg);
        vChassisMoveCameraLateral(fMm, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed());
    }
    return 1u;
}

/*
 * 人质对位一步到位（STEP_HOSTAGE_ALIGN_WAIT 的原语）:
 *   ① 发 "4" 拿三只人质的形状 + 中心像素（已按 cx 升序 = 左/中/右）;
 *   ② 取**参数指定的那一只**（0=左 / 1=中 / 2=右）作基准;
 *   ③ 它的 cx/cy 偏差 → 车体横移/纵移（各超死区才动, X→Y 顺序）。
 *
 * 与小球对位（ucRpiBallAlign）同构, 三处**有意**的不同:
 *   · 基准人质由**积木参数**给 —— 小球固定取中间那颗（基准绑死在录制动作组
 *     ARM_ACT_3/4/5 的位姿上）; 人质区要挑哪一只取决于战术, 让路线去说。
 *   · **不做转正**: 0x02 帧没有 yaw/fit_ok 扩展字段, 而圆柱/圆锥/腰鼓三种形状的
 *     质心高度本来就不同（同一排里腰鼓的质心比圆锥高）, 拿三个质心算连线角量到的
 *     一半是形状差异、不是车头歪 —— 会越转越歪。要做必须先在 Pi 端给 0x02 帧加
 *     扩展字段（照抄色球帧的做法）, 不是本端能单方面完成的; 所以别在这里"顺手补上"。
 *   · 标定量是**独立的一套 HOSTAGE_***（人质区停止位姿与球区不同, 共用会互相打架）。
 *
 * 成功返回 1; 参数越界/识别不全/超时返回 0 且**绝不动车**
 * （识别失败/超时有失败音 —— 等待函数投, 一次失败一声; 参数越界不响, 留痕）。
 * ⚠️ 平移是开环, 精度全压在那几个标定量上; 想更准就多排一步（每步都是一次新的识别 +
 *    修正, 天然迭代收敛）。
 * ⚠️ 失败后脚本照常走下一步（本执行器无失败分支） —— 排路线时对位步后面紧跟抓取,
 *    失败就等于按没对位的老状态去抓, 不比现状差, 但要心里有数。
 * 仅任务上下文调用。
 */
uint8_t ucRpiHostageAlign(uint32_t ulTimeoutMs, uint8_t ucIdx)
{
    stHostageTgtTdf astT[HOSTAGE_MAX_COUNT];
    float fDxMm;
    float fDyMm;
    char  acDbg[RPI_DATA_MAX];

    if (ucIdx >= (uint8_t)HOSTAGE_MAX_COUNT)
    {
        /* 参数越界（控制台下拉只有 0~2, 这里挡的是手写 map.c / 老参数错位）。
         * 不在这里悄悄夹到 0 或 2 —— 夹了就是「选的是左、对到的是右」, 静默且难查。 */
        vRpiSendDebug("HSTG fail: idx out of range");
        return 0u;
    }

    if (!ucHostageScanOnce(astT, ulTimeoutMs))
    {
        vRpiSendDebug("HSTG fail: scan");   /* 具体原因由 ucHostageScanOnce 内部那条串说明 */
        return 0u;
    }

    /* 测量结果进 Pi 日志（[STM-DBG], serial_comm 打印; 短串无碍 —— 发送早已是 TX DMA
     * 环入队即返回, 且对位期间 PID 任务空闲） */
    (void)snprintf(acDbg, sizeof(acDbg), "HSTG meas: #%u(%u,%u) shp=%u n=%u",
                   (unsigned)ucIdx, (unsigned)astT[ucIdx].usCx, (unsigned)astT[ucIdx].usCy,
                   (unsigned)astT[ucIdx].ucShape, (unsigned)HOSTAGE_MAX_COUNT);
    vRpiSendDebug(acDbg);

    /* 基准人质偏差 → 车体 mm。默认: 图像 X→横移(cx 驱动)、图像 Y→纵移(cy 驱动)。
     * 横向方向符号走 hs_side_sign（**本组独立标定**, 不像小球那样借 BALL_SIDE_SIGN）;
     * 纵向: 人质偏画面下方(cy 大) = 车停得太近 → 该后退, 故取 (中心 − cy)。
     * ⚠️ hs_xy_swap=1 时**连轴带系数整套对调**（相机相对车体装了 90° 的情形）: 横移改由
     *    cy 驱动、纵移改由 cx 驱动, 两路 mm/px 增益也跟着换过来用; 方向符号仍各走
     *    hs_side_sign / hs_fwd_sign（对调后可能要翻它们 —— 现场试出来是哪个就翻哪个）。 */
    if (ucGetHsXySwap() != 0u)
    {
        fDxMm = ((float)astT[ucIdx].usCy - fGetHsCyCenter()) * fGetHsPxToMmY() * fGetHsSideSign();
        fDyMm = (fGetHsCxCenter() - (float)astT[ucIdx].usCx) * fGetHsPxToMmX() * fGetHsFwdSign();
    }
    else
    {
        fDxMm = ((float)astT[ucIdx].usCx - fGetHsCxCenter()) * fGetHsPxToMmX() * fGetHsSideSign();
        fDyMm = (fGetHsCyCenter() - (float)astT[ucIdx].usCy) * fGetHsPxToMmY() * fGetHsFwdSign();
    }

    /* 修正量进 Pi 日志, 再动车（日志先发免得被 1s 级的平移阻塞垫后）。
     * 方向字母 = 实际要做的动作: R/L=右/左横移, F/B=前进/后退, skip=死区内不动。 */
    {
        char    acDx[12];
        char    acDy[12];
        uint8_t ucDxMove = ((fDxMm > fGetHsDxDeadband())
                            || (fDxMm < -fGetHsDxDeadband())) ? 1u : 0u;
        uint8_t ucDyMove = ((fDyMm > fGetHsDyDeadband())
                            || (fDyMm < -fGetHsDyDeadband())) ? 1u : 0u;

        bt_fmt_f2(acDx, (int)sizeof(acDx), fDxMm);
        bt_fmt_f2(acDy, (int)sizeof(acDy), fDyMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "HSTG fix: #%u dx=%smm(%s) dy=%smm(%s)",
                       (unsigned)ucIdx,
                       acDx, ucDxMove ? ((fDxMm > 0.0f) ? "R" : "L") : "skip",
                       acDy, ucDyMove ? ((fDyMm > 0.0f) ? "F" : "B") : "skip");
        vRpiSendDebug(acDbg);

        if (ucDxMove != 0u)
        {
            vChassisMoveCameraLateral(fDxMm, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed());
        }
        if (ucDyMove != 0u)
        {
            vChassisMoveCameraForward(fDyMm, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed());
        }
    }
    return 1u;
}

/* ==================== 排爆抓取（按二维码选球 → 动作预设 → 复位） ==================== */

/*
 * 排爆（STEP_BOMB_GRAB_WAIT 的原语）:
 *   ① 读二维码**首位** = 目标色（1红/2绿/3蓝, 与 color_t / Pi 的颜色码同码）;
 *   ② 读**已识别**的色球结果（三球按 cx 升序）→ 找目标色排在哪一位（0/1/2 = 左/中/右）;
 *   ③ 播那一位对应的动作预设（左/中/右三个 ID 由参数给）;
 *   ④ 播复位预设（参数给，通常 ARM_ACT_11「复位回待命」）。
 *
 * ⚠️ ③ 与 ④ 的**阻塞与否各自跟随那个动作自己标的播放方式**（ucArmPlayByMode）——
 *    「阻塞到位」= 等它整段走完; 「启动即返回」= 排队给后台、本步立即往下走。
 *    所以四组组合（③/④ 各两种）都是「你在「机械臂」Tab 标什么就是什么」;
 *    两条都标「启动即返回」时积木立刻返回, 臂在后台按「抓取→复位」依次跑完。
 *    别再改回写死的 vArmPlayWait（那会让「启动即返回」静默失效, 真踩过）。
 *    ⚠️ 「③启动即返回 + ④阻塞到位」这组仍是**整段阻塞**: ④ 的"到位"是以它自己排队的
 *       位置为准的（不会去掐 ③）, 所以路线要等「抓取 + 复位」跑完才往下走。想让路线
 *       真的立刻走, 两条都得标「启动即返回」。
 *
 * 依赖「读最近一次结果」—— 与 IF 条件同口径, 所以**本步必须排在二维码扫描 + 色球识别之后**
 * （路线里对位那一步会发 "5" 拿到三球结果, 正好复用）。任何一步数据不全就**一步都不动**
 * （扫描类失败有失败音 —— 各等待函数投; 其余失败调试串留痕）。
 * ⚠️ 动作预设 ID 走参数（控制台「动作」下拉自动取自 emArmActionIdTdf）—— 换了/新录了预设只改
 *    参数, 不必动代码。执行器已把本步登记进 c_bIsArmStep: 进这一步前会先掐掉后台臂序列。
 * 返回 1 = 抓完 + 复位完; 0 = 数据不全（未动臂）。仅任务上下文调用。
 */
uint8_t ucRpiBombGrab(emArmActionIdTdf emLeft, emArmActionIdTdf emMid,
                      emArmActionIdTdf emRight, emArmActionIdTdf emReset)
{
    int32_t         iQr;
    uint8_t         ucWant;
    uint8_t         ucStatus = 0;
    uint8_t         ucCnt    = 0;
    stColorTgtTdf   astT[COLOR_MAX_COUNT];
    uint8_t         ucI;
    int             iIdx = -1;
    uint8_t         ucPlay = 1u;      /* 两次 ucArmPlayByMode 的降级标志（见 ③④ 之后的告警） */
    emArmActionIdTdf emPlay;

    /* ① 二维码首位 = 目标色（100~399 才有效, 与 IF 条件 / 对位链路同一判据） */
    iQr = iGetQrCode();
    if ((iQr < 100) || (iQr > 399)) { return 0u; }
    ucWant = (uint8_t)(iQr / 100);

    /* ② 色球结果快照（三球齐才算有效）, 找目标色排第几位 */
    vGetColorResult(&ucStatus, &ucCnt, astT);
    if ((ucStatus != 1u) || (ucCnt != (uint8_t)COLOR_MAX_COUNT)) { return 0u; }

    for (ucI = 0u; ucI < ucCnt; ucI++)
    {
        if (astT[ucI].ucColor == ucWant) { iIdx = (int)ucI; break; }
    }
    if (iIdx < 0) { return 0u; }

    /* ③ 位置 → 动作预设。走 ucArmPlayByMode: **阻塞方式跟着这个动作自己标的播放方式**
     *    （阻塞到位 = 等它整段走完; 启动即返回 = 投后台队列、本步立即往下走）。
     *    这与路线里那块"动作大积木"走的是同一批原语（台架试跑 == 比赛跑路线的证据链）。 */
    emPlay = (iIdx == 0) ? emLeft : ((iIdx == 1) ? emMid : emRight);
    ucPlay = ucArmPlayByMode(emPlay);

    /* ④ 复位（沉淀延时按设计规则不藏在动作函数里 —— 需要就在路线里补 STEP_DELAY）。
     *    复位**与左/中/右同一套规则**: 它标了「启动即返回」就排在抓取后面一起后台跑完,
     *    标了「阻塞到位」就在这里等整条队列（含刚排的抓取）跑空 —— 四组组合都是
     *    「你标什么就是什么」。⚠️「③启动即返回 + ④阻塞到位」整段仍会阻塞（见 app_arm_action.h）。 */
    if (ucArmPlayByMode(emReset) == 0u) { ucPlay = 0u; }

    /* 降级（队列满 / 等后台超时）要让人当场知道: 「这条没排上」或「臂可能还在动, 而路线
     * 要往下走了」—— 本文件给不了臂侧的日志（那是 app_arm_action.c 的地盘, 而且反向包含
     * 会成环）, 所以给 Pi 打一条调试串留痕。正常路径永远不会走到。 */
    if (ucPlay == 0u)
    {
        vRpiSendDebug("BOMB arm play degraded (queue full / drain timeout)");
    }
    return 1u;
}

/* ==================== 反恐任务（按二维码第三位选人质 → 播对应动作预设） ==================== */

/*
 * 反恐任务（STEP_HOSTAGE_RESCUE_WAIT 的原语）：与排爆抓取 ucRpiBombGrab **同构**,
 *   ① 读二维码**第三位** = 救援目标的形状（1圆柱/2圆锥/3腰鼓, 与 emHostageShapeTdf 同码）;
 *   ② 读**已识别**的人质结果（三只按 cx 升序 = 左/中/右）→ 找这个形状排在哪一位;
 *   ③ 播那一位对应的动作预设（左/中/右三个 ID 由参数给）→ **播完就结束, 不复位**。
 *
 * 与排爆抓取逐条对照（同一套结构, 只换了「看哪一位」的键, 别把两处抄串）:
 *   · 键: 二维码**首位**=目标色 → 在三球里找颜色   ‖  二维码**第三位**=形状 → 在三只人质里找形状
 *   · 表: color_t(1红/2绿/3蓝) + stColorTgtTdf    ‖  emHostageShapeTdf(1圆柱/2圆锥/3腰鼓) + stHostageTgtTdf
 *     —— 两组码值都恰好是 1/2/3, 但**语义完全不同**（颜色 vs 形状）, 拿混了不会报错只会抓错东西;
 *   · 取位: `iQr / 100`                          ‖  `iQr % 10`（并对 1~3 单独判一次, 见下）
 *   · 末尾: 播复位预设                             ‖  **不复位**(保持机械臂最后姿态继续走路线)
 *
 * 最后那条是**本步存在的理由**: 复位由路线自己决定 —— 想复位就在后面排一块「播放动作」,
 * 不想复位（例如接着还要去抓下一只、或保持抓持状态去别处放）就什么都不排。所以这里
 * **不要**加复位参数、也不要偷偷补一次复位。
 *
 * ⚠️ ③ 的**阻塞与否跟随那个动作自己标的播放方式**（ucArmPlayByMode）—— 与排爆抓取同一套规则。
 *
 * 依赖「读最近一次结果」—— 与 IF 条件同口径, 所以**本步必须排在二维码扫描 + 人质识别之后**
 * （路线里人质对位那一步会发 "4" 拿到三只结果, 正好复用）。任何一步数据不全
 * （二维码 100~399 之外 / 第三位不在 1~3 / 三只不齐 / 三只里没这个形状）→ **拒绝且一步不动**
 * （不瞎抓: 宁可停下让人看, 也不能伸着爪子往一只不知道形状的人质上撞）。
 * ⚠️ 动作预设 ID 走参数（控制台「动作」下拉自动取自 emArmActionIdTdf）—— 换了/新录了预设只改
 *    参数, 不必动代码。执行器已把本步登记进 c_bIsArmStep: 进这一步前会先掐掉后台臂序列。
 * 返回 1 = 动作播完; 0 = 数据不全（未动臂）。仅任务上下文调用。
 */
uint8_t ucRpiHostageRescue(emArmActionIdTdf emLeft, emArmActionIdTdf emMid,
                           emArmActionIdTdf emRight)
{
    int32_t          iQr;
    uint8_t          ucWant;
    uint8_t          ucStatus = 0;
    uint8_t          ucCnt    = 0;
    stHostageTgtTdf  astT[HOSTAGE_MAX_COUNT];
    uint8_t          ucI;
    int              iIdx = -1;
    emArmActionIdTdf emPlay;

    /* ① 二维码第三位 = 目标形状（100~399 才有效, 与 IF 条件同一判据）。
     *    ⚠️ 光判 100~399 **不够**: 390 也在范围内, 但第三位是 0 ——
     *    第三位必须**单独**判 1~3, 否则 0 会被当成"没找到形状"（走同一个拒绝出口）
     *    或更糟: 落到某个越界的形状码上。 */
    iQr = iGetQrCode();
    if ((iQr < 100) || (iQr > 399)) { return 0u; }
    ucWant = (uint8_t)(iQr % 10);
    if ((ucWant < 1u) || (ucWant > (uint8_t)HOSTAGE_MAX_COUNT)) { return 0u; }

    /* ② 人质结果快照（三只齐才算有效 —— 少一只"左/中/右"的序号就不指向原来那只了,
     *    与小球对位/人质对位同一条纪律）, 找目标形状排第几位 */
    vGetHostageResult(&ucStatus, &ucCnt, astT);
    if ((ucStatus != 1u) || (ucCnt != (uint8_t)HOSTAGE_MAX_COUNT)) { return 0u; }

    for (ucI = 0u; ucI < ucCnt; ucI++)
    {
        if (astT[ucI].ucShape == ucWant) { iIdx = (int)ucI; break; }
    }
    if (iIdx < 0) { return 0u; }

    /* ③ 位置 → 动作预设。走 ucArmPlayByMode: **阻塞方式跟着这个动作自己标的播放方式**
     *    （阻塞到位 = 等它整段走完; 启动即返回 = 投后台队列、本步立即往下走）。
     *    与路线里那块"动作大积木"走同一批原语（台架试跑 == 比赛跑路线的证据链）。
     *    播完即返回 —— **不收尾、不复位**（见函数头说明）。 */
    emPlay = (iIdx == 0) ? emLeft : ((iIdx == 1) ? emMid : emRight);
    if (ucArmPlayByMode(emPlay) == 0u)
    {
        vRpiSendDebug("HSTG arm play degraded (queue full / drain timeout)");
    }
    return 1u;
}

/* ==================== 单动作抓取（色球/人质, 2026-10 新方案） ====================
 * 与上面两块（排爆/反恐）**并存**: 那两块"左/中/右各播一个录制动作", 对停车位/角度
 * 的容忍度全压在三个录制位姿上; 这一套只录**一个**"抓中间目标"的动作 —— 非中间目标
 * 靠把车平移 (位置-1)×one_lat_mm 挪过来, 抓的永远是"中间正前方"那个位置。 */

/*
 * 平移分量（蓝牙 one_lat_mm / one_lat_dir 现取现用）:
 *   返回值直接是**车体 mm**, 轴由调用方按 one_lat_dir 分（1/2=横向 3/4=纵向）。
 *   左(0)取负、右(2)取正是"图像从左到右 = 正向平移朝第一颗"的默认约定,
 *   现场试出来反了就在蓝牙里把 1⇄2（或 3⇄4）换一档, 不用重编译。
 *   iIdx==1（中间目标）或 one_lat_dir==0 → 0（不平移, 退化成老的单点对位）。
 */
static float c_fOneGrabLatMm(int iIdx)
{
    uint8_t ucDir = ucGetOneLatDir();
    float   fMm;

    if ((ucDir == 0u) || (iIdx == 1)) { return 0.0f; }

    fMm = (iIdx < 1) ? -fGetOneLatMm() : fGetOneLatMm();
    if ((ucDir == 2u) || (ucDir == 4u)) { fMm = -fMm; }    /* "负向"档 = 整体翻符号 */
    return fMm;
}

/*
 * 等 vArmTask 的后台序列真正跑完（arm_seq_busy 出现过 1 再回 0）。
 * 单动作抓取的抓取/放置都走 vArmPlayNoWait（入队即返回, 好让底盘在臂动作期间
 * 并行做巡线收敛/回中）, "下一步该干什么"必须等臂真正空闲再定 —— 本函数就是那道闸。
 * ⚠️ ucSeenRun = 发臂处**当场闩下**的"动作已开始"（vOneGrabLatchArmSeen）—— 只靠
 *    在本循环里采样"见过 1"在异步化之后**不够**: 动作常常在平移/收敛期间就播完了,
 *    本循环开始时 busy 已经回 0, "见过 1"永远闩不上 → 白等满 30s 超时（真踩过）。
 * 返回 1 = 跑完; 0 = 被中止 / 超时（ONE_GRAB_ARM_WAIT_MS 兜底, 调用方打调试串继续走,
 *   与排爆那块"降级留痕"同一条纪律）。仅任务上下文调用。
 */
static uint8_t ucOneGrabWaitArm(uint8_t ucSeenRun)
{
    uint32_t ulWaited  = 0u;
    uint8_t  ucSeen    = ucSeenRun;

    while (ulWaited < (uint32_t)ONE_GRAB_ARM_WAIT_MS)
    {
        if (ucGetScriptAbortReq() != 0u) { return 0u; }   /* 中止位: 「等外部事件的循环」铁律 */
        if (ucGetArmSeqBusy() != 0u)
        {
            ucSeen = 1u;
        }
        else if (ucSeen != 0u)
        {
            return 1u;                                    /* 1→0: 序列（含队列）真跑空了 */
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10u;
    }
    return 0u;
}

/* ---- 单动作抓取: 平移 × 延时异步臂（2026-10 加, 取代旧的一体式阻塞 vOneGrabShift）----
 * 底盘位置移动**只发不等**, 等待由这里按估时 10ms 切片自己做; 从 xT0 起算第 ulDelayMs
 * 毫秒（还没发过的话）把 emAct 异步投给臂泵 —— 于是臂的"开始"由延时参数决定,
 * "结束"由 ucOneGrabWaitArm 兜底等齐（底盘不等它开始, 只等它结束）。
 * ulDelayMs=0 → 臂与底盘同时启动（全并行）; D 比整段平移还长 → 平移走完、到点再播
 * （等价旧行为再加一段纯延时）; D 落在双轴平移中间 → 在后一轴的窗口里发出。
 * ⚠️ 轴完成判据是估时（没有 EMM 应答可轮询, 与 fChassisMoveDelay 同精度）。
 * ⚠️ 中止: 返回 0; 在飞的位置移动 EMM 自终止（与过减速带那条注释同一口径）。 */

/* 发完臂后守一小段窗, 把 arm_seq_busy 的"已开始"闩进 *pbSeen —— 泵认领 ≤20ms,
 * 动作一跑就是秒级, 300ms 窗内 busy 必为 1。⚠️ 这道闩**必须**在发完当场做:
 * 旧版只在收尾的等待循环里采样"见过 1", 而异步化之后动作常常在平移/收敛期间就
 * 播完了 —— 收尾循环开始采样时 busy 已回到 0, "见过 1"永远闩不上 → 白等满
 * ONE_GRAB_ARM_WAIT_MS(30s) 才超时放行（台架实测的"有时卡很久"就是它）。 */
static void vOneGrabLatchArmSeen(uint8_t *pbSeen)
{
    uint32_t ulWaited = 0u;

    while ((ulWaited < 300u) && (ucGetArmSeqBusy() == 0u))
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        ulWaited += 5u;
    }
    if (ucGetArmSeqBusy() != 0u) { *pbSeen = 1u; }
}

/* 当场发臂 + 闩"已开始"（*pbIssued 守卫只发一次）。
 * 剩余时间触发（ucOneGrabAxisWithArm）之后它只剩**兜底**角色: 两轴都被死区跳过
 * （整段没动, "到位"=现在）或 lead<垫底 pad 没赶上轴内触发 —— 直接补发, 不再等。 */
static void ucOneGrabArmIssueNow(uint8_t *pbIssued, uint8_t *pbSeen, emArmActionIdTdf emAct)
{
    if (*pbIssued == 0u)
    {
        vArmPlayNoWait(emAct);
        *pbIssued = 1u;
        vOneGrabLatchArmSeen(pbSeen);
    }
}

/* 「单步确定」的阶段边界（2026-11 加）: 先清速（位置移动的估时走完可能还有残余），
 * 再等 MAIN / 中止位。返回 1=操作员按了 MAIN, 0=脚本被中止（调用方原样返回 0）。 */
static uint8_t ucOneGrabStepWait(void)
{
    vChassisMove(0, 0, 0);
    return ucScriptWaitKeyConfirm();
}

/* 只平移、**不发臂**（单步确定的「移动」段用）: 发位置指令 → 按梯形估时切片等完。
 * 与 ucOneGrabAxisWithArm 的底盘部分同一套启动/估时, 差别只在**完全不碰臂泵** ——
 * 抓取留给下一个阶段（按 MAIN 后由调用方单独发）。返回 0=被中止。 */
static uint8_t ucOneGrabMoveAxis(uint8_t ucLateral, float fMm, int iAccel, int iSpeed)
{
    TickType_t x0 = xTaskGetTickCount();
    uint32_t   ulEst;

    if (ucLateral != 0u) { vChassisMoveCameraLateralStart(fMm, iAccel, iSpeed); }
    else                 { vChassisMoveCameraForwardStart(fMm, iAccel, iSpeed); }

    ulEst = (uint32_t)fChassisMoveTimeEst(fMm, (uint16_t)iAccel, (uint16_t)iSpeed,
                                          (ucLateral != 0u) ? AXIS_X : AXIS_Y);

    while (ulElapsedMs(x0) < ulEst)
    {
        if (ucGetScriptAbortReq() != 0u) { return 0u; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 1u;
}

/* 一轴平移 + "剩 lead 发臂"融合（2026-10-20 起, 取代旧的"起点后延时"）:
 * 底盘位置指令发出后按梯形估时切片等待; 发臂 = 「倒计时(本轴估时剩余 + ulNextEstMs
 * —— 整段平移还有下一轴时 = 那一轴的估时, 锚定**最终到位** —— 再垫
 * ONE_GRAB_LEAD_PAD_MS)掉到 ulLeadMs 以下」**且**「距整段平移起点(xT0)已过
 * ONE_GRAB_MIN_DELAY_MS」。后者是短行程兜底: 短行程触发首拍即中 = 与底盘同时
 * 开始, 设了最短延迟就强制车先走/停稳一段(0=旧行为, 见 app_config.h)。
 * 轴估时走完而最短延迟没到时**留在本轴里等**, 发完(或到点)才交棒 —— 下一轴的
 * 位置指令相应推迟, 底盘原地多等, 无害; 提前交棒会让收尾兜底绕过最短延迟立即发。
 * 返回 0=被中止。 */
static uint8_t ucOneGrabAxisWithArm(uint8_t ucLateral, float fMm, int iAccel, int iSpeed,
                                    TickType_t xT0, uint32_t ulLeadMs, uint32_t ulNextEstMs,
                                    uint8_t *pbIssued, uint8_t *pbSeen, emArmActionIdTdf emAct)
{
    TickType_t xAxis0 = xTaskGetTickCount();
    uint32_t   ulEst;
    uint32_t   ulMinDelay = (uint32_t)ONE_GRAB_MIN_DELAY_MS;

    if (ucLateral != 0u) { vChassisMoveCameraLateralStart(fMm, iAccel, iSpeed); }
    else                 { vChassisMoveCameraForwardStart(fMm, iAccel, iSpeed); }

    ulEst = (uint32_t)fChassisMoveTimeEst(fMm, (uint16_t)iAccel, (uint16_t)iSpeed,
                                          (ucLateral != 0u) ? AXIS_X : AXIS_Y);

    for (;;)
    {
        uint32_t ulElapsed = ulElapsedMs(xAxis0);
        uint32_t ulLeft    = (ulEst > ulElapsed) ? (ulEst - ulElapsed) : 0u;
        uint32_t ulSegMs   = ulElapsedMs(xT0);     /* 距整段平移起点(最短延迟锚点) */

        if (ucGetScriptAbortReq() != 0u) { return 0u; }

        if ((*pbIssued == 0u)
            && ((ulLeft + ulNextEstMs + (uint32_t)ONE_GRAB_LEAD_PAD_MS) <= ulLeadMs)
            && (ulSegMs >= ulMinDelay))
        {
            vArmPlayNoWait(emAct);            /* 双条件到点: 异步发臂 + 当场闩"已开始" */
            *pbIssued = 1u;
            vOneGrabLatchArmSeen(pbSeen);
        }
        if ((ulLeft == 0u)
            && ((*pbIssued != 0u) || (ulSegMs >= ulMinDelay)))
        {
            return 1u;                        /* 本轴估时走完, 且兜底不会再违反最短延迟 */
        }

        vTaskDelay(pdMS_TO_TICKS(10));        /* 10ms 切片: 中止与触发都按此精度 */
    }
}

/* 旧版「到 D 发臂」：从 xT0 起算第 ulDelayMs 毫秒才投臂（0=与回退同时开始）。
 * **只给放置用**（2026-10-23 用户定）: 回退到爆破桶通常比放置动作短得多，
 * 用"剩提前量"会在首拍就命中（等于没有提前量），而且放置后还要等内部收敛；
 * 放置侧改回这个简单延时，播完立即返回。抓取仍走上面的「剩提前量」版本。 */
static uint8_t ucOneGrabAxisWithArmDelay(uint8_t ucLateral, float fMm, int iAccel, int iSpeed,
                                         TickType_t xT0, uint32_t ulDelayMs,
                                         uint8_t *pbIssued, uint8_t *pbSeen, emArmActionIdTdf emAct)
{
    TickType_t xAxis0 = xTaskGetTickCount();
    uint32_t   ulEst;

    if (ucLateral != 0u) { vChassisMoveCameraLateralStart(fMm, iAccel, iSpeed); }
    else                 { vChassisMoveCameraForwardStart(fMm, iAccel, iSpeed); }

    ulEst = (uint32_t)fChassisMoveTimeEst(fMm, (uint16_t)iAccel, (uint16_t)iSpeed,
                                          (ucLateral != 0u) ? AXIS_X : AXIS_Y);

    for (;;)
    {
        uint32_t ulElapsedAxis = ulElapsedMs(xAxis0);
        uint32_t ulLeft        = (ulEst > ulElapsedAxis) ? (ulEst - ulElapsedAxis) : 0u;

        if (ucGetScriptAbortReq() != 0u) { return 0u; }

        if ((*pbIssued == 0u) && (ulElapsedMs(xT0) >= ulDelayMs))
        {
            vArmPlayNoWait(emAct);
            *pbIssued = 1u;
            vOneGrabLatchArmSeen(pbSeen);
        }
        if (ulLeft == 0u) { return 1u; }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* 无平移时（中间球）也要遵守放置延时: 等到 xT0+ulDelayMs 再发。
 * 回退轴估时先走完而延时没到 -> 轴函数返回时还没发, 由这里补等。 */
static uint8_t ucOneGrabArmIssueAt(TickType_t xT0, uint32_t ulDelayMs,
                                   uint8_t *pbIssued, uint8_t *pbSeen, emArmActionIdTdf emAct)
{
    while (ulElapsedMs(xT0) < ulDelayMs)
    {
        if (ucGetScriptAbortReq() != 0u) { return 0u; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (*pbIssued == 0u)
    {
        vArmPlayNoWait(emAct);
        *pbIssued = 1u;
        vOneGrabLatchArmSeen(pbSeen);
    }
    return 1u;
}

/*
 * 人质单动作抓取的「回退间距分量 + 放置」段（2026-11 加, 仅 place≠0 时走）:
 * 与色球块⑥同口径 —— 车从"对齐位+间距"只回退**间距分量**回到中间对齐位
 * （= 爆爆桶所在位; 中间人质分量=0 → 一步不动, 对齐 dx/dy 的修正刻意保留）,
 * 从回退起点 xT1 起算 ulPlaceDelayMs 毫秒投放置动作（0=回退同时放）。
 * 回退串行在抓取臂跑完之后（爪里已有人质）, 调用方保证先 ucOneGrabWaitArm。
 * 放置动作播完立即返回, **不做放置后收敛**（需要收敛的路线自己在块后排）。
 * 返回 1 = 继续（臂等待超时降级也返回 1, 只留痕）, 0 = 被中止。
 */
static uint8_t ucHstgOneGrabBackAndPlace(float fSpdSideOut, float fSpdFwdOut,
                                         uint32_t ulPlaceDelayMs,
                                         emArmActionIdTdf emPlace)
{
    TickType_t xT1 = xTaskGetTickCount();   /* 回退起点: 放置延时的锚 */
    uint8_t    ucIssued = 0u;
    uint8_t    ucArmSeen = 0u;
    uint8_t    ucOk = 1u;

    if (fSpdSideOut != 0.0f)
    {
        ucOk = ucOneGrabAxisWithArmDelay(1u, -fSpdSideOut, (int)iGetHsAlignAccel(),
                                         (int)iGetHsAlignSpeed(), xT1, ulPlaceDelayMs,
                                         &ucIssued, &ucArmSeen, emPlace);
    }
    else if (fSpdFwdOut != 0.0f)
    {
        ucOk = ucOneGrabAxisWithArmDelay(0u, -fSpdFwdOut, (int)iGetHsAlignAccel(),
                                         (int)iGetHsAlignSpeed(), xT1, ulPlaceDelayMs,
                                         &ucIssued, &ucArmSeen, emPlace);
    }
    if (ucOk == 0u)
    {
        vRpiSendDebug("HSTG1 abort: stop by request");
        return 0u;
    }
    if (ucIssued == 0u)
    {
        /* 中间人质=一步不动（回退没跑）或延时比回退估时还长: 补等到点再发 */
        if (ucOneGrabArmIssueAt(xT1, ulPlaceDelayMs, &ucIssued, &ucArmSeen, emPlace) == 0u)
        {
            vRpiSendDebug("HSTG1 abort: stop by request");
            return 0u;
        }
    }
    /* 放置动作播完立即返回: 不再做放置后收敛（需要收敛的路线自己在后面排一块）。 */
    if (ucOneGrabWaitArm(ucArmSeen) == 0u)
    {
        vRpiSendDebug("HSTG1 arm wait timeout/abort (continue)");
    }
    return 1u;
}

/*
 * 色球单动作抓取的「识别定位」段（2026-10-30 从 ucRpiBallGrabOne 原文搬出, 逻辑一字未改）:
 *   ② 发 "5" 识别 + 按二维码首位定位（失败带方向 → 拉回一步重扫一次）
 *   ③ [mode=1] 视觉 yaw 转正 + 复测 → 末尾中止位检查。
 * 返回 1 = 定位成功（*piIdx = 目标序号 0左/1中/2右, pastT 已填）; 0 = 失败/中止（调试串已发）。
 * 各步机理见 ucRpiBallGrabOne 函数头 ②③, 那边不再重复。
 */
static uint8_t ucBallGrabOneLocateInner(stColorTgtTdf *pastT, uint32_t ulTimeoutMs,
                                        uint8_t ucMode, uint8_t ucRev, int *piIdx)
{
    int32_t iQr;
    uint8_t ucWant;
    uint8_t ucI;
    int     iIdx = -1;
    char    acDbg[RPI_DATA_MAX + 8u];

    /* ② 识别 + 按二维码定位（与排爆那块同一套键: 首位=色, 100~399 才有效）。
     * 识别失败且 Pi 给了方向（剩两球、缺端球在画面外）→ 开环拉回一步再试一次
     * （2026-10-26 加, 与 ALIGN_PULLBACK 同款: 主任务路线本步常是唯一识别入口,
     * 不接拉回则车偏出画面时一步死）。无方向/重扫仍失败 → 照旧失败返回。 */
    if (!ucBallScanOnce(pastT, ulTimeoutMs))
    {
        if (ucBallPullbackOnce(ucRev) == 0u)
        {
            vRpiSendDebug("BALL1 fail: scan");
            return 0u;
        }
        if (!ucBallScanOnce(pastT, ulTimeoutMs))
        {
            vRpiSendDebug("BALL1 fail: scan(after pull)");
            return 0u;
        }
    }
    iQr = iGetQrCode();
    if ((iQr < 100) || (iQr > 399)) { return 0u; }
    ucWant = (uint8_t)(iQr / 100);
    if ((ucWant < 1u) || (ucWant > 3u)) { return 0u; }
    for (ucI = 0u; ucI < (uint8_t)COLOR_MAX_COUNT; ucI++)
    {
        if (pastT[ucI].ucColor == ucWant) { iIdx = (int)ucI; break; }
    }
    if (iIdx < 0) { return 0u; }
    (void)snprintf(acDbg, sizeof(acDbg), "BALL1 tgt: color=%u pos=%u",
                   (unsigned)ucWant, (unsigned)iIdx);
    vRpiSendDebug(acDbg);

    /* ③ mode=1: 视觉 yaw 转正 + 复测（逐句照 ucRpiBallAlign 的对应段, 那边的注释同样成立）。
     * rev=1: 相机朝另一边看时图像里的连线角符号镜像 → 修正量取反（ball_yaw_sign 不动）。 */
    if (ucMode == 1u)
    {
        if (ucGetBallYawEn() != 0u)
        {
            float fYawVeh = (fGetBallYawLine() - fGetBallYawTarget())
                            * fGetBallYawScale() * fGetBallYawSign()
                            * ((ucRev != 0u) ? -1.0f : 1.0f);

            if ((fYawVeh > fGetBallYawDeadband()) || (fYawVeh < -fGetBallYawDeadband()))
            {
                char acYaw[12];
                bt_fmt_f2(acYaw, (int)sizeof(acYaw), fYawVeh);
                (void)snprintf(acDbg, sizeof(acDbg), "BALL1 turn: %sdeg(veh)", acYaw);
                vRpiSendDebug(acDbg);

                vPidSpinTurn((fYawVeh > 0.0f) ? 1 : 0,
                             (fYawVeh >= 0.0f) ? fYawVeh : -fYawVeh, 0u);
                if (!ucBallScanOnce(pastT, ulTimeoutMs))
                {
                    vRpiSendDebug("BALL1 fail: rescan");
                    return 0u;
                }
            }
        }
        else
        {
            vRpiSendDebug("BALL1 yaw: ball_yaw_en=0, skip turn");
        }
    }

    if (ucGetScriptAbortReq() != 0u)
    {
        vRpiSendDebug("BALL1 abort: stop by request");
        return 0u;
    }

    *piIdx = iIdx;
    return 1u;
}

/*
 * 上面的壳: 整个识别段包在「OpenMV 补光临时压灭 → 延时 → 识别 → 恢复」里
 * （BALL_LED_OFF_EN, 2026-10-30 加）: 场地换边后 OpenMV 的 LED 热点进了主摄视野,
 * 冲淡色球识别的 HSV 阈值 —— 灭灯识别、识别完（**或放弃**）立刻恢复成 Pi 配置里
 * 记住的亮度, 后面 mode=2 的并行收敛/路线巡线照常用灯（"兼得", 用户 2026-10-30 定）。
 * ⚠️ **为什么包一层而不是逐处补**: 这段的提前 return 有一大把（首扫失败/拉回后仍失败/
 *    二维码越界/没这颜色/转正复测失败/中止）, 逐处补 = 将来谁加一条出口就漏一处,
 *    症状是"灯一直灭着"（自愈要等下一次巡线起流被 Pi 对齐回来 —— 还自愈得慢）。
 *    结构上包一层就不会漏, 与 ucRpiRoadAlign/ucRpiRoadYaw 的补光壳同款。
 * ⚠️ 关灯用 vRpiOpenmvLed(0)（"L,0"）、恢复用 vRpiOpenmvLedRestore()（裸 "L"）——
 *    两者都**幂等**、都**不写** Pi 的 config; 刻意**不用** "T"（有状态翻转,
 *    灯若已经是灭的, 它反而会点亮）。
 * ⚠️ 识别期间 OpenMV 流是**停的**（mode=2 的收敛在进本段前已收流）, 灭灯对它无感;
 *    BALL_LED_OFF_DELAY_MS 给主摄**自动曝光**留出适应"灯灭了"的时间（AE 追稳前那几帧
 *    颜色会漂, 而标定是开灯时做的 —— 机理同 ROAD_LED_OFF_DELAY_MS）; 只在进门等一次,
 *    段内重扫（拉回后/转正后）不重复等。纯阻塞等待（让出式, 不查中止位 —— 有界固定时长,
 *    同看路那对的口径）, 不占积木 I(0) 超时预算。
 * ⚠️ EN=0 = 不碰补光灯（老行为逐字节同形）。 */
static uint8_t ucBallGrabOneLocate(stColorTgtTdf *pastT, uint32_t ulTimeoutMs,
                                   uint8_t ucMode, uint8_t ucRev, int *piIdx)
{
#if (BALL_LED_OFF_EN != 0)
    uint8_t ucRet;

    vRpiOpenmvLed(0u);              /* 关: OpenMV 的 LED 热点在主摄视野里冲淡色球 HSV 阈值 */
    if ((uint32_t)BALL_LED_OFF_DELAY_MS > 0u)
    {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)BALL_LED_OFF_DELAY_MS));
    }
    ucRet = ucBallGrabOneLocateInner(pastT, ulTimeoutMs, ucMode, ucRev, piIdx);
    vRpiOpenmvLedRestore();         /* 开: 唯一出口, 各失败路径也照恢复 */
    return ucRet;
#else
    return ucBallGrabOneLocateInner(pastT, ulTimeoutMs, ucMode, ucRev, piIdx);
#endif
}

/*
 * 色球单动作抓取（STEP_BALL_GRAB_ONE_WAIT 的原语, 流程见 app_rpi.h 的声明处说明）:
 *   ① [mode=2] 先巡线收敛把车摆正（前提=该区看得到线; 听谁由积木参数 src 选:
 *      0=相机/1=陀螺(咬 fGyroTarget)/2=不修, 2026-10-07 加, 之前写死相机）
 *   ② 发 "5" 拿三球 → 二维码首位(目标色)定位 iIdx（照排爆那块的 ①②, 颜色不随车动变）
 *      [2026-10-26 加] 识别失败且 Pi 失败帧带方向（剩两球/缺端球出画面）→ 开环
 *      拉回一步（ucBallPullbackOnce, 与 ALIGN_PULLBACK 同款）再重扫一次; 无方向
 *      或重扫仍失败才放弃 —— 主任务路线本步常是唯一识别入口, 不接拉回则偏出画面
 *      一步死（"宁可赌也不空跑", 用户 2026-10-26 定）
 *      [2026-10-30 加] 整个识别段（②③, 见 ucBallGrabOneLocate）包在「OpenMV 补光
 *      临时压灭/恢复」里: 场地换边后 LED 热点进了主摄视野, 灭灯识别、识别完立刻点回
 *   ③ [mode=1] ball_yaw_en=1 且超死区 → vPidSpinTurn 转正 + 复测（照 ucRpiBallAlign:
 *      转正视差已含在复测坐标里）; 开关没开则退化为不转正并打一行日志
 *   ④ 中球偏差 dx/dy（BALL_* 标定, 与 ucRpiBallAlign 同式）+ 平移分量 c_fOneGrabLatMm(iIdx)
 *      → 车体横移/纵移到位 —— 目标球此刻就在"中间球"的录制位姿正前方
 *   ⑤ 平移 × **剩提前量异步抓取**（并行; 2026-10-20 由"起点后延时"改）: 底盘先动,
 *      估时倒计时掉到 grab_lead_ms（再垫 pad）以下才把抓取动作投给臂泵 —— "开始"
 *      锚定**到位时刻**而不是起点: 短行程不白等, 长行程不会车未到爪先伸。
 *      grab_lead_ms = 动作里伸爪前段的时长（动作属性, 与平移距离无关）; 0=到位后才发。
 *      短行程另受编译期 ONE_GRAB_MIN_DELAY_MS 最短延迟兜底（0=同时开始）。
 *      本步**不等它开始、只等它结束**。
 *   ⑥ [mode=2] 并行巡线收敛（底盘已空闲, 与臂仍在播的动作并行）→ 等臂空（球已抓牢）
 *   ⑦ **只回退间距分量**（2026-10-03 用户定, 别改回整段回退）× **起点后延时**异步放置
 *      （2026-10-23 放置改回旧语义: 回退通常比放置动作短, "剩提前量"首拍就命中=没作用;
 *      place_delay_ms 从回退起点起算, 0=回退同时放）:
 *      侧球时车从"对齐位+间距"回到**中间对齐位**
 *      （= 爆破桶所在位）, 中间球分量=0 → 一步不动; 对齐 dx/dy 的修正**刻意保留**
 *      —— 车就该停在对齐好的位置（旧版整段回退会溜回块前的偏移位, 真踩过）。
 *      ⚠️ 间距回退只在"爪里已有球、臂已抬起"之后走, 所以它排在等臂空之后。
 *      ⚠️ 放置动作播完立即返回: **不再做放置后收敛**（用户 2026-10-23 定）;
 *         需要收敛的路线自己在这块后面排「巡线收敛」。
 *      ⚠️ place=0（控制台「无」, 2026-11 加）: **只回退、不投臂** —— 决赛若改成
 *         「色球只抓不放」, 车停位（中间对齐位）与 ⑧ 的残留清零跟原行为一致,
 *         路线几何不用重排; 单步确定的最后一段（放到爆爆桶）也不存在。
 *   ⑧ 清掉「路径残留偏移」: 净位移=对齐修正（定义上就是把车摆回标定位）, 留着
 *      旧值只会让后面的世界系平移误吃
 * 失败音: 识别失败/超时各响一声失败音（ucRpiWaitColorResult 投, 拉回重扫也各算一次）;
 *   数据不全不动车; 臂等待超时(降级, 继续走)不响（调试串留痕）。
 * 调试串前缀 BALL1（老对位链是 BALL, 别混读）。仅任务上下文调用。
 *
 * ⚠️ ucRev「反向执行」（2026-10-30 加, 积木 int 尾槽勾选）: 道具（球排+爆破桶）换到路的
 *    另一边、相机/臂朝另一边看 —— 图像相对车体的两个轴向映射与"图像左=排的哪一头"
 *    整体镜像。勾上时本步方向**整体取反**: 对位 dx/dy 的方向符号（BALL_SIDE_SIGN /
 *    ball_fwd_sign 的**本步有效值**, 蓝牙标定值不动）、间距分量、识别失败拉回、
 *    mode=1 的视觉 yaw 转正。⚠️ 只翻方向不翻轴（one_lat_dir 的 1/2 vs 3/4 选轴不受影响）;
 *    像素空间的标定（cx/cy 中心、px↔mm 比例尺）随镜像重标 —— 用「标定」Tab 换预设,
 *    与本开关无关。**残留不受影响**: ⑧照旧清零。
 */
uint8_t ucRpiBallGrabOne(uint32_t ulTimeoutMs, uint8_t ucMode,
                         emArmActionIdTdf emGrab, emArmActionIdTdf emPlace,
                         uint32_t ulGrabLeadMs, uint32_t ulPlaceDelayMs,
                         float fGyroTarget, uint8_t ucSrc, uint8_t ucCamPct,
                         uint8_t ucRev, uint8_t ucSingleStep)
{
    stColorTgtTdf astT[COLOR_MAX_COUNT];
    float   fDxMm, fDyMm, fLatMm;
    float   fSideSign, fFwdSign;                      /* 对位方向的有效符号（rev=1 整体取反） */
    float   fSideOut = 0.0f, fFwdOut = 0.0f;          /* ④ 整段平移的执行量（对位+间距） */
    float   fSpdSideOut = 0.0f, fSpdFwdOut = 0.0f;    /* 其中**间距分量**的执行量（⑥只回退这部分） */
    uint8_t ucSideMove, ucFwdMove;                    /* 各轴是否超死区（合成值判定） */
    uint8_t ucIssued, ucOk, ucArmSeen;                /* 发臂守卫 / 中止标志 / "动作已开始"闩锁 */
    int     iIdx = -1;                                /* 识别定位段的输出（ucBallGrabOneLocate） */
    char    acDbg[RPI_DATA_MAX + 8u];

    if ((!ucArmActionExists(emGrab))
        || (((uint8_t)emPlace != 0u) && (!ucArmActionExists(emPlace))))
    {
        vRpiSendDebug("BALL1 fail: action id not found");
        return 0u;
    }

    /* ① mode=2: 识别前先收敛（识别本身不动车, 收敛失败也只是少一道纠正, 照常往下）。
     * 收敛听谁 = 积木参数 src（档位同「巡线收敛」: 0=相机/1=陀螺/2=不修/3=融合, 2026-10-07 加
     * —— 相机侧看不清线或用户想咬死世界航向时选 1, 咬 fGyroTarget（陀螺/融合档都咬它,
     * 融合档 = 陀螺腿的目标）; 越界由内层 ucRpiLineSrcResolve 拒绝兜底）。⚠️ 之前写死相机, 用户现场要求可选。 */
    if (ucMode == 2u)
    {
        (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                               ucCamPct);
    }

    /* ② 识别 + 按二维码定位 + [mode=1 转正复测] → 整段见 ucBallGrabOneLocate（壳）/
     *    ucBallGrabOneLocateInner（原文）。⚠️ 2026-10-30 起识别窗口包在「OpenMV 补光
     *    临时压灭/恢复」里: 场地换边后 LED 热点进了主摄视野、冲淡色球 HSV 阈值 ——
     *    灭灯识别、识别完（或放弃）立刻按 Pi 配置亮度点回, 后面 mode=2 的并行收敛
     *    照常用灯。失败/中止 → 原样返回 0（调试串在内层已发）。 */
    if (ucBallGrabOneLocate(astT, ulTimeoutMs, ucMode, ucRev, &iIdx) == 0u)
    {
        return 0u;
    }

    /* ④ 中球偏差 dx/dy（BALL_* 标定）+ 平移分量 → 出动（实际执行量记账, 供 ⑥ 反向）。
     * 对位方向的有效符号: rev=1（道具换到路的另一边、相机/臂朝另一边看）时整体取反 ——
     * 偏差→车体平移的映射随镜像翻转; 间距与拉回同翻（全局标定值不动, 老场地预设不勾即可）。 */
    fSideSign = BALL_SIDE_SIGN;
    fFwdSign  = fGetBallFwdSign();
    if (ucRev != 0u) { fSideSign = -fSideSign; fFwdSign = -fFwdSign; }
    if (ucGetBallXySwap() != 0u)
    {
        fDxMm = ((float)astT[1].usCy - fGetBallCyCenter()) * fGetBallPxToMmY() * fSideSign;
        fDyMm = (fGetBallCxCenter() - (float)astT[1].usCx) * fGetBallPxToMmX() * fFwdSign;
    }
    else
    {
        fDxMm = ((float)astT[1].usCx - fGetBallCxCenter()) * fGetBallPxToMmX() * fSideSign;
        fDyMm = (fGetBallCyCenter() - (float)astT[1].usCy) * fGetBallPxToMmY() * fFwdSign;
    }
    fLatMm = c_fOneGrabLatMm(iIdx);
    if (ucRev != 0u) { fLatMm = -fLatMm; }   /* 反向: 间距分量往另一边叠加（选轴不变, 只翻符号） */
    {
        uint8_t ucLatOnSide = ((ucGetOneLatDir() == 1u) || (ucGetOneLatDir() == 2u)) ? 1u : 0u;
        float   fSide = fDxMm + ((ucLatOnSide != 0u) ? fLatMm : 0.0f);
        float   fFwd  = fDyMm + ((ucLatOnSide != 0u) ? 0.0f : fLatMm);
        char    acDx[12];
        char    acDy[12];
        char    acLat[12];

        bt_fmt_f2(acDx,  (int)sizeof(acDx),  fSide);
        bt_fmt_f2(acDy,  (int)sizeof(acDy),  fFwd);
        bt_fmt_f2(acLat, (int)sizeof(acLat), fLatMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "BALL1 fix: dx=%smm dy=%smm lat=%smm", acDx, acDy, acLat);
        vRpiSendDebug(acDbg);
        (void)snprintf(acDbg, sizeof(acDbg), "BALL1 lead: grab=%lums place_dly=%lums",
                       (unsigned long)ulGrabLeadMs, (unsigned long)ulPlaceDelayMs);
        vRpiSendDebug(acDbg);

        /* 死区判的是合成值: 合成没超死区的轴整段不动（对位+间距都没走） */
        ucSideMove = ((fSide > fGetBallDxDeadband()) || (fSide < -fGetBallDxDeadband())) ? 1u : 0u;
        ucFwdMove  = ((fFwd  > fGetBallDyDeadband()) || (fFwd  < -fGetBallDyDeadband())) ? 1u : 0u;
        fSideOut = (ucSideMove != 0u) ? fSide : 0.0f;
        fFwdOut  = (ucFwdMove  != 0u) ? fFwd  : 0.0f;
        /* 间距分量的执行量跟随所在轴: 该轴执行了它就跟着进去了, 没执行它也没动 */
        fSpdSideOut = (ucSideMove != 0u) ? (fSide - fDxMm) : 0.0f;
        fSpdFwdOut  = (ucFwdMove  != 0u) ? (fFwd  - fDyMm) : 0.0f;
    }

    if (ucSingleStep != 0u)
    {
        /* 单步确定（2026-11 加）: 色球对位→[MAIN]→移动→[MAIN]→抓取→[MAIN]→放到爆爆桶
         * (最后一段仅 place≠无 时存在)。每段结束先停车等 MAIN; 勾选后抓取不再按
         * grab_lead_ms 与移动并行（移动已停稳、操作员按 MAIN 后才发）,
         * 放置仍按 place_delay_ms 从回退起点计时。 */
        vRpiSendDebug("BALL1 step: align done, wait MAIN");
        if (ucOneGrabStepWait() == 0u) { return 0u; }

        /* 移动段: 只平移、不发臂（两轴次序与老路径一致: 横移在前, 纵移在后） */
        if (ucSideMove != 0u)
        {
            if (ucOneGrabMoveAxis(1u, fSideOut, (int)iGetBallAlignAccel(),
                                  (int)iGetBallAlignSpeed()) == 0u)
            {
                vRpiSendDebug("BALL1 abort: stop by request");
                return 0u;
            }
        }
        if (ucFwdMove != 0u)
        {
            if (ucOneGrabMoveAxis(0u, fFwdOut, (int)iGetBallAlignAccel(),
                                  (int)iGetBallAlignSpeed()) == 0u)
            {
                vRpiSendDebug("BALL1 abort: stop by request");
                return 0u;
            }
        }
        /* mode=2 的「抓取时并行收敛」归到移动段: 车已停、臂还没动, 收敛先修横向残差 */
        if (ucMode == 2u)
        {
            (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                                   ucCamPct);
        }
        vRpiSendDebug("BALL1 step: move done, wait MAIN");
        if (ucOneGrabStepWait() == 0u) { return 0u; }

        /* 抓取段: 立即发（不再有移动可并行）, 等臂跑完 */
        ucIssued  = 0u;
        ucArmSeen = 0u;
        ucOneGrabArmIssueNow(&ucIssued, &ucArmSeen, emGrab);
        if (ucOneGrabWaitArm(ucArmSeen) == 0u)
        {
            vRpiSendDebug("BALL1 arm wait timeout/abort (continue)");
            if (ucGetScriptAbortReq() != 0u) { return 0u; }
        }
        if ((uint8_t)emPlace != 0u)
        {
            vRpiSendDebug("BALL1 step: grab done, wait MAIN");
            if (ucOneGrabStepWait() == 0u) { return 0u; }
        }
    }
    else
    {
        /* ⑤ 平移 × 剩提前量异步抓取（并行）: 底盘先动, 估时倒计时掉到 grab_lead_ms
         *    （再垫 pad）以下才把抓取动作投给臂泵 —— "开始"锚定**到位时刻**而不是起点,
         *    短行程不白等、长行程不会车未到爪先伸; 本步**不等它开始、只等它结束**
         *    （ucOneGrabWaitArm）。grab_lead_ms = 动作里伸爪前段的时长（与距离无关）。 */
        {
            TickType_t xT0 = xTaskGetTickCount();   /* 整段平移起点: 最短延迟(ONE_GRAB_MIN_DELAY_MS)的锚 */
            uint32_t ulFwdEst = 0u;

            ucIssued  = 0u;
            ucArmSeen = 0u;
            ucOk      = 1u;
            /* 两轴都动时横移在先: 先把纵移估时算好递进去, 横移期间的倒计时才锚定最终到位 */
            if ((ucSideMove != 0u) && (ucFwdMove != 0u))
            {
                ulFwdEst = (uint32_t)fChassisMoveTimeEst(fFwdOut, (uint16_t)iGetBallAlignAccel(),
                                                         (uint16_t)iGetBallAlignSpeed(), AXIS_Y);
            }
            if (ucSideMove != 0u)
            {
                ucOk = ucOneGrabAxisWithArm(1u, fSideOut, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed(),
                                            xT0, ulGrabLeadMs, ulFwdEst, &ucIssued, &ucArmSeen, emGrab);
            }
            if ((ucOk != 0u) && (ucFwdMove != 0u))
            {
                ucOk = ucOneGrabAxisWithArm(0u, fFwdOut, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed(),
                                            xT0, ulGrabLeadMs, 0u, &ucIssued, &ucArmSeen, emGrab);
            }
            if (ucOk == 0u)
            {
                vRpiSendDebug("BALL1 abort: stop by request");
                return 0u;
            }
            if (ucIssued == 0u)
            {
                /* 两轴都被死区跳过（没动, 到位=现在）或 lead<垫底 pad 没赶上触发: 补发 */
                ucOneGrabArmIssueNow(&ucIssued, &ucArmSeen, emGrab);
            }
        }
        if (ucMode == 2u)
        {
            (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                                   ucCamPct);
        }
        if (ucOneGrabWaitArm(ucArmSeen) == 0u)
        {
            vRpiSendDebug("BALL1 arm wait timeout/abort (continue)");
            if (ucGetScriptAbortReq() != 0u) { return 0u; }
        }
    }

    /* ⑥ **只回退间距分量**（2026-10-03 用户定, 别改回整段回退）× **起点后延时**异步放置:
     *    侧球时车从"对齐位+间距"回到**中间对齐位**（= 爆破桶所在位）, 中间球分量=0 →
     *    一步不动; 对齐 dx/dy 的修正**刻意保留**。place_delay_ms 从回退起点(xT1)起算,
     *    0=回退同时放; 回退通常比放置动作短, 用"剩提前量"首拍就命中、没有意义,
     *    所以放置改回旧延时语义（2026-10-23 用户定）。
     *    ⚠️ place=0（控制台「无」, 2026-11 加）: 仍然**回退**间距分量（车停中间对齐位,
     *    路线几何与残留清零逻辑不变）, 只是**不投放置动作** —— 决赛「色球只抓不放」用。 */
    if ((uint8_t)emPlace != 0u)
    {
        TickType_t xT1 = xTaskGetTickCount();   /* 回退起点: 放置延时的锚 */

        ucIssued  = 0u;
        ucArmSeen = 0u;
        ucOk      = 1u;
        if (fSpdSideOut != 0.0f)
        {
            ucOk = ucOneGrabAxisWithArmDelay(1u, -fSpdSideOut, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed(),
                                             xT1, ulPlaceDelayMs, &ucIssued, &ucArmSeen, emPlace);
        }
        else if (fSpdFwdOut != 0.0f)
        {
            ucOk = ucOneGrabAxisWithArmDelay(0u, -fSpdFwdOut, (int)iGetBallAlignAccel(), (int)iGetBallAlignSpeed(),
                                             xT1, ulPlaceDelayMs, &ucIssued, &ucArmSeen, emPlace);
        }
        if (ucOk == 0u)
        {
            vRpiSendDebug("BALL1 abort: stop by request");
            return 0u;
        }
        if (ucIssued == 0u)
        {
            /* 中间球=一步不动（回退没跑）或延时比回退估时还长: 补等到点再发 */
            if (ucOneGrabArmIssueAt(xT1, ulPlaceDelayMs, &ucIssued, &ucArmSeen, emPlace) == 0u)
            {
                vRpiSendDebug("BALL1 abort: stop by request");
                return 0u;
            }
        }
        /* 放置动作播完立即返回: 不再做放置后收敛（需要收敛的路线自己在后面排一块）。 */
        if (ucOneGrabWaitArm(ucArmSeen) == 0u)
        {
            vRpiSendDebug("BALL1 arm wait timeout/abort (continue)");
        }
    }
    else
    {
        /* place=无: 只回退间距分量, 不投臂（回退仍串行在抓取臂跑完之后）。 */
        ucOk = 1u;
        if (fSpdSideOut != 0.0f)
        {
            ucOk = ucOneGrabMoveAxis(1u, -fSpdSideOut, (int)iGetBallAlignAccel(),
                                     (int)iGetBallAlignSpeed());
        }
        else if (fSpdFwdOut != 0.0f)
        {
            ucOk = ucOneGrabMoveAxis(0u, -fSpdFwdOut, (int)iGetBallAlignAccel(),
                                     (int)iGetBallAlignSpeed());
        }
        if (ucOk == 0u)
        {
            vRpiSendDebug("BALL1 abort: stop by request");
            return 0u;
        }
    }

    /* ⑧ 净位移=对齐修正（定义上就是把车摆回标定位）: 清残留, 防止上一圈人质留下
     *    的旧值被后面的世界系平移误吃 */
    vSetPathResidual(0.0f, 0.0f);

    return 1u;
}

/* 人质单动作抓取的「残留记账」（2026-11 从 ucRpiHostageGrabOne 抽出: 单步路径与
 * 老路径共用同一份口径）。只记实际平移量 fSideOut/fFwdOut; 与 mode=2 收敛的相互作用
 * 见 ucRpiHostageGrabOne 函数头。ucKeepRes=0 时什么都不做。 */
static void vHstgOneGrabRecordResidual(uint8_t ucKeepRes, uint8_t ucMode,
                                       float fSideOut, float fFwdOut)
{
    float fResWx, fResWy;
    char  acWx[12];
    char  acWy[12];
    char  acDbg[RPI_DATA_MAX + 8u];

    if (ucKeepRes == 0u) { return; }
    if ((ucMode == 2u)
        && ((ucGetOneLatDir() == 1u) || (ucGetOneLatDir() == 2u)))
    {
        /* 间距加在**横向**时, 收敛已把那一横向残差拉回线中心（视觉吸收）,
         * 再被世界系平移吃一次就是重复补偿 —— 这种编法把 keep_res 设 0。 */
        vRpiSendDebug("HSTG1 warn: lateral res absorbed by converge (keep_res=0?)");
    }
    vChassisBodyToWorld(fSideOut, fFwdOut, &fResWx, &fResWy);
    vSetPathResidual(fResWx, fResWy);
    bt_fmt_f2(acWx, (int)sizeof(acWx), fResWx);
    bt_fmt_f2(acWy, (int)sizeof(acWy), fResWy);
    (void)snprintf(acDbg, sizeof(acDbg), "HSTG1 res: wx=%smm wy=%smm", acWx, acWy);
    vRpiSendDebug(acDbg);
}

/*
 * 人质单动作抓取（STEP_HOSTAGE_GRAB_ONE_WAIT 的原语）:
 *   ① [mode=1/2] 识别前巡线收敛（0x02 帧没有连线角, 车头歪只能靠线或路线摆正;
 *      听谁由积木参数 src 选: 0=相机/1=陀螺(咬 fGyroTarget)/2=不修, 2026-10-07 加）
 *   ② 发 "4" 拿三只 → 二维码第三位(目标形状)定位 iIdx（照反恐那块的 ①②, 含 %10
 *      之后单独判 1~3 —— 390 的第三位是 0, 不判会落到越界形状码）
 *      [2026-10-26 加] 识别失败且失败帧恰带 1 只路标 → 单目标拉回（把它平移到
 *      理想列）再重扫一次: 是中间则车已对齐, 是端部则只偏 150mm 三只照样全在
 *      画面里, 重识别必然全出、后续对位自修（"宁可赌也不空跑", 用户 2026-10-26 定）
 *   ③ **中间那只**(index 1)的偏差 dx/dy（HOSTAGE_* 标定, 与 ucRpiHostageAlign 的
 *      idx=1 同式）+ 平移分量 → 车体横/纵移到位
 *   ④ 平移 × 剩提前量异步抓取（并行; grab_lead_ms 口径同色球块⑤ —— 锚定到位,
 *      = 动作伸爪前段时长, 0=到位后才发）
 *   ④′ [mode=2] 并行巡线收敛（底盘已空闲、臂还在播 —— 与色球块同口径,
 *      修掉开环平移的横向残差; 2026-10-06 加, 用户要求与色球抓取同等待遇）
 *   ⑤ [place=无 且 ucKeepRes=1] 实际平移量按当前指令航向折成世界系, 记进「路径残留偏移」
 *      （抓左/抓右的平移残留, 由后续世界系平移积木勾"叠加上次残留偏移"一次性吃掉）
 *   ⑤′ [place≠无, 2026-11 加] 等臂空后**只回退间距分量**到中间对齐位（=爆爆桶所在位）,
 *      从回退起点延时 place_delay_ms 投放置动作（0=回退同时放）, 播完清残留
 *      （净位移=对齐修正, 与色球块⑥⑧同口径; 此时 keep_res 不记账, 打一行警告）
 *   ⑥ 等臂空 → 结束。**place=无 时不复位、不回中**（复位由路线排, 与反恐那块同一条分工）。
 * ⚠️ keep_res 与"收敛"的关系**取决于间距加在哪根轴**（mode=2 的收敛与块后手排的
 *    收敛同论）: 收敛修的是**横向+航向** —— 间距加在横向(one_lat_dir=1/2)时横向
 *    残留会被视觉吸收, 再吃残留=重复补偿, 那种编法 keep_res 设 0; 间距加在纵向
 *    (3/4, 本车预期档)时收敛不修纵向, 残留照记照吃, 两者可共存。
 *    mode=2 + keep_res=1 + 横向间距 的组合会打一条警告串。
 * 失败音: 识别失败/超时各响一声（ucRpiWaitHostageResult 投）; 数据不全不动车;
 *   臂等待超时(降级, 继续走)不响（调试串留痕）。
 * 调试串前缀 HSTG1（老反恐是 HSTG）。仅任务上下文调用。
 *
 * ⚠️ ucRev「反向执行」（2026-10-30 加, 积木 int 尾槽勾选）: 道具（人质排）换到路的另一边、
 *    相机/臂朝另一边看 —— 图像相对车体的两个轴向映射与"图像左=排的哪一头"整体镜像。
 *    勾上时本步方向**整体取反**: 对位 dx/dy 的方向符号（hs_side_sign / hs_fwd_sign 的
 *    **本步有效值**, 蓝牙标定值不动）、间距分量、识别失败拉回。
 *    ⚠️ **残留自动跟着对称**: ⑤ 记的是实际位移折世界系, 反向后实际位移镜像、记出的残留
 *    镜像 —— 消费端（世界系平移勾"叠加上次残留偏移"= 计划−残留）公式一字不改, 同一个
 *    "抓左"从"离开时要多走"自动变成"少走"（2026-10-30 场地换边的验收判据）。
 *    ⚠️ 只翻方向不翻轴（one_lat_dir 的 1/2 vs 3/4 选轴不受影响）; 像素空间标定
 *    （cx/cy 中心、px↔mm）随镜像重标 —— 「标定」Tab 换预设, 与本开关无关。
 */
uint8_t ucRpiHostageGrabOne(uint32_t ulTimeoutMs, uint8_t ucMode,
                            emArmActionIdTdf emGrab, uint8_t ucKeepRes,
                            uint32_t ulGrabLeadMs, float fGyroTarget, uint8_t ucSrc,
                            uint8_t ucCamPct, uint8_t ucRev, uint8_t ucSingleStep,
                            emArmActionIdTdf emPlace, uint32_t ulPlaceDelayMs)
{
    stHostageTgtTdf astT[HOSTAGE_MAX_COUNT];
    float   fDxMm, fDyMm, fLatMm;
    float   fSideSign, fFwdSign;
    float   fSideOut = 0.0f, fFwdOut = 0.0f;
    float   fSpdSideOut = 0.0f, fSpdFwdOut = 0.0f;   /* 间距分量的执行量（place≠无 时回退这部分） */
    uint8_t ucSideMove, ucFwdMove;
    uint8_t ucArmSeen;                                /* "动作已开始"闩锁（④发臂时闩, ⑥消费） */
    int32_t iQr;
    uint8_t ucWant;
    uint8_t ucI;
    int     iIdx = -1;
    char    acDbg[RPI_DATA_MAX + 8u];

    if ((!ucArmActionExists(emGrab))
        || (((uint8_t)emPlace != 0u) && (!ucArmActionExists(emPlace))))
    {
        vRpiSendDebug("HSTG1 fail: action id not found");
        return 0u;
    }

    /* ① mode=1/2: 识别前先收敛（失败只是少一道纠正, 照常识别）。
     * 收敛听谁 = 积木参数 src（档位同「巡线收敛」含融合, 2026-10-07 加, 之前写死相机;
     * 陀螺/融合档咬 fGyroTarget（融合档 = 陀螺腿目标）, 越界由内层拒绝兜底）。 */
    if ((ucMode == 1u) || (ucMode == 2u))
    {
        (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                               ucCamPct);
    }

    /* ② 识别 + 按二维码第三位定位（与反恐那块同一套键与同一道防呆）。
     * 识别失败且失败帧恰带 1 只路标（Pi 失败帧带最好成绩, 2026-10-26 起）→
     * 单目标拉回（ucHstgPullToCenterOnce: 把它平移到理想列）再重扫一次;
     * 无路标/重扫仍失败 → 照旧失败返回。全程无蜂鸣（提示声用户自排）。 */
    if (!ucHostageScanOnce(astT, ulTimeoutMs))
    {
        if (ucHstgPullToCenterOnce(ucRev) == 0u)
        {
            vRpiSendDebug("HSTG1 fail: scan");
            return 0u;
        }
        if (!ucHostageScanOnce(astT, ulTimeoutMs))
        {
            vRpiSendDebug("HSTG1 fail: scan(after pull)");
            return 0u;
        }
    }
    iQr = iGetQrCode();
    if ((iQr < 100) || (iQr > 399)) { return 0u; }
    ucWant = (uint8_t)(iQr % 10);
    if ((ucWant < 1u) || (ucWant > (uint8_t)HOSTAGE_MAX_COUNT)) { return 0u; }
    for (ucI = 0u; ucI < (uint8_t)HOSTAGE_MAX_COUNT; ucI++)
    {
        if (astT[ucI].ucShape == ucWant) { iIdx = (int)ucI; break; }
    }
    if (iIdx < 0) { return 0u; }
    (void)snprintf(acDbg, sizeof(acDbg), "HSTG1 tgt: shape=%u pos=%u",
                   (unsigned)ucWant, (unsigned)iIdx);
    vRpiSendDebug(acDbg);

    if (ucGetScriptAbortReq() != 0u)
    {
        vRpiSendDebug("HSTG1 abort: stop by request");
        return 0u;
    }

    /* ③ 中间那只(index 1)偏差（hs_* 标定）+ 平移分量 → 出动（实际执行量记账）。
     * 对位方向的有效符号: rev=1（道具换到路的另一边、相机/臂朝另一边看）时整体取反 ——
     * 偏差→车体平移的映射随镜像翻转; 间距与拉回同翻（全局标定值不动）。 */
    fSideSign = fGetHsSideSign();
    fFwdSign  = fGetHsFwdSign();
    if (ucRev != 0u) { fSideSign = -fSideSign; fFwdSign = -fFwdSign; }
    if (ucGetHsXySwap() != 0u)
    {
        fDxMm = ((float)astT[1].usCy - fGetHsCyCenter()) * fGetHsPxToMmY() * fSideSign;
        fDyMm = (fGetHsCxCenter() - (float)astT[1].usCx) * fGetHsPxToMmX() * fFwdSign;
    }
    else
    {
        fDxMm = ((float)astT[1].usCx - fGetHsCxCenter()) * fGetHsPxToMmX() * fSideSign;
        fDyMm = (fGetHsCyCenter() - (float)astT[1].usCy) * fGetHsPxToMmY() * fFwdSign;
    }
    fLatMm = c_fOneGrabLatMm(iIdx);
    if (ucRev != 0u) { fLatMm = -fLatMm; }   /* 反向: 间距分量往另一边叠加（选轴不变, 只翻符号） */
    {
        uint8_t ucLatOnSide = ((ucGetOneLatDir() == 1u) || (ucGetOneLatDir() == 2u)) ? 1u : 0u;
        float   fSide = fDxMm + ((ucLatOnSide != 0u) ? fLatMm : 0.0f);
        float   fFwd  = fDyMm + ((ucLatOnSide != 0u) ? 0.0f : fLatMm);
        char    acDx[12];
        char    acDy[12];
        char    acLat[12];

        bt_fmt_f2(acDx,  (int)sizeof(acDx),  fSide);
        bt_fmt_f2(acDy,  (int)sizeof(acDy),  fFwd);
        bt_fmt_f2(acLat, (int)sizeof(acLat), fLatMm);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "HSTG1 fix: dx=%smm dy=%smm lat=%smm", acDx, acDy, acLat);
        vRpiSendDebug(acDbg);
        (void)snprintf(acDbg, sizeof(acDbg), "HSTG1 lead: grab=%lums", (unsigned long)ulGrabLeadMs);
        vRpiSendDebug(acDbg);

        /* 死区判的是合成值: 合成没超死区的轴整段不动 */
        ucSideMove = ((fSide > fGetHsDxDeadband()) || (fSide < -fGetHsDxDeadband())) ? 1u : 0u;
        ucFwdMove  = ((fFwd  > fGetHsDyDeadband()) || (fFwd  < -fGetHsDyDeadband())) ? 1u : 0u;
        fSideOut = (ucSideMove != 0u) ? fSide : 0.0f;
        fFwdOut  = (ucFwdMove  != 0u) ? fFwd  : 0.0f;
        /* 间距分量的执行量跟随所在轴: 该轴执行了它就跟着进去了, 没执行它也没动
         * （place≠无 时只回退这部分, 与色球块⑥同口径）。 */
        fSpdSideOut = (ucSideMove != 0u) ? (fSide - fDxMm) : 0.0f;
        fSpdFwdOut  = (ucFwdMove  != 0u) ? (fFwd  - fDyMm) : 0.0f;
    }

    if (ucSingleStep != 0u)
    {
        /* 单步确定（2026-11 加）: 人质对位→[MAIN]→移动→[MAIN]→抓取;
         * place≠无 时再加 →[MAIN]→放到爆爆桶（最后一段仍不等 MAIN）。
         * 每段结束先停车等 MAIN; 勾选后抓取不再按 grab_lead_ms 与移动并行。 */
        vRpiSendDebug("HSTG1 step: align done, wait MAIN");
        if (ucOneGrabStepWait() == 0u) { return 0u; }

        /* 移动段: 只平移、不发臂（横移在前, 与老路径同次序） */
        if (ucSideMove != 0u)
        {
            if (ucOneGrabMoveAxis(1u, fSideOut, (int)iGetHsAlignAccel(),
                                  (int)iGetHsAlignSpeed()) == 0u)
            {
                vRpiSendDebug("HSTG1 abort: stop by request");
                return 0u;
            }
        }
        if (ucFwdMove != 0u)
        {
            if (ucOneGrabMoveAxis(0u, fFwdOut, (int)iGetHsAlignAccel(),
                                  (int)iGetHsAlignSpeed()) == 0u)
            {
                vRpiSendDebug("HSTG1 abort: stop by request");
                return 0u;
            }
        }
        /* mode=2 的「等臂期间并行收敛」归到移动段（车已停、臂还没动） */
        if (ucMode == 2u)
        {
            (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                                   ucCamPct);
        }
        /* 残留记账只在「不放置」时做（place≠无 时车会回中, 结束时清残留）。 */
        if ((uint8_t)emPlace == 0u)
        {
            vHstgOneGrabRecordResidual(ucKeepRes, ucMode, fSideOut, fFwdOut);
        }
        else if (ucKeepRes != 0u)
        {
            vRpiSendDebug("HSTG1 warn: keep_res ignored (place set)");
        }
        vRpiSendDebug("HSTG1 step: move done, wait MAIN");
        if (ucOneGrabStepWait() == 0u) { return 0u; }

        /* 抓取段: 移动已停稳, 按 MAIN 后立即发 */
        ucArmSeen = 0u;
        {
            uint8_t ucIssued = 0u;

            ucOneGrabArmIssueNow(&ucIssued, &ucArmSeen, emGrab);
        }
        if (ucOneGrabWaitArm(ucArmSeen) == 0u)
        {
            vRpiSendDebug("HSTG1 arm wait timeout/abort (continue)");
            if (ucGetScriptAbortReq() != 0u) { return 0u; }
        }
        if ((uint8_t)emPlace == 0u)
        {
            return 1u;                                  /* 抓取是最后一段, 不再等 MAIN */
        }
        vRpiSendDebug("HSTG1 step: grab done, wait MAIN");
        if (ucOneGrabStepWait() == 0u) { return 0u; }
        if (ucHstgOneGrabBackAndPlace(fSpdSideOut, fSpdFwdOut, ulPlaceDelayMs, emPlace) == 0u)
        {
            return 0u;
        }
        /* 车已回中间对齐位（净位移=对位修正）: 清掉上一圈留下的残留。 */
        vSetPathResidual(0.0f, 0.0f);
        return 1u;
    }

    /* ④ 平移 × 剩提前量异步抓取（并行）: 底盘先动, 估时倒计时掉到 grab_lead_ms
     *    （再垫 pad）以下才把抓取动作投给臂泵 —— 锚定**到位时刻**, 口径与色球块⑤一致;
     *    本步**不等它开始、只等它结束**。 */
    {
        TickType_t xT0 = xTaskGetTickCount();   /* 整段平移起点: 最短延迟(ONE_GRAB_MIN_DELAY_MS)的锚 */
        uint32_t ulFwdEst = 0u;
        uint8_t  ucIssued = 0u;
        uint8_t  ucOk = 1u;

        ucArmSeen = 0u;
        /* 两轴都动时横移在先: 先把纵移估时算好递进去, 横移期间的倒计时才锚定最终到位 */
        if ((ucSideMove != 0u) && (ucFwdMove != 0u))
        {
            ulFwdEst = (uint32_t)fChassisMoveTimeEst(fFwdOut, (uint16_t)iGetHsAlignAccel(),
                                                     (uint16_t)iGetHsAlignSpeed(), AXIS_Y);
        }
        if (ucSideMove != 0u)
        {
            ucOk = ucOneGrabAxisWithArm(1u, fSideOut, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed(),
                                        xT0, ulGrabLeadMs, ulFwdEst, &ucIssued, &ucArmSeen, emGrab);
        }
        if ((ucOk != 0u) && (ucFwdMove != 0u))
        {
            ucOk = ucOneGrabAxisWithArm(0u, fFwdOut, (int)iGetHsAlignAccel(), (int)iGetHsAlignSpeed(),
                                        xT0, ulGrabLeadMs, 0u, &ucIssued, &ucArmSeen, emGrab);
        }
        if (ucOk == 0u)
        {
            vRpiSendDebug("HSTG1 abort: stop by request");
            return 0u;
        }
        if (ucIssued == 0u)
        {
            /* 两轴都被死区跳过（没动, 到位=现在）或 lead<垫底 pad 没赶上触发: 补发 */
            ucOneGrabArmIssueNow(&ucIssued, &ucArmSeen, emGrab);
        }
    }

    /* ④′ [mode=2] 并行巡线收敛（底盘已空闲、臂还在播, 与色球块同口径）:
     *    把开环平移的横向残差修掉。⚠️ 与 ⑤ 残留记账的相互作用见函数头 ——
     *    只修横向+航向, 不修纵向。 */
    if (ucMode == 2u)
    {
        (void)ucRpiLineConverge((uint32_t)iGetOneLineMs(), ucSrc, fGyroTarget, 0u,
                               ucCamPct);
    }

    if ((uint8_t)emPlace == 0u)
    {
        /* ⑤ 残留记账（单步/老路径共用, 见 vHstgOneGrabRecordResidual）:
         *    车体实际位移 → 世界系（按当前指令航向查一次逆表, 与消费端同一张表）。
         *    抓左/抓右那 ±one_lat_mm 就是"后面的路要多走/少走"的量, 记下来交给世界系平移。 */
        vHstgOneGrabRecordResidual(ucKeepRes, ucMode, fSideOut, fFwdOut);

        /* ⑥ 等臂空（"只等结束"）。不复位、不回中（见函数头）。 */
        if (ucOneGrabWaitArm(ucArmSeen) == 0u)
        {
            vRpiSendDebug("HSTG1 arm wait timeout/abort (continue)");
        }
        return 1u;
    }

    /* ⑤′ place≠无: 先等抓取臂跑完（爪里已有人质）, 再只回退间距分量到中间对齐位
     *    并投放置动作; 净位移=对位修正 → 结束时清残留。keep_res 在这个模式下不记账。 */
    if (ucKeepRes != 0u)
    {
        vRpiSendDebug("HSTG1 warn: keep_res ignored (place set)");
    }
    if (ucOneGrabWaitArm(ucArmSeen) == 0u)
    {
        vRpiSendDebug("HSTG1 arm wait timeout/abort (continue)");
        if (ucGetScriptAbortReq() != 0u) { return 0u; }
    }
    if (ucHstgOneGrabBackAndPlace(fSpdSideOut, fSpdFwdOut, ulPlaceDelayMs, emPlace) == 0u)
    {
        return 0u;
    }
    vSetPathResidual(0.0f, 0.0f);

    return 1u;
}


/* ==================== 靶标识别结果等待 ==================== */

/*
 * 等靶标识别结果帧（Pi tasks/_target.py 回传, TAG=0x04 二进制 COORD）:
 *   成功 = status=1（每颗靶的颜色/靶心/半径, 已由 Pi 按中心 x 升序排, 见 vGetTargetResult）;
 *   失败 = status=0（相机失败/超时/Pi 没找到靶）。返回 1=成功, 0=失败/超时。
 *   走独立 ready 标志（不碰 last packet / RpiFrameReady）, 不受坐标/QR/巡线帧干扰。
 *   失败/超时 → 投失败音（非阻塞; 中止不响 —— 停是操作员的意图）。
 *   仅任务上下文调用。
 */
uint8_t ucRpiWaitTargetResult(uint32_t ulTimeoutMs)
{
    uint32_t ulWaited = 0;
    uint8_t ucStatus = 0;

    (void)ucTakeTargetReady();              /* 清残留旧结果 */
    vSetTargetResult(0, 0, NULL);
    vSetTargetCam(0u);                      /* 伴随量跟着一起清（cam 与 status 是一组的） */

    while (ulWaited < ulTimeoutMs)
    {
        /* ⚠️ 查中止位（同 ucRpiWaitColorResult）: 标靶识别默认等 6~8s, 不查的话
         * 「停止 / 菜单请求」要等满它; 中止按"本次没结果"返回 0。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            vRpiSendDebug("TGTA abort: stop by request");
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ulWaited += 10;

        if (!ucTakeTargetReady()) { continue; }
        vGetTargetResult(&ucStatus, NULL, NULL);
        if (ucStatus != 1u)
        {
            vSetBuzzerReq(BUZZER_PAT_FAIL);     /* 识别失败: 失败音 */
            return 0u;
        }
        return 1u;
    }
    vSetBuzzerReq(BUZZER_PAT_FAIL);             /* 超时（Pi 没回帧/断链）: 同样响 */
    return 0;
}

/* ==================== 标靶对位（发靶标指令 + 等结果, 三颗齐才算数） ==================== */

/*
 * 识别一次靶标并**要求三颗齐全**。→ 1 = 成功且 astT 已填; 0 = 失败/不齐（原因进 Pi 日志）。
 *
 * ⚠️ **ucNeedAll=1（三颗齐）不能省**的场景: 本步取的是"第二个标靶"作基准（cx 升序的第 1 号）——
 *    只有三颗都在画面里时, "第二个"才真的是中间那颗。只检到两颗时, 右靶会被当成"第二个",
 *    于是**照着错误的位置把车横移过去**（而且不报错）。与 ucHostageScanOnce 的
 *    cnt==HOSTAGE_MAX_COUNT 是同一道门、同一个理由。
 * ⚠️ **ucNeedAll=0 的场景**: 调用方**按颜色找靶**、不依赖位置序号（平移射靶 SHOT）——
 *    目标色那颗在结果里就能干, 蓝靶被 Pi 侧散布作废只剩红绿时照样可用
 *    （cnt==3 的门在这里只会白白浪费一次识别: 目标色明明识别得很好, 却整步拒绝）。
 *    needAll=0 时只要求 status==1 且 cnt>=1, 目标色在不在由调用方自己找（找不到它有
 *    自己的失败串）。
 * ⚠️ 与 Pi 侧的 TARGET_MIN_TARGETS 是**两回事**: 那个是 Pi 判"识别任务成不成功"（默认 1,
 *    为的是瞄准时能凑近单个靶）, 它管的是**能不能用于对位**。两者可以不一致, 别去改那个。
 */
static uint8_t ucTargetScanOnce(stTargetTgtTdf *pastT, uint32_t ulTimeoutMs, uint8_t ucNeedAll,
                                uint8_t ucUsePicam)
{
    uint8_t ucStatus = 0;
    uint8_t ucCnt    = 0;
    char    acDbg[RPI_DATA_MAX];

    /* 等链路才发; 被顶掉/被中止 → 本次没结果（识别失败音由 ucRpiWaitTargetResult 投）
     * 超时随指令下发（同 ucRpiHostageScan 的理由, 见其注释）。
     * ⚠️ ucUsePicam（2026-10-16 加）: 只由「平移射靶」传 1 —— 其余用本函数的步骤
     *    （标靶对位/标靶校准）全程按主摄那套标定, **必须传 0**（见 app_rpi.h 的说明）。 */
    if (ucRpiSendTaskCmdWaitPi(RPI_TASK_TARGET, ulTimeoutMs, ucUsePicam) == 0u)
    {
        return 0u;
    }

    if (!ucRpiWaitTargetResult(ulTimeoutMs))
    {
        /* 把超时值一起打出来: 参数没配到（=0）时现象是"发完立刻判失败",
         * 与链路问题长得一模一样, 只有这个数分得清（同 ucHostageScanOnce）。 */
        vGetTargetResult(&ucStatus, &ucCnt, NULL);
        (void)snprintf(acDbg, sizeof(acDbg), "TGTA fail: wait(t=%lums) st=%u cnt=%u",
                       (unsigned long)ulTimeoutMs, (unsigned)ucStatus, (unsigned)ucCnt);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    vGetTargetResult(&ucStatus, &ucCnt, pastT);
    if ((ucStatus != 1u)
        || ((ucNeedAll != 0u) && (ucCnt != (uint8_t)TARGET_MAX_COUNT))
        || ((ucNeedAll == 0u) && (ucCnt == 0u)))
    {
        (void)snprintf(acDbg, sizeof(acDbg), "TGTA fail: st=%u cnt=%u (need %s)",
                       (unsigned)ucStatus, (unsigned)ucCnt,
                       (ucNeedAll != 0u) ? "1/3" : ">=1");
        vRpiSendDebug(acDbg);
        return 0u;
    }
    return 1u;
}

/*
 * 标靶对位一步到位（STEP_TARGET_ALIGN_WAIT 的原语）:
 *   ① 发靶标识别指令并等结果（**三颗齐才算数**, 见 ucTargetScanOnce）;
 *   ② 取第 ucIdx 颗（0=左/1=中/2=右, cx 升序）作基准;
 *   ③ 它的**横向**偏差 → 车体 mm → 横移底盘到位。
 *
 * 只做横向: 前后距离由路线排定, 本步不动它（人质/小球那两步是横+纵两轴, 见各自函数）。
 *
 * ⚠️ **不做转正**: 0x04 帧没有连线角字段, 而且三颗靶的半径/高度一致也说明不了车头朝向
 *    —— 车歪着的时候整排靶在画面里一起平移+透视, 单看一个点分不出来。
 *    要治车头朝向得另配一步（带 IMU 航向闭环的步骤）。
 * ⚠️ 参数越界（手写 map.c / 老参数错位）→ **不动车**（不响, 调试串留痕）, 不静默夹到 0/2
 *    —— 夹了就是「选的是左、对到的是右」。
 * ⚠️ 本步**不碰机械臂**, 故不进 c_bIsArmStep（与 ucRpiHostageAlign 同理）。
 * 返回 1 = 已按偏差横移（或已在死区内）; 0 = 数据不全/越界（未动车）。仅任务上下文调用。
 */
uint8_t ucRpiTargetAlign(uint32_t ulTimeoutMs, uint8_t ucIdx)
{
    stTargetTgtTdf astT[TARGET_MAX_COUNT];
    float fDxMm;
    char  acDbg[RPI_DATA_MAX];

    if (ucIdx >= (uint8_t)TARGET_MAX_COUNT)
    {
        vRpiSendDebug("TGTA fail: idx out of range");
        return 0u;
    }

    if (!ucTargetScanOnce(astT, ulTimeoutMs, 1u, 0u))   /* 标靶对位: 全程按主摄标定, 不用 pi */
    {
        vRpiSendDebug("TGTA fail: scan");   /* 具体原因由 ucTargetScanOnce 内部那条串说明 */
        return 0u;
    }

    (void)snprintf(acDbg, sizeof(acDbg), "TGTA meas: #%u col=%u(%u,%u) r=%u n=%u",
                   (unsigned)ucIdx, (unsigned)astT[ucIdx].ucColor,
                   (unsigned)astT[ucIdx].usCx, (unsigned)astT[ucIdx].usCy,
                   (unsigned)astT[ucIdx].usR, (unsigned)TARGET_MAX_COUNT);
    vRpiSendDebug(acDbg);

    /* 基准靶横向偏差 → 车体 mm。默认: 图像 X→横移(cx 驱动)。
     * ⚠️ xy_swap=1 时轴与系数整套对调（相机相对车体装了 90°）: 横移改由 cy 驱动、
     *    改用基准"行"；方向符号仍走 ta_side_sign（对调后可能要翻它 —— 现场试）。 */
    if (ucGetTaXySwap() != 0u)
    {
        fDxMm = ((float)astT[ucIdx].usCy - fGetTaCyCenter()) * fGetTaPxToMmX() * fGetTaSideSign();
    }
    else
    {
        fDxMm = ((float)astT[ucIdx].usCx - fGetTaCxCenter()) * fGetTaPxToMmX() * fGetTaSideSign();
    }

    /* 修正量先进 Pi 日志, 再动车（日志先发, 免得被 1s 级的平移阻塞垫后）。
     * 方向字母 = 实际要做的动作: R/L=右/左横移, skip=死区内不动。 */
    {
        char    acDx[12];
        uint8_t ucMove = ((fDxMm > fGetTaDxDeadband())
                          || (fDxMm < -fGetTaDxDeadband())) ? 1u : 0u;

        bt_fmt_f2(acDx, (int)sizeof(acDx), fDxMm);
        (void)snprintf(acDbg, sizeof(acDbg), "TGTA fix: #%u dx=%smm(%s)",
                       (unsigned)ucIdx, acDx,
                       ucMove ? ((fDxMm > 0.0f) ? "R" : "L") : "skip");
        vRpiSendDebug(acDbg);

        if (ucMove != 0u)
        {
            vChassisMoveCameraLateral(fDxMm, (int)iGetTaAlignAccel(), (int)iGetTaAlignSpeed());
        }
    }
    return 1u;
}

/* ==================== 靶标识别（组合动作） ==================== */

/*
 * 靶标识别一步到位（发指令 + 等结果 + 失败音）:
 *   结果在 Beam（vGetTargetResult 读: 状态 + 每颗靶的颜色/靶心/半径）。
 *   ⚠️ 只识别、**不动车也不动臂** —— 动臂的是 vTargetCalibAim。
 *   ⚠️ 识别期间机位必须静止（Pi 端多帧投票要求同一机位）。仅任务上下文调用。
 */
uint8_t ucRpiTargetScan(uint32_t ulTimeoutMs)
{
    uint8_t ucOk;

    /* 超时随指令下发（"6,ms", 见 ucRpiHostageScan 的理由）; 0 = 不带时限 */
    if (ucRpiSendTaskCmdWait(RPI_TASK_TARGET, ulTimeoutMs) == 0u)
    {
        return 0u;
    }
    ucOk = ucRpiWaitTargetResult(ulTimeoutMs);
    return ucOk;
}

/* ==================== 标靶校准（组合动作） ==================== */

/* ---- 瞄准点的距离视差补偿（见 app_config.h「距离视差」段头, 2026-10 加）----
 * fR = 当帧靶盘外圆等效半径[px]（0x04/0x05 帧都带, 现成的距离代理）。
 * EN=0（或锚点数<2）: 恒等于固定 tg_center 快照 —— **逐字节等同旧行为**;
 * EN=1: 在锚点表上**分段线性插值**（表按 R 升序填, r 越界钳到端点、不外推 ——
 *       标定范围之外宁可回到端点值, 别信线性外推）。
 * 两个消费者: vTargetCalibAim（0x04 终验/开环校准）与 ucRpiTargetServo（0x05 闭环）。 */
static void vTargetCenterForRadius(float fR, float *pfX, float *pfY)
{
#if (TARGET_CENTER_RCOMP_EN != 0) && (TARGET_CENTER_ANCHOR_N >= 2)
    static const float c_fR[TARGET_CENTER_ANCHOR_N] = TARGET_CENTER_ANCHOR_R;
    static const float c_fX[TARGET_CENTER_ANCHOR_N] = TARGET_CENTER_ANCHOR_CX;
    static const float c_fY[TARGET_CENTER_ANCHOR_N] = TARGET_CENTER_ANCHOR_CY;
    int i;

    if (fR <= c_fR[0])
    {
        *pfX = c_fX[0];
        *pfY = c_fY[0];
        return;
    }
    if (fR >= c_fR[TARGET_CENTER_ANCHOR_N - 1])
    {
        *pfX = c_fX[TARGET_CENTER_ANCHOR_N - 1];
        *pfY = c_fY[TARGET_CENTER_ANCHOR_N - 1];
        return;
    }
    for (i = 0; i < TARGET_CENTER_ANCHOR_N - 1; i++)
    {
        if (fR <= c_fR[i + 1])
        {
            float fT = (fR - c_fR[i]) / (c_fR[i + 1] - c_fR[i]);
            *pfX = c_fX[i] + fT * (c_fX[i + 1] - c_fX[i]);
            *pfY = c_fY[i] + fT * (c_fY[i + 1] - c_fY[i]);
            return;
        }
    }
    *pfX = c_fX[0];                      /* 不可达（上面已钳端点）, 纯兜底 */
    *pfY = c_fY[0];
#else
    (void)fR;
    *pfX = fGetTgCenterX();
    *pfY = fGetTgCenterY();
#endif
}

/*
 * 标靶校准一步到位（脚本动作 STEP_TARGET_CALIB_WAIT 的原语）:
 *   ① 读二维码 → **没认出来就整段不做**（拒绝返回, 不动机械臂, 留痕）;
 *   ② 取二维码**第二位** = 反恐靶颜色（1 红 / 2 绿 / 3 蓝, 与 color_t 同码）;
 *   ③ 发靶标识别指令并等结果（ucRpiTargetScan）;
 *   ④ 在结果里找目标色那颗（Pi 已按图像从左到右排序, 与二维码语义无关, 这里按色找）;
 *   ⑤ Δ = 靶心 − 瞄准点 → 乘比例尺 → 舵机6(画面X)/舵机3(画面Y) 微调。
 *
 * ⚠️ **本步的标定量全部是蓝牙可改的 Beam 量**（「蓝牙」Tab 的「标靶校准」组）:
 *    瞄准点 / 比例尺 / 方向 / 死区 / 步长上限 / 微调时长 / XY对调 / 识别超时 ——
 *    2026-09 之前它们只有 app_config.h 的编译期宏, 标定时每试一次都要重编译烧写,
 *    而这一步恰恰最需要反复试（先定方向、再定比例尺）。断电回宏的默认值。
 * ⚠️ **增量式微调**：vArmAimByPixels 以舵机**当前角度**为基准加偏移, 所以本动作天然可
 *    反复调用 —— 同一个积木连排两次就是两步迭代收敛, 不在函数里做循环。
 * ⚠️ **开环**（比例尺标定准不准决定落点）, 刻意不做视觉伺服闭环:
 *    Pi 端一帧要跑三色掩膜 + 多帧投票, 闭环滞后太大, 反而会摆振。
 * ⚠️ 任何一步数据不全都**原地放弃**, 绝不带病动机械臂（识别类失败有失败音）。
 * 仅任务上下文调用。
 */
uint8_t vTargetCalibAim(uint32_t ulTimeoutMs)
{
    int32_t iQr;
    uint8_t ucWant;
    uint8_t ucStatus = 0;
    uint8_t ucCnt = 0;
    stTargetTgtTdf astT[TARGET_MAX_COUNT];
    uint8_t ucI;
    uint8_t ucFound = 0;
    float fDx;
    float fDy;
    float fCenterX;
    float fCenterY;
    float fDeadband;
    uint32_t ulTimeout;

    /* 等结果帧的超时: **积木的 I(0) 优先**, 0（= 空槽/老行）才回落到蓝牙运行量
     * tg_calib_timeout_ms。⚠️ 以前这里只读 tg_*, 于是「标靶校准」积木那个 timeout_ms
     * 参数**完全没人消费** —— 控制台里改它不生效, map.c 里写的值也全被丢弃（不报错、
     * 只是"改了没用"）。现在两处都通。 */
    ulTimeout = (ulTimeoutMs != 0u) ? ulTimeoutMs : (uint32_t)iGetTgCalibTimeoutMs();

    /* 标定量**进门先取一次**（都是蓝牙可改的 Beam 量）。
     * 取成局部量的理由: ① 同一次调用里用同一个快照, 免得算到一半被人从蓝牙改掉;
     * ② 后面要拿同一个死区比两次, 逐次调用 getter 会读到两个可能不同的值。 */
    fCenterX  = fGetTgCenterX();
    fCenterY  = fGetTgCenterY();
    fDeadband = fGetTgDeadbandPx();

    /* ① 二维码没认出来 → 整段不做 */
    iQr = iGetQrCode();
    if (iQr < 0) { return 0; }

    /* ② 第二位 = 反恐靶颜色 */
    ucWant = (uint8_t)((iQr / 10) % 10);
    if ((ucWant < 1u) || (ucWant > 3u)) { return 0; }

    /* ③④ 识别 + 找目标色（ulTimeout 见函数头: 积木 I(0) 优先, 0 才用蓝牙量） */
    if (!ucRpiTargetScan(ulTimeout)) { return 0; }

    vGetTargetResult(&ucStatus, &ucCnt, astT);
    if (ucStatus != 1u) { return 0; }

    for (ucI = 0; ucI < ucCnt; ucI++)
    {
        if (astT[ucI].ucColor == ucWant) { ucFound = 1u; break; }
    }
    if (!ucFound) { return 0; }

    /* ⑤ 靶心 − 瞄准点 → 角度增量 → 微调（瞄准点/死区都是蓝牙可改的 Beam 量）。
     * EN=1 时瞄准点先按这颗靶的 r 做距离视差插值（EN=0 内部=上面的快照, 无变化） */
    vTargetCenterForRadius((float)astT[ucI].usR, &fCenterX, &fCenterY);
    fDx = (float)astT[ucI].usCx - fCenterX;
    fDy = (float)astT[ucI].usCy - fCenterY;
    if ((fDx > -fDeadband) && (fDx < fDeadband)) { fDx = 0.0f; }
    if ((fDy > -fDeadband) && (fDy < fDeadband)) { fDy = 0.0f; }

    if ((fDx == 0.0f) && (fDy == 0.0f))
    {
        return 1;
    }

    vArmAimByPixels(fDx, fDy);
    return 1;
}

/* ==================== 标靶流控闭环（新方案, 与上面开环迭代并存） ==================== */

/* 闭环排障用的节流与早退判据（见 ucRpiTargetServo 的「排障」那段说明）。
 * ⚠️ 过程串走 vRpiSendDebug。发送本身早已不是忙等（2026-09 起是 TX DMA 环, 见 app_uart.c）,
 *    但**仍然要节流**: 10Hz 逐拍打会把 RPI 链路流量和 Pi 侧日志一起刷爆, 而这条链路上
 *    同时还跑着 ~20Hz 的靶标流帧。 */
#define AIM_TRACE_EVERY_TICKS   3u   /* 每几拍打一条过程串（ctrl=100ms 时约 3Hz） */
#define AIM_NOPROG_TICKS        6u   /* 连续多少拍"偏差不下降"就判环没起作用 */
#define AIM_NOPROG_EPS_PX       1.0f /* 一拍的改善量小于此值就不算"在下降"（≈检测噪声） */
/* ---- 测量跳变拒收（2026-10-24 加, 只用于位移细调环 ucTargetShootCore）----
 * 一拍（tc_ctrl_ms≈180ms）内车最多走几 mm（≈几十 px）。**一拍跳上百 px** 只可能是
 * "这一拍量到的不是同一颗靶"—— 换了一颗、或者误检块顶替了它。现场实锤（2026-10-24
 * 16:59）: 主摄细调已经 `settled d=6px`，紧接着 pi(CSI) 起流抢 CPU → 红靶那一帧半径
 * 缩水 31%（90→62px）被 Pi 的相对半径门放过、又被本端的绝对半径闸拒掉 → iPick 退成
 * "离瞄准列最近的无色靶" → d 从 5 跳到 42/115/150px → P 环照单全收 → 行程累计 150mm
 * 撞行程闸冻结。
 * 处置: 把这一拍**当丢帧**（不计数、不出力、不累计行程）; 连着 AIM_JUMP_SKIP_MAX 拍都
 * 跳才采纳新值（真被外力挪走/换靶了也不至于卡死在那儿）。 */
#define AIM_JUMP_PX             120.0f /* 单拍 |Δd| 超过它就判"量错了"[px] */
#define AIM_JUMP_SKIP_MAX       2u     /* 连续跳过这么多拍后强制采纳（防死等） */

/* 底盘参与闭环（ts_chassis_en）的两条换挡判据 —— 放在这里与上面三条同一个归置点。 */
#define AIM_CHAS_HYST_PCT       150u /* 退出滞环 [%]: |dx| 要涨到门限的 1.5 倍才把 X 轴交回舵机 */
#define AIM_CHAS_DEBOUNCE       2u   /* 换挡去抖: 状态翻转需连续 N 拍成立（防门限附近执行器级 chatter）*/

/* 阶段化 + 踢步（2026-10-06 加; 参数面在 app_config.h「阶段化 X→Y」段, 这些是判据常数）。
 * 踢步/冻结的机理: 舵机3 无位置回读, 驱动模型照指令推进 —— 低于静摩擦的小步长会让
 * "指令目标"以每拍一步的速度从物理位置溜走, 突破静摩擦那拍一口气冲向溜走的终点
 * = 大角度超调。所以停滞先冻结（零增量, 不再累加）, 连续停滞够拍再踢一大步。 */
#define AIM_STALL_EPS_PX        1.0f /* 一拍 |dy| 改善小于此值算"停滞"（≈检测噪声+像素量化） */
#define AIM_KICK_ACC_PX         1.0f /* 踢步后 |dy| 改善≥此值才算"收货"（否则按浪费处理） */
#define AIM_KICK_MOVE_PX        0.5f /* 踢步后 |dy| 变化小于此值 = 舵机没动 → 模型退回踢前 */
#define AIM_KICK_BAND_SCALE     2.0f /* 踢步适用带 = 旧 conv_px × 本值: 带内 P 步长已低于静摩擦, 带外维持旧行为（noprog 兜底） */
#define AIM_DIST_MOVE_MAX_MM    15.0f/* 距离辅助单次上限 [mm]: 一次挪太多会大幅扰动 dx, 靠多次小步 */
#define AIM_DIST_MIN_MM         1.0f /* 距离辅助单次下限 [mm]: 再小就是抖动, 不挪 */
#define AIM_DIST_MAX_MOVES      2u   /* 距离辅助最多次数: 每次都要回 X 重修一圈, 时间上封顶 */

/* 阶段编号与踢步子状态（ts_seq_en=1 才用; 0 = 旧并行环, 阶段量不参与） */
#define TS_STAGE_X              1u   /* 阶段1: 只修左右（图像 cx; 舵机6→底盘接管, 现有机制） */
#define TS_STAGE_Y              2u   /* 阶段2: 只修上下（图像 cy; 舵机3 + 踢步 + 距离辅助） */
#define TS_KICKST_P             0u   /* 踢步子状态: 正常 P 律 */
#define TS_KICKST_EVAL          1u   /* 踢步子状态: 上一拍踢了, 本拍按图像裁决 */

/*
 * 标靶流控闭环（脚本动作 STEP_TARGET_SERVO_WAIT 的原语）:
 *   ① 读二维码 → 没认出来 / 第二位不在 1~3 就整段不做（拒绝, 一步不动臂, 留痕）;
 *   ② 进门取一次标定量快照（都是蓝牙可改的 Beam 量）;
 *   ③ 发 "7" 起流（Pi 起独立线程, 小跟踪窗内 ~20Hz 回传 TAG=0x05 流帧）;
 *   ④ 循环（每拍 ts_ctrl_ms）:
 *        新鲜度: 帧计数变了 → 刷新时刻; 停涨超 ts_stale_ms → **冻结**（convCnt 归零、
 *                本拍不动舵机）——注意是冻结而不是放弃: Pi 回来即续跑（同 app_line.c）;
 *        选靶:   ① color == 二维码第二位; ② 没有但有无色帧(color=0xFF) → 取离瞄准点
 *                最近的一颗（闭环本来就在把靶往瞄准点拉, 最近的那颗就是正在跟的）;
 *        偏差 dx/dy 两轴**都在** ts_conv_px 内 → convCnt++; 够 ts_conv_ticks 判收敛;
 *        否则 convCnt 归零, 按 Δ × tg 比例尺 × ts_kp_scale 算步长, 钳 ±ts_step_deg,
 *        累计行程超 ts_travel_deg 就拒绝放弃;
 *        两个舵机**各走一小步 S 曲线, 不等它到位** —— 节拍本身就是推进器,
 *        与 vArmAimByPixels 的 settle-wait 不同（这是连续闭环, 等一拍反而拖慢）。
 *   ⑤ **无论成败**（收敛 / 超时 / 行程超限）都发 "8" 停流 —— 放在唯一出口上,
 *      否则失败路径会把相机一直占着（Pi 侧还有 TARGET_STREAM_MAX_S 兜底, 那是最后一道）。
 *
 * 病根与处方（详见 app_config.h「标靶流控伺服」段头）: 老方案的开环执行链有三个随机源
 * （比例尺在非线性区失准且半收敛不报错 / 舵机回差使落点取决于最后一次逼近方向 /
 * 视差烤死在固定瞄准点）, 闭环把前两个**根治**（回差表现为死区, 环会持续推过去;
 * 比例尺失准只影响收敛速度、不影响终点）、第三个**缓解**（收敛姿态每次一致,
 * 残余视差变成常数偏移）。
 *
 * ⚠️ **瞄准点/比例尺/方向/XY对调共用 tg_***（与「标靶校准」同一套物理标定）——
 *    所以本步第一天就能继承已经标好的全部 tg_*, 不必重标。`ts_kp_scale` 是环路总增益,
 *    **振荡时降它**, 别去动那套共用的 tg 标定（动了会连带把开环那套也带偏）。
 * ⚠️ **P 环 + 三道阻尼, 刻意不做 I/D**: 靶静止、无负载扰动, P 到死区即收敛。
 *    单拍限幅 / 收敛阈值 / 连续拍数就是那三道阻尼。
 * ⚠️ **行程闸是安全夹, 不是精度旋钮**（对应老方案 TARGET_MAX_STEP_DEG 的教训）:
 *    闭环被一次误检带飞时会"越纠越偏", 累计行程一到就放弃, 代价限制在已知范围内。
 * ⚠️ 与 vTargetCalibAim 的**分工**: 本步读 0x05 流帧, 终验（STEP_TARGET_CALIB_WAIT）
 *    读 0x04 一次性结果 —— 两个快照刻意分开, 免得闭环停流时留下的最后一帧
 *    污染随后的终验（见 app_beam.h 的靶标流帧快照说明）。
 * ⚠️ **底盘参与闭环**（`ts_chassis_en`, 2026-09 加, 默认关）: 舵机6 颗粒度不够 + 有回差,
 *    细调段落点常在一环外且左右来回跳。开了之后 |dx| 落进 `ts_chassis_px` 以内时
 *    **舵机6 让位**（零增量软保持）, 改由**底盘位置模式小步"前后"进给**继续闭环
 *    （⚠️ 是**前后**不是左右: 相机装了 90°, 图像横向 cx 对应车体纵移 —— 见下发那段的说明）;
 *    画面纵向（图像 cy）全程归舵机3。
 *    换挡带迟滞（退门限 = 门限×1.5）与 2 拍去抖 —— 否则门限附近的单帧噪声会让两个执行器
 *    来回抢, 那正是要被消灭的那类 chatter。底盘阶段**不累计** noprog（底盘一拍的实际推进量
 *    可能小于 1px 的判据），安全兜底由 `ts_chassis_mm` 行程闸承担。
 *    ⚠️ 用位置模式而非速度模式的理由、以及 `en=0` 逐字节等同旧行为这条保证, 见
 *    app_config.h 的「底盘参与闭环」段头。底盘那一拍实际耗时 ≈ ctrl_ms + 平移阻塞 ≈ 2×,
 *    路线里那个 I(0) 要留余量（否则会被误诊成超时）。
 * ⚠️ **本循环带中止位检查**（`ucGetScriptAbortReq`, 位置在循环体最前面）—— 这是本函数
 *    原先**唯一漏掉**的一处「等外部事件的循环」, 加了底盘运动之后它从"停不下来"
 *    升级成安全问题（按了停、车还在走）。别删。
 * ⚠️ **阶段化 X→Y + 踢步 + 距离辅助**（`ts_seq_en`, 2026-10-06 加）: 开启后不再两轴并行,
 *    而是先只修左右（舵机6→底盘接管, 判据 ts_conv_x_px）→ 切只修上下（舵机3, 判据
 *    ts_conv_y_px, 比 conv_px 紧）。Y 停滞（连续 ts_stall_ticks 拍改善<1px）且
 *    |dy|<2×conv_px → 发一次 ts_kick_deg 大步（**不受 ts_step_deg 限幅**）, 下一拍按图像
 *    裁决（acc 收货 / rev 退回 / wst 原地重进, 全程最多 ts_kick_max 次）; 踢完仍卡 →
 *    **冻结模型**（堵"指令溜走"——舵机3 无位置回读, 见 app_config.h「阶段化 X→Y」段头）→
 *    |dy|<conv_px 宽容收货（打 TGTS grace 行）, 或距离辅助（ts_dist_*, 默认关, 开了就
 *    回 X 阶段重修一圈再回 Y）。过程行新增 st=y / kick / dist / grace / fin2, 旧的
 *    TGTS cfg/n=/fin/ok/fail/abort 格式逐字节不变。`ts_seq_en=0` 时本函数逐字节等同
 *    加这些之前的行为。
 * ⚠️ 只能任务上下文调用（内部 vTaskDelay）。返回 1=已收敛, 0=超时/行程超限/中止/数据不全。
 */
uint8_t ucRpiTargetServo(uint32_t ulTimeoutMs)
{
    /* ---- ② 进门取一次标定量快照（都是蓝牙可改的 Beam 量）----
     * 取成局部量的理由同 vTargetCalibAim: 同一次调用里用同一个快照, 免得算到一半
     * 被人从蓝牙改掉, 于是这一拍用的是一条控制律、下一拍换成另一条。 */
    float fCenterX  = fGetTgCenterX();
    float fCenterY  = fGetTgCenterY();
    float fKp       = fGetTsKpScale();
    float fStepMax  = fGetTsStepDeg();
    float fConvPx   = fGetTsConvPx();
    float fTravelMax= fGetTsTravelDeg();
    int32_t iConvTicks = iGetTsConvTicks();
    uint32_t ulCtrlMs  = (uint32_t)iGetTsCtrlMs();
    uint32_t ulMoveMs  = (uint32_t)iGetTsMoveMs();
    uint32_t ulStaleMs = (uint32_t)iGetTsStaleMs();
    /* 底盘参与闭环（ts_chassis_en=0 时这几个量一个都不参与, 行为与加这一段之前完全一致） */
    uint8_t  ucChasEn    = ucGetTsChassisEn();
    float    fChasPx     = fGetTsChassisPx();
    float    fChasPxToMm = fGetTsChassisPxToMm();
    float    fChasSign   = fGetTsChassisSign();
    float    fChasMmMax  = fGetTsChassisMm();
    /* 阶段化 + 踢步 + 距离辅助（ts_seq_en=0 时这些一个都不参与, 行为同加之前） */
    uint8_t  ucSeqEn     = ucGetTsSeqEn();
    float    fConvXpx    = fGetTsConvXPx();
    float    fConvYpx    = fGetTsConvYPx();
    uint32_t ulStallNeed = (uint32_t)iGetTsStallTicks();
    float    fKickDeg    = fGetTsKickDeg();
    uint32_t ulKickMax   = (uint32_t)iGetTsKickMax();
    uint8_t  ucDistEn    = ucGetTsDistEn();
    float    fDistPxToMm = fGetTsDistPxToMm();
    float    fDistMmMax  = fGetTsDistMm();

    /* ---- ③ 轴映射（语义照抄 vArmAimByPixels: swap=1 时两路**连舵机带比例尺**整套
     *         对调, 方向符号不跟着翻）---- */
    emServoDevNumTdf emDevX = ARM_SERVO6;
    emServoDevNumTdf emDevY = ARM_SERVO3;
    float fScaleX;
    float fScaleY;
    float fSignX;
    float fSignY;
    const stServoDeviceParamTdf *pstX;
    const stServoDeviceParamTdf *pstY;

    int32_t  iQr;
    uint8_t  ucWant;
    uint32_t ulElapsed   = 0u;
    uint32_t ulFrameSeen;
    TickType_t xFrameTick;
    int32_t  iConvCnt    = 0;
    float    fTravelX    = 0.0f;
    float    fTravelY    = 0.0f;
    uint8_t  ucOk        = 0u;
    uint8_t  ucAbortTravel = 0u;
    uint8_t  ucAbortNoprog = 0u;
    uint8_t  ucAbortStop   = 0u;    /* 中止位（操作员按的停, 不是故障） */
    uint8_t  ucAbortChasMm = 0u;    /* 底盘累计行程超闸 */
    /* ---- 底盘参与闭环的局部队列 ---- */
    float    fChasTravel = 0.0f;    /* 底盘累计**指令**行程 [mm]（不是实测: 打滑/架空照样累加） */
    float    fMm         = 0.0f;    /* 本拍要下发的横移量 [mm], 0 = 本拍不发 */
    uint8_t  ucChasActive = 0u;     /* 当前 X 轴归谁: 0=舵机6, 1=底盘（带迟滞+去抖） */
    uint8_t  ucChasCnt    = 0u;     /* 换挡去抖计数 */
    float    fAbsDx       = 0.0f;   /* |dx|（换挡判据用, 免得写两遍三元） */
    uint32_t ulTick      = 0u;      /* 已跑多少拍（过程串与排障用） */
    uint32_t ulNoprog    = 0u;      /* 连续"偏差不下降"的拍数 */
    float    fPrevAbsErr = -1.0f;   /* 上一拍的 |dx|+|dy|（<0 = 还没有） */
    /* ---- "等到位 + 停稳"节拍（2026-10-06 重写, 见循环顶那段说明）---- */
    uint8_t  ucLastMoved = 0u;      /* 上一拍是否发过舵机步（决定本拍要不要等到位） */
    uint32_t ulSettleMs  = 0u;      /* 停稳段 = ctrl-move 的差（进门算一次） */
    uint32_t ulWaited    = 0u;      /* 等到位实际耗时（计入 ulElapsed, 超时预算按真实耗时算） */
    uint16_t usLastR     = 0u;      /* 最后跟到的那颗靶的外圆半径[px]（TGTS fin 行用; 0=从没跟到） */
    /* ---- 阶段化 X→Y + 踢步的运行状态（ts_seq_en=1 才参与; 见函数头说明）---- */
    uint8_t  ucStage     = TS_STAGE_X;  /* 当前阶段（旧模式下不读它） */
    uint32_t ulStall     = 0u;          /* Y 连续停滞拍数（踢步触发用） */
    uint32_t ulKicks     = 0u;          /* 已用踢步次数（全程不重置, 防时间爆炸） */
    uint8_t  ucKickSt    = TS_KICKST_P; /* 踢步子状态 */
    float    fKickDir    = 0.0f;        /* 本次踢步方向（= 发踢那拍 P 律的符号） */
    float    fKickDy0    = 0.0f;        /* 踢前 dy（裁决行用） */
    float    fKickDy0Abs = 0.0f;        /* 踢前 |dy|（改善量基准） */
    float    fDyAbsPrev  = -1.0f;       /* 上一拍 |dy|（<0 = 还没有; 停滞判据用） */
    int32_t  iGraceCnt   = 0;           /* 宽容收敛连续拍数 */
    float    fDistTravel = 0.0f;        /* 距离辅助累计行程 [mm]（指令值, 同 fChasTravel 口径） */
    uint8_t  ucDistMoves = 0u;          /* 距离辅助已用次数 */
    uint8_t  ucMmFromDist = 0u;         /* 本拍 fMm 是否来自距离辅助（每拍在循环顶清）。
                                         * ⚠️ 为什么需要它: 距离辅助那拍**同时**把 ucStage 翻回 X,
                                         *    而下面 4166 的清零门控正是按 ucStage 判的 —— 不清这个标志
                                         *    就会把刚算出来的挪车量当场抹掉（症状: 距离辅助永远不动车,
                                         *    日志却照打 TGTS dist#N）, 且接管块会拿 CX 的量把它覆盖。
                                         *    有了它: 距离辅助那拍不许清 fMm、也不许接管块插手。 */
    uint8_t  ucMmLateral = 0u;          /* 本拍 fMm 的轴: 0=车体前后(CX辅助), 1=车体横移(距离辅助)
                                         * ⚠️ 每拍随 fMm 一起清, 否则会把下一次 CX 辅助的前后挪车
                                         *    误打到横移轴上 */
    float    fKickBand   = 0.0f;        /* 踢步适用带 [px] = 2×conv_px（进门算一次） */
    uint8_t  ucCnt;
    stTargetTgtTdf astT[TARGET_MAX_COUNT];
    int      iPick;
    int      i;
    /* ⚠️ 必须初始化: 参数配成 0 时（如「测试」积木拿不到参数, 见 vScriptTestLoop 的注释）
     *    循环一次都不跑, 末尾那条超时调试串要打印它们 —— 不初始化就是栈上的垃圾值,
     *    打印出来的是"看起来像个真实偏差"的数字, 反而把排障带偏。 */
    float    fDx = 0.0f;
    float    fDy = 0.0f;
    float    fStepX;
    float    fStepY;
    float    fAngX;
    float    fAngY;
    /* 四个小缓冲只给 cfg 行格式化用（dx/dy 在调试串里改用 %d 整数像素, 不需要缓冲） */
    char     acT1[12];
    char     acT2[12];
    char     acT3[12];
    char     acT4[12];
    char     acDbg[RPI_DATA_MAX + 8u];

    /* ---- ① 二维码校验（与 vTargetCalibAim 完全同款）---- */
    iQr = iGetQrCode();
    if (iQr < 0) { return 0u; }
    ucWant = (uint8_t)((iQr / 10) % 10);
    if ((ucWant < 1u) || (ucWant > 3u)) { return 0u; }

    if (ucGetTgXySwap() != 0u) { emDevX = ARM_SERVO3; emDevY = ARM_SERVO6; }
    fScaleX = (emDevX == ARM_SERVO6) ? fGetTgPxToDegX() : fGetTgPxToDegY();
    fScaleY = (emDevY == ARM_SERVO3) ? fGetTgPxToDegY() : fGetTgPxToDegX();
    fSignX  = (emDevX == ARM_SERVO6) ? fGetTgXSign()    : fGetTgYSign();
    fSignY  = (emDevY == ARM_SERVO3) ? fGetTgYSign()    : fGetTgXSign();

    /* 踢步适用带一次算好（= 旧 conv_px 的 2 倍; conv_px 是运时量快照, 循环里不再重算） */
    fKickBand = AIM_KICK_BAND_SCALE * fConvPx;

    /* ---- ④ 首次动舵机前掐掉后台臂序列（同 vArmAimByPixels）---- */
    vArmSeqAbort();

    /* ---- ④' 把**本次实际在用的控制参数**打一行到 Pi 日志 ----
     * 为什么值得单独一行: 「方向反了」和「增益/迟延导致振荡」在现象上都是"不收敛",
     * 而处方完全相反（翻 tg_ 的符号 vs 降 ts_kp_scale）。看着这一行就知道环里到底
     * 用的是哪个符号、哪个比例尺、哪个增益, 不用再去蓝牙 Tab 逐个核对。 */
    bt_fmt_f2(acT1, (int)sizeof(acT1), fSignX);
    bt_fmt_f2(acT2, (int)sizeof(acT2), fSignY);
    bt_fmt_f2(acT3, (int)sizeof(acT3), fScaleX);
    bt_fmt_f2(acT4, (int)sizeof(acT4), fKp);
    /* ⚠️ 整行必须留在 64 字节以内（帧 DATA 上限 RPI_DATA_MAX, 超了会被**从尾部**截掉,
     *    而尾巴上恰好是节拍与 kp）。所以这里用紧凑写法 s=/sc=, 不加多余空格。 */
    (void)snprintf(acDbg, sizeof(acDbg), "TGTS cfg s=%s/%s sc=%s sw=%u kp=%s c=%lu m=%lu",
                   acT1, acT2, acT3, (unsigned)ucGetTgXySwap(), acT4,
                   (unsigned long)ulCtrlMs, (unsigned long)ulMoveMs);
    vRpiSendDebug(acDbg);

    /* 底盘参与闭环时**另起一行**（上面那行已接近 RPI_DATA_MAX=64 的上限, 加不进去 ——
     * 超了会从尾部静默截断, 而尾巴上恰好是 c=/m=）。只在 en=1 时打, 保证 en=0 的日志不变。 */
    if (ucChasEn != 0u)
    {
        bt_fmt_f2(acT1, (int)sizeof(acT1), fChasPx);
        bt_fmt_f2(acT2, (int)sizeof(acT2), fChasPxToMm);
        bt_fmt_f2(acT3, (int)sizeof(acT3), fChasSign);
        bt_fmt_f2(acT4, (int)sizeof(acT4), fChasMmMax);
        (void)snprintf(acDbg, sizeof(acDbg), "TGTS chcfg px=%s sc=%s sg=%s mm=%s",
                       acT1, acT2, acT3, acT4);
        vRpiSendDebug(acDbg);

        /* 不变式: 接管带必须大于收敛死区, 否则接管带整个落在死区里, 底盘几乎没机会接管
         * （表现是"开了开关却从没见 ch on"）。只提示, 不静默改行为。 */
        if (fChasPx <= fConvPx)
        {
            vRpiSendDebug("TGTS warn: chassis_px <= conv_px");
        }
    }

    /* 阶段化/踢步/距离辅助参数也单打一行（只在 seq_en=1 时打 —— 旧模式日志保持逐字节不变;
     * 现场看这行就知道这次闭环用的是哪套阈值/踢多大的步, 不用去蓝牙 Tab 逐个核对） */
    if (ucSeqEn != 0u)
    {
        bt_fmt_f2(acT1, (int)sizeof(acT1), fConvXpx);
        bt_fmt_f2(acT2, (int)sizeof(acT2), fConvYpx);
        bt_fmt_f2(acT3, (int)sizeof(acT3), fKickDeg);
        (void)snprintf(acDbg, sizeof(acDbg), "TGTS seqcfg cx=%s cy=%s kg=%s kn=%lu st=%lu de=%u",
                       acT1, acT2, acT3, (unsigned long)ulKickMax,
                       (unsigned long)ulStallNeed, (unsigned)ucDistEn);
        vRpiSendDebug(acDbg);
    }

    /* ⚠️ 节拍/时长搭配检查: CTRL_MS 必须**大于** MOVE_MS, 差值就是"停稳段"。
     * 相等或更小时, 每拍读到的都是相机在舵机**运动中**采的帧 → 采样落点随机 →
     * 表现为"一拍过冲几倍、下一拍几乎不动"的振荡, 而不是稳定的欠冲/过冲
     * （台架实测踩过: 初始只偏 22px 却振荡到行程闸）。这条串就是给那个场景指路的。 */
    if (ulCtrlMs <= ulMoveMs)
    {
        vRpiSendDebug("TGTS warn: ctrl<=move no settle");
    }
    /* 停稳段 = ctrl-move 的差（参数语义没变, 只是 2026-10-06 起它在"舵机真正到位
     * 之后"才起算, 见循环顶）。ctrl<=move 的错配在上面警告过了, 这里钳个下限。 */
    ulSettleMs = (ulCtrlMs > ulMoveMs) ? (ulCtrlMs - ulMoveMs) : 50u;

    /* 起流前先记基线: 帧计数从这一刻开始涨才算"流真的来了"。 */
    ulFrameSeen = ulGetAimFrameCount();
    xFrameTick  = xTaskGetTickCount();

    /* 起流帧等链路（下面的循环超时与链路等待**互不相干**: 等待不算进 ulTimeoutMs）。
     * ⚠️ 这里**不要**发 AIM_STOP —— 流根本没起来; 「无论成败都发 8」那条纪律针对的是
     *    已经起过流的路径（唯一出口在函数末尾）。 */
    if (ucRpiSendTaskCmdWait(RPI_TASK_AIM_START, 0u) == 0u)
    {
        return 0u;
    }

    /* ---- ⑤ 闭环 ---- */
    while (ulElapsed < ulTimeoutMs)
    {
        ucMmFromDist = 0u;   /* 本拍量, 不许跨拍存活（见声明处注释: 距离辅助那拍靠它躲开清零） */

        /* 中止位: **必须在新鲜度判定之前**（放在循环体最前面, 连 vTaskDelay 都在它后面）。
         * 为什么位置不能更靠后: 下面那条 Stale 分支会 `continue` —— 放在它后面的话,
         * 链路一断（恰恰是最想按停的时刻）本循环就再也看不到中止位, 于是"按了停、
         * 舵机还在动、底盘还在走"。加了底盘运动之后这一条从"停不下来"升级成**安全问题**。
         * ⚠️ 工程铁律「凡是等外部事件的循环都必须带上它」—— 本循环曾经是唯一漏掉的一处。 */
        if (ucGetScriptAbortReq() != 0u) { ucAbortStop = 1u; break; }

        /* ---- 每拍的"动-停-测"等待（2026-10-06 重写, 原来是盲等 ctrl_ms）----
         * 上一拍发过舵机步 → 先等 S 曲线**真正走完**（含被速度闸削峰后的恒速收尾,
         * 用驱动的到位标志 ucServoIsSmoothStepDone; 上限兜底防舵机卡死把环挂死）,
         * 再等**停稳段**（= ctrl-move 的差, 参数语义照旧）→ 才采帧。
         * 盲等的两个病: ①被削峰的真实耗时 > move_ms, 固定停稳段被吃掉 → 采到运动帧
         * （dy 读数随机振荡, 现场把 ctrl 从 180 抬到 280 就是为它）; ②小步快收时又
         * 白等满 ctrl。改"等到位+停稳"后: 小步的拍 ~200ms 就能采（原来 380ms, 收敛
         * 速度近翻倍）, 大步也绝不会再采到运动帧 —— 速度与准确率同时受益, 且两个
         * 参数一个都不用改（ctrl=280/move=100 → 停稳段=180ms, 语义照旧）。
         * 上一拍没动舵机（死区/无帧/首拍）→ 只等停稳段。 */
        if (ucLastMoved != 0u)
        {
            ulWaited = 0u;
            while ((ulWaited < (ulMoveMs + 700u))
                   && ((ucServoIsSmoothStepDone(emDevX) == 0u)
                       || (ucServoIsSmoothStepDone(emDevY) == 0u)))
            {
                /* 中止位: 「凡是等外部事件的循环都必须带上它」—— 这里等的是舵机,
                 * 同一条铁律; 超时上限( move+700ms )兜底舵机卡死, 不会无限等。 */
                if (ucGetScriptAbortReq() != 0u) { ucAbortStop = 1u; break; }
                vTaskDelay(pdMS_TO_TICKS(5));
                ulWaited += 5u;
            }
            ulElapsed += ulWaited;
            if (ucAbortStop != 0u) { break; }
        }
        vTaskDelay(pdMS_TO_TICKS(ulSettleMs));
        ulElapsed += ulSettleMs;
        ucLastMoved = 0u;       /* 本拍动没动, 由下面下发那行重新置位 */

        /* 新鲜度: 帧计数推进 = 流在来。停涨超时 = Pi 掉线/停发 → **冻结**（不动舵机、
         * 也不判收敛）, 等它回来续跑 —— 与 app_line.c 对丢线的处理同一条思路。
         * ⚠️ 冻结不退出: 退出会把一次几秒的链路抖动变成"这次瞄准彻底失败"。 */
        if (ulGetAimFrameCount() != ulFrameSeen)
        {
            ulFrameSeen = ulGetAimFrameCount();
            xFrameTick  = xTaskGetTickCount();
        }
        else if ((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(ulStaleMs))
        {
            iConvCnt = 0;
            continue;
        }

        /* ⚠️ 不取 cam: 「标靶流控」全程只按主摄那套标定（tg_center/ts_chassis），
         *    它发的 "7" 也不带 is_use_picam → Pi 永远走主摄（见 ucRpiSendTaskCmdWaitPi）。 */
        vGetAimFrame(&ucCnt, astT, NULL);
        if (ucCnt == 0u) { iConvCnt = 0; continue; }   /* 流活着, 本帧没找到目标 */

        /* 选靶: ① 颜色 == 二维码第二位; ② 否则在**无色模式**帧(color=0xFF)里取离瞄准点
         * 最近的一颗 —— 闭环本来就在把靶往瞄准点拉, 所以最近的那颗就是正在跟的那颗。
         * ⚠️ 只认 color==0xFF 的帧走兜底; 有色帧里的**别的颜色**不参与兜底 ——
         *    那说明目标色真的丢了（被遮挡/出画）, 这时候随便挑一颗去追比不动更糟。 */
        iPick = -1;
        for (i = 0; i < (int)ucCnt; i++)
        {
            if (astT[i].ucColor == ucWant) { iPick = i; break; }
        }
        if (iPick < 0)
        {
            float fBest = 0.0f;

            for (i = 0; i < (int)ucCnt; i++)
            {
                float fDdx;
                float fDdy;
                float fD2;

                if (astT[i].ucColor != RPI_AIM_COLOR_UNKNOWN) { continue; }
                fDdx = (float)astT[i].usCx - fCenterX;
                fDdy = (float)astT[i].usCy - fCenterY;
                fD2  = fDdx * fDdx + fDdy * fDdy;
                if ((iPick < 0) || (fD2 < fBest)) { iPick = i; fBest = fD2; }
            }
        }
        if (iPick < 0) { iConvCnt = 0; continue; }     /* 没有目标色、也没有无色帧 */
        usLastR = astT[iPick].usR;

        /* EN=1 时按这颗靶的 r 插值瞄准点（距离视差补偿, 见 app_config.h「距离视差」;
         * EN=0 时函数内部=进门那份固定快照, 行为同旧）。⚠️ 无色兜底选靶（上面那段）
         * 用的仍是快照 fCenterX/Y —— iPick 还没定、拿不到 r, 且插值 center 与快照
         * 只差几个 px, 对"取最近那颗"的判定没有实质影响, 别为它提前引 r。 */
        vTargetCenterForRadius((float)astT[iPick].usR, &fCenterX, &fCenterY);
        fDx = (float)astT[iPick].usCx - fCenterX;
        fDy = (float)astT[iPick].usCy - fCenterY;

        /* 过程串（节流）: 这是整个闭环**唯一**的运行期可见性。
         * 没有它, 不收敛时只能拿到最后那一拍的偏差, 分不清是「方向反了」(偏差单调变大)
         * 还是「增益/迟延振荡」(偏差上下交替) —— 而两者处方相反。
         * ⚠️ 必须放在死区判定**之前**: 进了死区就 continue 了, 放后面会漏掉那些拍。 */
        ulTick++;
        if ((ulTick % AIM_TRACE_EVERY_TICKS) == 0u)
        {
            /* ⚠️ 必须是**两套格式串**: en=0 时多打一个 " mm=0" 就不再是逐字节等同旧行为。 */
            if (ucChasEn != 0u)
            {
                (void)snprintf(acDbg, sizeof(acDbg), "TGTS n=%lu dx=%d dy=%d trX=%d trY=%d mm=%d",
                               (unsigned long)ulTick, (int)fDx, (int)fDy,
                               (int)fTravelX, (int)fTravelY, (int)fMm);
            }
            else
            {
                (void)snprintf(acDbg, sizeof(acDbg), "TGTS n=%lu dx=%d dy=%d trX=%d trY=%d",
                               (unsigned long)ulTick, (int)fDx, (int)fDy,
                               (int)fTravelX, (int)fTravelY);
            }
            vRpiSendDebug(acDbg);
        }

        /* 「环没起作用」早退: 偏差**连续若干拍都不下降** → 提前收手, 别让机械臂
         * 一直往一个方向推（发散时它要一路推到行程闸才停, 那期间臂在乱摆）。
         * 判据用 |dx|+|dy|: 收敛时它单调下降; 发散时单调上升; **振荡时上下交替**
         * —— 所以这个闸抓的是"发散"与"停滞", **不误伤振荡**（振荡要靠降 ts_kp_scale 治,
         * 那是另一味药; 这里把它放过去, 由行程闸兜底）。
         * ⚠️ 前提是**车与靶都静止**（本步的设计前提）。车在动的话"偏差不降"是正常的,
         *    这个闸会误报 —— 但那时本步本来也不该用。
         * ⚠️ **底盘接管的那些拍不累计本闸**: 底盘的"停稳"来自 fChassisMoveDelay 的阻塞,
         *    一拍的实际推进量与 AIM_NOPROG_EPS_PX(1px) 不一定比得过（比例尺标小时更明显）,
         *    照常累计会误报"环没起作用"。底盘阶段的安全兜底交给 ts_chassis_mm 行程闸。 */
        if (ucChasActive == 0u)
        {
            float fAbsErr = ((fDx >= 0.0f) ? fDx : -fDx)
                          + ((fDy >= 0.0f) ? fDy : -fDy);

            if ((fPrevAbsErr >= 0.0f) && (fAbsErr > (fPrevAbsErr - AIM_NOPROG_EPS_PX)))
            {
                ulNoprog++;
            }
            else
            {
                ulNoprog = 0u;
            }
            fPrevAbsErr = fAbsErr;
            if (ulNoprog >= AIM_NOPROG_TICKS)
            {
                ucAbortNoprog = 1u;
                break;
            }
        }

        /* 两轴都在死区内才算"这一拍在靶上"。连续够数才判收敛 —— 单帧抖一下不算。 */
        if (ucSeqEn == 0u)
        {
            /* 旧并行环: 两轴都在旧 conv_px 内才算在靶（逐字节保持原行为） */
            if ((fDx > -fConvPx) && (fDx < fConvPx)
                && (fDy > -fConvPx) && (fDy < fConvPx))
            {
                iConvCnt++;
                if (iConvCnt >= iConvTicks) { ucOk = 1u; break; }
                continue;                                   /* 在靶上: 这一拍不推舵机 */
            }
            iConvCnt = 0;
        }
        else if (ucStage == TS_STAGE_X)
        {
            /* 阶段1只看 dx（dy 归阶段2管）: 达标 → 切阶段2。
             * ⚠️ 这里的 continue 与旧死区 continue 同语义 —— 本拍不推任何执行器,
             *    底盘接管块也在 continue 之后, 在靶期间车/舵机都歇着数拍。 */
            if ((fDx > -fConvXpx) && (fDx < fConvXpx))
            {
                iConvCnt++;
                if (iConvCnt >= iConvTicks)
                {
                    ucStage = TS_STAGE_Y;
                    iConvCnt    = 0;
                    iGraceCnt   = 0;
                    ulNoprog    = 0u;
                    fPrevAbsErr = -1.0f;
                    fDyAbsPrev  = -1.0f;
                    ulStall     = 0u;
                    ucChasActive = 0u;      /* 底盘换挡状态不带进 Y 阶段（阶段2里 X 冻结） */
                    ucChasCnt    = 0u;
                    (void)snprintf(acDbg, sizeof(acDbg), "TGTS st=y dx=%d dy=%d n=%lu",
                                   (int)fDx, (int)fDy, (unsigned long)ulElapsed);
                    vRpiSendDebug(acDbg);
                }
                continue;
            }
            iConvCnt = 0;
        }
        else
        {
            /* 阶段2: 主判据 = 更紧的 ts_conv_y_px（踢步要够到的目标）;
             * 宽容判据 = 旧 conv_px, 但只在**踢步用尽**后生效 —— 还有踢步余量就先踢,
             * 别提前躺进宽容带。 */
            if ((fDy > -fConvYpx) && (fDy < fConvYpx))
            {
                iConvCnt++;
                if (iConvCnt >= iConvTicks) { ucOk = 1u; break; }
                continue;
            }
            iConvCnt = 0;

            if ((ulKicks >= ulKickMax)
                && (fDy > -fConvPx) && (fDy < fConvPx))
            {
                iGraceCnt++;
                if (iGraceCnt >= iConvTicks)
                {
                    (void)snprintf(acDbg, sizeof(acDbg), "TGTS grace dy=%dpx n=%lu",
                                   (int)fDy, (unsigned long)ulElapsed);
                    vRpiSendDebug(acDbg);
                    ucOk = 1u;              /* 宽容收货: 与全成功靠 grace 行区分 */
                    break;
                }
                continue;                   /* 宽容带内不再推舵机, 稳稳数完拍 */
            }
            iGraceCnt = 0;
        }

        /* 偏差 → 角度增量（×tg 比例尺 ×ts_kp_scale 总增益 ×tg 方向）, 再钳单拍上限 */
        fStepX = fDx * fScaleX * fKp * fSignX;
        fStepY = fDy * fScaleY * fKp * fSignY;
        if (fStepX >  fStepMax) { fStepX =  fStepMax; }
        if (fStepX < -fStepMax) { fStepX = -fStepMax; }
        if (fStepY >  fStepMax) { fStepY =  fStepMax; }
        if (fStepY < -fStepMax) { fStepY = -fStepMax; }

        if (ucSeqEn != 0u)
        {
            if (ucStage == TS_STAGE_X)
            {
                /* 阶段1: Y 全程冻结（下发那段仍会以当前模型角重发零增量 S 曲线 = 软保持） */
                fStepY = 0.0f;
            }
            else
            {
                /* ---- 阶段2: X 冻结, Y 走"停滞→踢步→裁决"收尾 ----
                 * 机理见 AIM_STALL_* 段头。三个关键点:
                 * ① 停滞计数每拍都更（含踢步拍 —— 踢步的效果正是靠它观察）;
                 * ② 踢步幅度不受上面的单拍限幅钳制 —— 就是要越过它去突破静摩擦;
                 * ③ 停滞但无踢可踢时**冻结**（零增量）, 指令目标不再从物理位置溜走。 */
                float fDyAbs = (fDy >= 0.0f) ? fDy : -fDy;

                fStepX = 0.0f;
                /* ⚠️⚠️ 挪车量必须在本阶段**每拍清零**（2026-10-06 用户现场抓到的 bug）:
                 * fMm 的常规清零在下面底盘接管块里, 而那块**只在阶段1跑** —— X 阶段
                 * 最后一次 CX 辅助算出的 fMm 会跨过 st=y 存活下来, 而 Y 阶段每拍都
                 * 一路落到下发段 → 把那个旧量**每拍再执行一次**, 车在 Y 阶段一直滑。
                 * 清零放本分支顶部: 距离辅助在本段后面会重新 set, 当拍下发不受影响;
                 * 阶段1方向的泄漏（距离辅助的 fMm 漏进回修拍）由接管块的门控清零挡住。 */
                fMm = 0.0f;
                ucMmLateral = 0u;

                if ((fDyAbsPrev >= 0.0f) && (fDyAbs > (fDyAbsPrev - AIM_STALL_EPS_PX)))
                {
                    ulStall++;
                }
                else
                {
                    ulStall = 0u;
                }
                fDyAbsPrev = fDyAbs;

                if (ucKickSt == TS_KICKST_EVAL)
                {
                    /* ---- 裁决上一拍的踢步（此时图像已反映踢步的效果）---- */
                    float fImprove = fKickDy0Abs - fDyAbs;

                    ucKickSt = TS_KICKST_P;
                    ulKicks++;
                    ulStall = 0u;           /* 三个分支都重数, 给下一轮停滞一个干净起点 */
                    if (fImprove >= AIM_KICK_ACC_PX)
                    {
                        /* 收货: 齿面重新咬合后还真的近了 —— P 环从新位置继续 */
                        ulNoprog    = 0u;
                        fPrevAbsErr = -1.0f;
                        (void)snprintf(acDbg, sizeof(acDbg), "TGTS kick#%lu acc dy %d->%d",
                                       (unsigned long)ulKicks, (int)fKickDy0, (int)fDy);
                        vRpiSendDebug(acDbg);
                        fStepY = 0.0f;      /* 本拍先观察一拍, 下拍再回 P 律 */
                    }
                    else if (((fDy - fKickDy0) < AIM_KICK_MOVE_PX)
                             && ((fKickDy0 - fDy) < AIM_KICK_MOVE_PX))
                    {
                        /* 没动（|Δdy|<0.5px, 连 0.5° 都没踢动）: 模型溜出去一个踢步的量,
                         * 退回踢前角度把溜走量拽回来 —— 停滞冻结 + 这里 = 溜走量始终有界 */
                        (void)snprintf(acDbg, sizeof(acDbg), "TGTS kick#%lu rev dy %d->%d",
                                       (unsigned long)ulKicks, (int)fKickDy0, (int)fDy);
                        vRpiSendDebug(acDbg);
                        fStepY = -fKickDir * fKickDeg;
                    }
                    else
                    {
                        /* 动了但没变好（方向/幅度不对或过冲）: 不回退 —— 回退本身也是一次
                         * 咬合抽签, 留在原地让 P 环从新位置重进（它自己会纠） */
                        (void)snprintf(acDbg, sizeof(acDbg), "TGTS kick#%lu wst dy %d->%d",
                                       (unsigned long)ulKicks, (int)fKickDy0, (int)fDy);
                        vRpiSendDebug(acDbg);
                        fStepY = 0.0f;      /* 本拍观察 */
                    }
                }
                else if ((fDyAbs < fKickBand) && (ulStall >= ulStallNeed))
                {
                    if (ulKicks < ulKickMax)
                    {
                        /* 发踢: 方向 = 本拍 P 律的符号（上面已算好还没被覆盖）, 幅度独立于限幅 */
                        fKickDir    = (fStepY >= 0.0f) ? 1.0f : -1.0f;
                        fKickDy0    = fDy;
                        fKickDy0Abs = fDyAbs;
                        ucKickSt    = TS_KICKST_EVAL;
                        fStepY      = fKickDir * fKickDeg;
                        ulNoprog    = 0u;   /* 给这次干预留出观察拍, 别被 noprog 半路掐了 */
                        fPrevAbsErr = -1.0f;
                        bt_fmt_f2(acT1, (int)sizeof(acT1), fKickDeg);
                        (void)snprintf(acDbg, sizeof(acDbg), "TGTS kick#%lu go dy=%d k=%s",
                                       (unsigned long)(ulKicks + 1u), (int)fDy, acT1);
                        vRpiSendDebug(acDbg);
                    }
                    else if ((ucDistEn != 0u) && (ucChasEn != 0u)
                             && (fDistPxToMm != 0.0f) && (ucDistMoves < AIM_DIST_MAX_MOVES))
                    {
                        /* ---- 距离辅助（最后一档）: 踢完仍卡 → 底盘**离靶远近**微调 ----
                         * ⚠️⚠️ 轴是**车体横移**（vChassisMoveCameraLateral, v_x）, 不是前后!
                         * 本车相机**侧看**靶（图像横向 cx 对应车体纵移 —— 见下面下发段的
                         * 90° 说明）, 所以"离靶远近"（改变光点落点/靶面成像的那根轴）
                         * 物理上是车体**横移**; 车体前后只是沿靶面滑动, cy 纹丝不动
                         * （2026-10-06 首版实现就错在这里, 用户现场抓到: "到Y阶段底盘
                         * 还是在横移"）。下发段按 ucMmLateral 分流。
                         * 挪车必然扰动 dx（光点视差项 f·L/Z 随远近变）→ 立刻回阶段1重修
                         * 一圈再回阶段2, 这是设计好的小外环, 次数由 AIM_DIST_MAX_MOVES 封顶。 */
                        float fMmDist = fDy * fDistPxToMm;
                        float fRoom   = fDistMmMax - fDistTravel;

                        if (fMmDist >  AIM_DIST_MOVE_MAX_MM) { fMmDist =  AIM_DIST_MOVE_MAX_MM; }
                        if (fMmDist < -AIM_DIST_MOVE_MAX_MM) { fMmDist = -AIM_DIST_MOVE_MAX_MM; }
                        if (fMmDist >  fRoom) { fMmDist =  fRoom; }
                        if (fMmDist < -fRoom) { fMmDist = -fRoom; }
                        fStepY = 0.0f;      /* 挪车拍舵机不掺和, dy 变化才归因得清;
                                             * 挪不动（预算/幅度不足）时它同样是冻结步长 */
                        if ((fMmDist > AIM_DIST_MIN_MM) || (fMmDist < -AIM_DIST_MIN_MM))
                        {
                            fMm         = fMmDist;      /* 下面的下发/行程闸段统一处理 */
                            ucMmLateral = 1u;
                            ucMmFromDist = 1u;          /* ⚠️ 必须有: 本拍已把 ucStage 翻回 X,
                                                         * 下面那道清零门控会据此把 fMm 抹掉 ——
                                                         * 见变量声明处注释 */
                            fDistTravel += ((fMmDist >= 0.0f) ? fMmDist : -fMmDist);
                            ucDistMoves++;
                            ulStall     = 0u;
                            ulNoprog    = 0u;
                            fPrevAbsErr = -1.0f;
                            fDyAbsPrev  = -1.0f;
                            ucStage     = TS_STAGE_X;   /* 回 X 重修（挪车扰动了它） */
                            iConvCnt    = 0;
                            iGraceCnt   = 0;
                            (void)snprintf(acDbg, sizeof(acDbg), "TGTS dist#%u mm=%d dy=%d tr=%dmm",
                                           (unsigned)ucDistMoves, (int)fMmDist, (int)fDy,
                                           (int)fDistTravel);
                            vRpiSendDebug(acDbg);
                        }
                    }
                    else
                    {
                        /* 踢完仍停滞（且无距离辅助或不可用）: 冻结 —— 停滞冻结是"堵指令
                         * 溜走"的另一半: 与其每拍再推一个必然被回差吃掉的小步、让指令
                         * 目标越溜越远（突破静摩擦那拍就是大超调）, 不如停在这里等
                         * 宽容收敛 / noprog 早退。 */
                        fStepY = 0.0f;
                    }
                }
                /* else: 带外大误差或还没停滞 → fStepY 保持 P 律（与旧行为一致, noprog 兜底） */
            }
        }

        /* ---- 底盘分级接管（ts_chassis_en）----
         * 只有**画面横向那一路（图像 cx → 舵机6）**会换执行器; 画面纵向（图像 cy → 舵机3）
         * 全程归舵机。
         * ⚠️ 底盘那一路走的是**车的前后**而不是左右 —— 相机装了 90°, 见下面下发那段的说明。
         * ⚠️ 本块**必须放在单拍限幅之后**: 上面那两行 `fStepX = fDx * ...` 是无条件赋值,
         *    插在它们之前的话 `fStepX = 0` 会被立刻覆盖掉（那时症状是"开了开关舵机照动"）。
         * 迟滞 + 去抖的理由: 判据是 |dx| 与门限比大小, 单帧噪声就能让它来回翻 ——
         * 那正是"左右来回跳"这一类**执行器级 chatter**, 也就是本功能要消灭的东西本身。
         * 所以进门限用 ts_chassis_px、退门限用它的 1.5 倍, 且翻转要连续 AIM_CHAS_DEBOUNCE 拍成立。
         * ⚠️ en=0 时本块一次都不执行, 下面那段 `if (ucChasActive != 0)` 也永远不成立 ——
         *    `en=0` 逐字节等同旧行为这条就是靠这个保证的。
         * ⚠️ 阶段化模式（ts_seq_en=1）里本块**只在阶段1跑**: 阶段2 X 冻结, 底盘不再参与
         *    （距离辅助是另一条路, 在步长段里直接 set fMm, 不走本块的换挡状态机）。
         *    所以 fMm 的清零也要跟着门控 —— 阶段2里清了会把距离辅助的挪车量抹掉。
         *    ⚠️ fMm 清零时**必须连 ucMmLateral 一起清**: 距离辅助那拍把它置成了横移轴,
         *    下一拍回阶段1若 CX 辅助接管挪车, 带着旧标志会把前后挪车误打到横移轴上。
         * ⚠️⚠️ **本拍 fMm 来自距离辅助时两条都要让路**（ucMmFromDist）:
         *    距离辅助在同一拍里刚把 ucStage 翻回 X, 所以下面这条按 ucStage 判的门控会成立、
         *    把挪车量当场抹成 0（且接管块还会拿 CX 的量覆盖它）——
         *    结果就是"距离辅助全流程跑完、车一步不动, 日志却照打 TGTS dist#N"。 */
        if ((ucSeqEn == 0u) || ((ucStage == TS_STAGE_X) && (ucMmFromDist == 0u)))
        { fMm = 0.0f; ucMmLateral = 0u; }
        if ((ucChasEn != 0u) && (ucMmFromDist == 0u)
            && ((ucSeqEn == 0u) || (ucStage == TS_STAGE_X)))
        {
            uint8_t ucWantChas = ucChasActive;
            uint8_t ucSwitched = 0u;

            fAbsDx = (fDx >= 0.0f) ? fDx : -fDx;
            if (ucChasActive == 0u)
            {
                if (fAbsDx < fChasPx) { ucChasCnt++; } else { ucChasCnt = 0u; }
                if (ucChasCnt >= AIM_CHAS_DEBOUNCE) { ucWantChas = 1u; }
            }
            else
            {
                if (fAbsDx > (fChasPx * (float)AIM_CHAS_HYST_PCT / 100.0f)) { ucChasCnt++; }
                else { ucChasCnt = 0u; }
                if (ucChasCnt >= AIM_CHAS_DEBOUNCE) { ucWantChas = 0u; }
            }

            if (ucWantChas != ucChasActive)
            {
                ucChasActive = ucWantChas;
                ucChasCnt    = 0u;
                ucSwitched   = 1u;
            }

            if (ucChasActive != 0u)
            {
                /* 舵机6 让位: 本拍零增量。⚠️ 注意这不是"停止下发" —— 下面那段仍会每拍
                 * 以**当前实际角**为新起点重发一条零增量 S 曲线, 即**软保持**（不会抵抗
                 * 底盘/惯性带来的机械漂移）; 漂移由下一拍现取 fCurrentValue 一并纠回来。 */
                fStepX = 0.0f;
                /* 画面**横向**偏差 → 车体**前后**进给量 [mm]。
                 * 符号/比例尺都是本组自己的量（见 app_beam.h）, 别拿它和 ta_side_sign 混。 */
                fMm    = fDx * fChasPxToMm * fChasSign;
            }

            /* 换挡是事件而不是每拍状态, 单独打一行（过程串 3 拍才一条, 会漏掉快速换挡）,
             * 这也是"到底谁在干活"最直接的证据。 */
            if (ucSwitched != 0u)
            {
                (void)snprintf(acDbg, sizeof(acDbg), "TGTS ch %s dx=%dpx mm=%dmm tr=%dmm",
                               (ucChasActive != 0u) ? "on" : "off",
                               (int)fDx, (int)fMm, (int)fChasTravel);
                vRpiSendDebug(acDbg);
            }
        }

        /* 行程闸: 累计**本拍算出来的**步长（钳过单拍上限之后、还没下发）。
         * 闭环跑飞的症状正是"每拍都在往同一个方向推", 所以按"推了多少"算才对得上
         * （而不是按像素偏差算 —— 偏差会被比例尺与限幅缩放）。
         * ⚠️ 累加发生在判定**之前**, 所以超限那一拍本身不会被下发（先累加、再一刀切掉）——
         *    于是这个累计值比"实际走掉的"略大一点点。对安全夹来说偏保守, 是有意的。 */
        fTravelX += (fStepX >= 0.0f) ? fStepX : -fStepX;
        fTravelY += (fStepY >= 0.0f) ? fStepY : -fStepY;
        if ((fTravelX > fTravelMax) || (fTravelY > fTravelMax))
        {
            ucAbortTravel = 1u;
            break;
        }

        /* 底盘行程闸: 与上面的舵机行程闸**分开累加、分开报**。
         * 量纲不同（mm vs °）, 混在一起报会把"车走了多少 mm"读成"舵机走了多少度"——
         * 本函数末尾那条单位教训（travel x=-95.00）就是这么踩出来的。
         * ⚠️ 累加的是**指令值**不是实测: 轮子打滑/被架空时它照样涨（app_motor.c 把 EMM 的
         *    回调传成 NULL, 应用层没有任何应答解析）→ 它只能当安全夹, 不能当里程计。
         * ⚠️ 与上面同款: 先累加、后判定, 所以超限那一拍本身不会被下发（偏保守）。 */
        fChasTravel += (fMm >= 0.0f) ? fMm : -fMm;
        if ((ucChasEn != 0u) && (fChasTravel > fChasMmMax))
        {
            ucAbortChasMm = 1u;
            break;
        }

        /* 每拍**现取**当前角（刻度基准, 不能被缓存）。目标角按静态参数钳位 ——
         * 宁可少走一点也不越量程（同 vArmAimByPixels 的语义）。 */
        pstX = c_pstGetServoDeviceParam(emDevX);
        pstY = c_pstGetServoDeviceParam(emDevY);
        fAngX = pstX->stRunningParam.fCurrentValue + fStepX;
        fAngY = pstY->stRunningParam.fCurrentValue + fStepY;
        if (fAngX < pstX->stStaticParam.fValueMin) { fAngX = pstX->stStaticParam.fValueMin; }
        if (fAngX > pstX->stStaticParam.fValueMax) { fAngX = pstX->stStaticParam.fValueMax; }
        if (fAngY < pstY->stStaticParam.fValueMin) { fAngY = pstY->stStaticParam.fValueMin; }
        if (fAngY > pstY->stStaticParam.fValueMax) { fAngY = pstY->stStaticParam.fValueMax; }

        /* ⚠️ 下发后**不等到位**: 节拍本身就是推进器（下一拍的"等到位+停稳"就是它的
         *    行程时间, 2026-10-06 起等的是**真实到位**而不是固定 ctrl —— 见循环顶）。
         *    ucLastMoved 置位后, 下一拍会先等这两轴的曲线真正走完再采帧。 */
        vServoSetTargetValueTimed(emDevX, fAngX, ulMoveMs);
        vServoSetTargetValueTimed(emDevY, fAngY, ulMoveMs);
        ucLastMoved = 1u;

        /* ---- 底盘进给（位置模式, 阻塞到位）----
         * ⚠️⚠️ **走的是"前后"（v_y / AXIS_Y）, 不是"左右"**: 本车相机相对车体装了 90°
         *    （证据: BALL_XY_SWAP / HOSTAGE_XY_SWAP / TARGET_ALIGN_XY_SWAP 三个默认都是 1,
         *    注释写明"相机相对车体装了 90°时的情形"）, 于是**图像横向 cx 对应的是车的纵移**。
         *    色球对位那段代码是同一件事的现成例子（app_rpi.c 的 ball_xy_swap=1 分支）:
         *        fDxMm(横移) ← usCy ; fDyMm(纵移) ← usCx
         *    所以这里必须用 Forward 那条对偶。用 Lateral 会去改图像的 cy（轴用错, 而现象是
         *    "在动, 但偏差不降" —— 会被误诊成符号反了）。⚠️ 别再"顺手"改回 Lateral。
         * ⚠️ **`fMm == 0` 时必须跳过**: 下层 `vChassisMoveDistanceY` 在距离为 0 时静默
         *    直接返回（一条 EMM 指令都不发）, 但 `fChassisMoveDelay` 会**照样睡 200ms**
         *    （app_chassis.c 里 `ABS(distance) <= 2.0f → time = 200.0f`）——
         *    舵机阶段与死区里每拍白发 200ms 的呆等。
         * ⚠️ 位置模式**自终止**（脉冲走完电机就停）, 所以本步在**任何出口都不发停车指令** ——
         *    这既避免了速度模式那种"忘了发 0 就带着末速一直爬", 也顺带避开了
         *    app_chassis.c 那条「速度模式指令会主动中止在飞的位置移动」。
         * ⚠️ 速度/加速度复用「标靶对位」的 ta_align_*（同一种物理操作, 只影响行走耗时、
         *    **不影响终点**）。 */
        if (fMm != 0.0f)
        {
            /* ⚠️ 轴按 ucMmLateral 分流: CX 辅助 = 车体**前后**（上面 90° 说明）;
             * 距离辅助 = 车体**横移**（= 离靶远近 —— 相机侧看靶, 这根轴才改变光点落点/
             * 靶面成像, 见步长段距离辅助那条注释; 上面"别顺手改回 Lateral"的警告
             * 说的只是 CX 那一路, 别照搬到距离辅助） */
            if (ucMmLateral != 0u)
            {
                vChassisMoveCameraLateral(fMm, (int)iGetTaAlignAccel(), (int)iGetTaAlignSpeed());
            }
            else
            {
                vChassisMoveCameraForward(fMm, (int)iGetTaAlignAccel(), (int)iGetTaAlignSpeed());
            }
        }
    }

    /* ---- ⑥ 停流: 唯一出口, 收敛/超时/行程超限都经这里 ----
     * ⚠️ 少了这一步 Pi 会一直占着相机发流（它还有 TARGET_STREAM_MAX_S 兜底, 但那是
     *    最后一道保险, 不该当正常路径用）。 */
    vRpiSendTaskCmd(RPI_TASK_AIM_STOP);

    /* ---- ⑥' 收官数据行（2026-10 加, 「标靶练习」数据采集的数值记录）----
     * 六种出口（ok/noprog/travel/chasmm/stop/timeout）都经这里: 一行收齐本次闭环的
     * 舵机累计行程（trX/trY, 度）、底盘累计指令行程（ch, mm）、最后跟到那颗靶的半径
     * （r, px; 0 = 从没跟到过目标）。ok/abort 行刻意不带这些量 —— 都收在这里,
     * 解析端只认一行, 不用去拼"最后一条 n= 过程串"（结束时底盘若还 engaged,
     * 过程串里根本没有底盘累计总数）。 */
    (void)snprintf(acDbg, sizeof(acDbg), "TGTS fin: trX=%d trY=%d ch=%dmm r=%upx",
                   (int)fTravelX, (int)fTravelY, (int)fChasTravel, (unsigned)usLastR);
    vRpiSendDebug(acDbg);

    /* 阶段化/踢步/距离辅助的收官数据（只在 seq_en=1 时打, 旧模式日志逐字节不变）:
     * st = 出口时所在的阶段（x=还没进阶段2就结束 / y=阶段2里结束 —— 距离辅助回修那圈
     * 也算 x）; ky = 用掉的踢步数; dm/dt = 距离辅助次数与累计 mm。分析端拿它统计
     * "踢步有没有把 Y 压下去"。 */
    if (ucSeqEn != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "TGTS fin2: st=%c ky=%lu dm=%u dt=%dmm",
                       (ucStage == TS_STAGE_Y) ? 'y' : 'x',
                       (unsigned long)ulKicks, (unsigned)ucDistMoves, (int)fDistTravel);
        vRpiSendDebug(acDbg);
    }

    /* ---- ⑦ 调试串（现场排障第一手; 本步不蜂鸣）----
     * ⚠️⚠️ **单位必须写在标签里**: 早先版本这里打的是 "travel x=-95.00 y=-14.00",
     *     而 x=/y= 其实是**像素偏差**不是角度 —— 现场被读成"X 轴走了 95°"（上限才 25°）,
     *     整个排障方向就跑偏了（真踩过）。**凡是裸数字串, 一律把单位写进去**:
     *     dx/dy 是 px, trX/trY 是累计行程的度。 */
    if (ucOk != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "TGTS ok: dx=%dpx dy=%dpx n=%lums",
                       (int)fDx, (int)fDy, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 1u;
    }
    if (ucAbortNoprog != 0u)
    {
        /* 偏差连续若干拍不下降: 环没起作用。**先查方向**（tg_x_sign/tg_y_sign 对不对）,
         * 再看舵机是不是根本没动（驱动/量程钳位）。行程闸在这之前就拦住了, 所以
         * 打出来的 trX/trY 还远没到上限 —— 这正是它比 travel 那条更有用的地方。 */
        (void)snprintf(acDbg, sizeof(acDbg),
                       "TGTS fail: noprog dx=%dpx dy=%dpx trX=%d trY=%d n=%lums",
                       (int)fDx, (int)fDy, (int)fTravelX, (int)fTravelY,
                       (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortTravel != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg),
                       "TGTS fail: travel dx=%dpx dy=%dpx trX=%d trY=%d trmax=%d n=%lums",
                       (int)fDx, (int)fDy, (int)fTravelX, (int)fTravelY,
                       (int)fTravelMax, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortChasMm != 0u)
    {
        /* 底盘累计行程超闸。**单独一条串、单位写清 tr=mm** —— 复用上面 travel 那条会把
         * "车走了多少 mm"读成"舵机走了多少度"（本函数末尾那条单位教训就是这么来的）。
         * 排障含义: 方向标反了（ts_chassis_sign）、或比例尺离谱（ts_chassis_px_to_mm）、
         * 或轮子被架空/打滑（本闸按指令值累加, 分不出这三种）。 */
        (void)snprintf(acDbg, sizeof(acDbg),
                       "TGTS fail: chasmm dx=%dpx tr=%dmm trmax=%dmm n=%lums",
                       (int)fDx, (int)fChasTravel, (int)fChasMmMax,
                       (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortStop != 0u)
    {
        /* 操作员按的停: **不蜂鸣** —— 按停是意图, 不是故障（同 ucRpiLineConverge 那条）。
         * ⚠️ 这条必须排在下面 timeout 兜底**之前**, 否则会掉进 "TGTS fail: timeout",
         *    给排障一个错误归因（"超时"指向加时间/降增益, 实际是人按的停）。 */
        (void)snprintf(acDbg, sizeof(acDbg),
                       "TGTS abort: stop by request n=%lums", (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    (void)snprintf(acDbg, sizeof(acDbg),
                   "TGTS fail: timeout dx=%dpx dy=%dpx trX=%d trY=%d n=%lums",
                   (int)fDx, (int)fDy, (int)fTravelX, (int)fTravelY,
                   (unsigned long)ulElapsed);
    vRpiSendDebug(acDbg);
    return 0u;
}

/* ==================== 平移射靶（激光固定在底盘, 2026-10-07 加, 与上面两套打靶并存） ==================== */

/* ---- 选靶统一门（2026-10-08 加）: 半径合理 + 同色里取"离瞄准列最近" ----
 * 为什么要: 粗调与精调原来都是"**取帧里第一个同色的**"。目标一切到蓝色, 背景那块
 * **大面积蓝柜子与靶同色**; 流帧的跟踪窗丢失目标时会膨胀, 一旦把柜子框进来、而它在数组里
 * 排在真靶前面, 环就直接锁柜子往前冲 —— 2026-10-08 那两发"抽搐跑飞"（`fail: travel`,
 * 且记录的 tot 与实测 d 推不出同一方向）最可疑的一环。实测正常靶一直是 r=89~111px。
 * ⚠️ 与 AIM_* 那组一样是**编译期常量**（改完要重编译）。 */
#define AIM_R_TARGET_MIN_PX   70.0f     /* 靶盘外圆等效半径下限 [px] */
#define AIM_R_TARGET_MAX_PX   130.0f    /* 上限（背景同色物/地板反光会远大于此） */

/* pi(CSI)摄像头那一套半径门（2026-10-16 加）。
 * 为什么必须**各一套**: r 是"这个相机看到的外圆半径", 随**焦距/安装距离/分辨率**变 ——
 *   CSI 与主摄是两颗完全不同的镜头, 拿主摄那把尺去卡 CSI 的靶可能把**全部候选都剔掉**,
 *   现象是 `SHOT fail: color N bad radius` / `warn: color N r=..px rejected`,
 *   看着像背景同色物抢靶（2026-10-08 那个老病）, 极易误诊。
 * ⚠️ 按上面 AIM_* 那组的既有纪律, 这里是**编译期常量、刻意不做成蓝牙运时量**:
 *    标定手法 = 用 CSI 跑一步, 读 `SHOT meas: ... r=..` 或 Pi 日志里的 r, 照实测重编。
 * 初值 = 与主摄那对**相同**（"先别拦, 看到真值再收"）。
 * ⚠️ 交付前那个 90° 旋转（Pi 侧 config.PICAM_ROTATE）**不影响这两条**: r 是与方向
 *    无关的量, 转不转它一样 —— 要动的只有**焦距/安装距离/取流分辨率**。
 * ⚠️ 2026-10-16 按实测收窄成 150~330: Pi 侧取流定稿为 **2592×1944（传感器原生满分辨率）**
 *    后 px/mm = 传感器原生 **4.77** ⇒ 台架实测靶盘 **r ≈ 238px**
 *    （1920×1080 时代是 176、1600×1200 时代是 147 —— **改分辨率 = 换尺度, 必须重测**）。
 *    原来的 70/130 会把真靶当"半径不合理"剔掉, 满分辨率下 238 更是直接超上限 ✗;
 *    上界 330 仍挡得住同色大块（背景蓝柜子在满分辨率下 r≈888）✓。
 *    **换机器/换距离/换分辨率后必须重测**（标定手法见 app_config.h「平移射靶」段头）。 */
#define AIM_R_TARGET_PI_MIN_PX  150.0f
#define AIM_R_TARGET_PI_MAX_PX  330.0f

/* ---- 起流后「等车真的停稳」再进精调环（2026-10-08 加）----
 * 为什么必须: 粗调的"到位"是 fChassisMoveDelay 的**纯时间估算**（无编码器反馈）。
 * 估时偏短时车还在跑, 而精调环读到的 d 是"运动中的靶" → 朝**错误方向**出力:
 *   2026-10-08 实测 —— 命令 169mm / 估 1044ms, 1.5s 后车还在动（流帧 #6 仍见靶在移）;
 *   精调前三拍因此又往回多退了 46mm, 把粗调的超调放大一截（现象 = "大动作之后莫名
 *   往回走、还没收敛就停住"）; 位移更大那发（183mm）直接被推着发散到行程闸。
 * 判据: 连续 AIM_SETTLE_FRAMES 个**新帧**里选中靶的 cx 变化都 < AIM_SETTLE_PX。
 * ⚠️ 上限 AIM_SETTLE_TIMEOUT_MS 到了就**照常往下走**（不判失败）—— 识别/链路异常时
 *    "一直不稳"是常态, 卡死比带着残余误差开工更糟（环自己会收敛）。 */
#define AIM_SETTLE_POLL_MS     30u     /* 等停稳的采样间隔（流 ~25fps, 够密） */
#define AIM_SETTLE_FRAMES      3u      /* 连续几个新帧"没动"算停稳 */
#define AIM_SETTLE_PX          3.0f    /* 相邻新帧选中靶 cx 变化小于此值算"没动" [px] */
#define AIM_SETTLE_TIMEOUT_MS  800u    /* 停稳窗上限 [ms]（从**第一帧**算起） */

/* ---- 平移射靶「保持」档（ucHold=1, OLED 射靶测试页用; 2026-10-09 加）----
 * 保持环永不因超时/收敛/闸门退出, 只有中止位能停 —— 所以下面这几个是它的节流与自愈旋钮。 */
#define AIM_HOLD_COARSE_MS     10000u  /* 粗调识别 + 等停稳这两段有界前戏的时限（保持环本身不限时） */
#define AIM_HOLD_TRACE_TICKS   11u     /* 带内（|d|<收敛阈值）时过程串的心跳间隔: ~2s（180ms/拍） */
#define AIM_HOLD_REVIVE_MS     5000u   /* 帧停涨时重发 "7" 的最小间隔（Pi 侧 TARGET_STREAM_MAX_S=12s
                                        * 会让流自己退, 而本环不判超时 → 没有这条自愈就是冻住永不自愈） */
#define AIM_STATE_IDLE         0u
#define AIM_STATE_COARSE       1u
#define AIM_STATE_MOVING       2u
#define AIM_STATE_SETTLED      3u
#define AIM_STATE_FROZEN       4u   /* 撞闸/noprog → 停车冻结, 等操作员再按一格 */
/* 下面两个是"车为什么不动"的另外两种答案（保持档专用; 一次性档也有意义, 只是看不到）。
 * ⚠️ OLED 的标签把它们各画一个态 —— 少了它们, 丢靶/掉流时屏上会停在最后一个
 *    SETTLED 的 `OK!` 或某个陈旧 d 值上, 看着像"已经对准了、只是车没动"。 */
#define AIM_STATE_NOTGT        5u   /* 流活着, 但本帧没有目标色（丢靶/被半径门全剔除） */
#define AIM_STATE_STALE        6u   /* 帧计数停涨 = Pi 没在发（掉线/流线程退出） */

/* 靶盘半径是否合理。只判半径, 不判颜色/位置。
 * ucIsPi: 用哪台相机的门（两套常数, 见上面 AIM_R_TARGET_PI_* 的理由）。 */
static uint8_t ucAimRadiusOk(uint16_t usR, uint8_t ucIsPi)
{
    float fR   = (float)usR;
    float fMin = (ucIsPi != 0u) ? AIM_R_TARGET_PI_MIN_PX : AIM_R_TARGET_MIN_PX;
    float fMax = (ucIsPi != 0u) ? AIM_R_TARGET_PI_MAX_PX : AIM_R_TARGET_MAX_PX;

    return ((fR >= fMin) && (fR <= fMax)) ? 1u : 0u;
}

/*
 * 在「颜色 = ucColor 且半径合格」的候选里取**离瞄准列最近**的一颗（单轴 |偏差| 最小）。
 * 返回 -1 = 没有合格候选。
 * ⚠️ 别退回"取第一个同色的": 同色重复对象（背景柜子、地板反光）会抢走真靶。
 * ⚠️ 调用方传 ucCnt 必须是**真实条数**; 粗调那边按 TARGET_MAX_COUNT 全扫, 所以它必须
 *    先清零 astT（vGetTargetResult 只填 ucCnt 颗, 剩下的槽是栈垃圾, 其 ucColor 可能
 *    碰巧落在 1~3 = ucWant）。清零后垃圾槽 ucColor=0 且 usR=0, 两道门都过不去。
 */
static int iAimPickNearest(stTargetTgtTdf *pastT, uint8_t ucCnt, uint8_t ucColor,
                           uint8_t ucSwap, float fAimCol, float fAimRow, uint8_t ucIsPi)
{
    int     iPick = -1;
    float   fBest = 0.0f;
    uint8_t ucI;

    for (ucI = 0u; ucI < ucCnt; ucI++)
    {
        float fD;
        float fAbs;

        if (pastT[ucI].ucColor != ucColor)             { continue; }
        if (ucAimRadiusOk(pastT[ucI].usR, ucIsPi) == 0u) { continue; }

        fD   = (ucSwap != 0u) ? ((float)pastT[ucI].usCx - fAimCol)
                              : ((float)pastT[ucI].usCy - fAimRow);
        fAbs = (fD >= 0.0f) ? fD : -fD;
        if ((iPick < 0) || (fAbs < fBest)) { iPick = (int)ucI; fBest = fAbs; }
    }
    return iPick;
}

/*
 * 平移射靶一步到位（脚本动作 STEP_TARGET_SHOOT_WAIT 的原语）:
 *   前提 = 激光**固定在底盘上**, 俯仰/偏航在测试阶段手动定死（高度一次给定）——
 *   于是"瞄准"只剩一个自由度: 沿靶排**平移车体**。本步全程**不碰机械臂**（不进 c_bIsArmStep）,
 *   相机位姿由路线在前面排好的摆臂步骤给足。
 *
 *   ① 读二维码 → 没认出来 / 第二位不在 1~3 就整段不做（拒绝, 一步不动车, 留痕）;
 *   ② 粗调（现成的 "6"/0x04 全画面识别, **目标色在即可**、不要求三颗齐）: 按二维码第二位选色 →
 *      沿排偏差**按格换算**（偏差 ÷ tc_gap_px × tc_gap_mm）→ 一次纵移到位
 *      （钳 ±tc_max_mm, 死区 tc_db_mm 外才动; ~±150mm 标靶中心距在这段走掉 ——
 *      **"走多大"的总旋钮 = tc_gap_mm**, 改 160 只影响粗调, 精调环的 tc_px_to_mm 不受影响）。
 *      ⚠️⚠️ **粗调固定用主摄**（2026-10-18 定案）: 格距/瞄准列这套尺是主摄像素标定的,
 *      pi 的 px/mm 约两倍、视场又窄 —— 详见下面发 "6" 处那段 ⚠️⚠️;
 *   ③ 精调（现成的 "7"/0x05 流帧）: 发 "7" 起流（**tc_use_picam=1 时优先 pi**: 它固定在
 *      底盘、几何稳、px/mm 更细; 起不来/中途掉帧由 Pi 侧**整条改用主摄**重跑, 每帧
 *      status bit7 告诉本步换哪把瞄准尺）→ 每拍按沿排偏差小步纵移
 *      （**微调模式 tc_vel_mode 二选一**: 0=位置模式, 每拍走 tc_step_mm 内一步、阻塞到位
 *      = 天然停稳段, 步进量化细但有顿挫; 1=速度模式, 每拍按偏差比例发 rpm 连续修,
 *      平滑无顿挫 —— 指令持久, 死区/所有出口显式发 0 停车, 1rpm 量化 ≈ 5.08mm/s、
 *      死区外的"零速带"由顶格逻辑兜住（发最小 1rpm 顶进收敛带））
 *      → 连续 tc_conv_ticks 拍进 tc_conv_px 判收敛;
 *   ④ 无论成败发 "8" 停流（唯一出口, 同 ucRpiTargetServo 的纪律）;
 *   ⑤ keep_res=1 时把本步**累计纵移**折世界系记进「路径残留偏移」—— 抓左/右靶那 ~±150mm
 *      （标靶中心距）就是"离开靶区时要多走/少走"的量, 后面的世界系平移勾
 *      "叠加上次残留偏移"一次性补掉（同 ucRpiHostageGrabOne 的记账口径）。
 *
 * ⚠️⚠️ **轴是"纵移/前后"（v_y, vChassisMoveCameraForward）**: 本车相机装 90°,
 *    靶排**沿车体纵向**排开（同球排/人质排 —— 见 ONE_GRAB 的 one_lat_dir 预期 3/4 那条）,
 *    车体前后 = 沿靶面滑动, 靶排在画面里沿图像横向 cx 排开 → **沿排对齐 = cx 偏差驱动纵移**
 *    （swap=1 时; swap=0 的相机装法则改由 cy 驱动）。这与「标靶流控」底盘接管是同一根轴
 *    （它也是 vChassisMoveCameraForward 修 cx）。
 *    ❌ **别用 vChassisMoveCameraLateral（cy 驱动那路）**: 那是「标靶对位」修**离靶远近**
 *    的轴（横移改变距离、经视差动 cy）, 拿来沿排对齐 = 轴用错, 现象是"车在动、cx 偏差不降"
 *    （2026-10-07 首版实现就错在这里, 用户现场抓到"运动的轴弄反了"）。
 * ⚠️ **"走多大"与"收多快"是两套参数（2026-10-07 用户要求拆开）**:
 *    粗调按**格**换算 —— 位移 = 视觉偏差 ÷ tc_gap_px × tc_gap_mm, 用户要的"150"
 *    就是 tc_gap_mm（想走 160 直接改它, 只影响粗调）; tc_gap_px = 一格靶距在画面里
 *    的像素宽（流帧相邻靶 cx 差, 实测 ~274）。精调环每拍步长 = 偏差px × tc_px_to_mm
 *    （只影响收敛快慢, 不影响终点 —— 粗调走完大位移后只剩几个 px）。
 *    方向 tc_sign 两段共用（初值 = ts_chassis_sign 现场标定值; 刻意不共用 ts_chassis_* ——
 *    为调本步翻符号会连带改坏「标靶流控」的底盘接管）。
 * ⚠️ 瞄准常数（tc_cx_aim / tc_cy_aim, 由 ta_xy_swap 选其一）与 tc_gap_px 默认 0 =
 *    未标定 → 拒绝动车（同看路 road_dx_ideal 的哨兵约定, 宁可不动也不照占位值跑; 不响, 留痕）。
 *    标定: 车摆在激光正打中间靶中心的位姿 → 发"6" → 读中间靶 cx 填 tc_cx_aim（swap=1 本车）;
 *    gap_px = 相邻两颗靶的 cx 差。
 * ⚠️ 精调环对**步进位置模式**闭环（量化 1 脉冲 ≈ 0.095mm ≈ 0.28px, 比舵机细一个量级）,
 *    所以收敛阈值可以比舵机那套紧; 帧新鲜度复用 ts_stale_ms, 停涨 = 冻结不放弃（同 ⑤）。
 * ⚠️ noprog 闸判 |dpx| 连续不下降 —— 位置模式推进量稳定, 1px 判据成立（舵机环那条
 *    "底盘阶段不累计"的顾虑在这里不存在: 本环的"停稳"每拍都由阻塞平移提供）。
 * ⚠️ 循环体最前面查中止位（铁律）; 平移是位置模式、自终止, 任何出口不发停车指令。
 * ⚠️ ⚠️ 激光射击**不在本步** —— 对准即返回, 开火由路线紧后排 STEP_LASER_SHOOT。
 * ⚠️ I(0) 分别管粗调识别与精调环**各一段**: 总耗时上界 ≈ 识别(≤I(0)) + 粗调平移 +
 *    起流(~1s 全画面锁定) + 精调(≤I(0)), 给 10000ms 足够。
 * 返回 1 = 已对准（或粗调后已在死区内且精调立刻收敛）; 0 = 数据不全/未标定/超时/行程
 * 超限/中止。仅任务上下文调用（内部 vTaskDelay + 阻塞平移）。
 *
 * ⚠️⚠️ **保持档（ucHold=1, OLED「射靶测试」页专用, 2026-10-09 加）**：
 *    与上面**同一套测量、同一套控制律**，只改**退出语义**。用途 = 现场标定 `tc_cx_aim`
 *    （"瞄准列"）—— 日志**永远看不出打没打中**（每发都收敛到瞄准列, `SHOT ok: d=..px`
 *    是恒真的），只有「按一格 → 车跟到新目标 → 开一炮看光点偏哪」这个闭环能标。
 *    · **永不因超时/收敛/noprog/行程闸退出**: 只有中止位（OLED 按「停止」→ `vScriptAbort`）能停;
 *    · 每拍**重读**瞄准常数与收敛阈值 → 操作员按一格（±5px）, 偏差出死区, 车就跟着走;
 *      瞄准常数一变即**重置**行程闸/noprog/收敛计数（"每次微调一套新预算"）并打 `SHOT hold: aim->`;
 *    · 撞闸/noprog **不退出**: 发 0 停车 + 冻结 + 一条 `SHOT hold: ..frozen`, 等下次微调自恢复;
 *    · 起流自愈: 帧停涨超 2×ts_stale_ms 限速重发 "7"（Pi 侧 `TARGET_STREAM_MAX_S`=12s 会让流
 *      自己退, 而本环不判超时 —— 没有这条就是"冻住永不自愈"）;
 *    · 过程串节流: 死区外每拍、带内 ~2s 一条（"永不结束"必须改节流, 否则分钟级日志被撑爆）;
 *    · `keep_res` **恒 0**（保持测试绝不记残留 —— 挂着会被后面任何勾"叠加残留"的平移误吃）;
 *    · 遥测发布 `shoot_err`/`shoot_state`/`shoot_settle_tick`（OLED 标签 + 蓝牙 Tab 读它）;
 *    · **进门自动预热 pi**（2026-10-18 加, 见 ucRpiTargetShootHold）: 这一档没有路线能排
 *      预热积木, 不预热的话"这次按跑 pi、下次按跑主摄"随 CSI 冷启动的运气摇摆（瞄准列也
 *      跟着换台）—— 标定时最怕这个。
 *    粗调 + 等停稳那两段**照跑一遍**（有界, 用 `AIM_HOLD_COARSE_MS`）—— 让"随便摆个位姿按开始"
 *    也能先自动 snap 到当前瞄准列, 之后才靠调参数微调。
 */
/* ---- 两个公开壳 + 唯一实现 ----------------------------------------------------
 * `ucTargetShootCore` 是**唯一实现**（理由同 ucQrScanCommon: 两条路只差"要不要停",
 * 另抄一份迟早漂）。ucHold=0 = 一次性对准（老行为逐字节不变）; ucHold=1 = OLED
 * 射靶测试页的「保持」档, 语义见 app_rpi.h::ucRpiTargetShootHold 的函数头说明。
 * ⚠️ 出口统一清 shoot_state（保持环内部每拍自己写 COARSE/MOVING/SETTLED/FROZEN）。 */
static uint8_t ucTargetShootCore(uint32_t ulTimeoutMs, uint8_t ucKeepRes, uint8_t ucHold,
                                 uint8_t ucRev);

uint8_t ucRpiTargetShootAlign(uint32_t ulTimeoutMs, uint8_t ucKeepRes, uint8_t ucRev)
{
    uint8_t ucRet = ucTargetShootCore(ulTimeoutMs, ucKeepRes, 0u, ucRev);

    vSetShootState(AIM_STATE_IDLE);
    return ucRet;
}

uint8_t ucRpiTargetShootHold(void)
{
    uint8_t ucRet;

    /* ⚠️ 进门先把 pi 预热上（2026-10-18 加）: 保持档是**人在台架上反复按**的（OLED 射靶测试页）,
     *    前面没有路线能排预热积木, 而细调那股 "7" 只等 `PICAM_STREAM_UP_S`(3s) 起流 ——
     *    CSI 冷启动正好卡在这个边界上, 于是"这次按跑在 pi、下次按跑在主摄"（瞄准列也跟着
     *    换台）, 标定时极难判读。粗调(固定主摄)+等停稳本身要 ~2~3s, 正好交给它起流 ⇒
     *    **每次按都稳定地落在一台相机上**（pi 被占着/起不来 → 稳定地退回主摄 = 老方案）。
     *    ⚠️ 只发不等（同 STEP_RPI_WARM_CAM 那条帧）, 不阻塞脚本任务。
     *    ⚠️ 判据与 ②'' 那道门同源: tc_use_picam=0 或 pi 瞄准列没标 → 不预热（用不到,
     *       别白占着 CSI —— 它还常被网页预览要）。 */
    if ((ucGetTcUsePicam() != 0u)
        && (((ucGetTaXySwap() != 0u) ? fGetTcCxAimPi() : fGetTcCyAimPi()) > 0.0f))
    {
        vRpiWarmCamera(WARM_CAM_PI, 2u, 0u);
    }

    ucRet = ucTargetShootCore(AIM_HOLD_COARSE_MS, 0u, 1u, 0u);   /* 保持档恒正向（无 rev 概念, keep_res 恒 0 同理） */

    vSetShootState(AIM_STATE_IDLE);
    return ucRet;
}

static uint8_t ucTargetShootCore(uint32_t ulTimeoutMs, uint8_t ucKeepRes, uint8_t ucHold,
                                 uint8_t ucRev)
{
    /* ---- 进门取一次标定量快照（同 ucRpiTargetServo 的理由: 一次调用一条控制律）
     * ⚠️ 保持档（ucHold=1）在循环体顶部**每拍重读**瞄准常数与收敛阈值 —— 下面这份
     *    快照只是"粗调 + 等停稳"那两段前戏用的基线。 ----
     * ⚠️ ucRev「反向执行」（2026-10-30 加, 积木 int 尾槽勾选）: 靶排换到路的另一边、
     *    相机朝另一边看 —— 偏差→车体纵移的映射镜像, 沿排方向**本步内**整体取反
     *    （tc_sign 蓝牙标定值不动）。粗调按格换算 / 出画面补步 / 精调两模式全部跟着翻。
     *    **残留自动跟着对称**: fTotalMm 是带符号的实际累计纵移, 反向后同一侧靶记出的
     *    残留镜像 —— 消费端（世界系平移勾"叠加上次残留偏移"= 计划−残留）一字不改,
     *    反向前"打左靶离开时要多走"、反向后变成少走（2026-10-30 场地换边的验收判据）。
     *    ⚠️ 瞄准列/格距像素（tc_cx_aim/tc_gap_px, 必要时 pi 那对）随镜像重标,
     *    用「标定」Tab/蓝牙现值, 与本开关无关。
     *    ⚠️ **CSI 不参与反向**（2026-10-30 加）: pi 摄像头固定在底盘、不随臂转向
     *    另一边的靶排 —— rev=1 时细调强制只用主摄（ucUsePicam 门内强制 0,
     *    "7" 不带 pi 标志 = 与老方案逐字节同形）、粗调失败也不再拿 CSI 重试;
     *    tc_use_picam 蓝牙值不动。反向档"只能全程机械臂(主摄)瞄准"即定案;
     *    日后把 CSI 物理挪装到新一侧, 删掉那处强制即可恢复"细调优先 pi"。 */
    float    fAimCol     = fGetTcCxAim();       /* ta_xy_swap=1(本车)时的瞄准常数（沿排那根轴） */
    float    fAimRow     = fGetTcCyAim();       /* ta_xy_swap=0 时的瞄准常数 */
    uint8_t  ucSwap      = ucGetTaXySwap();     /* 轴映射继承「标靶对位」（相机装法是车身属性） */
    float    fGapMm      = fGetTcGapMm();       /* 粗调格距[mm]: 打边靶走多远的总旋钮（用户要的"150"） */
    float    fGapPx      = fGetTcGapPx();       /* 每格像素[px]: 一格靶距在画面里的宽（流帧相邻靶 cx 差） */
    float    fPxToMm     = fGetTcPxToMm();      /* 精调专用比例: 只管每拍步长（收敛快慢）, 粗调不用它 */
    float    fFwdSign    = fGetTcSign() * ((ucRev != 0u) ? -1.0f : 1.0f);
                                                /* 沿排方向（初值=ts_chassis_sign 的现场标定值;
                                                 * rev=1 本步内取反 —— cfg2 的 sg= 回显的就是这个有效值 */
    float    fDbMm       = fGetTcDbMm();
    float    fMaxMm      = fGetTcMaxMm();
    float    fConvPx     = fGetTcConvPx();
    int32_t  iConvTicks  = iGetTcConvTicks();
    float    fKp         = fGetTcKpScale();
    float    fStepMax    = fGetTcStepMm();
    float    fTravelMax  = fGetTcTravelMm();
    uint32_t ulCtrlMs    = (uint32_t)iGetTcCtrlMs();
    uint32_t ulStaleMs   = (uint32_t)iGetTsStaleMs();   /* 帧新鲜度继承 ts_*（同一条流） */
    uint8_t  ucVelMode   = ucGetTcVelMode();  /* 精调微调模式: 0=位置(每拍一步) 1=速度(连续) */
    int      iAccel      = iGetTcAccel();       /* 平移加速度/速度: 本步独立一对（粗调+精调共用） */
    int      iSpeed      = iGetTcSpeed();       /* 位置模式=纵移速度; 速度模式=微调速度上限 */

    /* ---- pi(树莓派 CSI)那一套（2026-10-16 加; 完整语义见 app_config.h「平移射靶」段头）----
     * ucUsePicam = 本次**请求**优先哪台: 进门快照, 之后只会被"未标定降级"改成 0;
     *              ⚠️ rev=1（靶排换边）**强制 0**: CSI 固定在底盘上, 不随臂转向另一边的靶排
     *              —— 细调不再优先 pi（"7" 不带 pi 标志, 线上与老方案一字不差）、
     *              粗调失败也不再拿 CSI 重试（见 ③）。tc_use_picam 蓝牙值不动 ——
     *              反向档"只能全程机械臂(主摄)瞄准"就是定案（2026-10-30）;
     *              日后若把 CSI 物理挪装到新一侧, 删掉这个强制即可恢复细调优先 pi。
     * ucCamNow   = 本拍**实际**是哪台在测: 跟着回帧 status 的 bit7 走（Pi 内部会自己从
     *              CSI 退回主摄, 可能**中途**变）—— 它是"用哪把瞄准尺"的唯一依据;
     * ucCamPrev  = 上一拍那个值（0xFF = 还没有过任何测量, 状态量不是相机码）。 */
    uint8_t  ucUsePicam  = ((ucRev != 0u) ? 0u : ucGetTcUsePicam());
    uint8_t  ucCamNow    = 0u;
    uint8_t  ucCamPrev   = 0xFFu;
    float    fAimColPi   = fGetTcCxAimPi();     /* CSI 那对瞄准常数（只分这一对, 见段头） */
    float    fAimRowPi   = fGetTcCyAimPi();

    int32_t  iQr;
    uint8_t  ucWant;
    stTargetTgtTdf astT[TARGET_MAX_COUNT];
    uint8_t  ucCnt;
    int      iPick;
    int      i;
    float    fCoarseResMm = 0.0f;   /* 粗调走完后的**主摄系**残差[mm]（2026-10-24 加, 带符号）。
                                     * 细调切到 pi 的第一拍打出来 —— (粗残差 + 细调首拍 d×px_to_mm)
                                     * 与 0 的差 = 两根瞄准列的固定失配, 标 tc_cx_aim_pi 用:
                                     *   新值 = 旧值 + (细调首拍d − 粗残差折px)  的口径一眼可读。 */
    uint32_t ulElapsed   = 0u;
    uint32_t ulFrameSeen;
    TickType_t xFrameTick;
    int32_t  iConvCnt    = 0;
    float    fTotalMm    = 0.0f;    /* 本步累计纵移[mm]（粗调+精调, keep_res 记账用） */
    float    fTravel     = 0.0f;    /* 精调累计**指令**行程[mm]（行程闸口径, 同 fChasTravel） */
    float    fDpx        = 0.0f;    /* 本拍沿排偏差[px]（swap=1 时 = cx−瞄准列） */
    float    fDmm        = 0.0f;    /* 本拍沿排偏差换算[mm]（>0 = 该往前走） */
    float    fMm;                   /* 本拍下发量（位置模式: mm） */
    int      iVy;                   /* 本拍下发量（速度模式: v_y rpm, 0=不出力） */
    float    fPrevAbs    = -1.0f;   /* 上一拍 |dpx|（noprog 判据, <0 = 还没有） */
    float    fPrevDpxAcc = 0.0f;    /* 上一拍被**采纳**的 dpx（跳变拒收用, 见下面的检查） */
    uint8_t  bHaveDpxAcc = 0u;      /* fPrevDpxAcc 有没有值（首拍/换相机/改瞄准列后清零） */
    uint8_t  ucJumpSkip  = 0u;      /* 连续被"跳变拒收"的拍数（连着 2 拍就采纳新的） */
    uint32_t ulNoprog    = 0u;
    uint32_t ulTick      = 0u;
    uint16_t usLastR     = 0u;      /* 最后跟到那颗靶的外圆半径[px]（fin 行用; 0=没跟到） */
    uint8_t  ucOk        = 0u;
    uint8_t  ucAbortNoprog = 0u;
    uint8_t  ucAbortTravel = 0u;
    uint8_t  ucAbortStop   = 0u;
    uint8_t  ucRejWarn     = 0u;    /* 半径门报过一次就不再刷（每步一次） */
    uint8_t  ucHadTgt      = 0u;    /* 上一有效拍有没有跟到靶（丢靶**边沿**判据, 2026-10-27 加:
                                     * 每次从"跟到"变"丢了"投一声 0.2s 短滴 —— 不是每帧都响,
                                     * 否则 20Hz 流里连丢 1s 就是 20 声; 重新跟到再丢才计下一声） */
    /* ---- 保持档（ucHold=1）专用 ---- */
    uint8_t  ucFrozen      = 0u;    /* 撞行程闸/无进展 → 停车冻结, 等下次微调自恢复（**不退出**） */
    uint8_t  ucStPrev      = AIM_STATE_COARSE; /* 上一拍发布的状态（只在变化时补一条日志） */
    float    fAimPrev      = 0.0f;  /* 上一拍用的瞄准常数: 变了 = 操作员在 OLED 上按了一格 */
    TickType_t xReviveTick = 0u;    /* 上次重发 "7" 的时刻（起流自愈的限速闸） */
    char     acDx[12];
    char     acDbg[RPI_DATA_MAX + 8u];

    /* ---- ① 二维码校验（与 vTargetCalibAim / ucRpiTargetServo 完全同款）---- */
    iQr = iGetQrCode();
    if (iQr < 0) { return 0u; }
    ucWant = (uint8_t)((iQr / 10) % 10);
    if ((ucWant < 1u) || (ucWant > 3u)) { return 0u; }

    /* ---- ② 瞄准常数未标定 → 拒绝动车（0 是哨兵值, 不是合法瞄准位置）----
     * swap=1(本车): 沿排那根轴由 cx 驱动 → 校验瞄准列; swap=0: 改由 cy 驱动 → 校验瞄准行。
     * tc_gap_px 同判: 0 = 没标"一格占多少 px", 粗调按格换算无从谈起。 */
    if (((ucSwap != 0u) && (fAimCol <= 0.0f))
        || ((ucSwap == 0u) && (fAimRow <= 0.0f))
        || (fGapPx <= 0.0f))
    {
        vRpiSendDebug("SHOT fail: aim not calibrated (read tc_cx_aim/gap_px)");
        return 0u;
    }

    /* ---- ②'' pi 瞄准列未标定 → 本次**细调相位自动降级成"只用主摄"**（2026-10-16 加）----
     * 这一条是本功能"出厂值（0）下完全惰性"的全部实现: 主摄那对已过上一道门 = 今天的
     * 行为一字不变, 只是多打一条 warn。故意**不蜂鸣**（未标定不是故障, 是还没标）。
     * ⚠️ 自 2026-10-18 起**粗调固定主摄**（见粗调段）, 所以这个位只决定细调那股 "7"
     *    要不要去试 pi: 0 = 细调也走主摄（老方案）, 1 = 细调优先 pi、起不来/掉帧自动退回。
     * ⚠️ 判定放在**主摄那道门之后**、且只判"优先的那一台":
     *    主摄那对必须永远可用 —— Pi 随时可能退回主摄, 两台都标好才敢走 pi 那一路。 */
    if (ucUsePicam != 0u)
    {
        if (((ucSwap != 0u) && (fAimColPi <= 0.0f))
            || ((ucSwap == 0u) && (fAimRowPi <= 0.0f)))
        {
            ucUsePicam = 0u;
            vRpiSendDebug("SHOT warn: pi aim not calibrated, use main");
        }
    }

    /* ---- ②' 把**本次实际在用的控制参数**打两行到 Pi 日志（照抄 ucRpiTargetServo 的 TGTS cfg）----
     * 为什么值得单独打: 「速度模式零速带卡住」「收敛阈值贴着检测噪声带」「增益/比例尺对不上」
     * 三类不收敛在现象上**都是"车不动、然后 timeout"**, 而处方完全不同（依次改 tc_px_to_mm /
     * tc_conv_px / tc_kp_scale）—— 看着这两行就知道环里到底在用哪套值, 不用再去蓝牙 Tab
     * 逐个核对。⚠️ 2026-10-07 现场就是因为没有这条, 只能从 SHOT n= 的 d= 与 mm= 反算比例尺。
     * ⚠️ 两行都必须留在 RPI_DATA_MAX(64) 以内: 超了是**从尾部静默截断**, 而尾巴上恰好是
     *    最后那个字段。故用紧凑写法（p=/k=/c= 不加多余空格）, 整数能表达的一律用 %d ——
     *    浮点走 bt_fmt_f2（全工程除 app_oled.c 外不用 %f）。
     * ⚠️ 位置在"未标定门"之后、粗调之前: 粗调失败(scan/颜色没找到)时同样打, 这两行是唯一
     *    能说明"当时参数是什么"的东西。每次调用（= 每个射靶步）打一次。 */
    {
        char acT1[12];
        char acT2[12];
        char acT3[12];

        bt_fmt_f2(acT1, (int)sizeof(acT1), fPxToMm);
        bt_fmt_f2(acT2, (int)sizeof(acT2), fKp);
        bt_fmt_f2(acT3, (int)sizeof(acT3), fConvPx);
        (void)snprintf(acDbg, sizeof(acDbg),
                       "SHOT cfg p=%s k=%s c=%s/%ld v=%u ct=%lu sp=%d st=%d",
                       acT1, acT2, acT3, (long)iConvTicks,
                       (unsigned)ucVelMode, (unsigned long)ulCtrlMs,
                       iSpeed, (int)fStepMax);
        vRpiSendDebug(acDbg);

        /* 第二行: 瞄准常数（打印**本次实际在用那根轴**的）、粗调尺、方向、死区/钳位/加速度。
         * ⚠️ a= 必须跟着 sw= 变 —— swap=0 时环读的是 tc_cy_aim, 打 cx 那个数会把人带偏。 */
        (void)snprintf(acDbg, sizeof(acDbg),
                       "SHOT cfg2 sw=%u a=%d g=%d/%d sg=%d db=%d mx=%d ac=%d",
                       (unsigned)ucSwap,
                       (int)((ucSwap != 0u) ? fAimCol : fAimRow),
                       (int)fGapMm, (int)fGapPx, (int)fFwdSign,
                       (int)fDbMm, (int)fMaxMm, iAccel);
        vRpiSendDebug(acDbg);

        /* 第三行（2026-10-16 加, 2026-10-18 改成 fine_pi=）: **细调相位**请求哪台相机
         * + pi 那根瞄准常数。⚠️ 粗调自 2026-10-18 起固定主摄（不看这个位, 见下面粗调段的
         * ⚠️⚠️）, 所以这里的值只说明"接下来的 "7" 优先用谁"。
         * ⚠️ 必须**另起一行** —— 上面 cfg2 那行已经贴着 RPI_DATA_MAX(64) 的静默截断线,
         *    再挂字段进去, 丢掉的正好是尾巴上的 ac=(实测踩过同类)。 */
        (void)snprintf(acDbg, sizeof(acDbg),
                       "SHOT cam: fine_pi=%u a_pi=%d g_px=%d",
                       (unsigned)ucUsePicam,
                       (int)((ucSwap != 0u) ? fAimColPi : fAimRowPi),
                       (int)fGapPx);
        vRpiSendDebug(acDbg);
    }

    /* ---- ②'' 进门基线 + 状态发布 ----
     * fAimPrev 记「本次实际在用那根轴」的瞄准常数 —— 保持环顶部靠它检测"操作员按了一格"
     * （按了 → 重置行程闸/noprog/收敛计数, 见循环体）。一次性档也照发, 遥测更有意义。 */
    fAimPrev = (ucSwap != 0u) ? fAimCol : fAimRow;
    vSetShootErr(0.0f);
    vSetShootSettleTick(0u);
    vSetShootState(AIM_STATE_COARSE);
    vSetShootCam(0u);       /* 进门先按主摄报（粗调那一帧回来后才改得准） */

    /* ---- ③ 粗调: 全画面识别(needAll=0: **目标色在即可**) → 按色选靶 → 一次纵移 ----
     * 复用 ucTargetScanOnce（发"6"+等 0x04）。⚠️ **不要求三颗齐** —— 那是「标靶对位」
     * 取"第二个=中间"才需要的; 本步按**颜色**找靶、不依赖位置序号, Pi 侧某色被散布
     * 作废（cnt<3）时目标色照样可用 —— 2026-10-07 现场踩过: 蓝 81px 冷启动散布被废,
     * cnt=2, 目标红明明识别得很好(961,635)r90 却整步拒绝。内层串说具体原因。 */
    /* ⚠️ 先清零 astT 再扫描: ucTargetScanOnce 内部走 vGetTargetResult, 后者**只填实际检到的
     *    ucCnt 颗**, 剩下的槽是栈上的垃圾。下面按 TARGET_MAX_COUNT 全扫时, 垃圾槽的 ucColor
     *    可能碰巧落在 1~3（= ucWant）→ 锁上一个根本不存在的靶。清零后垃圾槽 ucColor=0 且
     *    usR=0, 颜色与半径两道门都过不去。 */
    for (i = 0; i < (int)TARGET_MAX_COUNT; i++)
    {
        astT[i].ucColor = 0u;
        astT[i].usCx    = 0u;
        astT[i].usCy    = 0u;
        astT[i].usR     = 0u;
    }

    /* ⚠️⚠️ 粗调**首选主摄, 失败后用 CSI 重试一次**（2026-10-24 改, 原为固定主摄）——
     *    ⚠️ rev=1（靶排换边）时这次重试整个跳过（下面那道门）: CSI 不随臂转,
     *    看不到另一边的靶排 —— 反向档粗调也是纯主摄（2026-10-30 加）——
     *    首选主摄的理由不变（2026-10-18 定案的三条仍然全对）:
     *    ① 粗调的位移按**主摄像素尺**换算: fDmm = fDpx × tc_gap_mm / tc_gap_px;
     *       tc_gap_px 只有主摄那一把（实测 150mm ↔ 289px ≈ 1.9px/mm）。pi 满视场
     *       (2592×1944) 在同一距离是 ≈3.8px/mm（靶 r 95→190px, 正好两倍）⇒ 拿 pi 的
     *       像素差套主摄的格距会算出**约两倍**的位移 —— 所以 pi 那次重试**单独折算**
     *       （下面 fGapPx 就地乘 2, 见那条注释）;
     *    ② pi 的角视场更窄, 三颗排开时常只框到 1~2 颗 —— 但**目标色在**就够选靶
     *       （ucNeedAll=0）, 而主摄经常连目标色都框不到（机位远了/被臂挡）, 那时 pi
     *       靠分辨率优势往往还有（2026-10-24 实测: 主摄 `color 2 not found` 的同一
     *       位姿, pi 锁到 B(848)R(1393)G(1934) 三颗全在）;
     *    ③ 主摄冷开 ~1.3s、pi 冷启动 ~3s —— 所以 pi 只作**第二方案**, 主摄失败才轮到它
     *       （预热过就几乎不花额外时间; 等价于色球对位链"主方案失败→替补接上"的语义）。
     *    于是 tc_use_picam 仍只管**细调那股流**（下面的 "7"）, 本段只决定粗调那一下。 */
    if (!ucTargetScanOnce(astT, ulTimeoutMs, 0u, 0u))
    {
        /* ---- 主摄那一把没成 → CSI 重试一次（方案链第二槽, 学 ucRpiAlignChain）----
         * 前提: pi 那对瞄准常数可用（未标定时细调也会降级主摄, 这里同样不试）。
         * ⚠️ rev=1 时**跳过这次重试**: CSI 固定在底盘、看不到换边后的靶排,
         *    别白烧一个超时窗（I(0)）去等一台注定空手的相机。
         * ⚠️ ucTargetScanOnce 再发一次 "6" —— Pi 侧 tasks/_target.py 的 run_target_detect
         *    会先试 CSI 再退回主摄, 所以**重试的这台就是 pi**（除非 pi 也 fail, 那时
         *    返回的又是主摄结果, 与首试重复 —— 用 ucCamNow 判别, 重复则如实报失败）。 */
        if (((fAimColPi <= 0.5f) && (fAimRowPi <= 0.5f)) || (ucRev != 0u))
        {
            vRpiSendDebug((ucRev != 0u) ? "SHOT fail: scan (rev=1, no pi retry)"
                                        : "SHOT fail: scan (pi aim not calibrated, no retry)");
            return 0u;
        }
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT retry: coarse via pi");
        vRpiSendDebug(acDbg);
        if (!ucTargetScanOnce(astT, ulTimeoutMs, 0u, 1u))
        {
            vRpiSendDebug("SHOT fail: scan (both cameras)");
            return 0u;
        }
    }

    /* ---- ③' 这一帧是**哪台**测的（0x04 的 status bit7; Pi 内部可能已经退回主摄）----
     * 它是"用哪把瞄准尺"的唯一依据: 粗调这一下的选靶/偏差/半径门全都按它选。
     * ⚠️ 两台都不会是"未标定" —— 进门那两道门保证了两对常数都可用（见 ②'' 的注释）。 */
    ucCamNow  = ucGetTargetCam();
    ucCamPrev = ucCamNow;
    if (ucCamNow != 0u) { fAimCol = fAimColPi; fAimRow = fAimRowPi; }
    fAimPrev = (ucSwap != 0u) ? fAimCol : fAimRow;   /* 保持环的"操作员按了一格"基线 */
    vSetShootCam(ucCamNow);
    (void)snprintf(acDbg, sizeof(acDbg), "SHOT cam: now=%s",
                   (ucCamNow != 0u) ? "pi" : "main");
    vRpiSendDebug(acDbg);

    /* 选靶（2026-10-08 改）: 同色里取**离瞄准列最近**且半径合格的一颗（原为"取第一个同色的",
     * 背景同色物会抢走真靶 —— 见 iAimPickNearest 的注释）。 */
    iPick = iAimPickNearest(astT, (uint8_t)TARGET_MAX_COUNT, ucWant, ucSwap, fAimCol, fAimRow,
                            ucCamNow);
    if (iPick < 0)
    {
        /* 区分"这个颜色压根没检到"与"检到了但半径不合理": 前者是识别/跨帧散布问题,
         * 后者多半是背景同色物或误检 —— 处方完全不同, 别都报成 not found。 */
        uint8_t ucSeen = 0u;

        for (i = 0; i < (int)TARGET_MAX_COUNT; i++)
        {
            if (astT[i].ucColor == ucWant) { ucSeen = 1u; break; }
        }
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT fail: color %u %s",
                       (unsigned)ucWant, (ucSeen != 0u) ? "bad radius" : "not found");
        vRpiSendDebug(acDbg);

        /* ---- 目标色出画面 → 按 Pi 失败帧的方向字节**开环补一步再扫一次** ----
         * （2026-10-24 加, 学色球 ALIGN_PULLBACK 的"方向字节 + 固定步长"）:
         * 车偏出靶排太多时目标色直接出画面, 纯识别怎么扫都是 not found; 但 Pi 知道
         * "目标色往哪边出去了"（被半径门剔掉的候选 bbox 位置就是证据）→ 失败帧带 dir。
         * 处方: 沿 dir 走 **半格**（tc_gap_mm 的一半 ≈ 75mm —— 补步是"找回视野",
         * 不是对位, 走多了容易冲过头; 真剩大偏差由粗调/精调收）。每步最多补一次:
         * 与链"同一方案只试一次"同一条纪律, 不然识别坏了就在这死循环烧超时。
         * ⚠️ dir 语义（app_rpi.h::RPI_TGT_FAIL_DIR_OFF）: 1=目标在画面**右**侧出界
         *    = 车要**往右**去追 = 沿排 +; 2=左。符号仍乘 fFwdSign（方向标定翻转时
         *    补步跟着翻, 与粗调同一条链）。 */
        {
            uint8_t ucDir = ucGetTargetPullDir();

            vSetTargetPullDir(0u);           /* 消费即清（快照语义, 同 BallPullDir） */
            if (ucDir != 0u)
            {
                float fStepMm = ((ucDir == 1u) ? 1.0f : -1.0f) * (fGapMm * 0.5f) * fFwdSign;
                char acStep[12];

                bt_fmt_f2(acStep, (int)sizeof(acStep), fStepMm);
                (void)snprintf(acDbg, sizeof(acDbg), "SHOT retry: dir=%u step=%smm",
                               (unsigned)ucDir, acStep);
                vRpiSendDebug(acDbg);
                vChassisMoveCameraForward(fStepMm, iAccel, iSpeed);
                fTotalMm += fStepMm;

                for (i = 0; i < (int)TARGET_MAX_COUNT; i++)   /* 同上面的清零纪律 */
                {
                    astT[i].ucColor = 0u; astT[i].usCx = 0u;
                    astT[i].usCy = 0u;    astT[i].usR = 0u;
                }
                if (ucTargetScanOnce(astT, ulTimeoutMs, 0u, 0u))
                {
                    ucCamNow  = ucGetTargetCam();
                    if (ucCamNow != 0u) { fAimCol = fAimColPi; fAimRow = fAimRowPi; }
                    ucCamPrev = ucCamNow;
                    fAimPrev = (ucSwap != 0u) ? fAimCol : fAimRow;
                    vSetShootCam(ucCamNow);
                    iPick = iAimPickNearest(astT, (uint8_t)TARGET_MAX_COUNT, ucWant,
                                            ucSwap, fAimCol, fAimRow, ucCamNow);
                    if (iPick >= 0)
                    {
                        (void)snprintf(acDbg, sizeof(acDbg),
                                       "SHOT retry: found after step (cam=%s)",
                                       (ucCamNow != 0u) ? "pi" : "main");
                        vRpiSendDebug(acDbg);
                    }
                }
                if (iPick < 0)
                {
                    /* 补了也没找回 —— 如实报失败（别在下面拿过期数据算偏差） */
                    (void)snprintf(acDbg, sizeof(acDbg), "SHOT fail: color %u not found (after step)",
                                   (unsigned)ucWant);
                    vRpiSendDebug(acDbg);
                    return 0u;
                }
            }
            else
            {
                return 0u;               /* 无方向信息（旧 Pi / 目标色压根没出现）: 原行为 */
            }
        }
    }

    /* 粗调偏差 → **按格换算**: 位移 = 视觉偏差 ÷ 每格px × 格距mm —— "打边靶走多大"
     * 只由 tc_gap_mm 决定（用户改 160 只放大粗调, 精调环的 tc_px_to_mm 不受影响）。
     * swap=1(本车): 沿排纵移 ← cx; swap=0: 纵移 ← cy。连续换算不取整格:
     * 瞄准列的标定残差(±几px)被线性带进位移, 由精调环收掉, 无跳变。
     * ⚠️ 本次是 **pi 测的**（方案链第二槽, 2026-10-24 加）→ 每格px 要**就地乘 2**:
     *    pi 满视场的像素密度是主摄的两倍（同一物理格距 150mm, 主摄 289px、pi ≈578px,
     *    与"靶 r 95→190"一致）。不乘的话位移会算出约两倍 —— 正是 2026-10-18 那条
     *    "pi 像素差套主摄格距"警告的场景; 当时靠"固定主摄"回避, 方案链放开后必须折算。
     *    ⚠️ 折算系数**硬编码 2.0** 而不是新标定量: 它 = 两台相机的取流宽度比
     *       (2592/1920 = 1.35) × 靶在画面里的占比差（pi 视场窄、同距下靶占比更大）
     *       的乘积, 实测就是 ~2.0（r 95→190 是最直接的证据）。真发现差, 优先改
     *       tc_gap_px 把主摄那把标准, 这里保持简单。 */
    fDpx = (ucSwap != 0u) ? ((float)astT[iPick].usCx - fAimCol)
                          : ((float)astT[iPick].usCy - fAimRow);
    fDmm = fDpx * fGapMm / (fGapPx * ((ucCamNow != 0u) ? 2.0f : 1.0f)) * fFwdSign;

    (void)snprintf(acDbg, sizeof(acDbg), "SHOT meas: #%u col=%u(%u,%u) r=%u c=%u",
                   (unsigned)iPick, (unsigned)astT[iPick].ucColor,
                   (unsigned)astT[iPick].usCx, (unsigned)astT[iPick].usCy,
                   (unsigned)astT[iPick].usR, (unsigned)ucCamNow);
    vRpiSendDebug(acDbg);

    /* 粗调安全夹: 误检带飞的敞口由 tc_max_mm 限定（150mm 间距 + 停车误差, 250 够兜） */
    if (fDmm >  fMaxMm) { fDmm =  fMaxMm; }
    if (fDmm < -fMaxMm) { fDmm = -fMaxMm; }

    if (ucGetScriptAbortReq() != 0u)
    {
        vRpiSendDebug("SHOT abort: stop by request");
        return 0u;
    }

    {
        uint8_t ucMove = ((fDmm > fDbMm) || (fDmm < -fDbMm)) ? 1u : 0u;

        bt_fmt_f2(acDx, (int)sizeof(acDx), fDmm);
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT fix: dy=%smm(%s gap=%.1f)",
                       acDx, ucMove ? ((fDmm > 0.0f) ? "F" : "B") : "skip",
                       (double)(fDpx / fGapPx));   /* 偏差折几格 —— 走多大 = 格数 × tc_gap_mm */
        vRpiSendDebug(acDbg);

        if (ucMove != 0u)
        {
            vChassisMoveCameraForward(fDmm, iAccel, iSpeed);   /* 位置模式阻塞到位（沿排纵移） */
            fTotalMm += fDmm;
        }
        fCoarseResMm = fDmm;    /* 粗调残差（带符号）—— 细调切到 pi 的第一拍打出来（标 tc_cx_aim_pi 用） */
    }

    /* ---- ④ 精调: 起流 → 位置模式小步闭环 ----
     * 起流前记基线（帧计数从这一刻开始涨才算"流真的来了"）;
     * 起流帧等链路; 失败时拒绝返回 —— 流根本没起来, **不发 "8"**（同 ucRpiTargetServo）。 */
    ulFrameSeen = ulGetAimFrameCount();
    xFrameTick  = xTaskGetTickCount();

    if (ucRpiSendTaskCmdWaitPi(RPI_TASK_AIM_START, 0u, ucUsePicam) == 0u)
    {
        return 0u;
    }

    /* ---- ④' 起流成功 → **先等车真的停稳**, 再进精调环（2026-10-08 加, 见 AIM_SETTLE_* 段注释）----
     * 粗调的"到位"是纯时间估算, 估短了车还在跑; 此刻读到的 d 指错方向, 精调环会朝错的方向
     * 出力（现场: 精调前三拍多退了 46mm, 把粗调超调又放大一截）。这一段纯粹是"让测量可信"。
     * ⚠️ 中止位照查（同主循环纪律）; 全部耗时**计入 ulElapsed**（step 总预算口径不变）。
     * ⚠️ 停稳窗的上限**从第一帧算起**, 不是从这个函数进入算起 —— 起流到出帧本身要几百 ms,
     *    从进入算的话上限会先被吃掉、白等一轮（那正是这条要修的东西）。 */
    {
        uint32_t ulT0     = 0u;                       /* 停稳窗自身耗时（从第一帧起算） */
        uint32_t ulSeen   = ulGetAimFrameCount();
        float    fPrevCx  = 0.0f;
        uint8_t  ucHave   = 0u;                       /* 已经有上一帧可比较 */
        uint8_t  ucStable = 0u;                       /* 连续"没动"的帧数 */
        uint8_t  ucOkSet  = 0u;                       /* 1 = 真判到停稳（0 = 撞上限） */
        uint8_t  ucBegin  = 0u;                       /* 已见到第一帧 */

        while (ulElapsed < ulTimeoutMs)
        {
            if (ucGetScriptAbortReq() != 0u) { break; }
            if ((ucBegin != 0u) && (ulT0 >= AIM_SETTLE_TIMEOUT_MS)) { break; }

            vTaskDelay(pdMS_TO_TICKS(AIM_SETTLE_POLL_MS));
            ulElapsed += AIM_SETTLE_POLL_MS;
            if (ucBegin != 0u) { ulT0 += AIM_SETTLE_POLL_MS; }

            if (ulGetAimFrameCount() == ulSeen) { continue; }   /* 还没有新帧 */
            ulSeen  = ulGetAimFrameCount();
            ucBegin = 1u;

            vGetAimFrame(&ucCnt, astT, &ucCamNow);
            if (ucCamNow != ucCamPrev)
            {
                /* 相机在停稳窗里换了（Pi 从 CSI 退回主摄）→ 换瞄准尺, 并且**重新数**:
                 * 两个相机的坐标不可比, 拿旧的一帧减新的那一帧会造出一个假的"动了"。
                 * ⚠️ fAimPrev 也要同步过去 —— 否则保持档下一拍会把它当成"操作员按了一格"
                 *    而白丢一次行程闸。 */
                fAimCol   = (ucCamNow != 0u) ? fGetTcCxAimPi() : fGetTcCxAim();
                fAimRow   = (ucCamNow != 0u) ? fGetTcCyAimPi() : fGetTcCyAim();
                fAimPrev  = (ucSwap != 0u) ? fAimCol : fAimRow;
                ucCamPrev = ucCamNow;
                ucHave    = 0u;
                ucStable  = 0u;
                vSetShootCam(ucCamNow);
                (void)snprintf(acDbg, sizeof(acDbg), "SHOT cam: now=%s",
                               (ucCamNow != 0u) ? "pi" : "main");
                vRpiSendDebug(acDbg);
                /* ⚠️ 切到 pi 的那一刻把**粗调残差**一并打出（2026-10-24 加, 只在细调段
                 *    第一次换相机时打一次 —— coarseres 已消费就置 NaN 哨兵免得每拍刷屏）:
                 *    (粗残差 mm − 细调首拍 d×tc_px_to_mm) ≈ 两根瞄准列的固定失配,
                 *    标 tc_cx_aim_pi 的口径 = 旧值 + 失配折 px。 */
                if ((ucCamNow != 0u) && (fCoarseResMm == fCoarseResMm))
                {
                    char acRes[12];
                    bt_fmt_f2(acRes, (int)sizeof(acRes), fCoarseResMm);
                    (void)snprintf(acDbg, sizeof(acDbg), "SHOT coarse-res: %smm", acRes);
                    vRpiSendDebug(acDbg);
                    fCoarseResMm = (float)NAN;         /* NaN = 已打, 不再刷; NAN 替 0.0f/0.0f 免 #222-D */
                }
            }
            if (ucCnt == 0u) { ucHave = 0u; ucStable = 0u; continue; }

            iPick = iAimPickNearest(astT, ucCnt, ucWant, ucSwap, fAimCol, fAimRow, ucCamNow);
            if (iPick < 0)
            {
                iPick = iAimPickNearest(astT, ucCnt, RPI_AIM_COLOR_UNKNOWN,
                                        ucSwap, fAimCol, fAimRow, ucCamNow);
            }
            if (iPick < 0) { ucHave = 0u; ucStable = 0u; continue; }

            {
                float fCx    = (ucSwap != 0u) ? (float)astT[iPick].usCx
                                              : (float)astT[iPick].usCy;
                float fDelta = fCx - fPrevCx;

                if (fDelta < 0.0f) { fDelta = -fDelta; }
                if ((ucHave != 0u) && (fDelta < AIM_SETTLE_PX)) { ucStable++; }
                else                                            { ucStable = 0u; }
                fPrevCx = fCx;
                ucHave  = 1u;
            }

            if (ucStable >= AIM_SETTLE_FRAMES) { ucOkSet = 1u; break; }
        }

        (void)snprintf(acDbg, sizeof(acDbg), "SHOT settle: t=%lums s=%u",
                       (unsigned long)ulT0, (unsigned)ucOkSet);
        vRpiSendDebug(acDbg);
    }

    /* 停稳窗跨了好几个 tick（也吃掉一点预算）, 重记一次新鲜度基线 ——
     * 否则精调第一拍进去就会撞上 stale 分支、白丢一拍。 */
    ulFrameSeen = ulGetAimFrameCount();
    xFrameTick  = xTaskGetTickCount();

    /* ⚠️ 保持档**不限时**: 退出只由中止位（与内部闸门的"冻结"）决定 —— 见 ucHold 的说明。 */
    while ((ucHold != 0u) || (ulElapsed < ulTimeoutMs))
    {
        /* 中止位: 必须在新鲜度判定之前（Stale 分支会 continue, 放后面链路一断就看不见） */
        if (ucGetScriptAbortReq() != 0u) { ucAbortStop = 1u; break; }

        /* ---- 保持档: 每拍重读瞄准常数与收敛阈值 ----------------------------------
         * 这一条就是"在 OLED 上按一格、车就跟到新目标"的全部实现。
         * ⚠️ 瞄准常数一变就**重置行程闸/noprog/收敛计数**（"每次微调一套新预算"）: 否则
         *    150mm 的行程闸会被一次次微调累计用光, 几分钟后必然撞闸冻结（看着像"这步坏了"）。
         * ⚠️ 两根瞄准常数**成对重读**: swap 选中的那根变了就两根都刷新, 免得中途改 swap 读到
         *    新旧混搭。⚠️ 冻结态也在这里复位 —— 按一格 = 操作员明确要求"再试一次"。 */
        if (ucHold != 0u)
        {
            /* ⚠️ 每拍重读的是**当帧那台相机**那一对（ucCamNow 跟着回帧走）——
             *    Pi 中途从 CSI 退回主摄时, 环要立刻改用主摄那把瞄准尺（别再读 CSI 那根）。
             *    相机**换**这件事由下面读帧那一处统一处理（换尺 + 同步 fAimPrev）。 */
            float fAimNow = (ucCamNow != 0u)
                          ? ((ucSwap != 0u) ? fGetTcCxAimPi() : fGetTcCyAimPi())
                          : ((ucSwap != 0u) ? fGetTcCxAim()   : fGetTcCyAim());

            fConvPx    = fGetTcConvPx();        /* OLED「收敛阈值」当场生效 */
            iConvTicks = iGetTcConvTicks();
            if (fAimNow != fAimPrev)
            {
                /* 操作员在 OLED 上按了一格（瞄准常数变了）→ 全套复位, 理由见函数头。
                 * ⚠️ 两根**成对**刷新: swap 选中的那根变了就两根都刷新, 免得读到新旧混搭。
                 * ⚠️ 冻结态也在这里复位 —— 按一格 = 操作员明确要求"再试一次"。 */
                fAimCol  = (ucCamNow != 0u) ? fGetTcCxAimPi() : fGetTcCxAim();
                fAimRow  = (ucCamNow != 0u) ? fGetTcCyAimPi() : fGetTcCyAim();
                fAimPrev = fAimNow;
                fTravel  = 0.0f;
                ulNoprog = 0u;
                fPrevAbs = -1.0f;
                bHaveDpxAcc = 0u;       /* 改瞄准列 = 坐标系基准变了, 跳变判据从头算 */
                ucJumpSkip  = 0u;
                iConvCnt = 0;
                ucOk     = 0u;
                ucFrozen = 0u;
                vSetShootSettleTick(0u);
                (void)snprintf(acDbg, sizeof(acDbg), "SHOT hold: aim->%dpx", (int)fAimNow);
                vRpiSendDebug(acDbg);
            }
        }

        /* 节流: 平移阻塞（fChassisMoveDelay）自带停稳, 这里只负责测量间隔 */
        vTaskDelay(pdMS_TO_TICKS(ulCtrlMs));
        ulElapsed += ulCtrlMs;

        /* ---- 保持档·冻结: 撞过闸 / 判过"环没起作用" → 停车静默, 等操作员再按一格 ----
         * 放在测帧之前: 冻结期间连靶都不用找, 只维持节拍与新鲜度基线。
         * ⚠️ 冻结**不是退出** —— 一次误按不该把整个测试会话搞没（那正是"永不结束"的语义）。 */
        if (ucFrozen != 0u)
        {
            if (ulGetAimFrameCount() != ulFrameSeen)
            {
                ulFrameSeen = ulGetAimFrameCount();
                xFrameTick  = xTaskGetTickCount();
            }
            continue;
        }

        /* 新鲜度: 帧计数停涨 = Pi 掉线/停发 → 冻结（不动车、不判收敛）, 回来即续跑 */
        if (ulGetAimFrameCount() != ulFrameSeen)
        {
            ulFrameSeen = ulGetAimFrameCount();
            xFrameTick  = xTaskGetTickCount();
        }
        else if ((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(ulStaleMs))
        {
            iConvCnt = 0;
            if (ucHold != 0u) { vSetShootState(AIM_STATE_STALE); }  /* 屏上要说清"不是我不动, 是没测量" */
            /* ---- 保持档·起流自愈（2026-10-09 加, **必需**）----
             * Pi 侧 `TARGET_STREAM_MAX_S`(12s) 到点会让流线程**自己退出**（安全阀: 防 "8" 丢失后
             * 一直占着相机）, 而本环**不判超时** → 没有这条就是"冻在那儿永不自愈"。
             * 限速重发 "7"（只发不等, 同收尾帧纪律）: 新线程起来后帧号继续涨, 环自动续跑。
             * ⚠️ 阈值取 2×stale（不是 1×）: 单次丢几帧是常态, 别一抖就重起流。 */
            if ((ucHold != 0u)
                && ((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(2u * ulStaleMs))
                && ((xTaskGetTickCount() - xReviveTick) > pdMS_TO_TICKS(AIM_HOLD_REVIVE_MS)))
            {
                xReviveTick = xTaskGetTickCount();
                vRpiSendTaskCmdFlag(RPI_TASK_AIM_START, ucUsePicam);
                vRpiSendDebug("SHOT hold: stream stale, re-arm");
            }
            continue;
        }

        vGetAimFrame(&ucCnt, astT, &ucCamNow);

        /* ---- 相机变了（Pi 从 CSI 退回主摄 / 反过来）→ **换瞄准尺**（2026-10-16 加）----
         * 必须在这里、拿到本帧 cam 之后**立刻**换: 下面选靶/算偏差马上要用它, 慢一拍
         * 就是拿新相机的坐标去减旧相机的瞄准列 —— 静默的错值（本步最怕的那种）。
         * 两台坐标系不可比, 所以连续量（收敛计数 / noprog 基线）一并清零, 免得把跳变
         * 当成"没有进展"误判 noprog; fAimPrev 同步过去, 免得保持档下一拍把它当成
         * "操作员按了一格"而白丢行程闸。
         * ⚠️ **不重置行程闸**（两种档都不）: 它夹的是"这一步真走了多少 mm"的安全量,
         *    回落后重置等于白送一倍; Pi 侧"退回后不再重试 CSI"保证了每步最多换一次。 */
        if (ucCamNow != ucCamPrev)
        {
            fAimCol   = (ucCamNow != 0u) ? fGetTcCxAimPi() : fGetTcCxAim();
            fAimRow   = (ucCamNow != 0u) ? fGetTcCyAimPi() : fGetTcCyAim();
            fAimPrev  = (ucSwap != 0u) ? fAimCol : fAimRow;
            ucCamPrev = ucCamNow;
            iConvCnt  = 0;
            fPrevAbs  = -1.0f;
            bHaveDpxAcc = 0u;           /* 换相机 = 换坐标系, 跳变判据从头算（见 AIM_JUMP_PX） */
            ucJumpSkip  = 0u;
            vSetShootCam(ucCamNow);
            (void)snprintf(acDbg, sizeof(acDbg), "SHOT cam: now=%s",
                           (ucCamNow != 0u) ? "pi" : "main");
            vRpiSendDebug(acDbg);
        }

        if (ucCnt == 0u)
        {
            if (ucHadTgt != 0u)
            {
                vSetBuzzerReq(BUZZER_PAT_TICK);        /* 丢靶一次: 短滴一声 */
                ucHadTgt = 0u;
            }
            iConvCnt = 0;
            if (ucHold != 0u) { vSetShootState(AIM_STATE_NOTGT); }
            continue;                                  /* 流活着, 本帧没找到靶 */
        }

        /* 选靶（2026-10-08 改）: 同色里取**离瞄准列最近**且半径合格的一颗;
         * 本帧没有该颜色时, 退到无色帧(0xFF)、同一套规则（照抄 ucRpiTargetServo 的兜底语义）。
         * ⚠️ 别退回"取第一个同色的" —— 背景同色物会抢走真靶, 见 iAimPickNearest 的注释。 */
        iPick = iAimPickNearest(astT, ucCnt, ucWant, ucSwap, fAimCol, fAimRow, ucCamNow);
        if (iPick < 0)
        {
            iPick = iAimPickNearest(astT, ucCnt, RPI_AIM_COLOR_UNKNOWN,
                                    ucSwap, fAimCol, fAimRow, ucCamNow);
        }
        if (iPick < 0)
        {
            /* 该颜色检到了、但半径不合理 → 多半是背景同色物（柜子）或误检。每步只报一次。 */
            if (ucRejWarn == 0u)
            {
                uint8_t ucI;

                for (ucI = 0u; ucI < ucCnt; ucI++)
                {
                    if ((astT[ucI].ucColor == ucWant)
                        && (ucAimRadiusOk(astT[ucI].usR, ucCamNow) == 0u))
                    {
                        ucRejWarn = 1u;
                        (void)snprintf(acDbg, sizeof(acDbg),
                                       "SHOT warn: color %u r=%upx rejected",
                                       (unsigned)ucWant, (unsigned)astT[ucI].usR);
                        vRpiSendDebug(acDbg);
                        break;
                    }
                }
            }
            if (ucHadTgt != 0u)
            {
                vSetBuzzerReq(BUZZER_PAT_TICK);        /* 本帧没跟到靶: 同丢靶口径 */
                ucHadTgt = 0u;
            }
            iConvCnt = 0;
            if (ucHold != 0u) { vSetShootState(AIM_STATE_NOTGT); }
            continue;                                  /* 没有目标色、也没有无色帧 */
        }
        usLastR = astT[iPick].usR;
        ucHadTgt = 1u;                                 /* 这一拍跟到了（下次丢才算"丢一次"） */

        fDpx = (ucSwap != 0u) ? ((float)astT[iPick].usCx - fAimCol)
                              : ((float)astT[iPick].usCy - fAimRow);

        /* ---- 测量跳变拒收（见 AIM_JUMP_PX 的注释）: 这一拍量到的怕是"另一颗靶" ----
         * 当丢帧处理: 不计数、不出力、不累计行程, 直接把这一拍跳过。
         * ⚠️ 必须在 fDmm / noprog / 收敛计数**之前** —— 否则坏值照样会被算进 noprog 基线
         *    （fPrevAbs）和收敛计数, 等于没拦。
         * ⚠️ 首拍（bHaveDpxAcc=0）不判: 粗调走完那一刻的真残差可能就是几百 px, 那是合法的。
         * ⚠️ 换相机 / 操作员改瞄准列时由下面那两处把 bHaveDpxAcc 清零（坐标系换了, 前后不可比）。 */
        if (bHaveDpxAcc != 0u)
        {
            float fJump = fDpx - fPrevDpxAcc;

            if (((fJump > AIM_JUMP_PX) || (fJump < -AIM_JUMP_PX))
                && (ucJumpSkip < AIM_JUMP_SKIP_MAX))
            {
                ucJumpSkip++;
                (void)snprintf(acDbg, sizeof(acDbg),
                               "SHOT warn: d jump %dpx rejected (prev %dpx)",
                               (int)fDpx, (int)fPrevDpxAcc);
                vRpiSendDebug(acDbg);
                continue;
            }
        }
        ucJumpSkip    = 0u;
        fPrevDpxAcc   = fDpx;
        bHaveDpxAcc   = 1u;

        fDmm = fDpx * fPxToMm * fFwdSign;

        vSetShootErr(fDpx);     /* 保持档遥测: 每拍都发（OLED 标签的 d= 与蓝牙 Tab 都读它） */

        /* 过程串（节流）: 分不清"方向反了"还是"振荡"时看这一串（同 TGTS n= 的理由）。
         * ⚠️ 2026-10-08 起**第 1 拍也打**: 原来每 3 拍才打, 前 540ms 是盲区, 而「粗调走完
         *    那一刻的真残差」恰好落在里面 —— 2026-10-08 那发跑飞的 tot 与实测 d 推不出同一
         *    方向, 就是查不到这一拍才定不了案。n=1 那次几乎没有精调介入, 读到的就是真残差。
         * ⚠️ 保持档**另用一套节流**: 死区外每拍打（正在修正, 属暂态）, 带内只 ~2s 一条心跳 ——
         *    否则跑上几分钟, 带内那几百拍会把 Pi 日志撑爆（"永不结束"必然带来的问题）。 */
        ulTick++;
        {
            uint8_t ucPrint;

            if (ulTick == 1u) { ucPrint = 1u; }
            else if ((ucHold != 0u) && (fDpx < fConvPx) && (fDpx > -fConvPx))
            {
                ucPrint = ((ulTick % AIM_HOLD_TRACE_TICKS) == 0u) ? 1u : 0u;
            }
            else
            {
                ucPrint = ((ulTick % AIM_TRACE_EVERY_TICKS) == 0u) ? 1u : 0u;
            }

            if (ucPrint != 0u)
            {
                (void)snprintf(acDbg, sizeof(acDbg), "SHOT n=%lu d=%dpx mm=%dmm tr=%dmm tot=%dmm",
                               (unsigned long)ulTick, (int)fDpx, (int)fDmm,
                               (int)fTravel, (int)fTotalMm);
                vRpiSendDebug(acDbg);
            }
        }

        /* 收敛判据: |dpx| 连续 N 拍进死区。
         * ⚠️ 必须**排在 noprog 之前**（2026-10-07 现场教训）: d 在检测噪声带内抖动时,
         *    偶尔跳到阈值边界的拍会把 iConvCnt 清零, 连续 5 拍攒不满; 而 noprog 把
         *    "在靶上稳定不降"当成"偏差不下降"先攒到 6 拍误杀 —— 现象正是 d=1px
         *    已经稳稳对准却 `SHOT fail: noprog` 放弃。死区内的拍在这里 continue 掉,
         *    根本走不到 noprog —— noprog 只抓"死区外还卡着不降"的真故障。
         * ⚠️ 速度模式（tc_vel_mode=1）在靶的那拍要**显式发 0 停车** —— 速度指令是
         *    持久的, 上拍发的速度不会自动消失; 位置模式自终止不用管。 */
        if ((fDpx > -fConvPx) && (fDpx < fConvPx))
        {
            iConvCnt++;
            if (iConvCnt >= iConvTicks)
            {
                ucOk = 1u;                  /* 一次性档: 收敛 = 出口; 保持档: 只记"稳住了" */
                if (ucHold == 0u) { break; }
                /* 保持档: 不退出。状态转 SETTLED 的**那一刻**记一次 tick（OLED 靠它做
                 * 1s 的 OK! 窗）; 只在"不是已经稳住"的边沿写, 免得每拍都刷新窗口。 */
                if (ucStPrev != AIM_STATE_SETTLED)
                {
                    ucStPrev = AIM_STATE_SETTLED;
                    vSetShootSettleTick(xTaskGetTickCount());
                    vSetShootState(AIM_STATE_SETTLED);
                    (void)snprintf(acDbg, sizeof(acDbg), "SHOT hold: settled d=%dpx", (int)fDpx);
                    vRpiSendDebug(acDbg);
                }
            }
            if (ucVelMode != 0u) { vChassisMove(0, 0, 0); }
            continue;                       /* 在靶上: 这一拍不推车、不数 noprog */
        }
        iConvCnt = 0;

        /* 偏差 → 输出（双模式, 2026-10-07 加 tc_vel_mode 切换）:
         * · 位置模式: 每拍走 fMm = 偏差mm × 增益, 钳 ±tc_step_mm, 阻塞到位（步进量化
         *   0.095mm 很细, 但"走→停→测"每拍一顿 —— 用户嫌顿挫）;
         * · 速度模式: 每拍发 v_y = 偏差mm × 增益 ÷ 5.079 的 **rpm 连续修**（1rpm =
         *   5.08mm/s, EMM uint16 整数量化）, 平滑无顿挫; 钳 ±tc_speed;
         *   死区外换算不足 1rpm 的"零速带"由上面的顶格逻辑兜住（发最小 1rpm）。 */
        iVy = 0;
        fMm = fDmm * fKp;
        if (ucVelMode != 0u)
        {
            iVy = (int)(fMm / 5.079f);
            if (iVy == 0)
            {
                /* ---- 零速带顶格（2026-10-07）----
                 * 偏差在死区外但比例律换算不足 1rpm（0.25mm/px × 0.8 增益下要 ~20px
                 * 才够 1rpm —— 死区 3px 到 20px 之间全程零速, 车停着 d 不降, 纯耗超时）。
                 * 处方: 朝修正方向发**最小 1rpm** 顶一下 —— 1rpm × 180ms ≈ 0.9mm ≈ 3.6px,
                 * 一步跨进 3px 收敛带; 过冲最多到对侧 ~0.5px 仍在带内, 不会振。 */
                iVy = (fMm >= 0.0f) ? 1 : -1;
            }
            if (iVy >  iSpeed) { iVy =  iSpeed; }
            if (iVy < -iSpeed) { iVy = -iSpeed; }
        }
        else
        {
            if (fMm >  fStepMax) { fMm =  fStepMax; }
            if (fMm < -fStepMax) { fMm = -fStepMax; }
        }

        /* 行程闸（先累加后判定, 超限拍不下发）。速度模式按"指令速度 × 节拍"折 mm
         * （指令值口径, 同位置模式 —— 打滑/不出力照样涨, 只当安全夹）。 */
        if (ucVelMode != 0u)
        {
            fTravel += (((iVy >= 0) ? (float)iVy : -(float)iVy) * 5.079f
                        * ((float)ulCtrlMs / 1000.0f));
        }
        else
        {
            fTravel += (fMm >= 0.0f) ? fMm : -fMm;
        }
        if (fTravel > fTravelMax)
        {
            if (ucHold == 0u) { ucAbortTravel = 1u; break; }
            /* 保持档: **不退出** —— 停车冻结 + 一条日志, 等操作员再按一格
             * （那一按在循环顶部把行程闸清零, 等于"再给一套新预算"）。 */
            ucFrozen = 1u;
            ucStPrev = AIM_STATE_FROZEN;
            if (ucVelMode != 0u) { vChassisMove(0, 0, 0); }
            vSetShootState(AIM_STATE_FROZEN);
            (void)snprintf(acDbg, sizeof(acDbg), "SHOT hold: travel gate tr=%dmm frozen",
                           (int)fTravel);
            vRpiSendDebug(acDbg);
            continue;
        }

        /* noprog 早退（**只看死区外且本拍真出过力的拍**）: |dpx| 连续不下降 = 环没起作用
         * （方向反了会单调变大, 比例尺离谱/被顶住会恒定不降）。死区内的拍已在上面
         * continue; 零速带已由顶格逻辑兜成 ±1rpm（出力拍）, 这个"没出力"的豁免
         * 现在只防 tc_speed=0（用户把速度设 0 = 明确不要它动）。 */
        {
            uint8_t ucMoved = (ucVelMode != 0u) ? ((iVy != 0) ? 1u : 0u)
                                                : ((fMm != 0.0f) ? 1u : 0u);

            if (ucMoved != 0u)
            {
                float fAbs = (fDpx >= 0.0f) ? fDpx : -fDpx;

                if ((fPrevAbs >= 0.0f) && (fAbs > (fPrevAbs - AIM_NOPROG_EPS_PX)))
                {
                    ulNoprog++;
                }
                else
                {
                    ulNoprog = 0u;
                }
                fPrevAbs = fAbs;
                if (ulNoprog >= AIM_NOPROG_TICKS)
                {
                    if (ucHold == 0u) { ucAbortNoprog = 1u; break; }
                    /* 保持档: 同上 —— 方向反了/被顶住要**看得见**（冻结 + 日志 + 屏上 FROZEN）,
                     * 而不是把整个测试会话一刀结束。 */
                    ucFrozen = 1u;
                    ucStPrev = AIM_STATE_FROZEN;
                    if (ucVelMode != 0u) { vChassisMove(0, 0, 0); }
                    vSetShootState(AIM_STATE_FROZEN);
                    (void)snprintf(acDbg, sizeof(acDbg), "SHOT hold: noprog d=%dpx frozen",
                                   (int)fDpx);
                    vRpiSendDebug(acDbg);
                    continue;
                }
            }
        }

        /* 保持档: 走到这儿就是"死区外"（收敛那拍上面已经 continue 了）→ 本拍真出力,
         * 状态转 MOVING。OLED 那张「运动中」是给操作员看"现在别开火"的安全提示。 */
        if ((ucHold != 0u) && (ucStPrev != AIM_STATE_MOVING))
        {
            ucStPrev = AIM_STATE_MOVING;
            vSetShootState(AIM_STATE_MOVING);
        }

        /* 下发。速度模式 v_y>0=前进（vChassisMove 约定, 与位置模式 Forward 同向）,
         * **持久指令**: 死区/收敛/中止的出口都靠上面的显式 0 或下面的收尾停车兜住。 */
        if (ucVelMode != 0u)
        {
            if (iVy != 0)
            {
                vChassisMove(0, iVy, 0);        /* 沿排纵移（速度模式, 连续） */
                /* ⚠️ fTotalMm 必须**带符号**累计（iVy 本身带方向）—— 残留记账靠它,
                 *    取绝对值会把打左靶（负向）精调段的符号丢掉, 残留补错方向
                 *    （2026-10-07 修）; fTravel 那边才取绝对值（安全夹口径）。 */
                fTotalMm += ((float)iVy * 5.079f * ((float)ulCtrlMs / 1000.0f));
            }
        }
        else
        {
            if (fMm != 0.0f)
            {
                vChassisMoveCameraForward(fMm, iAccel, iSpeed); /* 沿排纵移（位置模式阻塞到位） */
                fTotalMm += fMm;
            }
        }
    }

    /* ---- ⑤ 停流: 唯一出口（同 ucRpiTargetServo, 少了它 Pi 会一直占着相机）----
     * ⚠️ 速度模式必须**显式停车**（速度指令持久, 不发 0 会带着末速一直爬 —— 位置模式
     *    自终止, 发 0,0,0 反而会主动中止在飞的位置移动, 所以只在速度模式发）。 */
    if (ucVelMode != 0u)
    {
        vChassisMove(0, 0, 0);
    }
    vRpiSendTaskCmd(RPI_TASK_AIM_STOP);

    /* ---- ⑥ 残留记账: 累计纵移折世界系（本步只沿排前后、不横移不转向, 航向查一次即可）----
     * 抓左/右靶那 ±150mm 落在 fwd 分量; side 分量恒 0（本步不动横移轴）。 */
    if (ucKeepRes != 0u)
    {
        float   fResWx;
        float   fResWy;
        char    acWy[12];

        vChassisBodyToWorld(0.0f, fTotalMm, &fResWx, &fResWy);
        vSetPathResidual(fResWx, fResWy);
        bt_fmt_f2(acDx, (int)sizeof(acDx), fResWx);
        bt_fmt_f2(acWy, (int)sizeof(acWy), fResWy);
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT res: wx=%smm wy=%smm", acDx, acWy);
        vRpiSendDebug(acDbg);
    }

    /* ---- ⑦ 收官数据行（口径同 TGTS fin/ok/fail, 单位写进标签）----
     * tot = 本步累计纵移[mm], tr = 精调段累计指令行程[mm]。
     * c= 是本步**最终**在用的相机（0=主摄/1=pi）: 一发打偏时先看它 —— 若与预期不符
     * （比如标的是 pi 那套瞄准列却回落到主摄 → 落点按主摄那套算）就一目了然。 */
    (void)snprintf(acDbg, sizeof(acDbg), "SHOT fin: tot=%dmm tr=%dmm r=%upx c=%u",
                   (int)fTotalMm, (int)fTravel, (unsigned)usLastR, (unsigned)ucCamNow);
    vRpiSendDebug(acDbg);

    /* ⚠️ 保持档**不走这条**: 它的"收敛"只是"现在稳住了"、**不是退出条件** —— 能走到这儿
     * 一定是被中止的（`ucAbortStop` 那条在下面）。不挡住的话, 操作员按「停止」会被报成
     * `SHOT ok` —— 把"按停"说成"成功", 日志误导。 */
    if ((ucOk != 0u) && (ucHold == 0u))
    {
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT ok: d=%dpx n=%lums",
                       (int)fDpx, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 1u;
    }
    if (ucAbortNoprog != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT fail: noprog d=%dpx tr=%dmm n=%lums",
                       (int)fDpx, (int)fTravel, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortTravel != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT fail: travel d=%dpx tr=%dmm trmax=%dmm n=%lums",
                       (int)fDpx, (int)fTravel, (int)fTravelMax, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortStop != 0u)
    {
        /* 操作员按的停: 不蜂鸣（按停是意图不是故障, 同 ucRpiTargetServo） */
        (void)snprintf(acDbg, sizeof(acDbg), "SHOT abort: stop by request n=%lums",
                       (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    (void)snprintf(acDbg, sizeof(acDbg), "SHOT fail: timeout d=%dpx tr=%dmm n=%lums",
                   (int)fDpx, (int)fTravel, (unsigned long)ulElapsed);
    vRpiSendDebug(acDbg);
    return 0u;
}

/* ==================== 爆爆桶对位（2026-10-24 加） ====================
 * STEP_BARREL_ALIGN_WAIT 的唯一原语（ucRpiBarrelAlign, 声明见 app_rpi.h）。
 * 闭环照抄 ucTargetShootCore 的**精调环**（④~⑦）, 刻意**没有**的三段:
 *   ① 二维码选目标 —— 桶没有色码; 选哪颗桶 = "OpenMV 挑好再报"（面积最大者在
 *      相机端已定, 用户 2026-10-24 定案）, 本端只认 0x0B 帧里那一颗;
 *   ② 粗调 —— 靠路线把车开到桶前（巡线/世界系平移积木）, 本步只收最后一段残差;
 *   ③ pi/主摄双瞄准尺 —— 数据源只有 OpenMV 一台（0x0B 无相机位）。
 * 新增的一段 = **只测不改模式**（bc_cx_aim=0 未标定）: 起流采样 bc_meas_ms,
 * 报选中桶 cx 的中位数（"BKT meas:" 行 + bucket_err 遥测）, 一格不动 —— 标定自举
 * （同 road_dx_ideal/tc_cx_aim 的"0=未标定拒绝动车"哨兵约定, 但给一条能**读到数**
 * 的路, 否则瞄准常数永远是 0）。手动填 bc_cx_aim 后再跑就是正常闭环。
 * ⚠️ **轴是纵移 v_y**（vChassisMoveCameraForward, cx 偏差驱动 —— 同平移射靶定案;
 *    首版就错在横移的那次教训见 ucTargetShootCore 函数头, 别再犯）。
 * ⚠️ 0x0B 帧 cx 与蓝牙遥测 bucket_err 都是 OpenMV **320×240** 口径（≠ 0x04/0x05）。
 * ⚠️ bc_db_mm 在本环的角色 = **最小出力门**（换算位移低于它不出力, 防原地蹭）,
 *    带内/收敛只由 bc_conv_px 判 —— 两个旋钮要配套（conv_px × px_to_mm ≥ db_mm,
 *    见 app_config.h 注释）, 配反了会"偏差在带外却永不出力"白烧超时。
 * ⚠️ 循环体最前面查中止位（铁律）; 位置模式平移自终止, 任何出口不发停车指令;
 *    速度模式出口显式发 0（指令持久）。
 * ⚠️ 起流自愈: Pi 侧 BUCKET_MAX_S(30s) 到点会让流自己退, 帧停涨超 2×bc_stale_ms
 *    限速重发 "J"（同射靶保持档那条必需的自愈, 没有它就是冻住永不自愈）。 */
#define BKT_TRACE_EVERY_TICKS   3u     /* 每几拍打一条过程串（ctrl=180ms 时 ~2Hz） */
#define BKT_NOPROG_TICKS        6u     /* 连续多少拍"偏差不下降"判环没起作用（同 AIM_*） */
#define BKT_NOPROG_EPS_PX       1.0f   /* 一拍改善量小于此不算"在下降"（≈检测噪声） */
#define BKT_SETTLE_POLL_MS      30u    /* 等停稳的采样间隔（流 ~13-15fps, 够密） */
#define BKT_SETTLE_FRAMES       3u     /* 连续几个新帧"没动"算停稳 */
#define BKT_SETTLE_PX           4.0f   /* 相邻新帧桶 cx 变化小于此值算"没动" [px] */
#define BKT_SETTLE_TIMEOUT_MS   800u   /* 停稳窗上限 [ms]（从第一帧算起; 到点照常往下走） */
#define BKT_REVIVE_MS           2000u  /* 起流自愈重发 "J" 的限速闸 */
#define BKT_MEAS_MAX_N          16     /* 只测模式最多采几帧算中位（2s × 13fps 远用不满） */

uint8_t ucRpiBarrelAlign(uint32_t ulTimeoutMs, uint8_t ucKeepRes)
{
    /* ---- 进门取一次标定量快照（同 ucTargetShootCore: 一次调用一条控制律）---- */
    float    fAimCol     = fGetBcCxAim();
    float    fAimRow     = fGetBcCyAim();
    uint8_t  ucSwap      = ucGetBcSwap();
    float    fPxToMm     = fGetBcPxToMm();
    float    fFwdSign    = fGetBcSign();
    float    fDbMm       = fGetBcDbMm();
    float    fConvPx     = fGetBcConvPx();
    int32_t  iConvTicks  = iGetBcConvTicks();
    float    fKp         = fGetBcKpScale();
    float    fStepMax    = fGetBcStepMm();
    float    fTravelMax  = fGetBcTravelMm();
    uint32_t ulCtrlMs    = (uint32_t)iGetBcCtrlMs();
    uint32_t ulStaleMs   = (uint32_t)iGetBcStaleMs();
    uint32_t ulMeasMs    = (uint32_t)iGetBcMeasMs();
    uint8_t  ucVelMode   = ucGetBcVelMode();
    int      iAccel      = iGetBcAccel();
    int      iSpeed      = iGetBcSpeed();
    uint8_t  ucMeasOnly;                 /* 只测不改模式（瞄准常数 0 = 未标定） */

    uint16_t usCx = 0u;
    uint16_t usCy = 0u;
    uint16_t usW  = 0u;
    uint16_t usH  = 0u;
    uint8_t  ucValid = 0u;
    uint32_t ulElapsed = 0u;
    uint32_t ulFrameSeen;
    TickType_t xFrameTick;
    int32_t  iConvCnt  = 0;
    float    fTotalMm  = 0.0f;    /* 本步累计纵移[mm]（keep_res 记账用, 带符号） */
    float    fTravel   = 0.0f;    /* 累计**指令**行程[mm]（行程闸口径） */
    float    fDpx      = 0.0f;    /* 本拍桶偏差[px]（swap=1 时 = cx−瞄准列） */
    float    fDmm      = 0.0f;    /* 本拍偏差换算[mm] */
    float    fMm;                 /* 本拍下发量（位置模式: mm） */
    int      iVy;                 /* 本拍下发量（速度模式: v_y rpm, 0=不出力） */
    float    fPrevAbs  = -1.0f;   /* 上一拍 |dpx|（noprog 判据） */
    uint32_t ulNoprog  = 0u;
    uint32_t ulTick    = 0u;
    uint8_t  ucOk        = 0u;
    uint8_t  ucAbortNoprog = 0u;
    uint8_t  ucAbortTravel = 0u;
    uint8_t  ucAbortStop   = 0u;
    TickType_t xReviveTick = 0u;  /* 上次重发 "J" 的时刻（自愈限速闸） */
    char     acDbg[RPI_DATA_MAX + 8u];

    ucMeasOnly = (((ucSwap != 0u) ? fAimCol : fAimRow) <= 0.0f) ? 1u : 0u;

    /* ---- ① 起流: 基线先记（帧计数从这一刻起涨才算"流真的来了"）; 起流帧等链路,
     * 失败时拒绝返回 —— 流根本没起来, **不发 "K"**（同 ucRpiTargetServo/ucTargetShootCore）。 */
    ulFrameSeen = ulGetBucketFrameCount();
    xFrameTick  = xTaskGetTickCount();

    if (ucRpiSendTaskCmdWait(RPI_TASK_BUCKET_START, 0u) == 0u)
    {
        return 0u;
    }

    /* ---- ② 等车真的停稳再进环（同 ④' 的"让测量可信"段, 桶版精简: 无换台/无选色）----
     * 全部耗时计入 ulElapsed（step 总预算口径不变）; 上限从**第一帧**算起;
     * 到点没判到停稳**照常往下走**（不判失败 —— 链路慢时宁可持续测量也别空烧）。 */
    {
        uint32_t ulT0     = 0u;
        uint32_t ulSeen   = ulGetBucketFrameCount();
        float    fPrevCx  = 0.0f;
        uint8_t  ucHave   = 0u;
        uint8_t  ucStable = 0u;
        uint8_t  ucOkSet  = 0u;
        uint8_t  ucBegin  = 0u;

        while (ulElapsed < ulTimeoutMs)
        {
            if (ucGetScriptAbortReq() != 0u) { break; }
            if ((ucBegin != 0u) && (ulT0 >= BKT_SETTLE_TIMEOUT_MS)) { break; }

            vTaskDelay(pdMS_TO_TICKS(BKT_SETTLE_POLL_MS));
            ulElapsed += BKT_SETTLE_POLL_MS;
            if (ucBegin != 0u) { ulT0 += BKT_SETTLE_POLL_MS; }

            if (ulGetBucketFrameCount() == ulSeen) { continue; }   /* 还没有新帧 */
            ulSeen  = ulGetBucketFrameCount();
            ucBegin = 1u;

            vGetBucketFrame(&ucValid, &usCx, &usCy, &usW, &usH);
            if (ucValid == 0u) { ucHave = 0u; ucStable = 0u; continue; }

            {
                float fCx    = (ucSwap != 0u) ? (float)usCx : (float)usCy;
                float fDelta = fCx - fPrevCx;

                if (fDelta < 0.0f) { fDelta = -fDelta; }
                if ((ucHave != 0u) && (fDelta < BKT_SETTLE_PX)) { ucStable++; }
                else                                            { ucStable = 0u; }
                fPrevCx = fCx;
                ucHave  = 1u;
            }

            if (ucStable >= BKT_SETTLE_FRAMES) { ucOkSet = 1u; break; }
        }

        (void)snprintf(acDbg, sizeof(acDbg), "BKT settle: t=%lums s=%u",
                       (unsigned long)ulT0, (unsigned)ucOkSet);
        vRpiSendDebug(acDbg);
    }

    /* ---- ③ 只测不改模式（未标定的标定自举）: 再采 ulMeasMs, 报 cx 中位数 ---- */
    if (ucMeasOnly != 0u)
    {
        uint16_t ausSamp[BKT_MEAS_MAX_N];
        uint8_t  ucN = 0u;
        uint8_t  ucI;
        uint8_t  ucJ;

        while ((ulElapsed < ulTimeoutMs) && (ulElapsed < ulMeasMs))
        {
            if (ucGetScriptAbortReq() != 0u) { break; }

            vTaskDelay(pdMS_TO_TICKS(BKT_SETTLE_POLL_MS));
            ulElapsed += BKT_SETTLE_POLL_MS;

            if (ulGetBucketFrameCount() == ulFrameSeen) { continue; }
            ulFrameSeen = ulGetBucketFrameCount();

            vGetBucketFrame(&ucValid, &usCx, &usCy, &usW, &usH);
            if (ucValid == 0u) { continue; }
            vSetBucketErr((ucSwap != 0u) ? ((float)usCx - fAimCol)
                                         : ((float)usCy - fAimRow));
            if (ucN < (uint8_t)BKT_MEAS_MAX_N)
            {
                ausSamp[ucN] = ((ucSwap != 0u) ? usCx : usCy);
                ucN++;
            }
        }

        /* 插入排序取中位（n≤16, 开销可忽略; 偶数个取上中位 —— 与 Pi 侧 _med 同款） */
        for (ucI = 1u; ucI < ucN; ucI++)
        {
            uint16_t usKey = ausSamp[ucI];

            for (ucJ = ucI; (ucJ > 0u) && (ausSamp[ucJ - 1u] > usKey); ucJ--)
            {
                ausSamp[ucJ] = ausSamp[ucJ - 1u];
            }
            ausSamp[ucJ] = usKey;
        }
        {
            uint16_t usMed = (ucN > 0u) ? ausSamp[ucN / 2u] : 0u;

            (void)snprintf(acDbg, sizeof(acDbg),
                           "BKT meas: cx=%u cy=%u w=%u h=%u n=%u  <-- 填进 bc_cx_aim",
                           (unsigned)((ucSwap != 0u) ? usMed : 0u),
                           (unsigned)((ucSwap == 0u) ? usMed : 0u),
                           (unsigned)usW, (unsigned)usH, (unsigned)ucN);
        }
        vRpiSendDebug(acDbg);
        vRpiSendTaskCmd(RPI_TASK_BUCKET_STOP);   /* 唯一出口纪律: 无论成败都停流 */
        (void)snprintf(acDbg, sizeof(acDbg), "BKT meas: done (aim not calibrated)");
        vRpiSendDebug(acDbg);
        return 0u;
    }

    /* 停稳窗跨了好几个 tick, 重记一次新鲜度基线（同 ucTargetShootCore 那条注释） */
    ulFrameSeen = ulGetBucketFrameCount();
    xFrameTick  = xTaskGetTickCount();

    /* ---- ④ 精调闭环（同 ucTargetShootCore 精调环的桶版）---- */
    while (ulElapsed < ulTimeoutMs)
    {
        /* 中止位: 必须在最前（Stale 分支会 continue, 放后面链路一断就看不见） */
        if (ucGetScriptAbortReq() != 0u) { ucAbortStop = 1u; break; }

        /* 节流: 位置模式平移阻塞自带停稳, 这里只负责测量间隔 */
        vTaskDelay(pdMS_TO_TICKS(ulCtrlMs));
        ulElapsed += ulCtrlMs;

        /* 新鲜度: 帧计数停涨 = Pi 掉线/停发 → 冻结（不动车、不判收敛）, 回来即续跑;
         * 超 2×stale 且过了限速闸 → 重发 "J" 自愈（Pi 侧 BUCKET_MAX_S 会自己退流） */
        if (ulGetBucketFrameCount() != ulFrameSeen)
        {
            ulFrameSeen = ulGetBucketFrameCount();
            xFrameTick  = xTaskGetTickCount();
        }
        else if ((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(ulStaleMs))
        {
            iConvCnt = 0;
            if (((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(2u * ulStaleMs))
                && ((xTaskGetTickCount() - xReviveTick) > pdMS_TO_TICKS(BKT_REVIVE_MS)))
            {
                xReviveTick = xTaskGetTickCount();
                vRpiSendTaskCmd(RPI_TASK_BUCKET_START);
                vRpiSendDebug("BKT: stream stale, re-arm");
            }
            continue;
        }

        vGetBucketFrame(&ucValid, &usCx, &usCy, &usW, &usH);
        if (ucValid == 0u)
        {
            iConvCnt = 0;
            continue;                              /* 流活着, 本拍没找到桶 */
        }

        fDpx = (ucSwap != 0u) ? ((float)usCx - fAimCol)
                              : ((float)usCy - fAimRow);
        fDmm = fDpx * fPxToMm * fFwdSign;

        vSetBucketErr(fDpx);     /* 遥测: 标定/排障在蓝牙 Tab 直接看, 不必翻日志 */

        ulTick++;
        if ((ulTick == 1u) || ((ulTick % BKT_TRACE_EVERY_TICKS) == 0u))
        {
            (void)snprintf(acDbg, sizeof(acDbg), "BKT n=%lu d=%dpx mm=%dmm tr=%dmm tot=%dmm",
                           (unsigned long)ulTick, (int)fDpx, (int)fDmm,
                           (int)fTravel, (int)fTotalMm);
            vRpiSendDebug(acDbg);
        }

        /* 收敛判据: |dpx| 连续 N 拍进带（**排在 noprog 之前**, 同 ucTargetShootCore 的
         * 现场教训: d 在噪声带内抖动时 noprog 会把"在带上稳住"误杀成"没进展"） */
        if ((fDpx > -fConvPx) && (fDpx < fConvPx))
        {
            iConvCnt++;
            if (iConvCnt >= iConvTicks)
            {
                ucOk = 1u;
                break;
            }
            if (ucVelMode != 0u) { vChassisMove(0, 0, 0); }
            continue;                       /* 在带上: 这一拍不推车、不数 noprog */
        }
        iConvCnt = 0;

        /* 偏差 → 输出（双模式, 同 ucTargetShootCore）:
         * · 位置模式: 每拍走 fMm = 偏差mm × 增益, 钳 ±bc_step_mm, 阻塞到位;
         * · 速度模式: 每拍发 v_y = fMm ÷ 5.079 的 rpm 连续修, 钳 ±bc_speed;
         *   不足 1rpm 的零速带由顶格逻辑兜住（发最小 1rpm）。 */
        iVy = 0;
        fMm = fDmm * fKp;
        if (ucVelMode != 0u)
        {
            iVy = (int)(fMm / 5.079f);
            if (iVy == 0)
            {
                iVy = (fMm >= 0.0f) ? 1 : -1;   /* 零速带顶格（同 2026-10-07 那条） */
            }
            if (iVy >  iSpeed) { iVy =  iSpeed; }
            if (iVy < -iSpeed) { iVy = -iSpeed; }
        }
        else
        {
            if (fMm >  fStepMax) { fMm =  fStepMax; }
            if (fMm < -fStepMax) { fMm = -fStepMax; }
        }

        /* 最小出力门（bc_db_mm）: 换算位移低于它不出力（防原地蹭）。⚠️ 与收敛阈值要
         * 配套（conv_px×px_to_mm ≥ db_mm, 见函数头）—— 配反了会"带外却永不出力"。
         * 速度模式不出力时顶格逻辑已经兜过, 本门只作用于位置模式。 */
        if ((ucVelMode == 0u) && (fMm > -fDbMm) && (fMm < fDbMm))
        {
            fMm = 0.0f;
        }

        /* 行程闸（先累加后判定, 超限拍不下发; 速度模式按指令速度×节拍折 mm） */
        if (ucVelMode != 0u)
        {
            fTravel += (((iVy >= 0) ? (float)iVy : -(float)iVy) * 5.079f
                        * ((float)ulCtrlMs / 1000.0f));
        }
        else
        {
            fTravel += (fMm >= 0.0f) ? fMm : -fMm;
        }
        if (fTravel > fTravelMax)
        {
            ucAbortTravel = 1u;
            break;
        }

        /* noprog 早退（只看死区外且本拍真出过力的拍）: |dpx| 连续不下降 = 环没起作用 */
        {
            uint8_t ucMoved = (ucVelMode != 0u) ? ((iVy != 0) ? 1u : 0u)
                                                : ((fMm != 0.0f) ? 1u : 0u);

            if (ucMoved != 0u)
            {
                float fAbs = (fDpx >= 0.0f) ? fDpx : -fDpx;

                if ((fPrevAbs >= 0.0f) && (fAbs > (fPrevAbs - BKT_NOPROG_EPS_PX)))
                {
                    ulNoprog++;
                }
                else
                {
                    ulNoprog = 0u;
                }
                fPrevAbs = fAbs;
                if (ulNoprog >= BKT_NOPROG_TICKS)
                {
                    ucAbortNoprog = 1u;
                    break;
                }
            }
        }

        /* 下发（速度模式 v_y>0=前进, **持久指令**; 位置模式阻塞到位自终止） */
        if (ucVelMode != 0u)
        {
            if (iVy != 0)
            {
                vChassisMove(0, iVy, 0);        /* 纵移（速度模式, 连续） */
                fTotalMm += ((float)iVy * 5.079f * ((float)ulCtrlMs / 1000.0f));
            }
        }
        else
        {
            if (fMm != 0.0f)
            {
                vChassisMoveCameraForward(fMm, iAccel, iSpeed); /* 纵移（位置模式阻塞到位） */
                fTotalMm += fMm;
            }
        }
    }

    /* ---- ⑤ 停流: 唯一出口（少了它 Pi 会一直转发、相机一直发 B 帧）----
     * 速度模式必须**显式停车**（指令持久, 不发 0 会带着末速一直爬）。 */
    if (ucVelMode != 0u)
    {
        vChassisMove(0, 0, 0);
    }
    vRpiSendTaskCmd(RPI_TASK_BUCKET_STOP);

    /* ---- ⑥ 残留记账: 累计纵移折世界系（本步只前后、不横移不转向, 航向查一次即可）---- */
    if ((ucKeepRes != 0u) && (ucOk != 0u))
    {
        float   fResWx;
        float   fResWy;
        char    acDx[12];

        vChassisBodyToWorld(0.0f, fTotalMm, &fResWx, &fResWy);
        vSetPathResidual(fResWx, fResWy);
        bt_fmt_f2(acDx, (int)sizeof(acDx), fResWx);
        (void)snprintf(acDbg, sizeof(acDbg), "BKT res: wx=%smm", acDx);
        vRpiSendDebug(acDbg);
    }

    /* ---- ⑦ 收官数据行（口径同 SHOT fin/ok/fail）---- */
    (void)snprintf(acDbg, sizeof(acDbg), "BKT fin: tot=%dmm tr=%dmm",
                   (int)fTotalMm, (int)fTravel);
    vRpiSendDebug(acDbg);

    if (ucOk != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "BKT ok: d=%dpx n=%lums",
                       (int)fDpx, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 1u;
    }
    if (ucAbortNoprog != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "BKT fail: noprog d=%dpx tr=%dmm n=%lums",
                       (int)fDpx, (int)fTravel, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortTravel != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "BKT fail: travel d=%dpx tr=%dmm trmax=%dmm n=%lums",
                       (int)fDpx, (int)fTravel, (int)fTravelMax, (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }
    if (ucAbortStop != 0u)
    {
        (void)snprintf(acDbg, sizeof(acDbg), "BKT abort: stop by request n=%lums",
                       (unsigned long)ulElapsed);
        vRpiSendDebug(acDbg);
        return 0u;
    }

    (void)snprintf(acDbg, sizeof(acDbg), "BKT fail: timeout d=%dpx tr=%dmm n=%lums",
                   (int)fDpx, (int)fTravel, (unsigned long)ulElapsed);
    vRpiSendDebug(acDbg);
    return 0u;
}

/* ==================== 接收任务 ==================== */

/* Pi 参数缓存（"P" 命令族）的两个内部函数: 实现放在**文件末尾**的「Pi 参数缓存」段
 * —— 那段要按生成块里的 g_astRpiCfgParam 表长开缓存数组（sizeof）, 而生成块在末尾。 */
static void vRpiCfgPoll(void);
static void ucRpiCfgHandleReply(const uint8_t *pucData, uint8_t ucDataLen);
/* 「自动下发二维码」的 QS 回执解析（实现也在文件末尾; 排在 ucRpiCfgHandleReply 之前拦） */
static uint8_t ucRpiQrAutoReply(const uint8_t *pucData, uint8_t ucDataLen);

/* Pi 端握手请求 DATA（serial_comm.send 补零到 9B, 本端按前 9 字节匹配） */
static const uint8_t s_aucSyncHs1[9] = { 'S','Y','N','C','_','H','S','_','1' };

/* 接收暂存: 跨 take 保留「已收到但还没凑成整帧」的残帧。
 * 取圆环一圈大小, 保证任何一次 take 都能整段放下, 残帧挤不掉新数据
 * （帧上限 = RPI_FRAME_OVERHEAD + RPI_DATA_MAX = 69B, 余量充足）。 */
#define RPI_RX_STAGE_SIZE   UART_DMA_BUF_SIZE

/*
 * UART_RPI 唯一 RX 消费者: 阻塞等新字节（100ms 超时醒来时检查心跳超时）→ 追加进暂存
 * → 从暂存头逐帧扫描, 凑满整帧校验后按 TYPE 分发:
 *   COORD "SYNC_HS_1"(9B)  → 自动回 ACK 0xCC + 置链路 OK + vSetHandshakeSeen
 *   COORD 巡线帧(8B, TAG=0x01) → vSetLineData 快照（30Hz 流, 不进 last packet）
 *   COORD 人质帧(≥3B, TAG=0x02) → vSetHostageResult 快照（同上, 不进 last packet）
 *   COORD 色球帧(≥3B, TAG=0x03) → vSetColorResult 快照（同上, 不进 last packet）
 *   COORD 靶标帧(≥3B, TAG=0x04) → vSetTargetResult + vSetTargetReady（一次性, 不进 last packet）
 *   COORD 靶标流帧(≥3B, TAG=0x05) → vSetAimFrame（~20Hz 连续流, 无 ready、只推帧计数, 不进 last packet）
 *   COORD 拍照结果(2B, TAG=0x06) → vSetPhotoResult + vSetPhotoDoneTick（命令 "9" 的回应, 不进 last packet）
 *   COORD 调试二维码(9B, ASCII "QP<ddd>") → vSetQrCode + vSetQrManual（Web 自动下发的预设, OLED 显示 QR:xxx*; 不进 last packet）
 *   COORD 其他(≥9B)       → vSetLastRpiPacket + vSetRpiFrameReady（坐标/QR 结果）
 *   HELLO(0B)             → Pi 空闲心跳帧, 仅刷新存活时刻
 *   ACK / 其他类型         → 丢弃（结果帧不要求应答, ACK 仅握手期有意义）
 *
 * 为什么要扫描而不是「一次 take 正好一帧」: 接收层交付的是**本次新增字节**
 * （见 app_uart.h 的契约），帧可能跨 take 被切开（圆环绕回处必发生）、
 * 也可能几帧粘在一次 take 里。所以这里按字节流解析: 帧头/长度/EOF/CRC 任一不过
 * 就丢 1 字节重同步, 帧不全就把残帧留到下一次 —— 本端无重传, 全靠这套自愈。
 */
void vRpiTask(void *argument)
{
    static uint8_t aucRx[RPI_RX_STAGE_SIZE];    /* 残帧暂存（任务级静态, 不占 256 字任务栈） */
    uint16_t usFill = 0;                        /* 暂存内有效字节数 */

    (void)argument;

    for (;;)
    {
        uint16_t usPos = 0;                     /* 本轮在 aucRx 里的解析游标 */
        uint16_t usGot;
        uint8_t ucType;
        uint8_t ucDataLen;
        const uint8_t *pucData;

        /* 0. Pi 参数轮询（OLED 调参屏在显示某个 Pi 参数时才有事做; 平时一个字节都不发）。
         *    ⚠️ 放在阻塞收字节**之前** —— 每轮（最短 100ms）都有机会推进,
         *       而不是"只有 Pi 发帧过来才轮询"（那会让空闲时永远取不到值）。 */
        vRpiCfgPoll();

        /* 1. 追加本次新增字节（残帧已在缓冲头, 直接接在后面） */
        usGot = usUartDmaTakeCopy(UART_RPI, &aucRx[usFill],
                                  (uint16_t)(RPI_RX_STAGE_SIZE - usFill),
                                  pdMS_TO_TICKS(100));

        if (usGot == 0u)
        {
            /* 100ms 没等到新字节: 检查心跳超时（仅 OK 态; Pi 心跳 2s, 阈值 > 2×）。
             * 暂存里的残帧原样留着（下面 continue 不动 usFill）——它可能是 Pi 掉线时
             * 切了一半的帧, 所以这个检查不能加「暂存为空」的前置条件, 否则永远等不到。
             * 同理 continue 不再往下走: 残帧不成整帧, 解析也只会立刻 break。 */
            if (emGetRpiLinkState() == RPI_LINK_OK &&
                (xTaskGetTickCount() - s_xLastRpiFrameTick)
                    > pdMS_TO_TICKS(RPI_DISCONNECT_TIMEOUT_MS))
            {
                vSetRpiLinkState(RPI_LINK_DISCONNECTED);
            }

            /* 兜底自愈: 已判掉线 = 本该每 2s 来一次心跳却 8s 无帧。有两种可能:
             *   ① Pi 真掉线/没上电（重装 DMA 无害, 等着就行）;
             *   ② 本端 RX DMA 被接收错误停掉（HAL 的阻塞性错误处理, 见 app_uart.c 的
             *      HAL_UART_ErrorCallback 注释）—— 那种情况下 TX 还是好的, 对端看不出异常,
             *      不重装就**永远是聋的**。
             * 每 5s 最多重装一次, 健康链路上永不触发（心跳 2s < 8s）。 */
            if ((emGetRpiLinkState() == RPI_LINK_DISCONNECTED)
                && ((xTaskGetTickCount() - s_xRxRearmTick) > pdMS_TO_TICKS(5000)))
            {
                s_xRxRearmTick = xTaskGetTickCount();
                vUartReArmRx(UART_RPI);
            }
            continue;
        }
        usFill = (uint16_t)(usFill + usGot);

        /* 2. 逐帧消费暂存内容 */
        while (usPos < usFill)
        {
            if (aucRx[usPos] != RPI_SOF)                { usPos++; continue; }
            if ((uint16_t)(usFill - usPos) < RPI_FRAME_OVERHEAD) { break; }  /* 头不全: 留着 */

            ucDataLen = aucRx[usPos + 2];
            /* 长度非法 = 这个 SOF 是数据里的巧合, 跳过它继续找 */
            if (ucDataLen > RPI_DATA_MAX)               { usPos++; continue; }
            /* 先判长度再看 EOF/CRC: 保证下面的索引都在 [usPos, usFill) 内 */
            if ((uint16_t)(usFill - usPos)
                < (uint16_t)(ucDataLen + RPI_FRAME_OVERHEAD)) { break; }      /* 帧不全: 留着 */
            if (aucRx[usPos + 4 + ucDataLen] != RPI_EOF) { usPos++; continue; }
            if (ucEmmV5Crc8(&aucRx[usPos + 1], (uint8_t)(2 + ucDataLen))
                != aucRx[usPos + 3 + ucDataLen])        { usPos++; continue; }

            ucType  = aucRx[usPos + 1];
            pucData = &aucRx[usPos + 3];

            /* 校验通过的有效帧: 刷新链路存活时刻（握手/心跳/坐标/巡线都算）。
             * Disconnected 态收到任何有效帧 = 链路恢复（Pi 忙完/重连后发心跳）, 回 OK */
            s_xLastRpiFrameTick = xTaskGetTickCount();
            if (emGetRpiLinkState() == RPI_LINK_DISCONNECTED) {
                vSetRpiLinkState(RPI_LINK_OK);
            }

            switch (ucType)
            {
            case RPI_TYPE_COORD:
                /* 诊断行（2026-09 加）: 大帧（色球 21B / 人质 18B / 靶标 24B）到底有没有
                 * 到达并校验通过。排查「Pi 说识别成功、单片机一直超时」时靠它一眼分辨:
                 *   · **没有这行** = 字节根本没进来 → 查链路/RX（见 app_uart.c 的 RX 恢复注释）;
                 *   · **有这行但下游还超时** = 帧进来了, 问题在解析/消费侧。
                 * 小帧（9B 心跳/握手/坐标）不打印, 免得刷屏; 大帧本身低频, 开销可忽略。
                 *
                 * ⚠️ **靶标流帧（TAG=0x05）必须排除在外**（2026-09 加）: 它是 ~20Hz 的
                 *    连续流, count=2/3 时 LEN=17/24, 全都落进这条 >=15 的判据里 ——
                 *    不排除的话每帧一条 [STM-DBG], **RPI 链路流量翻倍 + 刷爆 Pi 侧日志**
                 *    （Pi 那边自己也只能按 1Hz 摘录, 见 CLAUDE.md）。
                 *    ⚠️ 原始理由「vRpiSendDebug 走的是忙等的 HAL_UART_Transmit、会拖垮
                 *      vPidTask」在 2026-09 发送改 TX DMA 环后**已不成立** —— 但它本来
                 *      就该排除（那是流量/日志问题, 不是 CPU 问题）, **别据此删掉这一行**。 */
                /* ⚠️ 短帧要显式列进来（12B 的看路帧 < 15B, 不加就完全看不见 —— 台架排障靠这行） */
                if ((ucDataLen >= 15u || pucData[0] == RPI_RECT_TAG
                     || pucData[0] == RPI_ROAD_TAG || pucData[0] == RPI_ROADYAW_TAG
                     || pucData[0] == RPI_CFG_TAG || pucData[0] == RPI_HEALTH_TAG)
                    && pucData[0] != RPI_AIM_TAG)
                {
                    char acDbg[32];
                    (void)snprintf(acDbg, sizeof(acDbg), "COORD tag=%u len=%u",
                                   (unsigned)pucData[0], (unsigned)ucDataLen);
                    vRpiSendDebug(acDbg);
                }
                if (ucDataLen == 9 && memcmp(pucData, s_aucSyncHs1, 9) == 0)
                {
                    /* 握手请求: 应答（无论脚本是否在等, 保证 Pi 随时能握上）
                     * 并置链路 OK —— Pi 断开重连重新握手也从这里恢复 OK */
                    uint8_t ucAck = RPI_ACK_HANDSHAKE;
                    vRpiSendFrame(RPI_TYPE_ACK, &ucAck, 1);
                    vSetRpiLinkState(RPI_LINK_OK);
                    vSetHandshakeSeen();
                }
                else if (ucDataLen == RPI_BEEP_LEN && pucData[0] == RPI_BEEP_TAG)
                {
                    /* 蜂鸣命令（Pi → 单片机, 2026-10-27 加, 见 app_rpi.h TAG 的说明）:
                     * **只投请求**（vSetBuzzerReq → 蜂鸣任务响）—— vRpiTask 禁阻塞,
                     * 失败音整段 2.1s 只能在 vBuzzerTask 里慢慢响。
                     * 0/未知 pattern = 忽略（前向兼容: 新固件加音型, 旧固件不误响）。 */
                    if ((pucData[1] == BUZZER_PAT_FAIL) || (pucData[1] == BUZZER_PAT_TICK))
                    {
                        vSetBuzzerReq(pucData[1]);
                    }
                }
                else if (ucDataLen == RPI_HEALTH_LEN && pucData[0] == RPI_HEALTH_TAG)
                {
                    /* 相机健康检查结果（命令 "Z" 的回应, 2026-11 加）:
                     * [TAG=0x0D][M][Q][Pi][MV], 每项 1=通过 0=失败。
                     * ⚠️ 写序: 先结果、再清 busy、最后置 ready —— 按 ready 门消费的
                     *    ucRpiHealthCheck 拿到的必是同一帧的四项。
                     * ⚠️ 5B 天然落不进下面的 `ucDataLen >= 9` 通用分支, 但按惯例仍排在
                     *    它之前（与 0x07/0x08/0x09/0x0B 同一条纪律）。 */
                    vSetHealthResult(pucData[1], pucData[2], pucData[3], pucData[4]);
                    vSetHealthBusy(0u);
                    vSetHealthReady();
                }
                else if (ucDataLen == RPI_PHOTO_LEN && pucData[0] == RPI_PHOTO_TAG)
                {
                    /* 拍照结果帧（命令 "9" 的回应, 见 app_rpi.h 该 TAG 的注释）:
                     * 只做一件事 —— 置结果 + 记时刻, 由 OLED 任务闪一句「拍照 成功!/失败!」。
                     * ⚠️ 不做 ready/一次性信号: 它不是"等结果"的同步点, 纯粹是播报;
                     *    做成信号会被某个读者偷走（同 IF 条件那条纪律）。 */
                    vSetPhotoResult(pucData[1]);
                    vSetPhotoDoneTick(xTaskGetTickCount());
                }
                else if (ucDataLen == RPI_LINE_LEN && pucData[0] == RPI_LINE_TAG)
                {
                    /* 巡线帧: i16 LE 定点 → 物理量, 整帧快照进 Beam */
                    int16_t nErr   = (int16_t)((uint16_t)pucData[1]
                                             | ((uint16_t)pucData[2] << 8));
                    int16_t nSteer = (int16_t)((uint16_t)pucData[3]
                                             | ((uint16_t)pucData[4] << 8));
                    int16_t nAng   = (int16_t)((uint16_t)pucData[5]
                                             | ((uint16_t)pucData[6] << 8));
                    vSetLineData(nErr / 10000.0f, nSteer / 1000.0f,
                                 nAng / 10.0f, pucData[7]);
                }
                else if (ucDataLen >= RPI_HOSTAGE_HDR_LEN && pucData[0] == RPI_HOSTAGE_TAG)
                {
                    /* 人质识别帧: [TAG][status][count] + count×[shape][cx u16 LE][cy u16 LE]
                     * 整帧快照进 Beam（同巡线帧做法, 不进 last packet）。 */
                    uint8_t ucCnt = pucData[2];
                    uint8_t ucI;
                    stHostageTgtTdf astT[HOSTAGE_MAX_COUNT];

                    if (ucCnt > RPI_HOSTAGE_MAX_CNT) { ucCnt = RPI_HOSTAGE_MAX_CNT; }
                    if ((uint16_t)(RPI_HOSTAGE_HDR_LEN + RPI_HOSTAGE_TGT_LEN * ucCnt)
                        > ucDataLen)
                    {
                        ucCnt = 0;                  /* 长度与 count 不符: 丢弃目标只留状态 */
                    }
                    for (ucI = 0; ucI < ucCnt; ucI++)
                    {
                        const uint8_t *p = &pucData[RPI_HOSTAGE_HDR_LEN
                                                    + RPI_HOSTAGE_TGT_LEN * ucI];
                        astT[ucI].ucShape = p[0];
                        astT[ucI].usCx = (uint16_t)((uint16_t)p[1]
                                                  | ((uint16_t)p[2] << 8));
                        astT[ucI].usCy = (uint16_t)((uint16_t)p[3]
                                                  | ((uint16_t)p[4] << 8));
                    }
                    vSetHostageResult(pucData[1], ucCnt, astT);
                    vSetHostageReady();
                }
                else if (ucDataLen >= RPI_COLOR_HDR_LEN && pucData[0] == RPI_COLOR_TAG)
                {
                    /* 色球识别帧: [TAG][status][count] + count×[color][cx u16 LE][cy u16 LE]
                     * 整帧快照进 Beam（同巡线/人质帧做法, 不进 last packet）。 */
                    uint8_t ucCnt = pucData[2];
                    uint8_t ucI;
                    stColorTgtTdf astT[COLOR_MAX_COUNT];

                    if (ucCnt > RPI_COLOR_MAX_CNT) { ucCnt = RPI_COLOR_MAX_CNT; }
                    if ((uint16_t)(RPI_COLOR_HDR_LEN + RPI_COLOR_TGT_LEN * ucCnt)
                        > ucDataLen)
                    {
                        ucCnt = 0;                  /* 长度与 count 不符: 丢弃目标只留状态 */
                    }
                    for (ucI = 0; ucI < ucCnt; ucI++)
                    {
                        const uint8_t *p = &pucData[RPI_COLOR_HDR_LEN
                                                    + RPI_COLOR_TGT_LEN * ucI];
                        astT[ucI].ucColor = p[0];
                        astT[ucI].usCx = (uint16_t)((uint16_t)p[1]
                                                  | ((uint16_t)p[2] << 8));
                        astT[ucI].usCy = (uint16_t)((uint16_t)p[3]
                                                  | ((uint16_t)p[4] << 8));
                    }
                    /* 扩展字段（小球对位, 2026-09）: [yaw i16 LE 0.01°][fit_ok u8],
                     * 仅 Pi 新版成功帧携带（LEN=21）。写序: 先写这两个再写快照、
                     * 最后 vSetColorReady —— 按 ready 门消费的一方拿到的必是同一帧的值。
                     * 旧 Pi（LEN=18）/ 失败帧（LEN=3）→ 兜底 0/合格, 消费方只在
                     * status==1 且 count==3 时才信 yaw。 */
                    if (ucDataLen >= RPI_COLOR_EXT_LEN)
                    {
                        int16_t nYaw = (int16_t)((uint16_t)pucData[RPI_COLOR_EXT_OFF]
                                               | ((uint16_t)pucData[RPI_COLOR_EXT_OFF + 1u] << 8));
                        vSetBallYawLine(nYaw / 100.0f);
                        vSetBallFitOk(pucData[RPI_COLOR_EXT_OFF + 2u]);
                    }
                    else
                    {
                        vSetBallYawLine(0.0f);
                        vSetBallFitOk(1u);
                    }
                    /* 失败帧方向字节（2026-10-26 加, ALIGN_PULLBACK 用）:
                     * [TAG][00][00][dir] LEN=4 —— 0=无信息 1=缺球从画面左出界 2=右。
                     * 成功帧不带 → 0; 旧 Pi 失败帧 LEN=3 → ucDataLen 判住同样 0。
                     * 旧固件 + 新 Pi: LEN=4 走上面同一条路径, 第 4 字节被本判据
                     * (status==0 才读)之外无视, 向后兼容。写序: 先这里再 vSetColorResult。 */
                    vSetBallPullDir(((ucDataLen > RPI_COLOR_FAIL_DIR_OFF)
                                     && (pucData[1] == 0u))
                                    ? pucData[RPI_COLOR_FAIL_DIR_OFF] : 0u);
                    vSetColorResult(pucData[1], ucCnt, astT);
                    vSetColorReady();
                }
                else if (ucDataLen >= RPI_TARGET_HDR_LEN && pucData[0] == RPI_TARGET_TAG)
                {
                    /* 靶标识别帧: [TAG][status][count]
                     *              + count×[color][cx u16 LE][cy u16 LE][r u16 LE]
                     * 整帧快照进 Beam（同巡线/人质/色球帧做法, 不进 last packet）。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前** ——
                     *    否则 3+7×count（最长 24B）的整帧会被当「9 字节 last packet」
                     *    吞掉, 永远到不了这里。 */
                    uint8_t ucCnt = pucData[2];
                    uint8_t ucI;
                    stTargetTgtTdf astT[TARGET_MAX_COUNT];

                    if (ucCnt > RPI_TARGET_MAX_CNT) { ucCnt = RPI_TARGET_MAX_CNT; }
                    if ((uint16_t)(RPI_TARGET_HDR_LEN + RPI_TARGET_TGT_LEN * ucCnt)
                        > ucDataLen)
                    {
                        ucCnt = 0;                  /* 长度与 count 不符: 丢弃目标只留状态 */
                    }
                    for (ucI = 0; ucI < ucCnt; ucI++)
                    {
                        const uint8_t *p = &pucData[RPI_TARGET_HDR_LEN
                                                    + RPI_TARGET_TGT_LEN * ucI];
                        astT[ucI].ucColor = p[0];
                        astT[ucI].usCx = (uint16_t)((uint16_t)p[1]
                                                  | ((uint16_t)p[2] << 8));
                        astT[ucI].usCy = (uint16_t)((uint16_t)p[3]
                                                  | ((uint16_t)p[4] << 8));
                        astT[ucI].usR  = (uint16_t)((uint16_t)p[5]
                                                  | ((uint16_t)p[6] << 8));
                    }
                    /* ⚠️ status 是**位域**（见 app_rpi.h 的 RPI_TGT_ST_*）: bit0 才是成功位,
                     *    bit7 = 本次测量来自 pi(CSI) 摄像头。这里摘开、各归各位 ——
                     *    于是本文件里三处 `status == 1u` 的消费方**一个字节都不用改**。
                     *    写序: cam 必须在 vSetTargetReady() **之前**, 按 ready 门消费的
                     *    一方才会拿到同一帧的 cam（同色球那对扩展字段的纪律）。 */
                    /* 失败帧方向字节（2026-10-24 加, 射靶"目标色出画面"补步用）:
                     * [TAG][status=0][count=0][dir] LEN=4 —— 0=无信息 1=目标色在画面右侧
                     * 出界 2=左侧。成功帧不带; 旧 Pi 失败帧 LEN=3 → 长度判住同样 0。
                     * 旧固件 + 新 Pi: LEN=4 走同一分支, status!=0 时被下面判据无视 —— 兼容。
                     * 写序: 先这里再 vSetTargetResult（同 BallPullDir 的纪律）。 */
                    vSetTargetPullDir(((ucDataLen > RPI_TGT_FAIL_DIR_OFF)
                                       && ((pucData[1] & RPI_TGT_ST_OK) == 0u))
                                      ? pucData[RPI_TGT_FAIL_DIR_OFF] : 0u);
                    vSetTargetResult((uint8_t)(pucData[1] & RPI_TGT_ST_OK), ucCnt, astT);
                    vSetTargetCam((uint8_t)((pucData[1] & RPI_TGT_ST_CAM_PI) ? 1u : 0u));
                    vSetTargetReady();
                }
                else if (ucDataLen >= RPI_AIM_HDR_LEN && pucData[0] == RPI_AIM_TAG)
                {
                    /* 靶标流帧: [TAG][status][count] + count×[color][cx][cy][r]
                     * **与 0x04 帧逐字节同布局**, 差别只在它是 ~20Hz 连续流。
                     * 走**独立快照 + 帧计数**（vSetAimFrame, 同巡线帧）:
                     *   · 不置 ready（流的语义是"帧计数还在不在涨", 不是"有结果到达"）;
                     *   · 不进 last packet。
                     * count=0（丢目标）照常推进帧计数 —— 消费方靠这个区分
                     * 「流活着但没找到」与「流停了」。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前** ——
                     *    否则 17/24B 的流帧会被当「9 字节 last packet」吞掉, 永远到不了这里。
                     *    （与 0x04 分支同一条纪律, 见上面那段注释。） */
                    uint8_t ucCnt = pucData[2];
                    uint8_t ucI;
                    stTargetTgtTdf astT[TARGET_MAX_COUNT];

                    if (ucCnt > RPI_AIM_MAX_CNT) { ucCnt = RPI_AIM_MAX_CNT; }
                    if ((uint16_t)(RPI_AIM_HDR_LEN + RPI_AIM_TGT_LEN * ucCnt)
                        > ucDataLen)
                    {
                        ucCnt = 0;                  /* 长度与 count 不符: 丢弃本帧目标, 但帧计数照推(见下) */
                    }
                    for (ucI = 0; ucI < ucCnt; ucI++)
                    {
                        const uint8_t *p = &pucData[RPI_AIM_HDR_LEN
                                                    + RPI_AIM_TGT_LEN * ucI];
                        astT[ucI].ucColor = p[0];
                        astT[ucI].usCx = (uint16_t)((uint16_t)p[1]
                                                  | ((uint16_t)p[2] << 8));
                        astT[ucI].usCy = (uint16_t)((uint16_t)p[3]
                                                  | ((uint16_t)p[4] << 8));
                        astT[ucI].usR  = (uint16_t)((uint16_t)p[5]
                                                  | ((uint16_t)p[6] << 8));
                    }
                    /* ⚠️ 长度与 count 不符时仍然**推帧计数**（count 归 0）: 帧计数在这里的
                     *    语义是"Pi 还在发"（新鲜度）, 不是"这一帧好不好"。若校验失败就不推,
                     *    闭环那边看到的是"帧计数停涨"→ 判成 Pi 掉线而冻结, 把一个
                     *    格式错误伪装成链路故障（两个完全不同的病因）。 */
                    /* status 的位域与 0x04 **同一套**: bit7 = 本次来自 pi(CSI)。
                     * 流帧本来就每帧带 status（今天恒 1）, 所以这条路连字节数都没变。
                     * ⚠️ cam 跟着快照走、**每拍**读: Pi 可能中途从 CSI 退回主摄,
                     *    静止不动的瞄准列会静默跑偏（见 vSetAimFrame 的头注释）。 */
                    vSetAimFrame(ucCnt, astT,
                                 (uint8_t)((pucData[1] & RPI_TGT_ST_CAM_PI) ? 1u : 0u));
                }
                else if (ucDataLen >= RPI_RECT_LEN && pucData[0] == RPI_RECT_TAG)
                {
                    /* 矩形地板帧（命令 "A" 的回应, 见 app_rpi.h 该 TAG 的注释）:
                     * [TAG][status][cx u16 LE][cy u16 LE][flags][spread][保留 u16] = 10B。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前** ——
                     *    否则 10B 整帧会被当「9 字节 last packet」吞掉, 永远到不了这里。
                     * ⚠️ 失败帧 (status=0) **也要置 ready**: 消费方靠它立刻解锁返回 0,
                     *    不然每个失败槽要实打实烧满整个超时（与 0x03 色球帧同一条纪律）。 */
                    vSetRectResult(pucData[1],
                                   (uint16_t)((uint16_t)pucData[2]
                                              | ((uint16_t)pucData[3] << 8)),
                                   (uint16_t)((uint16_t)pucData[4]
                                              | ((uint16_t)pucData[5] << 8)),
                                   pucData[6], pucData[7]);
                    vSetRectReady();
                }
                else if (ucDataLen >= RPI_ROAD_LEN && pucData[0] == RPI_ROAD_TAG)
                {
                    /* 看路帧（命令 "G" 的回应, 见 app_rpi.h 该 TAG 的注释）:
                     * [TAG][status][dx i16 /10][ex i16][ey i16][flags][spread][dy u16 /10] = 12B。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前** ——
                     *    否则 12B 整帧会被当「9 字节 last packet」吞掉, 永远到不了这里。
                     * ⚠️ 失败帧 (status=0) **也要置 ready**（同 0x07 那条纪律）:
                     *    消费方靠它立刻解锁返回 0, 不白烧满整个超时。
                     * ⚠️ dx/ex/ey 按 **i16** 解释（dx 要留负号、ex 可以到 0 附近）,
                     *    尾字段 dy 按 **u16**（下边缘的 y 不可能为负）。 */
                    vSetRoadResult(pucData[1],
                                   (int16_t)((uint16_t)pucData[2]
                                             | ((uint16_t)pucData[3] << 8)),
                                   (int16_t)((uint16_t)pucData[4]
                                             | ((uint16_t)pucData[5] << 8)),
                                   (int16_t)((uint16_t)pucData[6]
                                             | ((uint16_t)pucData[7] << 8)),
                                   pucData[8], pucData[9],
                                   (uint16_t)((uint16_t)pucData[10]
                                              | ((uint16_t)pucData[11] << 8)));
                    vSetRoadReady();
                }
                else if (ucDataLen >= RPI_ROADYAW_LEN && pucData[0] == RPI_ROADYAW_TAG)
                {
                    /* 看路校准帧（命令 "H" 的回应, 见 app_rpi.h 该 TAG 的注释）:
                     * [TAG][status][ang i16 /100][cols][rms /10][spread /100][flags] = 8B。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前**（同 0x07/0x08 的纪律）;
                     *    8B 天然落不进那条分支, 但旧固件没有本分支时整帧会被静默丢弃 ——
                     *    部署顺序先 Pi 后固件。
                     * ⚠️ 失败帧 (status=0) **也要置 ready**（同 0x07/0x08 那条纪律）:
                     *    消费方靠它立刻解锁返回 0, 不白烧满整个超时。
                     * ⚠️ ang 按 **i16** 解释（角度可以为负）; cols/rms/spread/flags 都是 u8。 */
                    vSetRoadYawResult(pucData[1],
                                      (int16_t)((uint16_t)pucData[2]
                                                | ((uint16_t)pucData[3] << 8)),
                                      pucData[4], pucData[5], pucData[6], pucData[7]);
                    vSetRoadYawReady();
                }
                else if (ucDataLen >= 2u && pucData[0] == RPI_CFG_TAG)
                {
                    /* Pi 参数帧（命令 "P"/"P?" 的回应, 见 app_rpi.h 该 TAG 的注释与
                     * docs/串口协议.md §6k）: 就地喂给缓存（名字索引, 与顺序无关）。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前**（同 0x07/0x08/0x09
                     *    的纪律）—— 这些帧普遍 ≥9B, 漏进去就会被当成 QR/坐标结果帧,
                     *    误置 RpiFrameReady 把某个正等着"真结果"的步骤提前放行。
                     * ⚠️ 只做缓存更新: **不置 ready、不碰 last packet、不回帧、不鸣叫**
                     *    （vRpiTask 里禁阻塞那条纪律）—— 它就是一条旁路回执。
                     * ⚠️ "QS,…"（「自动下发二维码」的回执, 2026-10 加）与 P 族的回执共用
                     *    这一个 TAG —— 先拦 QS, 剩下的才喂参数缓存。 */
                    if (ucRpiQrAutoReply(pucData, ucDataLen) == 0u)
                    {
                        ucRpiCfgHandleReply(pucData, ucDataLen);
                    }
                }
                else if (ucDataLen == 9 && pucData[0] == 'Q' && pucData[1] == 'P'
                         && pucData[2] >= '0' && pucData[2] <= '9'
                         && pucData[3] >= '0' && pucData[3] <= '9'
                         && pucData[4] >= '0' && pucData[4] <= '9')
                {
                    /* Web 控制端「自动下发二维码」的调试帧（Pi 握手成功后推,
                     * 9B ASCII "QP<ddd>"）: 当场写值 + 打**人工预设**标记 ——
                     * OLED 第 1 页立刻显示 `QR:213*`, 后续所有扫码步骤直接跳过
                     * 并沿用这个值（与 OLED 第 2 页手设 / 调试积木同一语义）。
                     * ⚠️ 刻意用独立前缀 "QP"（Preset）而不是 "QR": "QR" 是扫码
                     *    **结果**帧, 只该由 ucRpiWaitQrResult 在等待窗内消费;
                     *    把它复用成推送通道会把真实扫码结果也误标成「调试值」。
                     * ⚠️ 兼容性双向安全: 旧固件收到 QP 落进下面 last-packet 分支,
                     *    QR 等待窗按前缀不匹配忽略它（Pi 侧扫码指令注入兜底）;
                     *    旧 Pi 不发 QP, 本分支静默不触发。
                     * ⚠️ 这里不许有任何阻塞调用（vBuzzerBeep 就带 vTaskDelay, 而
                     *    vRpiTask 里禁阻塞 —— 「绝不能在 vRpiTask 里调」那条纪律）: 「调试好了」的提示
                     *    就是 OLED 的 `QR:213*` 本身 + 下面这行调试串。
                     * ⚠️ 标记只能断电/复位清（同 vSetQrCodeRandom 的纪律）——
                     *    调试完记得重新上电再上场比赛。 */
                    int32_t iCode = (int32_t)(pucData[2] - '0') * 100
                                  + (int32_t)(pucData[3] - '0') * 10
                                  + (int32_t)(pucData[4] - '0');
                    char acDbg[24];
                    vSetQrCode(iCode);
                    vSetQrManual(1u);
                    (void)snprintf(acDbg, sizeof(acDbg), "QR preset from Pi: %d",
                                   (int)iCode);
                    vRpiSendDebug(acDbg);
                }
                else if (ucDataLen >= RPI_BUCKET_LEN && pucData[0] == RPI_BUCKET_TAG)
                {
                    /* 爆爆桶流帧（命令 "J" 起流期间, 2026-10-24 加, 见 app_rpi.h 该 TAG 的注释
                     * 与 docs/串口协议.md §6m）:
                     * [TAG][status][cx u16][cy u16][w u16][h u16] = 10B 定长, OpenMV 320×240 口径。
                     * 走**独立快照 + 帧计数**（vSetBucketFrame, 同 0x05 那条纪律）:
                     *   · 不置 ready（流没有"结果到达"这个概念, 新鲜度靠帧计数）;
                     *   · 不进 last packet;
                     *   · status bit0=0（本拍无桶/陈旧）**照样推帧计数** —— 消费方靠它区分
                     *     「流活着但没找到」与「流停了」。
                     * ⚠️ 本分支必须排在下面通用的 `ucDataLen >= 9` **之前** ——
                     *    否则 10B 整帧会被当「9 字节 last packet」吞掉（同 0x07/0x08 的纪律）。 */
                    vSetBucketFrame((uint8_t)(pucData[1] & RPI_BUCKET_ST_OK),
                                    (uint16_t)((uint16_t)pucData[2]
                                               | ((uint16_t)pucData[3] << 8)),
                                    (uint16_t)((uint16_t)pucData[4]
                                               | ((uint16_t)pucData[5] << 8)),
                                    (uint16_t)((uint16_t)pucData[6]
                                               | ((uint16_t)pucData[7] << 8)),
                                    (uint16_t)((uint16_t)pucData[8]
                                               | ((uint16_t)pucData[9] << 8)));
                }
                else if (ucDataLen >= 9)
                {
                    vSetLastRpiPacket(pucData);
                    vSetRpiFrameReady();
                }
                break;

            case RPI_TYPE_HELLO:        /* Pi 空闲心跳帧: 仅刷新存活时刻（上面已做） */
                break;

            default:                    /* ACK / DEBUG / 未知: 丢弃 */
                break;
            }

            /* 整帧处理完: 游标跳到下一帧帧头 */
            usPos = (uint16_t)(usPos + ucDataLen + RPI_FRAME_OVERHEAD);
        }

        /* 3. 残帧左移到缓冲头, 下一次 take 直接接在它后面 */
        usFill = (uint16_t)(usFill - usPos);
        if (usFill > 0u)
        {
            memmove(aucRx, &aucRx[usPos], usFill);
        }
    }
}

/* ==== Pi 参数表: 生成块 开始（tools/gen_rpi_cfg.py, 勿手改） ==== */
static const char s_acRpiCfgOpt0[] = "1=手动|3=自动";
static const char s_acRpiCfgOpt1[] = "0=固定|1=自动";
static const char s_acRpiCfgOpt2[] = "0=关|1=开";
static const char s_acRpiCfgOpt3[] = "0=无色|1=带色";
static const char s_acRpiCfgOpt4[] = "0=不转|90=顺90|180=180|270=逆90";
static const char s_acRpiCfgOpt5[] = "auto=自动|incandescent=白炽|tungsten=钨丝|fluorescent=荧光|indoor=室内|daylight=日光|cloudy=阴天";

/* 名字/中文名/分组/类型/选项表。⚠️ 值不在这里 —— 运行时用 "P?,<名>" 现取
 * （见 docs/串口协议.md §6k），表里只放**元数据**（Pi 掉线时也要能画菜单）。 */
const stRpiCfgParamTdf g_astRpiCfgParam[] =
{
    /* ---- 二维码 ---- */
    { "QR_SCAN_TIMEOUT",       "二维码识别超时(秒,无指令时限时的兜底)", "二维码",     RPI_CFG_T_F32, NULL },
    { "QR_SCAN_CENTER_CROP",   "扫码中心裁剪比例(0=关)",       "二维码",     RPI_CFG_T_F32, NULL },
    { "QR_MAIN_CENTER_CROP",   "主摄扫码中心裁剪比例(0=关)",     "二维码",     RPI_CFG_T_F32, NULL },
    { "QR_DIFF_TIMEOUT",       "自研扫码超时(秒,同上兜底)",      "二维码",     RPI_CFG_T_F32, NULL },
    { "QR_DIFF_KNOWN",         "自研候选集(如 123=111~333)", "二维码",     RPI_CFG_T_STR, NULL },
    { "QR_DIFF_MAX_SIZE",      "自研扫码最大边长(px)",        "二维码",     RPI_CFG_T_I32, NULL },
    { "QR_DIFF_CENTER_CROP",   "自研扫码中心裁剪比例(0=关)",     "二维码",     RPI_CFG_T_F32, NULL },
    { "QR_DIFF_DEBUG_KEEP",    "自研失败留档保留份数",          "二维码",     RPI_CFG_T_I32, NULL },
    /* ---- 指令 ---- */
    { "CMD_MARGIN_S",          "单片机给超时里本端留的余量(秒)",    "指令",      RPI_CFG_T_F32, NULL },
    { "CMD_MAX_TIMEOUT_S",     "本端预算上限(秒,护租约看门狗)",    "指令",      RPI_CFG_T_F32, NULL },
    /* ---- 人质 ---- */
    { "HOSTAGE_CONF_THR",      "人质识别置信度阈值",           "人质",      RPI_CFG_T_F32, NULL },
    { "HOSTAGE_IOU_THR",       "人质识别NMS-IoU阈值",       "人质",      RPI_CFG_T_F32, NULL },
    { "HOSTAGE_TIMEOUT",       "人质识别超时(秒)",           "人质",      RPI_CFG_T_F32, NULL },
    { "HOSTAGE_MIN_TARGETS",   "人质识别成功最少目标数",         "人质",      RPI_CFG_T_I32, NULL },
    { "HOSTAGE_ANCHOR_CONF",   "人质排除法锚点置信度(0~1)",     "人质",      RPI_CFG_T_F32, NULL },
    { "HOSTAGE_AUTO_EXPOSURE", "人质曝光模式(3自动/1手动)",     "人质",      RPI_CFG_T_I32, s_acRpiCfgOpt0 },
    { "HOSTAGE_EXPOSURE",      "人质手动曝光(100µs,独立于主摄)", "人质",      RPI_CFG_T_I32, NULL },
    { "HOSTAGE_AUTO_WB",       "人质白平衡(1自动/0固定)",      "人质",      RPI_CFG_T_I32, s_acRpiCfgOpt1 },
    { "HOSTAGE_WB_TEMPERATURE", "人质固定色温(K)",           "人质",      RPI_CFG_T_I32, NULL },
    /* ---- 色球 ---- */
    { "COLOR_TIMEOUT",         "色球识别超时(秒)",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_PROC_SCALE",      "色球检测降采样比例",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_SAT_MIN",         "色球找球饱和度下限",           "色球",      RPI_CFG_T_I32, NULL },
    { "COLOR_VAL_MIN",         "色球找球亮度下限",            "色球",      RPI_CFG_T_I32, NULL },
    { "COLOR_MIN_AREA",        "色球最小连通域面积",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_MIN_CIRCULARITY", "色球圆度下限",              "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_MIN_VOTES",       "色球单色最少命中帧数",          "色球",      RPI_CFG_T_I32, NULL },
    { "COLOR_MAX_SPREAD_PX",   "色球跨帧散布上限(px)",        "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_MIN_GAP_PX",      "色球中心最小间隔(px)",        "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_INFER_EN",        "缺球推断开关(1开)",          "色球",      RPI_CFG_T_I32, s_acRpiCfgOpt2 },
    { "COLOR_INFER_MIN_BALLS", "缺球推断最少检出球数",          "色球",      RPI_CFG_T_I32, NULL },
    { "COLOR_INFER_RETRY",     "推断前先多采一轮(1开)",        "色球",      RPI_CFG_T_I32, s_acRpiCfgOpt2 },
    { "COLOR_BALL_U_CENTER",   "球位模型:中球 u",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_S",          "球位模型:相邻间距 u",         "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_S_TOL",      "球位模型:间距允许偏差",         "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_CENTER_TOL", "球位模型:中线残差上限",         "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_U_MIN",      "球位模型:u 下限",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_U_MAX",      "球位模型:u 上限",           "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_AGREE_TOL",  "两路估计允许差",             "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_CX_CENTER",  "兜底用:理想中球列(px)",       "色球",      RPI_CFG_T_F32, NULL },
    { "COLOR_BALL_CX_TOL_PX",  "兜底用:理想列容差(px)",       "色球",      RPI_CFG_T_F32, NULL },
    /* ---- 矩形 ---- */
    { "RECT_TIMEOUT",          "矩形地板识别超时(秒)",         "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_VOTES",        "矩形地板成功帧数下限",          "矩形",      RPI_CFG_T_I32, NULL },
    { "RECT_MAX_SPREAD_PX",    "矩形地板中心跨帧散布上限(px)",    "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_V_MAX",            "黑胶带亮度上限(V)",          "矩形",      RPI_CFG_T_I32, NULL },
    { "RECT_S_MAX",            "黑胶带饱和度上限(S)",         "矩形",      RPI_CFG_T_I32, NULL },
    { "RECT_CLOSE_K",          "暗掩膜闭运算核(px)",         "矩形",      RPI_CFG_T_I32, NULL },
    { "RECT_MIN_LINE_LEN",     "长边最小长度(×画面宽)",        "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_SHORT_LINE",   "短边最小长度(×画面宽)",        "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_GAP_FRAC",     "两长边最小间距(×板宽)",        "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_AREA_FRAC",    "四边形最小可见面积(占画面)",      "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_INNER_V",      "框内平均亮度下限",            "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_INNER_RATIO",  "框内亮度/画面中位亮度下限",       "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_HW_RATIO",         "长边:短边像素比(三边估中心用)",    "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_INNER_CLOSE",      "围住区掩膜闭运算核(px)",       "矩形",      RPI_CFG_T_I32, NULL },
    { "RECT_EPS_FRAC",         "多边形拟合精度(×周长)",        "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_FILL",         "四边形填充率下限",            "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_TAPE_NEAR_PX",     "边框外偏判胶带距离(px)",       "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MAX_ASPECT",       "四边形长宽比上限",            "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_MIN_ASPECT",       "四边形长宽比下限",            "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_OUTLIER_K",        "多帧离群倍数(MAD)",         "矩形",      RPI_CFG_T_F32, NULL },
    { "RECT_OUTLIER_MIN_PX",   "多帧离群下限(px)",          "矩形",      RPI_CFG_T_F32, NULL },
    /* ---- 标靶 ---- */
    { "TARGET_TIMEOUT",        "靶标识别超时(秒)",           "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_PROC_SCALE",     "靶标检测降采样比例",           "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_SAT_MIN",        "靶标饱和度下限",             "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_VAL_MIN",        "靶标亮度下限",              "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_MIN_AREA",       "靶标最小连通域面积(降采样后)",     "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_TOL",            "靶标跨色尺寸一致倍数上限",        "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_RING_Q_MIN",     "靶标靶环内外径比下限",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_RING_Q_MAX",     "靶标靶环内外径比上限",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_FALLBACK_Q_MIN", "靶标兜底路径q地板",           "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_VOTE_FRAMES",    "靶标最多采帧数",             "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_MIN_VOTES",      "靶标单色最少命中帧数",          "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_MAX_SPREAD_PX",  "靶标双簇判据簇间距(px,两簇各够票数才作废)", "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_OUTLIER_K",      "靶标多帧离群倍数(MAD)",       "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_OUTLIER_MIN_PX", "靶标多帧离群下限(px)",        "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_MIN_TARGETS",    "靶标识别成功最少靶数",          "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_MAX_TARGETS",    "靶标回传目标数上限",           "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_R_MAX",          "靶标半径上限(px,主摄)",       "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_R_MAX_PI",       "靶标半径上限(px,pi摄像头)",    "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_USE_ROI",        "靶标ROI裁切开关(1开/0关)",    "标靶",      RPI_CFG_T_I32, s_acRpiCfgOpt2 },
    { "TARGET_ROI_X0",         "靶标ROI左(相对)",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_ROI_Y0",         "靶标ROI上(相对)",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_ROI_X1",         "靶标ROI右(相对)",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_ROI_Y1",         "靶标ROI下(相对)",          "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_STREAM_HZ",      "靶标流发送频率上限(Hz)",       "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_STREAM_MARGIN_PX", "靶标流跟踪窗边距(px)",        "标靶",      RPI_CFG_T_I32, NULL },
    { "TARGET_STREAM_R_TOL",   "靶标流半径门(0~1)",         "标靶",      RPI_CFG_T_F32, NULL },
    { "TARGET_STREAM_USE_COLOR", "靶标流颜色跟踪开关(1带色/0无色)",  "标靶",      RPI_CFG_T_I32, s_acRpiCfgOpt3 },
    { "TARGET_STREAM_MAX_S",   "靶标流安全时长上限(秒)",        "标靶",      RPI_CFG_T_F32, NULL },
    /* ---- 看路向 ---- */
    { "ROADYAW_TIMEOUT",       "看路校准超时(秒)",           "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_VOTE_FRAMES",   "看路校准最多采帧数",           "看路向",     RPI_CFG_T_I32, NULL },
    { "ROADYAW_MIN_VOTES",     "看路校准成功帧数下限",          "看路向",     RPI_CFG_T_I32, NULL },
    { "ROADYAW_MAX_SPREAD_DEG", "看路校准角度跨帧散布上限(度)",     "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_ATTEMPTS",      "看路校准总轮数上限(含重采)",      "看路向",     RPI_CFG_T_I32, NULL },
    { "ROADYAW_OUTLIER_K",     "看路校准多帧离群倍数(MAD)",     "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_OUTLIER_MIN_DEG", "看路校准多帧离群下限(度)",       "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_ROI_TOP",       "看路校准子窗上(相对)",         "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_ROI_BOTTOM",    "看路校准子窗下(相对)",         "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_ROI_LEFT",      "看路校准子窗左(相对)",         "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_ROI_RIGHT",     "看路校准子窗右(相对,罩一段直边)",   "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_MIN_SPAN_PX",   "看路校准直段最短长度(px)",      "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_MIN_COLS",      "看路校准拟合内点数下限",         "看路向",     RPI_CFG_T_I32, NULL },
    { "ROADYAW_MAX_RMS_PX",    "看路校准拟合残差RMS上限(px)",   "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_INLIER_PX",     "看路校准剔点阈值(px)",        "看路向",     RPI_CFG_T_F32, NULL },
    { "ROADYAW_EDGE_MARGIN_PX", "看路校准贴窗边剔除带(px)",      "看路向",     RPI_CFG_T_F32, NULL },
    /* ---- 看路 ---- */
    { "ROAD_TIMEOUT",          "看路识别超时(秒)",           "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_AUTO_EXPOSURE",    "看路曝光模式(V4L2:1手动/3自动;手动对补光切换免疫)", "看路",      RPI_CFG_T_I32, s_acRpiCfgOpt0 },
    { "ROAD_EXPOSURE",         "看路手动曝光(100µs;过暗调大/过曝调小,小步调)", "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_VOTE_FRAMES",      "看路最多采帧数",             "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_MIN_VOTES",        "看路成功帧数下限",            "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_MAX_SPREAD_PX",    "看路Δx跨帧散布上限(px)",      "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_OUTLIER_K",        "看路多帧离群倍数(MAD)",       "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_OUTLIER_MIN_PX",   "看路多帧离群下限(px)",        "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_PROC_SCALE",       "看路降采样比例",             "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_ROI_TOP",          "看路ROI上(相对)",          "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_ROI_BOTTOM",       "看路ROI下(相对,须盖住黄区左边缘)", "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_ROI_LEFT",         "看路ROI左(相对)",          "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_ROI_RIGHT",        "看路ROI右(相对)",          "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_WARM_FLOOR_YB",    "看路Y-B阈值地板(自适应阈值的下限)", "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_PEAK_FRAC",        "看路谷底法黄区峰显著度(峰高>=主峰x它)", "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_WARM_V_MIN",       "看路黄区亮度下限(V)",         "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_WARM_OPEN_K",      "看路黄区开运算核(px)",        "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_WARM_CLOSE_K",     "看路黄区闭运算核(px)",        "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_WARM_MIN_AREA",    "看路黄区最小面积(px2)",       "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_WARM_CONTRAST_MIN", "看路黄度对比度闸门(块内-块外Y-B中位差下限)", "看路",      RPI_CFG_T_I32, NULL },
    { "ROAD_WARM_MAX_AREA_FRAC", "看路掩膜泛滥闸门(黄区占ROI面积比上限)", "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_DX_MIN_PX",        "看路Δx下限(px)",          "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_DX_MAX_PX",        "看路Δx上限(px)",          "看路",      RPI_CFG_T_F32, NULL },
    { "ROAD_AUTO_WB",          "看路白平衡(1自动/0固定)",      "看路",      RPI_CFG_T_I32, s_acRpiCfgOpt1 },
    { "ROAD_WB_TEMPERATURE",   "看路固定色温(K,与Y-B判据配套)",  "看路",      RPI_CFG_T_I32, NULL },
    /* ---- 巡线 ---- */
    { "LINE_KP",               "巡线PID-Kp",            "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_KI",               "巡线PID-Ki",            "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_KD",               "巡线PID-Kd",            "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_ERR_SIGN",         "巡线误差方向(1/-1)",        "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_SETPOINT",         "巡线目标偏移(offset单位)",    "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_SEND_HZ",          "巡线转发频率(Hz)",          "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_MAX_DURATION",     "巡线最长时长(秒)",           "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_LOST_HOLD_S",      "丢线保持时间(秒)",           "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_LOST_CONFIRM_S",   "丢线定向找线确认期(秒)",        "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_LOST_SEARCH_S",    "丢线找线时长(秒)",           "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_SEARCH_STEER",     "找线steer(负号=倒向)",      "巡线",      RPI_CFG_T_F32, NULL },
    { "LINE_LOG_HZ",           "巡线原始值打印频率(Hz,0=关)",   "巡线",      RPI_CFG_T_F32, NULL },
    /* ---- OpenMV ---- */
    { "OPENMV_LED_BRIGHTNESS", "OpenMV补光亮度(0~100, 点亮值; 命令T只临时灭灯)", "OpenMV",  RPI_CFG_T_I32, NULL },
    /* ---- 相机 ---- */
    { "CAM_MAIN_FOURCC",       "主摄像头像素格式",            "相机",      RPI_CFG_T_STR, NULL },
    { "CAM_QR_FOURCC",         "扫码摄像头像素格式",           "相机",      RPI_CFG_T_STR, NULL },
    { "CAM_MAIN_AUTO_EXPOSURE", "主摄曝光模式(3自动/1手动)",     "相机",      RPI_CFG_T_I32, s_acRpiCfgOpt0 },
    { "CAM_MAIN_EXPOSURE",     "主摄手动曝光(100µs,100=10ms)", "相机",      RPI_CFG_T_I32, NULL },
    { "CAM_QR_AUTO_EXPOSURE",  "扫码相机曝光模式(3自动/1手动)",   "相机",      RPI_CFG_T_I32, s_acRpiCfgOpt0 },
    { "CAM_QR_EXPOSURE",       "扫码相机手动曝光(100µs)",     "相机",      RPI_CFG_T_I32, NULL },
    { "CAM_MAIN_AUTO_WB",      "主摄白平衡(1自动/0固定)",      "相机",      RPI_CFG_T_I32, s_acRpiCfgOpt1 },
    { "CAM_MAIN_WB_TEMPERATURE", "主摄固定色温(K,2800~6500)", "相机",      RPI_CFG_T_I32, NULL },
    { "CAM_QR_AUTO_WB",        "扫码相机白平衡(1自动/0固定)",    "相机",      RPI_CFG_T_I32, s_acRpiCfgOpt1 },
    { "CAM_QR_WB_TEMPERATURE", "扫码相机固定色温(K)",         "相机",      RPI_CFG_T_I32, NULL },
    /* ---- pi摄像头 ---- */
    { "PICAM_TARGET_WIDTH",    "pi摄像头取流宽(2592=原生满视场)", "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_TARGET_HEIGHT",   "pi摄像头取流高(必须>1080否则视场窄)", "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_ROTATE",          "pi摄像头交付前旋转(改这个=换坐标系,瞄准常数要重标)", "pi摄像头",   RPI_CFG_T_I32, s_acRpiCfgOpt4 },
    { "PICAM_TARGET_FPS",      "pi摄像头取流帧率",           "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_TARGET_QUALITY",  "pi摄像头MJPEG质量",        "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_WARM_FRAMES",     "pi摄像头起流丢弃帧数(等曝光/白平衡收敛)", "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_TRY_BUDGET_S",    "pi摄像头识别子预算(秒,超了就退回主摄)", "pi摄像头",   RPI_CFG_T_F32, NULL },
    { "PICAM_STREAM_UP_S",     "pi摄像头起流等待上限(秒)",      "pi摄像头",   RPI_CFG_T_F32, NULL },
    { "PICAM_STALE_S",         "pi摄像头流掉帧多久算死(秒,退回主摄)", "pi摄像头",   RPI_CFG_T_F32, NULL },
    { "PICAM_IDLE_TIMEOUT_S",  "pi摄像头空闲自动关闭(秒)",      "pi摄像头",   RPI_CFG_T_F32, NULL },
    { "PICAM_USES_MAX",        "pi摄像头预热次数上限",         "pi摄像头",   RPI_CFG_T_I32, NULL },
    { "PICAM_AWB_MODE",        "pi摄像头白平衡模式(色识别飘先动它)", "pi摄像头",   RPI_CFG_T_STR, s_acRpiCfgOpt5 },
    { "PICAM_AWB_GAINS",       "pi摄像头固定白平衡增益(r,g)",   "pi摄像头",   RPI_CFG_T_STR, NULL },
    /* ---- 画面 ---- */
    { "FRAME_WIDTH",           "摄像头分辨率宽",             "画面",      RPI_CFG_T_I32, NULL },
    { "FRAME_HEIGHT",          "摄像头分辨率高",             "画面",      RPI_CFG_T_I32, NULL },
    /* ---- 租约 ---- */
    { "LEASE_IDLE_TIMEOUT_S",  "相机租约空闲释放(秒,也是每次使用的窗口)", "租约",      RPI_CFG_T_F32, NULL },
    { "LEASE_WARM_WAIT_S",     "等预热落地上限(秒)",          "租约",      RPI_CFG_T_F32, NULL },
    { "LEASE_USES_MAX",        "预热次数上限(窗口最多放宽到它×上述秒数)", "租约",      RPI_CFG_T_I32, NULL },
    { "LEASE_STALE_S",         "租约帧新鲜度阈值(秒,超时判读线程停摆)", "租约",      RPI_CFG_T_F32, NULL },
    /* ---- 系统 ---- */
    { "SYSTEM_CMD_CONFIRM_S",  "关机/重启二次确认窗口(秒)",      "系统",      RPI_CFG_T_F32, NULL },
};
const uint16_t g_usRpiCfgParamNum = (uint16_t)(sizeof(g_astRpiCfgParam) / sizeof(g_astRpiCfgParam[0]));
/* ==== Pi 参数表: 生成块 结束 ==== */

/* ==================== Pi 参数缓存（"P" 命令族, 2026-10 加） ====================
 * 让 OLED 调参屏直接读/改 **Pi 侧 config.py 的参数**（协议见 docs/串口协议.md §6k）:
 *   读: vRpiCfgPoll 在 vRpiTask 里按需发 "P?,<名>", 回值进本段缓存;
 *       屏上只读缓存（ucRpiCfgCached / pcRpiCfgStrAt）—— **读方从不自己发串口**,
 *       所以 OLED/蓝牙任何任务上下文都能调, 也就不会跟"等结果"的等待窗抢帧。
 *   写: vRpiCfgSet 发 "P,<名>,<值>"（**临时**改 Pi 进程内存, 不落盘, 重启还原）。
 *
 * 三条纪律:
 *   ① **只查"想要"的项**（vRpiCfgWant, OLED 画某一行时标它）—— 一百多项全查会把链路刷爆,
 *      而一屏最多同时看到 4 行。屏一离开, 一帧内（50ms）就没人标了, 自动停。
 *   ② **一次只有一条在飞**（s_usCfgFly）: 回值按名字索引、本身不怕交错, 但 Pi 侧任务是
 *      **串行**的（"4"/"5" 这类要跑好几秒）, 并发多条只会让"谁的回值"纠缠不清 —— 一条一条来。
 *   ③ **绝不置 ready / 不碰 last packet / 不鸣叫**（同 vRpiTask 那条禁阻塞纪律）:
 *      本帧是旁路回执, 对任何"等结果"的窗口都是隐形的。
 *
 * ⚠️ 缓存会旧, 但**不再按时间自动重查**（2026-10 改 —— 用户口径: "在被更改时、在需要时
 *    才更新", 原来的 3s 新鲜期让 Pi 调参屏一挂就是每秒两条 "P?" 刷爆对端日志）。
 *    现在重查只剩三个触发:
 *      ① 该项**从没回过值**（ASKED=0, 第一次显示）;
 *      ② 最近一条回值是 **ERR** → 按新鲜期重试, 最多 RPI_CFG_RETRY_MAX 次
 *         （治"Pi 侧还没就绪"的瞬态失败, 同时不让"名字没登记"的永久失败无限重试）;
 *      ③ 链路**重新变 OK** 的那一拍 —— 全表状态作废, 在屏上的那几行各自重查一次
 *         （Pi 可能重启过, 旧缓存已经没有意义）。
 *    其余时间屏上显示的一直是缓存值: 用户改值走 "P,<名>,<值>"、Pi 回 "OK,<名>,<值>"
 *    顺带刷新缓存; Pi 侧/网页改的值要等上面三个触发之一才反映到屏上 —— 这是用
 *    "刷新及时性"换"链路清净"的**有意取舍**（调参屏挂多久都不再刷一条查询）。
 * ⚠️ 收到 ERR 时**清掉 VALID** —— 宁可让屏上显示"--"也不显示一个骗人的数。
 * ⚠️ 表长由生成块派生（sizeof）—— 表重生成后本段自动跟着对, 没有第二处要改的数字。 */
#define RPI_CFG_ST_WANT   0x01u   /* OLED 正在显示它（只有它会被轮询） */
#define RPI_CFG_ST_VALID  0x02u   /* 有值（数值看 s_afCfgVal, 字符串看 s_aacCfgStr） */
#define RPI_CFG_ST_FLY    0x04u   /* 查询在飞（等 "V,…"/"ERR,…" 回来） */
#define RPI_CFG_ST_ASKED  0x08u   /* 收到过回值（= 知道该等多久再重问） */
#define RPI_CFG_ST_ERR    0x10u   /* 最近一条回值是 ERR（值已清, 屏上打"--"） */

#define RPI_CFG_NOIDX     0xFFFFu
#define RPI_CFG_POLL_MS   300u    /* 两条查询之间的最小间隔 */
#define RPI_CFG_FLY_MS    2000u   /* 在飞超时（Pi 侧长任务会把 P? 排在它后面, 给宽点） */
#define RPI_CFG_FRESH_MS  3000u   /* **只**用于 ERR 重试的间隔（不再是全量刷新节奏） */
#define RPI_CFG_RETRY_MAX 3u      /* 同一条参数连续 ERR 的重试上限（防永久失败无限刷） */
#define RPI_CFG_STR_MAX   24u     /* 一个 str 型参数的值最长缓存（含 NUL; 现表里最长 "MJPG"） */
#define RPI_CFG_STR_NUM   8u      /* 最多同时缓存几个 str 参数（现表里 3 个; 超了就不缓存） */
#define RPI_CFG_ERR_MAX   40u     /* 最近一条 ERR 原因（屏上闪一行用） */
#define RPI_CFG_CMD_MAX   48u     /* "P?,<名>" / "P,<名>,<值>" 的发送缓冲（名字最长 23） */

#define RPI_CFG_MAX       (sizeof(g_astRpiCfgParam) / sizeof(g_astRpiCfgParam[0]))

static uint8_t  s_aucCfgSt[RPI_CFG_MAX];     /* 状态位（RPI_CFG_ST_*） */
static float    s_afCfgVal[RPI_CFG_MAX];     /* 数值缓存（F32/I32） */
static uint32_t s_aulCfgTick[RPI_CFG_MAX];   /* 最近一次收到回值的 tick（ASKED 失效时无意义） */
static uint8_t  s_aucCfgRetry[RPI_CFG_MAX];  /* 连续 ERR 计数（收到正常回值即清零; 封顶不再试） */
static char     s_aacCfgStr[RPI_CFG_STR_NUM][RPI_CFG_STR_MAX];  /* str 型按"第几个 str"分槽 */
static uint16_t s_usCfgScan     = 0u;        /* 轮询游标 */
static uint16_t s_usCfgFly      = RPI_CFG_NOIDX;   /* 在飞查询的项（NOIDX = 没有） */
static uint32_t s_ulCfgFlyTick  = 0u;
static uint32_t s_ulCfgNextTick = 0u;        /* 节奏闸门（下一次允许发查询的时刻） */
static char     s_acCfgErr[RPI_CFG_ERR_MAX]; /* 最近一条 ERR 的原因 */
static uint32_t s_ulCfgErrTick  = 0u;
static uint8_t  s_ucCfgLastLink = 0u;        /* 上一拍链路是否 OK（探测「重新连上」沿） */

/* ---- 小工具: 查表 / 切片查表 / str 槽位 ---- */

/* 按名字切片查表（回值里的名字不是 NUL 结尾的, 不能直接 strcmp） */
static const stRpiCfgParamTdf *cpstRpiCfgFindN(const char *pcName, uint16_t usLen)
{
    uint16_t i;

    if ((pcName == NULL) || (usLen == 0u)) { return NULL; }
    for (i = 0u; i < g_usRpiCfgParamNum; i++)
    {
        const char *pcTab = g_astRpiCfgParam[i].pcName;

        if ((uint16_t)strlen(pcTab) != usLen) { continue; }
        if (memcmp(pcTab, pcName, (size_t)usLen) == 0) { return &g_astRpiCfgParam[i]; }
    }
    return NULL;
}

const stRpiCfgParamTdf *cpstRpiCfgFind(const char *pcName)
{
    if (pcName == NULL) { return NULL; }
    return cpstRpiCfgFindN(pcName, (uint16_t)strlen(pcName));
}

/* 表项 → 下标 */
static uint16_t usRpiCfgIdxOf(const stRpiCfgParamTdf *pstIt)
{
    if ((pstIt == NULL) || (pstIt < &g_astRpiCfgParam[0])
        || (pstIt >= &g_astRpiCfgParam[RPI_CFG_MAX]))
    {
        return RPI_CFG_NOIDX;
    }
    return (uint16_t)(pstIt - &g_astRpiCfgParam[0]);
}

/* str 型参数的槽位号 = 它在表里是**第几个** str（超出 RPI_CFG_STR_NUM 返回 0xFF）。
 * 一槽一项: 一屏同时显示两个 str 参数时不会互相踩。 */
static uint8_t ucRpiCfgStrRank(uint16_t usIdx)
{
    uint16_t i;
    uint8_t  ucRank = 0u;

    if (usIdx >= g_usRpiCfgParamNum) { return 0xFFu; }
    if (g_astRpiCfgParam[usIdx].ucType != RPI_CFG_T_STR) { return 0xFFu; }
    for (i = 0u; i < usIdx; i++)
    {
        if (g_astRpiCfgParam[i].ucType == RPI_CFG_T_STR) { ucRank++; }
    }
    return (ucRank < RPI_CFG_STR_NUM) ? ucRank : 0xFFu;
}

/* 解析回值里的数字（形状 = Python "%.6g"/"%d" 的产物）:
 *   [-]<digits>[.<digits>][(e|E)[+|-]<digits>]
 * ⚠️ 有意**不**用 atof/strtof/sscanf: 那几个会把整套浮点解析拉进镜像（同 app_bt.c 那条
 *    "只用整数格式化"的取舍 —— 格式化那边用 %.6g 是因为标准库里早就有浮点输出,
 *    见 app_oled.c 的调参行; **解析**这条路一走就是新拉一大坨, 不值）。
 * 返回 0 = 形状不认识 —— 调用方当失败,**不写缓存**（宁可显示"--"也不写个垃圾数）。 */
static uint8_t ucRpiCfgParseNum(const char *pc, uint16_t usLen, float *pfVal)
{
    uint16_t i = 0u;
    uint8_t  ucNeg = 0u;
    uint8_t  ucDig = 0u;
    float    fV = 0.0f;

    if ((pc == NULL) || (pfVal == NULL) || (usLen == 0u)) { return 0u; }

    if ((pc[0] == '-') || (pc[0] == '+'))
    {
        ucNeg = (uint8_t)((pc[0] == '-') ? 1u : 0u);
        i = 1u;
    }
    while ((i < usLen) && (pc[i] >= '0') && (pc[i] <= '9'))
    {
        fV = (fV * 10.0f) + (float)(pc[i] - '0');
        ucDig = 1u;
        i++;
    }
    if ((i < usLen) && (pc[i] == '.'))
    {
        float fScale = 0.1f;

        i++;
        while ((i < usLen) && (pc[i] >= '0') && (pc[i] <= '9'))
        {
            fV += fScale * (float)(pc[i] - '0');
            fScale *= 0.1f;
            ucDig = 1u;
            i++;
        }
    }
    if (ucDig == 0u) { return 0u; }                 /* 一位数字都没有 → 不认 */
    if ((i < usLen) && ((pc[i] == 'e') || (pc[i] == 'E')))
    {
        int16_t iExp = 0;
        uint8_t ucENeg = 0u;
        uint8_t ucEDig = 0u;

        i++;
        if ((i < usLen) && ((pc[i] == '-') || (pc[i] == '+')))
        {
            ucENeg = (uint8_t)((pc[i] == '-') ? 1u : 0u);
            i++;
        }
        while ((i < usLen) && (pc[i] >= '0') && (pc[i] <= '9'))
        {
            if (iExp < 38) { iExp = (int16_t)((iExp * 10) + (pc[i] - '0')); }
            ucEDig = 1u;
            i++;
        }
        if (ucEDig == 0u) { return 0u; }
        while (iExp > 0) { fV = (ucENeg != 0u) ? (fV * 0.1f) : (fV * 10.0f); iExp--; }
    }
    *pfVal = (ucNeg != 0u) ? -fV : fV;
    return 1u;
}

/* ---- 对外接口（声明见 app_rpi.h）---- */

void vRpiCfgWant(uint16_t usIdx)
{
    if (usIdx >= g_usRpiCfgParamNum) { return; }
    taskENTER_CRITICAL();
    s_aucCfgSt[usIdx] |= RPI_CFG_ST_WANT;
    taskEXIT_CRITICAL();
}

void vRpiCfgUnwantAll(void)
{
    uint16_t i;

    taskENTER_CRITICAL();
    for (i = 0u; i < (uint16_t)RPI_CFG_MAX; i++)
    {
        s_aucCfgSt[i] = (uint8_t)(s_aucCfgSt[i] & (uint8_t)~RPI_CFG_ST_WANT);
    }
    taskEXIT_CRITICAL();
}

uint8_t ucRpiCfgCached(uint16_t usIdx, float *pfVal)
{
    if (usIdx >= g_usRpiCfgParamNum) { return 0u; }
    if ((s_aucCfgSt[usIdx] & RPI_CFG_ST_VALID) == 0u) { return 0u; }
    if (pfVal != NULL) { *pfVal = s_afCfgVal[usIdx]; }
    return 1u;
}

const char *pcRpiCfgStrAt(uint16_t usIdx)
{
    uint8_t ucRank = ucRpiCfgStrRank(usIdx);

    if (ucRank == 0xFFu) { return ""; }
    if ((s_aucCfgSt[usIdx] & RPI_CFG_ST_VALID) == 0u) { return ""; }
    return s_aacCfgStr[ucRank];
}

/* 发一条临时设参。
 * ⚠️ 只在链路 OK 时发: 链路不通还发 = 值只改在本端缓存里, 屏上"改了"而 Pi 那边没动,
 *    这种"看着成了"的假象比明确失败坏得多（回值里 ERR 也会清 VALID, 见解析段）。
 * ⚠️ 乐观写缓存（立刻显示新值）: 随后 Pi 回的 "OK,<名>,<值>" 会用**它自己的**格式化
 *    结果覆盖一次（两边口径都是 6 位有效数字, 通常一字不差）。 */
void vRpiCfgSet(uint16_t usIdx, float fVal)
{
    char acBuf[RPI_CFG_CMD_MAX];
    int  iN;

    if (usIdx >= g_usRpiCfgParamNum) { return; }
    if (emGetRpiLinkState() != RPI_LINK_OK) { return; }

    iN = snprintf(acBuf, sizeof(acBuf), "P,%s,%.6g",
                  g_astRpiCfgParam[usIdx].pcName, (double)fVal);
    if ((iN <= 0) || (iN >= (int)sizeof(acBuf))) { return; }

    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)iN);

    s_afCfgVal[usIdx]      = fVal;
    taskENTER_CRITICAL();
    s_aucCfgSt[usIdx] = (uint8_t)((s_aucCfgSt[usIdx]
                                   | (uint8_t)(RPI_CFG_ST_VALID | RPI_CFG_ST_ASKED))
                                  & (uint8_t)~RPI_CFG_ST_ERR);
    taskEXIT_CRITICAL();
    s_aulCfgTick[usIdx] = (uint32_t)xTaskGetTickCount();
    s_acCfgErr[0]       = '\0';
}

const char *pcRpiCfgLastErr(void)
{
    if (s_acCfgErr[0] == '\0') { return ""; }
    if ((xTaskGetTickCount() - s_ulCfgErrTick) > pdMS_TO_TICKS(2000)) { return ""; }
    return s_acCfgErr;
}

/* ---- 轮询（只在 vRpiTask 里调）----
 * 找一个「想要 且 需要查询」的项发 "P?,<名>":
 *   · 从没回过值（没 ASKED）→ 查（第一次显示这一行）;
 *   · 回过 ERR（没 VALID 但 ASKED）→ 按新鲜期重试, 封顶 RPI_CFG_RETRY_MAX 次 ——
 *     治"Pi 侧还没就绪"的瞬态失败, 而**不会打成一串**: 节拍看的是**回值时刻**
 *     （s_aulCfgTick）, 不是发查询的时刻; "名字没登记"那种永久失败试满上限就停。
 *   · **有值就不再自动重查**（2026-10 改, 见段头那条）—— 刷屏的根源就是这里原来
 *     的"过了新鲜期就重查"。
 * 链路**重新变 OK** 的那一拍把全表状态作废（见函数中间那段）, 在屏上的行各自重查一次。
 */
static void vRpiCfgPoll(void)
{
    uint32_t ulNow = (uint32_t)xTaskGetTickCount();
    uint16_t usIdx = RPI_CFG_NOIDX;
    uint16_t i;

    /* 在飞超时: 放弃它（Pi 侧长任务可能把 P? 排在好几秒后面; 那次回值迟到也照样会被
     * 解析进缓存, 只是本端不再等它）。放开的那一刻**允许立刻换一条**, 不为丢掉的
     * 那条白等一个节奏窗。 */
    if (s_usCfgFly != RPI_CFG_NOIDX)
    {
        if ((ulNow - s_ulCfgFlyTick) < pdMS_TO_TICKS(RPI_CFG_FLY_MS)) { return; }
        taskENTER_CRITICAL();
        s_aucCfgSt[s_usCfgFly] = (uint8_t)(s_aucCfgSt[s_usCfgFly] & (uint8_t)~RPI_CFG_ST_FLY);
        taskEXIT_CRITICAL();
        s_usCfgFly      = RPI_CFG_NOIDX;
        s_ulCfgNextTick = ulNow;
    }

    if (emGetRpiLinkState() != RPI_LINK_OK)
    {
        s_ucCfgLastLink = 0u;                 /* 记住"不在", 重连那一拍才有沿可判 */
        return;                               /* 链路不通: 一个字节都不发 */
    }

    if (g_usRpiCfgParamNum == 0u) { return; }

    /* 链路「不在 → OK」的上升沿: Pi 可能重启过（临时值全没了）, 旧缓存一律作废 ——
     * 在屏上的那几行接下来各自重查一次, 不在屏上的不查（等它再被显示时按"没值"走）。 */
    if (s_ucCfgLastLink == 0u)
    {
        s_ucCfgLastLink = 1u;
        taskENTER_CRITICAL();
        for (i = 0u; i < (uint16_t)RPI_CFG_MAX; i++)
        {
            s_aucCfgSt[i]    = 0u;            /* WANT 会在下一拍由 OLED 重标, 直接全清最干净 */
            s_aucCfgRetry[i] = 0u;
        }
        taskEXIT_CRITICAL();
        s_usCfgFly = RPI_CFG_NOIDX;           /* 在飞的那条也作废（回值来了也无名可落） */
        s_usCfgScan = 0u;
    }

    for (i = 0u; i < g_usRpiCfgParamNum; i++)
    {
        uint8_t  ucSt;
        uint16_t usCur = s_usCfgScan;

        s_usCfgScan = (uint16_t)((s_usCfgScan + 1u) % g_usRpiCfgParamNum);
        ucSt = s_aucCfgSt[usCur];
        if ((ucSt & RPI_CFG_ST_WANT) == 0u) { continue; }   /* 没在屏上 → 不查 */
        if ((ucSt & RPI_CFG_ST_ASKED) != 0u)
        {
            /* 有回值历史: 只有"ERR 重试"一种理由再问（有 VALID 的不再自动刷, 见段头） */
            if ((ucSt & RPI_CFG_ST_VALID) != 0u) { continue; }
            if ((ulNow - s_aulCfgTick[usCur]) < pdMS_TO_TICKS(RPI_CFG_FRESH_MS)) { continue; }
            if (s_aucCfgRetry[usCur] >= RPI_CFG_RETRY_MAX) { continue; }
        }
        /* 到这儿 = 这条该查询了（从没回过值, 或 ERR 重试到点）。**统一守节奏闸门** ——
         * 四条新翻出来的行也是一条一条发, 不是一梭子四连发（Pi 侧串行, 一串只会排队）。 */
        if ((int32_t)(ulNow - s_ulCfgNextTick) < 0) { return; }
        usIdx = usCur;
        break;
    }
    if (usIdx == RPI_CFG_NOIDX) { return; }

    {
        char acBuf[RPI_CFG_CMD_MAX];
        int  iN = snprintf(acBuf, sizeof(acBuf), "P?,%s", g_astRpiCfgParam[usIdx].pcName);

        if ((iN <= 0) || (iN >= (int)sizeof(acBuf))) { return; }
        vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)acBuf, (uint8_t)iN);

        taskENTER_CRITICAL();
        s_aucCfgSt[usIdx] |= RPI_CFG_ST_FLY;
        taskEXIT_CRITICAL();
        s_usCfgFly      = usIdx;
        s_ulCfgFlyTick  = ulNow;
        s_ulCfgNextTick = ulNow + pdMS_TO_TICKS(RPI_CFG_POLL_MS);
    }
}

/* ---- 回值解析（只在 vRpiTask 的 TAG 分支里调）----
 * 文本三形状（见 app_rpi.h::RPI_CFG_TAG）:
 *   "V,<名>,<type>,<值>[,<选项表>]"   取参成功
 *   "OK,<名>,<值>"                    临时设参成功
 *   "ERR,<名>,<原因>"  / "ERR,<原因>"  失败（名字对不上表 = 全局错误, 照样收下原因）
 * 名字一律**按切片**比对（回值不是 NUL 结尾的 C 串）, 值可以是负号/小数点/指数。
 * ⚠️ 认不出的形状**静默丢**（老 Pi 或手发的）—— 这里不是"响亮失败"的地方:
 *    vRpiTask 里打日志要走 vRpiSendDebug, 而它每帧一条会把链路刷爆。 */
static void ucRpiCfgHandleReply(const uint8_t *pucData, uint8_t ucDataLen)
{
    const char            *pcTxt = (const char *)&pucData[1];
    uint16_t               usLen = (uint16_t)(ucDataLen - 1u);   /* 不含 TAG */
    uint16_t               usPos, usName, usNameEnd;
    uint8_t                ucKind;                                /* 0=V 1=OK 2=ERR */
    const stRpiCfgParamTdf *pst;
    uint16_t               usIdx;
    uint32_t               ulNow = (uint32_t)xTaskGetTickCount();

    if (usLen == 0u) { return; }

    if ((usLen >= 2u) && (pcTxt[0] == 'V') && (pcTxt[1] == ','))
    {
        ucKind = 0u;
        usPos  = 2u;
    }
    else if ((usLen >= 3u) && (memcmp(pcTxt, "OK,", 3u) == 0))
    {
        ucKind = 1u;
        usPos  = 3u;
    }
    else if ((usLen >= 4u) && (memcmp(pcTxt, "ERR,", 4u) == 0))
    {
        ucKind = 2u;
        usPos  = 4u;
    }
    else
    {
        return;                                /* 形状不认识: 静默丢 */
    }

    /* 名字 = usPos 起、到下一个 ',' 或结尾 */
    usName    = usPos;
    usNameEnd = usName;
    while ((usNameEnd < usLen) && (pcTxt[usNameEnd] != ',')) { usNameEnd++; }

    pst   = cpstRpiCfgFindN(&pcTxt[usName], (uint16_t)(usNameEnd - usName));
    usIdx = usRpiCfgIdxOf(pst);

    /* 这一条回值 = 在飞的那条查询有结果了（名字对不上或是全局错误时, 也说明
     * "我发出去的那条"已经不会再被回答 —— 按名字比对, 只清自己的）。 */
    if ((s_usCfgFly != RPI_CFG_NOIDX)
        && ((usIdx == RPI_CFG_NOIDX) || (usIdx == s_usCfgFly)))
    {
        taskENTER_CRITICAL();
        s_aucCfgSt[s_usCfgFly] = (uint8_t)(s_aucCfgSt[s_usCfgFly] & (uint8_t)~RPI_CFG_ST_FLY);
        taskEXIT_CRITICAL();
        s_usCfgFly = RPI_CFG_NOIDX;
    }

    if (ucKind == 2u)
    {
        /* 失败: 记原因 + **清 VALID**（屏上改回"--", 并触发一次重查去拿真值）;
         * 名字切不出来 / 对不上表 = 全局错误（如"参数名为空"), 一样收下原因文本。 */
        uint16_t usErr = (usNameEnd < usLen) ? (uint16_t)(usNameEnd + 1u) : usLen;
        uint16_t usN   = (uint16_t)(usLen - usErr);

        if (usN >= RPI_CFG_ERR_MAX) { usN = (uint16_t)(RPI_CFG_ERR_MAX - 1u); }
        memcpy(s_acCfgErr, &pcTxt[usErr], (size_t)usN);
        s_acCfgErr[usN] = '\0';
        s_ulCfgErrTick  = ulNow;

        if (usIdx != RPI_CFG_NOIDX)
        {
            taskENTER_CRITICAL();
            s_aucCfgSt[usIdx] = (uint8_t)((s_aucCfgSt[usIdx]
                                           | (uint8_t)(RPI_CFG_ST_ASKED | RPI_CFG_ST_ERR))
                                          & (uint8_t)~RPI_CFG_ST_VALID);
            taskEXIT_CRITICAL();
            if (s_aucCfgRetry[usIdx] < 0xFFu) { s_aucCfgRetry[usIdx]++; }  /* 封顶后不再重试 */
            s_aulCfgTick[usIdx] = ulNow;
        }
        return;
    }

    if (usIdx == RPI_CFG_NOIDX) { return; }    /* V/OK 但名字不在表里（固件旧/表没重生成）*/

    if (g_astRpiCfgParam[usIdx].ucType == RPI_CFG_T_STR)
    {
        /* V 的形状: `<名>,<type>,<值>[,<选项表>]` —— str 型没有选项表, 值一直到结尾;
         * OK 的形状: `<名>,<值>` —— 同为"到结尾"。 */
        uint16_t usVal = (uint16_t)(usNameEnd + 1u);
        uint16_t usN;
        uint8_t  ucRank;

        if (usNameEnd >= usLen) { return; }    /* 连值都没有（"V,名"）: 丢 */
        if (ucKind == 0u)                      /* V: 跳过 <type> 字段 */
        {
            while ((usVal < usLen) && (pcTxt[usVal] != ',')) { usVal++; }
            if (usVal >= usLen) { return; }
            usVal++;
        }
        if (usVal >= usLen) { return; }         /* 空值: 丢 */
        ucRank = ucRpiCfgStrRank(usIdx);
        if (ucRank != 0xFFu)                    /* 槽位不够就只更新状态, 不缓存串 */
        {
            usN = (uint16_t)(usLen - usVal);
            if (usN >= RPI_CFG_STR_MAX) { usN = (uint16_t)(RPI_CFG_STR_MAX - 1u); }
            memcpy(s_aacCfgStr[ucRank], &pcTxt[usVal], (size_t)usN);
            s_aacCfgStr[ucRank][usN] = '\0';
        }
    }
    else
    {
        /* 数值（F32/I32）: V 是 `<名>,<type>,<值>[,<选项表>]`, 值窗口 = 到下一个 ',' 或结尾;
         * OK 是 `<名>,<值>`。 */
        uint16_t usVal = (uint16_t)(usNameEnd + 1u);
        uint16_t usEnd;
        float    fV;

        if (usNameEnd >= usLen) { return; }
        if (ucKind == 0u)                      /* V: 跳过 <type> 字段 */
        {
            while ((usVal < usLen) && (pcTxt[usVal] != ',')) { usVal++; }
            if (usVal >= usLen) { return; }
            usVal++;
        }
        usEnd = usVal;
        while ((usEnd < usLen) && (pcTxt[usEnd] != ',')) { usEnd++; }
        if (usEnd == usVal) { return; }
        if (ucRpiCfgParseNum(&pcTxt[usVal], (uint16_t)(usEnd - usVal), &fV) == 0u) { return; }

        if (g_astRpiCfgParam[usIdx].ucType == RPI_CFG_T_I32)
        {
            fV = (fV >= 0.0f) ? (float)((int32_t)(fV + 0.5f)) : (float)((int32_t)(fV - 0.5f));
        }
        s_afCfgVal[usIdx] = fV;
    }

    taskENTER_CRITICAL();
    s_aucCfgSt[usIdx] = (uint8_t)((s_aucCfgSt[usIdx]
                                   | (uint8_t)(RPI_CFG_ST_VALID | RPI_CFG_ST_ASKED))
                                  & (uint8_t)~RPI_CFG_ST_ERR);
    taskEXIT_CRITICAL();
    s_aucCfgRetry[usIdx] = 0u;                 /* 之前的 ERR 只是瞬态: 计数清零 */
    s_aulCfgTick[usIdx] = ulNow;
    s_acCfgErr[0]       = '\0';
}

/* ==================== 「自动下发二维码」指令族（OLED 第 2 页, 2026-10 加） ====================
 * 治的是「台架上调好的调试码, 到了赛场还得在 OLED 上重设一遍」:
 *   上传 = 把当前值写进 Pi 的 web_auto_qr.json 并启用 —— 下一次上电, Pi 握手一成功
 *   就按既有机制推 "QP<ddd>" 调试帧（vRpiTask 里那个分支）, 车上电即是上次的码。
 *   清空 = 本端（OLED 那边做）清回 -1/清人工标记 + Pi 端关开关并清值, 两头一起回原点。
 *
 * 指令（ASCII CMD 帧, 对齐 Pi handler.py::_dispatch_qr_auto; 全部**只发不等**）:
 *   "QA"        查询            → 回 TAG=0x0A "QS,<e>,<值>"
 *   "QE,<0|1>"  开/关           → 回 QS
 *   "QU,<ddd>"  上传并启用      → 回 QS（Pi 侧写文件后, 中途改值监听会立刻推调试帧）
 *   "QC"        清空 Pi 端      → 回 QS
 * 回执里只取开关位（e）: 值字段（三位数 / "---"）本端暂不消费 —— 屏上要看值时,
 * 行 1 显示的就是本端 iGetQrCode(), 而调试帧一推它就会被写上。
 *
 * ⚠️ 状态镜像只有一份（s_ucQrAutoSt）, 三个**写动作都乐观写**（QE 翻转 / QU 上传=启用 /
 *    QC 清空=关 —— 2026-10-18 起, 按下那一拍屏上就变, 别等一个往返）, QS 回执来了用
 *    真相覆盖 —— 两头哪个先到都以回执为准。
 * ⚠️ ucRpiQrAutoState 的过期即归「未知」是**读侧副作用**（读过期值顺手清掉）:
 *    所有读它的调用点（OLED 显示/查询判据）语义一致 —— "0 = 该重新问了"。
 * ⚠️ 只在链路 OK 时发才有意义 —— 本段不重复判链路（调用方 OLED 已判）, 链路不在时
 *    发出去的帧会被 Pi 侧当噪声丢弃（无害, 但查询限频就白转一圈）。 */

#define RPI_QRAUTO_FRESH_MS  10000u   /* QS 回执的镜像新鲜期: 过期 = 「未知」= 该重问 */
#define RPI_QRAUTO_CMD_MAX     10u    /* 最长 "QU,999" + NUL */

static uint8_t  s_ucQrAutoSt   = 0u;  /* 0=未知 1=开 2=关（QS 回执 / 三个写动作乐观写入） */
static uint32_t s_ulQrAutoTick = 0u;  /* 最近一次状态写时刻（新鲜期起点） */
static uint8_t  s_ucQrAutoAnn  = 0u;  /* 最近一次**已播报/已乐观呈现**的值（不随新鲜期过期）——
                                       * QS 调试串只在与它不同时打一条: 10s 一次的过期重查
                                       * 回了原状态不会刷屏, 只有"Pi 说了不一样的"才响 */

static void vRpiQrAutoSend(const char *pcCmd)
{
    vRpiSendFrame(RPI_TYPE_CMD, (const uint8_t *)pcCmd, (uint8_t)strlen(pcCmd));
}

void vRpiQrAutoQuery(void)
{
    vRpiQrAutoSend("QA");
}

void vRpiQrAutoSet(uint8_t ucOn)
{
    char acBuf[RPI_QRAUTO_CMD_MAX];

    (void)snprintf(acBuf, sizeof(acBuf), "QE,%u", (unsigned)((ucOn != 0u) ? 1u : 0u));
    vRpiQrAutoSend(acBuf);

    /* 乐观写镜像: 屏上立刻翻转, QS 回执来了按真相覆盖（Pi 拒掉时它会回原状态）
     * —— 同步「已呈现」值, 回执一致时调试串不响（见 s_ucQrAutoAnn 的注释） */
    s_ucQrAutoSt   = (ucOn != 0u) ? 1u : 2u;
    s_ucQrAutoAnn  = s_ucQrAutoSt;
    s_ulQrAutoTick = (uint32_t)xTaskGetTickCount();
}

void vRpiQrUpload(int32_t iCode)
{
    char acBuf[RPI_QRAUTO_CMD_MAX];
    uint32_t ulV = (uint32_t)(((iCode % 1000) + 1000) % 1000);   /* 折成 0..999, 负数不外泄 */

    (void)snprintf(acBuf, sizeof(acBuf), "QU,%03u", (unsigned)ulV);
    vRpiQrAutoSend(acBuf);        /* 值合法性由 Pi 判（非三位 1~3 回 ERR, 不推调试帧） */

    /* 乐观写镜像: 上传**必启用** —— 按下那一拍「自动下发QR」就翻成 √, 别等 QS 回执
     * （回执迟到/丢了屏上也说得通: Pi 端文件已按启用写, 过期重查一次就归位）。 */
    s_ucQrAutoSt   = 1u;
    s_ulQrAutoTick = (uint32_t)xTaskGetTickCount();
}

void vRpiQrClear(void)
{
    vRpiQrAutoSend("QC");

    /* 乐观写镜像: 清空 = Pi 端关开关（与 QE,0 同一乐观口径） */
    s_ucQrAutoSt   = 2u;
    s_ulQrAutoTick = (uint32_t)xTaskGetTickCount();
}

uint8_t ucRpiQrAutoState(void)
{
    if ((s_ucQrAutoSt != 0u)
        && ((xTaskGetTickCount() - s_ulQrAutoTick) > pdMS_TO_TICKS(RPI_QRAUTO_FRESH_MS)))
    {
        s_ucQrAutoSt = 0u;        /* 过期即归「未知」—— 下次进页面重新问 */
    }
    return s_ucQrAutoSt;
}

/* QS 回执解析（只在 vRpiTask 的 TAG 分支里调, 排在 ucRpiCfgHandleReply **之前**）。
 * 形状 "QS,<e>[,<值>]": e∈{0,1}; 值 = 三位数或 "---"（暂不消费）。
 * 返回 1 = 本帧是 QS、已消费; 0 = 不是（调用方继续喂参数缓存）。
 * ⚠️ 认不出的形状**静默丢**（老 Pi / 手发的）—— 同 ucRpiCfgHandleReply 那条纪律:
 *    这里打日志要走 vRpiSendDebug, 而它每帧一条会把链路刷爆。
 * ⚠️ 唯一的例外是**状态翻转**时打一行（2026-10-18 加）: "× 没显示"那类现场
 *    靠它一眼分辨「回执没到」还是「到了没画」—— 只在变化时打, 10s 一次的
 *    过期重查回了原状态不会刷屏。 */
static uint8_t ucRpiQrAutoReply(const uint8_t *pucData, uint8_t ucDataLen)
{
    const char *pcTxt = (const char *)&pucData[1];
    uint16_t    usLen = (uint16_t)(ucDataLen - 1u);   /* 不含 TAG */
    uint8_t     ucOld;
    char        acDbg[16];

    if (usLen < 4u) { return 0u; }                   /* 最短 "QS,0," */
    if ((pcTxt[0] != 'Q') || (pcTxt[1] != 'S') || (pcTxt[2] != ',')) { return 0u; }
    if ((pcTxt[3] != '0') && (pcTxt[3] != '1')) { return 0u; }

    ucOld          = s_ucQrAutoAnn;
    s_ucQrAutoSt   = (pcTxt[3] == '1') ? 1u : 2u;
    s_ulQrAutoTick = (uint32_t)xTaskGetTickCount();

    if (s_ucQrAutoSt != ucOld)
    {
        s_ucQrAutoAnn = s_ucQrAutoSt;
        (void)snprintf(acDbg, sizeof(acDbg), "QR auto: %s",
                       (s_ucQrAutoSt == 1u) ? "on" : "off");
        vRpiSendDebug(acDbg);
    }
    return 1u;
}
