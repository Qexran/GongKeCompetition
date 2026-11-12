/**
  * @file       app_oled.c
  * @brief      OLED 显示模块：多页带光标菜单（可滚动）+ 状态页。
  *
  * 菜单页一屏只显示 4 行, 但一张表**最多 32 个启用项** —— 放不下的靠 UP/DOWN
  * **滚动窗口**查看（窗口 = s_ucTop + vOledWindowSync）, 右侧画竖直滚动条。
  * 换行时**选中行的文字从右侧滑入**（8px → 0, 约 100ms）—— 开关 = 蓝牙「OLED」组的
  * `oled_menu_anim`（上电初值 = app_config.h 的 OLED_MENU_ANIM_EN）。
  *
 * 页面（LEFT/RIGHT 翻页, **两端夹住不循环**）:
 *   第 1 页 = 状态页（<源>-Yaw / QR / RPI / Time）—— 比赛看的那页; 标签随姿态源变
 *   第 2 页 = 二维码设定（数字编辑 + 功能小菜单: 自动下发 / 上传 / 清空, 见 vOledPageQr）
 *   第 3 页 = 任务菜单（树莓派关机 / 握手让出 / 执行任务→跑预设 14/15/16 / 开关激光）
 *   第 4 页 = 巡线菜单（巡线测试开关 / 拍照占位 / 调参→三个巡线 Beam 量）
  *
  * 三条纪律（都在下面的注释里展开）:
  *   ① 菜单**不能**碰 ucTakeMenuConfirm（那是脚本任务的一次性「继续」键）—— 走
  *      vOledKeyPost/ucTakeOledKeyMask 这条自己的通道; 并且只要不在第 1 页就把
  *      vSetOledMenuBusy(1) 拉高, 让按键任务**别再**喂脚本那份（否则在菜单里选任务
  *      会顺带把停在 STEP_STOP 的路线放走, 车突然动）。
  *   ② 菜单**不直接执行**任何会阻塞或动底盘的事（握手/让出会无限阻塞、跑路线会动底盘）
  *      —— 一律 vScriptPostReq 投给脚本任务, 本任务只负责显示与投递。
  *   ③ 本任务优先级最低(1), 50ms 一拍。任何阻塞在这里的调用 = 整屏停摆 + 操作员唯一的
  *      反馈通道（RPI 链路等）消失。所以这里只做「读 Beam + 画 + 投递」。
  *
  * ⚠️ 全页只在**这里**用浮点格式化（`%.1f` / `%.2f` 显示 ATK-Yaw / 调参值; 2026-10 起
  *    通用调参行又加了 `%.5g`）。工程其它地方为省 printf 体积一律用 bt_fmt_f2, 但本文件
  *    从改之前就在用 %f（见下面状态页）, 已链进固件, 且它能自己把宽度压到最短
  *    （调参行的 128px 预算很紧 —— 值那半截最多 48px）。
  * ⚠️ 中文字模由 tools/fontgen/ 生成进 Core/Src/OLED_Data.c; 没列进字表的字会显示成
  *    「方框问号」默认图形 —— 加菜单文案时记得同步 tools/fontgen/glyphs.txt。
  */

#include "main.h"             /* OLED_SDA_Pin / OLED_SCL_Pin 等引脚宏 */
#include "app_oled.h"
#include "app_beam.h"
#include "app_sensors.h"      /* GYRO_DEV_* / UART_GYRO_*（第 1 页标签按选型查表）*/
#include "app_rpi.h"          /* vRpiShutdownPi（第 3 页） */
#include "app_actuator.h"     /* vRelayOn / vRelayOff / vLaserShoot（第 3 页「开关激光」） */
#include "map.h"              /* SCRIPT_PRESET_n（控制台生成, 见 app_oled_menu.c） */
#include "app_oled_menu.h"    /* 菜单数据表: g_astOledPage / 动作码 / 调参量 */
#include "key_device.h"       /* ucKeyIsPressed / KEY_LEFT / KEY_RIGHT（调参行按住连发） */
#include "OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>            /* vsnprintf（带数字的混排行） */
#include <stdarg.h>
#include <string.h>           /* strlen / memcpy（通用调参行的选项表与 Pi 回值切片） */

/* ==================================================================
 * 屏幕常量
 * ================================================================== */

#define OLED_W        128u   /* 屏宽（驱动里也是硬编码 128, 见 OLED.c 的 aucDisplayBuf[8][128]） */
#define OLED_ROW_H     16u   /* 一行高 = OLED_8X16 的字高; 4 行占满 64px */
#define OLED_HAN_W     16u   /* 一个汉字宽（16x16 字库, 只有这一种尺寸） */
#define OLED_ASC_W      8u   /* 一个 ASCII 宽（OLED_8X16） */

/* 行 y 坐标 */
#define OLED_Y1         0u
#define OLED_Y2        16u
#define OLED_Y3        32u
#define OLED_Y4        48u

/* ==================================================================
 * 页 / 子态
 * ================================================================== */

/* 页 —— 前两条是**代码页**（内容由本文件的 vOledPageStatus / vOledPageQr 画）,
 * 其余是**菜单页**（内容来自 app_oled_menu.c 的数据表）。
 * ⚠️ 「0 = 状态页、1 = 二维码页」是**跨文件契约**: 页面表由控制台生成, 它保证这两条
 *    永远在最前面且只读。页数走 `g_ucOledPageNum`（与页面表同源派生）, 别再写死。 */
#define OLED_PAGE_STATUS 0u   /* 第 1 页: 状态页 */
#define OLED_PAGE_QR     1u   /* 第 2 页: 二维码设定 */

/* 子态（各页共用编号, 语义由页决定） */
#define OLED_MODE_NAV    0u   /* 顶层列表: UP/DOWN 选, MAIN 进, LEFT/RIGHT 翻页 */
#define OLED_MODE_EDIT   1u   /* 第 2 页: 编辑数字位 */
#define OLED_MODE_CONFIRM 2u  /* 「是/否」确认框（页无关, 在 vOledTask 里统一处理）*/
#define OLED_MODE_SUBMENU 3u  /* 进了某一项的子菜单: 内容看 s_pstSubList */

/* ⚠️ 屏只有 4 行（64px / 16px）—— 驱动的 vOledShowImage 在 Y > 63 **直接 return**,
 * 第 5 行起静默不画。所以**一屏同时只能看到 4 行**; 放不下的靠 UP/DOWN 滚动
 * （窗口 = s_ucTop + vOledWindowSync, 见 vOledMenuDraw 上方那一段）。 */
#define OLED_MAX_ROWS    4u     /* 一屏同时可见的行数（= 窗口高度）—— **不再是表的容量上限** */

/* 一页 / 一个子菜单最多几个**启用项**（控制台同步拦; 这是固件侧的防御性兜底）。
 * 第 33 项起既看不见也选不到 —— 与控制台的 32 项上限同一条口径。
 * ⚠️ 上限受滚动条几何约束: 条目数最多 33（32 项 + 子菜单那条「返回」）, 此时滑块高
 *    256/33 = 7px, 仍在 [5, 64] 内; 再往上抬要先确认滑块不会细到看不见。 */
#define OLED_MAX_ITEMS   32u

/* 右侧竖直滚动条预留（最右 8px）。菜单标签的宽度预算因此从 128px 降到 120px
 * （见 tools/webconsole/oled_menu.py 的 LABEL_PX）。 */
#define OLED_SCROLLBAR_W 8u
#define OLED_LABEL_W     (OLED_W - OLED_SCROLLBAR_W)   /* 120: 文字实际能用的宽度 */
#define OLED_BAR_X       122u                          /* 轨道左沿（再往右留 2px 气隙） */
#define OLED_BAR_W       4u                            /* 轨道宽 */
#define OLED_BAR_H       (OLED_MAX_ROWS * OLED_ROW_H)  /* 64: 轨道高 = 一屏 */

/* 子菜单里「返回」那一行的光标哨兵（**不是表下标** —— 表下标一定 < 256） */
#define OLED_CURSOR_BACK 0xFFu

/* 换行动画: 上下选择时, **选中那一行的文字从右侧滑入**（横向 8px → 0）。
 * 开关 = 蓝牙「OLED」组的 oled_menu_anim（上电初值 = app_config.h 的 OLED_MENU_ANIM_EN）。
 * ⚠️ 只动**选中行**、只动**文字** —— 刻意不做"整屏滚动滑动": 屏 4 行 × 16px, 而驱动
 *    画不了 y<0 的内容（vOledShowImage 在 Y>63 直接 return）, 整屏滑会在顶边留细黑边。
 * ⚠️ 偏移上限 8px 的依据: 菜单标签预算 120px, 120+8 = 128 正好铺满一行, 再大就会在动画
 *    的头 1~2 帧里把长标签从尾部截掉（见 vOledDrawRowX）。 */
#define OLED_ANIM_X0     8u    /* 起始偏移 [px] */
#define OLED_ANIM_STEP   4u    /* 每拍收回 [px] → 8px 用 2 拍 ≈ 100ms（本任务 50ms 一拍） */

/* 执行一项之后的去向（ucOledRunAct 的返回值）—— 把散落的 s_ucMode 赋值收成一处。
 * ⚠️ ENTER_CONFIRM 时调用方必须**立刻 return**: vOledEnterConfirm 已经把 mode 设好了。*/
#define OLED_RES_STAY           0u
#define OLED_RES_ENTER_SUB      1u
#define OLED_RES_ENTER_CONFIRM  2u

/* OLED_MODE_CONFIRM 的用途 —— 前两种是「是/否」两选一（按「是」之后干的事不同）,
 * 第三种是**单键提示框**（只有 [返回]）。
 * 提示语与「是」的动作都由 s_ucConfirmKind 分派（见 CONFIRM 分支与画那段）。 */
#define OLED_CONFIRM_SHUTDOWN 0u  /* 关机: 两帧 sd（第 3 页）*/
#define OLED_CONFIRM_NO_QR    1u  /* 还没设二维码: 「是」= 跳到第 2 页去设
                                   * （第 4 页「执行任务」+ 第 5 页「射靶测试」会弹;
                                   *   「开始巡线测试」2026-10 起不再要二维码, 改查 Pi 在线）*/
#define OLED_CONFIRM_NO_LINK  2u  /* **发送失败: 未连接到树莓派** —— 第 3 页关机时链路不是 OK,
                                   * 一帧都不发, 改弹这个（见 ucOledConfirmKeys 里那道门）。
                                   * ⚠️ 这是**单键**提示框: 没有是/否, 任何键的语义都只是 [返回]。*/
#define OLED_CONFIRM_NO_PI    3u  /* **前置条件: 未连接到树莓派** —— 「执行任务」「开始巡线测试」
                                   * 都要 Pi 在线（任务内部第一步就是视觉步骤, Pi 不在 = 跑到那
                                   * 一步失败卡死）。与 NO_LINK 同为单键提示框, 差别只在**不画**
                                   * 「发送失败:」那行 —— 还没开始发, 谈不上"发送失败"。 */

/* 二维码算「已设置」的判据 —— **不是**「!=-1」: 与 c_ucCondQr / 对位链路
 * 同一个 100~399 口径（三位数的含义是 百位=目标色 + 后两位序号）。
 * 扫码失败、从没扫过（-1）、旧 Pi 的越界值统统算「没设置」。 */
#define OLED_QR_IS_SET(iQr)   (((iQr) >= 100) && ((iQr) <= 399))

/* 瞬态提示的显示时长 */
#define OLED_QR_OK_MS     2000u   /* 第 2 页退出编辑后的 "OK!" */
#define OLED_HS_MSG_MS    1000u   /* 第 3 页「让出成功!」 */
#define OLED_LINE_START_GRACE_MS 500u  /* 巡线测试启动宽限（见 ucOledLineTestOn） */
#define OLED_SHOOT_START_GRACE_MS 500u /* 射靶测试启动宽限（见 ucOledShootTestOn） */
#define OLED_SHOOT_OK_MS     1000u     /* 「停止射靶 OK!」的 1s 窗口（看**时刻**不看事件, 同拍照那条） */

/* 「射靶保持」环的状态码 —— 与 app_rpi.c 的 `AIM_STATE_*` / Beam `shoot_state` 一一对应。
 * 这里**只读**（标签靠它决定四个态）; ⚠️ 改 app_rpi.c 那边记得同步这三个数。 */
#define OLED_SHOOT_ST_COARSE  1u   /* 粗调/等停稳前戏 */
#define OLED_SHOOT_ST_MOVING  2u   /* 本拍真发了修正 */
#define OLED_SHOOT_ST_SETTLED 3u   /* 连续 N 拍在靶 */
#define OLED_SHOOT_ST_FROZEN  4u   /* 撞行程闸/noprog → 停车冻结, 等操作员再按一格 */
#define OLED_SHOOT_ST_NOTGT   5u   /* 流活着但本帧没有目标色（丢靶/被半径门全剔除） */
#define OLED_SHOOT_ST_STALE   6u   /* 帧计数停涨 = Pi 没在发（掉线/流线程 12s 到点退了） */

/* 「瞄准列」每格的步长 [px]。⚠️ 它**必须大于** tc_conv_px（默认 4.5）—— 否则按一下还在
 * 死区里、车纹丝不动, 操作员会以为"调了没用"（这正是这里取 5 而不是 2 的原因: 2px 步进
 * 比流帧检测噪声带 ±2px 还小, 在那上面做分辨率没有意义）。 */
#define OLED_SHOOT_CX_STEP_PX 5.0f
#define OLED_PHOTO_MSG_MS 1000u        /* 「拍照 成功!/失败!」显示 1s, 之后回「拍照」*/

/* ==================================================================
 * 模块内状态
 * ================================================================== */

static uint8_t  s_ucPage   = OLED_PAGE_STATUS;
static uint8_t  s_ucMode   = OLED_MODE_NAV;
/* 光标 —— 一词多义是**有意**的: 菜单项的表下标 / 第 2 页的数字位 / 确认框 0或1。
 * ⚠️ 子菜单里还可能是哨兵 OLED_CURSOR_BACK（表示停在固件的「返回」那一行）。
 * ⚠️ 别把菜单那套 step 助手用到 QR / 确认框上（那边是定长 3 位 / 二选一）。 */
static uint8_t  s_ucCursor = 0u;

/* 当前子菜单是哪张表（只在 OLED_MODE_SUBMENU 下有效）。
 * 顶层菜单的表**不存这里** —— 由 s_ucPage 现查（pstOledPageList）, 少一份要对齐的状态。 */
static const stOledListTdf *s_pstSubList = 0;

/* 滚动窗口的**顶行**对应的**条目序号**（不是表下标!）—— 只在菜单页有效。
 * 条目序号 = 「启用项按表序数第几个」, 子菜单的最后一条是固件的「返回」。
 * ⚠️ 它**不是真相, 只是个缓存**: 每拍画之前都由 vOledWindowSync 重算/钳位,
 *    所以任何地方漏改它都不会画错（最多晚 50ms 修好）。 */
static uint8_t  s_ucTop = 0u;

/* 换行动画: 选中行文字的**当前横向偏移** [px]（0 = 没有动画在跑）。
 * 由 vOledAnimStart 起跳、vOledAnimTick 每拍收回, 画的时候只作用在选中那一行。 */
static uint8_t  s_ucAnimOfs = 0u;

/* 前向声明 —— 翻页要用到这两个, 它们的定义在下面「菜单页 —— 通用引擎」那一段 */
static const stOledListTdf *pstOledPageList(uint8_t ucPage);
static void vOledCursorNormalize(const stOledListTdf *pstList);
/* 滚动条几何（菜单引擎与第 2 页小菜单共用; 定义在「菜单页」段末尾） */
static void vOledScrollbarDrawGeo(uint8_t ucN, uint8_t ucRows, uint8_t ucTop,
                                  uint8_t ucY0, uint8_t ucH);

/* 第 2 页: 二维码编辑缓冲（每位 1~3, 与 STEP_QR_SET_TEST 的防呆同域） */
static uint8_t  s_aucQrDig[3] = { 1u, 1u, 1u };
static uint32_t s_ulQrOkTick  = 0u;   /* 退出编辑的时刻（显示 "OK!"） */

/* 第 3 页: 关机确认的「第一帧已发」标志（Pi 的防误触门分两次操作, 见 CONFIRM 分支） */
static uint8_t  s_bShutdownArmed = 0u;

/* 确认框当前是哪一种用途（OLED_CONFIRM_*）—— 进框时设, 画与「是」的动作都看它 */
static uint8_t  s_ucConfirmKind = OLED_CONFIRM_SHUTDOWN;

/* 进「是/否」确认框的统一入口（把 mode 与用途一起设好, 免得漏设 kind 拿到上一次的） */
static void vOledEnterConfirm(uint8_t ucKind)
{
    s_ucConfirmKind  = ucKind;
    s_ucMode         = OLED_MODE_CONFIRM;
    s_ucCursor       = 0u;      /* 默认停在「是」 */
    s_bShutdownArmed = 0u;      /* 关机那两帧的计数对齐到「还没发第一帧」 */
}

/* 第 4 页: 巡线测试开关。⚠️ 它不是「本页自己的真相」—— 每拍由 ucOledLineTestOn()
 * 拿设备真值对齐（ucScriptLineTestActive）, 这里只是个防抖的影子（见那个函数的注释）。 */
static uint8_t  s_bLineTestOn  = 0u;
static uint32_t s_ulLineTestTick = 0u;

/* 「射靶测试」开关的影子 —— 同 s_bLineTestOn 的性质: 每拍由 ucOledShootTestOn() 拿**设备真值**
 * 对齐, 这里只当"投递成功 → 脚本真开跑"那 ≤50ms 空窗的防抖（见那条函数的注释）。 */
static uint8_t  s_bShootTestOn    = 0u;
static uint32_t s_ulShootTestTick = 0u;
/* 「已投递、还没看到预设真跑起来」——**专门覆盖首次跑前面的那套"机械臂待命展开"**
 * （阻塞好几秒）。详见 ucOledShootTestOn 的注释: 没有它, 那几秒里标签会弹回「开始」。 */
static uint8_t  s_bShootPending   = 0u;

void vAppOledInit(void)
{
    stOledStaticParamTdf stInit;
    stInit.pstSclGpioPort = GPIOF;   /* GPIOF */
    stInit.usSclPin       = GPIO_PIN_1;         /* GPIO_PIN_1 */
    stInit.pstSdaGpioPort = GPIOF;   /* GPIOF */
    stInit.usSdaPin       = GPIO_PIN_0;         /* GPIO_PIN_0 */
    vOledDeviceInit(&stInit, OLED0);
}

/* ==================================================================
 * 绘制助手
 * ================================================================== */

/**
  * @brief       画一行「汉字 + ASCII 混排」文本
  * @param       ucX/ucY  左上角
  * @param       pcStr    UTF-8 串（汉字 16px 宽, ASCII 8px 宽）
  * @retval      画完后的 x（下一个可落点）
  *
  * 汉字走 vOledShowChinese（16x16, 自带清底）, ASCII 走 vOledShowChar（8x16, **不清底**）
  * —— 所以调用方每行之前要自己 vOledClearArea 一次（沿用状态页的老做法防残影）。
  * ⚠️ 超宽**直接截断**不折行（宁可少画几个字, 也别把两行叠在一起）。
  * ⚠️ 逐字交给 vOledShowChinese: 它按 '\0' 收尾, 直接给中段指针会把
  *    后面整串都画出来（而且 x 步进也会全错）。
  * ⚠️⚠️ 非 ASCII 的**字节数按 UTF-8 首字节判**（2026-11-01 修）, 不能再一律取
  *    `OLED_CHN_CHAR_WIDTH`(3): 汉字是 3 字节, 但界面里还有 2 字节的符号
  *    （× U+00D7 / Δ U+0394）。一律取 3 会把后面那个 ASCII 一起吞进查找键
  *    （"Δx" → 键 "Δx" 查不到, 画默认方框且 x 消失; 配合字库 >255 条时
  *    查表下标 uint8_t 回绕 → 整屏挂死, 见 `vOledShowChinese` 那条注释）。
  */
static uint8_t ucOledDrawMixed(uint8_t ucX, uint8_t ucY, const char *pcStr)
{
    const char *p = pcStr;

    while ((*p != '\0') && (ucX < OLED_W))
    {
        if ((unsigned char)(*p) >= 0x80u)
        {
            char    acOne[OLED_CHN_CHAR_WIDTH + 2];   /* 4 字节 UTF-8 + NUL */
            uint8_t ucLen;
            uint8_t i;

            if      (((unsigned char)p[0] & 0xF8u) == 0xF0u) { ucLen = 4u; }
            else if (((unsigned char)p[0] & 0xF0u) == 0xE0u) { ucLen = 3u; }
            else if (((unsigned char)p[0] & 0xE0u) == 0xC0u) { ucLen = 2u; }
            else                                             { ucLen = 1u; } /* 非法首字节 */

            for (i = 0u; i < ucLen; i++)
            {
                acOne[i] = p[i];
                if (p[i] == '\0') { break; }
            }
            acOne[i] = '\0';
            if (i == 0u) { break; }              /* 兜底, 不该发生 */

            vOledShowChinese(OLED0, ucX, ucY, acOne);
            ucX = (uint8_t)(ucX + OLED_HAN_W);
            p  += i;
        }
        else
        {
            vOledShowChar(OLED0, ucX, ucY, OLED_8X16, *p);
            ucX = (uint8_t)(ucX + OLED_ASC_W);
            p++;
        }
    }

    return ucX;
}

/* 清整行再画: 汉字/ASCII 混排时长度会变, 不清就会留上一帧的残影
 * （ASCII 的 vOledShowChar **不清底**, 只有汉字走 vOledShowImage 会清自己那 16px）。
 * ucXOfs = 文字的横向起点（换行动画用; 其它调用点一律 0）。
 * ⚠️ 清的是**整行 128px**（不是 120px）: 那正是擦掉上一条滚动条的那只手, 别改窄。
 * ⚠️ ucXOfs 最大只到 OLED_ANIM_X0(8): 标签预算 120px, 120+8 = 128 正好铺满一行 ——
 *    再大就会在动画的头 1~2 帧里把长标签从尾部静默截掉（ucOledDrawMixed 到 128 就停）。 */
static void vOledDrawRowX(uint8_t ucY, uint8_t ucXOfs, const char *pcStr)
{
    vOledClearArea(OLED0, 0, ucY, OLED_W, OLED_ROW_H);
    (void)ucOledDrawMixed(ucXOfs, ucY, pcStr);
}

static void vOledDrawRow(uint8_t ucY, const char *pcStr)
{
    vOledDrawRowX(ucY, 0u, pcStr);
}

/* 清整行再画（格式化版） */
static void vOledDrawRowf(uint8_t ucY, const char *pcFmt, ...)
{
    char    acBuf[44];
    va_list arg;

    va_start(arg, pcFmt);
    (void)vsnprintf(acBuf, sizeof(acBuf), pcFmt, arg);
    va_end(arg);

    vOledDrawRowX(ucY, 0u, acBuf);
}

/* 清整行再画（格式化版 + 横向偏移 —— 换行动画用）。
 * ⚠️ 与 vOledDrawRowf 是**两个函数**（变参转发不出去）—— 别把偏移并进原来那个, 那要给
 *    4 个调用点都补一个恒 0 的实参, 白白碰到二维码页。 */
static void vOledDrawRowfX(uint8_t ucY, uint8_t ucXOfs, const char *pcFmt, ...)
{
    char    acBuf[44];
    va_list arg;

    va_start(arg, pcFmt);
    (void)vsnprintf(acBuf, sizeof(acBuf), pcFmt, arg);
    va_end(arg);

    vOledDrawRowX(ucY, ucXOfs, acBuf);
}

/* 反色高亮整行（菜单选中项）—— ⚠️ 必须在**画完文字之后**再调:
 * vOledReverseArea 是对显存做 XOR, 先反色再写字就只剩文字被反色了。
 * ⚠️ ucW: 有滚动条时必须只反色到 OLED_LABEL_W —— 否则反色会把最右那 8 列刷白,
 *    随后画上去的滑块就成了"白底白块"、整条看不见（正是选中行在看的那个位置）。 */
static void vOledHighlightRow(uint8_t ucY, uint8_t ucW)
{
    vOledReverseArea(OLED0, 0, ucY, ucW, OLED_ROW_H);
}

/* tick 起算、到现在还没超过 ulMs 毫秒？（ulStartTick==0 = 从没发生过） */
static uint8_t ucOledWithin(uint32_t ulStartTick, uint32_t ulMs)
{
    if (ulStartTick == 0u) { return 0u; }
    return (uint8_t)((xTaskGetTickCount() - ulStartTick) < pdMS_TO_TICKS(ulMs));
}

/* ⚠️ 本任务**不蜂鸣** —— 菜单的反馈全靠视觉（标签变化 / 横幅 / 提示框 / "OK!"）,
 * 与菜单引擎的既有惯例一致。别在这里加 vBuzzerBeep（它内部 vTaskDelay, 而本任务
 * 50ms 一拍禁阻塞; 2026-10-18 加过一版非阻塞蜂鸣, 按用户口径撤了）。 */

/* ==================================================================
 * 翻页
 * ================================================================== */

/**
  * @brief       处理 LEFT/RIGHT 翻页
  * @retval      1 = 本轮已翻页（调用方别再处理本页按键）
  *
  * ⚠️ 只在**页面顶层**（OLED_MODE_NAV）调用 —— 编辑/确认/子菜单里那两个键是页内用的
  *    （数字位移位、调参数值、退回上级）。
  * ⚠️ **两端夹住, 不做循环**: 第 1 页 LEFT 无作用、第 4 页 RIGHT 无作用（按需求）。
  */
static uint8_t ucOledTurnPage(uint8_t ucKeys)
{
    uint8_t bTurn = 0u;

    if ((ucKeys & OLED_KEY_LEFT) != 0u)
    {
        if (s_ucPage > OLED_PAGE_STATUS) { s_ucPage--; bTurn = 1u; }
    }
    else if ((ucKeys & OLED_KEY_RIGHT) != 0u)
    {
        /* ⚠️ 页数是**动态**的（控制台能增删菜单页）→ 用 g_ucOledPageNum。
         *    先判非零再做减法: 无符号下溢会把「第 1 页」判成还能右翻。 */
        if ((g_ucOledPageNum > 0u) && (s_ucPage < (uint8_t)(g_ucOledPageNum - 1u)))
        {
            s_ucPage++;
            bTurn = 1u;
        }
    }

    if (bTurn != 0u)
    {
        /* 翻页是**唯一的全局重置点**: 回顶层、丢掉子菜单、光标落到新页第一个可用项 */
        s_ucMode     = OLED_MODE_NAV;
        s_ucCursor   = 0u;
        s_ucTop      = 0u;               /* 新页窗口从头开始（vOledWindowSync 本会自愈） */
        s_ucAnimOfs  = 0u;               /* 换页时丢掉半截的滑入动画, 别在新页上续播 */
        s_pstSubList = 0;
        vOledCursorNormalize(pstOledPageList(s_ucPage));
    }
    return bTurn;
}

/* ==================================================================
 * 提示框 —— 「是/否」确认框（第 3 页关机 / 第 4 页还没设二维码）
 *         + 单键提示框 [返回]（第 3 页关机时链路不是 OK: 发送失败）
 *
 * ⚠️ 它**不按页分派**: 用途存在 s_ucConfirmKind 里, 由进框时的 vOledEnterConfirm 设好。
 *    所以 vOledTask 在分派页之前就先把它处理掉并 continue —— 两种页的确认框只有
 *    提示语不同, 布局（第 3 行提示 + 第 4 行 是/否）完全一样;
 *    单键那种是「第 2/3 行两句话 + 第 4 行 [返回]」（见 vOledConfirmDraw）。
 * ================================================================== */

/* 返回 1 = 已经跳到别的页（调用方本拍别再画了） */
static uint8_t ucOledConfirmKeys(uint8_t ucKeys)
{
    /* ---- 单键提示框（OLED_CONFIRM_NO_LINK）: 只有 [返回] ----
     * ⚠️ 必须在下面那套「是/否」之前处理干净并 return: NO_LINK 没有两选一, 落进下面
     *    就会把 LEFT/RIGHT 当成「挪光标」、MAIN 当成「按是」（而那时 s_ucCursor=0 = 是）。
     * ⚠️ 关掉它时**返回 1**（= 本拍已跳到别的页, 调用方别再画）: 返回 0 的话这一拍
     *    会拿已经变回 NAV 的 mode 去画提示框, 到下一拍才切回菜单。
     * ⚠️ LEFT 也认（与「执行任务」子菜单那条 LEFT=退回上级同习惯）—— 这个框里 LEFT
     *    本来没有别的用途。 */
    if (s_ucConfirmKind == OLED_CONFIRM_NO_LINK)
    {
        if ((ucKeys & (OLED_KEY_MAIN | OLED_KEY_LEFT)) != 0u)
        {
            s_ucMode   = OLED_MODE_NAV;
            s_ucCursor = 0u;
            return 1u;
        }
        return 0u;
    }

    /* ---- 单键提示框（OLED_CONFIRM_NO_PI）: 与 NO_LINK 同一套键语义 ----
     * 唯一差别在**画**（不画「发送失败:」那行, 见 vOledConfirmDraw）。 */
    if (s_ucConfirmKind == OLED_CONFIRM_NO_PI)
    {
        if ((ucKeys & (OLED_KEY_MAIN | OLED_KEY_LEFT)) != 0u)
        {
            s_ucMode   = OLED_MODE_NAV;
            s_ucCursor = 0u;
            return 1u;
        }
        return 0u;
    }

    /* ⚠️ 这一行的 LEFT/RIGHT 是**选择**不是「退回上级」, 所以不像别的子页那样把 LEFT
     *    当返回用 —— 要取消就选「否」按 MAIN。 */
    if ((ucKeys & OLED_KEY_LEFT) != 0u)
    {
        s_ucCursor = 0u;                 /* 是 */
    }
    else if ((ucKeys & OLED_KEY_RIGHT) != 0u)
    {
        s_ucCursor = 1u;                 /* 否 */
        s_bShutdownArmed = 0u;           /* 挪开了就当放弃, 免得下次回来直接发第二帧 */
    }
    else if ((ucKeys & OLED_KEY_MAIN) != 0u)
    {
        if (s_ucCursor == 1u)
        {
            s_bShutdownArmed = 0u;       /* 否: 干净地取消 */
            s_ucMode   = OLED_MODE_NAV;
            s_ucCursor = 0u;
        }
        else if (s_ucConfirmKind == OLED_CONFIRM_NO_QR)
        {
            /* 「还没设二维码」→ 是 = **跳到第 2 页**去设。
             * ⚠️ 只跳页、**不顺便进编辑态**: 进编辑就得有个「当前位」被反色选中,
             *    操作员一眼看到反色可能随手就改了 —— 让他自己按 MAIN 进编辑更稳。
             * ⚠️ 跳页后让调用方立刻收手: 与翻页同一套, 否则本拍还会按旧页画一帧。 */
            s_ucPage   = OLED_PAGE_QR;
            s_ucMode   = OLED_MODE_NAV;
            s_ucCursor = 0u;
            return 1u;
        }
        else if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            /* ---- 链路不是 OK: 这一帧发给谁? ----
             * WAITING/DISCONNECTED 时 Pi 根本没在听, 而 vRpiShutdownPi 只发不等回应
             * （字节会照常推上线）→ 静默按下去操作员会以为关成了, 车却一直开着,
             * 现场再查就是「按了没反应」。所以**发之前先看链路**: 不是 OK 就一帧都不发,
             * 改弹「发送失败: 未连接到树莓派」。
             * ⚠️ 判据就是第 1 页 RPI 那行用的同一个量（emGetRpiLinkState）, 别再另记一个
             *    影子状态; WAITING(0) 与 DISCONNECTED(2) **都算没连上**（只有 OK 才发）。
             * ⚠️ 走 vOledEnterConfirm 而不是直接赋 s_ucMode —— 它顺带把 s_bShutdownArmed
             *    清零: 这次一帧都没发出去, 不能算「第一帧已发」; 不然链路恢复后按一次
             *    「是」就会把第二帧直接发出去（= 少一道确认, 而那是不可逆动作）。 */
            vOledEnterConfirm(OLED_CONFIRM_NO_LINK);
        }
        else if (s_bShutdownArmed == 0u)
        {
            /* ---- 关机第一步 ----
             * 关机的两帧都在**这个确认框里**发, 而且分两次 MAIN:
             *   第 1 次按「是」→ 发第一帧（Pi 登记）+ 提示变成「再按一次」;
             *   第 2 次按「是」→ 发第二帧（Pi 执行）。
             * ⚠️⚠️ 为什么不「进确认框就发第一帧」（那样只要按两次 MAIN, 更省事）:
             *    进框就发的话, 「进框 → 选否 → 10s 内又进框」= 两帧落在 Pi 的确认窗口里
             *    → **没按过「是」也会真关机**。Pi 只认帧数, 本端发出去的帧收不回来
             *    （没有「撤销」指令）, 所以只能**不提前发**。对一个不可逆的动作,
             *    多按一次 = 多一道确认, 值。
             * vRpiShutdownPi 只发不等回应（非阻塞）, 在 OLED 任务里调是安全的。 */
            vRpiShutdownPi();
            s_bShutdownArmed = 1u;
            s_ucCursor = 0u;
            /* 留在确认框里等第二次 MAIN —— 不退出, 否则操作员不知道还差一次。 */
        }
        else
        {
            /* 第二帧 → Pi 两道门都过 = 真关机。之后心跳会停, 8s 后链路自动判
             * DISCONNECTED —— 那是预期现象。 */
            vRpiShutdownPi();
            s_bShutdownArmed = 0u;
            s_ucMode   = OLED_MODE_NAV;
            s_ucCursor = 0u;
        }
    }
    return 0u;
}

static void vOledConfirmDraw(void)
{
    vOledClearArea(OLED0, 0, OLED_Y1, OLED_W, OLED_ROW_H);
    vOledClearArea(OLED0, 0, OLED_Y2, OLED_W, OLED_ROW_H);

    /* ---- 单键提示框: 三行纯文字, 第 1 行**故意留空** ----
     * 留空是为了和「是/否」那套的版式对上（提示语都在屏幕下半部, 视线不用上下跳）。
     * ⚠️ 宽度都贴着一行 128px 的预算算过: 「未连接到树莓派」7 个汉字 = 112px,
     *    「发送失败:」= 4 汉字 + ':' = 72px, 「[返回]」= 48px（超了会被**静默截掉**）。 */
    if ((s_ucConfirmKind == OLED_CONFIRM_NO_LINK) || (s_ucConfirmKind == OLED_CONFIRM_NO_PI))
    {
        if (s_ucConfirmKind == OLED_CONFIRM_NO_LINK)
        {
            vOledDrawRow(OLED_Y2, "发送失败:");
        }
        else
        {
            /* NO_PI 是**前置条件**（还没开始发, 谈不上"发送失败"）→ 第 2 行留空,
             * 提示语直接落在第 3 行 —— 与 NO_LINK 同一个下半屏版式。 */
            vOledClearArea(OLED0, 0, OLED_Y2, OLED_W, OLED_ROW_H);
        }
        vOledDrawRow(OLED_Y3, "未连接到树莓派");
        vOledDrawRow(OLED_Y4, "[返回]");
        return;
    }

    /* 第 3 行: 提示语按用途分派。
     * 关机时发完第一帧要换提示 —— 操作员得知道「还差一次」, 否则按一下没反应就走了
     * （而那一下其实已经在 Pi 那边登记了）。 */
    if (s_ucConfirmKind == OLED_CONFIRM_NO_QR)
    {
        /* ⚠️ `还未设置二维码数字！` 是 9 个汉字 = 144px, **超一行 128px 会被静默截掉**,
         *    所以压成 6 个汉字 + '!'（104px）。 */
        vOledDrawRow(OLED_Y3, "还未设二维码!");
    }
    else
    {
        vOledDrawRow(OLED_Y3, (s_bShutdownArmed != 0u) ? "再按一次" : "是否关机?");
    }

    /* ⚠️ 这一行**不画固定的 `>` 标记** —— 选中态完全靠反色表示。
     *    画了固定标记的话, 选到「否」时那个 `>` 还杵在「是」旁边, 看着像「是」仍被选中。
     *    所以是/否两个汉字的 x 坐标就是字符格算出来的 0 和 32（"是  否" = 是 + 2 空格 + 否）。 */
    vOledDrawRow(OLED_Y4, "是  否");
    vOledReverseArea(OLED0,
                     (s_ucCursor == 0u) ? 0u : (uint8_t)(OLED_HAN_W + (2u * OLED_ASC_W)),
                     OLED_Y4, OLED_HAN_W, OLED_ROW_H);
}

/* ==================================================================
 * 第 1 页: 状态页（第 4 行 = 健康检查结果 / 秒表, 见下方第 4 行绘制）
 * ================================================================== */

static void vOledPageStatus(uint8_t ucKeys)
{
    /* RPI 链路状态显示名（与 emRpiLinkStateTdf 枚举值对应, 见 app_beam.h） */
    static const char *s_apcLinkName[3] = { "Waiting", "OK", "Disconnected" };
    float   fAttVal = fGetAttitudeYaw();
    emRpiLinkStateTdf emLink = emGetRpiLinkState();

    if (emLink > RPI_LINK_DISCONNECTED) {
        emLink = RPI_LINK_WAITING;
    }

    /* LEFT 在这页无作用（不做循环）; RIGHT 翻到第 2 页 */
    (void)ucOledTurnPage(ucKeys);

    vOledClearArea(OLED0, 0, OLED_Y1, OLED_W, OLED_ROW_H);
    /* 标签跟着 **实际在用的那一颗**（有效源）走, 设备名按**该口的选型**查表 ——
     * 全部纯 ASCII, 不需要新字模。
     * ⚠️ 用有效源而不是首选 gyro_src: 热备自动切换接管后, 屏幕上显示的必须是"现在真的
     *    在喂数据的这颗"。末尾的 `!` = 有效源 ≠ 首选（自动切换把它换掉了）——与 QR 的
     *    `*` 同一套"一眼看出状态"的路子。
     * ⚠️ 宽度: 最长 `ATK!-Yaw=-180.00` = 16 字符 = 128px, 正好一行（预算就是 128px）。 */
    {
        static const char *s_apcGyroDevName[3] = { "ATK", "M0", "HWT" };
        uint8_t ucEff = ucGetGyroSrcEff();
        uint8_t ucDev = (ucEff != 0u) ? (uint8_t)GYRO_SUB_DEV : (uint8_t)GYRO_MAIN_DEV;

        if (ucDev > 2u) { ucDev = 2u; }     /* 兜底（app_sensors.c 有编译期 #error） */

        vOledPrintf(OLED0, 0, OLED_Y1, OLED_8X16, "%s%s-Yaw=%.2f",
                    s_apcGyroDevName[ucDev],
                    (ucEff != ucGetGyroSrc()) ? "!" : "",
                    fAttVal);
    }

    /* 第 2 行: 二维码结果。⚠️ 尾缀一个 `*` = **人工预设的**（不是扫来的）——
     * 那是「扫码步骤被跳过」唯一的现场可见性: 台架演练完忘了重新上电时,
     * 比赛时会看到 `QR:111*` 一直不动, 一眼就知道不是相机/二维码的问题。
     * 星号只在人工预设时出现, 比赛（扫来的值）显示与改动前完全一样。 */
    vOledClearArea(OLED0, 0, OLED_Y2, OLED_W, OLED_ROW_H);
    if (ucGetQrManual() != 0u)
    {
        vOledPrintf(OLED0, 0, OLED_Y2, OLED_8X16, "QR:%d*", iGetQrCode());
    }
    else
    {
        vOledPrintf(OLED0, 0, OLED_Y2, OLED_8X16, "QR:%d", iGetQrCode());
    }

    /* RPI 握手/链路状态: Waiting / OK / Disconnected（重连握手后回 OK）。
     * 先 ClearArea 清行再写，防短词("OK")残留长词("Disconnected")残影。 */
    vOledClearArea(OLED0, 0, OLED_Y3, OLED_W, OLED_ROW_H);
    vOledPrintf(OLED0, 0, OLED_Y3, OLED_8X16, "RPI:%s", s_apcLinkName[(int)emLink]);

    /* 第 4 行（y=48）：平时是状态页秒表（STEP_TIMER / OLED「计时」控制）;
     * 握手时勾了「健康检查」→ 显示四路相机体检结果 `H:M√Q√Pi√MV√`。
     *   M=主摄 / Q=扫码副摄 / Pi=CSI / MV=OpenMV; √=通过 ×=失败 ?=检查中/未检查。
     * 让位规则（用户 2026-11 定）: 秒表**首次启动前**一直显示体检结果, 首次启动后
     * 永久让位给秒表（停止/重置都不回来 —— 比赛结束要看最终用时）。
     * 长度: `H:`(24px) + M/Q/√(24px×2) + Pi/MV/√(32px×2) = 128px, 正好一行;
     *       `?` 是 ASCII, 只会更短。
     * ⚠️ 用 ucOledDrawMixed 画（不是 vOledPrintf）: √/× 是 16px 汉字字模,
     *    ASCII 那条路会把它们写成 '?'。 */
    vOledClearArea(OLED0, 0, OLED_Y4, OLED_W, OLED_ROW_H);
    {
        uint8_t ucM  = RPI_HEALTH_UNKNOWN;
        uint8_t ucQ  = RPI_HEALTH_UNKNOWN;
        uint8_t ucPi = RPI_HEALTH_UNKNOWN;
        uint8_t ucMv = RPI_HEALTH_UNKNOWN;
        uint8_t ucBusy;
        uint8_t ucShowHealth;

        vGetHealthResult(&ucM, &ucQ, &ucPi, &ucMv);
        ucBusy = ucGetHealthBusy();

        /* 秒表还没首次启动过, 且正在检查或已有结果 → 显示体检行;
         * 秒表一旦启动过就永久让位（用户 2026-11 定: 比赛结束要看最终用时）。 */
        ucShowHealth = 0u;
        if (ucTimerEverStarted() == 0u)
        {
            if ((ucBusy != 0u)
                || (ucM != RPI_HEALTH_UNKNOWN) || (ucQ != RPI_HEALTH_UNKNOWN)
                || (ucPi != RPI_HEALTH_UNKNOWN) || (ucMv != RPI_HEALTH_UNKNOWN))
            {
                ucShowHealth = 1u;
            }
        }

        if (ucShowHealth != 0u)
        {
            /* 0=失败 ×, 1=通过 √, 其它（未检查/检查中）= ? */
            static const char *s_apcHealthSym[3] = { "×", "√", "?" };
            char    acBuf[32];
            uint8_t ucSymM  = (ucM  < 3u) ? ucM  : 2u;
            uint8_t ucSymQ  = (ucQ  < 3u) ? ucQ  : 2u;
            uint8_t ucSymPi = (ucPi < 3u) ? ucPi : 2u;
            uint8_t ucSymMv = (ucMv < 3u) ? ucMv : 2u;

            (void)snprintf(acBuf, sizeof(acBuf), "H:M%sQ%sPi%sMV%s",
                           s_apcHealthSym[ucSymM],  s_apcHealthSym[ucSymQ],
                           s_apcHealthSym[ucSymPi], s_apcHealthSym[ucSymMv]);
            (void)ucOledDrawMixed(0u, OLED_Y4, acBuf);
        }
        else
        {
            /* 秒表: < 1 分钟 → `Time:15.3s`（一位小数, 截断不四舍五入）
             *       ≥ 1 分钟 → `Time:1:05s`（分钟 + 两位秒, 只显示整数） */
            uint32_t ulMs = ulTimerElapsedMs();

            if (ulMs >= 60000u)
            {
                vOledPrintf(OLED0, 0, OLED_Y4, OLED_8X16, "Time:%lu:%02lus",
                            (unsigned long)(ulMs / 60000u),
                            (unsigned long)((ulMs % 60000u) / 1000u));
            }
            else
            {
                vOledPrintf(OLED0, 0, OLED_Y4, OLED_8X16, "Time:%lu.%lus",
                            (unsigned long)(ulMs / 1000u),
                            (unsigned long)((ulMs % 1000u) / 100u));
            }
        }
    }
}

/* ==================================================================
 * 第 2 页: 二维码设定（2026-10 起带「二维码功能」小菜单）
 * ==================================================================
 * 布局（一屏 4 行; 行 1 是**固定状态行**, 行 2..6 是一个 5 条目 × 3 可见行的滚动窗口）:
 *   行1  二维码:<值>|PI:OK/?    ← 恒显: 当前值 + 链路（后面的功能都依赖 Pi 在线）
 *   行2  [1] [1] [1]            ← 数字编辑行（**可选中, MAIN 直接进编辑** —— 2026-10-18
 *                                  起「二维码设定」那一项撤了, 它与此重复）
 *   行3  自动下发QR:√/×/?      ← MAIN = 翻转 Pi 的「自动下发二维码」开关(? = 未知/离线)
 *   行4  上传二维码              ← MAIN = 把当前值写进 Pi 并启用 —— 下一次上电,
 *                                  Pi 握手一成功就推这帧调试码（"QP<ddd>"）,
 *                                  **不用再手设**: 这就是「把台架调好的码带去比赛」的路。
 *   行5  清空二维码              ← MAIN = **本地 + 云端一起清**（本端值回 -1/清人工标记
 *                                  恢复真扫码; 链路 OK 时顺带清 Pi 端 —— 关自动下发
 *                                  + 清值, 下次上电不再推）。
 * ⚠️ 编辑态的按键仍是老三样: DOWN=+1 / UP=-1（与「上下」直觉相反, 别对调回去）、
 *    LEFT/RIGHT 移位、MAIN 写回并打人工预设标记; 编辑时第二行位置画提示「上下改左右移」。
 * ⚠️ 滚动条**复用菜单引擎那套**（vOledScrollbarDrawGeo, 轨道只罩菜单区 y=16..63,
 *    不碰行 1 的 128px 预算）→ 菜单标签宽度预算因此是 120px（本页最宽 112px ✓）,
 *    选中行反色也只反到 120（反到 128 会把轨道刷成"白底白块", 同菜单引擎那条纪律）。
 * ⚠️ 「自动下发」的状态查询是**按需**的: 光标在本页且值未知/过期且链路 OK 才发
 *    "QA"（2s 限频）—— 平时一个字节都不发, 不给链路添堵。
 * ⚠️ 三个动作**乐观更新本端镜像**（vRpiQrAutoSet/Upload/Clear 里做）: 按下那一拍屏上
 *    就翻, QS 回执来了按真相覆盖 —— 不等一个来回。
 */

/* 第 i 位数字的 x 坐标: 行是 "[d] [d] [d]", 每组 4 个字符 = 32px（"[d] "）,
 * 组内数字在第 2 个字符 —— 但高亮是反色整个 "[d]"（24px）, 所以只用得着组起点。 */
#define OLED_QR_GRP_W   32u

/* 本页可见的**菜单行数** —— 行 1 让给状态行, 只剩 3 行（Y2/Y3/Y4; 画到 Y=64 会被
 * 驱动静默丢掉, 窗口数学必须按 3 算, 别拿 OLED_MAX_ROWS(4) 来用 —— 第一版就栽在这）。 */
#define OLED_QR_VIS_ROWS (OLED_MAX_ROWS - 1u)

/* 小菜单的条目数与条目号（= s_ucCursor 在本页的语义） */
#define OLED_QR_ENTRIES  4u
#define OLED_QR_E_DIGIT  0u   /* 数字编辑行 */
#define OLED_QR_E_AUTO   1u   /* 自动下发QR */
#define OLED_QR_E_UPLOAD 2u   /* 上传二维码 */
#define OLED_QR_E_CLEAR  3u   /* 清空二维码（本地 + 云端一起） */

/* 窗口顶条目（0..OLED_QR_ENTRIES-OLED_QR_VIS_ROWS = 0..1）—— 菜单引擎的 s_ucTop
 * 是菜单页的, 本页自己一份, 互不干扰。 */
static uint8_t s_ucQrTop = 0u;

/* 「自动下发QR」的状态镜像在 **app_rpi**（s_ucQrAutoSt, QS 回执填充 + 新鲜期）,
 * 本页只留查询限频 —— 两处读状态（显示与查询判据）走同一个 ucRpiQrAutoState(),
 * 免得乐观翻转之后两份影子各说各话。 */
static uint32_t s_ulQrAutoAsk  = 0u;    /* 最近一次发 "QA" 的时刻（限频, 别刷爆链路） */
#define OLED_QR_AUTO_ASK_MS     2000u   /* 两次查询之间的最小间隔 */

/* 用 Beam 里的当前二维码值刷新编辑缓冲（无效值一律从 111 起, 按需求） */
static void vOledQrSyncFromBeam(void)
{
    int32_t iQr = iGetQrCode();

    if ((iQr >= 100) && (iQr <= 399))
    {
        s_aucQrDig[0] = (uint8_t)(iQr / 100);
        s_aucQrDig[1] = (uint8_t)((iQr / 10) % 10);
        s_aucQrDig[2] = (uint8_t)(iQr % 10);
    }
    else
    {
        s_aucQrDig[0] = 1u;
        s_aucQrDig[1] = 1u;
        s_aucQrDig[2] = 1u;
    }
}

/* 把窗口对齐到光标 —— 与菜单引擎的 vOledWindowSync 同一条「最小幅度跟随」规则,
 * 差别只剩: 本页没有「返回」条目、可见行数是 OLED_QR_VIS_ROWS(3) 不是 4。
 * 每拍画之前调一次, 自愈。 */
static void vOledQrWinSync(void)
{
    if (s_ucCursor >= OLED_QR_ENTRIES) { s_ucCursor = 0u; }   /* 防御: 翻页残留 */
    if (s_ucCursor < s_ucQrTop) { s_ucQrTop = s_ucCursor; }
    else if (s_ucCursor >= (uint8_t)(s_ucQrTop + OLED_QR_VIS_ROWS))
    {
        s_ucQrTop = (uint8_t)(s_ucCursor - OLED_QR_VIS_ROWS + 1u);
    }
    if (s_ucQrTop > (uint8_t)(OLED_QR_ENTRIES - OLED_QR_VIS_ROWS))
    {
        s_ucQrTop = (uint8_t)(OLED_QR_ENTRIES - OLED_QR_VIS_ROWS);
    }
}

/* 翻到本页时把小菜单的状态归零（翻页是全局重置点, 见 ucOledTurnPage;
 * 光标/窗口随它一起回位, 别把上一页的条目号带进来当本页的光标）。 */
static void vOledQrPageReset(void)
{
    s_ucCursor  = 0u;
    s_ucQrTop   = 0u;
}

/* 本页光标挪一格（带回绕; 全部条目恒可用, 不需要菜单引擎那套「跳过禁用项」） */
static void vOledQrCursorStep(int8_t iDir)
{
    if (iDir > 0) { s_ucCursor = (uint8_t)((s_ucCursor + 1u) % OLED_QR_ENTRIES); }
    else          { s_ucCursor = (s_ucCursor == 0u) ? (uint8_t)(OLED_QR_ENTRIES - 1u)
                                                    : (uint8_t)(s_ucCursor - 1u); }
}

/* 「自动下发QR」当前该显示的字符: √(开) / ×(关) / ?(未知、过期或链路不在) */
static const char *pcOledQrAutoMark(void)
{
    if (emGetRpiLinkState() != RPI_LINK_OK) { return "?"; }
    if (ucRpiQrAutoState() == 1u) { return "√"; }
    if (ucRpiQrAutoState() == 2u) { return "×"; }
    return "?";
}

/* MAIN 在小菜单各条目上的动作。返回 1 = 已弹出确认框（调用方本拍立刻收手）。
 * ⚠️ **本页一律不给蜂鸣**（同菜单引擎那条惯例: 反馈靠标签/横幅/提示框, 不靠声音）
 * —— 链路不在这类「按了做不了」走单键提示框（OLED_CONFIRM_NO_PI, 与「执行任务」
 * 「开始巡线测试」同款）; 「还没设值就上传」则什么都不做 —— 行 1 的 -1 自己会说话。 */
static uint8_t ucOledQrActivate(void)
{
    switch (s_ucCursor)
    {
    case OLED_QR_E_DIGIT:                       /* 数字行: MAIN 直接进编辑 */
        s_ucQrTop    = 0u;
        s_ucMode     = OLED_MODE_EDIT;
        s_ulQrOkTick = 0u;                      /* 把上次的 "OK!" 收掉 */
        return 0u;

    case OLED_QR_E_AUTO:                        /* 行3: 翻转 Pi 的「自动下发」 */
        if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_PI);
            return 1u;
        }
        /* 未知(0)也按「开」起步; 乐观翻转在 vRpiQrAutoSet 里做（QS 回执来纠正） */
        vRpiQrAutoSet((ucRpiQrAutoState() == 1u) ? 0u : 1u);
        return 0u;

    case OLED_QR_E_UPLOAD:                      /* 行4: 上传当前值到 Pi */
        if (OLED_QR_IS_SET(iGetQrCode()) == 0u)
        {
            return 0u;                          /* 当前值本来就没设: 上传个 -1 没有意义 */
        }
        if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_PI);
            return 1u;
        }
        vRpiQrUpload(iGetQrCode());             /* Pi 会回 QS 并顺手把调试码推回来 */
        s_ulQrOkTick = xTaskGetTickCount();     /* 行 1 闪 "OK!" 2s（纯视觉, 同保存那条） */
        return 0u;

    case OLED_QR_E_CLEAR:                       /* 行5: 清空二维码（本地 + 云端一起） */
    default:
        /* 本端: 清值 + 清「人工预设」标记（与打标记的那两个写入点对称 ——
         * 清完第 1/2 页立刻显示 -1, 扫码步骤恢复"真去问 Pi"）。纯本地, 无条件清。
         * 云端: 链路 OK 时顺带清 Pi 端（关自动下发 + 清值, 下次上电不再推）;
         * 链路不在就只清本地 —— 值清零本身就是可见反馈, 云端那半边等链路恢复
         * 后再按一次（或网页上清）收口。 */
        vSetQrCode(-1);
        vSetQrManual(0u);
        if (emGetRpiLinkState() == RPI_LINK_OK)
        {
            vRpiQrClear();
        }
        return 0u;
    }
}

static void vOledPageQr(uint8_t ucKeys)
{
    uint8_t aucShow[3];
    uint8_t k;

    if (s_ucMode == OLED_MODE_NAV)
    {
        /* 顶层: LEFT/RIGHT 翻页（两端夹住不循环）, 其余键在小菜单里选/激活 */
        if (ucOledTurnPage(ucKeys) != 0u) { vOledQrPageReset(); return; }

        if ((ucKeys & (OLED_KEY_UP | OLED_KEY_DOWN)) != 0u)
        {
            vOledQrCursorStep((ucKeys & OLED_KEY_DOWN) != 0u ? 1 : -1);
        }
        else if ((ucKeys & OLED_KEY_MAIN) != 0u)
        {
            /* 返回 1 = 已弹确认框（mode 已被设成 CONFIRM）→ 本拍别再画, 否则刚设好的
             * 状态会被下面的画分支盖掉一帧（同菜单引擎的 ENTER_CONFIRM 那条纪律）。 */
            if (ucOledQrActivate() != 0u) { return; }
        }

        /* 按需查询「自动下发」的状态: 光标停在本页 + 链路 OK + 值未知/过期 + 限频到点。
         * ⚠️ 刻意**不等**编辑态再查（编辑态滚数字, 那一行看不见）。 */
        if ((xTaskGetTickCount() - s_ulQrAutoAsk) > pdMS_TO_TICKS(OLED_QR_AUTO_ASK_MS))
        {
            if ((ucRpiQrAutoState() == 0u) && (emGetRpiLinkState() == RPI_LINK_OK))
            {
                vRpiQrAutoQuery();
                s_ulQrAutoAsk = xTaskGetTickCount();
            }
        }

        /* 显示当前真值（无效值显示 111 —— 与编辑起点一致, 免得一进编辑数字就跳）。
         * 每拍都从 Beam 同步: 行 2 既是「编辑起点」也是「当前扫码结果的实时反映」,
         * 两者用同一个缓冲, 所以进编辑时不需要再单独初始化一次。 */
        vOledQrSyncFromBeam();
    }
    else
    {
        /* 编辑态: DOWN = +1 / UP = -1（当前位 1~3 循环）, LEFT/RIGHT 移位（0→1→2→0 循环）。
         * ⚠️ 方向是 DOWN 加、UP 减（按需求）—— 与「上下」直觉相反, 别「顺手」对调回去。 */
        if ((ucKeys & OLED_KEY_DOWN) != 0u)
        {
            s_aucQrDig[s_ucCursor] = (s_aucQrDig[s_ucCursor] >= 3u) ? 1u
                                                                   : (uint8_t)(s_aucQrDig[s_ucCursor] + 1u);
        }
        else if ((ucKeys & OLED_KEY_UP) != 0u)
        {
            s_aucQrDig[s_ucCursor] = (s_aucQrDig[s_ucCursor] <= 1u) ? 3u
                                                                   : (uint8_t)(s_aucQrDig[s_ucCursor] - 1u);
        }

        if ((ucKeys & OLED_KEY_LEFT) != 0u)
        {
            s_ucCursor = (s_ucCursor == 0u) ? 2u : (uint8_t)(s_ucCursor - 1u);
        }
        else if ((ucKeys & OLED_KEY_RIGHT) != 0u)
        {
            s_ucCursor = (uint8_t)((s_ucCursor + 1u) % 3u);
        }

        if ((ucKeys & OLED_KEY_MAIN) != 0u)
        {
            /* 退出编辑 → 写回。⚠️ 与 STEP_QR_SET_TEST 同一性质: **会覆盖真实扫码结果**
             * （行 1 如实显示当前值, 所以覆盖是看得见的）。
             * 三位都是 1~3, 拼出来必落在 111..333, 不需要再校验。 */
            int32_t iQr = ((int32_t)s_aucQrDig[0] * 100)
                        + ((int32_t)s_aucQrDig[1] * 10)
                        +  (int32_t)s_aucQrDig[2];

            vSetQrCode(iQr);
            /* ⚠️ 同时打上「人工预设」标记（见 app_beam.h 的 ucGetQrManual）:
             * 之后路线里那些 STEP_QR_SCAN_WAIT 会**直接跳过、不再问树莓派**, 沿用这个值
             * —— 台架演练（没有二维码可扫）靠的就是它。
             * 复位/断电会清掉, 所以比赛上电后默认还是真扫码; 要长期保留用「上传二维码」。 */
            vSetQrManual(1u);
            s_ulQrOkTick = xTaskGetTickCount();
            s_ucMode     = OLED_MODE_NAV;
            s_ucCursor   = OLED_QR_E_DIGIT;
        }
    }

    /* ---- 画: 行 1 固定状态行 ---- */
    vOledQrWinSync();      /* 先对齐窗口再画（同菜单引擎的次序）—— 从别的页翻进来的
                            * 第一拍 s_ucQrTop 可能还留着上一次的值, 画完再修就晚一帧 */
    aucShow[0] = s_aucQrDig[0];
    aucShow[1] = s_aucQrDig[1];
    aucShow[2] = s_aucQrDig[2];

    /* ⚠️ 最宽 `二维码:111|PI:OK` = 48+8+24+8+16+8+16 = 128px, 正好用满一行。
     *    链路不在时用 "?"（8px）—— `二维码:111|PI:?` = 120px, 也放得下。 */
    if (ucOledWithin(s_ulQrOkTick, OLED_QR_OK_MS) != 0u)
    {
        vOledDrawRow(OLED_Y1, "OK!");           /* 保存/上传成功后闪 2s（占状态行, 不占菜单行） */
    }
    else
    {
        vOledDrawRowf(OLED_Y1, "二维码:%d|PI:%s", (int)iGetQrCode(),
                      (emGetRpiLinkState() == RPI_LINK_OK) ? "OK" : "?");
    }

    /* ---- 画: 行 2..4 = 小菜单窗口（s_ucQrTop 起 OLED_QR_VIS_ROWS=3 条）---- */
    for (k = 0u; k < OLED_QR_VIS_ROWS; k++)
    {
        uint8_t ucY    = (uint8_t)(OLED_Y2 + (k * OLED_ROW_H));
        uint8_t ucE    = (uint8_t)(s_ucQrTop + k);          /* 条目号 */
        uint8_t ucSell = (uint8_t)((s_ucMode == OLED_MODE_NAV)
                                   && (ucE == s_ucCursor)); /* 选中 = 整行反色 */

        if (ucE >= OLED_QR_ENTRIES)
        {
            vOledClearArea(OLED0, 0u, ucY, OLED_W, OLED_ROW_H);
            continue;
        }

        if ((s_ucMode == OLED_MODE_EDIT) && (k == 1u))
        {
            /* 编辑态: 紧挨数字行的那一回答提示语（DOWN/UP 与 LEFT/RIGHT 各管什么）——
             * 编辑必然发生在 top=0（光标在数字行）, 所以这一行永远在数字行下面一格。 */
            vOledDrawRow(ucY, "上下改左右移");
            continue;
        }

        switch (ucE)
        {
        case OLED_QR_E_DIGIT:
            /* 尾缀 ` *` = **人工预设**（同第 1 页 `QR:111*` 那套可见性）—— 原来
             * 挂在「二维码设定」那行的星号跟着那一项一起撤了, 挪到这里
             * （`[1] [1] [1] *` = 88+16 = 104px, 120 预算内）。 */
            vOledDrawRowf(ucY, "[%u] [%u] [%u]%s",
                          (unsigned)aucShow[0], (unsigned)aucShow[1], (unsigned)aucShow[2],
                          (ucGetQrManual() != 0u) ? " *" : "");
            /* 编辑态: 反色的是整个 "[d]" 三个字符（24px）而不是光秃秃一个数字, 位置更好认。
             * ⚠️ 必须在画完行之后再反色（XOR 会连没写的像素一起翻）。 */
            if (s_ucMode == OLED_MODE_EDIT)
            {
                vOledReverseArea(OLED0,
                                 (uint8_t)(s_ucCursor * OLED_QR_GRP_W),
                                 ucY,
                                 (uint8_t)(OLED_ASC_W * 3u),
                                 OLED_ROW_H);
            }
            break;

        case OLED_QR_E_AUTO:
            vOledDrawRowf(ucY, "自动下发QR:%s", pcOledQrAutoMark());
            break;

        case OLED_QR_E_UPLOAD:
            vOledDrawRow(ucY, "上传二维码");
            break;

        case OLED_QR_E_CLEAR:
        default:
            vOledDrawRow(ucY, "清空二维码");
            break;
        }

        /* ⚠️ 有滚动条 → 反色只到 OLED_LABEL_W(120), 反到 128 会把轨道刷成"白底白块"
         * （同菜单引擎 vOledHighlightRow 那条纪律）。 */
        if (ucSell != 0u) { vOledHighlightRow(ucY, OLED_LABEL_W); }
    }

    /* 滚动条**最后画**（盖住上面所有步骤写到 x>=120 的列）—— 复用菜单引擎那套几何,
     * 轨道只罩菜单区（y=16..63）: 行 1 的 128px 状态行在轨道外面, 一像素都不让。 */
    vOledScrollbarDrawGeo(OLED_QR_ENTRIES, OLED_QR_VIS_ROWS, s_ucQrTop,
                          OLED_Y2, (uint8_t)(OLED_QR_VIS_ROWS * OLED_ROW_H));
}


/* 投递一条脚本请求。返回 1=已投递, 0=槽位被占（上一条还没被脚本任务取走）。
 * 失败时长鸣提示, **不静默吞掉** —— 菜单上「按了没反应」的现场全靠这一声。
 * ⚠️ 三个投递点（执行任务 / 握手让出 / 巡线测试）都走这里, 别各写一份带蜂鸣的。 */
static uint8_t ucOledPostReq(uint8_t ucReq, uint8_t ucArg)
{
    if (vScriptPostReq(ucReq, ucArg) == 0u)
    {
        return 0u;
    }
    return 1u;
}

/* OpenMV 补光亮度: 本拍的影子值。开机回**本宏**（40, 与 OpenMV 端 main.py 的
 * LED_BRIGHTNESS 同名同值）。⚠️ **它不等于是相机当前的亮度**: 唯一真相是 Pi 侧
 * config.OPENMV_LED_BRIGHTNESS（可被 Web「设置参数」写成别的值, 现场现值就不是 40）,
 * 而且是**会话内易失**的（同陀螺目标角那类 Beam 量的口径）—— 只重启单片机而
 * Pi/相机还开着时, 这里显示的数未必是实际值。
 * 所以: 本页只是"我要下发的值"的编辑起点, 不是遥测。改一次 → 发 "L,<n>" 给 Pi,
 * 之后两边就一致了; 巡线起流（"2"）前 Pi 还会按它自己记住的值再对齐一次。 */
#define OLED_OPENMV_LED_DEFAULT 40u
#define OLED_OPENMV_LED_STEP    10u
static uint8_t s_ucOpenmvLed = OLED_OPENMV_LED_DEFAULT;

/* 陀螺目标角的 5 档循环（180.0 与 -180.0 是同一朝向, 但保留两档 ——
 * 现场「指定方向」时看得见两边; 真要指定方向的掉头请用 STEP_SPIN_TURN_WAIT）。
 * ⚠️ 这个值只在积木 src=陀螺 时被用到（src=相机/不修 的步骤一个字节都不读它）。 */
static const float s_afGyroTgt[5] = { 0.0f, 90.0f, 180.0f, -180.0f, -90.0f };
#define OLED_GYRO_TGT_NUM  5u

/* float -> 十分位整数（四舍五入, 负数也对）。调参按步长加减都在整数域做,
 * 免得 0.1 累加攒出 -1.5000000000000002 这种值再写回 Beam。 */
static int32_t iOledTenths(float fVal)
{
    float f = fVal * 10.0f;

    return (int32_t)((f >= 0.0f) ? (f + 0.5f) : (f - 0.5f));
}

/**
  * @brief       把「巡线测试」标签对齐到**设备真值**
  * @retval      1 = 现在该显示「停止巡线测试」
  *
  * ⚠️⚠️ 这里是**每拍双向对齐**, 不是「只在满足某条件时清掉」的弱自愈 —— 后者是
  *    2026-09 的一个真 bug: 原来只在 `script_running == 0` 时清 `s_bLineTestOn`,
  *    而那个量对**任何**路线都是 1（开机路线、菜单启动的任务预设…）→ 标签一旦置上就
  *    再也回不去, 卡在「停止巡线测试」; 更糟的是它会让 `case 1u` 走**停止**分支,
  *    「没设二维码」那道提示就永远不会弹（现场就是「提示没生效」）。
  * ⚠️ 真值来自 `ucScriptLineTestActive()`（在跑 **且** 跑的就是那条测试预设）。
  * ⚠️ 投递后到脚本真开跑之间有最多 50ms 空窗（请求还在槽里）, 那期间真值还是 0 ——
  *    靠 OLED_LINE_START_GRACE_MS 宽限把它算成「在跑」, 否则标签会闪一下、
  *    而操作员可能因此再按一次（那一次就成了「停」）。
  */
static uint8_t ucOledLineTestOn(void)
{
    if (ucScriptLineTestActive() != 0u)
    {
        s_bLineTestOn = 1u;                  /* 设备说在跑 → 拉高（也可能是别处启动的）*/
    }
    else if ((xTaskGetTickCount() - s_ulLineTestTick)
             > pdMS_TO_TICKS(OLED_LINE_START_GRACE_MS))
    {
        s_bLineTestOn = 0u;                  /* 过了宽限还没跑 → 拉低（被中止/自己跑完）*/
    }
    return s_bLineTestOn;
}

/* 「射靶测试」开关（2026-10-09 加, OLED 射靶测试页第 1 项）—— 与上面那条**同构**,
 * 区别在**真值要覆盖一段长得多的时间**: 那一条从投递到 `vScriptRun` 开跑只有 ≤50ms
 * （一个 500ms 宽限窗就够）; 这一条走 `SCRIPT_REQ_TASK`, 而**首次跑时它前面还有一整套
 * 「机械臂待命展开」**（阻塞好几秒）—— 只看 `ucScriptPresetActive()` 的话, 那几秒里标签会
 * 弹回「开始射靶测试」, 操作员一按就变成"再投一次"(中止+重跑)。所以加一个**待落地标志**
 * `s_bShootPending`:
 *   · 投递成功 → 置 1;
 *   · 看到 `ucScriptPresetActive(预设id)` 为真 → 清零（交棒: 从"我投的"变成"设备说在跑"）;
 *   · 脚本任务闲着（`script_running == 0`）→ 清零（那条请求没落地: 被顶掉/被拒）。
 * ⚠️ 这两个来源 + 500ms 宽限窗（覆盖"投递→脚本取走"那 ≤50ms）合起来才是完整的真值。
 * ⚠️ 宽限窗的起点是 `s_ulShootTestTick`, **停的时候要把它清 0** —— 否则「按下停止，
 *    500ms 内想重开」会被宽限窗挡成"还在跑"[那是"停"分支], 看着像"按了没反应"。 */
static uint8_t ucOledShootTestOn(uint8_t ucPresetId)
{
    uint8_t ucAct = ucScriptPresetActive(ucPresetId);

    if ((ucAct != 0u) || (ucGetScriptRunning() == 0u))
    {
        s_bShootPending = 0u;
    }

    if (ucAct != 0u)
    {
        s_bShootTestOn = 1u;
    }
    else if ((s_bShootPending != 0u)
             || ((s_ulShootTestTick != 0u)
                 && ((xTaskGetTickCount() - s_ulShootTestTick)
                     <= pdMS_TO_TICKS(OLED_SHOOT_START_GRACE_MS))))
    {
        s_bShootTestOn = 1u;
    }
    else
    {
        s_bShootTestOn = 0u;
    }

    return s_bShootTestOn;
}
/* ==================================================================
 * 菜单页 —— 通用引擎（页面表 / 子菜单共用）
 *
 * 菜单的**内容**在 `app_oled_menu.c` 的数据表里（由 Web 控制台「OLED」Tab 生成）;
 * 本文件只管「查表 → 按动作码分派 → 画」。
 *
 * ⚠️⚠️ **表是「结构」, 这里是「动词」** —— 标签/顺序/启停/指向哪张子表在表里;
 *     取值/步进/量程/副作用/蜂鸣/前置条件全在下面的 switch 里。别把动词往表里搬
 *     （三个调参量的行为各不相同, 硬凑成通用注册表会失真且更重）。
 * ⚠️ 屏只有 4 行（`OLED_MAX_ROWS`）, 驱动的 `vOledShowImage` 在 Y > 63 直接 return
 *     —— 超出的行**静默不画**, 所以这里必须自己钳住, 别指望驱动报错。
 * ⚠️ 但一处表**可以有多于 4 个启用项**: 超出的靠**滚动窗口**（`s_ucTop` + `vOledWindowSync`）
 *     显示, 右侧画竖直滚动条指示位置。窗口那一套集中在下面「滚动窗口」那几个函数里。
 * ================================================================== */

/* 这张表里有没有某个动作码（「让出成功!」横幅靠它挂） */
static uint8_t ucOledListHasAct(const stOledListTdf *pstList, uint8_t ucAct)
{
    uint8_t i;

    if (pstList == 0) { return 0u; }
    for (i = 0u; i < pstList->ucCount; i++)
    {
        if (pstList->pstItems[i].ucAct == ucAct) { return 1u; }
    }
    return 0u;
}

/* 当前页的表（第 1/2 页是代码页, 没有表 → 返回 0） */
static const stOledListTdf *pstOledPageList(uint8_t ucPage)
{
    if (ucPage >= g_ucOledPageNum) { return 0; }
    return g_astOledPage[ucPage].pstList;
}

static uint8_t ucOledPageKind(uint8_t ucPage)
{
    if (ucPage >= g_ucOledPageNum) { return (uint8_t)OLED_PAGEKIND_STATUS; }
    return g_astOledPage[ucPage].ucKind;
}

/* 表里有几个**可见**项（启用的, 截到 OLED_MAX_ITEMS 为止）。
 * ⚠️ 上限从 OLED_MAX_ROWS(4) 改成 OLED_MAX_ITEMS(32) —— 这是"放开行数"的**唯一**一处:
 *    放不下的那几项由窗口（s_ucTop）滚动显示, 不再被静默丢掉。 */
static uint8_t ucOledVisibleCount(const stOledListTdf *pstList)
{
    uint8_t i, n = 0u;

    if (pstList == 0) { return 0u; }
    for (i = 0u; (i < pstList->ucCount) && (n < OLED_MAX_ITEMS); i++)
    {
        if (pstList->pstItems[i].ucEnabled != 0u) { n++; }
    }
    return n;
}

/* 这一屏一共几个**条目** = 可见项 + （子菜单那条固件的「返回」）。
 * 上限 33（32 个启用项 + 返回）。窗口/滚动条/翻页全按它算。 */
static uint8_t ucOledEntryCount(const stOledListTdf *pstList, uint8_t bSub)
{
    return (uint8_t)(ucOledVisibleCount(pstList) + ((bSub != 0u) ? 1u : 0u));
}

/* 第 ucE 个**条目**的**表下标**（ucE 从 0 起, 只对 ucE < ucOledVisibleCount 有效）。
 * ⚠️⚠️ 越界**回 0xFF**（老版 ucOledVisibleIndex 回 0!）—— 回 0 会把"越界"调试成
 *    "第一项", 调用方一个没判就静默画/激活错的项。所有调用方**必须**判 0xFF。 */
static uint8_t ucOledEntryIndex(const stOledListTdf *pstList, uint8_t ucE)
{
    uint8_t i;

    if (pstList == 0) { return 0xFFu; }
    for (i = 0u; i < pstList->ucCount; i++)
    {
        if (pstList->pstItems[i].ucEnabled != 0u)
        {
            if (ucE == 0u) { return i; }
            ucE--;
        }
    }
    return 0xFFu;
}

/* 光标（表下标, 或子菜单的 OLED_CURSOR_BACK）落在**第几个条目**上（= 屏上逻辑行号）。
 * 哨兵 → ucOledVisibleCount（「返回」恒是最后一条）; 指着一个禁用项/越界 → 0xFF。 */
static uint8_t ucOledCursorEntry(const stOledListTdf *pstList, uint8_t bSub)
{
    uint8_t i, e = 0u;

    if (s_ucCursor == OLED_CURSOR_BACK)
    {
        return ((bSub != 0u) ? ucOledVisibleCount(pstList) : 0xFFu);
    }
    if (pstList == 0) { return 0xFFu; }
    for (i = 0u; i < pstList->ucCount; i++)
    {
        if (pstList->pstItems[i].ucEnabled != 0u)
        {
            if (i == s_ucCursor) { return e; }
            e++;
        }
    }
    return 0xFFu;
}

/* 光标落到一个**可用**的位置: 当前位可用就保持, 否则向前找第一个启用的。 */
static void vOledCursorNormalize(const stOledListTdf *pstList)
{
    uint8_t i;

    if ((pstList == 0) || (pstList->ucCount == 0u)) { s_ucCursor = 0u; return; }
    if ((s_ucCursor < pstList->ucCount)
        && (pstList->pstItems[s_ucCursor].ucEnabled != 0u)) { return; }
    s_ucCursor = 0u;
    for (i = 0u; i < pstList->ucCount; i++)
    {
        if (pstList->pstItems[i].ucEnabled != 0u) { s_ucCursor = i; return; }
    }
}

/* 把窗口对齐到光标 —— **幂等 + 自愈**: 先钳 s_ucTop 进 [0, N-4], 再**最小幅度**滚动
 * 让光标进窗口。同一组输入连调两次结果相同。
 * ⚠️ 每拍画之前调一次 ⇒ 任何地方漏改 s_ucTop 都在 50ms 内自动修好（翻页/进子菜单/
 *    退子菜单那几处显式复位只是"意图清楚", 不是正确性所必需）。 */
static void vOledWindowSync(const stOledListTdf *pstList, uint8_t bSub)
{
    uint8_t ucN   = ucOledEntryCount(pstList, bSub);
    uint8_t ucCur = ucOledCursorEntry(pstList, bSub);

    if (ucN <= OLED_MAX_ROWS) { s_ucTop = 0u; return; }   /* 一屏放得下 → 恒从第 0 条起 */
    if (ucCur == 0xFFu) { ucCur = 0u; }

    if (s_ucTop > (uint8_t)(ucN - OLED_MAX_ROWS)) { s_ucTop = (uint8_t)(ucN - OLED_MAX_ROWS); }
    if (ucCur < s_ucTop)
    {
        s_ucTop = ucCur;
    }
    else if (ucCur >= (uint8_t)(s_ucTop + OLED_MAX_ROWS))
    {
        s_ucTop = (uint8_t)(ucCur - OLED_MAX_ROWS + 1u);
    }
}

/* 菜单光标挪一格 —— **顶层页与子菜单共用**（差别只剩"有没有最后那条「返回」",
 * 那由 ucOledEntryCount 表达, 不再需要两个函数）。
 * ⚠️ 保留**回绕**（与改之前逐字一致: 顶层绕全部启用项, 子菜单绕 可见数+1）——
 *    改成钳位就是"行为回归", 现有 4 项页面的手感会变。
 * ⚠️ **有界查找**: 一个启用的都没有时原地不动 —— 用 while 会死循环, 而本任务优先级
 *    最低(1), 挂死 = 整屏停摆 + `vSetOledMenuBusy` 一直拉高, 最坏的一类故障。 */
static void vOledCursorStep(const stOledListTdf *pstList, uint8_t bSub, int8_t iDir)
{
    uint8_t ucN   = ucOledEntryCount(pstList, bSub);
    uint8_t ucCur = ucOledCursorEntry(pstList, bSub);
    uint8_t ucItems, k, e;

    if (ucN == 0u) { return; }
    if (ucCur == 0xFFu) { ucCur = 0u; }
    ucItems = ucOledVisibleCount(pstList);
    e = ucCur;

    for (k = 0u; k < ucN; k++)
    {
        e = (iDir > 0) ? (uint8_t)((e + 1u) % ucN)
                       : (uint8_t)((e == 0u) ? (ucN - 1u) : (e - 1u));
        if ((bSub != 0u) && (e == ucItems))          /* 最后那条 = 固件的「返回」 */
        {
            s_ucCursor = OLED_CURSOR_BACK;
            return;
        }
        {
            uint8_t ucIdx = ucOledEntryIndex(pstList, e);

            if (ucIdx != 0xFFu) { s_ucCursor = ucIdx; return; }
        }
    }
    /* 全禁用且没有「返回」→ 原地不动（不进死循环、不转圈） */
}

/* ---- 换行动画（选中行文字滑入）---- */

/* 起跳一次滑入。光标**真的换了行**时才调（同一行上没换 = 没动画）。
 * ⚠️ 开关关着时什么都不做 —— s_ucAnimOfs 恒 0 ⇒ 文字恒画在 x=0, 与加动画之前**逐字节一致**。 */
static void vOledAnimStart(void)
{
    if (ucGetOledMenuAnim() != 0u) { s_ucAnimOfs = OLED_ANIM_X0; }
}

/* 每拍收回一点（画之前调）。到 0 就停 —— 没有动画在跑时是本函数的最快路径（一次比较）。 */
static void vOledAnimTick(void)
{
    if (s_ucAnimOfs == 0u) { return; }
    if (ucGetOledMenuAnim() == 0u)      /* 动画中途被蓝牙关掉 → 立刻归位 */
    {
        s_ucAnimOfs = 0u;
        return;
    }
    s_ucAnimOfs = (uint8_t)((s_ucAnimOfs > OLED_ANIM_STEP) ? (s_ucAnimOfs - OLED_ANIM_STEP) : 0u);
}

/* 运行期标签（静态兜底标签在表里）—— ⚠️ 与**动作码**同键, 别按下标判:
 * 下标一被拖乱, 标签就会跑到别的项上, 而且不报错。 */
static const char *pcOledLiveLabel(const stOledItemTdf *pstIt)
{
    switch ((emOledActTdf)pstIt->ucAct)
    {
    case OLED_ACT_LASER_TOGGLE:   /* 激光是危险执行器, 开/关必须一眼看出 */
        /* ⚠️ 2026-10-09 由 `开关激光:开/关` 压成 `激光:开/关`（省 32px）: 射靶测试页要把
         * 「激光」那一行**右边拼上实时偏差** `d=-3px`（`激光:开 d=-3px` = 112px 正好, 而
         * 原来的 5 个字 + 那个尾巴 = 144px 会**从尾部静默截断**）。信息一点没少, 第 3 页跟着变短。 */
        return (ucGetLaserOn() != 0u) ? "激光:开" : "激光:关";

    case OLED_ACT_FILL_TOGGLE:    /* 补光灯（2026-10 加）: 现取现画（同激光那条）, 别缓存 */
        return (usGetFillDuty() != 0u) ? "补光灯:开" : "补光灯:关";

    case OLED_ACT_LINE_TEST:
        return (ucOledLineTestOn() != 0u) ? "停止巡线测试" : "开始巡线测试";

    case OLED_ACT_SHOOT_TEST:
        /* 「射靶测试」开关（2026-10-09 加）—— 四态。**实时偏差不上这行**（用户定: 拼在
         * 「激光」那行的尾巴上, 且只留 `d=`; 见 vOledMenuDraw 里那条 hook）。
         * 宽度预算（一行 128px; 汉字 16px / ASCII 8px）:
         *   开始射靶测试   = 6×16                = 96px
         *   停止射靶 运动中 = 4×16 + 8 + 3×16     = 120px
         *   停止射靶 OK!   = 4×16 + 8 + 3×8      = 96px
         *   停止射靶 冻结   = 4×16 + 8 + 2×16     = 104px
         *   停止射靶 无目标 = 4×16 + 8 + 3×16     = 120px
         *   停止射靶 无信号 = 4×16 + 8 + 3×16     = 120px
         * ⚠️「运动中」不是好看: 它是给操作员看"**现在别开火**"（车在动 = 光点在动）。
         * ⚠️ 后面那三个态回答的是同一个问题 ——「**车为什么不动**」: 冻结(撞闸/环没起作用、
         *    按一格可重试) / 无目标(丢靶或被半径门全剔除) / 无信号(Pi 没在发)。少了它们,
         *    屏上会停在最后一个 `OK!` 或陈旧 d 上, 看着像"已经对准了、只是车没动"。
         * ⚠️ 判据都用**设备真值**: 开跑/停 = ucOledShootTestOn(预设id), 状态 = Beam shoot_state,
         *    OK! 窗 = shoot_settle_tick 的时刻（**看时刻不看事件**, 同拍照那条）。 */
        if (ucOledShootTestOn(pstIt->ucArg) == 0u) { return "开始射靶测试"; }
        /* ⚠️ 粗调态**必须**单列一个标签: 那一段车随时会走 150mm 级别的纵移, 而此刻还没开始
         * 测偏差（d 还是 0）—— 显示成"停止射靶"（=已稳住）是**安全上会误导**的那种。 */
        if (ucGetShootState() == OLED_SHOOT_ST_COARSE) { return "停止射靶 粗调"; }
        if (ucGetShootState() == OLED_SHOOT_ST_MOVING) { return "停止射靶 运动中"; }
        if (ucGetShootState() == OLED_SHOOT_ST_FROZEN) { return "停止射靶 冻结"; }
        if (ucGetShootState() == OLED_SHOOT_ST_NOTGT)  { return "停止射靶 无目标"; }
        if (ucGetShootState() == OLED_SHOOT_ST_STALE)  { return "停止射靶 无信号"; }
        if (ucOledWithin(ulGetShootSettleTick(), OLED_SHOOT_OK_MS) != 0u) { return "停止射靶 OK!"; }
        return "停止射靶";

    case OLED_ACT_PHOTO:          /* 设备回话后闪 1s 成功/失败, 之后自己回静态标签 */
        if (ucOledWithin(ulGetPhotoDoneTick(), OLED_PHOTO_MSG_MS) != 0u)
        {
            return (ucGetPhotoResult() != 0u) ? "拍照 成功!" : "拍照 失败!";
        }
        return pstIt->pcLabel;

    case OLED_ACT_GYRO_SRC:       /* 现取现画（同激光那条）, 别缓存 */
        return (ucGetGyroSrc() != 0u) ? "陀螺源:副" : "陀螺源:主";

    case OLED_ACT_GYRO_CAL:       /* 陀螺校准（2026-10 加）: 三态, 同拍照那条的挂法 */
        /* 宽度预算（一行 128px; 汉字 16px / ASCII 8px, 菜单行另留滚动条 8px → 120px）:
         *   校准中请勿动车 = 7×16 = 112px ✓ —— busy 期间这条是给操作员的**安全提示**
         *     （车一动零偏就估歪了, 固件没法替他按住车）, 不是装饰;
         *   校准完成!/校准失败! = 4×16 + 8 = 72px ✓（闪 1s, 看 done_tick 时刻不看事件）。 */
        if (ucGetGyroCalBusy() != 0u) { return "校准中请勿动车"; }
        if (ucOledWithin(ulGetGyroCalDoneTick(), OLED_PHOTO_MSG_MS) != 0u)
        {
            return (ucGetGyroCalResult() != 0u) ? "校准完成!" : "校准失败!";
        }
        return pstIt->pcLabel;

    default:
        return pstIt->pcLabel;
    }
}

/* 这一项是不是**数值行**（吃 LEFT/RIGHT 调值、按 MAIN 在子菜单里退出的那一类）——
 * 两个调参动作码都算: TUNE_QTY 是固件硬编码的那 5 个量, TUNE_PARAM 是按名字的通用档。
 * ⚠️ 一处判"是不是调参行"就够的地方都该用它 —— 漏掉新动作码会让那一行**既不能调值、
 *    也不能靠 MAIN 退出**, 现象是"这一行按了没反应"（最难查的那种）。 */
static uint8_t ucOledIsTuneAct(uint8_t ucAct)
{
    return (uint8_t)((ucAct == (uint8_t)OLED_ACT_TUNE_QTY)
                     || (ucAct == (uint8_t)OLED_ACT_TUNE_PARAM));
}

/* 射靶「瞄准列」当前该读/写哪一根 —— 由**这一步实际在用的相机**定（2026-10-16 加）。
 * 两台相机（主摄 / pi CSI）相对位置不同、各有一对瞄准常数; 调错那一根的现象正是
 * "在 OLED 上按了一格、车却不动"（环读的是另一把尺）, 所以读写必须与环同源:
 * 判据 = ucGetShootCam()（0=主摄 / 1=pi, 由回帧 status 的 bit7 走）。
 * ⚠️ 没在跑射靶时它是上一次的值（上电为 0）—— 那时调的是主摄那根, 与改造前一致。 */
static float fOledShootAimGet(void)
{
    uint8_t ucSw = ucGetTaXySwap();

    if (ucGetShootCam() != 0u)
    {
        return (ucSw != 0u) ? fGetTcCxAimPi() : fGetTcCyAimPi();
    }
    return (ucSw != 0u) ? fGetTcCxAim() : fGetTcCyAim();
}

static void vOledShootAimSet(float fVal)
{
    uint8_t ucSw = ucGetTaXySwap();

    if (ucGetShootCam() != 0u)
    {
        if (ucSw != 0u) { vSetTcCxAimPi(fVal); } else { vSetTcCyAimPi(fVal); }
    }
    else
    {
        if (ucSw != 0u) { vSetTcCxAim(fVal); }   else { vSetTcCyAim(fVal); }
    }
}

/* 调参量: 把当前值格式化成「:值」那半截（标签本身来自表） */static void vOledTuneText(uint8_t ucQty, char *pcBuf, uint32_t ulCap)
{
    switch ((emOledTuneQtyTdf)ucQty)
    {
    case OLED_QTY_GYRO_TARGET:
        (void)snprintf(pcBuf, (size_t)ulCap, "%.1f", (double)fGetLineGyroTarget());
        break;

    case OLED_QTY_ANGLE_TARGET:
        (void)snprintf(pcBuf, (size_t)ulCap, "%.1f", (double)fGetLineAngleTarget());
        break;

    case OLED_QTY_SHOOT_CX:
        /* 瞄准常数 —— 读 ta_xy_swap 选中那根、且**当前在用那台相机**那一对
         * （本车 swap=1 = 瞄准列 tc_cx_aim / 用 pi 时 tc_cx_aim_pi）。
         * 取整显示: 环里是 float, 但 ±5px 的步长下小数位只会闪。 */
        (void)snprintf(pcBuf, (size_t)ulCap, "%.0f", (double)fOledShootAimGet());
        break;

    case OLED_QTY_SHOOT_CONV_PX:
        (void)snprintf(pcBuf, (size_t)ulCap, "%.1f", (double)fGetTcConvPx());
        break;

    case OLED_QTY_OPENMV_LED:
    default:
        (void)snprintf(pcBuf, (size_t)ulCap, "%u", (unsigned)s_ucOpenmvLed);
        break;
    }
}

/* 调参量的步进。⚠️ 每个量的行为是**逐字搬**过来的, 别"顺手写漂亮一点":
 *   陀螺 = 先找最近档再步进; 图像角 = 十分位整数域算（免得浮点累加攒出脏值）;
 *   补光 = 0..100 步 10, **即调即发** "L,<n>" + 短鸣。 */
static void vOledTuneStep(uint8_t ucQty, uint8_t ucKeys)
{
    int8_t  iDir = 0;
    uint8_t i;

    if ((ucKeys & OLED_KEY_RIGHT) != 0u) { iDir = 1; }
    else if ((ucKeys & OLED_KEY_LEFT) != 0u) { iDir = -1; }
    if (iDir == 0) { return; }

    switch ((emOledTuneQtyTdf)ucQty)
    {
    case OLED_QTY_GYRO_TARGET:
    {
        float   fCur   = fGetLineGyroTarget();
        uint8_t ucBest = 0u;
        float   fBest  = 1e9f;

        for (i = 0u; i < OLED_GYRO_TGT_NUM; i++)   /* 先找当前值最接近的那一档 */
        {
            float fD = fCur - s_afGyroTgt[i];

            if (fD < 0.0f) { fD = -fD; }
            if (fD < fBest) { fBest = fD; ucBest = i; }
        }
        /* 带上界回绕: RIGHT 下一档, LEFT 上一档 */
        ucBest = (iDir > 0) ? (uint8_t)((ucBest + 1u) % OLED_GYRO_TGT_NUM)
                            : (uint8_t)((ucBest + OLED_GYRO_TGT_NUM - 1u) % OLED_GYRO_TGT_NUM);
        vSetLineGyroTarget(s_afGyroTgt[ucBest]);
        break;
    }

    case OLED_QTY_ANGLE_TARGET:
    {
        int32_t iT = iOledTenths(fGetLineAngleTarget()) + (int32_t)iDir;

        if (iT < -900) { iT = -900; }
        if (iT >  900) { iT =  900; }
        vSetLineAngleTarget((float)iT / 10.0f);
        break;
    }

    case OLED_QTY_SHOOT_CX:
    {
        /* 瞄准常数 ±5px（= OLED_SHOOT_CX_STEP_PX）—— 一按必出死区, 车跟着走到新目标。
         * ⚠️ 写的是 **ta_xy_swap 选中那根**（本车 = tc_cx_aim）**且当前在用那台相机**那一对:
         *    写错那一根/那一台的现象都是"按了一格车不动"（环读的是另一把尺）。
         *    这两层都由 fOledShootAimGet/vOledShootAimSet 收口（见它们的注释）。 */
        float fNew = fOledShootAimGet() + ((float)iDir * OLED_SHOOT_CX_STEP_PX);

        if (fNew < 0.0f)    { fNew = 0.0f; }
        if (fNew > 1920.0f) { fNew = 1920.0f; }
        vOledShootAimSet(fNew);
        break;
    }

    case OLED_QTY_SHOOT_CONV_PX:
    {
        /* 收敛阈值 ±0.5px, 钳 [2.0, 30.0]。用"×2 整数域"算, 免得浮点累加攒出 4.499999。
         * ⚠️ 下限 2px 顶在检测噪声带（±2px）与 1rpm 顶格步长（≈1.7px/拍）的下沿 ——
         *    再往下就会在带边界反复清零连续计数、**永远判不成收敛**（现象 = 车停在靶上
         *    却迟迟不 OK）。 */
        int32_t i2 = (int32_t)(fGetTcConvPx() * 2.0f) + (int32_t)iDir;

        if (i2 <  4) { i2 =  4; }      /* 2.0px */
        if (i2 > 60) { i2 = 60; }      /* 30.0px */
        vSetTcConvPx((float)i2 / 2.0f);
        break;
    }

    case OLED_QTY_OPENMV_LED:
    default:
    {
        int32_t iL = (int32_t)s_ucOpenmvLed + ((int32_t)iDir * (int32_t)OLED_OPENMV_LED_STEP);

        if (iL < 0)   { iL = 0; }        /* 两端夹住（回绕会从 100 跳 10, 不如夹住直观） */
        if (iL > 100) { iL = 100; }
        s_ucOpenmvLed = (uint8_t)iL;
        vRpiOpenmvLed((uint32_t)iL);     /* 即调即发（只发不等）; 未连接时 Pi 只记住 */
        break;
    }
    }
}

/* ==================================================================
 * 通用调参行（OLED_ACT_TUNE_PARAM, 2026-10 加）
 * ==================================================================
 * 与上面那 5 个 OLED_QTY_* 的分工**很清楚**: 那些各有硬编码行为（陀螺 5 档循环、
 * 补光即调即发…）, 是一条条人工加进来的现场量; 这一档**全按参数自己的元数据**走 ——
 * 值在哪、能不能写、步长多大、有哪些选项, 都由 Beam 表 / Pi 参数表回答。
 * 于是"想在屏上调一个新量" = 菜单表里加一行 `OLED_ITEM_P(...)`（控制台的调参块下拉
 * 就是这个名字）, **不用改固件**。
 *
 * 名字 → 值源（顺序固定）:
 *   ① Beam 表（cpstFindBeamParam, 两百多项, 含只读遥测）—— 值在**本机**, 现取现写, 零延迟;
 *   ② Pi 参数表（cpstRpiCfgFind, config.py 生成的那张）—— 值在**树莓派**, 走后台轮询
 *      缓存（app_rpi.c 的「Pi 参数缓存」段）: 屏上只读缓存, 改值发 "P,<名>,<值>"
 *      （**临时**生效, 不落盘）;
 *   ③ 都不认 → 显示 "?" 且按键长鸣: 排菜单的人写错名字**当场**看得见。
 *
 * 步进规则（按元数据自动选）:
 *   · 有选项表（"0=关|1=开" / "1=手动|3=自动"）→ 在**选项值之间循环**（两端回绕）——
 *     二值表就是勾选框的手感, 多值表就是下拉框的手感;
 *   · Beam 的 0/1 型（整型且量程恰为 [0,1], 与「蓝牙」/「调参」Tab 的勾选框同一判据）→ 翻转;
 *   · 其余数值 → 步长: Beam 按**量程/20 归到 1/2/5×10^k**, Pi 按**当前值的量级**
 *     （config.py 里没有量程元数据; 越界由 Pi 判并回 ERR —— 本端**不臆造**量程）;
 *     整型步长至少 1。
 *   · 只读项（Beam 的 pfSet == NULL）→ 长鸣一声, 不动。
 *
 * ⚠️ 本任务的栈只有 384 字, 而这里用 %g 打浮点（同上面那些调参行, 早已链进来）。
 *    别在这条路径上再加第二层深调用 / 大局部数组。
 */

/* 选项表里找值 fV 对应的**标签**（"0=关|1=开" → "开"）。返回 1 = 找到。
 * 两个控制台与两张表用的是同一套形状, 所以这一个函数服务两边。 */
static uint8_t ucOledOptLabel(const char *pcOpts, float fV, char *pcOut, uint32_t ulCap)
{
    const char *pc = pcOpts;
    int32_t     iWant = (int32_t)((fV >= 0.0f) ? (fV + 0.5f) : (fV - 0.5f));

    if ((pc == NULL) || (pcOut == NULL) || (ulCap == 0u)) { return 0u; }

    while (*pc != '\0')
    {
        int32_t     iVal = 0;
        uint8_t     ucNeg = 0u;
        uint8_t     ucDig = 0u;
        const char *pcLbl;

        if (*pc == '-') { ucNeg = 1u; pc++; }
        while ((*pc >= '0') && (*pc <= '9'))
        {
            iVal = (iVal * 10) + (*pc - '0');
            ucDig = 1u;
            pc++;
        }
        if ((ucDig != 0u) && (*pc == '='))
        {
            uint32_t ulN;

            pc++;
            pcLbl = pc;
            while ((*pc != '\0') && (*pc != '|')) { pc++; }
            if (((ucNeg != 0u) ? -iVal : iVal) == iWant)
            {
                ulN = (uint32_t)(pc - pcLbl);
                if (ulN >= ulCap) { ulN = ulCap - 1u; }
                memcpy(pcOut, pcLbl, (size_t)ulN);
                pcOut[ulN] = '\0';
                return 1u;
            }
        }
        while ((*pc != '\0') && (*pc != '|')) { pc++; }   /* 这一段没用: 跳到下一个 '|' */
        if (*pc == '|') { pc++; }
    }
    return 0u;
}

/* 选项表里第 ucIdx 个选项的**值**（给"循环"用）。返回 0 = 没有这一项。 */
static uint8_t ucOledOptValueAt(const char *pcOpts, uint8_t ucIdx, float *pfVal)
{
    const char *pc = pcOpts;
    uint8_t     ucN = 0u;

    if ((pc == NULL) || (pfVal == NULL)) { return 0u; }

    while (*pc != '\0')
    {
        int32_t iVal = 0;
        uint8_t ucNeg = 0u;
        uint8_t ucDig = 0u;

        if (*pc == '-') { ucNeg = 1u; pc++; }
        while ((*pc >= '0') && (*pc <= '9'))
        {
            iVal = (iVal * 10) + (*pc - '0');
            ucDig = 1u;
            pc++;
        }
        while ((*pc != '\0') && (*pc != '|')) { pc++; }
        if (*pc == '|') { pc++; }
        if (ucDig == 0u) { continue; }        /* 形状不对的段: 跳过（不当作一个选项） */

        if (ucN == ucIdx)
        {
            *pfVal = (ucNeg != 0u) ? -(float)iVal : (float)iVal;
            return 1u;
        }
        ucN++;
    }
    return 0u;
}

/* 当前值在选项表里是第几个（找不到返回 0xFF）。 */
static uint8_t ucOledOptIndexOf(const char *pcOpts, float fV)
{
    uint8_t ucI;
    float   fOpt;

    if (pcOpts == NULL) { return 0xFFu; }
    for (ucI = 0u; ucI < 32u; ucI++)
    {
        if (ucOledOptValueAt(pcOpts, ucI, &fOpt) == 0u) { return 0xFFu; }
        if (fOpt == fV) { return ucI; }
    }
    return 0xFFu;
}

/* 选项表是不是「布尔量」: 恰好两个选项、值恰为 {0,1}（顺序不限）。
 * 与「蓝牙」/「地图」两个控制台的勾选框判据同一口径 —— 这种行在屏上显示
 * √/×（2026-10-18 加, 用户口径）而不是选项标签, 一来省宽度、二来跟勾选框
 * 的形一致。注意 1=手动|3=自动 这种**不是**布尔（值不是 0/1）, 照旧显示标签。 */
static uint8_t ucOledOptsIsBool(const char *pcOpts)
{
    float f0, f1, f2;

    if (pcOpts == NULL) { return 0u; }
    if (ucOledOptValueAt(pcOpts, 0u, &f0) == 0u) { return 0u; }
    if (ucOledOptValueAt(pcOpts, 1u, &f1) == 0u) { return 0u; }
    if (ucOledOptValueAt(pcOpts, 2u, &f2) != 0u) { return 0u; }   /* 第三个还有 = 不是布尔 */
    (void)f2;
    return (uint8_t)(((f0 == 0.0f) && (f1 == 1.0f)) ||
                     ((f0 == 1.0f) && (f1 == 0.0f)));
}

/* 布尔值的显示字符: √(真) / ×(假)。 */
static const char *pcOledBoolMark(float fV)
{
    return (fV > 0.5f) ? "√" : "×";
}

/* 自定义步进（pcStep 数字串）→ 正浮点; 解析失败/≤0 返回 0 = 用系统步进。
 * 刻意不拉 atof/strtof（同 ucRpiCfgParseNum 那条理由: 不为它拉整套浮点解析）——
 * 这里只要「[整数][.小数]」正数这一种形状, 十几行够用。 */
static uint8_t ucOledParsePosFloat(const char *pc, float *pfVal)
{
    float   fV   = 0.0f;
    float   fSc  = 0.1f;
    uint8_t ucDig = 0u;

    if (pc == NULL) { return 0u; }
    while ((*pc >= '0') && (*pc <= '9'))
    {
        fV = (fV * 10.0f) + (float)(*pc - '0');
        ucDig = 1u;
        pc++;
    }
    if (*pc == '.')
    {
        pc++;
        while ((*pc >= '0') && (*pc <= '9'))
        {
            fV += fSc * (float)(*pc - '0');
            fSc *= 0.1f;
            ucDig = 1u;
            pc++;
        }
    }
    if ((ucDig == 0u) || (fV <= 0.0f)) { return 0u; }
    *pfVal = fV;
    return 1u;
}

/* |v| 的量级指数 e（10^e ≤ |v| < 10^(e+1); v=0 → 0）。
 * 刻意不用 log10f: 省一次 libm 链接, 而且这里只要整数指数（循环最多十几拍）。 */
static int8_t iOledMagExp(float fV)
{
    int8_t e = 0;

    if (fV < 0.0f) { fV = -fV; }
    while ((fV >= 10.0f) && (e < 38)) { fV *= 0.1f; e++; }
    while ((fV < 1.0f) && (fV > 0.0f) && (e > -38)) { fV *= 10.0f; e--; }
    return e;
}

/* 10 的 n 次方（n 可负; 只用于步长, 范围小, 浮点乘除法足够）。 */
static float fOledPow10(int8_t n)
{
    float f = 1.0f;

    while (n > 0) { f *= 10.0f; n--; }
    while (n < 0) { f *= 0.1f; n++; }
    return f;
}

/* Beam 参数的可读值文本（布尔量 → √/×; **映射短标签**次之; 真选项标签再次;
 * 否则数值 + 单位）。单位只在**真的有**时才拼（表里 "" 很多; 拼上会白白吃掉
 * 128px 里的几格）。pcMap = 调参行的显示映射（OLED_ITEM_PM, 空 = 用真表）。 */
static void vOledTuneBeamText(const stBeamParamTdf *pstB, const char *pcMap,
                              char *pcBuf, uint32_t ulCap)
{
    float fV = (pstB->pfGet != NULL) ? pstB->pfGet() : 0.0f;

    if (ucOledOptsIsBool(pstB->pcOptions) != 0u)
    {
        (void)snprintf(pcBuf, (size_t)ulCap, "%s", pcOledBoolMark(fV));
        return;
    }
    if ((pcMap != NULL) && (pcMap[0] != '\0') &&
        (ucOledOptLabel(pcMap, fV, pcBuf, ulCap) != 0u))
    {
        return;                                    /* 映射里有 → 显示短标签 */
    }
    if (ucOledOptLabel(pstB->pcOptions, fV, pcBuf, ulCap) != 0u) { return; }
    if (pstB->pcUnit[0] != '\0')
    {
        (void)snprintf(pcBuf, (size_t)ulCap, "%.5g%s", (double)fV, pstB->pcUnit);
    }
    else
    {
        (void)snprintf(pcBuf, (size_t)ulCap, "%.5g", (double)fV);
    }
}

/* 一个通用调参行右侧的值文本（拼在 "%s:%s" 的后半截）。
 * ⚠️ 纯显示: 步进/写值都现取真值, **不从这个字符串反解**（否则显示精度就成了数值精度）。 */
static void vOledTuneParamText(const stOledItemTdf *pstIt, char *pcBuf, uint32_t ulCap)
{
    const stBeamParamTdf   *pstB;
    const stRpiCfgParamTdf *pstR;

    if ((pcBuf == NULL) || (ulCap == 0u)) { return; }
    if (pstIt->pcParam == NULL) { (void)snprintf(pcBuf, (size_t)ulCap, "?"); return; }

    pstB = cpstFindBeamParam(pstIt->pcParam);
    if (pstB != NULL)
    {
        vOledTuneBeamText(pstB, pstIt->pcMap, pcBuf, ulCap);
        return;
    }

    pstR = cpstRpiCfgFind(pstIt->pcParam);
    if (pstR != NULL)
    {
        uint16_t    usIdx = (uint16_t)(pstR - &g_astRpiCfgParam[0]);
        float       fV;
        uint32_t    ulN;

        /* 链路不通 = 屏上不给值（"--"）: 缓存里的旧值此刻没有意义, 显示它只会让人
         * 以为"Pi 那边就是这个数"。 */
        if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            (void)snprintf(pcBuf, (size_t)ulCap, "--");
            return;
        }
        if (pstR->ucType == RPI_CFG_T_STR)
        {
            const char *pcS = pcRpiCfgStrAt(usIdx);

            (void)snprintf(pcBuf, (size_t)ulCap, "%s", (pcS[0] != '\0') ? pcS : "--");
            return;
        }
        if (ucRpiCfgCached(usIdx, &fV) == 0u)      /* ERR 也会走到这儿（缓存被判无效） */
        {
            (void)snprintf(pcBuf, (size_t)ulCap, "--");
            return;
        }
        if (ucOledOptsIsBool(pstR->pcOpts) != 0u)  /* 布尔量 → √/×（同 Beam 那条, 用户口径） */
        {
            (void)snprintf(pcBuf, (size_t)ulCap, "%s", pcOledBoolMark(fV));
            return;
        }
        if ((pstIt->pcMap != NULL) && (pstIt->pcMap[0] != '\0') &&
            (ucOledOptLabel(pstIt->pcMap, fV, pcBuf, ulCap) != 0u))
        {
            return;                                /* 映射短标签优先于真选项标签 */
        }
        if (ucOledOptLabel(pstR->pcOpts, fV, pcBuf, ulCap) == 0u)
        {
            (void)snprintf(pcBuf, (size_t)ulCap, "%.5g", (double)fV);
        }
        /* 最近 2s 内有过 ERR → 尾巴上挂个 "!": 这个值**没被 Pi 确认**（多见于刚按了
         * 越界的值）。挂在值后面而不是替换它, 是因为 ERR 可能属于另一行 —— 这样标
         * 至少不会把别人的错说成本行的值。 */
        ulN = (uint32_t)strlen(pcBuf);
        if ((ulN + 1u < ulCap) && (pcRpiCfgLastErr()[0] != '\0'))
        {
            pcBuf[ulN]      = '!';
            pcBuf[ulN + 1u] = '\0';
        }
        return;
    }

    (void)snprintf(pcBuf, (size_t)ulCap, "?");   /* 两张表都不认的名字 */
}

/* 通用调参行的 LEFT/RIGHT。**每次都以一声鸣叫收尾**（短=已生效/已发出, 长=拒绝）:
 * 换行动画期间文字可能还没落到新值上, 声音是操作员最可靠的那句"这一下算数了"。 */
static void vOledTuneParamStep(const stOledItemTdf *pstIt, uint8_t ucKeys)
{
    const stBeamParamTdf   *pstB;
    const stRpiCfgParamTdf *pstR;
    int8_t                  iDir = 0;

    if ((ucKeys & OLED_KEY_RIGHT) != 0u)      { iDir = 1; }
    else if ((ucKeys & OLED_KEY_LEFT) != 0u)  { iDir = -1; }
    if (iDir == 0) { return; }
    if (pstIt->pcParam == NULL) { return; }

    pstB = cpstFindBeamParam(pstIt->pcParam);
    if (pstB != NULL)
    {
        float fCur, fNew, fOpt;
        uint8_t ucIdx;
        const char *pcCycle;

        /* 只读项（ucRW=0 / 没有 setter）: 长鸣一声, 不动 */
        if ((pstB->pfSet == NULL) || (pstB->ucRW == 0u)) { return; }

        fCur = (pstB->pfGet != NULL) ? pstB->pfGet() : 0.0f;

        /* ① 选项表: 循环 —— **映射优先**（pcMap 非空 = 按映射的短标签集与顺序循环,
         * 值都是真表里存在的, 写盘校验管着）; 没映射 = 真选项表。 */
        pcCycle = ((pstIt->pcMap != NULL) && (pstIt->pcMap[0] != '\0'))
                  ? pstIt->pcMap : pstB->pcOptions;

        ucIdx = ucOledOptIndexOf(pcCycle, fCur);
        if (ucIdx != 0xFFu)
        {
            uint8_t ucNum = 0u;
            uint8_t ucNext;

            while (ucOledOptValueAt(pcCycle, ucNum, &fOpt) != 0u) { ucNum++; }
            if (ucNum == 0u) { return; }
            ucNext = (uint8_t)(((int16_t)ucIdx + iDir + ucNum) % ucNum);
            if (ucOledOptValueAt(pcCycle, ucNext, &fNew) == 0u) { return; }
        }
        else
        {
            /* ② 0/1 型（整型且量程恰为 [0,1], 同两个控制台的勾选框判据）→ 翻转 */
            if (((pstB->ucType == BEAM_VAL_U8) || (pstB->ucType == BEAM_VAL_I8)
                 || (pstB->ucType == BEAM_VAL_I32))
                && (pstB->fMin == 0.0f) && (pstB->fMax == 1.0f))
            {
                fNew = (fCur > 0.5f) ? 0.0f : 1.0f;
            }
            else
            {
                /* ③ 数值: **自定义步进优先**（pcStep, 2026-10-18 加 —— 积木行上配了
                 *    就听用户的）; 没配 = 系统步长（量程/20 归到 1/2/5×10^k,
                 *    "按十下能从这头走到那头"）。 */
                float fStepUser;

                if (ucOledParsePosFloat(pstIt->pcStep, &fStepUser) != 0u)
                {
                    fNew = fCur + ((float)iDir * fStepUser);
                }
                else
                {
                    float fSpan = pstB->fMax - pstB->fMin;
                    int8_t iE    = iOledMagExp(fSpan / 20.0f);
                    float fStep;

                    if (fSpan <= 0.0f)      { iE = 0; }
                    fStep = fOledPow10(iE);
                    if (fStep <= 0.0f)      { fStep = 1.0f; }
                    if ((pstB->ucType != BEAM_VAL_F32) && (fStep < 1.0f)) { fStep = 1.0f; }
                    fNew = fCur + ((float)iDir * fStep);
                }
            }
        }
        /* 先钳到量程内再落值 —— ucBeamApply 对超范围是**拒绝**（不是钳位）, 直接交给它
         * 到两端就落不下去; 这里钳完, 再补一条"到头了"的判据: 值没变就直接返回。
         * 落值本身仍走 ucBeamApply（蓝牙 $B 那条路: 同一套只读/量程判定, 不抄第二份）。 */
        if (fNew < pstB->fMin) { fNew = pstB->fMin; }
        if (fNew > pstB->fMax) { fNew = pstB->fMax; }
        if (fNew == fCur)                                  /* 到头了（或量程为一点） */
        {
            return;
        }
        (void)ucBeamApply(pstIt->pcParam, fNew);           /* 落值（只读/超范围时它自己拒绝） */
        return;
    }

    pstR = cpstRpiCfgFind(pstIt->pcParam);
    if (pstR != NULL)
    {
        uint16_t usIdx = (uint16_t)(pstR - &g_astRpiCfgParam[0]);
        float    fCur = 0.0f;
        float    fNew;
        float    fOpt;
        uint8_t  ucIdx;
        const char *pcCycle;

        /* str 型没有"步进"这一说（候选集/像素格式）: 长鸣告诉操作员"这一行调不动"。 */
        if (pstR->ucType == RPI_CFG_T_STR) { return; }

        if (emGetRpiLinkState() != RPI_LINK_OK) { return; }
        if (ucRpiCfgCached(usIdx, &fCur) == 0u) { return; }

        /* ① 选项表: 循环 —— 映射优先（同 Beam 那条）。 */
        pcCycle = ((pstIt->pcMap != NULL) && (pstIt->pcMap[0] != '\0'))
                  ? pstIt->pcMap : pstR->pcOpts;

        ucIdx = ucOledOptIndexOf(pcCycle, fCur);
        if (ucIdx != 0xFFu)
        {
            uint8_t ucNum = 0u;
            uint8_t ucNext;

            while (ucOledOptValueAt(pcCycle, ucNum, &fOpt) != 0u) { ucNum++; }
            if (ucNum == 0u) { return; }
            ucNext = (uint8_t)(((int16_t)ucIdx + iDir + ucNum) % ucNum);
            if (ucOledOptValueAt(pcCycle, ucNext, &fNew) == 0u) { return; }
        }
        else
        {
            /* ② 数值: **自定义步进优先**（pcStep, 2026-10-18 加）; 没配 = 系统步长
             * （按**当前值的量级** —— config.py 没有量程元数据: 10^(e-1) = 量级下面
             * 两档, 例: 0.35 → 0.01, 45 → 1, 1900 → 100; 整型至少 1）。 */
            float fStepUser;

            if (ucOledParsePosFloat(pstIt->pcStep, &fStepUser) != 0u)
            {
                fNew = fCur + ((float)iDir * fStepUser);
            }
            else
            {
                int8_t iE    = iOledMagExp(fCur);
                float  fStep = fOledPow10((int8_t)(iE - 1));

                if (iE == 0 && fCur == 0.0f) { fStep = fOledPow10(-2); }   /* 0 起步: 0.01 */
                if (fStep <= 0.0f) { fStep = 1.0f; }
                if ((pstR->ucType == RPI_CFG_T_I32) && (fStep < 1.0f)) { fStep = 1.0f; }
                fNew = fCur + ((float)iDir * fStep);
            }
        }

        vRpiCfgSet(usIdx, fNew);
        return;
    }

}

/**
  * @brief   执行一项。返回「这一拍之后该干嘛」—— 把散落的 s_ucMode 赋值收成一个返回值。
  * @retval  emOledResTdf
  *
  * ⚠️ `OLED_RES_ENTER_CONFIRM` 时**调用方必须立刻 return**: `vOledEnterConfirm` 已经把
  *    mode 设成 CONFIRM 了, 再往下走会把刚设好的状态冲掉（2026-09 踩过）。
  */
static uint8_t ucOledRunAct(const stOledItemTdf *pstIt)
{
    switch ((emOledActTdf)pstIt->ucAct)
    {
    case OLED_ACT_NOP:               /* 占位/未实现: 只给一声回执 */
        return OLED_RES_STAY;

    case OLED_ACT_CONFIRM_SHUTDOWN:
        vOledEnterConfirm(OLED_CONFIRM_SHUTDOWN);
        return OLED_RES_ENTER_CONFIRM;

    case OLED_ACT_POST_HANDSHAKE:
        if (ucOledPostReq(SCRIPT_REQ_HANDSHAKE, 0u) != 0u)
        {
            /* 这一投递会中止正在跑的巡线测试（后写者胜）→ 那一项的标签也该回到「开始」。
             * 不清的话它会一直显示「停止巡线测试」而实际跑的是别的。 */
            s_bLineTestOn  = 0u;
            s_bShootTestOn  = 0u;
            s_bShootPending = 0u;      /* 射靶测试同论（宽限窗内不清会让标签留在"停止"） */
        }
        return OLED_RES_STAY;

    case OLED_ACT_POST_TASK:
        /* 前置门①: 二维码得先设好 —— 任务预设内部按二维码选球/选形状, 没设就跑 =
         * 跑到那一步长鸣且一步不动, 现场像「跑一半卡住」。
         * ⚠️ 判据 OLED_QR_IS_SET(100~399, 与 IF 条件/排爆同口径), **不是**「!=-1」。
         * 前置门②(2026-10 加): Pi 得在线 —— 这三条任务预设**第一步就是视觉步骤**,
         * 预设内部一点等重连的窗口都没有, Pi 不在 = 发不出指令、等到本步超时长鸣,
         * 现场同样像「跑一半卡住」。两个门都过才投递。 */
        if (OLED_QR_IS_SET(iGetQrCode()) == 0)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_QR);
            return OLED_RES_ENTER_CONFIRM;
        }
        if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_PI);
            return OLED_RES_ENTER_CONFIRM;
        }
        if (ucOledPostReq(SCRIPT_REQ_TASK, pstIt->ucArg) != 0u)
        {
            s_bLineTestOn  = 0u;
            s_bShootTestOn  = 0u;
            s_bShootPending = 0u;
        }
        return OLED_RES_STAY;

    case OLED_ACT_LINE_TEST:
        /* 前置门(2026-10 改): **Pi 在线**, 不再查二维码 —— 巡线测试那条预设从头到尾
         * 不读二维码内容（本来就不会用到）, 而起流/巡线帧全靠 Pi, 所以真正的
         * 前置条件是链路 OK; 原来那道二维码门只会白白拦住台架上的「不设码直接测」。 */
        if (ucOledLineTestOn() != 0u)
        {
            vScriptAbort();           /* 脚本任务在下个 20ms 拍里就停 */
            s_bLineTestOn = 0u;
        }
        else if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_PI);
            return OLED_RES_ENTER_CONFIRM;
        }
        else if (ucOledPostReq(SCRIPT_REQ_LINE_TEST, 0u) != 0u)
        {
            /* ⚠️ **只在投递成功时才翻标签**: 失败时若照样翻成「停止」, 下一次按键就会
             * 走到上面那个 abort 分支 —— 看着是「开始」实际发的是「停」。 */
            s_bLineTestOn    = 1u;
            s_bShootTestOn   = 0u;    /* 后写者胜: 别的测试起来了, 那一条的影子一律清掉 */
            s_bShootPending  = 0u;
            s_ulLineTestTick = xTaskGetTickCount();
        }
        return OLED_RES_STAY;

    case OLED_ACT_SHOOT_TEST:
        /* 「射靶测试」开关（2026-10-09 加）—— 与上面那条**同构**, 三处不同:
         *  ① 投的是 `SCRIPT_REQ_TASK`（**带 ucArg = 预设 id**）: 于是白送
         *     「QR 前置门 + 机械臂没展开过就先阻塞展开 + 主摄预热」。**展开那一步是必须的**
         *     —— 预设第一行是 `ARM_ACT_16`（识别标靶位姿）, 从折叠位直接摆过去会撞器件。
         *  ② 停 = `vScriptAbort()`: 那条保持积木在**自己的闭环里**查中止位, 所以是"立刻停"
         *     （不是等到步骤边界）, 且来得及给 Pi 发 "8" 停流。
         *  ③ 真值判据带**预设 id**（`ucOledShootTestOn(pstIt->ucArg)`）—— 所以
         *     app_oled.c 不硬编码任何预设号。
         * ⚠️ 目标色来自**二维码第二位**（同所有打靶步骤）→ 没设二维码就跑 = 跑到那步
         *    长鸣一步不动, 像"跑一半卡住"（这就是这道前置门存在的理由）。 */
        if (ucOledShootTestOn(pstIt->ucArg) != 0u)
        {
            vScriptAbort();
            /* ⚠️ 三个标志一起清, 且**宽限窗的起点也清 0** —— 否则「按停止 → 500ms 内想重开」
             * 会被宽限窗判成"还在跑"（走停止分支）, 看着像"按了没反应"。 */
            s_bShootTestOn    = 0u;
            s_bShootPending   = 0u;
            s_ulShootTestTick = 0u;
        }
        else if (OLED_QR_IS_SET(iGetQrCode()) == 0)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_QR);
            return OLED_RES_ENTER_CONFIRM;
        }
        else if (ucOledPostReq(SCRIPT_REQ_TASK, pstIt->ucArg) != 0u)
        {
            /* ⚠️ 同上面那条: **只在投递成功时才翻标签**（失败时翻 = 下次按键走成"停"）。
             * s_bShootPending 是这次投递的凭据 —— 它一路把标签撑到预设真跑起来（见那条函数）。 */
            s_bShootTestOn    = 1u;
            s_bShootPending   = 1u;
            s_bLineTestOn     = 0u;
            s_ulShootTestTick = xTaskGetTickCount();
        }
        return OLED_RES_STAY;

    case OLED_ACT_LASER_TOGGLE:
        /* 本页唯一**会动手**的项: 直接翻继电器。⚠️ 判据与写引脚都走 Beam 的同一个量
         * （vRelayOn/vRelayOff 收口）, 本页**不记影子状态** —— 否则路线里
         * STEP_LASER_ON/OFF 或 vLaserShoot 跑完, 这里显示的就跟实物对不上。 */
        if (ucGetLaserOn() != 0u) { vRelayOff(); }
        else                      { vRelayOn();  }
        return OLED_RES_STAY;

    case OLED_ACT_FILL_TOGGLE:
        /* 就地翻转补光灯（2026-10 加, TIM9 PWM）—— 与上面激光那条同构:
         * 判据与写入走同一个量（vFillLightSet 是全工程唯一的占空比出口且顺手写 Beam,
         * 本页不记影子）。开 = FILL_LIGHT_ON_DUTY（同积木 STEP_BUGUANG_KAI 不勾亮度
         * 时的默认档, 一处概念）; 关 = 0。
         * ⚠️ 任务自动补光是后写者胜: 任务步末的 vFillLightTaskEnd 会把这里刚开的灯
         *    灭掉、这里也能关掉任务步骤开着的灯 —— 既有语义, 不是 bug。 */
        if (usGetFillDuty() != 0u) { vFillLightSet(0u); }
        else                       { vFillLightSet((uint16_t)FILL_LIGHT_ON_DUTY); }
        return OLED_RES_STAY;

    case OLED_ACT_PHOTO:
        /* ⚠️ 去程三跳（本端发"9"给 Pi, Pi 转告 OpenMV）, **结果走第四跳异步回来**
         * （TAG=0x06 → Beam 的 photo_done_tick）。所以这里**只发不等**,
         * 短鸣只表示"指令已发出", 成败看那一行闪的「拍照 成功!/失败!」。 */
        vRpiPhotoTake();
        return OLED_RES_STAY;

    case OLED_ACT_ENTER_SUB:
        if (pstIt->pstSub == 0) { return OLED_RES_STAY; }
        s_pstSubList = pstIt->pstSub;
        s_ucCursor   = 0u;
        s_ucTop      = 0u;               /* 进子菜单从第一条看起（vOledWindowSync 本会自愈） */
        vOledCursorNormalize(pstIt->pstSub);
        return OLED_RES_ENTER_SUB;

    case OLED_ACT_RUN_PRESET:
        /* 与 POST_TASK 同一套投递, **只少了二维码前置门** —— 台架上跑测试预设
         * 不用先去设个二维码。臂待命那条行为照旧（同一个 SCRIPT_REQ_TASK）。 */
        if (ucOledPostReq(SCRIPT_REQ_TASK, pstIt->ucArg) != 0u)
        {
            s_bLineTestOn  = 0u;
            s_bShootTestOn  = 0u;
            s_bShootPending = 0u;
        }
        return OLED_RES_STAY;

    case OLED_ACT_ABORT_SCRIPT:
        /* 中止点在**步骤边界**（见 app_script.c）, 不是从半路掐底盘/臂 —— 所以这一声
         * 短鸣的意思是"收到", 车可能还要走到这一步末尾才停。 */
        vScriptAbort();
        return OLED_RES_STAY;

    case OLED_ACT_GYRO_SRC:
        /* 翻转主/副陀螺源。⚠️ 切源瞬间 yaw 会跳到另一颗的零点（两颗模块零点不同）,
         * 做绝对航向闭环的步骤会把它当大误差 —— 排表的人要保证**先停车再切**。 */
        vSetGyroSrc((ucGetGyroSrc() != 0u) ? 0u : 1u);
        return OLED_RES_STAY;

    case OLED_ACT_GYRO_CAL:
        /* 陀螺校准（2026-10 加）: 投给脚本任务 —— 校准要阻塞 M0≈21s / HWT101≈7s,
         * 只能在不发布 yaw 的任务里跑（docs/sensors.md 的两条硬约束）。
         * 校准对象是**有效源**那颗, 想校另一颗先去切「陀螺源」。
         * ⚠️ busy 期间再按 = 忽略（请求槽虽然空着也别再投 —— 车必须保持静止,
         *    中途再来一遍只会把刚估好的零偏搅掉）。成败走标签闪 1s（见 pcOledLiveLabel）。 */
        if (ucGetGyroCalBusy() == 0u)
        {
            (void)ucOledPostReq(SCRIPT_REQ_GYRO_CAL, 0u);
        }
        return OLED_RES_STAY;

    case OLED_ACT_TUNE_QTY:          /* 值靠 LEFT/RIGHT 调; MAIN 在这里 = 退出（见按键段） */
    case OLED_ACT_TUNE_PARAM:        /* 同上（通用调参行按 MAIN 也是退出, 见按键段的判据） */
        return OLED_RES_STAY;

    case OLED_ACT_TIMER:
        /* 计时: ucArg = 计时动作（emTimerCmdTdf, 开始/结束/重置）。与地图 Tab 的
         * STEP_TIMER 共用 vTimerCmd, 只改状态页第 4 行的秒表, 不动车、不阻塞。
         * 越界值由 vTimerCmd 的 default 忽略（排表侧下拉只会给 0~2）。 */
        vTimerCmd((emTimerCmdTdf)pstIt->ucArg);
        return OLED_RES_STAY;

    case OLED_ACT_PI_MAIN_RESTART:   /* 重启main（Pi 侧单帧直达, 不走 "r" 双发门） */
    case OLED_ACT_PI_SVC_RESTART:    /* 重启服务（systemctl restart; 停止状态执行 = 直接启动,
                                      * 所以它同时就是「开始」—— 但若服务已被手动 stop,
                                      * 串口那头没人收指令, 这一下只会落空） */
    case OLED_ACT_PI_LOG_CLEAR:      /* 清屏Log（GKS_Web 清屏按钮同一条通道） */
    {
        emRpiTaskTdf emTask;

        /* 前置门: 指令是发给 Pi 的, 链路不在 = 发给谁? （同关机那道门） */
        if (emGetRpiLinkState() != RPI_LINK_OK)
        {
            vOledEnterConfirm(OLED_CONFIRM_NO_PI);
            return OLED_RES_ENTER_CONFIRM;
        }
        /* 「重启服务/重启main」发出后串口会断（几分钟）—— 第 1 页 RPI 行变
         * Waiting/Disconnected 是预期, 服务自愈回来会重新握手置 OK。 */
        switch ((emOledActTdf)pstIt->ucAct)
        {
        case OLED_ACT_PI_MAIN_RESTART: emTask = RPI_TASK_PI_MAIN_RESTART; break;
        case OLED_ACT_PI_SVC_RESTART:  emTask = RPI_TASK_PI_SVC_RESTART;  break;
        default:                       emTask = RPI_TASK_PI_LOG_CLEAR;    break;
        }
        vRpiSendTaskCmd(emTask);
        return OLED_RES_STAY;
    }

    default:                          /* NONE 等不该出现的值 */
        return OLED_RES_STAY;
    }
}

/* 画一张菜单表（4 行窗口）。bSub=1 时表尾多一条固件自带的「返回」—— 它在**条目空间**里,
 * 随窗口一起滚, 不再挤死在第 4 行。
 *
 * ⚠️⚠️ **四行逐个处理: 要么画、要么清** —— 帧缓冲是持久的（`vOledClear` 只在初始化时调）,
 *     窗口一挪, 上一帧的内容就得靠这一帧的清行带走, 否则会"留着上一页/上一屏的内容"。
 * ⚠️ 行内容 = **条目序号** (s_ucTop + k), 不是表下标 —— 窗口就是在这里生效的。
 */
static void vOledMenuDraw(const stOledListTdf *pstList, uint8_t bSub)
{
    uint8_t ucItems = ucOledVisibleCount(pstList);                     /* 0..32 */
    uint8_t ucN     = (uint8_t)(ucItems + ((bSub != 0u) ? 1u : 0u));   /* 条目总数 0..33 */
    uint8_t ucHW    = ((ucN > OLED_MAX_ROWS) ? OLED_LABEL_W : OLED_W); /* 高亮宽度: 有滚动条时 120 */
    uint8_t k;

    for (k = 0u; k < OLED_MAX_ROWS; k++)
    {
        uint8_t ucY = (uint8_t)(OLED_Y1 + (k * OLED_ROW_H));
        uint8_t ucE = (uint8_t)(s_ucTop + k);          /* 这一行的**条目序号** */
        uint8_t ucIdx;

        if (ucE >= ucN)                                /* 窗口外/表尾 → 空行 */
        {
            vOledClearArea(OLED0, 0u, ucY, OLED_W, OLED_ROW_H);
            continue;
        }
        if ((bSub != 0u) && (ucE == ucItems))          /* 固件的「返回」= 最后一个条目 */
        {
            vOledDrawRowX(ucY, (uint8_t)((s_ucCursor == OLED_CURSOR_BACK) ? s_ucAnimOfs : 0u), "返回");
            if (s_ucCursor == OLED_CURSOR_BACK) { vOledHighlightRow(ucY, ucHW); }
            continue;
        }
        ucIdx = ucOledEntryIndex(pstList, ucE);
        if (ucIdx == 0xFFu)                            /* 不该发生（0xFF 契约的兜底） */
        {
            vOledClearArea(OLED0, 0u, ucY, OLED_W, OLED_ROW_H);
            continue;
        }
        {
            const stOledItemTdf *pIt = &pstList->pstItems[ucIdx];   /* C90: 块首 */
            /* 换行动画只作用在**选中那一行**上（其它行恒 0 ⇒ 与加动画之前逐字节一致）。 */
            uint8_t ucXOfs = (uint8_t)((ucIdx == s_ucCursor) ? s_ucAnimOfs : 0u);

            if (ucOledIsTuneAct(pIt->ucAct) != 0u)
            {
                char acVal[24];

                if (pIt->ucAct == (uint8_t)OLED_ACT_TUNE_PARAM)
                {
                    /* 通用调参行（按参数名, 2026-10 加）: Beam 现取 / Pi 读缓存。
                     * ⚠️ 屏上**同时**显示哪几行 = 哪几个 Pi 参数被轮询 —— 就在这儿标
                     *    （清除在 vOledTask 每拍开头, 见那里的注释）。 */
                    const stRpiCfgParamTdf *pstR;

                    vOledTuneParamText(pIt, acVal, (uint32_t)sizeof(acVal));
                    pstR = (cpstFindBeamParam(pIt->pcParam) != NULL)
                               ? NULL : cpstRpiCfgFind(pIt->pcParam);
                    if (pstR != NULL)
                    {
                        vRpiCfgWant((uint16_t)(pstR - &g_astRpiCfgParam[0]));
                    }
                }
                else
                {
                    vOledTuneText(pIt->ucArg, acVal, (uint32_t)sizeof(acVal));
                }
                vOledDrawRowfX(ucY, ucXOfs, "%s:%s", pIt->pcLabel, acVal);
            }
            else if ((pIt->ucAct == (uint8_t)OLED_ACT_LASER_TOGGLE)
                     && (ucOledListHasAct(pstList, (uint8_t)OLED_ACT_SHOOT_TEST) != 0u))
            {
                /* 「射靶测试」页的**激光那一行**右边拼上实时偏差: `激光:开 d=-3px`（112px）。
                 * ⚠️ 挂在**含 SHOOT_TEST 的那张表**上（照抄「让出成功!」横幅那条"按语义挂"的写法）
                 *    —— 第 3 页那张表没有它, 所以那边的激光行照旧是干净的 `激光:开`。
                 * ⚠️ 只在保持环真在跑（shoot_state != 0）时拼: 没跑时显示 `d=0px` 只会误导
                 *    （那是上一次会话的残留值）。
                 * ⚠️ 这是**唯一**新增的 printf 行（走 vOledDrawRowf）; 两个字段都是整数, 不做浮点
                 *    —— 本任务栈只有 384 字, 别在这里加 `%f`。
                 * ⚠️ 宽度: `激光:开 d=-12px` 恰好 120px（= 标签预算）; 三位数偏差（`d=-999px`）
                 *    会被右侧滚动条裁掉 —— 这是运行时标签, 控制台拦不到, 属已知限制。 */
                const char *pcLbl = pcOledLiveLabel(pIt);

                if (ucGetShootState() != 0u)
                {
                    vOledDrawRowfX(ucY, ucXOfs, "%s d=%dpx", pcLbl, (int)fGetShootErr());
                }
                else
                {
                    vOledDrawRowX(ucY, ucXOfs, pcLbl);
                }
            }
            else
            {
                /* ⚠️ 静态标签行走 vOledDrawRow（**不带 printf**）—— acBuf[44] + vsnprintf
                 * 是全工程最深的 printf 用法, 别给每一行都加上（本任务栈只有 384 字）。 */
                vOledDrawRowX(ucY, ucXOfs, pcOledLiveLabel(pIt));
            }
            if (ucIdx == s_ucCursor) { vOledHighlightRow(ucY, ucHW); }
        }
    }
}

/* 右侧竖直滚动条的**几何内核**（2026-10-18 抽出来 —— 第 2 页小菜单与菜单引擎共用同一套设计）。
 * 只在**放不下**（条目数 > 可见行数）时才画; 参数化「可见行数/轨道位置」之后,
 * 菜单引擎那条（4 行、轨道罩整屏）与二维码页那条（3 行、轨道只罩菜单区 y=16..63,
 * 不碰行 1 的 128px 状态行）是同一个函数。
 *
 * ⚠️⚠️ **必须画在所有行、空行清理、高亮反色之后**:
 *     那几步都会写到 x >= 120 这 8 列（vOledDrawRow* 清的是整行 128px）。
 *     画早了就会被它们擦掉, 或被反色刷成"白底白块"。
 *
 * 几何（屏 128x64; 以 ucY0/ucH 为轨道窗口）:
 *   预留带  x = 120..127（8px, 文字预算因此是 120px）
 *   轨道    x = 122..125（4px 宽, 空心矩形, 高 = ucH）
 *   滑块    x = 122..125, 高/位按下面公式, 实心矩形
 */
static void vOledScrollbarDrawGeo(uint8_t ucN, uint8_t ucRows, uint8_t ucTop,
                                  uint8_t ucY0, uint8_t ucH)
{
    uint8_t ucThumbH, ucThumbY, ucMax, ucSpan;

    /* 先把整条带子清干净: 帧缓冲是持久的, 从"32 项"翻到"4 项"时不清就会留着旧滑块。
     * （每行的整行清其实也顺带擦到了这里, 但**别依赖那件事** —— 哪天有人为省事把
     *   vOledDrawRow 的清行改窄, 残影就回来了。自己清一次, 一处收敛。） */
    vOledClearArea(OLED0, OLED_LABEL_W, ucY0, OLED_SCROLLBAR_W, ucH);

    if (ucN <= ucRows) { return; }               /* 放得下 → 一个像素都不画（老页面零变化） */

    /* 轨道: 空心矩形（左右两条竖线 + 上下两条边）—— 一眼看出这是"滚动条"。
     * ⚠️ 别在这条上用 Height==0 的空心矩形（vOledDrawRectangle 的 Y+Height-1 会 uint8 下溢）;
     *    两个调用点最低 48 高, 安全。 */
    vOledDrawRectangle(OLED0, OLED_BAR_X, ucY0, OLED_BAR_W, ucH, OLED_UNFILLED);

    /* 滑块高: 按"可见行数 / 总条目数"等比。菜单引擎 N=5 → 51px; N=32 → 8px; N=33 → 7px;
     * 二维码页 N=4/3 行 → 36px。下限 5 只作防御（现有点到不了; 抬上限前先核这里）。 */
    ucThumbH = (uint8_t)(((uint16_t)ucH * (uint16_t)ucRows) / ucN);
    if (ucThumbH < 5u) { ucThumbH = 5u; }

    /* 滑块位置: 顶行条目序号 / 最大序号 × 可滚动行程。
     * ⚠️ 乘法先升到 uint16_t: 菜单引擎最大是 57 * 29 = 1653 > 255。 */
    ucSpan = (uint8_t)(ucH - ucThumbH);
    ucMax  = (uint8_t)(ucN - ucRows);            /* ucTop 的上界（两个调用方的窗口同步都保证） */
    if (ucTop > ucMax) { ucTop = ucMax; }        /* 防御 */
    ucThumbY = (uint8_t)(ucY0
                         + ((ucMax == 0u) ? 0u
                                          : (uint8_t)(((uint16_t)ucSpan * (uint16_t)ucTop)
                                                      / (uint16_t)ucMax)));

    vOledDrawRectangle(OLED0, OLED_BAR_X, ucThumbY, OLED_BAR_W, ucThumbH, OLED_FILLED);
}

/* 菜单引擎那条（4 行、轨道罩整屏）—— 老入口, 行为与抽几何之前逐字节一致。 */
static void vOledScrollbarDraw(const stOledListTdf *pstList, uint8_t bSub)
{
    uint8_t ucN = ucOledEntryCount(pstList, bSub);

    if ((ucN > OLED_MAX_ROWS) && (s_ucTop > (uint8_t)(ucN - OLED_MAX_ROWS)))
    {
        s_ucTop = (uint8_t)(ucN - OLED_MAX_ROWS);    /* 防御: 允许本函数单独被调用（原逻辑） */
    }
    vOledScrollbarDrawGeo(ucN, OLED_MAX_ROWS, s_ucTop, 0u, OLED_BAR_H);
}

/* 菜单页（顶层菜单 / 子菜单共用）—— 按键 + 绘制 */
/* ---- 数值类调参行的按住连发（见 vOledPageMenu 里的说明）----
 * START=8 拍(≈400ms, 单击的正常时长之内不起连发); 之后每 2 拍(≈100ms)合成一步
 * = 10 步/秒。步长本身由各行自定（含自定义步进）, 所以"快"是相对的 —— 0.1 步长
 * 的行是 1°/s, 5px 步长的行是 50px/s。要更快改 TUNE_REPEAT_TICKS=1（20 步/s）。 */
#define TUNE_REPEAT_START_TICKS 8u
#define TUNE_REPEAT_TICKS       2u

static uint8_t s_aucTuneRepTicks[2];   /* [0]=LEFT [1]=RIGHT: 按住计时（拍） */
static uint8_t s_aucTuneRepOn[2];      /* 连发已接管（释放 Click 待吃） */

/* 这条调参行是不是**数值类**（连发只对它生效, 用户口径）:
 *   TUNE_QTY: 陀螺目标是 5 档循环（枚举感）→ 不连发; 其余（图像目标/补光亮度/
 *             瞄准列/收敛阈值）数值 → 连发。
 *   TUNE_PARAM: 布尔（屏上 √/×, 翻转）与 str（只读）→ 不连发; 其余数值 → 连发。 */
static uint8_t ucOledTuneRowNumeric(const stOledItemTdf *pstIt)
{
    if (pstIt == NULL) { return 0u; }

    if (pstIt->ucAct == (uint8_t)OLED_ACT_TUNE_QTY)
    {
        return (uint8_t)((emOledTuneQtyTdf)pstIt->ucArg != OLED_QTY_GYRO_TARGET);
    }
    if (pstIt->ucAct == (uint8_t)OLED_ACT_TUNE_PARAM)
    {
        const stBeamParamTdf   *pstB = cpstFindBeamParam(pstIt->pcParam);
        const stRpiCfgParamTdf *pstR;

        if (pstB != NULL)
        {
            return (uint8_t)(ucOledOptsIsBool(pstB->pcOptions) == 0u);
        }
        pstR = cpstRpiCfgFind(pstIt->pcParam);
        if (pstR != NULL)
        {
            return (uint8_t)((pstR->ucType != RPI_CFG_T_STR)
                             && (ucOledOptsIsBool(pstR->pcOpts) == 0u));
        }
    }
    return 0u;                             /* 名字两张表都不认: 该行本来就调不动 */
}

static void vOledPageMenu(uint8_t ucKeys)
{
    uint8_t              bSub    = (uint8_t)((s_ucMode == OLED_MODE_SUBMENU) ? 1u : 0u);
    const stOledListTdf *pstList = bSub ? s_pstSubList : pstOledPageList(s_ucPage);
    uint8_t              bTune;

    if (pstList == 0) { return; }        /* 不该发生（MENU 页一定有表） */

    /* 换行动画**每拍先收回一点, 再处理按键** —— 顺序反了就会把起跳那一帧(8px)吃掉:
     * 按键先置 8、同一拍又 tick 成 4, 于是实际只有 4→0 一帧。 */
    vOledAnimTick();

    /* 当前光标是不是停在「调参」那类行上（只有它吃 LEFT/RIGHT 调值）。
     * ⚠️ **必须先算它、再决定翻页** —— 2026-10-09 起**顶层页也吃 LEFT/RIGHT 调值**
     *    （原来只有子菜单吃）。代价: 光标停在调参行上时那一页翻不动（要翻页先按 UP/DOWN
     *    挪开）—— 这正是子菜单一直以来的规则, 现在只是把同一条规则放宽到顶层页。
     *    现有顶层页**一个调参行都没有** → 对老页面零影响（回归判据）。 */
    bTune = (uint8_t)((s_ucCursor != OLED_CURSOR_BACK)
                      && (s_ucCursor < pstList->ucCount)
                      && (ucOledIsTuneAct(pstList->pstItems[s_ucCursor].ucAct) != 0u));

    /* ---- 数值类调参行的「按住连发」（2026-10-18 加, 用户口径「长按快进快退」）----
     * 按键层只有 Click（**释放**时才发一次）, 所以"一直按着"由本任务自己查
     * ucKeyIsPressed(): 光标停在**数值类调参行**上时, 按住 LEFT/RIGHT 满
     * TUNE_REPEAT_START_TICKS(≈400ms) 起, 每 TUNE_REPEAT_TICKS(=2 拍 ≈100ms)
     * **合成**一份键掩码 —— 走的就是下面既有的调值分派, 没有第二套步进逻辑。
     *
     * ⚠️ 刻意**不放在按键层**: 那样所有页面按住都会连发（翻页连翻、状态页连跳）,
     * 单击/按住的手感全变 —— 用户明确要求**只对调参项生效**。非调参处:
     * 计时清零、什么都不合成, 单击手感与加连发之前逐拍一致（Click 在释放时
     * 照常到达, 没有引入任何延迟）。
     * ⚠️ 连发过的那次按住, 释放 Click 要**吃掉**（repOn 标志）—— 否则松手
     * 会"回弹"多走一步。 */
    {
        uint8_t ucDirBit[2];
        uint8_t ucDirKey[2];
        uint8_t i;

        ucDirBit[0] = OLED_KEY_LEFT;   ucDirKey[0] = (uint8_t)KEY_LEFT;
        ucDirBit[1] = OLED_KEY_RIGHT;  ucDirKey[1] = (uint8_t)KEY_RIGHT;
        for (i = 0u; i < 2u; i++)
        {
            uint8_t ucHeld = ucKeyIsPressed((emKeyDevNumTdf)ucDirKey[i]);

            if (ucHeld != 0u)
            {
                s_aucTuneRepTicks[i]++;
                if ((bTune != 0u)
                    && (ucOledTuneRowNumeric(&pstList->pstItems[s_ucCursor]) != 0u))
                {
                    if (s_aucTuneRepOn[i] != 0u)
                    {
                        if (s_aucTuneRepTicks[i] >= TUNE_REPEAT_TICKS)
                        {
                            s_aucTuneRepTicks[i] = 0u;
                            ucKeys |= ucDirBit[i];       /* 合成一步 */
                        }
                    }
                    else if (s_aucTuneRepTicks[i] >= TUNE_REPEAT_START_TICKS)
                    {
                        s_aucTuneRepOn[i]  = 1u;
                        s_aucTuneRepTicks[i] = 0u;
                        ucKeys |= ucDirBit[i];           /* 连发第一步 */
                    }
                }
                /* 非数值调参行: 只计时不合成 —— 单击手感与加连发之前一致。 */
            }
            else
            {
                if (s_aucTuneRepOn[i] != 0u)
                {
                    if ((ucKeys & ucDirBit[i]) != 0u)
                    {
                        ucKeys &= (uint8_t)~ucDirBit[i]; /* 吃掉释放 Click（不回弹） */
                    }
                    s_aucTuneRepOn[i] = 0u;
                }
                s_aucTuneRepTicks[i] = 0u;
            }
        }
    }

    /* 顶层: LEFT/RIGHT 翻页（两端夹住不循环）。⚠️ 光标在调参行上时那两个键让位给调值。
     * 子菜单里那两个键见下。 */
    if ((bSub == 0u) && (bTune == 0u))
    {
        if (ucOledTurnPage(ucKeys) != 0u) { return; }
    }

    if ((ucKeys & (OLED_KEY_UP | OLED_KEY_DOWN)) != 0u)
    {
        uint8_t ucPrev = s_ucCursor;   /* C90: 块首 */

        vOledCursorStep(pstList, bSub, ((ucKeys & OLED_KEY_UP) != 0u) ? -1 : 1);
        /* 换行动画只在**光标真的换了行**时起跳（到头/全禁用时 vOledCursorStep 原地不动,
         * 那一下不该有动画）。 */
        if (s_ucCursor != ucPrev) { vOledAnimStart(); }
    }
    else if ((bTune != 0u) && ((ucKeys & (OLED_KEY_LEFT | OLED_KEY_RIGHT)) != 0u))
    {
        /* ⚠️ 条件里**不再有 `bSub != 0u`** —— 顶层页的调参行（如射靶测试页的「瞄准列」
         * 「收敛阈值」）现在也在这里调值。翻页已在上面让位（见 bTune 那段注释）。
         * ⚠️ 两个动作码各有各的步进实现（OLED_QTY_* 那 5 个量是硬编码行为, TUNE_PARAM
         *    按参数元数据）—— 分派就在这儿, 别再往下塞第二层判据。 */
        if (pstList->pstItems[s_ucCursor].ucAct == (uint8_t)OLED_ACT_TUNE_PARAM)
        {
            vOledTuneParamStep(&pstList->pstItems[s_ucCursor], ucKeys);
        }
        else
        {
            vOledTuneStep(pstList->pstItems[s_ucCursor].ucArg, ucKeys);
        }
    }
    else if ((ucKeys & OLED_KEY_MAIN) != 0u)
    {
        if ((bSub != 0u) && (s_ucCursor == OLED_CURSOR_BACK))
        {
            s_ucMode   = OLED_MODE_NAV;      /* 「返回」 */
            s_ucCursor = 0u;
            s_ucTop    = 0u;                 /* 复位只是"意图清楚", vOledWindowSync 本会自愈 */
            return;                          /* 本拍不画: 下一拍画父层, 免得闪一帧刚退出的子菜单 */
        }
        else
        {
            const stOledItemTdf *pIt = &pstList->pstItems[s_ucCursor];
            uint8_t              ucRes;

            /* ⚠️ 派发前**再判一次启用**: 不然被藏起来的项还能被 MAIN 激活, 禁用就形同虚设 */
            if (pIt->ucEnabled == 0u)
            {
                /* 禁用项: 本拍不派发 */
            }
            else if ((bSub != 0u) && (ucOledIsTuneAct(pIt->ucAct) != 0u))
            {
                /* 调参行按 MAIN = 退出子菜单（与「返回」同义）—— 原有行为:
                 * 数值行没有 MAIN 语义, 免得只有一条路出去。 */
                s_ucMode   = OLED_MODE_NAV;
                s_ucCursor = 0u;
                s_ucTop    = 0u;
                return;                          /* 同「返回」: 本拍不画, 下一拍画父层 */
            }
            else
            {
                ucRes = ucOledRunAct(pIt);
                if (ucRes == OLED_RES_ENTER_SUB)
                {
                    s_ucMode = OLED_MODE_SUBMENU;
                }
                else if (ucRes == OLED_RES_ENTER_CONFIRM)
                {
                    return;      /* confirm 已把 mode 设好, 别往下走把它冲掉 */
                }
            }
        }
    }
    else if ((bSub != 0u) && ((ucKeys & OLED_KEY_LEFT) != 0u))
    {
        /* 子菜单里 LEFT 也退出 —— **原有行为**, 别删（TASKS 子菜单一直如此） */
        s_ucMode   = OLED_MODE_NAV;
        s_ucCursor = 0u;
        s_ucTop    = 0u;
        return;                                  /* 同「返回」: 本拍不画, 下一拍画父层 */
    }
    else
    {
        /* 没按键（或按了不认的键）: 什么都不做 */
    }

    /* ---- 画 ---- */
    vOledWindowSync(pstList, bSub);  /* 每拍把窗口对齐到光标（幂等 + 自愈, 见那条函数） */
    (void)ucOledLineTestOn();    /* 画之前先把「巡线测试」标签对齐到设备真值（每拍无条件） */
    vOledMenuDraw(pstList, bSub);

    /* 第 4 行: 「让出成功!」横幅（由脚本任务打时刻, 本页只按 1s 窗口显示）。
     * ⚠️ 挂在**含「握手让出」的那张表**上 —— 原来写死第 3 页, 现在页面可增删,
     *    按语义挂才站得住。⚠️ 必须**画在菜单项之后**（先画项、后画横幅, 横幅压住第 4 行）;
     *    ⚠️ 清行**必须在窗口判断里面**: 留在外面就是每拍无条件清第 4 行 → 刚画上的
     *    菜单项每拍都被擦掉、屏幕上看它永远是空的（2026-09 踩过）。 */
    if ((bSub == 0u) && (ucOledWithin(ulGetHandshakeDoneTick(), OLED_HS_MSG_MS) != 0u)
        && (ucOledListHasAct(pstList, (uint8_t)OLED_ACT_POST_HANDSHAKE) != 0u))
    {
        vOledClearArea(OLED0, 0u, OLED_Y4, OLED_W, OLED_ROW_H);
        (void)ucOledDrawMixed(0u, OLED_Y4, "让出成功!");
    }

    /* 滚动条**最后画** —— 它要盖在上面所有步骤（行/空行清理/高亮/横幅）写出来的 x>=120 那几列。 */
    vOledScrollbarDraw(pstList, bSub);
}






/* ==================================================================
 * 任务入口
 * ================================================================== */

void vOledTask(void *argument)
{
    uint8_t ucKeys;

    (void)argument;

    for (;;)
    {
        /* 一次性取走本轮全部按键（读后清零）—— 必须在分发**之前**取, 且只取一次。
         * 50ms 刷屏周期内的多次按键不会丢（位掩码累积, 不同的键各占一位）。 */
        ucKeys = ucTakeOledKeyMask();

        /* Pi 参数轮询的"想要"集合: **每拍先全清**, 由本拍真正画出来的那些行（vOledMenuDraw
         * 里）重新标 —— 于是"屏上显示什么就查什么": 一翻走就没人标了, 后台轮询一帧内自己停。
         * ⚠️ 清在这里而不是画里: 有若干分支（进/退子菜单、确认框）本拍**不画**,
         *    放在画里就会留下上一帧的集合, 那些参数会被一直轮询下去。 */
        vRpiCfgUnwantAll();

        /* 菜单占用标志: 只要不在第 1 页就拉高 → 按键任务不再置 ucTakeMenuConfirm,
         * 于是「在菜单里按 MAIN 选任务」不会顺带把停在 STEP_STOP 的路线放走。
         * 第 1 页（默认页、比赛看的那页）为 0, 路线行为一字不变。 */
        /* ⚠️ 判据是**页的类型**不是页下标 —— 页面表是动态的, 拿「!= 0」当判据的话
         * 页序一变, 第 1 页就不再 busy=0, 比赛路线里等 MAIN 的步骤行为会被改
         * （全工程最不能碰的纪律之一）。 */
        vSetOledMenuBusy((uint8_t)((ucOledPageKind(s_ucPage) != OLED_PAGEKIND_STATUS) ? 1u : 0u));

        /* 「是/否」确认框**两种页共用**（第 3 页关机 / 第 4 页还没设二维码）→ 在分派页之前
         * 先处理掉, 不然它就得在每个页的处理函数里各抄一份。 */
        if (s_ucMode == OLED_MODE_CONFIRM)
        {
            if (ucOledConfirmKeys(ucKeys) == 0u)
            {
                vOledConfirmDraw();
            }
            /* 返回 1 = 已跳到别的页 → 别画本拍了, 下一拍画新页 */
        }
        else
        {
            /* 按**页的类型**分派（页面表由控制台生成: 0=状态 1=二维码, 其余=菜单页）。
             * 菜单页无论排在第几页, 都走同一个通用引擎。 */
            switch (ucOledPageKind(s_ucPage))
            {
            case OLED_PAGEKIND_QR:
                vOledPageQr(ucKeys);
                break;

            case OLED_PAGEKIND_MENU:
                vOledPageMenu(ucKeys);
                break;

            default:
                vOledPageStatus(ucKeys);
                break;
            }
        }

        vOledUpdate(OLED0);

        /* 必须让出 CPU：软件 I2C 整屏刷新耗时数 ms，全速空转挤占同优先级任务 */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
