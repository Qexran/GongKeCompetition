/**
  * @file       app_rpi.h
  * @brief      树莓派通信模块：帧协议收发 + 启动握手 + 任务指令 + 结果等待。
  *
  *             帧协议（双端契约, 权威实现 = Pi 端 GKS_Competition/protocol.py,
  *             契约文档 = Pi 端 docs/串口协议.md）:
  *               SOF(0xAA) | TYPE(1) | LEN(1) | DATA[LEN≤64] | CRC8(1) | EOF(0x55)
  *               CRC8 覆盖 TYPE+LEN+DATA, 初值=首字节, 与 EMM V5 同表（ucEmmV5Crc8）。
  *
  *             TYPE 码（对齐 Pi protocol.py）:
  *               0x01 CMD    MCU→Pi 任务指令, DATA = ASCII 命令码("1"/"2"/"r"...)
  *               0x02 COORD  Pi→MCU 结果回传:
  *                             · "SYNC_HS_1"(9B)  = 握手请求 → 自动回 ACK 0xCC
  *                             · DATA[0]=0x01(8B) = 巡线帧 → Beam vSetLineData
  *                             · DATA[0]=0x02(≥3B)= 人质识别帧 → Beam vSetHostageResult
  *                             · DATA[0]=0x03(≥3B)= 色球识别帧 → Beam vSetColorResult
  *                             · DATA[0]=0x04(≥3B)= 靶标识别帧 → Beam vSetTargetResult
  *                             · DATA[0]=0x05(≥3B)= 靶标流帧(~20Hz) → Beam vSetAimFrame
  *                             · DATA[0]=0x06(2B) = 拍照回执 → Beam vSetPhotoResult
  *                             · DATA[0]=0x07(10B)= 矩形地板帧 → Beam vSetRectResult
  *                             · 其他 9B ASCII    = 坐标/QR 结果 → last packet
  *               0x03 QR     预留
  *               0x10 HELLO  预留
  *               0x20 ACK    双向应答, DATA=1B(0x00失败/0x01成功/0xCC握手)
  *               0xDB DEBUG  MCU→Pi 调试字符串, Pi 端 [STM-DBG] 打印后丢弃
  *
  *             握手时序（Pi 主动发起, docs/串口协议.md §5）:
  *               Pi 上电发 COORD "SYNC_HS_1"(约 2.5s 周期重发直到成功) →
  *               本端 vRpiTask 收到即自动回 ACK 0xCC → Pi 收到完成握手。
  *               ucRpiHandshake() = 等首个握手请求 + 静默确认窗（窗口内无新
  *               握手请求 = Pi 确已收到我们的应答, 双向链路皆通才放行）。
  *
  *             ⚠️ 「本次上电**已经握手成功过**」之后再调一次 ucRpiHandshake() =
  *                「判为断开、重新握手」：状态落回 RPI_LINK_WAITING 并重走上面整套流程
  *                （于是 bt_hs_wait 重新拉高, 蓝牙机械臂编辑模式又能进）。对端只在
  *                开机阶段发 SYNC_HS_1, 所以重握手要等它重启/重上电自己发, 本端不主动请求。
  *                判据是「脚本走到过阶段2」而**不是**「链路状态是 OK」—— vRpiTask 是
  *                独立任务, 对端比我们先上电时它会先应答并置 OK, 那一次照旧直接放行。
  */

#ifndef __APP_RPI_H
#define __APP_RPI_H

#include <stdint.h>
#include "app_arm_action.h"   /* emArmActionIdTdf（排爆抓取的四个"动作"参数类型） */

/* ==================== 帧常量（对齐 Pi protocol.py, 改动须双端同步） ==================== */
#define RPI_SOF             0xAAu
#define RPI_EOF             0x55u
#define RPI_FRAME_OVERHEAD  5u                     /* SOF+TYPE+LEN+CRC+EOF */
#define RPI_DATA_MAX        64u                    /* 契约 FRAME_MAX_LEN */

/* 帧类型 */
#define RPI_TYPE_CMD        0x01u   /* 任务指令(ASCII 命令码) */
#define RPI_TYPE_COORD      0x02u   /* 结果回传(坐标/巡线/握手) */
#define RPI_TYPE_QR         0x03u   /* 预留 */
#define RPI_TYPE_HELLO      0x10u   /* 预留 */
#define RPI_TYPE_ACK        0x20u   /* 应答 */
#define RPI_TYPE_DEBUG      0xDBu   /* 调试字符串 */

/* ACK 帧 DATA 码 */
#define RPI_ACK_FAIL        0x00u
#define RPI_ACK_OK          0x01u
#define RPI_ACK_HANDSHAKE   0xCCu   /* 握手应答(Pi 端 HANDSHAKE_ACK_BYTE) */

/* 巡线帧（Pi tasks/_line.py）: DATA = [TAG][err i16 LE /10000]
 *        [steer i16 LE /1000][angle i16 LE 0.1°][valid u8], 共 8B
 *        angle = 交界线倾角(0.1°) = 车相对线的航向偏差, 0=与线平行; 被航向环使用
 *                （app_line.c 的第二闭环）, **丢线时 Pi 送 0**（与 err 同约定）。 */
#define RPI_LINE_TAG        0x01u
#define RPI_LINE_LEN        8u

/* 人质识别结果帧（Pi tasks/_hostage.py, docs/串口协议.md §6c）: DATA = [TAG=0x02]
 *        [status u8][count u8] + count×[shape u8][cx u16 LE][cy u16 LE], 共 3+5×count B。
 *        shape: 1=圆柱 2=圆锥 3=腰鼓(emHostageShapeTdf); 按 cx 升序=图像从左到右。 */
#define RPI_HOSTAGE_TAG      0x02u
#define RPI_HOSTAGE_HDR_LEN  3u    /* [TAG][status][count] */
#define RPI_HOSTAGE_TGT_LEN  5u    /* [shape u8][cx u16 LE][cy u16 LE] */
#define RPI_HOSTAGE_MAX_CNT  3u

/* 色球识别结果帧（Pi tasks/_color.py, docs/串口协议.md §6d）: DATA = [TAG=0x03]
 *        [status u8][count u8] + count×[color u8][cx u16 LE][cy u16 LE], 共 3+5×count B。
 *        color: 1=红 2=绿 3=蓝（对齐 app_motion_types.h 的 color_t, 也等于二维码第一位）;
 *        按 cx 升序 = 图像从左到右（index 0/1/2 = 左/中/右）。
 *        扩展（小球对位, 2026-09, 仅成功帧携带）: 末尾追加 [yaw i16 LE 0.01°][fit_ok u8]
 *        → LEN=21。yaw = 三球质心 fitLine 的**图像**倾角（y 向下, 左高右低为正）——
 *        ⚠️ 图像角 ≠ 车体角: 车转 θ 图像只斜约 (h/Z)·θ（实测标定该比例, 预期 ~0.3）;
 *        fit_ok=0 = 共线残差超限, 有球识别可疑（读 Beam fGetBallYawLine/ucGetBallFitOk）。
 *        ⚠️ 2026-10-01 起**只是诊断量**: ucRpiBallAlign 不再据此拒动/重扫（见
 *        app_rpi.c::ucBallScanOnce 函数头）—— 蓝牙遥测 ball_fit_ok / Pi 日志里
 *        「共线NG」那行是它唯一的用处。
 *        旧 Pi 的 LEN=18 / 失败帧 LEN=3 → 按 yaw=0 / fit_ok=1 兜底。 */
#define RPI_COLOR_TAG        0x03u
#define RPI_COLOR_HDR_LEN    3u    /* [TAG][status][count] */
#define RPI_COLOR_TGT_LEN    5u    /* [color u8][cx u16 LE][cy u16 LE] */
#define RPI_COLOR_FAIL_DIR_OFF 3u /* 失败帧方向字节偏移(2026-10-26 加): [TAG][00][00][dir], LEN=4; dir 0=无信息 1=缺球从画面左出界 2=右; 旧 Pi LEN=3/成功帧不带 → 0 */
#define RPI_COLOR_MAX_CNT    3u
#define RPI_COLOR_EXT_OFF    18u   /* 扩展字段起始偏移 = HDR + 3×TGT（仅 count=3 成功帧存在） */
#define RPI_COLOR_EXT_LEN    21u   /* 含扩展字段的 DATA 总长: 18 + [yaw i16][fit_ok u8] */

/* 靶标识别结果帧（Pi tasks/_target.py, docs/串口协议.md §6e）: DATA = [TAG=0x04]
 *        [status u8][count u8] + count×[color u8][cx u16 LE][cy u16 LE][r u16 LE],
 *        共 3+7×count B（count ≤ 3 → 最长 24B）。
 *        color: 1=红 2=绿 3=蓝（= 二维码第二位）;
 *        cx/cy = **靶心**像素（全分辨率 1920×1080）;
 *        r     = 靶盘**外圆等效半径**[px]（Pi 侧内边界拟合值已按 Ø55/Ø100 折算）,
 *                视差补偿 offset = h·r/50 用它, 也可当粗略距离标尺。
 *        按 cx 升序 = 图像从左到右（与色球帧同约定）。 */
#define RPI_TARGET_TAG       0x04u
#define RPI_TARGET_HDR_LEN   3u    /* [TAG][status][count] */
#define RPI_TARGET_TGT_LEN   7u    /* [color u8][cx u16 LE][cy u16 LE][r u16 LE] */
#define RPI_TARGET_MAX_CNT   3u

/* ⚠️ **status 是位域（2026-10-16 起）, 帧长/布局一字未改**:
 *   bit0 = 1 成功 / 0 失败超时（**老语义不变**: 解析处摘出 bit0 再进 Beam, 所以
 *          三处 `status == 1u` 的消费方一个字节都不用改）;
 *   bit7 = 1 ⇒ **本次测量来自 pi(树莓派 CSI)摄像头**（0 = 主摄）。
 *          平移射靶据此选瞄准列（tc_cx_aim_pi vs tc_cx_aim）—— 见 ucTargetShootCore。
 *   其余位保留。
 *   为什么挤在 status 里、不新增一个字节: 加字节会让**老单片机把目标数组读错位**
 *   （0x04 的目标紧跟 header 起, 静默错值）; 而 status 的老消费方只是"== 1"判等,
 *   老固件看到 0x81 会判成**失败**（响亮、不动车）—— 安全的退化方向。
 *   加新字段前先想清楚这一点。 */
#define RPI_TGT_ST_OK       0x01u  /* 成功位（旧语义） */
#define RPI_TGT_ST_CAM_PI   0x80u  /* 本次来自 pi(CSI)摄像头 */
/* 失败帧方向字节（2026-10-24 加, 平移射靶的"目标色出画面"补步用）:
 * [TAG][status=0][count=0][dir]  LEN=4 —— dir: 0=无信息 1=目标色在画面**右**侧出界
 * （内容在画面里靠右/要往右找） 2=在**左**侧出界。与色球 0x03 失败帧的 [dir] 同一套语义
 * （色球: 1=缺球从画面左出界 —— 注意两者定义**相反**, 因为语义一个是"缺的球在哪"、
 *  一个是"要找的靶往哪边走", 在 Pi 侧各自换算好, 固件只管照着动车）。
 * 旧 Pi 失败帧 LEN=3 / 成功帧不带 → 0。旧固件 + 新 Pi: LEN=4 走同一分支,
 * 第 4 字节在 status!=0 时被无视 —— 向后兼容。 */
#define RPI_TGT_FAIL_DIR_OFF 3u

/* 靶标流帧（Pi tasks/_target_stream.py, docs/串口协议.md §6f）: DATA = [TAG=0x05]
 *        [status u8][count u8] + count×[color u8][cx u16 LE][cy u16 LE][r u16 LE],
 *        共 3+7×count B（最长 24B）。
 *        **与 0x04 靶标帧逐字节同布局**, 区别只在它是 ~20Hz 的**连续流**而不是一次性
 *        结果 —— 所以本端解析代码整段与 0x04 同构, 只是落进另一个 Beam 快照
 *        （vSetAimFrame, 带帧计数而非 ready 信号: 流没有"结果到达"这个概念）。
 *        count=0 = 本帧没找到（**流还活着**, 帧计数照常推进）; 单片机据此区分
 *        「流活着但丢目标」与「流停了（帧计数停涨）」。
 *        color=0xFF = Pi 无色模式（TARGET_STREAM_USE_COLOR=0, 颜色未知）——
 *        单片机退化成"取离瞄准点最近的那颗", 见 RPI_AIM_COLOR_UNKNOWN。
 *        ⚠️ status 的位域与 0x04 **同一套**（bit7 = 本次来自 pi/CSI; 见 RPI_TGT_ST_CAM_PI）
 *        —— 流帧本就每帧都带 status（今天恒 1）, 所以这条路**连字节数都没变**。
 *        ⚠️ 闭环**每拍**都要按它选瞄准列: Pi 可能中途从 CSI 退回主摄。 */
#define RPI_AIM_TAG          0x05u
#define RPI_AIM_HDR_LEN      3u    /* [TAG][status][count] */
#define RPI_AIM_TGT_LEN      7u    /* [color u8][cx u16 LE][cy u16 LE][r u16 LE] */
#define RPI_AIM_MAX_CNT      3u
#define RPI_AIM_COLOR_UNKNOWN 0xFFu /* 无色模式哨兵（见上面那段与 app_rpi.c 的选靶规则） */

/* ---- 爆爆桶坐标流帧（TAG=0x0B）: 命令 "J" 起流 / "K" 停流期间, Pi → 单片机 ~20Hz, 2026-10-24 加 ----
 * DATA = [TAG=0x0B][status u8][cx u16 LE][cy u16 LE][w u16 LE][h u16 LE] = 10B（定长）。
 * 语义见 docs/串口协议.md §6m。链路 = OpenMV v3.4 检测黑色爆爆桶 →（UART 文本帧）
 * → Pi **纯转发**（不占主摄/CSI, 互斥理由纯是 COORD 通道一条流）→ 本端闭环平移对位。
 *   status : **位域**: bit0 = 1 本拍有桶 / 0 无桶或数据陈旧（流**还活着**, 帧计数照涨）。
 *            其余位保留（**没有** bit7 相机位 —— 数据源只有 OpenMV 一台）。
 *   cx/cy  : 选中桶 **bbox 中心**像素坐标（**320×240**, OpenMV QVGA 口径 —— ⚠️ 与
 *            0x04/0x05 的 1920×1080 **不是一套**, 别抄那边的标定量）。
 *   w/h    : 桶 bbox 宽高[px]（轴对齐外接框; 只做遥测/距离粗估, 闭环只用 cx）。
 * ⚠️ **多桶策略 = "OpenMV 挑好再报"**（用户 2026-10-24 定案）: 相机端每帧只报**面积
 *    最大**的一个桶（=最近/最清晰）, 本端不用再选 —— 与 0x04/0x05 的"count 数组+
 *    离瞄准列最近"不同构, 别照抄那边的选靶代码。
 * ⚠️ 与 0x05 同一条纪律: **无桶/陈旧也照发**（流活性靠帧计数）, 走**独立快照 +
 *    帧计数**（vSetBucketFrame, 同 vSetAimFrame）; 不置 ready、不进 last packet。
 * ⚠️ 本分支必须排在通用 `ucDataLen >= 9` **之前** —— 否则 10B 整帧会被当
 *    「9 字节 last packet」吞掉（同 0x07/0x08 那条纪律）。 */
#define RPI_BUCKET_TAG       0x0Bu
#define RPI_BUCKET_LEN       10u   /* [TAG][status][cx][cy][w][h] */
#define RPI_BUCKET_ST_OK     0x01u /* status bit0: 本拍有桶（无桶=0, 流仍活着） */

/* ---- 拍照结果帧（TAG=0x06）：命令 "9" 的**回应**，Pi → 单片机 ----
 * [TAG][ok u8]    ok: 1=OpenMV 已存盘, 0=失败（含"等超时/相机没应答"）
 *
 * 为什么需要它: 命令 "9" 本身是**只发不回**的（OpenMV 对控制字节 'P' 不应答），
 * 没有这条帧的话单片机按了「拍照」只能"哑鸣一声", 根本不知道成没成。
 * 链路: 单片机 --"9"--> Pi --'P'--> OpenMV 存盘 --"P,1"--> Pi --本帧--> 单片机。
 * ⚠️ Pi 侧有**超时兜底**（config.py::PHOTO_ACK_TIMEOUT_S）: 到点没等到 OpenMV 回应
 *    就回一条 ok=0 —— 所以本帧**一定会来**（不会出现"永远等不到"的沉默）。
 * ⚠️ 长度只有 2B, 天然落不进通用 `LEN >= 9` 分支, 但仍按规矩放在它**之前**。 */
#define RPI_PHOTO_TAG        0x06u
#define RPI_PHOTO_LEN        2u    /* [TAG][ok u8] */

/* ---- 矩形地板识别结果帧（TAG=0x07）：命令 "A" 的回应, Pi → 单片机 ----
 * DATA = [TAG=0x07][status u8][cx u16 LE][cy u16 LE][flags u8][spread u8][保留 u16]
 *        = 10B（成功/失败一样长）。语义见 docs/串口协议.md §6h。
 *   status : 1=成功(cx/cy 有效); 0=失败(板子没识别出来/超时) → 消费方立刻解锁
 *   cx/cy  : **板心**像素(1920×1080 全分辨率)。板心 = 四边形对角线交点
 *            （世界矩形中心的射影, 透视下 ≠ 四角均值）
 *   flags  : bit0=三边估(缺一条边, 误差 ~15px 量级) bit1~3=用了几条边
 *            bit4=主路径(量胶带内沿) / 0=回退路径(量胶带中线)
 *   spread : 中心跨帧散布[px](封顶 255), 只做遥测 —— **不据此判失败**
 *
 * 用途: 缺球（且缺球推断也不成立）时的**备选对位基准** —— 板子位置每次一样,
 * 板心照样能当"车该停在哪"的参考（见 ucRpiAlignChain 的 ALIGN_RECT 方案）。
 * ⚠️ 长度特意取 **10 而不是 9**: 通用分支就是 `ucDataLen >= 9`（把 9B 当坐标/QR
 *    last packet），长度撞上会让人误判 —— 虽然本分支排在那之前, 也避开这个巧合。
 * ⚠️ Pi 侧任务内部有时限 + 采帧上限, 所以**这条帧一定会来**（失败也回 status=0）,
 *    本端的超时只是兜底（同拍照帧那条纪律）。 */
#define RPI_RECT_TAG         0x07u
#define RPI_RECT_LEN         10u   /* [TAG][status][cx][cy][flags][spread][保留] */
#define RPI_RECT_FLAG_EST    0x01u /* bit0: 三边估 */
#define RPI_RECT_FLAG_INNER  0x10u /* bit4: 主路径(量胶带内沿) */

/* ---- 看路结果帧（TAG=0x08）: 命令 "G" 的回应, Pi → 单片机, 2026-09 加 ----
 * DATA = [TAG=0x08][status u8][dx i16 LE /10][ex i16 LE][ey i16 LE][flags u8]
 *        [spread u8][dy u16 LE /10] = 12B（成功/失败一样长）。语义见 docs/串口协议.md §6i。
 *   status : 1=成功(dx 有效); 0=失败(没找到黄区/超时) → 消费方立刻解锁
 *   dx     : **Δx = 黄色区左边缘的 x**（单位 **0.1px**, i16 → ±3276.7px）。Pi 量好的,
 *            本端**直接拿来用**, 不再自己算距离。
 *   ex/ey  : 左边缘那一点的全分辨率像素坐标（**只做遥测/画图**, 本端不参与运算）
 *   flags  : bit0=左边缘有效(Δx 全靠它) bit1=黄区贴画面左边→Δx 饱和不可信
 *            bit2=黄区贴画面上/下边(只提示) bit3=面积刚够门限(勉强)
 *            bit4=**Δy 有效**(见 dy)
 *   spread : Δx 跨帧散布[px](封顶 255), 只做遥测 —— **不据此判失败**
 *   dy     : **Δy = 黄色区下边缘的 y**（单位 0.1px, u16 → 0~6553.5px）, 车体**横向**
 *            那一把尺（见 ucRpiRoadAlign 的第二个参数）。⚠️ 原先是「保留 u16」恒 0。
 *
 * ⚠️ dx 用 **i16 而不是 u16**（本工程里只有这条帧的坐标是有符号的, 别照别的帧改回去）:
 *    方向对调后误差可以为负, 且黄区贴到画面左边时 x 可以到 0 附近 —— u16 会被钳成 0
 *    而且**不报错**。dy 相反: 下边缘的 y 不可能为负, 所以就是 u16。
 * ⚠️ 长度特意取 **12 而不是 9**: 通用分支就是 `ucDataLen >= 9`（把 9B 当 last packet）,
 *    长度撞上会让人误判 —— 虽然本分支排在那之前, 也避开这个巧合。
 *    **加 dy 时刻意不改长度**(用原来那个恒 0 的保留字段): 改成 14 的话, 新固件按 `>= 14`
 *    判, 旧 Pi 发来的 12B 帧会掉进那条通用分支被当 last packet 吞掉（静默失效）。
 * ⚠️ Pi 侧任务内部有时限 + 采帧上限, 所以**这条帧一定会来**（失败也回 status=0）,
 *    本端的超时只是兜底（同矩形/拍照帧那条纪律）。 */
#define RPI_ROAD_TAG         0x08u
#define RPI_ROAD_LEN         12u   /* [TAG][status][dx/10][ex][ey][flags][spread][dy/10] */
#define RPI_ROAD_FLAG_EDGE   0x01u /* bit0: 左边缘有效（Δx 全靠它） */
#define RPI_ROAD_FLAG_CLIPX  0x02u /* bit1: 黄区贴画面左边 → Δx 饱和, 不可信 */
#define RPI_ROAD_FLAG_CLIPY  0x04u /* bit2: 黄区贴画面上/下边（只提示 —— ⚠️黄区总在画面右上角,
                                   *       所以这一位其实是**上边缘**被顶边裁掉触发的, 与「下边缘 dy」
                                   *       无关, 千万别拿它当 dy 的失效门, 否则 Y 对齐永远不生效） */
#define RPI_ROAD_FLAG_SMALL  0x08u /* bit3: 面积刚够门限（勉强） */
#define RPI_ROAD_FLAG_YEDGE  0x10u /* bit4: Δy(黄区下边缘 y) 有效 —— Pi 填了 dy 才置位。
                                    * ⚠️ 旧 Pi 不置此位 → 本端判「没有这个量」, Y 侧跳过(只打日志),
                                    *    纵向照常 —— 于是两个仓库谁先烧都不炸。 */

/* ---- 看路校准结果帧（TAG=0x09）: 命令 "H" 的回应, Pi → 单片机, 2026-10 加 ----
 * DATA = [TAG=0x09][status u8][ang i16 LE ×0.01°][cols u8][rms u8 ×0.1px]
 *        [spread u8 ×0.01°][flags u8] = 8B（成功/失败一样长）。语义见 docs/串口协议.md §6j。
 *   ang    : **黄区直边的图像方向角** —— 拟合直线与**图像竖直方向(y 轴)**的夹角,
 *            竖直=0°、向图像 x 正方向(画面右)倒为正、归一化 (−90,+90]。
 *            物理: 黄区固定在场上、相机固定在车上 ⇒ 直边在图像里的方向**只随车航向变**
 *            (直线方向对平移不敏感) —— 车停在基准位标一次 ideal, 之后
 *            err = (ang − ideal)×方向 = 「陀螺漂了多少」的视觉读数。
 *   cols   : 拟合内点数(封顶 255; 只做遥测)
 *   rms    : 内点残差 RMS [0.1px](质量门; 越小越直)
 *   spread : ang 跨帧散布 [0.01°](封顶 255 = 2.55°; 只做遥测)
 *   flags  : bit0=方向角有效(ang 全靠它) bit1=直段偏短/内点少(勉强, 只提示)
 *
 * ⚠️ 本帧只携带测量值; 落地由消费方对齐到相机真值（bias = 原始航向 − err,
 *    ROADYAW_APPLY_EN=1 时发布 yaw 统一扣）—— 修正走「到点离散改偏置」,
 *    不硬置 yaw、不在行车中改。
 * ⚠️ ang 与巡线帧的 angle(交界线倾角)**不是同一个东西**, 标定值别互相抄;
 *    **未标定哨兵 = 999 不是 0**(app_config.h::ROADYAW_ANGLE_IDEAL_DEG) ——
 *    竖直的直边 ang 本来就 = 0°, 0 是合法值。
 * ⚠️ 长度特意取 **8 而不是 9**: 通用分支是 `ucDataLen >= 9`(把 9B 当 last packet),
 *    8B 天然落不进去。**旧固件没有本 TAG 分支 → 8B 帧被静默丢弃**(不炸链路,
 *    单片机等结果超时失败) —— 部署顺序**先 Pi 后固件**(同预热指令那条纪律)。
 * ⚠️ Pi 侧任务内部有时限 + 采帧上限, 所以**这条帧一定会来**(失败也回 status=0)。 */
#define RPI_ROADYAW_TAG      0x09u
#define RPI_ROADYAW_LEN      8u    /* [TAG][status][ang/100][cols][rms/10][spread/100][flags] */
#define RPI_ROADYAW_FLAG_FIT 0x01u /* bit0: 方向角有效 */
#define RPI_ROADYAW_FLAG_FEW 0x02u /* bit1: 直段偏短/内点少（勉强, 只提示） */

/* ---- Pi 端参数帧（TAG=0x0A）: 命令 "P" 的回应, Pi → 单片机, 2026-10 加 ----
 * DATA = [TAG=0x0A] + ASCII 文本（UTF-8, 见 docs/串口协议.md §6k）:
 *   "V,<NAME>,<type>,<value>[,<opts>]"   取参成功（opts 省略 = 自由数值）
 *   "OK,<NAME>,<value>"                  临时设参成功
 *   "ERR,<NAME>,<原因>"                  失败（未登记/值不合法/形状不认识）
 * 文本总长（含 TAG）≤ RPI_DATA_MAX=64, 超长时 Pi 侧截断并在末尾留一个 '!'。
 * ⚠️ 分支必须排在通用 `ucDataLen >= 9`（last packet）**之前**: 这些帧普遍 ≥9B,
 *    不拦就会被当成 QR/坐标结果帧吞掉、并误置 RpiFrameReady。
 * ⚠️ 本分支**绝不置 ready / 不动 last packet** —— 它是"旁路遥测 + 调参回执",
 *    与任何"等结果"的等待窗无关（一次在飞查询对它们是隐形的）。
 * ⚠️ "P,<名>,<值>" 只改 **Pi 主程序进程的内存**, 不写 config_override.json
 *    → 重启还原（要持久化走 GKS_Web 的「参数」页）。*/
#define RPI_CFG_TAG          0x0Au

/* Pi 参数的**类型**（照 config.py::EDITABLE_CONFIG 的 type 字段） */
#define RPI_CFG_T_F32   0u   /* float   */
#define RPI_CFG_T_I32   1u   /* int     */
#define RPI_CFG_T_STR   2u   /* str（只读显示, 不做步进） */

/* ---- 蜂鸣命令帧（TAG=0x0C）: Pi → 单片机, 2026-10-27 加 ----
 * DATA = [TAG=0x0C][pattern u8] = 2B。语义见 docs/串口协议.md §6n。
 *   pattern: BUZZER_PAT_*（app_beam.h, **值是跨仓库契约**）:
 *            1=失败音（响 0.8s → 停 0.5s → 响 0.8s）  2=短滴（响 0.2s 一声）
 *   0 与未知值 = 忽略（旧固件没有本分支时整帧静默丢弃, 双向安全 —— 与 0x09 那条同款）。
 * ⚠️ 分支排在通用 `ucDataLen >= 9` **之前**（2B 天然落不进, 但按惯例归位）;
 *    解析侧**只投请求不执行**（vSetBuzzerReq → vBuzzerTask 响）—— vRpiTask 禁阻塞,
 *    失败音整段 2.1s 只能在蜂鸣任务里响。
 * ⚠️ 这是 Pi 控制蜂鸣器的**唯一**通道: 固件自身的失败蜂鸣是 app_rpi 各失败点
 *    直接投 BUZZER_PAT_*（同一执行者, 见 app_actuator.c::vBuzzerTask 段头）。 */
#define RPI_BEEP_TAG         0x0Cu
#define RPI_BEEP_LEN         2u    /* [TAG][pattern] */

/* ---- 相机健康检查结果帧（TAG=0x0D）: 命令 "Z" 的回应, Pi → 单片机, 2026-11 加 ----
 * DATA = [TAG=0x0D][M u8][Q u8][Pi u8][MV u8] = 5B（成功/失败一样长）。语义见
 * docs/串口协议.md §6o。
 *   M  = 主摄（/dev/camera_main, USB）
 *   Q  = 扫码副摄（/dev/camera_Qr, USB）
 *   Pi = 树莓派 CSI 摄像头（camera_picam）
 *   MV = OpenMV 寻线/爆爆桶相机
 * 每项：1=检查通过（能出帧） 0=失败/超时。
 * ⚠️ 本帧只报告结果，**不携带**"未检查"语义 —— 固件侧 2 是本地初始值（还没检查），
 *    不会出现在 Pi 发来的帧里。
 * ⚠️ 分支排在通用 `ucDataLen >= 9` **之前**（5B 天然落不进，但按惯例归位）。
 * ⚠️ Pi 侧检查完**会主动释放**四路占用（USB 租约 / CSI 持有器 / OpenMV 发 'S'）——
 *    本命令只用于握手后的一次性体检，不会把相机留热。 */
#define RPI_HEALTH_TAG       0x0Du
#define RPI_HEALTH_LEN       5u    /* [TAG][M][Q][Pi][MV] */
#define RPI_HEALTH_FAIL      0u    /* 该项失败/超时 */
#define RPI_HEALTH_OK        1u    /* 该项通过（能出帧） */
#define RPI_HEALTH_UNKNOWN   2u    /* 本端初始值：还没检查过（Pi 不会发这个值） */

/* Pi 参数表的一项（元数据, **不含值** —— 值运行时用 "P?,<名>" 现取）。
 * 表本体 g_astRpiCfgParam 由 tools/gen_rpi_cfg.py 从 Pi 的 config.py 生成,
 * 落在 app_rpi.c 的生成块里（改 config.py 后重跑那个脚本, 别手改表）。 */
typedef struct
{
    const char *pcName;      /* Pi 侧参数名（EDITABLE_CONFIG 里的, 全大写） */
    const char *pcLabel;     /* 中文名（界面显示） */
    const char *pcGroup;     /* 分组中文名 */
    uint8_t     ucType;      /* RPI_CFG_T_* */
    const char *pcOpts;      /* 选项表 "0=关|1=开"; NULL = 自由数值 */
} stRpiCfgParamTdf;

extern const stRpiCfgParamTdf g_astRpiCfgParam[];
extern const uint16_t         g_usRpiCfgParamNum;

/* 表项下标 → 参数（越界返回 NULL）。OLED 调参块按名字存, 每次现查（表是常量,
 * 现查比存下标稳: 表重生成后下标会漂, 名字不会）。 */
const stRpiCfgParamTdf *cpstRpiCfgFind(const char *pcName);

/* ---- 运行时缓存 + "P" 命令收发（实现见 app_rpi.c 的「Pi 参数缓存」段）----
 * 缓存由 vRpiTask 的后台轮询填充: 只查**被标记想要**的项（vRpiCfgWant, OLED 在画某个
 * Pi 参数行时标它）, 一次只有一条在飞 —— 读方（OLED/蓝牙遥测）**从不自己发起串口请求**,
 * 所以任何任务上下文都能调, 而且不会与"等结果"的等待窗抢帧。
 * ⚠️ 查询是**按需**的（2026-10 改）: 只查"没值"的行 + ERR 重试（封顶）+ 链路重连后重查
 *    一次 —— **不再按时间自动刷新**, 调参屏挂多久都不会刷 "P?"（详见 .c 段头的取舍说明）。
 * ⚠️ 值可能是**旧的或空**: 首次显示前是空（屏上画"--"）, 之后一直是有值以来的那一份
 *    （用户改值会经 Pi 回执刷新; Pi 侧/网页改的要等重查触发才反映）。 */
void     vRpiCfgWant(uint16_t usIdx);       /* 标记"这个参数要被显示"（幂等; 无值时立刻查） */
void     vRpiCfgUnwantAll(void);            /* 全部取消（每帧重算"在屏上那几行"时先调它） */
uint8_t  ucRpiCfgCached(uint16_t usIdx, float *pfVal);  /* 1=有值并回填（F32/I32） */
const char *pcRpiCfgStrAt(uint16_t usIdx);  /* STR 型缓存值（没有则 ""） */
void     vRpiCfgSet(uint16_t usIdx, float fVal);  /* 发 "P,<名>,<值>"; 乐观写缓存, 不阻塞 */
const char *pcRpiCfgLastErr(void);          /* 最近一条 ERR 的原因（2s 内有效, 否则 ""） */

/* ==================== 对位方案（STEP_BALL_ALIGN_WAIT 的 m1/m2/m3 槽）====================
 * 一个槽 = 一种"把车对到理想停车位"的做法, 链按槽序逐个试、**第一个成功即停**。
 * ⚠️ 尾注释是控制台「地图」Tab 下拉里的标签（parse_enums 取成员尾注释）。
 * ⚠️ 成员名别与其它枚举重名（控制台按下拉选项字符串比对）。
 * ⚠️ ALIGN_NONE = 0 是**唯一自洽的空槽值**: map_codec 把空参数槽补成 "0"
 *    (`_normalize_param("")` → "0"), 所以"不试"必须是 0（同 SCRIPT_PRESET_NONE 的做法）。 */
typedef enum {
    ALIGN_NONE     = 0,   /* 不试(空槽/不配这个方案) */
    ALIGN_BALL_MID = 1,   /* 三球中球(缺球就失败; 基准与动作组录制位姿一致) */
    ALIGN_RECT     = 2,   /* 矩形板心(缺球也能用; 标定 rect_cx/cy_center) */
    ALIGN_PULLBACK = 3,   /* 缺球开环拉回(沿排倒回固定距离再重扫三球; 距离=蓝牙 ball_pullback_mm) */
} emAlignMethodTdf;

/* ==================== 二维码解码算法（STEP_QR_SCAN_* 的 algo 槽）====================
 * 同一个"扫码"动作有两种解码器，Pi 端各一条任务码：
 *   自动 —— 「全帧定位 → 按检测框紧裁 → 强放大」级联（默认, 2026-10 重做）
 *           命令 "1"/"D" 走它。现场实测（码只有 46px）: 静止 2/10→10/10、
 *           平移 0~20% 全中、旋转 0~25° 全中（旧的固定中心裁剪在这两项上是 0）。
 *           它内部最后一级仍会回调微信引擎复核, 所以"以前能扫出来的场景"不会退化。
 *   自研 —— 几何重建 + 候选集校验，治斜视/小图/强透视，但慢得多
 * 相机是另一个轴（副摄 / 主摄），两个轴组合出四条命令（见下面的任务码）。
 * ⚠️ 尾注释是控制台「地图」Tab 下拉里的标签（parse_enums 取成员尾注释）—— 只写那两个词，
 *    "慢得多、超时要给足"这类操作提示写在 app_script.h 参数行的行尾备注里。
 * ⚠️ 成员名别与其它枚举重名（控制台按下拉选项字符串比对）。
 * ⚠️ 0 必须是最安全的那一档（= 一直以来的行为）：老路线行填的槽位默认按 0 解释。
 *    自动档内部含微信兜底, 所以"老路线不写 algo"仍然落到最稳的那条。 */
typedef enum {
    QR_ALGO_AUTO   = 0,   /* 自动 */
    QR_ALGO_DIFF   = 1,   /* 自研 */
} emQrAlgoTdf;

/* ==================== 预热哪台相机（STEP_RPI_WARM_CAM 的 cam 槽）====================
 * 两块板的预热**合成一条积木**之后, 相机就成了参数而不是"挑哪个任务码"。
 * "预热"的完整语义（开好相机 + 后台持续取最新帧 + 租约空闲窗口）见 app_rpi.c::vRpiWarmCamera
 * 与 Pi 端 camera_lease.py; 这里只负责把 cam 映射成任务码 "B"/"C"。
 * ⚠️ WARM_CAM_PI（2026-10-16 加）**不占新任务码**: 它复用识别主摄那条 "C", 只是多带一个
 *    `is_use_picam` 字段（"C,&lt;uses&gt;,1"）—— 于是"预热 + 随后那次识别"永远是同一台相机。
 *    Pi 侧把 CSI 的持有器放在 camera_picam（**不在** camera_lease 里）, 所以两颗可以同时热着。
 * ⚠️ WARM_CAM_MAIN_PI（2026-10-18 加）= **一条积木预热两颗**（主摄 + pi）: 实际是背靠背发
 *    两条帧（"C,<uses>" 与 "C,<uses>,1"）, 两个持有器互不相干, 可以同时热着。
 *    用途 = 「平移射靶」定案的分工（**粗调固定主摄、细调走 pi**）: 射靶前把 pi 提前热好,
 *    细调"7"第一步就命中热流, 不必现起那 ~3s（CSI 冷启动 > PICAM_TRY_BUDGET_S=2s 的老问题）。
 *    ⚠️ 预热窗口 = uses × 15s（两颗各算各的）: 积木放得离用它的步骤太远会被空闲回收,
 *       射靶前那种"隔一段行驶+巡线"的排法给 uses=2~3。
 * ⚠️ **档位是第三个参数槽（2026-10-23 定稿）** —— 它是一个**打包计划**
 *    （每次使用 2 bit），见下面的 WARM_PROF_* 一节。
 * ⚠️ 尾注释是控制台「地图」Tab 下拉里的标签（parse_enums 取成员尾注释）—— 简洁写,
 *    操作提示写在 app_script.h 参数行的行尾备注里。
 * ⚠️ 成员名别与其它枚举重名（控制台按下拉选项字符串比对）。
 * ⚠️ 0 是最安全的那一档（= 老路线/空参数槽默认解释成的那一档）: 主摄覆盖的识别任务最多,
 *    且 map.c 里 7 处预热有 6 处是主摄。
 * ⚠️ 别在「地图」Tab 的**元数据编辑器**里改 cam 的下拉: 保存会把派生出来的枚举写成
 *    `{WARM_CAM_MAIN=主摄,...}` 字面量冻进 app_script.h, 此后下拉不再跟随本枚举。
 *    （algo / action 那几个枚举型参数同理, 是控制台那边的既有行为, 别为它改控制台。） */
typedef enum {
    WARM_CAM_MAIN = 0,   /* 主摄(人质/色球/靶标/看路/主摄扫码) */
    WARM_CAM_QR   = 1,   /* 扫码副摄(二维码) */
    WARM_CAM_PI   = 2,   /* pi摄像头(树莓派CSI, 平移射靶细调用它) */
    WARM_CAM_MAIN_PI = 3, /* 主摄+pi(两颗一起预热) */
} emWarmCamTdf;

/* ==================== 预热档位：每次使用各自一档（STEP_RPI_WARM_CAM 的 prof 槽, 2026-10-23）====================
 * 档位 = 一套 **曝光 + 白平衡**（见 Pi 端 camera.py::_CAMERA_PROFILES）。与"哪台相机"正交。
 *
 * 为什么需要它: 普通预热（prof=0）按 main 档开相机 = **自动曝光**; 而「看路」要手动曝光
 * + 固定白平衡、「人质」要手动 350。从 main 换到这两个档是**曝光模式翻转**，切换后头
 * 0.5~2s 的帧不可用（过曝染黄 / Y−B 分布没拉开）—— 「预热过了，第一次看路还是慢」就是
 * 它: 那一步在**现场**换档（实测第一次 ~1s、第二次已是 road ~0.1s）。预热时选好档位，
 * Pi 就在**预热那一刻**把曝光/白平衡设好，那一步命中同档、首帧立刻可用。
 *
 * ⚠️ **prof 槽是一个"打包计划"，不是单个档位码**: 一次预热常常要被好几个任务消费
 *    （uses=N），而它们可能要不同的档 —— 所以每**次使用**占 **2 bit**，第 i 次在 bit 2i:
 *        bit 值: 0 = 默认档（main/qr）  1 = 看路档  2 = 人质档  3 = 保留（按默认档处理）
 *        例: uses=3, [默认, 看路, 人质] = 0 | (1<<2) | (2<<4) = 36
 *    单档写法天然兼容: prof=1 就是"第 1 次看路、其余默认"（老 map.c 的 1/2 不用改）。
 *    Pi 侧: 第 1 项在预热时应用; 之后**每用完一次**（release_camera）就在任务之间的空档
 *    里由后台线程提前切到下一项 —— 不占任务时间。拆包见 handler.py::_unpack_profiles。
 * ⚠️ 控制台把这个槽渲染成「按钮 + 弹窗」（DSL 里的 `<控件:warm_profiles:uses>`），
 *    不是普通下拉: 下拉只能表达一个值, 表达不了"N 次各自一档"。
 * ⚠️ 0 = 全默认 = 老行为: 老路线/老 map.c 条目只有两个槽, 第三槽自动补 0。
 * ⚠️ 档位只对 **USB 相机**有意义: WARM_CAM_PI（CSI）忽略它（CSI 的白平衡在
 *    config.PICAM_AWB_MODE/GAINS 里）。 */
#define WARM_PROF_SHIFT          2u          /* 每次使用占几位 */
#define WARM_PROF_MASK           0x3u        /* 单次使用档位码的掩码 */
#define WARM_PROF_USES_MAX       10u         /* 计划里最多打包几次使用（与 Pi LEASE_USES_MAX 对齐） */
#define WARM_PROF_PLAN_MAX       ((1u << (WARM_PROF_SHIFT * WARM_PROF_USES_MAX)) - 1u)

/* ==================== 巡线航向环的信号源（全局调参宏 LINE_CONV_SRC）====================
 * 2026-10-30 起 src **不再是积木参数**: 六种巡线积木（巡线收敛/直走/支点·自转接管/
 * 色球·人质单动作抓取）统一读 app_config.h 的 LINE_CONV_SRC（出厂 3=融合）。
 * 3 = **融合**（2026-10-08 加回, 输出端加权 —— 与 2026-09 删除的误差端加权是两回事,
 * 为什么安全见 app_config.h 的删除记录段与 app_line.c 融合分支）。
 *
 * ⚠️ **故意用 `#define` + `uint8_t` 参数, 不用 `typedef enum`**（同 ucRpiHostageAlign 的
 *    `ucIdx` 那一套）: 档位标签在 app_config.h 的调参 DSL（`选项:0=相机 …`），
 *    这里只提供常量名, 供 ucRpiLineSrcResolve 校验/分派。
 * ⚠️ **下面的值必须与 LINE_CONV_SRC 的 `选项:` 一一对应**（0/1/2/3 都是契约）。
 * ⚠️ `CAM` 必须是 0: 老 map.c 行的第三个 int 槽（原 gyro_w）**全是 0**（= 纯相机）,
 *    排在 0 位才能让历史行语义不变。 */
#define LINE_ANG_SRC_CAM   0u   /* 相机: 交界线图像角(贴着线走) */
#define LINE_ANG_SRC_GYRO  1u   /* 陀螺: 绝对世界航向(车头咬死目标角) */
#define LINE_ANG_SRC_NONE  2u   /* 不修航向: 只修位置(原 STEP_LINE_CONVERGE_POS_WAIT) */
#define LINE_ANG_SRC_FUSE  3u   /* 融合: 两条各自走完自己的环, 输出 w 按 LINE_CAM_PCT:(100−LINE_CAM_PCT) 加权
                                 *   （陀螺会漂、相机量的是场地角度 —— 现场要"信相机多些、陀螺兜底"）。
                                 *   陀螺腿咬的目标角 = 积木的 angle/gyro_target 槽（同一份 ±180 校验/写入）;
                                 *   占比 = Beam 运行时量 line_cam_pct, 上电默认 = LINE_CAM_PCT(50),
                                 *   蓝牙「巡线」组可当场覆盖。脚本层一律传 LINE_CAM_PCT_KEEP。 */

/* ucCamPct 的哨兵（2026-10-08 加）: 传它 = **不写** line_cam_pct, 沿用 Beam 当前值。
 * 2026-10-30 起六个脚本调用点**一律传它**（积木不再有 cam_pct 槽）; 数值通道保留给
 * 直接调用/以后可能恢复的逐块覆盖。0 不是哨兵 —— 融合档里 0 = "0% 相机 = 全听陀螺"。 */
#define LINE_CAM_PCT_KEEP  0xFFu


/* ==================== 任务指令码（对齐 Pi handler.py::_COMMANDS）====================
 * vRpiSendTaskCmd 查码表转 ASCII 命令串; Pi 端加命令时在此对表追加 */
typedef enum {
    RPI_TASK_QR          = 1,   /* "1"   二维码识别(阻塞, 结果回传 QR 帧) */
    RPI_TASK_LINE_START  = 2,   /* "2"   巡线转发开始(30Hz 巡线帧) */
    RPI_TASK_LINE_STOP   = 3,   /* "3"   巡线转发停止 */
    RPI_TASK_RESTART     = 4,   /* "r"   重启树莓派主程序 */
    RPI_TASK_END         = 5,   /* "end" 停止树莓派主循环 */
    RPI_TASK_HOSTAGE     = 6,   /* "4"   人质形状识别(阻塞, 结果回传 0x02 帧) */
    RPI_TASK_COLORBALL   = 7,   /* "5"   色球顺序识别(阻塞, 结果回传 0x03 帧) */
    RPI_TASK_TARGET      = 8,   /* "6"   靶标识别+靶心定位(阻塞, 结果回传 0x04 帧) */
    RPI_TASK_SHUTDOWN    = 9,   /* "sd"  树莓派**系统关机**(不等回应; 见 vRpiShutdownPi) */
    RPI_TASK_REBOOT      = 10,  /* "rb"  树莓派**系统重启**(不等回应; 见 vRpiRebootPi) */
    RPI_TASK_AIM_START   = 11,  /* "7"   靶标流跟踪开始(独立线程, 回传 0x05 流帧 ~20Hz) */
    RPI_TASK_AIM_STOP    = 12,  /* "8"   靶标流跟踪停止 */
    RPI_TASK_PHOTO       = 13,  /* "9"   拍照(见 vRpiPhotoTake): Pi 转告 OpenMV 存一张到它盘上的 photos/; 不等回应, 结果随后由 TAG=0x06 帧异步送回 */
    RPI_TASK_RECT        = 14,  /* "A"   矩形地板识别(阻塞, 结果回传 0x07 帧): 色球区黑胶带矩形的板心, 缺球时的备选对位基准 */
    RPI_TASK_WARM_QR     = 15,  /* "B"   预热扫码相机(只发不等, 无结果帧): Pi 提前开好 /dev/camera_Qr 并持续取帧 */
    RPI_TASK_WARM_MAIN   = 16,  /* "C"   预热主摄(只发不等, 无结果帧): 同上, 给随后的识别步骤省掉开相机的时间 */
    RPI_TASK_QR_MAIN     = 17,  /* "D"   主摄二维码识别(阻塞, 结果回传 QR 帧): 与 "1" 同一套解码链, 只把相机换成主摄 */
    RPI_TASK_QR_DIFF     = 18,  /* "E"   二维码识别(自研算法, 扫码副摄): 同 "1" 同一套结果帧, 只把解码器换成自研; 慢得多, 超时要给足 */
    RPI_TASK_QR_MAIN_DIFF= 19,  /* "F"   二维码识别(自研算法, 主摄): 同上但用主摄, 斜视扫码那条路走它 */
    RPI_TASK_ROAD        = 20,  /* "G"   看路(主摄量黄色区左边缘→Δx, 阻塞, 回传 0x08 帧) */
    RPI_TASK_CANCEL      = 21,  /* "x"   取消当前任务(只发不等, 无结果帧): 置Pi侧取消事件+停巡线流/靶标流; 中止脚本/重新握手时自动发, 治迟到结果帧错位 */
    RPI_TASK_LED_TOGGLE  = 22,  /* "T"   OpenMV 补光**临时**开关(只发不等, 无结果帧): 亮↔灭, 谁亮谁灭由 Pi 定(它记着最近下发的亮度)。⚠️「灭」是临时的 —— Pi 不写 config, 所以再开/任何一次巡线起流("2")前的自动对齐都会把灯点回原亮度 */
    RPI_TASK_ROAD_YAW    = 23,  /* "H"   看路校准(主摄拟合黄区直边图像方向角→航向基准, 阻塞, 回传 0x09 帧; **只测不改**: 不动车不写航向) */
    RPI_TASK_PI_MAIN_RESTART = 24, /* "RM"  重启树莓派主程序(2026-10-18 加, OLED「重启main」): 与 "r" 同一条 restart_program, 但**单帧直达**、不走 "r" 的双发确认门 —— 菜单按键本身就是一次刻意操作 */
    RPI_TASK_PI_SVC_RESTART  = 25, /* "RS"  重启 gks 服务(2026-10-18 加, OLED「重启服务」): systemctl restart(停止状态下执行 = 直接启动); 重启期间串口断几分钟, 属预期 */
    RPI_TASK_PI_LOG_CLEAR    = 26, /* "CL"  清屏日志(2026-10-18 加, OLED「清屏Log」): 走 GKS_Web 清屏按钮同一条 logstore.request_clear 通道 */
    RPI_TASK_WARM_PROF   = 27,  /* "W"   预热+每次使用的档位计划(2026-10-23 加, 只发不等): "W,<cam>,<uses>,<plan>", 见 WARM_PROF_* */
    RPI_TASK_BUCKET_START = 28, /* "J"   爆爆桶坐标流开始(2026-10-24 加, 独立线程, 回传 0x0B 流帧 ~20Hz; "J,<ms>"=预算) */
    RPI_TASK_BUCKET_STOP  = 29, /* "K"   爆爆桶坐标流停止 */
    RPI_TASK_ROAD_REV     = 30, /* "M"   看路(反向, 2026-10-31 加): 与 "G" 同一链路/同一 0x08 结果帧, 唯一差别 = Pi 侧整帧水平镜像后检测 —— Δx 变成量黄区**右边缘**(镜像系左边缘), ROI 左右对调; 数字全是镜像系坐标, ideal 按反向机位另标。⚠️ 双方部署: 老固件根本不发 "M"; 老 Pi 收 "M" 未知指令拒收(不炸链路) */
    RPI_TASK_HEALTH       = 31, /* "Z"   四路相机健康检查(2026-11 加, 阻塞, 回传 0x0D 帧): Pi 依次确认主摄/扫码副摄/CSI/OpenMV 能出帧, 查完立刻释放占用; "Z,<ms>"=预算 */
} emRpiTaskTdf;

/* ⚠️⚠️ 上面每个成员的尾注释**必须写在同一行**（不许用跨行的块注释）:
 *   控制台 tools/webconsole/server.py::parse_enums 的正则要求注释在同一行闭合,
 *   跨行的成员会被**静默丢掉、不进任务码下拉** —— 不报错、不告警, 只是选不到。
 *   RPI_TASK_PHOTO(13) / RPI_TASK_RECT(14) 曾经因此缺席下拉（2026-09 压成单行修掉）。
 *   ⚠️ 改本块时别在注释里写 C 注释的起止符号字面量 —— C 注释不嵌套, AC5 报
 *      "#9-D: nested comment is not allowed", 每个 include 到它的文件都报一遍。 */

/* ==================== 公开 API ==================== */
void vAppRpiInit(void);                    /* 创建发送互斥（vAppInit 调用） */
void vRpiTask(void *argument);             /* UART_RPI 唯一 RX 消费者 */
void vRpiSendFrame(uint8_t ucType, const uint8_t *pucData, uint8_t ucLen);
void vRpiSendTaskCmd(emRpiTaskTdf emTask); /* 查码表发 ASCII CMD 帧。⚠️ **照发、不等链路**
                                            * （链路不通就白丢, 与改造前一样）, 但会**顶掉**
                                            * 正挂起等链路的那个任务（见 ucRpiSendTaskCmdWait）*/
void vRpiSendTaskCmdFlag(emRpiTaskTdf emTask, uint32_t ulUsePicam);
                                           /* 同上（只发不等）, 但发 "<命令码>,0,1"（带
                                            * is_use_picam 标志; ulUsePicam==0 → 与 vRpiSendTaskCmd
                                            * 逐字节相同）。目前只给射靶保持档的起流自愈用:
                                            * 重发必须与当初那条起流**同一台相机**。
                                            * ⚠️ 三字段形状是新协议, 老 Pi 会响亮拒收。 */
void vRpiSendTaskCmdMs(emRpiTaskTdf emTask, uint32_t ulTimeoutMs);
                                           /* 同上, 但发 "<命令码>,<毫秒>" —— Pi 拿这个数
                                            * 当自己的超时(留余量后), 于是阻塞类指令的
                                            * 「谁先放弃」只由本端这一个数决定。
                                            * ⚠️ 只有 Pi 端登记过能收时限的指令可以这么发
                                            * (见 docs/串口协议.md §6); 其余指令多带一个
                                            * 逗号会被判为未知指令。ulTimeoutMs=0 → 退化成
                                            * vRpiSendTaskCmd */
void vRpiWarmCamera(emWarmCamTdf emCam, uint32_t ulUses, uint32_t ulProfPlan);
                                           /* 预热一台相机(脚本积木 STEP_RPI_WARM_CAM 的原语)。
                                            * 发裸 "B"/"C"(ulUses≤1) 或 "C,2" 这种带**次数**后缀
                                            * 的形状。同样**照发、不等链路**(见 .c 里的理由)。
                                            * ⚠️ 次数后缀是新协议: 旧 Pi 会把它判成未知指令而
                                            * 拒掉 —— 影响只是那次预热静默失效(退回现开现扫,
                                            * 慢 ~2s, 不坏), 所以部署顺序是**先改 Pi、再烧固件**。*/

/* 等链路 OK 再发任务（**唯一会等链路的入口**）: 返回 1 = 帧已发出（接着 ucRpiWaitXxx
 * 等结果即可）; 0 = 被中止 / 被新任务顶掉（**没发**）。
 * 链路不是 OK 时挂住等重连, 单槽不排队（后写者胜）; 等超过 RPI_LINK_RETRY_TIMEOUT_MS 就
 * 发一对 "r" 让 Pi 重启客户端程序, 如此往复**不封顶**（只靠中止位退出）。
 * ⚠️ 等待重连的时间**不计入积木自己的超时** —— 本函数在 ucRpiWaitXxx 之**前**返回, 所以
 *    积木的 I(0) 天然只覆盖"帧真正发出去之后"那一段。
 * ⚠️ 只能被**脚本任务**调（它假定同时只有一个等待者）。循环里查了中止位。
 * ⚠️ 只给「发起任务且要等结果」的原语用。**收尾帧**（LINE_STOP/AIM_STOP）与「只发不等」
 *    的几条（OLED 拍照/关机、STEP_RPI_SEND_TASK、"C" 预热主摄）保持用 vRpiSendTaskCmd ——
 *    收尾帧最需要发出去的时刻就是链路不可靠时, 门控它会让 Pi 一直跑着 30Hz 流。 */
uint8_t ucRpiSendTaskCmdWait(emRpiTaskTdf emTask, uint32_t ulTimeoutMs);

/* 同上, 但**多带一个 is_use_picam 标志**（2026-10-16 加, 目前只有平移射靶用）:
 *   ucUsePicam != 0 → 发 "<码>,<ms>,1"（"7" 起流走 "<码>,0,1", 第二字段是占位）;
 *   ucUsePicam == 0 → 与上面的不带标志版**逐字节相同**（所以"只用主摄"那条路不受影响）。
 * ⚠️ 三字段形状是新协议: 旧 Pi 判"形状不认识"而**响亮拒收** → 本端超时、本步失败
 *    （不会静默跑错相机）, 所以部署顺序仍是**先改 Pi、再烧固件**。 */
uint8_t ucRpiSendTaskCmdWaitPi(emRpiTaskTdf emTask, uint32_t ulTimeoutMs, uint8_t ucUsePicam);

/* 「等链路发任务」的只读遥测（蓝牙 Tab「通信」组）: rpi_pend / rpi_pend_ms /
 * rpi_link_retry / rpi_pend_drop。现场判读（配合已有的 rpi_link 与 rpi_rx_bytes）:
 *   · rpi_link 非 1 且 rpi_pend 非 0 且 rpi_pend_ms 在涨  → 任务在等重连;
 *   · rpi_link_retry 在涨                               → 正在往复重启 Pi;
 *   · rpi_pend 由非 0 变 0 而路线没往下走, rpi_pend_drop+1 → 被新任务顶掉了;
 *   · rpi_rx_bytes **冻结** + rpi_link 非 1 → Pi 根本没跑（关机/线断）, 重启指令发不进去;
 *   · rpi_rx_bytes **在涨** + rpi_link 非 1 → Pi 活着但从没重新握手
 *     —— 那正是「重启 Pi 客户端」唯一真正有效的场景。 */
uint8_t  ucGetRpiPendTask(void);    /* 当前挂起的任务码（0=无）*/
uint32_t ulGetRpiPendMs(void);      /* 已经等了多久 [ms]; 没挂起时为 0 */
uint16_t usGetRpiLinkRetry(void);   /* 累计发出的 "r" 帧数（奇数=卡在半对, 偶数=整对）*/
uint16_t usGetRpiPendDrop(void);    /* 累计「挂起中被顶掉」次数 */

void vRpiSendDebug(const char *pcStr);     /* 调试串发 Pi(打印后丢弃), 联调用 */
void vRpiKickPiClient(void);               /* **立刻**发一对 "r" 重启 Pi 客户端（= ucRpiHandshake
                                            * 阶段 1 里那个「对端活着却不发握手」自愈踢, 抽出来
                                            * 给蓝牙用）: 调用点 = `$B,MODE,0` 退出编辑模式且正处
                                            * 让位窗 —— Pi 只在启动阶段发 SYNC_HS_1, 不踢它,
                                            * 链路就一直停在 WAITING（原来自愈要等 6s 重试闸,
                                            * 加上 Pi 重启 20s+, 体感就是"退出后不恢复"）。
                                            * ⚠️ 带 500ms 让出式延时（"r" 双发间隔）, 只准在
                                            * **任务上下文**调（蓝牙任务可以, vRpiTask 不行）;
                                            * Pi 没上电时两帧白发, 无害。 */
void vRpiCancelPiTask(void);               /* 取消 Pi 侧正在跑的任务/巡线流/靶标流(发 "x", 只发不等)。
                                            * 两个调用时机: ① 本端中止脚本时(OLED 重投/菜单请求, 见 vScriptPostReq)
                                            *   —— 不取消的话被中止的旧任务会在 Pi 上跑完并回帧, 结果落进
                                            *   **下一轮**同类型步骤的等待窗口被当成新结果(迟到帧错位, 全程无报错);
                                            * ② 重新握手时(ucRpiHandshake 判「断开重连」那条路) —— 旧会话的
                                            *   巡线流/靶标流可能还活着占主摄。非阻塞, 任意任务上下文可调。 */
uint8_t ucRpiHandshake(void);              /* 等握手+静默确认, 成功返回 1;
                                            * 链路已 OK 时再调 = 判为断开并重新握手。
                                            * 阶段 1 的等待是**无限**的(Pi 不上电就停在那儿,
                                            * 那正是台架调臂的工作态: bt_hs_wait=1 可以进蓝牙
                                            * 编辑模式); 只有中止请求(OLED 菜单)能让它返回 0。
                                            * ⚠️ 只能脚本任务调用: 它阻塞, 且 s_bHsDone 是
                                            * 「只有脚本任务读写」的单写者假设 */
uint8_t ucRpiHealthCheck(uint32_t ulTimeoutMs);
                                            /* 四路相机健康检查（STEP_WAIT_HANDSHAKE 的
                                            * health 参数勾选时，握手成功后立刻调用）:
                                            * 发 "Z"（等链路）→ 等 0x0D 结果帧。
                                            * 返回 1 = 四项全通过; 0 = 有失败/超时/链路不通。
                                            * ⚠️ **阻塞**（脚本任务停在健康检查里, 后面任何
                                            *    发车步骤都不会执行）; 等待期查中止位。
                                            * ⚠️ 无论成败都会把结果写进 Beam（0=失败
                                            *    1=通过 2=未检查）, 供 OLED 第 1 页第 4 行
                                            *    与 IF 条件读取。超时/链路失败按四项全 0 处理。 */
uint8_t ucRpiWaitQrResult(uint32_t ulTimeoutMs);  /* 等二维码结果帧: 1=成功(见 iGetQrCode) 0=失败/超时 */
uint8_t ucRpiWaitHostageResult(uint32_t ulTimeoutMs); /* 等人质识别结果帧: 1=成功(见 vGetHostageResult) 0=失败/超时 */
uint8_t ucRpiQrScan(uint32_t ulTimeoutMs, emQrAlgoTdf emAlgo);
                                                  /* 组合动作: 发二维码指令+等结果+失败音(阻塞)。**扫码副摄**。
                                                   * emAlgo 选解码算法(自动/自研), 与相机一起决定发哪条任务码。
                                                   * 自动 = Pi 端「定位→紧裁→强放大」级联(命中实测 60~75ms);
                                                   * 自研 = 几何重建(单次 ~4.5s), 超时给足 7500 起。
                                                   * ⚠️ 二维码是**人工预设**的（ucGetQrManual）时**直接返回 1、
                                                   * 不问 Pi**, 沿用那个值 —— 一条
                                                   * `QR preset: skip scan` 调试串 + OLED 的 `QR:xxx*` 星号（不响, 无声跳过）
                                                   * ⚠️ 选自研时 ulTimeoutMs 要给足(它慢得多), 且必须大于
                                                   * Pi 侧的 QR_DIFF_TIMEOUT —— 固件先放弃的话, 那次结果帧
                                                   * 会落进下一轮重扫的等待窗口里被当成那一次的结果 */
uint8_t ucRpiQrScanMain(uint32_t ulTimeoutMs, emQrAlgoTdf emAlgo);
                                                  /* 同上, 但让 Pi 用**主摄**扫码(RPI_TASK_QR_MAIN / _DIFF)。
                                                   * 与 ucRpiQrScan 共用全部逻辑(含 qr_manual 短路与失败音),
                                                   * 只差发哪条任务码 —— 结果帧格式四条命令相同(见 §6a),
                                                   * 所以下游读的还是同一个 iGetQrCode() */
void vSetQrCodeRandom(void);                      /* 「随机二维码」(STEP_QR_SET_RANDOM 的原语): 把二维码结果写成
                                                   * 一个**随机的合法三位数**(每位 1~3, 共 27 种) + 打上人工预设
                                                   * 标记, 一行 `QR random: nnn` 调试串（不响）。
                                                   * 用途: 多次扫码识别无果之后的兜底 —— 排爆抓取/反恐任务/标靶校准
                                                   * 都按二维码选目标, 拿不到值就整步不动, 于是后面整条链都跑不下去。
                                                   * ⚠️ 打了标记 = 后面**所有**二维码扫描步骤都会跳过(不再问 Pi),
                                                   * 只能复位/断电清除(与「调试二维码」同一性质, 有意如此:
                                                   * 否则随后一次失败的重扫会把它覆盖成 -1, 下游又成了「没数据」)。
                                                   * ⚠️ **不保证与上一次的值不同**(27 种组合里撞上同一个是正常的,
                                                   * 别拿它当「必定换一个」用)。瞬时、不碰串口协议、仅任务上下文调用 */

/* ---- 「自动下发二维码」指令族（OLED 第 2 页的功能菜单, 2026-10 加）----
 * 指令都是 ASCII CMD 帧（对齐 Pi handler.py::_dispatch_qr_auto）, **只发不等**;
 * 真相以 Pi 的 TAG=0x0A "QS,<开/关>,<值>" 回执为准（ucRpiQrAutoState 读镜像）:
 *   vRpiQrAutoQuery  → "QA"       查询（OLED 进第 2 页且值未知/过期时按需发）
 *   vRpiQrAutoSet    → "QE,<0|1>" 开/关 Pi 的「自动下发二维码」
 *   vRpiQrUpload     → "QU,<ddd>" 把当前值写进 Pi 并**启用** —— 下一次上电 Pi 握手
 *                                  成功就推 "QP<ddd>" 调试帧（= 上电即是上次的码）
 *   vRpiQrClear      → "QC"       清 Pi 端（关开关 + 清值; 本端的清在调用方做）
 * ⚠️ 只在链路 OK 时才有意义, 但本函数不重复判（调用方 OLED 已判过, 免得两处口径漂移）。 */
void    vRpiQrAutoQuery(void);
void    vRpiQrAutoSet(uint8_t ucOn);
void    vRpiQrUpload(int32_t iCode);
void    vRpiQrClear(void);
uint8_t ucRpiQrAutoState(void);   /* QS 回执的镜像: 0=未知/过期 1=开 2=关（新鲜期见 .c） */
uint8_t ucRpiHostageScan(uint32_t ulTimeoutMs);   /* 组合动作: 发人质识别指令+等结果+失败音(阻塞) */
uint8_t ucRpiHostageRescue(emArmActionIdTdf emLeft, emArmActionIdTdf emMid, emArmActionIdTdf emRight);
                                                  /* 反恐任务(STEP_HOSTAGE_RESCUE_WAIT 原语): 二维码第三位=目标形状(1圆柱/2圆锥/3腰鼓) → 在**已识别**的三只人质里找它排第几位 → 播那一位的动作预设。⚠️**播完不复位**(保持最后姿态, 与排爆抓取的差别就在这), 复位由路线自己排; 数据不全失败音不动臂
                                                   * ⚠️ 阻塞与否**跟随那个动作自己标的播放方式**(ucArmPlayByMode) */
uint8_t ucRpiWaitColorResult(uint32_t ulTimeoutMs); /* 等色球结果帧: 1=成功(见 vGetColorResult) 0=失败/超时 */
uint8_t ucRpiColorScan(uint32_t ulTimeoutMs);     /* 组合动作: 发色球识别指令+等结果+失败音(阻塞, 不动车; 对位链内被压制, 归链统一管) */
uint8_t ucRpiBallAlign(uint32_t ulTimeoutMs);     /* 小球对位一步到位: 识别→(BALL_ALIGN_YAW_EN开)转正+复测→中球偏差横/纵移; 基准=中球, 与动作组 ARM_ACT_3/4/5 的录制位姿一致。⚠️ 现在由 ucRpiAlignChain 当 ALIGN_BALL_MID 方案调用（本身一行未改） */
uint8_t ucRpiWaitRectResult(uint32_t ulTimeoutMs); /* 等矩形地板结果帧(TAG=0x07): 1=成功(见 vGetRectResult) 0=失败/超时 */
uint8_t ucRpiRectAlign(uint32_t ulTimeoutMs);      /* 矩形板心对位(ALIGN_RECT 方案): 发"A"→等板心→偏差横/纵移; 标定量复用 BALL_* 那一套 + rect_cx/cy_center; 失败不动车（不响, 归链统一管） */
uint8_t ucRpiPullbackAlign(uint32_t ulTimeoutMs);  /* 缺球开环拉回(ALIGN_PULLBACK 方案, 2026-10-26 加): 读 Pi 失败帧的方向字节(缺的球从画面哪侧出界) → 沿排倒回 ball_pullback_mm(≈一颗球间距, 开环不求准, 只求把球排弄回画面) → 重跑三球对位。方向字节 0(无信息/Pi旧版/剩球<2) → 不动车直接失败。⚠️ 与矩形方案互为替补: 矩形量的是"车该停哪"(绝对基准), 本方案量的是"往哪边走能找回球"(相对方向+固定步长), 不依赖板子检测 */
uint8_t ucRpiWaitRoadResult(uint32_t ulTimeoutMs); /* 等看路结果帧(TAG=0x08): 1=成功(见 vGetRoadResult) 0=失败/超时 */
uint8_t ucRpiRoadAlign(uint32_t ulTimeoutMs, uint32_t ulUseY, uint32_t ulRev);
                                                   /* 看路对位: 发"G"→等 **Δx=黄色区左边缘x**→(理想Δx−实测Δx)×比例×方向 车体**纵移**一步(单次开环); 标定量 ROAD_*(蓝牙「看路」组); 未标定/失败→失败音, 不动车。
                                                    * ulUseY=0(默认/老行为): 只修纵向, **逐字节等于加这个功能之前**; =1: 再用同一帧回传的 **Δy=黄区下边缘y** 修一次**车体横向**。⚠️执行顺序 = **先横移再前后**(2026-09-30 用户指定), 两段位移量都是同一帧算出的开环量、与顺序无关
                                                    * ⚠️ Y 侧拿不到可信值(帧里没 bit4 / road_dy_ideal 未标定 / 已在死区内)一律**只打日志不响、纵向照常**; 纵向自己失败时才失败音+整步不动车。
                                                    * ⚠️ 遥测 road_dy **不管 ulUseY 都会发布**（帧里有 bit4 时）—— 标定横向时正好该让 use_y=0, 那样车一步都不会横移
                                                    * ulRev=0(默认/老行为): 发 "G" 正向看路; =1: 发 **"M" 反向看路**(RPI_TASK_ROAD_REV, 2026-10-31 加) —— Pi 侧整帧镜像后量黄区**右边缘**, "越近越小"的尺子语义不变; 车倒着放/换边接近 ⇒ **纵向补正与饱和救援乘 −1**(2026-11-01 定稿; 横移**不**乘——竖直轴不镜像, 映射与朝向无关), 大小/死区/钳位/符号标定值与正向共用; 两把尺的理想值都按方向分两把(反向 = **road_dx_ideal_rev/road_dy_ideal_rev**, 0=未标定哨兵同语义: 纵向拒动、横向跳过) */
void    vRoadSetIdeal(float fDxIdeal, float fDyIdeal);
                                                   /* 看路理想值(STEP_ROAD_SET_IDEAL 的原语): 把两个理想值写进 Beam 运行时量 —— 与蓝牙「看路」组是**同一对量**(谁后写谁生效), 于是每条路线预设能带自己的一套。
                                                    * ⚠️ **0(或负) = 这一项不动**: 0 是"未标定"的哨兵值, 直接写进去会把那一轴关掉(横向还会跳过并打日志), 所以这里刻意当"不改"。瞬时、不碰串口/底盘, 仅任务上下文调用 */
uint8_t ucRpiWaitRoadYawResult(uint32_t ulTimeoutMs); /* 等看路校准结果帧(TAG=0x09): 1=成功(见 vGetRoadYawResult) 0=失败/超时 */
uint8_t ucRpiRoadYaw(uint32_t ulTimeoutMs);
                                                   /* 看路校准(STEP_ROAD_YAW_WAIT 原语): 发"H"→等 **ang=黄区直边图像方向角**(0.01°) → **对齐到相机真值**: bias = wrap180(原始航向 − err), 使发布 yaw ≡ err —— **每次校准后全场航向零点 = "放直"方向**(上电摆放误差/累计漂移一并消掉, 首次也不豁免); 首次定标不限幅, 之后每拍限幅 ±ROADYAW_BIAS_MAX_DEG(帧跳变超限=看错了, 拒绝, 校准流程不响); ROADYAW_APPLY_EN=1 时 app_sensors 发布 yaw 统一扣 —— PID/巡线陀螺腿/OLED 全生效。⚠️世界坐标系不受影响（world_heading 记录换算回原始值）; 转弯 absolute 档/巡线 gyro_target 在校准后坐标系里教/排。
                                                    * err = (ang − roadyaw_ideal)×roadyaw_dir(mod 180 折回); ideal 默认 **+90°**、dir 默认 **−1**(均台架实测: 放直读 +90, 陀螺正漂时读数减小 90→88), 未标定哨兵 |值|>90 → 照测照报; 失败/钳位拒绝（均不响, 校准流程静音）。仅任务上下文调用 */
/* 色球区对位多方案链（STEP_BALL_ALIGN_WAIT 的原语）:
 * 按 m1→m2→m3 顺序逐个试、**第一个成功即停**; ALIGN_NONE 跳过、同一方案只试一次;
 * ⚠️ 每个方案各拿满 ulTimeoutMs（最坏阻塞 = 方案数 × 超时 + 平移时长）;
 * 进「拉回（赌）」或全失败 → 失败音 + 调试串, 不静默空转（链内其余失败不响）。 */
uint8_t ucRpiAlignChain(uint32_t ulTimeoutMs, emAlignMethodTdf emM1,
                        emAlignMethodTdf emM2, emAlignMethodTdf emM3);
uint8_t ucRpiHostageAlign(uint32_t ulTimeoutMs, uint8_t ucIdx); /* 人质对位一步到位(STEP_HOSTAGE_ALIGN_WAIT 原语): 发"4"识别(三只齐才算数)→取第 ucIdx 只(0左/1中/2右)作基准→它的 cx/cy 偏差横/纵移到位。⚠️不做转正(0x02 帧无 yaw 字段, 且三种形状质心高度不同, 三点连线角不可用); 标定量见 app_config.h 的 HOSTAGE_*; 识别不全/超时→失败音, 参数越界不响; 都放弃不动车 */
uint8_t ucRpiLineConverge(uint32_t ulConvergeMs, uint8_t ucSrc, float fGyroTarget,
                          uint8_t ucEarlyRet, uint8_t ucCamPct);
                                                     /* 组合动作: 发巡线开始+闭环横移收敛+发巡线停止(阻塞; Pi 未起流则不动车返回 0; 成败都不响, 留痕)。
                                                      * ucSrc = **航向来源**（四选一; 常量见上面的 LINE_ANG_SRC_*; 脚本层始终传 LINE_CONV_SRC）:
                                                      *   CAM/相机 = 图像角; GYRO/陀螺 = 绝对世界航向; NONE = 不修航向(只修位置);
                                                      *   FUSE/融合 = 两条各自走完自己的环, 输出 w 按相机占比加权（ucCamPct 参与, 见下）。
                                                      *   ⚠️ 源不可用时那一拍**就是没有它的修正**（不自动换源/不补权重）—— 丢线时如此, IMU 停摆时也如此。
                                                      * ⚠️ 参数类型是 uint8_t 而不是枚举; 2026-10-30 起来自全局宏, 不再有积木槽位。
                                                      * fGyroTarget = 世界航向[°](零点=开机朝向), **在 ucSrc=GYRO/FUSE 时有用**
 *   （融合档 = 陀螺腿咬的目标角; 越界与陀螺档同一份校验: 拒绝 + 本步不修航向）。
                                                      * ⚠️ 它只在这里写一次当这一步的起点, 之后蓝牙可当场覆盖; 下一块巡线收敛又写回自己的 gyro_target。
                                                      * 越界(±180 之外)或 ucSrc 认不出来 → 本步当成 **NONE(不修航向)**（不响, 留痕）,
                                                      * 不把脏值写进 Beam —— 刻意**不**退回相机: "笔误就偷偷换个源"正是这次要消灭的行为
                                                      * ucEarlyRet = 积木的「启动即返回」{1=是,0=否}: 1 = 按 app_config.h「巡线收敛增强」区的判据
                                                      *   **达标即提前返回**(0 < 实际 < converge_ms), 收敛时长内没判到再给 LINE_CONV_OVERTIME_MS
                                                      *   的**超时收敛**; 0 = 跑满 converge_ms(老行为逐字节不变)。
                                                      * ucCamPct = 融合档相机占比[%]（2026-10-08 加, **只在 ucSrc=FUSE 时有用**）:
                                                      *   0~100 = 钳位后写进 line_cam_pct 当这一步的起点;
                                                      *   LINE_CAM_PCT_KEEP(0xFF) = 不写, 沿用 Beam 当前值（= LINE_CAM_PCT 上电默认 / 蓝牙覆盖）。
                                                      *   ⚠️ 2026-10-30 起脚本层**一律传 KEEP**（积木已无 cam_pct 槽）。 */
uint8_t ucRpiPivotTurnLine(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                           uint32_t ulFwdMs, uint32_t ulSettleMs, uint32_t ulExtraLineMs,
                           uint32_t ulTailMs, uint8_t ucSrc, float fGyroTarget,
                           uint8_t ucSettleEarly, uint8_t ucCamPct);
                                                     /* 组合动作(STEP_SPIN_LINE_WAIT 的原语, 2026-10-30 加): **原地转弯 + 巡线接管** ——
                                                      * 与 ucRpiPivotTurnLine **同构同参**（F2I7, 逐位同形）, 只有两处不同:
                                                      * 转弯本体换 vPidSpinTurn/Until（四轮对称自转, 车心不动; 自转角速度约支点
                                                      * 的 2 倍, 交接瞬间残留也约 2 倍）; 调试串前缀 "SPIN"（支点版 "PVL"）。
                                                      * 交接后的全部段（盲走/补等/巡线两段/尾段/世界航向收尾）与支点版**共用
                                                      * 同一份代码**（s_ucTurnLineTail）—— 参数语义、链路不通降级、"settle 与 line
                                                      * 都为 0 仍跑一拍"等纪律全部照支点版, 完整说明见 app_rpi.c 两个函数头。 */
uint8_t ucRpiSpinTurnLine(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                          uint32_t ulFwdMs, uint32_t ulSettleMs, uint32_t ulExtraLineMs,
                          uint32_t ulTailMs, uint8_t ucSrc, float fGyroTarget,
                          uint8_t ucSettleEarly, uint8_t ucCamPct);
                                                     /* 组合动作(STEP_PIVOT_LINE_WAIT 的原语): **支点转弯 + 巡线接管**一步到位 ——
                                                      * 发 LINE_START 后**边转边等起流**(两个判据都在转弯闭环里逐拍查: 误差进
                                                      * PIVOT_LINE_HANDOFF_DEG **且** 帧已起流) → [可选: 先按**体坐标系**盲走
                                                      * ulFwdMs 再停] → 巡线接管, **分两段连调**: 先 ulSettleMs
                                                      * **只对准**(横移 vx + 航向 w, 不前进), 再 ulExtraLineMs **沿线走**
                                                      * (叠加前进分量, 边修边走) → [可选: 发 LINE_STOP 之后**尾段盲走**
                                                      * ulTailMs]。settle 与 line 都为 0 时仍跑一拍(交接不停车)。
                                                      * ⚠️ **重叠是这一步唯一的意义**: 写成"先等满起流再转"就一点没省。
                                                      * ⚠️ 直行段走 vChassisMove 的 v_y(**体坐标系**), **不走世界系变换** ——
                                                      *    不依赖航向累加器; 时长=ulFwdMs(0=不直行), 速度=PIVOT_LINE_FWD_SPEED。
                                                      *    它解决的是"转弯刚结束时相机看的是转角/弯道, 要先直行一段才看得见线"。
                                                      * ⚠️ ulTailMs 是**线到头之后**再往前顶一段(到点/进区), 与 ulFwdMs 同一种动作、
                                                      *    同一个速度宏、同样走体坐标系; 0=不盲走。它排在 **LINE_STOP 之后**
                                                      *    (那段不看线, 先把 Pi 的流停掉)。
                                                      * ⚠️ **巡线段听谁 = ucSrc**（脚本层传全局 LINE_CONV_SRC; 档位同 ucRpiLineConverge）
                                                      *    —— 两段巡线共用它。取陀螺时用 fGyroTarget（±180 越界 → 拒绝 + 本步不修航向）,
                                                      *    校验代码与巡线收敛**同一份**（ucRpiLineSrcResolve）。
                                                      *    ⚠️ 别再把这里写死成相机: "交接后咬住一个绝对航向"是合法的现场需求;
                                                      * ⚠️ 交接**不停车**(残留转速由巡线第一拍覆盖);
                                                      * ⚠️ 链路不通 → 不做任何巡线动作, 直接完整转弯, 返回 0（不响, 留痕; 刻意不用阻塞版
                                                      *    发包: 转弯是纯本地动作, 等重连会让车停着不转）;
                                                      * ⚠️ 需转弯终点能看到线。ulExtraLineMs=0 就只做交接收尾窗。
                                                      * ucSettleEarly = settle 段的「启动即返回」{1=是,0=否}（2026-10 加, 现位于
                                                      *    I(6); 老行缺槽补 0 = 跑满 = 老行为）: 1 = **只** settle 段按 app_config.h
 *    「巡线收敛增强」判据对准即提前进下一段（没判到最多再给 LINE_CONV_OVERTIME_MS
 *    超时收敛）; line 段时长=行程恒不吃, fwd/tail 盲走无判据。
 * ucCamPct = 融合档相机占比[%]（2026-10-08 加; **只在 ucSrc=FUSE 参与**, settle/line 两段共用）:
 *    2026-10-30 起脚本层**一律传 LINE_CAM_PCT_KEEP**（不写 line_cam_pct, 沿用 Beam 当前值 =
 *    LINE_CAM_PCT 上电默认 / 蓝牙覆盖）。仅任务上下文调用 */
uint8_t ucRpiLineStraightWalk(float fDistMm, int iAccel, int iSpeed,
                              uint8_t ucSrc, float fGyroTarget, uint8_t ucUseRes,
                              uint32_t ulSettleMs, uint8_t ucCamPct);
                                                   /* 巡线直走(STEP_LINE_STRAIGHT_WAIT 的原语, 2026-10 加): 沿线**边前进边修**走固定
                                                    * 距离 —— mm 按位置模式**同一条梯形估时公式**反推闭环时长(fChassisMoveTimeEst,
                                                    * 体坐标系 AXIS_Y)再乘**时间系数 k**(Beam line_time_scale, 蓝牙「巡线」组当场改:
                                                    * 估时公式搬到速度模式斜坡带系统偏差、实测偏长, 走过了调小没走到调大),
                                                    * 前进速度 iSpeed / 斜坡加速度 iAccel 都由积木参数给(accel 钳 1~255)。
                                                    * ucUseRes=1 先吃「路径残留偏移」(读出即清零): 世界系残留按指令航向折车体,
                                                    * **fwd 分量并入 fDistMm**(计划−残留), side 分量丢弃并在 LW2 对账串报出
                                                    * (横向留给巡线环, 别重复补偿)。
                                                    * 起流/等流/收流与「巡线收敛」同一条龙(ucRpiLineRun); ucSrc 来自全局
                                                    * LINE_CONV_SRC, 脚本层传 LINE_CAM_PCT_KEEP（融合占比用 LINE_CAM_PCT /
                                                    * 蓝牙 line_cam_pct）; fGyroTarget 在 src=陀螺/融合 时参与, 越界拒绝。
                                                    * ulSettleMs = **起步对准段**[ms]（2026-10-05 加, 现位于 I(3), 老行
                                                    *    缺槽补 0 = 不对准 = 老行为）: 起流门过后先原地收敛这么久(iFwdRpm=0, 用
                                                    *    「原地对准」增益组)再沿线走 —— 照搬支点转弯 ④ 的 settle→line 结构。
                                                    *    直走没有对准段时, 车从摆歪状态直接全速前进, 修正追不上发散（实测
                                                    *    「直走不纠偏」的根因; 支点接管好看正是因为转弯+settle 已把车摆正）。
                                                    * ⚠️ 实际时长取整到 20ms 拍, 落点偏差全折进 k 现场标; dist/speed/accel 任一
                                                    *    ≤0(没配齐) → 拒绝 + 不动车（不响, 留痕; 且**不**消费残留）;
                                                    *    走完按**源**写世界航向: 陀螺档=指令目标(90° 网格), 相机/融合=实测, 不修=不写 */
uint8_t ucRpiLineProbe(uint32_t ulProbeMs);       /* 主动探测「现在还看得见线吗」: 起流→采样→收流, **不动车**; 1=窗口内见过有效帧; IF 条件的原语 */
void    vRpiShutdownPi(void);                     /* 让树莓派**系统关机**（OLED 菜单用; 只发指令不等回应） */
void    vRpiRebootPi(void);                       /* 让树莓派**系统重启**（同上） */
void    vRpiPhotoTake(void);                      /* 拍照（OLED 菜单用）: 让 **OpenMV** 存一张到它自己盘上的
                                                   * photos/。⚠️ 本端只把 "9" 发给 Pi（去的方向是三跳）,
                                                   * 由 Pi 的 handler.py 调 openmv_serial 写 'P' 给相机。
                                                   * ✅ **结果会回来**（第四跳）: OpenMV 存盘后回写 "P,<ok>",
                                                   * Pi 转成 COORD TAG=0x06 帧 → Beam 的 photo_ok +
                                                   * photo_done_tick → OLED 闪一句「拍照 成功!/失败!」1s。
                                                   * ⚠️ 所以本函数**不等回应**（非阻塞, 任务上下文可调）,
                                                   * 结果看 ucGetPhotoResult()/ulGetPhotoDoneTick()。
                                                   * ⚠️ Pi 侧有超时兜底（config.PHOTO_ACK_TIMEOUT_S）,
                                                   * 所以那条结果帧**一定会来**（最坏是"超时失败"）。
                                                   * ⚠️ 判成功看那条帧, **别**看"盘上有没有": 按 OpenMV
                                                   * 官方行为, 照片要**重启相机**之后 PC 上才看得见 */
void    vRpiOpenmvLed(uint32_t ulLevel);          /* OpenMV 补光亮度（OLED 调参/积木 STEP_OPENMV_LED 用）:
                                                   * 发 "L,<0~100>" 给 Pi → openmv_serial 以 "L<n>\n"
                                                   * 转发相机（v3.2 运行时调参命令, 当场生效）。
                                                   * ⚠️ **不用** s_apcTaskCmd 那套（没登记 "L"）: 那套的
                                                   *    数值版对 0 会退化成裸命令码, 而 0=灯灭是合法值 ——
                                                   *    这里专用发送器, 任何值都带后缀。
                                                   * ⚠️ 只发不等（无结果帧）; OpenMV 回的 "#OK L=n" 遥测行
                                                   *    由 Pi 的 openmv_serial 丢弃（不计坏行）。
                                                   * ⚠️ Pi 会把值持久化并在每次巡线起流("2")前重发 ——
                                                   *    OpenMV 断电重启回编译默认也不丢。 */
void    vRpiOpenmvLedRestore(void);               /* OpenMV 补光**恢复成 Pi 配置里记住的亮度**:
                                                   * 发**裸** "L"（Pi 收到后重发 config.OPENMV_LED_BRIGHTNESS）。
                                                   * 与 vRpiOpenmvLed(0)（临时灭）配对用 —— **两者都不写
                                                   * Pi 的 config**, 所以"灭"永远是临时的。
                                                   * ⚠️ 为什么本端不自己记一个亮度: 「点亮时用多少」的唯一
                                                   *    真相在 Pi（OLED/积木/Pi Web 都在改它, 本端没有影子
                                                   *    副本）—— 发裸 "L" 幂等、不怕脱节。
                                                   * 目前唯一调用点 = ucRpiRoadAlign 的补光开关
                                                   * （app_config.h::ROAD_LED_OFF_EN）。只发不等。 */
uint8_t ucRpiWaitTargetResult(uint32_t ulTimeoutMs); /* 等靶标结果帧: 1=成功(见 vGetTargetResult) 0=失败/超时 */
uint8_t ucRpiTargetScan(uint32_t ulTimeoutMs);       /* 组合动作: 发靶标识别指令+等结果+失败音(阻塞, 不动车/不动臂) */
uint8_t ucRpiTargetAlign(uint32_t ulTimeoutMs, uint8_t ucIdx); /* 标靶对位一步到位(STEP_TARGET_ALIGN_WAIT 原语): 发"6"识别(**三颗齐才算数**, 否则"第二个"不是中间那颗)→取第 ucIdx 颗(0左/1中/2右)作基准→它的**横向**偏差横移到位。⚠️只横移(前后距离由路线定); ⚠️不做转正(0x04 帧无连线角字段); 标定量见 app_config.h 的 TARGET_ALIGN_(独立一套, 不共用 TARGET_ / HOSTAGE_ / BALL_ 那三套); 识别不全/超时→失败音, 参数越界不响; 都放弃不动车 */
uint8_t vTargetCalibAim(uint32_t ulTimeoutMs);       /* 组合动作(STEP_TARGET_CALIB_WAIT 的原语): 识别→按二维码第二位选靶→按比例尺微调舵机6/3。ulTimeoutMs = 本端等结果帧的毫秒数（0 = 用蓝牙运行量 tg_calib_timeout_ms）—— 积木的 I(0) 从这里进来, 别再让它空转 */
uint8_t ucRpiTargetServo(uint32_t ulTimeoutMs);      /* 标靶流控闭环(STEP_TARGET_SERVO_WAIT 原语): 发"7"起流 → 按二维码第二位选靶 → 每拍按像素偏差微调舵机6/3(**共用 tg_* 标定**) → 连续 N 拍进死区判收敛 / 超时 / 累计行程超限 / **中止位** → 无论成败都发"8"停流。⚠️**新方案**, 与开环迭代的 vTargetCalibAim 并存; 闭环节拍类参数见 app_config.h 的 TARGET_SERVO_(ts_*)。**底盘参与闭环**(ts_chassis_en, 出厂默认关): |dx| 落进门限后舵机6 让位、改由底盘位置模式小步**前后**进给闭环(⚠️相机装了 90°, 图像横向↔车体纵移; 见 app_rpi.c 该函数头)。仅任务上下文调用 */
uint8_t ucRpiTargetShootHold(void);                  /* 平移射靶**保持**(STEP_TARGET_SHOOT_HOLD_WAIT 原语, 2026-10-09 加, OLED 射靶测试页专用): 与 ucRpiTargetShootAlign **同一个核**(ucTargetShootCore 的 ucHold=1), 但**永不因超时/收敛/noprog/行程闸退出** —— 只有中止位(vScriptAbort)能停。粗调+等停稳照跑一遍(有界, 内部常量), 进保持环后: 每拍**重读** tc_cx_aim/tc_cy_aim(按 ta_xy_swap 选)与 tc_conv_px, 操作员在 OLED 上调一格 → 偏差出死区 → 车跟着走到新目标; **瞄准常数一变就重置行程闸/noprog/收敛计数**(每次微调一套新预算)并打 SHOT hold: 行。撞闸/noprog **不退出**: 发 0 停车 + 冻结 + 一条日志, 等下次微调自恢复。⚠️ 起流自愈: 帧停涨超 2×ts_stale_ms 限速重发 "7"(Pi 侧 TARGET_STREAM_MAX_S=12s 会自己退流, 而本环不判超时 → 没有它就是冻住永不自愈)。⚠️ keep_res **恒 0**(保持测试绝不记残留)。遥测发布 shoot_err/shoot_state/shoot_settle_tick。仅任务上下文调用 */
uint8_t ucRpiTargetShootAlign(uint32_t ulTimeoutMs, uint8_t ucKeepRes, uint8_t ucRev); /* 平移射靶(STEP_TARGET_SHOOT_WAIT 原语, 激光固定在底盘的第三套打靶方案, 2026-10-07 加): 粗调(发"6"识别, **目标色在即可**、不要求三颗齐 —— 按色找靶不依赖位置序号, 某色被 Pi 散布作废时不影响目标色)→沿排偏差**按格换算**(偏差÷tc_gap_px×tc_gap_mm —— **"走多大"总旋钮=tc_gap_mm**, 改 160 只影响粗调)→一次纵移, 钳 tc_max_mm; 精调(发"7"起流→0x05 流帧小步纵移闭环, **微调模式 tc_vel_mode**: 0=位置模式每拍一步/1=速度模式按偏差比例连续修(平滑; 指令持久——死区/所有出口显式发0停车; 1rpm量化≈5.08mm/s, 不足1rpm的零速拍不数noprog)→连续 N 拍进 tc_conv_px 判收敛) → 无论成败发"8"停流 → ucKeepRes=1 把累计纵移(≈±150mm 标靶中心距)折世界系记进「路径残留偏移」, 后面的世界系平移勾"叠加残留"补掉。⚠️**全程不碰机械臂**(不进 c_bIsArmStep, 相机位姿由路线排定); ⚠️轴是**车体纵移 v_y / vChassisMoveCameraForward**(相机装 90°、靶排沿车体纵向排开, 沿排对齐 = cx 偏差驱动纵移, swap=1 本车 —— 同「标靶流控」底盘接管那根轴; ❌不是横移, 首版就错在这); ⚠️瞄准常数 tc_cx_aim(本车)与 tc_gap_px 0=未标定 → 拒绝动车（不响, 留痕）; 方向 tc_sign 两段共用(ucRev=1 时**本步内**取反 —— 靶排换到路另一边、相机朝另一边看的镜像执行, 2026-10-30 加; 蓝牙标定值不动, 粗调/补步/精调/残留全部跟着翻, 保持档恒 0; 且 rev=1 **全程只用主摄** —— CSI 固定在底盘不随臂转向另一边的靶排, 细调不再优先 pi、粗调失败不再拿 CSI 重试, tc_use_picam 不动), 节拍类见 app_config.h 的 TARGET_SHOOT_(tc_*), 帧新鲜度复用 ts_stale_ms。仅任务上下文调用 */
uint8_t ucRpiBarrelAlign(uint32_t ulTimeoutMs, uint8_t ucKeepRes); /* 爆爆桶对位(STEP_BARREL_ALIGN_WAIT 原语, 2026-10-24 加): 发"J"起流(0x0B 流帧 ~20Hz, OpenMV 检测黑桶 → Pi 纯转发) → 等停稳 → 每拍按 cx 偏差小步**纵移**闭环（**轴映射与平移射靶同款**: 相机装 90°、图像 cx ↔ 车体纵移; ❌不是横移 —— 首版别再错）→ 收敛/超时/行程超限/noprog/中止 → 无论成败发"K"停流。⚠️ bc_cx_aim=0(未标定) → **只测不改**模式: 起流采 ~3s 报选中桶 cx 的中位（"BKT meas: cx=..(n=..)"）然后停流返回 0 —— 标定自举用（车摆到理想对车位姿跑一次, 读日志填 bc_cx_aim）; 闭环控制律/双模式/各闸门照抄 ucTargetShootCore 精调环, 标定量见 app_config.h 的 BUCKET_ALIGN_(bc_*)、蓝牙「爆爆桶」组。keep_res=1 把累计纵移折世界系记「路径残留偏移」(推开爆爆桶那一步的世界系平移勾"叠加残留"吃掉)。帧新鲜度复用 ts_stale_ms。仅任务上下文调用 */
uint8_t ucRpiBombGrab(emArmActionIdTdf emLeft, emArmActionIdTdf emMid,
                      emArmActionIdTdf emRight, emArmActionIdTdf emReset);
                                                     /* 组合动作(STEP_BOMB_GRAB_WAIT 的原语): 排爆, 按二维码首位选球→播对应动作预设→复位
                                                      * ⚠️ 参数类型必须是 emArmActionIdTdf: 控制台靠 case 里的 (emXxxTdf)I(n) 强制转换
                                                      *    判定参数是枚举, 写成 (uint8_t) 就没下拉、只能手敲数字
                                                      * ⚠️ 抓取与复位各自的阻塞方式**跟随那个动作自己标的播放方式**(ucArmPlayByMode) */

/* ==================== 单动作抓取（2026-10 新方案, 与上面两块并存） ====================
 * 思路: 只录**一个**抓取动作（= 抓"中间"那颗/那只的录制位姿）。到区后照老逻辑识别,
 * 拿到目标位置(0左/1中/2右)与**中间目标**的偏差 dx/dy → 平移 = dx/dy 叠加
 * (位置-1)×one_lat_mm（蓝牙「单动作抓取」组可调）—— 目标被平移到"中间"正前方,
 * 单动作直接抓。取代"左/中/右各录一个动作"对停车位的苛刻要求。
 *   色球版: 抓完**反向回中**再放置（放爆破桶）, 结束停在中间, 不影响后续路线;
 *           emPlace=0（控制台「无」）时不投放置动作, 但**仍回退**间距分量到中间对齐位
 *           （车停位与残留清零跟老行为一致）;
 *   人质版: **默认（emPlace=0）不回中**, 平移总位移折进世界系记为「路径残留偏移」
 *           (path_res_x/y), 由后续世界系平移积木的"叠加上次残留偏移"档一次性吃掉
 *           （抓左多走、抓右少走）; **emPlace≠0 时**抓完只回退间距分量到中间对齐位
 *           （=爆爆桶位）再放, 播完清残留（净位移=对位修正）。
 *   两版都带「反向执行」尾槽 ucRev（2026-10-30 加: 场地道具换到路的另一边时勾,
 *   本步方向整体镜像 —— 残留记账随实际位移自动跟着对称, 详见两函数头说明）。
 * ⚠️ 参数类型同样必须是 emArmActionIdTdf（下拉）。仅任务上下文调用。 */
uint8_t ucRpiBallGrabOne(uint32_t ulTimeoutMs, uint8_t ucMode,
                         emArmActionIdTdf emGrab, emArmActionIdTdf emPlace,
                         uint32_t ulGrabLeadMs, uint32_t ulPlaceDelayMs,
                         float fGyroTarget, uint8_t ucSrc, uint8_t ucCamPct,
                         uint8_t ucRev, uint8_t ucSingleStep);
                                                     /* STEP_BALL_GRAB_ONE_WAIT 的原语: 色球单动作抓取(识别→[mode纠正]→
                                                      * ⚠️ 识别失败时(2026-10-26 加): Pi 失败帧带方向(剩两球/缺端球出画面)
                                                      * → 开环拉回一步(ucBallPullbackOnce, 与 ALIGN_PULLBACK 同款)再重扫
                                                      * 一次, 无方向/重扫仍失败才放弃 —— 主任务路线本步常是唯一识别入口。
                                                      * ⚠️ 识别段(2026-10-30 加)包在「OpenMV 补光临时压灭/恢复」里
                                                      * (ucBallGrabOneLocate 壳; BALL_LED_OFF_EN/DELAY_MS 编译期):
                                                      * 场地换边后 LED 热点进了主摄视野、冲淡色球 HSV 阈值 —— 灭灯
                                                      * 识别、识别完(或放弃)立刻恢复, mode=2 的收敛照常用灯(识别期间
                                                      * OpenMV 流是停的, 灭灯无感; 模式同 ucRpiRoadAlign 的补光壳)。
                                                      * 平移**与抓取并行**(底盘先动, 估时倒计时掉到 ulGrabLeadMs 再垫 pad
                                                      * 以下才异步发抓取动作 —— 锚定**到位时刻**而非起点, 2026-10-20 由
                                                      * "起点后延时"改; 提前量 = 动作伸爪前段时长, 动作属性与距离无关,
                                                      * 0=到位后才发)→等臂空→**仅回退间距分量与放置并行**。
                                                      * ⚠️ 放置 2026-10-23 改回**起点后延时** ulPlaceDelayMs: 从回退起点
                                                      * 起算, 0=回退同时放; 回退通常比放置动作短, 用"剩提前量"首拍就命中,
                                                      * 等于没有提前量。中间球一步不动, 对齐dx/dy保留, 车停在中间对齐位
                                                      * =爆破桶位。**放置动作播完立即返回, 不再做放置后收敛**; 需要收敛的
                                                      * 路线自己在块后排「巡线收敛」。
                                                      * **不等臂开始、只等它结束**; 短行程不白等、长行程不会车未到爪先伸。
                                                      * 抓取短行程默认首拍即发(=与底盘同时开始) —— 要强制"车先走/停稳一段
                                                      * 再伸爪", 调大编译期 ONE_GRAB_MIN_DELAY_MS(调参 Tab; 从整段平移起点
                                                      * 起算, **只作用于抓取**; 放置用 ulPlaceDelayMs 自己的延时)。
                                                      * mode: 0=不纠正 1=视觉yaw转正(ball_yaw_en=1时转完复测, 开关没开则退化为
                                                      * mode 0 并打日志) 2=OpenMV巡线收敛(识别前收敛一次, 抓取时再并行收敛一次;
                                                      * 放置后**不**收敛; 前提=该区看得到线)。**收敛航向来源 ucSrc** = 全局
                                                      * LINE_CONV_SRC(0=相机/1=陀螺(咬 fGyroTarget 绝对世界航向)/2=不修/3=融合)。
                                                      * ucCamPct = 融合档相机占比[%; 2026-10-30 起脚本层**一律传 KEEP**
                                                      * （不写 line_cam_pct, 占比用 LINE_CAM_PCT / 蓝牙 line_cam_pct）。
                                                      * 数据不全→失败音不动。
                                                      * 平移分量/方向/收敛时长 = 蓝牙 one_lat_mm / one_lat_dir / one_line_ms。
                                                      * ucRev = 「反向执行」(2026-10-30 加, int 尾槽勾选): 道具换到
                                                      * 路的另一边、相机/臂朝另一边看 → 本步方向整体镜像(对位符号/
                                                      * 间距/拉回/视觉yaw转正取反; 全局标定值不动, 只翻方向不翻轴);
                                                      * 老场地预设不勾 = 原行为。像素空间标定随镜像用「标定」Tab 换。
                                                      * ucSingleStep = 「单步确定」(2026-11 加, int 尾槽):
                                                      * 1=分步执行, 每段结束**停车等 MAIN**: 色球对位→
                                                      * [MAIN]→移动→[MAIN]→抓取→[MAIN]→放到爆爆桶
                                                      * (抓取此时不再按 ulGrabLeadMs 与移动并行, 而是
                                                      * 按 MAIN 后立即发; 放置仍按 ulPlaceDelayMs 从回退
                                                      * 起点计时); 0=老行为(移动×提前量抓取并行)。
                                                      * ⚠️ emPlace=0（「无」）: 抓完仍回退间距分量, 但不投
                                                      * 放置动作; 单步确定的最后一段也不存在（抓取即结束）。 */
uint8_t ucRpiHostageGrabOne(uint32_t ulTimeoutMs, uint8_t ucMode,
                            emArmActionIdTdf emGrab, uint8_t ucKeepRes,
                            uint32_t ulGrabLeadMs, float fGyroTarget, uint8_t ucSrc,
                            uint8_t ucCamPct, uint8_t ucRev, uint8_t ucSingleStep,
                            emArmActionIdTdf emPlace, uint32_t ulPlaceDelayMs);
                                                     /* STEP_HOSTAGE_GRAB_ONE_WAIT 的原语: 人质单动作抓取(→[mode=1/2识别前巡线收敛]
                                                      * →识别→按二维码第三位定位→平移**与抓取并行**(ulGrabLeadMs 同色球口径
                                                      * —— 估时倒计时剩提前量才发臂, 锚定到位而非起点)
                                                      * →[mode=2 等臂期间并行巡线收敛, 修开环平移的横向残差, 与色球块同口径]
                                                      * →等臂空 → [emPlace=0: ucKeepRes=1 把总位移折世界系记进残留偏移,
                                                      * 结束; 不复位不回中] 或 [emPlace≠0: 只回退间距分量到中间对齐位
                                                      * (=爆爆桶位) + 从回退起点延时 ulPlaceDelayMs 投放置动作,
                                                      * 播完清残留]）。
                                                      * ucKeepRes 与收敛的关系取决于间距加在哪根轴: 横向(one_lat_dir=1/2)时收敛
                                                      * 吸收横向残留 → keep_res 设 0; 纵向(3/4)不受影响可共存。
                                                      * emPlace≠0 时 keep_res 不记账（车回中, 结束时清残留）。
                                                      * ucCamPct = 融合档相机占比[%; 2026-10-30 起脚本层**一律传 KEEP**
                                                      * （不写 line_cam_pct, 占比用 LINE_CAM_PCT / 蓝牙 line_cam_pct））。数据不全→失败音不动。
                                                      * ucRev = 「反向执行」(2026-10-30 加, 同色球口径): 对位符号/间距/拉回镜像;
                                                      * **残留自动跟着对称** —— 记账记实际位移, 消费端"叠加残留"公式不变,
                                                      * 同一个"抓左"从"离开时多走"变"少走"(场地换边的验收判据)。
                                                      * ucSingleStep = 「单步确定」(2026-11 加, int 尾槽):
                                                      * 1=分步执行, 每段结束**停车等 MAIN**: 人质对位→
                                                      * [MAIN]→移动→[MAIN]→抓取(最后一段不等);
                                                      * 抓取不再与移动并行(按 MAIN 后立即发); 0=老行为。 */

#endif /* __APP_RPI_H */
