/**
  * @file       app_beam.h
  * @brief      Beam 模块 —— 跨模块全局共享状态的 getter/setter 封装。
  *
  *             思想移植自 pro 项目的 Function/Beam 模块：把分散在各模块头文件里的
  *             extern 裸全局变量，收敛到本模块一处（单一事实来源），外部只能通过
  *             成对的 getter/setter 访问。变量唯一定义点在 app_beam.c。
  *
  *             命名遵循 QEPack 匈牙利前缀：v(void)、f(float)、i(int32_t)、uc(uint8_t)。
  *             并发：Cortex-M4 单字(32位对齐)读写原子，跨任务访问均经函数调用，
  *             FreeRTOS 任务切换即内存屏障，故**单字**状态不加锁；但两类例外必须加
  *             短临界区（实现见 app_beam.c，2026-09/10 渐次补齐）：
  *               ① 同帧多字快照（读方要同一帧的完整值，见 vSet/GetLineData 等各组）；
  *               ② 「读-清」一次性信号（读-改-写不原子，见 ucTakeMenuConfirm 注释
  *                  与 8 个 ucTake*Ready 同款修法）。
  */

#ifndef __APP_BEAM_H
#define __APP_BEAM_H

#include <stdint.h>
#include "app_script.h"      /* emTimerCmdTdf（秒表命令; 定义在那边因为 map.c 只能引到 app_script.h） */

/* ==================== 姿态角（°）==================== */
/* 生产者 = 传感器任务；消费者 = OLED 任务 */
float   fGetAttitudeRoll(void);
float   fGetAttitudePitch(void);
float   fGetAttitudeYaw(void);
void     vSetYawRaw(float fVal);         /* 原始(未扣校准偏置)工作航向, 传感器任务每帧写入 */
float    fGetAttitudeYawRaw(void);       /* 原始工作航向 —— **世界坐标系专用**（校准不影响世界系） */
void    vSetAttitude(float fRoll, float fPitch, float fYaw);

/* ==================== 菜单状态 ==================== */
/* 生产者 = 按键任务；消费者 = OLED 任务 */
int32_t iGetMenuIndex(void);
void    vSetMenuIndex(int32_t iVal);

int32_t iGetAttPage(void);              /* 姿态角显示页：0=Roll 1=Pitch 2=Yaw */
void    vSetAttPage(int32_t iVal);

uint8_t ucTakeMenuConfirm(void);        /* 读后自动清零（一次性信号） */
void    vSetMenuConfirm(void);          /* 置 1（MAIN 按下） */

/* ==================== 拍照结果（2026-09 加）====================
 * 命令 "9" 的**回应**（TAG=0x06 的 COORD 帧，见 app_rpi.h）：
 * 单片机 --"9"--> Pi --'P'--> OpenMV 存盘 --"P,1"--> Pi --本帧--> 单片机 → OLED 闪一句。
 *
 * 生产者 = vRpiTask（收帧时）; 消费者 = OLED 任务（第 4 页「拍照」那一行）。
 *
 * ⚠️ 用「结果 + 时刻」两个量而不是一次性信号: OLED 要按自己的 50ms 节拍判
 *    「**还在 1 秒内**」（显示 1s 后回「拍照」）, 那是"看时间"不是"接事件" ——
 *    信号会被别的读者偷走, 时刻不会（与 ulGetHandshakeDoneTick 同一套做法）。
 * ⚠️ Pi 侧有超时兜底, 所以这条**一定会来**: 要么 ok=1, 要么超时的 ok=0。
 *    单片机自己不设超时（那要另一个定时器, 且分不清"相机慢"与"相机没这功能"）。
 * ⚠️ 语义是「**上一次**拍照的结果」: 再按一次拍照会被新结果覆盖, 不需要清。 */
void    vSetPhotoResult(uint8_t ucOk);   /* 1=成功 0=失败 */
uint8_t ucGetPhotoResult(void);
void     vSetPhotoDoneTick(uint32_t ulTick);  /* 收帧时刻（=0 表示本次上电还没收到过）*/
uint32_t ulGetPhotoDoneTick(void);

/* ==================== 陀螺校准状态（2026-10 加）====================
 * OLED「陀螺校准」项的运行期标签 + 热备自动切换的让路信号。三个量是「结果 + 时刻 +
 * 进行中」同一套做法（同拍照那段）: OLED 按自己的 50ms 节拍看时刻, 不接一次性事件。
 *
 * 生产者 = **脚本任务**（ucGyroCalActiveSource, app_sensors.c —— 校准要阻塞 M0≈21s /
 * HWT101≈7s, 只能在不发布 yaw 的任务里跑）; 消费者 = OLED 任务（标签）+
 * vGyroSrcWatchdogTick（busy 期间不判停摆、不自动切换 —— 校准期间被校那颗**不出帧是
 * 预期动作**, 当停摆切走就是白跳一轮）。
 * ⚠️ busy 期间车必须静止 —— 这是给操作员看的（OLED 标签「校准中请勿动车」）, 固件没法替他按住车。 */
void     vSetGyroCalBusy(uint8_t ucVal);       /* 1=校准进行中（看门狗让路 + OLED 换标签） */
uint8_t  ucGetGyroCalBusy(void);
void     vSetGyroCalResult(uint8_t ucOk);      /* 1=校准动作走完 0=没法校（有效源是 ATK） */
uint8_t  ucGetGyroCalResult(void);
void     vSetGyroCalDoneTick(uint32_t ulTick); /* 收尾时刻（=0 表示本次上电还没校过）*/
uint32_t ulGetGyroCalDoneTick(void);

/* ==================== 二维码是人工预设的（2026-09 加）====================
 * 为真 = 当前的二维码值是**人工设的**（OLED 第 2 页编辑写入 / `STEP_QR_SET_TEST` 调试），
 * 不是扫码来的。于是 `STEP_QR_SCAN_WAIT` **直接返回、不问树莓派**, 沿用这个值
 * （见 `app_rpi.c::ucRpiQrScan`）。
 *
 * ⚠️ 为什么必须在发指令**之前**判: `ucRpiWaitQrResult` 一进来就 `vSetQrCode(-1)`
 *    把上一次的结果清掉 —— 只要把 `"1"` 发出去, 手工设的值就已经被「没扫到」覆盖了。
 * ⚠️ 它**只管「要不要扫码」**, 不管值的合法性: 手工值照样是 111..333（OLED 编辑器
 *    每位只能选 1~3; 调试积木另有 c_iQrTestSanitize 兜底）。IF 条件、排爆/反恐读的都是
 *    `iGetQrCode()` 那一个值, 手工设的和扫来的一视同仁, 所以那些地方一行都不用改。
 * ⚠️ RAM 量, **断电/复位清零 —— 比赛上电后默认就是真扫码**。
 *    所以台架演练完要记得重新上电, 否则「扫码步骤被跳过」会一直生效;
 *    OLED 第 1 页的 `QR:nnn*`（后缀星号）与遥测 `qr_manual` 就是为这个加的可见性。 */
uint8_t ucGetQrManual(void);
void    vSetQrManual(uint8_t ucVal);

/* ==================== 机械臂已展开（2026-09 加）====================
 * 语义：**本次上电**机械臂已经从「初始折叠位」**安全展开过**（跑过「机械臂待命」
 * 那条三段式展开序列）。生产者 = 机械臂动作层（app_arm_action.c：任何路径把它播完都置位）
 * 与脚本任务；消费者 = 脚本任务（OLED「执行任务」的前置条件）+ 蓝牙只读遥测。
 *
 * ⚠️ 它说的**不是**「现在正停在待命位姿」：用户的口径是「展开过就够了, 之后臂挪到哪儿
 *    都无所谓」。所以臂干完活停在别处时本标志仍然是 1。
 * ⚠️ 是 **RAM 量, 复位清零, 故意如此**：复位后操作员可能手工掰过臂, 重跑一遍展开序列
 *    是安全的（只是多动几秒）—— 比「复位后误以为已展开、从折叠位直接摆任务位姿撞桌子」好。
 * ⚠️ 为什么需要它：任务动作（抓球/打靶/抓人质）都**假定臂已展开**, 从折叠位直接摆过去
 *    会撞; 而把「机械臂待命」那一步塞进每个任务预设里又不对 —— 它只需要跑一次。 */
uint8_t ucGetArmDeployed(void);
void    vSetArmDeployed(uint8_t ucVal);

/* ==================== 激光（继电器）开关状态（2026-10 加）====================
 * 激光继电器**当前的通断状态**。生产者 = **执行器写引脚的那两条**
 * （app_actuator.c 的 vRelayOn / vRelayOff —— 脚本积木 STEP_LASER_ON/OFF、
 * vLaserShoot、上电初始化统统经它们, 没有别的写引脚的路径）;
 * 消费者 = OLED 任务（第 3 页「开关激光」那一行, app_oled.c）。
 *
 * ⚠️ 为什么记这个量、而不是让 OLED 自己数一下按键次数: 激光有**三个**写者
 *    （OLED 菜单、路线里的 STEP_LASER_ON/OFF、vLaserShoot）, 各记影子状态必然跟
 *    实物脱节 —— 比如路线跑过一次 vLaserShoot（开→等→关）之后激光已经灭了,
 *    菜单那行却还显示"开"。把状态收在 vRelayOn/vRelayOff 这一对收口里, 它就
 *    永远等于引脚电平（与 arm_deployed 同一套思路）。
 * ⚠️ RAM 量, 复位清零; 引脚初始化也是低电平（关）, 两者天然对齐。
 * ⚠️ 它是**危险执行器**的状态, 菜单那行必须如实显示 —— 别为了界面好看去猜/缓存。 */
uint8_t ucGetLaserOn(void);
void    vSetLaserOn(uint8_t ucVal);

/* 补光灯**当前占空比**（千分比 0~1000, 0=灭, 2026-10 加）—— 激光那个量的同套思路:
 * 补光灯有**四个**写者（OLED「补光灯」开关、路线积木 STEP_BUGUANG_KAI/_GUAN、
 * 任务自动补光 vFillLightTaskBegin/End）, 各记影子必然脱节 —— 而它们**全部**经
 * app_actuator.c::vFillLightSet 这一个 PWM 出口, 所以状态就收在它里面（调用处顺手
 * 写 Beam）, 永远等于实际 CCR。
 * 消费者 = OLED 任务（「补光灯:开/关」标签, 判据 usGetFillDuty() != 0）。
 * ⚠️ RAM 量, 复位清零 = 灭; TIM9 初始化 CCR=0 也是灭, 两者天然对齐。
 * ⚠️ 与任务自动补光是「后写者胜」: 任务步骤末尾的 vFillLightTaskEnd 会把 OLED 刚开
 *    的灯灭掉、OLED 也能关掉任务步骤开着的灯 —— 都是既有语义, 不是 bug（见
 *    app_actuator.c vFillLightTaskBegin 的说明）。 */
void     vSetFillDuty(uint16_t usDuty);
uint16_t usGetFillDuty(void);

/* ==================== 任务补光（2026-10 加, 编译期宏 → 运行时量）====================
 * app_config.h「任务补光」区那 11 个宏从这一版起只是**上电默认**, 真值在这里:
 * 「蓝牙」Tab「补光」组 / OLED「名字调参」按名绑定可写, 断电回宏（与小球对位那批
 * 同一套模式 —— 现场调好再把终值写回宏固化）。
 * 读者 = app_actuator.c::vFillLightTaskBegin（每个任务步骤**现取**, 改完下一步就生效,
 * 不用重编译）。PCT 的哨兵语义原样保留: >100（含 255）= 跟随 fill_task_pct 默认档。 */
uint8_t  ucGetFillTaskEn(void);       void vSetFillTaskEn(uint8_t ucVal);       /* 总开关 */
uint8_t  ucGetFillTaskPct(void);      void vSetFillTaskPct(uint8_t ucVal);      /* 默认亮度[%] */
uint8_t  ucGetFillTaskRoadEn(void);   void vSetFillTaskRoadEn(uint8_t ucVal);   /* 看路 */
uint8_t  ucGetFillTaskRoadPct(void);  void vSetFillTaskRoadPct(uint8_t ucVal);
uint8_t  ucGetFillTaskBallEn(void);   void vSetFillTaskBallEn(uint8_t ucVal);   /* 色球 */
uint8_t  ucGetFillTaskBallPct(void);  void vSetFillTaskBallPct(uint8_t ucVal);
uint8_t  ucGetFillTaskShootEn(void);  void vSetFillTaskShootEn(uint8_t ucVal);  /* 射靶 */
uint8_t  ucGetFillTaskShootPct(void); void vSetFillTaskShootPct(uint8_t ucVal);
uint8_t  ucGetFillTaskHsEn(void);     void vSetFillTaskHsEn(uint8_t ucVal);     /* 人质 */
uint8_t  ucGetFillTaskHsPct(void);    void vSetFillTaskHsPct(uint8_t ucVal);
uint8_t  ucGetFillTaskQrEn(void);     void vSetFillTaskQrEn(uint8_t ucVal);     /* 主摄扫码 */
uint8_t  ucGetFillTaskQrPct(void);    void vSetFillTaskQrPct(uint8_t ucVal);

/* ==================== OLED 菜单（生产者/消费者见各条注释）==================== */

/* 按键位掩码 —— 生产者 = 按键任务（vKeyHandler）; 消费者 = OLED 任务。
 * ⚠️ 与 ucTakeMenuConfirm 是**两条独立通道**（同一次 MAIN 按下会各自置一份）:
 *    那条是脚本任务的一次性「继续」键（STEP_STOP / STEP_DEAD / 让位积木在等它）,
 *    读后清零、只能有一个消费者。菜单共用会把它偷走, 所以菜单取这一份。
 * ⚠️ 用 #define 不用 typedef enum: server.py::parse_enums 会 glob Application 下的所有
 *    头文件, 多加一个枚举就会往控制台的参数下拉里塞一批垃圾选项。 */
#define OLED_KEY_MAIN   0x01u
#define OLED_KEY_LEFT   0x02u
#define OLED_KEY_RIGHT  0x04u
#define OLED_KEY_UP     0x08u
#define OLED_KEY_DOWN   0x10u

void    vOledKeyPost(uint8_t ucMask);      /* 按位 OR 累积（50ms 内不同的键都不丢） */
uint8_t ucTakeOledKeyMask(void);           /* 读后自动清零 */

/* 菜单是否占着 MAIN（= OLED 不在第 1 页）。生产者 = OLED 任务; 消费者 = 按键任务。
 * 为真时按键任务**不再**置 ucTakeMenuConfirm() —— 否则「在菜单里按 MAIN 选任务」
 * 会顺带把停在 STEP_STOP 的路线放走（车突然动）。第 1 页为假, 比赛路线行为不变。 */
uint8_t ucGetOledMenuBusy(void);
void    vSetOledMenuBusy(uint8_t ucVal);

/* 脚本任务请求通道 —— 生产者 = OLED 菜单; 消费者 = vScriptTask。
 * 菜单不能直接调 ucRpiHandshake()（无限阻塞 + 破坏 app_rpi.c 的单写者假设）,
 * 也不能自己跑路线（动底盘）, 所以一律投递给脚本任务。 */
#define SCRIPT_REQ_NONE      0u
#define SCRIPT_REQ_TASK      1u   /* ucArg = emScriptPresetIdTdf。跑一个**需要机械臂已展开**的
                                   * 任务预设（OLED「执行任务」那三条）: 机械臂没展开过就先
                                   * 阻塞播完「机械臂待命」再跑; 展开过（见 ucGetArmDeployed）
                                   * 就直接跑 —— 展开是一次性的, 所以不能写进任务预设里 */
#define SCRIPT_REQ_HANDSHAKE 2u   /* 重跑 ucRpiHandshake（脚本任务上下文） */
#define SCRIPT_REQ_LINE_TEST 3u   /* 无限时长巡线收敛（参数取 Beam 实时值） */
#define SCRIPT_REQ_GYRO_CAL  4u   /* 陀螺校准（OLED「陀螺校准」, 2026-10 加）: 校准**有效源**
                                   * 那颗（gyro_src_eff）—— M0 走 gyro_m0_bias_cal（约 21s）,
                                   * HWT101 走 CALSW=1 开自动校准→静止→关（约 7s）; ATK 没有校准
                                   * 协议, 拒绝并往 Pi 打一条调试串。ucArg 不用（0）。
                                   * ⚠️ 全程车必须静止, 且阻塞的是**脚本任务** —— 同一时刻
                                   *    不可能有路线在跑（单任务串行）, 这是这条请求能安全
                                   *    存在的前提, 别把它挪去别的任务。 */

uint8_t vScriptPostReq(uint8_t ucReq, uint8_t ucArg);  /* 1=已投递, 0=槽位被占（丢弃） */
uint8_t ucTakeScriptReq(uint8_t *pucArg);              /* 读后自动清零 */

/* 路线中止请求（**电平量, 不是一次性信号** —— 要被 vScriptRun 与 vLineFollowConverge
 * 两处观察, 见 app_beam.c 注释）。生产者 = OLED 菜单; 消费者 = 脚本任务/巡线闭环。 */
void    vScriptAbort(void);
uint8_t ucGetScriptAbortReq(void);
void    vSetScriptAbortReq(uint8_t ucVal);   /* vScriptTask 每次开始新请求前清零 */

/* 上次菜单「握手让出」跑完的 xTaskGetTickCount（0 = 本次上电还没跑过）。生产者 = vScriptTask;
 * 消费者 = OLED 任务（拿它算「让出成功!」显示 1s —— 看时刻而非接事件）。 */
void     vSetHandshakeDoneTick(uint32_t ulTick);
uint32_t ulGetHandshakeDoneTick(void);

/* ==================== 世界航向 / 运动坐标（移植自 pro Function/Beam）==================== */
/* 世界航向(°)：ucPidMove 完成时写入，vChassisMoveWorld 消费 */
float    fGetWorldHeading(void);
void     vSetWorldHeading(float fVal);

/* 视觉换算的移动偏移(mm)：生产者=坐标变换，消费者=底盘运动 */
float    fGetDirectX(void);
void     vSetDirectX(float fVal);
float    fGetDirectY(void);
void     vSetDirectY(float fVal);

/* 像素原点(px) */
float    fGetOriginX(void);
void     vSetOriginX(float fVal);
float    fGetOriginY(void);
void     vSetOriginY(float fVal);

/* 物块间距(mm)，常量 */
float    fGetDistanceWukuai(void);

/* 大分辨率 像素→毫米 增益（色环细定标，蓝牙可调） */
float    fGetVisualServoPxToMmX(void);
void     vSetVisualServoPxToMmX(float fVal);
float    fGetVisualServoPxToMmY(void);
void     vSetVisualServoPxToMmY(float fVal);

/* 底盘加速度(rpm/s)，vChassisMove 速度模式用 */
int32_t  iGetChassisAccel(void);
void     vSetChassisAccel(int32_t iVal);

/* ==================== 底盘动作组（时间轴速度混合播放器, app_chassis_action.c; 2026-10 加）==================== */
/* 增益/限幅四个量 = 蓝牙「底盘动作组」组的可写运行时量（上电默认 = app_config.h 的
 * CH_ACT_* 同名宏, 断电回宏; 播放器每拍现取, 改完下一拍生效）。
 * busy/playing 的**唯一写者是播放器**（开播写/收尾清）—— 注册进参数表是只读遥测
 * （PC 端 $B,CPLAY 后轮询 ch_busy 判断播完）, 同 arm_seq_busy 的口径: 中止路径
 * 不清它、收尾才清, busy=1 那几拍是诚实的「还在停稳中」。 */
float    fGetChActTimeScale(void);
void     vSetChActTimeScale(float fVal);
float    fGetChActHeadKp(void);
void     vSetChActHeadKp(float fVal);
float    fGetChActHeadKi(void);              /* 转弯航向 PID 的 I（保持层不用） */
void     vSetChActHeadKi(float fVal);
float    fGetChActHeadKd(void);              /* 转弯航向 PID 的 D（抑制超调） */
void     vSetChActHeadKd(float fVal);
float    fGetChActHeadKiMax(void);           /* 积分限幅（同 PIVOT/SPIN_TURN_MAX_INTEGRAL 口径） */
void     vSetChActHeadKiMax(float fVal);
float    fGetChActHeadDb(void);
void     vSetChActHeadDb(float fVal);
float    fGetChActWMax(void);
void     vSetChActWMax(float fVal);
float    fGetChActWheelMax(void);
void     vSetChActWheelMax(float fVal);
/* 支点转弯段（CH_ACT_KIND_PIVOT）的 PID —— 出厂值引用 PIVOT_TURN_*（现场标定） */
float    fGetChPivotKp(void);
void     vSetChPivotKp(float fVal);
float    fGetChPivotKi(void);
void     vSetChPivotKi(float fVal);
float    fGetChPivotKd(void);
void     vSetChPivotKd(float fVal);
float    fGetChPivotKiMax(void);
void     vSetChPivotKiMax(float fVal);
float    fGetChPivotWMax(void);
void     vSetChPivotWMax(float fVal);
uint8_t  ucGetChActBusy(void);
void     vSetChActBusy(uint8_t ucVal);
uint8_t  ucGetChActPlayingId(void);
void     vSetChActPlayingId(uint8_t ucVal);

/* 预览/停止请求槽（读后清零的一次性信号）。写方 = vBtTask（$B,CPLAY/$B,CSTOP）;
 * 读方 = vChassisActionTask（预览请求）/ 播放器逐拍（停止位, 预览与路线两条路都查）。
 * ⚠️ 一次性信号按铁律**不进**蓝牙参数表。 */
uint8_t  ucTakeChActPreviewReq(void);
void     vSetChActPreviewReq(uint8_t ucVal);
uint8_t  ucTakeChActStopReq(void);
void     vSetChActStopReq(uint8_t ucVal);

/* 色环流式细定标标志 / Y 像素差 */
uint8_t  ucGetSehuanStreaming(void);
void     vSetSehuanStreaming(uint8_t ucVal);
int8_t   cGetSehuanYDiff(void);
void     vSetSehuanYDiff(int8_t cVal);

/* 最近一帧树莓派 9 字节包（坐标变换的输入，替代 pro 的 g_aucSerialRxPacket）
 * ⚠️ 9 字节 = 多字, 写者是 vRpiTask、读者是脚本任务, 两者优先级不同 —— 所以
 *    「读」请用 **vGetLastRpiPacket(dst)**（临界区里一次拷走, 不会读到半新半旧）。
 *    pucGetLastRpiPacket 返回内部缓冲, 只在确定不会被写者插进来时用（历史调用点）。 */
uint8_t *pucGetLastRpiPacket(void);
void     vGetLastRpiPacket(uint8_t *pucDst);   /* 拷贝式读（推荐） */
void     vSetLastRpiPacket(const uint8_t *pucSrc);

/* 航向刷新标志（一次性信号）：传感器任务每帧置 1，PID 读后清零 */
uint8_t  ucTakeYawFresh(void);
void     vSetYawFresh(void);
/* 姿态帧累计数（2026-09 加）：与上面那个一次性信号同源（传感器任务真收到一帧才推进），
 * 但**只增不减、谁都能读** —— 差值就是 IMU 新鲜度（同 ulGetLineFrameCount 的用法）。
 * 用途: 巡线航向环的「世界航向」那条要判 IMU 有没有在喂数（yaw 停在旧值 = 误差恒定 =
 * 车按一个常数 w 一直转）。⚠️ 别改回读 ucTakeYawFresh(): 那是破坏性读, app_pid.c 三处在用。 */
uint32_t ulGetYawFrameCount(void);

/* ==================== 陀螺仪选源 + 热备（两口 × 三设备, 2026-10 扩展）====================
 * 模块/接线/驱动/选型见 app_config.h 的「姿态传感器选型开关」段与 app_sensors.h。
 * 生产者 = app_sensors.c（两个口各一个任务 + vGyroSrcWatchdogTick）；消费者 = OLED + 蓝牙遥测。
 *
 * ⚠️ 两个量不要混：
 *    `gyro_src`     = **首选**（操作员设的, 蓝牙可写, 上电默认 = SENSOR_GYRO_SRC 宏）；
 *    `gyro_src_eff` = **有效源**（实际在发的那一路, 只读遥测）。
 *    热备自动切换只动有效源；两者不同 = 自动切换接管了（OLED 标签上带 `!`）。
 *
 * ⚠️ 选源是**运行时**的：两个传感器任务都在跑、都在解析各自那路串口，但**只有有效源
 *    那一支**调 vSetAttitude / vSetYawFresh。两个都发会让 roll/pitch 与 yaw 来自不同
 *    模块 —— 而 vSetAttitude 是一个临界区里的三字快照（同帧强一致）。 */
uint8_t  ucGetGyroSrc(void);          /* 首选: 0=主口(USART6) / 1=副口(USART2) */
void     vSetGyroSrc(uint8_t ucVal);
uint8_t  ucGetGyroSrcEff(void);       /* 有效源（自动切换后的真实发布源）; 只读 */
void     vSetGyroSrcEff(uint8_t ucVal);   /* 唯一写者 = vGyroSrcWatchdogTick */

/* 热备自动切换开关（U8 0~1 ⇒ 蓝牙 Tab 自动渲染成勾选框）; 上电默认 = GYRO_AUTO_SWITCH 宏 */
uint8_t  ucGetGyroAutoSwitch(void);
void     vSetGyroAutoSwitch(uint8_t ucVal);

/* 两个口各自解出的姿态帧累计（只增）。写者 = 对应的传感器任务（与本口是否被选中无关）。
 * **停涨 = 那颗模块掉线 / 选型填错**（现场第一判据）；同时是热备看门狗的存活判据来源。 */
int32_t  iGetGyroFramesMain(void);
void     vBumpGyroFramesMain(void);
int32_t  iGetGyroFramesSub(void);
void     vBumpGyroFramesSub(void);

/* 自动切换发生次数（只增; >0 = 真发生过, 排障时先看它） */
int32_t  iGetGyroAutoSw(void);
void     vBumpGyroAutoSw(void);
float    fGetM0Yaw(void);             /* M0 最近一帧航向角 [°]，**模块原始符号**（未乘 M0_GYRO_YAW_SIGN） */
void     vSetM0Yaw(float fVal);       /* 写 + 帧计数自增（唯一写者 = vSensorTaskM0） */
uint32_t ulGetM0YawFrameCount(void);  /* 只增不减；**停涨 = 模块掉线 / 波特率不对** */

/* RPI 帧到达 / 握手可见（一次性信号）：vRpiTask 置 1，消费者读后清零 */
uint8_t  ucTakeRpiFrameReady(void);
void     vSetRpiFrameReady(void);
uint8_t  ucTakeHandshakeSeen(void);
void     vSetHandshakeSeen(void);

/* ==================== 巡线数据（Pi 30Hz 巡线帧快照）==================== */
/* 生产者 = vRpiTask；消费者 = 巡线/定标控制。
 * err=[-1,1] 归一化偏差, steer=PID 控制量, angle=°; valid=0 丢线（余值无效）。
 * 同帧多字需强一致 → 临界区快照读写（Beam 规则第 4 条）。 */
void     vSetLineData(float fErr, float fSteer, float fAngleDeg, uint8_t ucValid);
void     vGetLineData(float *pfErr, float *pfSteer, float *pfAngleDeg, uint8_t *pucValid);
uint32_t ulGetLineFrameCount(void);       /* 累计帧数, 差值=数据新鲜度 */

/* 巡线航向环里**相机那条的目标值**(°) —— app_line.c 用它算 err = 图像角 − 本值。
 * 生产者 = 蓝牙（$B,SET line_angle_target）/「调参」Tab（原 STEP_LINE_ANGLE_SET 积木已删）;
 * 消费者 = app_line.c::vLineFollowConverge（每次进循环现取现用, 不缓存 —— 于是
 * 蓝牙边跑边改能当场生效, 不必等下一步积木）。
 * ⚠️ 只在积木 `src=相机` 那一档被用到（两种信号源见 app_config.h 的「航向环」段）。
 * ⚠️易失: 断电回 app_config.h 的 LINE_ANGLE_TARGET_DEG 编译期默认;
 * 「调参」Tab 改的才是持久默认（同 BALL_* / HOSTAGE_* 那一套的两层关系）。 */
float    fGetLineAngleTarget(void);
void     vSetLineAngleTarget(float fVal);

/* 巡线航向环里**陀螺那条的目标角**(°) —— 只在积木 `src=陀螺` 那一档被用到, 2026-09 定案。
 * 口径: **世界航向, 零点 = 开机那一刻的车头朝向**, 与 STEP_SPIN_TURN_WAIT / PIVOT_TURN_WAIT
 * 的 absolute=1 完全一致（所以两处的角度数字可以互抄）。
 * app_line.c 里它的误差 = 短弧(本值 − fGetAttitudeYaw())（**真角度** —— 不再乘任何换算尺,
 * 那条尺 line_gyro_scale 随融合一起删了, 见 app_config.h 的删除记录）。
 * 生产者 = 「巡线收敛」积木进步骤时写(每步一个) / 蓝牙 $B,SET; 消费者 = app_line.c（每拍现取）。
 * ⚠️ 本量**只给目标**:"要不要听陀螺"由积木的 `src` 参数定, 它不决定任何权重/开关。
 * 易失: 断电回 app_config.h 的 LINE_GYRO_TARGET_DEG。 */
float    fGetLineGyroTarget(void);
void     vSetLineGyroTarget(float fVal);

/* 巡线航向环**融合档的相机占比**[%]（2026-10-08 加）—— 只在全局 `LINE_CONV_SRC=融合` 时被用到。
 * 语义: 两条源各自走完自己的环算出 w_cam / w_gyro 后, 输出按 相机:陀螺 = 本值:(100−本值)
 * 加权（**输出端**加权, 不是误差端 —— 为什么安全见 app_config.h「删除记录」与 app_line.c）。
 * 上电默认 = LINE_CAM_PCT; 2026-10-30 起脚本不再逐块写（六个调用点一律传 KEEP）,
 * 生产者只剩蓝牙 $B,SET; 消费者 app_line.c 每拍现取。易失: 断电回 LINE_CAM_PCT。 */
float    fGetLineCamPct(void);
void     vSetLineCamPct(float fVal);

/* 巡线直走的 mm→时间换算系数（2026-10 加, 上电默认 LINE_TIME_SCALE）:
 * 估时(ms) × 本值 = 「巡线直走」实际的闭环时长 —— 估时公式是位置模式的梯形曲线,
 * 搬到速度模式斜坡上带系统偏差（实测偏长）, 现场标定: 走过了调小、没走到调大。
 * 消费者 = ucRpiLineStraightWalk（每步现取, 蓝牙改完下一步就生效）; 断电回宏。 */
float    fGetLineTimeScale(void);
void     vSetLineTimeScale(float fVal);

/* ==================== 巡线控制律标定量（2026-10-06 加）====================
 * 增益/死区/限幅/方向的运行时层 —— 上电默认 = app_config.h 的同名宏
 * （LINE_STEER_TO_VX / LINE_WALK_STEER_TO_VX / LINE_VX_SIGN / LINE_VX_MAX /
 *  LINE_CAM_TO_W / LINE_WALK_CAM_TO_W / LINE_CAM_DEADBAND_DEG /
 *  LINE_GYRO_TO_W / LINE_WALK_GYRO_TO_W / LINE_GYRO_DEADBAND_DEG / LINE_W_MAX）。
 * 与上面 line_angle_target 同一套两层关系: 「调参」Tab 改宏 = 持久默认,
 * 蓝牙 $B,SET 改这里的 RAM = 当场生效（断电即回宏默认）。
 * 写方 = vBtTask（经 ucBeamApply 钳位）; 读方 = app_line.c::vLineFollowConverge
 * **每拍现取**（不缓存）→ 蓝牙边跑边改下一拍（20ms）就进控制律。
 * ⚠️ 保护类窗口量（LINE_STALE_TIMEOUT_MS / LINE_GYRO_STALE_MS /
 *    LINE_FIRST_FRAME_TIMEOUT_MS）与收敛判据量（LINE_CONVERGE_* / LINE_CE_*）
 *    **刻意不做**运行时量 —— 安全阀不给无线一键关掉的通路, 判据有跨参约束
 *    （航向阈值 ≥ 对应死区）, 都留在编译期。 */
float    fGetLineSteerToVx(void);
void     vSetLineSteerToVx(float fVal);
float    fGetLineWalkSteerToVx(void);
void     vSetLineWalkSteerToVx(float fVal);
float    fGetLineVxSign(void);
void     vSetLineVxSign(float fVal);
float    fGetLineVxMax(void);
void     vSetLineVxMax(float fVal);
float    fGetLineCamToW(void);
void     vSetLineCamToW(float fVal);
float    fGetLineWalkCamToW(void);
void     vSetLineWalkCamToW(float fVal);
float    fGetLineCamDeadband(void);
void     vSetLineCamDeadband(float fVal);
float    fGetLineGyroToW(void);
void     vSetLineGyroToW(float fVal);
float    fGetLineWalkGyroToW(void);
void     vSetLineWalkGyroToW(float fVal);
float    fGetLineGyroDeadband(void);
void     vSetLineGyroDeadband(float fVal);
float    fGetLineWMax(void);
void     vSetLineWMax(float fVal);

/* ==================== 底盘速度指令（最后一次下发的 vx/vy/w, rpm）==================== */
/* 生产者 = vChassisMove（脚本 / PID / 巡线闭环都经它下发）；消费者 = 遥测 / 调试
 * （第 1 页第 4 行 2026-10 已改显示秒表, 不再显示本值）。
 * 用途: 巡线闭环调参时看「最终给到电机的横移量」—— 闭环那一步调的正是
 *       vChassisMove(vx,0,0), 所以本值第一个数 = 乘完 LINE_STEER_TO_VX / LINE_VX_SIGN
 *       并限幅之后的最终横移速度。丢线时闭环会下发 (0,0,0), 这里如实显示 0。
 * 注意: 只反映**速度模式**指令; 位置模式移动（vChassisMoveWorld* / 弧线转弯）不发速度,
 *       本值会保持上一次的（显示的是"最后一次速度指令", 不是"当前实际转速"）。
 * 三个字需强一致 → 临界区快照读写（Beam 规则第 4 条）。 */
void     vSetChassisSpdCmd(int32_t iVx, int32_t iVy, int32_t iW);
void     vGetChassisSpdCmd(int32_t *piVx, int32_t *piVy, int32_t *piW);   /* 指针可为 NULL */

/* ==================== 状态页秒表（第 1 页第 4 行 Time）==================== */
/* 生产者 = 脚本任务（STEP_TIMER 积木）/ OLED 任务（OLED_ACT_TIMER 菜单项）；
 * 消费者 = OLED 任务（vOledPageStatus 每 50ms 取一次）。
 *
 * 命令枚举 emTimerCmdTdf 定义在 app_script.h（地图积木的参数类型, map.c 要能
 * 直接写出 TIMER_CMD_* —— 它只经 map.h 引 app_script.h, 引不到本文件）。
 *
 * 语义（三个动作码就是地图 / OLED 两个积木库里那个下拉的选项）:
 *   开始 = 从当前累计值继续走（已在走则保持, 不重新计时）;
 *   结束 = 冻结在当前累计值（再按开始 = 接着走）;
 *   重置 = 清零并停止。
 * 上电初值 = 停止 + 0。累计值按 xTaskGetTickCount 现算, 不占定时器、不阻塞。 */
void     vTimerCmd(emTimerCmdTdf emCmd);
uint32_t ulTimerElapsedMs(void);          /* 运行中现算, 停止时 = 冻结值 */
uint8_t  ucTimerEverStarted(void);        /* 本次上电是否**开始过**（健康检查显示让位判据） */

/* ==================== 相机健康检查结果（第 1 页第 4 行 H:...）====================
 * 生产者 = 脚本任务（ucRpiHealthCheck, 握手积木 health 参数勾选时）;
 * 消费者 = OLED 任务（vOledPageStatus）与 IF 条件（emScriptCondTdf 的 HEALTH_*）。
 * 四项：M=主摄 / Q=扫码副摄 / Pi=CSI / MV=OpenMV。
 * 每项：0=失败/超时 1=通过 2=未检查（本地初始值; Pi 的帧只发 0/1）。
 * ⚠️ busy=1 表示检查**正在进行**（OLED 画 H:M?Q?Pi?MV?）; 结果帧到达时由
 *    vRpiTask 先 vSetHealthResult、再 vSetHealthBusy(0), 最后置 ready —— 按 ready 门
 *    消费的一方拿到的是同一帧的四项。 */
void     vSetHealthBusy(uint8_t ucBusy);
uint8_t  ucGetHealthBusy(void);
void     vSetHealthResult(uint8_t ucM, uint8_t ucQ, uint8_t ucPi, uint8_t ucMv);
void     vGetHealthResult(uint8_t *pucM, uint8_t *pucQ, uint8_t *pucPi, uint8_t *pucMv);
void     vSetHealthReady(void);
uint8_t  ucTakeHealthReady(void);         /* 结果到达（一次性信号, 读后清零） */

/* ==================== 二维码结果 ==================== */
/* 生产者 = ucRpiWaitQrResult（脚本任务）；三位数结果, -1 = 无结果/失败 */
int32_t  iGetQrCode(void);
void     vSetQrCode(int32_t iVal);

/* ==================== 人质识别结果 ==================== */
/* 生产者 = vRpiTask（解析 Pi 的 TAG=0x02 结果帧，见 app_rpi.h 帧格式）；
 * 消费者 = 脚本任务（STEP_HOSTAGE_SCAN_WAIT）/ 后续抓取动作。
 * 目标已由 Pi 端按中心 x 升序（图像从左到右）排序。 */
typedef enum {
    SHAPE_CYLINDER = 1,   /* 圆柱 cylinder */
    SHAPE_CONE     = 2,   /* 圆锥 cone */
    SHAPE_DRUM     = 3,   /* 腰鼓 drum（Pi 模型类名 pellet） */
} emHostageShapeTdf;

typedef struct {
    uint8_t  ucShape;     /* emHostageShapeTdf 形状码 */
    uint16_t usCx;        /* 中心像素 x（1920×1080 画面） */
    uint16_t usCy;        /* 中心像素 y */
} stHostageTgtTdf;

#define HOSTAGE_MAX_COUNT  3u   /* 与 Pi 端 HOSTAGE_MAX_TARGETS 对齐, 双端同步改 */

/* 同帧多字需强一致 → 临界区快照读写（Beam 规则第 4 条, 同 vSetLineData）。
 * 指针参数可为 NULL（只读/只写感兴趣的字段）。 */
void     vSetHostageResult(uint8_t ucStatus, uint8_t ucCount, const stHostageTgtTdf *pstTgt);
void     vGetHostageResult(uint8_t *pucStatus, uint8_t *pucCount, stHostageTgtTdf *pstTgt);

/* 结果到达（一次性信号）：vRpiTask 置 1，ucRpiWaitHostageResult 读后清零 */
uint8_t  ucTakeHostageReady(void);
void     vSetHostageReady(void);

/* ==================== 色球识别结果（排爆区）==================== */
/* 生产者 = vRpiTask（解析 Pi 的 TAG=0x03 结果帧，见 app_rpi.h 帧格式）；
 * 消费者 = 脚本任务（STEP_COLOR_SCAN_WAIT / 小球对位链 STEP_BALL_ALIGN_WAIT，见 app_rpi.c）。
 * 目标已由 Pi 端按中心 x 升序（图像从左到右）排序: index 0/1/2 = 左/中/右。
 * 颜色码沿用 app_motion_types.h 的 color_t（1=红 2=绿 3=蓝），也等于二维码第一位，
 * 故对位链路（ucRpiAlignChain）可直接拿 iGetQrCode()/100 比对——刻意**不新增 typedef enum**：
 * server.py::parse_enums 会 glob Application 下的所有 .h，多一个 enum 会污染控制台参数下拉。 */
typedef struct {
    uint8_t  ucColor;     /* color_t: 1=红 2=绿 3=蓝 */
    uint16_t usCx;        /* 中心像素 x（1920×1080 画面） */
    uint16_t usCy;        /* 中心像素 y */
} stColorTgtTdf;

#define COLOR_MAX_COUNT  3u   /* 与 Pi 端 COLOR_MAX_TARGETS 对齐, 双端同步改 */

/* 同帧多字需强一致 → 临界区快照读写（Beam 规则第 4 条, 同 vSetHostageResult）。
 * 指针参数可为 NULL（只读/只写感兴趣的字段）。 */
void     vSetColorResult(uint8_t ucStatus, uint8_t ucCount, const stColorTgtTdf *pstTgt);
void     vGetColorResult(uint8_t *pucStatus, uint8_t *pucCount, stColorTgtTdf *pstTgt);

/* 结果到达（一次性信号）：vRpiTask 置 1，ucRpiWaitColorResult 读后清零。
 * ⚠️ 一次性信号绝不能进 s_astBeamParam[]（读会清零, 会打断真正的消费者）。 */
uint8_t  ucTakeColorReady(void);
void     vSetColorReady(void);

/* 色球帧扩展量（小球对位, 2026-09）: 与色球结果同帧到达 —— vRpiTask 的写序是
 * 先写这两个、再 vSetColorResult、最后 vSetColorReady, 故按 ready 门消费的一方
 * 拿到的必是同一帧的值; 平时作只读遥测（蓝牙 Tab 的 ball_yaw / ball_fit_ok）。
 * yaw 是**图像系**连线倾角（y 向下, 左高右低为正）, ⚠️ 不是车体航向角 ——
 * 换算比例待现场标定（预期车体角 ≈ 图像角 × ~3.3）。
 * 旧 Pi 帧无扩展字段时恒为 0 / 1。 */
void     vSetBallYawLine(float fVal);
float    fGetBallYawLine(void);
void     vSetBallFitOk(uint8_t ucVal);   /* 1 = 三点共线残差合格; 0 = 有球识别可疑 */
uint8_t  ucGetBallFitOk(void);
void     vSetBallPullDir(uint8_t ucVal); /* 失败帧方向字节(2026-10-26, ALIGN_PULLBACK 用): 0=无信息 1=缺球从画面左出界(内容整体左移=车沿+cx方向过冲) 2=右出界; 成功帧/旧 Pi LEN=3 → 0; 写序=先于 vSetColorResult(同帧一致) */
uint8_t  ucGetBallPullDir(void);

/* 矩形地板识别结果（TAG=0x07, 色球区缺球时的备选对位基准, 2026-09 加）:
 * 同帧多字强一致 → 临界区快照（同色球那一组）。写方 = vRpiTask,
 * 读方 = app_rpi 的 ucRpiWaitRectResult（按 ready 门消费）。
 * 写序: 先 vSetRectResult、最后 vSetRectReady —— 按 ready 门拿到的必是同一帧的值。 */
void     vSetRectResult(uint8_t ucStatus, uint16_t usCx, uint16_t usCy,
                        uint8_t ucFlags, uint8_t ucSpread);
void     vGetRectResult(uint8_t *pucStatus, uint16_t *pusCx, uint16_t *pusCy,
                        uint8_t *pucFlags, uint8_t *pucSpread);
uint8_t  ucGetRectStatus(void);          /* 只给蓝牙遥测 rect_status 用（同 color_status 那几条） */
uint8_t  ucTakeRectReady(void);          /* 结果到达（一次性信号, 读后清零） */
void     vSetRectReady(void);
/* flags: bit0=三边估 bit1~3=用了几条边 bit4=主路径(量胶带内沿)/0=回退(量中线) */

/* 对位方案链的结果（只读遥测, 2026-09 加）: 写方 = app_rpi 的 ucRpiAlignChain。
 * align_ok=1 表示链里有方案成功; align_method = 成功那一槽的方案码(emAlignMethodTdf)。 */
void     vSetAlignOk(uint8_t ucVal);
uint8_t  ucGetAlignOk(void);
void     vSetAlignMethod(uint8_t ucVal);
uint8_t  ucGetAlignMethod(void);

/* ==================== 蜂鸣请求（2026-10-27 加）====================
 * **非阻塞**蜂鸣的唯一投递口: 生产者 = vRpiTask（Pi 发来的 0x0C 蜂鸣命令帧,
 * 见 app_rpi.h）/ app_rpi 各失败点 / 蜂鸣任务自己的陀螺看门狗;
 * 消费者 = app_actuator 的蜂鸣任务（vBuzzerTask, 20ms 一拍）。
 *
 * ⚠️ 为什么不直接调 vBuzzerBeep: 它内部 vTaskDelay（失败音最长 2.1s）——
 *    vRpiTask 禁阻塞、平移精调环按控制周期节拍禁阻塞, 都只能发请求。
 * ⚠️ 覆盖写（后写者胜）: 蜂鸣任务正在响的时候再来的请求**覆盖**槽里的待响项,
 *    不排队不计数 —— 报警音只需要"最新那个失败", 连环失败自然连成一片长响。
 * ⚠️ 模式常量的**值是 Pi 串口协议的一部分**（0x0C 帧第 2 字节, 见 app_rpi.h）,
 *    别改; 未知值由 vRpiTask 解析侧丢弃。 */
#define BUZZER_PAT_NONE   0u   /* 无请求（槽的空闲值, 不投递） */
#define BUZZER_PAT_FAIL   1u   /* 失败音: 响 0.8s → 停 0.5s → 响 0.8s（共 2.1s） */
#define BUZZER_PAT_TICK   2u   /* 短滴: 响 0.2s 一声（平移精调环每丢一次靶 = 一声） */
void     vSetBuzzerReq(uint8_t ucPat);
uint8_t  ucTakeBuzzerReq(void);   /* 读后清零（一次性信号, 消费者只有蜂鸣任务） */

/* 矩形板心对位标定量（蓝牙可调, ⚠️易失: 断电回 app_config.h 的 RECT_ALIGN_* 默认）。
 * ⚠️ 只有两个中心量 —— 比例尺/方向/XY对调/加减速复用小球那一套（BALL_*）。
 * 写方 = app_bt; 读方 = app_rpi（ucRpiRectAlign 每次执行现取现用）。 */
void     vSetRectCxCenter(float fVal);
float    fGetRectCxCenter(void);
void     vSetRectCyCenter(float fVal);
float    fGetRectCyCenter(void);

/* 看路结果（TAG=0x08, 主摄看地面黄色区, 2026-09 加）:
 * 同帧多字强一致 → 临界区快照。写方 = vRpiTask, 读方 = ucRpiWaitRoadResult（按 ready 门消费）。
 * 写序: 先 vSetRoadResult、最后 vSetRoadReady —— 按 ready 门拿到的必是同一帧的值。
 * ⚠️ dx/ex/ey 是 **int16_t**: 方向对调后误差可以为负、黄区贴画面左边时 x 可到 0 附近;
 *    用无符号会被钳成 0 → 算出来的 Δ 是错的**而且不报错**（与 Pi 侧 i16 逐位对齐）。
 * ⚠️ dy10 **追加在最后**（不是插在中间）: 既有实参的位置一个都不动, 老调用点不会串位。
 *    dy = 黄区**下边缘的 y**, u16(0.1px) → 非负, 有效与否看 flags 的 bit4(RPI_ROAD_FLAG_YEDGE)。 */
void     vSetRoadResult(uint8_t ucStatus, int16_t nDx10, int16_t nEx, int16_t nEy,
                        uint8_t ucFlags, uint8_t ucSpread, uint16_t usDy10);
void     vGetRoadResult(uint8_t *pucStatus, int16_t *pnDx10, int16_t *pnEx,
                        int16_t *pnEy, uint8_t *pucFlags, uint8_t *pucSpread,
                        uint16_t *pusDy10);
uint8_t  ucTakeRoadReady(void);          /* 结果到达（一次性信号, 读后清零） */
void     vSetRoadReady(void);
/* flags: bit0=左边缘有效 bit1=黄区贴画面左边(Δx饱和) bit2=黄区贴上下边(只提示, **别当 dy 的门**)
 *        bit3=面积刚够门限 bit4=Δy(下边缘 y)有效 */
void     vSetRoadDxMeas(float fVal);     /* 本端换算成 px 的实测 Δx, 只做遥测 */
float    fGetRoadDxMeas(void);
void     vSetRoadDyMeas(float fVal);     /* 同 dy, 换算成 px 的实测 Δy, 只做遥测（标定就靠它读） */
float    fGetRoadDyMeas(void);

/* 看路对位标定量（蓝牙可调, ⚠️易失: 断电回 app_config.h 的 ROAD_* 默认）。
 * ⚠️ 独立一套, 不与小球/人质/标靶共用（各区理想停车位姿不同）。
 * 写方 = app_bt; 读方 = app_rpi（ucRpiRoadAlign 每次执行现取现用）。
 * ⚠️ 纵向(road_*) / 横向(road_*_y) 两套各自独立: 图像 x↔车体纵、图像 y↔车体横。 */
void     vSetRoadDxIdeal(float fVal);
float    fGetRoadDxIdeal(void);
void     vSetRoadDxIdealRev(float fVal);   /* 反向看路(rev=1)专用的纵向理想值 */
float    fGetRoadDxIdealRev(void);
void     vSetRoadScale(float fVal);
float    fGetRoadScale(void);
void     vSetRoadSign(float fVal);
float    fGetRoadSign(void);
void     vSetRoadDeadband(float fVal);
float    fGetRoadDeadband(void);
void     vSetRoadMaxMm(float fVal);
float    fGetRoadMaxMm(void);            /* ⚠️ 纵/横**共用**这一个安全钳位 */
void     vSetRoadAccel(int32_t iVal);
int32_t  iGetRoadAccel(void);
void     vSetRoadSpeed(int32_t iVal);
int32_t  iGetRoadSpeed(void);
/* 横向（Δy = 黄区下边缘的 y; use_y=1 时才用）*/
void     vSetRoadDyIdeal(float fVal);
float    fGetRoadDyIdeal(void);
void     vSetRoadDyIdealRev(float fVal);   /* 反向看路(rev=1)专用的横向理想值; 0=未标定→该轴跳过 */
float    fGetRoadDyIdealRev(void);
void     vSetRoadScaleY(float fVal);
float    fGetRoadScaleY(void);
void     vSetRoadDySign(float fVal);
float    fGetRoadDySign(void);
void     vSetRoadDyDeadband(float fVal);
float    fGetRoadDyDeadband(void);
/* 饱和救援步距（Δx 贴画面左缘 / use_y=1 时 Δy 下边缘被裁 → 车体系后退这段再重测一次;
 * 0=关闭。⚠️ 方向 = road_sign（与正常位移同一条；饱和帧的**值**不可信、不参与算式，
 * 但方向符号仍由标定确定）；后退后沉淀 ROAD_RESCUE_SETTLE_MS 再重发 "G"） */
void     vSetRoadRescueMm(float fVal);
float    fGetRoadRescueMm(void);

/* 看路校准结果（TAG=0x09, 主摄拟合黄区直边图像方向角, 2026-10 加）:
 * 同帧多字强一致 → 临界区快照（同看路那组）。写方 = vRpiTask, 读方 = ucRpiWaitRoadYawResult。
 * 写序: 先 vSetRoadYawResult、最后 vSetRoadYawReady —— 按 ready 门拿到的必是同一帧的值。
 * ang = 黄区直边与**图像竖直方向**的夹角(0.01°, i16, (−90,+90], 竖直=0/向画面右倒为正)
 * —— 黄区固定在场上、相机固定在车上 ⇒ 它只随**车航向**变, 是陀螺漂移的视觉基准。
 * ⚠️ 消费方 ucRpiRoadYaw **只测不改**（不动车不写航向）。 */
void     vSetRoadYawResult(uint8_t ucStatus, int16_t nAng100, uint8_t ucCols,
                           uint8_t ucRms10, uint8_t ucSpread, uint8_t ucFlags);
void     vGetRoadYawResult(uint8_t *pucStatus, int16_t *pnAng100, uint8_t *pucCols,
                           uint8_t *pucRms10, uint8_t *pucSpread, uint8_t *pucFlags);
uint8_t  ucTakeRoadYawReady(void);       /* 结果到达（一次性信号, 读后清零） */
void     vSetRoadYawReady(void);
/* flags: bit0=方向角有效 bit1=直段偏短/内点少(勉强, 只提示) */
void     vSetRoadYawMeas(float fVal);    /* 本端换算成 ° 的实测方向角, 只做遥测 */
float    fGetRoadYawMeas(void);
void     vSetRoadYawErr(float fVal);     /* (实测−ideal)×方向 [°], 只做遥测（=「陀螺漂了多少」读数） */
float    fGetRoadYawErr(void);

/* 看路校准标定量（蓝牙可调, ⚠️易失: 断电回 app_config.h 的 ROADYAW_* 默认）。
 * 写方 = app_bt; 读方 = app_rpi（ucRpiRoadYaw 每次执行现取现用）。
 * ⚠️ ideal 的**未标定哨兵是 |值|>90, 不是 0** —— 竖直的直边 ang 本来就 = 0°。 */
void     vSetRoadYawIdeal(float fVal);
float    fGetRoadYawIdeal(void);
void     vSetRoadYawDir(float fVal);
float    fGetRoadYawDir(void);

/* 看路校准的**对齐偏置**（2026-10 加）—— 把看路校准的角度读数落到陀螺上的那一套。
 * 链路: STEP_ROAD_YAW_WAIT **每次跑都对齐到相机真值**: bias = wrap180(原始航向 − err)
 * ⇒ 发布 yaw ≡ err, 即**全场航向零点 = "放直"方向**（上电摆放误差/累计漂移一并消掉,
 * 首次也不豁免）; 编译期 ROADYAW_APPLY_EN=1 时 **app_sensors 发布 yaw 统一扣除** ——
 * PID/巡线陀螺腿/OLED 全部受益（现场不碰蓝牙, 无运行时开关）。
 * ⚠️ 世界坐标系不受影响: world_heading 由 app_pid 记录时换算回原始值（fPidWorldYaw）。
 * ⚠️ 偏置**易失**（上电回 0 —— 航向零点本来就该每次开机重新定）。 */
void     vSetRoadYawBias(float fVal);    /* 当前对齐偏置 [°]（校准写入; 发布端扣它） */
float    fGetRoadYawBias(void);
void     vSetRoadYawCalibOk(uint8_t ucVal);  /* 已定标过一次（首次对齐不限幅, 之后每拍限幅） */
uint8_t  ucGetRoadYawCalibOk(void);

/* 小球对位标定量（运行时蓝牙可调, ⚠️易失: 断电回 app_config.h 的 BALL_* 编译期默认;
 * 「调参」Tab 改的才是持久默认）。写方 = app_bt（$B,SET 经 ucBeamApply 钳位）,
 * 读方 = app_rpi（ucRpiBallAlign 每次执行现取现用）。ball_yaw_en=Yaw修正开关(1开0关)。 */
void     vSetBallCxCenter(float fVal);
float    fGetBallCxCenter(void);
void     vSetBallCyCenter(float fVal);
float    fGetBallCyCenter(void);
void     vSetBallPxToMmX(float fVal);
float    fGetBallPxToMmX(void);
void     vSetBallPxToMmY(float fVal);
float    fGetBallPxToMmY(void);
void     vSetBallYawScale(float fVal);
float    fGetBallYawScale(void);
void     vSetBallYawSign(float fVal);
float    fGetBallYawSign(void);
void     vSetBallYawTarget(float fVal);   /* 理想停止位下三球连线角的读数 [°]（图像角） */
float    fGetBallYawTarget(void);
void     vSetBallYawDeadband(float fVal);
float    fGetBallYawDeadband(void);
void     vSetBallDxDeadband(float fVal);
float    fGetBallDxDeadband(void);
void     vSetBallDyDeadband(float fVal);
float    fGetBallDyDeadband(void);
void     vSetBallFwdSign(float fVal);
float    fGetBallFwdSign(void);
void     vSetBallYawEn(uint8_t ucVal);
uint8_t  ucGetBallYawEn(void);
void     vSetBallXySwap(uint8_t ucVal);   /* 1 = 横/纵两路「轴+增益」整套对调（相机装成 90°） */
uint8_t  ucGetBallXySwap(void);
void     vSetBallAlignAccel(int32_t iVal);
int32_t  iGetBallAlignAccel(void);
void     vSetBallAlignSpeed(int32_t iVal);
int32_t  iGetBallAlignSpeed(void);
void     vSetBallPullbackMm(float fVal);  /* 缺球开环拉回距离(ALIGN_PULLBACK, 2026-10-26): ≈一颗球间距, 拉回不求准只求球排回画面; 编译期默认 BALL_PULLBACK_MM */
float    fGetBallPullbackMm(void);

/* 机械臂舵机运行时量（2026-10 加; 上电初值 = app_config.h 的 ARM_SERVO_MAX_SPEED_DPS /
 * ARM_SERVO_CURVE, 「调参」Tab 改的是持久默认）:
 *   arm_speed_dps = 全臂速度闸的**运行时版**（°/s, 影响恒速趋近 + S 曲线削峰）;
 *   arm_curve     = S 曲线插值形状（0=五次S 1=三次S 2=余弦 3=线性）。
 * ⚠️ 两个 setter 都**当场下发**（驱动读的是舵机自己的 fSpeed 字段/全局曲线, 只存 Beam
 *    不下发 = 不生效）—— 与其它"存起来、用时再取"的标定量不同, 别照抄成纯存储。
 * 「舵机速度」积木(STEP_ARM_SPEED_SET)与蓝牙 Tab 都走这两个。 */
void     vSetArmSpeedDps(float fVal);
float    fGetArmSpeedDps(void);
void     vSetArmCurve(uint8_t ucVal);
uint8_t  ucGetArmCurve(void);

/* 人质对位标定量（2026-09 加, 与小球对位**各持一套**, 理由见 app_config.h 的
 * 「人质对位」段头: 两区停止位姿不同, 共用一套会互相打架）。
 * 写方 = app_bt（$B,SET 经 ucBeamApply 钳位）, 读方 = app_rpi（ucRpiHostageAlign
 * 每次执行现取现用, 不缓存）。⚠️易失: 断电回 app_config.h 的 HOSTAGE_* 编译期默认;
 * 「调参」Tab 改的才是持久默认。命名 Hs = Hostage, 与只读遥测 hostage{1,2,3}_* 区分开。 */
void     vSetHsCxCenter(float fVal);
float    fGetHsCxCenter(void);
void     vSetHsCyCenter(float fVal);
float    fGetHsCyCenter(void);
void     vSetHsPxToMmX(float fVal);
float    fGetHsPxToMmX(void);
void     vSetHsPxToMmY(float fVal);
float    fGetHsPxToMmY(void);
void     vSetHsDxDeadband(float fVal);
float    fGetHsDxDeadband(void);
void     vSetHsDyDeadband(float fVal);
float    fGetHsDyDeadband(void);
void     vSetHsSideSign(float fVal);
float    fGetHsSideSign(void);
void     vSetHsFwdSign(float fVal);
float    fGetHsFwdSign(void);
void     vSetHsXySwap(uint8_t ucVal);
uint8_t  ucGetHsXySwap(void);
void     vSetHsAlignAccel(int32_t iVal);
int32_t  iGetHsAlignAccel(void);
void     vSetHsAlignSpeed(int32_t iVal);
int32_t  iGetHsAlignSpeed(void);

/* 单动作抓取标定量（2026-10 加: STEP_BALL_GRAB_ONE_WAIT / STEP_HOSTAGE_GRAB_ONE_WAIT 用;
 * 思路 = 只录一个"抓中间目标"的动作, 非中间目标靠平移 (位置-1)×one_lat_mm 挪到中间再抓）。
 * 写方 = app_bt（$B,SET 经 ucBeamApply 钳位）, 读方 = app_rpi 的 ucRpiBallGrabOne /
 * ucRpiHostageGrabOne。⚠️易失: 断电回 app_config.h 的 ONE_GRAB_* 编译期默认。 */
void     vSetOneLatMm(float fVal);
float    fGetOneLatMm(void);
void     vSetOneLatDir(uint8_t ucVal);      /* 0=不叠加 1=横向+ 2=横向- 3=纵向+ 4=纵向- */
uint8_t  ucGetOneLatDir(void);
void     vSetOneLineMs(int32_t iVal);
int32_t  iGetOneLineMs(void);

/* 抓取任务残留偏移（世界系 mm, 2026-10 加）: 人质单动作抓取**不回中**, 它平移过的
 * 总位移按当时指令航向换算进世界系存在这里; 后续「世界系平移」积木勾了
 * "叠加上次残留偏移"就按 **计划位移 − 残留** 执行（车已偏过去多少就少走多少,
 * 落回路线原定终点）并**清零（一次性, 消费即走）**。
 * 两float成对读写, 走临界区快照（Beam 规则第 4 条: 多字强一致）。
 * 写方 = app_rpi（单动作抓取）, 读方 = app_chassis（vChassisMoveWorld 消费）。
 * path_res_x/y 的只读 getter 只给蓝牙遥测表用（单值, 无清零副作用）。 */
void     vSetPathResidual(float fWx, float fWy);
void     vGetPathResidual(float *pfWx, float *pfWy);          /* 快照读, 不清零 */
void     vConsumePathResidual(float *pfWx, float *pfWy);      /* 原子地"读出+清零" */
float    fGetPathResX(void);                                   /* 仅蓝牙遥测表用 */
float    fGetPathResY(void);

/* ==================== 靶标校准标定量（反恐区打靶, 2026-09 加）==================== */
/* 与前两组同一套惯例（写方 = app_bt 的 $B,SET 经 ucBeamApply 钳位; 读方 = 执行时现取现用）,
 * 但**补得晚**: 标靶校准原先只有 app_config.h 的编译期宏, 是三个标定环节里唯一
 * 「每试一次都要重编译烧写」的 —— 而它偏偏最需要反复试（先定方向、再定比例尺）。
 * 现在「蓝牙」Tab 的「标靶校准」组当场可改, 零重编译; 「调参」Tab 改的仍是持久默认。
 * ⚠️ 易失: 断电回 app_config.h 的 TARGET_* 编译期默认。命名 Tg = TarGet（避开已有的
 *    只读遥测 target1_* / 帧字段名 target_*, 也避开 TARGET_* 宏本身）。 */
void     vSetTgCenterX(float fVal);
float    fGetTgCenterX(void);
void     vSetTgCenterY(float fVal);
float    fGetTgCenterY(void);
void     vSetTgPxToDegX(float fVal);
float    fGetTgPxToDegX(void);
void     vSetTgPxToDegY(float fVal);
float    fGetTgPxToDegY(void);
void     vSetTgXSign(float fVal);
float    fGetTgXSign(void);
void     vSetTgYSign(float fVal);
float    fGetTgYSign(void);
void     vSetTgDeadbandPx(float fVal);
float    fGetTgDeadbandPx(void);
void     vSetTgMaxStepDeg(float fVal);
float    fGetTgMaxStepDeg(void);
void     vSetTgMoveMs(int32_t iVal);
int32_t  iGetTgMoveMs(void);
void     vSetTgMoveSlackMs(int32_t iVal);
int32_t  iGetTgMoveSlackMs(void);
void     vSetTgXySwap(uint8_t ucVal);
uint8_t  ucGetTgXySwap(void);
void     vSetTgCalibTimeoutMs(int32_t iVal);
int32_t  iGetTgCalibTimeoutMs(void);

/* ==================== 标靶对位标定量（反恐区打靶, 2026-09 加）===================== */
/* 走靶标帧(0x04)里**基准靶**的横向偏差去横移底盘, 让车先摆正、再由机械臂细瞄。
 * 与小球/人质对位同构, 标定量同样**各持一套**（反恐区停止位姿不同, 共用就是系统性偏差）,
 * 但**只有横向**: 前后距离由路线决定, 本步不动它（要动就把 vChassisMoveCameraForward
 * 那条补上, 与 hs_* 对称）。
 * 写方 = app_bt 的 $B,SET 经 ucBeamApply 钳位; 读方 = ucRpiTargetAlign 每次执行现取现用。
 * ⚠️ 易失: 断电回 app_config.h 的 TARGET_ALIGN_* 编译期默认; 「调参」Tab 改的才是持久默认。
 * 命名 Ta = Target Align —— 与上面的 tg_*（TarGet 瞄准微调）分开: 那套管"舵机转多少度",
 * 这套管"车走多少毫米", 单位都不同, 别混着调。 */
void     vSetTaCxCenter(float fVal);
float    fGetTaCxCenter(void);
void     vSetTaCyCenter(float fVal);
float    fGetTaCyCenter(void);
void     vSetTaPxToMmX(float fVal);
float    fGetTaPxToMmX(void);
void     vSetTaDxDeadband(float fVal);
float    fGetTaDxDeadband(void);
void     vSetTaSideSign(float fVal);
float    fGetTaSideSign(void);
void     vSetTaXySwap(uint8_t ucVal);
uint8_t  ucGetTaXySwap(void);
void     vSetTaAlignAccel(int32_t iVal);
int32_t  iGetTaAlignAccel(void);
void     vSetTaAlignSpeed(int32_t iVal);
int32_t  iGetTaAlignSpeed(void);

/* ==================== 标靶流控闭环节拍量（反恐区打靶, 2026-09 加）====================
 * 链路 = STEP_TARGET_SERVO_WAIT → app_rpi.c::ucRpiTargetServo（详见该函数与
 * app_config.h 的「标靶流控伺服」段）。
 *
 * ⚠️ 与上面「标靶校准」tg_* 的分工, 一句话:**瞄准点/比例尺/方向/XY对调共用 tg_***,
 *    本组只管"走多快、何时算收敛、何时放弃"这类闭环节拍参数。
 *    物理标定只有一套（同一个相机、同一对舵机）, 所以两方案共享 tg_*; 而节拍是两套方案
 *    各自的实现细节（开环一步到位 vs 10Hz 连续逼近）, 强行统一只会让两边都不好调。
 *
 * 写方 = app_bt（$B,SET 经 ucBeamApply 钳位; 「蓝牙」Tab 的「标靶流控」组）;
 * 读方 = ucRpiTargetServo **进门取一次快照**（算到一半被人从蓝牙改掉也不会中途换控制律）。
 * ⚠️ 易失: 断电回 app_config.h 的 TARGET_SERVO_* 编译期默认;
 *    「调参」Tab 改的才是持久默认（与 BALL_ / HOSTAGE_ / TARGET_ 那几套同一惯例）。
 * 命名 Ts = Target Servo。
 *
 * ⚠️⚠️ 本段注释里**不能**用「BALL_、星号、斜杠」串起来的那种简写（把三个宏名写成
 *    一个通配样式）: 那个「星号紧跟斜杠」的两个字就是 C 块注释的**结束符**, C 注释
 *    又不嵌套 —— 于是整段注释**提前闭合**, 后面的字全变成代码, AC5 报一片
 *    identifier "HOSTAGE_" is undefined 加 unrecognized token（真踩过, 一次 312 个错误
 *    全从这里来）。要并列几个宏前缀就老老实实写成 BALL_ / HOSTAGE_ / TARGET_。
 *    CLAUDE.md 里「别在注释里写带 斜杠星号 的字面量」是同一条规矩的另一半。 */
void     vSetTsCtrlMs(int32_t iVal);       /* 控制节拍 [ms]: 闭环每拍间隔 */
int32_t  iGetTsCtrlMs(void);
void     vSetTsMoveMs(int32_t iVal);       /* 单拍 S 曲线时长 [ms] */
int32_t  iGetTsMoveMs(void);
void     vSetTsStepDeg(float fVal);        /* 单拍修正上限 [°] */
float    fGetTsStepDeg(void);
void     vSetTsConvPx(float fVal);         /* 收敛阈值 [px]: 两轴偏差均低于此值算"在靶" */
float    fGetTsConvPx(void);
void     vSetTsConvTicks(int32_t iVal);    /* 收敛拍数: 连续 N 拍在靶才判收敛 */
int32_t  iGetTsConvTicks(void);
void     vSetTsTravelDeg(float fVal);      /* 单轴累计行程上限 [°]: 超此值判异常, 拒绝放弃 */
float    fGetTsTravelDeg(void);
void     vSetTsStaleMs(int32_t iVal);      /* 帧新鲜度 [ms]: 帧计数停涨超此值 = 冻结不动舵机 */
int32_t  iGetTsStaleMs(void);
void     vSetTsKpScale(float fVal);        /* 增益系数 ×tg_px_to_deg_x/y（振荡时降它, 不动共用标定） */
float    fGetTsKpScale(void);

/* ---- 底盘参与闭环（2026-09 加, 见 app_config.h 的「底盘参与闭环」段头）----
 * 语义: |dx| 落进 CHASSIS_PX 以内后**舵机6 让位**, 改由**底盘位置模式小步前后进给**继续闭环;
 *       画面纵向（图像 cy）仍全程归舵机3。
 * ⚠️ **是"前后"不是"左右"**: 本车相机相对车体装了 90°, 图像横向 cx 对应车体纵移
 *    （证据与推导见 app_config.h 该段头）。所以这四个量配的是「cx 像素 → 车前后 mm」,
 *    与「标靶对位」的 ta_side_sign / ta_px_to_mm_x **不是同一套** ——
 *    ta_ 那两个在 ta_xy_swap=1 下配的是 **cy → 横移**。
 *    共用会让符号对错取决于一个无关开关, 而且会把两套方案的标定绑死
 *    （为调本闭环翻符号会连带改坏「标靶对位」）, 所以独立一套。
 *    初值仍是有依据的猜测: PX_TO_MM 取 BALL_PX_TO_MM_X（它就是"cx→纵移"那把尺）、
 *    SIGN 取 BALL_FWD_SIGN 的等效符号; 但**方向必须上台验一次**。 */
void     vSetTsChassisEn(uint8_t ucVal);       /* 底盘参与闭环总开关: 0=关(出厂默认, 行为与加这个之前完全一致) */
uint8_t  ucGetTsChassisEn(void);
void     vSetTsChassisPx(float fVal);          /* 接管门限 [px]: |dx| 小于它 → 舵机6 让位、底盘接管 */
float    fGetTsChassisPx(void);
void     vSetTsChassisPxToMm(float fVal);      /* 前后比例 [mm/px]: 只影响收敛速度与是否过冲, 不影响终点 */
float    fGetTsChassisPxToMm(void);
void     vSetTsChassisSign(float fVal);        /* 前后方向 (±1) */
float    fGetTsChassisSign(void);
void     vSetTsChassisMm(float fVal);          /* 累计行程闸 [mm]: 安全夹, 按**指令值**累加 */
float    fGetTsChassisMm(void);

/* 阶段化 X→Y + Y轴踢步 + 底盘距离辅助（2026-10-06 加; 语义见 app_config.h「阶段化 X→Y」段头。
 * ts_seq_en=0 时整组不参与, 行为与加这组之前完全一致） */
void     vSetTsSeqEn(uint8_t ucVal);           /* 阶段化开关: 1=先左右后上下, 0=旧并行环 */
uint8_t  ucGetTsSeqEn(void);
void     vSetTsConvXPx(float fVal);            /* 阶段1(左右)收敛阈值 [px], 固件严格< */
float    fGetTsConvXPx(void);
void     vSetTsConvYPx(float fVal);            /* 阶段2(上下)收敛阈值 [px]（比 conv_px 紧, 靠踢步够到） */
float    fGetTsConvYPx(void);
void     vSetTsStallTicks(int32_t iVal);       /* 停滞判定拍数: 连续 N 拍改善<1px → 踢 */
int32_t  iGetTsStallTicks(void);
void     vSetTsKickDeg(float fVal);            /* 踢步幅度 [°]: 独立于 ts_step_deg 限幅 */
float    fGetTsKickDeg(void);
void     vSetTsKickMax(int32_t iVal);          /* 踢步次数上限（全程共用不重置） */
int32_t  iGetTsKickMax(void);
void     vSetTsDistEn(uint8_t ucVal);          /* 距离辅助开关（默认关, 需先实测灵敏度） */
uint8_t  ucGetTsDistEn(void);
void     vSetTsDistPxToMm(float fVal);         /* dy→远近比例 [mm/px], **带符号**（车体横移=离靶远近那根轴, 不是前后） */
float    fGetTsDistPxToMm(void);
void     vSetTsDistMm(float fVal);             /* 距离辅助行程闸 [mm] */
float    fGetTsDistMm(void);

/* ==================== 平移射靶标定量（激光固定在底盘, 2026-10-07 加）====================
 * 链路 = STEP_TARGET_SHOOT_WAIT → app_rpi.c::ucRpiTargetShootAlign（详见该函数头与
 * app_config.h 的「平移射靶」段）。前提: 激光固定在底盘、角度测试阶段手动定死,
 * "瞄准"只剩**沿靶排平移**一个自由度 —— 且轴是**车体纵移 v_y**（本车相机装 90°、
 * 靶排沿车体纵向排开, 沿排对齐 = cx 偏差驱动 vChassisMoveCameraForward, swap=1 本车;
 * ⚠️ 不是横移 —— 首版实现就错在这里, 用户现场抓到"运动的轴弄反了"）。
 * 粗调("6"全画面识别选色→一次纵移) + 精调("7"流帧位置模式小步闭环), 全程不碰机械臂。
 *
 * 与 ta_ / ts_ 的分工:
 *   · 行走加减速**复用 ta_accel / ta_speed**（只管走多快、不管走到哪）;
 *   · 帧新鲜度**复用 ts_stale_ms**（同一条 0x05 流, 新鲜度是 Pi 侧属性）;
 *   · 比例尺/方向/死区**独立一套**（tc_px_to_mm / tc_sign / tc_db_mm, 初值 =
 *     ts_chassis 那对"cx→纵移"的现场标定值）—— 刻意不直接共用 ts_chassis 量:
 *     为调本步翻符号会连带改坏「标靶流控」的底盘接管（同 ts_chassis 当年不共用
 *     ta_side_sign 的理由）;
 *   · 本组核心是**瞄准常数**: tc_cx_aim（本车 swap=1 用列）/ tc_cy_aim（swap=0 才用行）,
 *     **只有 swap 选中的那个参与运算**。标定: 车摆在激光正打中间靶中心的位姿 →
 *     发"6" → 读中间靶 cx 填 tc_cx_aim。
 *     ⚠️ 瞄准常数 0 = 未标定 → 本步拒绝动车（不响, 留痕; 同 road_dx_ideal 的哨兵约定）。
 * 写方 = app_bt 的 $B,SET 经 ucBeamApply 钳位（「蓝牙」Tab 的「平移射靶」组）;
 * 读方 = ucRpiTargetShootAlign 进门取一次快照。
 * ⚠️ 易失: 断电回 app_config.h 的 TARGET_SHOOT_* 编译期默认。命名 Tc = Target Chassis-move。 */
void     vSetTcCxAim(float fVal);              /* 瞄准列 [px]: ta_xy_swap=1(**本车**)时参与(图像X→纵移); 0=未标定 */
float    fGetTcCxAim(void);
void     vSetTcCyAim(float fVal);              /* 瞄准行 [px]: ta_xy_swap=0 时才参与; 0=未标定 */
float    fGetTcCyAim(void);
void     vSetTcGapMm(float fVal);              /* 粗调格距 [mm]: **打边靶走多远的总旋钮** —— 粗调位移=视觉偏差÷GAP_PX×本值, 改 160 只影响粗调 */
float    fGetTcGapMm(void);
void     vSetTcGapPx(float fVal);              /* 每格像素 [px]: 一格靶距在画面里占多少 px(流帧相邻靶 cx 差); 0=未标定拒绝动车 */
float    fGetTcGapPx(void);
void     vSetTcPxToMm(float fVal);             /* 精调前移比例 [mm/px]: **只归精调环**(每拍步长), 别拿它调粗调 —— 那是 tc_gap_mm */
float    fGetTcPxToMm(void);
void     vSetTcSign(float fVal);               /* 前移方向 (±1): cx 偏差为正时车该往前还是往后 */
float    fGetTcSign(void);
void     vSetTcDbMm(float fVal);               /* 前移死区 [mm]: 粗调偏差小于此值不动车(精调环收尾) */
float    fGetTcDbMm(void);
void     vSetTcAccel(int32_t iVal);            /* 平移加速度 (EMM 档位 0~255): 粗调+精调纵移共用, 独立一套不共用 ta_ */
int32_t  iGetTcAccel(void);
void     vSetTcSpeed(int32_t iVal);            /* 平移速度 [rpm]: 位置模式=纵移速度; 速度模式=微调速度上限 */
int32_t  iGetTcSpeed(void);
void     vSetTcVelMode(uint8_t ucVal);         /* 精调微调模式: 0=位置模式(每拍一步, 有顿挫) 1=速度模式(连续修, 平滑) */
uint8_t  ucGetTcVelMode(void);
void     vSetTcMaxMm(float fVal);              /* 粗调上限 [mm]: 粗调一次纵移的钳位(安全夹, 兜 150mm 间距+停车误差) */
float    fGetTcMaxMm(void);
void     vSetTcConvPx(float fVal);             /* 收敛阈值 [px]: 精调环 |沿排偏差| 连续 N 拍低于此值判对准 */
float    fGetTcConvPx(void);
void     vSetTcConvTicks(int32_t iVal);        /* 收敛拍数: 连续 N 拍在靶才判收敛 */
int32_t  iGetTcConvTicks(void);
void     vSetTcKpScale(float fVal);            /* 增益 ×tc_px_to_mm（振荡降它, 不动共用的标定） */
float    fGetTcKpScale(void);
void     vSetTcStepMm(float fVal);             /* 精调单拍上限 [mm] */
float    fGetTcStepMm(void);
void     vSetTcTravelMm(float fVal);           /* 精调累计行程闸 [mm]: 安全夹, 超此值拒绝放弃 */
float    fGetTcTravelMm(void);
void     vSetTcCtrlMs(int32_t iVal);           /* 控制节拍 [ms]: 精调每拍的测量间隔(平移阻塞自带停稳) */
int32_t  iGetTcCtrlMs(void);
/* ----- pi(树莓派 CSI)摄像头那一套（2026-10-16 加, 见 app_config.h 的「平移射靶」段）-----
 * CSI 那颗**固定在底盘上**(主摄装在机械臂上 → 臂位姿/残余振动会让瞄准列漂), 故优先用它看靶;
 * 它打不开/掉线/看不到靶时由 **Pi 内部**退回主摄, 并用回帧 status 的 bit7 告诉本端这一帧
 * 到底是哪台测的（见 app_rpi.h 的 RPI_TGT_ST_CAM_PI）。
 * ⚠️ 只分**瞄准列/行**这一对（相对位置不同）; 其余比例尺/方向/门限暂时共用 ——
 *    实测偏差大再按同一写法细分（最该先补的是每格像素, 见 app_config.h 段头）。
 * ⚠️⚠️ **坐标系与主摄那两根不是一套**: CSI 是**侧装**的, Pi 侧交付前先转 90°
 *    （`config.PICAM_ROTATE=90`）⇒ 画面宽高换个个儿 = **1080×1920**
 *    （瞄准列 0~1080 / 瞄准行 0~1920）。改那个角度或尺寸 = 换坐标系,
 *    这两个常数要重标、app_beam.c 里那两行的钳位也要跟着改。 */
void     vSetTcCxAimPi(float fVal);            /* pi 瞄准列 [px]（0~1080）: ta_xy_swap=1(**本车**)时参与; 0=未标定→本步自动只用主摄(惰性) */
float    fGetTcCxAimPi(void);
void     vSetTcCyAimPi(float fVal);            /* pi 瞄准行 [px]（0~1920）: ta_xy_swap=0 的装法才用 */
float    fGetTcCyAimPi(void);
void     vSetTcUsePicam(uint8_t ucVal);        /* 优先 pi 摄像头: 1=优先CSI(失败自动退回主摄) 0=只用主摄(标定兜底参数时用) */
uint8_t  ucGetTcUsePicam(void);

/* ----- 爆爆桶对位标定量（2026-10-24 加; 见 app_config.h「爆爆桶对位」段头）-----
 * 闭环 = ucTargetShootCore 精调环的桶版（app_rpi.c::ucRpiBarrelAlign, 命令 "J"/0x0B 流帧）。
 * 坐标系 = OpenMV **320×240**（bc_cx_aim 0~320 / bc_cy_aim 0~240）, 与 tc/tg 那些全不是一套。
 * 刻意独立一套标定量（同 tc 不共用 ts_chassis 的理由: 调它不连带改坏射靶）。 */
void     vSetBcCxAim(float fVal);              /* 瞄准列 [px]（0~320）: bc_swap=1(**本车**)时参与; **0=未标定 → 只测不改模式**（起流采 ~3s 报中位 cx, 标定自举） */
float    fGetBcCxAim(void);
void     vSetBcCyAim(float fVal);              /* 瞄准行 [px]（0~240）: bc_swap=0 的装法才参与 */
float    fGetBcCyAim(void);
void     vSetBcSwap(uint8_t ucVal);            /* 轴映射: 1=cx 偏差驱动纵移(本车, 同平移射靶) 0=cy 驱动(换装法才改) */
uint8_t  ucGetBcSwap(void);
void     vSetBcPxToMm(float fVal);             /* 比例尺 [mm/px]: 每拍出力 = 偏差px × 本值 × bc_kp_scale */
float    fGetBcPxToMm(void);
void     vSetBcSign(float fVal);               /* 纵移方向: 偏差单调变大 = 反了改 -1 */
float    fGetBcSign(void);
void     vSetBcDbMm(float fVal);               /* 纵移死区 [mm] */
float    fGetBcDbMm(void);
void     vSetBcAccel(int32_t iVal);            /* 平移加速度（EMM 档位 0~255） */
int32_t  iGetBcAccel(void);
void     vSetBcSpeed(int32_t iVal);            /* 平移速度 [rpm]: 位置模式=纵移速度 / 速度模式=上限 */
int32_t  iGetBcSpeed(void);
void     vSetBcVelMode(uint8_t ucVal);         /* 微调模式: 0=位置(每拍一步) 1=速度(连续修) */
uint8_t  ucGetBcVelMode(void);
void     vSetBcConvPx(float fVal);             /* 收敛阈值 [px] */
float    fGetBcConvPx(void);
void     vSetBcConvTicks(int32_t iVal);        /* 收敛拍数 */
int32_t  iGetBcConvTicks(void);
void     vSetBcKpScale(float fVal);            /* 增益（×bc_px_to_mm） */
float    fGetBcKpScale(void);
void     vSetBcStepMm(float fVal);             /* 位置模式单拍上限 [mm] */
float    fGetBcStepMm(void);
void     vSetBcTravelMm(float fVal);           /* 行程闸 [mm]（安全夹） */
float    fGetBcTravelMm(void);
void     vSetBcCtrlMs(int32_t iVal);           /* 控制节拍 [ms] */
int32_t  iGetBcCtrlMs(void);
void     vSetBcStaleMs(int32_t iVal);          /* 0x0B 帧新鲜度 [ms]（停涨超它冻结, 超 2× 自愈重发 "J"） */
int32_t  iGetBcStaleMs(void);
void     vSetBcMeasMs(int32_t iVal);           /* 只测模式采样时长 [ms]（bc_cx_aim=0 时用） */
int32_t  iGetBcMeasMs(void);

/* 「爆爆桶对位」运行期遥测（不是标定量; 唯一写者 = ucRpiBarrelAlign 的只测/闭环环）:
 * 只测模式标 bc_cx_aim 时直接看它（= 每拍选中桶的 cx 偏差）, 不必翻 Pi/调试日志。 */
void     vSetBucketErr(float fVal);
float    fGetBucketErr(void);

/* ----- 「射靶保持」运行期遥测（2026-10-09 加; 不是标定量, 断电即失效）-----
 * 唯一写者 = app_rpi.c::ucRpiTargetShootHold 的保持环; 消费者 = OLED 射靶测试页
 * （标签上的 d= 与运动中/OK!）+ 蓝牙 Tab「平移射靶」组的只读遥测。
 * shoot_state: 0=空闲(没在跑) 1=粗调 2=运动中(本拍真发了修正) 3=稳住(连续 N 拍在靶)
 *              4=冻结(撞行程闸/noprog, 停车等着操作员再按一格)
 *              5=无目标(流活着但本帧没有目标色) 6=无信号(帧计数停涨=Pi 没在发)
 *              ⚠️ 4/5/6 是三种不同的"车为什么不动", OLED 标签各画一个态;
 *                 改这里的编号要同步 app_rpi.c 的 AIM_STATE_* 与 app_oled.c 的 OLED_SHOOT_ST_*。
 * shoot_settle_tick: 最近一次「运动中→稳住」那一刻, OLED 拿 ucOledWithin 做 1s 的 OK! 窗。 */
void     vSetShootErr(float fVal);
float    fGetShootErr(void);
void     vSetShootState(uint8_t ucVal);
uint8_t  ucGetShootState(void);
void     vSetShootSettleTick(uint32_t ulVal);
uint32_t ulGetShootSettleTick(void);

/* 本步**实际在用**哪台相机（0=主摄 / 1=pi CSI; 2026-10-16 加）: 由回帧的相机位跟着走,
 * 保持档每拍更新 —— 手机上就能看出"现在到底是哪颗在测", 不用翻 Pi 日志。
 * 唯一写者 = ucTargetShootCore; 只读遥测（蓝牙「平移射靶」组的 shoot_cam）。 */
void     vSetShootCam(uint8_t ucVal);
uint8_t  ucGetShootCam(void);

/* ==================== 靶标识别结果（反恐区打靶）==================== */
/* 生产者 = vRpiTask（解析 Pi 的 TAG=0x04 结果帧，见 app_rpi.h 帧格式）；
 * 消费者 = 脚本任务（STEP_TARGET_CALIB_WAIT → app_rpi.c::vTargetCalibAim）。
 * 目标已由 Pi 端按中心 x 升序（图像从左到右）排序: index 0/1/2 = 左/中/右。
 * 颜色码沿用 app_motion_types.h 的 color_t（1=红 2=绿 3=蓝），也等于**二维码第二位**
 * —— 同色球那边一样刻意**不新增 typedef enum**（server.py::parse_enums 会 glob
 * Application 下的所有 .h，多一个 enum 会污染控制台参数下拉）。
 * usR 是**外圆等效半径**（Pi 侧内边界拟合值已按 Ø55/Ø100 折算；固件的视差补偿
 * offset = h·r/50 按外缘半径定义），顺带可当粗略距离标尺。 */
typedef struct {
    uint8_t  ucColor;     /* color_t: 1=红 2=绿 3=蓝 */
    uint16_t usCx;        /* 靶心像素 x（1920×1080 画面） */
    uint16_t usCy;        /* 靶心像素 y */
    uint16_t usR;         /* 靶盘外圆等效半径[px] */
} stTargetTgtTdf;

#define TARGET_MAX_COUNT  3u   /* 与 Pi 端 TARGET_MAX_TARGETS 对齐, 双端同步改 */

/* 同帧多字需强一致 → 临界区快照读写（Beam 规则第 4 条, 同 vSetColorResult）。
 * ⚠️ ucStatus 只收 **bit0**（1=成功）: 解析处已把"这台是哪台"的 bit7 摘掉另发
 *    （见 vSetTargetCam）, 所以三处 `status == 1u` 的消费方一个字节都不用改。 */
void     vSetTargetResult(uint8_t ucStatus, uint8_t ucCount, const stTargetTgtTdf *pstTgt);
void     vGetTargetResult(uint8_t *pucStatus, uint8_t *pucCount, stTargetTgtTdf *pstTgt);

/* 这一帧(0x04)**实际是哪台相机**测的: 0=主摄 / 1=pi CSI（2026-10-16 加）。
 * 写序: vRpiTask 里**紧挨在 vSetTargetReady() 之前**写 —— 按 ready 门消费的一方
 * 拿到的必是同一帧的 cam（与色球那对扩展字段同一条纪律）。 */
void     vSetTargetCam(uint8_t ucVal);
uint8_t  ucGetTargetCam(void);

/* 0x04 **失败帧**的方向字节（2026-10-24 加, 平移射靶"目标色出画面"补步用）:
 * 0=无信息（成功帧 / 旧 Pi LEN=3 / 超时没回帧） 1=目标色在画面**右**侧出界 2=左侧。
 * 写序: vRpiTask 里**先于** vSetTargetResult（同帧一致, 同 BallPullDir 的纪律）;
 * 消费方 = ucTargetShootCore 的补步（读后即清, 快照语义）。 */
void     vSetTargetPullDir(uint8_t ucVal);
uint8_t  ucGetTargetPullDir(void);

/* 结果到达（一次性信号）：vRpiTask 置 1，ucRpiWaitTargetResult 读后清零。
 * ⚠️ 一次性信号绝不能进 s_astBeamParam[]（读会清零, 会打断真正的消费者）。 */
uint8_t  ucTakeTargetReady(void);
void     vSetTargetReady(void);

/* ==================== 靶标流帧快照（反恐区打靶, TAG=0x05, ~20Hz）====================
 * 生产者 = vRpiTask（解析 Pi 的 TAG=0x05 流帧, 见 app_rpi.h 帧格式）;
 * 消费者 = 脚本任务（STEP_TARGET_SERVO_WAIT → app_rpi.c::ucRpiTargetServo）。
 *
 * ⚠️ **与上面的 0x04 快照是两个独立的东西, 别合成一个** —— 一次性的"标靶校准"
 *    (STEP_TARGET_CALIB_WAIT) 读 0x04, 闭环的"标靶流控"(STEP_TARGET_SERVO_WAIT) 读 0x05。
 *    合起来的话, 闭环跑完刚停流时留下的最后一帧会污染随后的终验（终验会拿着一帧
 *    ~20Hz 的旧测量当"权威结果"用）。
 *
 * ⚠️ **刻意不设 ready 一次性信号, 只用帧计数**（同巡线帧的做法）: 流没有"结果到达"
 *    这个概念 —— 消费方关心的是「帧计数还在不在涨」(新鲜度), 而不是「有没有新结果」。
 *    设了 ready 反而会让读它的一方把信号偷走（这正是 app_script.c 的 IF 条件注释里
 *    那条纪律）。count=0（丢目标）也是合法帧, 帧计数照常推进。
 *
 * 同帧多字强一致 → 临界区快照（Beam 规则第 4 条）。
 * ⚠️ ucCam（2026-10-16 加）= 这一帧**实际是哪台相机**测的（0=主摄/1=pi CSI）:
 *    跟着快照走而不是另起一个标量, 因为闭环**每拍**要用它选瞄准列（哪台就是哪把尺）;
 *    Pi 可能**中途**从 CSI 退回主摄, 单看进门那一次会一直用错的瞄准列（静默跑偏）。 */
void     vSetAimFrame(uint8_t ucCount, const stTargetTgtTdf *pstTgt, uint8_t ucCam);
void     vGetAimFrame(uint8_t *pucCount, stTargetTgtTdf *pstTgt, uint8_t *pucCam);
uint32_t ulGetAimFrameCount(void);   /* 累计帧数（差值 = 新鲜度, 同 ulGetLineFrameCount） */

/* ==================== 爆爆桶流帧快照（TAG=0x0B, ~20Hz, 2026-10-24 加）====================
 * 生产者 = vRpiTask（解析 Pi 的 0x0B 流帧, 见 app_rpi.h 帧格式与 docs/串口协议.md §6m）;
 * 消费者 = 脚本任务（STEP_BARREL_ALIGN_WAIT → app_rpi.c::ucRpiBarrelAlign）。
 *
 * ⚠️ **与上面的 0x05 快照是两个独立的东西**（同 0x04/0x05 分开的理由: 流的最后一帧
 *    不能污染别的消费方）。⚠️ 同样**只帧计数、没有 ready**（流没有"结果到达"的概念;
 *    消费方关心「帧计数还在不在涨」= 新鲜度）。单桶定长帧（相机端"挑好再报"）, 无数组。
 * ⚠️ ucValid = status bit0（本拍有桶）; 无桶帧也推计数（新鲜度语义, 同 vSetAimFrame）。 */
void     vSetBucketFrame(uint8_t ucValid, uint16_t usCx, uint16_t usCy,
                         uint16_t usW, uint16_t usH);
void     vGetBucketFrame(uint8_t *pucValid, uint16_t *pusCx, uint16_t *pusCy,
                         uint16_t *pusW, uint16_t *pusH);
uint32_t ulGetBucketFrameCount(void);   /* 累计帧数（差值 = 新鲜度） */


/* ==================== RPI 链路状态 ==================== */
/* 生产者 = app_rpi（握手 / 心跳超时）；消费者 = OLED 任务 */
typedef enum {
    RPI_LINK_WAITING      = 0,   /* 等待握手（Pi 尚未响应） */
    RPI_LINK_OK           = 1,   /* 已握手, 链路正常 */
    RPI_LINK_DISCONNECTED = 2,   /* 曾握手后断开（心跳超时） */
} emRpiLinkStateTdf;

emRpiLinkStateTdf emGetRpiLinkState(void);
void vSetRpiLinkState(emRpiLinkStateTdf emVal);

/* 从最近 RPI 包第 8 字节取检测颜色码（高 nibble）：1=红 2=绿 3=蓝 4=无 */
static inline uint8_t ucGetDetectedColor(void)
{
    return pucGetLastRpiPacket()[8] >> 4;
}

/* 是否检测到有效物块（颜色码 1/2/3） */
static inline uint8_t bIsDetectionValid(void)
{
    return ucGetDetectedColor() < 4;
}

/* ==================== 蓝牙机械臂编辑模式（生产者/消费者见各条注释）==================== */
/* 这三个量跨任务（vBtTask 与脚本任务），按 Beam 规则必须收敛到这里。
 * 每个量都只有一个「写 1」的写者；bt_edit_mode 允许两个写者「都写 0」（见 .c 注释）。 */

/* 脚本任务正停在握手等待阶段 1（ucRpiHandshake）——「臂可以交出去」的唯一判据。
 * 生产者 = 脚本任务；消费者 = vBtTask（用它当 JOG/GRIP/POS 的闸门）。 */
uint8_t  ucGetBtHsWait(void);
void     vSetBtHsWait(uint8_t ucVal);

/* 蓝牙机械臂编辑模式请求。生产者 = vBtTask（收到 $B,MODE,1）；
 * 消费者 = 握手泵 vBtEditPump（停住让位）。 */
uint8_t  ucGetBtEditMode(void);
void     vSetBtEditMode(uint8_t ucVal);

/* 脚本是否正在跑: vScriptRun 进入时置 1、返回时清 0（vScriptTask 随后永久停车）。
 * 蓝牙编辑模式的入场条件之一（脚本没跑 = 路线动不了舵机, 让位安全）+ 只读遥测。 */
void     vSetScriptRunning(uint8_t ucVal);
uint8_t  ucGetScriptRunning(void);

/* 脚本请求槽里是否有待处理请求: 编辑模式入场门用它堵「投递→开跑」之间 running
 * 仍为 0 的几十毫秒窗口（见 app_beam.c 该函数的注释, 2026-10 加）。 */
uint8_t  ucGetScriptReqPending(void);

/* 上一条有效蓝牙指令的 xTaskGetTickCount()——握手泵的看门狗用它判 PC 是否掉线。
 * 生产者 = vBtTask；消费者 = vBtEditPump。0 表示「还没收到过指令」。
 *
 * ⚠️ **已知的第二个写者**（Beam 规则的显式例外, 别当成 bug）：
 *   app_arm_action.c::vArmPlayWait 在长播放期间逐条调 vSetBtCmdTick 喂狗。
 *   因为 $B,PLAY 是阻塞的, 那段时间 vBtTask 收不到新指令, 不喂就会被 8s 空闲
 *   超时判成「PC 掉线」而自动退出编辑模式。两个写者都只写「当前 tick」,
 *   语义是单调推进, 且互斥于编辑模式闸门（只有编辑模式下 vBtTask 才在收指令）。
 *   另一个写者是脚本任务（路线里的 STEP_ARM_PLAY_WAIT）—— 那时编辑模式本就是 0,
 *   喂狗无副作用。 */
uint32_t ulGetBtCmdTick(void);
void     vSetBtCmdTick(uint32_t ulVal);

/* ==================== 组动作角度偏移（台架标定, 2026-09 加） ====================
 * 下发角 = 动作预设里写的角 + 本偏移 —— 把「编辑模式拖准的落点」整体平移到「播放落点」
 * （拖滑杆是恒速趋近、播放是 S 曲线, 死区+齿轮回差让两者停的位置不同）。
 * 写方 = app_bt（$B,SET 经 ucBeamApply 钳位; 蓝牙 Tab 的「臂偏移」组）;
 * 读方 = app_arm.c::vArmGroupMoveNoWait（组动作的唯一收口, Wait/NoWait/$B,ARM 都经它）。
 * ⚠️ 刻意不作用于拖滑杆（$B,JOG）与夹爪: 编辑模式是标定基准, 挪走它就再也标不准。
 * ⚠️ 易失: 断电回 app_config.h 的 ARM_GROUP_OFS_* 编译期默认;
 *    「调参」Tab 改的才是持久默认（与 BALL_* / HOSTAGE_* 同一套惯例）。
 * ucIdx = 组顺序 0..4 → SERVO1, SERVO2, SERVO3, SERVO4, SERVO6; 越界读回 0、写忽略。 */
float    fGetArmGroupOfs(uint8_t ucIdx);
void     vSetArmGroupOfs(uint8_t ucIdx, float fVal);

/* ==================== 机械臂后台序列 ==================== */

/* 有后台序列（vArmPlayNoWait 投递、vArmTask 推进）正在跑。
 * **唯一写者** = app_arm_action.c::vArmActionPump（认领时写 1, 收尾时写 0）;
 * vArmSeqAbort 刻意不碰它（中止后当前那一拍仍在动舵机, 报忙才诚实）。
 * 消费者 = 控制台「蓝牙」Tab 的遥测面板 + 现场排障时看「车不动了是不是臂还在动」。 */
uint8_t  ucGetArmSeqBusy(void);
void     vSetArmSeqBusy(uint8_t ucVal);

/* ==================== 脚本跳转遥测 ==================== */

/* 有了 IF/跳转之后, 「车现在跑到哪张表了」是现场排障第一个要问的问题 —— 这两个量把
 * 脚本帧栈的状态如实报出来, 「蓝牙」Tab 的参数面板里直接能看到（只读）。
 * **唯一写者** = 脚本任务（app_script.c::vScriptRun 在进场/压栈/弹栈时写）;
 * 读者 = 蓝牙 / OLED。单字节读写在 Cortex-M4 上原子, 按 Beam 规则第 4 条不加锁。 */

/* 当前帧所在的预设 id（emScriptPresetIdTdf 的值）; 0 = 未知 / 顶层预设不在注册表里
 * （后者本身就是个有用的诊断: 说明 map.c 的注册表漏了正在跑的那张表）。 */
uint8_t  ucGetScriptPresetId(void);
void     vSetScriptPresetId(uint8_t ucVal);

/* 跳转深度: 0 = 顶层路线, 1+ = 被 STEP_SCRIPT_CALL 跳进来的子预设。
 * ⚠️ 嵌套 IF（STEP_IF/ELSE/END_IF）**不占**这个深度 —— 它是位置配对, 不是调用。 */
uint8_t  ucGetScriptCallDepth(void);
void     vSetScriptCallDepth(uint8_t ucVal);

/* ==================== OLED 菜单动画（2026-10 加） ==================== */
/* 换行动画开关: 上下选择时选中那一行的文字从右侧滑入（横向 8px → 0）。
 * 上电默认 = app_config.h 的 OLED_MENU_ANIM_EN（「调参」Tab 可改, 要重编译）;
 * 写方 = app_bt（$B,SET 经 ucBeamApply 钳位; 蓝牙 Tab「OLED」组, U8/0~1 → 前端渲染成勾选框）;
 * 读方 = app_oled.c 的换行动画（**每拍现取**, 所以是"当场生效"而不是 BALL_* 那种标定量）。
 * ⚠️ 易失: 断电回宏的上电初值。 */
uint8_t  ucGetOledMenuAnim(void);
void     vSetOledMenuAnim(uint8_t ucVal);

/* ==================== 蓝牙可访问参数注册表 ==================== */
/* 设备自述的运行时参数元数据：控制台「蓝牙」Tab 的参数面板完全由这张表派生
 * （参数名/中文名/分组/类型/单位/范围/当前值），控制台侧不写死任何默认值。
 *
 * ⚠️ 类型常量故意用 #define 而不是 typedef enum：CLAUDE.md 明令别往被
 *    server.py::parse_enums glob 到的 Application 头文件里加 typedef enum，
 *    否则会污染「地图」Tab 的参数下拉。 */
#define BEAM_VAL_F32   0u   /* float   */
#define BEAM_VAL_I32   1u   /* int32_t */
#define BEAM_VAL_U8    2u   /* uint8_t */
#define BEAM_VAL_I8    3u   /* int8_t  */

typedef struct
{
    const char *pcName;      /* 参数名（ASCII, 协议用, 唯一） */
    const char *pcLabel;     /* 中文名（界面显示） */
    const char *pcGroup;     /* 分组（中文, 界面按它归类） */
    const char *pcUnit;      /* 单位, 可为 "" */
    uint8_t     ucType;      /* BEAM_VAL_* */
    uint8_t     ucRW;        /* 1=可读可写, 0=只读 */
    uint8_t     ucWarn;      /* 1=运行中改会影响控制律, 界面打 ⚠ */
    float       fMin;        /* 写入范围（只读项无意义） */
    float       fMax;
    float     (*pfGet)(void);   /* 读, 类型已统一适配为 float */
    void      (*pfSet)(float);  /* 写, 只读项为 NULL */
    /* 可选的**下拉选项**（2026-10 加）: "0=主口|1=副口" —— 「蓝牙」Tab 见到它就渲染成
     * 下拉框（否则按类型走勾选框/文本框/只读文本）。只读项也照渲染, 但下拉置灰。
     * ⚠️ 分隔符只能选 `|`/`:` 这类 —— 字段里**不能出现 `,` 或 `\n`**（$B 协议无转义,
     *    一个逗号就把整条记录串行）。NULL/不写 = 无选项, 报文行尾**少一个字段**,
     *    老客户端照旧解析。 */
    const char *pcOptions;
} stBeamParamTdf;

/* 取整张表（*pusCount 回填条目数）。表在 app_beam.c 里 static const, 落 flash。 */
const stBeamParamTdf *cpstGetBeamParamTable(uint16_t *pusCount);

/* 按名字查找, 找不到返回 NULL */
const stBeamParamTdf *cpstFindBeamParam(const char *pcName);

/* 按名字写值（内部先按 fMin/fMax 钳位再调 pfSet）。
 * 返回 0=OK 1=未找到 2=只读 3=超范围（超范围时不写, 由协议回 $B,ERR,RANGE）。 */
uint8_t ucBeamApply(const char *pcName, float fVal);

#endif /* __APP_BEAM_H */
