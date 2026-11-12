/**
  * @file       app_chassis_action.h
  * @brief      底盘「动作组」表 —— 一段可命名的时间轴速度序列, 由路线里的
  *             STEP_CHASSIS_ACTION_PLAY_WAIT 按 ID 引用, 或蓝牙 $B,CPLAY 预览播放。
  *
  *             与机械臂动作预设（app_arm_action）同一套工程链路: 数据表在本模块的
  *             「数据区」, 由 Web 控制台「底盘动作组」Tab 生成与维护; 保存后自动
  *             出现在「地图」积木库里成为一块大积木。播放复用比赛脚本同一批原语
  *             （vChassisMoveA / vArmPlayNoWait / vChassisWorldToBodyEx）, 所以台架上
  *             预览看到的运动与比赛跑路线逐字节同一条路径。
  *
  *             ⚠️ 本头文件的「生成块」与控制台的解析器是一对, 改格式要两边一起改。
  *
  *             ==================== 执行模型（速度混合器） ====================
  *             动作组 = 一张时间轴条目表。播放器以 20ms 节拍推进时间轴, 每拍:
  *               ① 活跃的**车体系**平移段速度向量求和（不过象限表, 2026-10 定案）;
  *               ② 选**唯一航向权威**（转弯段 > 航向保持）出旋转 w;
  *               ③ 四轮转速饱和钳 → 一次 vChassisMoveA 下发。
  *             多段叠加 = 速度向量相加（麦轮解算 speed[]=±vx±vy∓w 天然支持）;
  *             起步/收尾斜坡对称后, 同速同向交叠 = 恒速无缝接续, 异速 = 线性过渡。
  *             「直走 + 末尾转弯」叠加 = 车体方向随转弯旋转的弧线（"边前进边转弯"
  *             的驾驶直觉; 平移不再依赖航向累加器 —— 世界系语义只存在于转弯目标
  *             与保持层）。
  *
  *             航向约定: 转弯只转 90° 的倍数, 世界航向累加器（Beam world_heading）
  *             是 90° 逻辑网格的记账器（供转弯目标/保持层/世界系积木查表）;
  *             转弯段收尾**永远写指令目标**（收敛也好超时也好）, 几度残差由陀螺仪
  *             与视觉锚点吸收, 不进累加器（同 docs/reviews.md 2026-10-09 第 ⑤ 条的
  *             世界航向记账原则）。台架搬车后按「⌖ 航向归零」重建账面/物理关系。
  */

#ifndef __APP_CHASSIS_ACTION_H
#define __APP_CHASSIS_ACTION_H

#include <stdint.h>

/* ==================== 生成块 ====================
 * 下面这一段由 Web 控制台「底盘动作组」Tab **整段重写**, 手改会被下次保存覆盖。
 * 文件其余部分（结构体、API 声明）是手写的, 控制台不碰。 */
/* ==== 生成块: 开始 ==== */
/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次
 * 保存后指向另一个动作组（同 ARM_ACTION_NEXT_ID 的坑）。 */
#define CH_ACT_NEXT_ID    3u

/* 动作组 ID: 路线表的 STEP_CHASSIS_ACTION_PLAY_WAIT 用它引用; 控制台「动作组」
 * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。
 * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emChActIdTdf)I(0)。
 *
 * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的。别改成 #define ——
 *    那样「动作组」参数就没有下拉了。 */
typedef enum {
    CH_ACT_1 = 1,   /* 示例: 前进末尾叠加右转 */
    CH_ACT_2 = 2,   /* 底盘动作组 */
} emChActIdTdf;

/* 预设张数（= g_astChActGroup[] 的长度, 控制台同步维护） */
#define CH_ACT_GROUP_NUM        2u
/* ==== 生成块: 结束 ==== */

/* ==================== 条目类型（#define 而非枚举: 别污染参数下拉） ====================
 * 与 LINE_ANG_SRC_* 同一纪律: 枚举会被 parse_enums 收成下拉选项, 而这里
 * 的常量只服务数据表, 不进任何积木参数。 */
#define CH_ACT_KIND_MOVE      0u   /* 车体系平移段（速度向量, 时长=估时×k 现算） */
#define CH_ACT_KIND_TURN      1u   /* 陀螺闭环转弯段（四轮绕中心自转, 转到位为止） */
#define CH_ACT_KIND_ARM       2u   /* 臂动作事件（到点投 vArmPlayNoWait, 启动即返回） */
#define CH_ACT_KIND_ARM_WAIT  3u   /* 等臂空闲事件（冻结时间轴+停车, 臂泵跑空或超时后恢复） */
#define CH_ACT_KIND_PIVOT     4u   /* 支点转弯段（单侧驱动+陀螺闭环, 2026-10 加）:
                                    * 激活时按最短路径定支点方向并**定死**（每拍带符号
                                    * 驱动速度纠正, 过冲反转不换支点边, 同 vPidPivotTurn）;
                                    * 期间**独占车轮**（平移段不出力, 支点侧要保持力）;
                                    * 增益走 ch_pivot_*（出厂值引用 PIVOT_TURN_*）;
                                    * 同样无段上限（转到位为止 + CH_ACT_TURN_MS 兜底） */

/* ==================== 条目结构体 ====================
 * 一张动作组表 = 一个 stChActItemTdf 数组, 由控制台生成。
 * 字段含义按 ucKind 分组（不相干的字段写 0）:
 *   MOVE    : usStart(时间轴起点) + fWx/fWy(**车体**位移 mm: +X=车头左方,
 *             +Y=车头前方, 两个都填=斜向; 2026-10 起不过世界系象限表 ——
 *             "什么方向就是什么方向", 转弯叠平移 = 车体方向随转弯旋转的弧线)
 *             + nSpeed(rpm) + ucAccel(0~255)
 *             + ucHold(航向保持{1=开,0=关})。时长**不在表里**: 播放开始时按
 *             fChassisMoveTimeEst(距离,加速度,速度) × 时间系数 k 现算 —— k 是
 *             蓝牙运行时量, 改了下一次播放就生效（控制台排版用的是宏默认值）。
 *             速度剖面与估时公式**同形**（长行程梯形/短行程三角）: 窗口末速度
 *             归零、车滑行到点停住, 不再"时间一到全速急停"（剖面内少走的距离
 *             由 k 吸收）。
 *   TURN    : usStart + nDelta(**±90 的倍数**; 相对档不许 0, 绝对档允许 0=咬回
 *             开机朝向) + nSpeed(w 的段内限幅 rpm, **>0 时盖过全局 ch_w_max** ——
 *             段上抬速度必须看得见效果; 0=用全局) + ucAbs(角度模式{0=相对, 1=绝对})
 *             + ucAccel(本段 EMM 加速度档 1~255, 0=自动: 转弯进行中它的**斜坡平滑
 *             是打滑命门**, 设了就用它 —— 优先于平移段与全局; 没设沿用「平移段
 *             最高档/全局」的老选择)。usDur **预留恒 0**。
 *             ⚠️ **没有段上限**（2026-10 取消, 车主定案）: 闭环的自然语义就是「转到
 *             位为止」, 组尾等最后一个转弯收敛（防故障兜底 = CH_ACT_TURN_MS, 只在
 *             IMU 新鲜仍无法收敛的真故障时触发, 正常永远碰不到）。
 *             相对 = 从**当前累加器**转 nDelta（叠加平移时就是在弧线上继续转）;
 *             绝对 = 直接咬 nDelta 这个世界航向（前面的段把航向记账带偏了之后,
 *             用它拉回已知朝向）。两档的目标都落在 90° 网格上, 累加器的记账纪律
 *             不变。执行 = 陀螺 P 环转到目标航向, 连续 CH_ACT_CONV_TICKS 拍进
 *             CH_ACT_CONV_DEG 即收敛。
 *   PIVOT   : usStart + nDelta/nSpeed/ucAbs（与 TURN **同形同义**: ±90 倍数、
 *             相对/绝对、w≤——这里约束的是**驱动侧**轮速）+ ucAccel 恒 0
 *             （斜坡由 ch_pivot 增益闭环自己平滑, 同 PIVOT_TURN_ACCEL=0）。
 *             usDur 恒 0（无段上限, 同 TURN）。执行 = 单侧双轮驱动 + 陀螺闭环
 *             （增益 ch_pivot_*）, 期间独占车轮; 支点方向激活时按最短路径定死。
 *   ARM     : usStart + ucArmId(emArmActionIdTdf)。到点那一拍投
 *             vArmPlayNoWait —— 后写者胜、与路线里 STEP_ARM_PLAY_NO_WAIT 同语义。
 *   ARM_WAIT: usStart + usDur(等臂上限 ms)。到点冻结时间轴、输出零速(+航向保持),
 *             ucGetArmSeqBusy()==0 恢复; 超时(默认 60s)长鸣放行 —— 同臂队列
 *             drain 的「放行但必须响」哲学。 */
typedef struct
{
    uint8_t                ucKind;    /* CH_ACT_KIND_* */
    uint16_t               usStart;   /* 时间轴起点 [ms]（20ms 网格, 控制台吸附） */
    uint16_t               usDur;     /* ARM_WAIT=等臂上限; MOVE/TURN/PIVOT 恒 0（无段上限, 预留） */
    float                  fWx;       /* MOVE: 车体位移 X [mm]（正=车头左方） */
    float                  fWy;       /* MOVE: 车体位移 Y [mm]（正=车头前方） */
    int16_t                nSpeed;    /* MOVE/TURN/PIVOT: 速度 [rpm]（TURN/PIVOT=本段 w≤） */
    uint8_t                ucAccel;   /* MOVE/TURN: EMM 加速度档 1~255（0=跟随全局底盘加速度, 估时同口径）; PIVOT 恒 0 */
    int16_t                nDelta;    /* TURN: 转角/目标航向 [°], ±90 的倍数, 正=左转 */
    uint8_t                ucArmId;   /* ARM: 动作 ID（emArmActionIdTdf） */
    uint8_t                ucHold;    /* MOVE: 航向保持 {1=开, 0=关}（陀螺咬住累加器） */
    uint8_t                ucAbs;     /* TURN: 角度模式 {0=相对(累加器±delta), 1=绝对(咬 delta 这个航向)}。
                                       * ⚠️ 2026-10 **追加在末尾**的尾字段: 老的 10 字段初始化行
                                       *    缺省补 0 = 相对 = 原行为, 一个字节不用改。 */
} stChActItemTdf;

/* 数据区行初始化宏（控制台按条目类型选宏; 全部字段按结构体声明顺序铺开） */
#define CH_ACT_MOVE(s, wx, wy, spd, acc, hold) \
    { CH_ACT_KIND_MOVE,  (s), 0u, (wx), (wy), (spd), (acc), 0,   0u,    (hold), 0u }
#define CH_ACT_TURN(s, dur, delta, spd, abs, acc) \
    { CH_ACT_KIND_TURN,  (s), (dur), 0.0f, 0.0f, (spd), (acc), (delta), 0u,  0u, (abs) }
#define CH_ACT_PIVOT(s, delta, spd, abs) \
    { CH_ACT_KIND_PIVOT, (s), 0u, 0.0f, 0.0f, (spd), 0u,  (delta), 0u,  0u, (abs) }
#define CH_ACT_ARM(s, id) \
    { CH_ACT_KIND_ARM,   (s), 0u, 0.0f, 0.0f, 0,    0u,  0,   (id),  0u,    0u }
#define CH_ACT_ARM_WAIT(s, dur) \
    { CH_ACT_KIND_ARM_WAIT, (s), (dur), 0.0f, 0.0f, 0,  0u,  0,   0u,   0u,    0u }

/* 一个动作组: 稳定 ID + 标题 + 条目表 */
typedef struct
{
    uint8_t                    ucId;      /* 稳定 ID（= 枚举值） */
    const char                *pcTitle;   /* 标题（也进积木库与路线表的标签） */
    const stChActItemTdf      *pstItems;
    uint16_t                   usCount;
} stChActGroupTdf;

extern const stChActGroupTdf g_astChActGroup[CH_ACT_GROUP_NUM];

/* ==================== 播放 ==================== */

/* 整组阻塞播放到走完（仅任务上下文, 一般是脚本任务; 内部 20ms 节拍推进）。
 * 路线里的 STEP_CHASSIS_ACTION_PLAY_WAIT 走这一条 —— 与 $B,CPLAY 预览同一条
 * 播放器代码（预览在 vChassisActionTask 里也调它）, 台架预览=比赛路径。
 * ID 不存在 / 表里有非 90° 倍数的转弯 / 预览正在播 → 长鸣 + 直接返回（不动车）。
 * ⚠️ 播放期间逐拍查脚本中止电平（ucGetScriptAbortReq）与 $B,CSTOP 请求位。 */
void vChassisActionPlayWait(emChActIdTdf emId);

/* 蓝牙预览任务入口（freertos.c 创建, APP_PRIO_SCRIPT 同级）:
 * 等 Beam 的预览请求槽（vSetChActPreviewReq 投递）→ 校验互斥 → 播放。
 * 与机械臂 $B,PLAY 的**阻塞**模式刻意不同: 底盘在跑时必须留一条随时能进的
 * $B,CSTOP —— 所以 vBtTask 只投递就返回, 播放在本任务里走。 */
void vChassisActionTask(void *argument);

/* ==================== 台架草稿（$B,CPUSH 在线推送; 2026-10 加） ====================
 * 解决「控制台改完必须重编译烧录才能预览」: 预览前由控制台把**当前编辑的这一份**
 * 经蓝牙推进固件的 RAM 草稿区 —— $B,CPUSH,<id>,<count> 开窗 → count 条 $B,CIT 行
 * → 收满自动武装。$B,CPLAY,<id> 命中草稿就播草稿, 没有才回落 FLASH 编译表。
 *
 * ⚠️⚠️ **路线/比赛（vChassisActionPlayWait）永远只读 FLASH 编译表** —— 这是本项目
 *    「比赛跑的就是编译进固件的那份」铁律（同 $B,PLAY 注释）: 台架上推的草稿断电
 *    即失, 绝不会渗进比赛; 控制台改完要跑比赛路线就必须 保存 + Keil 重编译 + 烧录。
 * ⚠️ 写侧只有 vBtTask（两条 $B 命令）, 且播放器忙时 Begin/Store 直接拒绝 —— 草稿
 *    不会在播放途中被改写, 预览任务读它是稳定的（Beam 规则第 4 条, 不加锁）。 */
uint8_t ucChActTryBegin(uint8_t ucId, uint8_t ucCount);          /* 开窗; 0=播放器忙 */
uint8_t ucChActTryStore(const stChActItemTdf *pstItem);          /* 收一行; 收满自动武装; 0=没窗口/忙 */
const stChActGroupTdf *pstChActPreviewResolve(uint8_t ucId);     /* 预览取表: 草稿优先, 回落 FLASH */
uint8_t ucChActPreviewExists(uint8_t ucId);                      /* FLASH 或 武装草稿 里有这个 id */

#endif /* __APP_CHASSIS_ACTION_H */
