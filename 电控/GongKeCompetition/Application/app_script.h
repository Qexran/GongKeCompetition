/**
  * @file       app_script.h
  * @brief      脚本步骤表模块（移植自 pro 项目 Motion/script.h）。
  *
  *             把"流程顺序"从代码逻辑里抽成数据表：每个步骤 = 一个原子动作
  *             (一次底盘位移 / 一次使能 / 一段延时)，vScriptRun() 遍历分发到
  *             app_chassis 等模块的阻塞式原语。改路线只改表，不改执行器。
  *
  *             与 pro 的差异 (裸机忙等 → FreeRTOS):
  *               STEP_DELAY 用 vTaskDelay 让出 CPU；
 *   STEP_DEAD                            (无参, 无参, 无参数, 不动车, 等 MAIN 按键继续)
  */

#ifndef __APP_SCRIPT_H
#define __APP_SCRIPT_H

#include <stdint.h>

/* ==================== 步骤类型枚举 ==================== */
/* 步骤设计规则（详见 CLAUDE.md「脚本动作表设计规则」）:
 *   1. 颗粒度粗: 一步 = 一个完整业务任务, 多个小步骤封进一个动作大函数再进表;
 *   2. 一动作一函数: vScriptRun 的 case 只调一个原语函数, map.c 每行尾注释标原语便于跳转;
 *   3. 延时归属: 参与动作过程的延时留在动作函数内, 动作完成后的沉淀延时显式写 STEP_DELAY;
 *   4. 阻塞后缀: 过程性动作必带 _WAIT(阻塞到完成)/_NO_WAIT(启动即返回);
 *      瞬时指令(发帧/设参数/GPIO/PWM 写)与流程控制步骤(STEP_DELAY/STOP/DEAD)无后缀。
 *
 * 参数映射表 (afF / anI 字段顺序) —— 控制台「地图」Tab 的积木参数由此自动派生:
 *   DSL: 名字[=中文名][[单位]][{值=标签,...}][<显隐:参数名=值>]  , 行尾 // 为备注;
 *   枚举型参数的选项自动取自头文件 typedef enum, 布尔型用 {1=开,0=关} 显式给出。
 *   显隐 = 本参数只在本积木的**另一个参数**读到该值时才显示; 只做字符串相等、只看同一个
 *   积木、只支持一个条件, 值里不能有 , ; < >。（语法仍在, 但**目前没有任何积木用到**它
 *   —— 用例见 git 历史里的 use_fusion/gyro_w 那一对。）
 *   ⚠️ 它只影响控制台**显示**, 不影响写盘: 隐藏时该参数照样带在宏里（值一个字符不动）。
 *   右键积木库条目可直接编辑下列内容并写回本文件。
 *   STEP_EMM_ENABLE                      anI: enable=使能{1=使能,0=失能}
 *   STEP_EMM_CTRL_MODE                   anI: mode=环模式{1=开环,2=闭环}   // 切换底盘四电机(地址1~4)的开环/闭环控制模式(EMM 0x46 0x69), 当场生效。⚠️**不写驱动器 Flash**, 掉电自动回原模式(刻意如此: 写进去就没有"改回来"的保证); 开环=当普通步进用(丢步不报错), 闭环=编码器参与(有堵转/到位判断)
 *   STEP_CHASSIS_MOVE_NO_WAIT            anI: vx=横移速度[rpm], vy=前进速度[rpm], w=旋转速度[rpm]   // +vx=右移 +vy=前进; 发完即返回
 *   STEP_CHASSIS_STOP                    (无参)
 *   STEP_CHASSIS_MOVE_WORLD_WAIT         afF: wx=位移X[mm]<控件:move_var>, wy=位移Y[mm]<控件:move_var>; anI: use_res=叠加上次残留偏移{1=是,0=否}
 *   STEP_CHASSIS_MOVE_WORLD_X_WAIT       afF: wx=位移X[mm]<控件:move_var>; anI: use_res=叠加上次残留偏移{1=是,0=否}   // 正=启动左方; 位移参数可绑「位移变量」（app_config.h「路线位移变量」段, 控制台「位移」Tab 管理）; 残留=人质单动作抓取留下的世界系偏移, 勾"是"=本次按**计划−残留**走(车已偏过去就少走、抓取时后撤过就多走), 用完清零
 *   STEP_CHASSIS_MOVE_WORLD_Y_WAIT       afF: wy=位移Y[mm]<控件:move_var>; anI: use_res=叠加上次残留偏移{1=是,0=否}   // 正=启动前方; 位移参数可绑位移变量, 同上一条; 残留同上
 *   STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT afF: wd=位移[mm]<控件:move_var>; anI: dir=斜向{0=左上,1=右上,2=左下,3=右下}   // 世界系斜向平移(方向下拉, 距离带符号: 负=反方向; 位移参数可绑位移变量, 同上)。⚠️2026-10 由「斜左上/斜右上」两块合并而来 —— 原 STEP_CHASSIS_MOVE_WORLD_DIAG_RF_WAIT 已删(积木库里没有它了); dir 追加在 int 末尾, 老行(F1)缺槽补 0 = 斜左上 = 原来那块的正向, 一个字节不用改
 *   STEP_CHASSIS_SET_SPEED_ACCEL         anI: accel=加速度[rpm/s], speed=速度[rpm]
 *   STEP_CHASSIS_SET_ACCEL               anI: accel=加速度[rpm/s]   // 速度模式加速度, 写 Beam
 *   STEP_CROSS_RAMP_WAIT                 afF: dist_mm=冲坡前进量[mm];  anI: align_ms=顶住时长[ms]   // 过减速带: 低速顶住被动对齐 + 大力冲坡; 前一步须把车停在坎前 10~20mm 且车头朝坎, 后面必须排 STEP_DELAY + 一块巡线收敛。⚠️ 2026-10-30 起 src 不再逐块可选: 确需「只修位置不修航向」只能临时把全局 LINE_CONV_SRC 改 2 并重编译
 *   STEP_CHASSIS_ACTION_PLAY_WAIT        anI: action=动作组   // 播放底盘动作组(时间轴速度混合, 2026-10 加): 平移段/陀螺闭环转弯段/臂事件按时间轴叠加连播 —— 直走+转弯叠加=弧线(无停车间隙), 臂事件=边走边收/展臂; 阻塞到整组走完。动作组在控制台「底盘动作组」Tab 排(保存后自动出现在积木库); 预览走蓝牙 $B,CPLAY; 调 k/增益走蓝牙「底盘动作组」组。选项自动取自 emChActIdTdf
 *   STEP_PID_MOVE_WAIT                   afF: target=目标角[°];      anI: vx=横移速度[rpm], vy=前进速度[rpm], time_ms=时长[ms], mode(PID_MODE_*)=模式
 *   STEP_PID_MOVE_AND_TURN_WAIT          afF: target=目标角[°];      anI: vx=横移速度[rpm], vy=前进速度[rpm]
 *   STEP_PID_TURN_WAIT                   afF: target=目标角[°]   // 两段原地转
 *   STEP_BACK_TO_ROAD_WAIT               afF: target=目标角[°]   // 回退回正
 *   STEP_PIVOT_TURN_WAIT                 afF: angle=转角或绝对航向[°];   anI: dir=转向{1=左转,0=右转}, absolute=角度模式{0=相对(从当前朝向转这么多),1=绝对(转到这个IMU航向)}, rev=反向执行{1=是,0=否}   // 支点转弯: 只驱动单侧双轮, 另一侧当支点, 陀螺仪闭环到位(车心会平移)。⚠️绝对模式下 dir 不起作用; 不填 absolute 就是相对(=0)。rev=1(2026-11-01 加)反向执行: 车倒着放/场地换边时勾 —— **转向不变**(左转仍逆时针), **驱动侧换对侧且速度取反**(左转: 右侧正转⇒左侧倒转, 支点换边、车心弧线镜像, 复现正向放置的世界轨迹)
 *   STEP_PIVOT_LINE_WAIT                 afF: angle=转角或绝对航向[°], gyro_target=陀螺目标角[°];   anI: dir=转向{1=左转,0=右转}, absolute=角度模式{0=相对(从当前朝向转这么多),1=绝对(转到这个IMU航向)}, fwd_ms=交接前盲走时长[ms], settle_ms=交接后收尾对准时长[ms], line_ms=交接后沿线走时长[ms], tail_ms=尾段盲走时长[ms], settle_early=settle段启动即返回{1=是,0=否}   // 支点转弯→巡线接管: 边转边等起流(与转弯重叠), 误差进 PIVOT_LINE_HANDOFF_DEG 后把舵交给巡线。四个时长按顺序: fwd_ms 交接前**不看线**直行(转弯刚结束相机看的是转角/弯道); settle_ms 交接后**只对准不前进**(横移+转正); line_ms 交接后**边前进边修**沿线走; tail_ms 沿线走跑完**再盲走**一段(线到头了还要往前顶一点, **在发 LINE_STOP 之后**, 那一段不看线)。settle_ms 与 line_ms 都为 0 时只跑一拍巡线(交接是不停车的, 得让巡线有机会接管残留转速)。交接后听谁的航向 = **全局调参宏 LINE_CONV_SRC**(0相机/1陀螺/2不修/3融合, 出厂 3=融合); 选陀螺/融合时 gyro_target 越界拒绝, 融合档陀螺腿咬它, 融合占比 = LINE_CAM_PCT(出厂 50, 蓝牙 line_cam_pct 可当场覆盖)。前进速度共用 PIVOT_LINE_FWD_SPEED, 都是**体坐标系**。settle_early=是: **只**作用 settle_ms 段 —— 按调参Tab「巡线收敛增强」判据对准即提前进下一段(没达标最多再给超时收敛时间); line_ms 段刻意不吃(时长=行程, 提前退=少走)。⚠️需转弯终点能看到线; 链路不通只做完整转弯（不响, 留痕）
 *   STEP_SPIN_TURN_WAIT                  afF: angle=转角或绝对航向[°];   anI: dir=转向{1=左转,0=右转}, absolute=角度模式{0=相对(从当前朝向转这么多),1=绝对(转到这个IMU航向)}   // 原地转弯: 四轮对称反向出力绕车身中心自转(车心不动), 陀螺仪闭环到位; 参数与支点同形同义
 *   STEP_SPIN_LINE_WAIT                  afF: angle=转角或绝对航向[°], gyro_target=陀螺目标角[°];   anI: dir=转向{1=左转,0=右转}, absolute=角度模式{0=相对(从当前朝向转这么多),1=绝对(转到这个IMU航向)}, fwd_ms=交接前盲走时长[ms], settle_ms=交接后收尾对准时长[ms], line_ms=交接后沿线走时长[ms], tail_ms=尾段盲走时长[ms], settle_early=settle段启动即返回{1=是,0=否}   // 原地转弯→巡线接管(2026-10-30 加): 支点接管的**四轮自转版** —— 转弯本体换 SPIN_TURN_* 增益, 车心不动(支点版会平移), 自转角速度约支点 2 倍 → 同一交接阈值(共用 PIVOT_LINE_HANDOFF_DEG)下交接瞬间的残留自转也约 2 倍, 巡线第一拍要多修一些。参数/时序/交接判据与「支点转弯→误差进阈值交给巡线」**逐位同形**(F2I7), 交接后的全部段与支点版共用同一份固件代码 —— 全部语义看那一条积木的备注, 排表经验可互相套用。⚠️需转弯终点能看到线; 交接后要原地对准、车心不许漂的场合用本积木替代支点版
 *   STEP_ARC_TURN_WAIT                   afF: delta_deg=转角[°], fwd_mm=前进量[mm]   // 位置模式弧线; 转角 +=左转
 *   STEP_ARC_TURN_L90_WAIT / _R90_WAIT   (无参, 弧线 90°)
 *   STEP_ARM_GROUP_WAIT / _NO_WAIT       afF: s1=舵机1, s2=舵机2, s3=舵机3, s4=舵机4, s6=舵机6   // 组=SERVO1~4+6, 夹爪5不在内
 *   STEP_SET_ZHUASHOU_WAIT               anI: kai_he=开合{1=张开,0=闭合}, degree=程度[%]
 *   STEP_ARM_PLAY_WAIT                   anI: action=动作   // 播放已保存的动作预设(整段走 S 曲线, **阻塞到走完**); 选项自动取自 emArmActionIdTdf
 *   STEP_ARM_PLAY_NO_WAIT                anI: action=动作, delay_ms=延迟执行[ms]   // 播放已保存的动作预设(启动即返回); 选项自动取自 emArmActionIdTdf。delay_ms>0: **后台**计时, 到点才把动作投给臂泵(臂闲时≤20ms精度; 有序列在播则顺延到臂空, 不掐正在播的), 计时**不阻塞后续步骤**(转弯/巡线照常先走 —— 老写法只能排 STEP_DELAY 等着, 车就得停); 0=立即(缺省=老行为)。到点前再投一条=后写者胜; 脚本中止/任何新臂指令(vArmSeqAbort)都会**丢弃**还没到点的投递 —— 按了停臂不许事后自己动
 *   STEP_ARM_SPEED_SET                   anI: dps=舵机速度[°/s]   // 中途修改舵机速度(瞬时, 不动车不动臂): 写全臂速度闸并**当场下发**6舵机 —— 影响之后的恒速趋近与S曲线削峰, 对**正在播放**的动作当拍生效(下一拍削峰即用新值)。30~1000越界钳位, 上电回 ARM_SERVO_MAX_SPEED_DPS。想让某段动作慢/快就把这块排在那段动作之前; 30°/s 以下基本不动、1000 等于不限
 *   STEP_ARM_DEPLOY_MARK                 anI: deployed=展开标记{1=已展开,0=未展开}   // 机械臂「已展开」标记的运行时改写(瞬时, 不动臂不发指令): 该标记决定 OLED「执行任务」与可重入预设被调入时**要不要先做一次机械臂待命展开**。置0 = 声明"臂已折回/被动过"（路线把臂收起来之后排一块, 下次任务自动重做展开）; 置1 = 声明已展开。上电初始 = 未展开, 播完待命序列自动置1
 *   STEP_LASER_ON / _OFF                 (无参, 继电器开/关激光)
 *   STEP_BUGUANG_KAI                     anI: custom=自定义亮度{1=是,0=否}, pct=亮度[%]<启用:custom=1>   // 补光灯开(PWM): 不勾=用宏 FILL_LIGHT_ON_DUTY(出厂 100%); 勾「自定义亮度」再填 pct(0~100, 折千分比下发), **只管这一次开灯**, 关了再开又是宏值。两槽都追加在 int 末尾 —— 老行(无参)缺槽补 0 = 不勾 = 用宏, 一个字节不用改
 *   STEP_BUGUANG_GUAN                    (无参, 补光灯关(占空比 0))
 *   STEP_BEEP                            anI: ms=响时长[ms], period=周期模式{1=是,0=否}, times=次数<启用:period=1>, gap_ms=静音间隔[ms]<启用:period=1>   // 蜂鸣器: 不勾「周期」= 响 ms 毫秒(一声); 勾了 = 「响 ms → 静音 gap_ms」循环 times 次(times<1 按 1)。阻塞(内部 vTaskDelay), 时长全占脚本任务; times/gap_ms 只在 period=1 时参与(勾选联动启用, 槽值仍在)
 *   STEP_CALI_WAIT                       anI: method1=方法1, method2=方法2   // 空壳; 选项自动取自 emCaliMethodTdf
 *   STEP_WAIT_HANDSHAKE                  anI: health=健康检查{1=是,0=否}   // 等 Pi 握手 + 静默确认, 无限等待; 链路已 OK 时再执行 = 判为断开、重走一遍握手。**勾选健康检查**时, 握手成功后立刻阻塞做一次四路相机体检(发 "Z", Pi 依次确认 主摄/扫码副摄/CSI/OpenMV 能出帧, 查完释放占用): 等待期间脚本停在原地, 后面任何发车步骤都不会执行; 结果显示在 OLED 第 1 页第 4 行 `H:M√Q√Pi√MV√`(失败项 ×, 检查中 ?), 秒表首次启动后让位给 Time; 结果还能被 IF 条件「健康检查...」读取, 路线可自行对失败项做分支处理(不勾/读取 = 只显示, 路线照常继续)。⚠️ 链路失败/超时按四项全失败显示(不响)。⚠️ 体检结束会释放四路相机占用, 所以它应排在「预热摄像头」积木之前(握手本来就在表头): 排在后会把预热窗口收掉。等待期间按 MAIN 可提前结束静默确认窗 —— 握手链路一通路线就放行, 不用干等 3s; 这一按会保留给后面「按主按键开始」那步直接起步
 *   STEP_QR_SCAN_WAIT                    anI: timeout_ms=超时[ms], algo=解码算法{QR_ALGO_AUTO=自动,QR_ALGO_DIFF=自研}   // 发指令+等结果+失败音, 结果存 iGetQrCode。algo 选项自动取自 emQrAlgoTdf。**自动**=Pi 端「全帧定位→按检测框紧裁→强放大」级联(2026-10 重做, 命中实测 60~75ms, 平移/旋转/缩放都不敏感), 给 3000~5000 即可; **自研**=几何重建(单次实测 4.5s, Pi 用本值当预算, 别小于 6000, 推荐 7500)。⚠️ 超时只在**失败**时才走满, 命中会提前返回
 *   STEP_QR_SCAN_MAIN_WAIT               anI: timeout_ms=超时[ms], algo=解码算法{QR_ALGO_AUTO=自动,QR_ALGO_DIFF=自研}   // 同上, 但用主摄扫码(不占扫码副摄, 可吃"C"预热)。超时取值同上: 自动 3000~5000 / 自研 7500+
 *   STEP_QR_SET_RANDOM                   (无参)   // 随机二维码: 每次执行把结果写成一个随机的合法三位数(每位 1~3, 共 27 种), 不经过 Pi —— 多次扫码失败后的兜底。⚠️ 会打上人工预设标记 → 后面所有二维码扫描步骤直接跳过(只能复位/断电清除; 不响)
 *   STEP_HOSTAGE_SCAN_WAIT               anI: timeout_ms=超时[ms]   // 发"4"+等结果+失败音, 结果存 vGetHostageResult。⚠️超时取 7500(人质检测是主摄 YOLO, ~5s + 开摄像头 ~1s), 别照抄二维码/小球的 6000
 *   STEP_COLOR_SCAN_WAIT                 anI: timeout_ms=超时[ms]   // 发指令+等结果+失败音, 结果存 vGetColorResult; ⚠️车须静止
 *   STEP_RPI_SEND_TASK                   anI: task_x=任务码   // 瞬时发送; 选项自动取自 emRpiTaskTdf
 *   STEP_RPI_WARM_CAM                    anI: cam=预热哪台, uses=使用次数, prof=每次档位<控件:warm_profiles:uses>   // 预热摄像头(只发不等, 瞬时): 让 Pi 在车移动/摆臂的空档先把相机开好, 后面那几次识别就不用等 ~2s 开相机。cam 选项自动取自 emWarmCamTdf(主摄/扫码副摄/pi摄像头/主摄+pi); uses=这台相机接下来要被几个任务用(0/1 与不填同 = 空闲窗口 15s; N≥2 → 窗口放宽到 N×15s); prof=**每次使用各自的档位**(点按钮弹窗, 共 uses 行, 每行 默认/看路档/人质档)。⚠️ 排在**会花时间的那段之前**(移动/巡线/阻塞摆臂), 紧贴着使用点等于白预热。⚠️ **主摄+pi** = 一条积木发两条预热帧(主摄 "C" + CSI "C,1,1"), 两颗同时热着互不干扰; 「平移射靶」前推荐用这一档 —— 它粗调固定主摄、细调走 pi, 而 CSI 冷启动 ~3s 必须在射靶前就热好(细调起流只等 3s)。⚠️ 预热窗口 = uses × 15s, 隔了一段行驶/巡线的排法给 uses=2~3。⚠️ **档位为什么重要**: 普通预热是 main 档(自动曝光), 而看路要手动曝光+固定白平衡、人质要手动 350 —— 从 main 换过去是**曝光模式翻转**, 切换后头 0.5~2s 帧不可用(「第一次看路总是慢」就是这个)。在这里给第几次使用指定档位, Pi 会在**上一次用完的空档**里提前切好, 那一步就不等。二维码/色球/靶标/主摄扫码共用**默认档**, 不用为它们单独设。⚠️ 档位只对 USB 相机有意义, 选 pi 时忽略; 换档预热同样会关掉旧租约, 别在靶标流运行期间发。⚠️ 老路线只有两个槽 → 第三槽补 0(全默认), 行为一点不变
 *   STEP_OPENMV_LED                      anI: level=亮度[0~100]   // OpenMV 补光灯亮度(只发不等, 瞬时): 发"L,<level>"→Pi→相机运行时命令, 当场生效。⚠️ 与车上补光灯(STEP_BUGUANG_*)是**两盏灯**; 0=**临时灭灯**(Pi 不写配置, 原亮度留着, 再开/起流即回去); ≥1 才持久化并在每次巡线起流("2")前自动重发(OpenMV 断电重启也不丢)。OLED「巡线→调参→补光亮度」改的是同一个值
 *   STEP_OPENMV_LED_TOGGLE               (无参)   // OpenMV 补光**临时**开关(只发不等, 瞬时): 亮↔灭。⚠️「灭」是临时的 —— Pi 不写配置, 所以再开一次、或任何一次巡线起流("2")前的自动对齐, 都会把灯点回原亮度; 要设固定亮度用上面那块
 *   STEP_LINE_CONVERGE_WAIT              afF: angle=陀螺目标角[°]; anI: converge_ms=收敛时长[ms], early_ret=启动即返回{1=是,0=否}   // 巡线收敛: 起流 → 按全局 LINE_CONV_SRC 修航向 + 横移纠偏 → 收流。相机=贴线走(**丢线那一拍不修航向**); 陀螺=车头咬死 angle 这个绝对世界航向(线与航向不一致时会斜着平移; **IMU 停摆那一拍不修**); 不修=只修位置(过雪弗板/交界被遮挡时用, 那一刻相机给的 angle 不可信); 融合=相机/陀螺**两条各自走完自己的环**(各自死区/增益照旧), 两个输出 w 按 LINE_CAM_PCT:(100−LINE_CAM_PCT) 加权 —— 陀螺仪会漂、相机量的是场地角度; 谁不可用谁本拍出 0 力, **不把权重补给对方**。angle 在 src=陀螺/融合 时参与(=陀螺腿咬的世界航向, 越界拒绝); 融合占比 = LINE_CAM_PCT(出厂 50, 蓝牙「巡线」组 line_cam_pct 可当场覆盖)。early_ret=是: 按「调参」Tab 巡线收敛增强区的判据**达标即提前返回**(0<实际<收敛时长; 融合档两源各判各的, 阈值 0=不判该源), 没达标最多再给超时收敛时间; =否/缺省: 跑满收敛时长(老行为)
 *   STEP_LINE_STRAIGHT_WAIT              afF: dist_mm=距离[mm], gyro_target=陀螺目标角[°];   anI: accel=加速度[rpm/s], speed=前进速度[rpm], use_res=叠加上次残留偏移{1=是,0=否}, settle_ms=起步对准时长[ms]   // 巡线直走: 沿线**边前进边修**走固定距离 —— mm 按位置模式同款梯形估时反推时长(体坐标系), 再乘**时间系数 k**(蓝牙「巡线」组 line_time_scale: 估时搬到速度模式斜坡带系统偏差, 走过了调小没走到调大, 落点偏差全折进 k 现场标); accel/speed 同时管估时与实际斜坡(accel 钳 1~255, 钳后仍与估时同一个值); dist/speed/accel 任一 ≤0=没配齐, 拒绝不动车(不响, 留痕; 不消费残留); use_res=1 先吃「路径残留偏移」读出即清零, fwd 分量并入距离(side 丢弃, LW2 串报出); 航向源 = 全局 LINE_CONV_SRC(同「巡线收敛」), 融合占比 = LINE_CAM_PCT(出厂 50, 蓝牙 line_cam_pct 可当场覆盖; settle_ms 对准段与沿线段共用), gyro_target 在 src=陀螺/融合 时参与(=陀螺腿目标, 越界拒绝); settle_ms>0: 起流后先**原地**对准这么久再沿线走(照搬支点转弯 settle→line 结构, 把摆歪的起步先修掉 —— 直走全速前进时修正追不上发散, 摆歪起步必先对准; 0=不对准=老行为)
 *   STEP_BALL_ALIGN_WAIT                 anI: timeout_ms=每方案超时[ms], m1=方案1, m2=方案2, m3=方案3   // 色球区多方案对位链: 按 m1→m2→m3 顺序试, 第一个成功即停; 每方案各拿满超时; 空槽=不试(新拖的积木默认三槽都不试, 要填 m1 与超时); 进「拉回（赌）」或全失败/没配 → 失败音不动车（链内其余失败静音）。方案码: 1=三球中球 2=矩形板心 3=缺球开环拉回(读Pi失败帧方向字节→沿排倒回ball_pullback_mm→重扫三球; 需Pi 2026-10-26版)
 *   STEP_HOSTAGE_ALIGN_WAIT              anI: timeout_ms=超时[ms], idx=对位基准{0=左边人质,1=中间人质,2=右边人质}   // 人质对位: 发"4"识别(三只齐才算数) → 取 idx 选定的那只 → cx/cy 偏差横/纵移到位。⚠️超时取 7500(不是 6000: Pi 跑 YOLO ~5s + 开摄像头 ~1s); 标定量 HOSTAGE_*; 不做转正; 识别不全/超时→失败音, 不动车
 *   STEP_ROAD_ALIGN_WAIT                 anI: timeout_ms=超时[ms], use_y=是否同时修横向{0=否,1=是}, rev=反向看路{1=是,0=否}   // 看路对位: 发"G"让主摄找地面上的**黄色区** → Δx=左边缘的 x 修一步**车体纵向**; use_y=1 时再用同一帧的 Δy=黄区**下边缘的 y** 修一步**车体横向**(执行顺序: 先横后纵)。⚠️**单次开环**(内部不死循环), 要迭代就在路线里**连排两块**; 标定量 ROAD_*(蓝牙「看路」组当场可改); 横向要先把 road_dy_ideal/scale_y/sign_y 标好, 没标好只是那一轴跳过(纵向照常); 纵向未标定/没看到黄区/超时 → 失败音, 不动车; ⚠️**饱和救援**(2026-10-30 加): 测到"尺子没数"的帧(①黄区贴画面左缘=横向全黄/Δx≈0 ②use_y=1 时下边缘被裁=蓝牙 road_dy=0) → **车体系开环后退 road_rescue_mm 再重测一次**(不叠加世界坐标系; **方向 = road_sign**, 与正常位移同一条, 不是写死; 后退后沉淀 ROAD_RESCUE_SETTLE_MS 再重发 "G"; 蓝牙 road_rescue_mm 可调, 0=关), 还饱和/重测失败 → 整步失败; 重测用整份 timeout, 整步最坏 ≈2×超时+后退时间+沉淀; rev=1(2026-10-31 加) 发"M"反向看路(返程机位黄区从画面**左上**进入, Pi 整帧镜像后量**右边缘**): **纵向补正与饱和救援方向取反**(2026-11-01 定稿; 横向**不**取反——竖直轴不镜像); 理想Δx/Δy 都有**独立的反向标定量「反向看路理想Δx/Δy」**(蓝牙「看路」组 road_dx_ideal_rev/road_dy_ideal_rev · 标定 Tab 的 ROAD_*_REV_PX, 反向停车位读 road_dx/road_dy 填入; 0=未标定→纵向拒动+失败音、横向跳过), 比例/死区/钳位/方向符号与正向共用
 *   STEP_ROAD_SET_IDEAL                  afF: dx_ideal=理想Δx[px], dy_ideal=理想Δy[px]   // 看路理想值: 把两个理想值写进运行时量(同蓝牙「看路」组的 road_dx_ideal/road_dy_ideal), 于是**每条路线预设可以带自己的一套**, 不用重编译也不用蓝牙。⚠️**0=这一项不动**(0 是"未标定"的哨兵值, 直接写进去会把那一轴关掉, 所以这里刻意当"不改"); 与蓝牙是同一对量, **谁后写谁生效**; 易失, 断电回 app_config.h 的 ROAD_*
 *   STEP_ROAD_YAW_WAIT                   anI: timeout_ms=超时[ms]   // 看路校准: 发"H"让主摄拟合**黄区直边的图像方向角**(黄区固定在场上/相机固定在车上 ⇒ 方向只随车航向变), **每次跑都对齐到相机真值**: bias = wrap180(原始航向 − err) ⇒ 发布 yaw ≡ err —— **每次校准后全场航向零点 = "放直"方向**(上电摆放误差/累计漂移一并消掉, 首次也不豁免; 不响)。编译期 ROADYAW_APPLY_EN=1(默认) 时 **app_sensors 发布 yaw 统一扣偏置** —— PID/巡线陀螺腿/支点转弯/OLED 全部用修正后的航向(现场不碰蓝牙, 蓝牙只拿来调参数); 首次定标不限幅, 之后每拍限幅 ±ROADYAW_BIAS_MAX_DEG(帧跳变超限=看错了, 拒绝; 校准流程不响)。⚠️ **世界坐标系不受影响**(世界航向记录换算回原始值); 转弯 absolute 档/巡线陀螺腿的 gyro_target 在校准后坐标系里教/排: 路线开头(车摆正之后)先排一块, 再教/排绝对航向; 之后到点各排一块持续对齐。标定: roadyaw_ideal 出厂默认 **+90°**、roadyaw_dir 默认 **−1**(均台架实测: 放直读 +90, 陀螺正漂时读数减小 90→88; 换机位/换边蓝牙改)。⚠️ ang 在 ±90 两侧跳是正常的(mod 180 表示), 看 err; ⚠️ Pi 侧窗(ROADYAW_ROI_*)要罩住一段**直边**; 车须静止(~1-2s/轮, 瞬态失败自动重采最多 3 轮)
 *   STEP_TARGET_ALIGN_WAIT               anI: timeout_ms=超时[ms], idx=对位基准{0=左边靶,1=中间靶,2=右边靶}   // 标靶对位: 发"6"识别(三颗齐才算数) → 取第 idx 颗的横向偏差横移摆正车。⚠️只横移、不做转正; 标定量 TARGET_ALIGN_*; 识别不全→失败音, 越界不响; 都不动车。排在标靶校准之前(粗调用车、细调用臂)
 *   STEP_TARGET_SCAN_WAIT                anI: timeout_ms=超时[ms]   // 靶标识别(仅扫描): 发"6"+等结果+失败音, 结果存 vGetTargetResult, **不动车也不动臂**。流控闭环前的可选预检/锁定; 单独排一块只想看识别结果也用它
 *   STEP_TARGET_CALIB_WAIT               anI: timeout_ms=超时[ms]   // 标靶校准: 识别靶心 → 按二维码第二位选目标色 → Δ=靶心−瞄准点 → 按比例尺微调舵机6(画面X)/舵机3(画面Y), 增量式(连排两步即迭代收敛); 标定量 tg_*(=TARGET_*); 二维码没认出来则整段不做; 超时 0=用蓝牙 tg_calib_timeout_ms
 *   STEP_TARGET_SERVO_WAIT               anI: timeout_ms=超时[ms]   // 标靶流控闭环: 发"7"起流(Pi 小窗跟踪, 0x05 流帧 ~20Hz) → 10Hz P 环微调舵机6/3(标定量共用 tg_*, 节拍见 ts_*) → 收敛/超时/行程超限 → 发"8"停流。终验可紧后排一块标靶校准
 *   STEP_TARGET_SHOOT_WAIT               anI: timeout_ms=超时[ms], keep_res=记录残留偏移{1=是,0=否}, rev=反向执行{1=是,0=否}   // 平移射靶(激光固定在底盘, 全程不动机械臂): **粗调固定主摄、细调优先 pi**(2026-10-18 定案) —— 粗调=发"6"识别(目标色在即可, **永远用主摄**: 格距/瞄准列的尺是主摄像素标定的, pi 的 px/mm 约两倍)按二维码第二位选色→沿排偏差按格换算(÷每格px×**格距mm tc_gap_mm** —— 打边靶走多远的总旋钮, 改160只影响粗调)一次**纵移**; 精调=发"7"起流, 步进位置模式小步纵移闭环(tc_px_to_mm只管每拍步长)修到收敛→发"8"停流。⚠️轴是车体纵移(前后, cx 偏差驱动 —— 靶排沿车体纵向排开)。⚠️**细调优先 pi(CSI)**(它固定在底盘上, 几何稳; 主摄在机械臂上会漂): 由 tc_use_picam 定, CSI 打不开/掉线/看不到靶时 Pi 自己整条退回主摄并在回帧里报是哪台 → 本步按它选 tc_cx_aim_pi 还是 tc_cx_aim; **pi 瞄准列没标(0)时细调自动只用主摄**(= 粗细都主的老方案)。⚠️ 本步前建议排一块 cam=**主摄+pi** 的预热积木(CSI 冷启动 ~3s, 细调起流只等 3s)。keep_res=是 把累计纵移(≈±150mm)记成"路径残留偏移", 离开靶区的世界系平移勾"叠加上次残留偏移"补掉。标定量见蓝牙「平移射靶」组(tc_*); 激光射击紧后排 STEP_LASER_SHOOT。rev=反向(道具换到路另一边、相机朝另一边看时勾): 本步沿排方向整体镜像(tc_sign 本步内取反, 蓝牙标定值不动), 残留记账随实际位移自动跟着翻、消费端"叠加残留"用法不变; 且**全程只用主摄** —— CSI 固定在底盘、不随臂转向另一边的靶排, rev 时细调不再优先 pi、粗调失败也不再拿 CSI 重试(tc_use_picam 蓝牙值不动; 预热积木用「主摄」那档就够)
 *   STEP_LASER_SHOOT                     anI: ms=打开时长[ms]   // 激光射击: 开激光 → 等 ms → 关激光, 一步到位
 *   STEP_TARGET_SHOOT_HOLD_WAIT          (无参)   // 平移射靶**保持**(OLED「射靶测试」页专用): 与「平移射靶」同一套测量/控制律, 但**永不因超时/收敛/闸门退出** —— 只有 OLED 按「停止」能停。粗调+等停稳照跑一遍, 之后每拍重读 tc_cx_aim(瞄准列, swap=1 本车): 在 OLED 上按一格(±5px)→ 偏差出死区 → 车跟到新目标。撞行程闸/noprog 只"停车冻结+打日志"不退出(按一格自恢复)。参数全在蓝牙「平移射靶」组。⚠️ 保持测试**不记**残留偏移(keep_res 恒 0)。⚠️ 排表时把它放最后一步(它不会自己结束)
 *   STEP_BARREL_ALIGN_WAIT               anI: timeout_ms=超时[ms], keep_res=记录残留偏移{1=是,0=否}   // 爆爆桶对位(2026-10-24 加): 发"J"起流(OpenMV 检测黑色爆爆桶 → Pi 纯转发 0x0B 流帧 ~20Hz) → 等停稳 → 每按 bc_ctrl_ms 拍按 cx 偏差小步**纵移**闭环(轴映射与平移射靶同款: 相机装 90°, 图像 cx ↔ 车体纵移)→ 收敛/超时/行程超限/noprog/中止 → 发"K"停流。⚠️ bc_cx_aim=0(未标定)时本步进**只测不改**模式: 起流采 ~3s 报中位 cx("BKT meas:" 行 + 蓝牙遥测 bucket_err)后返回 —— 车摆到理想推桶位姿跑一次, 读数填蓝牙「爆爆桶」组 bc_cx_aim 再跑。keep_res=是 把累计纵移记「路径残留偏移」(推开桶那步的世界系平移勾"叠加残留"吃掉)。⚠️ 本步**不占主摄**(OpenMV 独立串口), 但与巡线流/靶标流共享 COORD 通道 —— Pi 侧起流时自动互斥。对位完成后推桶由紧后排的世界系平移/底盘积木做
 *   STEP_QR_SET_TEST                     anI: code=调试值{111=111,112=112,113=113,121=121,122=122,123=123,131=131,132=132,133=133,211=211,212=212,213=213,221=221,222=222,223=223,231=231,232=232,233=233,311=311,312=312,313=313,321=321,322=322,323=323,331=331,332=332,333=333}   // 测试用: 直接把二维码结果写成这个三位数(不经过 Pi), 没扫码/台架上演练后续步骤。⚠️ 每位只能是 1~3(下拉里已限定), 固件另做防呆; 会覆盖真实扫码结果
 *   STEP_ARM_EDIT_WAIT                   (无参)   // 中途让位手动操纵机械臂: 停车 + 强制开编辑模式 + 把链路钉在 WAITING(不让 Pi 上线把窗口闩掉), 停在这里等 MAIN 键。⚠️不真的断开 Pi; 期间「机械臂」Tab 可点动/录制
 *   STEP_BOMB_GRAB_WAIT                  anI: left=抓左边动作, mid=抓中间动作, right=抓右边动作, reset=抓完复位动作   // 排爆: 按二维码首位(目标色 1红/2绿/3蓝)在已识别的三球里定位 → 播对应的抓取动作预设 → 再播复位预设。⚠️阻塞与否跟随所选动作的「播放方式」(_WAIT 不代表一定阻塞); 数据不全则长鸣不动
 *   STEP_HOSTAGE_RESCUE_WAIT             anI: left=左只动作, mid=中只动作, right=右只动作   // 反恐任务: 按二维码第三位(目标形状 1圆柱/2圆锥/3腰鼓)在已识别的三只人质里定位 → 播那一位的动作预设, 不复位。阻塞与否跟随所选动作的「播放方式」; 数据不全则拒绝不动（不响, 留痕）
 *   STEP_BALL_GRAB_ONE_WAIT              afF: gyro_target=陀螺目标角[°];   anI: timeout_ms=识别超时[ms], mode=角度纠正{0=不纠正,1=视觉yaw转正,2=OpenMV巡线收敛}, grab=抓取动作, place=放置动作<可空>, grab_lead_ms=抓取提前量[ms], place_delay_ms=放置延时[ms], rev=反向执行{1=是,0=否}, single_step=单步确定{1=是,0=否}   // 色球单动作抓取(2026-10 新方案, 与排爆并存): 只录**一个**"抓中间球"的动作 → 识别定位目标(0左/1中/2右) → 中球偏差dx/dy叠加(位置-1)×one_lat_mm平移过去(目标=新"中间球")。⚠️识别失败且Pi给方向(剩两球/缺端球出画面)→开环拉回一步再重扫一次(2026-10-26加, 同ALIGN_PULLBACK; 距离=蓝牙ball_pullback_mm)。抓取动作**异步并行**: 底盘先动, 估时倒计时到「剩 grab_lead_ms(垫底pad)」才发动作 —— 锚定**到位时刻**而非起点, 短行程不白等/长行程不会车未到爪先伸(0=到位后才发), 本步只等它结束; 提前量=动作里伸爪前段的时长(动作属性, 与距离无关, 2026-10-20 由起点后延时改)。放置动作同样异步, 但 2026-10-23 改回**起点后延时** place_delay_ms(从回退起点起算, 0=回退同时放; 回退通常比放置动作短, 用剩提前量首拍即中=没作用), 放置动作播完立即返回、**不再做放置后收敛**。**place 选「无」(0)** = 不投放置动作: 抓完仍**只回退间距分量**到中间对齐位(车停位与残留清零跟老行为一致), 适合决赛「色球只抓不放」的编法。抓完**仅回退间距分量**(中间球=一步不动, 对齐修正保留, 车停在对齐位=爆破桶位)。mode=1需蓝牙ball_yaw_en=1; mode=2=识别前巡线收敛+抓取时并行收敛一次(放置后不收敛; 该区须看得见线), 收敛听谁由 **全局 LINE_CONV_SRC** 决定(档位同「巡线收敛」含融合; 陀螺/融合档咬死 gyro_target 这个绝对世界航向, 越界由内层拒绝; 融合占比 = LINE_CAM_PCT, 识别前收敛与抓取时的并行收敛共用); 分量one_lat_mm/one_lat_dir在蓝牙「单动作抓取」组调。rev=反向(道具换边、相机朝另一边看时勾): 对位/间距/拉回/yaw转正方向整体镜像, 车停对齐位与清残留逻辑不变。**single_step=1**: 分步执行, 每个阶段结束**停车等 MAIN 键**再继续 —— 色球对位→[MAIN]→移动→[MAIN]→抓取→[MAIN]→放到爆爆桶(最后一段仅 place≠无 时存在, place=无则抓取即结束); 勾选后 grab_lead_ms 不再起作用(移动已停稳, 抓取在按 MAIN 后立即发), 放置仍按 place_delay_ms 从回退起点计时; ⚠️ 等键时 OLED 须停在第 1 页(菜单页的 MAIN 归菜单, 同 STEP_DEAD)
 *   STEP_HOSTAGE_GRAB_ONE_WAIT           afF: gyro_target=陀螺目标角[°];   anI: timeout_ms=识别超时[ms], mode=角度纠正{0=不纠正,1=识别前巡线收敛,2=识别前收敛+抓取时并行收敛}, grab=抓取动作, keep_res=记录残留偏移{1=是,0=否}, grab_lead_ms=抓取提前量[ms], rev=反向执行{1=是,0=否}, single_step=单步确定{1=是,0=否}, place=放置动作<可空>, place_delay_ms=放置延时[ms]   // 人质单动作抓取(2026-10 新方案, 与反恐并存): 单个"抓中间人质"动作。抓取动作**异步并行**: 底盘先动, 估时倒计时到「剩 grab_lead_ms(垫底pad)」才发动作(锚定**到位时刻**而非起点, 2026-10-20 由起点后延时改; 提前量=动作里伸爪前段的时长, 与距离无关, 0=到位后才发), 本步只等它结束。mode=2=识别前收敛一次+抓取平移后等臂期间再并行收敛一次(底盘已空闲时与臂并行, 修开环平移的横向残差), 收敛听谁由 **全局 LINE_CONV_SRC** 决定(档位同「巡线收敛」含融合; 陀螺/融合档咬死 gyro_target, 越界由内层拒绝; 融合占比 = LINE_CAM_PCT, 两次收敛共用)。**place=无(0) 时不放置: 不回中不复位**, keep_res=1把平移总位移记为"路径残留偏移", 后面的世界系平移勾"叠加上次残留偏移"一次性吃掉(抓左多走/抓右少走)。**place≠无(放置)**: 抓完等臂空后**只回退间距分量**到中间对齐位(=爆爆桶位), 从回退起点延时 place_delay_ms 投放置动作(0=回退同时放), 播完立即返回; 净位移=对位修正, 结束时**清残留**(此时 keep_res 不记账, 留痕)。⚠️ mode=2的收敛只修横向+航向: 间距加在横向(one_lat_dir=1/2)时横向残留被视觉吸收, 那种编法keep_res=0; 加在纵向(3/4,本车预期档)不受影响可共存。⚠️超时取7500。rev=反向(道具换边、相机朝另一边看时勾): 对位/间距/拉回方向整体镜像; 残留记账随实际位移自动翻, 消费端"叠加残留"不变(同一"抓左"从多走变少走)。**single_step=1**: 分步执行, 每个阶段结束**停车等 MAIN 键**再继续 —— 人质对位→[MAIN]→移动→[MAIN]→抓取; 放置(place≠无)再加→[MAIN]→放到爆爆桶(抓取不再是最后一段); 勾选后 grab_lead_ms 不再起作用(移动已停稳, 抓取在按 MAIN 后立即发), 放置仍按 place_delay_ms 从回退起点计时; ⚠️ 等键时 OLED 须停在第 1 页(菜单页的 MAIN 归菜单, 同 STEP_DEAD)
 *   STEP_TEST_LOOP_WAIT                  anI: step=被测试的积木, rounds=轮数   // 测试: 把选中的积木反复执行, 每轮两短声并等 MAIN 键继续; 轮数 0=不限。参数整条照抄当前路线里同类型的积木(路线里没用过就全 0 = 等于不动); 不能选「测试」自己与「重置脚本」
 *   STEP_TIMER                           anI: cmd=计时动作   // 计时: 控制 OLED 第 1 页第 4 行的秒表; 开始=从当前值继续走, 结束=冻结在当前值, 重置=清零并停止
 *   STEP_DELAY                           anI: ms=时长[ms]
 *   STEP_STOP                            (无参, 停车, 阻塞等 MAIN 按键继续)
 *   STEP_DEAD                            (无参数, 不动车, 等 MAIN 按键继续)
 *   STEP_SCRIPT_RESET                    (无参, 重置脚本: 重启**当前这一帧**所在的表, 跳过表首连续的握手/等键)
 *   STEP_IF                              anI: cond=条件   // 条件分支(容器): 成立落进本行下面, 一路执行到同层的「否则」或「结束如果」; 不成立跳到同层「否则」之后(没有否则就跳到「结束如果」之后)。三块标记靠**位置**配对, 可任意层嵌套, 「否则」可省略。⚠️ 条件选「巡线是否在线上」那一项时会阻塞并跟树莓派对话, 其余条件都是瞬时快照(含「健康检查: ...」五项 —— 读握手时那次体检的结果, 不重新检查)；各条件判据见 emScriptCondTdf
 *   STEP_ELSE                            (无参)   // 「否则」标记(容器): 条件不成立时的分支从这里开始。单独出现无意义, 必须与同层「如果」配对
 *   STEP_END_IF                          (无参)   // 「结束如果」标记(容器): 配对区间到此为止。单独出现无意义
 *   STEP_SCRIPT_CALL                     anI: target=跳转目标   // 无条件跳到该预设, 跑完自动跳回下一行。选项自动取自 emScriptPresetIdTdf
 *   STEP_SCRIPT_RETURN                   (无参)   // 从被跳进来的子预设提前返回, 落在那条 STEP_SCRIPT_CALL 的下一行; 顶层（没被跳转）执行时 = 本轮路线到此为止
 */

/* ==================== 计时器命令（STEP_TIMER 的 cmd / OLED「计时」块的参数） ====================
 * 两个积木库共用这一份: 地图 Tab 的下拉由 app_script.c 的 `(emTimerCmdTdf)I(0)` 派生,
 * OLED Tab 的下拉由 oled_menu.ACT_ARG_ENUM 映射到本枚举类型。
 * ⚠️ 定义在 app_script.h 而不是 app_beam.h: 控制台会把符号写进 map.c, 而 map.c 只经
 *    map.h → app_script.h 取声明, 引不到 app_beam.h。 */
typedef enum {
    TIMER_CMD_START = 0,   /* 开始 */
    TIMER_CMD_STOP,        /* 结束 */
    TIMER_CMD_RESET,       /* 重置 */
} emTimerCmdTdf;

typedef enum {
    /* ---- 电机 ---- */
    STEP_EMM_ENABLE,                      /* 使能/失能底盘电机 1~4 */
    STEP_EMM_CTRL_MODE,                   /* 切换步进电机环模式(底盘电机1~4 开环/闭环; 不写 Flash, 掉电即回原模式) */

    /* ---- 底盘 ---- */
    STEP_CHASSIS_MOVE_NO_WAIT,            /* 速度模式, 发完指令即返回 */
    STEP_CHASSIS_STOP,                    /* 停车（速度 0） */
    STEP_CHASSIS_MOVE_WORLD_WAIT,         /* vChassisMoveWorld(wx, wy) */
    STEP_CHASSIS_MOVE_WORLD_X_WAIT,       /* vChassisMoveWorldX(wx) */
    STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       /* vChassisMoveWorldY(wy) */
    STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT, /* 世界系斜向平移(方向由 dir 槽下拉: 0左上/1右上/2左下/3右下)。⚠️2026-10 由「斜左上/斜右上」两块合并, 原 _DIAG_RF_WAIT 成员已删 —— 表里存的是**符号名**, 老的 RF 行会变成编译错误(响亮失败, 胜过静默指向另一个方向); 车体原语 vChassisMoveWorldDiagLF/RF 仍在 */
    STEP_CHASSIS_SET_SPEED_ACCEL,         /* vChassisSetWorldSpeedAccel(accel, speed) */
    STEP_CHASSIS_SET_ACCEL,               /* vChassisSetAccel(accel) */
    STEP_CROSS_RAMP_WAIT,                 /* 过减速带: 低速顶住坎面被动对齐, 再位置模式大力冲过 */
    STEP_CHASSIS_ACTION_PLAY_WAIT,        /* 播放底盘动作组(时间轴速度混合): 平移/陀螺转弯/臂事件按时间轴叠加连播, 阻塞到整组走完(2026-10 加) */

    /* ---- PID 闭环运动（边走边转系, 依赖 IMU） ---- */
    STEP_PID_MOVE_WAIT,                   /* ucPidMove 通用闭环 */
    STEP_PID_MOVE_AND_TURN_WAIT,          /* 平移+纠偏 → 停下消差 */
    STEP_PID_TURN_WAIT,                   /* CALIBRATE→FINE 两段原地转 */
    STEP_BACK_TO_ROAD_WAIT,               /* 小速度回退同时回正航向 */
    STEP_PIVOT_TURN_WAIT,                 /* 单侧双轮支点转弯(陀螺仪闭环; rev=1 对侧驱动+速度取反, 转向不变) */
    STEP_PIVOT_LINE_WAIT,                 /* 支点转弯→误差进阈值交给巡线(交接后听谁由全局 LINE_CONV_SRC 决定; settle段可对准即走 settle_early) */
    STEP_SPIN_TURN_WAIT,                  /* 四轮绕中心原地自转(陀螺仪闭环) */
    STEP_SPIN_LINE_WAIT,                  /* 原地转弯→误差进阈值交给巡线(支点接管的四轮自转版, 与 PIVOT_LINE 同参) */

    /* ---- 弧线转弯（位置模式写死, 不依赖 IMU） ---- */
    STEP_ARC_TURN_WAIT,                   /* vChassisArcTurnPos(delta, fwd) */
    STEP_ARC_TURN_L90_WAIT,               /* 弧线左转 90° */
    STEP_ARC_TURN_R90_WAIT,               /* 弧线右转 90° */

    /* ---- 机械臂 ---- */
    STEP_ARM_EDIT_WAIT,                   /* 让位手动操纵: 停车+强制编辑模式, 等 MAIN 继续 */
    STEP_ARM_GROUP_WAIT,                  /* 组动作(阻塞至到位/超时) */
    STEP_ARM_GROUP_NO_WAIT,               /* 组动作(启动即返回) */
    STEP_SET_ZHUASHOU_WAIT,               /* 夹爪开合(KAI/HE, 程度), 阻塞等 S 曲线到位 */
    STEP_ARM_PLAY_WAIT,                   /* vArmPlayWait(), 播放已保存的动作预设(整段) */
    STEP_ARM_PLAY_NO_WAIT,                /* vArmPlayNoWait()/vArmPlayNoWaitDelay(), 启动即返回; delay_ms=后台延时到点再投(不阻塞后续步骤) */

    /* ---- 执行器 ---- */
    STEP_LASER_ON,                        /* 打开激光（继电器 JIGUANG） */
    STEP_LASER_OFF,                       /* 关闭激光 */
    STEP_BUGUANG_KAI,   /* 打开补光灯(可勾「自定义亮度」%: 0=不勾=用宏 FILL_LIGHT_ON_DUTY) */
    STEP_BUGUANG_GUAN,   /* 关闭补光灯 */
    STEP_BEEP,          /* 蜂鸣器: 响 ms; 勾「周期」= 响ms→静音gap_ms 循环 times 次 */

    /* ---- 校准（app_cali 空壳） ---- */
    STEP_CALI_WAIT,                       /* 顺序组合两个子校准方法 */

    /* ---- 树莓派 ---- */
    STEP_WAIT_HANDSHAKE,                  /* 等待握手(Pi主动, 无限等待; 已连上后再执行=判为断开重连; 按MAIN可跳过确认窗提前放行) */
    STEP_QR_SCAN_WAIT,   /* 副摄二维码扫描 */
    STEP_QR_SCAN_MAIN_WAIT,   /* 主摄二维码扫描 */
    STEP_QR_SET_RANDOM,                   /* 随机二维码（不经过 Pi） */
    STEP_HOSTAGE_SCAN_WAIT,               /* 人质识别: 发指令+等结果+失败音 */
    STEP_COLOR_SCAN_WAIT,                 /* 色球识别: 发指令+等结果+失败音, 读 vGetColorResult */
    STEP_RPI_SEND_TASK,                   /* 发任务指令帧(ASCII 命令码, 瞬时) */
    STEP_RPI_WARM_CAM,                    /* 预热摄像头(只发不等, 瞬时) */
    STEP_LINE_CONVERGE_WAIT,              /* 巡线收敛闭环: 发LINE_START+读Pi steer横移+发LINE_STOP一步到位(航向源=全局 LINE_CONV_SRC, 融合两环输出按 LINE_CAM_PCT 加权; early_ret=达标提前返回) */
    STEP_LINE_STRAIGHT_WAIT,              /* 巡线直走(沿线边前进边修走固定距离, mm 按位置模式同款估时×时间系数反推时长, 速度/加速度可调, 可吃残留偏移, settle_ms=起步先对准) */
    STEP_BALL_ALIGN_WAIT,                 /* 色球区对位(多方案链): 按槽序试, 第一个成功即停; 三球中球/矩形板心/缺球开环拉回 */
    STEP_HOSTAGE_ALIGN_WAIT,              /* 人质对位: 识别→按参数选定的基准人质偏差横/纵移到位 */
    STEP_ROAD_ALIGN_WAIT,                 /* 看路对位: 主摄量黄色区左边缘→车体纵移一步, 可选再用下边缘修横向(开环, 端口令“G”; rev=1 反向看路, 端口令“M”) */
    STEP_ROAD_SET_IDEAL,                  /* 看路理想值: Δx/Δy 写进运行时量(0=该项不动, 同蓝牙 road_dx_ideal/road_dy_ideal) */
    STEP_ROAD_YAW_WAIT,                   /* 看路校准: 拟合黄区直边图像方向角, 对齐到相机真值(每次跑都把航向零点对到“放直”, 端口令“H”) */
    /* ---- 工具 ---- */
    STEP_DELAY,                           /* vTaskDelay(ms) 让出式延时 */
    STEP_STOP,                            /* 停车, 阻塞等 MAIN 按键继续 */
    STEP_DEAD,   /* 按主按键开始 */
    STEP_SCRIPT_RESET,                    /* 重置脚本: 重启**当前这一帧**所在的表(不弹栈, 深度0时 = 回到本轮路线第一条实质步骤) */

    /* ---- 流程控制（嵌套分支 / 跳到别的预设 / 重置） ---- */
    STEP_IF,                              /* 如果: 条件不成立走否则分支 */
    STEP_ELSE,                            /* 否则: 条件不成立时从这里继续 */
    STEP_END_IF,                          /* 结束如果 */
    STEP_SCRIPT_CALL,                     /* 无条件跳到目标预设, 跑完自动跳回下一行 */
    STEP_SCRIPT_RETURN,                   /* 从被跳进来的子预设提前返回 */

    /* ---- 排爆（排爆区抓取） ---- */
    STEP_BOMB_GRAB_WAIT,                  /* 排爆: 按二维码首位选球 → 播对应动作预设 → 复位 */
    STEP_BALL_GRAB_ONE_WAIT,              /* 色球单动作抓取: 只录“抓中间球”一个动作, 其余目标平移过来抓(抓完反向回中再放置) */

    /* ---- 反恐任务（人质区救援） ---- */
    STEP_HOSTAGE_RESCUE_WAIT,             /* 反恐任务: 按二维码第三位选人质形状 → 播对应动作预设（**不复位**） */
    STEP_HOSTAGE_GRAB_ONE_WAIT,           /* 人质单动作抓取: 单个“抓中间”动作+平移定位; 默认不回中（残留交给“叠加残留”), 选了放置动作则回中再放 */

    /* ---- 打靶（反恐区） ---- */
    STEP_TARGET_ALIGN_WAIT,               /* 标靶对位: 识别(三颗齐)→取基准靶横向偏差横移摆正车 */
    STEP_TARGET_SCAN_WAIT,                /* 靶标识别(仅扫描): 发“6”+等结果+失败音, 预检/锁定用 */
    STEP_TARGET_CALIB_WAIT,               /* 标靶校准: 识别靶心 + 按比例尺微调瞄准轴（开环迭代） */
    STEP_TARGET_SERVO_WAIT,               /* 标靶流控闭环: 起流→10Hz P环微调舵机6/3→收敛停流 */
    STEP_TARGET_SHOOT_WAIT,               /* 平移射靶(激光固定底盘): 沿靶排纵移粗调+流帧位置模式闭环, 不动机械臂 */
    STEP_TARGET_SHOOT_HOLD_WAIT,          /* 射靶保持(OLED测试用): 同上但**永不结束**, 调瞄准列就跟到新目标 */

    /* ---- 爆爆桶（看桶对位, 2026-10-24 加） ---- */
    STEP_BARREL_ALIGN_WAIT,               /* 爆爆桶对位: 发“J”起流→0x0B 流帧纵移闭环(未标定=只测模式读数), 命令 “J”/“K” */
    STEP_LASER_SHOOT,                     /* 激光射击: 开激光, 等一段时间后自动关 */
    STEP_QR_SET_TEST,                     /* 调试二维码（测试用, 每位 1~3） */

    /* ---- 测试 ---- */
    STEP_TEST_LOOP_WAIT,                  /* 反复执行选中的积木, 每轮等 MAIN 继续 */

    /* ---- OpenMV 附件 ---- */
    STEP_OPENMV_LED,                      /* OpenMV 补光亮度(0~100, 只发不等; 与车上补光灯是两盏灯) */
    STEP_OPENMV_LED_TOGGLE,               /* OpenMV 补光临时开关(亮↔灭, 只发不等; 灭是临时的) */

    /* ---- 机械臂（运行时调节, 2026-10 加） ---- */
    STEP_ARM_SPEED_SET,                   /* 中途设置舵机速度(°/s, 瞬时): 写 arm_speed_dps 并当场下发 */
    STEP_ARM_DEPLOY_MARK,                 /* 改写「机械臂已展开」标记(瞬时): 置0=声明臂已折回(下次任务重做待命展开), 置1=声明已展开 */

    /* ---- 计时 ---- */
    STEP_TIMER,                           /* 计时(开始/结束/重置状态页秒表; 瞬时) */

    STEP_COUNT                            /* 哨兵, 非实际步骤 */
} emScriptStepTypeTdf;

/* ==================== IF 条件判据（STEP_IF 的 cond 参数） ====================
 * 加一个条件 = 加一个成员 + 在 app_script.c 的 s_astScriptCond[] 加一行 ——
 * 控制台「地图」Tab 的下拉选项自动来自本枚举的成员尾注释, 控制台侧一行都不用改。
 *
 * ⚠️ 不要加 _COUNT/_NUM 哨兵成员: server.py::parse_enums 把枚举的**每个**成员都收成
 *    一个真下拉选项, 哨兵会变成一个能选中、选中又没意义的条目。个数用 app_script.c
 *    里的 SCRIPT_COND_NUM 取(那张表是文件内 static, 头文件里 sizeof 不到)。
 *
 * 两种语义混在同一张表里, 挑条件时要知道自己在挑哪种:
 *   ·「快照」型（QR / HOSTAGE / COLOR）: 只读**最近一次**结果, 瞬时、不阻塞、无副作用。
 *      读的是上一次扫描留下的值, 所以 IF 要排在对应的扫描步骤紧后面, 否则读到的是旧战况。
 *   ·「主动探测」型（LINE）: **会阻塞 LINE_PROBE_MS 并跟树莓派对话**
 *      （发 LINE_START → 等窗口 → 发 LINE_STOP）, 期间不动底盘。
 * ⚠️ 所以 STEP_IF **不保证是瞬时的**。
 *
 * 「自定义开关」条件（2026-10-30 加）: 控制台「自定义」Tab 管理 —— 用户在控制台上新增
 * 一个开关 = 本枚举尾部「生成块」里多一个成员 + app_script.c 条件注册表生成块里多一行
 * （读 app_config.h「自定义开关」段的 SW_* 宏）。状态是**编译期**的: 控制台上切换开关
 * 只改宏值, 要 Keil 重编译烧录才上车。id 从 SCRIPT_COND_SW_BASE(100) 起、只增不重用
 * （删掉再新建拿新号 —— 复用旧号会让 map.c 里残留的 SCRIPT_COND_SW_n 无声指向别的开关）。 */
typedef enum {
    SCRIPT_COND_NONE    = 0,   /* 无条件（恒成立） */
    SCRIPT_COND_QR      = 1,   /* 二维码结果（上次扫描） */
    SCRIPT_COND_HOSTAGE = 2,   /* 人质识别结果（上次扫描） */
    SCRIPT_COND_COLOR   = 3,   /* 色球识别结果（上次扫描） */
    SCRIPT_COND_LINE    = 4,   /* 巡线是否在线上（主动探测，会阻塞） */

    /* 相机健康检查结果（2026-11 加）: 读 ucRpiHealthCheck 写在 Beam 的四项结果,
     * **不阻塞、不主动检查** —— 只是「上一次握手体检」的快照。没勾健康检查时四项
     * 恒为"未检查"(2), 所以这些条件全部不成立（路线照常往下走, 与不读取时行为一致）。
     * 判据: 「有失败」= 任一项 0; 「主摄失败」等 = 该项 0（未检查不算失败）。 */
    SCRIPT_COND_HEALTH_FAIL    = 5,   /* 健康检查: 任一项失败 */
    SCRIPT_COND_HEALTH_M_FAIL  = 6,   /* 健康检查: 主摄(M)失败 */
    SCRIPT_COND_HEALTH_Q_FAIL  = 7,   /* 健康检查: 扫码副摄(Q)失败 */
    SCRIPT_COND_HEALTH_PI_FAIL = 8,   /* 健康检查: 树莓派CSI(Pi)失败 */
    SCRIPT_COND_HEALTH_MV_FAIL = 9,   /* 健康检查: OpenMV(MV)失败 */

    /* ⚠️ 下面这对**单行**标记之间由「自定义」Tab 整段重写（手改会被覆盖）:
     * 每成员 = 一个编译期开关, 成员值 = id（100 起）, 尾注释 = 条件下拉的标签 = 中文名;
     * 求值注册在 app_script.c 的 s_astScriptCond[] 生成块（同名标记）。 */
    /* ==== 自定义条件生成块: 开始 ==== */
    SCRIPT_COND_SW_108 = 108,   /* 自定义:色球反向 */
    SCRIPT_COND_SW_109 = 109,   /* 自定义:标靶反向 */
    SCRIPT_COND_SW_110 = 110,   /* 自定义:人质反向 */
    SCRIPT_COND_SW_111 = 111,   /* 自定义:左边封路 */
    SCRIPT_COND_SW_113 = 113,   /* 自定义:色球与人质交换 */
    SCRIPT_COND_SW_114 = 114,   /* 自定义:二维码板与人质台交换 */
    SCRIPT_COND_SW_115 = 115,   /* 自定义:人质放置到爆爆桶+抓取色球 */
    /* ==== 自定义条件生成块: 结束 ==== */
} emScriptCondTdf;

/* ==================== 步骤表条目结构体 ==================== */

#define SCRIPT_MAX_FLOAT_PARAMS  6   /* 与 pro 保持一致, 便于后续移植更多步骤类型 */
#define SCRIPT_MAX_INT_PARAMS    9   /* 2026-10-08: 8→9, 支点转弯追加「融合相机占比」尾槽（F2I9）——
                                         * 巡线收敛/直走/抓取那些追加的 cam_pct 都在 8 个 int 以内。
                                         * 2026-10-30: src/cam_pct 移除后当前最大是 7（F2I7）;
                                         * 2026-11: 人质单动作抓取追加 place/place_delay 后**顶到 9**
                                         * （F1I9）—— 再加参数得先扩这个数组 */

/* 脚本帧栈的格数。**不是递归** —— 帧栈是 vScriptRun 里的静态数组, 而脚本任务的栈只有
 * 512 字（Core/Src/freertos.c）, 真递归会把 chassis→EMM 那条调用链吃穿。
 * ⚠️ 第 0 格是**顶层路线**（g_pstScriptActive）, 所以真正能嵌套的跳转层数是
 *    SCRIPT_CALL_DEPTH − 1 = **3 层**, 不是 4 层。
 * 到顶的跳转会被**拒绝**、按「继续」落下去（不是顶掉最老的帧）, 不会越界。
 * ⚠️ 这只挡**嵌套深度**, 不挡**总工作量**: 互相调用（A→B→A）在深度内是合法程序,
 *    步数按乘积放大 —— 路线照样跑得完, 只是耗时远超预期。见 CLAUDE.md。 */
#define SCRIPT_CALL_DEPTH        4

typedef struct {
    emScriptStepTypeTdf eType;                        /* 步骤类型 */
    const char         *pcName;                       /* 步骤名（调试用） */
    float               afF[SCRIPT_MAX_FLOAT_PARAMS];  /* float 参数序列, 未用补 0 */
    int32_t             anI[SCRIPT_MAX_INT_PARAMS];    /* int 参数序列, 未用补 0 */
} stScriptStepTdf;

/* ==================== 表条目初始化宏 ====================
 * 免去手写参数个数与占位 0：按"几个 float + 几个 int"选宏（如 _F1I2 = 1 float + 2 int）,
 * 参数类型由宏约束（float/int 位置写反编译器会警告）。
 * 特殊布局仍可直接写结构体字面量（C90 嵌套花括号初始化）。
 * 新步骤类型参数更多时按需追加宏, 不提前堆全。
 * ⚠️⚠️ **控制台（tools/webconsole/map_codec._serialize_step）按"每个家族的参数个数上限"选宏名,
 *    上限写死在那边（I≤6 / F≤5 / F1I≤5 / F2I≤7）。所以每个家族的 **1..上限 必须一个不缺** ——
 *    缺一个（比如当初没有 _I4 而有 4 个 int 参数的积木）就会编出
 *    `SCRIPT_STEP_Ixxx(...过多实参...)`, 直到 Keil 编译才炸 `#55-D`（真踩过两次）。
 *    `test_server.py::TestSerializeStep::test_every_emittable_macro_exists` 盯着这条。 */
#define SCRIPT_STEP(type, name)                { (type), (name), {0}, {0} }
#define SCRIPT_STEP_F1(type, name, f0)         { (type), (name), {(f0)}, {0} }
#define SCRIPT_STEP_F2(type, name, f0, f1)     { (type), (name), {(f0), (f1)}, {0} }
#define SCRIPT_STEP_F3(type, name, f0, f1, f2) { (type), (name), {(f0),(f1),(f2)}, {0} }
#define SCRIPT_STEP_F4(type, name, f0, f1, f2, f3) \
                                               { (type), (name), {(f0),(f1),(f2),(f3)}, {0} }
#define SCRIPT_STEP_F5(type, name, f0, f1, f2, f3, f4) \
                                               { (type), (name), {(f0),(f1),(f2),(f3),(f4)}, {0} }
#define SCRIPT_STEP_I1(type, name, i0)         { (type), (name), {0}, {(i0)} }
#define SCRIPT_STEP_I2(type, name, i0, i1)     { (type), (name), {0}, {(i0), (i1)} }
#define SCRIPT_STEP_I3(type, name, i0, i1, i2) { (type), (name), {0}, {(i0), (i1), (i2)} }
#define SCRIPT_STEP_I4(type, name, i0, i1, i2, i3) { (type), (name), {0}, {(i0), (i1), (i2), (i3)} }
/* 2026-10 加 I5/I6（I 族顶到 6）: 单动作抓取的「异步臂」两个参数
 * （STEP_BALL_GRAB_ONE_WAIT 的 grab_lead_ms / place_delay_ms、
 * STEP_HOSTAGE_GRAB_ONE_WAIT 的 grab_lead_ms; 抓取=**到位前提前量**(2026-10-20 改),
 * 放置=**回退起点后延时**(2026-10-23 改回旧语义); 0=按各自语义: 抓取到位后才发、
 * 放置回退同时放）。⚠️ 参数**追加在 int 末尾**, 老行（I4）缺的尾槽补 0, map.c 不必重写。 */
#define SCRIPT_STEP_I5(type, name, i0, i1, i2, i3, i4) \
                                               { (type), (name), {0}, {(i0), (i1), (i2), (i3), (i4)} }
#define SCRIPT_STEP_I6(type, name, i0, i1, i2, i3, i4, i5) \
                                               { (type), (name), {0}, {(i0), (i1), (i2), (i3), (i4), (i5)} }
#define SCRIPT_STEP_F1I1(type, name, f0, i0)   { (type), (name), {(f0)}, {(i0)} }
#define SCRIPT_STEP_F1I2(type, name, f0, i0, i1) \
                                               { (type), (name), {(f0)}, {(i0), (i1)} }
#define SCRIPT_STEP_F1I3(type, name, f0, i0, i1, i2) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2)} }
#define SCRIPT_STEP_F1I4(type, name, f0, i0, i1, i2, i3) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3)} }
/* ⚠️ 5 个 int 是 F1I 族**曾经最多的一个**（另有 F2I 那一族更宽, 见下）: 加它时同时抬了
 *    map_codec.MACRO_CAP_F1I 4→5。再往上加要两边一起动。 */
#define SCRIPT_STEP_F1I5(type, name, f0, i0, i1, i2, i3, i4) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3), (i4)} }
/* ---- F1I6 / F1I7（2026-10-07 加, 单动作抓取的「收敛航向来源」拆分）----
 * BALL_GRAB_ONE(6 int→7) 与 HOSTAGE_GRAB_ONE(5 int→6) 各加一个 float 槽 gyro_target
 * (src=陀螺时的咬死目标角) + int 尾槽 src —— float 槽插在 int 之前 = 老行全量重写
 * (map.c 5 行已按 F1I7/F1I6 重写, 缺的槽位填 0=相机=老行为)。
 * ⚠️ 2026-10-30 src/cam_pct 从积木移除 → 现在 BALL=F1I8(…,rev,single_step)、
 *    HOSTAGE=F1I9(…,single_step,place,place_delay, 2026-11 起); F1I6/F1I7 仅作能力储备。 */
#define SCRIPT_STEP_F1I6(type, name, f0, i0, i1, i2, i3, i4, i5) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3), (i4), (i5)} }
#define SCRIPT_STEP_F1I7(type, name, f0, i0, i1, i2, i3, i4, i5, i6) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6)} }
/* ---- F1I8（2026-10-08 加, 单动作抓取的「融合相机占比」尾槽）----
 * BALL_GRAB_ONE 追加 int 尾槽 cam_pct（src=融合时的相机占比 %, 追加在末尾 = 老行
 * 缺槽补 0; src∈{0,1,2} 时该槽不参与, map.c 那些行一个字节不用改）。
 * ⚠️ 2026-10-30 cam_pct 从积木移除 → 现在 F1I8 由 BALL_GRAB_ONE 使用（8 个 int:
 *    timeout/mode/grab/place/grab_lead/place_delay/rev/single_step）。 */
#define SCRIPT_STEP_F1I8(type, name, f0, i0, i1, i2, i3, i4, i5, i6, i7) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6), (i7)} }
/* ---- F1I9（2026-10-30 加, 场地换边的「反向执行」尾槽）----
 * BALL_GRAB_ONE 追加 int 尾槽 rev（勾选=反向: 道具换到路的另一边、相机/臂朝另一边看时,
 * 本步方向整体镜像 —— 对位符号/间距分量/拉回/视觉yaw转正取反, 见 docs/grab-one.md）。
 * 追加在末尾 = 老行缺槽补 0 = 正向 = 原行为, map.c 一个字节不用改。
 * ⚠️ 2026-10-30 src/cam_pct 移除后 BALL 是 F1I8(…,rev,single_step);
 *    2026-11 人质单动作抓取追加 place/place_delay → HOSTAGE 顶到 F1I9。
 * ⚠️ SCRIPT_MAX_INT_PARAMS(9) 顶格 —— 这是 int 槽的天花板, 再加参数得先扩数组。 */
#define SCRIPT_STEP_F1I9(type, name, f0, i0, i1, i2, i3, i4, i5, i6, i7, i8) \
                                               { (type), (name), {(f0)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6), (i7), (i8)} }
/* ---- F2I 族（**2 个 float + n 个 int**）----
 * 2026-09 为 STEP_PIVOT_LINE_WAIT 新增: 它要「转角/绝对航向」(float) **加**「交接后若听陀螺的
 * 那个目标角」(float), 再加 5 个原有 int 与新增的 `src` —— 1 个 float 的 F1I 族装不下。
 * ⚠️⚠️ 老规矩: 本族 1..6 一个都不能缺（控制台按个数选宏名, 缺一个就编出实参过多的行,
 *     只在 Keil 才炸 #55-D）。抬个数要同步 map_codec.MACRO_CAP_F2I。
 * ⚠️ 控制台按「**先所有 float、后所有 int**」的位置读参数, 所以往**已有积木**插 float 槽会让
 *    老行整体错位（老行的第一个 int 被当成新 float 读）—— 用 F2I 的新积木必须把 map.c 里
 *    那些老行一并重写, 别只加宏。 */
#define SCRIPT_STEP_F2I1(type, name, f0, f1, i0) \
                                               { (type), (name), {(f0), (f1)}, {(i0)} }
#define SCRIPT_STEP_F2I2(type, name, f0, f1, i0, i1) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1)} }
#define SCRIPT_STEP_F2I3(type, name, f0, f1, i0, i1, i2) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2)} }
#define SCRIPT_STEP_F2I4(type, name, f0, f1, i0, i1, i2, i3) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3)} }
#define SCRIPT_STEP_F2I5(type, name, f0, f1, i0, i1, i2, i3, i4) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3), (i4)} }
/* ⚠️ 7 个 int 是**全工程最宽的一条**（STEP_PIVOT_LINE_WAIT / STEP_SPIN_LINE_WAIT, F2I7）:
 *    抬了 map_codec.MACRO_CAP_F2I 与 F2I 整族。再加参数就接着抬这一族, 别去动 F1I。 */
#define SCRIPT_STEP_F2I6(type, name, f0, f1, i0, i1, i2, i3, i4, i5) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3), (i4), (i5)} }
/* 2026-10 加尾段盲走（`tail_ms`）→ 7 个 int, 仍是 STEP_PIVOT_LINE_WAIT 顶着上限。
 * ⚠️ 2026-10-30 src/cam_pct 从积木移除后, 支点/自转接管就是本宏:
 *    dir, absolute, fwd_ms, settle_ms, line_ms, tail_ms, settle_early。
 * ⚠️ 新槽是**追加在 int 末尾**的: 老行（F2I6）的两个 float 与六个 int **位置一个都没动**,
 *    所以 map.c 里那几行不必重写 —— 控制台读老行时把缺的尾槽补成 `0`。
 *    这与设计规则第 8 条说的"插 float 槽必须重写老行"是两回事: 那条说的是**位置错位**。 */
#define SCRIPT_STEP_F2I7(type, name, f0, f1, i0, i1, i2, i3, i4, i5, i6) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6)} }
/* 2026-10 加 settle 段「启动即返回」（`settle_early`）→ 8 个 int, 仍是 STEP_PIVOT_LINE_WAIT
 * 顶着上限（map_codec.MACRO_CAP_F2I 已同步抬到 8）。同 tail_ms 一样**追加在 int 末尾**:
 * 老行（F2I7）一个字节都不用改, 缺的尾槽补 `0` = 否 = settle 段跑满（老行为）。
 * ⚠️ 2026-10-30 src 移除后本宏当前没有使用者, 仅作能力储备。 */
#define SCRIPT_STEP_F2I8(type, name, f0, f1, i0, i1, i2, i3, i4, i5, i6, i7) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6), (i7)} }
/* 2026-10-08 加融合相机占比（`cam_pct`）→ 9 个 int, 仍是 STEP_PIVOT_LINE_WAIT 顶着上限
 * （map_codec.MACRO_CAP_F2I 已同步抬到 9）。同 settle_early 一样**追加在 int 末尾**:
 * 老行（F2I8）一个字节都不用改, 缺的尾槽补 `0`; src∈{0,1,2} 时该槽不参与。
 * ⚠️ 2026-10-30 src/cam_pct 移除后本宏当前没有使用者, 仅作能力储备。 */
#define SCRIPT_STEP_F2I9(type, name, f0, f1, i0, i1, i2, i3, i4, i5, i6, i7, i8) \
                                               { (type), (name), {(f0), (f1)}, {(i0), (i1), (i2), (i3), (i4), (i5), (i6), (i7), (i8)} }

/* ==================== 公开 API ==================== */

/* 逐步骤执行脚本表，分发到对应底层阻塞原语（仅任务上下文调用） */
void vScriptRun(const stScriptStepTdf *pstTable, uint16_t usCount);

/* 等 MAIN 键继续 / 中止位（2026-11 加, 单动作抓取「单步确定」的阶段边界用）。
 * 1 = 操作员按了 MAIN; 0 = 脚本被中止（菜单请求/停）。读后清零 —— 一个按键只放行一段。
 * 仅任务上下文调用（内部 vTaskDelay 让出）。 */
uint8_t ucScriptWaitKeyConfirm(void);

/* 「巡线测试」那条预设（SCRIPT_LINE_TEST_PRESET）是不是**正在跑** —— OLED 第 4 页的
 * 开关标签用它取设备真值。⚠️ 不能只看 `script_running`（那对任何路线都是 1），
 * 否则标签会卡在「停止巡线测试」而实际跑的是别的（2026-09 踩过）。详见 .c 注释。 */
uint8_t ucScriptLineTestActive(void);

/* 通用版：**某个预设**是不是正在当顶层路线跑（2026-10-09 加, 从上面那个抽出来的壳）。
 * OLED 的「射靶测试」开关用它取设备真值 —— 预设 id 由菜单表项的 ucArg 带过来,
 * 所以 app_oled.c 不必硬编码任何预设号。⚠️ 判据同上面那条, 别只看 script_running。 */
uint8_t ucScriptPresetActive(uint8_t ucPresetId);

/* FreeRTOS 任务入口：等 MAIN 键启动 → 跑路线表（map.c）→ 回等待态 */
void vScriptTask(void *argument);

/* 路线表已抽到 map.c/map.h：多套预设 g_astScriptPresetN[] + g_pstScriptActive 指针,
 * 跑哪套由 map.c 末尾的 #define SCRIPT_PRESET_ACTIVE 决定（控制台可切换） */

#endif /* __APP_SCRIPT_H */
