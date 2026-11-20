# config.py — GKS 工科赛 全局参数
import json
import math
import os

# 项目根目录（自动定位，避免硬编码路径漂移）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = BASE_DIR

# systemd 服务名（控制台通过它启停 main.py、开关机自启、查看日志）
SERVICE_NAME = "gks.service"

# 日志：所有 print() / logging 输出写入 logs/YYYYMMDD_HHMMSS_log.txt
LOG_DIR = os.path.join(BASE_DIR, "logs")
ENABLE_LOGGING = True

# 空闲主循环心跳日志间隔（秒）—— main.py 尚无任务，靠心跳证明存活
HEARTBEAT_INTERVAL = 30

# 拍照回执超时[秒]：命令 '9' 发出后, 等 OpenMV 回写 "P,<ok>" 的上限。
# 到点还没等到就回一条"失败"给单片机 —— 否则相机没上电 / 跑的是没实现 'P' 的旧固件时,
# 单片机那边是一片沉默, 现场查不出来（"按了没反应"最难查的那种）。
# 正常路径 OpenMV 存盘只要几十~几百 ms（QVGA JPEG 压缩），3s 很宽裕。
PHOTO_ACK_TIMEOUT_S = 3.0

# ==================== 摄像头参数 ====================
# 设备节点（V4L2 符号链接，与 GCS/cv_python 完全一致，由 udev 规则创建）
CAM_MAIN_INDEX = "/dev/camera_main"    # 主摄像头 —— 物块检测/色环/码垛等
CAM_QR_INDEX = "/dev/camera_Qr"        # 二维码扫码副摄像头

# V4L2 FOURCC 像素格式:
#   "MJPG" = MJPEG 压缩，带宽低，兼容性略差
#   "YUYV" = 原始格式，兼容性最好，带宽较高
#   花屏时尝试互换，空字符串 "" 表示使用摄像头默认格式
CAM_MAIN_FOURCC = "MJPG"
CAM_QR_FOURCC = "MJPG"

# 曝光（每次打开相机时应用一次，见 camera.py::CameraManager._apply_exposure）
#   *_AUTO_EXPOSURE: 3 = 自动（光圈优先，默认，跟不设一样）/ 1 = 手动
#   *_EXPOSURE:      手动曝光值，单位 100µs（100 = 10ms）。只在手动模式下生效。
# 这颗 UVC 相机**没有 gain 控件**，亮度只能靠曝光值压。
# 分三档（对应 camera.open_camera(profile=...)）：
#   CAM_MAIN_* —— 色球 / 靶标（主摄）
#   CAM_QR_*   —— 扫码相机
#   HOSTAGE_*  —— 人质识别**专用**，与 CAM_MAIN_* 互相独立：改它不动色球/靶标，
#                 反之亦然（想一致就把两处设成同样的值）
# ⚠ 色球与靶标仍共用主摄那一档：改 CAM_MAIN_EXPOSURE 会同时压低两者亮度，
#   跟着要重标 COLOR_VAL_MIN / TARGET_VAL_MIN 这类按亮度阈值的项。
CAM_MAIN_AUTO_EXPOSURE = 3
CAM_MAIN_EXPOSURE = 166
CAM_QR_AUTO_EXPOSURE = 3
CAM_QR_EXPOSURE = 166
HOSTAGE_AUTO_EXPOSURE = 1
HOSTAGE_EXPOSURE = 350
# 看路（指令 "G"）专用档：看的是**地面**，与人质/色球/靶标都不同。
# ⚠️ 补光灯会把黄胶带打得近乎过曝（那时黄色掩膜整幅空、自动退 Canny 兜底），
#    所以这一档大概率要往**小曝光**方向调，别照抄 166。
ROAD_AUTO_EXPOSURE = 3
ROAD_EXPOSURE = 166

# 白平衡（每次打开相机时应用一次，见 camera.py::CameraManager._apply_white_balance）
#   *_AUTO_WB:        1 = 自动（驱动默认）/ 0 = 手动固定，用 *_WB_TEMPERATURE
#   *_WB_TEMPERATURE: 固定色温[K]（本相机量程 2800~6500、步进 10，v4l2-ctl 实测）
# ⚠️ 每个档位都**显式设**，哪怕值就是驱动的默认值：V4L2 控件状态**跨 open 保留** ——
#    看路把 AWB 关掉之后，下一个开主摄的任务（色球/靶标/网页预览）不设就继承了"关"。
# ⚠️ 自动档下色温控件是 inactive，写了被驱动忽略 —— 所以只在 0 档写它。
CAM_MAIN_AUTO_WB = 1
CAM_MAIN_WB_TEMPERATURE = 6500
CAM_QR_AUTO_WB = 1
CAM_QR_WB_TEMPERATURE = 6500
HOSTAGE_AUTO_WB = 1
HOSTAGE_WB_TEMPERATURE = 6500
# 看路专用：**固定 6500K**（AWB 关）。
# 为什么只有它固定：AWB **每次开相机都从零重新收敛、要 ~2s / ~40 帧**（实测 R−B：
#   第 1 帧 −47 → 第 15 帧 −14.5 → 第 40 帧才到 +6.3，之后才稳）。而看路是
#   「开相机 → 预热 15 帧(~0.9s) → 立刻采 9 帧」—— 量到的正是收敛过程中的某一瞬，
#   Δx 因此随"这些帧落在收敛曲线的哪一段"漂移。固定档**第 1 帧**就稳
#   （同一机位连测三次 6.5 / 7.1 / 7.3，且一直平到第 90 帧）。
# 为什么是 6500：本机位 AWB 收敛值就在 6500（收敛后 R−B 7.5~7.8，固定 6500 是 7.1）
#   —— 取它 = 画面与"AWB 收敛后"一致，**换 WB 后 Y−B 分布只平移几点、谷底自适应会跟上**。
# ⚠️ 换场地/换灯后要重标，顺序：先调 *_WB_TEMPERATURE（画面冷暖对不对）→ 再调
#    但**固定 WB 仍是纪律**：白平衡一变，"什么算黄"跟着变，现场帧和标定就两套口径。
ROAD_AUTO_WB = 0
ROAD_WB_TEMPERATURE = 6500

# 摄像头生命周期: "on_demand"=按需开关, "persistent"=常驻复用
# ⚠️ 至今仍是**占位**: 全仓库没有任何地方读它。实际的「复用」由 camera_lease.py
#    的租约机制实现（单片机发 "B"/"C" 预热指令时生效），不受本项控制。
CAMERA_LIFECYCLE = "on_demand"

# ==================== 相机租约（预热/复用，见 camera_lease.py）====================
# 空闲看门狗: 租约持有相机但这么久没人用就自动释放。
# ⚠️ 这不是省电的旋钮, 是**安全阀** —— 单片机发了预热却没跟进识别时, 没有它
#    相机就被永久占住、Web 预览打不开, 而且没有任何地方会报错。
# 取值要盖住「预热指令 → 真正用到」之间最长的一段路线（当前最长是预热主摄后
# 走 1000mm + 巡线收敛）。设小了只是省不到时间（会退回现开现扫）, 不会坏。
# ⚠️ 2026-09 起它同时是**每次使用的窗口**: 预热指令可以带个次数（"C,2" = 这台相机
#    接下来要被 2 个任务消费），实际窗口 = 次数 × 本值（camera_lease.idle_limit_s）。
#    倍数在**声明的次数用完之前一直保持**（用完才回到 1 倍）—— 所以两个间隔都拿到
#    同样的宽限，而不是越用越窄。次数不带 / 填 1 → 就是本值，与加这个参数之前一字不差。
LEASE_IDLE_TIMEOUT_S = 15.0

# 上面那个「次数」的上限（相机预热指令 "B,2"/"C,2" 里那个数）。
# ⚠️ 存在的唯一理由是**安全阀**: 窗口 = 次数 × LEASE_IDLE_TIMEOUT_S, 一个手滑的
#    "C,99" 会把相机钉住 25 分钟 —— 而主摄同时还是网页预览要抢的那台, 期间预览
#    一直打不开。夹住时会打一条 WARN（camera_lease._clamp_uses）。
LEASE_USES_MAX = 10

# acquire 等待「同一台相机正在预热」落地的上限。不等的话两边会同时开同一台
# 设备, 后到的那个吃 EBUSY（同 handler._cmd_aim_start 的 old.join 那个坑）。
# ⚠️ 不能贴着实测的开相机耗时（2026-09-26 台架实测冷开 ≈1.5~2s, 不是注释里
#    以前写的 1.3s）: 留不够的话, Pi 一忙（YOLO 首次加载 / 日志 IO）预热线程
#    就会超时, acquire 返回 None → 调用方兜底 open_camera **与预热线程同时开同一台
#    设备** → EBUSY。那正是「随机打开失败」的第二个来源。留一倍余量。
# ⚠️ 改**本项**（以及 LEASE_IDLE_TIMEOUT_S）之后要 `sudo systemctl restart gks.service`
#    才生效: 消费它们的 camera_lease 跑在 **main.py 那个进程**里, 而 Web「设置参数」
#    页由**另一个进程** GKS_Web 提供 —— 页面上的 save_override 只改到 GKS_Web 自己那份
#    config 副本 + config_override.json, main.py 要重启才会重新加载。
#    （页面改完立刻生效的是别的参数: 那些由 GKS_Web 自己用, 与 main.py 无关。）
LEASE_WARM_WAIT_S = 4.0

# 租约「帧新鲜度」阈值（秒）：最新帧距今超过它 = 读线程已停摆（USB 重置 / 设备
# wedge / 线程死亡），租约判为**不健康**——LeaseHandle.read() 停发陈帧、warm/acquire
# 不再命中它、看门狗立刻回收。治的是「断电重跑后识别结果与上次一字不差 / 检出 0 个
# 目标、重启 main.py 才恢复」：那正是冻结的 _latest 被当成实时画面反复解码（同一块
# 内存 → 坐标必然完全一致）。
# ⚠️ 必须大于 _reapply_profile 的丢弃窗（0.5s）+ 读线程正常抖动，2.0s 起步安全。
# ⚠️ 改后同样要 `sudo systemctl restart gks.service` 才生效（同上）。
LEASE_STALE_S = 2.0

# 摄像头初始化失败重试间隔（秒），0 表示不重试直接返回 None
CAM_INIT_RETRY_INTERVAL = 2

# 默认分辨率
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

# 高分辨率模式（后续精细定标等任务使用）
FRAME_WIDTH_HIGH = 1920
FRAME_HEIGHT_HIGH = 1080

# ==================== 串口 / 单片机通信 ====================
# 协议定义见 protocol.py（帧格式/TYPE 码/CRC8），交付单片机端的说明见 docs/串口协议.md。
SERIAL_PORT = "/dev/ttyUSB_GKS"   # 单片机主串口，符号链接按 CH340 芯片版本号绑定（仓库根 99-gks-devices.rules）
BAUDRATE = 115200                 # 波特率（单片机端固定 115200）
SERIAL_TIMEOUT = 0.1              # 读取超时（秒），影响 receive() 轮询粒度

# 模拟器虚拟串口：GKS_Web 的无头单片机模拟器监听此 TCP 地址。
# 与真实串口平级、无静态开关 —— main.py 会同时等待「真实串口」与「模拟器虚拟串口」
# 两个信号，谁先就绪就用谁：
#   - 真实串口先就绪 → 正常模式（比赛）
#   - 模拟器先就绪   → 模拟模式（无单片机单端调试）
# 比赛时不启动网页模拟器即可自动进入正常模式。
SIMULATOR_HOST = "127.0.0.1"
SIMULATOR_PORT = 9997

# 握手：主程序启动后发 HANDSHAKE_DATA，等待单片机回 HANDSHAKE_ACK_BYTE。
HANDSHAKE_DATA = "SYNC_HS_1"
HANDSHAKE_ACK_BYTE = 0xCC

# 空闲心跳帧间隔(秒)：握手成功后空闲时定期发 TYPE_HELLO 空帧，
# 单片机端据此检测链路存活（超时未收到判 Disconnected，重连重握手恢复 OK）。
# 设小可快速感知断开，但会轻微增加串口占用——2s 足够。
RPI_HEARTBEAT_INTERVAL = 2.0

# ==================== OpenMV 寻线（中转站） ====================
# OpenMV 按需发送：默认不发送，收到树莓派写来的 'G'(开始) 才发文本帧
# "L,<offset>,<angle>,<valid>\r\n"，写 'S'(停止) 停止。常驻线程读取
# （openmv_serial.py）；收到单片机巡线命令后由 tasks/_line.py 归一化 + PID
# + 以 TYPE_COORD 帧实时转发单片机。
# ⚠ 「端口打开成功」≠「传感器在发」：空闲时零字节是正常的；发了 'G' 后仍无数据才告警
#   （openmv_serial._check_silent + /line_status 的 expecting/silent_s/hint）。
OPENMV_PORT = "/dev/ttyUSB_OpenMV"   # ATK-USB-UART(CH340)，同样按芯片版本号绑定（不看物理插口）
OPENMV_BAUDRATE = 115200
OPENMV_SILENT_WARN_S = 3.0           # 发了 'G' 后多久收不到数据就告警（正常 ~30Hz，3s 无数据即异常）
OPENMV_LED_BRIGHTNESS = 40           # OpenMV 补光亮度 0~100（相机 P6 PWM，巡线补光）。
                                     # 消费者: handler._cmd_openmv_led（单片机 "L,<n>" / OLED
                                     # 「巡线→调参→补光亮度」/ 积木 STEP_OPENMV_LED 都改它）。
                                     # ⚠️ 这个值是「**点亮时用多少**」的唯一真相, 也是命令 'T'
                                     # （临时开关）"再开"时用的值: 单片机发来的 0 只临时灭灯、
                                     # **不写这里**, 否则关一次就再也开不回来（现场踩过）。
                                     # ≥1 时才持久化到 config_override.json, 且每次巡线起流("2")
                                     # 前自动重发 —— OpenMV 断电重启回编译默认也不丢。
                                     # ⚠️ 默认 40 与 OpenMV 端 main.py 的 LED_BRIGHTNESS 编译默认
                                     # 一致; 改这边不会动相机端编译值（那边要重烧）。

# PID 参数（⚠️ 2026-10 勘误: Web 设置页保存**不会**对 main.py 即时生效 —— 那是 GKS_Web
# 进程的副本; 消费者在 main.py(gks.service), 要重启它才生效。见上方看路掩膜那段说明）
LINE_KP = 0.7          # 比例
LINE_KI = 0.0          # 积分（初始 0，实车再调）
LINE_KD = 0.15         # 微分
LINE_I_LIMIT = 50.0    # 积分限幅（绝对值）
LINE_D_FILTER = 0.6    # 微分低通系数（0=不滤波，越大越平滑）
LINE_ERR_SIGN = -1.0    # 误差方向标定（实车若转向反了改 -1，免改代码）
LINE_SETPOINT = 20.0     # 巡线目标偏移(offset 单位, 名义 ±100)：0 = 相机零点/画面正中
                          # 想稳定偏在线某一侧就填 ±(几~几十); 正负先小步试(±5)看车往哪偏
                          # 在 _line.py 算 err 前减掉; Web 设置页可热调(写 config_override.json)

LINE_SEND_HZ = 30.0        # 转发频率（Hz）
LINE_STALE_TIMEOUT = 0.5   # 传感器数据过期阈值（秒），过期按丢线处理
LINE_MAX_DURATION = 1000.0   # 最大巡线时长保护（秒），超时自动停

# 丢线三段（2026-09 升级为"方向感知"）：
#   OpenMV 帧第 5 字段 state 报丢线分类，⚠️ 语义**以相机盘 main.py 头部为准**：
#   2 = 压灰区（off 相对直线基线衰减）/ 3 = 压黄区（off 抬升）/ 4 = 说不清。
#   （**不是**旧版的「2=边界远出画面 / 3=压到黄区」—— 那两代在 main.py 头部
#    「第三次↔第四次」之间翻过一次，数字沿用但含义换了，详见 tasks/_line.py 文件头。）
#   state=2/3：先静默 LINE_LOST_CONFIRM_S（躲单帧毛刺），其后**按 state 定向找线**
#   —— steer 大小取 |LINE_SEARCH_STEER|，方向 = state + LINE_ERR_SIGN 推导
#   (tasks/_line.py::_toward_steer) **再乘 LINE_SEARCH_STEER 的极性**（正=沿用推导、
#   负=整个反过来；2026-10-01 起这个符号才真的生效，见 tasks/_line.py 文件头）。
#   state=4/旧固件：走旧三段（保持 → 固定方向找线 → 停），此时 LINE_SEARCH_STEER 的
#   **符号**就是那个兜底方向。两路都受 HOLD+SEARCH 总预算封顶，超时停（防跑飞）。
LINE_LOST_HOLD_S = 5.0      # 丢线后沿用「最后有效 steer」的秒数：短暂丢线不再打断动作
                            # （丢线率一高，清零会让车在「横移↔停车」间反复跳，几乎不前进）
                            # 0 = 丢线立刻停（旧行为）。方向明确(state=2/3)时它只当总预算的一部分
LINE_LOST_CONFIRM_S = 0.4   # 定向找线确认期（秒）：方向明确的丢线先静默这么久再动，
                            # 单帧毛刺（运动模糊/曝光抖动）在这里就被吸收掉，不会误触发找线
LINE_LOST_SEARCH_S = 0.0    # 找线段时长（秒，从最后有效帧起算的总预算 = HOLD_S+本值）；
                            # 0 = 不找线，直接停
LINE_SEARCH_STEER = 0.0     # 找线速度（steer 单位; 单片机侧 ×LINE_STEER_TO_VX 得 rpm）
                            # **大小** = |本值|；**符号 = 定向找线的整体极性**（2026-10-01 起
                            # 真的生效）：正 = 用 state + LINE_ERR_SIGN 推导的方向（见
                            # tasks/_line.py::_toward_steer），负 = 把那一整套反过来。
                            # → 现场发现"收到方向 2 时车一直朝一边靠"，**第一件事就是把这个
                            #   符号翻过来**（不用改相机盘、也不用动 LINE_ERR_SIGN——那会把
                            #   PID 一起翻）。方向对不对看日志里的 `steer=` / `找线=`。
                            # 例：单片机 LINE_STEER_TO_VX=30 时填 0.4 → 12rpm 慢速横移找线
                            # 两处用它: ① 丢线找线段 ② offset 满偏(±100, 交界线贴画面
                            # 边缘)时直接改用它, 方向跟 err 符号（那一处**不吃**极性）
LINE_LOG_HZ = 5.0          # 巡线原始值打印频率（Hz），0 = 关闭。
                           # 限流打印：转发循环 30Hz，逐帧打印会刷爆日志文件（伤 SD 卡，见
                           # main.py::_TeeWriter）。打印的是归一化/PID 之前的传感器读数。

# ==================== 系统级操作（关机/重启）的二次确认 ====================
# 单片机发来的危险指令（"sd" 系统关机 / "rb" 系统重启 / "r" 重启主程序）**必须连发两次**
# 才执行：第一次只登记并打日志，同一条指令在 SYSTEM_CMD_CONFIRM_S 秒内第二次到达才真动手。
# 窗口可以给得比较宽松（默认 10s），因为「两次」是**两次操作**（操作员按两次键，中间还要看
# OLED 提示），不是连着发两帧 —— 连发两帧等于没有确认（误触也会连发）。
# 超窗口再发 = 重新登记，所以「隔很久误按一次」什么也不会发生。0 = 关闭确认（危险）。
SYSTEM_CMD_CONFIRM_S = 10.0

# ==================== Web 实时预览 ====================
# 统一调试预览方案（MJPEG over HTTP），替代 cv2.imshow / Qt 窗口。
# 端口与 JPEG 质量由 web_preview.py 使用。
WEB_PREVIEW_PORT = 8080          # MJPEG 推流默认端口
WEB_PREVIEW_QUALITY = 70         # JPEG 编码质量 (1-100)

# ==================== 二维码识别 ====================
# ---- 超时：单片机说了算（"<命令码>,<毫秒>" 扩展，2026-09 加）----
# 阻塞类视觉指令（"1"/"D"/"E"/"F"）的超时**随指令下发**：单片机把它那一步愿意等的
# 毫秒数附在命令码后面，本端拿它减一个余量当自己的预算。于是「谁先放弃」只由
# **单片机那一个数**决定，不用两个仓库各记一个数字互相迁就。
#
# 余量：单片机是从**它发指令的那一刻**开始计时的，而这段时间里本端还要
# 「开相机 + 扫描 + 回帧」。要盖住冷启动开相机（租约没命中时实测 ~1.3s）。
# 预热 "B"/"C" 能省掉开相机的时间，但余量照留 —— 宁可少试一轮也不能迟到
# （迟到的结果帧会落进下一次重扫的等待窗口里被当成那一次的结果）。
QR_CMD_MARGIN_S = 1.5

# 本端预算的上限。相机租约的空闲看门狗（LEASE_IDLE_TIMEOUT_S）只被 read() 续命，
# 单次扫描超过它会在**解码途中**把设备关掉，后续 read() 全失败、扫描循环空转到超时。
# 超了就夹住并打一条 WARN（真实耗时见日志里的「用时 x.xxs」）。
QR_CMD_MAX_TIMEOUT_S = 12.0

# 单次识别超时（秒），超时或被打断返回失败。
# ⚠️ 从串口来的指令**不再用这个值**（见上面的 QR_CMD_MARGIN_S）——它退化成两种情况
#    的兜底：① 老固件（不带毫秒）发来的指令；② 网页 /scan 就地扫码（没有单片机参与）。
QR_SCAN_TIMEOUT = 5.0

# 扫码预处理：识别前先把画面按此比例放大（提升小二维码识别率；1.0=不放大）。
# ⚠ 仅对 pyzbar/opencv 有意义（直接吃像素）；微信引擎检测输入固定 384x384，
#   整帧等比缩放后二维码占比不变，调此参数无效——请调 QR_SCAN_CENTER_CROP。
QR_SCAN_SCALE = 1.0

# 识别前先把画面降采样到该最大边长内（保证动态识别速度，二维码不需要全分辨率）
QR_SCAN_MAX_SIZE = 960

# 扫码前裁取画面中心区域的比例（0.0=不裁剪，0.5=取中心 50% 正方形区域）。
# 微信引擎检测网络输入固定 384x384，整帧会被等比缩到 384 再检测；二维码在
# 整图中的占比越大越易检出。二维码固定居中且较小时（如 100x100），裁掉四周
# 杂乱背景可显著提升检测率（放大无效，因为整帧最终仍被缩到 384）。
QR_SCAN_CENTER_CROP = 0.3

# 透视校正：识别前先找二维码黑块并矫正畸变（侧着/斜着时），再识别
QR_SCAN_PERSPECTIVE = True

# ---- 主摄扫码（命令 "D"，2026-09 加）----
# 与上面 QR_SCAN_* 是**两套独立参数**：命令 "1" 用扫码副摄（/dev/camera_Qr），
# 命令 "D" 用主摄（/dev/camera_main），而两者的取景差得很远 —— 共用一套会互相打架。
# ⚠ 最要紧的是中心裁剪：QR_SCAN_CENTER_CROP=0.3 是为副摄的近距离取景调的，
#   主摄对着场地时二维码**多半不在画面中心 30% 内** → 主摄这条路默认 0.0（不裁剪）。
#   真需要裁（比如主摄也贴得很近）再按现场情况往上调。
QR_MAIN_SCAN_SCALE = 1.0
QR_MAIN_SCAN_MAX_SIZE = 960
QR_MAIN_CENTER_CROP = 0.0
QR_MAIN_SCAN_PERSPECTIVE = True

# 扫码最后一帧保存路径（main.py 执行命令「1」/「D」后写入标注帧，Web 控制端预览显示用）
QR_LAST_FRAME_PATH = os.path.join(LOG_DIR, "qr_last.jpg")

# 微信二维码引擎（OpenCV wechat_qrcode）CNN 模型目录
WECHAT_QR_MODEL_DIR = os.path.join(BASE_DIR, "models", "wechat_qrcode")

# ---- 自研难度算法（命令 "E" 副摄 / "F" 主摄，2026-09 加）----
# 与 wechat 并列的**手动第二档**，治斜视/小码/强透视（见 tasks/_qr_diff.py）。
# 它是一整套自研几何重建 + 候选集匹配，**慢**，所以单独给自己一套参数。
#
# 台架实测（2026-09-26，本车 8GB 树莓派 + 主摄对着码板）：成功一次 **4.48s**，
# 开发机同图 1.0s → 倍率 4.5×（落在预估的 3~5× 里）。
# 同一位姿下 wechat **连试三次全失败**（「未检测到二维码」）—— 这就是它存在的理由。
#
# ⚠️ 从串口来的指令**不再用这个值**（用单片机下发的，见 QR_CMD_MARGIN_S）。它退化成
#    网页 /scan 就地扫码时的兜底。
# ⚠️ 选「自研」的那一步，timeout_ms 建议给 **7500**：本端预算 = 7500/1000 − 1.5 = 6.0s，
#    实测 4.5s 够用；而 7500 仍 < RPI_DISCONNECT_TIMEOUT_MS(8s)，链路不会中途闪断。
#    ⚠️ 别为了多给余量把它抬过 8s：RPI_DISCONNECT_TIMEOUT_MS 是**编译期全局**，
#       抬了积木超时不会跟着动，链路中途会闪一次 Disconnected（不是故障，Pi 一应答
#       即自愈，代价是 OLED 闪一下 + 蓝牙臂编辑窗被关一次）。
QR_DIFF_TIMEOUT = 6.0

# 候选集：赛题二维码内容固定是三位数、每位 1~3 → '123' = 111..333 共 27 种。
# ⚠️ '123' 里那三个字符是**每一位的取值范围**（字母表 × 长度 3），不是「从 123 开始」。
#    换别的就用显式列表，如 '111,222,333'。**长度只能 1~3 位**（数字模式单块上限），
#    配成 4 位会当场报错而不是静默退化。
# ⚠️ 内容不在候选集里时自研**必然失败**（它靠 ECC 字节精确匹配定内容，不做一般解码）。
QR_DIFF_KNOWN = "123"

# 自研的预处理参数（第三条路，与 QR_SCAN_* / QR_MAIN_SCAN_* 都不同）：
#   中心裁剪 0.0   —— 斜视位姿下码面不一定在画面中心，裁了就没了
#   max_size 1280  —— 比 wechat 的 960 宽：自研的定位角检测是**按分辨率调过**的
#                     （块大小 51/75/101、轮廓面积门限 400），降到 960 会把小码的
#                     定位角压到门限以下。代价是几何阶段更慢。
QR_DIFF_SCALE = 1.0
QR_DIFF_MAX_SIZE = 1280
QR_DIFF_CENTER_CROP = 0.0

# 自研失败时的中间产物留档（原图/正视图/21x21 采样矩阵/各候选命中数）。
# ⚠️ 只在失败时写，且只保留最近 QR_DIFF_DEBUG_KEEP 次 —— Pi 的 logs/ 是只增不减的，
#    反复重试会把 SD 卡写满。
QR_DIFF_DEBUG_DIR = os.path.join(LOG_DIR, "Difficult")
QR_DIFF_DEBUG_KEEP = 20

# ==================== 人质形状识别（主摄像头 YOLO）====================
# 指令 "4"：主摄像头识别 圆柱/圆锥/腰鼓 三种人质形状，按图像从左到右排序，
# 以 TYPE_COORD 二进制帧（TAG=0x02）回传 形状码+中心像素坐标，见 docs/串口协议.md §6c。
HOSTAGE_MODEL_PATH = os.path.join(BASE_DIR, "gks_yolo_model", "best.onnx")
HOSTAGE_CLASSES_PATH = os.path.join(BASE_DIR, "gks_yolo_model", "classes.names")

# ⚠ 必须与 ONNX 导出时的 --imgsz 完全一致（vision/export.py，静态输入尺寸）。
#   模型原生训练尺寸 1280；当前导出 640（Pi CPU 单帧 ~1s，6s 超时内可多帧重试）。
#   若现场小目标漏检严重：重新以 --imgsz 1280 导出并同步改此处（推理约慢 4 倍）。
HOSTAGE_INPUT_SIZE = 640

HOSTAGE_CONF_THR = 0.4     # 检出置信度阈值
HOSTAGE_IOU_THR = 0.5      # NMS IoU 阈值
HOSTAGE_TIMEOUT = 6.0      # 单次识别超时（秒）—— 须 < 单片机 RPI_DISCONNECT_TIMEOUT_MS(8s)
HOSTAGE_MIN_TARGETS = 3    # 凑满该数量目标才算成功（比赛 3 个人质）
HOSTAGE_MAX_TARGETS = 3    # 回传目标数上限（超出按置信度取前 N）
HOSTAGE_ANCHOR_CONF = 0.90 # 排除法锚点置信度：台面恒为 圆柱/圆锥/腰鼓 **各一个**（绝无
                           # 两个同形）。单帧检出 {A,A,B}（能凑满 3 个但两个同形）且重复中
                           # 较高一枚 ≥ 此值时，把较低的那枚按「缺席的第三形」重标
                           # （tasks/_hostage.py::_apply_elimination）。锚点不达标不动——
                           # 弱证据不改判，交给后续帧。

# 扫描最后一帧保存路径（执行指令 "4" 后写入标注帧，Web 控制端预览显示用）
HOSTAGE_LAST_FRAME_PATH = os.path.join(LOG_DIR, "hostage_last.jpg")

# 模型类名 → 形状码（与单片机端 emHostageShapeTdf 对齐：1=圆柱 2=圆锥 3=腰鼓）。
# 按类名（而非类 ID）映射——类 ID 随训练顺序变化，类名由 classes.names 唯一确定。
HOSTAGE_NAME2SHAPE = {"cylinder": 1, "cone": 2, "pellet": 3,
                      "drum": 3, "waist drum": 3}
# 形状码 → 中文名（日志/标注/Web 显示用）
HOSTAGE_SHAPE_NAMES = {1: "圆柱", 2: "圆锥", 3: "腰鼓"}

# ==================== 色球顺序识别（排爆区，主摄像头 HSV + 连通域）====================
# 指令 "5"：主摄像头识别一行三球（红/绿/蓝各一个，顺序抽签决定）的颜色顺序与中心
# 坐标，以 TYPE_COORD 二进制帧（TAG=0x03）回传 color+cx+cy，见 docs/串口协议.md §6d。
#
# ⚠️ 多帧投票要求**小车静止**——所有票必须来自同一机位。单片机须在底盘停稳后才发 "5"。
#
# ⚠️ 判定分两步（见 _color.py::_detect_frame）—— 这是阈值能放很宽的关键:
#   ① **找球**: 只看「饱和且够亮」, **完全不看色相**（COLOR_SAT_MIN / COLOR_VAL_MIN）。
#      为什么不用三色掩膜找: 宽色相阈值会把球在桌面上的**彩色反光**也吃进来,
#      连通域变成碎形（实测绿球圆度只有 0.29 被剔掉）; 只看饱和度, 球是紧密整体（0.74）。
#   ② **定色**: 在球轮廓内比「哪一族色相像素最多」—— **相对判定**, 不是绝对阈值。
#
#   所以三族色相是**对色相环的无空隙划分**（见下）, 不留缝——留缝就会出现
#   「色相漂到缝里 → 三族都不认」的球。配合①②, 光照/白平衡把色相推走也只会
#   影响判成哪一族, 不会整颗球消失。参考: pro 项目 cv2_python 的 max(r,g,b) 判定。
#
# 色相族 (OpenCV H∈[0,179])。**红跨 0/180, 必须是两段**, 且三族首尾相接不留缝:
#   红 155-180 ∪ 0-12 | 绿 13-88 | 蓝 89-154
# 初值参照命题 CMYK 参考色换算（红 H≈0/179、绿 H≈54、蓝 H≈90）与 cv2_python 实测
# （红高段从 155 起 —— 160 都还不够, 玫红/粉红会掉出去）。
COLOR_HUE_FAMILIES = {
    1: [(0, 12), (155, 180)],   # 红（跨 0 回绕; 155 起以容玫红/粉红）
    2: [(13, 88)],              # 绿
    3: [(89, 154)],             # 蓝（青蓝 H≈90 到深蓝 H≈154 都收）
}

COLOR_SAT_MIN = 100           # 找球用的饱和度下限（0~255）。太低会把彩色反光/背景当球
COLOR_VAL_MIN = 60            # 找球用的亮度下限（0~255）。防近黑像素（近黑 S 可能很高）
COLOR_PROC_SCALE = 0.5        # 检测前降采样比例（1.0=全分辨率; 1920×1080 下 0.5 约快 3~4 倍）
                              # ⚠️ 回传坐标已自动还原回全分辨率（1920×1080），别在这里换算
COLOR_MIN_AREA = 60.0         # 最小连通域面积[px²]（**降采样后**坐标系下的面积）
COLOR_MAX_AREA = 200000.0     # 最大连通域面积[px²]（防整片背景）
COLOR_MIN_CIRCULARITY = 0.60  # 圆度 4πA/P² 下限（1.0=正圆）。0.5 太松会放进背景块
COLOR_MIN_COLOR_PIXELS = 40   # 球内获胜族最少像素数[px², 降采样]; 低于此认为该轮廓不是色球
COLOR_VOTE_FRAMES = 5         # 最多采帧数
COLOR_MIN_VOTES = 3           # 单色最少命中帧数（众数语义）
COLOR_MAX_SPREAD_PX = 40.0    # 同色 cx 跨帧最大散布[px]（超限=车在动/误检 → 该色作废）
COLOR_MIN_GAP_PX = 30.0       # 三球 cx 最小间隔[px]（防同一球被高光裂成两块）
COLOR_FIT_MAX_DIST_PX = 10.0  # 三球共线质检上限[px]: 质心到 fitLine 直线最大距离超它 → fit_ok=0。
                              # ⚠️ 2026-10-01 起这个判据**只影响日志/遥测**(Pi 日志的
                              #    「共线NG」那行、蓝牙 ball_fit_ok)——单片机端已不再据此
                              #    重扫或拒动（"宁可动错也不拒动", 见 app_rpi.c::ucBallScanOnce）。
                              #    所以本值不再决定"对位会不会被拦", 只是"哪帧该被标成可疑"。
                              # 10px 的来历（放开到 4 会天天报警）：场地摆球公差 ±10mm ≈ ±10px
                              #    纵向偏差是**静态的**(投票平均不掉), 实测绿球低 13px 就顶爆
                              #    4px 阈值; fitLine 是最小二乘, 单球 10px 偏差带来的角度噪声
                              #    有界(~0.7°图像角), 而**判定要真报警的那种错配**实测残差
                              #    是 100px 量级（例: 131/168/37px）—— 中间留着很宽的余量。

COLOR_TIMEOUT = 5.0           # 单次识别超时（秒）—— 须 < 单片机 RPI_DISCONNECT_TIMEOUT_MS(8s)
COLOR_MIN_TARGETS = 3         # 凑满该数量才算成功（比赛 3 个球）
COLOR_MAX_TARGETS = 3         # 回传目标数上限（与单片机端 COLOR_MAX_COUNT 对齐, 双端同步改）

# 颜色码 → 中文名（日志/标注/Web 显示用；与单片机端 color_t 对齐: 1红 2绿 3蓝，
# 也等于二维码第一位）
COLOR_COLOR_NAMES = {1: "红", 2: "绿", 3: "蓝"}

# 色球识别最后一帧保存路径（执行指令 "5" 后写入标注帧，Web 控制端预览显示用）
COLOR_LAST_FRAME_PATH = os.path.join(LOG_DIR, "color_last.jpg")

# ---- 缺球推断（2026-09）：只认出两颗时把第三颗推回来 ----
# 赛题必定给三颗球（红/绿/蓝各一），所以"只认出两颗"基本是识别问题。推断出来的第三颗
# **不打标记**：下游（单片机对位/抓取）拿到的就是一个正常的三色结果。
# 位置推断优先用板子当坐标系（板子四角 → 单应变换 → 球位在板归一化坐标里是固定的），
# 表对不上则退回"等距自洽"，板子都看不到再用"两球外推"；任何一层判据不过就**不推**。
# 算法与判据见 tasks/_color.py 顶部「缺球推断」那一节。
COLOR_INFER_EN = 1            # 1=启用缺球推断
COLOR_INFER_MIN_BALLS = 2     # 恰好检到这么多颗才推（1 = 只认出一颗也推，更激进）
COLOR_INFER_RETRY = 1         # 1=推断之前先多采一轮（票数不够时）
# 球位模型（板归一化坐标 u=横向 0~1；实测本机位 0.159/0.500/0.842, v≈0.375）
# ⚠️ 上真场拿三球齐全的一帧复核这三个 u：跑一次「色球识别」，看日志里的 u 值
COLOR_BALL_U_CENTER = 0.5     # 中球位（= 球行横跨板子中线；实测正是 0.500）
COLOR_BALL_S = 0.341          # 相邻球位间距（板宽比例；实测 0.341/0.342）
COLOR_BALL_S_TOL = 0.05       # 推导间距与标定值的允许偏差
COLOR_BALL_CENTER_TOL = 0.05  # 球行中线残差上限
COLOR_BALL_U_MIN = 0.08       # 球位必须在板内且不贴边
COLOR_BALL_U_MAX = 0.92
COLOR_BALL_AGREE_TOL = 0.04   # 两条独立估计（中线先验 / 间距外推）的允许差
# 「两球外推」兜底用的弱先验：理想停车位下中球该在的像素列（对齐单片机 BALL_CX_CENTER）
COLOR_BALL_CX_CENTER = 926.0
COLOR_BALL_CX_TOL_PX = 60.0

# 矩形识别最后一帧保存路径（执行指令 "A" 后写入标注帧，排障用）
RECT_LAST_FRAME_PATH = os.path.join(LOG_DIR, "rect_last.jpg")

# ==================== 色球区矩形地板（黑胶带边框，主摄像头）====================
# 用途：一个**不依赖色球**的横向基准 —— 缺球时三球齐那套整段拒动，而板子位置每次一样，
# 故"板心在画面里的坐标"可以当对位基准（标定一次长期用）。算法见 tasks/_rect.py 顶部。
# 只在 GKS_Web 控制端的「色球区矩形」按钮里跑（GKS_Web/app.py::_handle_rect），
# **没有**对应的串口指令（单片机侧还没有消费它的步骤）。
# ⚠️ RECT_HW_RATIO 是**三边估中心**用的唯一经验常数（长边:短边像素比），
#    标定法：车停在理想停车位、四边都齐全时跑一次，用返回的四角算
#    ((上边+下边)/2) / ((左边+右边)/2) 填进来（本机位实测 3.66）。
# ⚠️ 三边估出来的中心与四边精确中心**在同一机位差 ~20px**（仿射补边不吃透视），
#    标定目标值请用你实际会跑的那个形态，两者别混用。
RECT_TIMEOUT = 5.0            # 单次识别超时（秒）—— 9 帧 × ~180ms + 开摄像头 ≈ 2.6s
RECT_VOTE_FRAMES = 9          # 最多采帧数（采多几帧才压得住抖动；内点取均值，噪声 ÷√N）
RECT_MIN_VOTES = 3            # 成功帧数下限（帧数够才给数，抗单帧误检）
RECT_MAX_SPREAD_PX = 40.0     # 中心跨帧散布上限[px]（超限=车在动/误检 → 报失败）
RECT_OUTLIER_K = 3.0          # 多帧: MAD 离群倍数（抖得野的帧按它剔）
RECT_OUTLIER_MIN_PX = 6.0     # 多帧: 离群判据下限[px]（抖得小就一个不剔）

RECT_V_MAX = 100              # 黑胶带：亮度上限（V）
RECT_S_MAX = 120              # 黑胶带：饱和度上限（S，挡掉彩色的暗部/反光）
RECT_CLOSE_K = 9              # 暗掩膜闭运算核（桥接胶带断线）
RECT_CLOSE_ITER = 2
RECT_MIN_SEG_LEN = 0.06       # 直线片段最小长度（× 画面宽）
RECT_ANGLE_TOL_DEG = 8.0      # 共线聚类的角度容差
RECT_DIST_TOL_PX = 30.0       # 共线聚类的垂距容差（≈ 胶带宽×2）
RECT_MIN_LINE_LEN = 0.12      # 长边（板子的长边）最小支撑长度（× 画面宽）
RECT_MIN_SHORT_LINE = 0.04    # 短边最小支撑长度（短边天生短，还被画面切过 → 给很小）
RECT_TAPE_OFF1 = 8            # 胶带判据的垂直采样偏移1[px]
RECT_TAPE_OFF2 = 16           # 同上偏移2[px]
RECT_PAR_TOL_DEG = 12.0       # 与长边"近平行"的判据（另一条长边）
RECT_TRANS_MIN_DEG = 45.0     # 与长边"横交"的判据（短边）
RECT_MIN_GAP_FRAC = 0.10      # 两条长边的最小间距（× 板宽）——挡"同一条胶带的内外沿"
RECT_END_TOL_PX = 30.0        # 长边必须经过短边胶带端点的容差[px]（角点判据，挡贯穿地面缝）
RECT_MIN_WIDTH_FRAC = 0.25    # 两条短边的最小间距（× 长边长度）
RECT_MIN_AREA_FRAC = 0.04     # 四边形可见面积下限（占画面）
RECT_MIN_INNER_V = 130.0      # 框内平均亮度下限（挡车体那种暗色封闭区）
RECT_MIN_INNER_RATIO = 0.75   # 框内平均亮度 / 画面中位亮度 的下限（同上，自适应光照）
RECT_HW_RATIO = 3.66          # 长边:短边（像素, 理想停车位四边齐全时实测）——三边估用
RECT_INNER_CLOSE = 25         # 主路径: 围住区掩膜的闭运算核（吞球影/胶带毛刺，出干净四角）
RECT_EPS_FRAC = 0.02          # 主路径: 多边形拟合精度（× 周长）
RECT_TAPE_NEAR_PX = 6.0       # 主路径: 判"这边紧贴胶带"的外偏距离[px]
RECT_MIN_FILL = 0.85          # 主路径: 面积/最小外接矩形 下限（四边形够"实"）
RECT_MAX_ASPECT = 12.0        # 主路径: 最小外接矩形 长/短 上限
RECT_MIN_ASPECT = 2.5         # 主路径: 长/短 下限 —— 治"板子被暗纹切成一块"
                              #   （碎块的长宽比会明显变小；本机位整块实测 3.5~3.8）
RECT_EDGE_BAND_FRAC = 0.01    # 主路径: 边拟合只收"距该边 < 该比例×周长"的点
RECT_EDGE_MIN_PTS = 40        # 主路径: 每条边最少点数，不够就退回 approxPolyDP 顶点

# ==================== 看路（黄色区左边缘测量，主摄像头，命令 "G"）====================
# 算法（第二版：2026-09-27 换成"整块黄色区"思路，详见 tasks/_road.py 顶部注释）：
#   黄色区（比赛是黄胶带，练习地图上是那块米黄篷布）**一般落在画面右上角**；
#   车越往前开它越大、**左边缘越往左跑** —— 于是"黄色区左边缘的 x"本身就是一把测距尺。
#   Δx = 黄色区左边缘的 x（全分辨率像素；用户口径的"原点" = 画面左边 x=0）。
#   单片机按 (理想Δx − 实测Δx) × 比例系数 × 方向(±1) 修一次**车体纵向**。
# ⚠️ 判据用 **R−B（色度差）**，不用 HSV 的 H/S/V：补光灯把画面中心打到过曝（S 掉下去、
#    V 顶到 255）时，亮度/饱和度类阈值会被光照带着跑；R−B 是色度关系，光照整体明暗变化
#    基本不动它。实测本机位：米黄区 R−B≈**+7** / 灰地≈−11 / 黑线≈−18 / 车身≈−32
#    —— 18 级余量，阈值取 0 附近就干净分开，**车身自己就被排除**（不用另做剔除）。
# ⚠️ 第一版做的是"拟合弯道两条直边→求交→圆弧相切→取切点"（`看路.jpg` 的红色标注就是
#    那么画的）。它在**合成本例上完全正确**，但在真实机位上过不去（篷布褶皱/软阴影边：
#    Hough 把圆弧的弦当直线、折线化又因边界碎片化只给出短边 → 配不成"两条腿"）。
#    那一版存在 `tasks/_road.py.bak-curve`，完整复盘见 `Desktop/road_offline/README.md`。
# ⚠️ 标定顺序：先定 ROAD_PROC_SCALE 与 ROI（看 Web 预览里黄框贴不贴边）→ 再调
#    再看 Web 预览（掩膜要一次框全黄区、又不沾灰地）→ 最后才标单片机的
#    ROAD_DX_IDEAL_PX（把车摆到理想停车位，读蓝牙 Tab 的 road_dx 填进去）。
# ⚠️ 只读一帧时车必须静止（多帧投票要求同一机位，与色球/矩形一致）。
ROAD_LAST_FRAME_PATH = os.path.join(LOG_DIR, "road_last.jpg")

ROAD_TIMEOUT = 4.0            # 单次识别超时（秒）—— 必须 < 单片机 RPI_DISCONNECT_TIMEOUT_MS(8s)
ROAD_VOTE_FRAMES = 9          # 最多采帧数
ROAD_MIN_VOTES = 3            # 成功帧数下限
ROAD_MAX_SPREAD_PX = 25.0     # Δx 跨帧散布上限[px]（超限=车在动/锁到别处 → 失败）
ROAD_OUTLIER_K = 3.0          # 多帧: MAD 离群倍数
ROAD_OUTLIER_MIN_PX = 5.0     # 多帧: 离群判据下限[px]（抖得小就一个不剔）

ROAD_PROC_SCALE = 0.5         # 降采样倍数（用户第一要求是快）
ROAD_ROI_TOP = 0.0            # ROI 上边（占画面比例）—— 黄区在右上，一般不用裁上边
ROAD_ROI_BOTTOM = 0.72        # ROI 下边 ⚠️ 车身/机械臂全在画面下半，靠它切掉；但
                              #   **黄区左边缘必须落在窗内** —— 切太狠会把左边缘切到框外
ROAD_ROI_LEFT = 0.0            # ROI 左边 ⚠️ 裁它**不会平移 Δx**（Δx 一律按**全分辨率**计,
                              #   见 _detect_one 里 x0 + xmin/sc），所以它只用来"把左侧的
                              #   干扰源排出窗外"。实测（2026-09-30）：比赛图上 0~0.16 时
                              #   Δx 恒为 270、Δy 恒为 618（黄区左边缘落在**画面宽的 0.275**
                              #   处 → 硬上限就在那附近）；台架间左侧有一条暖斑
                              #   （x≈20~80px ≈ 0.04）+ 大片左墙，现值 **0.10**（左边线在
                              #   x=192），裁完 Δx 972.7 与裁前 971.8 一致、3/3 稳。
                              #   ⚠️ **留下限的判据**: 要显著小于"停车位姿下 Δx 占画面宽的
                              #   比例"（比赛图 0.275）——**还要留"车比理想位更近"的余量**
                              #   （那时黄区左边缘更靠左、Δx 更小）。实践上别超过 ~0.13。
                              #   ⚠️ 一旦把黄区本体裁掉，测到的 Δx 会**贴在 ROI 左边线上**
                              #   （此时 Pi 会置 flags bit1 `FLAG_CLIP_X`, 但单片机**并不据此
                              #   判失败** —— 那一位只是提示）。
ROAD_ROI_RIGHT = 1.0

# ---- 黄色区掩膜（2026-10 改版：判据 R−B → **Y−B=(R+G)/2−B**，阈值改**每帧自适应**）----
# ⚠️ 为什么要换判据：换了更浅的新地图后，灰地自己变暖（实测现场帧 R−B ≈ +2~+8），
#    黄区只剩 +11~+19 —— R−B 余量从旧图的 18~20 级掉到 5~8 级，固定阈值 20（旧比赛图
#    口径）把整块黄区关在门外，掩膜只剩噪声碎片 → 锁错/失败（road_failed/1.jpg 复现）。
#    Y−B（黄度，蓝黄对立色度）实测同批帧：黄区 +12~+17 / 灰地 +2~+6 / 过曝热点 −6~−10
#    （偏蓝，天然排除）/ 白斑马线 +1~+2 —— 余量 10~13 级，是 R−B 的两倍；且对中性黄
#    (=R−B)、黄绿(≈G−B，新地图)、橙黄胶带(≈R−B) 都不吃亏。
# ⚠️ 为什么阈值不再写死：固定阈值的死穴是"换图/换光就得重标，标在分布中段就碎成小岛"
#    （2026-09-30 的头号坑，历史教训保留在下面）。现在每帧在 Y−B 直方图上现算（<2ms）：
#      ① 谷底：主峰（灰地）与右边第一个显著峰（黄区）之间 —— 自动落在两分布空隙里；
#      ② 黄区即主峰（车很近）→ 主峰−6；③ 极小黄区（车远）→ p99.5 锚定；
#      都过不了 → 直接判"没有黄色区"，**不出巨块假数**。算法细节见 tasks/_road.py 顶部。
#    ⚠️ 加大闭运算核救不了碎岛（close 15/31/41 → 0/3、2/3、2/3，其中一次还报了 1515 的错值）。
#    ⚠️ 用饱和度也分不开（黄区 S≈28~31 vs 白板 S≈18~21），别试。
#    历史各场景 R−B 实测（留档，别拿它当现在的阈值依据）：
#      真实比赛图: 黄区 +21~23 / 灰地 0~+1 / 白斑马线 +2~+4；练习地图: 米黄区 +7 / 灰地 −11；
#      台架间白板被暖光染到 10~17（"白"被染暖 → 别拿台架间读数当场地依据）。
#    **到场第一步**：开 Web 的「看路」预览看一眼 —— ① 黄区被框住 ② 白的东西/车身不在框里
#    ③ 掩膜别是"碎岛"（现在还碎 = 光照极端，把 ROAD_PEAK_FRAC 往下调一档试试）。
#    ⚠️ **判据/阈值口径变了，Δx 会平移 10~30px**（掩膜边缘定义跟着变）—— 到场**必须重标**
#       单片机的 `road_dx_ideal` / `road_dy_ideal`，旧标定作废！
#    ⚠️ 现值由 Web「设置参数」写进 config_override.json（**不在本文件里**）。⚠️ 双进程陷阱:
#       Web 页保存只改 **GKS_Web 自己进程**的副本 + 文件; 消费它的是 **main.py(gks.service)**
#       —— main.py 只在启动时加载一次, 所以改完**必须 `sudo systemctl restart gks`** 才对车生效!
#       （本文件里的数 = 这个进程从没被 Web 改过时的初值。）
ROAD_WARM_FLOOR_YB = 4        # Y−B 地板：自适应阈值的下限 —— "黄区再浅也得比白的多几点黄"。
                              #   实测黄区最低 +7（旧练习图），填 4 全场景留余量；
                              #   调高=更挑剔（冷光下黄区会被整个甩掉），调低=垃圾掩膜风险。
ROAD_PEAK_FRAC = 0.05         # 谷底判黄区峰的显著度：峰高 ≥ 主峰×它才算黄区。
                              #   0.05 ≈ 黄区占 ROI ≥4~5% 就走谷底；更小的黄区走 p99.5 锚定。
                              #   实测黄区占比 2%→峰比 0.023、5%→0.058、8%→0.097。
ROAD_WARM_V_MIN = 40          # 亮度下限(V=max(B,G,R)) —— 挡全黑噪声（车身已被 Y−B 排除，双保险）
ROAD_WARM_OPEN_K = 5          # 开运算核[px, 处理尺度]：去噪点
ROAD_WARM_CLOSE_K = 15        # 闭运算核[px, 处理尺度]：把区内的褶皱/反光洞补上
ROAD_WARM_MIN_AREA = 3000     # 黄区最小面积[px², 全分辨率] —— 小于它当"没看到黄区"

# ---- 闸与标志 ----
ROAD_DX_MIN_PX = 8.0          # Δx 下限[px] —— 更小说明黄区在画面最左边（几乎看不到）
ROAD_DX_MAX_PX = 1900.0       # Δx 上限[px] —— 大于它说明认错了东西（画面宽 1920）

# ==================== 靶标识别 + 靶心定位（反恐区打靶，主摄像头）====================
# 指令 "6"：识别三色靶盘（红/绿/蓝外环）并定位**每一颗的圆心**，以 TYPE_COORD
# 二进制帧（TAG=0x04）回传 color+cx+cy+r，见 docs/串口协议.md §6e。
#
# 算法见 tasks/_target.py 顶部注释。参数值来自 GongKeCompetition 仓库
# tools/target_offline/ 的离线评估（61 个用例，几何照现场实拍标定）：
#   中位误差 0.26mm、p95 0.59mm（判据是命题图 7 的 10 环半径 7.5mm）。
#
# ⚠️ **色相族不在这里配** —— 直接复用上面的 COLOR_HUE_FAMILIES，
#    「什么算红」只留一个来源，色球和靶标共用，免得两处阈值各自漂。
#
# ⚠️ 外环是**印刷色**，饱和度比塑料球低得多，所以 SAT/VAL 下限与色球那套分开定。
TARGET_TIMEOUT = 6.0          # 单次识别超时（秒）—— 须 < 单片机 RPI_DISCONNECT_TIMEOUT_MS(8s)
                              # 它只是**上限**：凑满 TARGET_VOTE_FRAMES 帧就立刻返回，
                              # 成功路径不等它。它决定的只是「采不满时多久认输」。
                              # 2026-09 提速后实测 ~220ms/帧（全分辨率 1920×1080），
                              # 8 帧 ≈ 1.8s，所以 6s 留了 ~3 倍余量给 Pi 被抢 CPU 的情形。
                              # ⚠️ 调它或 VOTE_FRAMES 之前先看日志里那行「靶标采帧 N 帧，用时..s」。
TARGET_SAT_MIN = 65           # 饱和度下限（0~255）。印刷色比色球的塑料件低
TARGET_VAL_MIN = 50           # 亮度下限（0~255）
TARGET_PROC_SCALE = 1.0       # 检测前降采样比例（1.0 = 全分辨率，见下）
                              # ⚠️ 回传坐标已自动还原回全分辨率，别在这里换算
                              # ⚠️⚠️ **默认 1.0，别随手调小**。实测 0.5 倍时「遮挡左 1/4 盘面」
                              #    会**整颗靶漏检**：降采样后遮挡物边缘点在轮廓里占的比例变高，
                              #    半径的 MAD 相对变大，圆度关卡把正确的圆判掉。
                              #    另外形态学核是固定 5×5、不随分辨率缩放，降采样后相对变粗
                              #    也会吃掉细结构。要开先跑 tools/target_offline 的 61 个用例复验。
TARGET_MIN_AREA = 120.0       # 最小连通域面积[px²]（**降采样后**坐标系下的面积）
TARGET_TOL = 2.0              # 跨色尺寸一致性：任一颗与中位数的倍数上限
                              # ⚠️ 别调紧。三靶并排同距实测半径差 <10%，但相机**凑近某一个**
                              #    时近的会明显大；2.0 是给那种情况留的余量。调紧会把真靶误杀。
TARGET_RING_Q_MIN = 0.35      # 「像不像靶环」的**内孔/外径**比下限（判据②'，2026-09 加）
TARGET_RING_Q_MAX = 0.85      # 上限。q = 内孔拟合半径 ÷ 该候选外接圆半径
                              # 物理依据：靶盘是「色环 + 白心」，设计值即 _target._R_INNER_OVER_OUTER
                              # = Ø55/Ø100 = **0.55**。现场实测（1920×1080、三靶并排）:
                              #   真靶环 q = 0.52~0.55（绿/蓝两色、连续 8 帧都稳定）
                              #   一切杂块 q ≤ 0.23：蓝柜子 0.00~0.03、蓝箱 0.00~0.06、
                              #   绿/红杂块 0.00~0.23、红靶与木椅腿的**合并块** 0.13~0.15
                              #   两者完全不重叠 —— 所以 0.35 这条线卡在中间的空白带里。
                              # ⚠️ 它不是「又一个尺寸判据」：q 只看**单个候选自己的形状**，
                              #    跟别的候选没关系，所以「三色凑巧一样大」骗不过它
                              #    （2026-09 蓝靶误识别的根因就是：三个杂块凑成"尺寸一致"组
                              #    骗过了纯尺寸的跨色共识，而它们的 q 全是 0.00~0.16）。
                              # ⚠️ 下限别往上抬到 0.55 附近：靶被遮挡/合并时外轮廓会撑大、
                              #    q 会掉下来（红靶合并块 0.13），要留出这个余量。
                              # ⚠️ 被 q 判掉的候选**没有丢**：还有 _detect_frame 末尾那条
                              #    「没被共识覆盖的颜色退回该色最大候选」兜底，那颗红靶正是
                              #    靠它（合并块的内孔）救回来的。两道关分工不同，别合成一道。
TARGET_VOTE_FRAMES = 8        # 最多采帧数
TARGET_MIN_VOTES = 3          # 单色最少命中帧数（众数语义）
TARGET_MAX_SPREAD_PX = 40.0   # 同色 cx 跨帧最大散布[px]（超限=机位在动/误检 → 该色作废）
TARGET_MIN_TARGETS = 1        # 识别成功的最少靶数
                              # ⚠️ 定为 1 而不是 3：**哪颗是目标由单片机按二维码第二位挑**，
                              #    Pi 侧不知道任务码。而且瞄准时凑近某一个靶，另外两个可能
                              #    已经出画 —— 要求 3 颗会让"贴近打靶"这个正常姿态直接判失败。
                              #    代价：只有 ≥2 颗同时在画面里时，跨色尺寸一致性才起作用
                              #    （见 _pick_consistent 的 min_colors）。
TARGET_MAX_TARGETS = 3        # 回传目标数上限（与单片机端 TARGET_MAX_CNT 对齐，双端同步改）
TARGET_R_MAX = 200.0          # 候选**外圆等效半径上限[px]**（全分辨率坐标系；候选级判据）
                              # 物理依据：设计射击距离上真靶 r≈90~110（2026-10-02 实测
                              # r90~104），车凑到最近也就 ~150；而背景蓝色挡板/柜子的
                              # 连通域 r 数百 —— 一条与颜色无关的绝对尺寸线就能把
                              # 「兜底撞上背景大块」挡死（2026-10-02 蓝靶跨帧散布 1272px
                              # 作废的根因：粘连帧把共识淘汰掉的蓝色让给了兜底，兜底取
                              # 面积最大 = 背景块，圆心甩到背景上 → 整色作废）。
                              # ⚠️ 它不依赖色相阈值，换打印工具/换靶纸不影响 —— 与其收窄
                              #    色域（颜色不标准就坏），不如卡死「靶在画面里就这么大」。
                              # ⚠️ 定值方法：跑 tools/replay_target_tune.py --scan-rmax，
                              #    用真实 frames 看「真靶最大 r」与「背景块最小 r」的空档取中。
                              # ⚠️ 取大了挡不住背景（退回旧行为），取小了会误杀凑近时的
                              #    真靶 —— 三颗并排可同时入画的几何约束本身就压着 r 上限。

# ROI 裁切：瞄准闭环会把靶心拉到画面中心附近，「靶心应该在的地方」是个已知的小窗口，
# 窗口外的干扰（现场背景那面蓝柜子）根本不用去看。
# ⚠️ **默认关**：窗口位置得看着**真实瞄准位姿**定，配错了会静默地什么都找不到
#    （而不会报错）。等台架上确认靶心落点之后再开。
# ⚠️ 窗口形状要按现场排布定：三个靶**水平并排**、柜子在**上方背景** →
#    该用「全宽 + 下移的横带」，而不是居中正方形（正方形会把左右两个靶一起切掉，
#    而那两个正是跨色尺寸一致性的基准）。相对坐标，0~1。
TARGET_USE_ROI = 0            # 1 = 开；见上面两条警告
TARGET_ROI_X0 = 0.00
TARGET_ROI_Y0 = 0.40
TARGET_ROI_X1 = 1.00
TARGET_ROI_Y1 = 0.98

# 颜色码 → 中文名（与单片机端 color_t 对齐: 1红 2绿 3蓝，也等于二维码第一位）
TARGET_COLOR_NAMES = {1: "红", 2: "绿", 3: "蓝"}

# 靶标识别最后一帧保存路径（执行指令 "6" 后写入标注帧，Web 控制端预览显示用）
TARGET_LAST_FRAME_PATH = os.path.join(LOG_DIR, "target_last.jpg")

# ==================== 靶标流式跟踪（反恐区打靶, 命令 "7"/"8", 2026-09 加）====================
# 瞄准闭环的**测量源**: 单片机发 "7" 起流后, 本端在小跟踪窗内高频回传靶心
# （TAG=0x05 流帧, 见 docs/串口协议.md §6f）, 单片机以 ~10Hz 做 P 环微调舵机 6/3;
# 收敛/超时后发 "8" 停流。
#
# ⚠ 与指令 "6"（一次性全画面多帧投票）是**两套并存**的方案, 单片机端分别对应
#   STEP_TARGET_SERVO_WAIT（流控闭环）/ STEP_TARGET_CALIB_WAIT（开环迭代）。
#   "6" 的路径一行未动; 本文件里的 TARGET_* 参数两边共用（阈值/色相族/尺寸判据同源）。
#
# ⚠ 跟踪窗是**动窗**: 舵机动 → 靶心在画面里走几百 px, 窗必须跟着目标走, 而不是
#   锁定时的那个静态框。做法 = 各跟踪目标"上一帧位置 ± MARGIN"的并集 bbox。
# ⚠ **分辨率不降**: 靶外圆 r≈55px、内孔 r≈27px, 精度就靠全分辨率
#   （TARGET_PROC_SCALE 那条教训: 降到 0.5 时遮挡用例整颗漏检）。提速靠**裁**, 不靠**缩**。

TARGET_STREAM_HZ          = 30.0   # 发送/处理上限频率(Hz)，0=不限。2026-10-06 读帧线程双缓冲后可贴到相机 30fps 天花板（旧串行版 25 也只跑出 ~22）
TARGET_STREAM_INIT_FRAMES = 3      # 起流锁定的全画面采帧数（用 _detect_frame 单帧, 非多帧投票）
TARGET_STREAM_MARGIN_PX   = 150    # 跟踪窗边距[px]; 丢失后按 ×(1+连续丢失帧数) 逐帧扩张
TARGET_STREAM_R_TOL       = 0.35   # 半径门: 候选 r 必须落在上一帧 r×(1±tol) 内
TARGET_STREAM_USE_COLOR   = 1      # 1=按色相族跟踪(推荐); 0=无色模式(S/V 掩膜, 回传 color=0xFF)
TARGET_STREAM_MAX_S       = 12.0   # 流线程安全上限(秒), 防 "8" 丢失后一直占着相机
TARGET_STREAM_LOG_HZ      = 1.0    # 状态日志限流(Hz), 0=关。打 fps/窗尺寸/丢失数
TARGET_STREAM_LAST_FRAME_PATH = os.path.join(LOG_DIR, "target_stream_last.jpg")  # 周期性落帧(Web 预览用)

# ---- 标靶练习数据采集（2026-10 加, 配合 GKS_Web「标靶练习」页; 消费方在 tasks/_target_stream.py）----
# 两层门控: ARCHIVE_FRAMES 只在 VERBOSE=1 时生效 —— 详细日志关了, 帧序列归档连带禁用
# （练习/调试时两个都开, 比赛时两个都关, 日志与磁盘零开销）。
TARGET_STREAM_VERBOSE        = 0  # 详细日志: 1=闭环期间逐帧打 [靶标流#N] 行(各靶心/半径/窗, ~30行/s)。0=只有 1Hz 状态行
TARGET_STREAM_ARCHIVE_FRAMES = 0  # 帧序列归档: 1=按 TARGET_STREAM_LOG_HZ 节奏把闭环原始帧归档进 run 目录 frames/aimseq_*(每发几MB)。⚠ 需 VERBOSE=1

# ⚠⚠ **无色模式（TARGET_STREAM_USE_COLOR=0）的已知风险**（接受才用, 默认带色）:
#   另外两颗靶是**等大的圆环**, 无色时唯一的区分手段是距离门 —— 一次大修正跳到邻靶
#   就锁错, 而且锁错之后闭环还会**持续把车/臂往邻靶上拉**（比开环错得更远）。
#   另外蓝柜子那种等大彩色块会被 S/V 掩膜抓进候选, 只能靠半径门 + 距离门 + 窗挡住。
#   带色模式（默认）用颜色当第一道门, 这才是可靠的。
# ⚠ 无色模式在实现上就是**给色相族传满量程** `[(0, 179)]`（OpenCV 8 位 HSV 的 H 是
#   0~179）。别改成"传空族表" —— `_color_mask` 对空族表返回**全零掩膜**（一个候选都没有,
#   流会一直 count=0 而看不出哪里错了）, 那个参数的含义是「哪些色相段算这个颜色」,
#   不是「要不要按颜色筛」。

# ==================== Web 控制页 ====================
# camera_web.py 控制页（摄像头预览 + 二维码识别按钮）的默认端口。
# ⚠ 该页面**已废弃**：main.py 不再 start_server()，唯一入口是 GKS_Web 的 :9000。
# 本端口现在只在「手动独立调试」时用：python3 camera_web.py [--port 8080]。
WEB_CONTROL_PORT = 8000


# ==================== Web「设置参数」可编辑项 ====================
# (名称, 类型, 中文说明)；类型: "int" / "float" / "str"
EDITABLE_CONFIG = [
    ("QR_SCAN_TIMEOUT", "float", "二维码识别超时(秒,无指令时限时的兜底)"),
    ("QR_SCAN_CENTER_CROP", "float", "扫码中心裁剪比例(0=关)"),
    ("QR_MAIN_CENTER_CROP", "float", "主摄扫码中心裁剪比例(0=关)"),
    ("QR_CMD_MARGIN_S", "float", "单片机给超时里本端留的余量(秒)"),
    ("QR_CMD_MAX_TIMEOUT_S", "float", "本端预算上限(秒,护租约看门狗)"),
    ("QR_DIFF_TIMEOUT", "float", "自研扫码超时(秒,同上兜底)"),
    ("QR_DIFF_KNOWN", "str", "自研候选集(如 123=111~333)"),
    ("QR_DIFF_MAX_SIZE", "int", "自研扫码最大边长(px)"),
    ("QR_DIFF_CENTER_CROP", "float", "自研扫码中心裁剪比例(0=关)"),
    ("QR_DIFF_DEBUG_KEEP", "int", "自研失败留档保留份数"),
    ("HOSTAGE_CONF_THR", "float", "人质识别置信度阈值"),
    ("HOSTAGE_IOU_THR", "float", "人质识别NMS-IoU阈值"),
    ("HOSTAGE_TIMEOUT", "float", "人质识别超时(秒)"),
    ("HOSTAGE_MIN_TARGETS", "int", "人质识别成功最少目标数"),
    ("HOSTAGE_ANCHOR_CONF", "float", "人质排除法锚点置信度(0~1)"),
    ("OPENMV_LED_BRIGHTNESS", "int", "OpenMV补光亮度(0~100, 点亮值; 命令T只临时灭灯)"),
    ("COLOR_TIMEOUT", "float", "色球识别超时(秒)"),
    ("COLOR_PROC_SCALE", "float", "色球检测降采样比例"),
    ("COLOR_SAT_MIN", "int", "色球找球饱和度下限"),
    ("COLOR_VAL_MIN", "int", "色球找球亮度下限"),
    ("COLOR_MIN_AREA", "float", "色球最小连通域面积"),
    ("COLOR_MIN_CIRCULARITY", "float", "色球圆度下限"),
    ("COLOR_MIN_VOTES", "int", "色球单色最少命中帧数"),
    ("COLOR_MAX_SPREAD_PX", "float", "色球跨帧散布上限(px)"),
    ("COLOR_MIN_GAP_PX", "float", "色球中心最小间隔(px)"),
    ("COLOR_INFER_EN", "int", "缺球推断开关(1开)"),
    ("COLOR_INFER_MIN_BALLS", "int", "缺球推断最少检出球数"),
    ("COLOR_INFER_RETRY", "int", "推断前先多采一轮(1开)"),
    ("COLOR_BALL_U_CENTER", "float", "球位模型:中球 u"),
    ("COLOR_BALL_S", "float", "球位模型:相邻间距 u"),
    ("COLOR_BALL_S_TOL", "float", "球位模型:间距允许偏差"),
    ("COLOR_BALL_CENTER_TOL", "float", "球位模型:中线残差上限"),
    ("COLOR_BALL_U_MIN", "float", "球位模型:u 下限"),
    ("COLOR_BALL_U_MAX", "float", "球位模型:u 上限"),
    ("COLOR_BALL_AGREE_TOL", "float", "两路估计允许差"),
    ("COLOR_BALL_CX_CENTER", "float", "兜底用:理想中球列(px)"),
    ("COLOR_BALL_CX_TOL_PX", "float", "兜底用:理想列容差(px)"),
    ("RECT_TIMEOUT", "float", "矩形地板识别超时(秒)"),
    ("RECT_MIN_VOTES", "int", "矩形地板成功帧数下限"),
    ("RECT_MAX_SPREAD_PX", "float", "矩形地板中心跨帧散布上限(px)"),
    ("RECT_V_MAX", "int", "黑胶带亮度上限(V)"),
    ("RECT_S_MAX", "int", "黑胶带饱和度上限(S)"),
    ("RECT_CLOSE_K", "int", "暗掩膜闭运算核(px)"),
    ("RECT_MIN_LINE_LEN", "float", "长边最小长度(×画面宽)"),
    ("RECT_MIN_SHORT_LINE", "float", "短边最小长度(×画面宽)"),
    ("RECT_MIN_GAP_FRAC", "float", "两长边最小间距(×板宽)"),
    ("RECT_MIN_AREA_FRAC", "float", "四边形最小可见面积(占画面)"),
    ("RECT_MIN_INNER_V", "float", "框内平均亮度下限"),
    ("RECT_MIN_INNER_RATIO", "float", "框内亮度/画面中位亮度下限"),
    ("RECT_HW_RATIO", "float", "长边:短边像素比(三边估中心用)"),
    ("RECT_INNER_CLOSE", "int", "围住区掩膜闭运算核(px)"),
    ("RECT_EPS_FRAC", "float", "多边形拟合精度(×周长)"),
    ("RECT_MIN_FILL", "float", "四边形填充率下限"),
    ("RECT_TAPE_NEAR_PX", "float", "边框外偏判胶带距离(px)"),
    ("RECT_MAX_ASPECT", "float", "四边形长宽比上限"),
    ("RECT_MIN_ASPECT", "float", "四边形长宽比下限"),
    ("RECT_OUTLIER_K", "float", "多帧离群倍数(MAD)"),
    ("RECT_OUTLIER_MIN_PX", "float", "多帧离群下限(px)"),
    ("ROAD_TIMEOUT", "float", "看路识别超时(秒)"),
    ("ROAD_VOTE_FRAMES", "int", "看路最多采帧数"),
    ("ROAD_MIN_VOTES", "int", "看路成功帧数下限"),
    ("ROAD_MAX_SPREAD_PX", "float", "看路Δx跨帧散布上限(px)"),
    ("ROAD_OUTLIER_K", "float", "看路多帧离群倍数(MAD)"),
    ("ROAD_OUTLIER_MIN_PX", "float", "看路多帧离群下限(px)"),
    ("ROAD_PROC_SCALE", "float", "看路降采样比例"),
    ("ROAD_ROI_TOP", "float", "看路ROI上(相对)"),
    ("ROAD_ROI_BOTTOM", "float", "看路ROI下(相对,须盖住黄区左边缘)"),
    ("ROAD_ROI_LEFT", "float", "看路ROI左(相对)"),
    ("ROAD_ROI_RIGHT", "float", "看路ROI右(相对)"),
    ("ROAD_WARM_FLOOR_YB", "int", "看路Y-B阈值地板(自适应阈值的下限)"),
    ("ROAD_PEAK_FRAC", "float", "看路谷底法黄区峰显著度(峰高>=主峰x它)"),
    ("ROAD_WARM_V_MIN", "int", "看路黄区亮度下限(V)"),
    ("ROAD_WARM_OPEN_K", "int", "看路黄区开运算核(px)"),
    ("ROAD_WARM_CLOSE_K", "int", "看路黄区闭运算核(px)"),
    ("ROAD_WARM_MIN_AREA", "int", "看路黄区最小面积(px2)"),
    ("ROAD_DX_MIN_PX", "float", "看路Δx下限(px)"),
    ("ROAD_DX_MAX_PX", "float", "看路Δx上限(px)"),
    ("TARGET_TIMEOUT", "float", "靶标识别超时(秒)"),
    ("TARGET_PROC_SCALE", "float", "靶标检测降采样比例"),
    ("TARGET_SAT_MIN", "int", "靶标饱和度下限"),
    ("TARGET_VAL_MIN", "int", "靶标亮度下限"),
    ("TARGET_MIN_AREA", "float", "靶标最小连通域面积(降采样后)"),
    ("TARGET_TOL", "float", "靶标跨色尺寸一致倍数上限"),
    ("TARGET_RING_Q_MIN", "float", "靶标靶环内外径比下限"),
    ("TARGET_RING_Q_MAX", "float", "靶标靶环内外径比上限"),
    ("TARGET_VOTE_FRAMES", "int", "靶标最多采帧数"),
    ("TARGET_MIN_VOTES", "int", "靶标单色最少命中帧数"),
    ("TARGET_MAX_SPREAD_PX", "float", "靶标跨帧散布上限(px)"),
    ("TARGET_MIN_TARGETS", "int", "靶标识别成功最少靶数"),
    ("TARGET_MAX_TARGETS", "int", "靶标回传目标数上限"),
    ("TARGET_R_MAX", "float", "靶标外圆半径上限(px,挡背景大块)"),
    ("TARGET_USE_ROI", "int", "靶标ROI裁切开关(1开/0关)"),
    ("TARGET_ROI_X0", "float", "靶标ROI左(相对)"),
    ("TARGET_ROI_Y0", "float", "靶标ROI上(相对)"),
    ("TARGET_ROI_X1", "float", "靶标ROI右(相对)"),
    ("TARGET_ROI_Y1", "float", "靶标ROI下(相对)"),
    ("TARGET_STREAM_HZ", "float", "靶标流发送频率上限(Hz)"),
    ("TARGET_STREAM_MARGIN_PX", "int", "靶标流跟踪窗边距(px)"),
    ("TARGET_STREAM_R_TOL", "float", "靶标流半径门(0~1)"),
    ("TARGET_STREAM_USE_COLOR", "int", "靶标流颜色跟踪开关(1带色/0无色)"),
    ("TARGET_STREAM_MAX_S", "float", "靶标流安全时长上限(秒)"),
    ("TARGET_STREAM_VERBOSE", "int", "标靶练习详细日志(1开/0关)"),
    ("TARGET_STREAM_ARCHIVE_FRAMES", "int", "练习帧序列归档(1开/0关,需详细日志开)"),
    ("FRAME_WIDTH", "int", "摄像头分辨率宽"),
    ("FRAME_HEIGHT", "int", "摄像头分辨率高"),
    ("CAM_MAIN_FOURCC", "str", "主摄像头像素格式"),
    ("CAM_QR_FOURCC", "str", "扫码摄像头像素格式"),
    ("CAM_MAIN_AUTO_EXPOSURE", "int", "主摄曝光模式(3自动/1手动)"),
    ("CAM_MAIN_EXPOSURE", "int", "主摄手动曝光(100µs,100=10ms)"),
    ("CAM_QR_AUTO_EXPOSURE", "int", "扫码相机曝光模式(3自动/1手动)"),
    ("CAM_QR_EXPOSURE", "int", "扫码相机手动曝光(100µs)"),
    ("HOSTAGE_AUTO_EXPOSURE", "int", "人质曝光模式(3自动/1手动)"),
    ("HOSTAGE_EXPOSURE", "int", "人质手动曝光(100µs,独立于主摄)"),
    ("CAM_MAIN_AUTO_WB", "int", "主摄白平衡(1自动/0固定)"),
    ("CAM_MAIN_WB_TEMPERATURE", "int", "主摄固定色温(K,2800~6500)"),
    ("CAM_QR_AUTO_WB", "int", "扫码相机白平衡(1自动/0固定)"),
    ("CAM_QR_WB_TEMPERATURE", "int", "扫码相机固定色温(K)"),
    ("HOSTAGE_AUTO_WB", "int", "人质白平衡(1自动/0固定)"),
    ("HOSTAGE_WB_TEMPERATURE", "int", "人质固定色温(K)"),
    ("ROAD_AUTO_WB", "int", "看路白平衡(1自动/0固定)"),
    ("ROAD_WB_TEMPERATURE", "int", "看路固定色温(K,与Y-B判据配套)"),
    ("LEASE_IDLE_TIMEOUT_S", "float", "相机租约空闲释放(秒,也是每次使用的窗口)"),
    ("LEASE_WARM_WAIT_S", "float", "等预热落地上限(秒)"),
    ("LEASE_USES_MAX", "int", "预热次数上限(窗口最多放宽到它×上述秒数)"),
    ("LEASE_STALE_S", "float", "租约帧新鲜度阈值(秒,超时判读线程停摆)"),
    ("LINE_KP", "float", "巡线PID-Kp"),
    ("LINE_KI", "float", "巡线PID-Ki"),
    ("LINE_KD", "float", "巡线PID-Kd"),
    ("LINE_ERR_SIGN", "float", "巡线误差方向(1/-1)"),
    ("LINE_SETPOINT", "float", "巡线目标偏移(offset单位)"),
    ("LINE_SEND_HZ", "float", "巡线转发频率(Hz)"),
    ("LINE_MAX_DURATION", "float", "巡线最长时长(秒)"),
    ("LINE_LOST_HOLD_S", "float", "丢线保持时间(秒)"),
    ("LINE_LOST_CONFIRM_S", "float", "丢线定向找线确认期(秒)"),
    ("LINE_LOST_SEARCH_S", "float", "丢线找线时长(秒)"),
    ("LINE_SEARCH_STEER", "float", "找线steer(负号=倒向)"),
    ("LINE_LOG_HZ", "float", "巡线原始值打印频率(Hz,0=关)"),
    ("SYSTEM_CMD_CONFIRM_S", "float", "关机/重启二次确认窗口(秒)"),
]

# Web 端保存的参数覆盖持久化文件（启动时加载，运行时即时生效）
_OVERRIDE_FILE = os.path.join(BASE_DIR, "config_override.json")
_EDITABLE_TYPES = {name: typ for name, typ, _desc in EDITABLE_CONFIG}


def _coerce(name, value):
    """按类型把字符串值转换成目标类型，失败抛 ValueError。

    ⚠️ 2026-10: 数值型额外拒收 NaN/Inf —— float("nan") 在 Python 里是合法转换,
    但它会顺着 LINE_KP 之类一路流进控制律: int(round(nan*10000)) 直接抛 ValueError
    杀死巡线线程, min/max 还会把 NaN 积分静默钳成满幅; json.dump 更会写出非标准的
    NaN 字面量毒化持久化文件。在入口拒掉是最便宜的一道闸。
    """
    typ = _EDITABLE_TYPES.get(name)
    if typ == "int":
        v = int(value)
        if not math.isfinite(v):        # int 理论上不会非有限, 防 bool/int 子类怪值
            raise ValueError("非有限值")
        return v
    if typ == "float":
        v = float(value)
        if not math.isfinite(v):
            raise ValueError("数值必须是有限数（拒绝 nan/inf）")
        return v
    return str(value)


def load_overrides():
    """启动时加载 Web 端保存的参数覆盖，应用到当前进程（运行时生效）。

    ⚠️ 2026-10: 文件读不出来/解析失败不再**静默** —— 那是「现场标定无声全丢、
    行为突变且零线索」的事故形态（真踩过评估）。现在: ① 打响亮的 WARN; ② 把坏
    文件改名成 .bad 备份（保住现场调的参数供手工抢救）, 免得下一次 save 用空 {}
    起步把它彻底覆盖掉。
    ⚠️ 只对「文件存在但读不懂」做备份; 文件不存在是正常首次运行, 不响。
    """
    if not os.path.exists(_OVERRIDE_FILE):
        return
    try:
        with open(_OVERRIDE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError(f"顶层不是 dict 而是 {type(data).__name__}")
    except Exception as e:
        backup = _OVERRIDE_FILE + ".bad"
        try:
            os.replace(_OVERRIDE_FILE, backup)
            print(f"[ERROR] config_override.json 损坏({e}) —— 已备份到 {backup}, "
                  f"本次按 config.py 默认值运行! 手工从备份里抢回参数后删掉 .bad。")
        except OSError:
            print(f"[ERROR] config_override.json 损坏({e})且无法备份 —— "
                  f"本次按 config.py 默认值运行（现场标定可能已失效, 注意核对）!")
        return
    for name, _typ, _desc in EDITABLE_CONFIG:
        if name in data:
            try:
                globals()[name] = _coerce(name, data[name])
            except (ValueError, TypeError):
                print(f"[WARN] 覆盖参数 {name}={data[name]!r} 非法, 保持默认值")


def save_override(name, value):
    """保存单个参数：运行时即时生效 + 持久化到 config_override.json。

    返回 (ok, message)。
    ⚠️ 2026-10: ① 落盘改为**临时文件 + os.replace 原子替换** —— 直接 "w" truncate
    写一半掉电/进程被杀会留半截文件, load 端旧版静默吞掉 = 所有现场标定无声丢失;
    ② 读旧文件失败时不再用 {} 起步抹掉其它参数, 而是拒绝保存（让操作员先处理坏文件）。
    """
    try:
        coerced = _coerce(name, value)
    except (ValueError, TypeError) as e:
        return False, f"{name} 的值 {value!r} 无法保存（{e}）"
    globals()[name] = coerced
    try:
        with open(_OVERRIDE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("顶层不是 dict")
    except FileNotFoundError:
        data = {}
    except Exception as e:
        return False, (f"现有 config_override.json 读不了({e}) —— 为避免覆盖丢参数, "
                       f"本次不保存。看启动日志里的 .bad 备份提示先抢救。")
    data[name] = coerced
    tmp = _OVERRIDE_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())        # 树莓派上防掉电丢元数据
        os.replace(tmp, _OVERRIDE_FILE)  # 原子替换: 要么旧文件完好, 要么新文件完整
    except Exception as e:
        try:
            os.remove(tmp)
        except OSError:
            pass
        return False, f"持久化失败: {e}"
    return True, f"{name} 已更新为 {coerced}"


# 启动时应用 Web 端保存的覆盖
load_overrides()
