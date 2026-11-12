# config.py — GKS 工科赛 全局参数
import json
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

# 摄像头生命周期: "on_demand"=按需开关, "persistent"=常驻复用
CAMERA_LIFECYCLE = "on_demand"

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

# PID 参数（Web 设置页可在线调，转发循环每帧动态读取）
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
#   OpenMV 帧第 5 字段 state 报丢线方向（2=边界远出画面 / 3=压到黄区 / 4=不明）。
#   state=2/3：先静默 LINE_LOST_CONFIRM_S（躲单帧毛刺），其后**按 state 定向找线**
#   —— steer 大小取 |LINE_SEARCH_STEER|，方向由 state + LINE_ERR_SIGN 推导（不再死写）。
#   state=4/旧固件：走旧三段（保持 → 固定方向找线 → 停），此时 LINE_SEARCH_STEER 的
#   **符号**才还参与（兜底猜测）。两路都受 HOLD+SEARCH 总预算封顶，超时停（防跑飞）。
LINE_LOST_HOLD_S = 5.0      # 丢线后沿用「最后有效 steer」的秒数：短暂丢线不再打断动作
                            # （丢线率一高，清零会让车在「横移↔停车」间反复跳，几乎不前进）
                            # 0 = 丢线立刻停（旧行为）。方向明确(state=2/3)时它只当总预算的一部分
LINE_LOST_CONFIRM_S = 0.4   # 定向找线确认期（秒）：方向明确的丢线先静默这么久再动，
                            # 单帧毛刺（运动模糊/曝光抖动）在这里就被吸收掉，不会误触发找线
LINE_LOST_SEARCH_S = 0.0    # 找线段时长（秒，从最后有效帧起算的总预算 = HOLD_S+本值）；
                            # 0 = 不找线，直接停
LINE_SEARCH_STEER = 0.0     # 找线速度的**大小**（steer 单位; 单片机侧 ×LINE_STEER_TO_VX 得 rpm）
                            # 符号仅在「方向不明的兜底找线」里还有效; 定向找线只取 |本值|,
                            # 方向 = state + LINE_ERR_SIGN 推导（见 tasks/_line.py::_toward_steer）
                            # 例：单片机 LINE_STEER_TO_VX=30 时填 ±0.4 → 12rpm 慢速横移找线
                            # 两处用它: ① 丢线找线段 ② offset 满偏(±100, 交界线贴画面
                            # 边缘)时直接改用它, 方向跟 err 符号
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
# 单次识别超时（秒），超时或被打断返回失败
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

# 扫码最后一帧保存路径（main.py 执行命令「1」后写入标注帧，Web 控制端预览显示用）
QR_LAST_FRAME_PATH = os.path.join(LOG_DIR, "qr_last.jpg")

# 微信二维码引擎（OpenCV wechat_qrcode）CNN 模型目录
WECHAT_QR_MODEL_DIR = os.path.join(BASE_DIR, "models", "wechat_qrcode")

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
COLOR_FIT_MAX_DIST_PX = 10.0  # 三球共线质检上限[px]: 质心到 fitLine 直线最大距离超它 → fit_ok=0。⚠️别收紧回 4: 场地摆球公差 ±10mm ≈ ±10px 纵向偏差是**静态的**(投票平均不掉), 实测绿球低 13px 就把 4px 阈值顶爆、对位整段拒动; fitLine 本身是最小二乘, 单球 10px 偏差带来的角度噪声有界(~0.7°图像角), 放宽比拒动划算

COLOR_TIMEOUT = 5.0           # 单次识别超时（秒）—— 须 < 单片机 RPI_DISCONNECT_TIMEOUT_MS(8s)
COLOR_MIN_TARGETS = 3         # 凑满该数量才算成功（比赛 3 个球）
COLOR_MAX_TARGETS = 3         # 回传目标数上限（与单片机端 COLOR_MAX_COUNT 对齐, 双端同步改）

# 颜色码 → 中文名（日志/标注/Web 显示用；与单片机端 color_t 对齐: 1红 2绿 3蓝，
# 也等于二维码第一位）
COLOR_COLOR_NAMES = {1: "红", 2: "绿", 3: "蓝"}

# 色球识别最后一帧保存路径（执行指令 "5" 后写入标注帧，Web 控制端预览显示用）
COLOR_LAST_FRAME_PATH = os.path.join(LOG_DIR, "color_last.jpg")

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

TARGET_STREAM_HZ          = 25.0   # 发送/处理上限频率(Hz)，0=不限。实测达不到就按实际跑
TARGET_STREAM_INIT_FRAMES = 3      # 起流锁定的全画面采帧数（用 _detect_frame 单帧, 非多帧投票）
TARGET_STREAM_MARGIN_PX   = 150    # 跟踪窗边距[px]; 丢失后按 ×(1+连续丢失帧数) 逐帧扩张
TARGET_STREAM_R_TOL       = 0.35   # 半径门: 候选 r 必须落在上一帧 r×(1±tol) 内
TARGET_STREAM_USE_COLOR   = 1      # 1=按色相族跟踪(推荐); 0=无色模式(S/V 掩膜, 回传 color=0xFF)
TARGET_STREAM_MAX_S       = 12.0   # 流线程安全上限(秒), 防 "8" 丢失后一直占着相机
TARGET_STREAM_LOG_HZ      = 1.0    # 状态日志限流(Hz), 0=关。打 fps/窗尺寸/丢失数
TARGET_STREAM_LAST_FRAME_PATH = os.path.join(LOG_DIR, "target_stream_last.jpg")  # 周期性落帧(Web 预览用)

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
    ("QR_SCAN_TIMEOUT", "float", "二维码识别超时(秒)"),
    ("QR_SCAN_CENTER_CROP", "float", "扫码中心裁剪比例(0=关)"),
    ("HOSTAGE_CONF_THR", "float", "人质识别置信度阈值"),
    ("HOSTAGE_IOU_THR", "float", "人质识别NMS-IoU阈值"),
    ("HOSTAGE_TIMEOUT", "float", "人质识别超时(秒)"),
    ("HOSTAGE_MIN_TARGETS", "int", "人质识别成功最少目标数"),
    ("COLOR_TIMEOUT", "float", "色球识别超时(秒)"),
    ("COLOR_PROC_SCALE", "float", "色球检测降采样比例"),
    ("COLOR_SAT_MIN", "int", "色球找球饱和度下限"),
    ("COLOR_VAL_MIN", "int", "色球找球亮度下限"),
    ("COLOR_MIN_AREA", "float", "色球最小连通域面积"),
    ("COLOR_MIN_CIRCULARITY", "float", "色球圆度下限"),
    ("COLOR_MIN_VOTES", "int", "色球单色最少命中帧数"),
    ("COLOR_MAX_SPREAD_PX", "float", "色球跨帧散布上限(px)"),
    ("COLOR_MIN_GAP_PX", "float", "色球中心最小间隔(px)"),
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
    ("LINE_SEARCH_STEER", "float", "找线steer(符号=方向)"),
    ("LINE_LOG_HZ", "float", "巡线原始值打印频率(Hz,0=关)"),
    ("SYSTEM_CMD_CONFIRM_S", "float", "关机/重启二次确认窗口(秒)"),
]

# Web 端保存的参数覆盖持久化文件（启动时加载，运行时即时生效）
_OVERRIDE_FILE = os.path.join(BASE_DIR, "config_override.json")
_EDITABLE_TYPES = {name: typ for name, typ, _desc in EDITABLE_CONFIG}


def _coerce(name, value):
    """按类型把字符串值转换成目标类型，失败抛 ValueError。"""
    typ = _EDITABLE_TYPES.get(name)
    if typ == "int":
        return int(value)
    if typ == "float":
        return float(value)
    return str(value)


def load_overrides():
    """启动时加载 Web 端保存的参数覆盖，应用到当前进程（运行时生效）。"""
    try:
        with open(_OVERRIDE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return
    for name, _typ, _desc in EDITABLE_CONFIG:
        if name in data:
            try:
                globals()[name] = _coerce(name, data[name])
            except (ValueError, TypeError):
                pass


def save_override(name, value):
    """保存单个参数：运行时即时生效 + 持久化到 config_override.json。

    返回 (ok, message)。
    """
    try:
        coerced = _coerce(name, value)
    except (ValueError, TypeError):
        return False, f"{name} 的值 {value!r} 无法转换为 {_EDITABLE_TYPES.get(name)}"
    globals()[name] = coerced
    try:
        with open(_OVERRIDE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = {}
    data[name] = coerced
    try:
        with open(_OVERRIDE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        return False, f"持久化失败: {e}"
    return True, f"{name} 已更新为 {coerced}"


# 启动时应用 Web 端保存的覆盖
load_overrides()
