# CLAUDE.md

本文件为 Claude Code 在此仓库中工作时提供指导。

## 项目概况

GKS 工科赛 项目骨架。目前**仅含通用基础设施**（主程序生命周期、控制台、systemd 服务），
**尚未移植任何任务/视觉/串口逻辑**（参考姊妹项目 `/home/xixi/cv_python`，那是 GCS
机械臂视觉引导系统的完整实现）。

运行在树莓派上，通过触摸屏控制台 + systemd 服务管理主程序。

## 文件说明

- `main.py` — 主程序：日志双写、无头运行(SERVICE_MODE)、优雅退出；启动 OpenMV 读取线程后
  进入串口接收/指令分发主循环（`SerialInterruptHandler`）。**不再启动 Web 控制页**（见下）。
- `protocol.py` — 串口帧协议（SOF|TYPE|LEN|DATA|CRC8|EOF），交付单片机端的契约源，
  详见 `docs/串口协议.md`。
- `serial_comm.py` — 串口通信：同时等待「真实串口」与「模拟器虚拟串口」两个信号，
  谁先就绪就进哪个模式（正常/模拟，无静态开关）。另含链路故障自愈用的
  `restart_process()` / `note_link_ok()`（原地 `os.execl` 重启，见「链路故障自愈」节）。
- `openmv_serial.py` — OpenMV 寻线传感器常驻读取线程（模块级单例 `openmv_reader`）：
  读 `/dev/ttyUSB_OpenMV` 文本帧 `L,<offset>,<angle>,<valid>` 并缓存最新值；断线静默重试。
  OpenMV 按需发送（`send_command(b'G'/b'S')`）；「发了 'G' 却没数据」会限次告警，见「串口」节。
- `handler.py` — 指令分发：命令注册表 `_COMMANDS` + `SerialInterruptHandler`（握手 + 收帧分发）。
  巡线命令 `"2"`/`"3"` 起/停独立转发线程（`_cmd_line_start`/`_cmd_line_stop`）。
  ⚠ `run()` 退出分两种：真被叫停才退进程，链路故障则原地重启（见「链路故障自愈」节）。
- `control_panel.py` — 800x480 触摸屏控制台（PyQt5）：网络/SSH、服务启停/自启、
  main.py 启停、日志查看、控制台自启、重启/关机。
- `config.py` — 全局参数：项目路径、服务名、日志目录、心跳间隔、摄像头参数。
- `camera.py` — 摄像头管理：`CameraManager.open/read/release`（移植自 cv_python 的
  V4L2 初始化 + 预热校验核心逻辑）+ `open_camera(is_qr)` 便捷接口。
- `web_preview.py` — Web 实时预览框架（MJPEG over HTTP）。统一的调试预览方案，
  替代 `cv2.imshow`/Qt 窗口。`preview.publish(name, frame)` / `preview.add_camera(name, cap)`。
- `camera_web.py` — **已废弃的** Web 控制页（摄像头预览 + 二维码识别按钮）。main.py 不再
  `start_server()`；唯一入口是 GKS_Web（`:9000`，`GKS_Web/app.py`，已覆盖本页全部功能含参数设置）。
  文件保留的两个理由：① `handler.py` 的 `_stop_main_stream()` 要
  `from camera_web import _stop_stream`（人质/色球识别前释放主摄）——**别删文件**；
  ② 单独调试可 `python3 camera_web.py --port 8000`。⚠ 别在 main.py 里恢复 `start_server()`。
  另有意的功能损失：`/line_status` `/line_start` `/line_stop` 三条巡线调试路由不再提供
  （它们当年能工作是因为页面就在 main.py 进程里；GKS_Web 是另一进程，`import handler`
  拿到的是没有串口链路的新模块）。台架看数改看日志里 5Hz 的 `[巡线] 偏差=.. 航向角=.. 有效=..`。
- `tasks/_qr.py` — 二维码识别任务：OpenCV / pyzbar 两种方法各一个函数
  （`run_qr_opencv` / `run_qr_pyzbar`），5s 超时、三位数校验、支持打断。
- `tasks/_line.py` — 巡线转发任务：`run_line_follow` 循环读 OpenMV 最新值 → 归一化 →
  `LinePID` → 以 TYPE_COORD 巡线帧（DATA[0]=0x01 标记，8B）实时发单片机（默认 30Hz）。
  PID 参数每帧从 config 动态读（Web 设置页在线调）。
  目标位置：`err = LINE_ERR_SIGN × ((OpenMV offset − LINE_SETPOINT) / 100)`。
  `LINE_SETPOINT`（默认 0，Web 可热调）就是"理想值"—— 0 = 停在相机零点（画面正中），
  非 0 = 稳定偏在线某一侧（单位同 offset，名义 ±100；相机端也有个等效旋钮 `TARGET_OFFSET`，
  单位是像素，**两边只改一处**，都改会叠加）。
  丢线策略：`err` 恒置 0（协议约定 valid=0 时 err 无意义），`steer` 分三段 ——
  丢线后 `LINE_LOST_HOLD_S`（默认 0.5s）内**沿用最后有效 steer**（丢线率一高，清零会让
  单片机在「横移↔停车」间反复跳、车几乎不前进），其后 `LINE_LOST_SEARCH_S` 内下发固定
  `LINE_SEARCH_STEER` 找线（0=不找），之后才停（防跑飞）。
  ⚠️ **启动盲期单列一档（2026-09 修，别再当丢线）**：OpenMV 按需发送（收 'G' 才发），
  刚起线程那几帧 `openmv_reader` 缓存里是**上一次运行**留下的快照（`fresh=False`）→
  此时 **steer 恒 0**（帧照发、`valid=0`），等第一帧新鲜数据到了才谈保持/找线。
  旧版把它当丢线，于是 `last_steer is None` + `lost_s≈0` 直接落到「兜底找线」、下发
  **固定方向的满幅 `LINE_SEARCH_STEER`**，而单片机 `valid=0` 时**照常执行 steer** →
  **每一局开场车都朝一个方向猛横移一下**（日志里每次「[巡线] …⚠数据过期」紧跟的那一下）。
  判据是 `fresh`（本次有没有拿到过新鲜数据），**不是**缓存里的 `data["valid"]`
  （实测有 `有效=1` 的陈帧）。`lost_s` 也从第一帧新鲜数据才起算，不把盲期算进去。
  「起跑就偏」的两种情形（**已有新鲜数据之后**）照旧直接走找线：① **从没见过线**
  （`last_steer is None`，例如一开始 offset 就是 ±100 而 valid=0）→ 保持期没有值可保持，
  直接从找线段开始；② `|err| ≥ 1` 或 **offset 钉死在 ±100 且 err 指向回程**（满偏 = 交界线
  贴到画面边缘。⚠️ SETPOINT 不在画面中央时, 靠近满量程那一侧的 err 可能到不了 1 ——
  SETPOINT=-65 时灰侧上限 0.35 → steer 上限 0.246 ≈ 7rpm, 车偏得再远误差也不涨, 即
  2026-10-05 实测的「灰区修正慢」；钉边触发补的就是这个, 触发后同样满幅找线速度、方向跟
  err 符号。SETPOINT 恰标在 ±100 上时 err≈0, 刻意不触发——没有"回程方向"可言）→
  **直接改用找线速度**、方向跟 err 符号，不再指望被 KP 压小的 PID 输出。
  ⚠️ OpenMV 真没接/没上电时车会**一直静止**（旧版朝一个凭空的固定方向盲找线 10s）——
     有意的：一点传感器信息都没有时不动车（日志里每秒一条 ⚠OpenMV未连接/⚠数据过期）。
  ⚠️⚠️ **单片机侧在 `valid=0` 时是「照常执行 steer」**（`app_line.c`：丢线策略全在 Pi,
     固件只做 steer→vx 映射 + 限幅; 另有 `LINE_STALE_TIMEOUT_MS` 帧号新鲜度兜底），
     **所以 Pi 下发的每一个 steer 都真的会让车动**，不是空转。旧版这里写反了
     （写成"单片机无条件停车并忽略 steer"）—— 正是那条错认知让上面的"开场猛横移"看着无害。
  启动后按 `config.LINE_LOG_HZ`（默认 5Hz，0=关）**限流**打印 OpenMV 原始读数
  （`[巡线] 偏差=.. 航向角=..(0.1度) 有效=..`，归一化/PID 之前的值，用于确认数据在流）。
  未连接/过期会额外标 `⚠OpenMV未连接` / `⚠数据过期` —— 缓存快照里的 `valid` 是 OpenMV
  自报的，未必等于实际下发的 valid，不标就会在日志里误读成「数据正常」。
- `tasks/_hostage.py` — 人质形状识别任务（命令 `"4"`）：`run_hostage_detect` 用主摄像头跑
  YOLO（cv2.dnn ONNX，模型/类名懒加载缓存），凑满 `HOSTAGE_MIN_TARGETS`（默认 3）即成功，
  目标按中心 x 升序（图像从左到右）；`build_result_frame` 打包 TYPE_COORD 结果帧
  （DATA[0]=0x02 标记，[status][count][shape/cx/cy]×N，见 docs/串口协议.md §6c）。
  类别映射按类名（`config.HOSTAGE_NAME2SHAPE`：cylinder→1 / cone→2 / pellet→3），
  不按类 ID（ID 随训练顺序变）。模型在 `gks_yolo_model/`（best.onnx + classes.names），
  ⚠ `HOSTAGE_INPUT_SIZE` 必须与 ONNX 导出时的 --imgsz 完全一致（静态输入尺寸）。
- `tasks/_target.py` — 靶标识别 + 靶心定位任务（命令 `"6"`，反恐区打靶）：`run_target_detect`
  用主摄像头找三色靶盘并定位**每颗的圆心**；`build_target_frame` 打包 TYPE_COORD 结果帧
  （DATA[0]=0x04，`[status][count][color/cx/cy/r]×N`，见 docs/串口协议.md §6e）。
  **回传三颗，哪颗是目标由单片机按二维码第二位挑**（与色球同构，Pi 不掺和任务语义）。
  算法来自 GongKeCompetition 仓库 `tools/target_offline/` 的离线评估，四条判据：
  ⓪ **靶环内外径比 q**（`TARGET_RING_Q_MIN/MAX`，2026-09 加，最先执行）—— q = 内孔拟合
  半径 ÷ 该候选外接圆半径，靶盘设计值 0.55。现场实测真靶环 0.52~0.55、一切杂块 ≤0.23
  （蓝柜子/蓝箱/木椅腿/背景碎块），两者不重叠。**这是治「蓝靶有时指向画面角落一个 r≈50
  的杂块」的那一道**：判据①只是必要条件，三个杂块凑巧一样大就能拼出一个合法的"尺寸一致
  组"把真靶挤掉，而 q 只看**单个候选自己的形状**、跟别的候选没关系，所以骗不过它。
  ⚠️ 被 q 判掉的候选**没丢**：合并块（红靶与木椅腿 q=0.13）由 `_detect_frame` 末尾
  「没被共识覆盖的颜色退回该色最大候选」救回，那个合并块的内孔正是靶的白心。两道关别合并。
  ① **跨色尺寸一致性**选靶 —— 治「同色大块」（现场背景整面蓝柜子，实测蓝柜子外接圆半径
  455px vs 靶 55px）。⚠️ 做法必须是「在候选里挑一组尺寸匹配的」，**不能**是「各色取最大再
  校验」（按最大取，蓝选出来就是柜子，校验只能说"不对"、给不出正确的蓝靶）。
  ② **内边界优先、外边界兜底** —— 实拍量得外缘是手工贴的波浪边（拟合椭圆残差 std 2.15px）、
  内边界是干净的圆（0.44px），所以圆心以内边界为准；不依赖同伴，单靶也成立。
  ③ **ROI 裁切**（`TARGET_USE_ROI`，默认关）—— 瞄准闭环会把靶心拉到画面中心附近。
  ⚠️ `TARGET_PROC_SCALE` 默认 **1.0 别调小**：实测 0.5 倍时「遮挡左 1/4 盘面」整颗漏检。
  ⚠️ 圆度关卡必须用**半径的 MAD**，不能用全部轮廓点的 RMS —— 遮挡物的直边正是鲁棒拟合
  故意剔掉的离群点，用 RMS 会把一个拟合误差仅 0.2px 的正确圆判掉（真踩过）。
  ⚠️⚠️ **性能（2026-09，别改回去）**：这三条判据要逐候选跑，而一帧有 15~30 个候选 × 3 色，
  实现方式直接决定能不能用。三处关键：
    · **逐候选的掩膜只在它自己的包围盒里做**（`connectedComponentsWithStats` 的 bbox 切片），
      **不要**对每个候选做一次整帧 1920×1080 的 `(labels==lab)` + findContours ——
      实测那一步独占 **450ms**（整帧耗时的 65%），改 bbox 后降到 12ms。
      代价：候选里要带 `ox`/`oy`，由 `_locate_one` 加回全帧坐标（别忘）。
    · **HSV 每帧只转一次**给三色共用（原先前三色各转一次），掩膜用 `cv2.inRange` 而不是
      numpy 比较（后者每次 churn ~12MB 临时数组）。
    · **内孔拟合复用已经在手上的 CCOMP 轮廓树**（`fit_hole_in`），不再为找孔重跑一遍
      findContours。审计它们等价的是 `test_ring_q.py::TestHoleFitHelpersAgree`。
  实测：整帧 **~1.0s → ~0.22s**（8 帧 2.2s，全分辨率），现场连续 27 次识别全中三颗。
  日志里那行「靶标采帧 N 帧，用时..s」就是给 `TARGET_TIMEOUT`/`VOTE_FRAMES` 定标用的。
- `tasks/_target_stream.py` — 靶标**流式**跟踪任务（命令 `"7"` 起流 / `"8"` 停流，2026-09 加）。
  与上面的 `_target.py`（命令 `"6"`，一次性全画面多帧投票）**并存**，是「标靶流控闭环」的
  测量源：识别一次锁定目标后，在**小跟踪窗**内 ~20Hz 回传靶心（TYPE_COORD 流帧
  DATA[0]=0x05，与 §6e 的 0x04 **逐字节同布局**，见 docs/串口协议.md §6f），
  供单片机以 ~10Hz 做 P 环微调舵机 6/3；收敛/超时后单片机发 `"8"` 收流。
  `build_stream_frame` 打包（⚠️ 函数名不能叫 build_result_frame / build_target_frame，
  那两个已被占用，经 `tasks/__init__.py` 重新导出会互相覆盖）。
  **阈值/色相族/尺寸判据全部复用 `_target.py`**（私有几何件直接 import ——
  「什么算红」只留一个出处；改名会 ImportError，是响亮的失败不是静默失效）。
  三条实现要点：
    · **动窗**：跟踪窗 = 各目标"上一帧位置 ± `TARGET_STREAM_MARGIN_PX`"的并集 bbox，
      **必须跟着目标走**（舵机动一下靶心就走几百 px）；丢失时按 ×(1+连续丢失帧数) 逐帧
      扩张、封顶半幅 → 自然退化成全画面重搜，不必另写"重新锁定"。窗另有最小边长兜底
      （`_MIN_WIN_PX`）—— 目标停在窗边缘时窗缩到它身上会裁掉半边轮廓、拟合直接崩。
    · **提速靠「裁」不靠「缩」**：分辨率全程 `TARGET_PROC_SCALE`（1.0），
      窗内 678×320 只占整帧的 1/10。
    · **两道门**：半径门（`TARGET_STREAM_R_TOL`，±35%）+ 距离门。半径门是最有力的那道 ——
      `cand["r"]` 与 `last["r"]` 分别是 minEnclosingCircle(外轮廓) 与内边界÷0.55，
      两者都≈外圆半径，R_TOL **别收到 0.1 以下**（那会把真靶自己判掉）。
    · **读帧线程双缓冲（2026-10-06 加, `_FrameFeeder`）**：旧实现「cap.read() → 处理 → 发送」
      串行, read 等相机节拍（30fps ~33ms）+ 处理排后面 → ~45ms/帧 = 实测 ~22Hz。
      现在后台线程**持续读、只留最新一帧**（先放帧后推 seq, 主循环见 seq 变再取帧, 必是
      新帧; GIL 原子赋值无锁）, 读与处理重叠 → 帧率 ≈ 1/max(相机节拍, 处理耗时)。
      ⚠️⚠️ **读循环必须节流（`_loop` 里 25ms 上限 ≈40fps, 2026-10-06 补, 别删）**：
      对真 cap 毫无作用（read 本身 ~33ms）, 但 cap 是 **LeaseHandle**（接管成租约/命中
      预热）时 `read()` 只是取最新帧的引用、**立即返回** —— 不节流就是一个每秒几十万次的
      纯 Python 锁/GIL 空转循环, 把主循环检测的 Python 段饿出 4~15 倍（台架实测:
      算=658ms/帧 → 流 2fps → 单片机 600ms 新鲜度闸反复冻结, 症状「要等很久才开始/
      TGTS fail: timeout」; 同日志 07:00:04「租约空闲超 15s 自动释放」证明当时 cap 正是
      租约句柄）。释放纪律照旧: `finally` 先 `feeder.stop()` 再 `release_camera`;
      相机断供 = seq 停涨 → `wait_new` 超时返回 None → 走"无新帧不发"。1Hz 日志带
      「读=…ms 算=…ms」分解（fps 不够时区分相机没来帧 vs 检测太慢）; 预览落帧在后台线程。
    · **大候选的 bbox 几何预检（2026-10-06 加, `_target.py::_color_candidates`）**：
      `max(w,h)/2/scale > r_max_full` 的候选**在任何昂贵工作之前**跳过 —— bbox 是紧的,
      外接圆半径必然 ≥ max(w,h)/2, 所以这是 r_max 门的**零误杀等价改写**（实测两种窗、
      新旧候选集逐项一致）。动机: 窗扩张（tracker 丢一帧 margin 翻倍）会把**蓝柜子**
      （实测 r=448 / 1214×400 块）圈进窗 —— 旧顺序先对柜子做全套掩膜+findContours、
      再用 r_max 把它扔掉, Pi 上每帧白干几百 ms。与 2026-09「逐候选 bbox 切片」那次
      同一条优化思路: 别为注定要扔的东西付钱。
  ⚠️ **保存的基本都是原始测量，不做平滑**（直接替换上一帧位置）：单片机端自己有死区与
  收敛判据，这里再滤一道就是两级滤波串联，白加迟延 —— 而闭环最怕迟延。
  ⚠️ 本帧没找到的跟踪器**不回传**（不是回传陈旧值）：陈旧值会让单片机照着它继续推舵机。
  ⚠️ **无色模式（`TARGET_STREAM_USE_COLOR=0`）= 单目标跟踪**：所有跟踪器回传的颜色码都是
  `0xFF` 这同一个值，字典装不下三颗，所以改用**列表**装跟踪器；且只挑**离画面中心最近**
  的那颗跟（单片机那条"取离瞄准点最近的"兜底规则里，瞄准点默认就在画面中心附近）。
  带色模式（默认）才是可靠的：另外两颗靶是等大的圆环，无色时唯一区分手段是距离门，
  一次大修正跳到邻靶就锁错，而闭环还会**持续往错的那颗上拉**。
  ⚠️ 参数每帧从 `config` 模块动态读（Web 改即时生效），**不能** `from config import ...`。
  **定格帧 + 终态 + 详细日志（2026-10 加, 「标靶练习」的数据源）**：停流的 `finally` 里
  把「闭环最后看着的那帧」经 `logstore.archive_frame` 归档（`aimfinal`=原始帧 / `aimann`=标注帧,
  收敛/超时/中止/串口断**全都存**——超时的样本恰是练习最想看的）, 并打两行
  `[靶标流] 定格帧: frames/aimfinal_NNNN.jpg`（⚠️ 文件名进日志 = 练习页把帧与那一发 TGTS
  记录**精确配对**的键, 别改成不带文件名）与 `[靶标流] 终态: R(..,..)r.. G(..)…`。
  两个开关（`config.py`, Web「参数」页, 两层门控——`TARGET_STREAM_ARCHIVE_FRAMES` 只在
  `TARGET_STREAM_VERBOSE=1` 时生效）：前者 = 闭环逐帧打 `[靶标流#N]` 行（~30行/s, 默认关）;
  后者 = 按 `TARGET_STREAM_LOG_HZ` 节奏把过程帧归档成 `aimseq_*`（每发几 MB, 默认关）。
  ⚠️ 这两个开关由 **main.py 进程**消费, Web 改完要点「重启 main.py」才生效（双进程陷阱）。
- `JiGuangLogs/` — 标靶练习数据集（2026-10 加; `shots.jsonl` 追加式记录 + `condition.json`
  当前批条件 + `frames/` 每发的打点帧/定格帧**拷贝**）。写入方是 GKS_Web 的 `/practice` 页
  （见 GKS_Web/CLAUDE.md）; 数值来源 = log.txt 的 TGTS 调试串（单片机 `TGTS fin:` 行提供
  舵机/底盘累计与 r）。分析侧从 SMB share 直接读这个目录。
- `tasks/_color.py` — 色球顺序识别（命令 `"5"`）：主摄 HSV 先找球再定色 + 多帧投票，
  凑满三色即成功、按 cx 升序回传（`docs/串口协议.md` §6d）。
  **缺球推断（2026-09 加）**：只认出两颗时先多采一轮，再用**板子当坐标系**（板子四角 → 单应
  变换 → 球位在板归一化坐标里是固定的：实测 u=0.159/0.500/0.842、v≈0.375）把缺的那颗的
  **颜色与位置**推回来，照旧当三色结果回传 —— **刻意不打推断标记**（赛题必给三颗，只认出
  两颗基本是识别问题），下游单片机一行不用改。三层逐级降级：A1 球位表 → A2 等距自洽
  （中线先验 / 间距外推两路互验）→ B 两球外推；**任何一层判据不过就不推**（判失败、长鸣）。
  实测缺中球（=对位基准）误差 0~9px、缺端球 1~13px。判据细节见 `_color.py` 顶部那一节；
  可调量 `COLOR_INFER_*` / `COLOR_BALL_*` 在 `config.py`（Web「参数」页「色球识别」组）。
- `tasks/_rect.py` — 色球区**矩形地板**（黑胶带边框）识别（2026-09 加）。用途是给一个
  **不依赖色球**的横向基准：缺球时三球齐那套整段拒动，而板子位置每次一样，所以"板心"
  同样能当对位基准（标定一次长期用）。
  **只看黑胶带、不看板面颜色**（板面可能是白的也可能是黄的）。两条路，结果里的 `path` 说明走的哪条：
  · `enclosed`（**主路径**，四边齐全）：暗掩膜 → 闭运算 → **被胶带围住**（不与画面边界连通）
    的亮区 → 拟合四边形 → 对角线交点（精确，`estimated=False`）。这条路**不会被与胶带
    近乎共线、但没围住板子**的暗线骗走（地面缝/阴影/另一块板）—— 真踩过：底边胶带旁
    ~20px 有贯穿全画面的地面缝，按下面那条路会把它当同一条线 → 中心抖 ±25px。
  · `lines`（回退，缺一条边/板子伸出画面）：轮廓折线成直线片段 → **胶带判据**
    （`_is_tape`：胶带是细暗条、两侧都亮；车体/轮子那种剪影只有一侧亮 —— 少了这一道
    车身轮廓会混进来当"边线"）→ 共线聚类最小二乘重拟合（⚠️ `_canon_angle` 把方向角归一到
    ±90°，否则同一条胶带两条边因 rho 翻号永远聚不到一起）→ 长边（画面里 1000+px）/
    短边（横交 ≥45°、取"跨过长边中点 + 张得够开 + 胶带最长"的一对）→ 四边齐就求交，
    缺一条边按 `RECT_HW_RATIO` 仿射补出来（`estimated=True`）。
  ⚠️ **三边估出来的中心与四边精确中心差 15~30px（主要在横向）**：仿射补边不吃透视，
  是方法本身的偏差、标定补不掉 → **标定就用你实际会跑的那个形态**，两者别混用。
  ⚠️ 只剩两条边直接报失败（定不出尺寸，绝不猜）；框内亮度必须过"绝对下限 + 比画面中位亮"
  两关（挡掉车体那种大片暗区拼出的假四边形）。"缺远端长边"那一档会因此**响亮失败**，是故意的。
  ⚠️ 多帧投票（`RECT_VOTE_FRAMES`/`RECT_MIN_VOTES`）+ 跨帧散布闸，与色球同一条纪律：
  **车必须静止**。参数全在 `config.py` 的 RECT_* 段（Web「参数」页可调，前缀分组
  「色球区矩形」，`GKS_Web/app.py::CFG_GROUPS` 要同步加前缀）。
  ⚠️ 标注文字**一律 ASCII**（`cv2.putText` 画不了中文，写进去就是一排 `?`，真踩过）。
  **两条入口**：GKS_Web 的「色球区矩形(黑框)」按钮（`app.py::_handle_rect` / `GET /rect`，
  就地跑、只给页面看）与单片机指令 **`"A"`**（`handler.py::_cmd_rect`，结果回 **0x07 帧**，
  见 `docs/串口协议.md` §6h）—— 后者是「缺球且推断不成立」时的备选对位方案。
  离线回归工具（改阈值先用它跑，不接摄像头）：`C:\Users\Administrator\Desktop\rect_offline\`。
- `test_camera.py` — 双摄像头测试脚本（默认 web 推流预览；`--no-preview` 读帧统计）。
- `gks.service` — systemd 服务文件（`SERVICE_MODE=1`，`Restart=always`）。
- `../GKS 控制台.desktop` — 桌面上的控制台启动器。

## 摄像头

- 设备节点（与 GCS/cv_python 相同，由 udev 规则 `/etc/udev/rules.d/99-gks-devices.rules` 创建）：
  - `CAM_MAIN_INDEX = /dev/camera_main`（主摄像头）
  - `CAM_QR_INDEX = /dev/camera_Qr`（扫码摄像头）
- 均 `MJPG`、`on_demand` 生命周期、640x480。
- 主摄像头 udev 规则现绑定 `1e45:2050`（已从 `05a3:2b01` 更新）；扫码摄像头 `2ce3:0010`。
  硬件更换时需同步更新 `/etc/udev/rules.d/99-gks-devices.rules`。

### ⚠️ 相机档位 = 曝光 + 白平衡（`camera.py::_CAMERA_PROFILES`，2026-10 加白平衡）

每个档位（`main` / `qr` / `hostage` / `road`）在 `config.py` 里有**四个**常量：
`*_AUTO_EXPOSURE` / `*_EXPOSURE` / `*_AUTO_WB` / `*_WB_TEMPERATURE`。
`open_camera(profile=…)` 与 `camera_lease._reapply_profile()` 走的是**同一份**映射
（`camera.profile_params()`），所以"预热按哪个档、真用时就是哪个档"。

- ⚠️ **每个档位都要显式设满四个**：V4L2 的控件状态**跨 open 保留**，不设 ≠ 保持原样 ——
  看路把 AWB 关掉之后，下一个开主摄的任务（色球/靶标/网页预览）不设就继承了"关"。
- ⚠️ 自动档下 `*_WB_TEMPERATURE` 是 inactive、写了被驱动忽略，所以只在 `AUTO_WB=0` 时写它。
- **本相机白平衡量程 2800~6500K、步进 10**（`v4l2-ctl -d /dev/camera_main --list-ctrls` 实测）；
  OpenCV 4.10 的 V4L2 后端支持 `CAP_PROP_AUTO_WB`(44) / `CAP_PROP_WB_TEMPERATURE`(45)。
- ⚠️⚠️ **AWB 每次开相机都从零重新收敛、要 ~2s / ~40 帧**（实测 R−B：第 1 帧 −47 →
  第 15 帧 −14.5 → 第 40 帧才到 +6.3）。`CameraManager.open()` 的 15 帧预热**不够**。
  所以「开相机 → 采少量帧 → 按**颜色**判定」的任务要小心：量到的可能是收敛途中的某一瞬。
  **看路（`"G"`）就是靠固定白平衡（`ROAD_AUTO_WB=0` / `ROAD_WB_TEMPERATURE=6500`）
  解决这个漂移的**（台架上实测：固定 6500 从第 1 帧就稳，且与 AWB 收敛后的值一致，
  所以看路的 Y−B 判据只吃一个稳定的色偏口径）。2026-10 起阈值改每帧自适应
  （直方图谷底，见 `tasks/_road.py`），换图换灯不用重标阈值，但**换判据/换 WB 后
  Δx 会平移，单片机 `road_dx_ideal`/`road_dy_ideal` 要重标**。
- ⚠️ 改完 `camera.py` / `config.py` 要**重启两个进程**：`sudo systemctl restart gks.service`
  （main.py）+ GKS_Web 那个（`gks-web.service`，网页预览与页面上的识别都走 `open_camera`）。

## 相机租约（预热 / 复用 / 跨进程抢相机）—— `camera_lease.py`，2026-09

单片机在车移动/摆臂的空档先发预热指令（`"B"` 扫码副摄 / `"C"` 主摄），本模块把相机开好并
**后台持续取最新帧**，随后视觉任务命中租约直接解码，省掉开相机的时间。
⚠️ **实测冷开一台相机 ≈1.5~2s**（不是旧注释里的 1.3s），而解码只要 **0.56s** —— 开相机就是大头。

- **次数后缀（2026-09 加）**：`"C,2"` 里那个数是**使用次数**（不是毫秒 —— 同一个形状两种含义，
  见 `docs/串口协议.md` §6）：= 「这台相机接下来要被 2 个任务消费」。它把空闲窗口放宽成
  `次数 × LEASE_IDLE_TIMEOUT_S`（`idle_limit_s()`），倍数在**声明的次数用完之前一直保持**,
  于是隔了一二十秒的第二个任务也能命中同一份租约。不带数字/1 = 老行为一字不差。
  ⚠️ **任务用完相机是不关它的**（`release_camera(LeaseHandle)` 只 `touch()`）—— 所以卡住复用的
  从来不是"任务结束", 而是那个看门狗窗口; 次数放宽的正是它。
  ⚠️ 放宽的是**安全阀**本身: 窗口也等于「发了预热却没跟进识别时相机被白占多久」, 所以有上限
  `config.LEASE_USES_MAX`(10), 且带着没用完的预算被释放时 `release_now()` 会打一行
  `预热次数预算还剩 N 档没用完`。
  ⚠️ 写它的时候踩过两个坑, 都写进 `warm()` 的注释了: ① 次数必须在 `release_now()` **之后**设
  （那条路会清空全部状态）; ② `warm()` 的两条早退（同相机已在租约/已在预热）也必须**重设**次数,
  否则「租约还活着时再发一条预热」（固件的常态）会让次数后缀静默失效。
  ⚠️ 它是**看门狗**的事, 与 `QR_CMD_MAX_TIMEOUT_S` 那条（单次扫描不能在解码途中把设备关掉）
  互不影响。

- 任务侧只认两个入口：`acquire_camera()` / `release_camera()`（`tasks/*.py` 不许碰 `LEASE` 内部）。
  `LeaseHandle.release()` 是**故意的**空操作；`LeaseHandle.read()` 与 `cv2.VideoCapture.read()` 同形。
- **冷开自动接管成租约（2026-10-06 加, `LEASE.adopt`）**：`acquire_camera` 未命中时的冷开兜底,
  旧行为返回的是**裸句柄**、任务用完真关 —— 于是「连续几个主摄任务、中间没有 C/B 预热」时
  每个任务都全量重开（台架实测: 看路连续两条 "G" 各自重开 ~1s, 直到中间恰好出现一条 "C"）。
  现在冷开成功即 `LEASE.adopt(cap)` 接管成租约（起读线程, 与 `_open_and_serve` 同一段
  临界区纪律）, **下一个任务直接命中**（档位不同时 acquire 自带就地换档）。接管失败的两种
  情形退回裸句柄旧行为: 同台预热恰好在跑 / 已有租约占着。⚠️ 连带效果: 主摄任务结束后
  相机会**保持打开 ~15s**（空闲窗）才被看门狗释放 —— 与预热租约同一套纪律, 不是泄漏。
- **读线程连续失败 → 快速判死（2026-10-06 加）**：`_reader` 记**连续**读帧失败次数,
  ≥5 次（~1s, 一次失败读的循环 ~0.1~0.2s）`_healthy_locked` 直接判不健康 → 看门狗当场
  回收重开。根因: 台架实测一次坏 open（`@ -1.0fps`, cap.read() 全 False）要等帧龄超
  `LEASE_STALE_S`(2s) 才被判死, 预热的 2s 全打了水漂。⚠️ 换档暂停期间读线程不进 read,
  不累计失败, 不误伤。
- ⚠️⚠️ **文件头那四条必须守住**（改之前先读 `camera_lease.py` 顶部）：① 必须持续 read（否则
  拿到陈帧且不报错）；② 空闲看门狗 `lease_watchdog` 不能删（唯一的释放安全阀）；
  ③ 主摄多相机档（曝光+白平衡）要**就地重设** + 丢收敛帧；④ **预热会牺牲租约里的另一台相机**（`warm()` 目标不同
  时先 `release_now()`）→ **别在靶标流（`"7"`）运行期间发预热**，那会掐断正在跑的主摄流。

### ⚠️⚠️ 跨进程抢相机：`/abort` 兜底（2026-09-26 修，这是「随机打开失败」的主因）

真正的相机独占者是 **GKS_Web 那个独立进程**（网页预览，`:9000`，`GKS_Web/app.py::_streams`）——
它 open 之后一直占着到浏览器断开，而 `handler._stop_main_stream()` 只停**本进程**内的推流，
**对它是空操作**。于是「谁先开到设备谁赢」，现象随机：预热失败（一闪而过的 WARN）→ 识别任务
5×0.3s 重试也抢不回来 → `[ERROR] 无法打开摄像头索引 N`。

处方：`_open_with_abort_retry()` —— **open 失败就打 `http://127.0.0.1:9000/abort?cam=main|qr`
请 GKS_Web 停推流，等 0.3s 再试一次**。用在两处：`_Lease._open_and_serve()`（预热路径）与
模块级 `acquire_camera()`（冷开兜底；任务的 `_open_with_retry` 反复调它 → 天然重试）。
- ⚠️ 只在**第一次真失败之后**才打扰 GKS_Web —— 比赛时没人开预览，行为与加这条之前完全一样。
- ⚠️ 够不着 GKS_Web（Connection refused）是 **normal case**，静默忽略、也**不再多开一次相机**。
- ⚠️ `/abort` 会顺带 set() 页面内正在跑的识别（`_scan_abort`）—— 可接受：页面识别与车上任务
  本来就在抢同一台相机，语义上就该车优先。
- ⚠️ **`/abort` 的语义 = 让出该相机的「一切持有形式」：停推流 + 释放本进程的租约**
  （2026-10 补第二条腿，`camera_lease.release_lease_if`）。起因: 网页里的就地识别会把
  冷开的相机 adopt 成租约留在 **GKS_Web 进程**, 而旧版 `/abort` 只停推流 → 租约把相机
  攥到看门狗到期（网页进程以前根本没跑看门狗 = **永久**, 2026-10-04 07:52 的日志:
  main.py 连打 8 次 `/abort` 全无效, 只能拔插 USB）。⚠️ `release_now` 遇读线程卡在
  DQBUF 时会把设备留给读线程自己关（~1s）, 故被抢方仍可能在 0.3s×3 的第一轮重试里
  失败一次, 下一轮必中 —— 从「永久失败」变成「≤2s」。
- ⚠️⚠️ **`lease_watchdog` 必须在每一个会持有租约的进程里各跑一份**（2026-10 定案）：
  `main.py` 启动时拉一份, `GKS_Web/app.py` 启动时也拉一份（"网页点一次识别, 相机被
  gks-web 攥到天荒地老"就是少这份）。它是模块级的无限循环, import 不启动, 谁用谁起线程。
- 现场判据：日志出现 `已请 GKS_Web 停掉…推流（跨进程抢相机: 有人开着网页预览）` = 命中过它；
  gks-web journal 出现 `/abort: 已让出本进程的…相机租约` = 第二条腿生效过。

### 另外两个已修的确定性 bug（2026-09-26）

- `LEASE_WARM_WAIT_S` **2.5 → 4.0**：它贴着实测的 ~2s 开相机耗时，Pi 一忙（YOLO 首次加载 / 日志 IO）
  预热线程就超时 → `acquire()` 返回 None → 调用方兜底 `open_camera` **与预热线程同时开同一台设备**
  → EBUSY。这是「随机打开失败」的第二个来源。
- `_open_and_serve` 把 `th.start()` 挪进**认领 `_cap` 的同一个临界区**，`release_now` 的 join
  加 `try/except RuntimeError`：原来「认领了 `_cap` 但线程还没 start」的那一瞬间若 `release_now()`
  进来，它会对未启动的线程 join → RuntimeError → 异常让它在 `release(cap)` **之前**就中断 →
  读线程随后 start 成功、租约状态却已清空 → **那台相机再无任何路径会关它**（永久 EBUSY，只能重启 main.py）。
- 配套：`_reader` 在 `finally` 里自己 release 设备（设备归读线程所有），
  `release_now()` 的 join 超时后**不再**对使用中的句柄动手，改为留给读线程（写日志）。
- `_reapply_profile` 换档时**同时清 `_latest`**（否则收敛窗里还在发放换档前的旧帧）；
  `warm()` 命中同相机换档后补上 `self._profile = prof`（原来漏了，会白重设一次档、白丢一个收敛窗）。
  ⚠️ 它换的是**曝光 + 白平衡**两件（走 camera.profile_params，与 open_camera 同一份映射）——
  看路的固定白平衡与其它档的自动白平衡靠它在这里互相切回来。

⚠️ **改完本模块 / `config.py` 的 `LEASE_*` 要 `sudo systemctl restart gks.service` 才生效**：
消费它们的是 **main.py 那个进程**，而 Web「设置参数」页由**另一个进程**（GKS_Web）提供 ——
页面上的 `save_override` 只改到 GKS_Web 自己那份 config 副本 + `config_override.json`，
main.py 要重启才会重新加载。（GKS_Web 的 `/config` 因此可能显示**旧值**，那是显示滞后、不是没改成功。）

## 串口（两个 CH340，按芯片版本号 bcdDevice 区分）

| 设备节点 | 硬件 | 识别依据 | 用途 |
|----------|------|----------|------|
| `/dev/ttyUSB_GKS` | 单片机主串口（CH340） | `bcdDevice=="8134"` | 帧协议 + 握手 |
| `/dev/ttyUSB_OpenMV` | ATK-USB-UART → OpenMV（CH340） | `bcdDevice=="0264"` | 寻线文本帧，@115200 |

- 两个 dongle 的 `idVendor:idProduct` 都是 `1a86:7523`、都没有序列号，唯一能区分的就是 bcdDevice。
  规则见仓库根 `99-gks-devices.rules`（装到 `/etc/udev/rules.d/`）→ **换插口也不会认错**。
  换新 dongle（bcdDevice 变了）时改那两行，否则符号链接一个都不建（报「串口不存在」）。
- ⚠ **别改回按物理口绑**（踩过）：曾用 `KERNELS=="1-1.4"` 绑 OpenMV、其余口绑 GKS。两根线一插反就是
  **静默认错** —— `/dev/ttyUSB_GKS` 开得开却永远收不到 ACK（握手一直「未成功」），`/dev/ttyUSB_OpenMV`
  也报 `connected=true` 却零字节，界面看着一切正常。另：当时 `/etc` 里还有 `fix_device.sh` 生成的
  `99-fixed-devices.rules` 也写了 tty 规则，两个文件抢同一个符号链接名，更难查。
- **绑定分家**：摄像头（VID 随模组更换而变）仍由 `~/Desktop/fix_device.sh` 按物理端口生成
  `99-fixed-devices.rules`；**该脚本已不再管串口**，别把 tty 规则加回去（会与 `99-gks-devices.rules` 抢名字）。
- **不拆线认口**（怀疑插错/换过线时先跑这个）：往每个 `/dev/ttyUSB*` 发一帧握手，
  回 `aa 20 01 cc` 的那个就是单片机 ——

  ```bash
  cd ~/Desktop/GKS_Competition && python3 -c "
  import serial, time; from protocol import pack_frame, TYPE_COORD
  f = pack_frame(TYPE_COORD, b'SYNC_HS_1')
  for p in ['/dev/ttyUSB0', '/dev/ttyUSB1']:
      s = serial.Serial(p, 115200, timeout=0.3); s.write(f); s.flush(); time.sleep(0.5)
      print(p, s.read(64).hex(' ')); s.close()"
  ```

  OpenMV 侧同理：写 `b'G'` 后应连续吐 `L,...`（验完写 `b'S'` 还原空闲）。
  端口被 main.py 占着时先停服务；`sudo ./fix_device.sh --detect` 也会列出每个口 + bcdDevice。
- OpenMV 是独立传感器但**按需发送**：默认不发，收到 `'G'` 才发 `L,<offset>,<angle>,<valid>\r\n`，
  `'S'` 停（触发方 `handler._cmd_line_start/_stop`）。⚠ 所以「端口打开成功」≠「传感器在发」，
  空闲零字节是**正常**的；发了 `'G'` 后仍无数据才告警（`openmv_serial._check_silent`，阈值
  `config.OPENMV_SILENT_WARN_S`），`/line_status` 同时给出 `expecting / silent_s / hint`。
- 树莓派收到命令 `"2"` 后进入巡线转发（PID 参数在 config.py，Web 可在线调），`"3"` 停止。

## ⚠️ 新增/修改「串口指令」的同步清单

协议是**跨仓库契约**，改一处必须同步其它处 —— 漏掉不会报错，而是**运行时静默失效**
（指令没反应 / 结果帧被丢弃），排查代价很高。按下面逐项打勾：

**新增一条指令（如 `"4"` 人质识别）要动 4 处：**
（⚠️ 单字符 `"1"`~`"9"` 与 `r`/`end`/`sd`/`rb` 都已占用 —— 新指令用**字母**，如矩形地板的
`"A"`；2026-09 又占掉 `"E"`/`"F"`）

1. `tasks/_xxx.py` — 任务函数（只返回结果 dict，**不碰串口**，保持可单测，见 `tasks/_qr.py` 注释）
2. `tasks/__init__.py` — 重新导出任务函数，**以及 handler 会 import 的任何其它符号**
   （如 `build_result_frame`）。漏了会在 main.py 启动时抛 `ImportError`，
   表现为「启动后无声退出」（`sys.stderr` 未进日志；`main.py` 现有兜底会打印 traceback）
3. `handler.py` — 写 `_cmd_xxx()` + 在 `_COMMANDS` 注册表加一条；
   若该任务用 COORD 帧回传且会与巡线并发，**在 `handle()` 里先 `_cmd_line_stop()`**
   （串口发送无跨线程锁，防帧交错），如命令 `"1"`、`"4"`
   - ⚠️ 那个互斥元组判的是**拆出来的 head**，不是整串 payload（`"F,7500"` 也要能命中）
   - ⚠️ 阻塞类任务若想让**单片机决定超时**，加进 `_MS_COMMANDS` 白名单：
     它就能收 `<命令码>,<毫秒>` 形状、用 `_budget_from_ms()` 倒推自己的预算
     （见「超时随指令下发」一节）。**不在白名单里的指令带了逗号会被拒并打 WARN**
     —— 刻意不静默忽略那个数值。
   - ⚠️⚠️ 那张白名单现在是**两族共用**的：`"1"/"D"/"E"/"F"/"G"` 那个数是**毫秒**，
     预热 `"B"/"C"` 那个数是**使用次数**（相机租约窗口的倍数，见「相机租约」一节）。
     往白名单里加新命令时**先想清楚自己属于哪一族** —— 照着字面把毫秒抄过去就是静默错权重。
4. `../GKS_Web/simulator.py::COMMANDS` — 网页指令按钮表（三处命令码须与 `docs/串口协议.md` §6 一致）

**⚠️「只换相机/只换算法、结果同构」的命令要复用同一帧型（`"D"`/`"E"`/`"F"` 是范例，2026-09）：**
四条扫码命令（`"1"` 副摄+微信 / `"D"` 主摄+微信 / `"E"` 副摄+自研 / `"F"` 主摄+自研）
返回的是**同一种帧**，所以单片机的 `ucRpiWaitQrResult` 与全部下游一个字都不用改。

**任务函数必须能收 `timeout=`：** 从串口进来的指令由 handler 用
`run_fn(timeout=_budget_from_ms(ms))` 调用（见 `_cmd_qr_common`）。新写任务函数时
签名照 `run_qr_wechat(timeout=None, abort_event=None)` 那样留出这两个关键字参数 ——
否则 `handle()` 那条带时限的分支会直接 TypeError。

**⚠️「只换相机、结果同构」的命令要复用同一帧型（`"D"` 是范例，2026-09 加）：**

扫码有**两条命令**，差别**只在用哪台相机**，协议层完全同构：

| 命令 | 任务函数 | 相机 | 结果帧 |
|---|---|---|---|
| `"1"` | `_qr.run_qr_wechat` | 扫码副摄 `/dev/camera_Qr` | §6a 的 9B ASCII |
| `"D"` | `_qr.run_qr_wechat_main` | 主摄 `/dev/camera_main` | **同一个** §6a |

- 两条都经 `handler.py::_cmd_qr_common(run_fn)`，`_cmd_qr` / `_cmd_qr_main` 各自只传一个
  run_fn。**别把回传/存最后一帧那套复制成第二份**。
- **不要为它新增结果帧**（不加 TAG、不加 §6x）：单片机侧就是靠"格式相同"才做到
  `ucRpiWaitQrResult` 与下游（IF 条件/排爆/反恐）一个字都不用改。加"只换相机"的命令
  一律照此办理。
- ⚠️ **`"D"`/`"E"`/`"F"` 都必须进 `handle()` 里那个互斥元组**
  （`if head in (b"1", b"4", b"5", b"6", b"A", b"D", b"E", b"F"):`）
  —— 主摄那两条不先停巡线/靶标流就会被流线程占着设备；而 `"E"` 虽然用副摄，
  它一样发 COORD 结果帧、同样要停巡线防帧交错。⚠️ 判据是**拆出来的 head**（见 `_split_payload`），
  不然 `"F,7500"` 会漏判。
  ⚠️ 而 `"B"`/`"C"`（预热）**刻意不在**那个元组里：它们只开相机、不发结果帧，
  且 `"B"` 恰恰排在巡线**之前**。别因为加了 E/F 就把 B/C 一起塞进去。
- ⚠️ **预处理参数三套独立**（`config.py`）：副摄 `QR_SCAN_*`、主摄 `QR_MAIN_SCAN_*`、
  自研 `QR_DIFF_*`。
  最要紧的是 `QR_SCAN_CENTER_CROP=0.3`（只取画面中心 30%）是为副摄的近距离取景调的，
  主摄对着场地时二维码多半不在中心 → `QR_MAIN_CENTER_CROP` 默认 **0.0（不裁剪）**；
  自研那条也 0.0，且 `QR_DIFF_MAX_SIZE=1280`（它的定位角检测按分辨率调过，960 会漏）。
  改这些要在 Web「设置参数」里能调 → **必须登记进 `config.py::EDITABLE_CONFIG`**
  （那是唯一登记处；漏了不报错，只是页面分组里没有它）。
- 主摄那条走 `acquire_camera(is_qr=False, profile="qr")`：**曝光档是关于目标的**
  （二维码对比度），不是关于相机的；租约命中时**就地换曝光档**不重开设备 ——
  所以「先 `"C"` 预热主摄、再 `"D"` 扫码」能省下 ~2s 里的大部分
  （⚠️ 前提是 `"C"` 与 `"D"` 之间**有 ≥2s 间隔**，见「相机租约」一节：间隔不够等于白预热）。

## 自研难度二维码算法（命令 `"E"`/`"F"`，2026-09 加）

`tasks/_qr_diff.py` —— **第二套解码器**，与微信引擎**手动二选一**（不做自动兜底）。
治的是 wechat 治不了的图：斜视 / 模块像素太少 / 强透视。

**为什么要有它**（台架实测 2026-09-26，主摄对着码板同一位姿）：wechat **连试三次全失败**
（「未检测到二维码」），自研解出 `233`，**4.48s**（开发机同图 1.0s → 倍率 4.5×）。

### 它是什么 / 不是什么

- 移植自桌面项目 DiffcultQR（几何重建 + 候选集校验）。**只跑候选集模式**：
  先用三次形变场把二维码几何重建出来，再在**已知候选集**里按 ECC 字节精确匹配定内容。
- ⚠️ **候选集 = `config.QR_DIFF_KNOWN`（默认 `"123"` = 111~333 共 27 种）**：
  赛题二维码内容固定是三位数、每位 1~3。**内容不在候选集里它必然失败**
  （它不做一般解码）。`"123"` 里那三个字符是**每一位的取值范围**，不是「从 123 开始」。
- ⚠️ **只认 version 1（21×21）**：全流程写死 `N=21`（定位角间距、26 个码字、
  暗模块位置）。有个宽进严出的 `est_n` 守卫抓「显然不是 V1」，但**真正防住乱解码的是
  候选集匹配本身**（要求 26 个码字里至少 6 个校验字节逐位相同，几何错位只会「解不出来」，
  不会「解出一个错的数」）。
- ⚠️ **零新依赖**：它自带一个极小的纯 Python GF(256)/RS 编码器，只用来在 import 时
  预生成 27×4 条候选码字表（108 次，实测 **11ms**，之后缓存）。**只需要 cv2 + numpy** ——
  别为了它去装 reedsolo/zxing-cpp。
- ⚠️ **刻意不含快速通道**（zxing/pyzbar/OpenCV）：那是 wechat 那条路的职责；混进别的
  解码器会让「微信 vs 自研」的对比失去意义。
- ⚠️ 它**不返回 `poly`**：几何建立在自有的 1320×1320 画布上，坐标要经 Hcur⁻¹ 才能映射回
  帧空间。`_annotate` 对 `poly=None` 已有分支，别为画个框去凑一个会画错的变换。

### 排障：失败留档（`logs/Difficult/<时间戳>/`）

自研的失败可能卡在**四层中的任一层**（定位角 → 几何拟合 → 模块采样 → 候选匹配），
而 wechat 的失败只有「没检到」一种 —— 所以**失败时**把中间产物写下来
（成功不留；只保留最近 `QR_DIFF_DEBUG_KEEP` 份，Pi 的 `logs/` 是只增不减的）：

| 文件 | 用来回答 |
|---|---|
| `frame.jpg` | 喂进解码器的原图 —— **实测靠它一眼看出「主摄对着天花板」** |
| `warp.png` | 校正后的正视图 —— 几何重建对不对 |
| `matrix.npy`/`.png` | 21×21 采样矩阵 —— 模块采样对不对 |
| `hits.txt` | 卡在哪一层 + `hits_per_ecc` —— 「采样对但不在候选集」还是「采样本身就错」 |

日志里另有每轮一行的 `[INFO] 自研: <stage> (x.xxs, score=..) -> <原因>`，
以及任务级的 `检测到二维码 [diff]: 233（用时 4.48s）`（`_run_task` 里新加的耗时打点）。

### ⚠️ 慢解码器对既有循环的两个影响（都踩过）

1. **`_scan_qr` 只在每轮开头查 deadline** → 一次尝试 4.5s 时，预算 6s 会跑成 ~9s，
   顶穿「本端必须比单片机先放弃」。修法：`_scan_qr` 把本轮总 deadline 放进
   `_SCAN_DEADLINE`，`_decode_diff` 拿它当自己内部的止损点；另加一道
   `_DIFF_MIN_SLICE_S`(1.6s) —— 剩余时间不够跑一轮就**不再起跑**
   （实测把 7.13s 收到 6.04s）。
2. **失败留档的目录**要限量删旧（`_prune`），否则反复重试会把 SD 卡写满。

### 跑分 / 攒样张

- `bench_qr.py`（仓库根，**在 Pi 上跑**）：同一批图片，微信与自研各跑一遍，
  出成功率 + 耗时表。⚠️ 它按**文件路径**直接加载 `tasks/_qr_diff.py`，
  不走 `from tasks import ...` —— 那会把 `tasks/__init__` → `_qr.py` → `camera_lease`
  → `serial_comm` → **pyserial** 整条硬件栈拖进来，于是没有 pyserial 的机器上连自研都跑不了。
- ⚠️ **A/B 必须同相机**：Web 的 `/scan` 里 `wechat` vs `diff`（副摄）、
  `wechat_main` vs `diff_main`（主摄）才是同相机对比。跨相机比出来的差异说明不了算法好坏。
- `grab_qr.py`（仓库根，**在 PC 上跑**）：从 `/snapshot.jpg?cam=main|qr` 抓一帧存进
  `qr_corpus/` 攒样本。⚠️ 采之前先在页面上点「退出」停预览 —— 摄像头跨进程独占，
  推流占着时开相机会失败（EBUSY）。`qr_corpus/` 里的 `*.jpg` 本来就在 `.gitignore` 里。

**⚠️ 长任务要开线程 + 与其它流互斥（`"7"`/`"8"` 靶标流是范例，2026-09 加）：**

命令 `"2"`（巡线转发）与 `"7"`（靶标流）都是**长任务**，跑独立线程而不是同步阻塞主循环 ——
否则期间心跳/其它指令全进不来。线程化之后心跳照发，链路比阻塞式的 `"6"` 更健康。
两条流**必须互斥**（都独占主摄 + COORD 通道），所以：
   · `"7"` 里先 `_cmd_line_stop()`；`"2"` 里先 `_cmd_aim_stop()`；
   · `handle()` 里 `"1"/"4"/"5"/"6"` 与 `"end"`/`"r"` 分支**两条都要停**
     （结果帧与流帧同为 COORD，交错会让单片机拿流帧去喂 `wait_target_result`）。
⚠️ `_cmd_aim_start` 里还要**等上一个流线程释放相机**（`old.join(timeout=)`）：
不等的话 `_open_with_retry` 会拿到"设备仍被占用"而失败，表现是单片机那边帧计数一帧都不涨、
冻到超时长鸣，而日志里只有一条"主摄像头打开失败"，很容易被误判成硬件问题。
⚠️ 模拟器侧：**流帧不进通用日志**（~20Hz 逐帧写会在 25 秒内刷爆 maxlen=500 的日志环），
改成 1Hz 摘录（帧率 + 靶数 + 坐标）。判"新一轮起流"要用**距上一帧的间隔**，
不能写成"距本轮起点"——后者在流跑满间隔后每帧都成立，于是每帧都重置。

**顺带（跨到单片机仓库 `GongKeCompetition/`）：**

- `Application/app_rpi.h::emRpiTaskTdf` 追加枚举 + `Application/app_rpi.c::s_apcTaskCmd[]`
  追加 ASCII 串 —— **表下标必须等于枚举值**（表按枚举下标索引）
- `docs/串口协议.md` §6 命令表加行；涉及新结果帧则新增 §6x 帧格式说明

**新增一种「二进制结果帧」还要注意：**

- 挑一个 `< 0x20` 的 TAG（**0x01 巡线 / 0x02 人质 / 0x03 色球 / 0x04 靶标 / 0x05 靶标流 / 0x06 拍照 / 0x07 矩形地板** 已占用；ASCII 帧首字节 ≥ `'0'`，天然不冲突）
- 单片机 `app_rpi.c::vRpiTask` 的 COORD 分发加分支，且**必须放在通用 `LEN >= 9` 分支之前**
  —— 否则会被当作「9 字节 last packet」吞掉，永远到不了解析逻辑
- 长度/字段用**小端** `struct.pack("<...")`，与巡线帧同约定

**生效方式**：改 `GKS_Competition/` 要重启 `gks.service`（或让单片机发 `"r"`）；
改 `GKS_Web/` 要重启 GKS_Web 服务（模拟器是网页服务里的线程）。

## ⚠️ 链路故障自愈（串口 I/O 错误 → 原地重启，别改回去）

**症状（2026-09 现场）**：USB 串口适配器掉线，盘上 `/dev/ttyUSB_GKS` 消失，日志停在

```
[INFO] 正在握手：发 SYNC_HS_1，等单片机回 ACK 0xCC ...
[ERROR] 接收失败: [Errno 5] Input/output error
[INFO] 收到中断信号，正在清理资源...     ← 谎话：根本不是中断
[INFO] 资源已释放，程序退出。
```

进程就此死掉。systemd（`gks.service`，`Restart=always`）会把服务拉起来，但**调试模式**
（GKS_Web 控制台「开始/重启 main.py」走的是 `console.start_main_py()` 的 `Popen`，
注释写明「无自动重启」）下没有任何东西会管它 —— 现象就是「Pi 掉一次串口，车从此不响应」。

**根因**：`handler.run()` 原来把 `(KeyboardInterrupt, OSError, ValueError, SerialDisconnected)`
一律打印成「收到中断信号」然后退出。而握手阶段的一次 I/O 错误正是这么一路传上来的：
`receive()` 读 `in_waiting` 抛 `OSError` → `_mark_disconnected()` 置空 `ser` →
`_handshake()` 下一轮 `send()` 抛 `SerialDisconnected`（那里只有 `except SerialDisconnected: raise`）
→ 落到 `run()` 的 except。`_run()` 主循环里那个「串口断开则重连并重新握手」的分支**覆盖不到握手阶段**。

**现在**：`run()` 按「谁让它退出的」分两种 —— `should_stop()` 为真（Ctrl+C / SIGTERM / 菜单）
才真退出；否则调 `serial_comm.restart_process()` **原地 `os.execl` 重启**（不退出进程）。
配套三条，都别当成可有可无：

1. **重启前必须关串口**（`_exec_self()` 上方注释）：`os.execl` 是「换进程镜像」不是「起新进程」，
   没关掉的 tty 会一直被这个 PID 占着，新镜像再打开就是「设备忙」。
2. **连续重启逐次加长**（`RESTART_DELAY_S` 3s → `RESTART_DELAY_MAX_S` 60s，计数走环境变量
   `GKS_LINK_RESTART_N` 跨 exec 继承）：适配器真掉线时「连上→报错→重启」收敛不了，
   而每次重启都会**新建一个日志文件**（`main.setup_logging` 按启动时刻命名）——
   固定 3s 会在几分钟内往 SD 卡堆出上千个小文件。
   清零只在**握手成功**那一刻（`serial_comm.note_link_ok()`，`_handshake` 里）：
   本文件要治的正是「口打得开、但一读就 EIO」那种适配器假死，那时 `open()` 是成功的，
   只有走完一次真收发才算链路真好 —— **别改成「串口一打开就清零」**。
3. `send_frame()` 在 `ser` 被置空时报的是 `last_io_error`（那次 `[Errno 5]` 本身）而不是
   「串口未打开」—— 后者会把人带去查权限/占用，方向全错。

`restart_program()`（单片机下发的 `"r"` 指令、等 20s 让机械臂/底盘停稳）**行为未变**，
两者共用 `_exec_self()`。

## 安装 / 部署

```bash
# 1. 安装并启用 systemd 服务（开机自启 main.py）
sudo cp gks.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable gks.service      # 开机自启
sudo systemctl start gks.service       # 立即启动

# 2. 桌面启动器设为可执行 + 受信任
chmod +x "/home/xixi/Desktop/GKS 控制台.desktop"
```

## 约定

- 服务名 `gks.service`。串口已接入：main.py 会同时等待真实串口 `/dev/ttyUSB_GKS`
  与模拟器虚拟串口 `127.0.0.1:9997`（由 GKS_Web 提供），二者平级、谁先就绪用谁。
- 控制台所有 sudo 命令依赖用户 `xixi` 的 NOPASSWD 权限（与 cv_python 一致）。
- **调试预览统一用 web 推流（`web_preview.py`），不再使用 `cv2.imshow`/Qt 窗口。**
  需要调试时 `from web_preview import preview`，调 `preview.publish()` / `preview.add_camera()` 即可。
- 后续接入任务时，遵循 cv_python 的架构：串口指令 → handler 分发 → tasks/* 任务函数，
  视觉检测参数集中在 config.py。
