# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

本文件为 Claude Code 在此仓库中工作时提供指导（现场笔记以中文为主）。

> 📌 **仓库现状（2026-10-27 核对）**：本文件是多轮现场调试沉淀的「坑位地图」，
> **改任何相机 / 串口 / 视觉参数之前，先读对应那一节**。各节里的 ⚠️ 大多是
> 「改回去就会静默失效」的约束，不是历史八卦。
> ⚠️ **本机 git 基线是几周前的**：`camera_lease.py` / `camera_picam.py` / `logstore.py` /
> `tasks/_qr_auto.py` / `_qr_diff.py` / `_rect.py` / `_road.py` / `_road_yaw.py` /
> `_bucket.py` / `bench_qr.py` / `grab_qr.py` / `test_picam_reap.py` / `tools/` 全是
> **未跟踪**状态，已跟踪文件也普遍有未提交修改 —— **`git checkout` / `git clean` 会毁掉
> 工作区**。回滚请从 `_bak-review-20261006/` 或同目录的 `*.bak-*` 快照取（均不入库）。
> ⚠️ **改完 `config.py` / `camera*.py` / `handler.py` 要重启两个进程**：
> `sudo systemctl restart gks.service`（main.py）+ GKS_Web（`gks-web.service`，`:9000`）。
> 两个进程各持一份 config，Web 页面改参数只写 `config_override.json`，main.py 需重启才加载。
> 📌 当前现场 override 与出厂值差异很大（如 `LINE_SETPOINT=-28` / `LINE_KP=0.85` /
> `LINE_SEND_HZ=50` / `PICAM_ROTATE=0`）—— **判断实际行为先读 `config_override.json`**，
> 别照 `config.py` 里的出厂默认值推断。

## 项目概况

GKS 工科赛（树莓派 + STM32）车载视觉/串口任务系统。树莓派负责视觉检测、相机管理、
巡线 PID 与结果帧回传；单片机负责运动控制、任务编排与超时；GKS_Web（兄弟仓库，
`:9000`）负责网页控制/预览/模拟器。三方之间只有两类契约：

- **串口帧协议**：`protocol.py` ↔ `docs/串口协议.md` ↔ 单片机仓库 `GongKeCompetition/`
- **参数登记表**：`config.py::EDITABLE_CONFIG`（Web 参数页与单片机 `"P"` 族都按它校验）

参考姊妹项目 `/home/xixi/cv_python`（GCS 机械臂视觉引导），但本仓库已大幅演化，
不要按那个项目的结构假设本仓库。

## 常用命令

本仓库没有 build/lint/format 配置，是直接跑在树莓派上的 Python 3 脚本集合。

```bash
# —— 运行 ——
sudo systemctl restart gks.service     # 主程序（SERVICE_MODE=1，无头，Restart=always）
sudo systemctl status gks.service
python3 main.py                        # 手动前台调试（同时等真串口与模拟器虚拟口）
python3 control_panel.py               # 800x480 触摸屏控制台（PyQt5）

# —— 语法检查（本机无 cv2/pyserial 时也可跑）——
python3 -m py_compile main.py handler.py config.py serial_comm.py protocol.py tasks/*.py

# —— 自检 / 离线工具（都不需要单片机在线）——
python3 test_picam_reap.py             # CSI 遗留 rpicam-vid 回收的单元测试（带断言）
python3 test_camera.py --no-preview    # 双 USB 相机取流统计；默认开 web 预览
python3 camera_picam.py                # 列 CSI 相机 / 抓一帧预览（台架自检）
python3 bench_qr.py [目录|图片]         # 二维码跑分：微信 vs 自研（在 Pi 上跑；不含新 zxing 引擎）
python3 tools/replay_target_tune.py    # 靶标离线回放调参（喂 logs/run_*/frames）
python3 tools/replay_target_tune.py ... --scan-rmax 100,300,25   # 扫 TARGET_R_MAX
python3 color_calib.py --image x.jpg   # 色球 HSV 阈值标定（或 --camera 在线）
python3 grab_qr.py 标签 --cam qr       # 【PC 上跑】从 /snapshot.jpg 攒 qr_corpus/

# —— 部署 ——
sudo cp gks.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl enable --now gks.service
```

⚠️ **没有 pytest 测试套件**；`test_picam_reap.py` 是唯一带断言的自检脚本，直接
`python3 test_picam_reap.py` 跑。其余 `test_*.py` / `bench_*.py` 是台架工具。

## 架构总览（跨文件才能看懂的部分）

**两个常驻进程，各有一份 config 副本**：

- `main.py`（`gks.service`）：串口链路 + 所有视觉任务 + 相机租约/CSI 持有器。
- `GKS_Web/app.py`（`:9000`，兄弟仓库）：控制页、MJPEG 预览、模拟器、参数页、
  标靶练习数据入库。**它也会直接调 `tasks/*` 做识别**（页面按钮），因此同样持有相机、
  同样需要 `lease_watchdog`。

两者共享 `config_override.json`，但**内存不共享** —— 这是「页面显示旧值 / 改了不生效」
类问题的总根源。

**main.py 启动链路**：`logstore.start_run()` 建运行目录 → 起相机租约看门狗、
CSI 看门狗、OpenMV 读取线程、日志清屏看门狗 → `serial_comm` 同时等
`/dev/ttyUSB_GKS` 与模拟器虚拟口（谁先就绪用谁）→ 握手 → `SerialInterruptHandler.run()`
主循环收帧、查 `_COMMANDS` 分发。

**三种任务形态**：

- **同步阻塞**（`"1"/"4"/"5"/"6"/"A"/"G"/"H"` 等）：在主循环线程跑，单片机给时限
  （`_MS_COMMANDS` 白名单决定能否带 `<ms>` 数字），期间靠 `abort_event` 被打断。
- **独立线程长任务**：巡线 `"2"/"3"`、靶标流 `"7"/"8"`、爆爆桶流 `"J"/"K"`。
  心跳与其它指令照常收发；三条流**互斥**（都占 COORD 通道，前两条还占主摄），
  起一条前先停另一条。
- **只发不回 / 单帧控制**：预热 `"B"/"C"/"W"`、补光 `"L"/"T"`、取消 `"x"`、
  OLED `"RM"/"RS"/"CL"`、取参/临时设参 `"P"`、自动下发二维码 `"QA"/"QE"/"QU"/"QC"`。

**相机资源三层**：

1. `camera_lease.py`：两颗 USB 相机（主摄 / 扫码）的单槽租约 + 预热 + 跨进程 `/abort`。
2. `camera_picam.py`：CSI 相机独立持有器（libcamera 独占、`rpicam-vid` 子进程），
   不受 USB 租约约束，两台可同时热。
3. `camera.py::open_camera(profile=…)`：真正的 V4L2 打开 + 曝光/白平衡档位。
   `camera_web.py` 已废弃，但 `handler` 仍 import 它的 `_stop_stream`。

**串口/协议**：`protocol.py` 负责帧编解码与 CRC；`docs/串口协议.md` 是交付单片机方的
正式契约（命令表 §6、结果帧 §6a–§6m）。任何新增命令/帧型都必须同步
（本仓库 handler + 文档 + 单片机仓库 + GKS_Web 模拟器），清单见「新增/修改串口指令」节。

**配置与日志**：`config.py` 是唯一可调参数登记处（`EDITABLE_CONFIG`）；`config_override.json`
是现场持久化覆盖。`logstore.py` 把每次启动的日志/归档帧放进 `logs/run_<时间戳>/`，
`JiGuangLogs/` 是标靶练习数据集（由 GKS_Web `/practice` 页写入）。

## 文件说明

- `main.py` — 主程序：日志双写、无头运行(SERVICE_MODE)、优雅退出；`logstore.start_run()`
  建运行目录，起相机租约/CSI/OpenMV/清屏四条看门狗线程后进入串口接收/指令分发主循环
  （`SerialInterruptHandler`）。**不再启动 Web 控制页**（见下）。
- `protocol.py` — 串口帧协议（SOF|TYPE|LEN|DATA|CRC8|EOF），交付单片机端的契约源，
  详见 `docs/串口协议.md`。帧类型与 CRC 表是双端契约，**改一个字节就要同步单片机**。
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
- `config.py` — 全局参数 + `EDITABLE_CONFIG` 登记表（Web 参数页 / 单片机 `"P"` 族的唯一
  校验源）；`_coerce` 拒绝未登记的名字与非有限值。`config_override.json` 是现场覆盖。
- `logstore.py` — 运行目录/日志/归档帧管理（`logs/run_<时间戳>/`）、清屏标志文件、
  标靶练习帧拷贝、旧 run 清理。唯一入口 API 比 `logs/` 目录布局更稳定，别自己拼路径。
- `camera.py` — 摄像头管理：`CameraManager.open/read/release`（移植自 cv_python 的
  V4L2 初始化 + 预热校验核心逻辑）+ `open_camera(profile=…)` / `profile_params()`；
  `_CAMERA_PROFILES` 是曝光 + 白平衡的唯一定义处（见「相机档位」节）。
- `camera_lease.py` — USB 相机租约/预热/跨进程互斥（`acquire_camera` / `release_camera`）
  与 `lease_watchdog`，见「相机租约」节。任务代码不许直接碰 `LEASE` 内部。
- `camera_picam.py` — CSI（libcamera）相机持有器 `PICAM` + `reap_stale` 遗留进程回收，
  见「pi 摄像头」节。独立于 USB 租约。
- `web_preview.py` — Web 实时预览框架（MJPEG over HTTP）。统一的调试预览方案，
  替代 `cv2.imshow`/Qt 窗口。`preview.publish(name, frame)` / `preview.add_camera(name, cap)`。
- `camera_web.py` — **已废弃的** Web 控制页（摄像头预览 + 二维码识别按钮）。main.py 不再
  `start_server()`；唯一入口是 GKS_Web（`:9000`，`GKS_Web/app.py`，已覆盖本页全部功能含参数设置）。
  文件保留的两个理由：① `handler.py` 的 `_stop_main_stream()` 要
  `from camera_web import _stop_stream`（人质/色球识别前释放主摄）——**别删文件**；
  ② 单独调试可 `python3 camera_web.py --port 8000`。⚠ 别在 main.py 里恢复 `start_server()`。
  另有意的功能损失：`/line_status` `/line_start` `/line_stop` 三条巡线调试路由不再提供
  （它们当年能工作是因为页面就在 main.py 进程里；GKS_Web 是另一进程，`import handler`
  拿到的是没有串口链路的新模块）。台架看数改看日志里 `LINE_LOG_HZ`（出厂 5Hz、现场
  override 4Hz）的 `[巡线] 偏差=.. 航向角=.. 有效=..`。
- `tasks/_qr.py` — 二维码任务的总入口：**`"1"`/`"D"` 现在走新引擎 `run_qr_auto` /
  `run_qr_auto_main`**（见 `tasks/_qr_auto.py`）；`run_qr_wechat*` 是旧链路（zxing 轮子
  缺失时自动回退 / 对照用）；`run_qr_diff*` 是自研难度算法（`"E"`/`"F"`）。
  全部返回结构相同，5s 级超时、三位数且每位 1~3 校验、支持 `abort_event`。
- `tasks/_qr_auto.py` — **新扫码引擎（2026-10-23）**：「zxing 解不出也给位置」当检测器 →
  按检测框紧裁 1.6 倍 → 放大到 960 → 放大后再 CLAHE → 多二值化器重试；检测失败才走
  中心裁剪阶梯。现场真帧基线 2/10 → 10/10。**只吃一帧 BGR、不碰相机/串口**，便于离线跑分。
- `tasks/_line.py` — 巡线转发任务：`run_line_follow` 循环读 OpenMV 最新值 → 归一化 →
  `LinePID` → 以 TYPE_COORD 巡线帧（DATA[0]=0x01 标记，8B）实时发单片机
  （发送频率 `LINE_SEND_HZ`：出厂 30Hz，现场 override 50Hz）。
  PID 参数每帧从 config 动态读（Web 设置页在线调）。
  目标位置：`err = LINE_ERR_SIGN × ((OpenMV offset − LINE_SETPOINT) / 100)`。
  `LINE_SETPOINT`（Web 可热调；**出厂 20，现场 override -28** —— 一律先读 override）
  就是"理想值"—— 0 = 停在相机零点（画面正中），
  非 0 = 稳定偏在线某一侧（单位同 offset，名义 ±100；相机端也有个等效旋钮 `TARGET_OFFSET`，
  单位是像素，**两边只改一处**，都改会叠加）。
  丢线策略：`err` 恒置 0（协议约定 valid=0 时 err 无意义），`steer` 分三段 ——
  ⚠️ **这几个"默认值"以 `config_override.json` 为准，别照本文档的出厂值判断**（2026-10 审查）：
  `config.py` 出厂是 `LINE_LOST_HOLD_S=5.0` / `LINE_LOST_SEARCH_S=0.0` / `LINE_SEARCH_STEER=0.0`
  （= 保持 5s 后**不找线直接停**），而**现场 override 是 `5.0 / 5.0 / 1.0`**（保持 5s → 定向/兜底
  满幅找线最多 10s）。本文档旧版写的是 `0.5s`，三处都对不上 —— 排障前先读 override。
  丢线后 `LINE_LOST_HOLD_S`（**现为 5s**）内**沿用最后有效 steer**（丢线率一高，清零会让
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
   ⚠️⚠️ **"⓪ 凑不齐两色"时的降级池必须是「有内孔的候选」，不能是"全部候选"(2026-10-16 修)**:
   原来那句 `work = ring if 够两色 else cands_by_color` 把判据⓪**整个关掉**, 于是又回到
   "三个杂块凑巧一样大"的老坑 —— 现场（CSI 帧, `logs/run_20261016_054241`）蓝靶与**蓝柜子
   合并**（整块被 r_max 剔掉）⇒ 只剩红色像靶环 ⇒ 退回全部 ⇒ 地面三个 r≈11~15、**无孔**
   的小杂块凑成一组被选中, 真红靶（q=0.54、有孔、r=177）反被挤掉、标记画到地上;
   随后 `_locate_one` 又把三个杂块判掉 ⇒ 整帧只剩 0~1 颗（"识别框错位/画在地上"的真相）。
   现在: 凑不齐两色 → 退回 **hole_fit 非空的候选**（"有白心"是不依赖同伴的靶盘证据,
   与末尾那把兜底钥匙同款）; 连它都凑不齐两色才真的退回全部（保旧行为）。
   ⚠️ 离线套件 `tools/target_offline/`（GongKeCompetition 仓库）的 `detect.py` 是**同一份
   算法的离线镜像**, 改这里必须同步改那边, 并跑 `python evaluate.py --quick` 复验 ——
   2026-10-16 这次改动后 41 用例指标一字未变（m_triple_q 中位 0.26mm / p95 0.66 / 98%≤7.5mm）。
   ⚠️⚠️ **「白心救回」（`rescue_hole`，2026-10-16 加，只给 pi 那条路开）**：
   `r_max` 那条绝对尺寸线的假设是"合并块也远小于 200px"，可**靶整颗贴在同色背景上**时
   两者合成一个**巨块** —— 实测（CSI 帧，蓝靶贴蓝柜子）：合并块 **r=1114px / 1.7M 像素** ✗
   远超上限，而**内孔仍是靶盘的白心**（q=0.089 ⇒ 反推外径 ≈180px，与旁边红靶的 176 一致）。
   不救的话那一色**连候选都不是**（共识/兜底都碰不到它）⇒ 现象 =「画面里明明有蓝靶，
   蓝色却整色认不出」。
   现在（`_color_candidates(rescue_hole=True)`）：被 r_max 挡掉的大块，只要**拟合到内孔**
   且**按内孔反推的外径 ≤ r_max**，就当候选收进来 —— 圆心/半径**都按白心算**
   （`_locate_one` 的内边界优先正好也走 hole_fit，两边口径一致），并打上 `merged` 标记；
   末尾那把兜底对它**不再拿 q 卡**（它的 q = 内孔÷巨块外接圆，天生很小：实测 0.089，
   而地板是 0.05 —— 再大一点的合并块就会被地板挡掉，等于白救）。资格在候选级已经验过
   两条硬的：①有内孔 ②反推外径合理；背景大块（墙洞/椅背光影）几乎过不了 ②。
   ⚠️ **为什么只给 pi**：主摄那条路要**保持既有行为逐字节不变**（另一台相机上"巨块+洞"
   更可能是真背景物）；而且 CSI 满分辨率下"同色大面积背景"的概率高得多。
   ⚠️ 验收（2026-10-16 实测, `logs/run_20261016_061938/frames/target_0002.jpg`）：
   开关关 = 只认出 G/R 两颗；开关开 = **三颗齐**（`B(1430,1068)r181` 落在白心上 ✓）;
   757 张**主摄**归档帧 A/B = **0 处差异**（开关关 ⇒ 与改造前逐字节相同 ✓）。
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
- `tasks/_road.py` — 看路（命令 `"G"`）：主摄看地面，Y−B **自适应阈值**（直方图谷底，换灯/
  换图不用重标）分割黄色区，测弯道交点与右切点的 Δx/Δy，回 0x08 帧（§6i）。
  其掩膜函数 `_warm_mask` 也被 `_road_yaw` 复用 —— 「什么算黄区」只有一个出处。
- `tasks/_road_yaw.py` — 看路校准（命令 `"H"`，2026-10）：拟合黄区**直边的图像方向角**，
  给单片机不依赖陀螺仪的航向读数，回 0x09 帧（§6j）。**只测不改**：落地/对齐由固件做。
  详见「看路校准」节。
- `tasks/_bucket.py` — 爆爆桶坐标转发（命令 `"J"` 起流 / `"K"` 停流，2026-10-24）：
  **纯转发** OpenMV 的 `B,<valid>,<cx>,<cy>,<w>,<h>` 文本帧 → COORD TAG `0x0B` 流帧
  （§6m）。检测全在相机端；无桶/陈旧也照发（流活性靠帧计数）。
- `bench_qr.py` / `grab_qr.py` — 二维码跑分与攒样张（见「跑分 / 攒样张」节）。
- `test_picam_reap.py` — CSI 遗留 `rpicam-vid` 回收的单元测试（唯一带断言的自检）。
- `tools/replay_target_tune.py` — 靶标离线回放调参（候选死因 CSV / 扫 `TARGET_R_MAX`）。
- `color_calib.py` — 色球 HSV 阈值标定（离线图或在线主摄）。
- `test_camera.py` — 双摄像头测试脚本（默认 web 推流预览；`--no-preview` 读帧统计）。
- `gks.service` — systemd 服务文件（`SERVICE_MODE=1`，`Restart=always`）。
- `../GKS 控制台.desktop` — 桌面上的控制台启动器。

## 摄像头

- 设备节点（与 GCS/cv_python 相同，由 udev 规则 `/etc/udev/rules.d/99-gks-devices.rules` 创建）：
  - `CAM_MAIN_INDEX = /dev/camera_main`（主摄像头）
  - `CAM_QR_INDEX = /dev/camera_Qr`（扫码摄像头）
- 均 `MJPG`、`on_demand` 生命周期，取流 **1920x1080**（`config.FRAME_WIDTH/HEIGHT`；
  旧文档写的 640x480 已过时）。
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

## pi 摄像头（树莓派 CSI）—— `camera_picam.py`，2026-10-16 起供识别用

第三颗相机：CSI 排线上那颗（本机 OV5647），走 **libcamera**（`rpicam-vid` 出 MJPEG，
Python 侧按 SOI/EOI 切帧）。**不能用 `cv2.VideoCapture` 打开**（内核只给 unicam 原始 Bayer 节点，
没有 ISP 管线），也**不按 `/dev/videoN` 找它**（编号会漂）—— 用 `rpicam-*- --list-cameras` 认传感器。

**为什么接进识别**：主摄装在**机械臂**上，臂位姿与残余振动会让「瞄准列」漂；CSI **固定在底盘上**。
所以「平移射靶」（`"6"`/`"7"`）优先用它看靶，它不行时**自动退回主摄**。

| 件 | 说明 |
|---|---|
| `PICAM`（模块级单例） | 预热/持有器：`warm(uses)`(**非阻塞**，后台起流+丢前 `PICAM_WARM_FRAMES` 帧等 AE/AWB 收敛)、`acquire(wait_s)`→句柄或 None、`read()`→(ok, BGR)、`alive()`、`release()`(只续期)、`close()`(真 terminate) |
| `picam_watchdog()` | 1s 一拍的空闲看门狗（`main.py` 起线程）——**不能省**：发了预热却没跟进识别就永久占着 ISP，网页预览从此打不开且无人报错 |
| 与 `camera_lease` 的关系 | **没有关系**（那是两颗 USB 相机的单槽租约，收一颗会放另一颗）。CSI 独立 ⇒ **两台可以同时热着**，`"C,<uses>,1"` 不会顶掉主摄的租约 |
| 识别参数 | `config.PICAM_TARGET_WIDTH/HEIGHT/FPS/QUALITY`（**2592×1944 = 4:3 原生满视场**，见下面那条 ⚠️⚠️；单片机 `tc_cx_aim_pi` 按 **0~2592**、`tc_cy_aim_pi` 按 **0~1944** 钳位，全工程坐标都是"全分辨率像素"）、`PICAM_AWB_MODE`/`PICAM_AWB_GAINS`（色识别飘先动它——主摄当年正因 AWB 漂移被固定 6500K）、`PICAM_TRY_BUDGET_S`/`PICAM_STREAM_UP_S`/`PICAM_STALE_S`/`PICAM_IDLE_TIMEOUT_S`/`PICAM_USES_MAX` |
| 手动扫描的预算 | 网页「靶标识别(pi)」**不受单片机时限约束** ⇒ 由 `GKS_Web/app.py` 的 `PICAM_MANUAL_WAIT_S`(等起流) / `PICAM_MANUAL_TRY_BUDGET_S`(CSI 子预算 8s) / `TARGET_MANUAL_TIMEOUT_S`(总 14s) 顶着，经 `run_target_detect(..., pi_budget=)` 传进去。⚠️ 仍走 `config.PICAM_TRY_BUDGET_S`(2s) 的话，满分辨率 0.6~1.8s/帧只够 1~3 帧 <2s ⇒ 凑不够 `TARGET_MIN_VOTES`(3) 票 ⇒ **每次点都退回主摄**（拿主摄 1920×1080 的 cx 去填 `tc_cx_aim_pi` 会填错一整列） |

- **退回主摄是单行道**：`tasks/_target.py::run_target_detect(use_pi=)` 先试 CSI（最多
  `PICAM_TRY_BUDGET_S`，**剩下的预算留给主摄**——不然 CSI 看不到靶会把单片机给的时限吃光，
  退化成"主摄根本没机会跑"）；`tasks/_target_stream.py::run_target_stream(use_pi=)` 同理，
  起不来/中途掉帧 → **整条改用主摄重跑**。⚠️ 一旦回落**不再回试 CSI**（接触不良的排线会让
  回帧那个 cam 位在 20Hz 上反复跳，单片机每拍重置计数器，永远收不敛）。
- **回帧要带"哪台测的"**：0x04/0x05 帧 `status` 的 **bit7**（`build_target_frame` /
  `build_stream_frame` 的 `cam_is_pi`）。**帧长/布局一字未改** —— 加字节会让老单片机
  把目标数组读错位（静默错值），挤位只会让老固件把 pi 成功判成失败（响亮、不动车）。
- ⚠️⚠️ **视场/坐标系（2026-10-16 换过一轮，别照旧笔记）**：侧装那颗 CSI 取流写
  `PICAM_TARGET_WIDTH/HEIGHT = 2592/1944` + `PICAM_TARGET_FPS`（出厂 12，2026-10-24
  从 10 提上来；满分辨率档实测上限 ~15，写 20 会起流失败），`PICAM_TARGET_QUALITY=75`。
  ① **请求高必须 > 1080**：写 1920×1080（或 1440×1080）时 libcamera 都挑 1920×1080 那档
     **16:9 窄视场档**，把 4:3 传感器的纵向**裁掉 25%** ⇒ 三个靶排开时最外面那个会出画；
     把高写成 1944 才会落到**原生满视场**档（拿"画面里靶的直径"量过：换档前 55px→换档后 238px 同距离）。
  ② **`PICAM_ROTATE` 现场是 0**（`config.py` 里那句 90 是"侧装默认"的旧值，实际被
     `config_override.json` 的 `"PICAM_ROTATE": 0` 盖掉 —— 两个进程都吃 overrides）。
     所以**交付给识别的坐标系就是 2592×1944**（瞄准列 0~2592、瞄准行 0~1944），
     单片机 `tc_cx_aim_pi`/`tc_cy_aim_pi` 的钳位也按它写的。改尺寸/角度 = 换坐标系 ⇒
     瞄准常数重标 + 单片机钳位跟着改（网页那台**预览**不转/不裁，永远直通 —— 它只给人看）。
  ③ 满分辨率下采一帧 **0.6~1.8s**（降频时更慢）⇒ 任何"给 CSI 的预算"都要按这个量级算，
     2s 只够 1~3 帧（见上面「手动扫描的预算」那行）。
- ⚠️⚠️ **「pi摄像头: 失败（抓帧失败: failed to acquire camera …）就是「相机被占着」**，
  **不是相机坏了**（坏了/没插报的是"libcamera 没看到 CSI 摄像头"）。libcamera **独占**：
  谁先 open 谁赢，后到的只能拿到这句。现存的四个持有者与解法：
  | 谁 | 怎么认 | 怎么解 |
  |---|---|---|
  | 网页的 pi 预览（本进程） | `_pi_stream` 活着（检查会直接报"预览中"） | 关掉预览；`/abort?cam=pi` 现在**真会收掉它**（`_handle_abort` → `_stop_pi_stream`） |
  | **main.py 的 PICAM 持有器**（另一个进程） | 刚跑过 `"6,..,1"`/`"7,0,1"`/`"C,..,1"` —— 空闲窗是 `PICAM_IDLE_TIMEOUT_S × 预热次数`（默认 15s，`uses` 大就几十秒） | 等窗口到期；`/abort?cam=pi` **够不着它**（没有反向通道），重试也是白等 |
  | 退不掉的 `rpicam-vid`（僵尸/孤儿） | `pgrep -a rpicam` 看得见 | `kill <pid>`；`systemctl restart gks.service` 也会按 cgroup 一起收 |
  | **自己上一次运行留下的取流进程**（换镜像重启 / kill -9 / 掉电） | `pgrep -a rpicam` 里 `argv[0]` 是 **`rpicam-vid-gks`**（识别流专用标签），或形状吻合且 PPID 是本进程/1 | **2026-10-16 起全自动**：`camera_picam.reap_stale()` 在启动时、每次起流前、网页「自检」前收掉它 —— 不必再手工 kill（见下两条） |
  诊断一行：`pgrep -a rpicam`（`probe()` 失败时也把它拼进给界面看的那句话里）。
- ⚠️ **谁都不许漏收 `rpicam-vid`**：它是**子进程**，父进程没了它还活着。所有"换进程镜像/退出"
  的路径都收：`handler` 的 `"end"`/`"r"` 分支、`_safe_restart_program`（接收线程侧的救援）、
  **`SerialInterruptHandler.run()` 的链路故障自愈（2026-10-16 补 —— 这条就是 2026-10-16 现场
  那次 `failed to acquire camera` 的根因：它 `os.execl` 前没交还相机）**、`main.py` 退出前那
  一小段（都是 `camera_picam.PICAM.release_now()`）。漏一条 = 那颗相机**从此谁都开不了**，
  只能人工 kill 或重启整机。
- ✅ **兜底自愈（2026-10-16 加）**：万一还是漏了一条（或进程被 kill -9 / 掉电带走），
  `camera_picam.reap_stale()` 会把"**我们自己留下的**"取流进程收掉。判据是**三选一**
  （① `argv[0]` 带 `OWN_TAG`；② 识别流的命令行形状 **且** PPID = 本进程 —— 换镜像不改 PID，
  所以"上一次那个我"留下的子进程父名不变；③ 同样的形状 **且** PPID = 1 —— 父被 kill -9 后
  被 init 收养），再加一条**排除**（本进程**在用的**流登记在 `_LIVE_STREAMS`，弱引用：
  对象被丢掉就自动失效，那种"没 close() 就丢弃"的子进程会被重新当成遗留收掉）。
  调用点：`main.py` 启动、`PicamSource._open_now`（起流失败且 stderr 含 `acquire` 时**先清
  自己再重试**，之后才轮到"请网页让位"）、`probe()`（网页自检按钮）、看门狗每 10s。
  ⚠️ **不会误伤网页预览**（它是 GKS_Web 进程起的 → PPID 对不上、也不是孤儿）。
  判据有单测钉着：`python3 test_picam_reap.py`（喂假 /proc 树 + 真 kill 一个子进程）。
- ⚠️ **闸门顺序：「已有热流」在先，「失败退避」在后（2026-10-16 修）**：`acquire()` 与 `warm()`
  的**第一件事**是看手上有没有 ready 的热流 —— 有就直接交付/续期，**完全不看退避**。
  写反了会出这种现场（2026-10-16 实测）：`"6"` 的 CSI 尝试失败（相机那时真被占着）→
  `note_failure()` 记 10s 退避；同一条 `_open_now` 的自愈（收遗留 + 重试）1s 后**真把流
  起好了**（日志有「预热完成: ov5647 1920x1080@20fps」—— 那时还是**旧档**，2026-10-16 才换成
  2592×1944@10）)；紧接着 `"7,0,1"` 进门 → 撞在退避上
  → 立刻 None → 日志「pi(CSI) 起流失败」并退回主摄 —— **而相机明明热着、帧就在手上**
  （操作员一眼判据：跑完这一轮去模拟器/网页里看 pi 画面又是好的）。
  配套三条：① `_adopt()` 起流成功即**清退避**（刚起好就没理由继续退避）; ② `warm()` 遇到
  已在跑的流只续期并回 True（退避不该拦"这台已经在跑"）; ③ `acquire()` 的等待循环**不再调
  `use_pi()`**（它会跑 `rpicam-hello --list-cameras`，而相机被自己占着时那条会阻塞到
  超时 —— 冷启动那次 acquire 会被它吃掉一大截预算）。
  这几条有断言钉着：`python3 test_picam_reap.py` 第 ⑤ 节（桩流 + 退避期）。
- ⚠️⚠️ **判活看的是「流自己的出帧时刻」，不是「我们取过帧」（2026-10-16 修，这条最阴）**：
  `PICAM.alive()` / `read()` 里的"多久没新帧"原先读的是 `PicamSource._last_frame` —— 而它
  **只在 `read()` 里刷新**。于是"**预热好、还没人用**"的那段（正是预热的意义！）判据恒为假：
  现场 05:28:07「pi摄像头(CSI) 预热完成」→ 05:28:10 `"6,12…,1"` 一进门 `alive()` → False →
  「pi(CSI) 取流中断 → 交给主摄重试」；05:28:14 的 `"7,0,1"` 同样「中断（不再出帧）」退回主摄。
  用户判据很直接：**跑完这一轮去网页/模拟器里看 pi 画面又是好的**（预览那条路不用 PICAM）。
  现在真相只有一个：`PicamStream._last_frame_t`（**读线程**每出一帧打一次）+ `last_frame_t()`
  访问器；`alive()` 与 `read()` 都看它。⚠️ 反向也要守住：子进程半死（管道在、不再出图）时
  必须仍然判"陈帧"拒发（否则把同一张老图一路喂给检测器 → 检测器稳稳报同一坐标）。
  `_last_frame` 那个字段**已删除**（两处真相 = 迟早对不上）。
- ℹ️ **"收不掉遗留取流进程"的可能性（2026-10-16 记一条）**：`reap_stale` 收不掉的常见两种 ——
  ①进程卡在不可中断状态 D（在 libcamera 里等设备）; ②**不是同一个用户的进程**（手工
  `sudo python3 main.py` 起的流属于 root）。消息里会带 pid + 命令行 + `sudo kill -9 <pid>`
  的提示（同一 pid 60s 内只报一次，免得看门狗刷屏）。
- ⚠️ **网页那个 pi 预览会挡住识别**（预览也是 `rpicam-vid`，libcamera 独占）：
  起流失败会被吞掉并退回主摄，**现象只是"一直用主摄"**。要真用 CSI 就先关掉预览。
- ⚠️ **`"7"` 现在也进 `_MS_COMMANDS`**（它唯一的数字语义是 `is_use_picam`）；
  允许两段数字的只有 `"6"`/`"7"`/`"C"`（`handler.py::_TWO_NUM_HEADS`），
  形状解析在 `_split_payload_ext`（老入口 `_split_payload` 仍回 2 元组，~20 处调用方没动）。
- 台架自测：`python3 camera_picam.py`（列相机+抓帧+预览存一帧）。

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

⚠️ **带档位预热 `"W,<cam>,<uses>,<prof>"`（2026-10-23 加）**：`cam` 与 `prof` 是**正交**两轴
（cam 决定开哪台设备，prof 决定曝光/白平衡档）。`cam` 码：0 主摄 / 1 扫码副摄 / 2 只 CSI /
3 主摄+CSI；`prof` 是按 `uses` 打包的多档位编码，越界退回默认档（不是不预热）。
它走 `_dispatch_warm_w`，**不走** `_split_payload`；CSI 忽略 prof（白平衡在
`PICAM_AWB_MODE/GAINS`）。默认档仍走老的 `"B"/"C"`，老固件一个字节都不用改。

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

**当前命令表全貌**（`handler.py::_COMMANDS`，权威源；文档只是镜像）：

| 类别 | 命令 | 说明 |
|---|---|---|
| 任务（同步阻塞） | `"1"`/`"D"` | 二维码（`"1"` 副摄 / `"D"` 主摄，新引擎 `run_qr_auto*`） |
| | `"E"`/`"F"` | 自研难度二维码（副摄 / 主摄） |
| | `"4"`/`"5"`/`"6"` | 人质 / 色球 / 靶标（一次性多帧投票） |
| | `"A"`/`"G"`/`"H"` | 矩形地板 / 看路 / 看路校准 |
| | `"Z"` | 四路相机健康检查（2026-11 加，回 0x0D 帧；查完解除占用；握手积木勾选时发） |
| 长任务流 | `"2"`/`"3"` | 巡线起/停 |
| | `"7"`/`"8"` | 靶标流起/停 |
| | `"J"`/`"K"` | 爆爆桶流起/停（2026-10-24） |
| 预热 | `"B"`/`"C"`/`"W"` | 扫码副摄 / 主摄 / 带档位预热 |
| 参数/自动二维码 | `"P"`/`"QA"`/`"QE"`/`"QU"`/`"QC"` | 取参/临时设参；自动下发二维码族 |
| 控制/杂项 | `"x"`/`"L"`/`"T"`/`"RM"`/`"RS"`/`"CL"` | 取消 / 补光亮度 / 补光开关 / 重启 main / 重启服务 / 清屏 |
| 系统 | `"end"`/`"r"`/`"sd"`/`"rb"`/`"9"` | 退出 / 重启程序 / 关机 / 重启 / 拍照 |

⚠️ `"P"`/`"W"`/`"QA"`/`"QE"`/`"QU"`/`"QC"` 在**查 `_COMMANDS` 之前**被 `_dispatch_cfg` /
`_dispatch_warm_w` / `_dispatch_qr_auto` 拦走 —— 它们的字段形状（参数名 / 三段数字 /
开关位）`_split_payload` 拆不了。新增同类命令照此办理，别塞进通用拆分表。

**新增一条指令（如 `"J"` 爆爆桶）要动 4 处：**
（⚠️ 单字符 `"1"`~`"9"`、`r`/`end`/`sd`/`rb`/`x`/`L`/`T`、`RM`/`RS`/`CL`、`E`/`F`/`A`/`W`
都已占用 —— 新指令先查 `_COMMANDS`）

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
   - ⚠️⚠️ 那张白名单现在是**多族共用**的（当前 `_MS_COMMANDS` 含
     `1/D/E/F/G/H/A/4/5/6/7/J/L` 等，2026-11 起还有 `Z`）：任务命令那个数是**毫秒**；预热 `"B"/"C"` 是
     **使用次数**（相机租约窗口的倍数）；`"L"` 是**补光亮度 0~100**；`"W,<cam>,<uses>,<prof>"`
     是三段、不走通用拆分。往白名单里加新命令时**先想清楚自己属于哪一族** ——
     照着字面把毫秒抄过去就是静默错权重。
4. `../GKS_Web/simulator.py::COMMANDS` — 网页指令按钮表（三处命令码须与 `docs/串口协议.md` §6 一致）

**任务函数必须能收 `timeout=`：** 从串口进来的指令由 handler 用
`run_fn(timeout=_budget_from_ms(ms))` 调用（见 `_cmd_qr_common`）。新写任务函数时
签名照 `run_qr_wechat(timeout=None, abort_event=None)` 那样留出这两个关键字参数 ——
否则 `handle()` 那条带时限的分支会直接 TypeError。

**⚠️「只换相机、结果同构」的命令要复用同一帧型（`"D"` 是范例，2026-09 加）：**

四条扫码命令（`"1"` 副摄+新引擎 / `"D"` 主摄+新引擎 / `"E"` 副摄+自研 / `"F"` 主摄+自研；
旧微信链路仍在 `run_qr_wechat*`，只作 zxing 缺轮子时的回退与对照）返回**同一种帧**。

其中 `"1"` / `"D"` 差别**只在用哪台相机**，协议层完全同构：

| 命令 | 任务函数 | 相机 | 结果帧 |
|---|---|---|---|
| `"1"` | `_qr.run_qr_auto` | 扫码副摄 `/dev/camera_Qr` | §6a 的 9B ASCII |
| `"D"` | `_qr.run_qr_auto_main` | 主摄 `/dev/camera_main` | **同一个** §6a |

- 两条都经 `handler.py::_cmd_qr_common(run_fn)`，`_cmd_qr` / `_cmd_qr_main` 各自只传一个
  run_fn。**别把回传/存最后一帧那套复制成第二份**。
- **不要为它新增结果帧**（不加 TAG、不加 §6x）：单片机侧就是靠"格式相同"才做到
  `ucRpiWaitQrResult` 与下游（IF 条件/排爆/反恐）一个字都不用改。加"只换相机"的命令
  一律照此办理。
- ⚠️ **`"D"`/`"E"`/`"F"` 都必须进 `handle()` 里那个互斥元组**
  （**实际是 10 项**：`if head in (b"1", b"4", b"5", b"6", b"A", b"D", b"E", b"F", b"G", b"H"):`
  —— 本文档旧版只列了 8 项、漏了 `"G"`，别照那份清单照抄）
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

- 挑一个 `< 0x20` 的 TAG（**0x01 巡线 / 0x02 人质 / 0x03 色球 / 0x04 靶标 / 0x05 靶标流 /
  0x06 拍照 / 0x07 矩形地板 / 0x08 看路 / 0x09 看路校准 / 0x0A P 族参数回帧 /
  0x0B 爆爆桶流** 已占用；ASCII 帧首字节 ≥ `'0'`，天然不冲突）
- 单片机 `app_rpi.c::vRpiTask` 的 COORD 分发加分支，且**必须放在通用 `LEN >= 9` 分支之前**
  —— 否则会被当作「9 字节 last packet」吞掉，永远到不了解析逻辑
- 长度/字段用**小端** `struct.pack("<...")`，与巡线帧同约定

**生效方式**：改 `GKS_Competition/` 要重启 `gks.service`（或让单片机发 `"r"`）；
改 `GKS_Web/` 要重启 GKS_Web 服务（模拟器是网页服务里的线程）。

## Pi 端参数 `"P"` 族 / 自动下发二维码 `"QA"` 族（docs §6k / §6l，2026-10 加）

- `"P?,<NAME>"` 取参、`"P,<NAME>,<VALUE>"` **临时设参**：只改 **main.py 进程内存**
  （`config.set_temporary()`），**不写 `config_override.json`**，重启还原，也**不影响
  GKS_Web 进程**（页面显示不会跟着变）。名字必须在 `EDITABLE_CONFIG`，值按类型转换，
  NaN/Inf 与非法选项一律回 `ERR`。回帧是 COORD TAG `0x0A` 的 ASCII（`V/OK/ERR`）。
  要持久化仍走 Web 参数页（写盘 + 重启生效）—— 两条路的语义别混。
- `"QA"` 查询 / `"QE,<0|1>"` 开关 / `"QU,<ddd>"` 上传并启用 / `"QC"` 清空：
  读写 `web_auto_qr.json`（与 GKS_Web 同一份真相），握手后 Pi 主动推 `QP<ddd>` 伪装帧。
  `QU` 的值必须三位且每位 1~3；回帧同为 TAG `0x0A` 的 `QS,<e>,<值>`。
  这四条在 `_COMMANDS` 查表**之前**由 `_dispatch_qr_auto` 拦走。

## 看路校准（命令 `"H"`，2026-10 加）

`tasks/_road_yaw.py` —— 主摄拟合**黄区直边的图像方向角**，给单片机一个**不依赖陀螺仪的
航向读数**（抗 yaw 漂移）。物理依据：黄区固定在场上、相机固定在车上 ⇒ 直边在图像里的
方向**只随车航向变**（直线方向对平移不敏感）。结果回 **0x09 帧（8B）**，见
`docs/串口协议.md` §6j；任务码 `"H"`（阻塞类, 毫秒后缀, 互斥元组第 10 项）。

- ⚠️ **本任务的帧只携带测量值**（Pi 侧始终只测不改）; 落地由固件**对齐到相机真值**
  （2026-10-08: bias = wrap180(原始航向 − err) ⇒ 发布 yaw ≡ err, **每次校准后全场航向
  零点 = "放直"方向**, 首次也不豁免; 编译期 ROADYAW_APPLY_EN=1 时发布 yaw 统一扣）。
  修正走「到点离散改偏置」, 不硬置 yaw、不在行车中改。⚠️ **世界坐标系不受影响**
  （固件把世界航向记录换算回原始值）—— 校准只劫持闭环航向与显示用的那个陀螺值。
- ⚠️ **瞬态失败自动重采**（`ROADYAW_ATTEMPTS`, 默认 **3** 轮）：散布超限/剔离群后不足/
  有效帧不足这类瞬态失败在**同一时限内**自动重采（每轮 ~1.5s, 超时照旧卡死）；双簇
  （真两条直边）不重采, 直接失败。路线超时建议 ≥6000（现默认即是）。
- ⚠️ **掩膜没有自己的一套**：整体复用 `_road.py` 的 `_warm_mask`（Y−B 自适应）与
  `ROAD_WARM_*`/`ROAD_PEAK_FRAC` —— 「什么算黄区」只有一个来源。自己的旋钮只有
  `ROADYAW_ROI_*`（人工罩住一段**直边**）、质量门（`ROADYAW_MIN_SPAN_PX` /
  `ROADYAW_MAX_RMS_PX` / `ROADYAW_MIN_COLS`）与投票三件套（° 口径）。
- ⚠️ **窗边框点会被自动剔除**（`ROADYAW_EDGE_MARGIN_PX`）：黄区通常比窗大, 掩膜被窗
  裁断处轮廓会沿窗边框走一段, 而窗边框是**车固连**的直线 —— 混进拟合 = 测到一个常数
  （正是"拿车身当基准"那个死胡同的缩影）。窗被黄区填满时边界根本没穿过窗, 会报
  「边界点太少」—— 挪窗, 别调门限。
- ⚠️⚠️ **方向角是 mod 180° 的量（2026-10-08 实测踩坑后加的投票/err 纪律）**：
  +89.9° 与 −89.9° 是**同一个朝向**（差 0.2°, 不是 180°）。本机位上篷布下边缘是
  **近水平**直边 = 正好贴着 ±90° 边界 —— 画面再纯净, 相邻两帧读数也会在 ±89.9 之间跳,
  直接拿原始读数投票会被判成「两簇差 180°」或「散布 179.9° 超限」而失败
  （03:33 那次失败就是这个, 不是掩膜问题）。现在投票按**圆均值**
  （`0.5·atan2(⟨sin2θ⟩,⟨cos2θ⟩)`）折参考系后跑线性判据; 单片机 err 也按 mod 180
  折回（`fmodf` 三行）—— 遥测上 ang 在 ±90 两侧跳是**正常**的, 看 err 才是连续量。
- ⚠️ 角度口径：与**图像竖直方向**的夹角, 竖直=0°、向画面右倒为正、(−90,+90]；
  **与巡线帧的 angle 不是同一个东西**, 标定值别互相抄。**未标定哨兵在单片机侧是 999
  不是 0**（竖直直边的 0° 是合法值, 见 `GongKeCompetition/Application/app_config.h`
  的 `ROADYAW_ANGLE_IDEAL_DEG`）。
- ⚠️ 练习地图的黄区是**大圆弧**, 没有真直边：短窗拟合出的是切线方向, 配合"固定停车位
  标定"仍可用; 正赛黄胶带才是真直边。台架验证可贴一条直胶带。
- 加了 `"H"` 之后**三处清单**都齐了：`handler.py`（`_COMMANDS`/`_MS_COMMANDS`/
  互斥元组/`_FAIL_FRAME_SENDERS`）、`GKS_Web/simulator.py::COMMANDS`、
  `docs/串口协议.md`（§6j + 指令表）。

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
- 任务架构固定为：串口指令 → `handler` 分发 → `tasks/*` 任务函数（只返回 dict，**不碰串口**）。
  视觉参数集中在 `config.py`，且必须登记进 `EDITABLE_CONFIG` 才能从 Web / 单片机调。
- 标注图上写字**一律 ASCII**（`cv2.putText` 画不了中文）；结果帧用**小端** `struct.pack`。
- `*.bak-*` / `_bak-*` 是现场回滚快照，别删也别提交；`logs/`、`JiGuangLogs/`、`qr_corpus/`
  是运行数据，不在 git 里。
