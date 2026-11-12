# AGENTS.md

工科赛小车：STM32F407 + FreeRTOS（裸 API V11.3.0）+ QEPack 驱动固件，树莓派 YOLOv8n 视觉引导，麦轮底盘 + 六轴机械臂；配套本地 Web 控制台（改路线/调参/录臂动作/OLED 菜单）与 Pi 端检测工程。

## 改对应区域前必读的文档

- **根 `CLAUDE.md`** — 全局铁律 + 文档索引（目录结构、串口入口、中断/并发、Beam、命名、安全姿态、脚本设计规则）。细节功能章节已按主题拆到 `docs/`，**改哪块先按根文件「文档索引」打开对应那份**。
- **`docs/*.md`** — 按主题拆分的功能细节（`uart` / `pi-link` / `bt-protocol` / `line-follow` / `target` / `grab-one` / `vision-tasks` / `road` / `chassis-misc` / `oled` / `sensors` / `console-metadata` / `reviews`）。这些**不会自动加载**，动对应模块前先读。
- **`tools/webconsole/CLAUDE.md`** — Web 控制台全部文档。改 `tools/webconsole/` 下任何文件、或改控制台会回写的文件（`map.c`/`map.h`、`app_config.h`、`app_script.h`、`app_arm_action.c/.h`、`app_oled_menu.c`、`tools/fontgen/glyphs.txt`）之前先读它。

## 目录结构

- `Application/` — 应用层，每模块一个 `app_*.c`；跨模块状态统一走 `app_beam`。`map.c` = 路线预设表（**纯数据**），`app_script.c` = 执行器；改路线只动 `map.c`。`app_config.h` = 全部可调宏。`app_chassis_action.c/.h` = 底盘动作组（时间轴速度混合播放器，数据区由控制台生成，同机械臂动作预设模式）。
- `Core/` — CubeMX 生成（`freertos.c` 建任务）；`Drivers/QEPack/` — 自研驱动（匈牙利命名）；`ThirdParty/` — FreeRTOS-Kernel。
- `MDK-ARM/` — Keil 工程入口 `GongKeCompetition.uvprojx`（**被 .gitignore 忽略**，只在本机 µVision 里存在）。**新增 .c 文件必须手动加进 uvprojx**（Application 组、锚定在相邻文件后），否则链接期报 Undefined symbol。
- `tools/webconsole/` — stdlib-only Python + 原生 JS 的 Web 控制台（十一个 Tab）。`chassis_action.py` = 底盘动作组表的解析/序列化（铁律同 `arm_action.py`）；`calib.py` = 「标定」Tab（主摄标定预设, 看路/色球/人质/射靶粗调 19 个宏按摄像头整套管理, 预设存 `calib.json`, 清单在 `tuning.py::CALIB_GROUPS`——这批参数不进「调参」Tab）；`moves.py` = 「位移」Tab（路线位移变量的可视化管理 + 示意图, 变量本体在 `app_config.h`「路线位移变量」段、示意图存 `moves.json`——`map.c` 的「默认路线」及其子预设的位移参数槽写的就是这些宏名）；`switches.py` = 「自定义」Tab（用户自建 IF 条件开关: 宏在 `app_config.h`「自定义开关」段 + `app_script.h/.c` 生成块, 清单存 `switches.json`）。
- `tools/fontgen/` — OLED 汉字字模生成（Pillow，只在造字时用，固件与控制台不碰它）。
- `vision/` — YOLOv8 训练（自带 `.venv`；`datasets/`、`runs/`、`*.pt` 不入库），README 在该目录下。
- `.pi_work/` — 树莓派端两工程的工作副本（`cv_python`、`GKS_Competition`，含各自文档）；Pi 端地址**随现场换网段变**（形如 `\\192.168.<网段>.125\shared\Desktop\GKS_Competition`，最后一段固定 `.125`，网段随现场变，用前先 ping）。`.pi_work\GKS_Competition` 是 gitlink（无 .gitmodules 映射），别 `git add` 它下面的东西。

## Pi 端协作（改 Pi 侧代码的流程）

- Pi 端工程**不在本仓库**，通过 SMB 共享目录直接读写：`tasks/_target*.py`、`tasks/_road.py`、`camera_lease.py`、`camera_picam.py`、`handler.py`、`config.py`、`logstore.py` 都在那边。
- **纪律：先备份再覆盖**（`copy 文件 文件.bak-<主题>`），改完用 SHA256 前缀比对校验落盘一致（SMB 会静默失败，`copy` 报"已复制 1 个文件"也要验）。Pi 离线时把待部署文件暂存到 `.pi_work\pending_*\`，恢复后推。
- 改 Pi 侧 Python 后**必须重启 main.py**（不热加载）；重启走 Web 控制台或让用户操作。
- Pi 本机（Windows 侧）有 cv2/numpy，可把 Pi 的真实帧拷回 `TEMP` 离线复放验证算法（`_road.py`/`_target.py` 都是纯函数，import config 桩即可跑），**先离线验证再上真机**。日志在 Pi 的 `logs/run_<ts>/log.txt`，`frames/` 存标注帧+原始帧（`*_before_*` 是训练模式原始帧）。
- 供电不稳会同时打掉 USB 相机与串口（`[Errno 5]`+`读帧连续失败`成串出现）——先怀疑电池/供电，再查代码。

## 常用命令

```bash
# Web 控制台测试（unittest，逐文件直接运行，全程沙箱不碰真实硬件/仓库外文件；
# node 可选，没装相关测试自动 skip）
python tools/webconsole/test_server.py        # 还有 test_bt / test_tuning / test_calib / test_arm_action /
                                              # test_oled_menu / test_capture / test_host_tools /
                                              # test_chassis_action，改 Python 或前端后都要跑

# 起 Web 控制台（端口 8410，绑 0.0.0.0，自动开浏览器）
webconsole.bat            # 或 python tools/webconsole/server.py
# 改完服务端代码必须重启它（allow_reuse_address=False，双开报端口占用）
```

- **固件构建**（命令行可用，别只靠控制台测试 + 目测）：`cd MDK-ARM && C:/Users/Administrator/AppData/Local/Keil_v5/UV4/UV4.exe -b GongKeCompetition.uvprojx -j0 -o build_<主题>.log` —— **UV4 退出码不反映成败，必须看 log 末行 `N Error(s)`**，项目约定 **0 Error / 0 Warning**（`-b` 依赖跟踪会重编改动头文件的下游 .c；拿不准就 µVision 里 Rebuild 全量）。

## 硬约定（违反会破坏现场调好的东西）

- **现场标定过的值不要顺手「修」**（`TARGET_*`、`ARM_SERVO_CURVE`、地图里 80 多度的转弯等——见 `docs/reviews.md` 里的「本轮明确不动」）。
- 串口收发唯一入口：读 `usUartDmaTakeCopy`、写 `vUartSendArray`；绝不能在临界区里发串口；`app_uart.c` 里的几道 RX 卡死/ErrorCallback 兜底是修过的真故障，**别当冗余删掉**。
- TX DMA 映射（`app_uart.c`）、补光灯 TIM9（`app_actuator.c`）等是**手写在 CubeMX 生成代码之外**的——重生成冲不掉是刻意的，别搬进 `.ioc`。
- 控制台 Python **stdlib-only**（唯一例外：`capture.py` 的可选 cv2）。别引 pyserial/Flask/numpy 等第三方依赖（蓝牙串口走 ctypes 直调 Win32）。
- `map_codec.py` 是 `map.c`/`app_arm_action.c` 行级解析/序列化的**唯一来源**，别抄回 `server.py`（有测试钉着）。
- 用户自由文本写盘前必须过 `map_codec` 的 `sanitize_*` 清洗；「无操作保存零 diff」是所有写盘模块的纪律。
- **别往 `Application/*.h` 加 `typedef enum`**（会被地图/OLED Tab 的枚举 glob 扫进下拉）；有意进下拉的除外。结构体条目的可选字段用**追加在末尾**的 `#define` 槽（同 `ucAbs`/`ucAccel`），别插中间——插槽会错位老数据行，追加则老行缺省补 0 天然兼容。
- **块注释里别写 `/**`**（如 `/**加粗**` 这种 markdown 强调）——C 注释不嵌套，AC5 报 `#9-D nested comment`，每个 include 的文件各报一遍。尾注释必须与成员**同行**（跨行会被 `parse_enums` 静默丢出下拉）。
- map.c 的整数参数带 `u` 后缀（`3u`）：**前端比对必须走 `optKey()`** 剥后缀再转数字，`Number("3u")` 是 NaN（真踩过：弹窗行数因此退化成 1）。
- 固件↔Pi 的数值约定（档位码/打包位域等）必须**两边同注释互指**（"数字必须与 handler.py::_WARM_PROFILES 一致"）；改协议先 Pi 后固件（老 Pi 收新形状=响亮拒收退老行为，反向=静默错）。
- 往 `app_config.h` 加参数必须带 DSL 尾注释（显示名/单位/范围）；段头必须是**单行** `/* ==== 名字 ==== */`。
- 新增路线预设一律**追加到 `map.c` 文件末尾**（id 顺序由文件顺序决定）。
- Web 控制台**无鉴权且绑 0.0.0.0**，蓝牙/机械臂 Tab 会驱动真实舵机——改动这些链路时把安全姿态当约束而非待办。
- **相机的"确认收敛"纪律**（`camera_lease.py`）: 换曝光档/冷开后帧要等"静默 + 连续 N 帧亮度稳定"才发放；改这段参数前先看日志 `换档收敛确认完成: 首帧延后 XXXms`（真实现场耗时）。租约健康判定在换档窗内有豁免（`_switch_until`），别删——现场两次换档叠加实测 3~4s 出不了帧。

## 底盘动作组（app_chassis_action + chassis_action.py, 2026-10 加）

- **两条数据通道要分清**: 预览 = 控制台经蓝牙推的 RAM 草稿（`$B,CPUSH`+`$B,CIT`, 断电即失, **disabled/raw 条目不推**）; 路线/比赛（`STEP_CHASSIS_ACTION_PLAY_WAIT`）**永远只读 FLASH 编译表**（`vChassisActionPlayWait` 只查 `c_pstChActFind`）——「台架试跑==比赛」靠的是保存+Keil 重编译+烧录, 别让草稿渗进路线。
- 播放器 20ms 一拍混合: 平移段向量**车体系直通**（+X=车头左, +Y=车头前, 不过象限表, 车主定案）+ 起步/收尾对称斜坡（同速同向交叠=恒速无缝）; 航向权威一拍只有一个（转弯/支点段 > 保持层）; 支点段（kind=4）独占车轮（平移让位）、驱动侧不插值、支点方向激活时按最短路径定死。
- 转弯/支点段**无段上限**（转到位为止, 组尾等收敛; `CH_ACT_TURN_MS` 是防故障兜底不是段参数）; 世界航向累加器只认 90° 网格、永远写指令目标; `CH_ACT_*` 出厂值引用 `PIVOT_TURN_*`（现场标定联动）。
- `$B,CIT` 的 kind 范围校验与 `CH_ACT_KIND_*` 一一对应——**加新段类型时 5 处要同步**: 固件枚举/播放器/CIT 校验、chassis_action.py 的 RE_ITEM+渲染+白名单、bt_link 的 kind_map、app.js 的 KINDS/COLORS/参数行。漏一处就是 CITBAD/静默丢行。
- 控制台 `h()` 对 `checked/disabled/value` 必须走属性赋值（setAttribute 会让勾选框锁死——踩过）; 字段被 UI 改过必须置 `edited`（否则保存按原始行回吐, 界面被旧版覆盖——踩过）。
- 前端布局: `#pane-chact` 是 flex 列（不是 grid——`#chact-main` 会掉进第 1 列留死白）; 右栏 340px 必须显式给; 全局 `.btn.tiny{margin-left:auto}` 要在本 pane 内压掉。

## Git 约定

- 提交信息用中文、conventional 前缀（`feat:` / `fix:`）。
- `*.bak`（控制台自动备份）、`bt.json`/`cap.json`（各机私有的硬件标识）**不入库**；`phrases.json`/`arm_trash.json`/`tuning.json`/`calib.json`（标定预设）、`moves.json`（位移示意图）/`switches.json`（自定义开关清单）是共享内容，**要入库**——`moves.json` 连着 `docs/DefaultMap.JPG`（示意图图片，一起入库），`switches.json` 连着 `app_config.h`「自定义开关」段与 `app_script.h/.c` 生成块，别只提交一半。
- `docs/`（按主题拆分的文档 + 图片）与 `.zcode/plans/` 之外的工作区杂项：提交前用 `git status` 过一遍，`Application/*.bak`、`map.c.bak-*` 这类手术备份别提交。
- 根目录的 `5000'`、`5)`、`-p`、`copy` 等是 shell 引号事故留下的垃圾文件，别提交。
