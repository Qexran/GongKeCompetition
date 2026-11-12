# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目是什么

OpenMV（MicroPython）**单线巡线视觉程序** + 一套**离线闭环调试工具链**。摄像头侧装、镜头朝下，
交界线横向于画面。算法从画面里测出两个量并以串口文本帧发给树莓派：

```
"L,<offset>,<angle>,<valid>,<state>\r\n"   @115200
```

`offset` 归一化 [-100,100]（0 = 交界线在目标行），`angle` 单位 0.1°，`state` 见下文。
场地是「道路 + 边界区」二分（路面灰 / 边界黄，蓝/粉为出发/终止区）。

**目标工作流**：不上车完成问题的发现→定位→修复→回归。人只做两件事：标注真值、看渲染视频。
权威流程文档是 `labeltool/AI_闭环训练说明.md`（含资产地图、判读 cheat-sheet、红线），**先读它**。

## 单一事实源（最重要的架构约束）

- `main.py` 是**唯一允许修改算法的文件**，是相机盘 `E:\main.py` 的工作主副本。
  改动后部署 = 拷贝到相机盘 + **重启相机**（不重启不生效，踩过）。
- `harness/replay.py::load_from_main()` 直接 `exec` 主文件的**常量段和纯函数段**；
  `labeltool/server.py` 复用 replay。所以**永远不要在 harness / labeltool 里复制算法常量或纯函数**——改了不会生效，只会骗自己。
- ⚠️ `load_from_main()` 用**行首字符串锚点**切代码段（`FRAME_W, FRAME_H`、`DRAW =`、`CTRL_X`、
  `# ---------------- 串口`、`# 派生几何`、`# ==================== 测量核心`、`def lsq`、
  `def col_profile`、`def cross_row`、`# ---------------- 绘制`）。**重命名、重排或删掉这些行会让回放器/平台静默加载错代码段**。改主文件时避开这些锚点行。
- ⚠️ `harness/replay.py::Replay.step()` 是主循环（1~5 步 + 丢线分类）的 **numpy 平行实现**，
  统计量用 numpy 复算、逻辑逐句对齐。**改主循环的控制流必须同步改这里**，否则离线结论不再代表相机。
- `main.py` 的 `classify_lost()` 读模块全局 `last_offset` / `base_off`、写 `lost_near`；
  `Replay` 通过注入命名空间（`ns["last_offset"]` 等）满足这个约定。改动这些全局名会同时打断 replay 与 server。
- 参考色 `ref_bnd` / `ref_road`、出量滤波状态等都是跨帧状态；算法**有状态**，算第 i 帧必须从 0 跑到 i
  （`server.py::_algo_record` 为此做了会话缓存，只支持前向增量）。

## 常用命令

路径都以脚本自身位置推导，从任何目录跑都行。

```bash
# 无真值体检：跑两个视频，输出相机同款调试行 + 分段时间线 + 异常检测(标签翻转/offset 跳变) + CSV + 拼板
python harness/replay.py

# 只跑一个视频
python harness/replay.py --video ../校赛地图运行视频/校赛地图1.mp4

# 参数扫掠：临时覆盖主文件常量，不改文件（如 --set DELTA_GATE=20）
python harness/replay.py --set DELTA_GATE=20 --set BASE_RATE=0.3

# 标注/验证平台 :8765（或双击 启动标注平台.bat：起服务 + 开浏览器 + 打印 PID）
python labeltool/server.py

# 停服：双击 停止标注平台.bat（按端口 8765 找到 PID → /T 杀进程树），或直接关掉那个 cmd 窗口
# 服务跑在启动它的 cmd 窗口里，窗口与 PID 一一对应（刻意不开 Flask debug：reloader 会另起
# 子进程，窗口/PID 就对不上了）。验证页 ⟳ 重跑算法（R）会热重载 main.py，改算法不必重启服务。

# 闭环比对：预测 vs 真值 → tasks/<t>/report.json（summary + worst + st_bad + jit_bad）
curl -X POST http://127.0.0.1:8765/api/task/map1_push/compare

# 热重载 main.py + 清空回放会话缓存（验证页那个按钮就是打这个）
curl -X POST http://127.0.0.1:8765/api/reload_algo
#   → {"ok":true} / 失败时 {"ok":false,"error":"SyntaxError: ...","stale":true}
#   stale=true 表示**仍在用上一次成功加载的算法**（宁可跑旧代码，也不让平台因一个手误起不来）

# 渲染验证视频（预测点/绿拟合线/青真值线/标签烧进画面）→ tasks/<t>/verif.mp4 + verif/
curl -X POST http://127.0.0.1:8765/api/task/map1_push/render
```

没有测试框架、没有 lint 配置。**回归验证 = `replay.py` 报告不比上一次差 + `/compare` 指标不退化**；
有真值改动时往 `labeltool/METRICS_LOG.md`（自建）追加一行「日期/改动摘要/off MAE/状态判对率」看趋势。

`main.py` **不能在 PC 上运行**（`import sensor, image, pyb, machine` 是 OpenMV 专有）。它只能被
`load_from_main()` 分段 `exec`，或在相机上跑。`photo` / `openmv_serial.py` 等相机盘/Pi 侧文件不在本仓库。

## 测量核心（`main.py`）

固定测量区（`MEAS_X/Y`）、顶部隔离带只取远侧色心、底部带取近侧色心。每帧：

1. **两侧色心**：上/下带 `(a,b)` 中位数 → `margin = max(|Δa|,|Δb|)`，`margin < MARGIN_MIN` 即判无交界。
2. **自动选判别通道**：a / b 里分得更开的那个。
3. **逐列距离剖面**：每列切 13 条横带取 `(a,b,L)` 均值，判别量
   `f = |条−上参考|² − |条−下参考|²`，`cross_row()` 找 `f` 过零行（取**邻域总落差最大**的那次穿越，
   落差不足不认）；再用 `refine_local()` 以交界近侧局部参考色细化一遍（抵消顶/底参考带被远端阴影、
   LED 过曝污染导致的列间拱形）。
4. **抗离群拟合** `robust_fit()`：穷举 RANSAC（8 列只有 28 对），固定门槛 `RANSAC_TOL_PX`（刻意不用 MAD/3σ）。
5. **出量**：优先黑线拟合 `fb`（练习图黑条中心最锐利），RMS 超限自动回落色边 `fa`；RMS 超
   `CURVE_RMS_PX` → `state=1` 并用**靠车近半段**内点重拟合；过中数+EMA 滤波（`FILT_MED_N` 默认关，
   实测开中数反而有害，原因见注释）；`TARGET_OFFSET` 把「摆正时的交界行」标定成 offset=0。

换测量核心/换出量源都会平移交界行定义 → 必须按 `angle_test.py` 流程重标 `TARGET_OFFSET`（该文件不在本仓库）。

## 丢线分类与方向定律（最易翻车处）

`state` 语义与真值标注**完全一致**：

| state | 含义 | 树莓派恢复动作 |
|---|---|---|
| 0 | 有效，正常直线 | — |
| 1 | 有效，交界是曲线（路口/弯道，已用近段重拟合） | — |
| 2 | 丢线：车在**路面灰侧**（压灰区） | 让 off **增大**（回边界） |
| 3 | 丢线：车在**黄区侧**（压黄区） | 让 off **减小**（回路面） |
| 4 | 丢线：说不清（交界还在画面里只是测不出） | 走旧保持策略 |

- 主判据（现行）：`Δ = 丢线时冻结的 off − 直线基线`。跌落 = 灰塌陷 → 2；抬升 = 黄塌陷 → 3；
  `|Δ| < DELTA_GATE` → 落到备胎（① ≥2 个零星点中位行 ② 两带颜色对参考色投票 ③ 4）。
  基线在**直线帧**里慢跟踪、偏出 `BASE_GATE` 即冻结，两个操作数在丢线瞬间都冻结 →
  **Δ 整段丢线期恒定**，这是「标签在丢线期来回跳」这个历史 bug 的结构性解法。
- **方向定律**（2026-09-27 第四次校准定案，前面几版已作废，主文件头有完整历史）：
  压向**被监控的**黄区 ⇔ off→正、边界行上升、从画面**底**出；离线入路内 ⇔ off→负、从**顶**出。
  ⚠️ 场地两侧都有黄区，穿过路面 = 离线（灰侧），别被对面黄区迷惑。
- 参考色只在交界行位于 `REF_SAFE_ROW [70,195]` 的**健康帧**里刷新（防被污染成对面）；
  「卡全灰默认报 2」「margin 冲突规则」两个兜底已被删除（都曾是翻车根因）。
- ⚠️ **部署顺序：Pi 端解析器（4/5 字段都收）必须先于相机端新帧格式上线**，旧解析器会把 5 字段帧整行拒收
  （症状：`bad_lines` 猛涨、数据永远陈旧、车不动）。
- 终端标签是给**人**看的（所在面：直线/直线·黄/直线·灰/路口弧/压黄区/压灰区/不明 + 丢线帧 `d=<Δ>`），
  `st` 数字是给**树莓派**的（找回方向）。两者解耦。

## 数据资产与红线

```
main.py                                  算法本体（工作主副本，唯一可改算法的文件）
校赛地图运行视频/*.mp4                    原始实拍视频（只读）
harness/replay.py, harness/out/          回放器 + 输出(CSV/分段报告/拼板)
labeltool/server.py + static/index.html  标注/验证/比对平台 :8765
labeltool/tasks/<任务名>/                 frames/ 去重内容帧 · annotations.json 人工真值
                                         meta.json · report.json(/compare 输出) · verif/ + verif.mp4
```

- **不要动 `校赛地图运行视频/` 和 `tasks/*/frames/`** —— 帧序列一变，标注全部错位。
- 真值口径：`line` = 该帧真实交界线两点（320×240，看不见就 `null`），`state` 同算法语义。
  真值 off/ang 由 `server.py::gt_from_line()` 用**主文件的公式与常量**换算，与算法同源、无口径差。
- 改算法前先 `cp` 一份时间戳备份到 `backups/`（自建目录），并在 `main.py` 头部变更记录追加一行
  （第几次 / 什么根因 / 什么改法）。头部那条「方向定律」注释链（第 1~7 次）是标定历史的唯一记录，**别删**。
- 平台任务名只允许字母数字下划线；`/compare` 的 MAE 只有在真值覆盖了足量**有效帧**时才有意义
  （任务刚建、只标了几帧时，MAE 会被少数极端帧主导）。
- **标注要连片标全，不要为了省工时抽样标**（用户 2026-09-28 明确）：角度抖动是「小幅度、高频」，
  `off_mae` / `off_max` / `st_bad` 全按幅度排序、看不见它；而**没有逐帧真值就无法区分
  「算法在抖」和「车真的在动」** —— 预测序列的方差里两个来源是混在一起的。标全给的是逐帧真值
  轨迹，能算 `残差 = 预测 − 真值` 的逐帧台阶与 std，才有分辨力。手画线的角度量化噪声约 0.1~0.2°
  （1px/370px → atan≈0.16°），历史抖动是 1° 量级，真值够用。
- **抖动指标已在 `/compare` 里**（2026-09-28 加，`server.py::_res_jitter`）：`summary.jitter.{off,ang}`
  给 `res_std` / `stair_std` / `stair_max` / `pred_stair_std` vs `gt_stair_std`（**键名自带单位**：
  ang 轴带 `_deg`、off 轴是归一化单位）；顶层 `jit_bad` 按**残差逐帧台阶**（不是绝对误差）排序；
  比对页有对应卡片与"抖动最猛帧"表。
  台阶只在**帧号相邻**的残差点对之间算 —— 跨丢线/未标的缺口不算一步。所以**零散标注算不出抖动**
  （`jit_bad` 只有一对台阶时 `stair_std` 就是 `-`，`n_stairs=0` 时全 `-`），必须连片标：
  这就是"不该抽样标注"的量化理由。判读：真值台阶平顺而预测台阶大 = 算法噪声；两边都大 = 车在动。
  检验 `FILT_EMA_A` / 黑线优先这类改动的**唯一正确指标是残差 std，不是 MAE**。
  未被覆盖的：`replay.py` 的无真值异常检测仍只查 offset 跳变（没有真值本就判不了抖动）。

## 环境与已知坑

- Windows + Git Bash；Python 3.14，依赖 `cv2` / `numpy` / `flask` / `PIL` 已装在系统解释器里，
  没有 venv、没有 requirements.txt。
- ⚠️ **Windows 非 ASCII 路径（2026-09-28 踩过）**：`cv2.imread` 读不到非 ASCII 路径 —— 返回 `None`，
  日志里路径会显示成"把 UTF-8 当 GBK"的乱码（如 `tasks/娴嬭瘯/`，`测试` 的 UTF-8 字节被按 GBK 解释）。
  原因：cv2 把路径交给 OpenCV 的 ANSI 文件 API。**读图一律走 `server.py::imread_u`**
  （`np.fromfile` + `cv2.imdecode`）；写侧 `cv2.imwrite` 同样不行，用 `cv2.imencode` + 文件对象写。
  **`cv2.VideoCapture` / `VideoWriter` 不受影响**（FFmpeg 后端吃 UTF-8），别顺手一起改。
  任务名允许中文（`str.isalnum()` 对 CJK 返回 True），所以这条路径一定会被走到。
- ⚠️ 改 `server.py` 本身**不能**靠验证页的 ⟳ 热重载（那只重载 `main.py`）—— 必须重启服务。
  ⟳ 只用于改算法的场景（那才是高频的）。
- OpenMV API 有版本漂移：`sval()` 兼容统计字段是属性(5.0)/方法(4.x) 两种；`DRAW_STYLE` 自检探测
  `draw_*` 吃元组还是散参数，两者都不行则关绘制但测量照跑。改绘制相关代码别破坏这些探测。
- `main.py` 主循环用 `try/except` 包住全流程并带 `stage` 标记 —— 出错也要继续发帧（`valid=0`），
  别让链路哑掉；任何新增的可选功能（如 `photo`）都要按这个模式做「失败只关自己」。
- 平台前端（`static/index.html`）：标注页 A/D 切帧、J 跳未标帧、拖拽/双击画线、Esc 取消；
  验证页播放 verif 帧序列；比对页出 summary 卡片 + 最差帧表 + 状态判错表。
