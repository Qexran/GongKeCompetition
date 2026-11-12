# 控制台元数据（全部从源码派生）

> 由根 `CLAUDE.md` 拆分；章节标题保持原样（源码注释按节名引用）。
> 相关：`tools/webconsole/CLAUDE.md`、根 `CLAUDE.md`「安全姿态」

## 控制台元数据 —— 全部以源码为数据源（勿在 server.py 里写死）

积木库、参数中文名/单位、下拉选项**一律从源码派生**，改源码控制台即自动跟着变：

| 控制台显示 | 来源 |
|---|---|
| 积木清单 / 分组 | `app_script.h` 的 `emScriptStepTypeTdf` 枚举 + `/* ---- 分组 ---- */` 注释 |
| 积木说明（积木名建议值） | 枚举成员的尾注释 `STEP_DELAY,  /* vTaskDelay(ms) 让出式延时 */` |
| 原语函数（行尾 `// xxx()`） | `app_script.c` 每个 `case` 体第一个函数调用 |
| 参数中文名 / 单位 / 备注 | `app_script.h` 参数映射注释的 DSL |
| 枚举参数的下拉选项 | 头文件 `typedef enum` 成员 + 成员尾注释作标签，类型由 `app_script.c` 里 `(emXxxTdf)I(n)` 强制转换确定 |
| 布尔参数的下拉选项 | **`app_script.h` 参数映射 DSL** 里显式 `{1=开,0=关}`，或提示里的 `1=X 0=Y` |
| 参数显隐（只在另一参数取某值时显示） | **`app_script.h` 参数映射 DSL** 尾部 `<显隐:参数名=值>` |
| 「蓝牙」Tab 的 Beam 参数面板 | 固件 `app_beam.c` 的 `s_astBeamParam[]` 注册表（**运行时**经 `$B,LIST` 上报，不是解析源码） |

⚠️⚠️ **表里的 `{值=标签}` 只适用于 `app_script.h`（脚本积木参数），不要照搬进 `app_config.h`**：在 `app_config.h` 的调参 DSL 里 **`{...}` 是「换算」而且是个闭集**（只有 `ms->Hz` 与 `‰->%`，见 `tuning.py::CONVERSIONS`）。在那边写 `{0=不对调,1=对调}` 的后果**不是显示难看，而是这个参数在「调参」Tab 里根本存不进去** —— 写盘抛「未知转换」、warnings 里带 `saved: false`，用户只看到一句 ⚠ 而值没落盘（2026-09 真踩过，两个布尔宏）。`app_config.h` 里布尔开关的正确写法是 **`@(0~1)` + 备注里写 `0=… 1=…`**（照着 `BALL_XY_SWAP` / `HOSTAGE_XY_SWAP` 那两行抄）。`test_tuning.py` 有两条守着它：`TestRealFile::test_no_unknown_conversions`（只读扫换算合法性）与 `TestRealFileRoundTripsThroughTheWritePath`（把真实文件复制出来**逐参数回写自身**）。⚠️ 只测「无操作保存」覆盖不到这类错 —— `write_config({}, [])` 什么都不改，根本走不到 `from_display`，只在写盘路径上炸的错全都能溜过去。

⚠️ **参数值比对要归一化整数后缀**（`app.js::optKey`）：写盘端 `_normalize_param` 会给非负整数补 `u`（`1` → `1u`），而下拉选项的 value 是 DSL 里的裸值（`1`/`0`）—— 直接字符串比对就会在 UI 里渲染出 `1u  (枚举外)`（用户报的就是这句；`转向`/`开合`/`使能`/`绝对角模式` 这类数字选项全中）。以后凡是要把「参数值」和「选项值」比一比的地方，都过 `optKey()`；`test_server.py` 有两条测试盯着（一条真在 node 里跑 `optKey`，一条数 `paramInput` 里两处比对是否都走了它）。
