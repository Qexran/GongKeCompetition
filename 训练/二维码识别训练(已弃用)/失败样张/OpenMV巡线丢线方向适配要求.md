# OpenMV 巡线丢线方向分类 —— 树莓派端适配要求

> 2026-09-26 整理。OpenMV 相机端（E:\main.py，相机盘）**已于 2026-09-26 部署完毕**，
> 原版备份在相机盘 `E:\main.py.bak-prestate`。本文档是树莓派端（GKS_Competition）
> 的配套适配要求，改完 Pi 端三个文件后整套方案才生效。

---

## 一、接口变更：串口帧从 4 字段扩为 5 字段

```
旧:  "L,<offset>,<angle>,<valid>\r\n"          @115200
新:  "L,<offset>,<angle>,<valid>,<state>\r\n"  @115200
```

第 5 字段 `state` 语义：

| state | 含义 | 树莓派侧应做的事 |
|---|---|---|
| 0 | 正常直线测量（valid=1） | 正常 PID |
| 1 | 有效但交界是曲线/路口（valid=1，OpenMV 已用近半段重拟合） | 正常 PID（信息性标记，不改变控制） |
| 2 | 丢线（valid=0）：边界远出画面上方 / 交界点全在上方（车偏进路里）。**整帧灰（两面分不开）也归它** —— 2026-09-26 车上实测标定：本场上全灰就是边界远出 | **向边界方向**横移找线 |
| 3 | 丢线（valid=0）：压到边界区（黄区）/ 交界点全在画面下方 | **向路内方向**横移找线 |
| 4 | 丢线（valid=0）：交界其实还在画面里（路口/混合帧，margin 还开着）或开机未采到参考色 | 走旧三段兜底（保持→固定方向→停） |

OpenMV 端分类依据（仅供理解，不需要 Pi 端复现）：
- 三级判别：①零星交界点的**中位行**（<155 报 2，>175 报 3）→ ②两侧参考色
  （有效帧上/下带色心 EMA）的 (a,b) 色度距离投票 → ③兜底：颜色没结论时
  margin 塌（相机没同时看到两面，整帧灰）按实测标定报 2；margin 还开着才报 4；
- state=1 的判定：出量拟合全局 RMS > 5px（直线帧只有 1~2px，不会误报）；
- 丢线时 offset/angle 仍沿用上一帧值（旧协议语义未变），**只有 state 是新信息**。

## 二、⚠️ 部署顺序与当前状态（重要）

- **旧版 Pi 解析器收到 5 字段帧会整行拒收**（`len(parts) != 4` → bad_lines 猛涨、
  数据永远陈旧、巡线一启动就按丢线/过期处理停车）。
- 相机端已先部署了。在 Pi 端更新完成之前，**不要触发巡线命令（单片机 "2"）**。
  OpenMV 只在收到 'G' 后才发帧，不巡线则完全无影响（扫码/色球/靶标等任务照常）。
- 若期间必须巡线：把相机盘 `main.py.bak-prestate` 拷回 `main.py` 临时回退。
- 正确顺序：**先改 Pi 三件套 → 再上电跑车验证**。

## 三、需要修改的三个文件

### 1. `openmv_serial.py` —— 解析器接受 4/5 字段（向后兼容）

**(a) `_parse_line()` 整体替换：**

```python
def _parse_line(line):
    """解析一行文本帧 "L,<offset>,<angle>,<valid>[,<state>]"。

    第 5 字段可缺（旧 OpenMV 固件），缺省为 0。
    返回 (offset, angle, valid, state) 或 None（非法行）。
    """
    try:
        text = line.strip().decode("ascii", errors="ignore") if isinstance(line, bytes) else line.strip()
        parts = text.split(",")
        if len(parts) not in (4, 5) or parts[0] != "L":
            return None
        offset = int(parts[1])
        angle = int(parts[2])
        valid = int(parts[3])
        state = int(parts[4]) if len(parts) == 5 else 0
        if valid not in (0, 1):
            return None
        if state not in (0, 1, 2, 3, 4):
            return None
        if not (-200 <= offset <= 200):    # 名义范围 ±100，留裕量防噪点
            return None
        if not (-2000 <= angle <= 2000):   # 名义范围 ±900 (0.1°)
            return None
        return offset, angle, valid, state
    except (ValueError, IndexError, AttributeError):
        return None
```

**(b) `OpenMVReader.__init__`** 增加一行缓存字段：

```python
        self._valid = 0
        self._state = 0        # ← 新增（丢线方向分类，旧固件恒 0）
```

**(c) `get_latest()`** 返回字典增加 `"state": self._state,`（docstring 同步提一句）。

**(d) `_run()` 读循环** 解包与缓存改为 4 元组：

```python
                    offset, angle, valid, state = parsed
                    with self._lock:
                        ...
                        self._state = state
```

### 2. `tasks/_line.py` —— 方向感知的定向找线（核心）

**(a) 新增辅助函数**（放在 `_clamp` 之后）：

```python
def _toward_steer():
    """「向边界方向横移」的 steer 符号（+1/-1），由 err→steer 同一条标定链推导：
    offset=-100 = 边界远在画面上方（车偏进路里），此刻 err =
    LINE_ERR_SIGN*(-100-SETPOINT)/100，其符号正是 PID 会输出的、把车带向边界的
    steer 方向。LINE_ERR_SIGN / LINE_SETPOINT 改了这里自动跟着变 —— 找线方向
    不再死写（旧版丢线一律按「踩到右边界」往一个方向找，偏进路里时越找越偏）。"""
    return 1.0 if config.LINE_ERR_SIGN * ((-100.0 - config.LINE_SETPOINT) / 100.0) > 0 else -1.0
```

**(b) 满偏分支**（`if abs(err) >= 1.0 and ...`）里方向同样改用推导（可选但建议，消除另一处符号死写）：

```python
                if abs(err) >= 1.0 and config.LINE_SEARCH_STEER != 0.0:
                    mag = abs(config.LINE_SEARCH_STEER)
                    steer = _toward_steer() * mag if err > 0 else -_toward_steer() * mag
```

**(c) 丢线分支**（`else:` 整段替换）。要点：
- state=2/3 且配置了搜索速度：先静默 `LINE_LOST_CONFIRM_S`（躲单帧毛刺，期间沿用
  last_steer），其后按 state 定向找线（大小取 `|LINE_SEARCH_STEER|`，方向由
  `_toward_steer()` 推导，state=3 取反）；
- state=4 / 未配置搜索速度：走旧三段（保持→固定方向兜底→停）；
- 两路都受 `LINE_LOST_HOLD_S + LINE_LOST_SEARCH_S` 总预算封顶，超时停（防跑飞）；
- 单片机在 valid=0 时照常执行 steer（app_line.c 现行行为），航向环自动切陀螺，
  所以找线帧不用伪造 valid=1，err/angle 照旧置 0。

```python
            else:
                lost_frames += 1
                lost_s = now - last_valid_t
                err, angle = 0.0, 0.0
                # state 只在数据新鲜时可信（OpenMV 断流时缓存是陈的）
                st = data.get("state", 0) if fresh else 0
                steer_mag = abs(config.LINE_SEARCH_STEER)
                budget_s = config.LINE_LOST_HOLD_S + config.LINE_LOST_SEARCH_S
                if st in (2, 3) and steer_mag != 0.0 and lost_s <= budget_s:
                    if lost_s < config.LINE_LOST_CONFIRM_S:
                        # 确认期：沿用车丢线前正在执行的 steer（它本就朝修正方向打）
                        steer = last_steer if last_steer is not None else 0.0
                    else:
                        toward = _toward_steer()
                        steer = toward * steer_mag if st == 2 else -toward * steer_mag
                        last_steer = steer
                elif last_steer is not None and lost_s <= config.LINE_LOST_HOLD_S:
                    steer = last_steer                      # 旧三段①: 短丢线不打断动作
                elif steer_mag != 0.0 and lost_s <= budget_s:
                    steer = config.LINE_SEARCH_STEER        # 旧三段②: 方向不明的兜底找线
                else:
                    steer = 0.0                             # 旧三段③: 停（防跑飞）
```

**(d) 日志打印**（可选）：丢线帧补打 state，便于台架确认方向分类是否正确：

```python
                st_note = ("" if data["valid"]
                           else f" 丢线方向={data.get('state', 0)}"
                                " (2=边界远出/3=压黄区/4=不明)")
```

### 3. `config.py` —— 新参数 + 注释更新

**(a) 新增参数**（放在 `LINE_LOST_HOLD_S` 附近，并加进 `EDITABLE_CONFIG` 列表）：

```python
LINE_LOST_CONFIRM_S = 0.4   # 定向找线确认期（秒）：方向明确的丢线先静默这么久再动，
                            # 单帧毛刺（运动模糊/曝光抖动）在这里被吸收，不会误触发找线
```

`EDITABLE_CONFIG` 中追加：

```python
    ("LINE_LOST_CONFIRM_S", "float", "丢线定向找线确认期(秒)"),
```

**(b) 注释更新**（`LINE_LOST_HOLD_S` / `LINE_LOST_SEARCH_S` / `LINE_SEARCH_STEER`）：
说明新语义 —— 定向找线只取 `|LINE_SEARCH_STEER|` 作大小、方向由 state+LINE_ERR_SIGN
推导；符号仅在 state=4 的兜底路径还有效；HOLD+SEARCH 是两路共用的总预算。

**(c) 现行 override 值**（config_override.json：HOLD=5.0, SEARCH=5.0, STEER=-0.5）
新逻辑直接可用：定向找线 0.4s 确认后启动、总预算 10s、速度 0.5。**无需改值**。

## 四、单片机端：无需任何改动

`app_line.c`（2026-09 版）已满足前提：valid=0 时照常执行 Pi 下发的 steer，
帧号新鲜度保护独立工作。唯一要确认的是车上烧的**就是这版**（步末调试串有
`LINE mix:` 自述串即为新版；若是很早的"valid=0 无条件停车"版本，先刷固件）。

## 五、上车验证清单（按序）

1. Pi 端三件套更新、服务重启后，不连相机看日志无报错；
2. 触发巡线（"2"），正常直线段：Pi 日志 `有效=1`、无 bad_lines 猛涨
   （涨 = 解析器没更新成功）；OpenMV 终端打 `valid=1[直线]`；
3. 手推车把边界推出画面上沿：OpenMV 终端 `valid=0[边界远出]`（st=2），
   Pi 日志 `丢线方向=2`，确认期后车**向边界方向**横移；
4. 手推车压到黄区上：`valid=0[压黄区]`（st=3，整帧全黄也是它），车**向路内**横移；
   整帧灰（两面分不开）按 **[边界远出]** 处理（st=2，2026-09-26 实测标定）；
   只有交界还在画面里却没测到时才打 `valid=0[不明]`；
5. 摆到路口大弧：`valid=1[路口弧]`（st=1），offset/angle 平滑（近段重拟合生效）；
6. 全程 bad_lines 不增长、丢线恢复后 err 无积分跳变（PID 状态冻结设计）。

## 六、OpenMV 端可调常量参考（已在 E:\main.py，改这里不用动 Pi）

| 常量 | 值 | 作用 |
|---|---|---|
| `LOST_EMA` | 0.25 | 参考色 EMA 系数（新帧占比） |
| `LOST_CLASS_GATE` | 25.0 | 色度距离投票门槛（两面色心距 ≈65，压黄区帧距离 ≈10） |
| `FAR_ROW` / `NEAR_ROW` | 155 / 175 | 点位置票的行阈值（目标行 164 ±10 的中间带不硬判） |
| `CURVE_RMS_PX` | 5.0 | 曲线判别门槛（直线帧 RMS 1~2px） |
| `NEAR_HALF_ROW` | 120（由窗 (0,240) 推导） | 近段重拟合取"靠车这半边"的分界行 |

—— 以上常量在相机盘 main.py 顶部注释里都有完整理由说明。
