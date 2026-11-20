# JiGuangLogs — 标靶练习数据集

GKS_Web「标靶练习」页（`http://<pi>:9000/practice`）写入；分析侧直接从 SMB share 读。

## 校正历史

- **2026-10-05 第一轮**：实地 6 发（同射击位姿，有效性已对账）测得光斑系统性偏移
  (+5.7, −8.6)px，`app_config.h` 的 `TARGET_CENTER_X/Y` 955/502 → **961/493**。
  预期落点残差均值 10.8px≈5.8mm → ~4.6px≈2.4mm。该偏移随车位/朝向变——换射击位姿后需重测。
- **2026-10-05 第二轮（验证批 3 发后）**：残差 (+2.8, −4.0)px（用户目视亦确认偏右上），
  961/493 → **964/489**（累计自原始 955/502 共 +9/−13）。⚠️ 方向约定：err = 光斑 − 检测中心，
  瞄准点**加** err（图像 y 向下为正，"偏上"= y 减）。

- `shots.jsonl` — **每一发一条**（键 = run + 发次 cfg 行时间；重复标注 = 覆盖修正，删记录走
  页面「删」按钮）：闭环数值（outcome/n_ms/dx/dy/trX/trY/ch_mm/r_px，来自 log.txt 的 TGTS 串）
  + 本批条件（cond）+ 打点标注（click/hit/err/err_px + `mark_mode`：live=实时帧直测光斑 /
  frozen=定格帧上人眼映射）+ 检测靶列表（targets）+ 证据帧文件名
  （files）+ **`log` = 该发完整过程日志**（cfg→下一发之间的原始行，含 TGTS 逐拍串/换挡/定格帧行，
  标注时从 run 的 log.txt 固化进来 —— run 目录被清理后这就是追溯那一发全过程的唯一副本）。
- `condition.json` — 当前批条件（位置编号 / 车头角度 / 备注），换位置或角度时在页面改；
  已落进记录的标签标错了不用删重标 —— 记录表里**双击「位置/角度」就地改**那一条。
- `frames/<时间戳>_{dot,dotann,final,finalann,seqNN}.jpg` — 每发的证据帧拷贝：
  `dot`=标注底图（live=点光斑时抓的实时帧、frozen=定格帧副本），`dotann`=其标注版
  （含点击十字），`final`/`finalann`=闭环定格帧（原始/标注），`seqNN`=该发时间窗内的
  过程帧序列（只有当时开着归档开关才有）。全部从 run 目录 `frames/` 拷来。

详细日志与过程帧序列的开关在 Web「参数」页（`TARGET_STREAM_VERBOSE` /
`TARGET_STREAM_ARCHIVE_FRAMES`），改完需重启 main.py；过程帧 `aimseq_*` 留在
`logs/run_*/frames/`（不入本目录，会随日志清理消失——标注时落在该发时间窗内的会被
拷贝成 `seqNN` 固化下来）。
