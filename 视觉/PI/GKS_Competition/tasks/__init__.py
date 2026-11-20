# tasks/ — 视觉检测任务包
#
# 仿照 GCS(cv_python/tasks/__init__.py)：任务函数在子模块实现，
# 统一在这里重新导出，供 handler / 主循环 import。

# ⚠️ 扫码有**两台相机 × 三种算法**六条入口：
#    run_qr_auto       = 命令 "1" 新引擎（定位→紧裁→强放大）+ 扫码副摄 ← 2026-10 主路
#    run_qr_auto_main  = 命令 "D" 新引擎 + 主摄
#    run_qr_wechat     = 旧: 微信引擎 + 扫码副摄（保留作对照/兜底）
#    run_qr_wechat_main= 旧: 微信引擎 + 主摄
#    run_qr_diff       = 命令 "E" 自研难度算法 + 扫码副摄
#    run_qr_diff_main  = 命令 "F" 自研难度算法 + 主摄
#    六者返回结构完全相同 —— 单片机侧因此能用同一套结果帧解析。
from tasks._qr import (run_qr_opencv, run_qr_pyzbar, run_qr_wechat,
                       run_qr_wechat_main, run_qr_diff, run_qr_diff_main,
                       run_qr_auto, run_qr_auto_main)
from tasks._line import run_line_follow
from tasks._hostage import run_hostage_detect, build_result_frame
# ⚠️ 色球侧必须叫 build_color_frame，不能也叫 build_result_frame ——
#    同名再经本文件重新导出会覆盖人质的，handler._cmd_hostage 会把色球帧当人质帧发出去。
from tasks._color import run_color_detect, build_color_frame
# ⚠️ 靶标侧的打包函数必须叫 build_target_frame —— 三个 build_* 名字互不相同，
#    否则经本文件重新导出会互相覆盖（色球那条注释就是这个坑的由来）。
from tasks._target import run_target_detect, build_target_frame
# ⚠ 靶标**流式**跟踪（命令 "7"/"8"）是独立模块, 与上面的 run_target_detect（命令 "6"）
#    并存 —— 两者的打包函数名必须互不相同（build_target_frame / build_stream_frame）,
#    否则经本文件重新导出会互相覆盖（上面两条注释就是这个坑的由来）。
from tasks._target_stream import run_target_stream, build_stream_frame
# 色球区**矩形地板**识别（黑胶带边框，2026-09 加）：两条入口 —— 控制端按钮（就地跑）
# 与单片机指令 "A"（结果回 0x07 帧，见 docs/串口协议.md §6h）。
# ⚠️ 打包函数必须叫 build_rect_frame —— 四个 build_* 名字互不相同，
#    否则经本文件重新导出会互相覆盖（上面几条注释就是这个坑的由来）。
from tasks._rect import run_rect_detect, build_rect_frame
# 看路（命令 "G"，2026-09 加）：主摄看地面，测弯道（黄胶带/灰地交界的带圆角直角）的
# **交点** 与 **右切点**，Δx = 两点距离；结果回 0x08 帧（见 docs/串口协议.md §6i）。
# ⚠️ 打包函数必须叫 build_road_frame —— 五个 build_* 名字互不相同，
#    否则经本文件重新导出会互相覆盖（上面几条注释就是这个坑的由来）。
from tasks._road import run_road_detect, build_road_frame
# 看路校准（命令 "H"，2026-10 加）：主摄拟合**黄区直边的图像方向角**（世界固定的线,
# 方向只随车航向变）—— 陀螺漂移的视觉基准。**只测不改**: 结果只进遥测, 不动车不写航向。
# 结果回 0x09 帧（见 docs/串口协议.md §6j）。⚠️ 打包函数必须叫 build_road_yaw_frame ——
# 六个 build_* 名字互不相同, 否则经本文件重新导出会互相覆盖。
from tasks._road_yaw import run_road_yaw_detect, build_road_yaw_frame
# 爆爆桶坐标转发（命令 "J"/"K"，2026-10-24 加）：OpenMV v3.4 检测黑桶 → 本线程
# 纯转发 0x0B 流帧（见 docs/串口协议.md §6m）。⚠️ 打包函数必须叫 build_bucket_frame
# —— 七个 build_* 名字互不相同, 否则经本文件重新导出会互相覆盖。
from tasks._bucket import run_bucket_stream, build_bucket_frame
