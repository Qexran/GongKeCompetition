# tasks/_road.py — 「看路」：主摄看地面，量**黄色区左边缘的 x**（2026-09 加，第二版；
#                   2026-10 改版：判据 R−B → **Y−B**、阈值改**自适应**、投票**早停**提速）
#
# 用途：车过弯/到点靠开环里程会**走过头**。地面上的黄色区（比赛是黄胶带，练习地图上是
# 那块米黄篷布）**一般落在画面右上角**，车越往前开它越大、**左边缘越往左跑** ——
# 于是"黄色区左边缘的 x"本身就是一把**测距尺**：Δx 越小 = 车离得越远。
#
# 测量定义（用户口径，别自己改）：
#   Δx = **黄色区左边缘的 x**（全分辨率像素；"原点" = 画面左边 x=0）。
#   单片机按 (理想Δx − 实测Δx) × 比例系数 × 方向(±1) 修一次**车体纵向**。
#   Δy = **黄色区下边缘的 y**（同样全分辨率像素；2026-09-30 加）—— 同一块色块的**另一个轴**，
#        单片机按同一套算式修一次**车体横向**（本车相机相对车体转了 90° ⇒ 图像 y↔车体横向）。
#        ⚠️ 用**下**边缘不用上边缘: 黄区固定在画面右上角, 上边缘永远被画面顶边裁掉
#        （实测现场帧 bbox top 全是 0、flags 全是 0x05，那个 bit2 就是上边缘触发的）,
#        下边缘实测 278~576px 全在窗内且随机位变化。
#
# ==================== 2026-10 改版：为什么判据从 R−B 换成 Y−B ====================
# 换了更浅的新地图后 R−B 判据崩了。实测（现场帧 road_failed/1.jpg、2.jpg，固定曝光
# 166 + 固定 WB 6500K）：**黄区 R−B ≈ +11~+19、灰地 R−B ≈ +2~+8** —— 灰地自己变暖了，
# 余量从旧图的 18~20 级掉到 **5~8 级**；标定阈值 20（旧比赛图口径）直接把整块黄区关在
# 门外，掩膜只剩顶部噪声碎片（area 4092、fl=0x1D）→ 锁错/失败。
#   换 **Y−B = (R+G)/2 − B**（黄度，蓝黄对立色度）后实测同一批帧：
#     黄区 **+12~+17** / 灰地 **+2~+6** / 过曝热点 **−6~−10**（偏蓝，天然排除）
#     / 白斑马线 **+1~+2** / 车身黑件 **大负值** —— 余量 ≈ **10~13 级**，是 R−B 的两倍。
#   为什么 Y−B 对各种黄都稳: 中性黄(R≈G)时 Y−B = R−B（旧图口径不变）；**黄绿**(新地图
#   篷布 G>R)时它取两通道均值 ≈ G−B，信号翻倍；**橙黄**(胶带 R>G)时它 ≈ R−B 不吃亏。
#   纯红/纯绿的干扰物则会被另一半拉低 —— 比单通道差更能代表"黄"。
#
# ==================== 2026-10-28 加：地面灰世界归一 ====================
# 现场（比赛场地）实测：**地面自己的 Y−B = +13~+14**（旧场地 0~+2），黄区只有 +19~+31 ——
# 反差只有 12 级，恰好让谷底法的"鞍点 ≤ 较矮峰 50%"卡在边界上（实测 53%~60%）。同一
# 静止画面（存帧比对：地面/篷布 BGR 中位一致）会逐帧翻面：翻到"没找到黄区峰"支路 →
# "黄区即主峰"把地面主峰(+13)当成黄区 → 阈值 main−6 = 7 → 整个地面过阈 → 掩膜泛滥
# 75%~84% → 泛滥闸把帧全拒，4s 白跑（2026-10-28 05:11 run：8 次看路 4 次 0 命中；
# 05:26 run：5 次里 3 次 0 命中）。
# 处方：`_floor_gray_world` 从 ROI 左下的**地面参考块**估三通道增益（灰世界），把地面
# 拉回 0 附近，再走原来那套谷底自适应/候选打分 —— **阈值选择代码一行不改**。
# 离线回归（18 帧现场 + 10 帧练习/台架，用真机同一份 `_road.py` 回放）：
#   · 现场 18 帧**全部出数**（含原本 0/52、0/43、0/44 的帧）；
#   · 原本成功的帧 Δx 全部在 ±6px 内；练习/台架帧 Δx ±4px、Δy ±2px；
#   · 现场帧阈值从 22 降到 4~6（黄区浅的那批；050552 那批 18~20），掩膜会吃进篷布更
#     暗的下半截 ⇒ **Δy 会 +10~+40px**，到场必须重读 `road_dy_ideal`（Δx 建议顺手复读
#     一遍）—— 与"改阈值先重标 ideal"是同一条纪律。
#   · **峰分离度门限 `ROAD_MIN_PEAK_SEP` 代码缺省 10→8，本场 `config.py` 定 7**
#     （2026-10-30；Web「参数」页可调，改完重启 gks.service 生效）：归一后本场健康帧
#     sep 只有 9~12，10 会把 sep=9 的好帧误判成"没稳"（transient 不耗票 → 整步卡到
#     超时）；病态帧实测 5 级，7 仍拦得住。9~12 是"相对地面"口径，换场地别按旧值
#     23~27 想。
# 关闭开关：config.py 加 `ROAD_WB_FLOOR_EN = 0`（缺省 1）→ 行为与本改动之前一致；
# 参考块位置/护栏阈值同样可用 `ROAD_WB_FLOOR_*` 调（见 `_params()`）。
# ====================================================================

# ==================== 2026-10-30 更新：参考块挪底部 + AWB + CLIP_X 交叉检查 ====================
# 03:32 现场测试（车故意贴近/斜看，黄篷布铺满左上、AWB 开着）：
#   · 原参考块（ROI 左中 40~70%）正好落在黄篷布上（参考块 Y−B 比篷布还高）→ 归一化把篷布
#     当白点洗掉 → 9 帧全"没有黄色区"、或只剩 1372px² 碎片（"黄色区太小"）。
#   · 参考块改到 **ROI 左下 75~95% / x 2~22%**（两种机位都是地面）：两帧都检到篷布
#     （面积 662k/538k，thr 32/39，AWB 18~20% 增益）；28 帧回归（18 现场 + 10 练习）
#     Δx/Δy 与旧参考块逐帧一致（±2px）。
#   · 但「黄区即主峰」档的 main−6 阈值会裁掉篷布偏暗的左端，左缘看起来像真边界（dx=274，
#     实际左缘出画）。新增低 20 级交叉检查：同一个块碰到 ROI 左边且面积 ≥1.2× → 置
#     CLIP_X；run_road_detect 对它**立即以 status=1 放行**给固件的饱和救援
#     （ROAD_RESCUE_MM 后退→重测；不要求 3 票、饱和 dx 不进算式）。⚠️ **不能判失败/
#     不计票** —— 固件只认 status=1 帧的 bit1；status=0 根本进不了救援分支
#     （03:54 实测 9 帧全被排除 → 固件 `ROAD fail: wait st=0`，救援一次没跑）。
#   · 因此看路档**可以跑 AWB**（每帧按地面归一抵消全局色温）；参考块必须在地面、
#     CLIP_X 走固件的饱和救援。固定 6500K 仍是后备档；两种档的 ideal 各自标定。
# ==========================================================================================

# ==================== 2026-10-31 加：反向看路（run_road_detect 的 rev=True） ====================
# 车从路的另一边接近黄区时（返程段机位），画面整体左右镜像：黄区改从画面**左上**进入，
# 随车接近其**右边缘**往右跑。做法不是另写一套"量右边缘"，而是 **整帧水平镜像后走同一套
# 管线**（读帧后 cv2.flip 一次，见 run_road_detect）：
#   · 镜像系里黄区回到"右上角"、量的仍是镜像系**左边缘** —— ROI/自适应阈值/最大连通块/
#     CLIP_X/Δy/投票聚合**全部零改动**，语义逐字继承（"实测Δx 变小 = 车走近"、road_sign、
#     饱和救援方向都跟正向同一套标定值）。分数式 ROI 镜像后窗口自动左右对调 = "ROI 反向"。
#   · **灰世界参考块**：**不镜像**，沿用正向的 ROI 左下 (0.02~0.22 / 0.75~0.95)。
#     ⚠️ 2026-11-02 现场翻车修正（road_0019）: 早先版本把块 x 镜像成 (1-x1, 1-x0) 以为
#     "镜像系右下 = 真实左下地面" —— 实测**错**：车靠近时镜像系右下会漂到黄区边缘
#     （块中位 Y−B +10.5、亮度 167，正常地面 −20），又没到 `wb_floor_max_yb`(40) 护栏
#     → 归一化把黄区当白点洗掉 → 阈值塌到地板 4 → 掩膜只剩零星碎片 → 锁到 dx=1408 的
#     碎片（正常 40 万 px² → 5896），随后的跨帧散布闸判失败。整帧镜像后画面**本就回到
#     正向布局**（黄区右上、左下地面），正向标好/钉死的左下块直接用最稳（同一批失败帧里
#     左侧块四帧全是干净地面 −8.5~−32，右侧块一帧漂到 +10.5）。
#   · Δx/Δy/edge 全部是**镜像系坐标**（真实右边缘读数与它差一个 1920 翻转关系）——
#     单片机侧 road_dx_ideal / road_dy_ideal 要按反向机位**另标**（路线里排
#     STEP_ROAD_SET_IDEAL 覆盖，或蓝牙现改）；比例/死区/方向符号与正向同套。
#   · 归档帧（frames/）与 Web 预览也是镜像图（角标 REV），与检测数字自洽；要看真实取景
#     把图再水平翻回来即可。
#   · 命令通道：串口 "M"（handler._cmd_road_rev，对齐固件 RPI_TASK_ROAD_REV = 30）。
#     部署顺序先 Pi 后固件；老固件根本不会发 "M"，老 Pi 收到 "M" 响亮拒收（未知指令）。
# ==========================================================================================

# ==================== 2026-10 改版：阈值为什么改自适应 ====================
# 固定阈值的死穴是"换图/换光就得重标，标在分布中段就碎成小岛"（config.py 里 2026-09-30
# 的头号坑）。现在阈值每帧现算（`_adaptive_threshold`，直方图 1/16 采样，全耗 <2ms）：
#   ① **谷底**：D 直方图主峰（灰地）与右边第一个过**显著性检验**的峰（黄区：峰高 ≥ 主峰
#      5% 且两峰间鞍点沉到较矮峰的 50% 以下）之间取谷底 —— 阈值自动落在两分布的空隙里，
#      地图换色/光照漂移都跟着走。显著性检验滤掉地面褶皱/光照梯度造出的"分布肩部假峰"
#      （2026-10-02 旧 log 回归实测：没这检验时阈值会被骗到地面分布内部 → 褶皱泄漏 → 假数）。
#   ② **黄区即主峰**（车很近，黄区占画面大头）：阈值 = 主峰 − 6。
#   ③ **黄区太小构不成峰**（车远）：阈值 = clamp(主峰+4, p99.5−3)，地板钳到
#      ROAD_WARM_FLOOR_YB —— p99.5 锚的是"最黄的那一小撮"。
#   三段都过不了（没有比主峰黄 ≥5 级的像样亮块）→ 直接判"画面里没有黄色区"，
#   **不出巨块**（旧版阈值失手时会整片灰地过阈 → Δx 贴死 ROI 左缘 → 单片机拿假数修车）。
#   ROAD_WARM_FLOOR_YB 是最后一道保险（"黄区再浅也必须比白的多几点黄"），正常别动。
#
# ==================== 2026-10-09 加：黄度对比度闸门 ====================
# Y−B 的盲区是**绿色/青色**：G≫B 同样把 (R+G)/2−B 拉高，灰绿墙面/台架的**阴影与亮斑**
# 在直方图上就是一个像模像样的"第二峰"，谷底法照常给出阈值 → 锁到阴影块 → **假成功**
# （2026-10-08 07:10:32 实锤：相机对着墙/台架，阴影块 Y−B+18 vs 墙面+6，报了 dx=1185.6
#  的假成功，单片机拿它修了 5.78mm）。色度上阴影与米黄篷布**分不开**（篷布在该固定
# WB 下自己也偏绿：R−G≈−6），但"和背景的**反差**"分得开：真篷布比灰地黄 **+30**（三帧
# 一致），阴影只比墙面"黄" +7~+12。所以选完最大块后再量一次块内/块外的 Y−B 中位差
# （_warm_contrast，<1ms），不够反差就整帧判"没有黄色区"—— 宁可失败也不出假数。
#
# ==================== 2026-10 改版：速度 ====================
#   ① 投票**早停**：凑满 min_votes 且已见 Δx 极差 ≤ 离群下限(5px) → 提前收工。
#      静止机位下后面的帧只会重复同一答案 —— 实测 9 帧全采极差 0~2px，通常 **3 帧即回**
#      （单片机侧从 ~2~3s 降到 ~1~2s， chiefly 省在少读 6 帧上）。早停只是**不再读**，
#      聚合口径与采满完全相同（全部 hits 过同一套双峰/MAD 判据），所以不会出现
#      "早停的成功换成采满就失败"；快照帧数不同带来的差 ≤ 几 px（远小于 7px 死区）。
#      ⚠️ 2026-10-30 改：判据从「**全部**已采帧的极差」改成「**最近 min_votes 帧**
#      （滑动窗）的极差」，并要求多攒一票（len(hits) > min_votes）。现场 05:45 那次
#      5/12 步看路采满 9 帧，其中 4 步的第 1 帧偏 6~8px、后两帧完全一致 ——
#      极差 6~8 只差 1~3px 卡在 5px 门外，多读 6 帧白花 ~0.6s/步；采满后从第 4 帧起
#      逐帧完全一致。滑动窗让"开头一帧没坐稳"不再拖住整步，而"多攒一票"保证不是
#      三帧碰巧同值；真正的双簇/车在动在窗里仍表现为极差大，照旧采满。
#      ⚠️ 另加一道「**全程极差 ≤ max_spread_px** 才许早停」：车没停稳时可能是"前几帧
#      漂、最近几帧碰巧稳"（06:36:20 road_0007：最近 3 帧 2px、全程 28px），只看滑动窗
#      会提前收工再被最终散布闸判失败；加上这条就退回采满，由最终判据用全部帧裁决。
#   ② V 通道用 max(B,G,R) 直接算（HSV 的 V 本来就是它），免掉整幅 cvtColor。
#   ③ 阈值直方图只吃 1/16 像素。降采样(ROAD_PROC_SCALE)照旧。
#
# ⚠️⚠️ 判据/阈值口径变了 ⇒ 掩膜边缘跟着挪 ⇒ **Δx 有 10~30px 级平移** —— 到场第一步重标
#    单片机的 ROAD_DX_IDEAL_PX / ROAD_DY_IDEAL_PX（把车摆到理想停车位，读蓝牙 Tab 的
#    road_dx/road_dy 填进去），旧标定作废！
# ⚠️ 已知局限：补光热点若正好压住黄区**顶部**的左边界，那一段掩膜会被挖穿、Δx 偏大
#    （实测机位热点在画面中部偏下、黄区左边界走画面右上，不挡；Web /road 预览一眼可查）。
#
# ⚠️ 只读一帧时车必须静止（多帧投票要求同一机位，与色球/矩形一致）。
# ⚠️ 坐标一律回**全分辨率 1920×1080**（与色球/人质/矩形帧同口径，单片机按它标定）。
# ⚠️ 任务层只返回结果 dict，串口回传/页面显示由调用方负责（同 _rect.py，保持纯净可测）。
#
# 历史（别把第一版捡回来）：第一版做的是"拟合弯道两条直边→求交→圆弧相切→取右切点"
#   （`看路.jpg` 上那两条红线就是那么画的）。它在**合成本例上完全正确**（Δ 几乎等于圆角
#   半径 R），但在真实机位上过不去：练习地图那条边界是篷布褶皱/软阴影边，Hough 会把圆弧的
#   弦当直线、折线化又因边界碎片化只给出短边 → 永远配不成"两条腿"。那一版留在
#   `tasks/_road.py.bak-curve`，完整复盘与证据见 `Desktop/road_offline/README.md`。
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

ROAD_TAG = 0x08   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）

# 帧内 flags 位（与固件 Application/app_rpi.h 的 RPI_ROAD_FLAG_* 逐位对齐）
FLAG_EDGE_OK = 0x01   # bit0: 左边缘有效（**Δx 全靠它**）
FLAG_CLIP_X = 0x02    # bit1: 黄色区触到画面左/右边 → Δx 可能饱和，不可信
FLAG_CLIP_Y = 0x04    # bit2: 黄色区触到画面上/下边（只提示，不影响用）
FLAG_SMALL = 0x08     # bit3: 面积刚够门限（勉强），建议核对
FLAG_Y_EDGE_OK = 0x10  # bit4: **Δy 有效**（黄区**下边缘**没被裁掉）—— 单片机的横向修正靠它

# D 直方图（Y−B 的值域足够宽：车身黑件 −32、热点 −10、灰地 −11~+8、黄区 +7~+23）
_HIST_LO, _HIST_HI, _HIST_BINS = -60, 80, 141
_MIN_SEP_BINS = 4     # 主峰与黄区峰的最小间隔[bin=1级]
_PROMINENCE = 0.5     # 谷底显著性：鞍点高度须 ≤ 两边较矮峰高的 50%（滤地面分布肩部假峰）
_PEAK_CANDIDATES = 4  # 主峰右侧参与打分的候选峰上限（补光灯的光斑峰 + 真黄区峰 + 余量）

_I16_MIN, _I16_MAX = -32768, 32767
_U16_MAX = 65535


def _clamp_i16(v):
    return max(_I16_MIN, min(_I16_MAX, int(round(v))))


def _clamp_u16(v):
    return max(0, min(_U16_MAX, int(round(v))))


def build_road_frame(ok, dx=0.0, ex=0.0, ey=0.0, flags=0, spread=0.0, dy=0.0):
    """打包回传帧 DATA（见 docs/串口协议.md §6i）：

    [TAG=0x08][status u8][dx i16 LE /10][ex i16 LE][ey i16 LE][flags u8][spread u8][dy u16 LE /10]
    = 12 B

      status : 1 = 成功（dx 有效）；0 = 失败（没找到黄区，全部填 0）
      dx     : **Δx = 黄色区左边缘的 x**，单位 **0.1px**（i16 → ±3276.7px，够用）→ 车体**纵向**
      ex/ey  : 左边缘那一点的全分辨率像素坐标（遥测/画图用，单片机不用）
      flags  : 见文件顶部 FLAG_*
      spread : Δx 跨帧散布[px]（封顶 255；只做遥测，单片机不据此判失败）
      dy     : **Δy = 黄色区下边缘的 y**，单位 **0.1px**（u16 → 0~6553.5px）→ 车体**横向**
               （本车相机相对车体转了 90° ⇒ 图像 y 那条轴就是车体横向）。**原先这里是
               "保留 u16" 恒 0** —— ⚠️ 填了它就必须同时置 FLAG_Y_EDGE_OK，否则单片机
               会把 0 读成"没有这个量"（那是它区分新旧 Pi 的唯一依据）。

    ⚠️ **失败帧也要发**（status=0）—— 单片机靠它立刻解锁，不然要空等满整个超时。
    ⚠️ 长度 12（**不是 9**）：固件里"9 字节 = last packet"那条通用分支会把这帧整个吞掉。
       **加了 dy 也刻意不改长度**（占原来那个保留字段）：改成 14 的话，按 `>= 14` 判的
       新固件会把旧 Pi 发来的 12B 帧当 last packet 吞掉（静默失效）。
    ⚠️ dx 用 **i16**：方向对调后误差可以为负，且黄区贴到画面左边时 x 可以到 0 附近 ——
       用 u16 会被钳成 0 而且**不报错**。dy 相反：下边缘的 y 不可能为负 → **u16**。
    """
    return struct.pack("<BBhhhBBH", ROAD_TAG, 1 if ok else 0,
                       _clamp_i16(dx * 10.0), _clamp_i16(ex), _clamp_i16(ey),
                       int(flags) & 0xFF,
                       max(0, min(255, int(round(spread)))), _clamp_u16(dy * 10.0))

# 标注用色（BGR）
_MASK_BGR = (0, 200, 0)
_EDGE_BGR = (0, 255, 255)
_DY_BGR = (255, 128, 255)      # 黄区下边缘那条横线（横向那把尺）


def _params():
    """参数每次调用现取（Web 参数页改完即时生效，同 _rect.py 的做法）。"""
    from config import (ROAD_PROC_SCALE, ROAD_ROI_TOP, ROAD_ROI_BOTTOM,
                        ROAD_ROI_LEFT, ROAD_ROI_RIGHT, ROAD_WARM_FLOOR_YB,
                        ROAD_PEAK_FRAC, ROAD_WARM_V_MIN, ROAD_WARM_OPEN_K,
                        ROAD_WARM_CLOSE_K, ROAD_WARM_MIN_AREA,
                        ROAD_WARM_CONTRAST_MIN, ROAD_WARM_MAX_AREA_FRAC,
                        ROAD_VOTE_FRAMES, ROAD_MIN_VOTES, ROAD_MAX_SPREAD_PX,
                        ROAD_OUTLIER_K, ROAD_OUTLIER_MIN_PX, ROAD_DX_MIN_PX,
                        ROAD_DX_MAX_PX)
    import config
    return dict(proc_scale=ROAD_PROC_SCALE,
                roi_top=ROAD_ROI_TOP, roi_bottom=ROAD_ROI_BOTTOM,
                roi_left=ROAD_ROI_LEFT, roi_right=ROAD_ROI_RIGHT,
                floor=ROAD_WARM_FLOOR_YB, peak_frac=ROAD_PEAK_FRAC,
                v_min=ROAD_WARM_V_MIN, open_k=ROAD_WARM_OPEN_K,
                close_k=ROAD_WARM_CLOSE_K, min_area=ROAD_WARM_MIN_AREA,
                contrast_min=ROAD_WARM_CONTRAST_MIN,
                max_area_frac=ROAD_WARM_MAX_AREA_FRAC,
                vote_frames=ROAD_VOTE_FRAMES, min_votes=ROAD_MIN_VOTES,
                max_spread_px=ROAD_MAX_SPREAD_PX,
                outlier_k=ROAD_OUTLIER_K, outlier_min_px=ROAD_OUTLIER_MIN_PX,
                dx_min=ROAD_DX_MIN_PX, dx_max=ROAD_DX_MAX_PX,
                # 2026-10-23 加的两道质量闸（config 里暂未登记 → getattr 缺省,
                # 现场要用 Web 参数页调时再往 config.py 补这两行）
                sat_max_frac=float(getattr(config, "ROAD_SAT_MAX_FRAC", 0.10)),
                # 2026-10-28：代码缺省 10→8；2026-10-30 本场 config.py 定 7（Web 可调）。
                # 本场地面偏暖 + 黄区偏浅，WB 归一后健康帧的 sep 只有 9~12（原色度空间
                # 是 12）；10 会把 sep=9 的好帧误判成"没稳"、且 transient 不耗票 → 整步
                # 卡到超时。病态帧实测 5 级，7 仍能拦住。
                min_peak_sep=int(getattr(config, "ROAD_MIN_PEAK_SEP", 8)),
                # 2026-10-28 加：地面灰世界归一（治"整幅略黄"场地, 见 _floor_gray_world）。
                # config 里暂未登记 → 全部 getattr 缺省; 现场要关它/挪参考块再往
                # config.py 补同名宏（不用重启服务端, 但改 config 后照旧要重启 gks.service）。
                wb_floor_en=bool(int(getattr(config, "ROAD_WB_FLOOR_EN", 1))),
                wb_floor_x0=float(getattr(config, "ROAD_WB_FLOOR_X0", 0.02)),
                wb_floor_x1=float(getattr(config, "ROAD_WB_FLOOR_X1", 0.22)),
                # 2026-10-30 改：参考块从 ROI 左中(40~70%)挪到**左下(75~95%)**。
                # 原位置在"车贴得很近/斜着看"时正好落在黄篷布上（ROI_BOTTOM 一改更明显），
                # 归一化把篷布当白点洗掉 → 整帧"没有黄色区"（03:32 现场两帧实测）。
                # 左下条带在两种机位都是地面（轮子在画面中/右下方，x 2~22% 碰不到）。
                wb_floor_y0=float(getattr(config, "ROAD_WB_FLOOR_Y0", 0.75)),
                wb_floor_y1=float(getattr(config, "ROAD_WB_FLOOR_Y1", 0.95)),
                wb_floor_vmin=float(getattr(config, "ROAD_WB_FLOOR_VMIN", 40.0)),
                wb_floor_vmax=float(getattr(config, "ROAD_WB_FLOOR_VMAX", 250.0)),
                # AWB 下地面可能明显偏紫（本场紫地面 18~20% 增益）—— 25% 会误跳过归一,
                # 0.35 仍能挡住"参考块选到彩色物体"的离谱增益。
                wb_floor_max_gain=float(getattr(config, "ROAD_WB_FLOOR_MAX_GAIN", 0.35)),
                wb_floor_max_yb=float(getattr(config, "ROAD_WB_FLOOR_MAX_YB", 40.0)))


# ==================== 单帧检测 ====================
def _roi_rect(shape, p):
    """分数式 ROI → 像素 (x0, y0, x1, y1)。**车身/机械臂全在画面下半**，靠它切掉。"""
    h, w = shape[:2]
    x0 = max(0, min(w - 1, int(round(p["roi_left"] * w))))
    y0 = max(0, min(h - 1, int(round(p["roi_top"] * h))))
    x1 = max(x0 + 1, min(w, int(round(p["roi_right"] * w))))
    y1 = max(y0 + 1, min(h, int(round(p["roi_bottom"] * h))))
    return x0, y0, x1, y1


def _adaptive_threshold(D, valid, p):
    """在 D（Y−B）直方图上**每帧现算**阈值。返回 (thr, note, sep) 或 (None, 失败原因, None)。

    三段式（详见文件顶部"阈值为什么改自适应"）：
      ① 谷底  ② 黄区即主峰 → 主峰−6  ③ 极小黄区 → p99.5 锚定。都过不了 → None。
    ⚠️ 直方图只吃 1/16 像素（D[::4,::4]）：分布足够稳，耗时 <2ms。
    sep = ①里被选中的黄区峰与主峰的间隔[级]（②③两段给 None —— 它们没有"第二峰"可量）。

    ⚠️⚠️ 2026-10-23 **候选峰打分**（补光灯引入的第三个分布，本案实测）：
    原来的 ① 只取"主峰右侧**第一个**显著峰"，这个前提在**没有补光灯**时成立
    （画面里非灰地的大块暖色只有黄区）。加了 6500K 白光补光灯后，灯的**地面光斑**
    在 Y−B 上是个独立小峰（实测 +12~13），夹在灰地(+5~8)与真黄区(+37)之间 ——
    于是"第一个显著峰"落在光斑上: 阈值 13 → 掩膜=整片被照亮的灰地 → 真黄区根本
    没被看到（run_20261023_021305 第 3 次看路: `谷底 主峰+8/黄区+13`、sep=5、
    42 帧全废、白白失败一步；而同一帧真黄区 Y−B 中位 +37、饱和仅 0.2%，
    数据本身完全可用）。
    所以现在**收集主峰右侧所有过显著性检验的峰**（最多 _PEAK_CANDIDATES 个），
    每个都按它前面的谷底算一个候选阈值，再用**同一把尺子**打分:
        score = median(D[阈内]) − median(D[阈外])
    即"这块比背景黄多少"（与最终 _warm_contrast 同口径）—— 光斑候选 ≈ +7、
    真黄区候选 ≈ +27，取分高者。**不是**取最高峰: 褶皱/噪声在黄区右侧还能造峰
    （见下文的 2026-09-30 教训），纯按位置或高度选都会踩坑，只有"块内块外反差"
    是黄区独有的性质。
    ⚠️ 肩部假峰那道纪律仍然有效: 候选必须先过 saddle 显著性检验, 才进入打分。
    """
    d = D[::4, ::4][valid[::4, ::4]]
    if d.size < 2000:                     # ROI 里几乎没有有效像素（对错墙了？）
        return None, "有效像素太少", None
    hist, _ = np.histogram(d, bins=_HIST_BINS, range=(_HIST_LO, _HIST_HI))
    k = np.array([1, 2, 3, 2, 1], np.float32)
    hs = np.convolve(hist.astype(np.float32), k / k.sum(), "same")
    hs[-1] = hs[-2]                       # 卷积右边缘塌掉，用前一 bin 补
    main = int(np.argmax(hs))
    # ① 向右收集候选峰：局部极大 + 峰高 ≥ peak_frac×主峰 + **鞍点显著性检验**。
    #    ⚠️ 没有显著性检验会栽在"地面分布的肩部涟漪"上：地面褶皱/光照梯度让灰地直方图
    #    拖着带小鼓包的长尾，第一个鼓包就能把谷底骗进**地面分布内部**（阈值过低 → 褶皱
    #    泄漏 → 掩膜贴到 ROI 左缘 → Δx=192 假数；2026-10-02 旧 log 回归实测踩过，
    #    那批帧黄区 Y−B 其实 +28~+34、地面 +4~5，分离度极好，纯纯是扫描被肩部截停）。
    #    ⚠️ 也别直接用全局 argmax 找黄区：褶皱会在黄区内部造出子峰，argmax 可能跳过
    #    黄区抓到更远的噪声峰，同样把谷底卡进黄区分布中段（碎岛坑，2026-09-30 实测）。
    cands = []                            # [(thr, sep, peak_d)]
    for b in range(main + _MIN_SEP_BINS, _HIST_BINS - 1):
        if len(cands) >= _PEAK_CANDIDATES:
            break
        if hs[b] >= p["peak_frac"] * hs[main] and hs[b] >= hs[b - 1] and hs[b] >= hs[b + 1]:
            saddle = float(hs[main:b + 1].min())
            if saddle <= _PROMINENCE * min(hs[main], hs[b]):
                valley = main + int(np.argmin(hs[main:b + 1]))
                thr = float(max(valley, main + 2) + _HIST_LO)   # 谷底贴着主峰时至少抬 2 级
                cands.append((max(thr, float(p["floor"])), int(b - main), int(b + _HIST_LO)))
    if cands:
        # 打分（见 docstring）：块内中位 − 块外中位。用 1/4 采样控耗时（~2ms/候选）。
        Ds = D[::4, ::4]
        ok = valid[::4, ::4]
        best = None
        for thr, sep, peak_d in cands:
            m = (Ds > thr) & ok
            n_in = int(m.sum())
            if n_in < 200 or n_in >= ok.sum():
                continue
            score = float(np.median(Ds[m]) - np.median(Ds[ok & ~m]))
            if best is None or score > best[0]:
                best = (score, thr, sep, peak_d)
        if best is not None:
            score, thr, sep, peak_d = best
            tag = "" if len(cands) == 1 else f"/候选{len(cands)}选1"
            return (thr,
                    f"谷底 主峰{main + _HIST_LO:+d}/黄区{peak_d:+d}"
                    f"(反差{score:+.0f}{tag})",
                    sep)
    main_d = main + _HIST_LO
    if main_d >= p["floor"] + 4:          # ② 主峰本身就是黄区（车很近，黄区占大头）
        return float(max(main_d - 6, p["floor"])), f"黄区即主峰({main_d:+d})", None
    p995 = float(np.percentile(np.clip(d, _HIST_LO, _HIST_HI), 99.5))
    # ⚠️ 分位数必须钳到直方图值域：图上若有饱和黄污染（烘焙标注/车上黄线标过曝,
    #    Y−B 可到 200+），不钳的话 p99.5 会飞到分布外、锚定完全失效（实测 p99.5=208）。
    if p995 - main_d >= 5:                # ③ 黄区太小构不成峰 → 高分位锚定
        thr = max(p["floor"], min(main_d + 4, p995 - 3))
        return float(thr), f"小黄区 主峰{main_d:+d}/p99.5={p995:.0f}", None
    return None, "画面里没有比地面黄的像样亮块（没有黄色区）", None


def _floor_gray_world(b, g, r, p):
    """地面灰世界归一（2026-10-28 加）：从 ROI 左下"必定是地面"的参考块估三通道增益。

    **为什么需要**：场地灯偏暖时**地面自己的 Y−B 就能到 +13**（旧场地 0~+2），黄区只比
    地面黄 10~13 级 —— 谷底法的"鞍点 ≤ 较矮峰 50%"正好卡在边界上（本场实测 53%~60%）。
    同一静止画面会逐帧翻面；翻到"没找到黄区峰"支路时，"黄区即主峰"把**地面**主峰(+13)
    当成黄区 → 阈值 main−6 = 7 → 整个地面过阈 → 掩膜泛滥 75%~84% → 泛滥闸全拒。
    把参考块三通道各自除以自己的中位数（乘 max/med）后，地面回到 0 附近、黄区相对
    色度不变，原来的谷底自适应/候选打分一行不用改。

    ⚠️ 参考块固定在 **ROI 左下**（x 2~22%、y 75~95%）：黄区固定在右上、轮子/机械臂在
       中/右下 —— 这条左下条带在地面。**不能再挪回 40~70%**（2026-10-30 实测）：车贴得
       很近/斜着看（或 ROI_BOTTOM 调小）时 40~70% 正好落在黄篷布上，归一化把篷布当
       白点洗掉 → 整帧"没有黄色区"。左下条带在两种机位都是地面。
    ⚠️ 这条归一化也是**跑 AWB 的前提**：它按地面参考每帧抵消全局色温（AWB 漂移对
       Y−B 的平移被减掉），所以看路档不必强锁 6500K；但参考块必须落在地面。
    ⚠️ 护栏：块太暗/过曝、块自己已经明显发黄、或所需增益偏离 > max_gain —— 都说明
       视野被挡 / 车贴到篷布跟前 / 参考块选错了。此时**不归一**（退回原始通道），
       别把黄区当白点洗掉。
    ⚠️ 反向看路（rev）**不改参考块**：整帧镜像后画面回到正向布局（黄区右上、左下地面），
       正向的 ROI 左下块直接适用。2026-11-02 现场验证：镜像版本（块挪右下）在车靠近时会
       漂到黄区边缘（Y−B +10.5、亮度 167）, 归一化把黄区洗掉 → 阈值塌到地板、掩膜碎成
       碎片、锁到假 dx（road_0019 复盘见文件头）。
    返回 (kb, kg, kr, dev) 或 None（None = 不归一）。
    """
    h, w = b.shape
    xs = slice(max(0, int(p["wb_floor_x0"] * w)),
               min(w, int(p["wb_floor_x1"] * w)))
    ys = slice(max(0, int(p["wb_floor_y0"] * h)),
               min(h, int(p["wb_floor_y1"] * h)))
    if xs.stop - xs.start < 8 or ys.stop - ys.start < 8:
        return None
    # 隔点取样（1/4 像素）就够稳，耗时 <1ms
    pb = b[ys, xs][::2, ::2].ravel()
    pg = g[ys, xs][::2, ::2].ravel()
    pr = r[ys, xs][::2, ::2].ravel()
    if pb.size < 500:
        return None
    mb = float(np.median(pb))
    mg = float(np.median(pg))
    mr = float(np.median(pr))
    v = max(mb, mg, mr)
    if v < p["wb_floor_vmin"] or v > p["wb_floor_vmax"]:
        return None
    if 0.5 * (mg + mr) - mb >= p["wb_floor_max_yb"]:
        return None                      # 参考块自己就很黄 → 多半是篷布/胶带, 不是地面
    kb = v / max(mb, 1.0)
    kg = v / max(mg, 1.0)
    kr = v / max(mr, 1.0)
    dev = max(abs(kb - 1.0), abs(kg - 1.0), abs(kr - 1.0))
    if dev > p["wb_floor_max_gain"]:
        return None                      # 偏得太多 —— 不是光照色偏, 是看错东西了
    if dev < 0.005:                      # 已经中性, 省掉三次浮点乘
        return None
    return kb, kg, kr, dev


def _warm_mask(sub, p):
    """黄色区掩膜：**Y−B = (R+G)/2 − B > 自适应阈值**（黄度色差 + 直方图谷底）。

    ⚠️ 别改回单帧固定阈值、也别改回 R−B：新浅色地图上灰地自己 R−B≈+6，R−B 余量只剩
       5 级（实测 road_failed/1.jpg），固定阈值标在哪儿都两头漏。Y−B + 谷底自适应
       对"地图换色/光照漂移"免疫，阈值永远落在灰地与黄区分布的空隙里。
    ⚠️ 2026-10-28 加地面灰世界归一（`_floor_gray_world`）：整幅偏黄时地面 +13 会把
       谷底判据逼到临界线 —— 归一后 D 的口径变成"相对地面"，阈值会明显变小（4~7）。
    ⚠️ V 用 max(B,G,R) 直接算（HSV 的 V 就是它），省一整幅 cvtColor。
    返回 (mask, D, valid, thr, note, sep)；算不出阈值时 mask 为 None（D/valid 照给，调用方
    拿不到掩膜时不会用到它们）。D/valid 供黄度对比度闸门（_warm_contrast）复用，
    免得调用方再算一遍三个 int16 通道。sep 见 _adaptive_threshold（闸门用）。
    """
    b = sub[:, :, 0].astype(np.int16)
    g = sub[:, :, 1].astype(np.int16)
    r = sub[:, :, 2].astype(np.int16)
    wb = _floor_gray_world(b, g, r, p) if p.get("wb_floor_en") else None
    if wb is None:
        D = (r + g) // 2 - b
    else:
        kb, kg, kr = wb[0], wb[1], wb[2]
        D = ((g.astype(np.float32) * kg + r.astype(np.float32) * kr) * 0.5
             - b.astype(np.float32) * kb).astype(np.int16)
    valid = sub.max(axis=2) >= int(p["v_min"])     # 挡全黑噪声（车身已被 Y−B 排除，双保险）
    thr, note, sep = _adaptive_threshold(D, valid, p)
    if thr is None:
        if wb is not None:
            note = "%s [WB%.0f%%]" % (note, 100.0 * wb[3])
        return None, D, valid, thr, note, sep
    if wb is not None:
        note = "%s WB%.0f%%" % (note, 100.0 * wb[3])
    m = ((D > thr) & valid).astype(np.uint8) * 255
    ko = int(p["open_k"]) | 1
    kc = int(p["close_k"]) | 1
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc)))
    return m, D, valid, thr, note, sep


def _warm_contrast(D, valid, lab, i):
    """**黄度对比度闸门**（2026-10-09 加）：锁定块内 Y−B 中位数 − 块外(背景)中位数。

    为什么非它不可：Y−B 对**绿色/青色同样给高值**（G≫B 拉高均值），灰绿墙面/台架的
    阴影与亮斑能骗过谷底自适应阈值造出假"黄区峰"，报出假 Δx 修错车（2026-10-08
    07:10 实锤：阴影块 +18 vs 墙面 +6 报了 dx=1185 的假成功）。真黄区是**比背景黄一
    大截的实体**（实测米黄篷布 +31 vs 灰地 +1，对比度恒 +30），阴影只是同一面墙上
    暗一点的地方（+7~+12）—— 中位差把这两类分得干干净净。

    返回 (contrast, n_out)：contrast = 块内中位 − 块外中位（背景采样 1/16 提速）；
    背景点太少（黄区占满窗，判不了）时返回 (None, n_out)，调用方应放行（车很近的
    合法场景，别误杀）。
    """
    sel = lab == i
    outside = D[::4, ::4][valid[::4, ::4] & ~sel[::4, ::4]]
    if outside.size < 500:
        return None, int(outside.size)
    inside = D[sel]
    if inside.size == 0:
        return None, int(outside.size)
    return float(np.median(inside) - np.median(outside)), int(outside.size)


def _detect_one(frame, p):
    """单帧：Y−B 自适应抠黄色区 → 取**最大连通块** → 最左那一列 → Δx; bbox 下边 → Δy。坐标回全分辨率。"""
    fail = dict(success=False, reason="", dx=0.0, dy=0.0, edge=None, flags=0, mask=None,
                roi=None, scale=1.0, area=0.0, thr=None, note="", contrast=None)
    x0, y0, x1, y1 = _roi_rect(frame.shape, p)
    sub = frame[y0:y1, x0:x1]
    sc = float(p["proc_scale"])
    if abs(sc - 1.0) > 1e-6:
        sub = cv2.resize(sub, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
    hh, ww = sub.shape[:2]

    # ---- 过饱和闸门（2026-10-23 加）：大面积像素贴 255 → 曝光没稳/直射反光, 帧不可信 ----
    #    治的是补光灯(6500K) + 曝光自动→手动交接期的过曝帧: 整幅亮到发黄, Y−B 分布塌掉,
    #    自适应阈值仍能"成功"抠出半幅假黄区（01:27 run 实测: 分离度 5 级 + 面积 49% 刚好
    #    漏过泛滥闸 → Δx=430 假成功, 单片机白修 84mm）。健康帧饱和像素 <1%（现场实测）。
    #    ⚠️ transient=True → 投票循环豁免票预算多等几帧, 不把 9 帧预算烧在过渡期上。
    sat_frac = float((sub.max(axis=2) >= 250).mean())
    if sat_frac > float(p["sat_max_frac"]):
        return dict(fail, transient=True,
                    reason="过饱和像素占 ROI %.0f%% > %.0f%%（曝光没稳/补光直射反光），帧不可信"
                           % (100.0 * sat_frac, 100.0 * p["sat_max_frac"]),
                    roi=(x0, y0, x1, y1), scale=sc)

    mask, D, valid, thr, note, sep = _warm_mask(sub, p)
    if mask is None:
        return dict(fail, reason=note, roi=(x0, y0, x1, y1), scale=sc, thr=None, note=note)
    # ---- 峰分离度闸门（2026-10-23 加）：黄区峰离主峰太近 = Y−B 分布没拉开 = 没稳 ----
    #    健康帧 23~27 级（2026-10-23 三次成功看路全部如此）; 换档交接期病态帧 5 级。
    #    ⚠️ 2026-10-28：本场（地面偏暖 + 黄区浅）WB 归一后健康帧只有 9~12 级 ——
    #    缺省门限已从 10 降到 8（见 _params）；病态帧 5 级仍拦得住。
    #    只在"谷底"档（sep 非 None）可判 —— "黄区即主峰/小黄区"两档没有第二峰, 不适用。
    if sep is not None and sep < p["min_peak_sep"]:
        return dict(fail, transient=True,
                    reason="黄区峰与地面主峰只差 %d 级 (<%d) —— Y−B 分布没拉开, "
                           "多半曝光/白平衡没稳（补光灯刚开/换档交接期），不给数"
                           % (sep, p["min_peak_sep"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)
    n, lab, st, _ce = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return dict(fail, reason="画面里没有黄色区（自适应阈值下无亮块）", mask=mask,
                    roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)
    # ⚠️ 取**最大连通块**：车上的黄色线标/另一条胶带是小的，取最大就天然躲开它们
    #    （多帧投票里还有双峰判据兜底：真锁错时会分两簇、直接不给数）
    # ⚠️ 2026-09-30 试过"按帧间共识重挑块"来治"掩膜碎成小岛 → 最大块逐帧换人"，**撤了**:
    #    掩膜碎了就没有正确答案 —— 多数小岛占多数时共识会稳定地锁在**错块**上（实测真报过
    #    1515 而真相是 870），而势均力敌时它会把"分成两簇 → 明确不给数"悄悄变成"多数决"。
    #    2026-10 起谷底自适应阈值把"阈值卡在分布中段 → 碎岛"这个根源掐掉了，这条纪律继续。
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    a_full = float(st[i, cv2.CC_STAT_AREA]) / (sc * sc)
    if a_full < p["min_area"]:
        return dict(fail, reason="黄色区太小（%.0f < %.0f px²）"
                                 % (a_full, p["min_area"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)
    # ---- 掩膜泛滥闸门（2026-10-09 午加）：黄区面积超 ROI 的一半以上 → 失败 ----
    #    治的是"白平衡/曝光没锁住 → 整幅染黄 → 全图成掩膜 → Δx 贴死 ROI 左缘(192)"
    #    （12:15:29 实锤：面积 140 万 = ROI 的 78%，Δx=192 恒等于 ROI 左边界）。
    #    正常帧篷布只占 ROI 6%~31%。"车贴到篷布跟前"确实会超，但那时左边缘已出画，
    #    Δx 量的是窗框不是篷布，失败才是对的。详见 config.ROAD_WARM_MAX_AREA_FRAC。
    #    ⚠️ 2026-10-23 加 transient=True：泛滥多半是**几秒级瞬态**（曝光模式交接 /
    #       机构从视野里扫过 —— 01:06 run 实测第 1 帧好、第 2~8 帧泛滥、第 9 帧又好），
    #       投票循环据此豁免它的票预算再多等一会儿，而不是把 9 帧预算烧在坏帧上直接失败。
    roi_area_full = float((x1 - x0) * (y1 - y0))
    if a_full > float(p["max_area_frac"]) * roi_area_full:
        return dict(fail, transient=True,
                    reason="黄区面积占 ROI %.0f%% > %d%%（掩膜泛滥）—— "
                                 "多半白平衡/曝光没锁住、整幅染黄（或视野被摆臂/机构"
                                 "短暂挡住），不给数"
                                 % (100.0 * a_full / roi_area_full,
                                    int(100.0 * p["max_area_frac"])),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)
    # ---- 黄度对比度闸门（2026-10-09 加）：块内得比背景黄一大截，否则是阴影/杂色 ----
    #    治的是"Y−B 把灰绿墙的阴影当黄区 → 假 Δx"（详见 _warm_contrast 的 docstring）。
    #    背景(<500 采样点)判不了时放行（黄区占满窗 = 车很近的合法场景）。
    #    ⚠️ 2026-10-23 起也按瞬态处理（transient=True, 豁免票预算）: 对比度低同样是
    #       "分布没拉开"的病态——多等几帧比立刻失败好; 真阴影机位会一路低到 deadline 判负。
    contrast, _n_out = _warm_contrast(D, valid, lab, i)
    if contrast is not None and contrast < float(p["contrast_min"]):
        return dict(fail, transient=True,
                    reason="黄区不够黄（块内比背景仅黄 %.0f 级 < %.0f）—— "
                                 "锁到的多半是阴影/杂色块，不给数"
                                 % (contrast, p["contrast_min"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note, contrast=contrast)

    ys, xs = np.nonzero(lab == i)
    xmin = int(xs.min())
    ymed = int(np.median(ys[xs == xmin]))           # 最左那一列的代表 y
    bx, by = st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP]
    bw, bh = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]

    flags = FLAG_EDGE_OK
    # ⚠️ CLIP_X 只看**左边**：Δx 是"左边缘的 x"，只有它贴到画面左边才会饱和。
    #    黄区**右侧**跑出画面是常态（它本来就铺到画面右边），不该报"不可信"。
    if bx <= 2:
        flags |= FLAG_CLIP_X

    # ---- 左边缘截断交叉检查（2026-10-30 加）----
    # 只在「黄区即主峰」档（车很近/黄区占大头）：阈值 = 主峰−6，会把黄区偏暗的左端
    # 裁掉；掩膜左缘看起来像"真边界"，实际黄区已经延伸到 ROI 左外（03:32 现场实测：
    # 主阈 32 时左缘 x=274，低 20 级时整个块碰到 ROI 左边、面积 1.5 倍）。
    # 用同帧低 20 级再抠一次：主块中心落在的那个更大的块碰到 ROI 左边 → 真左缘出画
    # → 置 CLIP_X（run_road_detect 会立即以 status=1 放行给固件的饱和救援）。
    # 正常「谷底/小黄区」档不走这条（阈值本来就低，低 20 级已到地面，没有判据意义）。
    if (thr is not None) and float(thr) >= 12.0 and str(note).startswith("黄区即主峰"):
        thr_low = max(float(p["floor"]), float(thr) - 20.0)
        m2 = ((D > thr_low) & valid).astype(np.uint8) * 255
        ko2 = int(p["open_k"]) | 1
        kc2 = int(p["close_k"]) | 1
        m2 = cv2.morphologyEx(m2, cv2.MORPH_OPEN,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko2, ko2)))
        m2 = cv2.morphologyEx(m2, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc2, kc2)))
        n2, lab2, st2, _ce2 = cv2.connectedComponentsWithStats(m2, 8)
        if n2 >= 2:
            cxi = min(mask.shape[1] - 1, max(0, int(
                st[i, cv2.CC_STAT_LEFT] + st[i, cv2.CC_STAT_WIDTH] / 2)))
            cyi = min(mask.shape[0] - 1, max(0, int(
                st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT] / 2)))
            j = int(lab2[cyi, cxi])
            if j > 0 and st2[j, cv2.CC_STAT_LEFT] <= 2 and \
                    st2[j, cv2.CC_STAT_AREA] >= 1.2 * st[i, cv2.CC_STAT_AREA]:
                flags |= FLAG_CLIP_X
    if by <= 2 or by + bh >= hh - 2:
        flags |= FLAG_CLIP_Y
    if a_full < 2.0 * p["min_area"]:
        flags |= FLAG_SMALL

    # ---- Δy = 黄区**下边缘**的 y（喂单片机的横向修正）----
    # ⚠️ 为什么是**下**边缘而不是上边缘: 黄区固定在画面**右上角** ⇒ 上边缘永远被画面顶边
    #    裁掉（实测四个现场帧的 bbox top 全是 0、flags 全是 0x05 —— 那个 bit2 就是**上**
    #    边缘触发的），拿它当尺子等于量一个恒为 0 的常数。下边缘实测 278~576px 全在窗内。
    # ⚠️ 有效性只看**下边**有没有贴到 ROI/画面下沿（贴了就饱和）; 上边被裁无所谓。
    dy = y0 + (by + bh - 1) / sc                    # bbox 是 [top, top+height-1], 回全分辨率
    if by + bh < hh - 2:
        flags |= FLAG_Y_EDGE_OK

    return dict(success=True, reason="", flags=flags, mask=mask, scale=sc,
                roi=(x0, y0, x1, y1), area=a_full, thr=thr, note=note,
                contrast=contrast,
                dx=x0 + xmin / sc,                       # 回全分辨率
                dy=dy,
                edge=(x0 + xmin / sc, y0 + ymed / sc))


# ==================== 多帧投票 ====================
def _spread(vals):
    return (max(vals) - min(vals)) if vals else 0.0


def _median(vals):
    xs = sorted(vals)
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _ambiguous_gap(vals, p):
    """帧值是否分成**两簇**（两簇都够票数、间距超散布闸）→ 返回间距，否则 0。

    ⚠️ 必须在剔离群**之前**判（`_rect.py` 踩过同一个坑）：偶尔会有几帧锁到**另一块**
       黄区（车上的黄线标、另一条胶带），于是 5 帧读 A、4 帧读 B。MAD 会把少数那一簇
       当离群剔掉，剩下的看着又紧又稳 → 报出一个错值而 spread 还是 0。先说是哪一簇。
    """
    n = len(vals)
    if n < 2 * p["min_votes"]:
        return 0.0
    order = sorted(vals)
    for k in range(p["min_votes"], n - p["min_votes"] + 1):
        gap = order[k] - order[k - 1]
        if gap > p["max_spread_px"]:
            return gap
    return 0.0


def _inlier_mask(vals, p):
    """MAD 离群判据：|v − 中位| ≤ max(下限, k×1.4826×MAD)（抖得小就一个不剔）。"""
    med = _median(vals)
    dev = [abs(v - med) for v in vals]
    thr = max(p["outlier_min_px"], p["outlier_k"] * 1.4826 * _median(dev))
    return [d <= thr for d in dev]


def _annotate(frame, res, rev=False):
    """画 ROI、黄色区轮廓、左边缘竖线（全部 ASCII 文本，cv2 画不了中文）。
    rev=True（反向看路）时右上角画 REV 角标 —— 整幅是镜像系，别当正常取景读。"""
    if frame is None:
        return None
    out = frame.copy()
    if rev:
        cv2.putText(out, "REV", (out.shape[1] - 150, 42), 0, 1.2, (0, 128, 255), 3)
    res = res or {}
    roi = res.get("roi")
    if roi is not None:
        cv2.rectangle(out, (int(roi[0]), int(roi[1])), (int(roi[2]), int(roi[3])),
                      (128, 128, 128), 2)
    mask = res.get("mask")
    if mask is not None:
        sc = res.get("scale", 1.0) or 1.0
        ox, oy = (roi[0], roi[1]) if roi is not None else (0, 0)
        off = np.array([ox, oy], float)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            q = (c.reshape(-1, 2).astype(float) / sc + off).astype(np.int32)
            cv2.polylines(out, [q], True, _MASK_BGR, 3)
    e = res.get("edge")
    if e is not None:
        x, y = int(e[0]), int(e[1])
        cv2.line(out, (x, 0), (x, out.shape[0]), _EDGE_BGR, 2)
        cv2.circle(out, (x, y), 10, _EDGE_BGR, -1)
        # 黄区**下边缘**（横向那把尺）也画出来 + 标出它有没有被裁
        dy = res.get("dy")
        if dy:
            cv2.line(out, (0, int(dy)), (out.shape[1], int(dy)), _DY_BGR, 2)
        cv2.putText(out, "dx=%.1fpx edge(%d,%d) area=%.0f thr=%.0f fl=0x%02X c=%+d"
                    % (res.get("dx", 0.0), x, y, res.get("area", 0.0),
                       res.get("thr") or 0.0, res.get("flags", 0),
                       int(res.get("contrast") or 0)),
                    (20, 40), 0, 1.1, _EDGE_BGR, 3)
        if dy:
            cv2.putText(out, "dy=%.1fpx%s"
                        % (dy, "" if res.get("flags", 0) & FLAG_Y_EDGE_OK else " (clipped)"),
                        (20, 80), 0, 1.1, _DY_BGR, 3)
    return out


def run_road_detect(timeout=None, abort_event=None, rev=False):
    """主摄「看路」：量黄色区左边缘的 x（纵向）与**下边缘的 y**（横向）。
    限时 timeout 秒，默认 config.ROAD_TIMEOUT。

    rev=True（**反向看路**，2026-10-31 加）：整帧水平镜像后走同一套管线 —— 镜像系左
    边缘 = 真实画面的**右边缘**，ROI 自动左右对调，灰世界参考块换到镜像系右下（真实
    左下地面）。返回的 dx/dy/edge/归档帧全是**镜像系坐标**；比例/死区/方向符号与正向
    同套标定值，理想值按反向机位另标。串口命令 "M"（handler._cmd_road_rev，对齐固件
    RPI_TASK_ROAD_REV）。

    逐帧 `_detect_one` → 多帧去离群（顺序不能换：**先判双峰，再 MAD 剔跳档**）→ 内点取均值。
    ⚠️ 2026-10-23：掩膜泛滥这类**瞬态**坏帧不消耗票预算（只受 deadline 限总时长）——
       曝光模式交接/机构扫过视野的染黄期持续 ~2s, 把它烧在预算里会让"再等一秒就能成功"
       变成整步失败（01:06 run 实测 7/9 帧泛滥 → 差一帧凑不满 3 票）。
    ⚠️ 2026-10 早停：凑满 min_votes 且已见 Δx 极差 ≤ MAD 剔离群**下限**就提前收工。
       2026-10-30 起判据改为「**最近 min_votes 帧**的极差」且要求多攒一票 ——
       开头一帧没坐稳（现场 6~8px）不再拖到满帧；聚合口径不变（全部 hits 过同一套
       双峰/MAD），抖动/歧义在窗里照旧表现为极差大、自动采满。
       ⚠️ 额外要求**全程极差 ≤ max_spread_px** 才允许早停：车没停稳时前几帧漂、
       最近几帧又碰巧稳（06:36:20 road_0007 实测：最近 3 帧 2px、全程 28px），
       历史已经"不干净"就照旧采满，让最终散布闸看到全部证据再判。

    Returns dict: {"success", "reason", "frame"(标注帧), "dx", "dy", "edge"(ex,ey),
                   "area", "flags", "votes", "inliers", "dropped", "spread",
                   "dy_spread", "frames", "dy_frames", "thr"}
        `dx` = **Δx（黄色区左边缘的 x）**，全分辨率像素 → 单片机修车体纵向。
        `dy` = **Δy（黄色区下边缘的 y）**，全分辨率像素 → 单片机修车体横向
              （有效性看 flags 的 bit4 `FLAG_Y_EDGE_OK`）。
        aborted=True 只在被打断时出现。
    """
    from config import ROAD_TIMEOUT

    timeout = ROAD_TIMEOUT if timeout is None else timeout
    fail = {"success": False, "reason": "", "frame": None, "dx": 0.0, "dy": 0.0,
            "edge": None, "area": 0.0, "flags": 0, "votes": 0, "inliers": 0,
            "dropped": 0, "spread": 0.0, "dy_spread": 0.0, "frames": [], "thr": None}

    cap = acquire_camera(is_qr=False, profile="road")
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 看路任务被打断")
            return dict(fail, aborted=True)
        return dict(fail, reason="主摄像头打开失败")

    try:
        p = _params()
        # ⚠️ 2026-11-02: rev **不**改参考块位置（早先镜像过, 现场 road_0019 证明会漂到
        #    黄区边缘把黄区洗掉 —— 完整复盘见文件头「反向看路」节）。镜像后画面已回到
        #    正向布局, 正向的 ROI 左下块直接适用。
        print(f"[INFO] 开始看路{'（反向: 整帧镜像, 量黄区右边缘）' if rev else ''}"
              f"（黄色区左边缘，最多 {p['vote_frames']} 帧，"
              f"限时 {timeout:.0f}s）...")
        deadline = time.time() + timeout
        hits = []
        frames_seen = 0        # 计入票预算的帧（出测量结果 / 非瞬态失败）
        frames_tried = 0       # 实际处理的帧数（含豁免的瞬态坏帧）—— 只进日志口径
        read_fails = 0         # read 返回 None 的次数（2026-10-28 加, 见失败出口）
        last_fail = None       # (frame, res)：最近一帧失败检测 —— 0 命中时拿它出"验尸帧"
        last_raw = None        # 最近读到的原始帧 —— 打断/判废出口也带帧（统一记录接口,
                               # 见 handler._save_frame: 失败样本恰是排障最想看的）

        def _fail_frame():
            """失败/打断出口用的帧: 命中帧的标注 > 失败验尸帧 > 最后原始帧。"""
            if hits:
                return _annotate(hits[0][1], hits[0][0], rev)
            if last_fail is not None:
                return _annotate(last_fail[0], last_fail[1], rev)
            return last_raw

        while time.time() < deadline and frames_seen < p["vote_frames"]:
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 看路任务被打断")
                return dict(fail, aborted=True, frame=_fail_frame())
            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                read_fails += 1
                time.sleep(0.05)
                continue
            last_raw = frame
            if rev:
                # 反向: 整帧水平镜像后走同一套管线（见文件头「反向看路」节）。
                # 之后的检测/标注/归档全部留在镜像系 —— 与回传的 Δx/Δy 同一坐标系。
                frame = cv2.flip(frame, 1)
            frames_seen += 1
            frames_tried += 1
            r = _detect_one(frame, p)
            if r["success"]:
                # CLIP_X = 黄区左边缘贴 ROI 左边（Δx 饱和）——**检到了但不能当尺子**。
                # 2026-10-30：这种帧要**立即以 status=1 放行**给固件的「饱和救援」
                # （ROAD_RESCUE_MM：车体系后退一段 → 重发 "G" 重测一次）。固件只在
                # status=1 的帧上看 bit1；若这里判失败/不计票，固件收到 status=0 根本
                # 进不了救援分支（03:54 现场实测：9 帧全 CLIP_X 被排除 → 固件
                # "ROAD fail: wait st=0"，用户加的救援步距一次都没触发）。
                # 饱和帧的 dx 是窗框值，固件会丢弃并退后重测 —— 所以这里**不要求
                # 3 票、也不做 dx 范围/散布闸**；一帧就回，节省整步时间。
                if r["flags"] & FLAG_CLIP_X:
                    print(f"[INFO] 看路第 {frames_tried} 帧: "
                          f"{'右' if rev else '左'}边缘出画（Δx 饱和），"
                          f"放行给固件救援 (dx={r['dx']:.0f} area={r['area']:.0f})")
                    res = dict(r)
                    res.update(votes=1, inliers=1, dropped=0, spread=0.0,
                               dy_spread=0.0,
                               frames=[round(r["dx"], 1)],
                               dy_frames=[round(r["dy"], 1)])
                    res["frame"] = _annotate(frame, r, rev)
                    res["frame_raw"] = frame
                    return res
                hits.append((r, frame))
                # ---- 早停（见 docstring）：静止机位下后面的帧只会重复同一答案 ----
                # ⚠️ 2026-10-30：只看**最近 min_votes 帧**的极差（滑动窗）+ 多攒一票。
                #    开头一帧没坐稳（现场 6~8px）不再把整步拖到 9 帧；后段真稳时
                #    第 4 帧就收工。最终聚合仍是全部 hits 的 MAD 中位，窗口外的老帧
                #    照旧参与/被剔，只是"什么时候可以不再读"变聪明了。
                if len(hits) > p["min_votes"]:
                    recent = [h[0]["dx"] for h in hits[-p["min_votes"]:]]
                    # ⚠️ 再要求**全程极差**不超最终散布闸（`max_spread_px`）：06:36:20
                    #    现场有一次车没停稳（road_0007 运动模糊）——最近 3 帧 2px 很稳，
                    #    但前几帧漂了 28px；若只看最近窗会在 7 帧停下、然后被最终散布闸
                    #    （25px）判失败。加上这条，历史已不干净就照旧采满，让最终判据
                    #    拿到全部证据再下结论（与改版前行为一致）。
                    if _spread(recent) <= p["outlier_min_px"] and \
                            _spread([h[0]["dx"] for h in hits]) <= p["max_spread_px"]:
                        print(f"[INFO] 看路早停: {len(hits)} 帧, 最近 "
                              f"{p['min_votes']} 帧极差 {_spread(recent):.1f}px 已够稳")
                        break
            else:
                print(f"[WARN] 看路第 {frames_tried} 帧失败: {r['reason']}")
                if r.get("mask") is not None:
                    last_fail = (frame, r)
                if r.get("transient"):
                    # ⚠️ 瞬态坏帧（曝光交接/机构扫过视野造成的整幅染黄泛滥, 2026-10-23
                    #    01:06 run 实测连烧 7 帧预算直接失败）**不消耗票预算**, 多等它
                    #    过去 —— 只有 deadline 限住总时长, 不会无限循环。真没黄区那类
                    #    "场景答案"照旧计票, 车对着墙时行为与从前一致。
                    frames_seen -= 1
            time.sleep(0.01)

        if len(hits) < p["min_votes"]:
            reason = (f"看路失败：{frames_tried} 帧里只有 {len(hits)} 帧有效"
                      f"（需≥{p['min_votes']}）")
            print(f"[WARN] {reason}")
            best = hits[0] if hits else None
            shown = _annotate(best[1], best[0], rev) if best else None
            if shown is None and last_fail is not None:
                # 0 命中也要给"验尸帧"（2026-10-09 加）：画出被拒的掩膜 + 红字提示 ——
                # 否则桌面/无黄区场景下 Web 与 run 目录里**一张图都没有**，排障只能瞎猜
                # 相机当时在看什么。中文 reason 画不了（cv2 无中文字体），红字只给 ASCII
                # 指引，具体原因看日志（"黄区不够黄"= 锁到阴影/杂色；"没有黄色区"= 没抠到块）。
                shown = _annotate(last_fail[0], last_fail[1], rev)
                if shown is not None:
                    # 红字画在 (20,130)：别压住 _annotate 在 (20,40)/(20,80) 的读数。
                    c = last_fail[1].get("contrast")
                    cv2.putText(shown,
                                "FAILED: no yellow zone%s (see log)"
                                % ("" if c is None else " c=%+d" % int(c)),
                                (20, 130), 0, 1.1, (0, 0, 255), 3)
            if shown is None:
                shown = last_raw        # 连掩膜都没有 → 最后的原始帧也是现场证据
            # ⚠️ 失败也带**原始帧**（2026-10-23 加）: 离线重标 WB/曝光全靠它 ——
            #    只存标注帧的话, 检测量(sep/sat/对比度)都被画线污染, 没法做参数扫描。
            # ⚠️⚠️ **一帧都没读到**也要归档（2026-10-28 加）—— 上面的帧链
            #    （hits > last_fail > last_raw）全空 = "租约被回收/设备 3s 没出帧"这种
            #    最需要留现场的情况, 恰恰一张图都没有, frames/ 序号断档、回看网格里
            #    这次失败**凭空消失**（2026-10-27 run 实测: 12 次看路只有 10 张图,
            #    两次"0 帧"失败无任何影像）。处方: ①用当前 cap 再抢救性读一帧;
            #    ②读不到就生成**黑底时间戳帧**（内容写明"未取到帧 + 失败次数"）——
            #    归档记录的意义是"这次尝试存在、且当时什么都看不到", 黑帧本身就是证据。
            if shown is None:
                import numpy as _np
                import cv2 as _cv2
                probe = CameraManager.read(cap, "Main Camera", max_retries=1)
                if probe is not None:
                    shown = probe
                else:
                    shown = _np.zeros((1080, 1920, 3), _np.uint8)
                    _cv2.putText(shown,
                                 "NO FRAME (read_fails=%d, see log)" % read_fails,
                                 (20, 60), 0, 1.2, (0, 0, 255), 2)
                    _cv2.putText(shown, time.strftime("%H:%M:%S"),
                                 (20, 110), 0, 1.2, (0, 0, 255), 2)
            return dict(fail, reason=reason, votes=len(hits), frame=shown,
                        frame_raw=last_raw)

        dxs = [h[0]["dx"] for h in hits]
        # ① **先判双峰**（顺序反了会把少数那一簇当离群剔掉、静默报错值）
        gap = _ambiguous_gap(dxs, p)
        if gap > 0.0:
            reason = (f"Δx 分成两簇（差 {gap:.0f}px 且两边都够票数）—— "
                      f"可能锁到了另一块黄区，不给数")
            print(f"[WARN] {reason}  各帧 Δx={[round(v) for v in dxs]}")
            return dict(fail, reason=reason, votes=len(hits),
                        frame=_annotate(hits[0][1], hits[0][0], rev),
                        frames=[round(v, 1) for v in dxs], frame_raw=last_raw)

        # ② MAD 剔跳档
        keep = _inlier_mask(dxs, p)
        idx = [i for i in range(len(hits)) if keep[i]]
        dropped = len(hits) - len(idx)
        if len(idx) < p["min_votes"]:
            reason = (f"看路抖动过大：{len(hits)} 帧里剔除 {dropped} 个离群帧后只剩 "
                      f"{len(idx)} 帧")
            print(f"[WARN] {reason}  各帧 Δx={[round(v) for v in dxs]}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, frame=_annotate(hits[0][1], hits[0][0], rev))

        # ③ 内点取均值
        dx = sum(dxs[i] for i in idx) / len(idx)
        spread = _spread([dxs[i] for i in idx])
        if spread > p["max_spread_px"]:
            reason = f"Δx 跨帧散布 {spread:.0f}px 超限（车在动？）"
            print(f"[WARN] 看路{reason}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, spread=spread, frame=_fail_frame())
        # ---- Δx 上限: 超了说明认错了东西（画面宽 1920）, 照旧整帧判负 ----
        if dx > p["dx_max"]:
            reason = (f"Δx={dx:.0f}px 不在 [{p['dx_min']:.0f},{p['dx_max']:.0f}] 内"
                      f"（黄区几乎看不到 / 认错了东西？）")
            print(f"[WARN] 看路{reason}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, spread=spread, frame=_fail_frame())

        # ---- Δx 下限不再整帧判负（2026-10-30 加, 饱和救援）----
        # dx < dx_min = 黄区贴画面左缘（"横向全黄", Δx 饱和, 量的是窗框不是黄区）,
        # 旧的处置是把整帧判成失败 —— 但这正是"车冲过头"的独有症状, 单片机拿不到帧
        # 就没机会自救。现在**带 FLAG_CLIP_X 旗标放行**, 由单片机做"后退一段 → 重测"
        # 的饱和救援（固件 ucRpiRoadAlignInner 见旗标就后退 road_rescue_mm 再测一次,
        # **绝不消费饱和值** —— 值本身仍不可信, 这里只是把"饱和"这个事实递过去）。
        # ⚠️ 逐帧的 FLAG_CLIP_X（bx≤2, 本机 ROI_LEFT=0 时即 dx≤4px）通常已经 OR 进来;
        #     这里强制再补一次: dx 落在 (4, dx_min) 的缝里时逐帧旗标不置位, 不补的话
        #     固件会把 6px 当真数去修车。
        # ⚠️ 老固件收到这种帧会拿饱和 dx 算位移（钳位后 ±road_max_mm）—— 部署顺序:
        #     **先烧新固件, 再推本文件**（新固件见旗标就救援/判失败, 不吃饱和值）。
        dx_sat = False
        if dx < p["dx_min"]:
            dx_sat = True        # ⚠️ flags 聚合在下面才定义, 这里只能记布尔（旗标聚合后补）
            print(f"[WARN] 看路{'(反向)' if rev else ''}: Δx={dx:.1f}px < 下限 "
                  f"{p['dx_min']:.0f}px —— 黄区贴画面{'右' if rev else '左'}缘（Δx 饱和）, "
                  f"带 CLIP_X 旗标放行, 由单片机救援")

        edge = (sum(hits[i][0]["edge"][0] for i in idx) / len(idx),
                sum(hits[i][0]["edge"][1] for i in idx) / len(idx))
        rep = min(idx, key=lambda j: abs(hits[j][0]["dx"] - dx))
        flags = 0
        for i in idx:                       # 任何一帧贴边都算（保守）
            flags |= hits[i][0]["flags"]
        if dx_sat:                          # Δx 饱和放行（见上面 dx_min 闸门）: 强制带旗标
            flags |= FLAG_CLIP_X
        # Δy 跟着**同一批内点帧**取均值（双峰/MAD 仍只判 Δx）—— 同一块黄区、同一机位,
        # 拆成两套内点只会让两边都不一致。散布只进日志（**不对 dy 判失败**:
        # 横向那一轴单片机会自己按死区/钳位兜底, 这里再拦一次只会让整步白跑）。
        dys = [h[0]["dy"] for h in hits]
        dy = sum(dys[i] for i in idx) / len(idx)
        dy_spread = _spread([dys[i] for i in idx])
        res = dict(success=True, reason="", dx=dx, dy=dy, edge=edge,
                   area=hits[rep][0]["area"], flags=flags, votes=len(hits),
                   inliers=len(idx), dropped=dropped, spread=spread,
                   dy_spread=dy_spread, thr=hits[rep][0]["thr"],
                   contrast=hits[rep][0].get("contrast"),
                   frames=[round(v, 1) for v in dxs],
                   dy_frames=[round(v, 1) for v in dys])
        print("[INFO] 看路%s成功: Δx=%.1fpx 左边缘=(%.0f,%.0f) 面积=%.0f "
              "帧%d(剔%d) 散布%.1fpx fl=0x%02X thr=%.0f(%s) c=%+d"
              % ("(反向)" if rev else "", dx, edge[0], edge[1], res["area"], len(hits),
                 dropped, spread, flags, res["thr"] or 0.0, hits[rep][0]["note"],
                 int(hits[rep][0].get("contrast") or 0)))
        print("[INFO]   Δy=%.1fpx 散布%.1fpx %s"
              % (dy, dy_spread, "(有)" if flags & FLAG_Y_EDGE_OK else "(无效, 下边缘被裁)"))
        print("[INFO]   各帧 Δx=%s" % [round(v) for v in dxs])
        print("[INFO]   各帧 Δy=%s" % [round(v) for v in dys])
        # 画在**最接近最终值的那一帧**上（预览里看到的数就是返回的数）
        res["frame"] = _annotate(hits[rep][1],
                                 dict(hits[rep][0], dx=dx, dy=dy, edge=edge, flags=flags),
                                 rev)
        res["frame_raw"] = hits[rep][1]     # 未标注的原始帧（训练模式成对归档用）
        return res
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）；普通 cap → 真释放
