# -*- coding: utf-8 -*-
"""
单线巡线 —— 侧装摄像头视觉程序
================================
场地：车道灰色，车道外黄色，灰/黄直接交界（无黑线）。
摄像头侧装、镜头朝下，画面里只有「灰色车道 + 黄色外部」，
中间一条灰/黄交界线。本程序只处理平直路段（不考虑弯道）。

输出两个量：
  1. offset —— 横向位置偏差，归一化 [-100, 100]，0 = 交界线在目标位置
  2. angle  —— 航向角偏差，单位 0.1°，0 = 与道路平行（"水平"）

说明：
  - offset 用灰/黄两个区域外接矩形的相邻边近似交界线位置。
  - angle 用灰色车道区域的主轴方向（与交界线平行）计算航向。
  - 两个量的正负号需实测确认后在 STM32 侧定方向。

串口占位帧（协议未定，随时可改）：
  0xA5 0x5A [valid] [off_l] [off_h] [ang_l] [ang_h]
  off / ang 均为有符号 16 位小端。
"""

import sensor, image, time, math
from pyb import UART

# ---------------- 传感器初始化 ----------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(30)
sensor.set_auto_gain(False)            # 关自动增益，保证颜色稳定
sensor.set_auto_whitebal(False)        # 关自动白平衡
# 视实际安装方向，可能需要取消下面某几行注释（先实测再定）：
# sensor.set_hmirror(True)
# sensor.set_vflip(True)

# ---------------- 标定参数（必须实测标定！） ----------------
# 灰色车道 RGB(195,196,198) 粗略起点：L~78 a~0 b~0
GRAY_LAB   = (50, 86, -23, 127, -128, 9)
# 黄色外部 RGB(246,247,207) 粗略起点：L~96 a~-4 b~18
YELLOW_LAB = (36, 100, -29, 127, 3, 127)
# 用 Tools → Machine Vision → Threshold Editor 分别标定，
# 关键保证灰/黄互不重叠（L 通道：灰<88、黄>85；b 通道：灰<15、黄>3）。

PIXELS_THRESHOLD = 200   # 区域至少这么多像素才有效
AREA_THRESHOLD   = 200

# ---------------- 控制参数 ----------------
TARGET_OFFSET = 0.0   # 目标横向偏移（相对图像中心，像素）。默认 0 = 居中
ERR_GAIN      = 200   # 误差映射系数：满偏(±半幅)对应 ±100

# ---------------- 串口（协议未定，占位） ----------------
uart = UART(3, 115200, timeout_char=1000)

clock = time.clock()
last_offset = 0
last_angle  = 0

def line_angle(x1, y1, x2, y2):
    """线段相对图像 X 轴的角度，范围 (-180, 180]"""
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

while True:
    clock.tick()
    img = sensor.snapshot()

    # 1) 分别找灰色车道和黄色外部区域
    gray   = img.find_blobs([GRAY_LAB],
                            pixels_threshold=PIXELS_THRESHOLD,
                            area_threshold=AREA_THRESHOLD, merge=True)
    yellow = img.find_blobs([YELLOW_LAB],
                            pixels_threshold=PIXELS_THRESHOLD,
                            area_threshold=AREA_THRESHOLD, merge=True)

    valid  = False
    offset = last_offset
    angle  = last_angle

    if gray and yellow:
        g = max(gray,   key=lambda b: b.pixels())   # 最大灰色块 = 车道
        y = max(yellow, key=lambda b: b.pixels())   # 最大黄色块 = 外部
        valid = True

        # 2) 判断交界线方向：两区域质心在哪个轴分得更开
        dx = abs(g.cx() - y.cx())
        dy = abs(g.cy() - y.cy())

        if dx > dy:
            lateral  = 'x'          # 交界线沿 Y（垂直），横向 = X
            lat_span = img.width()
            if g.cx() < y.cx():     # 灰左黄右
                edge = (g.x() + g.w() + y.x()) / 2.0
            else:                   # 黄左灰右
                edge = (y.x() + y.w() + g.x()) / 2.0
        else:
            lateral  = 'y'          # 交界线沿 X（水平），横向 = Y
            lat_span = img.height()
            if g.cy() < y.cy():     # 灰上黄下
                edge = (g.y() + g.h() + y.y()) / 2.0
            else:                   # 黄上灰下
                edge = (y.y() + y.h() + g.y()) / 2.0

        # ---- 3) 横向偏移 ----
        error_px = (edge - lat_span / 2.0) - TARGET_OFFSET
        offset = int(error_px / lat_span * 2 * ERR_GAIN)
        offset = max(-100, min(100, offset))

        # ---- 4) 航向角 ----
        # 灰色车道区域的主轴 = 行驶方向 = 与交界线平行
        lx1, ly1, lx2, ly2 = g.major_axis_line()
        angle_deg = line_angle(lx1, ly1, lx2, ly2)
        if lateral == 'x':
            angle_deg = angle_deg - 90.0   # 交界线垂直：对齐时主轴≈±90
        else:
            pass                            # 交界线水平：对齐时主轴≈0
        # 归一化到 (-90, 90]
        if angle_deg > 90:
            angle_deg -= 180
        elif angle_deg < -90:
            angle_deg += 180
        angle = int(angle_deg * 10)         # 0.1° 单位
        angle = max(-900, min(900, angle))

        last_offset = offset
        last_angle  = angle

        # ---- 5) 可视化 ----
        img.draw_rectangle(g.rect(), color=(0, 255, 0))        # 灰区绿框
        img.draw_rectangle(y.rect(), color=(255, 0, 0))        # 黄区红框
        img.draw_line(lx1, ly1, lx2, ly2,
                      color=(255, 0, 255), thickness=2)         # 主轴(品红)
        if lateral == 'x':
            img.draw_line(int(edge), 0, int(edge), img.height(),
                          color=(255, 255, 0), thickness=2)     # 交界线(黄)
            img.draw_line(int(lat_span / 2 + TARGET_OFFSET), 0,
                          int(lat_span / 2 + TARGET_OFFSET), img.height(),
                          color=(0, 0, 255))                     # 目标线(蓝)
        else:
            img.draw_line(0, int(edge), img.width(), int(edge),
                          color=(255, 255, 0), thickness=2)
            img.draw_line(0, int(lat_span / 2 + TARGET_OFFSET),
                          img.width(), int(lat_span / 2 + TARGET_OFFSET),
                          color=(0, 0, 255))

    # 6) 状态显示
    img.draw_string(0, 0,
                    "off:%d ang:%.1f %s" % (offset, angle / 10.0,
                                            "OK" if valid else "LOST"),
                    color=(255, 255, 255), scale=2)

    # 7) 串口输出（占位帧）
    o = offset & 0xFFFF
    a = angle & 0xFFFF
    uart.write(bytearray([0xA5, 0x5A, 1 if valid else 0,
                          o & 0xFF, (o >> 8) & 0xFF,
                          a & 0xFF, (a >> 8) & 0xFF]))

    # print("%.1f fps off=%d ang=%.1f valid=%d" % (clock.fps(), offset, angle/10.0, valid))
