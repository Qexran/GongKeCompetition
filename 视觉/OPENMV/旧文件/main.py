# -*- coding: utf-8 -*-
"""
单线巡线 —— 侧装摄像头视觉程序（单色二分版）
================================
场地是「道路 + 边界区」的二分：只检测其中一种颜色，另一种由画面
其余部分自然构成，从构造上保证两边不重叠。

摄像头侧装、镜头朝下，灰/黄交界线始终横向（沿图像 X 轴）。

输出两个量：
  1. offset —— 横向位置偏差，归一化 [-100, 100]，0 = 交界线在目标位置
  2. angle  —— 航向角偏差，单位 0.1°，0 = 与道路平行（"水平"）

串口占位帧（协议未定，随时可改）：
  0xA5 0x5A [valid] [off_l] [off_h] [ang_l] [ang_h]
  off / ang 均为有符号 16 位小端。

固件注意：OpenMV 5.0.0 起 find_blobs 返回的 blob 字段是属性
（blob.cx / blob.pixels / blob.rect / blob.rotation），不带括号。
"""

import sensor, image, time, math
from pyb import UART
from machine import Pin, PWM

# ---------------- 传感器初始化 ----------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
# 固定曝光/增益/白平衡，保证亮度确定、颜色阈值稳定
sensor.set_auto_exposure(False, exposure_us=9000)   # 固定曝光时间(us)，需实测调
sensor.set_auto_gain(False, gain_db=20)              # 固定增益(dB)
sensor.set_auto_whitebal(False)                     # 固定白平衡
sensor.skip_frames(30)                              # 等设置生效
# 视实际安装方向，可能需要取消下面某几行注释（先实测再定）：
# sensor.set_hmirror(True)
# sensor.set_vflip(True)

# ---------------- 单色二分配置 ----------------
# 只检测一种颜色，另一种由画面其余部分构成（二分，不可能重叠）。
# REGION_ROLE 选择检测的是哪一侧：
#   'outside' = 道路外的黄色边界区（推荐，黄色比灰色好分）
#   'road'    = 灰色道路本身
REGION_ROLE = 'outside'

# 两种颜色阈值（实测标定值），按 REGION_ROLE 选用其一
GRAY_LAB   = (16, 34, -26, 0, -17, 9)   # 灰色道路
YELLOW_LAB = (36, 54, -32, 11, 7, 46)   # 黄色边界区

if REGION_ROLE == 'road':
    COLOR_LAB = GRAY_LAB
    BOX_COLOR = (0, 255, 0)          # 道路 → 绿框
else:
    COLOR_LAB = YELLOW_LAB
    BOX_COLOR = (255, 0, 0)          # 边界区 → 红框

PIXELS_THRESHOLD = 900   # 检测块至少这么多像素才有效
AREA_THRESHOLD   = 200

# ---------------- 控制参数 ----------------
TARGET_OFFSET = 0.0   # 目标横向偏移（相对图像中心，像素）。默认 0 = 居中
ERR_GAIN      = 200   # 误差映射系数：满偏(±半幅)对应 ±100
LENS_CORR_STRENGTH = 1.8   # 镜头畸变校正强度：1.0~3.0 越大校正越强；0 表示关闭
MAX_REGION_RATIO = 0.85    # 检测块面积占画面比例上限，超过视为不可信（阈值太松）
LED_BRIGHTNESS  = 50       # LED 补光亮度 0~100（0=灭，100=最亮）

# ---------------- 串口（协议未定，占位） ----------------
uart = UART(3, 115200, timeout_char=1000)

# ---------------- LED 补光（P6，PWM，常亮） ----------------
led = PWM(Pin("P6"), freq=1000, duty_u16=0)
led.duty_u16(int(LED_BRIGHTNESS / 100 * 65535))   # 按 LED_BRIGHTNESS 固定亮度

clock = time.clock()
last_offset = 0
last_angle  = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    img = img.lens_corr(strength=LENS_CORR_STRENGTH)   # 镜头畸变校正

    # 1) 只检测一种颜色
    blobs = img.find_blobs([COLOR_LAB],
                           pixels_threshold=PIXELS_THRESHOLD,
                           area_threshold=AREA_THRESHOLD, merge=True)

    valid  = False
    offset = last_offset
    angle  = last_angle

    if blobs:
        b = max(blobs, key=lambda x: x.pixels)   # 最大检测块

        # 二分校验：正常二分里，检测块不该占据几乎整个画面
        # （阈值太松、把另一侧也吞了，就会出现这种情况）
        frame_pixels = img.width() * img.height()
        if b.pixels <= MAX_REGION_RATIO * frame_pixels:
            valid = True

            # 2) 交界线 = 检测块面向画面中线的那条横边
            lat_span = img.height()
            center_y = lat_span / 2.0
            if b.cy > center_y:        # 块在下半部 → 交界在块上边
                edge = b.y
            else:                      # 块在上半部 → 交界在块下边
                edge = b.y + b.h

            # 3) 横向偏移
            error_px = (edge - lat_span / 2.0) - TARGET_OFFSET
            offset = int(error_px / lat_span * 2 * ERR_GAIN)
            offset = max(-100, min(100, offset))

            # 4) 航向角
            # 检测块的主轴旋转角 rotation（弧度，[0,π)）；横向目标对齐时≈0
            rot_deg = math.degrees(b.rotation)   # [0, 180)
            angle_deg = rot_deg
            if angle_deg > 90:                  # 折叠 180→0 方向的歧义
                angle_deg -= 180
            angle = int(angle_deg * 10)          # 0.1° 单位
            angle = max(-900, min(900, angle))

            last_offset = offset
            last_angle  = angle

        # 5) 可视化（无论可信与否都画框，方便观察异常）
        img.draw_rectangle(b.rect, color=BOX_COLOR)

    # 6) 状态输出（打印到 IDE 串口终端）
    print("valid=%d off=%d ang=%.1f" % (1 if valid else 0, offset, angle / 10.0))

    # 7) 串口输出（占位帧）
    o = offset & 0xFFFF
    a = angle & 0xFFFF
    uart.write(bytearray([0xA5, 0x5A, 1 if valid else 0,
                          o & 0xFF, (o >> 8) & 0xFF,
                          a & 0xFF, (a >> 8) & 0xFF]))
