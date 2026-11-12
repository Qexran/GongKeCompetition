# -*- coding: utf-8 -*-
"""
P6 LED 亮度控制测试（PWM 呼吸灯）
用 machine.PWM 自动匹配 P6 的 PWM 通道，占空比控制亮度。
"""
import time
from machine import Pin, PWM

# ---- PWM 初始化 ----
pwm = PWM(Pin("P6"), freq=1000, duty_u16=0)   # 1kHz，初始熄灭

# ---- 设定固定亮度 ----
# duty_u16 范围 0~65535（0=灭，65535=最亮）
pwm.duty_u16(32768)   # 50% 亮度
time.sleep(1)

# ---- 呼吸灯演示（亮度 0→100%→0 循环）----
while True:
    for d in range(0, 65536, 4096):      # 渐亮
        pwm.duty_u16(d)
        time.sleep_ms(20)
    for d in range(65535, -1, -4096):    # 渐暗
        pwm.duty_u16(d)
        time.sleep_ms(20)
