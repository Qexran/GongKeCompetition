# OpenMV 竖置视觉巡线方案 — 详细实现计划

## 一、硬件布局

### 摄像头安装方式

OpenMV 竖置（旋转 90°），镜头朝下对准地面，安装在机器人底盘前部。

```
机器人俯视图：          摄像头竖置后的图像：

    ┌──────┐            ┌──────────────────┐
    │ 前方 │            │  上方 (边界颜色)  │ ← 图像顶部
    │      │            │  (246,247,207)   │
    │  📷 ←│ 竖置       │──────────────────│
    │      │            │  下方 (道路颜色)  │ ← 图像底部
    │ 底盘 │            │  (195,196,198)   │
    └──────┘            └──────────────────┘
```

竖置后，图像的 Y 轴对应机器人的左右方向：
- 图像上方 = 机器人一侧（如左侧）
- 图像下方 = 机器人另一侧（如右侧）
- 图像水平中线 = 机器人中心线

### 安装高度

建议距地面 80~120mm，视野覆盖路面宽度约 150~200mm。太近视野窄，太远分辨率低。

---

## 二、核心算法原理

### 判断逻辑

竖置摄像头看到的画面是一个 **横向截面**：

```
真实场景（从机器人前方看）：
         边界色带         道路         边界色带
    ─────────────────────────────────────────────
    (246,247,207)    (195,196,198)   (246,247,207)
    ─────────────────────────────────────────────

竖置摄像头图像（对应关系）：
    ┌─────────────────────────────────────────┐
    │  边界色  │      道路色       │  边界色  │  ← 图像顶部
    │         │                   │          │
    │  边界色  │      道路色       │  边界色  │  ← 图像底部
    └─────────────────────────────────────────┘
         左侧        中心          右侧
```

**关键：在图像水平方向上，分别统计左侧和右侧的道路色面积。**

- **左侧道路面积 > 右侧道路面积** → 机器人偏右，需要左纠偏
- **右侧道路面积 > 左侧道路面积** → 机器人偏左，需要右纠偏
- **两侧面积接近** → 居中，直行

### 为什么用面积差而不是绝对面积

绝对面积受光照、地面脏污影响大。**两侧面积的差值**天然抵消了这些干扰，更稳定。

---

## 三、图像处理流程

### Step 1：图像采集与预处理

```
原始图像 → 裁剪 ROI（图像下半部分） → LAB 转换 → 阈值二值化
```

- **裁剪 ROI**：只取图像下方 60%~70%，排除远端无关区域，减少噪声
- **LAB 转换**：比 RGB 对光线变化更鲁棒

### Step 2：颜色阈值分割

用 OpenMV 的 `img.find_blobs()` 分别检测两种颜色：

```python
# 道路灰色 (195, 196, 198) — 需实测标定 LAB 阈值
road_blobs = img.find_blobs([road_lab_threshold],
                             pixels_threshold=100,
                             area_threshold=100,
                             merge=True)

# 边界黄色 (246, 247, 207) — 需实测标定 LAB 阈值
boundary_blobs = img.find_blobs([boundary_lab_threshold],
                                 pixels_threshold=100,
                                 area_threshold=100,
                                 merge=True)
```

### Step 3：左右分区统计

将检测到的道路色团按 X 坐标分为左右两部分：

```python
img_center_x = img.width() // 2  # 图像水平中线

left_road_area = 0
right_road_area = 0

for blob in road_blobs:
    if blob.cx() < img_center_x:
        left_road_area += blob.pixels()
    else:
        right_road_area += blob.pixels()
```

### Step 4：计算偏移量

```python
# 偏移量：正值=偏右，负值=偏左
offset = left_road_area - right_road_area

# 归一化到 [-100, +100] 范围（可选）
total_area = left_road_area + right_road_area
if total_area > 0:
    normalized_offset = int(offset * 100 / total_area)
else:
    normalized_offset = 0  # 未检测到道路，保持上一次状态
```

### Step 5：发送给 STM32

保持现有帧格式 `0xA3 0xB3 [数据] 0xC3`：

```
帧结构（6字节）：
[0xA3] [0xB3] [偏移高字节] [偏移低字节] [面积高字节] [面积低字节] [0xC3]
```

偏移量为有符号 16 位整数（小端序），STM32 侧解析后叠加到 PID 输出。

---

## 四、LAB 阈值标定方法

### 在 OpenMV IDE 中操作

1. 连接 OpenMV，打开 **Tools → Frame Buffer**
2. 将摄像头对准实际场地，分别对准 **道路** 和 **边界** 区域
3. 打开 **Tools → Machine Vision → Threshold Editor**
4. 切换到 **LAB** 色彩空间
5. 调整 L/A/B 的 min/max 值，直到目标颜色区域变白、其他区域变黑
6. 记录两组阈值

### 预估阈值（基于 RGB 值推算，仅供参考）

道路 RGB(195, 196, 198) ≈ 浅灰色：
```python
# 道路灰色 LAB 阈值（需实测校准）
road_threshold = (40, 128, 128, 10, 125, 135)  # L:40-128, A:10-?, B:?
```

边界 RGB(246, 247, 207) ≈ 淡黄色：
```python
# 边界淡黄 LAB 阈值（需实测校准）
boundary_threshold = (200, 255, 100, 140, 150, 200)  # L:200-255, A:100-140, B:150-200
```

**⚠️ 以上阈值仅为推算，必须在实际场地上用 OpenMV IDE 标定！**

---

## 五、OpenMV 完整代码框架

```python
import sensor, image, pyb, struct, time

# ============ 初始化 ============
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_vflip(True)   # 根据实际安装方向调整
sensor.set_hmirror(True) # 根据实际安装方向调整
sensor.skip_frames(30)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

# ============ 串口初始化 ============
uart = pyb.UART(3, 115200, timeout_char=1000)

# 帧头帧尾
HEADER1 = 0xA3
HEADER2 = 0xB3
FOOTER  = 0xC3

# ============ 颜色阈值（LAB，需实测标定） ============
# 道路灰色 RGB(195,196,198)
road_threshold = (30, 60, 125, 135, 120, 135)  # TODO: 实测标定

# 边界黄色 RGB(246,247,207)
boundary_threshold = (200, 255, 100, 140, 150, 200)  # TODO: 实测标定

# ============ ROI 配置 ============
# 裁剪图像下半部分作为有效区域（排除远端噪声）
ROI_Y_START = 80    # 从第80行开始（240高度的图像，取下方160行）
ROI_H = 160         # ROI高度

# ============ 偏移滤波 ============
offset_buffer = []   # 滑动窗口滤波
FILTER_SIZE = 5      # 窗口大小

def filtered_offset(new_val):
    """滑动平均滤波，抑制抖动"""
    offset_buffer.append(new_val)
    if len(offset_buffer) > FILTER_SIZE:
        offset_buffer.pop(0)
    return int(sum(offset_buffer) / len(offset_buffer))

# ============ 主循环 ============
while True:
    clock.tick()
    img = sensor.snapshot()

    # --- 裁剪ROI ---
    roi_img = img.copy(roi=(0, ROI_Y_START, img.width(), ROI_H))

    # --- 检测道路色团 ---
    road_blobs = roi_img.find_blobs([road_threshold],
                                     pixels_threshold=80,
                                     area_threshold=80,
                                     merge=True)

    # --- 检测边界色团 ---
    boundary_blobs = roi_img.find_blobs([boundary_threshold],
                                         pixels_threshold=80,
                                         area_threshold=80,
                                         merge=True)

    # --- 左右分区统计道路面积 ---
    img_center_x = img.width() // 2
    left_road_area = 0
    right_road_area = 0

    for blob in road_blobs:
        # 调试：在图像上画出色团
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

        if blob.cx() < img_center_x:
            left_road_area += blob.pixels()
        else:
            right_road_area += blob.pixels()

    # 边界色团也画出来（调试用）
    for blob in boundary_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))

    # --- 计算偏移量 ---
    total_area = left_road_area + right_road_area
    if total_area > 100:  # 至少检测到一定面积才有效
        # 偏移量：正值=偏右需左纠，负值=偏左需右纠
        raw_offset = left_road_area - right_road_area
        # 归一化到 [-100, +100]
        offset = int(raw_offset * 100 / total_area)
        # 滤波
        offset = filtered_offset(offset)
    else:
        offset = 0  # 未检测到道路，不纠偏

    # --- 调试显示 ---
    img.draw_string(0, 0, "L:%d R:%d" % (left_road_area, right_road_area),
                    color=(255,255,255), scale=1)
    img.draw_string(0, 12, "Off:%d" % offset,
                    color=(255,255,255), scale=1)

    # --- 发送给STM32 ---
    # 有符号16位偏移量，小端序
    offset_u16 = offset & 0xFFFF  # 转为无符号处理字节
    packet = bytearray([
        HEADER1, HEADER2,
        (offset_u16 >> 8) & 0xFF,   # 偏移高字节
        offset_u16 & 0xFF,           # 偏移低字节
        (total_area >> 8) & 0xFF,    # 面积高字节（调试用）
        total_area & 0xFF,           # 面积低字节
        FOOTER
    ])
    uart.write(packet)

    # print("FPS:", clock.fps(), "Offset:", offset)
```

---

## 六、STM32 侧修改

### 1. UART 接收解析

在 `USART2_IRQHandler`（连接 OpenMV 的串口）中解析新的偏移帧：

```c
// 在现有 OpenMV 数据解析逻辑中增加偏移量解析
// 帧格式: [0xA3][0xB3][off_h][off_l][area_h][area_l][0xC3]

// 解析偏移量（有符号16位，小端序）
int16_t lane_offset = (int16_t)((data_buf[2] << 8) | data_buf[3]);

// 存入全局变量供主循环使用
extern int16_t lane_offset;
```

### 2. PID 控制中叠加纠偏

在 `TIM6_DAC_IRQHandler` 的电机控制中叠加偏移修正：

```c
// 在现有的 Moto_zitai 计算之后，叠加巡线纠偏
// lane_offset > 0 表示偏右，需要减小右侧速度/增大左侧速度

int lane_correction = lane_offset * LANE_GAIN;  // LANE_GAIN 需调试

// 将纠偏量叠加到四个电机
// 根据实际电机布局调整正负号
MOTOLU += lane_correction;   // 左前
MOTOLD += lane_correction;   // 左后
MOTORU -= lane_correction;   // 右前
MOTORD -= lane_correction;   // 右后
```

### 3. 新增全局变量

```c
// main.c 中新增
int16_t lane_offset = 0;        // 巡线偏移量
int8_t  lane_mode = 0;          // 0=关闭巡线，1=开启巡线
#define LANE_GAIN 2             // 纠偏增益，需调试
```

### 4. 通过按键/二维码控制开关

在状态机中根据任务阶段开启/关闭巡线：

```c
// 到达目标点前开启巡线，到达后关闭
if (到达减速区域) {
    lane_mode = 1;  // 开启视觉巡线
}
if (到达目标点) {
    lane_mode = 0;  // 关闭巡线，切换到任务模式
}
```

---

## 七、调试步骤

### Phase 1：阈值标定（单独调试 OpenMV）

1. 连接 OpenMV IDE，打开帧缓冲
2. 将摄像头竖置安装到实际位置
3. 分别对准道路和边界，用阈值编辑器标定 LAB 值
4. 确认两种颜色能被稳定区分
5. 在代码中打印 `road_blobs` 和 `boundary_blobs` 的数量和面积

### Phase 2：偏移量验证

1. 手动将机器人放在道路正中央，观察 `left_road_area ≈ right_road_area`
2. 将机器人向左偏移，确认 `right_road_area > left_road_area`
3. 将机器人向右偏移，确认 `left_road_area > right_road_area`
4. 观察偏移量的数值范围和稳定性

### Phase 3：串口通信验证

1. STM32 通过 OLED 显示接收到的 `lane_offset` 值
2. 确认值与 OpenMV IDE 打印的一致
3. 确认帧解析无误

### Phase 4：闭环调试

1. 先用小增益（`LANE_GAIN = 1`）开启巡线
2. 观察机器人是否能保持在道路中央
3. 逐步调整增益直到纠偏效果稳定
4. 测试直线、弯道、出界恢复等场景

---

## 八、潜在问题与对策

| 问题 | 原因 | 对策 |
|------|------|------|
| 阈值漂移 | 光照变化 | 用 LAB 空间 + 固定增益/白平衡；或增加自适应阈值 |
| 面积突变 | 地面脏污/反光 | 增大 `pixels_threshold`；滑动滤波窗口调大 |
| 纠偏震荡 | 增益过大 | 降低 `LANE_GAIN`；加死区（\|offset\| < 5 时不纠偏） |
| 丢失道路 | 摄像头视野不足 | 调整安装高度/角度；未检测到时保持上次偏移量 |
| 帧率不足 | 图像处理耗时 | 降低分辨率（QVGA→QQVGA）；减少 ROI 范围 |

---

## 九、任务阶段集成

在现有状态机中，巡线纠偏只在**直线行驶段**开启：

```
二维码 → 直线行驶(开巡线) → 排爆点(关巡线) → 排爆任务
       → 直线行驶(开巡线) → 反恐点(关巡线) → 反恐任务
       → 直线行驶(开巡线) → 救援点(关巡线) → 救援任务
```

转弯时关闭巡线（转弯时视野中只有边界色，无法正确判断偏移）。

---

## 十、改动文件清单

| 文件 | 改动内容 |
|------|----------|
| `vision/openmv/巡线摄像头.py` | **新建** — OpenMV 巡线主程序 |
| `firmware/stm32f407/app/main.c` | 新增 `lane_offset` 全局变量；UART 解析增加偏移帧；TIM6 中叠加纠偏；状态机中增加 lane_mode 开关 |
| `firmware/stm32f407/app/main.c` (OLED) | 增加巡线偏移量显示 |
