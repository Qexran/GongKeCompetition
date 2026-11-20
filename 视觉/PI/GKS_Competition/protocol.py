# protocol.py — 单片机(ST M32等) ↔ 树莓派 串口帧协议
#
# 帧格式 (双向统一):
#   SOF | TYPE | LEN | DATA(LEN) | CRC8 | EOF
#   - CRC8 覆盖 TYPE+LEN+DATA (查表 CRC8, 初值=首字节)
#   - LEN+CRC8 已能定界, EOF 作解析状态机重同步的二次确认
#
# ⚠ 本文件是双端契约: 帧常量 / TYPE 码 / CRC8 表任一改动,
#   必须同步到单片机端 (串口协议头文件), 否则通信全断。
#   交付文档见 docs/串口协议.md —— 单片机端适配时以此为准。

SOF = 0xAA
EOF = 0x55
FRAME_MAX_LEN = 64

# ---- 帧类型 (TYPE) —— 单片机端须与此一致 ----
TYPE_CMD   = 0x01   # 任务指令  : DATA = 命令码 (ASCII '1'/'2'/'end' 等)
TYPE_COORD = 0x02   # 结果回传  : DATA = 坐标/状态 (X(4) Y(4) COLOR(1) 等)
TYPE_QR    = 0x03   # 二维码任务: DATA = 6B 颜色码 (预留)
TYPE_HELLO = 0x10   # 握手
TYPE_ACK   = 0x20   # 应答
TYPE_DEBUG = 0xDB   # 调试字符串 (单片机 → Pi)

# ---- COORD 帧的 TAG（DATA 首字节, 单片机端 app_rpi.h 同名宏须一致）----
# 已占用: 0x01 巡线 / 0x02 人质 / 0x03 色球 / 0x04 靶标 / 0x05 靶标流 / 0x06 拍照 /
#         0x07 矩形地板 / 0x08 看路 / 0x09 看路校准 / 0x0A Pi 参数 / 0x0B 爆爆桶流 /
#         0x0C 蜂鸣命令 / 0x0D 相机健康检查
TAG_BEEP = 0x0C   # 蜂鸣命令 (Pi → 单片机, 2026-10-27 加): DATA = [0x0C][pattern]
TAG_HEALTH = 0x0D  # 相机健康检查 (命令 "Z" 的回应, 2026-11 加):
                   # DATA = [0x0D][M][Q][Pi][MV], 每项 1=通过 0=失败;
                   # 须与固件 app_rpi.h::RPI_HEALTH_TAG 一致, 见 docs/串口协议.md §6o

# ---- 蜂鸣音型（TAG_BEEP 帧第 2 字节; 与固件 app_beam.h::BUZZER_PAT_* 同值, 跨仓库契约）----
BEEP_PAT_FAIL = 1  # 失败音: 响 0.8s → 停 0.5s → 响 0.8s（共 2.1s; 固件蜂鸣任务异步执行）
BEEP_PAT_TICK = 2  # 短滴: 响 0.2s 一声
# ⚠ 固件自己也会在失败点投失败音（任务失败/超时、看路失败、平移精调环丢靶、陀螺仪损坏;
#   详见固件 docs/pi-link.md 的「蜂鸣命令」节）—— 这些同源事件**别再发 TAG_BEEP, 会双响**。
#   本命令留给 Pi 独有的判据（内部重试耗尽、Web/调试手动触发等）。

# ---- CRC8 查表 (与 gcs_stm32 一致, 沿用经实战验证的表) ----
_CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]


def crc8(data: bytes) -> int:
    """初值=首字节, 逐字节查表迭代。空数据返回 0。"""
    if not data:
        return 0
    crc = data[0]
    for b in data[1:]:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def pack_frame(ftype: int, data: bytes = b"") -> bytes:
    """打包一帧: SOF + TYPE + LEN + DATA + CRC8(TYPE+LEN+DATA) + EOF"""
    if len(data) > FRAME_MAX_LEN:
        raise ValueError(f"帧数据过长: {len(data)} > {FRAME_MAX_LEN}")
    body = bytes((ftype & 0xFF, len(data))) + data
    return bytes((SOF,)) + body + bytes((crc8(body), EOF))


class FrameParser:
    """增量帧解析状态机。

    逐字节喂入 feed(); 收到一帧完整且 CRC/EOF 校验通过时返回 (ftype, data),
    否则返回 None。校验失败/超长自动丢弃并重同步, 累加 err_count。
    """

    _WAIT_SOF, _TYPE, _LEN, _DATA, _CRC, _EOF = range(6)

    def __init__(self):
        self.err_count = 0
        self.reset()

    def reset(self):
        self._state = self._WAIT_SOF
        self._ftype = 0
        self._len = 0
        self._data = bytearray()

    def feed(self, byte: int):
        st = self._state
        if st == self._WAIT_SOF:
            if byte == SOF:
                self._state = self._TYPE
        elif st == self._TYPE:
            self._ftype = byte
            self._state = self._LEN
        elif st == self._LEN:
            self._len = byte
            self._data = bytearray()
            if byte > FRAME_MAX_LEN:
                self.err_count += 1
                self._resync(byte)                     # 超长, 丢弃并重同步
            elif byte == 0:
                self._state = self._CRC
            else:
                self._state = self._DATA
        elif st == self._DATA:
            self._data.append(byte)
            if len(self._data) >= self._len:
                self._state = self._CRC
        elif st == self._CRC:
            calc = crc8(bytes((self._ftype, self._len)) + self._data)
            if byte == calc:
                self._state = self._EOF
            else:
                self.err_count += 1
                self._resync(byte)                     # CRC 失败, 丢弃重同步
        elif st == self._EOF:
            if byte == EOF:
                self._state = self._WAIT_SOF
                return (self._ftype, bytes(self._data))  # 完整帧就绪
            self.err_count += 1                        # 帧尾不符, 丢弃重同步
            self._resync(byte)
        return None

    def _resync(self, byte):
        """丢弃当前帧后重同步 —— **把刚吃掉的那个字节再当一次帧头看**。

        ⚠️ 原来是直接 `self._state = self._WAIT_SOF` 就完事: 那个字节若**正是下一帧的
        SOF**, 它就被白白吃掉 → 下一帧整个丢掉, 要等再下一帧才恢复（对端每帧一次
        心跳/结果帧, 代价是一整帧的结果）。固件那侧的解析是逐字节重扫的
        （app_rpi.c 的扫帧: 帧头/长度/EOF/CRC 任一不过就丢 1 字节重新同步）,
        这里对齐它。
        """
        if byte == SOF:
            self._state = self._TYPE
        else:
            self._state = self._WAIT_SOF
