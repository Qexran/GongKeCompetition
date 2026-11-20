# camera.py — 摄像头管理（GKS 工科赛）
#
# 移植自 cv_python/camera.py 的核心打开逻辑。当前为独立骨架，尚未接入
# 串口健康检查与 state 单例（TODO，后续任务接入时补）。
#
# 关键点：CameraManager.open() 里的 V4L2 初始化顺序不能乱 ——
#   先设分辨率 → 切 FOURCC → 再设分辨率 → 预热 15 帧并校验实际分辨率。
# 因为 V4L2 驱动在切换 FOURCC 时会把分辨率重置为该格式下的第一个可用值，
# 顺序错了会导致最终分辨率为"尽力而为"的错误值或花屏。
# 曝光 / 白平衡：open_camera() 每次打开都按 config 的档位重新应用（人质识别、看路各有
# 独立档位），见 CameraManager._apply_exposure / _apply_white_balance —— 两者都必须在
# 预热之前设。曝光那 15 帧是为了"别让识别头几帧还在等新曝光生效"；白平衡更狠：AWB 每次
# 开相机都从零重新收敛、要 ~2s（见 config.py 的 ROAD_AUTO_WB 注释），15 帧根本不够。
import os
import re

import cv2
import time

from config import FRAME_WIDTH, FRAME_HEIGHT


class CameraManager:
    @staticmethod
    def _resolve_index(index):
        """将设备节点解析为整数索引（若可能）。

        树莓派上 OpenCV 的 V4L2 后端按「名称/路径」打开不可靠，会报
        "backend is generally available but can't be used to capture by name"；
        按「整数索引」打开则稳定。这里把 /dev/camera_Qr -> /dev/video2 之类的
        符号链接解析成 2 再交给 cv2.VideoCapture。
        """
        try:
            real = os.path.realpath(index)
        except Exception:
            real = index
        m = re.match(r"/dev/video(\d+)$", real)
        return int(m.group(1)) if m else index

    @staticmethod
    def open(index, width=None, height=None, fourcc="MJPG", exposure=None, wb=None):
        """打开摄像头并完成 V4L2 初始化 + 预热验证。

        Args:
            index: 设备节点（如 /dev/camera_main 或 /dev/video0）
            width, height: 目标分辨率；None 时取 config 当前值
            fourcc: FOURCC 四字符（"MJPG"/"YUYV"），"" 表示不设置用默认
            exposure: (auto_exposure, 曝光值) 二元组；None = 不动曝光（test_camera.py
                这类直接调本函数的调试脚本走 None，保持驱动当前值）
            wb: (auto_wb, 固定色温K) 二元组；None = 不动白平衡（同上）。⚠️ 走生产路径
                请用 open_camera()：它会按档位**总是**给这一项（不设 ≠ 保持原样，见
                config.py 的 CAM_MAIN_AUTO_WB 注释）

        Returns:
            cv2.VideoCapture 对象；失败返回 None（调用方可据此重试）。
        """
        if width is None:
            from config import FRAME_WIDTH
            width = FRAME_WIDTH
        if height is None:
            from config import FRAME_HEIGHT
            height = FRAME_HEIGHT
        idx = CameraManager._resolve_index(index)
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 回退：不带 CAP_V4L2，让 OpenCV 自动选择后端（GCS 兜底做法，
            # 显式 CAP_V4L2 与 libv4l2 冲突时有时能绕过 UVC 状态残留问题）。
            cap.release()
            cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            print(f"[ERROR] 无法打开摄像头索引 {index}")
            cap.release()  # 释放可能已打开的文件描述符，避免内核 EBUSY
            return None

        # 分辨率 → FOURCC → 分辨率（顺序很重要，见文件头注释）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if fourcc:
            fourcc_val = cv2.VideoWriter_fourcc(*fourcc)
            cap.set(cv2.CAP_PROP_FOURCC, fourcc_val)
            actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_str = "".join(
                chr((actual_fourcc >> i) & 0xFF) for i in range(0, 32, 8)
            ).strip("\x00")
            print(f"[INFO] 摄像头 FOURCC: {fourcc_str} (请求: {fourcc})")
        else:
            print("[INFO] 摄像头 FOURCC: 使用默认格式（未设置）")

        # FOURCC 切换后重新设分辨率（驱动可能已重置），确保最终生效
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # 曝光与白平衡都要在预热之前设死：那 15 帧本来就是留给 UVC 稳定 AWB/AE 的，
        # 放到预热之后会让识别拿到的头几帧还在等新设置生效。
        # 白平衡先设：它换的是"整幅的色偏"，比曝光粗得多（实测原地切换 2~3 帧即稳），
        # 之后再设曝光，AE 收敛的那 15 帧同时把两件事都带到底。
        if wb:
            CameraManager._apply_white_balance(cap, wb[0], wb[1])
        if exposure:
            CameraManager._apply_exposure(cap, exposure[0], exposure[1])

        # 预热：丢弃前几帧（UVC 需时间稳定 AWB/AE），同时校验实际分辨率。
        # 某些 UVC 驱动 set() 后未立即生效，读到首帧才触发格式协商。
        warmup_ok = 0
        resolution_checked = False
        for i in range(15):
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                if not resolution_checked:
                    resolution_checked = True
                    if abs(int(w) - int(width)) > 5 or abs(int(h) - int(height)) > 5:
                        print(f"[WARN] 摄像头 {index} 分辨率异常 "
                              f"(第{i+1}帧 {w}x{h}, 期望 {width}x{height})，重设后重新预热")
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                        warmup_ok = 0
                        continue
                warmup_ok += 1
        if warmup_ok < 3:
            actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"[WARN] 摄像头 {index} 预热失败 "
                  f"({warmup_ok}/15 帧成功, 分辨率 {actual_w}x{actual_h})，关闭并返回 None")
            cap.release()
            return None

        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[INFO] 摄像头 {index} 打开成功: {actual_w}x{actual_h} @ {actual_fps}fps")
        return cap

    @staticmethod
    @staticmethod
    def _v4l2_set(cap, prop, value, tries=3, delay=0.05):
        """V4L2 控件 set 带**重试**（2026-10-09 加）。

        刚 open 的相机上偶发 set 返回 False（EBUSY 类瞬态）—— 单次调用失败就放弃
        会让该控件保持旧值：白平衡没关 → AWB 从头收敛 → 整幅逐帧漂色，看路在
        12:15:29 因此报了 Δx=192 的假成功（掩膜泛滥占 ROI 78%）。重试 ≤tries 次、
        间隔 delay 秒；仍失败才让调用方走"不支持"的告警分支。
        """
        for _ in range(max(1, tries)):
            if cap.set(prop, value):
                return True
            time.sleep(delay)
        return False

    @staticmethod
    def _apply_exposure(cap, auto_exposure, exposure_value):
        """应用 V4L2 曝光。每次打开相机调一次 —— 识别链路因此总能吃到 config 的值。

        auto_exposure: 3 = 自动（光圈优先）/ 1 = 手动。
            ⚠ cv2 的 CAP_V4L2 后端用的就是 V4L2 原始值（1/3），**不是** DShow 那套
            0.25/0.75（实测：set 1 后 v4l2-ctl 读回 1，set 3 读回 3）。
            ⚠️ 只认 1/3：别的值（如 0）驱动直接拒绝，"手动"就变成没设。
        exposure_value: 手动曝光，单位 100µs（100 = 10ms）。
            自动模式下该控件是 inactive、写了也被驱动忽略，所以只在手动模式写它，
            免得日志看着"设上了"其实没生效。

        设不上（相机不支持该控件）只告警不抛错：识别该照跑，不该因为曝光挂掉。
        """
        if not CameraManager._v4l2_set(cap, cv2.CAP_PROP_AUTO_EXPOSURE, auto_exposure):
            print(f"[WARN] 摄像头不支持 auto_exposure={auto_exposure}，曝光保持驱动当前值")
            return
        if int(auto_exposure) != 1:
            print(f"[INFO] 曝光: 自动 (auto_exposure={auto_exposure})")
            return
        if exposure_value is None:
            print("[WARN] 选了手动曝光但没给曝光值，保持驱动当前值")
            return
        if not CameraManager._v4l2_set(cap, cv2.CAP_PROP_EXPOSURE, exposure_value):
            print(f"[WARN] 摄像头不支持 exposure={exposure_value}，曝光保持驱动当前值")
            return
        got = cap.get(cv2.CAP_PROP_EXPOSURE)
        print(f"[INFO] 曝光: 手动 {exposure_value} (100µs) | 读回 {got}")

    @staticmethod
    def _apply_white_balance(cap, auto_wb, wb_temperature):
        """应用 V4L2 白平衡。每次打开相机调一次。

        auto_wb: 1 = 自动 / 0 = 手动固定（用 wb_temperature）。
            ⚠️ 与曝光同理，cv2 的 CAP_V4L2 后端这把就是 V4L2 原始值（bool 0/1）；
            色温控件在本相机是 2800~6500、步进 10（v4l2-ctl 实测）。
        wb_temperature: 固定色温[K]。自动模式下该控件是 inactive、写了也被驱动忽略，
            所以只在手动模式写它，免得日志看着"设上了"其实没生效。

        ⚠️ 为什么看路非固定不可：AWB 每次开相机都**从零重新收敛 ~2s**（实测 R−B 从 −47
        爬到 +6.3 才停），而看路是"开相机 → 预热 15 帧 → 立刻采 9 帧"，正好落在那段
        漂移里 → Δx 随时机漂移。固定档第 1 帧就稳。细节见 config.py 的 ROAD_AUTO_WB。

        设不上（相机不支持该控件）只告警不抛错：识别该照跑，不该因为白平衡挂掉。
        """
        if not CameraManager._v4l2_set(cap, cv2.CAP_PROP_AUTO_WB, auto_wb):
            print(f"[WARN] 摄像头不支持 auto_wb={auto_wb}，白平衡保持驱动当前值"
                  f"（已重试 3 次）—— ⚠️ 固定档设不上时 AWB 在自动收敛，"
                  f"看路可能整幅漂色/掩膜泛滥，注意核对本次测量")
            return
        if int(auto_wb) != 0:
            print(f"[INFO] 白平衡: 自动 (auto_wb={auto_wb})")
            return
        if wb_temperature is None:
            print("[WARN] 选了固定白平衡但没给色温，保持驱动当前值")
            return
        if not CameraManager._v4l2_set(cap, cv2.CAP_PROP_WB_TEMPERATURE, wb_temperature):
            print(f"[WARN] 摄像头不支持 wb_temperature={wb_temperature}，白平衡保持驱动当前值")
            return
        got = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        print(f"[INFO] 白平衡: 固定 {wb_temperature}K | 读回 {got}")

    @staticmethod
    def read(cap, camera_name="Camera", max_retries=3):
        """读取一帧，失败自动重试。失败返回 None。"""
        if cap is None:
            return None
        for _ in range(max_retries):
            ret, frame = cap.read()
            if ret:
                return frame
            time.sleep(0.05)
        time.sleep(0.1)  # 失败退避，避免调用方 while True 无延时刷屏
        return None

    @staticmethod
    def release(cap):
        if cap:
            cap.release()


# 相机档位表：profile 名 → config 里四个常量的**名字**（曝光 2 个 + 白平衡 2 个）。
# 存名字而不是值：open_camera 每次调用时现取，于是 Web「设置参数」改完下一次识别
# 就生效，不用重启进程。
# ⚠️ 四个都要给：不设 ≠ 保持原样 —— V4L2 控件状态**跨 open 保留**（看路把 AWB 关掉之后，
# 下一个开主摄的任务不设就继承了"关"）。所以每个档位都显式设满四个。
_CAMERA_PROFILES = {
    "main": ("CAM_MAIN_AUTO_EXPOSURE", "CAM_MAIN_EXPOSURE",
             "CAM_MAIN_AUTO_WB", "CAM_MAIN_WB_TEMPERATURE"),
    "qr": ("CAM_QR_AUTO_EXPOSURE", "CAM_QR_EXPOSURE",
           "CAM_QR_AUTO_WB", "CAM_QR_WB_TEMPERATURE"),
    "hostage": ("HOSTAGE_AUTO_EXPOSURE", "HOSTAGE_EXPOSURE",
                "HOSTAGE_AUTO_WB", "HOSTAGE_WB_TEMPERATURE"),
    # 看路（命令 "G"）：看地面。⚠️ 档名写错会**静默**回落到 "main"（见下面 get 的默认值），
    # 所以 tasks/_road.py 里那个 profile="road" 与这里必须对上。
    "road": ("ROAD_AUTO_EXPOSURE", "ROAD_EXPOSURE",
             "ROAD_AUTO_WB", "ROAD_WB_TEMPERATURE"),
}


def profile_params(profile):
    """档位名 → {"exposure": (模式, 值), "wb": (模式, 色温)}，值现取 config。

    唯一入口：open_camera()（整机开相机）与 camera_lease._reapply_profile()（租约就地
    换档）走的是**同一份**映射 —— 否则"预热时按 A 档、真用时按 B 档"就会两边不一致，
    而那种不一致的症状只是"识别偶尔差一点"，查不出来。
    """
    import config

    a_e, v_e, a_w, v_w = _CAMERA_PROFILES.get(profile, _CAMERA_PROFILES["main"])
    return {"exposure": (getattr(config, a_e), getattr(config, v_e)),
            "wb": (getattr(config, a_w), getattr(config, v_w))}


def open_camera(is_qr=False, width=None, height=None, profile=None):
    """按 config 配置打开主摄像头或扫码摄像头。失败返回 None。

    width/height 为 None 时取 config 当前值（支持 Web「设置参数」即时生效）。

    profile: 相机档位，决定用 config 里哪一组曝光+白平衡参数（见 _CAMERA_PROFILES）。
        None = 按 is_qr 走 "main"/"qr"；"hostage" = 人质识别专用档；"road" = 看路专用档
        （它固定白平衡，见 config.py 的 ROAD_AUTO_WB）。
    曝光与白平衡每次打开相机都重新应用（见 CameraManager._apply_exposure /
    _apply_white_balance）—— 各识别任务都是每次进来开一次相机，所以「每次识别都吃到
    当前 config 值」。
    """
    import config

    if width is None:
        width = config.FRAME_WIDTH
    if height is None:
        height = config.FRAME_HEIGHT
    idx = config.CAM_QR_INDEX if is_qr else config.CAM_MAIN_INDEX
    fourcc = config.CAM_QR_FOURCC if is_qr else config.CAM_MAIN_FOURCC

    p = profile_params(profile or ("qr" if is_qr else "main"))
    return CameraManager.open(idx, width, height, fourcc=fourcc,
                              exposure=p["exposure"], wb=p["wb"])
