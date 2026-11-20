# tasks/_hostage.py — 人质形状识别（主摄像头 YOLOv8 ONNX，cv2.dnn 推理）
#
# 指令 "4"：识别 圆柱/圆锥/腰鼓 三种人质形状，凑满 HOSTAGE_MIN_TARGETS 个目标
# 即成功，按图像从左到右（中心 x 升序）排序。任务层只返回结果 dict，串口回传由
# handler.py::_cmd_hostage 统一发送（TYPE_COORD TAG=0x02 二进制帧，见
# docs/串口协议.md §6c），保持任务函数纯净可测（与 _qr.py 同构）。
#
# 模型/类名懒加载只做一次并缓存（仿 _qr.py::_get_wechat_detector）。
# 推理管线移植自 vision/scripts/dnn_infer.py（letterbox + blob + NMS + 坐标反映射），
# 类名 → 形状码按 config.HOSTAGE_NAME2SHAPE 映射（类 ID 随训练顺序变，类名稳定）。
import os
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

HOSTAGE_TAG = 0x02   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）

_NET = None       # cv2.dnn ONNX 网络（懒加载缓存）
_CLASSES = None   # 类名列表，与 classes.names 同序（懒加载缓存）

# 形状码 → 画面标注用英文短名（cv2.putText 画不了中文）
_SHAPE_EN = {1: "CYL", 2: "CONE", 3: "DRUM"}
_COLORS = {1: (0, 255, 0), 2: (0, 165, 255), 3: (255, 165, 0)}


def _get_net():
    """懒加载 YOLO ONNX 网络（加载较慢，只建一次并缓存）。文件缺失返回 None。"""
    global _NET
    if _NET is None:
        from config import HOSTAGE_MODEL_PATH
        if not os.path.exists(HOSTAGE_MODEL_PATH):
            return None
        _NET = cv2.dnn.readNetFromONNX(HOSTAGE_MODEL_PATH)
        _NET.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        _NET.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return _NET


def _get_classes():
    """读 classes.names（类 ID → 类名，与训练导出一致）。缺失返回 None。"""
    global _CLASSES
    if _CLASSES is None:
        from config import HOSTAGE_CLASSES_PATH
        try:
            with open(HOSTAGE_CLASSES_PATH, "r", encoding="utf-8") as f:
                names = [ln.strip() for ln in f if ln.strip()]
            if names:
                _CLASSES = names
        except Exception:
            pass
    return _CLASSES


def _letterbox(img, size):
    """保持宽高比 resize + 灰色 pad 到 size×size。返回 (padded, scale, pad_left, pad_top)。"""
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = size - nw, size - nh
    left, top = pad_w // 2, pad_h // 2
    padded = cv2.copyMakeBorder(
        resized, top, pad_h - top, left, pad_w - left,
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    return padded, r, left, top


def _detect(net, img, size, conf_thr, iou_thr):
    """单帧推理，返回目标列表（坐标在原始图像像素空间）；类名文件缺失返回 None。

    每项: {"shape", "shape_name", "cx", "cy", "conf", "box"}。
    未知类名（HOSTAGE_NAME2SHAPE 无映射）的目标丢弃（防御模型多训了别的类）。
    """
    from config import HOSTAGE_NAME2SHAPE, HOSTAGE_SHAPE_NAMES
    classes = _get_classes()
    if classes is None:
        return None
    h0, w0 = img.shape[:2]
    padded, r, left, top = _letterbox(img, size)
    blob = cv2.dnn.blobFromImage(padded, 1 / 255.0, (size, size), swapRB=True, crop=False)
    net.setInput(blob)
    out = net.forward()            # (1, 4+nc, anchors)

    pred = out[0].T                # (anchors, 4+nc)
    boxes = pred[:, :4]            # cx, cy, w, h（letterbox 空间）
    scores = pred[:, 4:]
    class_ids = scores.argmax(1)
    confs = scores.max(1)

    keep = confs > conf_thr
    boxes, confs, class_ids = boxes[keep], confs[keep], class_ids[keep]
    if len(boxes) == 0:
        return []

    cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = np.clip((cx - w / 2 - left) / r, 0, w0)
    y1 = np.clip((cy - h / 2 - top) / r, 0, h0)
    x2 = np.clip((cx + w / 2 - left) / r, 0, w0)
    y2 = np.clip((cy + h / 2 - top) / r, 0, h0)

    xywh = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)
    idx = cv2.dnn.NMSBoxes(xywh.tolist(), confs.tolist(), conf_thr, iou_thr)
    idx = idx.flatten() if len(idx) > 0 else []

    dets = []
    for i in idx:
        cid = int(class_ids[i])
        if cid >= len(classes):
            continue
        shape = HOSTAGE_NAME2SHAPE.get(classes[cid].lower())
        if shape is None:
            continue
        bx1, by1, bx2, by2 = int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])
        dets.append({
            "shape": shape,
            "shape_name": HOSTAGE_SHAPE_NAMES.get(shape, classes[cid]),
            "cx": (bx1 + bx2) // 2, "cy": (by1 + by2) // 2,
            "conf": float(confs[i]), "box": (bx1, by1, bx2, by2),
        })
    return dets


def _apply_elimination(dets):
    """台面先验的排除法重标（2026-10-02 加）。

    用户定律：台面恒为 圆柱/圆锥/腰鼓 **各一个**，绝无两个同形。YOLO 偶发把侧影
    （半个物体入画/压相似背景）认错形 → 单帧凑满 3 个却出现 {A,A,B}（两个同形）——
    单片机按「第二个物体」定位运动坐标，重复形必错一个。规则：重复形中置信度
    ≥ HOSTAGE_ANCHOR_CONF 的那枚当**锚点**（它就是这个形），另一枚较低的**不是**
    这个形 → 按排除法重标为缺席的第三形（位置不动，只改判标签）。

    只处理恰有唯一解的 {A,A,B} 型（恰好一个形重复一次 + 恰好一个形缺席）；
    {A,A,A}（推不出谁是谁）、三形齐全、两目标帧一律不动——弱证据不改判，交给后续帧。
    返回 (dets, note)：note 为 None 或一行给日志的说明；重标目标带 "inferred" 标记
    （标注图上以 INF 后缀显示，串口帧不带它——坐标/形状码本身已按新判发出）。
    """
    from config import HOSTAGE_ANCHOR_CONF, HOSTAGE_SHAPE_NAMES
    if len(dets) < 3:
        return dets, None
    counts = {}
    for d in dets:
        counts[d["shape"]] = counts.get(d["shape"], 0) + 1
    missing = set(HOSTAGE_SHAPE_NAMES) - set(counts)
    duplicated = [s for s, c in counts.items() if c == 2]
    if len(missing) != 1 or len(duplicated) != 1:
        return dets, None
    anchor, wrong = sorted((d for d in dets if d["shape"] == duplicated[0]),
                           key=lambda d: -d["conf"])
    if anchor["conf"] < HOSTAGE_ANCHOR_CONF:
        return dets, None          # 锚点不够硬：宁可用后续帧，不拿弱证据改判
    old_name = wrong["shape_name"]
    wrong["shape"] = next(iter(missing))
    wrong["shape_name"] = HOSTAGE_SHAPE_NAMES.get(wrong["shape"], "?")
    wrong["inferred"] = True
    return dets, (f"排除法重标：{anchor['shape_name']} conf={anchor['conf']:.2f} 为锚点，"
                  f"重复的 {old_name} conf={wrong['conf']:.2f} → 改判 {wrong['shape_name']}")


def _annotate(frame, dets):
    """画检测框 + **中心十字** + 「序号.形状 (cx,cy) 置信度」标签（序号 = 从左到右顺序）。

    中心像素与色球识别同款标注（见 tasks/_color.py::_annotate）: 现场标定「人质对位」的
    hs_cx_center / hs_cy_center 就是照这里的数填的 —— Web 预览上直接读得出每只的中心像素，
    不用再靠蓝牙遥测（hostage{1,2,3}_x/y）反查。十字的意义是「这个坐标到底落在物体哪个
    点上」一眼可见：质心偏上/偏下（三种形状高度不同）会影响 cy 的读数，光看框看不出来。

    ⚠️ 画标注只是给人看的，**失败不该连累识别结果**：每个目标的绘制各自兜在 try 里，
    一个画不出来只是少画那一个（并打一行日志），不回滚其余、更不抛出 ——
    这一刻真正要保住的是 targets 与串口回传，不是这张预览图。标定工具等外部调用
    可能传不全 box/cx/cy，同样不该把整轮识别搞崩。
    """
    out = frame.copy()
    w = out.shape[1]
    for rank, d in enumerate(dets, 1):
        try:
            color = _COLORS.get(d["shape"], (255, 0, 255))
            cx, cy = int(d["cx"]), int(d["cy"])
            box = d.get("box")
            if box is not None:
                x1, y1, x2, y2 = [int(v) for v in box]
                cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            else:
                x1, y1 = cx - 40, cy - 40          # 没框就按中心退化成一个小方框
                cv2.rectangle(out, (x1, y1), (cx + 40, cy + 40), color, 2)
            # 中心十字: 与色球识别同一画法（MARKER_CROSS，28px）
            cv2.drawMarker(out, (cx, cy), color, cv2.MARKER_CROSS, 28, 3)
            label = f"{rank}.{_SHAPE_EN.get(d['shape'], '?')} ({cx},{cy}) {d['conf']:.2f}"
            if d.get("inferred"):
                label += " INF"    # 排除法改判的目标：框/十字仍是它，标签已按新判
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            # 贴右边缘的物体: 标签块整体左移, 免得被画面裁掉半截（标定读数时最需要看清的就是它）
            bx = max(0, min(x1, w - tw - 1))
            cv2.rectangle(out, (bx, max(0, y1 - th - 10)), (bx + tw, y1), color, -1)
            cv2.putText(out, label, (bx, max(th + 2, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)
        except Exception as e:
            print(f"[WARN] 人质标注第 {rank} 个目标画失败（跳过它, 识别结果不受影响）: {e}")
    return out


def build_result_frame(ok, targets):
    """打包回传帧 DATA（见 docs/串口协议.md §6c）：
    [TAG=0x02][status u8][count u8] + count×([shape u8][cx u16 LE][cy u16 LE])，
    小端对齐单片机端解析（与巡线帧同约定）。失败帧 = [0x02][00][00]（count=0）。
    """
    data = struct.pack("<BBB", HOSTAGE_TAG, 1 if ok else 0, len(targets))
    for t in targets:
        data += struct.pack("<BHH", t["shape"], t["cx"], t["cy"])
    return data


def _open_hostage_with_retry(abort_event, attempts=5, delay=0.3):
    """打开主摄像头，失败时少量重试。

    Web 触发识别前会先断开主摄推流，但推流线程释放设备需要一小段时间，
    这里重试等待设备空闲（EBUSY）。串口直接调用时同样受益。

    曝光走 "hostage" 档（config.HOSTAGE_AUTO_EXPOSURE / HOSTAGE_EXPOSURE），
    与色球/靶标用的 CAM_MAIN_* 互相独立 —— 同一个物理相机，但人质可以有自己的亮度。
    ⚠️ 走租约时这个档位**必须**被尊重：camera_lease 命中租约但档位不同时会在
       **已开的设备上就地重设曝光与白平衡**并丢掉收敛期那几帧（见 _reapply_profile）——
    否则主摄若被 "C" 按 main 档预热过，人质就会拿到色球那档曝光而识别失败。
    """
    for i in range(attempts):
        if abort_event is not None and abort_event.is_set():
            return None
        cap = acquire_camera(is_qr=False, profile="hostage")   # 主摄像头（人质曝光档）
        if cap is not None:
            return cap
        print(f"[WARN] 主摄像头打开失败 (第{i + 1}/{attempts}次)，稍后重试...")
        time.sleep(delay)
    return None


def _get_timeout(timeout):
    """timeout 为 None 时从 config 取当前值（支持 Web「设置参数」即时生效）。"""
    if timeout is None:
        from config import HOSTAGE_TIMEOUT
        return HOSTAGE_TIMEOUT
    return timeout


def run_hostage_detect(timeout=None, abort_event=None):
    """主摄像头人质形状识别（限时 timeout 秒，默认取 config.HOSTAGE_TIMEOUT）。

    循环读帧检测，凑满 HOSTAGE_MIN_TARGETS 个目标即成功；成功目标按中心 x
    升序（图像从左到右）排序，超过 HOSTAGE_MAX_TARGETS 按置信度取前 N。

    Returns dict: {"success", "reason", "frame", "targets", "aborted"(仅打断时)}；
        targets = [{"shape", "shape_name", "cx", "cy", "conf"}, ...]
        超时失败时 targets 为「最好成绩」（仅 Web 显示用，单片机端收失败帧）。
    """
    from config import (HOSTAGE_CONF_THR, HOSTAGE_IOU_THR, HOSTAGE_INPUT_SIZE,
                        HOSTAGE_MAX_TARGETS, HOSTAGE_MIN_TARGETS)

    net = _get_net()
    if net is None:
        return {"success": False, "reason": "YOLO 模型缺失 (gks_yolo_model/best.onnx)",
                "frame": None, "targets": []}
    if _get_classes() is None:
        return {"success": False, "reason": "类名文件缺失 (gks_yolo_model/classes.names)",
                "frame": None, "targets": []}

    timeout = _get_timeout(timeout)
    cap = _open_hostage_with_retry(abort_event)
    if cap is None:
        # 打开阶段被打断同样视为「主动放弃」，而非失败
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 人质识别任务被打断")
            return {"success": False, "aborted": True, "reason": "",
                    "frame": None, "targets": []}
        return {"success": False, "reason": "主摄像头打开失败",
                "frame": None, "targets": []}

    try:
        print(f"[INFO] 开始人质形状检测，凑满 {HOSTAGE_MIN_TARGETS} 个目标，"
              f"限时 {timeout:.0f}s...")
        deadline = time.time() + timeout
        best = []            # 单帧最佳成绩（目标数最多的那帧）
        best_frame = None

        while time.time() < deadline:
            # 可随时打断：每一轮先检查中断信号
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 人质识别任务被打断")
                return {"success": False, "aborted": True, "reason": "",
                        "frame": best_frame, "targets": best}

            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue

            dets = _detect(net, frame, HOSTAGE_INPUT_SIZE,
                           HOSTAGE_CONF_THR, HOSTAGE_IOU_THR)
            if dets is None:
                time.sleep(0.05)
                continue
            if len(dets) > HOSTAGE_MAX_TARGETS:
                dets = sorted(dets, key=lambda d: -d["conf"])[:HOSTAGE_MAX_TARGETS]
            # 台面先验：凑满 3 个但两个同形时按排除法重标（_apply_elimination）。
            # 必须在成功判定**之前**——不然 {A,A,B} 帧直接当成功发出，重复形到单片机。
            dets, elim_note = _apply_elimination(dets)
            if elim_note:
                print(f"[INFO] 人质识别{elim_note}")
            if len(dets) > len(best):
                best, best_frame = dets, frame

            if len(best) >= HOSTAGE_MIN_TARGETS:
                best = sorted(best, key=lambda d: d["cx"])   # 从左到右
                annotated = _annotate(best_frame, best)
                names = " ".join(d["shape_name"] for d in best)
                print(f"[INFO] 人质识别成功（从左到右）: {names}")
                # 结果回传由 handler.py::_cmd_hostage 统一发送，任务层不碰串口
                return {"success": True, "reason": "",
                        "frame": annotated, "targets": best}

            time.sleep(0.01)  # 避免空转刷满 CPU

        best = sorted(best, key=lambda d: d["cx"])
        annotated = _annotate(best_frame, best) if best_frame is not None else None
        reason = (f"人质识别超时 ({timeout:.0f}s)：最多检出 "
                  f"{len(best)}/{HOSTAGE_MIN_TARGETS} 个目标")
        print(f"[WARN] {reason}")
        return {"success": False, "reason": reason,
                "frame": annotated, "targets": best}
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
