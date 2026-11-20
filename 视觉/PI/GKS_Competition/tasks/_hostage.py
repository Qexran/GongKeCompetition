# tasks/_hostage.py — 人质形状识别（主摄像头 YOLOv8 ONNX，cv2.dnn 推理）
#
# 指令 "4"：识别 圆柱/圆锥/腰鼓 三种人质形状，凑满 HOSTAGE_MIN_TARGETS 个目标
# 即成功，按图像从左到右（中心 x 升序）排序。任务层只返回结果 dict，串口回传由
# handler.py::_cmd_hostage 统一发送（TYPE_COORD TAG=0x02 二进制帧，见
# docs/串口协议.md §6c），保持任务函数纯净可测（与 _qr.py 同构）。
#
# ---- 跨帧并集 + 缺只补救（2026-10-26 加，对标色球四件套的适配版）----
# ① 跨帧并集: 旧版"单帧最佳"要求单帧三只全中（检出率连乘），YOLO 1.5s/帧一次
#    只跑 3~4 帧, 单帧全中很脆。现按形状归并（每形状取最高 conf 的检出）——
#    前提"圆柱/圆锥/腰鼓各一只"（赛题规则, 2026-10-26 用户确认; ⚠️ 决赛若改题
#    允许重复形状 → 关 HOSTAGE_UNC_EN 退回单帧最佳旧行为, 不用删代码）。
# ② 次级候选池: conf 落在 [HOSTAGE_CONF_LO, CONF_THR) 的框不丢, 同样按形状
#    归并; 并集不齐时池里有缺的形状 → 实测坐标直接补。
# ③ 几何补齐（_hstg_rescue, 判据详见其 docstring）: 缺中间=端部连线中点(池里
#    中点近邻优先); 缺端部=等距外推(150mm 物理等距)。**端部坐标不被消费**(平移
#    走 one_lat_mm 查表), 糙点无所谓; 中间坐标是硬需求(对位基准)。
# ⚠️ **人质侧不做色球那套"±固定距离"拉回, 用两层更优的替代（2026-10-26 用户定稿）**:
#   · 剩 2 只 → 几何补齐(③)即成功: 补出的坐标不被消费, 对位用实测中间把车修回
#     正位（对位本身就是修正）。
#   · 剩 1 只 → **单目标拉回**（固件侧, ucRpiHostageGrabOne 识别失败分支）: 把已
#     检出的那只**平移到理想列**再重扫——不需要知道它是谁: 是中间则车已对齐;
#     是端部则车只偏 150mm, 三只照样全在画面里（出画需 ~175mm+）, 重识别必然
#     全出、后续对位自修。比固定距离开环准（按实测偏差定向）。路标坐标由
#     0x02 失败帧携带（handler 失败帧带最好成绩, 固件从快照读, 协议零改动）。
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
    """懒加载 YOLO ONNX 网络（加载较慢，只建一次并缓存）。**加载不出来返回 None。**

    ⚠️ 这里必须把「文件在、但读不了」也归到 None: `readNetFromONNX` 对**损坏的 /
    本机 OpenCV 不支持的 opset** 会抛异常, 而它一旦抛出去就一路穿到 handler 的兜底
    except —— 那条路**一帧都不回**, 单片机白等满 7.5s 超时 + 长鸣, 而你只能看到 Pi 侧
    一行 `[ERROR] 未捕获异常`, 与「相机打不开」长得完全不一样, 排查方向容易跑偏。
    （文件**不存在**那条本来就有兜底, 见 run_hostage_detect 的 net is None 分支。）
    """
    global _NET
    if _NET is None:
        from config import HOSTAGE_MODEL_PATH
        if not os.path.exists(HOSTAGE_MODEL_PATH):
            return None
        try:
            net = cv2.dnn.readNetFromONNX(HOSTAGE_MODEL_PATH)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as e:                      # noqa: BLE001
            print(f"[ERROR] 人质模型加载失败（{HOSTAGE_MODEL_PATH}）: "
                  f"{type(e).__name__}: {e} —— 按「模型缺失」处理, 会回一条失败帧让单片机解锁")
            return None
        _NET = net
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


def _detect(net, img, size, conf_thr, iou_thr, conf_lo=None):
    """单帧推理，返回目标列表（坐标在原始图像像素空间）；类名文件缺失返回 None。

    每项: {"shape", "shape_name", "cx", "cy", "conf", "box", "lo"}。
    未知类名（HOSTAGE_NAME2SHAPE 无映射）的目标丢弃（防御模型多训了别的类）。
    conf_lo（< conf_thr, 2026-10-26 加）给了时: 分数在 [conf_lo, conf_thr) 的框
    也返回、带 "lo": True —— 次级候选池用（主聚合只认高分框; NMS 阈值同步用低值,
    否则低分框进不了 NMS）。
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

    thr = (conf_lo if (conf_lo is not None and conf_lo < conf_thr) else conf_thr)
    keep = confs > thr
    boxes, confs, class_ids = boxes[keep], confs[keep], class_ids[keep]
    if len(boxes) == 0:
        return []

    cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = np.clip((cx - w / 2 - left) / r, 0, w0)
    y1 = np.clip((cy - h / 2 - top) / r, 0, h0)
    x2 = np.clip((cx + w / 2 - left) / r, 0, w0)
    y2 = np.clip((cy + h / 2 - top) / r, 0, h0)

    xywh = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)
    idx = cv2.dnn.NMSBoxes(xywh.tolist(), confs.tolist(), thr, iou_thr)
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
            "lo": bool(confs[i] < conf_thr),
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


def _hstg_rescue(found, lo_pool, frame_w, cx_center, mid_gap_px, near_tol_px):
    """跨帧并集仍不齐（恰剩 2 只）→ 次级池/几何补第三只。返回 (found, notes)。

    需求精确化（2026-10-26 用户）: **中间那只的坐标是硬需求**（对位基准; 缺端部时
    补出的坐标不被消费——平移走 one_lat_mm 查表, 糙点无所谓）。

    ⚠️⚠️ 缺谁由**间距 d** 判, 不是"两只中点偏移"（2026-10-26 用户问倒后改的,
       原中点设计是错的）: 缺中间 d=全排宽(~771px)、缺端部 d=单间距(~400px)——
       间距对车平移**不变**, 一刀切分得开; 而中点偏移在"端部没检出(车正)"与
       "端部出画(车偏)"两种成因下**方向相反**, 根本分不开。
    · 缺中间 → 坐标 = 两端部连线中点(透视下真中点略偏, 误差~16px≈5mm, 开环平移
      量级可接受); 次级池里任意形状的框若在中点 ±near_tol 内 → 用实测替换(更准)。
    · 缺端部 → 谁是中间 = 离 cx_center 近的那只(收敛后车偏 ≪ 半间距, 不会翻);
      缺位 = 沿连线等距外推(150mm 物理等距的图像近似), 出画则钳回边界内——
      出画也照补: 端部坐标不被消费, 且对位用实测中间会把车修回正位
      （对位本身就是修正, 这是人质侧不需要拉回的原因, 见文件头）。
    剩 ≤1 只无判据（单只不知在哪个槽）, 原样返回（老实失败）。
    纯函数（不碰推理/串口）, 离线可单独验。
    """
    from config import HOSTAGE_SHAPE_NAMES
    notes = []
    if len(found) != 2:
        return found, notes
    missing = [s for s in (1, 2, 3) if s not in found]
    if len(missing) != 1:
        return found, notes
    ms = missing[0]
    mname = HOSTAGE_SHAPE_NAMES.get(ms, str(ms))

    # a. 次级池直补: 低分框自带形状, 池里恰有缺的形状 → 实测坐标直接用
    if ms in lo_pool:
        det = dict(lo_pool[ms])
        det["inferred"] = "pool"
        found[ms] = det
        notes.append(f"次级池补上 {mname}({det['cx']},{det['cy']}) conf={det['conf']:.2f}")
        return found, notes

    # b. 几何补齐
    ks = sorted(found.values(), key=lambda d: d["cx"])
    x1, y1 = float(ks[0]["cx"]), float(ks[0]["cy"])
    x2, y2 = float(ks[1]["cx"]), float(ks[1]["cy"])
    d = x2 - x1
    if d < 1.0:
        return found, notes

    if d >= mid_gap_px:
        # 缺中间: 连线中点(池里中点近邻优先——任意形状的低分框, 坐标实测比插值准)
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        for det in lo_pool.values():
            if abs(det["cx"] - mx) <= near_tol_px:
                mx, my = float(det["cx"]), float(det["cy"])
                notes.append(f"中点取次级池实测框({det['shape_name']})")
                break
        found[ms] = {"shape": ms, "shape_name": mname,
                     "cx": mx, "cy": my, "conf": 0.0, "inferred": "geo"}
        notes.append(f"几何插值补上 中间{mname}({mx:.0f},{my:.0f}) ← 端部连线中点")
        return found, notes

    # 缺端部: 离 cx_center 近的是中间; 缺位沿连线等距外推(出画钳回——坐标不被消费)
    if abs(x1 - cx_center) <= abs(x2 - cx_center):
        xm, ym, side = x1, y1, -1          # x1 是中间 → 组合[中,右端] → 缺左端
    else:
        xm, ym, side = x2, y2, +1          # x2 是中间 → 组合[左端,中] → 缺右端
    nx = xm + side * d
    ny = ym + side * (y2 - y1)     # cy 沿连线斜率同向外推
    # 外推点若出画: 钳回边界内（排序仍正确: 钳位值仍是三只中最左/最右）
    oob = ""
    if nx < 0.0:
        nx, oob = 0.0, "(原外推 %.0f 已出画左, 钳回)" % nx
    elif nx >= frame_w:
        nx, oob = float(frame_w - 1), "(原外推 %.0f 已出画右, 钳回)" % nx
    found[ms] = {"shape": ms, "shape_name": mname,
                 "cx": nx, "cy": ny, "conf": 0.0, "inferred": "geo"}
    where = "左端" if side < 0 else "右端"
    notes.append(f"几何外推补上 {where}{mname}({nx:.0f},{ny:.0f}) ← 等距外推{oob}")
    return found, notes


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

    ⚠️ 坐标要钳到 u16（兄弟函数 _color.build_color_frame / _rect.build_rect_frame /
    _target.build_target_frame 都钳了, 只有这里原来没有）: 越界的 cx/cy 会让
    `struct.pack("<H")` 抛 struct.error → 穿到 handler 兜底 → **一帧都不回**,
    单片机白烧满超时。正常路径上坐标已经被 clip 到画面内, 这是兜底。
    """
    def _u16(v):
        return max(0, min(65535, int(round(float(v)))))

    data = struct.pack("<BBB", HOSTAGE_TAG, 1 if ok else 0, len(targets))
    for t in targets:
        data += struct.pack("<BHH", int(t["shape"]), _u16(t["cx"]), _u16(t["cy"]))
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

    **跨帧并集**（2026-10-26 改, 旧版是"单帧最佳"）: 按形状归并、每形状取最高
    conf 的检出 —— 单帧三只全中（检出率连乘）改成了"每只至少在一帧出现"。
    凑满 HOSTAGE_MIN_TARGETS 即成功; 成功目标按中心 x 升序（图像从左到右）。
    并集仍不齐且剩恰 2 只 → _hstg_rescue（次级池/几何补第三只, 见文件头）。
    ⚠️ 车静止前提不变（并集的坐标跨帧取, 车动会混机位——同色球投票的纪律）。

    Returns dict: {"success", "reason", "frame", "targets", "aborted"(仅打断时),
                   "frame_raw"(仅成功时: 未标注的原始帧)}；
        targets = [{"shape", "shape_name", "cx", "cy", "conf"}, ...]
        超时失败时 targets 为「最好成绩」（仅 Web 显示用，单片机端收失败帧）；
        失败/打断时 frame 尽力给「最后的原始帧」（统一记录接口, 失败也留档）。
    """
    from config import (HOSTAGE_CONF_THR, HOSTAGE_IOU_THR, HOSTAGE_INPUT_SIZE,
                        HOSTAGE_MAX_TARGETS, HOSTAGE_MIN_TARGETS,
                        HOSTAGE_UNC_EN, HOSTAGE_CONF_LO, HOSTAGE_CX_CENTER,
                        HOSTAGE_MID_GAP_PX, HOSTAGE_NEAR_TOL_PX)

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
        print(f"[INFO] 开始人质形状检测（跨帧并集, 凑满 {HOSTAGE_MIN_TARGETS} 个目标，"
              f"限时 {timeout:.0f}s）...")
        deadline = time.time() + timeout
        found = {}       # 跨帧并集: shape -> 最高 conf 的高分检出（2026-10-26 加）
        lo_pool = {}     # 次级候选池: shape -> 最高 conf 的低分检出（conf_lo~conf_thr）
        best = []        # 单帧最佳成绩（保留: 失败时给 Web 显示, 兼容旧语义）
        best_frame = None
        last_frame = None    # 最近读到的一帧 —— 失败/打断也要留档（统一记录接口,
                             # 见 handler._save_frame: 失败样本恰是排障最想看的）
        frame = None          # 循环一帧都没读到时收尾补救段引用它, 先占位

        while time.time() < deadline:
            # 可随时打断：每一轮先检查中断信号
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 人质识别任务被打断")
                return {"success": False, "aborted": True, "reason": "",
                        "frame": best_frame if best_frame is not None else last_frame,
                        "targets": best}

            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue
            last_frame = frame

            dets = _detect(net, frame, HOSTAGE_INPUT_SIZE,
                           HOSTAGE_CONF_THR, HOSTAGE_IOU_THR,
                           conf_lo=(HOSTAGE_CONF_LO if HOSTAGE_UNC_EN else None))
            if dets is None:
                time.sleep(0.05)
                continue
            hi_dets = [d for d in dets if not d.get("lo")]
            if len(hi_dets) > HOSTAGE_MAX_TARGETS:
                hi_dets = sorted(hi_dets, key=lambda d: -d["conf"])[:HOSTAGE_MAX_TARGETS]
            # 台面先验：单帧凑满 3 个但两个同形时按排除法重标（_apply_elimination）。
            # 必须在归并**之前**——{A,A,B} 帧重标后归并才拿得到第三形的坐标。
            hi_dets, elim_note = _apply_elimination(hi_dets)
            if elim_note:
                print(f"[INFO] 人质识别{elim_note}")
            for d in hi_dets:
                if d["shape"] not in found or d["conf"] > found[d["shape"]]["conf"]:
                    found[d["shape"]] = d
            for d in dets:                      # 次级池只收低分框（含重标可能换下的? 不——重标只在 hi 内做）
                if d.get("lo") and (d["shape"] not in lo_pool
                                    or d["conf"] > lo_pool[d["shape"]]["conf"]):
                    lo_pool[d["shape"]] = d
            if len(hi_dets) > len(best):
                best, best_frame = hi_dets, frame

            if len(found) >= HOSTAGE_MIN_TARGETS:
                break            # 并集凑齐, 下面统一走成功出口
            time.sleep(0.01)  # 避免空转刷满 CPU

        # ---- 收尾补救（2026-10-26）: 并集不齐且恰剩 2 只 → 次级池/几何补第三只 ----
        if (HOSTAGE_UNC_EN and len(found) == 2
                and len(found) < HOSTAGE_MIN_TARGETS):
            found, notes = _hstg_rescue(found, lo_pool,
                                        float(frame.shape[1]) if frame is not None else 1920.0,
                                        HOSTAGE_CX_CENTER, HOSTAGE_MID_GAP_PX,
                                        HOSTAGE_NEAR_TOL_PX)
            for n in notes:
                print(f"[INFO] 人质补救: {n}")

        targets = sorted(found.values(), key=lambda d: d["cx"])
        if len(targets) >= HOSTAGE_MIN_TARGETS:
            annotated = _annotate(best_frame if best_frame is not None else last_frame,
                                  targets)
            names = " ".join(d["shape_name"] for d in targets)
            print(f"[INFO] 人质识别成功（从左到右）: {names}")
            # 结果回传由 handler.py::_cmd_hostage 统一发送，任务层不碰串口
            return {"success": True, "reason": "",
                    "frame": annotated,
                    "frame_raw": (best_frame if best_frame is not None else last_frame),
                    "targets": targets}

        best = sorted(best, key=lambda d: d["cx"])
        annotated = _annotate(best_frame, best) if best_frame is not None else None
        reason = (f"人质识别超时 ({timeout:.0f}s)：最多检出 "
                  f"{len(best)}/{HOSTAGE_MIN_TARGETS} 个目标")
        print(f"[WARN] {reason}")
        # 一帧都没检到时 annotated 为 None → 退回最后的原始帧（别让 run 目录里
        # 这次任务一个图都不剩, 那种"失败了连现场都没有"最伤排障）
        return {"success": False, "reason": reason,
                "frame": annotated if annotated is not None else last_frame,
                "targets": best}
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
