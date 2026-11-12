# -*- coding: utf-8 -*-
"""判据②'（靶环内外径比 q）的回归测试。

跑法：`python tools/target_offline/test_ring_q.py`（stdlib unittest，不碰相机/串口/Pi）。

为什么要单独钉住这个判据：跨色尺寸一致性（判据①）**只是必要条件，不是充分条件**。
现场同色干扰块够多时，三个杂块凑巧一样大就能拼出一个合法的"尺寸一致组"，把真靶挤掉
——2026-09 现场实测的症状就是「蓝靶有时指向画面角落一个 r≈50 的杂块」。
q（= 内孔拟合半径 ÷ 该候选外接圆半径，靶盘设计值 0.55）只看**单个候选自己的形状**，
跟别的候选没关系，所以骗不过它。

⚠️ 下面的 `REAL_BAD_FRAME` 是**现场实测那一帧的真实候选表**（照抄 Pi 侧的诊断输出），
不是编出来的。`test_pick_consistent_*` 两条断言互为对照：同一条数据，旧判据选错、
新判据选对 —— 谁都别把这条改动悄悄地改回去。
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import detect  # noqa: E402
import synth   # noqa: E402


def _cand(color, r, q, cx=0.0, cy=0.0, holes=1):
    """造一个候选。q 直接给（省掉拟合），pick_consistent 只看 r/q/holes。"""
    return {"color": color, "r": float(r), "q": float(q), "cx": float(cx),
            "cy": float(cy), "holes": holes, "area": r * r * 3.0,
            "mask": None, "contour": None, "hole_fit": None}


# 现场一帧（1920×1080，三靶并排，蓝靶在画面中间）
#   真值：绿 r≈65@(705,532)、蓝 r≈67@(901,528)、红靶与木椅腿**合并**成 r≈270 的大块
#   （红靶靠 find_target 的「退回该色最大候选」+ 合并块内孔救回，不在这条测试的范围内）
# 旧判据输掉的原因：{红26, 绿65, 蓝50} 是唯一的"3 色"组，而排序键 (-颜色数, 尺寸比)
# 让"颜色多"压过了"尺寸真的一致" —— 那组的尺寸比是 65/26=2.53，烂得很，却因为多一色而胜出。
REAL_BAD_FRAME = {
    1: [_cand(1, 270, 0.13, 1204, 426, 5), _cand(1, 26, 0.14, 1517, 156)],
    2: [_cand(2, 105, 0.00, 1015, 413), _cand(2, 92, 0.00, 1614, 410),
        _cand(2, 65, 0.55, 705, 532)],
    3: [_cand(3, 503, 0.00, 880, 117, 7), _cand(3, 245, 0.04, 176, 239, 2),
        _cand(3, 67, 0.52, 901, 528), _cand(3, 55, 0.10, 1458, 98),
        _cand(3, 50, 0.00, 1232, 479)],
}


class TestRingQOnRealFrame(unittest.TestCase):
    def test_old_judge_picks_the_wrong_blue(self):
        """旧判据（只筛"有没有内孔"）在真实帧上会选错蓝靶 —— 这里把它钉成一个**已知**行为。

        它现在是绿的，但如果有人把 q 门槛拆了、又恰好让这条测试也变红，那是**好事**：
        说明这次改动确实动了那条老路径，值得重新看一遍。
        """
        picked = detect.pick_consistent(
            {k: list(v) for k, v in REAL_BAD_FRAME.items()}, ring_q=None)
        self.assertIn(3, picked)
        # 旧判据选中了画面右上角的杂块（蓝 = WRONG），而不是 (901,528) 的真蓝靶
        self.assertGreater(abs(picked[3]["cx"] - 901.0), 100.0)

    def test_ring_gate_fixes_the_blue(self):
        """新判据（+ 靶环 q 门槛）在**同一条数据**上选对绿靶与蓝靶。"""
        picked = detect.pick_consistent(
            {k: list(v) for k, v in REAL_BAD_FRAME.items()},
            ring_q=(detect.RING_Q_MIN, detect.RING_Q_MAX))
        self.assertEqual(sorted(picked), [2, 3], "该选出一绿一蓝两颗真靶")
        self.assertAlmostEqual(picked[2]["cx"], 705.0, delta=20.0)
        self.assertAlmostEqual(picked[3]["cx"], 901.0, delta=20.0)

    def test_red_is_left_to_the_fallback(self):
        """红靶在这一帧里 q=0.13（外轮廓被木椅腿撑大）-> **必须**被门槛挡在池子外。

        它没丢：`find_target` 会给没被共识覆盖的目标色退回该色最大候选，
        那颗 r=270 的合并块的内孔正是红靶的白心。两道关分工不同，别合成一道。
        """
        picked = detect.pick_consistent(
            {k: list(v) for k, v in REAL_BAD_FRAME.items()},
            ring_q=(detect.RING_Q_MIN, detect.RING_Q_MAX))
        self.assertNotIn(1, picked)


class TestRingQOnSyntheticRings(unittest.TestCase):
    """合成靶环上 q 必须落在设计值 0.55 附近 —— 门槛区间得罩住它。"""

    def test_real_target_q_is_the_design_ratio(self):
        img, _truth = synth.make_case()
        cands = detect.color_candidates(img, "red")
        self.assertTrue(cands, "标称用例该找到红靶候选")
        q = cands[0]["q"]
        self.assertAlmostEqual(q, detect.R_INNER_OVER_OUTER, delta=0.06)
        self.assertTrue(detect.RING_Q_MIN <= q <= detect.RING_Q_MAX,
                        "真靶环必须过得了门槛（q=%.2f）" % q)

    def test_same_color_ring_distractor_is_a_thinner_ring(self):
        """「同色圆环干扰」（蓝桶口：有孔、比靶大）的 q 明显偏高 —— 记录这个事实。

        它**不靠 q 拦**（q≈0.75 仍在区间内），靠的是判据①尺寸一致性（大 2.6 倍 > tol）。
        所以门槛上界**故意放得宽**（0.85）：别让它去跟判据①抢活，
        真把上界收到 0.70 附近会把"外环被轻微腐蚀、外接圆变小"的真靶也一起误杀。
        """
        trio = [("blue", (0.0, 0.0), 1200.0),
                ("green", (-500.0, 0.0), 1200.0),
                ("red", (500.0, 0.0), 1200.0)]
        img, _truth = synth.make_case(color_name="blue", targets=trio,
                                      distractor_rings=[("blue", 0.30, 0.62,
                                                         0.115, 0.030, 0.75)])
        cands = detect.color_candidates(img, "blue")
        qs = sorted(c["q"] for c in cands)
        self.assertTrue(any(q > 0.7 for q in qs), "该有 q≈0.75 的干扰环：%s" % qs)


class TestGateIsBypassedInsteadOfBlocking(unittest.TestCase):
    """门槛**不能**把单靶场面堵死。

    靶被遮挡时外轮廓会撑大、q 会掉出区间；若这时 `pick_consistent` 直接返回空、
    又没人兜底，单靶（另外两颗出画）就整颗漏检 —— 而"凑近打某一个靶"正是常规打法。

    契约是两层：`pick_consistent` 凑不齐 `min_colors` 个颜色时返回 `{}`（它只负责
    "跨色共识"，单色时本来就没有共识可谈），**兜底在 `find_target`** ——
    它会退回该色最大候选，交给下游的内孔/圆度判据把关（`m_triple`）。
    所以这里 assert 的是端到端结果，而不是 `pick_consistent` 自己。
    """

    def test_consensus_declines_when_only_one_color_is_visible(self):
        """只有一色 -> 没有"跨色"共识可言 -> 返回空（这是契约，不是 bug）。"""
        only_red = {1: [_cand(1, 65, 0.55, 700, 500)], 2: [], 3: []}
        self.assertEqual(
            detect.pick_consistent(only_red, ring_q=(0.35, 0.85)), {})

    def test_single_target_survives_even_a_gate_that_rejects_everything(self):
        """端到端：门槛把**一切**候选都判掉，单靶画面照样出结果（走 find_target 兜底）。"""
        img, truth = synth.make_case()
        for band in (None, (0.35, 0.85), (0.90, 0.95)):
            t = detect.m_triple(img, "red", ring_q=band)
            self.assertIsNotNone(t, "ring_q=%s 时不该漏检" % (band,))
            self.assertAlmostEqual(t["cx"], truth["cx"] * 1.0, delta=3.0)

    def test_band_is_the_documented_one(self):
        """区间与 Pi 侧 config.TARGET_RING_Q_MIN/MAX 是同一对值 —— 改一处要同步另一处。"""
        self.assertEqual((detect.RING_Q_MIN, detect.RING_Q_MAX), (0.35, 0.85))


class TestHoleFitHelpersAgree(unittest.TestCase):
    """`fit_hole_in`（复用轮廓树）必须与 `fit_hole_checked`（重跑 findContours）等价 ——
    它俩判据一致、只是省一遍 findContours，别在优化时把语义改歪了。"""

    def test_in_tree_variant_matches_the_recheck_variant(self):
        img, _truth = synth.make_case()
        import cv2
        mask = detect.color_mask(img, "red")
        nlab, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        found_any = False
        for lab in range(1, nlab):
            if stats[lab, cv2.CC_STAT_AREA] < 120.0:
                continue
            comp = (labels == lab).astype("uint8") * 255
            cnts, hier = cv2.findContours(comp, cv2.RETR_CCOMP,
                                          cv2.CHAIN_APPROX_SIMPLE)
            hier = hier[0]
            i0 = max((i for i in range(len(cnts)) if hier[i][3] == -1),
                     key=lambda i: cv2.contourArea(cnts[i]))
            a = detect.fit_hole_in(cnts, hier, i0)
            b = detect.fit_hole_checked(comp)
            if a is None or b is None:
                self.assertIsNone(a)
                self.assertIsNone(b)
                continue
            for x, y in zip(a, b):
                self.assertAlmostEqual(x, y, places=6)
            found_any = True
        self.assertTrue(found_any, "标称用例该能拟合出内孔")


if __name__ == "__main__":
    unittest.main(verbosity=2)
