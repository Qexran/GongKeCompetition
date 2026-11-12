#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib（「标定」Tab 后端）的离线测试 —— 全部在 tempfile 沙箱里跑，
绝不写真实的 Application/app_config.h 与 tools/webconsole/calib.json。

    python tools/webconsole/test_calib.py

核心不变量:
  · save 只写 calib.json, 一个字节不碰 app_config.h;
  · apply 全等时不写盘（不建 .bak、app_config.h 逐字节不变、mtime 不动）;
  · apply 有差异时只有目标行变化（其余行逐字节保留）, 且 .bak 先建;
  · 两边 base_version 的乐观锁（calib.json / app_config.h 各自一票）;
  · CALIB_PARAMS 与「调参」Tab 互斥（tuning.state 不含它们）但仍可写。
"""

import json
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tuning as T                                                   # noqa: E402
import calib as C                                                    # noqa: E402

# 覆盖四个标定段: 19 个 CALIB_PARAMS 宏 + 各段一个「非标定量」（必须留在「调参」Tab）
# + 一个无关段。值形制混着 float/uint, 验证回写时类型保真。
FIXTURE = """\
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ==================== 看路（主摄量黄色区, 车体纵向 + 可选横向修正） ==================== */
#define ROAD_DX_IDEAL_PX           1150.0f    /* 理想Δx [px] @(0~1920) // 黄区左边缘 */
#define ROAD_DX_IDEAL_REV_PX       730.0f     /* 反向看路理想Δx [px] @(0~1920) // 反向机位黄区右边缘(镜像系) */
#define ROAD_DY_IDEAL_PX           560.0f     /* 理想Δy [px] @(0~1080) // 黄区下边缘 */
#define ROAD_DY_IDEAL_REV_PX       610.0f     /* 反向看路理想Δy [px] @(0~1080) // 反向机位黄区下边缘 */
#define ROAD_DX_SCALE_MM_PER_PX    0.22f      /* 比例系数 [mm/px] @(0.01~5) // 说明 */
#define ROAD_DY_SCALE_MM_PER_PX    0.26f      /* 横向比例系数 [mm/px] @(0.01~5) // 说明 */
#define ROAD_ALIGN_SPEED           60         /* 对位速度 [rpm] @(0~65535) // 非标定量 */

/* ==================== 色球识别（排爆区, Pi HSV → 开环横移对位） ==================== */
#define BALL_CX_CENTER            926.0f      /* 理想停止位中球列 [px] @(0~1920) // 说明 */
#define BALL_CY_CENTER            408.0f      /* 理想停止位中球行 [px] @(0~1080) // 说明 */
#define BALL_PX_TO_MM_X           0.3f        /* 横向比例 [mm/px] @(0~) // 说明 */
#define BALL_PX_TO_MM_Y           0.3f        /* 纵向比例 [mm/px] @(0~) // 说明 */
#define BALL_YAW_SCALE            1.25f       /* 图像角倍率 @(0~) // 说明 */
#define BALL_YAW_TARGET_DEG       2.5f        /* 理想三球连线角 [°] @(~90) // 说明 */
#define BALL_ALIGN_SPEED          60          /* 对位速度 [rpm] @(0~65535) // 非标定量 */

/* ==================== 人质对位（人质区） ==================== */
#define HOSTAGE_CX_CENTER         760.0f      /* 理想基准人质列 [px] @(0~1920) // 说明 */
#define HOSTAGE_CY_CENTER         550.0f      /* 理想基准人质行 [px] @(0~1080) // 说明 */
#define HOSTAGE_PX_TO_MM_X        0.18f       /* 横向比例 [mm/px] @(0~) // 说明 */
#define HOSTAGE_PX_TO_MM_Y        0.24f       /* 纵向比例 [mm/px] @(0~) // 说明 */

/* ==================== 平移射靶（激光固定在底盘） ==================== */
#define TARGET_SHOOT_CX_AIM     975.0f    /* 瞄准列 [px] @(0~1920) // 说明 */
#define TARGET_SHOOT_CY_AIM     0.0f      /* 瞄准行 [px] @(0~1080) // 说明 */
#define TARGET_SHOOT_GAP_PX     289.0f    /* 每格像素 [px] @(1~1920) // 说明 */
#define TARGET_SHOOT_SPEED      60        /* 平移速度 [rpm] @(0~65535) // 非标定量 */

/* ==================== 无关段 ==================== */
#define OTHER_PARAM              7         /* 别的 [x] @(0~9) // 不该进标定 */

#endif /* __APP_CONFIG_H */
"""

# 一套与 fixture 现值完全一致的预设（apply 全等路径用）
SAME_VALUES = {
    "ROAD_DX_IDEAL_PX": "1150", "ROAD_DX_IDEAL_REV_PX": "730",
    "ROAD_DY_IDEAL_PX": "560", "ROAD_DY_IDEAL_REV_PX": "610",
    "ROAD_DX_SCALE_MM_PER_PX": "0.22", "ROAD_DY_SCALE_MM_PER_PX": "0.26",
    "BALL_CX_CENTER": "926", "BALL_CY_CENTER": "408",
    "BALL_PX_TO_MM_X": "0.3", "BALL_PX_TO_MM_Y": "0.3",
    "BALL_YAW_SCALE": "1.25", "BALL_YAW_TARGET_DEG": "2.5",
    "HOSTAGE_CX_CENTER": "760", "HOSTAGE_CY_CENTER": "550",
    "HOSTAGE_PX_TO_MM_X": "0.18", "HOSTAGE_PX_TO_MM_Y": "0.24",
    "TARGET_SHOOT_CX_AIM": "975", "TARGET_SHOOT_CY_AIM": "0",
    "TARGET_SHOOT_GAP_PX": "289",
}


def changed_values(**over):
    """SAME_VALUES 的副本, 关键参数换值。"""
    v = dict(SAME_VALUES)
    v.update(over)
    return v


class Sandbox(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_calib_"))
        self.cfg = self.tmp / "app_config.h"
        self.cfg.write_text(FIXTURE, encoding="utf-8", newline="")
        self.jsn = self.tmp / "calib.json"
        for mod, name, val in ((T, "APP_CONFIG", self.cfg),
                               (T, "PREFS_FILE", self.tmp / "tuning.json"),
                               (C, "CALIB_JSON", self.jsn)):
            p = mock.patch.object(mod, name, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    # ---- 帮手 ----

    def save_preset(self, name="主摄A", values=None, active=None, preset_id=None,
                    base_version="AUTO"):
        """走 run_calib_action('save') 存一套预设, 返回 (ok, data, error)。
        base_version 缺省取当前 calib.json 的 mtime（不存在 = None, 首次保存合法）。"""
        presets = [{"id": preset_id, "name": name, "note": "",
                    "values": values if values is not None else dict(SAME_VALUES)}]
        payload = {"presets": presets, "active": active, "next_id": 1,
                   "base_version": C.version() if base_version == "AUTO" else base_version}
        ok, data, error, _status = C.run_calib_action("save", payload)   # 2026-10-31 起回 4 元组
        return ok, data, error

    def reload(self):
        return self.cfg.read_text(encoding="utf-8")

    def drop_macro(self, name):
        """从沙箱 app_config.h 里删掉一个宏（模拟改名/删行脱钩）。"""
        lines = self.reload().split("\n")
        kept = [ln for ln in lines if not re.match(r"#define\s+%s\s" % name, ln)]
        self.assertEqual(len(kept), len(lines) - 1, "没删掉 %s 的行" % name)
        self.cfg.write_text("\n".join(kept), encoding="utf-8", newline="")


class TestState(Sandbox):
    def test_groups_served_with_metadata_and_current_values(self):
        st = C.state()
        self.assertNotIn("error", st)
        self.assertEqual([g["title"] for g in st["groups"]],
                         ["看路", "色球", "人质", "射靶粗调"])
        by = {it["name"]: it for g in st["groups"] for it in g["items"]}
        self.assertEqual(sorted(by), sorted(T.CALIB_PARAMS))
        self.assertEqual(by["BALL_CX_CENTER"]["display_value"], 926)
        self.assertEqual(by["BALL_CX_CENTER"]["display"], "理想停止位中球列")
        self.assertEqual(by["TARGET_SHOOT_GAP_PX"]["unit"], "px")
        self.assertEqual(st["presets"], [])
        self.assertIsNone(st["version"])            # calib.json 还不存在

    def test_missing_macro_reported_not_silent(self):
        self.drop_macro("BALL_YAW_SCALE")
        st = C.state()
        self.assertEqual(st["missing"], ["BALL_YAW_SCALE"])
        self.assertIn("BALL_YAW_SCALE", st["error"])
        names = [it["name"] for g in st["groups"] for it in g["items"]]
        self.assertNotIn("BALL_YAW_SCALE", names)   # 缺的不硬塞
        self.assertIn("BALL_CX_CENTER", names)      # 其余照常

    def test_tuning_state_excludes_calib_params(self):
        """17 个标定参数从「调参」Tab 消失; 非标定量与无关段仍在。"""
        st = T.state()
        names = {it["name"] for g in st["groups"] for it in g["items"]}
        for n in T.CALIB_PARAMS:
            self.assertNotIn(n, names, "%s 不该出现在调参 Tab" % n)
        # 留下的: 非标定量 + 无关段（本段没有被剔空）
        self.assertIn("ROAD_ALIGN_SPEED", names)
        self.assertIn("BALL_ALIGN_SPEED", names)
        self.assertIn("TARGET_SHOOT_SPEED", names)
        self.assertIn("OTHER_PARAM", names)
        titles = [g["title"] for g in st["groups"]]
        self.assertIn("无关段", titles)


class TestSave(Sandbox):
    def test_save_writes_json_not_app_config(self):
        before = self.reload()
        ok, data, err = self.save_preset()
        self.assertTrue(ok, err)
        self.assertIsNotNone(data["version"])
        self.assertEqual(data["next_id"], 2)
        self.assertEqual(data["presets"][0]["id"], 1)      # 服务端发号
        self.assertEqual(self.reload(), before)            # app_config.h 一字未动
        disk = json.loads(self.jsn.read_text(encoding="utf-8"))
        self.assertEqual(disk["presets"][0]["name"], "主摄A")

    def test_save_is_zero_diff_when_unchanged(self):
        ok, data, _ = self.save_preset()
        ver = data["version"]
        ok2, data2, _, _st = C.run_calib_action("save", {
            "presets": data["presets"], "active": None,
            "next_id": data["next_id"], "base_version": ver})
        self.assertTrue(ok2)
        self.assertFalse(data2["changed"])
        self.assertEqual(data2["version"], ver)            # 没写盘, mtime 没动

    def test_stale_base_version_conflicts(self):
        ok, data, err, _st = C.run_calib_action("save", {
            "presets": [], "active": None, "next_id": 1,
            "base_version": 12345.0})
        self.assertFalse(ok)
        self.assertEqual(data["error"], "conflict")

    def test_incomplete_preset_rejected(self):
        vals = dict(SAME_VALUES)
        del vals["ROAD_DY_IDEAL_PX"]
        ok, data, err = self.save_preset(values=vals)
        self.assertFalse(ok)
        self.assertIn("ROAD_DY_IDEAL_PX", err)
        self.assertFalse(self.jsn.exists())

    def test_invalid_value_rejected_with_name(self):
        ok, data, err = self.save_preset(values=changed_values(BALL_CX_CENTER="abc"))
        self.assertFalse(ok)
        self.assertIn("BALL_CX_CENTER", err)

    def test_unknown_macro_keys_dropped(self):
        vals = changed_values(FAKE_MACRO="1")
        ok, data, err = self.save_preset(values=vals)
        self.assertTrue(ok, err)
        self.assertNotIn("FAKE_MACRO", data["presets"][0]["values"])
        self.assertEqual(sorted(data["presets"][0]["values"]), sorted(T.CALIB_PARAMS))

    def test_name_sanitize_and_caps(self):
        ok, data, err = self.save_preset(name="主摄  A\n\r满屏都是字" * 10)
        self.assertTrue(ok, err)
        name = data["presets"][0]["name"]
        self.assertEqual(name, name[:C.MAX_NAME_LEN])
        self.assertNotIn("\n", name)
        ok, _, err = self.save_preset(name="   ")
        self.assertFalse(ok)
        self.assertIn("预设名", err)

    def test_deleting_active_preset_rejected(self):
        ok, data, _ = self.save_preset()
        pid = data["presets"][0]["id"]
        ok2, data2, _, _st = C.run_calib_action("save", {
            "presets": [{"id": pid, "name": "x", "note": "",
                         "values": dict(SAME_VALUES)}],
            "active": pid, "next_id": 99,
            "base_version": data["version"]})               # active 指向唯一预设
        self.assertTrue(ok2)
        ok3, data3, err3, _st = C.run_calib_action("save", {     # 删掉它
            "presets": [], "active": pid, "next_id": 99,
            "base_version": data2["version"]})
        self.assertFalse(ok3)
        self.assertIn("不能删", err3)

    def test_next_id_never_reuses(self):
        ok, data, _ = self.save_preset(preset_id=7)
        self.assertEqual(data["next_id"], 8)                # 已有 id 7 → 8
        ok2, data2, _ = self.save_preset()                  # 新预设发号 8, next→9
        self.assertEqual(data2["presets"][0]["id"], 8)
        self.assertEqual(data2["next_id"], 9)

    def test_preset_count_capped(self):
        presets = [{"id": i + 1, "name": "p%d" % i, "note": "",
                    "values": dict(SAME_VALUES)} for i in range(C.MAX_PRESETS + 1)]
        ok, _, err, _st = C.run_calib_action("save", {
            "presets": presets, "active": None, "next_id": 99})
        self.assertFalse(ok)
        self.assertIn("最多", err)


class TestApply(Sandbox):
    def test_apply_equal_values_writes_nothing(self):
        ok, data, _ = self.save_preset()
        before, mtime = self.reload(), self.cfg.stat().st_mtime
        ok2, data2, err2, _st = C.run_calib_action("apply", {
            "id": data["presets"][0]["id"],
            "base_version": data["version"],
            "base_cfg": T.version()})
        self.assertTrue(ok2, err2)
        self.assertFalse(data2["changed_cfg"])
        self.assertEqual(self.reload(), before)             # 逐字节不变
        self.assertEqual(self.cfg.stat().st_mtime, mtime)   # 没写盘
        self.assertFalse((self.tmp / "app_config.h.bak").exists())
        disk = json.loads(self.jsn.read_text(encoding="utf-8"))
        self.assertEqual(disk["active"], data["presets"][0]["id"])   # active 记上了

    def test_apply_changes_only_target_lines(self):
        ok, data, _ = self.save_preset(values=changed_values(
            BALL_CX_CENTER="900", TARGET_SHOOT_GAP_PX="300"))
        before_lines = self.reload().split("\n")
        ok2, data2, err2, _st = C.run_calib_action("apply", {
            "id": data["presets"][0]["id"],
            "base_version": data["version"],
            "base_cfg": T.version()})
        self.assertTrue(ok2, err2)
        self.assertTrue(data2["changed_cfg"])
        self.assertIn("Keil", data2["note"])
        after_lines = self.reload().split("\n")
        self.assertEqual(len(after_lines), len(before_lines))
        diff = [(a, b) for a, b in zip(before_lines, after_lines) if a != b]
        self.assertEqual(len(diff), 2)                      # 只动了两个宏的行
        joined = "\n".join([diff[0][1], diff[1][1]])
        self.assertIn("#define BALL_CX_CENTER", joined)
        self.assertIn("900.0f", joined)                     # float 类型保真
        self.assertIn("#define TARGET_SHOOT_GAP_PX", joined)
        self.assertIn("300.0f", joined)
        # 尾注释原样保留（行重建不丢 DSL）
        self.assertIn("理想停止位中球列", joined)
        self.assertTrue((self.tmp / "app_config.h.bak").exists())

    def test_apply_stale_cfg_conflicts(self):
        ok, data, _ = self.save_preset(values=changed_values(BALL_CX_CENTER="900"))
        ok2, data2, err2, _st = C.run_calib_action("apply", {
            "id": data["presets"][0]["id"],
            "base_version": data["version"],
            "base_cfg": 12345.0})                           # app_config.h 旧戳
        self.assertFalse(ok2)
        self.assertEqual(data2["error"], "conflict")
        self.assertIn("926", self.reload())                 # 一字未写

    def test_apply_stale_json_conflicts(self):
        ok, data, _ = self.save_preset()
        ok2, data2, err2, _st = C.run_calib_action("apply", {
            "id": data["presets"][0]["id"],
            "base_version": 12345.0,
            "base_cfg": T.version()})
        self.assertFalse(ok2)
        self.assertEqual(data2["error"], "conflict")

    def test_apply_unknown_id(self):
        ok, data, err, _st = C.run_calib_action("apply", {"id": 999})
        self.assertFalse(ok)
        self.assertIn("没有这套预设", err)

    def test_apply_with_missing_macro_refused(self):
        ok, data, _ = self.save_preset()
        self.drop_macro("BALL_YAW_SCALE")
        ok2, _, err2, _st = C.run_calib_action("apply", {
            "id": data["presets"][0]["id"],
            "base_version": data["version"],
            "base_cfg": T.version()})
        self.assertFalse(ok2)
        self.assertIn("BALL_YAW_SCALE", err2)


class TestTuningStillWritesCalibParams(Sandbox):
    def test_write_config_accepts_calib_macro(self):
        """「调参」Tab 不再**显示**这批参数, 但 write_config 照写（apply 复用它）。"""
        ok, resp, st = T.write_config({"BALL_CX_CENTER": "900"}, [], T.version())
        self.assertTrue(ok, resp)
        self.assertIn("900.0f", self.reload())
        # 类型/风格保真: 尾注释还在
        for ln in self.reload().split("\n"):
            if ln.startswith("#define BALL_CX_CENTER"):
                self.assertIn("理想停止位中球列", ln)
                break
        else:
            self.fail("行没了")


class TestRealSourceContract(unittest.TestCase):
    """对真实仓库文件只读校验: CALIB_PARAMS 里的宏必须真的存在（防改名后脱钩）。"""

    def test_every_calib_macro_exists_in_real_app_config(self):
        text = T.APP_CONFIG.read_text(encoding="utf-8")
        for n in T.CALIB_PARAMS:
            self.assertRegex(text, r"#define\s+%s\b" % n,
                             "app_config.h 里找不到 %s —— CALIB_GROUPS 与源码脱钩了" % n)

    def test_shipped_calib_json_is_complete_and_valid(self):
        """随仓库发货的 calib.json（现役主摄初始预设, 2026-10-30 导入）必须始终
        对得上真实 app_config.h: 每套预设都是完整快照、值合法、active/next_id 自洽。
        手改坏这份文件时这里先红, 而不是标定 Tab 上静默丢预设。"""
        self.assertTrue(C.CALIB_JSON.exists(),
                        "tools/webconsole/calib.json 不见了 —— 入库文件, 现役标定的家底")
        data = C._load()
        self.assertTrue(data["presets"], "至少要有一套预设（现役主摄初始导入）")
        ids = [int(p["id"]) for p in data["presets"]]
        self.assertEqual(len(ids), len(set(ids)), "预设 id 重复")
        self.assertGreaterEqual(data["next_id"], max(ids) + 1, "next_id 比最大 id 还小")
        if data["active"] is not None:
            self.assertIn(data["active"], ids, "active 指向不存在的预设")
        by_name, missing = C._calib_items()
        self.assertFalse(missing)
        for p in data["presets"]:
            for n in T.CALIB_PARAMS:
                self.assertIn(n, p.get("values") or {},
                              "预设 %r 缺 %s（预设必须是完整快照）" % (p.get("name"), n))
                T.from_display(by_name[n], p["values"][n])   # 非法值直接抛


if __name__ == "__main__":
    unittest.main(verbosity=2)
