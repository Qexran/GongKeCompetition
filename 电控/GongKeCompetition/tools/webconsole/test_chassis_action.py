#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chassis_action.py 的测试（全程沙箱: 不碰真实 app_chassis_action.c/.h, 不碰真实回收站）。

    python tools/webconsole/test_chassis_action.py

核心不变量（与 test_arm_action.py 同款, 本项目对「回写源码」的统一要求）:
  · 无操作保存 = 两个文件都**字节级不变, 且 mtime 不变**（连 .bak 都不该建）
  · 未编辑的条目按**原始物理行**回吐 → 手写的排版不该被规范化
  · 只改该改的那几行
"""

import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chassis_action as CA                                     # noqa: E402

FIXTURE_H = '''#ifndef __APP_CHASSIS_ACTION_H
#define __APP_CHASSIS_ACTION_H

#include <stdint.h>

/* ==== 生成块: 开始 ==== */
/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次
 * 保存后指向另一个动作组（同 ARM_ACTION_NEXT_ID 的坑）。 */
#define CH_ACT_NEXT_ID    3u

/* 动作组 ID: 路线表的 STEP_CHASSIS_ACTION_PLAY_WAIT 用它引用; 控制台「动作组」
 * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。
 * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emChActIdTdf)I(0)。
 *
 * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的。别改成 #define ——
 *    那样「动作组」参数就没有下拉了。 */
typedef enum {
    CH_ACT_1 = 1,   /* 前进并右转 */
} emChActIdTdf;

/* 预设张数（= g_astChActGroup[] 的长度, 控制台同步维护） */
#define CH_ACT_GROUP_NUM        1u
/* ==== 生成块: 结束 ==== */

#endif
'''

FIXTURE_C = '''#include "app_chassis_action.h"

/* ==== 数据区: 开始 ==== */

/* ===== 预设: 前进并右转 ===== */
/* 备注: 样例 */
const stChActItemTdf g_astChActItems1[] = {
    CH_ACT_MOVE     (0u,    0.0f, 500.0f, 150, 120, 1u),  /* 前进 500mm */
    CH_ACT_TURN     (1600u, 2000u, -90, 60, 0u, 80u),     /* 尾段右转(相对), 自带加速度 */
    CH_ACT_ARM      (400u, ARM_ACT_11),                   /* 半路收臂 */
    CH_ACT_ARM_WAIT (2200u, 0u),                          /* 等臂收完 */
};

const stChActGroupTdf g_astChActGroup[CH_ACT_GROUP_NUM] = {
    { 1u, "前进并右转", g_astChActItems1, 4u },
};

/* ==== 数据区: 结束 ==== */

void vChassisActionTask(void *argument) { (void)argument; }
'''


class Sandbox(unittest.TestCase):
    """临时文件 + mock.patch.object 路径常量（照 test_arm_action.py::Sandbox）。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_chact_"))
        self.c = self.tmp / "app_chassis_action.c"
        self.h = self.tmp / "app_chassis_action.h"
        self.trash = self.tmp / "chact_trash.json"
        self.c.write_text(FIXTURE_C, encoding="utf-8", newline="")
        self.h.write_text(FIXTURE_H, encoding="utf-8", newline="")
        for name, val in (("ACTION_C", self.c), ("ACTION_H", self.h),
                          ("TRASH_JSON", self.trash)):
            p = mock.patch.object(CA, name, val)
            p.start()
            self.addCleanup(p.stop)
        # 臂 id 校验不打真实仓库: 沙箱里认 1 与 11 存在
        p = mock.patch.object(CA, "_arm_ids", lambda: {1, 11})
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    # ---- 帮手 ----

    def version(self):
        return CA.parse_chact()["version"]

    def save(self, groups, base_version=None):
        return CA.save({"groups": groups},
                       self.version() if base_version is None else base_version)


class TestParse(Sandbox):

    def test_parse_basic(self):
        st = CA.parse_chact()
        self.assertEqual(st["next_id"], 3)
        self.assertEqual(len(st["groups"]), 1)
        g = st["groups"][0]
        self.assertEqual(g["id"], 1)
        self.assertEqual(g["title"], "前进并右转")
        self.assertEqual(g["note"], "样例")
        self.assertEqual(g["count"], 4)
        kinds = [it["kind"] for it in g["items"]]
        self.assertEqual(kinds, ["move", "turn", "arm", "arm_wait"])

        m = g["items"][0]
        self.assertEqual((m["start"], m["wx"], m["wy"], m["speed"], m["accel"]),
                         (0, 0.0, 500.0, 150, 120))
        self.assertTrue(m["hold"])
        t = g["items"][1]
        self.assertEqual((t["start"], t["dur"], t["delta"], t["speed"]),
                         (1600, 2000, -90, 60))
        a = g["items"][2]
        self.assertEqual((a["start"], a["arm_id"]), (400, 11))
        w = g["items"][3]
        self.assertEqual((w["start"], w["dur"]), (2200, 0))

    def test_parse_disabled(self):
        text = self.c.read_text(encoding="utf-8")
        text = text.replace("    CH_ACT_ARM      (400u",
                            "//    CH_ACT_ARM      (400u")
        self.c.write_text(text, encoding="utf-8", newline="")
        g = CA.parse_chact()["groups"][0]
        self.assertTrue(g["items"][2]["disabled"])
        self.assertFalse(g["items"][0]["disabled"])
        # 聚合计数是文件里的原文（固件侧的真实计数只在保存时重算）
        self.assertEqual(g["count"], 4)

    def test_turn_abs_field(self):
        # 6 参行: abs/accel 读出（dur 槽已废弃, 转到位为止）
        st = CA.parse_chact()
        t = st["groups"][0]["items"][1]
        self.assertEqual(t["abs"], 0)
        self.assertEqual(t["accel"], 80)

        # 绝对档: delta=0 合法（咬回开机朝向）, 且按当前字段重渲染成 6 参（dur 恒 0）
        ok, resp, _ = self.save([
            {"title": "绝对档", "note": "", "items": [
                {"kind": "turn", "start": 0, "dur": 2000, "delta": 0,
                 "speed": 50, "abs": 1, "tag": ""}]}])
        self.assertTrue(ok, resp)
        line = self.c.read_text(encoding="utf-8")
        self.assertIn("CH_ACT_TURN     (0u, 0u, 0, 50, 1u, 0u),", line)

        # accel 越界拒绝
        ok, resp, _ = self.save([
            {"title": "加速度越界", "note": "", "items": [
                {"kind": "turn", "start": 0, "dur": 2000, "delta": 90,
                 "speed": 50, "abs": 0, "accel": 300, "tag": ""}]}])
        self.assertFalse(ok)
        self.assertIn("加速度", resp["message"])

    def test_pivot_roundtrip_and_validation(self):
        """支点转弯段（2026-10 加）: 渲染 CH_ACT_PIVOT 4 参; 角度纪律与 turn 同一套。"""
        ok, resp, _ = self.save([
            {"title": "支点段", "note": "", "items": [
                {"kind": "pivot", "start": 500, "delta": -90, "speed": 130,
                 "abs": 0, "tag": ""}]}])
        self.assertTrue(ok, resp)
        line = self.c.read_text(encoding="utf-8")
        self.assertIn("CH_ACT_PIVOT     (500u, -90, 130, 0u),", line)

        # 解析回来字段对得上
        g = CA.parse_chact()["groups"][0]
        self.assertEqual(g["items"][0]["kind"], "pivot")
        self.assertEqual((g["items"][0]["delta"], g["items"][0]["speed"]), (-90, 130))

        # 角度纪律与 turn 同一套: 非 ±90 倍数拒绝
        ok, resp, _ = self.save([
            {"title": "支点坏角", "note": "", "items": [
                {"kind": "pivot", "start": 0, "delta": 45, "speed": 130,
                 "abs": 0, "tag": ""}]}])
        self.assertFalse(ok)
        self.assertIn("±90", resp["message"])

    def test_legacy_turn_row_migrates_on_save(self):
        """5 参旧行（没有加速度槽）: 解析缺省 accel=0; 下一次保存**自动迁移**成 6 参。

        不能用「拒绝保存」逼用户手改 —— 没改东西保存失败只会让人困惑; 而固件宏
        已经是 6 参, 旧行原样回吐是编译错误, 所以迁移是唯一安全的出口。
        ⚠️ 迁移后 dur 槽写成 0（段上限已取消, 固件忽略该槽）。"""
        text = self.c.read_text(encoding="utf-8")
        text = text.replace("CH_ACT_TURN     (1600u, 2000u, -90, 60, 0u, 80u)",
                            "CH_ACT_TURN     (1600u, 2000u, -90, 60, 0u)")
        self.c.write_text(text, encoding="utf-8", newline="")
        g = CA.parse_chact()["groups"][0]
        self.assertEqual(g["items"][1]["abs"], 0)        # 解析照常, 缺省=相对
        self.assertEqual(g["items"][1]["accel"], 0)      # 缺省=自动
        ok, resp, _ = self.save(st_groups(g))
        self.assertTrue(ok, resp)
        new = self.c.read_text(encoding="utf-8")
        self.assertIn("CH_ACT_TURN     (1600u, 0u, -90, 60, 0u, 0u),  /* 尾段右转(相对), 自带加速度 */", new)

    def test_estimate_matches_firmware_shape(self):
        # 500mm@150rpm/120 加速度 ≈ 1.79s（与固件 fChassisMoveTimeEst 同公式）
        ms = CA.estimate_ms(500, 120, 150, time_scale=1.0, pulse_per_mm=10.3)
        self.assertTrue(1700 < ms < 1900, ms)
        # 速度 0 / 距离 ≤2mm 的哨兵与固件同口径
        self.assertEqual(CA.estimate_ms(0, 120, 150), 200.0)
        self.assertEqual(CA.estimate_ms(500, 120, 0), 200.0)

    def test_snap_grid(self):
        self.assertEqual(CA.snap_ms(37, 20), 40)
        self.assertEqual(CA.snap_ms(30, 20), 40)
        self.assertEqual(CA.snap_ms(-5, 20), 0)

    def test_config_consts_parses_tune_defaults(self):
        """调参卡的离线默认值来自 app_config.h 的 CH_ACT_* 宏 —— 解析不到会静默
        退成内置保守值。只钉「键存在 + 量纲合理」, 不钉具体数值: 现场调完会写回
        固化, 那是合法的值变化, 不该拆测试。"""
        c = CA.config_consts()
        for key, hi in (("head_kp", 20), ("head_ki", 5), ("head_kd", 20),
                        ("head_ki_max", 50), ("head_db_deg", 10),
                        ("w_max", 200), ("wheel_max", 1000)):
            self.assertIn(key, c)
            self.assertIsInstance(c[key], float)
            self.assertTrue(0 <= c[key] <= hi, (key, c[key]))


def st_groups(g):
    """单个解析出的组 → save 的 groups 载荷（深拷, 防止测试间串改）。"""
    import copy
    return [copy.deepcopy(g)]


class TestRoundtrip(Sandbox):

    def test_noop_save_zero_diff(self):
        st = CA.parse_chact()
        ok, resp, _ = self.save([dict(g) for g in st["groups"]])
        self.assertTrue(ok, resp)
        self.assertFalse(resp["changed"])       # 连 .bak 都不该建
        self.assertEqual(self.c.read_text(encoding="utf-8"), FIXTURE_C)
        self.assertEqual(self.h.read_text(encoding="utf-8"), FIXTURE_H)

    def test_edit_single_item_only_that_line_changes(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][0]["wy"] = 650
        g["items"][0]["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertTrue(ok, resp)
        new = self.c.read_text(encoding="utf-8")
        # 重渲染的行: 参数变了, 尾注释（tag）保留
        self.assertIn("CH_ACT_MOVE     (0u, 0.0f, 650.0f, 150, 120, 1u),  /* 前进 500mm */", new)
        # 其余行原样: 转弯/臂/等臂那三行不动
        self.assertIn("CH_ACT_TURN     (1600u, 2000u, -90, 60, 0u, 80u),", new)
        self.assertIn("CH_ACT_ARM      (400u, ARM_ACT_11),", new)
        self.assertIn("CH_ACT_ARM_WAIT (2200u, 0u),", new)

    def test_disable_toggle_comments_line(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][1]["disabled"] = True
        g["items"][1]["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertTrue(ok, resp)
        new = self.c.read_text(encoding="utf-8")
        self.assertIn("//    CH_ACT_TURN", new)
        # 聚合计数 = 活行数（禁用的行是注释, 不进数组）
        self.assertRegex(new, r'\{ 1u, "前进并右转", g_astChActItems1, 3u \}')


class TestLifecycle(Sandbox):

    def test_add_group_gets_new_id_and_table(self):
        st = CA.parse_chact()
        new_g = {"title": "回中", "note": "", "items": [
            {"kind": "move", "start": 0, "wx": -100, "wy": 0, "speed": 100,
             "accel": 100, "hold": True, "tag": "横移"},
        ]}
        ok, resp, _ = self.save(st["groups"] + [new_g])
        self.assertTrue(ok, resp)
        st2 = CA.parse_chact()
        self.assertEqual(len(st2["groups"]), 2)
        self.assertEqual(st2["groups"][1]["id"], 3)     # next_id=3 发号
        self.assertEqual(st2["next_id"], 4)
        h = self.h.read_text(encoding="utf-8")
        self.assertIn("CH_ACT_3 = 3,   /* 回中 */", h)
        self.assertIn("#define CH_ACT_GROUP_NUM        2u", h)
        c = self.c.read_text(encoding="utf-8")
        self.assertIn("g_astChActItems3[]", c)

    def test_id_never_reused_after_delete(self):
        st = CA.parse_chact()
        ok, resp, _ = self.save(st["groups"] + [
            {"title": "第二组", "note": "", "items": [
                {"kind": "turn", "start": 0, "dur": 2000, "delta": 90,
                 "speed": 50, "tag": ""}]}])
        self.assertTrue(ok, resp)
        # 删掉 id=1 之后新建: 新号必须 >3（不复用 1）
        ok, resp, _ = CA.delete_group(1, self.version())
        self.assertTrue(ok, resp)
        st = CA.parse_chact()
        self.assertEqual([g["id"] for g in st["groups"]], [3])
        ok, resp, _ = self.save(st["groups"] + [
            {"title": "第三组", "note": "", "items": [
                {"kind": "arm_wait", "start": 0, "dur": 0, "tag": ""}]}])
        self.assertTrue(ok, resp)
        st = CA.parse_chact()
        self.assertEqual([g["id"] for g in st["groups"]], [3, 4])

    def test_delete_last_group_rejected(self):
        ok, resp, _ = CA.delete_group(1, self.version())
        self.assertFalse(ok)
        self.assertEqual(resp["error"], "last")

    def test_delete_moves_to_trash_and_restore(self):
        # 先加第二组（删最后一组会被 'last' 拦下, 那是另一个测试）
        st = CA.parse_chact()
        ok, resp, _ = self.save(st["groups"] + [
            {"title": "第二组", "note": "", "items": [
                {"kind": "turn", "start": 0, "dur": 2000, "delta": 90,
                 "speed": 50, "tag": ""}]}])
        self.assertTrue(ok, resp)

        ok, resp, _ = CA.delete_group(1, self.version())
        self.assertTrue(ok, resp)
        trash = CA.load_trash()
        self.assertEqual(len(trash), 1)
        self.assertEqual(trash[0]["title"], "前进并右转")
        self.assertEqual(len(trash[0]["items"]), 4)
        ok, resp, _ = CA.restore_group(trash[0]["trash_id"], self.version())
        self.assertTrue(ok, resp)
        st = CA.parse_chact()
        self.assertEqual([g["title"] for g in st["groups"]],
                         ["第二组", "前进并右转恢复"])
        self.assertEqual(st["groups"][1]["id"], 4)     # 新号, 不复用旧 1
        self.assertEqual(CA.load_trash(), [])


class TestValidation(Sandbox):

    def test_turn_not_multiple_of_90_rejected(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][1]["delta"] = -70
        g["items"][1]["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertFalse(ok)
        self.assertIn("±90", resp["message"])

    def test_move_without_distance_rejected(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][0].update({"wx": 0, "wy": 0, "edited": True})
        ok, resp, _ = self.save(st["groups"])
        self.assertFalse(ok)
        self.assertIn("位移", resp["message"])

    def test_move_bad_speed_rejected(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][0].update({"speed": 0, "edited": True})
        ok, resp, _ = self.save(st["groups"])
        self.assertFalse(ok)
        self.assertIn("速度", resp["message"])

    def test_arm_event_unknown_id_rejected(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][2]["arm_id"] = 999
        g["items"][2]["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertFalse(ok)
        self.assertIn("ARM_ACT", resp["message"])

    def test_too_many_items_rejected(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        for _ in range(20):
            g["items"].append({"kind": "arm_wait", "start": 0, "dur": 0, "tag": ""})
        ok, resp, _ = self.save(st["groups"])
        self.assertFalse(ok)
        self.assertIn("上限", resp["message"])

    def test_start_snapped_to_grid_on_save(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        g["items"][0]["start"] = 37       # 不在 20ms 网格上
        g["items"][0]["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertTrue(ok, resp)
        new = self.c.read_text(encoding="utf-8")
        self.assertIn("CH_ACT_MOVE     (40u,", new)

    def test_conflict_detected(self):
        ok, resp, status = self.save(CA.parse_chact()["groups"],
                                     base_version={"c": 123.0, "h": 456.0})
        self.assertFalse(ok)
        self.assertEqual(status, 409)
        self.assertEqual(resp["error"], "conflict")


class TestArmItemRender(Sandbox):
    """2026-10-31 修的两类洞的钉子:
    ① 编辑过的「臂动作」条目曾渲染成 ARM_ACT_11u —— 那是**另一个标识符**（非法 C,
       Keil 报 undefined identifier）, 且自家 RE_ARM_ID 读不回来（重载 arm_id 丢成 0）。
    ② 手写的 /* */ 注释行曾被数进 usCount —— 固件按 count 遍历会越界读表。"""

    def test_edited_arm_item_roundtrip(self):
        st = CA.parse_chact()
        g = st["groups"][0]
        arm = next(it for it in g["items"] if it.get("kind") == "arm")
        arm["start"] = 800
        arm["edited"] = True
        ok, resp, _ = self.save(st["groups"])
        self.assertTrue(ok, resp)
        new = self.c.read_text(encoding="utf-8")
        # ARM_ACT_11 是枚举成员, 不带 u 后缀（map.c 全部引用皆如此）
        self.assertIn("CH_ACT_ARM      (800u, ARM_ACT_11),", new)
        self.assertNotIn("ARM_ACT_11u", new)
        # 回读: arm_id 不丢
        st2 = CA.parse_chact()
        arm2 = next(it for it in st2["groups"][0]["items"] if it.get("kind") == "arm")
        self.assertEqual(arm2.get("arm_id"), 11)

    def test_count_ignores_hand_comment_lines(self):
        st = CA.parse_chact()
        ok, resp, _ = self.save(st["groups"])
        self.assertTrue(ok, resp)
        # 手动往表体里插一行普通块注释（解析器会收成 raw 条目原样回吐）
        text = self.c.read_text(encoding="utf-8")
        text = text.replace("    CH_ACT_ARM      (400u, ARM_ACT_11),",
                            "    /* 调试用的手写注释 */\n    CH_ACT_ARM      (400u, ARM_ACT_11),")
        self.c.write_text(text, encoding="utf-8", newline="")
        st2 = CA.parse_chact()
        ok, resp, _ = self.save(st2["groups"])
        self.assertTrue(ok, resp)
        # 计数仍是 4（注释行不是初始化项）—— 数成 5 固件就越界了
        self.assertRegex(self.c.read_text(encoding="utf-8"),
                         r'\{ 1u, "前进并右转", g_astChActItems1, 4u \}')


if __name__ == "__main__":
    unittest.main(verbosity=2)
