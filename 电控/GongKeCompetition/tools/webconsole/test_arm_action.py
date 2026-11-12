#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_action.py 的测试（全程沙箱: 不碰真实 app_arm_action.c/.h, 不碰真实回收站）。

    python tools/webconsole/test_arm_action.py

核心不变量（与 test_tuning.py 同款, 这是本项目对「回写源码」的统一要求）:
  · 无操作保存 = 两个文件都**字节级不变, 且 mtime 不变**（连 .bak 都不该建）
  · 未编辑的条目按**原始物理行**回吐 → 手写的排版不该被规范化
  · 只改该改的那几行
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
import arm_action as A                                              # noqa: E402

# 与真实 app_script.h 派生的 catalog 同形的一小块（序列化时选宏用）
CAT = {
    "STEP_ARM_GROUP_WAIT": {"params_f": ["s1", "s2", "s3", "s4", "s6"], "params_i": [],
                            "primitive": "vArmGroupMoveWait"},
    "STEP_ARM_GROUP_NO_WAIT": {"params_f": ["s1", "s2", "s3", "s4", "s6"], "params_i": [],
                               "primitive": "vArmGroupMoveNoWait"},
    "STEP_SET_ZHUASHOU_WAIT": {"params_f": [], "params_i": ["kai_he", "degree"],
                               "primitive": "vArmSetZhuaShou"},
    "STEP_DELAY": {"params_f": [], "params_i": ["ms"], "primitive": "vTaskDelay"},
}

FIXTURE_H = '''#ifndef __APP_ARM_ACTION_H
#define __APP_ARM_ACTION_H

#include "app_script.h"

/* ==== 生成块: 开始 ==== */
/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次
 * 保存后指向另一个动作（那会让路线里的引用于无声中换目标）。 */
#define ARM_ACTION_NEXT_ID    3u

/* 动作 ID: 路线表的 STEP_ARM_PLAY_WAIT / _NO_WAIT 用它引用动作; 控制台「动作」
 * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。
 * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emArmActionIdTdf)I(0)。
 *
 * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的（它的 glob 覆盖 Application
 *    下的头文件）。别因为「怕污染参数下拉」把它改成 #define —— 那样「动作」参数
 *    就没有下拉了, 只能手敲 ARM_ACT_n。 */
typedef enum {
    ARM_ACT_1 = 1,   /* 抓取A */
    ARM_ACT_2 = 2,   /* 放置B */
} emArmActionIdTdf;

/* 预设张数（= g_astArmAction[] 的长度, 控制台同步维护） */
#define ARM_ACTION_NUM        2u
/* ==== 生成块: 结束 ==== */

typedef struct {
    uint8_t                ucId;
    const char            *pcTitle;
    uint8_t                ucNoWait;
    const stScriptStepTdf *pstSteps;
    uint16_t               usCount;
} stArmActionTdf;

extern const stArmActionTdf g_astArmAction[ARM_ACTION_NUM];

#endif
'''

FIXTURE_C = '''#include "app_arm_action.h"

/* ==== 数据区: 开始 ==== */

/* ===== 预设: 抓取A ===== */
/* 播放方式: 阻塞到位 */
/* 备注: 先降到料位 */
const stScriptStepTdf g_astArmActionSteps1[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 90.0f, 45.0f, 90.0f, 90.0f, 180.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿1-夹爪", 1u, 100u),  // vArmSetZhuaShou()
    SCRIPT_STEP_I1     (STEP_DELAY,                           "延时", 300u),  // vTaskDelay()
};

/* ===== 预设: 放置B ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps2[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 120.0f, 80.0f, 90.0f, 90.0f, 180.0f),  // vArmGroupMoveWait()
};

const stArmActionTdf g_astArmAction[ARM_ACTION_NUM] = {
    { 1u, "抓取A", 0u, g_astArmActionSteps1, 3u },
    { 2u, "放置B", 1u, g_astArmActionSteps2, 1u },
};

/* ==== 数据区: 结束 ==== */

void vArmPlayWait(emArmActionIdTdf emId) { (void)emId; }
'''


class Sandbox(unittest.TestCase):
    """临时文件 + mock.patch.object 路径常量（照 test_tuning.py::Sandbox）。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_arm_"))
        self.c = self.tmp / "app_arm_action.c"
        self.h = self.tmp / "app_arm_action.h"
        self.trash = self.tmp / "arm_trash.json"
        self.c.write_text(FIXTURE_C, encoding="utf-8", newline="")
        self.h.write_text(FIXTURE_H, encoding="utf-8", newline="")
        for name, val in (("ACTION_C", self.c), ("ACTION_H", self.h),
                          ("TRASH_JSON", self.trash)):
            p = mock.patch.object(A, name, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    # ---- 帮手 ----

    def version(self):
        return A.parse_arm()["version"]

    def save(self, presets, base_version=None, next_id=None):
        return A.save({"presets": presets, "next_id": next_id},
                      self.version() if base_version is None else base_version, CAT)

    def texts(self):
        return (self.c.read_text(encoding="utf-8"), self.h.read_text(encoding="utf-8"))


SKELETON_C = '''#include "app_arm_action.h"

/* ==== 数据区: 开始 ==== */
/* ==== 数据区: 结束 ==== */
'''

SKELETON_H = '''#ifndef __APP_ARM_ACTION_H
#define __APP_ARM_ACTION_H

#include "app_script.h"

/* ==== 生成块: 开始 ==== */
/* ==== 生成块: 结束 ==== */

#endif
'''


class TestGeneratedFileIsStable(unittest.TestCase):
    """**用生成器造出来的文件, 再保存一次必须一字节不动。**

    这条单独测, 是因为它锁的是「造文件」与「解析回读」两条代码路径对得上 ——
    只测手写 fixture 会漏掉这条（真踩过: HTTP 上第一次保存报 changed=True, 文件被
    悄悄归一化了, 而单元测试全绿）。
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_arm_gen_"))
        self.c = self.tmp / "app_arm_action.c"
        self.h = self.tmp / "app_arm_action.h"
        # 空骨架: 让下面的 write_files 是**真的在造文件**（拿 FIXTURE 当起点会恰好在
        # 第一次就零 diff, 那样这条测试就什么都没测到）
        self.c.write_text(SKELETON_C, encoding="utf-8", newline="")
        self.h.write_text(SKELETON_H, encoding="utf-8", newline="")
        for name, val in (("ACTION_C", self.c), ("ACTION_H", self.h),
                          ("TRASH_JSON", self.tmp / "arm_trash.json")):
            p = mock.patch.object(A, name, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def test_bootstrap_then_save_is_a_noop(self):
        # 1) 用生成器写一遍（模拟控制台首次造出这两个文件）
        model = [{
            "id": 1, "title": "抓取A", "note": "先降到料位", "nowait": False,
            "items": [
                {"kind": "pose", "tag": "位姿1", "angles": [90, 45, 90, 90, 180],
                 "gripper": {"enabled": True, "kai_he": 1, "degree": 100}, "edited": True},
                {"kind": "delay", "tag": "延时", "ms": 300, "edited": True},
            ],
        }, {
            "id": 2, "title": "放置B", "note": "", "nowait": True,
            "items": [{"kind": "pose", "tag": "位姿1", "angles": [120, 80, 90, 90, 180],
                       "gripper": {"enabled": False, "kai_he": 1, "degree": 100},
                       "edited": True}],
        }]
        changed, _version = A.write_files(model, CAT, 1)
        self.assertTrue(changed, "空骨架应当被真的写成预设表")
        after_bootstrap = (self.c.read_text(encoding="utf-8"),
                           self.h.read_text(encoding="utf-8"))
        mtimes = (self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns)

        # 2) 解析回来, 原样再存一次 —— 必须什么都不动
        st = A.parse_arm()
        self.assertEqual([p["title"] for p in st["presets"]], ["抓取A", "放置B"])
        self.assertFalse(st["presets"][0]["nowait"])
        self.assertTrue(st["presets"][1]["nowait"])

        ok, resp, _ = A.save({"presets": st["presets"], "next_id": st["next_id"]},
                             st["version"], CAT)
        self.assertTrue(ok, resp)
        self.assertFalse(resp["changed"], "生成器造的文件再存一次竟然变了")
        self.assertEqual((self.c.read_text(encoding="utf-8"),
                          self.h.read_text(encoding="utf-8")), after_bootstrap)
        self.assertEqual((self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns), mtimes)

    def test_row_counts_match_the_written_tables(self):
        """聚合初始化里的条目数 = 表里真实的物理行数（一个位姿带夹爪就是两行）。"""
        model = [{
            "id": 1, "title": "A", "note": "", "nowait": False,
            "items": [
                {"kind": "pose", "tag": "位姿1", "angles": [1, 2, 3, 4, 5],
                 "gripper": {"enabled": True, "kai_he": 1, "degree": 100}, "edited": True},
                {"kind": "delay", "tag": "延时", "ms": 10, "edited": True},
            ],
        }]
        A.write_files(model, CAT, 1)
        st = A.parse_arm()
        body = A._serialize_items(st["presets"][0]["items"], CAT)
        text = self.c.read_text(encoding="utf-8")
        self.assertIn("{ 1u, \"A\", 0u, g_astArmActionSteps1, %du }" % len(body), text)
        self.assertEqual(len(body), 3)          # 位姿 + 夹爪 + 延时


class TestPlayModeField(Sandbox):
    """播放方式的权威 = 聚合行的第 3 段（固件运行时读它）, 表头那行注释只是它的渲染。

    这一组守的是「手改 C 之后控制台还能不能如实读出播放方式」——改字段、改注释、
    改错格式三种情形各一条。固件那边靠 vArmPlayByMode 读同一个字段,
    所以这里读到的就是排爆抓取/反恐任务实际会用的那个值。
    """

    ROW1 = '{ 1u, "抓取A", 0u, g_astArmActionSteps1, 3u },'

    def _rewrite_row(self, new):
        self.c.write_text(self.c.read_text(encoding="utf-8").replace(self.ROW1, new),
                          encoding="utf-8", newline="")

    def test_field_wins_over_the_comment(self):
        # 手改: 字段=启动即返回, 注释却还写着「阻塞到位」—— 以字段为准(固件只认它),
        # 并且保存时把注释规范回字段值（否则操作员看着注释以为改了, 实际没改）。
        self._rewrite_row('{ 1u, "抓取A", 1u, g_astArmActionSteps1, 3u },')
        st = A.parse_arm()
        self.assertTrue(st["presets"][0]["nowait"])
        ok, resp, _ = self.save(st["presets"])
        self.assertTrue(ok, resp)
        self.assertIn("/* 播放方式: 启动即返回 */", self.c.read_text(encoding="utf-8"))

    def test_legacy_row_reads_the_comment_then_migrates(self):
        # 旧格式（没有播放方式字段）: 仍按注释读（= 老行为, 不惊动旧文件）,
        # 保存一次即迁移成带字段的新格式。
        self._rewrite_row('{ 1u, "抓取A", g_astArmActionSteps1, 3u },')
        st = A.parse_arm()
        self.assertFalse(st["presets"][0]["nowait"])     # 来自注释「阻塞到位」
        self.assertTrue(st["presets"][1]["nowait"])      # 来自注释「启动即返回」
        ok, resp, _ = self.save(st["presets"])
        self.assertTrue(ok, resp)
        self.assertIn(self.ROW1, self.c.read_text(encoding="utf-8"))

    def test_malformed_row_raises_instead_of_dropping_the_preset(self):
        # 段数不对的行**必须报错**。原来静默跳过 → 预设从界面消失, 而保存时数据区整段
        # 重生成 → 那张步骤表被无声删掉（这正是加字段最容易踩的坑）。
        self._rewrite_row('{ 1u, "抓取A", 0u, 1u, g_astArmActionSteps1, 3u },')
        with self.assertRaises(A.ArmError):
            A.parse_arm()


class TestParse(Sandbox):

    def test_presets_in_aggregate_order(self):
        p = A.parse_arm()
        self.assertEqual([x["id"] for x in p["presets"]], [1, 2])
        self.assertEqual([x["title"] for x in p["presets"]], ["抓取A", "放置B"])
        self.assertEqual(p["next_id"], 3)

    def test_mode_and_note_lines(self):
        p = A.parse_arm()
        self.assertFalse(p["presets"][0]["nowait"])
        self.assertEqual(p["presets"][0]["note"], "先降到料位")
        self.assertTrue(p["presets"][1]["nowait"])
        self.assertEqual(p["presets"][1]["note"], "")

    def test_pose_absorbs_its_gripper_line(self):
        """位姿 + 紧跟的夹爪 = 一条（界面上的一行）, 而不是两条。"""
        items = A.parse_arm()["presets"][0]["items"]
        self.assertEqual([i["kind"] for i in items], ["pose", "delay"])
        self.assertEqual(items[0]["gripper"], {"enabled": True, "kai_he": 1, "degree": 100})
        self.assertEqual(items[0]["angles"], [90.0, 45.0, 90.0, 90.0, 180.0])
        self.assertEqual(len(items[0]["lines"]), 2)          # 位姿行 + 夹爪行

    def test_standalone_gripper_is_its_own_item(self):
        """不跟在位姿后面的夹爪单列成一条 —— 不能被吞进上一行。"""
        body = ['    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 1.0f, 2.0f, 3.0f, 4.0f, 5.0f),  // vArmGroupMoveWait()',
                '    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿1-夹爪", 1u, 100u),  // vArmSetZhuaShou()',
                '    SCRIPT_STEP_I1     (STEP_DELAY,                           "延时", 300u),  // vTaskDelay()',
                '    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "再夹", 0u, 50u),  // vArmSetZhuaShou()']
        items = A._items_from_body(body)
        self.assertEqual([i["kind"] for i in items], ["pose", "delay", "grip"])
        self.assertEqual(items[2]["kai_he"], 0)
        self.assertEqual(items[2]["degree"], 50)

    def test_every_item_has_a_unique_id(self):
        """每条子积木都必须带唯一 id —— 前端拿它当 DOM id、选中判据、拖拽 skip 判据、
        删除的定位键。**漏掉过一次**，后果是 `data-id` 根本没被设上（h() 会跳过
        undefined），于是「点一块选中全部」+「拖动其它块不让位」+「删除永远删第一块」，
        三个看起来毫不相干的症状全都来自这一个字段。"""
        for preset in A.parse_arm()["presets"]:
            ids = [it.get("id") for it in preset["items"]]
            self.assertTrue(all(ids), "有条目没有 id: %r" % (ids,))
            self.assertEqual(len(ids), len(set(ids)), "id 重复: %r" % (ids,))

    def test_merged_gripper_keeps_one_id(self):
        """位姿 + 夹爪合并成一条后只留一个 id（不能是两条各一个）。"""
        body = ['    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 1.0f, 2.0f, 3.0f, 4.0f, 5.0f),  // vArmGroupMoveWait()',
                '    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿1-夹爪", 1u, 100u),  // vArmSetZhuaShou()']
        items = A._items_from_body(body)
        self.assertEqual(len(items), 1)
        self.assertTrue(items[0]["id"])

    def test_all_kinds_get_ids(self):
        """三种 kind（含只读块）都要有 id。"""
        # ⚠️ 夹爪要排在**不紧跟位姿**的位置, 否则会按设计并进上一条位姿（那另有一条测试）
        body = ['    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 1.0f, 2.0f, 3.0f, 4.0f, 5.0f),  // vArmGroupMoveWait()',
                '    SCRIPT_STEP_I1     (STEP_DELAY,                           "延时", 300u),  // vTaskDelay()',
                '    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "再夹", 0u, 50u),  // vArmSetZhuaShou()',
                '    SCRIPT_STEP_I1     (STEP_SOMETHING_ELSE,                  "别的", 1u),  // vOther()']
        items = A._items_from_body(body)
        self.assertEqual([i["kind"] for i in items], ["pose", "delay", "grip", "raw"])
        ids = [i["id"] for i in items]
        self.assertTrue(all(ids), ids)
        self.assertEqual(len(ids), len(set(ids)), ids)

    def test_ids_survive_a_save_and_reload(self):
        presets = A.parse_arm()["presets"]
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        for preset in A.parse_arm()["presets"]:
            ids = [it["id"] for it in preset["items"]]
            self.assertTrue(all(ids), ids)
            self.assertEqual(len(ids), len(set(ids)), ids)

    def test_unknown_step_kept_as_raw(self):
        """白名单外的步骤原样留着当只读块 —— 不猜、不丢。"""
        body = ['    SCRIPT_STEP_I1     (STEP_SOMETHING_ELSE,              "别的", 1u),  // vOther()']
        items = A._items_from_body(body)
        self.assertEqual(items[0]["kind"], "raw")
        self.assertEqual(items[0]["lines"], body)

    def test_param_suffixes(self):
        self.assertEqual(A._param_float("90.0f"), 90.0)
        self.assertEqual(A._param_float("300u"), 300.0)
        self.assertEqual(A._param_float("-30"), -30.0)
        self.assertEqual(A._param_int("100u"), 100)
        self.assertEqual(A._param_float("0x1f"), 0.0)        # 非十进制原样失败, 不瞎剥 f


class TestNoOpSave(Sandbox):
    """无操作保存必须什么都不动 —— 这条坏了, 另一个 Tab 的下次保存就会吃 409。"""

    def test_byte_identical_and_mtime_untouched(self):
        before = self.texts()
        mtimes = (self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns)

        ok, resp, status = self.save(A.parse_arm()["presets"])

        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)
        self.assertFalse(resp["changed"], "内容没变却报告 changed")
        self.assertEqual(self.texts(), before, "无操作保存改动了文件字节")
        self.assertEqual((self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns), mtimes,
                         "无操作保存动了 mtime（会让另一个 Tab 吃 409）")
        self.assertFalse(self.c.with_suffix(".c.bak").exists(), "无操作保存不该建 .bak")

    def test_hand_written_formatting_is_kept(self):
        """手改过的排版（多余空格、没有行尾原语注释）不该被规范化。"""
        odd = '    SCRIPT_STEP_I1(STEP_DELAY,   "延时",   300u),'
        text = self.c.read_text(encoding="utf-8").replace(
            '    SCRIPT_STEP_I1     (STEP_DELAY,                           "延时", 300u),  // vTaskDelay()',
            odd)
        self.c.write_text(text, encoding="utf-8", newline="")
        self.assertNotEqual(text, FIXTURE_C)

        ok, resp, _ = self.save(A.parse_arm()["presets"])
        self.assertTrue(ok, resp)
        self.assertIn(odd, self.c.read_text(encoding="utf-8"))


class TestEdits(Sandbox):

    def test_rename_only_touches_its_own_lines(self):
        presets = A.parse_arm()["presets"]
        presets[1]["title"] = "放置B改"
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)

        old = FIXTURE_C.split("\n")
        new = self.c.read_text(encoding="utf-8").split("\n")
        self.assertEqual(len(old), len(new), "改个标题不该改变行数")
        changed = [n for o, n in zip(old, new) if o != n]
        # 只该动两行: 预设头注释 + 聚合初始化里那一行
        self.assertEqual(changed, ['/* ===== 预设: 放置B改 ===== */',
                                   '    { 2u, "放置B改", 1u, g_astArmActionSteps2, 1u },'])
        self.assertIn('/* 放置B改 */', self.h.read_text(encoding="utf-8"))

    def test_edited_item_is_regenerated_others_kept_raw(self):
        presets = A.parse_arm()["presets"]
        presets[0]["items"][1]["ms"] = 500
        presets[0]["items"][1]["edited"] = True
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        text = self.c.read_text(encoding="utf-8")
        self.assertIn('"延时", 500u', text)
        # 同一条预设里没编辑的那条仍在（原始行）
        self.assertIn('"位姿1", 90.0f, 45.0f', text)

    def test_nowait_flag_roundtrip(self):
        presets = A.parse_arm()["presets"]
        presets[0]["nowait"] = True
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        self.assertIn("/* 播放方式: 启动即返回 */", self.c.read_text(encoding="utf-8"))
        # 字段（固件运行时读的那个）也要跟着变
        self.assertIn('{ 1u, "抓取A", 1u, g_astArmActionSteps1, 3u },',
                      self.c.read_text(encoding="utf-8"))
        self.assertTrue(A.parse_arm()["presets"][0]["nowait"])

    def test_next_id_never_recycled(self):
        """删掉最大 ID 后, NEXT_ID 必须继续往前走 —— 复用它会让 map.c 里
        残留的 ARM_ACT_n 于无声中指向另一个动作。"""
        presets = A.parse_arm()["presets"]
        ok, resp, _ = A.delete_preset(2, self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertEqual(A.parse_arm()["next_id"], 3)

        ok, resp, _ = self.save(A.parse_arm()["presets"] + [
            {"title": "新的", "note": "", "nowait": False,
             "items": [{"kind": "delay", "tag": "延时", "ms": 100, "edited": True}]}])
        self.assertTrue(ok, resp)
        self.assertEqual([p["id"] for p in A.parse_arm()["presets"]], [1, 3],
                         "新预设复用了已删的 ID")
        self.assertEqual(A.parse_arm()["next_id"], 4)

    def test_enum_matches_tables(self):
        presets = A.parse_arm()["presets"]
        presets[1]["items"].append({"kind": "delay", "tag": "延时", "ms": 50, "edited": True})
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        text = self.c.read_text(encoding="utf-8")
        h = self.h.read_text(encoding="utf-8")
        self.assertIn("#define ARM_ACTION_NUM        2u", h)
        self.assertIn("ARM_ACT_1 = 1,", h)
        self.assertIn("ARM_ACT_2 = 2,", h)
        self.assertIn('{ 2u, "放置B", 1u, g_astArmActionSteps2, 2u }', text)


class TestGuards(Sandbox):

    def test_empty_preset_list_rejected(self):
        ok, resp, status = self.save([])
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("至少", resp["message"])

    def test_preset_without_items_rejected(self):
        ok, resp, status = self.save([{"title": "空的", "note": "", "items": []}])
        self.assertFalse(ok)
        self.assertEqual(status, 400)

    def test_conflict_on_stale_version(self):
        ok, resp, status = self.save(A.parse_arm()["presets"], base_version={"c": 1.0, "h": 1.0})
        self.assertFalse(ok)
        self.assertEqual(status, 409)
        self.assertEqual(resp["error"], "conflict")

    def test_last_preset_cannot_be_deleted(self):
        A.delete_preset(2, self.version(), CAT)
        ok, resp, status = A.delete_preset(1, self.version(), CAT)
        self.assertFalse(ok)
        self.assertEqual(status, 400)

    def test_missing_marker_is_reported_not_crashed(self):
        self.c.write_text("/* 没有标记 */\n", encoding="utf-8", newline="")
        st = A.state()
        self.assertIn("error", st)
        self.assertEqual(st["presets"], [])


class TestSanitize(unittest.TestCase):

    def test_comment_breakers_are_neutralised(self):
        for bad in ("a*/b", "a/*b", "a\nb", "a\r\nb", 'a"b', "a//b", "a(b"):
            got = A.sanitize_comment(bad)
            self.assertNotIn("*/", got, bad)
            self.assertNotIn("/*", got, bad)
            self.assertNotIn("\n", got, bad)
            self.assertNotIn('"', got, bad)

    def test_title_is_truncated(self):
        self.assertLessEqual(len(A.sanitize_comment("长" * 80)), A.MAX_TITLE_LEN)

    def test_empty_title_keeps_a_fallback(self):
        """清空标题不能真的写出一个空名字 —— 动作列表里那一行会变成空白。

        ⚠️ 这条钉的是「清洗规则搬进 map_codec 后行为没变」: 规则本体现在有了
           `fallback` 参数, 而 arm 这条路必须继续传「臂组动作」。
        """
        self.assertEqual(A.sanitize_comment(""), "臂组动作")
        self.assertEqual(A.sanitize_comment("   "), "臂组动作")

    def test_repeated_slashes_are_all_collapsed(self):
        """`//` 折干净（旧版只折一次, `'a///b'` 会留下 `'a//b'` → 尾逗号插进字符串里）。"""
        self.assertNotIn("//", A.sanitize_comment("a///b"))

    def test_injection_survives_a_save(self):
        """标题里塞注释终止符, 存回去后文件仍能被解析（不产生半截注释）。"""
        tmp = Path(tempfile.mkdtemp(prefix="gks_arm_inj_"))
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        c, h = tmp / "a.c", tmp / "a.h"
        c.write_text(FIXTURE_C, encoding="utf-8", newline="")
        h.write_text(FIXTURE_H, encoding="utf-8", newline="")
        with mock.patch.object(A, "ACTION_C", c), mock.patch.object(A, "ACTION_H", h):
            presets = A.parse_arm()["presets"]
            presets[0]["title"] = "偷偷 */ 结束注释"
            ok, resp, _ = A.save({"presets": presets}, A.parse_arm()["version"], CAT)
            self.assertTrue(ok, resp)
            # 还能解析回来 = 注释没被提前闭合
            self.assertEqual(len(A.parse_arm()["presets"]), 2)


class TestTrash(Sandbox):

    def test_delete_moves_to_trash(self):
        ok, resp, _ = A.delete_preset(2, self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertEqual([p["id"] for p in A.parse_arm()["presets"]], [1])
        trash = A.load_trash()
        self.assertEqual(len(trash), 1)
        self.assertEqual(trash[0]["title"], "放置B")
        self.assertEqual(trash[0]["id"], 2)
        self.assertTrue(resp["trash_id"])

    def test_restore_naming_and_fresh_id(self):
        A.delete_preset(2, self.version(), CAT)
        tid = A.load_trash()[0]["trash_id"]

        ok, resp, _ = A.restore_preset(tid, self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertEqual(resp["title"], "放置B恢复")
        p = A.parse_arm()
        self.assertEqual([x["title"] for x in p["presets"]], ["抓取A", "放置B恢复"])
        self.assertEqual(p["presets"][1]["id"], 3, "恢复必须发新 ID, 不能复用已删的 2")
        self.assertEqual(A.load_trash(), [])

    def test_restore_does_not_stack_suffixes(self):
        """删「放置B恢复」再恢复, 名字应回到「放置B恢复」而不是叠成「放置B恢复恢复」。"""
        A.delete_preset(2, self.version(), CAT)
        A.restore_preset(A.load_trash()[0]["trash_id"], self.version(), CAT)
        A.delete_preset(3, self.version(), CAT)
        ok, resp, _ = A.restore_preset(A.load_trash()[0]["trash_id"], self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertEqual(resp["title"], "放置B恢复")

    def test_restore_gets_numbered_when_name_is_taken(self):
        """名字被活着的预设占了才编号 —— 这是 (x) 存在的唯一理由。"""
        A.delete_preset(2, self.version(), CAT)
        presets = A.parse_arm()["presets"]
        presets[0]["title"] = "放置B恢复"
        self.save(presets)

        ok, resp, _ = A.restore_preset(A.load_trash()[0]["trash_id"], self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertEqual(resp["title"], "放置B恢复(1)")
        titles = [x["title"] for x in A.parse_arm()["presets"]]
        self.assertEqual(len(titles), len(set(titles)), "恢复后标题重名了")

    def test_restore_keeps_content(self):
        A.delete_preset(1, self.version(), CAT)
        tid = A.load_trash()[0]["trash_id"]
        ok, resp, _ = A.restore_preset(tid, self.version(), CAT)
        self.assertTrue(ok, resp)
        got = [p for p in A.parse_arm()["presets"] if p["id"] == resp["presets"][-1]["id"]][0]
        self.assertEqual(len(got["items"]), 2)
        self.assertEqual(got["items"][0]["gripper"]["degree"], 100)

    def test_purge_only_touches_trash(self):
        A.delete_preset(2, self.version(), CAT)
        tid = A.load_trash()[0]["trash_id"]
        before = self.texts()
        ok, resp, status = A.purge_trash(tid)
        self.assertTrue(ok)
        self.assertEqual(status, 200)
        self.assertEqual(A.load_trash(), [])
        self.assertEqual(self.texts(), before, "彻底删除不该碰 C 文件")

    def test_trash_is_not_created_by_a_plain_save(self):
        self.save(A.parse_arm()["presets"])
        self.assertFalse(self.trash.exists(), "普通保存不该写出回收站文件")


class TestOfflineJoints(unittest.TestCase):
    """没连设备时画灰色滑杆用的占位表 —— 全部从源码派生（只读真实源码, 不写盘）。"""

    def test_shape_and_names(self):
        j = A.offline_joints()
        self.assertEqual(len(j), 5, "组轴是 SERVO1~4 + SERVO6 五个")
        self.assertEqual([x["name"] for x in j], ["s1", "s2", "s3", "s4", "s6"])
        self.assertEqual([x["index"] for x in j], [0, 1, 2, 3, 4])

    def test_ranges_come_from_the_firmware(self):
        """180° 轴量程 0~180, 360° 底座量程 0~360（后者走 vServoDeviceInit_Angle360At）。"""
        j = A.offline_joints()
        for x in j[:4]:
            self.assertEqual((x["min"], x["max"]), (0.0, 180.0), x["name"])
        self.assertEqual((j[4]["min"], j[4]["max"]), (0.0, 360.0), "s6 底座应是 0~360")

    def test_angles_are_the_power_on_pose(self):
        """初始位置取 s_fArmPoseInit —— 且必须剥掉 C 的 f 后缀。

        踩过: 用裸 float() 解析 '90.0f' 会失败并静默回退成 0, 于是五个滑杆看着像
        「位姿全归零」（而 0° 对舵机是个危险位置）。

        ⚠️ 期望值**从源码现取**（用一条独立于 _param_float 的朴素剥后缀写法）, 不在测试里
        抄一串常量 —— 台架上调上电位姿是常事, 抄常量会变成「改一次挂一次」的假警报,
        而这条测试真正要防的是「静默回退成 0」。组顺序 SERVO1,2,3,4,6 → 位姿表下标 0,1,2,3,5。
        """
        angles = [x["angle"] for x in A.offline_joints()]
        self.assertEqual(len(angles), 5)

        src = A.ARM_C.read_text(encoding="utf-8")
        raw = re.sub(r"/\*.*?\*/", "", A.RE_POSE_INIT.search(src).group(1), flags=re.S)
        pose = [float(t.strip().rstrip("fFuUlL")) for t in raw.split(",") if t.strip()]
        self.assertEqual(angles, [pose[0], pose[1], pose[2], pose[3], pose[5]])

        # 那个「全归零」的坑单独钉一句（它才是这条测试存在的理由）
        self.assertNotEqual(angles, [0.0, 0.0, 0.0, 0.0, 0.0])

    def test_degrades_to_empty_when_sources_are_missing(self):
        with mock.patch.object(A, "ARM_C", Path("Z:/nope.c")), \
             mock.patch.object(A, "ARM_CONFIG_H", Path("Z:/nope.h")):
            self.assertEqual(A.offline_joints(), [])

    def test_state_carries_it(self):
        self.assertIn("joints_offline", A.state())


class TestGripperSpan(unittest.TestCase):
    """夹爪端点（角度 ↔ 开合% 反解用）必须从 app_config.h 现取, 不在服务端抄一份。

    夹爪**不进**设备上报的关节表, 所以「获取当前位姿」拿到的是角度, 而控件是
    「开合 + 程度%」—— 这个反解只能靠编译期常量, 也就只能从源码派生（同 joints_offline）。
    """

    def test_reads_the_ends_from_app_config(self):
        span = A.gripper_span()
        self.assertIsNotNone(span)
        cfg = A.ARM_CONFIG_H.read_text(encoding="utf-8")
        for key, name in (("open", "ZHUASHOU_OPEN_ANGLE"),
                          ("close", "ZHUASHOU_CLOSE_ANGLE")):
            want = float(re.search(r"#define\s+%s\s+([-\d.]+)" % name,
                                   cfg).group(1))
            self.assertEqual(span[key], want)

    def test_min_max_bracket_the_ends(self):
        span = A.gripper_span()
        self.assertLessEqual(span["min"], min(span["open"], span["close"]))
        self.assertGreaterEqual(span["max"], max(span["open"], span["close"]))

    def test_degrades_to_none_when_sources_missing(self):
        with mock.patch.object(A, "ARM_CONFIG_H", Path("Z:/nope.h")):
            self.assertIsNone(A.gripper_span())

    def test_state_carries_it(self):
        self.assertIn("gripper_span", A.state())


DISABLED_DELAY = ('//    SCRIPT_STEP_I1     (STEP_DELAY,'
                  '                           "延时", 300u),  // vTaskDelay()')


class TestDisable(Sandbox):
    """子积木的「眼睛」= 禁用 = 行首 `//`（与 map.c 的单步禁用同一套规则）。

    最要紧的一条：**被禁用的行是注释、不进数组**，所以聚合初始化里的条目数必须跟着
    减 —— 数错了 `vArmPlayWait` 会按 usCount 越过表尾读别的东西。
    """

    def comment_out_delay(self):
        """把 fixture 里那条延时注释掉（模拟手写一个禁用，或上次保存的结果）。"""
        text = self.c.read_text(encoding="utf-8").replace(
            '    SCRIPT_STEP_I1     (STEP_DELAY,                           "延时", 300u),  // vTaskDelay()',
            DISABLED_DELAY)
        self.c.write_text(text, encoding="utf-8", newline="")

    def test_parse_reads_the_eye(self):
        self.comment_out_delay()
        items = A.parse_arm()["presets"][0]["items"]
        self.assertEqual([i["kind"] for i in items], ["pose", "delay"])
        self.assertFalse(items[0]["disabled"])
        self.assertTrue(items[1]["disabled"], "行首 // 没被认成禁用")

    def test_pose_and_grip_are_disabled_together(self):
        """位姿 + 紧跟的夹爪是一条积木、两行 —— **两行都注释了**才算禁用。"""
        pose = ('    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,'
                '                  "位姿1", 1.0f, 2.0f, 3.0f, 4.0f, 5.0f),  // vArmGroupMoveWait()')
        grip = ('    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,'
                '               "位姿1-夹爪", 1u, 100u),  // vArmSetZhuaShou()')
        self.assertFalse(A._items_from_body([pose, grip])[0]["disabled"])
        # 只注释了位姿那一行 → 整条**不算**禁用（夹爪那一行还在数组里）
        self.assertFalse(A._items_from_body(["//" + pose, grip])[0]["disabled"])
        self.assertTrue(A._items_from_body(["//" + pose, "//" + grip])[0]["disabled"])

    def test_toggle_writes_the_eye_on_every_line(self):
        presets = A.parse_arm()["presets"]
        presets[0]["items"][0]["disabled"] = True          # 位姿（带夹爪 → 两行）
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        text = self.c.read_text(encoding="utf-8")
        self.assertIn("//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT", text)
        self.assertIn("//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT", text)
        # 同一条预设里没被禁用的那条原样不动
        self.assertIn('\n    SCRIPT_STEP_I1     (STEP_DELAY,', text)

    def test_count_excludes_disabled_rows(self):
        """聚合初始化的条目数 = 进得了数组的行数（禁用的行不是初始化项）。"""
        presets = A.parse_arm()["presets"]
        presets[0]["items"][0]["disabled"] = True          # 位姿 + 夹爪 两行一起走
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        self.assertIn('{ 1u, "抓取A", 0u, g_astArmActionSteps1, 1u },',
                      self.c.read_text(encoding="utf-8"))

    def test_second_save_is_a_noop(self):
        """禁用状态必须能原样解析回来 —— 否则第二次保存会把文件改回去（diff 一片）。"""
        presets = A.parse_arm()["presets"]
        presets[0]["items"][1]["disabled"] = True
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        self.assertTrue(resp["changed"])
        after = self.texts()

        st = A.parse_arm()
        self.assertTrue(st["presets"][0]["items"][1]["disabled"])
        ok, resp, _ = A.save({"presets": st["presets"], "next_id": st["next_id"]},
                             st["version"], CAT)
        self.assertTrue(ok, resp)
        self.assertFalse(resp["changed"], "禁用后的文件再存一次竟然变了")
        self.assertEqual(self.texts(), after)

    def test_enable_restores_the_original_line(self):
        """再点一次眼睛（启用）要**还原成原来的物理行**, 不能留下 `//    ` 这种残迹。"""
        self.comment_out_delay()
        presets = A.parse_arm()["presets"]
        presets[0]["items"][1]["disabled"] = False
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)
        text = self.c.read_text(encoding="utf-8")
        self.assertIn('    SCRIPT_STEP_I1     (STEP_DELAY,', text)
        self.assertNotIn("//    SCRIPT_STEP_I1", text)

    def test_all_disabled_is_rejected(self):
        """全禁用 = 空数组体, C 编译不过 —— 保存前就拦下来。"""
        presets = [A.parse_arm()["presets"][0]]
        for it in presets[0]["items"]:
            it["disabled"] = True
        ok, resp, status = self.save(presets)
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("禁用", resp["message"])

    def test_disable_survives_the_trash(self):
        presets = A.parse_arm()["presets"]
        presets[0]["items"][1]["disabled"] = True
        ok, resp, _ = self.save(presets)
        self.assertTrue(ok, resp)

        ok, resp, _ = A.delete_preset(1, self.version(), CAT)
        self.assertTrue(ok, resp)
        self.assertTrue(resp["trash_id"])
        ok, resp, _ = A.restore_preset(resp["trash_id"], self.version(), CAT)
        self.assertTrue(ok, resp)
        back = [p for p in A.parse_arm()["presets"] if p["title"].startswith("抓取A")][0]
        self.assertTrue(back["items"][1]["disabled"], "回收站把禁用状态丢了")


class TestRestoreTitle(unittest.TestCase):

    def test_suffix_sequence(self):
        self.assertEqual(A.restore_title("抓取", set()), "抓取恢复")
        self.assertEqual(A.restore_title("抓取", {"抓取恢复"}), "抓取恢复(1)")
        self.assertEqual(A.restore_title("抓取", {"抓取恢复", "抓取恢复(1)"}), "抓取恢复(2)")


class TestDispatch(Sandbox):

    def test_unknown_action(self):
        ok, data, err, _st = A.run_arm_action("nope", {}, CAT)   # 2026-10-31 起回 4 元组
        self.assertFalse(ok)
        self.assertIn("未知动作", err)

    def test_error_message_is_returned_not_raised(self):
        ok, data, err, _st = A.run_arm_action("delete", {"id": 999}, CAT)
        self.assertFalse(ok)
        self.assertTrue(err)

    def test_state_shape(self):
        st = A.state()
        for key in ("version", "next_id", "presets", "trash"):
            self.assertIn(key, st)

    def test_trash_json_is_utf8_readable(self):
        A.delete_preset(2, self.version(), CAT)
        raw = json.loads(self.trash.read_text(encoding="utf-8"))
        self.assertEqual(raw["items"][0]["title"], "放置B")
        self.assertIn("deleted_at", raw["items"][0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
