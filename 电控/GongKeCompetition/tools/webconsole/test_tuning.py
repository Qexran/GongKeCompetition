#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tuning（app_config.h 调参后端）的离线测试 —— 全部在 tempfile 沙箱里跑，
绝不写真实的 Application/app_config.h。

    python tools/webconsole/test_tuning.py

最重要的一条是 test_roundtrip_is_byte_identical：无操作保存必须字节级零 diff，
否则「调参页点一下保存」就会把整个头文件的 diff 搅浑。
"""

import shutil
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tuning as T                                                  # noqa: E402

# 覆盖各条路径: 分区头、前置块注释、u/f/括号/纯整数/符号值、转换、开区间范围
FIXTURE = """\
/**
  * @file       app_config.h
  */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ==================== A 组 ==================== */
/* 前置块注释（不是尾注释, 不该被当成 DSL）
   第二行 */
#define PLAIN_UINT      3000u       /* 超时 [ms] @(0~) // 说明文字 */
#define PLAIN_FLOAT     45.0f       /* 角度 [°] @(0~180) // pro 初值 */
#define PAREN_INT        (-30)      /* 回退速度 [rpm] // 负值=后退 */
#define PAREN_FLOAT      (+1.0f)    /* 方向 // 反了改 -1 */
#define BARE_INT        80          /* 弧线速度 [rpm] @(0~65535) // 落进 uint16_t */
#define SYMBOLIC        UART_ID_RPI /* 树莓派 // USART1 PA9/PA10 */
#define NOCMT           123

/* ==================== B 组 ==================== */
#define PERIOD_MS       20u         /* 节拍 [Hz] {ms->Hz} @(5~200) // 底层 20ms */
#define DUTY            500u        /* 占空比 [%] {‰->%} @(0~100) // 千分比 */
#define BLANK_PCT       255u        /* 空值参数 [%] @(0~100) // 留空=255 默认取DUTY(空框+「默认」hint); 0=不亮; 1~100=单独亮度 */

#endif /* __APP_CONFIG_H */
"""


class Sandbox(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_tuning_"))
        self.cfg = self.tmp / "app_config.h"
        self.cfg.write_text(FIXTURE, encoding="utf-8", newline="")
        for name, val in (("APP_CONFIG", self.cfg),
                          ("PREFS_FILE", self.tmp / "tuning.json")):
            p = mock.patch.object(T, name, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def reload(self):
        return self.cfg.read_text(encoding="utf-8")

    def item(self, name):
        for g in T.parse_config(self.reload())["groups"]:
            for it in g["items"]:
                if it["name"] == name:
                    return it
        raise AssertionError("fixture 里没有 %s" % name)


class TestRoundTrip(Sandbox):
    def test_roundtrip_is_byte_identical(self):
        """每个条目重建后必须与原行逐字节一致。"""
        for g in T.parse_config(FIXTURE)["groups"]:
            for it in g["items"]:
                self.assertEqual(T._rebuild_line(it), it["line"],
                                 "重建不一致: %s" % it["name"])

    def test_no_op_save_is_byte_identical(self):
        """空改动保存 → 文件一个字节都不变。"""
        before = self.reload()
        ok, resp, st = T.write_config({}, None, base_version=T.version())
        self.assertTrue(ok, resp)
        self.assertEqual(self.reload(), before)

    def test_unchanged_value_keeps_raw_line(self):
        """值填成原值 → 该行不进重建路径。"""
        before = self.reload()
        T.write_config({"PLAIN_UINT": "3000"}, None)
        self.assertEqual(self.reload(), before)


class TestParse(Sandbox):
    def test_groups_and_counts(self):
        g = T.parse_config(FIXTURE)["groups"]
        self.assertEqual([x["title"] for x in g], ["A 组", "B 组"])
        self.assertEqual([len(x["items"]) for x in g], [7, 3])

    def test_preceding_block_comment_is_not_dsl(self):
        """#define 上面的块注释不能被当成尾注释。"""
        it = self.item("PLAIN_UINT")
        self.assertEqual(it["meta"]["display"], "超时")
        self.assertEqual(it["meta"]["note"], "说明文字")

    def test_no_comment_falls_back_to_macro_name(self):
        it = self.item("NOCMT")
        self.assertFalse(it["has_comment"])
        self.assertEqual(T.public_item(it)["display"], "NOCMT")

    def test_open_ended_range(self):
        self.assertEqual(T.parse_dsl("x [ms] @(2500~)")["range"], {"lo": "2500", "hi": ""})
        self.assertEqual(T.parse_dsl("x [ms] @(~100)")["range"], {"lo": "", "hi": "100"})

    def test_note_may_contain_brackets(self):
        """备注里的 [] {} 不能被当成单位/转换（先切 // 再解析）。"""
        m = T.parse_dsl("轴数 [个] @(5~5) // 与 afF[6] 三处耦合")
        self.assertEqual(m["unit"], "个")
        self.assertEqual(m["range"], {"lo": "5", "hi": "5"})
        self.assertEqual(m["note"], "与 afF[6] 三处耦合")

    def test_dsl_format_roundtrip(self):
        for s in ("超时 [ms] @(0~)", "占空比 [%] {‰->%} @(0~100) // 千分比",
                  "方向 // 反了改 -1", "无修饰"):
            self.assertEqual(T.format_dsl(T.parse_dsl(s)), s, s)


class TestEmptySentinel(Sandbox):
    """空值哨兵参数（备注 DSL「留空=255 默认取DUTY」, FILL_TASK_*_PCT 首用）:
    留空 = 跟随默认, 填 0 = 字面 0（如「开灯不亮」）—— 两种状态靠哨兵值区分。"""

    def test_parse_dsl_derives_empty_and_source(self):
        m = T.parse_dsl("亮度 [%] @(0~100) // 留空=255 默认取DUTY(空框+「默认」hint); 0=不亮")
        self.assertEqual(m["empty"], 255)
        self.assertEqual(m["empty_from"], "DUTY")
        self.assertEqual(m["note"], "留空=255 默认取DUTY(空框+「默认」hint); 0=不亮")

    def test_no_marker_means_no_empty(self):
        m = T.parse_dsl("亮度 [%] // 0=没单独填")          # 旧式 0 哨兵备注不算
        self.assertIsNone(m["empty"])
        self.assertIsNone(T.parse_dsl("亮度 [%]")["empty"])

    def test_sentinel_state_displays_blank_without_range_warning(self):
        """值恰为哨兵 → display_value 发空串（界面渲染空框）, 且不把 255 当「超范围」。"""
        pub = T.public_item(self.item("BLANK_PCT"))
        self.assertEqual(pub["display_value"], "")
        self.assertIsNone(pub["range_warning"])
        self.assertEqual(pub["empty"], 255)
        self.assertEqual(pub["empty_from"], "DUTY")

    def test_non_sentinel_value_displays_normally(self):
        pub = T.public_item(self.item("PLAIN_UINT"))
        self.assertEqual(pub["display_value"], 3000)
        self.assertIsNone(pub["empty"])

    def test_blank_submit_writes_sentinel_zero_diff(self):
        """留空提交 = 写回哨兵 = 数值没变 → 一个字节都不落盘。"""
        before = self.reload()
        ok, resp, _ = T.write_config({"BLANK_PCT": ""}, None)
        self.assertTrue(ok)
        self.assertEqual(self.reload(), before)
        self.assertEqual(resp.get("warnings"), [])

    def test_zero_submit_is_literal_zero_not_default(self):
        ok, _, _ = T.write_config({"BLANK_PCT": "0"}, None)
        self.assertTrue(ok)
        self.assertEqual(T.to_number(self.item("BLANK_PCT")), 0.0)

    def test_value_then_blank_roundtrip_restores_sentinel(self):
        T.write_config({"BLANK_PCT": "60"}, None)
        self.assertEqual(T.to_number(self.item("BLANK_PCT")), 60.0)
        T.write_config({"BLANK_PCT": ""}, None)
        it = self.item("BLANK_PCT")
        self.assertEqual(T.to_number(it), 255.0)
        self.assertEqual(T.public_item(it)["display_value"], "")


class TestValueKinds(Sandbox):
    def test_classify(self):
        cases = [("3000u", "uint", False), ("45.0f", "float", False),
                 ("(-30)", "int", True), ("(+1.0f)", "float", True),
                 ("80", "int", False), ("UART_ID_RPI", "raw", False),
                 (".5f", "float", False), ("1e3", "raw", False)]
        for raw, kind, paren in cases:
            c = T.classify_value(raw)
            self.assertEqual((c["kind"], c["paren"]), (kind, paren), raw)

    def test_format_preserves_style(self):
        """用户不敲 u/f: 填数字, 按原类型/括号回写。"""
        cases = [("PLAIN_UINT", "3000", "3000u"),
                 ("PLAIN_FLOAT", "45", "45.0f"),
                 ("PAREN_INT", "-30", "(-30)"),
                 ("PAREN_FLOAT", "1", "(+1.0f)"),
                 ("BARE_INT", "80", "80"),
                 ("SYMBOLIC", "UART_ID_BT", "UART_ID_BT")]
        for name, typed, expect in cases:
            self.assertEqual(T.from_display(self.item(name), typed), expect, name)

    def test_float_style_preserved(self):
        self.assertEqual(T.from_display(self.item("PLAIN_FLOAT"), "10.5"), "10.5f")
        self.assertEqual(T.from_display(self.item("PLAIN_FLOAT"), "150"), "150.0f")

    def test_negative_rejected_for_unsigned(self):
        with self.assertRaises(ValueError):
            T.from_display(self.item("PLAIN_UINT"), "-5")

    def test_garbage_rejected(self):
        with self.assertRaises(ValueError):
            T.from_display(self.item("PLAIN_UINT"), "abc")

    def test_paren_int_keeps_parens_after_edit(self):
        line = T._rebuild_line(self.item("PAREN_INT"),
                               value_text=T.from_display(self.item("PAREN_INT"), "-45"))
        self.assertIn("(-45)", line)
        self.assertIn("/* 回退速度 [rpm] // 负值=后退 */", line)


class TestLiteralTolerance(Sandbox):
    """回归: 输入框早期直接显示存储字面量（300u/45.0f/(-30)），用户连后缀一起改时
    曾抛 ValueError → 参数被静默跳过 → 界面重载后看着像「改动被复原」。
    现在展示层只给裸数字, 解析层再兜一层, 两种写法都必须收。"""

    def test_display_value_is_bare_number(self):
        self.assertEqual(T.public_item(self.item("PLAIN_UINT"))["display_value"], 3000)
        self.assertEqual(T.public_item(self.item("PLAIN_FLOAT"))["display_value"], 45)
        self.assertEqual(T.public_item(self.item("PAREN_INT"))["display_value"], -30)
        self.assertEqual(T.public_item(self.item("PAREN_FLOAT"))["display_value"], 1)
        self.assertEqual(T.public_item(self.item("BARE_INT"))["display_value"], 80)

    def test_symbolic_still_text(self):
        self.assertEqual(T.public_item(self.item("SYMBOLIC"))["display_value"], "UART_ID_RPI")

    def test_suffix_and_parens_accepted(self):
        cases = [("PLAIN_UINT", "300u", "300u"),
                 ("PLAIN_UINT", "300U", "300u"),
                 ("PLAIN_FLOAT", "45.0f", "45.0f"),
                 ("PLAIN_FLOAT", "90F", "90.0f"),
                 ("PAREN_INT", "(-30)", "(-30)"),
                 ("PAREN_FLOAT", "(+1.0f)", "(+1.0f)"),
                 ("BARE_INT", "80", "80")]
        for name, typed, expect in cases:
            self.assertEqual(T.from_display(self.item(name), typed), expect, typed)

    def test_display_roundtrips_without_spurious_change(self):
        """把界面显示的值原样存回去, 必须判为「没变」而不写盘。"""
        for name in ("PLAIN_UINT", "PLAIN_FLOAT", "PAREN_INT", "PAREN_FLOAT",
                     "BARE_INT", "PERIOD_MS", "DUTY"):
            it = self.item(name)
            shown = T.public_item(it)["display_value"]
            self.assertEqual(T.from_display(it, shown), it["value"],
                             "%s: 显示值 %r 存回来变成了别的" % (name, shown))

    def test_converted_rounding_is_not_a_change(self):
        """换算来回取整（33.3Hz → 30ms）第二次存不该再写一遍。"""
        ok, _, _ = T.write_config({"PERIOD_MS": "33.3"}, None)
        self.assertTrue(ok)
        before = self.reload()
        self.assertIn("30u", before)
        T.write_config({"PERIOD_MS": "33.3"}, None)
        self.assertEqual(self.reload(), before, "同值换算结果不该再写一次")

    def test_rejected_value_is_reported_as_unsaved(self):
        ok, resp, _ = T.write_config({"PLAIN_UINT": "abc"}, None)
        self.assertTrue(ok)
        w = [x for x in resp["warnings"] if x["name"] == "PLAIN_UINT"]
        self.assertEqual(len(w), 1)
        self.assertFalse(w[0]["saved"], "被拒的参数必须标 saved=False, 前端才会把值留在框里")

    def test_range_warning_is_marked_saved(self):
        ok, resp, _ = T.write_config({"PLAIN_FLOAT": "999"}, None)
        w = [x for x in resp["warnings"] if x["name"] == "PLAIN_FLOAT"]
        self.assertTrue(w and w[0]["saved"], "超范围是写进去了的, 不该标 saved=False")

    def test_warnings_are_structured(self):
        ok, resp, _ = T.write_config({"NOPE": "1", "PLAIN_UINT": "abc"}, None)
        for w in resp["warnings"]:
            self.assertEqual(set(w), {"name", "msg", "saved"}, w)
            self.assertIsInstance(w["saved"], bool)


class TestConversion(Sandbox):
    def test_ms_to_hz(self):
        it = self.item("PERIOD_MS")
        self.assertEqual(T.to_display(it), 50)                 # 20ms → 50Hz
        self.assertEqual(T.from_display(it, "50"), "20u")      # 50Hz → 20ms
        self.assertEqual(T.from_display(it, "40"), "25u")

    def test_permille_to_percent(self):
        it = self.item("DUTY")
        self.assertEqual(T.to_display(it), 50)                 # 500‰ → 50%
        self.assertEqual(T.from_display(it, "50"), "500u")
        self.assertEqual(T.from_display(it, "15"), "150u")     # 旧值 15% 能填回来

    def test_zero_display_rejected_not_written(self):
        """ms->Hz 在 0 处会除零 —— 必须拒掉, 不能写进去一个坏值。"""
        it = self.item("PERIOD_MS")
        with self.assertRaises(ValueError):
            T.from_display(it, "0")
        before = self.reload()
        ok, resp, _ = T.write_config({"PERIOD_MS": "0"}, None)
        self.assertTrue(ok)
        self.assertEqual(self.reload(), before, "非法值不该产生任何改动")
        self.assertTrue(any(w["name"] == "PERIOD_MS" for w in resp["warnings"]), resp)

    def test_stored_zero_gives_conv_error_not_crash(self):
        T.write_config({"PERIOD_MS": "40"}, None)              # 先存 25u 再手改坏
        self.cfg.write_text(self.reload().replace("25u", "0u"),
                            encoding="utf-8", newline="")
        pub = T.public_item(self.item("PERIOD_MS"))
        self.assertIsNotNone(pub["conv_error"])


class TestRange(Sandbox):
    def test_in_range_no_warning(self):
        self.assertIsNone(T.check_range(self.item("PLAIN_FLOAT")["meta"], 45))

    def test_out_of_range_warns(self):
        self.assertIn("高于上限", T.check_range(self.item("PLAIN_FLOAT")["meta"], 200))
        self.assertIn("低于下限", T.check_range(self.item("PLAIN_FLOAT")["meta"], -1))

    def test_no_range_no_warning(self):
        self.assertIsNone(T.check_range(self.item("PAREN_INT")["meta"], 99999))

    def test_out_of_range_still_saves(self):
        """超范围只警告, 不拦保存（用户要的是黄框, 不是阻断）。"""
        ok, resp, _ = T.write_config({"PLAIN_FLOAT": "999"}, None)
        self.assertTrue(ok)
        self.assertIn("999.0f", self.reload())
        self.assertTrue(any("超出范围" in w["msg"] for w in resp["warnings"]), resp)


class TestWrite(Sandbox):
    def test_backup_created(self):
        T.write_config({"PLAIN_FLOAT": "90"}, None)
        self.assertEqual((self.tmp / "app_config.h.bak").read_text(encoding="utf-8"),
                         FIXTURE)

    def test_only_target_line_changes(self):
        T.write_config({"PLAIN_FLOAT": "90"}, None)
        diff = [(a, b) for a, b in zip(FIXTURE.split("\n"), self.reload().split("\n"))
                if a != b]
        self.assertEqual(len(diff), 1, diff)
        self.assertIn("45.0f", diff[0][0])
        self.assertIn("90.0f", diff[0][1])

    def test_conflict_on_stale_version(self):
        ok, resp, st = T.write_config({"PLAIN_FLOAT": "90"}, None, base_version=1.0)
        self.assertFalse(ok)
        self.assertEqual(st, 409)
        self.assertEqual(self.reload(), FIXTURE)

    def test_unknown_param_skipped(self):
        ok, resp, _ = T.write_config({"NOPE": "1"}, None)
        self.assertTrue(ok)
        self.assertEqual(self.reload(), FIXTURE)

    def test_new_param_into_existing_section(self):
        ok, resp, _ = T.write_config({}, [{
            "name": "MY_TIMEOUT", "value": "500u", "kind": "uint",
            "meta": T.parse_dsl("我的超时 [ms] @(0~) // 新加的"),
            "target": "A 组"}] + [], None)
        self.assertTrue(ok, resp)
        lines = self.reload().split("\n")
        i = next(n for n, l in enumerate(lines) if "MY_TIMEOUT" in l)
        # 落在 A 组最后一个 #define(NOCMT) 之后、B 组标题之前
        self.assertLess(i, next(n for n, l in enumerate(lines) if "B 组" in l))
        self.assertGreater(i, next(n for n, l in enumerate(lines) if "NOCMT" in l))
        self.assertIn("500u", lines[i])
        self.assertIn("我的超时 [ms] @(0~) // 新加的", lines[i])

    def test_new_param_new_section(self):
        T.write_config({}, [{"name": "FRESH", "value": "7", "kind": "int",
                             "meta": T.parse_dsl("新旋钮"), "new_section": "调试新增"}])
        text = self.reload()
        self.assertIn("==================== 调试新增 ====================", text)
        self.assertIn("#define FRESH", text)
        self.assertLess(text.index("FRESH"), text.index("#endif"))

    def test_write_meta_only_touches_comment(self):
        T.write_meta("PLAIN_FLOAT", T.parse_dsl("新名字 [°] @(0~180) // 新备注"))
        line = next(l for l in self.reload().split("\n") if "PLAIN_FLOAT" in l)
        self.assertIn("45.0f", line, "值不该被动")
        self.assertIn("新名字 [°] @(0~180) // 新备注", line)
        other = next(l for l in self.reload().split("\n") if "PAREN_INT" in l)
        self.assertIn("负值=后退", other, "别的行不该被动")

    def test_write_meta_unknown(self):
        ok, resp, st = T.write_meta("NOPE", T.parse_dsl("x"))
        self.assertFalse(ok)
        self.assertEqual(st, 404)

    def test_duplicate_new_section_writes_one_header(self):
        """同一分区名的新参数无论来几个, 分区头只写一次。

        ⚠️ 靠的是 seen 集合, 不是「在 `for p in fresh` 里重新赋值 fresh」—— 后者绑不上:
           `for` 的迭代器在循环开始时就指向原列表, 重新赋值那个**名字**不影响迭代,
           于是第二个同分区参数会再写一个空分区头。
        """
        T.write_config({}, [
            {"name": "F1", "value": "1", "kind": "int",
             "meta": T.parse_dsl("一"), "new_section": "同一个新分区"},
            {"name": "F2", "value": "2", "kind": "int",
             "meta": T.parse_dsl("二"), "new_section": "同一个新分区"},
        ], None)
        text = self.reload()
        self.assertEqual(text.count("==================== 同一个新分区 ===================="), 1,
                         "同一个新分区写了多个分区头")
        self.assertIn("#define F1", text)
        self.assertIn("#define F2", text)
        # 两个参数都要落在那个分区头**之后**（不能被甩到别处）
        head = text.index("同一个新分区")
        self.assertGreater(text.index("#define F1"), head)
        self.assertGreater(text.index("#define F2"), head)

    def test_write_meta_takes_the_write_lock(self):
        """write_meta 与 write_config **写同一个文件**, 必须共用 WRITE_LOCK。

        ⚠️ 它是「读文件 → 改一行 → 整文件写回」: 一个客户端改值、另一个改注释时,
           不加锁会交错写 app_config.h（轻则丢一次更新, 重则写坏文件, 还各自覆盖 .bak）。
        """
        T.WRITE_LOCK.acquire()                       # 冒充「另一个客户端正在保存」
        t = threading.Thread(target=T.write_meta,
                             args=("PLAIN_FLOAT", T.parse_dsl("抢锁测试")))
        t.start()
        t.join(0.3)
        self.assertTrue(t.is_alive(), "write_meta 没拿 WRITE_LOCK —— 与 write_config 会交错写盘")
        T.WRITE_LOCK.release()
        t.join(2.0)
        self.assertFalse(t.is_alive(), "锁释放后 write_meta 没能完成")
        self.assertIn("抢锁测试",
                      next(l for l in self.reload().split("\n") if "PLAIN_FLOAT" in l))


class TestPrefs(Sandbox):
    def test_save_load(self):
        T.save_prefs({"visible": {"A": True}, "order": {"A 组": ["A", "B"]}})
        p = T.load_prefs()
        self.assertEqual(p["visible"], {"A": True})
        self.assertEqual(p["order"], {"A 组": ["A", "B"]})
        self.assertFalse((self.tmp / "tuning.json.tmp").exists())

    def test_missing_is_empty(self):
        self.assertEqual(T.load_prefs(), {"visible": {}, "order": {}})

    def test_default_visible_only_numeric(self):
        self.assertTrue(T.default_visible(self.item("PLAIN_UINT")))
        self.assertFalse(T.default_visible(self.item("SYMBOLIC")),
                         "符号/表达式默认不显示")


class TestRealFile(unittest.TestCase):
    """对真实的 app_config.h 跑只读检查（不写盘）。"""

    def test_parses_and_rebuilds(self):
        items = [it for g in T.parse_config()["groups"] for it in g["items"]]
        self.assertGreater(len(items), 40)
        for it in items:
            self.assertEqual(T._rebuild_line(it), it["line"], it["name"])

    def test_every_param_has_display_name(self):
        """规整过的文件里, 每个参数都该有显示名（注释即参数名）。"""
        missing = [it["name"] for g in T.parse_config()["groups"]
                   for it in g["items"] if not it["has_comment"]]
        self.assertEqual(missing, [], "这些宏还没有 DSL 注释: %s" % missing)

    def test_arm_servo_speed_cap_is_tunable(self):
        """舵机速度总闸必须在「调参」Tab 里可调 —— 它被现场反复调（「舵机太快了」）。

        这条只断言**元数据还在**（显示名/单位/范围），不断言数值: 那是个随时会被拧的旋钮。
        """
        # 用 state()（面向界面的规整结果）而不是 parse_config()（行级原始字段）
        by = {it["name"]: it
              for g in T.state()["groups"] for it in g["items"]}
        it = by.get("ARM_SERVO_MAX_SPEED_DPS")
        self.assertIsNotNone(it, "舵机速度总闸从 app_config.h 里消失了？")
        self.assertTrue(it["has_comment"], "少了尾注释就进不了调参 Tab")
        self.assertEqual(it["display"], "舵机最高转速")
        self.assertIn("/s", it["unit"])
        self.assertIsNotNone(it["range"])

    def test_known_conversions(self):
        """断言换算**关系**而不是具体数值 —— 这些是可调的旋钮, 值随时会变。"""
        by = {it["name"]: it for g in T.parse_config()["groups"] for it in g["items"]}

        period = by["LINE_LOOP_PERIOD_MS"]
        self.assertEqual(T.to_display(period), 1000.0 / T.to_number(period))   # ms ↔ Hz
        self.assertEqual(T.from_display(period, T.to_display(period)), period["value"])

        duty = by["FILL_LIGHT_ON_DUTY"]
        self.assertEqual(T.to_display(duty), T.to_number(duty) / 10.0)         # ‰ ↔ %
        self.assertEqual(T.from_display(duty, T.to_display(duty)), duty["value"])

    def test_priority_range_is_present(self):
        by = {it["name"]: it for g in T.parse_config()["groups"] for it in g["items"]}
        for name in ("APP_PRIO_OLED", "APP_PRIO_KEY", "APP_PRIO_SCRIPT", "APP_PRIO_PID",
                     "APP_PRIO_ARM", "APP_PRIO_MOTOR", "APP_PRIO_COMM", "APP_PRIO_SENSOR"):
            self.assertEqual(by[name]["meta"]["range"], {"lo": "1", "hi": "6"}, name)

    def test_no_unknown_conversions(self):
        """每个参数的 `{...}` 都必须是 CONVERSIONS 里认识的**换算**。

        ⚠️ 这条是 2026-09 真踩出来的: `{值=标签}`（下拉选项）是 **app_script.h 参数映射 DSL**
        的语法, 不是 app_config.h 的 —— 在这个文件里 `{}` 只表示**换算**, 而换算是**闭集**
        （只有 ms->Hz 与 ‰->%）。在一个布尔宏上照抄 `{0=不对调,1=对调}` 的后果不是显示难看,
        而是**这个参数在「调参」Tab 里根本存不进去**: 写盘会抛「未知转换」,
        warnings 里带 `saved: false`, 用户只看到一句 ⚠ 而值没落盘。
        布尔开关的正确写法是 `@(0~1)` + 备注里写 `0=... 1=...`（看 BALL_XY_SWAP / HOSTAGE_XY_SWAP）。
        """
        bad = [(it["name"], it["meta"].get("conv"))
               for g in T.parse_config()["groups"] for it in g["items"]
               if (it["meta"].get("conv") or "") not in ("", *T.CONVERSIONS)]
        self.assertEqual(bad, [], "这些宏用了不存在的换算: %s" % bad)

    def test_line_conv_src_is_a_global_four_way_choice(self):
        """2026-10-30: 收敛航向来源提升为全局调参宏。

        LINE_CONV_SRC 是 0~3 的四选一（备注里 `选项:` 派生下拉，与 app_rpi.h 的
        LINE_ANG_SRC_* 对应）。LINE_CAM_PCT 只钉**存在与量程**、不钉具体值 ——
        它是现场可调量（出厂 50, 2026-11-01 现场调到 40）, 钉值就会跟标定流程打架。
        """
        by = {it["name"]: it for g in T.state()["groups"] for it in g["items"]}
        it = by["LINE_CONV_SRC"]
        self.assertEqual(it["num"], 3.0)
        self.assertEqual(it["range"], {"lo": "0", "hi": "3"})
        self.assertEqual([o["value"] for o in it["options"]], ["0", "1", "2", "3"])
        self.assertIn("相机", it["options"][0]["label"])
        self.assertIn("融合", it["options"][3]["label"])
        self.assertIn("LINE_CAM_PCT", by)
        self.assertEqual(by["LINE_CAM_PCT"]["range"], {"lo": "0", "hi": "100"})


class TestRealFileRoundTripsThroughTheWritePath(unittest.TestCase):
    """把真实 app_config.h 复制出来, **逐个参数回写它自己的值** —— 必须零警告、零 diff。

    ⚠️ 为什么不能只测「无操作保存」: 那种调用 `write_config({}, [])` 什么都不改,
    根本不会走到 `from_display`（换算/类型检查都在那里）, 于是「未知转换」这类
    **只在写盘路径上炸**的错全都能溜过去 —— 2026-09 就是这么漏掉两个布尔宏的。
    这里对**每一个**参数都走一遍真实写盘路径, 把那条路也覆盖上。

    （仍然不碰仓库里的真实文件: 改的是 tempfile 里的副本。）
    """

    def test_every_param_writes_back_unchanged(self):
        tmp = Path(tempfile.mkdtemp(prefix="gks_tuning_write_"))
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        cfg = tmp / "app_config.h"
        shutil.copyfile(T.APP_CONFIG, cfg)

        with mock.patch.object(T, "APP_CONFIG", cfg), \
             mock.patch.object(T, "PREFS_FILE", tmp / "tuning.json"):
            before = cfg.read_bytes()
            parsed = T.parse_config()

            # 页面会把 display_value 原样回传 —— 这里逐参数模拟这一次回传
            changes = {}
            for g in parsed["groups"]:
                for it in g["items"]:
                    pi = T.public_item(it)
                    self.assertIsNone(pi["conv_error"],
                                      "%s 的换算算不出来: %s" % (it["name"], pi["conv_error"]))
                    changes[it["name"]] = pi["display_value"]

            self.assertGreater(len(changes), 40)
            ok, resp, status = T.write_config(changes, [])
            warned = [(w.get("name"), w.get("msg"))
                      for w in resp.get("warnings", []) if not w.get("saved")]
            self.assertEqual(warned, [], "这些参数回写被拒: %s" % warned)
            self.assertTrue(ok, "write_config 失败: %s" % resp)
            self.assertEqual(cfg.read_bytes(), before,
                             "逐参数原值回写必须字节级零 diff")


class TestDeleteParams(Sandbox):
    """delete_params: 「位移」Tab 删变量走的路径（引用检查是 moves.py 的事, 这里只管删行）。"""

    def test_delete_removes_whole_lines_and_keeps_rest(self):
        """删掉的行连 DSL 尾注释一起消失; 其余行（含对齐风格）一个字节不动。"""
        ok, resp, st = T.delete_params(["PLAIN_FLOAT", "PAREN_INT"], base_version=T.version())
        self.assertTrue(ok, resp)
        text = self.reload()
        self.assertNotIn("PLAIN_FLOAT", text)
        self.assertNotIn("PAREN_INT", text)
        self.assertIn("#define PLAIN_UINT      3000u       /* 超时 [ms] @(0~) // 说明文字 */", text)
        self.assertIn("#define SYMBOLIC        UART_ID_RPI", text)
        self.assertEqual(sorted(resp["removed"]), ["PAREN_INT", "PLAIN_FLOAT"])

    def test_delete_reports_missing_names(self):
        """在名单里的删掉、不在的报 missing —— 名单是「要删谁」, 不是「要留谁」。"""
        ok, resp, st = T.delete_params(["PLAIN_UINT", "NOT_THERE"])
        self.assertTrue(ok, resp)
        self.assertEqual(resp["removed"], ["PLAIN_UINT"])
        self.assertEqual(resp["missing"], ["NOT_THERE"])
        self.assertNotIn("#define PLAIN_UINT", self.reload())
        self.assertIn("#define PLAIN_FLOAT", self.reload())    # 名单外的没被殃及

    def test_delete_all_missing_is_404(self):
        ok, resp, st = T.delete_params(["NOT_THERE"])
        self.assertFalse(ok)
        self.assertEqual(st, 404)

    def test_delete_conflict_on_stale_version(self):
        ok, resp, st = T.delete_params(["PLAIN_UINT"], base_version=1.0)
        self.assertFalse(ok)
        self.assertEqual(st, 409)
        self.assertIn("#define PLAIN_UINT", self.reload())


class TestEmptySection(Sandbox):
    """往「只有段头、还没有宏」的空分区追加参数 —— 2026-10-30 修: 原来走
    g["items"][-1] 直接 IndexError（「自定义开关」段建好后的第一条 add 真踩）。"""

    def test_add_into_empty_section(self):
        ok, resp, st = T.write_config(None, [{
            "name": "NEW_ONE", "value": "1", "kind": "int",
            "meta": {"display": "新参数", "range": {"lo": "0", "hi": "1"}},
            "target": "B 组"}])
        self.assertTrue(ok, resp)
        text = self.reload()
        self.assertIn("#define NEW_ONE", text)
        # 落点必须在「B 组」段头之后、该组第一个宏之前（而不是文件别处）
        self.assertGreater(text.find("NEW_ONE"), text.find("B 组"))
        self.assertLess(text.find("NEW_ONE"), text.find("#endif"))




if __name__ == "__main__":
    unittest.main(verbosity=2)
