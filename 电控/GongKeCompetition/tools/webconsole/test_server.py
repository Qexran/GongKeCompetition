#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""server.py 里纯函数部分的测试（不启服务、不落盘）。

    python tools/webconsole/test_server.py
"""

import io
import json
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_action                                                    # noqa: E402
import calib                                                         # noqa: E402
import host_tools                                                    # noqa: E402
import map_codec
import moves                                                     # noqa: E402
import switches                                                  # noqa: E402
import oled_menu                                                     # noqa: E402
import server                                                        # noqa: E402
import srccheck                                                      # noqa: E402
import tuning                                                        # noqa: E402

# 不启监听地借 Handler 的方法做单元测试（_catalog / do_POST）—— 与 test_bt.py 里
# `S.Handler.__new__(S.Handler)` 同一个套路。
Handler = server.Handler


class TestParamValueNormalization(unittest.TestCase):
    """参数值比较必须忽略整数后缀 `u`（以及浮点槽的 `.0f`）。

    写盘端 `_normalize_param` 会给非负整数补 `u`（`1` → `1u`），而下拉选项的 value 是
    DSL 里的裸值（`1`/`0`）—— 不归一化就会出现「值明明在枚举里，下拉却显示 `1u  (枚举外)`」
    （用户现场报的正是这句; 转向/开合/使能 这类数字选项全中）。
    """

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def test_optkey_ignores_suffixes(self):
        out = self._js(
            "console.log('K=' + [optKey('1u'), optKey('1'), optKey('0U'), optKey(' 2u '), "
            "optKey('90.0f'), optKey('90'), optKey(''), optKey(undefined), "
            "optKey('0.5')].join('|'));")
        self.assertIn("K=1|1|0|2|90|90|||0.5", out)

    def test_param_input_compares_via_optkey(self):
        """两处比对（是否有枚举外项 / 哪个 option 选中）都必须走 optKey。"""
        js = srccheck.read_source("static/app.js")
        self.assertEqual(js.count("optKey(o.value) === optKey(cur)"), 2,
                         "paramInput 里两处选项比对都要用 optKey 归一化")


class TestParamShowIf(unittest.TestCase):
    """参数显隐（DSL 尾部 `<显隐:参数名=值>`）—— 只在同一积木的另一个参数读到该值时显示。

    守两条不变量，它们各自对应一个真实的坑:
      · **元数据保存不许把标记弄丢**: /api/meta 是整行重写参数注释, 服务端能重新推导
        tip/enum_type, 但推不出显隐条件 —— 靠前端 saveMetaEditor 原样透传。
      · **前端不许硬编码参数名**: 条件来自 app_script.h 的注释行, 机制必须通用。
    """

    SPEC = "gyro_w=陀螺仪信任占比[%]<显隐:use_fusion=1>"

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def _fn_body(self, name):
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function %s\([^)]*\)[\s\S]*?\n\}\n" % re.escape(name), js)
        self.assertIsNotNone(m, "没在 app.js 里找到 function %s" % name)
        return m.group(0)

    # ---- DSL 层（纯函数） ----
    def test_marker_is_stripped_before_options_and_unit(self):
        """标记必须**最先**剥：选项/单位两个正则都是 $ 锚定的，`=` 那次 _split_top 也认不出它。"""
        m = server._parse_param_spec(self.SPEC, "")
        self.assertEqual(m["show_if"], {"name": "use_fusion", "value": "1"})
        self.assertEqual(m["name"], "gyro_w")
        self.assertEqual(m["raw"], "gyro_w")
        self.assertEqual(m["label"], "陀螺仪信任占比")
        self.assertEqual(m["unit"], "%")
        self.assertIsNone(m["options"])
        self.assertIsNone(server._bool_options_from_tip(m["tip"]))

    def test_spec_str_reproduces_the_marker(self):
        for spec in (self.SPEC,
                     "gyro_w=陀螺仪信任占比[%]{0=关,1=开}<显隐:use_fusion=1>",
                     "mode(PID_MODE_*)=模式[%]<显隐:use_fusion=1>"):
            meta = server._parse_param_spec(spec, "")
            out = server._spec_str(meta)
            self.assertEqual(out, spec, "解析→回写必须逐字节相同: %s" % spec)
            self.assertEqual(out.count("<显隐:"), 1, "标记只许追加一次: %s" % out)

    def test_parse_format_is_a_fixed_point(self):
        """不动点 = 「再保存一次是 no-op」的充分条件。"""
        for spec in (self.SPEC, "mode(PID_MODE_*)=模式[%]<显隐:use_fusion=1>"):
            once = server._spec_str(server._parse_param_spec(spec, ""))
            twice = server._spec_str(server._parse_param_spec(once, ""))
            self.assertEqual(twice, once)

    def test_marker_value_rejects_separators(self):
        """值里带 , ; < → 整条拒识, 且**逐字节无损**（那些字符是 DSL 的顶层切分符）。"""
        for spec in ("gyro_w=占比[%]<显隐:a=1,2>", "gyro_w=占比[%]<显隐:a=1;2>",
                     "gyro_w=占比[%]<显隐:a=<1>"):
            meta = server._parse_param_spec(spec, "")
            self.assertNotIn("show_if", meta, spec)
            self.assertEqual(server._spec_str(meta), spec, "拒识也要无损: %s" % spec)

    def test_malformed_marker_is_lossless(self):
        """缺 `>` 的残标记: 不报错、不丢字符（显示成原始文本, 但绝不 500）。"""
        spec = "gyro_w=占比[%]<显隐:use_fusion=1"
        meta = server._parse_param_spec(spec, "")
        self.assertNotIn("show_if", meta)
        self.assertEqual(server._spec_str(meta), spec)

    # ---- 编目层 ----
    def test_line_converge_param_layout(self):
        """巡线收敛的参数布局 = C 侧槽位契约（float 在前）。

        2026-10-30: `src`(航向来源) / `cam_pct`(融合相机占比) 提升为**全局调参宏**
        （app_config.h 的 LINE_CONV_SRC / LINE_CAM_PCT）→ 本积木只剩
        F(0)=angle + I(0)=converge_ms, I(1)=early_ret（F1I2）。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}.get("STEP_LINE_CONVERGE_WAIT")
        if cat is None:
            self.skipTest("catalog 里没有 STEP_LINE_CONVERGE_WAIT")
        self.assertEqual(cat["params_f"], ["angle"],
                         "float 槽在最前: app_script.c 里是 F(0)")
        self.assertEqual(cat["params_i"], ["converge_ms", "early_ret"],
                         "int 槽随后: I(0)=converge_ms, I(1)=early_ret")
        self.assertEqual(cat["params_meta"][2]["unit"], "")
        self.assertNotIn("show_if", cat["params_meta"][2], "显隐机制目前一处都不用")

    def test_early_ret_slot_is_a_yes_no_dropdown(self):
        """early_ret 必须是**是/否下拉**（2026-10 加的「启动即返回」）。

        2026-10-30 src/cam_pct 移除后它是最后一个 int（I(1)）; 老形状的 2 参数行
        由 _serialize_step 把缺的尾槽补 0 = 否 = 老行为。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        if "STEP_LINE_CONVERGE_WAIT" not in cat:
            self.skipTest("catalog 里没有 STEP_LINE_CONVERGE_WAIT")
        meta = cat["STEP_LINE_CONVERGE_WAIT"]["params_meta"][2]
        self.assertEqual(meta["name"], "early_ret")
        self.assertEqual([o["value"] for o in meta.get("options", [])], ["1", "0"],
                         "DSL 的 {1=是,0=否}: 勾'是'才启用达标提前返回")
        line = map_codec._serialize_step(
            {"step": "STEP_LINE_CONVERGE_WAIT", "tag": "巡线收敛",
             "params": ["0.0f", "1000", "1"]}, cat)[0]
        self.assertIn("SCRIPT_STEP_F1I2", line)
        self.assertIn("0.0f, 1000u, 1u)", line,
                      "1 float + 2 int → F1I2（angle / converge_ms / early_ret）")
        # 落进 map.c 之后必须能原样解析回来 —— 控制台「改个参数 → 保存」走的就是这条路。
        self.assertEqual(server.serialize_map(server._parse_body([line]), cat), [line])

    def test_line_source_and_cam_pct_are_global_not_step_params(self):
        """src / cam_pct 已提升为全局调参宏（2026-10-30）。

        六个用到巡线收敛的积木都不再带这两个参数; 全局量在 app_config.h:
        `LINE_CONV_SRC`（0相机/1陀螺/2不修/3融合）与 `LINE_CAM_PCT`。
        调参 Tab 侧的契约由 test_tuning.py 钉。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        for step in ("STEP_LINE_CONVERGE_WAIT", "STEP_LINE_STRAIGHT_WAIT",
                     "STEP_PIVOT_LINE_WAIT", "STEP_SPIN_LINE_WAIT",
                     "STEP_BALL_GRAB_ONE_WAIT", "STEP_HOSTAGE_GRAB_ONE_WAIT"):
            entry = cat.get(step)
            if entry is None:
                continue
            self.assertNotIn("src", entry["params_i"], step)
            self.assertNotIn("cam_pct", entry["params_i"], step)

    def test_float_plus_two_int_params_pick_the_f1i2_macro(self):
        """1 个 float + 2 个 int → SCRIPT_STEP_F1I2（宏名上限与真实宏表失配时 Keil 才报 #55-D）。

        ⚠️ 载体曾经是巡线收敛（它 2026-10 加了 early_ret 尾槽, 已是 F1I3, 见上面
        test_early_ret_slot_is_a_yes_no_dropdown）—— 换成原地转弯的 dir/absolute,
        同样恰好 1 float + 2 int。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        line = map_codec._serialize_step(
            {"step": "STEP_SPIN_TURN_WAIT", "tag": "原地转弯",
             "params": ["0.0f", "1", "0"]}, cat)[0]
        self.assertIn("SCRIPT_STEP_F1I2", line)
        self.assertIn("0.0f, 1u, 0u", line, "float 在前、int 在后 —— 槽位顺序就是契约")
        # 落进 map.c 之后必须能原样解析回来 —— 控制台「改个参数 → 保存」走的就是这条路,
        # 再点一次保存必须零 diff（否则每存一次就多一行 diff, 谁也说不清哪来的）。
        self.assertEqual(server.serialize_map(server._parse_body([line]), cat), [line])

    def test_action_params_after_a_float_slot_are_dropdowns(self):
        """F1I 积木的 int 动作参数必须派生下拉 —— casts 下标要先减掉 float 槽数。

        回归: STEP_BALL_GRAB_ONE_WAIT 的槽位是 F(0)=gyro_target + I(0..7), 动作在
        I(2)=grab / I(3)=place。parse_catalog 早先拿 `I(n)` 的 n 直接当扁平下标,
        本积木带 1 个 float → 整体后移一位: grab 借 place 的转换显示成下拉,
        **place 没有选项、退化成数字填空**（用户报的就是这个）。
        同一条也覆盖人质单动作抓取的 grab/place（同样是 float 后的 int 槽）。

        2026-11: 两个积木的 place 都带 DSL `<可空>` → 选项里必须有 0=无, 而 grab
        不许有（抓取动作不能是「无」）。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        for step, names in (("STEP_BALL_GRAB_ONE_WAIT", ("grab", "place")),
                            ("STEP_HOSTAGE_GRAB_ONE_WAIT", ("grab", "place"))):
            entry = cat.get(step)
            if entry is None:
                self.skipTest("catalog 里没有 %s" % step)
            metas = {m["name"]: m for m in entry["params_meta"]}
            for name in names:
                meta = metas[name]
                self.assertEqual(meta.get("enum_type"), "emArmActionIdTdf",
                                 "%s.%s 应由 (emArmActionIdTdf)I(n) 派生成动作下拉" %
                                 (step, name))
                self.assertTrue(meta.get("options"),
                                "%s.%s 的 action 下拉不能是空的" % (step, name))
                self.assertTrue(any(str(o["value"]).startswith("ARM_ACT_")
                                    for o in meta["options"]),
                                "%s.%s 的选项必须来自 emArmActionIdTdf" % (step, name))
            place = metas["place"]
            self.assertEqual(place["options"][0], {"value": "0", "label": "无"},
                             "%s.place 的「无」必须在选项首位（新积木默认不放置）" % step)
            enum_opts = server.parse_enums()["emArmActionIdTdf"]
            self.assertEqual(place["options"][1:],
                             [dict(o) for o in enum_opts],
                             "「无」是追加项, 枚举选项原样跟在后面")
            self.assertTrue(place.get("allow_none"), "place 带 <可空> 标记")
            self.assertNotIn("allow_none", metas["grab"],
                             "抓取动作不带 <可空> —— 下拉里不能出现 0=无")
            self.assertFalse(any(str(o["value"]) == "0" for o in metas["grab"]["options"]),
                             "%s.grab 不应有 0 选项" % step)

    def test_three_int_params_pick_the_i3_macro(self):
        """3 个 int → SCRIPT_STEP_I3（用别的三 int 积木当被测对象: 巡线收敛已是 F1I2）。

        ⚠️ 载体曾经是 STEP_IF_ELSE（它已随嵌套容器 IF 一起删掉）。换成反恐任务的三个动作号 ——
           同样恰好 3 个 int 槽, 而且它是**枚举型**参数, 顺带把「枚举值原样回写」也钉住了。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        line = map_codec._serialize_step(
            {"step": "STEP_HOSTAGE_RESCUE_WAIT", "tag": "反恐任务",
             "params": ["ARM_ACT_8", "ARM_ACT_7", "ARM_ACT_6"]}, cat)[0]
        self.assertIn("SCRIPT_STEP_I3", line)
        self.assertIn("ARM_ACT_8, ARM_ACT_7, ARM_ACT_6", line)
        self.assertEqual(server.serialize_map(server._parse_body([line]), cat), [line])

    def test_no_param_uses_a_condition_anymore(self):
        """显隐标记**目前一处都不用**（原来只有巡线收敛的 gyro_w 用, 随 use_fusion 一起没了）。

        语法仍在（前端/解析器都还在测），但真文件里多出标记就说明有行被误改错了。
        """
        hits = [(c["type"], m["name"]) for c in server.parse_catalog()
                for m in c["params_meta"] if m.get("show_if")]
        self.assertEqual(hits, [], "真文件里不该再有 <显隐:...> 标记")

    # ---- 前端层 ----
    def test_param_visible_helper(self):
        out = self._js(
            "const pCat = { params_f: [], params_i: ['use_fusion','gyro_w'], params_meta: [\n"
            "  {name:'use_fusion', label:'是否使用传感器融合', unit:'', tip:'', "
            "options:[{value:'0',label:'关'},{value:'1',label:'开'}]},\n"
            "  {name:'gyro_w', label:'陀螺仪信任占比', unit:'%', tip:'', "
            "show_if:{name:'use_fusion', value:'1'}} ]};\n"
            "const pG = pCat.params_meta[1], pSw = pCat.params_meta[0];\n"
            "console.log('V=' + [\n"
            "  paramVisible({step:'X', params:['1','30']},  pG, pCat),\n"
            "  paramVisible({step:'X', params:['0','30']},  pG, pCat),\n"
            "  paramVisible({step:'X', params:['1u','30']}, pG, pCat),\n"
            "  paramVisible({step:'X', params:[]},          pG, pCat),\n"
            "  paramVisible({step:'X', params:['0','30']},  pSw, pCat),\n"
            "  paramVisible({step:'X', params:['1','30']},  {name:'x'}, pCat),\n"
            "].join('|'));")
        # 开→显示 / 关→隐藏 / 写盘补的 u 要归一化 / 老 map.c 槽位缺失→隐藏 / 无条件→恒显示 / 条件找不到→放行
        self.assertIn("V=true|false|true|false|true|true", out)

    def test_both_param_render_sites_call_the_helper(self):
        for fn in ("renderBlock", "renderInspector"):
            body = self._fn_body(fn)
            self.assertIn("paramVisible(it,", body, fn)
            self.assertIn("syncVis()", body, fn)
            self.assertIn("classList.toggle('hidden'", body,
                          "%s 只能就地切类名, 别改成重画（拖拽中重建 DOM 会丢控件）" % fn)

    def test_frontend_hardcodes_no_param_names(self):
        js = srccheck.read_source("static/app.js")
        self.assertNotIn("use_fusion", js, "显隐机制必须通用, 不许把本用例写死进前端")
        self.assertNotIn("gyro_w", js)

    def test_meta_editor_passes_show_if_through(self):
        """漏了这一行 = 改一次中文名就把显隐标记无声抹掉。"""
        self.assertIn("show_if", self._fn_body("saveMetaEditor"))


class TestParamAllowNone(unittest.TestCase):
    """「可空」标记（DSL 尾部 `<可空>`）—— 下拉在枚举前追加 0=无。

    2026-11 加：两个单动作抓取积木的 place 用它表达「不放置」（决赛换边）。
    守两条:
      · 解析时剥掉并置 allow_none（否则 `place=放置动作<可空>` 整个当参数名）;
      · 元数据保存必须原样透传（否则改一次中文名, 0=无 就消失）。
    """

    SPEC = "place=放置动作<可空>"

    def _fn_body(self, name):
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function %s\([^)]*\)[\s\S]*?\n\}\n" % re.escape(name), js)
        self.assertIsNotNone(m, "没在 app.js 里找到 function %s" % name)
        return m.group(0)

    def test_marker_is_stripped_and_round_trips(self):
        meta = server._parse_param_spec(self.SPEC, "")
        self.assertEqual(meta["name"], "place")
        self.assertEqual(meta["label"], "放置动作")
        self.assertTrue(meta.get("allow_none"))
        self.assertIsNone(meta.get("options"))
        out = server._spec_str(meta)
        self.assertEqual(out, self.SPEC, "解析→回写必须逐字节相同")
        self.assertEqual(out.count("<可空>"), 1, "标记只许追加一次")

    def test_parses_after_unit_and_options(self):
        """标记挂在行尾, 必须在 {选项}/[单位] 之前剥掉, 否则两个 $ 锚定的正则双双失配。"""
        meta = server._parse_param_spec("place=放置动作{0=无}<可空>", "")
        self.assertTrue(meta.get("allow_none"))
        self.assertEqual(meta["label"], "放置动作")
        self.assertEqual(meta["options"], [{"value": "0", "label": "无"}])

    def test_meta_editor_passes_allow_none_through(self):
        """漏了它 = 改一次中文名, 「放置动作」下拉里的 0=无 就没了。"""
        self.assertIn("allow_none", self._fn_body("saveMetaEditor"))

    def test_real_catalog_puts_none_first_only_for_place(self):
        cat = {c["type"]: c for c in server.parse_catalog()}
        for step in ("STEP_BALL_GRAB_ONE_WAIT", "STEP_HOSTAGE_GRAB_ONE_WAIT"):
            entry = cat.get(step)
            if entry is None:
                self.skipTest("catalog 里没有 %s" % step)
            metas = {m["name"]: m for m in entry["params_meta"]}
            self.assertTrue(metas["place"].get("allow_none"), step)
            self.assertEqual(metas["place"]["options"][0]["label"], "无", step)
            self.assertNotIn("allow_none", metas["grab"], step)


class AllowNoneWriteMetaSandbox(unittest.TestCase):
    """元数据保存（整行重写参数注释）不许把 `<可空>` 弄丢 —— 丢了下拉就没有「无」。

    与 WidgetWriteMetaSandbox 同一套沙箱: SCRIPT_H 指到临时副本。
    ⚠️ 保存后选项会以显式 `{...}` 写进 DSL（元数据编辑器一贯如此）, 所以这里只查
    标记与 0=无 仍在, 不要求整行字节相同。
    """

    STEP = "STEP_BALL_GRAB_ONE_WAIT"

    def setUp(self):
        if not server.SCRIPT_H.is_file():
            self.skipTest("app_script.h 不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="allownonesbx-"))
        self.h = self.tmp / "app_script.h"
        shutil.copy2(server.SCRIPT_H, self.h)
        p = mock.patch.object(server, "SCRIPT_H", self.h)
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def payload(self, with_allow_none=True):
        cat = {c["type"]: c for c in server.parse_catalog()}[self.STEP]
        params = []
        for m in cat["params_meta"]:
            d = {"name": m["name"], "raw": m.get("raw") or m["name"],
                 "label": m.get("label") or m["name"], "unit": m.get("unit") or "",
                 "options": m.get("options"), "show_if": m.get("show_if"),
                 "enable_if": m.get("enable_if"), "widget": m.get("widget")}
            if with_allow_none:
                d["allow_none"] = m.get("allow_none")
            params.append(d)
        return {"step": self.STEP, "params": params,
                "note": cat["note"], "n_f": len(cat["params_f"])}

    def _place_spec(self):
        line = next(ln for ln in self.h.read_text(encoding="utf-8").split("\n")
                    if ln.startswith(" *   %s " % self.STEP))
        spec = line.split("//", 1)[0]
        part = next(p for p in spec.split("anI: ", 1)[1].split(", ")
                    if p.strip().startswith("place="))
        return part.strip()

    def test_metadata_save_keeps_allow_none(self):
        ok, resp, _ = server.write_meta(self.payload())
        self.assertTrue(ok, resp)
        spec = self._place_spec()
        self.assertIn("<可空>", spec)
        self.assertEqual(spec.count("<可空>"), 1, "标记只许追加一次（否则每存一次多一个）")
        self.assertIn("0=无", spec)

    def test_payload_without_allow_none_would_drop_it(self):
        """负向对照: 这正是前端必须透传 allow_none 的理由。"""
        ok, resp, _ = server.write_meta(self.payload(with_allow_none=False))
        self.assertTrue(ok, resp)
        spec = self._place_spec()
        self.assertNotIn("<可空>", spec)


class ShowIfWriteMetaSandbox(unittest.TestCase):
    """元数据保存（/api/meta → write_meta）是**整行重写**参数注释。

    全程沙箱: 只把 SCRIPT_H 指到临时副本（SCRIPT_C 只读, 不碰）。
    ⚠️ 显隐标记是**测试自己塞进去的**（`MARK`）: 真文件里现在一处都不用（原来只有巡线收敛
    的 gyro_w 用, 随 use_fusion 一起删了）—— 但机制还在, 这条测试守的是机制本身,
    不该依赖"某个积木恰好还在用"。
    """

    MARK = "<显隐:converge_ms=1000>"
    STEP = "STEP_LINE_CONVERGE_WAIT"

    def setUp(self):
        if not server.SCRIPT_H.is_file():
            self.skipTest("app_script.h 不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="showifsbx-"))
        self.h = self.tmp / "app_script.h"
        shutil.copy2(server.SCRIPT_H, self.h)
        # 往临时副本的参数行尾塞一个显隐标记（照 DSL: 尾注释之前）
        txt = self.h.read_text(encoding="utf-8")
        head = " *   %s " % self.STEP
        line = next(ln for ln in txt.split("\n") if ln.startswith(head))
        marked = line.replace("   // ", "%s   // " % self.MARK, 1)   # 紧贴参数(与 _spec_str 的版式一致)
        self.assertNotEqual(marked, line, "没插进标记 —— 参数行格式变了?")
        self.h.write_text(txt.replace(line, marked), encoding="utf-8", newline="")
        p = mock.patch.object(server, "SCRIPT_H", self.h)
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def payload(self, with_show_if=True):
        """照 app.js::saveMetaEditor 现在发的形状。"""
        cat = {c["type"]: c for c in server.parse_catalog()}[self.STEP]
        params = []
        for m in cat["params_meta"]:
            d = {"name": m["name"], "raw": m.get("raw") or m["name"],
                 "label": m.get("label") or m["name"], "unit": m.get("unit") or "",
                 "options": m.get("options")}
            if with_show_if:
                d["show_if"] = m.get("show_if")
            params.append(d)
        return {"step": self.STEP, "params": params,
                "note": cat["note"], "n_f": len(cat["params_f"])}

    def test_metadata_save_keeps_the_marker(self):
        before = self.h.read_text(encoding="utf-8")
        self.assertIn(self.MARK, before, "沙箱里就该有标记")
        ok, resp, _ = server.write_meta(self.payload())
        self.assertTrue(ok, resp)
        after = self.h.read_text(encoding="utf-8")
        self.assertIn(self.MARK, after, "元数据保存把标记弄丢了")
        self.assertEqual(after, before, "只重写注释行, 内容没变就该零 diff")

    def _param_line(self):
        """app_script.h 里该积木的参数映射行。
        （不能直接 assertNotIn 整个文件 —— 文件头的 DSL 说明块里本来就写着 <显隐:...> 这个语法。）"""
        hit = [ln for ln in self.h.read_text(encoding="utf-8").split("\n")
               if ln.startswith(" *   %s " % self.STEP)]
        self.assertEqual(len(hit), 1, "没定位到参数映射行")
        return hit[0]

    def test_payload_without_show_if_would_drop_it(self):
        """负向对照: 这正是「前端必须透传」的理由。"""
        self.assertIn(self.MARK, self._param_line())
        ok, resp, _ = server.write_meta(self.payload(with_show_if=False))
        self.assertTrue(ok, resp)
        self.assertNotIn("<显隐:", self._param_line())


class WidgetWriteMetaSandbox(unittest.TestCase):
    """控件标记（<控件:move_var>）与显隐同一套纪律: 解析时剥、回写时补。

    与 ShowIfWriteMetaSandbox 的差别: 显隐标记是测试自己塞的, 控件标记 2026-10-29 起
    **真实文件里就有**（四个 STEP_CHASSIS_MOVE_WORLD_* 的位移参数）—— 直接拿真行当
    fixture, 顺带验证「无操作元数据保存零 diff」在这条 DSL 行上成立。"""

    STEP = "STEP_CHASSIS_MOVE_WORLD_X_WAIT"

    def setUp(self):
        if not server.SCRIPT_H.is_file():
            self.skipTest("app_script.h 不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="widgetsbx-"))
        self.h = self.tmp / "app_script.h"
        shutil.copy2(server.SCRIPT_H, self.h)
        p = mock.patch.object(server, "SCRIPT_H", self.h)
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def payload(self, with_widget=True):
        """照 app.js::saveMetaEditor 现在发的形状（widget 必须透传）。"""
        cat = {c["type"]: c for c in server.parse_catalog()}[self.STEP]
        params = []
        for m in cat["params_meta"]:
            d = {"name": m["name"], "raw": m.get("raw") or m["name"],
                 "label": m.get("label") or m["name"], "unit": m.get("unit") or "",
                 "options": m.get("options"), "show_if": m.get("show_if"),
                 "enable_if": m.get("enable_if")}
            if with_widget:
                d["widget"] = m.get("widget")
            params.append(d)
        return {"step": self.STEP, "params": params,
                "note": cat["note"], "n_f": len(cat["params_f"])}

    def test_metadata_save_keeps_move_var_widget(self):
        before = self.h.read_text(encoding="utf-8")
        self.assertIn("<控件:move_var>", before, "真文件里就该有控件标记")
        ok, resp, _ = server.write_meta(self.payload())
        self.assertTrue(ok, resp)
        self.assertEqual(self.h.read_text(encoding="utf-8"), before,
                         "元数据保存把 <控件:move_var> 弄丢了 / 产生了多余 diff")

    def test_payload_without_widget_would_drop_it(self):
        """负向对照: 这正是前端必须透传 widget 的理由（漏了 = 专用控件退化成输入框）。
        ⚠️ 只查 `//` 之前的参数段 —— 备注正文里出现「控件」字样不算标记。"""
        ok, resp, _ = server.write_meta(self.payload(with_widget=False))
        self.assertTrue(ok, resp)
        line = next(ln for ln in self.h.read_text(encoding="utf-8").split("\n")
                    if ln.startswith(" *   %s " % self.STEP))
        spec = line.split("//", 1)[0]
        self.assertNotIn("<控件:", spec)


class TestNormalizeParam(unittest.TestCase):
    """用户不必自己敲 u/f —— 页面按槽位类型补后缀。"""

    def test_uint_suffix_added_to_bare_int(self):
        self.assertEqual(server._normalize_param("8000", False), "8000u")
        self.assertEqual(server._normalize_param("0", False), "0u")

    def test_negative_int_gets_no_u(self):
        """-30u 是无符号取负语义, 放进 int32_t 初始化是坑 —— 必须不补。"""
        self.assertEqual(server._normalize_param("-30", False), "-30")
        self.assertEqual(server._normalize_param("+5", False), "+5")

    def test_float_slot_gets_dot_zero_f(self):
        self.assertEqual(server._normalize_param("8000", True), "8000.0f")
        self.assertEqual(server._normalize_param("-30", True), "-30.0f")

    def test_existing_suffix_left_alone(self):
        self.assertEqual(server._normalize_param("8000u", False), "8000u")
        self.assertEqual(server._normalize_param("2.5f", True), "2.5f")

    def test_non_numeric_left_alone(self):
        """符号/表达式原样透传, 不要瞎加后缀。"""
        for v in ("0x10", "1e3", "PID_X", "(-30)"):
            self.assertEqual(server._normalize_param(v, False), v, v)

    def test_empty_uses_slot_default(self):
        self.assertEqual(server._normalize_param("", False), "0")
        self.assertEqual(server._normalize_param("", True), "0.0f")


class TestSerializeStep(unittest.TestCase):
    """_serialize_step 走一遍完整链路: 编出的行该带上 u 后缀。"""

    CAT = {
        "STEP_TEST_NOWAIT": {
            "params_f": [], "params_i": ["n", "m"], "primitive": "ucTestPrim",
        },
        "STEP_TEST_FLOAT": {
            "params_f": ["a"], "params_i": [], "primitive": "vTestF",
        },
    }

    def test_int_params_get_u(self):
        line = server._serialize_step(
            {"step": "STEP_TEST_NOWAIT", "tag": "测试", "params": ["8000", "3"]},
            self.CAT)[0]
        self.assertIn("SCRIPT_STEP_I2", line)
        self.assertIn("8000u, 3u", line)
        self.assertIn("// ucTestPrim()", line)

    def test_negative_int_param_not_suffixed(self):
        line = server._serialize_step(
            {"step": "STEP_TEST_NOWAIT", "tag": "测试", "params": ["-30", "3"]},
            self.CAT)[0]
        self.assertIn("-30, 3u", line)

    def test_every_emittable_macro_exists(self):
        """生成器**可能编出的每个宏名**都必须在 app_script.h 里真实存在。

        生成器按家族参数个数封顶选名字（I≤6 / F≤5 / F1I≤5 / F2I≤8）。只要某个家族 1..上限
        缺一个宏, 就会在带那种参数个数的积木上编出「宏名对、实参多」的行, 直到 Keil 编译才报
        `#55-D: too many arguments in invocation of macro`（STEP_BOMB_GRAB_WAIT 4 个 int
        参数时就撞过这个, 当时没有 _I4）。这条测试把两边的表钉在一起。
        """
        h = (Path(__file__).resolve().parent.parent.parent
             / "Application" / "app_script.h").read_text(encoding="utf-8")
        have = set(re.findall(r"#define\s+(SCRIPT_STEP(?:_[A-Z0-9]+)?)\s*\(", h))
        # ⚠️ 后缀可选: 零参积木编出的是光秃秃的 `SCRIPT_STEP`
        # 上限直接引用生成器自己的常量 —— 以后抬上限时这条测试自动跟着核对。
        for cap, fam in ((map_codec.MACRO_CAP_I, "I"),
                         (map_codec.MACRO_CAP_F, "F"),
                         (map_codec.MACRO_CAP_F1I, "F1I"),
                         (map_codec.MACRO_CAP_F2I, "F2I")):
            for n in range(1, cap + 1):
                name = "SCRIPT_STEP_%s%d" % (fam, n)
                self.assertIn(name, have,
                              "生成器 %s 家族上限 %d, 但 app_script.h 缺 %s" % (fam, cap, name))

    def test_every_real_step_emits_a_defined_macro(self):
        """**真实 catalog 的每个积木**跑一遍生成器, 它编出的宏名必须在 app_script.h 里存在。

        上一条测的是"家族成员齐不齐", 这条测的是"**这个积木**的参数个数落在哪一族、那族够不够宽"
        —— 两者互补: 比如某天加一个 3 float + 2 int 的积木, 家族成员一个不缺也会编出一个
        根本不存在的 `SCRIPT_STEP_F1I2`（生成器对 n_f≥3 只有兜底分支）, 只有这条测得到。
        这是把「控制台编出来的行」提前到测试里编译一遍, 免得只在 Keil 那一步炸 #55-D。
        """
        h = (Path(__file__).resolve().parent.parent.parent
             / "Application" / "app_script.h").read_text(encoding="utf-8")
        have = set(re.findall(r"#define\s+(SCRIPT_STEP(?:_[A-Z0-9]+)?)\s*\(", h))
        # ⚠️ 后缀可选: 零参积木编出的是光秃秃的 `SCRIPT_STEP`
        cat = {c["type"]: c for c in server.parse_catalog()}
        self.assertGreater(len(cat), 20, "catalog 没解析出来, 这条测试就白测了")
        for step, meta in cat.items():
            n_f, n_i = len(meta.get("params_f", [])), len(meta.get("params_i", []))
            item = {"step": step, "tag": "x", "params": ["0"] * (n_f + n_i)}
            line = map_codec._serialize_step(item, cat)[0]
            # 零参积木编出来的是光秃秃的 `SCRIPT_STEP`（无 `_F..` 后缀）, 所以后缀可选
            macro = re.search(r"SCRIPT_STEP(_[A-Z0-9]+)?", line)
            self.assertIsNotNone(macro, "没编出宏名: %s" % line)
            self.assertIn(macro.group(0), have,
                          "%s（%d float + %d int）编出的 %s 在 app_script.h 里不存在 "
                          "—— 要么缺宏, 要么该给生成器加一族"
                          % (step, n_f, n_i, macro.group(0)))

    def test_pivot_line_param_layout(self):
        """支点转弯接管的参数布局 = C 侧槽位契约（**2 个 float 在前**）。

        2026-10-30: `src` / `cam_pct` 提升为全局宏（LINE_CONV_SRC / LINE_CAM_PCT）→
        本积木是 **F2I7**: F(0)=angle, F(1)=gyro_target;
        I(0..4)=dir,absolute,fwd_ms,settle_ms,line_ms; I(5)=tail_ms, I(6)=settle_early。
        """
        allcat = {c["type"]: c for c in server.parse_catalog()}
        cat = allcat.get("STEP_PIVOT_LINE_WAIT")
        if cat is None:
            self.skipTest("catalog 里没有 STEP_PIVOT_LINE_WAIT")
        self.assertEqual(cat["params_f"], ["angle", "gyro_target"],
                         "两个 float 槽在最前: app_script.c 里是 F(0)/F(1)")
        self.assertEqual(cat["params_i"],
                         ["dir", "absolute", "fwd_ms", "settle_ms", "line_ms",
                          "tail_ms", "settle_early"],
                         "7 个 int 槽随后: I(0)..I(6)")
        # settle_early 的下拉索引 = params_f 个数 + 它的 int 下标 = 2+6 = params_meta[8]
        self.assertEqual([o["value"] for o in cat["params_meta"][8].get("options", [])],
                         ["1", "0"])
        # 宏名: 2 float + 7 int → F2I7
        line = map_codec._serialize_step(
            {"step": "STEP_PIVOT_LINE_WAIT", "tag": "支点转弯",
             "params": ["90.0f", "0.0f", "1", "0", "0", "500", "0", "0", "1"]},
            allcat)[0]
        self.assertIn("SCRIPT_STEP_F2I7", line)
        self.assertIn("90.0f, 0.0f, 1u, 0u, 0u, 500u, 0u, 0u, 1u", line,
                      "两个 float 在前、七个 int 在后 —— 槽位顺序就是契约")
        self.assertEqual(server.serialize_map(server._parse_body([line]), allcat), [line])

    def test_line_straight_param_layout(self):
        """巡线直走的参数布局 = C 侧槽位契约（2 float + 4 int）。

        2026-10-30: `src` / `cam_pct` 提升为全局宏 → 本积木 F2I4:
        F(0)=dist_mm, F(1)=gyro_target; I(0)=accel, I(1)=speed,
        I(2)=use_res（{1=是,0=否}）, I(3)=settle_ms（起步对准）。
        """
        cat_all = {c["type"]: c for c in server.parse_catalog()}
        cat = cat_all.get("STEP_LINE_STRAIGHT_WAIT")
        if cat is None:
            self.skipTest("catalog 里没有 STEP_LINE_STRAIGHT_WAIT")
        self.assertEqual(cat["params_f"], ["dist_mm", "gyro_target"],
                         "两个 float 槽在最前: app_script.c 里是 F(0)/F(1)")
        self.assertEqual(cat["params_i"], ["accel", "speed", "use_res", "settle_ms"],
                         "4 个 int 槽随后: I(0)..I(3)")
        # use_res 的下拉索引 = params_f 个数 + 它的 int 下标 = 2+2 = params_meta[4]
        self.assertEqual([o["value"] for o in cat["params_meta"][4].get("options", [])],
                         ["1", "0"])
        # 宏名: 2 float + 4 int → F2I4
        line = map_codec._serialize_step(
            {"step": "STEP_LINE_STRAIGHT_WAIT", "tag": "巡线直走",
             "params": ["600.0f", "0.0f", "150", "60", "1", "300"]}, cat_all)[0]
        self.assertIn("SCRIPT_STEP_F2I4", line)
        self.assertIn("600.0f, 0.0f, 150u, 60u, 1u, 300u", line,
                      "两个 float 在前、四个 int 在后 —— 槽位顺序就是契约")
        self.assertEqual(server.serialize_map(server._parse_body([line]), cat_all), [line])

    def test_sparse_param_holes_serialize_as_zero_not_crash(self):
        """params 里的**稀疏洞**（JSON null）必须按空槽补 0, 绝不 500。

        2026-10 真踩过: 浏览器里老行比新 catalog 短, 中间留**稀疏洞**, JSON.stringify
        把洞序列化成 null → 服务端 _normalize_param(None) 崩 → 保存 500
        （'NoneType' object has no attribute 'strip'）。None = 未配置槽位 = 补 0。
        这里用支点新布局（2 float + 7 int, F2I7）覆盖 holes。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        line = map_codec._serialize_step(
            {"step": "STEP_PIVOT_LINE_WAIT", "tag": "支点转弯",
             "params": ["90.0f", "0.0f", "1", "0", "0", "500", None, "1", None]},
            cat)[0]
        self.assertIn("SCRIPT_STEP_F2I7", line)
        self.assertIn("90.0f, 0.0f, 1u, 0u, 0u, 500u, 0, 1u, 0", line,
                      "None 槽走空串路径补 0（不带 u）, 其余值原样")

    def test_too_many_params_refuses_instead_of_truncating(self):
        """参数个数超出宏族上限时**抛错拒掉**，绝不截到上限编出一行实参过多的行。

        2026-09 真踩过：控制台进程比 `app_script.h` 旧（给积木加了参数但**服务端代码没重启**）
        → 8 个实参配了 `SCRIPT_STEP_F1I5`（上限 5）的宏名 → 那份 map.c 编译报
        `#55-D: too many arguments`。截断版会把一份编译不过的文件留在盘上；
        抛错版当场拒掉那次保存（`map.c` 一个字节都不动），原因也写在报错里。
        """
        cat = {
            "STEP_TEST_WIDE": {"params_f": ["a", "b"],
                               "params_i": ["c", "d", "e", "f", "g", "h", "i", "j", "k", "l"],
                               "primitive": "ucWide"},   # 2 float + 10 int（F2I 上限 9, 2026-10-08 抬的）
            "STEP_TEST_F1I9": {"params_f": ["a"],
                               "params_i": ["c", "d", "e", "f", "g", "h", "i", "j", "k", "l"],
                               "primitive": "ucF1I9"},   # 1 float + 10 int（F1I 上限 9, 2026-10-30 抬的）
        }
        for step in ("STEP_TEST_WIDE", "STEP_TEST_F1I9"):
            with self.assertRaises(ValueError) as ctx:
                map_codec._serialize_step(
                    {"step": step, "tag": "x", "params": ["0"] * 10}, cat)
            self.assertIn("重启控制台", str(ctx.exception),
                          "报错要说清自救办法 —— 这条分支就是给服务端比头文件旧准备的")

    def test_four_int_params_pick_the_i4_macro(self):
        """4 个 int 参数必须选 SCRIPT_STEP_I4 —— 宏名上限与 app_script.h 的真实宏表失配
        时会编成 `SCRIPT_STEP_I3(...4 个实参...)`, 直到 Keil 编译才报
        `#55-D: too many arguments in invocation of macro`（STEP_BOMB_GRAB_WAIT 真踩过）。"""
        cat = dict(self.CAT)
        cat["STEP_TEST_I4"] = {"params_f": [],
                               "params_i": ["a", "b", "c", "d"],
                               "primitive": "ucTest4"}
        line = server._serialize_step(
            {"step": "STEP_TEST_I4", "tag": "四参数", "params": ["1", "2", "3", "4"]},
            cat)[0]
        self.assertIn("SCRIPT_STEP_I4", line)
        self.assertNotIn("SCRIPT_STEP_I3", line)
        self.assertIn("1u, 2u, 3u, 4u", line)

    def test_float_param_gets_f(self):
        line = server._serialize_step(
            {"step": "STEP_TEST_FLOAT", "tag": "测试", "params": ["45"]},
            self.CAT)[0]
        self.assertIn("SCRIPT_STEP_F1", line)
        self.assertIn("45.0f", line)


class TestRealMapRoundtrip(unittest.TestCase):
    """真实 map.c 的往返: 解析 → 重新序列化 == 原数组体逐行不变。

    这是「无操作保存字节级零 diff」在**真实文件**上的回归保护 —— 此前只在
    沙箱 fixture 上测过，map.c 真的被改坏（多一个逗号/丢一行）时不会有人报。
    """

    def test_every_preset_roundtrips_byte_identical(self):
        if not server.MAP_C.is_file():
            self.skipTest("map.c 不存在")
        lines = server.MAP_C.read_text(encoding="utf-8").split("\n")
        blocks = server._locate_presets(lines)
        self.assertTrue(blocks, "map.c 里应至少有一张预设表")

        cat = {c["type"]: c for c in server.parse_catalog()}
        for b in blocks:
            body = lines[b["open"] + 1:b["close"]]
            with self.subTest(preset=b["ident"]):
                self.assertEqual(server.serialize_map(server._parse_body(body), cat), body)

    def test_line_step_rows_match_new_catalog_arity(self):
        """六种巡线积木在 map.c 里的每行参数个数必须等于新 catalog 的槽数。

        2026-10-30 删 src/cam_pct 把 int 槽整体左移: 漏改一行 → 编译照过、值静默错位。
        这条把**所有预设 + 禁用行 + 共用物理行**都扫一遍。
        """
        if not server.MAP_C.is_file():
            self.skipTest("map.c 不存在")
        six = {"STEP_LINE_CONVERGE_WAIT", "STEP_LINE_STRAIGHT_WAIT",
               "STEP_PIVOT_LINE_WAIT", "STEP_SPIN_LINE_WAIT",
               "STEP_BALL_GRAB_ONE_WAIT", "STEP_HOSTAGE_GRAB_ONE_WAIT"}
        cat = {c["type"]: c for c in server.parse_catalog()}
        lines = server.MAP_C.read_text(encoding="utf-8").split("\n")
        for b in server._locate_presets(lines):
            for it in server._parse_body(lines[b["open"] + 1:b["close"]]):
                if it.get("kind") != "step" or it["step"] not in six:
                    continue
                want = len(cat[it["step"]]["params_f"]) + len(cat[it["step"]]["params_i"])
                self.assertEqual(
                    len(it["params"]), want,
                    "%s 的 %s: %d 个参数 != catalog %d 个槽（删/加参数后必须重写该行）"
                    % (b["ident"], it["step"], len(it["params"]), want))


class TestArmActionPaletteEntries(unittest.TestCase):
    """「机械臂动作」大积木的虚拟条目（只读真实源码, 不写盘）。"""

    def setUp(self):
        self.cat = server.parse_catalog()
        self.by_type = {c["type"]: c for c in self.cat}

    def test_types_are_unique(self):
        """虚拟条目必须用合成键 —— 与真步骤重名会把真条目从 cat_by_type 里顶掉。"""
        self.assertEqual(len(self.by_type), len(self.cat))

    def test_virtual_entry_points_at_a_real_step(self):
        for c in self.cat:
            if c.get("synth_step"):
                self.assertIn(c["synth_step"], self.by_type)
                self.assertIn("action_id", c)

    def test_virtual_entry_serializes_to_the_real_step(self):
        """拖进画布后落到 map.c 的行必须是真步骤名 + 预填的动作 ID。"""
        for c in self.cat:
            if not c.get("synth_step"):
                continue
            line = server._serialize_step(
                {"step": c["synth_step"], "tag": c["desc"],
                 "params": c["synth_params"], "primitive": c["primitive"]},
                self.by_type)[0]
            self.assertIn("(" + c["synth_step"] + ",", line)
            self.assertIn(c["synth_params"][0], line)
            self.assertNotIn(c["type"] + ",", line)      # 合成键绝不能进 C
            self.assertIn("// %s()" % c["primitive"], line)

    def test_real_entry_is_not_polluted(self):
        """克隆出来的虚拟条目不该改到真条目的参数元数据。"""
        for c in self.cat:
            if c.get("synth_step"):
                real = self.by_type[c["synth_step"]]
                self.assertNotIn("synth_step", real)
                self.assertIsNot(real["params_meta"], c["params_meta"])

    def test_action_param_has_enum_dropdown(self):
        """「动作」参数的下拉来自 emArmActionIdTdf —— 靠 app_script.c 里那句
        (emArmActionIdTdf)I(0)。删了它下拉就没了（不报错, 只是变成手敲）。"""
        entry = self.by_type.get("STEP_ARM_PLAY_WAIT")
        self.assertIsNotNone(entry, "app_script.h 里没有 STEP_ARM_PLAY_WAIT")
        meta = entry["params_meta"][0]
        self.assertEqual(meta.get("enum_type"), "emArmActionIdTdf")
        self.assertTrue(meta.get("options"))


class TestWriteMapRejectsPaletteKeys(unittest.TestCase):
    """虚拟条目的 type 只能留在积木库里, 写进 map.c 必须被拦住。"""

    def test_synth_type_in_a_step_is_rejected(self):
        if not server.MAP_C.is_file():
            self.skipTest("map.c 不存在")
        before = server.MAP_C.stat().st_mtime_ns
        ok, resp, status = server.write_map(
            {"base_version": server.MAP_C.stat().st_mtime,
             "active": "x",
             "presets": [{"ident": "x", "label": "x", "items": [
                 {"id": "i0", "kind": "step", "edited": True,
                  "step": "ARM_PLAY_1", "tag": "t", "params": ["ARM_ACT_1"]}]}]},
            {})
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertEqual(server.MAP_C.stat().st_mtime_ns, before, "被拒的保存却动了 map.c")


class TestIfStepCatalog(unittest.TestCase):
    """嵌套容器 IF 的条件参数, 以及「同类型枚举参数各自独立」那条老坑。

    载体说明: 这一段原来盯的是 STEP_IF_ELSE（它已随嵌套容器 IF 一起删掉）。「两个同类型枚举
    参数会共享 options list」那个 bug 与 IF 无关, 现在换 `STEP_BOMB_GRAB_WAIT`（4 个
    emArmActionIdTdf）继续钉; 条件下拉挪到 `STEP_IF`。
    ⚠️ 这几条**不带 skipTest 护身符**: 这些积木必须存在。少了它们不是「还没做到」而是
       「catalog 派生坏了」, 静静 skip 掉就等于没有测试。
    """

    def setUp(self):
        self.cat = {c["type"]: c for c in server.parse_catalog()}

    def test_if_condition_param_has_its_own_enum(self):
        c = self.cat["STEP_IF"]
        self.assertEqual(c["primitive"], "vScriptIf", "原语名 = case 体里第一个函数调用")
        self.assertEqual(c["params_f"], [], "条件是 int 槽")
        self.assertEqual(c["params_i"], ["cond"])
        m = c["params_meta"][0]
        self.assertEqual(m["enum_type"], "emScriptCondTdf")
        self.assertTrue(m["options"], "条件下拉是空的")

    def test_the_markers_have_no_params(self):
        """「否则」/「结束如果」无参 —— 有参数就说明 DSL 或 case 体被误改了。"""
        for step, prim in (("STEP_ELSE", "vScriptElse"), ("STEP_END_IF", "vScriptEndIf")):
            with self.subTest(step=step):
                c = self.cat[step]
                self.assertEqual(c["primitive"], prim)
                self.assertEqual(c["params_f"], [])
                self.assertEqual(c["params_i"], [])

    def test_same_enum_params_do_not_share_an_options_list(self):
        """parse_catalog 早先是按**引用**把 enums[etype] 挂上去的 —— 两个同类型参数会指向
        同一个 list 对象, 前端右键改一个选项会连带改另一个。"""
        c = self.cat["STEP_BOMB_GRAB_WAIT"]        # 4 个槽全是 emArmActionIdTdf
        metas = c["params_meta"]
        self.assertEqual(len(metas), 4)
        for m in metas:
            self.assertEqual(m["enum_type"], "emArmActionIdTdf")
        self.assertIsNot(metas[0]["options"], metas[1]["options"], "两个动作下拉共享了同一个 list")
        self.assertIsNot(metas[0]["options"][0], metas[1]["options"][0])

    def test_unsafe_targets_are_flagged_in_the_dropdown(self):
        """「不可跳入」要出现在选目标的那一刻 —— 就在下拉标签里。

        IF 分支不再跳预设之后, 还选预设目标的只剩 `STEP_SCRIPT_CALL`。"""
        c = self.cat["STEP_SCRIPT_CALL"]
        labels = {o["value"]: o["label"] for o in c["params_meta"][0]["options"]}
        self.assertIn("继续", labels.get("SCRIPT_PRESET_NONE", ""))
        unsafe = [v for v, l in labels.items() if "不可跳入" in l]
        self.assertTrue(unsafe, "一个「不可跳入」标记都没有 —— 真实 map.c 里有握手/重置的预设")


class TestMapIncludeWarning(unittest.TestCase):
    """用了「机械臂动作」大积木却没 include app_arm_action.h → 编译期
    `identifier "ARM_ACT_n" is undefined`。控制台保存时提醒一句（**只提醒不拦**）。

    真踩过: 台架上拖完积木、保存、编译，对着 `ARM_ACT_1 is undefined` 猜是不是哪里写错了,
    其实只是 map.c 少一行 #include。
    """

    def test_warns_when_include_is_missing(self):
        items = [{"ident": "g_astScriptPresetX", "label": "x", "items": [
            {"id": "i0", "kind": "step", "step": "STEP_ARM_PLAY_WAIT", "tag": "t",
             "params": ["ARM_ACT_1"]}]}]
        warn = server._map_include_warning(items, '#include "map.h"\nint x;\n')
        self.assertIn("app_arm_action.h", warn)

    def test_quiet_when_include_present(self):
        items = [{"ident": "p", "label": "x", "items": [
            {"id": "i0", "kind": "step", "step": "STEP_ARM_PLAY_WAIT", "tag": "t",
             "params": ["ARM_ACT_1"]}]}]
        self.assertEqual(server._map_include_warning(
            items, '#include "app_arm_action.h"\n'), "")

    def test_quiet_for_ordinary_routes(self):
        items = [{"ident": "p", "label": "x", "items": [
            {"id": "i0", "kind": "step", "step": "STEP_DEAD", "tag": "t", "params": []}]}]
        self.assertEqual(server._map_include_warning(items, "#include \"map.h\"\n"), "")

    def test_real_map_has_the_include(self):
        """真实的 map.c 一旦用了大积木, 就**必须**带上这个头（否则仓库本身编译不过）。"""
        if not server.MAP_C.is_file():
            self.skipTest("map.c 不存在")
        text = server.MAP_C.read_text(encoding="utf-8")
        if "STEP_ARM_PLAY_" in text:
            self.assertIn("app_arm_action.h", text,
                          "map.c 用了机械臂动作步骤但没 include app_arm_action.h —— 编译必挂")


class TestCodecIsSingleSource(unittest.TestCase):
    """map.c 的行解析只有 map_codec.py 一个来源。

    server.py 用 `from map_codec import (...)` 重新绑定而非自己再定义一份 ——
    一旦有人图省事把函数复制回 server.py，两份就会各自漂移（改了 A 忘了 B）。
    """

    CODEC_FUNCS = ("_split_top", "_locate_presets", "_strip_comment_marks", "_split_call",
                   "_split_args", "_is_wrap_marker", "_parse_section_header", "_parse_body",
                   "_to_line_comments", "_to_block_comments", "_toggle_disable",
                   "_normalize_param", "_ensure_comma", "_serialize_step", "serialize_map",
                   "_make_preset_block",
                   # 预设注册表那一套（「跳到别的预设」积木）—— 同样只有 map_codec 一个来源
                   "find_marked", "parse_preset_registry", "parse_next_preset_id",
                   "preset_call_unsafe", "preset_refs", "serialize_preset_registry",
                   "serialize_preset_header")

    def test_server_does_not_redefine_them(self):
        src = (Path(__file__).resolve().parent / "server.py").read_text(encoding="utf-8")
        for name in self.CODEC_FUNCS:
            self.assertNotIn("def %s(" % name, src, name)


class PresetSandbox(unittest.TestCase):
    """把 MAP_C / MAP_H 指向临时副本 —— 全程不碰仓库里的真实文件。

    「零 diff」是这套东西唯一的验收标准, 而它只有在**真实的 map.c/map.h** 上测才有意义
    （沙箱 fixture 是我自己造的, 自己造的自己当然对得上）。
    """

    def setUp(self):
        if not (server.MAP_C.is_file() and server.MAP_H.is_file()):
            self.skipTest("map.c / map.h 不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="presetsbx-"))
        self.c = self.tmp / "map.c"
        self.h = self.tmp / "map.h"
        shutil.copy2(server.MAP_C, self.c)
        shutil.copy2(server.MAP_H, self.h)
        for mod, attr, val in ((server, "MAP_C", self.c), (server, "MAP_H", self.h)):
            p = mock.patch.object(mod, attr, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    # ---- 小工具 ----
    def payload(self, presets=None, active=None):
        m = server.parse_map()
        return {"base_version": m["version"],
                "active": active or m["active"],
                "presets": m["presets"] if presets is None else presets}

    def save(self, payload):
        return server.write_map(payload, {c["type"]: c for c in server.parse_catalog()})

    def h_text(self):
        return self.h.read_text(encoding="utf-8")

    def c_text(self):
        return self.c.read_text(encoding="utf-8")

    @staticmethod
    def dead(tag="t"):
        return {"id": "i0", "kind": "step", "edited": True,
                "step": "STEP_DEAD", "tag": tag, "params": []}

    # 嵌套容器 IF 的三块标记（app_script.h 的 STEP_IF / STEP_ELSE / STEP_END_IF）。
    @staticmethod
    def if_open(cond="SCRIPT_COND_QR", tag="如果"):
        return {"id": "i0", "kind": "step", "edited": True,
                "step": "STEP_IF", "tag": tag,
                "params": [cond], "primitive": "vScriptIf"}

    @staticmethod
    def if_else(tag="否则"):
        return {"id": "i0", "kind": "step", "edited": True,
                "step": "STEP_ELSE", "tag": tag, "params": [],
                "primitive": "vScriptElse"}

    @staticmethod
    def if_close(tag="结束如果"):
        return {"id": "i0", "kind": "step", "edited": True,
                "step": "STEP_END_IF", "tag": tag, "params": [],
                "primitive": "vScriptEndIf"}

    @staticmethod
    def call_step(target="SCRIPT_PRESET_NONE", tag="调用预设"):
        """还选预设目标的积木（IF 分支不再跳预设了, 这条路只剩 STEP_SCRIPT_CALL）。"""
        return {"id": "i0", "kind": "step", "edited": True,
                "step": "STEP_SCRIPT_CALL", "tag": tag, "params": [target]}


class TestPresetRegistryNoop(PresetSandbox):
    """无操作保存必须**字节级零 diff** —— 这是整套回写机制的前提。

    第一次保存之所以也不是零 diff, 是因为生成器与手写格式必须逐字符一致;
    对不上就说明有一边漂了, 所以这里连第一次也一起断言。
    """

    def test_first_save_matches_the_committed_files(self):
        c0, h0 = self.c.read_bytes(), self.h.read_bytes()
        ok, resp, status = self.save(self.payload())
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)
        self.assertEqual(self.c.read_bytes(), c0, "map.c 第一次保存就产生了 diff")
        self.assertEqual(self.h.read_bytes(), h0, "map.h 第一次保存就产生了 diff")

    def test_second_save_is_a_true_noop(self):
        self.save(self.payload())
        c1, h1 = self.c.read_bytes(), self.h.read_bytes()
        h_mtime = self.h.stat().st_mtime_ns
        self.save(self.payload())
        self.assertEqual(self.c.read_bytes(), c1)
        self.assertEqual(self.h.read_bytes(), h1)
        self.assertEqual(self.h.stat().st_mtime_ns, h_mtime,
                         "map.h 内容没变却又写了一次 —— 白写会让别的 Tab 下次保存吃 409")

    def test_noop_save_does_not_touch_map_c_at_all(self):
        """map.c 的 mtime 是版本戳: 内容没变就**一个字节都不写**, 连 .bak 都不建。

        ⚠️ 与 map.h / arm_action 那两处同款纪律。漏了这条的后果不是"多写一次盘", 而是
           **另一个干净客户端的 /api/rev 会判成「远端改动」**（revNewer 是严格 `>`），
           于是它白自动重载一轮、还弹一句「已被其他客户端修改」—— 用户明明什么都没改。
        """
        self.save(self.payload())
        c_bytes, c_mtime = self.c.read_bytes(), self.c.stat().st_mtime_ns
        bak = self.c.with_suffix(".c.bak")
        if bak.exists():
            bak.unlink()                             # 先清掉, 好断言「这次没建」
        self.save(self.payload())                    # 无操作再存一次
        self.assertEqual(self.c.read_bytes(), c_bytes)
        self.assertEqual(self.c.stat().st_mtime_ns, c_mtime,
                         "无操作保存动了 map.c 的 mtime（会让另一个 Tab 吃一轮无谓的重载）")
        self.assertFalse(bak.exists(), "无操作保存不该建 .bak")

    def test_a_real_edit_still_writes_and_backs_up(self):
        """反过来钉一遍: 真改了东西时必须写盘 + 建 .bak（别把上面那条修成「永不写」）。"""
        p = self.payload()
        p["presets"][0]["items"] = list(p["presets"][0]["items"]) + [self.dead("新加的")]
        ok, resp, _ = self.save(p)
        self.assertTrue(ok, resp)
        self.assertTrue(self.c.with_suffix(".c.bak").is_file(), "改动了却没建 .bak")
        self.assertIn("新加的", self.c_text())


class TestPresetIds(PresetSandbox):
    """预设 id **只增不重用**（同 ARM_ACTION_NEXT_ID 那个坑）。"""

    def test_registry_id_matches_the_table_name(self):
        """g_astScriptPresetN ↔ SCRIPT_PRESET_N 必须一一对应, 否则下拉选的和实际跑的不是一条。"""
        reg = map_codec.parse_preset_registry(self.c_text().split("\n"))
        self.assertTrue(reg, "map.c 里没有预设注册表区块")
        for ident, e in reg.items():
            m = map_codec.RE_PRESET_IDENT.match(ident)
            self.assertIsNotNone(m, "注册表里的标识不是 g_astScriptPresetN: %s" % ident)
            self.assertEqual(int(m.group(1)), e["id"], ident)

    def test_parse_map_exposes_id_and_next_id(self):
        """id 只增不重用 → **允许空洞**, 所以不能断言「连续 1..N」。

        ⚠️ 原来断的是 `list(range(1, N+1))`, 与自家设计直接矛盾: 删掉中间某个预设后
           那个号自此不再复用（`SCRIPT_PRESET_NEXT_ID` 单调）, 于是**任何带空洞的合法
           仓库状态都会挂**。当前 Application/map.c 的 13 号就是这种空洞（map.h 里
           NEXT_ID 已经是 20）—— 那不是 bug, 是设计要的样子。
           要钉的是「不重复」与「next_id 高于所有已用的号」这两条真约束。
        """
        m = server.parse_map()
        ids = [p["id"] for p in m["presets"]]
        self.assertTrue(all(i is not None for i in ids),
                        "有预设没分到 id —— 它不会进注册表, 固件按 id 查表就查不到: %r" % ids)
        self.assertEqual(len(set(ids)), len(ids), "预设 id 重复: %r" % ids)
        self.assertEqual(ids, sorted(ids), "预设 id 不是递增的（parse_map 按文件顺序给）: %r" % ids)
        self.assertGreater(m["next_id"], max(ids),
                           "next_id 必须大于所有已用的号 —— 否则新预设会复用旧号, "
                           "而 map.c 里残留的 SCRIPT_PRESET_n 会无声指向另一条路线")

    def test_new_preset_uses_next_id(self):
        m = server.parse_map()
        fresh = m["next_id"]
        m["presets"].append({"ident": "g_astScriptPreset%d" % fresh,
                             "label": "新预设", "items": [self.dead()]})
        ok, resp, status = self.save(self.payload(m["presets"], active=m["active"]))
        self.assertTrue(ok, resp)
        self.assertEqual(resp["next_id"], fresh + 1)
        self.assertIn("SCRIPT_PRESET_%d" % fresh, self.h_text())
        self.assertIn("g_astScriptPreset%d" % fresh, self.c_text())

    def test_freed_gap_number_is_rejected(self):
        """编号空档：中间某号被删过、注册表里已经没它了, 客户端却还拿它建新预设。

        ⚠️ 服务端**挡不住**「删掉最后一号再新建同号」那种情形 —— 删除要到 phase A 才落地,
           那一刻文件注册表里那个号还在, 于是新来的同号被判成「老预设保留」, 与「编辑了
           那个预设的步骤」完全无法区分（拿不到任何额外信息）。**那条防在前端**:
           nextPresetIdent 从 NEXT_ID 分配 → 新预设拿到更大的号 → map.c 里残留的
           SCRIPT_PRESET_n 变成 undefined → 编译期硬失败。见 CLAUDE.md。
           这里能挡的是空档号 —— 同样会无声指向另一条路线。
        """
        m = server.parse_map()
        doomed = m["presets"][2]                        # 删中间的预设3, 腾出一个**空档**号
        freed = doomed["id"]

        kept = [p for p in m["presets"] if p["ident"] != doomed["ident"]]
        ok, resp, _ = self.save(self.payload(kept, active=m["active"]))
        self.assertTrue(ok, resp)
        # ⚠️ 必须**整词**匹配（后面不能还是数字）: 子串 assertNotIn 会被 SCRIPT_PRESET_30
        #    撞上（查 "SCRIPT_PRESET_3"、命中的却是 30）—— 2026-10 加预设 30 后真踩过。
        self.assertIsNone(re.search(r"SCRIPT_PRESET_%d(?!\d)" % freed, self.h_text()),
                          "删掉的预设还留在 map.h 的枚举里")

        # 拿着旧状态的客户端要新建一个**同号**的预设
        kept = kept + [{"ident": doomed["ident"], "label": "冒名顶替",
                        "items": [self.dead()]}]
        c_before, h_before = self.c.read_bytes(), self.h.read_bytes()
        mt_c, mt_h = self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns

        ok, resp, status = self.save(self.payload(kept, active=m["active"]))
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("已被回收", resp["message"])
        self.assertEqual(self.c.read_bytes(), c_before, "被拒的保存动了 map.c")
        self.assertEqual(self.h.read_bytes(), h_before, "被拒的保存动了 map.h")
        self.assertEqual((self.c.stat().st_mtime_ns, self.h.stat().st_mtime_ns),
                         (mt_c, mt_h))

    def test_next_id_survives_deleting_the_highest(self):
        """删掉最大号之后 NEXT_ID **不能回退** —— 回退就是下一次回收的入口。"""
        m = server.parse_map()
        before = m["next_id"]
        kept = m["presets"][:-1]
        self.assertTrue(self.save(self.payload(kept, active=m["active"]))[0])
        self.assertEqual(server.parse_map()["next_id"], before,
                         "删掉最大号把 NEXT_ID 拉回去了")

    def test_file_without_registry_can_still_be_saved(self):
        """老 map.c（还没有注册表区块）必须能照常保存 —— 首次保存时补上区块。

        ⚠️ 这里同时防一个**自己踩出来的**坑: 编号回收校验一旦不看「注册表在不在」,
           老文件里那些 g_astScriptPreset1..N 会被全判成「低于 NEXT_ID 的新号」,
           整个文件**一个字节都存不进去**。注册表为空时一律照收。
        """
        m = server.parse_map()
        # 手写一个「还没有注册表」的 map.c: 删掉整个注册表区块
        span = map_codec.find_marked(self.c_text().split("\n"),
                                     map_codec.RE_PRESET_REG_BEGIN,
                                     map_codec.RE_PRESET_REG_END)
        self.assertIsNotNone(span, "真实 map.c 里没有注册表区块")
        lines = self.c_text().split("\n")
        self.c.write_text("\n".join(lines[:span[0] - 1] + lines[span[1] + 1:]),
                          encoding="utf-8", newline="")

        m = server.parse_map()
        self.assertTrue(all(p["id"] is None for p in m["presets"]),
                        "注册表都删了, id 不该还读得出来")
        ok, resp, status = self.save(m)
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)
        reg = map_codec.parse_preset_registry(self.c_text().split("\n"))
        self.assertEqual(len(reg), len(m["presets"]), "首次保存没把注册表补上")

    def test_duplicate_ident_is_rejected(self):
        m = server.parse_map()
        m["presets"].append(dict(m["presets"][0]))
        c_before = self.c.read_bytes()
        ok, resp, status = self.save(self.payload(m["presets"], active=m["active"]))
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("重复提交", resp["message"])
        self.assertEqual(self.c.read_bytes(), c_before)

    def test_non_conforming_ident_still_gets_an_id(self):
        """手写的表名（不是 g_astScriptPresetN 形式）也必须进注册表。

        表名长什么样与「能不能进注册表」无关 —— 注册表才是 id 的真值来源, 标识里那个
        数字只是「这个编号能不能复用」的依据。
        """
        m = server.parse_map()
        m["presets"].append({"ident": "g_astMyRoute", "label": "手写表",
                             "items": [self.dead()]})
        ok, resp, status = self.save(self.payload(m["presets"], active=m["active"]))
        self.assertTrue(ok, resp)
        reg = map_codec.parse_preset_registry(self.c_text().split("\n"))
        self.assertIn("g_astMyRoute", reg, "手写表名的预设没进注册表")
        self.assertIn("SCRIPT_PRESET_%d" % reg["g_astMyRoute"]["id"], self.h_text())

    def test_all_hand_named_tables_still_produce_a_valid_registry(self):
        """最凶的那一版: **每张表**都是手写表名（老 map.c 就长这样）。

        ⚠️ 锁一个真踩过的坑: 曾经把这类表整行跳出注册表 —— 于是「表名全不合规」时
           生成的是**空初始化列表** `= { };`, armcc 实报
           `#1514: an empty initializer is invalid for an array with unspecified bound`。
           固件里 app_script.c 无条件引用 g_astScriptPreset / g_usScriptPresetNum,
           所以整个工程编译不过, 而报错指在控制台自己生成的区块上。
        """
        m = server.parse_map()
        renamed = []
        for i, p in enumerate(m["presets"], 1):
            q = dict(p)
            q["ident"] = "g_route%d" % i
            renamed.append(q)

        ok, resp, status = self.save(self.payload(renamed, active=renamed[0]["ident"]))
        self.assertTrue(ok, resp)

        block = self.c_text().split("预设注册表: 开始")[1].split("预设注册表: 结束")[0]
        self.assertNotIn("{\n};", block, "生成了空初始化列表 —— C 编译不过")
        rows = [ln for ln in block.split("\n") if ln.strip().startswith("{")]
        self.assertEqual(len(rows), len(renamed), "注册表行数与预设数对不上")

        ids = [map_codec.RE_PRESET_REG_ENTRY.match(ln).group(1) for ln in rows]
        self.assertEqual(len(ids), len(set(ids)),
                         "注册表 id 有重复 → map.h 的枚举会出现重复成员")
        h = self.h_text()
        for i in ids:
            self.assertIn("SCRIPT_PRESET_%s" % i, h)


class TestUserTextIsSanitisedOnWrite(PresetSandbox):
    """写回路径对用户输入必须清洗 —— 用户自由文本（积木名 / 分区标题 / 预设名）会插进
    map.c 的 C 字符串字面量与块注释里, 不清洗就是自伤式的语法错误。

    比"编译不过"更坏的是**静默数据损坏**: tag 里一个多余的 `"` 会让 _split_call 的字符串
    跟踪被带偏、参数切分错位, 下次保存就悄悄改写邻近条目。所以这里的判据不是"文件里没有
    那个字符", 而是**写出去还能原样解析回来**。

    ⚠️ 清洗只作用于 `edited` 条目 —— 未编辑条目仍按原始物理行回吐,
       所以 test_second_save_is_a_true_noop 那条零 diff 不受影响。
    """

    def _single_item(self, item):
        """把整个 map.c 压成「一张预设、一个条目」, 免得真实路线里的条目干扰判据。"""
        m = server.parse_map()
        first = dict(m["presets"][0])
        first["items"] = [item]
        return self.payload([first], active=first["ident"])

    def test_quote_in_tag_round_trips(self):
        ok, resp, _ = self.save(self._single_item(self.dead('我"的"名字')))
        self.assertTrue(ok, resp)
        self.assertNotIn('我"的"名字', self.c_text(), "双引号没被清洗, 写出的 C 字符串是坏的")
        # 关键: 写出去还得能解析回来, 且条目数不变（错位的症状是条目被刷出/多出）
        m = server.parse_map()
        self.assertEqual(len(m["presets"][0]["items"]), 1)
        self.assertEqual(m["presets"][0]["items"][0]["tag"], "我'的'名字")

    def test_slashes_in_tag_do_not_swallow_the_comma(self):
        """`//` 会让 _ensure_comma 在行内第一个 `//` 处插尾逗号 → 逗号插进字符串里。"""
        ok, resp, _ = self.save(self._single_item(self.dead("抓取//A")))
        self.assertTrue(ok, resp)
        m = server.parse_map()
        self.assertEqual(m["presets"][0]["items"][0]["tag"], "抓取/A",
                         "tag 里还留着 // —— 尾逗号会被插进字符串里, C 语法错误")

    def test_unbalanced_paren_in_tag_does_not_shift_params(self):
        """不成对括号让 _parse_body 的括号计数错位 → 邻近条目被错误刷出。"""
        open_it = self.if_open()
        open_it["tag"] = "如果 (条件"
        m = server.parse_map()
        first = dict(m["presets"][0])
        first["items"] = [open_it, self.if_close()]     # 标记必须成对, 否则新校验会先拦下
        ok, resp, _ = self.save(self.payload([first], active=first["ident"]))
        self.assertTrue(ok, resp)
        m = server.parse_map()
        items = m["presets"][0]["items"]
        self.assertEqual(len(items), 2, "括号没配平 → 条目数变了（解析错位）")
        self.assertEqual(items[0]["params"], ["SCRIPT_COND_QR"],
                         "参数被切错位了 —— 正是「重载后邻近积木被悄悄改写」那个症状")

    def test_cleaning_is_reported_back_to_the_user(self):
        """清洗**必须说出来** —— 静默改写用户输入比不改更糟: 他会在下次重载时看到一个
        自己没打过的名字, 只会觉得「控制台把我的东西弄丢了」。"""
        ok, resp, _ = self.save(self._single_item(self.dead('我"的"名字')))
        self.assertTrue(ok, resp)
        self.assertIn("warning", resp, "改写了用户文本却一声不吭")
        self.assertIn("积木名", resp["warning"])
        self.assertIn("我'的'名字", resp["warning"], "提示里要说清改成了什么")

    def test_a_clean_tag_produces_no_warning(self):
        """没被改写就别打扰 —— 每次保存都弹提示会让真正的告警失去意义。"""
        ok, resp, _ = self.save(self._single_item(self.dead("干净的积木名")))
        self.assertTrue(ok, resp)
        self.assertNotIn("warning", resp)

    def test_section_title_cannot_close_the_comment(self):
        """分区标题里的 `*/` 会提前闭合注释, 后面半行变成**活的 C 代码**。"""
        sec = {"id": "i0", "kind": "section", "edited": True, "disabled": False,
               "wrap": False, "title": "坏标题 */ int x = 1; /*", "text": "坏标题"}
        ok, resp, _ = self.save(self._single_item(sec))
        self.assertTrue(ok, resp)
        text = self.c_text()
        self.assertNotIn("*/ int x", text, "分区标题的 */ 漏出去了 —— 后面半行成了活代码")
        m = server.parse_map()
        self.assertEqual(len(m["presets"][0]["items"]), 1)

class TestNewPresetLabelIsSanitised(PresetSandbox):
    """**新建**预设的名字也要清洗。

    ⚠️ 原来只有改名（phase C）走 _preset_header_label, 新建（phase B）传的是原始 label ——
       于是新建一个名叫 `坏名 */ x` 的预设直接写坏 map.c 头注释。两个 phase 必须同一套。
    """

    def test_new_preset_with_comment_breaker_in_label(self):
        m = server.parse_map()
        fresh = m["next_id"]
        presets = m["presets"] + [{"ident": "g_astScriptPreset%d" % fresh,
                                   "label": "坏名 */ int x = 1; /*",
                                   "items": [self.dead()]}]
        ok, resp, status = self.save(self.payload(presets, active=m["active"]))
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)
        text = self.c_text()
        self.assertNotIn("*/ int x", text, "新建预设的 label 漏出去了 —— 头注释被提前闭合")
        # 新表本身仍然解析得出来（标题被洗过, 但表在）
        self.assertIn("g_astScriptPreset%d" % fresh, text)
        idents = [p["ident"] for p in server.parse_map()["presets"]]
        self.assertIn("g_astScriptPreset%d" % fresh, idents)


class TestPresetJumpRailings(PresetSandbox):
    """两条「只提醒不拦」的护栏 + 下拉标签里的「不可跳入」标记。"""

    def test_unsafe_presets_are_marked_in_the_label(self):
        """警告要出现在**选目标的那一刻**（下拉里）, 而不是保存时弹个 toast。

        真实 map.c 里: 预设1 表尾有「重置脚本」, 预设7/8/9/11/12 表首有握手 —— 都不可跳入;
        预设3/4/6/10 表首只有等键, 安全。
        ⚠️ 这条盯的是「表首握手 ⇒ 标记不可跳入」这个不变量, 所以挑一个**当前**表首真有
        握手的预设来断言。原来用的是预设2 —— 它从 2026-09 起被改成了菜单「开始巡线测试」
        跑的那条（表首那两步握手/等键已被注释掉, 见 CLAUDE.md「OLED 菜单」）, 于是不再
        满足前提; 若哪天 7 也被改掉, 换另一个表首带握手的预设即可（`preset_call_unsafe`
        会忽略被禁用的行, 所以注释掉的不算）。
        """
        m = server.parse_map()
        unsafe = {p["ident"]: map_codec.preset_call_unsafe(p["items"]) for p in m["presets"]}
        self.assertTrue(unsafe["g_astScriptPreset1"], "预设1 表尾的重置没被认出来")
        self.assertTrue(unsafe["g_astScriptPreset7"], "预设7 表首的握手没被认出来")

        h = self.h_text()
        self.assertIn("/* 默认路线（不可跳入） */", h)
        self.assertIn("/* 测试减速带 */", h)          # 安全的那个不带标记
        self.assertNotIn("测试减速带（不可跳入）", h)

    def test_dangling_reference_only_warns(self):
        """引用了 payload 里不存在的预设 → 编译期 undefined identifier, 但不该拦住保存
        （用户可能正要手工把那条表加回来）。"""
        m = server.parse_map()
        ref = m["next_id"] + 5                          # 一个不存在的号
        m["presets"][0]["items"].append(
            self.call_step(target="SCRIPT_PRESET_%d" % ref))
        ok, resp, status = self.save(self.payload(m["presets"], active=m["active"]))
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)
        self.assertIn("SCRIPT_PRESET_%d" % ref, resp.get("warning", ""))

    def test_if_markers_roundtrip_byte_identical(self):
        """三块标记落地后必须能被原样读回来（未编辑条目按原始物理行回吐）。

        ⚠️ 这里刻意**不传 primitive** —— 原语名应当由 catalog 从 app_script.c 的 case 体
           推出来。传了就等于自己把答案写进题目, 而真实路径（控制台保存）是推出来的:
           case 体里少写一个 `(emScriptCondTdf)I(0)` 这种错, 传 primitive 的写法看不见。
        """
        cat = {c["type"]: c for c in server.parse_catalog()}
        for item, want_prim, want_macro in (
                ({"step": "STEP_IF", "tag": "如果",
                  "params": ["SCRIPT_COND_QR"]}, "vScriptIf", "SCRIPT_STEP_I1"),
                ({"step": "STEP_ELSE", "tag": "否则",
                  "params": []}, "vScriptElse", "SCRIPT_STEP "),
                ({"step": "STEP_END_IF", "tag": "结束如果",
                  "params": []}, "vScriptEndIf", "SCRIPT_STEP ")):
            with self.subTest(step=item["step"]):
                line = server._serialize_step(dict(item), cat)[0]
                self.assertIn(want_macro, line)
                self.assertIn("// %s()" % want_prim, line)
                self.assertEqual(server.serialize_map(server._parse_body([line]), cat), [line])

    def test_label_with_comment_terminator_does_not_break_the_headers(self):
        """标题是用户输入的, 它会进**两个** C 注释: `map.h` 的枚举尾注释与
        `map.c` 的 `/* ===== 预设: X ===== */` 头注释。一个 `*/` 就能把任一处拆散。

        ⚠️ map.c 那头**曾经是真坏的**（写的是未净化的原文）: 注释在 `*/` 处提前闭合,
           后半行 `后面全是注释 ===== */` 变成 C 代码 → 编译不过。而当时只有 map.h
           被测试盯着, 所以测试还给了「恶意标题已经处理好了」的假信心。
        """
        m = server.parse_map()
        m["presets"][1]["label"] = "坏标题 */ 后面全是注释"
        ok, resp, status = self.save(self.payload(m["presets"], active=m["active"]))
        self.assertTrue(ok, resp)

        c_hdr = [ln for ln in self.c_text().split("\n") if ln.startswith("/* ===== 预设: 坏标题")]
        self.assertEqual(len(c_hdr), 1, "map.c 的预设头注释行找不着了")
        self.assertEqual(c_hdr[0].count("*/"), 1,
                         "标题里的 */ 把 map.c 的头注释提前闭合了（后半行变成 C 代码）")
        self.assertTrue(c_hdr[0].rstrip().endswith("*/"), c_hdr[0])

        h = self.h_text()
        span = map_codec.find_marked(h.split("\n"),
                                     map_codec.RE_PRESET_GEN_BEGIN,
                                     map_codec.RE_PRESET_GEN_END)
        self.assertIsNotNone(span, "标题里的 */ 把 map.h 的生成块标记拆掉了")
        label_lines = [ln for ln in h.split("\n")[span[0]:span[1]] if "坏标题" in ln]
        self.assertEqual(len(label_lines), 1)
        self.assertEqual(label_lines[0].count("*/"), 1,
                         "标题里带进来的 */ 会提前闭合枚举尾注释")

        # 再存一次必须还是零 diff（净化结果稳定）——注意 payload 里带的仍是**没洗过**的
        # 标题（前端在内存里留的是原文）, 所以这条同时验了「洗过再写 = 同一个字节」
        c_after, h_after = self.c.read_bytes(), self.h.read_bytes()
        self.save(self.payload(m["presets"], active=m["active"]))
        self.assertEqual(self.c.read_bytes(), c_after, "重存产生了 diff（洗过的标题不稳定）")
        self.assertEqual(self.h.read_bytes(), h_after)

    def test_half_broken_registry_markers_are_rejected(self):
        """标记只坏一半时**不能**再插一个区块 —— 那会出现两个 g_astScriptPreset[]。"""
        self.c.write_text(
            "\n".join(ln for ln in self.c_text().split("\n")
                      if not map_codec.RE_PRESET_REG_END.match(ln)),
            encoding="utf-8", newline="")

        c_before = self.c.read_bytes()
        ok, resp, status = self.save(server.parse_map())
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("不配对", resp["message"])
        self.assertEqual(self.c.read_bytes(), c_before)

    def test_map_h_failure_is_warned_not_silent(self):
        """map.h 维护不了时必须**说出来**。

        它同时是 id 单调性的唯一持久化处（SCRIPT_PRESET_NEXT_ID）和「目标」下拉的选项
        来源 —— 静默失败的话, 用户会看到「注册表里有 SCRIPT_PRESET_9、枚举里没有」这种
        编译错误, 而且 id 会开始被回收。
        """
        self.h.write_text(
            "\n".join(ln for ln in self.h_text().split("\n")
                      if not map_codec.RE_PRESET_GEN_END.match(ln)),
            encoding="utf-8", newline="")

        ok, resp, status = self.save(server.parse_map())
        self.assertTrue(ok, resp)                      # map.c 那边是成功的
        self.assertIn("map.h 没能更新", resp.get("warning", ""))


class TestMarkerBalance(PresetSandbox):
    """嵌套容器 IF 的配对校验（服务端保存前硬拦, 错误码 `unbalanced_if`）。

    ⚠️ 为什么不配对的表这么危险: C 编译期**不报错**（三块标记只是三条普通数组项）。
       固件那套位置配对会从「如果」一路扫到表尾、把本帧收掉 —— 现象是「后半张路线静默
       不执行」, 比编译错误难查得多。所以这道理必须拦在**写盘之前**。

    每条都断言「400 + 两个文件的 mtime 与内容都不变」: 被拒的保存不许留下半个字节。"""

    def _items(self, *its):
        m = server.parse_map()
        first = dict(m["presets"][0])
        first["items"] = list(its)
        return self.payload([first], active=first["ident"])

    def _assert_rejected(self, payload):
        before_c, before_h = self.c_text(), self.h_text()
        ok, resp, status = self.save(payload)
        self.assertFalse(ok, resp)
        self.assertEqual(status, 400)
        self.assertEqual(resp.get("error"), "unbalanced_if", resp)
        self.assertEqual(self.c_text(), before_c, "被拒的保存却动了 map.c")
        self.assertEqual(self.h_text(), before_h, "被拒的保存却动了 map.h")

    def test_stray_end_if_is_rejected(self):
        self._assert_rejected(self._items(self.if_close()))

    def test_missing_end_if_is_rejected(self):
        self._assert_rejected(self._items(self.if_open(), self.dead()))

    def test_double_else_is_rejected(self):
        self._assert_rejected(self._items(self.if_open(), self.if_else(), self.dead(),
                                         self.if_else(), self.if_close()))

    def test_nested_balanced_is_accepted(self):
        ok, resp, status = self.save(self._items(
            self.if_open(),                                                    # 0 外层如果
            self.if_open(cond="SCRIPT_COND_COLOR"), self.dead(),               # 1 内层, then 分支
            self.if_else(), self.dead(), self.if_close(),                      # 3 内层否则 + 收口
            self.dead(),                                                       # 6 外层 then 继续
            self.if_else(), self.dead(), self.if_close()))                     # 7 外层否则 + 收口
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)

    def test_whole_block_disabled_still_balances(self):
        """整块禁用 = 块内每一行都置 disabled（控制台上眼睛走的那条路）→ 真数组里三块
        标记一起消失, 照样平衡。"""
        its = [self.if_open(), self.if_else(), self.dead(), self.if_close()]
        for it in its:
            it["disabled"] = True
        ok, resp, status = self.save(self._items(*its))
        self.assertTrue(ok, resp)
        self.assertEqual(status, 200)

    def test_half_disabled_block_is_rejected(self):
        """只禁用「如果」而留着「结束如果」→ 数组里只剩一块落单的 END_IF。

        ⚠️ 这条正是「按 disabled 过滤后再配对」在起作用; 不过滤的话它会被放行, 而写出来
           的数组真的不平衡。"""
        a = self.if_open()
        a["disabled"] = True
        self._assert_rejected(self._items(a, self.dead(), self.if_close()))

    @staticmethod
    def _section(title, wrap):
        return {"id": "i0", "kind": "section",
                "lines": ["    /* ===== %s ===== */" % title],
                "disabled": bool(wrap), "wrap": bool(wrap), "title": title, "text": ""}

    def test_wrapped_section_hiding_a_marker_is_rejected(self):
        """分区包裹（`/* */`）把配对的一半罩住: 收口标记仍是活的 C 代码, 而「如果」被注释掉了。

        ⚠️ 判据必须与 serialize_map **同一套**「哪些行真的进数组」的口径 —— 漂移了就会出现
           「校验说平衡、写出来的 C 却不平衡」。所以下面**拿序列化结果反过来核对**:
           包裹的 `/*` 起于第一个分区、`*/` 落在第二个分区之前, 于是「如果」在注释里而
           「结束如果」在外面。
        ⚠️ 包裹区**整段罩住**一个块（`/*` 与 `*/` 都在块外）时是**合法**的 —— 那就是「整块
           禁用」, 数组里三块标记一起消失。所以这里必须用**两个分区**把区间切成两半,
           否则测不出东西（我第一版就写成了那个测不出东西的形状）。
        """
        its = [self._section("上半", wrap=True), self.if_open(), self.dead(),
               self._section("下半", wrap=False), self.dead(), self.if_close()]
        self._assert_rejected(self._items(*its))

        cat = {c["type"]: c for c in server.parse_catalog()}
        text = "\n".join(map_codec.serialize_map(its, cat))
        i_if = text.index("STEP_IF,")
        i_end = text.index("STEP_END_IF")
        self.assertLess(i_if, i_end, "两条标记的顺序不对 —— 前提不成立")
        self.assertIn("    */", text[i_if:i_end],
                      "包裹没有在收口标记之前闭合 —— 那样它就被一起罩住了, 不该判失衡")


class TestNestedIfFrontend(unittest.TestCase):
    """分支归属与缩进**只在 app.js 里推导** —— 服务端一行都看不见, 只能在 node 里钉。"""

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def test_marker_names_agree_with_the_catalog(self):
        """app.js 里的三个字面量必须与 app_script.h 的枚举成员一模一样。

        改一处忘另一处的话, 画布会**静默不缩进**（当普通积木画）, 而谁也不报错。"""
        js = srccheck.read_source("static/app.js")
        cat = {c["type"] for c in server.parse_catalog()}
        for name in ("STEP_IF", "STEP_ELSE", "STEP_END_IF"):
            with self.subTest(step=name):
                self.assertIn(name, cat, "app_script.h 里没这个成员")
                self.assertIn("'%s'" % name, js, "app.js 没跟上枚举改名")

    def test_depth_and_else_are_derived_from_the_flat_list(self):
        out = self._js("""
function mk(id, step) { return {id: id, kind: 'step', step: step}; }
var items = [mk('a','STEP_IF'), mk('b','STEP_DEAD'), mk('c','STEP_IF'), mk('d','STEP_DEAD'),
             mk('e','STEP_ELSE'), mk('f','STEP_DEAD'), mk('g','STEP_END_IF'), mk('h','STEP_DEAD'),
             mk('i','STEP_ELSE'), mk('j','STEP_DEAD'), mk('k','STEP_END_IF'), mk('l','STEP_DEAD')];
var st = scanIfStructure(items, 'P1');
console.log('D=' + items.map(function (x) { return st.depth.get(x.id); }).join(','));
console.log('E=' + [st.hasElse.get('a'), st.hasElse.get('c')].join(','));
console.log('S=' + JSON.stringify([ifSpanIdx(items, 0), ifSpanIdx(items, 2)]));
console.log('X=' + JSON.stringify([elseSpanIdx(items, 4), elseSpanIdx(items, 8)]));
console.log('K=' + JSON.stringify([st.keyOf.get('a'), st.keyOf.get('c')]));
/* 「否则」可省略: 没有 else 的块 hasElse 为 false, 收口仍然找得到 */
var st2 = scanIfStructure([mk('x','STEP_IF'), mk('y','STEP_DEAD'), mk('z','STEP_END_IF')]);
console.log('N=' + [st2.hasElse.get('x') === true,
                     st2.endOfIf.get('x') === 'z',
                     st2.depth.get('y')].join(','));
""")
        self.assertIn("D=0,1,1,2,1,2,1,1,0,1,0,0", out)
        self.assertIn("E=true,true", out)
        self.assertIn("S=[[0,10],[2,6]]", out)
        self.assertIn("X=[[5,5],[9,9]]", out)
        self.assertIn("N=false,true,1", out)
        # 折叠键 = 预设 ident + 「第几个『如果』块」的链 —— 增删普通积木不会让它错位
        self.assertIn('K=["P1#0","P1#0.0"]', out)

    def test_collapsed_by_default_hides_the_whole_block(self):
        """「如果」块**默认折叠**: 块内积木与两块标记行都不画, 画布上只剩一个头。

        同时钉住: 折叠是**独立于 dirty 的视图状态**（不改 map.c 的保存状态）。
        """
        out = self._js("""
function mk(id, step) { return {id: id, kind: 'step', step: step}; }
S.items = [mk('p','STEP_DEAD'), mk('a','STEP_IF'), mk('b','STEP_DEAD'), mk('c','STEP_ELSE'),
           mk('d','STEP_DEAD'), mk('e','STEP_END_IF'), mk('f','STEP_DEAD')];
var st = scanIfStructure(S.items, 'P');
console.log('HID=' + JSON.stringify(['p','a','b','c','d','e','f'].map(function (i) {
  return hiddenIfIds(S.items, st).has(i);
})));
S.ifExpanded = new Set(['P#0']);
console.log('HID2=' + JSON.stringify(['a','b','c','d','e','f'].map(function (i) {
  return hiddenIfIds(S.items, st).has(i);
})));
console.log('DEFAULT=' + (S.ifExpanded.size === 0 || true));
""")
        # 折叠: 只有头可见 → b/c/d/e 全藏; p/f 在块外
        self.assertIn("HID=[false,false,true,true,true,true,false]", out)
        # 展开后: 全可见
        self.assertIn("HID2=[false,false,false,false,false,false]", out)

    def test_collapse_state_survives_a_reload(self):
        """折叠状态存在 localStorage（per 浏览器）—— **刷新不改变折叠行为**。

        ⚠️ 为什么是 localStorage 而不是服务端文件: 折叠是视图偏好, 队友的页面不该跟着变,
           而且服务端的文件会被 `/api/rev` 多客户端同步当成「远端改动」触发重载。
        """
        out = self._js("""
var store = {};
globalThis.localStorage = {
  getItem: function (k) { return Object.prototype.hasOwnProperty.call(store, k) ? store[k] : null; },
  setItem: function (k, v) { store[k] = String(v); },
  removeItem: function (k) { delete store[k]; } };
function mk(id, step) { return {id: id, kind: 'step', step: step}; }
S.items = [mk('a','STEP_IF'), mk('b','STEP_DEAD'), mk('e','STEP_END_IF')];
S.presets = [{ ident: 'P', label: 'p', items: S.items }];
S.activeIdx = 0;
S.ifExpanded = new Set();
/* ① 默认折叠 → 点一下三角展开 → 写进 localStorage */
var head = { id: 'a', kind: 'step', step: 'STEP_IF', tag: '如果', params: ['SCRIPT_COND_QR'] };
S.ifStruct = scanIfStructure(S.items, 'P');
console.log('BEFORE=' + ifCollapsed(head));
/* setIfExpanded = 状态 + 落盘（不含重绘, 所以 node 桩里也能测） */
setIfExpanded(head, true);
console.log('AFTER=' + ifCollapsed(head));
console.log('SAVED=' + (store['gks.map.ifExpanded'] || ''));
setIfExpanded(head, false);
console.log('RE_COLLAPSED=' + ifCollapsed(head));
setIfExpanded(head, true);
/* ② 模拟刷新: 清掉内存态, 再走一遍"加载时读 localStorage"那条路 */
S.ifExpanded = new Set();
(function reload() {
  var arr = JSON.parse(localStorage.getItem('gks.map.ifExpanded') || 'null');
  if (Array.isArray(arr)) S.ifExpanded = new Set(arr);
})();
console.log('RELOADED=' + ifCollapsed(head));
""")
        self.assertIn("BEFORE=true", out)          # 默认折叠
        self.assertIn("AFTER=false", out)          # 点三角 = 展开
        self.assertIn('SAVED=["P#0"]', out)        # 键 = 预设 ident + 路径
        self.assertIn("RELOADED=false", out)       # 刷新后仍然是展开的

    def test_collapse_does_not_mark_the_route_dirty(self):
        """折叠**不是** map.c 的改动 —— 置脏会让「保存」亮起来, 还会让 `/api/rev` 那条
        「有未保存改动」的提示条常驻。"""
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function toggleIfCollapsed\(it\) \{[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没在 app.js 里找到 toggleIfCollapsed")
        self.assertNotIn("markDirty", m.group(0),
                         "折叠是视图偏好, 不该把路线标成「改过」")

    def test_whole_block_drag_span(self):
        """拖「如果」搬的是**整块**（头 + 两个分支 + 两块标记）—— 只搬头那一行会立刻失衡。

        区间连续才安全: 位置式嵌套只看相对顺序, 整段原样搬走之后仍然配对。
        非「如果」的积木返回 null → `makeDnd` 退化成「只搬它自己」（普通积木行为不变）。"""
        out = self._js("""
function mk(id, step) { return {id: id, kind: 'step', step: step}; }
S.items = [mk('p','STEP_DEAD'), mk('a','STEP_IF'), mk('b','STEP_DEAD'), mk('c','STEP_ELSE'),
           mk('d','STEP_DEAD'), mk('e','STEP_END_IF'), mk('f','STEP_DEAD')];
console.log('S=' + JSON.stringify([ifSpanIds('a'), ifSpanIds('b'), ifSpanIds('p')]));
""")
        self.assertIn('S=[["a","b","c","d","e"],null,null]', out)

    def test_stray_markers_do_not_throw(self):
        """手改过的 map.c（多一块「结束如果」）也要能画出来 —— 拦保存是服务端的事,
        画布这边的失职是「白屏」, 那用户连修的机会都没有。"""
        out = self._js("""
var items = [{id:'a', kind:'step', step:'STEP_END_IF'}, {id:'b', kind:'step', step:'STEP_IF'}];
var st = scanIfStructure(items);
console.log('D=' + [st.depth.get('a'), st.depth.get('b')].join(','));
console.log('LEN=' + items.length);
""")
        self.assertIn("D=0,0", out)
        self.assertIn("LEN=2", out)

    def test_if_block_is_created_as_three_rows(self):
        """拖一块「如果」必须一次落三行 —— 只落一块 = 立刻失衡, 服务端保存会拒。"""
        out = self._js("""
S.catByType = { STEP_IF: {type: 'STEP_IF', desc: '如果: 条件不成立走否则分支',
                          primitive: 'vScriptIf', params_f: [], params_i: ['cond'],
                          params_meta: [{name: 'cond', label: '条件', unit: '', tip: '',
                                         options: [{value: 'SCRIPT_COND_QR', label: '二维码'}]}]} };
var made = createItemsFromCatalog(S.catByType.STEP_IF);
console.log('M=' + JSON.stringify(made.map(function (x) { return [x.step, x.params]; })));
""")
        self.assertIn(
            'M=[["STEP_IF",["SCRIPT_COND_QR"]],["STEP_ELSE",[]],["STEP_END_IF",[]]]', out)

    def test_render_canvas_with_nested_if_does_not_throw(self):
        """真的把 `renderCanvas` 跑一遍（喂一个含嵌套 IF + 分支内分区标题的列表）。

        ⚠️ 为什么值得为它搭一个迷你 DOM: `renderIfHead` / `renderMarkerRow` 一旦抛异常,
        表现是**画布整片空白**（`renderCanvas` 从头就没跑完）—— 最糟的那种失败模式, 而且
        `node --check` 与服务端测试都看不见（顶层那一跑只是**定义**函数, 不调用它）。

        同时钉住「分支里的分区标题**不开新 zone**」: 照旧切的话「如果」头与它的「结束如果」
        会分居两个 `<section>`, 缩进与拖放落点全乱 —— 而那是个合法的 map.c。
        """
        out = self._js("""
function El(tag) {
  this.tagName = tag; this.children = []; this.parentNode = null;
  this.className = ''; this.attrs = {}; this.style = {}; this.textContent = '';
  this.classList = { add: function () {}, remove: function () {}, toggle: function () {},
                     contains: function () { return false; } };
}
El.prototype.appendChild = function (c) { this.children.push(c); if (c) c.parentNode = this; return c; };
El.prototype.insertBefore = function (c) { return this.appendChild(c); };
El.prototype.removeChild = function () {};
El.prototype.setAttribute = function (k, v) {
  this.attrs[k] = v;
  /* 浏览器会把 style 属性解析进 CSSOM（h() 就是用 setAttribute 设样式的,
     而 insertPlaceholder 反过来读 ref.style.marginLeft）—— 桩必须照做,
     否则「占位槽继承缩进」那条在测试里永远读到空串。 */
  if (k === 'style') {
    var self = this;
    String(v).split(';').forEach(function (decl) {
      var p = decl.split(':');
      if (p.length === 2) {
        var prop = p[0].trim().replace(/-([a-z])/g, function (m, ch) { return ch.toUpperCase(); });
        self.style[prop] = p[1].trim();
      }
    });
  }
};
El.prototype.addEventListener = function () {};
El.prototype.removeEventListener = function () {};
El.prototype.querySelector = function () { return new El('span'); };
El.prototype.querySelectorAll = function () { return []; };
El.prototype.getBoundingClientRect = function () {
  return { top: 0, bottom: 0, left: 0, right: 0, height: 0, width: 0 }; };
El.prototype.replaceWith = function () {};
El.prototype.closest = function () { return null; };
globalThis.Node = El;
document.createElement = function (t) { return new El(t); };
document.createTextNode = function () { return new El('#text'); };
var root = new El('div');
document.getElementById = function (id) { return id === 'canvas-inner' ? root : null; };

S.catByType = {
  STEP_IF: { type: 'STEP_IF', primitive: 'vScriptIf', params_f: [], params_i: ['cond'],
             params_meta: [{ name: 'cond', label: '条件', unit: '', tip: '',
                             options: [{ value: 'SCRIPT_COND_QR', label: '二维码' }] }] },
  STEP_DEAD: { type: 'STEP_DEAD', primitive: 'vScriptWaitKeyConfirm',
               params_f: [], params_i: [] } };
S.items = [
  { id: 'a0', kind: 'step', step: 'STEP_IF',   tag: '如果',     params: ['SCRIPT_COND_QR'] },
  { id: 'a1', kind: 'step', step: 'STEP_DEAD', tag: '停一下',   params: [] },
  { id: 'a2', kind: 'section', title: '分支里的分区', lines: ['    /* ===== 分支里的分区 ===== */'] },
  { id: 'a3', kind: 'step', step: 'STEP_IF',   tag: '如果(内)', params: ['SCRIPT_COND_QR'] },
  { id: 'a4', kind: 'step', step: 'STEP_DEAD', tag: '内层',     params: [] },
  { id: 'a5', kind: 'step', step: 'STEP_ELSE', tag: '否则(内)', params: [] },
  { id: 'a6', kind: 'step', step: 'STEP_DEAD', tag: '内层否',   params: [] },
  { id: 'a7', kind: 'step', step: 'STEP_END_IF', tag: '结束(内)', params: [] },
  { id: 'a8', kind: 'step', step: 'STEP_ELSE', tag: '否则',     params: [] },
  { id: 'a9', kind: 'step', step: 'STEP_DEAD', tag: '外层否',   params: [] },
  { id: 'b0', kind: 'step', step: 'STEP_END_IF', tag: '结束',   params: [] },
  { id: 'b1', kind: 'step', step: 'STEP_DEAD', tag: '块外',     params: [] }];
S.activeIdx = 0; S.selectedId = null; S.seq = 1;
S.presets = [{ ident: 'g_astScriptPresetX', label: 'x', items: S.items }];
/* ⚠️ 「如果」块**默认折叠** —— 这条测的是展开态的渲染, 所以先显式展开这两块
   （键 = 预设 ident + 路径, 见 scanIfStructure）。折叠态由 collapse 那几条单独测。 */
S.ifExpanded = new Set(['g_astScriptPresetX#0', 'g_astScriptPresetX#0.0']);

renderCanvas();
console.log('ZONES=' + root.children.length);
var rows = [];
(function walk(el) {
  for (var i = 0; i < el.children.length; i++) {
    var c = el.children[i];
    if (c.className && c.className.indexOf('block') === 0) {
      rows.push(c.className + '@' + (c.attrs['data-id'] || '') + '@' + (c.style.marginLeft || ''));
    }
    walk(c);
  }
})(root);
console.log('ROWS=' + JSON.stringify(rows));
""")
        self.assertIn("ZONES=1", out, "分支里的分区标题开了新 zone —— 缩进与落点会全乱")
        rows = json.loads(out.split("ROWS=", 1)[1].splitlines()[0])
        self.assertEqual(len(rows), 12, rows)

        def row(i):
            cls, did, margin = rows[i].split("@")
            return cls, did, margin

        # 外层「如果」在 0 层; 内层在 22px; 各自的分支内容再进一格
        self.assertEqual(row(0), ("block if-head", "a0", "0px"))
        self.assertEqual(row(1)[1:], ("a1", "22px"))
        self.assertEqual(row(2)[0], "block marker sec-inline", "分支内分区标题应当缩进渲染")
        self.assertEqual(row(2)[2], "22px")
        self.assertEqual(row(3)[:2], ("block if-head", "a3"))
        self.assertEqual(row(3)[2], "22px", "嵌套的「如果」应当与它自己的「否则」同层")
        self.assertEqual(row(4)[2], "44px", "内层分支的内容再进一格")
        self.assertEqual(row(5)[0], "block marker m-else")
        self.assertEqual(row(5)[2], "22px")
        self.assertEqual(row(7), ("block marker m-close", "a7", "22px"))
        self.assertEqual(row(8)[0], "block marker m-else")
        self.assertEqual(row(8)[2], "0px", "外层「否则」回到 0 层")
        self.assertEqual(row(9)[2], "22px")
        self.assertEqual(row(10), ("block marker m-close", "b0", "0px"))
        self.assertEqual(row(11)[2], "0px", "块外的普通积木回到 0 层")

        # ---- 同一个列表, 换成**全部折叠**再渲染一遍 ----
        # 外层一折, 里面的嵌套块整个都看不见 —— 只剩「外层头 + 块外那块死积木」两行。
        out2 = self._js("""
function El(tag) {
  this.tagName = tag; this.children = []; this.parentNode = null;
  this.className = ''; this.attrs = {}; this.style = {}; this.textContent = '';
  this.classList = { add: function () {}, remove: function () {}, toggle: function () {},
                     contains: function () { return false; } };
}
El.prototype.appendChild = function (c) { this.children.push(c); if (c) c.parentNode = this; return c; };
El.prototype.insertBefore = function (c) { return this.appendChild(c); };
El.prototype.removeChild = function () {};
El.prototype.setAttribute = function (k, v) { this.attrs[k] = v; };
El.prototype.addEventListener = function () {};
El.prototype.removeEventListener = function () {};
El.prototype.querySelector = function () { return new El('span'); };
El.prototype.querySelectorAll = function () { return []; };
El.prototype.getBoundingClientRect = function () {
  return { top: 0, bottom: 0, left: 0, right: 0, height: 0, width: 0 }; };
El.prototype.replaceWith = function () {};
El.prototype.closest = function () { return null; };
globalThis.Node = El;
document.createElement = function (t) { return new El(t); };
document.createTextNode = function () { return new El('#text'); };
var root = new El('div');
document.getElementById = function (id) { return id === 'canvas-inner' ? root : null; };

S.catByType = {
  STEP_IF: { type: 'STEP_IF', primitive: 'vScriptIf', params_f: [], params_i: ['cond'],
             params_meta: [{ name: 'cond', label: '条件', unit: '', tip: '',
                             options: [{ value: 'SCRIPT_COND_QR', label: '二维码' }] }] },
  STEP_DEAD: { type: 'STEP_DEAD', primitive: 'vScriptWaitKeyConfirm',
               params_f: [], params_i: [] } };
S.items = [
  { id: 'a0', kind: 'step', step: 'STEP_IF',   tag: '如果',   params: ['SCRIPT_COND_QR'] },
  { id: 'a1', kind: 'step', step: 'STEP_DEAD', tag: '停一下', params: [] },
  { id: 'a2', kind: 'section', title: '分支里的分区', lines: ['    /* ===== 分支里的分区 ===== */'] },
  { id: 'a3', kind: 'step', step: 'STEP_IF',   tag: '如果(内)', params: ['SCRIPT_COND_QR'] },
  { id: 'a4', kind: 'step', step: 'STEP_DEAD', tag: '内层',   params: [] },
  { id: 'a5', kind: 'step', step: 'STEP_ELSE', tag: '否则(内)', params: [] },
  { id: 'a6', kind: 'step', step: 'STEP_DEAD', tag: '内层否', params: [] },
  { id: 'a7', kind: 'step', step: 'STEP_END_IF', tag: '结束(内)', params: [] },
  { id: 'a8', kind: 'step', step: 'STEP_ELSE', tag: '否则',   params: [] },
  { id: 'a9', kind: 'step', step: 'STEP_DEAD', tag: '外层否', params: [] },
  { id: 'b0', kind: 'step', step: 'STEP_END_IF', tag: '结束', params: [] },
  { id: 'b1', kind: 'step', step: 'STEP_DEAD', tag: '块外',   params: [] }];
S.activeIdx = 0; S.selectedId = null; S.seq = 1;
S.presets = [{ ident: 'g_astScriptPresetX', label: 'x', items: S.items }];
S.ifExpanded = new Set();          /* 默认: 全部折叠 */

renderCanvas();
console.log('ZONES2=' + root.children.length);
var rows2 = [];
(function walk(el) {
  for (var i = 0; i < el.children.length; i++) {
    var c = el.children[i];
    if (c.className && c.className.indexOf('block') === 0) {
      rows2.push(c.className + '@' + (c.attrs['data-id'] || ''));
    }
    walk(c);
  }
})(root);
console.log('ROWS2=' + JSON.stringify(rows2));
console.log('SEL2=' + JSON.stringify(S.selectedId));
""")
        self.assertIn("ZONES2=1", out2)
        rows2 = json.loads(out2.split("ROWS2=", 1)[1].splitlines()[0])
        # 外层头（带 if-collapsed）+ 块外那块死积木; 内层头整个被折进去了
        self.assertEqual(len(rows2), 2, rows2)
        self.assertEqual(rows2[0], "block if-head if-collapsed@a0")
        self.assertEqual(rows2[1], "block@b1")
        self.assertIn("SEL2=null", out2)


class TestTuningRowShowsMacroName(unittest.TestCase):
    """调参行要在显示名旁边写**宏名**（`app_config.h` 里那个 `#define` 的名字）, 而改名只动显示名。

    动机很实际: 页面上看到「理想停止位中球列」, 想去源码里找它得知道它叫 `BALL_CX_CENTER` ——
    以前只能靠双击改名/右键元数据弹窗才能看见宏名。
    """

    # 迷你 DOM（同 TestNestedIfFrontend 那套, 外加一个保留文本的 createTextNode ——
    # 这条测试要读文字, 而默认桩把文本丢了）
    _DOM = """
function El(tag) {
  this.tagName = tag; this.children = []; this.parentNode = null;
  this.className = ''; this.attrs = {}; this.style = {}; this.textContent = '';
  this.classList = { add: function () {}, remove: function () {}, toggle: function () {},
                     contains: function () { return false; } };
}
El.prototype.appendChild = function (c) { this.children.push(c); if (c) c.parentNode = this; return c; };
El.prototype.insertBefore = function (c) { return this.appendChild(c); };
El.prototype.removeChild = function () {};
El.prototype.setAttribute = function (k, v) { this.attrs[k] = v; };
El.prototype.addEventListener = function () {};
El.prototype.removeEventListener = function () {};
El.prototype.querySelector = function () { return null; };
El.prototype.querySelectorAll = function () { return []; };
El.prototype.getBoundingClientRect = function () {
  return { top: 0, bottom: 0, left: 0, right: 0, height: 0, width: 0 }; };
El.prototype.replaceWith = function () {};
El.prototype.closest = function () { return null; };
globalThis.Node = El;
document.createElement = function (t) { return new El(t); };
document.createTextNode = function (s) { var e = new El('#text'); e.text = String(s); return e; };
var __byId = {};
globalThis.__el = function (id) { return __byId[id] || (__byId[id] = new El('div')); };
document.getElementById = function (id) { return globalThis.__el(id); };
function __txt(el) {
  var s = '';
  (function w(e) {
    for (var i = 0; i < e.children.length; i++) {
      var c = e.children[i];
      if (c.text !== undefined) s += c.text;
      w(c);
    }
  })(el);
  return s;
}
function __find(el, cls, out) {
  out = out || [];
  if (el.className && (' ' + el.className + ' ').indexOf(' ' + cls + ' ') >= 0) out.push(el);
  for (var i = 0; i < el.children.length; i++) __find(el.children[i], cls, out);
  return out;
}
function __kids(el) {
  var out = [];
  for (var i = 0; i < el.children.length; i++) {
    var c = el.children[i];
    out.push((c.className || c.tagName) + '=' + __txt(c));
  }
  return out;
}
"""

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"),
                                            self._DOM + probe)
        self.assertEqual(rc, 0, out)
        return out

    ITEMS = """
TUNE.groups = [{ title: '色球识别', items: [
  { name: 'BALL_CX_CENTER', display: '理想停止位中球列', visible: true, kind: 'num',
    unit: 'px', conv: '', value: '960', display_value: '960', note: '停车位基准' },
  { name: 'RAMP_ALIGN_SPEED', display: 'RAMP_ALIGN_SPEED', visible: true, kind: 'num',
    unit: '', conv: '', value: '80', display_value: '80', note: '' }]}];
TUNE.changed = {};
renderTuning();
var rows = __find(__el('tuning-canvas'), 'tune-item');
"""

    def test_display_name_is_followed_by_the_macro_name(self):
        out = self._js(self.ITEMS + """
console.log('ROWS=' + rows.length);
console.log('A=' + JSON.stringify(__kids(__find(rows[0], 'tune-name')[0])));
console.log('B=' + JSON.stringify(__kids(__find(rows[1], 'tune-name')[0])));
""")
        self.assertIn("ROWS=2", out)
        # 显示名、宏名、备注点 —— 就这三样, 且宏名在显示名**右边**
        self.assertIn('A=["tn-disp=理想停止位中球列","tn-macro=BALL_CX_CENTER","tune-note-dot=•"]', out)
        # 源码里没写显示名时 display 就是宏名（tuning.py 的兜底）→ **不重复写一遍**
        self.assertIn('B=["tn-disp=RAMP_ALIGN_SPEED"]', out)

    def test_macro_name_is_not_the_rename_field(self):
        """改名只动显示名 —— 双击进编辑时输入框的初值必须是 `it.display`,
        而不是「显示名 + 宏名」拼起来的一串（那样一保存就把宏名写进注释里了）。"""
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function renameParam\(it\) \{[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没在 app.js 里找到 renameParam")
        self.assertIn("editInPlace(el, it.display", m.group(0),
                      "改名的初值不是显示名 —— 宏名会被一起带进输入框")

    def test_macro_is_shown_for_the_real_config(self):
        """真实 app_config.h 上抽查一条: 有显示名的参数必须两种名字都给出来。

        这条盯的是「`display` 兜底成宏名」那个判据 —— 若哪天 tuning.py 改成把兜底值
        塞进别的字段, 这里会先红, 而不是页面上悄悄变成 `ROAD_ALIGN_SPEED ROAD_ALIGN_SPEED`。
        （2026-10 起抽查对象从 BALL_CX_CENTER 换成 ROAD_ALIGN_SPEED: 前者进了
         「标定」Tab 的托管清单, 不再出现在 /api/tuning —— 顺带钉住这条互斥约定。）
        """
        # ⚠️ 用 state() 而不是 parse_config(): 前者才是 /api/tuning 真正下发的形状
        #    （parse_config 的 display 是**解析出来的元数据**, 没写就是 None;
        #      state→public_item 才做「display 兜底成宏名」那一步）。
        state = tuning.state()
        items = [it for g in state["groups"] for it in g["items"]]
        names = {it["name"] for it in items}
        for n in tuning.CALIB_PARAMS:              # 标定 Tab 托管的 → 调参 Tab 必须没有
            self.assertNotIn(n, names, "%s 应由「标定」Tab 托管" % n)
        hit = [it for it in items if it["name"] == "ROAD_ALIGN_SPEED"]
        self.assertTrue(hit, "app_config.h 里没有 ROAD_ALIGN_SPEED —— 参数被改名了?")
        it = hit[0]
        self.assertNotEqual(it["display"], it["name"], "这条参数没有显示名, 测试前提不成立")
        self.assertTrue(it["display"], "显示名为空 —— 界面上会把宏名当显示名")
        # 有显示名的参数: 两种名字都要给出来（显示名 + 宏名）
        self.assertEqual(it["name"], "ROAD_ALIGN_SPEED")
        self.assertIn("速度", it["display"])


class TestRealMapPresetGuard(unittest.TestCase):
    """盯着**仓库里真实的** map.c / map.h —— 光靠沙箱测试发现不了「真实文件被改坏」。"""

    def test_every_table_is_in_the_registry(self):
        if not (server.MAP_C.is_file() and server.MAP_H.is_file()):
            self.skipTest("map.c / map.h 不存在")
        lines = server.MAP_C.read_text(encoding="utf-8").split("\n")
        tables = server._locate_presets(lines)
        reg = map_codec.parse_preset_registry(lines)
        self.assertEqual({b["ident"] for b in tables}, set(reg),
                         "map.c 里有表没进注册表（或反之）—— 跳转目标会指错")

    def test_every_registry_row_has_an_enum_member(self):
        if not (server.MAP_C.is_file() and server.MAP_H.is_file()):
            self.skipTest("map.c / map.h 不存在")
        reg = map_codec.parse_preset_registry(server.MAP_C.read_text(encoding="utf-8").split("\n"))
        h = server.MAP_H.read_text(encoding="utf-8")
        for e in reg.values():
            self.assertIn("SCRIPT_PRESET_%d" % e["id"], h, "map.h 少了这个枚举成员")
            self.assertIn("/* %s" % e["title"], h, "枚举标签与注册表标题对不上")

    def test_next_id_does_not_collide(self):
        """NEXT_ID 必须严格大于所有已用编号, 否则新预设会分到一个在用的号。"""
        if not (server.MAP_C.is_file() and server.MAP_H.is_file()):
            self.skipTest("map.c / map.h 不存在")
        reg = map_codec.parse_preset_registry(server.MAP_C.read_text(encoding="utf-8").split("\n"))
        if not reg:
            self.skipTest("map.c 里还没有预设注册表")
        rec = map_codec.parse_next_preset_id(server.MAP_H.read_text(encoding="utf-8").split("\n"))
        self.assertGreater(rec, max(e["id"] for e in reg.values()))

    def test_map_h_generated_block_is_locatable(self):
        if not server.MAP_H.is_file():
            self.skipTest("map.h 不存在")
        span = map_codec.find_marked(server.MAP_H.read_text(encoding="utf-8").split("\n"),
                                     map_codec.RE_PRESET_GEN_BEGIN,
                                     map_codec.RE_PRESET_GEN_END)
        self.assertIsNotNone(span, "map.h 缺生成块标记 —— 控制台没法再维护这张枚举")

    def test_real_map_markers_are_balanced(self):
        """真实 map.c 里每张预设的容器标记都配对 —— 迁移/手改漏一块「结束如果」就靠这条。

        ⚠️ 这是**唯一**能发现「文件级」标记失衡的地方: 沙箱测试喂的是自造数据。"""
        m = server.parse_map()
        self.assertTrue(m["presets"], "map.c 一个预设都没有?")
        for p in m["presets"]:
            with self.subTest(preset=p["ident"]):
                self.assertEqual(
                    map_codec.marker_balance_error(p["items"], p["label"]), "")

    def test_nested_if_does_not_consume_presets(self):
        """需求本体: 条件分支**不占预设**。

        扫码失败重试链原先靠四个整预设当 else 分支（17/18/20/21）。
        现在它们是嵌套 IF, 那四个预设应当已经不在注册表里 —— 谁把它们加回来（或把扫码
        改回跳转写法）, 这条先红。
        ⚠️ 2026-10-29 起「默认路线」表体整体换成了原「新路线」内容（调子预设的编排,
        正文不再有 IF）, 嵌套 IF 链住在「常规扫码」（预设39, 两层）与「安全扫码」
        （预设37, 一层）里 —— 断言跟着搬过去。"""
        reg = map_codec.parse_preset_registry(
            server.MAP_C.read_text(encoding="utf-8").split("\n"))
        titles = " ".join(e["title"] for e in reg.values())
        for gone in ("二维码第一次扫码失败", "二维码第二次扫码失败",
                     "二维码第三次扫码失败", "二维码第四次扫码失败"):
            self.assertNotIn(gone, titles, "扫码失败重试链又被拆成预设了 —— 应当写成嵌套 IF")
        p39 = [p for p in server.parse_map()["presets"]
               if p["ident"] == "g_astScriptPreset39"][0]
        opens = [it for it in p39["items"]
                 if it.get("step") == "STEP_IF" and not it.get("disabled")]
        self.assertGreaterEqual(len(opens), 2,
                                "常规扫码里那两处条件分支不见了")
        self.assertEqual(
            sum(1 for it in p39["items"]
                if it.get("step") == "STEP_END_IF" and not it.get("disabled")),
            len(opens), "「如果」与「结束如果」数量不等")


class MovesSandbox(unittest.TestCase):
    """「位移」Tab 的沙箱: moves.json / map.c 副本 / app_config.h 副本全部指向临时目录。

    变量清单与引用检查都吃**真实的** app_config.h / map.c 副本 —— 沙箱 fixture 自己造的
    数据自己当然对得上, 「段标题对得上 map.c 里的宏名」只有在真实文件上测才有意义。"""

    def setUp(self):
        if not (server.MAP_C.is_file() and tuning.APP_CONFIG.is_file()):
            self.skipTest("map.c / app_config.h 不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="movessbx-"))
        self.mjson = self.tmp / "moves.json"
        self.c = self.tmp / "map.c"
        self.cfg = self.tmp / "app_config.h"
        shutil.copy2(server.MAP_C, self.c)
        shutil.copy2(tuning.APP_CONFIG, self.cfg)
        for mod, attr, val in ((moves, "MOVES_JSON", self.mjson),
                               (moves, "MAP_C", self.c),
                               (tuning, "APP_CONFIG", self.cfg)):
            p = mock.patch.object(mod, attr, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def state(self):
        return moves.state()

    def cfg_text(self):
        return self.cfg.read_text(encoding="utf-8")


class SwitchesSandbox(unittest.TestCase):
    """「自定义」Tab 的沙箱: 全部源文件用副本 + 清单/ROOT 指 tmp。

    ⚠️ server.parse_enums 用 ROOT.glob 扫目录, ROOT 也必须 patch（tmp 里要建
    Application/ 子目录, 文件名与真件一致）—— 否则条件下拉来源的断言在测真件。"""

    def setUp(self):
        if not (server.SCRIPT_H.is_file() and server.SCRIPT_C.is_file()
                and tuning.APP_CONFIG.is_file() and server.MAP_C.is_file()):
            self.skipTest("源文件不存在")
        self.tmp = Path(tempfile.mkdtemp(prefix="swsbx-"))
        (self.tmp / "Application").mkdir()
        self.h = self.tmp / "Application" / "app_script.h"
        self.c = self.tmp / "Application" / "app_script.c"
        self.cfg = self.tmp / "Application" / "app_config.h"
        self.mp = self.tmp / "Application" / "map.c"
        for src, dst in ((server.SCRIPT_H, self.h), (server.SCRIPT_C, self.c),
                         (tuning.APP_CONFIG, self.cfg), (server.MAP_C, self.mp)):
            shutil.copy2(src, dst)
        for mod, attr, val in ((switches, "SWITCHES_JSON", self.tmp / "switches.json"),
                               (switches, "MAP_C", self.mp),
                               (switches, "SCRIPT_H", self.h),
                               (switches, "SCRIPT_C", self.c),
                               (tuning, "APP_CONFIG", self.cfg),
                               (server, "SCRIPT_H", self.h),
                               (server, "SCRIPT_C", self.c),
                               (server, "ROOT", self.tmp)):
            p = mock.patch.object(mod, attr, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def add_a(self):
        ok, resp, st = switches.add("SW_TEST_A", "测试开关甲", "任务", "", None)
        self.assertTrue(ok, resp)
        return resp["id"]

    def test_add_writes_macro_enum_getter_and_registry(self):
        self.add_a()
        h = self.h.read_text(encoding="utf-8")
        self.assertIn("SCRIPT_COND_SW_100 = 100,   /* 自定义:测试开关甲 */", h)
        c = self.c.read_text(encoding="utf-8")
        self.assertIn("static uint8_t c_ucCondSw100(void)   /* 测试开关甲 */", c)
        self.assertIn("{ SCRIPT_COND_SW_100, c_ucCondSw100 },", c)
        self.assertIn("\n#define SW_TEST_A", self.cfg.read_text(encoding="utf-8"))
        cond = server.cached_enums()["emScriptCondTdf"]
        self.assertEqual(cond[-1]["label"], "自定义:测试开关甲")

    def test_add_into_empty_section(self):
        """「自定义开关」段存在但还没有宏（初态）→ add 不得 IndexError（真踩过）。"""
        self.add_a()
        ok, resp, st = switches.add("SW_TEST_B", "测试开关乙", "", "", None)
        self.assertTrue(ok, resp)
        self.assertEqual(resp["id"], 101)

    def test_toggle_and_rename_sync_all_three_places(self):
        sid = self.add_a()
        ok, resp, st = switches.toggle(sid, "0", tuning.version())
        self.assertTrue(ok, resp)
        self.assertEqual(switches.state()["switches"][0]["value"], 0)
        ok, resp, st = switches.rename(sid, display="测试开关乙", note="改过",
                                       base_version=tuning.version())
        self.assertTrue(ok, resp)
        self.assertIn("SCRIPT_COND_SW_100 = 100,   /* 自定义:测试开关乙 */",
                      self.h.read_text(encoding="utf-8"))
        line = next(ln for ln in self.cfg.read_text(encoding="utf-8").split("\n")
                    if "SW_TEST_A" in ln)
        self.assertIn("测试开关乙", line)
        self.assertIn("改过", line)
        self.assertIn("@(0~1)", line)          # 全字段透传: 范围没被抹掉

    def test_delete_refused_while_referenced(self):
        sid = self.add_a()
        self.mp.write_text(
            "/* ===== 预设: 默认路线 ===== */\n"
            "const stScriptStepTdf g_astScriptPreset1[] = {\n"
            "    SCRIPT_STEP_I1 (STEP_IF, \"如果\", SCRIPT_COND_SW_100),\n"
            "};\n", encoding="utf-8", newline="")
        ok, resp, st = switches.delete(sid, tuning.version())
        self.assertFalse(ok)
        self.assertEqual(st, 400)
        self.assertEqual(resp["error"], "referenced")
        self.assertIn("SCRIPT_COND_SW_100", self.h.read_text(encoding="utf-8"))

    def test_delete_unrefed_and_next_id_monotonic(self):
        sid = self.add_a()
        ok, resp, st = switches.delete(sid, tuning.version())
        self.assertTrue(ok, resp)
        self.assertNotIn("SCRIPT_COND_SW_100", self.h.read_text(encoding="utf-8"))
        self.assertNotIn("SW_TEST_A", self.cfg.read_text(encoding="utf-8"))
        ok, resp, st = switches.add("SW_TEST_C", "删除后新建", "", "", None)
        self.assertTrue(ok, resp)
        self.assertEqual(resp["id"], 101)      # 只增不回退 —— 复用 100 会静默换义

    def test_rewrite_is_zero_diff(self):
        self.add_a()
        switches.add("SW_TEST_B", "测试开关乙", "", "", None)
        before_h = self.h.read_text(encoding="utf-8")
        before_c = self.c.read_text(encoding="utf-8")
        switches.write_generated(switches._load()["switches"])
        self.assertEqual(self.h.read_text(encoding="utf-8"), before_h)
        self.assertEqual(self.c.read_text(encoding="utf-8"), before_c)

    def test_duplicate_guards(self):
        self.add_a()
        self.assertEqual(switches.add("SW_TEST_A", "x", "", "", None)[2], 400)
        self.assertEqual(switches.add("bad name", "x", "", "", None)[2], 400)
        self.assertEqual(switches.add("SW_OTHER", "测试开关甲", "", "", None)[2], 400)


class TestMovesState(MovesSandbox):
    def test_state_lists_the_route_move_variables(self):
        """variables 段来自 app_config.h 的「路线位移变量」, 显示名/单位/负值都要对。"""
        st = self.state()
        names = [v["name"] for v in st["variables"]]
        self.assertIn("MOVE_LAMP_X", names)
        lamp = next(v for v in st["variables"] if v["name"] == "MOVE_LAMP_X")
        self.assertEqual(lamp["display"], "灯泡横向")
        self.assertEqual(lamp["num"], -490.0)
        self.assertEqual(lamp["unit"], "mm")
        self.assertTrue(st["image"]["exists"])          # docs/DefaultMap.JPG 已入库

    def test_variables_carry_full_meta_for_rename(self):
        """改译名走 /api/tuning/meta 是整条尾注释重写 —— conv/range/note 必须下发。"""
        lamp = next(v for v in self.state()["variables"] if v["name"] == "MOVE_LAMP_X")
        self.assertEqual(lamp["range"], {"lo": "-2000", "hi": "2000"})
        self.assertTrue(lamp["note"])


class TestMovesConfig(MovesSandbox):
    def test_save_roundtrip_zero_diff_and_conflict(self):
        ok, resp, st = moves.save_config(
            {"image": "docs/DefaultMap.JPG", "start": {"x": 10, "y": 20},
             "segments": [{"name": "MOVE_LAMP_X", "x1": 1, "y1": 2, "x2": 3, "y2": 4}]},
            None)
        self.assertTrue(ok, resp)
        v1 = resp["version"]
        cur = json.loads(self.mjson.read_text(encoding="utf-8"))
        self.assertEqual(cur["start"], {"x": 10, "y": 20})
        self.assertEqual(cur["segments"][0]["name"], "MOVE_LAMP_X")
        # 同内容再存 → 零 diff（mtime 不动）; 旧版本号 → 409
        ok2, resp2, st2 = moves.save_config(cur, v1)
        self.assertTrue(ok2, resp2)
        self.assertEqual(resp2["version"], v1)
        ok3, resp3, st3 = moves.save_config(cur, 1.0)
        self.assertFalse(ok3)
        self.assertEqual(st3, 409)

    def test_bad_entries_are_dropped_not_fatal(self):
        """坏条目（非数字坐标/坏名字）丢弃、越界路径拦下, 不该把整份保存拒绝。"""
        ok, resp, st = moves.save_config(
            {"image": "../outside.jpg", "start": {"x": "x", "y": 1},
             "segments": [{"name": "bad name!", "x1": 1, "y1": 2, "x2": 3, "y2": 4},
                          {"name": "MOVE_LAMP_X", "x1": "a", "y1": 2, "x2": 3, "y2": 4},
                          {"name": "MOVE_LAMP_X", "x1": 1, "y1": 2, "x2": 3, "y2": 4}]},
            None)
        self.assertTrue(ok, resp)
        cfg = json.loads(self.mjson.read_text(encoding="utf-8"))
        self.assertEqual(cfg["image"], "../outside.jpg")     # 只按原样存; 读取端 resolve_image 拦
        self.assertIsNone(cfg["start"])
        # 坏名字洗成 ""（条目还在）、坏坐标整条丢弃
        self.assertEqual(len(cfg["segments"]), 2)
        self.assertEqual(cfg["segments"][0]["name"], "")
        self.assertEqual(cfg["segments"][1]["name"], "MOVE_LAMP_X")
        self.assertIsNone(moves.resolve_image(cfg))          # .. 出界 → 拒绝读


class TestMovesVarDelete(MovesSandbox):
    def test_delete_refused_while_map_still_references(self):
        """真实 map.c 的「默认路线」引用着 MOVE_LAMP_X → 删除必须 400 且文件不动。"""
        before = self.cfg_text()
        ok, resp, st = moves.delete_var("MOVE_LAMP_X", tuning.version())
        self.assertFalse(ok)
        self.assertEqual(st, 400)
        self.assertEqual(resp["error"], "referenced")
        self.assertIn("默认路线", resp["message"])
        self.assertEqual(self.cfg_text(), before)

    def test_delete_succeeds_after_map_unrefers(self):
        """map.c 里的引用清掉后: 删宏整行消失 + 绑定线段解绑。"""
        self.c.write_text(
            "/* ===== 预设: 默认路线 ===== */\n"
            "const stScriptStepTdf g_astScriptPreset1[] = {\n"
            "//    SCRIPT_STEP_F1 (STEP_CHASSIS_MOVE_WORLD_X_WAIT, \"前往灯泡处\", MOVE_LAMP_X),\n"
            "};\n", encoding="utf-8", newline="")
        moves.save_config({"segments": [{"name": "MOVE_LAMP_X", "x1": 1, "y1": 2,
                                         "x2": 3, "y2": 4}]}, None)
        ok, resp, st = moves.delete_var("MOVE_LAMP_X", tuning.version())
        self.assertTrue(ok, resp)
        self.assertNotIn("MOVE_LAMP_X", self.cfg_text())
        self.assertIn("禁用行", resp["message"])              # 仅禁用行引用 → 放行但警告
        cfg = json.loads(self.mjson.read_text(encoding="utf-8"))
        self.assertEqual(cfg["segments"], [])                 # 绑定线段一并解绑

    def test_delete_unknown_var_is_404(self):
        ok, resp, st = moves.delete_var("NOT_A_MOVE_VAR", None)
        self.assertFalse(ok)
        self.assertEqual(st, 404)


class TestMovesVarAdd(MovesSandbox):
    def test_add_appends_to_existing_section(self):
        n0 = len(moves.variables())
        ok, resp, st = moves.add_var("MOVE_TEST_X", "测试位移", "-123", tuning.version())
        self.assertTrue(ok, resp)
        vs = moves.variables()
        self.assertEqual(len(vs), n0 + 1)
        v = vs[-1]
        self.assertEqual((v["name"], v["display"], v["num"], v["unit"]),
                         ("MOVE_TEST_X", "测试位移", -123.0, "mm"))
        self.assertIn("#define MOVE_TEST_X", self.cfg_text())
        self.assertIn("路线位移变量", self.cfg_text())        # 还是写进了同一个段

    def test_add_rejects_duplicate_and_bad_name(self):
        self.assertEqual(moves.add_var("MOVE_LAMP_X", "x", "0", None)[2], 400)
        self.assertEqual(moves.add_var("bad-name", "x", "0", None)[2], 400)


class TestRevisions(unittest.TestCase):
    """GET /api/rev 是「别人改了没有」的唯一来源（前端每 5s 轮询一次）。

    ⚠️ 三个键必须与各 Tab 保存响应里的 version **同源同义**（都是 st_mtime）——
       不然前端拿手里的 S.version / TUNE.version / ARM.version 去比会永远不相等,
       于是自己保存完立刻「自动重载」一次。
    """

    def test_keys_cover_the_three_tabs_plus_the_palette(self):
        rev = server.revisions()
        self.assertEqual(set(rev),
                         {"map", "tuning", "script_h", "arm", "chact", "oled", "calib"})
        self.assertEqual(set(rev["arm"]), {"c", "h"})
        self.assertEqual(set(rev["chact"]), {"c", "h"})   # 底盘动作组（2026-10 加）
        # oled 多一个 glyphs: 菜单保存时会往 tools/fontgen/glyphs.txt 追加新字,
        # 那也是一处「别人改了没有」的来源（手改字表同样要吃 409）。
        self.assertEqual(set(rev["oled"]), {"c", "h", "glyphs"})
        # calib 是标定 Tab 的 calib.json 戳; 它的第二个数据源 app_config.h 复用 tuning 键
        self.assertEqual(rev["calib"], calib.version())

    def test_stamps_match_what_the_write_endpoints_report(self):
        """逐项对着写入端点自己的版本戳比 —— 这是前端依赖的那条契约。"""
        rev = server.revisions()
        self.assertEqual(rev["map"], server.MAP_C.stat().st_mtime)
        self.assertEqual(rev["tuning"], tuning.version())
        self.assertEqual(rev["script_h"], server.SCRIPT_H.stat().st_mtime)
        arm = arm_action.parse_arm()["version"]
        self.assertEqual(rev["arm"], arm)
        self.assertEqual(rev["oled"], oled_menu.version())

    def test_script_h_is_here_on_purpose(self):
        """app_script.h 不能省: 「右键积木 → 编辑元数据」走 POST /api/meta 改它,
        那条路一个字节都不碰 map.c —— 少了这个键, 别人改的积木库永远同步不到。"""
        self.assertIn("script_h", server.revisions())

    def test_missing_file_does_not_raise(self):
        """文件不在（或读写竞态撞上）要回 None, 不能 500 ——
        500 会让每个客户端每 5s 弹一次错误。"""
        with mock.patch.object(server, "MAP_C", Path("no-such-file-xyz.c")):
            self.assertIsNone(server.revisions()["map"])

    def test_route_is_registered_as_an_exact_match_get(self):
        src = srccheck.read_source("server.py")
        self.assertIn('elif self.path == "/api/rev":', src)
        # 只读端点: 绝不能进 POST 白名单或 Origin 校验元组（进了就自相矛盾）
        whitelist, origin = srccheck.guard_tuples(src)
        self.assertNotIn("/api/rev", whitelist)
        self.assertNotIn("/api/rev", origin)

    def test_oled_routes_are_registered(self):
        """`/api/oled` 会回写仓库源文件（app_oled_menu.c + 字表）→
        白名单与 Origin 元组**两处都要登记**，漏一处那条路就没人看守。"""
        src = srccheck.read_source("server.py")
        self.assertIn('elif self.path == "/api/oled":', src)
        whitelist, origin = srccheck.guard_tuples(src)
        self.assertIn("/api/oled", whitelist)
        self.assertIn("/api/oled", origin)

    def test_calib_routes_are_registered(self):
        """`/api/calib` 写 calib.json, apply 还会改 app_config.h（仓库源文件）→
        同上, 白名单与 Origin 元组两处都要登记。"""
        src = srccheck.read_source("server.py")
        self.assertIn('elif self.path == "/api/calib":', src)
        whitelist, origin = srccheck.guard_tuples(src)
        self.assertIn("/api/calib", whitelist)
        self.assertIn("/api/calib", origin)


class TestLanBinding(unittest.TestCase):
    """绑 0.0.0.0（局域网队友要连）; 回环地址照旧自动开浏览器。"""

    def setUp(self):
        self.src = srccheck.read_source("server.py")

    def test_binds_all_interfaces(self):
        self.assertIn('BIND_HOST = "0.0.0.0"', self.src)
        self.assertIn('ThreadingHTTPServer((BIND_HOST, PORT), Handler)', self.src)

    def test_opens_loopback_in_the_browser(self):
        """本机别绕一圈网卡 —— 而且防火墙拦入站时回环照样能用。"""
        self.assertIn("webbrowser.open(LOOPBACK_URL)", self.src)

    def test_keeps_exclusive_binding(self):
        """双开必须报错。绑 0.0.0.0 之后更容易被忽略, 所以钉一条。"""
        self.assertIn("allow_reuse_address = False", self.src)

    def test_port_has_one_source_of_truth(self):
        """server.PORT 取自 host_tools.CONSOLE_PORT —— 两处各写一个 8410 迟早会对不上。"""
        self.assertEqual(server.PORT, host_tools.CONSOLE_PORT)
        self.assertNotIn("PORT = 8410", self.src)


class TestGuardTuplesExtraction(unittest.TestCase):
    """srccheck.guard_tuples 必须抠出**精确**的那两个元组。

    ⚠️ 这条是给一个真实的坑兜底: 原来的正则没锚行首, 而 `do_GET` 里的
       `elif self.path.startswith("/static/"):` 含有子串 "if self.path.startswith("，
       于是 re.search 从那儿开始、一路吃到 do_POST 的 `)) and` —— 抠出来的是一大段
       跨越 do_GET 的文本。后果不是报错, 而是两条 assertIn **永远通过**：
       路径明明被从 Origin 元组里删了, do_GET 的 elif 里还留着, 测试照样绿。
    """

    def test_origin_tuple_is_not_swallowed_by_do_get(self):
        _, origin = srccheck.guard_tuples(srccheck.read_source("server.py"))
        self.assertNotIn("do_GET", origin)
        self.assertNotIn("/static/", origin, "抠出来的是跨 do_GET 的一大段文本")
        # 元组本体就该是这五条（少一条 = 那个端点没做 Origin 校验）
        for path in ("/api/tools", "/api/tuning", "/api/bt", "/api/capture", "/api/arm"):
            self.assertIn(path, origin)

    def test_whitelist_tuple_is_exact(self):
        whitelist, _ = srccheck.guard_tuples(srccheck.read_source("server.py"))
        self.assertNotIn("do_GET", whitelist)
        self.assertIn("/api/map", whitelist)
        self.assertNotIn("/api/rev", whitelist, "只读端点不该在 POST 白名单里")

    def test_every_source_writing_endpoint_is_origin_guarded(self):
        """凡是**会回写仓库源文件**的端点都必须做 Origin 校验。

        ⚠️ `/api/map` 与 `/api/meta` 原先漏了（白名单里有、Origin 元组里没有）——
           跨站页面用 `mode:'no-cors'` + `Content-Type: text/plain` 发一个 JSON 体
           不会触发预检, 服务端又不看 Content-Type, 于是能直接重写 map.c/app_script.h。
           绑 0.0.0.0 之后控制台开在更多人的浏览器里, 这个洞的暴露面跟着变大。
        """
        whitelist, origin = srccheck.guard_tuples(srccheck.read_source("server.py"))
        for p in ("/api/map", "/api/meta", "/api/tuning", "/api/arm", "/api/calib"):
            self.assertIn(p, origin, "%s 会写仓库源文件却没做 Origin 校验" % p)


class TestCatalogCache(unittest.TestCase):
    """积木库缓存必须**感知外部修改** —— 队友 git pull / 手改 app_script.h 之后,
    前端的多客户端同步会正确地检测到 script_h 变了、去拉 /api/catalog, 并提示
    「积木库已被其他客户端修改, 已刷新」; 缓存不看磁盘的话, 它吐回的是**旧内容** ——
    用户看到提示却看不到任何变化, 极难查。

    全程沙箱: SCRIPT_H 指向临时副本, 不动仓库里的真实头文件。
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="catalogcache-"))
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        # 造一个最小可解析的 app_script.h（parse_catalog 只认 STEP_ 开头的映射注释与枚举）
        self.h = self.tmp / "app_script.h"
        self._write("STEP_DEAD", "1s 延时")
        p = mock.patch.object(server, "SCRIPT_H", self.h)
        p.start()
        self.addCleanup(p.stop)
        Handler.catalog_cache = None                  # 类属性是全局的, 用例之间要清干净
        self.addCleanup(setattr, Handler, "catalog_cache", None)

    def _write(self, name, desc):
        self.h.write_text(
            "typedef enum {\n"
            '    %s,  /* %s */\n'
            "} emScriptStepTypeTdf;\n"
            "\n"
            "/*\n"
            " *   %s   anI: ms=时长(ms)\n"
            " */\n" % (name, desc, name), encoding="utf-8", newline="")

    def _h(self):
        return Handler.__new__(Handler)                # 不启监听, 只借 _catalog()

    def _desc(self):
        return next(c["desc"] for c in self._h()._catalog() if c["type"] == "STEP_DEAD")

    def test_external_edit_is_picked_up(self):
        self.assertEqual(self._desc(), "1s 延时")
        self._write("STEP_DEAD", "改了说明")           # 模拟队友 git pull / 手改
        self.assertEqual(self._desc(), "改了说明",
                         "外部改了 app_script.h 而缓存不失效 —— 前端提示「已刷新」却看不到变化")
        # 步骤类型名也跟着变（新增/改名同样要反映出来）
        self._write("STEP_DELAY2", "另一个")
        types = [c["type"] for c in self._h()._catalog()]
        self.assertIn("STEP_DELAY2", types)

    def test_cache_is_actually_reused(self):
        """反向钉一遍: 磁盘没变时**不该**重复解析（否则每次请求全量扫头文件, 白花几十毫秒）。"""
        self._h()._catalog()
        with mock.patch.object(server, "parse_catalog",
                               side_effect=AssertionError("磁盘没变却又解析了一次")):
            self._h()._catalog()                      # 命中缓存, 不该再解析

    def test_key_covers_every_source_file(self):
        """键必须覆盖 parse_catalog/parse_enums 真正读到的每一处, 一处都不能少。"""
        names = {str(p) for p in server._parse_catalog_sources()}
        for want in (str(server.SCRIPT_H), str(server.SCRIPT_C),
                     str(arm_action.ACTION_C), str(arm_action.ACTION_H),
                     str(arm_action.TRASH_JSON)):
            self.assertIn(want, names, "缓存键漏了 %s —— 改它不会让积木库失效" % want)
        # ENUM_GLOBS 扫到的头文件也必须在（map.h 的预设枚举就是「IF 目标」下拉的来源）
        self.assertIn(str(server.MAP_H), names)

    def test_missing_file_is_not_an_error(self):
        with mock.patch.object(server, "SCRIPT_H", self.tmp / "没有这个文件.h"):
            key = server.catalog_key()
        self.assertTrue(key)
        self.assertTrue(any(v[1] is None for v in key),
                        "读不到的文件要记 (path, None, None) —— 文件从无到有也算一次变化")


class TestStaticFileGuard(unittest.TestCase):
    """`/static/` 的越界检查。

    ⚠️ 判据要用 relative_to 而不是 `str(fp).startswith(str(base))`: 后者没有结尾分隔符,
       指向**同级 static 打头的目录**的路径能通过检查（经典的 startswith 陷阱）。
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="staticguard-"))
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.base = self.tmp / "static"
        self.base.mkdir()
        (self.base / "app.js").write_text("// ok\n", encoding="utf-8")
        # 同级目录名以 static 打头 —— startswith 会把它误判成「在 static 里面」
        self.sibling = self.tmp / "staticX"
        self.sibling.mkdir()
        (self.sibling / "secret.txt").write_text("不该被读到\n", encoding="utf-8")
        # cwd 无关: server.STATIC_DIR 指向临时目录
        p = mock.patch.object(server, "STATIC_DIR", self.base)
        p.start()
        self.addCleanup(p.stop)

    def test_normal_file_is_served(self):
        fp = server._static_file("app.js")
        self.assertIsNotNone(fp)
        self.assertEqual(fp.name, "app.js")

    def test_sibling_directory_is_rejected(self):
        self.assertIsNone(server._static_file("../staticX/secret.txt"),
                          "前缀缝隙: 同级 staticX 目录被放行了")

    def test_dotdot_escape_is_rejected(self):
        self.assertIsNone(server._static_file("../../etc/passwd"))

    def test_absolute_path_is_rejected(self):
        self.assertIsNone(server._static_file(str(self.sibling / "secret.txt")))

    def test_missing_file_is_rejected(self):
        self.assertIsNone(server._static_file("nope.js"))

    def test_directory_is_rejected(self):
        """目录不是文件 —— 否则 is_file() 之外还要防遍历出目录清单。"""
        (self.base / "sub").mkdir()
        self.assertIsNone(server._static_file("sub"))


class TestPostBodyHardening(unittest.TestCase):
    """POST body 的三种坏输入各自给 4xx, 别一律 500（前端只能报「HTTP 500」, 无法自查）。"""

    def _post_status(self, content_length, body=b""):
        h = Handler.__new__(Handler)                  # 不启监听
        h.headers = {"Content-Length": content_length}
        h.path = "/api/map"
        h.rfile = io.BytesIO(body)
        sent = []
        with mock.patch.object(Handler, "_origin_ok", return_value=True), \
             mock.patch.object(Handler, "_send",
                               side_effect=lambda s, *a, **k: sent.append(s)):
            Handler.do_POST(h)
        return sent

    def test_non_numeric_content_length_is_400(self):
        self.assertEqual(self._post_status("abc"), [400])

    def test_oversized_body_is_413(self):
        self.assertEqual(self._post_status(str(server.MAX_BODY_BYTES + 1)), [413])

    def test_broken_json_is_400(self):
        self.assertEqual(self._post_status("3", b"{,}"), [400])

    def test_missing_body_is_400(self):
        self.assertEqual(self._post_status("0"), [400])


class TestTagHintAgreesWithTheServer(unittest.TestCase):
    """前端那个「保存时会自动替换」提示（app.js::mapTagIssues）**不能误报**。

    它是唯一一处有意写了两遍的规则: 转换的真值在服务端 map_codec.sanitize_tag, 前端
    只**检测**不转换（复刻转换必然漂移）。所以这里钉的是那条单方向的契约 ——
    **前端提示了, 服务端就一定会改写**。反过来不成立（只多/少空格时前端不吭声,
    那只是少一句提醒, 不会骗人）。
    """

    CASES = ['我"的"名字', "a\\\\b", "抓取//A", "a///b", "IF (条件", "抓取A)",
             "干净的积木名", "抓取(A)", "打开激光（继电器 JIGUANG）"]

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def test_hint_fires_only_when_the_server_would_rewrite(self):
        probe = "".join(
            "console.log('R=' + JSON.stringify([mapTagIssues(%s), %s]));\n"
            % (json.dumps(c, ensure_ascii=False), json.dumps(c, ensure_ascii=False))
            for c in self.CASES)
        out = self._js(probe)
        rows = [json.loads(ln[len("R="):]) for ln in out.splitlines() if ln.startswith("R=")]
        self.assertEqual(len(rows), len(self.CASES), out)
        for raw, (hint, val) in zip(self.CASES, rows):
            written = map_codec.sanitize_tag(val)
            if hint:
                self.assertNotEqual(written, val,
                                    "前端报了「会替换」但服务端一个字符都不改: %r" % raw)


class TestMultiClientSync(unittest.TestCase):
    """前端「多客户端同步」那一段（app.js）—— 判据的语义只有在这里能钉住。"""

    def _js(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def test_newer_means_strictly_newer(self):
        """服务端的戳**严格更新**才算「别人改了」。

        ⚠️ 写成 `!==` 就错了: 轮询是异步的, 保存之后还可能收到一个**保存之前**发出的
        响应（里面是旧戳）, `!==` 会把自己刚写的东西判成远端改动而白重载一次 ——
        这正是 NEW_FALSE 那个用例。
        """
        out = self._js(
            "console.log('NEW=' + revNewer('map', 101.0, 100.0));"
            "console.log('OLD=' + revNewer('map', 100.0, 101.0));"
            "console.log('TOL=' + revNewer('map', 100.0 + 1e-9, 100.0));"
            "console.log('SAME=' + revNewer('map', 100.0, 100.0));"
            "console.log('NULL=' + revNewer('map', null, 100.0));"
            "console.log('ARM_SAME=' + revNewer('arm', {c:2,h:2}, {c:2,h:2}));"
            "console.log('ARM_C=' + revNewer('arm', {c:3,h:2}, {c:2,h:2}));"
            "console.log('ARM_H=' + revNewer('arm', {c:2,h:9}, {c:2,h:2}));")
        for want in ("NEW=true", "OLD=false", "TOL=false", "SAME=false",
                     "NULL=false", "ARM_SAME=false", "ARM_C=true", "ARM_H=true"):
            self.assertIn(want, out)

    def test_clean_reloads_dirty_only_warns(self):
        """把三条语义真跑一遍（DOM 相关的几处打桩）:

          ① 干净 + 远端更新 → 调重载函数
          ② 同一份响应再判 → **什么都不做**（重载已把版本戳追平, 不能反复重载）
          ③ 有未保存改动   → **只挂提示条, 一个重载函数都不许调**（不覆盖用户的改动）

        这是整个特性的核心语义, 只做源码断言不够 —— 分支写反了照样「看着对」。
        ⚠️ 打桩的 loadAll 必须**照实更新 S.version**（真 loadAll 就干这个）: 不更新的话
           ② 永远不成立、会无限重载 —— 那是我打桩打错, 不是代码错。
        """
        probe = r"""
let hit = [];
toast = (m) => hit.push('toast');
revNoticeSync = () => hit.push('notice');
revCheckNow = () => {};
reloadCatalog = async () => hit.push('catalog');
loadAll = async () => { S.version = 200; hit.push('loadAll'); };   /* 照实追平版本戳 */
S.version = 100;
curTab = 'map';
const j = {map: 200, tuning: 5, script_h: 5, arm: {c: 5, h: 5}};
S.dirty = false;
revEvaluate(j);
setTimeout(() => {
  console.log('CLEAN=' + hit.join(','));
  hit = [];
  revEvaluate(j);                 /* ② 重载完 S.version=200 → 追平, 不该有动作 */
  setTimeout(() => {
    console.log('AGAIN=[' + hit.join(',') + ']');
    hit = [];
    S.version = 100; S.dirty = true;
    revEvaluate(j);               /* ③ 有未保存改动 → 只提示 */
    setTimeout(() => console.log('DIRTY=' + hit.join(',')), 5);
  }, 5);
}, 5);
"""
        out = self._js(probe)
        self.assertIn("CLEAN=loadAll,notice,toast", out)
        self.assertIn("AGAIN=[]", out, "已追平的版本戳不该触发第二次重载")
        self.assertIn("DIRTY=notice", out, "有未保存改动时**绝不能**自动重载")

    def test_reload_is_blocked_while_the_user_is_interacting(self):
        """光看 dirty 不够: 拖拽只在**落座**时才标记, 参数框是 onchange（失焦才触发）
        —— 期间重建 DOM 会把正在拖的东西/没提交的文本弄丢。

        所以三样都得在 revBlocked 里: 焦点在输入框、指针按着、正在播放动作。
        """
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function revBlocked\(tab\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 revBlocked")
        body = m.group(0)
        self.assertIn("document.activeElement", body)
        self.assertIn("ptrDown", body)
        self.assertIn("BT.playing", body)
        # 指针必须是「置位/复位」而不是计数 —— 计数漏一次就永久失灵（再也等不到重载）
        self.assertIn("document.addEventListener('pointerup', () => { ptrDown = false; }, true)",
                      js)
        self.assertIn("window.addEventListener('blur'", js)

    def test_only_the_current_tab_is_touched(self):
        """非当前 Tab 一概不碰: 隐藏的 pane 里量出来全是 0, 重建画布会留下错位占位槽。

        所以轮询只判当前 Tab, 切 Tab 时用缓存的那份响应补判（switchTab 末尾）。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("if (curTab !== 'map') return;", js)          # revCatalog
        self.assertIn("const tab = revEditing(curTab) && revLoaded(curTab) ? curTab : null;", js)
        self.assertIn("revCheckNow();", js)
        m = re.search(r"function switchTab\(id\)[\s\S]*?\n\}\n", js)
        self.assertIn("revCheckNow();", m.group(0), "switchTab 末尾没补判")

    def test_poll_starts_once_and_hits_the_revision_endpoint(self):
        js = srccheck.read_source("static/app.js")
        self.assertEqual(js.count("setInterval(revPoll,"), 1)
        self.assertIn("'/api/rev'", js)
        self.assertIn("revInit();", re.search(r"function init\(\)[\s\S]*?\n\}\n",
                                              js).group(0))

    def test_notice_dom_exists(self):
        """提示条必须自带「重载」按钮 —— 机械臂 Tab 里**本来没有任何重载入口**,
        409 时用户除了 Ctrl+F5 无路可走。"""
        html = srccheck.read_source("static/index.html")
        css = srccheck.read_source("static/style.css")
        for el in ('id="notice"', 'id="notice-text"', 'id="notice-reload"',
                   'id="notice-dismiss"'):
            self.assertIn(el, html)
        self.assertIn("#notice", css)
        self.assertIn("#notice.hidden { display: none; }", css)

    def test_arm_409_keeps_the_server_message(self):
        """服务端 409 的正文里写好了中文说明, 不能丢掉只剩「HTTP 409」。"""
        js = srccheck.read_source("static/app.js")
        m = re.search(r"async function armCall\(action, extra\)[\s\S]*?\n\}\n", js)
        body = m.group(0)
        self.assertIn("j.message || j.error", body)
        self.assertIn("e.status = res.status", body)
        self.assertIn("if (e.status === 409) revPoll();", js)


class TestPresetReentrantMark(unittest.TestCase):
    """「可重入的任务」标记（2026-10-18 加, 预设栏勾选框）:
    map.h 枚举注释落 （可重入）, parse_preset_reentrant 读回; 带它就不再追加
    不可跳入 标记（作者显式声明可被 STEP_SCRIPT_CALL 调入, 压过启发式）。"""

    def test_header_suppresses_unsafe_when_reentrant(self):
        entries = [{"ident": "g_astScriptPreset7", "title": "看路", "id": 7,
                    "unsafe": True, "reentrant": True}]
        joined = "\n".join(map_codec.serialize_preset_header(entries, 8))
        self.assertIn("（可重入）", joined)
        self.assertNotIn("（不可跳入）", joined)

    def test_unsafe_mark_still_applies_without_reentrant(self):
        entries = [{"ident": "g_astScriptPreset7", "title": "看路", "id": 7,
                    "unsafe": True, "reentrant": False}]
        joined = "\n".join(map_codec.serialize_preset_header(entries, 8))
        self.assertIn("（不可跳入）", joined)
        self.assertNotIn("（可重入）", joined)

    def test_reentrant_round_trips_through_map_h_lines(self):
        lines = map_codec.serialize_preset_header(
            [{"ident": "g_astScriptPreset7", "title": "看路转弯", "id": 7,
              "unsafe": True, "reentrant": True}], 8)
        self.assertEqual(map_codec.parse_preset_reentrant(lines), {7})
        lines2 = map_codec.serialize_preset_header(
            [{"ident": "g_astScriptPreset2", "title": "默认路线", "id": 2,
              "unsafe": False, "reentrant": False}], 8)
        self.assertEqual(map_codec.parse_preset_reentrant(lines2), set())


if __name__ == "__main__":
    unittest.main(verbosity=2)
