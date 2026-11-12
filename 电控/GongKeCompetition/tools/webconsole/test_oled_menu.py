"""tools/webconsole/oled_menu.py 的单元测试 —— **全程沙箱, 不碰仓库真实文件**。

跑法::

    python tools/webconsole/test_oled_menu.py

沙箱手法照 ``test_arm_action.py`` / ``test_tuning.py``: 把受管的三个文件复制到临时
目录, 再用 ``mock.patch.object`` 把模块级常量指过去。``_run_py``（会去调
``gen_font.py`` 重写 ``Core/Src/OLED_Data.c``）一律 mock 掉。

⚠️ 这里最要紧的两条不是「功能对不对」而是**契约**:

* ``test_noop_save_is_byte_identical_and_untouched`` —— 无操作保存必须**字节级零 diff
  且 mtime 不动**。白写一次会让另一个 Tab 的下次保存吃 409。
* ``test_real_file_roundtrips_through_the_write_path`` —— 把真实文件解析出来的模型
  **原样走一遍写盘路径**。只测「无操作保存」是**不够**的: 那样走不到序列化分支,
  只在写盘路径上炸的错会全部溜过去（照 ``test_tuning.py`` 那条教训）。
"""

import os
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import oled_menu as O                                       # noqa: E402


def _read(p):
    return Path(p).read_text(encoding="utf-8")


class Sandbox(unittest.TestCase):
    """三个受管文件全部沙箱化。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_oled_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.c = self.tmp / "app_oled_menu.c"
        self.h = self.tmp / "app_oled_menu.h"
        self.glyphs = self.tmp / "glyphs.txt"
        shutil.copy2(O.MENU_C, self.c)
        shutil.copy2(O.MENU_H, self.h)
        shutil.copy2(O.GLYPHS_TXT, self.glyphs)
        for name, val in (("MENU_C", self.c), ("MENU_H", self.h),
                          ("GLYPHS_TXT", self.glyphs)):
            p = mock.patch.object(O, name, val)
            p.start()
            self.addCleanup(p.stop)
        # 别真去跑 gen_font.py / check_coverage.py
        self.run_py = mock.patch.object(O, "_run_py", return_value=(0, "", ""))
        self.m_run_py = self.run_py.start()
        self.addCleanup(self.run_py.stop)

    def model(self):
        return O.parse_region(_read(self.c))

    def save(self, model, base=None):
        return O.save(model, O.version() if base is None else base)


# ==================== 契约: 零 diff ====================

class TestRealFile(Sandbox):
    def test_real_file_parses_and_roundtrips(self):
        """真实文件: 解析 → 序列化必须**逐字节相同**（否则第一次保存就产生全文件 diff）。"""
        text = _read(self.c)
        self.assertEqual("\n".join(O._replace_region(text.split("\n"),
                                                     O.serialize_region(self.model()))), text)

    def test_real_file_passes_its_own_validation(self):
        self.assertEqual(O._check(self.model()), "")

    def test_real_file_has_no_missing_glyphs(self):
        """真实菜单的全部汉字都已在字表里 —— 这条挂了说明字表与菜单脱节了。"""
        self.assertEqual(O.missing_han(self.model()), [])

    def test_catalog_shape(self):
        m = self.model()
        # ⚠️ 表集合/页面数量/页面内容全是排菜单的人随时在改的（2026-10-18 一天里改了
        # 六七版: 增删子表、删菜单页…）—— 唯一的**跨文件契约**是前两页 = 状态页 + 
        # 二维码页（app_oled.c 的代码页按位置画, 见 OLED_PAGE_STATUS/QR）, 其余随便改。
        kinds = [p["kind"].replace("OLED_PAGEKIND_", "") for p in m["pages"]]
        self.assertGreaterEqual(len(kinds), 2)
        self.assertEqual(kinds[:2], ["STATUS", "QR"])
        self.assertTrue(all(k == "MENU" for k in kinds[2:]))
        labels = [i["label"] for i in m["lists"]["s_stListLine"]["items"]]
        self.assertTrue(labels)
        self.assertTrue(all(isinstance(x, str) and x for x in labels))
        # 「射靶测试」那张表（2026-10-09 加）若还在, 钉它的形状 —— ⚠️ 只在**存在**时钉:
        # 2026-10-18 排菜单的人把它挪进了子菜单（s_stListSub3）, 表可能随时被删/改名。
        if "s_stListShoot" in m["lists"]:
            self.assertEqual([i["label"] for i in m["lists"]["s_stListShoot"]["items"]],
                             ["开始射靶测试", "开关激光", "瞄准列", "收敛阈值"])
            self.assertEqual([i["arg"] for i in m["lists"]["s_stListShoot"]["items"]],
                             ["SCRIPT_PRESET_34", None, "OLED_QTY_SHOOT_CX",
                              "OLED_QTY_SHOOT_CONV_PX"])

    def test_generic_tune_rows_carry_param_names(self):
        """通用调参行（OLED_ITEM_P）: 名字进 `param`, `arg` 必须是 None（两种参数槽互斥）。

        ⚠️ 2026-10-18 起改为**机制扫查**而不是钉具体标签 —— 排菜单的人随时在重排/
        改名（一天三版）, 钉标签只会让每次改菜单都来改测试。这里对**全部表的所有行**查:
          · TUNE_PARAM 行必须带 param、不许带 arg;
          · TUNE_QTY 行必须带 arg、不许带 param;
          · 其余动作不许带 param（带 = 写进 C 就是死项/编不过, 校验另拦, 这里查形状）。
        另留一对具体锚（KP→LINE_KP）钉「名字真落在 param 上」这个契约。
        """
        m = self.model()
        for lname, lst in m["lists"].items():
            for it in lst["items"]:
                where = "%s 的「%s」" % (lname, it["label"])
                if it["act"] == "OLED_ACT_TUNE_PARAM":
                    self.assertTrue(it["param"], where + " 没带参数名")
                    self.assertIsNone(it["arg"], where + " 带了 arg")
                elif it["act"] == "OLED_ACT_TUNE_QTY":
                    self.assertIsNotNone(it["arg"], where + " 没带 arg")
                    self.assertIsNone(it["param"], where + " 带了 param")
                else:
                    self.assertIsNone(it["param"], where + " 只有通用调参行能带参数名")
        rows = {i["label"]: i for i in m["lists"]["s_stListPiTune"]["items"]}
        self.assertEqual(rows["KP"]["param"], "LINE_KP")

    def test_shoot_test_rows_carry_a_real_preset(self):
        """「开始射靶测试」那一项**必须带预设 id**（2026-10-23 现场踩过）。

        ⚠️ 机制扫查而不是钉表名: 排菜单的人随时把这一项挪进子菜单/改标签,
        这里只查「凡是 OLED_ACT_SHOOT_TEST 的行, arg 都得是一个真预设」。
        `ucArg = 0` 的症状是**按了没反应**: 固件查不到预设, 只往 Pi 打一条
        `MENU: task preset id not found`。
        """
        m = self.model()
        hits = [(n, it) for n in m["order"] for it in m["lists"][n]["items"]
                if it["act"] == "OLED_ACT_SHOOT_TEST"]
        for n, it in hits:
            where = "%s 的「%s」" % (n, it["label"])
            self.assertTrue(it["arg"], where + " 没带预设 id（固件 ucArg=0 → 查不到预设）")
            self.assertFalse(str(it["arg"]).endswith("_NONE"), where + " 选了 NONE 哨兵")


class TestNoOpSave(Sandbox):
    def test_noop_save_is_byte_identical_and_untouched(self):
        before_c, before_g = _read(self.c), _read(self.glyphs)
        st_c, st_g = self.c.stat(), self.glyphs.stat()

        ok, resp, status = self.save(self.model())

        self.assertTrue(ok)
        self.assertEqual(status, 200)
        self.assertFalse(resp["changed"])
        self.assertEqual(resp["glyphs_added"], [])
        self.assertEqual(_read(self.c), before_c)
        self.assertEqual(_read(self.glyphs), before_g)
        # 「没变就不写」—— mtime 也要一动不动
        self.assertEqual(self.c.stat().st_mtime_ns, st_c.st_mtime_ns)
        self.assertEqual(self.glyphs.stat().st_mtime_ns, st_g.st_mtime_ns)
        # 真·无操作: 一个子进程都不该起
        self.m_run_py.assert_not_called()

    def test_real_file_roundtrips_through_the_write_path(self):
        """把真实文件解析出来的模型原样走一遍写盘路径 —— 这才走得到序列化分支。"""
        m = self.model()
        for name in m["order"]:                    # 逐条把值原样回写自身
            for it in m["lists"][name]["items"]:
                it["label"] = it["label"]
                it["enabled"] = it["enabled"]
        before = _read(self.c)
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertEqual(_read(self.c), before)


class TestEditRoundTrip(Sandbox):
    def test_label_edit_lands_and_reparses(self):
        m = self.model()
        item = m["lists"]["s_stListTask"]["items"][0]
        item["label"] = "关机测试"
        item["edited"] = True

        ok, resp, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertTrue(resp["changed"])

        m2 = O.parse_region(_read(self.c))
        self.assertEqual(m2["lists"]["s_stListTask"]["items"][0]["label"], "关机测试")
        # 改过的行重新解析后不再带 edited（内容已落进 raw）
        self.assertFalse(m2["lists"]["s_stListTask"]["items"][0]["edited"])

    def test_disable_an_item(self):
        m = self.model()
        m["lists"]["s_stListLine"]["items"][0]["enabled"] = False
        m["lists"]["s_stListLine"]["items"][0]["edited"] = True
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        m2 = O.parse_region(_read(self.c))
        self.assertFalse(m2["lists"]["s_stListLine"]["items"][0]["enabled"])

    def test_reorder_items(self):
        m = self.model()
        items = m["lists"]["s_stListTask"]["items"]
        items[0], items[1] = items[1], items[0]
        for it in items:
            it["edited"] = True
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        m2 = O.parse_region(_read(self.c))
        self.assertEqual([i["label"] for i in m2["lists"]["s_stListTask"]["items"]][:2],
                         ["握手让出", "树莓派关机"])

    def test_add_a_new_menu_page(self):
        m = self.model()
        m["order"].append("s_stListExtra")
        m["lists"]["s_stListExtra"] = {
            "title": "扩展菜单", "array": "s_astOledMenuExtra", "edited": True,
            "items": [{"label": "占位", "act": "OLED_ACT_NOP", "enabled": True,
                       "arg": None, "sub": None, "edited": True}]}
        m["pages"].append({"kind": "OLED_PAGEKIND_MENU", "list": "s_stListExtra",
                           "edited": True})
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        m2 = O.parse_region(_read(self.c))
        self.assertIn("s_stListExtra", m2["order"])
        self.assertEqual(m2["pages"][-1]["list"], "s_stListExtra")


# ==================== 参数清单（「参数」弹窗的数据源） ====================

SMALL_BEAM = """\
static const stBeamParamTdf s_astBeamParam[] =
{
    { "gain_x",   "增益X", "视觉", "mm/px", BEAM_VAL_F32, 1u, 1u, 0.01f,  5.f, fGetGainX,  vSetGainX },
    { "err_now",  "实时误差", "视觉", "",    BEAM_VAL_F32, 0u, 0u, 0.00f,  0.f, fGetErr,   NULL,   "0=关|1=开" },
    { "flag_on",  "开关",   "底盘", "",    BEAM_VAL_U8,  1u, 0u, 0.00f,  1.f, _bgFlag,    _bsFlag },
};
"""

SMALL_RPI = """\
static const char s_acRpiCfgOpt0[] = "1=手动|3=自动";

const stRpiCfgParamTdf g_astRpiCfgParam[] =
{
    { "LINE_KP",   "巡线PID-Kp", "巡线", RPI_CFG_T_F32, NULL },
    { "CAM_EXP",   "相机曝光模式", "相机", RPI_CFG_T_I32, s_acRpiCfgOpt0 },
    { "QR_KNOWN",  "候选集",    "二维码", RPI_CFG_T_STR, NULL },
};
const uint16_t g_usRpiCfgParamNum = (uint16_t)(sizeof(g_astRpiCfgParam) / sizeof(g_astRpiCfgParam[0]));
"""


class TestParamCatalog(unittest.TestCase):
    """两张参数表的扫描: 行形状 / 类型映射 / 选项表 / 名字唯一性。

    ⚠️ 这里用**合成的小表**（不依赖真实表当前有多少项, 那数字天天在变）;
    另外一条 test_real_tables_are_fully_scanned 盯真实文件"一行不漏"。
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_cat_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.beam = self.tmp / "app_beam.c"
        self.rpi = self.tmp / "app_rpi.c"
        self.beam.write_text(SMALL_BEAM, encoding="utf-8")
        self.rpi.write_text(SMALL_RPI, encoding="utf-8")
        for name, val in (("BEAM_C", self.beam), ("RPI_C", self.rpi)):
            p = mock.patch.object(O, name, val)
            p.start()
            self.addCleanup(p.stop)

    def test_beam_rows_parse_with_metadata(self):
        rows = {r["name"]: r for r in O.parse_beam_params()}
        self.assertEqual(sorted(rows), ["err_now", "flag_on", "gain_x"])
        self.assertEqual(rows["gain_x"]["label"], "增益X")
        self.assertEqual(rows["gain_x"]["group"], "视觉")
        self.assertEqual(rows["gain_x"]["unit"], "mm/px")
        self.assertEqual(rows["gain_x"]["kind"], "float")
        self.assertFalse(rows["gain_x"]["ro"])
        self.assertEqual((rows["gain_x"]["min"], rows["gain_x"]["max"]), ("0.01", "5."))
        # 只读遥测（rw=0）也要收 —— 屏上"看着遥测"是这一档的正经用法
        self.assertTrue(rows["err_now"]["ro"])
        self.assertEqual(rows["err_now"]["options"], "0=关|1=开")
        self.assertEqual(rows["flag_on"]["kind"], "uint")

    def test_rpi_rows_parse_and_resolve_option_constants(self):
        rows = {r["name"]: r for r in O.parse_rpi_params()}
        self.assertEqual(sorted(rows), ["CAM_EXP", "LINE_KP", "QR_KNOWN"])
        self.assertEqual(rows["LINE_KP"]["kind"], "float")
        self.assertEqual(rows["CAM_EXP"]["options"], "1=手动|3=自动")   # 常量 → 文本
        self.assertEqual(rows["QR_KNOWN"]["kind"], "str")
        self.assertTrue(rows["QR_KNOWN"]["ro"])                        # str 型在屏上不能调
        self.assertEqual(rows["LINE_KP"]["src"], "pi")
        self.assertEqual(rows["LINE_KP"]["group"], "巡线")

    def test_catalog_names_are_the_union(self):
        cat = O.param_catalog()
        self.assertEqual(len(cat["names"]), 6)
        self.assertEqual(len(set(cat["names"])), 6)          # 唯一（保存白名单靠它）
        self.assertEqual(set(cat["names"]),
                         {"gain_x", "err_now", "flag_on", "LINE_KP", "CAM_EXP", "QR_KNOWN"})

    def test_missing_files_yield_an_empty_catalog(self):
        """读不到文件（还没拉代码/路径不对）时给空清单, 不抛 —— 页面得能打开。"""
        p = mock.patch.object(O, "BEAM_C", self.tmp / "没有这个文件.c")
        p.start()
        self.addCleanup(p.stop)
        cat = O.param_catalog()
        self.assertEqual(cat["beam"], [])
        self.assertEqual(len(cat["pi"]), 3)


class TestParamCatalogReal(unittest.TestCase):
    """真实文件那一条**不能**在合成沙箱里跑（setUp 已经把两个路径换掉了）——
    单独放一个类, 直接用模块常量（= 仓库里的真文件）。"""

    def test_real_tables_are_fully_scanned(self):
        """表本体的每一行都要被认出来（一行没认出来 = 弹窗里少一个可选项,
        而且不会报错 —— 只能靠这条盯住）。"""
        body = re.search(r"static const stBeamParamTdf\s+\w+\s*\[\s*\]\s*=\s*\{(.*?)\n\};",
                         _read(O.BEAM_C), re.S).group(1)
        rows = [ln for ln in body.split("\n") if ln.strip().startswith("{")]
        parsed = O.parse_beam_params()
        self.assertEqual(len(rows), len(parsed))
        self.assertEqual(len(set(r["name"] for r in parsed)), len(parsed))
        names = set(O.param_catalog()["names"])
        for probe in ("line_cam_db_deg", "line_err", "LINE_KP", "COLOR_INFER_EN"):
            self.assertIn(probe, names, probe)



    def bad(self, mutate):
        m = self.model()
        mutate(m)
        return O._check(m)


# ==================== 校验 ====================

class ValidationBase(Sandbox):
    def bad(self, mutate):
        m = self.model()
        mutate(m)
        return O._check(m)


def _nop(label, enabled=True):
    return {"label": label, "act": "OLED_ACT_NOP", "enabled": enabled,
            "arg": None, "sub": None, "param": None, "edited": True}


def _param_row(label, name, act="OLED_ACT_TUNE_PARAM"):
    """通用调参行（OLED_ITEM_P）: 参数槽是 `param`（字符串参数名）而不是 `arg`。"""
    return {"label": label, "act": act, "enabled": True,
            "arg": None, "sub": None, "param": name, "edited": True}


def _enabled_n(m, name):
    return len([i for i in m["lists"][name]["items"] if i["enabled"]])


def _grow_to(m, name, n):
    """把表里的启用项补到 n 个（只加不裁）—— 按**当前表长**算, 这样出厂表以后再加项
    也不会把这几条边界测试弄失效。"""
    items = m["lists"][name]["items"]
    cur = _enabled_n(m, name)
    assert cur < n, "表 %s 已经有 %d 个启用项, 补不到 %d" % (name, cur, n)
    items.extend(_nop("第%d项" % i) for i in range(n - cur))


class TestValidation(ValidationBase):
    def test_rejects_more_than_thirty_two_enabled_rows(self):
        """一页 32 项是上限（第 33 项起在车上既看不见也选不到）。"""
        err = self.bad(lambda m: _grow_to(m, "s_stListTask", 33))
        self.assertIn("超过 32 项上限", err)

    def test_thirty_two_enabled_items_pass_and_survive_a_round_trip(self):
        """32 项要通过校验, 且能往返写盘（证明序列化/解析确实吃得下 >4 行）。"""
        m = self.model()
        _grow_to(m, "s_stListTask", 32)
        self.assertEqual(O._check(m), "")
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        back = O.parse_region(_read(self.c))
        self.assertEqual(_enabled_n(back, "s_stListTask"), 32)

    def test_disabled_items_do_not_count_towards_the_limit(self):
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"].extend(
            _nop("第%d项" % i, enabled=False) for i in range(40)))
        self.assertEqual(err, "")

    def test_submenu_can_hold_thirty_two_items(self):
        """子菜单与页面同一条 32 项上限 ——「返回」是固件补的最后一条, **不占**额度。"""
        m = self.model()
        _grow_to(m, "s_stListPiTune", 32)
        self.assertEqual(O._check(m), "")            # 32 项 + 固件那条「返回」= 33 个条目
        err = self.bad(lambda mm: _grow_to(mm, "s_stListPiTune", 33))
        # SUB_MAX_ITEMS == MAX_ITEMS 时, 通用「每表 32 项」那条先命中（子表本身也在 order 里）
        self.assertIn("超过 32 项上限", err)

    def test_rejects_empty_list(self):
        err = self.bad(lambda m: m["lists"]["s_stListTask"].update({"items": []}))
        self.assertIn("一项都没有", err)

    def test_rejects_overwide_label(self):
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"][0].update(
            {"label": "一二三四五六七八九"}))
        self.assertIn("太宽", err)
        self.assertIn("120px", err)          # 预算从 128px 收到 120px（右侧 8px 让给滚动条）

    def test_a_120px_label_still_fits_but_128px_does_not(self):
        """零余量边界: 一行 128px, 最右 8px 是滚动条 ⇒ 标签预算正好 120px。
        现存最长的标签（`巡线状态:待实现`）就正好压在 120px 上 —— 再多一个字就装不下。"""
        self.assertEqual(O.text_width("巡线状态:待实现"), 120)
        err = self.bad(lambda m: m["lists"]["s_stListLine"]["items"].append(
            _nop("一二三四:五六七")))                       # 4 汉字 + ':' + 3 汉字 = 120px
        self.assertEqual(err, "")
        err = self.bad(lambda m: m["lists"]["s_stListLine"]["items"].append(
            _nop("一二三四五六七八")))                       # 8 汉字 = 128px
        self.assertIn("120px", err)

    def test_rejects_overwide_tune_label(self):
        # ⚠️ 整行 update（连 act 一起换成调参行）: 排菜单的人随时会改第 0 行是什么动作,
        # 只改 label 的话这行不再是调参行, 走 120px 预算就拦不住、测试假红/假绿。
        err = self.bad(lambda m: m["lists"]["s_stListTune"]["items"][0].update(
            {"label": "五个汉字啊", "act": "OLED_ACT_TUNE_QTY",
             "arg": "OLED_QTY_GYRO_TARGET"}))
        self.assertIn("调参行", err)

    def test_tune_rows_reserve_room_for_the_value_suffix(self):
        """4 汉字 + ":−180.0" = 120px 仍放得下; 5 汉字就不行了。"""
        self.assertEqual(O.text_width("陀螺目标") + O.text_width(":-180.0"), 120)
        err = self.bad(lambda m: m["lists"]["s_stListTune"]["items"][0].update(
            {"label": "陀螺目标角", "act": "OLED_ACT_TUNE_QTY",
             "arg": "OLED_QTY_GYRO_TARGET"}))
        self.assertIn("调参行", err)

    # ---- 通用调参行（OLED_ITEM_P / OLED_ACT_TUNE_PARAM, 2026-10 加）----

    def test_generic_tune_row_needs_a_param_name(self):
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"].append(
            _param_row("没选参数", None)))
        self.assertIn("没选参数", err)
        self.assertIn("通用调参行", err)

    def test_generic_tune_row_label_uses_the_tune_budget(self):
        """它也是调参行（固件要拼「:值」）⇒ 标签预算同样是 64px, 不是 120px。"""
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"].append(
            _param_row("五个汉字啊", "LINE_KP")))
        self.assertIn("调参行", err)
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"].append(
            _param_row("四个汉字", "LINE_KP")))           # 4 汉字 = 64px ✓
        self.assertEqual(err, "")

    def test_param_name_must_be_a_plain_identifier(self):
        """参数名会被原样内插进生成的 C 字符串 ⇒ 白名单是第一道门（防注入）。"""
        for bad_name in ("a-b", "1abc", "x\"), 0u }; void evil(", "名字"):
            err = self.bad(lambda m, n=bad_name: m["lists"]["s_stListPiTune"]["items"].append(
                _param_row("怪名字", n)))
            self.assertTrue(err, bad_name)
            self.assertIn("参数名", err, bad_name)

    def test_param_name_is_only_for_the_generic_tune_act(self):
        """别的动作码带了参数名 = 写错了动作码（那名字永远不会被读到）。"""
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"].append(
            _param_row("配错动作", "LINE_KP", act="OLED_ACT_NOP")))
        self.assertIn("只有 OLED_ACT_TUNE_PARAM", err)

    def test_param_name_must_exist_when_the_whitelist_is_given(self):
        """给了白名单（服务端会现扫 Beam 表 ∪ Pi 参数表）时, 名字必须在里面 ——
        不在 = 车上显示 "?" 并长鸣的死项。"""
        def real_names(m):
            return set(i["param"] for n in m["order"] for i in m["lists"][n]["items"]
                       if i.get("param"))

        m = self.model()
        good = real_names(m) | {"LINE_KP"}
        self.assertEqual(O._check(m, good), "")                 # 表里现成的名字都合法
        m["lists"]["s_stListPiTune"]["items"].append(_param_row("我编的", "NO_SUCH_PARAM"))
        self.assertEqual(O._check(m), "")                       # 不给白名单: 只查形状
        err = O._check(m, good)
        self.assertIn("都查不到", err)
        m2 = self.model()
        m2["lists"]["s_stListPiTune"]["items"].append(_param_row("真的", "LINE_KP"))
        self.assertEqual(O._check(m2, real_names(m2) | {"LINE_KP"}), "")

    def test_generic_tune_row_survives_a_round_trip(self):
        """改一个参数名 → 写盘 → 重新解析: 行形状（OLED_ITEM_P）与名字都要原样回来。"""
        m = self.model()
        # ⚠️ 动态挑一条调参行（表名/下标都随排菜单的人变）; 步进/映射清掉,
        #    期望回到最普通的 OLED_ITEM_P 形状。
        it = next(i for n in m["order"] for i in m["lists"][n]["items"]
                  if i["act"] == "OLED_ACT_TUNE_PARAM")
        it["param"] = "CAM_MAIN_EXPOSURE"
        it["step"] = None
        it["map"] = None
        it["edited"] = True
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        back = O.parse_region(_read(self.c))
        back_it = next(i for n in back["order"] for i in back["lists"][n]["items"]
                       if i["label"] == it["label"])
        self.assertEqual(back_it["param"], "CAM_MAIN_EXPOSURE")
        self.assertIn('OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "%s", 1u, "CAM_MAIN_EXPOSURE"),'
                      % it["label"], _read(self.c))

    def test_tune_row_with_step_round_trips_as_ps(self):
        """带步进的调参行: 写盘出 OLED_ITEM_PS、重解析回 step 字段（2026-10-18 加）。

        ⚠️ 按**标签定位**而不是 items[下标]、清步进只断言**这一行**回到 P 形状 ——
        排菜单的人随时在增删行/给别的行配步进（「标靶调试」里就有三条）,
        钉下标或全局 assertNotIn 都会和并行编辑打架。
        """
        m = self.model()
        it = next(i for n in m["order"] for i in m["lists"][n]["items"]
                  if i["act"] == "OLED_ACT_TUNE_PARAM")
        it["edited"] = True
        it["param"] = "CAM_MAIN_EXPOSURE"
        it["step"] = "0.01"
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertIn('OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "%s", 1u, "CAM_MAIN_EXPOSURE", "0.01"),'
                      % it["label"], _read(self.c))
        back = O.parse_region(_read(self.c))
        back_it = next(i for n in back["order"] for i in back["lists"][n]["items"]
                       if i["label"] == it["label"])
        self.assertEqual(back_it["step"], "0.01")
        self.assertEqual(back_it["param"], "CAM_MAIN_EXPOSURE")
        # 再把步进清空 → **这一行**回到普通 OLED_ITEM_P（文件里别的步进行不受影响）
        m2 = self.model()
        it2 = next(i for n in m2["order"] for i in m2["lists"][n]["items"]
                   if i["label"] == it["label"])
        it2["edited"] = True
        it2["step"] = None
        ok, _, status = self.save(m2)
        self.assertTrue(ok, status)
        self.assertIn('OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "%s", 1u, "CAM_MAIN_EXPOSURE"),'
                      % it["label"], _read(self.c))
        back2 = O.parse_region(_read(self.c))
        back2_it = next(i for n in back2["order"] for i in back2["lists"][n]["items"]
                        if i["label"] == it["label"])
        self.assertIsNone(back2_it["step"])

    def test_step_validation(self):
        """步进只在通用调参行上、必须是正数字符串。"""
        # 非调参行带步进 → 拦
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"][0].update(
            {"step": "1"}))
        self.assertIn("步进", err)
        # 调参行带非数字 → 拦
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"][1].update(
            {"step": "abc"}))
        self.assertIn("步进", err)
        # 负数/零形状 → 拦（"−0.5" 含负号, 不在 [0-9.] 白名单里）
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"][1].update(
            {"step": "-0.5"}))
        self.assertIn("步进", err)
        # 合法步进 → 放行
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"][1].update(
            {"step": "0.01"}))
        self.assertEqual(err, "")

    def test_map_round_trips_as_pm(self):
        """下拉行的显示映射: 写盘出 OLED_ITEM_PM（step 槽可空）、重解析回 map 字段;
        清掉映射 → 回到 OLED_ITEM_P。⚠️ 按标签定位（并行编辑友好, 同上条）。"""
        m = self.model()
        it = next(i for n in m["order"] for i in m["lists"][n]["items"]
                  if i["act"] == "OLED_ACT_TUNE_PARAM")
        it["edited"] = True
        it["step"] = None                          # 只配映射不配步进 → step 槽空串
        it["map"] = "1=手|3=自"
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertIn('OLED_ITEM_PM(OLED_ACT_TUNE_PARAM, "%s", 1u, "%s", "", "1=手|3=自"),'
                      % (it["label"], it["param"]), _read(self.c))
        back = O.parse_region(_read(self.c))
        back_it = next(i for n in back["order"] for i in back["lists"][n]["items"]
                       if i["label"] == it["label"])
        self.assertEqual(back_it["map"], "1=手|3=自")
        self.assertIsNone(back_it["step"])
        # 清映射 → P 形状
        m2 = self.model()
        it2 = next(i for n in m2["order"] for i in m2["lists"][n]["items"]
                   if i["label"] == it["label"])
        it2["edited"] = True
        it2["map"] = None
        ok, _, status = self.save(m2)
        self.assertTrue(ok, status)
        self.assertIn('OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "%s", 1u, "%s"),'
                      % (it["label"], it["param"]), _read(self.c))

    def test_map_validation(self):
        """映射只在调参行上; 段缺 '='、值重复、值不在真选项表里都要拦。

        param_opts = {参数名: 真选项表}（server 现扫传入的形状）。
        ⚠️ 调参行按**动态定位**（排菜单的人随时增删行, 下标/参数名都会漂）——
        opts 就以「当前选中那条行的参数名」为键现配, 值域校验与行无关。"""
        m0 = self.model()
        probe = next(i for n in m0["order"] for i in m0["lists"][n]["items"]
                     if i["act"] == "OLED_ACT_TUNE_PARAM")
        opts = {probe["param"]: "0=甲|1=乙"}

        def check(mutate):
            m = self.model()
            mutate(m)
            return O._check(m, None, opts)

        def touch(mm):
            row = next(i for n in mm["order"] for i in mm["lists"][n]["items"]
                       if i["act"] == "OLED_ACT_TUNE_PARAM")
            row["edited"] = True
            return row

        # 非调参行带映射 → 拦
        err = check(lambda m: m["lists"]["s_stListTask"]["items"][0].update(
            {"map": "0=手"}))
        self.assertIn("映射", err)
        # 值重复 → 拦
        err = check(lambda m: touch(m).update({"map": "0=手|0=自"}))
        self.assertIn("重复", err)
        # 段缺 '=' → 拦
        err = check(lambda m: touch(m).update({"map": "0=手|1"}))
        self.assertIn("'1'", err)
        # 值不在真选项表里（只有 0/1, 没有 2）→ 拦
        err = check(lambda m: touch(m).update({"map": "2=二"}))
        self.assertIn("不在参数", err)
        # 短标签太宽（4 个汉字 = 64px > 48px 预算）→ 拦
        err = check(lambda m: touch(m).update({"map": "0=甲乙丙丁"}))
        self.assertIn("太宽", err)
        # 合法映射 → 放行
        err = check(lambda m: touch(m).update({"map": "0=甲|1=乙"}))
        self.assertEqual(err, "")

    def test_missing_han_includes_map_labels(self):
        """映射短标签的字也是运行期字符串 —— missing_han 要把它们算进"用到"。"""
        m = self.model()
        it = next(i for n in m["order"] for i in m["lists"][n]["items"]
                  if i["act"] == "OLED_ACT_TUNE_PARAM")
        it["map"] = "1=鸂|3=鶒"                      # 两个生僻字, 字表里必然没有
        miss = O.missing_han(m)
        self.assertIn("鸂", miss)
        self.assertIn("鶒", miss)

    def test_rejects_the_none_sentinel_as_a_preset(self):
        """`SCRIPT_PRESET_NONE` 是「不跳转」的哨兵 —— 选到它固件只会长鸣（不静默,
        但对操作员就是个死项), 保存前先拦。"""
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"][0].update(
            {"act": "OLED_ACT_RUN_PRESET", "arg": "SCRIPT_PRESET_NONE"}))
        self.assertIn("还没选预设", err)

    def test_run_preset_and_post_task_need_a_preset_argument(self):
        """`OLED_ACT_SHOOT_TEST` 也是「参数 = 预设 id」那一类（2026-10-23 加）。

        ⚠️ 它曾经漏在这张表外面 → 控制台生成 `OLED_ITEM(...)`（没有 arg）→ 固件拿到
        `ucArg = 0` → `cpstFindScriptPreset(0)` 查不到, 车上按「开始射靶测试」只打
        `MENU: task preset id not found`（看着像按键没反应）。
        """
        for act in ("OLED_ACT_POST_TASK", "OLED_ACT_RUN_PRESET", "OLED_ACT_SHOOT_TEST"):
            err = self.bad(lambda m, a=act: m["lists"]["s_stListTask"]["items"][0].update(
                {"act": a, "arg": None}))
            self.assertIn("还没选预设", err, act)

    def test_other_preset_actions_are_untouched(self):
        """别把这条校验扩到不需要预设的动作上（否则它们全变非法）。"""
        err = self.bad(lambda m: m["lists"]["s_stListTask"][
            "items"].__setitem__(0, {"label": "中止", "act": "OLED_ACT_ABORT_SCRIPT",
                                     "enabled": True, "arg": None, "sub": None,
                                     "edited": True}))
        self.assertEqual(err, "")

    def test_allows_a_tune_row_on_a_page(self):
        """「调参」行**可以**放页面上（2026-10-09 起）—— 固件改成了光标停在调参行上时
        LEFT/RIGHT 就地调值（翻页让位给调值, 想翻页先按 UP/DOWN 挪开光标）。

        ⚠️ 这条原来断的是「必须拒绝」, 与固件同步放开。**改回拒绝之前先看固件**:
        app_oled.c::vOledPageMenu 的 bTune 判据（它决定了页面上那两个键到底调不调值）。
        """
        err = self.bad(lambda m: m["lists"]["s_stListLine"]["items"][0].update(
            {"act": "OLED_ACT_TUNE_QTY", "arg": "OLED_QTY_GYRO_TARGET"}))
        self.assertEqual(err, "")

    def test_rejects_a_submenu_inside_a_submenu(self):
        """固件只存一个 `s_pstSubList` —— 套两层会丢掉上一层, 退出时直接弹回页面。"""
        err = self.bad(lambda m: m["lists"]["s_stListPiTune"]["items"][0].update(
            {"act": "OLED_ACT_ENTER_SUB", "sub": "s_stListTune", "arg": None}))
        self.assertIn("只支持一层", err)

    def test_rejects_editing_the_two_code_pages(self):
        err = self.bad(lambda m: m["pages"].__setitem__(
            0, {"kind": "OLED_PAGEKIND_MENU", "list": "s_stListTask"}))
        self.assertIn("代码页", err)

    def test_rejects_dangling_submenu_reference(self):
        """指向一张不存在的子表 —— 写进 C 就是 `undefined identifier`, 宁可这里先拦。"""
        err = self.bad(lambda m: m["lists"]["s_stListTask"]["items"][2].update(
            {"sub": "s_stListNope"}))
        self.assertIn("不存在的子表", err)

    def test_submenu_is_auto_reordered_before_its_referrer(self):
        """C 要求符号先声明（`OLED_ITEM_SUB` 里是直接取地址）→ 保存时**自动**把子表排到
        引用它的表之前。排菜单的人不需要知道这件事。⚠️ 引用对动态找（表名随编辑变）。"""
        m = self.model()
        referrer, sub = next((n, i["sub"])
                             for n in m["order"]
                             for i in m["lists"][n]["items"]
                             if i["act"] == "OLED_ACT_ENTER_SUB" and i.get("sub"))
        # 故意把「子表」挪到引用它的表**之后**
        order = list(m["order"])
        order.remove(sub)
        order.insert(order.index(referrer) + 1, sub)
        m["order"] = order
        self.assertEqual(O._check(m), "", "顺序不该再由 _check 判")
        ok, _, status = self.save(m)
        self.assertTrue(ok, status)
        order = O.parse_region(_read(self.c))["order"]
        self.assertLess(order.index(sub), order.index(referrer))

    def test_legal_order_is_a_fixed_point_of_the_sort(self):
        """已经合法的顺序必须是**恒等变换** —— 否则无操作保存会整段重排、diff 爆掉。"""
        m = self.model()
        self.assertEqual(O._stable_topo_order(m["order"], m["lists"])[0], list(m["order"]))
        self.assertEqual(O._stable_topo_order(m["order"], m["lists"])[1], [])

    def test_mutual_submenu_reference_is_rejected(self):
        """两张表互相引用 = 成环, 拓扑排序排不出来 → 报错（否则写出去的 C 编不过）。
        ⚠️ 引用对动态找: 找现成的「引用者→子表」, 再让子表第一行反过来指向引用者。"""
        m = self.model()
        referrer, sub = next((n, i["sub"])
                             for n in m["order"]
                             for i in m["lists"][n]["items"]
                             if i["act"] == "OLED_ACT_ENTER_SUB" and i.get("sub"))
        first = m["lists"][sub]["items"][0]
        first["act"] = "OLED_ACT_ENTER_SUB"
        first["sub"] = referrer
        first["arg"] = None
        first["param"] = None
        ok, resp, status = self.save(m)
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertIn("成环", resp["message"])

    def test_rejects_referencing_a_list_that_is_not_in_order(self):
        m = self.model()
        m["pages"].append({"kind": "OLED_PAGEKIND_MENU", "list": "s_stListGhost",
                           "edited": True})
        self.assertIn("不存在的表", O._check(m))


# ==================== 字表 / 补字 ====================

class TestGlyphs(Sandbox):
    def _absent_char(self):
        listed = O.listed_han(_read(self.glyphs))
        for ch in "鑫囧犇淼焱垚掱":
            if ch not in listed:
                return ch
        self.skipTest("找不到不在字表里的汉字")

    def test_missing_han_finds_new_characters(self):
        ch = self._absent_char()
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "关机" + ch
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True
        self.assertEqual(O.missing_han(m), [ch])

    def test_save_appends_new_glyphs_and_runs_the_font_generator(self):
        ch = self._absent_char()
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "关机" + ch
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True

        ok, resp, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertEqual(resp["glyphs_added"], [ch])
        self.assertIn(ch, _read(self.glyphs))
        # 有新字 → 必须跑字模生成（否则屏上是方框问号）
        self.assertTrue(self.m_run_py.called)
        self.assertIn(str(O.GEN_FONT_PY), [str(c.args[0]) for c in self.m_run_py.call_args_list])
        # 并做了覆盖校验
        self.assertIn(str(O.CHECK_COVERAGE_PY),
                      [str(c.args[0]) for c in self.m_run_py.call_args_list])

    def test_edit_without_new_glyphs_skips_the_font_generator(self):
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "关机测试"   # 全是字表里已有的
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True
        ok, resp, status = self.save(m)
        self.assertTrue(ok, status)
        self.assertEqual(resp["glyphs_added"], [])
        gen_calls = [c for c in self.m_run_py.call_args_list
                     if str(c.args[0]) == str(O.GEN_FONT_PY)]
        self.assertEqual(gen_calls, [], "没加新字就不该重跑字模生成")

    def test_glyphs_are_appended_one_per_line(self):
        ch = self._absent_char()
        O._append_glyphs([ch])
        self.assertIn("\n" + ch + "\n", _read(self.glyphs))
        self.assertIn(ch, O.listed_han(_read(self.glyphs)))


# ==================== 事务性 / 冲突 ====================

class TestTransactional(Sandbox):
    def test_conflict_returns_409(self):
        ok, resp, status = self.save(self.model(), base={"c": 1.0, "h": 1.0, "glyphs": 1.0})
        self.assertFalse(ok)
        self.assertEqual(status, 409)
        self.assertEqual(resp["error"], "conflict")

    def test_invalid_returns_400_without_writing(self):
        before = _read(self.c)
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "一二三四五六七八九"
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True
        ok, resp, status = self.save(m)
        self.assertFalse(ok)
        self.assertEqual(status, 400)
        self.assertEqual(_read(self.c), before)

    def test_font_generator_failure_rolls_everything_back(self):
        ch = TestGlyphs._absent_char(self)
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "关机" + ch
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True
        before_c, before_g = _read(self.c), _read(self.glyphs)

        self.m_run_py.return_value = (1, "", "boom")
        with self.assertRaises(O.OledMenuError):
            self.save(m)

        self.assertEqual(_read(self.c), before_c, "菜单文件没回滚")
        self.assertEqual(_read(self.glyphs), before_g, "字表没回滚")

    def test_coverage_failure_is_a_warning_not_an_error(self):
        m = self.model()
        m["lists"]["s_stListTask"]["items"][0]["label"] = "关机测试"
        m["lists"]["s_stListTask"]["items"][0]["edited"] = True
        self.m_run_py.return_value = (1, "", "缺字")
        ok, resp, status = self.save(m)
        self.assertTrue(ok)
        self.assertEqual(status, 200)
        self.assertTrue(resp["warnings"])


class TestNaming(Sandbox):
    def test_item_macros_carry_the_right_fields(self):
        """三种表行宏 → 三个形状; 解析出来必须与写出去的一致（防 label/act 取反的老 bug）。"""
        line_plain = O._fmt_item({"label": "甲", "act": "OLED_ACT_NOP",
                                  "enabled": True, "arg": None, "sub": None})
        line_arg = O._fmt_item({"label": "乙", "act": "OLED_ACT_POST_TASK",
                                "enabled": False, "arg": "SCRIPT_PRESET_3", "sub": None})
        line_sub = O._fmt_item({"label": "丙", "act": "OLED_ACT_ENTER_SUB",
                                "enabled": True, "arg": None, "sub": "s_stListX"})
        self.assertEqual(line_plain, '    OLED_ITEM(OLED_ACT_NOP, "甲", 1u),')
        self.assertEqual(line_arg, '    OLED_ITEM_A(OLED_ACT_POST_TASK, "乙", 0u, SCRIPT_PRESET_3),')
        self.assertEqual(line_sub, '    OLED_ITEM_SUB("丙", 1u, &s_stListX),')

        body = [line_plain, line_arg, line_sub]
        items = O._parse_items(body)
        self.assertEqual([i["label"] for i in items], ["甲", "乙", "丙"])
        self.assertEqual([i["act"] for i in items],
                         ["OLED_ACT_NOP", "OLED_ACT_POST_TASK", "OLED_ACT_ENTER_SUB"])
        self.assertEqual([i["enabled"] for i in items], [True, False, True])
        self.assertEqual(items[1]["arg"], "SCRIPT_PRESET_3")
        self.assertEqual(items[2]["sub"], "s_stListX")


class TestPreviewMatchesFirmware(unittest.TestCase):
    """预览的排版常量必须与固件**同源同义**。

    ⚠️ 不一致时预览会**静默地**和实车不一样 —— 而预览存在的全部意义就是
    「看到什么就是烧进去什么」。这类漂移不会报错, 只会让人看着预览排出一个
    在车上被截断的标签。
    """

    @classmethod
    def setUpClass(cls):
        cls.c = (O.ROOT / "Application" / "app_oled.c").read_text(encoding="utf-8")
        cls.js = (O.APP_DIR / "static" / "app.js").read_text(encoding="utf-8")

    def _def(self, name):
        m = re.search(r"#define\s+%s\s+(\d+)u" % name, self.c)
        self.assertIsNotNone(m, "app_oled.c 里找不到 %s" % name)
        return int(m.group(1))

    def test_screen_geometry_agrees(self):
        self.assertEqual(self._def("OLED_W"), 128)
        self.assertEqual(self._def("OLED_ROW_H"), 16)

    def test_rows_start_at_the_expected_y(self):
        ys = re.findall(r"#define\s+OLED_Y(\d)\s+(\d+)u", self.c)
        self.assertEqual([(a, int(b)) for a, b in ys],
                         [("1", 0), ("2", 16), ("3", 32), ("4", 48)])

    def test_frontend_uses_the_same_constants(self):
        m = re.search(r"const OLED_W = (\d+), OLED_ROW_H = (\d+);", self.js)
        self.assertIsNotNone(m, "app.js 里找不到 OLED_W / OLED_ROW_H")
        self.assertEqual((int(m.group(1)), int(m.group(2))),
                         (self._def("OLED_W"), self._def("OLED_ROW_H")))

    def test_scrollbar_constants_agree(self):
        """滚动窗口的三个常量必须与固件**同源同义**（一页 16 项 / 一屏 4 行 / 滚动条 8px）,
        否则预览画出的窗口与车上不一致, 而这类漂移不报错。"""
        m = re.search(r"const OLED_MAX_ITEMS = (\d+), OLED_WIN = (\d+), OLED_BAR_W = (\d+);",
                      self.js)
        self.assertIsNotNone(m, "app.js 里找不到 OLED_MAX_ITEMS / OLED_WIN / OLED_BAR_W")
        self.assertEqual((int(m.group(1)), int(m.group(2)), int(m.group(3))),
                         (self._def("OLED_MAX_ITEMS"), self._def("OLED_MAX_ROWS"),
                          self._def("OLED_SCROLLBAR_W")))
        # 服务端那几个常量也要与固件同一套（否则「控制台放行、车上画不下」）
        self.assertEqual((O.MAX_ROWS, O.MAX_ITEMS, O.SUB_MAX_ITEMS, O.SCROLLBAR_PX),
                         (self._def("OLED_MAX_ROWS"), self._def("OLED_MAX_ITEMS"),
                          self._def("OLED_MAX_ITEMS"), self._def("OLED_SCROLLBAR_W")))
        # 文字预算: 固件与前端都必须是「一行宽 - 滚动条宽」
        self.assertIsNotNone(re.search(r"#define\s+OLED_LABEL_W\s+\(OLED_W - OLED_SCROLLBAR_W\)",
                                       self.c), "app_oled.c 的 OLED_LABEL_W 不再是 OLED_W - OLED_SCROLLBAR_W")
        self.assertIsNotNone(re.search(r"const OLED_LABEL_W = OLED_W - OLED_BAR_W;", self.js),
                             "app.js 的 OLED_LABEL_W 不再是 OLED_W - OLED_BAR_W")
        self.assertEqual(self._def("OLED_W") - self._def("OLED_SCROLLBAR_W"), 120)

    def test_glass_is_exactly_four_rows_and_inside_the_module(self):
        """玻璃区必须正好 4 行高（64px）—— 大了会画出屏外, 小了会少一行。"""
        m = re.search(r"OLED_MOD = \{ w: (\d+), h: (\d+), gx: (\d+), gy: (\d+), "
                      r"gw: (\d+), gh: (\d+) \}", self.js)
        self.assertIsNotNone(m, "app.js 里找不到 OLED_MOD")
        w, h, gx, gy, gw, gh = (int(x) for x in m.groups())
        self.assertEqual(gw, self._def("OLED_W"))
        self.assertEqual(gh, 4 * self._def("OLED_ROW_H"))
        self.assertLessEqual(gx + gw, w, "玻璃越出画布右边")
        self.assertLessEqual(gy + gh, h, "玻璃越出画布下边")
        # 画布逻辑尺寸必须能被 CSS 整数放大（image-rendering: pixelated 的前提）
        css = (O.APP_DIR / "static" / "style.css").read_text(encoding="utf-8")
        cm = re.search(r"#oled-screen \{[\s\S]*?width: (\d+)px; height: (\d+)px;", css)
        self.assertIsNotNone(cm, "style.css 里找不到 #oled-screen 的尺寸")
        self.assertEqual(int(cm.group(1)) % w, 0, "CSS 宽度不是逻辑宽度的整数倍")
        self.assertEqual(int(cm.group(2)) % h, 0, "CSS 高度不是逻辑高度的整数倍")


if __name__ == "__main__":
    unittest.main(verbosity=2)
