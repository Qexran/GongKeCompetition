/**
  * @file       app_oled_menu.c
  * @brief      OLED 菜单的数据表本体 —— 由 Web 控制台生成/维护, 数据模型见 app_oled_menu.h。
  *
  * ⚠️⚠️ **「数据区」两个标记之间的整段由控制台整段重写（含块标题注释）**。
  *     手改标记之间的内容, 会在控制台下次保存时被规范化覆盖 —— 想留长期说明就写在
  *     标记**之外**（本注释块这么长就是为此）。
  *
  * ⚠️ **表的书写顺序有约束: 子表必须定义在引用它的页/父表之前**（宏里是直接取地址,
  *     编译器要求符号已声明）。当前顺序: 两张子表 → 引用它们的两张页表 → 页面表。
  *
  * ⚠️ **一页 / 一个子菜单最多 32 个启用项** —— 一屏只画 4 行, 放不下的靠 UP/DOWN **滚动**
  *     查看（右侧画竖直滚动条）。「返回」由固件补在**最后一条**（随列表滚动, 不占额度）,
  *     它不进表, 所以生成表漏了它也退得出来。子菜单的表里**没有**「返回」项。
  *
  * ⚠️ 表里只存**静态兜底标签**：「开关激光:开/关」「停止巡线测试」「拍照 成功!」这些
  *     运行期标签由 app_oled.c 按动作码拼; 调参行的 `:值` 同理（所以这里写 "陀螺目标"
  *     而不是 "陀螺目标:%.1f" —— **格式化串不进表**）。
  *
 * ------------------------------------------------------------------
 * 第 5 页「射靶测试」（2026-10-09 加, `s_stListShoot`）—— 说明只能写在标记**外面**,
 * 因为标记之间的整段由控制台重写（写进去的注释会被规范化掉, 那是它的既定行为）。
 *
 * 布局与宽度（**文字预算 120px** = 一行 128px 减掉右侧滚动条那 8px; 汉字 16px / ASCII 8px）。
 * 本页 4 项正好占满一屏（一屏就是 4 行）:
 *   1 `开始射靶测试` / `停止射靶 …`  ← 运行期标签, 由 app_oled.c 按动作码拼。共**七态**:
 *        开始射靶测试（没跑） / 停止射靶 粗调 / 停止射靶 运动中 / 停止射靶 OK! /
 *        停止射靶 冻结 / 停止射靶 无目标 / 停止射靶 无信号
 *        （后面几个回答的是同一个问题 —— **车为什么不动**）
 *   2 `激光:开 d=-3px`               ← 激光那半截是运行期标签; ` d=-3px` 由
 *        app_oled.c::vOledMenuDraw **按语义挂**（"这张表里有射靶测试项"才拼, 照抄
 *        「让出成功!」横幅那条挂法）—— 所以第 3 页那张表的激光行不带它。112px。
 *   3 `瞄准列:980`                   ← ±5px/格（5 > tc_conv_px 默认 4.5 → 一按必出死区）
 *   4 `收敛阈值:4.5`                 ← ±0.5px, 钳 [2.0, 30.0]
 *
 * ⚠️ 第 1 项的参数 = **预设 id**（`SCRIPT_PRESET_34`）: 既是 `SCRIPT_REQ_TASK` 的参数,
 *    也是 app_oled.c 判「这条测试在不在跑」的真值来源（`ucScriptPresetActive(arg)`）——
 *    别再把它当"选了哪个任务"另存一份。
 * ⚠️ **目标色不在这页选**: 走**第 2 页二维码**的第二位（同所有打靶步骤）。所以点「开始」时
 *    没设二维码会弹「还未设二维码!」确认框（与巡线测试同一条前置门）。
 * ⚠️ **这页的两个调参行是"页面上的调参行"**（不是子菜单里那种）: 固件 2026-10-09 起允许
 *    光标停在调参行上时 LEFT/RIGHT **就地调值**, 代价是那一行上的翻页让位给调值
 *    （想翻页先按 UP/DOWN 把光标挪开）。控制台的保存校验（`oled_menu.py::_check`）已同步
 *    放开 —— **两边必须一起改**: 只放开校验 = 页面上出死项; 只改固件 = 这一页存不进去。
 *
 * ------------------------------------------------------------------
 * 「Pi 参数子菜单」（2026-10 加, `s_stListPiTune` + 调参子菜单里那两条 Beam 行）
 *
 * 这两张表里的 `OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "标签", 1u, "参数名")` 是**通用调参行**:
 * 值由参数名现取, 固件那条分支（app_oled.c::vOledTuneParamStep/Text）不认识任何**具体**
 * 参数 —— 加一行、换个名字都不用改固件。名字 → 值源: ① Beam 表（本机, 现取现写）;
 * ② Pi 参数表（树莓派, 后台轮询缓存）。
 *
 * ⚠️ Pi 那批的**值在树莓派上, 改的是"临时值"**: 只改 Pi 主程序进程内存里的 config,
 *    不写 config_override.json, **Pi 重启就还原**（要固化去 GKS_Web 的「参数」页）。
 *    Pi 不认这个名字（config.py::EDITABLE_CONFIG 里没有）时回 ERR, 屏上显示 "--" ——
 *    改 Pi 侧的可编辑参数表之后, 记得重跑 `tools/gen_rpi_cfg.py` 重生成单片机侧的表。
 * ⚠️ 名字存的是**字符串**而不是下标/枚举: 两张表都会重排（Beam 表加参数、Pi 表由脚本
 *    重生成）, 名字不会。
 * ⚠️ 宽度：标签 ≤64px（4 汉字）, 值 ≤48px（6 ASCII）—— 屏上拼成 `标签:值` 一共 ≤120px。
 *    新标签用的汉字记得进 tools/fontgen/glyphs.txt（否则屏上显示成方框问号）。
 */

#include "app_oled_menu.h"
#include "map.h"              /* SCRIPT_PRESET_14/15/16 + TIMER_CMD_*（经 app_script.h） */

/* ==================== 数据区（控制台整段重写） ==================== */
/* ==== 数据区: 开始 ==== */

/* ---- 人质调试 ---- */
static const stOledItemTdf s_astOledMenuTune[] = {
    OLED_ITEM_A(OLED_ACT_POST_TASK, "执行任务", 1u, SCRIPT_PRESET_16),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想列", 1u, "hs_cx_center", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想行", 1u, "hs_cy_center", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "dx死区", 1u, "hs_dx_db_mm", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "dy死区", 1u, "hs_dy_db_mm", "2"),
    OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "白平衡", 1u, "HOSTAGE_AUTO_WB"),
    OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "置信阈值", 1u, "HOSTAGE_CONF_THR"),
    OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "XY对调", 1u, "hs_xy_swap"),
};
static const stOledListTdf s_stListTune = OLED_LIST(s_astOledMenuTune);

/* ---- 巡线调试 ---- */
static const stOledItemTdf s_astOledMenuPiTune[] = {
    OLED_ITEM(OLED_ACT_LINE_TEST, "开始巡线测试", 1u),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "补光亮度", 1u, "OPENMV_LED_BRIGHTNESS", "5"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "横向目标", 1u, "LINE_SETPOINT", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "角度目标", 1u, "line_angle_target", "0.1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "角度死区", 1u, "line_cam_db_deg", "0.1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "陀螺死区", 1u, "line_gyro_db_deg", "0.1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "转发频率", 1u, "LINE_SEND_HZ", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "KP", 1u, "LINE_KP", "0.01"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "KI", 1u, "LINE_KI", "0.01"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "KD", 1u, "LINE_KD", "0.01"),
};
static const stOledListTdf s_stListPiTune = OLED_LIST(s_astOledMenuPiTune);

/* ---- PI管理菜单 ---- */
static const stOledItemTdf s_astOledMenuTask[] = {
    OLED_ITEM(OLED_ACT_CONFIRM_SHUTDOWN, "树莓派关机", 1u),
    OLED_ITEM(OLED_ACT_POST_HANDSHAKE, "握手让出", 1u),
    OLED_ITEM(OLED_ACT_PI_MAIN_RESTART, "重启main", 1u),
    OLED_ITEM(OLED_ACT_PI_SVC_RESTART, "重启服务", 1u),
    OLED_ITEM(OLED_ACT_PI_LOG_CLEAR, "清屏Log", 1u),
    OLED_ITEM(OLED_ACT_GYRO_SRC, "陀螺源", 1u),
    OLED_ITEM(OLED_ACT_GYRO_CAL, "陀螺校准", 1u),
    OLED_ITEM(OLED_ACT_FILL_TOGGLE, "补光灯", 1u),
};
static const stOledListTdf s_stListTask = OLED_LIST(s_astOledMenuTask);

/* ---- 标靶调试 ---- */
static const stOledItemTdf s_astOledMenuSub1[] = {
    OLED_ITEM_A(OLED_ACT_SHOOT_TEST, "开始射靶测试", 1u, SCRIPT_PRESET_34),
    OLED_ITEM(OLED_ACT_LASER_TOGGLE, "开关激光", 1u),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "粗瞄准列", 1u, "tc_cx_aim", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "细瞄准列", 1u, "tc_cx_aim_pi", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "精调比例", 1u, "tc_px_to_mm", "0.01"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "收敛阈值", 1u, "tc_conv_px", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "平移速度", 1u, "tc_speed", "5"),
};
static const stOledListTdf s_stListSub1 = OLED_LIST(s_astOledMenuSub1);

/* ---- 色球调试 ---- */
static const stOledItemTdf s_astOledMenuSub4[] = {
    OLED_ITEM_A(OLED_ACT_POST_TASK, "执行任务", 1u, SCRIPT_PRESET_14),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想列", 1u, "ball_cx_center", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想行", 1u, "ball_cy_center", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "dx死区", 1u, "ball_dx_db_mm", "2"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "dy死区", 1u, "ball_dy_db_mm", "2"),
    OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "XY对调", 1u, "ball_xy_swap"),
};
static const stOledListTdf s_stListSub4 = OLED_LIST(s_astOledMenuSub4);

/* ---- 看路调试 ---- */
static const stOledItemTdf s_astOledMenuSub2[] = {
    OLED_ITEM_A(OLED_ACT_POST_TASK, "执行仅看路", 1u, SCRIPT_PRESET_23),
    OLED_ITEM_A(OLED_ACT_POST_TASK, "执行看路+转弯", 1u, SCRIPT_PRESET_36),
    OLED_ITEM(OLED_ACT_FILL_TOGGLE, "补光灯", 1u),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "亮度", 1u, "fill_task_road_pct", "5"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想Δx", 1u, "road_dx_ideal", "5"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "X比例", 1u, "road_scale", "0.01"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "纵移死区", 1u, "road_db_mm", "1"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "理想Δy", 1u, "road_dy_ideal", "5"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "Y比例", 1u, "road_scale_y", "0.01"),
    OLED_ITEM_PS(OLED_ACT_TUNE_PARAM, "横移死区", 1u, "road_db_mm_y", "1"),
    OLED_ITEM_A(OLED_ACT_POST_TASK, "执行反向看路", 1u, SCRIPT_PRESET_47),
};
static const stOledListTdf s_stListSub2 = OLED_LIST(s_astOledMenuSub2);

/* ---- 快速调试 ---- */
static const stOledItemTdf s_astOledMenuSub3[] = {
    OLED_ITEM(OLED_ACT_GYRO_CAL, "陀螺校准", 1u),
    OLED_ITEM(OLED_ACT_LASER_TOGGLE, "开关激光", 1u),
};
static const stOledListTdf s_stListSub3 = OLED_LIST(s_astOledMenuSub3);

/* ---- 调试页 ---- */
static const stOledItemTdf s_astOledMenuLine[] = {
    OLED_ITEM_SUB("快速调试", 1u, &s_stListSub3),
    OLED_ITEM_SUB("巡线调试", 1u, &s_stListPiTune),
    OLED_ITEM_SUB("标靶调试", 1u, &s_stListSub1),
    OLED_ITEM_SUB("人质调试", 1u, &s_stListTune),
    OLED_ITEM_SUB("色球调试", 1u, &s_stListSub4),
    OLED_ITEM_SUB("看路调试", 1u, &s_stListSub2),
};
static const stOledListTdf s_stListLine = OLED_LIST(s_astOledMenuLine);

/* ---- 页面表 ---- */
const stOledPageTdf g_astOledPage[] = {
    { (uint8_t)OLED_PAGEKIND_STATUS, 0 },
    { (uint8_t)OLED_PAGEKIND_QR, 0 },
    { (uint8_t)OLED_PAGEKIND_MENU, &s_stListLine },
    { (uint8_t)OLED_PAGEKIND_MENU, &s_stListTask },
};
const uint8_t g_ucOledPageNum = (uint8_t)(sizeof(g_astOledPage) / sizeof(g_astOledPage[0]));

/* ==== 数据区: 结束 ==== */
