/* GKS Web 控制台 — Tab 框架 + 「地图」Tab（map.c 积木编辑器）
 *
 * 数据流: GET /api/catalog + /api/map → 内存 items（扁平列表, 与 map.c 数组体
 * 一一对应）→ 渲染分区视图; 编辑标 edited, 保存 POST /api/map 由服务端序列化。
 */
'use strict';

/* ==================== 小工具 ==================== */

function h(tag, attrs, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === 'class') el.className = v;
    /* checked/disabled/value 必须走**属性赋值**而不是 setAttribute:
     * HTML 的 checked 是「默认勾选」—— setAttribute('checked', false) 打上的
     * 仍然是"存在"标记（显示为勾选!）, 且永不更新实时状态 → 重渲染后浏览器的
     * 切换方向与真实状态恒反, 勾选框直接锁死（底盘动作组的「启」踩过）。 */
    else if (k === 'checked') el.checked = !!v;
    else if (k === 'disabled') el.disabled = !!v;
    else if (k === 'value') el.value = (v === undefined || v === null) ? '' : v;
    /* value 特例同样要防 undefined: `el.value = undefined` 会变成字符串 "undefined"
     * 显示在框里（标定 Tab 的旧预设缺新参数时踩过）—— 同 setAttribute 分支的守卫 */
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else if (v !== null && v !== undefined) el.setAttribute(k, v);
  }
  for (const c of children.flat()) {
    if (c === null || c === undefined) continue;
    /* 数字/布尔也要转成文本节点: 直接 appendChild(9600) 会抛
     * "parameter 1 is not of type 'Node'"（协议里的波特率等本来就是数字）。 */
    const isNode = c instanceof Node;
    el.appendChild(isNode ? c : document.createTextNode(String(c)));
  }
  return el;
}

/* 提示条带队列: 单个 DOM 元素, 连续调用若不排队, 后一条会瞬间顶掉前一条
 * —— 曾经因此让「保存有警告」的具体内容一秒都看不到, 只留下摘要。 */
let toastTimer = null;
let toastQueue = [];
let toastBusy = false;

function toast(msg, kind) {
  toastQueue.push({ msg, kind });
  if (!toastBusy) nextToast();
}

function nextToast() {
  const el = document.getElementById('toast');
  clearTimeout(toastTimer);
  if (!toastQueue.length) { toastBusy = false; el.className = 'hidden'; return; }
  toastBusy = true;
  const { msg, kind } = toastQueue.shift();
  el.textContent = msg;
  el.className = 'show' + (kind ? ' ' + kind : '');
  toastTimer = setTimeout(nextToast, kind === 'err' ? 5000 : 2200);
}

const ZONE_COLORS = 8;
function zoneColorIdx(title) {
  let s = 0;
  for (const ch of title || '') s = (s * 131 + ch.codePointAt(0)) >>> 0;
  return s % ZONE_COLORS;
}

/* ==================== Tab 框架 ==================== */

const TABS = [
  { id: 'map',     label: '地图' },
  { id: 'tuning',  label: '调参' },
  { id: 'moves',   label: '位移' },
  { id: 'switches', label: '自定义' },
  { id: 'calib',   label: '标定' },
  { id: 'capture', label: '采集' },
  { id: 'tools',   label: '调试助手' },
  { id: 'bt',      label: '蓝牙' },
  { id: 'arm',     label: '机械臂' },
  { id: 'chact',   label: '底盘动作组' },
  { id: 'oled',    label: 'OLED' },
];

let curTab = 'map';

function renderTabs() {
  const nav = document.getElementById('tabs');
  nav.textContent = '';
  for (const t of TABS) {
    nav.appendChild(h('button', {
      class: 'tab' + (t.id === curTab ? ' active' : ''),
      'data-tab': t.id,
      onclick: () => switchTab(t.id),
    }, t.label));
  }
}

/* 切换 Tab: 切 pane/tab 高亮, 并按 Tab 显示顶栏按钮。
 * 顶栏的「重载 / 保存 map.c / 未保存圆点」只对地图 Tab 有意义, 切走必须藏掉,
 * 否则会出现「在调试助手页点保存 map.c」。 */
function switchTab(id) {
  const t = TABS.find(x => x.id === id);
  if (!t) return;

  curTab = id;
  for (const x of TABS) {
    const pane = document.getElementById('pane-' + x.id);
    if (pane) pane.classList.toggle('active', x.id === id);
  }
  document.querySelectorAll('#tabs .tab').forEach(el =>
    el.classList.toggle('active', el.dataset.tab === id));
  /* 顶栏操作组按 Tab 切换: 「重载/保存 map.c」只对地图有意义, 「保存 app_config.h」只对调参 */
  document.getElementById('map-actions').classList.toggle('hidden', id !== 'map');
  document.getElementById('tuning-actions').classList.toggle('hidden', id !== 'tuning');
  document.getElementById('oled-actions').classList.toggle('hidden', id !== 'oled');

  if (id === 'tools' && !TOOLS.loaded) loadTools();          /* 首次进入才拉取 */
  if (id === 'tuning' && !TUNE.loaded) loadTuning();
  if (id === 'moves') movesEnter();                          /* 位移: 绑骨架 + 懒加载 */
  if (id === 'switches') swEnter();                          /* 自定义开关: 懒加载 */
  if (id === 'calib') calibEnter();                              /* 标定: 绑骨架 + 懒加载 */
  if (id === 'bt' && !BT.loaded) loadBt();
  if (id === 'arm' && !ARM.loaded) ARM.loading = loadArm();   /* 共享 promise, 见 loadArm */
  else if (id === 'arm') { armRenderAll(); renderJoints(); }   /* 回来时把隐藏期间变过的重绘 */
  if (id === 'chact') chactEnter();                            /* 底盘动作组: 绑骨架 + 懒加载 */
  if (id === 'oled' && !OLED.loaded) loadOled();
  else if (id === 'oled') oledRenderAll();       /* 回来时把隐藏期间改过的重绘 */
  if (id === 'capture' && !CAP.loaded) loadCapture();
  else if (id === 'capture') capStartTimers();               /* 从别的 Tab 回来要重新起泵 */
  /* 离开「链路相关」的三个 Tab（蓝牙/机械臂/底盘动作组）要停掉轮询定时器 —— chact 的
   * ch_busy 轮询离开必须停（对着可能已断开的串口发请求毫无意义, 且 linkTabVisible
   * 只认蓝牙/机械臂两个 Tab, 这里单独停）。 */
  if (id !== 'chact') chactStopBusyPoll('播放器: 状态未知（已离开本页）');
  /* 离开「链路相关」的两个 Tab（蓝牙/机械臂）要停掉两个定时器：本项目别处没有任何
   * 轮询, 不收拾的话切走以后还会一直对着可能已断开的串口发请求; 而且两个 Tab 同时
   * 各跑一个 5Hz 的 $B,POS 会互相抢命令锁, 表现为回读卡顿。
   * 从别的 Tab 回到这两个之一时, 若还在编辑模式就把回读泵重新起上。 */
  if (!linkTabVisible()) btStopTimers();
  else if (BT.edit) { btStartPoll(); renderJoints(); }
  /* 采集同理: 帧泵与状态轮询离开必须停, 否则切走后还在对着摄像头发请求。 */
  if (id !== 'capture') capStopTimers();
  /* 多客户端同步: 非当前 Tab 的远端改动一直没处理（见「多客户端同步」一节）,
   * 用缓存的那份 /api/rev 响应在这里重判一次 —— 不发新请求。 */
  revCheckNow();
}

/* ==================== 状态 ==================== */

const S = {
  catalog: [],
  catByType: {},
  presets: [],            // [{ident, label, id, items}] —— map.c 里每张 g_astScriptPresetN[]
                          //   id = 预设注册表里的编号(跳转目标的落地点); null = 没进注册表
  nextId: 1,              // 新建预设该用的下一个编号 —— **只增不重用**, 由服务端给
  activeIdx: 0,           // 正在编辑的预设下标
  runIdent: '',           // 设备运行的预设 ident（写回 map.c 的 SCRIPT_PRESET_ACTIVE）
  items: [],              // = presets[activeIdx].items 的活引用（拖拽原地增删改, 勿重新赋值）
  version: 0,
  dirty: false,
  selectedId: null,
  seq: 1,
  zonesMeta: [],          // [{el, ids, nextFirstId}] 拖拽定位用
  ifStruct: null,         // 每次 renderCanvas 由 scanIfStructure 算出来的分支结构（见那一段）
  ifExpanded: new Set(),  // 被**显式展开过**的「如果」块的键 —— 不在集合里 = 折叠（默认）
};

function curPreset() { return S.presets[S.activeIdx]; }
function bindItems() { S.items = S.presets[S.activeIdx].items; }

/* ==================== 「如果」块的折叠状态 ====================
 * 「如果」块**默认折叠**, 这里只记「被显式展开过」的那些键（所以空集合 = 全部折叠）。
 *
 * 键 = 预设 ident + 嵌套路径（`g_astScriptPreset1#0` / `g_astScriptPreset1#0.1` ——
 * 路径是「第几个『如果』块」的链, 见 scanIfStructure 的 keyOf）。**不用条目 id**:
 * 那个是 `_parse_body` 按行号生成的（i0/i1…）, 插/删任何一条积木都会整体挪位,
 * 于是「在 A 块下面加一块」会让折叠状态跳到 B 块上去。
 *
 * ⚠️ 存 **localStorage（per 浏览器）**, 不进仓库、也不走服务端:
 *    · 折叠是**视图偏好**, 和滚动位置同类 —— 队友的页面不该因为我把一块折起来就跟着变;
 *    · 本项目有 `/api/rev` 多客户端同步, 把它塞进服务端文件会被当成「远端改动」触发重载
 *      （见 CLAUDE.md 的「多客户端同时编辑」一节）;
 *    · 与 `bt.json`/`cap.json` 同一条纪律: 各机不同的东西不共享。
 * ⚠️ localStorage 在隐私模式/被禁时**会抛异常**, 所以两个助手都吞掉异常 —— 存不下不是故障,
 *    只是刷新后回到「默认折叠」。node 桩里也没有它（typeof 判断兜住, 否则一加载就炸）。 */
const IF_EXPANDED_KEY = 'gks.map.ifExpanded';

function lsGetJson(k) {
  try {
    if (typeof localStorage === 'undefined' || !localStorage) return null;
    return JSON.parse(localStorage.getItem(k) || 'null');
  } catch (e) { return null; }
}

function lsSetJson(k, v) {
  try {
    if (typeof localStorage === 'undefined' || !localStorage) return;
    localStorage.setItem(k, JSON.stringify(v));
  } catch (e) { /* 配额满 / 隐私模式: 忽略, 下次刷新回到默认折叠 */ }
}

(function loadIfExpanded() {
  const arr = lsGetJson(IF_EXPANDED_KEY);
  if (Array.isArray(arr)) S.ifExpanded = new Set(arr.filter(x => typeof x === 'string'));
})();

function markDirty() { S.dirty = true; document.getElementById('dirty-dot').classList.remove('hidden'); }
function clearDirty() { S.dirty = false; document.getElementById('dirty-dot').classList.add('hidden'); }

function catalogOf(step) { return S.catByType[step] || { params_f: [], params_i: [], hint: '' }; }
function paramLabels(cat) { return cat.params_f.concat(cat.params_i); }
function paramMeta(cat) {
  if (cat.params_meta && cat.params_meta.length) return cat.params_meta;
  return paramLabels(cat).map(n =>
    ({ name: n, label: n.split('(')[0], unit: cat.unit || '', tip: '' }));
}

/* 参数控件: 有 options(枚举/布尔) → 下拉; 否则文本输入。
 * options 全部由服务端从源码派生(枚举定义/DSL), 前端不硬编码任何取值。 */
/* 参数值的比较键: 忽略整数后缀 `u`（以及浮点槽的 `.0f`）。
 *
 * 为什么需要单向归一化: 写盘时 map_codec._normalize_param 会给非负整数补 `u`
 * （`1` → `1u`、`10` → `10u`），而下拉选项的 value 是 DSL 里写的**裸值**（`1`/`0`）。
 * 直接字符串比对就会出现「值明明就在枚举里, 下拉却显示 `1u  (枚举外)`」——
 * 转向/开合/使能 这类数字选项的参数全中（现场真踩过, 用户报「参数被翻译成 1u(枚举外)」）。
 * 只在这一处归一化: 选项值本身不动, 写回 map.c 仍按原样（未改动的行零 diff 不受影响）。 */
function optKey(v) {
  return String(v === undefined || v === null ? '' : v).trim()
    .replace(/[uU]$/, '')          // 整数槽后缀
    .replace(/\.0f$/i, '');        // 浮点槽的整数写法（1.0f → 1）
}

/* 参数显隐: 条件写在 app_script.h 的 DSL 里（`<显隐:参数名=值>`）, 服务端解析成
 * params_meta[i].show_if —— 前端**不硬编码**任何参数名/取值（改注释行即可生效）。
 * ⚠️ 比对必须走 optKey: 写盘时整数槽会被补 u（1 → 1u）, 而标记里写的是裸值。
 * ⚠️ 条件参数解析不出来（被改名/删掉）时**显式放行** —— 让一个用户看不出原因的
 *    参数凭空消失, 比多显示一个参数难查得多。
 * cat 显式传入（不在内部 catalogOf）→ 纯函数, node 探针可以直接喂字面量对象。 */
function paramVisible(it, m, cat) {
  const sw = m && m.show_if;
  if (!sw || !sw.name) return true;
  const metas = paramMeta(cat || catalogOf(it.step));
  const idx = metas.findIndex(x => x.name === sw.name);
  if (idx < 0) return true;
  return optKey((it.params || [])[idx]) === optKey(sw.value);
}

/* 参数启用（`<启用:参数名=值>`, 2026-10 加, 与显隐同一套机器）:
 * 条件满足 = 可用; 不满足 = 禁用（**看得见点不动**, 与显隐"整行消失"的差别就在这）。
 * ⚠️ 判不出条件参数时**放行**（可用）—— 理由同 paramVisible: 静默锁死一个输入框,
 *    用户只会以为"这积木坏了"。 */
function paramEnabled(it, m, cat) {
  const en = m && m.enable_if;
  if (!en || !en.name) return true;
  const metas = paramMeta(cat || catalogOf(it.step));
  const idx = metas.findIndex(x => x.name === en.name);
  if (idx < 0) return true;
  return optKey((it.params || [])[idx]) === optKey(en.value);
}

/* 两个选项恰为 0/1 **且标签是 是/否** 的参数 → 渲染成勾选框（「勾选」比下拉直觉得多:
 * STEP_BUGUANG_KAI 的「自定义亮度」、STEP_BEEP 的「周期模式」、启动即返回/叠加残留…）。
 * ⚠️ 标签不是 是/否 的两档参数（转向{左转,右转}、角度模式{相对,绝对}、开合{张开,闭合}、
 * 串口{主口,副口}…）**保持下拉** —— 那种勾上了得靠猜意思（2026-10-18 用户反馈）,
 * 勾选框只留给"是/否"语义的开关。 */
function isBoolOptions(m) {
  if (!m.options || m.options.length !== 2) return false;
  if (m.options.map(o => optKey(o.value)).sort().join(',') !== '0,1') return false;
  const labels = m.options.map(o => (o.label || '').trim());
  return labels.includes('是') && labels.includes('否');
}

/* ==================== 「每次使用档位」按钮 + 弹窗（2026-10-23 加）====================
 * 给 STEP_RPI_WARM_CAM 的 prof 槽用（DSL 标记 `<控件:warm_profiles:uses>`）。
 * 为什么不是一个下拉: 一次预热要被 uses 个任务消费, 而它们可能要不同的档 ——
 * 下拉只能表达一个值。这里用**按钮 + 弹窗**, 弹窗里按 uses 逐行选。
 *
 * 数据编码（与固件 app_rpi.h::WARM_PROF_*、Pi handler._unpack_profiles **必须一致**）:
 *   每次使用占 **2 bit**, 第 i 次在 bit 2i 上；值 0=默认 / 1=看路 / 2=人质。
 *   例: uses=3, [默认, 看路, 人质] = 0 | (1<<2) | (2<<4) = 36。
 *   单档老值天然兼容（1 = 第 1 次看路）。
 * ⚠️ 档位清单在这里是硬编码的三项 —— 它属于**协议**（2 bit 码位定死了 0/1/2），
 *    不是可自由增删的枚举选项; 加档位要三家（这里 / 固件 WARM_PROF_* / Pi _WARM_PROFILES）
 *    一起改。 */
const WARM_PROF_OPTS = [
  { code: 0, label: '默认(识别档)' },
  { code: 1, label: '看路档' },
  { code: 2, label: '人质档' },
];

/* ⚠️ 次数必须走 optKey 归一化再转数字: map.c 里整数是带后缀的 **`3u`**，
 * Number("3u") = NaN → 退化成 1 次（弹窗只显示一行, 现场踩过）。
 * 与写盘侧 map_codec._normalize_param 的 `u` 后缀是同一条约定。 */
function warmUsesOf(v) {
  const n = Number(optKey(v));
  return Math.max(1, Math.min(Number.isFinite(n) ? n : 1, 10));
}

function warmPlanDecode(plan, uses) {
  const n = warmUsesOf(uses);
  const out = [];
  for (let i = 0; i < n; i++) out.push((Number(optKey(plan)) >> (2 * i)) & 0x3);
  return out;
}

function warmPlanEncode(list) {
  let v = 0;
  list.forEach((code, i) => { v += (Number(code) & 0x3) * Math.pow(4, i); });
  return String(v);
}

function warmPlanSummary(plan, uses) {
  const list = warmPlanDecode(plan, uses);
  if (!list.some(c => c)) return '全默认';
  /* 连续相同档压成 "看路档×2", 免得长了撑爆按钮 */
  const parts = [];
  list.forEach(c => {
    const name = (WARM_PROF_OPTS.find(o => o.code === c) || WARM_PROF_OPTS[0]).label;
    const last = parts[parts.length - 1];
    if (last && last.name === name) last.n++;
    else parts.push({ name, n: 1 });
  });
  return parts.map(p => (p.n > 1 ? p.name + '×' + p.n : p.name)).join('  ');
}

function openWarmProfilesDialog(it, m, idx, uses, onChange, btn) {
  const usesN = warmUsesOf(uses);
  const raw = it.params[idx];
  const list = warmPlanDecode(raw === undefined || raw === '' ? 0 : raw, usesN);
  while (list.length < usesN) list.push(0);

  const body = h('div', {});
  body.appendChild(h('p', { class: 'modal-hint' },
    `这次预热会被 ${usesN} 个任务消费（使用次数 = ${usesN}）。给每一次指定档位: `
    + 'Pi 会在上一次用完的空档里提前切到下一档, 那一步就不用现场换档。'));

  const rowsBox = h('div', {});
  list.forEach((code, i) => {
    const sel = h('select', { class: 'p-select' });
    for (const o of WARM_PROF_OPTS) sel.appendChild(h('option', { value: String(o.code) }, o.label));
    sel.value = String(code);
    sel.onchange = (e) => { list[i] = Number(e.target.value); };
    rowsBox.appendChild(h('div', { class: 'param-row' },
      h('span', { class: 'p-name' }, '第 ' + (i + 1) + ' 次'), sel));
  });
  body.appendChild(rowsBox);

  openTuningModal('预热档位（每次使用）', body, async () => {
    const packed = warmPlanEncode(list);
    onChange(packed);
    if (btn) btn.textContent = warmPlanSummary(packed, usesN);   /* 按钮就地更新 */
    toast('已设置: ' + warmPlanSummary(packed, usesN));
    return true;
  }, '应用');
}

/* 取本积木当前的「使用次数」（widget 的 depends 参数）—— **点击时现读**, 不缓存:
 * 详情面板里改完次数不会重建按钮, 缓存了就会按旧次数弹窗。 */
function warmUsesFromItem(it, m) {
  const depName = (m.widget && m.widget.depends) || 'uses';
  const di = paramLabels(catalogOf(it.step)).indexOf(depName);
  return (di >= 0 && it.params[di] !== undefined) ? it.params[di] : 1;
}

function paramInput(it, m, idx, onChange, forceEditable) {
  const cur = (it.params[idx] !== undefined) ? String(it.params[idx]) : '';
  const tip = (m.tip ? m.tip + '  |  ' : '') + m.name;
  /* 专用控件（DSL `<控件:名字[:依赖参数]>`）: 在一切通用渲染之前拦走。
   * 目前只有 warm_profiles（"每次使用档位"）—— 见 openWarmProfilesDialog。 */
  if (m.widget && m.widget.name === 'warm_profiles') {
    const btn = h('button', {
      class: 'btn small',
      title: tip + '（点击逐次设置）',
      disabled: (!forceEditable && it.disabled) ? '' : null,
      onclick: () => openWarmProfilesDialog(it, m, idx, warmUsesFromItem(it, m), onChange, btn),
    }, cur === '' ? '设置…' : warmPlanSummary(cur, warmUsesFromItem(it, m)));
    return btn;
  }
  /* 灰色(禁用)积木: 画布上控件禁用, 但右侧详情始终可编辑
     (禁用积木的参数常需先调好再启用; 否则点灰色积木会发现参数根本点不动) */
  const lock = (!forceEditable && it.disabled) ? '' : null;

  /* 勾选框: 布尔型参数（选项恰为 0/1）。
   * ⚠️⚠️ `checked` 绝不能塞进 h() 的属性对象 —— h() 走 setAttribute,
   * `checked: false` 会写成 checked="false", 而布尔属性只要出现就算已勾选,
   * 0 的开关会显示成"开"（「蓝牙」Tab 踩过的同一个坑）。必须建完直接赋 .checked。 */
  if (isBoolOptions(m)) {
    const chk = h('input', {
      type: 'checkbox',
      title: tip + (lock ? '  (积木已禁用, 可在右侧详情中编辑)' : ''),
      disabled: lock,
      onchange: (e) => onChange(e.target.checked ? '1' : '0'),
    });
    chk.checked = optKey(cur) === '1';
    return chk;
  }

  /* 位移变量（<控件:move_var> 标记, app_script.h DSL —— 服务端派生, 前端不写死步骤名）:
   *  · 当前值是宏名 → 徽章: 点击 = 弹窗换绑/改回数字, Ctrl+点击 = 跳「位移」Tab;
   *  · 当前值是数字/空 → 数字输入框 + 「▾」（同一个选择弹窗）。
   * 徽章纯视图（无 input 即无 onchange, it.params 原样 → 保存零 diff）; ⚠️ 变量数值的
   * 常规修改入口在「位移」Tab / 调参 Tab —— 这里换绑改的是 map.c 这一步的引用。 */
  if (m.widget && m.widget.name === 'move_var') {
    const mvTip = (tip ? tip + '  |  ' : '') + '可填数字, 或点 ▾ 绑一个位移变量';
    if (MOVES.varMap && MOVES.varMap[cur]) {
      const mv = MOVES.varMap[cur];
      return h('button', {
        class: 'p-var', type: 'button', disabled: lock,
        title: cur + ' = ' + (mv.num === null || mv.num === undefined ? '?' : mv.num)
             + ' ' + (mv.unit || 'mm') + '\n点击: 换绑 / 改回数字    Ctrl+点击: 去「位移」Tab',
        onclick: (e) => {
          if (e.ctrlKey || e.metaKey) { movesRevealVar(cur); return; }
          openMoveVarPicker(it, idx, onChange);
        },
      }, mv.display);
    }
    return h('span', { class: 'p-movevar' },
      h('input', {
        value: cur,
        title: mvTip + (lock ? '  (积木已禁用, 可在右侧详情中编辑)' : ''),
        disabled: lock,
        onchange: (e) => onChange(e.target.value),
      }),
      h('button', {
        class: 'p-var', type: 'button', title: '选择位移变量', disabled: lock,
        onclick: () => openMoveVarPicker(it, idx, onChange),
      }, '▾'));
  }

  /* 兜底徽章（积木库还是旧版、没带 move_var 标记时也认得出宏名, 交互一致） */
  if (MOVES.varMap && MOVES.varMap[cur]) {
    const mv = MOVES.varMap[cur];
    return h('button', {
      class: 'p-var', type: 'button', disabled: lock,
      title: cur + ' = ' + (mv.num === null || mv.num === undefined ? '?' : mv.num)
           + ' ' + (mv.unit || 'mm') + '\n点击: 换绑 / 改回数字    Ctrl+点击: 去「位移」Tab',
      onclick: (e) => {
        if (e.ctrlKey || e.metaKey) { movesRevealVar(cur); return; }
        openMoveVarPicker(it, idx, onChange);
      },
    }, mv.display);
  }

  if (!m.options || !m.options.length) {
    return h('input', {
      value: cur,
      title: tip + (lock ? '  (积木已禁用, 可在右侧详情中编辑)' : ''),
      disabled: lock,
      onchange: (e) => onChange(e.target.value),
    });
  }

  const sel = h('select', {
    class: 'p-select',
    title: tip + (lock ? '  (积木已禁用, 可在右侧详情中编辑)' : ''),
    disabled: lock,
    onchange: (e) => onChange(e.target.value),
  });
  if (!m.options.some(o => optKey(o.value) === optKey(cur))) {   // 当前值不在枚举内(空/自定义)
    sel.appendChild(h('option', { value: cur }, cur ? cur + '  (枚举外)' : '—'));
  }
  for (const o of m.options) {
    sel.appendChild(h('option', {
      value: o.value, selected: optKey(o.value) === optKey(cur) ? '' : null,
    }, o.label));
  }
  return sel;
}

function newId() { return 'n' + (S.seq++); }

function newItemFromCatalog(cat) {
  /* 枚举/布尔参数预填第一个选项 → 新积木放下即有效, 不会出现空占位 */
  const defaults = (cat.params_meta || []).map(m =>
    (m.options && m.options.length) ? String(m.options[0].value) : '');
  /* 「机械臂动作」大积木是**虚拟条目**: type 是合成键(ARM_PLAY_<id>), 只存在于积木库,
     落到 map.c 的必须是真步骤名与预填好的动作 ID。
     ⚠️ 别用 cat.type 当 step —— 会往 map.c 里写 `ARM_PLAY_3,`（服务端 write_map
        有硬拦, 会回 400, 但那时已经白拖一次了）。 */
  return {
    id: newId(), kind: 'step', edited: true, lines: null, disabled: false,
    macro: null, step: cat.synth_step || cat.type,
    tag: (cat.desc || cat.type).split('(')[0].trim().slice(0, 16),
    params: cat.synth_params || defaults, primitive: cat.primitive || '',
  };
}

/* ==================== 数据加载 ==================== */

async function loadAll() {
  const [cat, map] = await Promise.all([
    fetch('/api/catalog').then(r => r.json()),
    fetch('/api/map').then(r => r.json()),
  ]);
  S.catalog = cat;
  S.catByType = {};
  for (const c of cat) S.catByType[c.type] = c;
  S.presets = map.presets;
  S.nextId = map.next_id || 1;
  S.runIdent = map.active;
  S.reentrantIds = map.reentrant || [];      /* 「可重入」预设 id 集合（map.h 枚举标记） */
  S.activeIdx = Math.max(0, S.presets.findIndex(p => p.ident === map.active));
  bindItems();
  S.version = map.version;
  S.selectedId = null;
  clearDirty();
  renderPalette();
  renderPresetBar();
  renderCanvas();
  renderInspector();
}

/* ==================== 预设栏（切换/新建/复制/重命名/删除） ==================== */

function renderPresetBar() {
  const sel = document.getElementById('preset-select');
  sel.textContent = '';
  S.presets.forEach((p, i) => {
    sel.appendChild(h('option', {
      value: String(i), selected: i === S.activeIdx ? '' : null,
    }, (p.ident === S.runIdent ? '★ ' : '') + p.label));
  });
  const cur = curPreset();
  document.getElementById('preset-label').value = cur ? cur.label : '';
  const box = document.getElementById('preset-active');
  box.checked = !!cur && cur.ident === S.runIdent;
  /* 「可重入的任务」勾选框（2026-10-18 加）: 状态来自 map.h 枚举注释里的
   * （可重入）标记（/api/map GET 的 reentrant 列表）。id 走 presetIdOf —— 本会话
   * 新建的预设也算得出（原来判 cur.id == null, 新建/刚保存的预设勾不了, 要重载）。
   * 只有手写表名且没进注册表的才真没有 id → 当不了跳转目标, 勾了也没意义 → 禁用。 */
  const reBox = document.getElementById('preset-reentrant');
  const reId = presetIdOf(cur);
  reBox.checked = reId !== null && (S.reentrantIds || []).includes(reId);
  reBox.disabled = reId === null;
  document.getElementById('preset-del').disabled = S.presets.length <= 1;
}

function switchPreset(idx) {
  if (idx === S.activeIdx || !S.presets[idx]) return;
  if (!confirmDiscard()) { renderPresetBar(); return; }
  S.activeIdx = idx;
  S.selectedId = null;
  bindItems();                      // 切换到该预设的 items（活引用）
  renderPresetBar();
  renderCanvas();
  renderInspector();
  document.getElementById('canvas').scrollTop = 0;
}

function confirmDiscard() {
  return !S.dirty || confirm('当前有未保存的修改，切换/新建会丢弃它们。\n（想保留请先点右上角「保存 map.c」）\n继续吗？');
}

/* 新预设的 ident: g_astScriptPreset<NEXT_ID>（ident 一旦生成就稳定, 改名只动 label）。
 *
 * ⚠️ 编号必须从服务端的 S.nextId 分配, **不能**再按「现有最大号 + 1」算:
 *    删掉预设 8 再新建, 按 max+1 会**复用 8**, 而 map.c 里可能还残留着 SCRIPT_PRESET_8
 *    的引用 → **无声指向另一条路线**（正是 ARM_ACTION_NEXT_ID 那个坑）。
 *    NEXT_ID 是单调的, 所以删号之后新预设拿到的是更大的号, 残留引用变成
 *    undefined identifier → **编译期硬失败** —— 响亮地坏掉, 胜过静默跑错。
 *    （服务端也有一道校验挡「编号空档」, 但删最大号那种情形它看不见, 见 server.py。）
 *
 * ⚠️ 这里是**创建时**定 ident, 所以就地自增 —— 撤回/重做不走这条路（预设的增删本来
 *    就不进撤回栈, 那条路由回收站兜）。 */
function nextPresetIdent() {
  return 'g_astScriptPreset' + (S.nextId++);
}

/* 预设的注册表 id。磁盘读回的预设带 p.id; **本会话新建**（还没重载过）的只有 ident
 * —— 保存成功的响应也不回填 id（只回 version/active/next_id）。服务端给合规 ident
 * 分的 id 恰是其中的号（server._preset_entries: pid = int(ident 尾号)）, 所以照解析即可。
 * 手写表名且没进过注册表 → null（它当不了跳转目标）。
 * ⚠️ 凡是「这个预设的 id 是几」都走这里 —— 直接判 p.id == null 会把刚新建/刚保存的
 *    预设当成「没有 id」（「可重入的任务」勾选框灰掉、直到重载才能勾, 2026-11 踩过）。 */
function presetIdOf(p) {
  if (!p) return null;
  if (p.id !== null && p.id !== undefined) return p.id;
  const m = /^g_astScriptPreset(\d+)$/.exec(p.ident || '');
  return m ? Number(m[1]) : null;
}

/* 预设 ident → 它在路线里作跳转目标时写的那个 token（SCRIPT_PRESET_n）。
 * 没有编号（标识不是 g_astScriptPresetN 的手写表）→ 空串, 它当不了目标。 */
function presetToken(p) {
  const id = presetIdOf(p);
  return id === null ? '' : 'SCRIPT_PRESET_' + id;
}

/* 哪些预设用「IF / 跳转」引用了这个 token → 返回那些预设的名字。
 *
 * ⚠️ 扫**所有**参数槽而不是只看第 2/3 个: STEP_IF_ELSE 的两个去向在 1、2,
 *    STEP_SCRIPT_CALL 的目标在 0 —— 按槽位写死早晚会漏。
 * ⚠️ 在前端算而不新开一个端点: S.presets 里就有全部步骤和参数, 不用改 Origin 白名单、
 *    不用加 getElementById 字面量（test_bt.py 有测试盯这两件事）。 */
function presetRefs(token) {
  const out = [];
  if (!token) return out;
  for (const p of S.presets) {
    for (const it of (p.items || [])) {
      if ((it.params || []).some(v => String(v).trim() === token)) out.push(p.label);
    }
  }
  return out;
}

/* 空预设播种一步停车: C 不允许空初始化列表, 否则编译失败 */
function seedPresetItems() {
  const cat = S.catByType['STEP_STOP'];
  return cat ? [newItemFromCatalog(cat)] : [];
}

function addPreset() {
  if (!confirmDiscard()) return;
  const p = { ident: nextPresetIdent(), label: '预设' + (S.presets.length + 1),
              items: seedPresetItems() };
  S.presets.push(p);
  S.activeIdx = S.presets.length - 1;
  S.selectedId = null;
  bindItems();
  markDirty();
  renderPresetBar(); renderCanvas(); renderInspector();
  document.getElementById('preset-label').focus();
  toast('已新建预设（记得填个好名字）');
}

function dupPreset() {
  const src = curPreset();
  if (!src) return;
  if (!confirmDiscard()) return;
  /* 深拷贝条目（params 是数组, 必须逐条复制, 否则两份预设共享同一数组） */
  const items = src.items.map(it => ({ ...it, params: it.params ? it.params.slice() : it.params }));
  items.forEach(it => { it.id = newId(); });
  S.presets.push({ ident: nextPresetIdent(), label: src.label + ' 副本', items });
  S.activeIdx = S.presets.length - 1;
  S.selectedId = null;
  bindItems();
  markDirty();
  renderPresetBar(); renderCanvas(); renderInspector();
  toast('已复制预设');
}

function delPreset() {
  if (S.presets.length <= 1) return;
  const cur = curPreset();
  /* 被「IF / 跳转」引用着的话先说清楚后果 —— 那些行会变成 C 编译期的
   * undefined identifier（**故意的硬失败**, 胜过静默指向别的预设）, 但用户得先知道
   * 他要弄坏几条路线。同「机械臂」Tab 删动作前的引用提示。 */
  const refs = presetRefs(presetToken(cur));
  const msg = refs.length
    ? '预设「' + cur.label + '」正被这些路线用「IF / 跳转」引用着:\n  · '
      + refs.join('\n  · ')
      + '\n\n删掉之后那些积木会指向一个不存在的预设, 编译会报 undefined identifier。\n'
      + '（这是故意的硬失败 —— 比无声跳到别处强）\n\n确定删除吗？'
    : '删除预设「' + cur.label + '」？（保存后从 map.c 移除）';
  if (!confirm(msg)) return;
  S.presets.splice(S.activeIdx, 1);
  S.activeIdx = Math.max(0, S.activeIdx - 1);
  S.selectedId = null;
  bindItems();
  markDirty();
  renderPresetBar(); renderCanvas(); renderInspector();
  toast('已删除预设');
}

/* ==================== 左栏: 积木库 ==================== */

function renderPalette() {
  const list = document.getElementById('palette-list');
  const q = document.getElementById('palette-search').value.trim().toLowerCase();
  list.textContent = '';

  const groups = [];
  const gIdx = {};
  for (const c of S.catalog) {
    const blob = (c.type + ' ' + (c.synth_step || '') + ' ' + c.desc + ' ' +
                  c.hint + ' ' + c.primitive).toLowerCase();
    if (q && !blob.includes(q)) continue;
    if (gIdx[c.group] === undefined) { gIdx[c.group] = groups.length; groups.push({ name: c.group, items: [] }); }
    groups[gIdx[c.group]].items.push(c);
  }
  if (!groups.length) { list.appendChild(h('p', { class: 'hint' }, '无匹配积木')); return; }

  for (const g of groups) {
    list.appendChild(h('div', { class: 'pal-group' }, g.name));
    for (const c of g.items) {
      /* 调色板条目**不做显隐判断**: 这里没有条目值可判, 而且 chip 是「这个参数存在」
         的唯一去处 —— 藏了反而会误述那些已经开了开关的既有路线。有条件就写在 tooltip 里。 */
      const paramChips = paramMeta(c).map(m =>
        h('span', {
          class: 'chip',
          title: m.name + (m.show_if ? '（需 ' + m.show_if.name + '=' + m.show_if.value + ' 才显示）' : ''),
        }, m.label));
      /* 虚拟条目(机械臂动作): 用用户起的标题当积木名 —— 那才是他在找的东西;
         真步骤用短类型名。右键虚拟条目是「去编辑那个动作」而不是改元数据。 */
      const synth = !!c.synth_step;
      /* 「否则」/「结束如果」是**结构行**: 单独插一块立刻就是失衡的路线（服务端会拒），
       * 所以它们在积木库里只作展示、不可拖也不可点。要它们请拖「如果」（一次落三行）,
       * 或在「结束如果」行上点「＋添加失败分支」。 */
      const mk = IF_MARKERS[c.type];
      const structOnly = (mk === 'else' || mk === 'close');
      const item = h('div', {
        class: 'pal-item' + (structOnly ? ' pal-struct' : ''),
        title: structOnly
          ? (c.type + '  是「如果」块的结构行，不单独放：拖「如果」会一次落下 如果/否则/结束如果')
          : ((c.synth_step || c.type) + (synth ? '  拖入/点击添加; 右键去编辑该动作' : '  拖入/点击添加; 右键编辑元数据')),
        onclick: () => {
          if (structOnly || MDND.justDragged()) return;
          appendItems(createItemsFromCatalog(c));
        },
        oncontextmenu: (e) => { e.preventDefault(); openPaletteMenu(c); },
        onpointerdown: (e) => {
          if (structOnly || e.button !== 0) return;
          MDND.begin({ mode: 'new', type: c.type }, e);
        },
      },
        h('div', { class: 'pal-name' }, synth ? (c.desc || c.type) : shortType(c.type)),
        h('div', { class: 'pal-desc' }, synth ? (c.hint || '机械臂动作') : (c.desc || c.hint || '')),
        paramChips.length ? h('div', { class: 'pal-chips' }, paramChips) : null,
      );
      list.appendChild(item);
    }
  }
}

/* 积木库条目的右键: 真步骤 → 编辑元数据(写回 app_script.h);
 * 虚拟条目(机械臂动作) → 跳到「机械臂」Tab 去编辑那个动作本身。 */
function openPaletteMenu(c) {
  if (c.synth_step) { armOpenPreset(c.action_id); return; }
  openMetaEditor(c);
}

function shortType(t) { return t.replace(/^STEP_/, ''); }

/* ==================== 嵌套容器 IF（如果 / 否则 / 结束如果） ====================
 * map.c 里这三块标记是**扁平的**三条数组项, 嵌套关系完全由**位置**决定:
 * 「如果」开一层, 同层的「否则」把这一层切成两半, 同层的「结束如果」收口。
 *
 * 所以画布**不需要**给条目加 children/parent 字段: 渲染前扫一遍算出每条的层级就够了。
 * 而且这带来一条很有用的性质 —— **往扁平数组任意位置插一条, 都不可能让标记失衡**
 * （失衡只可能来自「删掉其中一块标记」）, 所以拖放落点不需要做任何嵌套合法性判断。
 *
 * ⚠️ 三个名字必须与 app_script.h 的枚举成员逐字一致; 改一处就得改两处 ——
 *    test_server.py 里有一条测试对着 catalog 核这三个字面量。
 */
const IF_OPEN = 'STEP_IF';
const IF_ELSE = 'STEP_ELSE';
const IF_CLOSE = 'STEP_END_IF';
const IF_MARKERS = { [IF_OPEN]: 'open', [IF_ELSE]: 'else', [IF_CLOSE]: 'close' };
const IF_INDENT_PX = 22;

function ifMarkerKind(it) {
  return (it && it.kind === 'step' && IF_MARKERS[it.step]) || '';
}

/* 扫一遍扁平列表 → 每条的层级 + 配对关系 + **折叠键**。落单的标记也照常算（多出来的
 * 「否则」/「结束如果」按层级 0 画）—— **先得能画出来**, 用户才有机会去修; 拦保存是服务端
 * 的事（错误码 unbalanced_if）。这条路径在「队友手改了 map.c 并 push」之后一定会走到。
 *
 * `keyOf` = 每个「如果」块的**折叠键**: `<预设 ident>#<路径>`, 路径是「第几个『如果』块」的链
 * （第 0 层的第 2 个 = `#1`; 它里面第 0 层… 即 `#1.0`）。这样**增删普通积木不会让折叠状态
 * 错位**（条目 id 是行号生成的, 一插就整体挪位 —— 别拿它当键）。
 * ⚠️ 插入/删除**另一个「如果」块**仍会让它后面的路径挪位, 这是位置式锚点的固有代价;
 *    换成用户可见的名字（tag）反而不稳 —— 改个名字就丢状态。 */
function scanIfStructure(items, presetIdent) {
  const depth = new Map();       /* 条目 id → 缩进层级 */
  const hasElse = new Map();     /* 「如果」id → 这个块有没有「否则」 */
  const elseOfIf = new Map();    /* 「如果」id → 配对的「否则」id */
  const endOfIf = new Map();     /* 「如果」id → 配对的「结束如果」id */
  const keyOf = new Map();       /* 「如果」id → 折叠键 */
  const stack = [];              /* 还没收口的「如果」id */
  const counters = [0];          /* counters[层] = 该层已经遇到过几个「如果」 */
  const path = [];               /* 当前所在的位置链 */
  const prefix = presetIdent || '';

  for (const it of items || []) {
    const k = ifMarkerKind(it);
    if (k === 'open') {
      const d = stack.length;
      path.push(counters[d] || 0);
      counters[d] = (counters[d] || 0) + 1;
      counters[d + 1] = 0;                     /* 进下一层, 该层重新数 */
      depth.set(it.id, d);                     /* 与自己的「否则/结束」同层 */
      keyOf.set(it.id, prefix + '#' + path.join('.'));
      stack.push(it.id);
      continue;
    }
    if (k === 'else') {
      if (stack.length) {
        const top = stack[stack.length - 1];
        depth.set(it.id, stack.length - 1);
        hasElse.set(top, true);
        elseOfIf.set(top, it.id);
      } else {
        depth.set(it.id, 0);
      }
      continue;
    }
    if (k === 'close') {
      if (stack.length) {
        const top = stack.pop();
        depth.set(it.id, stack.length);        /* 弹出后 = 「如果」自己那一层 */
        endOfIf.set(top, it.id);
        path.pop();
      } else {
        depth.set(it.id, 0);
      }
      continue;
    }
    depth.set(it.id, stack.length);            /* 普通积木/分区: 当前开着的层数 */
  }
  return { depth, hasElse, elseOfIf, endOfIf, keyOf };
}

const EMPTY_IF_STRUCT = { depth: new Map(), hasElse: new Map(), elseOfIf: new Map(),
                          endOfIf: new Map(), keyOf: new Map() };

/* 一个「如果」块的折叠键（结构信息还没算出来时退回条目 id —— 只为兜底, 不该发生）。 */
function ifKey(it) { return ifStruct().keyOf.get(it.id) || ('#' + it.id); }

function ifCollapsed(it) { return !S.ifExpanded.has(ifKey(it)); }

/* 改折叠状态并落盘（**不重绘**）。与重绘分开是为了可测 —— 状态机是纯逻辑, 画布那套
 * 只有浏览器里跑得起来（测试是 node 桩, 没有 DOM）。 */
function setIfExpanded(it, on) {
  const key = ifKey(it);
  if (on) S.ifExpanded.add(key);
  else S.ifExpanded.delete(key);
  lsSetJson(IF_EXPANDED_KEY, [...S.ifExpanded]);
}

/* 折叠开关。⚠️ **不 markDirty**: 折叠是视图偏好, 不是 map.c 的改动 —— 置脏会让「保存」
 * 按钮亮起来、让 `/api/rev` 那条「有未保存改动」的提示条常驻。 */
function toggleIfCollapsed(it) {
  setIfExpanded(it, !S.ifExpanded.has(ifKey(it)));
  renderCanvas();
  renderInspector();     /* 折起来可能把当前选中项藏进去（renderCanvas 会清掉选中） */
}

/* 折叠时要**藏起来**的条目 id: 每个折叠的「如果」块, 从它的下一条一路藏到配对的「结束如果」
 * （含两块标记 —— 于是画布上只剩一个头, 落点锚也只剩头前后的位置, 正好符合"折起来就看不见"）。
 * 嵌套在其中、自己也是折叠的块同理（外层折叠时它整个都看不见）。
 *
 * 用「当前处在几个折叠块内部」的水位线算, **不用递归**: 外层折叠 → depth > 0 → 里面的
 * 「如果」头自己也藏起来; 收口行在深度归零**之前**判断, 所以它也跟着藏。 */
function hiddenIfIds(items, st) {
  const hidden = new Set();
  const tops = [];          /* 每个未收口块的「是否折叠」 */
  let depth = 0;            /* 当前处在几个折叠块内部 */

  for (const it of items || []) {
    const k = ifMarkerKind(it);
    if (depth > 0) hidden.add(it.id);
    if (k === 'open') {
      const collapsed = !S.ifExpanded.has(st.keyOf.get(it.id));
      tops.push(collapsed);
      if (collapsed) depth++;
    } else if (k === 'close') {
      if (tops.length && tops.pop()) depth--;
    }
  }
  return hidden;
}

function ifStruct() { return S.ifStruct || EMPTY_IF_STRUCT; }
function ifDepthOf(it) { const d = ifStruct().depth.get(it.id); return d === undefined ? 0 : d; }

/* 行首缩进。标记行与它自己那层的「如果」对齐（所以嵌套的「结束如果」也缩进）,
 * 分支里的积木再往里一格。 */
function indentOf(it) { return ifDepthOf(it) * IF_INDENT_PX; }

/* 一个「如果」块在扁平数组里的区间 [起始下标, 结束下标]（含两端标记）。找不到收口
 * 标记时右端取数组末尾 —— 与固件「扫到表尾就收掉本帧」同一个兜底。 */
function ifSpanIdx(items, ifIdx) {
  let d = 0;
  for (let i = ifIdx; i < items.length; i++) {
    const k = ifMarkerKind(items[i]);
    if (k === 'open') d++;
    else if (k === 'close') {
      d--;
      if (d === 0) return [ifIdx, i];
    }
  }
  return [ifIdx, items.length - 1];
}

/* 一个「否则」标记后面、到配对「结束如果」之前的区间（右端**不含**收口标记）。
 * 左端不含「否则」自己。落单的「否则」→ 空区间。 */
function elseSpanIdx(items, elseIdx) {
  let d = 0;
  for (let i = elseIdx + 1; i < items.length; i++) {
    const k = ifMarkerKind(items[i]);
    if (k === 'open') d++;
    else if (k === 'close') {
      if (d === 0) return [elseIdx + 1, i - 1];
      d--;
    }
  }
  return [elseIdx + 1, items.length - 1];
}

/* 整个「如果」块（头 + 两个分支 + 两块标记）的条目 id 列表 —— 拖动与删除都以它为单位。
 * 区间是**连续**的, 所以整组搬家之后标记仍然配对（位置式嵌套只看相对顺序, 不看绝对位置）。
 * 传入的不是「如果」就返回 null（`makeDnd` 会退化成「只搬它自己」, 与普通积木一致）。 */
function ifSpanIds(id) {
  const idx = S.items.findIndex(i => i.id === id);
  if (idx < 0 || ifMarkerKind(S.items[idx]) !== 'open') return null;
  const [a, b] = ifSpanIdx(S.items, idx);
  return S.items.slice(a, b + 1).map(i => i.id);
}

/* ==================== 中栏: 画布（分区 + 积木） ==================== */

function renderCanvas() {
  const root = document.getElementById('canvas-inner');
  root.textContent = '';
  S.zonesMeta = [];
  S.ifStruct = scanIfStructure(S.items, (S.presets[S.activeIdx] || {}).ident || '');
  /* 折叠的块: 只画头, 块内与两块标记行都不画（见 hiddenIfIds）。 */
  const hidden = hiddenIfIds(S.items, S.ifStruct);
  S.ifHidden = hidden;
  /* 选中项被折进去了 → 清掉选中。否则检查器一直显示一块画布上根本看不见的积木
   * （"删了但它还在右边"那种困惑）。切换预设/重载/折叠都经这一条, 只有一处判据。 */
  if (S.selectedId && hidden.has(S.selectedId)) S.selectedId = null;

  /* 按扁平 items 切分区: section 开新 zone, 之前无 section 的归 "(全局)"。
   * ⚠️ 只有**层级 0** 的分区标题才开新 zone: 落在 IF 分支里的分区标题退化成 zone 内一行缩进
   *    的小标题。若照旧切, IF 头与它的「结束如果」会分居两个 <section> —— 缩进层级与拖放
   *    落点都会错位, 而那是**完全合法**的 map.c（服务端不会拦）。
   * ⚠️ 折叠掉的条目在**这里**就滤掉（而不是等渲染时再跳过）: 下游的分区步数、落点锚
   *    `z.ids`、`nextFirstId`、「空分区」提示全都读 `z.items`, 在一处滤干净就没有第二份判据。 */
  const zones = [];
  let cur = { section: null, items: [] };
  for (const it of S.items) {
    if (hidden.has(it.id)) continue;
    if (it.kind === 'section' && ifDepthOf(it) === 0) {
      zones.push(cur);
      cur = { section: it, items: [] };
    } else {
      cur.items.push(it);
    }
  }
  zones.push(cur);

  for (const z of zones) {
    const sec = z.section;
    const title = sec ? sec.title : '(全局)';
    const zone = h('section', {
      class: 'zone zc' + zoneColorIdx(title) + (sec && sec.disabled ? ' zdis' : ''),
      onclick: (e) => { if (e.target === zone || e.target.classList.contains('zone-head')) selectItem(sec ? sec.id : null); },
    });

    const titleEl = h('span', { class: 'zone-title' }, title);
    if (sec) {
      titleEl.title = '双击编辑分区名';
      titleEl.addEventListener('dblclick', (e) => {
        e.stopPropagation();
        editInPlace(titleEl, sec.title, (v) => {
          sec.title = v; sec.edited = true; markDirty(); renderCanvas();
        });
      });
    }

    const head = h('div', { class: 'zone-head' },
      sec ? h('button', {
        class: 'eye' + (sec.disabled ? ' off' : ''), title: '禁用/启用该分区（整段注释）',
        onclick: (e) => { e.stopPropagation(); sec.disabled = !sec.disabled; markDirty(); renderCanvas(); },
      }, sec.disabled ? '🚫' : '👁') : null,
      titleEl,
      h('span', { class: 'zone-count' },
        z.items.filter(i => i.kind === 'step' && !ifMarkerKind(i)).length + ' 步'),
    );
    zone.appendChild(head);

    /* 所有行（含 IF 标记行）都是 zone 的**直接子节点**, 且都带 data-id ——
     * makeDnd 的 blockEl / computeBeforeId / insertPlaceholder 因此天然把它们当落点锚:
     * 「落在标记行上方」正好等于「落在 then 分支末 / else 分支末 / 块前 / 块后」。 */
    const ids = [];
    for (const it of z.items) {
      const k = ifMarkerKind(it);
      zone.appendChild(k === 'open' ? renderIfHead(it)
                      : k ? renderMarkerRow(it, k)
                      : it.kind === 'step' ? renderBlock(it)
                      : it.kind === 'section' ? renderInnerSection(it)
                      : renderRaw(it));
      if (it.kind === 'step') ids.push(it.id);
    }
    if (!z.items.length) zone.appendChild(h('div', { class: 'zone-empty' }, '空分区, 拖积木进来'));
    root.appendChild(zone);
    S.zonesMeta.push({ el: zone, ids });
  }

  /* 每区记录下一区首块 id（拖到区尾的插入锚点）; 末区无下一区 → null */
  for (let i = 0; i < S.zonesMeta.length; i++) {
    const nxt = S.zonesMeta[i + 1];
    S.zonesMeta[i].nextFirstId = (nxt && nxt.ids.length) ? nxt.ids[0] : null;
  }
}

function renderBlock(it) {
  const cat = catalogOf(it.step);

  const paramRow = h('div', { class: 'b-params' });
  const rows = [];
  /* 驱动的参数一改就地刷新从属参数的显隐/启用 —— 这里**不能**重画整块/整个画布
     （重画会把手里的控件换掉; 拖拽中重建 DOM 的坑见 armRefreshBlock 的注释）。 */
  function syncVis() {
    for (const r of rows) {
      r.el.classList.toggle('hidden', !paramVisible(it, r.m, cat));
      /* 启用联动（<启用:…>）: 条件不满足 = 禁用, 与"积木已禁用"的 lock 叠加（谁禁都算禁）。
       * ⚠️ 初始态也走这里 —— 与显隐同一条路, 两条路必然漂移(armRefreshModified 的教训)。 */
      if (r.inp && r.m.enable_if) {
        r.inp.disabled = !paramEnabled(it, r.m, cat) || (!r.fe && it.disabled);
      }
    }
  }
  paramMeta(cat).forEach((m, i) => {
    const inp = paramInput(it, m, i, (v) => {
      it.params[i] = v; it.edited = true; markDirty(); syncVis();
    });
    const row = h('label', { class: 'p-item' },
      h('span', { class: 'p-name' }, m.label),
      inp,
      m.unit ? h('span', { class: 'p-unit' }, m.unit) : null,
    );
    rows.push({ m, el: row, inp, fe: false });
    paramRow.appendChild(row);
  });
  syncVis();   /* 初始态与更新态走同一条路 —— 两条路必然漂移(armRefreshModified 的教训) */

  return h('div', {
    id: 'blk-' + it.id,
    class: 'block' + (it.disabled ? ' disabled' : '') + (S.selectedId === it.id ? ' sel' : ''),
    'data-id': it.id,
    /* 缩进 = 它落在第几层 IF 分支里（顶层积木是 0）。纯渲染量, 不落进 map.c。 */
    style: 'margin-left:' + indentOf(it) + 'px',
    onclick: (e) => { if (!e.target.closest('input,select,button') && !MDND.justDragged()) selectItem(it.id); },
    oncontextmenu: (e) => {
      if (e.target.closest('input,select,button')) return;
      e.preventDefault();
      selectItem(it.id);
      openMetaEditor(cat, it);
    },
    onpointerdown: (e) => {
      if (e.button !== 0) return;
      if (e.target.closest('input,select,button')) return;
      MDND.begin({ mode: 'move', id: it.id }, e);
    },
  },
    h('div', { class: 'b-head' },
      h('button', {
        class: 'eye' + (it.disabled ? ' off' : ''), title: it.disabled ? '启用（去掉行首 //）' : '禁用（行首加 //）',
        onclick: (e) => { e.stopPropagation(); it.disabled = !it.disabled; markDirty(); renderCanvas(); },
      }, it.disabled ? '🚫' : '👁'),
      h('span', { class: 'b-tag' }, it.tag || it.step),
      h('span', { class: 'b-type' }, shortType(it.step)),
      it.primitive ? h('span', { class: 'b-prim' }, '// ' + it.primitive) : null,
      /* 「机械臂动作」大积木: 不展开内部步骤（那会把它变成一串普通积木, 失去
       * 「一处改、处处跟着变」的意义）, 但给一个跳到来源预设去改的入口。 */
      armActionIdOf(it) ? h('button', {
        class: 'eye', title: '去「机械臂」Tab 编辑这个动作',
        onclick: (e) => { e.stopPropagation(); armOpenPreset(armActionIdOf(it)); },
      }, '✎') : null,
    ),
    paramRow,
  );
}

/* 这一步是不是「机械臂动作」大积木? 是就返回它的动作 ID（ARM_ACT_<n> → n）。
 * 解析不出来就返回 0（当作普通步骤, 不给 ✎）。 */
function armActionIdOf(it) {
  if (it.step !== 'STEP_ARM_PLAY_WAIT' && it.step !== 'STEP_ARM_PLAY_NO_WAIT') return 0;
  const m = /^ARM_ACT_(\d+)$/.exec((it.params || [])[0] || '');
  return m ? parseInt(m[1], 10) : 0;
}

/* ==================== 嵌套容器 IF 的渲染 ==================== */

/* 「否则」/「结束如果」用的最小条目（**不依赖 catalog**）: 积木库比页面新时
 * catalogOf() 会回退成空对象, 那时也必须能落出成对的标记。 */
function newMarkerItem(step) {
  const cat = catalogOf(step);
  return {
    id: newId(), kind: 'step', edited: true, lines: null, disabled: false,
    macro: null, step,
    tag: (cat.desc || step).split('(')[0].trim().slice(0, 16),
    params: [], primitive: cat.primitive || '',
  };
}

/* 拖出/点出「如果」时**一次落三行**: C 数组里三块标记必须成对, 只落一块立刻就是失衡
 * 状态（服务端保存会拒）。失败分支默认空着, 不想要就在「否则」行上点 ✕ 移除。
 * 其它积木照旧只落一行。 */
function createItemsFromCatalog(cat) {
  if (!cat) return [];
  if (cat.type !== IF_OPEN) return [newItemFromCatalog(cat)];
  return [newItemFromCatalog(cat), newMarkerItem(IF_ELSE), newMarkerItem(IF_CLOSE)];
}

/* 「如果」头: 条件下拉**内联在头部**（「如果 条件:[▼]」）, 不再有 b-params 行。
 * ⚠️ 拖它 = 拖**整个块**（MDND 的 cfg.dragSet 给的是 ifSpanIds, 头/两个分支/两块标记
 *    一起搬）—— 区间连续, 所以搬完标记仍然配对。绝不能只搬头那一行。
 * ⚠️ 眼睛是**整块**开关（toggleIfBlock）: 只注释掉「如果」一行 = 「永远走 else 分支」,
 *    那是个静默改变控制流的开关, 比不提供危险得多。 */
function renderIfHead(it) {
  const cat = catalogOf(it.step);
  const m = paramMeta(cat)[0];
  const hasElse = !!ifStruct().hasElse.get(it.id);
  const collapsed = ifCollapsed(it);
  /* 折起来时给一行摘要 —— 否则整个块只剩一个「如果 xxx」, 看不出里面有多少东西 */
  let sum = null;
  if (collapsed) {
    const at = S.items.indexOf(it);
    const [a, b] = ifSpanIdx(S.items, at);
    const n = S.items.slice(a + 1, b)
      .filter(x => x.kind === 'step' && !ifMarkerKind(x)).length;
    sum = (hasElse ? '2 个分支' : '1 个分支（无「否则」）') + ' · ' + n + ' 步';
  }
  return h('div', {
    id: 'blk-' + it.id,
    class: 'block if-head' + (collapsed ? ' if-collapsed' : '')
         + (it.disabled ? ' disabled' : '') + (S.selectedId === it.id ? ' sel' : ''),
    'data-id': it.id,
    style: 'margin-left:' + indentOf(it) + 'px',
    onclick: (e) => { if (!e.target.closest('input,select,button') && !MDND.justDragged()) selectItem(it.id); },
    oncontextmenu: (e) => {
      if (e.target.closest('input,select,button')) return;
      e.preventDefault(); selectItem(it.id); openMetaEditor(cat, it);
    },
    onpointerdown: (e) => {
      if (e.button !== 0) return;
      if (e.target.closest('input,select,button')) return;
      MDND.begin({ mode: 'move', id: it.id }, e);   /* ids 由 cfg.dragSet 展开成整块 */
    },
  },
    h('div', { class: 'b-head' },
      /* 倒三角 = 折叠开关（**默认折叠**）。它是 button, 所以挂在整个头上的
       * onclick/onpointerdown 都会跳过它 —— 点它既不选中也不启动拖拽。 */
      h('button', {
        class: 'eye disc', title: collapsed ? '展开这一块' : '折叠这一块（只留标题）',
        onclick: (e) => { e.stopPropagation(); toggleIfCollapsed(it); },
      }, collapsed ? '▶' : '▼'),
      h('button', {
        class: 'eye' + (it.disabled ? ' off' : ''),
        title: it.disabled ? '启用整块（块内每一行去掉行首 //）'
                           : '禁用整块（块内每一行都加行首 //；会一并清掉各步原来各自的禁用状态）',
        onclick: (e) => { e.stopPropagation(); toggleIfBlock(it.id, !it.disabled); },
      }, it.disabled ? '🚫' : '👁'),
      h('span', { class: 'b-tag' }, '如果'),
      m ? h('span', { class: 'if-cond' },
        h('span', { class: 'p-name' }, m.label + ':'),
        paramInput(it, m, 0, (v) => { it.params[0] = v; it.edited = true; markDirty(); }),
        m.unit ? h('span', { class: 'p-unit' }, m.unit) : null) : null,
      sum ? h('span', { class: 'if-sum' }, sum) : null,
      it.primitive ? h('span', { class: 'b-prim' }, '// ' + it.primitive) : null,
      h('button', {
        class: 'btn tiny', title: '删除整个「如果」块（含两个分支的全部积木）',
        onclick: (e) => { e.stopPropagation(); deleteItem(it.id); },
      }, '✕ 删除整块'),
    ),
    (collapsed || hasElse) ? null
      : h('div', { class: 'branch-hint' }, '没有「否则」分支：条件不成立时直接继续'),
  );
}

/* 「否则」/「结束如果」结构行。它们**是** .block 的子类: makeDnd 的 flip() 靠
 * root.querySelectorAll('.block') 收集要滑开的元素, 少了这个类名, 拖动旁边的积木时
 * 这两行会原地不动、占位槽换位看着像卡住。但它们不挂 onpointerdown/onclick —— 不参与拖拽,
 * 也不能单独选中或禁用（见 deleteItem / toggleIfBlock 的注释）。 */
function renderMarkerRow(it, kind) {
  const st = ifStruct();
  const wrap = (children) => h('div', {
    id: 'blk-' + it.id,
    class: 'block marker m-' + kind + (it.disabled ? ' disabled' : ''),
    'data-id': it.id,
    style: 'margin-left:' + indentOf(it) + 'px',
  }, h('div', { class: 'b-head' }, children));

  if (kind === 'else') {
    return wrap([
      h('span', { class: 'mk-name' }, '否则'),
      h('span', { class: 'mk-note' }, '条件成立时跳过下面这一段'),
      h('button', {
        class: 'btn tiny', title: '移除失败分支（连同它下面的积木一起删掉）',
        onclick: (e) => { e.stopPropagation(); removeElseBranch(it.id); },
      }, '✕'),
    ]);
  }

  /* 收口行。找不到配对的「如果」（手改过）就只画一行「结束如果」。 */
  const ifItem = S.items.find(x => st.endOfIf.get(x.id) === it.id);
  if (ifItem && !st.hasElse.get(ifItem.id)) {
    /* 没有失败分支: 这一行显示成「失败 继续」——与「条件不成立就直接继续」的读法一致,
     * 并给一个补分支的入口。 */
    return wrap([
      h('span', { class: 'mk-name' }, '失败 继续'),
      h('button', {
        class: 'btn tiny', title: '给这个「如果」补一块「否则」失败分支',
        onclick: (e) => { e.stopPropagation(); addElseBranch(ifItem.id); },
      }, '＋ 添加失败分支'),
    ]);
  }
  return wrap([h('span', { class: 'mk-name' }, '结束如果')]);
}

/* 落在 IF 分支里的分区标题: **不开新 zone**（那会把「如果」头与它的「结束如果」劈成两个
 * <section>, 缩进与落点全乱）, 退化成 zone 内一行缩进的小标题。 */
function renderInnerSection(it) {
  const el = h('div', {
    class: 'block marker sec-inline',
    'data-id': it.id,
    style: 'margin-left:' + indentOf(it) + 'px',
    title: '分区标题（在分支内）',
    onclick: (e) => { if (!e.target.closest('button') && !MDND.justDragged()) selectItem(it.id); },
  },
    h('div', { class: 'b-head' },
      h('span', { class: 'mk-name' }, it.title || '(分区)'),
      h('span', { class: 'mk-note' }, '分区标题'),
    ),
  );
  const t = el.querySelector('.mk-name');
  t.addEventListener('dblclick', (e) => {
    e.stopPropagation();
    editInPlace(t, it.title, (v) => { it.title = v; it.edited = true; markDirty(); renderCanvas(); });
  });
  return el;
}

/* 整块启用/禁用: 把「如果」到配对「结束如果」之间**每一行**的 disabled 一起翻
 * （= 手工把整块注释掉的效果, map.c 里标记仍然成对, 于是服务端的配对校验照样通过）。
 * ⚠️ 会丢掉分支内各步**各自**的禁用状态 —— 恢复时一律置回 false, 所以用 toast 说清楚。 */
function toggleIfBlock(ifId, on) {
  const idx = S.items.findIndex(i => i.id === ifId);
  if (idx < 0) return;
  const [a, b] = ifSpanIdx(S.items, idx);
  for (let i = a; i <= b; i++) S.items[i].disabled = !!on;
  markDirty(); renderCanvas(); renderInspector();
  if (!on) toast('已启用整块「如果」（块内各步原来各自的禁用状态已一并清掉）');
}

/* 给「如果」补一块「否则」: 插在配对「结束如果」**之前**（于是失败分支初始为空）。 */
function addElseBranch(ifId) {
  const idx = S.items.findIndex(i => i.id === ifId);
  if (idx < 0) return;
  const [, b] = ifSpanIdx(S.items, idx);
  S.items.splice(b, 0, newMarkerItem(IF_ELSE));
  markDirty(); renderCanvas();
  toast('已添加「否则」失败分支（默认是空的, 把积木拖到它下面）');
}

/* 移除失败分支 = 删掉「否则」标记**连同它下面的整段内容**。
 * ⚠️ 绝不能只删标记那一行 —— 位置式嵌套下那只等于把失败分支的积木**并进 then 分支**,
 *    是个静默改变控制流的操作。 */
function removeElseBranch(elseId) {
  const idx = S.items.findIndex(i => i.id === elseId);
  if (idx < 0) return;
  const [a, b] = elseSpanIdx(S.items, idx);
  const n = Math.max(0, b - a + 1);
  S.items.splice(idx, b - idx + 1);
  if (S.selectedId && !S.items.some(i => i.id === S.selectedId)) S.selectedId = null;
  markDirty(); renderCanvas(); renderInspector();
  toast(n ? ('已移除失败分支（连带 ' + n + ' 块积木）') : '已移除失败分支');
}

function renderRaw(it) {
  const first = (it.lines || []).find(l => l.trim()) || '';
  return h('div', { class: it.lines && it.lines[0] && it.lines[0].trim() ? 'rawline' : 'rawgap',
                    title: (it.lines || []).join('\n') }, first.trim());
}

/* 行内就地编辑（双击分区标题等） */
function editInPlace(el, text, onCommit) {
  const inp = h('input', { class: 'inline-edit', value: text });
  el.replaceWith(inp);
  inp.focus(); inp.select();
  let done = false;
  /* 值没变就不回调 onCommit: Esc / 原样失焦都算「取消」—— 不然只是双击看了一眼,
   * edited/dirty 全亮、/api/rev 也因此改走提示条 */
  const commit = (v) => {
    if (done) return;
    done = true;
    if (v !== text) onCommit(v);
  };
  inp.addEventListener('blur', () => commit(inp.value.trim() || text));
  inp.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') inp.blur();
    else if (e.key === 'Escape') commit(text);
  });
}

function selectItem(id) {
  S.selectedId = id;
  document.querySelectorAll('.block.sel').forEach(el => el.classList.remove('sel'));
  const el = document.getElementById('blk-' + id);
  if (el) el.classList.add('sel');
  renderInspector();
}

/* ==================== 右栏: 检查器 ==================== */

/* 积木名（tag）里**会被服务端写盘时替换掉**的字符, 返回一句提示; 干净则返回 ''。
 *
 * ⚠️ 这里只**检测并提示**, 不重写转换规则: 转换的真值在服务端
 *    map_codec.sanitize_tag（`"`→`'`、反斜杠→`/`、`//`→`/`、去掉不成对的 ASCII 括号）。
 *    前端复刻一份必然与服务端漂移, 而"照前端洗过的值显示、磁盘上却是另一个值"比不洗更糟。
 *    所以要提示就提示"会发生什么", 别自己算出结果来。
 * ⚠️ 不成对的括号会让 map_codec._parse_body 的括号计数错位 —— 条目的参数被刷错位,
 *    表现为「重载后邻近的积木被悄悄改写」, 比编译报错难查得多。
 */
function mapTagIssues(v) {
  const s = String(v == null ? '' : v);
  const hits = [];
  if (s.indexOf('"') >= 0) hits.push('双引号');
  if (s.indexOf('\\') >= 0) hits.push('反斜杠');
  if (s.indexOf('//') >= 0) hits.push('//');
  const opens = (s.match(/\(/g) || []).length;
  const closes = (s.match(/\)/g) || []).length;
  if (opens !== closes) hits.push('不成对的括号');
  if (!hits.length) return '';
  return '含 ' + hits.join('、') + ' —— 保存时会自动替换（否则会写坏 map.c 里的 C 字符串，'
       + '甚至让重载后邻近积木的参数错位）';
}

function renderInspector() {
  const body = document.getElementById('inspector-body');
  body.textContent = '';
  const it = S.items.find(i => i.id === S.selectedId);
  if (!it) { body.appendChild(h('p', { class: 'hint' }, '点选积木或分区查看详情')); return; }

  if (it.kind === 'section') {
    body.appendChild(h('h3', {}, '分区'));
    body.appendChild(h('label', { class: 'fld' }, '标题（双击画布标题也可编辑）',
      h('input', { value: it.title, onchange: (e) => { it.title = e.target.value; it.edited = true; markDirty(); renderCanvas(); } })));
    body.appendChild(h('label', { class: 'fld' }, '注释内容'));
    body.appendChild(h('pre', { class: 'sec-text' }, it.text || ''));
    return;
  }

  if (it.kind !== 'step') {
    body.appendChild(h('h3', {}, '原始行'));
    body.appendChild(h('pre', { class: 'sec-text' }, (it.lines || []).join('\n')));
    return;
  }

  const cat = catalogOf(it.step);
  body.appendChild(h('h3', {}, it.tag || it.step));
  if (it.disabled) {
    body.appendChild(h('div', { class: 'badge-off' }, '🚫 该积木已禁用（保存后为行首 // 注释），参数仍可编辑'));
  }
  if (it.step === IF_OPEN) {
    body.appendChild(h('p', { class: 'hint' },
      '「如果」是容器积木：条件成立时执行它下面直到「否则」为止的积木；不成立时执行「否则」'
      + '下面直到「结束如果」为止的；没有「否则」就直接继续往下走。'
      + '删除它会连同两个分支一起删（那是唯一不会把标记删失衡的删法）。'));
  } else if (ifMarkerKind(it)) {
    body.appendChild(h('p', { class: 'hint' },
      '这是「如果」块的结构行（否则 / 结束如果），不能单独编辑、禁用或删除 —— '
      + '它一旦单独消失，后面整段的归属就变了。请整块处理：头部那块「如果」上有眼睛（整块启用/禁用）和 ✕（整块删除）。'));
    return;
  }
  body.appendChild(h('div', { class: 'kv' }, h('span', {}, '类型'), h('code', {}, it.step)));
  body.appendChild(h('div', { class: 'kv' }, h('span', {}, '分组'), h('span', {}, cat.group || '-')));
  if (cat.desc) body.appendChild(h('div', { class: 'kv' }, h('span', {}, '说明'), h('span', {}, cat.desc)));
  if (cat.primitive) body.appendChild(h('div', { class: 'kv' }, h('span', {}, '原语'), h('code', {}, cat.primitive + '()')));
  if (cat.hint) body.appendChild(h('div', { class: 'kv' }, h('span', {}, '参数说明'), h('span', {}, cat.hint)));

  const tagWarn = h('div', { class: 'fld-warn hidden' });
  const syncTagWarn = (v) => {
    tagWarn.textContent = mapTagIssues(v);
    tagWarn.classList.toggle('hidden', !tagWarn.textContent);
  };
  const tagInput = h('input', {
    value: it.tag,
    oninput: (e) => syncTagWarn(e.target.value),
    onchange: (e) => { it.tag = e.target.value; it.edited = true; markDirty(); renderCanvas(); },
  });
  body.appendChild(h('label', { class: 'fld' }, '积木名 (tag)', tagInput));
  syncTagWarn(it.tag);
  body.appendChild(tagWarn);

  const rows = [];
  function syncVis() {
    for (const r of rows) {
      r.el.classList.toggle('hidden', !paramVisible(it, r.m, cat));
      /* 启用联动与画布同一条判据（详情页 forceEditable=true, 只有 enable_if 会禁） */
      if (r.inp && r.m.enable_if) r.inp.disabled = !paramEnabled(it, r.m, cat);
    }
  }
  paramMeta(cat).forEach((m, i) => {
    const inp = paramInput(it, m, i, (v) => {
      it.params[i] = v; it.edited = true; markDirty();
      syncVis();        /* 先刷本窗格, 再重画画布: 画布那边的显隐/启用由同一条判据算出 */
      renderCanvas();
    }, true),                                   /* 详情页始终可编辑, 含禁用积木 */
    row = h('div', { class: 'fld param-row', title: (m.tip ? m.tip + '  |  ' : '') + m.name },
      h('span', { class: 'p-name' }, m.label),
      inp,
      h('span', { class: 'p-unit' }, m.unit || ''),
    );
    rows.push({ m, el: row, inp, fe: true });
    body.appendChild(row);
  });
  syncVis();

  body.appendChild(h('div', { class: 'insp-actions' },
    h('button', { class: 'btn danger', onclick: () => { deleteItem(it.id); } },
      it.step === IF_OPEN ? '删除整个「如果」块' : '删除'),
  ));
}

/* ==================== 增删 ==================== */

/* 一次落多行（「如果」要同时落 如果+否则+结束如果）。 */
function appendItems(list) {
  if (!list || !list.length) return;
  for (const it of list) S.items.push(it);
  markDirty();
  renderCanvas();
  selectItem(list[0].id);
  document.getElementById('blk-' + list[0].id)?.scrollIntoView({ behavior: 'smooth', block: 'center' });
}

function appendItem(item) { appendItems([item]); }

/* 删除的**唯一入口**（检查器的「删除」按钮、垃圾桶、以及「如果」头上的 ✕ 都走它）。
 *
 * ⚠️ 删除必须**按 IF 块成对扩**: 位置式嵌套下, 少一块标记 = 后面整段的归属都变了
 *    （「否则」没了 → 失败分支的积木并进 then 分支; 「结束如果」没了 → 后面剩下的全部
 *    落进分支里）。所以命中任一标记就扩成整块删, 而不是提示一句让用户去别处删 ——
 *    画布上没有「别处」可去。整块删掉时用 toast 报清楚删了几行。
 * 普通积木照旧只删自己。 */
function deleteItem(id) {
  const idx = S.items.findIndex(i => i.id === id);
  if (idx < 0) return;

  const kind = ifMarkerKind(S.items[idx]);
  let a = idx, b = idx;
  if (kind === 'open') {
    [a, b] = ifSpanIdx(S.items, idx);            /* 「如果」→ 连同配对的「结束如果」 */
  } else if (kind === 'else') {
    b = Math.max(idx, elseSpanIdx(S.items, idx)[1]);   /* 「否则」→ 连同它的整段 */
  } else if (kind === 'close') {
    /* 现扫一遍而不是读 S.ifStruct: 那个是上一次 renderCanvas 的快照, 读它会有
     * 「拿陈旧的配对关系去决定删多少」的窗口（最终还是会失衡）。 */
    const st = scanIfStructure(S.items);
    const ifItem = S.items.find((x, i) => i < idx && st.endOfIf.get(x.id) === id);
    if (ifItem) [a, b] = ifSpanIdx(S.items, S.items.indexOf(ifItem));
  }

  const removed = S.items.splice(a, b - a + 1);
  if (S.selectedId && !S.items.some(i => i.id === S.selectedId)) S.selectedId = null;
  markDirty();
  renderCanvas();
  renderInspector();
  toast(removed.length > 1
    ? ('已删除整个「如果」块（' + removed.length + ' 行）')
    : '已删除（保存后生效）');
}

/* ==================== 拖拽（pointer 自绘, 手机桌面式让位） ====================
 * 一份实现, 「地图」与「机械臂」两个画布共用 —— 由 cfg 注入各自的画布元素、条目
 * 数组与重绘/删除回调。这里面有 FLIP 让位、占位槽、6px 阈值、Ctrl 复制、拖出画布
 * 反悔, 都是调出来的手感; 复制第二份必然分叉。
 *
 * cfg = {
 *   innerEl()       → 画布内层容器（.zone 的父节点）
 *   canvasEl()      → 画布外框（判「拖出画布 = 反悔」）
 *   trashEl()       → 垃圾桶（判「拖进去 = 删除」）; 没有就传 null
 *   blockId(id)     → 积木元素的 DOM id
 *   items()         → 条目数组（**活引用**, 直接 splice）
 *   zonesMeta()     → [{el, ids, nextFirstId}]（拖拽落点计算用, 见 renderCanvas）
 *   catalogOf(type) → catalog 条目
 *   createItem(cat) → 新条目对象
 *   render()        → 重绘
 *   select(id)      → 选中
 *   deleteItem(id)  → 删除
 *   beforeChange()  → 可选, 每次真正改数据前调一次（机械臂 Tab 用它压撤回栈）
 *   allowCopy       → 是否允许 Ctrl 拖拽复制（默认 true）
 *   dragSet(id)     → 可选, 拖这一块时要**一起搬走**的 id 列表（默认只有它自己）。
 *                     机械臂 Tab 用它做多选整组拖动; 单块拖动时返回 [id]。
 *   selectMany(ids) → 可选, 一次选中多块（落座后保持整组选中）; 没给就 select(ids[0])
 *   deleteItems(ids)→ 可选, 一次删多块（整组拖进垃圾桶）; 没给就 deleteItem(ids[0])
 * }
 *
 * ⚠️ 元素 id 由调用方给（blockId）: 两个画布的 DOM **同时存在于文档里**, 都用
 *    `blk-<id>` 的话 getElementById 会跨画布取错元素 —— 地图用 blk-*, 机械臂用 blk-arm-*。
 */
const DND_MOVE_THRESHOLD_SQ = 36;   /* 6px: 区分点击与拖拽 */

function makeDnd(cfg) {
  let placeholderEl = null;
  let justDragged = false;          /* 抑制拖拽结束后的 click */
  const D = {
    candidate: null,   /* {mode:'move'|'new', id|type, pointerId, x, y}  未超阈值前 */
    active: null,      /* {mode, id|type, ghost, offsetX, offsetY, w, h, lastBeforeId, lastCopying} */
  };

  function blockEl(id) { return document.getElementById(cfg.blockId(id)); }

  /* FLIP 让位动画: 占位槽换位时其余积木/分区平滑滑开。
   * 同步强制回流提交起点(无 rAF 竞态, 连续移动可无缝重定向);
   * zone 整体位移与块内位移分开计算, 避免父子双重平移。 */
  let flipSeq = 0;
  function _flipAnim(el, dy) {
    if (!el || !el.isConnected || Math.abs(dy) < 2) return;
    const token = ++flipSeq;
    el._flipToken = token;
    el.style.transition = 'none';
    el.style.transform = 'translateY(' + dy + 'px)';
    void el.offsetWidth;                                  /* 强制提交起始位置 */
    el.style.transition = 'transform .24s cubic-bezier(.25,.9,.3,1)';
    el.style.transform = '';
    const done = (ev) => {
      if (ev && ev.propertyName !== 'transform') return;
      if (el._flipToken !== token) return;                /* 已被更新的动画接管, 跳过清理 */
      el.style.transition = '';
      el.style.transform = '';
      el.removeEventListener('transitionend', done);
    };
    el.addEventListener('transitionend', done);
    setTimeout(done, 340);                                /* 兜底清理 */
  }

  function flip(mutate) {
    const root = cfg.innerEl();
    const zones = [...root.querySelectorAll('.zone')];
    const blocks = [...root.querySelectorAll('.block')];
    const zBefore = new Map(zones.map(el => [el, el.getBoundingClientRect().top]));
    const bBefore = new Map(blocks.map(el => [el, el.getBoundingClientRect().top]));
    mutate();
    const zShift = new Map();
    for (const z of zones) {
      const dy = (zBefore.get(z) || 0) - z.getBoundingClientRect().top;
      zShift.set(z, dy);
      _flipAnim(z, dy);
    }
    for (const b of blocks) {
      const dy = (bBefore.get(b) || 0) - b.getBoundingClientRect().top;
      const zs = zShift.get(b.closest('.zone')) || 0;
      _flipAnim(b, dy - zs);                              /* 扣掉分区自身位移 */
    }
  }

  function clearPlaceholder() {
    if (placeholderEl && placeholderEl.parentNode) placeholderEl.parentNode.removeChild(placeholderEl);
    placeholderEl = null;
  }

  function insertPlaceholder(beforeId, copying) {
    clearPlaceholder();
    /* 整组拖动时槽里写清"几块", 否则一个吸收了三块高度的槽看着像 bug。
     * 让位空间 D.active.h 也是按整组高度算的（见 startDrag）。 */
    const n = (D.active && D.active.ids) ? D.active.ids.length : 1;
    const label = (copying ? '⧉ 复制' : '⇅ 移动') + (n > 1 ? (' ' + n + ' 块') : '') +
                  (copying ? '到此处' : '');
    placeholderEl = h('div', {
      class: 'drop-slot' + (copying ? ' copy' : ''), 'data-pos': beforeId == null ? '__end__' : beforeId,
    }, copying || n > 1 ? label : '');
    if (D.active) placeholderEl.style.height = D.active.h + 'px';   /* 让位空间=被拖积木实际高度 */

    /* 积木是 .zone 的子节点, 占位槽须插进同一个 zone 内、目标积木之前 */
    const canvasEl = cfg.innerEl();
    const zones = canvasEl.querySelectorAll('.zone');
    if (beforeId) {
      const ref = blockEl(beforeId);
      const host = (ref && ref.parentNode) || zones[zones.length - 1] || canvasEl;
      host.insertBefore(placeholderEl, ref || null);
      /* 占位槽继承落点行的横向缩进 —— 嵌套 IF 的层级是从**位置**推出来的, 而占位槽
       * 自己不占位置, 不抄这份缩进就看不出它会落进哪个分支。 */
      if (ref) placeholderEl.style.marginLeft = ref.style.marginLeft || '0px';
    } else {
      (zones[zones.length - 1] || canvasEl).appendChild(placeholderEl);   /* 末尾 */
    }
  }

  function setPlaceholder(beforeId, copying) {
    if (D.active && beforeId === D.active.lastBeforeId && copying === D.active.lastCopying) return;
    flip(() => insertPlaceholder(beforeId, copying));
    D.active.lastBeforeId = beforeId;
    D.active.lastCopying = copying;
  }

  /* 一次拖动要跳过的 id 集合 = 被拖的那一组自己（它们不能当落点锚） */
  function dragSkipIds(payload) {
    if (!payload || payload.mode !== 'move') return new Set();
    return new Set(payload.ids && payload.ids.length ? payload.ids : [payload.id]);
  }

  function computeBeforeId(y, skipIds) {
    const zones = cfg.zonesMeta();
    for (let zi = 0; zi < zones.length; zi++) {
      const z = zones[zi];
      const zr = z.el.getBoundingClientRect();
      const isLast = zi === zones.length - 1;
      if (y <= zr.bottom || isLast) {
        for (const id of z.ids) {
          if (skipIds && skipIds.has(id)) continue;   /* 跳过被拖的那一组自身 */
          const el = blockEl(id);
          if (!el) continue;
          const r = el.getBoundingClientRect();
          if (y < r.top + r.height / 2) return id;
        }
        return z.nextFirstId;             // 该区尾 → 下一区首块之前（null=最末）
      }
    }
    return null;
  }

  function isOverTrash(x, y) {
    const t = cfg.trashEl();
    if (!t) return false;
    const r = t.getBoundingClientRect();
    return x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;
  }

  /* 指针是否在画布区域内 → 拖出画布(含拖回左侧积木库)视为取消操作 */
  function isOverCanvas(x, y) {
    const c = cfg.canvasEl().getBoundingClientRect();
    return x >= c.left && x <= c.right && y >= c.top && y <= c.bottom;
  }

  function createGhost(srcEl) {
    const ghost = srcEl.cloneNode(true);
    ghost.removeAttribute('id');
    ghost.className = 'block drag-ghost';
    const r = srcEl.getBoundingClientRect();
    ghost.style.cssText = 'position:fixed;left:' + r.left + 'px;top:' + r.top + 'px;'
      + 'width:' + r.width + 'px;margin:0;pointer-events:none;z-index:1000;';
    document.body.appendChild(ghost);
    return ghost;
  }

  /* ---- 多选整组拖动（机械臂 Tab） ----
   * 单块那套（createGhost / r.height）一个字不动 —— 机械臂 Tab 单拖的手感与「地图」
   * 完全一致; 下面这两个只在**一次拖一块以上**时才走。 */
  function groupHeight(els) {
    let h = 0;
    for (const el of els) h += el.getBoundingClientRect().height;
    if (els.length > 1) {
      /* 块间那段真实间距（按前两块实测; 同一分区里的积木间距一样） */
      const a = els[0].getBoundingClientRect(), b = els[1].getBoundingClientRect();
      h += Math.max(0, b.top - a.bottom) * (els.length - 1);
    }
    return h;
  }

  /* 整组的 ghost = 各块的克隆摞在一个 fixed 容器里, 以**第一块**定位（光标相对它取偏移,
   * 于是"抓住哪就往哪走"的直觉与单块拖动一致）。 */
  function createGroupGhost(els) {
    const wrap = h('div', { class: 'block drag-ghost drag-group' });
    const r = els[0].getBoundingClientRect();
    for (const el of els) {
      const c = el.cloneNode(true);
      c.removeAttribute('id');            /* 文档里不能同时存在两份同名 id */
      wrap.appendChild(c);
    }
    wrap.style.cssText = 'position:fixed;left:' + r.left + 'px;top:' + r.top + 'px;'
      + 'width:' + r.width + 'px;pointer-events:none;z-index:1000;';
    document.body.appendChild(wrap);
    return wrap;
  }

  /* 从积木库拖出来的 ghost: 内容由调用方给（cfg.newGhost(type, cat)），
   * 没给就用「标签 + 类型」的通用样子。 */
  function newGhost(type) {
    const cat = cfg.catalogOf(type) || {};
    if (cfg.newGhost) return cfg.newGhost(type, cat);
    return h('div', { class: 'block drag-ghost drag-new' },
      h('div', { class: 'b-head' },
        h('span', { class: 'b-tag' }, ((cat.desc || type).split('(')[0].trim().slice(0, 16))),
        h('span', { class: 'b-type' }, shortType(type)),
      ));
  }

  function startDrag(e) {
    const c = D.candidate;
    D.candidate = null;
    const srcEl = c.mode === 'move' ? blockEl(c.id) : null;
    if (c.mode === 'move' && !srcEl) return;

    /* 注意: 不要用 h 作局部变量名——会遮蔽全局 DOM 构造助手 h() */
    let ghost, dragH, dragW, offsetX, offsetY, beforeId;
    if (c.mode === 'move') {
      /* 拖这一块要一起搬走哪些？（cfg.dragSet 没给 = 就它自己, 与抽取前一致） */
      const ids = (cfg.dragSet ? cfg.dragSet(c.id) : null) || [c.id];
      const els = ids.map(blockEl).filter(Boolean);
      if (!els.length) return;
      const r = els[0].getBoundingClientRect();
      ghost = (els.length > 1) ? createGroupGhost(els) : createGhost(srcEl);
      dragH = (els.length > 1) ? groupHeight(els) : r.height;
      dragW = r.width;
      offsetX = e.clientX - r.left; offsetY = e.clientY - r.top;
      beforeId = computeBeforeId(e.clientY, new Set(ids));
      D.active = { mode: 'move', id: c.id, ids: ids, ghost, offsetX, offsetY, w: dragW, h: dragH,
                   lastBeforeId: beforeId, lastCopying: false,
                   overTrash: false, overCanvas: true };
      /* 整组一起 add('drag-lift')（display:none 抽离文档流）—— 占位槽的高度也就等于
       * 它们本来占的地方, 其余积木的让位动画因此与单块拖动走的是同一条 FLIP 路径。 */
      flip(() => {
        for (const el of els) el.classList.add('drag-lift');
        insertPlaceholder(beforeId, false);
      });
    } else {
      ghost = newGhost(c.type);
      ghost.style.cssText = 'position:fixed;left:' + (e.clientX - 120) + 'px;top:' + (e.clientY - 22) + 'px;'
        + 'width:240px;margin:0;pointer-events:none;z-index:1000;';
      document.body.appendChild(ghost);
      dragH = 46; dragW = 240; offsetX = 120; offsetY = 22;
      beforeId = computeBeforeId(e.clientY, null);
      D.active = { mode: 'new', id: null, type: c.type, ghost, offsetX, offsetY, w: dragW, h: dragH,
                   lastBeforeId: beforeId, lastCopying: false,
                   overTrash: false, overCanvas: true };
      flip(() => insertPlaceholder(beforeId, false));
    }
  }

  function moveDrag(e) {
    const d = D.active;
    d.ghost.style.left = (e.clientX - d.offsetX) + 'px';
    d.ghost.style.top = (e.clientY - d.offsetY) + 'px';

    d.overTrash = isOverTrash(e.clientX, e.clientY);
    const trash = cfg.trashEl();
    if (trash) trash.classList.toggle('hot', d.overTrash);
    if (d.overTrash) { clearPlaceholder(); d.lastBeforeId = null; d.overCanvas = false; return; }

    d.overCanvas = isOverCanvas(e.clientX, e.clientY);
    if (!d.overCanvas) {                       /* 拖出画布: 不收占位槽, 松手即取消 */
      flip(() => clearPlaceholder());
      d.lastBeforeId = null;
      return;
    }

    const copying = cfg.allowCopy !== false && (e.ctrlKey || e.metaKey);
    const beforeId = computeBeforeId(e.clientY, dragSkipIds(d));
    setPlaceholder(beforeId, copying);
  }

  function endDrag(e) {
    const d = D.active;
    D.active = null;
    const trash = cfg.trashEl();
    if (trash) trash.classList.remove('hot');
    justDragged = true;
    setTimeout(() => { justDragged = false; }, 0);

    /* 落座: 移除 ghost, 交给 applyDrop/deleteItem 的 render 重建 DOM
     * (重建会同时清掉占位槽与源积木的 drag-lift, 画面直接呈现最终顺序)。 */
    d.ghost.remove();

    if (d.overTrash) {
      if (d.mode === 'move') {
        cfg.beforeChange && cfg.beforeChange();
        const ids = d.ids && d.ids.length ? d.ids : [d.id];
        if (ids.length > 1 && cfg.deleteItems) cfg.deleteItems(ids);
        else cfg.deleteItem(d.id);
      } else cfg.render();                              /* 清残留占位槽 */
      return;
    }

    if (!d.overCanvas) {                                /* 拖出画布 = 反悔: 不改动数据 */
      cfg.render();                                     /* 复原源积木与占位槽 */
      if (d.mode === 'new') toast('已取消添加');
      return;
    }

    const copying = cfg.allowCopy !== false && (e.ctrlKey || e.metaKey);
    const beforeId = computeBeforeId(e.clientY, dragSkipIds(d));
    if (d.mode === 'move') applyDrop({ mode: 'move', id: d.id, ids: d.ids }, beforeId, copying);
    else applyDrop({ mode: 'new', type: d.type }, beforeId, false);
  }

  function onPointerMove(e) {
    if (D.active) { e.preventDefault(); moveDrag(e); return; }
    if (D.candidate && e.pointerId === D.candidate.pointerId) {
      const dx = e.clientX - D.candidate.x, dy = e.clientY - D.candidate.y;
      if (dx * dx + dy * dy > DND_MOVE_THRESHOLD_SQ) startDrag(e);
    }
  }

  function onPointerUp(e) {
    if (D.candidate) { D.candidate = null; return; }      /* 未超阈值, 交给 click */
    if (D.active) endDrag(e);
  }

  function applyDrop(payload, beforeId, duplicate) {
    const items = cfg.items();
    if (payload.mode === 'new') {
      const cat = cfg.catalogOf(payload.type);
      if (!cat) return;
      cfg.beforeChange && cfg.beforeChange();
      /* 一次可以落多行: 「如果」必须同时落 如果/否则/结束如果, 否则立刻就是失衡状态。
       * cfg.createItems 是可选的（机械臂画布不定义它 → 走原来的单条路径）。 */
      const made = cfg.createItems ? cfg.createItems(cat) : [cfg.createItem(cat)];
      if (!made.length) return;
      const pos = beforeId ? items.findIndex(i => i.id === beforeId) : items.length;
      items.splice(pos < 0 ? items.length : pos, 0, ...made);
      cfg.render(); cfg.select(made[0].id);
      return;
    }
    if (payload.mode !== 'move') return;

    /* 被拖的可能是一组（cfg.dragSet）—— 整组按原来的相对顺序落座。 */
    const ids = (payload.ids && payload.ids.length) ? payload.ids : [payload.id];
    const moving = items.filter(i => ids.includes(i.id));
    if (!moving.length) return;
    cfg.beforeChange && cfg.beforeChange();

    let put;
    if (duplicate) {                       // Ctrl 拖拽 = 原地留一份, 插入克隆
      put = moving.map(src => ({ ...src, id: cfg.newId(),
               params: Array.isArray(src.params) ? src.params.slice() : src.params }));
    } else {
      /* 落点锚在被拖的这组**里面** = 原地不动（computeBeforeId 本来就跳过整组, 但跨分区的
       * nextFirstId 仍可能指到组内那块 —— 这条兜底别删） */
      if (beforeId && ids.includes(beforeId)) return;
      const gone = new Set(ids);
      for (let i = items.length - 1; i >= 0; i--) {
        if (gone.has(items[i].id)) items.splice(i, 1);
      }
      put = moving;
    }
    let pos = beforeId ? items.findIndex(i => i.id === beforeId) : items.length;
    if (pos < 0) pos = items.length;
    items.splice(pos, 0, ...put);
    cfg.render();
    /* 整组落座后保持整组选中 —— 接着还能一次右键搬去别的动作 */
    const selIds = put.map(i => i.id);
    if (cfg.selectMany) cfg.selectMany(selIds); else cfg.select(selIds[0]);
  }

  document.addEventListener('pointermove', onPointerMove);
  document.addEventListener('pointerup', onPointerUp);
  document.addEventListener('pointercancel', onPointerUp);
  /* 拖到浏览器外松手（部分平台没有 pointercancel）: 窗口失焦时按「反悔」收场 ——
   * 伪造画布外坐标走 endDrag 的取消分支（复原源积木/清占位槽, 不动数据）。 */
  window.addEventListener('blur', () => {
    if (!D.active) { D.candidate = null; return; }
    endDrag({ clientX: -9999, clientY: -9999, ctrlKey: false, metaKey: false });
  });

  return {
    /* 登记候选（未超阈值不启动）—— 点击 vs 拖拽靠移动距离区分 */
    begin(payload, e) {
      D.candidate = { ...payload, pointerId: e.pointerId, x: e.clientX, y: e.clientY };
    },
    justDragged() { return justDragged; },
  };
}

/* 「地图」画布的拖拽实例（行为与抽取前完全一致, 只是现在走 cfg 注入）。 */
const MDND = makeDnd({
  innerEl: () => document.getElementById('canvas-inner'),
  canvasEl: () => document.getElementById('canvas'),
  trashEl: () => document.getElementById('trash'),
  blockId: (id) => 'blk-' + id,
  items: () => S.items,
  zonesMeta: () => S.zonesMeta,
  catalogOf: (type) => S.catByType[type],
  createItem: newItemFromCatalog,
  createItems: createItemsFromCatalog,      /* 只在地图这边给: 拖「如果」要一次落三行 */
  dragSet: ifSpanIds,                       /* 拖「如果」= 拖整块（不是「如果」就返回 null） */
  newId,
  render: renderCanvas,
  select: selectItem,
  deleteItem,
  beforeChange: markDirty,
});

/* ==================== 积木元数据编辑器（右键积木/库条目） ====================
 * 保存 → POST /api/meta → 服务端写回 app_script.h 的注释 → 重新派生 catalog
 * 补齐缺失的说明/中文名/单位/选项后, 下拉菜单与标签下次读取即自动带上。 */

let metaCtx = null;

function parseOptions(text) {
  const t = (text || '').trim();
  if (!t) return null;
  const opts = [];
  for (const seg of t.split(/[,，]/)) {
    const s = seg.trim();
    if (!s) continue;
    const i = s.indexOf('=');
    opts.push(i < 0 ? { value: s, label: s }
                    : { value: s.slice(0, i).trim(), label: s.slice(i + 1).trim() });
  }
  return opts.length ? opts : null;
}

function openMetaEditor(cat, it) {
  const metas = paramMeta(cat);
  const nF = (cat.params_f || []).length;
  const body = document.getElementById('modal-body');
  body.textContent = '';
  document.getElementById('modal-title').textContent = '编辑积木 · ' + cat.type;

  body.appendChild(h('p', { class: 'modal-hint' },
    '保存后写回 Application/app_script.h（枚举尾注释 + 参数映射注释），' +
    '控制台下次读取自动带上。留空的字段表示"未填写"。'));

  const descInp = h('input', { value: cat.desc || '' });
  body.appendChild(h('label', { class: 'fld' }, '积木说明（写回枚举尾注释）', descInp));

  const noteInp = h('input', { value: cat.note || cat.hint || '' });
  body.appendChild(h('label', { class: 'fld' }, '参数备注（写回行尾 // ）', noteInp));

  const rows = [];
  if (metas.length) {
    body.appendChild(h('div', { class: 'modal-sect' }, '参数（顺序 = 表内顺序，前 ' + nF + ' 个为 float）'));
    metas.forEach((m, i) => {
      const labelInp = h('input', { value: m.label || '' });
      const unitInp = h('input', { class: 'narrow', value: m.unit || '' });
      const optInp = h('input', {
        value: (m.options || []).map(o => o.value + '=' + o.label).join(', '),
        placeholder: '留空 = 自动（枚举 / 文本输入）',
      });
      rows.push({ m, labelInp, unitInp, optInp });
      body.appendChild(h('div', { class: 'meta-row' },
        h('div', { class: 'meta-name' }, m.name + (i < nF ? ' · float' : ' · int')
          + (m.show_if ? ' · 显隐:' + m.show_if.name + '=' + m.show_if.value : '')
          + (m.enable_if ? ' · 启用:' + m.enable_if.name + '=' + m.enable_if.value : '')),
        h('label', {}, h('span', {}, '中文名'), labelInp),
        h('label', {}, h('span', {}, '单位'), unitInp),
        h('label', { class: 'wide' }, h('span', {}, '选项（值=标签，逗号分隔）'), optInp),
      ));
    });
  } else {
    body.appendChild(h('p', { class: 'modal-hint' }, '该积木无参数'));
  }

  metaCtx = { cat, nF, rows, descInp, noteInp };
  document.getElementById('modal').classList.remove('hidden');
  descInp.focus();
}

function closeMetaEditor() {
  document.getElementById('modal').classList.add('hidden');
  metaCtx = null;
}

let _savingMeta = false;
async function saveMetaEditor() {
  if (!metaCtx) return;
  if (_savingMeta) return;    /* 防重入（同 save） */
  _savingMeta = true;
  const { cat, nF, rows, descInp, noteInp } = metaCtx;
  const params = rows.map(r => ({
    name: r.m.name, raw: r.m.raw || r.m.name,
    label: r.labelInp.value.trim(), unit: r.unitInp.value.trim(),
    options: parseOptions(r.optInp.value),
    /* ⚠️ 必须原样透传: 本接口是**整行重写** app_script.h 的参数注释, tip/enum_type 那些
       服务端能重新推导, 而显隐/启用条件与控件标记推不出来 —— 漏了就是「改一次中文名,
       <显隐>/<启用>/<控件:move_var> 无声消失, 专用控件退化成普通输入框」。
       <可空> 同理: 漏了它, 「放置动作」下拉里的 0=无 就没了。 */
    show_if: r.m.show_if || null,
    enable_if: r.m.enable_if || null,
    widget: r.m.widget || null,
    allow_none: r.m.allow_none || null,
  }));
  try {
    const resp = await fetch('/api/meta', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      /* base_version = /api/rev 的 script_h 戳: 两客户端并发改元数据时后写收 409,
       * 不再静默覆盖先写（与「地图」保存同款乐观锁, 2026-10-09） */
      body: JSON.stringify({ step: cat.type, desc: descInp.value.trim(),
                             note: noteInp.value.trim(), params, n_f: nF,
                             base_version: (REV.last && REV.last.script_h) || null }),
    });
    if (!resp.ok) {
      const j = await resp.json().catch(() => ({}));
      toast('写回失败: ' + (j.message || j.error || ('HTTP ' + resp.status)), 'err');
      return;
    }
    closeMetaEditor();
    await reloadCatalog();          // 只刷 catalog, 不动 map 的未保存编辑
    toast('已写回 app_script.h（备份 .h.bak）');
  } catch (e) {
    toast('写回失败: ' + e.message, 'err');
  } finally {
    _savingMeta = false;
  }
}

/* 只重新派生积木库(保留画布上未保存的 map 编辑) */
async function reloadCatalog() {
  const cat = await fetch('/api/catalog').then(r => r.json());
  S.catalog = cat;
  S.catByType = {};
  for (const c of cat) S.catByType[c.type] = c;
  renderPalette();
  renderCanvas();
  renderInspector();
}

/* ==================== 保存 / 重载 ==================== */

let _savingMap = false;   /* 写盘防重入的旗子（各保存函数一个, 见 save 内注释） */

async function save() {
  /* 编译期风险检查: 任何一张预设全被禁用 → 该数组成为空初始化列表, C 编译失败 */
  const empties = S.presets.filter(p =>
    !p.items.some(i => i.kind === 'step' && !i.disabled)).map(p => p.label);
  if (empties.length && !confirm(
      '这些预设里没有任何启用的步骤：\n  ' + empties.join('、') +
      '\n\nC 不允许空初始化列表，保存后 map.c 会编译失败。\n仍要保存吗？')) {
    return;
  }
  if (_savingMap) return;   /* 防重入: 双击/连点会发两个带同一 base_version 的 POST,
                             * 第二个必吃 409 —— 弹出误导性的「已在外部被修改」 */
  _savingMap = true;
  try {
    /* ⚠️ 先把 params 的**稀疏洞**填成空串再发: 老行参数数组比 catalog 短时, 直接改
       尾槽（it.params[9]=v 跳过了 i=8）会留下洞, JSON.stringify 把洞序列化成 null ——
       服务端 _normalize_param 崩（'NoneType' object has no attribute 'strip', 保存 500,
       2026-10 支点转弯加 settle_early 后真踩过）。Array.from 会把洞当 undefined 访问,
       正好一次填平; 短于 catalog 的尾巴不用管（服务端自己补 0）。 */
    for (const p of S.presets) for (const it of p.items) {
      if (it.kind === 'step' && Array.isArray(it.params)) {
        it.params = Array.from(it.params, (x) => (x === undefined || x === null) ? '' : x);
      }
    }
    const r = await fetch('/api/map', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        base_version: S.version, active: S.runIdent, presets: S.presets,
        reentrant_ids: S.reentrantIds || [],
      }),
    });
    if (r.status === 409) { toast('map.c 已在外部被修改，请先「重载」再保存', 'err'); return; }
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      toast('保存失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err');
      return;
    }
    const j = await r.json();
    S.version = j.version;
    S.runIdent = j.active;
    if (j.next_id) S.nextId = j.next_id;      /* 服务端重算过的 id 分配水位 */
    clearDirty();
    renderPresetBar();
    /* ⚠️ 也要重拉 catalog: 保存会重生成 map.h 的 emScriptPresetIdTdf, 而那是「IF / 跳转」
     * 目标下拉的选项来源（服务端已失效缓存, 但浏览器这份 S.catalog 是页面加载时拉的）。
     * 不刷的话: 新建预设 → 保存 → 去选目标, 下拉里**没有**这个新预设, 直到 Ctrl+F5 ——
     * 现象正好是「这功能没做」。reloadCatalog 不动 S.presets, 未保存的编辑不受影响。 */
    /* reloadCatalog 单独兜底: 保存**已经成功** —— 它挂了不该被上面的 catch
     * 吃成「保存失败」的假话 */
    try { await reloadCatalog(); } catch (e2) { toast('刷新积木库失败: ' + e2.message, 'err'); }
    toast('已保存 → Application/map.c（备份 .bak）');
    /* 保存成功但服务端有提醒（如用了机械臂动作步骤却没 include 那个头）——
     * 那类问题只会在 Keil 里以 `identifier XXX is undefined` 出现, 在这儿说清更省事 */
    if (j.warning) toast('⚠ ' + j.warning, 'err');
  } catch (err) {
    toast('保存失败: ' + err.message, 'err');
  } finally {
    _savingMap = false;
  }
}

async function reloadFromDisk() {
  if (S.dirty && !confirm('有未保存修改，重载将丢弃，继续？')) return;
  await loadAll();
  toast('已从磁盘重载');
}

/* ==================== Tab: 调试助手 ====================
 * 移植自 pro/调试助手/调试助手.py。服务端逻辑在 tools/webconsole/host_tools.py,
 * 这里只负责填 UI、发请求、写剪贴板。常用语存 tools/webconsole/phrases.json。 */

const TOOLS = {
  loaded: false,        /* 首次切入本 Tab 时才拉取 */
  phrases: [],
  piIp: '',
  sel: null,            /* 常用语选中下标 */
};
let pEditIdx = -1;      /* 常用语编辑对话框: -1=新增, >=0=改第几条 */

const ACTION_LABEL = {
  update: '更新 SSH 和网络位置',
  rdp: '远程桌面连接',
  folder: '打开远程文件夹',
  trae: '启动远程 Trae',
  vscode: '启动远程 VSCode',
  console: '启动树莓派控制台',
};

/* 日志区按行追加, 失败行标红 */
function tlog(msg, kind) {
  const el = document.getElementById('tools-log');
  const span = document.createElement('span');
  if (kind) span.className = kind;
  span.textContent = msg;
  el.appendChild(span);
  el.scrollTop = el.scrollHeight;
}

/* 写剪贴板: 优先 Clipboard API; 权限被拒/老浏览器退回 execCommand
 * (textarea 需真实存在才能 select)。
 * ⚠️ Clipboard API 要求**安全上下文** —— 本机走 127.0.0.1 时算, 队友走局域网
 * `http://192.168.x.x:8410` 时**不算**（`navigator.clipboard` 直接是 undefined,
 * 取 .writeText 就抛 TypeError）。所以局域网客户端的常态就是下面那条兜底路径,
 * **别把兜底删了**, 也别在别处直接碰 navigator.clipboard。 */
async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);
    return true;
  } catch (e) {
    const ta = h('textarea', { class: 'copy-tmp' });
    ta.value = text;
    document.body.appendChild(ta);
    ta.select();
    let ok = false;
    try { ok = document.execCommand('copy'); } catch (e2) { ok = false; }
    ta.remove();
    return ok;
  }
}

async function copyCard(id) {
  const v = document.getElementById(id).dataset.val;
  if (!v) return;
  const ok = await copyText(v);
  toast(ok ? '已复制 ' + v : '复制失败（浏览器未授权剪贴板）', ok ? '' : 'err');
}

/* ---- 加载 ---- */

async function loadTools() {
  try {
    const j = await fetch('/api/tools').then(r => r.json());
    TOOLS.loaded = true;
    TOOLS.piIp = j.pi_ip || '';
    TOOLS.phrases = j.phrases || [];
    setNet('net-local', j.local_ip);
    setNet('net-console', j.console_url);       /* 发给队友的那个地址（服务端绑 0.0.0.0） */
    setNet('net-pi', j.pi_ip);
    setNet('net-ssh', j.ssh);
    setNet('net-share', j.share);
    renderPhrases();
  } catch (e) {
    toast('读取调试助手信息失败: ' + e.message, 'err');
  }
}

function setNet(id, val) {
  const el = document.getElementById(id);
  el.querySelector('b').textContent = val || '—';
  el.dataset.val = val || '';
}

/* ---- 一键操作 ---- */

async function runTool(action, dryRun) {
  const btns = [...document.querySelectorAll('#pane-tools .btn.tool')];
  btns.forEach(b => { b.disabled = true; });
  tlog('> ' + (ACTION_LABEL[action] || action) + (dryRun ? '（仅预览）' : '') + '\n');
  try {
    const r = await fetch('/api/tools/action', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action, dry_run: !!dryRun }),
    });
    const j = await r.json().catch(() => ({}));
    (j.log || []).forEach(line => tlog(line, /✗|失败|错误/.test(line) ? 'bad' : ''));
    if (!r.ok) tlog('请求失败: ' + (j.error || ('HTTP ' + r.status)) + '\n', 'bad');
    else if (!j.ok) tlog('动作未成功完成（详见上文）\n', 'bad');
  } catch (e) {
    tlog('请求失败: ' + e.message + '\n', 'bad');
  } finally {
    btns.forEach(b => { b.disabled = false; });
  }
}

/* ---- 常用语 ---- */

function renderPhrases() {
  const list = document.getElementById('phrase-list');
  list.textContent = '';
  if (!TOOLS.phrases.length) {
    list.appendChild(h('p', { class: 'hint' }, '还没有常用语，点下面「＋ 新增」'));
    return;
  }
  TOOLS.phrases.forEach((p, i) => {
    const preview = p.dynamic ? '（动态：扫共享取最新 logcat 日志）' : (p.text || '');
    /* 只绑单击: 双击会先触发两次 click, 动态常用语就要扫两遍远程共享(每遍可能好几秒)。
     * 编辑走下面的「✎ 编辑」按钮(依赖单击选中)。 */
    list.appendChild(h('div', {
      class: 'phrase' + (TOOLS.sel === i ? ' sel' : ''),
      title: '单击复制；改内容点下面「✎ 编辑」',
      onclick: () => { TOOLS.sel = i; renderPhrases(); copyPhrase(p, i); },
    },
      h('b', {}, p.name || '(未命名)'),
      h('span', {}, preview.length > 46 ? preview.slice(0, 43) + '...' : preview),
    ));
  });
}

/* 动态常用语要扫远程共享取最新日志 → 必须问服务端; 静态的也走同一条路,
 * 免得 {pi_ip} 替换逻辑在前后端各写一份。 */
async function copyPhrase(p, idx) {
  const item = document.querySelectorAll('#phrase-list .phrase')[idx];
  if (item) item.classList.add('busy');
  tlog('[解析] ' + (p.name || '') + (p.dynamic ? '（扫远程共享, 可能几秒）' : '') + '…\n');

  let text = p.text || '';
  try {
    const r = await fetch('/api/tools/action', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'phrase', phrase: p }),
    });
    const j = await r.json();
    (j.log || []).forEach(line => tlog(line));
    if (j.text) text = j.text;
    else text = text.replace(/\{pi_ip\}/g, TOOLS.piIp);     /* 服务端没给就本地兜底 */
  } catch (e) {
    tlog('解析失败: ' + e.message + '（复制未解析原文）\n', 'bad');
    text = text.replace(/\{pi_ip\}/g, TOOLS.piIp);
  } finally {
    if (item) item.classList.remove('busy');
  }

  const ok = await copyText(text);
  toast(ok ? '已复制：' + (p.name || '') : '复制失败（浏览器未授权剪贴板）', ok ? '' : 'err');
}

function openPhraseEditor(idx) {
  pEditIdx = (idx === undefined || idx === null) ? -1 : idx;
  const p = pEditIdx >= 0 ? TOOLS.phrases[pEditIdx] : { name: '', text: '', dynamic: false };
  document.getElementById('pmodal-title').textContent = pEditIdx >= 0 ? '编辑常用语' : '新增常用语';
  document.getElementById('pm-name').value = p.name || '';
  document.getElementById('pm-text').value = p.text || '';
  document.getElementById('pm-dynamic').checked = !!p.dynamic;
  document.getElementById('pmodal').classList.remove('hidden');
  document.getElementById('pm-name').focus();
}

function closePhraseEditor() {
  document.getElementById('pmodal').classList.add('hidden');
  pEditIdx = -1;
}

async function savePhraseEditor() {
  const name = document.getElementById('pm-name').value.trim();
  if (!name) { toast('请先填名称', 'err'); return; }
  const entry = { name, text: document.getElementById('pm-text').value.trim() };
  if (document.getElementById('pm-dynamic').checked) entry.dynamic = 'latest_log';

  const next = TOOLS.phrases.slice();
  if (pEditIdx >= 0) next[pEditIdx] = entry; else next.push(entry);
  if (await savePhrases(next)) closePhraseEditor();
}

async function delPhrase() {
  const p = TOOLS.phrases[TOOLS.sel];
  if (!p) { toast('先选中一条常用语', 'err'); return; }
  if (!confirm('删除常用语「' + p.name + '」？')) return;
  const next = TOOLS.phrases.filter((_, i) => i !== TOOLS.sel);
  TOOLS.sel = null;
  await savePhrases(next);
}

async function savePhrases(list) {
  try {
    const r = await fetch('/api/tools/phrases', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ phrases: list }),
    });
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      toast('保存失败: ' + (j.error || ('HTTP ' + r.status)), 'err');
      return false;
    }
    TOOLS.phrases = list;
    renderPhrases();
    toast('常用语已保存 → phrases.json');
    return true;
  } catch (e) {
    toast('保存失败: ' + e.message, 'err');
    return false;
  }
}

/* ==================== Tab: 调参 ====================
 * 数据源是 Application/app_config.h 的注释 DSL（见 tools/webconsole/tuning.py 顶部）。
 * 服务端负责换算与写盘；这里只管展示、校验和即时反馈。
 * tuning.json 只存「显示哪些 + 什么顺序」；显示名/单位/范围/备注都在头文件注释里。 */

const TUNE = {
  loaded: false,
  version: 0,
  groups: [],          // [{title, items:[…]}]
  changed: {},         // {宏名: 界面填的值}  保存时提交
  byName: {},          // 宏名 → item 快照（用于算已改）
  sortMode: false,
};

/* 与服务端 CONVERSIONS 对应的两个公式（闭集, 就这两个）。
 * 放在前端只是为了打字时能立刻显示「= 20u」, 真正写盘仍以服务端换算为准。 */
const CONV_FNS = {
  'ms->Hz': { toDisplay: v => 1000 / v, toStored: v => 1000 / v },
  '‰->%':   { toDisplay: v => v / 10,   toStored: v => v * 10 },
};

function convToDisplay(conv, v) {
  const f = CONV_FNS[conv];
  if (!f || !v) return v;
  return f.toDisplay(v);
}
function convToStored(conv, v) {
  const f = CONV_FNS[conv];
  if (!f || !v) return null;                  // 0 会被除零, 交给服务端也要拒
  return f.toStored(v);
}

/* ---------- 加载 / 保存 ---------- */

async function loadTuning() {
  try {
    const j = await fetch('/api/tuning').then(r => r.json());
    TUNE.loaded = true;
    TUNE.version = j.version;
    TUNE.groups = j.groups || [];
    TUNE.byName = {};
    for (const g of TUNE.groups) for (const it of g.items) TUNE.byName[it.name] = it;
    TUNE.changed = {};
    clearTuneDirty();
    renderTuning();
  } catch (e) {
    toast('读取 app_config.h 失败: ' + e.message, 'err');
  }
}

function tuneDirty() { TUNE.dirty = true; document.getElementById('tuning-dot').classList.remove('hidden'); }
function clearTuneDirty() { TUNE.dirty = false; document.getElementById('tuning-dot').classList.add('hidden'); }

let _savingTuning = false;
async function saveTuning() {
  if (_savingTuning) return;    /* 防重入（同 save） */
  _savingTuning = true;
  try {
    const typed = Object.assign({}, TUNE.changed);      // 留一份, 被拒的参数要把它留在框里
    const r = await fetch('/api/tuning', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ base_version: TUNE.version, values: TUNE.changed }),
    });
    if (r.status === 409) { toast('app_config.h 已在外部被修改，请先「重载」再保存', 'err'); return; }
    const j = await r.json();
    if (!r.ok) { toast('保存失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return; }

    /* ⚠️ warnings 是 [{name, msg, saved}]。saved=false 表示这个参数**没写进去**,
     * 必须把它填的值留在输入框里并明说原因 —— 否则界面重新加载后看起来就是
     * 「我的改动被默默复原了」（这正是之前那个 bug 的表象）。 */
    const warns = j.warnings || [];
    const rejected = warns.filter(w => !w.saved).map(w => w.name);

    await loadTuning();
    for (const n of rejected) {
      if (typed[n] !== undefined) TUNE.changed[n] = typed[n];
    }
    if (rejected.length) { tuneDirty(); renderTuning(); }

    if (warns.length) {
      /* 合成一条: 之前逐条 toast, 后一条会立刻顶掉前一条, 结果只看得见摘要 */
      toast('⚠ ' + warns.map(w => w.name + ' ' + w.msg).join('；'), 'err');
    } else {
      toast('已保存 → Application/app_config.h（备份 .bak）');
    }
  } catch (e) {
    toast('保存失败: ' + e.message, 'err');
  } finally {
    _savingTuning = false;
  }
}

/* ---------- 渲染 ---------- */

function renderTuning() {
  const root = document.getElementById('tuning-canvas');
  root.textContent = '';
  let shown = 0, total = 0;

  for (const g of TUNE.groups) {
    const items = g.items.filter(it => it.visible);
    total += g.items.length;
    shown += items.length;
    if (!items.length) continue;

    const card = h('section', { class: 'tune-group', 'data-group': g.title },
      h('div', { class: 'tune-group-head' },
        h('span', { class: 'tune-group-title' }, g.title),
        h('span', { class: 'tune-group-n' }, items.length + ' 项'),
      ));
    for (const it of items) card.appendChild(renderTuneItem(g.title, it));
    root.appendChild(card);
  }
  if (!shown) root.appendChild(h('p', { class: 'hint' }, '没有勾选任何参数，点左上「☰ 参数选择」挑几个。'));
  document.getElementById('tune-count').textContent = shown + ' / ' + total + ' 个参数';
}

function renderTuneItem(groupTitle, it) {
  const changed = Object.prototype.hasOwnProperty.call(TUNE.changed, it.name);
  const row = h('div', {
    class: 'tune-item' + (changed ? ' changed' : ''),
    'data-name': it.name, 'data-group': groupTitle,
  });

  /* 显示名 + **宏名**（`app_config.h` 里那个 `#define` 的名字, 只读）。
   *
   * ⚠️ 宏名单独一个 `.tn-macro`（`flex: none`）而**不是**并进显示名那格: 显示名那格是
   *    可省略的（`ellipsis`, 长名字要让位）, 而宏名正是拿来"回源码里找它"的 —— 被
   *    ellipsis 吃掉就白写了。
   * ⚠️ 双击改名**只改显示名**: `editInPlace` 的初值就是 `it.display`, 而且它把整个
   *    `.tune-name`（含这个宏名）替成输入框、提交后 `renderTuning` 再画回来 ——
   *    所以改名时输入框里只有显示名, 宏名不可能被顺手改掉（宏名本来就改不得: 改它等于
   *    重命名 C 宏, 那是「新建参数」/手工改源码的事, 不是改注释）。
   * ⚠️ 源码里没写显示名时 `tuning.py` 把 `display` 兜底成宏名 —— 那种情况**不要**再写一遍
   *    （`理想停止位中球列 BALL_CX_CENTER` 是想要的, `BALL_CX_CENTER BALL_CX_CENTER` 不是）。 */
  const dupesMacro = (it.display === it.name);
  const nameEl = h('div', {
    class: 'tune-name',
    title: '双击改显示名' + (it.note ? '\n备注：' + it.note : '')
         + (dupesMacro ? '' : '\n宏名 ' + it.name + '（app_config.h 里的名字, 改它请直接改源码）'),
    ondblclick: (e) => { e.stopPropagation(); renameParam(it); },
  },
    h('span', { class: 'tn-disp' }, it.display),
    dupesMacro ? null : h('code', { class: 'tn-macro', title: 'app_config.h 里的宏名' }, it.name),
  );
  if (it.note) nameEl.appendChild(h('span', { class: 'tune-note-dot', title: it.note }, '•'));

  row.appendChild(nameEl);
  row.appendChild(tuneInput(it));
  return row;
}

/* 值控件: 数值型给输入框 + 单位后缀 + 换算预览; raw(符号/表达式) 给文本框 */
function tuneInput(it) {
  const val = Object.prototype.hasOwnProperty.call(TUNE.changed, it.name)
    ? TUNE.changed[it.name] : it.display_value;
  const box = h('div', { class: 'tune-input' });

  if (it.kind === 'raw') {
    box.appendChild(h('input', {
      class: 'tune-val', value: val, title: '符号/表达式, 原样写回',
      oninput: (e) => setTuneValue(it, e.target.value, box),
    }));
    return box;
  }

  /* 下拉选项: 宏备注里写了「选项:0=xx 1=yy」的参数（tuning.py 派生 it.options,
   * ARM_SERVO_CURVE 首用）。本质仍是数值写盘（select 给的就是 "0"/"1" 这类字符串,
   * 与手敲等价, 走同一套批量保存）; 当前值按**数字**比对来选中 —— 当前值不在选项里
   * （手改过头/枚举外加的值）时补一个"(枚举外)"项把它如实显示出来, 不悄悄换值。 */
  if (it.options && it.options.length) {
    const cur = Number(val);
    const sel = h('select', { class: 'tune-val',
      onchange: (e) => setTuneValue(it, e.target.value, box) });
    let matched = false;
    for (const o of it.options) {
      const opt = h('option', { value: o.value }, o.label);
      if (Number(o.value) === cur) { opt.selected = true; matched = true; }
      sel.appendChild(opt);
    }
    if (!matched) sel.appendChild(h('option', { value: val, selected: 'true' }, val + ' (枚举外)'));
    box.appendChild(sel);
    return box;
  }

  /* 布尔开关: 整型且范围恰为 0~1（kind 只有 uint/int/float/raw 四种, 故浮点不算）。
   * 与「蓝牙」Tab 的 btParamRow 同一套判据, 但这里**不即时写盘** —— 只标脏,
   * 等「保存」按钮一起提交（调参 Tab 的保存是批量的, 见 saveTuning）。
   * ⚠️ checked **绝不能**进 h() 的属性对象: h() 走 setAttribute, `checked:false`
   *    会写成 checked="false", 而布尔属性只要出现就算已勾选 —— 开关会显示成"开"。
   * ⚠️ class 仍给 'tune-val': setTuneValue / applyWarn 都按它找控件。 */
  if ((it.kind === 'int' || it.kind === 'uint')
      && String((it.range || {}).lo) === '0' && String((it.range || {}).hi) === '1') {
    const chk = h('input', { class: 'tune-val', type: 'checkbox' });
    chk.checked = (Number(val) === 1);
    chk.addEventListener('change', () => setTuneValue(it, chk.checked ? '1' : '0', box));
    box.appendChild(chk);
    box.appendChild(h('span', { class: 'tune-unit' }, '0/1'));
    return box;
  }

  /* 空值哨兵参数（备注 DSL「留空=255 默认取FILL_TASK_PCT」, tuning.py 派生 it.empty;
   * 哨兵态下 display_value 就是 ""）: 输入框**留空** + placeholder 提示「默认」,
   * 真填 0~100 才是字面值 —— 「留空=跟随默认」与「填 0=不亮」就这样区分开。
   * placeholder 尽量带出默认现值（按 empty_from 指的宏从 TUNE.byName 现查）, 查不到只写「默认」。
   * ⚠️ h() 对 undefined 属性直接跳过, 非哨兵参数自然不带 placeholder。 */
  let ph, ti;
  if (it.empty != null) {
    const src = it.empty_from && TUNE.byName[it.empty_from];
    ph = '默认' + (src ? ('(' + src.display_value + (src.unit || '') + ')') : '');
    ti = '留空 = 用默认（文件里存哨兵值 ' + it.empty + '）; 填 0~100 是字面值（0 = 不亮）';
  } else {
    ti = it.conv ? ('底层存 ' + it.value + '，这里按 ' + it.conv + ' 填显示值') : it.name;
  }
  const inp = h('input', {
    class: 'tune-val', value: val, placeholder: ph, title: ti,
    oninput: (e) => setTuneValue(it, e.target.value, box),
  });
  box.appendChild(inp);
  const tail = h('span', { class: 'tune-unit' }, it.unit || '');
  box.appendChild(tail);
  const hint = h('span', { class: 'tune-conv' });
  box.appendChild(hint);
  applyWarn(it, val, inp, hint);
  return box;
}

function applyWarn(it, shown, inp, hint) {
  const r = it.range || {};
  let bad = null;
  const v = parseFloat(shown);
  if (!isNaN(v)) {
    if (r.lo !== '' && r.lo !== undefined && v < parseFloat(r.lo)) bad = '低于下限 ' + r.lo;
    if (r.hi !== '' && r.hi !== undefined && v > parseFloat(r.hi)) bad = '高于上限 ' + r.hi;
  }
  inp.classList.toggle('warn', !!bad);
  inp.title = bad ? ('⚠ ' + bad + '（范围 ' + r.lo + '~' + r.hi + '，仅提示不拦保存）') : '';
  if (hint) hint.textContent = it.conv && !isNaN(v) ? convPreview(it.conv, v) : '';
}

function convPreview(conv, v) {
  const stored = convToStored(conv, v);
  if (stored === null) return '⚠ 不能为 0';
  const num = Math.abs(stored - Math.round(stored)) < 1e-9 ? Math.round(stored) : stored.toFixed(2);
  return '= ' + num + ({ 'ms->Hz': 'ms', '‰->%': '‰' }[conv] || '');
}

function setTuneValue(it, value, box) {
  const orig = String(it.display_value);
  if (value === orig) delete TUNE.changed[it.name];
  else TUNE.changed[it.name] = value;

  const row = box.closest('.tune-item');
  if (row) row.classList.toggle('changed', Object.prototype.hasOwnProperty.call(TUNE.changed, it.name));
  applyWarn(it, value, box.querySelector('.tune-val'), box.querySelector('.tune-conv'));
  if (Object.keys(TUNE.changed).length) tuneDirty(); else clearTuneDirty();
}

/* ---------- 双击改名 / 右键改元数据 ---------- */

function renameParam(it) {
  const el = document.querySelector('.tune-item[data-name="' + it.name + '"] .tune-name');
  if (!el) return;
  editInPlace(el, it.display, async (v) => {
    if (!v || v === it.display) { renderTuning(); return; }
    await postMeta(it, Object.assign({}, metaOf(it), { display: v }));
    await reloadTuningKeepEdits();
  });
}

function metaOf(it) {
  return { display: it.display, unit: it.unit || '', conv: it.conv || '',
           range: { lo: (it.range || {}).lo || '', hi: (it.range || {}).hi || '' },
           note: it.note || '' };
}

async function postMeta(it, meta) {
  const r = await fetch('/api/tuning/meta', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    /* base_version = /api/rev 的 tuning 戳, 同「地图」的乐观锁（2026-10-09） */
    body: JSON.stringify({ name: it.name, meta,
                           base_version: (REV.last && REV.last.tuning) || null }),
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { toast('写回注释失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return false; }
  toast('已写回 app_config.h（备份 .bak）');
  return true;
}

/* 改注释后重读, 但保住还没保存的值编辑 */
async function reloadTuningKeepEdits() {
  const keep = Object.assign({}, TUNE.changed);
  const dirty = TUNE.dirty;
  await loadTuning();
  TUNE.changed = keep;
  if (dirty) tuneDirty();
  renderTuning();
}

/* ---------- 参数选择 ---------- */

function openPicker() {
  const body = h('div', {});
  body.appendChild(h('p', { class: 'modal-hint' },
    '勾选要显示的参数（存在 tools/webconsole/tuning.json，下次打开保持不变）。' +
    '以后往 app_config.h 新增的数值参数会自动出现在这里并默认勾上。'));

  for (const g of TUNE.groups) {
    const head = h('label', { class: 'pick-group' },
      h('input', { type: 'checkbox', 'data-grp': g.title }),
      h('b', {}, g.title),
      h('span', { class: 'pick-n' }, ' ' + g.items.length + ' 个'));
    body.appendChild(head);
    const list = h('div', { class: 'pick-list' });
    for (const it of g.items) {
      list.appendChild(h('label', { class: 'pick-item', title: it.name },
        h('input', { type: 'checkbox', 'data-name': it.name,
                     checked: it.visible ? '' : null }),
        h('span', {}, it.display),
        h('code', {}, it.name)));
    }
    body.appendChild(list);

    const boxes = [head.querySelector('input')]
      .concat([...list.querySelectorAll('input')]);
    const sync = () => {
      const all = boxes.slice(1).every(b => b.checked);
      const some = boxes.slice(1).some(b => b.checked);
      boxes[0].checked = all;
      boxes[0].indeterminate = some && !all;
    };
    boxes[0].addEventListener('change', () => {
      boxes.slice(1).forEach(b => { b.checked = boxes[0].checked; });
    });
    boxes.slice(1).forEach(b => b.addEventListener('change', sync));
    sync();
  }

  openTuningModal('参数选择', body, async () => {
    const visible = {};
    for (const g of TUNE.groups) {
      for (const it of g.items) {
        const box = body.querySelector('input[data-name="' + it.name + '"]');
        visible[it.name] = box ? box.checked : it.visible;
      }
    }
    return savePrefs({ visible });
  }, '保存选择');
}

async function savePrefs(patch) {
  const payload = Object.assign({ visible: currentVisibleMap(), order: currentOrderMap() }, patch);
  const r = await fetch('/api/tuning/prefs', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { toast('保存偏好失败: ' + (j.error || ('HTTP ' + r.status)), 'err'); return false; }
  await loadTuning();
  return true;
}

function currentVisibleMap() {
  const m = {};
  for (const g of TUNE.groups) for (const it of g.items) m[it.name] = it.visible;
  return m;
}

function currentOrderMap() {
  const m = {};
  for (const g of TUNE.groups) m[g.title] = g.items.filter(i => i.visible).map(i => i.name);
  return m;
}

/* ---------- 新建参数 ---------- */

function openNewParam() {
  const body = h('div', {});
  const fName = h('input', { placeholder: '如 ARM_GROUP_WAIT_TIMEOUT_MS' });
  const fValue = h('input', { value: '0', placeholder: '如 3000 / 45.0 / 500' });
  const fDisplay = h('input', { placeholder: '如 组动作等待超时' });
  const fUnit = h('input', { class: 'narrow', placeholder: 'ms' });
  const fConv = convSelect();
  const fLo = h('input', { class: 'narrow', placeholder: '下限' });
  const fHi = h('input', { class: 'narrow', placeholder: '上限' });
  const fNote = h('input', { placeholder: '这个参数是干什么的' });

  const sel = h('select', {});
  for (const g of TUNE.groups) sel.appendChild(h('option', { value: g.title }, g.title));
  sel.appendChild(h('option', { value: '__new__' }, '（新建分区…）'));
  const fSection = h('input', { placeholder: '新分区名', class: 'hidden' });
  sel.addEventListener('change', () => {
    fSection.classList.toggle('hidden', sel.value !== '__new__');
  });

  body.appendChild(h('p', { class: 'modal-hint' },
    '写回 Application/app_config.h。值只管填数字，u/f 后缀按值的写法自动补；' +
    '带小数写 45.0 这样即可。'));
  body.appendChild(h('label', { class: 'fld' }, '宏名（C 标识符，全大写）', fName));
  body.appendChild(h('label', { class: 'fld' }, '初始值', fValue));
  body.appendChild(h('label', { class: 'fld' }, '显示名（写进注释）', fDisplay));
  body.appendChild(h('div', { class: 'trow' },
    h('label', {}, h('span', {}, '单位'), fUnit),
    h('label', {}, h('span', {}, '换算'), fConv),
    h('label', {}, h('span', {}, '下限'), fLo),
    h('label', {}, h('span', {}, '上限'), fHi)));
  body.appendChild(h('label', { class: 'fld' }, '备注', fNote));
  body.appendChild(h('label', { class: 'fld' }, '放进哪个分区', sel));
  body.appendChild(fSection);

  openTuningModal('新建参数', body, async () => {
    const name = fName.value.trim();
    if (!/^[A-Za-z_]\w*$/.test(name)) { toast('宏名不合法', 'err'); return false; }
    if (TUNE.byName[name]) { toast('已存在同名宏', 'err'); return false; }
    const isNew = sel.value === '__new__';
    if (isNew && !fSection.value.trim()) { toast('填个新分区名', 'err'); return false; }
    const r = await fetch('/api/tuning', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        base_version: TUNE.version, values: TUNE.changed,
        new_params: [{
          name, value: fValue.value.trim() || '0',
          meta: { display: fDisplay.value.trim() || name, unit: fUnit.value.trim(),
                  conv: fConv.value, range: { lo: fLo.value.trim(), hi: fHi.value.trim() },
                  note: fNote.value.trim() },
          target: isNew ? null : sel.value,
          new_section: isNew ? fSection.value.trim() : null,
        }],
      }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { toast('新建失败: ' + (j.error || ('HTTP ' + r.status)), 'err'); return false; }
    await loadTuning();
    const warns = j.warnings || [];
    toast(warns.length
      ? '⚠ ' + warns.map(w => w.name + ' ' + w.msg).join('；')
      : '已写入 app_config.h（备份 .bak），记得重新编译',
      warns.length ? 'err' : '');
    return true;
  }, '创建');
}

function convSelect(value) {
  const sel = h('select', {});
  sel.appendChild(h('option', { value: '' }, '（无）'));
  for (const c of Object.keys(CONV_FNS)) {
    sel.appendChild(h('option', { value: c, selected: c === value ? '' : null }, c));
  }
  return sel;
}

/* ---------- 元数据编辑（右键） ---------- */

function openMeta(it) {
  const m = metaOf(it);
  const body = h('div', {});
  body.appendChild(h('p', { class: 'modal-hint' },
    '写回 app_config.h 里 ' + it.name + ' 那一行的尾注释。值那一段原样保留，diff 里只看得到注释在变。'));

  const fDisplay = h('input', { value: m.display });
  const fUnit = h('input', { class: 'narrow', value: m.unit });
  const fConv = convSelect(m.conv);
  const fLo = h('input', { class: 'narrow', value: m.range.lo, placeholder: '下限' });
  const fHi = h('input', { class: 'narrow', value: m.range.hi, placeholder: '上限' });
  const fNote = h('input', { value: m.note });

  body.appendChild(h('label', { class: 'fld' }, '显示名', fDisplay));
  body.appendChild(h('div', { class: 'trow' },
    h('label', {}, h('span', {}, '单位'), fUnit),
    h('label', {}, h('span', {}, '换算'), fConv),
    h('label', {}, h('span', {}, '下限'), fLo),
    h('label', {}, h('span', {}, '上限'), fHi)));
  body.appendChild(h('label', { class: 'fld' }, '备注', fNote));
  body.appendChild(h('div', { class: 'kv' },
    h('span', {}, '底层值'), h('code', {}, it.value)));

  openTuningModal('编辑参数 · ' + it.name, body, async () => {
    const ok = await postMeta(it, {
      display: fDisplay.value.trim() || it.name, unit: fUnit.value.trim(),
      conv: fConv.value, range: { lo: fLo.value.trim(), hi: fHi.value.trim() },
      note: fNote.value.trim(),
    });
    if (ok) await reloadTuningKeepEdits();
    return ok;
  }, '保存并写回 app_config.h');
}

/* ---------- 通用对话框 ---------- */

let tmodalSave = null;

function openTuningModal(title, bodyEl, onSave, saveLabel) {
  document.getElementById('tmodal-title').textContent = title;
  const body = document.getElementById('tmodal-body');
  body.textContent = '';
  body.appendChild(bodyEl);
  document.getElementById('tmodal-save').textContent = saveLabel || '保存';
  tmodalSave = onSave;
  document.getElementById('tmodal').classList.remove('hidden');
}

function closeTuningModal() {
  document.getElementById('tmodal').classList.add('hidden');
  tmodalSave = null;
}

async function runTuningModalSave() {
  if (!tmodalSave) return;
  const btn = document.getElementById('tmodal-save');
  btn.disabled = true;
  try {
    if (await tmodalSave()) closeTuningModal();
  } catch (e) {
    toast('失败: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
  }
}

/* ---------- 拖动排序（只在本分区内, 不影响 app_config.h 的行顺序） ---------- */

let dragEl = null;

function onTuneDragStart(e) {
  if (!TUNE.sortMode) return;
  const row = e.target.closest('.tune-item');
  if (!row) return;
  dragEl = row;
  row.classList.add('dragging');
  e.preventDefault();
}

function onTuneDragMove(e) {
  if (!dragEl) return;
  const over = document.elementFromPoint(e.clientX, e.clientY);
  const row = over && over.closest('.tune-item');
  if (!row || row === dragEl) return;
  if (row.dataset.group !== dragEl.dataset.group) return;   /* 只在本分区内换位 */
  const r = row.getBoundingClientRect();
  const after = e.clientY > r.top + r.height / 2;
  row.parentNode.insertBefore(dragEl, after ? row.nextSibling : row);
}

async function onTuneDragEnd() {
  if (!dragEl) return;
  dragEl.classList.remove('dragging');
  dragEl = null;
  /* 把当前 DOM 顺序写进 tuning.json（只是本页展示顺序） */
  const order = currentOrderMap();
  for (const card of document.querySelectorAll('.tune-group')) {
    order[card.dataset.group] =
      [...card.querySelectorAll('.tune-item')].map(el => el.dataset.name);
  }
  await savePrefs({ order });
  toast('顺序已记住（不影响 app_config.h）');
}

/* ==================== Tab: 蓝牙 ==================== */
/* 对接固件 app_bt.c 的 $B 行协议。三条设计约束:
 *  1. 「Beam 运行时参数」面板**完全由设备上报**（$B,LIST）渲染 —— 参数名/中文名/
 *     分组/单位/类型/范围/当前值全部来自固件 app_beam.c 的注册表, 这里不写死任何
 *     默认值, 所以宏改了/初值改了, 面板自动跟着变。
 *  2. 机械臂编辑模式只有在「脚本停在握手等待阶段1」时才被设备接受; 一旦脚本恢复
 *     运行, 设备就回 NOTEDIT —— 我们据此**立刻退出编辑态并停轮询**, 不做任何假设。
 *  3. 编辑模式里按 5Hz 轮询 $B,POS 回读真实角度; 离开本 Tab 必须停掉定时器
 *     （本项目别处没有任何轮询, 这是第一处, 不收拾会一直打已断开的串口）。
 */

const BT = {
  loaded: false,
  available: true,
  connected: false,
  ports: [],
  selected: '',
  params: [],
  joints: [],
  actions: [],            // [{name, angles:[5], gripper:{enabled,kai_he,degree}}]
  edit: false,
  grip: { kai_he: 1, degree: 100 },
  posTimer: null,
  autoTimer: null,
  touched: {},            // 关节 i 最近一次本地拖动时刻（回读时别抢用户正在拖的滑块）
  jogTimer: {},           // 拖动节流
  code: '',
  logLines: [],           // 共享日志环（两个连接面板各写一份, 内容一致）
  dragging: false,        // 正在拖关节滑杆 → 暂停 $B,POS 轮询与 $B,LIST 自动刷新
  playing: false,         // 正在整段播放 → 同上（那时设备也不会应答）
  posFails: 0,            // $B,POS 连续失败次数（见 btPollPos: 单拍超时不判掉线）
};

let btLogT = 0;           // 日志水位线: 只把服务端新产生的行追加进来

/* 连接面板实例表（「蓝牙」与「机械臂」各挂一个）。
 * 面板只是**视图** —— 连接状态与日志都在这份共享的 BT 状态里, 所以两个 Tab
 * 天然看到同一个链路（同一时刻只可能存在一条蓝牙连接）。 */
const CONN_PANELS = [];
const BT_LOG_MAX = 400;
/* $B,POS 连续失败几次才判「链路掉了」并退出编辑模式。见 btPollPos —— 单拍超时是
 * 蓝牙偶发抖动, 不该把人从台架的编辑模式里踢出去。 */
const BT_POS_FAILS_TO_DROP = 2;

/* 日志: 一条内存环 + 每个已挂载面板各写一份。
 * 内容一致（数据共通）, 位置各自独立（不破坏任一 Tab 的版面）。 */
function btlog(msg, kind) {
  BT.logLines.push({ text: msg, kind: kind || '' });
  if (BT.logLines.length > BT_LOG_MAX) {
    BT.logLines.splice(0, BT.logLines.length - BT_LOG_MAX);
  }
  for (const p of CONN_PANELS) appendLogLine(p, msg, kind);
}

function appendLogLine(p, msg, kind) {
  const el = p.logEl;
  if (!el) return;
  const span = document.createElement('span');
  if (kind) span.className = kind;
  span.textContent = msg;
  el.appendChild(span);
  while (el.childNodes.length > BT_LOG_MAX) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

function btLogClear() {
  BT.logLines = [];
  for (const p of CONN_PANELS) if (p.logEl) p.logEl.textContent = '';
}

async function btCall(action, extra) {
  const res = await fetch('/api/bt', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ action: action }, extra || {})),
  });
  if (!res.ok) throw new Error('HTTP ' + res.status);
  return res.json();
}

/* 统一动作包装: 打日志、弹 toast、把 data 交回调用方; 失败回 null。
 * quiet = 不弹 toast; silent = 连日志都不打（轮询用, 否则日志会被刷爆）。 */
async function btRun(action, extra, opts) {
  opts = opts || {};
  try {
    const j = await btCall(action, extra);
    if (!opts.silent) {
      for (const l of (j.log || [])) btlog(l.text, l.kind);
    }
    if (!j.ok) {
      if (!opts.silent && j.error) btlog('✗ ' + j.error, 'bad');
      if (!opts.quiet && !opts.silent) toast(j.error || '操作失败', 'err');
      /* onError: 给「静默失败也要换个说法报出来」的调用方用（见 btEnterEdit）——
       * 那些路径把 toast 文案交给调用方, 免得这里先弹一条、调用方又弹一条。 */
      if (opts.onError) opts.onError(j.error || '操作失败');
      return null;
    }
    return j.data === undefined ? {} : j.data;
  } catch (e) {
    if (!opts.silent) btlog('✗ ' + e.message, 'bad');
    if (!opts.quiet && !opts.silent) toast('请求失败: ' + e.message, 'err');
    if (opts.onError) opts.onError(e.message);
    return null;
  }
}

async function btSyncState() {
  const res = await fetch('/api/bt');
  const st = await res.json();
  BT.available = !!st.available;
  BT.connected = !!st.connected;
  /* 连接状态一变就清「$B,POS 连续失败」计数: 它只为「同一条链路上的偶发抖动」累计,
   * 跨连接累计会把重连后的第一次超时也算成第二次（见 btPollPos）。 */
  BT.posFails = 0;
  BT.ports = st.ports || [];
  BT.selected = st.selected || '';
  for (const p of CONN_PANELS) { renderPorts(st, p); renderConn(st, p); }
  btReplayLog(st.log);
  return st;
}

async function loadBt() {
  BT.loaded = true;
  try { await btSyncState(); }
  catch (e) { toast('读取蓝牙状态失败: ' + e.message, 'err'); }
  renderBtParams();
  renderJoints();
}

/* 把服务端环形日志里「还没显示过」的行追加进来（水位线 btLogT） */
function btReplayLog(entries) {
  let maxT = btLogT;
  for (const e of (entries || [])) {
    if (e.t <= btLogT) continue;
    if (e.t > maxT) maxT = e.t;
    const prefix = e.dir === 'tx' ? '→ ' : (e.dir === 'sys' ? '' : '← ');
    btlog(prefix + e.text, e.dir === 'sys' ? 'bad' : (e.dir === 'tx' ? 'tx' : ''));
  }
  btLogT = maxT;
}

/* ==================== 共用连接面板 ====================
 * 「蓝牙」与「机械臂」两个 Tab 都要连接同一台设备, 于是把这块抽成一个工厂:
 * 面板只是视图, 状态全在上面那个 BT 对象里 —— 同一时刻只可能有一条链路。
 *
 * 为什么不保留静态 DOM + 给 id 加前缀: 原来那 5 个函数全写死了 getElementById,
 * 挂两份就得维护两份静态 DOM 且处处拼前缀; 由工厂用 h() 现搭 DOM 只要一个空挂载点。
 *
 * onConnected: 连上之后该 Tab 自己要补的活（蓝牙 Tab 是读参数表, 机械臂 Tab 还要
 * 顺带刷关节滑杆）。抽出来是为了不让工厂知道任何一个 Tab 的业务。
 */
function makeConnPanel(opt) {
  /* opt.extra: 「机械臂」Tab 传进来的那一小块静态 DOM（进/退编辑模式），
   * 会被搬进下面那行按钮里。⚠️ **必须在 `mount.textContent = ''` 之前抓住引用** ——
   * 它本来就摆在 #arm-conn-mount 里当占位（注释见 index.html）, 那行会把它摘下来,
   * 之后只剩这个引用还指着它, 再 appendChild 就换到按钮行里去了。 */
  const extra = opt.extra || null;
  const led = h('span', { class: 'led off' });
  const note = h('span', { class: 'hint' });
  const port = h('select', { class: 'conn-port', title: '本车的蓝牙串口（或任意 USB-TTL 口）' });
  const baud = h('select', { title: '波特率（固件固定 115200）' });
  const refreshBtn = h('button', { class: 'btn ghost', onclick: () => linkRefreshPorts() }, '↻ 刷新');
  const connectBtn = h('button', { class: 'btn primary', onclick: () => linkConnect(panel) }, '连接');
  const disconnectBtn = h('button', { class: 'btn ghost', onclick: () => linkDisconnect() }, '断开');

  const row = h('div', { class: 'conn-row' }, refreshBtn, connectBtn, disconnectBtn);
  if (extra) row.appendChild(extra);

  const root = h('div', { class: 'conn-panel' },
    h('div', { class: 'conn-row' }, led, h('b', {}, '连接'), note),
    h('div', { class: 'conn-row' },
      h('span', { class: 'conn-lab' }, '串口'), port,
      h('span', { class: 'conn-lab' }, '波特率'), baud),
    row,
  );

  opt.mount.textContent = '';
  opt.mount.appendChild(root);

  const panel = { id: opt.id, els: { led, note, port, baud, connect: connectBtn, disconnect: disconnectBtn },
                  logEl: opt.logEl || null, onConnected: opt.onConnected || null };
  CONN_PANELS.push(panel);
  return panel;
}

function renderPorts(st, panel) {
  const sel = panel.els.port;
  const baud = panel.els.baud;
  sel.textContent = '';

  /* 先定选中项，再建选项 —— 这样「请选择」占位才能放在第一个。
   * 自动选中只在一种前提下做：**这台机器上只有一个蓝牙设备**。
   * 一旦有 ≥2 个（本车模块 + 耳机/手环…），就绝不猜：这里 4 个口分属 2 台设备，
   * 唯一可用的传出口恰恰是耳机的 —— 猜"能用的那个"会去连耳机。 */
  const want = (st && st.selected) || BT.selected;
  let pick = BT.ports.some(p => p.port === want) ? want : '';
  if (!pick && BT.ports.length) {
    const bound = BT.ports.filter(p => p.is_bt && !p.generic);
    const devs = new Set(bound.map(p => p.device || p.port));
    if (devs.size === 1) {
      /* 只有一台蓝牙设备（常见情形）→ 选它的传出口，没有就用它唯一的那个口 */
      const out = bound.find(p => p.direction === 'out');
      pick = (out || bound[0]).port;
    } else if (devs.size === 0) {
      /* 没有蓝牙设备（纯 USB 串口环境）→ 只有一个串口时直接选它 */
      const plain = BT.ports.filter(p => !p.is_bt);
      if (plain.length === 1) pick = plain[0].port;
    }
    /* devs.size ≥ 2 → 不猜，留占位；用户选一次后记进 bt.json，下次自动选中 */
  }

  if (!BT.ports.length) {
    sel.appendChild(h('option', { value: '' }, '（没有可用串口）'));
  } else if (!pick) {
    sel.appendChild(h('option', { value: '' }, '— 请选择本车的蓝牙模块 —'));
  }

  for (const p of BT.ports) {
    /* 蓝牙项写清三件事：属于哪台设备、是传出还是传入、链路是否已通。
     * 机器上常有多个蓝牙串口（耳机/手环/本车模块各一个），不标出来就只能靠猜。 */
    let tag = '';
    if (p.is_bt) {
      tag = ' · 蓝牙' + (p.device ? ' ' + p.device : '');
      if (p.direction === 'out') {
        tag += ' 传出' + (p.ready === false ? '(未连接)' : '');
      } else if (p.generic) {
        tag += ' 通用传入(勿选)';
      } else {
        tag += ' 传入(打不开)';
      }
    }
    sel.appendChild(h('option', { value: p.port },
      p.port + (p.name ? ' · ' + p.name : '') + tag));
  }
  sel.value = pick;                        /* pick 为空即停在占位项上 */

  baud.textContent = '';
  for (const b of ((st && st.baud_choices) || [115200])) {
    baud.appendChild(h('option', { value: b }, b));
  }
  if (st && st.baud) baud.value = String(st.baud);
  if (baud.value === '') baud.value = '115200';
}

function renderConn(st, panel) {
  const port = (st && st.port) || '';
  panel.els.led.className = 'led ' + (BT.connected ? 'on' : 'off');
  if (!BT.available) {
    panel.els.note.textContent = '当前平台不支持串口（仅 Windows）';
  } else if (BT.connected) {
    panel.els.note.textContent = '已连接 ' + (port || '') + (BT.edit ? ' · 机械臂编辑模式' : '');
  } else {
    panel.els.note.textContent = BT.ports.length ? '' : '未发现串口（蓝牙配对后点「刷新」）';
  }
  panel.els.connect.disabled = BT.connected;
  panel.els.disconnect.disabled = !BT.connected;
}

async function linkConnect(panel) {
  const port = panel.els.port.value;
  const baud = parseInt(panel.els.baud.value, 10) || 115200;
  if (!port) { toast('先选一个串口', 'err'); return; }
  const data = await btRun('connect', { port: port, baud: baud });
  if (data === null) return;
  btLogT = 0;                       /* 服务端日志已重置 */
  btLogClear();
  await btSyncState();
  if (panel.onConnected) await panel.onConnected();
}

async function linkDisconnect() {
  await btRun('disconnect', {});
  btSetEdit(false);
  BT.params = []; BT.joints = [];
  await btSyncState();
  renderBtParams(); renderJoints();
}

async function linkRefreshPorts() {
  const data = await btRun('refresh', {});
  if (data === null) return;
  await btSyncState();
  btlog('刷新到 ' + BT.ports.length + ' 个串口');
}

/* ---- Beam 参数 ---- */

async function btReadParams(silent) {
  const data = await btRun('list', {}, { quiet: !!silent, silent: !!silent });
  if (data === null) return;
  BT.params = data.params || [];
  BT.joints = data.joints || [];
  renderBtParams();
  renderJointsKeepValues();
}

function renderBtParams() {
  const box = document.getElementById('bt-params');
  /* 自动刷新 / 写入回读都会重建这张表 —— 若用户正光标在某个输入框里打字,
   * 重建会把他刚敲的内容清掉。正在编辑就跳过这次重绘（下次刷新再补上）。 */
  const act = document.activeElement;
  /* ⚠️ SELECT 也要skip: 下拉框正被拉开时重建 DOM 会把选项列表抽走（用户点不中）。
   *    两类控件的语义一样 —— "正在交互就先不重绘, 下次刷新再补"。 */
  if (act && box.contains(act) && (act.tagName === 'INPUT' || act.tagName === 'SELECT')) return;

  box.textContent = '';
  if (!BT.params.length) {
    box.appendChild(h('p', { class: 'hint' }, BT.connected
      ? '还没有读取。点右上角「读取参数」。'
      : '未连接。选好串口 → 连接 → 点「读取参数」。'));
    return;
  }
  const order = [];
  const groups = {};
  for (const p of BT.params) {
    if (!groups[p.group]) { groups[p.group] = []; order.push(p.group); }
    groups[p.group].push(p);
  }
  for (const g of order) {
    /* 组头带条目数 —— 与「调参」Tab 的 .tune-group-head 同形（标题 + 右侧「N 项」） */
    const sec = h('div', { class: 'bt-group' });
    sec.appendChild(h('div', { class: 'bt-group-head' },
      h('span', {}, g),
      h('span', { class: 'bt-group-n' }, groups[g].length + ' 项')));
    for (const p of groups[g]) sec.appendChild(btParamRow(p));
    box.appendChild(sec);
  }
}

/* 布尔量判据: 设备自述里**没有 bool 类型**（协议只有 F32/I32/U8/I8）, 布尔就是一个
 * 取值域恰为 0~1 的 U8（见 app_beam.h 的 stBeamParamTdf 说明）。只能靠「类型 + 值域」认。
 * ⚠️ 判据放宽（比如只看 U8）会把别的 0~1 参数也变成勾选框。 */
function isBoolParam(p) {
  return p.type === 'U8' && Number(p.min) === 0 && Number(p.max) === 1;
}

function btParamRow(p) {
  const row = h('div', { class: 'bt-row' + (p.rw ? '' : ' ro') });
  row.appendChild(h('span', {
    class: 'bt-name',
    title: p.name + (p.warn ? ' · 运行中修改会影响控制律' : (p.rw ? '' : ' · 只读')),
  }, p.label + (p.warn ? ' ⚠' : '')));

  if (p.options && p.options.length) {
    /* 下拉选项（固件 pcOptions = "0=主口那颗|1=副口那颗"）—— 优先级高于勾选框:
     * 同样值域 0~1 的参数, 填了选项就说明该讲"名字"而不是"0/1"。
     * 可写 → 选完即写; 只读 → 置灰但仍显示当前选项名（比裸 "0"/"1" 可读得多）。
     * ⚠️ 当前值不在选项里（固件改了选项表 / 页面是旧的）→ 补一个"(枚举外)"项如实显示,
     *    不悄悄换值; 比对要过 optKey（写盘会把整数补 u, 而选项 value 是裸值）。 */
    const sel = h('select', { class: 'bt-sel', title: p.name });
    for (const o of p.options) {
      const opt = h('option', { value: o.value }, o.label);
      if (optKey(o.value) === optKey(p.value)) { opt.selected = true; }
      sel.appendChild(opt);
    }
    if (!p.options.some((o) => optKey(o.value) === optKey(p.value))) {
      sel.appendChild(h('option', { value: p.value }, p.value + ' (枚举外)'));
      sel.value = p.value;
    }
    if (p.rw) {
      sel.addEventListener('change', () => btWriteParam(p, sel));
    } else {
      sel.disabled = true;
    }
    row.appendChild(sel);
    return row;
  }

  if (p.rw && isBoolParam(p)) {
    /* 开关: 勾/去勾即写（没有「写入」按钮 —— 勾选框本身就是即时提交的语义）。
     * ⚠️ **不能**把 checked 塞进 h() 的属性对象: h() 走 setAttribute, 于是 `checked:false`
     *    会写成 `checked="false"` —— 布尔属性**只要出现就算已勾选**, 结果是"关"的开关显示成"开"。
     *    必须建完元素后直接赋属性（下面两行）。
     * ⚠️ value 要跟着一起维护: btWriteParam 读的是 inp.value, 而勾选框默认值是 'on'。
     * ⚠️ **只挂 change, 不挂 blur**: btWriteParam 里那段「置 p.value = inp.value 再回读」
     *    是为文本框的 blur 竞争写的; 勾选框再挂 blur 会把同一个值重复 SET 一遍。 */
    const inp = h('input', { class: 'bt-chk', type: 'checkbox' });
    inp.value = p.value;
    inp.checked = (p.value !== '0');
    inp.addEventListener('change', () => {
      inp.value = inp.checked ? '1' : '0';
      btWriteParam(p, inp);
    });
    row.appendChild(inp);
    row.appendChild(h('span', { class: 'bt-range' }, '0 / 1'));
  } else if (p.rw) {
    const inp = h('input', { class: 'bt-val', type: 'text', value: p.value });
    inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') btWriteParam(p, inp); });
    inp.addEventListener('blur', () => { if (inp.value !== p.value) btWriteParam(p, inp); });
    row.appendChild(inp);
    if (p.unit) row.appendChild(h('span', { class: 'bt-unit' }, p.unit));
    row.appendChild(h('span', { class: 'bt-range' }, p.min + '~' + p.max));
    row.appendChild(h('button', {
      class: 'btn ghost tiny', onclick: () => btWriteParam(p, inp),
    }, '写入'));
  } else {
    row.appendChild(h('b', { class: 'bt-roval' }, p.value));
    if (p.unit) row.appendChild(h('span', { class: 'bt-unit' }, p.unit));
  }
  return row;
}

async function btWriteParam(p, inp) {
  const data = await btRun('set', { name: p.name, value: inp.value });
  if (data === null) return;
  /* ⚠️ 先把读回来的对象上的值更新掉, 再回读整表。理由:
   * 回车写入后输入框**仍持焦点**, 而 renderBtParams 见输入框有焦点会**跳过重绘**
   * （见那边注释: 免得清掉用户正在敲的内容）→ p.value 还是旧值, 于是随后的 blur
   * 判 `inp.value !== p.value` 成立, 把同一个值又 SET 一遍（多一次蓝牙往返）。
   * 置成写入值后 blur 那一下就是空操作; 设备真钳位/换算过的话, 下面的回读会覆盖它。 */
  p.value = inp.value;
  /* 写完整表回读: 以设备为准（可能被范围钳位拒绝, 也可能被固件换算过） */
  await btReadParams(true);
}

/* ---- 机械臂编辑 ---- */

/* 关节中文名复用「地图」Tab 从 app_script.h 参数 DSL 派生的标签（s1=舵机1 …）,
 * 同一份关节标签只留一个来源, 不在前端再抄一遍。 */
function btJointLabels() {
  try {
    return paramMeta(catalogOf('STEP_ARM_GROUP_WAIT')).map(m => m.label || m.name);
  } catch (e) {
    return [];
  }
}

/* 关节滑杆在「蓝牙」与「机械臂」两个 Tab 里各有一份。
 * ⚠️ 一律操作**当前可见窗格**里那份: 原来写的是全局
 *    querySelectorAll('#bt-joints …'), 挂上第二份后会一次抓到 10 个滑块,
 *    于是回读/记录位姿全都张冠李戴。 */
function jointBox() {
  const pane = document.querySelector('.pane.active') || document;
  return pane.querySelector('.bt-joints-box');
}

function jointRanges() {
  const box = jointBox();
  return box ? box.querySelectorAll('input[type=range][data-joint]') : [];
}

/* ---- 选中与「关节」面板的关系 ----
 * 面板有**两种模式**, 由「有没有选中一个位姿积木」决定:
 *   选中了位姿块 → 滑杆显示**那个块存的角度**, 拖动就是改积木里的数据（离线也能改）;
 *   没选中       → 滑杆显示**实时设备值**（拖动就是让机械臂动, 录新位姿用）;
 *                  没连设备时退化成灰色离线占位。
 * 这条规则让「调一条已有动作」和「录一条新动作」共用同一组滑杆, 不用来回切模式。 */

/* 选中的**一块**。多选（Ctrl+点击累加）时返回 null —— 见 armMultiSel：
 * 一组积木没有一个「它存的角度」可谈, 滑杆必须整体退化成灰色。 */
function armSelectedItem() {
  if (ARM.selIds.length !== 1) return null;
  return ARM.items.find(x => x.id === ARM.selectedId) || null;
}

/* 多选了几个？（Ctrl+点击累加出来的那一组） */
function armMultiSel() { return ARM.selIds.length > 1; }

/* 滑杆的角度目标: 选中的**位姿块**（不是位姿块就没有角度可改） */
function armPoseTarget() {
  const it = armSelectedItem();
  return (it && it.kind === 'pose') ? it : null;
}

/* 夹爪行的目标: 位姿块自带的夹爪 / 独立的夹爪块 / 都没有（那就用 BT.grip 当「下一个位姿」的） */
function armGripTarget() {
  const it = armSelectedItem();
  if (!it) return null;
  if (it.kind === 'pose') return { obj: it.gripper, owner: it, isPose: true };
  if (it.kind === 'grip') return { obj: it, owner: it, isPose: false };
  return null;
}

function armGripSummary(obj) {
  return '夹爪' + (obj.kai_he ? '开' : '合') + (obj.degree || 0) + '%';
}

/* 「改过没有」的显示: 左边框黄标（.armmod）+ 积木上那个 ↺ 按钮的显隐。
 *
 * ⚠️ 这两样**必须**在这里就地更新, 不能只挂在 armRenderBlock 里: 拖滑杆 / 改延时都
 *    **不能**重建 DOM（见 armRefreshBlock 的警告）, 只重建的话拖完半天块还是不变黄 ——
 *    真踩过（「修改了但是没有变黄」）。
 * ⚠️ ↺ 只在**有基线**（= 磁盘上原来就有这块）且确实改过时才出现: 新加的块没有
 *    「原值」可恢复, 给它一个按了没用的按钮只会让人以为坏了。 */
function armRefreshModified(it) {
  const el = document.getElementById('blk-arm-' + it.id);
  if (!el) return;
  const mod = armItemModified(it);
  el.classList.toggle('armmod', mod);
  el.title = mod ? (armHasBaseline(it.id) ? '改过（还没保存）—— 点 ↺ 恢复原值'
                                           : '新增的（还没保存）')
                 : (it.disabled ? '已禁用：保存后这一条在固件里被注释掉、不会执行' : '');
  const rev = el.querySelector('.b-revert');
  if (rev) rev.classList.toggle('hidden', !(mod && armHasBaseline(it.id)));
}

/* 改过块里的值后, 让它自己那行摘要就地更新。
 * ⚠️ **不能改成 armRenderCanvas()**: 拖动滑杆时重建 DOM 会把正在拖的那个滑块丢掉
 *    （指针还按着, 元素却已经不在文档里了 → 拖动直接断掉）。 */
function armRefreshBlock(id) {
  const it = ARM.items.find(x => x.id === id);
  const el = document.getElementById('blk-arm-' + id);
  if (!it || !el) return;
  armRefreshModified(it);          /* 黄标与 ↺ 也走这条路（早退之前, 延时块也要） */
  const body = el.querySelector('.b-body');
  if (!body || it.kind === 'delay' || it.kind === 'raw') return;
  body.textContent = (it.kind === 'pose') ? armPoseSummary(it) : armGripSummary(it);
}

/* 微调箭头的一步 = ±0.1°。抽成纯函数是为了能单独验算: 夹量程、一位小数
 * （不取整的话 0.1 累加会攒出 90.20000000000002 这种数, 一路写进 C 表里）。 */
const ARM_STEP_DEG = 0.1;

function armStepValue(base, dir, min, max) {
  return Math.round(Math.min(max, Math.max(min, base + dir * ARM_STEP_DEG)) * 10) / 10;
}

function renderJoints() {
  const box = jointBox();
  /* ⚠️ 空面板时直接返回: 「蓝牙」Tab 已经没有关节容器了（那里只剩调参）,
   *   而从 loadBt / linkDisconnect / onConnected 过来的调用是不分区格的 ——
   *   少了这一行, 在蓝牙 Tab 上任何一个都会以 box 为 null 抛 TypeError,
   *   连带滑杆永远轮不到渲染。 */
  if (!box) return;
  box.textContent = '';

  const tgt = armPoseTarget();          /* 选中了位姿块 → 编辑那个块 */
  const live = BT.joints.length > 0;    /* 没选中 → 实时设备值 */
  const multi = armMultiSel();          /* 多选 → 滑杆谁都不属于, 一律灰掉 */
  const ranges = live ? BT.joints : (ARM.jointsOffline || []);
  /* ⚠️ 多选时 **editable 必须是 false** —— 一组积木没有「它存的角度」,
   *    退化成「实时设备值」更糟: 拖一下会把整组里唯一能被改的那块悄悄改掉。 */
  const editable = !multi && (!!tgt || live);

  /* 状态放在**卡片标题右边**, 不藏 —— 「现在拖的是积木数据还是真机」必须一眼可辨 */
  const note = document.getElementById('arm-joints-note');
  if (note) {
    note.textContent = multi
      ? ('已选中 ' + ARM.selIds.length + ' 块 · 不绑定任何积木')
      : (tgt ? ('编辑「' + (tgt.tag || '位姿') + '」的角度')
             : (live ? '实时值（设备上报，可拖）'
                     : (ranges.length ? '未选中积木 · 离线占位（灰色）' : '')));
  }

  if (!ranges.length) {
    box.appendChild(h('p', { class: 'hint' },
      '还没读到关节表（连接后进「进入编辑模式」会自动读一次）。'));
    return;
  }
  const labels = btJointLabels();

  for (const j of ranges) {
    const val = tgt ? Number(tgt.angles[j.index] ?? j.angle) : Number(j.angle);
    const row = h('div', { class: 'bt-joint' + (editable ? '' : ' off') });
    row.appendChild(h('span', { class: 'bt-name' }, labels[j.index] || j.name));
    const rng = h('input', {
      type: 'range', min: j.min, max: j.max, step: 1,
      value: Math.round(val), 'data-joint': j.index,
      disabled: editable ? null : '',
      title: tgt ? '改的是这个积木里存的角度'
                 : (multi ? '已选中多块积木 —— 点画布空白处取消多选后才能拖'
                          : (live ? null : '未连接设备')),
    });
    const out = h('b', { class: 'bt-jval' }, val.toFixed(1) + '°');

    /* 拖滑杆与双击输入走**同一条落库路径**（写积木 / 记 touched / 让真机跟过去）,
     * 两处各写一遍迟早漂开。 */
    const applyJoint = (v) => {
      rng.value = v;
      out.textContent = v.toFixed(1) + '°';
      BT.touched[j.index] = Date.now();
      if (tgt) {
        tgt.angles[j.index] = v;
        tgt.edited = true;
        armMarkDirty();
        armRefreshBlock(tgt.id);
      }
      btJogThrottled(j.index, v);   /* 编辑模式下顺便让真机跟过去（不在编辑模式会自动跳过） */
    };
    if (editable) {
      rng.addEventListener('pointerdown', () => {
        BT.dragging = true;
        /* 一次拖动只压一次快照 —— 每个 input 都压的话撤回栈会被中间态灌满 */
        if (tgt) armPushUndo();
      });
      rng.addEventListener('input', () => applyJoint(parseFloat(rng.value)));
    }
    attachJvalEdit(out, {
      enabled: editable, min: j.min, max: j.max, unit: '°', commit: applyJoint,
      /* 编辑期间同样占住 BT.dragging —— 否则 5Hz 的 $B,POS 回读会把
       * out.textContent 重写掉, 连带把正在输入的那个 <input> 一起抹掉。 */
      onStart: () => { BT.dragging = true; if (tgt) armPushUndo(); },
      onEnd:   () => { BT.dragging = false; },
    });

    /* ---- 滑杆两端的 0.1° 微调箭头 ----
     * 拖滑杆是粗调（step=1）, 这两个是「再挪一丁点」——台架上对位时差的常常就是零点几度。
     * 落库**走的就是 applyJoint**: 与拖滑杆、双击输入同一条路（写积木 / 记 touched /
     * 编辑模式下让真机跟过去）, 三处各写一遍迟早漂开。
     * 短按一步; **按住** 400ms 后每 60ms 一步（挪 2° 要点 20 下太累）。
     * ⚠️ 一次按住只压**一次**撤回快照（按 pointerdown 算, 与拖滑杆同规则）。
     * ⚠️ 按住期间占住 BT.dragging: 否则 5Hz 的 $B,POS 回读会把滑块拽回去。
     * ⚠️ 必须 setPointerCapture 接管指针 —— 不然在按钮外松手时 pointerup 收不到,
     *    连发定时器会一直跑下去（松手后滑块自己一路爬到底）。
     * ⚠️ 灰态（未选中/多选/未连设备）下**不挂监听**, 与滑杆同一条规矩:
     *    挂了就是「看着能按, 按了没反应」。 */
    const mkStep = (dir, glyph, tip) => {
      let holdT = null, repT = null, repeated = false;
      const stop = () => {
        clearTimeout(holdT); clearInterval(repT);
        holdT = null; repT = null;
      };
      const one = () => {
        /* 基准取**显示出来的那个数**: 它就是用户看到的当前值（实时回读也往这里写）,
         * 而 range 的值在灰态/边界上不一定跟得上。 */
        const shown = parseFloat(out.textContent);
        const base = isFinite(shown) ? shown : Number(rng.value);
        const v = armStepValue(base, dir, j.min, j.max);
        if (v !== base) applyJoint(v);      /* 已经顶到量程端头就什么都不做 */
      };
      const btn = h('button', {
        class: 'btn ghost tiny bt-step', title: tip,
        disabled: editable ? null : '',
        /* 短按走 click（键盘 Enter/空格也走这条路）; 连发已经跑过就跳过这一次 ——
         * 否则长按松手时会多走一步。 */
        onclick: (e) => {
          e.stopPropagation();
          if (repeated) { repeated = false; return; }
          one();
        },
      }, glyph);
      if (editable) {
        btn.addEventListener('pointerdown', (e) => {
          if (e.button !== 0) return;
          repeated = false;
          BT.dragging = true;
          if (tgt) armPushUndo();
          try { btn.setPointerCapture(e.pointerId); } catch (err) { /* 老浏览器: 退化成普通按 */ }
          holdT = setTimeout(() => { repeated = true; repT = setInterval(one, 60); }, 400);
        });
        /* BT.dragging 不用在这里收 —— document 级的 pointerup 已经在收（见 init） */
        btn.addEventListener('pointerup', stop);
        btn.addEventListener('pointercancel', stop);
        btn.addEventListener('lostpointercapture', stop);
      }
      return btn;
    };
    row.appendChild(mkStep(-1, '◀', '微调 -0.1°（按住连发）'));
    row.appendChild(rng);
    row.appendChild(mkStep(1, '▶', '微调 +0.1°（按住连发）'));
    row.appendChild(out);
    box.appendChild(row);
  }

  /* 夹爪: 开/合 + 程度%。刻意**不做角度↔开合% 双向联动** —— vArmSetZhuaShou 的
   * 映射是多对一的(开70% 与 合30% 落到同一角度), 反解有歧义; 按步骤语义给控件,
   * 记录动作时才能无损转回 STEP_SET_ZHUASHOU_WAIT。 */
  const gt = armGripTarget();           /* 多选时 armGripTarget 也回 null（见 armSelectedItem） */
  const gripEditable = !multi && (!!gt || live);
  const curKai = gt ? (gt.obj.kai_he ? 1 : 0) : (BT.grip.kai_he ? 1 : 0);
  const curDeg = gt ? (gt.obj.degree || 0) : (BT.grip.degree || 0);

  const grow = h('div', { class: 'bt-joint bt-grip' + (gripEditable ? '' : ' off') });
  grow.appendChild(h('span', { class: 'bt-name' }, '夹爪'));
  const kai = h('select', { class: 'bt-grip-opt', disabled: gripEditable ? null : '' });
  kai.appendChild(h('option', { value: '1' }, '张开'));
  kai.appendChild(h('option', { value: '0' }, '闭合'));
  kai.value = String(curKai);
  const deg = h('input', { type: 'range', min: 0, max: 100, step: 1, value: curDeg,
                           disabled: gripEditable ? null : '' });
  const degOut = h('b', { class: 'bt-jval' }, curDeg + '%');
  const gripRo = h('span', { class: 'bt-grip-ro' }, '');

  const applyGrip = (kaiV, degV, fromSlider) => {
    if (gt) {
      gt.obj.kai_he = kaiV;
      gt.obj.degree = degV;
      if (gt.isPose) gt.obj.enabled = true;   /* 动了就是想让它有开合动作 */
      gt.owner.edited = true;
      armMarkDirty();
      armRefreshBlock(gt.owner.id);
    }
    /* ⚠️ 不论有没有选中积木, **都要镜像进 BT.grip** —— 下发真机的 btGripNow()
     *    读的是 BT.grip 而不是 gt.obj。原来只在 else 分支里写, 于是「选中一个位姿块
     *    再拖夹爪」会把**上一次**的 BT.grip 发下去（真机不动或动到别处）。 */
    BT.grip.kai_he = kaiV;
    BT.grip.degree = degV;
    degOut.textContent = degV + '%';
    if (fromSlider) btGripThrottled(); else btGripNow();
  };
  if (gripEditable) {
    kai.addEventListener('change', () =>
      applyGrip(parseInt(kai.value, 10), parseInt(deg.value, 10), false));
    deg.addEventListener('input', () =>
      applyGrip(parseInt(kai.value, 10), parseInt(deg.value, 10), true));
  }

  /* 双击那个百分比也能直接输入（与关节读数同一套）。**取整**再下发: 预设立项里
   * degree 是整数（_fmt_gripper_step 走 _to_int）, 留着小数只会在保存时被悄悄截掉。 */
  attachJvalEdit(degOut, {
    enabled: gripEditable, min: 0, max: 100, unit: '%',
    commit: (v) => {
      const d = Math.round(v);
      deg.value = d;
      applyGrip(parseInt(kai.value, 10), d, false);
    },
    onStart: () => { BT.dragging = true; if (gt) armPushUndo(); },
    onEnd:   () => { BT.dragging = false; },
  });

  grow.appendChild(kai);
  grow.appendChild(deg);
  grow.appendChild(degOut);
  grow.appendChild(gripRo);
  box.appendChild(grow);
  BT.gripRoEl = gripRo;

  /* 底部说明: 说清「现在拖的是积木还是真机」以及怎么回到另一种模式 */
  if (multi) {
    box.appendChild(h('p', { class: 'hint' },
      '已选中 ' + ARM.selIds.length + ' 块积木 —— 滑杆不绑定任何一块, 所以是灰的。'
      + '拖动其中任意一块会整组一起搬走；右键能把整组复制/移动到别的动作。'
      + '点画布空白处取消多选。'));
  } else if (tgt) {
    box.appendChild(h('p', { class: 'hint' },
      '拖动即写进这个积木（可 Ctrl+Z 撤回）；编辑模式下同时会让机械臂跟过去。'
      + '点画布空白处取消选中，就回到「拖滑杆 = 动真机」。'));
  } else if (!live) {
    box.appendChild(h('p', { class: 'hint' },
      '未选中积木时这里是灰色占位。点左侧某个位姿积木可以编辑它的角度；'
      + '连接设备并进入编辑模式后，直接拖这里就是让机械臂动（录新位姿用）。'));
  }
  if (gt && gt.isPose && !gt.obj.enabled) {
    box.appendChild(h('p', { class: 'hint' },
      '这个位姿原本不带夹爪动作；动一下夹爪就会给它加上。'));
  }
}

/* 只更新滑块数值, 不重建 DOM（重建会把正在拖的滑块弄丢） */
function renderJointsKeepValues() {
  const ranges = jointRanges();
  if (!ranges.length) { renderJoints(); return; }
  for (const rng of ranges) {
    const i = parseInt(rng.dataset.joint, 10);
    const j = BT.joints.find(x => x.index === i);
    if (!j) continue;
    rng.min = j.min; rng.max = j.max;
    if (Date.now() - (BT.touched[i] || 0) > 1200) {
      rng.value = j.angle;
      const out = rng.parentElement.querySelector('.bt-jval');
      if (out) out.textContent = j.angle.toFixed(1) + '°';
    }
  }
}

function btJogThrottled(index, angle) {
  clearTimeout(BT.jogTimer[index]);
  BT.jogTimer[index] = setTimeout(() => {
    if (!BT.edit) return;      /* 设备只在编辑模式下收臂指令, 别白发（会静默 NOTEDIT） */
    /* ms=0 = 固件的**跟手模式**: 目标直接写进舵机, 不走 S 曲线插值。
     * 走 S 曲线的话每次新目标都把曲线从缓入段重启, 拖起来就是「松手才走完」。
     * 50ms 拖尾节流（latest-wins: 只发最后一个值, 中间的直接丢掉）——
     * 原来的 80ms 是为走完整曲线留的余量, 直通模式不需要。 */
    btRun('jog', { index: index, angle: angle, ms: 0 }, { quiet: true, silent: true });
  }, 50);
}

function btGripThrottled() {
  clearTimeout(BT.jogTimer.grip);
  BT.jogTimer.grip = setTimeout(btGripNow, 120);
}

/* ⚠️ 不弹 toast: 这个函数会被夹爪滑块每 120ms 调一次, 离线编辑积木时弹一次就刷屏了 */
function btGripNow() {
  if (!BT.edit) return;
  btRun('grip', { kai_he: BT.grip.kai_he, degree: BT.grip.degree },
    { quiet: true, silent: true });
}

function btSetEdit(on, auto) {
  BT.edit = !!on;
  const onBtn = document.getElementById('arm-edit-on');
  const offBtn = document.getElementById('arm-edit-off');
  if (onBtn) onBtn.disabled = BT.edit;
  if (offBtn) offBtn.disabled = !BT.edit;
  /* 「底盘动作组」Tab 的进/出按钮同一个状态源 —— 编辑模式全工程只有一份,
   * 从哪个 Tab 进/出（含设备自己超时踢出、另一边手动进出）两边按钮都要跟上。 */
  const chOn = document.getElementById('chact-edit-on');
  const chOff = document.getElementById('chact-edit-off');
  if (chOn) chOn.disabled = BT.edit;
  if (chOff) chOff.disabled = !BT.edit;
  /* 「获取当前位姿」跟着编辑模式走: $B,POS 有门禁, 不在编辑模式设备只回 NOTEDIT */
  const poseBtn = document.getElementById('arm-get-pose');
  if (poseBtn) poseBtn.disabled = !BT.edit;
  if (BT.edit) {
    btStartPoll();
  } else if (BT.posTimer) {
    clearInterval(BT.posTimer);
    BT.posTimer = null;
  }
  if (auto) btlog('设备已自动退出编辑模式（树莓派握手成功，或蓝牙断开超时）', 'bad');
}

function btStartPoll() {
  if (BT.posTimer) return;
  BT.posTimer = setInterval(btPollPos, 200);      /* 5Hz 回读真实角度 */
}

/* 离开蓝牙 Tab 时把所有定时器停掉（switchTab 调用; btPollPos 里还有一道双保险） */
function btStopTimers() {
  if (BT.posTimer) { clearInterval(BT.posTimer); BT.posTimer = null; }
  if (BT.autoTimer) { clearInterval(BT.autoTimer); BT.autoTimer = null; }
  const cb = document.getElementById('bt-param-auto');
  if (cb) cb.checked = false;
}

/* 「链路相关」的三个 Tab（蓝牙 + 机械臂 + 底盘动作组）—— 定时器只在这三个 Tab 之一
 * 可见时跑, 否则两个 5Hz 的 $B,POS 会交替抢同一条命令锁, 表现为回读卡顿。
 * ⚠️ 底盘动作组必须在列: 它的预览走非阻塞 $B,CPLAY, 播放结束后没有别的东西喂
 *    固件的 8s 空闲看门狗 —— 靠这里的 5Hz $B,POS 保活, 否则每次预览完编辑模式
 *    都会被固件判成「PC 掉线」踢掉, 用户就得重新进一次。 */
function linkTabVisible() { return curTab === 'bt' || curTab === 'arm' || curTab === 'chact'; }

async function btPollPos() {
  if (!linkTabVisible()) { btStopTimers(); return; }   /* 双保险: 别让定时器漏出去 */
  /* 拖动/播放期间不发: 一是会跟正在发的 JOG 抢命令锁, 二是回读会把用户正拖的
   * 滑块拽回去（BT.touched 兜了 1.2s, 但那是兜底不是本分）。 */
  if (BT.dragging || BT.playing) return;
  const pos = await btRun('pos', {}, { quiet: true, silent: true });
  if (pos === null) {
    /* ⚠️ 单拍超时很可能是蓝牙**偶发抖动**（命令超时 1.5s）, 而 btSetEdit(false, true)
     * 会顺手把用户踢出编辑模式 —— 台架上正调到一半被踢掉非常恼人。
     * 连续 BT_POS_FAILS_TO_DROP 拍都失败才判掉线（5Hz 轮询下多等一拍 = 200ms,
     * 真掉线照样立刻能看出来）。 */
    if (++BT.posFails < BT_POS_FAILS_TO_DROP) return;
    btSetEdit(false, true);
    return;
  }
  BT.posFails = 0;
  btApplyPos(pos);
}

function btApplyPos(pos) {
  const ranges = jointRanges();
  for (const rng of ranges) {
    const i = parseInt(rng.dataset.joint, 10);
    if (Date.now() - (BT.touched[i] || 0) < 1200) continue;   /* 用户正在拖, 别抢 */
    const v = pos.angles[i];
    if (typeof v !== 'number') continue;
    rng.value = v;
    const out = rng.parentElement.querySelector('.bt-jval');
    if (out) out.textContent = v.toFixed(1) + '°';
  }
  if (BT.gripRoEl) {
    BT.gripRoEl.textContent = '当前 ' + pos.gripper.toFixed(1) + '°' + (pos.done ? ' · 停稳' : '');
  }
}

/* 双击读数就地编辑（关节的度数 / 夹爪的百分比）。
 *
 * ⚠️ 提交后必须把那个 <b> 还原回原位: btApplyPos（5Hz 轮询）与 applyJoint 都靠
 *    parentElement.querySelector('.bt-jval') 找它 —— 换成 input 却不还回去, 回读就
 *    从此失效。所以**不能**复用现成的 editInPlace: 那个是 replaceWith 后不还原的
 *    （它是给积木标签用的, 那些元素不参与回读）。
 * 越界 / 非数字 → **本次修改作废**: 原样恢复, 也不发任何指令, 只弹一句提示。 */
function attachJvalEdit(out, opt) {
  if (!opt.enabled) return;
  out.classList.add('bt-jval-edit');
  out.title = '双击可直接输入';
  out.addEventListener('dblclick', () => {
    if (out.querySelector('input')) return;          /* 已经在编辑中了 */
    const old = out.textContent;
    const cur = parseFloat(old);
    const inp = h('input', { class: 'bt-jvedit', type: 'number',
                             min: opt.min, max: opt.max, step: 'any',
                             value: isFinite(cur) ? cur : '' });
    if (opt.onStart) opt.onStart();
    out.textContent = '';
    out.appendChild(inp);
    inp.focus(); inp.select();

    let done = false;
    const finish = (apply) => {
      if (done) return;
      done = true;
      const v = parseFloat(inp.value);
      const ok = apply && isFinite(v) && v >= opt.min && v <= opt.max;
      inp.remove();
      if (ok) {
        opt.commit(v);                               /* 由 commit 负责写回显示文字 */
      } else {
        out.textContent = old;
        if (apply && isFinite(v)) {
          toast('超出范围 ' + opt.min + '~' + opt.max + opt.unit + '，本次修改无效', 'err');
        }
      }
      if (opt.onEnd) opt.onEnd();
    };
    inp.addEventListener('blur', () => finish(true));
    inp.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); inp.blur(); }
      else if (e.key === 'Escape') { e.preventDefault(); finish(false); }
    });
  });
}

/* 夹爪角度 → 控件用的（开合, 程度%）。
 *
 * 固件映射是**多对一**的（app_arm.c::vArmSetZhuaShouNoWait:
 *   openPct = kai ? degree : 100 - degree;  angle = CLOSE + (OPEN-CLOSE)·openPct/100）
 * ——「张开 70%」与「闭合 30%」是同一个角度, 所以反解必须先选分支。选法就是
 * 「根据范围自动选择区间」: 开度落在**半程以上**记成「张开 d%」, 否则「闭合 (100-d)%」。
 * 两条分支都能无损回代（只是 1% = 全量程/100 的量化误差）。 */
function armGripperFromAngle(angle, span) {
  const d = span.close - span.open;
  if (!isFinite(d) || Math.abs(d) < 0.01) return null;
  let pct = (span.close - angle) / d * 100;          /* 开度 0~100 */
  if (!isFinite(pct)) return null;
  pct = Math.max(0, Math.min(100, pct));
  return (pct >= 50)
    ? { kai_he: 1, degree: Math.round(pct) }
    : { kai_he: 0, degree: Math.round(100 - pct) };
}

/* 「获取当前位姿」: 读设备实时角度, **当成滑杆的目标值**填进去并落库。
 *
 * 与 btApplyPos 不是重复劳动:
 *   btApplyPos 是 5Hz 轮询的**无副作用显示**（只更新「当前 xx°」那行小字, 且避让正在拖的滑块）;
 *   这里是用户显式说「把真机现在这个样子记下来」, 所以主动覆盖滑杆, 并走与拖动完全相同的
 *   落库路径（写积木 / 写 BT.grip / 让真机跟过去——跟过去是空操作, 它本来就在那儿）。
 * 按钮在 btSetEdit 里跟着编辑模式启用: $B,POS 有门禁, 不在编辑模式设备只回 NOTEDIT。 */
async function armGetPose(opts) {
  const quiet = !!(opts && opts.quiet);
  const pos = await btRun('pos', {}, quiet ? { quiet: true } : {});
  if (pos === null) return;

  const tgt = armPoseTarget();
  /* 写进积木前压一份快照 —— 「获取当前位姿」会**改数据**, 而且进编辑模式时是自动
   * 调用的（见 btEnterEdit）, 没有撤回点的话一次误触就把块里的值盖掉了。 */
  if (tgt) armPushUndo();
  for (const rng of jointRanges()) {
    const i = parseInt(rng.dataset.joint, 10);
    const v = pos.angles[i];
    if (typeof v !== 'number' || !isFinite(v)) continue;
    rng.value = v;
    const out = rng.parentElement.querySelector('.bt-jval');
    if (out) out.textContent = v.toFixed(1) + '°';
    BT.touched[i] = Date.now();          /* 别让紧接着的那次轮询把它拉回去 */
    if (tgt) {
      tgt.angles[i] = v;
      tgt.edited = true;
      armRefreshBlock(tgt.id);
    }
  }
  if (tgt) armMarkDirty();

  /* 夹爪: 设备回的是**角度**, 而控件是「开合 + 程度%」—— 反解见 armGripperFromAngle。
   * OPEN/CLOSE 是编译进固件的常量（设备不上报）, 由服务端从 app_config.h 派生。 */
  const box = jointBox();
  const kai = box && box.querySelector('.bt-grip-opt');
  const deg = box && box.querySelector('.bt-grip input[type=range]');
  if (ARM.gripperSpan && kai && deg) {
    const g = armGripperFromAngle(pos.gripper, ARM.gripperSpan);
    if (g) {
      kai.value = String(g.kai_he);
      deg.value = String(g.degree);
      /* 派发 input 复用挂好的监听器 —— 和手拖那条路一字不差, 不用把 renderJoints
       * 里的闭包再暴露一份出来。 */
      deg.dispatchEvent(new Event('input', { bubbles: true }));
    }
  }
  if (!quiet) toast('已读取当前位姿');
}

/* 进入编辑模式。auto=true 是「连上就自动试一次」那条路。
 *
 * **成败都报 toast**（含自动那次）——「有没有让位成功」是操作员唯一要确认的事,
 * 藏在日志里没人翻。自动那条的失败文案前面加一句「未自动进入」, 免得看着像 bug
 * （设备不在开机等握手的阶段、或压根没上电, 自动进不去都是常态）。
 * ⚠️ 手动按钮的监听器必须写成 `() => btEnterEdit()`（见 arm-edit-on 的注册）——
 *    直接挂函数的话 click 的 Event 会被当成 auto=true, 手动那条就报错了。
 * 2026-10: auto 那条路在首试失败后会**静默重试**几次 —— 因为调用方可能刚投过
 * 「握手让出」（$B,HS）: 脚本把当前路线跑完这一步（≤1s 居多）才真正进让位窗,
 * 期间 MODE,1 一律 NOHANDSHAKE, 立刻报死反而是假故障。 */
async function btEnterEdit(auto) {
  let data = await btRun('mode', { on: true }, {
    quiet: true,          /* 成败都由这里统一报, 别让 btRun 先弹一条 */
    silent: true,         /* 重试会打好几条失败日志, 全静默, 最后一次统一说 */
    onError: () => {},
  });
  if (data === null && auto) {
    for (let i = 0; i < 8 && data === null; i++) {
      await new Promise(r => setTimeout(r, 350));
      data = await btRun('mode', { on: true }, { quiet: true, silent: true, onError: () => {} });
    }
  }
  if (data === null) {
    toast(auto ? '未自动进入编辑模式（设备不在让位窗, 或脚本没在让位）' :
      '进入编辑模式失败（设备不在握手等待阶段）', 'err');
    return false;
  }
  btSetEdit(true);
  /* 关节滑杆的内容（范围 + 当前角度）由设备上报, 进编辑模式就顺手拉一次 ——
   * 否则「连接 → 直接点进入编辑模式」会看到一个空面板（这步不能省）。 */
  await btReadParams(true);
  renderJoints();
  /* 再自动获取一次当前位姿（等同点一次「获取当前位姿」）: LIST 带回来的角度是
   * 设备缓存里那一份, 而这里要的是**此刻**的读数, 而且夹爪那一行只有 $B,POS 有
   * （LIST 的关节表里没有夹爪）—— 不补这一步, 进编辑模式后夹爪控件显示的是上一次
   * 的开合/程度, 直接拖就等于把臂往一个凭空的姿态上带。
   * quiet: 这是自动行为, 别弹 toast; 失败也只是日志里一行（改了 btSetEdit 的按钮态
   * 之后设备仍可能已退出编辑模式, 那时回 NOTEDIT 是正常的）。 */
  await armGetPose({ quiet: true });
  renderJoints();
  toast('已进入编辑模式：脚本已让位, 六个关节交给蓝牙点动', 'ok');
  return true;
}

/* 退出编辑模式。**什么都不动舵机** —— 脚本一离开让位状态就接着跑路线, 这里再回位姿
 * 会盖掉路线那一步的臂动作（固件是后写者胜）。要回上电默认位姿就在路线里排组动作。
 * 2026-10 起固件在「退出且正处让位窗」时会立刻踢 Pi 重启客户端 → 链路自动回到 OK、
 * 脚本接着走（toast 里说明, 免得操作员看 OLED 还停在 RPI:Waiting 就以为没恢复）。 */
async function btExitEdit() {
  const data = await btRun('mode', { on: false });
  if (data === null) return;
  btSetEdit(false);
  toast('已退出编辑模式（Pi 链路自动恢复中, 约 20s）', 'ok');
}

/* ==================== Tab: 机械臂 ====================
 * 台架上录一段臂动作 → 存成「动作预设」→ 自动变成「地图」积木库里的一块大积木。
 *
 * 与「地图」是两层独立的东西（像嵌套 for）: 外层是路线预设（map.c）, 内层是动作
 * 预设（app_arm_action.c）; 路线里用 STEP_ARM_PLAY_WAIT 按 ID 引用动作。删一个动作
 * 会让引用它的路线行变成**编译错误**（故意的硬失败, 免得静默指向别的动作）, 所以
 * 删除前会问一句引用了多少步。
 *
 * 播放一律走比赛用的那批原语: 单条 = $B,ARM（5 轴同时启 S 曲线 + 夹爪）; 整段要么
 * PC 逐条编排（未保存的编辑也能播）, 要么交给固件 $B,PLAY 播已编译的那一份 —— 后者
 * 与路线里的 STEP_ARM_PLAY_WAIT 是**同一个函数**, 是「台架试跑 == 比赛」的证据链。
 * 拖滑杆则相反, 走固件的「跟手模式」（ms=0）, 见 btJogThrottled。
 *
 * 保存是**显式**的（像「地图」Tab）: 改动只留在内存, 按「保存动作」才写盘。
 * 撤回/重做只管当前预设的条目与标题/备注/播放方式 —— 预设的增删不走撤回栈,
 * 那条路由「回收站」兜（两套安全网各管一件事, 别指望一个按钮什么都兜住）。
 */

const ARM_UNDO_MAX = 50;
const ARM_POSE_DEFAULT = [90, 90, 90, 90, 180];   /* 与固件上电默认位姿一致 */

const ARM = {
  loaded: false,
  version: null,        // {c, h} 两个 mtime, 保存时回传做冲突检测
  presets: [],          // [{id, title, note, nowait, items:[...]}]（id 为 null = 还没存过）
  activeIdx: 0,
  items: [],            // = presets[activeIdx].items 的**活引用**（勿重新赋值）
  trash: [],
  dirty: false,
  selectedId: null,     // 主选中项（= selIds 的最后一个）; 「关节」面板只在单选时绑它
  selIds: [],           // 选中的全部 id（Ctrl+点击累加）—— 复制/移动/整组拖动都看它
  seq: 1,
  undo: [],
  redo: [],
  zonesMeta: [],
  jointsOffline: [],    // 没连设备时画灰色滑杆用的占位表（服务端从源码派生）
  gripperSpan: null,    // {open,close,min,max}: 夹爪角度↔开合% 反解用（服务端从 app_config.h 派生）
  base: {},             // {条目 id: 上次加载/保存时的副本} —— 「这个块改了没有」的基线
};

function armCurPreset() { return ARM.presets[ARM.activeIdx] || null; }
function armSyncItems() { const p = armCurPreset(); ARM.items = p ? p.items : []; }
function armNewId() { return 'a' + (ARM.seq++); }
function armMarkDirty() { ARM.dirty = true; armRenderBar(); }

/* ---- 「这个块被改过没有」的基线 ----
 * ARM.dirty（预设级）只能告诉你「这条动作有问题」, 看不出**是哪一块**。台架上录
 * 一条长动作时, 改坏了一块却记不住是哪个, 就成了常见事故 —— 于是留一份「上次加载/
 * 保存时的样子」, 逐块比对: 对不上 → 积木左边框变黄（.armmod）+ 选中后关节卡片里
 * 出现「↺ 恢复原值」。
 *
 * 基线**只在下面这几处重建**: armReload / armSave / armSwitchPreset /
 * armAddPreset / armDupPreset / armOpenPreset —— 也就是「与磁盘对齐」和「换了一条
 * 动作」的时刻。⚠️ 别塞进 armRenderAll: 它的调用点里有撤回/重做, 在那里重建基线
 * 会让刚撤回掉的东西变成「没改过」。 */
function armItemKey(it) {
  const g = it.gripper
    ? [!!it.gripper.enabled, it.gripper.kai_he | 0, it.gripper.degree | 0] : null;
  const a = it.angles ? it.angles.map(Number) : null;
  return JSON.stringify([
    it.kind, it.tag || '', a, g,
    (it.ms === undefined || it.ms === null) ? null : Number(it.ms),
    !!it.disabled, it.lines || null,
  ]);
}

function armBaseCapture() {
  ARM.base = {};
  for (const it of ARM.items) ARM.base[it.id] = JSON.parse(JSON.stringify(it));
}

function armHasBaseline(id) { return !!(ARM.base && ARM.base[id]); }

/* 改过了吗? 没有基线的（新加的块）也算 —— 它同样还没进固件, 一样该提醒。 */
function armItemModified(it) {
  const b = ARM.base && ARM.base[it.id];
  if (!b) return true;
  return armItemKey(it) !== armItemKey(b);
}

/* 把一个块恢复成基线里的样子（id 必须保住: 它是 DOM id、选中判据、拖拽 skip 判据）。 */
function armRevertItem(id) {
  const b = ARM.base && ARM.base[id];
  const i = ARM.items.findIndex(x => x.id === id);
  if (!b || i < 0) { toast('这一块没有可恢复的原值', 'err'); return; }
  armPushUndo();
  const copy = JSON.parse(JSON.stringify(b));
  copy.id = id;
  ARM.items[i] = copy;
  armMarkDirty();
  armRenderCanvas();
  renderJoints();
  btlog('↺ 已恢复「' + (copy.tag || armKindName(copy.kind)) + '」到上次加载/保存时的样子');
}

function armKindName(kind) {
  return kind === 'delay' ? '延时'
    : (kind === 'grip' ? '夹爪' : (kind === 'pose' ? '位姿' : '只读'));
}

/* 没连设备时滑杆读出来全是 0 —— 那不是个能用的位姿（0° 会把关节顶到极限）,
 * 所以退回到固件的上电默认位姿。 */
function armDefaultAngles() {
  const a = btCurrentAngles();
  return a.some(v => v !== 0) ? a : ARM_POSE_DEFAULT.slice();
}

function armPoseItem(name) {
  return {
    kind: 'pose', tag: name || '位姿', angles: armDefaultAngles(),
    gripper: { enabled: true, kai_he: BT.grip.kai_he, degree: BT.grip.degree },
    disabled: false,
    edited: true,
  };
}

/* ---- 请求 ---- */

async function armCall(action, extra) {
  const res = await fetch('/api/arm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ action: action }, extra || {})),
  });
  if (!res.ok) {
    /* ⚠️ 正文里是服务端写好的中文说明（如 409 的 CONFLICT）—— 不能丢掉它只回一句
     * 「HTTP 409」, 用户不知道发生了什么。响应体解析失败才退回状态码。 */
    const j = await res.json().catch(() => ({}));
    const e = new Error(j.message || j.error || ('HTTP ' + res.status));
    e.status = res.status;
    throw e;
  }
  return res.json();
}

/* 与 btRun 同款: 打日志、弹 toast、把 data 交回调用方; 失败回 null。 */
async function armRun(action, extra, opts) {
  opts = opts || {};
  try {
    const j = await armCall(action, extra);
    if (!opts.silent) for (const l of (j.log || [])) btlog(l.text, l.kind);
    if (!j.ok) {
      const msg = j.error || '操作失败';
      if (!opts.silent) btlog('✗ ' + msg, 'bad');
      if (!opts.quiet && !opts.silent) toast(msg, 'err');
      return null;
    }
    return j.data === undefined ? {} : j.data;
  } catch (e) {
    if (!opts.silent) btlog('✗ ' + e.message, 'bad');
    if (!opts.quiet && !opts.silent) toast('请求失败: ' + e.message, 'err');
    /* 409 = 别人先保存了。立刻拉一次版本戳, 让提示条（带「↺ 重载」按钮）出现 ——
     * 否则用户只看到一句冲突提示, 而机械臂 Tab 里**本来没有任何重载入口**。 */
    if (e.status === 409) revPoll();
    return null;
  }
}

/* ---- 加载 ---- */

function loadArm() {
  /* ARM.loaded 仍在本入口同步置位（switchTab/revLoaded 靠它判「进过这个 Tab」）;
   * 真正的加载 promise 存 ARM.loading —— 并发调用共享同一份, 别发第二个请求。
   * （armOpenPreset 必须 await 它: 只判 ARM.loaded 会在空 presets 上 findIndex,
   *   首次点积木 ✎ 必弹「这个动作不在此工程里」的假错误。） */
  ARM.loaded = true;
  if (!ARM.loading) {
    ARM.loading = (async () => {
      try {
        await btSyncState();
        /* 关节滑杆的内容（5 个轴的角度范围）**由设备上报**（$B,LIST）, 不连就没有 ——
         * 所以这里要主动拉一次。少了这一步, 从「蓝牙」Tab 连好再切过来会看到空面板。 */
        if (BT.connected) await btReadParams(true);
      } catch (e) { /* 拿不到连接状态不影响编辑 */ }
      try {
        await armReload();  /* 里面会 renderJoints —— 离线占位表也是它带回来的 */
      } catch (e) {
        toast('读取动作预设失败: ' + e.message, 'err');
      } finally {
        ARM.loading = null;
      }
    })();
  }
  return ARM.loading;
}

async function armReload() {
  const keepId = (armCurPreset() || {}).id;
  const res = await fetch('/api/arm');
  const st = await res.json();
  if (st.error) toast('读取动作预设失败: ' + st.error, 'err');
  ARM.version = st.version || null;
  ARM.presets = st.presets || [];
  ARM.trash = st.trash || [];
  ARM.jointsOffline = st.joints_offline || [];
  ARM.gripperSpan = st.gripper_span || null;
  /* 服务端没给离线关节表 = 页面上的 JS 比服务端旧（重启服务端不会刷新已打开的页面）。
   * 那种情况下滑杆会静默退化成一句提示, 所以这里明说一句 —— 排查过一次的事。 */
  if (!st.joints_offline) {
    btlog('⚠ 服务端没有 joints_offline —— 页面是旧的, 请 Ctrl+F5 强制刷新', 'bad');
  }
  /* 同理: 少了夹爪量程则「获取当前位姿」不回填夹爪那一行（其余关节正常） */
  if (!st.gripper_span) {
    btlog('⚠ 服务端没有 gripper_span —— 页面旧或 app_config.h 里 ZHUASHOU_* 解析失败, '
        + '「获取当前位姿」不会回填夹爪', 'bad');
  }
  const idx = keepId ? ARM.presets.findIndex(p => p.id === keepId) : -1;
  ARM.activeIdx = idx >= 0 ? idx : Math.min(ARM.activeIdx, Math.max(0, ARM.presets.length - 1));
  ARM.dirty = false;
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  /* ⚠️ 顺序不能反: 先对齐基线**再**渲染。反过来的话画布是拿**上一条**（或保存前）的
   * 基线去比的, 所有块会一路黄着, 而且之后再没人重绘 → 黄到下次操作才消失。 */
  armSyncItems();
  armBaseCapture();   /* 刚与磁盘对齐 → 这就是「原值」的基线 */
  armRenderAll();
  renderJoints();     /* 没连设备也要把灰色滑杆画出来（离线占位表刚拿到） */
}

/* ---- 渲染 ---- */

function armRenderAll() {
  armSyncItems();
  /* 选中的那些如果已经不在了（切预设 / 重载 / 被删 / 被搬到别的动作）,
   * 选中态要一并清掉 —— 否则「关节」面板会一直显示一个已不存在的积木的角度 */
  if (ARM.selectedId && !ARM.items.some(i => i.id === ARM.selectedId)) ARM.selectedId = null;
  ARM.selIds = ARM.selIds.filter(id => ARM.items.some(i => i.id === id));
  if (!ARM.selIds.length) ARM.selectedId = null;
  else if (ARM.selIds.indexOf(ARM.selectedId) < 0) ARM.selectedId = ARM.selIds[ARM.selIds.length - 1];
  armRenderBar();
  armRenderCanvas();
  /* ⚠️ 上面刚清掉的**可能正是**选中的那块, 而「关节」面板是照着选中态画的 ——
   * 只清状态不重画, 卡片就还写着「编辑「位姿3」的角度」、滑杆也还能拖, 但那个块
   * 已经不在 ARM.items 里了: 拖了没反应（写进一个游离对象）, 正是那句注释想防的事。
   * 撤回/重做、删除、重启后重新对齐都会走到这里, 所以重画放在函数**末尾**兜住全部调用点。 */
  renderJoints();
}

function armRenderBar() {
  const sel = document.getElementById('arm-preset-select');
  sel.textContent = '';
  ARM.presets.forEach((p, i) => {
    sel.appendChild(h('option', { value: i },
      (p.title || ('动作' + (i + 1))) + (p.id ? '' : ' · 未保存')));
  });
  sel.value = String(ARM.activeIdx);

  const p = armCurPreset();
  const t = document.getElementById('arm-title');
  const n = document.getElementById('arm-note');
  if (document.activeElement !== t) t.value = p ? (p.title || '') : '';
  if (document.activeElement !== n) n.value = p ? (p.note || '') : '';
  document.getElementById('arm-mode').value = (p && p.nowait) ? 'nowait' : 'wait';

  document.getElementById('arm-preset-del').disabled = ARM.presets.length <= 1;
  document.getElementById('arm-undo').disabled = !ARM.undo.length;
  document.getElementById('arm-redo').disabled = !ARM.redo.length;
  document.getElementById('arm-save').textContent = ARM.dirty ? '保存动作 *' : '保存动作';
  document.getElementById('arm-trash-open').textContent =
    ARM.trash.length ? ('♻ 回收站(' + ARM.trash.length + ')') : '♻ 回收站';
}

function armRenderCanvas() {
  /* 进 DOM 前先保证每条都有**唯一** id: 它同时是 DOM id（blk-arm-<id>）、选中判据、
   * 拖拽落点的 skip 判据、删除的定位键。缺了/重了会表现成「点一块选中全部」+
   * 「拖动其它块不让位」+「删除永远删第一块」—— 服务端一定要给（arm_action 有测试），
   * 但这条静默失效太难查, 所以这里再兜一次并留痕。 */
  const seen = new Set();
  let patched = 0;
  for (const it of ARM.items) {
    if (!it.id || seen.has(it.id)) { it.id = armNewId(); patched++; }
    seen.add(it.id);
  }
  if (patched) btlog('⚠ ' + patched + ' 条子积木的 id 缺失或重复, 已临时补上（服务端 bug?）', 'bad');

  const inner = document.getElementById('arm-canvas-inner');
  inner.textContent = '';
  const zone = h('div', { class: 'zone' });
  const ids = [];
  if (!ARM.items.length) {
    zone.appendChild(h('p', { class: 'hint' },
      '这个动作还是空的。用右侧「记录当前位姿」加一条（C 不允许空数组体, 至少留一条）。'));
  }
  for (const it of ARM.items) { zone.appendChild(armRenderBlock(it)); ids.push(it.id); }
  inner.appendChild(zone);
  /* 单分区: 拖拽落点计算只需要一个容器（见 makeDnd::computeBeforeId） */
  ARM.zonesMeta = [{ el: zone, ids: ids, nextFirstId: null }];
}

/* 换了一条动作 / 与磁盘对齐 / 整条被替换时, 选中一律归零。
 * ⚠️ 必须**显式**清, 不能只靠 armRenderAll 里那句「id 不在 items 里就清掉」:
 *    两条预设的子积木 id **会重号**（都是从 map.c 解析出来的 i0/i1…）, 于是切过去
 *    之后新动作里会有一块"莫名其妙已经被选中" —— 这时拖滑杆改的就是它。 */
function armClearSelection() { ARM.selIds = []; ARM.selectedId = null; }

/* 选中的**唯一入口**: 单选 / Ctrl+点击（切换一块）/ 取消, 三种都落到这里。
 * selIds 是「选了哪几块」的唯一真相, selectedId 只是它的末位（主选中）。 */
function armSetSelection(ids) {
  ARM.selIds = (ids || []).filter(id => ARM.items.some(i => i.id === id));
  ARM.selectedId = ARM.selIds.length ? ARM.selIds[ARM.selIds.length - 1] : null;
  for (const el of document.querySelectorAll('#arm-canvas-inner .block')) {
    el.classList.toggle('sel', ARM.selIds.includes(el.dataset.id));
  }
  renderJoints();     /* 「关节」面板跟着选中走: 选中位姿块 → 编辑它存的角度 */
}

function armSelect(id) { armSetSelection(id ? [id] : []); }

/* Ctrl+点击: 在选中集合里加上/拿掉这一块（多选取「并集」, 与文件管理器一致）。
 * ⚠️ 取消掉最后一块 = 回到「没选中」, 滑杆回到实时设备值 —— 不要让它变成"还剩 0 块"。 */
function armToggleSelect(id) {
  const cur = ARM.selIds.slice();
  const i = cur.indexOf(id);
  if (i >= 0) cur.splice(i, 1); else cur.push(id);
  armSetSelection(cur);
}

/* 点画布空白处取消选中 → 「关节」面板回到「拖滑杆 = 动真机」那套 */
function armDeselect() {
  if (!ARM.selIds.length) return;
  armSetSelection([]);
}

function armRenderBlock(it) {
  const mod = armItemModified(it);
  const el = h('div', {
    id: 'blk-arm-' + it.id,
    class: 'block' + (it.disabled ? ' disabled' : '') + (mod ? ' armmod' : '')
                 + (ARM.selIds.includes(it.id) ? ' sel' : ''),
    'data-id': it.id,
    title: mod
      ? (armHasBaseline(it.id) ? '改过（还没保存）—— 选中它可「恢复原值」'
                               : '新增的（还没保存）')
      : (it.disabled ? '已禁用：保存后这一条在固件里被注释掉、不会执行' : null),
    onclick: (e) => {
      if (e.target.closest('input,select,button')) return;
      if (ARMDND.justDragged()) return;
      /* Ctrl+点击 = 多选（累加/取消这一块）; 平点 = 单选。
       * 多选之后整组能一起拖走、一起右键复制/移动到别的动作。 */
      if (e.ctrlKey || e.metaKey) armToggleSelect(it.id);
      else armSelect(it.id);
    },
    /* 右键菜单: 复制 / 移动到别的动作预设（对**整个选中组**生效）。
     * 右键一块**不在**当前选中组里的积木 → 先把选中换成它, 再弹菜单
     * （与「地图」Tab 右键一致: 菜单永远是对着你看的那一块开的）。 */
    oncontextmenu: (e) => {
      if (e.target.closest('input,select,button')) return;
      e.preventDefault();
      if (!ARM.selIds.includes(it.id)) armSelect(it.id);
      openArmCtx(e.clientX, e.clientY);
    },
    onpointerdown: (e) => {
      if (e.button !== 0) return;
      if (e.target.closest('input,select,button')) return;
      ARMDND.begin({ mode: 'move', id: it.id }, e);
    },
  });

  const head = h('div', { class: 'b-head' });
  /* 眼睛 = 禁用/启用这一条（禁用后保存出来是行首 // 的注释行, 固件里不存在,
   * 于是它既不执行也不占 usCount）—— 与「地图」Tab 的单步禁用同一套。 */
  head.appendChild(h('button', {
    class: 'eye' + (it.disabled ? ' off' : ''),
    title: it.disabled ? '启用（去掉行首 //）' : '禁用（行首加 //）',
    onclick: (e) => {
      e.stopPropagation();
      armPushUndo();
      it.disabled = !it.disabled;
      armMarkDirty();
      armRenderCanvas();
      renderJoints();
    },
  }, it.disabled ? '🚫' : '👁'));
  const tag = h('span', { class: 'b-tag', title: '双击改名' }, it.tag || armKindName(it.kind));
  tag.addEventListener('dblclick', (e) => {
    e.stopPropagation();
    editInPlace(tag, it.tag || '', (v) => {
      armPushUndo();
      it.tag = v;
      it.edited = true;
      armMarkDirty();
      armRenderCanvas();
    });
  });
  head.appendChild(tag);
  head.appendChild(h('span', { class: 'b-type' }, armKindName(it.kind)));
  el.appendChild(head);

  if (it.kind === 'delay') {
    const ms = h('input', { type: 'number', class: 'arm-ms', min: 0, step: 10, value: it.ms });
    ms.addEventListener('change', () => {
      armPushUndo();
      it.ms = Math.max(0, parseInt(ms.value, 10) || 0);
      it.edited = true;
      armMarkDirty();
      armRefreshBlock(it.id);      /* 黄标/↺ 就地更新 —— 这里也不重建 DOM */
    });
    el.appendChild(h('div', { class: 'b-body' },
      h('span', { class: 'p-name' }, '等待'), ms, h('span', { class: 'p-unit' }, 'ms')));
  } else if (it.kind === 'pose') {
    el.appendChild(h('div', { class: 'b-body' }, armPoseSummary(it)));
  } else if (it.kind === 'raw') {
    el.appendChild(h('div', { class: 'b-body' },
      h('span', { class: 'hint' },
        '只读块（' + (it.step || '未知步骤') + '）—— 保存时按原样写回, 界面上不改它')));
  }

  const btns = h('div', { class: 'b-btns' });
  if (it.kind === 'pose') {
    btns.appendChild(h('button', {
      class: 'btn ghost tiny', title: '播放这一条（走 S 曲线）',
      onclick: (e) => { e.stopPropagation(); armPlayItem(it); },
    }, '▶'));
  }
  /* ↺ 恢复原值: 只在「改过的、且磁盘上原来就有」的块上露出来（显隐由
   * armRefreshModified 就地切 —— 拖动中不重建 DOM 也照样能亮起来）。 */
  if (it.kind !== 'raw') {
    btns.appendChild(h('button', {
      class: 'btn ghost tiny b-revert' + ((mod && armHasBaseline(it.id)) ? '' : ' hidden'),
      title: '恢复这一块到上次加载/保存时的样子',
      onclick: (e) => { e.stopPropagation(); armRevertItem(it.id); },
    }, '↺'));
  }
  btns.appendChild(h('button', {
    class: 'btn ghost tiny', title: '删除这一条（也可以拖到右下角垃圾桶）',
    onclick: (e) => { e.stopPropagation(); armDeleteItem(it.id); },
  }, '✕'));
  el.appendChild(btns);
  return el;
}

function armPoseSummary(it) {
  const names = ['s1', 's2', 's3', 's4', 's6'];   /* 与固件组顺序一致（夹爪不在组里） */
  const parts = (it.angles || []).map((a, i) => names[i] + ' ' + Math.round(a));
  const g = it.gripper || {};
  if (g.enabled) parts.push('夹爪' + (g.kai_he ? '开' : '合') + (g.degree || 0) + '%');
  return parts.join(' · ');
}

/* ---- 编辑当前预设 ---- */

function armDeleteItem(id) { armDeleteItems([id]); }

/* 删一组（拖到垃圾桶时给整组用）。倒着删, 免得下标在遍历中挪位。
 * 选中集合由 armRenderAll 统一清理（删掉的 id 已经不在了）。 */
function armDeleteItems(ids) {
  const gone = ARM.items.filter(x => ids.includes(x.id));
  if (!gone.length) return;
  armPushUndo();
  for (let i = ARM.items.length - 1; i >= 0; i--) {
    if (ids.includes(ARM.items[i].id)) ARM.items.splice(i, 1);
  }
  armMarkDirty();
  armRenderAll();     /* 里面清掉已不存在的选中 id */
  renderJoints();
  if (gone.length > 1) btlog('✕ 已删除 ' + gone.length + ' 块子积木');
}

function armSnapshot() {
  const p = armCurPreset();
  if (!p) return null;
  return JSON.parse(JSON.stringify(
    { items: p.items, title: p.title, note: p.note, nowait: p.nowait }));
}

/* 在**每次真正改数据之前**压一份快照（makeDnd 的 beforeChange 也指向它） */
function armPushUndo() {
  const s = armSnapshot();
  if (!s) return;
  ARM.undo.push(s);
  if (ARM.undo.length > ARM_UNDO_MAX) ARM.undo.shift();
  ARM.redo.length = 0;
}

function armApplySnapshot(s) {
  const p = armCurPreset();
  if (!p || !s) return;
  p.items = s.items; p.title = s.title; p.note = s.note; p.nowait = s.nowait;
  ARM.dirty = true;
  armRenderAll();
}

function armUndo() {
  if (!ARM.undo.length) return;
  const cur = armSnapshot();
  if (cur) ARM.redo.push(cur);
  armApplySnapshot(ARM.undo.pop());
}

function armRedo() {
  if (!ARM.redo.length) return;
  const cur = armSnapshot();
  if (cur) ARM.undo.push(cur);
  armApplySnapshot(ARM.redo.pop());
}

function armEnsureEdit() {
  if (BT.edit) return true;
  toast('先进入编辑模式：机械臂只在脚本停在握手等待阶段 1 时才接受控制', 'err');
  const b = document.getElementById('arm-edit-on');
  if (b) b.focus();
  return false;
}

/* 「＋ 记录当前位姿」: 把滑杆上现在这组角度收成一条新积木。
 *
 * ⚠️ 刻意**不要求先进编辑模式**（原来要求, 于是没连设备/没进编辑模式时关节区是灰的,
 *    这个按钮也跟着点不动 —— 而「先离线把动作排出来、之后再上台架微调」是常态用法）。
 *    灰态下滑杆里是服务端从源码派生的**固件上电默认位姿**（joints_offline, 不是 0°）,
 *    记下来是安全的; 在线时记的就是设备刚上报的实时角度。
 *    真要动机械臂仍然得进编辑模式 —— 本函数只录数据, 一条指令都不发。 */
/* 新积木插在哪: **选中块的下面**（没选中就追加到末尾）。
 * 多选时插在整个选中组的下面 —— 「在选中的这一组之后」是最自然的理解。
 * 原来一律 push 到末尾: 想在中间补一步, 加完还得把它拖上去。 */
function armInsertIdx() {
  let last = -1;
  ARM.items.forEach((it, i) => { if (ARM.selIds.includes(it.id)) last = i; });
  return last < 0 ? ARM.items.length : last + 1;
}

/* 插入 + 选中它（选中 = 滑杆立刻绑到这一块上, 接着微调就行）。
 * 先把选中态改好**再**渲染 —— armRenderBlock 是照着选中态画 .sel 的。
 * ⚠️ **必须先给 id 再设选中**: armPoseItem() 和「加延时」造出来的块**都没有 id**
 *    （id 一直是 armRenderCanvas 那条兜底补的）, 而兜底在本函数**之后**才跑 ——
 *    补出来的 id 落不进已经写死的 selIds, 于是留下 `selIds = [undefined]`:
 *    症状是新积木根本没被选中（画布上不亮、滑杆退化成「实时设备值」而不是
 *    「编辑「位姿N」的角度」）, 还顺带每次都打一条
 *    「id 缺失或重复, 已临时补上（服务端 bug?）」的**假警告** —— 明明是本地的块。 */
function armInsertItem(it) {
  if (!it.id) it.id = armNewId();
  ARM.items.splice(armInsertIdx(), 0, it);
  ARM.selIds = [it.id];
  ARM.selectedId = it.id;
  armMarkDirty();
  armRenderCanvas();
  renderJoints();
}

function armRecord() {
  if (!armCurPreset()) return;
  armPushUndo();
  const it = armPoseItem('位姿' + (ARM.items.length + 1));
  armInsertItem(it);
  btlog('＋ 记录位姿: ' + it.angles.map(v => v.toFixed(0)).join('/')
        + (BT.edit ? '' : '（当前不在编辑模式, 记的是滑杆上的值）'));
}

function armAddDelay() {
  if (!armCurPreset()) return;
  armPushUndo();
  armInsertItem({ kind: 'delay', tag: '延时', ms: 200, disabled: false, edited: true });
}

/* ---- 预设 ---- */

function armSwitchPreset(idx) {
  if (idx === ARM.activeIdx) return;
  if (ARM.dirty && !confirm('当前动作有未保存的修改，切走会丢掉。继续？')) {
    document.getElementById('arm-preset-select').value = String(ARM.activeIdx);
    return;
  }
  ARM.activeIdx = idx;
  ARM.dirty = false;
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  armSyncItems();
  armBaseCapture();   /* 换了一条动作 → 基线跟着换（否则会拿上一条的块去比） */
  armRenderAll();
}

async function armAddPreset() {
  const title = h('input', { class: 'meta-input', value: '动作' + (ARM.presets.length + 1) });
  const note = h('input', { class: 'meta-input', placeholder: '可选' });
  const body = h('div', {},
    h('label', { class: 'meta-row' }, h('span', {}, '标题'), title),
    h('label', { class: 'meta-row' }, h('span', {}, '备注'), note),
    h('p', { class: 'hint' },
      '标题会显示在「地图」积木库里, 也会写进 app_arm_action.h 的动作枚举尾注释。' +
      '新动作自带一条「当前滑杆位姿」—— C 不允许空数组体, 至少要有一条子积木。'));

  await openTuningModal('新建动作', body, async () => {
    ARM.presets.push({
      id: null,
      title: (title.value || '').trim() || ('动作' + (ARM.presets.length + 1)),
      note: (note.value || '').trim(),
      nowait: false,
      items: [armPoseItem('位姿1')],
    });
    ARM.activeIdx = ARM.presets.length - 1;
    ARM.dirty = true;
    ARM.undo = []; ARM.redo = [];
    armClearSelection();
    armSyncItems();
    armBaseCapture();   /* 新动作自带的那条 = 它的起点, 别一上来就黄着 */
    armRenderAll();
    return true;
  }, '创建');
}

function armDupPreset() {
  const p = armCurPreset();
  if (!p) return;
  const items = JSON.parse(JSON.stringify(p.items)).map(it => ({ ...it, id: armNewId() }));
  ARM.presets.push({
    id: null,
    title: (p.title || '动作') + '-副本',
    note: p.note, nowait: p.nowait, items: items,
  });
  ARM.activeIdx = ARM.presets.length - 1;
  ARM.dirty = true;
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  armSyncItems();
  armBaseCapture();   /* 副本的起点 = 它自己, 别整条新动作全是黄的 */
  armRenderAll();
}

async function armDelPreset() {
  const p = armCurPreset();
  if (!p) return;
  if (ARM.presets.length <= 1) { toast('至少要留一个动作预设', 'err'); return; }
  if (!p.id) {                                   /* 还没存过 → 本地删掉就行 */
    ARM.presets.splice(ARM.activeIdx, 1);
    ARM.activeIdx = Math.max(0, ARM.activeIdx - 1);
    ARM.dirty = true;
    ARM.undo = []; ARM.redo = [];
    armClearSelection();
    armRenderAll();
    return;
  }
  if (ARM.dirty) { toast('有未保存的修改 —— 先保存或切走再删', 'err'); return; }

  const refs = await armRun('refs', { id: p.id }, { quiet: true });
  const n = refs ? (refs.referenced || 0) : 0;
  let msg = '删除动作「' + p.title + '」？\n\n会从 Application/app_arm_action.c 里删掉，' +
            '「地图」积木库里那块也会消失。';
  if (n) {
    msg += '\n\n⚠️ 路线里有 ' + n + ' 步引用了它（' +
           ((refs.where || []).join('、')) + '）——\n那些行会变成编译错误。' +
           '这是故意的（免得静默指向别的动作），但你要去把那些步骤换成别的动作。';
  }
  msg += '\n\n（会先在回收站留一份，可以恢复）';
  if (!confirm(msg)) return;

  const data = await armRun('delete', { id: p.id, base_version: ARM.version });
  if (data === null) return;
  toast('已删除，可在回收站恢复', 'ok');
  await armReload();
  await reloadCatalog();
}

/* ---- 子积木右键菜单: 复制 / 移动到别的动作预设 ----
 *
 * 作用对象是**整个选中组**（Ctrl+点击多选出来的那几块; 单选时组里就那一块）。
 * 「复制」/「移动」都**追加到目标预设的末尾** —— 目标预设的画布此刻没显示出来,
 * 让用户挑"插在第几步"是没法确认落点的（看不见）。要精调位置就去目标动作里拖,
 * 那条路能看见落点。
 *
 * ⚠️ 跨预设的改动**清空撤回/重做**: 撤回快照只存当前预设的条目, 撤回来会让被搬走的
 *    块"在两边各有一份" —— 那是个会骗人的撤回按钮。与「预设的增删不走撤回栈,
 *    那条路由回收站兜」是同一条纪律。
 * ⚠️ 搬过去的条目**一律换新 id**: id 只要在一个预设内唯一即可, 但两个预设的 id 很
 *    可能重号（都是从 map.c 解析出来的 i0/i1…）, 原样搬过去就成了「一个预设里两块
 *    同 id」——症状见 armRenderCanvas 顶部那段（点一块选中全部 / 拖动不让位 /
 *    删除永远删第一块）。
 */
function armTransferSelection(targetIdx, mode) {
  const picked = ARM.items.filter(it => ARM.selIds.includes(it.id));
  const dst = ARM.presets[targetIdx];
  if (!picked.length || !dst) return;
  const n = picked.length;
  const same = targetIdx === ARM.activeIdx;

  if (mode === 'move' && same) { toast('这些块已经在当前动作里了'); return; }
  if (mode === 'move' && picked.length >= ARM.items.length) {
    toast('不能把整条动作搬空 —— 源动作会变成空数组, C 编译不过', 'err');
    return;
  }

  const copies = picked.map(it => {
    const c = JSON.parse(JSON.stringify(it));
    c.id = armNewId();
    return c;
  });

  /* 在本动作里复制: 与「记录当前位姿」同一个落点规则（选中的那块下面）, 走正常撤回 */
  if (mode === 'copy' && same) {
    armPushUndo();
    ARM.items.splice(armInsertIdx(), 0, ...copies);
    armSetSelection(copies.map(c => c.id));
    armMarkDirty();
    armRenderCanvas();
    renderJoints();
    btlog('⧉ 已在本动作里复制 ' + n + ' 块');
    return;
  }

  const name = dst.title || ('动作' + (targetIdx + 1));
  if (mode === 'move') {
    /* 倒着删, 免得下标在遍历中挪位 */
    for (let i = ARM.items.length - 1; i >= 0; i--) {
      if (ARM.selIds.includes(ARM.items[i].id)) ARM.items.splice(i, 1);
    }
  }
  dst.items.push(...copies);
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  armMarkDirty();
  armRenderAll();      /* 里面清掉已经不在这条动作里的选中 id */
  renderJoints();
  btlog((mode === 'copy' ? '⧉ 复制 ' : '➜ 移动 ') + n + ' 块 → 「' + name + '」的末尾'
        + '（跨动作的改动不进撤回栈）');
  toast((mode === 'copy' ? '已复制 ' : '已移动 ') + n + ' 块到「' + name + '」', 'ok');
}

let armCtxEl = null;      /* 右键菜单元素（引用存变量, **不用** getElementById） */

function closeArmCtx() {
  if (armCtxEl && armCtxEl.parentNode) armCtxEl.parentNode.removeChild(armCtxEl);
  armCtxEl = null;
}

/* 页面滚动 → 关掉右键菜单（挂载见 init, 用**捕获**阶段）。
 * ⚠️ 但**菜单自己滚不算**: 预设一多, 「复制到… / 移动到…」第二屏就撑出滚动条
 *    （`.ctxmenu` 是 `max-height:70vh; overflow-y:auto`）, 而 scroll 事件虽然**不冒泡**,
 *    却照样走**捕获阶段** —— window 上这条捕获监听收得到菜单内部的滚动, 于是
 *    「滚一下目标列表 → 菜单当场消失, 目标根本够不着」。target 落在菜单里就放过。
 *    （滚到列表尽头之后的「滚动链」另由 `.ctxmenu` 的 `overscroll-behavior: contain` 挡住,
 *      否则会接着滚动画布 = 页面滚动 = 照样关。） */
function armCtxOnScroll(e) {
  if (armCtxEl && e.target && armCtxEl.contains(e.target)) return;
  closeArmCtx();
}

/* 贴边时把菜单翻到光标另一侧（否则会跑出视口看不见）。内容换了一屏要再夹一次。 */
function armCtxClamp(el) {
  const x = parseFloat(el.dataset.ax), y = parseFloat(el.dataset.ay);
  const r = el.getBoundingClientRect();
  el.style.left = (x + r.width > window.innerWidth - 8 ? Math.max(8, x - r.width) : x) + 'px';
  el.style.top = (y + r.height > window.innerHeight - 8 ? Math.max(8, y - r.height) : y) + 'px';
}

/* ⚠️ 菜单的 DOM 由 JS 现搭、引用存在 armCtxEl 里 —— 千万不要改用 getElementById:
 *    那个字面量会被 test_bt 的「app.js 里写死的 id 必须在 index.html 里」那条契约
 *    检查拦下（菜单是动态的, 不该往 index.html 里塞静态 DOM）。 */
function openArmCtx(x, y) {
  closeArmCtx();
  const el = h('div', { class: 'ctxmenu' });
  el.dataset.ax = String(x);
  el.dataset.ay = String(y);
  el.style.left = x + 'px';
  el.style.top = y + 'px';
  el.appendChild(h('div', { class: 'ctx-cap' }, '选中 ' + ARM.selIds.length + ' 块子积木'));
  el.appendChild(h('button', {
    class: 'ctxitem', onclick: () => armCtxTargets(el, 'copy'),
  }, '⧉ 复制到…'));
  el.appendChild(h('button', {
    class: 'ctxitem', onclick: () => armCtxTargets(el, 'move'),
  }, '➜ 移动到…'));
  document.body.appendChild(el);
  armCtxEl = el;
  armCtxClamp(el);
}

/* 第二屏: 目标预设列表 —— 就地换掉菜单内容（不再弹一个对话框, 少一层上下文切换） */
function armCtxTargets(el, mode) {
  el.textContent = '';
  el.appendChild(h('div', { class: 'ctx-cap' },
    (mode === 'copy' ? '⧉ 复制 ' : '➜ 移动 ') + ARM.selIds.length + ' 块到…'));
  let n = 0;
  ARM.presets.forEach((p, i) => {
    const same = i === ARM.activeIdx;
    if (mode === 'move' && same) return;        /* 移到本动作 = 什么都没发生 */
    n++;
    el.appendChild(h('button', {
      class: 'ctxitem', title: same ? '插在选中块的下面' : '追加到那条动作的末尾',
      onclick: () => { closeArmCtx(); armTransferSelection(i, mode); },
    }, (p.title || ('动作' + (i + 1)))
       + (same ? '（当前 · 插在选中块下面）' : ' · ' + p.items.length + ' 条')));
  });
  if (!n) {
    el.appendChild(h('div', { class: 'ctx-cap' },
      '没有别的动作预设 —— 用顶栏的「＋ 新建」建一个再来搬'));
  }
  el.appendChild(h('button', {
    class: 'ctxitem ctx-back', onclick: closeArmCtx,
  }, '取消'));
  armCtxClamp(el);
}

/* ---- 回收站 ---- */

function openArmTrash() {
  const body = h('div', {});
  const render = () => {
    body.textContent = '';
    if (!ARM.trash.length) {
      body.appendChild(h('p', { class: 'hint' }, '回收站是空的。'));
      return;
    }
    for (const t of ARM.trash) {
      body.appendChild(h('div', { class: 'arm-trash-row' },
        h('div', { class: 'arm-trash-name' }, t.title || '(无标题)'),
        h('span', { class: 'hint' },
          (t.note ? t.note + ' · ' : '') + ((t.items || []).length) + ' 条 · ' +
          (t.deleted_at || '')),
        h('button', {
          class: 'btn ghost tiny',
          onclick: async () => {
            const d = await armRun('restore', { trash_id: t.trash_id, base_version: ARM.version });
            if (d === null) return;
            await armReload();
            render();
            await reloadCatalog();
            toast('已恢复为「' + (d.title || '') + '」', 'ok');
          },
        }, '恢复'),
        h('button', {
          class: 'btn ghost tiny',
          onclick: async () => {
            if (!confirm('彻底删除「' + t.title + '」？这一步不可再恢复。')) return;
            const d = await armRun('purge', { trash_id: t.trash_id });
            if (d === null) return;
            ARM.trash = d.trash || [];
            render();
          },
        }, '彻底删除')));
    }
  };
  render();
  /* 保存按钮当「关闭」用: onSave 直接返回 true 走「成功即关闭」那条路 */
  openTuningModal('回收站（' + ARM.trash.length + '）', body, async () => true, '关闭');
}

/* ---- 保存 ---- */

let _savingArm = false;
async function armSave() {
  if (_savingArm) return;      /* 防重入（同 save） */
  _savingArm = true;
  try {
    await armSaveInner();
  } finally {
    _savingArm = false;
  }
}

async function armSaveInner() {
  const p = armCurPreset();
  const idx = ARM.activeIdx;
  const data = await armRun('save', {
    base_version: ARM.version,
    presets: ARM.presets.map(x => ({
      id: x.id || null, title: x.title, note: x.note,
      nowait: !!x.nowait, items: x.items,
    })),
  });
  if (data === null) return;

  /* 服务端回的是**重新解析过**的预设（带回原始物理行、edited=false），
   * 直接换上 —— 这样紧接着再保存一次就是字节零 diff。 */
  ARM.version = data.version;
  ARM.presets = data.presets || ARM.presets;
  ARM.activeIdx = Math.min(idx, ARM.presets.length - 1);
  ARM.dirty = false;
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  armSyncItems();
  armBaseCapture();          /* 已落盘, 现在的样子就是新的「原值」（先对齐再渲染, 见 armReload） */
  armRenderAll();

  /* reloadCatalog 单独兜底: 保存**已经成功**, 它挂了不该表现为「保存失败」 */
  try { await reloadCatalog(); } catch (e) { btlog('⚠ 刷新积木库失败: ' + e.message, 'bad'); }
  /* 积木库里那块大积木跟着更新（地图 Tab 的未保存编辑不受影响） */
  btlog('✔ 已写入 Application/app_arm_action.c（备份 .bak） — 需重编译固件才能在车上跑');
  toast('已保存。需重编译固件才能在车上跑', 'ok');
}

/* ---- 播放 ---- */

async function armPlayItem(it) {
  if (!armEnsureEdit()) return;
  if (it.kind !== 'pose') { toast('这一条不是位姿，没有可播的东西', 'err'); return; }
  /* 禁用不影响**单条**试跑: ▶ 是「我就想看看这一条」的显式动作, 照播, 只提一句。 */
  if (it.disabled) btlog('（这一条已被禁用 —— 路线里不会执行, 这里只是试跑）');
  const g = it.gripper || {};
  const data = await btRun('arm', {
    angles: it.angles,
    kai_he: g.enabled ? g.kai_he : BT.grip.kai_he,
    degree: g.enabled ? g.degree : BT.grip.degree,
  }, { quiet: true });
  if (data === null) return;
  if (!await btWaitDone(6000)) {
    btlog('⚠ 等待到位超时（6s）—— 臂可能还没停稳', 'bad');
  }
}

/* PC 编排: 逐条 $B,ARM（5 轴同时启 S 曲线）+ 延时块用 PC 侧等待。
 * 为什么不让固件播: 这样「还没保存的编辑」也能立刻试 —— 屏幕上是啥就播啥。
 * 被眼睛禁用的条目**跳过**（眼睛的语义就是「这一步不执行」, 固件那边也一样）。 */
async function armPlayAll() {
  if (!armEnsureEdit()) return;
  if (!ARM.items.length) { toast('动作是空的', 'err'); return; }
  const live = ARM.items.filter(x => !x.disabled);
  if (!live.length) { toast('所有子积木都被禁用了 —— 没有可播的', 'err'); return; }
  if (live.length < ARM.items.length) {
    btlog('（跳过 ' + (ARM.items.length - live.length) + ' 条被禁用的）');
  }
  BT.playing = true;
  btlog('▶ 播放整个序列（' + live.length + ' 条）…');
  try {
    for (let i = 0; i < live.length; i++) {
      const it = live[i];
      if (it.kind === 'delay') {
        await new Promise(r => setTimeout(r, Math.max(0, it.ms | 0)));
        continue;
      }
      if (it.kind !== 'pose') continue;
      const g = it.gripper || {};
      const ok = await btRun('arm', {
        angles: it.angles,
        kai_he: g.enabled ? g.kai_he : BT.grip.kai_he,
        degree: g.enabled ? g.degree : BT.grip.degree,
      }, { quiet: true, silent: true });
      if (ok === null) { btlog('✗ 中断: 设备不再接受点动', 'bad'); return; }
      /* 每条都**等到真的停稳**再发下一条（$B,POS 的 done 位 = 六轴全部到位）。
       * 超时也别装作没事: 直接发下一条就是我们以前看到的「动作提前播放」。 */
      if (await btWaitDone(6000)) {
        btlog('  第 ' + (i + 1) + '/' + live.length + ' 条到位');
      } else {
        btlog('  ⚠ 第 ' + (i + 1) + '/' + live.length
              + ' 条等待到位超时（6s）—— 下一条会提前发, 臂可能还没停稳', 'bad');
      }
    }
    btlog('✓ 播放结束');
  } finally {
    BT.playing = false;
  }
}

/* 让**固件**播它自己编译进去的那一份 —— 与路线里的 STEP_ARM_PLAY_WAIT 是同一个
 * 函数。这是唯一能验证「保存→编译→车上真跑」这条链的办法（PC 编排验证的是 PC 的
 * 数据, 不是固件里的那份）。 */
async function armPlayDevice() {
  const p = armCurPreset();
  if (!p || !p.id) { toast('这个动作还没保存 —— 固件里没有它', 'err'); return; }
  if (ARM.dirty) { toast('有未保存的修改：固件里那份是旧的。先保存，或改用「播放整个序列」', 'err'); return; }
  if (!armEnsureEdit()) return;
  BT.playing = true;
  btlog('▶ 让固件播放动作 ' + p.id + '（整段含表内延时，可能要几秒）…');
  try {
    const d = await btRun('play', { id: p.id }, { quiet: true });
    if (d === null) return;
    btlog('✓ 整段播放完成');
  } finally {
    BT.playing = false;
  }
}

/* 「地图」Tab 里那块大积木的 ✎ / 右键 → 跳到这里编辑来源预设 */
async function armOpenPreset(id) {
  switchTab('arm');
  /* 必须等**加载 promise 本体**: ARM.loaded 在入口就同步置 true, 只判它然后直接
   * findIndex 会在还没回来的空 presets 上必得 -1 —— 首次点击弹假错误 */
  if (!ARM.loaded) await (ARM.loading || loadArm());
  const idx = ARM.presets.findIndex(p => p.id === id);
  if (idx < 0) {
    toast('这个动作不在此工程里（别的分支存的？先重新加载）', 'err');
    return;
  }
  ARM.activeIdx = idx;
  ARM.dirty = false;
  ARM.undo = []; ARM.redo = [];
  armClearSelection();
  armSyncItems();
  armBaseCapture();   /* 刚跳到这条动作, 基线 = 它现在的样子 */
  armRenderAll();
}

/* ---- 连接状态低频自检 ----
 * 原来只在 连接/断开/刷新 之后才同步, 于是空闲时拔线界面不会自己变。
 * 只刷连接态与日志, **不重建串口下拉** —— 重建会把用户还没连接就选好的口冲掉。 */
async function btPollConn() {
  if (!linkTabVisible()) return;
  try {
    const st = await (await fetch('/api/bt')).json();
    const was = BT.connected;
    BT.available = !!st.available;
    BT.connected = !!st.connected;
    for (const p of CONN_PANELS) renderConn(st, p);
    btReplayLog(st.log);
    if (was && !BT.connected) {
      btSetEdit(false);
      btlog('链路已断开', 'bad');
    }
  } catch (e) { /* 服务端没起来时静默, 下次再试 */ }
}

/* 「机械臂」画布的拖拽实例。
 * ⚠️ blockId 用 blk-arm- 前缀: 两个画布的 DOM 同时存在于文档里, 都用 blk- 的话
 *    getElementById 会跨画布取错元素。
 * allowCopy=false: 同一条动作里复制一份位姿没有意义（要复用就直接再记一次）。 */
const ARMDND = makeDnd({
  innerEl: () => document.getElementById('arm-canvas-inner'),
  canvasEl: () => document.getElementById('arm-canvas'),
  trashEl: () => document.getElementById('arm-trash'),
  blockId: (id) => 'blk-arm-' + id,
  items: () => ARM.items,
  zonesMeta: () => ARM.zonesMeta,
  createItem: () => armPoseItem('位姿'),
  newId: armNewId,
  catalogOf: () => null,          /* 机械臂画布没有积木库, 不存在「拖入新块」 */
  render: armRenderCanvas,
  select: armSelect,
  selectMany: armSetSelection,    /* 整组落座后保持整组选中 */
  deleteItem: armDeleteItem,
  deleteItems: armDeleteItems,    /* 整组拖进垃圾桶 = 一次删光, 不是只删第一块 */
  dragSet: armDragSet,            /* Ctrl+多选出来的一组, 拖一块 = 整组一起搬 */
  /* ⚠️ 必须连 armMarkDirty 一起调: 拖动重排**改变条目顺序**, 是要保存的改动, 而
   * makeDnd 的 beforeChange 只负责压撤回栈。少了这半句, 拖完一块再换动作/重载,
   * 顺序改动就无声丢了 —— 而且界面还显示「没改过」。 */
  beforeChange: () => { armPushUndo(); armMarkDirty(); },
  allowCopy: false,
});

/* 拖这一块时到底搬走哪些: **拖的是选中组里的一块** → 整组; 否则就它自己
 * （顺手把选中换成它 —— 用户拖的是它, 选中态跟上, 免得"拖完选中还停在别处"）。
 * ⚠️ 这个函数只在**过了 6px 阈值、真开始拖**时才被调（见 makeDnd::startDrag）,
 *    所以单纯的按下/点击不会改选中。 */
function armDragSet(id) {
  if (ARM.selIds.includes(id)) return ARM.selIds.slice();
  armSelect(id);
  return [id];
}

/* ---- 关节滑杆共用的小工具 ---- */

function btCurrentAngles() {
  const angles = [];
  for (const rng of jointRanges()) {
    angles[parseInt(rng.dataset.joint, 10)] = parseFloat(rng.value);
  }
  for (let i = 0; i < 5; i++) if (typeof angles[i] !== 'number') angles[i] = 0;
  return angles.slice(0, 5);
}

/* 等设备把 S 曲线走完（$B,POS 的 done 位）。回 null = 掉出编辑模式, 立刻收手。 */
async function btWaitDone(timeoutMs) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    const pos = await btRun('pos', {}, { quiet: true, silent: true });
    if (pos === null) return false;
    if (pos.done) return true;
    await new Promise(r => setTimeout(r, 120));
  }
  return false;
}

/* ==================== Tab: 底盘动作组（时间轴速度混合, 2026-10 加） ====================
 * 布局仿「机械臂」Tab: 顶栏选组/改名/保存; 左边是条目表 + 只读时间轴条;
 * 右边连接 + 编辑模式 + ▶预览/⏹停止（走 $B,CPLAY/$B,CSTOP, 非阻塞, 轮询 ch_busy）。
 * 一期是**表格式**编辑（拖拽时间轴在二期）; 时间轴条只读, 但叠加关系一眼可见。 */

const CHACT = {
  loaded: false,
  version: null,        // {c, h} mtime, 保存时回传做冲突检测
  groups: [],           // [{id, title, note, items:[...]}]（id 为 null = 还没存过）
  activeIdx: 0,
  trash: [],
  armOptions: [],       // [{id, title}] —— 臂事件下拉（服务端从 arm_action 派生）
  consts: { loop_ms: 20, time_scale: 1.0, pulse_per_mm_y: 10.3, items_max: 16 },
  tune: {},             // {参数名: 设备当前值} —— 调参卡; 空 = 还没从设备回读过
  dirty: false,
  seq: 1,
  busyTimer: null,      // 预览后的 ch_busy 轮询句柄
};

const CHACT_KINDS = [
  ['move', '平移段'], ['turn', '转弯段'], ['pivot', '支点转弯'],
  ['arm', '臂动作'], ['arm_wait', '等臂空闲'],
];

function chactCur() { return CHACT.groups[CHACT.activeIdx] || null; }
function chactNewId() { return 'c' + (CHACT.seq++); }
function chactMarkDirty() { CHACT.dirty = true; chactRenderBar(); }

/* 字段被 UI 改过 → 必须置 edited: 保存时该条才按**当前字段**重新生成 C 行。
 * ⚠️ edited=false 的条目走「原始物理行回吐」（零 diff 机制）—— 漏置 edited 的
 * 症状正是「界面上改了、一保存就被旧版盖回去」: 改动只活在内存字段里,
 * 序列化时被原始行整个顶掉。所以每个改渲染字段的 handler 都必须走本函数。
 * （眼睛/上下移/删除不在此列: 眼睛是对原始行加 // 前缀, 挪动/删除动的是数组
 * 顺序与成员, 都不需要重渲染该行。） */
function chactTouch(it) {
  it.edited = true;
  chactMarkDirty();
}

function chactKindName(kind) {
  const f = CHACT_KINDS.find(x => x[0] === kind);
  return f ? f[1] : '只读';
}

/* 固件 fChassisMoveTimeEst 的 JS 镜像（AXIS_Y 档）—— 排版显示平移段时长。
 * ⚠️ 与 chassis_action.py::estimate_ms / 固件公式三处必须同源; 实际播放时长固件用
 *    蓝牙运行时 ch_time_scale 现算, 这里用宏默认（界面提示里写明了这一点）。 */
function chactEstimateMs(dist, accel, speed) {
  const k = Number(CHACT.consts.time_scale) || 1.0;
  const ppm = Number(CHACT.consts.pulse_per_mm_y) || 10.3;
  if (!speed) return 200;
  if (!accel) accel = 256;
  const d = Math.abs(dist);
  if (d <= 2.0) return 200;
  const q = d * ppm;
  const qf = 320.0 * speed * speed / (accel * 6.0);
  const tf = 2000.0 * speed / accel;
  const ms = (qf >= q) ? tf * Math.sqrt(q / qf)
                       : 1000.0 * (q - qf) / (320.0 * speed / 6.0) + tf;
  return ms * k;
}

/* 一条条目的占用时长: 平移段现算, 等臂 = 表值; **转弯段没有时长**（转到位为止,
 * 时间轴上只画起点标记 ▸, 总时长不含它 —— 组尾会等收敛）。 */
function chactItemDur(it) {
  if (it.kind === 'move') {
    const d = Math.hypot(Number(it.wx) || 0, Number(it.wy) || 0);
    return chactEstimateMs(d, Number(it.accel) || 0, Number(it.speed) || 0);
  }
  if (it.kind === 'arm_wait') return Number(it.dur) || 0;
  return 0;   /* 臂事件瞬时; 转弯无段上限 */
}

function chactTotalMs(g) {
  /* 只统计会播的条目（禁用/手写行不占时间轴刻度）; 转弯段无时长（转到位为止） */
  let t = 0;
  for (const it of (g && g.items) || []) {
    if (it.disabled || it.kind === 'raw') continue;
    t = Math.max(t, (Number(it.start) || 0) + chactItemDur(it));
  }
  return t;
}

/* ---- 请求 ---- */

async function chactCall(action, extra) {
  const res = await fetch('/api/chact', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ action: action }, extra || {})),
  });
  if (!res.ok) {
    const j = await res.json().catch(() => ({}));
    const e = new Error(j.message || j.error || ('HTTP ' + res.status));
    e.status = res.status;
    throw e;
  }
  return res.json();
}

async function loadChact() {
  CHACT.loaded = true;
  try {
    const res = await fetch('/api/chact');
    const st = await res.json();
    chactApplyState(st);
  } catch (e) {
    toast('读取底盘动作组失败: ' + e.message, 'err');
  }
  chactRenderAll();
}

function chactApplyState(st) {
  if (st.error) {
    toast('app_chassis_action 解析失败: ' + st.error, 'err');
    return;
  }
  CHACT.version = st.version;
  CHACT.groups = st.groups || [];
  CHACT.trash = st.trash || [];
  CHACT.armOptions = st.arm_options || [];
  if (st.consts) CHACT.consts = st.consts;
  CHACT.next_id = st.next_id || 1;
  if (CHACT.activeIdx >= CHACT.groups.length) CHACT.activeIdx = 0;
  CHACT.dirty = false;
  /* ⚠️ 调参卡必须在这重渲染: loadChact 是异步的, chactEnter 里同步的那次
   * chactTuneRender 会跑在 consts 到货之前 —— 离线默认值全显示成 0（实测踩过）。 */
  chactTuneRender();
}

function chactRenderAll() {
  chactRenderBar();
  chactRenderItems();
  chactRenderTimeline();
  chactRenderTrash();
}

/* ---- 顶栏 ---- */

function chactRenderBar() {
  const sel = document.getElementById('chact-group-select');
  sel.textContent = '';
  CHACT.groups.forEach((g, i) => {
    sel.appendChild(h('option', { value: i },
      (g.title || ('动作组' + (i + 1))) + (g.id ? '' : ' · 未保存')));
  });
  if (CHACT.groups.length) sel.value = String(CHACT.activeIdx);

  /* 与 armRenderBar 同一条纪律: 输入框正被聚焦时别回灌 value（会把光标顶飞） */
  const g = chactCur();
  const t = document.getElementById('chact-title');
  const n = document.getElementById('chact-note');
  if (document.activeElement !== t) t.value = g ? (g.title || '') : '';
  if (document.activeElement !== n) n.value = g ? (g.note || '') : '';
  document.getElementById('chact-save').textContent =
    CHACT.dirty ? '保存动作组 *' : '保存动作组';
  document.getElementById('chact-trash-open').textContent =
    CHACT.trash.length ? ('⬆ 回收站(' + CHACT.trash.length + ')') : '⬆ 回收站';
}

function chactSwitchGroup(idx) {
  if (CHACT.dirty && !confirm('当前动作组有未保存的修改，切换会丢掉。继续？')) return;
  CHACT.activeIdx = Math.max(0, Math.min(idx, CHACT.groups.length - 1));
  CHACT.dirty = false;
  chactRenderAll();
}

function chactBlankGroup() {
  return { id: null, title: '', note: '', items: [
    { id: chactNewId(), kind: 'move', start: 0, wx: 0, wy: 300, speed: 150,
      accel: 120, hold: true, tag: '前进', disabled: false, edited: true },
  ] };
}

function chactGroupAdd() {
  if (CHACT.dirty && !confirm('当前动作组有未保存的修改，新建会丢掉。继续？')) return;
  CHACT.groups.push(chactBlankGroup());
  CHACT.activeIdx = CHACT.groups.length - 1;
  CHACT.dirty = true;
  chactRenderAll();
}

function chactGroupDup() {
  const g = chactCur();
  if (!g) return;
  const copy = JSON.parse(JSON.stringify(g));
  copy.id = null;
  copy.title = (g.title || '动作组') + '副本';
  copy.items.forEach(it => { it.id = chactNewId(); it.edited = true; });
  CHACT.groups.push(copy);
  CHACT.activeIdx = CHACT.groups.length - 1;
  CHACT.dirty = true;
  chactRenderAll();
}

async function chactGroupDel() {
  const g = chactCur();
  if (!g) return;
  if (!g.id) {           /* 还没保存过的: 直接撤掉, 不进回收站 */
    CHACT.groups.splice(CHACT.activeIdx, 1);
    if (!CHACT.groups.length) CHACT.groups.push(chactBlankGroup());
    CHACT.activeIdx = Math.min(CHACT.activeIdx, CHACT.groups.length - 1);
    CHACT.dirty = true;
    chactRenderAll();
    return;
  }
  let msg = '删除「' + (g.title || g.id) + '」？';
  try {
    const r = await chactCall('refs', { id: g.id });
    const n = (r.data && r.data.referenced) || 0;
    if (n) msg += '\n\n⚠ map.c 里有 ' + n + ' 步引用它（'
                  + (r.data.where || []).join('、') + '）,\n删除后那些行会编译报错。';
  } catch (e) { /* 引用数查不到不拦删除, 只是少一层提示 */ }
  if (!confirm(msg + '\n（会先在回收站留一份）')) return;
  try {
    const r = await chactCall('delete', { id: g.id, base_version: CHACT.version });
    if (!r.ok) throw new Error(r.error || r.data && r.data.message || '删除失败');
    /* ⚠️ 删除响应只带 version/trash_id, **不带 groups** —— 拿它赋值会把整张组表
     * 清成空数组（现象: 删完一条, 界面上所有组都消失）。重新拉一次全量状态。 */
    const st = await (await fetch('/api/chact')).json();
    chactApplyState(st);
    chactRenderAll();
    toast('已删除（回收站可恢复）');
  } catch (e) {
    toast(e.message, 'err');
  }
}

let _savingChact = false;
async function chactSave() {
  if (_savingChact) return;    /* 防重入（同 save） */
  _savingChact = true;
  const g0 = chactCur();
  if (g0) {   /* 顶栏的标题/备注输入框值要先进 state（渲染时是反向灌的） */
    g0.title = document.getElementById('chact-title').value;
    g0.note = document.getElementById('chact-note').value;
  }
  try {
    const r = await chactCall('save',
                              { groups: CHACT.groups, base_version: CHACT.version });
    if (!r.ok) throw new Error(r.error || r.data && r.data.message || '保存失败');
    CHACT.version = r.data.version;
    CHACT.groups = r.data.groups;
    CHACT.dirty = false;
    chactRenderAll();
    toast(r.data.changed ? '已写回 app_chassis_action.c/.h（记得上 Keil 重编译）'
                         : '内容没变, 未写盘');
  } catch (e) {
    toast(e.message, 'err');
  } finally {
    _savingChact = false;
  }
}

async function chactReload() {
  if (CHACT.dirty && !confirm('有未保存修改，重载将丢弃，继续？')) return;
  CHACT.dirty = false;
  await loadChact();
}

/* ---- 条目表 ---- */

function chactNumInput(val, onChange, opts) {
  opts = opts || {};
  const inp = h('input', {
    class: 'chact-num', type: 'number',
    value: val, step: opts.step || 20, min: opts.min, max: opts.max,
    title: opts.title || '',
  });
  inp.addEventListener('change', () => onChange(inp.value));
  return inp;
}

function chactSnap(v) {
  const loop = Number(CHACT.consts.loop_ms) || 20;
  return Math.max(0, Math.round((Number(v) || 0) / loop) * loop);
}

function chactRenderItems() {
  const box = document.getElementById('chact-items');
  box.textContent = '';
  const g = chactCur();
  if (!g) {
    box.appendChild(h('p', { class: 'hint' }, '（没有动作组）'));
    return;
  }

  const table = h('table', { class: 'chact-table' });
  const thead = h('tr', {},
    h('th', { title: '禁用 = 行注释掉, 不进数组' }, '启'),
    h('th', {}, '类型'),
    h('th', { title: '时间轴起点, 吸附 ' + CHACT.consts.loop_ms + 'ms 网格' }, '起点ms'),
    h('th', { class: 'chact-params' }, '参数'),
    h('th', { title: '平移段: 时长=估时×k 现算; 其余=表值' }, '时长ms'),
    h('th', {}, '备注'),
    h('th', {}, '操作'));
  table.appendChild(thead);

  g.items.forEach((it, idx) => {
    const row = h('tr', { class: it.disabled ? 'chact-off' : '' });

    /* 眼睛 */
    row.appendChild(h('td', {}, h('input', {
      type: 'checkbox', checked: !it.disabled,
      onchange: e => { it.disabled = !e.target.checked; chactMarkDirty(); chactRenderItems(); chactRenderTimeline(); },
    })));

    /* 类型（已有序目可换类型: 换了参数回默认, 免得残留旧字段） */
    const kindSel = h('select', { class: 'chact-kind' },
      ...CHACT_KINDS.map(([k, name]) => h('option', { value: k }, name)));
    kindSel.value = it.kind;
    kindSel.addEventListener('change', () => {
      chactReplaceKind(it, kindSel.value);
      chactMarkDirty();
      chactRenderItems();
      chactRenderTimeline();
    });
    row.appendChild(h('td', {}, it.kind === 'raw'
      ? h('span', { class: 'hint', title: (it.lines || []).join('\n') }, '只读行')
      : kindSel));

    /* 起点 */
    row.appendChild(h('td', {}, it.kind === 'raw' ? '' :
      chactNumInput(it.start, v => {
        it.start = chactSnap(v); chactTouch(it); chactRenderItems(); chactRenderTimeline();
      }, { step: CHACT.consts.loop_ms, min: 0 })));

    /* 参数（按类型） */
    const pcell = h('td', { class: 'chact-params' });
    if (it.kind === 'move') {
      pcell.appendChild(chactNumInput(it.wx, v => { it.wx = Number(v) || 0; chactTouch(it); chactRenderTimeline(); }, { step: 10, title: '车体位移X mm（正=车头左方）' }));
      pcell.appendChild(document.createTextNode(' X '));
      pcell.appendChild(chactNumInput(it.wy, v => { it.wy = Number(v) || 0; chactTouch(it); chactRenderTimeline(); }, { step: 10, title: '车体位移Y mm（正=车头前方; 过弯叠平移时随车头旋转）' }));
      pcell.appendChild(document.createTextNode(' mm  速'));
      pcell.appendChild(chactNumInput(it.speed, v => { it.speed = Number(v) || 0; chactTouch(it); chactRenderTimeline(); }, { step: 10, min: 1, max: 500, title: 'rpm' }));
      pcell.appendChild(document.createTextNode(' 加'));
      pcell.appendChild(chactNumInput(it.accel, v => { it.accel = Number(v) || 0; chactTouch(it); chactRenderTimeline(); }, { step: 10, min: 1, max: 255, title: 'EMM 加速度档 1~255' }));
      const hold = h('input', { type: 'checkbox', checked: !!it.hold, title: '航向保持: 陀螺咬住世界航向（默认开）' });
      hold.addEventListener('change', e => { it.hold = e.target.checked; chactTouch(it); });
      pcell.appendChild(hold);
      pcell.appendChild(document.createTextNode('保持'));
    } else if (it.kind === 'turn' || it.kind === 'pivot') {
      /* 角度模式（2026-10 加）: 相对 = 从当前航向转 delta（叠加平移 = 弧线继续转）;
       * 绝对 = 直接咬 delta 这个世界航向（前面段把记账带偏后拉回已知朝向）。
       * 两档选项都锁在 90° 网格上; 相对不许 0, 绝对允许 0（咬回开机朝向）。
       * pivot（支点转弯）与 turn 同形同义, 只是驱动方式是单侧+陀螺闭环。 */
      const msel = h('select', { class: 'chact-kind',
        title: '相对=从当前航向再转这么多（转弯叠平移就是弧线）; 绝对=直接转到这个航向' },
        h('option', { value: '0' }, '相对'),
        h('option', { value: '1' }, '绝对'));
      msel.value = it.abs ? '1' : '0';
      msel.addEventListener('change', () => {
        it.abs = msel.value === '1' ? 1 : 0;
        if (!it.abs && !Number(it.delta)) it.delta = -90;   // 相对不许 0, 给个合法默认
        chactTouch(it);
        chactRenderItems();     // 重建: 角度下拉的选项集随模式换
      });
      pcell.appendChild(msel);
      const dsel = h('select', { title: it.abs
        ? '目标世界航向（绝对, 90° 网格; 零点=开机朝向）'
        : '转角（相对, ±90 的倍数; 世界航向是 90° 网格的记账器）' });
      const opts = it.abs
        ? [[0, '咬 0°(开机朝向)'], [90, '咬 +90°'], [-90, '咬 -90°'],
           [180, '咬 180°'], [-180, '咬 -180°']]
        : [[90, '左转 90'], [-90, '右转 90'], [180, '左转 180'], [-180, '右转 180']];
      for (const [v, label] of opts) {
        dsel.appendChild(h('option', { value: v }, label));
      }
      /* 手改出来的非选项值（如旧数据）也保得住, 不许 select 静默吃掉 */
      if (![...dsel.options].some(o => Number(o.value) === Number(it.delta))) {
        dsel.appendChild(h('option', { value: it.delta }, '咬 ' + it.delta + '°'));
      }
      dsel.value = String(it.delta);
      dsel.addEventListener('change', () => { it.delta = Number(dsel.value); chactTouch(it); });
      pcell.appendChild(dsel);
      if (it.kind === 'turn') {
        pcell.appendChild(document.createTextNode(' 加'));
        pcell.appendChild(chactNumInput(it.accel, v => { it.accel = Number(v) || 0; chactTouch(it); }, { step: 10, min: 0, max: 255, title: '本段 EMM 加速度档 1~255; 0=自动（沿用平移段/全局档）。转弯的斜坡平滑是打滑命门 —— 摆振/甩尾就把它调小' }));
      }
      pcell.appendChild(document.createTextNode(' w≤'));
      pcell.appendChild(chactNumInput(it.speed, v => { it.speed = Number(v) || 0; chactTouch(it); }, { step: 10, min: 0, max: 255, title: it.kind === 'pivot'
        ? '本段驱动侧轮速限幅 rpm（>0 时盖过全局 ch_pivot_w_max; 0=用全局, 出厂 130=现场标定值）'
        : '本段旋转限幅 rpm（>0 时盖过全局 ch_w_max —— 抬它立刻看得见; 0=用全局 ch_w_max）' }));
      pcell.appendChild(document.createTextNode(' （转到位为止）'));
    } else if (it.kind === 'arm') {
      const asel = h('select', { title: '到点投给臂泵（启动即返回, 后写者胜）' });
      if (!(CHACT.armOptions || []).length) {
        asel.appendChild(h('option', { value: 0 }, '（先去「机械臂」Tab 建动作）'));
      } else {
        for (const a of CHACT.armOptions) {
          asel.appendChild(h('option', { value: a.id }, (a.title || ('动作' + a.id))));
        }
        asel.value = String(it.arm_id);
      }
      asel.addEventListener('change', () => { it.arm_id = Number(asel.value); chactTouch(it); });
      pcell.appendChild(asel);
    } else if (it.kind === 'arm_wait') {
      pcell.appendChild(document.createTextNode('等臂上限'));
      pcell.appendChild(chactNumInput(it.dur, v => { it.dur = Number(v) || 0; chactTouch(it); }, { step: 1000, min: 0, title: '等臂空闲上限 ms（0=固件默认 60s）; 期间时间轴冻结、车停住但保持航向' }));
      pcell.appendChild(document.createTextNode(' ms'));
    } else {
      pcell.appendChild(h('span', { class: 'hint' }, '（手写行, 只读）'));
    }
    row.appendChild(pcell);

    /* 时长（平移段现算; 转弯/支点无段上限 → 显示「到位」） */
    row.appendChild(h('td', { class: 'chact-dur' },
      it.kind === 'raw' ? ''
        : ((it.kind === 'turn' || it.kind === 'pivot')
           ? '到位' : String(Math.round(chactItemDur(it))))));

    /* 备注 */
    const tag = h('input', { class: 'chact-tag', value: it.tag || '',
                             placeholder: '备注', maxlength: 30 });
    tag.addEventListener('change', () => { it.tag = tag.value; chactTouch(it); });
    row.appendChild(h('td', {}, it.kind === 'raw' ? '' : tag));

    /* 操作: 上移/下移/删 */
    const ops = h('td', {});
    if (it.kind !== 'raw') {
      ops.appendChild(h('button', { class: 'btn ghost tiny', title: '上移',
        onclick: () => { if (idx > 0) { const t = g.items[idx - 1]; g.items[idx - 1] = g.items[idx]; g.items[idx] = t; chactMarkDirty(); chactRenderItems(); } } }, '↑'));
      ops.appendChild(h('button', { class: 'btn ghost tiny', title: '下移',
        onclick: () => { if (idx < g.items.length - 1) { const t = g.items[idx + 1]; g.items[idx + 1] = g.items[idx]; g.items[idx] = t; chactMarkDirty(); chactRenderItems(); } } }, '↓'));
      ops.appendChild(h('button', { class: 'btn ghost tiny', title: '删除这一条',
        onclick: () => { g.items.splice(idx, 1); chactMarkDirty(); chactRenderItems(); chactRenderTimeline(); } }, '✕'));
    }
    row.appendChild(ops);
    table.appendChild(row);
  });
  box.appendChild(table);
}

function chactReplaceKind(it, kind) {
  it.kind = kind;
  it.edited = true;
  if (kind === 'move') {
    it.wx = 0; it.wy = 300; it.speed = 150; it.accel = 120; it.hold = true;
  } else if (kind === 'turn') {
    it.delta = -90; it.speed = 60; it.abs = 0; it.accel = 0; it.dur = 0;   // dur 槽已废弃
  } else if (kind === 'pivot') {
    /* 支点转弯: 与 turn 同形（delta/abs/w≤）, dur/accel 恒 0;
     * w≤ 出厂默认 = PIVOT_TURN_MAX_SPEED(130, 驱动侧轮速) */
    it.delta = -90; it.speed = 130; it.abs = 0; it.accel = 0; it.dur = 0;
  } else if (kind === 'arm') {
    it.arm_id = (CHACT.armOptions[0] && CHACT.armOptions[0].id) || 0;
  } else {
    it.dur = 60000;
  }
}

function chactAddItem(kind) {
  const g = chactCur();
  if (!g) return;
  const it = { id: chactNewId(), kind, start: 0, tag: '', disabled: false, edited: true };
  chactReplaceKind(it, kind);
  it.tag = chactKindName(kind);
  g.items.push(it);
  CHACT.dirty = true;
  chactRenderItems();
  chactRenderTimeline();
  chactRenderBar();
}

/* ---- 时间轴（只读条; 2026-10 改: **每条条目独占一行**, 按起点排序 —— 同类型
 *      不再共用泳道, 交叠的先后顺序一眼可读; 类型靠颜色区分, 禁用/手写行不画） ---- */

const CHACT_KIND_COLORS = { move: '#4a7fd4', turn: '#e08a2e', pivot: '#2f9e6e',
                            arm: '#9a5fd0', arm_wait: '#8a8a8a' };

function chactBarLabel(it) {
  if (it.kind === 'move') {
    return (Number(it.wy) || 0) + 'Y/' + (Number(it.wx) || 0) + 'Xmm@' + it.speed;
  }
  if (it.kind === 'turn') {
    return (it.abs ? ('咬' + it.delta + '°')
                   : ((it.delta > 0 ? '左' : '右') + '转' + Math.abs(it.delta) + '°'))
         + '→到位';
  }
  if (it.kind === 'pivot') {
    return '支点' + ((it.abs ? ('咬' + it.delta + '°')
                             : ((it.delta > 0 ? '左' : '右') + '转' + Math.abs(it.delta) + '°')))
         + '→到位';
  }
  if (it.kind === 'arm') return '臂#' + it.arm_id;
  return '等臂';
}

function chactRenderTimeline() {
  const box = document.getElementById('chact-timeline');
  box.textContent = '';
  const g = chactCur();
  const all = (g && g.items) || [];
  const shown = all.filter(it => !it.disabled && it.kind !== 'raw');
  const nOff = all.length - shown.length;
  const total = chactTotalMs(g);
  const hasTurn = shown.some(it => it.kind === 'turn' || it.kind === 'pivot');
  box.appendChild(h('div', { class: 'chact-tl-total' },
    '总时长 ≈ ' + (total / 1000).toFixed(2) + ' s'
    + (hasTurn ? ' + 转弯到位（无段上限, 组尾等收敛）' : '')
    + (nOff ? '（' + nOff + ' 条禁用/手写行不显示）' : '')
    + (total || shown.length ? '' : '（空）')));

  const wrap = h('div', { class: 'chact-tl-wrap' });
  /* 按起点排序（同起点按表内顺序）—— 时间轴的行序 = 播放的先后 */
  const ordered = shown.map((it, i) => ({ it, i })).sort((a, b) =>
    (Number(a.it.start) || 0) - (Number(b.it.start) || 0) || a.i - b.i);

  if (!ordered.length) {
    wrap.appendChild(h('div', { class: 'chact-tl-lane' },
      h('span', { class: 'chact-tl-lab' }, '—'),
      h('div', { class: 'chact-tl-track' })));
  }
  for (const { it, i } of ordered) {
    const dur = chactItemDur(it);
    const w = total > 0 ? Math.max(0.7, (dur / total) * 100) : 100;
    const l = total > 0 ? Math.min(99.3, ((Number(it.start) || 0) / total) * 100) : 0;
    const rowEl = h('div', { class: 'chact-tl-lane' },
      h('span', { class: 'chact-tl-lab', title: it.tag || '' },
        (i + 1) + '·' + chactKindName(it.kind) + (it.tag ? '·' + it.tag : '')));
    const track = h('div', { class: 'chact-tl-track' });
    track.appendChild(h('div', {
      class: 'chact-tl-bar',
      style: 'left:' + l.toFixed(2) + '%;width:' + w.toFixed(2) + '%;'
           + 'background:' + (CHACT_KIND_COLORS[it.kind] || '#888') + ';',
      title: chactBarLabel(it) + ' @' + it.start + 'ms'
           + (dur > 0 ? ' +' + Math.round(dur) + 'ms' : '（转到位为止）'),
    }, dur > 0 ? chactBarLabel(it) : '▸'));
    rowEl.appendChild(track);
    wrap.appendChild(rowEl);
  }
  box.appendChild(wrap);
}

/* ---- 回收站 ---- */

function chactRenderTrash() {
  const list = document.getElementById('chact-trash-list');
  list.textContent = '';
  if (!(CHACT.trash || []).length) {
    list.appendChild(h('p', { class: 'hint' }, '（空）'));
    return;
  }
  for (const t of CHACT.trash) {
    list.appendChild(h('div', { class: 'chact-trash-row' },
      h('span', {}, (t.title || ('组' + t.id)) + ' · ' + (t.deleted_at || '')),
      h('button', { class: 'btn ghost tiny', onclick: () => chactRestore(t.trash_id) }, '恢复'),
      h('button', { class: 'btn ghost tiny', title: '彻底删除, 不可再恢复',
        onclick: async () => {
          if (!confirm('彻底删除「' + (t.title || '') + '」？这一步不可再恢复。')) return;
          try {
            const r = await chactCall('purge', { trash_id: t.trash_id });
            if (r.ok) { CHACT.trash = r.data.trash || []; chactRenderTrash(); }
          } catch (e) { toast(e.message, 'err'); }
        } }, '✕')));
  }
}

async function chactRestore(trashId) {
  try {
    const r = await chactCall('restore', { trash_id: trashId, base_version: CHACT.version });
    if (!r.ok) throw new Error(r.error || '恢复失败');
    CHACT.version = r.data.version;
    CHACT.groups = r.data.groups;
    CHACT.activeIdx = CHACT.groups.length - 1;
    CHACT.dirty = false;
    try { const st = await (await fetch('/api/chact')).json(); chactApplyState(st); } catch (e) {}
    chactRenderAll();
    toast('已恢复为「' + r.data.title + '」');
  } catch (e) {
    toast(e.message, 'err');
  }
}

/* ---- 预览（$B,CPLAY / $B,CSTOP; 非阻塞, 轮询 ch_busy） ---- */

function chactBusyNote(text) {
  document.getElementById('chact-busy').textContent = text;
}

function chactStopBusyPoll(note) {
  if (!CHACT.busyTimer) return;
  clearInterval(CHACT.busyTimer);
  CHACT.busyTimer = null;
  if (note) chactBusyNote(note);   /* 切走时顺手把「▶ 播放中…」换成不再刷新的说明 */
}

async function chactPollBusy() {
  const r = await btRun('get', { name: 'ch_busy' }, { quiet: true, silent: true });
  if (r === null) {          /* 链路断了/掉出编辑模式: 固件那边会自己停, 别再轮询 */
    chactStopBusyPoll();
    chactBusyNote('播放器: 链路断开（预览已自动停止）');
    return;
  }
  if (!Number(r.value)) {
    chactStopBusyPoll();
    chactBusyNote('播放器: 空闲');
  } else {
    chactBusyNote('播放器: ▶ 播放中…（可随时停止）');
  }
}

async function chactPlay() {
  const g = chactCur();
  if (!g) return;
  if (!g.id) { toast('先「保存动作组」拿到编号再预览', 'err'); return; }
  if (!BT.connected) { toast('先在右侧连接蓝牙并进入编辑模式', 'err'); return; }
  if (!BT.edit) { toast('先进入编辑模式（预览的入场门）', 'err'); return; }
  if (!(g.items || []).length) { toast('这一组还没有条目', 'err'); return; }
  /* 在线推送（2026-10 加）: 预览前把**控制台当前这一份**（含未保存的修改）经蓝牙
   * 推进固件 RAM 草稿区, 预览命中草稿播出 —— 所见即所播, 不用每次重编译。
   * ⚠️ 路线/比赛仍只读 FLASH 编译表: 改完要上比赛 = 保存 + Keil 重编译 + 烧录。 */
  const r = await btRun('cpush', { id: g.id, items: g.items });
  if (r === null) return;
  if (r.skipped_raw) {
    btlog('⚠ 本组有 ' + r.skipped_raw + ' 条手写只读行, 在线预览跳过它们'
          + '（完整效果以编译烧录为准）', 'bad');
  }
  const r2 = await btRun('cplay', { id: g.id });
  if (r2 === null) return;
  chactStopBusyPoll();
  chactPollBusy();
  CHACT.busyTimer = setInterval(chactPollBusy, 500);
}

async function chactStop() {
  chactStopBusyPoll();
  const r = await btRun('cstop', {}, { quiet: true });
  chactBusyNote(r === null ? '播放器: 停止指令发送失败（再点一次）' : '播放器: 已请求停止');
}

/* 航向关系重置（2026-10 加）: 上电后手动搬车回出发区, 物理朝向已转过而世界航向
 * 账面还在 0° —— 相对转弯会转错、保持层会打架。此键把账面对齐到当前车头方向
 * （重建上电时的关系）。不动车、不受编辑模式门槛, 摆好车按。 */
async function chactHr0() {
  if (!BT.connected) { toast('先连接蓝牙（航向归零是对设备状态的写入）', 'err'); return; }
  const r = await btRun('chr0', {});
  if (r === null) return;
  if (r.ref) btlog('⌖ 航向已归零: 当前车头方向 = 新 0° 基准（设备报 ' + r.ref + '°）');
}

/* ---- 调参卡（蓝牙实时; 2026-10 加） ----
 * 把「底盘动作组」整组 Beam 参数做成本 Tab 的内联编辑器 —— 调 k/调 PID 不用再切
 * 蓝牙 Tab。值分两层: 未连蓝牙显示 app_config.h 的宏默认（服务端从源码解析,
 * disabled 不可改）; 连上后「回读」/自动拉取设备实时值, 改动走 $B,SET 立即生效
 * （播放中也生效, 断电回宏 —— 满意后写回宏固化）。 */

const CHACT_TUNE_DEFS = [
  ['ch_time_scale', '时间系数 k', 0.01, 0.01, 5, 2],
  ['ch_head_kp', '航向 P', 0.1, 0, 20, 2],
  ['ch_head_ki', '航向 I', 0.05, 0, 5, 2],
  ['ch_head_kd', '航向 D', 0.1, 0, 20, 2],
  ['ch_head_ki_max', '积分限幅', 0.5, 0, 50, 1],
  ['ch_head_db_deg', '保持死区 °', 0.1, 0, 10, 1],
  ['ch_w_max', '旋转限幅 rpm', 5, 0, 200, 0],
  ['ch_wheel_max', '四轮饱和钳 rpm', 10, 0, 1000, 0],
  ['ch_pivot_kp', '支点 P', 0.1, 0, 20, 2],
  ['ch_pivot_ki', '支点 I', 0.05, 0, 5, 2],
  ['ch_pivot_kd', '支点 D', 0.1, 0, 20, 2],
  ['ch_pivot_ki_max', '支点积分限幅', 0.5, 0, 50, 1],
  ['ch_pivot_w_max', '支点驱动限幅 rpm', 5, 0, 255, 0],
];

function chactTuneRender() {
  const box = document.getElementById('chact-tune-rows');
  box.textContent = '';
  const offline = !BT.connected;
  for (const [name, label, step, min, max, dec] of CHACT_TUNE_DEFS) {
    /* 离线默认值 = app_config.h 宏（服务端解析）, 键 = 蓝牙参数名去掉 ch_ 前缀 */
    const cfgKey = name.replace(/^ch_/, '');
    const val = (CHACT.tune[name] === undefined)
      ? (CHACT.consts[cfgKey] !== undefined ? CHACT.consts[cfgKey] : 0)
      : CHACT.tune[name];
    const row = h('div', { class: 'chact-tune-row' });
    const inp = h('input', {
      class: 'chact-num', type: 'number',
      value: Number(val).toFixed(dec), step: step, min: min, max: max,
      title: name + (offline ? '（宏默认, 未连蓝牙不可改）' : ''),
      disabled: offline,
    });
    inp.addEventListener('change', () => chactTuneWrite(name, inp.value, dec));
    row.appendChild(h('span', { class: 'chact-tune-lab' }, label));
    row.appendChild(inp);
    box.appendChild(row);
  }
  if (offline) {
    box.appendChild(h('p', { class: 'hint' },
      '未连蓝牙 —— 上面是 app_config.h 的宏默认（上电初值）。'));
  }
}

async function chactTuneRead() {
  if (!BT.connected) { toast('先连接蓝牙（调参卡读的是设备实时值）', 'err'); return; }
  let okN = 0;
  for (const [name] of CHACT_TUNE_DEFS) {
    const r = await btRun('get', { name: name }, { quiet: true, silent: true });
    if (r !== null && r.value !== undefined) {
      CHACT.tune[name] = Number(r.value);
      okN++;
    }
  }
  chactTuneRender();
  btlog(okN === CHACT_TUNE_DEFS.length
        ? ('已回读 ' + okN + ' 个调参量（设备实时值）')
        : ('回读完成 ' + okN + '/' + CHACT_TUNE_DEFS.length + '（部分失败, 看日志）'),
        okN ? '' : 'bad');
}

async function chactTuneWrite(name, value, dec) {
  const v = Number(value);
  if (Number.isNaN(v)) { chactTuneRender(); return; }
  const r = await btRun('set', { name: name, value: v });
  if (r === null) {
    btlog('✗ ' + name + ' 写入失败, 已回读设备值', 'bad');
    await chactTuneRead();          // 失败回读设备真值, 输入框自动弹回
    return;
  }
  CHACT.tune[name] = Number(v.toFixed(dec));
  btlog('已写入 ' + name + ' = ' + v.toFixed(dec) + '（即时生效, 断电回宏）');
  chactTuneRender();
}

/* ---- 编辑模式（与「机械臂」Tab 同一条链路: 同一时刻只有一个编辑模式） ----
 * 按钮态的**唯一**同步点在 btSetEdit（含设备超时踢出/另一边手动进出的所有路径）,
 * 这里只负责调它; finally 兜底是防 btEnterEdit 后半段（回读参数/取位姿）万一抛
 * 异常时按钮卡在旧态 —— 那种情况下模式多半已经进去了, 按钮必须跟上。 */
async function chactEditOn() {
  try {
    await btEnterEdit(false);
  } finally {
    chactSyncEditBtns();
  }
}

async function chactEditOff() {
  try {
    await btExitEdit();
  } finally {
    chactSyncEditBtns();
  }
}

function chactSyncEditBtns() {
  document.getElementById('chact-edit-on').disabled = !!BT.edit;
  document.getElementById('chact-edit-off').disabled = !BT.edit;
}

/* ---- 静态骨架的事件绑定（进 Tab 首次加载时绑一次） ---- */

let CHACT_BOUND = false;

function chactBindOnce() {
  if (CHACT_BOUND) return;
  CHACT_BOUND = true;

  document.getElementById('chact-group-select')
    .addEventListener('change', e => chactSwitchGroup(Number(e.target.value)));
  /* 与 arm-title/note 同款: input 事件里直接写 state —— 只 markDirty 不写值的话,
   * blur 后重渲染会把输入框回灌成 state 里的旧值, 标题/备注永远存不进去 */
  document.getElementById('chact-title')
    .addEventListener('input', (e) => {
      const g0 = chactCur();
      if (g0) g0.title = e.target.value;
      chactMarkDirty();
    });
  document.getElementById('chact-note')
    .addEventListener('input', (e) => {
      const g0 = chactCur();
      if (g0) g0.note = e.target.value;
      chactMarkDirty();
    });
  document.getElementById('chact-group-add').onclick = chactGroupAdd;
  document.getElementById('chact-group-dup').onclick = chactGroupDup;
  document.getElementById('chact-group-del').onclick = chactGroupDel;
  document.getElementById('chact-save').onclick = chactSave;
  document.getElementById('chact-reload').onclick = chactReload;
  document.getElementById('chact-add-move').onclick = () => chactAddItem('move');
  document.getElementById('chact-add-turn').onclick = () => chactAddItem('turn');
  document.getElementById('chact-add-arm').onclick = () => chactAddItem('arm');
  document.getElementById('chact-add-wait').onclick = () => chactAddItem('arm_wait');
  document.getElementById('chact-play').onclick = chactPlay;
  document.getElementById('chact-stop').onclick = chactStop;
  document.getElementById('chact-hr0').onclick = chactHr0;
  document.getElementById('chact-edit-on').onclick = chactEditOn;
  document.getElementById('chact-edit-off').onclick = chactEditOff;
  document.getElementById('chact-trash-open').onclick = () => {
    document.getElementById('chact-trash-card').classList.remove('hidden');
  };
  document.getElementById('chact-trash-close').onclick = () => {
    document.getElementById('chact-trash-card').classList.add('hidden');
  };
  document.getElementById('chact-log-clear').onclick = () => {
    const el = document.getElementById('chact-log');
    if (el) el.textContent = '';
  };

  makeConnPanel({
    id: 'chact',
    mount: document.getElementById('chact-conn-mount'),
    logEl: document.getElementById('chact-log'),
    onConnected: chactTuneRead,       /* 连上即拉设备实时值, 调参卡解锁 */
  });
  document.getElementById('chact-tune-read').onclick = chactTuneRead;
}

function chactEnter() {
  chactBindOnce();
  if (!CHACT.loaded) loadChact();
  else chactRenderAll();
  chactSyncEditBtns();
  /* 调参卡: 首次进 Tab 就渲染（离线=宏默认; 连着蓝牙且还没读过设备值 → 自动拉一次） */
  chactTuneRender();
  if (BT.connected && CHACT.tune.ch_head_kp === undefined) chactTuneRead();
}

/* ==================== Tab: 标定 ====================
 * 主摄标定预设的管理中心（后端 calib.py, 预设库 = calib.json, 参数清单 = 服务端
 * tuning.py 的 CALIB_GROUPS —— 前端不写死任何宏名/元数据, 全部来自 GET /api/calib）。
 * 数据流: GET /api/calib → CALIB（presets + groups 元数据 + 当前宏值）→ 渲染;
 *   「保存预设」POST action:save 只落 calib.json;
 *   「设为当前」POST action:apply 写回 app_config.h 的宏（服务端先备份 .bak）,
 *   生效要等 Keil 重编译烧录 —— 界面上的 ★ 只表示「宏里现在是这套」。 */

const CALIB = {
  loaded: false,
  version: null,        // calib.json 的 mtime（保存/apply 的乐观锁）
  cfgVersion: null,     // app_config.h 的 mtime（apply 的乐观锁 + rev 轮询比对）
  presets: [],          // [{id, name, note, values:{宏名:显示值}}]（id=null = 还没存过）
  active: null,         // 当前写进宏里的那套预设 id（服务端 calib.json 记账）
  nextId: 1,
  groups: [],           // [{title, items:[元数据]}] —— 显示名/单位/范围/备注/当前宏值
  byName: {},           // 宏名 → 元数据（含 display_value = 当前宏值）
  selIdx: 0,
  dirty: false,
};

let CALIB_BOUND = false;

function calibCur() { return CALIB.presets[CALIB.selIdx] || null; }

function calibMarkDirty() { CALIB.dirty = true; calibRenderBar(); }

/* 预设值与宏值比的是**数值**（1150 与 1150.0 是同一个数）; 非数字（不该出现）退化为文本比 */
function calibNumEq(a, b) {
  const x = Number(String(a).trim()), y = Number(String(b).trim());
  if (Number.isFinite(x) && Number.isFinite(y)) return Math.abs(x - y) < 1e-9;
  return String(a).trim() === String(b).trim();
}

/* ---- 请求 ---- */

async function loadCalib() {
  CALIB.loaded = true;
  try {
    const res = await fetch('/api/calib');
    const st = await res.json();
    calibApplyState(st);
  } catch (e) {
    toast('读取标定预设失败: ' + e.message, 'err');
  }
  calibRenderAll();
}

function calibApplyState(st) {
  CALIB.version = st.version;
  CALIB.cfgVersion = st.cfg_version;
  CALIB.groups = st.groups || [];
  CALIB.byName = {};
  for (const g of CALIB.groups) for (const it of g.items) CALIB.byName[it.name] = it;
  CALIB.presets = st.presets || [];
  CALIB.active = (st.active === undefined) ? null : st.active;
  CALIB.nextId = st.next_id || 1;
  if (CALIB.selIdx >= CALIB.presets.length) CALIB.selIdx = 0;
  CALIB.dirty = false;
  /* 宏缺失（改名/删行）不静默 —— 横幅点名, 保存/apply 在服务端也会拦 */
  const err = document.getElementById('calib-error');
  if (st.error) {
    err.textContent = '⚠ ' + st.error + ' —— 标定 Tab 的参数名与 app_config.h 脱钩了, 先修源码';
    err.classList.remove('hidden');
  } else {
    err.classList.add('hidden');
  }
}

function calibRenderAll() {
  calibRenderBar();
  calibRenderTables();
  calibRenderStatus();
}

/* ---- 顶栏 ---- */

function calibRenderBar() {
  const sel = document.getElementById('calib-select');
  sel.textContent = '';
  CALIB.presets.forEach((p, i) => {
    sel.appendChild(h('option', { value: i },
      (p.id != null && p.id === CALIB.active ? '★ ' : '') +
      (p.name || ('预设' + (i + 1))) + (p.id == null ? ' · 未保存' : '')));
  });
  if (CALIB.presets.length) sel.value = String(CALIB.selIdx);

  /* 同 chactRenderBar 的纪律: 输入框正被聚焦时别回灌 value（会把光标顶飞） */
  const p = calibCur();
  const n = document.getElementById('calib-name');
  const note = document.getElementById('calib-note');
  if (document.activeElement !== n) n.value = p ? (p.name || '') : '';
  if (document.activeElement !== note) note.value = p ? (p.note || '') : '';
  document.getElementById('calib-save').textContent =
    CALIB.dirty ? '保存预设 *' : '保存预设';
  document.getElementById('calib-apply').disabled = !p;
  document.getElementById('calib-dup').disabled = !p;
  document.getElementById('calib-del').disabled = !p;
}

function calibSwitch(idx) {
  if (CALIB.dirty && !confirm('当前预设的修改还没保存，切换会丢掉。继续？')) return;
  CALIB.selIdx = Math.max(0, Math.min(idx, CALIB.presets.length - 1));
  CALIB.dirty = false;
  calibRenderAll();
}

/* 新建 = 拿**当前宏值**当种子（换新摄像头的第一步: 老标定打底, 再逐项重标） */
function calibAdd() {
  if (CALIB.dirty && !confirm('当前预设的修改还没保存，新建会丢掉。继续？')) return;
  const values = {};
  for (const name in CALIB.byName) values[name] = String(CALIB.byName[name].display_value);
  CALIB.presets.push({
    id: null, name: '新预设' + (CALIB.presets.length + 1), note: '', values,
  });
  CALIB.selIdx = CALIB.presets.length - 1;
  CALIB.dirty = true;
  calibRenderAll();
}

function calibDup() {
  const p = calibCur();
  if (!p) return;
  /* 深拷贝当前所见（含未保存修改）, id 置空让服务端发新号 —— 换新摄像头时
   * 拿老相机的标定当起点逐项重标, 正是这条路径。不弹确认: 原预设原样还在。 */
  const src = JSON.parse(JSON.stringify(p));
  src.id = null;
  src.name = (p.name || '预设') + ' 副本';
  CALIB.presets.push(src);
  CALIB.selIdx = CALIB.presets.length - 1;
  CALIB.dirty = true;
  calibRenderAll();
}

function calibDel() {
  const p = calibCur();
  if (!p) return;
  /* 当前写进宏里的那套不让删: calib.json 的 active 要一直指着真实存在的预设 */
  if (p.id != null && p.id === CALIB.active) {
    toast('这套是当前写进宏里的预设, 不能删 —— 先对别套「设为当前」', 'err');
    return;
  }
  if (!confirm('删除预设「' + (p.name || '未命名') + '」？')) return;
  CALIB.presets.splice(CALIB.selIdx, 1);
  CALIB.selIdx = Math.max(0, Math.min(CALIB.selIdx, CALIB.presets.length - 1));
  CALIB.dirty = true;
  calibRenderAll();
}

let _savingCalib = false;
async function calibSave() {
  if (_savingCalib) return false;   /* 防重入（同 save）; false = 这次没保存成功 */
  _savingCalib = true;
  try {
    return await calibSaveInner();
  } finally {
    _savingCalib = false;
  }
}

async function calibSaveInner() {
  const body = {
    action: 'save',
    presets: CALIB.presets,
    active: CALIB.active,
    next_id: CALIB.nextId,
    base_version: CALIB.version,
  };
  let j;
  try {
    const res = await fetch('/api/calib', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    j = await res.json();
  } catch (e) {
    toast('保存失败: ' + e.message, 'err');
    return false;
  }
  if (!j.ok) {
    toast(j.error || '保存失败', 'err');
    return false;
  }
  const d = j.data;
  CALIB.version = d.version;
  CALIB.presets = d.presets;
  CALIB.nextId = d.next_id;
  CALIB.active = d.active;
  CALIB.dirty = false;
  calibRenderAll();
  toast('标定预设已保存' + (d.changed ? '' : '（内容没变, 没写盘）'));
  return true;
}

/* 「设为当前」: 服务端按 calib.json 里**已保存**的那套写宏 —— 本地有未保存修改时
 * 先自动保存（否则界面上改的值根本不会上车, 还以为换好了相机）。 */
async function calibApply() {
  const p = calibCur();
  if (!p) return;
  if (CALIB.dirty) {
    const ok = await calibSave();
    if (!ok) return;
  }
  const cur = calibCur();                       /* save 给新预设发了 id, 重取一次 */
  if (!cur || cur.id == null) { toast('预设还没保存好, 取消', 'err'); return; }
  if (!confirm('把「' + (cur.name || '未命名') + '」写进 app_config.h 的宏？\n' +
               '（自动备份 .bak; 之后要在 Keil 全量重编译并烧录, 车上才生效）')) return;
  let j;
  try {
    const res = await fetch('/api/calib', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'apply', id: cur.id,
        base_version: CALIB.version, base_cfg: CALIB.cfgVersion,
      }),
    });
    j = await res.json();
  } catch (e) {
    toast('设为当前失败: ' + e.message, 'err');
    return;
  }
  if (!j.ok) {
    toast(j.error || '设为当前失败', 'err');
    return;
  }
  const d = j.data;
  CALIB.active = d.active;
  CALIB.version = d.version;
  CALIB.cfgVersion = d.cfg_version;             /* 回填, 免得 rev 轮询误判成远端改动 */
  CALIB.dirty = false;
  calibRenderAll();
  toast(d.note || '已设为当前', 'ok');
  for (const w of d.warnings || []) toast(w.name + ': ' + w.msg, 'err');
}

function calibReload() {
  if (CALIB.dirty && !confirm('本地修改还没保存，重载会丢掉。继续？')) return;
  CALIB.dirty = false;
  loadCalib();
}

/* ---- 参数表 ---- */

function calibRenderTables() {
  const host = document.getElementById('calib-tables');
  host.textContent = '';
  const p = calibCur();
  if (!p) {
    host.appendChild(h('div', { class: 'tune-group' },
      h('div', { class: 'hint' },
        '还没有标定预设 —— 点「＋ 新建」拿当前宏值当种子建一套（一台主摄一套）。')));
    return;
  }
  for (const g of CALIB.groups) {
    const card = h('div', { class: 'tune-group' },
      h('div', { class: 'tune-group-head' },
        h('span', { class: 'tune-group-title' }, g.title),
        h('span', { class: 'tune-group-n' }, g.items.length + ' 项')));
    for (const it of g.items) card.appendChild(calibRow(it, p));
    host.appendChild(card);
  }
}

function calibRow(it, p) {
  const cur = it.display_value;                 // app_config.h 里的当前宏值
  const val = p.values[it.name] ?? '';          // 旧预设缺新参数: 空串, 别显示 "undefined"
  const diff = !calibNumEq(val, cur);
  const name = h('div', { class: 'tune-name', title: it.note || '' },
    h('span', { class: 'tn-disp' }, it.display),
    h('code', { class: 'tn-macro' }, it.name));
  const inp = h('input', {
    class: 'tune-val', value: val,
    title: '预设值（显示单位' + (it.unit ? ' ' + it.unit : '') + '）',
  });
  inp.addEventListener('input', () => {
    p.values[it.name] = inp.value;
    calibMarkDirty();
    calibRefreshDiff(inp, it.name, p);          // 只刷同行对比列, 不重建表（别把焦点弄丢）
  });
  const curCell = h('span', {
    class: 'calib-cur' + (diff ? ' diff' : ''),
    title: 'app_config.h 里的当前宏值' + (diff ? ' —— 与预设不一致（改了还没「设为当前」）' : ''),
  }, String(cur));
  return h('div', { class: 'tune-item', 'data-name': it.name }, name,
    h('div', { class: 'tune-input' }, inp,
      h('span', { class: 'tune-unit' }, it.unit || '')),
    curCell);
}

function calibRefreshDiff(inp, name, p) {
  const row = inp.closest('.tune-item');
  if (!row) return;
  const cell = row.querySelector('.calib-cur');
  const meta = CALIB.byName[name];
  if (cell && meta) {
    const diff = !calibNumEq(p.values[name], meta.display_value);
    cell.classList.toggle('diff', diff);
  }
}

function calibRenderStatus() {
  const el = document.getElementById('calib-status');
  const n = CALIB.presets.length;
  if (!n) { el.textContent = '（还没有预设）'; return; }
  const act = CALIB.presets.find(x => x.id != null && x.id === CALIB.active);
  el.textContent = '共 ' + n + ' 套预设; 当前写进宏里: ' +
    (act ? ('★ ' + (act.name || ('#' + act.id))) : '（无 —— 还没对任何一套「设为当前」）');
}

/* ---- 静态骨架的事件绑定（进 Tab 首次加载时绑一次） ---- */

function calibBindOnce() {
  if (CALIB_BOUND) return;
  CALIB_BOUND = true;

  document.getElementById('calib-select')
    .addEventListener('change', e => calibSwitch(Number(e.target.value)));
  document.getElementById('calib-name')
    .addEventListener('input', () => {
      const p = calibCur();
      if (p) { p.name = document.getElementById('calib-name').value; calibMarkDirty(); }
    });
  document.getElementById('calib-note')
    .addEventListener('input', () => {
      const p = calibCur();
      if (p) { p.note = document.getElementById('calib-note').value; calibMarkDirty(); }
    });
  document.getElementById('calib-add').onclick = calibAdd;
  document.getElementById('calib-dup').onclick = calibDup;
  document.getElementById('calib-del').onclick = calibDel;
  document.getElementById('calib-save').onclick = calibSave;
  document.getElementById('calib-apply').onclick = calibApply;
  document.getElementById('calib-reload').onclick = calibReload;
}

function calibEnter() {
  calibBindOnce();
  if (!CALIB.loaded) loadCalib();
  else calibRenderAll();
}

/* ==================== Tab: 采集 ====================
 * 数据集采集: 单帧轮询 /api/capture/frame.jpg 画进 canvas, 分类存到 vision/datasets/raw/。
 * ⚠️ 不是 MJPEG —— server.py 全文件只有 _send 一个响应写入口（必写 Content-Length）,
 * 流式响应一旦有异常逃到 do_GET 外层的 except → _send(500), 就会往半个流里插第二个
 * 状态行, 只能靠纪律守住。轮询用现成的 _send 发单帧 JPEG, 结构上没这个可能, 代价只是
 * 127.0.0.1 上 ~30 req/s。详见 capture.py 顶部。
 * 类别名 / 各类计数 / 摄像头状态**全部由服务端下发**（GET /api/capture）, 前端不写死
 * 任何类别名 —— 与「地图」「调参」「蓝牙」同一条路子。
 */

const CAP = {
  loaded: false,
  available: true,
  enabled: true,          /* 总开关: 关掉即释放摄像头设备, 见 capSetEnabled */
  hint: '',
  state: 'idle',
  ready: false,
  index: null,
  resolution: '',
  classes: [],
  counts: {},
  cams: [],
  sel: 'cylinder',
  interval: 2.0,
  pollMs: 33,
  rawDir: '',
  prefs: {},
  autoTimer: null,
  statusTimer: null,
  frameTimer: null,
  fps: { n: 0, t: 0 },
  lastShot: 0,
  lastFrameAt: 0,
  offlineShown: false,
};

const CAP_IMG = new Image();        /* 复用同一个 Image, 免得每帧造对象 */

function capLog(msg, kind) {
  const el = document.getElementById('cap-log');
  if (!el) return;
  const span = document.createElement('span');
  if (kind) span.className = kind;
  span.textContent = msg;
  el.appendChild(span);
  el.scrollTop = el.scrollHeight;
}

async function capCall(action, extra) {
  const res = await fetch('/api/capture', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ action: action }, extra || {})),
  });
  if (!res.ok) throw new Error('HTTP ' + res.status);
  return res.json();
}

/* 同 btRun: 打日志、弹 toast、把 data 交回调用方; 失败回 null。
 * quiet = 不弹 toast; silent = 连日志都不打（轮询用, 否则日志会被刷爆）。 */
async function capRun(action, extra, opts) {
  opts = opts || {};
  try {
    const j = await capCall(action, extra);
    if (!opts.silent) for (const l of (j.log || [])) capLog(l.text, l.kind);
    if (!j.ok) {
      if (!opts.silent && j.error) capLog('✗ ' + j.error, 'bad');
      if (!opts.quiet && !opts.silent) toast(j.error || '操作失败', 'err');
      return null;
    }
    return j.data === undefined ? {} : j.data;
  } catch (e) {
    if (!opts.silent) capLog('✗ ' + e.message, 'bad');
    if (!opts.quiet && !opts.silent) toast('请求失败: ' + e.message, 'err');
    return null;
  }
}

/* 拉状态。light = 轮询用: 只刷计数/状态, 不重建类别按钮（否则每 2s 把用户正点的按钮换掉） */
async function capFetchState(light) {
  let st;
  try { st = await fetch('/api/capture').then(r => r.json()); }
  catch (e) { return null; }
  CAP.available = !!st.available;
  CAP.hint = st.hint || '';
  CAP.state = st.state || 'idle';
  CAP.ready = !!st.ready;
  CAP.index = (st.index === undefined) ? null : st.index;
  CAP.resolution = st.resolution || '';
  CAP.classes = st.classes || [];
  CAP.counts = st.counts || {};
  CAP.rawDir = st.raw_dir || '';
  CAP.pollMs = st.poll_ms || 33;
  CAP.prefs = st.prefs || {};
  /* 总开关状态以服务端为准（cap.json）。默认开 —— 没这个键的老配置照旧能用。 */
  CAP.enabled = CAP.prefs.enabled !== false;
  const enableBox = document.getElementById('cap-enable');
  if (enableBox) enableBox.checked = CAP.enabled;
  if (!CAP.available) { capRenderUnavailable(); return st; }
  if (light) {
    capRenderCounts();
  } else {
    if (CAP.prefs.class) CAP.sel = CAP.prefs.class;
    if (typeof CAP.prefs.interval === 'number') {
      CAP.interval = CAP.prefs.interval;
      const inp = document.getElementById('cap-interval');
      if (inp) inp.value = CAP.interval;
    }
    capRenderClasses();
  }
  capRenderStatus();
  return st;
}

/* opencv 没装: 整个面板换成一张说明卡。服务端其余部分照常, 这里只是没法预览/采集。 */
function capRenderUnavailable() {
  if (CAP.offlineShown) return;
  CAP.offlineShown = true;
  const body = document.getElementById('cap-body');
  if (!body) return;
  body.textContent = '';
  body.appendChild(h('section', { class: 'tools-card' },
    h('h3', {}, '采集不可用'),
    h('p', { class: 'hint' }, CAP.hint || 'opencv-python 未安装。'),
    h('p', { class: 'hint' }, '在仓库根执行：', h('code', {}, 'pip install opencv-python')),
    h('p', { class: 'hint' }, '装完重启控制台即可；其余 Tab 不受影响。')));
}

function capCountText(name) { return (CAP.counts[name] || 0) + ' 张'; }

function capRenderClasses() {
  const box = document.getElementById('cap-classes');
  if (!box) return;
  box.textContent = '';
  if (!CAP.classes.length) {
    box.appendChild(h('p', { class: 'hint' }, '类别表未下发。'));
    return;
  }
  CAP.classes.forEach((c, i) => {
    box.appendChild(h('button', {
      class: 'cap-class' + (c.name === CAP.sel ? ' active' : ''),
      'data-class': c.name,
      title: '快捷键 ' + (i + 1),
      onclick: () => capSelect(c.name),
    }, h('b', {}, c.label), h('span', { class: 'count' }, capCountText(c.name))));
  });
}

/* 只刷计数文本、不重建按钮（轮询与采集后调用） */
function capRenderCounts() {
  const box = document.getElementById('cap-classes');
  if (!box) return;
  const btns = box.querySelectorAll('.cap-class');
  if (btns.length !== CAP.classes.length) { capRenderClasses(); return; }
  CAP.classes.forEach((c, i) => {
    const span = btns[i].querySelector('.count');
    if (span) span.textContent = capCountText(c.name);
    btns[i].classList.toggle('active', c.name === CAP.sel);
  });
}

function capSelect(name) {
  if (!CAP.classes.some(c => c.name === name)) return;
  CAP.sel = name;
  capRenderCounts();
  capRun('prefs', { class: name }, { quiet: true, silent: true });
}

function capSelectByIndex(i) {
  const c = CAP.classes[i];
  if (c) capSelect(c.name);
}

function capStatusText() {
  if (!CAP.enabled) return '已关闭（摄像头未被占用）';
  switch (CAP.state) {
    case 'ok': return CAP.ready ? '摄像头就绪' : '正在取第一帧…';
    case 'disconnected': return '摄像头已断开，等待重新接入…';
    case 'connecting': return '正在连接摄像头…';
    case 'starting': return '正在查找摄像头…';
    case 'paused': return '已暂停（扫描设备中）';
    default: return '摄像头未启动（先选一个摄像头）';
  }
}

function capRenderStatus() {
  const txt = document.getElementById('cap-status-text');
  if (!txt) return;
  const ok = CAP.enabled && (CAP.state === 'ok') && CAP.ready;
  /* 帧超过 2.5s 没来 = 画面已不可信, 即便状态还写着 ok 也要遮上 */
  const stale = ok && CAP.lastFrameAt > 0 && (Date.now() - CAP.lastFrameAt > 2500);
  const text = stale ? '画面已停滞，等待恢复…' : capStatusText();
  txt.textContent = text;
  const overlayText = document.getElementById('cap-overlay-text');
  if (overlayText) overlayText.textContent = text;
  const led = document.getElementById('cap-led');
  if (led) led.className = 'led ' + ((ok && !stale) ? 'on' : 'off');
  const overlay = document.getElementById('cap-offline');
  if (overlay) overlay.classList.toggle('hidden', ok && !stale);
  const shot = document.getElementById('cap-shot');
  if (shot) shot.disabled = !CAP.enabled || !CAP.ready;
  const autoBtn = document.getElementById('cap-auto');
  if (autoBtn) autoBtn.disabled = !CAP.enabled;
  const res = document.getElementById('cap-res');
  if (res) {
    res.textContent = (CAP.index === null || CAP.index === undefined) ? ''
      : ('#' + CAP.index + (CAP.resolution ? ' · ' + CAP.resolution : ''));
  }
  document.querySelectorAll('#cap-cameras .cap-cam').forEach(el =>
    el.classList.toggle('active', String(el.dataset.index) === String(CAP.index)));
}

function capRenderCams() {
  const box = document.getElementById('cap-cameras');
  if (!box) return;
  box.textContent = '';
  if (!CAP.cams.length) {
    box.appendChild(h('p', { class: 'hint' }, '没扫到摄像头。确认相机插好，点「重新扫描」。'));
    return;
  }
  for (const cam of CAP.cams) {
    const card = h('button', {
      class: 'cap-cam' + (String(cam.index) === String(CAP.index) ? ' active' : ''),
      'data-index': cam.index,
      title: '切到这个摄像头',
      onclick: () => capOpen(cam.index),
    });
    card.appendChild(cam.thumb
      ? h('img', { src: 'data:image/jpeg;base64,' + cam.thumb, alt: 'cam ' + cam.index })
      : h('div', { class: 'noimg' }, '无预览'));
    card.appendChild(h('b', {}, '摄像头 #' + cam.index));
    card.appendChild(h('span', {}, cam.resolution || 'unknown'));
    box.appendChild(card);
  }
}

/* ---- 定时器 ---- */

function capStartTimers() {
  if (!CAP.available || !CAP.enabled) return;     /* 关掉时一个定时器都不起, 也就一个请求都不发 */
  capStartPump();
  if (!CAP.statusTimer) CAP.statusTimer = setInterval(capStatusTick, 2000);
}

/* 总开关: 关 → 让服务端**释放设备**并停掉全部定时器; 开 → 接回上次那个摄像头。
 * 状态存 cap.json（服务端), 所以下次开控制台仍然生效 —— 这也意味着关掉之后
 * 控制台重启也不会自动去抢摄像头。 */
async function capSetEnabled(on, quiet) {
  CAP.enabled = !!on;
  const cb = document.getElementById('cap-enable');
  if (cb) cb.checked = CAP.enabled;
  capRun('prefs', { enabled: CAP.enabled }, { quiet: true, silent: true });
  if (!CAP.enabled) {
    capStopAuto();
    capStopTimers();
    await capRun('close', {}, { quiet: quiet, silent: quiet });
    capClearView();
    capLog('已关闭采集：摄像头设备已释放，其它程序可以用它了');
    return;
  }
  capLog('已启用采集');
  capStartTimers();
  const last = CAP.prefs.index;                   /* 接回上次那个摄像头, 没有就重扫 */
  if (typeof last === 'number') capOpen(last, quiet);
  else capScan(true);
}

/* 关掉后画面必须清掉: 留着最后一帧会让人以为摄像头还开着 */
function capClearView() {
  const c = document.getElementById('cap-canvas');
  if (c) c.getContext('2d').clearRect(0, 0, c.width, c.height);
  const fps = document.getElementById('cap-fps');
  if (fps) fps.textContent = '-- fps';
  CAP.fps = { n: 0, t: 0 };
  CAP.lastFrameAt = 0;
  capRenderStatus();
}

/* 离开「采集」Tab 时把帧泵/轮询/自动采集全停掉（switchTab 调用; 各 tick 里还有双保险）。
 * ⚠️ 「停自动采集」与「停整个 Tab」是两件事: 后者才清全部。 */
function capStopTimers() {
  capStopAuto();
  if (CAP.statusTimer) { clearInterval(CAP.statusTimer); CAP.statusTimer = null; }
  if (CAP.frameTimer) { clearTimeout(CAP.frameTimer); CAP.frameTimer = null; }
  CAP_IMG.onload = null;        /* 掐掉在途请求的回调, 免得它回来又续上泵 */
  CAP_IMG.onerror = null;
  CAP.lastFrameAt = 0;
}

function capStopAuto() {
  if (CAP.autoTimer) { clearInterval(CAP.autoTimer); CAP.autoTimer = null; }
  const btn = document.getElementById('cap-auto');
  if (btn) { btn.textContent = '▶ 开始自动 [A]'; btn.classList.remove('primary'); }
}

async function capStatusTick() {
  if (curTab !== 'capture' || !CAP.enabled) { capStopTimers(); return; }  /* 双保险 */
  await capFetchState(true);
}

/* ---- 帧泵: 单帧轮询画进 canvas ---- */

function capStartPump() {
  if (!CAP.available || !CAP.enabled || CAP.frameTimer) return;
  capPump();
}

function capPump() {
  CAP.frameTimer = null;
  if (curTab !== 'capture' || !CAP.enabled) { capStopTimers(); return; }
  CAP_IMG.onload = () => {
    if (curTab !== 'capture' || !CAP.enabled) return;   /* 期间切走/关掉了: 别再续泵 */
    capDraw(CAP_IMG);
    CAP.frameTimer = setTimeout(capPump, CAP.pollMs);
  };
  CAP_IMG.onerror = () => {                    /* 无帧/未就绪/断线 → 慢速重试 */
    if (curTab !== 'capture' || !CAP.enabled) return;
    capRenderStatus();
    CAP.frameTimer = setTimeout(capPump, 500);
  };
  CAP_IMG.src = '/api/capture/frame.jpg?t=' + Date.now();   /* ?t= 破缓存（服务端会剥查询串） */
}

function capDraw(img) {
  const c = document.getElementById('cap-canvas');
  if (!c) return;
  if (c.width !== img.naturalWidth || c.height !== img.naturalHeight) {
    c.width = img.naturalWidth; c.height = img.naturalHeight;
  }
  c.getContext('2d').drawImage(img, 0, 0);
  CAP.lastFrameAt = Date.now();
  capCountFps();
  capRenderStatus();
}

function capCountFps() {
  const f = CAP.fps;
  f.n++;
  const now = Date.now();
  if (!f.t) { f.t = now; return; }
  if (now - f.t >= 1000) {
    const el = document.getElementById('cap-fps');
    if (el) el.textContent = Math.round(f.n * 1000 / (now - f.t)) + ' fps';
    f.n = 0; f.t = now;
  }
}

/* ---- 动作 ---- */

async function loadCapture() {
  CAP.loaded = true;
  await capFetchState(false);
  if (!CAP.available) return;              /* 面板已换成 pip 提示, 不启任何定时器 */
  if (!CAP.enabled) { capRenderStatus(); return; }   /* 关着: 一个定时器都不起, 不碰摄像头 */
  capStartTimers();
  await capScan(true);                     /* 首次进入自动扫一次 */
  /* 服务端自己不会抢摄像头 —— 那把开关在这里: 记住上次用的 index 就自动接上 */
  const last = CAP.prefs.index;
  if (typeof last === 'number') capOpen(last, true);
}

async function capScan(quiet) {
  const btn = document.getElementById('cap-rescan');
  if (btn) btn.disabled = true;
  if (!quiet) capLog('正在扫描摄像头（画面会暂停几秒）…');
  const data = await capRun('scan', {}, { quiet: true, silent: quiet });
  if (btn) btn.disabled = false;
  CAP.cams = (data && data.cameras) || [];
  capRenderCams();
  if (!quiet) capLog('找到 ' + CAP.cams.length + ' 个摄像头。');
}

async function capOpen(index, quiet) {
  if (!CAP.enabled) {                          /* 关着的时候点摄像头卡片: 别偷偷开设备 */
    if (!quiet) toast('先勾上「启用」再选摄像头', 'err');
    return;
  }
  const data = await capRun('open', { index: index }, { quiet: quiet, silent: quiet });
  if (data === null) return;
  CAP.index = data.index;
  capRenderStatus();
  /* 等它真的开起来（最多 20s）。换机后的画面靠下一帧轮询自然切过去, 不用重连任何流。 */
  for (let i = 0; i < 40; i++) {
    await new Promise(r => setTimeout(r, 500));
    if (curTab !== 'capture') return;
    const st = await capFetchState(true);
    if (!st) return;
    if (st.state === 'ok' || st.state === 'disconnected') break;
  }
}

async function capShot() {
  if (!CAP.enabled) return;
  const now = Date.now();
  if (now - CAP.lastShot < 60) return;      /* 空格连按 / 自动采集撞车的防抖 */
  CAP.lastShot = now;
  const data = await capRun('capture', { class: CAP.sel });
  if (data === null) return;
  const btn = document.getElementById('cap-shot');
  if (btn) { btn.classList.add('flash'); setTimeout(() => btn.classList.remove('flash'), 180); }
  CAP.counts = data.counts || CAP.counts;
  capRenderCounts();
}

function capToggleAuto() {
  if (!CAP.enabled) { toast('采集已关闭', 'err'); return; }
  if (CAP.autoTimer) { capStopAuto(); toast('自动采集已停止'); return; }
  const inp = document.getElementById('cap-interval');
  const sec = parseFloat((inp && inp.value) || '0');
  if (!(sec > 0)) { toast('自动采集间隔要大于 0 秒', 'err'); return; }
  CAP.interval = sec;
  capRun('prefs', { interval: sec }, { quiet: true, silent: true });
  CAP.autoTimer = setInterval(() => {
    if (curTab !== 'capture') { capStopTimers(); return; }
    capShot();
  }, Math.max(100, sec * 1000));
  const btn = document.getElementById('cap-auto');
  if (btn) { btn.textContent = '⏸ 停止自动 [A]'; btn.classList.add('primary'); }
  capLog('自动采集已开始：每 ' + sec + ' 秒一张');
}

/* ==================== 多客户端同步（局域网多人同时编辑） ====================
 * 服务端绑 0.0.0.0 之后，一台机器上的控制台可以同时被几个浏览器打开。轮询
 * GET /api/rev（三个版本戳的 stat，与 btPollConn 同一档开销）来发现「队友改了文件」：
 *
 *   · 当前 Tab **干净** → 自动重载（等价于点「↺ 重载」），并提示一句。
 *   · 当前 Tab **有未保存改动 / 正在操作** → 只挂常驻提示条 #notice（带「↺ 重载」按钮），
 *     **绝不覆盖**。这个「待重载」不另存布尔量 —— 版本戳本身就记着它，下一次轮询
 *     比对不上就会重新评估。
 *   · **非当前 Tab 一概不碰**：隐藏的 pane 里 getBoundingClientRect 量出来全是 0，
 *     重建画布会留下一堆错位的占位槽；那边的「过期」由版本戳记着，切过去时
 *     revCheckNow() 会重判（switchTab 末尾调用）。
 *
 * ⚠️ 判据不能用 `!==`，要用「服务端的戳**严格更新**」：轮询是异步的，保存之后还可能
 *    收到一个**保存之前**发出的响应，`!==` 会把自己刚写的东西判成「远端改动」白重载一次。
 * ⚠️ 光看 dirty 不够，见 revBlocked。
 */
const REV_POLL_MS = 5000;
const REV_EPS = 1e-6;                      /* mtime 是浮点; 口径同服务端的 base_version 比较 */
const REV_FILE = { map: 'map.c', tuning: 'app_config.h', arm: 'app_arm_action.c',
                   chact: 'app_chassis_action.c', oled: 'app_oled_menu.c',
                   calib: 'calib.json' };

const REV = {
  last: null,        // 最近一次 /api/rev 的响应（切 Tab 时拿它重判, 不再发请求）
  scriptH: null,     // 已知的 app_script.h 版本戳（积木库元数据）
  notice: null,      // 提示条当前指向哪个 Tab（null = 没显示）
  muted: {},         // 「稍后」按下时的 {tab: 戳} —— 只静音**这一次**改动
  timer: null,
};

/* 指针是否按着。拖拽/拖滑杆/长按微调都是 pointer 驱动, 期间重建 DOM 会把正在拖的东西
 * 弄丢。用「置位/复位」而不是计数: 每次 up/cancel/blur 都归零, 计数漏一次就永久失灵。 */
let ptrDown = false;

function revEditing(tab) {
  return tab === 'map' || tab === 'tuning' || tab === 'arm' || tab === 'chact'
      || tab === 'oled' || tab === 'calib';
}

function revLoaded(tab) {
  if (tab === 'map') return true;                 /* init() 里就 loadAll() 了 */
  if (tab === 'tuning') return TUNE.loaded;
  if (tab === 'chact') return CHACT.loaded;
  if (tab === 'oled') return OLED.loaded;
  if (tab === 'calib') return CALIB.loaded;
  return ARM.loaded;
}

/* 本机已知的戳: 各 Tab 自己的字段, 保存成功后都会更新（别在这里另存一份） */
function revKnown(tab) {
  if (tab === 'map') return S.version;
  if (tab === 'tuning') return TUNE.version;
  if (tab === 'chact') return CHACT.version;      /* {c, h} */
  if (tab === 'oled') return OLED.version;        /* {c, h, glyphs} */
  if (tab === 'calib') return CALIB.version;      /* calib.json 的 mtime */
  return ARM.version;                             /* {c, h} */
}

function revNewer(tab, now, seen) {
  if (now == null || seen == null) return false;
  /* 机械臂/底盘动作组与 OLED 的戳都是「一个文件一个 mtime」的对象 */
  if (tab === 'arm' || tab === 'chact' || tab === 'oled') {
    const keys = tab === 'oled' ? ['c', 'h', 'glyphs'] : ['c', 'h'];
    return keys.some(k => now[k] != null && seen[k] != null && now[k] > seen[k] + REV_EPS);
  }
  return now > seen + REV_EPS;
}

/* 「稍后」= 把**这一个**戳静音; 对方再保存一次（戳又变新了）提示条自己回来 */
function revMuted(tab) {
  const m = REV.muted[tab];
  return m != null && !revNewer(tab, REV.last ? REV.last[tab] : null, m);
}

/* 现在能不能安全地重建这个 Tab 的 DOM —— 返回空串表示可以, 否则返回原因（给提示条用）。
 * ⚠️ 只判 dirty 是不够的, 有两个「dirty 还是 false 但正在交互」的窗口:
 *   ① 拖拽只在**落座**时才标记（makeDnd 的 beforeChange）→ 拖动进行中 dirty 仍是 false;
 *   ② 地图的参数框与检查器文本框是 onchange（**失焦**才触发）→ 只打字不失焦也不置脏。 */
function revBlocked(tab) {
  const a = document.activeElement;
  const pane = document.getElementById('pane-' + tab);
  if (a && pane && pane.contains(a)
      && (a.tagName === 'INPUT' || a.tagName === 'TEXTAREA' || a.tagName === 'SELECT')) {
    return '正在输入';
  }
  if (ptrDown) return '正在拖拽';
  /* 五个弹窗（元数据/调参/短语/选参/地图弹窗）都在 pane 外 —— 开着时也别重建,
   * 不然弹窗正开着、底下的画布被换了一版, 关掉弹窗会有一瞬间错位感 */
  for (const mid of ['modal', 'tmodal', 'pmodal', 'paramodal', 'mapmodal']) {
    const m = document.getElementById(mid);
    if (m && !m.classList.contains('hidden')) return '弹窗打开中';
  }
  /* 序列播放没有指针参与, 但它会一直被 $B,POS 回读刷滑杆 —— 同样别重建 */
  if (BT.playing) return '正在播放动作';
  return '';
}

function revReload(tab, auto) {
  const what = REV_FILE[tab] + ' 已被其他客户端修改，重载将丢弃你的未保存改动，继续？';
  (async () => {
    try {
      if (tab === 'map') {
        if (S.dirty && !confirm(what)) return;
        await loadAll();
      } else if (tab === 'tuning') {
        if (Object.keys(TUNE.changed).length && !confirm(what)) return;
        await loadTuning();
      } else if (tab === 'arm') {
        if (ARM.dirty && !confirm(what)) return;
        await armReload();
      } else if (tab === 'chact') {
        if (CHACT.dirty && !confirm(what)) return;
        CHACT.dirty = false;
        await loadChact();
      } else if (tab === 'oled') {
        if (OLED.dirty && !confirm(what)) return;
        await loadOled();
      } else if (tab === 'calib') {
        if (CALIB.dirty && !confirm(what)) return;
        CALIB.dirty = false;
        await loadCalib();
      } else return;
    } catch (e) {
      toast('重载失败: ' + e.message, 'err');
      return;
    }
    REV.notice = null;
    revNoticeSync();
    toast(auto ? REV_FILE[tab] + ' 已被其他客户端修改，已自动重载'
               : '已从磁盘重载 ' + REV_FILE[tab]);
    revCheckNow();
  })();
}

/* 积木库（app_script.h）变了 = 别人用「右键积木 → 编辑元数据」改了说明/中文名/选项。
 * reloadCatalog 只换 S.catalog, **不动画布上的编辑**（S.presets 是内存里的编辑缓冲）,
 * 所以脏着也能刷; 但它内部会 renderCanvas → 仍然受「别在拖动中重建 DOM」约束。 */
function revCatalog(j) {
  if (j.script_h == null) return;
  if (REV.scriptH == null) { REV.scriptH = j.script_h; return; }   /* 首次只记基线 */
  if (!(j.script_h > REV.scriptH + REV_EPS)) return;
  if (curTab !== 'map') return;            /* 别的 Tab 上别动地图 DOM; 切过去时 revCheckNow 补上 */
  if (revBlocked('map')) return;           /* 拖动/输入中 → 下一轮再说（戳没更新, 还会再来） */
  REV.scriptH = j.script_h;
  reloadCatalog();
  toast('积木库元数据已被其他客户端修改，已刷新');
}

function revEvaluate(j) {
  REV.last = j;
  revCatalog(j);
  revCalibCfg(j);
  const tab = revEditing(curTab) && revLoaded(curTab) ? curTab : null;
  if (!tab) { REV.notice = null; revNoticeSync(); return; }
  if (!revNewer(tab, j[tab], revKnown(tab))) {
    if (REV.notice) { REV.notice = null; revNoticeSync(); }     /* 追平了 → 提示条自己消失 */
    return;
  }
  const dirty = tab === 'map' ? S.dirty
              : tab === 'tuning' ? TUNE.dirty
              : tab === 'chact' ? CHACT.dirty
              : tab === 'oled' ? OLED.dirty
              : tab === 'calib' ? CALIB.dirty
              : ARM.dirty;
  if (revBlocked(tab) || dirty) { REV.notice = tab; revNoticeSync(); return; }
  revReload(tab, true);
}

/* 「标定」Tab 的第二个数据源: 参数元数据与「当前宏值」列都来自 app_config.h ——
 * 别的客户端在「调参」改了值 / 另一台机 apply 了预设（tuning 戳变新）也要静默刷新,
 * 否则那一列陈旧。calib.json 自己变新时不用管（主路径的 revReload 会整份重载）;
 * 自己在标定 Tab apply 后已把 cfgVersion 回填成响应里的新戳, 不会被这条误伤。 */
function revCalibCfg(j) {
  if (curTab !== 'calib' || !CALIB.loaded) return;
  if (revNewer('calib', j.calib, CALIB.version)) return;   /* json 变了走主路径 */
  if (j.tuning == null || CALIB.cfgVersion == null) return;
  if (!(j.tuning > CALIB.cfgVersion + REV_EPS)) return;
  if (CALIB.dirty || revBlocked('calib')) return;
  CALIB.cfgVersion = j.tuning;
  loadCalib();
}

async function revPoll() {
  let j;
  try {
    j = await (await fetch('/api/rev')).json();
  } catch (e) {
    return;                                /* 服务端重启中/断网 → 静默, 下一轮再说 */
  }
  revEvaluate(j);
}

/* 用缓存的那份响应立刻重判（切 Tab / 重载完之后调）—— 不再发请求 */
function revCheckNow() { if (REV.last) revEvaluate(REV.last); }

function revNoticeSync() {
  const el = document.getElementById('notice');
  if (!el) return;
  const tab = REV.notice;
  if (!tab || tab !== curTab || revMuted(tab)) { el.classList.add('hidden'); return; }
  document.getElementById('notice-text').textContent =
    REV_FILE[tab] + ' 已被其他客户端修改 —— 你这边' +
    (revBlocked(tab) || '有未保存改动') + '，没有自动重载（你的改动不会被覆盖）。';
  document.getElementById('notice-reload').textContent = '↺ 重载 ' + REV_FILE[tab];
  el.classList.remove('hidden');
}

function revInit() {
  document.addEventListener('pointerdown', () => { ptrDown = true; }, true);
  document.addEventListener('pointerup', () => { ptrDown = false; }, true);
  document.addEventListener('pointercancel', () => { ptrDown = false; }, true);
  window.addEventListener('blur', () => { ptrDown = false; });
  document.getElementById('notice-reload').addEventListener('click', () => {
    const tab = REV.notice || (revEditing(curTab) ? curTab : null);
    if (tab) revReload(tab, false);
  });
  document.getElementById('notice-dismiss').addEventListener('click', () => {
    /* 「稍后」只藏提示条, **不丢待办**: 记下被静音的戳。仍在编辑 → 下一轮戳没变、
     * 已知值没追平 → muted 让它保持安静; 对方再改一次提示条自己回来。 */
    if (REV.notice) REV.muted[REV.notice] = REV.last ? REV.last[REV.notice] : true;
    revNoticeSync();
  });
  REV.timer = setInterval(revPoll, REV_POLL_MS);
}

/* ==================== Tab「OLED」 ====================
 * 编辑 Application/app_oled_menu.c 的数据区（OLED 菜单的表）。
 * 与「地图」Tab 同构: 左侧积木库 → 拖/点进画布 → 保存 POST /api/oled。
 *
 * · **积木库里的块 = 动作码**（服务端从 app_oled_menu.h 的枚举派生, 前端不写死）。
 *   块名取枚举的尾注释 —— **拖进来之后就正是它在 OLED 上显示的那行字**。
 *   想要「陀螺目标」而不是「调参」, 把标签改掉即可（标签始终可编辑）。
 * · 一页 / 一个子菜单最多 **16** 个启用项; 一屏只显示 4 行, 放不下的靠 UP/DOWN **滚动**
 *   查看（右侧画竖直滚动条）。子菜单的「返回」由固件补在**最后一条**、随列表滚动。
 * · **预览按 glyphs.txt 的覆盖率画**: 字表里没有的汉字画成「方框问号」, 与车上的
 *   vOledShowChinese 兜底图形一致 ⇒ **预览即验收**。
 */

const OLED = {
  loaded: false,
  version: null,        /* {c, h, glyphs} —— 保存时回传的 base_version */
  order: [],            /* 表符号名的文件顺序（服务端保存时会做稳定拓扑排序） */
  lists: {},            /* name -> {title, array?, items:[...]} */
  pages: [],            /* [{kind, list}] —— 前两条恒为状态页/二维码页 */
  options: {},          /* 枚举下拉（服务端从源码派生） */
  argEnum: {},          /* 动作码 -> 该动作的参数取自哪个枚举 */
  params: { beam: [], pi: [], names: [] },   /* 「参数」弹窗的清单（服务端现扫两张参数表） */
  missing: [],          /* 用到但字表还没列出的汉字 */
  sel: null,            /* 当前编辑的表名 */
  selItem: null,        /* 当前选中的积木 id */
  dirty: false,
  seq: 0,               /* 生成条目 id 用 */
};

const OLED_W = 128, OLED_ROW_H = 16;      /* 与固件 OLED_W / OLED_Y1..Y4 / OLED_ROW_H 对齐 */
/* 与固件同源同义（app_oled.c）: 一页/子菜单最多 OLED_MAX_ITEMS 项, 一屏显示 OLED_WIN 行,
 * 放不下的靠滚动; 最右 OLED_BAR_W 像素是滚动条的预留带 ⇒ 文字预算 OLED_LABEL_W。 */
const OLED_MAX_ITEMS = 32, OLED_WIN = 4, OLED_BAR_W = 8;
const OLED_LABEL_W = OLED_W - OLED_BAR_W;   /* 120 —— 一行文字实际能用的宽度 */
const OLED_ACT_NONE = 'OLED_ACT_NONE';
const OLED_ACT_SUB = 'OLED_ACT_ENTER_SUB';
/* 按**参数名**调的调参行（值在 Beam 表或 Pi 参数表里, 见 app_oled.c::vOledTuneParamStep）。
 * ⚠️ 这个常量是**跨文件契约**（oled_menu.py::ACT_TUNE_PARAM 同名）—— 改一处要改两处。 */
const OLED_ACT_PARAM = 'OLED_ACT_TUNE_PARAM';

/* 汉字判据 —— 与固件（OLED_CHN_CHAR_WIDTH / gen_font.is_han）同一套区间 */
function oledIsHan(ch) {
  const c = ch.codePointAt(0);
  return (c >= 0x4E00 && c <= 0x9FFF) || (c >= 0x3400 && c <= 0x4DBF)
      || (c >= 0x3000 && c <= 0x303F) || (c >= 0xFF00 && c <= 0xFFEF);
}

/* 一行文字的像素宽（汉字 16px、其余 8px）—— 与固件 ucOledDrawMixed 的步进一致 */
function oledWidth(s) {
  let w = 0;
  for (const ch of String(s || '')) w += oledIsHan(ch) ? 16 : 8;
  return w;
}

function oledShort(act) { return String(act || '').replace('OLED_ACT_', ''); }
function oledOptionsFor(enumName) { return (OLED.options && OLED.options[enumName]) || []; }
function oledArgEnumOf(act) { return OLED.argEnum[act] || ''; }
function oledIsPageList(name) { return OLED.pages.some(p => p.list === name); }
function oledIsSub(name) { return !oledIsPageList(name); }
function oledListTitle(name) { return (OLED.lists[name] && OLED.lists[name].title) || name; }

/* ---------------------------------------------------------------- 参数清单
 * 「参数」弹窗的数据 = 服务端现扫出来的两张表（Beam 表 / Pi 参数表, 见 oled_menu.py::
 * param_catalog）。这里只做两件小事: 名字 → 条目、条目 → 一句人话。 */
function oledParamOf(name) {
  const cat = OLED.params || {};
  for (const src of ['beam', 'pi']) {
    const hit = (cat[src] || []).find(p => p.name === name);
    if (hit) return hit;
  }
  return null;
}

/* 「参数」那一格的提示语: 来源 + 单位 + 只读/选项。 */
function oledParamHint(p) {
  if (!p) return '参数不在清单里（固件会显示 ? 并长鸣）—— 重新选一个';
  const bits = [p.src === 'pi' ? '树莓派' : '本机(Beam)'];
  if (p.unit) bits.push('单位 ' + p.unit);
  if (p.ro) bits.push('只读');
  if (p.options) bits.push('选项 ' + p.options);
  if (p.kind === 'str') bits.push('字符串（屏上只看, 不能调）');
  if (p.warn) bits.push('运行中改会影响控制律');
  return bits.join(' · ');
}

/* 运行期会把静态标签换掉的那几个动作 —— 预览只能画静态标签, 这里给一句提示 */
const OLED_RUNTIME_LABEL = {
  OLED_ACT_LASER_TOGGLE: '运行期标签带「:开 / :关」',
  OLED_ACT_LINE_TEST: '运行期标签是「开始 / 停止巡线测试」',
  OLED_ACT_PHOTO: '按下去会临时显示「拍照 成功! / 失败!」',
  OLED_ACT_TUNE_QTY: '固件会在标签后面拼上「:值」',
  OLED_ACT_TUNE_PARAM: '值来自「参数」那一格（Beam 表 / 树莓派），左右键改；只读项长鸣',
};

/* ---------------------------------------------------------------- 条目 id */

/* 每个条目一个稳定 id: DOM id、拖拽落点、删除定位都靠它（同机械臂 Tab 那条纪律） */
function oledEnsureIds() {
  for (const nm of OLED.order) {
    const l = OLED.lists[nm];
    if (!l) continue;
    for (const it of l.items) if (!it.id) it.id = 'oi' + (++OLED.seq);
  }
}

function oledFindItem(id) {
  for (const nm of OLED.order) {
    const l = OLED.lists[nm];
    if (!l) continue;
    const hit = l.items.find(i => i.id === id);
    if (hit) return hit;
  }
  return null;
}

function oledItemList(id) {
  for (const nm of OLED.order) {
    const l = OLED.lists[nm];
    if (l && l.items.some(i => i.id === id)) return l;
  }
  return null;
}

/* 选中一块 —— **就地改 class, 不重建 DOM**。
 * ⚠️ 两条理由: ① 重建会把**正在按下**的那个元素换掉, 拖动接不下去;
 *    ② 每点一下就整片重绘也太重。 */
function oledSelectItem(id) {
  OLED.selItem = id;
  const inner = document.getElementById('oled-canvas-inner');
  if (inner && typeof inner.querySelectorAll === 'function') {
    for (const el of inner.querySelectorAll('.block.sel')) el.classList.remove('sel');
    const el = document.getElementById('blk-oled-' + id);
    if (el) el.classList.add('sel');
  }
  oledRenderPreview();          /* 预览那边把选中那行画成反色, 跟着走 */
}

/* ---------------------------------------------------------------- 积木库 */

/* 块 = 动作码, 块名 = 枚举尾注释（也就是拖进来之后的默认标签） */
function oledPalette() {
  return oledOptionsFor('emOledActTdf')
    .filter(o => o.value !== OLED_ACT_NONE)          /* 「空槽」不是真动作, 不给拖 */
    .map(o => ({
      type: o.value,
      name: o.label || oledShort(o.value),
      arg: oledArgEnumOf(o.value),
    }));
}

function oledPaletteOf(type) { return oledPalette().find(p => p.type === type) || null; }

function oledNewItemFromAct(act) {
  const p = oledPaletteOf(act);
  const it = {
    id: 'oi' + (++OLED.seq), act: act,
    label: p ? p.name : oledShort(act),
    enabled: true, arg: null, sub: null, param: null, edited: true,
  };
  const ae = oledArgEnumOf(act);
  if (ae) {
    const opts = oledOptionsFor(ae);
    /* ⚠️ 跳过 `*_NONE` 这类**哨兵值**: 预设枚举的第一项是 `SCRIPT_PRESET_NONE`
     *（"不跳预设"）, 预填它 = 静默拿到一个空指向, 排表的人在车上才会发现。 */
    const real = opts.filter(o => !/_NONE$/.test(String(o.value)));
    const first = real.length ? real[0] : null;
    if (first) it.arg = first.value;
  }
  /* 通用调参行: **不预填**参数名 —— 随便挑一个（比如第一个 Beam 参数）只会让人
   * 拿到一个"看着配好了"的死项; 空着时那一格显示「（点这里选参数）」, 保存校验
   * 也会拦住（oled_menu.py::_check 的"没选参数"）。 */
  if (act === OLED_ACT_PARAM) it.param = '';
  /* 拖「子菜单」进来**总是新建一张表** —— 复用现有的那张会让两块指向同一张表
   *（看不见的耦合）。想指向已有的, 用下面那个下拉挑。 */
  if (act === OLED_ACT_SUB) it.sub = oledAddSub();
  return it;
}

/* ---------------------------------------------------------------- 表 / 页面 */

/* 新表插在**当前表之前** —— 子表必须写在引用它的表之前（C 要求符号先声明）。
 * 服务端保存时还会做一次稳定拓扑排序兜底, 所以这里插错位也不会写出坏文件。 */
function oledNewListName(prefix) {
  let n = 1;
  while (OLED.lists[prefix + n]) n++;
  return prefix + n;
}

function oledAddSub() {
  const name = oledNewListName('s_stListSub');
  OLED.lists[name] = {
    title: '新子菜单', array: 's_astOledMenu' + name.slice('s_stList'.length),
    edited: true,
    items: [{ id: 'oi' + (++OLED.seq), label: '占位', act: 'OLED_ACT_NOP',
              enabled: true, arg: null, sub: null, edited: true }],
  };
  const at = OLED.sel ? Math.max(0, OLED.order.indexOf(OLED.sel)) : OLED.order.length;
  OLED.order.splice(at, 0, name);
  oledMarkDirty();
  return name;
}

function oledAddPage() {
  const name = oledNewListName('s_stListPage');
  OLED.lists[name] = {
    title: '新菜单页', array: 's_astOledMenu' + name.slice('s_stList'.length),
    edited: true,
    items: [{ id: 'oi' + (++OLED.seq), label: '占位', act: 'OLED_ACT_NOP',
              enabled: true, arg: null, sub: null, edited: true }],
  };
  OLED.order.push(name);
  OLED.pages.push({ kind: 'OLED_PAGEKIND_MENU', list: name, edited: true });
  OLED.sel = name;
  oledMarkDirty();
  oledRenderAll();
}

/* 页面排序: 第 idx 页左右挪一格。
 * ⚠️ 前两条（状态页 / 二维码页）是**跨文件契约**（固件按 kind 分派, 控制台保证 0/1 固定）
 *    —— 菜单页只能排它们之后。
 * ⚠️ 只需要动 pages 数组的**顺序**: 页面表是按数组顺序写出去的, 而每一条的 raw 行本身
 *    没变（`{ (uint8_t)..., &s_stListXxx }` 与位置无关）, 所以不用标 edited。 */
function oledMovePage(idx, dir) {
  const j = idx + dir;
  if (j < 2) { toast('状态页 / 二维码页固定在最前两页'); return; }
  if (j >= OLED.pages.length) return;
  const t = OLED.pages[idx];
  OLED.pages[idx] = OLED.pages[j];
  OLED.pages[j] = t;
  oledMarkDirty();
  oledRenderAll();
}

/* 删一张表: 连同引用它的页面与积木一起收拾掉（不然会留下指不到东西的引用） */
function oledDeleteList(name) {
  const refs = [];
  if (oledIsPageList(name)) refs.push('一个页面');
  for (const nm of OLED.order) {
    if (nm === name || !OLED.lists[nm]) continue;
    if (OLED.lists[nm].items.some(i => i.sub === name)) refs.push('「' + oledListTitle(nm) + '」里的一块「子菜单」');
  }
  if (refs.length && !confirm('「' + oledListTitle(name) + '」正被 ' + refs.join('、') +
                              ' 引用，一起删掉？')) return;
  OLED.pages = OLED.pages.filter(p => p.list !== name);
  for (const nm of OLED.order) {
    if (nm === name || !OLED.lists[nm]) continue;
    OLED.lists[nm].items = OLED.lists[nm].items.filter(i => i.sub !== name);
    OLED.lists[nm].edited = true;
  }
  OLED.order = OLED.order.filter(n => n !== name);
  delete OLED.lists[name];
  if (OLED.sel === name) {
    const p = OLED.pages.find(x => x.list);
    OLED.sel = p ? p.list : (OLED.order[0] || null);
  }
  oledMarkDirty();
  oledRenderAll();
}

/* ---------------------------------------------------------------- 载入 / 保存 */

async function loadOled() {
  let j;
  try {
    j = await (await fetch('/api/oled')).json();
  } catch (e) {
    toast('读取 app_oled_menu.c 失败: ' + e.message, 'err');   /* 别 unhandled rejection 留白页 */
    return;
  }
  OLED.version = j.version;
  OLED.order = j.order;
  OLED.lists = j.lists;
  OLED.pages = j.pages;
  OLED.options = j.options || {};
  OLED.argEnum = j.arg_enum || {};
  OLED.params = j.params || { beam: [], pi: [], names: [] };
  OLED.missing = j.missing_glyphs || [];
  OLED.loaded = true;
  oledEnsureIds();
  if (!OLED.sel || !OLED.lists[OLED.sel]) {
    const p = OLED.pages.find(x => x.list);
    OLED.sel = p ? p.list : (OLED.order[0] || null);
  }
  OLED.selItem = null;
  oledClearDirty();
  oledRenderAll();
}

async function oledReload() {
  await loadOled();
  toast('已从磁盘重载 app_oled_menu.c');
  revCheckNow();
}

function oledMarkDirty() {
  OLED.dirty = true;
  document.getElementById('oled-dot').classList.remove('hidden');
}

function oledClearDirty() {
  OLED.dirty = false;
  document.getElementById('oled-dot').classList.add('hidden');
}

let _savingOled = false;
async function oledSave() {
  if (_savingOled) return;     /* 防重入（同 save） */
  _savingOled = true;
  try {
    await oledSaveInner();
  } finally {
    _savingOled = false;
  }
}

async function oledSaveInner() {
  const body = {
    base_version: OLED.version,
    order: OLED.order, lists: OLED.lists, pages: OLED.pages,
  };
  let r, j;
  try {
    r = await fetch('/api/oled', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    j = await r.json();
  } catch (e) { toast('保存失败: ' + e.message, 'err'); return; }

  if (r.status === 409) {
    toast('app_oled_menu.c 已在外部被修改，请先「↺ 重载」再保存', 'err');
    return;
  }
  if (!r.ok || !j.ok) { toast('保存失败: ' + (j.message || j.error || r.status), 'err'); return; }
  if (j.glyphs_added && j.glyphs_added.length) {
    toast('已补进字表并重生成字模: ' + j.glyphs_added.join(' '));
  }
  for (const w of (j.warnings || [])) toast(w, 'err');
  /* 重新拉一次: 服务端把未编辑的行回吐成规范格式, 页面这份 raw 已经过期 */
  await loadOled();
  toast('已保存 app_oled_menu.c —— 记得回 Keil 重新编译烧写');
  revCheckNow();
}

/* ---------------------------------------------------------------- 渲染 */

function oledRenderAll() {
  oledRenderPalette();
  oledRenderBar();
  oledRenderCanvas();
  oledRenderPreview();
}

function oledRenderPalette() {
  const list = document.getElementById('oled-palette-list');
  const q = document.getElementById('oled-palette-search').value.trim().toLowerCase();
  list.textContent = '';

  const blocks = oledPalette().filter(p =>
    !q || (p.name + ' ' + p.type).toLowerCase().includes(q));
  if (!blocks.length) {
    list.appendChild(h('p', { class: 'hint' }, '无匹配积木'));
    return;
  }

  for (const p of blocks) {
    const argHint = p.arg === 'emScriptPresetIdTdf' ? '参数: 预设'
                  : p.arg === 'emOledTuneQtyTdf' ? '参数: 调参量'
                  : p.arg === 'emTimerCmdTdf' ? '参数: 计时动作'
                  : p.type === OLED_ACT_SUB ? '点开一层子菜单' : '';
    list.appendChild(h('div', {
      class: 'pal-item',
      title: p.name + '  拖入或点击添加',
      onclick: () => {
        if (OLEDDND.justDragged()) return;
        oledAddItem(oledNewItemFromAct(p.type), null);
      },
      onpointerdown: (e) => {
        if (e.button !== 0) return;
        OLEDDND.begin({ mode: 'new', type: p.type }, e);
      },
    },
      h('div', { class: 'pal-name' }, p.name),
      h('div', { class: 'pal-desc' }, p.arg ? argHint
        : (OLED_RUNTIME_LABEL[p.type] || oledShort(p.type))),
    ));
  }
}

/* 插一块到当前表（beforeId=null 表示末尾） */
function oledAddItem(it, beforeId) {
  const l = OLED.lists[OLED.sel];
  if (!l) { toast('先选一张表', 'err'); return; }
  const pos = beforeId ? l.items.findIndex(i => i.id === beforeId) : l.items.length;
  l.items.splice(pos < 0 ? l.items.length : pos, 0, it);
  l.edited = true;
  OLED.selItem = it.id;
  oledMarkDirty();
  oledRenderAll();
}

function oledRenderBar() {
  /* ---- 页面 chips ---- */
  const chips = document.getElementById('oled-page-chips');
  chips.textContent = '';
  OLED.pages.forEach((p, idx) => {
    const locked = idx < 2;                        /* 状态页 / 二维码页是代码页 */
    const isCur = p.list && p.list === OLED.sel;
    const kindName = locked ? (idx === 0 ? '状态页' : '二维码页') : oledListTitle(p.list);
    const chip = h('div', {
      class: 'oled-chip' + (locked ? ' locked' : '') + (isCur ? ' cur' : ''),
      title: locked ? '代码页: 内容由固件画, 不能编辑' : '点一下切到这一页',
      onclick: () => {
        if (locked || !p.list) { toast('这是代码页，内容由固件画'); return; }
        OLED.sel = p.list;
        OLED.selItem = null;
        oledRenderAll();
      },
    },
      h('span', { class: 'oled-chip-no' }, String(idx + 1)),
      h('span', {}, kindName),
      /* 排序: 只在**菜单页**上给 ◀/▶，而且挪不出前两条代码页之外（那是跨文件契约）。
       * ⚠️ 没走 makeDnd: 那套是**纵向**几何（按 clientY 算落点）+ `.zone/.block` 结构，
       *    页条是横向的 —— 硬套就得改共用件, 不值当。 */
      locked ? null : h('button', {
        class: 'oled-chip-mv', title: '往左挪一格', disabled: idx <= 2 ? '' : null,
        onclick: (e) => { e.stopPropagation(); oledMovePage(idx, -1); },
      }, '◀'),
      locked ? null : h('button', {
        class: 'oled-chip-mv', title: '往右挪一格',
        disabled: idx === OLED.pages.length - 1 ? '' : null,
        onclick: (e) => { e.stopPropagation(); oledMovePage(idx, 1); },
      }, '▶'),
      locked ? null : h('button', {
        class: 'oled-chip-x', title: '删掉这一页',
        onclick: (e) => {
          e.stopPropagation();
          const nm = p.list;
          /* ⚠️ 必须问一句: 删页面会连带把它那张表也收掉（没人再引用时）, 是破坏性动作 */
          const orphan = nm && !OLED.pages.some(x => x.list === nm && x !== p)
              && !OLED.order.some(n => OLED.lists[n]
                   && OLED.lists[n].items.some(i => i.sub === nm));
          const what = '删掉第 ' + (idx + 1) + ' 页「' + oledListTitle(nm) + '」？'
              + (orphan ? '\n（它那张表没有别处引用，会一起删掉）' : '\n（它那张表还有别处引用，会保留）')
              + '\n这个改动要保存 + 重新编译烧写才会生效。';
          if (!confirm(what)) return;
          OLED.pages.splice(idx, 1);
          /* 只有没人再引用它时才顺手把表也收掉, 免得文件里堆垃圾 */
          if (nm && !OLED.pages.some(x => x.list === nm)
              && !OLED.order.some(n => OLED.lists[n] && OLED.lists[n].items.some(i => i.sub === nm))) {
            OLED.order = OLED.order.filter(n => n !== nm);
            delete OLED.lists[nm];
          }
          if (OLED.sel === nm) {
            const q = OLED.pages.find(x => x.list);
            OLED.sel = q ? q.list : (OLED.order[0] || null);
          }
          oledMarkDirty();
          oledRenderAll();
        },
      }, '✕'));
    chips.appendChild(chip);
  });

  /* ---- 表选择 ---- */
  const sel = document.getElementById('oled-list-select');
  sel.textContent = '';
  for (const nm of OLED.order) {
    sel.appendChild(h('option', { value: nm, selected: nm === OLED.sel ? '' : null },
                      oledListTitle(nm) + (oledIsSub(nm) ? '  （子菜单）' : '  （页面）')));
  }
  sel.value = OLED.sel || '';

  const box = document.getElementById('oled-list-title');
  box.textContent = '';
  const l = OLED.lists[OLED.sel];
  if (l) {
    /* 标题也是**平时静态文字、双击才变输入框** —— 与块上的标签同一套手感。
     * 常驻一个输入框会让人以为这里是"输入区"而不是"显示名"。 */
    const t = h('span', {
      class: 'oled-title', title: '双击改名 —— 这一页 / 这张表在文件里的标题（写进注释）',
    }, l.title || OLED.sel);
    t.addEventListener('dblclick', (e) => {
      e.stopPropagation();
      editInPlace(t, l.title || '', (v) => {
        l.title = v.trim() || OLED.sel;
        l.edited = true;
        oledMarkDirty();
        oledRenderBar();
      });
    });
    box.appendChild(t);
    if (oledIsSub(OLED.sel)) {
      box.appendChild(h('button', {
        class: 'btn ghost tiny', title: '删掉这张子菜单表',
        onclick: () => oledDeleteList(OLED.sel),
      }, '🗑'));
    }
  }

  /* ---- 启用计数 ---- */
  const cap = OLED_MAX_ITEMS;                 /* 页面与子菜单同一条上限（放不下的靠滚动） */
  const on = l ? l.items.filter(i => i.enabled).length : 0;
  const c = document.getElementById('oled-count');
  c.textContent = l ? ('启用 ' + on + ' / 上限 ' + cap) : '';
  c.className = 'oled-cap' + (on > cap ? ' bad' : '');
}

function oledRenderCanvas() {
  const inner = document.getElementById('oled-canvas-inner');
  inner.textContent = '';
  const l = OLED.lists[OLED.sel];
  if (!l) {
    inner.appendChild(h('p', { class: 'hint' }, '从左侧拖一块积木进来'));
    return;
  }

  const zone = h('div', { class: 'zone oled-zone' });
  for (const it of l.items) zone.appendChild(oledItemBlock(l, it));
  inner.appendChild(zone);

  /* 子菜单的「返回」由固件补在**最后一条**（随列表滚动）—— 画出来, 免得用户以为少了它 */
  if (oledIsSub(OLED.sel)) {
    inner.appendChild(h('div', { class: 'oled-ghost-row' }, '返回'));
  }
}

function oledItemBlock(l, it) {
  const isTune = it.act === 'OLED_ACT_TUNE_QTY' || it.act === OLED_ACT_PARAM;
  const capPx = isTune ? 64 : OLED_LABEL_W;      /* 调参行还要拼「:值」, 标签只有 64px */
  const w = oledWidth(it.label);

  const blk = h('div', {
    class: 'block oled-item' + (it.enabled ? '' : ' off') + (it.id === OLED.selItem ? ' sel' : ''),
    id: 'blk-oled-' + it.id,
  });

  const head = h('div', { class: 'oled-row1' });

  /* 眼睛（启用/藏起来）—— ⚠️ checked 不能塞进 h() 属性对象, 所以没用 checkbox */
  const eye = h('button', {
    class: 'oled-eye',
    title: it.enabled ? '点一下藏起来（不显示、导航跳过）' : '点一下显示出来',
    onpointerdown: (e) => e.stopPropagation(),        /* 别让按钮点击变成拖拽 */
    onclick: (e) => {
      e.stopPropagation();
      it.enabled = !it.enabled;
      it.edited = true;
      oledMarkDirty();
      oledRenderAll();
    },
  }, it.enabled ? '👁' : '🚫');
  head.appendChild(eye);

  /* 标签: **平时是一段静态文字, 双击才变输入框**（同「地图」Tab 的改名手感）。
   * ⚠️ 宽度固定成一行的宽度（128px + 余量）—— 一个 OLED 一行最多 8 个汉字,
   *    输入框拉满整行既误导又难看。 */
  const tag = h('span', {
    class: 'oled-tag', title: '双击改名 —— 这就是 OLED 上显示的那行字',
  }, it.label || '');
  tag.addEventListener('dblclick', (e) => {
    e.stopPropagation();
    editInPlace(tag, it.label || '', (v) => {
      it.label = v;
      it.edited = true;
      oledMarkDirty();
      oledRenderAll();
    });
  });
  head.appendChild(tag);

  /* 宽度: 固件在 x≥128 是**静默截断**, 所以这里必须显式预警 */
  const over = w > capPx;
  head.appendChild(h('span', {
    class: 'oled-w' + (over ? ' bad' : ''),
    title: over ? '这一行放不下, 车上会被静默截断' : '这一行的像素宽（可用 120px, 右侧 8px 是滚动条）',
  }, w + 'px'));

  /* 类型徽标: 只在标签被改过时显示, 免得每行都重复一遍块名 */
  const p = oledPaletteOf(it.act);
  const defName = p ? p.name : oledShort(it.act);
  if (it.label !== defName) {
    const tip = OLED_RUNTIME_LABEL[it.act]
      ? (it.act + ' —— ' + OLED_RUNTIME_LABEL[it.act]) : it.act;
    head.appendChild(h('span', { class: 'oled-type', title: tip }, defName));
  }

  head.appendChild(h('button', {
    class: 'oled-x', title: '删掉这一块',
    onpointerdown: (e) => e.stopPropagation(),
    onclick: (e) => {
      e.stopPropagation();
      l.items = l.items.filter(x => x.id !== it.id);
      l.edited = true;
      if (OLED.selItem === it.id) OLED.selItem = null;
      oledMarkDirty();
      oledRenderAll();
    },
  }, '✕'));
  blk.appendChild(head);

  /* 参数行（只有需要参数的动作才有） */
  const ae = oledArgEnumOf(it.act);
  if (ae) {
    const opts = oledOptionsFor(ae);
    const s = h('select', {
      class: 'oled-arg',
      title: '这一块的参数',
      onpointerdown: (e) => e.stopPropagation(),
      onchange: (e) => { it.arg = e.target.value; it.edited = true; oledMarkDirty(); },
    });
    if (!opts.length) s.appendChild(h('option', { value: '' }, '（源码里没有 ' + ae + '）'));
    for (const o of opts) {
      s.appendChild(h('option', { value: o.value, selected: o.value === it.arg ? '' : null },
                      o.label + '（' + o.value + '）'));
    }
    if (it.arg) s.value = it.arg;
    blk.appendChild(h('div', { class: 'oled-row2' },
      h('span', { class: 'oled-cap-label' }, '参数'), s));
  } else if (it.act === OLED_ACT_PARAM) {
    /* 通用调参行: 「参数」= 一个**参数名**, 从两张参数表里选（382 项, 所以走弹窗分类选,
     * 不能塞进一个下拉）。名在清单里找不到时标红 —— 那在车上就是"显示 ? 并长鸣"。
     * 「步进」那一格按参数类型变脸（2026-10-18, 用户口径）:
     *   布尔（√/×）→ 灰显输入框, placeholder「布尔」—— 翻转不吃步进;
     *   下拉（有选项表）→ 「映射」按钮 → 弹窗编辑屏上短标签 + 拖动排序;
     *   数值/其余   → 步进输入框（旧行为, 空 = 系统步进）。 */
    const p = oledParamOf(it.param);
    const missing = !!(it.param && !p);
    const kind = oledParamValueKind(p);
    const btn = h('button', {
      class: 'oled-arg oled-param-btn' + (missing ? ' bad' : ''),
      title: oledParamHint(p),
      onpointerdown: (e) => e.stopPropagation(),
      onclick: (e) => { e.stopPropagation(); oledOpenParamPicker(it); },
    }, it.param ? (it.param + (p ? '（' + p.label + '）' : '（不在清单里!）'))
                : '（点这里选参数）');
    blk.appendChild(h('div', { class: 'oled-row2' },
      h('span', { class: 'oled-cap-label' }, '参数'), btn));
    if (kind === 'bool') {
      blk.appendChild(h('div', { class: 'oled-row2' },
        h('span', { class: 'oled-cap-label' }, '步进'),
        h('input', {
          class: 'oled-arg oled-step', value: '', placeholder: '布尔', disabled: '',
          title: '布尔量在屏上显示 √/×, 按 LEFT/RIGHT 是翻转 —— 不吃步进',
          onpointerdown: (e) => e.stopPropagation(),
        })));
    } else if (kind === 'enum') {
      const n = (it.map || '').split('|').filter(x => x).length;
      blk.appendChild(h('div', { class: 'oled-row2' },
        h('span', { class: 'oled-cap-label' }, '映射'),
        h('button', {
          class: 'oled-arg oled-param-btn' + ((it.param && !p) ? ' bad' : ''),
          title: '编辑屏上短标签与 LEFT/RIGHT 的循环顺序（值本身不变）',
          onpointerdown: (e) => e.stopPropagation(),
          onclick: (e) => { e.stopPropagation(); oledOpenMapModal(it); },
        }, it.map ? ('已映射 ' + n + ' 项: ' + it.map) : '（点这里配映射/排序）')));
    } else {
      const stepInp = h('input', {
        class: 'oled-arg oled-step',
        value: it.step || '',
        placeholder: '自动',
        title: '自定义步进: 按一下 LEFT/RIGHT 加减多少（留空 = 系统自动）',
        onpointerdown: (e) => e.stopPropagation(),
        onchange: (e) => {
          const v = e.target.value.trim();
          it.step = v || null;
          it.edited = true;
          oledMarkDirty();
        },
      });
      blk.appendChild(h('div', { class: 'oled-row2' },
        h('span', { class: 'oled-cap-label' }, '步进'), stepInp));
    }
    if (p) {
      /* 值的宽度预算（拼在「:」后面的那半截）: 数字最宽 6 ASCII = 48px; 选项标签是中文
       *（如「自动」= 32px）; 布尔量固定 16px（√/×）; str 型长度不定 —— 提示一句, 不拦。 */
      blk.appendChild(h('div', { class: 'oled-row2 oled-param-hint' },
        h('span', { class: 'oled-cap-label' }, oledParamHint(p))));
    }
  } else if (it.act === OLED_ACT_SUB) {
    const s = h('select', {
      class: 'oled-arg',
      title: '这一块打开哪张子表',
      onpointerdown: (e) => e.stopPropagation(),
      onchange: (e) => { it.sub = e.target.value; it.edited = true; oledMarkDirty(); oledRenderBar(); },
    });
    s.appendChild(h('option', { value: '' }, '（选子表）'));
    for (const nm of OLED.order) {
      if (nm === OLED.sel) continue;
      s.appendChild(h('option', { value: nm, selected: nm === it.sub ? '' : null },
                      oledListTitle(nm) + (oledIsSub(nm) ? '（子菜单）' : '（页面）')));
    }
    if (it.sub) s.value = it.sub;
    blk.appendChild(h('div', { class: 'oled-row2' },
      h('span', { class: 'oled-cap-label' }, '打开'), s));
  }

  /* 拖动 / 选中: ⚠️ 必须在这里 begin —— makeDnd 只负责后续的移动与落座,
   * 候选态得由元素自己的 pointerdown 报上去（漏了就是「拖不起来」）。 */
  blk.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    oledSelectItem(it.id);              /* ⚠️ 只改状态不刷 DOM 的话, 点击时看不到高亮 */
    OLEDDND.begin({ mode: 'move', id: it.id }, e);
  });

  return blk;
}

/* ---------------------------------------------------------------- 「参数」选择弹窗
 * 通用调参行（OLED_ACT_TUNE_PARAM）的参数名有**几百项**（Beam 表 + Pi 参数表, 数量随
 * 源码变, 弹窗里的提示会报实时数字）——
 * 塞进一个下拉框没法用, 所以走这个弹窗: 来源切换（全部 / 本机 / 树莓派）+ 搜索 +
 * **按分组归类**的列表, 点一下即选定。
 *
 * ⚠️ 清单是**服务端现扫**的（oled_menu.py::param_catalog, 每次 GET /api/oled 重算）——
 *    刚在 app_beam.c 里加的参数刷新页面就能选到; 前端这里**不缓存、不写死**任何名字。
 * ⚠️ 选定后顺手把**标签**也填上（只在标签还是默认块名时）: 中文名放得下就用中文名,
 *    放不下退到参数名 —— 绝不裁半截（见 oledParamPickApply 里的取舍）。
 */
const PARAMODAL = { item: null, src: 'all' };
const MAPMODAL = { item: null, rows: [] };   /* rows = [{value, label, short}]（当前顺序） */

/* 调参行参数的**值类型**: bool（选项表恰为 {0,1} 两档 → 屏上 √/×, 翻转）/ 
 * enum（有选项表 → LEFT/RIGHT 在档位间循环）/ num（其余: 数值/str）。
 * 判据与固件 ucOledOptsIsBool 同口径 —— 两边必须一起改。 */
function oledParamValueKind(p) {
  if (!p || !p.options) return 'num';
  const opts = p.options.split('|').map(x => x.trim()).filter(Boolean);
  if (opts.length !== 2) return 'enum';
  const vals = opts.map(x => (x.split('=')[0] || '').trim()).sort().join(',');
  return vals === '0,1' ? 'bool' : 'enum';
}

/* ---- 显示映射弹窗（下拉类调参行, 2026-10-18 加）----
 * rows 顺序 = LEFT/RIGHT 的循环顺序; short = 屏上短标签（空 = 用原标签）。
 * 落盘形状 = "值=短标签|值=短标签"（OLED_ITEM_PM 的 map 段）; 全部用原标签且
 * 原顺序 → 存 null（= 没有映射, 固件用真选项表）。 */
function oledMapParse(str, p) {
  const m = {};
  for (const seg of (str || '').split('|')) {
    if (!seg || !seg.includes('=')) continue;
    const [v, s] = seg.split('=', 2);
    m[v.trim()] = s.trim();
  }
  const opts = (p && p.options ? p.options : '').split('|').map(x => x.trim()).filter(Boolean);
  const rows = [];
  for (const o of opts) {
    const eq = o.indexOf('=');
    const value = (eq >= 0 ? o.slice(0, eq) : o).trim();
    const label = (eq >= 0 ? o.slice(eq + 1) : o).trim() || value;
    rows.push({ value: value, label: label, short: m[value] || '' });
  }
  /* 映射里指向真表之外的值（历史手改）也带出来放最后 —— 不静默丢, 让用户看得见 */
  for (const v of Object.keys(m)) {
    if (!rows.some(r => r.value === v)) {
      rows.push({ value: v, label: v, short: m[v] });
    }
  }
  return rows;
}

function oledMapSerialize(rows) {
  const mapped = rows
    .map(r => ({ value: r.value, short: (r.short || '').trim() }))
    .filter(r => r.short && r.short !== r.label);
  if (!mapped.length) return null;               /* 全是原标签 = 没有映射 */
  return mapped.map(r => r.value + '=' + r.short).join('|');
}

function oledOpenMapModal(it) {
  MAPMODAL.item = it;
  MAPMODAL.rows = oledMapParse(it.map, oledParamOf(it.param));
  document.getElementById('mapmodal-title').textContent =
    '显示映射 — ' + it.label + '（' + (it.param || '?') + '）';
  oledRenderMapModal();
  document.getElementById('mapmodal').classList.remove('hidden');
}

function oledCloseMapModal() {
  MAPMODAL.item = null;
  MAPMODAL.rows = [];
  document.getElementById('mapmodal').classList.add('hidden');
}

function oledRenderMapModal() {
  const box = document.getElementById('mapmodal-list');
  box.textContent = '';
  MAPMODAL.rows.forEach((r, i) => {
    const inp = h('input', {
      class: 'mapmodal-short',
      value: r.short,
      placeholder: r.label,
      maxlength: 12,
      title: 'OLED 上显示的短标签（≤3 个汉字; 留空 = 用原标签「' + r.label + '」）',
    });
    inp.addEventListener('input', () => { r.short = inp.value; });
    const row = h('div', { class: 'mapmodal-row', draggable: 'true' },
      h('span', { class: 'mapmodal-grip', title: '拖动排序' }, '⠿'),
      h('span', { class: 'mapmodal-val' }, r.value),
      h('span', { class: 'mapmodal-label' }, r.label),
      inp);
    /* HTML5 拖拽排序: dragstart 记源行号, drop 落到目标行 → oledMapModalMove 重排
     * rows 并重绘（短标签的输入值保存在 rows 里, 重绘不丢）。 */
    row.addEventListener('dragstart', (e) => {
      e.dataTransfer.setData('text/plain', String(i));
      e.dataTransfer.effectAllowed = 'move';
      row.classList.add('dragging');
    });
    row.addEventListener('dragend', () => row.classList.remove('dragging'));
    row.addEventListener('dragover', (e) => e.preventDefault());
    row.addEventListener('drop', (e) => {
      e.preventDefault();
      e.stopPropagation();
      oledMapModalMove(parseInt(e.dataTransfer.getData('text/plain'), 10), i);
    });
    box.appendChild(row);
  });
}

function oledMapModalMove(fromIdx, toIdx) {
  if (fromIdx === toIdx || fromIdx < 0 || toIdx < 0) return;
  if (fromIdx >= MAPMODAL.rows.length || toIdx >= MAPMODAL.rows.length) return;
  const [r] = MAPMODAL.rows.splice(fromIdx, 1);
  MAPMODAL.rows.splice(toIdx, 0, r);
  oledRenderMapModal();
}

function oledMapModalSave() {
  const it = MAPMODAL.item;
  if (!it) return;
  /* rows 数组就是真值（input 的 input 事件实时回写、拖动重排走 splice）——直接序列化 */
  it.map = oledMapSerialize(MAPMODAL.rows);
  it.edited = true;
  oledMarkDirty();
  oledRenderAll();
  oledCloseMapModal();
}

function oledMapModalClear() {
  MAPMODAL.rows.forEach(r => { r.short = ''; });
  /* 全清 = 恢复原标签/原顺序 → 存 null（固件用真选项表） */
  const it = MAPMODAL.item;
  if (it) { it.map = null; it.edited = true; oledMarkDirty(); }
  oledRenderMapModal();
}


function oledOpenParamPicker(it) {
  PARAMODAL.item = it;
  const s = document.getElementById('paramodal-search');
  s.value = '';
  oledParamPickSrc(PARAMODAL.src || 'all');
  document.getElementById('paramodal').classList.remove('hidden');
  s.focus();
}

function oledCloseParamPicker() {
  PARAMODAL.item = null;
  document.getElementById('paramodal').classList.add('hidden');
}

function oledParamPickSrc(src) {
  PARAMODAL.src = src;
  for (const b of document.querySelectorAll('#paramodal .pm-src-btn')) {
    b.classList.toggle('cur', b.dataset.src === src);
  }
  oledRenderParamPicker();
}

function oledRenderParamPicker() {
  const box = document.getElementById('paramodal-list');
  const hint = document.getElementById('paramodal-hint');
  const cur = PARAMODAL.item ? PARAMODAL.item.param : '';
  box.textContent = '';

  const q = (document.getElementById('paramodal-search').value || '').trim().toLowerCase();
  const cat = OLED.params || {};
  const groups = [];                         /* [{key, items}] —— 保序（源码顺序） */
  let total = 0;
  for (const src of ['beam', 'pi']) {
    if (PARAMODAL.src !== 'all' && PARAMODAL.src !== src) continue;
    for (const p of (cat[src] || [])) {
      if (q && !((p.name + ' ' + p.label + ' ' + (p.group || '')).toLowerCase().includes(q))) continue;
      const key = (src === 'pi' ? '树莓派 · ' : '本机 · ') + (p.group || '其它');
      let g = groups.find(x => x.key === key);
      if (!g) { g = { key: key, items: [] }; groups.push(g); }
      g.items.push(p);
      total++;
    }
  }

  hint.textContent = '本机(Beam) ' + ((cat.beam || []).length) + ' 项 · 树莓派 '
    + ((cat.pi || []).length) + ' 项 —— 共 ' + total + ' 项匹配。'
    + ' 本机参数现取现写; 树莓派参数走后台轮询、改的是**临时值**（不落盘）。';

  if (!total) {
    box.appendChild(h('p', { class: 'hint' }, '没有匹配的参数'));
    return;
  }

  for (const g of groups) {
    box.appendChild(h('div', { class: 'pm-grp' }, g.key + '（' + g.items.length + '）'));
    for (const p of g.items) {
      const badges = [];
      if (p.ro) badges.push('只读');
      if (p.unit) badges.push(p.unit);
      if (p.kind === 'str') badges.push('字符串');
      if (p.options) badges.push(p.options);
      if (p.warn) badges.push('⚠控制律');
      box.appendChild(h('div', {
        class: 'pm-item' + (p.name === cur ? ' cur' : ''),
        title: oledParamHint(p),
        onclick: () => oledParamPickApply(p),
      },
        h('span', { class: 'pm-name' }, p.name),
        h('span', { class: 'pm-label' }, p.label),
        h('span', { class: 'pm-badges' }, badges.join(' · ')),
      ));
    }
  }
}

function oledParamPickApply(p) {
  const it = PARAMODAL.item;
  if (!it) return;
  if (it.param !== p.name) it.map = null;   /* 换了参数, 旧映射的值域不再适用 —— 丢掉重配 */
  it.param = p.name;
  it.edited = true;
  /* 标签还停在**默认块名**（或空）时才跟着填 —— 手改过的名字不要被覆盖。
   * 中文名放得下就用它; 放不下就退到**参数名**（ASCII, 一般 56px 内放得下, 而且
   * 排障时一眼看出调的是哪个量）; 两个都放不下就留着默认块名, 由人自己起。
   * ⚠️ **绝不裁一刀**当标签 —— "巡线PID-Kp" 裁成 "巡线PID-" 这种半截名字比不填还坏
   *    （看着像名字, 其实是被切过的, 车上还会红着宽度告警）。 */
  const defName = (oledPaletteOf(it.act) || {}).name || oledShort(it.act);
  if (!it.label || it.label === defName) {
    if (oledLabelFits(p.label, 64))      it.label = p.label;
    else if (oledLabelFits(p.name, 64))  it.label = p.name;
  }
  oledMarkDirty();
  oledCloseParamPicker();
  oledRenderAll();
}

/* ---------------------------------------------------------------- 预览 */
/* 预览画布的逻辑尺寸（= 一个"模块"）；屏玻璃区嵌在框里。
 * 全部按 1:1 逻辑像素画, 再用 CSS 整数放大 —— 屏上的字才与车上同一套步进。 */
const OLED_MOD = { w: 144, h: 96, gx: 8, gy: 10, gw: 128, gh: 64 };

/* 画"模块"外框: 深色 PCB + 四角安装孔 + 底部金色排针 + 屏玻璃。
 * ⚠️ 玻璃底是**黑**的 —— 真实 OLED 是黑底白字, 只有反色（选中）那行是白底黑字。 */
function oledDrawModule(ctx) {
  const M = OLED_MOD;
  const px = (x, y, w, h, c) => { ctx.fillStyle = c; ctx.fillRect(x, y, w, h); };

  px(0, 0, M.w, M.h, '#0b2019');                       /* 板底 */
  ctx.strokeStyle = '#1d5544';                          /* 板边 */
  ctx.lineWidth = 1;
  ctx.strokeRect(0.5, 0.5, M.w - 1, M.h - 1);
  for (const c of [[4, 4], [M.w - 5, 4], [4, M.h - 5], [M.w - 5, M.h - 5]]) {
    px(c[0], c[1], 3, 3, '#061410');                    /* 四角安装孔 */
  }
  for (let x = 16; x < M.w - 16; x += 4) px(x, 84, 2, 5, '#c9a227');   /* 底部排针 */

  px(M.gx - 1, M.gy - 1, M.gw + 2, M.gh + 2, '#0a0a0a');   /* 屏沿 */
  px(M.gx, M.gy, M.gw, M.gh, '#000');                      /* 玻璃底: 黑 */
}

/* 画一行。
 * ⚠️⚠️ **反色 = 白底黑字**（选中行）, 普通行是**黑底白字** —— 与真实 OLED 一致。
 *     （第一版画反了: 整屏白底黑字、选中行深底白字。） */
function oledDrawRow(ctx, y, text, invert) {
  ctx.fillStyle = invert ? '#fff' : '#000';
  ctx.fillRect(0, y, OLED_MOD.gw, OLED_ROW_H);
  const fg = invert ? '#000' : '#fff';
  ctx.fillStyle = fg;
  ctx.strokeStyle = fg;
  ctx.lineWidth = 1;

  let x = 0;
  for (const ch of String(text || '')) {
    if (x >= OLED_MOD.gw) break;                   /* 固件也是静默截断 */
    const han = oledIsHan(ch);
    if (han && OLED.missing.indexOf(ch) >= 0) {
      /* 字表里没有 → 与固件同款「方框问号」兜底图形 */
      ctx.strokeRect(x + 2.5, y + 2.5, 11, 11);
      ctx.fillRect(x + 5, y + 5, 6, 2);
      ctx.fillRect(x + 7, y + 8, 2, 3);
    } else if (han) {
      ctx.font = '16px "SimSun","宋体",serif';
      ctx.fillText(ch, x, y + 13);
    } else {
      /* ASCII: 16px 等宽字形横向压一半 → 8px 步进, 与固件 OLED_F8x16 的宽度对上 */
      ctx.save();
      ctx.translate(x, y);
      ctx.scale(0.5, 1);
      ctx.font = '16px monospace';
      ctx.fillText(ch, 0, 13);
      ctx.restore();
    }
    x += han ? 16 : 8;
  }
}

/* 右侧竖直滚动条 —— 几何与固件 vOledScrollbarDraw 逐条对齐:
 * 预留带 x=120..127（8px）, 轨道 x=122..125（4px 宽、空心、高 64）, 滑块实心,
 * 高 = 64*4/N（下限 5）, y = (64-h)*top/(N-4)。
 * ⚠️ 必须画在**所有行之后**（同固件）—— 它会盖掉自己那条带子上的行内容。 */
function oledDrawScrollbar(ctx, top, total) {
  const H = OLED_WIN * OLED_ROW_H;               /* 64 */
  const BX = OLED_W - OLED_BAR_W + 2, BW = 4;    /* 122..125 */
  ctx.fillStyle = '#000';                        /* 先清整条预留带（帧缓冲持久, 同固件） */
  ctx.fillRect(OLED_LABEL_W, 0, OLED_BAR_W, H);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1;
  ctx.strokeRect(BX + 0.5, 0.5, BW - 1, H - 1);  /* 轨道（空心） */
  const h = Math.max(5, Math.floor(H * OLED_WIN / total));
  const m = total - OLED_WIN;
  const y = m === 0 ? 0 : Math.floor((H - h) * top / m);
  ctx.fillStyle = '#fff';
  ctx.fillRect(BX, y, BW, h);                    /* 滑块（实心） */
}

function oledRenderPreview() {
  const cv = document.getElementById('oled-screen');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  oledDrawModule(ctx);

  const info = document.getElementById('oled-preview-info');
  info.textContent = '';
  const l = OLED.lists[OLED.sel];
  if (!l) return;

  const M = OLED_MOD;
  const sub = oledIsSub(OLED.sel);
  const enabled = l.items.filter(i => i.enabled);
  /* 条目 = 启用项 (+ 子菜单那条固件补的「返回」)。一屏只显示 4 行, 放不下的滚动 ——
   * 窗口顶行按"选中块"定位（与固件 vOledWindowSync 同一条**最小幅度跟随**规则 ⇒ 预览即验收）。 */
  const entries = enabled.map(i => i.label);
  if (sub) entries.push('返回');
  const total = entries.length;
  const selIdx = enabled.indexOf(oledFindItem(OLED.selItem));

  let top = 0;
  if (total > OLED_WIN && selIdx >= 0) {
    top = Math.min(Math.max(selIdx - OLED_WIN + 1, 0), total - OLED_WIN);
  }
  /* 反色那行 = 光标停的位置（选中的那块被藏起来/不在窗口里时退回第一行 —— "刚进这页"的样子）。 */
  let hl = entries.length ? 0 : -1;
  if (selIdx >= 0 && selIdx - top >= 0 && selIdx - top < OLED_WIN) hl = selIdx - top;

  ctx.save();
  ctx.beginPath();
  ctx.rect(M.gx, M.gy, M.gw, M.gh);              /* 内容只画在玻璃区里 */
  ctx.clip();
  ctx.translate(M.gx, M.gy);
  entries.slice(top, top + OLED_WIN).forEach((t, i) => oledDrawRow(ctx, i * OLED_ROW_H, t, i === hl));
  if (total > OLED_WIN) oledDrawScrollbar(ctx, top, total);
  ctx.restore();

  /* ⚠️ 判据用**条目总数**（子菜单含固件那条「返回」）, 不能在截断之后再判 —— 那样恒不成立。 */
  if (total > OLED_MAX_ITEMS) {
    info.appendChild(h('p', { class: 'oled-warn bad' },
      '⚠ 超过 ' + OLED_MAX_ITEMS + ' 项 —— 第 ' + (OLED_MAX_ITEMS + 1) + ' 项起在车上既看不见也选不到。'));
  } else if (total > OLED_WIN) {
    info.appendChild(h('p', { class: 'oled-warn' },
      '共 ' + total + ' 项, 超出一屏 —— 预览画的是第 ' + (top + 1) + '..' + (top + OLED_WIN)
      + ' 项（车上按 UP/DOWN 滚动, 右侧滚动条指示位置）。'));
  }
  if (OLED.missing.length) {
    info.appendChild(h('p', { class: 'oled-warn' },
      '字表里还没有: ' + OLED.missing.join(' ') + '（预览画成方框；保存时会自动补字并重生成字模）'));
  }
}


const OLEDDND = makeDnd({
  innerEl: () => document.getElementById('oled-canvas-inner'),
  canvasEl: () => document.getElementById('oled-canvas'),
  trashEl: () => document.getElementById('oled-trash'),
  blockId: (id) => 'blk-oled-' + id,
  items: () => (OLED.lists[OLED.sel] ? OLED.lists[OLED.sel].items : []),
  zonesMeta: () => {
    const l = OLED.lists[OLED.sel];
    const z = document.querySelector('#oled-canvas-inner .zone');
    return (z && l) ? [{ el: z, ids: l.items.map(i => i.id), nextFirstId: null }] : [];
  },
  catalogOf: (type) => oledPaletteOf(type),
  createItem: (cat) => oledNewItemFromAct(cat.type),
  newId: () => 'oi' + (++OLED.seq),
  newGhost: (type, cat) => h('div', { class: 'block oled-item drag-ghost drag-new' },
    h('div', { class: 'oled-row1' },
      h('span', { class: 'oled-tag' }, (cat && cat.name) || oledShort(type)))),
  render: oledRenderAll,
  select: (id) => oledSelectItem(id),
  deleteItem: (id) => {
    const l = oledItemList(id);
    if (!l) return;
    l.items = l.items.filter(i => i.id !== id);
    l.edited = true;
    if (OLED.selItem === id) OLED.selItem = null;
    oledMarkDirty();
    oledRenderAll();
  },
  beforeChange: oledMarkDirty,
});

/* ---------------------------------------------------------------- 初始化 */

function oledInit() {
  document.getElementById('oled-palette-search').addEventListener('input', oledRenderPalette);
  document.getElementById('oled-list-select').addEventListener('change', (e) => {
    OLED.sel = e.target.value;
    OLED.selItem = null;
    oledRenderAll();
  });
  document.getElementById('oled-page-add').addEventListener('click', oledAddPage);
  document.getElementById('oled-sub-add').addEventListener('click', () => {
    OLED.sel = oledAddSub();
    oledRenderAll();
  });
  document.getElementById('btn-oled-reload').addEventListener('click', oledReload);
  document.getElementById('btn-oled-save').addEventListener('click', oledSave);
}
/* ==================== 启动 ==================== */

function init() {
  renderTabs();
  document.getElementById('btn-save').addEventListener('click', save);
  document.getElementById('btn-reload').addEventListener('click', reloadFromDisk);
  document.getElementById('palette-search').addEventListener('input', renderPalette);

  /* 预设栏 */
  document.getElementById('preset-select').addEventListener('change', (e) => {
    switchPreset(parseInt(e.target.value, 10));
  });
  document.getElementById('preset-label').addEventListener('change', (e) => {
    const p = curPreset();
    if (!p) return;
    p.label = e.target.value.trim() || p.ident;
    markDirty();
    renderPresetBar();
  });
  document.getElementById('preset-add').addEventListener('click', addPreset);
  document.getElementById('preset-dup').addEventListener('click', dupPreset);
  document.getElementById('preset-del').addEventListener('click', delPreset);
  document.getElementById('preset-reentrant').addEventListener('change', (e) => {
    /* 「可重入的任务」（2026-10-18 加）: 只改 S.reentrantIds 并标脏 —— 随下一次
     * 保存写进 map.h 枚举注释（可重入）, 「跳到别的预设」的目标下拉就会撤掉
     * 不可跳入 标记（服务端在 write_map 里用 reentrant_ids 压过 unsafe 启发式）。 */
    const p = curPreset();
    const pid = presetIdOf(p);          /* 新建/刚保存的预设也有（见 presetIdOf） */
    if (pid === null) { e.target.checked = false; return; }
    const ids = new Set(S.reentrantIds || []);
    if (e.target.checked) {
      ids.add(pid);
      toast('「' + p.label + '」已标记为可重入（保存后，「跳到别的预设」即可调入；'
          + '若表内有握手/重置步骤，调入时按固件语义执行）');
    } else {
      ids.delete(pid);
      toast('「' + p.label + '」已取消可重入标记');
    }
    S.reentrantIds = [...ids].sort((a, b) => a - b);
    markDirty();
  });
  document.getElementById('preset-active').addEventListener('change', (e) => {
    const p = curPreset();
    if (!p) return;
    if (e.target.checked) {
      S.runIdent = p.ident;
      markDirty();
      renderPresetBar();
      toast('设备将运行「' + p.label + '」（保存后生效）');
    } else {
      e.target.checked = p.ident === S.runIdent;   // 至少要有一个运行预设, 不允许取消
    }
  });

  /* 元数据编辑器 */
  document.getElementById('modal-save').addEventListener('click', saveMetaEditor);
  document.getElementById('modal-cancel').addEventListener('click', closeMetaEditor);
  document.getElementById('modal-close').addEventListener('click', closeMetaEditor);
  document.getElementById('modal').addEventListener('click', (e) => {
    if (e.target.id === 'modal') closeMetaEditor();       // 点遮罩关闭
  });

  /* 调参 Tab */
  document.getElementById('btn-tune-save').addEventListener('click', saveTuning);
  document.getElementById('btn-tune-reload').addEventListener('click', async () => {
    if (Object.keys(TUNE.changed).length && !confirm('有未保存的改动，重载将丢弃，继续？')) return;
    await loadTuning();
    toast('已从磁盘重载');
  });
  document.getElementById('tune-picker').addEventListener('click', openPicker);
  document.getElementById('tune-new').addEventListener('click', openNewParam);
  document.getElementById('tune-sortmode').addEventListener('change', (e) => {
    TUNE.sortMode = e.target.checked;
    document.getElementById('tuning-canvas').classList.toggle('sortmode', TUNE.sortMode);
    toast(TUNE.sortMode ? '排序模式：拖动参数换顺序（只影响本页）' : '已退出排序模式');
  });
  document.getElementById('tuning-canvas').addEventListener('contextmenu', (e) => {
    const row = e.target.closest('.tune-item');
    if (!row) return;
    e.preventDefault();
    const it = TUNE.byName[row.dataset.name];
    if (it) openMeta(it);
  });
  const tcanvas = document.getElementById('tuning-canvas');
  tcanvas.addEventListener('pointerdown', onTuneDragStart);
  document.addEventListener('pointermove', onTuneDragMove);
  document.addEventListener('pointerup', onTuneDragEnd);
  /* 拖拽被打断（pointercancel / 窗口失焦时在窗外松手）: 只复位样式, 不当成交换落座 */
  document.addEventListener('pointercancel', () => {
    if (!dragEl) return;
    dragEl.classList.remove('dragging');
    dragEl = null;
  });
  window.addEventListener('blur', () => {
    if (!dragEl) return;
    dragEl.classList.remove('dragging');
    dragEl = null;
  });
  document.getElementById('tmodal-save').addEventListener('click', runTuningModalSave);
  document.getElementById('tmodal-cancel').addEventListener('click', closeTuningModal);
  document.getElementById('tmodal-close').addEventListener('click', closeTuningModal);
  document.getElementById('tmodal').addEventListener('click', (e) => {
    if (e.target.id === 'tmodal') closeTuningModal();
  });

  /* 连接面板: 两个 Tab 各挂一份, 共用上面的 BT 状态（同一时刻只可能有一条链路） */
  makeConnPanel({
    id: 'bt', mount: document.getElementById('bt-conn-mount'),
    logEl: document.getElementById('bt-log'),
    onConnected: async () => { await btReadParams(); renderJoints(); },
  });
  makeConnPanel({
    id: 'arm', mount: document.getElementById('arm-conn-mount'),
    logEl: document.getElementById('arm-log'),
    /* 「进入编辑模式 / 退出」搬进工厂搭的那行按钮, 紧跟 连接/断开。
     * 传元素本身（不是 id）—— 工厂开工第一步会清空 mount, 得先把引用交出去。 */
    extra: document.getElementById('arm-edit-row'),
    /* 连上先把关节表拿来（与能不能进编辑模式无关）；若 **Pi 在线**（rpi_link==1）
     * 则先投一次「握手让出」（$B,HS, 2026-10 加）再自动进编辑模式 —— Pi 在线时
     * 脚本多半在跑, 不让位就 NOHANDSHAKE。让位生效要等路线当前这一步跑完, 所以
     * btEnterEdit 的 auto 路径带静默重试。台架上连蓝牙的目的基本就是调臂,
     * 少点一次是一次（成败都 toast, 见 btEnterEdit）。 */
    onConnected: async () => {
      await btReadParams(true); renderJoints();
      const link = (BT.params || []).find(p => p.name === 'rpi_link');
      if (link && Number(link.value) === 1) {
        await btRun('hs', {}, { quiet: true });   /* 失败（固件旧/掉线）不拦下面的直接进 */
      }
      await btEnterEdit(true);
    },
  });

  /* 拖关节滑杆期间挂起轮询, 抬起/取消就恢复（挂 document 级, 免得每根滑块挂一遍） */
  document.addEventListener('pointerup', () => { BT.dragging = false; });
  document.addEventListener('pointercancel', () => { BT.dragging = false; });
  /* 连接状态低频自检: 空闲时拔线/掉线界面能自己变（原来只在手动操作后才同步）。
   * GET /api/bt 不会打开串口; btPollConn 也刻意不重建串口下拉 —— 那会把用户刚选好
   * 还没点的口冲掉。 */
  setInterval(btPollConn, 3000);

  /* 机械臂 Tab */
  document.getElementById('arm-preset-select').addEventListener('change', (e) => {
    armSwitchPreset(parseInt(e.target.value, 10));
  });
  document.getElementById('arm-title').addEventListener('input', (e) => {
    const p = armCurPreset();
    if (!p) return;
    p.title = e.target.value;
    armMarkDirty();
  });
  document.getElementById('arm-note').addEventListener('input', (e) => {
    const p = armCurPreset();
    if (!p) return;
    p.note = e.target.value;
    armMarkDirty();
  });
  document.getElementById('arm-mode').addEventListener('change', (e) => {
    const p = armCurPreset();
    if (!p) return;
    armPushUndo();
    p.nowait = (e.target.value === 'nowait');
    armMarkDirty();
  });
  document.getElementById('arm-undo').addEventListener('click', armUndo);
  document.getElementById('arm-redo').addEventListener('click', armRedo);
  document.getElementById('arm-preset-add').addEventListener('click', armAddPreset);
  document.getElementById('arm-preset-dup').addEventListener('click', armDupPreset);
  document.getElementById('arm-preset-del').addEventListener('click', armDelPreset);
  document.getElementById('arm-trash-open').addEventListener('click', openArmTrash);
  document.getElementById('arm-save').addEventListener('click', armSave);
  /* ⚠️ 包一层箭头: 直接挂 btEnterEdit 会把 click 的 Event 当成 auto 实参,
   *    手动进入失败就不报错了（auto 那条路是刻意静默的）。 */
  document.getElementById('arm-edit-on').addEventListener('click', () => btEnterEdit());
  document.getElementById('arm-edit-off').addEventListener('click', btExitEdit);
  /* ⚠️ 包一层箭头: 直接挂 armGetPose 会把 click 的 Event 当成 opts 实参,
   *    于是 quiet=!!Event=... 恒为 true, 手动点就再也不弹「已读取当前位姿」了。 */
  document.getElementById('arm-get-pose').addEventListener('click', () => armGetPose());
  /* 点画布空白处取消选中（点子积木由积木自己处理, 但事件会冒泡上来, 所以判一下） */
  document.getElementById('arm-canvas').addEventListener('click', (e) => {
    if (e.target.closest('.block')) return;
    armDeselect();
  });
  /* 右键菜单: 点菜单外 / 按 Esc / 页面滚动 都关掉（右键菜单开着却挡在那是很烦的）。
   * 用**捕获**阶段: 积木自己的 pointerdown 会启动拖拽候选, 菜单得先让路。 */
  document.addEventListener('pointerdown', (e) => {
    if (armCtxEl && !armCtxEl.contains(e.target)) closeArmCtx();
  }, true);
  /* 页面滚动关菜单（菜单自己滚不算, 理由见 armCtxOnScroll 的注释） */
  window.addEventListener('scroll', armCtxOnScroll, true);
  document.getElementById('arm-record').addEventListener('click', armRecord);
  document.getElementById('arm-add-delay').addEventListener('click', armAddDelay);
  document.getElementById('arm-play-all').addEventListener('click', armPlayAll);
  document.getElementById('arm-play-device').addEventListener('click', armPlayDevice);

  /* 调试助手 Tab */
  document.getElementById('btn-tools-refresh').addEventListener('click', loadTools);
  for (const id of ['net-local', 'net-console', 'net-pi', 'net-ssh', 'net-share']) {
    document.getElementById(id).addEventListener('click', () => copyCard(id));
  }
  document.getElementById('tool-update').addEventListener('click',
    () => runTool('update', document.getElementById('tools-dryrun').checked));
  document.getElementById('tool-rdp').addEventListener('click', () => runTool('rdp'));
  document.getElementById('tool-folder').addEventListener('click', () => runTool('folder'));
  document.getElementById('tool-trae').addEventListener('click', () => runTool('trae'));
  document.getElementById('tool-vscode').addEventListener('click', () => runTool('vscode'));
  document.getElementById('tool-console').addEventListener('click', () => runTool('console'));
  document.getElementById('tools-log-clear').addEventListener('click', () => {
    document.getElementById('tools-log').textContent = '';
  });
  document.getElementById('phrase-add').addEventListener('click', () => openPhraseEditor(-1));
  document.getElementById('phrase-edit').addEventListener('click', () => {
    if (TOOLS.sel === null) { toast('先选中一条常用语', 'err'); return; }
    openPhraseEditor(TOOLS.sel);
  });
  document.getElementById('phrase-del').addEventListener('click', delPhrase);
  document.getElementById('pmodal-save').addEventListener('click', savePhraseEditor);
  document.getElementById('pmodal-cancel').addEventListener('click', closePhraseEditor);
  document.getElementById('pmodal-close').addEventListener('click', closePhraseEditor);
  document.getElementById('pmodal').addEventListener('click', (e) => {
    if (e.target.id === 'pmodal') closePhraseEditor();        // 点遮罩关闭
  });

  /* OLED「参数」弹窗（通用调参行选参数名）—— 搜索框输入即过滤, 来源按钮切换两个表。 */
  document.getElementById('paramodal-close').addEventListener('click', oledCloseParamPicker);
  document.getElementById('paramodal').addEventListener('click', (e) => {
    if (e.target.id === 'paramodal') oledCloseParamPicker();  // 点遮罩关闭
  });
  document.getElementById('paramodal-search').addEventListener('input', oledRenderParamPicker);
  for (const b of document.querySelectorAll('#paramodal .pm-src-btn')) {
    b.addEventListener('click', () => oledParamPickSrc(b.dataset.src));
  }

  /* OLED「显示映射」弹窗（下拉类调参行, 2026-10-18 加）—— 拖动排序 + 短标签编辑。 */
  document.getElementById('mapmodal-close').addEventListener('click', oledCloseMapModal);
  document.getElementById('mapmodal-cancel').addEventListener('click', oledCloseMapModal);
  document.getElementById('mapmodal-save').addEventListener('click', oledMapModalSave);
  document.getElementById('mapmodal-clear').addEventListener('click', oledMapModalClear);
  document.getElementById('mapmodal').addEventListener('click', (e) => {
    if (e.target.id === 'mapmodal') oledCloseMapModal();      // 点遮罩关闭
  });

  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
      if (armCtxEl) { closeArmCtx(); return; }
      if (metaCtx) { closeMetaEditor(); return; }
      if (!document.getElementById('mapmodal').classList.contains('hidden')) {
        oledCloseMapModal(); return;
      }
      if (!document.getElementById('paramodal').classList.contains('hidden')) {
        oledCloseParamPicker(); return;
      }
      if (!document.getElementById('tmodal').classList.contains('hidden')) { closeTuningModal(); return; }
      if (!document.getElementById('pmodal').classList.contains('hidden')) closePhraseEditor();
      return;
    }
    if ((e.ctrlKey || e.metaKey) && e.key === 's') {
      e.preventDefault();
      /* 跟顶栏保存按钮一样按 Tab 分发（顶栏按钮在 switchTab 里切, 键盘原来漏了:
       * 在调参/OLED/底盘动作组按 Ctrl+S 会把 map.c 的**内存态**存出去,
       * 本 Tab 的未保存改动原封不动 —— 用户以为存了配置） */
      if (metaCtx) saveMetaEditor();
      else if (curTab === 'map') save();
      else if (curTab === 'arm') armSave();
      else if (curTab === 'tuning') saveTuning();
      else if (curTab === 'chact') chactSave();
      else if (curTab === 'calib') calibSave();
      else if (curTab === 'oled') oledSave();
    }
    /* 机械臂 Tab 的撤回/重做。在输入框里打字时不接管 —— 那时 Ctrl+Z 该撤文本。 */
    if (curTab === 'arm' && (e.ctrlKey || e.metaKey)) {
      const tag = (e.target.tagName || '').toUpperCase();
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      const k = (e.key || '').toLowerCase();
      if (k === 'z' && !e.shiftKey) { e.preventDefault(); armUndo(); return; }
      if (k === 'y' || (k === 'z' && e.shiftKey)) { e.preventDefault(); armRedo(); return; }
    }
    /* 采集 Tab 的快捷键: 空格采集 / 1-4 切类别 / A 启停自动。
     * 在输入框里打字时一律忽略（否则调间隔时按空格会连拍）。 */
    if (curTab === 'capture' && !e.ctrlKey && !e.metaKey) {
      const tag = (e.target.tagName || '').toUpperCase();
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (e.key === ' ') { e.preventDefault(); capShot(); return; }
      if (e.key >= '1' && e.key <= '9') { capSelectByIndex(parseInt(e.key, 10) - 1); return; }
      if (e.key === 'a' || e.key === 'A') { capToggleAuto(); return; }
    }
  });

  /* 蓝牙 Tab */
  document.getElementById('bt-param-read').addEventListener('click', () => btReadParams());
  document.getElementById('bt-log-clear').addEventListener('click', btLogClear);
  document.getElementById('arm-log-clear').addEventListener('click', btLogClear);
  document.getElementById('bt-param-auto').addEventListener('change', (e) => {
    if (BT.autoTimer) { clearInterval(BT.autoTimer); BT.autoTimer = null; }
    if (e.target.checked) {
      /* 3s 一次：每次是一条完整的 $B,LIST（约 2.7KB），蓝牙链路上别太密。
       * ⚠️ 拖动滑杆期间必须让路: 一次 LIST 是逐行阻塞发（行间 vTaskDelay(1)）,
       * 会把正在拖的那根滑块顿住小半秒, 手感直接塌掉。 */
      BT.autoTimer = setInterval(() => {
        if (!linkTabVisible()) { btStopTimers(); return; }
        if (BT.dragging) return;
        btReadParams(true);
      }, 3000);
    }
  });

  /* 采集 Tab */
  document.getElementById('cap-enable').addEventListener('change', (e) => {
    capSetEnabled(e.target.checked);
  });
  document.getElementById('cap-rescan').addEventListener('click', () => capScan(false));
  document.getElementById('cap-shot').addEventListener('click', capShot);
  document.getElementById('cap-auto').addEventListener('click', capToggleAuto);
  document.getElementById('cap-interval').addEventListener('change', (e) => {
    const sec = parseFloat(e.target.value);
    if (!(sec > 0)) { toast('自动采集间隔要大于 0 秒', 'err'); e.target.value = CAP.interval; return; }
    CAP.interval = sec;
    capRun('prefs', { interval: sec }, { quiet: true, silent: true });
  });
  document.getElementById('cap-log-clear').addEventListener('click', () => {
    document.getElementById('cap-log').textContent = '';
  });
  document.getElementById('cap-reveal').addEventListener('click', async () => {
    const data = await capRun('reveal');
    if (data) toast('已打开 ' + data.raw_dir);
  });
  /* 断开/掉线时按钮状态要跟上 */
  /* 连接按钮初始态由 renderConn 按 BT.connected 决定（面板已挂上） */

  oledInit();       /* OLED Tab: 顶栏按钮 + 表选择器 + 两个「＋」 */

  revInit();        /* 多客户端同步: 起轮询 + 挂提示条的两个按钮 */

  loadAll().catch(err => toast('加载失败: ' + err.message, 'err'));
  loadMoves();      /* 位移变量清单是「地图」Tab 徽章的数据源 —— 启动就拉, 到位后重画一次 */
}

document.addEventListener('DOMContentLoaded', init);

/* ==================== Tab: 位移 ====================
 * 路线位移变量（app_config.h「路线位移变量」段）的可视化管理。
 * · 变量的增删改查: 改值/改译名走 /api/tuning（与「调参」Tab 同一个端点 —— 同一个文件
 *   同一把锁, 不另造第二套）; 新增/删除走 /api/moves 的 add_var / delete_var
 *   （删除前服务端查 map.c 引用, 启用行引用直接拒 —— 删了就是 undefined identifier）。
 * · 右侧示意图 = moves.json（图片路径 + 起点标记 + 线段绑定）, **纯可视化**:
 *   线段只是「这个变量对应图上这一段路」的标注, 不做任何比例尺/投影换算;
 *   改完自动保存（小文件, 没有保存按钮）。坐标是**图像像素**坐标, 换图重画即可。
 * · MOVES.varMap 同时服务「地图」Tab 的参数徽章（paramInput 里查它）—— 所以本模块
 *   的数据在启动时就预载（init 里那条 loadMoves()）, 不是进 Tab 才拉。 */
const MOVES = {
  loaded: false,
  data: null,          // GET /api/moves 的响应体 {version, tuning_version, config, image, variables}
  varMap: {},          // 宏名 → {display, num, unit}（地图 Tab 徽章用）
  changed: {},         // 左栏还没保存的数值改动 {宏名: 显示值字符串}
  img: null,           // Image 对象（加载成功才有; 失败/没配 → null, 画布出占位底）
  selSeg: -1,          // 选中的线段下标; -1 = 无
  drag: null,          // {kind:'start'|'seg1'|'seg2'|'new', seg?, x1?, y1?, x2?, y2?}
  bound: false,
};

const SEG_COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12',
                    '#9b59b6', '#1abc9c', '#e67e22', '#f1c40f'];

function movesEnter() {
  movesBindOnce();
  if (!MOVES.loaded) loadMoves();
  else { movesRenderVars(); movesRenderSel(); movesDrawMap(); }
}

function movesBindOnce() {
  if (MOVES.bound) return;
  MOVES.bound = true;
  document.getElementById('moves-add').addEventListener('click', movesAddDialog);
  document.getElementById('moves-save').addEventListener('click', saveMovesValues);
  document.getElementById('moves-del-seg').addEventListener('click', movesDeleteSelSeg);
  document.getElementById('moves-sel-var').addEventListener('change', async (e) => {
    const seg = MOVES.data && MOVES.data.config.segments[MOVES.selSeg];
    if (!seg) return;
    seg.name = e.target.value;
    await movesSaveConfig();
    movesDrawMap();
  });
  const imgPath = document.getElementById('moves-img-path');
  imgPath.addEventListener('change', async () => {
    if (!MOVES.data) return;
    MOVES.data.config.image = imgPath.value.trim() || 'docs/DefaultMap.JPG';
    await movesSaveConfig();
    await movesLoadImage();
    movesDrawMap();
  });
  movesBindCanvas();
}

async function loadMoves() {
  try {
    const st = await (await fetch('/api/moves')).json();
    MOVES.loaded = true;
    MOVES.data = st;
    MOVES.varMap = {};
    for (const v of st.variables) MOVES.varMap[v.name] = v;
    MOVES.changed = {};                     // 清单换了, 未保存的值编辑不跨清单保留
    if (MOVES.selSeg >= st.config.segments.length) MOVES.selSeg = -1;
    const inp = document.getElementById('moves-img-path');
    if (document.activeElement !== inp) inp.value = st.config.image || '';
    await movesLoadImage();
    if (curTab === 'moves') { movesRenderVars(); movesRenderSel(); }
    movesDrawMap();
    /* 清单是「地图」Tab 徽章的数据源 —— 启动时它先于本请求渲染完, 宏名还躺在裸输入框里;
     * 到位后重画一次。护栏同 /api/rev 的自动重载: 拖拽/输入中/有未保存改动时不动
     * （渲染是从 S.items 的模型重建, 不会丢已落进模型的东西, 但会丢没失焦的输入框）。 */
    if (curTab === 'map' && !revBlocked('map') && !S.dirty) { renderCanvas(); renderInspector(); }
  } catch (e) {
    toast('读取位移变量失败: ' + e.message, 'err');
  }
}

/* ---- 左栏: 变量列表 ---- */

function movesRenderVars() {
  const root = document.getElementById('moves-vars');
  root.textContent = '';
  const st = MOVES.data;
  if (!st || !st.variables.length) {
    root.appendChild(h('p', { class: 'hint' }, '还没有变量 —— 点「＋ 新增」建一个。'));
    return;
  }
  for (const v of st.variables) root.appendChild(movesVarRow(v));
}

function movesVarRow(v) {
  const changed = Object.prototype.hasOwnProperty.call(MOVES.changed, v.name);
  const row = h('div', { class: 'moves-var' + (changed ? ' changed' : ''), 'data-name': v.name });
  const nameEl = h('div', {
    class: 'moves-var-name',
    title: '双击改译名 · 宏名 ' + v.name + '（app_config.h）',
    ondblclick: (e) => { e.stopPropagation(); movesRename(v); },
  },
    h('span', { class: 'tn-disp' }, v.display),
    h('code', { class: 'tn-macro' }, v.name));
  const inp = h('input', {
    class: 'tune-val', value: changed ? MOVES.changed[v.name] : String(v.num ?? ''),
    title: v.name + ' 的距离 [mm]（负值 = 沿世界系负轴走）',
    oninput: (e) => { MOVES.changed[v.name] = e.target.value; row.classList.add('changed'); },
  });
  row.appendChild(nameEl);
  row.appendChild(h('div', { class: 'moves-var-val' },
    inp,
    h('span', { class: 'p-unit' }, v.unit || 'mm'),
    h('button', {
      class: 'btn tiny danger', title: '删除这个变量（服务端先查 map.c 引用, 启用行引用会拒）',
      onclick: () => movesDeleteVar(v),
    }, '🗑')));
  return row;
}

async function saveMovesValues() {
  const names = Object.keys(MOVES.changed);
  if (!names.length) { toast('没有改过的数值'); return; }
  const typed = Object.assign({}, MOVES.changed);      // 被拒的参数要把值留在框里
  try {
    const r = await fetch('/api/tuning', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ base_version: MOVES.data.tuning_version, values: MOVES.changed }),
    });
    if (r.status === 409) { toast('app_config.h 已在外部被修改，请先重载', 'err'); return; }
    const j = await r.json();
    if (!r.ok) { toast('保存失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return; }
    const warns = j.warnings || [];
    const rejected = warns.filter(w => !w.saved).map(w => w.name);
    await loadMoves();
    for (const n of rejected) {
      if (typed[n] !== undefined) MOVES.changed[n] = typed[n];
    }
    movesRenderVars();
    if (warns.length) toast('⚠ ' + warns.map(w => w.name + ' ' + w.msg).join('；'), 'err');
    else toast('已写回 app_config.h（备份 .bak）—— Keil 重编译烧录后才上车');
  } catch (e) {
    toast('保存失败: ' + e.message, 'err');
  }
}

function movesRename(v) {
  const el = document.querySelector('.moves-var[data-name="' + v.name + '"] .moves-var-name');
  if (!el) return;
  editInPlace(el, v.display, async (nv) => {
    if (!nv || nv === v.display) { movesRenderVars(); return; }
    const r = await fetch('/api/tuning/meta', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      /* ⚠️ write_meta 是整条尾注释重写, 必须**全字段**透传（缺了 conv/range/note 就被无声抹掉,
       *    下次保存按空回吐 —— 「调参」Tab 的 saveMetaEditor 同一条纪律）。 */
      body: JSON.stringify({
        name: v.name, base_version: MOVES.data.tuning_version,
        meta: { display: nv, unit: v.unit || 'mm', conv: v.conv || '',
                range: v.range || { lo: '', hi: '' }, note: v.note || '' },
      }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { toast('改译名失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return; }
    toast('已写回 app_config.h（备份 .bak）');
    await loadMoves();
  });
}

function movesAddDialog() {
  const name = h('input', { placeholder: 'MOVE_XXX_2', title: '宏名: 大写字母/数字/下划线。定了就别再改 —— map.c 里的引用要跟着手改' });
  const disp = h('input', { placeholder: '译名（界面上显示的中文短名）' });
  const val = h('input', { value: '0', placeholder: '0' });
  const body = h('div', {},
    h('p', { class: 'modal-hint' },
      '新增一个位移变量并写回 app_config.h 的「路线位移变量」段（[mm], 范围 ±2000）。'
      + '要让它上路线, 还得把 map.c 里某步「世界系平移」的位移参数填成这个宏名（在「地图」Tab 编辑）。'),
    h('label', { class: 'fld' }, '宏名', name),
    h('label', { class: 'fld' }, '译名', disp),
    h('label', { class: 'fld' }, '初始值 [mm]', val));
  openTuningModal('新增位移变量', body, async () => {
    const r = await fetch('/api/moves', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'add_var', name: name.value.trim(),
                             display: disp.value.trim(), value: val.value.trim() || '0',
                             base_version: MOVES.data.tuning_version }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { toast('新增失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return false; }
    toast('已新增 → app_config.h（备份 .bak）');
    await loadMoves();
    return true;
  }, '新增');
}

async function movesDeleteVar(v) {
  if (!MOVES.data) return;
  const bound = MOVES.data.config.segments.filter(s => s.name === v.name).length;
  let msg = '删除变量「' + v.display + '」（' + v.name + '）？';
  if (bound) msg += '\n图上有 ' + bound + ' 条线段绑着它, 会一并解绑。';
  msg += '\n服务端会先查 map.c 引用 —— 还有启用的步骤在用就删不掉。';
  if (!confirm(msg)) return;
  const r = await fetch('/api/moves', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'delete_var', name: v.name,
                           base_version: MOVES.data.tuning_version }),
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { toast('删除失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return; }
  toast(j.message || ('已删除 ' + v.name));      /* 删除成功是好消息, 别用 err 红样式 */
  await loadMoves();
}

/* 「地图」Tab 徽章/▾ 的点击落点: 跳到位移 Tab 并高亮那一行（name 为空 = 只切过去）。 */
function movesRevealVar(name) {
  switchTab('moves');
  if (!name) return;
  const go = () => {
    const row = document.querySelector('.moves-var[data-name="' + name + '"]');
    if (!row) { toast('变量 ' + name + ' 不存在了'); return; }
    row.scrollIntoView({ block: 'center' });
    row.classList.add('flash');
    setTimeout(() => row.classList.remove('flash'), 1700);
  };
  if (MOVES.loaded) go();
  else loadMoves().then(go);
}

/* 位移变量选择弹窗（paramInput 的 move_var 控件）: 换绑 / 改回数字 / 去「位移」Tab。
 * 「改回数字」把当前宏名的数值原样落回输入框 —— 车走的距离不变, 只是脱离变量单一来源。 */
function openMoveVarPicker(it, idx, onChange) {
  const cur = (it.params[idx] !== undefined) ? String(it.params[idx]) : '';
  const vars = (MOVES.data && MOVES.data.variables) || [];
  const curVar = MOVES.varMap ? MOVES.varMap[cur] : null;
  const body = h('div', {});
  body.appendChild(h('p', { class: 'modal-hint' },
    '这个位移参数可以填数字，也可以绑一个「位移变量」—— 以后改长度只改变量一处'
    + '（「位移」Tab / 调参 Tab），不用回路线里逐块找。绑定写进 map.c 的是宏名，'
    + 'Keil 重编译烧录后生效。'));
  const list = h('div', { class: 'movevar-list' });
  const pick = (v) => { onChange(v); closeTuningModal(); };
  list.appendChild(h('button', {
    class: 'movevar-item' + (curVar ? '' : ' dim'), type: 'button',
    onclick: () => pick(curVar ? String(curVar.num) : ''),
  },
    h('span', { class: 'mv-disp' }, '不使用变量（直接填数字）'),
    curVar ? h('code', { class: 'tn-macro' }, '落到 ' + curVar.num + ' mm') : null));
  for (const v of vars) {
    list.appendChild(h('button', {
      class: 'movevar-item' + (cur === v.name ? ' sel' : ''), type: 'button',
      onclick: () => pick(v.name),
    },
      h('span', { class: 'mv-disp' }, v.display),
      h('code', { class: 'tn-macro' }, v.name + ' = ' + v.num + ' ' + (v.unit || 'mm'))));
  }
  body.appendChild(list);
  if (!vars.length) body.appendChild(h('p', { class: 'hint' }, '还没有位移变量。'));
  body.appendChild(h('div', { class: 'movevar-foot' },
    h('button', {
      class: 'btn small',
      onclick: () => { closeTuningModal(); movesRevealVar(curVar ? curVar.name : null); },
    }, '去「位移」Tab 管理变量')));
  openTuningModal('选择位移变量', body, async () => true, '关闭');
}

/* ---- 右栏: 示意图 ---- */

function movesLoadImage() {
  return new Promise((resolve) => {
    const st = MOVES.data;
    if (!st || !st.image || !st.image.exists) { MOVES.img = null; movesSyncEmptyHint(); resolve(); return; }
    const im = new Image();
    im.onload = () => { MOVES.img = im; movesSyncEmptyHint(); resolve(); };
    im.onerror = () => { MOVES.img = null; movesSyncEmptyHint(); resolve(); };
    im.src = st.image.url + '?v=' + st.image.mtime;      /* 破缓存 */
  });
}

function movesSyncEmptyHint() {
  const st = MOVES.data;
  const missing = !st || !st.image || !st.image.exists || !MOVES.img;
  document.getElementById('moves-map-empty').classList.toggle('hidden', !missing);
}

function movesCanvasXY(e) {
  const cv = document.getElementById('moves-map');
  const rect = cv.getBoundingClientRect();
  return [(e.clientX - rect.left) * cv.width / Math.max(1, rect.width),
          (e.clientY - rect.top) * cv.height / Math.max(1, rect.height)];
}

/* 命中半径: 屏幕 8px 换算成图像像素（画布是图像原尺寸、CSS 等比缩放） */
function movesHitR(cv) {
  const rect = cv.getBoundingClientRect();
  return 8 * cv.width / Math.max(1, rect.width);
}

function movesDistToSeg(x, y, s) {
  const dx = s.x2 - s.x1, dy = s.y2 - s.y1;
  const len2 = dx * dx + dy * dy;
  const t = len2 ? Math.max(0, Math.min(1, ((x - s.x1) * dx + (y - s.y1) * dy) / len2)) : 0;
  return Math.hypot(x - (s.x1 + t * dx), y - (s.y1 + t * dy));
}

function movesBindCanvas() {
  const cv = document.getElementById('moves-map');
  cv.addEventListener('pointerdown', (e) => {
    if (!MOVES.data || e.button !== 0) return;
    const [x, y] = movesCanvasXY(e);
    const r = movesHitR(cv);
    const cfg = MOVES.data.config;
    if (cfg.start && Math.hypot(x - cfg.start.x, y - cfg.start.y) <= r * 1.5) {
      MOVES.drag = { kind: 'start' };
      cv.setPointerCapture(e.pointerId);
      e.preventDefault();
      return;
    }
    for (let i = 0; i < cfg.segments.length; i++) {
      const s = cfg.segments[i];
      if (Math.hypot(x - s.x1, y - s.y1) <= r) { MOVES.drag = { kind: 'seg1', seg: i }; break; }
      if (Math.hypot(x - s.x2, y - s.y2) <= r) { MOVES.drag = { kind: 'seg2', seg: i }; break; }
    }
    if (MOVES.drag) {
      movesSelectSeg(MOVES.drag.seg);
      cv.setPointerCapture(e.pointerId);
      e.preventDefault();
      return;
    }
    for (let i = 0; i < cfg.segments.length; i++) {
      if (movesDistToSeg(x, y, cfg.segments[i]) <= r) {
        movesSelectSeg(i);
        movesDrawMap();
        return;
      }
    }
    /* 空白处按下 = 开始画一条新线段（松手落座） */
    MOVES.drag = { kind: 'new', x1: x, y1: y, x2: x, y2: y };
    cv.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  cv.addEventListener('pointermove', (e) => {
    if (!MOVES.drag || !MOVES.data) return;
    const [x, y] = movesCanvasXY(e);
    const d = MOVES.drag, cfg = MOVES.data.config;
    if (d.kind === 'start') cfg.start = { x: Math.round(x), y: Math.round(y) };
    else if (d.kind === 'seg1') { const s = cfg.segments[d.seg]; s.x1 = x; s.y1 = y; }
    else if (d.kind === 'seg2') { const s = cfg.segments[d.seg]; s.x2 = x; s.y2 = y; }
    else if (d.kind === 'new') { d.x2 = x; d.y2 = y; }
    movesDrawMap();
  });
  cv.addEventListener('pointerup', async (e) => {
    const d = MOVES.drag;
    MOVES.drag = null;
    if (!d || !MOVES.data) return;
    const [x, y] = movesCanvasXY(e);
    const cfg = MOVES.data.config;
    if (d.kind === 'new') {
      if (Math.hypot(x - d.x1, y - d.y1) < movesHitR(cv) * 2) {
        movesSelectSeg(-1);                       // 原地点击 = 取消选择, 不算画线
        movesDrawMap();
        return;
      }
      if (cfg.segments.length >= 64) { toast('线段太多了（上限 64 条）', 'err'); movesDrawMap(); return; }
      cfg.segments.push({ name: '', x1: Math.round(d.x1), y1: Math.round(d.y1),
                          x2: Math.round(x), y2: Math.round(y) });
      await movesSaveConfig();
      movesSelectSeg(cfg.segments.length - 1);
      movesDrawMap();
      return;
    }
    await movesSaveConfig();
    movesDrawMap();
  });
  cv.addEventListener('pointercancel', () => {
    /* 触摸被系统抢走（来电/手势）: 状态复位 —— 不然 MOVES.drag 残留,
     * 之后鼠标划过画布会继续拖着端点/橡皮筋走 */
    MOVES.drag = null;
    movesDrawMap();
  });
}

function movesSelectSeg(i) {
  MOVES.selSeg = i;
  movesRenderSel();
}

function movesRenderSel() {
  const has = MOVES.data && MOVES.selSeg >= 0 && MOVES.data.config.segments[MOVES.selSeg];
  document.getElementById('moves-sel-panel').classList.toggle('hidden', !has);
  document.getElementById('moves-del-seg').classList.toggle('hidden', !has);
  if (!has) return;
  const seg = MOVES.data.config.segments[MOVES.selSeg];
  const sel = document.getElementById('moves-sel-var');
  sel.textContent = '';
  sel.appendChild(h('option', { value: '' }, '（未绑定）'));
  for (const v of MOVES.data.variables) {
    sel.appendChild(h('option', { value: v.name }, v.display + '（' + v.name + '）'));
  }
  sel.value = seg.name || '';
  document.getElementById('moves-sel-label').textContent =
    '选中线段 #' + (MOVES.selSeg + 1)
    + (seg.name && !MOVES.varMap[seg.name] ? '（绑定的变量已删除）' : '');
}

async function movesDeleteSelSeg() {
  if (!MOVES.data || MOVES.selSeg < 0) return;
  MOVES.data.config.segments.splice(MOVES.selSeg, 1);
  MOVES.selSeg = -1;
  await movesSaveConfig();
  movesRenderSel();
  movesDrawMap();
}

async function movesSaveConfig() {
  const st = MOVES.data;
  if (!st) return;
  try {
    const r = await fetch('/api/moves', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ base_version: st.version, config: st.config }),
    });
    if (r.status === 409) { toast('示意图已被其他客户端修改, 请刷新页面后再试', 'err'); return; }
    const j = await r.json();
    if (!r.ok) { toast('保存示意图失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return; }
    st.version = j.version;
  } catch (e) {
    toast('保存示意图失败: ' + e.message, 'err');
  }
}

function movesDrawMap() {
  const cv = document.getElementById('moves-map');
  const ctx = cv.getContext('2d');
  const st = MOVES.data;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  if (MOVES.img) {
    if (cv.width !== MOVES.img.naturalWidth || cv.height !== MOVES.img.naturalHeight) {
      cv.width = MOVES.img.naturalWidth;
      cv.height = MOVES.img.naturalHeight;
    }
    ctx.drawImage(MOVES.img, 0, 0);
  } else {
    if (cv.width !== 800 || cv.height !== 600) { cv.width = 800; cv.height = 600; }
    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, cv.width, cv.height);
  }
  if (!st) return;
  const k = cv.width / Math.max(1, cv.getBoundingClientRect().width);   // 屏幕 px → 图像 px
  const lw = Math.max(1.5, 2.5 * k);
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  st.config.segments.forEach((s, i) => {
    const v = s.name ? MOVES.varMap[s.name] : null;
    const col = v ? SEG_COLORS[i % SEG_COLORS.length] : '#7f8c8d';
    ctx.strokeStyle = col;
    ctx.lineWidth = MOVES.selSeg === i ? lw * 2 : lw;
    ctx.beginPath(); ctx.moveTo(s.x1, s.y1); ctx.lineTo(s.x2, s.y2); ctx.stroke();
    ctx.fillStyle = col;
    for (const [px, py] of [[s.x1, s.y1], [s.x2, s.y2]]) {
      ctx.beginPath(); ctx.arc(px, py, 3.5 * k, 0, Math.PI * 2); ctx.fill();
    }
    const mx = (s.x1 + s.x2) / 2, my = (s.y1 + s.y2) / 2;
    ctx.font = Math.max(11, 13 * k) + 'px sans-serif';
    const label = v ? v.display : '未绑定';
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillRect(mx - tw / 2 - 4 * k, my - 9 * k, tw + 8 * k, 18 * k);
    ctx.fillStyle = '#fff';
    ctx.fillText(label, mx, my);
  });
  /* 正在画的新线段（黄色虚线橡皮筋） */
  const d = MOVES.drag;
  if (d && d.kind === 'new') {
    ctx.strokeStyle = '#f1c40f';
    ctx.lineWidth = lw;
    ctx.setLineDash([6 * k, 4 * k]);
    ctx.beginPath(); ctx.moveTo(d.x1, d.y1); ctx.lineTo(d.x2, d.y2); ctx.stroke();
    ctx.setLineDash([]);
  }
  const s0 = st.config.start;
  if (s0) {
    ctx.strokeStyle = '#2ecc71';
    ctx.fillStyle = 'rgba(46,204,113,0.25)';
    ctx.lineWidth = lw;
    ctx.beginPath(); ctx.arc(s0.x, s0.y, 9 * k, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#2ecc71';
    ctx.font = Math.max(11, 13 * k) + 'px sans-serif';
    ctx.fillText('起点', s0.x, s0.y);
  }
}

/* ==================== Tab: 自定义 ====================
 * 自定义开关 = STEP_IF 的**编译期**条件判据（后端 switches.py）:
 *   新增开关 = app_config.h「自定义开关」段一个 0/1 宏 + app_script.h emScriptCondTdf
 *   生成块一个成员（条件下拉多一个「自定义:xxx」）+ app_script.c 注册表生成块一行。
 *   · 点击开关 = 改宏值（tuning.write_config, 备份 .bak）—— ⚠️ 编译层: Keil 重编译烧录才上车;
 *   · 双击名字 = 改译名（tuning.write_meta 全字段透传 + 生成块重写 → 下拉标签跟着变）;
 *   · 右键 = 编辑弹窗（译名/分类/备注）; 分类只存清单 JSON（纯 UI 分组, 不进固件）;
 *   · 删除 = 服务端先查 map.c 引用（被 IF 用的删不掉 —— 删了就是 undefined identifier）。
 * add/rename/delete/set_switches 会改 app_script.h 的生成块 → 服务端失效积木库缓存,
 * 「地图」Tab 的条件下拉由 catalog 重拉自动跟上（本 Tab 保存后回地图要 Ctrl+F5 是老规矩）。 */
const SWD = {
  loaded: false,
  data: null,           // GET /api/switches 的响应体
  bound: false,
};

function swEnter() {
  swBindOnce();
  if (!SWD.loaded) loadSwitches();
}

function swBindOnce() {
  if (SWD.bound) return;
  SWD.bound = true;
  document.getElementById('sw-add').addEventListener('click', swAddDialog);
}

async function loadSwitches() {
  try {
    const st = await (await fetch('/api/switches')).json();
    SWD.loaded = true;
    SWD.data = st;
    swRender();
  } catch (e) {
    toast('读取自定义开关失败: ' + e.message, 'err');
  }
}

function swRender() {
  const root = document.getElementById('switches-list');
  root.textContent = '';
  const st = SWD.data;
  if (!st) return;
  if (!st.switches.length) {
    root.appendChild(h('p', { class: 'hint' },
      '还没有开关 —— 点「＋ 新增开关」建一个。它会出现在「地图」Tab 的 IF 积木条件里。'));
    return;
  }
  /* 按 group 分节渲染（顺序 = 清单顺序; 空 group 归「未分类」） */
  const seen = {};
  let order = [];
  for (const s of st.switches) {
    const g = s.group || '未分类';
    if (!seen[g]) { seen[g] = []; order.push(g); }
    seen[g].push(s);
  }
  for (const g of order) {
    const card = h('div', { class: 'sw-group' },
      h('div', { class: 'tune-group-head' },
        h('span', { class: 'tune-group-title' }, g),
        h('span', { class: 'tune-group-n' }, seen[g].length + ' 个')));
    for (const s of seen[g]) card.appendChild(swRow(s));
    root.appendChild(card);
  }
}

function swRow(s) {
  const missing = s.macro_missing;      // 宏被人手删了（清单与头文件脱钩）
  const chk = h('input', {
    type: 'checkbox',
    class: 'sw-toggle',
    title: s.name + ' = ' + s.value + '\n点击切换（编译层 —— 重编译烧录后才上车）',
    disabled: missing ? '' : null,
    onchange: (e) => swToggle(s, e.target.checked),
  });
  chk.checked = (s.value === 1);        /* ⚠️ checked 不能进 h() 的属性对象（setAttribute 坑） */

  const row = h('div', {
    class: 'sw-item' + (missing ? ' sw-broken' : ''),
    'data-id': s.id,
    oncontextmenu: (e) => { e.preventDefault(); swEditDialog(s); },
  },
    chk,
    h('div', {
      class: 'moves-var-name',
      title: '双击改译名 · 右键编辑（分类/备注）\n' + s.name + '（app_config.h「自定义开关」段）'
           + (s.note ? '\n备注：' + s.note : ''),
      ondblclick: (e) => { e.stopPropagation(); swRename(s); },
    },
      h('span', { class: 'tn-disp' }, s.display),
      h('code', { class: 'tn-macro' }, 'SCRIPT_COND_SW_' + s.id)),
    h('span', { class: 'sw-state' }, missing ? '⚠ 宏缺失' : (s.value === 1 ? '成立' : '不成立')),
    h('button', {
      class: 'btn tiny danger',
      title: '删除这个开关（被 IF 引用着会删不掉）',
      onclick: () => swDelete(s),
    }, '🗑'));
  return row;
}

async function swPost(payload) {
  const r = await fetch('/api/switches', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  const j = await r.json().catch(() => ({}));
  if (r.status === 409) {
    /* app_config.h 被外部改过（调参/位移 Tab、另一个客户端）→ 本面板的戳过期。
     * 自动重拉清单刷新戳, 用户重试即可 —— 与「标定」Tab revCalibCfg 静默刷新同思路。 */
    toast('app_config.h 已在外部被修改 —— 已刷新数据, 请重试刚才的操作', 'err');
    await loadSwitches();
    return null;
  }
  if (!r.ok) { toast('操作失败: ' + (j.message || j.error || ('HTTP ' + r.status)), 'err'); return null; }
  return j;
}

async function swToggle(s, checked) {
  const j = await swPost({ action: 'toggle', id: s.id, value: checked ? '1' : '0',
                           base_version: SWD.data.tuning_version });
  if (!j) { swRender(); return; }       // 失败: 按服务端状态重画（勾选框回弹）
  s.value = checked ? 1 : 0;
  toast(j.message || '已写回', 'ok');
  swRender();
}

async function swRename(s) {
  const el = document.querySelector('.sw-item[data-id="' + s.id + '"] .moves-var-name');
  if (!el) return;
  editInPlace(el, s.display, async (v) => {
    if (!v || v === s.display) { swRender(); return; }
    const j = await swPost({ action: 'rename', id: s.id, display: v,
                             base_version: SWD.data.tuning_version });
    if (!j) { swRender(); return; }
    toast('已改名 → 生成块与 app_config.h 同步更新（Keil 重编译后条件下拉是新名字）');
    await loadSwitches();
  });
}

function swAddDialog() {
  const name = h('input', { placeholder: 'SW_XXXX', title: '宏名: 大写字母/数字/下划线。定了就别再改' });
  const disp = h('input', { placeholder: '译名（条件下拉里显示的中文短名）' });
  const grp = h('input', { placeholder: '分类（可留空; 之后右键可改）' });
  const note = h('input', { placeholder: '备注（写这个条件什么时候成立）' });
  const body = h('div', {},
    h('p', { class: 'modal-hint' },
      '新增一个开关 = app_config.h 的 0/1 宏 + IF 积木条件里的「自定义:译名」条目。'
      + '状态是编译期的 —— 改一次要 Keil 重编译烧录。宏名定了就别再改。'),
    h('label', { class: 'fld' }, '宏名', name),
    h('label', { class: 'fld' }, '译名', disp),
    h('label', { class: 'fld' }, '分类', grp),
    h('label', { class: 'fld' }, '备注', note));
  openTuningModal('新增开关', body, async () => {
    const j = await swPost({ action: 'add', name: name.value.trim(),
                             display: disp.value.trim(), group: grp.value.trim(),
                             note: note.value.trim(),
                             base_version: SWD.data.tuning_version });
    if (!j) return false;
    toast(j.message || '已新增', 'ok');
    await loadSwitches();
    return true;
  }, '新增');
}

function swEditDialog(s) {
  const disp = h('input', { value: s.display });
  const grp = h('input', { value: s.group || '', placeholder: '留空 = 未分类' });
  const note = h('input', { value: s.note || '' });
  const body = h('div', {},
    h('p', { class: 'modal-hint' },
      '编辑「' + s.display + '」（' + s.name + ' = ' + (s.value === 1 ? '成立' : '不成立')
      + '）。宏名与 id 不可改; 译名改完条件下拉里同步变。'),
    h('label', { class: 'fld' }, '译名', disp),
    h('label', { class: 'fld' }, '分类', grp),
    h('label', { class: 'fld' }, '备注', note));
  openTuningModal('编辑开关', body, async () => {
    const j = await swPost({ action: 'rename', id: s.id,
                             display: disp.value.trim(), group: grp.value.trim(),
                             note: note.value.trim(),
                             base_version: SWD.data.tuning_version });
    if (!j) return false;
    toast('已写回（生成块 + app_config.h + 清单）', 'ok');
    await loadSwitches();
    return true;
  }, '保存');
}

async function swDelete(s) {
  if (!confirm('删除开关「' + s.display + '」（' + s.name + '）？\n'
      + '被 IF 引用着会删不掉（服务端会查）。删除后 map.c 里残留的引用会编译失败 —— 那是有意的。')) return;
  const j = await swPost({ action: 'delete', id: s.id,
                           base_version: SWD.data.tuning_version });
  if (!j) return;
  toast(j.message || '已删除', 'ok');
  await loadSwitches();
}
