# tools/webconsole/ —— 本地 Web 控制台

本目录是 Web 控制台的全部实现（stdlib-only Python + 原生 JS 前端）。这份文档随本目录一起加载 —— 改这里的代码、或改控制台会回写的那些文件（`map.c` / `map.h` / `app_config.h` / `app_script.h` / `app_arm_action.c/.h`）之前，先读它。

启动：双击仓库根 `webconsole.bat`（`server.py`，端口 8410，浏览器自动打开）。**改完服务端代码要重启它**（`allow_reuse_address=False`，双开会直接报端口占用）。已实装十一个 Tab：「地图」「调参」「位移」「自定义」「标定」「采集」「调试助手」「蓝牙」「机械臂」「底盘动作组」「OLED」。

**安全姿态（无鉴权）见仓库根 `CLAUDE.md`；控制台元数据的来源（源码为唯一数据源）见 `docs/console-metadata.md`。**

## 服务端基础设施

- ⚠️ **绑 `0.0.0.0`（`BIND_HOST`），局域网队友能一起用**：终端打「本机」（`127.0.0.1`，即自动开浏览器那条）与「局域网」（发队友那条）；「调试助手」Tab 网络卡片有**「本控制台」**可点即复制。端口常量唯一来源 `host_tools.CONSOLE_PORT`（`server.PORT` 取自它 —— 别再往 `server.py` 写 8410）。
  - **队友连不上先查 Windows 防火墙**：首次绑 `0.0.0.0` 弹「Windows Defender 防火墙已阻止」，勾**专用网络**+允许；**曾点「取消」会留下永久阻止规则、之后一直静默失败**。管理员执行：`netsh advfirewall firewall add rule name="GKS Web 控制台 8410" dir=in action=allow protocol=TCP localport=8410 profile=domain,private`（更严可加 `remoteip=localsubnet`；场地网段怪时去掉）。
- **跨站防护没放宽**：受保护 POST 只放行「host 属本机地址集合 **且** Origin 的 authority 与请求 `Host` 一致」的 Origin（`server.py::origin_ok`），第二道防 DNS rebinding。本机地址集合 = 回环 + 机器名 + 各网卡 IP（`getaddrinfo` + `host_tools.get_local_ip()`，30s 缓存，未命中刷新）；**mDNS/路由器域名那种探测不出来的**写 `GKS_CONSOLE_HOSTS`（逗号分隔）。被拒时 stdout 打实际 Origin/Host、响应体写自救办法 —— 「本机正常、队友一保存就 403」唯一线索，**别删**。
  - ⚠️ **凡会回写仓库源文件的端点都必须进那个 Origin 元组**（`/api/map`、`/api/meta` 原先漏了：白名单有、Origin 元组没有；`test_server.py::TestGuardTuplesExtraction` 钉着）。加写盘端点时**两处都要登记**。
- **多客户端同时编辑**：靠 base_version/409 乐观锁兜底（三份文件 mtime 当版本戳），前端另有 `GET /api/rev` 轮询（5s）做**变更通知** —— 当前 Tab 干净就自动重载并提示，有未保存改动/正在操作就只挂 `#notice`（带「↺ 重载」按钮），**绝不覆盖**。
  - `/api/rev` 回 `{map, tuning, script_h, arm:{c,h}}`，**必须与各写入端点回给前端的 `version` 同源同义**，否则拿版本戳去比永远不相等 → 自己保存完立刻「自动重载」一次（`test_server.py::TestRevisions` 钉）。`script_h` 不能省：`POST /api/meta` 改 `app_script.h`（积木库元数据）时一个字节都不碰 `map.c`。
  - ⚠️ 判据是**严格更新**（`>` + 1e-6），**不能写成 `!==`**：轮询异步，可能收到**保存之前**的旧戳响应，`!==` 会把自己刚写的东西判成远端改动、白重载一次。
  - ⚠️ **只判当前 Tab**；切过去时 `switchTab` 末尾的 `revCheckNow()` 用缓存响应补判。
  - ⚠️⚠️ **光看 dirty 不够，`revBlocked` 那三条不能删**：① 拖拽只在**落座**时才标记（`makeDnd` 的 `beforeChange`）→ 拖动中 dirty 仍 false，半途重载丢掉正在拖的东西；② 地图的参数框与检查器文本框是 `onchange`（**失焦**才触发）→ 只打字不失焦不置脏；③ 指针按着（`ptrDown`，置位/复位、不用计数）+ `BT.playing` 期间同样别重建 DOM。
  - ⚠️ **写盘串行化**：三个写入端点都是「读 mtime 比对 base_version → 整文件写回」，之间有 TOCTOU 窗口 —— 两客户端同一瞬间保存会**都通过**校验再交错写同一文件。各模块一把自己的锁（`server.WRITE_LOCK` / `tuning.WRITE_LOCK` / `arm_action.WRITE_LOCK`，机械臂是 RLock + `@_serialized` 装饰在公开写入口上，校验与写盘分居两函数）。**新增写盘入口时记得带上锁**；跨模块不加全局大锁（三份文件互不相干）。
    ⚠️ 「一把锁」按**文件**算，不按端点：`tuning.write_config`（改值）与 `tuning.write_meta`（改注释）同写 `app_config.h`，**必须共用** `tuning.WRITE_LOCK`（`write_meta` 曾漏，两客户端一改值一改注释就交错写盘）；`server.write_meta`（改 `app_script.h`）同款。
  - **「内容没变就一个字节都不写」是四个写盘模块共用的纪律**（`map.c` / `map.h` / `app_arm_action.c/.h` 都守；**只有 `app_config.h` 仍无条件重写**，字节级相同、只有 mtime 变）。
  - **机械臂 Tab 的重载入口就是提示条那个按钮** —— 原先无任何重载入口（`armReload` 只被 `loadArm`/删除/恢复调用），409 时除 Ctrl+F5 无路可走。别再把它去掉。
  - ⚠️ **`makeDnd` 的拖动重排也要 `armMarkDirty()`**（`ARMDND.beforeChange`）：重排**改条目顺序**，是要保存的改动；只压撤回栈不置脏，拖完换动作/重载就无声丢了。
  - **多人同时用的真约束**：蓝牙链路与摄像头是**单实例**（`bt_link.LINK` / `capture.CAMERA` 各一把模块级锁），多人点动只是排队；一次 `$B,LIST`（~2.7KB 逐行阻塞发）会把别人的滑杆拖顿 → **同一时间只让一个人开「蓝牙/机械臂」Tab**。机械臂编辑模式是**设备全局**的，8s 看门狗（`BT_EDIT_IDLE_TIMEOUT_MS`）由**任何**客户端的任一 `$B` 指令续命；交接时**主动退出编辑模式**。`tuning.json`/`bt.json`/`cap.json`/`phrases.json` 是**共用单文件、后写者胜**。**没有门禁**：同网段任何知道地址的人都能改 `map.c`、驱动机械臂、在你这台机上拉起远程桌面（见「安全姿态」）。
- **`map_codec.py` 是行级编解码的唯一来源**：`map.c` 与 `app_arm_action.c` 的解析/序列化共用它（正则、`_parse_body`、`serialize_map`、`_serialize_step`…）；`server.py` 用 `from map_codec import (...)` 重绑定，故 `server._parse_body` 之类的引用仍有效。⚠️ **别把这些函数再抄回 `server.py`**（`test_server.py::TestCodecIsSingleSource` 拦），也别塞进与具体文件相关的东西。
  - ⚠️ **用户自由文本在写盘前必须过清洗**（`sanitize_tag` / `sanitize_comment_text` / `sanitize_block_comment`，**唯一来源就在 `map_codec`**）：积木名 tag 进 C 字符串字面量（`"` `\` 破坏它、`//` 让 `_ensure_comma` 把尾逗号插进字符串、不成对的 ASCII 括号让 `_parse_body` 计数错位），分区标题与预设名进块注释（`*/` **提前闭合注释 → 后面半行变成活的 C 代码**）。`bt_link.sanitize_tag` / `arm_action.sanitize_comment` / `server._preset_header_label` 只是**带各自缺省参数的薄壳**，别再抄回规则。
    - 清洗**只作用于 `edited` 条目**（未改动的按原始行回吐）。
    - ⚠️ `sanitize_tag` **必须幂等**：`make_arm_snippet` 洗一遍、`_serialize_step` 再洗一遍是常态，不幂等就会洗出两个不同结果。`//` 要**循环**折干净且**先截断再配平括号**。
    - 预设名有**两条**写入路径（新建 / 改名），**两条都要洗**。夹爪子积木的 tag 走 `bt_link._fmt_gripper_step`。
- **积木库缓存按磁盘状态键控**（`server.catalog_key()`）：`Handler.catalog_cache` 只在本进程自己的保存动作里失效是**不够**的 —— 队友 `git pull` / 手改 `app_script.h` 后，前端会检测到 `script_h` 变了并提示「已刷新」。键 = `parse_catalog`/`parse_enums` 真正读到的**每个文件的路径 + mtime_ns + size**（含 `ENUM_GLOBS` 扫到的全部头文件 —— ⚠️ 必须带路径名：新增/删除一个 `Application/*.h` 会改变输出却不动既有文件的 mtime）。**给 `parse_catalog` 加新数据源时记得同步 `_parse_catalog_sources()`**，漏了它改那个文件不会让缓存失效。
## 前端共用件

- **前端两处共用件**（`static/app.js`，两个 Tab 各用一份，别复制第二份）：
  - `makeConnPanel({id, mount, logEl, onConnected})` —— 「连接」面板工厂。DOM 由 JS 现搭，状态全在共享的 `BT` 对象上。
  - `makeDnd(cfg)` —— 自定义 pointer 拖拽（FLIP 让位 / 占位槽 / 6px 阈值 / Ctrl 复制 / 拖出画布反悔）。
    ⚠️ **元素 id 由 `cfg.blockId` 给**：两个画布的 DOM **同时存在于文档里**，都用 `blk-<id>` 的话 `getElementById` 会跨画布取错元素 —— 地图用 `blk-`，机械臂用 `blk-arm-`。
    ⚠️ `beforeChange` 只在**落座时**调一次 —— 「拖动算不算改动」得由调用方自己表达（地图给 `markDirty`，机械臂给 `armPushUndo` + `armMarkDirty`）。
    ⚠️ 关节滑杆**只存在于「机械臂」Tab**（`#arm-joints.bt-joints-box`）。一律用 `jointBox()`/`jointRanges()` 取**当前可见窗格**里那份，且 `renderJoints()` 开头**必须有 `if (!box) return;`** —— 它的调用点不分区格，少了这行会在「蓝牙」Tab 上抛 TypeError，而且**连带滑杆永远轮不到渲染**。
    ⚠️ 滑杆**内容来自设备**（`$B,LIST` 的关节表：范围 + 当前角度），所以 `loadArm()` 与 `btEnterEdit()` 都要主动拉一次 —— 否则「蓝牙 Tab 连好 → 切机械臂」或「连接 → 直接进编辑模式」都看到空面板。
    ⚠️ 定时器只由「链路相关的两个 Tab 之一可见」驱动（`linkTabVisible()`）：两个 5Hz 的 `$B,POS` 会互相抢命令锁。拖动/播放期间还要让路（`BT.dragging`/`BT.playing`）。
- **测试帮手 `srccheck.py`**：`guard_tuples()`（从 `server.py` 源码抠出白名单与 Origin 元组 —— 别再写「某个子串出现几次」那种数法）、`run_js_top_level()`（在 node 的最小 DOM 桩里跑 `app.js` 顶层，抓「页面一加载就炸」的引用错误 —— `node --check` 只看语法看不见）。没装 node 时相关测试自动 skip。
  - ⚠️ **`do_POST` 那两处元组的源码形状是冻结契约**：必须保持「行首 `if self.path ...` + 内联字面量元组 + 只有一个 `)`」—— `guard_tuples` 的两个正则都是 `^\s*if` 加 `.*?`，取**文件里第一处**匹配且到第一个 `)` 就停。抽成 `_needs_origin(path)` 之类的助手、或写成 `startswith(tuple(X))`，抠出来的就是空串/半截（两条测试会失配）。
  - ⚠️ 更隐蔽的那条：**别在文件更靠前处新增 `if self.path.startswith(...) and`** —— `do_GET` 的 `elif self.path.startswith("/static/"):` **含有**子串 "if self.path.startswith("，正则从那儿开始吃，抠出跨 `do_GET` 的一大段，于是两条 `assertIn` **永远通过**（删了路径也绿）。已加 `^` 锚定修掉，`test_server.py::TestGuardTuplesExtraction` 钉着它。
## 「地图」Tab

积木化编辑 `map.c`（读 `/api/catalog`+`/api/map`，保存 POST 回写，未修改条目按原始行回写保证无操作保存 diff 为空）；保存前自动备份 `Application/map.c.bak`。
  - **保存会写两个文件**：`map.c`（预设表 + 预设注册表区块）与 `map.h`（`emScriptPresetIdTdf` 枚举 + `SCRIPT_PRESET_NEXT_ID` 生成块，备份 `Application/map.h.bak`）。⚠️ `map.h` **只在字节真的变了才落盘**（连 `.bak` 都不建）—— 内容没变却白写一次，会让另一个 Tab 的下次保存吃 409（`map.c` 的 mtime 是版本戳）。注册表与枚举必须由**同一份列表**产出，分两次算就会互相对不上。
  - **参数的「启用」联动与勾选框（2026-10 加）**：DSL 尾部 `<启用:参数名=值>`（`server.py::RE_ENABLE_IF`，与 `<显隐:…>` 同一套机器）→ `params_meta[i].enable_if` → 前端 `paramEnabled()`：参数**永远渲染**，条件不满足时输入框 `disabled`（灰的、看得见点不动）——与显隐「整行消失」的差别就这一句。用例：`STEP_BUGUANG_KAI` 的 `pct`、`STEP_BEEP` 的 `times/gap_ms`（跟着 `custom`/`period` 勾选框走）。⚠️ 选项**恰为 0/1 两档**的参数渲染成**勾选框**（`isBoolOptions`），`onChange` 回 `'1'/'0'`；⚠️⚠️ `checked` 绝不能塞进 `h()` 的属性对象（`setAttribute` 写 `checked="false"` = 恒勾选，建完元素直接赋 `.checked` —— 与「蓝牙」Tab 同一个坑）。画布与右侧详情**两处都要**挂启用刷新（`renderBlock`/`renderInspector` 的 `syncVis` 各自收 `r.inp`）。改判据只改 `app_script.h` 的 DSL 注释行，前端零改动。
  - `/api/map` 保存成功后**必须让 `parse_catalog` 缓存失效**：`map.h` 的枚举是「跳转目标」下拉的选项来源，不失效则新建的预设不进下拉，直到重启服务端。
  - ⚠️ **params 的稀疏洞必须在保存前填平（`app.js::save` 开头那次 `Array.from` 稠密化）**：给积木**追加尾槽**后，浏览器里解析出来的老行比新 catalog 短——直接改最尾槽（`it.params[9]=v`）会**跳过中间的下标**留下稀疏洞，`JSON.stringify` 把洞序列化成 `null` → 服务端 `_normalize_param(None)` 崩 → 保存 500 `'NoneType' object has no attribute 'strip'`（2026-10 支点转弯一次加 tail_ms+settle_early 两槽后真踩过：老 F2I6 行 8 参数，用户直接改第 10 格）。服务端 `_normalize_param` 也加了 None 防护（None=空槽=补 0）当兜底——**两层都要在**：前端管"以后不再发 null"，服务端管"旧页面/外部客户端已经发来的 null"。回归测试 `test_sparse_param_holes_serialize_as_zero_not_crash`。
  - **条件分支（「如果」容器）**：拖一块「如果」**一次落三行**（`STEP_IF` + `STEP_ELSE` + `STEP_END_IF`）—— 三块标记必须成对，只落一块立刻失衡。画布上「否则」/「结束如果」是**结构行**（`.block.marker`）不是积木：不可拖、不可点选、无眼睛。
    - **层级是从扁平列表推出来的**（`app.js::scanIfStructure` → `indentOf`），条目 JSON **没有** children/parent 字段，缩进用**行内 margin-left**。所以往任意位置插入都不可能破坏配对（失衡只可能来自"删掉一块标记"）。
    - ⚠️ **分区切分只在层级 0 处开新 zone**：分支里的分区标题退化成区内的**缩进子标题**（`renderInnerSection`）。
    - ⚠️ **拖「如果」头 = 拖整块**（`MDND` 的 `cfg.dragSet: ifSpanIds` → 头 + 两个分支 + 两块标记一起搬，走机械臂那套 `createGroupGhost`/`groupHeight`）⚠️ 标记行**不挂** `onpointerdown`，单独搬一块标记 = 立刻失衡。
    - ⚠️ 把整块拖进垃圾桶走的是 `cfg.deleteItem(头)`（地图没给 `deleteItems`）→ 被 `deleteItem` 的区间扩展接住，删的仍是整块。
    - ⚠️ **删任一标记 = 连同整块一起删**（`deleteItem` 按 `ifSpanIdx` / `elseSpanIdx` 扩区间）；「否则」行上的 ✕ = 删该分支**连同它的内容**。绝不能只删标记那一行。
    - ⚠️ **眼睛是整块的**（`toggleIfBlock`）：块内每一行一起去掉/加上行首 `//`，会**一并清掉块内各步原来各自的禁用状态**。标记行单独的眼**故意不给**。
    - ⚠️ `makeDnd` 的两处改动都是 **opt-in**：`insertPlaceholder` 让占位槽继承落点行的 `marginLeft`；以及新增 `cfg.createItems` 钩子。标记行的 class **必须有 `.block`**，否则 `flip()` 的 `querySelectorAll('.block')` 收不到它。
    - ⚠️ **保存前服务端硬拦**：`map_codec.marker_balance_error`（错误码 `unbalanced_if`），判据见「条件分支」一节。
    - **折叠**：头最左边倒三角（`▼` 展开 / `▶` 折叠）只藏/露这一块 —— **默认折叠**，块内积木与「否则/结束如果」两行都不渲染（只剩一个头 + 摘要 `2 个分支 · 5 步`）。折叠的块拖进垃圾桶仍是**整块**删。
      - **状态存 `localStorage` 的 `gks.map.ifExpanded`**（只记「被显式展开过」的键，空 = 全折叠），**刷新/重开浏览器都不变**。
      - ⚠️ **不要把它挪到服务端文件**（`map.json` 之类）：折叠是**视图偏好**，队友的页面不该因为我把一块折起来就跟着变。与 `bt.json`/`cap.json` 同一条纪律：各机不同的东西不共享。
      - **键 = `<预设 ident>#<嵌套路径>`**（`g_astScriptPreset1#0`，它里面第一个块是 `#0.0`），由 `scanIfStructure` 的 `keyOf` 算（`app.js::ifKey`）。⚠️ **别改用条目 id**：那是 `_parse_body` 按行号生成的（`i0/i1…`），插/删任何一条积木都会整体挪位 → 折叠状态会跳到别的块上。
      - ⚠️ 折叠**不 `markDirty`**：它是视图状态不是 `map.c` 的改动（`test_server.py` 有一条测试盯着 `toggleIfCollapsed` 里不许出现 `markDirty`）。
      - 状态机（`setIfExpanded`）与重绘（`toggleIfCollapsed`）**分开**。
    - ⚠️ **改完 `app_script.h` 要 Ctrl+F5**：浏览器里那份 catalog 是加载时拉下来的，旧的那份没有这三个积木 / `STEP_IF` 没有 `cond` 参数（条件会被写成 0 = 恒成立）。
  - **预设**：画布上方预设栏可切换/新建/复制/重命名/删除路线预设；勾「设备运行此预设」= 写回 `#define SCRIPT_PRESET_ACTIVE`（**切换只改这一行**，map.c 其余字节不动）。新建预设自动播种一步 `STEP_STOP`——C 不允许空初始化列表，任何一张表全被禁用都会编译失败。
    - ⚠️ 新预设的编号从服务端 `/api/map` 的 `next_id` 分配（**只增不重用**），`nextPresetIdent` 别再按「现有最大号 +1」算 —— 理由见「跳到别的预设」。
    - 删除预设前会列出所有**跳转**引用它的路线。⚠️ **条件分支不算引用** —— IF 不跳预设，只是就地嵌套。
    - ⚠️ **手写/手改 `map.c` 预设时，表头 `/* ===== 预设: xxx ===== */` 必须落在数组行上方 5 行以内**（`map_codec._locate_presets` 只往回看 5 行找表头；超了标题解析不到，下次保存会把注册表/map.h 枚举里的预设名退化成表名 `g_astScriptPresetN`）。
      - ⚠️ 那 5 行回溯**撞到上一张表的收尾 `};` 就停**（2026-10 加）：上一张表的表体很短（≤3 行）+ 当前表没有自己的表头时，不停的话会把**上一张表的表头**认成自己的 —— 删/改这张表会连着把上一张的表头一起删掉/改名。
    - ⚠️ **新增预设一律追加到文件最后**：控制台按**文件顺序**解析预设并生成注册表，插在中间会让 id 顺序错位（`test_server.py::TestPresetIds` 会拦）。手写 `map.h` 的枚举行要**逐字符照生成器的格式**（`%-18s = %d,   /* 标题 */`，逗号后 3 空格），否则下次保存会重写那一行。
## 「调参」Tab

把 `Application/app_config.h` 的 `#define` 变成可调参数，保存直接回写（备份 `Application/app_config.h.bak`）。解析/序列化在 `tools/webconsole/tuning.py`，测试 `test_tuning.py`／`test_server.py`。
  - **注释即参数名**。元数据（显示名/单位/换算/范围/备注）写在 `app_config.h` 的**尾注释**里，不进 `tuning.json`。
  - **DSL**（`tuning.py` 顶部有完整说明）：
    ```
    #define NAME  VALUE  /* <显示名> [<显示单位>] {<转换>} @(<范围>) // <备注> */
    ```
    `@(lo~hi)` 以**显示单位**计，任一端可省（`@(2500~)` / `@(~100)`）。值与类型**不进 DSL**：`3000u`/`45.0f`/`(-30)`/`80`/`UART_ID_RPI` 由值本身推断，编辑后按原类型回写，**用户不用自己敲 u/f**。
  - **换算**是闭集，只有 `ms->Hz` 与 `‰->%`（见 `tuning.py::CONVERSIONS`）；加新换算要**前后端一起改**。
  - **每行在显示名旁写出宏名**（`理想停止位中球列 BALL_CX_CENTER`）。宏名是 `<code class="tn-macro">`，**只读**、`flex: none`。
    - ⚠️ **改名（双击 / 右键元数据）只动显示名**：`editInPlace` 的初值就是 `it.display`。
    - ⚠️ **源码里没写显示名时 `display` 会被兜底成宏名**（`tuning.py::public_item`）。
    - ⚠️ 判据（`display === name`）依赖**服务端下发的是兜底后的 display**，所以测试要拿 `tuning.state()` 而不是 `tuning.parse_config()`。
  - **布尔参数渲染成勾选框**（2026-10 加）：判据是 **`kind` 为 `int`/`uint` 且 `@(范围)` 恰为 `0~1`**（`app.js::tuneInput`）。它与文本输入框走的是**同一套** `setTuneValue`/`applyWarn`（控件仍带 `class="tune-val"`），所以**仍是批量语义** —— 勾完只标脏，等「保存」按钮一起提交（不像「蓝牙」Tab 那样勾即写）。
    - ⚠️ **`checked` 绝不能塞进 `h()` 的属性对象**：`h()` 走 `setAttribute`，`checked: false` 会写成 `checked="false"`，而布尔属性**只要出现就算已勾选** → 宏为 `0` 的开关会显示成"开"。必须建完元素后直接赋 `chk.checked`（同「蓝牙」Tab 的 `btParamRow`）。
    - `@(0~1)` 的**浮点**参数不会被认成勾选框（只认 `int`/`uint`），这是有意的 —— 需要小数的开关写浮点即可绕开。
  - **空值哨兵参数（2026-10 加，`FILL_TASK_*_PCT` 首用）**：备注 DSL 写「**留空=255 默认取FILL_TASK_PCT**」（`tuning.py::parse_dsl` 派生 `empty`/`empty_from`，与「选项:」同一条纪律 —— 只派生字段，note 原样往返、`format_dsl` 不知情）。哨兵态（值恰为 255）下 `public_item` 把 `display_value` 发成**空串**、范围告警天然跳过 → 前端渲染**空输入框 + placeholder「默认(60%)」**（按 `empty_from` 从 `TUNE.byName` 现查默认现值，查不到只写「默认」）；空串提交 → `from_display` 写回哨兵字面量 → 数值没变 → 零 diff。要解决的就是「**留空=跟随默认**与**填 0=字面不亮**的区分」—— 固件侧把 >100 一律当「未填」转默认。没有哨兵备注的参数行为一字不变（空串照旧报「不是合法数字」拒保存）。
  - **⚠️ 以后往 `app_config.h` 加参数，请按上面的 DSL 写尾注释**——否则控制台拿不到显示名/单位/范围，只会打回宏名。
  - **全局航向源（2026-10-30 加）**：`LINE_CONV_SRC`（0相机/1陀螺/2不修/3融合，出厂 3）靠备注里的 `选项:` 派生成下拉；六种巡线积木统一读它（原积木的 `src`/`cam_pct` 参数已移除）。融合占比仍是 `LINE_CAM_PCT`（出厂 50，蓝牙 `line_cam_pct` 可当场覆盖）。
  - **⚠️⚠️ 分组靠「段头注释」，而且段头必须是单行的** `/* ==== 名字 ==== */`：`tuning.py::RE_SECTION` 的正则是 `^\s*/\*\s*=+\s*(.+?)\s*=+\s*\*/\s*$`，**只认单行**；写成多行块注释（`/*` 与 `*/` 隔好几行）匹配不上 → 那一段参数会被**静默归到上一段**，「调参」Tab 里看着像"根本没加进去"。
  - **⚠️ 同理，新段要插在上一段的 `#define` **全部结束之后**：段头一旦落在上一段中间，上一段**剩下的** `#define` 会整批被算进新段。加完参数扫一眼分组计数对不对。
  - **范围只做黄框提示，不拦保存**。有些约束是**跨参数**的（如 `APP_PRIO_PID` 必须 > `APP_PRIO_SCRIPT`；`ARM_GROUP_NUM` 与 `app_arm.c` 的初始化表、`afF[6]` 三处耦合），这些只能写进 `// 备注`，控制台不校验。
  - 别往 `app_config.h` 加 `typedef enum`：`server.py::parse_enums` 的 glob 覆盖 `Application/*.h`，加进去会污染「地图」Tab 的参数下拉。
## 「位移」Tab

路线位移变量（`app_config.h` 的「路线位移变量」段）的可视化管理 + 一张示意图（2026-10-29 加）。逻辑 `tools/webconsole/moves.py`，测试 `test_server.py` 的 `TestMoves*`（全程沙箱）。
  - **变量本体与中文译名都在 `app_config.h`**：`#define MOVE_LAMP_X (-490.0f)  /* 灯泡横向 [mm] @(-2000~2000) // … */`。增删改查**复用 tuning.py** —— 改值/改译名走 `/api/tuning`、`/api/tuning/meta`，新增/删除走 `/api/moves` 的 `add_var`/`delete_var`。不存在第二套存储，「位移」Tab 与「调参」Tab 看到的永远一致。
  - **`map.c` 参数槽里写的就是宏名**（「默认路线」主表**及其调用的子预设**——安全/常规扫码、走左/走右边、过减速带、复位回起点——的 `STEP_CHASSIS_MOVE_WORLD_*` 位移参数填 `MOVE_*`，与 `TIMER_CMD_START`/`ARM_ACT_n` 同一待遇，C 编译期展开；测试沙盒预设不在调用链上，刻意不提）。「地图」Tab 里这种参数的交互（2026-10-30 定稿）：
    - **能不能选变量由服务端派生**：四个世界系平移积木的位移参数在 `app_script.h` DSL 里挂了 `<控件:move_var>` 标记（同 `<控件:warm_profiles>` 的机制）—— 前端不写死步骤名，改哪些积木可绑变量只动那一行 DSL。
    - **已绑定（值是宏名）→ 徽章**：显示中文译名；**点击 = 「选择位移变量」弹窗**（换绑别的变量 / 「不使用变量」把当前数值落回输入框 / 跳「位移」Tab），**Ctrl+点击 = 直接跳「位移」Tab**。
    - **未绑定（数字/空）→ 数字输入框 + 「▾」**：同一个弹窗选变量，选完 `it.params[i]` 写入宏名。从积木库拖新积木走的就是这条路（默认空, 不预填任何变量 —— 预填=静默绑错位移）。
    - ⚠️ 徽章**纯视图**（无 input 即无 onchange, 保存零 diff）; 浏览器里积木库还是旧版（没带 widget 标记）时有一条兜底分支照样认得出宏名（`paramInput` 里的 generic badge）。改完 `app_script.h` 要 Ctrl+F5（catalog 是页面加载时拉的）。
  - **⚠️ 控件标记的往返 = `_spec_str` 补、`_parse_param_spec` 剥**（2026-10-30 修的坑）：`_spec_str` 原来不回写 widget —— **右键编辑一次积木元数据, `<控件:move_var>`/`<控件:warm_profiles>` 就无声消失**、专用控件退化成普通输入框。修法与显隐对称（解析剥掉、回写补回）, `saveMetaEditor` 必须**原样透传 widget**（`WidgetWriteMetaSandbox` 正负两条钉着）。⚠️ 手写 DSL 行的版式必须与序列化器一致（`; anI` **一个空格**, 不是三个）—— 不一致首次元数据保存会产生重排 diff（无操作零 diff 破功）。
  - **删变量 = 编译期硬失败的门**：`delete_var` 先行级扫 `map.c` —— **启用行**引用 → 400 拒绝（删了就是 `undefined identifier`，同删 `ARM_ACT_n` 的口径）；**仅禁用行**（行首 `//`，不进编译）引用 → 放行但 message 带警告。**改宏名**没有工具，要连 `map.c` 全部引用一起手改。
  - **示意图是纯可视化**（`moves.json`，**入库**共享）：图片路径（仓库内**相对路径**，`resolve_image` 拦 `..` 与绝对路径 —— 它会进服务端文件读取）+ 起点标记 + 线段绑定表。线段唯一语义 =「这个变量对应图上这一段路」，**刻意不做比例尺/投影换算** —— 数值只在左栏改，图上摆错位置只是难看、不影响任何数据。坐标是**图像像素**坐标（画布按图像原尺寸绘制、CSS 等比缩放），换图重画即可。改动自动保存（base_version/409）。
  - **徽章数据源启动预载**：`init` 里就调 `loadMoves()`（不是进 Tab 才拉）—— 清单到位时若正停在「地图」Tab 会重画一次，护栏同 `/api/rev` 自动重载（`revBlocked('map')` + `!S.dirty`，输入中不重画）。
  - **`/api/rev` 刻意不含 moves 键**：moves.json 只是可视化标注，冲突靠保存时的 409，不值得进轮询。改译名时 `meta` 必须**全字段透传**（conv/range/note —— `write_meta` 是整条尾注释重写，缺哪个哪个被抹掉，`variables()` 因此把全套 meta 下发）。
  - 新增变量的宏名**定了就别改**；DSL 固定 float + `[mm]` + `@(-2000~2000)`。POST 白名单与 Origin 元组**两处都已登记** `/api/moves`。

## 「自定义」Tab

自定义开关 = STEP_IF 的**编译期**条件判据管理中心（2026-10-30 加）。逻辑 `tools/webconsole/switches.py`，测试 `test_server.py::SwitchesSandbox`（全程沙箱）。在这里新增一个开关，IF 积木的条件下拉里就多一个「自定义:xxx」条目 —— 条件成立与否由一个 0/1 宏决定，**改完 Keil 重编译烧录才上车**。
  - **一份数据三处落点，由本模块一起改**：① 清单 `tools/webconsole/switches.json`（**入库**：`{id, 宏名, 译名, 分类, 备注}` + `next_id`，**只增不回退** —— 复用旧号会让 map.c 残留的 `SCRIPT_COND_SW_n` 无声指向别的开关）；② `app_config.h`「自定义开关」段的 SW_* 宏（**状态**，带 DSL 尾注释 → 「调参」Tab 自动可见/可勾）；③ `app_script.h` 枚举尾部 + `app_script.c` 两处「生成块」（条件下拉成员 / 求值 getter / 注册表行 —— 标记之间由控制台整段重写，手改会被覆盖）。
  - **⚠️ 三对生成块标记必须各自单行**（`map_codec.find_marked` 行锚定，多行标记找不到 —— 同调参段头的坑）。生成块行混在 `s_astScriptCond[]` 数组**内部**（初始化列表里的注释行合法），所以固件的线性查询一行不用改。`_rewrite_marked` 的 `tail = lines[span[1]:]` —— `find_marked` 的 `span[1]` 就是结束标记行本身，切成 `+1` 会把标记吃掉（第一次写成功、第二次找不到块，首测真踩）。
  - **动作语义**：`add` = 先宏（`tuning.write_config`，空段也能插 —— `g["items"][-1]` 对空分区 IndexError 的坑同天修掉）→ 再生成块（getter 引用宏，反了留编译错窗口）→ 最后 JSON；宏已在段里而清单没有 = 上次没建完，**接管续建**不重复建宏。`rename` = `tuning.write_meta` **全字段透传**（同「位移」Tab 那条纪律）+ 生成块重写（下拉标签 = 中文名）。`toggle` = `tuning.write_config` 改 0/1。`delete` 先查 map.c 引用（`SCRIPT_COND_SW_<id>`，启用行引用拒绝 / 仅禁用行引用放行带警告 —— 与「位移」删变量同口径）。`set_switches` = 批量保存顺序/分组（顺序 = 生成块书写顺序 = 下拉顺序）。
  - **锁**：写 app_script.h/.c 的生成块**必须持 `server.WRITE_LOCK`**（与 `/api/meta` 的 write_meta 同文件）；锁序固定 server→tuning（switches 动作内部调 tuning 的锁），无反向嵌套 → 无死锁。add/rename/delete/set 成功后失效 `catalog_cache` + `_ENUM_CACHE`（emScriptCondTdf 变了 → IF 条件下拉的来源）。
  - **前端**：`SWD` 模块，点击切换 / 双击改名（`editInPlace`）/ 右键编辑弹窗 / 分组小节。勾选框照旧 **`checked` 不能进 `h()` 属性对象**。宏被人手删了的开关标「⚠ 宏缺失」并禁点。
  - POST 白名单与 Origin 元组两处已登记 `/api/switches`。改完 `app_script.h` 的老规矩照旧：**重启控制台 + Ctrl+F5**。

## 「标定」Tab

主摄标定预设的管理中心（2026-10 加）。一台主摄一套预设：看路/色球/人质/射靶粗调四组里**随主摄个体变**的量（图像列/行设定点 + px↔mm 比例尺，共 19 个；2026-11-01 看路组加 `ROAD_DX_IDEAL_REV_PX`/`ROAD_DY_IDEAL_REV_PX` 反向看路理想Δx/Δy）。逻辑 `tools/webconsole/calib.py`，测试 `test_calib.py`（全程沙箱）。
  - **参数清单的唯一来源 = `tuning.py::CALIB_GROUPS`**（放 tuning.py 防循环导入）。这 19 个宏从「调参」Tab **真移除**（`tuning.state()` 剔除、剔空的段不下发）—— 以后改这些值只在「标定」Tab；但 `tuning.write_config` 照常可写（apply 复用它）。加/减参数要连 calib 校验、`test_calib.py` 的 fixture、入库 `calib.json` 的预设快照一起改；`TestRealSourceContract` 钉着每个宏必须在真实 `app_config.h` 里存在（防改名脱钩）。
  - **两条写盘路径分清**：「保存预设」只写 `tools/webconsole/calib.json`（**入库**，同 tuning.json；`{active, next_id, presets:[{id,name,note,values}]}`，`next_id` 只增不减、服务端发号）；「设为当前」(apply) 才写 `app_config.h` 的宏 —— 经 `tuning.write_config`（.bak/类型保真/行级零 diff 全复用），**值全等时一个字节不写**（连 .bak 都不建，否则「调参」Tab 的版本戳白动一次），随后在 calib.json 记 `active`。apply 的响应带 `cfg_version`，前端回填成已知戳 —— `/api/rev` 的 tuning 键变新不会被误判成远端改动。
  - **仓库随发一套「现役主摄（初始导入）」预设**（2026-10-30 从当时 app_config.h 现值脚本导入, active=1）—— 空文件起步没人会先建预设, 现役家底必须跟着仓库走。`test_calib.py::test_shipped_calib_json_is_complete_and_valid` 钉着它: 每套预设完整快照、值对真实 app_config.h 合法、active/next_id 自洽。
  - **预设必须是完整快照**：save 侧硬拦缺参数/非法值（「半套新相机半套旧相机」比不生效更危险）。`active` 指向的预设不能删（要换先对别套 apply）。名称/备注只进 JSON，轻清洗（剥控制字符/限长），不过 map_codec 的 C 注释清洗。
  - **生效语义**：apply 只改宏，不动任何硬件 —— 要 Keil 全量重编译烧录才上车（「比赛=FLASH 编译表」纪律）。OLED/蓝牙上的调参改的是车上 RAM（Beam 运行时量，断电回宏），现场标完把终值抄进预设再 apply 固化 —— 界面「宏」列与预设值不一致（橙色）= 改了还没 apply。
  - 端点：`GET /api/calib`、`POST /api/calib`（`{action}` ∈ save|apply，契约同 chact）；会写 calib.json 且 apply 改仓库源文件 → POST 白名单与 Origin 元组**两处都已登记**。`/api/rev` 加了 `calib` 键（calib.json 的 mtime）；「标定」Tab 的第二个数据源 app_config.h 复用 `tuning` 键 —— `revCalibCfg`（app.js）看到 tuning 戳变新也静默刷新（否则「宏」列陈旧）。
  - 前端：`#pane-calib` 布局同 chact（flex 列 + 右栏 340px）；预设下拉 ★ 标当前写进宏的那套；「＋ 新建」以**当前宏值**当种子；「设为当前」前若有未保存修改会先自动保存（服务端只认已保存的预设）。懒加载 + `CALIB_BOUND` 绑一次，模式同 chact。
  - **刻意不做**：蓝牙推送/读回（换预设 = 写宏重编译，链路最短）；picam CSI（`*_PI`）与爆爆桶（`BUCKET_*`，OpenMV 320×240）不进预设 —— 换主摄不影响它们；预设回收站。
## 「调试助手」Tab

宿主（PC）侧联机调试，移植自 `pro/调试助手/调试助手.py`。逻辑在 `tools/webconsole/host_tools.py`，单元测试 `test_host_tools.py`（`python tools/webconsole/test_host_tools.py`，全程沙箱，不碰真实 `~/.ssh/config`）。
  - 端点：`GET /api/tools`、`POST /api/tools/action`、`POST /api/tools/phrases`；后两个会起外部进程/改宿主配置，故加了 `Origin` 校验（外站页面 → 403；判据见上文「Web 控制台」的跨站防护）。
  - 常用语存 `tools/webconsole/phrases.json`（**入库**，团队共用）；`dynamic: "latest_log"` 的条目由**服务端**解析。
  - 动作：更新 SSH/网络位置（带「仅预览」）、远程桌面、远程文件夹、远程 Trae、远程 VSCode、树莓派 Web 控制台，外加网络位置地址一键复制（`\\<pi>\shared`）。
  - 树莓派地址 = 本机前三段 + `.125`（`host_tools.PI_HOST_SUFFIX`）；Pi 上 log 目录 = `host_tools.PI_SHARE_LOG_DIR`（当前值 `cv_python\logs` 是 **pro 项目**的路径，GongKe 的 Pi 工程是 `Desktop\GKS_Competition`，换工程时改这一行 + 对应常用语文本）；Pi 控制台端口 = `host_tools.PI_CONSOLE_PORT`（9000）。
  - **改宿主文件前先备份**：`~/.ssh/config` → `config.bak`。
  - ⚠️ 两个别踩回去的坑（详见 `host_tools.py` 注释）：① 生成 `.lnk` **故意**走 PowerShell 的 `WScript.Shell` 而非 pywin32 —— 为保住 stdlib-only，别再引回第三方依赖；② 读 `.lnk` 里 UNC 必须**同时试 ANSI 和 UTF-16** —— 现代 Windows 的 `.lnk` 在目标主机不可达时只有 UTF-16 副本。
## 「蓝牙」Tab

走蓝牙串口（固件 **UART5**，PC12/PD2，115200 8N1）无线读写单片机运行时参数、点动机械臂。传输在 `tools/webconsole/bt_serial.py`，协议/会话/片段生成在 `bt_link.py`，测试 `test_bt.py`（**全程沙箱**：FakePort 顶替真串口、`BT_JSON` 也要 mock，否则会往仓库里写真实的 `bt.json`）。
  - 端点：`GET /api/bt`、`POST /api/bt`（`{action}` ∈ `refresh|connect|disconnect|list|set|mode|jog|grip|pos|snippet`）；会驱动真实硬件，故与 `/api/tools` 同级做 `Origin` 校验（判据同上）。
  - **connect 的 PING 握手带重试**（`bt_link._handshake`，节奏常量 `PING_*`）：COM 口打开成功只代表 RFCOMM 链路建好，模块的 SPP 数据通路/流控未必立刻就绪，头几个字节可能被丢 —— PING 首试 2s、再重试 2 次（各 1s，间隔 0.3s），别改回「一次没应答就判死」。报错分两档：设备回了 `$B,ERR,*`（多为建链瞬间垃圾行触发的 SYNC，固件语义本来就是「请你重发」）→ 带**原文**报「握手失败」，它活着；全程静默才报「没有应答」。⚠️ 别把判据写回 `status != "ok"` 一刀切；「PC 端波特率下拉」对经典蓝牙 SPP **无效**（USB-TTL 直连才生效），要匹配的是**模块 ↔ 主板 UART5 的 115200**（`usart.c`），报错文案里也是这么引导的。
  - **为什么用 ctypes 而不是 pyserial**：Python 标准库没有串口模块，而 Windows 的 COM 口不能当普通文件读写。为保住 stdlib-only 的约定，`bt_serial.py` 用 `ctypes` 直调 `CreateFileW/SetCommState/SetCommTimeouts/ReadFile/WriteFile`。**别再引回第三方依赖。**
  - ⚠️ **串口枚举必须从 PnP 设备树来，不能只看注册表**（4 个蓝牙口曾一个都没列出来）：`HARDWARE\DEVICEMAP\SERIALCOMM`、`.NET GetPortNames()`、`Win32_SerialPort` **在蓝牙 SPP 口上是同一批数据**，而**蓝牙 SPP 虚拟串口只在 RFCOMM 已连接时才注册进 SERIALCOMM** → 模块没上电时 PnP 里有它（`Class=Ports`、状态 OK），这三个来源却都看不到 → 「要连上才能选，要选又得先连上」的死循环。现在 `refresh_ports()` 取**并集**：`_query_pnp_ports()`（`Get-CimInstance Win32_PnPEntity -Filter "PNPClass='Ports'"`，全集，含未就绪的蓝牙口）+ `_enum_registry()`（就绪标记 `ready`）。**打开一个未就绪的蓝牙口本身就会触发 RFCOMM 建链**，所以「未连接」只作界面提示、不拦选择。
  - 认蓝牙口的可靠依据是 `PNPDeviceID` 里的 `BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}`（蓝牙串口 SPP 服务 UUID）；名称/设备路径里的 `BTH`/`BTPORT`/`Bluetooth`/`蓝牙` 作兜底。设备实例 ID 里那 12 位十六进制是该口关联的**蓝牙设备地址**，`Get-PnpDevice -Class Bluetooth` 的 `BTHENUM\DEV_<addr>` 能反查出设备名 —— `refresh_ports()` 已把它塞进 `device` 字段，下拉里显示成「COM35 · 蓝牙 QE_GKS」。地址全 0 = **不挂任何设备的通用传入口**，永远连不上具体模块。
  - ⚠️ **方向判据（改过一次，别再改回去）**：只看实例 ID 里的**设备地址**——
      · 地址**全 0** → **传入(Incoming)**
      · 带**真实设备地址**（12 位十六进制，可用 `BTHENUM\DEV_<addr>` 反查设备名）→ **传出(Outgoing)**，能主动建链
    ❌ 别按 `HardwareID` 的 `_VID&…_PID&…` / `_LOCALMFG&…` 猜方向：本车模块的传出口 HardwareID 是 `_LOCALMFG&0002`，按那个判会**把传出口判成传入口**，把用户带偏。应用程序 `CreateFile` 一个传入口必然得到 `ERROR_FILE_NOT_FOUND`(2)，而**这个错不代表「没配对」**（`_not_found_hint()` 已按「驱动坏 / 通用传入口 / 传入 / 传出但没开机」四档改写提示）。
    **症状**：配对成功、端口变多，但一连就报「不存在」。修法：`控制面板 → 设备和打印机 → 右键该蓝牙设备 → 属性 → 服务 → 勾选「串行端口」`；或用 `蓝牙设置 → 更多蓝牙选项 → COM 端口 → 添加 → 传出`。
    ⚠️ 这些 COM 号**会在每次枚举/改动后重排**（通用传入口的实例索引 0/3/5 变来变去），别用「COM35 就是传出口」这种结论去硬编码，一律现查。
  - ⚠️⚠️ **还有一种更糟的系统级故障：蓝牙串口驱动绑错 INF，一个蓝牙口都建不出来。** 症状：4 个蓝牙口在 PnP 里**全是「存在 + Status=OK + `Device Parameters\PortName` 已分配」**，但 `CreateFileW("\\\\.\\COMxx")` 一律 `ERROR_FILE_NOT_FOUND(2)`，USB 串口 `COM8` 正常；`HARDWARE\DEVICEMAP\SERIALCOMM` 里也**一个蓝牙口都没有**。设备被绑到 OEM INF（`DeviceDesc = @oem3.inf,...Standard Serial over Bluetooth link`）。判别法：`_any_bt_port_ready()` —— 只要**没有任何**蓝牙口进 SERIALCOMM 就是驱动级问题，报错直接指去重装驱动（`REINSTALL_HINT`），**别再让用户去折腾方向/服务**。修法：`设备管理器 → 查看 → 显示隐藏的设备 → 端口(COM 和 LPT)` → 卸载所有「Standard Serial over Bluetooth link」→ 扫描硬件改动 → 重新配对。**兜底路线**：真修不好就插 USB-TTL 转接 —— 控制台对任何串口一样工作，蓝牙只省一根线。
  - **自动选中只在「本机只有一个蓝牙设备」时做**：有 ≥2 台（本车 + 耳机）时留「请选择」占位。选定后记进 `bt.json`，下次自动选中。
  - **`Bluetooth Device (RFCOMM Protocol TDI)` 不是串口**：那是 RFCOMM 传输层设备，**永远不产生 COM 口**，不该出现在串口下拉里 —— 别为它去改枚举。
  - **数据方向**：`BtLink` 的后台读线程是 `ReadFile` 的**唯一属主**；`send_command()` 只写口 + 等事件，绝不自己读。命令锁挂在模块级单例 `LINK` 上。
  - **「Beam 参数」面板完全由设备上报**（`$B,LIST`）驱动：参数名/中文名/分组/类型/单位/范围/当前值统统来自固件 `app_beam.c` 的注册表，控制台侧**不写死任何默认值**。
    - **布尔量渲染成勾选框**（2026-09-27 加）：协议里**没有 bool 类型**（只有 F32/I32/U8/I8），布尔就是一个取值域恰为 `0~1` 的 U8，所以前端靠 **`type === 'U8' && min === 0 && max === 1`**（`app.js::isBoolParam`）认它 → 渲染 `<input type=checkbox>`，**勾/去勾即写**（没有「写入」按钮）。判据放宽会把别的 0~1 参数也变成勾选框。
      - ⚠️⚠️ **不能把 `checked` 塞进 `h()` 的属性对象**：`h()` 走 `setAttribute`，于是 `checked: false` 会写成 `checked="false"` —— 而布尔属性**只要出现就算已勾选**，结果是"关"的开关显示成"开"。必须建完元素后直接赋属性（`inp.checked = ...`）。
      - ⚠️ `value` 要跟着 `checked` 一起维护：`btWriteParam` 读的是 `inp.value`，而勾选框的默认值是 `'on'`。
      - ⚠️ **只挂 `change`，不挂 `blur`**：`btWriteParam` 里那段「置 `p.value = inp.value` 再回读」是为文本框的 blur 竞争写的，勾选框再挂 blur 会把同一个值重复 SET 一遍。
  - **机械臂编辑模式有两个入场时机**（**界面在「机械臂」Tab，不在这个 Tab**）：
    ① **让位窗**（`bt_hs_wait = 1`）—— `ucRpiHandshake` 阶段 1 里置位并调 `vBtEditPump()` 让位，或路线里排了 `STEP_ARM_EDIT_WAIT`（2026-09 加，停车 + 钉住链路 + 等 MAIN）。脚本一离开让位状态 `bt_hs_wait` 归零，`$B,JOG/GRIP/POS/ARM/PLAY` 立即回 `NOTEDIT`；
    ② **脚本根本没在跑**（`vScriptRun` 已返回、`vScriptTask` 停在 50ms 空闲请求循环里）—— 那时路线动不了舵机，放行是安全的，遥测 `script_running` 就是给这个判断/排障用的。
    ③ **`$B,HS`（2026-10 加）＝ 从 PC 远程投一次「握手让出」**（与 OLED 第 3 页同一入口 → `SCRIPT_REQ_HANDSHAKE`）：Pi 在线时脚本多半在跑，不让位就 `NOHANDSHAKE`。「机械臂」Tab **连上后自动走这条**：`onConnected` 读 Beam `rpi_link`，=1（Pi 在线）先发 `$B,HS` 再 `btEnterEdit(true)`；btEnterEdit 的 auto 路径首试失败会**静默重试**（350ms×8 —— 让位要等路线当前这一步跑完，立即报死是假故障）。**退出自动恢复**：`$B,MODE,0` 时若正处让位窗，固件**立刻**踢 Pi 重启客户端（`vRpiKickPiClient`，原来自愈要等 6s 重试闸 + Pi 重启 20s+），链路几十秒自己回 OK。
    **点动与路线抢不了舵机仍是结构性保证**：`vScriptRun` **一进函数就 `vSetBtEditMode(0)` 收回所有权**，所以「脚本停着时进的编辑模式」不会被随后开跑的路线继承。阶段 2（静默确认窗）**不能**调泵：停住会让窗口永不过期而卡死。
    「蓝牙」Tab 现在只剩**调参**（连接 + Beam 参数 + 日志），机械臂编辑整块搬去了「机械臂」Tab；两边共用同一个连接面板与同一条链路。
  - ⚠️ **`STEP_WAIT_HANDSHAKE` 必须留在「设备跑的那张预设」里**（= `SCRIPT_PRESET_ACTIVE` 指向的那张，当前 `g_astScriptPreset1` 第一步）。它是编辑模式的**入场时机**之一；删了它 `bt_hs_wait` 永远是 0，那条让位窗就没了。代价：树莓派不上电时开机即停在阶段 1 —— 这正是台架上调臂的工作态。要在不接 Pi 时单测底盘，把那一行用 `//` 禁用或换预设。
    - ⚠️ 2026-09 起**这条不再是唯一入口**：OLED 第 3 页的「握手让出」会投一次 `ucRpiHandshake`（`s_bHsDone` 已置 → 判为断开重来 → 阶段 1 让位），台架中途调臂走它更直接。
    - ⚠️ 「对端先上电」那种常见开机顺序下，运行预设表首那次握手会**直接放行**（`s_bHsDone=0` 且链路已 OK），`bt_hs_wait` 根本不会拉高 —— 别把「表首有握手」当成一定能进编辑模式。
    - ⚠️ **`g_astScriptPreset2` 从 2026-09 起不再是「运行中的预设」**：它被改成菜单「开始巡线测试」跑的那条（见「OLED 菜单」），表首那两步握手/等键**已被注释掉**，所以它**不可跳入**的标记也没了（`test_server.py::TestPresetJumpRailings` 的断言已改盯预设 7）。
  - **看门狗**：编辑模式期间 PC 若掉线/拔线，8s 无 `$B` 指令后固件自己退出让位（`BT_EDIT_IDLE_TIMEOUT_MS`），不会把脚本永久卡死。OLED 上有 `BT:HS WAIT/ARM EDIT` 指示 —— PC 崩掉时唯一的操作员反馈。
  - **动作片段**由**服务端** `bt_link.make_arm_snippet()` 生成，版式与「地图」Tab 的 `server.py::_serialize_step` **逐字符一致**，贴进 `map.c` 后控制台能原样显示/拖拽/保存。生成器会洗掉标签里的 `"`、`\\`、`//` 和不配对的 ASCII 括号。
  - `tools/webconsole/bt.json` **不入库**（存本机 COM 口，各机不同；与团队共用的 `phrases.json` 不同）。

## 「机械臂」Tab

台架录臂动作 → 存成**动作预设** → 变成「地图」积木库里一块**大积木**（无积木库）。逻辑 `tools/webconsole/arm_action.py`，测试 `test_arm_action.py`（全程沙箱）。
  - **两层独立的东西**：外层 = 路线预设（`map.c`，有 `SCRIPT_PRESET_ACTIVE`）；内层 = 动作预设（`Application/app_arm_action.c/.h`，**没有**「设备跑哪个」的概念，由路线里那块大积木决定）。两文件、两端点（`/api/arm`）、两版本戳。
  - **文件格式**（只替换标记之间，其余手写不碰）：`.h` 的 `/* ==== 生成块: 开始/结束 ==== */` 之间 = `ARM_ACTION_NEXT_ID` + `emArmActionIdTdf` + `ARM_ACTION_NUM`；`.c` 的 `/* ==== 数据区: 开始/结束 ==== */` 之间 = 每预设一张步骤表 + `g_astArmAction[]` 聚合初始化。预设表头**固定三行**：`/* ===== 预设: 标题 ===== */` / `/* 播放方式: 阻塞到位|启动即返回 */` / `/* 备注: ... */`。
    ⚠️⚠️ 三行**无条件重新生成**（手改排版保存即被规范回），步骤表内部行按原始行保留；**聚合初始化每行 5 段**：`{ id, "标题", <0|1>u, 表名, 条目数 },`，第 3 段 = 播放方式（`stArmActionTdf::ucNoWait`，**运行时唯一数据源**，表头 `播放方式:` 注释由它渲染，固件只认字段）。旧文件（4 段）仍读得进来、保存一次即迁移 5 段；段数不对等的行**直接抛错**不静默跳过（静默跳过会让预设从界面消失、保存时数据区整段重生成删掉它）。改段数/顺序必须连 `arm_action.py` 一起改（`RE_ENTRY` / `_serialize_aggregate` 是一对）；**`ARM_ACTION_NEXT_ID` 只增不减** —— `max(ids)+1` 会让删掉最大 ID 后新预设复用那个号，而 `map.c` 可能还留着 `ARM_ACT_n` 引用 → **于无声中指向另一个动作**。
  - **`emArmActionIdTdf` 是「有意命中 glob」**：`server.py::parse_enums` 扫 `Application/*.h`，该枚举**正是**「动作」下拉的来源（绑定 = `app_script.c` 里 `STEP_ARM_PLAY_WAIT` 的 case 体写了 `(emArmActionIdTdf)I(0)`）。别改成 `#define`，否则下拉消失（只能手敲 `ARM_ACT_n`）；「别往被 glob 的头文件加 enum」那条禁令只针对**不打算进下拉**的枚举（如 `app_beam.h` 的 `BEAM_VAL_*`）。
  - **积木库里那块「大积木」是虚拟条目**：`parse_catalog()` 末尾为每个已保存动作克隆一条真步骤条目，`type` 用**合成键** `ARM_PLAY_<id>`（避免顶掉真条目）、`synth_step`/`synth_params` 指明落地成什么。前端必须用 `synth_step` 当步骤名 —— 用合成键会往 `map.c` 写 `ARM_PLAY_3,`（`write_map` 硬拦回 400）。画布上那步**不展开**内部步骤，行首有 `✎` 跳到来源预设改。
  - ⚠️ **`map.c` 用了 `STEP_ARM_PLAY_*` 就必须 `#include "app_arm_action.h"`**：`emArmActionIdTdf`（`ARM_ACT_n`）定义在 `app_arm_action.h`，`app_script.h` 只声明步骤类型本身；少了它报 `identifier "ARM_ACT_1" is undefined`。**不能让 `app_script.h` 去 include 它** —— 两者互相需要（`app_arm_action.h` 要用 `stScriptStepTdf`），循环包含时先被包含的拿不到对方类型。控制台保存 map.c 会**提醒**（`_map_include_warning`，只提醒不拦），`test_server.py` 有测试盯着。
  - ⚠️ **删除动作会让引用它的路线行变成编译错误**（`ARM_ACT_n` 未定义）——**故意的硬失败**，胜过静默指向别的动作。删除前前端调 `POST {"action":"refs"}` 拿引用数与所在预设名再确认。
  - **回收站**存 `tools/webconsole/arm_trash.json`（**入库**，与 `phrases.json` 同侧：被删的动作是项目内容，队友拉下来也应在回收站里）。恢复时发**新 ID**、名字取「原名恢复」「原名恢复(1)」…（原名自带的 `恢复(x)` 尾巴先剥掉）。
  - **显式保存才写盘**；撤回/重做只管当前预设的条目与标题/备注/播放方式，预设增删不走撤回栈（由回收站兜）。
  - **两种「播放」谁阻塞**：`$B,ARM` 固件侧**非阻塞**（改目标 + 启曲线即回 ACK）；「单条 ▶」与「播放整个序列」是**PC 侧轮询式阻塞**（`btWaitDone` 每 120ms 读 `$B,POS` 的 `done`，6s 超时仅兜底）；`$B,PLAY` 是**固件侧阻塞**（`vArmPlayWait` 逐条 `vArmGroupMoveWait`）。
    ⚠️ 三条路都吃同一个 `done` 标志 —— 它提前翻 1 就表现为「**动作提前播放**」，根因与修法见「舵机速度总闸」那节（别在控制台这边打补丁）。
  - ⚠️ **新建预设必须带一条子积木**（前端自动播种当前滑杆位姿）：C 不允许空数组体，空的 `g_astArmActionStepsN[]` 编译不过；空预设列表也一样，`write_arm` 直接拒。同理**一条预设不能所有子积木都被禁用**（`_check_presets` 保存前就拦）。
  - **子积木也有「眼睛」（禁用 = 行首 `//`）**，与「地图」Tab 单步禁用**同一套规则**（复用 `map_codec._toggle_disable`）。位姿带的夹爪两行一起注释（界面一块积木 = C 里两行一起进出数组）。禁用条目在「播放整个序列」里被跳过。
    ⚠️⚠️ **聚合初始化的条目数必须扣掉被禁用的行**（它们是注释、**不进数组**，而 `usCount` 是显式写死的 `_active_line_count`）：多算会让 `vArmPlayWait` 按 `usCount` 越过表尾读别的东西。`map.c` 靠 `sizeof(SCRIPT_PRESET_ACTIVE)` 自动取长度，没这个问题。`raw`（只读块）**不套** `_toggle_disable` —— 它没有眼睛，那些行可能是手写的普通注释行，`_toggle_disable(.., False)` 会把行首 `//` 剥掉、无端改字节（零 diff 就破了）。
  - **没连设备时关节滑杆画成灰色（不可拖），不是空白**。占位表由服务端 `arm_action.offline_joints()` **从源码派生**（`app_arm.c` 的 `s_aemArmGroup`/`s_apcArmJointName`/`s_fArmPoseInit`/哪个轴走 `Angle360At`，`project_config.h` 的 `SERVO_DEFAULT_ANGLE_*`），经 `GET /api/arm` 的 `joints_offline` 下发。**真值仍来自设备上报**（`$B,LIST`），一连上就整体替换。
    ⚠️ 解析 `s_fArmPoseInit` 必须用 `_param_float`（剥 C 的 `f` 后缀）—— 裸 `float("90.0f")` 会失败并静默回退成 0（五个滑杆看着像「位姿全归零」，0° 对舵机是危险位）；灰态下**不要挂 input/pointerdown 监听**（挂了就是「看着能拖，拖了没反应」）。
    ⚠️⚠️ **关节滑杆必须单独一张卡，且 `#arm-side > .tools-card { flex: none; }`**（它原在「位姿」卡里被压成 0 高：滑杆「凭空消失」、`#arm-side` 没溢出、连滚动条都不出现）；这一列只有日志卡 `grow`，其余一律 `flex: none`，滑杆容器再加 `min-height` 兜底。**页面 JS 比服务端旧时 `joints_offline` 不存在，滑杆会静默退化成一句提示**（重启服务端**不会**刷新已打开的页面 → 要 Ctrl+F5；`armReload` 里专门有一条 `btlog` 提示）。
  - **选中可以是一组**：`ARM.selIds` 是「选了哪几块」的唯一真相（`ARM.selectedId` 只是末位）。平点 = 单选，**Ctrl+点击 = 累加/取消一块**（`armToggleSelect`），三种都落到 `armSetSelection` 一个入口。整组能**一起拖走**、**一起右键搬运**、**一起删**。
    ⚠️ 换动作 / 重载 / 保存 / 跳转之后**必须显式 `armClearSelection()`**：两条预设的子积木 id **会重号**（都是 map.c 解析出的 `i0/i1…`），只靠「id 不在 items 里就清掉」挡不住 → 新动作里会有一块「莫名其妙已经被选中」，这时拖滑杆改的就是它。
  - **新积木落在选中块的下面**：`armInsertIdx()`（选中组最后一块之后；没选中才是末尾）+ `armInsertItem()`（插进去并选中它）。
  - **右键子积木 = 复制 / 移动到别的动作预设**（`openArmCtx` → 「…到…」→ 目标预设列表）。菜单 DOM 由 JS 现搭（`armCtxEl`），**不往 index.html 塞静态 DOM**（那条 getElementById 字面量契约会拦）。作用对象是**整个选中组**；右键组外的块会先把选中换成它。
    ⚠️ **菜单自己滚不能把菜单关掉**（2026-09-30 修）：scroll **不冒泡、照走捕获阶段** → init 里 `window.addEventListener('scroll', …, true)` 也收得到菜单内滚动；直接挂 `closeArmCtx` 会让「滚目标列表（`.ctxmenu` 是 `max-height:70vh; overflow-y:auto`）→ 菜单当场消失」，故改挂 `armCtxOnScroll`（target 在菜单里就放过）+ `.ctxmenu { overscroll-behavior: contain }`。`test_bt.py::TestArmMultiSelect` 两条测试盯着。
    ⚠️⚠️ 跨预设搬运：一律**追加到目标预设末尾**（目标画布不可见，挑「插在第几步」没法确认落点）；条目**一律换新 id**（两预设 id 重号是常态 → 原样搬过去就是「一个预设里两块同 id」，症状见 `armRenderCanvas` 顶部那段；本动作内复制则走正常撤回路径）；**跨预设的改动清空撤回/重做**（撤回快照只存当前预设，撤回来会让搬走的块「两边各一份」——会骗人的撤回按钮，日志里说明）。
    ⚠️ 不能把整条动作搬空（源预设成空数组，C 编译不过）→ 拦下并提示。
  - **整组拖动走共用的 `makeDnd`**（共用件，非专用第二份）：`cfg.dragSet(id)`（拖的这块在选中组里 → 整组）、`cfg.selectMany(ids)`（落座后保持整组选中）、`cfg.deleteItems(ids)`（整组拖进垃圾桶 = 一次删光，不是只删第一块）。让位/动画仍走同一条 FLIP 路径（整组 `add('drag-lift')` + 占位槽高度 = `groupHeight(els)`）；幽灵是 `createGroupGhost`（以**第一块**定位）。⚠️ 单块那条路（`createGhost` / `r.height`）一个字没动。
  - **「关节」面板跟着选中走（两种模式，同一组滑杆）**：选中**位姿块** → 滑杆显示**该块存的角度**，拖动即改积木（`edited=true`，离线也能改）；**没选中** → 实时设备值（拖动 = 让机械臂动，录新位姿用），离线退化成灰色占位。可拖条件因此是 `!multi && (!!tgt || live)` —— **多选时必须一律灰掉**（一组积木没有「它存的角度」，退化成实时设备值更糟：拖一下就把组里唯一能改的那块悄悄改掉了）。点**画布空白处**取消选中（事件会冒泡，所以要 `if (e.target.closest('.block')) return;`）。
    **滑杆两端各有一个 ±0.1° 微调箭头**（`mkStep`）：短按一步，按住 400ms 后每 60ms 一步。落库走同一个 `applyJoint`，一步算式抽成 `armStepValue()`（夹量程 + 一位小数，免得 0.1 累加攒出 `90.20000000000002`）。⚠️ 必须 `setPointerCapture`，否则按钮外松手收不到 `pointerup`、连发定时器一直跑（松手后滑块自己爬到底），长按松手多来的一次 `click` 要用 `repeated` 标志吃掉。⚠️ CSS 选择器得是 `.bt-joint .btn.bt-step`：按钮带 `.btn.tiny`，那条规则的 `margin-left: auto` 会把左箭头推到右边去。夹爪那一行**没有**箭头 —— 量纲是整数 %，不是度。
    ⚠️ 改块里的值时只能调 `armRefreshBlock(id)`（就地更新那一行摘要）—— **绝不能调 `armRenderCanvas()`**：拖动中重建 DOM 会把正在拖的滑块丢掉（指针还按着、元素已不在文档里）→ 拖动直接断掉。
    ⚠️ 撤回快照在 `pointerdown` 里压**一次**，不是每个 `input` 一次，否则撤回栈全是中间态。
    **「进入编辑模式 / 退出」按钮在「连接」卡片里**。两个按钮是 index.html 的**静态 DOM**，由 `makeConnPanel` 的 `opt.extra` 搬进它搭的那行按钮里（⚠️ 工厂开工第一步会清空 `#arm-conn-mount`，所以得在调用前先把元素引用交出去）；id 必须留在 HTML 里 —— 服务端有一条测试核对 app.js 里 `getElementById` 的字面量都在 index.html 里。
    **进出编辑模式成败都弹 toast**（含连上后自动试的那次，失败文案前加「未自动进入」）—— `btRun` 的 `opts.onError` 就是为「静默失败要换个说法报出来」的调用方开的。
    **进编辑模式后会自动 `$B,POS` 一次**（等同点一次「获取当前位姿」，`btEnterEdit` 末尾）：`$B,LIST` 的关节表里**没有夹爪**，不补这一步，夹爪控件显示上一次的开合/程度，直接拖就等于把臂往一个凭空的姿态上带。自动那次是静默的（`{quiet:true}`）。
  - **每块自带「改过没有」的黄标**：基线 `ARM.base`（= 上次加载/保存时各条目的副本）与当前条目逐块比对（`armItemModified`），对不上 → 积木**左边框变黄**（`.armmod`）、底部按钮排出现「↺」（`armRevertItem`，恢复时保住 id）。基线只在 `armReload` / `armSave` / `armSwitchPreset` / `armAddPreset` / `armDupPreset` / `armOpenPreset` 重建，且**必须先 `armSyncItems()` + `armBaseCapture()` 再 `armRenderAll()`** ——顺序反了就是拿上一条/保存前的基线去画。
    新加的块（没有基线）也算「改过」（黄），但没有可恢复的原值 → 不给 ↺ 按钮。
    ⚠️⚠️ **别把基线重建塞进 `armRenderAll`**（调用点含撤回/重做，重建会让刚撤回的东西立刻变「没改过」）；黄标与 ↺ 的刷新走 **`armRefreshModified()`，由 `armRefreshBlock()` 调**（排在延时/只读块的那个早退**之前**）—— 只在 `armRenderBlock` 里挂类名的话，拖完半天块还是不变黄。
  - **灰态也能「记录当前位姿」**：`armRecord` 刻意**不要求先进编辑模式**（原要求，故没连设备时点不动）。灰态下记的是**固件上电默认位姿**（`joints_offline`，不是 0°），在线时记设备刚上报的实时角度；**只录数据、一条指令都不发**。
    ⚠️ 记下来的永远是**原始角**（`$B,POS` 读数，不含任何偏移）；组动作播放时会叠加「臂偏移」（见上文「组动作角度偏移」）—— 所以「编辑模式拖准的位置」与「播放到位的位置」是两个数，两者之差正是那个参数要你填进去的东西。
  - ⚠️⚠️ **每条子积木必须带唯一 `id`**（服务端 `_items_from_body` 负责，前端 `armRenderCanvas` 再兜一次）。它同时是 DOM id（`blk-arm-<id>`）、选中判据、拖拽落点的 skip 判据、删除的定位键。漏掉时的症状**看着毫不相干**：① 点一块结果**全部**被选中（`h()` 跳过 `undefined` 属性 → `data-id` 没设 → `undefined === undefined` 对每块成立）；② 拖动时其它块**不让位**（`id === skipId` 对每项成立 → 占位槽永远落在末尾）；③ 删除永远删掉**第一块**。反查时先看 `id`。
## 「OLED」Tab

编辑 `Application/app_oled_menu.c` 的**数据区**（OLED 菜单的表），右栏 128×64 预览。逻辑 `oled_menu.py`，测试 `test_oled_menu.py`（**全程沙箱**）。
  - **两层独立的东西**（同「机械臂」Tab 的形状）：外层 = 固件的 `app_oled_menu.h`（动作码 / 调参量两个枚举，**控制台只读不写**）；内层 = `app_oled_menu.c` 的数据区（各页项表、子表、页面表，控制台整段生成）。三个端点/戳：`/api/oled`、`revisions()["oled"] = {c, h, glyphs}`。
  - **⚠️ 核心分工: 表是「结构」, `switch` 是「动词」** —— 能编辑的是标签/顺序/启停/指向哪个预设或子表/绑哪个调参量；**行为**（取值/步进/量程/离散档位/副作用/蜂鸣/前置条件）全在 `app_oled.c` 的 switch 里。加一个**动作码**要改固件（`.h` 枚举 + `app_oled.c` 的 case），不是控制台能生成的。
  - **⚠️ 一页 / 一个子菜单最多 32 个启用项**（控件名 `MAX_ITEMS` / `SUB_MAX_ITEMS`，服务端保存前硬拦 400）。屏一屏只画 4 行（64px/16px），放不下的靠 UP/DOWN **滚动**查看，右侧画竖直滚动条 —— 窗口模型（`s_ucTop` + `vOledWindowSync`，固件侧）与预览同一条**最小幅度跟随**规则。子菜单的「返回」由固件补在**最后一条**、随列表滚动、**不占**这 32 个额度（它不进表，于是「生成表漏了返回 → 操作员卡死」这个失效模式在构造上不存在）。⚠️ 滚动条占最右 8px ⇒ **菜单标签宽度预算从 128px 收到 120px**（`LABEL_PX = ROW_PX - SCROLLBAR_PX`；现存最长标签 `巡线状态:待实现` 就正好 120px，**零余量**）；调参行仍是 64px。前端有实时宽度预警与预览里的「返回」幽灵行。
  - **换行动画（2026-10 加，固件侧）**：上下选择时选中那一行的文字从右侧滑入（8px → 0，约 100ms）。开关是**固件**的事，控制台只有两个入口：**「调参」Tab** 的 `OLED_MENU_ANIM_EN`（`@(0~1)`，编译期上电初值）与**「蓝牙」Tab 新分组「OLED」**的 `oled_menu_anim`（U8 域 0~1 ⇒ 自动渲染成勾选框，**当场生效**、断电回宏值）。⚠️ 预览**不画**这个动画（它是逐帧过渡，画布只画稳态；预览的「即验收」不覆盖它）。
  - **⚠️ 子表的书写顺序由服务端自动排**（`_stable_topo_order` 稳定拓扑排序）：C 要求符号先声明，但**不该让排菜单的人操心**。对已经合法的文件是**恒等变换**（无操作保存仍零 diff，有测试盯着）；成环则 400。
  - **⚠️ 子表用直接符号引用**（`OLED_ITEM_SUB("执行任务", 1u, &s_stListTaskExec)`）而不是 id 注册表：子表被删/改名会变成 **`undefined identifier` 编译错误**。这是**有意的硬失败**（胜过重排后无声指向另一张表）—— 控制台**不会**提示，删了要自己接编译错误。
  - **⚠️ 子表必须写在引用它的表之前**（C 要求符号先声明）：`order` 是**文件顺序**，保存会按它书写。校验会拦「子表排在引用者之后」。
  - **⚠️ `parse_enums` 的形状约束**：`app_oled_menu.h` 的枚举**一行一个成员**（跨行成员被静默丢掉、下拉里选不到）、**不加 `_COUNT` 哨兵**（会被当成下拉选项）、**值只增不重用**（复用旧值 = 某个菜单项无声地换了含义）。三个下拉（动作/参数/子表）的选项都由 `server.py::cached_enums()` 从源码派生，前端不写死任何成员名。
  - **⚠️ 保存时自动补字**（`save()` 的事务性流程）：差集追加进 `tools/fontgen/glyphs.txt` → `subprocess` 跑 `gen_font.py` 重写 `Core/Src/OLED_Data.c` → 跑 `check_coverage.py`（非 0 只作 warning）。**任一步失败就把已写的文件回滚**（三个文件都 `.bak`）。
    - ⚠️⚠️ **子进程必须注入 `PYTHONUTF8=1`**：本机控制台是 GBK，`check_coverage.py` 打印 `✓`/`✗` 会抛 `UnicodeEncodeError` 并以**退出码 1** 结束 —— 即使覆盖其实是完整的。不注入就把「成功」误判成「失败」（实测复现过）。
    - ⚠️ **控制台不 import Pillow**，只调子进程 —— stdlib-only 硬约定不破（与「采集」Tab 的 cv2 是同一条纪律）。
    - ⚠️ **没加新汉字时不重跑 `gen_font.py`**（否则每次保存都白重写一遍 `OLED_Data.c`）；菜单没变又没新字时**一个字节不写、一个子进程不起**。
    - ⚠️ **`check_coverage.py` 的 `DEFAULT_TARGETS` 必须含 `Application/app_oled_menu.c`**：漏了 = 菜单标签完全不在覆盖检查里，改文案后屏上出方框问号而**这条检查一声不吭**。
  - ⚠️ **保存不会编译**（用户口径）：控制台只到源码 + 字模，**编译烧写回 Keil**。保存成功会 toast 提醒。
  - **画布画的是整个「OLED 模块」**（`app.js::oledDrawModule`）：深色 PCB 外框 + 四角安装孔 + 底部金色排针 + 中间的屏玻璃。逻辑尺寸 144×96（`OLED_MOD`），玻璃区在 `(8,10)` 起 128×64，CSS 3 倍整数放大 + `image-rendering: pixelated`。**CSS 不再自己画边框/底色**（`#oled-screen` 只有尺寸与阴影）。
  - ⚠️⚠️ **配色：黑底白字是常态, 反色（选中）那行才是白底黑字** —— 与真实 OLED 一致。第一版画反了（整屏白底黑字、选中行深底白字）。
  - **反色那行 = 光标位置, 跟着画布选中走**：点哪一块预览就反色哪一行（`oledSelectItem` 里顺手重绘预览）。选中的那块被藏起来/不在本表 → 退回第一行（= "刚进这页"的样子）。⚠️ 条目数 > 4 时预览画的是**窗口**（按选中块定位, 与固件 `vOledWindowSync` 同规则）+ 右侧滚动条（`oledDrawScrollbar`, 几何与固件逐条对齐）。
  - ⚠️ **预览面板刻意**不留说明文字（只有警告：超行、缺字）。「128×64」「黑底白字是常态」这类都被去掉了 —— 前者还不准（画布逻辑尺寸是 144×96, 128×64 只是里面那块玻璃）。
  - **预览按 `glyphs.txt` 的覆盖率画**：字表里没有的汉字画成「方框问号」，与固件 `vOledShowChinese` 的兜底图形一致 ⇒ **预览即验收**。排版常量（行高 16 / 屏宽 128）与 `app_oled.c` 的 `OLED_Y1..Y4`/`OLED_ROW_H` 是同一套，`test_oled_menu.py::TestPreviewMatchesFirmware` 四条断言盯着（含「玻璃区必须正好 4 行高」「CSS 尺寸必须是逻辑尺寸的整数倍」）—— ⚠️ 这些断言是**后补的**：文档曾经先写了「有断言盯着」而测试并不存在，别再让文档跑在代码前面。
  - **多客户端同步**：`oled` 的戳是 `{c, h, glyphs}` 对象（同 `arm`），`REV_FILE` / `revEditing` / `revLoaded` / `revKnown` / `revNewer` / `revReload` / `revEvaluate` 各有分支。
  - **三栏与「地图」Tab 同构**：左 = **积木库**（块 = 动作码）、中 = 画布（`.zone > .block`，拖拽走共用的 `makeDnd`，实例 `OLEDDND`，DOM id 前缀 `blk-oled-`）、右 = 预览。
  - **页面排序**：菜单页 chip 上有 `◀`/`▶`（`oledMovePage`）。⚠️ 挪不出前两条代码页之外（`j < 2` 直接挡住）——「0=状态页、1=二维码页」是**跨文件契约**。⚠️ 这里**没用** `makeDnd`：那套是纵向几何（按 `clientY` 算落点）+ `.zone/.block` 结构，页条是横向的，硬套要改共用件。⚠️ 换序只动 `pages` 数组顺序 → 不用标 `edited`（每一条的 raw 行与位置无关）。
  - ⚠️ **非汉字符号（×√°Δ…）自动补字**：`missing_han` 只收汉字/全角，2 字节符号（Δ U+0394、° U+00B0 之类）**不会**被自动补 —— 标签里要用就得手动进 `gen_font.py` 的 `EXTRA_WIDE` + `glyphs.txt` + 重跑 `gen_font.py`（2026-11-01 加 Δ 时补的这条）。
  - ⚠️⚠️ **加新动作码时, 它的汉字可能要手动补字表**：**块名**取自 `app_oled_menu.h` 的枚举尾注释（`.h` 的注释 `check_coverage.py` **不扫**），而**运行期标签**（如 `陀螺源:主/副`）是 `app_oled.c` 里的字符串字面量 —— **控制台的自动补字只看菜单表, 看不到运行期标签**。所以新增带运行期标签的动作码时：改完 `.c` 必须**手动把那几个字加进 `glyphs.txt` + 跑 `gen_font.py`**，判据是 `check_coverage.py`（它扫两个 `.c`）。块名的字不用管 —— 拖进菜单时它才会变成 `app_oled_menu.c` 的字面量，那时控制台会自动补。
  - **可用的动作码**（`app_oled_menu.h`）除了原有的关机/握手让出/执行任务/巡线测试/开关激光/拍照/子菜单/调参，2026-10 又加了三个：**跑预设**（= 执行任务**去掉二维码前置门**，台架上跑测试预设用）、**中止脚本**（`vScriptAbort` —— ⚠️ 原来菜单上**根本没有"停"的能力**）、**陀螺源**（主/副切换，标签带 `:主`/`:副`；⚠️ 切源瞬间 yaw 会跳到另一颗的零点，**先停车再切**）；之后再加一个 **名字调参**（`OLED_ACT_TUNE_PARAM`，见下条）；2026-10-23 再加 **计时**（`OLED_ACT_TIMER`，参数 0=开始/1=结束/2=重置，控制状态页第 4 行秒表，与地图 Tab 的 `STEP_TIMER` 共用 `vTimerCmd`）。
  - ⚠️⚠️ **动作码带参数时，`oled_menu.py` 里要登记两处，否则生成的行会丢掉参数**（2026-10-23 现场踩过）：
    `ACT_ARG_ENUM`（动作码 → 参数取自哪个枚举；决定前端「参数」那一格渲染成下拉还是禁用）和
    `ACT_NEEDS_PRESET`（这些动作的 `arg` 必须是真预设，拦住 `SCRIPT_PRESET_NONE`/空）。
    漏了两处的后果**不报错、只是静默失效**：控制台按无参生成为 `OLED_ITEM(...)`，固件拿到
    `ucArg = 0` —— 「开始射靶测试」（`OLED_ACT_SHOOT_TEST`，参数 = 预设 id）就是这么变成
    `MENU: task preset id not found`（屏上按了什么都不动）。正确的行：
    `OLED_ITEM_A(OLED_ACT_SHOOT_TEST, "开始射靶测试", 1u, SCRIPT_PRESET_34)`。
    ⚠️ 改完要**重启控制台**（模块启动时加载）+ 浏览器 Ctrl+F5。`test_oled_menu.py` 的
    `test_shoot_test_rows_carry_a_real_preset` / `test_run_preset_and_post_task_need_a_preset_argument`
    盯着这条形状（机制扫查，挪表/改标签都不假红）。
  - **名字调参（通用调参行，2026-10 加）**：块名「名字调参」，行的**第四个字段**是**参数名** —— `OLED_ITEM_P(OLED_ACT_TUNE_PARAM, "巡线Kp", 1u, "LINE_KP")`。固件按名字现查（`app_oled.c::vOledTuneParamStep/Text`），所以**加一行、换个名字都不用改固件**（这是它与「调参」（`OLED_ACT_TUNE_QTY`，5 个硬编码量）的区别）。
    - **参数名从哪来**：`oled_menu.py::param_catalog()` **现扫两张源码表** —— `app_beam.c::s_astBeamParam`（本机，含只读遥测）+ `app_rpi.c` 的生成块（树莓派 `config.py` 的参数，由 `tools/gen_rpi_cfg.py` 生成）。跟着 `/api/oled` 的 `params` 段一起下发（~80KB，只在页面加载/重载时拉）。
    - **前端**：那种行的「参数」那一格是个**弹窗按钮**（`oledOpenParamPicker`，id `#paramodal`）：来源切换（全部 / 本机 Beam / 树莓派）+ 搜索 + **按分组归类**，点一下即选定，并顺手把标签跟填成中文名或参数名 —— ⚠️ **放不下就不填/退到参数名，绝不裁半截**（`巡线PID-Kp` 裁成 `巡线PID-` 那种名字比不填还坏）。
    - **服务端保存**校验（`_check(..., valid_params=...)`，服务端每次现扫两张表传进来）：名字必须是合法标识符**且**在表里真的存在 —— 写错 = 车上显示 `?` 并长鸣的死项。⚠️ 别的动作码带了 `param`、或它自己没带，都拦。
    - ⚠️ **两边行为不一样**：本机（Beam）参数**现取现写、零延迟**；树莓派参数走**后台轮询缓存**（`app_rpi.c` 的「Pi 参数缓存」段，只查"正在屏上显示"的那几项，一次一条在飞），改值发 `"P,<名>,<值>"` = **临时值**（只改 Pi 进程内存，不落盘，Pi 重启还原）。协议见 `docs/串口协议.md` §6k。
  - **⚠️⚠️ 积木库里的块名 = 动作码枚举的尾注释 = 拖进来之后的默认菜单标签**（也就是 OLED 上那行字）。所以 `app_oled_menu.h` 里那两条枚举的尾注释写的是**用户看得见的话**, 不是给自己的说明 —— 说明写在枚举上方的块注释里。**改尾注释 = 改积木库里的块名**。
  - 拖进来后标签仍可改（要「陀螺目标」而不是「调参」就改标签）；标签被改过时行尾才显示动作码徽标, 免得每行都重复一遍块名。
  - ⚠️ **带参数的动作预填第一个选项时会跳过 `*_NONE` 哨兵**（预设枚举第一项是 `SCRIPT_PRESET_NONE` = "不跳预设"，预填它 = 静默拿到一个空指向）。⚠️ **拖「子菜单」进来总是新建一张表**（复用现有那张会让两块指向同一张表, 是看不见的耦合）——想指向已有的用下拉挑。
  - ⚠️ **超限预警要用「条目总数」判**（子菜单还要算上固件那条「返回」），不能在截断之后再判 —— 那样恒不成立、预警永远不弹（写过一版, 被 node 探针抓出来了）。现在两条：`> 16` 报「看不见选不到」，`> 4 且 ≤16` 报一句信息级「共 N 项, 预览画的是第 x..y 项」。
  - ⚠️⚠️ **画布上的块要拖得动, 必须自己报候选态**：`makeDnd` 只管「超过 6px 之后的移动与落座」, **候选态得由元素自己的 `pointerdown` 调 `OLEDDND.begin({mode:'move', id}, e)`**。漏了这一句的表现就是「块完全拖不起来」（而积木库那边能拖, 因为它自己有 `begin`）—— 踩过一次。
  - **标签与表标题平时都是一段静态文字, 双击才变输入框**（复用地图的 `editInPlace`：`replaceWith` 成 `<input>`、回车/失焦提交、Esc 还原）。常驻输入框会让人以为那里是"输入区"而不是"显示名"。⚠️ 全局 `.inline-edit` 是 `width: 60%`, 必须用 `.oled-item .inline-edit` / `#oled-list-title .inline-edit` 压窄 —— 屏一行只有 128px, 拉满整行既误导又难看。
  - ⚠️ **点一下就要选中（`.sel` 高亮）, 而 `pointerdown` 里必须调 `oledSelectItem()` 就地改 class** —— 只改 `OLED.selItem` 不刷 DOM 的话**点击完全看不出选中**（拖动时能看到, 那是 `makeDnd` 落座后重绘顺带刷出来的, 于是表现为「只有拖动才算选中」）。⚠️ 但**不能**在那里 `oledRenderCanvas()` 整片重绘: 会把**正在按下**的那个元素换掉。
  - ⚠️ **删页面 / 删表都要 `confirm`**（会连带收掉没人引用的那张表, 是破坏性动作）。页面的 ✕ 在 chip 上, 要 `stopPropagation` 免得顺带切页。
  - ⚠️ `.block` 带 `user-select: none`，双击改名的输入框必须显式 `user-select: text` 才选得中文字。`#oled-screen` 3 倍整数放大 + `image-rendering: pixelated`。
  - ⚠️ **探针模拟拖拽时要发**两次以上** `pointermove`**：第一次只够触发 `startDrag`，垃圾桶/落点判定发生在**后续**的 move 里 —— 只发一次会得到「拖进垃圾桶没删掉」的假故障（功能其实是好的）。
  - **验证手法**：`node` 里用最小 DOM 桩把 `loadOled() → 渲染 → 加积木` 整条链跑一遍（渲染路径是 `run_js_top_level` 覆盖不到的 —— 那个桩的 `getElementById` 恒返回 null）。

## 「采集」Tab

摄像头预览 + 分类采集数据集，给 `vision/` 人质检测模型攒图。逻辑 `tools/webconsole/capture.py`，测试 `test_capture.py`（**全程沙箱**：`FakeCv2`/`FakeCamera` 顶替、路径全 patch，**没装 opencv 也要全绿**）。原独立 Flask 工具（`Desktop\yolo\Capturer\`）已并入并删除。
  - 端点：`GET /api/capture`（状态+各类张数+偏好）、`GET /api/capture/frame.jpg`（当前帧，无帧回 404）、`POST /api/capture`（`{action}` ∈ `scan|open|capture|counts|prefs|reveal`）。会开真实摄像头并往仓库写图，故与 `/api/bt` 同级做 `Origin` 校验（判据同上）。
  - ⚠️ **stdlib-only 的唯一例外**：`capture.py` 顶部 `try: import cv2 / except ImportError: cv2 = None`。它是**可选**依赖 —— 没装时 `capture_state()` 回 `available:false`+提示，采集 Tab 显示 pip 命令，`server.py` 与其余四个 Tab 一位不动。**别把 cv2 变硬依赖、别引回 Flask、也别 import numpy**（「没画面」交给前端 CSS 遮罩解释状态；`cv2.putText` 是 ASCII 字体，写不出中文）。
  - ⚠️ **预览是「单帧轮询 + canvas」，不是 MJPEG**。`server.py` 全文件只有 `_send` 一个响应写入口，它必写 `Content-Length`；任何流式响应一旦有异常逃到 `do_GET` 外层的 `except → _send(500)`，就会往半个流里插第二个状态行。轮询用现成的 `_send` 发单帧 JPEG，结构上没这个可能。**要改成流式，先把这条想清楚再动。**
  - ⚠️ 路由是**精确匹配**，`do_GET`/`do_POST` 开头会 `self.path.partition("?")[0]` 剥掉查询串——前端靠 `?t=` 破缓存，不剥就 404。新增端点照旧用 `self.path ==` 比即可。
  - 摄像头由**一个后台线程（camera-watchdog）独占**：`ThreadingHTTPServer` 每请求一线程，多线程碰同一个 `VideoCapture` 会崩；请求线程只读缓存帧引用。命令锁挂**模块级单例** `CAMERA`（同 `bt_link.LINK`）。断线按 `DISCONNECT_THRESHOLD`/`RECONNECT_INTERVAL` 自动重连，且**只重开原 index，不偷偷换摄像头**。
  - **服务端不主动抢摄像头**：不 `open` 就一个设备都不开；「记住上次的摄像头」是前端拿 `prefs.index` 去调 `open` 实现的。
  - **总开关「启用」**（Tab 里摄像头卡片右上角）：关掉时前端调 `close` 动作，让**服务端立刻释放设备**并停掉全部定时器，同时清空 canvas（留着最后一帧会让人以为摄像头还开着）。状态写进 `cap.json` 的 `enabled`，所以**下次开控制台也不会自动去抢摄像头**。⚠️ 别把它做成「只停前端请求」——服务端看门狗线程会继续以 30fps 占着设备。
  - 落盘 `vision/datasets/raw/<类别>/<毫秒>.jpg`（与 `vision/scripts/capture.py`、`gui.py` 导入器同命名，`split_dataset.py` 会再缀一次目录名；`vision/datasets/` 已 gitignore）。**类别 = `vision/data.yaml` 的 `cylinder/cone/drum`**，外加 `all/` 三者同框暂存桶（`split_dataset.py` 的 `TRAIN_CLASSES` 会跳过它）。⚠️ 旧工具第一类叫 **`pellet`**，它在固件里就是 `SHAPE_DRUM`（`app_beam.h`：「腰鼓 drum（Pi 模型类名 pellet）」）——**别再把这个词带进新代码**。
  - ⚠️ **后端选择铁律：第一个能稳定出图的就用，不再挑最高分辨率**（`capture.py::_combo_table` / `_open_cap`）。本机实测（OpenCV 5.0.0 + 这颗 UVC 相机）：MSMF 设 `FOURCC=MJPG` 后**一帧都出不来**（0/30，报 `cap_msmf ... can't grab frame`），而 DSHOW(MJPG/默认) 与 MSMF 默认都稳；「逐个试完再挑最高的」会反复开合同一设备（已选中的那个还握着不释放），把 UVC 带进**「先吐一帧、之后每次 read 都失败」**（预览永远 404、状态 ok↔connecting 刷屏）。所以顺序是 DSHOW+MJPG → DSHOW → 平台默认 → MSMF+MJPG（垫底），且认一个组合可用要**连续 `STREAM_CHECK_FRAMES` 帧**都成功（只读到 1 帧不算数）。
  - `tools/webconsole/cap.json`（记上次的摄像头/类别/间隔）**不入库**（同 `bt.json`，各机摄像头索引不同）。
