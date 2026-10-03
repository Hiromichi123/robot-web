/**
 * 任务系统页签（两个）：**任务状态**（执行页） + **任务管理**（列表/JSON/编排）。
 *
 *   任务状态（mission-control-view）—— 只干"执行 + 看情况"：
 *     · 当前加载的任务  · 下发执行 / 停止（急停）（按钮和状态栏都放大，现场远看也清楚）
 *     · 相位/进度/结果  · 狗端等确认时的「继续 / 放弃」
 *   任务管理（mission-mgr-control-view）—— 只管"存、选、改、看"：
 *     · 任务列表（点行选中 / 修改 / 删除 / 新增）
 *     · 「加载到执行页」把选中的任务送去执行页（会**自动切到任务状态页签**）
 *     · JSON 预览（选中任务的文本）+ 复制 / 导出 / 导入
 *     · 子页面：点「新增」或「修改」才进来 —— 步骤编排（调色板/参数就地改/排序复制删除）
 *
 * 三个名字别混：**选中**（任务管理里选中的，决定 JSON 预览和编辑对象）、
 * **加载**（送去执行页的那个，决定"下发执行"发什么）、**运行中**（狗端正在跑的那个，来自 2Hz 状态）。
 *
 * 链路（不变）：
 *   本页 → (rosbridge :9090) → /dog/mission (std_msgs/String, JSON) → dog_node 解析执行
 *          ←                    /dog/mission_status (2Hz 心跳 + 变化时)     ← 进度/等确认
 *          →                    /dog/mission_cmd   (confirm/reject/abort)
 *
 * 几个刻意的取舍：
 *
 * 1. **状态以狗端发的为准**（和雷达建图开关那套同一个设计）。刷新页面/断线重连后，
 *    靠 2Hz 的状态自动对齐，本页不自己记"任务跑到哪了"。状态超过 1.5s 没来就显示
 *    「狗端未上线」并把下发按钮禁掉 —— 否则操作员点了没反应会以为是自己点的姿势不对。
 *
 * 2. **自己建 ROSLIB.Topic，不往 ros-bridge.js 里加东西**。那个文件是全局单例、
 *    被 8 个模块共享（改它要同步 bump 9 处 ?v=，漏一处会加载两份模块实例导致订阅静默失效）。
 *    这里只借它的连接：getRosClient()。写法照 js/ros-debug.js。
 *
 * 3. **本页的校验只是"提前告诉操作员"，权威校验在狗端**（core_2026 的 mission_plan.cpp）。
 *    两边不一致时以狗端为准，状态里会带 rejected + 原因。
 *
 * 两个面板共用一份状态（任务表、选中、加载、ROS 连接），所以是**一个模块**、
 * 三个出口（init / setXMissionViewVisible / setMissionMgrViewVisible），不拆成两个文件。
 */
import { getRosClient, addStatusListener } from './ros-bridge.js?v=980';
import { createMissionMap, parsePcd } from './mission-map.js?v=2';

const MISSION_TOPIC        = '/dog/mission';
const MISSION_CMD_TOPIC    = '/dog/mission_cmd';
const MISSION_STATUS_TOPIC = '/dog/mission_status';

const LS_MISSIONS_KEY  = 'web_sim_missions_v1';           // 已存任务列表（形状没变，向后兼容）
const LS_SELECTED_KEY  = 'web_sim_mission_selected_v1';   // 任务管理里选中的任务名
const LS_LOADED_KEY    = 'web_sim_mission_loaded_v1';     // 任务状态页里加载的任务名
const STATUS_STALE_MS  = 1500;   // 状态多久没来就认为狗端不在

/** 正在跑的相位里，这两个不允许再下发新任务（狗端也会丢，但界面先说清楚） */
const RUNNING_PHASES = ['running', 'waiting_confirm'];

/**
 * 步骤类型表 —— 与狗端 `mission_plan.hpp` 的 StepType 一一对应，字段名就是 JSON 的键。
 * `def` 是新增该步骤时的默认值（照狗端 HAL 的默认值取，点一下就是能跑的）。
 */
const STEP_TYPES = [
    { type: 'check_stand', label: '站立自检',
      fields: [{ k: 'timeout_s', label: '超时', unit: 's', def: 5, step: 1 }] },
    { type: 'getup', label: '起立',
      fields: [{ k: 'settle_s', label: '稳定', unit: 's', def: 10, step: 1 }] },
    { type: 'locomotion', label: '进入 RL', fields: [] },
    { type: 'move', label: '移动',
      fields: [
          { k: 'vx', label: '前后', unit: 'm/s', def: 0.1, step: 0.05 },
          { k: 'vy', label: '左右', unit: 'm/s', def: 0,   step: 0.05 },
          { k: 'wz', label: '转向', unit: 'rad/s', def: 0, step: 0.05 },
          { k: 'duration_s', label: '时长', unit: 's', def: 3, step: 0.5 },
      ] },
    { type: 'goto', label: '到点',
      fields: [
          { k: 'x', label: 'x', unit: 'm', def: 1.0, step: 0.1 },
          { k: 'y', label: 'y', unit: 'm', def: 0.0, step: 0.1 },
          { k: 'yaw', label: '航向', unit: 'rad', def: 0, step: 0.1 },
          { k: 'goto_timeout_s', label: '超时', unit: 's', def: 60, step: 5 },
      ],
      // z 只用于在 3D 视图里把航点标记放到点击的高度上；**不进 JSON**
      // （planOf 只导出 fields，extra 不会出去 —— 狗端的 goto 是 2D 的）
      extra: ['z'] },
    { type: 'wait', label: '等待',
      fields: [{ k: 'duration_s', label: '时长', unit: 's', def: 5, step: 1 }] },
    { type: 'getdown', label: '趴下',
      fields: [{ k: 'settle_s', label: '稳定', unit: 's', def: 1, step: 1 }] },
];

const STEP_BY_TYPE = new Map(STEP_TYPES.map((s) => [s.type, s]));

/** 相位 → 给操作员看的中文 + 颜色档（'ok' | 'warn' | 'bad' | ''） */
const PHASE_TEXT = {
    idle:            ['空闲（等待下发）', ''],
    running:         ['执行中', 'ok'],
    waiting_confirm: ['等待确认', 'warn'],
    done:            ['已完成', 'ok'],
    aborted:         ['已急停中止', 'warn'],
    failed:          ['失败', 'bad'],
    rejected:        ['任务被拒', 'bad'],
};

let els = {};
let rosOnline = false;

let txMission = null;   // ROSLIB.Topic 发布器
let txCmd     = null;
let rxStatus  = null;

/** 已存任务 [{name, confirm, steps, savedAt}] */
let missions = [];
/** 任务管理里选中的任务名（空 = 没选） */
let selectedName = '';
/** 任务状态页里加载的任务名（空 = 没加载） */
let loadedName = '';
/** 任务管理子页面里正在编辑的草稿 {name, confirm, steps}；不在子页面时是 null */
let draft = null;
/** 进编排层时那份任务的原名（'' = 新增）。保存时用它判断"是不是改了已加载那个的名字" */
let draftOrigin = '';
/** 点云地图视图（编辑器里那个）—— 懒建：真用到编辑器时才创建 WebGL 上下文 */
let mapView = null;
/** 当前已装进视图的地图名（避免重复加载同一张） */
let mapLoadedName = '';
/** 地图列表（来自 webserver 的 /api/maps）*/
let mapList = [];
/** 任务管理的两个子视图：'list' | 'editor' */
let mgrView = 'list';

let lastStatus = null;        // 最近一条状态（对象）
let lastStatusAt = 0;         // 本地收到时刻 —— 不信狗端的时间戳
let staleTimer = null;
let lastRunningTag = null;    // 上一拍"哪个任务在跑"，用来决定要不要重画列表

// ── 小工具 ───────────────────────────────────────────────
function nowMs() { return Date.now(); }

function num(v, fallback = 0) {
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
}

function esc(s) {
    return String(s == null ? '' : s)
        .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
}

function defaultParams(type) {
    const def = STEP_BY_TYPE.get(type);
    const p = {};
    for (const f of def ? def.fields : []) p[f.k] = f.def;
    return p;
}

function makeStep(type) { return { type, params: defaultParams(type) }; }

/**
 * 步骤的**归一化**：一律转成编排格式 `{type, params:{...}}`，认不出的类型丢掉。
 *
 * 为什么要它：步骤有两种写法在流通 ——
 *   · 编排格式 `{type:'move', params:{vx:0.1,...}}`（本模块内部、编辑器用）
 *   · JSON 平铺 `{type:'move', vx:0.1,...}`（下发给狗端的、导入文件里的）
 * 早期版本把后者直接存进了 localStorage，读回来就会 `s.params` 是 undefined。
 * 所有外部来源（localStorage / 导入文件 / 粘贴的 JSON）都要先过这一道。
 */
function normalizeSteps(raw) {
    return (raw || []).map((s) => {
        if (!s || !s.type) return null;
        const def = STEP_BY_TYPE.get(s.type);
        if (!def) return null;                    // 不认识的类型：狗端也会拒，别留着
        const params = defaultParams(s.type);     // 先铺一层默认值
        const src = (s.params && typeof s.params === 'object') ? s.params : s;
        for (const f of def.fields) {
            if (src[f.k] != null) params[f.k] = num(src[f.k], f.def);
        }
        // 界面专用的额外字段（如 goto 的 z）：存是要存的，导出 JSON 时不会带上
        for (const k of (def.extra || [])) {
            if (src[k] != null) params[k] = num(src[k], 0);
        }
        return { type: s.type, params };
    }).filter(Boolean);
}

function stepLabel(type) {
    const d = STEP_BY_TYPE.get(type);
    return d ? d.label : type;
}

/** 新任务/模板的默认内容：一条能跑起来的最短序列，操作员改数字即可 */
function templateMission() {
    return {
        name: '巡检A',
        confirm: 'each',
        steps: ['check_stand', 'getup', 'locomotion', 'move', 'getdown'].map(makeStep),
    };
}

/** 把 {name, confirm, steps} 变成下发给狗端的那份 JSON 对象 */
function planOf(m) {
    const plan = {
        version: 1,
        name: String(m.name || '未命名任务').slice(0, 32),
        confirm: m.confirm === 'none' ? 'none' : 'each',
        steps: (m.steps || []).map((s) => {
            const def = STEP_BY_TYPE.get(s.type);
            const out = { type: s.type };
            for (const f of def ? def.fields : []) out[f.k] = num(s.params[f.k], f.def);
            return out;
        }),
    };
    // 这份任务的航点是在哪张点云图上取的（自描述；狗端解析器忽略未知键）
    if (m.map) plan.map = String(m.map).slice(0, 128);
    return plan;
}

function findMission(name) { return missions.find((m) => m.name === name) || null; }
function selectedMission() { return findMission(selectedName); }
function loadedMission() { return findMission(loadedName); }

/**
 * 本地检查 —— 只是**提前提醒**，不是权威。狗端（mission_plan.cpp）才是。
 *
 * 唯一一条值得提醒的是"没进 RL 就要移动"：狗端刻意不按计划文本拒它
 * （上一轮任务可能还站着，那时"只移动"完全合法），而是在执行到那一步时
 * 看狗**当时的真实状态**。所以这里只能提示"除非它已经在 RL 模式"。
 */
function lint(steps) {
    const warns = [];
    let rl = false;
    (steps || []).forEach((s, i) => {
        if (s.type === 'locomotion') { rl = true; return; }
        if ((s.type === 'move' || s.type === 'goto') && !rl) {
            warns.push(`第 ${i + 1} 步是「${stepLabel(s.type)}」，但前面没有「进入 RL」` +
                       `（除非狗已经在 RL 模式，否则这步会被狗端判失败）`);
        }
    });
    return warns;
}

// ── 点云地图 / 航点 ──────────────────────────────────────
/**
 * 懒建地图视图。只有真进编辑器才创建 WebGL 上下文 —— 两个页签都不看的时候
 * 不该白白占一份 GPU 资源。
 */
function ensureMapView() {
    if (mapView || !els.mapCanvas) return mapView;
    mapView = createMissionMap({
        canvas: els.mapCanvas,
        onPick: (x, y, z) => {
            if (!draft) return;
            // 点一下 = 往任务末尾加一个「到点」。yaw 沿用已有 goto 的（多数场景是
            // "朝着目标走"就够），要改就在步骤列表里改那个数字。
            const lastGoto = [...draft.steps].reverse().find((s) => s.type === 'goto');
            const step = makeStep('goto');
            step.params.x = Number(x.toFixed(3));
            step.params.y = Number(y.toFixed(3));
            step.params.yaw = lastGoto ? lastGoto.params.yaw : 0;
            step.params.z = Number(z.toFixed(3));
            draft.steps.push(step);
            renderEditor();
        },
        onHint: (text) => { if (els.mapHint) els.mapHint.textContent = text; },
    });
    return mapView;
}

/** 拉一次地图列表（webserver 的 /api/maps），填进下拉框 */
async function refreshMapList() {
    if (!els.mapSelect) return;
    try {
        const res = await fetch('/api/maps', { cache: 'no-store' });
        const data = await res.json();
        mapList = Array.isArray(data.maps) ? data.maps : [];
    } catch (e) {
        mapList = [];
        if (els.mapState) els.mapState.textContent = '读不到地图列表（webserver 起了吗？）';
        return;
    }
    const cur = draft && draft.map ? draft.map : '';
    els.mapSelect.innerHTML = '<option value="">（不关联地图）</option>' +
        mapList.map((m) => {
            const pts = m.points ? `${Number(m.points).toLocaleString()} 点` : '';
            const when = m.started_at ? String(m.started_at).replace('T', ' ').slice(5) : '';
            const extra = [pts, when].filter(Boolean).join(' · ');
            return `<option value="${esc(m.name)}"${m.name === cur ? ' selected' : ''}>` +
                   `${esc(m.name.replace(/^map_/, '').replace(/\.pcd$/, ''))}` +
                   `${extra ? '（' + esc(extra) + '）' : ''}</option>`;
        }).join('');
    if (els.mapState) {
        els.mapState.textContent = mapList.length
            ? `${mapList.length} 张可选`
            : '板上还没有地图（去「雷达」页签建一张）';
    }
}

/** 把某张 pcd 装进视图（draft.map 也记下来） */
async function loadMapIntoView(name) {
    if (!draft) return;
    const view = ensureMapView();
    if (!view) return;

    draft.map = name || '';
    if (!name) {
        mapLoadedName = '';
        view.clear();
        if (els.mapPlaceholder) els.mapPlaceholder.hidden = false;
        if (els.mapState) els.mapState.textContent = '未关联地图';
        updateWaypointMarkers();       // 没图也把已有航点画出来（z 为 0）
        return;
    }

    if (els.mapPlaceholder) els.mapPlaceholder.hidden = true;
    if (els.mapState) els.mapState.textContent = `加载 ${name} …`;
    try {
        const res = await fetch(`/api/maps/${encodeURIComponent(name)}`, { cache: 'no-store' });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const cloud = parsePcd(await res.arrayBuffer());
        view.setCloud(cloud);
        mapLoadedName = name;
        if (els.mapState) els.mapState.textContent = `${name} · ${cloud.count.toLocaleString()} 点`;
        updateWaypointMarkers();
    } catch (e) {
        mapLoadedName = '';
        view.clear();
        if (els.mapState) els.mapState.textContent = `加载失败：${e.message}`;
    }
}

/** 把当前草稿里的「到点」画成小球（点云上的航点） */
function updateWaypointMarkers() {
    if (!mapView) return;
    const ws = (draft ? draft.steps : [])
        .filter((s) => s.type === 'goto')
        .map((s) => ({ x: num(s.params.x), y: num(s.params.y), z: num(s.params.z, 0) }));
    mapView.setWaypoints(ws);
}

// ── ROS 话题 ─────────────────────────────────────────────
function ensureTopics() {
    const ros = getRosClient();
    if (!ros) return false;
    // 已经建好 → 直接算成功。原来写的是
    //   `if (!ros || txMission || txCmd || rxStatus) return false;`
    // 语义反了：applyDesired() 在**刚连上 ros 时就调过一次** ensureTopics()，
    // 于是操作员点「下发执行」时三个话题早已非空 → 这里返回 false →
    // sendLoaded()/sendCmd() 在 `if (!ensureTopics()) return` 处静默返回，
    // 一个字节都不发。表现就是"点了没反应 / 抓不到任何 WebSocket 帧"
    // （2026-09-30 实机定位；同一个门也把「继续/放弃/急停」三个按钮一起锁死了）。
    if (txMission && txCmd && rxStatus) return true;

    const mk = (name, cb) => {
        const t = new ROSLIB.Topic({
            ros, name, messageType: 'std_msgs/msg/String',
            compression: 'none', throttle_rate: 0, queue_length: 0, queue_size: 1,
        });
        if (cb) t.subscribe(cb);
        return t;
    };
    txMission = mk(MISSION_TOPIC);
    txCmd     = mk(MISSION_CMD_TOPIC);
    rxStatus  = mk(MISSION_STATUS_TOPIC, onStatus);
    try { txMission.advertise(); } catch (e) { console.warn('[mission] advertise 失败:', e); }
    try { txCmd.advertise(); }     catch (e) { console.warn('[mission] advertise 失败:', e); }
    console.info('[mission] 📡 已订阅', MISSION_STATUS_TOPIC);
    return true;
}

function teardownTopics() {
    for (const t of [txMission, txCmd, rxStatus]) {
        if (!t) continue;
        try { t.unsubscribe(); } catch (e) { /* 忽略 */ }
        try { t.unadvertise(); } catch (e) { /* 忽略 */ }
    }
    txMission = txCmd = rxStatus = null;
}

/** 连接状态变了就重建/拆掉话题 —— 每次重连 ros 都是新对象。 */
function applyDesired() {
    if (rosOnline) {
        ensureTopics();
    } else {
        teardownTopics();
        lastStatus = null;
        renderStatus();
    }
}

function publish(topic, text) {
    if (!topic) return false;
    try {
        topic.publish(new ROSLIB.Message({ data: text }));
        return true;
    } catch (e) {
        console.warn('[mission] 发布失败:', e);
        return false;
    }
}

function sendCmd(cmd) {
    if (!ensureTopics()) return;
    publish(txCmd, cmd);
    console.info('[mission] 📤 指令', cmd);
}

/** 下发加载到执行页的那个任务 */
function sendLoaded() {
    const m = loadedMission();
    if (!m) return false;
    if (!ensureTopics()) return false;
    const plan = planOf(m);
    publish(txMission, JSON.stringify(plan));
    if (els.result) els.result.textContent = `已下发「${plan.name}」，等狗端回应…`;
    console.info('[mission] 📤 下发任务', plan.name, plan.steps.length, '步');
    return true;
}

function onStatus(msg) {
    lastStatusAt = nowMs();
    try {
        lastStatus = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (e) {
        console.warn('[mission] 状态不是合法 JSON:', msg && msg.data);
        return;
    }
    renderStatus();
}

/** 切到「任务状态」页签 —— 借用 main.js 已有的页签点击处理，不反向依赖它 */
function goToExecTab() {
    const btn = document.querySelector('#mission-tab');
    if (btn && typeof btn.click === 'function') btn.click();
}

// ── 渲染：任务状态（执行页）───────────────────────────────
function setConnState() {
    if (!els.connState) return;
    const fresh = lastStatus && (nowMs() - lastStatusAt) < STATUS_STALE_MS;
    if (!rosOnline) {
        els.connState.textContent = '未连接 ROS';
        els.connState.classList.remove('is-live');
    } else if (!fresh) {
        els.connState.textContent = '狗端未上线（等 dog_node）';
        els.connState.classList.remove('is-live');
    } else {
        els.connState.textContent = '狗端在线';
        els.connState.classList.add('is-live');
    }
}

function renderStatus() {
    const st = lastStatus;

    // 加载信息
    if (els.loadedInfo) {
        const m = loadedMission();
        if (m) {
            els.loadedInfo.textContent =
                `已加载：「${m.name}」（${m.steps.length} 步 · 确认=${m.confirm || 'each'}）`;
        } else if (loadedName) {
            els.loadedInfo.textContent = `已加载的「${loadedName}」找不到了（可能被删了）—— 去任务管理重新加载`;
        } else {
            els.loadedInfo.textContent = '未加载任务 —— 去「任务管理」页签选一个，点「加载到执行页」';
        }
    }

    // 大状态栏：相位 + 第几步 + 用时
    if (els.phase) {
        let text = '—', tone = '';
        if (st) {
            const [t, tn] = PHASE_TEXT[st.phase] || [st.phase || '—', ''];
            text = t; tone = tn;
            if (st.phase === 'running' || st.phase === 'waiting_confirm') {
                const nm = st.name ? `「${st.name}」` : '';
                text = `${t}\n${nm}第 ${st.index}/${st.total} 步 · ${stepLabel(st.type)}` +
                       ` · 已 ${num(st.elapsed_s).toFixed(1)}s`;
            }
        }
        els.phase.textContent = text;
        els.phase.hidden = !st;
        els.phase.classList.toggle('is-live', tone === 'ok');
        els.phase.classList.toggle('is-warn', tone === 'warn');
        els.phase.classList.toggle('is-bad', tone === 'bad');
    }

    if (els.progress) {
        let text = '尚未执行。';
        if (st && (st.phase === 'running' || st.phase === 'waiting_confirm')) {
            text = `正在跑第 ${st.index}/${st.total} 步：${stepLabel(st.type)}`;
        } else if (st && st.phase !== 'idle') {
            text = st.msg || text;
        } else if (st) {
            text = st.msg || '等待下发任务';
        }
        els.progress.textContent = text;
        els.progress.hidden = !st;
    }

    // 等确认：大提示 + 两个大按钮
    const waiting = !!st && st.phase === 'waiting_confirm';
    if (els.gate) els.gate.hidden = !waiting;
    if (els.gateMsg) {
        els.gateMsg.hidden = !waiting;
        if (waiting) els.gateMsg.textContent = st.prompt || '狗端暂停，等待确认';
    }

    if (els.result) {
        if (st && (st.phase === 'done' || st.phase === 'failed' ||
                   st.phase === 'aborted' || st.phase === 'rejected')) {
            els.result.textContent = st.msg || '';
            els.result.style.color = (st.phase === 'failed' || st.phase === 'rejected')
                ? 'var(--c-danger, #e07070)' : '';
        } else {
            els.result.textContent = '';
        }
    }

    setConnState();

    // 按钮可用性
    const running = !!st && RUNNING_PHASES.includes(st.phase);
    const hasLoaded = !!loadedMission();
    if (els.btnSend) {
        els.btnSend.disabled = !rosOnline || !hasLoaded || running;
        els.btnSend.title = !hasLoaded ? '先到「任务管理」里选一个任务并点「加载到执行页」'
                           : running ? '当前任务还在跑，等它结束或先急停'
                           : '把加载的任务发给狗端执行';
    }
    if (els.btnAbort) els.btnAbort.disabled = !rosOnline;   // 急停永不禁用（在线时）

    // 列表里那个"执行中"标记变了才重画（否则 2Hz 重画会打断点击）
    const tag = running ? (st.name || '') : '';
    if (tag !== lastRunningTag) {
        lastRunningTag = tag;
        renderSaved();
    }
}

// ── 渲染：任务管理（列表 + JSON）──────────────────────────
function renderSaved() {
    if (!els.savedList) return;
    if (els.savedCount) els.savedCount.textContent = `${missions.length} 个`;

    const running = !!lastStatus && RUNNING_PHASES.includes(lastStatus.phase);
    if (missions.length === 0) {
        els.savedList.innerHTML =
            '<li class="maps-empty">还没有任务。点下面的「+ 新增任务」建一个（会自动带上全套模板）。</li>';
    } else {
        els.savedList.innerHTML = missions.map((m, i) => {
            const isSel = m.name === selectedName;
            const isLoaded = m.name === loadedName;
            const isRunning = running && lastStatus.name === m.name;
            const tags = [isLoaded ? '已加载' : '', isRunning ? '执行中' : ''].filter(Boolean).join(' · ');
            return `
            <li class="maps-item ${isSel ? 'is-loaded' : ''}" data-idx="${i}" title="点这一行选中它">
              <div class="maps-info">
                <span class="maps-name">${esc(m.name)}${tags ? ` · ${tags}` : ''}</span>
                <span class="maps-meta">${(m.steps || []).length} 步 · 确认=${esc(m.confirm || 'each')}
                  ${m.savedAt ? ' · ' + new Date(m.savedAt).toLocaleString('zh-CN', { hour12: false }) : ''}</span>
              </div>
              <div class="maps-acts">
                <button type="button" class="lidar-btn" data-act="edit" data-idx="${i}">修改</button>
                <button type="button" class="lidar-btn maps-del" data-act="del" data-idx="${i}">删除</button>
              </div>
            </li>`;
        }).join('');
    }
    renderSelectionInfo();
}

function renderSelectionInfo() {
    const m = selectedMission();
    if (els.selected) {
        els.selected.textContent = m
            ? `当前选中：「${m.name}」（${(m.steps || []).length} 步 · 确认=${m.confirm || 'each'}）`
            : '当前选中：无 —— 点上面一行选中，或「+ 新增任务」';
    }
    if (els.btnLoad) {
        els.btnLoad.disabled = !m;
        els.btnLoad.title = m ? `把「${m.name}」送到任务状态页，之后在那边点「下发执行」`
                              : '先在列表里选中一个任务';
    }
    // JSON 预览跟选中任务走
    if (els.json) {
        els.json.value = m ? JSON.stringify(planOf(m), null, 2) : '';
        els.json.placeholder = m ? '' : '（先在列表里选中一个任务）';
    }
    for (const b of [els.btnCopy, els.btnExport]) if (b) b.disabled = !m;
}

// ── 渲染：任务管理子页面（步骤编排）───────────────────────
function renderEditor() {
    if (!draft) return;
    if (els.editorTitle) els.editorTitle.textContent = `编排：${draft.name || '（未命名）'}`;
    if (els.name) els.name.value = draft.name;
    if (els.confirm) els.confirm.value = draft.confirm === 'none' ? 'none' : 'each';

    if (els.stepList) {
        if (draft.steps.length === 0) {
            els.stepList.innerHTML = '<li class="maps-empty">还没有步骤。点下面的按钮加一步。</li>';
        } else {
            els.stepList.innerHTML = draft.steps.map((s, i) => {
                const def = STEP_BY_TYPE.get(s.type) || { fields: [] };
                const inputs = def.fields.map((f) => `
                    <label class="mission-param" title="${esc(f.label)}（${esc(f.unit)}）">
                      ${esc(f.label)}
                      <input type="number" class="ros-url-input mission-param-input"
                             style="width:62px" step="${f.step}" data-idx="${i}" data-key="${esc(f.k)}"
                             value="${esc(s.params[f.k])}" />
                    </label>`).join('');
                return `
                <li class="maps-item" data-idx="${i}">
                  <div class="maps-info">
                    <span class="maps-name">${i + 1}. ${esc(stepLabel(s.type))}</span>
                    <span class="maps-meta">${esc(s.type)}</span>
                    <div class="mission-params">${inputs}</div>
                  </div>
                  <div class="maps-acts">
                    <button type="button" class="lidar-btn" data-act="up"   data-idx="${i}" ${i === 0 ? 'disabled' : ''}>↑</button>
                    <button type="button" class="lidar-btn" data-act="down" data-idx="${i}" ${i === draft.steps.length - 1 ? 'disabled' : ''}>↓</button>
                    <button type="button" class="lidar-btn" data-act="dup"  data-idx="${i}">复制</button>
                    <button type="button" class="lidar-btn maps-del" data-act="del" data-idx="${i}">删除</button>
                  </div>
                </li>`;
            }).join('');
        }
        if (els.stepCount) els.stepCount.textContent = `${draft.steps.length} 步`;
    }

    updateWaypointMarkers();

    const warns = lint(draft.steps);
    if (els.lint) {
        els.lint.textContent = warns.length ? '⚠ ' + warns.join('；') : '';
        els.lint.style.color = warns.length ? '#e0b070' : '';
    }
}

function showMgrView(which) {
    mgrView = which;
    if (els.mgrListView) els.mgrListView.hidden = which !== 'list';
    if (els.editorView) els.editorView.hidden = which !== 'editor';
    if (which === 'editor') {
        renderEditor();
        // 画布在被隐藏时尺寸是 0，显示出来必须重新对齐（Three.js 不会自己知道）
        ensureMapView();
        mapView?.resize();
        refreshMapList();
        // 这份任务原来关联过地图就自动装回来
        const want = draft && draft.map ? draft.map : '';
        if (want !== mapLoadedName) loadMapIntoView(want);
        else updateWaypointMarkers();
    } else { renderSaved(); }
    const panelEl = document.querySelector('#control-panel');
    if (panelEl) panelEl.scrollTop = 0;   // 两层内容长度差很多，留着旧位置会看着像没切换
}

// ── 存档 ─────────────────────────────────────────────────
function loadMissions() {
    try {
        const raw = JSON.parse(localStorage.getItem(LS_MISSIONS_KEY) || '[]');
        missions = Array.isArray(raw)
            ? raw.filter((m) => m && m.name)
                 .map((m) => ({ ...m, steps: normalizeSteps(m.steps) }))
            : [];
    } catch (e) {
        missions = [];
    }
    try {
        selectedName = localStorage.getItem(LS_SELECTED_KEY) || '';
        loadedName   = localStorage.getItem(LS_LOADED_KEY) || '';
    } catch (e) {
        selectedName = loadedName = '';
    }
    // 名字对不上就地修掉（任务被删/被改名之后）
    if (!findMission(selectedName)) selectedName = missions.length ? missions[0].name : '';
    if (loadedName && !findMission(loadedName)) loadedName = '';
}

function persistMissions() {
    try {
        localStorage.setItem(LS_MISSIONS_KEY, JSON.stringify(missions));
        localStorage.setItem(LS_SELECTED_KEY, selectedName);
        localStorage.setItem(LS_LOADED_KEY, loadedName);
    } catch (e) { /* 隐私模式等写不进就只当次有效 */ }
}

/** 保存草稿到列表（同名覆盖），并选中它。返回是否成功 */
function saveDraft() {
    if (!draft) return false;
    const name = String(draft.name || '').trim();
    if (!name) { alert('任务名不能为空'); return false; }
    if ((draft.steps || []).length === 0) { alert('任务里至少要有一个步骤'); return false; }

    const entry = {
        name,
        confirm: draft.confirm,
        map: draft.map || '',            // 关联的点云地图（只给界面用，狗端不认）
        // 存**编排格式**（与 draft/编辑器一致）。别存 planOf() 的结果 ——
        // 那是给狗端的平铺格式，存回去下次读出来 `s.params` 就是 undefined（踩过）。
        steps: normalizeSteps(draft.steps),
        savedAt: nowMs(),
    };
    const at = missions.findIndex((m) => m.name === name);
    if (at >= 0) missions[at] = entry; else missions.push(entry);
    selectedName = name;
    // 改写的是**已加载**那个任务的名字时，加载指针跟着走 ——
    // 否则执行页会指向一个已经不存在的名字（提示"找不到"，得手动重新加载）。
    if (loadedName && draftOrigin && loadedName === draftOrigin) loadedName = name;
    persistMissions();
    return true;
}

/** 起个不重名的名字（新增任务时用，避免一进去就覆盖别人） */
function uniqueName(base) {
    if (!findMission(base)) return base;
    for (let i = 2; i < 100; ++i) {
        const n = `${base}${i}`;
        if (!findMission(n)) return n;
    }
    return `${base}_${nowMs()}`;
}

// ── 对外 ─────────────────────────────────────────────────
/** 任务状态页签（执行页）可见性 */
export function setMissionViewVisible(visible) {
    if (!visible) return;
    renderStatus();
}

/** 任务管理页签可见性 */
export function setMissionMgrViewVisible(visible) {
    if (!visible) return;
    loadMissions();          // 切回来重读一次：可能被别的标签页/上一次编辑改过
    showMgrView(mgrView);
    renderSelectionInfo();
}

export function initMissionView() {
    els = {
        // 任务状态（执行页）
        execPanel:   document.querySelector('#mission-control-view'),
        connState:   document.querySelector('#mission-conn-state'),
        loadedInfo:  document.querySelector('#mission-loaded'),
        btnSend:     document.querySelector('#mission-btn-send'),
        btnAbort:    document.querySelector('#mission-btn-abort'),
        gate:        document.querySelector('#mission-gate'),
        gateMsg:     document.querySelector('#mission-gate-msg'),
        btnContinue: document.querySelector('#mission-btn-continue'),
        btnGiveup:   document.querySelector('#mission-btn-giveup'),
        phase:       document.querySelector('#mission-phase'),
        progress:    document.querySelector('#mission-progress'),
        result:      document.querySelector('#mission-result'),

        // 任务管理（列表 + JSON + 编排）
        mgrPanel:    document.querySelector('#mission-mgr-control-view'),
        mgrListView: document.querySelector('#mission-mgr-list-view'),
        editorView:  document.querySelector('#mission-editor-view'),
        savedList:   document.querySelector('#mission-saved-list'),
        savedCount:  document.querySelector('#mission-saved-count'),
        selected:    document.querySelector('#mission-selected'),
        btnNew:      document.querySelector('#mission-btn-new'),
        btnLoad:     document.querySelector('#mission-btn-load'),
        json:        document.querySelector('#mission-json'),
        btnCopy:     document.querySelector('#mission-btn-copy'),
        btnExport:   document.querySelector('#mission-btn-export'),
        btnImport:   document.querySelector('#mission-btn-import'),
        fileInput:   document.querySelector('#mission-file-input'),

        // 子页面：步骤编排
        editorTitle: document.querySelector('#mission-editor-title'),
        name:        document.querySelector('#mission-name'),
        confirm:     document.querySelector('#mission-confirm'),
        stepList:    document.querySelector('#mission-step-list'),
        stepCount:   document.querySelector('#mission-step-count'),
        palette:     document.querySelector('#mission-palette'),
        btnTemplate: document.querySelector('#mission-btn-template'),
        btnClear:    document.querySelector('#mission-btn-clear'),
        lint:        document.querySelector('#mission-lint'),
        btnSave:     document.querySelector('#mission-btn-save'),
        btnCancel:   document.querySelector('#mission-btn-cancel'),

        // 子页面：点云地图与航点
        mapSelect:      document.querySelector('#mission-map-select'),
        mapReload:      document.querySelector('#mission-map-reload'),
        mapCanvas:      document.querySelector('#mission-map-canvas'),
        mapPlaceholder: document.querySelector('#mission-map-placeholder'),
        mapState:       document.querySelector('#mission-map-state'),
        mapHint:        document.querySelector('#mission-map-hint'),
    };
    // 两个面板都没在（例如 mobile.html 没同步改）就直接退出，不抛异常
    if (!els.execPanel && !els.mgrPanel) return;

    loadMissions();
    showMgrView('list');
    renderStatus();

    // ── 任务管理：列表选择 / 修改 / 删除 / 新增 / 加载 ────
    els.savedList?.addEventListener('click', (ev) => {
        const btn = ev.target.closest('button[data-act]');
        const row = ev.target.closest('li[data-idx]');
        if (!row) return;
        const i = Number(row.dataset.idx);
        if (!Number.isInteger(i) || i < 0 || i >= missions.length) return;

        if (!btn) {                       // 点行 = 选中
            selectedName = missions[i].name;
            persistMissions();
            renderSaved();
            return;
        }
        const act = btn.dataset.act;
        if (act === 'edit') {
            draft = {
                name: missions[i].name,
                confirm: missions[i].confirm === 'none' ? 'none' : 'each',
                steps: normalizeSteps(missions[i].steps),   // 顺带深拷贝，改草稿不动原任务
                map: missions[i].map || '',
            };
            draftOrigin = missions[i].name;
            showMgrView('editor');
        } else if (act === 'del') {
            const nm = missions[i].name;
            if (!confirm(`删除任务「${nm}」？`)) return;
            missions.splice(i, 1);
            if (selectedName === nm) selectedName = missions.length ? missions[0].name : '';
            if (loadedName === nm) loadedName = '';       // 执行页那个也得撤掉，别指向不存在的
            persistMissions();
            renderSaved();
            renderStatus();
        }
    });

    els.btnNew?.addEventListener('click', () => {
        draft = templateMission();
        draft.name = uniqueName(draft.name);
        draftOrigin = '';                 // 新增：没有"原名"
        draft.map = '';
        showMgrView('editor');
    });

    els.btnLoad?.addEventListener('click', () => {
        if (!selectedMission()) return;
        loadedName = selectedName;
        persistMissions();
        renderStatus();
        goToExecTab();                  // 加载完直接把人送到执行页
    });

    // ── 任务状态：执行 / 急停 / 确认 ─────────────────────
    els.btnSend?.addEventListener('click', sendLoaded);
    els.btnAbort?.addEventListener('click', () => sendCmd('abort'));
    els.btnContinue?.addEventListener('click', () => sendCmd('confirm'));
    els.btnGiveup?.addEventListener('click', () => sendCmd('reject'));

    // ── 任务管理：JSON 预览的复制/导出/导入 ──────────────
    els.btnCopy?.addEventListener('click', async () => {
        if (!els.json || !els.json.value) return;
        try {
            await navigator.clipboard.writeText(els.json.value);
        } catch (e) {
            els.json.select();   // 剪贴板不可用（非 https 等）就退化成选中，让人自己 Ctrl-C
        }
    });
    els.btnExport?.addEventListener('click', () => {
        const m = selectedMission();
        if (!m) return;
        const plan = planOf(m);
        const blob = new Blob([JSON.stringify(plan, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${plan.name || 'mission'}.json`;
        a.click();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
    els.btnImport?.addEventListener('click', () => els.fileInput?.click());
    els.fileInput?.addEventListener('change', () => {
        const f = els.fileInput.files && els.fileInput.files[0];
        if (!f) return;
        const reader = new FileReader();
        reader.onload = () => {
            try {
                const parsed = JSON.parse(String(reader.result));
                const steps = normalizeSteps(parsed.steps);
                if (steps.length === 0) throw new Error('文件里没有能识别的步骤');
                const name = String(parsed.name || f.name.replace(/\.json$/i, '') || '导入的任务');
                const entry = {
                    name, confirm: parsed.confirm === 'none' ? 'none' : 'each',
                    steps, savedAt: nowMs(),
                };
                const at = missions.findIndex((m) => m.name === name);
                if (at >= 0) missions[at] = entry; else missions.push(entry);
                selectedName = name;
                persistMissions();
                renderSaved();
            } catch (e) {
                alert(`导入失败：${e.message}`);
            }
        };
        reader.readAsText(f);
        els.fileInput.value = '';   // 允许重复导入同一个文件
    });

    // ── 子页面：任务名 / 确认方式 / 步骤编辑 ─────────────
    els.name?.addEventListener('input', () => {
        if (!draft) return;
        draft.name = els.name.value;
        if (els.editorTitle) els.editorTitle.textContent = `编排：${draft.name || '（未命名）'}`;
    });
    els.confirm?.addEventListener('change', () => {
        if (!draft) return;
        draft.confirm = els.confirm.value === 'none' ? 'none' : 'each';
    });
    els.palette?.addEventListener('click', (ev) => {
        const btn = ev.target.closest('button[data-add]');
        if (!btn || !draft) return;
        draft.steps.push(makeStep(btn.dataset.add));
        renderEditor();
    });
    els.btnTemplate?.addEventListener('click', () => {
        if (!draft) return;
        draft.steps = templateMission().steps;
        renderEditor();
    });
    els.btnClear?.addEventListener('click', () => {
        if (!draft) return;
        draft.steps = [];
        renderEditor();
    });
    els.stepList?.addEventListener('click', (ev) => {
        const btn = ev.target.closest('button[data-act]');
        if (!btn || !draft) return;
        const i = Number(btn.dataset.idx);
        if (!Number.isInteger(i) || i < 0 || i >= draft.steps.length) return;
        const act = btn.dataset.act;
        if (act === 'up' && i > 0) {
            [draft.steps[i - 1], draft.steps[i]] = [draft.steps[i], draft.steps[i - 1]];
        } else if (act === 'down' && i < draft.steps.length - 1) {
            [draft.steps[i + 1], draft.steps[i]] = [draft.steps[i], draft.steps[i + 1]];
        } else if (act === 'dup') {
            draft.steps.splice(i + 1, 0, { type: draft.steps[i].type, params: { ...draft.steps[i].params } });
        } else if (act === 'del') {
            draft.steps.splice(i, 1);
        } else {
            return;
        }
        renderEditor();
    });
    // 参数输入：改一个数字就地更新（不重排列表，否则输入框会失焦）
    els.stepList?.addEventListener('input', (ev) => {
        const inp = ev.target.closest('input[data-key]');
        if (!inp || !draft) return;
        const i = Number(inp.dataset.idx);
        if (!Number.isInteger(i) || i < 0 || i >= draft.steps.length) return;
        draft.steps[i].params[inp.dataset.key] = inp.value;
        updateWaypointMarkers();
    });

    // ── 子页面：点云地图 ─────────────────────────────────
    els.mapSelect?.addEventListener('change', () => {
        loadMapIntoView(els.mapSelect.value);
    });
    els.mapReload?.addEventListener('click', () => refreshMapList());

    // ── 子页面出口：保存 / 取消 ──────────────────────────
    els.btnSave?.addEventListener('click', () => {
        if (!saveDraft()) return;
        showMgrView('list');
        // 执行页显示的是"已加载那个任务"的名字/步数；保存（尤其改了名）之后要**立刻**跟上，
        // 否则要等下一次 2Hz 心跳才刷新（最多 0.5s 的空窗，看着像没生效）。
        renderStatus();
    });
    els.btnCancel?.addEventListener('click', () => showMgrView('list'));

    // ── 连接状态 ─────────────────────────────────────────
    addStatusListener((s) => {
        const on = s.connectionStatus === 'online';
        if (on !== rosOnline) {
            rosOnline = on;
            applyDesired();
        }
        setConnState();
    });
    staleTimer = setInterval(() => {
        setConnState();
    }, 500);
}

// 模块卸载（页面级单例，实际只在热重载/关页时发生）：清掉定时器
window.addEventListener('beforeunload', () => {
    if (staleTimer) { clearInterval(staleTimer); staleTimer = null; }
    teardownTopics();
});
