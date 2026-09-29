/**
 * ROS 话题调试工具。
 *
 * 面板「话题调试」页签,复用 ros-bridge.js 的 rosbridge WebSocket 连接,提供三块功能:
 *   1. 话题列表:通过 rosapi (/rosapi/topics) 拉取 rosbridge 上的全部话题与类型,
 *      点击某话题自动填入接收区/发送区的输入框。
 *   2. 接收区:对任意话题建立临时订阅,日志形式展示收到的消息(带时间戳),
 *      支持暂停/清空;高频率话题下按 ~8fps 批量刷新 DOM,最多保留 200 行。
 *   3. 发送区:对任意话题 advertise + publish。消息体为 JSON 对象文本
 *      (如 {"data": "hello"} 或 {"linear": {...}, "angular": {...}}),
 *      话题/类型变化时自动切换发布器,支持「停止发布」。
 *
 * ROS 断开时自动退订/停止发布并清理 UI。
 */
import { ROSLIB } from './roslib-shim.js';
import { getRosClient, addStatusListener, connectionStatus } from './ros-bridge.js?v=980';

// ── DOM 引用(initRosDebug 时填充) ─────────────────────────
let topicsRefreshBtn = null;  // 刷新话题按钮
let debugHint = null;         // 提示文字
let rxTopicInput = null;      // 接收话题名
let rxTypeInput = null;       // 接收消息类型
let rxToggleBtn = null;       // 订阅/退订按钮
let rxPauseInput = null;      // 暂停复选框
let rxClearBtn = null;        // 清空按钮
let rxLog = null;             // 日志容器
let rxCountOutput = null;     // 收到条数
let txTopicInput = null;      // 发送话题名
let txTypeInput = null;       // 发送消息类型
let txPayloadInput = null;    // JSON 消息体
let txSendBtn = null;         // 发布一次
let txStopBtn = null;         // 停止发布
let txStatusOutput = null;    // 发送状态

// ── 接收状态 ───────────────────────────────────────────────
let rxTopic = null;           // 当前 ROSLIB.Topic 订阅实例(null=未订阅)
let rxCount = 0;              // 累计收到条数
let rxPaused = false;         // 暂停时仍计数但不显示新消息
let rxBuffer = [];            // 待渲染缓冲 [{ time, text }]
let rxFlushTimer = null;      // DOM 批量刷新定时器

// ── 发送状态 ───────────────────────────────────────────────
let txTopic = null;           // 当前 ROSLIB.Topic 发布实例(null=未发布)
let txName = '';              // 已 advertise 的话题名
let txType = '';              // 已 advertise 的消息类型
let txSendCount = 0;          // 已发送次数

const RX_LOG_MAX_LINES = 200;  // 日志最多保留行数
const RX_FLUSH_MS = 120;       // 日志 DOM 刷新间隔(~8fps)
const RX_TEXT_MAX = 800;       // 单条消息显示的最大字符数

/** 当前时间字符串 HH:MM:SS.mmm */
function nowStamp() {
  const d = new Date();
  const t = d.toLocaleTimeString('zh-CN', { hour12: false });
  return `${t}.${String(d.getMilliseconds()).padStart(3, '0')}`;
}

/** 设置列表区提示文字(不覆盖点击话题的默认说明) */
function setHint(text) {
  if (debugHint) debugHint.textContent = text;
}

/** 设置发送状态文字 */
function setTxStatus(text) {
  if (txStatusOutput) txStatusOutput.value = text;
}

// ── 话题下拉(接收/发送区共用):rosapi 话题列表 + 各自历史(localStorage)融合 ──
const RX_HISTORY_KEY = 'web_sim_ros_rx_history_v1';
const TX_HISTORY_KEY = 'web_sim_ros_tx_history_v1';
const TOPIC_HISTORY_MAX = 15;
const HISTORY_LABEL = { rx: '订阅历史', tx: '发布历史' };
let rxDdPanel = null;         // 接收话题下拉面板
let txDdPanel = null;         // 发送话题下拉面板
let _topicInfos = [];         // rosapi 话题缓存 [{name, type}]
const _topicHistories = {
  rx: loadTopicHistory(RX_HISTORY_KEY),
  tx: loadTopicHistory(TX_HISTORY_KEY),
};

function loadTopicHistory(key) {
  try { return JSON.parse(localStorage.getItem(key)) || []; } catch { return []; }
}
function saveTopicHistory(side) {
  const key = side === 'rx' ? RX_HISTORY_KEY : TX_HISTORY_KEY;
  try { localStorage.setItem(key, JSON.stringify(_topicHistories[side])); } catch { /* 忽略 */ }
}
function addTopicHistory(side, name, type) {
  const list = _topicHistories[side];
  _topicHistories[side] = [{ name, type }, ...list.filter((h) => h.name !== name)].slice(0, TOPIC_HISTORY_MAX);
  saveTopicHistory(side);
}
function hideTopicDd(panel) {
  if (panel) { panel.hidden = true; panel.innerHTML = ''; }
}
/** 渲染话题下拉:历史 + 可用话题(按输入过滤,历史去重在前);接收/发送区共用 */
function renderTopicDd(side, panel, filter = '') {
  if (!panel) return;
  const q = filter.trim().toLowerCase();
  const hist = _topicHistories[side].filter((h) => !q || h.name.toLowerCase().includes(q));
  const seen = new Set(hist.map((h) => h.name));
  const avail = _topicInfos.filter((t) => (!q || t.name.toLowerCase().includes(q)) && !seen.has(t.name));
  panel.innerHTML = '';
  if (!hist.length && !avail.length) {
    const empty = document.createElement('div');
    empty.className = 'ros-debug-dd-empty';
    empty.textContent = q ? '无匹配话题' : '暂无话题(连接后自动获取)';
    panel.appendChild(empty);
    panel.hidden = false;
    return;
  }
  const frag = document.createDocumentFragment();
  const addSection = (label, items) => {
    if (!items.length) return;
    const head = document.createElement('div');
    head.className = 'ros-debug-dd-group';
    head.textContent = label;
    frag.appendChild(head);
    for (const it of items) {
      const item = document.createElement('div');
      item.className = 'ros-debug-dd-item';
      item.dataset.name = it.name;
      item.dataset.type = it.type || '';
      const n = document.createElement('span');
      n.className = 'dd-name';
      n.textContent = it.name;
      const t = document.createElement('span');
      t.className = 'dd-type';
      t.textContent = it.type || '';
      item.append(n, t);
      frag.appendChild(item);
    }
  };
  addSection(HISTORY_LABEL[side], hist);
  addSection('可用话题', avail);
  panel.appendChild(frag);
  panel.hidden = false;
}
/** 给话题输入框接上融合下拉:focus/输入展开,选中填充话题+类型,Esc/失焦关闭 */
function wireTopicDropdown(side, input, typeInput, panel, clearBtn) {
  if (!input || !panel) return;
  const show = () => { if (!input.disabled) renderTopicDd(side, panel, input.value); };
  input.addEventListener('focus', show);
  input.addEventListener('input', show);
  input.addEventListener('blur', () => setTimeout(() => hideTopicDd(panel), 120));
  input.addEventListener('keydown', (e) => { if (e.key === 'Escape') hideTopicDd(panel); });
  clearBtn?.addEventListener('click', () => {
    if (input.disabled) return;
    input.value = '';
    hideTopicDd(panel);
    input.focus();
  });
  panel.addEventListener('mousedown', (e) => {
    const item = e.target.closest('.ros-debug-dd-item');
    if (!item) return;
    e.preventDefault();
    input.value = item.dataset.name || '';
    if (typeInput && !typeInput.disabled && item.dataset.type) typeInput.value = item.dataset.type;
    hideTopicDd(panel);
  });
}

// ── 1. 话题数据(供接收区下拉;独立的话题列表区块已删除) ────

/** 拉取 rosbridge 话题列表,仅更新下拉数据缓存(_topicInfos),提示文字反馈进度 */
function refreshTopicList() {
  const ros = getRosClient();
  if (!ros || connectionStatus !== 'online') {
    setHint('未连接 ROS — 连接后可获取话题。');
    return;
  }
  setHint('正在获取话题列表…');
  try {
    ros.getTopics((info) => {
      const topics = (info && info.topics) || [];
      const types = (info && info.types) || [];
      _topicInfos = topics.map((name, i) => ({ name, type: types[i] || '' }));
      setHint(topics.length
        ? `已获取 ${topics.length} 个话题 — 在接收区话题输入框下拉中选择,或手动输入。`
        : 'rosbridge 上暂无话题(可能有节点未发布)。可手动输入话题名。');
    }, (err) => {
      setHint(`获取话题列表失败:${err?.message || err}`);
    });
  } catch (e) {
    setHint(`获取话题列表失败:${e?.message || e}`);
  }
}

// ── 2. 接收区 ──────────────────────────────────────────────

/** 订阅消息回调:计数 + 入渲染缓冲(暂停时只计数) */
function onRxMessage(msg) {
  rxCount += 1;
  if (rxPaused) return;
  let text;
  try { text = JSON.stringify(msg); } catch { text = String(msg); }
  if (text.length > RX_TEXT_MAX) {
    text = `${text.slice(0, RX_TEXT_MAX)} …(+${text.length - RX_TEXT_MAX} 字符)`;
  }
  rxBuffer.push({ time: nowStamp(), text });
  if (rxBuffer.length > RX_LOG_MAX_LINES) {
    rxBuffer.splice(0, rxBuffer.length - RX_LOG_MAX_LINES);
  }
}

/** 批量把缓冲写入日志 DOM(~8fps),高频率话题不至于卡死主线程 */
function flushRxLog() {
  if (!rxLog) return;
  if (rxBuffer.length) {
    const frag = document.createDocumentFragment();
    for (const item of rxBuffer.splice(0)) {
      const line = document.createElement('div');
      line.className = 'log-line';
      const time = document.createElement('span');
      time.className = 'log-time';
      time.textContent = item.time;
      const body = document.createElement('span');
      body.textContent = item.text;
      line.append(time, body);
      frag.appendChild(line);
    }
    rxLog.appendChild(frag);
    while (rxLog.children.length > RX_LOG_MAX_LINES) {
      rxLog.removeChild(rxLog.firstChild);
    }
    rxLog.scrollTop = rxLog.scrollHeight;
  }
  if (rxCountOutput) rxCountOutput.value = `${rxCount} 条`;
}

/** 清空接收日志与计数 */
function clearRxLog() {
  rxBuffer.length = 0;
  rxCount = 0;
  if (rxLog) rxLog.innerHTML = '';
  if (rxCountOutput) rxCountOutput.value = '0 条';
}

/** 订阅/退订切换 */
function toggleSubscribe() {
  if (rxTopic) {
    unsubscribeRx();
    return;
  }
  const ros = getRosClient();
  if (!ros) { setHint('请先连接 ROS 再订阅。'); return; }
  const name = rxTopicInput?.value.trim() || '';
  const type = rxTypeInput?.value.trim() || '';
  if (!name) { setHint('接收区:请输入话题名。'); return; }
  if (!type) { setHint('接收区:请输入消息类型(如 std_msgs/msg/String)。'); return; }
  rxTopic = new ROSLIB.Topic({
    ros, name, messageType: type,
    compression: 'none', throttle_rate: 0, queue_length: 0, queue_size: 1,
  });
  rxCount = 0;
  if (rxLog) rxLog.innerHTML = '';
  rxTopic.subscribe(onRxMessage);
  addTopicHistory('rx', name, type);
  if (rxToggleBtn) rxToggleBtn.textContent = '退订';
  if (rxTopicInput) rxTopicInput.disabled = true;
  if (rxTypeInput) rxTypeInput.disabled = true;
  setHint(`正在订阅 ${name}(${type}),等待消息…`);
}

/** 退订接收话题 */
function unsubscribeRx() {
  if (rxTopic) {
    try { rxTopic.unsubscribe(); } catch { /* 忽略 */ }
    rxTopic = null;
    setHint('已退订,可重新选择话题订阅。');
  }
  if (rxToggleBtn) rxToggleBtn.textContent = '订阅';
  if (rxTopicInput) rxTopicInput.disabled = false;
  if (rxTypeInput) rxTypeInput.disabled = false;
}

// ── 3. 发送区 ──────────────────────────────────────────────

/** 确保(话题名, 类型)对应的发布器已 advertise;变化时自动切换 */
function ensureAdvertised(name, type, ros) {
  if (txTopic && (txName !== name || txType !== type)) {
    try { txTopic.unadvertise(); } catch { /* 忽略 */ }
    txTopic = null;
  }
  if (!txTopic) {
    txTopic = new ROSLIB.Topic({
      ros, name, messageType: type,
      compression: 'none', queue_size: 1,
    });
    txTopic.advertise();
    txName = name;
    txType = type;
    txSendCount = 0;
  }
}

/** 发布一次当前填写的消息 */
function publishOnce() {
  const ros = getRosClient();
  if (!ros) { setTxStatus('ROS 未连接'); return; }
  const name = txTopicInput?.value.trim() || '';
  const type = txTypeInput?.value.trim() || '';
  if (!name || !type) { setTxStatus('请填写话题名与消息类型'); return; }
  let obj;
  try {
    obj = JSON.parse(txPayloadInput?.value || '');
  } catch (e) {
    setTxStatus(`JSON 解析失败:${e?.message || e}`);
    return;
  }
  if (obj === null || typeof obj !== 'object' || Array.isArray(obj)) {
    setTxStatus('消息 JSON 需为对象,如 {"data": "hello"}');
    return;
  }
  try {
    ensureAdvertised(name, type, ros);
    txTopic.publish(new ROSLIB.Message(obj));
    txSendCount += 1;
    addTopicHistory('tx', name, type);
    setTxStatus(`已发送 ${txSendCount} 次 → ${name}`);
  } catch (err) {
    setTxStatus(`发布失败:${err?.message || err}`);
  }
}

/** 停止发布:unadvertise 并复位状态 */
function stopPublish() {
  if (txTopic) {
    try { txTopic.unadvertise(); } catch { /* 忽略 */ }
    txTopic = null;
    txName = '';
    txType = '';
    txSendCount = 0;
  }
  setTxStatus('未发布');
}

// ── 断开清理 ───────────────────────────────────────────────

/** ROS 断开时清理接收/发送状态(ros 已关闭,直接置空即可) */
function teardownAll() {
  rxTopic = null;
  rxBuffer.length = 0;
  if (rxToggleBtn) rxToggleBtn.textContent = '订阅';
  if (rxTopicInput) rxTopicInput.disabled = false;
  if (rxTypeInput) rxTypeInput.disabled = false;
  txTopic = null; txName = ''; txType = ''; txSendCount = 0;
  setTxStatus('未发布');
}

// ── 初始化 ─────────────────────────────────────────────────

/** 页签切换到「话题调试」时调用:在线则刷新话题列表 */
export function onRosDebugTabShown() {
  if (connectionStatus === 'online') refreshTopicList();
}

/**
 * 初始化话题调试工具(绑定 DOM 事件 + 状态监听)。
 * 在 main.js 模块加载时调用一次;元素缺失时静默返回。
 */
export function initRosDebug() {
  topicsRefreshBtn = document.querySelector('#ros-debug-topics-refresh');
  debugHint = document.querySelector('#ros-debug-hint');
  rxTopicInput = document.querySelector('#ros-debug-rx-topic');
  rxTypeInput = document.querySelector('#ros-debug-rx-type');
  rxToggleBtn = document.querySelector('#ros-debug-rx-toggle');
  rxPauseInput = document.querySelector('#ros-debug-rx-pause');
  rxClearBtn = document.querySelector('#ros-debug-rx-clear');
  rxLog = document.querySelector('#ros-debug-rx-log');
  rxCountOutput = document.querySelector('#ros-debug-rx-count');
  txTopicInput = document.querySelector('#ros-debug-tx-topic');
  txTypeInput = document.querySelector('#ros-debug-tx-type');
  txPayloadInput = document.querySelector('#ros-debug-tx-payload');
  txSendBtn = document.querySelector('#ros-debug-tx-send');
  txStopBtn = document.querySelector('#ros-debug-tx-stop');
  txStatusOutput = document.querySelector('#ros-debug-tx-status');
  if (!rxLog) return;

  // 刷新话题(更新接收区下拉数据)
  topicsRefreshBtn?.addEventListener('click', refreshTopicList);

  // 接收区
  rxDdPanel = document.querySelector('#ros-debug-rx-dd');
  rxToggleBtn?.addEventListener('click', toggleSubscribe);
  // 接收话题输入框融合下拉(与发送区共用 wireTopicDropdown)
  wireTopicDropdown('rx', rxTopicInput, rxTypeInput, rxDdPanel, document.querySelector('#ros-debug-rx-topic-clear'));
  // 消息类型输入框内 × 按钮:一键清空消息类型
  document.querySelector('#ros-debug-rx-type-clear')?.addEventListener('click', () => {
    if (!rxTypeInput || rxTypeInput.disabled) return;
    rxTypeInput.value = '';
    rxTypeInput.focus();
  });
  rxPauseInput?.addEventListener('change', () => {
    rxPaused = !!rxPauseInput.checked;
  });
  rxClearBtn?.addEventListener('click', clearRxLog);

  // 发送区:话题输入融合下拉(同接收区) + 类型输入 × + 发布按钮
  txDdPanel = document.querySelector('#ros-debug-tx-dd');
  wireTopicDropdown('tx', txTopicInput, txTypeInput, txDdPanel, document.querySelector('#ros-debug-tx-topic-clear'));
  document.querySelector('#ros-debug-tx-type-clear')?.addEventListener('click', () => {
    if (!txTypeInput || txTypeInput.disabled) return;
    txTypeInput.value = '';
    txTypeInput.focus();
  });
  txSendBtn?.addEventListener('click', publishOnce);
  txStopBtn?.addEventListener('click', stopPublish);

  // 日志批量刷新定时器
  if (rxFlushTimer) clearInterval(rxFlushTimer);
  rxFlushTimer = setInterval(flushRxLog, RX_FLUSH_MS);

  // 连接状态:仅在 online/offline「切换」时拉话题或清理(状态监听器会高频触发)
  let _lastOnline = null;
  addStatusListener((snap) => {
    const online = snap.connectionStatus === 'online';
    if (topicsRefreshBtn) topicsRefreshBtn.disabled = !online;
    if (online && _lastOnline !== true) {
      refreshTopicList();
    } else if (!online && _lastOnline !== false) {
      teardownAll();
      setHint('未连接 ROS — 连接后可获取话题。');
    }
    _lastOnline = online;
  });
}
