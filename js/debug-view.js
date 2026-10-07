/**
 * 调试页签:一键启动/停止板上脚本(经 webserver 的 /api/debug/*),终端日志实时转发。
 *
 * 数据链路:按钮 → POST /api/debug/start | stop {target} → webserver 起/停
 *     slam: ~/start_slam.sh
 *     dog : ~/rosrun.sh "ros2 launch core_2026 dog_web.launch.py"
 * → 进程 stdout/stderr 落 ~/logs/<key>.log → 本模块按 offset 增量拉
 *   GET /api/debug/log?target=&offset= → <pre> 里滚动显示。
 *
 * 停止:整条进程组 SIGINT(等价终端 Ctrl-C),webserver 侧 10s 未完升 SIGKILL;
 * **只停本页启动过的**(pidfile 记录),手工起的显示为 external、按钮禁用不代停。
 * 停止期间 state 带 stopping → 徽标/按钮显示「停止中…」,start 也锁住。
 *
 * 可见时才轮询(log 1s / state 2s),切走停 —— 不白白打请求。
 * 谁连上这个页面谁就能点这些按钮(局域网信任模型与「任务编排」页同级,
 * 详见 webserver.py 文件头的「安全」段)。
 * 按钮**不弹确认框**(2026-10-03 按使用方要求去掉):点下去立即执行;按错就按
 * 对面的按钮 —— 起错了能停、停错了能再起。
 *
 * 单导入方模块(仅 main.js 引用),可独立 ?v= 版本号。
 */

const TARGETS = ['slam', 'dog'];

let els = {};
let viewVisible = false;
let stateTimer = 0;
let logTimer = 0;
const offsets = { slam: 0, dog: 0 };

export function initDebugView() {
  els = {
    refresh: document.querySelector('#debug-btn-refresh'),
    state: {
      slam: document.querySelector('#debug-state-slam'),
      dog: document.querySelector('#debug-state-dog'),
    },
    start: {
      slam: document.querySelector('#debug-start-slam'),
      dog: document.querySelector('#debug-start-dog'),
    },
    stop: {
      slam: document.querySelector('#debug-stop-slam'),
      dog: document.querySelector('#debug-stop-dog'),
    },
    msg: {
      slam: document.querySelector('#debug-msg-slam'),
      dog: document.querySelector('#debug-msg-dog'),
    },
    log: {
      slam: document.querySelector('#debug-log-slam'),
      dog: document.querySelector('#debug-log-dog'),
    },
  };
  if (!els.refresh) return;

  els.refresh.addEventListener('click', () => {
    pollState();
    for (const key of TARGETS) pollLog(key);
  });
  els.start.slam?.addEventListener('click', () => startTarget('slam'));
  els.start.dog?.addEventListener('click', () => startTarget('dog'));
  els.stop.slam?.addEventListener('click', () => stopTarget('slam'));
  els.stop.dog?.addEventListener('click', () => stopTarget('dog'));
}

/** 页签可见时才轮询;切走立即停(定时器挂着也只是空转,不如收干净) */
export function setDebugViewVisible(visible) {
  viewVisible = !!visible;
  clearInterval(stateTimer);
  clearInterval(logTimer);
  stateTimer = logTimer = 0;
  if (!viewVisible) return;
  pollState();
  for (const key of TARGETS) pollLog(key);
  stateTimer = setInterval(pollState, 2000);
  logTimer = setInterval(() => { for (const key of TARGETS) pollLog(key); }, 1000);
}

async function startTarget(key) {
  setMsg(key, '正在启动…');
  try {
    const res = await fetch('/api/debug/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ target: key }),
    });
    const obj = await res.json().catch(() => null);
    if (!res.ok || !obj || !obj.ok) {
      setMsg(key, `启动被拒:${(obj && obj.error) || ('HTTP ' + res.status)}`);
    } else {
      setMsg(key, `已启动(pid ${obj.pid}),日志如下`);
      offsets[key] = 0;
      if (els.log[key]) els.log[key].textContent = '';
      pollState();
      pollLog(key);
    }
  } catch (err) {
    setMsg(key, `请求失败:${err.message}`);
  }
}

async function stopTarget(key) {
  setMsg(key, '正在停止…');
  try {
    const res = await fetch('/api/debug/stop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ target: key }),
    });
    const obj = await res.json().catch(() => null);
    if (!res.ok || !obj || !obj.ok) {
      setMsg(key, `停止被拒:${(obj && obj.error) || ('HTTP ' + res.status)}`);
    } else if (obj.already_stopped) {
      setMsg(key, '它已经不在了(刚停完或从未启动),看下面日志确认');
    } else {
      setMsg(key, '已发停止信号(SIGINT),等各节点退出…');
    }
    pollState();
    pollLog(key);
  } catch (err) {
    setMsg(key, `请求失败:${err.message}`);
  }
}

async function pollState() {
  try {
    const res = await fetch('/api/debug/state', { cache: 'no-store' });
    const obj = await res.json();
    if (!obj || !obj.ok || !obj.targets) return;
    for (const key of TARGETS) {
      if (obj.targets[key]) renderState(key, obj.targets[key]);
    }
  } catch { /* 服务重启空窗等 — 忽略这轮 */ }
}

function renderState(key, st) {
  const el = els.state[key];
  if (!el) return;
  const startBtn = els.start[key];
  const stopBtn = els.stop[key];
  if (st.stopping) {
    // 停止信号已发、等各节点退出:徽标黄、两个按钮都锁
    el.textContent = '停止中…';
    el.classList.remove('is-on');
    el.classList.add('is-ext');
    if (startBtn) startBtn.disabled = true;
    if (stopBtn) {
      stopBtn.textContent = '停止中…';
      stopBtn.disabled = true;
      stopBtn.title = '';
    }
  } else if (st.running) {
    el.textContent = st.source === 'page'
      ? `运行中 · 本页启动 · pid ${st.pid}`
      : `运行中 · 手工起的 · pid ${st.pid}`;
    el.classList.toggle('is-on', st.source === 'page');
    el.classList.toggle('is-ext', st.source !== 'page');
    if (startBtn) startBtn.disabled = true;
    if (stopBtn) {
      stopBtn.textContent = '■ 停止';
      stopBtn.disabled = st.source !== 'page';
      stopBtn.title = st.source === 'page'
        ? '' : '手工起的，本页不代为停止（去终端 Ctrl-C）';
    }
  } else {
    el.textContent = '未运行';
    el.classList.remove('is-on', 'is-ext');
    if (startBtn) startBtn.disabled = false;
    if (stopBtn) {
      stopBtn.textContent = '■ 停止';
      stopBtn.disabled = true;
      stopBtn.title = '';
    }
  }
}

async function pollLog(key) {
  try {
    const res = await fetch(`/api/debug/log?target=${key}&offset=${offsets[key]}`,
      { cache: 'no-store' });
    if (!res.ok) return;
    const obj = await res.json();
    if (!obj || !obj.ok) return;
    offsets[key] = obj.offset;
    if (obj.text) appendLog(key, obj.text);
  } catch { /* 忽略这轮 */ }
}

// ROS 的日志带 ANSI 颜色(tty 上看的),<pre> 里要剥掉
const ANSI_RE = /\x1b\[[0-9;]*[A-Za-z]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)/g;
const LOG_MAX_LINES = 1200;

function appendLog(key, text) {
  const pre = els.log[key];
  if (!pre) return;
  const clean = text.replace(ANSI_RE, '').replace(/\r/g, '');
  if (!clean) return;
  // 贴底才自动滚 —— 用户往上翻历史时别把视口拽走
  const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 24;
  pre.textContent += clean;
  const lines = pre.textContent.split('\n');
  if (lines.length > LOG_MAX_LINES) {
    pre.textContent = `…(仅保留最近 ${LOG_MAX_LINES} 行)\n` +
      lines.slice(-LOG_MAX_LINES).join('\n');
  }
  if (atBottom) pre.scrollTop = pre.scrollHeight;
}

function setMsg(key, text) {
  const el = els.msg[key];
  if (el) el.textContent = text;
}
