/**
 * 电池(BMS)状态徽标（topbar 上的 🔋 小胶囊）。
 *
 * 数据链路：小脑(~192.168.8.236)上的 bms_ros_node 经 **BLE** 读电池 BMS
 * （Modbus RTU over BLE 透明传输）→ 发布 ROS2 话题 `/bms/state`
 * （std_msgs/msg/String，JSON）。大脑经 DDS 直连订阅，本模块只读并显示。
 *
 * JSON 字段（见小脑 ~/bms/bms_ros_node.py）：
 *   {ok, soc, voltage, current, remain_ah, temp:[t1,t2,t3],
 *    cell_min, cell_max, stale, ts}
 *
 * 显示规则：SOC 为主；ok=false 或超 90s 无新数据 → "--"；
 * SOC<30% 黄、<15% 红；悬停 tooltip 显示电压/电流/温度/压差。
 *
 * 单导入方模块（仅 main.js 引用），可独立 ?v= 版本号。
 * 注意：ros-bridge.js 必须用与其它模块相同的 ?v=980 引用，否则会加载两份实例。
 */
import { getRosClient, addStatusListener } from './ros-bridge.js?v=980';

const TOPIC_NAME = '/bms/state';
const STALE_MS = 90 * 1000;   // 超时判陈旧（小脑端本身也有 stale 标志，双保险）

let el = null;
let topic = null;
let last = null;              // 最近一条解析后的 JSON
let lastRxAt = 0;             // performance.now()
let tickTimer = null;

function setBadge(text, cls, title) {
  if (!el) return;
  el.textContent = text;
  el.className = 'battery-badge ' + (cls || '');
  if (title !== undefined) el.title = title;
}

function render() {
  if (!el) return;
  const fresh = last && (performance.now() - lastRxAt) <= STALE_MS;
  if (!fresh) {
    setBadge('🔋 --', 'is-stale',
             last ? '电池数据陈旧（超过 90s 未更新）' : '等待电池数据（/bms/state）');
    return;
  }
  const soc = Number(last.soc);
  const cls = last.ok === false ? 'is-stale'
            : (soc < 15 ? 'is-low' : (soc < 30 ? 'is-warn' : 'is-ok'));
  const tip = [];
  if (Number.isFinite(Number(last.voltage))) tip.push(`${Number(last.voltage).toFixed(1)} V`);
  if (Number.isFinite(Number(last.current))) tip.push(`${Number(last.current).toFixed(1)} A`);
  if (Array.isArray(last.temp)) tip.push(`${last.temp.join('/')} °C`);
  if (Number.isFinite(Number(last.cell_min)) && Number.isFinite(Number(last.cell_max))) {
    tip.push(`压差 ${Math.round((last.cell_max - last.cell_min) * 1000)} mV`);
  }
  if (last.ok === false && last.reason) tip.push(`(${last.reason})`);
  setBadge(`🔋 ${Number.isFinite(soc) ? soc + '%' : '--'}`, cls,
           (last.ok === false ? '电池离线' : `电量 ${soc}%`) +
           (tip.length ? ' · ' + tip.join(' · ') : ''));
}

function onMessage(msg) {
  try {
    last = JSON.parse(msg.data);
    lastRxAt = performance.now();
  } catch { /* 单条坏 JSON 忽略 */ }
  render();
}

function unsubscribe() {
  if (topic) {
    try { topic.unsubscribe(); } catch { /* 连接已断时忽略 */ }
    topic = null;
  }
}

function syncSubscription() {
  const ros = getRosClient();
  if (ros && !topic) {
    topic = new ROSLIB.Topic({
      ros, name: TOPIC_NAME, messageType: 'std_msgs/msg/String',
      compression: 'none', throttle_rate: 0, queue_length: 0, queue_size: 1,
    });
    topic.subscribe(onMessage);
  } else if (!ros && topic) {
    unsubscribe();
    render();               // 掉线 → 立刻回到 "--"
  }
}

export function initBattery() {
  el = document.querySelector('#battery-badge');
  if (!el) return;          // 页面没放徽标就静默跳过
  addStatusListener(() => syncSubscription());
  syncSubscription();
  if (tickTimer) clearInterval(tickTimer);
  tickTimer = setInterval(render, 5000);   // 只用于陈旧判定，不重绘数据
  render();
}
