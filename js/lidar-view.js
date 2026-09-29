/**
 * 雷达页签模块(Livox MID-360)+ 建图开关。
 *
 * 点云: /lidar_recorder/frame  sensor_msgs/msg/PointCloud2, ~10Hz, 约 5000 点/帧
 *      **只在建图开关打开时才有**。来源是板上 lidar_recorder 节点转发的
 *      Point-LIO 世界系逐帧点云(/cloud_registered,已去畸变、已按位姿拼好),
 *      字段 x,y,z,intensity 均 float32、point_step=16;
 *      解码器按 msg.fields 的 offset/datatype 泛化解析,兼容 uint8 intensity
 *      和 Point-LIO 原生那种更宽的 PointXYZINormal 布局。
 *      因为已经是世界系,"累加建图"直接叠就是全局一致的地图(机器人移动也不糊),
 *      和节点落盘的 pcd 是同一批点。
 * IMU: /livox/imu  sensor_msgs/msg/Imu, 200Hz 内置 BMI088(orientation 通常为空,姿态由重力估计)。
 *      它不属点云,仍按「雷达页签可见且在线」订阅,不受建图开关影响。
 *
 * 建图开关:按钮 → publishLidarRecord(Bool) → 板上节点开始/结束;
 * 节点每 0.5s 回一条 /lidar_recorder/status(JSON),**以它为准**驱动 UI 与点云订阅
 * —— 这样刷新页面/断线重连后按钮状态能自动对齐,而不是只信自己刚发出去的命令。
 *
 * 单导入方模块(仅 main.js 引用),可独立 ?v= 版本号。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  addStatusListener,
  addLidarFrameListener, addLidarImuListener,
  addLidarRecStatusListener,
  publishLidarRecord, setLidarRecordStreamEnabled,
  setLidarEnabled,
} from './ros-bridge.js?v=980';

let viewVisible = false;
let rosOnline = false;
let els = {};

// ── Three.js 资源 ─────────────────────────────────────────
let renderer = null;
let scene = null;
let camera3d = null;
let controls = null;
let pointsObj = null;
let pointsGeo = null;
let dataGroup = null;
let rafId = 0;
let autoRotate = false;
let colorMode = 'height'; // 'height' | 'intensity'
/** 最新一帧解码结果(切着色模式时重算颜色) */
let latestFrame = null;

// ── 建图开关状态 ─────────────────────────────────────────
/** 点了开关后等节点回状态的时限;超了就是节点没起来(或话题名对不上) */
const REC_PENDING_MS = 3000;
/** 'idle' | 'pending' | 'recording' —— 以节点 status 为准,不是本地乐观状态 */
let recPhase = 'idle';
let recWant = false;          // pending 期间:这次点的是"开始"还是"结束"
let recPendingUntil = 0;
let recStatus = null;         // 最近一条 status(JSON 对象)
let recSavedPath = '';        // 最近一次落盘的 pcd 路径
let recStaleWarned = false;   // "节点无响应"只提示一次,别刷屏
/**
 * 录制起点的本地时刻。按钮上的秒数用它算,不直接用 status 里的 elapsed_s ——
 * 点云帧很大,rosbridge 只有一条有序 WebSocket,状态消息会被大帧堵在后面,
 * 实测能滞后好几秒;那样秒数会"卡住不动",看着像死了。
 */
let recStartLocal = 0;

// ── 累加建图:Livox 是非重复扫描,单帧只覆盖场景的一小部分,
//    多帧持续接收并在固定坐标系重叠 → 体素去重后逐步还原完整场景。
//    静止(匀速)采集前提;机器人移动后需配合位姿(里程计)做坐标变换。
const ACC_VOXEL = 0.05;  // 体素边长(m):同一 5cm 立方体内只保留首个点
const ACC_MAX = 600000;  // 最大体素点数(position/color 缓冲按此预分配)
const ACC_KEY_BASE = 4096;
const ACC_KEY_OFFSET = 2048; // 覆盖 ±102m(voxel 坐标 0..4095)
let accumulate = true;
let accMap = null;          // 体素键 → 1(去重集合)
let accObj = null;          // 累加点云 Points(与单帧 pointsObj 切换显示)
let accGeo = null;
let accPos = null;          // 预分配 Float32Array(ACC_MAX*3),只增量写入
let accCol = null;
let accInten = null;        // 每点反射率(切反射率着色时用)
let accCount = 0;
let accFull = false;        // 达到容量上限
let accRgb = null;          // 每点真彩(packed float32,NaN=没有颜色);切真彩着色时用
let coloredCount = 0;       // 累加里带真彩的点数(面板上显示)
const recoloredAt = new Set();   // 本帧被"原地换色"的点下标(要单独推 GPU)

// ── 点大小(用户可调,与相机缩放解耦;两套材质同一尺寸) ──
const POINT_SIZE_KEY = 'web_sim_lidar_point_size';
const POINT_SIZE_MIN = 0.01;
const POINT_SIZE_MAX = 0.30;
const POINT_SIZE_DEFAULT = 0.05;
let pointSize = POINT_SIZE_DEFAULT;
let frameMat = null;
let accMat = null;

// ── 统计 ─────────────────────────────────────────────────
let frameCount = 0;
let windowFrames = 0;
let windowBytes = 0;
let windowStartedAt = 0;
let lastFrameAt = 0;
let lastPointCount = 0;
let lastMaxRange = 0;
let lastPointStep = 0;
let lastFrameId = '-';

// ── IMU ──────────────────────────────────────────────────
let lastLidarImu = null;
let lastRoll = 0;
let lastPitch = 0;
/** 6 通道滚动波形缓冲:ax,ay,az,gx,gy,gz */
const WAVE_LEN = 300;
const waveBuf = [
  new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN),
  new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN),
];
let waveHead = 0;
let waveSamples = 0;
let waveDirty = true;

const ACC_SCALE = 20;  // m/s² 波形满量程
const GYR_SCALE = 3;   // rad/s 波形满量程
const WAVE_COLORS = ['#ff7a7a', '#7ee08a', '#6db6ff', '#c96565', '#5fae6b', '#568bc9'];

function fmt(n, d = 2) { return Number.isFinite(n) ? n.toFixed(d) : '-'; }
function radToDeg(r) { return r * 180 / Math.PI; }

/** base64 → Uint8Array */
function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

// ── PointCloud2 解码 ─────────────────────────────────────
// ROS PointField datatype 枚举
const PF_INT8 = 1, PF_UINT8 = 2, PF_INT16 = 3, PF_UINT16 = 4, PF_INT32 = 5, PF_UINT32 = 6, PF_FLOAT32 = 7, PF_FLOAT64 = 8;

/**
 * 解析 PointCloud2 → { positions:Float32Array(n*3), intensity:Float32Array(n), count, maxRange, minZ, maxZ }
 * 非有限点(NaN/inf 无效回波)丢弃。
 */
function decodePointCloud2(msg) {
  const raw = typeof msg.data === 'string' ? b64ToBytes(msg.data) : msg.data;
  if (!raw) return null;
  const u8 = raw instanceof Uint8Array ? raw : new Uint8Array(raw);
  const pointStep = msg.point_step | 0;
  if (!pointStep) return null;
  const n = (msg.width | 0) * (msg.height > 0 ? msg.height | 0 : 1);
  if (!n) return null;
  const le = !msg.is_bigendian;

  // 按 fields 找偏移(默认 livox 驱动布局 x@0 y@4 z@8 intensity@12, 全 float32)
  // rgb 是彩色相机那一路带的(PCL 约定:一个 float32 里塞 0x00RRGGBB);
  // 没有颜色的点(雷达点)在下面填 NaN —— 用它区分"没有真彩"和"黑色"
  let ox = 0, oy = 4, oz = 8, oi = 12, itype = PF_FLOAT32, hasIntensity = false;
  let oc = -1, ctype = PF_FLOAT32;
  for (const f of msg.fields || []) {
    if (f.name === 'x') ox = f.offset;
    else if (f.name === 'y') oy = f.offset;
    else if (f.name === 'z') oz = f.offset;
    else if (f.name === 'intensity' || f.name === 'i') { oi = f.offset; itype = f.datatype; hasIntensity = true; }
    else if (f.name === 'rgb' || f.name === 'rgba') { oc = f.offset; ctype = f.datatype; }
  }

  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  const positions = new Float32Array(n * 3);
  const intensity = new Float32Array(n);
  const rgb = new Float32Array(n).fill(NaN);
  let valid = 0;
  let colored = 0;
  let maxRange = 0, minZ = Infinity, maxZ = -Infinity;
  const readIntensity = (p) => {
    switch (itype) {
      case PF_FLOAT32: return dv.getFloat32(p + oi, le);
      case PF_FLOAT64: return dv.getFloat64(p + oi, le);
      case PF_UINT8: return dv.getUint8(p + oi);
      case PF_INT8: return dv.getInt8(p + oi);
      case PF_UINT16: return dv.getUint16(p + oi, le);
      case PF_INT16: return dv.getInt16(p + oi, le);
      case PF_UINT32: return dv.getUint32(p + oi, le);
      case PF_INT32: return dv.getInt32(p + oi, le);
      default: return 0;
    }
  };
  // 颜色统一读成 packed float32 的位模式(和节点那边一致)
  const readRgb = (p) => {
    if (ctype === PF_FLOAT32) return dv.getFloat32(p + oc, le);
    if (ctype === PF_UINT32) {
      const u = dv.getUint32(p + oc, le);
      return new Float32Array(new Uint32Array([u]).buffer)[0];
    }
    return NaN;
  };
  for (let i = 0; i < n; i++) {
    const p = i * pointStep;
    if (p + Math.max(ox, oy, oz) + 4 > u8.length) break;
    const x = dv.getFloat32(p + ox, le);
    const y = dv.getFloat32(p + oy, le);
    const z = dv.getFloat32(p + oz, le);
    if (!Number.isFinite(x + y + z)) continue;
    const o = valid * 3;
    positions[o] = x; positions[o + 1] = y; positions[o + 2] = z;
    if (hasIntensity && p + oi + 1 <= u8.length) intensity[valid] = readIntensity(p);
    if (oc >= 0 && p + oc + 4 <= u8.length) {
      const c = readRgb(p);
      // 0.0 = 全黑,分不清"合法黑点"和"没填",按没填处理(相机不会真发纯黑)
      if (Number.isFinite(c) && c !== 0) { rgb[valid] = c; colored++; }
    }
    const r = Math.sqrt(x * x + y * y + z * z);
    if (r > maxRange) maxRange = r;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
    valid++;
  }
  if (!valid) return null;
  return {
    positions: positions.slice(0, valid * 3),
    intensity: intensity.slice(0, valid),
    rgb: rgb.slice(0, valid),
    colored,
    count: valid, maxRange, minZ: minZ === Infinity ? 0 : minZ, maxZ: maxZ === -Infinity ? 0 : maxZ,
  };
}

/** 高度→颜色:蓝(低)→青→绿→黄→红(高),固定量程 -2~3m,避免帧间闪烁 */
const _cTmp = new THREE.Color();
function heightColor(z, out, idx) {
  const t = THREE.MathUtils.clamp((z + 2) / 5, 0, 1);
  _cTmp.setHSL((1 - t) * 0.66, 0.85, 0.55);
  out[idx] = _cTmp.r; out[idx + 1] = _cTmp.g; out[idx + 2] = _cTmp.b;
}
/** 反射率→灰度(Livox intensity 常见 0~255) */
function intensityColor(v, out, idx) {
  const g = THREE.MathUtils.clamp(v / 255, 0.04, 1);
  out[idx] = g * 0.85; out[idx + 1] = g * 0.95; out[idx + 2] = g;
}
/** packed float32(0x00RRGGBB) → RGB 分量(0~1) */
function unpackRgb(packed, out, idx) {
  const u = new Uint32Array(new Float32Array([packed]).buffer)[0];
  out[idx] = ((u >> 16) & 0xFF) / 255;
  out[idx + 1] = ((u >> 8) & 0xFF) / 255;
  out[idx + 2] = (u & 0xFF) / 255;
}

/**
 * 按当前模式给一个点上色。
 * **'真彩' 模式下没有颜色的点回落到高度着色** —— 相机没扫到的地方不能是一片黑，
 * 那样地图看起来像破了个洞，反而比不上色更难读。
 */
function colorPoint(mode, z, inten, packed, out, idx) {
  if (mode === 'rgb') {
    if (Number.isFinite(packed)) { unpackRgb(packed, out, idx); return; }
    heightColor(z, out, idx);
    return;
  }
  if (mode === 'intensity') intensityColor(inten, out, idx);
  else heightColor(z, out, idx);
}

function recomputeColors(frame) {
  const colors = new Float32Array(frame.count * 3);
  const pos = frame.positions;
  for (let i = 0; i < frame.count; i++) {
    colorPoint(colorMode, pos[i * 3 + 2], frame.intensity[i], frame.rgb[i], colors, i * 3);
  }
  return colors;
}

// ── Three 场景 ───────────────────────────────────────────
function initThree() {
  const canvas = els.canvas;
  renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);

  scene = new THREE.Scene();
  camera3d = new THREE.PerspectiveCamera(60, 1, 0.05, 200);
  camera3d.position.set(5.5, -5.5, 4.2);

  controls = new OrbitControls(camera3d, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.minDistance = 0.3;
  controls.maxDistance = 80;
  controls.target.set(0, 0, 0);

  // 雷达数据是 z-up,Three 默认 y-up:全部数据放进绕 X 轴 -90° 的组(local z→world y)
  dataGroup = new THREE.Group();
  dataGroup.rotation.x = -Math.PI / 2;
  scene.add(dataGroup);

  // 地面网格(传感器原点为中心)
  const grid = new THREE.GridHelper(40, 40, 0x2a4458, 0x18262f);
  grid.material.transparent = true;
  grid.material.opacity = 0.55;
  grid.material.depthWrite = false;
  dataGroup.add(grid);
  dataGroup.add(new THREE.AxesHelper(1.2));

  pointsGeo = new THREE.BufferGeometry();
  pointsGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3), 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3), 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100); // 固定大球,跳过逐帧包围体重算
  frameMat = new THREE.PointsMaterial({
    size: pointSize, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.92, depthWrite: false,
  });
  const mat = frameMat;
  pointsObj = new THREE.Points(pointsGeo, mat);
  pointsObj.frustumCulled = false;
  dataGroup.add(pointsObj);

  // 累加点云:容量一次性预分配,运行时只往尾部追加并扩大 drawRange,
  // GPU 仅上传新增区间(addUpdateRange),不做任何全量重建/拷贝
  accMap = new Map();
  accPos = new Float32Array(ACC_MAX * 3);
  accCol = new Float32Array(ACC_MAX * 3);
  accInten = new Float32Array(ACC_MAX);
  accRgb = new Float32Array(ACC_MAX).fill(NaN);
  accGeo = new THREE.BufferGeometry();
  accGeo.setAttribute('position', new THREE.BufferAttribute(accPos, 3).setUsage(THREE.DynamicDrawUsage));
  accGeo.setAttribute('color', new THREE.BufferAttribute(accCol, 3).setUsage(THREE.DynamicDrawUsage));
  accGeo.setDrawRange(0, 0);
  accGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100);
  accMat = new THREE.PointsMaterial({
    size: pointSize, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.95, depthWrite: false,
  });
  accObj = new THREE.Points(accGeo, accMat);
  accObj.frustumCulled = false;
  accObj.visible = accumulate;
  dataGroup.add(accObj);
  pointsObj.visible = !accumulate;

  resizeRenderer();
  if (typeof ResizeObserver !== 'undefined') {
    const ro = new ResizeObserver(() => resizeRenderer());
    ro.observe(canvas.parentElement);
  }
  window.addEventListener('resize', resizeRenderer);

  const loop = () => {
    rafId = requestAnimationFrame(loop);
    if (!viewVisible) return;
    if (autoRotate) dataGroup.rotation.z += 0.004; // 绕竖直(雷达 z)轴缓慢转
    controls.update();
    renderer.render(scene, camera3d);
    if (waveDirty) { drawWaveform(); waveDirty = false; }
  };
  loop();
}

function resizeRenderer() {
  if (!renderer || !els.canvas) return;
  const w = els.canvas.clientWidth || 0;
  const h = els.canvas.clientHeight || 0;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera3d.aspect = w / h;
  camera3d.updateProjectionMatrix();
}

/** 用最新帧替换点云几何(单帧模式) */
function uploadFrame(frame) {
  latestFrame = frame;
  const colors = recomputeColors(frame);
  pointsGeo.setAttribute('position', new THREE.BufferAttribute(frame.positions, 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage));
}

// ── 累加建图 ─────────────────────────────────────────────
/** 体素哈希键:空间位置 → 0..4095³ 网格内的唯一整数 */
function voxelKey(x, y, z) {
  let ix = Math.floor(x / ACC_VOXEL) + ACC_KEY_OFFSET;
  let iy = Math.floor(y / ACC_VOXEL) + ACC_KEY_OFFSET;
  let iz = Math.floor(z / ACC_VOXEL) + ACC_KEY_OFFSET;
  if (ix < 0) ix = 0; else if (ix >= ACC_KEY_BASE) ix = ACC_KEY_BASE - 1;
  if (iy < 0) iy = 0; else if (iy >= ACC_KEY_BASE) iy = ACC_KEY_BASE - 1;
  if (iz < 0) iz = 0; else if (iz >= ACC_KEY_BASE) iz = ACC_KEY_BASE - 1;
  return (ix * ACC_KEY_BASE + iy) * ACC_KEY_BASE + iz;
}

/**
 * 把一帧并入累加地图:仅新体素追加到预分配缓冲尾部。
 * 返回本帧新增体素数。
 */
function insertAccumulated(frame) {
  if (!accMap || accCount >= ACC_MAX) { accFull = true; return 0; }
  const pos = frame.positions;
  const inten = frame.intensity;
  const rgb = frame.rgb;
  const start = accCount;
  for (let i = 0; i < frame.count; i++) {
    if (accCount >= ACC_MAX) { accFull = true; break; }
    const o = i * 3;
    const x = pos[o], y = pos[o + 1], z = pos[o + 2];
    const key = voxelKey(x, y, z);
    const at = accMap.get(key);
    if (at !== undefined) {
      // 同一体素:已经有雷达点(无色)、这回来的是相机点(带色) → 原地换成彩色的。
      // 和节点那边的落盘规则保持一致 —— 不然屏幕上和 pcd 里会长得不一样。
      if (Number.isFinite(rgb[i]) && !Number.isFinite(accRgb[at])) {
        accInten[at] = inten[i];
        accRgb[at] = rgb[i];
        colorPoint(colorMode, z, inten[i], rgb[i], accCol, at * 3);
        recoloredAt.add(at);
      }
      continue;
    }
    accMap.set(key, accCount);
    const w = accCount * 3;
    accPos[w] = x; accPos[w + 1] = y; accPos[w + 2] = z;
    accInten[accCount] = inten[i];
    accRgb[accCount] = rgb[i];
    if (Number.isFinite(rgb[i])) coloredCount++;
    colorPoint(colorMode, z, inten[i], rgb[i], accCol, w);
    accCount++;
  }
  // 原地换色的那些点也要把颜色推给 GPU(它们不在"新增区间"里)
  if (recoloredAt.size) {
    const ca = accGeo.getAttribute('color');
    for (const at of recoloredAt) ca.addUpdateRange(at * 3, 3);
    ca.needsUpdate = true;
    recoloredAt.clear();
  }
  const added = accCount - start;
  if (added > 0) {
    accGeo.setDrawRange(0, accCount);
    const pa = accGeo.getAttribute('position');
    const ca = accGeo.getAttribute('color');
    // addUpdateRange 的单位是"分量下标"(Float32 下标),顶点要 ×3
    pa.addUpdateRange(start * 3, added * 3);
    ca.addUpdateRange(start * 3, added * 3);
    pa.needsUpdate = true;
    ca.needsUpdate = true;
  }
  return added;
}

/** 切换着色模式后重刷全部累加点颜色(仅此时做一次全量上传) */
function recolorAccumulatedAll() {
  if (!accMap || accCount === 0) return;
  for (let i = 0; i < accCount; i++) {
    const w = i * 3;
    colorPoint(colorMode, accPos[w + 2], accInten[i], accRgb[i], accCol, w);
  }
  const ca = accGeo.getAttribute('color');
  ca.addUpdateRange(0, accCount * 3);
  ca.needsUpdate = true;
}

/** 清空累加地图 */
function clearAccumulated() {
  if (accMap) accMap.clear();
  accCount = 0;
  accFull = false;
  coloredCount = 0;
  recoloredAt.clear();
  if (accRgb) accRgb.fill(NaN);
  if (accGeo) accGeo.setDrawRange(0, 0);
}

/** 单帧/累加显示切换 */
function setAccumulateMode(on) {
  accumulate = !!on;
  if (accObj) accObj.visible = accumulate;
  if (pointsObj) pointsObj.visible = !accumulate;
}

/** 设置点大小(m,世界尺寸):同步两套材质并持久化 */
function applyPointSize(v) {
  pointSize = THREE.MathUtils.clamp(+v || POINT_SIZE_DEFAULT, POINT_SIZE_MIN, POINT_SIZE_MAX);
  if (frameMat) frameMat.size = pointSize;
  if (accMat) accMat.size = pointSize;
  if (els.sizeVal) els.sizeVal.textContent = pointSize.toFixed(2);
  try { localStorage.setItem(POINT_SIZE_KEY, String(pointSize)); } catch { /* 忽略 */ }
}

// ── 帧/IMU 回调 ─────────────────────────────────────────
function onLidarFrame(msg) {
  // 不因切走页签而丢帧:回来时看到的图应该是完整的一段,而不是"只有我盯着的那几秒"。
  // 渲染循环本来就会在不可见时自己跳过,这里只是继续解码/累加。
  const frame = decodePointCloud2(msg);
  if (!frame) return;
  if (accumulate) insertAccumulated(frame);
  else uploadFrame(frame);
  frameCount++;
  windowFrames++;
  // PointCloud2 二进制 base64 长度 ≈ 字节数 × 4/3
  const rawLen = (msg.point_step | 0) * frame.count;
  windowBytes += rawLen;
  lastPointCount = frame.count;
  lastMaxRange = frame.maxRange;
  lastPointStep = msg.point_step | 0;
  lastFrameId = String(msg.header?.frame_id || '-');
  const now = performance.now();
  if (!windowStartedAt) windowStartedAt = now;
  lastFrameAt = now;
  if (viewVisible) els.placeholder.style.display = 'none';
}

function onLidarImu(msg) {
  lastLidarImu = msg;
  const a = msg.linear_acceleration || {};
  const g = msg.angular_velocity || {};
  // BMI088 orientation 通常全 0:用重力向量估计 roll/pitch(静止/匀速时准)
  const ax = +a.x || 0, ay = +a.y || 0, az = +a.z || 0;
  const gNorm = Math.hypot(ax, ay, az);
  if (gNorm > 1e-3) {
    lastPitch = Math.asin(THREE.MathUtils.clamp(-ax / gNorm, -1, 1));
    lastRoll = Math.atan2(ay, az);
  }
  waveBuf[0][waveHead] = ax; waveBuf[1][waveHead] = ay; waveBuf[2][waveHead] = az;
  waveBuf[3][waveHead] = +g.x || 0; waveBuf[4][waveHead] = +g.y || 0; waveBuf[5][waveHead] = +g.z || 0;
  waveHead = (waveHead + 1) % WAVE_LEN;
  waveSamples++;
  waveDirty = true;
}

// ── 姿态球(由重力估计的 roll/pitch) ─────────────────────
function drawAttitude() {
  const c = els.attiCanvas;
  if (!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2, R = W / 2 - 2;
  ctx.clearRect(0, 0, W, H);
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.clip();
  // 天/地分界线随 roll 旋转、随 pitch 上下平移
  ctx.translate(cx, cy);
  ctx.rotate(-lastRoll);
  const off = THREE.MathUtils.clamp(radToDeg(lastPitch) / 45, -1, 1) * R;
  ctx.translate(0, off);
  ctx.fillStyle = '#6b4a1e'; // 地(褐)
  ctx.fillRect(-R, 0, 2 * R, 2 * R);
  ctx.fillStyle = '#2f6fae'; // 天(蓝)
  ctx.fillRect(-R, -2 * R, 2 * R, 2 * R);
  // 地平线
  ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-R, 0); ctx.lineTo(R, 0); ctx.stroke();
  // 俯仰刻度
  ctx.lineWidth = 1;
  for (const d of [-30, -20, -10, 10, 20, 30]) {
    const y = -d / 45 * R;
    ctx.beginPath(); ctx.moveTo(-14, y); ctx.lineTo(14, y); ctx.stroke();
  }
  ctx.restore();
  // 外圆 + 固定飞机符号(不随球转)
  ctx.strokeStyle = '#9fb4c4'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
  ctx.strokeStyle = '#ffd35c'; ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(cx - 26, cy); ctx.lineTo(cx - 8, cy);
  ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 26, cy);
  ctx.moveTo(cx - 8, cy); ctx.lineTo(cx - 8, cy - 6);
  ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 8, cy - 6);
  ctx.stroke();
  // 顶部滚转刻线
  ctx.fillStyle = '#9fb4c4'; ctx.font = '9px monospace'; ctx.textAlign = 'center';
  ctx.fillText(`R ${radToDeg(lastRoll).toFixed(1)}°  P ${radToDeg(lastPitch).toFixed(1)}°`, cx, H - 5);
}

// ── 6 通道滚动波形 ───────────────────────────────────────
function drawWaveform() {
  const c = els.waveCanvas;
  if (!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  // 零轴 + 量程参考线
  ctx.strokeStyle = 'rgba(255,255,255,0.12)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  for (const q of [0.25, 0.75]) { ctx.beginPath(); ctx.moveTo(0, H * q); ctx.lineTo(W, H * q); ctx.stroke(); }

  for (let ch = 0; ch < 6; ch++) {
    const buf = waveBuf[ch];
    const scale = ch < 3 ? ACC_SCALE : GYR_SCALE;
    ctx.strokeStyle = WAVE_COLORS[ch];
    ctx.lineWidth = 1;
    ctx.beginPath();
    const filled = Math.min(waveSamples, WAVE_LEN);
    for (let px = 0; px < filled; px++) {
      const idx = (waveHead - filled + px + WAVE_LEN) % WAVE_LEN;
      const v = THREE.MathUtils.clamp(buf[idx] / scale, -1, 1);
      const x = px / (WAVE_LEN - 1) * W;
      const y = H / 2 - v * (H / 2 - 3);
      if (px === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

// ── 数字面板 8Hz 刷新(高频消息不直写 DOM) ────────────────
let flushTimer = null;
/** 建图按钮/状态的刷新定时器(500ms;秒数要跳,且超时判定不能只在页签可见时跑) */
let recTimer = null;
function flushDom() {
  if (!viewVisible) return;
  const now = performance.now();
  const dt = windowStartedAt ? (now - windowStartedAt) / 1000 : 0;
  if (dt >= 0.5) {
    els.fps.textContent = `${fmt(windowFrames / dt, 1)} FPS`;
    els.kbps.textContent = `${(windowBytes / dt / 1024).toFixed(0)} KB/s`;
    els.fps.classList.toggle('is-stale', now - lastFrameAt > 1000);
    windowFrames = 0; windowBytes = 0; windowStartedAt = now;
  }
  els.points.textContent = `${lastPointCount} 点`;
  if (els.accPoints) {
    // 带上彩色点数:相机那一路有没有真的接进来,一眼就能看出来
    const colorNote = coloredCount ? `(彩 ${coloredCount})` : '';
    els.accPoints.textContent = `累计 ${accCount} 点${colorNote}`
      + (accFull ? '(满)' : '');
  }
  els.range.textContent = `${fmt(lastMaxRange, 1)} m`;
  els.step.textContent = lastPointStep ? `step ${lastPointStep}B` : '-';
  els.frame.textContent = lastFrameId;
  els.frameCount.textContent = `帧 ${frameCount}`;

  if (lastLidarImu) {
    const a = lastLidarImu.linear_acceleration || {};
    const g = lastLidarImu.angular_velocity || {};
    els.accX.textContent = fmt(a.x, 3); els.accY.textContent = fmt(a.y, 3); els.accZ.textContent = fmt(a.z, 3);
    els.gyrX.textContent = fmt(g.x, 3); els.gyrY.textContent = fmt(g.y, 3); els.gyrZ.textContent = fmt(g.z, 3);
    els.gyrXD.textContent = fmt(radToDeg(g.x), 1);
    els.gyrYD.textContent = fmt(radToDeg(g.y), 1);
    els.gyrZD.textContent = fmt(radToDeg(g.z), 1);
    drawAttitude();
  }
}

// ── 建图开关 UI ─────────────────────────────────────────
function fmtPoints(n) {
  const v = Number(n) || 0;
  return v >= 10000 ? `${(v / 10000).toFixed(1)} 万点` : `${v} 点`;
}

/** 状态胶囊里那一行:建图详情 / 落盘结果 / 未开始 */
function recStateText() {
  const s = recStatus;
  if (recPhase === 'recording') {
    if (s?.degraded) return `⚠ ${s.msg || '异常'}`;
    return `建图中 · ${s?.frames || 0} 帧 · ${fmtPoints(s?.points)} · `
         + `${(s?.mbps || 0).toFixed(1)} MB/s`;
  }
  if (recPhase === 'pending') return recWant ? '正在开始建图…' : '正在结束建图…';
  if (!rosOnline) return 'ROS 未连接';
  if (recSavedPath) return `已保存 ${recSavedPath.split('/').pop()} · ${fmtPoints(s?.points)}`;
  return '未开始建图(开关打开后每帧同时上图并落盘)';
}

/**
 * 500ms 刷一次建图按钮与状态文字。**不看页签可见性**(超时判定要在后台也算),
 * 只有真正写 DOM 时才判 visible。
 */
function tickRecording() {
  // 点了开关但节点一直没回状态 —— 多半是 lidar_recorder 没起来
  if (recPhase === 'pending' && performance.now() > recPendingUntil) {
    recPhase = (recStatus && recStatus.state === 'recording') ? 'recording' : 'idle';
    if (recPhase === 'idle') recStartLocal = 0;
    if (!recStaleWarned) {
      recStaleWarned = true;
      console.warn('[lidar-rec] 3s 内没收到 /lidar_recorder/status —— '
                 + '板上 lidar_recorder 节点没起来?');
    }
  }
  if (!viewVisible) return;

  const recording = recPhase === 'recording';
  if (els.btnRecord) {
    if (recording) {
      const secs = recStartLocal
        ? Math.floor((Date.now() - recStartLocal) / 1000)
        : Math.floor(recStatus?.elapsed_s || 0);
      els.btnRecord.textContent = `■ 结束建图 ${secs}s`;
      els.btnRecord.classList.add('is-recording');
      els.btnRecord.disabled = !rosOnline;
    } else if (recPhase === 'pending') {
      els.btnRecord.textContent = recWant ? '… 开始中' : '… 结束中';
      els.btnRecord.classList.remove('is-recording');
      els.btnRecord.disabled = true;
    } else {
      els.btnRecord.textContent = '● 开始建图';
      els.btnRecord.classList.remove('is-recording');
      els.btnRecord.disabled = !rosOnline;
    }
  }
  if (els.streamState) {
    els.streamState.textContent = !rosOnline ? 'ROS 未连接'
      : (recording ? '● 建图中' : '未建图');
    els.streamState.classList.toggle('is-live', recording);
  }
  if (els.recState) {
    els.recState.textContent = recStateText();
    els.recState.classList.toggle('is-recording', recording);
    els.recState.title = recSavedPath || '';
  }
}

// ── 可见性与订阅开关 ─────────────────────────────────────
function applyDesired() {
  // 点云不再跟页签可见性挂钩(见文件头注释):这里管的是雷达内置 IMU 与画布尺寸
  setLidarEnabled(viewVisible && rosOnline);
  if (viewVisible) resizeRenderer();
  tickRecording();
}

export function setLidarViewVisible(visible) {
  viewVisible = !!visible;
  applyDesired();
  if (viewVisible) {
    // 从隐藏恢复:canvas 尺寸可能为 0,重新对齐
    requestAnimationFrame(resizeRenderer);
  }
}

export function initLidarView() {
  els = {
    canvas: document.querySelector('#lidar-3d-canvas'),
    placeholder: document.querySelector('#lidar-placeholder'),
    streamState: document.querySelector('#lidar-stream-state'),
    fps: document.querySelector('#lidar-fps'),
    kbps: document.querySelector('#lidar-kbps'),
    points: document.querySelector('#lidar-points'),
    accPoints: document.querySelector('#lidar-acc-points'),
    range: document.querySelector('#lidar-range'),
    step: document.querySelector('#lidar-step'),
    frame: document.querySelector('#lidar-frame'),
    frameCount: document.querySelector('#lidar-frame-count'),
    btnReset: document.querySelector('#lidar-btn-reset'),
    btnTop: document.querySelector('#lidar-btn-top'),
    btnSide: document.querySelector('#lidar-btn-side'),
    btnRotate: document.querySelector('#lidar-btn-rotate'),
    btnColor: document.querySelector('#lidar-btn-color'),
    btnAcc: document.querySelector('#lidar-btn-acc'),
    btnClear: document.querySelector('#lidar-btn-clear'),
    btnRecord: document.querySelector('#lidar-btn-record'),
    recState: document.querySelector('#lidar-rec-state'),
    sizeRange: document.querySelector('#lidar-point-size'),
    sizeVal: document.querySelector('#lidar-point-size-val'),
    attiCanvas: document.querySelector('#lidar-atti'),
    waveCanvas: document.querySelector('#lidar-wave'),
    accX: document.querySelector('#lidar-acc-x'), accY: document.querySelector('#lidar-acc-y'), accZ: document.querySelector('#lidar-acc-z'),
    gyrX: document.querySelector('#lidar-gyr-x'), gyrY: document.querySelector('#lidar-gyr-y'), gyrZ: document.querySelector('#lidar-gyr-z'),
    gyrXD: document.querySelector('#lidar-gyr-x-d'), gyrYD: document.querySelector('#lidar-gyr-y-d'), gyrZD: document.querySelector('#lidar-gyr-z-d'),
  };
  if (!els.canvas) return;

  // 波形/姿态球固定内部分辨率(CSS 负责显示尺寸)
  els.waveCanvas.width = 600; els.waveCanvas.height = 110;
  els.attiCanvas.width = 132; els.attiCanvas.height = 132;

  initThree();

  const setView = (x, y, z, tx = 0, ty = 0, tz = 0) => {
    camera3d.position.set(x, y, z);
    controls.target.set(tx, ty, tz);
    controls.update();
  };
  els.btnReset?.addEventListener('click', () => setView(5.5, -5.5, 4.2));
  els.btnTop?.addEventListener('click', () => setView(0.01, 0.01, 14));
  els.btnSide?.addEventListener('click', () => setView(0.01, -10, 0.6));
  els.btnRotate?.addEventListener('click', () => {
    autoRotate = !autoRotate;
    els.btnRotate.classList.toggle('is-active', autoRotate);
    if (!autoRotate) dataGroup.rotation.z = 0;
  });
  // 着色模式三档循环:高度 → 反射率 → 真彩(相机那路带的颜色;没有的回落高度)
  els.btnColor?.addEventListener('click', () => {
    colorMode = colorMode === 'height' ? 'intensity'
      : (colorMode === 'intensity' ? 'rgb' : 'height');
    els.btnColor.textContent = colorMode === 'height' ? '着色:高度'
      : (colorMode === 'intensity' ? '着色:反射率' : '着色:真彩');
    els.btnColor.classList.toggle('is-active', colorMode === 'rgb');
    if (latestFrame) uploadFrame(latestFrame);
    recolorAccumulatedAll();
  });
  els.btnAcc?.addEventListener('click', () => {
    setAccumulateMode(!accumulate);
    els.btnAcc.classList.toggle('is-active', accumulate);
  });
  els.btnClear?.addEventListener('click', () => {
    clearAccumulated();
  });

  // 建图开关:发一条 Bool 给板上录制节点,之后以节点回的 status 为准
  // (不在这里直接改按钮态 —— 命令可能根本没被节点收到,那就该显示"无响应")
  els.btnRecord?.addEventListener('click', () => {
    const want = recPhase !== 'recording';
    if (!publishLidarRecord(want)) {
      if (els.recState) els.recState.textContent = 'ROS 未连接,无法建图';
      return;
    }
    recPhase = 'pending';
    recWant = want;
    recPendingUntil = performance.now() + REC_PENDING_MS;
    recStaleWarned = false;
    // 开始建图时清掉上一次的累计,免得新旧地图叠在一起
    if (want) { clearAccumulated(); recSavedPath = ''; recStartLocal = Date.now(); }
    else { recStartLocal = 0; }
    tickRecording();
  });

  addLidarRecStatusListener((s) => {
    recStatus = s;
    const recording = s.state === 'recording';
    recPhase = recording ? 'recording' : 'idle';
    // 点云订阅跟着**节点的真实状态**走:命令也可能是别的页面/命令行发的
    setLidarRecordStreamEnabled(recording);
    if (recording) {
      recSavedPath = '';
      // 以端点回的时刻为锚,把已经录了多久补上(状态本身可能滞后好几秒)
      if (!recStartLocal) {
        recStartLocal = Date.now() - (Number(s.elapsed_s) || 0) * 1000;
      }
    } else {
      recStartLocal = 0;
      // 完全以节点为准:节点说没有 out_path 就是没有 —— 否则页面开久了、
      // 节点重启过之后,会拿上一轮的路径配上这一轮的"0 点",显示成
      // 「已保存 xxx.pcd · 0 点」这种自相矛盾的话
      recSavedPath = String(s.out_path || '');
    }
    recStaleWarned = false;
    tickRecording();
  });

  // 点大小滑杆(拖动即时生效,刷新后保留)
  if (els.sizeRange) {
    els.sizeRange.min = String(POINT_SIZE_MIN);
    els.sizeRange.max = String(POINT_SIZE_MAX);
    els.sizeRange.step = '0.01';
    let saved = POINT_SIZE_DEFAULT;
    try { saved = parseFloat(localStorage.getItem(POINT_SIZE_KEY)) || POINT_SIZE_DEFAULT; } catch { /* 忽略 */ }
    els.sizeRange.value = String(saved);
    els.sizeRange.addEventListener('input', () => applyPointSize(els.sizeRange.value));
    applyPointSize(saved);
  }

  addLidarFrameListener(onLidarFrame);
  addLidarImuListener(onLidarImu);
  addStatusListener((s) => {
    const online = s.connectionStatus === 'online';
    if (online !== rosOnline) {
      rosOnline = online;
      applyDesired();
    }
  });

  applyDesired();
  if (flushTimer) clearInterval(flushTimer);
  flushTimer = setInterval(flushDom, 125);
  if (recTimer) clearInterval(recTimer);
  recTimer = setInterval(tickRecording, 500);
}
