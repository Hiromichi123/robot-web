/**
 * 地图管理页签：列出板子上的 pcd、加载到 3D 视图、删除。
 *
 * 数据从哪来：浏览器读不到板子的文件系统，列表/删除/读取都走 webserver.py 的
 * `/api/maps` 接口。接口和页面同源（都是 8080），所以不用处理 CORS。
 *
 * 为什么自带一个 3D 视图，而不是复用雷达页签那个：
 *   雷达页签那套是「实时流 + 体素累加」的状态机，且订阅跟录制开关绑定；
 *   把静态地图塞进去要跟它抢显示对象和开关状态。分开之后两边互不干扰，
 *   还能一边开着旧图对照、一边继续建新图。
 *
 * pcd 解析：只认 `DATA binary`（我们自己写的就是 binary，ascii 会明确报错
 * 而不是显示一坨错位的点）。字段按头里的 FIELDS/SIZE/TYPE/COUNT 泛化解析，
 * 不假设布局 —— 万一以后换成别的字段顺序也不用改这里。
 *
 * 单导入方模块（仅 main.js 引用），可独立 ?v= 版本号。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

let viewVisible = false;
let els = {};

// ── Three.js ─────────────────────────────────────────────
let renderer = null;
let scene = null;
let camera3d = null;
let controls = null;
let dataGroup = null;
let pointsObj = null;
let pointsGeo = null;
let mat = null;
let rafId = 0;

const POINT_SIZE_KEY = 'web_sim_map_point_size';
const POINT_SIZE_DEFAULT = 0.05;
let pointSize = POINT_SIZE_DEFAULT;
let colorMode = 'height';        // 'height' | 'intensity'

/** 当前加载的地图 {name, count, positions, intensity} */
let loaded = null;
/** 列表数据（来自 /api/maps） */
let maps = [];

function fmtBytes(n) {
  const v = Number(n) || 0;
  if (v >= 1 << 20) return `${(v / (1 << 20)).toFixed(1)} MB`;
  if (v >= 1 << 10) return `${(v / 1024).toFixed(0)} KB`;
  return `${v} B`;
}
function fmtPoints(n) {
  const v = Number(n) || 0;
  return v >= 10000 ? `${(v / 10000).toFixed(1)} 万点` : `${v} 点`;
}
function fmtTime(sec) {
  if (!Number.isFinite(sec)) return '-';
  const d = new Date(sec * 1000);
  const p = (x) => String(x).padStart(2, '0');
  return `${d.getMonth() + 1}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

// ── pcd 解析 ─────────────────────────────────────────────
/**
 * binary pcd 的 ArrayBuffer → {positions:Float32Array(n*3), intensity:Float32Array(n), count}
 * 失败时抛带中文原因的 Error（页面上直接显示这句）。
 */
export function parsePCDBinary(buf) {
  const bytes = new Uint8Array(buf);
  // 头一定是纯 ASCII，用 latin1 解（1 字符 = 1 字节，下标即字节偏移）
  const head = new TextDecoder('latin1').decode(bytes.subarray(0, Math.min(bytes.length, 16384)));
  const dataLine = /^[ \t]*DATA[ \t]+(\S+)/im.exec(head);
  if (!dataLine) throw new Error('文件头里没有 DATA 行，不是有效的 pcd');
  if (!/^binary/i.test(dataLine[1])) {
    throw new Error(`只支持 DATA binary 的 pcd（这个是 ${dataLine[1]}）`);
  }
  const bodyStart = head.indexOf('\n', dataLine.index) + 1;
  const header = head.slice(0, dataLine.index);
  const field = (key) => {
    const m = new RegExp(`^[ \\t]*${key}[ \\t]+(.+)$`, 'im').exec(header);
    return m ? m[1].trim().split(/\s+/) : null;
  };

  const names = field('FIELDS');
  const sizes = (field('SIZE') || []).map(Number);
  const types = (field('TYPE') || []);
  const counts = (field('COUNT') || names?.map(() => '1') || []).map(Number);
  const nDeclared = parseInt((field('POINTS') || field('WIDTH') || ['0'])[0], 10);
  if (!names || !sizes.length || !types.length) throw new Error('pcd 头缺少 FIELDS/SIZE/TYPE');
  if (!nDeclared) throw new Error('pcd 头里 POINTS/WIDTH 为 0');

  // 每个字段的字节偏移 + 每点字节数
  const offsets = {};
  let step = 0;
  for (let i = 0; i < names.length; i++) {
    offsets[names[i]] = step;
    step += (sizes[i] || 4) * (counts[i] || 1);
  }
  for (const need of ['x', 'y', 'z']) {
    if (!(need in offsets)) throw new Error(`pcd 里没有 ${need} 字段`);
  }

  const n = Math.min(nDeclared, Math.floor((bytes.length - bodyStart) / step));
  if (n <= 0) throw new Error('pcd 数据体是空的（文件被截断？）');

  const dv = new DataView(bytes.buffer, bytes.byteOffset + bodyStart, n * step);
  /** 读第 base 字节处（某个点的起点）的某个字段。base 必须带上，否则每点读的都是第 0 个点。 */
  const numAt = (base, fieldName) => {
    const i = names.indexOf(fieldName);
    if (i < 0) return 0;
    const off = base + offsets[fieldName];
    const t = types[i];
    const sz = sizes[i] || 4;
    if (t === 'F') return sz === 8 ? dv.getFloat64(off, true) : dv.getFloat32(off, true);
    if (t === 'U') return sz === 2 ? dv.getUint16(off, true) : dv.getUint8(off);
    if (t === 'I') return sz === 2 ? dv.getInt16(off, true) : dv.getInt8(off);
    return 0;
  };

  const positions = new Float32Array(n * 3);
  const intensity = new Float32Array(n);
  const rgb = new Float32Array(n).fill(NaN);
  const hasIntensity = 'intensity' in offsets;
  const hasRgb = 'rgb' in offsets || 'rgba' in offsets;
  const rgbKey = 'rgb' in offsets ? 'rgb' : 'rgba';
  let valid = 0;
  let colored = 0;
  for (let i = 0; i < n; i++) {
    const base = i * step;
    const x = numAt(base, 'x');
    const y = numAt(base, 'y');
    const z = numAt(base, 'z');
    if (!Number.isFinite(x + y + z)) continue;
    const o = valid * 3;
    positions[o] = x; positions[o + 1] = y; positions[o + 2] = z;
    if (hasIntensity) intensity[valid] = numAt(base, 'intensity');
    // 无色的点在文件里存的是 NaN(见 recorder 的 pcd_io)，读出来保持 NaN
    if (hasRgb) {
      const c = numAt(base, rgbKey);
      if (Number.isFinite(c)) { rgb[valid] = c; colored++; }
    }
    valid++;
  }
  if (!valid) throw new Error('pcd 里没有一个有效点');
  return {
    positions: positions.slice(0, valid * 3),
    intensity: intensity.slice(0, valid),
    rgb: rgb.slice(0, valid),
    colored,
    count: valid,
  };
}

// ── 着色 ─────────────────────────────────────────────────
const _c = new THREE.Color();
function heightColor(z, out, i) {
  const t = THREE.MathUtils.clamp((z + 2) / 5, 0, 1);
  _c.setHSL((1 - t) * 0.66, 0.85, 0.55);
  out[i] = _c.r; out[i + 1] = _c.g; out[i + 2] = _c.b;
}
function intensityColor(v, out, i) {
  const g = THREE.MathUtils.clamp(v / 255, 0.04, 1);
  out[i] = g * 0.85; out[i + 1] = g * 0.95; out[i + 2] = g;
}
/** packed float32(0x00RRGGBB) → RGB 分量(0~1) */
function unpackRgb(packed, out, i) {
  const u = new Uint32Array(new Float32Array([packed]).buffer)[0];
  out[i] = ((u >> 16) & 0xFF) / 255;
  out[i + 1] = ((u >> 8) & 0xFF) / 255;
  out[i + 2] = (u & 0xFF) / 255;
}
/** '真彩' 模式下没有颜色的点回落到高度着色 —— 相机没扫到的地方不能是一片黑 */
function colorPoint(mode, z, inten, packed, out, i) {
  if (mode === 'rgb') {
    if (Number.isFinite(packed)) { unpackRgb(packed, out, i); return; }
    heightColor(z, out, i);
    return;
  }
  if (mode === 'intensity') intensityColor(inten, out, i);
  else heightColor(z, out, i);
}
function recolor() {
  if (!loaded || !pointsGeo) return;
  const colors = new Float32Array(loaded.count * 3);
  for (let i = 0; i < loaded.count; i++) {
    colorPoint(colorMode, loaded.positions[i * 3 + 2], loaded.intensity[i],
               loaded.rgb[i], colors, i * 3);
  }
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
}

// ── 场景 ─────────────────────────────────────────────────
function initThree() {
  const canvas = els.canvas;
  renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);

  scene = new THREE.Scene();
  camera3d = new THREE.PerspectiveCamera(60, 1, 0.05, 500);
  camera3d.position.set(8, -8, 6);
  camera3d.up.set(0, 0, 1); // z-up(世界系=ROS 坐标,同 scene.js 的机器人视图)

  controls = new OrbitControls(camera3d, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.maxDistance = 200;

  // 世界系是 z-up(Point-LIO camera_init):数据组不旋转,世界坐标=ROS 坐标
  // (红=x 绿=y 蓝=z),取景数学因此可以直接用点云坐标。
  dataGroup = new THREE.Group();
  scene.add(dataGroup);

  // 网格铺在 XY 平面(=地图的 xy 地面,红绿轴所在平面):
  // GridHelper 默认躺在自己的 XZ 平面,绕 X 转 90° 才转过来。
  const grid = new THREE.GridHelper(60, 60, 0x2a4458, 0x18262f);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.002; // 抬一点,避免与贴地点的 z-fighting
  grid.material.transparent = true;
  grid.material.opacity = 0.5;
  grid.material.depthWrite = false;
  dataGroup.add(grid);
  dataGroup.add(new THREE.AxesHelper(1.5));

  pointsGeo = new THREE.BufferGeometry();
  pointsGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3), 3));
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3), 3));
  pointsGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1000);
  mat = new THREE.PointsMaterial({
    size: pointSize, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.95, depthWrite: false,
  });
  pointsObj = new THREE.Points(pointsGeo, mat);
  pointsObj.frustumCulled = false;
  pointsObj.visible = false;
  dataGroup.add(pointsObj);

  resizeRenderer();
  if (typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(() => resizeRenderer()).observe(canvas.parentElement);
  }
  window.addEventListener('resize', resizeRenderer);

  const loop = () => {
    rafId = requestAnimationFrame(loop);
    if (!viewVisible) return;
    controls.update();
    renderer.render(scene, camera3d);
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

/** 把「看看这张图」的相机摆到能装下整张地图的位置 */
function frameCamera() {
  if (!loaded || !loaded.count) return;
  const p = loaded.positions;
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < loaded.count; i++) {
    const x = p[i * 3], y = p[i * 3 + 1], z = p[i * 3 + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, cz = (minZ + maxZ) / 2;
  const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 2);
  controls.target.set(cx, cy, cz);
  // 世界坐标=ROS 坐标(z-up),相机直接放在地图中心的斜上方(z 抬 0.9 span)取全景
  camera3d.position.set(cx + span * 0.8, cy - span * 1.1, cz + span * 0.9);
  controls.update();
  // 给"俯视"按钮留着用:目标点 + 地图跨度
  loaded.center = { x: cx, y: cy, z: cz };
  loaded.span = span;
  return { minX, maxX, minY, maxY, minZ, maxZ };
}

// ── 列表与操作 ───────────────────────────────────────────
function setStateText(text, live = false) {
  if (!els.viewState) return;
  els.viewState.textContent = text;
  els.viewState.classList.toggle('is-live', !!live);
}

function renderList() {
  if (!els.list) return;
  if (!maps.length) {
    els.list.innerHTML = '<li class="maps-empty">还没有地图。到「雷达」页签点「开始建图」录一张。</li>';
    if (els.count) els.count.textContent = '0 个文件';
    return;
  }
  if (els.count) els.count.textContent = `${maps.length} 个文件`;
  els.list.innerHTML = maps.map((m) => {
    const on = loaded && loaded.name === m.name;
    const meta = [
      m.points != null ? fmtPoints(m.points) : null,
      fmtBytes(m.size),
      m.elapsed_s != null ? `${Number(m.elapsed_s).toFixed(0)}s` : null,
      fmtTime(m.mtime),
    ].filter(Boolean).join(' · ');
    return `<li class="maps-item${on ? ' is-loaded' : ''}" data-name="${m.name}">
      <div class="maps-info">
        <span class="maps-name">${m.name}</span>
        <span class="maps-meta">${meta}</span>
      </div>
      <div class="maps-acts">
        <button type="button" class="lidar-btn" data-act="load">${on ? '已加载' : '加载'}</button>
        <button type="button" class="lidar-btn maps-del" data-act="del">删除</button>
      </div>
    </li>`;
  }).join('');
}

async function refreshList() {
  setStateText('读取列表…');
  try {
    const r = await fetch('/api/maps', { cache: 'no-store' });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const j = await r.json();
    maps = j.maps || [];
    renderList();
    setStateText(loaded ? `已加载 ${loaded.name}` : '未加载地图', !!loaded);
  } catch (err) {
    if (els.list) {
      els.list.innerHTML = `<li class="maps-empty">读取失败：${err.message}</li>`;
    }
    setStateText('列表读取失败');
  }
}

async function loadMap(name) {
  setStateText(`加载 ${name}…`);
  if (els.placeholder) els.placeholder.textContent = `正在下载并解析 ${name}…`;
  try {
    const r = await fetch(`/api/maps/${encodeURIComponent(name)}`, { cache: 'no-store' });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const buf = await r.arrayBuffer();
    setStateText(`${name} 解析中…`);
    const parsed = parsePCDBinary(buf);
    loaded = { name, ...parsed };

    pointsGeo.setAttribute('position', new THREE.BufferAttribute(parsed.positions, 3));
    recolor();
    pointsGeo.computeBoundingSphere();
    pointsObj.visible = true;
    const box = frameCamera();
    if (els.placeholder) els.placeholder.style.display = 'none';
    if (els.info) {
      const span = box ? ` · 跨度 ${(box.maxX - box.minX).toFixed(1)}×${(box.maxY - box.minY).toFixed(1)}`
                       + `×${(box.maxZ - box.minZ).toFixed(1)} m` : '';
      const colorNote = parsed.colored ? ` · 彩 ${fmtPoints(parsed.colored)}` : '';
      els.info.textContent = `${name} · ${fmtPoints(parsed.count)}${colorNote}${span}`;
      els.info.title = name;
    }
    setStateText(`已加载 ${name}`, true);
    renderList();
    resizeRenderer();
  } catch (err) {
    setStateText(`加载失败：${err.message}`);
    if (els.placeholder) els.placeholder.textContent = `加载失败：${err.message}`;
  }
}

function clearLoaded() {
  loaded = null;
  if (pointsObj) pointsObj.visible = false;
  if (els.placeholder) {
    els.placeholder.style.display = '';
    els.placeholder.textContent = '左侧列表里点「加载」把某张 pcd 显示到这里\n拖动旋转 · 滚轮缩放';
  }
  if (els.info) els.info.textContent = '未加载地图';
  setStateText('未加载地图');
  renderList();
}

async function deleteMap(name) {
  if (!window.confirm(`删除 ${name}？\n（连同同名的 .json 边车一起删，不可恢复）`)) return;
  setStateText(`删除 ${name}…`);
  try {
    const r = await fetch(`/api/maps/${encodeURIComponent(name)}`, { method: 'DELETE' });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) throw new Error(j.error || `HTTP ${r.status}`);
    if (loaded && loaded.name === name) clearLoaded();
    await refreshList();
  } catch (err) {
    setStateText(`删除失败：${err.message}`);
  }
}

function applyPointSize(v) {
  pointSize = THREE.MathUtils.clamp(+v || POINT_SIZE_DEFAULT, 0.01, 0.30);
  if (mat) mat.size = pointSize;
  if (els.sizeVal) els.sizeVal.textContent = pointSize.toFixed(2);
  try { localStorage.setItem(POINT_SIZE_KEY, String(pointSize)); } catch { /* 忽略 */ }
}

// ── 对外 ─────────────────────────────────────────────────
export function setMapViewVisible(visible) {
  viewVisible = !!visible;
  if (!viewVisible) return;
  resizeRenderer();
  // 每次切回来都对一遍列表:期间可能又录了新图、或别人删了文件
  refreshList();
}

export function initMapManager() {
  els = {
    canvas: document.querySelector('#maps-3d-canvas'),
    placeholder: document.querySelector('#maps-placeholder'),
    viewState: document.querySelector('#maps-view-state'),
    info: document.querySelector('#maps-info'),
    list: document.querySelector('#maps-list'),
    count: document.querySelector('#maps-count'),
    btnRefresh: document.querySelector('#maps-btn-refresh'),
    btnReset: document.querySelector('#maps-btn-reset'),
    btnTop: document.querySelector('#maps-btn-top'),
    btnColor: document.querySelector('#maps-btn-color'),
    btnClear: document.querySelector('#maps-btn-clear'),
    sizeRange: document.querySelector('#maps-point-size'),
    sizeVal: document.querySelector('#maps-point-size-val'),
  };
  if (!els.canvas) return;

  initThree();

  const setView = (x, y, z) => {
    camera3d.position.set(x, y, z);
    controls.update();
  };
  els.btnReset?.addEventListener('click', () => (loaded ? frameCamera() : setView(8, -8, 6)));
  // 俯视:相机挪到**当前目标点**正上方(否则地图不在原点时视野跑偏),
  // y 用 -0.02 微偏移而不是精确 0 —— 视线与 camera.up(0,0,1) 平行会退化。
  // 屏幕上方=+y(绿)、右侧=+x(红),就是"地图上的正俯视平面图"。
  els.btnTop?.addEventListener('click', () => {
    const c = loaded?.center ?? { x: 0, y: 0, z: 0 };
    const dist = loaded?.span ? Math.max(10, loaded.span * 1.2) : 20;
    controls.target.set(c.x, c.y, c.z);
    setView(c.x, c.y - 0.02, c.z + dist);
  });
  // 三档循环:高度 → 反射率 → 真彩(有颜色的点用真彩,没有的回落高度)
  els.btnColor?.addEventListener('click', () => {
    colorMode = colorMode === 'height' ? 'intensity'
      : (colorMode === 'intensity' ? 'rgb' : 'height');
    els.btnColor.textContent = colorMode === 'height' ? '着色:高度'
      : (colorMode === 'intensity' ? '着色:反射率' : '着色:真彩');
    els.btnColor.classList.toggle('is-active', colorMode === 'rgb');
    recolor();
  });
  els.btnClear?.addEventListener('click', clearLoaded);
  els.btnRefresh?.addEventListener('click', refreshList);

  if (els.sizeRange) {
    els.sizeRange.min = '0.01';
    els.sizeRange.max = '0.30';
    els.sizeRange.step = '0.01';
    let saved = POINT_SIZE_DEFAULT;
    try { saved = parseFloat(localStorage.getItem(POINT_SIZE_KEY)) || POINT_SIZE_DEFAULT; } catch { /* 忽略 */ }
    els.sizeRange.value = String(saved);
    els.sizeRange.addEventListener('input', () => applyPointSize(els.sizeRange.value));
    applyPointSize(saved);
  }

  // 列表用事件委托:条目是 innerHTML 重绘的,逐个绑定会随刷新丢失
  els.list?.addEventListener('click', (ev) => {
    const btn = ev.target.closest('button[data-act]');
    if (!btn) return;
    const name = btn.closest('.maps-item')?.dataset?.name;
    if (!name) return;
    if (btn.dataset.act === 'load') loadMap(name);
    else if (btn.dataset.act === 'del') deleteMap(name);
  });
}
