/**
 * 建图页签模块 + 建图开关。
 *
 * 点云: /lidar_recorder/frame  sensor_msgs/msg/PointCloud2
 *      **只在建图开关打开时才有**。来源是板上 lidar_recorder 节点,它把源点云按
 *      /aft_mapped_to_init 的位姿变换到世界系,和节点落盘的 pcd 是同一批点。
 *      **源是哪一路由板上 launch 的 map_source 决定**(2026-10-07 换向):
 *        camera(默认) /camera/camera/depth/color/points —— 相机双目的彩色点云,
 *                      约 1.3 万点/帧、5Hz。几何和颜色同源,**不依赖相机-雷达外参**。
 *        lidar(旧)     /cloud_registered_body —— 雷达点云 + 相机投影补色。
 *      前端两条都能显示:解码器按 msg.fields 泛化解析,存在 rgb 就是彩点。
 *
 * 点云预览(2026-10-03 起默认开): 未建图时也能实时看 —— 直接订**几何源那一路**
 *      (相机路线 = /camera/camera/depth/color/points,Point-LIO 常发的那路在旧路线
 *      里是 /cloud_registered_body),用最近的
 *      /aft_mapped_to_init 位姿在**浏览器本地**搬到世界系。录制节点完全不参与,
 *      **不写任何文件**;点「开始建图」后画面切回 /lidar_recorder/frame。
 *      累加建图**默认开**(2026-10-06 改回;见下方「真彩融合」——真彩必须是累积的),
 *      想只看当前帧再点按钮切。
 *      ⚠️ 预览的位姿用的是"最近一条"而不是按帧时间戳配对(板上录制节点才做严格
 *      配对)—— 只求眼看,机器人在动时可能有厘米级错位;正式建图仍以录制流为准。
 *      字段 x,y,z,intensity,rgb 均 float32、point_step=20(rgb 是 PCL 打包位,无色为 NaN);
 *      解码器按 msg.fields 的 offset/datatype 泛化解析,兼容 uint8 intensity
 *      和 Point-LIO 原生那种更宽的 PointXYZINormal 布局。
 *      因为已经是世界系,"累加建图"直接叠就是全局一致的地图(机器人移动也不糊)。
 *
 * 显示约定: 世界系是 z-up(Point-LIO camera_init),相机 up=(0,0,1)、网格铺在
 *      XY 平面(红=x 绿=y 蓝=z)—— 与 scene.js 的机器人视图同一套约定。
 * IMU: /livox/imu  sensor_msgs/msg/Imu, 200Hz 内置 BMI088(orientation 通常为空,姿态由重力估计)。
 *      它不属点云,仍按「雷达页签可见且在线」订阅,不受建图开关影响。
 * 位姿 HUD: /aft_mapped_to_init  nav_msgs/msg/Odometry(Point-LIO 世界系,~10Hz)。
 *      画布左上大字显示 X/Y/Z/YAW/刷新率,**原频率不节流**;2s 没数据置灰。
 *      与内置 IMU 一样按「雷达页签可见且在线」订阅。
 *
 * 真彩融合(2026-10-06): **视频和点云都下到浏览器,只在前端融合** —— 板上那份
 *      (lidar_recorder._color_map_pass,只在建图时跑)照旧共存,最终要由这一份取代。
 *      浏览器把**累积地图里还没有颜色的点**投影进彩色相机画面取色,
 *      颜色"烤"进点里,攒够了可以「保存地图到板上」(POST /api/maps/save)。
 *      为什么必须靠转动:相机装在雷达正前方 4cm,挡住了雷达正前方 105° 扇区,
 *      相机 FOV ±41.5° 整个落在盲区里 —— **同一时刻两者看不到同一片区域**
 *      (板上节点实测 0/26967 个点进画面)。所以静止时颜色不会增加,转一圈才长出来。
 *      外参(车体→相机光学系)优先从 /tf_static 现场链出来(和 launch 单一来源);
 *      收不到就退回内置常量,并在小窗上标出"内置默认" —— 绝不静默用错的。
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
  addStatusListener, getRosClient,
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

// ── SLAM 位姿 HUD(/aft_mapped_to_init) ────────────────────
// 不经过 ros-bridge.js —— 动它要级联 bump 全部 9 处 import 的 ?v=,没必要。
// 照 mission-view.js 的写法:借连接(getRosClient)自建话题。生命周期与内置
// IMU 一致:雷达页签可见 ∧ ROS 在线;每条消息都刷(原频率,不节流)。
const SLAM_ODOM_TOPIC = '/aft_mapped_to_init';
const POSE_STALE_MS = 2000;   // 超这么久没新消息 → HUD 置灰(值保留,提示非实时)
const POSE_RATE_N = 20;       // 刷新率用最近 N 条的时间戳算
let odomSub = null;
let poseLastAt = 0;           // 最近一条消息的 performance.now() 时刻
const poseTimes = [];
/** 最近一帧位姿 {p:[x,y,z], q:[x,y,z,w]} —— 点云预览的 body→世界变换用它 */
let poseLatest = null;

// ── 点云预览 ─────────────────────────────────────────────
// 未建图时的实时画面:直接订**几何源那一路**的点云,本地乘位姿到世界系。
// 板上录制节点不参与、不落盘(见文件头「点云预览」)。录制中不订它(切换见
// rec 状态监听与 applyDesired)。
// **2026-10-07 换成相机路线的点云**:建图几何源已经是相机的双目点云
// (slam_only.launch.py 的 map_source:=camera 默认),预览再订雷达的
// /cloud_registered_body 就会和录制时的点云来源/密度对不上 —— 看着像两个东西。
const LIDAR_BODY_TOPIC = '/camera/camera/depth/color/points';
let previewSub = null;

/**
 * 传感器流订阅的 QoS:Point-LIO / livox 这些话题是 sensor_data(best_effort),
 * 而 roslib 不写 qos 时默认 reliable —— **对 best_effort 发布端会静默收不到**。
 * best_effort 订阅与两种发布端都兼容(reliable 发布端可以供给 best_effort 订阅,
 * 反之不行),所以 sensor 类话题统一用它。写法同 ros-bridge.js 的
 * _subscribeWithQos(那边的私有函数,这里用不了)。
 */
const SENSOR_QOS = { reliability: 'best_effort', durability: 'volatile', history: 'keep_last', depth: 2 };

/**
 * `/tf_static` 专用的 QoS:**必须 transient_local**,否则收不到 static_transform_publisher
 * 在我们连上来之前就锁存好的那几条(latch 语义:晚到的订阅者补发最后一条)。
 * 用默认的 volatile 会一条都收不到,而且**不报错** —— 表现为"外参链不出来"。
 */
const STATIC_TF_QOS = { reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 100 };

function subscribeWithQos(topic, callback, qos = SENSOR_QOS) {
  const orig = topic.callForSubscribeAndAdvertise.bind(topic);
  topic.callForSubscribeAndAdvertise = (msg) => {
    msg.qos = qos;
    orig(msg);
  };
  topic.subscribe(callback);
}

// ── 前端融合:相机给累积地图上色(2026-10-06)────────────────────
// 详见文件头「真彩融合」。板上那份投影取色(lidar_recorder._color_map_pass)照旧跑,
// 两边共存;这里这一份是给"雷达页实时真彩 + 存回板上"用的。
// 不经过 ros-bridge.js(动它要级联 bump 9 处 import 的 ?v=),照位姿 HUD 的写法自建话题。
const COLOR_IMG_TOPIC = '/camera/camera/color/image_rect_raw/compressed';
const COLOR_INFO_TOPIC = '/camera/camera/color/camera_info';
const TF_STATIC_TOPIC = '/tf_static';
const COLOR_PERIOD_MS = 200;       // 补色轮询周期(5Hz)—— 与板上那份同频
const COLOR_CHUNK = 40000;         // 每遍扫多少点(逐点 JS 循环,别一次把主线程占住)
const COLOR_IMG_STALE_MS = 600;    // 画面这么久没更新就这一遍不涂(宁可不涂也别涂错)
const COLOR_MAX_HITS_DRAW = 2500;  // 小窗上最多画几个命中点
const EXTRINSIC_FALLBACK_MS = 5000;// 等 /tf_static 的时限,超了退回内置常量
const BASE_FRAME = 'base_link';
const CAM_FRAME = 'camera_camera_rgb';
const BODY_FRAME = 'body';

let colorImgSub = null;            // 彩色画面(CompressedImage)
let colorInfoSub = null;           // 彩色内参(CameraInfo)
let tfStaticSub = null;            // 静态 TF(外参就在里面)
let colorInfo = null;              // {fx,fy,cx,cy,w,h}
let colorImgAt = 0;                // 最近一条画面的时刻(performance.now)
let colorBitmap = null;            // 最近解好的 ImageBitmap
let colorData = null;              // 它的像素(ImageData);换图后置 null 重取
let colorCv = null, colorCvCtx = null;
let colorDecoding = false;         // 上一张还在解 → 丢掉新来的,别排队
let colorCursor = 0;               // 补色游标(轮着扫累积地图)
let colorFilled = 0;               // 前端累计补上的点数
let colorPasses = 0;
let colorHits = [];                // 本遍命中的像素坐标(u,v 交替)——小窗叠点用
let colorOverlay = false;          // 小窗是否叠画命中点(标定外参的判据)
let camFrames = new Map();         // 静态 TF:child → [{parent, m}]（**一个 child 可能有多个父**)
let baseFromCam = null;            // THREE.Matrix4:base_link ← 相机光学系
let baseFromBody = null;           // THREE.Matrix4:base_link ← body
let extrinsicSrc = '';             // 'tf' | 'builtin'
let extrinsicSince = 0;            // 开始等 /tf_static 的时刻
let saving = false;                // 「保存地图到板上」正在发(防重复点)

/** RPY(弧度) → 四元数 [x,y,z,w]。
 *  **和 launch 里 quat_from_rpy 是同一套公式** —— 改这里要对着
 *  core_2026/launch/slam_only.launch.py 一起改,否则兜底外参和板上不一致。 */
function quatFromRpy(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy,
          cr * cp * cy + sr * sp * sy];
}

/** xyz + 四元数 → 4x4(parent ← child)。 */
function matrixFromXyzQuat(xyz, quat) {
  const m = new THREE.Matrix4().makeRotationFromQuaternion(
    new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]));
  m.setPosition(xyz[0], xyz[1], xyz[2]);
  return m;
}

// 内置兜底外参:抄 slam_only.launch.py 的 cam_xyz_quat / cam_opt_rpy,
// 以及 mount_xyz_rpy(雷达安装变换,用来把 body 接回 base_link)。
// ⚠️ cam_xyz_quat 的旋转**目前还没标定**(单位四元数 = 假定相机与车体同朝向),
// 颜色对不对完全取决于它;小窗"叠加命中点"就是给它做标定用的。
const BUILTIN_CAM_XYZ_QUAT = [0.326, 0, 0.095, 0, 0, 0, 1];
const BUILTIN_CAM_OPT_RPY = [-Math.PI / 2, 0, -Math.PI / 2];
const BUILTIN_MOUNT_XYZ_QUAT = [0.286, 0, 0, 0, -0.923880, 0, 0.382683];

function builtinExtrinsic() {
  return matrixFromXyzQuat(BUILTIN_CAM_XYZ_QUAT.slice(0, 3),
                           BUILTIN_CAM_XYZ_QUAT.slice(3))
    .multiply(matrixFromXyzQuat([0, 0, 0], quatFromRpy(...BUILTIN_CAM_OPT_RPY)));
}

function builtinBodyMount() {
  return matrixFromXyzQuat(BUILTIN_MOUNT_XYZ_QUAT.slice(0, 3),
                           BUILTIN_MOUNT_XYZ_QUAT.slice(3));
}

// ── 累加建图:Livox 是非重复扫描,单帧只覆盖场景的一小部分,
//    多帧持续接收并在固定坐标系重叠 → 体素去重后逐步还原完整场景。
//    静止(匀速)采集前提;机器人移动后需配合位姿(里程计)做坐标变换。
const ACC_VOXEL = 0.05;  // 体素边长(m):同一 5cm 立方体内只保留首个点
const ACC_MAX = 600000;  // 最大体素点数(position/color 缓冲按此预分配)
const ACC_KEY_BASE = 4096;
const ACC_KEY_OFFSET = 2048; // 覆盖 ±102m(voxel 坐标 0..4095)
// 默认**开**(2026-10-06 改回):真彩是"转动机器人、相机依次扫过雷达建好的面"
// 逐步烤进累积地图的,只显示当前帧永远不会有颜色(单帧里相机与雷达看不到同一片
// 区域)。想只看当前帧再点按钮切。
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
  camera3d.up.set(0, 0, 1); // z-up(世界系=ROS 坐标,同 scene.js 的机器人视图)

  controls = new OrbitControls(camera3d, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.minDistance = 0.3;
  controls.maxDistance = 80;
  controls.target.set(0, 0, 0);

  // 世界系是 z-up(Point-LIO camera_init):数据组不旋转,世界坐标=ROS 坐标
  // (红=x 绿=y 蓝=z)。dataGroup 仍保留 —— 自动旋转就是转它的 z(竖直轴)。
  dataGroup = new THREE.Group();
  scene.add(dataGroup);

  // 地面网格(传感器原点为中心)。GridHelper 默认躺在自己的 XZ 平面,
  // 绕 X 转 90° 才落进 XY 平面(=地图的 xy 地面,红绿轴所在平面)。
  const grid = new THREE.GridHelper(40, 40, 0x2a4458, 0x18262f);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.002; // 抬一点,避免与贴地点的 z-fighting
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

let lastCssW = 0, lastCssH = 0;
function resizeRenderer() {
  if (!renderer || !els.canvas) return;
  const w = els.canvas.clientWidth || 0;
  const h = els.canvas.clientHeight || 0;
  if (!w || !h) return;
  // 尺寸没变就直接返回:applyDesired() 现在会被建图状态监听(2Hz)调到,
  // setSize 每次都重设 canvas 的 width/height 属性 → 白白重建 GPU 缓冲
  if (w === lastCssW && h === lastCssH) return;
  lastCssW = w; lastCssH = h;
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
  // 融合那一路的游标/计数也要跟着归零,否则新地图会从旧游标处接着扫
  colorCursor = 0;
  colorFilled = 0;
  colorPasses = 0;
  colorHits = [];
  setSaveNote('');
  updateColorHud();
}

/** 单帧/累加显示切换 */
function setAccumulateMode(on) {
  accumulate = !!on;
  if (accObj) accObj.visible = accumulate;
  if (pointsObj) pointsObj.visible = !accumulate;
  if (accumulate) recolorAccumulatedAll();   // 累积期间被补上的颜色要显示出来
}

// ── 前端融合:相机给累积地图上色 ────────────────────────────
// 对应板上 lidar_recorder._color_map_pass 的浏览器版。规则刻意保持一致:
// 游标轮着扫、**只补不覆盖**(已有的颜色不重涂,否则每遍都要把可见的点全推给 GPU)。

function tfMatrix(tf) {
  const t = tf?.translation || {}, r = tf?.rotation || {};
  return matrixFromXyzQuat(
    [Number(t.x) || 0, Number(t.y) || 0, Number(t.z) || 0],
    [Number(r.x) || 0, Number(r.y) || 0, Number(r.z) || 0, Number(r.w) || 1]);
}

function onTfStatic(msg) {
  let changed = false;
  for (const t of msg.transforms || []) {
    const parent = String(t.header?.frame_id || '').replace(/^\//, '');
    const child = String(t.child_frame_id || '').replace(/^\//, '');
    if (!parent || !child || parent === child) continue;
    const list = camFrames.get(child) || [];
    const m = tfMatrix(t.transform);
    const prev = list.find((e) => e.parent === parent);
    // **同名父子对直接用新值**:链路重启/改了安装变换后，同名 TF 会重新发一遍，
    // 留着旧矩阵就会拿着过期的外参算颜色（而且看起来一切正常，最难查）。
    if (prev) prev.m = m;
    else list.push({ parent, m });
    camFrames.set(child, list);
    changed = true;
  }
  if (changed) resolveExtrinsic();
}

/**
 * 从静态 TF 里链出 base_link ← 相机光学系,顺带取 base_link ← body。
 *
 * **只认静态 TF**:body 同时被 launch 的安装变换和 Point-LIO 的里程计 TF 发布,
 * 查动态那种 tf2 会挑一个、结果不确定 —— 板上 launch 里同样绕开了这一点。
 *
 * **一个 child 可能有多个父**(实测:`camera_camera_rgb` 既挂在 launch 的
 * `camera_link` 下,又被相机驱动挂在 `camera_camera_left` 下;
 * `camera_camera_left` 同样既是 camera_link 的子、又是 camera_camera_imu 的子)。
 * 按"先到的为准"就是随机结果,所以这里做**最短路径 BFS**:
 * 经 camera_link 到 base_link 是 2 跳,经相机驱动那棵子树是 3 跳(还可能走不通),
 * 自然会选中 launch 那一套 —— 也正是板上 lidar_recorder 查 TF 时用的那条。
 */
function buildChain(leaf, root) {
  const queue = [[leaf, new THREE.Matrix4()]];
  const seen = new Set([leaf]);
  while (queue.length) {
    const [cur, m] = queue.shift();     // (cur ← leaf)
    if (cur === root) return m;
    for (const e of camFrames.get(cur) || []) {
      if (seen.has(e.parent)) continue;
      seen.add(e.parent);
      queue.push([e.parent, e.m.clone().multiply(m)]);   // (parent←cur)·(cur←leaf)
    }
  }
  return null;
}

function resolveExtrinsic() {
  const cam = buildChain(CAM_FRAME, BASE_FRAME);
  if (!cam) return false;
  const body = buildChain(BODY_FRAME, BASE_FRAME);
  baseFromCam = cam;
  if (body) baseFromBody = body;
  if (extrinsicSrc !== 'tf') console.info('[lidar] 外参取自 /tf_static:', BASE_FRAME, '←', CAM_FRAME);
  extrinsicSrc = 'tf';
  return true;
}

/** 等不到 /tf_static 就退回内置常量 —— 并在小窗上标出来,别静默用错的。 */
function fallbackExtrinsicIfStale() {
  if (baseFromCam || !extrinsicSince) return;
  if (performance.now() - extrinsicSince < EXTRINSIC_FALLBACK_MS) return;
  baseFromCam = builtinExtrinsic();
  if (!baseFromBody) baseFromBody = builtinBodyMount();
  extrinsicSrc = 'builtin';
  console.warn('[lidar] 5s 没收到 /tf_static，外参退回内置默认值（旋转尚未标定）');
}

function onColorImg(msg) {
  colorImgAt = performance.now();
  if (colorDecoding) return;               // 解码没跟上就丢,别排队
  const raw = typeof msg.data === 'string' ? b64ToBytes(msg.data) : msg.data;
  if (!raw) return;
  colorDecoding = true;
  createImageBitmap(new Blob([raw], { type: 'image/jpeg' }))
    .then((bmp) => {
      if (colorBitmap) colorBitmap.close();
      colorBitmap = bmp;
      colorData = null;                    // 换图了,像素要重取
      drawColorPreview(colorHits);
    })
    .catch(() => { /* 半截包/坏 JPEG:丢掉这一张 */ })
    .finally(() => { colorDecoding = false; });
}

function onColorInfo(msg) {
  const k = msg && msg.k;
  if (!k) return;
  const fx = Number(k[0]), fy = Number(k[4]), cx = Number(k[2]), cy = Number(k[5]);
  if (!Number.isFinite(fx) || !Number.isFinite(cx)) return;
  colorInfo = { fx, fy, cx, cy, w: msg.width | 0, h: msg.height | 0 };
}

/** 画面的像素(ImageData)。**一遍补色只取一次** —— getImageData 是整个方案里最贵的一步。 */
function colorPixels() {
  if (!colorBitmap) return null;
  if (colorData) return colorData;
  if (!colorCv) {
    colorCv = document.createElement('canvas');
    colorCvCtx = colorCv.getContext('2d', { willReadFrequently: true });
  }
  const w = colorBitmap.width, h = colorBitmap.height;
  if (colorCv.width !== w || colorCv.height !== h) {
    colorCv.width = w; colorCv.height = h;
  }
  colorCvCtx.drawImage(colorBitmap, 0, 0);
  colorData = colorCvCtx.getImageData(0, 0, w, h);
  return colorData;
}

const _packU32 = new Uint32Array(1);
const _packF32 = new Float32Array(_packU32.buffer);
/** r,g,b(0~255) → PCL 那套 packed float32(0x00RRGGBB)。和节点/落盘同一约定,
 *  不一致的话地图上红蓝会互换。 */
function packRgb(r, g, b) {
  _packU32[0] = ((r << 16) | (g << 8) | b) >>> 0;
  return _packF32[0];
}

/** 给累积地图里**还没有颜色的点**补色:投影进当前彩色画面取像素。 */
function colorAccumulatedPass() {
  fallbackExtrinsicIfStale();      // 先判外参:没有点云时也要能把"内置默认"标出来
  if (!accumulate || accCount === 0) return;
  if (!colorInfo || !baseFromCam || !poseLatest) return;
  if (!colorImgAt || performance.now() - colorImgAt > COLOR_IMG_STALE_MS) return;
  const img = colorPixels();
  if (!img) return;

  // 世界 ← 相机 = (相机←base_link) · (base_link←body) · (世界←body)⁻¹
  const m = baseFromCam.clone().invert();          // 相机 ← base_link
  if (baseFromBody) m.multiply(baseFromBody);      // 相机 ← body
  const p = poseLatest.p, q = poseLatest.q;
  const mb = new THREE.Matrix4().compose(
    new THREE.Vector3(p[0], p[1], p[2]),
    new THREE.Quaternion(q[0], q[1], q[2], q[3]),
    new THREE.Vector3(1, 1, 1));                   // 世界 ← body
  m.multiply(mb.invert());                         // 相机 ← 世界

  const e = m.elements;      // THREE 的 Matrix4.elements 是**列主序**
  const a00 = e[0], a01 = e[1], a02 = e[2];
  const a10 = e[4], a11 = e[5], a12 = e[6];
  const a20 = e[8], a21 = e[9], a22 = e[10];
  const tX = e[12], tY = e[13], tZ = e[14];

  const fx = colorInfo.fx, fy = colorInfo.fy, cx = colorInfo.cx, cy = colorInfo.cy;
  const d = img.data, W = img.width, H = img.height;
  const n = Math.min(COLOR_CHUNK, accCount - colorCursor);
  if (n <= 0) { colorCursor = 0; return; }
  const lo = colorCursor, hi = lo + n;
  colorCursor = hi >= accCount ? 0 : hi;   // 扫到底就绕回 0 = 整张地图扫完一轮
  colorPasses++;

  const hits = [];
  let filled = 0;
  for (let i = lo; i < hi; i++) {
    if (Number.isFinite(accRgb[i])) continue;      // 已有颜色:不重涂
    const o = i * 3;
    const x = accPos[o], y = accPos[o + 1], z = accPos[o + 2];
    const cz = a02 * x + a12 * y + a22 * z + tZ;
    if (!(cz > 1e-3)) continue;                    // 相机背后 / 贴太近
    const u = (a00 * x + a10 * y + a20 * z + tX) / cz * fx + cx;
    const v = (a01 * x + a11 * y + a21 * z + tY) / cz * fy + cy;
    if (!(u >= 0 && u < W - 1 && v >= 0 && v < H - 1)) continue;
    const pi = ((v | 0) * W + (u | 0)) * 4;        // ImageData 是 RGBA 四通道
    const packed = packRgb(d[pi], d[pi + 1], d[pi + 2]);
    accRgb[i] = packed;
    coloredCount++;
    colorFilled++;
    colorPoint(colorMode, z, accInten[i], packed, accCol, o);
    recoloredAt.add(i);
    if (hits.length < COLOR_MAX_HITS_DRAW * 2) hits.push(u, v);
    filled++;
  }
  // 原地换色的点也要把颜色推给 GPU(它们不在"新增区间"里,见 insertAccumulated)
  if (recoloredAt.size) {
    const ca = accGeo.getAttribute('color');
    for (const at of recoloredAt) ca.addUpdateRange(at * 3, 3);
    ca.needsUpdate = true;
    recoloredAt.clear();
  }
  colorHits = hits;
  drawColorPreview(hits);
  if (filled) updateColorHud();
}

/** 雷达页角落的小窗:铺相机画面 + 可选把本遍命中的雷达点画上去。
 *  **叠点就是给外参标定用的判据** —— 深度边缘和画面里物体边缘对齐了,颜色才是对的
 *  (板上 ~/calib_overlay.py 用的同一条判据)。 */
function drawColorPreview(hits) {
  const cv = els.colorPreview;
  if (!cv) return;
  const w = cv.clientWidth | 0, h = cv.clientHeight | 0;
  if (!w || !h) return;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  if (!colorBitmap) {
    ctx.fillStyle = 'rgba(12,20,32,0.55)';
    ctx.fillRect(0, 0, w, h);
    return;
  }
  ctx.drawImage(colorBitmap, 0, 0, w, h);
  // 小窗的宽高比跟着真实画面走(实测是 1088x1920 竖构图,写死横的会拉变形
  // ——而变形了就没法用它判断外参对齐不齐了)
  const ar = String(colorBitmap.width / colorBitmap.height);
  if (cv.dataset.ar !== ar) { cv.dataset.ar = ar; cv.style.aspectRatio = ar; }
  if (colorOverlay && hits && hits.length) {
    const sx = w / colorBitmap.width, sy = h / colorBitmap.height;
    ctx.fillStyle = 'rgba(120,255,140,0.9)';
    for (let i = 0; i < hits.length; i += 2) {
      ctx.fillRect(hits[i] * sx - 1, hits[i + 1] * sy - 1, 2, 2);
    }
  }
}

function updateColorHud() {
  if (!els.colorInfo) return;
  const src = extrinsicSrc === 'tf' ? 'TF 外参'
    : (extrinsicSrc === 'builtin' ? '内置外参(未标定)' : '外参未就绪');
  els.colorInfo.textContent = colorInfo
    ? `${src} · 彩 ${fmtPoints(coloredCount)} · 补色 ${colorPasses} 遍`
    : `${src} · 等彩色内参`;
  els.colorInfo.classList.toggle('is-warn', extrinsicSrc === 'builtin');
}

// ── 存回板上(POST /api/maps/save)──────────────────────────
/**
 * 把浏览器里累积(并且已经融合上色)的点云打包成**和 lidar_recorder 落盘完全一样**
 * 的裸格式(20 字节/点 x y z intensity rgb,各 float32,无色写 NaN)发给 webserver
 * —— 那边只负责加 pcd 头落盘,地图管理页读的就是这一种。
 */
async function saveMapToBoard() {
  if (saving) return;
  if (!accCount) { setSaveNote('还没有累积的点:先转动机器人走一圈'); return; }
  saving = true;
  const n = accCount;
  const buf = new ArrayBuffer(n * 20);
  const f = new Float32Array(buf);
  let w = 0;
  for (let i = 0; i < n; i++) {
    const o = i * 3;
    f[w++] = accPos[o]; f[w++] = accPos[o + 1]; f[w++] = accPos[o + 2];
    f[w++] = accInten[i];
    f[w++] = accRgb[i];              // 没颜色的本来就是 NaN
  }
  setSaveNote(`正在存 ${fmtPoints(n)}…`);
  if (els.btnSave) els.btnSave.disabled = true;
  try {
    const qs = new URLSearchParams({
      points: String(n), colored: String(coloredCount), voxel: String(ACC_VOXEL),
    });
    const res = await fetch(`./api/maps/save?${qs}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/octet-stream' },
      body: buf,
    });
    const out = await res.json().catch(() => null);
    if (!res.ok || !out || !out.ok) throw new Error((out && out.error) || `HTTP ${res.status}`);
    setSaveNote(`已存到板上:${out.name}(${fmtPoints(n)},彩 ${fmtPoints(coloredCount)})`);
  } catch (exc) {
    setSaveNote(`存回失败:${(exc && exc.message) || exc}`);
  } finally {
    saving = false;
    if (els.btnSave) els.btnSave.disabled = false;
  }
}

function setSaveNote(text) {
  if (els.saveNote) els.saveNote.textContent = text;
}

/** 开关前端融合那三路订阅(彩色画面/内参/静态 TF)。生命周期与位姿 HUD 一致。 */
function applyColorSubs(on) {
  const ros = getRosClient();
  if (on && ros && !colorImgSub) {
    colorImgSub = new ROSLIB.Topic({
      ros, name: COLOR_IMG_TOPIC, messageType: 'sensor_msgs/msg/CompressedImage',
      compression: 'none', throttle_rate: 100, queue_length: 1,
    });
    subscribeWithQos(colorImgSub, onColorImg);

    colorInfoSub = new ROSLIB.Topic({
      ros, name: COLOR_INFO_TOPIC, messageType: 'sensor_msgs/msg/CameraInfo',
      compression: 'none', throttle_rate: 0, queue_length: 1,
    });
    subscribeWithQos(colorInfoSub, onColorInfo);

    tfStaticSub = new ROSLIB.Topic({
      ros, name: TF_STATIC_TOPIC, messageType: 'tf2_msgs/msg/TFMessage',
      compression: 'none', throttle_rate: 0, queue_length: 100,
    });
    subscribeWithQos(tfStaticSub, onTfStatic, STATIC_TF_QOS);

    extrinsicSince = performance.now();
    console.info('[lidar] 订阅彩色画面/内参/静态 TF(前端融合上色)');
  } else if (!on && colorImgSub) {
    for (const t of [colorImgSub, colorInfoSub, tfStaticSub]) {
      try { t?.unsubscribe(); } catch { /* 忽略 */ }
    }
    colorImgSub = colorInfoSub = tfStaticSub = null;
    extrinsicSince = 0;
    console.info('[lidar] 取消前端融合订阅');
  }
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
  ingestFrame(msg, frame, { preview: false });
}

/** 点云预览:body 系 → 用最近位姿搬到世界系(**浏览器本地**,板上不参与)。 */
function onPreviewCloud(msg) {
  const pose = poseLatest;
  if (!pose) return;   // 还没有位姿,搬不到世界系,先不显示
  const frame = decodePointCloud2(msg);
  if (!frame) return;
  transformFrameToWorld(frame, pose);
  ingestFrame(msg, frame, { preview: true });
}

/** 世界系变换(就地):p_world = q·p_body·q* + t;顺带重算量程统计(与解码器同口径)。 */
function transformFrameToWorld(frame, pose) {
  const [qx, qy, qz, qw] = pose.q;
  const [px, py, pz] = pose.p;
  const pos = frame.positions;
  let maxRange = 0, minZ = Infinity, maxZ = -Infinity;
  for (let o = 0; o < pos.length; o += 3) {
    const x = pos[o], y = pos[o + 1], z = pos[o + 2];
    // v' = v + 2·(q_xyz × v) × ... 标准 Rodrigues: t = 2·(q_xyz × v)
    const tx = 2 * (qy * z - qz * y);
    const ty = 2 * (qz * x - qx * z);
    const tz = 2 * (qx * y - qy * x);
    const wx = x + qw * tx + (qy * tz - qz * ty) + px;
    const wy = y + qw * ty + (qz * tx - qx * tz) + py;
    const wz = z + qw * tz + (qx * ty - qy * tx) + pz;
    pos[o] = wx; pos[o + 1] = wy; pos[o + 2] = wz;
    const r = Math.sqrt(wx * wx + wy * wy + wz * wz);
    if (r > maxRange) maxRange = r;
    if (wz < minZ) minZ = wz;
    if (wz > maxZ) maxZ = wz;
  }
  frame.maxRange = maxRange;
  frame.minZ = minZ === Infinity ? 0 : minZ;
  frame.maxZ = maxZ === -Infinity ? 0 : maxZ;
}

/** 预览流与录制流的公共后半段:显示/累加/统计。 */
function ingestFrame(msg, frame, opts) {
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
  lastFrameId = opts.preview
    ? `预览 ${LIDAR_BODY_TOPIC}`
    : String(msg.header?.frame_id || '-');
  const now = performance.now();
  if (!windowStartedAt) windowStartedAt = now;
  lastFrameAt = now;
  if (viewVisible) els.placeholder.style.display = 'none';
}

/** 开关点云预览订阅(未建图 ∧ 页签可见 ∧ 在线;录制中让位给录制流)。 */
function applyPreviewSub(on) {
  const ros = getRosClient();
  if (on && ros && !previewSub) {
    previewSub = new ROSLIB.Topic({
      ros, name: LIDAR_BODY_TOPIC, messageType: 'sensor_msgs/msg/PointCloud2',
      compression: 'none', throttle_rate: 0, queue_length: 1,
    });
    subscribeWithQos(previewSub, onPreviewCloud);
    console.info('[lidar] 订阅点云预览(不落盘):', LIDAR_BODY_TOPIC);
  } else if (previewSub && (!on || !ros)) {
    try { previewSub.unsubscribe(); } catch { /* 忽略 */ }
    previewSub = null;
    console.info('[lidar] 取消点云预览订阅');
  }
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
/** 前端融合补色的定时器(5Hz;和板上那份 _color_map_pass 同频) */
let colorTimer = null;
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
  updateColorHud();

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
  if (!accCount) return '等点云(未建图时只看实时点云,不落盘)';
  // 未建图:报实时累积 + 彩色点比例。相机路线下点云自带 rgb,一般上一帧就全彩,
  // 所以不再有"转动机器人扫一圈才上色"那回事(那是雷达路线+投影补色时代的说法)。
  const colorNote = coloredCount
    ? ` · 彩 ${fmtPoints(coloredCount)}`
    : ' · 等彩色点';
  return `实时累积 ${fmtPoints(accCount)}${colorNote}`;
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
// ── SLAM 位姿订阅(与内置 IMU 同生命周期)────────────────────
function applyOdomSub(on) {
  const ros = getRosClient();
  if (on && ros && !odomSub) {
    odomSub = new ROSLIB.Topic({
      ros, name: SLAM_ODOM_TOPIC, messageType: 'nav_msgs/msg/Odometry',
      compression: 'none', throttle_rate: 0, queue_length: 1,
    });
    subscribeWithQos(odomSub, onSlamOdom);
    console.info('[lidar] 订阅 SLAM 位姿:', SLAM_ODOM_TOPIC);
  } else if (odomSub && (!on || !ros)) {
    try { odomSub.unsubscribe(); } catch { /* 忽略 */ }
    odomSub = null;
    poseLastAt = 0;
    poseTimes.length = 0;
    poseLatest = null;
    markPoseStale(true);
  }
}

function onSlamOdom(msg) {
  const p = msg && msg.pose && msg.pose.pose && msg.pose.pose.position;
  const q = msg && msg.pose && msg.pose.pose && msg.pose.pose.orientation;
  if (!p || !q) return;
  // 存一份数字位姿:点云预览要拿它把 body 系点搬到世界系
  poseLatest = {
    p: [Number(p.x), Number(p.y), Number(p.z)],
    q: [Number(q.x), Number(q.y), Number(q.z), Number(q.w)],
  };
  const now = performance.now();
  poseLastAt = now;
  poseTimes.push(now);
  if (poseTimes.length > POSE_RATE_N) poseTimes.shift();

  if (els.poseX) els.poseX.textContent = Number(p.x).toFixed(3);
  if (els.poseY) els.poseY.textContent = Number(p.y).toFixed(3);
  if (els.poseZ) els.poseZ.textContent = Number(p.z).toFixed(3);
  // 四元数 → 偏航角(度):yaw = atan2(2(wz+xy), 1-2(y²+z²))
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / Math.PI;
  if (els.poseYaw) els.poseYaw.textContent = yaw.toFixed(1) + '°';
  if (els.poseHz && poseTimes.length >= 3) {
    const dt = (poseTimes[poseTimes.length - 1] - poseTimes[0]) / 1000;
    if (dt > 0) els.poseHz.textContent = ((poseTimes.length - 1) / dt).toFixed(1) + ' Hz';
  }
  markPoseStale(false);
}

function markPoseStale(stale) {
  if (els.poseHud) els.poseHud.classList.toggle('is-stale', !!stale);
  if (stale && els.poseHz) els.poseHz.textContent = '—';
}

function applyDesired() {
  // 点云不再跟页签可见性挂钩(见文件头注释):这里管的是雷达内置 IMU、SLAM 位姿与画布尺寸
  setLidarEnabled(viewVisible && rosOnline);
  applyOdomSub(viewVisible && rosOnline);
  // 录制流:建图中才订(开关关掉时前端一个包都不该收 —— 见 ros-bridge.js)
  setLidarRecordStreamEnabled(recPhase === 'recording');
  // 点云预览:未建图时的实时画面(录制中让位给录制流)
  applyPreviewSub(viewVisible && rosOnline && recPhase !== 'recording');
  // 前端融合上色:彩色画面 + 内参 + 静态 TF(外参)。跟"点云到底在不在进"对齐 ——
  // 预览流本身就是按页签可见性订的,建图时录制流不跟可见性(见 ros-bridge.js),
  // 所以这两种情况下才需要那几路;其余时候订了也只是白下载 13Hz 的 JPEG。
  applyColorSubs(rosOnline && (viewVisible || recPhase === 'recording'));
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
    // 前端融合(相机给累积地图上色)+ 存回板上
    colorPreview: document.querySelector('#lidar-color-preview'),
    colorInfo: document.querySelector('#lidar-color-info'),
    colorOverlayEl: document.querySelector('#lidar-color-overlay'),
    btnSave: document.querySelector('#lidar-btn-save'),
    saveNote: document.querySelector('#lidar-save-note'),
    sizeRange: document.querySelector('#lidar-point-size'),
    sizeVal: document.querySelector('#lidar-point-size-val'),
    attiCanvas: document.querySelector('#lidar-atti'),
    waveCanvas: document.querySelector('#lidar-wave'),
    accX: document.querySelector('#lidar-acc-x'), accY: document.querySelector('#lidar-acc-y'), accZ: document.querySelector('#lidar-acc-z'),
    gyrX: document.querySelector('#lidar-gyr-x'), gyrY: document.querySelector('#lidar-gyr-y'), gyrZ: document.querySelector('#lidar-gyr-z'),
    gyrXD: document.querySelector('#lidar-gyr-x-d'), gyrYD: document.querySelector('#lidar-gyr-y-d'), gyrZD: document.querySelector('#lidar-gyr-z-d'),
    // SLAM 位姿 HUD(/aft_mapped_to_init)
    poseHud: document.querySelector('#lidar-pose-hud'),
    poseX: document.querySelector('#lidar-pose-x'),
    poseY: document.querySelector('#lidar-pose-y'),
    poseZ: document.querySelector('#lidar-pose-z'),
    poseYaw: document.querySelector('#lidar-pose-yaw'),
    poseHz: document.querySelector('#lidar-pose-hz'),
  };
  if (!els.canvas) return;

  // 位姿超时看门狗:数据断了 2s 就把 HUD 置灰(值保留,提示不是实时)
  setInterval(() => {
    if (!viewVisible || !poseLastAt) return;
    if (performance.now() - poseLastAt > POSE_STALE_MS) markPoseStale(true);
  }, 500);

  // 波形/姿态球固定内部分辨率(CSS 负责显示尺寸)
  els.waveCanvas.width = 600; els.waveCanvas.height = 110;
  els.attiCanvas.width = 132; els.attiCanvas.height = 132;

  initThree();

  const setView = (x, y, z, tx = 0, ty = 0, tz = 0) => {
    camera3d.position.set(x, y, z);
    controls.target.set(tx, ty, tz);
    controls.update();
  };
  // 相机预设(z-up 世界):俯视沿 -z 看,屏幕上方=+y(绿)、右侧=+x(红)。
  // y 用 -0.02 微偏移而不是精确 0 —— 视线与 camera.up(0,0,1) 平行会退化。
  els.btnReset?.addEventListener('click', () => setView(5.5, -5.5, 4.2));
  els.btnTop?.addEventListener('click', () => setView(0, -0.02, 14));
  els.btnSide?.addEventListener('click', () => setView(0, -14, 2.5));
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
  // 存回板上:把浏览器累积+融合好的点云 POST 给 webserver 落成 pcd
  els.btnSave?.addEventListener('click', () => { saveMapToBoard(); });
  // 小窗叠画命中点(外参标定用:深度边缘和画面里物体边缘对不对齐)
  els.colorOverlayEl?.addEventListener('change', () => {
    colorOverlay = !!els.colorOverlayEl.checked;
    drawColorPreview(colorHits);
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
    // 所有订阅都跟着**节点的真实状态**走(命令也可能是别的页面/命令行发的):
    // 录制流、预览流、融合那三路全在 applyDesired() 里统一算,别在这儿各订各的
    applyDesired();
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
  if (colorTimer) clearInterval(colorTimer);
  colorTimer = setInterval(colorAccumulatedPass, COLOR_PERIOD_MS);
  updateColorHud();
}
