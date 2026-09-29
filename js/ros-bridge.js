/**
 * ROS 话题桥接模块。
 *
 * 连接 rosbridge_server (WebSocket),订阅电机状态话题,
 * 将真机关节位置与仿真关节做绑定。
 *
 * 连接地址: ws://192.168.5.101:9090
 * 话题名:   /rl_real/motor_state
 * 类型:     std_msgs/msg/Float32MultiArray
 *
 * 消息布局(每个电机占 10 个 float,旧协议为 9 个 float 无温度,解析时自动识别):
 *   base = i * stride   (stride 优先按映射表电机数精确匹配 10/9,再退回整除启发式)
 *   idx        = data[base + 0]   (电机配置序号 i 的校验字段)
 *   target_pos = data[base + 1]
 *   real_pos   = data[base + 2]
 *   target_vel = data[base + 3]
 *   real_vel   = data[base + 4]
 *   target_tau = data[base + 5]
 *   real_tau   = data[base + 6]
 *   kp         = data[base + 7]
 *   kd         = data[base + 8]
 *   temp       = data[base + 9]   (°C,仅 10-float 新协议)
 */
import { ROSLIB } from './roslib-shim.js';
import * as THREE from 'three';
import { jointStates, currentModelId, shared } from './state.js?v=1';
import { setJointPose } from './joints.js?v=1';

// 真机地址: ws://192.168.5.101:9090
// 本地 mock: ws://127.0.0.1:9090 (js/_mock-rosbridge.js)
// 可通过页面顶部的 ROS 地址输入框动态修改,下面常量仅作为默认兜底 fallback
export const ROS_URL_DEFAULT = 'ws://127.0.0.1:9090';
// 兼容旧代码:仍导出 ROS_URL(只读)指向 DEFAULT
export const ROS_URL = ROS_URL_DEFAULT;
export const MOTOR_STATE_TOPIC = '/rl_real/motor_state';
export const IMU_STATE_TOPIC   = '/rl_real/imu_state';
export const COMMAND_TOPIC     = '/rl_real/command';       // 行为命令: getup / getdown / locomotion / vel_stop / zero（passive 已于 2026-09-29 全链路移除:卸力不可靠,一律 getdown 回趴下）
export const CMD_VEL_TOPIC     = '/rl_real/cmd_vel';       // 底盘速度: geometry_msgs/msg/Twist
export const FEEDBACK_TOPIC    = '/rl_real/feedback';      // 指令应答/状态反馈: std_msgs/msg/String
export const POSE2D_TOPIC      = '/rl_real/pose2d';        // 里程计位姿: std_msgs/msg/Float32MultiArray[7]
export const IMU_STREAM_ENABLE_TOPIC = '/rl_real/imu_stream_enable'; // IMU 数据流开关: std_msgs/msg/Bool
export const NOTIFY_TOPIC      = '/rl_real/notify';        // 机器人通知事件: std_msgs/msg/String (JSON: {id,type,content,timestamp})
export const NOTIFY_ACK_TOPIC  = '/rl_real/notify_ack';     // 通知回复: std_msgs/msg/String (JSON: {id,ok})
export const CHECK_STAND_TOPIC     = '/rl_real/check_stand';        // 站立检查触发: std_msgs/msg/String (上位机→机器人)
export const CHECK_STAND_RESULT_TOPIC = '/rl_real/check_stand_result'; // 站立检查结果: std_msgs/msg/String (机器人→上位机, JSON)
export const BRIEFING_PLAY_TOPIC   = '/rl_briefing/play';    // 作业交底语音播报: std_msgs/msg/String (上位机→机器人: 场景名/stop/list)
export const BRIEFING_STATUS_TOPIC = '/rl_briefing/status';  // 作业交底播报状态: std_msgs/msg/String (机器人→上位机)

// ── Insight 9 相机话题(传感器数据,发布端为 best_effort QoS,订阅必须 sensor_data/best_effort 才能匹配) ──
// 实测(2026-09 真机固件):image_raw 系列话题只注册不发流,实际数据流在 image_rect_raw(校正后)系列;
// 因此每通道按「rect 优先、raw 兜底」的候选列表订阅,首帧超时自动降级(见 _subscribeChannelCandidate)。
export const CAMERA_COLOR_TOPIC        = '/camera/camera/color/image_raw/compressed';       // CompressedImage (JPEG,旧固件)
export const CAMERA_COLOR_RECT_TOPIC   = '/camera/camera/color/image_rect_raw/compressed';  // CompressedImage (JPEG,当前固件实发 13Hz)
export const CAMERA_INFRA1_TOPIC       = '/camera/camera/infra1/image_raw';                 // Image (旧固件)
export const CAMERA_INFRA1_RECT_TOPIC  = '/camera/camera/infra1/image_rect_raw';            // Image (mono8 544x640,当前固件实发)
export const CAMERA_INFRA2_TOPIC       = '/camera/camera/infra2/image_raw';
export const CAMERA_INFRA2_RECT_TOPIC  = '/camera/camera/infra2/image_rect_raw';
export const CAMERA_IMU_TOPIC      = '/camera/camera/imu';               // sensor_msgs/msg/Imu
export const CAMERA_VIO_TOPIC      = '/camera/camera/vio_100hz';         // geometry_msgs/msg/PoseStamped (真机实测类型,非 Odometry)
export const CAMERA_VIO_STATUS_TOPIC = '/camera/camera/vio_status';      // std_msgs/msg/String (VIO 状态)
const CAMERA_TYPE_COMPRESSED   = 'sensor_msgs/msg/CompressedImage';
const CAMERA_TYPE_IMAGE        = 'sensor_msgs/msg/Image';
const CAMERA_TYPE_IMU          = 'sensor_msgs/msg/Imu';
const CAMERA_TYPE_POSE_STAMPED = 'geometry_msgs/msg/PoseStamped';
const ROS2_STRING_TYPE         = 'std_msgs/msg/String';
/** 相机通道 → 候选话题(按顺序尝试,首帧超时切下一个)/服务端节流(ms) */
const CAMERA_CHANNELS = {
  color: {
    throttleMs: 66,  // ≤15Hz
    candidates: [
      { topic: CAMERA_COLOR_RECT_TOPIC, type: CAMERA_TYPE_COMPRESSED },
      { topic: CAMERA_COLOR_TOPIC, type: CAMERA_TYPE_COMPRESSED },
    ],
  },
  infra1: {
    throttleMs: 100,
    candidates: [
      { topic: CAMERA_INFRA1_RECT_TOPIC, type: CAMERA_TYPE_IMAGE },
      { topic: CAMERA_INFRA1_TOPIC, type: CAMERA_TYPE_IMAGE },
    ],
  },
  infra2: {
    throttleMs: 100,
    candidates: [
      { topic: CAMERA_INFRA2_RECT_TOPIC, type: CAMERA_TYPE_IMAGE },
      { topic: CAMERA_INFRA2_TOPIC, type: CAMERA_TYPE_IMAGE },
    ],
  },
};
/**
 * 候选话题首帧超时:超过该时间没收到帧就退订并尝试下一个候选。
 * 实测真机彩色 rect 流是突发的(有时静默数秒),故取 4s 且候选会循环重试,直到某话题真正出帧才锁定。
 */
const CAMERA_CANDIDATE_TIMEOUT_MS = 4000;
/** best_effort/sensor_data QoS:相机节点用 best_effort 发布,reliable 订阅不会匹配 */
const CAMERA_BEST_EFFORT_QOS = { reliability: 'best_effort', durability: 'volatile', history: 'keep_last', depth: 2 };

// ── Livox(MID-360)雷达内置 IMU(同样是 sensor_data/best_effort QoS) ──
export const LIDAR_IMU_TOPIC = '/livox/imu';    // sensor_msgs/msg/Imu, 200Hz 雷达内置 IMU(BMI088)
const CAMERA_TYPE_POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2';
/** 雷达 IMU 200Hz → 卡片/波形 20Hz 足够(50ms) */
const LIDAR_IMU_THROTTLE_MS = 50;

// ── 建图录制:点云由板上的 lidar_recorder 节点统一出口 ──────────────
// 前端不再直接订 /livox/lidar:板上驱动是 xfer_format=1,那个话题上发的是
// CustomMsg(Point-LIO 要的格式),类型就对不上;而且「开关关掉时完全静默」
// 要求不开开关就一个点都不收。
// lidar_recorder 订的是 Point-LIO 的世界系逐帧点云 /cloud_registered
// (已 IMU 去畸变、已按位姿拼好),只在开关打开时转发到下面这个话题,
// 并把同一批点按体素去重后落盘成 pcd。
export const LIDAR_REC_COMMAND_TOPIC = '/lidar_recorder/command'; // std_msgs/msg/Bool(true=开始建图,false=结束)
export const LIDAR_REC_STATUS_TOPIC  = '/lidar_recorder/status';  // std_msgs/msg/String(JSON,2Hz)
export const LIDAR_REC_FRAME_TOPIC   = '/lidar_recorder/frame';   // sensor_msgs/msg/PointCloud2(世界系,转发帧)

/**
 * 复现时屏蔽真机回调:置 true 后,真实 ROS motor/imu 订阅消息全部丢弃,
 * 仅 injectMotorFrame / injectImuFrame 仍可注入,避免双数据流打架。
 */
let _replayMutingRealCallbacks = false;
export function setReplayMutingRealCallbacks(on) {
  _replayMutingRealCallbacks = !!on;
}
export function isReplayMutingRealCallbacks() {
  return _replayMutingRealCallbacks;
}

/** 运行时 ROS URL(可被 main.js 通过 setRosUrl 设置,来源于页面输入框的值) */
let _runtimeRosUrl = ROS_URL_DEFAULT;
/** 设置运行时 rosbridge WebSocket 地址,只在未连接状态下生效(或下次连接时使用) */
export function setRosUrl(url) {
  const u = String(url || '').trim();
  if (u) _runtimeRosUrl = u;
}
/** 获取当前生效的 rosbridge WebSocket 地址 */
export function getRosUrl() {
  return _runtimeRosUrl || ROS_URL_DEFAULT;
}

/**
 * 获取内部 ROSLIB.Ros 连接实例(话题调试工具等复用)。
 * 未连接/已断开时返回 null;调用方需自行检查 connectionStatus === 'online'。
 */
export function getRosClient() {
  return (connectionStatus === 'online' && ros) ? ros : null;
}

/** 行为命令最近发布次数(UI 显示用) */
export let commandPublishCount = 0;
export let lastCommandPublishAt = 0;
export let lastCommandPayload = '';

/** cmd_vel 最近发布次数 + 最后一次 Twist(UI 显示用) */
export let cmdVelPublishCount = 0;
export let lastCmdVelPublishAt = 0;
export let lastCmdVel = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };

/**
 * IMU Float32MultiArray[19] 字段定义。
 * layout.dim.label 声明:
 *   quat_w,quat_x,quat_y,quat_z, roll,pitch,yaw,
 *   acc_x,acc_y,acc_z, gyro_x,gyro_y,gyro_z,
 *   pos_north,pos_east,pos_down, vel_body_x,vel_body_y,vel_body_z
 */
const IMU_FIELDS = {
  QUAT_W: 0, QUAT_X: 1, QUAT_Y: 2, QUAT_Z: 3,
  ROLL: 4, PITCH: 5, YAW: 6,
  ACC_X: 7, ACC_Y: 8, ACC_Z: 9,
  GYRO_X: 10, GYRO_Y: 11, GYRO_Z: 12,
  POS_N: 13, POS_E: 14, POS_D: 15,
  VEL_X: 16, VEL_Y: 17, VEL_Z: 18,
};
/** IMU 最近帧数(对外只读,与 motor 的计数分开) */
export let imuMessageCount = 0;
/** IMU 最近一帧时间戳(ms) */
export let lastImuMessageAt = 0;

/**
 * /rl_real/pose2d Float32MultiArray[7] 字段定义:
 *   x, y, z, yaw, vel_x, vel_y, yaw_rate
 */
const POSE2D_FIELDS = {
  X: 0, Y: 1, Z: 2, YAW: 3,
  VEL_X: 4, VEL_Y: 5, YAW_RATE: 6,
};
/** pose2d 最近帧数 + 时间戳(对外只读) */
export let pose2dMessageCount = 0;
export let lastPose2dMessageAt = 0;

/**
 * 各模型的 "配置序号 i" -> 仿真关节名 映射表。
 * i 是消息中 9 元素块的块号(0 开始),对应配置表中的"配置序号"。
 */
export const JOINT_MAPPING = {
  // 模型 1:人形 (actuator XML 定义顺序,如需调整可改此处)
  '1': [
    'fl_steering_joint',
    'fr_steering_joint',
    'bl_steering_joint',
    'br_steering_joint',
    'lift_joint',
    'robot_left_joint1',
    'robot_left_joint2',
    'robot_left_joint3',
    'robot_left_joint4',
    'robot_left_joint5',
    'robot_left_joint6',
    'robot_left_joint7',
    'robot_left_finger_joint1',
    'robot_right_joint1',
    'robot_right_joint2',
    'robot_right_joint3',
    'robot_right_joint4',
    'robot_right_joint5',
    'robot_right_joint6',
    'robot_right_joint7',
    'robot_right_finger_joint1',
    'robot_head_pitch_joint',
    'robot_head_yaw_joint',
  ],
  // 模型 2:四足机器人 (按用户提供映射表:配置序号 0~15)
  '2': [
    'FL_hip_joint',   // 0  左前侧摆
    'FL_thigh_joint', // 1  左前髋
    'FL_calf_joint',  // 2  左前膝
    'FL_foot_joint',  // 3  左前踝
    'FR_hip_joint',   // 4  右前侧摆
    'FR_thigh_joint', // 5  右前髋
    'FR_calf_joint',  // 6  右前膝
    'FR_foot_joint',  // 7  右前踝
    'RL_hip_joint',   // 8  左后侧摆
    'RL_thigh_joint', // 9  左后髋
    'RL_calf_joint',  // 10 左后膝
    'RL_foot_joint',  // 11 左后踝
    'RR_hip_joint',   // 12 右后侧摆
    'RR_thigh_joint', // 13 右后髋
    'RR_calf_joint',  // 14 右后膝
    'RR_foot_joint',  // 15 右后踝
  ],
};

/** 位置源:'real' = real_pos, 'target' = target_pos */
export let positionSource = 'real';
/** 连接状态:'offline' | 'connecting' | 'online' | 'error' */
export let connectionStatus = 'offline';
/** 最近一次错误信息 */
export let lastError = '';
/** 最近一帧消息的时间戳(ms),用于 UI 判断数据流是否正常 */
export let lastMessageAt = 0;
/** 累计收到的消息帧数 */
export let messageCount = 0;
/** 最近一次消息解包到的电机块数 */
export let lastBlockCount = 0;
/** 是否启用 ROS 覆盖(即 ROS 数据生效且禁用本地滑块) */
export let rosControlActive = false;

let ros = null;
let motorStateSubscriber = null;
let imuStateSubscriber = null;
let feedbackSubscriber = null;
let pose2dSubscriber = null;
let notifySubscriber = null;
let notifyAckPublisher = null;
let checkStandPublisher = null;
let checkStandResultSubscriber = null;
let briefingPublisher = null;
let briefingStatusSubscriber = null;
// 相机订阅(按需创建:仅"相机"页签可见且 ROS 在线时订阅,省带宽)
let cameraChannelSub = null;         // 当前画面通道订阅(三选一,可能正在试某个候选话题)
let cameraCurrentChannel = null;     // 'color' | 'infra1' | 'infra2' | null
let cameraCandidateIdx = 0;          // 当前通道正在尝试的候选话题下标
let cameraCandidateTimer = null;     // 候选首帧超时计时器
let cameraImuSub = null;
let cameraVioSub = null;
let cameraVioStatusSub = null;
let cameraTelemetryEnabled = false;
// Livox 雷达内置 IMU 订阅(仅"雷达"页签可见且在线时订阅);点云见下面的建图录制
let lidarImuSub = null;
let lidarEnabled = false;
/** 客户端节流:上次向监听器分发的时间戳(ms) */
let lastLidarImuEmitAt = 0;

// 建图录制(见 LIDAR_REC_* 常量那段注释)
let lidarRecCommandPublisher = null;   // Bool → 板上录制节点
let lidarRecStatusSubscriber = null;   // String(JSON) ← 板上录制节点
let lidarRecFrameSub = null;           // 转发点云,仅录制中订阅
let lidarRecStreamDesired = false;     // 期望状态:是否该收转发点云(断连重连后按此恢复)
let commandPublisher = null;
let cmdVelPublisher = null;
const statusListeners = new Set();
const messageListeners = new Set();
const imuListeners = new Set();
const notifyListeners = new Set();
const checkStandListeners = new Set();
const briefingStatusListeners = new Set();
const cameraFrameListeners = new Set();
const cameraImuListeners = new Set();
const cameraVioListeners = new Set();
const cameraVioStatusListeners = new Set();
const lidarFrameListeners = new Set();
const lidarImuListeners = new Set();
/** 建图状态(解析后的 JSON 对象,不是原始字符串) */
const lidarRecStatusListeners = new Set();
/** 通知消息累计收到的条数 */
export let notifyMessageCount = 0;
/** 最近一次通知的简要信息(UI 状态栏显示) */
export let lastNotifySummary = '';
/** 状态栏 emit 节流:高频数据流下每 30/60 条一条太快,叠加 200ms 时间门限(≤5Hz) */
let _lastMotorStatusEmitAt = 0;
let _lastImuStatusEmitAt = 0;

/** 20Hz = 50ms 间隔(CLI --rate 20 对齐) */
const CMD_VEL_INTERVAL_MS = 50;
/** 2Hz = 500ms 间隔(CLI --rate 2 对齐) */
const COMMAND_INTERVAL_MS = 500;

/** 当前对外"请求的目标速度";20Hz 定时器会持续把它 publish 到 /rl_real/cmd_vel(vz=linear.z 垂直速度) */
const cmdVelRequest = { vx: 0, vy: 0, omega: 0, vz: 0 };
/** cmd_vel keep-alive 定时器句柄(非空=正在持续发布) */
let cmdVelTimerId = null;
/** cmd_vel request 非零时已持续帧数;用于"归零后还需多发 N 帧零速度再停 timer"的 N 计数 */
let cmdVelZeroFramesRemaining = 0;

/** status 事件(对外 UI):新增 command / cmd_vel 统计字段一起广播 */
export function getStatusSnapshot() {
  return {
    connectionStatus, lastError, rosControlActive, positionSource,
    lastMessageAt, messageCount, lastBlockCount,
    lastImuMessageAt, imuMessageCount,
    commandPublishCount, lastCommandPublishAt, lastCommandPayload,
    cmdVelPublishCount, lastCmdVelPublishAt, lastCmdVel,
    notifyMessageCount, lastNotifySummary,
  };
}
function emitStatus() {
  statusListeners.forEach((fn) => fn(getStatusSnapshot()));
}

/** 新消息到达时通知监听者(可选) */
function emitMessage(payload) {
  messageListeners.forEach((fn) => fn(payload));
}

/** IMU 新帧到达通知 */
function emitImu(payload) {
  imuListeners.forEach((fn) => fn(payload));
}

/** 通知事件到达时通知监听者 */
function emitNotify(payload) {
  notifyListeners.forEach((fn) => fn(payload));
}

/** 站立检查结果到达时通知监听者 */
function emitCheckStandResult(payload) {
  checkStandListeners.forEach((fn) => fn(payload));
}

/** 作业交底播报状态到达时通知监听者 */
function emitBriefingStatus(text) {
  briefingStatusListeners.forEach((fn) => fn(text));
}

/** 相机画面帧到达 { channel, msg } */
function emitCameraFrame(payload) {
  cameraFrameListeners.forEach((fn) => fn(payload));
}
/** 相机 IMU 帧 */
function emitCameraImu(msg) { cameraImuListeners.forEach((fn) => fn(msg)); }
/** VIO 位姿帧 */
function emitCameraVio(msg) { cameraVioListeners.forEach((fn) => fn(msg)); }
/** VIO 状态文本 */
function emitCameraVioStatus(text) { cameraVioStatusListeners.forEach((fn) => fn(text)); }
/** 雷达点云帧(原始 PointCloud2 msg,由 lidar-view 按 fields/point_step 解码) */
function emitLidarFrame(msg) { lidarFrameListeners.forEach((fn) => fn(msg)); }
/** 雷达内置 IMU 帧(BMI088,200Hz) */
function emitLidarImu(msg) { lidarImuListeners.forEach((fn) => fn(msg)); }
/** 建图状态(解析后的 JSON 对象) */
function emitLidarRecStatus(obj) { lidarRecStatusListeners.forEach((fn) => fn(obj)); }

export function addStatusListener(fn) { statusListeners.add(fn); emitStatus(); return () => statusListeners.delete(fn); }
export function addMessageListener(fn) { messageListeners.add(fn); return () => messageListeners.delete(fn); }
export function addImuListener(fn) { imuListeners.add(fn); return () => imuListeners.delete(fn); }
export function addNotifyListener(fn) { notifyListeners.add(fn); return () => notifyListeners.delete(fn); }
export function addCheckStandListener(fn) { checkStandListeners.add(fn); return () => checkStandListeners.delete(fn); }
export function addBriefingStatusListener(fn) { briefingStatusListeners.add(fn); return () => briefingStatusListeners.delete(fn); }
export function addCameraFrameListener(fn) { cameraFrameListeners.add(fn); return () => cameraFrameListeners.delete(fn); }
export function addCameraImuListener(fn) { cameraImuListeners.add(fn); return () => cameraImuListeners.delete(fn); }
export function addCameraVioListener(fn) { cameraVioListeners.add(fn); return () => cameraVioListeners.delete(fn); }
export function addCameraVioStatusListener(fn) { cameraVioStatusListeners.add(fn); return () => cameraVioStatusListeners.delete(fn); }
export function addLidarFrameListener(fn) { lidarFrameListeners.add(fn); return () => lidarFrameListeners.delete(fn); }
export function addLidarImuListener(fn) { lidarImuListeners.add(fn); return () => lidarImuListeners.delete(fn); }
export function addLidarRecStatusListener(fn) { lidarRecStatusListeners.add(fn); return () => lidarRecStatusListeners.delete(fn); }

export function setPositionSource(source) {
  positionSource = source === 'target' ? 'target' : 'real';
  emitStatus();
}

/** 设置 ROS 控制是否启用(通常连接成功后设为 true,断开设为 false) */
function setControlActive(active) {
  rosControlActive = active;
  emitStatus();
}

function setStatus(s, errorMsg = '') {
  connectionStatus = s;
  if (errorMsg) lastError = errorMsg;
  emitStatus();
}

/**
 * 将解包后的电机位置应用到对应关节。
 * 直接写入 state.value 与 state.target 并调用 setJointPose,绕过渲染循环中的阻尼插值,
 * 让仿真画面严格跟随真机数据,减少延迟。
 * 滑块 UI 同步改为「标脏 + ~20Hz 批量刷新」:高频消息下每条消息 × 每个电机都做
 * querySelector + input/output 写入会把主线程淹没在 DOM 工作里,导致浏览器严重卡顿。
 */
const _dirtySliderJoints = new Map(); // jointName -> 最近一次夹紧值
const _sliderRefsCache = new Map();   // jointName -> { input, out } | null(缺失/失效时重查)

/** 把标脏的关节同步到滑块 DOM(~20Hz 批量,代替每消息同步) */
function flushSliderSync() {
  if (!_dirtySliderJoints.size) return;
  for (const [jointName, clamped] of _dirtySliderJoints) {
    let refs = _sliderRefsCache.get(jointName);
    if (!refs || !refs.input.isConnected) {
      const input = document.querySelector(`input[type="range"][data-joint="${jointName}"]`);
      refs = input ? { input, out: input.parentElement.querySelector('output') } : null;
      _sliderRefsCache.set(jointName, refs);
    }
    if (!refs) continue;
    const state = jointStates.get(jointName);
    const precision = state ? (state.type === 'slide' ? 3 : state.controlMode === 'velocity' ? 1 : 2) : 2;
    refs.input.value = String(clamped);
    if (refs.out) refs.out.value = clamped.toFixed(precision);
  }
  _dirtySliderJoints.clear();
}
window.setInterval(flushSliderSync, 50);

function applyMotorPositions(blockIndex, pos) {
  const mapping = JOINT_MAPPING[currentModelId];
  if (!mapping) return;
  const jointName = mapping[blockIndex];
  if (!jointName) return;
  const state = jointStates.get(jointName);
  if (!state) return;
  const clamped = THREE.MathUtils.clamp(pos, state.min, state.max);
  state.value = clamped;
  state.target = clamped;
  setJointPose(state, clamped);
  // 滑块 UI:仅标脏,由 flushSliderSync ~20Hz 批量同步
  _dirtySliderJoints.set(jointName, clamped);
}

/**
 * 解析 Float32MultiArray 消息并应用到关节。
 * 每 9/10 个 float 为一块(自适应识别,见文件头布局说明);base+0 为 idx(与块号 i 做一致性校验,不一致时 warn)。
 *
 * 兼容多种消息封装:
 *  - 标准 roslib:     message.data = [float...]
 *  - 某些 ROS2 桥:    message.msg = { data: [...] }  或  message.msg.data = [...]
 *  - 直接数组:        message instanceof Array
 *  - data.dim + data.data 结构(完整 Float32MultiArray)
 */
function handleMotorStateMessage(message) {
  // ══════════════════════════════════════════════════════════════════
  // 复现时真机 ROS 回调被屏蔽(避免真机帧与 inject 复现帧冲突)
  // injectMotorFrame 内部走 handleMotorStateMessageImpl 绕过此守卫
  if (_replayMutingRealCallbacks) return;
  _handleMotorStateMessageImpl(message);
}

/**
 * 原始电机遥测(jointName → 9/10 字段块,未夹紧未加工):
 *   { idx, targetPos, realPos, targetVel, realVel, targetTau, realTau, kp, kd, temp?, at }
 *   temp 仅 10-float 新协议存在(°C)。
 * 数据看板 / 曲线从这里读原始值;applyMotorPositions 的夹紧值仅用于驱动仿真模型。
 */
export const motorTelemetry = new Map();

/** 电机消息实际处理逻辑(与 handleMotorStateMessage 分开以支持 inject 绕过 muting) */
function _handleMotorStateMessageImpl(message) {
  // ── 1. 用多种方式尝试取出 data 数组 ──────────────
  let data = null;
  if (Array.isArray(message)) {
    data = message;
  } else if (Array.isArray(message?.data) && message.data.length > 0) {
    data = message.data;
  } else if (Array.isArray(message?.msg?.data)) {
    data = message.msg.data;
  } else if (message?.msg && Array.isArray(message?.msg) && message.msg.length > 0) {
    data = message.msg;
  } else {
    for (const key of Object.keys(message || {})) {
      const v = message[key];
      if (Array.isArray(v) && v.length >= 9) { data = v; break; }
      if (v && typeof v === 'object' && Array.isArray(v.data) && v.data.length >= 9) { data = v.data; break; }
    }
  }
  if (!data || !Array.isArray(data) || data.length === 0) {
    if (!_handleMotorStateMessageImpl._warnedEmpty) {
      console.warn('[ros] 收到消息但未找到 data 数组,原始消息 dump:', JSON.stringify(message).slice(0, 500));
      _handleMotorStateMessageImpl._warnedEmpty = true;
    }
    return;
  }

  const mapping = JOINT_MAPPING[currentModelId];
  if (!mapping) {
    if (!_handleMotorStateMessageImpl._warnedModel) {
      console.warn('[ros] 当前模型', currentModelId, '无映射表,忽略消息。');
      _handleMotorStateMessageImpl._warnedModel = true;
    }
    return;
  }
  // stride 自适应:10-float 新协议(含温度) / 9-float 旧协议。
  // 优先按当前模型电机数精确匹配 —— 单纯用「长度能否被 10 整除」会误判:
  // 例如 20 个电机的旧协议长度 180 也能被 10 整除,会被错当成 stride=10。
  const jointCount = mapping.length || 0;
  let stride;
  if (jointCount > 0 && data.length === jointCount * 10) stride = 10;
  else if (jointCount > 0 && data.length === jointCount * 9) stride = 9;
  else stride = data.length % 10 === 0 ? 10 : 9;
  const blockCount = Math.floor(data.length / stride);
  if (blockCount === 0) {
    console.warn('[ros] data 长度不足一块(stride=', stride, ',长度=', data.length, '):', data.slice(0, 10));
    return;
  }
  lastMessageAt = performance.now();
  messageCount += 1;
  lastBlockCount = blockCount;
  if (messageCount % 30 === 1 && lastMessageAt - _lastMotorStatusEmitAt >= 200) {
    _lastMotorStatusEmitAt = lastMessageAt;
    emitStatus();
  }

  if (!_handleMotorStateMessageImpl._loggedFirst) {
    const preview = [];
    for (let i = 0; i < Math.min(blockCount, 4); i += 1) {
      const b = i * stride;
      const tempStr = stride === 10 ? ` temp=${data[b+9].toFixed(1)}°C` : '';
      preview.push(`i=${i}: idx=${data[b+0]} tgt=${data[b+1].toFixed(3)} real=${data[b+2].toFixed(3)} kp=${data[b+7]} kd=${data[b+8]}${tempStr}`);
    }
    console.info(`[ros] 首条消息:共 ${data.length} 个数 => ${blockCount} 个电机块。前几块:\n${preview.join('\n')}`);
    _handleMotorStateMessageImpl._loggedFirst = true;
  }

  let applied = 0;
  for (let i = 0; i < blockCount; i += 1) {
    const base = i * stride;
    const targetPos = Number(data[base + 1]);
    const realPos = Number(data[base + 2]);
    // 原始遥测入表(供数据看板/曲线使用,不做夹紧/位置源处理)
    const jointName = mapping[i];
    if (jointName) {
      // 原地复用既有对象,避免每条消息 × 每个电机分配新对象造成 GC 压力
      let tel = motorTelemetry.get(jointName);
      if (!tel) { tel = {}; motorTelemetry.set(jointName, tel); }
      tel.idx = Number(data[base + 0]);
      tel.targetPos = targetPos;
      tel.realPos = realPos;
      tel.targetVel = Number(data[base + 3]);
      tel.realVel = Number(data[base + 4]);
      tel.targetTau = Number(data[base + 5]);
      tel.realTau = Number(data[base + 6]);
      tel.kp = Number(data[base + 7]);
      tel.kd = Number(data[base + 8]);
      if (stride === 10) tel.temp = Number(data[base + 9]);
      else if (tel.temp !== undefined) delete tel.temp; // 源切回旧协议时清残留
      tel.at = lastMessageAt;
    }
    const pos = positionSource === 'target' ? targetPos : realPos;
    if (Number.isFinite(pos)) {
      applyMotorPositions(i, pos);
      applied += 1;
    }
  }
  // data 是 roslib 每条消息解码出的新数组,监听者只在调用内使用,无需再 slice 拷贝
  emitMessage({ blockCount, applied, at: lastMessageAt, data });
}

/**
 * 从 message 里兜底取 data 数组。
 * 复用到 motor / imu 两个 Float32MultiArray 话题。
 */
function extractFloat32Data(message, label) {
  if (Array.isArray(message)) return message;
  if (Array.isArray(message?.data) && message.data.length > 0) return message.data;
  if (Array.isArray(message?.msg?.data)) return message.msg.data;
  if (message?.msg && Array.isArray(message?.msg)) return message.msg;
  for (const key of Object.keys(message || {})) {
    const v = message[key];
    if (Array.isArray(v) && v.length >= 2) return v;
    if (v && typeof v === 'object' && Array.isArray(v.data) && v.data.length >= 2) return v.data;
  }
  return null;
}

/**
 * 解析 IMU 19-fields Float32MultiArray。
 * 写入 shared.imu;优先用四元数计算欧拉(RPY 字段备用)。
 */
function handleImuStateMessage(message) {
  // 复现时真机回调屏蔽。injectImuFrame 内部走 _handleImuStateMessageImpl 绕过。
  if (_replayMutingRealCallbacks) return;
  _handleImuStateMessageImpl(message);
}

function _handleImuStateMessageImpl(message) {
  const data = extractFloat32Data(message, 'imu_state');
  if (!data || !Array.isArray(data) || data.length < 19) {
    if (!_handleImuStateMessageImpl._warned) {
      console.warn('[imu] 消息无 data 或长度<19。len=', data?.length, '原始 sample=', JSON.stringify(message).slice(0, 300));
      _handleImuStateMessageImpl._warned = true;
    }
    return;
  }

  lastImuMessageAt = performance.now();
  imuMessageCount += 1;
  const imu = shared.imu;
  imu.lastMsgAt = lastImuMessageAt;
  imu.msgCount  = imuMessageCount;

  const f = IMU_FIELDS;
  imu.quat.w = Number(data[f.QUAT_W]);
  imu.quat.x = Number(data[f.QUAT_X]);
  imu.quat.y = Number(data[f.QUAT_Y]);
  imu.quat.z = Number(data[f.QUAT_Z]);
  imu.rpy.roll  = Number(data[f.ROLL]);
  imu.rpy.pitch = Number(data[f.PITCH]);
  imu.rpy.yaw   = Number(data[f.YAW]);
  imu.acc.x = Number(data[f.ACC_X]);
  imu.acc.y = Number(data[f.ACC_Y]);
  imu.acc.z = Number(data[f.ACC_Z]);
  // 角速度 rad/s
  imu.gyro.x = Number(data[f.GYRO_X]);
  imu.gyro.y = Number(data[f.GYRO_Y]);
  imu.gyro.z = Number(data[f.GYRO_Z]);
  // 位置(北东地)m
  imu.pos.north = Number(data[f.POS_N]);
  imu.pos.east  = Number(data[f.POS_E]);
  imu.pos.down  = Number(data[f.POS_D]);
  // 体坐标系速度 m/s
  imu.vel.x = Number(data[f.VEL_X]);
  imu.vel.y = Number(data[f.VEL_Y]);
  imu.vel.z = Number(data[f.VEL_Z]);

  if (!_handleImuStateMessageImpl._loggedFirst) {
    const n = Math.hypot(imu.quat.w, imu.quat.x, imu.quat.y, imu.quat.z);
    console.info(
      `[imu] 首帧。quat wxyz=(%s) 归一化模=%s  RPY(deg)=%s/%s/%s  |a|=%s m/s²`,
      [imu.quat.w,imu.quat.x,imu.quat.y,imu.quat.z].map(v=>v.toFixed(4)).join(','),
      n.toFixed(4),
      THREE.MathUtils.radToDeg(imu.rpy.roll).toFixed(1),
      THREE.MathUtils.radToDeg(imu.rpy.pitch).toFixed(1),
      THREE.MathUtils.radToDeg(imu.rpy.yaw).toFixed(1),
      Math.hypot(imu.acc.x,imu.acc.y,imu.acc.z).toFixed(3),
    );
    _handleImuStateMessageImpl._loggedFirst = true;
  }
  if (imuMessageCount % 60 === 1 && lastImuMessageAt - _lastImuStatusEmitAt >= 200) {
    _lastImuStatusEmitAt = lastImuMessageAt;
    emitStatus();
  }
  // data 是 roslib 每条消息解码出的新数组,监听者只在调用内使用,无需再 slice 拷贝
  emitImu({ imu: imu /* 引用不变,录制层会 copy */, at: lastImuMessageAt, data });
}

/**
 * 外部注入一帧「电机帧」(复现播放使用)。
 * 效果等价于收到一条 /rl_real/motor_state Float32MultiArray 消息:
 *   写 lastMessageAt/messageCount/lastBlockCount → emitStatus 按需 → applyMotorPositions → emitMessage
 * 这样 main.js / curves.js 无需改,自然走同一套 UI 管道。
 * @param {number[]} data  23 blocks × 9 floats 数组
 * @param {number} atMs    时间戳(performance.now() 基)
 */
export function injectMotorFrame(data, atMs) {
  // 绕过 muting 守卫,直接调用内部 impl
  _handleMotorStateMessageImpl(data);
  if (atMs) lastMessageAt = atMs;
}

/**
 * 外部注入一帧「IMU 帧」(复现播放使用)。
 * 效果等价于收到一条 /rl_real/imu_state Float32MultiArray 消息。
 * @param {number[]} data  19 floats 数组
 * @param {number} atMs    时间戳(performance.now() 基)
 */
export function injectImuFrame(data, atMs) {
  _handleImuStateMessageImpl(data);
  if (atMs) lastImuMessageAt = atMs;
}

/**
 * 处理 /rl_real/pose2d Float32MultiArray[7] 一帧:
 * x, y, z, yaw, vel_x, vel_y, yaw_rate → 写入 shared.rosPose2d
 * @param {{data: number[]}} msg
 */
function _handlePose2dMessageImpl(msg) {
  const data = Array.isArray(msg?.data) ? msg.data : [];
  if (data.length < 7) return;
  const f = POSE2D_FIELDS;
  const odom = shared.rosPose2d;
  odom.x = Number(data[f.X]);
  odom.y = Number(data[f.Y]);
  odom.z = Number(data[f.Z]);
  odom.yaw = Number(data[f.YAW]);
  odom.velX = Number(data[f.VEL_X]);
  odom.velY = Number(data[f.VEL_Y]);
  odom.yawRate = Number(data[f.YAW_RATE]);
  pose2dMessageCount += 1;
  lastPose2dMessageAt = performance.now();
  if (pose2dMessageCount === 1) {
    console.info('[pose2d] 首帧。x,y,z=(%s) yaw=%s rad  vel=(%s, %s) m/s  yaw_rate=%s rad/s',
      [odom.x, odom.y, odom.z].map(v => v.toFixed(3)).join(','),
      odom.yaw.toFixed(3), odom.velX.toFixed(3), odom.velY.toFixed(3), odom.yawRate.toFixed(3));
  }
}

/**
 * 处理 /rl_real/feedback std_msgs/String 一帧:
 * 指令应答 / 状态反馈文本 → 打印日志
 * @param {{data: string}} msg
 */
function _handleFeedbackMessageImpl(msg) {
  const text = String(msg?.data ?? '');
  console.info(`[feedback] ⬇️ ${text}`);
}

/**
 * 机器人通知事件 (/rl_real/notify):解析 JSON → 发射事件 → 自动回复 ack。
 * 协议:{"id":N,"type":"info|warning|error|note","content":"...","timestamp":秒}
 * @param {{data: string}} msg
 */
function _handleNotifyMessageImpl(msg) {
  const raw = String(msg?.data ?? '');
  let parsed = null;
  try { parsed = JSON.parse(raw); } catch (_) {
    // 非 JSON:当作纯文本通知(id=0, type=info)
    parsed = { id: 0, type: 'info', content: raw, timestamp: Date.now() / 1000 };
  }
  const id     = Number(parsed.id) || 0;
  const type   = String(parsed.type || 'info');    // info / warning / error / note
  const content= String(parsed.content || '');
  const ts     = Number(parsed.timestamp) || (Date.now() / 1000);
  const payload = { id, type, content, timestamp: ts, raw };

  notifyMessageCount++;
  // 状态栏简要摘要(截断防过长)
  lastNotifySummary = `[${type}] ${content.length > 50 ? content.slice(0, 50) + '…' : content}`;
  console.info(`[notify] 📨 #${id} ${type}: ${content}`);

  // 发射给 UI 监听者
  emitNotify(payload);
  emitStatus();

  // 自动回复 ack(收到即确认,不让机器人重发)
  publishNotifyAck(id, true);
}

/**
 * 发布通知回复 (/rl_real/notify_ack)。
 * @param {number} id  通知消息编号
 * @param {boolean} ok 是否已确认
 */
export function publishNotifyAck(id, ok = true) {
  if (!notifyAckPublisher) return;
  const payload = JSON.stringify({ id, ok: !!ok });
  try {
    notifyAckPublisher.publish(new ROSLIB.Message({ data: payload }));
    console.info(`[notify] ✅ ack sent #${id} ok=${ok}`);
  } catch (err) {
    console.warn('[notify] ack 发布失败:', err);
  }
}

/**
 * 站立检查结果 (/rl_real/check_stand_result):解析 JSON → 发射事件。
 * @param {{data: string}} msg
 */
function _handleCheckStandResultImpl(msg) {
  const raw = String(msg?.data ?? '');
  let parsed = null;
  try { parsed = JSON.parse(raw); } catch (_) {
    parsed = { ready: false, motors_ready: false, imu_ready: false, reason: raw };
  }
  const result = {
    ready: !!parsed.ready,
    motors_ready: !!parsed.motors_ready,
    imu_ready: !!parsed.imu_ready,
    reason: String(parsed.reason || ''),
    raw,
  };
  console.info(`[check-stand] 🔍 result: ready=${result.ready} motors=${result.motors_ready} imu=${result.imu_ready} reason=${result.reason}`);
  emitCheckStandResult(result);
}

/**
 * 发布站立检查触发 (/rl_real/check_stand)。
 */
export function publishCheckStand() {
  if (!checkStandPublisher) return false;
  try {
    checkStandPublisher.publish(new ROSLIB.Message({ data: 'check' }));
    console.info('[check-stand] 📤 sent check_stand trigger');
    return true;
  } catch (err) {
    console.warn('[check-stand] 发布失败:', err);
    return false;
  }
}

/**
 * 作业交底播报状态 (/rl_briefing/status):原样发射(内容由 C++ 端定义)。
 * @param {{data: string}} msg
 */
function _handleBriefingStatusImpl(msg) {
  const raw = String(msg?.data ?? '');
  if (!raw) return;
  console.info('[briefing] 📢 status:', raw);
  emitBriefingStatus(raw);
}

/**
 * 发布作业交底播报命令 (/rl_briefing/play)。
 * @param {string} scene 场景名(elevator/forklift/lifting/warehouse/height)或 stop(打断)/list(列出场)
 */
export function publishBriefing(scene) {
  if (!briefingPublisher) return false;
  try {
    briefingPublisher.publish(new ROSLIB.Message({ data: String(scene || '') }));
    console.info('[briefing] 📤 sent:', scene);
    return true;
  } catch (err) {
    console.warn('[briefing] 发布失败:', err);
    return false;
  }
}

// ── 相机:best_effort QoS 订阅 + 按需开关 ──────────────────
/**
 * vendored roslib 的 subscribe op 不含 qos 字段,这里包一层把 qos 注入报文。
 * rosbridge_server(ROS2) 支持 subscribe 消息携带 qos:{reliability,durability,history,depth};
 * 不支持的旧版会忽略多余字段,退回默认 reliable。
 */
function _subscribeWithQos(topic, callback, qos) {
  const origCall = topic.callForSubscribeAndAdvertise.bind(topic);
  topic.callForSubscribeAndAdvertise = (msg) => {
    if (qos) msg.qos = qos;
    origCall(msg);
  };
  topic.subscribe(callback);
}

/** UI 期望的相机状态(断连/重连后按此自动恢复) */
let cameraDesiredChannel = null;
/**
 * 切换画面通道订阅。channel: 'color'|'infra1'|'infra2'|null(取消)。
 * 期望状态会被记住,ROS 重连后自动重新订阅。
 */
export function setCameraChannel(channel) {
  cameraDesiredChannel = CAMERA_CHANNELS[channel] ? channel : null;
  _applyCameraDesired();
}
export function getCameraChannel() { return cameraDesiredChannel; }

/** 相机 IMU / VIO 数据订阅开关 */
export function setCameraTelemetryEnabled(on) {
  cameraTelemetryEnabled = !!on;
  _applyCameraDesired();
}
export function isCameraTelemetryEnabled() { return cameraTelemetryEnabled; }

function _clearCameraCandidateTimer() {
  if (cameraCandidateTimer) {
    clearTimeout(cameraCandidateTimer);
    cameraCandidateTimer = null;
  }
}

/**
 * 订阅某画面通道的第 cameraCandidateIdx 个候选话题。
 * 首帧在 CAMERA_CANDIDATE_TIMEOUT_MS 内到达 → 锁定该候选(之后即使短暂断流也不再切换,等发布端自行恢复);
 * 超时无帧 → 退订并尝试下一个候选,所有候选都无帧则循环往复(真机彩色流是突发的,不能一次失败就放弃)。
 */
function _subscribeChannelCandidate(channel) {
  const cfg = CAMERA_CHANNELS[channel];
  const idx = cameraCandidateIdx;
  const cand = cfg.candidates[idx];
  let gotFrame = false;
  const t = new ROSLIB.Topic({
    ros,
    name: cand.topic,
    messageType: cand.type,
    compression: 'none',
    throttle_rate: cfg.throttleMs,
    queue_length: 1,
  });
  _subscribeWithQos(t, (msg) => {
    if (!gotFrame) {
      gotFrame = true;
      _clearCameraCandidateTimer();
      console.info(`[camera] ✅ 画面生效: ${channel} → ${cand.topic}`);
    }
    emitCameraFrame({ channel, topic: cand.topic, msg });
  }, CAMERA_BEST_EFFORT_QOS);
  cameraChannelSub = t;
  cameraCurrentChannel = channel;
  console.info(`[camera] 订阅画面通道: ${channel} 候选 ${idx + 1}/${cfg.candidates.length} (${cand.topic}, best_effort, throttle=${cfg.throttleMs}ms)`);

  _clearCameraCandidateTimer();
  cameraCandidateTimer = setTimeout(() => {
    cameraCandidateTimer = null;
    if (gotFrame) return;
    // 用户可能已切走通道/断开:只有这个订阅仍是当前订阅时才继续降级
    if (cameraChannelSub !== t || cameraCurrentChannel !== channel) return;
    try { t.unsubscribe(); } catch { /* 忽略 */ }
    cameraChannelSub = null;
    // 循环尝试候选(rect/raw),直到某个话题真正出帧并锁定
    cameraCandidateIdx = (idx + 1) % cfg.candidates.length;
    console.warn(`[camera] ${cand.topic} ${CAMERA_CANDIDATE_TIMEOUT_MS}ms 内无帧,改试候选 ${cameraCandidateIdx + 1}/${cfg.candidates.length}: ${cfg.candidates[cameraCandidateIdx].topic}`);
    _subscribeChannelCandidate(channel);
  }, CAMERA_CANDIDATE_TIMEOUT_MS);
}

/** 按期望状态 + 当前连接情况,实际创建/销毁相机订阅 */
function _applyCameraDesired() {
  if (!ros || connectionStatus !== 'online') return;
  // ── 画面通道:不一致就先退掉再订新的 ──
  if (cameraCurrentChannel !== cameraDesiredChannel) {
    _clearCameraCandidateTimer();
    if (cameraChannelSub) {
      try { cameraChannelSub.unsubscribe(); } catch { /* 忽略 */ }
      cameraChannelSub = null;
      console.info('[camera] 取消画面通道:', cameraCurrentChannel);
    }
    cameraCurrentChannel = null;
    cameraCandidateIdx = 0;
    if (cameraDesiredChannel) {
      _subscribeChannelCandidate(cameraDesiredChannel);
    }
  }
  // ── 遥测(IMU/VIO/status):三合一开关 ──
  if (cameraTelemetryEnabled && !cameraImuSub) {
    cameraImuSub = new ROSLIB.Topic({ ros, name: CAMERA_IMU_TOPIC, messageType: CAMERA_TYPE_IMU, compression: 'none', throttle_rate: 50, queue_length: 1 });
    _subscribeWithQos(cameraImuSub, (msg) => emitCameraImu(msg), CAMERA_BEST_EFFORT_QOS);
    cameraVioSub = new ROSLIB.Topic({ ros, name: CAMERA_VIO_TOPIC, messageType: CAMERA_TYPE_POSE_STAMPED, compression: 'none', throttle_rate: 100, queue_length: 1 });
    _subscribeWithQos(cameraVioSub, (msg) => emitCameraVio(msg), CAMERA_BEST_EFFORT_QOS);
    cameraVioStatusSub = new ROSLIB.Topic({ ros, name: CAMERA_VIO_STATUS_TOPIC, messageType: ROS2_STRING_TYPE, compression: 'none', queue_length: 1 });
    _subscribeWithQos(cameraVioStatusSub, (msg) => emitCameraVioStatus(String(msg?.data ?? '')), CAMERA_BEST_EFFORT_QOS);
    console.info('[camera] 订阅遥测: imu + vio(PoseStamped) + vio_status (best_effort)');
  } else if (!cameraTelemetryEnabled && cameraImuSub) {
    try { cameraImuSub.unsubscribe(); } catch { /* 忽略 */ }
    try { cameraVioSub.unsubscribe(); } catch { /* 忽略 */ }
    try { cameraVioStatusSub.unsubscribe(); } catch { /* 忽略 */ }
    cameraImuSub = null; cameraVioSub = null; cameraVioStatusSub = null;
    console.info('[camera] 取消遥测订阅');
  }
}

/** 断开时清空相机订阅引用(实际 unadvertise/unsubscribe 随连接关闭一起走) */
function _resetCameraSubsRefs() {
  _clearCameraCandidateTimer();
  cameraChannelSub = null; cameraCurrentChannel = null; cameraCandidateIdx = 0;
  cameraImuSub = null; cameraVioSub = null; cameraVioStatusSub = null;
}

// ── Livox 雷达内置 IMU:按需订阅(雷达页签可见 ∧ ROS 在线) ──────────
// 点云不在这里订 —— 它只在「建图开关」打开时收,见下面那段。
/** 开关雷达内置 IMU 订阅(点云不受它影响) */
export function setLidarEnabled(on) {
  lidarEnabled = !!on;
  _applyLidarDesired();
}
export function isLidarEnabled() { return lidarEnabled; }

function _applyLidarDesired() {
  if (!ros || connectionStatus !== 'online') return;
  if (lidarEnabled && !lidarImuSub) {
    lidarImuSub = new ROSLIB.Topic({
      ros, name: LIDAR_IMU_TOPIC, messageType: CAMERA_TYPE_IMU,
      compression: 'none', throttle_rate: LIDAR_IMU_THROTTLE_MS, queue_length: 1,
    });
    _subscribeWithQos(lidarImuSub, (msg) => {
      const now = Date.now();
      if (now - lastLidarImuEmitAt < LIDAR_IMU_THROTTLE_MS) return;
      lastLidarImuEmitAt = now;
      emitLidarImu(msg);
    }, CAMERA_BEST_EFFORT_QOS);
    console.info(`[lidar] 订阅内置 IMU: ${LIDAR_IMU_TOPIC} (throttle=${LIDAR_IMU_THROTTLE_MS}ms)`);
  } else if (!lidarEnabled && lidarImuSub) {
    try { lidarImuSub.unsubscribe(); } catch { /* 忽略 */ }
    lidarImuSub = null;
    lastLidarImuEmitAt = 0;
    console.info('[lidar] 取消内置 IMU 订阅');
  }
}

// ── 建图录制开关 ────────────────────────────────────────────
/** 请求开始/结束建图:发一条 Bool 给板上 lidar_recorder 节点。返回是否真的发出去了。 */
export function publishLidarRecord(on) {
  if (!lidarRecCommandPublisher) return false;
  try {
    lidarRecCommandPublisher.publish(new ROSLIB.Message({ data: !!on }));
    console.info('[lidar-rec] 📤', on ? '开始建图' : '结束建图');
    return true;
  } catch (err) {
    console.warn('[lidar-rec] 发布失败:', err);
    return false;
  }
}

/** 开关此刻是否处于"想要建图"的状态(UI 判断用) */
export function isLidarRecordDesired() { return lidarRecStreamDesired; }

/**
 * 设置"是否接收转发点云"。真正的开关状态以节点的 /lidar_recorder/status 为准,
 * 这里只决定前端订不订点云 —— 开关关掉时前端一个包都不该收。
 * **不跟随页签可见性**:切到别的页签录制要继续,回来才有连续的图可看。
 */
export function setLidarRecordStreamEnabled(on) {
  lidarRecStreamDesired = !!on;
  _applyLidarRecStream();
}

function _applyLidarRecStream() {
  if (!ros || connectionStatus !== 'online') return;
  if (lidarRecStreamDesired && !lidarRecFrameSub) {
    lidarRecFrameSub = new ROSLIB.Topic({
      ros, name: LIDAR_REC_FRAME_TOPIC, messageType: CAMERA_TYPE_POINTCLOUD2,
      // throttle_rate 0:每一帧都要 —— 录下来的 pcd 和屏幕上看到的必须是同一批点
      compression: 'none', throttle_rate: 0, queue_length: 1,
    });
    _subscribeWithQos(lidarRecFrameSub, (msg) => emitLidarFrame(msg),
      CAMERA_BEST_EFFORT_QOS);
    console.info(`[lidar-rec] 订阅转发点云: ${LIDAR_REC_FRAME_TOPIC} (best_effort, 不节流)`);
  } else if (!lidarRecStreamDesired && lidarRecFrameSub) {
    try { lidarRecFrameSub.unsubscribe(); } catch { /* 忽略 */ }
    lidarRecFrameSub = null;
    console.info('[lidar-rec] 取消转发点云订阅');
  }
}

/** 状态话题(JSON 字符串) → 解析后分发;解析失败直接丢(mock/半截包) */
function _handleLidarRecStatusImpl(msg) {
  let obj = null;
  try { obj = JSON.parse(String(msg?.data ?? '')); } catch { return; }
  if (!obj || typeof obj !== 'object') return;
  emitLidarRecStatus(obj);
}

function _resetLidarSubsRefs() {
  lidarImuSub = null;
  lidarRecFrameSub = null;
}

/** 连接 ROS。返回 Promise,连接成功 resolve,失败 reject。 */
function connectRos() {
  return new Promise((resolve, reject) => {
    if (ros && (connectionStatus === 'online' || connectionStatus === 'connecting')) {
      resolve();
      return;
    }
    setStatus('connecting');
    setControlActive(false);
    const url = getRosUrl();
    console.info('[ros] 正在连接 WebSocket:', url);

    try {
      ros = new ROSLIB.Ros({ url });
    } catch (err) {
      const msg = `roslib 初始化失败:${err?.message || err}`;
      console.error('[ros]', msg);
      setStatus('error', msg);
      reject(err);
      return;
    }

    const onConnectionError = (err) => {
      const msg = `连接错误:${err?.message || (typeof err === 'string' ? err : 'WebSocket 无法连接')}`;
      console.error('[ros]', msg, err);
      setStatus('error', msg);
      setControlActive(false);
      reject(err);
    };

    ros.on('connection', () => {
      console.info('[ros] ✅ WebSocket 已连接到 rosbridge_server。开始话题订阅...');
      setStatus('online');

      // 先列出当前 bridge 上已注册的 topics,便于排查话题名/类型名是否正确
      try {
        ros.getTopics((topicsInfo) => {
          console.info('[ros] 📋 rosbridge 当前话题列表 (topics):', topicsInfo.topics);
          console.info('[ros] 📋 对应类型 (types):', topicsInfo.types);
          for (const target of [MOTOR_STATE_TOPIC, IMU_STATE_TOPIC]) {
            const idx = (topicsInfo.topics || []).indexOf(target);
            if (idx >= 0) {
              console.info(`[ros] ✅ 话题 ${target} 存在,实际类型: ${topicsInfo.types[idx]}`);
            } else {
              console.warn(`[ros] ⚠️ 在 rosbridge 上未找到话题 ${target}。检查真机发布节点是否启动。`);
            }
          }
        }, (err) => {
          console.warn('[ros] getTopics 失败(老版本 rosbridge 可能不支持):', err);
        });
      } catch (e) { /* 忽略 getTopics 本身的同步错误 */ }

      try {
        const ROS2_F32MA = 'std_msgs/msg/Float32MultiArray';
        const ROS2_STRING = 'std_msgs/msg/String';
        const ROS2_TWIST  = 'geometry_msgs/msg/Twist';
        const ROS2_BOOL   = 'std_msgs/msg/Bool';

        // ── ① 电机状态订阅 ─────────────────────────────
        motorStateSubscriber = new ROSLIB.Topic({
          ros,
          name: MOTOR_STATE_TOPIC,
          messageType: ROS2_F32MA,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 1,
        });
        console.info('[ros] 订阅话题:', MOTOR_STATE_TOPIC, '类型=', ROS2_F32MA);
        motorStateSubscriber.subscribe((msg) => {
          if (!handleMotorStateMessage._cbHit) {
            console.info('[ros] 📥 motor_state 订阅回调首次触发! keys:', Object.keys(msg || {}));
            handleMotorStateMessage._cbHit = true;
          }
          handleMotorStateMessage(msg);
        });

        // ── ② IMU 状态订阅 ─────────────────────────────
        imuStateSubscriber = new ROSLIB.Topic({
          ros,
          name: IMU_STATE_TOPIC,
          messageType: ROS2_F32MA,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 1,
        });
        console.info('[ros] 订阅话题:', IMU_STATE_TOPIC, '类型=', ROS2_F32MA);
        imuStateSubscriber.subscribe((msg) => {
          if (!handleImuStateMessage._cbHit) {
            console.info('[ros] 📥 imu_state 订阅回调首次触发! keys:', Object.keys(msg || {}));
            handleImuStateMessage._cbHit = true;
          }
          handleImuStateMessage(msg);
        });

        // ── ③ 行为命令发布器 /rl_real/command (String) ───
        commandPublisher = new ROSLIB.Topic({
          ros,
          name: COMMAND_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          queue_size: 1,
        });
        commandPublisher.advertise();
        console.info('[ros] 注册发布器:', COMMAND_TOPIC, '类型=', ROS2_STRING);

        // ── ④ 底盘速度发布器 /rl_real/cmd_vel (Twist) ──────
        cmdVelPublisher = new ROSLIB.Topic({
          ros,
          name: CMD_VEL_TOPIC,
          messageType: ROS2_TWIST,
          compression: 'none',
          queue_size: 1,
        });
        cmdVelPublisher.advertise();
        console.info('[ros] 注册发布器:', CMD_VEL_TOPIC, '类型=', ROS2_TWIST);

        // ── ⑤ 指令应答订阅 /rl_real/feedback (String) ─────
        feedbackSubscriber = new ROSLIB.Topic({
          ros,
          name: FEEDBACK_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 10,
        });
        console.info('[ros] 订阅话题:', FEEDBACK_TOPIC, '类型=', ROS2_STRING);
        feedbackSubscriber.subscribe((msg) => _handleFeedbackMessageImpl(msg));

        // ── ⑥ 里程计位姿订阅 /rl_real/pose2d (Float32MultiArray) ──
        pose2dSubscriber = new ROSLIB.Topic({
          ros,
          name: POSE2D_TOPIC,
          messageType: ROS2_F32MA,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 1,
        });
        console.info('[ros] 订阅话题:', POSE2D_TOPIC, '类型=', ROS2_F32MA);
        pose2dSubscriber.subscribe((msg) => {
          if (_replayMutingRealCallbacks) return; // 复现时与 motor/imu 一致静默
          _handlePose2dMessageImpl(msg);
        });

        // ── ⑦ 通知事件订阅 /rl_real/notify (String) ──────────
        notifySubscriber = new ROSLIB.Topic({
          ros,
          name: NOTIFY_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 10,
        });
        console.info('[ros] 订阅话题:', NOTIFY_TOPIC, '类型=', ROS2_STRING);
        notifySubscriber.subscribe((msg) => _handleNotifyMessageImpl(msg));

        // ── ⑧ 通知回复发布器 /rl_real/notify_ack (String) ─────
        notifyAckPublisher = new ROSLIB.Topic({
          ros,
          name: NOTIFY_ACK_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          queue_size: 10,
        });
        notifyAckPublisher.advertise();
        console.info('[ros] 注册发布器:', NOTIFY_ACK_TOPIC, '类型=', ROS2_STRING);

        // ── ⑨ 站立检查触发发布器 /rl_real/check_stand (String) ──
        checkStandPublisher = new ROSLIB.Topic({
          ros,
          name: CHECK_STAND_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          queue_size: 1,
        });
        checkStandPublisher.advertise();
        console.info('[ros] 注册发布器:', CHECK_STAND_TOPIC, '类型=', ROS2_STRING);

        // ── ⑩ 站立检查结果订阅 /rl_real/check_stand_result (String) ──
        checkStandResultSubscriber = new ROSLIB.Topic({
          ros,
          name: CHECK_STAND_RESULT_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 10,
        });
        console.info('[ros] 订阅话题:', CHECK_STAND_RESULT_TOPIC, '类型=', ROS2_STRING);
        checkStandResultSubscriber.subscribe((msg) => _handleCheckStandResultImpl(msg));

        // ── ⑪ 作业交底播报发布器 /rl_briefing/play (String) ──
        briefingPublisher = new ROSLIB.Topic({
          ros,
          name: BRIEFING_PLAY_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          queue_size: 1,
        });
        briefingPublisher.advertise();
        console.info('[ros] 注册发布器:', BRIEFING_PLAY_TOPIC, '类型=', ROS2_STRING);

        // ── ⑫ 作业交底播报状态订阅 /rl_briefing/status (String) ──
        briefingStatusSubscriber = new ROSLIB.Topic({
          ros,
          name: BRIEFING_STATUS_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 10,
        });
        console.info('[ros] 订阅话题:', BRIEFING_STATUS_TOPIC, '类型=', ROS2_STRING);
        briefingStatusSubscriber.subscribe((msg) => _handleBriefingStatusImpl(msg));

        // ── ⑬ 建图录制:开关命令发布 /lidar_recorder/command (Bool) ──
        lidarRecCommandPublisher = new ROSLIB.Topic({
          ros,
          name: LIDAR_REC_COMMAND_TOPIC,
          messageType: ROS2_BOOL,
          compression: 'none',
          queue_size: 1,
        });
        lidarRecCommandPublisher.advertise();
        console.info('[ros] 注册发布器:', LIDAR_REC_COMMAND_TOPIC, '类型=', ROS2_BOOL);

        // ── ⑭ 建图录制:状态订阅 /lidar_recorder/status (String,JSON) ──
        // 订阅它有两个用处:驱动按钮的"真的在录"状态(而不是只信自己刚发的命令),
        // 以及刷新页面/重连后把 UI 恢复成节点那边的真实状态。
        lidarRecStatusSubscriber = new ROSLIB.Topic({
          ros,
          name: LIDAR_REC_STATUS_TOPIC,
          messageType: ROS2_STRING,
          compression: 'none',
          throttle_rate: 0,
          queue_length: 0,
          queue_size: 10,
        });
        console.info('[ros] 订阅话题:', LIDAR_REC_STATUS_TOPIC, '类型=', ROS2_STRING);
        lidarRecStatusSubscriber.subscribe((msg) => _handleLidarRecStatusImpl(msg));

        // 相机订阅是按需建立的:连接成功后按 UI 期望状态(相机页签是否可见)自动订阅
        _applyCameraDesired();
        // Livox 雷达内置 IMU 同理:雷达页签可见时自动订阅
        _applyLidarDesired();
        // 建图转发点云:按录制开关的期望状态(刷新/重连后自动恢复)
        _applyLidarRecStream();

        setControlActive(true);
        resolve();
      } catch (err) {
        const msg = `订阅失败:${err?.message || err}`;
        console.error('[ros]', msg);
        setStatus('error', msg);
        setControlActive(false);
        reject(err);
      }
    });

    ros.on('error', onConnectionError);

    ros.on('close', () => {
      console.warn('[ros] WebSocket 已关闭。');
      setStatus(connectionStatus === 'error' ? 'error' : 'offline', '连接已关闭');
      setControlActive(false);
      motorStateSubscriber = null;
      imuStateSubscriber = null;
      feedbackSubscriber = null;
      pose2dSubscriber = null;
    });
  });
}

/** 断开 ROS 连接。 */
function disconnectRos() {
  try {
    // 断开时恢复复现静默标志,避免 seek/回放后 muting 卡住导致重连后收不到任何数据
    try { setReplayMutingRealCallbacks(false); } catch { /* 忽略 */ }
    if (cmdVelTimerId) { clearInterval(cmdVelTimerId); cmdVelTimerId = null; }
    if (_commandBurstTimerId) { clearInterval(_commandBurstTimerId); _commandBurstTimerId = null; }
    cmdVelRequest.vx = 0; cmdVelRequest.vy = 0; cmdVelRequest.omega = 0;
    cmdVelZeroFramesRemaining = 0;
    if (motorStateSubscriber) {
      try { motorStateSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      motorStateSubscriber = null;
    }
    if (imuStateSubscriber) {
      try { imuStateSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      imuStateSubscriber = null;
    }
    if (feedbackSubscriber) {
      try { feedbackSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      feedbackSubscriber = null;
    }
    if (pose2dSubscriber) {
      try { pose2dSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      pose2dSubscriber = null;
    }
    if (notifySubscriber) {
      try { notifySubscriber.unsubscribe(); } catch { /* 忽略 */ }
      notifySubscriber = null;
    }
    if (notifyAckPublisher) {
      try { notifyAckPublisher.unadvertise(); } catch { /* 忽略 */ }
      notifyAckPublisher = null;
    }
    if (checkStandPublisher) {
      try { checkStandPublisher.unadvertise(); } catch { /* 忽略 */ }
      checkStandPublisher = null;
    }
    if (checkStandResultSubscriber) {
      try { checkStandResultSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      checkStandResultSubscriber = null;
    }
    if (briefingPublisher) {
      try { briefingPublisher.unadvertise(); } catch { /* 忽略 */ }
      briefingPublisher = null;
    }
    if (briefingStatusSubscriber) {
      try { briefingStatusSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      briefingStatusSubscriber = null;
    }
    if (lidarRecCommandPublisher) {
      try { lidarRecCommandPublisher.unadvertise(); } catch { /* 忽略 */ }
      lidarRecCommandPublisher = null;
    }
    if (lidarRecStatusSubscriber) {
      try { lidarRecStatusSubscriber.unsubscribe(); } catch { /* 忽略 */ }
      lidarRecStatusSubscriber = null;
    }
    // 相机订阅随连接一起释放,只清引用(期望状态保留,重连后 _applyCameraDesired 自动恢复)
    _resetCameraSubsRefs();
    // Livox 雷达内置 IMU / 建图转发点云同理
    _resetLidarSubsRefs();
    if (commandPublisher) {
      try { commandPublisher.unadvertise(); } catch { /* 忽略 */ }
      commandPublisher = null;
    }
    if (cmdVelPublisher) {
      try { cmdVelPublisher.unadvertise(); } catch { /* 忽略 */ }
      cmdVelPublisher = null;
    }
    if (ros) {
      try { ros.close(); } catch { /* 忽略 */ }
      ros = null;
    }
  } finally {
    messageCount = 0;
    lastBlockCount = 0;
    lastMessageAt = 0;
    imuMessageCount = 0;
    lastImuMessageAt = 0;
    commandPublishCount = 0;
    lastCommandPublishAt = 0;
    lastCommandPayload = '';
    cmdVelPublishCount = 0;
    lastCmdVelPublishAt = 0;
    lastCmdVel = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    cmdVelRequest.vx = 0; cmdVelRequest.vy = 0; cmdVelRequest.omega = 0; cmdVelRequest.vz = 0;
    notifyMessageCount = 0;
    lastNotifySummary = '';
    handleMotorStateMessage._warnedEmpty = false;
    handleMotorStateMessage._warnedModel = false;
    handleMotorStateMessage._warnedIdx = false;
    handleMotorStateMessage._loggedFirst = false;
    handleMotorStateMessage._cbHit = false;
    handleImuStateMessage._warned = false;
    handleImuStateMessage._loggedFirst = false;
    handleImuStateMessage._cbHit = false;
    // 清空 IMU 的显示值,避免断开后还保留旧数
    const imu = shared.imu;
    imu.quat.w = 1; imu.quat.x = imu.quat.y = imu.quat.z = 0;
    imu.rpy.roll = imu.rpy.pitch = imu.rpy.yaw = 0;
    imu.acc.x = imu.acc.y = imu.acc.z = 0;
    imu.gyro.x = imu.gyro.y = imu.gyro.z = 0;
    imu.pos.north = imu.pos.east = imu.pos.down = 0;
    imu.vel.x = imu.vel.y = imu.vel.z = 0;
    imu.msgCount = 0; imu.lastMsgAt = 0;
    setStatus('offline');
    setControlActive(false);
  }
}

/** 切换连接:在线则断开,离线则连接。 */
export async function toggleConnection() {
  if (connectionStatus === 'online' || connectionStatus === 'connecting') {
    disconnectRos();
    return false;
  }
  await connectRos();
  return true;
}

// ─────────────────────────────────────────────
//   Publishers: 行为命令 (rl_real/command) + 底盘速度 (cmd_vel)
// ─────────────────────────────────────────────

/** 立即发一帧 String 命令;未连接时直接 return false,便于调试 + 状态计数。 */
function publishOneCommand(dataString) {
  if (!ros || !commandPublisher) return false;
  try {
    const msg = new ROSLIB.Message({ data: dataString });
    commandPublisher.publish(msg);
    commandPublishCount += 1;
    lastCommandPublishAt = performance.now();
    lastCommandPayload = dataString;
    emitStatus();
    return true;
  } catch (err) {
    console.warn('[ros] publish command failed:', err);
    return false;
  }
}

/**
 * 发布行为命令(对应 CLI: ros2 topic pub --times 3 --rate 2 /rl_real/command String "{data: xxx}")
 *  - times=3, interval=500ms,总计 1.0s
 *  - 同一时刻新的行为命令进来会 cancel 上一个未发完的 burst(避免叠发)
 * @param {string} cmd 允许值:'getup' | 'getdown' | 'locomotion' | 'vel_stop' | 'zero'
 * @returns {boolean} true = 已开始发布;false = 未连接/参数非法
 */
let _commandBurstTimerId = null;
export function publishBehaviorCommand(cmd) {
  const allowed = new Set(['getup', 'getdown', 'locomotion', 'vel_stop', 'zero']);
  if (!allowed.has(cmd)) {
    console.warn('[ros] publishBehaviorCommand:非法命令', cmd);
    return false;
  }
  if (connectionStatus !== 'online' || !commandPublisher) {
    console.warn('[ros] publishBehaviorCommand 跳过:ROS 未连接');
    return false;
  }
  if (_commandBurstTimerId) {
    clearInterval(_commandBurstTimerId);
    _commandBurstTimerId = null;
  }
  let remaining = 3;
  const tick = () => {
    publishOneCommand(cmd);
    console.debug(`[ros] 📤 publish '${cmd}' (${3 - remaining + 1}/3)`);
    remaining -= 1;
    if (remaining <= 0 && _commandBurstTimerId) {
      clearInterval(_commandBurstTimerId);
      _commandBurstTimerId = null;
    }
  };
  tick(); // CLI 不等待,立即发第 1 帧
  _commandBurstTimerId = setInterval(tick, COMMAND_INTERVAL_MS);
  return true;
}

/** 立即发一帧 Twist 到 /rl_real/cmd_vel(vz = linear.z 垂直速度) */
function publishOneTwist(vx, vy, omega, vz = 0) {
  if (!ros || !cmdVelPublisher) return false;
  const x = Number(vx) || 0;
  const y = Number(vy) || 0;
  const z = Number(omega) || 0;
  const w = Number(vz) || 0;
  try {
    const msg = new ROSLIB.Message({
      linear:  { x, y, z: w },
      angular: { x: 0, y: 0, z },
    });
    cmdVelPublisher.publish(msg);
    cmdVelPublishCount += 1;
    lastCmdVelPublishAt = performance.now();
    lastCmdVel = {
      linear:  { x, y, z: w },
      angular: { x: 0, y: 0, z },
    };
    // 避免 UI 每 50ms 重绘整颗顶栏胶囊(太占主线程),cmdVel 计数改 ~1fps 才 emit
    if (cmdVelPublishCount % 20 === 1) emitStatus();
    return true;
  } catch (err) {
    console.warn('[ros] publish cmd_vel failed:', err);
    return false;
  }
}

/** 启动 cmd_vel 20Hz keep-alive 定时器(若未启动) */
function ensureCmdVelTimer() {
  if (cmdVelTimerId) return;
  cmdVelTimerId = setInterval(() => {
    const { vx, vy, omega, vz } = cmdVelRequest;
    const isZero = Math.abs(vx) < 1e-6 && Math.abs(vy) < 1e-6 && Math.abs(omega) < 1e-6 && Math.abs(vz) < 1e-6;
    if (isZero) {
      if (cmdVelZeroFramesRemaining > 0) {
        publishOneTwist(0, 0, 0, 0);
        cmdVelZeroFramesRemaining -= 1;
      } else {
        // 已经是 0,且零速度尾帧都发完了 → 停 timer 省 CPU
        if (cmdVelTimerId) { clearInterval(cmdVelTimerId); cmdVelTimerId = null; }
      }
    } else {
      // 非零 → 保持 20Hz 持续发布
      publishOneTwist(vx, vy, omega, vz);
    }
  }, CMD_VEL_INTERVAL_MS);
}

/**
 * 设置下一个持续对外发布的 cmd_vel(对应 CLI: vel --x --y --yaw)
 *  - 非零:启动 20Hz keep-alive 定时器,每 50ms 发一帧(等价 --rate 20)
 *  - 归零:按 CLI --times 5 --rate 20 的停止约定,再追加 5 帧零速度才停 timer(避免真机收到一次零后又被 keepalive 逻辑卡)
 * @param {number} vx linear.x (m/s)
 * @param {number} vy linear.y (m/s)
 * @param {number} omega angular.z (rad/s)
 * @param {number=} vz linear.z 垂直速度 (m/s);不传(undefined)则保持当前值,传数字则覆盖
 * @returns {boolean} true=已接受请求,false=ROS 未启用
 */
export function setCmdVelRequest(vx, vy, omega, vz) {
  if (connectionStatus !== 'online' || !cmdVelPublisher) return false;
  cmdVelRequest.vx = Number(vx) || 0;
  cmdVelRequest.vy = Number(vy) || 0;
  cmdVelRequest.omega = Number(omega) || 0;
  if (vz !== undefined) cmdVelRequest.vz = Number(vz) || 0;
  const isZero =
    Math.abs(cmdVelRequest.vx) < 1e-6 &&
    Math.abs(cmdVelRequest.vy) < 1e-6 &&
    Math.abs(cmdVelRequest.omega) < 1e-6 &&
    Math.abs(cmdVelRequest.vz) < 1e-6;
  if (isZero) {
    cmdVelZeroFramesRemaining = 5; // 尾帧:发 5 帧零速度(对齐 CLI --times 5 --rate 20)
  } else {
    cmdVelZeroFramesRemaining = 0; // 非零时清掉"尾帧计数",避免停掉新的速度请求
  }
  ensureCmdVelTimer();
  return true;
}

/** 单独设置垂直速度 linear.z(摇杆/平移滑块不影响该通道;复位与停止会清零) */
export function setCmdVelVz(vz) {
  return setCmdVelRequest(cmdVelRequest.vx, cmdVelRequest.vy, cmdVelRequest.omega, vz);
}

/** 手动"停止底盘" = 立即全部置零(含垂直速度) + 安排 5 帧零 Twist 尾帧(更直观) */
export function stopCmdVel() {
  return setCmdVelRequest(0, 0, 0, 0);
}

/** 读当前对外的 cmd_vel 请求(UI/调试用) */
export function getCmdVelRequest() {
  return { ...cmdVelRequest };
}
