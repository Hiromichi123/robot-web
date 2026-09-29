/**
 * 模拟 ROS2 rosbridge_server (仅用于本地调试,不参与正式功能)
 *
 * 功能:
 *  - 监听 0.0.0.0:9090 (WebSocket, 兼容 rosbridge v2 JSON 协议,局域网可访问)
 *  - 话题订阅: 接受 roslib subscribe → 持续推送模拟数据
 *    ① /rl_real/motor_state  (std_msgs/msg/Float32MultiArray)  ~50Hz
 *    ② /rl_real/imu_state    (std_msgs/msg/Float32MultiArray)  ~50Hz
 *  - 话题发布: 接收 roslib advertise/publish → 控制台打印
 *    ① /rl_real/command (std_msgs/msg/String): 行为命令
 *    ② /rl_real/cmd_vel (geometry_msgs/msg/Twist): 底盘速度
 *  - 服务 /rosapi/topics: 返回已注册话题(便于 connect 后 getTopics 调试)
 *
 * 运行方式:
 *   node js/_mock-rosbridge.js
 *
 * 然后前端页面上点击「连接 ROS」(ros-bridge.js 默认连接 ws://192.168.5.101:9090,
 * 如需指向本地 127.0.0.1:9090,可把 js/ros-bridge.js 的 ROS_URL 临时改一下,
 * 或在本地 hosts 把 192.168.5.101 解析到 127.0.0.1)。
 */
const http = require('http');
const os = require('os');
const crypto = require('crypto');
const EventEmitter = require('events');

// ─────────────────────────────────────────────────────────
//  常量
// ─────────────────────────────────────────────────────────
// 监听地址:默认 0.0.0.0(所有网卡,局域网内手机/真机可连);
// 只想本机回环可用 MOCK_ROS_HOST=127.0.0.1 覆盖
const HOST = process.env.MOCK_ROS_HOST || '0.0.0.0';
// 端口: 环境变量 MOCK_ROS_PORT 或命令行参数(node _mock-rosbridge.js 9091)可覆盖,默认 9090
const PORT = Number(process.env.MOCK_ROS_PORT || process.argv[2] || 9090);
const MOTOR_HZ = 50;    // 电机状态推送频率
const IMU_HZ   = 50;    // IMU 状态推送频率

// 四足机器人 16 关节(模型 2)
const QUAD_JOINT_NAMES = [
  'FL_hip', 'FL_thigh', 'FL_calf', 'FL_foot',
  'FR_hip', 'FR_thigh', 'FR_calf', 'FR_foot',
  'RL_hip', 'RL_thigh', 'RL_calf', 'RL_foot',
  'RR_hip', 'RR_thigh', 'RR_calf', 'RR_foot',
];
// 人形机器人 23 关节(模型 1)
const HUMANOID_JOINT_NAMES = [
  'fl_steering', 'fr_steering', 'bl_steering', 'br_steering',
  'lift',
  'robot_left_joint1', 'robot_left_joint2', 'robot_left_joint3',
  'robot_left_joint4', 'robot_left_joint5', 'robot_left_joint6', 'robot_left_joint7',
  'robot_left_finger',
  'robot_right_joint1', 'robot_right_joint2', 'robot_right_joint3',
  'robot_right_joint4', 'robot_right_joint5', 'robot_right_joint6', 'robot_right_joint7',
  'robot_right_finger',
  'robot_head_pitch', 'robot_head_yaw',
];
const JOINT_COUNT = 23;   // 电机块数 = max(人形, 四足) = 23, 前端按模型映射只取前 16/23

// 关节零位(中立姿态),用于 real_pos 起始;后续加小正弦扰动模拟"机器人轻微摆动"
const JOINT_HOME = new Array(JOINT_COUNT).fill(0);
// 给几个关节加个初始小偏置,视觉上更自然
const JOINT_BIAS = [
  // 人形四腿 hip 略偏
  0.0, 0.0, 0.0, 0.0,
  // lift 居中高度(-0.35 ~ 0, 取中间)
  -0.15,
  // 左臂 J1~J7:轻微屈肘姿态
  -0.2, -0.6, 0.3, 0.9, -0.2, 0.1, -0.1,
  // 左手夹爪
  0.0,
  // 右臂 J1~J7:对称
  0.2, 0.6, -0.3, 0.9, 0.2, -0.1, 0.1,
  // 右手夹爪
  0.0,
  // 头部: 平视
  0.0, 0.0,
];

// ─────────────────────────────────────────────────────────
//  全局事件总线: 模拟"话题发布/订阅"
// ─────────────────────────────────────────────────────────
const bus = new EventEmitter();

// ─────────────────────────────────────────────────────────
//  WebSocket 服务端 (Node.js 内置 http + 手动 Upgrade 握手 + 数据帧编解码)
//  实现参考 RFC 6455 最小子集(仅支持 text 帧,足够 rosbridge JSON 协议)
// ─────────────────────────────────────────────────────────
const clients = new Set();

function computeAcceptKey(wsKey) {
  const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';
  return crypto.createHash('sha1').update(wsKey + GUID).digest('base64');
}

/** 编码 WebSocket text 帧(未掩码,服务端→客户端不需要掩码) */
function encodeTextFrame(payload) {
  const data = Buffer.from(payload, 'utf8');
  const len = data.length;
  let header;
  if (len < 126) {
    header = Buffer.alloc(2);
    header[0] = 0x81; // FIN=1, opcode=1(text)
    header[1] = len;
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[0] = 0x81;
    header[1] = 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[0] = 0x81;
    header[1] = 127;
    header.writeUInt32BE(0, 2);
    header.writeUInt32BE(len, 6);
  }
  return Buffer.concat([header, data]);
}

/** 解码一帧客户端消息(必须有掩码)。返回 null 表示缓冲区不够。 */
function decodeFrame(buf) {
  if (buf.length < 2) return null;
  const firstByte = buf[0];
  const secondByte = buf[1];
  const fin = (firstByte & 0x80) !== 0;
  const opcode = firstByte & 0x0F;
  const masked = (secondByte & 0x80) !== 0;
  let payloadLen = secondByte & 0x7F;
  let offset = 2;
  if (payloadLen === 126) {
    if (buf.length < offset + 2) return null;
    payloadLen = buf.readUInt16BE(offset);
    offset += 2;
  } else if (payloadLen === 127) {
    if (buf.length < offset + 8) return null;
    // 超过 4GB 没必要处理
    const hi = buf.readUInt32BE(offset);
    const lo = buf.readUInt32BE(offset + 4);
    if (hi !== 0) throw new Error('payload too large');
    payloadLen = lo;
    offset += 8;
  }
  let maskKey = null;
  if (masked) {
    if (buf.length < offset + 4) return null;
    maskKey = buf.slice(offset, offset + 4);
    offset += 4;
  }
  if (buf.length < offset + payloadLen) return null;
  const payload = Buffer.alloc(payloadLen);
  buf.copy(payload, 0, offset, offset + payloadLen);
  if (masked && maskKey) {
    for (let i = 0; i < payloadLen; i += 1) {
      payload[i] ^= maskKey[i % 4];
    }
  }
  const rest = buf.slice(offset + payloadLen);
  // close frame opcode=8
  if (opcode === 8) return { opcode: 'close', payload, rest };
  if (opcode === 0x9) return { opcode: 'ping', payload, rest };
  if (opcode === 0xA) return { opcode: 'pong', payload, rest };
  if (opcode === 1) return { opcode: 'text', text: payload.toString('utf8'), rest, fin };
  // 二进制帧(0x2)/ continuation(0x0) 忽略
  return { opcode: 'skip', rest };
}

/**
 * 为每个连接创建一个 WebSocket peer。
 * 对外暴露:
 *   peer.sendJSON(obj)  发送 JSON text 帧
 *   peer.close()        主动关闭
 * 事件:
 *   'message' (jsonObj) 收到一条完整 JSON 消息
 *   'close'
 */
function createPeer(socket) {
  let recvBuf = Buffer.alloc(0);
  let closed = false;
  const peer = new EventEmitter();

  peer.sendJSON = (obj) => {
    if (closed) return;
    try {
      socket.write(encodeTextFrame(JSON.stringify(obj)));
    } catch (e) { /* 忽略发送错误 */ }
  };
  peer.close = () => {
    if (closed) return;
    closed = true;
    try {
      const closePayload = Buffer.alloc(2);
      closePayload.writeUInt16BE(1000, 0);
      const head = Buffer.from([0x88, 0x02]); // FIN + close opcode, len=2
      socket.write(Buffer.concat([head, closePayload]));
    } catch { /* ignore */ }
    try { socket.end(); } catch { /* ignore */ }
  };

  socket.on('data', (chunk) => {
    recvBuf = recvBuf.length === 0 ? chunk : Buffer.concat([recvBuf, chunk]);
    try {
      while (recvBuf.length > 0) {
        const before = recvBuf.length;
        const frame = decodeFrame(recvBuf);
        if (!frame) return; // 不够一帧,等更多数据
        recvBuf = frame.rest;
        if (frame.opcode === 'close') {
          peer.close();
          return;
        }
        if (frame.opcode === 'ping') {
          if (!closed) {
            try {
              const head = Buffer.from([0x8A, frame.payload.length]); // pong
              socket.write(Buffer.concat([head, frame.payload]));
            } catch { /* ignore */ }
          }
          continue;
        }
        if (frame.opcode === 'text' && frame.fin) {
          try {
            const obj = JSON.parse(frame.text);
            peer.emit('message', obj);
          } catch (e) {
            console.warn('[mock-ros] 收到非JSON帧:', frame.text.slice(0, 80));
          }
        }
        if (recvBuf.length === before) {
          // 没有推进,避免死循环
          break;
        }
      }
    } catch (e) {
      console.warn('[mock-ros] 帧解码出错,断开连接:', e.message);
      peer.close();
    }
  });
  socket.on('end', () => { if (!closed) { closed = true; peer.emit('close'); } });
  socket.on('error', () => { if (!closed) { closed = true; peer.emit('close'); } });
  socket.on('close', () => { if (!closed) { closed = true; peer.emit('close'); } });
  return peer;
}

// ─────────────────────────────────────────────────────────
//  HTTP + Upgrade 服务器
// ─────────────────────────────────────────────────────────
const server = http.createServer((req, res) => {
  // 普通 HTTP 请求:返回一个小的健康检查页面
  res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
  res.end(`
    <h1>Mock rosbridge_server</h1>
    <p>WebSocket 端点: <code>ws://${HOST}:${PORT}/</code></p>
    <p>请在前端仿真页面点击「连接 ROS」(需把 js/ros-bridge.js 的 ROS_URL 指向本机或配置 hosts)</p>
    <ul>
      <li>Motor topic: /rl_real/motor_state @ ${MOTOR_HZ}Hz (${JOINT_COUNT} blocks × 10 floats)</li>
      <li>IMU   topic: /rl_real/imu_state   @ ${IMU_HZ}Hz (19 floats)</li>
      <li>Cmd   topic: /rl_real/command (std_msgs/msg/String)  → console log</li>
      <li>Vel   topic: /rl_real/cmd_vel (geometry_msgs/msg/Twist) → console log</li>
    </ul>
    <p>前端可监听的状态变化(浏览器控制台):</p>
    <pre>ros-bridge.js 会打印首帧消息/订阅回调触发等调试日志</pre>
  `);
});

server.on('upgrade', (req, socket, head) => {
  const wsKey = req.headers['sec-websocket-key'];
  if (!wsKey || req.headers['upgrade']?.toLowerCase() !== 'websocket') {
    socket.write('HTTP/1.1 400 Bad Request\r\n\r\n');
    try { socket.destroy(); } catch { /* ignore */ }
    return;
  }
  const accept = computeAcceptKey(wsKey);
  const handshake = [
    'HTTP/1.1 101 Switching Protocols',
    'Upgrade: websocket',
    'Connection: Upgrade',
    `Sec-WebSocket-Accept: ${accept}`,
    '\r\n',
  ].join('\r\n');
  socket.write(handshake);
  if (head && head.length) socket.unshift(head);

  // 创建 peer 并挂到客户端集合
  const peer = createPeer(socket);
  // 每个客户端订阅的 topic 集合(Set<string>)
  const subscriptions = new Set();
  // 每个客户端 advertise 的 topic → type
  const advertised = new Map();

  const onTopicPublished = (topic, msgObj) => {
    if (!subscriptions.has(topic)) return;
    peer.sendJSON({ op: 'publish', topic, msg: msgObj });
  };
  bus.on('publish', onTopicPublished);

  peer.on('message', (msg) => handleRosbridgeMessage(peer, msg, subscriptions, advertised));
  peer.on('close', () => {
    bus.off('publish', onTopicPublished);
    clients.delete(peer);
    console.log(`[mock-ros] 👋 客户端断开。当前在线 ${clients.size}`);
  });

  clients.add(peer);
  console.log(`[mock-ros] ✅ 新客户端连接。当前在线 ${clients.size}`);
  // 连接 2 秒后发一条欢迎通知(便于验证 UI 通知通道)
  setTimeout(() => {
    if (clients.has(peer)) {
      sendNotify('info', 'Mock rosbridge 已连接,通知通道正常');
    }
  }, 2000);
});

// ─────────────────────────────────────────────────────────
//  rosbridge v2 协议消息处理
// ─────────────────────────────────────────────────────────
function handleRosbridgeMessage(peer, msg, subscriptions, advertised) {
  const op = msg?.op;
  if (!op) return;
  switch (op) {
    // roslib 连接时一般不发 op=connect(新版默认直接建立 WebSocket 就视为连接)
    case 'connect': {
      peer.sendJSON({ op: 'connected' });
      break;
    }
    case 'subscribe': {
      const { topic } = msg;
      if (!topic) return;
      subscriptions.add(topic);
      const qos = msg.qos ? ` qos=${msg.qos.reliability || '?'}` : '';
      console.log(`[mock-ros] 📡 客户端订阅 ${topic}${qos}`);
      break;
    }
    case 'unsubscribe': {
      const { topic } = msg;
      if (!topic) return;
      subscriptions.delete(topic);
      console.log(`[mock-ros] 📡 客户端取消订阅 ${topic}`);
      break;
    }
    case 'advertise': {
      const { topic, type } = msg;
      if (!topic) return;
      advertised.set(topic, type || '');
      console.log(`[mock-ros] 📢 客户端 advertise ${topic} (type=${type || '?'})`);
      break;
    }
    case 'unadvertise': {
      const { topic } = msg;
      if (!topic) return;
      advertised.delete(topic);
      console.log(`[mock-ros] 📢 客户端 unadvertise ${topic}`);
      break;
    }
    case 'publish': {
      const { topic, msg: payload } = msg;
      if (!topic) return;
      // 转发到总线(如果有其他订阅者)
      bus.emit('publish', topic, payload || {});
      // 特定话题:控制台高亮打印
      if (topic === '/rl_real/command') {
        const text = payload?.data ?? String(payload);
        const ts = new Date().toISOString().slice(11, 23);
        console.log(`[mock-ros] ⬆️  [${ts}] /rl_real/command  data='${text}'`);
      } else if (topic === '/rl_real/cmd_vel') {
        const lin = payload?.linear || { x: 0, y: 0, z: 0 };
        const ang = payload?.angular || { x: 0, y: 0, z: 0 };
        const ts = new Date().toISOString().slice(11, 23);
        console.log(
          `[mock-ros] ⬆️  [${ts}] /rl_real/cmd_vel  lin=(x=${Number(lin.x).toFixed(3)}, y=${Number(lin.y).toFixed(3)}, z=${Number(lin.z).toFixed(3)})  ` +
          `ang=(z=${Number(ang.z).toFixed(3)} rad/s)`
        );
      } else {
        console.log(`[mock-ros] ⬆️  客户端 publish ${topic}`);
      }
      break;
    }
    case 'call_service': {
      // ros.getTopics() 会调用 /rosapi/topics 服务
      const { service, id } = msg;
      if (service === '/rosapi/topics' || service === 'rosapi/topics') {
        peer.sendJSON({
          op: 'service_response',
          service,
          id,
          values: {
            topics: [
              '/rl_real/motor_state',
              '/rl_real/imu_state',
              '/rl_real/command',
              '/rl_real/cmd_vel',
              '/rl_real/notify',
              '/rl_real/notify_ack',
              '/rl_real/check_stand',
              '/rl_real/check_stand_result',
              '/rl_briefing/play',
              '/rl_briefing/status',
              // 相机(Insight 9):与真机话题表一致 —— raw 系列只注册不发流,rect 系列才有数据
              '/camera/camera/color/camera_info',
              '/camera/camera/color/image_raw/compressed',
              '/camera/camera/color/image_rect_raw/compressed',
              '/camera/camera/imu',
              '/camera/camera/infra1/camera_info',
              '/camera/camera/infra1/image_raw',
              '/camera/camera/infra1/image_rect_raw',
              '/camera/camera/infra2/camera_info',
              '/camera/camera/infra2/image_raw',
              '/camera/camera/infra2/image_rect_raw',
              '/camera/camera/vio_100hz',
              '/camera/camera/vio_status',
              // Livox MID-360:3D 点云 + 雷达内置 BMI088 IMU
              '/livox/lidar',
              '/livox/imu',
            ],
            types: [
              'std_msgs/msg/Float32MultiArray',
              'std_msgs/msg/Float32MultiArray',
              'std_msgs/msg/String',
              'geometry_msgs/msg/Twist',
              'std_msgs/msg/String',
              'std_msgs/msg/String',
              'std_msgs/msg/String',
              'std_msgs/msg/String',
              'std_msgs/msg/String',
              'std_msgs/msg/String',
              'sensor_msgs/msg/CameraInfo',
              'sensor_msgs/msg/CompressedImage',
              'sensor_msgs/msg/CompressedImage',
              'sensor_msgs/msg/Imu',
              'sensor_msgs/msg/CameraInfo',
              'sensor_msgs/msg/Image',
              'sensor_msgs/msg/Image',
              'sensor_msgs/msg/CameraInfo',
              'sensor_msgs/msg/Image',
              'sensor_msgs/msg/Image',
              'geometry_msgs/msg/PoseStamped',
              'std_msgs/msg/String',
              'sensor_msgs/msg/PointCloud2',
              'sensor_msgs/msg/Imu',
            ],
          },
          result: true,
        });
        console.log('[mock-ros] 🔍 响应 /rosapi/topics 查询');
        return;
      }
      // 其他服务:直接返回 success=false
      peer.sendJSON({ op: 'service_response', service, id: id || null, values: {}, result: false });
      console.log(`[mock-ros] 🔍 未实现服务 ${service} → 返回 result=false`);
      break;
    }
    case 'set_level':
      // roslib 调试等级设置,忽略
      break;
    default:
      console.log(`[mock-ros] ⚠️  未知 op=${op}  msg=`, JSON.stringify(msg).slice(0, 160));
  }
}

// ─────────────────────────────────────────────────────────
//  模拟电机数据生成
// ─────────────────────────────────────────────────────────
const startTimeMs = Date.now();
let motorFrame = 0;
// 每个关节 last_real_pos:一阶低通模拟跟踪滞后 (τ≈120ms)
const _motorLastReal = new Array(JOINT_COUNT).fill(null);
// 每个关节温度(°C):一阶热惯性,τ≈8s(升温慢/散热慢),初始 38°C 环境温度
const _motorLastTemp = new Array(JOINT_COUNT).fill(38.0);

function buildMotorStateMessage() {
  const tSec = (Date.now() - startTimeMs) / 1000;
  motorFrame += 1;
  const dt = 1 / MOTOR_HZ; // 采样间隔 (秒)
  const tau = 0.12;        // 低通时间常数 (秒),≈120ms 跟踪滞后
  const alpha = 1 - Math.exp(-dt / tau);
  const tempAlpha = 1 - Math.exp(-dt / 8); // 温度热惯性时间常数 ≈8s
  const data = new Array(JOINT_COUNT * 10);
  for (let i = 0; i < JOINT_COUNT; i += 1) {
    const base = i * 10;
    data[base + 0] = i;
    const home = (JOINT_BIAS[i] ?? JOINT_HOME[i] ?? 0);
    const freq = 0.3 + (i % 7) * 0.11;
    const amp  = 0.04 + ((i * 13) % 7) * 0.008;
    const phase = (i * 0.53) % (Math.PI * 2);
    const wave = amp * Math.sin(2 * Math.PI * freq * tSec + phase);
    const targetPos = home + wave;
    // ── 实际位置 = 低通滞后 targetPos + 关节级偏置 + 测量噪声 ──
    // 1) 一阶低通:模拟电机响应时间,产生 phase/amplitude 滞后使曲线分离
    if (_motorLastReal[i] === null) _motorLastReal[i] = targetPos;
    const lp = _motorLastReal[i] + alpha * (targetPos - _motorLastReal[i]);
    // 2) 关节级跟踪偏置:小幅度与 target 不同频率的正弦 (~10-30% amp ratio)
    const biasFreq = 0.7 + (i % 5) * 0.23;
    const biasAmp  = amp * (0.1 + ((i * 11) % 9) * 0.025); // 10%~30% of wave amp
    const biasPh   = (i * 1.17) % (Math.PI * 2);
    const bias = biasAmp * Math.sin(2 * Math.PI * biasFreq * tSec + biasPh);
    // 3) 测量噪声 (±0.004 rad,比原先 ±0.0015 稍大更可见)
    const noise = (Math.random() - 0.5) * 0.004;
    const realPos = lp + bias + noise;
    _motorLastReal[i] = realPos;
    data[base + 1] = targetPos;
    data[base + 2] = realPos;
    // 速度: 一阶差分 (target/real 分别)
    const prevFrameData = null; // 暂简化为 0,对曲线显示无影响
    data[base + 3] = 0; // target_vel
    data[base + 4] = 0; // real_vel
    // 力矩: tau 随偏差 (target-real) 轻微波动 + 随机
    const err = targetPos - realPos;
    data[base + 5] = 0;                // target_tau
    data[base + 6] = err * 60 + (Math.random() - 0.5) * 0.12 + 0.1; // real_tau ≈ Kp*err + noise
    data[base + 7] = 20 + ((i * 7) % 10);
    data[base + 8] = 0.8 + ((i * 3) % 5) * 0.1;
    // 温度: 一阶热惯性逼近"负载目标温度"(环境 36°C + 电机固有温差 + |力矩|发热)
    const tempTarget = 36 + (i % 5) * 1.2 + Math.abs(data[base + 6]) * 1.8;
    _motorLastTemp[i] += tempAlpha * (tempTarget - _motorLastTemp[i]);
    data[base + 9] = Math.round(_motorLastTemp[i] * 10) / 10;
  }
  return {
    layout: {
      dim: [
        { label: `motors_${JOINT_COUNT}x10`, size: JOINT_COUNT, stride: JOINT_COUNT * 10 },
      ],
      data,
    },
  };
}

// ─────────────────────────────────────────────────────────
//  模拟 IMU 数据生成(19 字段)
// ─────────────────────────────────────────────────────────
let imuFrame = 0;
function buildImuStateMessage() {
  const tSec = (Date.now() - startTimeMs) / 1000;
  imuFrame += 1;
  // 轻微机身摆动 (roll~±1deg, pitch~±1.5deg, yaw 缓慢旋转)
  const roll  = THREE_DegToRad(1.0) * Math.sin(2 * Math.PI * 0.22 * tSec);
  const pitch = THREE_DegToRad(1.5) * Math.cos(2 * Math.PI * 0.18 * tSec + 0.3);
  const yaw   = THREE_DegToRad(12) * Math.sin(2 * Math.PI * 0.04 * tSec);
  // RPY → 四元数(ZYX 顺序)
  const { w, x, y, z } = eulerToQuatZYX(roll, pitch, yaw);
  // 加速度: 重力 + 微扰
  const g = 9.81;
  const ax = +Math.sin(pitch) * g + (Math.random() - 0.5) * 0.05;
  const ay = -Math.sin(roll) * Math.cos(pitch) * g + (Math.random() - 0.5) * 0.05;
  const az = +Math.cos(roll) * Math.cos(pitch) * g + (Math.random() - 0.5) * 0.05;
  // 陀螺仪(与 RPY 差分一致): d(roll)/dt 等
  const gx = THREE_DegToRad(1.0) * 2 * Math.PI * 0.22 * Math.cos(2 * Math.PI * 0.22 * tSec);
  const gy = -THREE_DegToRad(1.5) * 2 * Math.PI * 0.18 * Math.sin(2 * Math.PI * 0.18 * tSec + 0.3);
  const gz = THREE_DegToRad(12) * 2 * Math.PI * 0.04 * Math.cos(2 * Math.PI * 0.04 * tSec);
  // NED 位置: 缓慢漂移 + 原点
  const north = 0.02 * Math.sin(2 * Math.PI * 0.025 * tSec);
  const east  = 0.02 * Math.cos(2 * Math.PI * 0.03 * tSec);
  const down  = 0.0;
  // Body 速度
  const vbx = 0.01 * Math.sin(2 * Math.PI * 0.08 * tSec);
  const vby = 0.01 * Math.cos(2 * Math.PI * 0.09 * tSec + 0.5);
  const vbz = 0.0;

  const data = new Array(19);
  data[0] = w;  data[1] = x;  data[2] = y;  data[3] = z;  // quat wxyz
  data[4] = roll;  data[5] = pitch; data[6] = yaw;        // RPY rad
  data[7] = ax;  data[8] = ay;  data[9] = az;            // Acc m/s²
  data[10] = gx; data[11] = gy; data[12] = gz;           // Gyro rad/s
  data[13] = north; data[14] = east; data[15] = down;    // NED pos m
  data[16] = vbx; data[17] = vby; data[18] = vbz;        // Body vel m/s
  return {
    layout: {
      dim: [{
        label: 'quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pos_north,pos_east,pos_down,vel_body_x,vel_body_y,vel_body_z',
        size: 19,
        stride: 19,
      }],
      data_offset: 0,
    },
    data,
  };
}

// 数学辅助函数
function THREE_DegToRad(d) { return d * Math.PI / 180; }
function eulerToQuatZYX(roll, pitch, yaw) {
  // intrinsic Z(yaw) → Y(pitch) → X(roll), 与 Three.Euler 默认 'XYZ' 相反,手算:
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2),  sy = Math.sin(yaw / 2);
  const w =  cy * cp * cr + sy * sp * sr;
  const x =  cy * cp * sr - sy * sp * cr;
  const y =  sy * cp * sr + cy * sp * cr;
  const z =  sy * cp * cr - cy * sp * sr;
  return { w, x, y, z };
}

// ─────────────────────────────────────────────────────────
//  启动定时推送
// ─────────────────────────────────────────────────────────
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/rl_real/motor_state', buildMotorStateMessage());
}, Math.round(1000 / MOTOR_HZ));

setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/rl_real/imu_state', buildImuStateMessage());
}, Math.round(1000 / IMU_HZ));

// ─────────────────────────────────────────────────────────
//  模拟 Insight 9 相机话题(均为 best_effort QoS,按订阅过滤推送)
//  color: CompressedImage(JPEG) / infra1,2: raw mono8 Image
//  imu: sensor_msgs/Imu / vio: nav_msgs/Odometry / vio_status: String
// ─────────────────────────────────────────────────────────
const CAM_W = 192;
const CAM_H = 144;
const MOCK_JPEG_B64 = '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';
function camHeader(frameId) {
  const nowSec = Date.now() / 1000;
  return {
    stamp: { sec: Math.floor(nowSec), nanosec: Math.floor((nowSec % 1) * 1e9) },
    frame_id: frameId,
  };
}
function buildColorFrame() {
  return { header: camHeader('camera_camera_color_optical'), format: 'jpeg', data: MOCK_JPEG_B64 };
}
/** 动态 mono8 灰度图:流动干涉条纹(两通道相位不同,便于区分左右) */
function buildMonoFrame(channel, phase) {
  const t = (Date.now() - startTimeMs) / 1000;
  const buf = Buffer.allocUnsafe(CAM_W * CAM_H);
  for (let y = 0; y < CAM_H; y++) {
    const rowV = 30 * Math.cos(y * 0.12 - t * 2);
    for (let x = 0; x < CAM_W; x++) {
      let v = 128 + 80 * Math.sin(x * 0.15 + t * 4 + phase) + rowV;
      if (v < 0) v = 0; else if (v > 255) v = 255;
      buf[y * CAM_W + x] = v;
    }
  }
  return {
    header: camHeader(`camera_camera_${channel}_optical`),
    height: CAM_H,
    width: CAM_W,
    encoding: 'mono8',
    is_bigendian: 0,
    step: CAM_W,
    data: buf.toString('base64'),
  };
}
function buildCamImuMessage() {
  const t = (Date.now() - startTimeMs) / 1000;
  const q = eulerToQuatZYX(
    THREE_DegToRad(1.2) * Math.sin(2 * Math.PI * 0.2 * t),
    THREE_DegToRad(1.6) * Math.cos(2 * Math.PI * 0.17 * t),
    THREE_DegToRad(10) * Math.sin(2 * Math.PI * 0.05 * t),
  );
  return {
    header: camHeader('camera_camera_imu_optical'),
    orientation: q,
    orientation_covariance: new Array(9).fill(0),
    angular_velocity: {
      x: THREE_DegToRad(1.2) * 2 * Math.PI * 0.2 * Math.cos(2 * Math.PI * 0.2 * t),
      y: -THREE_DegToRad(1.6) * 2 * Math.PI * 0.17 * Math.sin(2 * Math.PI * 0.17 * t),
      z: 0,
    },
    angular_velocity_covariance: new Array(9).fill(0),
    linear_acceleration: {
      x: Math.sin(THREE_DegToRad(1.6) * Math.cos(2 * Math.PI * 0.17 * t)) * 9.81,
      y: 0.3 * Math.sin(2 * Math.PI * 1.1 * t),
      z: 9.75 + (Math.random() - 0.5) * 0.08,
    },
    linear_acceleration_covariance: new Array(9).fill(0),
  };
}
function buildVioMessage() {
  const t = (Date.now() - startTimeMs) / 1000;
  const yaw = 0.08 * Math.sin(2 * Math.PI * 0.05 * t);
  const q = eulerToQuatZYX(0, 0, yaw);
  // 真机实测:vio_100hz 是 geometry_msgs/msg/PoseStamped(不是 Odometry)
  return {
    header: camHeader('camera_camera_imu_optical'),
    pose: {
      position: {
        x: 0.25 * Math.sin(2 * Math.PI * 0.04 * t),
        y: 0.18 * Math.cos(2 * Math.PI * 0.033 * t),
        z: 1.05 + 0.02 * Math.sin(2 * Math.PI * 0.1 * t),
      },
      orientation: q,
    },
  };
}
const VIO_STATES = ['VIO OK · tracking', 'VIO OK · re-localizing', 'VIO OK · tracking'];
// 与真机固件行为一致:只推 image_rect_raw 系列;image_raw 系列话题在 rosapi 有名字但无流
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/camera/camera/color/image_rect_raw/compressed', buildColorFrame());
}, 66);
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/camera/camera/infra1/image_rect_raw', buildMonoFrame('infra1', 0));
}, 100);
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/camera/camera/infra2/image_rect_raw', buildMonoFrame('infra2', Math.PI));
}, 100);
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/camera/camera/imu', buildCamImuMessage());
}, 50);
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/camera/camera/vio_100hz', buildVioMessage());
}, 100);
setInterval(() => {
  if (clients.size === 0) return;
  const features = 120 + Math.round(40 * Math.sin((Date.now() - startTimeMs) / 900));
  const state = VIO_STATES[Math.floor((Date.now() - startTimeMs) / 4000) % VIO_STATES.length];
  bus.emit('publish', '/camera/camera/vio_status', {
    data: `${state} | features=${features} | fps=98 | drift=0.012m`,
  });
}, 1000);

// ─────────────────────────────────────────────────────────
//  模拟 Livox MID-360:3D 点云 50Hz + 内置 BMI088 IMU 200Hz
//  非重复花瓣扫描:水平 360°、垂直 -7°~+52°,每帧 3936~4032 点
//
//  场景是「静止的封闭房间」:雷达原点在房间中央、装高 0.8m。
//  非重复扫描每帧只打到场景的稀疏子集,前端把多帧持续重叠累加
//  (5cm 体素去重)才能逐步还原地面/天花/墙面/箱体的完整结构。
// ─────────────────────────────────────────────────────────
const LIDAR_EL_MIN = THREE_DegToRad(-7);
const LIDAR_EL_MAX = THREE_DegToRad(52);
const LIDAR_POINT_STEP = 16; // x,y,z,intensity 各 float32
// 房间:半宽 8m × 半深 6m,地面 -0.8m,天花 2.6m
const ROOM = { hx: 8, hy: 6, zFloor: -0.8, zCeil: 2.6 };
// 房内物体(轴对齐包围盒 [min,max] + 反射率基数)
const LIDAR_OBJECTS = [
  { min: [ 1.8, -2.8, -0.8], max: [ 3.0, -1.6,  0.4], base: 195 }, // 右前货箱
  { min: [-3.4,  1.6, -0.8], max: [-2.2,  2.8,  1.4], base: 170 }, // 左后高柜
  { min: [-0.8,  3.2, -0.8], max: [ 0.0,  4.0,  2.3], base: 150 }, // 立柱
  { min: [ 4.6,  0.6, -0.8], max: [ 6.2,  2.2, -0.35], base: 120 }, // 矮平台
  { min: [ 0.9,  1.0, -0.8], max: [ 1.7,  1.8, -0.5], base: 205 }, // 近场小箱
];
// 各表面反射率基数
const LIDAR_SURF_BASE = { floor: 90, ceiling: 70, wall: 150 };

/** 确定性 PRNG(每帧不同种子):模拟非重复扫描,帧间射线分布不重复 */
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
let lidarFrameNo = 0;

/** 射线(原点 0,方向 d)与轴对齐盒求交,返回首个进入距离 t,未命中返回 Infinity */
function rayAABB(dx, dy, dz, mn, mx) {
  let tmin = 0.02, tmax = Infinity;
  const o = [0, 0, 0], d = [dx, dy, dz], bmin = mn, bmax = mx;
  for (let k = 0; k < 3; k++) {
    if (Math.abs(d[k]) < 1e-9) {
      if (o[k] < bmin[k] || o[k] > bmax[k]) return Infinity;
    } else {
      let t1 = (bmin[k] - o[k]) / d[k];
      let t2 = (bmax[k] - o[k]) / d[k];
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      if (t1 > tmin) tmin = t1;
      if (t2 < tmax) tmax = t2;
      if (tmax < tmin) return Infinity;
    }
  }
  return tmin;
}

/** 对一条扫描射线求最近命中:{t, kind, base} */
function castScene(az, el) {
  const dx = Math.cos(el) * Math.cos(az);
  const dy = Math.cos(el) * Math.sin(az);
  const dz = Math.sin(el);
  let best = Infinity, kind = null, base = 0;
  const consider = (t, k, b) => { if (t < best) { best = t; kind = k; base = b; } };

  // 地面/天花(水平面)
  if (dz < -1e-9) consider(ROOM.zFloor / dz, 'floor', LIDAR_SURF_BASE.floor);
  if (dz > 1e-9) consider(ROOM.zCeil / dz, 'ceiling', LIDAR_SURF_BASE.ceiling);
  // 四面竖直墙:求交后校验落点在墙的平面范围内(z 在地面~天花之间,
  // 另一轴不超出相邻墙)
  const wallHits = [
    { t:  ROOM.hx / dx, v: dy * ( ROOM.hx / dx), z:  ROOM.hx * dz / dx, lim: ROOM.hy }, // x=+8
    { t: -ROOM.hx / dx, v: dy * (-ROOM.hx / dx), z: -ROOM.hx * dz / dx, lim: ROOM.hy }, // x=-8
    { t:  ROOM.hy / dy, v: dx * ( ROOM.hy / dy), z:  ROOM.hy * dz / dy, lim: ROOM.hx }, // y=+6
    { t: -ROOM.hy / dy, v: dx * (-ROOM.hy / dy), z: -ROOM.hy * dz / dy, lim: ROOM.hx }, // y=-6
  ];
  for (const w of wallHits) {
    if (w.t > 0 && Math.abs(w.v) <= w.lim && w.z >= ROOM.zFloor && w.z <= ROOM.zCeil) {
      consider(w.t, 'wall', LIDAR_SURF_BASE.wall);
    }
  }
  // 房内物体
  for (const obj of LIDAR_OBJECTS) {
    const t = rayAABB(dx, dy, dz, obj.min, obj.max);
    if (isFinite(t)) consider(t, 'object', obj.base);
  }
  return isFinite(best) ? { t: best, kind, base } : null;
}

function buildLidarFrame() {
  const t = (Date.now() - startTimeMs) / 1000;
  const rand = mulberry32((0x9E3779B9 ^ Math.imul(lidarFrameNo++ + 1, 2654435761)) >>> 0);
  const width = 3936 + Math.floor(rand() * 97); // 3936~4032
  const buf = Buffer.allocUnsafe(width * LIDAR_POINT_STEP);
  let emitted = 0;
  while (emitted < width) {
    const az = rand() * Math.PI * 2;
    // 均匀垂直覆盖 + 轻微花瓣调制;非重复扫描靠帧间种子不同自然累加
    let el = LIDAR_EL_MIN + rand() * (LIDAR_EL_MAX - LIDAR_EL_MIN)
      + 0.06 * Math.sin(2 * az + t * 1.3) + 0.04 * Math.sin(5 * az - t * 0.8);
    if (el < LIDAR_EL_MIN) el = LIDAR_EL_MIN;
    else if (el > LIDAR_EL_MAX) el = LIDAR_EL_MAX;
    const hit = castScene(az, el);
    if (!hit) continue; // 封闭房间理论上每射线都命中
    const ce = Math.cos(el);
    const dx = ce * Math.cos(az), dy = ce * Math.sin(az), dz = Math.sin(el);
    // 测量噪声 ±1.2cm
    const nx = (rand() + rand() + rand() - 1.5) * 0.016;
    const ny = (rand() + rand() + rand() - 1.5) * 0.016;
    const nz = (rand() + rand() + rand() - 1.5) * 0.016;
    const x = dx * hit.t + nx;
    const y = dy * hit.t + ny;
    const z = dz * hit.t + nz;
    // 反射强度:材质基数 × 距离衰减 + 噪声
    let intensity = hit.base / (1 + 0.05 * hit.t) + (rand() - 0.5) * 22;
    if (intensity < 8) intensity = 8; else if (intensity > 255) intensity = 255;
    const o = emitted * LIDAR_POINT_STEP;
    buf.writeFloatLE(x, o);
    buf.writeFloatLE(y, o + 4);
    buf.writeFloatLE(z, o + 8);
    buf.writeFloatLE(intensity, o + 12);
    emitted++;
  }
  return {
    header: camHeader('livox_frame'),
    height: 1,
    width,
    fields: [
      { name: 'x', offset: 0, datatype: 7, count: 1 },
      { name: 'y', offset: 4, datatype: 7, count: 1 },
      { name: 'z', offset: 8, datatype: 7, count: 1 },
      { name: 'intensity', offset: 12, datatype: 7, count: 1 },
    ],
    is_bigendian: 0,
    point_step: LIDAR_POINT_STEP,
    row_step: width * LIDAR_POINT_STEP,
    data: buf.toString('base64'),
    is_dense: 1,
  };
}
/** 雷达内置 BMI088:orientation 不发(协方差首元素 -1),姿态由前端按重力估计 */
function buildLidarImuMessage() {
  const t = (Date.now() - startTimeMs) / 1000;
  // 缓慢小角度摇摆(±2°/±1.5°)叠加电机类高频微振动
  const roll = THREE_DegToRad(2) * Math.sin(2 * Math.PI * 0.25 * t);
  const pitch = THREE_DegToRad(1.5) * Math.cos(2 * Math.PI * 0.21 * t);
  const vib = 0.25; // m/s² 振动量级
  return {
    header: camHeader('livox_frame'),
    orientation: { x: 0, y: 0, z: 0, w: 0 },
    orientation_covariance: [-1, 0, 0, 0, 0, 0, 0, 0, 0],
    angular_velocity: {
      x: THREE_DegToRad(2) * 2 * Math.PI * 0.25 * Math.cos(2 * Math.PI * 0.25 * t) + (Math.random() - 0.5) * 0.02,
      y: -THREE_DegToRad(1.5) * 2 * Math.PI * 0.21 * Math.sin(2 * Math.PI * 0.21 * t) + (Math.random() - 0.5) * 0.02,
      z: (Math.random() - 0.5) * 0.015,
    },
    angular_velocity_covariance: new Array(9).fill(0),
    linear_acceleration: {
      x: -9.81 * Math.sin(pitch) + (Math.random() - 0.5) * vib,
      y: 9.81 * Math.sin(roll) + (Math.random() - 0.5) * vib,
      z: 9.81 * Math.cos(roll) * Math.cos(pitch) + (Math.random() - 0.5) * vib,
    },
    linear_acceleration_covariance: new Array(9).fill(0),
  };
}
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/livox/lidar', buildLidarFrame());
}, 20); // 50Hz
setInterval(() => {
  if (clients.size === 0) return;
  bus.emit('publish', '/livox/imu', buildLidarImuMessage());
}, 5); // 200Hz

// 每 3s 打印一次推送量(便于观察活跃度)
let lastPrint = { motor: motorFrame, imu: imuFrame, t: Date.now() };
setInterval(() => {
  const now = Date.now();
  const dt = (now - lastPrint.t) / 1000;
  const motorRate = ((motorFrame - lastPrint.motor) / dt).toFixed(1);
  const imuRate   = ((imuFrame   - lastPrint.imu)   / dt).toFixed(1);
  console.log(
    `[mock-ros] ⏱  最近3s: clients=${clients.size},  motor=${motorRate}Hz (帧=${motorFrame}),  imu=${imuRate}Hz (帧=${imuFrame})`
  );
  lastPrint = { motor: motorFrame, imu: imuFrame, t: now };
}, 3000);

// ─────────────────────────────────────────────────────────
//  通知事件测试 (/rl_real/notify):模拟机器人向客户端推送通知
// ─────────────────────────────────────────────────────────
let _notifyId = 0;
function sendNotify(type, content) {
  _notifyId++;
  const msg = {
    op: 'publish',
    topic: '/rl_real/notify',
    msg: { data: JSON.stringify({ id: _notifyId, type, content, timestamp: Date.now() / 1000 }) },
  };
  for (const peer of clients) {
    try { peer.sendJSON(msg); } catch { /* 忽略 */ }
  }
  console.log(`[mock-ros] 📨 notify #${_notifyId} ${type}: ${content}`);
}

// 收到 command 时模拟机器人回复通知(与 C++ 端 FSM Enter() 发送的实际文案一致)
bus.on('publish', (topic, payload) => {
  if (topic === '/rl_real/command') {
    const cmd = String(payload?.data || '');
    setTimeout(() => {
      if (cmd === 'zero') {
        sendNotify('info', 'motors re-zeroed, ready for stand');
      } else if (cmd === 'getup' || cmd === '0') {
        sendNotify('info', 'Entered getup (standing up)');
      } else if (cmd === 'locomotion' || cmd === '1') {
        sendNotify('info', 'Entered RL locomotion');
      } else if (cmd === 'getdown' || cmd === '2') {
        sendNotify('info', 'Entered getdown (lying down)');
      } else if (cmd === 'passive' || cmd === '3') {
        sendNotify('info', 'Entered passive mode');
      }
    }, 300);
  } else if (topic === '/rl_real/check_stand') {
    // 站立检查:模拟返回"就绪"结果(电机零位达标 + IMU 正常)
    const result = { ready: true, motors_ready: true, imu_ready: true, reason: '' };
    const msg = {
      op: 'publish',
      topic: '/rl_real/check_stand_result',
      msg: { data: JSON.stringify(result) },
    };
    for (const peer of clients) {
      try { peer.sendJSON(msg); } catch { /* 忽略 */ }
    }
    console.log('[mock-ros] 🔍 check_stand result: ready=true');
  } else if (topic === '/rl_briefing/play') {
    // 作业交底播报:模拟机器人回复播报状态(与 C++ 端 briefing_audio_player 行为一致)
    const scene = String(payload?.data || '');
    if (scene === 'stop') {
      console.log('[mock-ros] ⏹ briefing stop');
    } else if (scene === 'list') {
      console.log('[mock-ros] 📋 briefing list: elevator/forklift/lifting/warehouse/height');
    } else {
      console.log(`[mock-ros] 🔊 briefing play: ${scene}`);
      setTimeout(() => {
        const msg = {
          op: 'publish',
          topic: '/rl_briefing/status',
          msg: { data: `正在播报作业交底:${scene}` },
        };
        for (const peer of clients) {
          try { peer.sendJSON(msg); } catch { /* 忽略 */ }
        }
      }, 300);
    }
  }
});

// ─────────────────────────────────────────────────────────
//  启动
// ─────────────────────────────────────────────────────────
server.on('error', (err) => {
  if (err && err.code === 'EADDRINUSE') {
    console.error(`[mock-ros] ❌ 端口 ${PORT} 已被占用!换一个端口启动: node js/_mock-rosbridge.js 9091`);
    console.error('[mock-ros]    然后在前端 ROS 地址填 ws://127.0.0.1:9091');
    console.error(`[mock-ros]    (占用者 PID 可用 netstat -ano | findstr :${PORT} 查询)`);
    process.exit(1);
  }
  throw err;
});

server.listen(PORT, HOST, () => {
  console.log('');
  console.log('╔══════════════════════════════════════════════════════════════════╗');
  console.log('║       Mock rosbridge_server (本地 ROS2 接口模拟器)                ║');
  console.log('╠══════════════════════════════════════════════════════════════════╣');
  console.log(`║  监听: ${HOST}:${PORT} (所有网卡)                                    ║`);
  console.log(`║  本机回环:   ws://127.0.0.1:${PORT}                                ║`);
  // 列出局域网 IPv4,手机/真机直接用这些地址连
  const lanIps = [];
  for (const ifaces of Object.values(os.networkInterfaces())) {
    for (const ni of ifaces || []) {
      if (ni.family === 'IPv4' && !ni.internal) lanIps.push(ni.address);
    }
  }
  for (const ip of lanIps) {
    console.log(`║  局域网连接: ws://${ip}:${PORT}${' '.repeat(Math.max(0, 36 - ip.length))}║`);
  }
  console.log(`║  HTTP 健康页: http://127.0.0.1:${PORT}/                                ║`);
  console.log('║                                                                  ║');
  console.log(`║  📡 订阅: /rl_real/motor_state  @ ${String(MOTOR_HZ).padEnd(3)}Hz   (${JOINT_COUNT}×10 floats)    ║`);
  console.log(`║  📡 订阅: /rl_real/imu_state    @ ${String(IMU_HZ).padEnd(3)}Hz   (19 floats)           ║`);
  console.log('║  📢 发布: /rl_real/command  (String)  → 本控制台日志              ║');
  console.log('║  📢 发布: /rl_real/cmd_vel  (Twist)   → 本控制台日志              ║');
  console.log('║                                                                  ║');
  console.log('║  使用方法:                                                        ║');
  console.log('║  方案A (推荐):临时改 js/ros-bridge.js 的 ROS_URL 为本机地址:      ║');
  console.log('║    export const ROS_URL = "ws://127.0.0.1:9090";                  ║');
  console.log('║  方案B: 修改本机 hosts: 192.168.5.101 → 127.0.0.1                 ║');
  console.log('╚══════════════════════════════════════════════════════════════════╝');
  console.log('');
});
