# 命令生命周期状态码 + 2Hz 心跳 —— 协议说明

> 面向：**web 控制台开发**
> 生效版本：2026-09-27 起小脑 `rl_real_JXG`（`~/rl_sar-w2_recovered_v1`）
> 本文所有报文样例都是**实测抓到的**，不是设计稿。

---

## 1. 为什么加这个

原来发一条命令，控制器只回一句 `command accepted: getup`，**之后执行到没到、成没成，上报不了**。
页面只能盲等或者靠固定时间猜。

现在控制器会为每条命令发一串状态码：

```
已下发(RECEIVED) → 执行中(EXECUTING) → 已完成(DONE) / 失败(FAILED) / 被拒绝(REJECTED)
```

外加每 0.5 秒一次心跳，页面可以据此判断链路还活着没有。

---

## 2. 传输层

- **WebSocket**，地址 `ws://<板子IP>:8088/ws`
- **单行 JSON 文本帧**（不是二进制）
- 页面现有代码里 `ros-bridge.js` 的 `ROS_URL_DEFAULT` 已经用的是 `location.host`，**不用改地址**

页面已有的其他通道（`motor_state` / `imu_state` / `pose2d` / `feedback` / `notify` …）
**一律不变**，本次只是新增两个下行通道 + 上行多一个可选字段。

---

## 3. 上行：发命令时带上 `seq`

```json
{"type":"command","data":"getup","seq":7}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | 固定 `"command"` |
| `data` | string | 命令名，同原来的用法（`getup` / `getdown` / `locomotion` / `vel_stop` / `zero` / `status` …；`passive` 已隐藏，见 §6） |
| `seq` | int，**可选** | 发送方自己分配的序号。**不传或传 0 = 不做关联**，控制器照常执行，只是回的状态码里 seq 为 0 |

### 关键规则：一次用户动作 = 一个 seq

现有页面发行为命令是**3 帧连发**（间隔 500 ms，对齐量产做法）。**这三帧必须共用同一个 seq**，
不要每帧递增。

理由：控制器把同 `(seq, cmd)` 的帧当作同一条命令的重传，会去重。
如果每帧递增，控制器会认为来了 3 条不同命令，**生命周期被重置 3 次**，
`elapsed_ms` 与超时计时都跟着乱，页面收到 3 组状态码。

> 去重键是 **(seq, cmd) 组合**，不是单独 seq。因为不同发送端（页面、上位机程序）
> 各自从 1 开始编号，只比 seq 会撞车。

---

## 4. 下行：命令生命周期 `cmd_state`

```json
{"type":"cmd_state","data":{"seq":7,"cmd":"getup","code":2,"phase":"DONE",
                            "target":"RLFSMStateGetUp","state":"RLFSMStateGetUp",
                            "elapsed_ms":3120}}
```

**注意 `data` 是 JSON 对象**（不是字符串），可以直接 `frame.data.code` 取用。

| 字段 | 类型 | 说明 |
|---|---|---|
| `seq` | int | 回带发送方的序号。**0 表示这条不是页面发的**（见下方"必须过滤 seq=0"） |
| `cmd` | string | 命令名（已归一化，如 `stand` 会归一成 `getup`） |
| `code` | int | 状态码，见下表 |
| `phase` | string | 状态码的文字形式，给人看/写日志用 |
| `target` | string \| null | 期望到达的 FSM 状态；`null` 表示该命令不改变 FSM 状态 |
| `state` | string | 报文生成时的实际 FSM 状态 |
| `elapsed_ms` | int | 从命令被接收到现在的毫秒数 |

### 状态码表

| code | phase | 含义 | 是否终态 |
|---|---|---|---|
| 0 | `RECEIVED` | 已接收（命令已解析并已请求状态切换） | 否 |
| 1 | `EXECUTING` | 执行中（还没到目标状态） | 否 |
| 2 | `DONE` | 已完成（当前 FSM 状态 == 目标状态） | **是** |
| 3 | `FAILED` | 超时仍未到达目标状态 | **是** |
| 4 | `REJECTED` | 命令无法识别 | **是** |

**终态判断：`code >= 2`。**

### 一条命令会收到几条 `cmd_state`

| 情况 | 序列 |
|---|---|
| 会改变 FSM 状态的命令（getup/locomotion/…），且当前不在目标态 | `0` → `1` → `2`（或 `3`） |
| 会改变状态，但**当前已经在目标态** | `0` → `2`（第一拍就发现已到达，不经过 `1`） |
| 不改变状态的命令（`nav_on`/`vel_stop`/`status`/`zero`…） | `0` → `2`，`target` 为 `null` |
| 无法识别的命令 | `4`（只有这一条） |

> `1 EXECUTING` **只在真正发生状态迁移时才会出现**。所以只发 `nav_on` 是观察不到 code=1 的，
> 这是设计使然，不是 bug。

---

## 5. 下行：心跳 `heartbeat`（2 Hz）

```json
{"type":"heartbeat","data":{"seq":1178,"state":"RLFSMStatePassive","nav":true,
                            "uptime_ms":594053,"active_seq":0}}
```

**实测频率 1.994 Hz**（周期 min 0.499s / max 0.505s，标准差 0.0026s）。

| 字段 | 类型 | 说明 |
|---|---|---|
| `seq` | int | 心跳自己的递增计数（**与命令的 seq 不是一回事**），可用来检测丢包 |
| `state` | string | 当前 FSM 状态 |
| `nav` | bool | `navigation_mode` 是否打开（决定 `cmd_vel` 是否受失联看门狗保护） |
| `uptime_ms` | int | 控制器启动至今毫秒数 |
| `active_seq` | int | 当前正在执行的命令 seq；无则为 0 |

**心跳是判断"链路是否还活着"最直接的信号**：超过 ~1.5 s 没收到就说明链路有问题。

---

## 6. 目标状态与超时

| 命令（含别名） | `target` | 超时 |
|---|---|---|
| `getup` / `stand` / `up` | `RLFSMStateGetUp` | 6000 ms |
| `getdown` / `down` / `lie` / `sit` | `RLFSMStateGetDown` **或** `RLFSMStatePassive` | 6000 ms |
| `locomotion` / `rl` / `walk` / `move` | `RLFSMStateRLLocomotion` | 5000 ms |
| 其余（`nav_on` / `nav_off` / `vel_stop` / `hold` / `zero` / `status` / `imu_on` / `imu_off` / `zero_motor N` / `exit`） | `null`（收到即完成） | — |

> **`passive` / `stop` / `safe_stop` 已从对大脑的 ROS 接口隐藏（2026-10-01）**：趴下一律
> 用 `getdown`（受控插值）。ROS 侧（含 /rl_real/command 与网页）发这些会被小脑直接
> 拒绝：feedback 回 `command rejected: passive hidden on ROS (use getdown)`，
> cmd_state 立即 `REJECTED`。内置 Web 调试台（WebSocket）与手柄 P 键仍保留 passive。

**`getdown` 为什么有两个目标**：`RLFSMStateGetDown` 是**过渡态**——趴下动画进度到 100% 时
会自动转到 `RLFSMStatePassive`。如果只等 `GetDown`，动画跑完那一瞬间就错过了，会误报超时。
所以两个都算"到达"。报文里的 `target` 字段只会显示第一个（`RLFSMStateGetDown`），
判断到达要用状态码而不是自己比对 `target == state`。

### FSM 状态全集（JXGw，即当前机型）

```
RLFSMStatePassive       卸力趴着（初始态）
RLFSMStateGetUp         已站起（稳定态，不会自己跳走）
RLFSMStateGetDown       正在趴下（过渡态，会自动转 Passive）
RLFSMStateRLLocomotion  RL 运动模式（稳定态）
```

> 另一机型 JXG 还多两个状态（`RLFSMStateRLDreamwaq`、`RLFSMStateRLNp3o`），本机用不到。

---

## 7. 页面必须做的过滤：**忽略 `seq == 0`**

除了页面之外，**大脑侧的桥节点也在每 2 Hz 轮询 `status`**（它发的是 `seq:0`）。
控制器会为这些轮询也发 `cmd_state`，也就是说：

> 页面会收到**大量 seq=0 的 `cmd_state`**（实测约 4 条/秒），它们跟页面的操作**毫无关系**。

所以页面的监听里必须 **`if (!st.seq) return;`**，只处理自己发出去的那个 seq。

另外建议再按 `seq:code` 去重，避免连发帧导致同一相位重复弹提示。

---

## 8. 实测样例（可直接对照调试）

页面点一次 `nav_on`（页面分配 seq=1，3 帧连发）：

```json
{"type":"cmd_state","data":{"seq":1,"cmd":"nav_on","code":0,"phase":"RECEIVED","target":null,"state":"RLFSMStatePassive","elapsed_ms":0}}
{"type":"cmd_state","data":{"seq":1,"cmd":"nav_on","code":2,"phase":"DONE","target":null,"state":"RLFSMStatePassive","elapsed_ms":0}}
```

发一条不认识命令：

```json
{"type":"cmd_state","data":{"seq":3,"cmd":"bogus_xyz","code":4,"phase":"REJECTED","target":null,"state":"RLFSMStatePassive","elapsed_ms":0}}
```

心跳：

```json
{"type":"heartbeat","data":{"seq":172,"state":"RLFSMStatePassive","nav":true,"uptime_ms":91052,"active_seq":0}}
```

桥的 status 轮询（**页面要过滤掉的**）：

```json
{"type":"cmd_state","data":{"seq":0,"cmd":"status","code":0,"phase":"RECEIVED","target":null,"state":"RLFSMStatePassive","elapsed_ms":0}}
{"type":"cmd_state","data":{"seq":0,"cmd":"status","code":2,"phase":"DONE","target":null,"state":"RLFSMStatePassive","elapsed_ms":0}}
```

---

## 9. 参考：这些在 ROS 侧对应什么

页面直接连的是 WebSocket，用不到这段；列在这里是为了和大脑侧对得上。

| WebSocket 通道 | ROS 话题（大脑 `192.168.8.137`） | 类型 |
|---|---|---|
| `cmd_state` | `/rl_real/cmd_state` | `std_msgs/String`（内容是 JSON） |
| `heartbeat` | `/rl_real/heartbeat` | `std_msgs/String`（内容是 JSON） |
| — | `/rl_real/heartbeat_alive` | `std_msgs/Bool`（桥维护的心跳存活标志） |
| `command` | `/rl_real/command` | `std_msgs/String` |
| `feedback` | `/rl_real/feedback` | `std_msgs/String` |

---

## 10. 已知限制 / 坑

1. **`EXECUTING`(1) 需要真实状态迁移才会出现** —— 只测 `nav_on` 看不到，别以为是坏了。
2. **`FAILED`(3) 目前没在真机验证过** —— 要触发它得让某条命令真的超时（比如狗被卡住站不起来），
   运动测试时才有机会看到。
3. **`seq` 不去重也能用** —— 控制器侧的 `(seq, cmd)` 去重已实现并编译，但**需要重启控制器才生效**。
   生效前连发 3 帧会收到 3 组状态码；页面按 `seq:code` 去重即可正常显示。
4. **心跳计数器 `seq` 是心跳自己的**，和命令的 seq 无关，别混用。
5. **不要自己拿 `target == state` 判断完成** —— `getdown` 会因为过渡态而误判；用 `code == 2`。
6. 页面改动后请**硬刷新**（Ctrl+F5），JS 文件是强缓存的。
