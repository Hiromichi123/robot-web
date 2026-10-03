# robot-web —— JXG 轮足狗的地面站前端

纯静态单页应用（Three.js + roslib，无构建步骤）。跑在**算法大脑**（NanoPi M5，192.168.8.137）上，
浏览器访问 `http://192.168.8.137:8080/`，页面里的 ROS 地址填 `ws://192.168.8.137:9090`。

rosbridge 那一路由狗端仓库的 `core_2026/launch/slam_only.launch.py` 提供（雷达 + Point-LIO +
小脑通信桥 `dog_ros2_bridge` + rosbridge），**它和 `dog_node` 都要另外起**。

## 怎么跑

板上没有 node，静态文件由本仓库根下的 `webserver.py` 在 8080 端口托管（入口仍是 `~/webserver.sh`，2026-10-03 收进仓库）——
它同时还提供「地图管理」页签要用的 `/api/maps` 接口（列目录/读取/删除 `~/lidar_maps` 下的 pcd）。
浏览器碰不到文件系统，所以那个后端不能省。

    ~/webserver.sh                 # 前台跑，监听 8080

## 页签

话题调试 / 报警配置 / 录制列表 / 移动控制 / 关节控制 / 笛卡尔控制 / 可视化 / 数据看板 /
相机 / 雷达 / 地图管理 / **任务状态** / **任务管理**

最后两个是任务编排系统（2026-09-29 加）：

- **任务管理**：拼装与存储任务。列表里点一行选中（行内「修改」「删除」，底部「+ 新增任务」），
  「加载到执行页」把选中的任务送去执行；下面是 JSON 预览（复制/导出/导入）。
  点「新增」或「修改」进**子页面**编排步骤（7 种步骤，参数就地改，↑↓ 排序/复制/删除/套模板）。
- **任务状态**：下发执行 + 看情况。大按钮「下发执行 / 停止（急停）」，狗端停在危险步骤前等确认时
  出现大按钮「继续 / 放弃」，下面是相位 + 第 n/N 步 + 用时的大状态栏。

链路：页面 → `/dog/mission`（整份 JSON）→ 狗端 `dog_node` 解析执行 →
`/dog/mission_status`（变化时 + 2Hz 心跳）回进度 → 页面的「继续/放弃/急停」走 `/dog/mission_cmd`。
**协议以狗端仓库的 `MISSION_PROTOCOL.md` 为准**，改一边必须同步另一边。
命令生命周期状态码 + 2Hz 心跳（`cmd_state` / `heartbeat`，console 页在用）见本仓库 `docs/CMD_STATE_PROTOCOL.md`。

## ⚠️ 改文件必读：`?v=` 缓存约定

所有跨文件 import 都带版本号（`./ros-bridge.js?v=980`）。**同一个文件被多处 import 时版本号必须一致**，
漏一处浏览器会加载两份模块实例 —— 表现为"订阅了却收不到数据"，而且**不报错**，极难查。

- 改 `js/ros-bridge.js`：要同步 bump 全部 9 处 import 点。
- 改 `js/main.js`：要 bump `index.html` 和 `mobile.html` 里的 `main.js?v=`。
- `index.html` 与 `mobile.html` 是**同一份应用的两套页面**：`main.js` 的 `setControlTab()` 里对面板
  元素的 `hidden` 赋值在 `try/catch` 之外 —— 少一个面板就是 TypeError，会打断整个页签切换。
  所以**改面板必须两个文件同步改**。

## 目录

- `index.html` / `mobile.html` —— 桌面与手机两套页面（手机打开 index 会自动跳到 mobile）
- `js/` —— 页面逻辑。`ros-bridge.js` 是**唯一的 ROS 出口**（话题常量、发布函数、订阅与监听都收口在这里），
  其它模块只从它取连接或调用它的函数
- `css/`、`vendor/`（three.js、roslib，第三方，一般不碰）
- `model/2/` —— 机器狗模型的网格（46MB，**必须留着**，否则 3D 视图里没有狗）
- `webserver.py` —— 板上托管的服务器（静态 + `/api/maps`），由 `~/webserver.sh` 起；静态根=脚本所在目录
- `docs/` —— 协议文档（`CMD_STATE_PROTOCOL.md`）
- `js/_mock-rosbridge.js`、`js/_dev-server.js` —— 本地调试用的模拟 rosbridge（不参与正式功能）
