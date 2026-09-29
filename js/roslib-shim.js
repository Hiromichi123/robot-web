/**
 * roslib UMD → ESM shim。
 * roslib 1.4.1 官方发行包是 UMD 格式,挂到 window.ROSLIB 全局。
 * 本模块在 index.html 中先通过 <script src> 加载 roslib.min.js 后,
 * 再从全局对象导出 ROSLIB 命名空间,供 ros-bridge.js import 使用。
 */
const g = (typeof window !== 'undefined' ? window : globalThis);
if (!g.ROSLIB) {
  throw new Error('ROSLIB 全局对象不存在,请确认 index.html 已先 <script src> 加载 roslib.min.js');
}
export const ROSLIB = g.ROSLIB;
export default g.ROSLIB;
