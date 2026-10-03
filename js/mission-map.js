/**
 * 任务编辑器里的**点云地图视图**：用来在已建好的地图上**点选航点**。
 *
 * 跟「地图管理」页签那个 3D 视图是**两个独立实例**（各自 canvas / 相机 / 状态），
 * 互不干扰 —— 那边是"看某张旧图"，这边是"在图上取坐标、加进任务"。
 * 分开的另一个理由：那边的订阅跟建图开关绑在一起，塞进编辑器要抢状态。
 *
 * 为什么点云上的点在**狗端的坐标**里可直接当航点：
 *   pcd 是世界系（Point-LIO 的 camera_init），录制时用 `/aft_mapped_to_init` 变换过来的；
 *   而狗端 goto 用的 `/lidar_data` 位姿也在同一个世界系。所以图上点一下拿到的 (x, y)
 *   就是可以直接填进 `goto` 的坐标。**前提是那份图确实是这台狗、这次开机建的**
 *   （换场地/重新上电后世界原点会变，旧图只能当参考）。
 *
 * pcd 格式（照 lidar_recorder 写出来的）：`x y z intensity rgb`，20 字节/点，DATA binary。
 * rgb 以 float 存的是**打包后的位**（PCL 的老规矩），没颜色的点写 NaN。
 * 字段按头里的 FIELDS/SIZE/TYPE/COUNT 泛化解析，不假设固定布局。
 *
 * 显示约定：世界系是 z-up，相机 up=(0,0,1)、参考网格铺在 XY 平面
 * （红=x 绿=y 蓝=z）—— 与雷达页/地图管理页/机器人视图同一套约定。
 * 显示朝向不影响取点：onPick 给的是点云原始世界坐标。
 *
 * 单导入方模块（仅 mission-view.js 引用），可独立 ?v= 版本号。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// pcd 解析搬到 pcd-parse.js 了：那是纯函数、不依赖 three，能在 node 里单测。
// 这里 re-export 一下，调用方的 import 不用改。
export { parsePcd } from './pcd-parse.js?v=1';

/** 高度着色（矮蓝高红），跟「地图管理」那个视图同一套观感 */
function heightColor(z, lo, hi, out, i) {
    const t = hi > lo ? Math.min(1, Math.max(0, (z - lo) / (hi - lo))) : 0.5;
    out[i]     = 0.25 + 0.75 * t;
    out[i + 1] = 0.45 + 0.35 * (1 - Math.abs(t - 0.5) * 2);
    out[i + 2] = 0.95 - 0.85 * t;
}

/**
 * 建一个地图视图。
 * @param {HTMLCanvasElement} canvas
 * @param {(x: number, y: number, z: number) => void} onPick  在世界系点云上单击
 * @param {(text: string) => void} [onHint]  给界面写一句提示
 */
export function createMissionMap({ canvas, onPick, onHint }) {
    const hint = (t) => { if (onHint) onHint(t); };

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060b12);
    const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 5000);
    camera.position.set(4, -4, 4);
    camera.up.set(0, 0, 1); // z-up(世界系=ROS 坐标,同雷达页/地图管理页/机器人视图)
    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.dampingFactor = 0.12;
    // 取航点时用左键单击，所以旋转放到左键拖动之外 —— 与地图管理那套相反无妨，
    // 这里宁可"拖动才能转"，也不要"点一下就转"把取点搞乱。
    controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };

    const pointsMat = new THREE.PointsMaterial({
        size: 0.05, sizeAttenuation: true, vertexColors: true,
    });
    let pointsObj = null;                 // THREE.Points
    let pointsGeo = null;
    /** 当前地图的世界包围盒（相机取景与取点阈值都用它换算） */
    let bbox = null;
    const markerGroup = new THREE.Group();
    scene.add(markerGroup);
    let axes = null;
    let grid = null;                      // 参考地面网格（XY 平面,红绿轴所在平面）

    controls.addEventListener('change', render);

    function render() {
        renderer.render(scene, camera);
    }

    function resize() {
        const el = canvas.parentElement;
        if (!el) return;
        const w = Math.max(1, el.clientWidth), h = Math.max(1, el.clientHeight);
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        render();
    }

    /** 清空当前地图（连标记一起） */
    function clear() {
        if (pointsObj) {
            scene.remove(pointsObj);
            pointsGeo?.dispose();
            pointsObj = null;
            pointsGeo = null;
        }
        if (axes) { scene.remove(axes); axes = null; }
        if (grid) {
            scene.remove(grid);
            grid.geometry.dispose();
            grid.material.dispose();
            grid = null;
        }
        markerGroup.clear();
        bbox = null;
        render();
    }

    /**
     * 装一份解析好的点云并取景。
     * @param {{positions: Float32Array, colors: Float32Array|null, count: number}} cloud
     */
    function setCloud(cloud) {
        clear();
        if (!cloud || !cloud.count) { hint('这份地图没有可显示的点'); return; }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
        // 没颜色就按高度着色（相机没扫到的面不能是一片黑）
        const cols = new Float32Array(cloud.count * 3);
        if (cloud.colors) {
            cols.set(cloud.colors);
        } else {
            let lo = Infinity, hi = -Infinity;
            for (let i = 2; i < cloud.positions.length; i += 3) {
                const z = cloud.positions[i];
                if (z < lo) lo = z;
                if (z > hi) hi = z;
            }
            for (let i = 0; i < cloud.count; ++i) {
                heightColor(cloud.positions[i * 3 + 2], lo, hi, cols, i * 3);
            }
        }
        geo.setAttribute('color', new THREE.BufferAttribute(cols, 3));
        geo.computeBoundingBox();
        bbox = geo.boundingBox.clone();

        const size = bbox.getSize(new THREE.Vector3());
        const diag = Math.max(1e-3, size.length());
        pointsMat.size = Math.min(0.2, Math.max(0.01, diag / 320));
        pointsGeo = geo;
        pointsObj = new THREE.Points(geo, pointsMat);
        scene.add(pointsObj);

        // 世界原点与坐标轴：航点坐标是相对这个原点的，给个方向感
        axes = new THREE.AxesHelper(diag * 0.12);
        scene.add(axes);

        const c = bbox.getCenter(new THREE.Vector3());

        // 参考地面网格：铺在 XY 平面（=地图的 xy 地面,红绿轴所在平面）。
        // GridHelper 默认躺在自己的 XZ 平面，绕 X 转 90° 才转过来；
        // 尺寸取"盖住整张图"的整十米，中心跟着地图中心走。
        const gridSize = Math.max(20, Math.ceil(Math.max(size.x, size.y) * 1.5 / 10) * 10);
        grid = new THREE.GridHelper(gridSize, Math.round(gridSize), 0x2a4458, 0x18262f);
        grid.rotation.x = Math.PI / 2;
        grid.position.set(c.x, c.y, 0.002); // 抬一点,避免与贴地点的 z-fighting
        grid.material.transparent = true;
        grid.material.opacity = 0.35;
        grid.material.depthWrite = false;
        scene.add(grid);
        controls.target.copy(c);
        camera.position.set(c.x - diag * 0.45, c.y - diag * 0.55, c.z + diag * 0.6);
        camera.near = diag / 2000;
        camera.far = diag * 20;
        camera.updateProjectionMatrix();
        controls.update();
        hint(`已加载 ${cloud.count.toLocaleString()} 个点 · 单击点云加航点 · 左键拖动旋转`);
        render();
    }

    /**
     * 重画航点标记。入参是 [{x, y, z}]（z 缺省就用 0）。
     * 标记做成小球，球心放在点击到的高度上，好看出它贴在哪面墙上/地上。
     */
    function setWaypoints(list) {
        markerGroup.clear();
        if (!Array.isArray(list) || list.length === 0) { render(); return; }
        const diag = bbox ? Math.max(1e-3, bbox.getSize(new THREE.Vector3()).length())
                          : 1;
        const r = Math.min(0.35, Math.max(0.03, diag / 90));
        const geo = new THREE.SphereGeometry(r, 12, 10);
        const mat = new THREE.MeshBasicMaterial({ color: 0x4ed5c6 });
        for (const w of list) {
            const m = new THREE.Mesh(geo, mat);
            m.position.set(w.x, w.y, Number.isFinite(w.z) ? w.z : 0);
            markerGroup.add(m);
        }
        render();
    }

    // ── 取点：按下与抬起基本没动才算"单击"（否则是在旋转/平移）────────
    let downX = 0, downY = 0, downT = 0;
    canvas.addEventListener('pointerdown', (e) => {
        downX = e.clientX; downY = e.clientY; downT = Date.now();
    });
    canvas.addEventListener('pointerup', (e) => {
        const moved = Math.hypot(e.clientX - downX, e.clientY - downY);
        if (moved > 6 || Date.now() - downT > 600) return;   // 拖动 → 不算点选
        if (!pointsObj) { hint('先选一张地图'); return; }

        const rect = canvas.getBoundingClientRect();
        const ndc = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1);
        const ray = new THREE.Raycaster();
        // 阈值随"地图尺度"和"相机远近"走：太小吃不中，太大点到空气里也算数
        const diag = Math.max(1e-3, bbox.getSize(new THREE.Vector3()).length());
        ray.params.Points.threshold = Math.min(0.6, Math.max(0.02, diag / 150))
                                    * Math.max(0.3, camera.position.distanceTo(controls.target) / diag);
        ray.setFromCamera(ndc, camera);
        const hit = ray.intersectObject(pointsObj, false)[0];
        if (!hit) { hint('这一下没点到点云上 —— 对准墙面/地面再点，或先放大'); return; }

        const pos = pointsObj.geometry.getAttribute('position');
        const p = hit.point;
        // 用命中的那个点的真实坐标（hit.point 是插值出来的，会飘到点云外面）
        const idx = hit.index;
        const x = idx != null ? pos.getX(idx) : p.x;
        const y = idx != null ? pos.getY(idx) : p.y;
        const z = idx != null ? pos.getZ(idx) : p.z;
        hint(`取到航点 (${x.toFixed(2)}, ${y.toFixed(2)})`);
        if (onPick) onPick(x, y, z);
    });

    if (typeof ResizeObserver !== 'undefined' && canvas.parentElement) {
        new ResizeObserver(() => resize()).observe(canvas.parentElement);
    }

    return { setCloud, setWaypoints, resize, clear, render };
}
