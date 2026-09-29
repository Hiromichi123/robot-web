/**
 * pcd（Point Cloud Data）解析 —— **纯函数，不依赖 three / DOM**，所以能在 node 里单测。
 *
 * 读的是 lidar_recorder 写出来的那一种：`x y z intensity rgb`，20 字节/点，DATA binary。
 * 但字段位置是按头里的 FIELDS/SIZE/TYPE/COUNT **泛化算**出来的，不假设固定布局 ——
 * 以后加字段、换顺序都不用改这里。
 *
 * rgb 是以 float 存着**打包后的位**（PCL 的老规矩）：把 float32 的位当 uint32 读，
 * 里面是 0x00RRGGBB。没有颜色的点写的是 **NaN**（写 0 会被当纯黑，地图上糊一片黑斑）。
 *
 * 单导入方模块（仅 mission-map.js 引用），可独立 ?v= 版本号。
 */

/**
 * @param {ArrayBuffer} buf 整个 pcd 文件
 * @returns {{positions: Float32Array, colors: Float32Array|null, count: number}}
 *          positions 是 n*3（已过滤掉坐标非有限的点）；colors 为 null = 整份图没有颜色
 * @throws {Error} 头不完整 / 不是 binary / 点数与数据长度对不上
 */
export function parsePcd(buf) {
    const bytes = new Uint8Array(buf);
    // 头是 ASCII，到 "DATA <fmt>" 那行为止（先解 4KB 足够，头部就几百字节）
    const headText = new TextDecoder('ascii').decode(bytes.subarray(0, Math.min(bytes.length, 4096)));
    const lines = headText.split('\n');
    const header = {};
    let dataLine = -1;
    for (let i = 0; i < lines.length; ++i) {
        const line = lines[i].trim();
        if (!line || line.startsWith('#')) continue;
        const sp = line.indexOf(' ');
        if (sp < 0) continue;
        const key = line.slice(0, sp).toUpperCase();
        header[key] = line.slice(sp + 1).trim();
        if (key === 'DATA') { dataLine = i; break; }
    }
    const field = (k) => (header[k] != null ? header[k].split(/\s+/) : null);
    const names = field('FIELDS');
    const sizes = field('SIZE');
    const types = field('TYPE');
    const counts = field('COUNT');
    const dataFmt = (header.DATA || '').toLowerCase();
    if (!names || !sizes || !types) throw new Error('pcd 头缺少 FIELDS/SIZE/TYPE');
    if (dataFmt !== 'binary') throw new Error(`只认 DATA binary（这份是 "${dataFmt || '?'}"）`);
    const n = Number(header.POINTS || header.WIDTH || 0);
    if (!Number.isFinite(n) || n <= 0) throw new Error('pcd 头里没有有效的 POINTS');

    // 每个字段在一条记录里的偏移（按 SIZE 累加，COUNT>1 的按个数算）
    const offsets = {};
    let stride = 0;
    for (let i = 0; i < names.length; ++i) {
        const cnt = counts ? Number(counts[i] || 1) : 1;
        offsets[names[i]] = { off: stride, type: (types[i] || 'F').toUpperCase(), cnt };
        stride += Number(sizes[i] || 4) * cnt;
    }
    // 数据起点 = 头（含 DATA 行）之后的第一个字节
    const headBytes = lines.slice(0, dataLine + 1).join('\n').length + 1;
    if (bytes.length - headBytes < n * stride) {
        throw new Error(`数据长度不够：头部说 ${n} 点 × ${stride} 字节，实际只有 ${bytes.length - headBytes}`);
    }
    const view = new DataView(buf, headBytes);

    const readNum = (base, key) => {
        const f = offsets[key];
        if (!f) return NaN;
        if (f.type === 'F') return view.getFloat32(base + f.off, true);
        if (f.type === 'U') return view.getUint32(base + f.off, true);
        return view.getInt32(base + f.off, true);
    };
    if (!(offsets.x && offsets.y && offsets.z)) throw new Error('pcd 里没有 x/y/z 字段');

    const positions = new Float32Array(n * 3);
    const colors = new Float32Array(n * 3);
    let coloredCount = 0;
    let valid = 0;
    const hasRgb = !!(offsets.rgb || offsets.rgba);
    const rgbKey = offsets.rgb ? 'rgb' : 'rgba';

    for (let i = 0; i < n; ++i) {
        const base = i * stride;
        const x = readNum(base, 'x'), y = readNum(base, 'y'), z = readNum(base, 'z');
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
        positions[valid * 3] = x;
        positions[valid * 3 + 1] = y;
        positions[valid * 3 + 2] = z;

        if (hasRgb) {
            const c = readNum(base, rgbKey);
            if (Number.isFinite(c)) {           // NaN = 这个点没颜色
                const bits = new Uint32Array(Float32Array.of(c).buffer)[0];
                colors[valid * 3]     = ((bits >> 16) & 0xff) / 255;
                colors[valid * 3 + 1] = ((bits >> 8) & 0xff) / 255;
                colors[valid * 3 + 2] = (bits & 0xff) / 255;
                ++coloredCount;
            }
        }
        ++valid;
    }
    return {
        positions: positions.slice(0, valid * 3),
        colors: coloredCount > 0 ? colors.slice(0, valid * 3) : null,
        count: valid,
    };
}
