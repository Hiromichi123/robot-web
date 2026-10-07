#!/usr/bin/env python3
"""web 地面站的服务器：静态文件 + 地图管理接口 + 调试页启动/日志接口。

为什么不用 `python3 -m http.server`：
    地图管理页签要**列出 / 删除 / 读取**板子上的 pcd，而浏览器碰不到文件系统，
    必须有后端。合成一个进程比另起一个端口省事 —— 页面同源，不用处理 CORS，
    也不会再多一个"忘了起"的东西。端口和脚本名都跟以前一样。
    调试页签（2026-10-03 加）同理由这里代劳：浏览器起不了进程、也读不到日志。

接口：
    GET    /api/maps           列出 ~/lidar_maps 下的 pcd（附同名 .json 边车里的元信息）
    GET    /api/maps/<name>    取某个 pcd 的原始字节
    DELETE /api/maps/<name>    删掉 pcd 及它的同名 .json
    POST   /api/maps/save?points=&colored=&voxel=   网页「雷达页」把前端融合好的
                               彩色点云存回板上（裸二进制，每点 20 字节；详见 _map_save）
    GET    /api/debug/state    调试页：两个启动目标（slam / dog）的运行状态
    POST   /api/debug/start    调试页：{"target":"slam"|"dog"} 启动对应脚本
    POST   /api/debug/stop     调试页：{"target":…} 停**本页启动的**（进程组 SIGINT，
                               10s 未完升 SIGKILL；手工起的进程不碰，返回 409）
    GET    /api/debug/log?target=&offset=   调试页：终端日志增量转发（从 offset 起）
    其它路径                   → 本脚本所在目录（本仓库根）下的静态文件（和以前一样）

安全：文件名只允许 [A-Za-z0-9_.-]+.pcd，且解析后必须仍在 MAPS_DIR 里 ——
这个服务监听 0.0.0.0，局域网里谁都能连，不能让路径穿越把 /etc 读出去。
另外静态路径里任何一段以 `.` 开头一律 403：本脚本已收进 web 仓库根，
那边有 .git/，而 python 的 http.server **不过滤点文件** —— 不挡的话
.git/config、整个 git 历史都能被局域网里任何人拉走。
调试接口能起/停板上进程 —— 与「任务编排」页同级的能力（那个页面本来就能让狗动），
信任模型没变：这个局域网谁连得上谁就能用。按钮不弹确认框（2026-10-03 按使用方
要求去掉），点下去立即生效。
"""

import json
import os
import posixpath
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

# 静态根 = 本脚本所在目录（脚本就在 web 仓库根）。不再写死 ~/web ——
# 脚本进仓库后，仓库在哪它就在哪，不依赖 checkout 路径。
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.expanduser('~/lidar_maps')
NAME_RE = re.compile(r'^[A-Za-z0-9_.-]{1,128}\.pcd$')

API_PREFIX = '/api/maps'
API_DEBUG_PREFIX = '/api/debug'

# 网页「存回后端」收的裸点云：每点 20 字节（x y z intensity rgb，各 float32）。
# 和 lidar_recorder 写 pcd 用的布局**必须一致** —— 地图管理页读的就是那一种。
SAVE_POINT_STEP = 20
MAX_SAVE_POINTS = 2_000_000        # 40MB 上限，防一条请求把内存/磁盘打满

LOG_DIR = os.path.expanduser('~/logs')

# ── 调试页签:可一键启动的目标 ─────────────────────────────────
# 直接跑板上的既有脚本(argv 不走 shell)。日志进 LOG_DIR/<key>.log,
# pidfile 记 <key>.pid —— webserver 重启后靠它把"是不是我们起的"找回来。
# guard 是 pgrep -f 的防重模式:和手工起的进程打架时拒绝,而不是起第二份。
DEBUG_TARGETS = {
    'slam': {
        'label': '链路(start_slam.sh)',
        'argv': ['/bin/bash', os.path.expanduser('~/start_slam.sh')],
        # start_slam.sh 自己还有一道更完整的检查;这里是给页面快速判状态用
        'guard': 'slam_only.launch.py',
    },
    'dog': {
        'label': 'dog_node(网页驱动 dog_web.launch.py)',
        'argv': [os.path.expanduser('~/rosrun.sh'),
                 'ros2 launch core_2026 dog_web.launch.py'],
        # 防重:dog_node 只能有一个 —— 手工起过再从这里起会双份控制。
        # 模式用安装路径而不是裸 'dog_node':pgrep -f 是子串匹配,裸词会命中
        # 任何"命令行里提到它"的进程(实测被一条 ssh 命令的 shell 误判过一次,
        # 2026-10-03)。_pgrep 还会再滤掉 shell 类进程,双保险。
        'guard': 'core_2026/dog_node',
    },
}
_debug_procs = {}          # key -> Popen(本进程起的;空了靠 pidfile 找回)
_debug_stopping = {}       # key -> True(停止看门狗正在跑)
_debug_lock = threading.Lock()


def _pgroup_members(pgid):
    """进程组里还活着的成员(pid 列表)。僵尸(Z)不算 —— 它在等回收,不是"还在跑"。

    进程组覆盖 micromamba → ros2 launch → 全部节点:start_new_session 让 pidfile 的
    pid 就是组长,组号不随组长退出而消失(孤儿也一样在组里)—— 这是停止按钮能一次
    停干净的关键(2026-10-03 实测:单杀组长 pid 会留下一串孤儿进程)。
    """
    members = []
    try:
        names = os.listdir('/proc')
    except OSError:
        return members
    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/stat', encoding='utf-8', errors='replace') as fh:
                stat = fh.read()
            # comm 字段带括号、内容可能有空格/括号 → 从最后一个 ')' 之后切;
            # 依次是 state ppid pgrp session ...
            rest = stat.rsplit(')', 1)[1].split()
            if int(rest[2]) == pgid and rest[0] != 'Z':
                members.append(int(name))
        except (OSError, ValueError, IndexError):
            continue
    return members


def _stop_watch(key, pgid, grace_s=10.0, kill_wait_s=5.0):
    """停止看门狗(后台线程):SIGINT 后组空即完成;grace 秒仍有活口(比如 micromamba
    不理 SIGINT)→ 升 SIGKILL;再等 kill_wait 秒清账。**不持 _debug_lock 等待** ——
    页面 1s/2s 的轮询不能被一个 10 秒的请求堵住。"""
    deadline = time.time() + grace_s
    while time.time() < deadline and _pgroup_members(pgid):
        time.sleep(0.25)
    if _pgroup_members(pgid):
        try:
            os.killpg(pgid, signal.SIGKILL)
            print(f'[debug] {key}: SIGINT 后 {grace_s:.0f}s 未完,已升 SIGKILL (pgid={pgid})')
        except ProcessLookupError:
            pass
        deadline = time.time() + kill_wait_s
        while time.time() < deadline and _pgroup_members(pgid):
            time.sleep(0.25)
    left = _pgroup_members(pgid)
    if left:
        print(f'[debug] {key}: 停止后仍有残留 {left}（pgid={pgid}）')
    else:
        print(f'[debug] {key}: 已停止 (pgid={pgid})')
    with _debug_lock:
        _debug_stopping.pop(key, None)
        _debug_procs.pop(key, None)


def _debug_paths(key):
    return (os.path.join(LOG_DIR, key + '.log'),
            os.path.join(LOG_DIR, key + '.pid'))


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True        # 存在但不归我们管(理论上不会有)
    return True


def _read_pidfile(key):
    _, pidf = _debug_paths(key)
    try:
        with open(pidf, encoding='utf-8') as fh:
            info = json.load(fh)
        return int(info.get('pid')), info.get('started_at')
    except (OSError, ValueError, TypeError):
        return None, None


# pgrep -f 是"整条命令行子串匹配",会命中任何**文本里提到**该名字的进程
# (ssh 承载的命令文本、grep/vim/编辑器……)。真实目标进程的可执行文件不是
# shell 类 —— 所以把 argv[0] 是 shell 的匹配一律滤掉;宁可漏判(状态显示未运行)
# 也不能误判成 external(那会让按钮被禁、启动被 409 拒)。
_SHELL_ARGV0 = {'bash', 'sh', 'dash', 'zsh', 'ssh', 'grep', 'pgrep', 'rg'}


def _pgrep(pattern):
    """pgrep -f 的 pid 列表(pgrep 不匹配它自己;滤掉 shell 类误命中)。"""
    try:
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True,
                             text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    pids = []
    for x in out.split():
        if not x.isdigit():
            continue
        pid = int(x)
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                argv0 = fh.read().split(b'\0', 1)[0].decode(errors='replace')
        except OSError:
            continue
        if os.path.basename(argv0) in _SHELL_ARGV0:
            continue
        pids.append(pid)
    return pids


def _debug_state(key):
    """某目标的运行状态:本页起的优先(pidfile/Popen),其次 pgrep 到的手工进程。"""
    logf, _ = _debug_paths(key)
    try:
        log_size = os.path.getsize(logf)
    except OSError:
        log_size = 0
    base = {'stopping': key in _debug_stopping, 'log_size': log_size}
    pid, started_at = _read_pidfile(key)
    proc = _debug_procs.get(key)
    if proc is not None and proc.poll() is None:
        return dict(base, running=True, pid=proc.pid, source='page',
                    started_at=started_at)
    if pid and _pid_alive(pid):
        return dict(base, running=True, pid=pid, source='page',
                    started_at=started_at)
    others = _pgrep(DEBUG_TARGETS[key]['guard'])
    if others:
        return dict(base, running=True, pid=others[0], source='external',
                    started_at=None, count=len(others))
    return dict(base, running=False, pid=None, source=None, started_at=started_at)


def _map_path(name):
    """把请求里的名字变成安全的绝对路径；不合法就返回 None。"""
    name = unquote(name or '')
    if not NAME_RE.match(name):
        return None
    path = os.path.realpath(os.path.join(MAPS_DIR, name))
    if os.path.dirname(path) != os.path.realpath(MAPS_DIR):
        return None
    return path


def _list_maps():
    """列出 pcd，按修改时间倒序。元信息取自同名 .json（没有就只报文件本身）。"""
    try:
        names = [n for n in os.listdir(MAPS_DIR) if n.endswith('.pcd')]
    except FileNotFoundError:
        return []
    items = []
    for name in names:
        full = os.path.join(MAPS_DIR, name)
        if not os.path.isfile(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        item = {
            'name': name,
            'size': st.st_size,
            'mtime': st.st_mtime,
            'points': None,
            'frames': None,
            'elapsed_s': None,
            'voxel_size': None,
            'started_at': None,
            'source': None,
            'note': None,
        }
        side = full[:-4] + '.json'
        try:
            with open(side, encoding='utf-8') as fh:
                meta = json.load(fh)
            for k in ('points', 'frames', 'elapsed_s', 'voxel_size',
                      'started_at', 'source', 'note'):
                if k in meta:
                    item[k] = meta[k]
        except (OSError, ValueError):
            pass          # 边车缺失/损坏不影响条目本身可用
        items.append(item)
    items.sort(key=lambda it: it['mtime'], reverse=True)
    return items


class Handler(SimpleHTTPRequestHandler):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    # ── 工具 ────────────────────────────────────────────────
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _err(self, code, msg):
        self._json({'ok': False, 'error': msg}, code)

    def _api_parts(self):
        """把 /api/maps[/name] 拆成 (name 或 None)；不是本接口就返回 False。"""
        path = urlparse(self.path).path
        if path == API_PREFIX or path == API_PREFIX + '/':
            return None
        if path.startswith(API_PREFIX + '/'):
            return posixpath.basename(path[len(API_PREFIX) + 1:])
        return False

    def _has_dot_segment(self):
        """静态路径里任何一段以 `.` 开头 → 挡掉（理由见文件头「安全」）。"""
        segs = urlparse(self.path).path.split('/')
        return any(s.startswith('.') for s in segs if s)

    # ── 路由 ────────────────────────────────────────────────
    def do_GET(self):
        if self._has_dot_segment():
            return self._err(403, '不允许访问隐藏路径')
        path = urlparse(self.path).path
        if path == API_DEBUG_PREFIX + '/state':
            return self._debug_state_json()
        if path == API_DEBUG_PREFIX + '/log':
            return self._debug_log_json()
        name = self._api_parts()
        if name is False:
            return super().do_GET()
        if name is None:
            return self._json({'ok': True, 'dir': MAPS_DIR, 'maps': _list_maps()})

        path = _map_path(name)
        if not path or not os.path.isfile(path):
            return self._err(404, f'没有这个地图：{name}')
        try:
            size = os.path.getsize(path)
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(size))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            with open(path, 'rb') as fh:
                while True:
                    chunk = fh.read(1 << 20)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except (OSError, BrokenPipeError) as exc:
            print(f'[maps] 读 {name} 失败：{exc!r}', file=sys.stderr)

    def do_HEAD(self):
        # 同 do_GET 的第一道闸：http.server 的 do_HEAD 会照发 200，也别让它探测。
        if self._has_dot_segment():
            self.send_response(403)
            self.end_headers()
            return
        return super().do_HEAD()

    def do_DELETE(self):
        name = self._api_parts()
        if name is False:
            return self._err(405, '不支持的路径')
        if name is None:
            return self._err(400, '要删除哪个？请在 /api/maps/<name> 上调用')
        path = _map_path(name)
        if not path or not os.path.isfile(path):
            return self._err(404, f'没有这个地图：{name}')
        removed = []
        try:
            os.remove(path)
            removed.append(name)
            side = path[:-4] + '.json'
            if os.path.isfile(side):
                os.remove(side)
                removed.append(os.path.basename(side))
        except OSError as exc:
            return self._err(500, f'删除失败：{exc}')
        print(f'[maps] 已删除 {", ".join(removed)}')
        return self._json({'ok': True, 'removed': removed})

    # ── 地图管理：网页存回 ───────────────────────────────────
    def _map_save(self):
        """网页「雷达页」把**前端融合好**的彩色点云存回板上。

        为什么要走网页：相机与雷达同时刻看不到同一片区域（相机装在雷达正前方
        4cm，把雷达正前方 105° 扇区整个挡住了），所以真彩只能靠转动机器人、让
        相机依次扫过雷达建好的面 —— 这套累积+投影取色现在放在浏览器里做
        （见 js/lidar-view.js 的 colorAccumulatedPass），存回来时这里只负责
        落成和 lidar_recorder 一样格式的 pcd。

        body 是**裸二进制**：每点 20 字节 x y z intensity rgb（各 float32，
        rgb 是 PCL 那套 packed 位，无色写 NaN）。点数与元信息走 query string
        —— 不解析 multipart，省得引一个解析器还容易出错。
        """
        qs = parse_qs(urlparse(self.path).query)
        try:
            n = int((qs.get('points') or ['0'])[0])
        except ValueError:
            return self._err(400, 'points 不是整数')
        if n <= 0 or n > MAX_SAVE_POINTS:
            return self._err(400, f'点数不合理：{n}（上限 {MAX_SAVE_POINTS}）')
        want = n * SAVE_POINT_STEP
        try:
            got = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            got = 0
        if got != want:
            return self._err(400, f'长度对不上：收到 {got} 字节，{n} 点应为 {want}')
        body = self.rfile.read(want)
        if len(body) != want:
            return self._err(400, f'只收到 {len(body)}/{want} 字节')

        def _int(key, default=0):
            try:
                return int((qs.get(key) or [default])[0])
            except ValueError:
                return default

        def _float(key, default=0.0):
            try:
                return float((qs.get(key) or [default])[0])
            except ValueError:
                return default

        colored = _int('colored')
        voxel = _float('voxel')
        os.makedirs(MAPS_DIR, exist_ok=True)
        name = 'map_' + time.strftime('%Y%m%d_%H%M%S') + '.pcd'
        path = os.path.join(MAPS_DIR, name)
        # 头按 pcd_io.VoxelMap.save 那一份抄，字段顺序/类型一处都不能变
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\n'
            'FIELDS x y z intensity rgb\n'
            'SIZE 4 4 4 4 4\n'
            'TYPE F F F F F\n'
            'COUNT 1 1 1 1 1\n'
            f'WIDTH {n}\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {n}\n'
            'DATA binary\n'
        ).encode('ascii')
        tmp = path + '.part'      # 先写 .part 再 rename：中途断了不留半截 pcd
        try:
            with open(tmp, 'wb') as fh:
                fh.write(header)
                fh.write(body)
            os.replace(tmp, path)
            side = path[:-4] + '.json'
            with open(side, 'w', encoding='utf-8') as fh:
                json.dump({
                    'source': 'web-fusion',
                    'pcd': name,
                    'points': n,
                    'colored_points': colored,
                    'voxel_size': voxel,
                    'frames': None,
                    'elapsed_s': None,
                    'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
                    'note': ('由网页前端累积并融合的彩色点云：点云来自 Point-LIO '
                             '/cloud_registered_body（浏览器本地按最近位姿搬到世界系），'
                             '颜色是浏览器把累积地图投影到彩色相机画面取色得到的 '
                             '（只补没有颜色的点，先到先得）。相机装在雷达正前方 4cm、'
                             '两者同时刻看不到同一片区域，所以颜色只覆盖转动时相机扫过的面。'),
                }, fh, ensure_ascii=False, indent=2)
        except OSError as exc:
            try:
                os.remove(tmp)
            except OSError:
                pass
            return self._err(500, f'写盘失败：{exc}')
        print(f'[maps] 网页存回 {name}（{n} 点，其中彩 {colored}）')
        return self._json({'ok': True, 'name': name, 'points': n})


    # ── 调试页：启动目标 / 停止目标 / 状态 / 日志 ─────────────
    def do_POST(self):
        path = urlparse(self.path).path
        if path == API_PREFIX + '/save':
            return self._map_save()
        if path == API_DEBUG_PREFIX + '/start':
            return self._debug_start()
        if path == API_DEBUG_PREFIX + '/stop':
            return self._debug_stop()
        return self._err(405, '不支持的路径')

    def _debug_state_json(self):
        out = {}
        with _debug_lock:
            for key, tgt in DEBUG_TARGETS.items():
                st = _debug_state(key)
                st['label'] = tgt['label']
                out[key] = st
        return self._json({'ok': True, 'targets': out})

    def _debug_log_json(self):
        qs = parse_qs(urlparse(self.path).query)
        key = (qs.get('target') or [''])[0]
        if key not in DEBUG_TARGETS:
            return self._err(400, f'没有这个目标：{key}')
        try:
            offset = max(0, int((qs.get('offset') or ['0'])[0]))
        except ValueError:
            offset = 0
        logf, _ = _debug_paths(key)
        try:
            size = os.path.getsize(logf)
        except OSError:
            size = 0
        if offset > size:
            offset = 0        # 日志换了一轮（启动时截断）→ 从头来
        text = ''
        try:
            with open(logf, 'rb') as fh:
                fh.seek(offset)
                text = fh.read().decode('utf-8', errors='replace')
        except OSError as exc:
            return self._err(500, f'读日志失败：{exc}')
        return self._json({'ok': True, 'text': text, 'offset': size, 'size': size})

    def _debug_start(self):
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}')
        except (ValueError, OSError):
            return self._err(400, '请求体不是 JSON')
        key = str(body.get('target') or '')
        tgt = DEBUG_TARGETS.get(key)
        if tgt is None:
            return self._err(400, f'没有这个目标：{key}')
        with _debug_lock:
            if key in _debug_stopping:
                return self._err(409, f"{tgt['label']} 正在停止中，等它停干净再启动")
            st = _debug_state(key)
            if st['running']:
                who = '本页启动的' if st['source'] == 'page' else '手工/别处启动的'
                return self._err(
                    409, f"{tgt['label']} 已在跑（{who}，pid {st['pid']}）—— 先停掉再从这里起")
            logf, pidf = _debug_paths(key)
            try:
                os.makedirs(LOG_DIR, exist_ok=True)
                fh = open(logf, 'w', encoding='utf-8', errors='replace')
                fh.write(f"=== {tgt['label']} · 由网页调试页启动 · "
                         f"{time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
                fh.flush()
                # start_new_session：脱离本服务的会话 —— 本服务重启/退出不带走它
                proc = subprocess.Popen(
                    tgt['argv'], stdout=fh, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, cwd=os.path.expanduser('~'),
                    start_new_session=True)
                fh.close()
            except OSError as exc:
                return self._err(500, f'启动失败：{exc}')
            _debug_procs[key] = proc
            try:
                with open(pidf, 'w', encoding='utf-8') as pf:
                    json.dump({'pid': proc.pid,
                               'started_at': time.strftime('%Y-%m-%d %H:%M:%S')}, pf)
            except OSError:
                pass           # pidfile 写不了不影响进程本身，只是重启后认不出
            print(f'[debug] 启动 {key}: pid={proc.pid} log={logf}')
        return self._json({'ok': True, 'target': key, 'pid': proc.pid})

    def _debug_stop(self):
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}')
        except (ValueError, OSError):
            return self._err(400, '请求体不是 JSON')
        key = str(body.get('target') or '')
        tgt = DEBUG_TARGETS.get(key)
        if tgt is None:
            return self._err(400, f'没有这个目标：{key}')
        with _debug_lock:
            if key in _debug_stopping:
                return self._json({'ok': True, 'stopping': True})
            # 只停**本页启动过**的(pidfile 就是启动记录)。手工起的进程永远不碰 ——
            # pgrep"看见"它只用于状态展示,不能拿来杀(可能在别的终端里调试)。
            pid, _ = _read_pidfile(key)
            if not pid:
                return self._err(
                    409, f"本页没有启动过{tgt['label']}；手工起的请去终端 Ctrl-C（本页不代为停止）")
            if not _pgroup_members(pid):
                _debug_procs.pop(key, None)
                return self._json({'ok': True, 'already_stopped': True})
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                _debug_procs.pop(key, None)
                return self._json({'ok': True, 'already_stopped': True})
            except PermissionError:
                return self._err(500, '没有权限向该进程组发信号')
            _debug_stopping[key] = True
            print(f'[debug] 停止 {key}: SIGINT → pgid={pid}')
        # 看门狗在后台线程等退出/升杀,这里立刻返回(轮询不被堵住)
        threading.Thread(target=_stop_watch, args=(key, pid), daemon=True).start()
        return self._json({'ok': True, 'stopping': True})

    def log_message(self, fmt, *args):
        # 静态请求太吵（页面一刷新几十条），只留 API 的；
        # 调试页的 /log 是 1s 一次轮询，也不记（start/state 的动静另有 print）
        if self.path.startswith(API_PREFIX):
            super().log_message(fmt, *args)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    os.makedirs(MAPS_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    print(f'web 地面站: http://0.0.0.0:{port}/  (静态目录 {WEB_DIR})')
    print(f'地图接口  : GET/DELETE /api/maps · POST /api/maps/save   地图目录 {MAPS_DIR}')
    print(f'调试接口  : GET /api/debug/state · POST /api/debug/start · '
          f'GET /api/debug/log   日志目录 {LOG_DIR}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
