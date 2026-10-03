#!/usr/bin/env python3
"""web 地面站的服务器：静态文件 + 地图管理接口。

为什么不用 `python3 -m http.server`：
    地图管理页签要**列出 / 删除 / 读取**板子上的 pcd，而浏览器碰不到文件系统，
    必须有后端。合成一个进程比另起一个端口省事 —— 页面同源，不用处理 CORS，
    也不会再多一个"忘了起"的东西。端口和脚本名都跟以前一样。

接口：
    GET    /api/maps           列出 ~/lidar_maps 下的 pcd（附同名 .json 边车里的元信息）
    GET    /api/maps/<name>    取某个 pcd 的原始字节
    DELETE /api/maps/<name>    删掉 pcd 及它的同名 .json
    其它路径                   → 本脚本所在目录（本仓库根）下的静态文件（和以前一样）

安全：文件名只允许 [A-Za-z0-9_.-]+.pcd，且解析后必须仍在 MAPS_DIR 里 ——
这个服务监听 0.0.0.0，局域网里谁都能连，不能让路径穿越把 /etc 读出去。
另外静态路径里任何一段以 `.` 开头一律 403：本脚本已收进 web 仓库根，
那边有 .git/，而 python 的 http.server **不过滤点文件** —— 不挡的话
.git/config、整个 git 历史都能被局域网里任何人拉走。
"""

import json
import os
import posixpath
import re
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

# 静态根 = 本脚本所在目录（脚本就在 web 仓库根）。不再写死 ~/web ——
# 脚本进仓库后，仓库在哪它就在哪，不依赖 checkout 路径。
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.expanduser('~/lidar_maps')
NAME_RE = re.compile(r'^[A-Za-z0-9_.-]{1,128}\.pcd$')

API_PREFIX = '/api/maps'


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

    def log_message(self, fmt, *args):
        # 静态请求太吵（页面一刷新几十条），只留 API 的
        if self.path.startswith(API_PREFIX):
            super().log_message(fmt, *args)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    os.makedirs(MAPS_DIR, exist_ok=True)
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    print(f'web 地面站: http://0.0.0.0:{port}/  (静态目录 {WEB_DIR})')
    print(f'地图接口  : GET/DELETE /api/maps   地图目录 {MAPS_DIR}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
