#!/usr/bin/env python3
"""SLAM Toolbox가 발행하는 /map(OccupancyGrid)을 실시간으로 그려서 보여주는 대시보드.

실제 맵핑 주행(mapping.launch.py) 시작 전에 미리 띄워두는 용도 — SLAM Toolbox가
아직 안 떠있으면 그냥 "데이터 없음" 상태로 대기하다가, 맵핑을 시작하면 자동으로
그리드가 채워지는 걸 브라우저에서 실시간으로 볼 수 있음.

의존성 추가 없이(Pillow 등 이미지 라이브러리 없이) 그리드 원시 바이트를 base64로
클라이언트에 보내고, 브라우저 <canvas>에서 직접 픽셀을 찍어 그림.

사용법:
    python3 tools/map_dashboard.py [--port 8768]
    (SLAM Toolbox가 /map을 발행하기 전까진 "데이터 없음" 상태로 대기)
"""
import argparse
import base64
import gzip
import json
import os
import re
import signal
import threading
import time
from dashboard_icon import FAVICON_TAG
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

state_lock = threading.Lock()
state = {"connected": False, "last_ts": 0.0}
STALE_S = 5.0

# 비상정지 대상: 로봇에 cmd_vel을 만들어 보낼 수 있는 프로세스들.
# 2026-08-16: 전부 carrotian 소유라 같은 유저인 대시보드가 권한 없이 신호를 보낼 수 있다.
#
# ⚠️ **실행파일 이름(argv[0]의 basename)으로 정확히 일치**시킨다. 예전엔 명령줄
# 전체에 대한 부분문자열 매칭이었는데, 그러면 "bt_navigator" 같은 흔한 문자열이
# 들어간 **무관한 프로세스까지 죽인다**. 실제로 진단용 셸(`pgrep -f
# 'nav2_bt_navigator|...'`)이 비상정지에 휘말려 죽는 걸 확인했다 — 실전에서는
# 사용자의 SSH 세션이나 모니터링 명령이 같은 이유로 죽을 수 있어서 위험하다.
# 비상정지 로직은 모터 대시보드와 **같은 코드**를 쓴다(estop_common).
# 복사해 두면 한쪽만 고쳐져 어긋나고, 하필 비상정지에서 그러면 위험하다.
from estop_common import kill_autonomous_processes


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_dashboard_bridge')
        # /map은 **TRANSIENT_LOCAL(래치)** 로 구독해야 한다. 기본 VOLATILE로 두면
        # 발행자가 이미 낸 마지막 맵을 못 받고, 다음 발행 때까지 "응답 없음"으로 보인다.
        # SLAM은 주기적으로 재발행해서 가려져 있었지만, map_server는 활성화 때 한 번만
        # 내므로 저장된 맵을 띄우면 화면이 영영 비어 있었다(2026-08-17 확인).
        # 대시보드를 재시작해도 즉시 마지막 맵이 뜨는 이점도 있다.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        # 비상정지용 — 자율주행이 계속 새 명령을 낼 수도 있으니 여러 번 강제로 0 발행.
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def emergency_stop(self):
        """0 속도를 여러 번 발행. **호출자는 이게 멈출 수 있다고 가정해야 한다.**

        2026-08-16: DDS 상태가 꼬이면(대량 kill -9 / /dev/shm 정리 후 등) 이
        publish가 무한정 안 돌아오는 걸 실제로 겪음 — 그때 비상정지 API 전체가
        응답을 안 했다. 그래서 이건 이제 '보조 수단'이고, 진짜 정지는
        프로세스를 죽이는 쪽(아래 do_POST)이 담당한다.
        """
        for _ in range(5):
            self._cmd_pub.publish(Twist())
            time.sleep(0.05)

    def emergency_stop_async(self, timeout=1.5):
        """emergency_stop을 별도 스레드로 돌려서, 멈춰도 호출자를 막지 않게 한다.

        반환값은 시간 내에 끝났는지 여부(진단용).
        """
        t = threading.Thread(target=self.emergency_stop, daemon=True)
        t.start()
        t.join(timeout=timeout)
        return not t.is_alive()

    def _on_map(self, msg: OccupancyGrid):
        # OccupancyGrid.data는 int8[] (-1=unknown, 0=free, 100=occupied).
        # 파이썬에서 그대로 bytes로 바꾸면 부호 처리가 꼬이므로 unsigned로 변환해서 전송,
        # 클라이언트에서 다시 signed로 해석함(-1은 255로 옴).
        raw = bytes((v & 0xFF) for v in msg.data)
        with state_lock:
            state.update({
                "connected": True,
                "last_ts": time.time(),
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "data_b64": base64.b64encode(raw).decode("ascii"),
            })


def staleness_watchdog():
    """맵의 '신선도'를 감시하되 **화면에서 지우지는 않는다.**

    /map은 스트리밍이 아니라 스냅샷이다. SLAM은 주기적으로 재발행하지만
    map_server(저장된 맵 열람)는 활성화 때 **딱 한 번** 래치로 발행한다. 예전엔
    STALE_S초 뒤 connected=False로 바꿔서 **맵이 잠깐 떴다가 사라졌다**(2026-08-17).

    맵은 한 번 받으면 계속 유효하다 — 오래됐다는 표시만 하고 그림은 남긴다.
    (라이다/IMU 같은 실시간 센서와 성격이 다르므로 같은 잣대를 쓰면 안 된다)
    """
    while True:
        with state_lock:
            if state.get("last_ts"):
                state["stale"] = (time.time() - state["last_ts"]) > STALE_S
        time.sleep(1.0)



# ---------------- 저장된 맵 관리 (2026-08-21) ----------------
#
# 오래전부터 할일에 있던 "맵 목록/보기/삭제" 기능.
#
# ⚠️ ~/maps에는 맵 말고도 진단 도구가 만든 .json/.png가 섞여 있다.
#    **.pgm과 .yaml이 쌍으로 있는 것만** 맵으로 본다.
# ⚠️ 사용자 입력이 파일 경로에 들어가므로 이름을 화이트리스트로 제한하고,
#    최종 경로가 MAPS_DIR 밖을 가리키면 거부한다(심볼릭 링크 포함).

MAPS_DIR = os.path.expanduser("~/maps")
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def _safe_map_path(name, ext):
    """검증된 MAPS_DIR 내부 경로만 돌려준다. 벗어나면 None."""
    if not SAFE_NAME.match(name or ""):
        return None
    root = os.path.abspath(MAPS_DIR)
    p = os.path.abspath(os.path.join(root, name + ext))
    if os.path.commonpath([p, root]) != root:
        return None
    return p


def _parse_map_yaml(path):
    """맵 yaml을 읽는다. PyYAML을 쓰지 않는다 — 이 도구는 의존성 없이 도는 게
    원칙이고, 형식이 `키: 값` 몇 줄로 고정돼 있다."""
    out = {}
    try:
        with open(path) as f:
            for line in f:
                if ":" not in line or line.strip().startswith("#"):
                    continue
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
    except OSError:
        return None
    return out


def _read_pgm(path):
    """P5 PGM -> (width, height, 데이터 bytes). 주석 줄도 건너뛴다."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
    except OSError:
        return None
    if not raw.startswith(b"P5"):
        return None
    vals, i = [], 2
    while len(vals) < 3 and i < len(raw):
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            while i < len(raw) and raw[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        try:
            vals.append(int(raw[i:j]))
        except ValueError:
            return None
        i = j
    if len(vals) < 3:
        return None
    i += 1                      # maxval 뒤 공백 1개
    w, h = vals[0], vals[1]
    body = raw[i:i + w * h]
    if len(body) < w * h:
        return None
    return w, h, body


def list_maps():
    out = []
    if not os.path.isdir(MAPS_DIR):
        return out
    for fn in sorted(os.listdir(MAPS_DIR)):
        if not fn.endswith(".yaml"):
            continue
        name = fn[:-5]
        pgm = os.path.join(MAPS_DIR, name + ".pgm")
        if not os.path.isfile(pgm):
            continue                      # 쌍이 아니면 맵이 아니다
        meta = _parse_map_yaml(os.path.join(MAPS_DIR, fn)) or {}
        try:
            res = float(meta.get("resolution", 0) or 0)
        except ValueError:
            res = 0.0
        info = {"name": name, "bytes": os.path.getsize(pgm),
                "mtime": os.path.getmtime(pgm), "resolution": res}
        pg = _read_pgm(pgm)
        if pg:
            w, h, body = pg
            info["width"], info["height"] = w, h
            known = sum(1 for b in body if b != 205)      # 205 = 미지
            info["area_m2"] = known * res * res
        out.append(info)
    out.sort(key=lambda x: -x["mtime"])
    return out


def read_saved_map(name):
    """저장된 맵을 **라이브 맵과 같은 형식**으로 돌려준다.
    그래야 브라우저의 그리기 코드를 그대로 재사용할 수 있다."""
    ypath = _safe_map_path(name, ".yaml")
    ppath = _safe_map_path(name, ".pgm")
    if not ypath or not ppath or not os.path.isfile(ppath):
        return None
    meta = _parse_map_yaml(ypath) or {}
    pg = _read_pgm(ppath)
    if not pg:
        return None
    w, h, body = pg
    # PGM은 위->아래, OccupancyGrid는 아래->위 이므로 행을 뒤집는다.
    # 값 변환: 205=미지(-1을 255로), 0=장애물(100), 254=빈공간(0)
    out = bytearray(w * h)
    for r in range(h):
        src = (h - 1 - r) * w
        dst = r * w
        for c in range(w):
            v = body[src + c]
            out[dst + c] = 255 if v == 205 else (100 if v < 128 else 0)
    try:
        origin = [float(x) for x in
                  meta.get("origin", "[0,0,0]").strip("[] ").split(",")]
    except ValueError:
        origin = [0.0, 0.0, 0.0]
    try:
        res = float(meta.get("resolution", 0.05) or 0.05)
    except ValueError:
        res = 0.05
    return {"connected": True, "saved": True, "name": name,
            "last_ts": os.path.getmtime(ppath), "stale": False,
            "width": w, "height": h, "resolution": res,
            "origin_x": origin[0] if origin else 0.0,
            "origin_y": origin[1] if len(origin) > 1 else 0.0,
            "data_b64": base64.b64encode(bytes(out)).decode("ascii")}


def save_current_map(name):
    """현재 /map을 PGM+YAML로 저장. 형식은 map_autosave.py와 같아야 한다."""
    ypath = _safe_map_path(name, ".yaml")
    ppath = _safe_map_path(name, ".pgm")
    if not ypath or not ppath:
        return None, "이름은 영문/숫자/밑줄/하이픈 1~64자만 가능"
    with state_lock:
        if not state.get("data_b64"):
            return None, "아직 받은 맵이 없다"
        w, h = state["width"], state["height"]
        res, ox, oy = state["resolution"], state["origin_x"], state["origin_y"]
        raw = base64.b64decode(state["data_b64"])
    try:
        os.makedirs(MAPS_DIR, exist_ok=True)
        with open(ppath, "wb") as f:
            f.write(("P5\n%d %d\n255\n" % (w, h)).encode())
            for r in range(h - 1, -1, -1):        # PGM은 위에서 아래로
                line = bytearray(w)
                for c in range(w):
                    b = raw[r * w + c]
                    v = -1 if b > 100 else b      # 255였던 -1 복원
                    line[c] = 205 if v < 0 else (0 if v >= 65 else 254)
                f.write(bytes(line))
        with open(ypath, "w") as f:
            # ⚠️ image에 **실제 파일명**을 쓴다. map_autosave.py는 이 줄을
            #    'latest.pgm'으로 고정해 놔서, 다른 이름으로 복사하면 깨진다
            #    (2026-08-17에 실제로 겪어 수동으로 고쳤다).
            f.write("image: %s.pgm\n" % name)
            f.write("resolution: %s\n" % res)
            f.write("origin: [%s, %s, 0.0]\n" % (ox, oy))
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    except OSError as e:
        return None, str(e)
    return name, None


def delete_map(name):
    """맵 하나 삭제. **pgm/yaml 쌍만** 지운다 — 같은 이름의 png/json은 안 건드린다."""
    removed = []
    for ext in (".pgm", ".yaml"):
        p = _safe_map_path(name, ext)
        if p and os.path.isfile(p):
            try:
                os.unlink(p)
                removed.append(os.path.basename(p))
            except OSError as e:
                return removed, str(e)
    if not removed:
        return removed, "그런 맵이 없다"
    return removed, None


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
__FAVICON__
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>맵 대시보드</title>
<style>
  .banner { padding:.5rem .75rem; border-radius:6px; margin:.5rem 0;
            background:#8a5a00; color:#fff; font-size:.9rem; }
  .saverow { display:flex; gap:.5rem; align-items:center; flex-wrap:wrap;
             margin-bottom:.6rem; }
  .saverow input { flex:1 1 12rem; padding:.4rem .5rem; border-radius:6px;
                   border:1px solid var(--border); background:transparent;
                   color:inherit; }
  button.mini { padding:.35rem .7rem; font-size:.85rem; border-radius:6px;
                cursor:pointer; }
  button.danger { color:#c33; }
  .msg { font-size:.85rem; color:var(--text-secondary); }
  #mapTable td { padding:.35rem .5rem; border-bottom:1px solid var(--border);
                 font-variant-numeric: tabular-nums; }
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10);
    --good: #0ca30c; --critical: #d03b3b;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10);
      --good: #0ca30c; --critical: #e66767;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --border: rgba(255,255,255,0.10);
    --good: #0ca30c; --critical: #e66767;
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 720px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; display: flex; align-items: center; gap: 8px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  #mapCanvas { display: block; margin: 0 auto; background: #808080;
    image-rendering: pixelated; max-width: 100%; border: 1px solid var(--border); }
  .status-row { display: flex; justify-content: space-between; font-size: 0.85rem;
    color: var(--text-secondary); margin-top: 8px; font-variant-numeric: tabular-nums; }
  .empty { text-align: center; color: var(--text-muted); padding: 60px 0; }
  #estopBtn { display: block; width: 100%; padding: 18px 0; margin: 16px 0;
    background: var(--critical); color: white; border: none; border-radius: 12px;
    font-size: 1.2rem; font-weight: 700; letter-spacing: 0.05em; cursor: pointer; }
  #estopBtn:active { filter: brightness(0.85); }
  #estopMsg { text-align: center; font-size: 0.85rem; margin-top: -8px; margin-bottom: 8px;
    color: var(--text-secondary); min-height: 1.2em; }
  a.nav-home { color: var(--accent, #2a78d6); text-decoration: none; font-size: 0.88rem; font-weight: 500;
    display: inline-flex; align-items: center; gap: 4px; padding: 4px 10px; border-radius: 6px;
    background: var(--surface-1); border: 1px solid var(--border); margin-bottom: 12px; }
  a.nav-home:hover { opacity: 0.8; }
</style>
</head>
<body>
<a class="nav-home" href="#" onclick="location.port='8764'; return false;">← 🏠 시스템 대시보드 (홈)</a>
<h1><span class="dot" id="connDot" style="background:var(--critical)"></span> 맵 대시보드</h1>
<button id="estopBtn">🛑 비상정지 (자율주행 전체 중단)</button>
<div id="estopMsg"></div>
<div id="viewBanner" class="banner" style="display:none"></div>

<div class="card">
  <canvas id="mapCanvas" width="10" height="10"></canvas>
  <div id="emptyMsg" class="empty">맵 데이터 없음 — SLAM Toolbox(mapping.launch.py)를 실행하면 여기 실시간으로 채워집니다.</div>
  <div class="status-row">
    <span id="dimText"></span>
    <span id="updatedText"></span>
  </div>
</div>

<div class="card">
  <div class="card-head">저장된 맵</div>
  <div class="saverow">
    <input id="saveName" placeholder="이름 (영문/숫자/_/-)" maxlength="64">
    <button id="saveBtn" class="mini">현재 맵 저장</button>
    <span id="saveMsg" class="msg"></span>
  </div>
  <table id="mapTable"></table>
</div>

<script>
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');

// occupancy(-1~100)를 회색조로: -1(미지)=중간회색, 0(빈공간)=흰색, 100(장애물)=검정
function occColor(v) {
  if (v === -1) return 128;
  const clamped = Math.max(0, Math.min(100, v));
  return Math.round(255 * (1 - clamped / 100));
}

// viewing === null 이면 라이브, 문자열이면 그 이름의 저장된 맵을 본다.
// let은 TDZ가 있어 tick()보다 먼저 선언돼야 한다.
let viewing = null;

// 격자 그리기. **라이브 맵과 저장된 맵이 같은 형식**이라 이 함수를 공유한다.
function renderGrid(d) {
  const bin = atob(d.data_b64);
  const w = d.width, h = d.height;
  canvas.width = w;
  canvas.height = h;
  const imgData = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    const raw = bin.charCodeAt(i);           // 0~255 (unsigned로 옴)
    const v = raw > 100 ? -1 : raw;          // 255였던 -1 복원
    const g = occColor(v);
    // OccupancyGrid는 (0,0)이 좌하단부터 row-major라 캔버스(좌상단 시작)로 옮길 때 y 뒤집기
    const row = Math.floor(i / w);
    const col = i % w;
    const destRow = h - 1 - row;
    const destIdx = (destRow * w + col) * 4;
    imgData.data[destIdx] = g;
    imgData.data[destIdx + 1] = g;
    imgData.data[destIdx + 2] = g;
    imgData.data[destIdx + 3] = 255;
  }
  ctx.putImageData(imgData, 0, 0);

  document.getElementById('dimText').textContent =
    `${w}×${h} 셀, 해상도 ${d.resolution.toFixed(3)}m/셀 (약 ${(w*d.resolution).toFixed(1)}×${(h*d.resolution).toFixed(1)}m)`;
  document.getElementById('updatedText').textContent =
    (d.saved ? '저장 시각: ' : '마지막 갱신: ')
    + new Date(d.last_ts * 1000).toLocaleTimeString()
    + (d.stale ? '  (갱신 멈춤 — SLAM 정지)' : '');
}

async function tick() {
  // 저장된 맵을 보는 중이면 라이브로 덮어쓰지 않는다.
  if (viewing !== null) return;
  // 탭이 숨겨져 있으면 폴링하지 않는다 — 자율주행 중 CPU 경쟁을 줄인다.
  // 이 프로젝트에서 프리즈의 유력한 방아쇠가 CPU 부하였다.
  if (document.hidden) return;
  let d;
  try { d = await (await fetch('/api/map')).json(); } catch (e) { return; }
  drawPayload(d);
}

tick();
// ---------------- 저장된 맵 관리 ----------------

function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return `${d.getMonth()+1}/${d.getDate()} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
}

async function loadMapList() {
  let list;
  try { list = await (await fetch('/api/maps')).json(); } catch (e) { return; }
  const rows = list.map(m => {
    const wh = (m.width && m.height) ? `${m.width}×${m.height}` : '-';
    const area = (m.area_m2 != null) ? `${m.area_m2.toFixed(1)} m²` : '-';
    const cur = (viewing === m.name) ? ' ← 보는 중' : '';
    return `<tr>
      <td><b>${m.name}</b>${cur}</td>
      <td>${wh}</td>
      <td>${area}</td>
      <td>${fmtTime(m.mtime)}</td>
      <td>
        <button class="mini" data-view="${m.name}">보기</button>
        <button class="mini danger" data-del="${m.name}">삭제</button>
      </td></tr>`;
  }).join('');
  document.getElementById('mapTable').innerHTML =
    rows || '<tr><td>저장된 맵이 없습니다.</td></tr>';
}

function drawPayload(d) {
  document.getElementById('connDot').style.background =
    d.connected ? 'var(--good)' : 'var(--critical)';
  if (!d.connected || !d.data_b64) {
    document.getElementById('emptyMsg').style.display = 'block';
    canvas.style.display = 'none';
    return;
  }
  document.getElementById('emptyMsg').style.display = 'none';
  canvas.style.display = 'block';
  renderGrid(d);
}

async function viewSaved(name) {
  let d;
  try { d = await (await fetch('/api/maps/data?name=' + encodeURIComponent(name))).json(); }
  catch (e) { return; }
  if (d.error) { alert(d.error); return; }
  viewing = name;
  const b = document.getElementById('viewBanner');
  b.style.display = 'block';
  b.innerHTML = `저장된 맵 <b>${name}</b> 을 보는 중입니다 (실시간 아님). ` +
                `<button class="mini" id="backLive">실시간으로 돌아가기</button>`;
  document.getElementById('backLive').onclick = () => {
    viewing = null;
    document.getElementById('viewBanner').style.display = 'none';
    loadMapList(); tick();
  };
  drawPayload(d);
  loadMapList();
}

document.getElementById('mapTable').addEventListener('click', async (e) => {
  const v = e.target.getAttribute('data-view');
  const del = e.target.getAttribute('data-del');
  if (v) { viewSaved(v); return; }
  if (del) {
    // 파일을 지우는 동작이라 되돌릴 수 없다 — 반드시 확인을 받는다.
    if (!confirm(`맵 "${del}" 을 삭제합니다.\n.pgm 과 .yaml 이 지워지며 되돌릴 수 없습니다.`)) return;
    const r = await (await fetch('/api/maps/delete', {
      method: 'POST', body: JSON.stringify({name: del})})).json();
    if (!r.ok) { alert(r.error || '삭제 실패'); return; }
    if (viewing === del) {
      viewing = null;
      document.getElementById('viewBanner').style.display = 'none';
    }
    loadMapList();
  }
});

document.getElementById('saveBtn').onclick = async () => {
  const name = document.getElementById('saveName').value.trim();
  const msg = document.getElementById('saveMsg');
  if (!name) { msg.textContent = '이름을 입력하세요'; return; }
  msg.textContent = '저장 중...';
  const r = await (await fetch('/api/maps/save', {
    method: 'POST', body: JSON.stringify({name})})).json();
  msg.textContent = r.ok ? `저장됨: ${r.name}` : (r.error || '저장 실패');
  if (r.ok) { document.getElementById('saveName').value = ''; loadMapList(); }
};

loadMapList();
setInterval(() => { if (!document.hidden) loadMapList(); }, 10000);
// 탭이 다시 보이면 즉시 한 번 갱신 (숨은 동안 멈춰 있었으므로)
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) { tick(); loadMapList(); }
});
setInterval(tick, 1500);

document.getElementById('estopBtn').addEventListener('click', async () => {
  const msg = document.getElementById('estopMsg');
  msg.textContent = '정지 명령 전송 중...';
  try {
    const res = await (await fetch('/api/estop', {method: 'POST'})).json();
    const base = res.killed && res.killed.length
      ? `정지됨 — 종료된 프로세스: ${res.killed.join(', ')}`
      : '정지 명령 발행됨 (실행 중이던 자율주행 프로세스는 없었음)';
    // zero_published=false면 ROS 발행이 막힌 것. 프로세스는 죽였으므로
    // diff_drive의 cmd_vel_timeout(0.5초)으로 정지는 되지만, 알려는 준다.
    msg.textContent = res.zero_published === false
      ? base + ' ⚠️ 0속도 발행 실패 — 0.5초 내 자동정지로 대체됨'
      : base;
  } catch (e) {
    msg.textContent = '정지 요청 실패 — 네트워크 확인 필요';
  }
});
</script>
</body>
</html>
"""

# 파비콘 삽입. PAGE가 일반 문자열이라 f-string 치환을 못 쓰므로 여기서 바꾼다.
PAGE = PAGE.replace("__FAVICON__", FAVICON_TAG)



class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        # 클라이언트가 받아준다면 압축해서 보낸다. 맵 그리드나 JSON 반복 필드는
        # 10배 가까이 줄어든다. WiFi가 약한 구간에서 이게 응답성을 좌우한다.
        enc = None
        if len(body) > 1024 and "gzip" in (self.headers.get("Accept-Encoding") or ""):
            packed = gzip.compress(body, 6)
            if len(packed) < len(body):
                body, enc = packed, "gzip"
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        if enc:
            self.send_header("Content-Encoding", enc)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/map":
            with state_lock:
                self._send(200, json.dumps(state).encode("utf-8"), "application/json")
        elif self.path == "/api/maps":
            self._send(200, json.dumps(list_maps()).encode("utf-8"),
                       "application/json")
        elif self.path.startswith("/api/maps/data"):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            d = read_saved_map((q.get("name") or [""])[0])
            if d is None:
                self._send(404, json.dumps({"error": "맵을 읽을 수 없다"}).encode(),
                           "application/json")
            else:
                self._send(200, json.dumps(d).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/estop":
            # ⚠️ 순서가 중요하다. 예전엔 0 속도 발행을 먼저 했는데, DDS가 꼬여서
            # publish가 안 돌아오면 뒤의 pkill까지 영영 도달을 못 했다(실제로
            # 2026-08-16에 비상정지 API가 통째로 응답불능이 된 걸 확인).
            #
            # 진짜 정지는 "명령을 만들어내는 프로세스를 죽이는 것"이다 —
            # diff_drive_node가 cmd_vel이 0.5초만 끊겨도 스스로 정지하므로
            # (cmd_vel_timeout), 소스를 죽이면 0.5초 안에 반드시 선다.
            # 0 속도 발행은 그 0.5초를 더 앞당기는 보조 수단일 뿐이라 나중에,
            # 그리고 멈춰도 응답을 막지 않도록 타임아웃을 걸고 돌린다.
            killed = kill_autonomous_processes()
            zero_ok = Handler.node.emergency_stop_async(timeout=1.5)
            self._send(200, json.dumps({
                "ok": True, "killed": killed,
                # false면 ROS 발행이 멈춘 것 — 그래도 위에서 소스를 죽였으므로
                # cmd_vel_timeout으로 정지는 보장된다. UI에 경고로 표시.
                "zero_published": zero_ok,
            }).encode("utf-8"), "application/json")
        elif self.path in ("/api/maps/save", "/api/maps/delete"):
            try:
                ln = int(self.headers.get("Content-Length", "0") or 0)
                body = json.loads(self.rfile.read(ln) or b"{}")
            except (ValueError, OSError):
                body = {}
            name = (body.get("name") or "").strip()
            if self.path.endswith("/save"):
                saved, err = save_current_map(name)
                self._send(200 if saved else 400,
                           json.dumps({"ok": bool(saved), "name": saved,
                                       "error": err}).encode("utf-8"),
                           "application/json")
            else:
                removed, err = delete_map(name)
                self._send(200 if removed and not err else 400,
                           json.dumps({"ok": bool(removed) and not err,
                                       "removed": removed,
                                       "error": err}).encode("utf-8"),
                           "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8768)
    args = ap.parse_args()

    rclpy.init()
    node = MapBridge()
    Handler.node = node

    threading.Thread(target=staleness_watchdog, daemon=True).start()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)

    # 2026-08-16: 다른 대시보드들과 동일한 이유로 SIGTERM 핸들러 +
    # spin 스레드 정리 추가 (재부팅/재시작 시 지연·core dump 방지).
    def _sig_handler(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    print(f"맵 대시보드 시작: http://<Pi IP>:{args.port}")
    try:
        server.serve_forever()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
