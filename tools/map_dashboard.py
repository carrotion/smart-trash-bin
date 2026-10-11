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
import math
import os
import re
import signal
import threading
import time
import urllib.parse
from dashboard_icon import FAVICON_TAG, HOME_ICON_TAG
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import keepout

# 탐색기(frontier_exploration_ros2)의 제어 서비스와 SLAM의 지도 저장 서비스 — 패키지가 없어도 대시보드는 떠야 한다
try:
    from frontier_exploration_ros2.srv import ControlExploration
except ImportError:
    ControlExploration = None
try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:
    SerializePoseGraph = None
EXPLORE_STATE = {0: "멈춤", 1: "탐색 중", 2: "시작 예약", 3: "멈춤 예약", 4: "멈추는 중", 5: "종료 중"}

state_lock = threading.Lock()
state = {"connected": False, "last_ts": 0.0}
STALE_S = 5.0
# 금지 구역(tools/keepout.py) 화면용 상태: 구역 서버가 실제로 적용한 version, 서버 생존, 지도 위 로봇 자세
keep_state = {"applied_version": None, "applied_n": 0, "server": False, "pose": None}
_last_pose = {"pose": None, "t": 0.0}     # 마지막으로 안 자세 — 지도 변환이 잠깐 끊긴 순간에도 "로봇 옆 금지"를 검사하려고
ROBOT_KEEP_CLEAR = 0.25     # 로봇에서 이 거리 안에는 구역을 새로 만들거나 옮겨 놓지 못한다(구역 선은 벽 — 로봇이 갇힌다)

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
        # 금지 구역: 서버가 적용한 내용을 되받아 화면에 "적용됨"을 정직하게 표시(파일에 썼다고 적용된 게 아니다)
        self.create_subscription(String, '/keepout_zones', self._on_zones, map_qos)
        self._tfb = Buffer(); self._tfl = TransformListener(self._tfb, self)
        self.create_timer(0.5, self._pose_tick)
        self._explore_cli = self.create_client(ControlExploration, 'control_exploration') if ControlExploration else None
        self._ser_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map') if SerializePoseGraph else None

    def _call(self, cli, req, timeout):
        """서비스 호출을 HTTP 처리 스레드에서 기다린다(노드는 다른 스레드가 돌린다). 응답 또는 None."""
        if cli is None or not cli.service_is_ready():
            return None
        fut = cli.call_async(req); t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def explore(self, action=None):
        """탐색기 상태 조회(action None) 또는 시작('start')·멈춤('stop'). 탐색기가 없으면 available False.
        상태만 물을 땐 지원하지 않는 동작 번호 0을 보낸다 — 탐색기는 아무것도 바꾸지 않고 지금 상태만 답한다."""
        if ControlExploration is None:
            return dict(available=False)
        req = ControlExploration.Request()
        req.action = {None: 0, 'start': ControlExploration.Request.ACTION_START, 'stop': ControlExploration.Request.ACTION_STOP}[action]
        r = self._call(self._explore_cli, req, 3.0)
        if r is None:
            return dict(available=False)
        return dict(available=True, state=int(r.state), label=EXPLORE_STATE.get(int(r.state), str(r.state)),
                    accepted=bool(r.accepted) if action else None, message=r.message if action else "")

    def serialize(self, path_noext):
        """SLAM의 위치 잡기용 자료(자세 그래프)를 path_noext.posegraph / .data로 저장. 성공 여부."""
        if SerializePoseGraph is None:
            return False
        req = SerializePoseGraph.Request(); req.filename = path_noext
        r = self._call(self._ser_cli, req, 20.0)
        return bool(r is not None and r.result == 0 and os.path.isfile(path_noext + ".posegraph"))

    def _on_zones(self, msg):
        try:
            d = json.loads(msg.data)
            keep_state.update(applied_version=d.get("version"), applied_n=len(d.get("zones") or []))
        except ValueError:
            pass

    def _pose_tick(self):
        pose = None
        try:
            t = self._tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            st = self._tfb.lookup_transform('map', 'odom', rclpy.time.Time()).header.stamp
            age = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
            if age < 3.0:       # SLAM이 꺼지면 마지막 변환이 남는다 — 오래된 자세는 버린다
                q = t.rotation
                pose = [round(t.translation.x, 3), round(t.translation.y, 3),
                        round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 4)]
        except Exception:
            pass
        keep_state["pose"] = pose
        if pose:
            _last_pose.update(pose=pose, t=time.time())
        keep_state["server"] = self.count_publishers('/keepout_zones') > 0

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

def keepout_payload():
    d = keepout.load()
    return dict(keep_state, version=d["version"], zones=d["zones"], prev=d["prev"], robot_clear=ROBOT_KEEP_CLEAR)


def keepout_save(zones):
    """화면에서 온 구역 목록을 검사해 파일에 쓴다. 반환 (ok, 오류 문장)."""
    try:
        zones = keepout.clean_zones(zones)
    except ValueError as e:
        return False, str(e)
    d = keepout.load()
    old = {json.dumps(z["poly"]) for z in d["zones"]}
    # ⚠️ 10-10 시뮬 화면 시험: 부하가 큰 순간 자세가 잠깐 비어 이 검사를 건너뛰고 로봇 위에 구역이 만들어졌다 → 최근 10초의 자세를 쓴다
    pose = keep_state["pose"] or (_last_pose["pose"] if time.time() - _last_pose["t"] < 10.0 else None)
    if pose:
        for z in zones:      # 새로 그렸거나 모양을 바꾼 구역만 본다(이미 있던 구역 옆에 로봇이 서 있어도 다른 구역은 고칠 수 있게)
            if json.dumps(z["poly"]) not in old and keepout.dist(pose[0], pose[1], z["poly"]) < ROBOT_KEEP_CLEAR:
                return False, f"로봇이 그 구역 안이나 바로 옆({ROBOT_KEEP_CLEAR * 100:.0f} cm 안)에 있습니다"
    keepout.save(dict(d, zones=zones))
    return True, None


def keepout_restore():
    d = keepout.load()
    if not d["prev"]:
        return False, "보관된 구역이 없습니다"
    keepout.save(dict(d, zones=d["zones"] + d["prev"], prev=[]))
    return True, None


MAPS_DIR = os.environ.get("MAPS_DIR") or os.path.expanduser("~/maps")       # 시뮬·시험은 MAPS_DIR로 다른 폴더
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
                "mtime": os.path.getmtime(pgm), "resolution": res,
                # 저장된 지도에서 위치를 잡으려면 자세 그래프가 있어야 한다(10-10부터 함께 저장). 구역은 그 지도 좌표의 것
                "posegraph": os.path.isfile(os.path.join(MAPS_DIR, name + ".posegraph")),
                "zones": len(keepout.load(os.path.join(MAPS_DIR, name + ".keepout.json"))["zones"])}
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
    save_extras(name)
    return name, None


def save_extras(name):
    """그림(pgm) 말고 다음에 이 지도를 **다시 쓰는 데** 필요한 것: SLAM 자세 그래프(위치 잡기용)와 금지 구역(이 지도 좌표).
    ⚠️ 그림만 저장하면 다음에 그 지도 안에서 위치를 잡을 방법이 줄어든다(slam_toolbox는 자세 그래프가 있어야 이어서 쓴다).
    반환 dict(posegraph=성공 여부, zones=구역 수) — 화면에 그대로 알린다."""
    base = _safe_map_path(name, "")
    out = dict(posegraph=False, zones=0)
    if not base:
        return out
    try:
        out["posegraph"] = Handler.node.serialize(base)
    except Exception:
        pass
    try:
        z = keepout.load()["zones"]
        if keep_state["server"]:        # 구역은 지금 켜진 지도의 것일 때만 함께 묶는다
            keepout.save(dict(version=0, zones=z, prev=[]), base + ".keepout.json"); out["zones"] = len(z)
    except OSError:
        pass
    last_extras.update(out)
    return out


last_extras = {}


def delete_map(name):
    """맵 하나 삭제. pgm/yaml 쌍과 함께 저장한 자세 그래프·구역 파일을 지운다 — 같은 이름의 png/json은 안 건드린다."""
    removed = []
    for ext in (".pgm", ".yaml", ".posegraph", ".data", ".keepout.json"):
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
                cursor:pointer; white-space:nowrap; }
  .scrollx { overflow-x:auto; }
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
    max-width: 1100px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; display: flex; align-items: center; gap: 8px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  #mapWrap { position: relative; margin: 0 auto; width: fit-content; max-width: 100%; }
  #mapCanvas { display: block; background: #808080; image-rendering: pixelated; border: 1px solid var(--border); }
  #ovl { position: absolute; left: 0; top: 0; width: 100%; height: 100%; touch-action: pan-y; cursor: default; }
  #ovl.draw { cursor: crosshair; touch-action: none; } #ovl.sel { touch-action: none; }
  .zonebar { display: flex; flex-wrap: wrap; gap: .5rem; align-items: center; margin-bottom: .6rem; }
  .zonebar button { padding: .5rem .9rem; font-size: .95rem; border-radius: 8px; cursor: pointer; border: 1px solid var(--border);
    background: var(--surface-1); color: inherit; }
  .zonebar button.on { background: var(--critical); color: #fff; border-color: var(--critical); }
  .zonebar button:disabled { opacity: .45; cursor: not-allowed; }
  #zoneState { font-size: .9rem; color: var(--text-secondary); flex: 1 1 14rem; }
  #zoneState.ok { color: var(--good); font-weight: 600; } #zoneState.bad { color: var(--critical); font-weight: 600; }
  #zoneHint { font-size: .85rem; color: var(--text-secondary); margin: .5rem 0 0; line-height: 1.5; }
  #zoneList { margin-top: .6rem; } #zoneList td { white-space: nowrap; padding: .3rem .5rem .3rem 0; border-bottom: 1px solid var(--border); font-size: .9rem;
    font-variant-numeric: tabular-nums; }
  #zoneList tr.sel td { background: rgba(208,59,59,.10); }
  #zoneList input { width: 9rem; max-width: 34vw; padding: .25rem .4rem; border-radius: 6px; border: 1px solid var(--border); background: transparent; color: inherit; }
  #prevBox { margin-top: .7rem; padding: .6rem .75rem; border: 1px dashed var(--text-muted); border-radius: 8px; font-size: .88rem; line-height: 1.5; }
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
<a class="nav-home" href="#" onclick="location.port='8764'; return false;">← __HOME_ICON__시스템 대시보드 (홈)</a>
<h1><span class="dot" id="connDot" style="background:var(--critical)"></span> 맵 대시보드</h1>
<button id="estopBtn">🛑 비상정지 (자율주행 전체 중단)</button>
<div id="estopMsg"></div>
<div id="viewBanner" class="banner" style="display:none"></div>

<div class="card">
  <div class="zonebar" id="exploreBar" style="display:none">
    <button id="exploreBtn"></button>
    <span id="exploreState" class="msg"></span>
  </div>
  <div class="zonebar">
    <button id="drawBtn" disabled>금지 구역 그리기</button>
    <button id="delBtn" disabled>선택한 구역 지우기</button>
    <span id="zoneState"></span>
  </div>
  <div id="mapWrap"><canvas id="mapCanvas" width="10" height="10"></canvas><canvas id="ovl"></canvas></div>
  <p id="zoneHint"></p>
  <table id="zoneList"></table>
  <div id="prevBox" style="display:none"></div>
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
  <div class="scrollx"><table id="mapTable"></table></div>
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

// ---------------- 금지 구역 편집 (2026-10-10) ----------------
// 로봇청소기 앱처럼: [금지 구역 그리기]를 누르고 지도 위를 끌어 상자를 만든다. 상자를 누르면 선택 — 모서리를 끌어 모양을
// 고치고, 안쪽을 끌어 옮긴다. 손을 떼면 바로 저장되고, 구역 서버가 되돌려 준 version이 같아지면 "적용됨"으로 표시한다.
// 좌표는 전부 지도 좌표(m). 화면 좌표 ↔ 지도 좌표는 G(지금 그려진 격자의 크기·원점)로 바꾼다.
const ovl = document.getElementById('ovl'), octx = ovl.getContext('2d');
let G = null;                 // {w,h,res,ox,oy,live,stale}
let K = null;                 // /api/keepout 응답(파일의 구역 + 서버 상태 + 로봇 자세)
let zones = [], sel = null, mode = 'idle', drag = null, dirty = false, zoneErr = '', seq = 0;
const HANDLE = 14;            // 모서리 손잡이를 잡는 반경(px)

function fitMap() {           // 격자를 카드 폭과 화면 높이 70%에 맞춰 키우거나 줄인다(작은 지도도 크게, 긴 지도는 화면 안에)
  if (!G) return;
  const availW = document.getElementById('mapWrap').parentElement.clientWidth - 2;
  const sc = Math.min(availW / G.w, innerHeight * 0.7 / G.h, 12);
  canvas.style.width = (G.w * sc) + 'px'; canvas.style.height = (G.h * sc) + 'px';
  const dpr = devicePixelRatio || 1, W = Math.round(G.w * sc * dpr), H = Math.round(G.h * sc * dpr);
  if (ovl.width !== W || ovl.height !== H) { ovl.width = W; ovl.height = H; }
}
const scale = () => ovl.getBoundingClientRect().width / G.w;              // px / 칸
const toPx = (x, y) => [(x - G.ox) / G.res * scale(), (G.h - (y - G.oy) / G.res) * scale()];
function toMap(ev) { const r = ovl.getBoundingClientRect(), k = scale();
  return [G.ox + (ev.clientX - r.left) / k * G.res, G.oy + (G.h - (ev.clientY - r.top) / k) * G.res]; }
function inPoly(x, y, p) { let c = false; for (let i = 0, j = p.length - 1; i < p.length; j = i++) {
  if ((p[i][1] > y) !== (p[j][1] > y) && x < (p[j][0] - p[i][0]) * (y - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]) c = !c; } return c; }
function sizeText(p) { const xs = p.map(q => q[0]), ys = p.map(q => q[1]);
  return `${(Math.max(...xs) - Math.min(...xs)).toFixed(1)} × ${(Math.max(...ys) - Math.min(...ys)).toFixed(1)} m`; }
const canEdit = () => G && G.live && viewing === null && K && K.server;

function drawOverlay() {
  if (!G) return;
  const dpr = devicePixelRatio || 1; octx.setTransform(dpr, 0, 0, dpr, 0, 0); octx.clearRect(0, 0, ovl.width, ovl.height);
  if (!G.live || viewing !== null) return;                                // 저장된 지도는 좌표가 달라 구역·로봇을 겹치지 않는다
  const path = p => { octx.beginPath(); p.forEach((q, i) => { const [x, y] = toPx(q[0], q[1]); i ? octx.lineTo(x, y) : octx.moveTo(x, y); }); octx.closePath(); };
  for (const z of ((K && K.prev) || [])) {                                // 지난 지도의 구역: 흐린 점선(지금 지도와 맞는지 눈으로 보라고)
    path(z.poly); octx.setLineDash([6, 5]); octx.strokeStyle = 'rgba(82,81,78,.8)'; octx.lineWidth = 1.5; octx.stroke(); octx.setLineDash([]);
  }
  zones.forEach((z, i) => {
    path(z.poly); octx.fillStyle = i === sel ? 'rgba(208,59,59,.38)' : 'rgba(208,59,59,.24)'; octx.fill();
    octx.strokeStyle = '#d03b3b'; octx.lineWidth = i === sel ? 3 : 2; octx.stroke();
    const cx = z.poly.reduce((a, q) => a + q[0], 0) / z.poly.length, cy = z.poly.reduce((a, q) => a + q[1], 0) / z.poly.length, [px, py] = toPx(cx, cy);
    octx.fillStyle = '#7a1414'; octx.font = '600 13px system-ui, sans-serif'; octx.textAlign = 'center';
    octx.fillText(z.name || `구역 ${z.id}`, px, py + 4); octx.textAlign = 'start';
    if (i === sel) for (const q of z.poly) { const [x, y] = toPx(q[0], q[1]);
      octx.fillStyle = '#fff'; octx.strokeStyle = '#d03b3b'; octx.lineWidth = 2; octx.beginPath(); octx.rect(x - 6, y - 6, 12, 12); octx.fill(); octx.stroke(); }
  });
  if (drag && drag.type === 'new') { const [x0, y0] = toPx(drag.a[0], drag.a[1]), [x1, y1] = toPx(drag.b[0], drag.b[1]);
    octx.fillStyle = 'rgba(208,59,59,.25)'; octx.strokeStyle = '#d03b3b'; octx.lineWidth = 2; octx.setLineDash([5, 4]);
    octx.fillRect(x0, y0, x1 - x0, y1 - y0); octx.strokeRect(x0, y0, x1 - x0, y1 - y0); octx.setLineDash([]); }
  if (K && K.pose) {                                                      // 로봇: 실제 크기(길이 19 × 폭 21 cm), 너무 작으면 12 px로
    const [x, y] = toPx(K.pose[0], K.pose[1]), k = scale() / G.res, s = Math.max(1, 12 / (0.21 * k));
    octx.save(); octx.translate(x, y); octx.rotate(-K.pose[2]); octx.scale(s, s);
    octx.fillStyle = 'rgba(42,120,214,.85)'; octx.fillRect(-0.095 * k, -0.105 * k, 0.19 * k, 0.21 * k);
    octx.beginPath(); octx.moveTo(0.16 * k, 0); octx.lineTo(0.095 * k, -0.06 * k); octx.lineTo(0.095 * k, 0.06 * k); octx.closePath(); octx.fill();
    octx.restore();
  }
}

function renderZoneUI() {
  const st = document.getElementById('zoneState'), drawBtn = document.getElementById('drawBtn'), hint = document.getElementById('zoneHint');
  const ok = canEdit();
  drawBtn.disabled = !ok; drawBtn.classList.toggle('on', mode === 'draw'); drawBtn.textContent = mode === 'draw' ? '그리기 취소' : '금지 구역 그리기';
  document.getElementById('delBtn').disabled = !(ok && sel !== null);
  ovl.className = mode === 'draw' ? 'draw' : (sel !== null ? 'sel' : '');
  st.className = '';
  if (zoneErr) { st.textContent = zoneErr; st.className = 'bad'; }
  else if (!K) st.textContent = '';
  else if (viewing !== null) st.textContent = '저장된 지도를 보는 중에는 구역을 고칠 수 없습니다.';
  else if (!K.server) st.textContent = '자율 스택이 꺼져 있어 금지 구역을 쓸 수 없습니다. 스택을 켜면 지도가 새로 시작되고, 그 지도 위에 그립니다.';
  else if (K.applied_version !== K.version) st.textContent = '적용하는 중…';
  else if (!zones.length) st.textContent = '금지 구역이 없습니다. 로봇은 지도에서 빈 곳이면 어디든 갑니다.';
  else { st.textContent = `금지 구역 ${zones.length}개 적용됨`; st.className = 'ok'; }
  hint.textContent = !ok ? '' : mode === 'draw' ? '지도 위에서 끌어 상자를 그리세요. 소파·침대처럼 로봇이 밑으로 들어가면 안 되는 곳을 덮으면 됩니다.'
    : sel !== null ? '모서리를 끌면 모양이 바뀌고, 안쪽을 끌면 옮겨집니다. 구역 선은 벽과 같아서 로봇 몸이 선을 넘지 않습니다.'
    : (zones.length ? '구역을 누르면 고칠 수 있습니다.' : '');
  document.getElementById('zoneList').innerHTML = (G && G.live && viewing === null) ? zones.map((z, i) =>
    `<tr class="${i === sel ? 'sel' : ''}" data-i="${i}"><td>구역 ${z.id}</td><td><input data-name="${i}" value="${(z.name || '').replace(/"/g, '&quot;')}" placeholder="이름(예: 소파)" maxlength="20" ${ok ? '' : 'disabled'}></td>`
    + `<td>${sizeText(z.poly)}</td><td><button class="mini danger" data-zdel="${i}" ${ok ? '' : 'disabled'}>지우기</button></td></tr>`).join('') : '';
  const pb = document.getElementById('prevBox'), np = (K && K.prev || []).length;
  pb.style.display = (np && ok) ? 'block' : 'none';
  if (np && ok) pb.innerHTML = `지난 지도에서 쓰던 구역 ${np}개가 보관돼 있습니다(지도 위 회색 점선). 지도는 켤 때마다 로봇이 놓인 자리를 기준으로 새로 그려지므로, `
    + `<b>지난번과 같은 자리·같은 방향에서 시작했을 때만</b> 위치가 맞습니다. 점선이 가구와 맞는지 보고 정하세요. `
    + `<button class="mini" id="prevUse">다시 쓰기</button> <button class="mini" id="prevDrop">버리기</button>`;
}

async function loadKeepout() {
  // ⚠️ 저장 전에 떠난 조회가 저장 뒤에 도착하면 옛 목록으로 덮어써 방금 그린 구역의 선택이 풀렸다(10-10 화면 시험) → 그런 응답은 버린다
  const s0 = seq; let k;
  try { k = await (await fetch('/api/keepout')).json(); } catch (e) { return; }
  if (s0 !== seq || dirty) return;
  K = k;
  if (!drag && !dirty) { zones = K.zones.map(z => ({id: z.id, name: z.name, poly: z.poly.map(q => q.slice())})); if (sel !== null && sel >= zones.length) sel = null; }
  if (!canEdit() && mode === 'draw') mode = 'idle';
  renderZoneUI(); drawOverlay();
}
async function saveZones() {
  dirty = true; zoneErr = ''; seq++;
  try {
    const r = await (await fetch('/api/keepout', {method: 'POST', body: JSON.stringify({zones})})).json();
    K = r; if (!r.ok) { zoneErr = r.error || '저장 실패'; sel = null; setTimeout(() => { zoneErr = ''; renderZoneUI(); }, 6000); }
    zones = r.zones.map(z => ({id: z.id, name: z.name, poly: z.poly.map(q => q.slice())}));      // 실패하면 서버의 것으로 되돌아간다
  } catch (e) { zoneErr = '저장 요청 실패 — 연결을 확인하세요'; }
  dirty = false; seq++; renderZoneUI(); drawOverlay();
}
function nextId() { return zones.reduce((m, z) => Math.max(m, z.id), 0) + 1; }

ovl.addEventListener('pointerdown', ev => {
  if (!canEdit()) return;
  const p = toMap(ev), k = scale();
  if (mode === 'draw') { drag = {type: 'new', a: p, b: p}; }
  else {
    let hit = null;
    if (sel !== null) zones[sel].poly.forEach((q, vi) => { const [x, y] = toPx(q[0], q[1]), r = ovl.getBoundingClientRect();
      if (Math.hypot(ev.clientX - r.left - x, ev.clientY - r.top - y) <= HANDLE) hit = vi; });
    if (hit !== null) drag = {type: 'vertex', vi: hit, moved: false};
    else { let zi = null; zones.forEach((z, i) => { if (inPoly(p[0], p[1], z.poly)) zi = i; });
      sel = zi; drag = zi === null ? null : {type: 'move', last: p, moved: false}; }
  }
  if (drag) { ovl.setPointerCapture(ev.pointerId); ev.preventDefault(); }
  renderZoneUI(); drawOverlay();
});
ovl.addEventListener('pointermove', ev => {
  if (!drag) return;
  const p = toMap(ev);
  if (drag.type === 'new') drag.b = p;
  else if (drag.type === 'vertex') { zones[sel].poly[drag.vi] = [+p[0].toFixed(3), +p[1].toFixed(3)]; drag.moved = true; }
  else if (drag.type === 'move') { const dx = p[0] - drag.last[0], dy = p[1] - drag.last[1];
    if (Math.hypot(dx, dy) * scale() / G.res > 2) { zones[sel].poly = zones[sel].poly.map(q => [+(q[0] + dx).toFixed(3), +(q[1] + dy).toFixed(3)]); drag.last = p; drag.moved = true; } }
  drawOverlay();
});
function endDrag(ev) {
  if (!drag) return;
  const d = drag; drag = null;
  if (d.type === 'new') {
    const x0 = Math.min(d.a[0], d.b[0]), x1 = Math.max(d.a[0], d.b[0]), y0 = Math.min(d.a[1], d.b[1]), y1 = Math.max(d.a[1], d.b[1]);
    mode = 'idle';
    if (x1 - x0 >= 0.15 && y1 - y0 >= 0.15) {
      zones.push({id: nextId(), name: '', poly: [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(q => [+q[0].toFixed(3), +q[1].toFixed(3)])});
      sel = zones.length - 1; saveZones(); return;
    }
    zoneErr = '너무 작습니다. 15 cm보다 크게 끌어 주세요.'; setTimeout(() => { zoneErr = ''; renderZoneUI(); }, 4000);
  } else if (d.moved) { saveZones(); return; }
  renderZoneUI(); drawOverlay();
}
ovl.addEventListener('pointerup', endDrag); ovl.addEventListener('pointercancel', endDrag);
document.getElementById('drawBtn').onclick = () => { mode = mode === 'draw' ? 'idle' : 'draw'; sel = null; renderZoneUI(); drawOverlay(); };
function removeZone(i) { zones.splice(i, 1); sel = null; saveZones(); }
document.getElementById('delBtn').onclick = () => { if (sel !== null) removeZone(sel); };
document.addEventListener('keydown', ev => {
  if (ev.target.tagName === 'INPUT') return;
  if ((ev.key === 'Delete' || ev.key === 'Backspace') && sel !== null && canEdit()) { ev.preventDefault(); removeZone(sel); }
  if (ev.key === 'Escape') { mode = 'idle'; sel = null; drag = null; renderZoneUI(); drawOverlay(); }
});
document.getElementById('zoneList').addEventListener('click', ev => {
  const del = ev.target.getAttribute('data-zdel');
  if (del !== null) { removeZone(+del); return; }
  const tr = ev.target.closest('tr'); if (tr && ev.target.tagName !== 'INPUT') { sel = +tr.dataset.i; renderZoneUI(); drawOverlay(); }
});
document.getElementById('zoneList').addEventListener('change', ev => {
  const i = ev.target.getAttribute('data-name'); if (i !== null) { zones[+i].name = ev.target.value.trim(); saveZones(); }
});
document.getElementById('prevBox').addEventListener('click', async ev => {
  const url = ev.target.id === 'prevUse' ? '/api/keepout/restore' : ev.target.id === 'prevDrop' ? '/api/keepout/drop_prev' : null;
  if (!url) return;
  try { K = await (await fetch(url, {method: 'POST', body: '{}'})).json(); } catch (e) { return; }
  dirty = false; loadKeepout();
});
// ---------------- 탐색 시작·멈춤 ----------------
// 탐색기는 켜질 때 멈춰 있다(런치 explore_autostart 기본 false) — 지도를 보고 금지 구역을 먼저 그린 뒤 시작한다.
// 멈춤은 비상정지가 아니다: 지금 걸음(최대 0.4 m)을 마치고 선다. 급하면 위의 빨간 비상정지.
let E = null;
function renderExplore() {
  const bar = document.getElementById('exploreBar'), btn = document.getElementById('exploreBtn'), st = document.getElementById('exploreState');
  bar.style.display = (E && E.available) ? 'flex' : 'none';
  if (!E || !E.available) return;
  const running = E.state === 1 || E.state === 2;
  btn.textContent = running ? '탐색 멈춤' : '탐색 시작'; btn.classList.toggle('on', running); btn.dataset.act = running ? 'stop' : 'start';
  st.textContent = running ? '탐색 중입니다. 멈춤을 누르면 지금 걸음을 마치고 섭니다(급하면 비상정지).'
    : (E.state === 4 ? '멈추는 중…' : '탐색이 멈춰 있습니다. 금지 구역을 먼저 그리고 시작하세요. 시작하면 로봇이 스스로 움직입니다.');
}
async function loadExplore() { try { E = await (await fetch('/api/explore')).json(); } catch (e) { return; } renderExplore(); }
document.getElementById('exploreBtn').onclick = async ev => {
  const act = ev.target.dataset.act;
  if (act === 'start' && !confirm(`탐색을 시작합니다. 로봇이 스스로 움직입니다. 로봇 주변이 비어 있고 금지 구역을 그렸는지 확인하세요.`)) return;
  try { E = await (await fetch('/api/explore/' + act, {method: 'POST', body: '{}'})).json(); } catch (e) { return; }
  renderExplore(); setTimeout(loadExplore, 1000);
};
setInterval(() => { if (!document.hidden) loadExplore(); }, 2000); loadExplore();
addEventListener('resize', () => { fitMap(); drawOverlay(); });
setInterval(() => { if (!document.hidden && !(document.activeElement && document.activeElement.hasAttribute('data-name'))) loadKeepout(); }, 1000);
loadKeepout();

// 격자 그리기. **라이브 맵과 저장된 맵이 같은 형식**이라 이 함수를 공유한다.
function renderGrid(d) {
  const bin = atob(d.data_b64);
  const W0 = d.width, H0 = d.height, res = d.resolution;
  // 아는 칸(벽·빈 곳)이 있는 범위만 잘라서 크게 보여 준다 — SLAM 격자는 모르는 여백이 넓어서, 통째로 그리면 정작 볼 곳이
  // 손톱만 해진다(10-10 실물: 도크에서 켠 직후 지도가 화면의 1/10). 구역과 로봇도 범위에 넣고 사방 1 m 여유를 둔다.
  let c0 = W0, c1 = -1, r0 = H0, r1 = -1;
  for (let i = 0; i < W0 * H0; i++) {
    if (bin.charCodeAt(i) > 100) continue;                 // 255 = 모름
    const row = (i / W0) | 0, col = i % W0;
    if (col < c0) c0 = col; if (col > c1) c1 = col; if (row < r0) r0 = row; if (row > r1) r1 = row;
  }
  if (c1 < 0) { c0 = 0; c1 = W0 - 1; r0 = 0; r1 = H0 - 1; }
  const ext = (x, y) => { const c = Math.floor((x - d.origin_x) / res), r = Math.floor((y - d.origin_y) / res);
    c0 = Math.min(c0, c); c1 = Math.max(c1, c); r0 = Math.min(r0, r); r1 = Math.max(r1, r); };
  if (!d.saved) { zones.forEach(z => z.poly.forEach(q => ext(q[0], q[1]))); if (K && K.pose) ext(K.pose[0], K.pose[1]); }
  const pad = Math.round(1.0 / res);
  c0 = Math.max(0, c0 - pad); r0 = Math.max(0, r0 - pad); c1 = Math.min(W0 - 1, c1 + pad); r1 = Math.min(H0 - 1, r1 + pad);
  const w = c1 - c0 + 1, h = r1 - r0 + 1;
  canvas.width = w;
  canvas.height = h;
  const imgData = ctx.createImageData(w, h);
  for (let row = r0; row <= r1; row++) for (let col = c0; col <= c1; col++) {
    const raw = bin.charCodeAt(row * W0 + col);            // 0~255 (unsigned로 옴)
    const g = occColor(raw > 100 ? -1 : raw);              // 255였던 -1 복원
    // OccupancyGrid는 (0,0)이 좌하단부터 row-major라 캔버스(좌상단 시작)로 옮길 때 y 뒤집기
    const k = ((h - 1 - (row - r0)) * w + (col - c0)) * 4;
    imgData.data[k] = g; imgData.data[k + 1] = g; imgData.data[k + 2] = g; imgData.data[k + 3] = 255;
  }
  ctx.putImageData(imgData, 0, 0);
  G = {w, h, res, ox: d.origin_x + c0 * res, oy: d.origin_y + r0 * res, live: !d.saved, stale: !!d.stale};
  fitMap(); drawOverlay();

  document.getElementById('dimText').textContent =
    `지도 격자 ${W0}×${H0} 칸(한 칸 ${(res*100).toFixed(0)} cm) 중 아는 범위 약 ${(w*res).toFixed(1)} × ${(h*res).toFixed(1)} m를 보여 줍니다`;
  document.getElementById('updatedText').textContent =
    (d.saved ? '저장 시각: ' : '마지막 갱신: ')
    + new Date(d.last_ts * 1000).toLocaleTimeString()
    + (d.stale ? '  (갱신 멈춤 — SLAM 정지)' : '');
}

async function tick() {
  // 저장된 맵을 보는 중이면 라이브로 덮어쓰지 않는다.
  if (viewing !== null) return;
  if (drag) return;            // 끌고 있는 동안에는 지도를 다시 그리지 않는다(보이는 범위가 바뀌면 손 밑의 자리가 움직인다)
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
      <td>${m.posegraph ? '위치 잡기 가능' : '그림만'}${m.zones ? ` · 구역 ${m.zones}` : ''}</td>
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
    document.getElementById('mapWrap').style.display = 'none'; G = null;
    return;
  }
  document.getElementById('emptyMsg').style.display = 'none';
  document.getElementById('mapWrap').style.display = 'block';
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
  msg.textContent = r.ok ? `저장됨: ${r.name}` + (r.extras ? ` (위치 잡기용 자료 ${r.extras.posegraph ? '포함' : '없음 — SLAM이 꺼져 있음'}, 금지 구역 ${r.extras.zones}개)` : '')
    : (r.error || '저장 실패');
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

# 파비콘 및 홈 아이콘 삽입. PAGE가 일반 문자열이라 f-string 치환을 못 쓰므로 여기서 바꾼다.
PAGE = PAGE.replace("__FAVICON__", FAVICON_TAG).replace("__HOME_ICON__", HOME_ICON_TAG)



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
        elif self.path == "/api/keepout":
            self._send(200, json.dumps(keepout_payload(), ensure_ascii=False).encode("utf-8"), "application/json")
        elif self.path == "/api/explore":
            self._send(200, json.dumps(Handler.node.explore(), ensure_ascii=False).encode("utf-8"), "application/json")
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
        elif self.path in ("/api/explore/start", "/api/explore/stop"):
            self._send(200, json.dumps(Handler.node.explore(self.path.rsplit("/", 1)[1]), ensure_ascii=False).encode("utf-8"), "application/json")
        elif self.path in ("/api/keepout", "/api/keepout/restore", "/api/keepout/drop_prev"):
            try:
                ln = int(self.headers.get("Content-Length", "0") or 0)
                body = json.loads(self.rfile.read(ln) or b"{}")
            except (ValueError, OSError):
                body = {}
            if self.path == "/api/keepout":
                ok, err = keepout_save(body.get("zones"))
            elif self.path.endswith("/restore"):
                ok, err = keepout_restore()
            else:
                keepout.save(dict(keepout.load(), prev=[])); ok, err = True, None
            self._send(200 if ok else 400, json.dumps(dict(keepout_payload(), ok=ok, error=err), ensure_ascii=False).encode("utf-8"),
                       "application/json")
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
                           json.dumps({"ok": bool(saved), "name": saved, "extras": dict(last_extras) if saved else None,
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
    args, ros_args = ap.parse_known_args()      # 나머지(--ros-args -p use_sim_time:=true 등)는 rclpy로 — 시뮬 도메인에서 시험할 때

    rclpy.init(args=ros_args or None)
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
