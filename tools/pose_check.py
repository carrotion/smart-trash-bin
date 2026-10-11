#!/usr/bin/env python3
"""지금 스캔이 지도와 맞는가 — "지도 위 내 자리를 믿어도 되는가"를 움직이기 전에 묻는다 (2026-10-11).

왜: 10-11 저장 지도로 시작한 뒤 사람이 로봇을 58 cm 들어 옮겼는데, 위치 잡기는 옮기기 전 자리를 그대로 냈다(방향은 IMU로
따라가고 위치는 그대로, 경고 없음). 그대로 목표를 받으면 틀린 자리 기준으로 걷고, 금지 구역도 틀린 자리에 놓인다.
사람이 스캔 추정과 비교해 알아챘다 — 사람이 안 봐도 걸려야 한다.

방법: 믿고 있는 자세에서 스캔 점을 지도에 놓아 본다. 점마다
  - on      : 지도의 벽 10 cm 안에 떨어짐(맞음)
  - free    : 지도는 빈 칸이라는데 뭔가 있음(새 물체·사람이거나, 자리가 틀렸거나)
  - unk     : 지도가 모르는 칸(판단 불가 — 탐색 중 새 구역을 볼 때는 정상)
  - through : 라이다에서 그 점까지 가는 길에 지도의 벽이 있음(벽 너머를 봄). 재 보니 맞는 자세에서도 중앙 10%·95% 분위 18~30%로
              구분이 안 돼 **판정에 쓰지 않는다**(녹화 측정 때만 계산)
판정: 아는 칸에 떨어진 점(on + free) 중 on의 비율이 RATIO_MIN 미만이면 "안 맞음". 아는 칸의 점이 KNOWN_MIN개 미만이면 판단하지 않는다.
선은 녹화로 쟀다(--bag, 10-11 첫 실물 탐색의 서 있는 순간 1,511개 — 맞는 자세와 일부러 어긋나게 한 자세의 분포):
    선 0.80: 맞는 자세 헛경보 0.1%(저장 지도)·0.3%(탐색 중 지도), 10 cm 어긋남 2%, 30 cm 83%, 58 cm 85~98%, 1 m 100%, 20° 100%, 10° 49%
    선 0.85면 10 cm 어긋남을 11% 거절(위치 잡기의 평소 잔차가 방에 따라 10 cm까지 나온다 → 일을 못 한다), 0.90이면 17%
    ⚠️ 긴 복도에서 복도 방향으로 옮겨진 것은 벽만으로는 못 가린다(58 cm 한 방향이 85%에서 멈추는 이유)

    python3 tools/pose_check.py --bag bags/20261011_155553_explore1 --map ~/maps/home_20261011     # 분포 측정
    python3 tools/pose_check.py --live            # 지금 켜진 스택에서 한 번 재 본다(움직이지 않음)
"""
import argparse, collections, math, os, sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from locate_on_map import dist_field, load_map

NEAR = 0.10        # 벽에 "떨어졌다"고 보는 거리(m) — locate_on_map의 on_wall과 같은 기준
BACK = 0.30        # 벽 너머 판정에서 점 바로 앞 이만큼은 보지 않는다(그 점이 속한 벽 자체, 칸 크기·자세 오차 몫)
LIDAR_X = 0.054


class Field:
    """지도 한 장: 벽 칸, 모르는 칸, 벽까지 거리장. 행 0 = 지도 아래쪽(y 작은 쪽)."""
    def __init__(self, occ, unk, res, ox, oy):
        self.occ, self.unk, self.res, self.ox, self.oy = occ, unk, res, ox, oy
        self.h, self.w = occ.shape; self.D = dist_field(occ, res, rmax=0.3)

    @classmethod
    def from_msg(cls, m):
        g = np.frombuffer(bytes(m.data), np.int8).reshape(m.info.height, m.info.width) if not isinstance(m.data, np.ndarray) \
            else np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width)
        return cls(g >= 65, g < 0, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)

    @classmethod
    def from_saved(cls, base):
        img, res, ox, oy = load_map(os.path.expanduser(base))
        return cls(img < 50, img == 205, res, ox, oy)

    def cells(self, x, y):
        i = np.floor((x - self.ox) / self.res).astype(np.int32); j = np.floor((y - self.oy) / self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        return np.clip(i, 0, self.w - 1), np.clip(j, 0, self.h - 1), ok


def scan_points(scan, lidar_x=LIDAR_X, rmin=0.25, rmax=8.0):
    """LaserScan → 로봇 기준 점(N×2). 라이다에서 rmin~rmax 사이만(차체·부착물, 먼 잡음 제외)."""
    r = np.asarray(scan.ranges, np.float32); a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r >= max(rmin, scan.range_min)) & (r <= min(rmax, scan.range_max))
    return np.stack([lidar_x + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)


def check(F, pose, pts, lidar_x=LIDAR_X, rays=False):
    """자세 pose=(x, y, yaw)에서 로봇 기준 점 pts를 지도 F에 놓아 본 결과(개수와 비율). rays=True면 벽 너머(through)도 센다."""
    n = len(pts)
    if n == 0:
        return dict(n=0, on=0, free=0, unk=0, through=0, on_frac=0.0, free_frac=0.0, through_frac=0.0, ratio=None)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    px = pose[0] + c * pts[:, 0] - s * pts[:, 1]; py = pose[1] + s * pts[:, 0] + c * pts[:, 1]
    lx, ly = pose[0] + c * lidar_x, pose[1] + s * lidar_x
    i, j, ok = F.cells(px, py)
    on = ok & (F.D[j, i] <= NEAR); unk = ~on & (~ok | F.unk[j, i]); free = ~on & ~unk
    # 벽 너머: 라이다 → 점 선분을 칸 크기 간격으로 훑어, 점 앞 BACK보다 먼 곳에 벽 칸이 있으면 그 점은 벽을 뚫고 본 것
    L = np.hypot(px - lx, py - ly); ux, uy = (px - lx) / np.maximum(L, 1e-6), (py - ly) / np.maximum(L, 1e-6)
    ss = np.arange(0.15, float(L.max()), F.res, dtype=np.float32) if rays else ()
    through = np.zeros(n, bool)
    if len(ss):
        X = lx + ux[:, None] * ss[None, :]; Y = ly + uy[:, None] * ss[None, :]
        ii, jj, okk = F.cells(X, Y)
        through = (okk & F.occ[jj, ii] & (ss[None, :] < (L - BACK)[:, None])).any(1)
    r = dict(n=n, on=int(on.sum()), free=int(free.sum()), unk=int(unk.sum()), through=int(through.sum()))
    r.update(on_frac=r['on'] / n, free_frac=r['free'] / n, through_frac=r['through'] / n,
             ratio=(r['on'] / (r['on'] + r['free']) if r['on'] + r['free'] else None))
    return r


RATIO_MIN = 0.80       # 근거는 파일 머리말
KNOWN_MIN = 40


def verdict(r):
    """(믿어도 되는가, 이유). 아는 칸에 떨어진 점이 너무 적으면 판단하지 않는다(True)."""
    if r['on'] + r['free'] < KNOWN_MIN:
        return True, ''
    if r['ratio'] < RATIO_MIN:
        return False, f"아는 칸에 떨어진 스캔 점 {r['on'] + r['free']}개 중 지도의 벽 위는 {r['ratio'] * 100:.0f}%뿐(기준 {RATIO_MIN * 100:.0f}%)"
    return True, ''


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


PERTURB = [('맞는 자세', 0, 0, 0), ('5 cm·2°', 0.05, 0.0, 2), ('10 cm', 0.0, 0.10, 0), ('20 cm', 0.20, 0.0, 0), ('30 cm', 0.0, 0.30, 0), ('58 cm(10-11 사례)', 0.0, 0.58, 0),
           ('58 cm 다른 방향', 0.58, 0.0, 0), ('1 m', 0.71, 0.71, 0), ('2 m', -1.41, 1.41, 0), ('10°', 0, 0, 10), ('20°', 0, 0, 20), ('45°', 0, 0, 45), ('180°', 0, 0, 180)]


def bag_stats(bag, saved, every=10, out=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    meta = open(os.path.join(bag, 'metadata.yaml')).read()
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap' if 'storage_identifier: mcap' in meta else 'sqlite3'), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/scan', '/map']))
    Ffin = Field.from_saved(saved) if saved else None
    mo = None; hist = collections.deque(maxlen=400); live_msg = None; live = None; k = 0
    rows = []       # (t, 지도 종류, 어긋남 이름, on_frac, free_frac, through_frac, unk_frac, n)
    while rd.has_next():
        topic, data, tns = rd.read_next(); t = tns * 1e-9
        if topic == '/tf':
            for tr in deserialize_message(data, TFMessage).transforms:
                v = (tr.transform.translation.x, tr.transform.translation.y, _yaw(tr.transform.rotation))
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    mo = v
                elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                    hist.append((t,) + v)
        elif topic == '/map':
            live_msg = deserialize_message(data, OccupancyGrid); live = None
        elif topic == '/scan':
            k += 1
            if k % every or mo is None or live_msg is None or len(hist) < 20:
                continue
            now = hist[-1]; old = next((h for h in hist if now[0] - h[0] <= 0.6), None)
            if now[0] - old[0] < 0.4 or math.hypot(now[1] - old[1], now[2] - old[2]) > 0.003 or abs(now[3] - old[3]) > math.radians(0.3):
                continue                      # 서 있을 때만(주행기는 서서 판단한다)
            pts = scan_points(deserialize_message(data, LaserScan))
            if len(pts) < 40:
                continue
            if live is None:
                live = Field.from_msg(live_msg)
            pose = _compose(mo, now[1:])
            for kind, F in (('탐색 중 지도', live), ('저장 지도', Ffin)):
                if F is None:
                    continue
                for name, dx, dy, dd in PERTURB:
                    r = check(F, (pose[0] + dx, pose[1] + dy, pose[2] + math.radians(dd)), pts, rays=True)
                    rows.append((t, kind, name, r['on_frac'], r['free_frac'], r['through_frac'], r['unk'] / r['n'], r['n']))
    if out:
        import csv
        with open(out, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['t', 'map', 'perturb', 'on_frac', 'free_frac', 'through_frac', 'unk_frac', 'n']); w.writerows(rows)
    return rows


def show(rows):
    def pc(v, p):
        v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))] * 100
    for kind in ('탐색 중 지도', '저장 지도'):
        sel = [r for r in rows if r[1] == kind]
        if not sel:
            continue
        print(f'== {kind}: 서 있는 순간 {len(sel) // len(PERTURB)}개 (각 값은 5% / 중앙 / 95% 분위, %)')
        print(f'  {"어긋남":<18} {"벽 위(on)":>20} {"빈 칸(free)":>20} {"벽 너머(through)":>20} {"모름(unk)":>20}')
        for name, *_ in PERTURB:
            s = [r for r in sel if r[2] == name]
            print(f'  {name:<18} ' + ' '.join(f'{pc([r[c] for r in s], 0.05):5.0f} /{pc([r[c] for r in s], 0.5):4.0f} /{pc([r[c] for r in s], 0.95):4.0f}  ' for c in (3, 4, 5, 6)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag'); ap.add_argument('--map', help='저장된 지도 이름(확장자 없이) — 위치 잡기 방식의 분포도 본다')
    ap.add_argument('--every', type=int, default=10); ap.add_argument('--out')
    ap.add_argument('--live', action='store_true')
    a = ap.parse_args()
    if a.bag:
        show(bag_stats(a.bag, a.map, a.every, a.out)); return 0
    if a.live:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        import time
        rclpy.init(); n = rclpy.create_node('pose_check'); got = {}
        n.create_subscription(OccupancyGrid, '/map', lambda m: got.update(map=m), QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(LaserScan, '/scan', lambda m: got.update(scan=m), qos_profile_sensor_data)
        tfb = Buffer(); TransformListener(tfb, n); t0 = time.time(); pose = None
        while time.time() - t0 < 15 and (len(got) < 2 or pose is None):
            rclpy.spin_once(n, timeout_sec=0.1)
            try:
                tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.0)); pose = (tr.transform.translation.x, tr.transform.translation.y, _yaw(tr.transform.rotation))
            except Exception:
                pass
        rclpy.shutdown()
        if len(got) < 2 or pose is None:
            print('❌ 지도·스캔·자세 중 못 받은 것이 있다:', sorted(got), pose); return 2
        r = check(Field.from_msg(got['map']), pose, scan_points(got['scan'])); ok, why = verdict(r)
        print(f"자세 ({pose[0]:+.2f}, {pose[1]:+.2f}) m, {math.degrees(pose[2]):+.1f}° | 점 {r['n']}개: 벽 위 {r['on']}, 빈 칸 {r['free']}, 모름 {r['unk']}"
              + (f" → 아는 칸 중 벽 위 {r['ratio'] * 100:.0f}%" if r['ratio'] is not None else '') + f" → {'✅ 믿을 만함' if ok else '❌ ' + why}")
        return 0 if ok else 1
    ap.print_help(); return 2


if __name__ == '__main__':
    sys.exit(main())
