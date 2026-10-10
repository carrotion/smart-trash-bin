#!/usr/bin/env python3
"""라이다에 **새로 가려진 방향**이 있는지 검사 — 시험 전 자동 점검용.

⚠️ 2026-10-09: 라이다에 종이테이프 2곳이 붙은 채 4층 합격 시험을 했다(사용자가 10-10에 발견). 녹화로 보니 왼쪽 옆
74~100°(약 26°)가 스캔의 50~70%에서 12 cm 안에 막혀 있었다 — 그 점들은 차체 안이라 자기 필터가 지우므로 로봇은 그쪽을
못 본 채 움직였고, 아무 경고도 없었다. 9/27 녹화와 테이프를 뗀 뒤(10-10)에는 0%.

방법: 원시 스캔(/scan_raw, 스택이 없으면 서비스의 /scan)에서 2° 칸마다 "12 cm 안에서 돌아온 비율"을 구해,
기준(테이프 없는 9/27 녹화 + 10-10 측정에서 가까운 반사가 있던 칸 = 케이블 등 자기 구조물, ±6° 여유: 케이블이 날마다
몇 도씩 움직인다) 밖에서 30% 넘는 칸을 "새로 가려짐"으로 본다. 합이 6° 넘으면 ❌.

    python3 tools/lidar_blockage.py                 # 지금 스캔 5초 검사
    python3 tools/lidar_blockage.py --bag bags/20261009_124309_stepnav_obstacle   # 녹화 검사
    python3 tools/lidar_blockage.py --save-baseline # 지금 상태를 기준에 더함(가린 것이 없다고 확인했을 때만)
"""
import argparse, json, math, os, sys, time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(REPO, 'docs/calibration/lidar_self_baseline.json')
BIN, NEAR, FRAC, TOL_DEG, MAX_NEW_DEG = 2, 0.12, 0.30, 6, 6


def near_fraction(scans):
    """스캔 목록 → 2° 칸별 12 cm 안 반사 비율(길이 180 목록)."""
    n = 360 // BIN; tot = [0] * n; near = [0] * n
    for m in scans:
        for i, r in enumerate(m.ranges):
            k = int((math.degrees(m.angle_min + i * m.angle_increment) % 360) // BIN) % n
            tot[k] += 1
            if m.range_min <= r < NEAR:
                near[k] += 1
    return [near[k] / tot[k] if tot[k] else 0.0 for k in range(n)]


def load_baseline():
    return set(json.load(open(BASE))['bins_deg']) if os.path.exists(BASE) else set()


def new_blocked(frac, baseline):
    """기준 밖에서 가려진 칸(도) 목록."""
    out = []
    for k, f in enumerate(frac):
        d = k * BIN
        if f >= FRAC and not any(min((d - b) % 360, (b - d) % 360) <= TOL_DEG for b in baseline):
            out.append((d, round(f, 2)))
    return out


def check(scans, quiet=False):
    """(문제 없음 여부, 설명). 스캔이 20장 미만이면 판정 보류(True)."""
    if len(scans) < 20:
        return True, f'스캔 {len(scans)}장 — 라이다 가림 판정 보류'
    nb = new_blocked(near_fraction(scans), load_baseline())
    width = len(nb) * BIN
    if width > MAX_NEW_DEG:
        return False, (f'라이다에 새로 가려진 방향 {width}°: ' + ', '.join(f'{d}°({int(f * 100)}%)' for d, f in nb[:12])
                       + ' — 라이다에 붙은 것·늘어진 케이블 확인(0°=정면, 90°=왼쪽)')
    return True, f'라이다 새 가림 없음(기준 밖 가까운 반사 {width}°)'


def collect_live(topic, sec=5.0):
    import rclpy
    from sensor_msgs.msg import LaserScan
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = rclpy.create_node('lidar_blockage'); L = []
    n.create_subscription(LaserScan, topic, L.append, 20); t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    if own:
        rclpy.shutdown()
    return L


def collect_bag(path, topic, limit=3000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic])); L = []
    while r.has_next() and len(L) < limit:
        L.append(deserialize_message(r.read_next()[1], LaserScan))
    return L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default=None, help='기본: /scan_raw가 있으면 그것, 없으면 /scan')
    ap.add_argument('--bag'); ap.add_argument('--sec', type=float, default=5.0)
    ap.add_argument('--save-baseline', action='store_true')
    a = ap.parse_args()
    if a.bag:
        scans = collect_bag(a.bag, a.topic or '/scan_raw')
    else:
        topic = a.topic
        if topic is None:
            import subprocess
            topics = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout.split()
            topic = '/scan_raw' if '/scan_raw' in topics else '/scan'
        scans = collect_live(topic, a.sec)
    frac = near_fraction(scans)
    if a.save_baseline:
        bins = sorted(load_baseline() | {k * BIN for k, f in enumerate(frac) if f >= FRAC})
        json.dump(dict(note='가까운(<12 cm) 반사가 있던 2° 칸 = 자기 구조물. 테이프 없는 상태에서만 더할 것', bins_deg=bins,
                       updated=time.strftime('%Y-%m-%d %H:%M')), open(BASE, 'w'), ensure_ascii=False, indent=1)
        print(f'기준 저장: {len(bins)}칸 {bins}')
        return 0
    ok, msg = check(scans)
    print(('✅ ' if ok else '❌ ') + msg + f' (스캔 {len(scans)}장)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
