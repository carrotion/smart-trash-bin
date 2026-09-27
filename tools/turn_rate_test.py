#!/usr/bin/env python3
"""1층 기본 동작 ③ — 정해진 속도로 제자리 회전 (docs/motion_foundation.md 합격 기준).

w = 0.3, 0.5, 0.8 rad/s × 좌/우 각각 3초 명령, 자이로로 판정:
  정속 평균 오차 ≤ 10% (1~3초 구간) / 최대 ≤ 1.3 × 명령 / 역회전(명령 반대 −0.1 rad/s 초과) 0 / 90% 도달 ≤ 0.7 s
안전: |자이로| > 2.0 rad/s가 0.2초 넘으면 즉시 정지·중단. 주변 0.4 m 여유 검사. ⚠️ 바닥 전용.

    python3 tools/turn_rate_test.py [--ws 0.3 0.5 0.8] [--label 이름]
끝에 합격/불합격 표를 찍고 docs/calibration/turn_rate_*.json에 저장. 종료 코드 0 = 전부 합격.
"""
import argparse, json, math, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rotation_quality_test import scan_points

CRIT = dict(mean_err=0.10, peak_ratio=1.3, reversal=0, rise_s=0.7)


def judge(w, samples, dur):
    """samples: [(t_from_start, gz)]. 반환 지표 dict."""
    s = [(t, g * math.copysign(1, w)) for t, g in samples]           # 명령 방향을 +로
    steady = [g for t, g in s if 1.0 <= t <= dur]
    mean = sum(steady) / len(steady) if steady else 0.0
    peak = max((g for t, g in s if t <= dur), default=0.0)
    rev = sum(1 for t, g in s if 0.2 <= t <= dur and g < -0.1)
    rise = next((t for t, g in s if g >= 0.9 * abs(w)), float('inf'))
    m = dict(w=w, mean=mean, mean_err=abs(mean - abs(w)) / abs(w), peak=peak, peak_ratio=peak / abs(w),
             reversal=rev, rise_s=rise)
    m['pass'] = {k: (m[k] <= CRIT[k]) for k in CRIT}
    m['ok'] = all(m['pass'].values())
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ws', type=float, nargs='+', default=[0.3, 0.5, 0.8])
    ap.add_argument('--dur', type=float, default=3.0)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('turn_rate_test')
    S = {'g': [], 'gz': 0.0, 'scan': None}

    def on_imu(m):
        S['g'].append((time.time(), m.angular_velocity.z)); S['gz'] = m.angular_velocity.z
    n.create_subscription(Imu, '/imu/data', on_imu, 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)

    def stop():
        for _ in range(8):
            pub.publish(Twist()); spin(0.02)

    t0 = time.time()
    while (S['scan'] is None or len(S['g']) < 20) and time.time() - t0 < 10:
        spin(0.1)
    rows = []; aborted = False
    try:
        for w0 in a.ws:
            for sgn in (1, -1):
                w = sgn * w0
                near = min((math.hypot(x, y) for x, y in scan_points(S['scan'])), default=9)
                if near < 0.4:
                    print(f'❌ 주변 여유 {near:.2f} m < 0.4 — 중단'); aborted = True; return 1
                spin(1.5)
                S['g'] = []; ts = time.time(); over = None
                while time.time() - ts < a.dur:
                    c = Twist(); c.angular.z = float(w); pub.publish(c); spin(0.05)
                    if abs(S['gz']) > 2.0:
                        over = over or time.time()
                        if time.time() - over > 0.2:
                            stop(); print(f'❌ 안전 중단: |자이로| {S["gz"]:+.2f} rad/s'); aborted = True; return 1
                    else:
                        over = None
                stop()
                m = judge(w, [(t - ts, g) for t, g in S['g']], a.dur)
                rows.append(m)
                p = m['pass']
                print(f'w {w:+.2f}: 정속 {m["mean"]:.2f}(오차 {m["mean_err"]*100:4.0f}% {"✅" if p["mean_err"] else "❌"}) '
                      f'최대 {m["peak"]:.2f}(×{m["peak_ratio"]:.2f} {"✅" if p["peak_ratio"] else "❌"}) '
                      f'역회전 {m["reversal"]}{"✅" if p["reversal"] else "❌"} '
                      f'90%도달 {m["rise_s"]:.2f}s{"✅" if p["rise_s"] else "❌"}', flush=True)
    finally:
        stop()
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'turn_rate_{time.strftime("%Y%m%d_%H%M")}{("_" + a.label) if a.label else ""}.json')
        json.dump(dict(criteria=CRIT, rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1,
                  default=lambda x: None if x == float('inf') else x)
        ok = rows and all(r['ok'] for r in rows) and not aborted
        print(f'③ 제자리 회전: {"✅ 합격" if ok else "❌ 불합격"} ({sum(r["ok"] for r in rows)}/{len(rows)}) | 저장 {out}')
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
