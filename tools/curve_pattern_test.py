#!/usr/bin/env python3
"""Nav2식 곡선 명령 패턴에서 구동 제어가 발진하지 않는지 확인(09-27 실물 Nav2 첫 주행 발진 사고 재현 시험).

패턴(사고 직전 명령을 본뜸): 전진 v로 가며 w를 +W → 0 → −W → 0 교대, 이어서 후진으로 같은 패턴(제자리 근처 유지),
마지막에 DWB처럼 짧은 제자리 회전(±0.29)과 저속 전진(0.054)을 번갈아 준다.
판정: 단계별 자이로 최대 |w|. 명령 |w| ≤ 0.45이므로 실제가 1 rad/s를 넘으면 불합격.
안전: |자이로| > 1.5 rad/s가 0.2초 넘으면 즉시 정지·중단(구동 노드의 폭주 감지와 별개). 앞뒤 여유 검사, 첫 동작 뒤 도크 검사.
⚠️ 바닥 전용.

    python3 tools/curve_pattern_test.py --v 0.12 --w 0.39
"""
import argparse, json, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance
from dock_guard import looks_on_dock
from wheel_config import wheel_distances


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.12)
    ap.add_argument('--w', type=float, default=0.39)
    ap.add_argument('--reps', type=int, default=2)
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('curve_pattern_test')
    S = {'gz': 0.0, 'g': [], 'scan': None, 'enc': None}

    def on_imu(m):
        S['gz'] = m.angular_velocity.z; S['g'].append((time.time(), m.angular_velocity.z))
    n.create_subscription(Imu, '/imu/data', on_imu, 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S.update(enc=list(m.data)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def stop():
        for _ in range(8):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)

    over = {'since': None}

    def run(v, w, dur):
        """dur초 동안 (v,w)를 20 Hz로. 반환 (단계 최대 |자이로|, 안전 중단 여부)."""
        S['g'] = []; ts = time.time()
        while time.time() - ts < dur:
            c = Twist(); c.linear.x = v; c.angular.z = w; pub.publish(c)
            e = time.time() + 0.05
            while time.time() < e:
                rclpy.spin_once(n, timeout_sec=0.005)
            if abs(S['gz']) > 1.5:
                over['since'] = over['since'] or time.time()
                if time.time() - over['since'] > 0.2:
                    stop(); return max((abs(x) for _, x in S['g']), default=0), True
            else:
                over['since'] = None
        return max((abs(x) for _, x in S['g']), default=0), False

    t0 = time.time()
    while (S['scan'] is None or S['enc'] is None) and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    rows = []; first = True; aborted = False
    try:
        for rep in range(a.reps):
            for sv in (1, -1):
                clear = corridor_clearance(S['scan'], sv > 0, 0.0 if sv > 0 else 0.1)
                if clear < 0.6:
                    print(f'❌ {"앞" if sv > 0 else "뒤"} 여유 {clear:.2f} m — 중단'); return 1
                scan0, e0 = S['scan'], list(S['enc'])
                for w, dur in ((a.w, 0.6), (0.0, 1.0), (-a.w, 0.6), (0.0, 1.0)):
                    pk, ab = run(sv * a.v, w, dur)
                    rows.append(dict(v=sv * a.v, w=w, dur=dur, peak=pk))
                    print(f'  v {sv * a.v:+.2f} w {w:+.2f} {dur:.1f}s → 자이로 최대 {pk:.2f} rad/s', flush=True)
                    if ab:
                        print('❌ 안전 중단: |자이로| > 1.5 rad/s 0.2초 초과'); aborted = True; return 1
                stop(); rclpy.spin_once(n, timeout_sec=0.5)
                if first:
                    d = wheel_distances(e0, S['enc'])
                    bad, msg = looks_on_dock((d[0] + d[1]) / 2, scan0, S['scan']); print('  ' + msg)
                    if bad:
                        return 1
                    first = False
                time.sleep(0.8)
        # DWB식 짧은 제자리 회전 ↔ 저속 전진 교대
        for w, v, dur in ((0.29, 0.0, 0.5), (0.0, 0.054, 0.5), (-0.29, 0.0, 0.5), (0.0, 0.054, 0.5),
                          (0.45, 0.0, 0.4), (-0.45, 0.0, 0.4), (0.0, -0.054, 0.5)):
            pk, ab = run(v, w, dur)
            rows.append(dict(v=v, w=w, dur=dur, peak=pk))
            print(f'  v {v:+.3f} w {w:+.2f} {dur:.1f}s → 자이로 최대 {pk:.2f} rad/s', flush=True)
            if ab:
                print('❌ 안전 중단'); aborted = True; return 1
    finally:
        stop()
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'curve_pattern_{time.strftime("%Y%m%d_%H%M")}.json')
        json.dump(dict(rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1)
        worst = max((r['peak'] for r in rows), default=0)
        print(f'최대 자이로 {worst:.2f} rad/s → {"❌ 불합격" if worst > 1.0 or aborted else "✅ 합격(1 rad/s 미만)"} | 저장 {out}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
