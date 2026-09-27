#!/usr/bin/env python3
"""도크 위(바퀴만 공중에서 돎, 차체 정지)에서 모터 구동 중 자이로 치우침 측정 — 2층 원인 판정.

09-27: 1 m 직진 6회 중 5회에서 달리는 동안 자이로가 약 +0.0015 rad/s 치우침(정지 때와 반대, 10배).
차체가 안 움직이는 도크에서도 모터를 돌릴 때 치우치면 → 진동·전기 잡음이 원인(모터 구동 중 영점 보정으로 대응 가능).
안 치우치면 → 실제 주행에서만 생기는 현상(다른 원인).

단계: 정지 20 s → [전진 0.15, 정지, 후진 0.15, 정지, 전진 0.3, 정지, 좌회전 0.8, 정지, 우회전 0.8, 정지] 각 10 s.
각 단계 평균 자이로(처음 1 s 제외). ⚠️ 도크 전용: 매 단계 라이다 스캔 변화 > 1 cm면 차체가 움직인 것 → 즉시 정지·중단.

    python3 tools/dock_gyro_bias_test.py
"""
import json, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dock_guard import scan_change

PHASES = [('정지(기준)', 0.0, 0.0, 20), ('전진 0.15', 0.15, 0.0, 10), ('정지', 0.0, 0.0, 10), ('후진 0.15', -0.15, 0.0, 10),
          ('정지', 0.0, 0.0, 10), ('전진 0.3', 0.3, 0.0, 10), ('정지', 0.0, 0.0, 10), ('좌회전 0.8', 0.0, 0.8, 10),
          ('정지', 0.0, 0.0, 10), ('우회전 0.8', 0.0, -0.8, 10), ('정지', 0.0, 0.0, 10)]


def main():
    rclpy.init(); n = rclpy.create_node('dock_gyro_bias_test')
    S = {'g': [], 'scan': None}
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
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
    while S['scan'] is None and time.time() - t0 < 10:
        spin(0.1)
    ref = S['scan']; rows = []; aborted = False
    try:
        for name, v, w, dur in PHASES:
            S['g'] = []; ts = time.time()
            while time.time() - ts < dur:
                if v or w:
                    c = Twist(); c.linear.x = float(v); c.angular.z = float(w); pub.publish(c)
                spin(0.05)
                ch = scan_change(ref, S['scan'])
                if ch is not None and ch > 0.01:
                    stop(); print(f'❌ 차체가 움직임(스캔 변화 {ch*100:.1f} cm) — 도크가 아님, 중단'); aborted = True; return 1
            if v or w:
                stop()
            g = [x for t, x in S['g'] if t >= ts + 1.0]
            mean = sum(g) / len(g) if g else float('nan')
            rows.append(dict(phase=name, v=v, w=w, gyro_mean=mean, n=len(g)))
            print(f'{name:10s}: 자이로 평균 {mean:+.5f} rad/s ({len(g)}샘플)', flush=True)
    finally:
        stop()
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'dock_gyro_bias_{time.strftime("%Y%m%d_%H%M")}.json')
        json.dump(dict(rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1)
        base = [r['gyro_mean'] for r in rows if r['v'] == 0 and r['w'] == 0]
        if base:
            b = sum(base) / len(base)
            print(f'정지 평균 {b:+.5f} → 구동 단계는 정지 대비: ' + ', '.join(f'{r["phase"]} {r["gyro_mean"] - b:+.5f}' for r in rows if r['v'] or r['w']))
        print('저장:', out)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
