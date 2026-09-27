#!/usr/bin/env python3
"""1층 기본 동작 ④ — 정해진 각도까지 회전 (docs/motion_foundation.md 합격 기준).

각도 15, 45, 90, 180° × 좌/우를 tools/turn_to_angle.py(EKF 방향 폐루프)로 돌리고 판정:
  최종 오차 ≤ 2°(EKF) / ≤ 3°(라이다 ICP: 회전 전후 스캔 정합) / 최대 회전(0.2초 평균) ≤ 1.3 × w_max /
  시간 ≤ 각도/w_max + 2 s
안전: |자이로| > 2.0 rad/s 0.2초 초과 시 정지·중단. 주변 0.4 m 여유. ⚠️ 바닥 전용.

    python3 tools/turn_angle_test.py [--angles 15 45 90 180] [--label 이름]
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rotation_quality_test import scan_points, icp
from turn_to_angle import TurnToAngle, yaw_of, wrap

CRIT = dict(err_ekf=2.0, err_icp=3.0, peak_ratio=1.3, time_over=0.0)


def icp_rotation_deg(before, after, guess_rad):
    """회전 전 스캔(before)에 회전 후 스캔(after)을 맞춰 실제 회전각(도). 제자리 회전이라 초기값은 회전만."""
    pb, pa = np.array(scan_points(before)), np.array(scan_points(after))
    R = np.array([[math.cos(guess_rad), -math.sin(guess_rad)], [math.sin(guess_rad), math.cos(guess_rad)]])
    dx, dy, dth, resid = icp((R @ pa.T).T, pb)
    return math.degrees(guess_rad) + dth, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--angles', type=float, nargs='+', default=[15, 45, 90, 180])
    ap.add_argument('--w-max', type=float, default=0.5)
    ap.add_argument('--kp', type=float, default=1.5, help='남은 각(rad) × kp = 감속 목표 속도')
    ap.add_argument('--w-min', type=float, default=0.2)
    ap.add_argument('--coast', type=float, default=0.23, help='정지 명령 뒤 관성 시간(s) 추정')
    ap.add_argument('--settle', type=float, default=0.3, help='정지 판정에 필요한 조용한 시간(s)')
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('turn_angle_test')
    S = {'gz': 0.0, 'g': [], 'scan': None, 'ekf': None}

    def on_imu(m):
        S['gz'] = m.angular_velocity.z; S['g'].append((time.time(), m.angular_velocity.z))
    n.create_subscription(Imu, '/imu/data', on_imu, 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: S.update(ekf=yaw_of(m.pose.pose.orientation)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    class Abort(Exception):
        pass
    over = {'t': None}

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)
            if abs(S['gz']) > 2.0:
                over['t'] = over['t'] or time.time()
                if time.time() - over['t'] > 0.2:
                    for _ in range(8):
                        pub.publish(Twist())
                    raise Abort(f'|자이로| {S["gz"]:+.2f} rad/s')
            else:
                over['t'] = None

    t0 = time.time()
    while (S['scan'] is None or S['ekf'] is None or len(S['g']) < 20) and time.time() - t0 < 10:
        spin(0.1)
    turner = TurnToAngle(pub, spin, lambda: S['ekf'], lambda: S['gz'], kp=a.kp, w_min=a.w_min, w_max=a.w_max,
                         coast_s=a.coast, settle_s=a.settle)
    rows = []; aborted = False
    try:
        for ang in a.angles:
            for sgn in (1, -1):
                near = min((math.hypot(x, y) for x, y in scan_points(S['scan'])), default=9)
                if near < 0.4:
                    print(f'❌ 주변 여유 {near:.2f} m < 0.4 — 중단'); aborted = True; return 1
                spin(1.5)
                scan0, y0 = S['scan'], S['ekf']
                S['g'] = []; ts = time.time()
                r = turner.turn_by(math.radians(sgn * ang))
                spin(1.0)                                    # 정지 후 안정
                secs = time.time() - ts - 1.0
                y1 = S['ekf']; turned_ekf = math.degrees(wrap(y1 - y0))
                if abs(ang) >= 179:                          # 180°는 ±180 경계 — 부호 맞춰 펼침
                    turned_ekf = turned_ekf if turned_ekf * sgn > 0 else turned_ekf + 360 * sgn
                real, resid = icp_rotation_deg(scan0, S['scan'], math.radians(turned_ekf))
                # 0.2초 평균 최대 회전
                win = [(t - ts, abs(g)) for t, g in S['g']]
                avg = [sum(x for tt, x in win if t - 0.2 < tt <= t) / max(1, sum(1 for tt, x in win if t - 0.2 < tt <= t))
                       for t, _ in win if t >= 0.2]
                peak = max(avg, default=0.0)
                m = dict(angle=sgn * ang, err_ekf=abs(turned_ekf - sgn * ang), err_icp=abs(real - sgn * ang),
                         icp_resid_cm=resid * 100, peak=peak, peak_ratio=peak / a.w_max,
                         secs=secs, time_over=max(0.0, secs - (math.radians(ang) / a.w_max + 2.0)),
                         fixes=r['fixes'], ok_turn=r['ok'])
                m['pass'] = {k: (m[k] <= CRIT[k]) for k in CRIT}
                m['ok'] = all(m['pass'].values()) and r['ok'] and resid < 0.05
                rows.append(m); p = m['pass']
                print(f'{sgn*ang:+5.0f}°: EKF 오차 {m["err_ekf"]:.1f}°{"✅" if p["err_ekf"] else "❌"} '
                      f'라이다 오차 {m["err_icp"]:.1f}°{"✅" if p["err_icp"] else "❌"}(잔차 {resid*100:.1f}cm) '
                      f'최대 {peak:.2f}(×{m["peak_ratio"]:.2f}{"✅" if p["peak_ratio"] else "❌"}) '
                      f'{secs:.1f}초{"✅" if p["time_over"] else "❌"} 보정 {r["fixes"]}회', flush=True)
    except Abort as e:
        print('❌ 안전 중단:', e); aborted = True
    finally:
        for _ in range(8):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'turn_angle_{time.strftime("%Y%m%d_%H%M")}{("_" + a.label) if a.label else ""}.json')
        json.dump(dict(criteria=CRIT, args=vars(a), rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1)
        ok = rows and all(r['ok'] for r in rows) and not aborted
        print(f'④ 각도 회전: {"✅ 합격" if ok else "❌ 불합격"} ({sum(r["ok"] for r in rows)}/{len(rows)}) | 저장 {out}')
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
