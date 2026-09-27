#!/usr/bin/env python3
"""1층 기본 동작 ④ — 정해진 각도까지 회전 (docs/motion_foundation.md 합격 기준).

각도 15, 45, 90, 180° × 좌/우를 tools/turn_to_angle.py(EKF 방향 폐루프)로 돌리고 판정:
  최종 오차 ≤ 2°(EKF) / ≤ 3°(라이다 ICP: 회전 전후 스캔 정합) / 최대 회전(0.2초 평균) ≤ 1.2 rad/s /
  시간 ≤ 2.5 s + 각도/0.6 rad/s(time_limit)
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

# 튐 기준: 설정 대비 배율(1.3×) → **절대 1.2 rad/s**(0.2초 평균). ⚠️ 09-27: 최대 속도를 0.8→0.7→0.4로 낮춰도 실제 최대가
#   1.0~1.1 rad/s로 같았다(차체가 길게 돌면 자연 회전 속도로 감) → 배율은 설정만 낮추면 나빠지는 잘못된 척도.
#   중요한 건 실제 회전이 안전한 속도 안에 머무는지(사고 때 4.4, 폭주 잠금 2.5). 배율은 참고로 남김
CRIT = dict(err_ekf=2.0, err_icp=3.0, peak=1.2, time_over=0.0)


def time_limit(ang_deg):
    """허용 시간 3.0 s + 각/0.6 rad/s(90° ≈ 5.6 s, 180° ≈ 8.2 s). 09-27 사용자 "해보고 합리적 수준으로" →
    D 설정·coast 0.17 16회 실측(보정 없는 회전은 모두 이내, 보정 1회 붙으면 +3 s)을 보고 정함."""
    return 3.0 + math.radians(abs(ang_deg)) / 0.6


# 합격 = 항목별 통과 비율(한 번씩의 편차가 있어 전체 비율로 판정, 09-27 결정)
RATE = dict(err_icp=0.95, err_ekf=0.90, peak=0.95, time_over=0.80)


def icp_rotation_deg(before, after, guess_rad):
    """회전 전 스캔(before)에 회전 후 스캔(after)을 맞춰 실제 회전각(도). 제자리 회전이라 초기값은 회전만."""
    pb, pa = np.array(scan_points(before)), np.array(scan_points(after))
    R = np.array([[math.cos(guess_rad), -math.sin(guess_rad)], [math.sin(guess_rad), math.cos(guess_rad)]])
    dx, dy, dth, resid = icp((R @ pa.T).T, pb)
    return math.degrees(guess_rad) + dth, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--angles', type=float, nargs='+', default=[15, 45, 90, 180])
    # 기본값 None = turn_to_angle.py 기본값(09-27 ④ 비교 D 설정)을 그대로
    ap.add_argument('--w-max', type=float, default=None)
    ap.add_argument('--kp', type=float, default=None, help='남은 각(rad) × kp = 감속 목표 속도')
    ap.add_argument('--w-min', type=float, default=None)
    ap.add_argument('--coast', type=float, default=None, help='정지 명령 뒤 관성 시간(s) 추정')
    ap.add_argument('--settle', type=float, default=None, help='정지 판정에 필요한 조용한 시간(s)')
    ap.add_argument('--accel', type=float, default=None, help='명령 가속 제한(rad/s²)')
    ap.add_argument('--tol', type=float, default=None, help='보정 회전 기준(°)')
    ap.add_argument('--max-fix', type=int, default=None)
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
    kw = {k: v for k, v in dict(kp=a.kp, w_min=a.w_min, w_max=a.w_max, coast_s=a.coast, settle_s=a.settle,
                                accel=a.accel, tol_deg=a.tol, max_fix=a.max_fix).items() if v is not None}
    turner = TurnToAngle(pub, spin, lambda: S['ekf'], lambda: S['gz'], **kw)
    w_max = turner.w_max
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
                # 부호 있는 값도 남김(09-27: 절댓값만 있어 EKF/자이로 배율을 방향별로 못 구했다)
                m = dict(angle=sgn * ang, turned_ekf=turned_ekf, turned_icp=real, err_ekf=abs(turned_ekf - sgn * ang), err_icp=abs(real - sgn * ang),
                         icp_resid_cm=resid * 100, peak=peak, peak_ratio=peak / w_max,
                         secs=secs, time_over=max(0.0, secs - time_limit(ang)),
                         fixes=r['fixes'], ok_turn=r['ok'], main_err=r.get('main_err_deg'))
                m['pass'] = {k: (m[k] <= CRIT[k]) for k in CRIT}
                m['ok'] = all(m['pass'].values()) and r['ok'] and resid < 0.05
                rows.append(m); p = m['pass']
                print(f'{sgn*ang:+5.0f}°: EKF 오차 {m["err_ekf"]:.1f}°{"✅" if p["err_ekf"] else "❌"} '
                      f'라이다 오차 {m["err_icp"]:.1f}°{"✅" if p["err_icp"] else "❌"}(잔차 {resid*100:.1f}cm) '
                      f'최대 {peak:.2f} rad/s{"✅" if p["peak"] else "❌"}(×{m["peak_ratio"]:.2f}) '
                      f'{secs:.1f}초{"✅" if p["time_over"] else "❌"} 보정 {r["fixes"]}회(첫 회전 뒤 {(r.get("main_err_deg") or 0) * (1 if ang * sgn > 0 else -1):+.1f}°)', flush=True)
    except Abort as e:
        print('❌ 안전 중단:', e); aborted = True
    finally:
        for _ in range(8):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'turn_angle_{time.strftime("%Y%m%d_%H%M")}{("_" + a.label) if a.label else ""}.json')
        json.dump(dict(criteria=CRIT, args=vars(a), turner=dict(kp=turner.kp, w_min=turner.w_min, w_max=turner.w_max, accel=turner.accel, settle=turner.settle_s), rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1)
        rates = {k: (sum(r['pass'][k] for r in rows) / len(rows) if rows else 0.0) for k in RATE}
        ok = bool(rows) and not aborted and all(rates[k] >= RATE[k] for k in RATE) and all(r['ok_turn'] for r in rows)
        print('항목별 통과율: ' + ', '.join(f'{k} {rates[k]*100:.0f}%(기준 {RATE[k]*100:.0f}%)' for k in RATE))
        print(f'④ 각도 회전: {"✅ 합격" if ok else "❌ 불합격"} | 저장 {out}')
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
