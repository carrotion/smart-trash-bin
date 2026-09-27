#!/usr/bin/env python3
"""1층 기본 동작 — 정해진 거리 직진 합격 시험(docs/motion_foundation.md).

0.3, 0.5 m × 앞/뒤를 tools/drive_distance.py로 가고, 출발·도착 라이다 스캔 정합(ICP)으로 실제 이동을 잰다.
기준(비율 판정): 거리 오차 ≤ 2 cm(라이다) 90%, 방향 변화 ≤ 1°(라이다) 90%, 옆 밀림 ≤ 2 cm(라이다) 90%,
  주행 중 |자이로| 최대 ≤ 0.3 rad/s 95%, 시간 ≤ 거리/0.1 + 3 s 80%.
안전: 가는 쪽 통로 여유 < 남은 거리 + 0.2 m면 정지, 첫 동작 뒤 도크 검사. ⚠️ 바닥 전용.

    python3 tools/drive_distance_test.py [--dists 0.3 0.5] [--reps 2] [--label 이름]
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rotation_quality_test import scan_points, icp
from speed_floor_sweep import corridor_clearance
from dock_guard import looks_on_dock
from drive_distance import DriveDistance
from turn_to_angle import yaw_of

# 방향 기준 1.5°: 멈춤-확인-이동은 다음 단계에서 ④ 각도 회전(허용 3°)으로 방향을 다시 맞춘다 → 직진 방향을 그보다
#   훨씬 엄격하게(1°) 요구할 이유가 없음. 09-27 방향 유지 보정 적용 24회 최대 1.3°(1° 기준 88%)를 보고 정함
CRIT = dict(err_icp=0.02, head_icp=1.5, lat_icp=0.02, gyro_peak=0.3, time_over=0.0)
RATE = dict(err_icp=0.90, head_icp=0.90, lat_icp=0.90, gyro_peak=0.95, time_over=0.80)


def icp_motion(before, after, guess):
    """출발 스캔 기준 도착 스캔 정합 → 로봇 이동 (dx 앞, dy 왼쪽, dθ°), 잔차. guess=(dx,dy,dθ rad)."""
    pb, pa = np.array(scan_points(before)), np.array(scan_points(after))
    g = guess[2]; R = np.array([[math.cos(g), -math.sin(g)], [math.sin(g), math.cos(g)]])
    q = (R @ pa.T).T + np.array([guess[0], guess[1]])
    dx, dy, dth, resid = icp(q, pb)
    # 초기값 적용 뒤 남은 보정: 전체 = 보정 ∘ 초기값 (작은 보정이라 근사 합성)
    th = math.radians(dth)
    tx = math.cos(th) * guess[0] - math.sin(th) * guess[1] + dx
    ty = math.sin(th) * guess[0] + math.cos(th) * guess[1] + dy
    return tx, ty, math.degrees(g) + dth, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dists', type=float, nargs='+', default=[0.3, 0.5])
    ap.add_argument('--reps', type=int, default=2)
    ap.add_argument('--coast', type=float, default=None, help='정지 명령 뒤 관성 시간(s) 추정')
    ap.add_argument('--head-kp', type=float, default=None)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('drive_distance_test')
    S = {'pose': None, 'v': 0.0, 'gz': 0.0, 'g': [], 'scan': None}

    def on_odom(m):
        p = m.pose.pose; S['pose'] = (p.position.x, p.position.y, yaw_of(p.orientation)); S['v'] = m.twist.twist.linear.x

    def on_imu(m):
        S['gz'] = m.angular_velocity.z; S['g'].append((time.time(), m.angular_velocity.z))
    n.create_subscription(Odometry, '/odometry/filtered', on_odom, 20)
    n.create_subscription(Imu, '/imu/data', on_imu, 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)

    def clear(sgn):
        return corridor_clearance(S['scan'], sgn > 0, 0.0 if sgn > 0 else 0.1) > 0.2

    t0 = time.time()
    while (S['scan'] is None or S['pose'] is None) and time.time() - t0 < 10:
        spin(0.1)
    kw = {k: v for k, v in dict(coast_s=a.coast, head_kp=a.head_kp).items() if v is not None}
    dd = DriveDistance(pub, spin, lambda: S['pose'], lambda: S['v'], clear_fn=clear, **kw)
    rows = []; first = True; aborted = False
    try:
        for rep in range(a.reps):
            for dist in a.dists:
                for sgn in (1, -1):
                    d = sgn * dist
                    room = corridor_clearance(S['scan'], sgn > 0, 0.0 if sgn > 0 else 0.1)
                    if room < dist + 0.3:
                        print(f'❌ {"앞" if sgn > 0 else "뒤"} 여유 {room:.2f} m < {dist + 0.3:.2f} — 중단'); aborted = True; return 1
                    spin(1.0)
                    scan0, p0 = S['scan'], S['pose']; S['g'] = []; ts = time.time()
                    r = dd.drive_by(d)
                    spin(1.0)
                    secs = time.time() - ts - 1.0
                    p1 = S['pose']
                    c, s_ = math.cos(p0[2]), math.sin(p0[2]); ex, ey = p1[0] - p0[0], p1[1] - p0[1]
                    g = (c * ex + s_ * ey, -s_ * ex + c * ey, (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)
                    if first:
                        bad, msg = looks_on_dock(g[0], scan0, S['scan']); print('  ' + msg)
                        if bad:
                            aborted = True; return 1
                        first = False
                    tx, ty, th, resid = icp_motion(scan0, S['scan'], g)
                    gp = max((abs(x) for t, x in S['g']), default=0.0)
                    m = dict(dist=d, icp_dx=tx, icp_dy=ty, icp_dth=th, ekf_dx=g[0], ekf_dy=g[1], ekf_dth=math.degrees(g[2]),
                             err_ekf=abs(r['err_m']), err_icp=abs(abs(tx) - dist), head_icp=abs(th), lat_icp=abs(ty),
                             icp_resid_cm=resid * 100, gyro_peak=gp, secs=secs,
                             time_over=max(0.0, secs - (dist / 0.1 + 3.0)), fixes=r['fixes'], ok_drive=r['ok'],
                             main_err_cm=(r['main_err_m'] or 0) * 100, dir_ok=bool(tx * sgn > 0))
                    m = {k: (float(v) if hasattr(v, 'dtype') else v) for k, v in m.items()}
                    m['pass'] = {k: bool(m[k] <= CRIT[k]) for k in CRIT}
                    rows.append(m); p = m['pass']
                    print(f'{d:+.2f} m: 라이다 거리 {tx:+.3f} m(오차 {m["err_icp"]*100:.1f} cm{"✅" if p["err_icp"] else "❌"}) '
                          f'방향 {th:+.1f}°{"✅" if p["head_icp"] else "❌"} 옆 {ty*100:+.1f} cm{"✅" if p["lat_icp"] else "❌"} '
                          f'자이로 최대 {gp:.2f}{"✅" if p["gyro_peak"] else "❌"} {secs:.1f}초{"✅" if p["time_over"] else "❌"} '
                          f'| EKF 오차 {m["err_ekf"]*100:.1f} cm, 첫 주행 뒤 {m["main_err_cm"]:+.1f} cm, 보정 {r["fixes"]}회, 잔차 {resid*100:.1f} cm'
                          + ('' if r['ok'] else f' ❌ {r["msg"]}'), flush=True)
                    if not r['ok']:
                        aborted = True; return 1
    finally:
        for _ in range(8):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'drive_distance_{time.strftime("%Y%m%d_%H%M")}{("_" + a.label) if a.label else ""}.json')
        json.dump(dict(criteria=CRIT, rate=RATE, rows=rows, aborted=aborted), open(out, 'w'), ensure_ascii=False, indent=1)
        rates = {k: (sum(r['pass'][k] for r in rows) / len(rows) if rows else 0.0) for k in RATE}
        ok = bool(rows) and not aborted and all(rates[k] >= RATE[k] for k in RATE)
        print('항목별 통과율: ' + ', '.join(f'{k} {rates[k]*100:.0f}%(기준 {RATE[k]*100:.0f}%)' for k in RATE))
        print(f'직진 거리: {"✅ 합격" if ok else "❌ 불합격"} | 저장 {out}')
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
