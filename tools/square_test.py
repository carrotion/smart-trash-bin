#!/usr/bin/env python3
"""정사각형 복귀 시험 — "의도대로 움직이고 위치를 정확히 읽는가"를 숫자 두 개로 본다.

docs/todo.md 1a-4: 설계를 바꿀 때마다 이 숫자로 판단한다.

동작: [전진 side_m → 멈춤 → 제자리 좌회전 90°] × 4 → 출발점 복귀.
  - 전진 거리는 앞바퀴(M1,M2) 엔코더(뒷바퀴 3·4번 채널은 오독 이력이 있어 제외)
  - 회전 각도는 자이로 적분(/imu/data)
측정:
  ① 실제 복귀 오차: 출발 전 / 도착 후 라이다 스캔 ICP → dx, dy, dθ (실행 정확도)
  ② EKF가 믿는 복귀 오차: /odometry/filtered 출발·도착 자세 차 (위치 추정)
  ③ 추정 오차 = ②와 ①의 차이 — 지도 품질과 직결되는 값

⚠️ 바닥 전용. 앞쪽과 왼쪽으로 side_m + 0.3 m 이상 비어 있어야 한다(전진 전마다 검사).
velocity 모드 diff_drive_node 실행 중일 것.

    python3 tools/square_test.py --side 0.8 --speed 0.15 --turn 0.5
"""
import argparse, json, math, os, statistics as st, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray

from wheel_config import wheel_distances
from rotation_quality_test import scan_points, icp
from speed_floor_sweep import corridor_clearance


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', type=float, default=0.8)
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--turn', type=float, default=0.5)
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('square_test')
    S = {'g': [], 'scan': None, 'enc': None, 'ekf': None, 'rf2o': None}
    # 비교용: rf2o가 떠 있으면 그 복귀 오차도 같이 기록(09-26 재평가)
    n.create_subscription(Odometry, '/odom_rf2o', lambda m: S.update(
        rf2o=(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S.update(enc=list(m.data)), 20)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: S.update(
        ekf=(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.01)

    def stop(settle=1.0):
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)
        spin(settle)

    t0 = time.time()
    while (S['scan'] is None or S['enc'] is None or S['ekf'] is None or len(S['g']) < 50) and time.time() - t0 < 10:
        spin(0.1)
    if S['scan'] is None or S['enc'] is None or S['ekf'] is None:
        print('❌ /scan, /wheel_encoders, /odometry/filtered 중 미수신'); return 1

    spin(1.0)
    start_scan = scan_points(S['scan'])
    ekf0 = S['ekf']
    rf0 = S['rf2o']
    try:
        for leg in range(4):
            clear = corridor_clearance(S['scan'], True)
            if clear < args.side + 0.3:
                print(f'❌ {leg + 1}번째 전진 전 앞 여유 {clear:.2f} m < {args.side + 0.3:.2f} — 중단')
                stop(); return 1
            e0 = list(S['enc'])
            t_leg = time.time()
            leg_limit = args.side / args.speed * 2.0 + 2.0   # 예상 시간의 2배 + 2초
            while True:
                d = wheel_distances(e0, S['enc'])
                if (d[0] + d[1]) / 2 >= args.side:
                    break
                # ⚠️ 09-26: 루프에 시간 제한이 없어 센서가 이상하면 끝없이 달릴 수 있었다
                if time.time() - t_leg > leg_limit:
                    stop(); print(f'❌ {leg + 1}번째 전진이 {leg_limit:.0f}초 안에 안 끝남(엔코더 이상?) — 중단'); return 1
                c = Twist(); c.linear.x = args.speed; pub.publish(c); spin(0.02)
            stop()
            S['g'] = []; yaw = 0.0; last = time.time(); t_turn = last
            turn_limit = (math.pi / 2) / args.turn * 2.0 + 2.0
            while yaw < math.pi / 2 - args.turn * 0.12:
                c = Twist(); c.angular.z = args.turn; pub.publish(c); spin(0.02)
                now = time.time()
                g = [v for t, v in S['g'] if t > last]
                if g:
                    yaw += st.mean(g) * (now - last)
                last = now
                # ⚠️ 09-26: IMU가 주행 중 재시작돼 자이로가 0을 내는 동안 90°에 영원히 못 닿아
                # 약 5분간 회전했다. 시간 제한 + 1.5초 지나도 자이로 적분이 10° 미만이면 중단
                if now - t_turn > turn_limit or (now - t_turn > 1.5 and yaw < math.radians(10)):
                    stop(); print(f'❌ {leg + 1}번째 회전 이상(경과 {now - t_turn:.1f}초, 자이로 적분 '
                                  f'{math.degrees(yaw):.1f}°) — 중단'); return 1
            stop()
            print(f'  {leg + 1}/4 완료 (전진 {(d[0] + d[1]) / 2:.3f} m, 회전 {math.degrees(yaw):.1f}°+관성)', flush=True)
    finally:
        stop(0.5)

    spin(1.0)
    end_scan = scan_points(S['scan'])
    ekf1 = S['ekf']
    rf1 = S['rf2o']
    # ICP: 도착 스캔을 출발 스캔에 맞추는 변환 → 로봇의 실제 복귀 오차(출발 자세 기준)
    dx, dy, dth, resid = icp(end_scan, start_scan)
    # EKF: 출발 자세 기준 도착 위치
    c0, s0 = math.cos(ekf0[2]), math.sin(ekf0[2])
    ex, ey = ekf1[0] - ekf0[0], ekf1[1] - ekf0[1]
    ekf_dx, ekf_dy = c0 * ex + s0 * ey, -s0 * ex + c0 * ey
    ekf_dth = math.degrees((ekf1[2] - ekf0[2] + math.pi) % (2 * math.pi) - math.pi)
    # ICP 변환(도착 스캔 → 출발 스캔)이 곧 출발 자세 기준 로봇의 도착 자세다
    # (가짜 스캔 검증: 정답 +0.1, +0.05, +5° → ICP +0.105, +0.055, +4.76°)
    real_x, real_y, real_th = dx, dy, dth
    res = {
        'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'label': args.label, 'side_m': args.side,
        'speed': args.speed, 'turn': args.turn,
        'real_return_m': math.hypot(real_x, real_y), 'real_return_deg': real_th,
        'ekf_return_m': math.hypot(ekf_dx, ekf_dy), 'ekf_return_deg': ekf_dth,
        'estimate_error_m': math.hypot(ekf_dx - real_x, ekf_dy - real_y),
        'estimate_error_deg': ekf_dth - real_th, 'icp_resid_m': resid,
    }
    print(f'\n① 실제 복귀 오차(라이다): {res["real_return_m"] * 100:.1f} cm, {real_th:+.1f}°  (ICP 잔차 {resid * 100:.1f} cm)')
    print(f'② EKF가 믿는 복귀 오차:   {res["ekf_return_m"] * 100:.1f} cm, {ekf_dth:+.1f}°')
    print(f'③ 위치 추정 오차(②−①):  {res["estimate_error_m"] * 100:.1f} cm, {res["estimate_error_deg"]:+.1f}°')
    if rf0 and rf1:
        c, s_ = math.cos(rf0[2]), math.sin(rf0[2]); ex, ey = rf1[0] - rf0[0], rf1[1] - rf0[1]
        rx, ry = c * ex + s_ * ey, -s_ * ex + c * ey
        rth = math.degrees((rf1[2] - rf0[2] + math.pi) % (2 * math.pi) - math.pi)
        res.update(rf2o_return_m=math.hypot(rx, ry), rf2o_return_deg=rth,
                   rf2o_estimate_error_m=math.hypot(rx - real_x, ry - real_y), rf2o_estimate_error_deg=rth - real_th)
        print(f'④ rf2o가 믿는 복귀 오차:  {res["rf2o_return_m"] * 100:.1f} cm, {rth:+.1f}° → '
              f'추정 오차 {res["rf2o_estimate_error_m"] * 100:.1f} cm, {res["rf2o_estimate_error_deg"]:+.1f}°')
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo, 'docs/calibration/square_test.json')
    hist = json.load(open(path)) if os.path.exists(path) else []
    hist.append(res)
    json.dump(hist, open(path, 'w'), ensure_ascii=False, indent=1)
    print('저장:', path)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
