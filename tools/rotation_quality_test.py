#!/usr/bin/env python3
"""제자리 회전 품질 측정 — 속도별로 자이로 기준 정확히 한 바퀴(360°) 돌고 비교한다.

배경(2026-09-25, docs/todo.md 1a): 사용자 제안 — 회전 출력에 따라 슬립과 동작 차이가
뚜렷하므로, 가장 깨끗하게 도는 "최적 회전 속도"를 찾아 회전 기본값으로 쓴다.

속도마다(좌/우 각각):
  - 미끄러짐: 자이로 회전량 ÷ 앞바퀴(M1,M2) 엔코더로 계산한 회전량
    (뒷바퀴 3·4번 채널은 오독 이력이 있어 제외. 실효 트레드 wheel_separation 기준)
  - 흔들림: 회전 중간 구간 각속도 표준편차
  - 정지 정확도: 멈추고 1.5초 뒤 총 회전량 − 360°
  - 위치 밀림: 회전 전후 라이다 스캔 ICP 정합 → 실제 이동량(m)과 각도 차

사용법 (바닥, 주변 여유 확보, velocity 모드 diff_drive_node 실행 중):
    python3 tools/rotation_quality_test.py [--speeds 0.3,0.5,0.7,0.9,1.2]
결과: docs/calibration/rotation_quality_YYYYMMDD_HHMM.json
"""
import argparse, json, math, os, statistics as st, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

from wheel_config import wheel_distances

TRACK = 0.3291        # diff_drive_node wheel_separation(실효)
LIDAR_X = 0.054
HALF = 0.13           # 차체 + 부착물 제외 범위


def scan_points(m):
    pts = []
    for i, r in enumerate(m.ranges):
        if not (m.range_min <= r <= min(m.range_max, 6.0)):
            continue
        a = m.angle_min + i * m.angle_increment
        x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
        if abs(x) <= HALF and abs(y) <= HALF:
            continue
        pts.append((x, y))
    return np.array(pts)


def icp(src, dst, iters=40):
    """src를 dst에 맞추는 강체변환(R,t). 반환: dx, dy, dθ(도), 평균 잔차."""
    R = np.eye(2); t = np.zeros(2); cur = src.copy()
    for _ in range(iters):
        d2 = ((cur[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1); dist = np.sqrt(d2[np.arange(len(cur)), j])
        keep = dist < max(0.05, np.percentile(dist, 80))
        a, b = cur[keep], dst[j[keep]]
        ma, mb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
        Ri = (U @ Vt).T
        if np.linalg.det(Ri) < 0:
            Vt[1] *= -1; Ri = (U @ Vt).T
        ti = mb - Ri @ ma
        cur = (Ri @ cur.T).T + ti
        R, t = Ri @ R, Ri @ t + ti
    return t[0], t[1], math.degrees(math.atan2(R[1, 0], R[0, 0])), float(dist[keep].mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default='0.3,0.5,0.7,0.9,1.2')
    # 여러 바퀴: 자이로 눈금 오차가 바퀴 수만큼 커져 라이다로 정밀하게 보인다
    # (2026-09-25: 1바퀴 시험에서 자이로 360° 뒤 라이다 각 차 평균 약 5.8°, 좌/우 부호 대칭)
    ap.add_argument('--turns', type=int, default=1)
    # 영점: 1초 평균은 불확실성 ±0.0035 rad/s라 5바퀴(약 63초)에 ±13° — 눈금 판정을 가렸다(09-25)
    ap.add_argument('--bias-sec', type=float, default=10.0)
    args = ap.parse_args()
    speeds = [float(x) for x in args.speeds.split(',')]

    rclpy.init()
    n = rclpy.create_node('rotation_quality_test')
    S = {'g': [], 'scan': None, 'enc': None, 'odom': None}
    n.create_subscription(Odometry, '/odom', lambda m: S.update(odom=(m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S.update(enc=list(m.data)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.01)

    t0 = time.time()
    while (S['scan'] is None or S['enc'] is None or len(S['g']) < 50) and time.time() - t0 < 10:
        spin(0.1)
    if S['scan'] is None or S['enc'] is None or len(S['g']) < 50:
        print('❌ /scan, /wheel_encoders, /imu/data 중 미수신'); return 1
    near = min((math.hypot(x, y) for x, y in scan_points(S['scan'])), default=9)
    print(f'주변 최근접 {near:.2f} m')
    if near < 0.35:
        print('❌ 여유 부족 — 중단'); return 1

    def stop():
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)

    def still_scan():
        spin(0.3)
        return scan_points(S['scan'])

    S['g'] = []
    spin(args.bias_sec)
    bias = st.mean(g for _, g in S['g'])
    print(f'자이로 영점 {bias:+.5f} rad/s ({args.bias_sec:.0f}초 평균, 표준오차 {st.pstdev(g for _, g in S["g"]) / math.sqrt(len(S["g"])):.5f})')
    results = []
    for w in speeds:
        for sgn in (1, -1):
            cmd = sgn * w
            before = still_scan()
            e0 = list(S['enc'])
            o0 = S['odom']
            S['g'] = []
            yaw = 0.0; last = time.time(); ts = last; rates = []
            while True:
                c = Twist(); c.angular.z = cmd; pub.publish(c)
                spin(0.02)
                now = time.time()
                g = [v for t, v in S['g'] if t > last]
                if g:
                    rate = st.mean(g) - bias
                    yaw += rate * (now - last); rates.append((now - ts, rate))
                last = now
                # 멈추는 데 걸리는 회전을 예상해 조금 일찍 정지(0.12초 분량)
                if abs(yaw) >= 2 * math.pi * args.turns - abs(cmd) * 0.12 or now - ts > 2 * math.pi * args.turns / w + 8:
                    break
            stop()
            t_stop = time.time()
            spin(1.5)
            yaw += sum((v - bias) * 0.02 for t, v in S['g'] if t > t_stop)  # 관성분(50Hz)
            after = still_scan()
            d = wheel_distances(e0, S['enc'])
            wheel_yaw = (d[1] - d[0]) / TRACK            # 앞바퀴만
            mid = [r for tt, r in rates if 0.3 * rates[-1][0] < tt < 0.8 * rates[-1][0]]
            dx, dy, dth, resid = icp(after, before)
            o1 = S['odom']
            odom_shift = math.hypot(o1[0] - o0[0], o1[1] - o0[1]) if o0 and o1 else None
            row = {'cmd': cmd, 'gyro_deg': math.degrees(yaw), 'overshoot_deg': math.degrees(abs(yaw)) - 360 * args.turns, 'turns': args.turns,
                   'slip_ratio': yaw / wheel_yaw if abs(wheel_yaw) > 0.1 else None,
                   'jitter': st.pstdev(mid) if len(mid) > 2 else None, 'mean_rate': st.mean(mid) if mid else None,
                   'shift_m': math.hypot(dx, dy), 'icp_dtheta_deg': dth, 'icp_resid': resid,
                   'odom_shift_m': odom_shift, 'bias': bias}
            # ICP의 dθ는 "회전 뒤 스캔을 회전 전으로 맞추는 각" — 실제 회전 = 자이로 회전 + dθ(부호는 회전 방향 기준)
            actual = abs(math.degrees(yaw)) + math.copysign(1, cmd) * dth
            row['actual_deg_lidar'] = actual
            row['gyro_scale_needed'] = actual / abs(math.degrees(yaw))
            results.append(row)
            print(f'   → 라이다 기준 실제 {actual:.1f}° / 자이로 {abs(math.degrees(yaw)):.1f}° = 필요 눈금 {row["gyro_scale_needed"]:.4f}', flush=True)
            print(f'{cmd:+.1f} rad/s: 회전 {row["gyro_deg"]:+.0f}° (넘침 {row["overshoot_deg"]:+.1f}°) | '
                  f'자이로/바퀴 {row["slip_ratio"]:.2f} | 흔들림 {row["jitter"]:.3f} (평균 {row["mean_rate"]:+.2f}) | '
                  f'위치 밀림 {row["shift_m"]*100:.1f} cm(오도메트리 {odom_shift*100:.1f} cm), 각 {dth:+.1f}° (ICP 잔차 {resid*100:.1f} cm)', flush=True)
            spin(0.5)
    stop()
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo, time.strftime('docs/calibration/rotation_quality_%Y%m%d_%H%M.json'))
    with open(path, 'w') as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    print('저장:', path)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
