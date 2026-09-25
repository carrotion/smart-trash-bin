#!/usr/bin/env python3
"""제자리 회전 출발 시나리오 매트릭스 — 목표 각속도 × 출발 방식 × 방향별 90° 회전.

배경(2026-09-25, 사용자 제안): 회전 출발 과회전(첫 1초 30~50%)을 각가속도 램프로
억제하려 했는데, 사용자 지적 — 제자리 회전은 네 바퀴에 같은 크기 출력을 주므로
출력을 서서히 올리면 정지마찰 문턱이 낮은 바퀴부터 먼저 돌고(M2가 가장 늦음) 로봇이
한쪽을 축으로 틀어지는 역효과가 날 수 있다. 추측 대신 시나리오별로 재서 데이터로
남기고, 목표 각속도를 낮춰 가며 출발이 무너지는 지점(cliff)도 찾는다.

회전마다 기록:
  - err_deg: 라이다 ICP 기준 실제 회전 − 90° (자이로로 90°에서 멈춤)
  - start_delay: 명령 후 자이로가 0.05 rad/s를 넘기까지(s)
  - wheel_start_spread: 네 바퀴 엔코더가 처음 움직인 시각의 최대−최소(s) — 출발 비대칭
    (⚠️ 좌회전에서 M4는 3·4번 채널 오독이 있어 참고만)
  - overshoot: 출발 1초 안 최대 각속도 ÷ 목표
  - shift_cm: 라이다 ICP 기준 위치 밀림

    python3 tools/turn_start_matrix.py   (바닥, velocity 모드 diff_drive_node 실행 중)
결과: docs/calibration/turn_start_matrix_YYYYMMDD_HHMM.json
"""
import argparse, json, math, os, statistics as st, sys, time
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray

import numpy as np
from rotation_quality_test import scan_points, icp


def icp_with_guess(after, before, guess_deg):
    """90° 회전은 항등 초기값에서 ICP가 엉뚱한 해에 빠진다(09-25 1차 실행 무효) →
    자이로가 잰 회전각으로 먼저 돌려 놓고 남은 차이만 ICP로 구한다."""
    g = np.radians(guess_deg)
    R = np.array([[np.cos(g), -np.sin(g)], [np.sin(g), np.cos(g)]])
    dx, dy, dth, resid = icp((R @ after.T).T, before)
    # 전체 변환 = ICP 결과 ∘ 초기 회전
    return dx, dy, guess_deg + dth, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default='0.15,0.2,0.3,0.5,0.8,1.2')
    ap.add_argument('--ramps', default='0,0.75,1.5,3.0', help='angular_accel_limit, 0=계단')
    args = ap.parse_args()
    speeds = [float(x) for x in args.speeds.split(',')]
    ramps = [float(x) for x in args.ramps.split(',')]

    rclpy.init()
    n = rclpy.create_node('turn_start_matrix')
    S = {'g': [], 'scan': None, 'enc': []}
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S['enc'].append((time.time(), list(m.data))), 50)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    pcli = n.create_client(SetParameters, '/diff_drive_node/set_parameters')

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.01)

    def set_ramp(v):
        req = SetParameters.Request(parameters=[Parameter(
            name='angular_accel_limit',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))])
        fut = pcli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
        return fut.result() is not None

    def stop(settle):
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)
        spin(settle)

    t0 = time.time()
    while (S['scan'] is None or len(S['enc']) < 5 or len(S['g']) < 50) and time.time() - t0 < 10:
        spin(0.1)
    if not pcli.wait_for_service(timeout_sec=3.0):
        print('❌ diff_drive_node 파라미터 서비스 없음'); return 1
    near = min((math.hypot(x, y) for x, y in scan_points(S['scan'])), default=9)
    print(f'주변 최근접 {near:.2f} m')
    if near < 0.35:
        print('❌ 여유 부족 — 중단'); return 1
    S['g'] = []
    spin(3.0)
    bias = st.mean(g for _, g in S['g'])     # /imu/data는 이미 영점 추적 적용 — 잔여만

    rows = []
    try:
        for w in speeds:
            for ramp in ramps:
                if not set_ramp(ramp):
                    print('❌ 파라미터 설정 실패'); return 1
                for sgn in (1, -1):
                    cmd = sgn * w
                    before = scan_points(S['scan'])
                    S['g'] = []
                    ts = time.time(); last = ts; yaw = 0.0
                    timeout = math.pi / 2 / w + 6.0
                    while abs(yaw) < math.pi / 2 - w * 0.12 and time.time() - ts < timeout:
                        c = Twist(); c.angular.z = cmd; pub.publish(c); spin(0.02)
                        now = time.time()
                        g = [v for t, v in S['g'] if t > last]
                        if g:
                            yaw += (st.mean(g) - bias) * (now - last)
                        last = now
                    timed_out = time.time() - ts >= timeout
                    t_stop = time.time()
                    stop(1.2)
                    yaw += sum((v - bias) * 0.02 for t, v in S['g'] if t > t_stop)   # 관성분(50Hz)
                    after = scan_points(S['scan'])
                    dx, dy, dth, resid = icp_with_guess(after, before, np.degrees(yaw))
                    rates = [(t - ts, (v - bias) * sgn) for t, v in S['g'] if t >= ts]
                    moving = [t for t, r in rates if r > 0.05]
                    start_delay = moving[0] if moving else None
                    # 과회전: 0.2초(약 10샘플) 이동평균의 최대 — 순간 최대는 저속에서 자이로 잡음에 부풀려졌다
                    sm = [st.mean(r for _, r in rates[k:k + 10]) for k in range(max(0, len(rates) - 9))]
                    first_sec = [v for (t, _), v in zip(rates, sm) if t <= (start_delay or 0) + 1.0]
                    # (바퀴별 출발 시각 차는 엔코더가 약 16Hz라 60ms 미만 차이를 못 봐서 뺐다)
                    row = {'w': w, 'dir': 'CCW' if sgn > 0 else 'CW', 'ramp': ramp,
                           'gyro_deg': math.degrees(abs(yaw)),
                           'err_deg': abs(dth) - 90.0, 'gyro_vs_lidar_deg': abs(dth) - math.degrees(abs(yaw)),
                           'icp_dtheta': dth, 'shift_cm': math.hypot(dx, dy) * 100, 'icp_resid_cm': resid * 100,
                           'start_delay': start_delay,
                           'overshoot': (max(first_sec) / w) if first_sec else None,
                           'timed_out': timed_out}
                    rows.append(row)
                    f = lambda x, fmt: (fmt % x) if x is not None else '  -  '
                    print(f'w {w:.2f} 램프 {ramp:4.2f} {row["dir"]:3s} | 90° 오차 {f(row["err_deg"], "%+5.1f")}° '
                          f'(자이로와 차 {row["gyro_vs_lidar_deg"]:+.1f}°) | 출발지연 {f(start_delay, "%.2f")}s | '
                          f'과회전 {f(row["overshoot"], "%.2f")}배 | 밀림 {row["shift_cm"]:.1f}cm (잔차 {resid * 100:.1f})'
                          + (' | ⏱ 시간초과' if timed_out else ''), flush=True)
    finally:
        stop(0.3)
        set_ramp(1.5)

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo, time.strftime('docs/calibration/turn_start_matrix_%Y%m%d_%H%M.json'))
    json.dump(rows, open(path, 'w'), ensure_ascii=False, indent=1)
    print('저장:', path)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
