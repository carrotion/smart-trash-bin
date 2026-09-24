#!/usr/bin/env python3
"""엔코더 폐루프로 정확한 거리만큼 전진한다.

⚠️ 2026-08-29 배경: 시간×속도로 이동거리를 예측하는 개루프 방식은 이날
전면 실패했다 — min_pwm 데드존 + motor_pwm_offset(특히 M3 +240, 오늘
커넥터 수리로 과잉보정이 됨)가 겹쳐 정속 구간 자체가 명령의 2~2.7배로
나갔고, 거기에 정지 후 관성 코스팅(0.65~0.8초에 걸쳐 수십cm)까지 더해져
실측 배율이 최대 5배까지 벌어졌다(0.1m/s×2초 명령 20cm -> 실제 100cm).

그래서 "시간을 재서 멈추기" 대신 **매 순간 엔코더로 실제 이동거리와
실제 속도를 직접 측정**하고, "지금 멈추면 코스팅으로 얼마나 더 갈지"를
실시간으로 예측해서 그 예측 합이 목표에 도달하는 순간 구동을 끊는다.
코스팅거리는 정속 상태 실측 13개(v_at_stop 0.29~0.96m/s)를 선형피팅한
`coast = COAST_K * v` (K=0.53s, 원점통과 모델 — 절편모델이 RMSE는 근소하게
낫지만 v=0에서 코스팅이 0이 아니게 나와 물리적으로 부자연스러움. 제곱/
지수 모델은 전부 선형보다 못 맞음). 실측 검증: 목표 50cm 명령 -> 엔코더
계산 49.5cm, 줄자 실측도 "50cm 조금 못 미침"으로 일치(오차 1% 수준).

⚠️ 이 K값은 도크 위가 아니라 **바닥 실주행**, 이 기체(1호기)의 오늘
(2026-08-29) 마찰 상태 기준이다. 기체가 다르거나 바닥 재질이 다르거나
날짜가 많이 지나면(마모/재수리로 마찰이 또 바뀌면) `tools/coast_sweep.py`로
재측정할 것. 데이터: docs/calibration/coast_sweep.csv

사용법:
    python3 tools/precise_move.py 0.5   # 50cm 전진
"""
import rclpy, math, time, sys
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from wheel_config import center  # 채널 매핑/부호/카운트당거리 단일 출처

SPEED = 0.2
COAST_K = 0.53  # coast_dist ≈ COAST_K * v_at_stop, docs/calibration/coast_sweep.csv 13개 실측 피팅
# ⚠️ 2026-09-24: 이 K는 370 4개 시절 값 — 뒷바퀴 520 교체로 관성/마찰이 바뀌어
# 무효일 가능성 높음. coast_sweep.py로 재측정 전까지는 거리 정확도를 믿지 말 것.

TARGET_M = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
SAFETY_MARGIN_M = 0.10


# ⚠️ 2026-09-24: 예전엔 여기서 ENCODER_SIGN=-1, 857.2를 4채널 공통으로 썼다.
# 뒷바퀴 520 교체·재배선 뒤 그 공식은 전진 거리를 실제의 1/34로 읽어서
# 목표 미도달로 판단, 타임아웃까지 계속 주행하는 상태였다. 공용 모듈로 교체.
dist = center


def min_front(msg):
    best = float('inf')
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min <= r <= msg.range_max):
            continue
        if 0.078 <= r <= 0.084:  # 자기 구조물 대역(hardware_inventory.md)
            continue
        ang = math.degrees(msg.angle_min + i * msg.angle_increment) % 360
        if (ang >= 335 or ang <= 25) and r < best:
            best = r
    return best


def main():
    rclpy.init()
    n = rclpy.create_node('precise_move')
    pub = n.create_publisher(Twist, 'cmd_vel', 1)
    state = {}
    n.create_subscription(Int32MultiArray, '/wheel_encoders',
                           lambda m: state.update(enc=list(m.data)), 5)
    n.create_subscription(LaserScan, '/scan_raw', lambda m: state.update(raw=m), 5)
    t0 = time.time()
    while ('enc' not in state or 'raw' not in state) and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'enc' not in state:
        print('구독 실패'); return 1

    clearance = min_front(state['raw'])
    print(f'전방 여유: {clearance:.2f}m, 목표: {TARGET_M*100:.0f}cm')
    if clearance < TARGET_M + SAFETY_MARGIN_M:
        print('여유 부족 — 중단'); return 1

    start_enc = list(state['enc'])
    hist = []

    print('주행 시작...')
    t_run_start = time.time()
    driving = True
    while True:
        now = time.time()
        d_now = dist(start_enc, state['enc'])
        hist.append((now, d_now))
        hist[:] = [h for h in hist if now - h[0] < 0.3]

        # 2026-08-29 버그 수정: 엔코더는 diff_drive_node 폴링 주기(~60~80ms)
        # 마다만 갱신되는데 이 루프는 ~20ms마다 도니, hist 창이 아직 짧을 때
        # (예: 초반 몇 틱) 두 샘플의 시간차가 거의 0에 가까운 채로 거리만
        # 살짝 달라지면 속도가 폭발적으로 커진다(실측: 24m/s까지 튐 ->
        # 코스팅 예측이 12.8m로 튀어 목표 200cm를 37cm 만에 "달성"으로
        # 오판, 조기정지). 최소 시간창(0.15초) 미만이면 그 추정을 못 믿고
        # v_est=0으로 본다(구동을 계속하는 쪽이 안전 — 코스팅 보정이
        # 없어도 그냥 목표에 못 미쳐 멈추는 방향이라 과주행보다 안전).
        span = hist[-1][0] - hist[0][0] if len(hist) >= 2 else 0.0
        if span >= 0.15:
            v_est = (hist[-1][1] - hist[0][1]) / span
            v_est = max(0.0, min(v_est, 2.0))  # 물리적 상한(안전판)
        else:
            v_est = 0.0

        predicted_final = d_now + COAST_K * v_est

        if driving and predicted_final >= TARGET_M:
            driving = False
            print(f'  t={now-t_run_start:.2f}s d={d_now*100:.1f}cm v_est={v_est:.3f}m/s '
                  f'predicted={predicted_final*100:.1f}cm -> 구동 중단')

        msg = Twist()
        if driving:
            msg.linear.x = SPEED
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.02)

        if not driving and v_est < 0.005 and now - t_run_start > 0.5:
            break
        if now - t_run_start > 6.0:
            print('  시간초과 — 강제 종료'); break

    stop = Twist()
    for _ in range(5):
        pub.publish(stop); time.sleep(0.03)
    time.sleep(0.3)
    rclpy.spin_once(n, timeout_sec=0.3)

    final_d = dist(start_enc, state['enc'])
    print(f'\n=== 최종 이동거리: {final_d*100:.2f}cm '
          f'(목표 {TARGET_M*100:.0f}cm, 오차 {(final_d-TARGET_M)*100:+.1f}cm) ===')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
