#!/usr/bin/env python3
"""정속 주행 후 정지 명령을 보내고, 관성으로 얼마나 더 굴러가는지(코스팅
거리)를 속도별로 측정해서 `docs/calibration/coast_sweep.csv`에 누적 기록한다.

배경: `tools/precise_move.py`의 코스팅 보정 모델(`coast = COAST_K * v`)을
만들기 위한 데이터 수집 도구. 2026-08-29에 13개 실측(v_at_stop 0.29~0.96m/s)
으로 선형모델이 제곱/지수 모델보다 더 잘 맞는 것을 확인했다 — 자세한 피팅
비교는 그날 세션 기록 참고. 기체·바닥·시점이 달라지면(마모, 재수리 등)
재측정해서 `tools/precise_move.py`의 COAST_K를 갱신할 것.

⚠️ 소프트웨어 가속램프는 감속에 적용 안 됨(diff_drive_node.py 확인 완료) —
여기서 측정하는 코스팅은 순수 물리적 관성이다.
⚠️ CRUISE_S를 너무 짧게 주면(특히 높은 속도) 가속 램프가 안 끝난 채로
정속 구간을 측정하게 돼 데이터가 오염된다 — 명령속도가 높을수록
CRUISE_S를 넉넉히 줄 것(예: 0.8m/s면 최소 1.0초).

사용법:
    python3 tools/coast_sweep.py <속도m/s> [정속시간s, 기본 1.0]
    예: python3 tools/coast_sweep.py 0.3 0.9
"""
import rclpy, math, time, sys, csv, os
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from wheel_config import left_right  # 채널 매핑/부호/카운트당거리 단일 출처

SPEED = float(sys.argv[1])
CRUISE_S = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0


# ⚠️ 2026-09-24: 예전엔 (M1,M2)=왼쪽으로 묶고 부호/카운트를 공통으로 썼다 —
# 520 교체·재배선 뒤 좌우 묶음과 부호가 전부 틀려 공용 모듈로 교체.
lr_dist = left_right


def min_wide_front(msg):
    """335~25도(좁은 정면)보다 넓은 300~60도 — 직진 폐루프 드리프트를
    감안해 좌우로 여유를 더 본다(2026-08-29 실측: 왼쪽으로 3~4% 드리프트)."""
    best = float('inf')
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min <= r <= msg.range_max):
            continue
        if 0.078 <= r <= 0.084:
            continue
        ang = math.degrees(msg.angle_min + i * msg.angle_increment) % 360
        if (ang >= 300 or ang <= 60) and r < best:
            best = r
    return best


def main():
    rclpy.init()
    n = rclpy.create_node('coast_sweep')
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

    clearance = min_wide_front(state['raw'])
    print(f'넓은 전방(300~60도) 여유: {clearance:.2f}m')
    if clearance < 0.6:
        print('여유 부족 — 중단'); return 1

    start_enc = list(state['enc'])
    hist = []
    tend = time.time() + CRUISE_S
    while time.time() < tend:
        now = time.time()
        dl, dr = lr_dist(start_enc, state['enc'])
        d_now = (dl + dr) / 2.0
        hist.append((now, d_now))
        hist[:] = [h for h in hist if now - h[0] < 0.25]
        msg = Twist(); msg.linear.x = SPEED
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.02)

    if len(hist) >= 2:
        v_at_stop = (hist[-1][1] - hist[0][1]) / max(hist[-1][0] - hist[0][0], 1e-3)
    else:
        v_at_stop = SPEED

    at_stop_enc = list(state['enc'])
    stop = Twist()
    for _ in range(5):
        pub.publish(stop); time.sleep(0.03)

    tmon = time.time() + 1.5
    last_d = 0.0
    still_count = 0
    while time.time() < tmon:
        rclpy.spin_once(n, timeout_sec=0.02)
        dl, dr = lr_dist(at_stop_enc, state['enc'])
        d = (dl + dr) / 2.0
        still_count = still_count + 1 if abs(d - last_d) < 0.001 else 0
        last_d = d
        if still_count > 15:  # 약 0.3초 이상 변화 없음 = 완전정지
            break

    dl_c, dr_c = lr_dist(at_stop_enc, state['enc'])
    coast_dist = (dl_c + dr_c) / 2.0
    dl_cr, dr_cr = lr_dist(start_enc, at_stop_enc)
    cruise_dist = (dl_cr + dr_cr) / 2.0
    dl_total, dr_total = lr_dist(start_enc, state['enc'])

    print(f'명령속도={SPEED} 정속거리={cruise_dist*100:.1f}cm '
          f'v_at_stop={v_at_stop:.3f}m/s 코스팅={coast_dist*100:.2f}cm')
    print(f'좌우 비교(전체 구간) 좌={dl_total*100:.1f}cm 우={dr_total*100:.1f}cm '
          f'차이={(dr_total-dl_total)*100:+.1f}cm')

    fpath = 'docs/calibration/coast_sweep.csv'
    newfile = not os.path.exists(fpath)
    with open(fpath, 'a', newline='') as f:
        w = csv.writer(f)
        if newfile:
            w.writerow(['cmd_speed', 'cruise_s', 'v_at_stop', 'coast_dist_m'])
        w.writerow([SPEED, CRUISE_S, v_at_stop, coast_dist])
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
