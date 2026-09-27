#!/usr/bin/env python3
"""바닥에서 명령 속도별 실제 직진 속도를 잰다 — 전진 후 같은 속도로 후진해 제자리 복귀.

왜 필요한가(2026-09-24, docs/todo.md #1): `_speed_to_pwm`은 속도/1.5×1200으로
PWM을 계산하고 120 미만을 120으로 끌어올린다. 그래서 Nav2 `max_vel_x`를 0.04까지
낮춰도 0.15 m/s 이하 명령은 전부 같은 PWM이 된다. 실제 속도가 명령과 얼마나
다른지, 그리고 저속 명령끼리 구분되는지 확인한다.

실제 속도는 두 가지로 따로 잰다:
  - 엔코더: 정속 구간(출발 0.3초 이후) 앞/뒤 축 평균 속도
  - 라이다: 정지 상태 전/후 정면 거리(±5°) 중앙값 차이 = 지면 기준 실제 이동량
    (바퀴가 미끄러져도 속지 않는다. 엔코더 총 이동량과 비교해 슬립을 본다)

사용법 (바닥, 앞뒤 여유 확보, 다른 사람이 조종하지 않는 상태):
    python3 tools/speed_floor_sweep.py [--speeds 0.04,0.08,0.15,0.2,0.3] [--sec 1.0]
결과: docs/calibration/speed_floor_sweep_YYYYMMDD_HHMM.csv
"""
import argparse, csv, math, os, statistics, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String

from wheel_config import wheel_distances, FRONT, REAR

MAX_REAL_SPEED = 0.8   # m/s — 여유거리 계산용 보수적 상한(0.2 명령에 0.59 실측)
COAST_MARGIN = 0.35    # m — 정지 후 관성 + 안전 여유
LIDAR_X = 0.054        # base_link 기준 라이다 x (docs/hardware_inventory.md)
HALF = 0.11            # 차체 반폭/반길이 (footprint ±0.11)
CORRIDOR = 0.10        # 좌우로 더 보는 여유 — 직진 중 휘는 것 감안


def corridor_clearance(msg, forward=True, ignore_within=0.0, corridor=CORRIDOR):
    """주행 방향 통로(차체 폭 + 좌우 corridor) 안에서 차체 끝부터 가장 가까운 점까지.

    각도 부채꼴로 보면 차체 안 구조물(케이블 등, 0.13m)이나 옆쪽 물체가 걸려
    오판한다(2026-09-24 첫 시도: 후방 0.13m로 중단 — 대부분 차체 안쪽이었다).
    """
    best = float('inf')
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min <= r <= msg.range_max):
            continue
        a = msg.angle_min + i * msg.angle_increment
        x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
        if abs(x) <= HALF + 0.005 and abs(y) <= HALF + 0.005:
            continue   # 차체 안 = 자기 구조물
        if abs(y) > HALF + corridor:
            continue
        gap = (x - HALF) if forward else (-HALF - x)
        if gap > ignore_within:
            best = min(best, gap)
    return best


def sector(msg, center_deg, half_deg, fn=min):
    vals = []
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min <= r <= msg.range_max) or r < 0.12:  # 자기 구조물 제외
            continue
        ang = math.degrees(msg.angle_min + i * msg.angle_increment) % 360
        d = (ang - center_deg + 180) % 360 - 180
        if abs(d) <= half_deg:
            vals.append(r)
    return fn(vals) if vals else float('inf')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default='0.04,0.08,0.15,0.2,0.3')
    ap.add_argument('--sec', type=float, default=1.0)
    # 2026-09-24: 정후방 차체 끝 1~3.5cm에 드문드문 반사(166~202°) — 사용자가 뒤가 비어
    # 있음을 확인 → 로봇 뒤로 튀어나온 부착물(케이블 추정). 사람이 확인했을 때만 쓸 것
    ap.add_argument('--rear-ignore-within', type=float, default=0.0,
                    help='차체 뒤끝에서 이 거리(m) 이내 후방 반사는 무시(사람 확인 후)')
    args = ap.parse_args()
    speeds = [float(s) for s in args.speeds.split(',')]

    rclpy.init()
    n = rclpy.create_node('speed_floor_sweep')
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    st = {'enc_hist': []}

    def on_enc(m):
        st['enc'] = list(m.data)
        st['enc_hist'].append((time.time(), list(m.data)))
    n.create_subscription(Int32MultiArray, '/wheel_encoders', on_enc, 20)
    # 상시 라이다 서비스는 /scan만 발행(/scan_raw는 매핑 런치 때만). 자기 구조물은
    # sector()가 0.12m 미만을 버려서 걸러진다.
    n.create_subscription(LaserScan, '/scan', lambda m: st.update(scan=m), 5)
    n.create_subscription(String, '/battery_tier', lambda m: st.update(tier=m.data), 5)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.02)

    t0 = time.time()
    while ('enc' not in st or 'scan' not in st) and time.time() - t0 < 8:
        spin(0.1)
    if 'enc' not in st or 'scan' not in st:
        print('❌ /wheel_encoders 또는 /scan 미수신'); return 1

    def stop():
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)

    def front_median():
        vals = []
        for _ in range(8):   # 약 0.8초, 스캔 여러 장의 중앙값
            spin(0.1)
            vals.append(sector(st['scan'], 0, 5, fn=statistics.median))
        vals = [v for v in vals if math.isfinite(v)]
        return statistics.median(vals) if vals else float('nan')

    def drive(v):
        """v로 args.sec 동안 주행 → 정지·정착. (엔코더 총거리, 정속 구간 속도)."""
        e0 = list(st['enc'])
        t_start = time.time()
        st['enc_hist'] = []
        while time.time() - t_start < args.sec:
            m = Twist(); m.linear.x = v; pub.publish(m); spin(0.04)
        stop(); spin(1.0)
        steady = [(t, e) for t, e in st['enc_hist'] if t - t_start >= 0.3 and t - t_start <= args.sec]
        v_enc = v_front = v_rear = float('nan')
        if len(steady) >= 2:
            (ta, ea), (tb, eb) = steady[0], steady[-1]
            d = wheel_distances(ea, eb)
            dt = tb - ta
            v_front = sum(d[i] for i in FRONT) / len(FRONT) / dt
            v_rear = sum(d[i] for i in REAR) / len(REAR) / dt
            v_enc = (v_front + v_rear) / 2
        dist = sum(wheel_distances(e0, st['enc'])) / 4
        return dist, v_enc, v_front, v_rear

    rows = []
    for v in speeds:
        # 전진한 만큼 후진해 돌아오므로 후방은 관성 여유만 있으면 된다
        need = MAX_REAL_SPEED * args.sec + COAST_MARGIN
        f_clear = corridor_clearance(st['scan'], True)
        r_clear = corridor_clearance(st['scan'], False, args.rear_ignore_within)
        print(f'\n[명령 {v:.2f} m/s] 차체 끝부터 통로 여유 전방 {f_clear:.2f} (필요 {need:.2f}) / '
              f'후방 {r_clear:.2f} m (필요 {COAST_MARGIN:.2f})')
        if f_clear < need or r_clear < COAST_MARGIN:
            print('  ❌ 여유 부족 — 여기서 중단'); break
        if st.get('tier', 'ok') != 'ok':
            print(f'  ❌ 배터리 {st["tier"]} — 중단'); break

        lf0 = front_median()
        d_fwd, v_enc, vf, vr = drive(v)
        lf1 = front_median()
        lidar_d = lf0 - lf1
        d_back, *_ = drive(-v)
        slip = (d_fwd - lidar_d) / d_fwd * 100 if d_fwd > 0.01 and math.isfinite(lidar_d) else float('nan')
        print(f'  정속 속도(엔코더) {v_enc:.3f} m/s = 명령의 {v_enc / v:.1f}배 '
              f'(앞축 {vf:.3f} / 뒤축 {vr:.3f})')
        print(f'  전진 총거리 엔코더 {d_fwd:.3f} m / 라이다 {lidar_d:.3f} m (슬립 {slip:+.0f}%), '
              f'후진 복귀 {d_back:+.3f} m')
        rows.append([v, args.sec, round(v_enc, 4), round(vf, 4), round(vr, 4),
                     round(d_fwd, 4), round(lidar_d, 4), round(d_back, 4)])

    stop()
    if rows:
        os.makedirs('docs/calibration', exist_ok=True)
        path = time.strftime('docs/calibration/speed_floor_sweep_%Y%m%d_%H%M.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['cmd_mps', 'sec', 'v_enc_steady', 'v_front', 'v_rear',
                        'fwd_enc_m', 'fwd_lidar_m', 'back_enc_m'])
            w.writerows(rows)
        print(f'\n저장: {path}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
