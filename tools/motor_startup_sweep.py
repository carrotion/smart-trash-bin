#!/usr/bin/env python3
"""모터 기동 지연/요 이탈을 **파워 레벨별로** 스윕 측정한다.

motor_startup_test.py는 speed 한 값만 잰다. 이건 그 측정을 5%~100%(기본, --step로
조절) 구간에 걸쳐 반복해서, "파워가 낮을수록/높을수록 기동 지연이 어떻게
바뀌는가"를 한 번에 모은다. 무부하(도크)/부하(바닥) 두 조건을 각각 돌려서
비교하는 것을 전제로 한다 — 조건(condition)을 인자로 받아 결과 JSON에 남긴다.

⚠️ 매 사이클마다 **왕복**한다(전진 후 같은 시간 후진). 그렇게 안 하면 20단계 x
   반복횟수만큼 로봇이 계속 앞으로만 가서 공간이 아무리 넓어도 부족해진다.
⚠️ burst는 speed와 무관하게 고정(기본 0.8초) — 기동 과도구간(0~0.5초)을 항상
   충분히 담기 위함. 속도가 빠르면 그만큼 더 멀리 가므로 공간을 넉넉히 확보할 것
   (최고속 100%(=MAX_LINEAR) x 0.8초 ≈ 0.8m 편도, 왕복 1.6m).

사용법:
    python3 tools/motor_startup_sweep.py --condition unloaded --json docs/calibration/sweep_unloaded.json
    python3 tools/motor_startup_sweep.py --condition loaded --json docs/calibration/sweep_loaded.json
"""
import argparse, json, math, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

MAX_LINEAR = 1.0  # m/s — diff_drive_node/motor_dashboard와 동일 기준선 (100%)


class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('motor_startup_sweep')
        self.gyro, self.enc = [], []
        self.n.create_subscription(Imu, '/imu/data',
            lambda m: self.gyro.append((time.time(), m.angular_velocity.z)), 50)
        self.n.create_subscription(Int32MultiArray, '/wheel_encoders',
            lambda m: self.enc.append((time.time(), list(m.data))), 50)
        self.pub = self.n.create_publisher(Twist, '/cmd_vel', 10)

    def drive(self, v):
        t = Twist(); t.linear.x = float(v); t.angular.z = 0.0
        self.pub.publish(t)

    def stop(self):
        for _ in range(6):
            self.drive(0.0); time.sleep(0.04)

    def spin(self, sec, v=None, flag=None):
        end = time.time() + sec
        while time.time() < end:
            if flag and flag['v']: return
            if v is not None: self.drive(v)
            rclpy.spin_once(self.n, timeout_sec=0.01)


def measure_level(r, speed, burst, cycles, flag):
    """한 속도 레벨에서 cycles회 반복 측정 -> (yaw_profile, enc_profile, total_yaw_deg)."""
    starts = []
    for i in range(cycles):
        if flag['v']: break
        r.spin(1.2, None, flag)                # 완전 정지 확보
        t0 = time.time(); starts.append(t0)
        r.spin(burst, speed, flag)             # 전진(측정 구간)
        r.stop()
        # ⚠️ 2026-08-21: 여기가 짧으면(예전 0.6초) 전진 정지 시 관성이 덜 가라앉은
        # 채로 후진이 시작돼서, 후진 구간(왕복 복귀용, 비측정)에 전진의 잔류
        # 움직임이 섞여든다. 완전 정지 확보용 1.2초와 맞춰서 넉넉히 준다.
        r.spin(1.2, None, flag)
        r.spin(burst, -speed, flag)            # 후진(왕복 복귀, 측정 안 함)
        r.stop()
        r.spin(1.2, None, flag)
    if not starts:
        return None

    BIN = 0.05
    NB = int(burst / BIN)
    cum = 0.0
    yaw_profile = []
    for k in range(NB):
        vals = []
        for t0 in starts:
            w = [g for t, g in r.gyro if t0 + k*BIN <= t < t0 + (k+1)*BIN]
            if w: vals.append(sum(w)/len(w))
        if not vals: continue
        avg = sum(vals)/len(vals)
        cum += avg * BIN
        yaw_profile.append({'ms': int(k*BIN*1000), 'w': avg, 'cum_deg': math.degrees(cum)})

    ENC_BIN = 0.1
    NE = int(burst / ENC_BIN)
    enc_profile = []
    for k in range(NE):
        acc = [[] for _ in range(4)]
        for t0 in starts:
            base = None
            for t, v in r.enc:
                if t <= t0: base = v
            if base is None: continue
            w = [v for t, v in r.enc if t0 + k*ENC_BIN <= t < t0 + (k+1)*ENC_BIN]
            if not w: continue
            for c in range(min(4, len(w[-1]))):
                acc[c].append(abs(w[-1][c] - base[c]))
        if not any(acc): continue
        row = [sum(a)/len(a) if a else 0 for a in acc]
        enc_profile.append({'ms': int(k*ENC_BIN*1000), 'counts': row})

    latency_ms = [None, None, None, None]
    final_counts = enc_profile[-1]['counts'] if enc_profile else [0, 0, 0, 0]
    for c in range(4):
        thr = final_counts[c] * 0.2
        tms = next((p['ms'] for p in enc_profile if p['counts'][c] >= thr and thr > 0), None)
        latency_ms[c] = tms

    return {
        'speed': speed, 'cycles': len(starts),
        'total_yaw_deg': math.degrees(cum),
        'yaw_profile': yaw_profile,
        'encoder_profile': enc_profile,
        'final_counts': final_counts,
        'latency_ms': latency_ms,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--condition', required=True, choices=['unloaded', 'loaded'],
                    help='무부하(도크)인지 부하(바닥)인지 — 결과에 기록됨')
    ap.add_argument('--charging', action='store_true',
                    help='충전 중이면 표시(결과에 기록, 기본은 아니라고 가정)')
    ap.add_argument('--min-pct', type=float, default=5.0)
    ap.add_argument('--max-pct', type=float, default=100.0)
    ap.add_argument('--step-pct', type=float, default=5.0)
    ap.add_argument('--burst', type=float, default=0.8, help='레벨당 편도 구동 시간(초)')
    ap.add_argument('--cycles', type=int, default=3, help='레벨당 반복 횟수')
    ap.add_argument('--battery-v', type=float)
    ap.add_argument('--json', required=True)
    args = ap.parse_args()

    levels = []
    p = args.min_pct
    while p <= args.max_pct + 1e-9:
        levels.append(round(p, 1))
        p += args.step_pct

    r = Runner()
    r.spin(2.0)
    if len(r.gyro) < 40 or len(r.enc) < 10:
        print(f'❌ 수신 부족 (자이로 {len(r.gyro)}, 엔코더 {len(r.enc)})'); return 1
    print(f'수신 확인 — 자이로 {len(r.gyro)}, 엔코더 {len(r.enc)}')
    print(f'조건: {args.condition}{" (충전 중)" if args.charging else " (충전 안 함)"}, '
          f'{len(levels)}단계 x {args.cycles}회, burst {args.burst}s\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    results = []
    try:
        for pct in levels:
            if flag['v']: break
            speed = MAX_LINEAR * pct / 100.0
            print(f'  {pct:>5.1f}% (speed {speed:.3f} m/s) ... ', end='', flush=True)
            res = measure_level(r, speed, args.burst, args.cycles, flag)
            if res is None:
                print('중단')
                break
            results.append({'pct': pct, **res})
            lat = res['latency_ms']
            latstr = ' '.join(f'M{i+1}={lat[i]}ms' if lat[i] is not None else f'M{i+1}=-'
                              for i in range(4))
            print(f'요이탈 {res["total_yaw_deg"]:+6.2f}°  지연[{latstr}]')
    finally:
        r.stop()

    json.dump({
        'date': time.strftime('%Y-%m-%d %H:%M'),
        'condition': args.condition,
        'charging': args.charging,
        'battery_v': args.battery_v,
        'burst': args.burst, 'cycles': args.cycles,
        'levels': results,
    }, open(args.json, 'w'), ensure_ascii=False, indent=1)
    print(f'\n저장: {args.json}')

    r.n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
