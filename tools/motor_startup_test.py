#!/usr/bin/env python3
"""모터별 **기동 지연 차이**와 그로 인한 출발 시 방향 틀어짐을 측정한다.

왜 이 측정인가
--------------
정상 주행 중의 좌우 불균형은 트림이나 폐루프로 잡을 수 있다. 그러나 **출발 순간
0.1~0.5초 사이의 과도구간**은 다르다 — 폐루프가 오차를 감지해 반응하기도 전에
이미 방향이 틀어져 버린다. 그 틀어진 각도는 이후 아무리 잘 달려도 사라지지 않는다.

2026-08-19에 직진 편차 -11°를 트림 전 범위(-0.4~+0.8)로 훑어도 부호조차 안
바뀌었는데, 원인의 일부가 여기일 수 있다.

무엇을 재나
-----------
- **자이로 50Hz**: 출발 후 시간별 요 각속도 -> 과도구간에서 몇 도가 틀어지는가 (피해)
- **엔코더 16.5Hz**: 채널별 누적 카운트 -> 어느 모터가 늦게 뜨는가 (원인)

한 번의 출발은 잡음이 크므로 **여러 번 반복해 출발 시점 기준으로 평균**낸다.

⚠️ 바닥에서, 앞이 2m 정도 트인 곳에서 실행할 것. 1회당 약 0.24m 전진한다.
"""
import argparse, json, math, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('motor_startup_test')
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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--burst', type=float, default=1.2, help='1회 전진 시간(초)')
    ap.add_argument('--cycles', type=int, default=5)
    ap.add_argument('--json')
    args = ap.parse_args()

    r = Runner()
    r.spin(2.0)                       # 구독 확인 (앞 구간 샘플 누락 방지)
    if len(r.gyro) < 40 or len(r.enc) < 10:
        print(f'❌ 수신 부족 (자이로 {len(r.gyro)}, 엔코더 {len(r.enc)})'); return 1
    print(f'  수신 확인 — 자이로 {len(r.gyro)}, 엔코더 {len(r.enc)}\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    starts = []
    try:
        for i in range(args.cycles):
            if flag['v']: break
            r.spin(1.5, None, flag)               # 완전 정지 상태 확보
            t0 = time.time(); starts.append(t0)
            print(f'  {i+1}/{args.cycles} 출발 ... ', end='', flush=True)
            r.spin(args.burst, args.speed, flag)
            r.stop()
            r.spin(1.0, None, flag)
            print('완료')
    finally:
        r.stop()

    if not starts:
        print('측정 없음'); return 1

    # --- 출발 시점 정렬 평균 ---
    BIN = 0.05                          # 50ms 구간
    NB = int(args.burst / BIN)
    print(f'\n=== 출발 후 시간별 요 각속도 (rad/s, {len(starts)}회 평균) ===')
    print(f'  {"시각(ms)":>9} {"각속도":>9} {"누적 각도":>11}')
    cum = 0.0
    prof = []
    for k in range(NB):
        vals = []
        for t0 in starts:
            w = [g for t, g in r.gyro if t0 + k*BIN <= t < t0 + (k+1)*BIN]
            if w: vals.append(sum(w)/len(w))
        if not vals: continue
        avg = sum(vals)/len(vals)
        cum += avg * BIN
        prof.append({'ms': int(k*BIN*1000), 'w': avg, 'cum_deg': math.degrees(cum)})
        print(f'  {int(k*BIN*1000):>9} {avg:>+9.3f} {math.degrees(cum):>+10.2f}°')

    print(f'\n  {args.burst}초 전진 후 누적 이탈 {math.degrees(cum):+.2f}°')

    # --- 채널별 기동 ---
    print(f'\n=== 채널별 누적 엔코더 (출발 시점 대비, {len(starts)}회 평균) ===')
    print(f'  {"시각(ms)":>9} {"M1":>8} {"M2":>8} {"M3":>8} {"M4":>8}')
    ENC_BIN = 0.1
    NE = int(args.burst / ENC_BIN)
    enc_prof = []
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
        enc_prof.append({'ms': int(k*ENC_BIN*1000), 'counts': row})
        print(f'  {int(k*ENC_BIN*1000):>9} ' + ' '.join(f'{x:>8.0f}' for x in row))

    if enc_prof:
        last = enc_prof[-1]['counts']
        mx = max(last) if max(last) > 0 else 1
        print(f'\n  최종 카운트 대비 비율:  ' +
              '  '.join(f'M{i+1} {100*last[i]/mx:.0f}%' for i in range(4)))
        # 각 채널이 최종의 20%에 도달한 시각 = 기동 지연 지표
        print(f'\n  기동 지연 (최종의 20% 도달 시각):')
        for c in range(4):
            thr = last[c] * 0.2
            tms = next((p['ms'] for p in enc_prof if p['counts'][c] >= thr and thr > 0), None)
            print(f'    M{c+1}: ' + (f'{tms} ms' if tms is not None else '판정 불가'))

    if args.json:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'speed': args.speed,
                   'cycles': len(starts), 'yaw_profile': prof,
                   'encoder_profile': enc_prof,
                   'total_yaw_deg': math.degrees(cum)},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n  저장: {args.json}')

    r.n.destroy_node(); rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
