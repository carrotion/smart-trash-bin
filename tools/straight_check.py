#!/usr/bin/env python3
"""바닥에서 직진 성능을 잰다. 자이로 적분으로 방향 이탈각, 엔코더로 채널별 슬립.

⚠️ **측정 전 자이로 수신을 확인한다.** 노드를 만들자마자 시작하면 구독이 붙기 전
   구간의 샘플이 통째로 빠져서 **이탈각이 작게 나온다**. 2026-08-19에 이 버그로
   "1회차만 성적이 좋다"는 없는 현상을 쫓을 뻔했다.

⚠️ 바닥에서, 앞뒤로 1.5m 정도 트인 곳. 전진 후 같은 거리 후진해 제자리로 돌아온다.

읽는 법
-------
- **주행중 이탈각**: 명령 구간 동안 누적된 방향 변화. 음수 = 오른쪽으로 휨
- **정지후 총**: 정지 1초 뒤까지 포함. 관성으로 되돌아가는 몫이 여기 들어간다
  (실사용도 버튼 On/Off를 반복하므로 이쪽이 체감에 가깝다)
- **채널별 엔코더**: 바닥에서는 4바퀴가 같은 지면 속도로 구속되므로 카운트가
  비슷해야 정상이다. 유독 다른 채널이 있으면 그 바퀴가 미끄러지거나 끌리는 것

    python3 tools/straight_check.py --trials 4 --speed 0.2
"""
import argparse, json, math, signal, statistics, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--sec', type=float, default=2.5)
    ap.add_argument('--trials', type=int, default=4)
    ap.add_argument('--battery-v', type=float)
    ap.add_argument('--json')
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('straight_check')
    gyro, enc = [], []
    n.create_subscription(Imu, '/imu/data',
        lambda m: gyro.append((time.time(), m.angular_velocity.z)), 50)
    n.create_subscription(Int32MultiArray, '/wheel_encoders',
        lambda m: enc.append((time.time(), list(m.data))), 50)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def drive(v):
        t = Twist(); t.linear.x = float(v); t.angular.z = 0.0; pub.publish(t)
    def stop():
        for _ in range(6): drive(0.0); time.sleep(0.04)
    def spin(sec, v=None, flag=None):
        end = time.time() + sec
        while time.time() < end:
            if flag and flag['v']: return
            if v is not None: drive(v)
            rclpy.spin_once(n, timeout_sec=0.01)
    def yaw(t0, t1):
        seg = [(t, w) for t, w in gyro if t0 <= t <= t1]
        tot = 0.0
        for i in range(1, len(seg)):
            dt = seg[i][0] - seg[i-1][0]
            if 0 < dt < 0.2: tot += seg[i][1] * dt
        return math.degrees(tot), len(seg)

    # ⚠️ 구독 확인 — 위 주석 참고
    d = time.time() + 10
    while time.time() < d and (len(gyro) < 40 or len(enc) < 10):
        rclpy.spin_once(n, timeout_sec=0.02)
    if len(gyro) < 40:
        print(f'❌ 자이로 수신 부족({len(gyro)})'); return 1
    print(f'  수신 확인 — 자이로 {len(gyro)}, 엔코더 {len(enc)}\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    rows = []
    try:
        for k in range(args.trials):
            if flag['v']: break
            spin(1.0, None, flag)
            e0 = enc[-1][1] if enc else None
            t0 = time.time()
            spin(args.sec, args.speed, flag)
            td = time.time(); stop()
            spin(1.0, None, flag)
            e1 = enc[-1][1] if enc else None
            a_dur, _ = yaw(t0, td)
            a_tot, cnt = yaw(t0, time.time())
            de = [abs(e1[i]-e0[i]) for i in range(4)] if (e0 and e1) else None
            rows.append({'trial': k+1, 'during': a_dur, 'total': a_tot,
                         'enc': de, 'samples': cnt})
            es = ('  엔코더 ' + ' '.join(f'{x:>5d}' for x in de)) if de else ''
            print(f'  {k+1}회차  주행중 {a_dur:+6.1f}°  정지후 총 {a_tot:+6.1f}°{es}')
            # 복귀
            spin(args.sec, -args.speed, flag); stop(); spin(1.0, None, flag)
    finally:
        stop()

    if rows:
        tot = [r['total'] for r in rows]
        print(f'\n  {len(tot)}회 평균 {statistics.fmean(tot):+.1f}°  '
              f'표준편차 {statistics.pstdev(tot):.1f}°')
        encs = [r['enc'] for r in rows if r['enc']]
        if encs:
            avg = [statistics.fmean([e[i] for e in encs]) for i in range(4)]
            mx = max(avg) if max(avg) > 0 else 1
            print(f'  채널별 평균 엔코더  ' + ' '.join(f'M{i+1} {avg[i]:.0f}' for i in range(4)))
            print(f'  최대 대비 비율      ' + ' '.join(f'M{i+1} {100*avg[i]/mx:.0f}%' for i in range(4)))
            print('  (바닥에서는 4바퀴가 같은 지면 속도로 구속되므로 비슷해야 정상)')
    if args.json:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'speed': args.speed,
                   'battery_v': args.battery_v, 'rows': rows},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n  저장: {args.json}')
    n.destroy_node(); rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
