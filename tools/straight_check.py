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

# ⚠️ 2026-09-24: 뒷바퀴 370->520 교체로 채널별 카운트/회전이 달라짐 —
# 원시 카운트를 그대로 비교하면 뒤가 앞보다 ~2.9배 많이 나오는 게 정상인데
# "미끄러짐"으로 오판하게 된다. 미터로 환산해서 비교(값은 공용 모듈).
from wheel_config import METERS_PER_COUNT

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--sec', type=float, default=2.5)
    ap.add_argument('--trials', type=int, default=4)
    ap.add_argument('--battery-v', type=float)
    ap.add_argument('--json')
    ap.add_argument('--allow-other-publishers', action='store_true')
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

    # 다른 /cmd_vel 발행자가 있으면 시험 중 누가 버튼을 눌러 명령이 섞일 수 있다
    # — 예방 차원의 확인이다. ⚠️ 정정(2026-09-24): 이날 "M4 26%, 이탈 -51°"를
    # 대시보드 혼입 탓으로 적었으나 틀렸다. 대기 중 대시보드는 아무것도 발행하지
    # 않고(15초간 0개 실측), 진짜 원인은 diff_drive 시작 시 자이로 바이어스가
    # -0.438 rad/s로 잘못 잡힌 것이었다(재시작으로 우연히 사라짐).
    # 자세한 경위: docs/hardware_change_procedure.md 부록 B.
    others = [i.node_name for i in n.get_publishers_info_by_topic('/cmd_vel')
              if i.node_name != 'straight_check']
    if others and not args.allow_other_publishers:
        print(f'❌ /cmd_vel에 다른 발행자가 있다: {others}')
        print('   시험 중 누가 조종하면 명령이 섞인다. 확실히 하려면 끌 것:')
        print('   sudo systemctl stop smart-trash-motor-dashboard smart-trash-map-dashboard')
        print('   (무시하고 진행하려면 --allow-other-publishers)')
        n.destroy_node(); rclpy.shutdown(); return 1

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
            # 복귀 — ⚠️ 이것도 **측정한다.**
            # 좌우 편향(추력/정렬)은 전진과 후진에서 **방향이 뒤집혀** 나타난다:
            #   ω = (v_오른쪽 - v_왼쪽)/W 이므로 왼쪽이 강하면 전진 ω<0, 후진 ω>0.
            # 따라서 두 방향의 **크기가 같으면 단순 편향**이고, 다르면 다른 원인이
            # 섞여 있다는 뜻이다. 후진을 안 재면 그 구분을 못 한다(2026-08-20 보완).
            b0 = time.time()
            spin(args.sec, -args.speed, flag)
            bd = time.time(); stop(); spin(1.0, None, flag)
            b_dur, _ = yaw(b0, bd)
            b_tot, _ = yaw(b0, time.time())
            rows[-1]['back_during'] = b_dur
            rows[-1]['back_total'] = b_tot
            print(f'         후진   주행중 {b_dur:+6.1f}°  정지후 총 {b_tot:+6.1f}°')
    finally:
        stop()

    if rows:
        tot = [r['total'] for r in rows]
        print(f'\n  전진 {len(tot)}회 평균 {statistics.fmean(tot):+.1f}°  '
              f'표준편차 {statistics.pstdev(tot):.1f}°')
        bt = [r['back_total'] for r in rows if 'back_total' in r]
        if bt:
            fm, bm = statistics.fmean(tot), statistics.fmean(bt)
            print(f'  후진 {len(bt)}회 평균 {bm:+.1f}°  '
                  f'표준편차 {statistics.pstdev(bt):.1f}°')
            print()
            if fm * bm < 0:
                sym = 100 * abs(abs(fm) - abs(bm)) / max(abs(fm), abs(bm), 1e-9)
                print(f'  부호가 반대 -> 좌우 편향의 전형적 신호')
                print(f'  크기 차이 {sym:.0f}%  '
                      + ('(대칭 — 단순 편향)' if sym < 30
                         else '(비대칭 — 다른 원인이 섞여 있다)'))
            else:
                print(f'  ⚠️ 전진·후진이 같은 부호다. 좌우 편향만으로는 설명 안 된다 —')
                print(f'     자이로 바이어스나 바닥 기울기 같은 방향 무관 요인 의심.')
        encs = [r['enc'] for r in rows if r['enc']]
        if encs:
            avg = [statistics.fmean([e[i] for e in encs]) for i in range(4)]
            # 원시 카운트가 아니라 실제 이동거리(m)로 환산해서 비교 — 앞/뒤가
            # 카운트/회전이 달라 원시값은 애초에 단위가 다르다(주석 참고).
            avg_m = [avg[i] * METERS_PER_COUNT[i] for i in range(4)]
            mx_m = max(avg_m) if max(avg_m) > 0 else 1
            print(f'  채널별 평균 엔코더(원시)  ' + ' '.join(f'M{i+1} {avg[i]:.0f}' for i in range(4)))
            print(f'  채널별 평균 이동거리(m)   ' + ' '.join(f'M{i+1} {avg_m[i]:.3f}' for i in range(4)))
            print(f'  최대 대비 비율            ' + ' '.join(f'M{i+1} {100*avg_m[i]/mx_m:.0f}%' for i in range(4)))
            print('  (바닥에서는 4바퀴가 같은 지면 거리를 이동하므로 m 기준으로 비슷해야 정상)')
    if args.json:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'speed': args.speed,
                   'battery_v': args.battery_v, 'rows': rows},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n  저장: {args.json}')
    n.destroy_node(); rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
