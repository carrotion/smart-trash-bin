#!/usr/bin/env python3
"""직진 트림(`straight_trim`)을 자이로로 실측한다.

원리
----
직진 명령 중에 로봇이 휘면 **자이로에 요 회전이 잡힌다.** 그걸 적분하면 총 방향
이탈각(도)이 나온다. 자로 잴 필요가 없고, 반복 측정이 쉽다.

    left  *= (1 - trim)
    right *= (1 + trim)

trim이 양수면 왼쪽을 줄이고 오른쪽을 키운다. 이탈각의 **부호가 바뀌는 지점**이
최적값이다.

⚠️ 정지 시 편차도 함께 잰다. CLAUDE.md에 기록된 통찰: 총 편차는 "주행 중
   편향"(trim으로 보정됨)과 "정지 시 관성으로 원래 기계 얼라인먼트로 복귀하는
   편향"(trim과 무관, 항상 존재)의 합이다. 실사용도 버튼 On/Off를 반복하므로
   **정지 후 최종 방향**이 실사용에 가까운 지표다.

⚠️ **부호가 뒤집혔을 가능성까지 보려면 음수부터 훑을 것.** 임의로 자른 범위만
   재면 최적값이 그 밖에 있어도 모른다(cliff test 원칙).

공간
----
전진 0.2m/s x 2.5초 = 0.5m 이동 후 같은 거리 후진해서 제자리로 돌아온다.
**앞뒤로 1.5m 정도** 트여 있으면 된다.

    python3 tools/straight_trim_calibrate.py
    python3 tools/straight_trim_calibrate.py --trims "-0.4,-0.2,0,0.2,0.4,0.6"
"""

import argparse
import json
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

SPEED = 0.2
DRIVE_SEC = 2.5
SETTLE_SEC = 1.0


class Runner(Node):
    def __init__(self):
        super().__init__('straight_trim_calibrate')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gyro = []
        self.create_subscription(Imu, '/imu/data',
                                 lambda m: self.gyro.append(
                                     (time.time(), m.angular_velocity.z)), 50)

    def drive(self, v):
        t = Twist()
        t.linear.x = float(v)
        # ⚠️ angular.z는 반드시 정확히 0.0이어야 한다 —
        #    diff_drive_node는 w == 0.0일 때만 트림을 적용한다.
        t.angular.z = 0.0
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self.drive(0.0)
            time.sleep(0.05)

    def spin_for(self, sec, v=None, flag=None):
        end = time.time() + sec
        while time.time() < end:
            if flag and flag['v']:
                return
            if v is not None:
                self.drive(v)       # cmd_vel 워치독 대비 재발행
            rclpy.spin_once(self, timeout_sec=0.05)


def yaw_change(gyro, t0, t1):
    """구간 자이로 적분 -> 방향 변화(도)."""
    seg = [(t, w) for t, w in gyro if t0 <= t <= t1]
    if len(seg) < 5:
        return None
    total = 0.0
    for i in range(1, len(seg)):
        dt = seg[i][0] - seg[i - 1][0]
        if 0 < dt < 0.2:
            total += seg[i][1] * dt
    return math.degrees(total)


def get_params(node, target, names, timeout=5.0):
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request(); req.names = list(names)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    out = {}
    for name, v in zip(names, fut.result().values):
        if v.type == 2:
            out[name] = int(v.integer_value)
        elif v.type == 3:
            out[name] = float(v.double_value)
        else:
            return None
    return out


def set_params(node, target, values, timeout=5.0):
    cli = node.create_client(SetParameters, f'{target}/set_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    req = SetParameters.Request()
    req.parameters = [Parameter(k, value=v).to_parameter_msg()
                      for k, v in values.items()]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return (fut.done() and fut.result() is not None
            and all(r.successful for r in fut.result().results))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--node', default='/diff_drive_node')
    ap.add_argument('--trims', default='-0.4,-0.2,0.0,0.2,0.4,0.6,0.8')
    ap.add_argument('--speed', type=float, default=SPEED)
    ap.add_argument('--battery-v', type=float)
    ap.add_argument('--json')
    args = ap.parse_args()

    trims = [float(v) for v in args.trims.split(',')]

    rclpy.init()
    node = Runner()
    node.spin_for(2.0)
    if len(node.gyro) < 20:
        print('❌ /imu/data 수신 부족'); return 1

    saved = get_params(node, args.node, ['straight_trim'])
    if saved is None:
        print('❌ straight_trim을 못 읽었다. 복원을 보장할 수 없어 중단.'); return 1
    print(f'현재 트림: {saved["straight_trim"]}\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    rows = []
    try:
        for tr in trims:
            if flag['v']:
                break
            if not set_params(node, args.node, {'straight_trim': tr}):
                print(f'  trim {tr}: 설정 실패'); continue
            time.sleep(0.3)
            print(f'  trim {tr:>+5.2f}  전진 ... ', end='', flush=True)

            t0 = time.time()
            node.spin_for(DRIVE_SEC, args.speed, flag)
            t_drive = time.time()
            node.stop()
            node.spin_for(SETTLE_SEC, None, flag)
            t_end = time.time()

            during = yaw_change(node.gyro, t0, t_drive)
            total = yaw_change(node.gyro, t0, t_end)
            if during is None or total is None:
                print('측정 실패')
            else:
                print(f'주행중 {during:+6.1f}°   정지후 총 {total:+6.1f}°')
                rows.append((tr, during, total))

            # 제자리로 복귀 (측정 안 함)
            node.spin_for(DRIVE_SEC, -args.speed, flag)
            node.stop()
            node.spin_for(1.0, None, flag)
    finally:
        node.stop()
        ok = set_params(node, args.node, saved)
        print(f'\n트림 복원: {"성공" if ok else "❌ 실패 — 수동 확인 필요"}')

    if rows:
        print(f'\n  {"trim":>6} {"주행중":>9} {"정지후 총":>11}  판정')
        for tr, during, total in rows:
            mark = ''
            if abs(total) < 3:
                mark = '  ✅ 거의 직진'
            elif total > 0:
                mark = '  왼쪽으로 휨'
            else:
                mark = '  오른쪽으로 휨'
            print(f'  {tr:>+6.2f} {during:>+8.1f}° {total:>+10.1f}°{mark}')

        # 부호가 바뀌는 구간에서 선형 보간으로 최적값 추정
        best = None
        for i in range(1, len(rows)):
            a, b = rows[i - 1], rows[i]
            if a[2] == 0:
                best = a[0]; break
            if a[2] * b[2] < 0:      # 부호 전환
                f = abs(a[2]) / (abs(a[2]) + abs(b[2]))
                best = a[0] + (b[0] - a[0]) * f
                break
        print()
        if best is not None:
            print(f'  ✅ 이탈각 0 교점: trim ≈ {best:+.2f}')
            print('     이 값으로 다시 재서 확인할 것(선형 보간이므로 근사치).')
        else:
            print('  ⚠️ 시험 범위 안에서 부호가 바뀌지 않았다.')
            print('     최적값이 범위 밖이다 — 범위를 넓혀 다시 잴 것(cliff test).')

    if args.json:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'),
                   'battery_v': args.battery_v, 'speed': args.speed,
                   'rows': [{'trim': t, 'during_deg': d, 'total_deg': o}
                            for t, d, o in rows]},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n  저장: {args.json}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
