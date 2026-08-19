#!/usr/bin/env python3
"""회전 PWM -> 실제 각속도 특성을 잰다. 스키드 스티어 마찰 변화를 정량 비교하는 용도.

⚠️ **바닥에서, 주변이 트인 곳**에서 실행할 것.
   차체가 실제로 돌아야 자이로에 잡힌다. 도크 위(바퀴 뜬 상태)에서는 바퀴만 헛돌고
   차체는 가만히 있어서 각속도가 0으로 나온다 — 측정 자체가 성립하지 않는다.

⚠️ **배터리로만 측정할 것.** 충전 중에는 단자 전압이 12.6~13.2V로 올라가는데
   DC 모터 회전수는 인가 전압에 거의 비례한다. 그러면 전압 효과와 마찰 효과가
   뒤섞여서 "축간거리를 줄여서 좋아졌는지"를 구분할 수 없다.
   (2026-08-19 현재 전압 센싱이 없어 사후 보정도 불가능하다)

왜 폐루프를 끄나
----------------
자이로 폐루프(turn_kp/turn_ki)가 켜져 있으면 목표 각속도를 맞추려고 PWM을 알아서
조절한다. 그러면 "어느 PWM에서 얼마나 도는가"라는 개루프 특성을 못 잰다.
측정 동안만 끄고, 끝나면 **실행 시점에 읽어둔 원래 값으로** 되돌린다.

비교 기준 (2026-08-17, 축간거리 125mm, 배터리 구동)
    PWM 250 -> 0.187 rad/s
    PWM 400 -> 0.530 rad/s
    PWM 500 -> 0.773 rad/s
축간거리를 108mm로 줄이면 M_scrub ∝ L 이므로 마찰이 13.6% 감소 -> 같은 PWM에서
더 빨라야 한다.

    python3 tools/rotation_calibrate.py
"""

import argparse
import math
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

# diff_drive_node의 _turn_pwm 모델:
#   pwm = min_turn_pwm + (|w|/max_angular_speed) * (max_turn_pwm - min_turn_pwm)
# 목표 PWM에서 역산해 명령할 각속도를 구한다.
MIN_TURN_PWM = 260.0
MAX_TURN_PWM = 570.0
MAX_ANGULAR = 1.0

TARGET_PWMS = [300, 350, 400, 450, 500, 550]
# 저속 구간. Nav2의 실제 운용 범위(max_vel_theta 0.6)가 여기 들어있는데
# 고속 스윕만으로는 이 구간을 전혀 못 본다.
# ⚠️ min_turn_pwm(260)이 하한이라 그 아래는 명령이 0으로 잘린다.
#    측정 동안만 min_turn_pwm=0으로 풀고 끝나면 되돌린다.
LOW_PWMS = [170, 190, 210, 230, 250, 270, 300]
DRIVE_SEC = 2.5
REST_SEC = 2.0


def w_for_pwm(pwm):
    ratio = (pwm - MIN_TURN_PWM) / (MAX_TURN_PWM - MIN_TURN_PWM)
    return max(0.0, min(1.0, ratio)) * MAX_ANGULAR


class Runner(Node):
    def __init__(self):
        super().__init__('rotation_calibrate')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gyro = []
        self.create_subscription(Imu, '/imu/data',
                                 lambda m: self.gyro.append(
                                     (time.time(), m.angular_velocity.z)), 50)

    def drive(self, w):
        t = Twist()
        t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self.drive(0.0)
            time.sleep(0.05)


def remote_params(node, target, names, timeout=5.0):
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request(); req.names = list(names)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    vals = fut.result().values
    if len(vals) != len(names):
        return None
    # ⚠️ 타입을 섞어서 선언해 뒀다: min/max_turn_pwm은 **정수**,
    #    turn_kp/ki와 max_angular_speed는 **실수**다.
    #    실수만 받도록 했다가 읽기가 통째로 실패했다(2026-08-19).
    #    복원할 때도 원래 타입 그대로 돌려줘야 하므로 타입을 함께 보관한다.
    out = {}
    for name, v in zip(names, vals):
        if v.type == 2:      # PARAMETER_INTEGER
            out[name] = int(v.integer_value)
        elif v.type == 3:    # PARAMETER_DOUBLE
            out[name] = float(v.double_value)
        else:
            return None
    return out


def set_params(node, target, values, timeout=5.0):
    cli = node.create_client(SetParameters, f'{target}/set_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, 'no service'
    req = SetParameters.Request()
    req.parameters = [Parameter(k, value=v).to_parameter_msg()
                      for k, v in values.items()]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return False, 'no response'
    bad = [r.reason for r in fut.result().results if not r.successful]
    return (not bad), ('; '.join(bad) if bad else 'ok')


def window_rate(gyro, t0, t1):
    """구간 평균 각속도(rad/s). 가감속 과도구간은 호출부에서 잘라낸다."""
    w = [g for t, g in gyro if t0 <= t <= t1]
    if len(w) < 5:
        return None, 0
    return statistics.fmean(abs(x) for x in w), len(w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--node', default='/diff_drive_node')
    ap.add_argument('--sec', type=float, default=DRIVE_SEC)
    ap.add_argument('--low', action='store_true',
                    help='저속 구간(PWM 170~300) 측정. min_turn_pwm을 잠시 0으로 푼다')
    ap.add_argument('--json',
                    help='결과 저장 경로. 측정 조건(전압/축간거리)과 함께 남겨야 '
                         '다음 측정과 비교할 수 있다 — 2026-08-17 측정은 전압을 '
                         '안 남겨서 비교에 제약이 생겼다')
    ap.add_argument('--battery-v', type=float,
                    help='측정 시점 배터리 전압(V). 모터 회전수는 전압에 거의 '
                         '비례하므로 이게 없으면 비교가 무의미해진다')
    ap.add_argument('--pwms',
                    help='임의 PWM 목록. 예: "160,140,120,100,80,60,40". '
                         'min_turn_pwm을 0으로 풀어 저속까지 명령 가능하게 한다. '
                         '**동작이 멈추는 지점까지 내려가서 경계를 확인할 것** — '
                         '임의로 자른 범위만 재면 그 밖을 모른 채 모델을 만들게 된다')
    args = ap.parse_args()

    rclpy.init()
    node = Runner()
    deadline = time.time() + 10
    while time.time() < deadline and len(node.gyro) < 20:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(node.gyro) < 20:
        print('❌ /imu/data 수신 부족 — IMU 서비스를 확인할 것'); return 1

    names = ['turn_kp', 'turn_ki', 'min_turn_pwm', 'max_turn_pwm',
             'max_angular_speed']
    saved = remote_params(node, args.node, names)
    if saved is None:
        print(f'❌ {args.node}의 현재 파라미터를 못 읽었다. '
              '복원을 보장할 수 없어 중단한다.')
        return 1
    override = {'turn_kp': 0.0, 'turn_ki': 0.0}
    pwms = TARGET_PWMS
    if args.pwms:
        override['min_turn_pwm'] = 0     # 정수 파라미터 — 실수로 주면 거부된다
        pwms = [int(v) for v in args.pwms.split(',')]
    elif args.low:
        override['min_turn_pwm'] = 0
        pwms = LOW_PWMS
    ok, why = set_params(node, args.node, override)
    if not ok:
        print(f'❌ 파라미터 변경 실패: {why}'); return 1
    print(f'적용: {override}')
    print(f'복원할 값: {saved}\n')
    lo = 0.0 if (args.low or args.pwms) else float(saved['min_turn_pwm'])
    hi = float(saved['max_turn_pwm'])
    span = float(saved['max_angular_speed'])

    def cmd_for(pwm):
        return max(0.0, min(1.0, (pwm - lo) / (hi - lo))) * span

    stop_flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: stop_flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: stop_flag.update(v=True))

    rows = []
    try:
        for pwm in pwms:
            if stop_flag['v']:
                break
            w = cmd_for(pwm)
            print(f'  PWM {pwm} (명령 {w:.3f} rad/s) ... ', end='', flush=True)
            t0 = time.time()
            node.drive(w)
            while time.time() - t0 < args.sec and not stop_flag['v']:
                node.drive(w)          # cmd_vel 워치독 대비 재발행
                rclpy.spin_once(node, timeout_sec=0.05)
            t1 = time.time()
            node.stop()
            # 앞 0.8초는 가속 과도구간이라 버린다
            rate, n = window_rate(node.gyro, t0 + 0.8, t1)
            if rate is None:
                print('측정 실패')
            else:
                # 정지 시 자이로 잡음이 약 0.03 rad/s이므로 그 위여야 실제 회전
                mark = '  <-- 안 돎' if rate < 0.06 else ''
                print(f'{rate:.3f} rad/s{mark}')
            if rate:
                rows.append((pwm, w, rate, n))
            rest = time.time()
            while time.time() - rest < REST_SEC and not stop_flag['v']:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop()
        ok, why = set_params(node, args.node, saved)
        print(f'\n폐루프 복원: {"성공" if ok else "실패 — " + why}')
        if not ok:
            print(f'⚠️ 수동 복원: ros2 param set {args.node} turn_kp {saved["turn_kp"]}')

    if rows:
        # 2026-08-17 기준값 (축간거리 125mm)
        ref = {250: 0.187, 400: 0.530, 500: 0.773}
        print(f'\n  {"PWM":>5} {"실측 rad/s":>12} {"08-17":>9} {"변화":>9}')
        for pwm, w, rate, n in rows:
            base = ref.get(pwm)
            if base:
                print(f'  {pwm:>5} {rate:>12.3f} {base:>9.3f} '
                      f'{100*(rate-base)/base:>+8.1f}%')
            else:
                print(f'  {pwm:>5} {rate:>12.3f} {"-":>9} {"-":>9}')
        print('\n  축간거리 125->108mm로 마찰이 13.6% 줄었으므로 같은 PWM에서')
        print('  더 빨라야 한다. 다만 배터리 전압이 다르면 결과가 흔들린다 —')
        print('  두 측정 모두 배터리 구동, 비슷한 충전 상태여야 비교가 유효하다.')
        if args.json:
            import json as _json
            _json.dump({'date': time.strftime('%Y-%m-%d %H:%M'),
                        'battery_v': args.battery_v,
                        'points': [{'pwm': p, 'cmd_rad_s': round(w, 3),
                                    'rad_s': round(r, 3), 'n': n}
                                   for p, w, r, n in rows]},
                       open(args.json, 'w'), ensure_ascii=False, indent=1)
            print(f'\n  저장: {args.json}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
