#!/usr/bin/env python3
"""모터 구동이 지자기센서에 주는 간섭을 정량 측정한다.

실험 설계
---------
로봇을 **도크에 올려 바퀴를 띄운 채로** 돌린다. 그러면 차체가 회전하지 않으므로
방위가 고정되고, 관측되는 자기장 변화는 **순수하게 모터 간섭**만 남는다.
바닥에서 돌리면 회전에 의한 변화와 뒤섞여 구분할 수 없다.

⚠️ 회전 폐루프를 잠시 끈다(turn_kp=0, turn_ki=0). 바퀴가 떠 있으면 자이로가 0을
   유지해서 적분항이 한계까지 감기고 PWM이 최대로 치솟는다 — 그러면 어떤 구동
   세기에서 얼마나 간섭이 생기는지 대응이 안 된다. 끝나면 원래 값으로 되돌린다.

측정 결과는 "명령 세기별 방위 오차(도)"로 낸다. 그게 실제로 쓸 수 있는 숫자다.

사용법
------
    python3 tools/mag_motor_interference.py --out /tmp/maginterf.json
"""

import argparse
import json
import math
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, MagneticField
from rcl_interfaces.srv import GetParameters, SetParameters

# (라벨, linear.x, angular.z, 지속시간초)
PHASES = [
    ('기준(정지)',        0.00, 0.00, 8.0),
    ('회전 0.3 rad/s',    0.00, 0.30, 6.0),
    ('정지',              0.00, 0.00, 4.0),
    ('회전 0.6 rad/s',    0.00, 0.60, 6.0),
    ('정지',              0.00, 0.00, 4.0),
    ('회전 1.0 rad/s',    0.00, 1.00, 6.0),
    ('정지',              0.00, 0.00, 4.0),
    ('직진 0.10 m/s',     0.10, 0.00, 6.0),
    ('정지',              0.00, 0.00, 4.0),
    ('직진 0.15 m/s',     0.15, 0.00, 6.0),
    ('기준(정지, 사후)',  0.00, 0.00, 8.0),
]


class Runner(Node):
    def __init__(self):
        super().__init__('mag_motor_interference')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mag = []
        self.gyro = []
        self.create_subscription(MagneticField, '/imu/mag', self._on_mag, 50)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 50)

    def _on_mag(self, m):
        self.mag.append((time.time(),
                         m.magnetic_field.x * 1e6,
                         m.magnetic_field.y * 1e6,
                         m.magnetic_field.z * 1e6))

    def _on_imu(self, m):
        self.gyro.append((time.time(), m.angular_velocity.z))

    def drive(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def stop(self):
        """정지 명령은 여러 번 보낸다 — 한 번 유실되면 계속 도는 것보다 낫다."""
        for _ in range(5):
            self.drive(0.0, 0.0)
            time.sleep(0.05)


def set_remote_params(node, target, values, timeout=5.0):
    """다른 노드의 파라미터를 바꾼다. 성공 여부를 반드시 확인해서 반환한다."""
    cli = node.create_client(SetParameters, f'{target}/set_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, f'{target}/set_parameters 서비스 없음'
    req = SetParameters.Request()
    req.parameters = [Parameter(k, value=v).to_parameter_msg()
                      for k, v in values.items()]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return False, '응답 없음'
    bad = [r.reason for r in fut.result().results if not r.successful]
    return (not bad), ('; '.join(bad) if bad else 'ok')


def get_remote_params(node, target, names, timeout=5.0):
    """복원할 값을 **실행 시점에 읽어온다.** 하드코딩해 두면 나중에 게인을 튜닝했을 때
    엉뚱한 값으로 되돌려 놓고도 아무도 모른다."""
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = list(names)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    vals = fut.result().values
    if len(vals) != len(names):
        return None
    out = {}
    for name, v in zip(names, vals):
        if v.type != 3:          # PARAMETER_DOUBLE
            return None
        out[name] = v.double_value
    return out


def window(samples, t0, t1):
    return [s for s in samples if t0 <= s[0] <= t1]


def analyze(phases, out_path):
    print(f'\n{"구간":<20}{"수평µT":>9}{"방위°":>9}{"방위흔들림":>11}'
          f'{"기준대비":>10}{"자이로":>9}')
    print('-' * 70)

    base = None
    rows = []
    for label, lin, ang, s in phases:
        if not s['mag']:
            print(f'{label:<20}  (샘플 없음)')
            continue
        hs = [math.hypot(x, y) for _, x, y, _ in s['mag']]
        bs = [(-math.degrees(math.atan2(y, x))) % 360 for _, x, y, _ in s['mag']]
        # 방위 평균은 각도라서 벡터로 평균내야 한다(359°와 1°의 산술평균은 180°가 된다)
        cx = statistics.fmean(math.cos(math.radians(b)) for b in bs)
        cy = statistics.fmean(math.sin(math.radians(b)) for b in bs)
        bmean = math.degrees(math.atan2(cy, cx)) % 360
        spread = math.degrees(math.acos(max(-1.0, min(1.0, math.hypot(cx, cy)))))
        gz = statistics.fmean(abs(g) for _, g in s['gyro']) if s['gyro'] else 0.0

        if base is None and label.startswith('기준'):
            base = bmean
        dev = '' if base is None else f'{((bmean - base + 180) % 360) - 180:+.1f}°'
        print(f'{label:<20}{statistics.fmean(hs):>9.1f}{bmean:>9.1f}'
              f'{spread:>10.1f}°{dev:>10}{gz:>9.3f}')
        rows.append({'label': label, 'linear': lin, 'angular': ang,
                     'horiz': statistics.fmean(hs), 'bearing': bmean,
                     'spread_deg': spread, 'gyro_abs': gz,
                     'dev_deg': None if base is None
                                else ((bmean - base + 180) % 360) - 180,
                     'n': len(s['mag'])})

    if out_path:
        json.dump(rows, open(out_path, 'w'), ensure_ascii=False, indent=1)
        print(f'\n원자료: {out_path}')

    moving = [r for r in rows if (r['linear'] or r['angular'])]
    if moving and base is not None:
        worst = max(moving, key=lambda r: abs(r['dev_deg']))
        print(f'\n최대 방위 편차: {worst["label"]}에서 {worst["dev_deg"]:+.1f}°')
        gz = statistics.fmean(r['gyro_abs'] for r in rows if r['label'].startswith('기준'))
        spun = [r for r in moving if r['gyro_abs'] > 0.05]
        if spun:
            print(f'⚠️ 차체가 실제로 회전했다(자이로 {max(r["gyro_abs"] for r in spun):.2f} rad/s) '
                  f'— 방위 변화에 회전분이 섞였다. 바퀴가 떠 있는지 확인할 것.')
        else:
            print('차체 회전 없음(자이로 정지) — 방위 변화는 전부 모터 간섭이다.')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/maginterf.json')
    ap.add_argument('--drive-node', default='/diff_drive_node')
    args = ap.parse_args()

    rclpy.init()
    node = Runner()

    # DDS 디스커버리가 붙는 데 걸리는 시간은 일정하지 않다. 고정 대기(2초)로 했더니
    # 멀쩡한 센서를 "죽었다"고 오판한 적이 있다 — **실제로 받을 때까지** 기다린다.
    deadline = time.time() + 15.0
    while time.time() < deadline and (len(node.mag) < 20 or len(node.gyro) < 20):
        rclpy.spin_once(node, timeout_sec=0.05)

    if len(node.mag) < 20:
        print(f'❌ 15초 동안 /imu/mag를 {len(node.mag)}개밖에 못 받았다. '
              '자기센서 멈춤 여부를 확인할 것(journalctl -u smart-trash-imu). 중단한다.')
        node.destroy_node(); rclpy.shutdown(); return 1
    if len(node.gyro) < 20:
        print(f'❌ /imu/data 수신 부족({len(node.gyro)}개) — 차체 회전 여부를 '
              '검증할 수 없으므로 중단한다.')
        node.destroy_node(); rclpy.shutdown(); return 1

    # 죽은 값이 반복 발행되는 경우도 걸러낸다(발행 주파수만으로는 살았는지 알 수 없다)
    if len({m[1:] for m in node.mag}) < 2:
        print('❌ 자기센서 값이 전혀 변하지 않는다 — 멈춤 상태다. 중단한다.')
        node.destroy_node(); rclpy.shutdown(); return 1
    print(f'센서 확인: 자기 {len(node.mag)}개, 자이로 {len(node.gyro)}개 수신\n')

    saved = None
    results = []
    try:
        # 폐루프를 끈다(위 docstring 참고). 되돌릴 값은 지금 읽어둔다.
        current = get_remote_params(node, args.drive_node, ['turn_kp', 'turn_ki'])
        if current is None:
            print(f'❌ {args.drive_node}의 현재 게인을 못 읽었다. '
                  '복원을 보장할 수 없으므로 중단한다.')
            return 1
        ok, why = set_remote_params(node, args.drive_node,
                                    {'turn_kp': 0.0, 'turn_ki': 0.0})
        if not ok:
            print(f'❌ 폐루프 비활성화 실패: {why}')
            print('   그대로 돌리면 적분항이 감겨 PWM이 최대로 치솟는다. 중단한다.')
            return 1
        saved = current
        print(f'폐루프 해제(turn_kp=0, turn_ki=0) — 피드포워드 PWM만 적용된다')
        print(f'  종료 시 복원할 값: {saved}\n')

        stop_flag = {'v': False}
        signal.signal(signal.SIGINT, lambda *a: stop_flag.update(v=True))
        signal.signal(signal.SIGTERM, lambda *a: stop_flag.update(v=True))

        for label, lin, ang, dur in PHASES:
            if stop_flag['v']:
                print('중단 신호 — 정지한다')
                break
            print(f'  {label} ... ', end='', flush=True)
            t0 = time.time()
            node.drive(lin, ang)
            while time.time() - t0 < dur and not stop_flag['v']:
                node.drive(lin, ang)      # 워치독 대비 계속 재발행
                rclpy.spin_once(node, timeout_sec=0.05)
            t1 = time.time()
            # 앞 1초는 가감속 과도구간이라 버린다
            results.append((label, lin, ang,
                            {'mag': window(node.mag, t0 + 1.0, t1),
                             'gyro': window(node.gyro, t0 + 1.0, t1)}))
            print('완료')
    finally:
        # 무슨 일이 있어도 반드시 정지시키고 파라미터를 되돌린다
        node.stop()
        if saved:
            ok, why = set_remote_params(node, args.drive_node, saved)
            print(f'\n폐루프 복원: {"성공" if ok else "실패 — " + why}')
            if not ok:
                print('⚠️ 수동으로 되돌릴 것: ros2 param set /diff_drive_node turn_kp 250.0')

    rc = analyze(results, args.out)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
