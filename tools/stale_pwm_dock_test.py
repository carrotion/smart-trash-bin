#!/usr/bin/env python3
"""멈춘 뒤 모터 보드에 PWM이 남는가 — 도크 위 판정 시험(바퀴가 떠 있어 작은 PWM도 보인다, 2026-11-01).

사고: 스택을 내리고 11분 뒤 도크에 올리자 명령 0인데 뒷바퀴가 돌았다. 의심: 능동 제동 PWM을 보내는 중에 명령이 끊기면 시간 초과 처리가 0을 안 보낸다.
순서: 직진·회전을 잠깐 → 0을 1~2번만 → 명령 끊고 3초 관찰 → 0을 보내 확실히 정지.
합격: 관찰 구간 끝의 바퀴 속도 전부 0. ⚠️ 바퀴가 도니 반드시 도크 위(바퀴 뜬 상태)에서.

    python3 tools/stale_pwm_dock_test.py [--reps 2]
"""
import argparse, json, sys, time, urllib.request

import rclpy
from geometry_msgs.msg import Twist


# 0을 1~2번만: 떠 있는 바퀴는 금방 멈춰 0을 다섯 번 보내면 제동이 저절로 끝나 재현되지 않는다(첫 시도 0/3, 이 모양으로 고치기 전 7/8)
SHAPES = [('직진 0.15, 0을 1번', 0.15, 0.0, 1), ('직진 0.3, 0을 1번', 0.3, 0.0, 1), ('직진 0.3, 0을 2번', 0.3, 0.0, 2), ('후진 0.15, 0을 1번', -0.15, 0.0, 1),
          ('회전 +1, 0을 1번', 0.0, 1.0, 1), ('회전 −1, 0을 1번', 0.0, -1.0, 1)]


def rates():
    return json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=3))['motor']['rate']


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--reps', type=int, default=2); a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('stale_pwm_dock_test'); pub = n.create_publisher(Twist, '/cmd_vel', 10); time.sleep(1.0); bad = total = 0

    def send(v, secs, hz=20, w=0.0):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); e = time.time() + secs
        while time.time() < e:
            pub.publish(m); time.sleep(1.0 / hz)
    try:
        for k in range(a.reps):
            for name, v, w, zeros in SHAPES:
                send(v, 1.5, w=w)
                for _ in range(zeros):
                    pub.publish(Twist()); time.sleep(0.03)
                time.sleep(3.0); r = rates(); moving = max(abs(x) for x in r) > 50; bad += moving; total += 1
                print(f'{k + 1}회 {name}: 명령 끊고 3초 뒤 바퀴 속도(틱/초) {[round(x) for x in r]} → {"❌ 남은 PWM으로 돈다" if moving else "✅ 정지"}', flush=True)
                send(0.0, 1.5)
    finally:
        send(0.0, 1.0); rclpy.shutdown()
    print(f'{"❌" if bad else "✅"} {total}번 중 {bad}번 PWM이 남음')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
