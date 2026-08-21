#!/usr/bin/env python3
"""자율주행 세션 중 실제 명령/위치를 CSV로 남긴다.

목적: "뭔가 이상했다"는 사후 진술만으로 원인을 추측하지 않기 위함(2026-08-21
반성 — 로그 없이 nav2_params.yaml만 읽고 원인을 짐작했다가 지적받음). 이 도구를
자율주행과 같이 띄워두면, 다음에 문제가 생겼을 때 실제 명령 속도·경로·정체
시점을 데이터로 바로 확인할 수 있다.

사용법:
    python3 tools/nav_session_logger.py --out docs/calibration/nav_session_TIMESTAMP.csv
    (Ctrl-C로 종료, 그때까지의 기록이 저장됨)
"""
import argparse, csv, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('nav_session_logger')
    state = {'cmd_v': 0.0, 'cmd_w': 0.0, 'x': None, 'y': None, 'yaw': 0.0}

    def on_cmd(msg):
        state['cmd_v'] = msg.linear.x
        state['cmd_w'] = msg.angular.z

    import math
    def on_odom(msg):
        p = msg.pose.pose
        state['x'] = p.position.x
        state['y'] = p.position.y
        q = p.orientation
        state['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    n.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
    n.create_subscription(Odometry, '/odometry/filtered', on_odom, 10)

    f = open(args.out, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'cmd_v', 'cmd_w', 'x', 'y', 'yaw_deg'])

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    print(f'기록 시작 -> {args.out}  (Ctrl-C로 종료)')
    t0 = time.time()
    n_rows = 0
    period = 1.0 / args.hz
    next_t = time.time()
    try:
        while not flag['v']:
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            if now >= next_t:
                next_t = now + period
                if state['x'] is not None:
                    w.writerow([f'{now - t0:.2f}', f'{state["cmd_v"]:.3f}',
                               f'{state["cmd_w"]:.3f}', f'{state["x"]:.3f}',
                               f'{state["y"]:.3f}', f'{math.degrees(state["yaw"]):.1f}'])
                    n_rows += 1
    finally:
        f.close()
        print(f'\n종료 — {n_rows}행 기록됨: {args.out}')
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
