#!/usr/bin/env python3
"""사용자가 폰/대시보드로 직접 조종하는 동안 자이로/오도메트리를 연속 로깅.

round_trip_gyro_diag.py는 스크립트가 직접 cmd_vel을 몰아서 왕복시키지만,
이건 **순수 수동 조종 시험용** — 로봇을 전혀 조종하지 않고 /imu/data(원시
자이로), /odom(휠), /odometry/filtered(EKF)만 구독해서 시간축으로 기록한다.
Layer 1 매핑 시험(거실->A방->복귀)에서 "최종 지도만 있고 과정을 못 본다"는
문제를 풀기 위한 것 — 나중에 로그 타임스탬프와 지도/기억을 대조해서 정확히
어느 구간에서 요(yaw) 오차가 붙는지 짚어낼 수 있다.

Ctrl+C로 멈추면 그때까지 기록을 저장한다.

사용법:
    python3 tools/loop_gyro_diag.py --out docs/calibration/loop_gyro_diag.csv
"""
import argparse
import csv
import math
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('loop_gyro_diag')
    state = {'gyro_z': None, 'ekf_yaw': None, 'odom_yaw': None,
             'ekf_x': None, 'ekf_y': None, 'odom_x': None, 'odom_y': None}

    f = open(args.out, 'w', newline='')
    writer = csv.DictWriter(f, fieldnames=[
        't', 'gyro_z', 'ekf_x', 'ekf_y', 'ekf_yaw_deg',
        'odom_x', 'odom_y', 'odom_yaw_deg'])
    writer.writeheader()

    def on_imu(msg):
        state['gyro_z'] = msg.angular_velocity.z
        writer.writerow({
            't': time.time(),
            'gyro_z': state['gyro_z'],
            'ekf_x': state['ekf_x'], 'ekf_y': state['ekf_y'],
            'ekf_yaw_deg': math.degrees(state['ekf_yaw']) if state['ekf_yaw'] is not None else '',
            'odom_x': state['odom_x'], 'odom_y': state['odom_y'],
            'odom_yaw_deg': math.degrees(state['odom_yaw']) if state['odom_yaw'] is not None else '',
        })

    def on_ekf(msg):
        p = msg.pose.pose
        state['ekf_x'], state['ekf_y'] = p.position.x, p.position.y
        state['ekf_yaw'] = yaw_from_quat(p.orientation)

    def on_odom(msg):
        p = msg.pose.pose
        state['odom_x'], state['odom_y'] = p.position.x, p.position.y
        state['odom_yaw'] = yaw_from_quat(p.orientation)

    n.create_subscription(Imu, '/imu/data', on_imu, 50)
    n.create_subscription(Odometry, '/odometry/filtered', on_ekf, 20)
    n.create_subscription(Odometry, '/odom', on_odom, 20)

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    print(f'로깅 시작 — 조종 끝나면 Ctrl+C (또는 SIGTERM). 저장: {args.out}')
    try:
        while not flag['v']:
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        f.close()
        n.destroy_node()
        rclpy.shutdown()
    print(f'저장 완료: {args.out}')


if __name__ == '__main__':
    main()
