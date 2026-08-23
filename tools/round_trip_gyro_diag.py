#!/usr/bin/env python3
"""round_trip_drift.py에서 발견된 문제(EKF 요 이탈이 휠 단독보다 큼) 원인 진단용.

전진-정지-후진-정지를 몇 회만 반복하면서 /imu/data(원시 자이로 z)와
/odometry/filtered(EKF 요)를 **연속으로 고주파 기록**한다. round_trip_drift.py는
사이클 끝에서만 스냅샷을 찍어서 "언제" 틀어지는지 모른다 — 이건 정지/역전
순간에 자이로가 이상값을 내는지 시계열로 직접 본다.

사용법:
    python3 tools/round_trip_gyro_diag.py --cycles 4 --speed 0.2 --sec 2.5 \
        --out docs/calibration/gyro_diag.csv
"""
import argparse
import csv
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Runner:
    def __init__(self, writer):
        rclpy.init()
        self.n = rclpy.create_node('round_trip_gyro_diag')
        self.writer = writer
        self.gyro_z = None
        self.ekf_yaw = None
        self.odom_yaw = None
        self.cmd_v = 0.0
        self.n.create_subscription(Imu, '/imu/data', self._on_imu, 50)
        self.n.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 50)
        self.n.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.pub = self.n.create_publisher(Twist, '/cmd_vel', 10)

    def _on_imu(self, msg):
        self.gyro_z = msg.angular_velocity.z
        self._log()

    def _on_ekf(self, msg):
        self.ekf_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def _on_odom(self, msg):
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def _log(self):
        if self.gyro_z is None:
            return
        self.writer.writerow({
            't': time.time(),
            'cmd_v': self.cmd_v,
            'gyro_z': self.gyro_z,
            'ekf_yaw_deg': math.degrees(self.ekf_yaw) if self.ekf_yaw is not None else '',
            'odom_yaw_deg': math.degrees(self.odom_yaw) if self.odom_yaw is not None else '',
        })

    def drive(self, v):
        self.cmd_v = v
        t = Twist(); t.linear.x = float(v); t.angular.z = 0.0
        self.pub.publish(t)

    def stop(self):
        for _ in range(6):
            self.drive(0.0); time.sleep(0.04)

    def spin(self, sec, v=None, flag=None):
        end = time.time() + sec
        while time.time() < end:
            if flag and flag['v']:
                return
            if v is not None:
                self.drive(v)
            rclpy.spin_once(self.n, timeout_sec=0.01)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--sec', type=float, default=2.5)
    ap.add_argument('--cycles', type=int, default=4)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    f = open(args.out, 'w', newline='')
    writer = csv.DictWriter(f, fieldnames=['t', 'cmd_v', 'gyro_z', 'ekf_yaw_deg', 'odom_yaw_deg'])
    writer.writeheader()

    r = Runner(writer)
    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    try:
        for i in range(args.cycles):
            if flag['v']:
                break
            print(f'cycle {i+1}/{args.cycles}: forward')
            r.spin(0.8, None, flag)
            r.spin(args.sec, args.speed, flag)
            r.stop()
            print(f'cycle {i+1}/{args.cycles}: backward')
            r.spin(0.8, None, flag)
            r.spin(args.sec, -args.speed, flag)
            r.stop()
            r.spin(0.8, None, flag)
    finally:
        r.stop()
        f.close()
        r.n.destroy_node()
        rclpy.shutdown()
    print(f'저장: {args.out}')


if __name__ == '__main__':
    main()
