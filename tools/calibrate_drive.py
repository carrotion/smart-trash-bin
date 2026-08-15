#!/usr/bin/env python3
"""주행 캘리브레이션 — 오도메트리 파라미터를 실측으로 보정한다.

자로 잰 물리 치수는 출발점일 뿐이고, 스키드-스티어 4WD는 회전 시 바퀴가 미끄러져서
실효 트레드가 물리 트레드보다 크다. 그래서 "실제로 움직여보고 역산"하는 게 표준.

  straight : 직진시켜서 odom 거리 vs 실제 자로 잰 거리 → counts_per_rev 보정
  rotate   : 제자리 회전시켜서 odom 각도 vs 실제 회전각 → wheel_separation 보정
  maxspeed : 최대 출력으로 잠깐 굴려 실제 속도 측정 → max_wheel_speed 보정

사용법 (diff_drive_node가 떠 있어야 함):
    python3 tools/calibrate_drive.py straight --speed 0.15 --duration 3
    python3 tools/calibrate_drive.py rotate --speed 1.0 --duration 4
    python3 tools/calibrate_drive.py maxspeed --duration 2

주행이 끝나면 odom 기준 이동량을 출력한다. 실제 측정값을 넣으면 보정 계수를 계산:
    python3 tools/calibrate_drive.py straight --speed 0.15 --duration 3 --actual 1.0
    python3 tools/calibrate_drive.py rotate --speed 1.0 --duration 4 --actual 360
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CalibNode(Node):
    def __init__(self):
        super().__init__('calibrate_drive')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.odom = None

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self.odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x,
        }

    def wait_for_odom(self, timeout=5.0):
        start = time.time()
        while self.odom is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError('odom을 못 받음 — diff_drive_node가 떠 있는지 확인')
        return dict(self.odom)

    def drive(self, linear, angular, duration):
        """지정 속도로 duration초 주행. 끝나면 확실히 정지시킴."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        samples = []
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom:
                samples.append(self.odom['vx'])
        # 정지 — 여러 번 확실히
        stop = Twist()
        for _ in range(10):
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.5)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        return samples


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['straight', 'rotate', 'maxspeed'])
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--actual', type=float, default=None,
                    help='실제 측정값 (straight=미터, rotate=도). 주면 보정계수까지 계산')
    args = ap.parse_args()

    rclpy.init()
    node = CalibNode()
    try:
        before = node.wait_for_odom()
        print(f'시작 위치: x={before["x"]:.4f} y={before["y"]:.4f} yaw={math.degrees(before["yaw"]):.2f}°')
        print(f'{args.duration}초간 주행...')

        if args.mode == 'straight':
            samples = node.drive(args.speed, 0.0, args.duration)
        elif args.mode == 'rotate':
            samples = node.drive(0.0, args.speed, args.duration)
        else:  # maxspeed
            samples = node.drive(10.0, 0.0, args.duration)  # 상한에 걸려 최대 PWM로 나감

        after = node.wait_for_odom()
        print(f'종료 위치: x={after["x"]:.4f} y={after["y"]:.4f} yaw={math.degrees(after["yaw"]):.2f}°')

        dx = after['x'] - before['x']
        dy = after['y'] - before['y']
        dist = math.hypot(dx, dy)
        dyaw = math.degrees(after['yaw'] - before['yaw'])

        print(f'\n--- odom 기준 결과 ---')
        print(f'이동 거리: {dist:.4f} m  (dx={dx:.4f}, dy={dy:.4f})')
        print(f'회전각: {dyaw:.2f}°  (누적 wrap 주의 — 360° 넘으면 수동 계산 필요)')
        if samples:
            moving = [s for s in samples if abs(s) > 0.01]
            if moving:
                print(f'평균 속도: {sum(moving)/len(moving):.3f} m/s (최대 {max(moving, key=abs):.3f})')

        if args.actual is not None:
            print(f'\n--- 보정 계수 ---')
            if args.mode == 'straight':
                ratio = dist / args.actual
                print(f'odom {dist:.4f}m vs 실제 {args.actual}m → 비율 {ratio:.4f}')
                print(f'counts_per_rev를 현재값 × {ratio:.4f} 로 수정하세요.')
                print(f'  (예: 899.1 → {899.1 * ratio:.1f})')
            elif args.mode == 'rotate':
                ratio = dyaw / args.actual
                print(f'odom {dyaw:.2f}° vs 실제 {args.actual}° → 비율 {ratio:.4f}')
                print(f'wheel_separation을 현재값 × {ratio:.4f} 로 수정하세요.')
                print(f'  (예: 0.17 → {0.17 * ratio:.4f})')
    finally:
        try:
            stop = Twist()
            for _ in range(5):
                node.pub.publish(stop)
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
