#!/usr/bin/env python3
"""웨이포인트 대조 실측용 pose 캡처 (Issue #3, Track 0 지원).

round_trip_drift.py는 왕복(전진-후진)이라 이동 자체를 스크립트가 자동으로
몬다. 웨이포인트 대조("시작점 -> 방 중앙" 등)는 매번 목적지 방향이 달라
자동 주행으로 일반화하기 어렵다 — 그래서 이 도구는 로봇을 몰지 않고,
**사용자가 로봇을 수동으로 이동시키는 동안 시작/도착 시점의 pose만
스냅샷으로 캡처**한다. 캡처된 JSON은 그대로 waypoint_error.py --log에 넣으면 된다.

사용법:
    python3 tools/waypoint_capture.py --out docs/calibration/waypoint_run1.json
    (로봇을 시작점에 놓고 Enter -> 목적지로 수동 이동 -> 도착 후 Enter)
"""
import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Capturer:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('waypoint_capture')
        self.raw = None
        self.ekf = None
        self.n.create_subscription(Odometry, '/odom', self._on_raw, 10)
        self.n.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 10)

    def _on_raw(self, msg):
        p = msg.pose.pose
        self.raw = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _on_ekf(self, msg):
        p = msg.pose.pose
        self.ekf = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def wait_for_data(self, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline and (self.raw is None or self.ekf is None):
            rclpy.spin_once(self.n, timeout_sec=0.02)
        return self.raw is not None and self.ekf is not None

    def snapshot(self, seconds=0.5):
        """지정 시간 동안 spin해서 최신 pose를 갱신한 뒤 스냅샷."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.n, timeout_sec=0.02)
        return self.raw, self.ekf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='결과 JSON 저장 경로')
    ap.add_argument('--label', default='', help='이 측정에 대한 메모(방 이름 등)')
    args = ap.parse_args()

    c = Capturer()
    print('/odom, /odometry/filtered 구독 대기 중...')
    if not c.wait_for_data():
        print('❌ 토픽 수신 실패 — diff_drive_node/ekf_node가 떠 있는지 확인할 것')
        c.n.destroy_node(); rclpy.shutdown()
        return 1

    input('로봇을 시작점(웨이포인트 출발 지점)에 놓고 Enter...')
    start_raw, start_ekf = c.snapshot()
    print(f'  시작 — /odom: ({start_raw[0]:+.3f}, {start_raw[1]:+.3f}) {start_raw[2]:+.1f}°   '
          f'EKF: ({start_ekf[0]:+.3f}, {start_ekf[1]:+.3f}) {start_ekf[2]:+.1f}°')

    print('\n이제 로봇을 목표 지점까지 수동으로 이동시키세요 (텔레옵/조종 앱 등).')
    input('도착하면 Enter...')
    # 이동 직후 pose가 약간 흔들릴 수 있어 0.5초 정도 안정화 후 스냅샷.
    end_raw, end_ekf = c.snapshot()
    print(f'  도착 — /odom: ({end_raw[0]:+.3f}, {end_raw[1]:+.3f}) {end_raw[2]:+.1f}°   '
          f'EKF: ({end_ekf[0]:+.3f}, {end_ekf[1]:+.3f}) {end_ekf[2]:+.1f}°')

    data = {
        'date': time.strftime('%Y-%m-%d %H:%M'),
        'label': args.label,
        'start_raw': list(start_raw), 'start_ekf': list(start_ekf),
        'end_raw': list(end_raw), 'end_ekf': list(end_ekf),
    }
    with open(args.out, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print(f'\n저장: {args.out}')
    print('채점: python3 tools/waypoint_error.py --start-px <PX PY> --target-px <PX PY> '
          f'--log {args.out}')

    c.n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
