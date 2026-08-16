#!/usr/bin/env python3
"""위치별 WiFi 신호세기 로거 — 로봇 좌표(EKF /odometry/filtered)와 두 WiFi
인터페이스(wlan0, USB 동글)의 신호세기를 같이 기록해서, 나중에 맵 위에
신호 히트맵을 그려 음영지역을 찾을 수 있게 한다.

2026-08-16: "주방 등 음영지역에서 접속이 잘 안 됨" 문제를 정량적으로
확인하기 위해 추가. 위치는 EKF가 주는 odom 프레임 좌표라 map->odom
드리프트가 크면 실제 방 좌표와 살짝 어긋날 수 있음 — 큰 문제는 아니지만
참고할 것(SLAM 세션 하나 안에서 로깅하면 드리프트 누적이 적어서 괜찮음).

사용법:
    python3 tools/wifi_signal_logger.py [--interval 1.0] [--out ~/wifi_signal.csv]
    (로봇을 조종해서 신호 약한 곳들을 돌아다니면 됨 — SLAM/diff_drive_node가
    떠있어야 위치가 기록됨. 안 떠있으면 x,y는 빈 값으로 남고 신호만 기록됨.)
"""
import argparse
import csv
import os
import re
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# 두 곳에 따로 하드코딩했다가 한쪽만 갱신해서 조용히 깨졌던 적이 있음
# (2026-08-16, USB 동글을 5GHz 지원 모델로 교체하면서 인터페이스명이 바뀜) —
# 앞으로 인터페이스가 또 바뀌면 여기 한 곳만 고치면 되게 상수로 뺌.
IFACE_BUILTIN = "wlan0"
IFACE_USB = "wlx00e04c4d96cb"
IFACES = [IFACE_BUILTIN, IFACE_USB]
_WIRELESS_RE = re.compile(r"^\s*(\S+):\s+\S+\s+([\-\d.]+)\.\s+([\-\d.]+)\.")


def read_signal_levels():
    """/proc/net/wireless에서 인터페이스별 link quality/level(dBm) 읽기."""
    levels = {}
    try:
        with open("/proc/net/wireless") as f:
            for line in f:
                m = _WIRELESS_RE.match(line)
                if m and m.group(1) in IFACES:
                    levels[m.group(1)] = {
                        "quality": float(m.group(2)),
                        "level_dbm": float(m.group(3)),
                    }
    except OSError:
        pass
    return levels


class SignalLoggerNode(Node):
    def __init__(self, out_path, interval):
        super().__init__('wifi_signal_logger')
        self._out_path = out_path
        self._interval = interval
        self._x = None
        self._y = None
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        self.create_timer(interval, self._on_timer)

        new_file = not os.path.exists(out_path)
        self._f = open(out_path, "a", newline="")
        self._writer = csv.writer(self._f)
        if new_file:
            self._writer.writerow(
                ["iso", "x", "y", "wlan0_dbm", "wlan0_quality",
                 "usb_dbm", "usb_quality"])
            self._f.flush()

    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _on_timer(self):
        levels = read_signal_levels()
        wlan0 = levels.get(IFACE_BUILTIN, {})
        usb = levels.get(IFACE_USB, {})
        row = [
            time.strftime("%H:%M:%S"),
            f"{self._x:.3f}" if self._x is not None else "",
            f"{self._y:.3f}" if self._y is not None else "",
            wlan0.get("level_dbm", ""),
            wlan0.get("quality", ""),
            usb.get("level_dbm", ""),
            usb.get("quality", ""),
        ]
        self._writer.writerow(row)
        self._f.flush()
        os.fsync(self._f.fileno())
        self.get_logger().info(
            f"x={row[1]} y={row[2]} wlan0={row[3]}dBm usb={row[5]}dBm")

    def destroy_node(self):
        try:
            self._f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", default=os.path.expanduser("~/wifi_signal.csv"))
    parsed = ap.parse_args()

    rclpy.init(args=args)
    node = SignalLoggerNode(parsed.out, parsed.interval)
    node.get_logger().info(f"신호 로깅 시작 -> {parsed.out}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
