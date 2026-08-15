#!/usr/bin/env python3
"""맵 자동 저장 — SLAM Toolbox의 /map을 주기적으로 파일로 떠둔다.

2026-08-15에 집 한 바퀴를 다 돌아 맵을 만들었는데, 직후 전원이 갑자기 끊기면서
저장 전이라 통째로 날아갔다. 맵핑은 시간이 오래 걸리는 작업이라 중간 저장이 없으면
사고 한 번에 전부 잃는다 — 그래서 주기적으로 자동 저장한다.

nav2_map_server의 map_saver 서비스 대신 /map을 직접 받아 PGM+YAML로 쓴다
(외부 서비스 의존 없이 동작하고, 저장 실패가 조용히 묻히지 않도록).

사용법:
    python3 tools/map_autosave.py [--interval 60] [--dir ~/maps]
"""
import argparse
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class MapAutoSaver(Node):
    def __init__(self, out_dir, interval):
        super().__init__('map_autosave')
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.interval = interval
        self.last_save = 0.0
        self.saved_count = 0
        self.create_subscription(OccupancyGrid, 'map', self._on_map, 1)
        self.get_logger().info(f'맵 자동 저장 시작 — {self.out_dir}, {interval}초 간격')

    def _on_map(self, msg: OccupancyGrid):
        now = time.time()
        if now - self.last_save < self.interval:
            return
        self.last_save = now

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return

        base = os.path.join(self.out_dir, 'latest')
        try:
            # PGM: OccupancyGrid는 좌하단 기준 row-major, PGM은 좌상단부터라 뒤집어 씀
            with open(base + '.pgm', 'wb') as f:
                f.write(f'P5\n{w} {h}\n255\n'.encode())
                for row in range(h - 1, -1, -1):
                    line = bytearray(w)
                    for col in range(w):
                        v = msg.data[row * w + col]
                        # ROS map 규약: 미지=205(회색), 빈공간=254(흰색), 장애물=0(검정)
                        line[col] = 205 if v < 0 else (0 if v >= 65 else 254)
                    f.write(bytes(line))

            with open(base + '.yaml', 'w') as f:
                f.write(
                    f"image: latest.pgm\n"
                    f"resolution: {msg.info.resolution}\n"
                    f"origin: [{msg.info.origin.position.x}, "
                    f"{msg.info.origin.position.y}, 0.0]\n"
                    f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

            self.saved_count += 1
            known = sum(1 for v in msg.data if v >= 0)
            self.get_logger().info(
                f'맵 저장 #{self.saved_count}: {w}x{h}, 탐색 {100*known/len(msg.data):.1f}%')
        except OSError as e:
            self.get_logger().error(f'맵 저장 실패: {e}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--interval', type=float, default=60.0)
    ap.add_argument('--dir', default='~/maps')
    args = ap.parse_args()

    rclpy.init()
    node = MapAutoSaver(args.dir, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
