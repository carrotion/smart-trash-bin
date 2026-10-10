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

import shutil

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
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
        # 위치 잡기용 자료(SLAM 자세 그래프)와 금지 구역도 주기적으로 떠 둔다(10-10): 그림(pgm)만 있으면 전원이 끊긴 뒤
        #   그 지도를 이어 쓸 방법이 줄고, 구역은 그 지도 좌표에서만 뜻이 있다. 그림보다 무거워 5분에 한 번
        self.graph_interval = 300.0
        self.last_graph = time.time()
        try:
            from slam_toolbox.srv import SerializePoseGraph
            self._ser_type = SerializePoseGraph
            self._ser = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        except ImportError:
            self._ser = None
        self.get_logger().info(f'맵 자동 저장 시작 — {self.out_dir}, {interval}초 간격')

    def _save_graph(self):
        if self._ser is None or not self._ser.service_is_ready():
            return
        req = self._ser_type.Request(); req.filename = os.path.join(self.out_dir, 'latest')
        fut = self._ser.call_async(req)

        def done(f):
            try:
                ok = f.result().result == 0
            except Exception:
                ok = False
            zones = os.path.expanduser('~/maps/keepout_zones.json')
            if ok and os.path.isfile(zones):
                try:
                    shutil.copyfile(zones, os.path.join(self.out_dir, 'latest.keepout.json'))
                except OSError:
                    pass
            self.get_logger().info('위치 잡기용 자료 저장 ' + ('완료' if ok else '실패'))
        fut.add_done_callback(done)

    def _on_map(self, msg: OccupancyGrid):
        now = time.time()
        if now - self.last_save < self.interval:
            return
        self.last_save = now
        if now - self.last_graph >= self.graph_interval:
            self.last_graph = now
            self._save_graph()

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
