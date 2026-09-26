"""라이다 스캔을 항상 같은 개수의 각도 칸으로 다시 나눈다(scan_in → scan_out).

왜 필요한가 (2026-09-26 발견):
LD06은 한 바퀴에 454~456점을 내보낸다(실측 455점 62%, 454/456점 38%). slam_toolbox는
**첫 스캔의 점 개수를 기준으로 삼고, 개수가 다른 스캔을 버린다**
("LaserRangeScan contains 454 range readings, expected 455"). 그래서 실제 매핑에서 스캔의
약 38%가 버려졌고, 녹화 재생에서 첫 스캔이 456점으로 잡히자 약 83%가 버려졌다 — 첫 스캔이
몇 점이냐에 따라 SLAM 입력이 크게 달라진다.

방법: 출력은 [0, 2π)를 num_bins칸으로 고정. 입력 점마다 각도로 칸을 정하고, 한 칸에
여러 점이 떨어지면 **가장 가까운 거리**를 쓴다(장애물을 놓치지 않는 쪽). 빈 칸은 inf.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanResampleNode(Node):
    def __init__(self):
        super().__init__('scan_resample')
        self.declare_parameter('num_bins', 455)
        # 입력 QoS는 센서 기본(BEST_EFFORT)으로 받아 RELIABLE/BEST_EFFORT 발행자 모두 수용,
        # 출력은 RELIABLE — sensor_dashboard 등 RELIABLE 구독자가 조용히 끊기지 않게(08-16 교훈)
        self._n = int(self.get_parameter('num_bins').value)
        self._inc = 2.0 * math.pi / self._n
        self._pub = self.create_publisher(LaserScan, 'scan_out', 10)
        self.create_subscription(LaserScan, 'scan_in', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'스캔 재분할: {self._n}칸, {math.degrees(self._inc):.3f}° 간격')

    def _on_scan(self, m: LaserScan):
        n = self._n
        rng = [math.inf] * n
        has_int = len(m.intensities) == len(m.ranges)
        inten = [0.0] * n if has_int else []
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            a = (m.angle_min + i * m.angle_increment) % (2.0 * math.pi)
            k = int(round(a / self._inc)) % n
            if r < rng[k]:
                rng[k] = r
                if has_int:
                    inten[k] = m.intensities[i]
        out = LaserScan()
        out.header = m.header
        out.angle_min = 0.0
        out.angle_increment = self._inc
        out.angle_max = self._inc * (n - 1)
        out.scan_time = m.scan_time
        out.time_increment = m.scan_time / n if m.scan_time > 0 else m.time_increment
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.ranges = rng
        out.intensities = inten
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ScanResampleNode()
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
