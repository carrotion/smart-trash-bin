#!/usr/bin/env python3
"""
⚠️ 폐기됨 (2026-08-17). laser_filters의 StaticLaserScanPolygonFilter로 대체했다.
   설정: smart_trash_bringup/config/scan_self_filter.yaml

   왜 버렸나: 이 노드는 "거리 X 미만이면 차체"로 잘라서 **각도 정보를 통째로 버린다.**
   임계값 0.15m일 때 자기 구조물(실측 0.080m)뿐 아니라 **벽에 닿기 직전 구간
   (0.105~0.15m)까지 같이 지웠다.** 차체 반폭이 0.11m라 벽에 닿으면 그 벽이 사라지고,
   코스트맵에서 지워져 컨트롤러가 계속 밀어붙였다(실주행에서 관측).

   다각형 방식이 옳은 이유는 기하다: 차체(22x22cm) 안쪽에 찍힌 반사는 물리적으로
   실제 장애물일 수 없다. 임계값을 감으로 고를 필요가 없어진다.

   실행 진입점은 남겨 두지만 런치에서는 더 이상 쓰지 않는다.
LD06 스캔에서 로봇 자기 구조물 반사를 걸러내는 필터 노드.

**왜 필요한가 (2026-08-16 실측으로 확정)**

LD06을 로봇 상판에 올려놓으면 라이다가 자기 아래 구조물(마운트/상판 가장자리/
스탠드오프)을 360° 전방향에서 본다. 실측 히스토그램:

    0.06~0.08m :  48 포인트
    0.08~0.10m :  99 포인트
    0.10~0.30m :   0 포인트   <- 완전히 비어있음
    0.30m~     : 실제 환경

즉 자기반사(0.06~0.10m)와 실제 환경(0.30m~)이 깨끗하게 분리돼 있고, 그 사이
0.10~0.30m는 텅 비어 있다. 그래서 임계값을 그 빈 구간 한가운데인 **0.15m**로
잡으면 실제 장애물은 하나도 안 버리면서 자기반사만 정확히 제거된다.

안 걸러내면 SLAM 맵에 로봇 자기 위치가 장애물(occupancy 100)로 박히고, Nav2
global_costmap에서는 로봇을 감싼 lethal 링이 생겨서 **어떤 목표에 대해서도
경로가 안 나온다**(NavFn: "Failed to create plan"). 2026-08-16 첫 자율주행
테스트가 실패한 실제 원인이 이것.

**왜 여기서 거르나 (다른 방법이 안 되는 이유)**

- LD06 드라이버(`ldlidar_stl_ros2`)는 범위 관련 파라미터를 아예 선언하지 않음
  (`angle_crop_*`로 각도 자르기만 가능 — 자기반사는 전방향이라 못 씀).
- SLAM Toolbox에는 `min_laser_range` 파라미터가 **존재하지 않는다**
  (`ros2 param list /slam_toolbox` 결과 `max_laser_range`만 있음). yaml에 적어도
  조용히 무시되므로 거기 기대면 안 됨 — 실제로 그렇게 착각해서 한참 헤맸음.
- Nav2 obstacle_layer의 `obstacle_min_range`는 코스트맵만 보호하고 SLAM 맵은
  못 지킨다. 그래서 소스인 스캔 자체를 고치는 게 유일하게 양쪽 다 해결하는 방법.

버려진 포인트는 `inf`로 바꾼다(포인트를 지우면 인덱스↔각도 대응이 깨지므로
개수는 그대로 두고 값만 무효화). 출력 메시지의 `range_min`도 임계값으로 올려서,
이 토픽을 받는 어떤 소비자든 스스로 판단할 수 있게 한다.

사용법:
    ros2 run smart_trash_sensors scan_filter_node --ros-args \
        -p input_topic:=scan_raw -p output_topic:=scan -p range_min:=0.15
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        # 0.15m — 위 docstring의 실측 히스토그램상 자기반사(≤0.10m)와
        # 실제 환경(≥0.30m) 사이 빈 구간의 한가운데.
        self.declare_parameter('range_min', 0.15)

        self._range_min = float(self.get_parameter('range_min').value)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # ⚠️ RELIABLE이어야 함. LD06 드라이버가 RELIABLE로 발행하고,
        # sensor_dashboard도 RELIABLE로 구독한다. 반면 SLAM/Nav2 코스트맵은
        # SensorDataQoS(BEST_EFFORT)로 구독하는데, RELIABLE 발행자는 BEST_EFFORT
        # 구독자를 만족시키므로 RELIABLE로 두면 양쪽 다 받는다.
        # (처음에 BEST_EFFORT로 뒀다가 sensor_dashboard가 조용히 끊겼음.)
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(LaserScan, out_topic, qos)
        self.create_subscription(LaserScan, in_topic, self._on_scan, qos)

        # 얼마나 걸러내고 있는지 주기적으로 알려줌 — 마운트가 바뀌면 이 수치가
        # 달라지므로, 임계값을 다시 잡아야 하는 시점을 여기서 눈치챌 수 있다.
        self._total = 0
        self._dropped = 0
        self.create_timer(30.0, self._report)

        self.get_logger().info(
            f'스캔 필터 시작: {in_topic} -> {out_topic}, range_min={self._range_min}m')

    def _on_scan(self, msg: LaserScan):
        rmin = self._range_min
        ranges = list(msg.ranges)
        dropped = 0
        for i, v in enumerate(ranges):
            # NaN/inf는 이미 무효값이라 건드릴 필요 없음.
            if v < rmin and not math.isnan(v) and not math.isinf(v):
                ranges[i] = float('inf')
                dropped += 1
        msg.ranges = ranges
        # 소비자가 직접 판단할 수 있도록 메시지의 최소범위도 올려서 알린다.
        msg.range_min = max(msg.range_min, rmin)
        self._pub.publish(msg)

        self._total += len(ranges)
        self._dropped += dropped

    def _report(self):
        if self._total == 0:
            self.get_logger().warn('스캔이 안 들어오고 있음 — 라이다 노드/토픽명 확인 필요')
            return
        pct = 100.0 * self._dropped / self._total
        self.get_logger().info(
            f'자기반사 필터링: {self._dropped}/{self._total} 포인트 ({pct:.1f}%)')
        self._total = 0
        self._dropped = 0


def main():
    rclpy.init()
    node = ScanFilterNode()
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
