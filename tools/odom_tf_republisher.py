#!/usr/bin/env python3
"""`/odom` 메시지를 odom->base_link TF로 다시 발행한다. 가방 재생 전용.

왜 필요한가
-----------
가방을 재생해 SLAM을 다시 돌릴 때 `/tf`를 그대로 틀면 안 된다. 녹화 당시 SLAM이
만든 map->odom 변환이 들어 있어서, 새로 띄운 SLAM과 **같은 변환을 두 노드가
발행**하게 되고 TF 트리가 오염된다(자세가 튀거나 조회가 실패한다).

그래서 재생에서는 `/tf`를 빼고, 대신 `/odom`(순수 휠/EKF 오도메트리)에서
odom->base_link를 여기서 만들어 준다. base_link->base_laser는 정적 변환이라
`/tf_static`에 있는 걸 그대로 쓰면 된다.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_republisher')
        self.br = TransformBroadcaster(self)
        self.n = 0
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.create_timer(5.0, self._report)
        self.get_logger().info('/odom -> odom->base_link TF 재발행 시작')

    def _on_odom(self, msg):
        t = TransformStamped()
        # 원본 타임스탬프를 그대로 쓴다. 지금 시각으로 바꾸면 SLAM이 스캔과
        # 짝을 못 맞춰서 조용히 전부 버린다.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)
        self.n += 1

    def _report(self):
        self.get_logger().info(f'재발행 {self.n}건')


def main():
    rclpy.init()
    node = OdomTfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
