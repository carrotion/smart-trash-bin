import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import qwiic_icm20948

# ICM-20948 default full-scale range after begin(): accel +/-2g, gyro +/-250dps.
# See docs/phase0_hardware_ros2_guide.md IMU section / Week 2 calibration task —
# re-verify these sensitivities against the real sensor during IMU bring-up.
ACCEL_SENSITIVITY_LSB_PER_G = 16384.0
GYRO_SENSITIVITY_LSB_PER_DPS = 131.0
GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


class ImuPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self._frame_id = frame_id
        self._imu = qwiic_icm20948.QwiicIcm20948()
        if not self._imu.connected:
            raise RuntimeError(
                'ICM-20948을 I2C 버스에서 찾을 수 없습니다. Qwiic 케이블/전원 연결을 확인하세요.')
        self._imu.begin()

        self._pub = self.create_publisher(Imu, 'imu/data', 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(f'ICM-20948 연결됨, {rate_hz}Hz로 imu/data 퍼블리시 시작')

    def _on_timer(self):
        if not self._imu.dataReady():
            return
        self._imu.getAgmt()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.linear_acceleration.x = self._imu.axRaw / ACCEL_SENSITIVITY_LSB_PER_G * GRAVITY
        msg.linear_acceleration.y = self._imu.ayRaw / ACCEL_SENSITIVITY_LSB_PER_G * GRAVITY
        msg.linear_acceleration.z = self._imu.azRaw / ACCEL_SENSITIVITY_LSB_PER_G * GRAVITY

        msg.angular_velocity.x = self._imu.gxRaw / GYRO_SENSITIVITY_LSB_PER_DPS * DEG_TO_RAD
        msg.angular_velocity.y = self._imu.gyRaw / GYRO_SENSITIVITY_LSB_PER_DPS * DEG_TO_RAD
        msg.angular_velocity.z = self._imu.gzRaw / GYRO_SENSITIVITY_LSB_PER_DPS * DEG_TO_RAD

        # This node doesn't fuse orientation — robot_localization EKF (Week 4) does
        # that downstream. Mark orientation unknown per REP-145.
        msg.orientation_covariance[0] = -1.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
