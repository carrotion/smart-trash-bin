import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import qwiic_icm20948

# ICM-20948 default full-scale range after begin(): accel +/-2g, gyro +/-250dps.
# AK09916 magnetometer: 0.15 uT/LSB (16-bit, +/-4912 uT).
MAG_SENSITIVITY_UT_PER_LSB = 0.15
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

        # 지자기센서(AK09916, ICM-20948 내장) 발행 — 2026-08-17 추가.
        # 목적: 이 로봇에서 자기센서가 방위 기준으로 쓸 만한지 **평가**하기 위함.
        # 아직 어떤 추정에도 쓰지 않는다(EKF 입력 아님). 우려되는 점:
        #  - 모터 4개(영구자석 + 수 A 전류)가 센서 바로 아래 -> 지자기(~50uT)를 덮을 수 있음
        #  - 실내 철재(냉장고 등)로 인한 왜곡. 로봇이 갇혔던 곳이 하필 냉장고 옆
        # 회전 중 자이로와 나란히 비교해서 판단할 것. 쓸 만하면 센서를 모터에서
        # 더 멀리/높이 옮기는 것도 선택지(사용자 확인).
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        try:
            self._imu.startupMagnetometer()
            self._mag_ok = True
            self.get_logger().info('자기센서(AK09916) 초기화 완료 — imu/mag 발행')
        except Exception as e:
            self._mag_ok = False
            self.get_logger().warn(f'자기센서 초기화 실패: {e} (IMU 나머지는 정상 동작)')
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

        # EKF가 신뢰도 가중에 쓰는 값. ICM-20948 자이로/가속도계 노이즈에 대한
        # 보수적인 추정치(정밀 캘리브레이션 전) — 대각항만 채움, 축간 상관 없음으로 가정.
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.0004       # (rad/s)^2
            msg.linear_acceleration_covariance[i] = 0.02      # (m/s^2)^2

        self._pub.publish(msg)

        if self._mag_ok:
            mag = MagneticField()
            mag.header = msg.header
            # AK09916: 0.15 uT/LSB -> 테슬라 단위(REP-145)로 변환
            mag.magnetic_field.x = self._imu.mxRaw * MAG_SENSITIVITY_UT_PER_LSB * 1e-6
            mag.magnetic_field.y = self._imu.myRaw * MAG_SENSITIVITY_UT_PER_LSB * 1e-6
            mag.magnetic_field.z = self._imu.mzRaw * MAG_SENSITIVITY_UT_PER_LSB * 1e-6
            # 아직 하드/소프트 아이언 보정을 안 했으므로 공분산은 미상(0)으로 둔다.
            self._mag_pub.publish(mag)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
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
