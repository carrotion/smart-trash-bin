import math
import time

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

        # 하드아이언 보정값 (µT). 로봇 자신의 철재/자석이 만드는 **상수 벡터**라서
        # 그냥 빼주면 된다. 기체마다 배선/부품 배치가 달라 값이 다르므로 파라미터로 둔다
        # — 2호기부터는 /etc/smart-trash/calibration.yaml로 분리할 것(fleet 문서 참고).
        # 측정: tools/mag_calibrate.py로 손 회전 2바퀴 -> 원 적합의 중심.
        # 1호기 2026-08-17 실측: x=-0.44, y=+14.37 (샘플 8135, 커버리지 100%, 잔차 1.87µT)
        self.declare_parameter('mag_hard_iron_x', 0.0)
        self.declare_parameter('mag_hard_iron_y', 0.0)
        self.declare_parameter('mag_hard_iron_z', 0.0)
        self._mag_off = (
            self.get_parameter('mag_hard_iron_x').value,
            self.get_parameter('mag_hard_iron_y').value,
            self.get_parameter('mag_hard_iron_z').value,
        )
        if any(self._mag_off):
            self.get_logger().info(
                f'하드아이언 보정 적용: {self._mag_off[0]:+.2f} / '
                f'{self._mag_off[1]:+.2f} / {self._mag_off[2]:+.2f} µT')
        else:
            self.get_logger().warn(
                '하드아이언 보정값이 0이다 — 방위가 최대 수십 도 틀어질 수 있다. '
                'tools/mag_calibrate.py로 측정할 것.')

        # 자기센서 멈춤(freeze) 감지용 상태. 상세 이유는 _publish_mag 참고.
        self.declare_parameter('mag_stale_sec', 2.0)
        self._mag_stale_sec = self.get_parameter('mag_stale_sec').value
        self._mag_last_raw = None
        self._mag_last_change = time.monotonic()
        self._mag_recover_at = 0.0
        self._mag_frozen = False

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
            self._publish_mag(msg.header)

    def _publish_mag(self, header):
        """자기센서 값 발행 + **멈춤 감지/자동 복구**.

        왜 필요한가 (2026-08-17 실측):
        AK09916은 ICM-20948 **내부 I2C 마스터**를 통해 읽힌다. 이 자기센서가 응답을
        멈추면(qwiic 라이브러리 주석에도 "After a ICM reset the Mag sensor may stop
        responding over the I2C master"라고 적혀 있다) ICM 쪽 EXT_SLV_SENS_DATA
        레지스터는 **마지막 값을 그대로 붙들고 있다.** 그러면 노드는 아무 에러 없이
        죽은 값을 50Hz로 계속 퍼블리시한다 — 토픽 주파수만 보면 완전히 정상으로 보인다.
        실제로 33초간 1654샘플이 전부 바이트 단위로 동일했다(살아있으면 최소 LSB
        노이즈라도 흔들린다). 조용히 실패하는 유형이라 반드시 감지해야 한다.

        대응:
          - 원시값이 `mag_stale_sec` 동안 한 번도 안 바뀌면 멈춤으로 판정
          - 멈춘 동안에는 **발행을 중단**한다. 죽은 값을 계속 내보내면 하위 소비자가
            그걸 신선한 값으로 믿는다 — 안 보내면 대시보드가 "응답 없음"으로 뜬다
          - startupMagnetometer()로 재초기화 시도(내부에서 I2C 마스터를 리셋한다).
            5초에 한 번으로 제한해서 실패가 반복돼도 루프를 막지 않는다
        """
        raw = (self._imu.mxRaw, self._imu.myRaw, self._imu.mzRaw)
        now = time.monotonic()

        if raw != self._mag_last_raw:
            self._mag_last_raw = raw
            self._mag_last_change = now
            if self._mag_frozen:
                self._mag_frozen = False
                self.get_logger().info('자기센서 복구됨 — imu/mag 발행 재개')
        elif now - self._mag_last_change > self._mag_stale_sec:
            if not self._mag_frozen:
                self._mag_frozen = True
                self.get_logger().warn(
                    f'자기센서 멈춤 감지: 원시값이 {now - self._mag_last_change:.1f}초간 '
                    '변화 없음. 발행을 중단하고 재초기화를 시도한다.')
            if now >= self._mag_recover_at:
                self._mag_recover_at = now + 5.0
                try:
                    self._imu.startupMagnetometer()
                except Exception as e:
                    self.get_logger().warn(f'자기센서 재초기화 실패: {e}')
            return  # 멈춘 값은 내보내지 않는다

        mag = MagneticField()
        mag.header = header
        # AK09916: 0.15 uT/LSB -> 테슬라 단위(REP-145)로 변환.
        # 하드아이언 편향은 µT 단위에서 빼고 나서 테슬라로 바꾼다.
        ox, oy, oz = self._mag_off
        mag.magnetic_field.x = (raw[0] * MAG_SENSITIVITY_UT_PER_LSB - ox) * 1e-6
        mag.magnetic_field.y = (raw[1] * MAG_SENSITIVITY_UT_PER_LSB - oy) * 1e-6
        mag.magnetic_field.z = (raw[2] * MAG_SENSITIVITY_UT_PER_LSB - oz) * 1e-6
        # 소프트아이언(타원 왜곡)은 아직 미보정이라 공분산은 미상(0)으로 둔다.
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
