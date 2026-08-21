import math
import threading
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

        # 자이로 Z축 바이어스 보정 (rad/s). 정지 상태에서도 0이 아닌 상수값이 실려
        # 나온다 — 노이즈라면 흔들려야 하는데 90초/3066샘플 평균에 표준오차가
        # 0.0011rad/s로 매우 작아(값 대비 7배 이상 작음) 상수 바이어스로 판정.
        # EKF는 이 값을 자이로 Z(vyaw)로 그대로 적분하므로, 정지 중에도 방향이
        # 서서히 도는 것처럼 보이고 직진 시험에서 "전진/후진이 같은 부호로 휨"
        # (좌우 편향이면 반대 부호여야 함)의 원인 중 하나로 지목됨(2026-08-21).
        # 측정: 정지 상태에서 /imu/data의 angular_velocity.z를 60초 이상 평균.
        # 1호기 2026-08-21 실측(IMU 전방 재장착 직후): -0.00782 rad/s
        self.declare_parameter('gyro_bias_z', 0.0)
        self._gyro_bias_z = self.get_parameter('gyro_bias_z').value
        if self._gyro_bias_z:
            self.get_logger().info(f'자이로 Z 바이어스 보정 적용: {self._gyro_bias_z:+.5f} rad/s')
          def _init_magnetometer(self):
        """AK09916 지자기 센서 초기화 및 ICM-20948 보조 I2C 마스터 ODR 동기화(137.5Hz)."""
        self._imu.startupMagnetometer()
        self._imu.setBank(3)
        self._imu._i2c.writeByte(self._imu.address, self._imu.AGB3_REG_I2C_MST_ODR_CONFIG, 3)
        self._imu._i2c.writeByte(self._imu.address, self._imu.AGB3_REG_I2C_MST_DELAY_CTRL, 0x01)
        self._imu.setBank(0)

    def _publish_mag(self, header):
        """자기센서 값 발행 + **멈춤 감지/자동 복구**.

        왜 필요한가 (2026-08-17 실측):
        AK09916은 ICM-20948 **내부 I2C 마스터**를 통해 읽힌다. 이 자기센서가 응답을
        멈추면 ICM 쪽 EXT_SLV_SENS_DATA 레지스터는 마지막 값을 그대로 붙들고 있다.
        그러면 노드는 아무 에러 없이 죽은 값을 50Hz로 계속 퍼블리시한다.

        개선된 와치독 판정:
          - 로봇이 정지해 있을 때는 자기장 값이 동일한 것이 정상이므로 리셋하지 않음.
          - 로봇이 회전 중(|gz| > 0.05 rad/s)인데도 `mag_stale_sec` 동안 한 번도 안 바뀌면 멈춤으로 판정.
          - 멈춘 동안에는 백그라운드 스레드에서 _init_magnetometer()로 재초기화 시도.
        """
        raw = (self._imu.mxRaw, self._imu.myRaw, self._imu.mzRaw)
        now = time.monotonic()

        # 자이로 회전 상태 판정 (단위 테스트 호환을 위해 getattr 처리)
        gz_raw = getattr(self._imu, 'gzRaw', 1000)
        gz = gz_raw / GYRO_SENSITIVITY_LSB_PER_DPS * DEG_TO_RAD - getattr(self, '_gyro_bias_z', 0.0)
        is_rotating = abs(gz) > 0.05

        if raw != self._mag_last_raw:
            self._mag_last_raw = raw
            self._mag_last_change = now
            if self._mag_frozen:
                self._mag_frozen = False
                self.get_logger().info('자기센서 복구됨 — imu/mag 발행 재개')
        elif not is_rotating:
            # 정지 상태에서는 값이 동일한 것이 정상이므로 타임스탬프 갱신
            self._mag_last_change = now
        elif now - self._mag_last_change > self._mag_stale_sec:
            if not self._mag_frozen:
                self._mag_frozen = True
                self.get_logger().warn(
                    f'자기센서 멈춤 감지: 회전 중 원시값이 {now - self._mag_last_change:.1f}초간 '
                    '변화 없음. 발행을 중단하고 재초기화를 시도한다.')
            if now >= self._mag_recover_at and not self._mag_reinit_busy:
                self._mag_recover_at = now + 5.0
                self._mag_reinit_busy = True

                def _reinit():
                    try:
                        self._init_magnetometer()
                    except Exception as e:
                        self.get_logger().warn(f'자기센서 재초기화 실패: {e}')
                    finally:
                        self._mag_reinit_busy = False

                threading.Thread(target=_reinit, daemon=True).start()
            return  # 멈춘 값은 내보내지 않는다�� 값을 계속 내보내면 하위 소비자가
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
            # ⚠️ 2026-08-21: startupMagnetometer()를 이 콜백에서 직접(동기) 부르면
            # I2C 재초기화가 끝날 때까지 이 콜백 전체가 블록된다 — /imu/data 발행도
            # 같은 콜백 안에서 이루어지므로, 재초기화가 잦을 때(지금처럼 ~2초마다)
            # 가속도/자이로 스트림에도 최대 수백ms~초 단위 공백이 생겼다(실측:
            # ros2 topic hz /imu/data에서 최대 1.5초 공백 관측, 재초기화 빈도와
            # 시간이 일치). 재초기화는 백그라운드 스레드로 넘겨서 /imu/data
            # 발행 루프가 절대 이걸 기다리지 않게 한다.
            if now >= self._mag_recover_at and not self._mag_reinit_busy:
                self._mag_recover_at = now + 5.0
                self._mag_reinit_busy = True

                def _reinit():
                    try:
                        self._imu.startupMagnetometer()
                    except Exception as e:
                        self.get_logger().warn(f'자기센서 재초기화 실패: {e}')
                    finally:
                        self._mag_reinit_busy = False

                threading.Thread(target=_reinit, daemon=True).start()
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
