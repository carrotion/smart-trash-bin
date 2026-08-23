import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import qwiic_icm20948
from geometry_msgs.msg import Twist

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

    def _init_magnetometer(self):
        """AK09916 초기화 및 ICM-20948 보조 I2C 마스터 ODR(137.5Hz) & STOP 조건(0x17) 설정."""
        self._imu.startupMagnetometer()
        self._imu.setBank(3)
        self._imu._i2c.writeByte(self._imu.address, self._imu.AGB3_REG_I2C_MST_ODR_CONFIG, 3)
        self._imu._i2c.writeByte(self._imu.address, self._imu.AGB3_REG_I2C_MST_CTRL, 0x17)
        self._imu._i2c.writeByte(self._imu.address, self._imu.AGB3_REG_I2C_MST_DELAY_CTRL, 0x01)
        self._imu.setBank(0)

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
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.declare_parameter('gyro_bias_z', 0.0)
        self._gyro_bias_z = self.get_parameter('gyro_bias_z').value
        if self._gyro_bias_z:
            self.get_logger().info(f'자이로 Z 바이어스 보정 적용: {self._gyro_bias_z:+.5f} rad/s')
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

        # 2026-08-23: 왕복(전진-정지-후진) 진단(tools/round_trip_gyro_diag.py)에서
        # 발견 — 모터 시작/정지/역전 순간에 자이로 z가 순간적으로 크게 튄다
        # (정지 상태 기준 표준편차 0.057rad/s의 5~8배, 예: +0.31~+0.44rad/s).
        # 로봇이 그렇게 빨리 회전한 게 아니라 모터 전류 서지/진동이 자이로에
        # 노이즈로 잡힌 것으로 추정 — 그런데 EKF는 이걸 실제 회전으로 믿고
        # 적분해서, 왕복 20회 후 요 이탈이 휠 단독(13.1°)보다 EKF 쪽이 더
        # 컸다(33.1°, 원래는 EKF가 더 나아야 정상).
        # 같은 로그의 전환-이후-경과시간별 |자이로z| 평균을 보면 20~100ms 구간이
        # 정지 기준선 대비 1.1~1.8배로 들뜨고 120~150ms부터 기준선으로 돌아온다
        # (표본 15개, 거칠지만 방향은 뚜렷함 — 모터 캘리브레이션 때처럼 반복
        # cliff test로 다듬을 여지 있음). 데이터를 버리지 않고 **공분산을
        # 부풀려서 EKF가 알아서 덜 믿게** 하는 쪽을 택함 — 이미 이 노드가
        # orientation_covariance[0]=-1로 "이 필드는 못 믿는다"를 표시하던
        # 방식과 같은 원리.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gyro_settle_after_cmd_sec', 0.15)
        self.declare_parameter('gyro_unsettled_covariance', 50.0)  # 정상치(0.0004)보다 훨씬 큼
        self._gyro_settle_sec = self.get_parameter('gyro_settle_after_cmd_sec').value
        self._gyro_unsettled_cov = self.get_parameter('gyro_unsettled_covariance').value
        self._last_cmd_v = 0.0
        self._last_cmd_w = 0.0
        self._last_cmd_change_t = 0.0  # 시작하자마자는 "막 전환된" 것으로 보수적으로 취급
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd_vel, 10)

        self.declare_parameter('mag_stale_sec', 2.0)
        self._mag_stale_sec = self.get_parameter('mag_stale_sec').value
        self._mag_last_raw = None
        self._mag_last_change = time.monotonic()
        self._mag_recover_at = 0.0
        self._mag_frozen = False
        self._mag_reinit_busy = False   # 재초기화 스레드 중복 실행 방지

        try:
            self._init_magnetometer()
            self._mag_ok = True
            self.get_logger().info('자기센서(AK09916) 초기화 및 ODR 동기화(137.5Hz) 완료 — imu/mag 발행')
        except Exception as e:
            self._mag_ok = False
            self.get_logger().warn(f'자기센서 초기화 실패: {e} (IMU 나머지는 정상 동작)')
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(f'ICM-20948 연결됨, {rate_hz}Hz로 imu/data 퍼블리시 시작')

    def _on_cmd_vel(self, msg):
        # 부호가 바뀌거나 정지<->구동 전환이면 "모터 전환"으로 본다. 아주 미세한
        # 값 흔들림(PID 잔진동 등)까지 매번 반응하지 않게 임계값을 둠.
        eps = 1e-3
        if (abs(msg.linear.x - self._last_cmd_v) > eps
                or abs(msg.angular.z - self._last_cmd_w) > eps):
            self._last_cmd_change_t = time.monotonic()
        self._last_cmd_v = msg.linear.x
        self._last_cmd_w = msg.angular.z

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
        msg.angular_velocity.z = (self._imu.gzRaw / GYRO_SENSITIVITY_LSB_PER_DPS * DEG_TO_RAD
                                   - self._gyro_bias_z)

        # This node doesn't fuse orientation — robot_localization EKF (Week 4) does
        # that downstream. Mark orientation unknown per REP-145.
        msg.orientation_covariance[0] = -1.0

        # EKF가 신뢰도 가중에 쓰는 값. ICM-20948 자이로/가속도계 노이즈에 대한
        # 보수적인 추정치(정밀 캘리브레이션 전) — 대각항만 채움, 축간 상관 없음으로 가정.
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.0004       # (rad/s)^2
            msg.linear_acceleration_covariance[i] = 0.02      # (m/s^2)^2

        # 모터 전환 직후 settle 구간이면 z축(요 회전에 쓰이는 축)만 공분산을
        # 크게 부풀려서 EKF가 이 순간의 자이로z를 덜 믿게 한다. x/y는 안 건드림
        # (진단 데이터가 z만 다뤘고, 평지 주행에선 x/y 자이로가 요 추정에 안 쓰임).
        since_cmd = time.monotonic() - self._last_cmd_change_t
        if since_cmd < self._gyro_settle_sec:
            msg.angular_velocity_covariance[8] = self._gyro_unsettled_cov

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
            # 정지 상태에서는 자기장 값이 동일한 것이 정상이므로 타임스탬프 갱신
            self._mag_last_change = now
        elif now - self._mag_last_change > self._mag_stale_sec:
            if not self._mag_frozen:
                self._mag_frozen = True
                self.get_logger().warn(
                    f'자기센서 멈춤 감지: 회전 중 원시값이 {now - self._mag_last_change:.1f}초간 '
                    '변화 없음. 발행을 중단하고 재초기화를 시도한다.')
            # ⚠️ 2026-08-21: startupMagnetometer()를 이 콜백에서 직접(동기) 부르면
            # I2C 재초기화가 끝날 때까지 이 콜백 전체가 블록된다 — /imu/data 발행도
            # 같은 콜백 안에서 이루어지므로, 재초기화가 잦을 때(지금처럼 ~2초마다)
            # 가속도/자이로 스트림에도 최대 수백ms~초 단위 공백이 생겼다(실측:
            # ros2 topic hz /imu/data에서 최대 1.5초 공백 관측, 재초기화 빈도와
            # 시간이 일치). 재초기화는 백그라운드 스레드로 넘겨서 /imu/data
            # 발행 루프가 절대 이걸 기다리지 않게 한다.
            if now >= self._mag_recover_at and not getattr(self, '_mag_reinit_busy', False):
                self._mag_recover_at = now + 5.0
                self._mag_reinit_busy = True

                def _reinit():
                    try:
                        if hasattr(self, '_init_magnetometer'):
                            self._init_magnetometer()
                        else:
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
        #
        # ⚠️ 2026-08-21 밤 01f5a38이 여기 X축에 "차체 전후 180° 반전(base_link 반전)"을
        # 근거로 부호(-)를 추가했었는데, 2026-08-26 팩트체크로 되돌림 — 그런 반전은
        # 문서 어디에도 없다(08-17 라이다 상판→하판 재장착은 라이다 자체 얘기고,
        # 같은 날 IMU 재장착(b65fef5)도 "PCB 앞쪽 끝단, z 유지"라는 위치 이동이지
        # 회전이 아니다). 같은 날 밤 dfb71af가 sensor_dashboard.py의 atan2 부호도
        # 반대로 되돌려서, 두 반전이 우연히 상쇄되지 않고 겹쳐 정확히 180°(+잔차)
        # 틀어진 방위가 나왔다 — 사용자가 실제로 동쪽을 보고 있는데 대시보드가
        # 282°(서쪽 근처)를 표시해서 발견됨. 라이브 원시값(x=4.05, y=-18.6µT)에
        # 이 줄의 원래(부호 없는) 형태 + dashboard의 원래 -atan2(y,x)를 같이 적용하면
        # 77.7°로 동쪽에 가깝게 복원됨 — 두 부호는 항상 같이 검증할 것.
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
