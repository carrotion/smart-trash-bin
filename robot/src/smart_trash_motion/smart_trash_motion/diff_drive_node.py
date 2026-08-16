"""4WD 차동구동 노드 — /cmd_vel 구독해서 모터 구동, 엔코더로 /odom + TF 발행.

Yahboom 4채널 드라이버와 시리얼로 통신한다. 프로토콜/안전 규약은 전부 CLAUDE.md에
실측으로 기록된 내용을 따름 — 특히:
  - MOTOR_TYPE=1 고정 (TT 프리셋 3/4는 엔코더 리포트가 죽고 정지 명령도 안 먹는
    사고가 있었음, 절대 사용 금지)
  - $pwm: 개루프 구동만 사용 ($spd:는 TT 프리셋 전용으로 추정, 섞어 쓰지 말 것)
  - 시리얼 read는 절대 블로킹하면 안 됨 (in_waiting 없이 read(1)하면 명령당 1초씩
    멈춰서 요청이 밀림 — 실제로 겪은 버그)

물리 파라미터는 2026-08-15 실측값이 기본값 (바퀴 지름 67mm, 좌우 간격 170mm,
1회전당 899.1카운트). 엔코더는 PWM 양수(전진)일 때 카운트가 감소해서 부호 반전 필요.

⚠️ 이 노드는 /dev/ttyUSB0를 독점하므로 motor_dashboard 서비스와 동시 실행 불가.
   `sudo systemctl stop smart-trash-motor-dashboard` 후 실행할 것.
"""
import math
import re
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

_MALL_RE = re.compile(r"^\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#$")


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0335)       # 지름 67mm 실측
        # ⚠️ 물리 트레드는 0.17m지만 여기 쓰는 건 "실효" 트레드(1.94배).
        # 4WD 스키드-스티어는 제자리 회전 시 네 바퀴가 다 옆으로 미끄러져서,
        # 엔코더가 도는 만큼 실제로 안 돌아감 (odom 373.7° vs 실제 193° 실측).
        # 물리 치수를 그대로 넣으면 회전을 2배 과대평가하므로 반드시 실측 보정값 사용.
        # 이 값은 바닥 재질(마루)에 의존 — 카펫 등에서는 재캘리브레이션 필요.
        self.declare_parameter('wheel_separation', 0.3291)
        # 손으로 10회전 돌려 구한 899.1을, 실제 1.16m 주행 대비 odom 1.106m로
        # 나온 비율(0.953)로 보정한 값 — 주행 실측이 더 신뢰도 높음 (2026-08-15)
        self.declare_parameter('counts_per_rev', 857.2)
        self.declare_parameter('encoder_sign', -1)           # PWM+ 전진 시 카운트 감소
        self.declare_parameter('max_pwm', 1200)
        # 모터 스펙 상한(무부하 280RPM × 바퀴원주 0.2105m ≈ 0.98m/s) 기준.
        # PWM 343에서 0.436m/s 실측됨 — 개루프라 명령 속도와 실제가 정확히는 안 맞고,
        # 데드존(min_pwm) 때문에 약 0.3m/s 이하로는 제어가 어려움.
        # PWM 343 → 0.436m/s 실측을 선형 외삽하면 PWM 1200에서 약 1.5m/s.
        # 1.0으로 두면 명령보다 실제가 빨라져 슬라이더 눈금이 안 맞음.
        self.declare_parameter('max_wheel_speed', 1.5)
        # 데드존 보정 하한. 250은 너무 높아서 명령 0.1~0.2m/s가 전부 이 값으로
        # 뭉개졌고, 슬라이더 20%와 40%의 체감 속도가 같아지는 버그가 있었음.
        # PWM 343에서 0.436m/s가 실측됐으니 저속 구간은 이보다 훨씬 아래에 있음.
        self.declare_parameter('min_pwm', 120)
        self.declare_parameter('cmd_vel_timeout', 0.5)       # 이 시간 명령 없으면 정지
        self.declare_parameter('odom_rate_hz', 20.0)
        # 2026-08-16 EKF(robot_localization) 도입 이후 기본값 False로 변경.
        # odom->base_link TF는 이제 ekf_node가 전담 발행함 — 이 노드가 같이 발행하면
        # 두 노드가 같은 TF를 발행하는 충돌(깜빡임/불일치)이 생김.
        self.declare_parameter('publish_tf', False)
        # 2026-08-16 EKF 도입 이후 기본값 False로 변경. 예전엔 이 노드가 직접
        # 자이로를 섞어 odom의 yaw를 계산했는데, 이제 그 융합은 ekf_node가
        # (원본 /imu/data를 직접 구독해서) 전담한다. 이 노드가 내부적으로 이미
        # 자이로를 섞은 odom을 내보내면서 EKF에도 같은 원본 자이로를 또 넣으면
        # 같은 물리 측정값을 이중으로 반영하는 셈이라(공분산이 부당하게 좁아짐),
        # 여기서는 순수 휠 기반 odom만 내보내고 IMU 자이로 융합은 EKF에 맡긴다.
        # (휠 기반 회전각이 부정확한 건 알려진 사실이지만, 그래서 아래 공분산에서
        # yaw/vyaw를 일부러 크게 잡아 EKF가 이 값을 불신하도록 만들어뒀다.)
        self.declare_parameter('use_imu_for_yaw', False)
        self.declare_parameter('gyro_bias_samples', 100)     # 시작 시 정지 상태 바이어스 측정

        p = self.get_parameter
        self._wheel_radius = p('wheel_radius').value
        self._wheel_separation = p('wheel_separation').value
        self._counts_per_rev = p('counts_per_rev').value
        self._encoder_sign = p('encoder_sign').value
        self._max_pwm = p('max_pwm').value
        self._max_wheel_speed = p('max_wheel_speed').value
        self._min_pwm = p('min_pwm').value
        self._cmd_vel_timeout = p('cmd_vel_timeout').value
        self._publish_tf = p('publish_tf').value

        # 카운트 → 미터
        self._meters_per_count = (2.0 * math.pi * self._wheel_radius) / self._counts_per_rev

        self._serial_lock = threading.Lock()
        self._ser = serial.Serial(p('port').value, p('baudrate').value, timeout=0.2)
        time.sleep(0.3)
        self._configure_driver()

        # 오도메트리 상태
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_counts = None
        self._prev_odom_time = None

        self._last_cmd_time = 0.0
        self._target_left_pwm = 0
        self._target_right_pwm = 0

        # IMU 자이로 상태 (use_imu_for_yaw일 때만 사용)
        self._use_imu = p('use_imu_for_yaw').value
        self._gyro_bias_samples_target = p('gyro_bias_samples').value
        self._gyro_bias = 0.0
        self._gyro_bias_acc = 0.0
        self._gyro_bias_count = 0
        self._gyro_z = 0.0          # 바이어스 보정된 최신 각속도
        self._imu_seen = False

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # 4채널 원시 엔코더 누적값 — 웹 대시보드가 바퀴별 상태를 그리는 데 씀
        # (대시보드는 시리얼을 직접 못 열므로 이 노드가 유일한 창구)
        self._encoder_pub = self.create_publisher(Int32MultiArray, 'wheel_encoders', 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        # 큐 깊이 1 — 조종 명령은 "최신 것만" 유효하다.
        # 깊이 10이면 50ms 간격 발행 시 최대 10개(=500ms 분량)가 줄을 서서,
        # 버튼을 떼고 0을 보내도 앞의 주행 명령들이 먼저 처리되며 계속 달렸음
        # (사용자가 보고한 "떼도 500ms 더 감"의 정체).
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 1)
        if self._use_imu:
            self.create_subscription(Imu, 'imu/data', self._on_imu, 20)

        self.create_timer(1.0 / p('odom_rate_hz').value, self._on_odom_timer)
        self.create_timer(0.1, self._on_safety_timer)

        yaw_src = 'IMU 자이로' if p('use_imu_for_yaw').value else '휠 엔코더'
        self.get_logger().info(
            f'diff_drive 시작 — 바퀴반지름 {self._wheel_radius}m, 간격 {self._wheel_separation}m, '
            f'{self._counts_per_rev}카운트/회전, 회전각 출처: {yaw_src}')

    # --- 시리얼 ---

    def _send(self, cmd: bytes, wait=0.02):
        with self._serial_lock:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            time.sleep(wait)
            # in_waiting만큼만 읽음 — 응답 없는 명령($pwm 등)에서 블로킹되지 않게
            return self._ser.read(self._ser.in_waiting)

    def _configure_driver(self):
        # CLAUDE.md 실측 확정값. TYPE 3/4(TT)는 절대 금지.
        for cmd in (b'$mtype:1#', b'$deadzone:1900#', b'$wdiameter:67.00#'):
            resp = self._send(cmd, wait=0.15)
            self.get_logger().info(f'{cmd.decode()} -> {resp!r}')
        self._send(b'$pwm:0,0,0,0#')

    def _read_encoder(self):
        with self._serial_lock:
            self._ser.reset_input_buffer()
            self._ser.write(b'$upload:1#')
            time.sleep(0.04)
            raw = self._ser.read(self._ser.in_waiting)
            self._ser.write(b'$upload:0#')
            time.sleep(0.02)
            self._ser.reset_input_buffer()
        for line in reversed(raw.decode(errors='replace').split('\r\n')):
            m = _MALL_RE.match(line.strip())
            if m:
                return [int(g) for g in m.groups()]
        return None

    def _apply_pwm(self, left_pwm, right_pwm):
        # 채널 순서 M1(왼쪽뒤) M2(왼쪽앞) M3(오른쪽뒤) M4(오른쪽앞) — CLAUDE.md 매핑
        # 4채널 전부 양수 = 전진 (좌우 부호 반전 불필요, 실측 확인됨)
        self._send(f'$pwm:{left_pwm},{left_pwm},{right_pwm},{right_pwm}#'.encode())

    # --- 제어 ---

    def _speed_to_pwm(self, speed_mps):
        """바퀴 선속도(m/s) → PWM. 개루프 선형 근사 + 데드존 보정."""
        if abs(speed_mps) < 1e-4:
            return 0
        ratio = max(-1.0, min(1.0, speed_mps / self._max_wheel_speed))
        pwm = ratio * self._max_pwm
        # 데드존: 아주 작은 값은 모터가 아예 안 도니 최소 구동값까지 끌어올림
        if 0 < abs(pwm) < self._min_pwm:
            pwm = math.copysign(self._min_pwm, pwm)
        return int(max(-self._max_pwm, min(self._max_pwm, pwm)))

    def _on_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        half_track = self._wheel_separation / 2.0
        left_speed = v - w * half_track
        right_speed = v + w * half_track

        self._target_left_pwm = self._speed_to_pwm(left_speed)
        self._target_right_pwm = self._speed_to_pwm(right_speed)
        self._last_cmd_time = time.time()
        self._apply_pwm(self._target_left_pwm, self._target_right_pwm)

    def _on_imu(self, msg: Imu):
        """자이로 z를 받아 바이어스 보정 후 보관. 회전각은 이 값으로 적분한다."""
        wz = msg.angular_velocity.z

        # 시작 직후 정지 상태에서 바이어스(정지 시 0이 아닌 오프셋) 측정.
        # 이걸 안 빼면 가만히 있어도 각도가 계속 돈다.
        if self._gyro_bias_count < self._gyro_bias_samples_target:
            if self._target_left_pwm == 0 and self._target_right_pwm == 0:
                self._gyro_bias_acc += wz
                self._gyro_bias_count += 1
                if self._gyro_bias_count == self._gyro_bias_samples_target:
                    self._gyro_bias = self._gyro_bias_acc / self._gyro_bias_count
                    self.get_logger().info(
                        f'자이로 바이어스 보정 완료: {self._gyro_bias:+.5f} rad/s')
            return

        self._gyro_z = wz - self._gyro_bias
        self._imu_seen = True

    def _on_safety_timer(self):
        """cmd_vel이 끊기면 자동 정지 — 명령 유실/노드 죽음 대비."""
        if (self._target_left_pwm or self._target_right_pwm) and \
                time.time() - self._last_cmd_time > self._cmd_vel_timeout:
            self._target_left_pwm = 0
            self._target_right_pwm = 0
            self._apply_pwm(0, 0)
            self.get_logger().warn('cmd_vel 타임아웃 — 정지')

    # --- 오도메트리 ---

    def _on_odom_timer(self):
        counts = self._read_encoder()
        now = time.time()
        if counts is None:
            return

        enc_msg = Int32MultiArray()
        enc_msg.data = [int(c) for c in counts]
        self._encoder_pub.publish(enc_msg)

        if self._prev_counts is None:
            self._prev_counts = counts
            self._prev_odom_time = now
            return

        dt = now - self._prev_odom_time
        if dt <= 0:
            return

        # 좌/우 각각 앞뒤 바퀴 평균 (M1,M2=왼쪽 / M3,M4=오른쪽)
        d = [(counts[i] - self._prev_counts[i]) * self._encoder_sign * self._meters_per_count
             for i in range(4)]
        d_left = (d[0] + d[1]) / 2.0
        d_right = (d[2] + d[3]) / 2.0
        self._prev_counts = counts
        self._prev_odom_time = now

        d_center = (d_left + d_right) / 2.0

        # 전진 거리는 휠 엔코더(미끄러짐 적음), 회전각은 IMU 자이로(미끄러짐 무관).
        # 스키드-스티어에서 휠 기반 회전각은 바닥/속도/마모에 따라 매번 달라져서 못 씀.
        if self._use_imu and self._imu_seen:
            d_theta = self._gyro_z * dt
        else:
            d_theta = (d_right - d_left) / self._wheel_separation

        # 중간각 적분 — 회전 중 직선 근사보다 오차가 작음
        mid_theta = self._theta + d_theta / 2.0
        self._x += d_center * math.cos(mid_theta)
        self._y += d_center * math.sin(mid_theta)
        self._theta = math.atan2(math.sin(self._theta + d_theta),
                                 math.cos(self._theta + d_theta))

        stamp = self.get_clock().now().to_msg()
        q = self._yaw_to_quaternion(self._theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        # EKF(robot_localization)가 이 값으로 신뢰도를 판단한다. 휠 기반 선속도/거리는
        # 비교적 정확하지만(직선 주행 검증 완료), 회전각은 스키드-스티어 미끄러짐 때문에
        # 신뢰 불가 — yaw/vyaw 분산을 크게 잡아 EKF가 IMU 쪽을 우선하게 만든다.
        # 6x6 대각행렬만 채움(row-major, [x,y,z,roll,pitch,yaw] 순서).
        odom.pose.covariance[0] = 0.02    # x, m^2
        odom.pose.covariance[7] = 0.02    # y, m^2
        odom.pose.covariance[35] = 999.0  # yaw, rad^2 — 사실상 무시하라는 신호
        odom.twist.covariance[0] = 0.02   # vx, (m/s)^2
        odom.twist.covariance[35] = 999.0  # vyaw, (rad/s)^2 — 사실상 무시

        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def stop_and_close(self):
        try:
            for _ in range(3):
                self._apply_pwm(0, 0)
                time.sleep(0.05)
        finally:
            self._ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 어떤 경로로 종료되든 모터는 반드시 멈추고 나감
        node.stop_and_close()
        node.destroy_node()
        # systemd/pkill의 SIGTERM으로 이미 shutdown됐을 수 있어 중복 호출 방지
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
