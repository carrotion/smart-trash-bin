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
from tf2_ros import TransformBroadcaster

_MALL_RE = re.compile(r"^\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#$")


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0335)       # 지름 67mm 실측
        self.declare_parameter('wheel_separation', 0.17)     # 좌우 바퀴 간격 실측
        self.declare_parameter('counts_per_rev', 899.1)      # 2026-08-15 실측 캘리브레이션
        self.declare_parameter('encoder_sign', -1)           # PWM+ 전진 시 카운트 감소
        self.declare_parameter('max_pwm', 1200)
        self.declare_parameter('max_wheel_speed', 0.35)      # m/s, max_pwm에 대응 (미실측 추정치)
        self.declare_parameter('min_pwm', 250)               # 데드존 — 이보다 작으면 안 움직임
        self.declare_parameter('cmd_vel_timeout', 0.5)       # 이 시간 명령 없으면 정지
        self.declare_parameter('odom_rate_hz', 20.0)
        self.declare_parameter('publish_tf', True)

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

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        self.create_timer(1.0 / p('odom_rate_hz').value, self._on_odom_timer)
        self.create_timer(0.1, self._on_safety_timer)

        self.get_logger().info(
            f'diff_drive 시작 — 바퀴반지름 {self._wheel_radius}m, 간격 {self._wheel_separation}m, '
            f'{self._counts_per_rev}카운트/회전')

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
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
