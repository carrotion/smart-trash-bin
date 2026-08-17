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
        # 직진 트림 보정. 2026-08-16 사용자 관찰: 전진 명령을 줘도 미세하게
        # 오른쪽으로 쏠림 — 2026-08-14 기록된 M4(오른쪽앞) 인코더 반응 저하와
        # 일치(같은 PWM에도 오른쪽이 덜 밀림 → 우측으로 쏠림). 양수면 오른쪽
        # 바퀴를 더 세게, 왼쪽을 더 약하게 돌려 직진을 보정한다. 순수 직진/
        # 후진(각속도 0)일 때만 적용 — 회전 중엔 트림이 오히려 방해가 됨.
        # 실측 캘리브레이션(2026-08-16, 0.3m/s 직진 테스트, cliff test로 범위까지 확정):
        #   trim=0      -> 1.250m 이동, 우측 78mm(앞)/73mm(뒤)
        #   trim=0.016  -> 1.125m 이동, 우측 63mm(앞)/48mm(뒤)
        #   trim=0.11   -> 1.185m 이동, 우측 43mm(앞)/32mm(뒤)
        #   trim=0.248  -> 0.970m 이동, 우측 25mm(앞)/15mm(뒤)
        #   trim=0.35   -> 0.540m 이동, 쏠림 없음
        #   trim=0.5    -> 0.788m 이동, 우측 4mm(앞)/0mm(뒤)
        #   trim=0.7    -> 우측 0mm(앞)/좌측 2mm(뒤) — 부호 전환 확인
        # trim을 올릴수록 개선 기울기가 급격히 줄어드는 것처럼 보였는데(수확체감),
        # 사용자가 지적한 대로 원인은 두 가지가 섞여 있었기 때문: ①주행 중 편향
        # (trim으로 보정되는 부분, 실제로 계속 선형에 가깝게 줄어듦)과 ②정지 시
        # 관성으로 원래 기계적 얼라인먼트로 되돌아가는 편향(trim과 무관하게 항상
        # 존재, 같은 방향으로 고정). trim이 커질수록 ①은 0에 가까워지지만 ②는
        # 안 줄어드니 총합이 잘 안 줄어드는 것처럼 보인 것. 다만 실제 조종에서도
        # 버튼을 누르고 뗄 때마다 ②가 똑같이 일어나므로, "정지 후 최종 위치 편차"
        # 자체가 실사용과 가까운 지표라 이 기준으로 캘리브레이션하는 게 맞음.
        # 0.5~0.7 구간에서 오차 1% 미만(부호 전환 지점 포함) 확인 -> 중간값 채택.
        # 사용 가능 범위(오차 <1%): trim 0.5~0.7. 채택값: 0.6.
        self.declare_parameter('straight_trim', 0.6)
        # 회전 시 파워 부스트. 스키드-스티어는 제자리 회전할 때 바퀴가 옆으로
        # 미끄러지는 마찰을 이겨내야 해서 직진보다 훨씬 큰 토크가 필요함 —
        # 사용자가 "좌회전이 잘 안 돈다"고 보고(2026-08-16). 회전 성분(각속도로
        # 인한 좌우 속도차)에만 곱해서 부스트하고 직진(v) 성분은 그대로 둔다.
        # 실측 캘리브레이션 전 초기값 — 필요시 straight_trim처럼 실측으로 조정.
        # --- 회전 전용 PWM 모델 (2026-08-17 신설) ---
        # 직진용 속도→PWM 환산과 **완전히 분리**한다. 이유는 _turn_pwm() 주석 참고.
        # 아래 3개는 실측으로 확정할 값 — 우선 보수적 초기값을 둔다.
        # 실측 캘리브레이션 (2026-08-17, 마루바닥, 자이로 각속도 직접 측정):
        #   PWM 150 -> 0.038 rad/s      PWM 400 -> 0.530
        #   PWM 200 -> 0.095            PWM 500 -> 0.773
        #   PWM 250 -> 0.187            PWM 650 -> 1.294
        #   PWM 300 -> 0.224            PWM 800 -> 1.826
        # PWM 300 이상은 선형: 대략 w = 0.0032*PWM - 0.74 (100 PWM당 0.32 rad/s).
        # 정지마찰 문턱이 PWM 230 부근 — 그 아래로는 명령을 줘도 거의 안 돈다.
        #
        # 검증 결과(명령 대비 실제): 0.5 -> 93%, 0.8 -> 102%.
        # ⚠️ 저속은 구조적으로 넘친다(0.2 -> 175%). 마찰 문턱 때문에 어떤 작은
        #    명령이든 최소 PWM이 나가야 움직이기 때문. 안 도는 것보다 낫다고 판단.
        #    근본 해결은 자이로 피드백 기반 폐루프 각속도 제어(미구현).
        # ⚠️ 이 값들은 **바닥 재질·배터리 전압·모터 온도에 의존**한다. 카펫이나
        #    배터리가 빠진 상태에서는 재캘리브레이션 필요. 유닛마다도 다르므로
        #    2호기부터는 unit calibration 파일로 분리할 것(docs/fleet_and_lifecycle.md).
        self.declare_parameter('min_turn_pwm', 260)
        self.declare_parameter('max_turn_pwm', 570)
        self.declare_parameter('max_angular_speed', 1.0)

        # --- 폐루프 회전 제어 (2026-08-17) ---
        # 개루프는 "PWM 얼마면 몇 rad/s"를 표로 외우는 방식이라 바닥 재질·배터리
        # 전압·모터 온도가 바뀌면 그대로 틀어진다. 자이로로 실제 각속도를 재서
        # PWM을 보정하면 그 변화를 알아서 흡수한다. 개루프 캘리브레이션 값은
        # 피드포워드로 그대로 쓰고(응답 빠름), PI가 잔차만 메운다.
        # 실주행 검증(2026-08-17, 마루바닥, 제자리 회전):
        #   명령 0.3 -> 81%,  0.6 -> 96%,  1.0 -> 93%  (5초 유지 시 최종값)
        #   수렴 거동: 1초째 피드포워드로 앞서고, 2초째 적분이 과보정했다가
        #   3초부터 안정. 개루프(93~175%) 대비 저속 구간이 크게 개선됨.
        # 폐루프의 진짜 값어치는 지금의 정확도보다 **조건 변화에 대한 강건성**이다 —
        # 배터리가 빠지거나 바닥이 바뀌어도 자이로가 실측하므로 알아서 맞춘다.
        # 개루프는 그때마다 min/max_turn_pwm을 다시 재야 한다.
        self.declare_parameter('turn_closed_loop', True)
        self.declare_parameter('turn_kp', 250.0)      # PWM per (rad/s) 오차
        self.declare_parameter('turn_ki', 400.0)      # PWM per (rad/s·s)
        self.declare_parameter('turn_i_limit', 250.0) # 적분항이 낼 수 있는 최대 PWM
        # 자이로가 이 시간보다 오래 갱신 안 되면 폐루프를 끄고 피드포워드만 쓴다.
        self.declare_parameter('gyro_stale_sec', 0.3)
        self.declare_parameter('control_rate_hz', 50.0)

        # (구) turn_boost — 회전 PWM 모델로 대체되어 더 이상 쓰이지 않는다.
        # 파라미터 자체는 호환을 위해 남겨두되 제어에는 반영 안 함.
        self.declare_parameter('turn_boost', 1.4)
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
        self._straight_trim = p('straight_trim').value
        self._turn_boost = p('turn_boost').value   # 미사용(회전 PWM 모델로 대체)
        self._min_turn_pwm = p('min_turn_pwm').value
        self._max_turn_pwm = p('max_turn_pwm').value
        self._max_angular_speed = p('max_angular_speed').value
        self._turn_closed_loop = p('turn_closed_loop').value
        self._turn_kp = p('turn_kp').value
        self._turn_ki = p('turn_ki').value
        self._turn_i_limit = p('turn_i_limit').value
        self._gyro_stale_sec = p('gyro_stale_sec').value
        self._publish_tf = p('publish_tf').value

        # 런타임 파라미터 변경 반영 (2026-08-17 추가).
        # 이게 없으면 `ros2 param set`이 파라미터 서버 값만 바꾸고 노드 내부
        # 변수는 그대로라, 실측 캘리브레이션을 하면서 값을 바꿔도 **아무 효과가
        # 없는데 성공했다고 착각**하게 된다(실제로 회전 PWM 스윕 전체를 헛돌렸음).
        self.add_on_set_parameters_callback(self._on_params)


        # 카운트 → 미터
        self._meters_per_count = (2.0 * math.pi * self._wheel_radius) / self._counts_per_rev

        self._serial_lock = threading.Lock()
        self._ser = serial.Serial(p('port').value, p('baudrate').value, timeout=0.2)
        time.sleep(0.3)
        self._configure_driver()

        # 엔코더 폴링을 ROS 콜백에서 분리한 이유(2026-08-16): 시리얼 왕복(약 60ms,
        # sleep 0.04+0.02)이 odom_rate_hz(20Hz=50ms 주기)보다 길어서, 이 폴링을
        # ROS 타이머 콜백에서 직접 하면 단일 스레드 실행기가 구조적으로 항상
        # 밀려있는 상태가 됨 — 그 사이 도착한 cmd_vel(버튼 명령)을 실행기가
        # 처리를 시작조차 못 해서 "버튼이 씹히는" 현상으로 나타났음(실측: /odom
        # 목표 20Hz인데 실제 12~19Hz, 지연 최대 363ms 관측). 별도 스레드가
        # 백그라운드로 계속 폴링하고, ROS 타이머는 캐시된 최신값만 읽어서
        # 즉시 발행 — cmd_vel 콜백이 더 이상 엔코더 폴링에 막히지 않는다.
        self._counts_lock = threading.Lock()
        self._latest_counts = None
        self._latest_counts_ts = None
        self._encoder_thread_stop = threading.Event()
        self._encoder_thread = threading.Thread(
            target=self._encoder_poll_loop, daemon=True)
        self._encoder_thread.start()

        # 오도메트리 상태
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_counts = None
        self._prev_odom_time = None

        self._last_cmd_time = 0.0
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._turn_i = 0.0            # 회전 폐루프 적분항
        self._prev_control_time = None
        self._gyro_ts = None          # 자이로 최신 수신 시각(신선도 판단용)
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
        # ⚠️ 자이로는 **항상** 구독한다. use_imu_for_yaw가 False여도(=odom yaw는
        # EKF가 담당) 아래 폐루프 회전 제어에 실측 각속도가 필요하기 때문.
        # 예전엔 이 구독이 use_imu_for_yaw에 묶여 있어서, EKF 도입 후 자이로가
        # 이 노드에 아예 안 들어오고 있었다.
        self.create_subscription(Imu, 'imu/data', self._on_imu, 20)

        self.create_timer(1.0 / p('odom_rate_hz').value, self._on_odom_timer)
        self.create_timer(0.1, self._on_safety_timer)
        self.create_timer(1.0 / p('control_rate_hz').value, self._on_control_timer)

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

    def _encoder_poll_loop(self):
        """백그라운드 스레드: 시리얼 왕복(~60ms)이 걸리는 엔코더 폴링을 ROS
        실행기 스레드 밖에서 계속 반복한다. cmd_vel 콜백이 이 폴링에 막히지
        않게 하는 게 목적 — 클래스 docstring 및 __init__ 주석 참고."""
        while not self._encoder_thread_stop.is_set():
            counts = self._read_encoder()
            if counts is not None:
                now = time.time()
                with self._counts_lock:
                    self._latest_counts = counts
                    self._latest_counts_ts = now
                enc_msg = Int32MultiArray()
                enc_msg.data = [int(c) for c in counts]
                try:
                    self._encoder_pub.publish(enc_msg)
                except Exception:
                    pass  # 노드 종료 중 퍼블리셔가 이미 정리됐을 수 있음
            else:
                time.sleep(0.02)  # 응답 없을 때 바쁜 대기 방지

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

    def _on_params(self, params):
        """캘리브레이션 중 바꿀 값들만 런타임 반영. 시리얼/포트 관련은 제외."""
        from rcl_interfaces.msg import SetParametersResult
        live = {
            'straight_trim': '_straight_trim',
            'min_turn_pwm': '_min_turn_pwm',
            'max_turn_pwm': '_max_turn_pwm',
            'max_angular_speed': '_max_angular_speed',
            'max_wheel_speed': '_max_wheel_speed',
            'min_pwm': '_min_pwm',
            'cmd_vel_timeout': '_cmd_vel_timeout',
            'turn_closed_loop': '_turn_closed_loop',
            'turn_kp': '_turn_kp',
            'turn_ki': '_turn_ki',
            'turn_i_limit': '_turn_i_limit',
        }
        for prm in params:
            attr = live.get(prm.name)
            if attr:
                setattr(self, attr, prm.value)
        return SetParametersResult(successful=True)

    def _turn_pwm(self, w):
        """회전 명령(rad/s) → 좌우 PWM 차이. **직진 모델과 완전히 분리한다.**

        왜 분리하나 (2026-08-17 실측):
        예전엔 회전도 `w * half_track * turn_boost`로 선속도를 만든 뒤 직진용
        `_speed_to_pwm()`에 통과시켰다. 그런데 스키드-스티어는 제자리 회전 시
        바퀴 4개가 옆으로 긁혀야 해서 저항이 직진과 근본적으로 다르다. 그 결과
        **0.3~1.0 rad/s 어느 값을 줘도 실제로는 명령의 2~9%밖에 안 돌았다**
        (자이로·휠엔코더 양쪽으로 확인, 로봇이 자유로운 상태에서 측정).
        w=0.8일 때 PWM이 겨우 147이었는데, 실측 기록상 의미 있는 회전에는
        500 정도가 필요하다.

        비례식으로는 못 고친다 — 정지마찰을 못 이기는 구간에서는 PWM을 조금
        올려도 회전이 0 그대로다. 그래서 **최소 구동 PWM을 절대값으로 보장**하고,
        거기서부터 최대까지 선형 보간한다. 어떤 0이 아닌 회전 명령이든 적어도
        `min_turn_pwm`은 나가므로 "명령했는데 꿈쩍도 안 하는" 상황이 사라진다.

        회전이 안 되면 경로를 못 따라가고, 방향을 못 바꿔 장애물에 그대로
        박는다 — 실제로 의자 다리에 충돌했다. 그래서 이건 주행 품질이 아니라
        안전 문제에 가깝다.
        """
        if abs(w) < 1e-6:
            return 0
        ratio = min(1.0, abs(w) / self._max_angular_speed)
        pwm = self._min_turn_pwm + ratio * (self._max_turn_pwm - self._min_turn_pwm)
        pwm = min(pwm, self._max_pwm)
        return int(math.copysign(pwm, w))

    def _on_cmd_vel(self, msg: Twist):
        """목표만 저장한다. 실제 PWM 계산·적용은 _on_control_timer가 담당.

        예전엔 여기서 바로 PWM을 적용했는데, 폐루프 회전 제어를 넣으면서
        고정 주기로 오차를 적분해야 해서 제어 루프로 옮겼다. 지연은
        control_rate_hz(50Hz=20ms)라 조종 반응성에는 영향 없음
        (예전에 문제였던 건 363ms 지연이었다).
        """
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self._last_cmd_time = time.time()

    def _on_control_timer(self):
        """50Hz 제어 루프 — 직진은 개루프, 회전은 자이로 피드백 폐루프."""
        now = time.time()

        # 안전: 명령이 끊기면 즉시 정지하고 적분항도 비운다.
        if now - self._last_cmd_time > self._cmd_vel_timeout:
            if self._target_left_pwm or self._target_right_pwm:
                self._target_left_pwm = 0
                self._target_right_pwm = 0
                self._apply_pwm(0, 0)
            self._turn_i = 0.0
            self._prev_control_time = now
            return

        dt = now - self._prev_control_time if self._prev_control_time else 0.02
        self._prev_control_time = now
        dt = min(max(dt, 0.001), 0.1)

        v, w = self._cmd_v, self._cmd_w

        # --- 직진 성분 (개루프) ---
        left_speed = right_speed = v
        if w == 0.0 and self._straight_trim != 0.0:
            left_speed *= (1.0 - self._straight_trim)
            right_speed *= (1.0 + self._straight_trim)
        left_pwm = self._speed_to_pwm(left_speed)
        right_pwm = self._speed_to_pwm(right_speed)

        # --- 회전 성분 ---
        turn = float(self._turn_pwm(w))          # 캘리브레이션 기반 피드포워드
        gyro_fresh = (self._gyro_ts is not None
                      and now - self._gyro_ts < self._gyro_stale_sec)

        if self._turn_closed_loop and abs(w) > 1e-6 and gyro_fresh:
            err = w - self._gyro_z
            self._turn_i += err * dt
            # 적분 와인드업 방지: 적분항이 낼 수 있는 PWM을 직접 제한
            if self._turn_ki > 0:
                lim = self._turn_i_limit / self._turn_ki
                self._turn_i = max(-lim, min(lim, self._turn_i))
            turn += self._turn_kp * err + self._turn_ki * self._turn_i
        else:
            self._turn_i = 0.0

        turn = max(-self._max_pwm, min(self._max_pwm, turn))
        left_pwm -= turn
        right_pwm += turn

        clamp = lambda x: int(max(-self._max_pwm, min(self._max_pwm, x)))
        self._target_left_pwm = clamp(left_pwm)
        self._target_right_pwm = clamp(right_pwm)
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
        self._gyro_ts = time.time()
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
        # 실제 시리얼 폴링은 백그라운드 스레드(_encoder_poll_loop)가 하고,
        # 여기서는 캐시된 최신값만 읽는다 — 이 콜백이 절대 블로킹하지 않아야
        # 같은 ROS 실행기를 쓰는 cmd_vel 콜백이 밀리지 않는다.
        with self._counts_lock:
            counts = self._latest_counts
            now = self._latest_counts_ts
        if counts is None:
            return

        if self._prev_odom_time is not None and now == self._prev_odom_time:
            return  # 백그라운드 스레드가 아직 새 값을 못 읽음 — 중복 계산 방지

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
        self._encoder_thread_stop.set()
        try:
            for _ in range(3):
                self._apply_pwm(0, 0)
                time.sleep(0.05)
        finally:
            self._encoder_thread.join(timeout=1.0)
            self._ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        # 종료 신호와 타이머 콜백의 publish()가 겹치면 "context is invalid"로
        # 뜨는 무해한 경합(2026-08-16 확인) — finally에서 정리는 어차피 되므로
        # 그냥 삼킨다. 안 잡으면 매 재시작마다 지저분한 트레이스백만 남았음.
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
