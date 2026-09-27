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

2026-08-24 정정: 아래 경고는 낡은 문서였다 — motor_dashboard.py는 이미 `340fdc5`
커밋(과거)에서 시리얼 직결이 아니라 **`/cmd_vel` 발행 방식**으로 바뀌었다.
그래서 지금은 이 노드와 **동시 실행해도 포트 경합이 없다**(motor_dashboard가
/dev/motor를 아예 안 만짐). 실물 매핑 시험 중 엔코더가 35초간 조용히 멈추는
현상을 motor_dashboard 탓으로 오판해서 껐다가, 코드를 직접 보고서야 틀렸음을
확인 — 진짜 원인은 아직 미확정이다(_on_odom_timer의 새 stale 경고 참고).
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
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster

_MALL_RE = re.compile(r"^\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#$")


class DiffDriveNode(Node):
    # 채널(인덱스=드라이버 채널-1)의 좌/우 소속. 2026-09-24 재배선 기준
    # M1=왼쪽앞, M2=오른쪽앞, M3=왼쪽뒤, M4=오른쪽뒤. 구동(_apply_pwm)과
    # 오도메트리가 둘 다 이 값을 쓴다 — 예전엔 두 곳에 따로 박혀 있어서
    # 재배선 때 한쪽만 고쳐질 위험이 있었다. tools/wheel_config.py와 일치해야 함.
    LEFT = (0, 2)
    RIGHT = (1, 3)

    def __init__(self):
        super().__init__('diff_drive_node')

        # ⚠️ /dev/ttyUSB0 대신 **udev 고정 이름**을 쓴다(tools/udev/99-smart-trash.rules).
        # 2026-08-19 라이다가 USB로 옮겨오면서 USB 직렬 장치가 2개가 됐다.
        # ttyUSB 번호는 부팅 시 인식 순서로 정해지므로, 뒤바뀌면 이 노드가
        # **라이다 포트를 열고 모터 명령을 라이다로 쏜다.**
        self.declare_parameter('port', '/dev/motor')
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
        # ⚠️ 이 값은 이제 **앞바퀴(M1,M2,370모터) 전용**이다 — 아래 참고.
        self.declare_parameter('counts_per_rev', 857.2)
        # 뒷바퀴(M3,M4,520모터, 2026-09-24 교체) 전용 값. 손으로 10회전 돌려
        # 실측: 24756카운트/10회전 = 2475.6. 검산: 11펄스/회전 x 쿼드러처(x4)
        # x 감속비56 = 2464 — 실측치와 거의 일치(주문한 감속비 56 확인됨).
        # ⚠️ 370의 857.2처럼 아직 실주행 거리 대비 보정(0.953 같은 계수)을
        # 거치지 않은 **1차 raw 값** — 다음 정밀주행 검증에서 다시 볼 것.
        self.declare_parameter('counts_per_rev_rear', 2475.6)
        # 뒷바퀴(520) PWM 보정 — 앞바퀴(370)와 같은 PWM에서 같은 실제 속도가
        # 나오게 하는 affine 변환(rear_pwm = slope*front_pwm + offset).
        # 도크 무부하 스윕(tools/axle_speed_sweep.py, PWM 200~800) 선형회귀로
        # 유도 — 상세 근거는 _apply_pwm 주석 참고. ⚠️ PWM 120~200(데드존~
        # 실측범위) 구간은 외삽이라 저속 재검증 필요.
        self.declare_parameter('rear_pwm_affine_slope', 1.7692)
        self.declare_parameter('rear_pwm_affine_offset', 461.82)
        # 채널별 부호 (2026-09-24 실측, docs/hardware_change_procedure.md 1장 3~4번).
        # encoder_signs: 전진할 때 원시 카운트가 줄면 -1. pwm_signs: 드라이버에 양수를
        # 주면 후진하는 채널은 -1(오른쪽 두 채널이 반대 극성으로 배선됨).
        # tools/wheel_config.py의 ENCODER_SIGN/PWM_SIGN과 같아야 함(hw_preflight.py 검사).
        self.declare_parameter('encoder_signs', [-1, 1, 1, -1])
        self.declare_parameter('pwm_signs', [1, -1, 1, -1])
        # ⚠️ 2026-08-17: 로봇의 "앞"을 반대쪽으로 재정의함.
        # 이유: 하판→상판으로 넘어가는 케이블 뭉치가 2개였는데, 앞뒤를 바꾸면
        # 1개로 줄일 수 있다(사용자). 상하판을 넘는 케이블은 라이다 차폐의
        # 주범이라 실익이 크다. 라이다는 이미 새 정면을 보도록 장착돼 있어
        # TF는 손댈 게 없다.
        #
        # 결론(2026-08-17): **채택 안 함(False)**. 라이다를 원래 정면으로 되돌려
        # 달기로 해서 앞뒤 재정의가 불필요해졌다. 코드는 남겨둔다 — 나중에
        # 케이블 배치를 이유로 다시 검토할 수 있고, 그때 True로 켜면 된다.
        # 켤 경우 **straight_trim만 재측정**하면 된다(회전 관련은 전부 유효).
        #
        # 이 플래그가 True면 **직진 성분만** 부호를 뒤집는다:
        #  - 새 전진(+v)은 옛 후진이므로 PWM 부호를 반대로
        #  - 엔코더도 반대로 세지므로 부호를 같이 뒤집어 odom이 +로 나오게
        # 회전은 뒤집지 않는다 — z축 180° 회전은 CCW를 보존하므로 각속도
        # 명령과 바퀴 배정이 그대로 유효하다(그래서 min/max_turn_pwm 재측정 불필요).
        self.declare_parameter('invert_drive_direction', False)
        self.declare_parameter('max_pwm', 1200)
        # 드라이버 내부 데드존($deadzone). Yahboom 예제의 520 기본값 1900을 그대로
        # 써왔다(310=1600, TT=1000~1250). 의미는 문서에 없다 — 2026-09-24 바닥에서
        # 명령 0.04~0.15가 전부 약 0.4m/s로 나와(PWM 120 바닥), 드라이버가 이 값을
        # 출력에 깔아주는지 시험하려고 파라미터로 뺐다(docs/todo.md #1).
        self.declare_parameter('driver_deadzone', 1900)
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
        # --- 채널별 PWM 오프셋 (2026-08-20 실측) ---
        # 모터마다 기계적 드래그가 달라서 같은 PWM에도 회전수가 다르다.
        #
        # ⚠️ **곱셈(gain)이 아니라 덧셈(offset)이어야 한다.** 실측이 방식을 정해준다:
        #    도크 위 채널별 개별 구동(PWM 200/300/450/600, tools/motor_channel_test.py)
        #      M3 부족분(절대)  1005 -> 1006 -> 1031 ->  969   **거의 일정**
        #      M3 부족분(비율) 25.6% -> 23.2% -> 20.8% -> 17.5%  감소
        #    절대량이 일정하다는 것은 **일정한 저항 토크(기계적 드래그)**를 뜻한다.
        #    비율로 곱해 보정하면 저속에선 모자라고 고속에선 과해진다.
        #
        # 원인 판별 근거 (세 가설이 서로 다른 흔적을 남긴다):
        #    접촉 저항(I x R)  -> 전류가 클수록 부족분 **증가**  (불일치)
        #    약한 모터(낮은 Kv) -> **비율**이 일정              (불일치)
        #    기계적 드래그      -> **절대량**이 일정            (일치)
        #  사용자 독립 관찰로도 확인: "M3만 관성이 부족한 것처럼 빨리 멈춘다"
        #  — 같은 마찰이 구동 중엔 속도를 깎고 전원을 끊으면 더 빨리 세운다.
        #
        # 환산: 응답 기울기 약 4.2 카운트/PWM. M3 부족분 약 1000 카운트 -> +238 PWM.
        #   검산 PWM 200: 2914 + 984 = 3898 (기준 3919) / PWM 600: 4555 + 984 = 5539 (기준 5524)
        # M1은 부족분이 342 -> 223으로 줄어 순수 드래그는 아니다. 운용 구간
        # (PWM 150~400)에 맞춰 약 75로 둔다.
        #
        # ⚠️ 정지(pwm=0)에는 적용하지 않는다 — 더하면 가만히 있어야 할 때 기어간다.
        # ⚠️ **기체별 값이다.** 2호기는 재측정할 것(docs/fleet_and_lifecycle.md).
        # ⚠️ 배선/바퀴를 건드리면 다시 잴 것 — 2026-08-20에 커넥터 작업 후
        #    M3가 4382 -> 3335로 바뀌었다.
        # ⚠️ 2026-09-24 전부 0으로 리셋 — 뒷바퀴 370->520 교체 + 채널 배선을
        # 좌/우 묶음에서 앞/뒤 묶음으로 다시 함(_apply_pwm 주석 참고). 위 75/240
        # 값은 옛 매핑의 특정 물리 모터(구 M1=왼쪽뒤, 구 M3=오른쪽뒤 370)를 대상으로
        # 잰 것이라 지금은 전혀 다른 모터/위치에 적용되는 셈이라 무효. 앞(M1,M2)은
        # 같은 370이지만 채널이 바뀌었고, 뒤(M3,M4)는 아예 다른 모델(520)이라
        # 마찰 특성 자체가 다시 측정돼야 함 — straight_check.py/motor_channel_test.py로
        # 재캘리브레이션 전까지는 0으로 둔다.
        self.declare_parameter('motor_pwm_offset_m1', 0)    # 왼쪽 앞 (370) — 재측정 필요
        self.declare_parameter('motor_pwm_offset_m2', 0)     # 오른쪽 앞 (370) — 재측정 필요
        self.declare_parameter('motor_pwm_offset_m3', 0)   # 왼쪽 뒤 (520, 신규) — 재측정 필요
        self.declare_parameter('motor_pwm_offset_m4', 0)     # 오른쪽 뒤 (520, 신규) — 재측정 필요

        # 🔄 2026-08-19: 0.6 -> 0.0. 직진 폐루프가 이 역할을 대신한다.
        #    트림은 원리적으로 약하다(위 _on_control_timer 주석 참고) — 데드존
        #    보정에 무력화되고, 남은 좌우 PWM 차로는 스크럽 마찰을 못 이긴다.
        #    폐루프와 같이 두면 서로 싸우므로 0으로 내린다.
        #    (개루프로 되돌릴 일이 생기면 실측 이력은 아래 주석과 git 로그 참고)
        self.declare_parameter('straight_trim', 0.0)
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
        # 🔄 2026-08-19 전면 재캘리브레이션 (축간거리 125->108mm, 재배선 후)
        #    바닥 실측, 배터리 11.78V, 폐루프 OFF, PWM 40~550 전 구간 스윕
        #    (`tools/rotation_calibrate.py`, 데이터 `docs/calibration/rotation_pwm.json`).
        #
        #    선형 적합:  w = 0.00412 * pwm - 0.2032   (잔차 평균 0.025 rad/s)
        #    -> 데드존 PWM **49** (이전 230), 최대 각속도 **2.15 rad/s** (이전 1.0)
        #
        #    ⚠️ 왜 이렇게 크게 바뀌었나: 축간거리 단축으로 스크럽 마찰(M∝L)이 13.6%
        #       줄었는데, 모터가 **토크-속도 곡선의 스톨 근처**에서 돌고 있어서
        #       부하가 조금만 줄어도 동작점이 크게 올라갔다. 같은 PWM 250에서
        #       0.187 -> 0.766 rad/s (+309%).
        #
        #    ⚠️ 고치기 전 모델은 명령 대비 **저속에서 4.2배** 과속했다
        #       (명령 0.3 -> 실제 1.25). 폐루프가 그걸 되돌리느라 계속 싸웠다.
        #
        #    ⚠️ 이 값들은 **바닥 재질·배터리 전압·모터 온도에 의존**한다. 카펫에서는
        #       다시 재야 한다. 유닛마다도 다르므로 2호기부터는 unit calibration
        #       파일로 분리할 것(docs/fleet_and_lifecycle.md).
        #    ⚠️ 데드존을 실제보다 **낮게** 잡는 쪽이 안전하다 — 안 움직이면 폐루프
        #       적분항이 알아서 PWM을 올리지만, 높게 잡으면 오버슛이 난다.
        self.declare_parameter('min_turn_pwm', 49)
        self.declare_parameter('max_turn_pwm', 570)
        self.declare_parameter('max_angular_speed', 2.15)

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
        # 🔄 2026-08-19 재튜닝: 250/400 -> 140/220
        #    재배치로 플랜트가 **1.76배 민감**해졌다(PWM->rad/s 기울기 0.00234 ->
        #    0.00412). 같은 게인이면 루프 이득이 그만큼 커져서 **진동**한다.
        #    실제로 250/400에서 추종 오차 44%, 흔들림(각속도 표준편차) 0.391이었다.
        #    1.76로 나눈 값(142)이 실측 최적점(140)과 거의 일치했다.
        #
        #    실측 비교 (명령 0.3/0.6 rad/s, 바닥, 배터리 11.78V):
        #      kp=250 ki=400 -> 오차 44%, 흔들림 0.391
        #      kp=140 ki=220 -> 오차  2%, 흔들림 0.074   <- 채택
        #      kp= 90 ki=140 -> 오차  4%, 흔들림 0.059
        #
        #    ⚠️ 피드포워드(min_turn_pwm/max_angular_speed)를 바꾸면 **게인도 같이
        #       재튜닝해야 한다.** 플랜트 이득이 변하면 루프 이득이 따라 변한다.
        self.declare_parameter('turn_kp', 140.0)      # PWM per (rad/s) 오차
        self.declare_parameter('turn_ki', 220.0)      # PWM per (rad/s·s)
        self.declare_parameter('turn_i_limit', 250.0) # 적분항이 낼 수 있는 최대 PWM
        # 자이로가 이 시간보다 오래 갱신 안 되면 폐루프를 끄고 피드포워드만 쓴다.
        self.declare_parameter('gyro_stale_sec', 0.3)
        self.declare_parameter('control_rate_hz', 50.0)

        # --- 속도 제어 모드 (2026-09-24 신설, docs/todo.md #1) ---
        # 'legacy': 위 개루프 PWM 모델(드라이버 deadzone 1900 + PWM 0~1200 단위).
        #   바닥 실측: 명령 0.04~0.3 → 실제 0.39~0.49 m/s. 드라이버가 deadzone을
        #   출력 바닥으로 깔아서(출력=deadzone+PWM) 0.4 m/s 밑으로 못 간다.
        # 'velocity': 드라이버 deadzone 0, 바퀴별 목표 속도(m/s) → 축별 피드포워드
        #   + 엔코더 속도 PI. 회전 폐루프도 PWM이 아니라 각속도(rad/s)를 보정한다.
        #   기동 시에만 읽는다(deadzone이 모드와 짝이라 도중 전환 불가).
        # ⚠️ 2026-09-26 기본값 legacy → velocity(사용자 승인). legacy는 명령과 무관하게 약 0.4 m/s,
        #   회전 PI(370 시절 게인)가 발진해 회전 중 역회전(5초에 4번). velocity는 역회전 0, 출력 조절 가능,
        #   출발 틀어짐 약 0.5°, 정사각형 복귀 8~13 cm. 근거 docs/todo.md 1e·1f.
        #   Nav2 속도(max_vel_x 0.04)는 legacy 과속을 전제로 한 값 → 자율주행 전 재조정 필요.
        self.declare_parameter('control_mode', 'velocity')
        self.declare_parameter('velocity_driver_deadzone', 0)
        # 축별 피드포워드: |pwm| = pwm0 + |v|/slope. 도크 무부하, deadzone 0 스윕
        # (docs/calibration/axle_speed_sweep_dz0_20260924.csv) 1800~3000 구간 적합:
        #   앞(370) 0.40/0.795/1.13 m/s → slope 0.000608, pwm0 1142
        #   뒤(520) 0.23/0.443/0.631 m/s → slope 0.000334, pwm0 1111
        # 바닥 부하에선 모자란다 — 나머지는 아래 PI가 메운다.
        # ⚠️ 첫 도크 시험(2026-09-24)에서 축 평균 pwm0로는 안 됐다 — 같은 뒤축이라도
        #   M3(왼쪽뒤) 800→0.008, 1200→0.003 m/s(걸림), M4(오른쪽뒤) 600→0.058,
        #   1200→0.163 m/s로 데드존이 2배 가까이 다르다. 그래서 pwm0은 바퀴별.
        #   실제보다 **낮게** 잡는다 — 모자라면 PI 적분이 올려주지만 넘치면 튄다.
        #   M1 1100(1200→0.122) / M2 1180(1800→0.376 역산) / M3 1350(1800→0.151 역산)
        #   / M4 700(1200→0.163 역산)
        # ⚠️ 2026-09-26 바닥: M4를 1350(M3와 같게)으로 올림. 바퀴 하나씩 잰 도크 값(M4 700)을 쓰니
        #   출발 순간 왼쪽 뒤가 오른쪽 뒤보다 약 2배 세게 밀어서, 전진하면 오른쪽으로·후진하면
        #   왼쪽으로 3~4° 틀어진 뒤 출발했다(사용자 관찰 "M2가 늦게 반응해 방향이 틀어진다").
        #   바닥에선 바퀴 하나의 데드존보다 **좌우 균형**이 방향을 정한다. 0.15 m/s 출발 3회
        #   (tools 밖 start_lag 시험): 0.5초 틀어짐 전진 −3.1~−4.2° → −0.4~−0.6°, 후진 +2.1~+4.0° → +0.2~+0.6°
        #   (아래 kinetic M4 800과 함께). 근거는 docs/todo.md 1f.
        self.declare_parameter('ff_pwm0', [1100.0, 1180.0, 1350.0, 1350.0])
        # 구르기 시작한 뒤의 pwm0(운동마찰). 2026-09-25 바닥: 0.105 m/s로 구르는 중
        # PWM 앞 약 860~1000, M3 약 1030, M4 약 500까지 내려도 속도가 유지됐다 —
        # 위 정지마찰 값(도크 측정)을 계속 주면 출발 뒤 목표의 2배로 튄다. 바퀴 속도가
        # ff_moving_threshold를 넘으면 이 값으로 바꾼다. 낮게 잡아도 PI가 채운다.
        # ⚠️ 2026-09-26: M4 300 → 800(M3와 같게). 300이면 구르는 중에도 오른쪽 뒤가 약해 방향이
        #   계속 오른쪽으로 흘렀다(2.5초 동안 −1.8° → −2.6°). 바퀴 PI(wheel_kp 0.5)가 약해 이
        #   차이를 못 메운다 — 위 ff_pwm0 주석 참고.
        self.declare_parameter('ff_pwm0_kinetic', [750.0, 800.0, 800.0, 800.0])
        self.declare_parameter('ff_moving_threshold', 0.02)   # m/s
        self.declare_parameter('ff_front_slope', 0.000608)
        self.declare_parameter('ff_rear_slope', 0.000334)
        # --- 엔코더 타당성 검사 (2026-09-25, 오도메트리와 velocity 모드 공용) ---
        # 520 교체 후 드라이버 3·4번 채널(뒷바퀴)이 회전 패턴에서 불가능한 값을 읽는다
        # (4번: 초당 약 22,000카운트 = −1.9 m/s, 520 무부하 최고 0.63 m/s). 커넥터
        # 맞바꿈 시험에서 오독은 보드 4번 채널에 남았다. 370 4개 시절 녹화엔 0건.
        # 하드웨어 대책 전까지 소프트웨어로 막는다:
        #   ① 바퀴별 물리 상한을 넘으면 그 바퀴 값을 버린다
        #   ② 같은 측 앞/뒤가 반대 방향이거나 크게 다르면 앞바퀴를 믿는다(앞 채널은
        #      오독 이력 없음, 바닥에선 같은 측 바퀴가 지면으로 묶여 같은 속도여야 함)
        self.declare_parameter('wheel_max_speed_front', 1.3)   # 370 무부하 최고 1.13
        self.declare_parameter('wheel_max_speed_rear', 0.8)    # 520 무부하 최고 0.63
        # 0.15 → 0.05 (2026-09-25 바닥: M3가 0.022로 오독하는데 앞 0.145와 차이 0.12라 놓침)
        self.declare_parameter('side_disagree_abs', 0.05)      # m/s — 이만큼 넘게 다르고
        self.declare_parameter('side_disagree_rel', 0.5)       # 큰 쪽의 50% 넘게 다르면 불일치
        # 바퀴 속도 PI — 게인은 "m/s 오차당 m/s 보정" 무차원이고 축별 slope로
        # 나눠 PWM으로 바꾼다(앞/뒤 플랜트 이득 차이를 자동 반영).
        self.declare_parameter('wheel_kp', 0.5)
        self.declare_parameter('wheel_ki', 3.0)          # 1/s
        self.declare_parameter('wheel_i_limit', 0.5)     # m/s 환산 상한
        self.declare_parameter('velocity_max_pwm', 3500)
        # 뒤 520 무부하 최고 0.63 m/s(스펙 178rpm) — 여유를 두고 자른다
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('encoder_stale_sec', 0.3)
        # 자이로 각속도 PI (rad/s 오차당 rad/s 보정)
        self.declare_parameter('yaw_kp', 0.5)
        self.declare_parameter('yaw_ki', 1.0)
        self.declare_parameter('yaw_i_limit', 1.0)       # rad/s 환산 상한
        # 직진·곡선 주행 중 방향 유지 전용 게인(제자리 회전은 위 yaw_kp/ki 그대로).
        # ⚠️ 2026-09-26 분리: 달리는 중 작은 회전 명령은 실제로 2~5할만 반영된다(스키드 조향 옆 미끄럼,
        #   tools/curve_gain_test.py: w 0.05 → 19~54%, 0.1 → 35~52%, 0.3 → 약 100%). 그래서 제자리 회전과
        #   같은 게인(0.5/1.0)으로는 직진 방향이 2.5초에 1.6°씩 흘렀다. 게인별 2.5초 방향 흐름 평균:
        #   0.5/1.0 → 1.6°, 1.5/3.0 → 1.3°, 3.0/6.0 → 0.6~1.0°(start_lag_test 0.12 m/s 앞뒤 2회씩).
        #   ⚠️ 5.0/10.0은 발산 — 로봇이 좌우로 ±44°까지 흔들림. 그래서 발산 값의 40%인 2.0/4.0
        #   (바닥·배터리가 달라져도 여유). 정사각형 복귀(EKF 폐루프 회전과 함께): 0.5/1.0에서
        #   9.0 cm −6.2° / 15.6 cm −14.6° → 2.0/4.0에서 5.1 cm −2.7° / 4.3 cm −3.5°.
        # 능동 제동(정지 명령 후 바퀴 반대 방향 출력). ⚠️ 2026-09-26: 출력을 0으로 끊기만 하면 바퀴가 굴러
        #   멈춰서 0.15 m/s → 6~8 cm·0.6초, 0.3 m/s → 13~18 cm·0.9초(감속 약 0.16 m/s², Nav2 가정 0.3의 절반,
        #   tools/stop_distance_test.py). brake_kp: 측 속도(m/s) 1당 PWM 환산 비율(0이면 끔).
        #   안전: brake_max_s까지만, |속도| < brake_min_speed면 0, 제동 중 속도가 오히려 커지면 즉시 중단.
        #   바닥 비교(엔코더/EKF 기준 정지 명령 후 이동): 0 → 0.15 m/s 6~8 cm, 0.3 m/s 13~18 cm /
        #   2 → 0.15 m/s 3~5 cm / **4 → 0.15 m/s 1~5 cm, 0.3 m/s 6~9 cm** / 6 → 0.3 m/s 5~10 cm(나아지지 않고
        #   엔코더·EKF 차가 벌어짐 = 바퀴는 멈추고 차체가 미끄러짐). → 4. 회전 정지도 제동돼 폐루프 회전 오차가
        #   평균 −1.2° → −0.8~+0.3°로 모임(정사각형 0.7 m). 위치 추정 오차 6.5 cm(이전 1.7~4.4) — 제동 미끄럼 의심, 관찰 중
        self.declare_parameter('brake_kp', 4.0)
        self.declare_parameter('brake_max_pwm', 1500.0)
        self.declare_parameter('brake_min_speed', 0.02)
        self.declare_parameter('brake_max_s', 0.6)
        self.declare_parameter('yaw_hold_kp', 2.0)
        self.declare_parameter('yaw_hold_ki', 4.0)
        # ⚠️ 2026-09-27 실물 Nav2 첫 주행 사고: 곡선 명령(v 0.107, w 0.39)에서 이 방향 유지 루프가 발진 —
        #   실제 회전 1.85 rad/s(명령의 4배) → w 0으로 바뀌자 반대로 넘쳐 **±4.4 rad/s(자이로 측정 한계)로 약 31초
        #   좌우 발진**(녹화 bags/20260927_130109_real_goal1). 2.0/4.0은 직진(w=0)에서만 시험했었다. 달리는 중 작은
        #   회전은 옆 미끄럼에 막히다가(곡선 이득 2~5할) 한번 미끄러지면 확 돌아서, 보정이 커지면 폭주한다.
        #   → 보정 상한: 명령에 더하는 회전을 ±yaw_hold_max_corr로 제한(직진 흐름 보정엔 0.1 이하면 충분),
        #     상한에 걸린 동안 적분 정지(anti-windup).
        self.declare_parameter('yaw_hold_max_corr', 0.3)       # rad/s
        # 회전 폭주 감지: 명령 |w| < yaw_runaway_cmd_max인데 실제 |자이로| > yaw_runaway_rate가
        #   yaw_runaway_sec 넘게 이어지면 정지·잠금(재시작 전까지 구동 거부, 선속도 폭주 감지와 같은 방식)
        self.declare_parameter('yaw_runaway_rate', 2.5)        # rad/s
        self.declare_parameter('yaw_runaway_cmd_max', 1.0)     # rad/s
        self.declare_parameter('yaw_runaway_sec', 0.3)
        self.declare_parameter('velocity_debug', False)
        # --- velocity 모드의 제자리 회전 (2026-09-25 바닥 실측) ---
        # 바퀴별 속도 PI로 제자리 회전을 하면 바퀴 목표가 0.05~0.1 m/s로 너무 낮아
        # 스크럽 정지마찰에 걸려 한쪽씩 붙었다 튀었다(흔들림 ±0.2~0.5 rad/s, 제자리인데
        # 0.3 m 밀림). 반면 네 바퀴에 같은 출력을 고정하면 회전이 매끄러웠다(±0.03~0.05):
        #   좌회전  1620→0.07 1680→0.16 1740→0.28 1800→0.41 1900→0.60 2000→0.89 rad/s
        #   우회전  1620→0.05 1680→0.14 1740→0.21 1800→0.28 1900→0.38 2000→0.45 rad/s
        # 직선 적합: 좌 w=0.00224(pwm−1615), 우 w=0.00098(pwm−1525) — **좌우 비대칭이 크다**
        # (M2가 가장 약함, 사용자 관찰과 일치). 그래서 |v|가 작으면 이 방향별 모델 +
        # 자이로 PI(yaw_kp/ki, rad/s 단위를 방향별 기울기로 PWM 환산)로 돌린다.
        self.declare_parameter('turn_in_place_max_v', 0.02)          # m/s — 이하이면 제자리 회전 모드
        # 제자리 회전 명령 중에는 오도메트리 전진량을 0으로 둔다(legacy/velocity 공통).
        # 2026-09-25 바닥: 회전 중 바퀴가 좌우 비대칭으로 미끄러져(M2 약함, M4 오독 배제로
        # 오른쪽은 M2 하나) 오도메트리가 여러 바퀴에 0.5 m 전진했다고 기록 — 라이다 ICP
        # 기준 실제 밀림은 한 바퀴당 1~3 cm. 0이 수십 cm의 가짜 전진보다 사실에 가깝다.
        self.declare_parameter('odom_zero_translation_in_place', True)
        # --- velocity 모드 전환 정지 간격 + 각가속도 제한 (2026-09-25, 사용자 제안) ---
        # 직진 중 급히 회전하거나 회전 중 바로 출발하면 관성·미끄러짐이 겹쳐 궤적이 튀고
        # 오도메트리가 오염된다. 동작 종류(직진↔제자리 회전)가 바뀌면 바퀴·자이로가
        # 멈춘 것을 확인하고(최소 dwell_min, 최대 dwell_max) 다음 동작을 시작한다.
        # 회전 명령은 가속 방향으로만 각가속도 제한 — 바닥 실측에서 출발 1초 과회전
        # 30~50%(정지마찰이 아니라 출발 순간 최대 오차에 비례 보정이 크게 걸린 것:
        # 회전 시작·정지 PWM이 둘 다 약 1600으로 히스테리시스 없음). 감속·정지는 즉시.
        # 1.5 → 0.75 (2026-09-25 회전 출발 매트릭스 90°×48회: 출발 과회전 평균 계단 1.95 /
        # 0.75 → 1.54 / 1.5 → 1.80 / 3.0 → 1.95배. 사용자가 우려한 역효과(바퀴별 문턱 차로
        # 로봇이 틀어짐)는 위치 밀림이 램프와 무관해 나타나지 않음 — turn_start_matrix_20260925_2007.json)
        self.declare_parameter('angular_accel_limit', 0.75)     # rad/s^2, 0이면 끔
        self.declare_parameter('motion_dwell_min', 0.15)        # s
        self.declare_parameter('motion_dwell_max', 0.6)         # s
        self.declare_parameter('dwell_settle_gyro', 0.05)       # rad/s
        self.declare_parameter('dwell_settle_speed', 0.03)      # m/s
        # 직선 적합은 우회전 고속에서 틀렸다(2000→0.45, 2200→0.97로 급변) → 측정표 보간.
        # 1차 재시험(직선 적합): 정속 정확도 96~109%, 흔들림 좌 0.04 / 우 고속 0.3~0.68.
        # 보정 이득도 그 지점의 실제 기울기로 PWM 환산한다(기울기 하한 turn_min_slope).
        # ⚠️ 2026-09-27 재측정(tools/turn_friction_calibrate.py, 바닥, 11.5 V, 돌고 있는 상태에서 출력을 50씩 내리며
        #   각 1초 평균, 방향별 2회 평균 — docs/calibration/turn_friction_20260927_1359.json). 예전 표(09-25)는 우회전을
        #   과소평가(2000에서 0.45 vs 실측 0.86)해서 우회전 0.5 명령에 약 25% 과속했다(turn_rate_test 기준 측정).
        #   돌기 시작 출력(1560)과 멈추는 출력(1550)은 거의 같다 — 출발 튐은 정지마찰이 아니라 가속 지연 동안
        #   보정이 부푸는 탓(아래 turn_engage_ratio).
        self.declare_parameter('turn_table_pwm', [1550.0, 1600.0, 1650.0, 1700.0, 1750.0, 1800.0, 1850.0, 1900.0, 1950.0, 2000.0, 2050.0, 2100.0])
        self.declare_parameter('turn_table_w_ccw', [0.0, 0.08, 0.15, 0.23, 0.31, 0.40, 0.53, 0.58, 0.86, 0.92, 1.09, 1.35])
        self.declare_parameter('turn_table_w_cw', [0.0, 0.13, 0.16, 0.21, 0.29, 0.34, 0.40, 0.44, 0.71, 0.86, 0.90, 0.96])
        # 출발 구간: 실제 회전이 목표의 이 비율(최소 0.1 rad/s)에 닿기 전까지는 자이로 보정(비례·적분)을 쓰지 않고
        #   회전표 피드포워드만 준다. ⚠️ 2026-09-27: 모터 가속 지연(약 0.3초) 동안 보정이 +106~+135 PWM 쌓였다가
        #   따라잡는 순간 출발 튐 1.6~2.7배(turn_rate_test 기준 측정 0/6 합격).
        self.declare_parameter('turn_engage_ratio', 0.5)
        self.declare_parameter('turn_min_slope', 0.0015)             # rad/s per PWM
        # 제자리 회전 좌우 출력 균형 [좌회전, 우회전]: 오른쪽 ×√b, 왼쪽 ÷√b(총 출력 유지).
        # 2026-09-25 매트릭스: 우회전만 90°마다 10~14 cm 밀림(회전 중심 약 8 cm 치우침,
        # 우회전 때 역방향인 오른쪽 M2가 가장 약함) — tools/turn_balance_test.py로 측정:
        #   우회전 b 1.0 → 11.6 cm / **1.1 → 2.4 cm** / 1.2 → 11.3 cm(반대 방향) / 1.4 → 15.7 cm.
        #   밀림 벡터로 역산한 회전 중심: b 1.0은 오른쪽 바퀴 줄(오른쪽 8.2 cm), b 1.2는 왼쪽
        #   바퀴 줄 — 한쪽이 바닥에 붙고 다른 쪽이 끌고 도는 두 상태 사이의 경계가 1.1.
        #   0.1 차이로 약 10 cm가 달라지므로 고정값은 출발점일 뿐, 회전 중 추적 보정이 필요하다
        #   (turn_balance_20260925_2021.json)
        self.declare_parameter('turn_balance', [1.0, 1.1])
        # 좌우 대칭 루프(2026-09-25): 제자리 회전 중 좌우 측 속도 크기가 같아지도록 균형을
        # 실시간 조정한다. 대칭 오차 e = (|왼쪽| − |오른쪽|) / (|왼쪽| + |오른쪽|) — 오른쪽이
        # 붙어 느리면 e > 0 → 오른쪽 출력을 올린다. 균형 = turn_balance × exp(kp·e + ki·∫e),
        # ∫e는 방향별로 기억(다음 회전의 출발점, 노드 재시작 시 초기화), |log 균형 보정| ≤ limit.
        self.declare_parameter('turn_symmetry_enable', True)
        # 관성 모델 시뮬: kp 0.1/0.0/0.03 모두 3회전 안에 균형 안정, 학습 균형 1.07~1.11(실측 최적 1.1).
        # 관성 없는 모델에선 kp가 두 상태를 매 주기 넘나들게 해 보수적으로 작게 둔다
        self.declare_parameter('turn_sym_kp', 0.03)
        self.declare_parameter('turn_sym_ki', 0.15)         # 1/s
        self.declare_parameter('turn_sym_limit', 0.4)       # log 단위(약 0.67~1.5배)
        self.declare_parameter('velocity_pi_mode', 'wheel')   # 'wheel' | 'side' — 아래 _velocity_control 참고  # 목표/측정/PWM 로그(0.25초 간격)

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
        # 바이어스 측정 창을 "정지 상태"로 인정하는 조건. 2026-09-24 사고: USB-C를
        # 다시 꽂는 순간 노드가 떴고 사람이 로봇을 만지던 중이라 바이어스가
        # -0.438 rad/s(정상 -0.005)로 잡혔다. 폐루프가 그걸 "회전 중"으로 믿고
        # 보정해서 도크에선 오른쪽 바퀴가 1.2초 만에 역전, 바닥 직진은 -51° 이탈.
        # (당시엔 /cmd_vel 발행자 충돌로 오진했다.) 정지 실측: 평균 -0.005,
        # 표준편차 0.03 rad/s(100샘플 창). 08-21~09-24 기동 60여 회의 정상 범위는
        # -0.011~+0.013이고, 비정상은 -0.438(09-24), +0.672(08-22 23:15),
        # +0.0486(08-29 17:03) — 마지막 것을 잡으려고 0.03으로 둔다(정상 최대의
        # 2.4배). 조건을 못 넘으면 창을 버리고 다시 잰다
        # — 그동안 _gyro_ts가 None이라 회전 폐루프는 꺼진 채(개루프)로 동작한다.
        self.declare_parameter('gyro_bias_max_abs', 0.03)
        self.declare_parameter('gyro_bias_max_std', 0.08)

        # 2026-08-21 파워 스윕 실측: 출발 즉시 목표 PWM으로 점프하면 바퀴 4개의
        # 접지력 차이(특히 후륜/전륜 비대칭, 파워가 클수록 심해짐)가 슬립으로
        # 드러나서 요 이탈이 생긴다(고파워 구간에서 최대 +1.63° 관측,
        # docs/calibration/sweep_loaded_20260821_2059.json). 목표 속도까지
        # 선형 가속도 제한을 걸어 순간 토크 스텝을 줄이면 정지마찰 한계를 넘길
        # 확률이 줄어든다.
        # ⚠️ **가속(출발)에만 적용, 감속(정지)은 램프 없이 즉시** — 정지 버튼은
        # "지연 없이 즉시 반응"이 이미 문서화된 안전 요구사항이다(motor_dashboard.py
        # hardStop 주석 참고). 슬립 최적화보다 안전이 우선이라 감속 경로는 안 건드림.
        self.declare_parameter('linear_accel_limit', 3.0)   # m/s^2, 가속 방향에만 적용

        p = self.get_parameter
        self._wheel_radius = p('wheel_radius').value
        self._wheel_separation = p('wheel_separation').value
        self._counts_per_rev = p('counts_per_rev').value            # 앞바퀴(370) 전용
        self._counts_per_rev_rear = p('counts_per_rev_rear').value  # 뒷바퀴(520) 전용
        self._rear_pwm_affine_slope = p('rear_pwm_affine_slope').value
        self._rear_pwm_affine_offset = p('rear_pwm_affine_offset').value
        self._invert_drive = p('invert_drive_direction').value
        # ⚠️ 2026-09-24 재배선 후 실측(motor_channel_test.py + 육안 전후진):
        #   M1(왼쪽앞) 전진시 엔코더 음수 / M2(오른쪽앞) 양수
        #   M3(왼쪽뒤,520) 양수 / M4(오른쪽뒤,520) 음수
        #   ✅ 4채널 전부 실측 확정(바닥 직진 cmd_vel 0.2 전진 중 원시 델타
        #   M1<0, M2>0, M3>0, M4<0 — 발행자 충돌 제거한 깨끗한 측정).
        # 패턴이 단순 좌/우나 앞/뒤가 아니라 채널마다 달라 배열 파라미터로 둔다.
        self._encoder_sign = [int(s) for s in p('encoder_signs').value]
        if self._invert_drive:
            self._encoder_sign = [-s for s in self._encoder_sign]
        self._pwm_sign = [int(s) for s in p('pwm_signs').value]
        self._max_pwm = p('max_pwm').value
        self._max_wheel_speed = p('max_wheel_speed').value
        self._min_pwm = p('min_pwm').value
        self._cmd_vel_timeout = p('cmd_vel_timeout').value
        self._straight_trim = p('straight_trim').value
        self._linear_accel_limit = p('linear_accel_limit').value
        self._motor_offset = [p(f'motor_pwm_offset_m{i}').value for i in (1, 2, 3, 4)]
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

        self._control_mode = p('control_mode').value
        if self._control_mode == 'velocity' and self._invert_drive:
            # invert_drive는 엔코더 부호를 뒤집어 odom만 맞추는 방식이라 바퀴 속도
            # 폐루프에 그대로 쓰면 양의 되먹임(폭주)이 된다.
            self.get_logger().error('invert_drive_direction과 velocity 모드는 같이 못 쓴다 — legacy로 동작')
            self._control_mode = 'legacy'
        if self._control_mode not in ('legacy', 'velocity'):
            self.get_logger().error(f'알 수 없는 control_mode={self._control_mode} — legacy로 동작')
            self._control_mode = 'legacy'
        self._ff_pwm0 = [float(x) for x in p('ff_pwm0').value]
        self._ff_pwm0_kinetic = [float(x) for x in p('ff_pwm0_kinetic').value]
        self._ff_moving_threshold = p('ff_moving_threshold').value
        self._ff_slope = {'front': p('ff_front_slope').value, 'rear': p('ff_rear_slope').value}
        self._wheel_max = [p('wheel_max_speed_front').value] * 2 + [p('wheel_max_speed_rear').value] * 2
        self._side_disagree_abs = p('side_disagree_abs').value
        self._side_disagree_rel = p('side_disagree_rel').value
        self._enc_rejects = [0] * 4        # 바퀴별로 버린 횟수(진단용)
        self._side_speed = {'L': 0.0, 'R': 0.0}
        self._side_reverse_since = {'L': None, 'R': None}
        self._wheel_kp = p('wheel_kp').value
        self._wheel_ki = p('wheel_ki').value
        self._wheel_i_limit = p('wheel_i_limit').value
        self._velocity_max_pwm = p('velocity_max_pwm').value
        self._max_linear_speed = p('max_linear_speed').value
        self._encoder_stale_sec = p('encoder_stale_sec').value
        self._yaw_kp = p('yaw_kp').value
        self._yaw_ki = p('yaw_ki').value
        self._yaw_i_limit = p('yaw_i_limit').value
        self._yaw_hold_kp = p('yaw_hold_kp').value
        self._brake_kp = float(p('brake_kp').value)
        self._brake_max_pwm = float(p('brake_max_pwm').value)
        self._brake_min_speed = float(p('brake_min_speed').value)
        self._brake_max_s = float(p('brake_max_s').value)
        self._brake_start = None
        self._brake_v0 = {'L': 0.0, 'R': 0.0}
        self._brake_out = [0.0] * 4
        self._yaw_hold_ki = p('yaw_hold_ki').value
        self._yaw_hold_max_corr = float(p('yaw_hold_max_corr').value)
        self._yaw_runaway_rate = float(p('yaw_runaway_rate').value)
        self._yaw_runaway_cmd_max = float(p('yaw_runaway_cmd_max').value)
        self._yaw_runaway_sec = float(p('yaw_runaway_sec').value)
        self._yaw_runaway_since = None
        self._velocity_debug = p('velocity_debug').value
        self._velocity_pi_mode = p('velocity_pi_mode').value
        self._turn_in_place_max_v = p('turn_in_place_max_v').value
        self._odom_zero_in_place = p('odom_zero_translation_in_place').value
        self._angular_accel_limit = p('angular_accel_limit').value
        self._dwell_min = p('motion_dwell_min').value
        self._dwell_max = p('motion_dwell_max').value
        self._dwell_settle_gyro = p('dwell_settle_gyro').value
        self._dwell_settle_speed = p('dwell_settle_speed').value
        self._ramped_w = 0.0
        self._last_active_mode = None     # 'rotate' | 'translate'
        self._last_active_t = 0.0
        self._dwell_start = None
        self._turn_table_pwm = [float(x) for x in p('turn_table_pwm').value]
        self._turn_engage_ratio = float(p('turn_engage_ratio').value)
        self._turn_engaged = False
        self._turn_engaged_dir = -1
        self._turn_table_w = [[float(x) for x in p('turn_table_w_ccw').value],
                              [float(x) for x in p('turn_table_w_cw').value]]
        self._turn_min_slope = p('turn_min_slope').value
        self._turn_balance = [float(x) for x in p('turn_balance').value]
        self._turn_sym_enable = p('turn_symmetry_enable').value
        self._turn_sym_kp = p('turn_sym_kp').value
        self._turn_sym_ki = p('turn_sym_ki').value
        self._turn_sym_limit = p('turn_sym_limit').value
        self._turn_sym_i = [0.0, 0.0]      # 방향별 대칭 적분(log 균형)
        self._wheel_i = [0.0] * 4
        self._wheel_speed_raw = [0.0] * 4
        self._wheel_speed = [0.0] * 4      # 엔코더로 잰 바퀴별 속도(m/s, 전진 +)
        self._wheel_target = [0.0] * 4
        self._vel_prev = None              # (counts, ts) — 속도 계산용 직전 샘플
        self._fault = None                 # 폭주 감지 시 잠금(재시작 전까지 구동 거부)

        # 런타임 파라미터 변경 반영 (2026-08-17 추가).
        # 이게 없으면 `ros2 param set`이 파라미터 서버 값만 바꾸고 노드 내부
        # 변수는 그대로라, 실측 캘리브레이션을 하면서 값을 바꿔도 **아무 효과가
        # 없는데 성공했다고 착각**하게 된다(실제로 회전 PWM 스윕 전체를 헛돌렸음).
        self.add_on_set_parameters_callback(self._on_params)


        # 카운트 → 미터. ⚠️ 2026-09-24: 뒷바퀴 520 교체로 감속비가 앞바퀴(370)와
        # 달라져 채널별 배열로 변경(바퀴 지름은 앞/뒤 동일, 0.0335m 그대로 —
        # 사용자 확인). M1,M2=앞(370) / M3,M4=뒤(520).
        _circumference = 2.0 * math.pi * self._wheel_radius
        _mpc_front = _circumference / self._counts_per_rev
        _mpc_rear = _circumference / self._counts_per_rev_rear
        self._meters_per_count = [_mpc_front, _mpc_front, _mpc_rear, _mpc_rear]

        self._serial_lock = threading.Lock()
        # exclusive=True(TIOCEXCL): 다른 프로세스가 이미 포트를 쥐고 있으면 열기에서 실패한다.
        # 2026-09-26 사고: EKF 재시작이 Wants=로 구동 서비스를 끌어올려, 수동으로 띄운 구동
        # 노드와 서비스 노드가 같은 포트에 명령을 섞었다 → 정사각형 시험 첫 변 0.8 m 목표에 2.3 m 주행
        self._ser = serial.Serial(p('port').value, p('baudrate').value, timeout=0.2, exclusive=True)
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

        # ⚠️ 2026-09-27: 이 노드의 경과 시간 계산은 전부 time.monotonic(). 예전엔 time.time()(벽시계)이라
        #   부팅 후 NTP가 시계를 11.7시간 앞당긴 순간 "배터리 신호·엔코더 42186초 미수신" 경고가 떴다.
        #   시계가 거꾸로 가면 반대로 cmd_vel 시간 초과 정지가 늦어질 수 있는 구조였다(파이 5는 RTC 배터리 없음).
        self._last_cmd_time = 0.0
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._ramped_v = 0.0          # 가속 방향 램프의 현재값(감속은 즉시 추종)
        self._turn_i = 0.0            # 회전 폐루프 적분항
        self._prev_control_time = None
        self._gyro_ts = None          # 자이로 최신 수신 시각(신선도 판단용)
        self._target_left_pwm = 0
        self._target_right_pwm = 0

        # 배터리 critical 게이트 (2026-08-29 사고 재현 → 2026-09-04 수정):
        # motor_dashboard.py의 INA219 감시가 critical(10.6V)을 감지하면 그동안
        # Nav2/explore 프로세스만 죽였는데, 그날 오후는 캘리브레이션 스크립트
        # (precise_move.py 등)가 cmd_vel을 직접 쏘고 있어 죽일 대상이 없었고
        # 자동정지 로그(`killed=[]`)만 남은 채 실제로는 안 멈췄다. "누가 cmd_vel을
        # 보내는지"를 추적하는 대신, 실제 PWM이 나가는 이 노드 자체가 tier를 알고
        # critical이면 어떤 명령이 와도 거부하도록 함 — 발행자를 안 가린다.
        # 기본값 'ok': motor_dashboard가 아예 안 떠 있어도(현재도 흔한 상황) 기존
        # 동작을 막지 않는다 — 이 게이트는 critical을 "적극적으로 알게 됐을 때"만
        # 개입하고, 모르는 상태를 critical로 취급하지 않는다(기존 동작 보존).
        self._battery_tier = 'ok'
        self._battery_tier_ts = None  # 2026-09-04: staleness 가시성 (Resie 리뷰, task 13)
        self.create_subscription(String, 'battery_tier', self._on_battery_tier, 1)

        # IMU 자이로 상태 (use_imu_for_yaw일 때만 사용)
        self._use_imu = p('use_imu_for_yaw').value
        self._gyro_bias_samples_target = p('gyro_bias_samples').value
        self._gyro_bias = 0.0
        self._gyro_bias_max_abs = p('gyro_bias_max_abs').value
        self._gyro_bias_max_std = p('gyro_bias_max_std').value
        self._gyro_bias_window = []
        self._gyro_bias_count = 0   # 채택된 창의 샘플 수(0이면 아직 미보정)
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
        if self._control_mode == 'velocity':
            dz = int(self.get_parameter('velocity_driver_deadzone').value)
        else:
            dz = int(self.get_parameter('driver_deadzone').value)
        self.get_logger().info(f'제어 모드 {self._control_mode}, 드라이버 deadzone {dz}')
        for cmd in (b'$mtype:1#', f'$deadzone:{dz}#'.encode(), b'$wdiameter:67.00#'):
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
                return [int(g) for g in m.groups()], raw
        return None, raw

    def _encoder_poll_loop(self):
        """백그라운드 스레드: 시리얼 왕복(~60ms)이 걸리는 엔코더 폴링을 ROS
        실행기 스레드 밖에서 계속 반복한다. cmd_vel 콜백이 이 폴링에 막히지
        않게 하는 게 목적 — 클래스 docstring 및 __init__ 주석 참고.

        2026-08-24 진단용 계측 추가: 실물 매핑 시험 중 원인불명으로 엔코더가
        수십 초씩 조용히 멈추는 현상을 실측(motor_dashboard 무관, CPU 부하도
        무관 — power_trace.csv로 반증됨). 실패 구간의 시작/끝/원시 바이트를
        남겨서 다음 재현 시 원인(시리얼 프로토콜 상태 꼬임 vs 실제 물리적
        정지 등)을 가릴 수 있게 한다."""
        fail_streak = 0
        fail_streak_start = None
        last_heartbeat = 0.0
        while not self._encoder_thread_stop.is_set():
            counts, raw = self._read_encoder()
            if counts is not None:
                if fail_streak > 0:
                    dur = time.monotonic() - fail_streak_start
                    self.get_logger().warn(
                        f'엔코더 응답 복구됨 — {fail_streak}회 연속 실패, '
                        f'{dur:.1f}초 동안 정지해있었음')
                fail_streak = 0
                now = time.monotonic()
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
                now = time.monotonic()
                if fail_streak == 0:
                    fail_streak_start = now
                    self.get_logger().warn(
                        f'엔코더 응답 파싱 실패 시작 — 원시 바이트({len(raw)}): {raw!r}')
                fail_streak += 1
                if now - last_heartbeat > 2.0:
                    self.get_logger().warn(
                        f'엔코더 응답 계속 실패 중 — {fail_streak}회, '
                        f'{now - fail_streak_start:.1f}초째. 최근 원시 바이트'
                        f'({len(raw)}): {raw!r}')
                    last_heartbeat = now
                time.sleep(0.02)  # 응답 없을 때 바쁜 대기 방지

    def _apply_pwm(self, left_pwm, right_pwm, apply_offset=True):
        # ⚠️ 2026-09-24 매핑 전면 변경 — 뒷바퀴 520 모터 교체 배선 작업.
        # 채널 배선이 "좌/우로 묶임"에서 "앞/뒤로 묶임"으로 바뀌었다:
        #   M1=왼쪽앞, M2=오른쪽앞(둘 다 기존 370) / M3=왼쪽뒤, M4=오른쪽뒤(둘 다 신규 520)
        # 예전 매핑(M1=왼쪽뒤,M2=왼쪽앞,M3=오른쪽뒤,M4=오른쪽앞)은 더 이상 유효하지 않음.
        #
        # ⚠️⚠️ 채널별 방향 실측 완료(motor_channel_test.py + 육안 확인, 5초씩 개별 구동):
        #   M1(왼쪽앞) 양수PWM -> 전진 ✅   M2(오른쪽앞) 양수PWM -> **후진** ❌
        #   M3(왼쪽뒤) 양수PWM -> 전진 ✅   M4(오른쪽뒤) 양수PWM -> **후진** ❌
        # 패턴이 모터 모델이 아니라 **좌/우로 갈림** — 오른쪽 두 채널(M2,M4) 전부
        # 반대. 재배선하면서 좌우 미러링(원래는 왼쪽/오른쪽이 물리적으로 거울
        # 대칭 장착이라 배선 극성도 반대로 맞춰야 "양수=전진"이 양쪽 다 성립하는
        # 구조였음) 없이 일관된 극성으로 연결한 것으로 보임. 오른쪽 채널만
        # 부호를 뒤집어서 보정 — 물리적으로 재배선하는 대신 소프트웨어에서 처리
        # (pwm_signs 파라미터).
        vals = [0, 0, 0, 0]
        for i in self.LEFT:
            vals[i] = left_pwm * self._pwm_sign[i]
        for i in self.RIGHT:
            vals[i] = right_pwm * self._pwm_sign[i]
        # ⚠️ 2026-09-24: 뒷바퀴(M3,M4,520)는 앞바퀴(370)와 감속비가 달라 같은
        # PWM에서 실제 속도가 다르다 — 도크 위 무부하 스윕 실측(tools/
        # axle_speed_sweep.py, PWM 200~800 5개 구간)으로 각 축의 PWM->속도
        # 선형회귀를 각각 구하고(front: speed=0.000689*pwm+0.483, rear:
        # speed=0.000389*pwm+0.303), 두 식을 연립해 "앞 PWM이 내는 실제
        # 속도를 뒤 PWM도 내게" 하는 관계를 유도했다:
        #   rear_pwm = 1.7692*front_pwm + 461.8  (rear_pwm_affine_* 파라미터)
        # ⚠️ 처음에 단순 배율(1/0.6037≈1.656)만 곱했더니 실측이 어긋났다 —
        # 두 곡선 다 원점을 안 지나서(데드존/절편) 배율만으론 안 맞고
        # 기울기+절편을 같이 맞춰야 함. 검증: front_pwm=400 실측 결과 앞
        # 0.75m/s대 vs 뒤(배율만 적용시) 0.55m/s대로 어긋났었고, affine
        # 적용 후 재계산상 일치 확인.
        # ⚠️⚠️ 이 절편(461.8)이 커서 **0 근처까지 그대로 적용하면 위험** —
        # "정지" 명령인데도 뒤 채널에 462 언저리 PWM을 줘버린다. 그래서
        # front_pwm==0일 때만 명시적으로 0 처리하고, 그 외에는 부호를 보존한
        # 채 affine 적용 + max_pwm 클램프. ⚠️ 데드존 부근(PWM 120~200 사이,
        # 실측 범위 밖)은 여전히 외삽 — 저속 정밀 검증 필요(다음 세션).
        for i in (2, 3):
            if vals[i] == 0:
                continue
            scaled = math.copysign(
                self._rear_pwm_affine_slope * abs(vals[i]) + self._rear_pwm_affine_offset,
                vals[i])
            vals[i] = max(-self._max_pwm, min(self._max_pwm, scaled))
        out = []
        for i, pv in enumerate(vals):
            if pv != 0 and apply_offset:
                # 부호를 따라 더한다(후진 시에도 같은 방향으로 보강).
                # 정지(0)에는 적용하지 않는다 — 더하면 가만히 있어야 할 때 기어간다.
                # 순수 회전(apply_offset=False, 호출부 주석 참고)에도 적용 안 함 —
                # 직진 드래그 보정용 상수라 회전 폐루프의 좌우 대칭 가정을 깬다.
                pv = pv + math.copysign(self._motor_offset[i], pv)
                pv = max(-self._max_pwm, min(self._max_pwm, pv))
            out.append(int(pv))
        self._send(f'$pwm:{out[0]},{out[1]},{out[2]},{out[3]}#'.encode())

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
            'rear_pwm_affine_slope': '_rear_pwm_affine_slope',
            'rear_pwm_affine_offset': '_rear_pwm_affine_offset',
            'wheel_kp': '_wheel_kp',
            'wheel_ki': '_wheel_ki',
            'wheel_i_limit': '_wheel_i_limit',
            'velocity_max_pwm': '_velocity_max_pwm',
            'max_linear_speed': '_max_linear_speed',
            'yaw_kp': '_yaw_kp',
            'yaw_ki': '_yaw_ki',
            'yaw_i_limit': '_yaw_i_limit',
            'yaw_hold_kp': '_yaw_hold_kp',
            'brake_kp': '_brake_kp',
            'brake_max_pwm': '_brake_max_pwm',
            'brake_min_speed': '_brake_min_speed',
            'brake_max_s': '_brake_max_s',
            'yaw_hold_ki': '_yaw_hold_ki',
            'yaw_hold_max_corr': '_yaw_hold_max_corr',
            'yaw_runaway_rate': '_yaw_runaway_rate',
            'velocity_debug': '_velocity_debug',
            'velocity_pi_mode': '_velocity_pi_mode',
            'turn_symmetry_enable': '_turn_sym_enable',
            'turn_sym_kp': '_turn_sym_kp',
            'turn_sym_ki': '_turn_sym_ki',
            'turn_sym_limit': '_turn_sym_limit',
            'angular_accel_limit': '_angular_accel_limit',
            'motion_dwell_min': '_dwell_min',
            'motion_dwell_max': '_dwell_max',
            'dwell_settle_gyro': '_dwell_settle_gyro',
            'dwell_settle_speed': '_dwell_settle_speed',
        }
        for prm in params:
            attr = live.get(prm.name)
            if attr:
                setattr(self, attr, prm.value)
            elif prm.name == 'ff_pwm0':
                self._ff_pwm0 = [float(x) for x in prm.value]
            elif prm.name == 'ff_pwm0_kinetic':
                self._ff_pwm0_kinetic = [float(x) for x in prm.value]
            elif prm.name == 'ff_moving_threshold':
                self._ff_moving_threshold = prm.value
            elif prm.name == 'turn_table_w_ccw':
                self._turn_table_w[0] = [float(x) for x in prm.value]
            elif prm.name == 'turn_table_w_cw':
                self._turn_table_w[1] = [float(x) for x in prm.value]
            elif prm.name == 'turn_min_slope':
                self._turn_min_slope = prm.value
            elif prm.name == 'turn_balance':
                self._turn_balance = [float(x) for x in prm.value]
            elif prm.name == 'turn_in_place_max_v':
                self._turn_in_place_max_v = prm.value
            elif prm.name == 'odom_zero_translation_in_place':
                self._odom_zero_in_place = prm.value
            elif prm.name == 'ff_front_slope':
                self._ff_slope['front'] = prm.value
            elif prm.name == 'ff_rear_slope':
                self._ff_slope['rear'] = prm.value
        return SetParametersResult(successful=True)

    # --- 속도 제어 모드 (control_mode='velocity') ---

    def _send_pwm4(self, vals):
        """바퀴별 PWM(전진 +)을 채널 극성만 반영해 그대로 보낸다 — legacy의
        뒷바퀴 affine/모터 오프셋을 거치지 않는다."""
        lim = self._velocity_max_pwm
        out = [int(max(-lim, min(lim, vals[i] * self._pwm_sign[i]))) for i in range(4)]
        self._send(f'$pwm:{out[0]},{out[1]},{out[2]},{out[3]}#'.encode())

    def _side_values(self, v):
        """바퀴별 값 v[4](거리 또는 속도, 전진 +) → 측별 값 {'L','R'}.

        dt로 나눈 속도 기준으로 판정해야 하므로 호출자는 속도 스케일로 넘긴다
        (오도메트리는 거리/dt를 넘기고 결과에 다시 dt를 곱한다).
        """
        out = {}
        self._wheel_valid = [True] * 4
        for side, (f, r) in (('L', self.LEFT), ('R', self.RIGHT)):
            ok_f = abs(v[f]) <= self._wheel_max[f]
            ok_r = abs(v[r]) <= self._wheel_max[r]
            if ok_f and ok_r:
                big = max(abs(v[f]), abs(v[r]))
                diff = abs(v[f] - v[r])
                if diff > self._side_disagree_abs and diff > self._side_disagree_rel * big:
                    ok_r = False   # 불일치 — 오독 이력 없는 앞바퀴를 믿는다
            for i, ok in ((f, ok_f), (r, ok_r)):
                self._wheel_valid[i] = ok
                if not ok:
                    self._enc_rejects[i] += 1
                    self.get_logger().warn(
                        f'M{i + 1} 엔코더 값 버림 {v[i]:+.2f} m/s (같은 측 '
                        f'M{(r if i == f else f) + 1} {v[r if i == f else f]:+.2f}, '
                        f'누적 {self._enc_rejects})', throttle_duration_sec=2.0)
            if ok_f and ok_r:
                out[side] = (v[f] + v[r]) / 2.0
            elif ok_f:
                out[side] = v[f]
            elif ok_r:
                out[side] = v[r]
            else:
                out[side] = None   # 둘 다 못 믿음 — 호출자가 처리
        return out

    def _turn_table_lookup(self, d, w_abs):
        """측정표에서 |w| → (PWM, 그 구간 기울기 rad/s per PWM). 표 밖은 끝 구간으로 외삽."""
        P, W = self._turn_table_pwm, self._turn_table_w[d]
        for k in range(1, len(P)):
            if w_abs <= W[k] or k == len(P) - 1:
                dw = W[k] - W[k - 1]
                slope = dw / (P[k] - P[k - 1]) if dw > 1e-6 else 1e-6
                return P[k - 1] + (w_abs - W[k - 1]) / slope, slope
        return P[-1], 1e-6

    def _turn_in_place(self, w, dt, gyro_fresh):
        """제자리 회전: 방향별 회전 모델(피드포워드) + 자이로 PI, 네 바퀴 같은 크기.

        바퀴 속도 PI는 쓰지 않는다(저속 스크럽 정지마찰로 붙었다 튐 — 파라미터 주석 참고).
        """
        self._wheel_i = [0.0] * 4
        d = 0 if w > 0 else 1                     # 0=좌회전(CCW), 1=우회전(CW)
        base, slope = self._turn_table_lookup(d, abs(w))
        slope = max(slope, self._turn_min_slope)
        corr = 0.0
        # 출발 구간 판정(turn_engage_ratio 주석) — 방향이 바뀌면 다시 출발 구간
        if self._turn_engaged_dir != d:
            self._turn_engaged = False
            self._turn_engaged_dir = d
        if not self._turn_engaged and gyro_fresh and \
                self._gyro_z * w > 0 and abs(self._gyro_z) >= max(0.1, self._turn_engage_ratio * abs(w)):
            self._turn_engaged = True
        if self._turn_closed_loop and gyro_fresh and not self._turn_engaged:
            self._turn_i = 0.0
        elif self._turn_closed_loop and gyro_fresh:
            err = w - self._gyro_z
            self._turn_i += err * dt
            if self._yaw_ki > 0:
                lim = self._yaw_i_limit / self._yaw_ki
                self._turn_i = max(-lim, min(lim, self._turn_i))
            # ⚠️ 2026-09-27: 출발 과회전 억제 — 정지마찰에 붙어 있는 동안 적분이 쌓였다가 떨어지는 순간 한꺼번에
            #   나가 0.29 rad/s 명령이 0.5~0.72 rad/s까지 치솟았다(curve_pattern_test). 실제 회전이 명령을 넘어선
            #   순간(같은 방향) 쌓인 "더 돌아라" 적분을 비운다.
            if self._gyro_z * w > 0 and abs(self._gyro_z) > abs(w) and self._turn_i * w > 0:
                self._turn_i = 0.0
            # 오차를 회전 방향 기준 크기로 바꿔 PWM에 더한다(더 돌아야 하면 +)
            corr = math.copysign(1.0, w) * (self._yaw_kp * err + self._yaw_ki * self._turn_i) / slope
        else:
            self._turn_i = 0.0
        mag = max(0.0, base + corr)               # 보정이 방향을 뒤집지는 못하게
        sgn = math.copysign(1.0, w)
        enc_fresh = self._update_wheel_speeds()
        log_b = math.log(max(0.1, self._turn_balance[d]))
        sym_e = 0.0
        if self._turn_sym_enable and enc_fresh:
            vl, vr = abs(self._side_speed['L']), abs(self._side_speed['R'])
            if vl + vr > 0.02:                    # 둘 다 서 있으면(출발 전) 판단 보류
                sym_e = (vl - vr) / (vl + vr)
                lim = self._turn_sym_limit
                self._turn_sym_i[d] = max(-lim, min(lim, self._turn_sym_i[d] + self._turn_sym_ki * sym_e * dt))
            log_b += max(-self._turn_sym_limit, min(self._turn_sym_limit,
                                                    self._turn_sym_kp * sym_e + self._turn_sym_i[d]))
        bal = math.sqrt(math.exp(log_b))
        pwm = [0.0] * 4
        for i in self.LEFT:
            pwm[i] = -sgn * mag / bal
        for i in self.RIGHT:
            pwm[i] = sgn * mag * bal
        if self._velocity_debug:
            self.get_logger().warn(
                f'turn_debug w={w:+.2f} gyro={self._gyro_z:+.3f} base={base:.0f} corr={corr:+.0f} '
                f'측 L {self._side_speed["L"]:+.3f} R {self._side_speed["R"]:+.3f} 대칭오차 {sym_e:+.2f} '
                f'균형 {math.exp(log_b):.3f} PWM {[int(x) for x in pwm]}', throttle_duration_sec=0.25)
        self._target_left_pwm = int(pwm[self.LEFT[0]])
        self._target_right_pwm = int(pwm[self.RIGHT[0]])
        self._send_pwm4(pwm)

    def _ramp_w(self, w, dt):
        """각속도 명령에 가속 방향으로만 제한을 건다(감속·정지는 즉시).

        ⚠️ 2026-09-27: 예전엔 방향 역전도 즉시였다 — +0.45 → −0.45가 한 번에 들어가 도는 중인 차체에 곧바로
        반대 출력이 걸렸고, Nav2(DWB)가 부호를 자주 뒤집는 패턴에서 순간 1.7~2.3 rad/s가 나왔다
        (curve_pattern_test). 역전은 먼저 0으로 내린 뒤 새 방향으로 램프를 타고 오른다."""
        cur = self._ramped_w
        if cur != 0.0 and w != 0.0 and (w > 0) != (cur > 0):
            cur = 0.0                                  # 역전: 감속(0까지)은 즉시, 새 방향은 아래 램프로
        accelerating = abs(w) > abs(cur) and (cur == 0.0 or (w > 0) == (cur > 0))
        if accelerating and self._angular_accel_limit > 0:
            step = self._angular_accel_limit * dt
            cur += max(-step, min(step, w - cur))
        else:
            cur = w
        self._ramped_w = cur
        return cur

    def _motion_dwell(self, mode, now, enc_fresh, gyro_fresh):
        """직진↔제자리 회전 전환이면 멈춤을 확인할 때까지 True(=0 출력). 파라미터 주석 참고."""
        if mode == 'stop':
            if now - self._last_active_t > self._dwell_max:
                self._last_active_mode = None      # 충분히 멈춰 있었으면 간격 불필요
            self._dwell_start = None
            return False
        self._last_active_t = now
        if self._last_active_mode is not None and mode != self._last_active_mode:
            if self._dwell_start is None:
                self._dwell_start = now
            elapsed = now - self._dwell_start
            settled = (gyro_fresh and abs(self._gyro_z) < self._dwell_settle_gyro and enc_fresh
                       and max(abs(self._side_speed['L']), abs(self._side_speed['R'])) < self._dwell_settle_speed)
            if elapsed < self._dwell_min or (elapsed < self._dwell_max and not settled):
                return True
            self._dwell_start = None
        self._last_active_mode = mode
        return False

    def _brake_pwm(self, enc_fresh, now):
        """정지 명령 직후 능동 제동 출력(바퀴별). 파라미터 brake_* 주석 참고."""
        if self._brake_kp <= 0:
            return [0.0] * 4
        if self._brake_start is None:
            self._brake_start = now
            self._brake_v0 = dict(self._side_speed)
            self._brake_out = [0.0] * 4
        if now - self._brake_start > self._brake_max_s:
            self._brake_out = [0.0] * 4
            return self._brake_out
        if enc_fresh:
            out = [0.0] * 4
            for i in range(4):
                k = 'L' if i in self.LEFT else 'R'
                sp = self._side_speed[k]
                if abs(sp) > abs(self._brake_v0[k]) + 0.05:     # 제동인데 빨라짐 → 부호 이상, 즉시 끊음
                    self._brake_start = now - self._brake_max_s - 1.0
                    self.get_logger().error(f'능동 제동 중 {k}측 속도 증가({self._brake_v0[k]:+.2f} → {sp:+.2f}) — 제동 중단')
                    self._brake_out = [0.0] * 4
                    return self._brake_out
                if abs(sp) < self._brake_min_speed:
                    continue
                slope = self._ff_slope['front' if i in (0, 1) else 'rear']
                b = -self._brake_kp * sp / slope
                out[i] = max(-self._brake_max_pwm, min(self._brake_max_pwm, b))
            self._brake_out = out
        return self._brake_out

    def _reset_velocity_state(self):
        self._ramped_w = 0.0
        self._wheel_target = [0.0] * 4
        self._wheel_i = [0.0] * 4
        self._side_reverse_since = {'L': None, 'R': None}

    def _update_wheel_speeds(self):
        """엔코더 새 샘플이 오면 바퀴별 속도를 갱신. 반환: 엔코더가 신선한지."""
        with self._counts_lock:
            counts, ts = self._latest_counts, self._latest_counts_ts
        if counts is None:
            return False
        if self._vel_prev is None:
            self._vel_prev = (list(counts), ts)
        else:
            pc, pts = self._vel_prev
            dt = ts - pts
            if dt > 1e-3:
                sp = [(counts[i] - pc[i]) * self._encoder_sign[i] * self._meters_per_count[i] / dt
                      for i in range(4)]
                self._wheel_speed_raw = sp   # 진단용 원시값
                side = self._side_values(sp)
                for k in ('L', 'R'):
                    if side[k] is not None:
                        self._side_speed[k] = 0.5 * self._side_speed[k] + 0.5 * side[k]
                # 바퀴별 측정: 믿을 수 있으면 자기 값, 오독 판정이면 같은 측 값으로 대신
                for i in range(4):
                    k = 'L' if i in self.LEFT else 'R'
                    val = sp[i] if self._wheel_valid[i] else side[k]
                    if val is not None:
                        self._wheel_speed[i] = 0.5 * self._wheel_speed[i] + 0.5 * val
                self._vel_prev = (list(counts), ts)
        return time.monotonic() - ts < self._encoder_stale_sec

    def _velocity_control(self, v, w, dt, now, gyro_fresh):
        if self._fault:
            self._send_pwm4([0, 0, 0, 0])
            return
        enc_fresh = self._update_wheel_speeds()
        v = max(-self._max_linear_speed, min(self._max_linear_speed, v))
        w = self._ramp_w(w, dt)

        if abs(v) > self._turn_in_place_max_v:
            mode = 'translate'
        elif abs(w) > 1e-3:
            mode = 'rotate'
        else:
            mode = 'stop'
        if mode != 'stop':
            self._brake_start = None
        # 회전 폭주 감지(파라미터 yaw_runaway_* 주석, 09-27 사고) — 제어 경로와 무관하게 자이로로 판정
        # 정지 명령 중(mode 'stop')은 제외 — 손으로 들고 돌리는 걸 폭주로 잠그지 않게
        if mode != 'stop' and gyro_fresh and abs(self._gyro_z) > self._yaw_runaway_rate and abs(w) < self._yaw_runaway_cmd_max:
            if self._yaw_runaway_since is None:
                self._yaw_runaway_since = now
            elif now - self._yaw_runaway_since > self._yaw_runaway_sec:
                self._fault = (f'회전 폭주: 명령 {w:+.2f} rad/s인데 실제 {self._gyro_z:+.2f} rad/s가 '
                               f'{self._yaw_runaway_sec:.1f}초 넘게 — 제어 발진 의심, 재시작 전까지 구동 거부')
                self.get_logger().error(self._fault)
                self._send_pwm4([0, 0, 0, 0])
                return
        else:
            self._yaw_runaway_since = None
        if self._motion_dwell(mode, now, enc_fresh, gyro_fresh):
            self._reset_velocity_state()      # 전환 대기 — 출력 0, 적분·램프 초기화
            self._turn_i = 0.0
            self._target_left_pwm = self._target_right_pwm = 0
            self._send_pwm4([0, 0, 0, 0])
            return

        if mode == 'rotate':
            self._turn_in_place(w, dt, gyro_fresh)
            return
        self._turn_engaged = False
        self._turn_engaged_dir = -1

        # 회전: 자이로 PI가 각속도(rad/s)를 보정한다. 직진(w=0) 중에도 켜서 휨을 잡는다.
        moving = abs(v) > 1e-6 or abs(w) > 1e-6
        w_eff = w
        if self._turn_closed_loop and moving and gyro_fresh:
            err = w - self._gyro_z
            self._turn_i += err * dt
            if self._yaw_hold_ki > 0:
                lim = self._yaw_i_limit / self._yaw_hold_ki
                self._turn_i = max(-lim, min(lim, self._turn_i))
            corr = self._yaw_hold_kp * err + self._yaw_hold_ki * self._turn_i
            cl = self._yaw_hold_max_corr
            if abs(corr) > cl:
                self._turn_i -= err * dt          # 상한에 걸린 동안은 적분하지 않음(anti-windup)
                corr = math.copysign(cl, corr)
            w_eff = w + corr
        else:
            self._turn_i = 0.0

        if not moving:
            self._reset_velocity_state()
            self._target_left_pwm = self._target_right_pwm = 0
            self._send_pwm4(self._brake_pwm(enc_fresh, now))
            return

        # 스키드-스티어: 실효 트레드(wheel_separation, 물리 폭의 약 2배)로 바퀴 속도 배분
        half = self._wheel_separation / 2.0
        side = {'L': v - w_eff * half, 'R': v + w_eff * half}
        # 폭주 감지: 측 속도(타당성 검사를 거친 값)가 목표와 반대로 빠르게 계속 → 부호 오류
        for k in ('L', 'R'):
            tgt, meas = side[k], self._side_speed[k]
            if enc_fresh and abs(tgt) > 1e-4 and tgt * meas < 0 and abs(meas) > 0.15:
                if self._side_reverse_since[k] is None:
                    self._side_reverse_since[k] = now
                elif now - self._side_reverse_since[k] > 0.5:
                    self._fault = (f'{k}측 목표 {tgt:+.2f} m/s인데 {meas:+.2f} m/s로 0.5초 넘게 '
                                   '반대로 돎 — 부호/배선 확인 전까지 구동 거부')
                    self.get_logger().error(self._fault)
                    self._send_pwm4([0, 0, 0, 0])
                    return
            else:
                self._side_reverse_since[k] = None
        # PI 단위(velocity_pi_mode):
        #  'wheel' — 바퀴마다 자기 엔코더로 PI(오독 판정 바퀴만 같은 측 값으로 대신).
        #            뒷바퀴도 제 목표로 구동돼 끌려가며 브레이크가 되지 않는다
        #  'side'  — 측 속도 하나로 PI, 같은 측 두 바퀴에 같은 보정. 2026-09-25 바닥
        #            첫 시험에서 앞축 0.106 / 뒤축 0.054 m/s로 뒷바퀴가 끌려갔다
        #            (앞뒤가 다르면 앞만 믿어서 뒤 출력이 피드포워드에만 맡겨짐)
        pwm = [0.0] * 4
        for i in range(4):
            k = 'L' if i in self.LEFT else 'R'
            s = side[k]
            self._wheel_target[i] = s
            if abs(s) < 1e-4 or not enc_fresh:
                self._wheel_i[i] = 0.0
                if abs(s) < 1e-4:
                    continue
            if self._velocity_pi_mode == 'side':
                meas = self._side_speed[k]
            else:
                meas = self._wheel_speed[i]
            corr = 0.0
            if enc_fresh:
                err = s - meas
                self._wheel_i[i] += err * dt
                if self._wheel_ki > 0:
                    lim = self._wheel_i_limit / self._wheel_ki
                    self._wheel_i[i] = max(-lim, min(lim, self._wheel_i[i]))
                corr = self._wheel_kp * err + self._wheel_ki * self._wheel_i[i]
            slope = self._ff_slope['front' if i in (0, 1) else 'rear']
            # 정지마찰 보상: 서 있으면 떼어낼 출력, 목표 방향으로 구르고 있으면 운동마찰 출력
            rolling = s * self._wheel_speed[i] > 0 and abs(self._wheel_speed[i]) > self._ff_moving_threshold
            pwm0 = self._ff_pwm0_kinetic[i] if rolling else self._ff_pwm0[i]
            out = math.copysign(pwm0 + abs(s) / slope, s) + corr / slope
            if out * s < 0:   # 보정이 방향을 뒤집지는 못하게(정지마찰 근처 떨림 방지)
                out = 0.0
            pwm[i] = out
        # 안전 경로(타임아웃/critical)가 "구동 중"인지 판단하는 데 쓴다 — 한쪽 축만
        # 0이어도 놓치지 않게 좌/우 중 큰 값
        if self._velocity_debug:
            self.get_logger().warn(
                f'vel_debug enc_fresh={enc_fresh} w_eff={w_eff:+.2f} '
                f'목표 {[round(x, 3) for x in self._wheel_target]} '
                f'측정 {[round(x, 3) for x in self._wheel_speed]} '
                f'측 {{L: {self._side_speed["L"]:+.3f}, R: {self._side_speed["R"]:+.3f}}} '
                f'PWM {[int(x) for x in pwm]} I {[round(x, 3) for x in self._wheel_i]}',
                throttle_duration_sec=0.25)
        self._target_left_pwm = int(max((pwm[i] for i in self.LEFT), key=abs))
        self._target_right_pwm = int(max((pwm[i] for i in self.RIGHT), key=abs))
        self._send_pwm4(pwm)

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
        self._last_cmd_time = time.monotonic()

    def _on_battery_tier(self, msg: String):
        prev = self._battery_tier
        self._battery_tier = msg.data
        self._battery_tier_ts = time.monotonic()
        if msg.data == 'critical' and prev != 'critical':
            self.get_logger().warn(
                '배터리 critical 수신 — cmd_vel 발행자와 무관하게 구동 거부')

    def _on_control_timer(self):
        """50Hz 제어 루프 — 직진은 개루프, 회전은 자이로 피드백 폐루프."""
        now = time.monotonic()

        # 안전: 배터리 critical이면 누가 뭘 명령하든 무조건 거부한다. 이게
        # cmd_vel_timeout보다 먼저 와야 한다 — 명령이 계속 들어오고 있어도
        # (예: 캘리브레이션 스크립트가 살아서 계속 발행 중) 여기서 막아야
        # 한다. motor_dashboard 미실행 등으로 tier를 아예 모르면(기본값 'ok')
        # 이 분기는 타지 않는다 — 위 __init__ 주석 참고.
        if self._battery_tier == 'critical':
            if self._target_left_pwm or self._target_right_pwm:
                self._target_left_pwm = 0
                self._target_right_pwm = 0
                self._apply_pwm(0, 0)
            self._turn_i = 0.0
            self._ramped_v = 0.0
            self._reset_velocity_state()
            self._prev_control_time = now
            return

        # 안전: 명령이 끊기면 즉시 정지하고 적분항도 비운다.
        if now - self._last_cmd_time > self._cmd_vel_timeout:
            if self._target_left_pwm or self._target_right_pwm:
                self._target_left_pwm = 0
                self._target_right_pwm = 0
                self._apply_pwm(0, 0)
            self._turn_i = 0.0
            self._ramped_v = 0.0
            self._reset_velocity_state()
            self._prev_control_time = now
            return

        dt = now - self._prev_control_time if self._prev_control_time else 0.02
        self._prev_control_time = now
        dt = min(max(dt, 0.001), 0.1)

        # 가속(0에서 멀어지는 방향)에만 선형 램프를 건다. 감속/역전/정지는 목표를
        # 즉시 추종 — 정지 응답성은 안전 요구사항이라 여기서 늦추면 안 된다
        # (위 linear_accel_limit 파라미터 선언부 주석 참고).
        target_v = self._cmd_v
        accelerating = (abs(target_v) > abs(self._ramped_v)
                        and (self._ramped_v == 0.0
                             or (target_v >= 0) == (self._ramped_v >= 0)))
        if accelerating and self._linear_accel_limit > 0:
            max_step = self._linear_accel_limit * dt
            diff = target_v - self._ramped_v
            self._ramped_v += max(-max_step, min(max_step, diff))
        else:
            self._ramped_v = target_v

        v, w = self._ramped_v, self._cmd_w

        if self._control_mode == 'velocity':
            gyro_fresh = (self._gyro_ts is not None
                          and now - self._gyro_ts < self._gyro_stale_sec)
            self._velocity_control(v, w, dt, now, gyro_fresh)
            return

        # --- 직진 성분 (개루프) ---
        if self._invert_drive:
            v = -v          # 새 전진 = 옛 후진 (위 invert_drive_direction 주석 참고)
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

        # 🔄 2026-08-19: **직진에도 폐루프를 적용**한다(w=0 포함).
        # 예전 조건은 `abs(w) > 1e-6`이라 직진 중에는 폐루프가 꺼졌고, 무력한
        # straight_trim만 남았다. 자이로가 "오른쪽으로 휘고 있다"를 뻔히 보면서
        # 그 정보를 안 썼던 것이다.
        #
        # 왜 트림으로는 안 되나 (2026-08-19 실측):
        #   trim을 -0.4~+0.8까지 훑어도 이탈각이 -17°~-12.5°로 4.5°밖에 안 변했고
        #   **부호가 한 번도 안 바뀌었다**. 두 가지 이유:
        #   (a) _speed_to_pwm의 데드존 보정이 트림을 무력화한다. 0.2m/s 명령에서
        #       기본 PWM 160인데, trim 0.4 이상이면 좌측이 min_pwm(120)으로
        #       끌어올려져 **고정된다** (0.4/0.6/0.8이 사실상 같은 결과).
        #   (b) 남은 좌우 PWM 차(약 168)로는 스키드-스티어의 스크럽 마찰을 이기고
        #       요를 만들 수 없다. 제자리 회전 때는 좌우가 반대로 돌아 차이가
        #       1000 이상인데 그 1/6도 안 된다.
        #
        # 완전 정지(v=0, w=0) 상태에서는 끈다 — 자이로 잡음에 반응해 미세하게
        # 움직이면 안 된다.
        moving = abs(w) > 1e-6 or abs(v) > 1e-6
        if self._turn_closed_loop and moving and gyro_fresh:
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

        # 2026-08-22: 회전 중 한쪽 바퀴가 몇 초씩 완전히 안 도는 현상을 실측
        # (반대 방향 회전 직후 왼쪽만 엔코더 변화 0). min_turn_pwm(49)은
        # 피드포워드에만 최소값을 보장하고, 그 위에 얹히는 폐루프 보정이
        # 방향전환 순간 피드포워드를 깎으면 최종 합산 PWM이 49 밑으로 떨어질
        # 수 있는데 이를 막는 장치가 없다 — 사용자 가설(데드존/스크럽마찰
        # 임계값). 다음 재현 시 실제 숫자를 보려고 로깅만 추가, 아직 수정 안 함.
        if abs(w) > 0.05:
            err_val = err if (self._turn_closed_loop and moving and gyro_fresh) else float('nan')
            near_deadzone = (0 < abs(self._target_left_pwm) < self._min_turn_pwm
                              or 0 < abs(self._target_right_pwm) < self._min_turn_pwm)
            level = self.get_logger().warn if near_deadzone else self.get_logger().debug
            level(f'turn_debug w={w:.3f} gyro_z={self._gyro_z:.3f} err={err_val:.3f} '
                  f'turn_pwm={turn:.0f} left={self._target_left_pwm} right={self._target_right_pwm}',
                  throttle_duration_sec=0.2)

        # 2026-08-24: motor_pwm_offset은 직진 드래그(M3) 보정용으로 08-20에
        # 추가됐는데, 08-19에 튜닝된 회전 폐루프(turn_kp/turn_ki)는 이걸 감안한
        # 적이 없다. 순수 회전(v≈0)에서 좌우가 대칭(left=-right)이라고 가정하고
        # 게인을 잡았는데, 오프셋이 그대로 얹히면 비대칭이 커진다 — 실측(예:
        # left=-33/right=+33에 오프셋 적용 시 M1=-108/M2=-33/M3=+273/M4=+33,
        # M3만 8배 가까이 세게 밀림). 재현 시험에서 방향반전 없이 한 방향만
        # 명령해도 자이로z가 명령(0.6)의 3~5배(최대 2.9)까지 발진하는 걸
        # 확인했고, 순수 회전에서 오프셋을 끄니(정지 시 오프셋 미적용과 같은
        # 논리 연장) 평균 0.603/표준편차 0.164로 안정화됨을 실측 확인(수정 전:
        # 부호까지 뒤집히는 발진). 방향전환 재현시험(6회)의 데드존 경고도
        # 17건 -> 3건으로 줄어듦 — 원래 2026-08-22에 미확정이었던 "회전 중
        # 한쪽 바퀴가 안 도는 현상"의 주 원인이었던 것으로 결론.
        apply_offset = abs(v) > 1e-3
        self._apply_pwm(self._target_left_pwm, self._target_right_pwm, apply_offset)

    def _on_imu(self, msg: Imu):
        """자이로 z를 받아 바이어스 보정 후 보관. 회전각은 이 값으로 적분한다."""
        wz = msg.angular_velocity.z

        # 시작 직후 정지 상태에서 바이어스(정지 시 0이 아닌 오프셋) 측정.
        # 이걸 안 빼면 가만히 있어도 각도가 계속 돈다.
        # 모터 명령이 0인 것만으로는 부족하다 — 사람이 들고 있거나 옮기는 중일 수
        # 있다(2026-09-24). 창 하나를 모아 평균/표준편차로 정지 여부를 판정한다.
        if self._gyro_bias_count == 0:
            if self._target_left_pwm or self._target_right_pwm:
                self._gyro_bias_window = []
                return
            # imu_publisher_node는 자기 영점을 잡는 동안(약 2초) z를 0으로 채우고 공분산을
            # 크게 표시한다. 2026-09-26 부팅에서 이 가짜 0만 받아 바이어스 +0.00000(표준편차 0)으로
            # 확정됐다 — 결과는 무해했지만 정지 판정이 가짜 데이터에 속은 것. 그런 샘플은 뺀다.
            if msg.angular_velocity_covariance[8] > 1.0:
                return
            self._gyro_bias_window.append(wz)
            if len(self._gyro_bias_window) < self._gyro_bias_samples_target:
                return
            win = self._gyro_bias_window
            self._gyro_bias_window = []
            mean = sum(win) / len(win)
            std = math.sqrt(sum((x - mean) ** 2 for x in win) / len(win))
            if abs(mean) > self._gyro_bias_max_abs or std > self._gyro_bias_max_std:
                self.get_logger().warn(
                    f'자이로 바이어스 측정 거부 — 평균 {mean:+.4f} / 표준편차 {std:.4f} rad/s '
                    f'(한계 {self._gyro_bias_max_abs} / {self._gyro_bias_max_std}). '
                    '로봇이 움직이는 중으로 보고 다시 잰다. 그동안 회전 폐루프 꺼짐',
                    throttle_duration_sec=5.0)
                return
            self._gyro_bias = mean
            self._gyro_bias_count = len(win)
            self.get_logger().info(
                f'자이로 바이어스 보정 완료: {mean:+.5f} rad/s (표준편차 {std:.4f})')
            return

        # 공분산이 큰(IMU가 "믿기 어렵다"고 표시한 — 영점 확정 전, 모터 전환 직후) 샘플은
        # 회전 폐루프 입력으로 쓰지 않는다(값도 갱신 안 함 → gyro_stale_sec 뒤 폐루프가 꺼짐).
        # 09-26: IMU 재시작 중 가짜 0을 믿은 회전 PI가 출력을 최대로 올려 5분간 약 2.1 rad/s로
        # 돌았다. 모터 전환 직후 0.15초 표시는 gyro_stale_sec(0.3초)보다 짧아 영향 없다.
        if msg.angular_velocity_covariance[8] > 1.0:
            return
        self._gyro_z = wz - self._gyro_bias
        self._gyro_ts = time.monotonic()
        self._imu_seen = True

    def _on_safety_timer(self):
        """cmd_vel이 끊기면 자동 정지 — 명령 유실/노드 죽음 대비."""
        if (self._target_left_pwm or self._target_right_pwm) and \
                time.monotonic() - self._last_cmd_time > self._cmd_vel_timeout:
            self._target_left_pwm = 0
            self._target_right_pwm = 0
            self._apply_pwm(0, 0)
            self.get_logger().warn('cmd_vel 타임아웃 — 정지')

        # 2026-09-04 (Resie 리뷰, task 13): battery_tier가 fail-open이라
        # (motor_dashboard 미실행 시 기본값 'ok'로 계속 주행 허용) — 이건 의도된
        # 설계지만, "한 번은 받았는데 그 뒤로 끊긴" 경우(I2C 불안정 등, CLAUDE.md
        # 기록된 /dev/i2c-1 그룹 권한 재부팅마다 바뀌는 이슈 등)는 조용히
        # 무력화되면 안 된다 — 8/29 사고와 같은 계열의 실패모드. 한 번도 못
        # 받은 경우(motor_dashboard가 애초에 안 떠있는 정상 상황)는 경고하지
        # 않는다 — 그건 fail-open의 정상 범위.
        if self._battery_tier_ts is not None and \
                time.monotonic() - self._battery_tier_ts > 15.0:
            self.get_logger().warn(
                f'battery_tier {time.monotonic()-self._battery_tier_ts:.0f}초간 미수신 — '
                f'마지막 값({self._battery_tier})으로 계속 주행 중(fail-open). '
                f'motor_dashboard/I2C 상태 확인 필요',
                throttle_duration_sec=30.0)

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
            # 2026-08-24: 예전엔 이 경로가 조용히 그냥 return이었다 — 백그라운드
            # 폴링 스레드가 몇 초든 몇십 초든 새 값을 못 읽어도(시리얼 포트 경합,
            # 드라이버 응답 파싱 실패 등) 아무 신호 없이 odom이 멈췄다. 실물
            # 매핑 시험에서 35초간 조용히 멈춘 게 실측 확인됨(원인 후보:
            # motor_dashboard와 포트 동시 점유 — 문서화된 금지 조합인데 그날
            # 안 지켜져 있었음). 얼마나 오래 멈췄는지는 알아야 하므로 경고한다.
            stale_sec = time.monotonic() - now
            if stale_sec > 0.5:
                self.get_logger().warn(
                    f'엔코더 값이 {stale_sec:.1f}초째 갱신 안 됨 — odom이 멈춰있다 '
                    '(시리얼 포트 경합/드라이버 무응답 의심, motor_dashboard 등 '
                    '다른 프로세스가 같은 포트를 열고 있지 않은지 확인할 것)',
                    throttle_duration_sec=2.0)
            return  # 백그라운드 스레드가 아직 새 값을 못 읽음 — 중복 계산 방지

        if self._prev_counts is None:
            self._prev_counts = counts
            self._prev_odom_time = now
            return

        dt = now - self._prev_odom_time
        if dt <= 0:
            return

        # ⚠️ 2026-09-24: 뒷바퀴 520 교체 재배선으로 채널 매핑이 "좌/우 묶음"에서
        # "앞/뒤 묶음"으로 바뀌었다(M1=왼쪽앞,M2=오른쪽앞,M3=왼쪽뒤,M4=오른쪽뒤).
        # 예전 코드는 (M1,M2)를 "왼쪽"으로 묶었는데 그건 옛 매핑 기준이라 지금은
        # 틀림 — 앞왼쪽+오른쪽앞을 "왼쪽"으로 평균내는 셈이었다. 올바른 묶음은
        # 왼쪽=(M1,M3), 오른쪽=(M2,M4). encoder_sign/meters_per_count 둘 다
        # 이제 채널별 리스트(뒷바퀴 520이 감속비가 달라 카운트당 거리도 다름).
        d = [(counts[i] - self._prev_counts[i]) * self._encoder_sign[i] * self._meters_per_count[i]
             for i in range(4)]
        # 2026-09-25: 뒷바퀴 채널 오독 방어 — 속도로 환산해 타당성 검사 후 측별 거리
        side = self._side_values([x / dt for x in d])
        d_left = side['L'] * dt if side['L'] is not None else 0.0
        d_right = side['R'] * dt if side['R'] is not None else 0.0
        self._prev_counts = counts
        self._prev_odom_time = now

        d_center = (d_left + d_right) / 2.0
        # 제자리 회전 명령 중: 전진량 0 (파라미터 주석 참고). 명령이 살아 있을 때만.
        if (self._odom_zero_in_place
                and abs(self._cmd_v) <= self._turn_in_place_max_v and abs(self._cmd_w) > 1e-3
                and time.monotonic() - self._last_cmd_time < self._cmd_vel_timeout):
            d_center = 0.0

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
