"""바퀴/채널 구성의 단일 출처 (tools/ 스크립트 공용).

왜 있나 (2026-09-24): 채널 매핑·엔코더 부호·카운트당거리가 스크립트 7곳에
각자 복사돼 있었다. 뒷바퀴 520 교체로 매핑이 바뀌었을 때 diff_drive_node만
고치고 나머지를 빠뜨려서, precise_move.py가 전진 거리를 거의 0으로 읽는
상태(목표 미도달로 판단해 타임아웃까지 계속 주행 = 충돌 위험)로 남아 있었다.

⚠️ 런타임 기준은 diff_drive_node의 ROS 파라미터다. 이 파일은 그 값을 따라가는
사본이므로, 한쪽을 바꾸면 다른 쪽도 같은 커밋에서 바꿀 것 —
tools/hw_preflight.py가 둘이 일치하는지 검사한다.
절차: docs/hardware_change_procedure.md
"""
import math

WHEEL_RADIUS = 0.0335  # 앞/뒤 동일(지름 67mm)

# 인덱스 = 드라이버 채널 - 1.  $MAll/$pwm 순서와 같다.
NAMES = ['M1 왼쪽앞(370)', 'M2 오른쪽앞(370)', 'M3 왼쪽뒤(520)', 'M4 오른쪽뒤(520)']
LEFT = (0, 2)
RIGHT = (1, 3)
FRONT = (0, 1)
REAR = (2, 3)

# 전진할 때 원시 엔코더가 줄어들면 -1 (실측 2026-09-24)
ENCODER_SIGN = [-1, 1, 1, -1]
# 드라이버에 원시 $pwm을 직접 보낼 때 "전진"이 되는 부호 (오른쪽 극성 반전 배선)
PWM_SIGN = [1, -1, 1, -1]
# 앞 370: 857.2(주행 보정값) / 뒤 520: 2475.6(손 10회전 raw, 주행 보정 전)
COUNTS_PER_REV = [857.2, 857.2, 2475.6, 2475.6]
METERS_PER_COUNT = [(2.0 * math.pi * WHEEL_RADIUS) / c for c in COUNTS_PER_REV]


def wheel_distances(a, b):
    """두 $MAll 스냅샷 사이 바퀴별 이동거리(m). 전진이 +."""
    return [(b[i] - a[i]) * ENCODER_SIGN[i] * METERS_PER_COUNT[i] for i in range(4)]


def left_right(a, b):
    """(왼쪽 평균, 오른쪽 평균) 이동거리(m)."""
    d = wheel_distances(a, b)
    return (sum(d[i] for i in LEFT) / 2.0, sum(d[i] for i in RIGHT) / 2.0)


def center(a, b):
    """차체 중심 이동거리(m)."""
    l, r = left_right(a, b)
    return (l + r) / 2.0


def raw_pwm(pwm, channels):
    """지정 채널들이 '전진' 방향 pwm으로 도는 $pwm 4원소 리스트."""
    vals = [0, 0, 0, 0]
    for i in channels:
        vals[i] = int(pwm * PWM_SIGN[i])
    return vals


if __name__ == '__main__':
    # 자가검증: 2026-09-24 바닥 직진(cmd_vel 0.2) 중 0.16초 구간 실측 원시 델타
    a = [0, 0, 0, 0]
    b = [-405, 347, 999, -983]
    d = wheel_distances(a, b)
    assert all(x > 0 for x in d), f'전진인데 음수 바퀴가 있다: {d}'
    spread = max(d) / min(d)
    assert spread < 1.25, f'바퀴 간 거리 차가 너무 크다: {d}'
    l, r = left_right(a, b)
    print(f'OK  바퀴별 {[f"{x:.4f}" for x in d]} m, 좌 {l:.4f} / 우 {r:.4f}')
