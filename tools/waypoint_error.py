#!/usr/bin/env python3
"""웨이포인트 오차 계산/비교 도구 (Issue #3, Track 0 "웨이포인트 대조" 지원용).

Remi가 실물 로봇으로 "시작점 → 방 중앙 → (복귀)"를 이동시켜 도착 시점의
휠 단독(/odom)/EKF(/odometry/filtered) pose를 재는데, 그 결과가 `our_home`
정답 지도(GIMP 손트레이싱, 오차 없음) 기준으로 얼마나 정확한지 채점하는
용도. round_trip_drift.py처럼 두 채널(휠/EKF)을 나란히 비교한다 — 다만
이건 로봇을 직접 조작하지 않는 **오프라인 채점 도구**다(실물 측정은
waypoint_capture.py 또는 Remi가 준비하는 별도 스크립트가 담당).

세 가지 오차를 함께 낸다(어느 하나만으로는 놓치는 게 있어서):
  - 위치 오차(m): 목표 지점과 실제 도착점 사이의 유클리드 거리.
    "웨이포인트에 얼마나 정확히 도착했나"를 가장 직관적으로 보여주는 메인 지표.
  - 거리 오차(m): 이동한 거리 자체의 차이(부호 있음, 실제-목표).
    방위는 맞았는데 못 미치거나 지나쳤는지를 구분해준다.
  - 방위 오차(°): 이동 방향(시작→도착 벡터의 방위각)의 차이(부호 있음).
    거리는 맞았는데 엉뚱한 방향으로 갔는지를 구분해준다.
  위치 오차만으로는 "거리가 틀렸는지 방향이 틀렸는지"를 못 가르기 때문에
  셋을 함께 보고한다.

좌표계: our_home.png는 0.05m/px, 크기 14.9x9.85m, 이미지 row 0(맨 위)이
world +y에 대응한다(2026-08-24 Phase 2에서 알려진 시작점(-2.7, 3.43)의
픽셀좌표와 정확히 일치시켜 검증된 매핑 — tools/phase2_multi_start.py 참고).

사용법:
    # 자체 검증 (합성 데이터, 로봇/실물 로그 불필요)
    python3 tools/waypoint_error.py --self-test

    # 실물 로그 채점 (비트맵 픽셀좌표로 시작/목표 지정)
    python3 tools/waypoint_error.py \
        --start-px 95 30 --target-px 155 55 \
        --log docs/calibration/waypoint_run1.json

    # world 미터 좌표로 직접 지정(비트맵 없이)
    python3 tools/waypoint_error.py \
        --start-xy -2.7 3.43 --target-xy 0.3 2.17 \
        --log docs/calibration/waypoint_run1.json
"""
import argparse
import json
import math
import sys

# our_home.png 비트맵 상수 (robot/src/stage_ros2/world/our_home.world 주석 참고)
IMG_W, IMG_H = 298, 197
WORLD_W, WORLD_H = 14.9, 9.85


def pixel_to_world(px, py):
    """비트맵 픽셀좌표 -> world 미터좌표. row 0(이미지 맨 위) = world +y.
    2026-08-24 Phase 2에서 알려진 시작점(-2.7, 3.43)과 정확히 일치 검증됨."""
    wx = (px / IMG_W - 0.5) * WORLD_W
    wy = (0.5 - py / IMG_H) * WORLD_H
    return wx, wy


def target_displacement(start_xy, target_xy):
    """두 world 좌표 사이의 목표 변위 — (거리 m, 방위각 도, world 좌표계
    기준 atan2(dy,dx), 0도=+x축, 반시계 양수)."""
    dx = target_xy[0] - start_xy[0]
    dy = target_xy[1] - start_xy[1]
    dist = math.hypot(dx, dy)
    bearing = math.degrees(math.atan2(dy, dx))
    return dist, bearing


def wrap_deg(a):
    """각도를 -180~180 범위로 정규화."""
    return (a + 180.0) % 360.0 - 180.0


def pose_error(start_xy, end_xy, target_dist, target_bearing):
    """실제 이동(start_xy -> end_xy)이 목표 변위(target_dist, target_bearing)를
    얼마나 정확히 달성했는지 세 가지 오차로 계산.

    Returns dict: actual_dist_m, actual_bearing_deg, position_error_m,
    distance_error_m(부호 있음, 실제-목표), bearing_error_deg(부호 있음, 실제-목표)."""
    actual_dist, actual_bearing = target_displacement(start_xy, end_xy)

    target_x = start_xy[0] + target_dist * math.cos(math.radians(target_bearing))
    target_y = start_xy[1] + target_dist * math.sin(math.radians(target_bearing))
    position_error_m = math.hypot(end_xy[0] - target_x, end_xy[1] - target_y)

    distance_error_m = actual_dist - target_dist
    bearing_error_deg = wrap_deg(actual_bearing - target_bearing)

    return {
        'actual_dist_m': actual_dist,
        'actual_bearing_deg': actual_bearing,
        'position_error_m': position_error_m,
        'distance_error_m': distance_error_m,
        'bearing_error_deg': bearing_error_deg,
    }


def load_log(path):
    """waypoint_capture.py 또는 동등한 포맷의 JSON 로그를 읽는다.
    필수 키: start_raw, start_ekf, end_raw, end_ekf (각각 [x, y, yaw_deg])."""
    with open(path) as f:
        data = json.load(f)
    required = ('start_raw', 'start_ekf', 'end_raw', 'end_ekf')
    missing = [k for k in required if k not in data]
    if missing:
        raise ValueError(f'로그 파일에 필수 키 누락: {missing}')
    return data


def print_report(target_dist, target_bearing, err_raw, err_ekf):
    print(f'목표 변위: {target_dist:.3f}m @ {target_bearing:+.1f}°\n')
    for label, err in (('휠 단독(/odom)', err_raw), ('EKF(/odometry/filtered)', err_ekf)):
        print(f'  {label}')
        print(f'    실제 이동      : {err["actual_dist_m"]:.3f}m @ {err["actual_bearing_deg"]:+.1f}°')
        print(f'    위치 오차       : {err["position_error_m"]*100:.1f}cm')
        print(f'    거리 오차(실-목): {err["distance_error_m"]*100:+.1f}cm')
        print(f'    방위 오차(실-목): {err["bearing_error_deg"]:+.1f}°')
        print()
    if err_ekf['position_error_m'] < err_raw['position_error_m']:
        print('  -> EKF가 휠 단독보다 정확하다.')
    else:
        print('  -> ⚠️ EKF가 휠 단독보다 오히려 부정확하다 — 확인 필요.')


def run_from_log(args):
    if args.start_px:
        start_xy = pixel_to_world(*args.start_px)
    else:
        start_xy = tuple(args.start_xy)
    if args.target_px:
        target_xy = pixel_to_world(*args.target_px)
    else:
        target_xy = tuple(args.target_xy)

    target_dist, target_bearing = target_displacement(start_xy, target_xy)
    data = load_log(args.log)

    # 로그 자체에 기록된 시작점(실물 로봇의 odom 원점)을 기준으로 오차를 낸다 —
    # start_xy(정답 지도 좌표)는 목표 변위 계산에만 쓰고, 실제 이동 오차는
    # 로봇 좌표계(odom 원점=0,0) 안에서 상대적으로 비교하는 게 맞다. odom/EKF는
    # 전역 좌표가 아니라 시작 시점을 원점으로 하는 상대 좌표이기 때문.
    start_raw = tuple(data['start_raw'][:2])
    start_ekf = tuple(data['start_ekf'][:2])
    end_raw = tuple(data['end_raw'][:2])
    end_ekf = tuple(data['end_ekf'][:2])

    err_raw = pose_error(start_raw, end_raw, target_dist, target_bearing)
    err_ekf = pose_error(start_ekf, end_ekf, target_dist, target_bearing)

    print_report(target_dist, target_bearing, err_raw, err_ekf)

    if args.json_out:
        with open(args.json_out, 'w') as f:
            json.dump({
                'target_dist_m': target_dist, 'target_bearing_deg': target_bearing,
                'start_xy': start_xy, 'target_xy': target_xy,
                'error_raw': err_raw, 'error_ekf': err_ekf,
            }, f, ensure_ascii=False, indent=1)
        print(f'저장: {args.json_out}')


def self_test():
    """합성 데이터로 도구 자체를 검증한다 — 실물/로봇 불필요.
    목표 변위에 알려진 오차를 의도적으로 주입한 가짜 도착점을 만들어서,
    pose_error()가 그 오차를 정확히 되짚어내는지 확인한다."""
    failures = []

    def check(name, cond, detail):
        status = 'PASS' if cond else 'FAIL'
        print(f'  [{status}] {name}' + (f' — {detail}' if not cond and detail else ''))
        if not cond:
            failures.append(name)

    # 1) 정확히 목표대로 이동 -> 오차 전부 0
    start = (0.0, 0.0)
    target_dist, target_bearing = 5.0, 30.0
    end = (start[0] + target_dist * math.cos(math.radians(target_bearing)),
           start[1] + target_dist * math.sin(math.radians(target_bearing)))
    e = pose_error(start, end, target_dist, target_bearing)
    check('완벽한 이동 -> 위치오차 0',
          abs(e['position_error_m']) < 1e-9, f"got {e['position_error_m']}")
    check('완벽한 이동 -> 거리오차 0',
          abs(e['distance_error_m']) < 1e-9, f"got {e['distance_error_m']}")
    check('완벽한 이동 -> 방위오차 0',
          abs(e['bearing_error_deg']) < 1e-9, f"got {e['bearing_error_deg']}")

    # 2) 거리만 +30cm 오차 주입, 방위는 정확 -> 위치오차도 정확히 30cm여야 함
    injected_dist_err = 0.30
    end2 = (start[0] + (target_dist + injected_dist_err) * math.cos(math.radians(target_bearing)),
            start[1] + (target_dist + injected_dist_err) * math.sin(math.radians(target_bearing)))
    e2 = pose_error(start, end2, target_dist, target_bearing)
    check('거리만 +30cm 주입 -> distance_error_m≈+0.30',
          abs(e2['distance_error_m'] - injected_dist_err) < 1e-6,
          f"got {e2['distance_error_m']:.4f}")
    check('거리만 +30cm 주입 -> position_error_m≈0.30(방위 안 틀렸으므로 동일)',
          abs(e2['position_error_m'] - injected_dist_err) < 1e-6,
          f"got {e2['position_error_m']:.4f}")
    check('거리만 +30cm 주입 -> bearing_error_deg≈0',
          abs(e2['bearing_error_deg']) < 1e-6, f"got {e2['bearing_error_deg']:.4f}")

    # 3) 방위만 +5도 오차 주입, 거리는 정확 -> position_error는 현(chord) 길이 공식과 일치해야 함
    injected_bearing_err = 5.0
    bearing3 = target_bearing + injected_bearing_err
    end3 = (start[0] + target_dist * math.cos(math.radians(bearing3)),
            start[1] + target_dist * math.sin(math.radians(bearing3)))
    e3 = pose_error(start, end3, target_dist, target_bearing)
    expected_chord = 2 * target_dist * math.sin(math.radians(injected_bearing_err) / 2)
    check('방위만 +5도 주입 -> bearing_error_deg≈+5',
          abs(e3['bearing_error_deg'] - injected_bearing_err) < 1e-6,
          f"got {e3['bearing_error_deg']:.4f}")
    check('방위만 +5도 주입 -> distance_error_m≈0',
          abs(e3['distance_error_m']) < 1e-6, f"got {e3['distance_error_m']:.4f}")
    check('방위만 +5도 주입 -> position_error_m≈현길이 공식',
          abs(e3['position_error_m'] - expected_chord) < 1e-6,
          f"got {e3['position_error_m']:.4f}, expected {expected_chord:.4f}")

    # 4) 거리+방위 동시 오차, 대략적 범위(예시 케이스: 30cm/5도) 안에서 재현되는지
    #    -- Issue #3 예시("29~31cm/4~6도 범위면 통과")와 동일한 형태의 관용 검증
    dist4, bearing4 = 3.0, -60.0
    dist_err4, bearing_err4 = 0.30, 5.0
    end4 = (start[0] + (dist4 + dist_err4) * math.cos(math.radians(bearing4 + bearing_err4)),
            start[1] + (dist4 + dist_err4) * math.sin(math.radians(bearing4 + bearing_err4)))
    e4 = pose_error(start, end4, dist4, bearing4)
    check('복합 오차(거리+30cm, 방위+5도) -> distance_error 29~31cm 범위',
          0.29 <= e4['distance_error_m'] <= 0.31, f"got {e4['distance_error_m']*100:.1f}cm")
    check('복합 오차(거리+30cm, 방위+5도) -> bearing_error 4~6도 범위',
          4.0 <= e4['bearing_error_deg'] <= 6.0, f"got {e4['bearing_error_deg']:.2f}°")

    # 5) 픽셀->world 변환 왕복 및 알려진 기준점 검증
    known_px, known_world = (95, 30), (-2.7, 3.43)
    got_world = pixel_to_world(*known_px)
    check('pixel_to_world(95,30) ≈ 알려진 시작점(-2.7, 3.43) (±3cm, 격자반올림 오차)',
          math.hypot(got_world[0] - known_world[0], got_world[1] - known_world[1]) < 0.03,
          f'got {got_world}')

    # 6) 각도 정규화(wrap_deg) 경계 케이스
    check('wrap_deg(190) == -170', abs(wrap_deg(190) - (-170)) < 1e-9, f'got {wrap_deg(190)}')
    check('wrap_deg(-190) == 170', abs(wrap_deg(-190) - 170) < 1e-9, f'got {wrap_deg(-190)}')
    check('wrap_deg(0) == 0', abs(wrap_deg(0)) < 1e-9, f'got {wrap_deg(0)}')

    print()
    if failures:
        print(f'❌ {len(failures)}개 실패: {failures}')
        return 1
    print(f'✅ 전부 통과')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--self-test', action='store_true',
                     help='합성 데이터로 도구 자체 검증 (실물/로봇 불필요)')
    ap.add_argument('--start-px', type=float, nargs=2, metavar=('PX', 'PY'),
                     help='시작점 비트맵 픽셀좌표')
    ap.add_argument('--target-px', type=float, nargs=2, metavar=('PX', 'PY'),
                     help='목표점 비트맵 픽셀좌표')
    ap.add_argument('--start-xy', type=float, nargs=2, metavar=('X', 'Y'),
                     help='시작점 world 미터좌표 (--start-px 대신 사용 가능)')
    ap.add_argument('--target-xy', type=float, nargs=2, metavar=('X', 'Y'),
                     help='목표점 world 미터좌표 (--target-px 대신 사용 가능)')
    ap.add_argument('--log', help='waypoint_capture.py 로그 JSON 파일')
    ap.add_argument('--json-out', help='결과를 JSON으로도 저장')
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if not args.log:
        ap.error('--log가 필요합니다 (또는 --self-test로 자체 검증만 실행)')
    if not (args.start_px or args.start_xy):
        ap.error('--start-px 또는 --start-xy 중 하나가 필요합니다')
    if not (args.target_px or args.target_xy):
        ap.error('--target-px 또는 --target-xy 중 하나가 필요합니다')

    run_from_log(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
