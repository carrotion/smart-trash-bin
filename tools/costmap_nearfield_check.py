#!/usr/bin/env python3
"""로컬 코스트맵의 근접 사각지대(obstacle_min_range 원점 오류) 수정 검증 도구.

배경: 2026-09-04, nav2_costmap_2d 소스 확인 결과 obstacle_min_range/
raytrace_min_range의 근접 컷오프 기준점이 base_link가 아니라 **라이다 센서
원점**이었다. 라이다가 base_link보다 5.4cm 앞에 있어서, 기존 설정(0.11m)은
실제로 base_link 기준 0.164m까지 장애물 마킹을 막고 있었다 — 로봇 몸통
앞끝(0.11m)에 물체가 닿기 직전까지도 코스트맵엔 안 보이는 사각지대. 두 값을
0.0으로 낮춰 self-filter(±0.105m 폴리곤, 이미 base_link 기준으로 정확히
자기 구조물을 거름)에 전적으로 의존하도록 수정.

이 도구는 base_link 정면 축 위 여러 거리에 있는 로컬 코스트맵 셀의 비용값을
실시간으로 찍어서, 사용자가 그 근처에 얇은 물체를 대는 동안 **어느 거리부터
LETHAL(100)로 마킹되는지** 직접 확인한다. 수정 전엔 ~16cm까지 마킹 안 됨,
수정 후엔 ~10~11cm부터 마킹돼야 정상(자기 몸체 바로 앞).

사용법:
    python3 tools/costmap_nearfield_check.py [초, 기본 40]
"""
import rclpy, time, sys, math
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from tf2_ros import Buffer, TransformListener

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
OFFSETS_M = [0.08, 0.10, 0.11, 0.12, 0.13, 0.14, 0.16, 0.18, 0.20, 0.25, 0.30]


def main():
    rclpy.init()
    n = Node('costmap_nearfield_check')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, n)
    state = {}
    n.create_subscription(Costmap, '/local_costmap/costmap_raw',
                           lambda m: state.update(cm=m), 2)

    t0 = time.time()
    while 'cm' not in state and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'cm' not in state:
        print('구독 실패 — /local_costmap/costmap_raw가 안 보임. Nav2 스택 확인.')
        return 1

    print(f'검증 시작 — {DURATION:.0f}초. 로봇 정면 축 위, 표시된 거리 부근에 '
          f'얇은 물체를 대보면서 어느 지점부터 LETHAL로 바뀌는지 관찰하면 됨.')
    print(f'참고: 자기 구조물 폴리곤 경계 10.5cm, 로봇 앞면 11cm, '
          f'수정 전 사각지대 끝 16.4cm\n')

    t_end = time.time() + DURATION
    last_report = 0.0

    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.time()
        if now - last_report < 1.5:
            continue
        last_report = now

        cm = state.get('cm')
        if cm is None:
            continue

        try:
            tf = tf_buffer.lookup_transform(cm.header.frame_id, 'base_link', rclpy.time.Time())
        except Exception as e:
            print(f'TF 조회 실패: {e}')
            continue

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        res = cm.metadata.resolution
        ox = cm.metadata.origin.position.x
        oy = cm.metadata.origin.position.y
        w = cm.metadata.size_x
        h = cm.metadata.size_y

        # 정면 축 한 줄만 보면 물체가 중앙선에서 살짝만 벗어나도 놓친다 —
        # 전방 반원(0.4m 반경, base_link 기준 전방 ±90도) 안의 셀을 전부 훑어서
        # LETHAL(>=253) 중 base_link에 가장 가까운 것의 실제 거리를 찾는다.
        best_lethal = None
        cells_r = int(0.4 / res)
        bx_cell = int((bx - ox) / res)
        by_cell = int((by - oy) / res)
        for dy_c in range(-cells_r, cells_r + 1):
            for dx_c in range(-cells_r, cells_r + 1):
                mx, my = bx_cell + dx_c, by_cell + dy_c
                if not (0 <= mx < w and 0 <= my < h):
                    continue
                cost = cm.data[my * w + mx]
                # 253=INSCRIBED_INFLATED_OBSTACLE, 254=LETHAL_OBSTACLE — 진짜
                # 장애물만. 255=NO_INFORMATION(미관측)은 장애물이 아니다 —
                # 처음 버전이 이걸 놓쳐서 로봇 바로 밑 미관측 셀을 "장애물"로
                # 오판했었다(2026-09-04 발견, 원본 커밋 참고).
                if cost not in (253, 254):
                    continue
                wx = ox + (mx + 0.5) * res
                wy = oy + (my + 0.5) * res
                # 로봇 기준 전방인지(뒤쪽 오탐 배제, 대략 전방 180도)
                rel_ang = math.atan2(wy - by, wx - bx) - yaw
                rel_ang = (rel_ang + math.pi) % (2 * math.pi) - math.pi
                if abs(rel_ang) > math.pi / 2:
                    continue
                dist = math.hypot(wx - bx, wy - by)
                if best_lethal is None or dist < best_lethal:
                    best_lethal = dist

        parts = []
        for d in OFFSETS_M:
            wx = bx + d * math.cos(yaw)
            wy = by + d * math.sin(yaw)
            mx = int((wx - ox) / res)
            my = int((wy - oy) / res)
            if 0 <= mx < w and 0 <= my < h:
                cost = cm.data[my * w + mx]
            else:
                cost = -1
            tag = 'L' if cost in (253, 254) else ('u' if cost == 255 else ('i' if cost > 0 else 'f'))
            parts.append(f'{d*100:.0f}={cost}{tag}')
        lethal_str = f'최근접 LETHAL={best_lethal*100:.1f}cm' if best_lethal is not None else '전방 반경 0.4m 내 LETHAL 없음'
        print(f'[{now-t0:5.1f}s] {lethal_str}  |  정면축: ' + ' '.join(parts))

    print('\n종료. LETHAL(253~255)이 몇 cm부터 뜨는지가 실제 근접 사각지대 끝.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
