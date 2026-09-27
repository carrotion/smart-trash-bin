#!/usr/bin/env python3
"""4층 주행기(step_navigator) 로직 오프라인 가상 시험 — 로봇·ROS 그래프 없이.

가상 차체: 회전은 base_link가 아니라 **회전 중심(뒤로 rot_off m)** 을 축으로 돈다(09-27 합격 시험에서 마지막
150~175° 회전이 위치를 앞뒤로 3~10 cm 민 현상 재현), 회전·직진에 배율 오차, 장애물 없음(스캔은 전부 먼 거리).
경로 계획은 직선. 확인 항목:
  1. 복귀 + 마지막 방향 맞춤 회전: 회전 중심 어긋남이 있어도 도착 ≤ 3 cm(앞뒤 보정)·방향 ≤ 5°
  2. 5~12 cm 뒤 목표: 180° 돌지 않고 후진으로 도착(회전 합 < 20°)
  3. 뒤가 막히면 후진하지 않고 보통 경로(돌아서 전진)로 도착
  4. 균등 분할: 1.28 m 목표가 조기 정지 없이 ≤ 3 cm

    python3 tools/test_step_navigator_sim.py
"""
import math, os, sys, types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_navigator import StepNavigator
from turn_to_angle import wrap


class Scan:
    def __init__(self, rear_block=None):
        self.angle_min, self.angle_increment, self.range_min, self.range_max = -math.pi, 2 * math.pi / 455, 0.02, 12.0
        self.ranges = [5.0] * 455
        if rear_block is not None:   # 로봇 바로 뒤(라이다 기준 180°)에 벽
            for i in range(455):
                a = self.angle_min + i * self.angle_increment
                if abs(wrap(a - math.pi)) < math.radians(25):
                    self.ranges[i] = rear_block


def make(rot_off=0.04, turn_scale=1.02, drive_scale=1.01, rear_block=None):
    nav = StepNavigator.__new__(StepNavigator)
    nav.step_m, nav.xy_tol, nav.yaw_tol, nav.turn_tol = 0.4, 0.05, math.radians(5), math.radians(5)
    nav.settle_s, nav.blocked_max, nav.clear_margin, nav.back_max, nav.side_margin = 0.5, 3, 0.25, 0.15, 0.05
    nav.log = []; P = {'x': 0.0, 'y': 0.0, 'th': 0.0, 'turned': 0.0}
    nav.S = {'scan': Scan(rear_block), 'pose': None, 'v': 0.0, 'gz': 0.0}
    nav.settle = lambda: None; nav.spin = lambda t: None
    nav.map_pose = lambda: (P['x'], P['y'], P['th'])
    nav.plan = lambda here, goal: ([(here[0] + (goal[0] - here[0]) * k / 40, here[1] + (goal[1] - here[1]) * k / 40) for k in range(41)], '')
    nav._done = types.MethodType(StepNavigator._done, nav)
    nav.pub = types.SimpleNamespace(publish=lambda m: None)

    def turn_by(d):
        d2 = d * turn_scale
        cx, cy = P['x'] - rot_off * math.cos(P['th']), P['y'] - rot_off * math.sin(P['th'])   # 회전 중심(뒤로 rot_off)
        c, s = math.cos(d2), math.sin(d2); rx, ry = P['x'] - cx, P['y'] - cy
        P['x'], P['y'] = cx + c * rx - s * ry, cy + s * rx + c * ry
        P['th'] = wrap(P['th'] + d2); P['turned'] += abs(d2)
        return dict(ok=True, msg='', err_deg=0.0, fixes=0, secs=1.0)

    def drive_by(d):
        d2 = d * drive_scale
        P['x'] += d2 * math.cos(P['th']); P['y'] += d2 * math.sin(P['th'])
        return dict(ok=True, msg='', err_m=0.0, fixes=0, secs=1.0)
    nav.turner = types.SimpleNamespace(turn_by=turn_by)
    nav.driver = types.SimpleNamespace(drive_by=drive_by)
    return nav, P


def main():
    import builtins
    quiet = builtins.print
    res = []

    def check(name, cond, detail):
        res.append(cond); quiet(f'{"✅" if cond else "❌"} {name}: {detail}')

    builtins.print = lambda *a, **k: None
    try:
        # 1. 1 m 앞 → 출발점 복귀 + 원래 방향(180° 가까운 마지막 회전)
        for off in (0.0, 0.03, 0.06):
            nav, P = make(rot_off=off); nav.go((1.0, 0.0, None)); r = nav.go((0.0, 0.0, 0.0))
            e = math.hypot(P['x'], P['y']) * 100; ye = math.degrees(P['th'])
            kinds = [x['kind'] for x in nav.log]
            builtins.print = quiet
            check(f'복귀+마지막 회전(회전 중심 뒤 {off * 100:.0f} cm)', r['ok'] and e <= 3.0 and abs(ye) <= 5,
                  f'{e:.1f} cm, {ye:+.1f}°, 앞뒤 보정 {kinds.count("final_fix")}회')
            builtins.print = lambda *a, **k: None
        # 2. 가까운 뒤 목표 → 후진
        for back in (0.06, 0.12):
            nav, P = make(); P['x'] = back; r = nav.go((0.0, 0.0, None))
            builtins.print = quiet
            check(f'{back * 100:.0f} cm 뒤 목표 후진', r['ok'] and math.degrees(P['turned']) < 20 and math.hypot(P['x'], P['y']) <= 0.05,
                  f'회전 합 {math.degrees(P["turned"]):.0f}°, 남은 {math.hypot(P["x"], P["y"]) * 100:.1f} cm, {[x["kind"] for x in nav.log]}')
            builtins.print = lambda *a, **k: None
        # 3. 뒤 막힘 → 돌아서 전진
        # 차체 뒤 약 19 cm 벽(뒤 0.1 m 안은 부착물 자리라 일부러 무시하는 범위)
        nav, P = make(rear_block=0.35); P['x'] = 0.12; r = nav.go((0.0, 0.0, None))
        builtins.print = quiet
        # 가상 벽은 차체에 붙어 있어(돌아도 늘 뒤) 가장 나쁜 경우 — 진동 없이 합격 기준(10 cm) 안에서 끝나야
        check('뒤 막힘이면 후진 안 함·진동 없음', r['ok'] and 'back' not in [x['kind'] for x in nav.log] and math.hypot(P['x'], P['y']) <= 0.10
              and len(nav.log) <= 8,
              f'{[x["kind"] for x in nav.log]}, 남은 {math.hypot(P["x"], P["y"]) * 100:.1f} cm')
        builtins.print = lambda *a, **k: None
        # 5. 옆 물체: 옆면에서 7 cm(09-27 병 사건)면 직진 허용, 3 cm면 막힘 — 통로 옆 여유 5 cm
        from speed_floor_sweep import corridor_clearance
        for side_gap, want_block in ((0.07, False), (0.03, True)):
            sc = Scan(); y = -(0.11 + side_gap)
            for i in range(455):
                a = sc.angle_min + i * sc.angle_increment
                for x in (0.12, 0.15, 0.18):   # 차체 앞끝 바로 옆, 병 크기
                    if abs(math.atan2(y, x - 0.054) - a) < sc.angle_increment / 2:
                        sc.ranges[i] = math.hypot(x - 0.054, y)
            room = corridor_clearance(sc, True, 0.0, 0.05)
            builtins.print = quiet
            check(f'옆면 {side_gap * 100:.0f} cm 물체', (room < 0.3) == want_block, f'앞 여유 {room:.2f} m → {"막힘" if room < 0.3 else "통과"}')
            builtins.print = lambda *a, **k: None
        # 4. 균등 분할
        nav, P = make(rot_off=0.0, drive_scale=1.0); r = nav.go((1.28, 0.0, None))
        builtins.print = quiet
        check('1.28 m 균등 분할', r['ok'] and abs(P['x'] - 1.28) <= 0.03,
              f'{(P["x"] - 1.28) * 100:+.1f} cm, 걸음 {[x["arg"] for x in nav.log if x["kind"] == "drive"]}')
    finally:
        builtins.print = quiet
    print(f'\n4층 주행기 가상 시험: {sum(res)}/{len(res)} 통과')
    return 0 if all(res) else 1


if __name__ == '__main__':
    sys.exit(main())
