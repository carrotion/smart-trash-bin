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


class WorldScan(dict):
    """'scan'을 읽을 때마다 세상에 고정된 점들(world)을 현재 자세 P에서 본 스캔으로 만든다."""
    def __init__(self, P, world, **kw):
        super().__init__(**kw); self.P, self.world = P, world

    def __getitem__(self, k):
        if k != 'scan':
            return super().__getitem__(k)
        sc = Scan(); c, s = math.cos(self.P['th']), math.sin(self.P['th'])
        for wx, wy in self.world:
            dx, dy = wx - self.P['x'], wy - self.P['y']; x, y = c * dx + s * dy - 0.054, -s * dx + c * dy
            r = math.hypot(x, y); i = int((math.atan2(y, x) - sc.angle_min) / sc.angle_increment) % 455
            sc.ranges[i] = min(sc.ranges[i], r)
        return sc


def bottle(cx, cy, rad=0.033):
    return [(cx + rad * math.cos(k * math.pi / 8), cy + rad * math.sin(k * math.pi / 8)) for k in range(16)]


def make(rot_off=0.04, turn_scale=1.02, drive_scale=1.01, rear_block=None, world=None):
    nav = StepNavigator.__new__(StepNavigator)
    nav.step_m, nav.xy_tol, nav.yaw_tol, nav.turn_tol = 0.4, 0.05, math.radians(5), math.radians(5)
    nav.settle_s, nav.blocked_max, nav.clear_margin, nav.back_max, nav.side_margin = 0.5, 3, 0.25, 0.15, 0.05
    nav.rot_radius = 0.22
    nav.log = []; P = {'x': 0.0, 'y': 0.0, 'th': 0.0, 'turned': 0.0, 'turn_clear': []}
    if world is not None:
        nav.S = WorldScan(P, world, pose=None, v=0.0, gz=0.0)
    else:
        nav.S = {'scan': Scan(rear_block), 'pose': None, 'v': 0.0, 'gz': 0.0}
    nav.settle = lambda: None; nav.spin = lambda t: None
    nav.map_pose = lambda: (P['x'], P['y'], P['th'])
    nav.plan = lambda here, goal: ([(here[0] + (goal[0] - here[0]) * k / 40, here[1] + (goal[1] - here[1]) * k / 40) for k in range(41)], '')
    nav._done = types.MethodType(StepNavigator._done, nav)
    nav.pub = types.SimpleNamespace(publish=lambda m: None)

    def turn_by(d):
        if world:   # 돌기 시작할 때 로봇 중심에서 가장 가까운 장애물 거리
            P['turn_clear'].append(min(math.hypot(wx - P['x'], wy - P['y']) for wx, wy in world))
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
        # 6. 병이 오른쪽 옆면 7 cm(중심 0.21 m)에 붙은 채 90° 왼쪽 목표 → 떨어진 뒤 회전, 돌 때 장애물 > 0.20 m
        nav, P = make(world=bottle(0.03, -0.21)); r = nav.go((0.0, 1.0, None))
        kinds = [x['kind'] for x in nav.log]
        builtins.print = quiet
        check('옆 병 → 떨어진 뒤 회전', r['ok'] and 'rot_clear' in kinds and min(P['turn_clear']) > 0.20,
              f'돌 때 최소 거리 {min(P["turn_clear"]) * 100:.1f} cm, {kinds[:5]}, 도착 {r["final_err_m"] * 100:.1f} cm')
        builtins.print = lambda *a, **k: None
        # 7. 앞·뒤·옆이 다 막힘 → 돌지 않고 막힘 보고
        wall = bottle(0.2, 0.0) + bottle(-0.24, 0.0) + bottle(0.03, -0.2) + bottle(0.03, 0.2) + bottle(0.2, 0.12) + bottle(-0.2, -0.12)
        nav, P = make(world=wall); r = nav.go((0.0, 1.0, None))
        builtins.print = quiet
        check('사방 막힘 → 돌지 않음', not r['ok'] and not P['turn_clear'], f'{r["msg"]}, 회전 {len(P["turn_clear"])}회')
        builtins.print = lambda *a, **k: None
        # 8. 돌기 전 미리 보기(_room_dir): 지금 스캔으로 계산한 "돈 뒤 앞 여유"가 실제로 돈 뒤(회전 중심 어긋남 없이)의 여유와 같은지
        from speed_floor_sweep import corridor_clearance as cc
        worst = 0.0
        for deg in (-120, -60, -25, 0, 25, 60, 120):
            nav, P = make(rot_off=0.0, turn_scale=1.0, world=bottle(0.45, 0.10) + bottle(-0.2, 0.5) + bottle(0.3, -0.45))
            pred = nav._room_dir(math.radians(deg)); P['th'] = math.radians(deg); real = cc(nav.S['scan'], True, 0.0, nav.side_margin)
            if min(pred, real) < 2.0:      # 가까운 장애물이 있는 방향만(빈 방향은 가상 스캔의 5 m 고리라 의미 없음)
                worst = max(worst, abs(pred - real))
            elif (pred < 2.0) != (real < 2.0):
                worst = 9.9
        builtins.print = quiet
        check('돌기 전 미리 본 여유 = 돈 뒤 여유', worst <= 0.03, f'최대 차이 {worst * 100:.1f} cm(7방향)')
        builtins.print = lambda *a, **k: None
        # 9. 바로 옆 미확인 물체(10-10: 라이다는 13.5 cm보다 가까운 것을 0.08 m로 읽어 자기 필터가 지움) — 원시 스캔의 기준 밖 방향에
        #    0.08 m 반사가 있으면 한 걸음도·한 번도 돌지 않고 실패 보고. 기준 안 방향(자기 구조물)만 있으면 평소대로
        import collections
        def raw_with(deg_lo, deg_hi):
            sc = Scan()
            for i in range(455):
                d = math.degrees(sc.angle_min + i * sc.angle_increment) % 360
                if deg_lo <= d <= deg_hi:
                    sc.ranges[i] = 0.082
            return collections.deque([sc] * 5, maxlen=5)
        base = {52, 54, 56, 58, 60, 62, 64, 66, 68, 168, 170, 172, 186, 188, 236, 238, 240, 242, 244, 292, 294, 296, 298, 300, 302, 304, 306}
        nav, P = make(); nav._raw = raw_with(98, 118); nav._near_base = base; r = nav.go((0.0, 1.0, None))
        kinds = [x['kind'] for x in nav.log]
        builtins.print = quiet
        check('옆에 붙은 물체(원시 0.08 m, 98~118°) → 움직이지 않음', not r['ok'] and P['turned'] == 0 and 'drive' not in kinds and '미확인' in r['msg'], r['msg'])
        builtins.print = lambda *a, **k: None
        nav, P = make(); nav._raw = raw_with(54, 66); nav._near_base = base; r = nav.go((1.0, 0.0, None))
        builtins.print = quiet
        check('자기 구조물 방향(54~66°)만 가까우면 평소대로', r['ok'], r['msg'])
        builtins.print = lambda *a, **k: None
        # 10. 계획이 안 되는 자리(10-10: SLAM 유령 벽 속)에 들어섰을 때 — 마지막으로 계획된 자리로 되돌아가고, 두 번이면 그 목표는 포기.
        #     그 뒤 다른 목표는 정상(되돌아간 자리에서 계획이 됨). x > 0.55 구역에선 계획이 전부 실패하는 가상 planner
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0)
        straight = nav.plan
        nav.plan = lambda here, goal: (None, '빈 경로') if here[0] > 0.55 else straight(here, goal)
        r = nav.go((2.0, 0.0, None)); kinds = [x['kind'] for x in nav.log]
        r2 = nav.go((0.0, 1.0, None))
        builtins.print = quiet
        check('계획 안 되는 자리 → 되돌아감, 2번이면 포기, 다음 목표는 정상',
              not r['ok'] and kinds.count('retreat') == 2 and '되돌아감' in r['msg'] and r2['ok'],
              f'{r["msg"]} | 되돌아감 {kinds.count("retreat")}회, 다음 목표 {r2["msg"]} {r2["final_err_m"] * 100:.1f} cm')
        builtins.print = lambda *a, **k: None
        # 4. 균등 분할
        nav, P = make(rot_off=0.0, drive_scale=1.0); r = nav.go((1.28, 0.0, None))
        builtins.print = quiet
        check('1.28 m 균등 분할', r['ok'] and abs(P['x'] - 1.28) <= 0.03,
              f'{(P["x"] - 1.28) * 100:+.1f} cm, 걸음 {[x["arg"] for x in nav.log if x["kind"] == "drive"]}')
        builtins.print = lambda *a, **k: None
        # 6b. 앞 여유 0.55 m(병 표면이 통로 가장자리 0.66 m 앞) + 걸음 0.4 → 걸음을 줄여 간다(10-09 시뮬에서 31번 막혔던 상황)
        #     가짜 플래너는 직선이라 줄인 걸음 뒤에는 정말 막힌다 — 확인할 것: 줄인 걸음이 실행되고, 병 앞 0.24 m 안으로는 안 간다
        #     (병 중심 y −0.17: 표면이 통로 폭 ±0.16 안쪽에 확실히 걸리게 — −0.19는 가상 스캔 각도 양자화로 가장자리에 걸려 멀어지면 사라졌다)
        w = bottle(0.693, -0.17)
        nav, P = make(world=w); r = nav.go((0.8, 0.0, None))
        kinds = [x['kind'] for x in nav.log]
        gap = min(math.hypot(wx - P['x'], wy - P['y']) for wx, wy in w)
        builtins.print = quiet
        check('앞 여유 부족 → 걸음을 줄임', 'shorten' in kinds and 0.25 <= P['x'] <= 0.31 and not r['ok'] and gap > 0.36,
              f'{kinds[:4]}, 이동 {P["x"] * 100:.0f} cm, 병까지 중심 {gap * 100:.0f} cm, {r["msg"]}')
        builtins.print = lambda *a, **k: None
        # 5. 취소(액션 서버가 cancel을 세움) — 움직이지 않고 '취소'로 끝난다. ⚠️ 10-09: cancel을 __init__에서만 만들어
        #   이 가상 시험(객체를 __init__ 없이 만듦)이 통째로 죽었는데, 실패한 채 커밋됐다
        nav, P = make(rot_off=0.0); nav.cancel = True; r = nav.go((1.0, 0.0, None))
        builtins.print = quiet
        check('취소 요청 → 움직이지 않고 끝', not r['ok'] and r['msg'] == '취소' and not [x for x in nav.log if x['kind'] == 'drive'],
              f'{r["msg"]}, 직진 {len([x for x in nav.log if x["kind"] == "drive"])}회')
    finally:
        builtins.print = quiet
    print(f'\n4층 주행기 가상 시험: {sum(res)}/{len(res)} 통과')
    return 0 if all(res) else 1


if __name__ == '__main__':
    sys.exit(main())
