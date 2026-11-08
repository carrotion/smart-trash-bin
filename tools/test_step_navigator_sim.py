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
        builtins.print = lambda *a, **k: None
        # 11. 금지 구역(10-10, tools/keepout.py) — 구역 선은 가상의 벽: 실제 장애물이 하나도 없는 빈 곳에서 선만으로 멈춰야 한다
        import keepout
        box = lambda x0, y0, x1, y1, name='': dict(id=1, name=name, poly=[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        #  11a. 정면을 가로막은 구역(가짜 planner는 직선) → 선을 넘지 않고, 차체 앞끝이 선 앞 0.25 m(앞 여유)에서 멈춤
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); nav.set_zones([box(0.8, -1.0, 1.4, 1.0, '소파')])
        r = nav.go((2.0, 0.0, None)); front = P['x'] + 0.11
        builtins.print = quiet
        check('금지 구역이 정면 → 선 앞에서 멈춤', not r['ok'] and 0.2 <= P['x'] and front <= 0.8 - 0.24, f'차체 앞끝 {front:.2f} m(선 0.80 m), {r["msg"]}')
        builtins.print = lambda *a, **k: None
        #  11b. 목표가 구역 안 → 움직이지 않고 이유를 알림
        nav, P = make(); nav.set_zones([box(0.8, -1.0, 1.4, 1.0, '소파')]); r = nav.go((1.0, 0.0, None))
        builtins.print = quiet
        check('목표가 금지 구역 안 → 출발 안 함', not r['ok'] and not nav.log and '금지 구역' in r['msg'], r['msg'])
        builtins.print = lambda *a, **k: None
        #  11c. 로봇이 구역 안(구역을 로봇 위에 그렸을 때) → 움직이지 않음
        nav, P = make(); nav.set_zones([box(-0.5, -0.5, 0.5, 0.5)]); r = nav.go((2.0, 0.0, None))
        builtins.print = quiet
        check('로봇이 금지 구역 안 → 움직이지 않음', not r['ok'] and not nav.log and '안에 있음' in r['msg'], r['msg'])
        builtins.print = lambda *a, **k: None
        #  11d. 구역 모서리가 오른쪽 옆면 7 cm(회전 반경 안) + 90° 왼쪽 목표 → 병 때와 같이 떨어진 뒤 회전, 돌 때 구역까지 > 0.20 m
        zs = [box(-0.02, -0.28, 0.08, -0.18)]
        nav, P = make(); nav.set_zones(zs); gaps = []; t0_ = nav.turner.turn_by
        nav.turner.turn_by = lambda d: (gaps.append(keepout.nearest(P['x'], P['y'], zs)[0]), t0_(d))[1]
        r = nav.go((0.0, 1.0, None)); kinds = [x['kind'] for x in nav.log]
        builtins.print = quiet
        check('금지 구역 모서리가 옆 → 떨어진 뒤 회전', r['ok'] and 'rot_clear' in kinds and min(gaps) > 0.20,
              f'돌 때 구역까지 {min(gaps) * 100:.1f} cm, {kinds[:4]}')
        builtins.print = lambda *a, **k: None
        #  11e. 구역이 없으면 스캔을 건드리지 않는다(같은 객체), 구역을 지우면 원래대로
        nav, P = make(); same0 = nav._scan() is not None and not nav._zone_pts
        nav.set_zones([box(0.8, -1.0, 1.4, 1.0)]); r1 = nav.go((2.0, 0.0, None)); nav.set_zones([]); r2 = nav.go((2.0, 0.0, None))
        builtins.print = quiet
        check('구역을 지우면 다시 지나감', same0 and not r1['ok'] and r2['ok'], f'구역 있을 때: {r1["msg"]} / 지운 뒤: {r2["msg"]}')
        builtins.print = lambda *a, **k: None
        #  11g. 구역 서버의 말을 못 들었으면(실물에서 서버가 안 떴거나 아직 안 도착) 움직이지 않는다 — "모르면 없는 셈"이 아니다.
        #       가는 도중에 목표를 덮는 구역이 생기면 그 자리에서 멈춘다
        nav, P = make(); nav._zones_seen = False; r = nav.go((1.0, 0.0, None))
        nav2, P2 = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); d0 = nav2.driver.drive_by
        nav2.driver.drive_by = lambda d: (d0(d), nav2.set_zones([box(1.6, -1.0, 2.6, 1.0)]))[0]      # 첫 걸음 뒤 구역이 생김
        r2 = nav2.go((2.0, 0.0, None))
        builtins.print = quiet
        check('구역 정보 없음 → 움직이지 않음 / 도중에 생긴 구역 → 멈춤', not r['ok'] and not nav.log and '받지 못함' in r['msg']
              and not r2['ok'] and '금지 구역' in r2['msg'] and P2['x'] < 0.5, f'{r["msg"]} / {r2["msg"]}, 이동 {P2["x"] * 100:.0f} cm')
        builtins.print = lambda *a, **k: None
        #  11f. 경로 계획용 격자: 구역 안과 차체 반폭 안은 100(통행 불가), 팽창 반경까지는 1~99, 그 밖은 0
        m, ox, oy = keepout.build_mask([box(1.0, 1.0, 2.0, 1.6)])
        at = lambda x, y: int(m[int((y - oy) / 0.05), int((x - ox) / 0.05)])
        vals = [at(1.5, 1.3), at(1.5, 0.93), at(1.5, 0.80), at(1.5, 0.55), at(2.07, 1.67), at(-5.0, 3.0)]
        ok = vals[0] == 100 and vals[1] == 100 and 1 <= vals[2] <= 99 and vals[3] == 0 and vals[4] == 100 and vals[5] == 0
        try:
            keepout.clean_zones([dict(poly=[[0, 0], [0.05, 0], [0, 0.05]])]); small = False
        except ValueError:
            small = True
        try:
            keepout.clean_zones([dict(poly=[[0, 0], [50, 0], [0, 50]])]); far = False
        except ValueError:
            far = True
        builtins.print = quiet
        check('금지 구역 격자 값·입력 검사', ok and small and far, f'안 {vals[0]}, 선 밖 7 cm {vals[1]}, 20 cm {vals[2]}, 45 cm {vals[3]}, 모서리 밖 {vals[4]} | 너무 작은 구역 거부 {small}, 범위 밖 거부 {far}')
        builtins.print = lambda *a, **k: None
        #  12. 출발 전 자리 검사(tools/pose_check.py): 4 × 3 m 방. 스캔은 실제 자리에서 본 벽, 주행기가 믿는 자리(map_pose)는 따로 준다.
        #      맞으면 간다 / 0.5·0.4 m 어긋나 있으면 움직이지 않고 거절 / 한 번만 안 맞고(지나가는 사람) 다시 보니 맞으면 간다 /
        #      거절을 끄면(pose_check_enforce=False) 가되 기록에는 남는다
        import numpy as np
        wall = [(x / 100, y) for x in range(-100, 301) for y in (-1.5, 1.5)] + [(x, y / 100) for y in range(-150, 151) for x in (-1.0, 3.0)]
        g = np.full((100, 120), -1, np.int8); g[20:81, 20:101] = 0                     # 원점 (−2, −2.5), 칸 5 cm
        g[20, 20:101] = g[80, 20:101] = 100; g[20:81, 20] = g[20:81, 100] = 100
        fmap = types.SimpleNamespace(data=g, info=types.SimpleNamespace(width=120, height=100, resolution=0.05,
                                     origin=types.SimpleNamespace(position=types.SimpleNamespace(x=-2.0, y=-2.5))))

        def room(off=(0.0, 0.0)):
            nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0, world=wall); nav._map_msg = fmap
            nav.map_pose = lambda: (P['x'] + off[0], P['y'] + off[1], P['th'])
            return nav, P
        nav, P = room(); r = nav.go((1.0, 0.0, None)); pc = [e for e in nav.log if e['kind'] == 'pose_check']
        nav2, P2 = room((0.5, 0.4)); r2 = nav2.go((1.5, 0.4, None)); pc2 = [e for e in nav2.log if e['kind'] == 'pose_check']
        nav3, P3 = room(); real = nav3._pose_check; calls = []
        nav3._pose_check = lambda here: (calls.append(1), dict(real(here), ok=False, why='시험') if len(calls) == 1 else real(here))[1]
        r3 = nav3.go((1.0, 0.0, None))
        nav4, P4 = room((0.5, 0.4)); nav4.pose_check_enforce = False; r4 = nav4.go((1.5, 0.4, None))
        builtins.print = quiet
        check('출발 전 자리 검사: 맞으면 감 / 어긋나면 거절·이동 0 / 한 번만 안 맞으면 감 / 끄면 기록만',
              r['ok'] and pc and all(e['ok'] and e['ratio'] > 0.95 for e in pc)
              and not r2['ok'] and '안 맞음' in r2['msg'] and math.hypot(P2['x'], P2['y']) < 1e-9 and len(pc2) == 1 and pc2[0]['ratio'] < 0.8
              and r3['ok'] and len(calls) >= 2
              and r4['ok'] and not [e for e in nav4.log if e['kind'] == 'pose_check'][0]['ok'],
              f'맞는 자리 비율 {min(e["ratio"] for e in pc):.2f}({len(pc)}회) | 어긋난 자리 비율 {pc2[0]["ratio"]:.2f} → {r2["msg"][:28]}…, 이동 {math.hypot(P2["x"], P2["y"]) * 100:.0f} cm | 재확인 뒤 {r3["msg"]} | 끔 {r4["msg"]}')
        builtins.print = lambda *a, **k: None
        #  13. 왕복 끊기(11-01 닫힌 문 옆 틈): 위치에 따라 경로가 앞뒤로 뒤집히는 가상 planner. 2번째 큰 회전 전에 멈춘다.
        #      막다른 길에서 한 번 돌아서는 것은 그대로 도착해야 한다
        def line(here, pts):
            out = [(here[0], here[1])]
            for x, y, *_ in pts:
                x0, y0 = out[-1]; n = max(1, int(math.hypot(x - x0, y - y0) / 0.05))
                out += [(x0 + (x - x0) * k / n, y0 + (y - y0) * k / n) for k in range(1, n + 1)]
            return out
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0)
        nav.plan = lambda here, goal: (line(here, [goal]) if here[0] < 0.5 else line(here, [(-6.0, 0.0), (-6.0, 4.0), goal]), '')
        r = nav.go((2.0, 0.0, None)); big = [e for e in nav.log if e['kind'] == 'turn' and abs(e['arg']) > 150]
        kinds = [e['kind'] for e in nav.log]
        nav2, P2 = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); seen = []
        def dead_end(here, goal):
            if here[0] > 0.9:
                seen.append(1)
            return (line(here, [goal]) if seen else line(here, [(3.0, 0.0), (3.0, 3.0), goal]), '')
        nav2.plan = dead_end
        r2 = nav2.go((-1.0, 0.0, None))
        nav3, P3 = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); nav3.flip_max = 99
        nav3.plan = nav.plan; r3 = nav3.go((2.0, 0.0, None), max_steps=30)
        big3 = [e for e in nav3.log if e['kind'] == 'turn' and abs(e['arg']) > 150]
        builtins.print = quiet
        check('경로 왕복 → 2번째 큰 회전 전에 멈춤 / 막다른 길 한 번은 도착 / 끊기가 없으면 왕복 재현',
              not r['ok'] and '왕복' in r['msg'] and len(big) == 1 and 'flip_stop' in kinds and r2['ok'] and not r3['ok'] and len(big3) >= 5,
              f'{r["msg"]} | 큰 회전 {len(big)}번 | 막다른 길: {r2["msg"]} | 끊기 없을 때 큰 회전 {len(big3)}번')
        builtins.print = lambda *a, **k: None
        #  14. 시간 제한은 경로 길이에 맞춘다. 시계를 걸음마다 8초씩 흘린다(실측 약 20초/m). 15 m는 도착(예전 180초면 약 9 m에서 끊김),
        #      제자리걸음이면 짧은 경로는 180초, 긴 경로도 600초에서 끊는다
        import step_navigator as sn
        real_time = sn.time; clock = [1000.0]; sn.time = types.SimpleNamespace(time=lambda: clock[0], sleep=lambda t: None)
        try:
            def timed(goal, moves):
                nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); d0 = nav.driver.drive_by
                def drive(d):
                    clock[0] += 8.0
                    return d0(d) if moves else dict(ok=True, msg='', err_m=0.0, fixes=0, secs=8.0)
                nav.driver.drive_by = drive
                return nav.go(goal)
            r15 = timed((15.0, 0.0, None), True); rs = timed((2.0, 0.0, None), False); rl = timed((30.0, 0.0, None), False)
        finally:
            sn.time = real_time
        builtins.print = quiet
        check('시간 제한: 15 m 도착 / 제자리걸음 2 m는 180초, 30 m는 600초에 끊김',
              r15['ok'] and r15['secs'] > 180 and rs['msg'] == '시간 초과' and 180 <= rs['secs'] <= 196 and rl['msg'] == '시간 초과' and 600 <= rl['secs'] <= 616,
              f'15 m: {r15["msg"]} {r15["secs"]:.0f}초 | 2 m 제자리: {rs["secs"]:.0f}초 | 30 m 제자리: {rl["secs"]:.0f}초')
        builtins.print = lambda *a, **k: None
        builtins.print = lambda *a, **k: None
        #  15. 의자 밑(11-01 세 번째 탐색): 들어가다 다리가 옆면에 붙어 원시 스캔 260~268°에 0.08 — 방금 들어온 직진을 곧게 되돌려 나온다.
        #      (C의 뒤 벽은 차체 뒤 19 cm: 후진 여유 검사는 원래 차체 뒤 10 cm 안을 부착물 자리로 보지 않는다 — 시험 3과 같은 설계)
        #      다리 (0.40, −0.18) 근처(중심 20 cm 안)에서만 원시 반사가 생기는 가상 라이다
        class RawNear:
            def __init__(self, P, leg, deg=(260, 268)):
                self.P, self.leg, self.deg = P, leg, deg
            def __len__(self):
                return 5
            def __iter__(self):
                sc = Scan()
                if math.hypot(self.P['x'] - self.leg[0], self.P['y'] - self.leg[1]) < 0.20:
                    for i in range(455):
                        d = math.degrees(sc.angle_min + i * sc.angle_increment) % 360
                        if self.deg[0] <= d <= self.deg[1]:
                            sc.ranges[i] = 0.082
                return iter([sc] * 5)
        def chair(plan_detour=False, deg=(260, 268), rear_wall=False):
            nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0, world=[] if rear_wall else None); nav._near_base = base; nav._raw = RawNear(P, (0.40, -0.18), deg)
            straight = nav.plan
            if plan_detour:   # 빠져나온 뒤 planner가 다리를 피해 왼쪽으로 돈다(코스트맵에 다리가 찍힘)
                nav.plan = lambda here, goal: ((line(here, [(here[0], 0.35), (1.5, 0.35), goal]), '') if any(e['kind'] == 'back_out' for e in nav.log) and here[1] < 0.3 else straight(here, goal))
            if rear_wall:     # 다리를 만난 순간 뒤가 막힌다
                nu0 = nav._near_unknown
                def nu():
                    r = nu0()
                    if r and not nav.S.world:
                        nav.S.world.extend((P['x'] - 0.30, P['y'] + k / 100) for k in range(-15, 16))
                    return r
                nav._near_unknown = nu
            r = nav.go((1.5, 0.0, None)); kinds = [e['kind'] for e in nav.log]
            bo = [e for e in nav.log if e['kind'] == 'back_out']
            i = kinds.index('near_unknown') if 'near_unknown' in kinds else len(kinds)
            return r, kinds, bo, 'turn' in kinds[i:kinds.index('back_out')] if bo else False
        rA, kA, boA, turnA = chair(); rB, kB, boB, _ = chair(plan_detour=True); rC, kC, boC, _ = chair(rear_wall=True); rD, kD, boD, _ = chair(deg=(176, 184))
        builtins.print = quiet
        check('의자 밑: 들어온 직진만큼 곧게 후진(회전 없이, 1번) / 빠진 뒤 돌아가는 길이면 도착 / 뒤가 막히거나 물체가 뒤쪽이면 후진 안 함',
              len(boA) == 1 and not turnA and 0.03 < -boA[0]['arg'] <= 0.30 and not rA['ok'] and '미확인' in rA['msg']
              and len(boB) == 1 and rB['ok'] and not boC and '미확인' in rC['msg'] and not boD,
              f'A 후진 {[e["arg"] for e in boA]}, {rA["msg"][:16]} | B {rB["msg"]} | C 후진 {len(boC)}번, {rC["msg"][:16]} | D 후진 {len(boD)}번')
        builtins.print = lambda *a, **k: None
        #  16. 벽과 의자 다리 사이(11-07 다섯 번째 탐색): 왼쪽 벽이 차체 옆(중심 20 cm)을 x −0.2~0.45로 따라 이어지고 오른쪽 다리가 옆면 4 cm.
        #      뒤쪽 목표라 돌아야 한다 → 곧게 뒤로 빠져 돌 자리를 찾아(25 cm 안에는 없음) 돌고 도착 / 뒤도 막혔으면 움직이지 않고 실패
        def trap(rear_wall):
            w = [(x / 100, 0.20) for x in range(-20, 46, 2)] + bottle(0.0, -0.165, 0.015)
            if rear_wall:      # 앞뒤 다 막힘(뒤만 막으면 앞으로 60 cm 나가 돌 자리를 찾는다 — 그것도 옳은 판단)
                w += [(-0.33, y / 100) for y in range(-30, 31, 2)] + [(0.50, y / 100) for y in range(-30, 31, 2)]
            nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0, world=w); nav.last_fwd = 0.31
            r = nav.go((-1.5, 0.05, None)); return r, [e for e in nav.log], P
        rA, logA, PA = trap(False); rB, logB, PB = trap(True)
        mvA = [e['arg'] for e in logA if e['kind'] == 'rot_clear']
        moved_B = max(abs(e['arg']) for e in logB if e['kind'] in ('rot_clear', 'rot_retrace', 'drive')) if any(e['kind'] in ('rot_clear', 'rot_retrace', 'drive') for e in logB) else 0
        builtins.print = quiet
        check('벽과 다리 사이: 곧게 뒤로 빠져(25 cm 넘게) 돌고 도착 / 앞뒤 다 막히면 회전 반경 막힘으로 실패',
              rA['ok'] and mvA and mvA[0] < -0.25 and not rB['ok'] and '회전 반경' in rB['msg'] and min(P['turn_clear'] or [9]) > 0.0,
              f'A {rA["msg"]}, 뒤로 {mvA} | B {rB["msg"][:20]}, 움직인 최대 {moved_B}')
        builtins.print = lambda *a, **k: None
        #  17. 모터 보드 엔코더가 끊기면(11-07 사고) 걸음을 내딛지 않는다 / 신선하면 평소대로
        import time as _t
        nav, P = make(); nav._enc_ts = _t.monotonic() - 2.0; r = nav.go((1.0, 0.0, None))
        nav2, P2 = make(); nav2._enc_ts = _t.monotonic() + 3600; r2 = nav2.go((1.0, 0.0, None))
        builtins.print = quiet
        check('엔코더 2초 끊김 → 움직이지 않고 보드 응답 없음 / 신선하면 도착',
              not r['ok'] and '모터 보드 응답 없음' in r['msg'] and not nav.log and P['x'] == 0.0 and r2['ok'], f'{r["msg"][:22]} | {r2["msg"]}')
        builtins.print = lambda *a, **k: None
        #  18. 좁은 돌 자리(11-07 일곱 번째 탐색, 실제 스캔 176점 tools/test_data/trap7_pts.json): 앞 물체 중심 13.5 cm, 뒤 여유 19 cm.
        #      넓은 반경(0.24)으로는 돌 자리가 61 cm 뒤(불가) → 원래 반경(0.22)으로 다시 찾아 9~11 cm 뒤에서 돈다
        import json as _json
        w = [tuple(p) for p in _json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data', 'trap7_pts.json')))]
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0, world=w)
        r = nav._turn(0, math.radians(90), 'turn')           # 점은 이미 차체(base_link) 좌표
        mv = [e['arg'] for e in nav.log if e['kind'] == 'rot_clear']; rn = [e['arg'] for e in nav.log if e['kind'] == 'rot_near']
        builtins.print = quiet
        check('좁은 돌 자리(실제 스캔): 넓은 반경에 자리 없으면 원래 반경으로 약 10 cm 물러나 돈다',
              r.get('ok') and bool(mv) and -0.12 <= mv[0] <= -0.08 and rn and rn[0]['radius'] == 0.22, f'물러남 {mv}, 반경 {rn[0]["radius"] if rn else None}, {r}')
        builtins.print = lambda *a, **k: None
        builtins.print = lambda *a, **k: None
        #  19. 왕복 끊기 목표 근처 제외(11-08, 남은 경로 0.4 m 안)를 넣은 뒤에도: 가까운 목표엔 정상 도착 / 먼 왕복은 그대로 끊는다.
        #      목표 근처에서 실제로 지나쳐 돌아서는 장면은 가짜 planner로 재현이 어려워 실물에서 확인한다(여기선 회귀만)
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); nav.plan = lambda here, goal: (line(here, [goal]), '')
        r = nav.go((0.3, 0.0, None)); fl = [e for e in nav.log if e['kind'] == 'flip_stop']
        nav3, P3 = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0)
        nav3.plan = lambda here, goal: (line(here, [goal]) if here[0] < 0.5 else line(here, [(-6.0, 0.0), (-6.0, 4.0), goal]), '')
        r3 = nav3.go((2.0, 0.0, None))
        builtins.print = quiet
        check('왕복 끊기 목표 근처 제외 뒤: 가까운 목표 정상 도착 / 먼 왕복은 그대로 끊음',
              r['ok'] and not fl and not r3['ok'] and '왕복' in r3['msg'], f'가까운 목표: {r["msg"]} | 먼 왕복: {r3["msg"][:24]}')
        builtins.print = lambda *a, **k: None
        builtins.print = lambda *a, **k: None
        #  20. 빈 경로면 목표를 주변으로 옮겨 다시 묻는다(11-08, 덜 그려진 경계 근처 목표에서 NavFn 실패) — 원래 목표와 그 15 cm 안은 경로 없음,
        #      더 바깥은 경로 있음 → 0.2 m 옮겨 도착 / 옮기기를 끄면(goal_shift_max 0) 예전처럼 빈 경로로 실패
        G0 = (1.5, 0.0)
        def picky(here, goal):
            if math.hypot(goal[0] - G0[0], goal[1] - G0[1]) < 0.15:
                return None, '빈 경로'
            return line(here, [goal]), ''
        nav, P = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); nav.plan = picky
        r = nav.go((G0[0], G0[1], None)); gs = [e['arg'] for e in nav.log if e['kind'] == 'goal_shift']
        nav2, P2 = make(rot_off=0.0, turn_scale=1.0, drive_scale=1.0); nav2.plan = picky; nav2.goal_shift_max = 0.0
        r2 = nav2.go((G0[0], G0[1], None))
        builtins.print = quiet
        check('빈 경로면 목표를 0.2 m 옮겨 도착 / 끄면 예전처럼 실패',
              r['ok'] and '옮김' in r['msg'] and gs and gs[0]['d'] == 0.2 and math.hypot(P['x'] - G0[0], P['y'] - G0[1]) < 0.26
              and not r2['ok'] and '빈 경로' in r2['msg'], f'{r["msg"]} {gs[:1]} | 끔: {r2["msg"][:20]}')
    finally:
        builtins.print = quiet
    print(f'\n4층 주행기 가상 시험: {sum(res)}/{len(res)} 통과')
    return 0 if all(res) else 1


if __name__ == '__main__':
    sys.exit(main())
