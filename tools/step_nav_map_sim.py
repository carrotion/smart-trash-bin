#!/usr/bin/env python3
"""4층 주행기 지도 가상 시험 — **실제 집 정답 지도(our_home) + 실제 planner_server**, 움직임만 가상(로봇·Stage 없이, 수 초).

test_step_navigator_sim(직선 경로·장난감 장애물)과 Stage 시뮬(10분 단위) 사이를 메운다: 좁은 문틈 같은 **지도 위 특정 상황**을
그 자리에서 바로 재현한다. 도메인 42에 map_server + planner_server(전역 팽창은 tools/inflation_probe 틀, 0.35)만 띄운다.
가상 차체는 test_step_navigator_sim.make()와 같다(회전 중심 뒤 4 cm, 회전 1.02배·직진 1.01배). 스캔은 지도 점유 칸을
현재 자세에서 본 것(각도 칸마다 가장 가까운 점). 충돌 판정: 직진·회전 경로를 잘게 나눠 차체 사각형(±0.11 m) 안에 점유 칸이 들어오는지.

사례(월드 좌표, our_home.world 로봇 시작 (−2.7, 3.43)이 SLAM 지도 원점):
  wallstart: 10-10 Stage의 '빈 경로' 연쇄 — 출발 칸이 벽에 닿는 거리로 표시된 자리에서 출발(가까운 빈 점에서 다시 계획하는 복구 확인)
  door  : 10-09 Stage 2차의 (4.8, −1.2) 막힘 — 폭 0.40~0.45 m 문틈을 벽 0.2 m 아래 오른쪽 문설주 밑에서 비스듬히 진입
          (⚠️ 이 문은 실제 77 cm다 — 08-23 지도가 절반 폭으로 그려져 있었음. 좁은 틈 통과 시험으로 narrow 지도에 남김)

    python3 tools/step_nav_map_sim.py            # 모든 사례
    python3 tools/step_nav_map_sim.py --x 1.83 --y 0.79 --yaw 130 --gx 2.19 --gy 2.27
"""
import argparse, math, os, subprocess, sys, time

os.environ['ROS_DOMAIN_ID'] = '42'
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from test_step_navigator_sim import make

CASES = {'door': dict(start=(1.83, 0.79, 130.0), goal=(2.19, 2.27)),
         # 10-10 Stage: 문 안에서 문설주 0.15 m 옆에 섰더니 planner '빈 경로' → 이후 목표 35개 연속 실패. NavFn은 출발 칸이
         #   벽에 닿는 거리(중심 0.11 m 안에 벽 칸)면 경로를 못 만든다. 문설주 0.07 m(지도상)에서 출발해 그 상황을 만든다
         #   (실제 문 폭 지도에서 — 좁은 문 지도에선 출발 자세 자체가 반대쪽 문설주와도 너무 가까워 빠져나올 자리가 없다)
         'wallstart': dict(map='real', start=(2.14, 1.06, 80.0), goal=(0.46, -1.71))}
RES, OX, OY = 0.05, -7.45, -4.925
HALF = 0.11


MAPS = {'real': 'our_home.png',                   # 문 폭 실제값(A·B 0.75, C 0.85 m — 10-10 사용자 실측 77/77/87 cm)
        'narrow': 'our_home_narrowdoors.png'}     # 08-23 원본: 문 세 곳이 0.45 m로 그려짐 → 좁은 틈 통과 시험용으로 남김


def map_path(name):
    return os.path.abspath(os.path.join(HERE, '../robot/sim_worlds/bitmaps', MAPS[name]))


def load_world(name):
    im = np.array(Image.open(map_path(name)))[::-1, :]
    jj, ii = np.nonzero(im < 128)
    return np.stack([OX + (ii + 0.5) * RES, OY + (jj + 0.5) * RES], 1)


def others_running():
    """이미 떠 있는 Nav2 프로세스(실물 시험 스택·다른 시뮬)."""
    out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout.splitlines()
    return [ln.strip()[:90] for ln in out if any(f'/{n} ' in ln + ' ' for n in ('planner_server', 'map_server', 'controller_server', 'async_slam_toolbox_node'))]


def start_planner(radius='0.35', map_name='real'):
    # ⚠️ 10-10: 실물 시험 스택이 켜진 채 이 도구를 돌렸더니, 끝날 때 이름으로 정리하며 **실물 스택의 planner_server·
    #   lifecycle_manager까지 죽였다**(로봇 정지 중이라 무사). → ① 다른 Nav2가 떠 있으면 시작하지 않는다 ② 자기가 띄운
    #   프로세스 묶음(세션)만 죽인다
    run = others_running()
    if run:
        raise SystemExit('❌ 이미 Nav2 프로세스가 떠 있다 — 실물 시험 스택/다른 시뮬을 먼저 내릴 것:\n   ' + '\n   '.join(run[:4]))
    d = os.path.join(HERE, 'inflation_probe'); p = f'/tmp/step_nav_map_sim_{radius}.yaml'
    my = f'/tmp/step_nav_map_sim_{map_name}.yaml'
    open(my, 'w').write(open(os.path.join(d, 'map.yaml')).read().replace('image: ../../robot/sim_worlds/bitmaps/our_home.png', f'image: {map_path(map_name)}'))
    s = open(os.path.join(d, 'params.template.yaml')).read().replace('RADIUS', radius).replace('yaml_filename: map.yaml', f'yaml_filename: {my}')
    open(p, 'w').write(s)
    env = dict(os.environ); procs = []
    for cmd in (['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'base_link'],
                ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '--params-file', p],
                ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', p],
                ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', '__node:=lifecycle_manager_infl', '--params-file', p]):
        procs.append(subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
    return procs


def stop_planner(procs):
    # ros2 run은 파이썬 실행기라 그 PID만 죽이면 노드가 남는다(09-27) → 세션(프로세스 묶음)째로. 이름으로 죽이지 않는다(위 ⚠️)
    import signal
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for p in procs:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(1.0)


def run_case(node, ac, world, start, goal, quiet=False):
    near = world[np.hypot(world[:, 0] - start[0], world[:, 1] - start[1]) < 6.0]
    nav, P = make(world=[tuple(p) for p in near])
    P['x'], P['y'], P['th'] = start[0], start[1], math.radians(start[2])
    hits = {'n': 0, 'min_gap': 9.9}; track = [(P['x'], P['y'])]

    def body_check():
        c, s = math.cos(P['th']), math.sin(P['th'])
        dx, dy = near[:, 0] - P['x'], near[:, 1] - P['y']
        bx, by = c * dx + s * dy, -s * dx + c * dy
        gap = np.maximum(np.abs(bx), np.abs(by)) - HALF       # 차체 사각형까지(음수 = 안쪽)
        hits['min_gap'] = min(hits['min_gap'], float(gap.min()))
        hits['n'] += int((gap < -0.025).sum() > 0)            # 칸 반쪽(2.5 cm) 넘게 파고들면 충돌

    def ps(x, y):
        m = PoseStamped(); m.header.frame_id = 'map'; m.pose.position.x, m.pose.position.y = float(x), float(y); m.pose.orientation.w = 1.0
        return m

    def plan(here, g):
        req = ComputePathToPose.Goal(); req.start = ps(here[0], here[1]); req.goal = ps(g[0], g[1]); req.use_start = True
        f = ac.send_goal_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
        if not f.done() or not f.result().accepted:
            return None, '경로 요청 거부'
        r = f.result().get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=20)
        path = [(q.pose.position.x, q.pose.position.y) for q in r.result().result.path.poses] if r.done() else []
        return (path if len(path) >= 2 else None), ('' if len(path) >= 2 else '빈 경로')
    nav.plan = plan
    turn0, drive0 = nav.turner.turn_by, nav.driver.drive_by

    def turn_by(d):
        n = max(1, int(abs(d) / math.radians(3))); r = None
        for _ in range(n):
            r = turn0(d / n); body_check()
        return r

    def drive_by(d):
        n = max(1, int(abs(d) / 0.01)); r = None
        for _ in range(n):
            r = drive0(d / n); body_check()
        track.append((P['x'], P['y']))
        return r
    nav.turner.turn_by, nav.driver.drive_by = turn_by, drive_by
    import builtins
    real_print = builtins.print
    if quiet:
        builtins.print = lambda *a, **k: None
    try:
        r = nav.go((goal[0], goal[1], None), max_steps=40, timeout=60.0)
    finally:
        builtins.print = real_print
    return r, nav, P, hits, track


def random_cases(world, n, seed=11, clear=0.26, goal_clear=0.26):
    """집 안(가장 큰 연결 영역)에서 장애물과 clear 이상 떨어진 칸끼리 무작위 n쌍(직선 1.5~6 m), 출발 방향도 무작위.
    goal_clear: 목표의 장애물 여유. ⚠️ 주행기는 앞 장애물 0.2 m 안으로 직진하지 않으므로(drive clear_fn) 벽에서 약 0.31 m 안의
    목표는 그 방향으론 5 cm 안에 못 닿는다(설계상) — 주행기 성능만 보려면 0.40을 준다."""
    import random
    from collections import deque
    W, H = 298, 197
    occ = np.zeros((H, W), bool)
    occ[((world[:, 1] - OY) / RES).astype(int), ((world[:, 0] - OX) / RES).astype(int)] = True
    oj, oi = np.nonzero(occ); O = np.stack([oi, oj], 1).astype(np.float32); dist = np.zeros((H, W), np.float32)
    fj, fi = np.nonzero(~occ); F = np.stack([fi, fj], 1).astype(np.float32)
    for k in range(0, len(F), 2000):
        dist[fj[k:k + 2000], fi[k:k + 2000]] = np.sqrt(((F[k:k + 2000, None, :] - O[None, :, :]) ** 2).sum(-1)).min(1) * RES
    ok = dist > 0.16; seen = np.zeros_like(ok); best = []
    for sj, si in zip(*np.nonzero(ok)):
        if seen[sj, si]:
            continue
        comp = []; q = deque([(sj, si)]); seen[sj, si] = True
        while q:
            j, i = q.popleft(); comp.append((j, i))
            for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = j + dj, i + di
                if 0 <= a < H and 0 <= b < W and ok[a, b] and not seen[a, b]:
                    seen[a, b] = True; q.append((a, b))
        if len(comp) > len(best):
            best = comp
    cand = [(j, i) for j, i in best if dist[j, i] >= clear]
    rnd = random.Random(seed); out = {}
    while len(out) < n:
        (aj, ai), (bj, bi) = rnd.choice(cand), rnd.choice(cand)
        pa = (OX + (ai + .5) * RES, OY + (aj + .5) * RES); pb = (OX + (bi + .5) * RES, OY + (bj + .5) * RES)
        if 1.5 <= math.dist(pa, pb) <= 6.0 and dist[bj, bi] >= goal_clear:
            out[f'r{len(out):02d}'] = dict(start=(pa[0], pa[1], rnd.uniform(-180, 180)), goal=pb)
    return out


def main():
    ap = argparse.ArgumentParser()
    for k in ('x', 'y', 'yaw', 'gx', 'gy'):
        ap.add_argument('--' + k, type=float)
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--random', type=int, default=0, help='무작위 n쌍 일괄 측정')
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--goal-clear', type=float, default=0.26, help='목표의 장애물 여유(m)')
    ap.add_argument('--map', choices=list(MAPS), default=None, help='기본: 사례만 돌리면 narrow(좁은 문틈 시험), --random/직접 지정이면 real')
    ap.add_argument('--nav-file', help='비교용: 다른 step_navigator.py(예: git show HEAD:… 로 꺼낸 파일)')
    ap.add_argument('--set', nargs='*', default=[], help='StepNavigator 클래스 값 덮어쓰기 예: replan_after_escape=True')
    a = ap.parse_args()
    import test_step_navigator_sim as T
    if a.nav_file:
        import importlib.util
        sp = importlib.util.spec_from_file_location('step_navigator_alt', a.nav_file); mod = importlib.util.module_from_spec(sp); sp.loader.exec_module(mod)
        T.StepNavigator = mod.StepNavigator
    for kv in a.set:
        k, v = kv.split('='); setattr(T.StepNavigator, k, eval(v))
    map_name = a.map or ('narrow' if (a.x is None and not a.random) else 'real')
    world = load_world(map_name)
    cases = CASES if a.x is None else {'custom': dict(start=(a.x, a.y, a.yaw or 0.0), goal=(a.gx, a.gy))}
    if a.random:
        cases = dict(CASES if map_name == 'narrow' else {}, **random_cases(world, a.random, a.seed, goal_clear=a.goal_clear))
    ok_all = True; stat = []; procs = []
    groups = {}
    for name, c in cases.items():
        groups.setdefault(c.get('map', map_name), {})[name] = c
    try:
        rclpy.init(); node = rclpy.create_node('step_nav_map_sim'); ac = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        for gmap, gcases in groups.items():
            print(f'지도: {gmap} ({MAPS[gmap]})', flush=True)
            gworld = world if gmap == map_name else load_world(gmap)
            procs = start_planner(map_name=gmap)
            if not ac.wait_for_server(timeout_sec=25):
                print('❌ planner_server 안 뜸'); return 1
            time.sleep(3.0)
            for name, c in gcases.items():
                r, nav, P, hits, track = run_case(node, ac, gworld, c['start'], c['goal'], quiet=not a.verbose)
                kinds = [e['kind'] for e in nav.log]
                ok = r['ok'] and hits['n'] == 0; ok_all &= ok
                stat.append(dict(name=name, ok=r['ok'], msg=r['msg'], hits=hits['n'], steps=r['steps'], blocked=kinds.count('blocked'),
                                 turns=kinds.count('turn'), shifts=kinds.count('plan_shift'), min_gap=hits['min_gap'], start=c['start'], goal=c['goal']))
                if a.random and not a.verbose:
                    if not ok:
                        print(f'❌ {name}: {r["msg"]} | 충돌 {hits["n"]} | 출발 ({c["start"][0]:.2f},{c["start"][1]:.2f},{c["start"][2]:.0f}°) → 목표 ({c["goal"][0]:.2f},{c["goal"][1]:.2f})', flush=True)
                    continue
                print(f'{"✅" if ok else "❌"} {name}: {r["msg"]} | 남은 {r["final_err_m"] * 100:.1f} cm, 단계 {r["steps"]}, 회전 {kinds.count("turn")}, '
                      f'막힘 {kinds.count("blocked")}, 출발점 옮겨 계획 {kinds.count("plan_shift")}, 충돌 {hits["n"]}, 차체-장애물 최소 {hits["min_gap"] * 100:.1f} cm')
                print('   지나간 점:', ' → '.join(f'({x:.2f},{y:.2f})' for x, y in track))
            stop_planner(procs); procs = []; time.sleep(2.0)
        if a.random:
            n = len(stat); arr = sum(x['ok'] for x in stat); col = sum(x['hits'] > 0 for x in stat)
            print(f'=== {n}건: 도착 {arr} ({arr / n:.0%}), 충돌 있는 건 {col}, 단계 평균 {np.mean([x["steps"] for x in stat]):.1f}, '
                  f'막힘 합 {sum(x["blocked"] for x in stat)}, 회전 합 {sum(x["turns"] for x in stat)}, 출발점 옮겨 계획 {sum(x["shifts"] for x in stat)}, '
                  f'차체-장애물 최소 {min(x["min_gap"] for x in stat) * 100:.1f} cm')
            import json
            json.dump(stat, open(f'/tmp/step_nav_map_sim_{int(time.time())}.json', 'w'), default=float)
    finally:
        stop_planner(procs)
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
