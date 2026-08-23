#!/usr/bin/env python3
"""Remi가 docs/pc_tasks.md에 할당한 Phase 1 — explore_lite vs frontier_exploration_ros2,
고정 파라미터(potential_scale=2.0/gain_scale=2.0/min_frontier_size=0.4,
속도 0.04m/s, progress_checker 여유 30초)로 our_home에서 각 3회씩 순차 비교.

순차 실행 이유: 공정 비교 결과이므로 병렬 실행으로 인한 자원경쟁 오염 가능성을
아예 배제하기 위함(오늘 자원경쟁이 결과를 심하게 왜곡한 전례가 있었다).
"""
import csv
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.expanduser('~/smart-trash-bin/tools'))
import sim_param_sweep as sweep  # noqa: E402

REPO = os.path.expanduser('~/smart-trash-bin')
ROBOT = os.path.join(REPO, 'robot')
SCRATCH = '/tmp/phase1_compare'
os.makedirs(SCRATCH, exist_ok=True)
TIMEOUT_SEC = 200
N_REPEATS = 3
DOMAIN = 43

explore_file = sweep.make_explore_variant(0.4, 'phase1')
nav2_file = sweep.make_nav2_variant(0.04, 30.0, 'phase1')


def run_one(explorer, rep):
    tag = f'{explorer}_r{rep}'
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    cov_path = os.path.join(SCRATCH, f'{tag}_coverage.csv')
    inner_cmd = (
        f"timeout -s INT {TIMEOUT_SEC} ros2 launch smart_trash_bringup "
        f"sim_autonomous_mapping.launch.py world:=our_home explorer:={explorer} "
        f"nav2_params_file:={nav2_file} explore_params_file:={explore_file}"
    )
    full_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {ROBOT}/install/setup.bash && "
        f"xvfb-run -a bash -c '{inner_cmd} & "
        f"LP=$!; sleep 8; "
        f"python3 {REPO}/tools/map_coverage_logger.py --out {cov_path} "
        f"--interval 5 --sim-time --duration {TIMEOUT_SEC + 20} & "
        f"CP=$!; wait $LP; kill -INT $CP 2>/dev/null; wait $CP 2>/dev/null'"
    )
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), PATH='/usr/bin:' + os.environ.get('PATH', ''))
    t0 = time.time()
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(['bash', '-c', full_cmd], stdout=logf, stderr=subprocess.STDOUT,
                                 env=env, preexec_fn=os.setsid)
        # ⚠️ killpg를 정상/타임아웃 무관하게 항상 호출 — TimeoutExpired 경로에만
        # 두면 "정상 종료처럼 보이는데 자식이 안 죽는" 케이스를 놓친다
        # (2026-08-23 밤, Remi 지적 + optuna_search.py에서 실제로 겪음).
        try:
            proc.wait(timeout=TIMEOUT_SEC + 90)
        except subprocess.TimeoutExpired:
            pass
        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
    elapsed = time.time() - t0

    with open(log_path) as f:
        content = f.read()
    goal_switches = content.count('Begin navigating')
    progress_fail = content.count('Failed to make progress')
    collisions = content.count('Collision Ahead')
    completed = ('All frontiers traversed' in content or 'No frontiers found' in content
                 or 'No more frontiers found' in content)

    coverage = 0.0
    if os.path.exists(cov_path):
        last = None
        with open(cov_path) as f:
            for row in csv.DictReader(f):
                last = row
        if last:
            try:
                coverage = float(last['coverage_pct'])
            except (KeyError, ValueError):
                pass

    return {
        'explorer': explorer, 'rep': rep, 'completed': completed,
        'duration_s': round(elapsed, 1), 'goal_switches': goal_switches,
        'progress_fail': progress_fail, 'collisions': collisions,
        'coverage_pct': round(coverage, 1), 'log_path': log_path,
    }


results = []
for explorer in ('explore_lite', 'frontier'):
    for rep in range(N_REPEATS):
        print(f'[{explorer} rep{rep}] 시작...', flush=True)
        r = run_one(explorer, rep)
        results.append(r)
        print(f'  -> {r}', flush=True)

print('\n=== 전체 결과 ===')
for r in results:
    print(r)
