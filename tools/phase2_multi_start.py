#!/usr/bin/env python3
"""Phase 2 — frontier_exploration_ros2 최적 파라미터를 여러 시작 위치에서
반복 검증. explore_lite vs frontier 비교(docs/pc_tasks.md "🏁 최종 결론")에서
frontier가 안정성 면에서 압도적으로 우위였음을 확인했고, 그 최적값이 특정
시작 위치(거실, 원래 Optuna 탐색 기준점)에만 맞춰진 우연이 아닌지 검증한다.

시작 위치는 our_home.png 비트맵에서 0.5m 반경 완전 개방인 지점을 자동 탐색해
집 안 서로 다른 방(거실/침실1/침실2/복도/주방 추정)에서 하나씩 골랐다 —
Stage 좌표계 매핑(v1: 이미지 row0=world +y)은 알려진 원래 시작점(-2.7, 3.43)과
정확히 일치시켜 검증함. world 파일 변형은 robot/src/stage_ros2/world/에
our_home_start_*.world로 저장돼 있음(gitignore된 서브모듈 디렉토리라 커밋
안 됨 — 재현하려면 이 스크립트의 STARTS 딕셔너리와 make_worlds 로직 참고).

사용법:
    python3 tools/phase2_multi_start.py --n-jobs 4 --duration 150
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from optuna_search import (  # noqa: E402
    DOMAIN_LOCK, DOMAIN_CV, acquire_domain, release_domain,
    make_param_variant, run_trial, final_coverage, FRONTIER_BASE, SCRATCH,
)
import optuna_search  # noqa: E402
import threading

FRONTIER_BEST_PARAMS = {
    'weight_distance_wd': 0.37,
    'weight_gain_ws': 0.39,
    'occ_threshold': 68,
    'min_frontier_size_cells': 8,
    'mrtsp_solver': 'greedy',
    'robot_base_frame': 'base_link',
    'frontier_selection_min_distance': 0.35,
    'frontier_candidate_min_goal_distance_m': 0.35,
}

STARTS = {
    'baseline_living': 'our_home',              # 원래 Optuna 탐색 기준점 (거실, -2.7,3.43)
    'living2': 'our_home_start_living2',         # 거실 안쪽 다른 지점
    'bed1': 'our_home_start_bed1',               # 침실1
    'bed2': 'our_home_start_bed2',               # 침실2
    'hallway': 'our_home_start_hallway',         # 왼쪽 복도/안방
    'kitchen': 'our_home_start_kitchen',         # 오른쪽 아래 공간(주방 추정)
}


def worker(jobs, results, duration):
    while True:
        with DOMAIN_LOCK:
            if not jobs:
                return
            name, world = jobs.pop()
        domain = acquire_domain()
        # run_trial은 world를 인자로 안 받으므로 launch 커맨드에 world를 실어야
        # 함 — optuna_search.run_trial의 inner_cmd가 'world:=our_home' 하드코딩이라
        # 이 스크립트 전용으로 살짝 다른 실행 경로가 필요.
        try:
            r = run_trial_with_world(world, name, domain, duration)
        finally:
            release_domain(domain)
        results.append(r)


def run_trial_with_world(world, name, domain, duration):
    import subprocess, time, signal
    tag = f'phase2_{name}_d{domain}'
    param_file = make_param_variant(FRONTIER_BASE, FRONTIER_BEST_PARAMS, tag, 'frontier')
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    cov_path = os.path.join(SCRATCH, f'{tag}_coverage.csv')
    ROBOT = optuna_search.ROBOT
    REPO = optuna_search.REPO
    inner_cmd = (
        f"timeout -s INT {duration} ros2 launch smart_trash_bringup "
        f"sim_autonomous_mapping.launch.py world:={world} explorer:=frontier "
        f"frontier_params_file:={param_file}"
    )
    full_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {ROBOT}/install/setup.bash && "
        f"xvfb-run -a bash -c '{inner_cmd} & "
        f"LP=$!; sleep 8; "
        f"python3 {REPO}/tools/map_coverage_logger.py --out {cov_path} "
        f"--interval 5 --sim-time --duration {duration + 20} & "
        f"CP=$!; wait $LP; kill -INT $CP 2>/dev/null; wait $CP 2>/dev/null'"
    )
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), PATH='/usr/bin:' + os.environ.get('PATH', ''))
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(['bash', '-c', full_cmd], stdout=logf, stderr=subprocess.STDOUT,
                                 env=env, preexec_fn=os.setsid)
        root_pid = proc.pid
        try:
            proc.wait(timeout=duration + 120)
        except subprocess.TimeoutExpired:
            logf.write('\n[phase2] subprocess timeout\n')
        finally:
            optuna_search.kill_process_tree(root_pid)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
    coverage = final_coverage(cov_path)
    with open(log_path) as f:
        content = f.read()
    tf_stale = content.count('Transform data too old')
    progress_fail = content.count('Failed to make progress')
    collisions = content.count('Collision Ahead')
    print(f'[{name}] world={world} domain={domain} coverage={coverage:.1f}% '
          f'tf_stale={tf_stale} progress_fail={progress_fail} collisions={collisions}',
          flush=True)
    return {'name': name, 'coverage': coverage, 'tf_stale': tf_stale,
            'progress_fail': progress_fail, 'collisions': collisions}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n-jobs', type=int, default=4)
    ap.add_argument('--duration', type=int, default=150)
    args = ap.parse_args()

    optuna_search.DOMAIN_POOL = [100 + i for i in range(args.n_jobs)]

    jobs = list(STARTS.items())
    results = []
    threads = [threading.Thread(target=worker, args=(jobs, results, args.duration))
               for _ in range(args.n_jobs)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print('\n=== Phase 2 결과 요약 ===')
    for r in sorted(results, key=lambda x: list(STARTS.keys()).index(x['name'])):
        print(f"{r['name']:16s} coverage={r['coverage']:5.1f}% tf_stale={r['tf_stale']:3d} "
              f"progress_fail={r['progress_fail']:3d} collisions={r['collisions']}")
    covs = [r['coverage'] for r in results]
    print(f"\n평균 coverage: {sum(covs)/len(covs):.1f}%  "
          f"범위: {min(covs):.1f}~{max(covs):.1f}%  "
          f"조기종료(<10%): {sum(1 for c in covs if c < 10)}/{len(covs)}")


if __name__ == '__main__':
    main()
