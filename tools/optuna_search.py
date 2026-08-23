#!/usr/bin/env python3
"""explore_lite 파라미터 최적화 — Optuna 기반, PC(12코어)에서 병렬 탐색.

목적함수: t=150초 시점 실측 맵 커버리지%(map_coverage_logger.py 결과) 최대화.
"완료 선언"(All frontiers traversed) 대신 실측 커버리지를 쓰는 이유는
explore_lite가 오탐(3번 실패하고 "완료"로 착각)한 전례가 있기 때문 — 자기
신고보다 실측이 신뢰도가 높다.

병렬 실행: 각 트라이얼은 서로 다른 ROS_DOMAIN_ID(100~103, 총 4슬롯)를 락으로
관리하는 풀에서 빌려 쓴다. xvfb-run은 알아서 서로 다른 디스플레이를 잡으므로
별도 처리 불필요. Stage/Nav2/SLAM은 전부 CPU 워크로드라 GPU는 무관.

⚠️ 파이에서는 절대 이렇게 병렬로 돌리지 말 것 — 2026-08-23에 자원경합으로
SIGKILL 몰살+SSH 단절까지 겪었다. PC(12코어)에서만 병렬화한다.

사용법:
    python3 tools/optuna_search.py --n-trials 20 --n-jobs 4 --duration 150
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import optuna

REPO = os.path.expanduser('~/smart-trash-bin')
ROBOT = os.path.join(REPO, 'robot')
EXPLORE_BASE = os.path.join(ROBOT, 'src/smart_trash_bringup/config/explore_params.yaml')
SCRATCH = '/tmp/optuna_search'
os.makedirs(SCRATCH, exist_ok=True)

DOMAIN_POOL = None  # main()에서 --n-jobs 크기로 초기화
DOMAIN_LOCK = threading.Lock()
DOMAIN_CV = threading.Condition(DOMAIN_LOCK)


def acquire_domain():
    with DOMAIN_CV:
        while not DOMAIN_POOL:
            DOMAIN_CV.wait()
        return DOMAIN_POOL.pop()


def release_domain(d):
    with DOMAIN_CV:
        DOMAIN_POOL.append(d)
        DOMAIN_CV.notify()


def make_explore_variant(params, tag):
    with open(EXPLORE_BASE) as f:
        lines = f.readlines()
    out = []
    for line in lines:
        stripped = line.split(':')[0].strip()
        if stripped in params:
            indent = line[:len(line) - len(line.lstrip())]
            out.append(f'{indent}{stripped}: {params[stripped]}\n')
        else:
            out.append(line)
    path = os.path.join(SCRATCH, f'explore_{tag}.yaml')
    with open(path, 'w') as f:
        f.writelines(out)
    return path


def run_trial(explore_file, domain, duration, tag):
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    cov_path = os.path.join(SCRATCH, f'{tag}_coverage.csv')
    inner_cmd = (
        f"timeout -s INT {duration} ros2 launch smart_trash_bringup "
        f"sim_autonomous_mapping.launch.py world:=our_home explorer:=explore_lite "
        f"explore_params_file:={explore_file}"
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
    # ⚠️ 내부적으로 timeout(launch, duration초) -> coverage_logger(duration+20초)
    # -> wait/kill 정리 순으로 겹쳐 있어서, subprocess 자체 timeout을 duration+60
    # 정도로만 주면 N=4 동시 실행 시 종료 처리가 늦어질 때 초과할 수 있다
    # (2026-08-23 실제로 trial 0에서 발생 — TimeoutExpired). 여유를 크게 둔다.
    #
    # ⚠️ subprocess.run(timeout=)이 만료되면 파이썬은 직접 띄운 bash만 죽이고
    # 그 bash가 백그라운드(&)로 띄운 xvfb-run/ros2 launch 자식들은 안 죽는다 —
    # 그대로 두면 오늘 하루 종일 씨름한 것과 같은 고아 프로세스가 생긴다.
    # tools/sim_param_sweep.py(Remi 작성)와 동일하게 setsid로 새 프로세스
    # 그룹을 만들고, timeout 시 그룹 전체를 killpg로 정리한다.
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(['bash', '-c', full_cmd], stdout=logf, stderr=subprocess.STDOUT,
                                 env=env, preexec_fn=os.setsid)
        try:
            proc.wait(timeout=duration + 120)
        except subprocess.TimeoutExpired:
            logf.write('\n[optuna_search] subprocess timeout — 프로세스 그룹 강제 종료\n')
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=10)
    return cov_path, log_path


def final_coverage(cov_path):
    if not os.path.exists(cov_path):
        return 0.0
    last = None
    with open(cov_path) as f:
        for row in csv.DictReader(f):
            last = row
    if last is None:
        return 0.0
    try:
        return float(last['coverage_pct'])
    except (KeyError, ValueError):
        return 0.0


def objective(trial, duration):
    params = {
        'potential_scale': round(trial.suggest_float('potential_scale', 0.5, 4.0), 2),
        'gain_scale': round(trial.suggest_float('gain_scale', 0.5, 4.0), 2),
        'min_frontier_size': round(trial.suggest_float('min_frontier_size', 0.1, 1.0), 2),
        'min_frontier_distance': round(trial.suggest_float('min_frontier_distance', 0.2, 1.0), 2),
    }
    domain = acquire_domain()
    tag = f'trial{trial.number}_d{domain}'
    try:
        explore_file = make_explore_variant(params, tag)
        t0 = time.time()
        cov_path, log_path = run_trial(explore_file, domain, duration, tag)
        elapsed = time.time() - t0
        coverage = final_coverage(cov_path)
        with open(log_path) as f:
            content = f.read()
        tf_stale = content.count('Transform data too old')
        progress_fail = content.count('Failed to make progress')
        collisions = content.count('Collision Ahead')
        trial.set_user_attr('elapsed_s', round(elapsed, 1))
        trial.set_user_attr('tf_stale', tf_stale)
        trial.set_user_attr('progress_fail', progress_fail)
        trial.set_user_attr('collisions', collisions)
        print(f'[trial {trial.number}] domain={domain} params={params} '
              f'coverage={coverage:.1f}% tf_stale={tf_stale} '
              f'progress_fail={progress_fail} collisions={collisions}', flush=True)
        # 충돌은 안전 관련이라 강하게 페널티
        return coverage - collisions * 5.0
    finally:
        release_domain(domain)


def main():
    global DOMAIN_POOL
    ap = argparse.ArgumentParser()
    ap.add_argument('--n-trials', type=int, default=20)
    ap.add_argument('--n-jobs', type=int, default=4)
    ap.add_argument('--duration', type=int, default=150)
    ap.add_argument('--study-db', default=os.path.join(SCRATCH, 'study.db'))
    args = ap.parse_args()

    DOMAIN_POOL = [100 + i for i in range(args.n_jobs)]

    study = optuna.create_study(
        direction='maximize',
        storage=f'sqlite:///{args.study_db}',
        study_name='explore_lite_our_home',
        load_if_exists=True,
    )
    # catch 없이는 트라이얼 하나의 예외가 전체 study.optimize()를 중단시킨다
    # (2026-08-23 실제로 겪음 — 24개 중 6개만 돌고 죽었었다). 예상 못 한 예외가
    # 나도 그 트라이얼만 실패 처리하고 나머지는 계속 진행하게 한다.
    study.optimize(lambda t: objective(t, args.duration),
                    n_trials=args.n_trials, n_jobs=args.n_jobs,
                    catch=(Exception,))

    print('\n=== 최적 결과 ===')
    print('params:', study.best_params)
    print('value:', study.best_value)
    print(f'\nstudy DB: {args.study_db} (optuna-dashboard로 시각화 가능)')


if __name__ == '__main__':
    main()
