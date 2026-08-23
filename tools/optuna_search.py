#!/usr/bin/env python3
"""explore_lite/frontier_exploration_ros2 파라미터 최적화 — Optuna 기반,
PC(12코어)에서 병렬 탐색.

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
    python3 tools/optuna_search.py --explorer explore_lite --n-trials 20 --n-jobs 4 --duration 150
    python3 tools/optuna_search.py --explorer frontier --n-trials 20 --n-jobs 4 --duration 150
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
FRONTIER_BASE = os.path.join(ROBOT, 'src/frontier_exploration_ros2/config/params.yaml')
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


def make_param_variant(base_file, params, tag, prefix):
    """base_file을 읽어 params에 있는 키만 값을 치환한 변형 yaml을 만든다.
    라인 단위 치환이라 YAML 구조(중첩/스코프)와 무관하게 동작 — explore_params.yaml
    (/** 와일드카드)과 frontier_exploration_ros2/config/params.yaml
    (frontier_explorer: 명시 스코프) 둘 다 그대로 재사용 가능."""
    with open(base_file) as f:
        lines = f.readlines()
    out = []
    for line in lines:
        stripped = line.split(':')[0].strip()
        if stripped in params:
            indent = line[:len(line) - len(line.lstrip())]
            out.append(f'{indent}{stripped}: {params[stripped]}\n')
        else:
            out.append(line)
    path = os.path.join(SCRATCH, f'{prefix}_{tag}.yaml')
    with open(path, 'w') as f:
        f.writelines(out)
    return path


def run_trial(explorer, param_file, domain, duration, tag):
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    cov_path = os.path.join(SCRATCH, f'{tag}_coverage.csv')
    params_arg = ('explore_params_file' if explorer == 'explore_lite'
                  else 'frontier_params_file')
    inner_cmd = (
        f"timeout -s INT {duration} ros2 launch smart_trash_bringup "
        f"sim_autonomous_mapping.launch.py world:=our_home explorer:={explorer} "
        f"{params_arg}:={param_file}"
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
    # 그룹을 만들고 killpg로 정리한다.
    #
    # ⚠️⚠️ 2026-08-23 밤: 처음엔 killpg를 TimeoutExpired 예외 처리 안에만
    # 뒀었다(Remi가 docs/pc_tasks.md에서 지적). bash 스크립트 자체(wait $LP;
    # kill -INT $CP; wait $CP)는 정상 종료(exit 0)됐는데 xvfb-run/launch가
    # SIGINT 처리 중 일부 자식만 남기고 죽는 경우, proc.wait()는 정상 반환되고
    # 그 경로에는 killpg가 없어서 좀비가 그대로 남았다 — 실제로 trial13(수정
    # "후"에도 재현)이 ~2시간 살아남아 다음 배치(phase1_compare.py)와 같은
    # ROS_DOMAIN_ID에서 충돌, 그 비교 결과 6개 전부 무효화됨. 정상/타임아웃
    # 무관하게 **항상** killpg를 호출하도록 finally로 옮겼었다.
    #
    # ⚠️⚠️⚠️ 2026-08-23 심야: 그런데도 여전히 좀비가 남았다(도메인 101/103을
    # 각각 28분/38분째 물고 있는 고아 트리 실측). 원인은 `os.getpgid(proc.pid)`
    # 자체 — proc.wait()가 "정상 반환"(exit 0, TimeoutExpired 아님)되면 그
    # 시점에 파이썬이 이미 waitpid()로 proc.pid를 reap해버린다. 그 뒤에
    # os.getpgid(proc.pid)를 부르면 그 PID가 이미 죽어 존재하지 않으므로
    # ProcessLookupError가 나고, 바로 아래 except가 이를 조용히 삼켜서
    # **killpg가 사실상 한 번도 실행되지 않았다** — grandchildren(ros2 launch가
    # 띄운 stage/slam/controller/planner/... 전부)이 그대로 살아남아 좀비/고아가
    # 됐다. 타임아웃 경로에서만 pid가 아직 살아있어 우연히 성공했던 것.
    # preexec_fn=os.setsid로 시작했으므로 그룹 리더의 pgid는 항상 proc.pid와
    # 동일하다 — reap 여부와 무관하게 이미 알고 있는 값이니 getpgid 조회 자체가
    # 불필요하고 위험하다. proc.pid를 pgid로 바로 쓴다(ESRCH만 그룹이 이미
    # 전멸했다는 뜻이므로 무시).
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(['bash', '-c', full_cmd], stdout=logf, stderr=subprocess.STDOUT,
                                 env=env, preexec_fn=os.setsid)
        pgid = proc.pid  # setsid 세션 리더 = pgid. reap 후에도 유효한 값이라 getpgid() 불필요.
        try:
            proc.wait(timeout=duration + 120)
        except subprocess.TimeoutExpired:
            logf.write('\n[optuna_search] subprocess timeout\n')
        finally:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
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


def suggest_explore_lite_params(trial):
    return {
        'potential_scale': round(trial.suggest_float('potential_scale', 0.5, 4.0), 2),
        'gain_scale': round(trial.suggest_float('gain_scale', 0.5, 4.0), 2),
        'min_frontier_size': round(trial.suggest_float('min_frontier_size', 0.1, 1.0), 2),
        'min_frontier_distance': round(trial.suggest_float('min_frontier_distance', 0.2, 1.0), 2),
    }


def suggest_frontier_params(trial):
    # weight_distance_wd/weight_gain_ws가 explore_lite의
    # potential_scale/gain_scale과 개념적으로 대응. occ_threshold/
    # min_frontier_size_cells도 마찬가지로 min_frontier_size에 대응.
    # frontier_selection_min_distance/frontier_candidate_min_goal_distance_m은
    # 로봇 반경 기반 물리 상수(0.35, launch에서 하드코딩)라 탐색 대상에서 제외.
    return {
        'weight_distance_wd': round(trial.suggest_float('weight_distance_wd', 0.3, 3.0), 2),
        'weight_gain_ws': round(trial.suggest_float('weight_gain_ws', 0.3, 3.0), 2),
        'occ_threshold': trial.suggest_int('occ_threshold', 50, 85),
        'min_frontier_size_cells': trial.suggest_int('min_frontier_size_cells', 2, 12),
        'mrtsp_solver': trial.suggest_categorical('mrtsp_solver', ['dp', 'greedy']),
    }


def objective(trial, explorer, duration):
    if explorer == 'explore_lite':
        params = suggest_explore_lite_params(trial)
        base_file, prefix = EXPLORE_BASE, 'explore'
    else:
        params = suggest_frontier_params(trial)
        base_file, prefix = FRONTIER_BASE, 'frontier'
        # ⚠️ 2026-08-23 밤: 로봇 물리 상수(패키지 기본값은 TurtleBot3 Waffle
        # 기준이라 우리와 다름)를 명시적으로 강제 포함해야 한다. 안 넣으면
        # base_file(패키지 원본)의 robot_base_frame: base_footprint가 그대로
        # 복사되고, 이 오버라이드 파일이 launch의 하드코딩 dict보다 params
        # 리스트에서 뒤라 그 값이 이겨버린다 — 실제로 "base_footprint
        # 존재 안 함" 에러로 frontier_explorer가 목표를 단 한 번도 못
        # 시도하는 사고가 났다(coverage 7.2% x2, Begin navigating 0회).
        params.update({
            'robot_base_frame': 'base_link',
            'frontier_selection_min_distance': 0.35,
            'frontier_candidate_min_goal_distance_m': 0.35,
        })

    domain = acquire_domain()
    tag = f'{prefix}_trial{trial.number}_d{domain}'
    try:
        param_file = make_param_variant(base_file, params, tag, prefix)
        t0 = time.time()
        cov_path, log_path = run_trial(explorer, param_file, domain, duration, tag)
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
    ap.add_argument('--explorer', choices=['explore_lite', 'frontier'], default='explore_lite')
    ap.add_argument('--n-trials', type=int, default=20)
    ap.add_argument('--n-jobs', type=int, default=4)
    ap.add_argument('--duration', type=int, default=150)
    ap.add_argument('--study-db', default=None)
    args = ap.parse_args()
    if args.study_db is None:
        args.study_db = os.path.join(SCRATCH, f'study_{args.explorer}.db')

    DOMAIN_POOL = [100 + i for i in range(args.n_jobs)]

    study = optuna.create_study(
        direction='maximize',
        storage=f'sqlite:///{args.study_db}',
        study_name=f'{args.explorer}_our_home',
        load_if_exists=True,
    )
    # catch 없이는 트라이얼 하나의 예외가 전체 study.optimize()를 중단시킨다
    # (2026-08-23 실제로 겪음 — 24개 중 6개만 돌고 죽었었다). 예상 못 한 예외가
    # 나도 그 트라이얼만 실패 처리하고 나머지는 계속 진행하게 한다.
    study.optimize(lambda t: objective(t, args.explorer, args.duration),
                    n_trials=args.n_trials, n_jobs=args.n_jobs,
                    catch=(Exception,))

    print('\n=== 최적 결과 ===')
    print('params:', study.best_params)
    print('value:', study.best_value)
    print(f'\nstudy DB: {args.study_db} (optuna-dashboard로 시각화 가능)')


if __name__ == '__main__':
    main()
