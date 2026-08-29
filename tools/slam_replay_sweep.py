#!/usr/bin/env python3
"""SLAM 루프클로저 파라미터 재생 스윕 하네스 (pc_tasks.md 11-a, Track 0 후속).

배경: 2026-08-29 실물 매핑 2회 모두 루프클로저 0건, map->odom yaw 보정
62.8°/-30.3°(docs/midterm_review_20260829.md). 실물 반복 주행은 비효율적
(CLAUDE.md "🧪 검증 방식 전환" 절, 2026-08-17 합의)이라 rosbag 재생으로
오프라인 스윕한다 — Remi가 실물 bag을 올리면 --bag으로 바로 넘겨서 튜닝,
그 전까지는 PC의 stage_ros2로 만든 테스트 bag으로 배관만 검증해도 된다.

파이프라인 (전부 실측으로 확인됨, 2026-08-29):
  1. `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<병합yaml>
     use_sim_time:=true` — async_slam_toolbox_node는 LifecycleNode라 `ros2 run`으로
     직접 띄우면 configure/activate 이벤트가 없어 unconfigured 상태로 멈춘다
     (스캔을 하나도 처리 못 함, 처음에 이걸로 삽질함) — 반드시 이 launch 파일을
     그대로 재사용해야 자동 lifecycle 전이가 딸려온다.
  2. `map_frame`을 매 트라이얼 `map_replay`로 오버라이드 — bag에 이미 원본
     세션의 map->odom TF가 녹화돼 있을 수 있어서, 기본 프레임 이름을 그대로
     쓰면 재생용 인스턴스와 원본 기록이 같은 프레임 쌍을 동시에 발행해 tf2가
     충돌한다.
  3. 루프클로저 검출: 로그에서 "Loop closed!" 문자열 카운트(libkartoSlamToolbox.so를
     strings로 뒤져서 확인한 실제 로그 문구 — "loop"로만 grep하면 "Closing loop..."
     같은 시작 로그까지 섞여서 정확한 완료 횟수를 못 셈).
  4. 최종 map_replay->odom yaw: `tools/tf_yaw_probe.py`를 병렬로 띄워 재생 내내
     폴링, 마지막 성공 조회값을 사용(slam_toolbox 종료 후엔 TF도 끊기므로).

killpg 버그(docs/pc_tasks.md "🐛 killpg 버그 2건" 참고 — reap 이후 getpgid 호출
금지, job-control로 갈라진 하위 그룹은 killpg가 못 잡음)를 겪은 적이 있어서
처음부터 optuna_search.kill_process_tree()(ppid 트리 직접 순회)를 재사용한다.
배치 실행 전후 좀비 확인 습관도 동일 적용할 것.

사용법:
    # PC의 stage_ros2로 배관 검증용 테스트 bag 만들기(선택, 이미 있으면 생략)
    # -> tools/record_bag.sh 스타일로 sim_autonomous_mapping.launch.py 돌리며 녹화

    # Optuna 스윕
    python3 tools/slam_replay_sweep.py --bag <bag_dir> --n-trials 12 --n-jobs 4

    # 단발 실행(파라미터 하나만, 디버깅용)
    python3 tools/slam_replay_sweep.py --bag <bag_dir> --once \
        --loop-search-maximum-distance 3.0
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(__file__))
import optuna_search  # noqa: E402 — kill_process_tree, acquire/release_domain, DOMAIN 풀 재사용

REPO = os.path.expanduser('~/smart-trash-bin')
ROBOT = os.path.join(REPO, 'robot')
MAPPER_BASE = os.path.join(
    ROBOT, 'src/smart_trash_bringup/config/mapper_params_online_async.yaml')
SCRATCH = '/tmp/slam_replay_sweep'
os.makedirs(SCRATCH, exist_ok=True)

# 항상 고정으로 오버라이드하는 값들 — 스윕 대상 파라미터와 분리해서 관리.
FIXED_OVERRIDE = {
    'map_frame': 'map_replay',
}

LOOP_CLOSED_RE = re.compile(r'Loop closed!')


def make_merged_yaml(base_file, params, tag):
    """base_file(원본 slam_toolbox 파라미터)에 params(스윕값+고정 오버라이드)를
    라인 단위로 치환해 새 yaml을 만든다 — optuna_search.make_param_variant와
    동일 원리, mapper_params_online_async.yaml의 `/**` 와일드카드 스코프
    그대로 재사용 가능."""
    with open(base_file) as f:
        lines = f.readlines()
    out = []
    seen = set()
    for line in lines:
        stripped = line.split(':')[0].strip()
        if stripped in params:
            indent = line[:len(line) - len(line.lstrip())]
            out.append(f'{indent}{stripped}: {params[stripped]}\n')
            seen.add(stripped)
        else:
            out.append(line)
    missing = set(params) - seen
    if missing:
        raise ValueError(f'{base_file}에 없어서 오버라이드 못한 키: {missing}')
    path = os.path.join(SCRATCH, f'mapper_{tag}.yaml')
    with open(path, 'w') as f:
        f.writelines(out)
    return path


def run_one(bag_path, params, domain, tag, play_rate, probe_duration):
    """slam_toolbox 재생 세션 하나를 실행하고 (loop_count, final_yaw_deg,
    final_xy) 를 반환. yaw/xy는 TF를 못 잡았으면 None.

    ⚠️ probe는 --out 파일에 매 성공적인 폴링마다 즉시 덮어써서 결과를 남긴다
    (tf_yaw_probe.py 참고) — probe 프로세스 자체를 정리 과정에서 SIGKILL해도
    마지막으로 기록된 값이 파일에 남아있으므로 안전하다. 처음엔 "duration
    끝나면 stdout에 한 번만 출력" 방식이었는데, 그 출력 전에 프로세스가
    죽는 경로(kill_process_tree가 finally에서 항상 도는 구조)와 맞물려
    결과가 통째로 유실되는 걸 실측으로 겪었다."""
    merged = make_merged_yaml(MAPPER_BASE, params, tag)
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    probe_out = os.path.join(SCRATCH, f'{tag}_probe.txt')

    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    setup = f'source /opt/ros/jazzy/setup.bash && source {ROBOT}/install/setup.bash && '

    launch_cmd = (
        setup +
        f"exec ros2 launch slam_toolbox online_async_launch.py "
        f"slam_params_file:={merged} use_sim_time:=true"
    )
    logf = open(log_path, 'w')
    launch_proc = subprocess.Popen(['bash', '-c', launch_cmd], stdout=logf,
                                    stderr=subprocess.STDOUT, env=env,
                                    preexec_fn=os.setsid)
    launch_root_pid = launch_proc.pid
    probe_root_pid = None
    try:
        time.sleep(4)  # configure/activate 완료 대기(실측 0.3~4초, 병렬 조건 여유)

        probe_cmd = (
            setup +
            f"exec python3 {REPO}/tools/tf_yaw_probe.py --duration {probe_duration} "
            f"--target-frame map_replay --source-frame odom --use-sim-time "
            f"--out {probe_out}"
        )
        probe_proc = subprocess.Popen(['bash', '-c', probe_cmd], stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, env=env,
                                       preexec_fn=os.setsid)
        probe_root_pid = probe_proc.pid

        play_cmd = (
            setup +
            f"exec timeout -s INT {int(probe_duration)} "
            f"ros2 bag play {bag_path} --clock --rate {play_rate}"
        )
        subprocess.run(['bash', '-c', play_cmd], stdout=logf,
                        stderr=subprocess.STDOUT, env=env)
        logf.flush()
        # bag 재생이 끝난 시점의 TF가 곧 "최종값" — 굳이 probe의 나머지
        # duration까지 기다릴 필요 없이 바로 정리한다(더 빠르고, 어차피
        # --out 파일은 이미 최신값을 담고 있다).
        time.sleep(0.5)
    finally:
        if probe_root_pid is not None:
            optuna_search.kill_process_tree(probe_root_pid)
        optuna_search.kill_process_tree(launch_root_pid)
        try:
            launch_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        logf.close()

    with open(log_path) as f:
        content = f.read()
    loop_count = len(LOOP_CLOSED_RE.findall(content))

    yaw_deg, x, y = None, None, None
    if os.path.exists(probe_out):
        with open(probe_out) as f:
            for line in f:
                key, _, val = line.strip().partition('=')
                if val == 'NONE':
                    continue
                if key == 'YAW_DEG':
                    yaw_deg = float(val)
                elif key == 'X':
                    x = float(val)
                elif key == 'Y':
                    y = float(val)
    xy = (x, y) if x is not None and y is not None else None

    return {'loop_count': loop_count, 'final_yaw_deg': yaw_deg, 'final_xy': xy,
            'log_path': log_path}


def suggest_params(trial):
    return {
        'loop_search_maximum_distance': round(trial.suggest_float(
            'loop_search_maximum_distance', 1.0, 8.0), 2),
        'loop_match_minimum_response_coarse': round(trial.suggest_float(
            'loop_match_minimum_response_coarse', 0.15, 0.55), 3),
        'loop_match_minimum_response_fine': round(trial.suggest_float(
            'loop_match_minimum_response_fine', 0.25, 0.65), 3),
        'loop_match_minimum_chain_size': trial.suggest_int(
            'loop_match_minimum_chain_size', 4, 20),
    }


def score(result):
    """루프클로저 횟수만 스코어에 반영한다.

    ⚠️ 2026-08-29 밤 수정 — 처음엔 `abs(final_yaw_deg)`를 페널티로 뺐었는데,
    같은 날 실물 검증(docs/midterm_review_20260829.md 7.8절)에서 이게
    **잘못된 지표였음이 확정**됐다: map→odom yaw는 "odom 프레임의 세션 시작
    방향"과 "map 프레임의 절대 방향" 차이일 뿐이라 애초에 임의값이고, 작을
    이유가 없다(실전에서 맵이 "거의 완벽"했던 세션도 yaw는 -157.99°로 컸음).
    62.8°/-30.3°/73.4°를 "왜곡의 크기"로 해석한 것 자체가 부정확했던 것 —
    진짜 신호는 이 절대값이 아니라 **맵 내부 일관성**(방이 서로 직각을
    이루는지, 벽이 겹치는지, 루프클로저 성공 여부)이다. 방 직각도까지
    자동 측정하는 건 아직 이 하네스 범위 밖이라(이미지 분석 필요), 우선
    루프클로저 횟수만 남기고 yaw는 참고 정보로만 결과에 남긴다(스코어에서
    제외). yaw=None(TF 자체를 못 잡음)은 slam_toolbox가 아예 안 뜬 것 같은
    구조적 실패 신호라 페널티 유지."""
    if result['final_yaw_deg'] is None:
        return result['loop_count'] * 10.0 - 1000.0
    return result['loop_count'] * 10.0


def objective(trial, bag_path, play_rate, probe_duration):
    params = suggest_params(trial)
    merged_params = dict(params, **FIXED_OVERRIDE, debug_logging='true')
    domain = optuna_search.acquire_domain()
    tag = f'trial{trial.number}_d{domain}'
    try:
        result = run_one(bag_path, merged_params, domain, tag, play_rate, probe_duration)
    finally:
        optuna_search.release_domain(domain)
    s = score(result)
    trial.set_user_attr('loop_count', result['loop_count'])
    trial.set_user_attr('final_yaw_deg', result['final_yaw_deg'])
    print(f'[trial {trial.number}] domain={domain} params={params} '
          f'loop_count={result["loop_count"]} final_yaw_deg={result["final_yaw_deg"]} '
          f'score={s:.1f}', flush=True)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='rosbag 디렉토리(mcap)')
    ap.add_argument('--n-trials', type=int, default=12)
    ap.add_argument('--n-jobs', type=int, default=4)
    ap.add_argument('--play-rate', type=float, default=3.0,
                     help='ros2 bag play --rate (기본 3배속 — 스윕 시간 단축)')
    ap.add_argument('--probe-duration', type=float, default=None,
                     help='세션당 대기 시간(초). 생략 시 bag 길이/play_rate + 여유로 자동 계산')
    ap.add_argument('--study-db', default=os.path.join(SCRATCH, 'study.db'))
    ap.add_argument('--once', action='store_true',
                     help='Optuna 없이 --loop-search-maximum-distance 등 단발 실행(디버깅용)')
    ap.add_argument('--loop-search-maximum-distance', type=float, default=3.0)
    ap.add_argument('--loop-match-minimum-response-coarse', type=float, default=0.35)
    ap.add_argument('--loop-match-minimum-response-fine', type=float, default=0.45)
    ap.add_argument('--loop-match-minimum-chain-size', type=int, default=10)
    args = ap.parse_args()

    if args.probe_duration is None:
        bag_info = subprocess.run(
            ['bash', '-c', f'source /opt/ros/jazzy/setup.bash && ros2 bag info {args.bag}'],
            capture_output=True, text=True).stdout
        m = re.search(r'Duration:\s+([\d.]+)s', bag_info)
        bag_sec = float(m.group(1)) if m else 90.0
        args.probe_duration = bag_sec / args.play_rate + 15.0

    if args.once:
        params = dict(FIXED_OVERRIDE, debug_logging='true',
                       loop_search_maximum_distance=args.loop_search_maximum_distance,
                       loop_match_minimum_response_coarse=args.loop_match_minimum_response_coarse,
                       loop_match_minimum_response_fine=args.loop_match_minimum_response_fine,
                       loop_match_minimum_chain_size=args.loop_match_minimum_chain_size)
        optuna_search.DOMAIN_POOL = [100]
        domain = optuna_search.acquire_domain()
        result = run_one(args.bag, params, domain, 'once', args.play_rate, args.probe_duration)
        optuna_search.release_domain(domain)
        print(f'loop_count={result["loop_count"]} final_yaw_deg={result["final_yaw_deg"]} '
              f'final_xy={result["final_xy"]}')
        return 0

    import optuna
    optuna_search.DOMAIN_POOL = [100 + i for i in range(args.n_jobs)]
    study = optuna.create_study(direction='maximize', storage=f'sqlite:///{args.study_db}',
                                 study_name='slam_replay_sweep', load_if_exists=True)
    study.optimize(lambda t: objective(t, args.bag, args.play_rate, args.probe_duration),
                    n_trials=args.n_trials, n_jobs=args.n_jobs, catch=(Exception,))

    print('\n=== 최적 결과 ===')
    print('params:', study.best_params)
    print('score:', study.best_value)
    print(f'\nstudy DB: {args.study_db}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
