#!/usr/bin/env python3
"""사용자가 Pi에서 보고한 "1차: 목표전환 0회로 즉시 멈춤, 2차(재시도): 정상 완료"
재현 전용 — 같은 (max_vel=0.04, allowance=30.0, min_frontier_size=0.8) 조합을
N회 반복해서 "0회 목표전환" 부트스트랩 실패가 얼마나 자주 나는지, 나올 때
초기 구간(bt_navigator 활성화 ~ 첫 목표 시도)에 TF 관련 에러가 있는지를 본다.

tools/sim_param_sweep.py의 변형 생성/정리 로직을 그대로 재사용 — 같은 도구,
같은 조건으로 재현성을 보장하기 위함(직접 새로 짜면 조건이 미묘하게 달라질 위험).
"""
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.expanduser('~/smart-trash-bin/tools'))
import sim_param_sweep as sweep  # noqa: E402

N_TRIALS = 6
TIMEOUT_SEC = 210

nav2_file = sweep.make_nav2_variant(0.04, 30.0, 'repro')
explore_file = sweep.make_explore_variant(0.8, 'repro')

sweep.cleanup()

results = []
for i in range(N_TRIALS):
    tag = f'repro{i}'
    print(f'[{i+1}/{N_TRIALS}] {tag} 시작...', flush=True)

    log_path = os.path.join(sweep.SCRATCH, f'{tag}.log')
    inner_cmd = (
        f"timeout -s INT {TIMEOUT_SEC} ros2 launch "
        f"smart_trash_bringup sim_autonomous_mapping.launch.py world:=our_home "
        f"explorer:=explore_lite nav2_params_file:={nav2_file} "
        f"explore_params_file:={explore_file}"
    )
    full_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {sweep.ROBOT}/install/setup.bash && "
        f"xvfb-run -a bash -c '{inner_cmd}' 2>&1 | awk '{{ print systime(), $0; fflush(); }}'"
    )
    t0 = time.time()
    with open(log_path, 'w') as logf:
        subprocess.run(['bash', '-c', full_cmd], stdout=logf, stderr=subprocess.STDOUT,
                        env=dict(os.environ, ROS_DOMAIN_ID='42'))
    duration = time.time() - t0

    with open(log_path) as f:
        content = f.read()
    goal_switches = content.count('Begin navigating')
    tf_stale = content.count('Transform data too old')
    completed = ('All frontiers traversed' in content
                 or 'No frontiers found' in content)

    # 부트스트랩 실패(목표전환 0)인 경우, 활성화~파일 끝 사이 전체를 별도 저장
    # (사람이 직접 눈으로 볼 수 있게).
    bootstrap_failed = goal_switches == 0

    print(f'  -> duration={duration:.1f}s goal_switches={goal_switches} '
          f'tf_stale={tf_stale} completed={completed} '
          f'BOOTSTRAP_FAILED={bootstrap_failed}', flush=True)

    results.append({
        'tag': tag, 'duration_s': round(duration, 1),
        'goal_switches': goal_switches, 'tf_stale': tf_stale,
        'completed': completed, 'bootstrap_failed': bootstrap_failed,
        'log_path': log_path,
    })

    sweep.cleanup()
    time.sleep(2)

print('\n=== 요약 ===')
for r in results:
    print(r)

n_bootstrap_fail = sum(1 for r in results if r['bootstrap_failed'])
print(f'\n부트스트랩 실패(목표전환 0회): {n_bootstrap_fail}/{N_TRIALS}')
