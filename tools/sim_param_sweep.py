#!/usr/bin/env python3
"""explore_lite/Nav2 파라미터 배치 스윕 — stage_ros2 시뮬레이션 (2026-08-23).

기존 방식(파라미터 하나 바꾸고 결과 보고 다음 걸 정하는 대화식 반복)의 비효율을
사용자가 지적함: "배치 단위로 파라미터를 정해서 한번에 돌려서 결과를 보고
판단하자." Isaac Sim류 GPU 병렬 시뮬레이션과 달리 Stage는 CPU 단일 스레드라
동시 병렬은 안 되지만, **사람 개입 없이 순차 자동 실행 + 결과 일괄 비교**는
가능하다 — 이 스크립트가 그 역할.

같은 실제 집 맵(our_home world — GIMP로 손으로 트레이싱한 정확한 벽, 열린 시작지점)에서 조합마다 새로 띄우고,
자연완료(All frontiers traversed) 또는 timeout까지 기다린 뒤 로그에서
지표를 뽑아 결과 테이블에 쌓는다.

사용법:
    ROS_DOMAIN_ID는 스크립트 내부에서 42로 고정(실물과 격리).
    python3 tools/sim_param_sweep.py
"""
import csv
import os
import re
import signal
import subprocess
import sys
import time

REPO = os.path.expanduser('~/smart-trash-bin')
ROBOT = os.path.join(REPO, 'robot')
SCRATCH = '/tmp/sim_param_sweep'
os.makedirs(SCRATCH, exist_ok=True)

DOMAIN_ENV = dict(os.environ)
DOMAIN_ENV['ROS_DOMAIN_ID'] = '42'

# (max_vel_x, movement_time_allowance) 짝 — 속도를 낮추면 진행체크 시간도
# 같이 늘려야 한다는 게 오늘 확인한 교훈. required_movement_radius(0.3)는 고정.
SPEED_PRESETS = [
    (0.04, 30.0),
    (0.06, 20.0),
    (0.08, 15.0),
]
MIN_FRONTIER_SIZES = [0.2, 0.4, 0.8]

TIMEOUT_SEC = 200
NAV2_BASE = os.path.join(ROBOT, 'src/smart_trash_bringup/config/nav2_params.yaml')
EXPLORE_BASE = os.path.join(ROBOT, 'src/smart_trash_bringup/config/explore_params.yaml')


def make_nav2_variant(max_vel, allowance, tag):
    with open(NAV2_BASE) as f:
        text = f.read()
    text = re.sub(r'max_vel_x: [\d.]+', f'max_vel_x: {max_vel}', text)
    text = re.sub(r'max_speed_xy: [\d.]+', f'max_speed_xy: {max_vel}', text)
    text = re.sub(r'movement_time_allowance: [\d.]+', f'movement_time_allowance: {allowance}', text)
    path = os.path.join(SCRATCH, f'nav2_{tag}.yaml')
    with open(path, 'w') as f:
        f.write(text)
    return path


def make_explore_variant(min_size, tag):
    with open(EXPLORE_BASE) as f:
        text = f.read()
    text = re.sub(r'min_frontier_size: [\d.]+', f'min_frontier_size: {min_size}', text)
    path = os.path.join(SCRATCH, f'explore_{tag}.yaml')
    with open(path, 'w') as f:
        f.write(text)
    return path


def cleanup():
    names = ['stage_ros2', 'controller_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'explore', 'lifecycle_manager', 'async_slam_toolbox_node']
    subprocess.run(
        "ps aux | grep -E '" + '|'.join(names) + "' | grep -v grep | awk '{print $2}' | xargs -r kill -9",
        shell=True)
    time.sleep(2)
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    import mapping_cleanup
    mapping_cleanup.clean_orphan_shm()


def run_one(nav2_file, explore_file, tag):
    log_path = os.path.join(SCRATCH, f'{tag}.log')
    cmd = (
        f"xvfb-run -a ros2 launch smart_trash_bringup sim_autonomous_mapping.launch.py "
        f"world:=our_home nav2_params_file:={nav2_file} explore_params_file:={explore_file}"
    )
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(
            ['setsid', 'bash', '-c',
             f"source /opt/ros/jazzy/setup.bash && source {ROBOT}/install/setup.bash && exec {cmd}"],
            stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT,
            env=DOMAIN_ENV, cwd=ROBOT, preexec_fn=os.setsid)

    start = time.time()
    finished_naturally = False
    while time.time() - start < TIMEOUT_SEC:
        time.sleep(3)
        try:
            with open(log_path) as f:
                content = f.read()
        except FileNotFoundError:
            content = ''
        if 'All frontiers traversed' in content or 'Exploration stopped' in content:
            finished_naturally = True
            break

    duration = time.time() - start
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=10)

    with open(log_path) as f:
        content = f.read()
    goal_switches = content.count('Begin navigating')
    progress_fail = content.count('Failed to make progress')
    collisions = content.count('Collision Ahead')
    no_valid = content.count('No valid trajectories')

    cleanup()
    return {
        'tag': tag,
        'finished': finished_naturally,
        'duration_s': round(duration, 1),
        'goal_switches': goal_switches,
        'progress_fail': progress_fail,
        'collision': collisions,
        'no_valid_traj': no_valid,
    }


def main():
    results = []
    results_path = os.path.join(SCRATCH, 'results.csv')
    combos = []
    for max_vel, allowance in SPEED_PRESETS:
        for min_size in MIN_FRONTIER_SIZES:
            combos.append((max_vel, allowance, min_size))

    print(f"총 {len(combos)}개 조합, 조합당 최대 {TIMEOUT_SEC}초")
    cleanup()

    for i, (max_vel, allowance, min_size) in enumerate(combos):
        tag = f"v{max_vel}_a{allowance}_f{min_size}"
        print(f"[{i+1}/{len(combos)}] {tag} 시작...", flush=True)
        nav2_file = make_nav2_variant(max_vel, allowance, tag)
        explore_file = make_explore_variant(min_size, tag)
        r = run_one(nav2_file, explore_file, tag)
        r['max_vel'] = max_vel
        r['allowance'] = allowance
        r['min_frontier_size'] = min_size
        results.append(r)
        print(f"  -> 완료={r['finished']} 시간={r['duration_s']}s 목표전환={r['goal_switches']} "
              f"실패={r['progress_fail']} 충돌={r['collision']}", flush=True)

        with open(results_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['tag', 'max_vel', 'allowance', 'min_frontier_size',
                                               'finished', 'duration_s', 'goal_switches',
                                               'progress_fail', 'collision', 'no_valid_traj'])
            w.writeheader()
            w.writerows(results)

    print(f"\n전체 완료. 결과: {results_path}")


if __name__ == '__main__':
    main()
