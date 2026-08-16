#!/usr/bin/env python3
"""매핑/자율주행 세션을 깨끗하게 정리한다 (프로세스 + DDS 공유메모리).

**왜 이게 별도 도구여야 하나 (2026-08-16에 비싸게 배운 것들)**

1. `pkill -f <패턴>`은 **자기 자신을 죽인다.** 패턴 문자열이 그걸 실행하는 셸의
   명령줄에도 들어있기 때문. 세션 중에 이걸로 몇 번이나 작업 셸이 날아갔고,
   비상정지 버튼도 같은 이유로 무관한 프로세스를 죽일 뻔했다.
   → 여기서는 `/proc/<pid>/cmdline`을 읽어 **argv[0]의 basename이 정확히 일치**할
     때만 죽인다. 부분문자열 매칭 안 함.

2. `kill -9`로 ROS 프로세스를 죽이면 `/dev/shm/fastrtps_*` 잠금파일이 남고,
   그러면 다음 SLAM이 `Failed init_port fastrtps_portNNNN`으로 아예 안 뜬다.
   → 살아있는 프로세스가 열고 있는 파일은 건드리지 않고 **고아 파일만** 지운다.
     (`lsof` 없이 `/proc/*/fd` 심볼릭 링크를 직접 확인 — 외부 명령 의존 없음)

3. 런치를 SIGKILL하면 자식들(라이다/SLAM/필터)이 **고아로 남는다.** 실제로
   중복 라이다 노드가 여러 개 쌓여서 `/dev/ttyAMA0`를 두고 경합했다.
   → 런치뿐 아니라 자식 노드들까지 이름으로 전부 훑는다.

사용법:
    python3 tools/mapping_cleanup.py            # 정리만
    python3 tools/mapping_cleanup.py --restart  # 정리 + diff-drive/EKF 재시작
                                                # (odom 누적 상태 초기화)
"""
import argparse
import os
import signal
import subprocess
import sys
import time

# argv[0]의 basename이 이 중 하나면 매핑/자율주행 세션 소속으로 본다.
SESSION_EXECUTABLES = {
    # Nav2
    "controller_server", "planner_server", "bt_navigator",
    "behavior_server", "lifecycle_manager",
    # 탐색
    "explore",
    # 센서/SLAM 파이프라인
    "ldlidar_stl_ros2_node", "async_slam_toolbox_node",
    "scan_filter_node", "static_transform_publisher",
}
# 런치 프로세스는 argv[0]이 python이라 이름으로 못 잡음 — 인자로 판별.
SESSION_LAUNCH_FILES = ("autonomous_mapping.launch.py", "mapping.launch.py")


def find_session_processes():
    """세션 소속 프로세스 [(pid, 설명)] 반환. 자기 자신은 제외."""
    me = os.getpid()
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                args = [a for a in f.read().decode('utf-8', 'replace').split('\0') if a]
        except OSError:
            continue
        if not args:
            continue  # 커널 스레드
        exe = os.path.basename(args[0])
        if exe in SESSION_EXECUTABLES:
            found.append((pid, exe))
        elif any(a.endswith(lf) for a in args for lf in SESSION_LAUNCH_FILES):
            found.append((pid, 'launch'))
    return found


def kill_session(max_rounds=4):
    """세션 프로세스가 0이 될 때까지 반복해서 죽인다.

    한 번에 안 죽는 이유: 런치가 죽으면서 자식을 새로 남기거나, 종료 중인
    프로세스가 잠깐 더 살아있는 경우가 있음. 실제로 1회 kill로는 매번 몇 개씩
    살아남았다.
    """
    for round_no in range(1, max_rounds + 1):
        procs = find_session_processes()
        if not procs:
            return True, round_no - 1
        for pid, _ in procs:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(2)
    return not find_session_processes(), max_rounds


def shm_files_in_use():
    """지금 살아있는 프로세스가 열고 있는 /dev/shm 파일 경로 집합."""
    in_use = set()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        fd_dir = f'/proc/{entry}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue  # 권한 없음(다른 유저) 또는 이미 종료
        for fd in fds:
            try:
                target = os.readlink(f'{fd_dir}/{fd}')
            except OSError:
                continue
            if target.startswith('/dev/shm/'):
                in_use.add(target)
    return in_use


def clean_orphan_shm():
    """아무도 안 쓰는 fastrtps 공유메모리 파일만 지운다. (지운 수, 보존 수)"""
    in_use = shm_files_in_use()
    removed = kept = 0
    try:
        names = os.listdir('/dev/shm')
    except OSError:
        return 0, 0
    for name in names:
        if 'fastrtps' not in name:
            continue
        path = f'/dev/shm/{name}'
        if path in in_use:
            kept += 1
            continue
        try:
            os.remove(path)
            removed += 1
        except OSError:
            kept += 1
    return removed, kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--restart', action='store_true',
                    help='정리 후 diff-drive/EKF 재시작 (누적 odom·yaw 초기화). '
                         'EKF를 같이 안 하면 이전 세션의 yaw를 그대로 들고 시작해서 '
                         '맵 방향이 통째로 틀어진다 — 2026-08-16 실제로 겪음.')
    args = ap.parse_args()

    procs = find_session_processes()
    print(f'세션 프로세스 {len(procs)}개 발견')
    for pid, name in procs:
        print(f'  {pid:>7} {name}')

    ok, rounds = kill_session()
    print(f'종료 {"완료" if ok else "실패(잔존 있음)"} — {rounds}회 반복')
    if not ok:
        for pid, name in find_session_processes():
            print(f'  잔존: {pid} {name}')

    removed, kept = clean_orphan_shm()
    print(f'DDS 공유메모리 — 고아 {removed}개 삭제, 사용중 {kept}개 보존')

    if args.restart:
        units = ['smart-trash-diff-drive.service', 'smart-trash-ekf.service']
        r = subprocess.run(['sudo', 'systemctl', 'restart'] + units,
                           capture_output=True, text=True)
        if r.returncode == 0:
            print('diff-drive/EKF 재시작 완료 (odom 초기화됨)')
        else:
            print(f'재시작 실패: {r.stderr.strip()}', file=sys.stderr)
            return 1

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
