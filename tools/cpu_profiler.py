#!/usr/bin/env python3
"""순간 CPU%를 정확히 잰다 — `ps -o pcpu`는 프로세스 시작 이후 누적평균이라
0.5초 간격 스파이크를 못 잡는다(2026-08-23 이걸로 헛다리 짚었음).
/proc/<pid>/stat의 utime+stime을 매 샘플 사이 델타로 계산해야 순간값이 나온다.

사용법:
    python3 tools/cpu_profiler.py --out cpu.csv --duration 150 --interval 0.2 \\
        --match stage_ros2 async_slam_tool controller_server planner_server \\
                behavior_server bt_navigator frontier_explor explore lifecycle_manager
"""
import argparse
import csv
import os
import time

CLK_TCK = os.sysconf('SC_CLK_TCK')


def find_pids(patterns):
    out = {}
    for pid_dir in os.listdir('/proc'):
        if not pid_dir.isdigit():
            continue
        try:
            with open(f'/proc/{pid_dir}/comm') as f:
                comm = f.read().strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        for pat in patterns:
            if pat in comm:
                out[int(pid_dir)] = comm
                break
    return out


def read_cpu_ticks(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().split(')')[-1].split()
        # comm은 괄호 안에 있고 공백을 포함할 수 있어 ')' 뒤부터 다시 센다.
        # utime=필드14, stime=필드15 (1-based, pid/comm/state 제외하고 ')' 다음
        # 부터 세면 utime은 인덱스 11, stime은 인덱스 12 (0-based)
        utime = int(fields[11])
        stime = int(fields[12])
        return utime + stime
    except (FileNotFoundError, ProcessLookupError, IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, required=True)
    ap.add_argument('--interval', type=float, default=0.2)
    ap.add_argument('--match', nargs='+', required=True,
                     help='comm 부분일치 패턴들')
    args = ap.parse_args()

    f = open(args.out, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['wall_epoch', 'comm', 'pid', 'cpu_pct'])
    f.flush()

    prev_ticks = {}
    prev_time = time.time()
    start = prev_time
    end = start + args.duration

    print(f'[cpu_profiler] {args.match} 추적 시작 -> {args.out} '
          f'(간격 {args.interval}s, 기간 {args.duration}s)', flush=True)

    while time.time() < end:
        time.sleep(args.interval)
        now = time.time()
        dt = now - prev_time
        pids = find_pids(args.match)
        for pid, comm in pids.items():
            ticks = read_cpu_ticks(pid)
            if ticks is None:
                continue
            key = pid
            if key in prev_ticks:
                dticks = ticks - prev_ticks[key]
                cpu_pct = 100.0 * dticks / (CLK_TCK * dt) if dt > 0 else 0.0
                writer.writerow([f'{now:.3f}', comm, pid, f'{cpu_pct:.1f}'])
            prev_ticks[key] = ticks
        f.flush()
        prev_time = now

    f.close()
    print(f'[cpu_profiler] 종료: {args.out}', flush=True)


if __name__ == '__main__':
    main()
