#!/usr/bin/env python3
"""프로세스를 이름으로 안전하게 종료한다. `pkill -f` 대신 이걸 쓸 것.

왜 필요한가
-----------
`pkill -f <패턴>`은 **자기 자신을 죽인다.** 패턴이 명령줄 전체를 훑는데,
그 명령을 실행 중인 셸의 명령줄에도 같은 문자열이 들어있기 때문이다.
2026-08-17 하루에만 여러 번 셸이 exit 144로 죽었다(SIGTERM=15, 128+15+1).
게다가 부모 셸이 죽으면 그 뒤 명령이 통째로 날아가서, 정작 하려던 정리 작업이
중간에 끊긴 채 남는다 — 조용히 실패하는 게 제일 나쁘다.

이 도구는:
  - **자기 자신과 모든 조상 프로세스**를 후보에서 제외한다
  - 명령줄 부분일치가 아니라 **실행파일 이름 단위**로 맞춘다.
    파이썬 노드는 argv[0]이 python3라서 argv[1]의 파일명도 함께 본다
  - 죽인 프로세스를 이름과 함께 보고한다(뭘 죽였는지 모르는 상태를 안 만든다)

사용법
------
    python3 tools/killproc.py mag_calibrate            # 이름 하나
    python3 tools/killproc.py explore controller_server  # 여러 개
    python3 tools/killproc.py --dry-run explore        # 죽일 목록만 확인
    python3 tools/killproc.py --kill9 explore          # TERM 후에도 남으면 KILL
"""

import argparse
import os
import signal
import sys
import time


def _cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode('utf-8', 'replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return []


def _ppid(pid):
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('PPid:'):
                    return int(line.split()[1])
    except (OSError, ValueError):
        pass
    return 0


def _ancestors(pid):
    """자기 자신부터 PID 1까지의 조상 집합. 이 안에 든 건 절대 죽이지 않는다."""
    out = set()
    cur = pid
    while cur > 1 and cur not in out:
        out.add(cur)
        cur = _ppid(cur)
    out.add(1)
    return out


def process_name(args):
    """프로세스를 대표하는 이름. python 래퍼는 벗겨내고 스크립트 이름을 쓴다."""
    if not args:
        return ''
    base = os.path.basename(args[0])
    if base.startswith('python') and len(args) > 1:
        for a in args[1:]:
            if not a.startswith('-'):
                return os.path.basename(a)
    return base


# 이 이름들은 **거의 모든 ROS 프로세스에 매칭된다.** systemd 서비스는 전부
# `ros2 run ...`으로 뜨는데 `ros2` 자체가 파이썬 스크립트라 실질 이름이 `ros2`가 된다.
# 2026-08-17에 `killproc.py ros2 ...`를 실행했다가 구동/EKF 서비스가 통째로 죽었고,
# 그걸 모른 채 데이터 녹화를 시작해서 조종해도 로봇이 안 움직였다.
DANGEROUS_NAMES = {'ros2', 'python', 'python3', 'bash', 'sh', 'systemd', 'launch'}


def _service_of(pid):
    """systemd 시스템 서비스에 속한 프로세스면 서비스 이름, 아니면 None."""
    try:
        with open(f'/proc/{pid}/cgroup') as f:
            for line in f:
                if '/system.slice/' in line:
                    unit = line.strip().rsplit('/', 1)[-1]
                    if unit.endswith('.service'):
                        return unit
    except OSError:
        pass
    return None


def find(names, include_services=False, skipped=None):
    protect = _ancestors(os.getpid())
    hits = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protect:
            continue
        args = _cmdline(pid)
        if not args:
            continue
        pname = process_name(args)
        for want in names:
            # 실행파일/스크립트 이름 단위 매칭. 확장자는 있어도 되고 없어도 된다.
            if pname == want or pname == want + '.py' or pname.rsplit('.', 1)[0] == want:
                # ⚠️ 2026-09-26: 임시로 띄운 static_transform_publisher를 끄려다 같은 실행파일을
                # 쓰는 smart-trash-imu-tf 서비스까지 죽였다 → EKF가 IMU를 버리고(TF 없음)
                # 정사각형 시험에서 방향 -72° 오차. 서비스 소속은 systemctl로만 다룬다.
                unit = _service_of(pid)
                if unit and not include_services:
                    if skipped is not None:
                        skipped.append((pid, pname, unit))
                    break
                hits.append((pid, pname, ' '.join(args)[:100]))
                break
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='+', help='실행파일 또는 스크립트 이름 (경로/패턴 아님)')
    ap.add_argument('--dry-run', action='store_true', help='죽이지 않고 목록만 출력')
    ap.add_argument('--kill9', action='store_true', help='TERM 후 남으면 KILL까지')
    ap.add_argument('--timeout', type=float, default=5.0, help='종료 대기 초')
    ap.add_argument('--include-services', action='store_true',
                    help='systemd 서비스 소속 프로세스도 대상에 포함(보통은 systemctl stop을 쓸 것)')
    ap.add_argument('--allow-dangerous', action='store_true',
                    help='ros2/python3 같은 포괄적 이름도 허용 (거의 항상 실수다)')
    args = ap.parse_args()

    bad = [n for n in args.names if n in DANGEROUS_NAMES]
    if bad and not args.allow_dangerous:
        print(f'❌ 너무 포괄적인 이름이다: {", ".join(bad)}')
        print('   ROS 노드는 전부 `ros2 run ...`으로 뜨므로 이 이름은 구동/EKF/센서')
        print('   서비스까지 싹 죽인다. 실제 노드 이름을 지정할 것.')
        print('   예: killproc.py diff_drive_node   (X: killproc.py ros2)')
        print('   정말 의도한 것이면 --allow-dangerous')
        return 2

    skipped = []
    hits = find(args.names, args.include_services, skipped)
    for pid, pname, unit in skipped:
        print(f'  ⏭  {pid:>7}  {pname:<24} — {unit} 서비스 소속이라 건너뜀(systemctl로 다룰 것, 강제는 --include-services)')
    if not hits:
        print(f'해당 프로세스 없음: {", ".join(args.names)}')
        return 0

    for pid, pname, cmd in hits:
        print(f'  {"[dry-run] " if args.dry_run else ""}{pid:>7}  {pname:<24} {cmd}')
    if args.dry_run:
        return 0

    for pid, _, _ in hits:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if not find(args.names, args.include_services):
            print(f'{len(hits)}개 종료 완료')
            return 0
        time.sleep(0.2)

    left = find(args.names, args.include_services)
    if args.kill9:
        for pid, _, _ in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(0.5)
        left = find(args.names, args.include_services)

    if left:
        print(f'⚠️ {len(left)}개가 아직 살아있다: {[p for p, _, _ in left]}')
        return 1
    print('종료 완료')
    return 0


if __name__ == '__main__':
    sys.exit(main())
