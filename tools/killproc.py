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


def find(names):
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
                hits.append((pid, pname, ' '.join(args)[:100]))
                break
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='+', help='실행파일 또는 스크립트 이름 (경로/패턴 아님)')
    ap.add_argument('--dry-run', action='store_true', help='죽이지 않고 목록만 출력')
    ap.add_argument('--kill9', action='store_true', help='TERM 후 남으면 KILL까지')
    ap.add_argument('--timeout', type=float, default=5.0, help='종료 대기 초')
    args = ap.parse_args()

    hits = find(args.names)
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
        if not find(args.names):
            print(f'{len(hits)}개 종료 완료')
            return 0
        time.sleep(0.2)

    left = find(args.names)
    if args.kill9:
        for pid, _, _ in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(0.5)
        left = find(args.names)

    if left:
        print(f'⚠️ {len(left)}개가 아직 살아있다: {[p for p, _, _ in left]}')
        return 1
    print('종료 완료')
    return 0


if __name__ == '__main__':
    sys.exit(main())
