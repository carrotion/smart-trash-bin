#!/usr/bin/env python3
"""성과물(맵/가방/지표/텔레메트리)을 ATS 서버로 올린다.

설계 원칙 — `docs/agent_architecture.md`
----------------------------------------
**로컬 에이전트는 서버를 절대 기다리지 않는다.** 이 프로세스는 로봇 동작과 완전히
분리돼 돌고, 서버가 없거나 죽어 있어도 로봇에는 아무 영향이 없다. 이 로봇은 자주
오프라인이다(주방 WiFi 음영지역이 실재한다) — 그게 예외가 아니라 정상 상태다.

지키는 규칙
-----------
1. **비차단.** 실패는 재시도 큐에 남기고 넘어간다. 절대 로봇 동작을 막지 않는다.
2. **서버가 확인한 뒤에만 로컬에서 치운다.** 가방이 29.5MB/4분이라 카드가 금방
   차지만, 확인 없는 삭제는 성과물 유실이다. 서버가 돌려준 체크섬이 일치해야 지운다.
3. **작은 것 먼저.** 지표/요약(수 KB)을 가방(수십 MB)보다 먼저 올린다. 링크가
   잠깐만 살아 있어도 제일 중요한 게 넘어가도록.
4. **쓰기 예산 안에서.** 업로드하려고 SD에 사본을 만들지 않는다 — 원본을 그대로
   읽어 보내고, 성공하면 `os.replace`(같은 파일시스템 내 이동)로 옮긴다.
5. **신호가 나쁘면 큰 파일은 미룬다.** 음영지역에서 30MB를 붙들고 재시도하는 건
   전력과 시간 낭비다.

디렉터리 구조
-------------
    ~/artifacts/outbox/<종류>/파일     올릴 것 (로봇이 여기에 떨군다)
    ~/artifacts/sent/<종류>/파일       서버 확인 완료 (보존 기간 지나면 삭제)

사용법
------
    python3 tools/artifact_uploader.py --server http://192.168.0.x:8080 --once
    python3 tools/artifact_uploader.py --server http://192.168.0.x:8080   # 상주
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request

# 종류별 우선순위 — 숫자가 작을수록 먼저. 작고 중요한 것부터 올린다.
PRIORITY = {
    'telemetry': 0,   # 일일 요약, 사고 기록 — 수 KB
    'metrics': 1,     # 맵 품질 지표 등 — 수 KB
    'maps': 2,        # pgm+yaml — 수백 KB
    'bags': 3,        # 주행 녹화 — 수십 MB
}
DEFAULT_ROOT = os.path.expanduser('~/artifacts')
# 이보다 크면 신호가 좋을 때만 시도한다(바이트)
LARGE_FILE = 5 * 1024 * 1024


def sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def wifi_quality():
    """0~100. 못 재면 None. 큰 파일 전송 여부 판단에만 쓴다."""
    try:
        out = subprocess.run(['cat', '/proc/net/wireless'], capture_output=True,
                             text=True, timeout=3).stdout
        for line in out.splitlines():
            if ':' in line and not line.strip().startswith('Inter'):
                parts = line.split()
                if len(parts) > 2:
                    # link quality는 보통 0~70 척도
                    return min(100, float(parts[2].rstrip('.')) * 100 / 70)
    except Exception:
        pass
    return None


def unit_id():
    """기체 식별자. 서버가 기체별로 나눠 보관하는 데 쓴다."""
    # /etc/smart-trash/unit.yaml 분리가 끝나면 거기서 읽을 것(fleet 문서 참고).
    return os.environ.get('SMART_TRASH_UNIT') or os.uname().nodename


def collect(root):
    """outbox의 파일을 우선순위 순으로 나열."""
    items = []
    outbox = os.path.join(root, 'outbox')
    if not os.path.isdir(outbox):
        return items
    for kind in os.listdir(outbox):
        d = os.path.join(outbox, kind)
        if not os.path.isdir(d):
            continue
        for name in sorted(os.listdir(d)):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                items.append((PRIORITY.get(kind, 9), kind, p))
            elif os.path.isdir(p):
                # 가방은 디렉터리다 — tar로 묶어 한 덩어리로 보낸다.
                items.append((PRIORITY.get(kind, 9), kind, p))
    items.sort(key=lambda t: (t[0], t[2]))
    return items


def pack_dir(path):
    """디렉터리를 tar로 묶는다. **SD가 아니라 /tmp(tmpfs)에** 만든다 — 카드 수명."""
    base = os.path.basename(path.rstrip('/'))
    dest = os.path.join('/tmp', f'{base}.tar')
    if os.path.exists(dest):
        os.unlink(dest)
    subprocess.run(['tar', '-cf', dest, '-C', os.path.dirname(path), base],
                   check=True)
    return dest


def upload(server, kind, path, timeout):
    """파일 하나 업로드. 서버가 돌려준 체크섬이 일치할 때만 True."""
    digest = sha256(path)
    size = os.path.getsize(path)
    url = (f'{server.rstrip("/")}/upload'
           f'?unit={unit_id()}&kind={kind}'
           f'&name={os.path.basename(path)}&sha256={digest}')
    with open(path, 'rb') as f:
        req = urllib.request.Request(url, data=f, method='POST')
        req.add_header('Content-Type', 'application/octet-stream')
        req.add_header('Content-Length', str(size))
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = json.loads(resp.read().decode('utf-8', 'replace'))
    # 서버가 받은 내용의 해시를 되돌려줘야 한다. 안 맞으면 성공으로 치지 않는다 —
    # "올렸다고 믿고 지웠는데 깨져 있었다"가 제일 나쁜 결말이다.
    if body.get('sha256') != digest:
        raise ValueError(f'체크섬 불일치: 서버 {body.get("sha256")} != 로컬 {digest}')
    return True


def run_once(args):
    items = collect(args.root)
    if not items:
        print('올릴 것 없음')
        return 0
    q = wifi_quality()
    sent = failed = skipped = 0
    for _, kind, path in items:
        is_dir = os.path.isdir(path)
        size = (sum(os.path.getsize(os.path.join(dp, f))
                    for dp, _, fs in os.walk(path) for f in fs)
                if is_dir else os.path.getsize(path))
        if size > LARGE_FILE and q is not None and q < 40:
            print(f'  건너뜀(신호 {q:.0f}%, {size/1e6:.1f}MB): {os.path.basename(path)}')
            skipped += 1
            continue

        packed = None
        try:
            target = pack_dir(path) if is_dir else path
            packed = target if is_dir else None
            upload(args.server, kind, target, args.timeout)
        except (urllib.error.URLError, OSError, ValueError, TimeoutError,
                subprocess.CalledProcessError) as e:
            print(f'  실패({type(e).__name__}): {os.path.basename(path)} — {e}')
            failed += 1
            continue
        finally:
            if packed and os.path.exists(packed):
                os.unlink(packed)

        # 서버 확인됨 — 이제서야 로컬에서 치운다
        dest_dir = os.path.join(args.root, 'sent', kind)
        os.makedirs(dest_dir, exist_ok=True)
        dest = os.path.join(dest_dir, os.path.basename(path.rstrip('/')))
        if os.path.exists(dest):
            shutil.rmtree(dest, ignore_errors=True) if os.path.isdir(dest) else os.unlink(dest)
        os.replace(path, dest)      # 같은 파일시스템 내 이동 = 추가 쓰기 없음
        print(f'  완료: {kind}/{os.path.basename(path)} ({size/1e6:.2f}MB)')
        sent += 1

    print(f'전송 {sent}, 실패 {failed}, 보류 {skipped}'
          + (f'  (WiFi {q:.0f}%)' if q is not None else ''))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--server', required=True, help='예: http://192.168.0.50:8080')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--interval', type=float, default=300.0)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--once', action='store_true')
    args = ap.parse_args()

    for kind in PRIORITY:
        os.makedirs(os.path.join(args.root, 'outbox', kind), exist_ok=True)

    if args.once:
        return run_once(args)
    while True:
        try:
            run_once(args)
        except Exception as e:      # 어떤 예외도 이 루프를 죽이면 안 된다
            print(f'주기 실행 중 예외(무시하고 계속): {type(e).__name__}: {e}')
        time.sleep(args.interval)


if __name__ == '__main__':
    sys.exit(main())
