#!/usr/bin/env python3
"""디스크 쓰기가 막히면 자동으로 강제 재부팅한다.

**왜 필요한가 (2026-08-17 01:07 사고)**

자율주행 중 시스템이 얼어붙었는데, systemd의 하드웨어 워치독
(`RuntimeWatchdogSec=10s`)이 **24분 동안 끝내 안 터졌다.** 이유는 이렇다:

- 블록 디바이스(USB SSD)가 응답을 멈춤 -> 파일시스템을 건드린 모든 프로세스가
  D 상태(중단 불가 대기)로 들어가 영영 안 돌아옴
- 그런데 커널과 systemd(PID 1)는 멀쩡히 살아있음 -> 워치독을 계속 쓰다듬음
  -> 하드웨어 워치독은 **이 고장을 원리적으로 못 잡는다**
- journald / power_monitor / 맵 자동저장이 전부 멈추고, SSH도 못 붙고,
  전원버튼조차 안 먹음(종료 절차가 디스크에서 막힘)

결국 사용자가 전원을 강제로 끊어야 했고, **그 하드 전원차단이 ext4 메타데이터를
손상시켰다**(`deleted inode referenced`). 즉 프리즈 자체보다 강제종료가 더 큰
피해를 만들었다. 이 워치독은 그 강제종료를 없애는 게 목적이다.

**어떻게 동작하나**

주기적으로 "쓰기가 살아있는지"를 확인한다. 핵심은 **검사를 자식 프로세스에서
한다**는 것 — 디스크가 죽으면 쓰는 쪽은 D 상태로 빠져 시그널로도 못 죽이므로,
부모가 직접 쓰면 부모까지 같이 죽어서 아무것도 못 한다. 부모는 자식을
timeout으로 기다리기만 하므로 절대 안 막힌다.

연속 실패가 쌓이면 `/proc/sysrq-trigger`로 즉시 재부팅한다. 일반 `reboot`는
디스크 동기화를 하려다 같이 막히므로 쓸 수 없다. sysrq 'b'는 커널이 곧바로
리셋하는 경로라 디스크가 죽어 있어도 동작한다.

재부팅 직전에 sysrq 'w'(블록된 태스크 덤프)를 찍어둔다 — 커널 링버퍼(메모리)에
남으므로 netconsole이 살아있으면 그 순간의 스택을 받을 수 있다.

**⚠️ 이 서비스는 파이를 재부팅시킬 수 있다.** 오탐을 막으려고 임계값을 아주
넉넉히 잡았다(4KB 쓰기+fsync에 20초 타임아웃 = 정상의 수천 배, 게다가 3회 연속).
끄려면: `sudo systemctl disable --now smart-trash-disk-watchdog`

사용법:
    python3 tools/disk_hang_watchdog.py [--interval 20] [--timeout 20]
                                        [--failures 3] [--dry-run]
"""
import argparse
import os
import sys
import time

# 검사용 파일. /var/tmp는 tmpfs가 아니라 실제 루트 파일시스템이어야 한다
# (/tmp가 tmpfs면 디스크가 죽어도 쓰기가 성공해서 감지를 못 한다).
PROBE_PATH = "/var/lib/smart-trash-disk-watchdog/probe"


def kmsg(text):
    """커널 링버퍼에 남긴다. 디스크를 안 거치므로 디스크가 죽어도 동작하고,
    netconsole이 붙어 있으면 그대로 외부로 흘러나간다."""
    try:
        with open("/dev/kmsg", "w") as f:
            f.write(f"smart-trash-disk-watchdog: {text}\n")
    except OSError:
        pass


def probe_once(path):
    """자식 프로세스에서 실행: 쓰기 + fsync 까지 성공해야 0으로 종료."""
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        try:
            os.write(fd, f"{time.time()}\n".encode())
            os.fsync(fd)          # 여기서 막히는 게 핵심 — 캐시가 아니라 실제 저장
        finally:
            os.close(fd)
        os._exit(0)
    except BaseException:
        os._exit(1)


def check_disk(path, timeout):
    """쓰기가 timeout 안에 끝나면 True.

    자식이 D 상태로 굳으면 부모는 그냥 기다리다 포기한다(죽일 수 없고, 죽일
    필요도 없다 — 어차피 곧 재부팅한다). 부모는 디스크를 안 건드리므로 안전.
    """
    pid = os.fork()
    if pid == 0:
        probe_once(path)

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            done, status = os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            return False
        if done == pid:
            return os.waitstatus_to_exitcode(status) == 0
        time.sleep(0.25)
    return False   # 시간 초과 = 디스크가 안 받아줌


def emergency_reboot(dry_run):
    """디스크를 건드리지 않는 경로로 즉시 재부팅."""
    kmsg("디스크 쓰기 불능 확정 — 블록된 태스크 덤프 후 강제 재부팅")
    try:
        # 'w' = 블록된(D 상태) 태스크 스택 덤프. 링버퍼에만 남는다.
        with open("/proc/sysrq-trigger", "w") as f:
            f.write("w")
        time.sleep(2)
    except OSError as e:
        kmsg(f"sysrq w 실패: {e}")

    if dry_run:
        kmsg("[dry-run] 여기서 재부팅했을 것 (실제로는 안 함)")
        print("[dry-run] 재부팅 지점 — 실제 재부팅은 생략", flush=True)
        return

    try:
        with open("/proc/sysrq-trigger", "w") as f:
            f.write("b")          # 즉시 리셋. sync/umount 안 함(하면 같이 막힘)
    except OSError as e:
        kmsg(f"sysrq b 실패: {e} — 마지막 수단으로 reboot -f 시도")
        os.system("reboot -f")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=20.0, help="검사 주기(초)")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="쓰기 한 번을 기다려주는 최대 시간(초)")
    ap.add_argument("--failures", type=int, default=3,
                    help="연속 몇 번 실패하면 재부팅할지")
    ap.add_argument("--dry-run", action="store_true",
                    help="재부팅 대신 로그만 남김 (검증용)")
    ap.add_argument("--probe-path", default=PROBE_PATH,
                    help="검사용 파일 경로. 검증할 때 일부러 쓸 수 없는 경로를 "
                         "줘서 실패 경로를 시험하는 용도로도 쓴다.")
    args = ap.parse_args()

    probe_path = args.probe_path
    try:
        os.makedirs(os.path.dirname(probe_path), exist_ok=True)
    except OSError:
        pass   # 검증용으로 일부러 못 만드는 경로를 준 경우

    kmsg(f"시작 — 주기 {args.interval:.0f}s, 타임아웃 {args.timeout:.0f}s, "
         f"연속 {args.failures}회 실패 시 재부팅"
         + (" [dry-run]" if args.dry_run else ""))

    consecutive = 0
    while True:
        ok = check_disk(probe_path, args.timeout)
        if ok:
            if consecutive:
                kmsg(f"디스크 쓰기 복구됨 (연속 실패 {consecutive}회에서 회복)")
            consecutive = 0
        else:
            consecutive += 1
            kmsg(f"디스크 쓰기 실패/시간초과 {consecutive}/{args.failures}")
            if consecutive >= args.failures:
                emergency_reboot(args.dry_run)
                if args.dry_run:
                    consecutive = 0     # dry-run은 계속 관찰
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main())
