#!/usr/bin/env python3
"""프리즈 순간의 증거를 microSD에 남기는 블랙박스 기록기.

**왜 필요한가 (2026-08-17 01:07 프리즈)**

시스템이 얼어붙었는데 **증거가 통째로 사라졌다.** 이유는 단순하다 — 커널이
에러 메시지를 쓰려는 대상이 바로 그 죽은 디스크(USB SSD)였기 때문이다.
그래서 10시간 세션 내내 USB 관련 커널 메시지가 단 한 줄도 안 남았고,
저널은 아무 에러 없이 그냥 끊겼다.

microSD는 USB가 아니라 SoC 내장 SDIO에 직결된다. **USB 링크가 죽어도 SD는
살아있다.** netconsole과 달리 WiFi에도 의존하지 않으므로, WiFi 음영지역인
주방에서 사고가 나도 확실히 기록된다.

**기록하는 것**

1. 하트비트(1초): 마지막 줄의 시각이 곧 **프리즈가 일어난 정확한 시점**.
2. D 상태(중단 불가 대기) 프로세스 수: 디스크가 죽어갈 때 여기 쌓인다.
   **이게 가장 중요한 조기 신호** — 0에서 갑자기 늘어나는 순간이 발단이다.
   어떤 프로세스가 어디서 막혔는지 커널 스택(wchan)까지 같이 남긴다.
   ⚠️ 단, D 상태는 정상 운영 중에도 순간적으로 나타난다(jbd2 저널 커밋,
   vcgencmd 펌웨어 호출 등). 그대로 다 기록하면 진짜 신호가 묻히므로,
   **같은 프로세스가 SUSTAINED_SEC 이상 계속 막혀 있을 때만** 상세를 남긴다.
3. 커널 링버퍼(/dev/kmsg) 실시간 전달: 디스크 워치독이 재부팅 직전에 찍는
   sysrq 'w'(블록된 태스크 스택 덤프)가 여기로 들어온다.
4. 블록/USB 계층 스냅샷(10초): sda 통계, USB 장치 상태.

**설계상 지켜야 하는 것**

이 프로세스는 사고 중에도 살아있어야 하므로 **SSD를 절대 건드리면 안 된다.**
- 쓰기는 오직 /var/log/blackbox(SD)로만.
- 로그를 stdout으로 내보내지 않는다(journald -> SSD로 이어짐).
- 읽는 곳은 /proc, /sys, /dev/kmsg 뿐 — 전부 메모리 기반이라 안 막힌다.
- 하트비트는 주기적으로 fsync 한다. 안 하면 페이지 캐시에만 남아서
  강제 리셋 시 마지막 몇 초가 통째로 날아간다(그 몇 초가 제일 중요하다).

사용법:
    python3 tools/blackbox_recorder.py [--dir /var/log/blackbox]
"""
import argparse
import os
import re
import sys
import time

BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"

# 이 시간(초) 이상 연속으로 D 상태인 프로세스만 "비정상"으로 본다.
# 정상적인 저널 커밋/펌웨어 호출은 1초 안에 풀린다.
SUSTAINED_SEC = 5.0


def unescape_kmsg(text):
    r"""커널은 /dev/kmsg로 내보낼 때 비ASCII 바이트를 \xNN으로 이스케이프한다.
    한글 메시지(디스크 워치독이 남기는 것 등)가 깨져 보이므로 되돌린다."""
    if "\\x" not in text:
        return text
    raw = re.sub(r"\\x([0-9a-fA-F]{2})",
                 lambda m: chr(int(m.group(1), 16)), text)
    try:
        return raw.encode("latin-1").decode("utf-8")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return text


def read_text(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def blocked_tasks():
    """D 상태(중단 불가 대기) 프로세스 목록. (pid, comm, wchan) 튜플.

    wchan = 커널 안에서 무엇을 기다리는지. 디스크가 죽으면 여기에
    io_schedule 계열이 잔뜩 찍힌다 — 원인 규명의 핵심 단서.
    """
    out = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        stat = read_text(f"/proc/{entry}/stat")
        if not stat:
            continue
        # comm이 괄호로 감싸여 있고 공백을 포함할 수 있어 뒤에서부터 파싱
        try:
            rparen = stat.rindex(")")
            comm = stat[stat.index("(") + 1:rparen]
            state = stat[rparen + 2]
        except ValueError:
            continue
        if state == "D":
            wchan = read_text(f"/proc/{entry}/wchan", "?").strip() or "?"
            out.append((entry, comm, wchan))
    return out


# 루트 디스크. 2026-08-17 microSD로 이관하면서 sda -> mmcblk0.
ROOT_DISK = "mmcblk0"


def disk_stats():
    """루트 디스크의 진행 중 I/O와 누적 쓰기량.

    `written_mb`가 중요한 이유: microSD는 SMART가 없어서(life_time/pre_eol_info
    미노출) 수명을 직접 못 읽는다. 대신 **하루에 몇 GB를 쓰는지**를 추적하면
    카드가 얼마나 빨리 닳는지 가늠할 수 있다 — SSD를 뗀 뒤 잃은 SMART 감시를
    대신하는 유일한 실용적 지표다.
    """
    line = read_text(f"/sys/block/{ROOT_DISK}/stat").split()
    if len(line) >= 11:
        # 필드 7 = 누적 기록 섹터(512B), 9 = 진행 중 I/O, 10 = I/O에 쓴 시간(ms)
        written_mb = int(line[6]) * 512 // (1024 * 1024)
        return f"inflight={line[8]} io_ms={line[9]} written_mb={written_mb}"
    return "n/a"


def usb_snapshot():
    parts = []
    for dev in sorted(os.listdir("/sys/bus/usb/devices")):
        base = f"/sys/bus/usb/devices/{dev}"
        product = read_text(f"{base}/product").strip()
        if not product:
            continue
        parts.append(f"{dev}:{product}"
                     f"[{read_text(f'{base}/power/runtime_status').strip()}]")
    return " ".join(parts)


class Sink:
    """SD에만 쓰는 파일. 용량이 차면 갈아엎어 계속 돌게 한다."""

    def __init__(self, path, max_bytes):
        self.path = path
        self.max_bytes = max_bytes
        self.fh = open(path, "a", buffering=1)   # 줄 단위 버퍼링

    def write(self, line):
        try:
            self.fh.write(line + "\n")
        except OSError:
            return
        if self.fh.tell() > self.max_bytes:
            self.rotate()

    def rotate(self):
        try:
            self.fh.close()
            os.replace(self.path, self.path + ".1")
            self.fh = open(self.path, "a", buffering=1)
        except OSError:
            pass

    def sync(self):
        try:
            self.fh.flush()
            os.fsync(self.fh.fileno())
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/var/log/blackbox")
    ap.add_argument("--heartbeat-interval", type=float, default=1.0)
    ap.add_argument("--snapshot-interval", type=float, default=10.0)
    ap.add_argument("--sync-interval", type=float, default=2.0)
    ap.add_argument("--max-bytes", type=int, default=256 * 1024 * 1024)
    args = ap.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    boot_id = read_text(BOOT_ID_PATH).strip()[:8]

    hb = Sink(os.path.join(args.dir, "heartbeat.log"), args.max_bytes)
    kern = Sink(os.path.join(args.dir, "kernel.log"), args.max_bytes)

    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    for s in (hb, kern):
        s.write(f"===== 기록 시작 {stamp} boot={boot_id} =====")
    hb.sync()
    kern.sync()

    # /dev/kmsg를 논블로킹으로 따라간다. 이미 쌓인 건 건너뛰고 새 메시지만.
    kmsg_fd = None
    try:
        kmsg_fd = os.open("/dev/kmsg", os.O_RDONLY | os.O_NONBLOCK)
        os.lseek(kmsg_fd, 0, os.SEEK_END)
    except OSError:
        kern.write("!! /dev/kmsg 열기 실패 — 커널 메시지 기록 불가")

    last_snapshot = 0.0
    last_sync = 0.0
    blocked_since = {}      # pid -> 언제부터 연속으로 D 상태인지
    warned = False

    while True:
        now = time.time()
        iso = time.strftime("%H:%M:%S", time.localtime(now))

        # --- 커널 메시지 먼저 (사고 순간 가장 중요한 정보) ---
        if kmsg_fd is not None:
            while True:
                try:
                    chunk = os.read(kmsg_fd, 8192)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if not chunk:
                    break
                for raw in chunk.decode("utf-8", "replace").splitlines():
                    if not raw.strip():
                        continue
                    # "<prio>,<seq>,<usec>,<flag>;<text>" 형식에서 본문만
                    text = unescape_kmsg(raw.split(";", 1)[-1])
                    kern.write(f"{iso} {text}")

        # --- 하트비트 ---
        blocked = blocked_tasks()
        loadavg = read_text("/proc/loadavg").split()[:3]

        # 연속으로 막혀 있는 시간을 추적한다(순간적인 D 상태는 걸러내기 위해)
        seen = {pid for pid, _, _ in blocked}
        for pid in list(blocked_since):
            if pid not in seen:
                del blocked_since[pid]
        for pid, _, _ in blocked:
            blocked_since.setdefault(pid, now)

        sustained = [(pid, comm, wchan) for pid, comm, wchan in blocked
                     if now - blocked_since.get(pid, now) >= SUSTAINED_SEC]

        hb.write(f"{now:.1f} {iso} load={','.join(loadavg)} "
                 f"D={len(blocked)} stuck={len(sustained)} {disk_stats()}")

        # 오래 막힌 것만 상세 기록 — 여기 뭔가 찍히면 그게 사고 시작점이다
        for pid, comm, wchan in sustained[:12]:
            held = now - blocked_since.get(pid, now)
            hb.write(f"    STUCK {held:.0f}s pid={pid} comm={comm} wchan={wchan}")

        if sustained and not warned:
            kern.write(f"{iso} !! {len(sustained)}개 프로세스가 "
                       f"{SUSTAINED_SEC:.0f}초 이상 D 상태 — 디스크 정지 징후")
            warned = True
        elif not sustained and warned:
            kern.write(f"{iso} -- D 상태 해소, 정상 복귀")
            warned = False

        # --- 주기 스냅샷 ---
        if now - last_snapshot >= args.snapshot_interval:
            hb.write(f"    USB {usb_snapshot()}")
            last_snapshot = now

        # --- 강제 동기화: 마지막 몇 초를 잃지 않도록 ---
        # 지속 정지가 보이면 사고 진행 중이므로 매 주기 동기화한다.
        if sustained or now - last_sync >= args.sync_interval:
            hb.sync()
            kern.sync()
            last_sync = now

        time.sleep(args.heartbeat_interval)


if __name__ == "__main__":
    sys.exit(main())
