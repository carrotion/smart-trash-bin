#!/usr/bin/env python3
"""Pi 5 전원(저전압/스로틀링) 실시간 모니터링 웹 대시보드.

vcgencmd get_throttled / pmic_read_adc를 주기적으로 폴링해서
브라우저에서 실시간으로 상태를 볼 수 있게 함. ROS2 노드가 아니라
하드웨어 배선/전원부 검증용 독립 진단 도구라 catkin_ws 밖에 둠.

사용법:
    python3 tools/power_monitor.py [--port 8765] [--interval 1.0]
    이후 브라우저에서 http://<Pi IP>:8765 접속

vcgencmd를 sudo 없이 쓰려면 carrotian이 video 그룹에 있어야 함
(CLAUDE.md 참고, 2026-08-13 usermod -aG video로 추가함).
"""
import argparse
import json
import re
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

THROTTLED_BITS = {
    0: "under_voltage_now",
    1: "freq_capped_now",
    2: "throttled_now",
    3: "soft_temp_limit_now",
    16: "under_voltage_since_boot",
    17: "freq_capped_since_boot",
    18: "throttled_since_boot",
    19: "soft_temp_limit_since_boot",
}

LABELS_KR = {
    "under_voltage_now": "지금 저전압",
    "freq_capped_now": "지금 클럭 제한",
    "throttled_now": "지금 스로틀링",
    "soft_temp_limit_now": "지금 온도제한",
    "under_voltage_since_boot": "부팅 후 저전압 발생 이력",
    "freq_capped_since_boot": "부팅 후 클럭제한 이력",
    "throttled_since_boot": "부팅 후 스로틀링 이력",
    "soft_temp_limit_since_boot": "부팅 후 온도제한 이력",
}

LOG_PATH = "/home/carrotian/power_monitor.log"
HISTORY_LEN = 600

state_lock = threading.Lock()
history = deque(maxlen=HISTORY_LEN)
latest = {}


def run_vcgencmd(args):
    try:
        out = subprocess.run(
            ["vcgencmd"] + args, capture_output=True, text=True, timeout=2)
        return out.stdout.strip()
    except Exception as e:
        return f"ERROR: {e}"


def parse_throttled(raw):
    m = re.search(r"0x([0-9a-fA-F]+)", raw)
    if not m:
        return {"raw": raw, "flags": {}, "bitmask": None}
    bitmask = int(m.group(1), 16)
    flags = {name: bool(bitmask & (1 << bit)) for bit, name in THROTTLED_BITS.items()}
    return {"raw": raw, "flags": flags, "bitmask": bitmask}


def parse_pmic_adc(raw):
    rails = {}
    for line in raw.splitlines():
        m = re.match(r"\s*(\S+)\s+(current|volt)\(\d+\)=([\d.]+)([AV])", line)
        if m:
            label, kind, value, unit = m.groups()
            rails[label] = {"value": float(value), "unit": unit}
    return rails


def log_line(text):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {text}"
    print(line, flush=True)
    try:
        with open(LOG_PATH, "a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def poll_loop(interval):
    prev_flags = None
    while True:
        throttled = parse_throttled(run_vcgencmd(["get_throttled"]))
        rails = parse_pmic_adc(run_vcgencmd(["pmic_read_adc"]))
        sample = {"ts": time.time(), "throttled": throttled, "rails": rails}

        with state_lock:
            latest.clear()
            latest.update(sample)
            history.append(sample)

        flags = throttled["flags"]
        if prev_flags is not None:
            for name, val in flags.items():
                if val and not prev_flags.get(name):
                    log_line(f"⚠ {LABELS_KR.get(name, name)} 시작")
                elif not val and prev_flags.get(name):
                    log_line(f"✅ {LABELS_KR.get(name, name)} 해제")
        prev_flags = flags
        time.sleep(interval)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>Pi5 전원 모니터</title>
<style>
  :root { color-scheme: light dark; }
  body { font-family: -apple-system, sans-serif; max-width: 720px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; }
  .grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin: 16px 0; }
  .card { border: 1px solid #8884; border-radius: 8px; padding: 10px 12px; font-size: 0.9rem; }
  .ok { background: #1a7f371a; border-color: #1a7f3766; }
  .bad { background: #c832321a; border-color: #c8323266; }
  table { width: 100%; border-collapse: collapse; font-size: 0.85rem; margin-top: 8px; }
  td, th { text-align: left; padding: 3px 6px; border-bottom: 1px solid #8882; }
  #log { font-family: monospace; font-size: 0.8rem; white-space: pre-wrap; max-height: 200px;
         overflow-y: auto; border: 1px solid #8884; border-radius: 8px; padding: 8px; }
  .stale { opacity: 0.4; }
</style>
</head>
<body>
<h1>Pi5 전원 상태 <span id="updated" style="font-size:0.7rem; font-weight:normal;"></span></h1>
<div class="grid" id="flags"></div>
<h2 style="font-size:1rem;">PMIC 레일 전압/전류</h2>
<table id="rails"></table>
<h2 style="font-size:1rem;">이벤트 로그</h2>
<div id="log"></div>
<script>
const flagOrder = ["under_voltage_now","throttled_now","freq_capped_now","soft_temp_limit_now",
                    "under_voltage_since_boot","throttled_since_boot","freq_capped_since_boot","soft_temp_limit_since_boot"];
const labels = {
  under_voltage_now: "지금 저전압", throttled_now: "지금 스로틀링",
  freq_capped_now: "지금 클럭제한", soft_temp_limit_now: "지금 온도제한",
  under_voltage_since_boot: "저전압 이력", throttled_since_boot: "스로틀링 이력",
  freq_capped_since_boot: "클럭제한 이력", soft_temp_limit_since_boot: "온도제한 이력",
};
let lastLog = [];

async function tick() {
  let res;
  try { res = await (await fetch("/api/status")).json(); }
  catch (e) { document.getElementById("updated").textContent = "(연결 끊김)"; return; }

  const flags = res.latest.throttled ? res.latest.throttled.flags : {};
  const grid = document.getElementById("flags");
  grid.innerHTML = "";
  for (const key of flagOrder) {
    const bad = !!flags[key];
    const div = document.createElement("div");
    div.className = "card " + (bad ? "bad" : "ok");
    div.textContent = (bad ? "🔴 " : "🟢 ") + labels[key];
    grid.appendChild(div);
  }

  const rails = res.latest.rails || {};
  const tbl = document.getElementById("rails");
  let html = "<tr><th>레일</th><th>값</th></tr>";
  for (const [name, info] of Object.entries(rails)) {
    html += `<tr><td>${name}</td><td>${info.value.toFixed(3)} ${info.unit}</td></tr>`;
  }
  tbl.innerHTML = html;

  const ts = res.latest.ts ? new Date(res.latest.ts * 1000).toLocaleTimeString() : "-";
  document.getElementById("updated").textContent = "(마지막 갱신: " + ts + ")";
}

async function tickLog() {
  try {
    const res = await (await fetch("/api/log")).json();
    document.getElementById("log").textContent = res.lines.join("\\n") || "(아직 이벤트 없음, 정상)";
    document.getElementById("log").scrollTop = 1e9;
  } catch (e) {}
}

tick(); tickLog();
setInterval(tick, 1000);
setInterval(tickLog, 3000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            with state_lock:
                body = json.dumps({
                    "latest": latest,
                    "history": list(history)[-120:],
                }).encode("utf-8")
            self._send(200, body, "application/json")
        elif self.path == "/api/log":
            try:
                with open(LOG_PATH) as f:
                    lines = f.readlines()[-100:]
            except OSError:
                lines = []
            body = json.dumps({"lines": [l.rstrip("\n") for l in lines]}).encode("utf-8")
            self._send(200, body, "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()

    t = threading.Thread(target=poll_loop, args=(args.interval,), daemon=True)
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    log_line(f"전원 모니터링 웹서버 시작: http://<Pi IP>:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
