#!/usr/bin/env python3
"""tools/lidar_watchdog.py 시험 — ROS·라이다 없이 (2026-11-22).

① 첫 스캔 전에는 아무것도 안 함(기동 중) ② 스캔이 꾸준하면 안 함 ③ stall_s 넘게 끊기면 드라이버를 끝냄
④ 끝낸 뒤 cooldown_s 안에는 또 끝내지 않음 ⑤ cooldown 지나도 계속 끊겨 있으면 다시 끝냄 ⑥ 드라이버가 없으면(서비스 소속 등) 안 함
⑦ 실제 프로세스: 이름이 ldlidar_stl_ros2_node인 가짜 프로세스를 SIGSTOP으로 얼려도(데이터 멈춤 흉내) TERM 뒤 KILL로 끝냄

    python3 tools/test_lidar_watchdog.py
"""
import os, signal, subprocess, sys, tempfile, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lidar_watchdog import Watchdog

fails = []


def check(name, ok, detail=''):
    print(f'{"✅" if ok else "❌"} {name}' + (f' — {detail}' if detail and not ok else ''))
    if not ok:
        fails.append(name)


def main():
    t = [0.0]; killed = []
    w = Watchdog(stall_s=3.0, cooldown_s=15.0, clock=lambda: t[0], find=lambda: [4242], kill=lambda p: killed.append(list(p)), log=lambda s: None)
    t[0] = 10.0
    check('① 첫 스캔 전에는 안 함', not w.tick() and not killed)
    for k in range(50):
        t[0] = 10.0 + k * 0.1; w.on_scan()
    t[0] += 2.9
    check('② 3초 안이면 안 함', not w.tick() and not killed)
    t[0] += 0.2
    check('③ 3초 넘게 끊기면 드라이버를 끝냄', w.tick() and killed == [[4242]], killed)
    t[0] += 10.0
    check('④ 끝낸 뒤 15초 안에는 또 안 함', not w.tick() and len(killed) == 1)
    t[0] += 5.1
    check('⑤ 15초 지나도 계속 끊겨 있으면 다시 끝냄', w.tick() and len(killed) == 2 and w.restarts == 2)
    w2 = Watchdog(stall_s=3.0, cooldown_s=15.0, clock=lambda: t[0], find=lambda: [], kill=lambda p: killed.append(p), log=lambda s: None)
    w2.on_scan(); t[0] += 5
    check('⑥ 드라이버 프로세스가 없으면 안 함', not w2.tick() and len(killed) == 2)

    # ⑦ 실제 프로세스: 이름이 정확히 드라이버 실행파일인 가짜를 띄우고 얼린다
    d = tempfile.mkdtemp(); exe = os.path.join(d, 'ldlidar_stl_ros2_node')
    import shutil; shutil.copy('/bin/sleep', exe)        # 셸 스크립트면 프로세스 이름이 sh로 잡힌다 — 진짜 실행파일로
    p = subprocess.Popen([exe, '60']); time.sleep(0.3); os.kill(p.pid, signal.SIGSTOP)
    w3 = Watchdog(stall_s=0.5, cooldown_s=15.0, log=lambda s: None)
    w3.on_scan(); time.sleep(0.7); r = w3.tick()
    try:
        p.wait(timeout=5); gone = True
    except subprocess.TimeoutExpired:
        gone = False; p.kill()
    check('⑦ 얼어 있는 실제 드라이버 이름 프로세스도 끝냄(TERM → KILL)', r and gone, (r, gone))

    print('합격' if not fails else f'불합격 {len(fails)}건: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
