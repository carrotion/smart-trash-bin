#!/usr/bin/env python3
"""제자리 회전 마찰 측정 — 정지마찰(돌기 시작 출력)과 운동마찰 표(돌고 있을 때 출력→각속도, 멈추는 출력).

1층 ③ 제자리 회전(docs/motion_foundation.md) 재설계용. 09-27 기준 측정에서 출발 튐 1.6~2.7배:
돌기 시작하는 출력이 돌고 있을 때 필요한 출력보다 커서, 떨어지는 순간 같은 출력으로 훨씬 빨리 돈다.
구동 노드의 회전표(09-25)는 둘을 구분하지 않았고 우회전은 약 200 과대였다.

방향(좌/우)마다 repeats회:
  ① 출발: 출력 start부터 +step씩(0.15초 간격) 올려 |자이로| > 0.1 rad/s가 되는 출력 = 정지마찰
  ② 하강: 곧바로 top 출력 1.0초 → 이후 −down씩(각 1.0초) 내리며 마지막 0.5초 평균 자이로 기록
     평균 |자이로| < 0.03이면 멈춘 것 = 운동마찰 하한
출력 방식은 구동 노드 제자리 회전과 같다: 왼쪽 바퀴 −, 오른쪽 +(좌회전), 네 바퀴 같은 크기, deadzone 0.
안전: |자이로| > 2.5 rad/s 즉시 정지·중단, 시작 전 주변 0.4 m 여유.

    sudo systemctl stop smart-trash-diff-drive
    python3 tools/turn_friction_calibrate.py
    sudo systemctl start smart-trash-diff-drive
⚠️ 바닥 전용, 배터리 구동(충전 중 아님).
"""
import argparse, json, math, os, subprocess, sys, threading, time
import rclpy
import serial
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wheel_config import PWM_SIGN
from rotation_quality_test import scan_points

LEFT, RIGHT = (0, 2), (1, 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=int, default=1400)
    ap.add_argument('--step', type=int, default=20)
    ap.add_argument('--top', type=int, default=2100)
    ap.add_argument('--down', type=int, default=50)
    ap.add_argument('--max', type=int, default=2600)
    ap.add_argument('--repeats', type=int, default=2)
    a = ap.parse_args()
    if subprocess.run(['systemctl', 'is-active', 'smart-trash-diff-drive'],
                      capture_output=True, text=True).stdout.strip() == 'active':
        print('❌ diff_drive_node가 /dev/motor 사용 중 — 먼저 sudo systemctl stop smart-trash-diff-drive'); return 1

    rclpy.init(); n = rclpy.create_node('turn_friction_calibrate')
    S = {'g': [], 'gz': 0.0, 'scan': None}

    def on_imu(m):
        S['gz'] = m.angular_velocity.z; S['g'].append((time.time(), m.angular_velocity.z))
    n.create_subscription(Imu, '/imu/data', on_imu, 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    stop_ev = threading.Event()
    th = threading.Thread(target=lambda: [rclpy.spin_once(n, timeout_sec=0.02) for _ in iter(lambda: stop_ev.is_set(), True)], daemon=True)
    th.start()
    t0 = time.time()
    while (S['scan'] is None or len(S['g']) < 20) and time.time() - t0 < 10:
        time.sleep(0.1)
    near = min((math.hypot(x, y) for x, y in scan_points(S['scan'])), default=9)
    if near < 0.4:
        print(f'❌ 주변 여유 {near:.2f} m < 0.4'); stop_ev.set(); return 1

    ser = serial.Serial('/dev/motor', 115200, timeout=0.2, exclusive=True)
    time.sleep(0.3)
    ser.write(b'$deadzone:0#'); time.sleep(0.15); ser.read(ser.in_waiting)

    def send(mag, d):
        """d=+1 좌회전(CCW), −1 우회전. 왼쪽 바퀴 −d·mag, 오른쪽 +d·mag(전진 기준) → PWM_SIGN 적용."""
        fwd = [0, 0, 0, 0]
        for i in LEFT:
            fwd[i] = -d * mag
        for i in RIGHT:
            fwd[i] = d * mag
        v = [int(fwd[i] * PWM_SIGN[i]) for i in range(4)]
        ser.write(f'$pwm:{v[0]},{v[1]},{v[2]},{v[3]}#'.encode())

    def zero():
        for _ in range(3):
            ser.write(b'$pwm:0,0,0,0#'); time.sleep(0.03)

    class Abort(Exception):
        pass

    def hold(mag, d, dur):
        """dur초 유지(0.05초마다 재전송). 반환: 그 구간 자이로 목록(명령 방향 +)."""
        te = time.time() + dur
        while time.time() < te:
            send(mag, d); time.sleep(0.05)
            if abs(S['gz']) > 2.5:
                zero(); raise Abort(f'|자이로| {S["gz"]:+.2f} > 2.5')
        return [(t, g * d) for t, g in S['g'] if t >= te - dur]

    results = []
    try:
        for rep in range(a.repeats):
            for d in (1, -1):
                name = '좌(CCW)' if d > 0 else '우(CW)'
                time.sleep(1.5)
                while abs(S['gz']) > 0.05:      # 앞 회전 잔여 움직임이 멎을 때까지(09-27 1회차 외 '1400에서 시작' 오판)
                    time.sleep(0.1)
                # ① 출발
                mag = a.start; brk = None
                while mag <= a.max:
                    g = hold(mag, d, 0.15)
                    if S['gz'] * d > 0.1:
                        brk = mag; break
                    mag += a.step
                if brk is None:
                    zero(); print(f'{name} #{rep+1}: {a.max}까지 안 돎'); continue
                # ② 하강
                table = []; stall = None
                g = hold(a.top, d, 1.0)
                mag = a.top
                while mag >= a.start:
                    g = hold(mag, d, 1.0)
                    tail = [x for t, x in g if t >= time.time() - 0.5]
                    w = sum(tail) / len(tail) if tail else 0.0
                    table.append((mag, round(w, 3)))
                    if w < 0.03:
                        stall = mag; break
                    mag -= a.down
                zero()
                results.append(dict(dir=d, rep=rep + 1, breakaway=brk, stall=stall, table=table))
                print(f'{name} #{rep+1}: 돌기 시작 {brk} | 멈춤 {stall} | 표 ' +
                      ' '.join(f'{m}:{w:.2f}' for m, w in table), flush=True)
    except Abort as e:
        print('❌ 안전 중단:', e)
    finally:
        zero(); ser.close(); stop_ev.set(); th.join(timeout=1.0)
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, 'docs/calibration', f'turn_friction_{time.strftime("%Y%m%d_%H%M")}.json')
        json.dump(dict(args=vars(a), results=results), open(out, 'w'), ensure_ascii=False, indent=1)
        print('저장:', out)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
