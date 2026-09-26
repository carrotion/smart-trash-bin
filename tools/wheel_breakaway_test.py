#!/usr/bin/env python3
"""바퀴별 정지마찰 문턱 측정(cliff test) — 도크 위(무부하), 드라이버 deadzone 0.

배경(2026-09-25, 사용자 질문): "무부하에서 모터별로 움직임이 시작되는 최소 출력값을
데이터로 가지고 있나?" — 없었다. 09-24 deadzone 0 스윕은 PWM 간격이 커서(800→1200→1800)
구간만 알고 CSV엔 축 평균만 남았다. velocity 모드의 바퀴별 피드포워드
(ff_pwm0 1100/1180/1350/700)는 그 구간에서 어림한 추정치였다.

바퀴 하나씩, 방향별로:
  - 출력을 start부터 step씩 올리며 바퀴가 움직이기 시작하는 PWM(정지마찰 문턱, 출발)
  - 거기서 step씩 내리며 멈추는 PWM(운동마찰 문턱, 정지)
  - repeats회 반복 → 평균·편차
움직임 판정: dwell 동안 엔코더 속도 > moving_mps. ⚠️ 4번 채널은 오독 이력이 있어
부호가 명령과 반대인 판독은 "오독"으로 따로 센다.

    sudo systemctl stop smart-trash-diff-drive
    python3 tools/wheel_breakaway_test.py
    sudo systemctl start smart-trash-diff-drive
결과: docs/calibration/wheel_breakaway_YYYYMMDD_HHMM.json
"""
import argparse, json, os, re, statistics as st, subprocess, sys, time
import serial

from wheel_config import PWM_SIGN, ENCODER_SIGN, METERS_PER_COUNT, NAMES

_MALL = re.compile(r"\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=int, default=400)
    ap.add_argument('--max', type=int, default=2600)
    ap.add_argument('--step', type=int, default=40)
    ap.add_argument('--dwell', type=float, default=0.25)
    ap.add_argument('--moving-mps', type=float, default=0.01)
    ap.add_argument('--repeats', type=int, default=2)
    args = ap.parse_args()

    if subprocess.run(['systemctl', 'is-active', 'smart-trash-diff-drive'],
                      capture_output=True, text=True).stdout.strip() == 'active':
        print('❌ diff_drive_node가 /dev/motor 사용 중 — 먼저 sudo systemctl stop smart-trash-diff-drive')
        return 1
    ser = serial.Serial('/dev/motor', 115200, timeout=0.2, exclusive=True)
    time.sleep(0.3)

    def cmd(c, wait=0.15):
        ser.reset_input_buffer(); ser.write(c.encode()); time.sleep(wait)
        return ser.read(ser.in_waiting)

    def enc():
        ser.reset_input_buffer(); ser.write(b'$upload:1#'); time.sleep(0.05)
        raw = ser.read(ser.in_waiting).decode(errors='replace')
        ser.write(b'$upload:0#'); time.sleep(0.02)
        m = _MALL.findall(raw)
        return [int(x) for x in m[-1]] if m else None

    def drive(i, fwd):
        vals = [0, 0, 0, 0]
        vals[i] = int(fwd * PWM_SIGN[i])
        ser.write(f'$pwm:{vals[0]},{vals[1]},{vals[2]},{vals[3]}#'.encode())

    def speed_at(i, fwd):
        """출력을 걸고 dwell 동안의 바퀴 속도(전진 기준 m/s)."""
        drive(i, fwd)
        a = enc(); ta = time.time()
        time.sleep(args.dwell)
        b = enc(); tb = time.time()
        if not a or not b:
            return None
        return (b[i] - a[i]) * ENCODER_SIGN[i] * METERS_PER_COUNT[i] / (tb - ta)

    def stop():
        for _ in range(3):
            ser.write(b'$pwm:0,0,0,0#'); time.sleep(0.05)

    rows = []
    try:
        cmd('$mtype:1#'); cmd('$deadzone:0#'); cmd('$upload:0#')
        for i in range(4):
            for sgn in (1, -1):
                for rep in range(args.repeats):
                    start_pwm = stop_pwm = None
                    misread = 0
                    p = args.start
                    while p <= args.max:
                        v = speed_at(i, sgn * p)
                        if v is not None and v * sgn < -args.moving_mps:
                            misread += 1
                        if v is not None and v * sgn > args.moving_mps:
                            start_pwm = p
                            break
                        p += args.step
                    if start_pwm is not None:
                        p = start_pwm
                        while p >= 0:
                            v = speed_at(i, sgn * p)
                            if v is not None and v * sgn < -args.moving_mps:
                                misread += 1
                            if v is None or v * sgn <= args.moving_mps:
                                stop_pwm = p
                                break
                            p -= args.step
                    stop(); time.sleep(0.6)
                    row = {'wheel': i + 1, 'name': NAMES[i], 'dir': 'fwd' if sgn > 0 else 'rev', 'rep': rep + 1,
                           'start_pwm': start_pwm, 'stop_pwm': stop_pwm, 'misread': misread}
                    rows.append(row)
                    print(f'M{i + 1} {NAMES[i]:10s} {row["dir"]} #{rep + 1}: 출발 {start_pwm} / 정지 {stop_pwm}'
                          + (f' (반대방향 판독 {misread}회)' if misread else ''), flush=True)
    finally:
        stop()
        ser.close()

    print('\n=== 요약 (평균, 반복 편차) ===')
    for i in range(4):
        for d in ('fwd', 'rev'):
            s = [r['start_pwm'] for r in rows if r['wheel'] == i + 1 and r['dir'] == d and r['start_pwm']]
            e = [r['stop_pwm'] for r in rows if r['wheel'] == i + 1 and r['dir'] == d and r['stop_pwm'] is not None]
            if s:
                print(f'M{i + 1} {d}: 출발 {st.mean(s):.0f} (±{(max(s) - min(s)) / 2:.0f}) / '
                      f'정지 {st.mean(e):.0f}' if e else f'M{i + 1} {d}: 출발 {st.mean(s):.0f}')
            else:
                print(f'M{i + 1} {d}: {args.max}까지 출발 안 함')
    print('⚠️ 드라이버 deadzone 0으로 남음 — diff_drive 서비스를 켜면 원래 값으로 설정된다')
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo, time.strftime('docs/calibration/wheel_breakaway_%Y%m%d_%H%M.json'))
    json.dump({'args': vars(args), 'rows': rows}, open(path, 'w'), ensure_ascii=False, indent=1)
    print('저장:', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
