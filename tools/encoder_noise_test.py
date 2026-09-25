#!/usr/bin/env python3
"""엔코더 오독(노이즈 유입) 판별 시험 — 도크 위, 드라이버에 직접 명령.

배경(2026-09-24~25, docs/todo.md #1): 뒷바퀴를 520으로 바꾼 뒤 드라이버 3·4번
채널이 회전 패턴(왼쪽 역방향 + 오른쪽 정방향)에서 물리적으로 불가능한 값을 읽었다
(4번: 초당 약 22,000카운트 = −1.9 m/s, 520 무부하 최고 0.63 m/s). M3↔M4 커넥터를
맞바꿔도 오독은 보드 4번 채널에 남았고, 370 4개 시절 녹화엔 0건이었다.

콘덴서·페라이트 코어·배선 정리 같은 하드웨어 대책을 **하기 전과 후에 이 도구를
같은 조건으로 돌려** 효과를 숫자로 비교한다. 결과는 JSON으로 누적 저장.

판정(샘플 = 약 0.07초 간격 엔코더 두 번 읽기):
  - 불가능 속도: 앞(370) > 1.3 m/s, 뒤(520) > 0.8 m/s
  - 명령 반대 방향: 명령한 방향과 반대로 0.15 m/s 넘게 읽힘

⚠️ 도크 위(바퀴 허공) 전용. diff_drive_node가 포트를 쥐고 있으면 거부한다.

    sudo systemctl stop smart-trash-diff-drive
    python3 tools/encoder_noise_test.py --label "대책 전"
    sudo systemctl start smart-trash-diff-drive
"""
import argparse, json, os, re, subprocess, sys, time
import serial

from wheel_config import PWM_SIGN, ENCODER_SIGN, METERS_PER_COUNT

MAX_SPEED = [1.3, 1.3, 0.8, 0.8]
REVERSE_TOL = 0.15
# (이름, 채널별 전진 기준 PWM) — 드라이버 deadzone 0 기준. 2026-09-24 재현 패턴
PATTERNS = [
    ('M4 단독 1200', [0, 0, 0, 1200]),
    ('회전 저출력', [-1300, 1300, -1600, 1200]),
    ('회전 고출력', [-2300, 2300, -2600, 2200]),
    ('반대 회전 저출력', [1300, -1300, 1600, -1200]),
    ('직진 저출력', [1300, 1300, 1600, 1200]),
    ('후진 저출력', [-1300, -1300, -1600, -1200]),
]
_MALL = re.compile(r"\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True, help='예: "대책 전", "콘덴서 후"')
    ap.add_argument('--sec', type=float, default=2.0)
    ap.add_argument('--out', default='docs/calibration/encoder_noise_test.json')
    args = ap.parse_args()

    st = subprocess.run(['systemctl', 'is-active', 'smart-trash-diff-drive'],
                        capture_output=True, text=True).stdout.strip()
    if st == 'active':
        print('❌ diff_drive_node가 /dev/motor를 쓰는 중 — 먼저 sudo systemctl stop smart-trash-diff-drive')
        return 1

    ser = serial.Serial('/dev/motor', 115200, timeout=0.2)
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

    def stop():
        for _ in range(3):
            ser.write(b'$pwm:0,0,0,0#'); time.sleep(0.05)

    results = []
    try:
        cmd('$mtype:1#'); cmd('$deadzone:0#'); cmd('$upload:0#')
        for name, fwd in PATTERNS:
            raw = [int(fwd[i] * PWM_SIGN[i]) for i in range(4)]
            prev = enc(); tp = time.time(); t0 = tp
            ser.write(f'$pwm:{raw[0]},{raw[1]},{raw[2]},{raw[3]}#'.encode())
            n = 0
            impossible = [0] * 4
            reverse = [0] * 4
            worst = [0.0] * 4
            while time.time() - t0 < args.sec:
                time.sleep(0.07)
                e = enc(); t = time.time()
                if e and prev and t - tp > 1e-3:
                    n += 1
                    for i in range(4):
                        v = (e[i] - prev[i]) * ENCODER_SIGN[i] * METERS_PER_COUNT[i] / (t - tp)
                        if abs(v) > abs(worst[i]):
                            worst[i] = v
                        if abs(v) > MAX_SPEED[i]:
                            impossible[i] += 1
                        if fwd[i] and v * fwd[i] < 0 and abs(v) > REVERSE_TOL:
                            reverse[i] += 1
                prev, tp = e, t
            stop(); time.sleep(1.0)
            bad = [impossible[i] + reverse[i] for i in range(4)]
            print(f'{name:12s} 샘플 {n:3d} | 불가능 {impossible} | 반대방향 {reverse} | '
                  f'최대 {[round(x, 2) for x in worst]} m/s {"✅" if not any(bad) else "❌"}')
            results.append({'pattern': name, 'fwd_pwm': fwd, 'samples': n,
                            'impossible': impossible, 'reverse': reverse,
                            'worst_mps': [round(x, 3) for x in worst]})
    finally:
        stop()
        ser.close()

    total = sum(sum(r['impossible']) + sum(r['reverse']) for r in results)
    samples = sum(r['samples'] for r in results)
    print(f'\n이상 샘플 합계 {total} / {samples} ({args.label})')
    print('⚠️ 드라이버 deadzone이 0으로 남았다 — diff_drive 서비스를 다시 켜면 원래 값으로 설정된다')

    hist = []
    if os.path.exists(args.out):
        with open(args.out) as f:
            hist = json.load(f)
    hist.append({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'label': args.label,
                 'sec': args.sec, 'total_bad': total, 'total_samples': samples,
                 'patterns': results})
    with open(args.out, 'w') as f:
        json.dump(hist, f, ensure_ascii=False, indent=1)
    print(f'저장: {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
