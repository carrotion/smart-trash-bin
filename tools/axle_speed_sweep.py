#!/usr/bin/env python3
"""앞(370)/뒤(520) 축을 같은 PWM으로 각각 스윕해서 무부하 회전속도를 비교한다.

배경: 2026-09-24 뒷바퀴 370->520 교체 후, 같은 명령에 같은 실제 바퀴속도가
나오도록 축별 PWM-속도 변환을 분리해야 한다(異종 모터라 감속비/토크가 다름).
이 스크립트는 그 첫 단계 — 도크 위 무부하 상태에서 PWM별 실제 rev/s를
측정해 두 축의 반응 곡선을 직접 비교한다.

⚠️ 도크 위(바퀴 허공) 전용. 무부하 측정이라 바닥 마찰 부하 상태와는 다를 수
있음 — 이후 실주행 대조가 별도로 필요할 수 있다.
⚠️ diff_drive_node가 시리얼 포트를 쓰고 있으면 포트 충돌 나니 먼저
   `sudo systemctl stop smart-trash-diff-drive` 할 것.

사용법:
    sudo systemctl stop smart-trash-diff-drive
    python3 tools/axle_speed_sweep.py
    sudo systemctl start smart-trash-diff-drive
"""
import serial, time, sys, csv, os, math

from wheel_config import WHEEL_RADIUS, COUNTS_PER_REV, PWM_SIGN

PORT = '/dev/motor'
BAUD = 115200
COUNTS_PER_REV_FRONT = COUNTS_PER_REV[0]
COUNTS_PER_REV_REAR = COUNTS_PER_REV[2]
PWMS = [150, 200, 300, 450, 600, 800]
DRIVE_S = 1.5


class Driver:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.3)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write(cmd.encode())

    def stop(self):
        for _ in range(3):
            self.send('$pwm:0,0,0,0#')
            time.sleep(0.05)

    def encoders(self, timeout=1.0):
        self.ser.reset_input_buffer()
        self.send('$read:0#')
        end = time.time() + timeout
        buf = ''
        while time.time() < end:
            buf += self.ser.read(128).decode('ascii', 'ignore')
            if '$MAll:' in buf and '#' in buf.split('$MAll:')[-1]:
                body = buf.split('$MAll:')[-1].split('#')[0]
                try:
                    return [int(v) for v in body.split(',')[:4]]
                except ValueError:
                    return None
        return None

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def sweep(d, axle_name, ch_indices, counts_per_rev, pwm_signs):
    """ch_indices: 0-based 채널 인덱스 튜플(예: (0,1)=M1,M2). pwm_signs: 각 채널에
    보낼 때 곱할 부호(오른쪽 채널은 극성반전 보정 -1 — _apply_pwm과 동일 규약)."""
    print(f'\n=== {axle_name} 스윕 ===')
    rows = []
    for pwm in PWMS:
        vals = [0, 0, 0, 0]
        for idx, sign in zip(ch_indices, pwm_signs):
            vals[idx] = int(pwm * sign)
        before = d.encoders()
        t0 = time.time()
        d.send('$pwm:{},{},{},{}#'.format(*vals))
        time.sleep(DRIVE_S)
        t1 = time.time()
        d.stop()
        time.sleep(0.8)
        after = d.encoders()
        if not (before and after):
            print(f'  PWM {pwm}: 엔코더 읽기 실패')
            continue
        dt = t1 - t0
        speeds = []
        for idx in ch_indices:
            delta = abs(after[idx] - before[idx])
            rev_s = (delta / counts_per_rev) / dt
            mps = rev_s * 2.0 * math.pi * WHEEL_RADIUS
            speeds.append(mps)
        avg_mps = sum(speeds) / len(speeds)
        print(f'  PWM {pwm:>4}: {[f"{s:.3f}" for s in speeds]} m/s (평균 {avg_mps:.3f})')
        rows.append((pwm, avg_mps))
        time.sleep(0.3)
    return rows


def main():
    d = Driver()
    try:
        d.send('$upload:1#')
        time.sleep(0.3)

        # 앞(M1,M2) — 왼쪽은 정극성, 오른쪽(M2)은 _apply_pwm과 동일하게 반전
        front_rows = sweep(d, '앞(370, M1+M2)', (0, 1), COUNTS_PER_REV_FRONT,
                           (PWM_SIGN[0], PWM_SIGN[1]))
        rear_rows = sweep(d, '뒤(520, M3+M4)', (2, 3), COUNTS_PER_REV_REAR,
                          (PWM_SIGN[2], PWM_SIGN[3]))
    finally:
        d.close()

    fpath = 'docs/calibration/axle_speed_sweep.csv'
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    with open(fpath, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['axle', 'pwm', 'mps'])
        for pwm, mps in front_rows:
            w.writerow(['front', pwm, f'{mps:.4f}'])
        for pwm, mps in rear_rows:
            w.writerow(['rear', pwm, f'{mps:.4f}'])

    print(f'\n=== 비교 (같은 PWM에서 앞 vs 뒤) ===')
    front_map = dict(front_rows)
    rear_map = dict(rear_rows)
    for pwm in PWMS:
        if pwm in front_map and pwm in rear_map:
            f_v, r_v = front_map[pwm], rear_map[pwm]
            ratio = r_v / f_v if f_v > 0 else float('nan')
            print(f'  PWM {pwm:>4}: 앞 {f_v:.3f} m/s vs 뒤 {r_v:.3f} m/s (뒤/앞 비율 {ratio:.2f})')

    print(f'\n원본 로그: {fpath}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
