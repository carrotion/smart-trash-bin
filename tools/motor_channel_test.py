#!/usr/bin/env python3
"""모터 4채널을 **하나씩** 구동해서 죽은 채널을 찾는다.

⚠️ 반드시 **바퀴가 허공에 뜬 상태**(도크 위)에서 실행할 것.
⚠️ 배터리 커넥터/메인스위치에 손이 닿는 상태로 실행할 것.
   (2026-08-14에 정지 명령이 안 먹혀 물리 전원차단으로 겨우 멈춘 사고가 있었다)

안전 설계
---------
- `$pwm:` **개루프만** 사용한다. `$spd:`와 섞어 쓰지 않는다(CLAUDE.md 수칙).
- `$mtype:3/4`(TT 프리셋)는 **절대 보내지 않는다.** 그 모드에서 정지 명령이
  안 먹혀 바퀴 4개가 안 멈춘 사고가 있었다. 이 스크립트는 mtype을 아예 안 건드린다.
- 채널당 구동은 **낮은 PWM으로 짧게**. 매 채널 후 정지하고 **엔코더가 실제로
  멈췄는지 확인**한 뒤 다음으로 넘어간다.
- 어떤 예외가 나도 `finally`에서 정지 명령을 **여러 번** 보낸다.

채널 매핑 (2026-09-24 갱신 — 뒷바퀴 370->520 교체하며 배선을 앞/뒤 묶음으로 재배선)
  M1 = 왼쪽 앞(370),  M2 = 오른쪽 앞(370),  M3 = 왼쪽 뒤(520, 신규),  M4 = 오른쪽 뒤(520, 신규)
  ⚠️ 2026-08-14~09-05까지의 예전 매핑(M1=왼쪽뒤 등, 좌/우 묶음)은 더 이상 유효하지 않음.
  4채널 전부 양수 PWM = 전진 (좌우 부호 반전 불필요)

사용법
------
    sudo systemctl stop smart-trash-diff-drive     # 포트 소유권을 넘겨받는다
    python3 tools/motor_channel_test.py --pwm 300 --sec 1.5
    sudo systemctl start smart-trash-diff-drive
"""

import argparse
import sys
import time

import serial

PORT = '/dev/motor'
BAUD = 115200
# ⚠️ 2026-09-24: 뒷바퀴 520 교체 + 채널 재배선(좌/우 묶음 -> 앞/뒤 묶음)으로 갱신.
# M1/M2=앞(기존 370), M3/M4=뒤(신규 520). 예전 매핑(M1=왼쪽뒤 등)은 무효.
NAMES = {1: '왼쪽 앞', 2: '오른쪽 앞', 3: '왼쪽 뒤(520)', 4: '오른쪽 뒤(520)'}


class Driver:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write(cmd.encode())
        self.ser.flush()

    def stop(self, times=3):
        """정지는 여러 번 보낸다 — 한 번 유실되면 계속 도는 것보다 낫다."""
        for _ in range(times):
            self.send('$pwm:0,0,0,0#')
            time.sleep(0.05)

    def encoders(self, timeout=1.0):
        """$MAll 누적 엔코더를 읽는다. 형식: $MAll:a,b,c,d#"""
        self.ser.reset_input_buffer()
        self.send('$read:0#')          # 일부 펌웨어는 요청이 필요 없다
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pwm', type=int, default=300,
                    help='시험 PWM. 낮게 시작할 것(데드존은 약 230)')
    ap.add_argument('--sec', type=float, default=1.5)
    ap.add_argument('--port', default=PORT)
    args = ap.parse_args()

    if abs(args.pwm) > 600:
        print('PWM이 너무 크다. 진단은 낮은 값으로 충분하다.'); return 1

    print(f'포트 {args.port} @ {BAUD}')
    print('⚠️ 바퀴가 허공에 떠 있는지 확인했는가? 배터리 커넥터에 손이 닿는가?\n')

    d = Driver(args.port, BAUD)
    results = {}
    try:
        d.send('$upload:1#')          # 누적 엔코더 보고 켜기
        time.sleep(0.3)
        base = d.encoders()
        if base is None:
            print('⚠️ 엔코더를 못 읽는다. 그래도 구동 시험은 진행하되 판정은 육안으로.')

        for ch in (1, 2, 3, 4):
            vals = [0, 0, 0, 0]
            vals[ch - 1] = args.pwm
            before = d.encoders()
            print(f'  M{ch} ({NAMES[ch]}) 구동 {args.pwm}, {args.sec}초 ... ',
                  end='', flush=True)
            d.send('$pwm:{},{},{},{}#'.format(*vals))
            time.sleep(args.sec)
            d.stop()
            # 2026-09-24: 0.6 -> 1.2초로 늘림. 520 모터(신규 뒷바퀴)가 370보다
            # 크고 무거워 관성이 더 클 수 있음 — 실측에서 0.7초 판정 창으로는
            # 정상 코스팅을 "정지 실패"로 오탐한 사례 있었음(M3, 실제로는 완전
            # 정지 확인됨). 판정 전에 충분히 가라앉을 시간을 준다.
            time.sleep(1.2)
            after = d.encoders()

            if before and after:
                delta = [after[i] - before[i] for i in range(4)]
                moved = abs(delta[ch - 1])
                cross = [f'M{i+1}' for i in range(4)
                         if i != ch - 1 and abs(delta[i]) > 30]
                results[ch] = (moved, delta, cross)
                verdict = '✅ 정상' if moved > 100 else '❌ 반응 없음'
                print(f'{verdict}  Δ{delta}')
                if cross:
                    print(f'      ⚠️ 다른 채널도 움직였다: {", ".join(cross)}')
            else:
                print('엔코더 읽기 실패 — 육안으로 확인할 것')

            # 정지 확인: 멈춘 뒤 값이 고정되는지 (위에서 이미 1.2초 가라앉힌 뒤라
            # 여기 걸리면 진짜 정지 실패로 봐도 됨 — 관성 오탐 여지를 줄임)
            s1 = d.encoders(); time.sleep(1.0); s2 = d.encoders()
            if s1 and s2 and any(abs(s2[i] - s1[i]) > 20 for i in range(4)):
                print('      ❌ 정지 후에도 엔코더가 변한다 — 즉시 물리 전원 차단!')
                return 2
            time.sleep(0.4)
    finally:
        d.close()

    if results:
        print('\n=== 요약 ===')
        dead = [ch for ch, (m, _, _) in results.items() if m <= 100]
        for ch, (m, _, _) in sorted(results.items()):
            print(f'  M{ch} ({NAMES[ch]:<8}) 엔코더 변화 {m:>6}  '
                  + ('❌ 죽은 채널' if m <= 100 else '정상'))
        if dead:
            print(f'\n  반응 없는 채널: {", ".join("M"+str(c) for c in dead)}')
            print('  엔코더는 이미 4채널 모두 살아있음이 확인됐으므로,')
            print('  원인은 **모터 구동선 또는 드라이버 출력단**이다.')
            print('  확인 순서: ① 해당 모터 전원 커넥터 체결 ② 드라이버 출력 단자')
            print('             ③ 다른 채널에 그 모터를 물려보기(모터 자체 고장 배제)')
        else:
            print('\n  ✅ 4채널 전부 정상')
    return 0


if __name__ == '__main__':
    sys.exit(main())
