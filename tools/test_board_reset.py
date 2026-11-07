#!/usr/bin/env python3
"""모터 보드 무응답 → 자동 RTS 리셋 로직 시험(로봇·ROS 없이, 2026-11-07 사고 후속).

가짜 시리얼: 처음 0.5초는 정상 응답, 그 뒤 응답 없음(보드 멈춤), RTS를 낮게→높게→낮게 주면 다시 응답(리셋)하고 카운터는 0부터.
확인: ① 무응답 1초 무렵 리셋 1번 ② RTS 순서가 실측과 같음 ③ 리셋 뒤 설정 명령·출력 0 재전송 ④ 위치 기준값 다시 잡기 플래그
⑤ 리셋 뒤에도 응답이 없으면 5초 간격으로만 다시 시도.

    python3 tools/test_board_reset.py
"""
import os, sys, threading, time, types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'robot', 'src', 'smart_trash_motion'))
from smart_trash_motion.diff_drive_node import DiffDriveNode


class FakeSerial:
    def __init__(self, silent_after=0.5, recover=True):
        self.t0 = time.monotonic(); self.silent_after = silent_after; self.recover = recover
        self.dead = False; self.count = 1000; self._rts = True; self.rts_log = []; self.written = []; self.buf = b''
    @property
    def rts(self):
        return self._rts
    @rts.setter
    def rts(self, v):
        self.rts_log.append(v)
        if self.rts_log[-3:] == [False, True, False] and self.recover:
            self.dead = False; self.count = 0; self.t0 = time.monotonic() + 999      # 리셋: 다시 살아남, 카운터 0
        self._rts = v
    def reset_input_buffer(self):
        self.buf = b''
    def write(self, b):
        self.written.append(b)
        if time.monotonic() - self.t0 > self.silent_after:
            self.dead = True
        if not self.dead and b == b'$upload:1#':
            self.count += 10; c = self.count; self.buf = f'$MAll:{c},{c},{c},{c}#\r\n'.encode()
    @property
    def in_waiting(self):
        return len(self.buf)
    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out


def make(fs):
    nd = DiffDriveNode.__new__(DiffDriveNode)
    log = []
    lg = types.SimpleNamespace(warn=lambda m, **k: log.append(('W', m)), error=lambda m, **k: log.append(('E', m)), info=lambda m, **k: log.append(('I', m)))
    nd.get_logger = lambda: lg; nd._ser = fs; nd._serial_lock = threading.Lock(); nd._counts_lock = threading.Lock()
    nd._encoder_thread_stop = threading.Event(); nd._latest_counts = None; nd._latest_counts_ts = None; nd._vel_prev = (1, 1)
    nd._board_reset_after_s = 1.0; nd._board_reset_gap_s = 5.0; nd._board_resets = 0; nd._last_board_reset = -1e9; nd._rebaseline = False
    nd._control_mode = 'velocity'; nd.get_parameter = lambda k: types.SimpleNamespace(value=0); nd._last_pwm4 = [1, 1, 1, 1]
    nd._encoder_pub = types.SimpleNamespace(publish=lambda m: None)
    return nd, log


def main():
    ok = []
    # A. 0.5초 뒤 멈춤 → 리셋 → 복구
    fs = FakeSerial(); nd, log = make(fs); th = threading.Thread(target=nd._encoder_poll_loop, daemon=True); th.start()
    t0 = time.monotonic(); reset_at = None
    while time.monotonic() - t0 < 4.0:
        if nd._board_resets and reset_at is None:
            reset_at = time.monotonic() - t0
        time.sleep(0.02)
    nd._encoder_thread_stop.set(); th.join(1.0)
    after = nd._latest_counts
    a = (nd._board_resets == 1 and reset_at is not None and 1.4 <= reset_at <= 2.0 and fs.rts_log[-3:] == [False, True, False]
         and b'$mtype:1#' in fs.written and b'$pwm:0,0,0,0#' in fs.written and nd._last_pwm4 == [0, 0, 0, 0] and nd._rebaseline is True
         and after is not None and after[0] < 500 and nd._vel_prev is None)
    ok.append(a)
    print(f'{"✅" if a else "❌"} 멈춤 0.5초 → 리셋 {reset_at and round(reset_at, 2)}초(무응답 약 1초 뒤), 리셋 {nd._board_resets}번, RTS {fs.rts_log[-3:]}, '
          f'설정 재전송 {b"$mtype:1#" in fs.written}, 출력 0 {nd._last_pwm4}, 기준값 다시 잡기 {nd._rebaseline}, 리셋 뒤 카운터 {after}')
    # B. 리셋해도 안 살아남 → 5초 간격으로만
    fs = FakeSerial(recover=False); nd, log = make(fs); th = threading.Thread(target=nd._encoder_poll_loop, daemon=True); th.start()
    time.sleep(7.2); nd._encoder_thread_stop.set(); th.join(1.0)
    b = nd._board_resets == 2
    ok.append(b)
    print(f'{"✅" if b else "❌"} 리셋해도 무응답 7초: 리셋 {nd._board_resets}번(약 1.5초·6.5초 무렵 2번이어야)')
    print(f'\n보드 리셋 시험: {sum(ok)}/{len(ok)} 통과')
    return 0 if all(ok) else 1


if __name__ == '__main__':
    sys.exit(main())
