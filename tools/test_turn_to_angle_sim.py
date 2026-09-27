#!/usr/bin/env python3
"""turn_to_angle 로직 오프라인 시험 — 가상 로봇(명령 지연·출발 과회전·관성·자이로 잡음)으로 로봇 없이 확인.

1층 ④ 각도 회전의 제어 로직 회귀 시험(docs/motion_foundation.md). 실물 판정은 turn_angle_test.py.
확인: ±15/90/180/270° 요청 방향대로 돌고(180° 이상 포함 — 09-27 wrap 버그), 최종 오차 < 2°, 가속 제한 준수,
구동 이상(방향이 안 바뀜) 시 중단.

    python3 tools/test_turn_to_angle_sim.py      # 종료 코드 0 = 통과
"""
import math, os, random, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from turn_to_angle import TurnToAngle


class Plant:
    """가상 차체: 명령 뒤 lag초 정지, 이후 over배로 0.4초 튐, 시정수 tau로 추종, 명령 0이면 coast로 감속."""

    def __init__(self, tau=0.15, coast=0.23, over=1.8, lag=0.1):
        self.yaw = 0.0; self.w = 0.0; self.cmd = 0.0; self.t_start = None
        self.tau, self.coast, self.over, self.lag = tau, coast, over, lag
        self.max_cmd_step = 0.0; self._last_cmd = 0.0; self._last_t = None

    def publish(self, msg):
        c = msg.angular.z; now = time.time()
        if self._last_t is not None and abs(c) > abs(self._last_cmd):
            self.max_cmd_step = max(self.max_cmd_step, (abs(c) - abs(self._last_cmd)) / max(1e-3, now - self._last_t))
        self._last_cmd, self._last_t = c, now
        if c != 0 and self.cmd == 0:
            self.t_start = now
        self.cmd = c

    def step(self, dt):
        if self.cmd == 0:
            self.w *= math.exp(-dt / self.coast)
        else:
            el = time.time() - (self.t_start or time.time())
            tgt = 0.0 if el < self.lag else (self.cmd * self.over if el < self.lag + 0.4 else self.cmd)
            self.w += (tgt - self.w) * (1 - math.exp(-dt / self.tau))
        self.yaw = (self.yaw + self.w * dt + math.pi) % (2 * math.pi) - math.pi   # EKF처럼 −π~π로 접힘


def run(deg, **kw):
    P = Plant(**kw)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            time.sleep(0.005); P.step(0.005)
    unwrapped = {'v': 0.0, 'y': 0.0}

    def track():                                   # 실제 누적 회전(판정용)
        d = (P.yaw - unwrapped['y'] + math.pi) % (2 * math.pi) - math.pi
        unwrapped['v'] += d; unwrapped['y'] = P.yaw
    T = TurnToAngle(P, lambda t: (spin(t), track()), lambda: P.yaw, lambda: P.w + random.gauss(0, 0.01))
    r = T.turn_by(math.radians(deg))
    spin(1.0); track()
    real = math.degrees(unwrapped['v'])
    return r, real, P.max_cmd_step


def main():
    fails = 0
    for deg, kw in ((15, {}), (-15, {}), (90, {}), (-90, {}), (180, {}), (-180, {}), (270, {}),
                    (90, dict(over=2.5, lag=0.25)), (90, dict(coast=0.12)), (90, dict(coast=0.35))):
        r, real, acc = run(deg, **kw)
        ok = r['ok'] and abs(real - deg) < 2.0 and acc <= 0.3 * 1.5   # 가속 0.3 rad/s² (샘플 간격 여유 ×1.5)
        fails += not ok
        print(f'{"✅" if ok else "❌"} 요청 {deg:+4d}° {kw}: 실제 {real:+7.1f}°(오차 {real - deg:+.1f}) 보정 {r["fixes"]}회 '
              f'{r["secs"]:.1f}초, 명령 가속 최대 {acc:.2f} rad/s²')
    # 구동 이상: 방향이 전혀 안 바뀌면 1.5초 뒤 중단해야 함
    class Dead:
        def publish(self, m): pass
    T = TurnToAngle(Dead(), time.sleep, lambda: 0.0, lambda: 0.0)
    t = time.time(); r = T.turn_by(math.radians(90))
    ok = (not r['ok']) and time.time() - t < 2.5
    fails += not ok
    print(f'{"✅" if ok else "❌"} 구동 이상 중단: {r["msg"]} ({time.time() - t:.1f}초)')
    print('통과' if fails == 0 else f'실패 {fails}건')
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
