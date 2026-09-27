"""EKF 방향 기준 폐루프 제자리 회전 — 목표 각도까지 감속하며 돌고, 멈춘 뒤 다시 재서 보정한다.

왜 필요한가 (2026-09-26):
- 구동 노드는 회전 명령이 0이 되면 출력을 즉시 끊는다 → 멈추는 순간 속도만큼 관성으로 더 돈다.
  그 양은 속도·바닥·배터리에 따라 달라서, 예전 도구처럼 "w×0.12초 앞당겨 멈춤"으로 고정 추정하면
  90°마다 약 2~3° 틀렸다(정사각형 시험 방향 오차 ±7~8°의 대부분).
- 판단 기준을 /odometry/filtered(EKF) 방향으로 두면 로봇이 "믿는" 방향과 실제 돌기가 일치한다.
  EKF 방향 = 자이로 적분이고, 정사각형 시험에서 EKF 방향 추정 오차는 1~4°였다.

방법:
  1) 남은 각 rem에 비례해 감속: |w| = clamp(kp·|rem|, w_min, w_max)
  2) |rem| < 예상 관성 각(|측정 w| × coast_s) 이면 정지 명령
  3) 자이로가 가라앉을 때까지 기다린 뒤 rem을 다시 읽고, |rem| > tol이면 w_min으로 짧게 추가 회전
     (넘쳤으면 반대로). 최대 max_fix번
실측 튜닝(2026-09-26 저녁, 바닥 정사각형 회전 기록 docs/calibration/square_turn_trace_*.json):
- coast_s 0.10 → 0.23: 0.2 rad/s에서 정지 명령 뒤 실제로 2~3° 더 돎(관성 시간 0.12~0.26초, 중앙 약 0.23)
- tol 1° → 3°, max_fix 3 → 1: 작은 보정 회전은 정지마찰을 뚫는 순간 튀어서(순간 0.5 rad/s) 한 번에 4~6°씩
  돈다 → 2.5°를 고치려다 반대로 5° 넘치는 왕복이 생겼다. 이 로봇이 만들 수 있는 최소 회전보다 작은 오차는
  고치지 않는 게 낫다
안전: 전체 시간 제한, 회전 명령 1.5초 뒤에도 자이로 적분이 5° 미만이면 중단(09-26 IMU 재시작 사고).
"""
import math
import time

from geometry_msgs.msg import Twist


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TurnToAngle:
    """get_yaw(): EKF 방향(rad), get_gyro(): 최신 자이로 z(rad/s), spin(t): t초 동안 콜백 처리."""

    # 기본값: 2026-09-27 ④ 비교(docs/motion_foundation.md) D 설정 — w_max 0.8(차체 자연 회전 속도, 낮추면 오히려 튐 증가),
    #   명령 가속 0.3 rad/s²(튐 7/8 기준 내), kp 3·w_min 0.25·settle 0.2(시간 단축). 가속 제한은 구동 노드가 아니라
    #   여기서 건다 — 대시보드 수동 조종 반응은 그대로 두기 위해(사용자 결정).
    def __init__(self, pub, spin, get_yaw, get_gyro, kp=3.0, w_min=0.25, w_max=0.8,
                 coast_s=0.23, tol_deg=3.0, max_fix=1, settle_gyro=0.03, settle_s=0.2, accel=0.3):
        self.pub, self.spin, self.get_yaw, self.get_gyro = pub, spin, get_yaw, get_gyro
        self.kp, self.w_min, self.w_max, self.coast_s = kp, w_min, w_max, coast_s
        self.accel = accel
        self._t_cmd = None
        self.tol, self.max_fix = math.radians(tol_deg), max_fix
        self.settle_gyro, self.settle_s = settle_gyro, settle_s
        self.trace = []   # (시각, 단계, 명령 w, 자이로, EKF 방향) — 실물 진단용
        self._w = 0.0
        self._phase = ''

    def _cmd(self, w):
        now = time.time()
        # 가속 제한: |w|를 키울 때만(같은 방향이거나 0에서 출발), 줄이거나 멈출 땐 즉시
        if self.accel > 0 and w != 0.0 and abs(w) > abs(self._w) and (self._w == 0.0 or (w > 0) == (self._w > 0)):
            dt = min(0.1, now - self._t_cmd) if self._t_cmd else 0.02
            w = math.copysign(min(abs(w), abs(self._w) + self.accel * dt), w)
        self._t_cmd = now
        c = Twist(); c.angular.z = float(w); self.pub.publish(c)
        self._w = float(w)
        self.trace.append((time.time(), self._phase, self._w, self.get_gyro(), self.get_yaw()))

    def stop(self, n=5):
        for _ in range(n):
            self._cmd(0.0); self.spin(0.03)

    def _settle(self, limit=2.0):
        t0 = time.time(); calm = None
        while time.time() - t0 < limit:
            self.spin(0.02)
            self.trace.append((time.time(), 'settle', 0.0, self.get_gyro(), self.get_yaw()))
            if abs(self.get_gyro()) < self.settle_gyro:
                calm = calm or time.time()
                if time.time() - calm >= self.settle_s:
                    return True
            else:
                calm = None
        return False

    def _drive(self, remaining, w_cap, time_limit):
        """남은 회전각 remaining(rad, 부호 = 방향)만큼 감속 회전 후 정지. (정상 여부, 설명, 실제 돈 각)
        ⚠️ 2026-09-27: 예전엔 목표 방향을 wrap(−π~π)으로 비교해 +180°를 요청해도 −π로 접혀 반대로 돌 수 있었다
        → 누적 회전량으로 남은 각을 센다(180° 이상도 요청 방향대로)."""
        t0 = time.time(); moved = 0.0; y_prev = self.get_yaw(); turned = 0.0
        while True:
            rem = remaining - turned
            g = self.get_gyro()
            if abs(rem) < max(abs(g) * self.coast_s, math.radians(0.3)):
                break
            w = math.copysign(max(self.w_min, min(w_cap, self.kp * abs(rem))), rem)
            self._cmd(w); self.spin(0.02)
            now = time.time(); y = self.get_yaw()
            dyaw = wrap(y - y_prev); turned += dyaw; moved += abs(dyaw); y_prev = y
            if now - t0 > time_limit:
                self.stop(); return False, f'시간 초과 {time_limit:.1f}초(남은 각 {math.degrees(rem):+.1f}°)', turned
            if now - t0 > 1.5 and moved < math.radians(5):
                self.stop(); return False, f'1.5초 동안 {math.degrees(moved):.1f}°만 돎 — 자이로/구동 이상', turned
        self.stop()
        turned += wrap(self.get_yaw() - y_prev)          # 정지 명령 동안 돈 만큼
        return True, '', turned

    def _settle_track(self):
        """_settle 동안 관성으로 돈 각(rad). 한 번에 180°를 넘지 않으므로 wrap 한 번으로 충분."""
        y = self.get_yaw(); self._settle(); return wrap(self.get_yaw() - y)

    def turn_by(self, delta_rad):
        """현재 EKF 방향에서 delta만큼 회전(부호 = 방향, 180° 이상 가능). 결과 dict(ok, msg, err_deg, fixes, secs)."""
        t0 = time.time()
        limit = abs(delta_rad) / self.w_min + 3.0
        self._phase = 'main'
        ok, msg, total = self._drive(delta_rad, self.w_max, limit)
        fixes = 0
        while ok:
            total += self._settle_track()
            rem = delta_rad - total
            if abs(rem) <= self.tol or fixes >= self.max_fix:
                break
            fixes += 1
            self._phase = f'fix{fixes}'
            ok, msg, t = self._drive(rem, self.w_min, abs(rem) / self.w_min + 3.0)
            total += t
        rem = delta_rad - total
        return dict(ok=ok, msg=msg, err_deg=-math.degrees(rem), fixes=fixes, secs=time.time() - t0)
