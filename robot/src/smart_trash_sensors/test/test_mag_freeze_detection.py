#!/usr/bin/env python3
"""_publish_mag의 자기센서 멈춤 감지/자동복구 검증.

하드웨어 없이 돈다: ImuPublisherNode를 object.__new__로 만들어 __init__(I2C 접근)을
우회하고, 실제 _publish_mag 코드를 그대로 호출한다 — 로직을 베껴 쓴 게 아니라
배포되는 코드 경로를 검증한다.

배경: AK09916이 ICM-20948 내부 I2C 마스터 위에서 응답을 멈추면 EXT_SLV_SENS_DATA가
마지막 값을 붙들고 있어서, 노드가 죽은 값을 50Hz로 계속 퍼블리시한다. 토픽 주파수만
보면 완전히 정상으로 보이는 조용한 실패다(2026-08-17 실측: 33초/1654샘플 전부 동일).

    python3 robot/src/smart_trash_sensors/test/test_mag_freeze_detection.py
"""
import time
from smart_trash_sensors.imu_publisher_node import ImuPublisherNode

class FakeImu:
    mxRaw = myRaw = mzRaw = 100
class FakePub:
    def __init__(self): self.n = 0
    def publish(self, m): self.n += 1
class FakeLog:
    def __init__(self): self.msgs = []
    def warn(self, m): self.msgs.append(('W', m))
    def info(self, m): self.msgs.append(('I', m))

n = object.__new__(ImuPublisherNode)          # __init__(하드웨어) 우회
n._imu = FakeImu(); n._mag_pub = FakePub(); n._log = FakeLog()
n.get_logger = lambda: n._log
n._mag_off = (0.0, 0.0, 0.0)
n._mag_stale_sec = 0.3
n._mag_last_raw = None
n._mag_last_change = time.monotonic()
n._mag_recover_at = 0.0
n._mag_frozen = False

inits = []
FakeImu.startupMagnetometer = lambda self: inits.append(1)

ok = True
def check(label, cond):
    global ok
    ok &= cond
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")

# 1) 값이 계속 바뀌면 정상 발행
for i in range(5):
    FakeImu.mxRaw = 100 + i
    n._publish_mag(None)
check("값이 변하면 발행된다 (5회)", n._mag_pub.n == 5)

# 2) 값이 고정되면 stale_sec 이후 발행 중단
base = n._mag_pub.n
FakeImu.mxRaw = 500
n._publish_mag(None)          # 첫 변화 -> 발행됨
check("고정 직전 마지막 값은 발행", n._mag_pub.n == base + 1)
time.sleep(0.35)
for _ in range(10):
    n._publish_mag(None)
check("멈춤 판정 후 발행 중단", n._mag_pub.n == base + 1)
check("멈춤 경고 로그 1회만", sum(1 for k, m in n._log.msgs if k == 'W' and '멈춤' in m) == 1)
check("재초기화 시도됨", len(inits) >= 1)
check("재초기화는 5초에 1회로 제한", len(inits) == 1)

# 3) 값이 다시 바뀌면 복구
FakeImu.mxRaw = 900
n._publish_mag(None)
check("복구 후 발행 재개", n._mag_pub.n == base + 2)
check("복구 로그 남음", any(k == 'I' and '복구' in m for k, m in n._log.msgs))
check("frozen 플래그 해제", n._mag_frozen is False)

print("\n" + ("전부 통과" if ok else "실패 있음"))
raise SystemExit(0 if ok else 1)
