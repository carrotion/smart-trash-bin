#!/usr/bin/env python3
"""바퀴 1회전당 엔코더 카운트를 실측으로 구하는 캘리브레이션 도구.

이론값(11펄스 × 감속비 21.3 × 체배)은 계산 가능하지만, Yahboom 드라이버가
쿼드러처를 x1/x2/x4 중 어떻게 체배하는지 문서에 없어서 실측이 유일하게 확실함.

시리얼 포트는 motor_dashboard 서비스가 쥐고 있으므로, 이 스크립트는 포트를 직접
열지 않고 그 서비스의 HTTP API(/api/status)로 엔코더 값만 읽는다 — 충돌 없음.

사용법:
    python3 tools/calibrate_encoder.py start   # 기준값 저장
    (바퀴를 손으로 정확히 N바퀴 돌림)
    python3 tools/calibrate_encoder.py stop N  # N바퀴 돌렸다고 알려주면 계산
"""
import json
import sys
import urllib.request

STATUS_URL = "http://localhost:8766/api/status"
STATE_PATH = "/tmp/encoder_calib_baseline.json"
from wheel_config import NAMES as WHEEL_NAMES  # 채널 매핑 단일 출처


def read_encoder():
    with urllib.request.urlopen(STATUS_URL, timeout=3) as resp:
        data = json.loads(resp.read())
    if not data.get("connected"):
        sys.exit("모터 드라이버 연결 안 됨 — smart-trash-motor-dashboard 서비스 확인 필요")
    return data["encoder"]


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("start", "stop"):
        sys.exit(__doc__)

    if sys.argv[1] == "start":
        enc = read_encoder()
        with open(STATE_PATH, "w") as f:
            json.dump(enc, f)
        print("기준값 저장:", enc)
        print("이제 바퀴를 정확히 N바퀴 돌린 뒤 'stop N'을 실행하세요.")
        return

    if len(sys.argv) < 3:
        sys.exit("몇 바퀴 돌렸는지 알려주세요. 예: python3 tools/calibrate_encoder.py stop 10")
    turns = float(sys.argv[2])

    try:
        with open(STATE_PATH) as f:
            base = json.load(f)
    except OSError:
        sys.exit("기준값이 없습니다. 먼저 'start'를 실행하세요.")

    now = read_encoder()
    print(f"기준값: {base}")
    print(f"현재값: {now}")
    print(f"\n{turns}바퀴 기준 계산 결과:")
    for i, name in enumerate(WHEEL_NAMES):
        delta = now[i] - base[i]
        if delta == 0:
            print(f"  {name}: 변화 없음 (이 바퀴는 안 돌린 듯)")
        else:
            print(f"  {name}: {delta:+d} 카운트 → 1회전당 {abs(delta)/turns:.1f} 카운트")


if __name__ == "__main__":
    main()
