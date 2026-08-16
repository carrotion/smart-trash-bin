#!/bin/bash
# WiFi 게이트웨이가 응답 없으면 wlan0을 재연결.
#
# 2026-08-16 Davi(antigravity) 감사 리포트 제안 반영. 배경: SSH/대시보드만
# 응답불능이고 시스템(전원/CPU/메모리/커널)은 전부 정상이었던 사고가 있었음
# (CLAUDE.md "TCP 서비스만 응답불능" 절 참고, 원인 확정은 안 됐지만 무선
# 링크가 조용히 죽어있었을 가능성에 대한 방어책으로 적용).
#
# 게이트웨이 IP는 하드코딩하지 않고 현재 라우팅 테이블에서 읽음(공유기 IP가
# 바뀌어도 그대로 동작하도록).
set -euo pipefail

GW=$(ip route show default dev wlan0 2>/dev/null | awk '{print $3; exit}')
if [ -z "$GW" ]; then
    exit 0  # wlan0에 기본 경로가 없으면(이더넷만 쓰는 중 등) 조용히 종료
fi

if ping -c 1 -W 2 "$GW" > /dev/null 2>&1; then
    exit 0
fi

sleep 3
if ping -c 1 -W 2 "$GW" > /dev/null 2>&1; then
    exit 0
fi

logger -t wifi-watchdog "게이트웨이($GW) 응답없음 — wlan0 재연결 시도"
wpa_cli -i wlan0 reassociate > /dev/null 2>&1 || true
