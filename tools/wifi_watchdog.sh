#!/bin/bash
# WiFi 게이트웨이가 응답 없으면 wlan0을 재연결 + 현재 접속 SSID를 world-readable
# 파일로 남김(대시보드가 일반 계정으로 SSID를 읽을 수 있게 — wpa_cli 소켓은
# root 그룹 소유라 sudo 없이는 접근이 안 돼서, root로 도는 이 스크립트가 대신
# 읽어서 파일로 넘겨줌).
#
# 2026-08-16 Davi(antigravity) 감사 리포트 제안 반영. 배경: SSH/대시보드만
# 응답불능이고 시스템(전원/CPU/메모리/커널)은 전부 정상이었던 사고가 있었음
# (CLAUDE.md "TCP 서비스만 응답불능" 절 참고, 원인 확정은 안 됐지만 무선
# 링크가 조용히 죽어있었을 가능성에 대한 방어책으로 적용).
set -uo pipefail

STATUS_FILE=/run/wifi_status.json

# --- SSID 상태를 world-readable 파일로 기록 (실패해도 워치독 로직엔 영향 없게 별도 처리) ---
write_ssid_status() {
    local tmp
    tmp=$(mktemp /run/wifi_status.XXXXXX)
    {
        echo "{"
        local first=1
        for iface in wlan0 wlx00e04c4d96cb; do
            local ssid
            ssid=$(wpa_cli -i "$iface" status 2>/dev/null | sed -n 's/^ssid=//p')
            [ $first -eq 0 ] && echo ","
            first=0
            printf '  "%s": %s' "$iface" "$([ -n "$ssid" ] && printf '"%s"' "$ssid" || echo null)"
        done
        echo
        echo "}"
    } > "$tmp" 2>/dev/null
    mv -f "$tmp" "$STATUS_FILE"
    chmod 644 "$STATUS_FILE"
}
write_ssid_status || true

# --- 게이트웨이(첫 번째 공유기, wlan0) 응답없으면 재연결 ---
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
