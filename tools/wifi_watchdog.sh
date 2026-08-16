#!/bin/bash
# WiFi 게이트웨이가 응답 없으면 재연결(두 인터페이스 다) + SSID를 world-readable
# 파일로 남김 + wlan0 절전모드 강제로 끔.
#
# 2026-08-16 Davi(antigravity) 감사 리포트 제안 반영. 배경: SSH/대시보드만
# 응답불능이고 시스템(전원/CPU/메모리/커널)은 전부 정상이었던 사고가 있었음
# (CLAUDE.md "TCP 서비스만 응답불능" 절 참고, 원인 확정은 안 됐지만 무선
# 링크가 조용히 죽어있었을 가능성에 대한 방어책으로 적용).
#
# 2026-08-16 저녁 추가: 처음엔 wlan0만 감시해서, USB WiFi(wlx00e04c4d96cb)가
# 1시간 넘게 끊긴 채 방치된 적이 있음(원인은 rtw88 Deep LPS 펌웨어 버그 —
# /etc/modprobe.d/rtw88-no-deep-lps.conf로 근본 해결했지만, 다른 원인으로
# 끊길 경우를 위해 두 인터페이스 다 감시하도록 확장). wlan0 절전모드(brcmfmac)도
# 매번 꺼서 응답지연 방지(로봇 제어는 저지연이 배터리 절약보다 우선).
set -uo pipefail

STATUS_FILE=/run/wifi_status.json
IFACES="wlan0 wlx00e04c4d96cb"

# --- SSID 상태를 world-readable 파일로 기록 (실패해도 워치독 로직엔 영향 없게 별도 처리) ---
write_ssid_status() {
    local tmp
    tmp=$(mktemp /run/wifi_status.XXXXXX)
    {
        echo "{"
        local first=1
        for iface in $IFACES; do
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

# --- wlan0 절전모드 강제 off (매번 재확인 — 재연결마다 다시 켜질 수 있음) ---
iw dev wlan0 set power_save off > /dev/null 2>&1 || true

# --- 각 인터페이스의 게이트웨이 응답없으면 재연결 ---
check_and_fix() {
    local iface=$1
    local gw
    gw=$(ip route show default dev "$iface" 2>/dev/null | awk '{print $3; exit}')
    if [ -z "$gw" ]; then
        return 0  # 이 인터페이스에 기본 경로가 없으면(안 붙어있는 등) 조용히 건너뜀
    fi

    if ping -c 1 -W 2 "$gw" > /dev/null 2>&1; then
        return 0
    fi
    sleep 3
    if ping -c 1 -W 2 "$gw" > /dev/null 2>&1; then
        return 0
    fi

    logger -t wifi-watchdog "$iface: 게이트웨이($gw) 응답없음 — 재연결 시도"
    wpa_cli -i "$iface" reassociate > /dev/null 2>&1 || true
}

for iface in $IFACES; do
    check_and_fix "$iface"
done
