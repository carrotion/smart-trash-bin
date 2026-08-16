#!/bin/bash
# WiFi 인터페이스 감시/자동복구 + SSID 상태 파일 기록.
#
# 2026-08-16 Davi(antigravity) 감사 리포트 제안 반영. 배경: SSH/대시보드만
# 응답불능이고 시스템(전원/CPU/메모리/커널)은 전부 정상이었던 사고가 있었음
# (CLAUDE.md "TCP 서비스만 응답불능" 절 참고, 원인 확정은 안 됐지만 무선
# 링크가 조용히 죽어있었을 가능성에 대한 방어책으로 적용).
#
# 2026-08-16 저녁 1차 확장: wlan0만 보던 걸 USB 어댑터까지 감시하도록 확장.
#
# 2026-08-16 밤 2차 확장 — USB WiFi가 또 죽어서 실제 구멍 2개를 발견해 메움:
#   (1) 절전모드를 wlan0에만 껐음. USB(rtw88_8821cu)는 power_save=on인 채로
#       남아서 "firmware failed to leave lps state" 펌웨어 버그가 재발함.
#       modprobe의 disable_lps_deep=1은 Deep LPS만 막지 일반 LPS는 안 막음 —
#       인터페이스별 `iw set power_save off`가 별도로 필요하다.
#   (2) check_and_fix가 "기본 경로 없으면 조용히 스킵"이라, 아예 끊어진(SCANNING
#       상태로 멈춘) 인터페이스는 영원히 손도 안 댔음. 정확히 그 상태로 몇십 분
#       방치됐던 것. 이제 링크 상태를 직접 보고, 연속 실패가 쌓이면 드라이버
#       모듈을 재로드한다(이전에 수동으로 해서 복구됐던 그 절차의 자동화).
#
# ⚠️ 모듈 재로드는 USB 어댑터에만 적용한다. wlan0(brcmfmac)은 SSH가 타고 들어오는
#    경로라, 이걸 재로드하면 작업 중인 세션이 끊긴다.
set -uo pipefail

STATUS_FILE=/run/wifi_status.json
STATE_DIR=/run/wifi_watchdog
IFACE_BUILTIN=wlan0
IFACE_USB=wlx00e04c4d96cb
IFACES="$IFACE_BUILTIN $IFACE_USB"

# 연속 몇 번(=몇 주기) 링크가 안 붙어 있으면 모듈 재로드까지 갈지.
# 타이머가 30초 간격이므로 3 = 약 90초 방치 후 개입.
RELOAD_AFTER_FAILURES=3

mkdir -p "$STATE_DIR"

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

# --- 절전모드 강제 off: 모든 무선 인터페이스에 매번 재적용 ---
# (재연결/모듈 재로드 때마다 기본값 on으로 되돌아오므로 매 주기 다시 꺼야 함)
for iface in $IFACES; do
    [ -e "/sys/class/net/$iface" ] || continue
    iw dev "$iface" set power_save off > /dev/null 2>&1 || true
done

# --- 드라이버 모듈 재로드(USB 전용 최후수단) ---
reload_driver() {
    local iface=$1
    local mod
    mod=$(basename "$(readlink -f "/sys/class/net/$iface/device/driver/module" 2>/dev/null)" 2>/dev/null)
    if [ -z "$mod" ] || [ "$mod" = "." ]; then
        logger -t wifi-watchdog "$iface: 드라이버 모듈명을 못 찾아 재로드 생략"
        return 1
    fi
    logger -t wifi-watchdog "$iface: 링크 복구 실패 누적 — 드라이버 모듈($mod) 재로드"
    modprobe -r "$mod" 2>/dev/null || true
    sleep 2
    modprobe "$mod" 2>/dev/null || true
    sleep 5
    # 재로드하면 인터페이스가 새로 올라오므로 netplan이 설정을 다시 입히게 함
    netplan apply > /dev/null 2>&1 || true
}

# --- 인터페이스별 점검 ---
# 반환 상태에 따라 실패 카운터를 올리거나 리셋한다.
check_and_fix() {
    local iface=$1
    local fail_file="$STATE_DIR/$iface.fail"

    if [ ! -e "/sys/class/net/$iface" ]; then
        return 0  # 물리적으로 안 꽂혀 있음 — 워치독이 할 수 있는 게 없음
    fi

    local wpa_state
    wpa_state=$(wpa_cli -i "$iface" status 2>/dev/null | sed -n 's/^wpa_state=//p')

    # (A) AP에 아예 안 붙어 있는 상태 (SCANNING/DISCONNECTED/INACTIVE 등).
    #     예전엔 "기본 경로 없음"으로 그냥 스킵되던 케이스 — 이제 여기서 잡는다.
    if [ "$wpa_state" != "COMPLETED" ]; then
        local fails
        fails=$(( $(cat "$fail_file" 2>/dev/null || echo 0) + 1 ))
        echo "$fails" > "$fail_file"
        logger -t wifi-watchdog "$iface: 미연결 상태(wpa_state=${wpa_state:-unknown}), 연속 ${fails}회 — 재연결 시도"
        wpa_cli -i "$iface" reconnect > /dev/null 2>&1 || true

        # 재연결로도 안 살아나면 모듈 재로드(USB만).
        if [ "$fails" -ge "$RELOAD_AFTER_FAILURES" ] && [ "$iface" = "$IFACE_USB" ]; then
            reload_driver "$iface"
            echo 0 > "$fail_file"
        fi
        return 0
    fi

    # (B) 붙어는 있는데 게이트웨이가 응답 없는 경우 — 기존 로직 유지.
    local gw
    gw=$(ip route show default dev "$iface" 2>/dev/null | awk '{print $3; exit}')
    if [ -z "$gw" ]; then
        echo 0 > "$fail_file"
        return 0
    fi

    if ping -c 1 -W 2 "$gw" > /dev/null 2>&1; then
        echo 0 > "$fail_file"
        return 0
    fi
    sleep 3
    if ping -c 1 -W 2 "$gw" > /dev/null 2>&1; then
        echo 0 > "$fail_file"
        return 0
    fi

    local fails
    fails=$(( $(cat "$fail_file" 2>/dev/null || echo 0) + 1 ))
    echo "$fails" > "$fail_file"
    logger -t wifi-watchdog "$iface: 게이트웨이($gw) 응답없음, 연속 ${fails}회 — 재연결 시도"
    wpa_cli -i "$iface" reassociate > /dev/null 2>&1 || true

    if [ "$fails" -ge "$RELOAD_AFTER_FAILURES" ] && [ "$iface" = "$IFACE_USB" ]; then
        reload_driver "$iface"
        echo 0 > "$fail_file"
    fi
}

for iface in $IFACES; do
    check_and_fix "$iface"
done
