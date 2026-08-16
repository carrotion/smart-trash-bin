# 📋 [사고 조사 리포트] SSH 무응답 및 대시보드 단절 현상 원인 규명

- **문서 번호**: AUDIT-20260816-03
- **조사 일시**: 2026-08-16 15:20 KST
- **작성자**: Davi (Antigravity System Auditor)
- **보고 대상**: Unkle Karrot, Remi (Claude Code)
- **사고 개요**: 2026-08-16 14:26 ~ 14:43 사이 Pi 5는 녹색 LED 점등 및 SSD 깜박임 상태를 유지했으나, 외부에서 SSH 접속 불가 및 웹 대시보드 무응답 발생. 전원 버튼으로 정상 종료 후 재부팅함.

---

## 1. 종합 결론 (Executive Summary)

> **핵심 판정**: **시스템(OS/커널) 다운이나 프로세스 크래시가 아닌, 2.4GHz Wi-Fi 무선 링크의 '침묵 단절(Silent Drop)' 현상**

- **근거**: 
  1. `power_trace.csv` 확인 결과 단절 시간(14:26~14:43) 동안 시스템은 **1초 간격으로 정상 기록(5.20V 전압 안정, CPU 10%, 메모리 63%, 발열 46°C, 스로틀링 0)** 중이었음.
  2. `map_autosave.py`가 14:41:55(저장 #78)까지 60초 주기로 **정상 디스크 I/O를 수행**하고 있었음 (사용자가 목격한 'SSD 깜박임'의 실체).
  3. 전원 버튼을 눌렀을 때 `systemd-logind`가 신호를 정상 수신하여 모든 프로세스와 파일시스템을 **100% 정상 종료(Graceful Shutdown)**함 (커널 락업 아님).
- **단절 원인 (복합 요인)**:
  - **요인 1**: Wi-Fi 드라이버의 **`power save enabled` (무선 절전 모드)** 기본 활성화로 인한 수신 패킷 드랍 및 라우터 ARP 테이블 만료.
  - **요인 2**: **USB 3.0 SSD의 2.4GHz 대역 RF 노이즈 간섭(EMI)**. 현재 Wi-Fi가 2.4GHz(`freq=2427`, Ch4)로 연결되어 있어 SSD 고속 I/O 발생 시 수신 감도가 급격히 저하됨.

---

## 2. 타임라인 및 로그 분석 (Timeline Analysis)

```
[10:26:03] 부팅 완료 (UAS 해제 usb-storage 정상)
[10:38:46] 커널 로그: brcmfmac: power save enabled (Wi-Fi 절전 모드 자동 켜짐)
[11:32:17] SSH 세션 연결 (carrotian)
[14:25:27] diff_drive_node 정상 기동, SLAM 맵핑 진행 중
[14:25:56] diff_drive_node: cmd_vel 타임아웃 경고 발생 (웹 조종 명령 수신 끊김)
  ─── [이 시점부터 외부 네트워크(SSH / 대시보드 HTTP) 단절 발생] ───
[14:26:15] map_autosave: 맵 저장 #62 (SSD 정상 쓰기)
[14:30:25] map_autosave: 맵 저장 #67 (SSD 정상 쓰기)
[14:35:35] map_autosave: 맵 저장 #72 (SSD 정상 쓰기)
[14:41:55] map_autosave: 맵 저장 #78 (SSD 정상 쓰기)
[14:43:02] power_trace.csv: 전압 5.213V, CPU 10.2%, 메모리 63.5% 정상 로깅
[14:43:03] Unkle Karrot이 본체 전원 버튼 입력 → systemd-logind가 정상 인지
[14:43:04] 서비스 및 유저 세션 정상 순차 종료 (Graceful Shutdown)
[14:44:33] 파일시스템 동기화 및 전원 안전 차단 완료
```

---

## 3. 심층 원인 분석 (Root Cause Details)

### 3.1 `brcmfmac` Wi-Fi 절전 모드 (Power Save) 문제
- Raspberry Pi 5의 Broadcom Wi-Fi 칩셋(`BCM43455`)은 기본적으로 `power save enabled` 상태로 동작합니다.
- 절전 모드에서는 AP(공유기)와의 비콘 주기 사이에 RF 수신기를 일시 차단(Sleep)합니다.
- 이 상태에서 네트워크 트래픽이 잠시 뜸해지면 공유기의 ARP 테이블에서 Pi의 IP/MAC 매핑이 만료되거나 수신 SYN 패킷을 놓쳐, **외부에서는 핑(Ping), SSH, HTTP 접속이 전부 Time-out으로 죽지만 Pi 내부에서는 연결이 유지된 것으로 착각하는 현상**이 발생합니다.

### 3.2 USB 3.0 SuperSpeed와 2.4GHz Wi-Fi 간 전자기 간섭 (EMI)
- USB 3.0은 5Gbps 전송 시 **2.4GHz ~ 2.5GHz 주파수 대역에 강한 광대역 RF 노이즈**를 방사합니다 (Intel 및 Raspberry Pi 공식 문서에 등재된 하드웨어 특성).
- 현재 외장 SSD(Ugreen ASMedia)가 USB 3.0 포트에 연결되어 있고, Wi-Fi는 **2.4GHz (`freq=2427`, 802.11n)**에 물려 있습니다.
- SLAM 맵 데이터 저장 및 트레이스 로깅으로 SSD 쓰기가 지속적으로 일어나는 상황에서, 로봇이 이동하며 Wi-Fi 신호 강도가 조금이라도 낮아지면 USB 3.0 노이즈가 Wi-Fi 신호를 완전히 덮어써버려 패킷 수신이 불가능해집니다.

---

## 4. 재발 방지 권장 조치 사항 (Action Plan)

### [즉시 조치 1] Wi-Fi 절전 모드 영구 비활성화 (P1)
Wi-Fi 파워 세이빙을 끄면 패킷 드랍과 무응답 프리징이 즉시 해결됩니다.

```bash
# 1. iw 도구 설치 (필요시)
sudo apt-get update && sudo apt-get install -y iw

# 2. 부팅 시 자동 적용을 위한 systemd 서비스 생성
sudo tee /etc/systemd/system/wifi-powersave-off.service << 'EOF'
[Unit]
Description=Disable WiFi Power Save on wlan0
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 3. 서비스 활성화 및 즉시 적용
sudo systemctl daemon-reload
sudo systemctl enable --now wifi-powersave-off.service
```

### [즉시 조치 2] SSH 서버 Keep-Alive 설정 (P1)
공유기 NAT/방화벽 타임아웃으로 인한 세션 끊김 방지:
`/etc/ssh/sshd_config.d/keepalive.conf` 파일 생성:
```ini
ClientAliveInterval 30
ClientAliveCountMax 3
```

### [권장 조치 3] 5GHz Wi-Fi 연결 전환 또는 물리적 차폐 (P2)
- 공유기에서 **5GHz 대역(SSID)**을 지원할 경우, netplan 설정을 5GHz로 연결하면 USB 3.0 노이즈 간섭을 원천 차단할 수 있습니다.
- 5GHz 사용이 어렵다면 SSD 케이블을 차폐 케이블로 교체하거나 Pi 5 본체와 SSD 간 거리를 10cm 이상 이격 배치합니다.

### [안전망 조치 4] 무선 네트워크 자동 자가 복구 스크립트 (P2)
게이트웨이(192.168.0.1)로 3분 이상 통신이 두절되면 Wi-Fi 인터페이스를 자동 재연결하는 헬스체크 데몬 배치.

---

## 5. 결론 및 안심 메시지
- 이번 현상은 **OS 커널 다운, 메모리 누수, 배터리 저전압 등의 하드웨어 결함이 아닙니다.**
- 시스템은 100% 정상 동작 중이었으며 무선 통신만 수신 불능 상태에 빠졌던 것이므로, **Wi-Fi 절전 모드 해제(P1)** 조치만으로 안정성을 대폭 향상시킬 수 있습니다.
