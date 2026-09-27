# 운영체제 설정 (로봇 유닛마다 설치)

2026-09-26 사고: 우분투 자동 업데이트가 주행 중 라이브러리를 갱신하고 서비스를 자동 재시작해
IMU·EKF가 주행 도중 재시작됐다(로봇이 약 2.1 rad/s로 5분 회전). 사용자 결정(A+B):
보안 업데이트는 유지하되 **주행 중엔 아무것도 재시작하지 않고**, 업데이트는 **새벽**에만.

| 파일 | 설치 위치 | 내용 |
|---|---|---|
| `needrestart-50-smart-trash.conf` | `/etc/needrestart/conf.d/50-smart-trash.conf` | 서비스 자동 재시작 금지(목록만), 반영은 재부팅 때 |
| `apt-daily.timer.conf` | `/etc/systemd/system/apt-daily.timer.d/smart-trash.conf` | 목록 갱신 03:00(±15분), 놓치면 건너뜀 |
| `apt-daily-upgrade.timer.conf` | `/etc/systemd/system/apt-daily-upgrade.timer.d/smart-trash.conf` | 자동 설치 03:30(±15분), 놓치면 건너뜀 |

```bash
sudo install -m644 tools/system/needrestart-50-smart-trash.conf /etc/needrestart/conf.d/50-smart-trash.conf
sudo install -Dm644 tools/system/apt-daily.timer.conf /etc/systemd/system/apt-daily.timer.d/smart-trash.conf
sudo install -Dm644 tools/system/apt-daily-upgrade.timer.conf /etc/systemd/system/apt-daily-upgrade.timer.d/smart-trash.conf
sudo systemctl daemon-reload && sudo systemctl restart apt-daily.timer apt-daily-upgrade.timer
systemctl list-timers apt-daily.timer apt-daily-upgrade.timer   # 다음 실행이 03:00 / 03:30 근처인지
```

⚠️ 업데이트가 설치됐는데 재시작하지 않은 서비스는 옛 라이브러리로 계속 돈다. 도크에서 가끔
재부팅해 반영할 것(`needrestart -b`로 재시작 필요 목록 확인 가능).

## 2026-09-27: 자동 업데이트 타이머 끔(사용자 결정 — 로봇 완성 후 다시 켬)

- 로봇은 밤에 꺼져 있어 03:30 예약은 실제로 돌지 않고, 대신 **부팅 후 NTP 시계 점프**(파이 5 RTC 배터리 없음)로
  "이미 지난 예약"이 돼 아침 주행 중에 실행됐다(09-27 08:50). 그래서 타이머 자체를 끔:
  `sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer`
- 수동 업데이트(도크에서 정지 상태일 때): `sudo apt update && sudo apt upgrade` → 끝나면 재부팅
- 다시 켜기: `sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer` (위 03:00/03:30 설정은 남아 있음)
