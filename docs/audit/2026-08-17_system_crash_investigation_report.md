# 📋 [사고 조사 및 긴급 감사 리포트] 연속 시스템 하드 락업(Hard Lockup) 원인 규명 및 파일시스템 손상 긴급 진단

- **문서 번호**: AUDIT-20260817-01
- **조사 일시**: 2026-08-17 02:05 KST
- **작성자**: Davi (Antigravity System Auditor)
- **보고 대상**: Unkle Karrot, Remi (Claude Code)
- **분석 대상**: 2026-08-17 01:07 및 01:37에 발생한 2회의 시스템 응답 불능(다운) 및 강제 재부팅 사고

---

## 1. 🚨 종합 진단 요약 (Executive Summary)

```
[1차 다운]: 2026-08-17 01:07:57 (16분 주행 중, SLAM 맵핑/자율탐색 진행 중)
[2차 다운]: 2026-08-17 01:37:45 (재부팅 후 5.5분 만에, CPU 60% 급증 구간)
[현재 상태]: 2회의 강제 종료 여파로 ext4 파일시스템 손상(EUCLEAN 117) 발생 중 (긴급 조치 필요)
```

| 검증 항목 | 상태 | 팩트 데이터 및 분석 결과 |
|---|---|---|
| **5V 전원 레일** | 🟢 **정상 (5.14~5.18V)** | 다운 직전까지 5V 전압 강하 없음. 언더볼티지 플래그 0건 (전원 차단/방전 가설 기각) |
| **CPU 온도 및 스로틀링** | 🟢 **정상 (52.3~53.5°C)** | 스로틀링 임계치(80°C+)에 한참 못 미침. 열 스트레스 아님 |
| **메모리 및 스왑** | 🟢 **정상 (41~57%)** | 4GB 스왑 사용량 0MB. OOM Killer 또는 스왑 스래싱 없음 |
| **근본 원인 추정** | 🔴 **PL011 UART DMA 락업** | LD06 고속 UART(230400bps) DMA 처리 중 CPU 부하 스파이크 시 커널 인터럽트 하드 락업 |
| **파일시스템 상태** | 🚨 **EMERGENCY (손상 발생)** | `sda2` 지연 블록 할당 실패(`error 117`), `Filesystem state: clean with errors` |

---

## 2. ⏱️ 팩트 타임라인 정밀 재구성 (Timeline of Incidents)

```mermaid
timeline
    title 2026-08-17 연속 다운 및 재부팅 타임라인
    section 1차 세션 (00:51 ~ 01:07)
        00:51 : 시스템 부팅 (Boot -1)
        00:53 : LD06 LiDAR UART DMA 채널 할당 (dma2chan2/3)
        01:04~01:07 : SLAM 맵 저장 #1, #2, #3 정상 완료
        01:07:57 : 로봇 주행 중 (x=-1.69, y=1.04) 갑작스러운 커널 락업 발생
    section 1차 강제 재부팅 (01:32)
        01:32 : 사용자 강제 리셋 후 2차 부팅
        01:33 : LD06 LiDAR UART DMA 재할당
        01:37:03 : CPU 점유율 38% -> 60% 급증, 메모리 41% -> 57% 상승
        01:37:45 : 부팅 5.5분 만에 2차 커널 락업 발생 (로그 끊김)
    section 현재 부팅 (01:49 ~ 현재)
        01:49 : 사용자 2차 강제 리셋 후 부팅 (Boot 0)
        01:49 ~ 현재 : ext4 메타데이터 오염으로 커널에 EUCLEAN(117) 에러 연속 발생
```

---

## 3. 📊 사고 직전 텔레메트리 데이터 분석 (`power_trace.csv`)

### ① 1차 사고 (01:07:57) 직전 5초 데이터
```csv
timestamp,time,5v_voltage,cpu_temp,cpu_usage,freq,memory_usage,throttle_bit
1786896473.0,01:07:53,5.1523,52.9,21.3,2400.03,51.3,0
1786896474.1,01:07:54,5.1898,52.35,20.6,2400.03,51.2,0
1786896475.1,01:07:55,5.1657,54.0,22.1,2400.03,51.3,0
1786896476.1,01:07:56,5.1255,52.35,21.2,2400.02,51.2,0
1786896477.2,01:07:57,5.1402,53.45,30.0,2400.02,50.4,0  <-- 마지막 기록 (즉사)
```
- **상황**: 로봇이 `x=-1.691 → -1.696`, `y=1.054 → 1.044`로 실제 주행 중이었으며, `map_autosave` 맵 저장 직후 정상 동작 상태에서 단 1ms의 징후 없이 즉사.

### ② 2차 사고 (01:37:45) 직전 부하 급증 구간
```csv
timestamp,time,5v_voltage,cpu_temp,cpu_usage,freq,memory_usage,throttle_bit
1786898224.0,01:37:03,5.1871,52.9,38.8,2400.02,43.2,0  <-- CPU 상승 시작
1786898226.1,01:37:06,5.1322,52.35,59.6,2400.02,45.5,0  <-- CPU 59.6% 피크
1786898227.1,01:37:07,5.1791,54.55,60.6,2400.03,47.9,0  <-- CPU 60.6% 피크
...
1786898263.7,01:37:43,5.1563,51.8,16.4,2400.02,57.0,0  <-- 메모리 57% 도달
1786898264.7,01:37:44,5.1348,52.9,10.5,2400.03,57.1,0
1786898265.7,01:37:45,5.1844,52.35,25.4,2400.03,57.1,0  <-- 마지막 기록 (즉사)
```
- **상황**: 재부팅 후 5분경 백그라운드 프로세스 기동 또는 SLAM 노드 초기화로 CPU가 60%까지 치솟고 메모리가 41%에서 57%로 가파르게 상승한 직후 커널 락업 발생.

---

## 4. 🔍 근본 원인 심층 분석 (Root Cause Deep-Dive)

### ❌ 기각된 가설들 (Counter-Evidence)
1. **배터리 저전압 / 전원 차단 가설**:
   - 5V 레일 전압은 5.14~5.18V로 완벽히 유지됨.
   - PMIC 저전압 스로틀링 비트는 `0`으로 전원 부족 증거 전무.
2. **CPU 과열(Thermal) 가설**:
   - 다운 시점 온도는 52~53.5°C로 정상 작동 범위(안전 마진 30°C 이상).
3. **OOM / 메모리 고갈 가설**:
   - 물리 메모리 여유 1.7GB 이상, 4GB 스왑 사용량 0B.

### 🔴 최유력 근본 원인: PL011 UART DMA 서브시스템의 커널 스핀락 데드락 (Kernel Hard Lockup)
1. **장치 특성**: LD06 LiDAR는 230400bps의 높은 보드레이트로 초당 23,040바이트의 데이터를 쉼 없이 GPIO UART(`/dev/ttyAMA0`)로 쏟아붓고 있습니다.
2. **커널 드라이버 거동**:
   - 커널은 ttyAMA0 포트에 대해 `pl011-axi (dma2chan2 TX / dma2chan3 RX)` 하드웨어 DMA 채널을 열어 패킷을 수신합니다.
   - 라즈베리파이 5(BCM2712) 리눅스 6.8 커널의 PL011 DMA 드라이버는 과거 로그에서도 관측된 바와 같이 `dma2chan2: BUG: dma2chan2, IRQ with no descriptors` 및 `ttyAMA0: 40 input overrun(s)` 버그를 내포하고 있습니다.
3. **락업 메커니즘**:
   - SLAM Toolbox 연산이나 시스템 프로세스 실행으로 멀티코어 CPU 부하가 급증(01:37에 60% 도달)하는 순간, DMA 인터럽트 핸들러가 처리 시간을 제때 할당받지 못함.
   - DMA 링버퍼 오버런 상태에서 IRQ 인터럽트 핸들러 내 스핀락(Spinlock) 경합 또는 데드락이 발생하여 커널 인터럽트 컨텍스트가 멈춤.
   - 이로 인해 커널이 패닉 메시지나 디스크 로그를 쓸 기회조차 갖지 못하고 CPU 코어가 즉시 하드 락업(Hard Lockup)에 빠짐.

---

## 5. ⚠️ 2차 파생 긴급 이슈: 파일시스템 손상 (`EUCLEAN 117`)

2회에 걸친 갑작스러운 전원 차단으로 인해 ext4 지연 블록 할당(Delayed Allocation) 구조체가 손상되었습니다.

### 🔴 현재 커널 에러 증거
```text
EXT4-fs (sda2): Delayed block allocation failed for inode 130602 at logical offset 820 with max blocks 1 with error 117
EXT4-fs (sda2): This should not happen!! Data will be lost
EXT4-fs error (device sda2): ext4_lookup:1866: inode #1305683: comm find: deleted inode referenced: 1303318
EXT4-fs error (device sda2) in ext4_free_inode:363: Corrupt filesystem
tune2fs -l /dev/sda2: Filesystem state: clean with errors
```
- **영향 파일**: `power_trace.csv` (inode 130602), `/var/log/kern.log` (inode 3381), `/var/log/syslog` (inode 3439).
- **위험도**: 🚨 **CRITICAL**. 시스템이 켜져 있는 동안 로그를 쓸 때마다 에러가 발생하며, 추가 디스크 손상 위험이 있습니다.

---

## 6. 🛠️ 감사관 권고 조치 사항 (Action Items)

### 🚨 [긴급 조치 - 즉시 수행] 파일시스템 정밀 복구 (`fsck`)
파일시스템 메타데이터를 즉시 수선해야 데이터 손실 및 I/O 블로킹을 방지할 수 있습니다.
```bash
# 1. 다음 부팅 시 루트 파일시스템 강제 검사 플래그 설정
sudo tune2fs -C 9999 -c 1 /dev/sda2

# 2. 시스템 안전 재부팅 (부팅 과정에서 fsck 자동 복구 수행)
sudo reboot
```

---

### 🛡️ [근본 해결 - 방지책] LiDAR 통신 경로 DMA 분리 및 안정화
1. **방법 A (권장): USB 시리얼 컨버터로 LiDAR 포트 이전**
   - LD06 LiDAR의 신호선을 GPIO UART 대신 USB-TTL 변환 젠더(`/dev/ttyUSB1` 등)로 이전.
   - USB ACM/FTDI 드라이버 계층을 사용함으로써 취약한 SoC 내부 PL011 GPIO DMA 인터럽트 충돌을 원천 차단.
2. **방법 B: PL011 UART DMA 비활성화 (PIO 모드 전환)**
   - `/boot/firmware/config.txt` 또는 `cmdline.txt` 설정을 통해 ttyAMA0의 DMA를 끄고 순수 인터럽트/PIO 모드로 구동하도록 설정.

---

### 🛡️ [시스템 하드닝] 커널 락업 시 자동 패닉/재부팅 설정
시스템이 응답 불능 상태로 무한 정체되지 않고 크래시 덤프를 남기며 자동 복구되도록 커널 파라미터 보강:
```bash
# /etc/sysctl.d/99-crash-watchdog.conf 생성
kernel.panic = 10
kernel.panic_on_oops = 1
kernel.hardlockup_panic = 1
kernel.softlockup_panic = 1
```
