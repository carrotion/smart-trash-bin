# 🛡️ 시스템 감사 및 개선 제안 리포트 (System Audit & Improvement Reports)

> 2026-11-07: 파이에서 일하는 Claude Code의 이름이 **Remi → Rodi**로 바뀌었다(Unkle Karrot 결정 — "Remi"는 이미 다른 곳에 있음). 아래 지난 기록의 Remi = 지금의 Rodi.

이 디렉터리는 프로젝트의 시스템 안정성, 하드웨어 연동, 서비스 라이프사이클, 전력 최적화 및 사고 로그 분석을 정기적/이슈 발생 시마다 점검하여 기록하는 **감사 리포트 및 개선 제안서** 보관소입니다.

## 📁 리포트 목록

- [2026-08-16: UAS 모드 해제 후 시스템 안정성 및 서비스 로그 감사 리포트](2026-08-16_uas_storage_and_service_audit.md)
  - ASMedia USB-SATA 드라이버 전환(`usb-storage.quirks`) 검증
  - systemd 서비스 종료 시 90초 SIGTERM 지연 분석 및 조치 사항
  - ROS 2 Jazzy 센서 및 오도메트리 토픽 통신 상태 점검

- [2026-08-16: 배터리 전력 최적화 및 서비스 안정성 개선 제안 리포트](2026-08-16_power_and_service_improvement_proposal.md)
  - 3S1P 배터리 전력 소모량 정밀 산출 및 상태별 지속시간 분석
  - 상태 머신(FSM) 기반 LiDAR/센서 동적 On/Off 제어 방안
  - 대시보드 웹서버 Graceful Shutdown(SIGTERM) 코드 개선 가이드
  - 3S2P 배터리 확장 및 I2C 전압 모니터링 로드맵

- [2026-08-16: [사고 조사] SSH 무응답 및 대시보드 단절 현상 원인 규명 리포트](2026-08-16_ssh_unresponsive_incident_report.md)
  - 14:26~14:43 SSH/대시보드 단절 시 시스템 및 I/O 상태 정밀 분석
  - Wi-Fi Silent Stall 메커니즘 및 팩트 타임라인 검증
  - 자동 자가 복구 워치독 스크립트 대책 제시

- [2026-08-16: [분석] 맵핑 주행 중 조종 명령 유실(버튼 씹힘) 원인 및 자원 경합 정밀 분석](2026-08-16_mapping_teleop_contention_audit.md)
  - 16:02~16:17 SLAM 맵핑 성과 분석 (우측 쏠림 보정 및 직교성 검증)
  - UI 입력부터 모터 시리얼 I/O까지 4대 자원 경합(Serial Lock 80% 독점, 큐 덮어쓰기, 프론트엔드 드랍) 규명
  - 조종 반응성 향상을 위한 즉시 해결책 제시

- **2026-08-17: [사고 조사] 연속 시스템 하드 락업 원인 규명 및 파일시스템 손상 긴급 진단** (`2026-08-17_system_crash_investigation_report.md`)
  - 01:07 및 01:37 2회 연속 다운 직전 텔레메트리(`power_trace.csv`) 정밀 분석
  - 5V 전압(5.14~5.18V)·온도(52~53°C) 정상 확인 (전원/방전 가설 기각)
  - PL011 UART DMA 서브시스템의 커널 스핀락 데드락 메커니즘 도출
  - 2차 강제 종료로 인한 ext4 파일시스템 손상(`EUCLEAN 117`) 긴급 복구(`fsck`) 및 방지책 권고

- **2026-08-17: [기술 조사] 4WD 스키드-스티어 마찰·슬립 극복 및 정지형 지자기 센서(ZUPT) 융합 보고서** (`2026-08-17_motion_slip_mitigation_and_odometry_optimization_report.md`)
  - 4WD 스키드-스티어 기구학/ICR 동역학 및 회전 저항($M_{scrub}$) 모델링
  - 축간거리(Wheelbase, $L$) 축소의 수학적 증명 및 2호기 하드웨어 설계 반영 ($L/W \approx 0.5$)
  - 실내 마루 보호용 전륜 2개 비대칭 테이핑(Tape Hack) 실증 방안
  - 가감속 Slew-rate 램프 및 슬립 감지 적응형 EKF 공분산(Adaptive Covariance)
  - 정지 판정(ZUPT) 기반 3D 틸트 보정 지자기 센서 헤딩 리셋 파이프라인
  - IEEE / IJRR / ASCE 관련 권위 학술 논문 8편 분석 및 인용

- **2026-08-17: [기술 평가] 자율탐색·오도메트리·센서 신기술 트렌드 종합 평가 보고서** (`2026-08-17_autonomous_navigation_and_sensor_technology_evaluation_report.md`)
  - Remi(SW 패키지: `frontier_exploration_ros2`, `rf2o_laser_odometry`, `MPPI`) 및 Davi(HW 센서/기구) 리서치 통합
  - 무비용·극저연산(0.9ms) 라이다 오도메트리(`rf2o`) + 상단 중앙 IMU 지자기 융합 아키텍처
  - 2호기 축간거리 축소 및 1~2만원대 고가성비 부품(광학플로우 PMW3901, 멀티존 ToF) 로드맵 수립
  - 4단계 순차적 검증 및 도입 파이프라인 확정


