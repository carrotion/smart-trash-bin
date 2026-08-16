# 🛡️ 시스템 감사 및 개선 제안 리포트 (System Audit & Improvement Reports)

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
  - Wi-Fi 절전 모드(`power save enabled`) 및 USB 3.0 2.4GHz 노이즈 간섭 규명
  - Wi-Fi Power Save 비활성화 및 무선 연결 안정화 대책 제시
