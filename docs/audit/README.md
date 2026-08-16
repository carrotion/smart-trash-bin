# 🛡️ 시스템 감사 리포트 (System Audit Reports)

이 디렉터리는 프로젝트의 시스템 안정성, 하드웨어 연동, 서비스 라이프사이클 및 로그 분석을 정기적/이슈 발생 시마다 점검하여 기록하는 **감사 리포트(Audit Reports)** 보관소입니다.

## 📁 리포트 목록

- [2026-08-16: UAS 모드 해제 후 시스템 안정성 및 서비스 로그 감사](2026-08-16_uas_storage_and_service_audit.md)
  - ASMedia USB-SATA 드라이버 전환(`usb-storage.quirks`) 검증
  - systemd 서비스 종료 시 90초 SIGTERM 지연 분석 및 조치 사항
  - ROS 2 Jazzy 센서 및 오도메트리 토픽 통신 상태 점검
