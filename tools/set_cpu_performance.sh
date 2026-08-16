#!/bin/bash
# CPU 거버너를 performance로 고정 (2026-08-16, 사용자 결정).
# ondemand(기본값)는 부하에 따라 클럭을 올렸다 내렸다 하는데, 그 램프업
# 지연이 로봇 제어 응답 일관성에 안 좋을 수 있어서 항상 최대클럭으로 고정.
# 트레이드오프: 배터리 소모/발열 증가 — 온도 여유(평소 45~49°C)는 충분해서
# 스로틀링 위험은 낮다고 판단.
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$gov" 2>/dev/null || true
done
