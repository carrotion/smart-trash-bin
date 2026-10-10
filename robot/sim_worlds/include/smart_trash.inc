# 스마트 쓰레기통 1호기 — Stage용 로봇 모델 (2026-10-10)
#
# ⚠️ 08-22~10-10의 Stage 시뮬은 전부 pioneer2dx_with_laser(차체 0.44 × 0.38 m, 라이다 앞 0.15 m·270°·5 m)로 돌았다.
#    우리 로봇은 0.21 × 0.20 m급이고 주행기·planner·충돌 감시가 그 크기를 전제로 하므로, 그 결과들은 좁은 곳에서
#    실제와 다르다(폭 0.38 m 차체가 0.45 m 문틈을 지나야 했다). 치수 근거: docs/hardware_inventory.md
#
# 모델에 없는 것(시뮬 사각): 스키드 조향 미끄럼, 뒤로 나온 케이블(차체 뒤 최대 9 cm), 라이다 자기 차폐(뒤쪽 약 25° 등),
#    라이다 근접 한계(13.5 cm보다 가까우면 0.08 m로 읽힘), 엔코더 오독

define ld06 ranger
(
  sensor(
    pose [ 0 0 0 0 ]
    size [ 0.04 0.04 0.03 ]
    range [ 0.02 12.0 ]      # LD06 12 m
    fov 360.0
    samples 455              # 실물 스캔 재분할(scan_resample_node)과 같은 칸 수
    color_rgba [ 0 0 1 0.15 ]
  )
  model
  (
    pose [ 0.0 0.0 -0.1 0.0 ]
    size [ 0.04 0.04 0.03 ]
    color "blue"
  )
)

define smart_trash_base position
(
  color "red"
  drive "diff"
  gui_nose 1
  obstacle_return 1
  ranger_return 0.5
  blob_return 1
  fiducial_return 1

  localization "odom"
  odom_error [ 0.05 0.05 0.0 0.1 ]   # pioneer 정의와 같은 값(비교를 위해 그대로)

  velocity_bounds [-1 1 0 0 0 0 -45.0 45.0 ]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -45 45.0 ]

  # 길이 0.22(Nav2 footprint ±0.11) × 폭 0.21(뒷바퀴 바깥면끼리, 10-10 실측) × 높이 0.20
  size [0.22 0.21 0.20]
  origin [0 0 0 0]          # base_link = 차체 가운데(축간 중앙)
  mass 2.0
)

define smart_trash_bot smart_trash_base
(
  ld06(pose [ 0.054 0 0 0 ])  # 라이다는 base_link 앞 5.4 cm(앞바퀴 축 선상)
)
