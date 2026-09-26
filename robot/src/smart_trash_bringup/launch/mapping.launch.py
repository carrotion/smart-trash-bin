import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
LD06 LiDAR + SLAM Toolbox(online async)로 2D 맵을 만드는 bringup launch.

⚠️ 2026-08-19 LD06을 **GPIO UART -> USB 어댑터로 이전**했다.
(그전까지는 /dev/ttyAMA0 직결. ttyAMA0의 PL011 DMA 버그로 input overrun이 반복돼
 감사 보고서가 USB 이전을 권고했고, 그걸 반영한 것 — docs/audit/2026-08-17 참고)

포트는 **udev 고정 이름 `/dev/lidar`** 를 쓴다(tools/udev/99-smart-trash.rules).
ttyUSB 번호는 부팅 시 인식 순서로 정해져서 모터 드라이버와 뒤바뀔 수 있기 때문이다.

서브모듈 ldlidar_stl_ros2의 launch/ld06.launch.py는 port_name이 하드코딩돼 있어
여기서 ldlidar_stl_ros2_node를 직접 구성한다(그 launch 파일을 include하지 않음).

odom->base_link는 아직 Week 3 diff-drive 오도메트리 노드가 없으므로
`publish_placeholder_odom`(기본 true)으로 identity tf를 임시 발행함 —
실제 휠 오도메트리 노드가 생기면 이 launch 인자를 false로 끌 것.

✅ 2026-08-17 라이다를 **하판에 바로 세워** 재장착 — 아래 상판 시절 보정은 해제됨.
   `laser_scan_dir`를 True(기본)로 되돌렸고, 실측으로 검증함: 로봇 왼쪽 30cm에
   평평한 물체를 놓으니 스캔에서 90° 주변(65.8~115.8°)에 잡힘 = ROS 규약
   (0°=정면, +90°=왼쪽)과 일치. z 높이도 실측 0.085m로 교체(이전 0.18은 placeholder).
   효과: 총 차폐 85° -> 41°, 정면 ±30° 차폐 7% -> 5%, 막힘 구간 10개 -> 6개.
   바퀴(지름 0.067m) 윗면 위로 스캔면이 올라가면서 바퀴 차폐가 사라진 것이 주효.
   남은 차폐 25°는 후방 두 구간(163~174°, 191~203°)에 몰려 있음(케이블 추정).

--- 아래는 상판 거꾸로 마운트 시절의 기록 (참고용, 현재는 해당 없음) ---
✅ LD06 상판 거꾸로 마운트 보정 (2026-08-15, 실측 캘리브레이션):
로봇을 방 모서리(전면벽+왼쪽벽)에 놓고 /scan에서 두 벽이 찍히는 각도를 실측함.
laser_scan_dir=True(반시계, 기본 가정)일 땐 왼쪽 벽이 -90°(270°) 쪽에 나와서
전면(0°) 대비 반대 방향으로 뒤집혀 있었음 — 상판 거꾸로 마운트로 인한 반전.
laser_scan_dir=False(시계방향)로 바꾸니 왼쪽 벽이 +90° 쪽으로 정확히 이동(실측
265°→90°, 미러 변환 예측치 95°와 거의 일치) — 전면=0°/왼쪽=+90° 정상 규약과 맞음.
이 보정은 드라이버 레벨(laser_scan_dir)에서 이미 각도 해석을 바로잡아주므로,
정적 tf에 별도 롤/피치 180도 회전은 필요 없음(아래 tf는 회전 없이 z높이만).
(z높이 placeholder 문제는 2026-08-17 실측 0.085m로 해결됨 — 위 참고)
"""


def generate_launch_description():
    bringup_share = get_package_share_directory('smart_trash_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params_file = LaunchConfiguration('slam_params_file')
    publish_placeholder_odom = LaunchConfiguration('publish_placeholder_odom')
    lidar_port = LaunchConfiguration('lidar_port')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            bringup_share, 'config', 'mapper_params_online_async.yaml'),
        description='SLAM Toolbox 파라미터 yaml 경로')

    declare_placeholder_odom_cmd = DeclareLaunchArgument(
        'publish_placeholder_odom',
        default_value='false',
        description=(
            'true면 odom->base_link를 identity tf로 임시 발행. '
            '2026-08-15부터 smart_trash_motion의 diff_drive_node가 실제 오도메트리로 '
            '이 tf를 발행하므로 기본 false — true로 켜면 TF가 중복 발행되어 깨짐.'))

    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/lidar',
        description=('LD06 시리얼 포트. udev 고정 이름(tools/udev/99-smart-trash.rules). '
                     '2026-08-19 GPIO UART -> USB 어댑터로 이전'))

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            # 드라이버는 원시 스캔을 scan_raw로 내보내고, scan_self_filter가
            # 로봇 자기 구조물 반사(0.06~0.10m)를 걸러서 scan으로 재발행한다.
            # 이렇게 하면 SLAM/Nav2/대시보드는 전부 기존처럼 scan만 보면 됨.
            'topic_name': 'scan_raw',
            'frame_id': 'base_laser',
            'port_name': lidar_port,
            'port_baudrate': 230400,
            # 상판 거꾸로 마운트 보정값, 2026-08-15 실측 캘리브레이션으로 확정
            # ⚠️ 이 값의 이력 (2026-08-17):
            #  - 상판 거꾸로 마운트 시절: False (좌우 반전 보정용)
            #  - 하판 이전 직후: 라이다 삼각형(정면 표시)이 **뒤를 향하게** 오설치돼
            #    있었고 False까지 남아 있어서, 변환이 θ_스캔 = 180° − θ_실제 가
            #    됐다(실측). 좌우는 보존되고 앞뒤만 뒤집혀서, "왼쪽에 물체 놓기"
            #    검증으로는 안 걸러지고 **로봇을 회전시켜야** 드러났다.
            #  - 현재: 삼각형이 정면을 보도록 물리적으로 바로잡음 -> True(기본값)
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }]
    )

    # 로봇 자기 구조물 반사 제거 (scan_raw -> scan).
    # 안 넣으면 로봇 자기 위치가 맵에 장애물로 박혀서 Nav2가 어떤 경로도 못 낸다.
    #
    # 2026-08-17: 직접 만든 scan_filter_node(거리 임계값 방식)를 버리고 표준 패키지
    # laser_filters의 **StaticLaserScanPolygonFilter**로 교체했다.
    #  - 임계값 방식은 각도를 무시해서, 0.15m로 자르면 자기 구조물(0.080m)뿐 아니라
    #    **벽에 닿기 직전 구간(0.105~0.15m)까지 같이 지웠다.** 그래서 벽이 코스트맵에서
    #    사라지고 로봇이 계속 밀어붙였다(실주행 관측).
    #  - 다각형 방식은 기하로 갈린다: 차체(22x22cm) 안쪽 반사는 실제 장애물일 수 없다.
    #  - 실측 검증: 자기 구조물 스캔당 40.5개 -> 0개, 0.105~0.15m 구간은 손실 0.
    # 설정과 근거는 config/scan_self_filter.yaml 참고.
    scan_self_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_self_filter',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'scan_self_filter.yaml')],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan_selffilt')],
    )

    # 점 개수 고정 (scan_selffilt -> scan). 2026-09-26: LD06은 한 바퀴에 454~456점을
    # 내는데 slam_toolbox는 첫 스캔과 개수가 다른 스캔을 버려서, 실제 매핑에서 스캔의
    # 약 38%가 버려지고 있었다. 455칸으로 재분할(칸마다 최소 거리)해 전부 쓰게 한다.
    # 녹화 재생 A/B/C에서 버림 0건 확인. 근거는 docs/todo.md 1e.
    scan_resample = Node(
        package='smart_trash_sensors',
        executable='scan_resample_node',
        name='scan_resample',
        output='screen',
        parameters=[{'num_bins': 455}],
        remappings=[('scan_in', 'scan_selffilt'), ('scan_out', 'scan')],
    )

    # 회전 보정은 laser_scan_dir(위)에서 이미 처리돼서 롤/피치는 0 그대로 둠.
    # z 높이: 2026-08-17 실측 0.085m (하판 이전 후, 사용자 측정).
    # 그 전까지 쓰던 0.18은 ld06.launch.py 기본값을 그대로 가져온 placeholder였고
    # 실제와 2배 넘게 차이났다(상판 시절 실제값도 0.095 정도로 추정).
    # 참고: 바퀴 지름이 0.067m라 윗면이 약 0.067m — 스캔면(0.085m)이 그 위로
    # 올라가면서 바퀴 차폐가 사라졌다(총 차폐 85° -> 41°).
    base_link_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld06',
        # 라이다 삼각형(정면 표시)을 로봇 정면에 맞춰 물리적으로 장착했으므로
        # 회전 보정 불필요 — z 높이만(실측 0.085m). 인자 순서는 x y z yaw pitch roll.
        # 2026-08-19 재배치: 라이다를 **앞바퀴 축 선상**으로 전방 이동(정면 시야 확보).
        # base_link는 차동구동의 회전 중심 = 축간 중앙이고 축간거리가 108mm이므로
        # 라이다는 그 절반인 **+54mm 전방**이다. 높이는 변화 없음(실측 0.085m).
        # ⚠️ 이 x 오프셋이 틀리면 폴리곤 자기필터가 엉뚱한 영역을 지운다
        #    (필터는 base_link 기준으로 동작한다 — config/scan_self_filter.yaml 참고).
        arguments=['0.054', '0', '0.085', '0', '0', '0', 'base_link', 'base_laser']
    )

    # slam_toolbox의 online_async_launch.py는 use_sim_time 기본값이 true라
    # 실물 로봇에서는 명시적으로 false로 덮어써야 함 (안 그러면 /clock을 기다리며 멈춤).
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    placeholder_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='placeholder_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=IfCondition(publish_placeholder_odom)
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_placeholder_odom_cmd)
    ld.add_action(declare_lidar_port_cmd)
    ld.add_action(ldlidar_node)
    ld.add_action(scan_self_filter)
    ld.add_action(scan_resample)
    ld.add_action(base_link_to_laser_tf)
    ld.add_action(placeholder_odom_tf)
    ld.add_action(slam_toolbox_launch)
    return ld
