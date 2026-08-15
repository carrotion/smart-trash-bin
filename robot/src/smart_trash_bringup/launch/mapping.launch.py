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

LD06은 USB 어댑터가 아니라 Pi5 GPIO UART(/dev/ttyAMA0, 230400bps)로 직결돼있음
(2026-08-15 실측 확인 — CLAUDE.md 참고). 서브모듈 ldlidar_stl_ros2가 제공하는
launch/ld06.launch.py는 port_name이 /dev/ttyUSB0로 하드코딩돼 있어서 우리 배선과
안 맞아, 여기서 ldlidar_stl_ros2_node를 직접 구성함(그 launch 파일을 include하지 않음).

odom->base_link는 아직 Week 3 diff-drive 오도메트리 노드가 없으므로
`publish_placeholder_odom`(기본 true)으로 identity tf를 임시 발행함 —
실제 휠 오도메트리 노드가 생기면 이 launch 인자를 false로 끌 것.

⚠️ TODO(실측 필요): LD06을 상판에 거꾸로(뒤집어서) 마운트함 — 아직 정확한 뒤집힘 축
(롤/피치)을 확인 안 해서 아래 static_transform_publisher의 회전값(현재 0,0,0)과
laser_scan_dir을 추측으로 바꾸지 않았음. 실측 후 두 군데 다 반영할 것:
  1. base_link->base_laser 정적 tf의 롤/피치 (지금 마지막 세 인자가 전부 0)
  2. 아래 laser_scan_dir 파라미터 (뒤집히면 실제 회전방향 해석이 반대가 될 수 있음)
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
        default_value='true',
        description=(
            'true면 odom->base_link를 identity tf로 임시 발행. '
            'Week 3 diff-drive 오도메트리 노드가 준비되면 false로 끌 것.'))

    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyAMA0',
        description='LD06 시리얼 포트 (GPIO UART, 2026-08-15 실측 확인)')

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan',
            'frame_id': 'base_laser',
            'port_name': lidar_port,
            'port_baudrate': 230400,
            # TODO(실측 필요, 위 모듈 docstring 참고): 상판에 거꾸로 마운트돼서
            # 실제 회전 방향이 반대일 수 있음 — 지금은 정방향 마운트 가정값(true) 그대로 둠.
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }]
    )

    # TODO(실측 필요, 위 모듈 docstring 참고): 뒤집힘 축 확인되면 마지막 세 인자
    # (roll pitch yaw, 현재 0 0 0)에 반영할 것. 높이(0.18)는 ld06.launch.py 기본값을
    # 그대로 가져온 placeholder라 이것도 실측으로 교체 필요.
    base_link_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld06',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
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
    ld.add_action(base_link_to_laser_tf)
    ld.add_action(placeholder_odom_tf)
    ld.add_action(slam_toolbox_launch)
    return ld
