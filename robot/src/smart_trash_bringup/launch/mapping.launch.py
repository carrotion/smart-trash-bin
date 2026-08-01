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

전제: ld06.launch.py가 base_link->base_laser 정적 tf와 /scan을 발행함
(robot/src/ldlidar_stl_ros2/launch/ld06.launch.py 참고).

odom->base_link는 아직 Week 3 diff-drive 오도메트리 노드가 없으므로
`publish_placeholder_odom`(기본 true)으로 identity tf를 임시 발행함 —
실제 휠 오도메트리 노드가 생기면 이 launch 인자를 false로 끌 것.
"""


def generate_launch_description():
    bringup_share = get_package_share_directory('smart_trash_bringup')
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params_file = LaunchConfiguration('slam_params_file')
    publish_placeholder_odom = LaunchConfiguration('publish_placeholder_odom')

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

    ld06_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_share, 'launch', 'ld06.launch.py'))
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
    ld.add_action(ld06_launch)
    ld.add_action(placeholder_odom_tf)
    ld.add_action(slam_toolbox_launch)
    return ld
