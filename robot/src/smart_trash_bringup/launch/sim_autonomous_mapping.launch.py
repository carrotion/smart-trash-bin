import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

"""
autonomous_mapping.launch.py의 시뮬레이션판 (2026-08-22).

실물 대신 stage_ros2(Stage 시뮬레이터)를 센서/오도메트리 소스로 쓴다. SLAM/Nav2/
explore_lite 설정 파일(mapper_params_online_async.yaml, nav2_params.yaml,
explore_params.yaml)은 **실물과 완전히 동일한 파일을 그대로 재사용**한다 —
시뮬에서 검증한 파라미터가 실물에도 그대로 옮겨가야 의미가 있기 때문.

⚠️ 반드시 실물과 다른 ROS_DOMAIN_ID에서 띄울 것. 같은 도메인에 실물 로봇의
diff_drive_node/EKF/모터 대시보드가 떠있는 상태로 이 launch를 켜면 시뮬 /cmd_vel과
실물 /cmd_vel이 겹쳐 실물이 같이 움직일 수 있다(2026-08-22 실제로 겪었다 —
다행히 그때는 명령이 안 나가서 무사했음). 예:
    ROS_DOMAIN_ID=42 xvfb-run -a ros2 launch smart_trash_bringup sim_autonomous_mapping.launch.py

Stage는 기본적으로 실물과 같은 프레임/토픽 이름을 쓴다(odom 프레임 "odom",
base_link 프레임 "base_link", /odom 토픽, odom->base_link TF까지 자체 발행) —
그래서 EKF·diff_drive_node 없이도 Nav2가 그대로 붙는다. 유일하게 다른 건 레이저
토픽 이름(Stage는 base_scan 고정, ranger.cpp에 하드코딩) — 여기서 scan으로 리매핑.
"""


def generate_launch_description():
    bringup_share = get_package_share_directory('smart_trash_bringup')
    stage_ros2_share = get_package_share_directory('stage_ros2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world = LaunchConfiguration('world')

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml'))
    declare_explore_params_cmd = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(bringup_share, 'config', 'explore_params.yaml'))
    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_share, 'config', 'mapper_params_online_async.yaml'))
    declare_enable_explore_cmd = DeclareLaunchArgument(
        'enable_explore', default_value='true')
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='cave'),
        description='stage_ros2/world/<world>.world (확장자 없이)')

    def stage_world_configuration(context):
        f = os.path.join(stage_ros2_share, 'world',
                          context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', f)]

    stage_world_configuration_cmd = OpaqueFunction(function=stage_world_configuration)

    # stage_ros2/launch/stage.launch.py를 그대로 include하지 않고 직접 Node로
    # 띄운다 — base_scan -> scan 리매핑을 넣기 위함(include로는 이 remap을
    # 끼워넣을 방법이 없다). 나머지 기본 파라미터(frame_id_odom=odom,
    # frame_id_base_link=base_link)는 실물과 이미 일치해서 안 건드림.
    stage_node = Node(
        package='stage_ros2', executable='stage_ros2', name='stage', output='screen',
        parameters=[{'world_file': [LaunchConfiguration('world_file')]}],
        remappings=[('base_scan', 'scan')],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
        }.items()
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore_node', output='screen',
        parameters=[{'use_sim_time': True}, explore_params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(LaunchConfiguration('enable_explore')))

    ld = LaunchDescription()
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_explore_params_cmd)
    ld.add_action(declare_slam_params_cmd)
    ld.add_action(declare_enable_explore_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(stage_world_configuration_cmd)
    ld.add_action(stage_node)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)
    ld.add_action(explore_node)
    return ld
