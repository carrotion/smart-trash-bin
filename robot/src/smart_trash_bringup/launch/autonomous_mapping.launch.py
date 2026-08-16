import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
자율주행 맵핑 — mapping.launch.py(LD06+SLAM Toolbox)에 Nav2(맵핑 중 네비게이션
패턴, map_server/amcl 없이 SLAM Toolbox의 /map을 직접 씀)와 explore_lite(프론티어
자율탐색)를 얹은 launch (2026-08-16).

⚠️ 첫 실행은 반드시 사람이 옆에서 지켜보고 즉시 개입(모터 대시보드 정지 버튼/
전원 차단) 가능한 상태로 할 것 — 범퍼/낙하 센서가 아직 없어 라이다 단독 회피만
가능하고, 실제 로봇으로 이 조합을 검증한 적이 없음.

전제조건: smart-trash-diff-drive, smart-trash-ekf, smart-trash-imu 서비스가
이미 떠 있어야 함(odom, /imu/data, odom->base_link TF 공급).
"""


def generate_launch_description():
    bringup_share = get_package_share_directory('smart_trash_bringup')
    explore_share = get_package_share_directory('explore_lite')

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml'))
    declare_explore_params_cmd = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(bringup_share, 'config', 'explore_params.yaml'))

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'mapping.launch.py')),
        launch_arguments={'publish_placeholder_odom': 'false'}.items()
    )

    # controller_server/planner_server는 각자 내부에 local_costmap/global_costmap을
    # nav2_params.yaml의 중첩 파라미터(local_costmap.local_costmap.*,
    # global_costmap.global_costmap.*)로 자동 생성한다 — 별도 costmap 노드 불필요.
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params_file])
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params_file])
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params_file])
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params_file])
    # velocity_smoother는 첫 버전에선 안 씀 — DWB 컨트롤러 자체 가감속 제한(acc_lim_x
    # 등)으로 충분하고, cmd_vel 리매핑 체인을 하나 줄여서 복잡도/실패 지점을 낮춤.
    # controller_server가 기본 토픽명 그대로 cmd_vel에 바로 발행 -> diff_drive_node.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[nav2_params_file])

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore_node', output='screen',
        parameters=[explore_params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    ld = LaunchDescription()
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_explore_params_cmd)
    ld.add_action(mapping_launch)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)
    ld.add_action(explore_node)
    return ld
