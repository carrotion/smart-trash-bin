import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression
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

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml'))
    declare_explore_params_cmd = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(bringup_share, 'config', 'explore_params.yaml'))
    # enable_explore:=false로 두면 Nav2 스택만 뜨고 아무도 목표를 안 보낸다 =
    # 로봇이 절대 안 움직임. 설정/코스트맵 검증을 안전하게 하기 위한 스위치.
    declare_enable_explore_cmd = DeclareLaunchArgument(
        'enable_explore', default_value='true')
    # 2026-09-27: 기본 탐색기를 frontier_exploration_ros2로. 같은 시뮬(Stage cave, 새 Nav2 설정) 4분 비교:
    #   explore_lite — 35 m², 목표 성공 2/5, "모든 경계 시도함"으로 2분 만에 스스로 중단
    #   frontier     — 75.7 m²(계속 탐색 중), 목표 성공 15/16. 08월 시뮬 비교 결론(docs/pc_tasks.md)과 같음
    # 2026-10-10: 움직임 방식. 'step' = 4층 멈춤-확인-이동 주행기(합격 시험 통과: loop 09-27, 장애물 코스 10-10).
    #   'nav2' = Nav2 연속 주행(DWB) — 09-27 실물에서 발진·방향 오차로 기반 재설계의 계기가 된 방식, 비교용으로 남김
    declare_navigator_cmd = DeclareLaunchArgument(
        'navigator', default_value='step',
        description="'step'(멈춤-확인-이동 주행기, 기본) 또는 'nav2'(연속 주행)")
    declare_explorer_cmd = DeclareLaunchArgument(
        'explorer', default_value='frontier',
        description="'frontier'(frontier_exploration_ros2, 기본) 또는 'explore_lite'")

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
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')])   # → collision_monitor → /cmd_vel (09-27, todo #12)
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params_file])
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')])   # → collision_monitor → /cmd_vel (09-27, todo #12)
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params_file])
    # velocity_smoother는 첫 버전에선 안 씀 — DWB 컨트롤러 자체 가감속 제한(acc_lim_x
    # 등)으로 충분하고, cmd_vel 리매핑 체인을 하나 줄여서 복잡도/실패 지점을 낮춤.
    # 2026-09-27: controller/behavior → cmd_vel_nav → collision_monitor → cmd_vel → diff_drive_node.
    # 충돌 감시: cmd_vel_nav를 받아 장애물이 정지 구역에 있거나 스캔이 0.5초 끊기면 0을 낸다
    # (nav2_params.yaml collision_monitor 절에 구역 근거). lifecycle_manager가 관리.
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[nav2_params_file])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[nav2_params_file])

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore_node', output='screen',
        parameters=[explore_params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
            LaunchConfiguration('explorer'), "' == 'explore_lite'"])))
    # 시뮬 런치(sim_autonomous_mapping.launch.py)와 같은 파라미터 — 시뮬에서 검증한 값을 그대로
    # 탐색기는 멈춘 채로 뜬다(10-10 결정): 지도 대시보드에서 금지 구역을 먼저 그리고 [탐색 시작]을 눌러야 움직인다.
    #   런치만으로 로봇이 스스로 출발하지 않게 하는 뜻도 있다. 바로 출발시키려면 explore_autostart:=true
    frontier_common = {
        'autostart': ParameterValue(LaunchConfiguration('explore_autostart'), value_type=bool),
        'robot_base_frame': 'base_link',
        'frontier_selection_min_distance': 0.35,
        'frontier_candidate_min_goal_distance_m': 0.35,
    }
    frontier_node = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer', output='screen',
        parameters=[frontier_common],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
            LaunchConfiguration('explorer'), "' == 'frontier' and '",
            LaunchConfiguration('navigator'), "' == 'nav2'"])))
    # navigator:=step(기본) — 탐색기의 목표를 4층 주행기 액션으로 보낸다. 값은 tools/sim_frontier_step.yaml(10-09/10-10 시뮬)과 같다:
    #   실패 3회면 그 구역을 잠시 제외(같은 목표를 4초마다 다시 보내 31번 막힌 10-09 시뮬 1차), 진행 없음 감시 30초
    #   (주행기 서버가 남은 거리를 보냄; 직진 없는 가장 긴 구간 실물 12.9초·시뮬 11.3초)
    frontier_step_node = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer', output='screen',
        parameters=[dict(frontier_common,
                         navigate_to_pose_action_name='step_navigate_to_pose',
                         # 탐색 후보는 "닿을 수 있는 곳의 경계"만(아래 explore_map_filter가 내는 지도). 10-11 첫 실물 탐색에서
                         #   구역·틈 너머로 들여다본 자리를 3~6번씩 "빈 경로"로 되풀이하고 끝난 줄도 몰랐다
                         map_topic='/map_explore',
                         frontier_suppression_enabled=True,
                         frontier_suppression_attempt_threshold=3,
                         frontier_suppression_no_progress_timeout_s=30.0)],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
            LaunchConfiguration('explorer'), "' == 'frontier' and '",
            LaunchConfiguration('navigator'), "' == 'step'"])))
    # 4층 주행기 액션 서버(파이썬 도구) — enable_explore:=false여도 뜬다(손으로 목표를 줄 수 있게). 움직임은 목표가 와야 시작
    repo_tools = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'tools'))
    step_server = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(repo_tools, 'step_nav_action_server.py')],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('navigator'), "' == 'step'"])))

    # 금지 구역 서버(파이썬 도구) — 항상 뜬다. 켜질 때 지난 구역을 치워 둔다(SLAM이 새 원점으로 새 지도를 그리므로)
    keepout_server = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(repo_tools, 'keepout_server.py')], output='screen')

    # 탐색기용 지도 필터(파이썬 도구) — 탐색 구성(navigator:=step)에서만. 원래 /map은 그대로 두고 /map_explore를 따로 낸다
    explore_map_filter = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(repo_tools, 'explore_map_filter.py')], output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
            LaunchConfiguration('explorer'), "' == 'frontier' and '",
            LaunchConfiguration('navigator'), "' == 'step'"])))

    declare_explore_autostart_cmd = DeclareLaunchArgument(
        'explore_autostart', default_value='false',
        description='true면 탐색기가 뜨자마자 탐색 시작. 기본 false — 지도 대시보드의 [탐색 시작]으로 시작(frontier 탐색기만 해당)')

    ld = LaunchDescription()
    ld.add_action(declare_explore_autostart_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_explore_params_cmd)
    ld.add_action(declare_enable_explore_cmd)
    ld.add_action(declare_explorer_cmd)
    ld.add_action(declare_navigator_cmd)
    ld.add_action(mapping_launch)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(collision_monitor)
    ld.add_action(lifecycle_manager)
    ld.add_action(explore_node)
    ld.add_action(frontier_node)
    ld.add_action(frontier_step_node)
    ld.add_action(step_server)
    ld.add_action(keepout_server)
    ld.add_action(explore_map_filter)
    return ld
