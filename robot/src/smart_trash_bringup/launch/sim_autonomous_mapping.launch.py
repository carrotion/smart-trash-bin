import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, RegisterEventHandler,
                             SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PythonExpression,
                                   TextSubstitution)
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
    declare_slam_launch_cmd = DeclareLaunchArgument(      # 실물 mapping.launch.py와 같은 인자(저장 지도에서 위치 잡기 시험용)
        'slam_launch', default_value='online_async_launch.py')
    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_share, 'config', 'mapper_params_online_async.yaml'))
    declare_enable_explore_cmd = DeclareLaunchArgument(
        'enable_explore', default_value='true')
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='cave'),
        description='stage_ros2/world/<world>.world (확장자 없이)')
    # 2026-08-23: explore_lite vs frontier_exploration_ros2 전략 비교용
    # (docs/exploration_strategy_research.md 후속). 같은 world/시작지점/Nav2/SLAM
    # 스택 위에서 프론티어 탐색기만 바꿔 낀다.
    declare_explorer_cmd = DeclareLaunchArgument(
        'explorer', default_value='explore_lite',
        description="'explore_lite' 또는 'frontier' (frontier_exploration_ros2)")
    # 2026-08-23: home_real/our_home 양쪽에서 공통으로 나온 "Transform data too
    # old"(map->odom TF 지연) 진단용. 빈 문자열이면 프로덕션 값(0.3) 그대로 —
    # nav2_params.yaml은 절대 안 건드리고 런치 오버레이로만 적용해서, 진단이
    # 끝나면 이 인자를 안 주는 것만으로 원래 상태로 완전히 돌아온다.
    declare_diag_tf_tolerance_cmd = DeclareLaunchArgument(
        'diag_transform_tolerance', default_value='',
        description='비우면 프로덕션 값(0.3) 유지. 숫자를 주면 controller_server/'
                    'behavior_server/bt_navigator/global·local_costmap의 '
                    'transform_tolerance를 전부 이 값으로 일괄 오버라이드(진단용).')
    # 2026-08-23: frontier_exploration_ros2 파라미터 탐색(Optuna)용 — 아래
    # frontier_node의 하드코딩 파라미터는 로봇 기하 기반 상수(robot_base_frame,
    # 거리 2개)만 유지하고, 나머지 탐색 대상 파라미터는 이 파일로 오버라이드한다.
    # 빈 문자열이면 패키지 기본값 그대로.
    declare_frontier_params_cmd = DeclareLaunchArgument(
        'frontier_params_file', default_value='',
        description='frontier_exploration_ros2 파라미터 오버라이드 yaml(비우면 '
                    '패키지 기본값 + 로봇 기하 상수만 적용).')

    def stage_world_configuration(context):
        f = os.path.join(stage_ros2_share, 'world',
                          context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', f)]

    stage_world_configuration_cmd = OpaqueFunction(function=stage_world_configuration)

    def diag_tf_tolerance_configuration(context):
        val = context.launch_configurations.get('diag_transform_tolerance', '')
        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', prefix='diag_tf_tolerance_',
                delete=False) as f:
            if val:
                for node_block in (
                    f'controller_server:\n  ros__parameters:\n'
                    f'    transform_tolerance: {val}\n'
                    f'    FollowPath:\n      transform_tolerance: {val}\n',
                    f'local_costmap:\n  local_costmap:\n    ros__parameters:\n'
                    f'      transform_tolerance: {val}\n',
                    f'planner_server:\n  ros__parameters:\n'
                    f'    transform_tolerance: {val}\n',
                    f'global_costmap:\n  global_costmap:\n    ros__parameters:\n'
                    f'      transform_tolerance: {val}\n'
                    f'      static_layer:\n        transform_tolerance: {val}\n',
                    f'behavior_server:\n  ros__parameters:\n'
                    f'    transform_tolerance: {val}\n',
                    f'bt_navigator:\n  ros__parameters:\n'
                    f'    transform_tolerance: {val}\n',
                ):
                    f.write(node_block)
            else:
                # 오버라이드 없음 — 무해한 no-op(이미 True인 값 재확인)만 적힌
                # 유효한 yaml. 빈 파일/빈 매핑을 --params-file로 주면 일부
                # rclcpp 버전에서 파싱 경고가 나서 이렇게 채워둔다.
                f.write('/**:\n  ros__parameters:\n    use_sim_time: true\n')
            diag_file = f.name
        return [SetLaunchConfiguration('diag_tf_override_file', diag_file)]

    diag_tf_tolerance_configuration_cmd = OpaqueFunction(
        function=diag_tf_tolerance_configuration)
    diag_tf_override_file = LaunchConfiguration('diag_tf_override_file')

    def frontier_params_configuration(context):
        val = context.launch_configurations.get('frontier_params_file', '')
        if val:
            return [SetLaunchConfiguration('frontier_params_override_file', val)]
        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', prefix='frontier_params_noop_',
                delete=False) as f:
            f.write('frontier_explorer:\n  ros__parameters:\n    use_sim_time: true\n')
            noop_file = f.name
        return [SetLaunchConfiguration('frontier_params_override_file', noop_file)]

    frontier_params_configuration_cmd = OpaqueFunction(
        function=frontier_params_configuration)
    frontier_params_override_file = LaunchConfiguration('frontier_params_override_file')

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
            [slam_toolbox_share, '/launch/', LaunchConfiguration('slam_launch')]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
        }.items()
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file, diag_tf_override_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')])   # 실물과 같이 collision_monitor 경유(09-27)
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file, diag_tf_override_file])
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file, diag_tf_override_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')])   # 실물과 같이 collision_monitor 경유(09-27)
    # ⚠️ nav2_params.yaml의 default_nav_to_pose_bt_xml은 실물 Pi 홈 경로
    # (/home/carrotian/...)로 하드코딩돼 있어 이 시뮬 환경(사용자명이 다름)에서는
    # 존재하지 않는 파일이 된다 — bt_navigator가 활성화 자체에 실패한다
    # (2026-08-23 첫 시도에서 확인). 패키지에 설치된 share 경로로 오버라이드.
    #
    # ⚠️ 처음엔 이걸 plain dict로 parameters 리스트 맨 뒤에 넣었는데 안 먹혔다.
    # launch_ros가 plain dict를 임시 yaml로 만들 때 노드 이름이 아니라 "/**"
    # 와일드카드로 스코프를 잡는데, ROS2 파라미터 우선순위는 **와일드카드보다
    # 노드명 명시 항목이 파일 순서와 무관하게 항상 이긴다.** nav2_params.yaml은
    # "bt_navigator:" 로 명시 스코프라 와일드카드 오버라이드가 항상 졌다
    # (2026-08-23, ps로 --params-file 순서까지 확인했는데도 값이 안 바뀌어서
    # 발견 — 순서 문제가 아니라 스코프 문제였음). 그래서 직접 노드명으로 스코프한
    # 임시 yaml을 만들어 맨 뒤에 넣는다(같은 스코프끼리는 파일 순서상 마지막이 이김).
    bt_xml_override_path = os.path.join(
        bringup_share, 'behavior_trees', 'navigate_to_pose_slow_recovery.xml')
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', prefix='bt_navigator_override_', delete=False) as f:
        f.write('bt_navigator:\n  ros__parameters:\n'
                f'    default_nav_to_pose_bt_xml: "{bt_xml_override_path}"\n')
        bt_xml_override_file = f.name

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[
            {'use_sim_time': True},
            nav2_params_file,
            bt_xml_override_file,
            diag_tf_override_file,
        ])
    # 2026-09-27: nav2_params.yaml의 lifecycle node_names에 collision_monitor가 들어가 시뮬에도 필요
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file])

    explore_lite_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
        LaunchConfiguration('explorer'), "' == 'explore_lite'"]))
    frontier_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('enable_explore'), "' == 'true' and '",
        LaunchConfiguration('explorer'), "' == 'frontier'"]))

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore_node', output='screen',
        parameters=[{'use_sim_time': True}, explore_params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=explore_lite_condition)

    # frontier_exploration_ros2 (mertgulerx) — explore_lite 대체 후보 비교용.
    # 패키지 기본값(config/params.yaml)은 TurtleBot3 Waffle 기준이라 그대로 못
    # 쓰는 파라미터만 오버라이드한다: robot_base_frame(우리는 base_footprint가
    # 없고 base_link뿐 — Stage가 odom->base_link를 직접 발행), 그리고 로봇
    # 반경 기반 권장값(2.5x 외접반경 0.141m ≈ 0.35m)으로 두 거리 파라미터.
    # 나머지(MRTSP, map 최적화, QoS 등)는 패키지 자체 기본값을 그대로 써서
    # "우리가 튜닝한 explore_lite" vs "저쪽 기본 전략" 비교가 되게 한다.
    frontier_node = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_base_frame': 'base_link',
            'frontier_selection_min_distance': 0.35,
            'frontier_candidate_min_goal_distance_m': 0.35,
        }, frontier_params_override_file],
        condition=frontier_condition)

    # ⚠️ 2026-08-23 밤 (Phase 1.5 완화책 a, Remi 결정): explore_node/frontier_node를
    # 그냥 나열하면 lifecycle_manager/SLAM과 동시에 시작돼서, 부트스트랩 극초반
    # (SLAM/코스트맵 초기화 부하가 몰리는 시점, 실측 t≈9s)에 TF가 살짝 지연된
    # 채로 첫 목표를 계산해버린다. explore_lite는 이 첫 실패 단 1회만으로
    # "모든 프론티어 소진"으로 오판하고 완전히 포기한다(explore_lite_r0.log로
    # 확인, home_real 3번실패/파이 0회전환과 같은 계열).
    #
    # 고정 시간 지연(TimerAction) 대신 lifecycle_manager가 실제로 찍는
    # "Managed nodes are active" 로그를 감지해서 그 시점에 시작한다 — 부트스트랩
    # 소요시간이 부하에 따라 달라져도(N=4 병렬 등) 견고하다. 문자열이 두 번
    # 이상 매치돼 두 번 시작되는 걸 막기 위해 플래그로 1회만 트리거.
    _explore_start_state = {'triggered': False}

    def start_explorer_when_nav2_active(event):
        if _explore_start_state['triggered']:
            return None
        text = event.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode('utf-8', errors='ignore')
        if 'Managed nodes are active' in text:
            _explore_start_state['triggered'] = True
            return [explore_node, frontier_node]
        return None

    explorer_start_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=lifecycle_manager,
            on_stdout=start_explorer_when_nav2_active,
            on_stderr=start_explorer_when_nav2_active,
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_explore_params_cmd)
    ld.add_action(declare_slam_params_cmd)
    ld.add_action(declare_slam_launch_cmd)
    ld.add_action(declare_enable_explore_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_explorer_cmd)
    ld.add_action(declare_diag_tf_tolerance_cmd)
    ld.add_action(declare_frontier_params_cmd)
    ld.add_action(stage_world_configuration_cmd)
    ld.add_action(diag_tf_tolerance_configuration_cmd)
    ld.add_action(frontier_params_configuration_cmd)
    ld.add_action(stage_node)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(collision_monitor)
    ld.add_action(lifecycle_manager)
    ld.add_action(explorer_start_handler)
    return ld
