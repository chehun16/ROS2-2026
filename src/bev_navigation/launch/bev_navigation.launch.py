"""
BEV Navigation 전체 시스템 Launch 파일

실행 순서:
    1. Gazebo 시뮬레이터 (House 환경)
    2. robot_state_publisher  - 커스텀 URDF (Depth 카메라 포함)
    3. spawn_entity            - Gazebo에 로봇 스폰
    4. SLAM Toolbox            - 실시간 맵 생성 (/map 제공)
    5. Nav2                    - 자율 이동
    6. rgbd_processor_node     - RGBD 전처리
    7. bev_occupancy_node      - BEV 맵 생성 (핵심)
    8. goal_sender_node        - 목표 전송 (5초 딜레이)
    9. RViz2                   - 시각화

실행 방법:
    ros2 launch bev_navigation bev_navigation.launch.py
    ros2 launch bev_navigation bev_navigation.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----------------------------------------------------------------
    # Launch Arguments
    # ----------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Gazebo 시뮬레이션 시간 사용 여부',
    )
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo', default_value='true',
        description='Gazebo 시뮬레이터 자동 실행 여부',
    )
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2', default_value='true',
        description='Nav2 네비게이션 스택 실행 여부',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='RViz2 시각화 실행 여부',
    )
    launch_slam_arg = DeclareLaunchArgument(
        'launch_slam', default_value='true',
        description='SLAM Toolbox 실행 여부 (Nav2 글로벌 맵 제공)',
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    launch_nav2   = LaunchConfiguration('launch_nav2')
    launch_rviz   = LaunchConfiguration('launch_rviz')
    launch_slam   = LaunchConfiguration('launch_slam')

    pkg_bev          = get_package_share_directory('bev_navigation')
    params_file      = os.path.join(pkg_bev, 'config', 'params.yaml')
    nav2_params_file = os.path.join(pkg_bev, 'config', 'nav2_params.yaml')
    urdf_file        = os.path.join(pkg_bev, 'urdf', 'turtlebot3_waffle_depth.urdf.xacro')

    # xacro → URDF 변환 (robot_state_publisher에 전달)
    robot_description = Command(['xacro ', urdf_file])

    # ----------------------------------------------------------------
    # 1. Gazebo — House 환경 (로봇 없이 world만 실행)
    #
    # turtlebot3_house.launch.py 대신 gazebo_ros의 gzserver/gzclient를
    # 직접 사용해 House world를 로드한다.
    # 이후 spawn_entity로 커스텀 URDF 로봇을 별도 스폰한다.
    # ----------------------------------------------------------------
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    house_world    = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_house.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'world': house_world,
            'verbose': 'false',
        }.items(),
        condition=IfCondition(launch_gazebo),
    )

    # ----------------------------------------------------------------
    # 2. robot_state_publisher — 커스텀 URDF (Depth 카메라 포함)
    #
    # xacro로 처리한 URDF를 /robot_description에 발행하고
    # TF 트리(base_link, camera_depth_link, …)를 자동으로 브로드캐스트.
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ----------------------------------------------------------------
    # 3. spawn_entity — Gazebo에 커스텀 로봇 스폰
    #
    # robot_state_publisher가 /robot_description을 발행한 뒤
    # spawn_entity가 해당 토픽을 읽어 Gazebo에 모델을 배치한다.
    # 0.5초 딜레이: robot_state_publisher 초기화 대기
    # ----------------------------------------------------------------
    spawn_robot = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3_waffle_depth',
                    '-topic',  '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01',
                    '-Y', '0.0',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    # ----------------------------------------------------------------
    # 4. SLAM Toolbox - 실시간 맵 생성 (Nav2 global costmap에 /map 제공)
    # ----------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_slam),
    )

    # ----------------------------------------------------------------
    # 5. Nav2 (navigation2 패키지 활용)
    # ----------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,   # BEV 맵 연결된 Nav2 파라미터
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    # ----------------------------------------------------------------
    # 6. 프로젝트 노드들
    # ----------------------------------------------------------------

    # rgbd_processor: 커스텀 URDF의 Depth 카메라 토픽 → 우리 토픽명
    #
    # Gazebo Depth 플러그인 발행 토픽:
    #   /camera/color/image_raw   (remapping으로 설정됨)
    #   /camera/depth/image_raw   (remapping으로 설정됨)
    #   /camera/color/camera_info (remapping으로 설정됨)
    # → 플러그인의 <remapping> 태그가 이미 올바른 이름으로 발행하므로
    #   여기서 추가 remapping 불필요.
    rgbd_processor_node = Node(
        package='bev_navigation',
        executable='rgbd_processor',
        name='rgbd_processor_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    bev_occupancy_node = Node(
        package='bev_navigation',
        executable='bev_occupancy',
        name='bev_occupancy_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Nav2가 준비된 뒤 목표 전송 (5초 딜레이)
    goal_sender_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='bev_navigation',
                executable='goal_sender',
                name='goal_sender_node',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    # ----------------------------------------------------------------
    # 7. RViz2
    # ----------------------------------------------------------------
    rviz_config = os.path.join(pkg_bev, 'config', 'bev_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen',
    )

    # ----------------------------------------------------------------
    # Launch Description 조합
    # ----------------------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        launch_gazebo_arg,
        launch_nav2_arg,
        launch_rviz_arg,
        launch_slam_arg,
        gazebo_launch,           # 1. Gazebo (House world)
        robot_state_publisher,   # 2. 커스텀 URDF TF 브로드캐스트
        spawn_robot,             # 3. Gazebo에 Depth 카메라 로봇 스폰
        slam_launch,             # 4. SLAM Toolbox (/map)
        nav2_launch,             # 5. Nav2
        rgbd_processor_node,     # 6a. RGBD 전처리
        bev_occupancy_node,      # 6b. BEV 맵 생성
        goal_sender_node,        # 6c. 목표 전송 (5초 딜레이)
        rviz_node,               # 7. RViz2
    ])
