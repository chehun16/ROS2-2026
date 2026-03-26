"""
BEV Navigation 전체 시스템 Launch 파일

실행 순서:
    1. Gazebo 시뮬레이터 (TurtleBot3 + 실내 환경)
    2. rgbd_processor_node  - RGBD 전처리
    3. bev_occupancy_node   - BEV 맵 생성 (핵심)
    4. Nav2                 - 자율 이동
    5. goal_sender_node     - 목표 전송
    6. RViz2                - 시각화

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    use_sim_time  = LaunchConfiguration('use_sim_time')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    launch_nav2   = LaunchConfiguration('launch_nav2')
    launch_rviz   = LaunchConfiguration('launch_rviz')

    pkg_bev = get_package_share_directory('bev_navigation')
    params_file     = os.path.join(pkg_bev, 'config', 'params.yaml')
    nav2_params_file = os.path.join(pkg_bev, 'config', 'nav2_params.yaml')

    # ----------------------------------------------------------------
    # 1. Gazebo + TurtleBot3 (turtlebot3_gazebo 패키지 활용)
    # ----------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch', 'turtlebot3_house.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_gazebo),
    )

    # ----------------------------------------------------------------
    # 2. Nav2 (navigation2 패키지 활용)
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
    # 3. 프로젝트 노드들
    # ----------------------------------------------------------------
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
    # 4. RViz2
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
        gazebo_launch,
        nav2_launch,
        rgbd_processor_node,
        bev_occupancy_node,
        goal_sender_node,
        rviz_node,
    ])
