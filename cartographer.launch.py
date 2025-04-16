#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 현재 패키지(cartographer_custom)의 공유 디렉토리 경로 찾기
    pkg_cartographer_custom = get_package_share_directory('cartographer_custom')

    # --- 기본 파일 경로 정의 ---
    # RViz 설정 파일 기본 경로
    default_rviz_config_path = os.path.join(pkg_cartographer_custom, 'rviz/cartographer.rviz')
    # Cartographer Lua 설정 파일이 있는 디렉토리 기본 경로
    default_config_dir = os.path.join(pkg_cartographer_custom, 'config')
    # Cartographer Lua 설정 파일 이름 기본값
    default_config_basename = 'config.lua'

    # --- Launch Arguments 선언 ---
    # 시뮬레이션 시간 사용 여부
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Cartographer 설정 디렉토리 경로
    config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=default_config_dir,
        description='Full path to config file to load'
    )

    # Cartographer 설정 파일 이름 (basename)
    config_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=default_config_basename,
        description='Name of lua file for cartographer'
    )

    # RViz 실행 여부 (명확성을 위해 이름 변경: cartographer_rviz -> use_rviz)
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true', # 기본값으로 RViz 실행 (필요에 따라 false로 변경 가능)
        description='Whether to start RVIZ'
    )

    # RViz 설정 파일 경로
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    # Occupancy Grid 해상도
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'
    )

    # Occupancy Grid 발행 주기
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='0.5', # OCR 기본값은 0.5초, 필요시 1.0 등으로 조절
        description='OccupancyGrid publishing period'
    )

    # --- Launch Configurations 가져오기 ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # --- Nodes 정의 ---

    # Cartographer SLAM 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        # remappings=[ # 필요시 토픽 이름 변경 (예: 오도메트리 토픽)
        #     ('/odom', '/odom_rf2o')
        # ]
    )

    # Cartographer Occupancy Grid 노드 (맵 생성)
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # RViz 노드 (use_rviz가 true일 때만 실행)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}] # RViz도 sim_time 사용 여부 인지해야 함
    )

    # 컨트롤러 노드 (매뉴얼에서 이 launch 파일에 포함시킴)
    # 매핑 중 로봇을 움직여야 하므로 함께 실행하는 것이 일반적
    controller_node = Node(
        package='teleop', # teleop 패키지 이름이 맞는지 확인 필요
        executable='controller', # controller.py 스크립트의 실행 파일 이름
        name='bot_controller',
        output='screen'
        # parameters=[...] # 필요한 파라미터가 있다면 추가
    )

    # --- LaunchDescription 생성 ---
    ld = LaunchDescription()

    # 환경 변수 설정 (선택 사항, Cartographer가 특정 환경 변수 필요시)
    # ld.add_action(SetEnvironmentVariable('ENV_VAR_NAME', 'value'))

    # Launch Arguments 추가
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_dir_arg)
    ld.add_action(config_basename_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(resolution_arg)
    ld.add_action(publish_period_sec_arg)

    # Nodes 추가
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)
    ld.add_action(controller_node) # teleop 컨트롤러 노드 추가

    return ld