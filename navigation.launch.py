#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# OCR 페이지 69의 내용 시작
def generate_launch_description():
    # OCR 페이지 69: 패키지 경로 설정 (변수 이름 prefix, bringup_prefix 사용)
    prefix = get_package_share_directory('navigation')
    bringup_prefix = get_package_share_directory('nav2_bringup')

    # OCR 페이지 70: LaunchConfiguration 정의 (파일 경로 포함)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            prefix,
            'maps',
            'my_map.yaml')) # OCR에서 줄바꿈 적용
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            prefix,
            'params',
            'param.yaml')) # OCR에서 줄바꿈 적용
    nav2_launch_file_dir = os.path.join(bringup_prefix, 'launch')
    rviz_config_dir = os.path.join(
        prefix,
        'rviz',
        'nav2.rviz') # OCR에서 줄바꿈 적용

    # OCR 페이지 70: LaunchDescription 반환 시작
    return LaunchDescription([
        # OCR 페이지 70: DeclareLaunchArgument 정의 (OCR 구조 최대한 유지)
        DeclareLaunchArgument(
            'map',
            default_value=map_dir, # OCR처럼 LaunchConfiguration 객체를 직접 할당 시도 (오류 가능성 높음)
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir, # OCR처럼 LaunchConfiguration 객체를 직접 할당 시도 (오류 가능성 높음)
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # OCR 페이지 71: IncludeLaunchDescription (nav2_bringup 실행)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']), # OCR 경로 구조 사용
            # OCR에 launch_arguments 전달 방식 명시 안됨, 표준 방식 사용 유지
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # OCR 페이지 71: Node (rviz2 실행)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir], # OCR 변수 이름 사용
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # OCR 페이지 71: Node (controller 실행)
        Node(
            package='teleop',
            executable='controller',
            name='bot_controller',
            output='screen'
        ),
    ])