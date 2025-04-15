#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory where the description package files are located
    pkg_description = get_package_share_directory('description')

    # Define default paths for URDF and RViz config files within the package
    default_rviz_config_path = os.path.join(pkg_description, 'rviz/model.rviz')
    default_urdf_model_path = os.path.join(pkg_description, 'urdf/model.urdf')

    # --- Declare Launch Arguments ---

    # Argument for the URDF model file path
    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )

    # Argument for the RViz configuration file path
    rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    # Argument to control whether RViz is launched
    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false', # Default is not to launch RViz
        description='Whether to start RVIZ'
    )

    # Argument to control whether simulation time is used
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false', # Default is to use real time
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Get Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # --- Define Nodes ---

    # Node to publish joint states (typically from sliders or other inputs)
    # This is needed for the robot_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Node to publish the robot's state (TF transforms) based on URDF and joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Load the URDF model from the specified path using the xacro command
            'robot_description': Command(['xacro ', urdf_model])
        }],
        # Pass the URDF file path as an argument (though parameters are more common now)
        # arguments=[default_urdf_model_path] # This line might be redundant if using parameters correctly
    )

    # Node to launch RViz for visualization, only if use_rviz argument is true
    rviz_node = Node(
        condition=IfCondition(use_rviz), # Only run if use_rviz is 'true'
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] # Load the specified RViz config
    )

    # Node to publish static transforms (e.g., map to odom, odom to base_footprint)
    # This is often needed for localization and navigation setup
    # It seems two static transforms are published here based on the OCR
    static_tf_map_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map2o', # Changed name to be more descriptive
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'] # x y z yaw pitch roll frame_id child_frame_id
        # condition=IfCondition(use_rviz) # This transform might be needed even without RViz
    )

    static_tf_odom_base_node = Node(
        condition=IfCondition(use_rviz), # Conditionally launched based on OCR, but often needed regardless
        package='tf2_ros',
        executable='static_transform_publisher',
        name='o2base', # Changed name to be more descriptive (original was map2o again?)
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )


    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add arguments to the launch description
    ld.add_action(urdf_model_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_sim_time_arg)

    # Add nodes to the launch description
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node) # RViz node (conditional)
    ld.add_action(static_tf_map_odom_node) # Static transform map -> odom
    ld.add_action(static_tf_odom_base_node) # Static transform odom -> base_footprint (conditional based on OCR)


    return ld