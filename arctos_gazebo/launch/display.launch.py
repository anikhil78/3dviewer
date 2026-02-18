"""
display.launch.py
=================
Minimal launch for verifying the Arctos robot model in RViz2 WITHOUT Gazebo.

Starts:
  1. robot_state_publisher  (arctos_sim.xacro → /robot_description + TF)
  2. joint_state_publisher_gui  (GUI sliders so joints can be driven manually)
  3. RViz2

Use this to confirm:
  a) The URDF / xacro parses correctly.
  b) All expected links/joints are in the TF tree.
  c) The camera_link and camera_optical_frame sit where expected on the arm.

Usage:
  ros2 launch arctos_gazebo display.launch.py
  ros2 launch arctos_gazebo display.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_arctos_gazebo = get_package_share_directory('arctos_gazebo')
    pkg_arctos_desc   = get_package_share_directory('arctos_description')

    xacro_file  = os.path.join(pkg_arctos_gazebo, 'urdf', 'arctos_sim.xacro')
    rviz_config = os.path.join(pkg_arctos_desc,   'rviz', 'arctos.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2',
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    # Publishes /robot_description and all TF frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # GUI sliders to move joints without a controller or Gazebo
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=(['-d', rviz_config] if os.path.exists(rviz_config) else []),
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
