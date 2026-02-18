"""
sim.launch.py
=============
Launches a full Gazebo Harmonic simulation of the Arctos robot arm:

  1. Processes arctos_sim.xacro  →  /robot_description
  2. Starts robot_state_publisher
  3. Starts Gazebo Harmonic with arctos_world.sdf
  4. Spawns the robot into Gazebo
  5. Spawns ros2_control controllers (joint_state_broadcaster, arm, hand)
  6. Starts ros_gz_bridge to expose depth-camera topics in ROS2
  7. Optionally starts RViz2

Usage:
  ros2 launch arctos_gazebo sim.launch.py
  ros2 launch arctos_gazebo sim.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Paths                                                                #
    # ------------------------------------------------------------------ #
    pkg_arctos_gazebo   = get_package_share_directory('arctos_gazebo')
    pkg_arctos_desc     = get_package_share_directory('arctos_description')

    world_file    = os.path.join(pkg_arctos_gazebo, 'worlds', 'arctos_world.sdf')
    xacro_file    = os.path.join(pkg_arctos_gazebo, 'urdf',   'arctos_sim.xacro')
    rviz_config   = os.path.join(pkg_arctos_desc,   'rviz',   'arctos.rviz')

    # ------------------------------------------------------------------ #
    # GZ_SIM_RESOURCE_PATH                                                 #
    # Gazebo Harmonic resolves package:// URIs (meshes, etc.) by scanning  #
    # GZ_SIM_RESOURCE_PATH for directories named after the package.        #
    # We build it from AMENT_PREFIX_PATH (the ROS install prefixes) by     #
    # appending /share to each prefix so Gazebo finds e.g.                 #
    #   arctos_description/meshes/base_link.stl                            #
    # Existing GZ_SIM_RESOURCE_PATH entries are preserved.                 #
    # ------------------------------------------------------------------ #
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    gz_share_paths = [
        os.path.join(p, 'share')
        for p in ament_prefix_path.split(':') if p
    ]
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = ':'.join(filter(None, gz_share_paths + [existing_gz_path]))

    # ------------------------------------------------------------------ #
    # GZ_SIM_SYSTEM_PLUGIN_PATH                                           #
    # ros-humble-gz-ros2-control installs its plugin (.so) into           #
    # /opt/ros/humble/lib.  Gazebo Harmonic must have that directory in   #
    # GZ_SIM_SYSTEM_PLUGIN_PATH or it cannot load gz_ros2_control-system, #
    # which causes the entire model spawn to abort (robot never appears). #
    # ------------------------------------------------------------------ #
    existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = ':'.join(filter(None, ['/opt/ros/humble/lib', existing_gz_plugin_path]))

    # ------------------------------------------------------------------ #
    # GZ_IP                                                                #
    # In WSL2 there are multiple network interfaces (eth0, lo, …).        #
    # gz-transport multicast discovery can fail to reach across them,     #
    # causing `ros_gz_sim create` to timeout waiting for world names even  #
    # when Gazebo is actually running.  Binding to loopback fixes this.   #
    # setdefault leaves the variable unchanged if the user pre-set it.    #
    # ------------------------------------------------------------------ #
    os.environ.setdefault('GZ_IP', '127.0.0.1')

    # ------------------------------------------------------------------ #
    # Arguments                                                            #
    # ------------------------------------------------------------------ #
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 alongside the simulation',
    )

    # ------------------------------------------------------------------ #
    # Robot description (xacro → URDF string)                             #
    # ------------------------------------------------------------------ #
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    # ------------------------------------------------------------------ #
    # robot_state_publisher                                                #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------ #
    # Gazebo Harmonic server                                               #
    # The -r flag starts the simulation running immediately.               #
    # ------------------------------------------------------------------ #
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        emulate_tty=True,          # surface Gazebo stdout/stderr to the console
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            # ogre2 software-rendering helpers (WSL2 / llvmpipe)
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'OGRE_RTT_MODE': 'Copy',
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            # Ensure Gazebo can find libgz_ros2_control-system.so
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        },
    )

    # ------------------------------------------------------------------ #
    # Spawn the robot URDF into Gazebo                                     #
    # Reads /robot_description published by robot_state_publisher.         #
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_arctos',
        arguments=[
            '-name',  'arctos',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Controllers                                                          #
    # The gz_ros2_control plugin starts the controller_manager inside Gz.  #
    # We wait for the spawn to finish before activating controllers.       #
    # ------------------------------------------------------------------ #
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arctos_arm_controller_spawner',
        arguments=['arctos_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arctos_hand_controller_spawner',
        arguments=['arctos_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain: spawn robot → joint_state_broadcaster → arm + hand controllers
    spawn_jsb_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, hand_controller_spawner],
        )
    )

    # ------------------------------------------------------------------ #
    # ros_gz_bridge — bridge Gazebo camera topics into ROS2                #
    #                                                                      #
    # Topic format: /gz_topic@ros_type[gz_type   (Gazebo → ROS2)          #
    #                                                                      #
    # The rgbd_camera sensor with <topic>camera</topic> publishes:         #
    #   /camera/image          RGB image                                   #
    #   /camera/depth_image    float32 depth image                         #
    #   /camera/camera_info    intrinsics + frame                          #
    #   /camera/points         PointCloud2                                 #
    # ------------------------------------------------------------------ #
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Simulation clock — needed for use_sim_time to work
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # RGB image
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Depth image
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera intrinsics (shared for both rgb and depth)
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Dense point cloud (XYZRGB)
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------ #
    # RViz2 (optional)                                                     #
    # Reuses the existing arctos.rviz config from arctos_description.      #
    # Software rendering is forced via LIBGL_ALWAYS_SOFTWARE=1 so it       #
    # works in WSL2 without a dedicated GPU.                               #
    # ------------------------------------------------------------------ #
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Use the existing rviz config if present; fall back to no config
        arguments=(['-d', rviz_config] if os.path.exists(rviz_config) else []),
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        gz_sim,
        # Wait for Gazebo to finish loading the world before spawning.
        # WSL2 + software rendering can take 15–30 s; 20 s is a safe floor.
        TimerAction(period=20.0, actions=[spawn_robot]),
        spawn_jsb_after_robot,
        spawn_arm_after_jsb,
        ros_gz_bridge,
        rviz2,
    ])
