"""
sim.launch.py
=============
Launches a full Gazebo Harmonic simulation of the Arctos robot arm:

  1. Processes arctos_sim.xacro → URDF string (at Python level, before launch)
  2. Writes that URDF to /tmp so Gazebo can load it via <include> in the world SDF
  3. Generates a modified world SDF that embeds the robot — no runtime gz-transport
     spawn call needed (works around WSL2 loopback-multicast breakage)
  4. Starts Gazebo Harmonic with the generated world
  5. Starts robot_state_publisher
  6. Spawns ros2_control controllers after a short delay
  7. Starts ros_gz_bridge to expose depth-camera topics in ROS2
  8. Optionally starts RViz2

Usage:
  ros2 launch arctos_gazebo sim.launch.py
  ros2 launch arctos_gazebo sim.launch.py rviz:=false
"""

import os
import subprocess
import tempfile

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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Paths                                                                #
    # ------------------------------------------------------------------ #
    pkg_arctos_gazebo = get_package_share_directory('arctos_gazebo')
    pkg_arctos_desc   = get_package_share_directory('arctos_description')

    xacro_file  = os.path.join(pkg_arctos_gazebo, 'urdf',   'arctos_sim.xacro')
    world_file  = os.path.join(pkg_arctos_gazebo, 'worlds', 'arctos_world.sdf')
    rviz_config = os.path.join(pkg_arctos_desc,   'rviz',   'arctos.rviz')

    # ------------------------------------------------------------------ #
    # Generate URDF from xacro at Python level (before launch graph).    #
    # Writing the URDF to a temp file lets Gazebo load it via <include>  #
    # in the world SDF — no gz-transport service call required.           #
    # This is the reliable alternative to ros_gz_sim create, which uses  #
    # gz-transport multicast discovery that breaks on WSL2 loopback.     #
    # ------------------------------------------------------------------ #
    urdf_bytes = subprocess.check_output(['xacro', xacro_file])
    urdf_str   = urdf_bytes.decode('utf-8')

    urdf_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='/tmp/arctos_sim_')
    urdf_tmp.write(urdf_str)
    urdf_tmp.close()
    urdf_path = urdf_tmp.name

    # ------------------------------------------------------------------ #
    # Build a world SDF that includes the robot.                          #
    # Gazebo Harmonic (libsdformat 14) converts URDF to SDF on the fly   #
    # when it encounters <include><uri>file://…</uri></include>.          #
    # ------------------------------------------------------------------ #
    with open(world_file, 'r') as f:
        world_sdf = f.read()

    robot_include_xml = (
        '\n'
        '    <!-- Robot model: generated from arctos_sim.xacro at launch time -->\n'
        '    <include>\n'
        f'      <uri>file://{urdf_path}</uri>\n'
        '      <name>arctos</name>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
        '  '
    )
    world_sdf_with_robot = world_sdf.replace('  </world>', robot_include_xml + '</world>')

    world_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', delete=False, prefix='/tmp/arctos_world_')
    world_tmp.write(world_sdf_with_robot)
    world_tmp.close()
    world_with_robot_file = world_tmp.name

    # ------------------------------------------------------------------ #
    # GZ_SIM_RESOURCE_PATH                                                 #
    # Gazebo resolves package:// URIs by scanning GZ_SIM_RESOURCE_PATH   #
    # for directories named after ROS packages.                           #
    # ------------------------------------------------------------------ #
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    gz_share_paths = [
        os.path.join(p, 'share')
        for p in ament_prefix_path.split(':') if p
    ]
    existing_gz_path   = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path   = ':'.join(filter(None, gz_share_paths + [existing_gz_path]))

    # ------------------------------------------------------------------ #
    # GZ_SIM_SYSTEM_PLUGIN_PATH                                           #
    # ros-humble-gz-ros2-control installs its plugin (.so) here.         #
    # ------------------------------------------------------------------ #
    existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = ':'.join(filter(None, ['/opt/ros/humble/lib', existing_gz_plugin_path]))

    # ------------------------------------------------------------------ #
    # Arguments                                                            #
    # ------------------------------------------------------------------ #
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 alongside the simulation',
    )

    # ------------------------------------------------------------------ #
    # robot_state_publisher                                                #
    # Use the already-generated URDF string directly rather than running  #
    # xacro again via Command substitution.                               #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(urdf_str, value_type=str),
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------ #
    # Gazebo Harmonic server                                               #
    # Loads the generated world SDF which already contains the robot.    #
    # No separate spawn step is needed.                                   #
    # ------------------------------------------------------------------ #
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_with_robot_file],
        output='screen',
        emulate_tty=True,
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
            'OGRE_RTT_MODE': 'Copy',
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        },
    )

    # ------------------------------------------------------------------ #
    # Controllers                                                          #
    # gz_ros2_control starts controller_manager inside Gz when the model  #
    # loads. Wait 30 s for Gazebo + plugin to be ready, then spawn.      #
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

    # Chain: joint_state_broadcaster → arm + hand controllers
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, hand_controller_spawner],
        )
    )

    # ------------------------------------------------------------------ #
    # ros_gz_bridge — bridge Gazebo D435 camera topics into ROS2          #
    # ------------------------------------------------------------------ #
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------ #
    # depth_image_proc — organised XYZRGB pointcloud from depth image    #
    # ------------------------------------------------------------------ #
    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb',
        remappings=[
            ('rgb/image_rect_color',        '/camera/image'),
            ('rgb/camera_info',             '/camera/camera_info'),
            ('depth_registered/image_rect', '/camera/depth_image'),
            ('depth_registered/points',     '/camera/points_registered'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # RViz2 (optional)                                                     #
    # ------------------------------------------------------------------ #
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
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
        # Wait 30 s for Gazebo + gz_ros2_control plugin to initialise,
        # then spawn controllers. (No separate robot-spawn step needed —
        # the robot is already in the world SDF.)
        TimerAction(period=30.0, actions=[joint_state_broadcaster_spawner]),
        spawn_arm_after_jsb,
        ros_gz_bridge,
        point_cloud_node,
        rviz2,
    ])
