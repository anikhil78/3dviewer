"""
sim.launch.py
=============
Launches a Gazebo Harmonic simulation of the Arctos robot arm with full
ros2_control — the same joint_trajectory_controller pipeline used on real
hardware.  This means joint commands tested in simulation transfer directly
to the physical arm without code changes.

  1. Processes arctos_sim.xacro → URDF (at Python level, before launch graph)
  2. Patches arctos_world.sdf to embed the robot via <include> — no gz-transport
     spawn call needed (ros_gz_sim create uses loopback multicast that breaks on
     WSL2; the SDF-include path bypasses this entirely)
  3. Starts Gazebo Harmonic with the patched world SDF
  4. Starts robot_state_publisher with the same URDF string
  5. Waits 30 s for Gazebo + gz_ros2_control plugin to initialise, then spawns
     controllers in order:
       joint_state_broadcaster  → publishes /joint_states from sim
       arctos_arm_controller    → joint_trajectory_controller (position, 6 DOF)
       arctos_hand_controller   → GripperActionController (Left_jaw_joint)
  6. Starts ros_gz_bridge for clock + D435 camera topics
  7. Starts depth_image_proc for organised XYZRGB point cloud
  8. Optionally starts RViz2

PREREQUISITE: gz_ros2_control must be built from source against gz-sim 8.
  The apt package ros-humble-gz-ros2-control 0.7.17 is compiled against
  gz-plugin 1.x (Ignition Fortress) and will NOT load with gz-sim 8 (Harmonic).
  Run  scripts/setup_gz_ros2_control.sh  once to build and install it.
  Then always source ~/ros2_ws/install/setup.bash before launching.

Usage:
  source ~/ros2_ws/install/setup.bash
  ros2 launch arctos_gazebo sim.launch.py
  ros2 launch arctos_gazebo sim.launch.py rviz:=false

Verify controllers after ~35 s:
  ros2 control list_controllers
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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
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
    # Prepend the source-built gz_ros2_control paths so Gazebo finds the #
    # correct gz-plugin 2.x build before any system paths.               #
    # The BUILD directory is listed first because gz-sim's SystemLoader  #
    # does not always follow symlinks when scanning plugin dirs, so the  #
    # real .so files must be reachable via a non-symlink path.           #
    # Gazebo's own plugin dirs (/usr/lib/.../gz-sim-8/plugins) are always #
    # searched regardless — this env var is purely additive.             #
    # Do NOT add /opt/ros/humble/lib — that package has been removed.    #
    # ------------------------------------------------------------------ #
    home_dir = os.path.expanduser('~')
    ws_install_lib = os.path.join(home_dir, 'ros2_ws', 'install', 'gz_ros2_control', 'lib')
    ws_build_lib   = os.path.join(home_dir, 'ros2_ws', 'build',   'gz_ros2_control')
    existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = ':'.join(filter(None, [
        ws_build_lib,             # actual .so files (gz-sim does not always follow symlinks)
        ws_install_lib,           # install-tree symlinks (belt-and-suspenders)
        existing_gz_plugin_path,  # anything already on the path
    ]))

    # ------------------------------------------------------------------ #
    # Arguments                                                            #
    # ------------------------------------------------------------------ #
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 alongside the simulation',
    )

    # ------------------------------------------------------------------ #
    # Static TF: world → base_link                                        #
    # robot_state_publisher publishes transforms relative to base_link   #
    # but publishes no world-frame anchor. Without this, RViz2 cannot    #
    # locate camera_optical_frame (or any robot frame) in the world      #
    # frame, making the point cloud invisible.                            #
    # ------------------------------------------------------------------ #
    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        # Do NOT set use_sim_time here: static_transform_publisher will not
        # publish any transform until it receives a /clock message, creating a
        # deadlock where RViz2 can never resolve the world frame before Gazebo
        # is fully up.  Static transforms are time-independent so wall-clock is
        # correct regardless.
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
            'GZ_IP': '127.0.0.1',
        },
    )

    # ------------------------------------------------------------------ #
    # ros2_control controllers                                             #
    # gz_ros2_control starts controller_manager inside Gazebo when the   #
    # robot model loads. We wait 30 s for Gazebo + plugin to initialise, #
    # then spawn controllers in order.                                    #
    #                                                                     #
    # IMPORTANT: requires gz_ros2_control built against gz-plugin 2.x.   #
    # The apt package ros-humble-gz-ros2-control 0.7.17 is compiled for  #
    # gz-plugin 1.x (Ignition Fortress) and will NOT load with gz-sim 8. #
    # Build from source — see README for instructions.                    #
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
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True, 'lazy': False}],
        additional_env={'GZ_IP': '127.0.0.1'},
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
        # Force all child processes (Gazebo, bridge, …) to bind gz-transport
        # to the loopback interface.  Without this, gz-transport discovery uses
        # 172.x.x.x (WSL2 virtual NIC) while Gazebo listens on 127.0.0.1, so
        # the bridge never discovers Gazebo and /camera/image has no subscriber.
        # SetEnvironmentVariable modifies os.environ of the launch process
        # itself, so every forked child inherits it — more reliable than
        # per-node additional_env which can fail to propagate in Humble.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        rviz_arg,
        world_to_base,
        robot_state_publisher,
        gz_sim,
        # Wait 30 s for Gazebo + gz_ros2_control plugin to initialise,
        # then spawn controllers. The robot is already in the world SDF so
        # no separate spawn step is needed.
        TimerAction(period=30.0, actions=[joint_state_broadcaster_spawner]),
        spawn_arm_after_jsb,
        ros_gz_bridge,
        point_cloud_node,
        rviz2,
    ])
