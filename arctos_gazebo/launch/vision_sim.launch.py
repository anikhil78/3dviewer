"""
vision_sim.launch.py
====================
Standalone Gazebo Harmonic simulation for vision development.

No robot arm, no controllers.  Launches:
  1. Processes vision_camera.urdf.xacro → URDF (camera-only, world-fixed)
  2. Patches vision_world.sdf to inject the camera via <include>
  3. Starts Gazebo Harmonic with the patched world
  4. Starts robot_state_publisher (camera TF only — world → camera frames)
  5. ros_gz_bridge — bridges /vision/* camera topics into ROS 2
  6. depth_image_proc — dense XYZRGB point cloud from depth + colour images
  7. Optionally starts RViz2

World contents:
  - Grey table (0.6 × 0.4 × 0.05 m)
  - Red box, blue cylinder, green sphere on the table
  - D435-style RGBD camera at (0.4, 0, 0.42), pitched 40° downward

Camera topics (ROS 2):
  /vision/image               — 640×480 RGB
  /vision/depth_image         — 640×480 float32 depth (metres)
  /vision/camera_info         — intrinsics
  /vision/points              — PointCloud2 (from Gazebo sensor)
  /vision/points_registered   — organised XYZRGB cloud (from depth_image_proc)

Usage:
  source ~/ros2_ws/install/setup.bash
  ros2 launch arctos_gazebo vision_sim.launch.py
  ros2 launch arctos_gazebo vision_sim.launch.py rviz:=false

Then run the vision processor in another terminal:
  ros2 run arctos_gazebo vision_processor
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Paths                                                                #
    # ------------------------------------------------------------------ #
    pkg = get_package_share_directory('arctos_gazebo')

    xacro_file = os.path.join(pkg, 'urdf',   'vision_camera.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'vision_world.sdf')

    # ------------------------------------------------------------------ #
    # Process xacro → URDF string (camera-only URDF, no arm)             #
    # ------------------------------------------------------------------ #
    urdf_bytes = subprocess.check_output(['xacro', xacro_file])
    urdf_str   = urdf_bytes.decode('utf-8')

    urdf_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='/tmp/vision_camera_')
    urdf_tmp.write(urdf_str)
    urdf_tmp.close()
    urdf_path = urdf_tmp.name

    # ------------------------------------------------------------------ #
    # Patch world SDF to inject camera via <include>                      #
    # (same mechanism as sim.launch.py — avoids gz-transport spawn which  #
    # breaks on WSL2 loopback multicast)                                  #
    # ------------------------------------------------------------------ #
    with open(world_file, 'r') as f:
        world_sdf = f.read()

    camera_include_xml = (
        '\n'
        '    <!-- Camera model: generated from vision_camera.urdf.xacro -->\n'
        '    <include>\n'
        f'      <uri>file://{urdf_path}</uri>\n'
        '      <name>vision_camera</name>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
        '  '
    )
    world_sdf_with_camera = world_sdf.replace(
        '  </world>', camera_include_xml + '</world>')

    world_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', delete=False, prefix='/tmp/vision_world_')
    world_tmp.write(world_sdf_with_camera)
    world_tmp.close()
    patched_world_file = world_tmp.name

    # ------------------------------------------------------------------ #
    # Environment — same WSL2 / software-rendering setup as sim.launch   #
    # ------------------------------------------------------------------ #
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    gz_share_paths = [
        os.path.join(p, 'share')
        for p in ament_prefix_path.split(':') if p
    ]
    gz_resource_path = ':'.join(
        filter(None, gz_share_paths + [os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    home_dir = os.path.expanduser('~')
    ws_install_lib = os.path.join(
        home_dir, 'ros2_ws', 'install', 'gz_ros2_control', 'lib')
    ws_build_lib = os.path.join(
        home_dir, 'ros2_ws', 'build', 'gz_ros2_control')
    gz_plugin_path = ':'.join(filter(None, [
        ws_build_lib,
        ws_install_lib,
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]))

    # ------------------------------------------------------------------ #
    # Arguments                                                            #
    # ------------------------------------------------------------------ #
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for point cloud and marker visualisation')

    # ------------------------------------------------------------------ #
    # robot_state_publisher — publishes camera TF only                    #
    # (world → camera_link → camera_optical_frame)                       #
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
    # Gazebo Harmonic                                                      #
    # ------------------------------------------------------------------ #
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', patched_world_file],
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
    # ros_gz_bridge — /vision/* topics                                    #
    # ------------------------------------------------------------------ #
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vision_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/vision/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vision/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vision/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vision/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True, 'lazy': False}],
        additional_env={'GZ_IP': '127.0.0.1'},
    )

    # ------------------------------------------------------------------ #
    # depth_image_proc — dense organised XYZRGB cloud                    #
    # Remaps the standard topic names to our /vision/* namespace.        #
    # Output: /vision/points_registered                                   #
    # ------------------------------------------------------------------ #
    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='vision_point_cloud',
        remappings=[
            ('rgb/image_rect_color',        '/vision/image'),
            ('rgb/camera_info',             '/vision/camera_info'),
            ('depth_registered/image_rect', '/vision/depth_image'),
            ('depth_registered/points',     '/vision/points_registered'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # RViz2 (optional)                                                     #
    # Shows: PointCloud2, MarkerArray from vision_processor               #
    # ------------------------------------------------------------------ #
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        gz_sim,
        ros_gz_bridge,
        point_cloud_node,
        rviz2,
    ])
