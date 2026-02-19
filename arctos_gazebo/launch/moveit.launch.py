"""
moveit.launch.py
================
Launches the MoveIt move_group node and RViz (with the MoveIt motion planning
panel) for the Arctos robot arm running in Gazebo.

This launch file is designed to run ALONGSIDE sim.launch.py, not instead of it.
The sim must already be up with controllers active before you run this.

Usage:
  # Terminal 1 — start the simulation (wait for controllers to come up, ~35 s):
  ros2 launch arctos_gazebo sim.launch.py rviz:=false

  # Terminal 2 — start MoveIt + RViz:
  ros2 launch arctos_gazebo moveit.launch.py

What this launches:
  1. move_group  — the MoveIt planning server.  It reads /robot_description
                   (already published by sim's robot_state_publisher), loads the
                   SRDF + kinematics + OMPL config, and connects to the arm /
                   gripper action servers that ros2_control is already serving.
  2. rviz2       — pre-loaded with the MoveIt MotionPlanning panel so you can
                   drag the interactive marker to a goal pose, click Plan, and
                   click Execute.  Disable with  rviz:=false.
"""

import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _read_file(path: str) -> str:
    with open(path) as f:
        return f.read()


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Paths                                                                #
    # ------------------------------------------------------------------ #
    pkg = get_package_share_directory('arctos_gazebo')
    pkg_desc = get_package_share_directory('arctos_description')

    xacro_file       = os.path.join(pkg, 'urdf',   'arctos_sim.xacro')
    srdf_file        = os.path.join(pkg, 'config', 'arctos.srdf')
    kinematics_file  = os.path.join(pkg, 'config', 'kinematics.yaml')
    controllers_file = os.path.join(pkg, 'config', 'moveit_controllers.yaml')
    joint_lim_file   = os.path.join(pkg, 'config', 'joint_limits.yaml')
    ompl_file        = os.path.join(pkg, 'config', 'ompl_planning.yaml')
    rviz_config_file = os.path.join(pkg, 'config', 'moveit.rviz')

    # ------------------------------------------------------------------ #
    # Generate URDF from xacro                                            #
    # move_group needs robot_description as a parameter.  The sim's      #
    # robot_state_publisher already has it, but move_group reads it from  #
    # its own parameter space, so we generate it here too.               #
    # ------------------------------------------------------------------ #
    urdf_str = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    # ------------------------------------------------------------------ #
    # Load config files                                                    #
    # ------------------------------------------------------------------ #
    srdf_str        = _read_file(srdf_file)
    kinematics      = _load_yaml(kinematics_file)
    controllers     = _load_yaml(controllers_file)
    joint_limits    = _load_yaml(joint_lim_file)
    ompl_planning   = _load_yaml(ompl_file)

    # ------------------------------------------------------------------ #
    # Arguments                                                            #
    # ------------------------------------------------------------------ #
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with the MoveIt motion planning panel',
    )

    # ------------------------------------------------------------------ #
    # move_group                                                           #
    #                                                                     #
    # Parameters breakdown:                                               #
    #   robot_description           — URDF string (kinematic model)       #
    #   robot_description_semantic  — SRDF string (groups, states, EEF)   #
    #   robot_description_kinematics — KDL IK solver per planning group   #
    #   robot_description_planning  — joint velocity/accel limits         #
    #   planning_pipelines          — list of enabled planners            #
    #   ompl / default_planning_pipeline — OMPL planner config            #
    #   moveit_controller_manager   — which controller plugin to use      #
    #   moveit_simple_controller_manager — controller→action mappings     #
    #   use_sim_time                — sync with Gazebo /clock             #
    # ------------------------------------------------------------------ #
    move_group_params = [
        {'robot_description':          urdf_str},
        {'robot_description_semantic': srdf_str},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning':   joint_limits},
        {'planning_pipelines':           ['ompl']},
        {'default_planning_pipeline':    'ompl'},
        {'ompl':                         ompl_planning},
        # Unpack the two top-level keys from moveit_controllers.yaml:
        #   moveit_controller_manager  (string)
        #   moveit_simple_controller_manager  (dict)
        controllers,
        {'use_sim_time': True},
        # Enable capability plugins needed for the RViz panel
        {'move_group/capabilities': ' '.join([
            'move_group/MoveGroupCartesianPathService',
            'move_group/MoveGroupExecuteTrajectoryAction',
            'move_group/MoveGroupGetPlanningSceneService',
            'move_group/MoveGroupKinematicsService',
            'move_group/MoveGroupMoveAction',
            'move_group/MoveGroupPlanService',
            'move_group/MoveGroupQueryPlannersService',
            'move_group/MoveGroupStateValidationService',
        ])},
        {'publish_robot_description_semantic': True},
        {'allow_trajectory_execution': True},
        {'monitor_dynamics': False},
    ]

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # ------------------------------------------------------------------ #
    # RViz2 — MoveIt motion planning panel                                 #
    #                                                                     #
    # RViz also needs robot_description_semantic and kinematics so it    #
    # can render the planning scene and compute IK for the marker goal.  #
    # ------------------------------------------------------------------ #
    rviz_params = [
        {'robot_description':          urdf_str},
        {'robot_description_semantic': srdf_str},
        {'robot_description_kinematics': kinematics},
        {'use_sim_time': True},
    ]

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        arguments=['-d', rviz_config_file],
        parameters=rviz_params,
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        move_group,
        rviz2,
    ])
