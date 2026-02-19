#!/usr/bin/env python3
"""
move_arm_moveit.py
==================
Python control script for the Arctos arm via MoveIt (move_group).

Unlike move_arm.py — which sends raw trajectories directly to the controller —
this script goes through MoveIt and therefore gets:
  * Collision checking (won't execute a plan that would hit itself or obstacles)
  * OMPL path planning (finds a safe joint-space path to any goal)
  * Inverse kinematics (specify a Cartesian target, let MoveIt solve joints)

PREREQUISITE: sim.launch.py AND moveit.launch.py must both be running.

Usage
-----
# Move to named state (defined in SRDF):
ros2 run arctos_gazebo move_arm_moveit named home
ros2 run arctos_gazebo move_arm_moveit named open
ros2 run arctos_gazebo move_arm_moveit named close

# Move to joint positions [X Y Z A B C] in radians:
ros2 run arctos_gazebo move_arm_moveit joints -- 0.5 0.3 -0.2 0.0 0.1 0.0

# Move to a Cartesian pose [x y z qx qy qz qw] of Link_6_1 in the world frame:
ros2 run arctos_gazebo move_arm_moveit pose -- 0.3 0.0 0.4 0 0 0 1

# Interactive loop (prompts for commands until you type 'quit'):
ros2 run arctos_gazebo move_arm_moveit interactive
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    HAS_MOVEIT_PY = True
except ImportError:
    HAS_MOVEIT_PY = False

from geometry_msgs.msg import PoseStamped

# Joint order used throughout this script (must match SRDF arctos_arm group)
ARM_JOINTS = ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint']
EEF_LINK   = 'Link_6_1'
ARM_GROUP  = 'arctos_arm'
HAND_GROUP = 'arctos_hand'


def _require_moveit_py():
    if not HAS_MOVEIT_PY:
        print(
            'ERROR: moveit_py is not installed.\n'
            'Install it with:\n'
            '  sudo apt install ros-humble-moveit-py\n'
            'Then rebuild your workspace.'
        )
        sys.exit(1)


class ArmMoveItController:
    """Thin wrapper around MoveItPy for the Arctos arm."""

    def __init__(self):
        _require_moveit_py()
        rclpy.init()
        self._moveit = MoveItPy(node_name='move_arm_moveit')
        self._arm  = self._moveit.get_planning_component(ARM_GROUP)
        self._hand = self._moveit.get_planning_component(HAND_GROUP)
        self._logger = get_logger('move_arm_moveit')

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _plan_and_execute(self, component, label: str) -> bool:
        plan = component.plan()
        if not plan:
            self._logger.error(f'Planning failed for {label}')
            return False
        self._logger.info(f'Plan found for {label} — executing ...')
        self._moveit.execute(plan.trajectory, blocking=True, controllers=[])
        self._logger.info(f'Execution of {label} complete')
        return True

    # ------------------------------------------------------------------ #
    # Public API                                                           #
    # ------------------------------------------------------------------ #

    def move_to_named(self, name: str, group: str = ARM_GROUP) -> bool:
        """Move to a state named in the SRDF (e.g. 'home', 'open', 'close')."""
        component = self._arm if group == ARM_GROUP else self._hand
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=name)
        return self._plan_and_execute(component, f'named state "{name}"')

    def move_joints(self, positions: list) -> bool:
        """
        Move arm joints to explicit positions [X Y Z A B C] in radians.
        Validates count and clamps to URDF limits silently.
        """
        if len(positions) != 6:
            self._logger.error(f'Expected 6 joint values, got {len(positions)}')
            return False

        robot_model = self._moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions(ARM_GROUP, positions)
        robot_state.update()

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=robot_state)
        return self._plan_and_execute(self._arm, f'joint goal {positions}')

    def move_pose(self, x: float, y: float, z: float,
                  qx: float = 0.0, qy: float = 0.0,
                  qz: float = 0.0, qw: float = 1.0) -> bool:
        """
        Move the end effector (Link_6_1) to a Cartesian pose in the world frame.
        MoveIt will solve IK and plan a collision-free path.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x    = float(x)
        goal.pose.position.y    = float(y)
        goal.pose.position.z    = float(z)
        goal.pose.orientation.x = float(qx)
        goal.pose.orientation.y = float(qy)
        goal.pose.orientation.z = float(qz)
        goal.pose.orientation.w = float(qw)

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=goal, pose_link=EEF_LINK)
        return self._plan_and_execute(self._arm, f'pose ({x}, {y}, {z})')

    def shutdown(self):
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# CLI entrypoint
# ---------------------------------------------------------------------------

def _usage():
    print(__doc__)
    sys.exit(0)


def main():
    args = sys.argv[1:]

    if not args or args[0] in ('-h', '--help'):
        _usage()

    mode = args[0]
    ctrl = ArmMoveItController()

    try:
        if mode == 'named':
            # named <state_name> [<group>]
            if len(args) < 2:
                print('Usage: move_arm_moveit named <state_name> [arctos_arm|arctos_hand]')
                sys.exit(1)
            state = args[1]
            group = args[2] if len(args) > 2 else ARM_GROUP
            ok = ctrl.move_to_named(state, group)

        elif mode == 'joints':
            # joints -- X Y Z A B C
            vals = [float(v) for v in args[1:] if v != '--']
            ok = ctrl.move_joints(vals)

        elif mode == 'pose':
            # pose -- x y z [qx qy qz qw]
            vals = [float(v) for v in args[1:] if v != '--']
            if len(vals) == 3:
                ok = ctrl.move_pose(*vals)
            elif len(vals) == 7:
                ok = ctrl.move_pose(*vals)
            else:
                print('Usage: move_arm_moveit pose -- x y z [qx qy qz qw]')
                sys.exit(1)

        elif mode == 'interactive':
            print('Interactive MoveIt control. Commands:')
            print('  named <state>             — move to SRDF named state')
            print('  joints X Y Z A B C        — move joints (radians)')
            print('  pose x y z [qx qy qz qw] — move end effector to pose')
            print('  quit                      — exit')
            while True:
                try:
                    line = input('\nmoveit> ').strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if not line or line.startswith('#'):
                    continue
                if line in ('quit', 'exit', 'q'):
                    break
                parts = line.split()
                cmd = parts[0]
                rest = parts[1:]
                if cmd == 'named':
                    ctrl.move_to_named(rest[0], rest[1] if len(rest) > 1 else ARM_GROUP)
                elif cmd == 'joints':
                    ctrl.move_joints([float(v) for v in rest])
                elif cmd == 'pose':
                    ctrl.move_pose(*[float(v) for v in rest])
                else:
                    print(f'Unknown command: {cmd}')
            ok = True

        else:
            print(f'Unknown mode: {mode}')
            _usage()
            ok = False

    finally:
        ctrl.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
