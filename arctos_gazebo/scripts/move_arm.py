#!/usr/bin/env python3
"""
move_arm.py — Send joint-position goals to the Arctos arm in Gazebo.

Usage
-----
# One-shot: set all 6 arm joints (radians) then exit
  ros2 run arctos_gazebo move_arm -- X Y Z A B C

# With explicit gripper position (metres, 0..0.015)
  ros2 run arctos_gazebo move_arm -- X Y Z A B C --gripper 0.01

# With custom move duration (seconds, default 2.0)
  ros2 run arctos_gazebo move_arm -- X Y Z A B C --duration 4.0

# Interactive mode — prompts for joint values in a loop
  ros2 run arctos_gazebo move_arm

Joint limits (radians unless noted)
-------------------------------------
  X_joint      : -3.14159  ..  3.14159
  Y_joint      : -0.96     ..  2.18
  Z_joint      : -1.5708   ..  0.9599
  A_joint      : -1.5708   ..  1.5708
  B_joint      : -1.55     ..  1.55
  C_joint      : -3.14159  ..  3.14159
  Left_jaw     :  0.0      ..  0.015 m
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ARM_JOINTS = ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint']
GRIPPER_JOINT = 'Left_jaw_joint'

LIMITS = {
    'X_joint':      (-math.pi,   math.pi),
    'Y_joint':      (-0.96,      2.18),
    'Z_joint':      (-math.pi/2, 0.9599),
    'A_joint':      (-math.pi/2, math.pi/2),
    'B_joint':      (-1.55,      1.55),
    'C_joint':      (-math.pi,   math.pi),
    GRIPPER_JOINT:  (0.0,        0.015),
}

ARM_CONTROLLER    = '/arctos_arm_controller/follow_joint_trajectory'
HAND_CONTROLLER   = '/arctos_hand_controller/follow_joint_trajectory'


def clamp(val, lo, hi, name):
    if val < lo or val > hi:
        print(f'  Warning: {name}={val:.4f} out of range [{lo}, {hi}]; clamping.')
    return max(lo, min(hi, val))


def make_goal(joints, positions, duration_sec):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints
    pt = JointTrajectoryPoint()
    pt.positions = positions
    secs = int(duration_sec)
    nsecs = int((duration_sec - secs) * 1e9)
    pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
    goal.trajectory.points = [pt]
    return goal


class ArmMover(Node):
    def __init__(self):
        super().__init__('move_arm')
        self._arm_client  = ActionClient(self, FollowJointTrajectory, ARM_CONTROLLER)
        self._hand_client = ActionClient(self, FollowJointTrajectory, HAND_CONTROLLER)

    def send(self, arm_positions, gripper_pos, duration_sec):
        arm_pos  = [clamp(v, *LIMITS[j], j) for v, j in zip(arm_positions, ARM_JOINTS)]
        hand_pos = clamp(gripper_pos, *LIMITS[GRIPPER_JOINT], GRIPPER_JOINT)

        print(f'\nSending arm goal  : {[round(v, 4) for v in arm_pos]}')
        print(f'Sending gripper   : {hand_pos:.4f} m')
        print(f'Move duration     : {duration_sec} s')

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Arm action server not available: {ARM_CONTROLLER}')
            return False

        arm_future = self._arm_client.send_goal_async(
            make_goal(ARM_JOINTS, arm_pos, duration_sec))
        rclpy.spin_until_future_complete(self, arm_future)
        arm_handle = arm_future.result()
        if not arm_handle.accepted:
            self.get_logger().error('Arm goal rejected.')
            return False

        # Send gripper goal only if hand controller is up
        if self._hand_client.wait_for_server(timeout_sec=2.0):
            hand_future = self._hand_client.send_goal_async(
                make_goal([GRIPPER_JOINT], [hand_pos], duration_sec))
            rclpy.spin_until_future_complete(self, hand_future)
        else:
            self.get_logger().warning('Hand controller not available; skipping gripper.')

        # Wait for arm to finish
        result_future = arm_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        print('Done.')
        return True


def parse_args():
    p = argparse.ArgumentParser(
        description='Send joint-position goals to the Arctos arm in Gazebo.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('positions', nargs='*', type=float, metavar='RAD',
                   help='6 arm joint positions in radians (X Y Z A B C). '
                        'Omit for interactive mode.')
    p.add_argument('--gripper', type=float, default=0.0, metavar='M',
                   help='Gripper opening in metres [0..0.015] (default: 0.0 = closed)')
    p.add_argument('--duration', type=float, default=2.0, metavar='SEC',
                   help='Move duration in seconds (default: 2.0)')
    return p.parse_args()


def interactive_loop(mover):
    print('\n=== Arctos arm interactive control ===')
    print('Joints: X  Y  Z  A  B  C  (radians)   gripper (metres, 0..0.015)')
    print('Type 6 values, optionally followed by gripper and duration.')
    print('Examples:')
    print('  0 0 0 0 0 0')
    print('  0.5 1.0 -0.3 0 0 0 0.01 3.0')
    print('Enter "q" to quit.\n')

    while True:
        try:
            raw = input('joint values> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        parts = raw.split()
        if len(parts) < 6:
            print(f'  Need at least 6 values, got {len(parts)}.')
            continue
        try:
            vals     = [float(x) for x in parts]
            arm_pos  = vals[:6]
            gripper  = vals[6] if len(vals) > 6 else 0.0
            duration = vals[7] if len(vals) > 7 else 2.0
            mover.send(arm_pos, gripper, duration)
        except ValueError as e:
            print(f'  Parse error: {e}')


def main():
    args = parse_args()

    rclpy.init()
    mover = ArmMover()

    if args.positions:
        if len(args.positions) != 6:
            print(f'Error: expected 6 joint positions, got {len(args.positions)}.', file=sys.stderr)
            sys.exit(1)
        mover.send(args.positions, args.gripper, args.duration)
    else:
        interactive_loop(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
