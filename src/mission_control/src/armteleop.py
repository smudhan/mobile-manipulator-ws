#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Key mappings: Lowercase decreases angle, Uppercase increases angle
KEY_MAPPINGS = {
    'q': (0, -1), 'Q': (0, 1),   # joint1
    'w': (1, -1), 'W': (1, 1),   # joint2
    'e': (2, -1), 'E': (2, 1),   # joint3
    'r': (3, -1), 'R': (3, 1),   # joint4
    't': (4, -1), 'T': (4, 1),   # joint5
    'y': (5, -1), 'Y': (5, 1),   # joint6
}

HELP_TEXT = """
-----------------------------------------------------------
Arm Joint Keyboard Teleop
-----------------------------------------------------------
Joint Controls (Decrease / Increase):
  joint1: [q] / [Shift + q]
  joint2: [w] / [Shift + w]
  joint3: [e] / [Shift + e]
  joint4: [r] / [Shift + r]
  joint5: [t] / [Shift + t]
  joint6: [y] / [Shift + y]

Reset to Zero : [0]
Step Size     : [+] increase step, [-] decrease step
Quit          : [Ctrl + C] or [ESC]
-----------------------------------------------------------
"""

def get_key(settings):
    """Capture a single keypress without waiting for Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_keyboard_teleop')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.positions = [0.0] * 6
        self.step = 0.05  # Radians per keypress (~2.86 degrees)
        self.time_to_reach = 1  # Duration in seconds for each move

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for FollowJointTrajectory action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected. Ready for keyboard input.')

    def send_goal(self):
        """Constructs and dispatches the FollowJointTrajectory goal."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.positions)
        point.time_from_start = Duration(sec=self.time_to_reach, nanosec=0)
        
        goal_msg.trajectory.points.append(point)
        self._action_client.send_goal_async(goal_msg)

    def print_state(self):
        formatted = [f"{p:+.2f}" for p in self.positions]
        print(f"\rPositions: {formatted} | Step: {self.step:.2f} rad", end="", flush=True)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArmTeleopNode()

    print(HELP_TEXT)
    node.print_state()

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in KEY_MAPPINGS:
                idx, direction = KEY_MAPPINGS[key]
                node.positions[idx] += direction * node.step
                node.send_goal()
                node.print_state()

            elif key == '0':
                node.positions = [0.0] * 6
                node.send_goal()
                node.print_state()

            elif key in ('+', '='):
                node.step = min(node.step + 0.01, 0.5)
                node.print_state()

            elif key in ('-', '_'):
                node.step = max(node.step - 0.01, 0.01)
                node.print_state()

            elif key == '\x03' or key == '\x1b':  # Ctrl+C or ESC
                break

            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nExiting teleoperation.")


if __name__ == '__main__':
    main()