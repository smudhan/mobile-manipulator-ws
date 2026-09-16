#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

MSG = """
--------------------------------------------------
Control Your Robot!
--------------------------------------------------
Moving around:
        w
   a    s    d

Speed adjustments:
   i : increase speed (+0.1)
   o : decrease speed (-0.1)

CTRL-C to quit
--------------------------------------------------
"""

class TeleopContinuous(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher to diff_drive_controller
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        # Speed settings
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        self.speed_step = 0.1

        # Track currently held keys
        self.pressed_keys = set()
        self.lock = threading.Lock()

        # Publish continuously at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_twist)

        # Print current settings
        print(MSG)
        self.print_speed()

    def print_speed(self):
        print(f"\rCurrent Linear Speed: {self.linear_speed:.2f} m/s | Angular Speed: {self.angular_speed:.2f} rad/s", end='', flush=True)

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return  # Ignore special keys

        with self.lock:
            # Handle speed changes
            if k == 'i':
                self.linear_speed = round(self.linear_speed + self.speed_step, 2)
                self.angular_speed = round(self.angular_speed + self.speed_step * 2, 2)
                self.print_speed()
            elif k == 'o':
                self.linear_speed = max(0.0, round(self.linear_speed - self.speed_step, 2))
                self.angular_speed = max(0.0, round(self.angular_speed - self.speed_step * 2, 2))
                self.print_speed()
            elif k in ['w', 'a', 's', 'd']:
                self.pressed_keys.add(k)

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return

        with self.lock:
            if k in self.pressed_keys:
                self.pressed_keys.remove(k)

    def publish_twist(self):
        twist = Twist()
        
        with self.lock:
            # Linear motion (X axis)
            if 'w' in self.pressed_keys and 's' not in self.pressed_keys:
                twist.linear.x = self.linear_speed
            elif 's' in self.pressed_keys and 'w' not in self.pressed_keys:
                twist.linear.x = -self.linear_speed
            else:
                twist.linear.x = 0.0

            # Angular motion (Z axis)
            if 'a' in self.pressed_keys and 'd' not in self.pressed_keys:
                twist.angular.z = self.angular_speed
            elif 'd' in self.pressed_keys and 'a' not in self.pressed_keys:
                twist.angular.z = -self.angular_speed
            else:
                twist.angular.z = 0.0

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopContinuous()

    # Start non-blocking global keyboard listener
    listener = keyboard.Listener(
        on_press=teleop_node.on_press,
        on_release=teleop_node.on_release
    )
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero twist to ensure robot stops on exit
        stop_twist = Twist()
        teleop_node.publisher_.publish(stop_twist)
        
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()
        print("\nNode stopped.")

if __name__ == '__main__':
    main()