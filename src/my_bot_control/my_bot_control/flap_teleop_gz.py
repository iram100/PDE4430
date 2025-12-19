#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

LEFT_TOPIC = '/model/my_bot/left_flap_cmd'
RIGHT_TOPIC = '/model/my_bot/right_flap_cmd'

STEP = 0.2
MIN_POS = -1.2
MAX_POS = 1.2


class FlapTeleop(Node):
    def __init__(self):
        super().__init__('flap_teleop_gz')

        self.left_pub = self.create_publisher(Float64, LEFT_TOPIC, 10)
        self.right_pub = self.create_publisher(Float64, RIGHT_TOPIC, 10)

        self.left = 0.0
        self.right = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info(
            "\nFLAP TELEOP (CAPITAL LETTERS)\n"
            "-----------------------------\n"
            "O / P  → Open flaps\n"
            "K / L  → Close flaps\n"
            "R      → Reset\n"
            "Ctrl+C → Quit\n"
        )

        self.create_timer(0.05, self.loop)

    def loop(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            if key == '\x03':
                rclpy.shutdown()
                return

            if key == 'O':
                self.left = max(self.left - STEP, MIN_POS)
                self.right = min(self.right + STEP, MAX_POS)

            elif key == 'P':
                self.left = min(self.left + STEP, MAX_POS)
                self.right = max(self.right - STEP, MIN_POS)

            elif key == 'K':
                self.left = max(self.left + STEP, MIN_POS)
                self.right = min(self.right - STEP, MAX_POS)

            elif key == 'L':
                self.left = min(self.left - STEP, MAX_POS)
                self.right = max(self.right + STEP, MIN_POS)

            elif key == 'R':
                self.left = 0.0
                self.right = 0.0

            self.left_pub.publish(Float64(data=self.left))
            self.right_pub.publish(Float64(data=self.right))


def main():
    rclpy.init()
    node = FlapTeleop()
    try:
        rclpy.spin(node)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
