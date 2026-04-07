#!/usr/bin/env python3

import sys
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class FlapTeleop(Node):
    def __init__(self):
        super().__init__('flap_teleop')

        # CORRECT topics (this was the bug)
        self.left_pub = self.create_publisher(
            Float64, '/left_flap_controller/command', 10
        )
        self.right_pub = self.create_publisher(
            Float64, '/right_flap_controller/command', 10
        )

        self.left_pos = 0.0
        self.right_pos = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info(
            "\nFLAP TELEOP ACTIVE\n"
            "a = open LEFT\n"
            "z = close LEFT\n"
            "s = open RIGHT\n"
            "x = close RIGHT\n"
            "q = quit\n"
        )

        self.create_timer(0.05, self.loop)

    def loop(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            if key == 'q':
                rclpy.shutdown()
                return

            if key == 'a':
                self.left_pos += 0.1
            elif key == 'z':
                self.left_pos -= 0.1
            elif key == 's':
                self.right_pos += 0.1
            elif key == 'x':
                self.right_pos -= 0.1
            else:
                return

            self.left_pos = max(-1.2, min(1.2, self.left_pos))
            self.right_pos = max(-1.2, min(1.2, self.right_pos))

            self.left_pub.publish(Float64(data=self.left_pos))
            self.right_pub.publish(Float64(data=self.right_pos))


def main():
    rclpy.init()
    node = FlapTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
