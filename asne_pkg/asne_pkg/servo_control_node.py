#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import pi


SERVO_RANGE = 120 * (pi / 180)  # radians

# TODO: determine if want to send angle on heading callback or on own timer (in case heading pub stalls)


class ServoControlNode(Node):
    def __init__(self):
        super().__init__("servo_control_node")

        self.heading_sub = self.create_subscription(Float64, "asne/goal_heading", 10, self.heading_callback)
        self.servo_pub = self.create_publisher(Float64, "asne/servo_angle", 10)

    def heading_callback(self, msg: Float64):  # rads
        servo_angle = msg.data  # TODO: convert desired heading to servo angle
        servo_angle = max(min(SERVO_RANGE, servo_angle), -SERVO_RANGE)

        servo_msg: Float64 = Float64()
        servo_msg.data = servo_angle  # rads
        self.servo_pub.publish()


def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    try:
        rclpy.spin(servo_control_node)
    finally:
        servo_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
