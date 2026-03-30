#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from asne_interfaces.msg import RCcontroller


class ManualContolNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")

        self.declare_parameter("max_linear_velocity", 1.0)

        self.max_linear_velocity: float = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_linear_velocity: float = 50.0

        self.desired_heading: float = 1.0
        self.desired_linear_velocity: float = 1.0

        self.rc_msg_sub = self.create_subscription(RCcontroller, "/asne/rc/channels", self.rc_msg_callback, 10)
        self.heading_pub = self.create_publisher(Float64, "/asne/heading/manual", 10)
        self.velocity_pub = self.create_publisher(Float64, "/asne/velocity/manual", 10)

    def rc_msg_callback(self, msg: RCcontroller):
        # Right joystick: linear velocity
        # Left joystick: desired heading
        # self.desired_linear_velocity = self.clamp_velocity(0, self.max_linear_velocity, (msg.joy_r_ud**3) * self.max_linear_velocity)
        self.desired_linear_velocity = float(max(-self.max_linear_velocity, min((msg.joy_r_ud**3) * self.max_linear_velocity, self.max_linear_velocity)))

        x: float = msg.joy_l_lr
        y: float = msg.joy_l_ud
        self.desired_heading = math.atan2(y, x)

        self.heading_pub.publish(Float64(data=self.desired_heading))
        self.velocity_pub.publish(Float64(data=self.desired_linear_velocity))


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    manual_control_node = ManualContolNode()
    try:
        rclpy.spin(manual_control_node)
    finally:
        manual_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
