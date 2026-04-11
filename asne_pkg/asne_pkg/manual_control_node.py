#!/usr/bin/env python3

import rclpy
from math import atan2, sin, cos
from rclpy.node import Node
from std_msgs.msg import Float64
from asne_interfaces.msg import RCcontroller

# Right joystick: linear velocity
# Left joystick: desired heading


class ManualContolNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")

        self.desired_heading: float = 1.0
        self.thrust_norm: float = 0.0  # percentage btw -1.00 and 1.00

        self.rc_msg_sub = self.create_subscription(RCcontroller, "/asne/rc/channels", self.rc_msg_callback, 10)
        self.heading_pub = self.create_publisher(Float64, "/asne/heading/manual", 10)
        self.manual_thrust_pub = self.create_publisher(Float64, "/asne/thrust/manual", 10)

        self.manual_pub_timer = self.create_timer(0.1, self.manual_timer_pub_cb)

    def rc_msg_callback(self, msg: RCcontroller):
        # self.desired_linear_velocity = float(max(-self.max_linear_velocity, min((msg.joy_r_ud**3) * self.max_linear_velocity, self.max_linear_velocity)))
        self.thrust_norm = float(max(-1.00, min(msg.joy_r_ud**3, 1)))

        x: float = msg.joy_l_lr
        y: float = msg.joy_l_ud
        self.desired_heading = atan2(y, x)
        self.desired_heading = atan2(sin(self.desired_heading), cos(self.desired_heading))  # norm. to (-pi, pi]

    def manual_timer_pub_cb(self):
        self.heading_pub.publish(Float64(data=self.desired_heading))
        self.manual_thrust_pub.publish(Float64(data=self.thrust_norm))


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
