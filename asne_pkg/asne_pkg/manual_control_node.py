#!/usr/bin/env python3

import rclpy
from math import pi
from rclpy.node import Node
from std_msgs.msg import Float64
from asne_interfaces.msg import RCcontroller

# Right joystick UD: torque
# Left joystick LR: servo angle directly

SERVO_RANGE = 120 * (pi / 180)  # radians


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")

        self.declare_parameter("max_torque_dt", 0.05)
        self.max_torque_dt: float = self.get_parameter("max_torque_dt").get_parameter_value().double_value

        self.desired_servo: float = 0.0
        self.target_torque: float = 0.0  # joystick command, percentage btw -1.00 and 1.00
        self.current_torque: float = 0.0  # rate-limited actual, percentage btw -1.00 and 1.00

        self.rc_msg_sub = self.create_subscription(RCcontroller, "/asne/rc/channels", self.rc_msg_callback, 10)
        self.servo_pub = self.create_publisher(Float64, "/asne/servo_angle/manual", 10)
        self.manual_thrust_pub = self.create_publisher(Float64, "/asne/torque/manual", 10)

        self.manual_pub_timer = self.create_timer(0.1, self.manual_timer_pub_cb)

    def rc_msg_callback(self, msg: RCcontroller):
        diff = self.target_torque - self.current_torque
        self.current_torque += max(-self.max_torque_dt, min(diff, self.max_torque_dt))

        self.target_torque = float(max(-1.0, min(msg.joy_r_ud**3, 1.0)))

        self.desired_servo = msg.joy_l_lr * SERVO_RANGE # should be between -2.094 and 2.094 (ish)

    def manual_timer_pub_cb(self):
        self.servo_pub.publish(Float64(data=self.desired_servo))
        self.manual_thrust_pub.publish(Float64(data=self.target_torque))


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    manual_control_node = ManualControlNode()
    try:
        rclpy.spin(manual_control_node)
    finally:
        manual_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
