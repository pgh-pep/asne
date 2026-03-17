#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import pi
from asne_interfaces.msg import State


SERVO_RANGE = 120 * (pi / 180)  # radians

# TODO: determine if want to send angle on heading callback or on own timer (in case heading pub stalls)


class ServoControlNode(Node):
    def __init__(self):
        super().__init__("servo_control_node")

        self.manual_heading_sub = self.create_subscription(Float64, "/asne/heading/manual", 10, self.heading_manual_callback)
        self.auto_heading_sub = self.create_subscription(Float64, "/asne/heading/autonomous", 10, self.heading_auto_callback)
        self.state_sub = self.create_subscription(State, "/asne/state", 10, self.state_callback)

        self.servo_pub = self.create_publisher(Float64, "/asne/servo_angle", 10)

        self.desired_manual_heading: float = 0.0
        self.desired_autonomous_heading: float = 0.0

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.state: State | None = None

    def heading_manual_callback(self, msg: Float64):  # rads
        servo_angle = msg.data  # TODO: convert desired heading to servo angle
        servo_angle = max(min(SERVO_RANGE, servo_angle), -SERVO_RANGE)

        self.desired_manual_heading = servo_angle

    def heading_auto_callback(self, msg: Float64):  # rads
        servo_angle = msg.data  # TODO: convert desired heading to servo angle
        servo_angle = max(min(SERVO_RANGE, servo_angle), -SERVO_RANGE)

        self.desired_autonomous_heading = servo_angle
        
    def state_callback(self, msg: State):
        self.state = msg.state

    def timer_callback(self):
        servo_msg: Float64 = Float64()

        match self.state:
            case State.MANUAL:
                servo_angle = self.desired_manual_heading
            case State.AUTONOMOUS:
                servo_angle = self.desired_autonomous_heading
            case State.STATIONARY:
                servo_angle = 0.0
            case State.RC_ESTOP:
                servo_angle = 0.0
            case State.GPS_ESTOP:
                servo_angle = 0.0
            case _:
                servo_angle = 0.0

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
