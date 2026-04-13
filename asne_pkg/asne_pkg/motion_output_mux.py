#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from asne_interfaces.msg import State


from math import pi

SERVO_RANGE = 120 * (pi / 180)


class MotionOutputMux(Node):
    def __init__(self):
        super().__init__("motion_output_mux")

        self.manual_torque_sub = self.create_subscription(Float64, "/asne/torque/manual", self.manual_torque_callback, 10)
        self.auto_torque_sub = self.create_subscription(Float64, "/asne/torque/autonomous", self.auto_torque_callback, 10)
        self.manual_servo_angle_sub = self.create_subscription(Float64, "/asne/servo_angle/manual", self.manual_head_callback, 10)
        self.auto_servo_angle_sub = self.create_subscription(Float64, "/asne/servo_angle/autonomous", self.auto_head_callback, 10)
        self.state_sub = self.create_subscription(State, "/asne/state", self.state_callback, 10)
        self.estop_sub = self.create_subscription(Bool, "/asne/state/estop_active", self.estop_cb, 10)

        self.torque_pub = self.create_publisher(Float64, "/asne/torque/final", 10)
        self.servo_angle_pub = self.create_publisher(Float64, "/asne/servo_angle/final", 10)  # pubs in angles

        self.manual_torque: float = 0.0
        self.auto_torque: float = 0.0
        self.manual_servo_angle: float = 0.0
        self.auto_servo_angle: float = 0.0

        self.system_state: int = State.MANUAL
        self.is_estop: bool = False

        self.update_loop = self.create_timer(0.1, self.update_timer_cb)

    def manual_torque_callback(self, msg: Float64):
        self.manual_torque = msg.data

    def auto_torque_callback(self, msg: Float64):
        self.auto_torque = msg.data

    def manual_head_callback(self, msg: Float64):
        self.manual_servo_angle = msg.data

    def auto_head_callback(self, msg: Float64):
        self.auto_servo_angle = msg.data

    def state_callback(self, msg: State):
        self.system_state = msg.state

    def estop_cb(self, msg: Bool):
        self.is_estop = msg.data

    def update_timer_cb(self):
        # if self.is_estop:
        #     servo_angle = 0.0
        #     torque = 0.0
        # elif self.system_state == State.AUTONOMOUS:
        #     torque = self.auto_torque
        #     servo_angle = self.auto_servo_angle
        # else:
        # self.get_logger().info("BRO")
        torque = self.manual_torque
        servo_angle = self.manual_servo_angle

        servo_angle = max(min(SERVO_RANGE, servo_angle), -SERVO_RANGE)
        servo_angle = servo_angle * 180 / pi

        servo_angle = 135 + 85 * (servo_angle / 120)

        self.torque_pub.publish(Float64(data=torque))
        self.servo_angle_pub.publish(Float64(data=servo_angle))


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    motion_output_mux = MotionOutputMux()
    try:
        rclpy.spin(motion_output_mux)
    finally:
        motion_output_mux.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
