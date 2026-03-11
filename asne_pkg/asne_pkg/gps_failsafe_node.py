#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.time import Time
from asne_interfaces.msg import State
from asne_interfaces.srv import SetState
from std_srvs.srv import Trigger


class GPSFailsafeNode(Node):
    def __init__(self):
        super().__init__("gps_failsafe_node")

        self.declare_parameter("gps_timeout_sec", 2.0)
        self.timeout = self.get_parameter("gps_timeout_sec").value

        self.gps_failsafe_active: bool = False
        self.last_gps_time: Time | None = None

        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.check_expiration_timer = self.create_timer(0.5, self.check_gps_expiration)

        self.set_state_client = self.create_client(SetState, "/asne/set_state")
        self.reset_estop_client = self.create_client(Trigger, "/asne/reset_estop")

    def gps_callback(self, msg: NavSatFix):
        self.last_gps_time = msg.header.stamp
        if self.gps_failsafe_active:
            self.get_logger().info("recovered GPS signal, stopping estop")
            self.gps_failsafe_active = False

            if self.reset_estop_client.wait_for_service(timeout_sec=0.2):
                self.reset_estop_client.call_async(Trigger.Request())
            else:
                self.get_logger().error("failed to reset estop")

    def check_gps_expiration(self):
        if self.last_gps_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_gps_time).nanoseconds * 1e-9
        if elapsed > self.timeout and not self.gps_failsafe_active:
            self.gps_failsafe_active = True
            self.get_logger().warn("GPS signal lost :(")

            if self.set_state_client.wait_for_service(timeout_sec=0.2):
                req = SetState.Request()
                req.state = State.GPS_ESTOP
                self.set_state_client.call_async(req)
            else:
                self.get_logger().error("failed to activate rc failsafe")


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    gps_failsafe_node = GPSFailsafeNode()
    try:
        rclpy.spin(gps_failsafe_node)
    finally:
        gps_failsafe_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
