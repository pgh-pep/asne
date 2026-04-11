#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("sim_bridge_node")
        self.heading_sub = self.create_subscription(Float64, "/asne/heading/final", self.heading_callback, 10)
        self.velo_sub = self.create_subscription(Float64, "/asne/velocity/final", self.velocity_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_cb)

        self.pos_pub = self.create_publisher(Float64, "/wamv/thrusters/middle/pos", 10)
        self.thrust_pub = self.create_publisher(Float64, "/wamv/thrusters/middle/thrust", 10)

        self.heading: float = 0.0
        self.velocity: float = 0.0

    def heading_callback(self, msg: Float64):
        self.heading = msg.data

    def velocity_callback(self, msg: Float64):
        self.velocity = msg.data

    def timer_cb(self):
        pos_msg = Float64()
        pos_msg.data = self.heading

        thrust_msg = Float64()
        thrust_msg.data = self.velocity

        self.pos_pub.publish(pos_msg)
        self.thrust_pub.publish(thrust_msg)
        self.get_logger().info(f"Publishing pos={self.heading:.3f} thrust={self.velocity:.3f}")


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    sim_bridge_node = SimBridgeNode()
    try:
        rclpy.spin(sim_bridge_node)
    finally:
        sim_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
