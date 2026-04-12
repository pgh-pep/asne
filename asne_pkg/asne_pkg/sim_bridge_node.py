#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("sim_bridge_node")
        self.omega_sub = self.create_subscription(Float64, "/asne/omega", self.omega_callback, 10)
        self.velo_sub = self.create_subscription(Float64, "/asne/velocity/final", self.velocity_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_cb)

        self.pos_pub = self.create_publisher(Float64, "/wamv/thrusters/middle/pos", 10)
        self.thrust_pub = self.create_publisher(Float64, "/wamv/thrusters/middle/thrust", 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.omega: float = 0.0
        self.velocity: float = 0.0
        

    def omega_callback(self, msg: Float64):
        self.omega = msg.data

    def velocity_callback(self, msg: Float64):
        self.velocity = msg.data

    def timer_cb(self):
        pos_msg = Float64()
        pos_msg.data = self.omega

        thrust_msg = Float64()
        thrust_msg.data = self.velocity

        self.pos_pub.publish(pos_msg)
        self.thrust_pub.publish(thrust_msg)
        self.publish_cmd_vel((self.velocity, 0.0, 0.0), (0.0, 0.0, self.omega))
    
    def publish_cmd_vel(self, linear: tuple[float, float, float], angular: tuple[float, float, float]):
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = "wamv/base_link"
        cmd_vel.header.stamp = self.get_clock().now().to_msg()

        cmd_vel.twist.linear.x = linear[0]
        cmd_vel.twist.linear.y = linear[1]
        cmd_vel.twist.linear.z = linear[2]

        cmd_vel.twist.angular.x = angular[0]
        cmd_vel.twist.angular.y = angular[1]
        cmd_vel.twist.angular.z = angular[2]

        self.cmd_vel_pub.publish(cmd_vel)

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
