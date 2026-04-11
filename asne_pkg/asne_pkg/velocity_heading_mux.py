#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from asne_interfaces.msg import State

class VelocityHeadingMux(Node):
    def __init__(self):
        super().__init__("velocity_heading_mux")

        self.manual_velocity_sub = self.create_subscription(Float64, "/asne/velocity/manual", self.manual_vel_callback, 10)
        self.auto_velocity_sub = self.create_subscription(Float64, "/asne/velocity/autonomous", self.auto_vel_callback, 10)
        self.manual_heading_sub = self.create_subscription(Float64, "/asne/heading/manual", self.manual_head_callback, 10)
        self.auto_heading_sub = self.create_subscription(Float64, "/asne/heading/autonomous", self.auto_head_callback, 10)
        self.state_sub = self.create_subscription(State, "/asne/state", self.state_callback, 10)

        self.velocity_pub = self.create_publisher(Float64, "/asne/velocity/final", 10)
        self.heading_pub = self.create_publisher(Float64, "/asne/heading/final", 10)
    
        self.manual_velocity: float = 0.0
        self.auto_velocity: float = 0.0
        self.manual_heading: float = 0.0
        self.auto_heading: float = 0.0
        self.system_state: State = State.AUTONOMOUS

        self.update_loop = self.create_timer(0.1, self.update_velocity)

    def manual_vel_callback(self, msg: Float64):
        self.manual_velocity = msg.data

    def auto_vel_callback(self, msg: Float64):
        self.auto_velocity = msg.data

    def manual_head_callback(self, msg: Float64):
        self.manual_heading = msg.data

    def auto_head_callback(self, msg: Float64):
        self.auto_heading = msg.data

    def state_callback(self, msg: State):
        self.system_state = msg
        
    def update_velocity(self):
        if (self.system_state == State.AUTONOMOUS):
            self.velocity_pub.publish(Float64(data=self.auto_velocity))
            self.heading_pub.publish(Float64(data=self.auto_heading))
        elif (self.system_state == State.MANUAL):
            self.velocity_pub.publish(Float64(data=self.manual_velocity))
            self.heading_pub.publish(Float64(data=self.manual_heading))


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    velocity_heading_mux = VelocityHeadingMux()
    try:
        rclpy.spin(velocity_heading_mux)
    finally:
        velocity_heading_mux.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()