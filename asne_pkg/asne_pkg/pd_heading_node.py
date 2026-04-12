#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from asne_pkg.pid import PID
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.time import Time, Duration
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class PDHeadingNode(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")

        self.create_subscription(Float64, "/asne/heading/final", self.heading_callback, 10)
        self.create_subscription(Float64, "/asne/velocity/final", self.velocity_callback, 10)
        
        self.omega_pub = self.create_publisher(Float64, "/asne/omega", 10)
        self.servo_pub = self.create_publisher(Float64, "/asne/desired_servo", 10)

        self.heading_pid = PID(0.1, 0, 0)
        self.desired_heading: float = 0.0
        self.linear_velocity: float = 0.0
        self.robot_pose: Pose
        self.robot_heading: float = 0.0
        self.pivot_distance: float = 1.0
        self.last_cycle_time = self.get_clock().now()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.update_controls)
    
    def heading_callback(self, msg: Float64):
        self.desired_heading = msg.data
    
    def velocity_callback(self, msg: Float64):
        self.linear_velocity = msg.data

    def get_robot_pose(self, target_frame: str = "map", source_frame: str = "wamv/base_link") -> PoseStamped:
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=1))

            pose = PoseStamped()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation

            pose.header.frame_id = target_frame

            return pose
        except Exception as ex:
            self.get_logger().error(f"Cannot transform from {source_frame} to {target_frame}: {ex}")
            return PoseStamped()

    def get_robot_heading(self, robot_pose: Pose) -> float:
        q = [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]

        _, _, yaw = euler_from_quaternion(q)
        return yaw
    
    def update_controls(self):
        dt = (self.get_clock().now().nanoseconds - self.last_cycle_time.nanoseconds) * 1e-9

        self.robot_pose = self.get_robot_pose().pose
        
        try:
            self.robot_heading = self.get_robot_heading(self.robot_pose)
            omega = self.heading_pid.update(self.desired_heading - self.robot_heading, dt)

            servo_angle = (self.pivot_distance * omega) / self.linear_velocity if self.linear_velocity != 0.0 else 0.0

            self.omega_pub.publish(Float64(data=omega)) # sim
            self.servo_pub.publish(Float64(data=servo_angle)) # real

            self.last_cycle_time = self.get_clock().now()
        except Exception as ex:
            self.get_logger().error(ex) 
        

def main(args=None):  # type: ignore
    rclpy.init(args=args)
    pd_heading_node = PDHeadingNode()
    try:
        rclpy.spin(pd_heading_node)
    finally:
        pd_heading_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
