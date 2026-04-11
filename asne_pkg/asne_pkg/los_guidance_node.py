#!/usr/bin/env python3

from typing import Tuple

import rclpy
from rclpy.node import Node
from math import atan2, sin, cos, sqrt
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Time
# from rclpy.clock import Time
from asne_interfaces.srv import NextWaypoint

import rclpy.task


class LOSGuidanceNode(Node):
    def __init__(self):
        super().__init__("los_guidance_node")

        self.declare_parameter("lookahead_dist", 10.0)  # m
        self.declare_parameter("waypoint_thresh", 10.0)  # m

        self.lookahead_dist: float = self.get_parameter("lookahead_dist").get_parameter_value().double_value
        self.waypoint_thresh: float = self.get_parameter("waypoint_thresh").get_parameter_value().double_value

        # all waypoints in ENU (m) -> (east = x, north = y)
        # WP_A will be init as (0,0)
        self.x: float | None = None
        self.y: float | None = None
        self.heading: float | None = None
        self.twist: Twist | None = None
        self.odom_ts: Time | None = None

        self.prev_wp: Tuple[float, float] | None = None
        self.curr_wp: Tuple[float, float] | None = None

        self.e_int: float = 0.0
        self.prev_los_time: float | None = None

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
        )

        self.heading_pub = self.create_publisher(Float64, "/asne/heading/autonomous", 10)
        self.velocity_pub = self.create_publisher(Float64, "/asne/velocity/autonomous", 10)

        self.next_wp_client = self.create_client(NextWaypoint, "asne/next_waypoint")
        self.los_timer = self.create_timer(0.1, self.LOS_guidance)
        # self.done_pub = self.create_publisher(Bool, "/asne/laps_complete", 10)

        self.requesting_wp: bool = False
        self.next_wp_client.wait_for_service()
        self.get_logger().info("fetching next wp:")
        self.request_next_waypoint()

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_ts = msg.header.stamp

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.twist = msg.twist.twist

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.heading = yaw

    def request_next_waypoint(self) -> None:
        self.requesting_wp = True
        future = self.next_wp_client.call_async(NextWaypoint.Request())
        future.add_done_callback(self.waypoint_callback)

    def waypoint_callback(self, future: rclpy.task.Future) -> None:
        self.get_logger().info("waypoint callback")
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn("next waypoint request failed")
            self.requesting_wp = False
            return

        wp: Point = result.waypoint
        if self.curr_wp is None:
            # for the first waypoint, use current pose
            self.prev_wp = (self.x, self.y) if (self.x is not None and self.y is not None) else (wp.x, wp.y)
        else:
            self.prev_wp = self.curr_wp

        self.curr_wp = (wp.x, wp.y)
        self.e_int = 0.0  # reset integral on new segment
        self.requesting_wp = False
        self.get_logger().info(f"Next waypoint recieved: ({wp.x:.2f}, {wp.y:.2f})")

    def LOS_guidance(self):
        # TODO: MIGHT NEED DEFAULT TO DISABLE THIS IF NOT IN AUTONOMOUS

        # dont know boat pose:
        if self.x is None or self.y is None:
            self.get_logger().warn("boat location unknown, switching back to manual mode")
            # send service to switch to manual mode
            return

        # boat pose expired:
        if self.odom_ts is None:
            return

        dt = (self.get_clock().now().to_msg().nanosec - self.odom_ts.nanosec) * 1e-9
        odom_expiration_thresh = 1.0  # s

        if dt > odom_expiration_thresh:
            self.get_logger().warn("boat location expired, switching back to manual mode")
            # send service to switch to manual mode
            return

        if self.curr_wp is None or self.prev_wp is None:
            self.get_logger().warn("No waypoint received yet", throttle_duration_sec=2.0)
            return

        x, y = self.x, self.y
        prev_wp = self.prev_wp
        curr_wp = self.curr_wp

        # case where we dont know boat pose (should never happen)
        # but point to target just in case (TODO: determine if this is the play)
        if prev_wp == curr_wp:
            psi_d = atan2(curr_wp[1] - y, curr_wp[0] - x)
            self.heading_pub.publish(Float64(data=psi_d))
            # self.velocity_pub.publish(Float64(data=50.0))
            return

        # path angle
        alpha = atan2(curr_wp[1] - prev_wp[1], curr_wp[0] - prev_wp[0])

        # cross track error (XTE) -> positive XTE = boat on right of path
        xte = (x - prev_wp[0]) * sin(alpha) - (y - prev_wp[1]) * cos(alpha)

        # TODO: test if integral term needed
        ki = 0
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_los_time is not None:
            dt = now - self.prev_los_time
            self.e_int += xte * dt

        self.prev_los_time = now

        # goal heading:
        psi_d = alpha - atan2(xte + ki * self.e_int, self.lookahead_dist)
        psi_d = atan2(sin(psi_d), cos(psi_d))  # norm. to (-pi, pi]

        # along track error (waypoint switching)
        ate = (x - prev_wp[0]) * cos(alpha) + (y - prev_wp[1]) * sin(alpha)
        seg_len = sqrt((curr_wp[0] - prev_wp[0]) ** 2 + (curr_wp[1] - prev_wp[1]) ** 2)

        # determine if switch to next waypoint
        if ate >= seg_len - self.waypoint_thresh and not self.requesting_wp:
            self.get_logger().info("reached current point")
            self.request_next_waypoint()

        self.heading_pub.publish(Float64(data=psi_d))
        # self.velocity_pub.publish(Float64(data=10.0))


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    los_guidance_node = LOSGuidanceNode()
    try:
        rclpy.spin(los_guidance_node)
    finally:
        los_guidance_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
