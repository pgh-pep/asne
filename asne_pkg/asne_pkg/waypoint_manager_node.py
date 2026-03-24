#!/usr/bin/env python3

from typing import Tuple

import rclpy
import numpy as np
import matplotlib.pyplot as plt 
import math
from rclpy.node import Node
from math import atan2, pi, sin, cos, sqrt
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from asne_interfaces.srv import NextWaypoint


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__("waypoint_manager_node")

        self.declare_parameter("laps", 2)
        self.declare_parameter("waypoint_thresh", 10.0)  # m

        self.declare_parameter("WP_A_lat", 36.8340)
        self.declare_parameter("WP_A_lon", -76.3700)
        self.declare_parameter("WP_B_lat", 36.8290)
        self.declare_parameter("WP_B_lon", -76.3600)

        self.declare_parameter("lane_width", 15.0)  # m
        self.declare_parameter("turn_rad", 10.0)  # m
        self.declare_parameter("turn_points", 6)

        self.waypoint_thresh: float = self.get_parameter("waypoint_thresh").value
        self.n_laps: int = self.get_parameter("laps").value

        self.WP_A_lat: float = self.get_parameter("WP_A_lat").value
        self.WP_A_lon: float = self.get_parameter("WP_A_lon").value
        self.WP_B_lat: float = self.get_parameter("WP_B_lat").value
        self.WP_B_lon: float = self.get_parameter("WP_B_lon").value

        self.origin = (self.WP_A_lat, self.WP_A_lon)

        self.lane_width: float = self.get_parameter("lane_width").value
        self.turn_rad: float = self.get_parameter("turn_rad").value
        self.turn_points: float = self.get_parameter("turn_points").value

        # all waypoints in ENU (m) -> (east = x, north = y)
        # (0,0) will b einti as WP_A
        self.path: Path = self.generate_path()
        self.current_path_idx: int = 0
        self.current_lap: int = 0

        self.waypoint_pub = self.create_publisher(Point, "asne/desired_waypoint", 10)

        self.next_waypoint_srv = self.create_service(NextWaypoint, "asne/next_waypoint", self.get_next_waypoint)

        self.reset_path_srv = self.create_service(Trigger, "asne/reset_path", self.reset_path)

        # path = self.generate_path()
        # plt.scatter([d.pose.position.x for d in list(path.poses)], [d.pose.position.y for d in list(path.poses)])
        # ax = plt.gca()
        # ax.set_aspect('equal')
        # ax.grid(True)
        # plt.show()

        # UNUSED: client to signal race completed
        self.race_over_client = self.create_client(Trigger, "asne/race_completed")

        self.waypoint_timer = self.create_timer(0.01, self.waypoint_timer_callback)

    def generate_path(self) -> Path:
        path: Path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"  # check

        B = np.array(self.gps_to_enu(self.WP_B_lat, self.WP_B_lon))
        r = self.turn_rad # distance between waypoints and foci in meters
        theta = atan2(B[1], B[0])

        base_angles = (math.pi/(self.turn_points-1)) * np.arange(0, self.turn_points)
        B_angles = base_angles + (theta - math.pi/2)
        B_vectors = B + r * np.array([[cos(d), sin(d)] for d in B_angles])
        A_angles = -base_angles + (theta - math.pi/2)
        A_vectors = r * np.array([[cos(d), sin(d)] for d in A_angles])
        vectors = np.concatenate((B_vectors, A_vectors), axis=0)

        for i in range(self.turn_points * 2):
            output_pose = PoseStamped()
            output_pose.pose.position.x = vectors[i][0]
            output_pose.pose.position.y = vectors[i][1]
            path.poses.append(output_pose)
    
        return path

    def waypoint_timer_callback(self):
        pose: Pose = self.path.poses[self.current_path_idx].pose

        if pose is None:
            self.get_logger().warn("Desired wp is None")
            return

        self.waypoint_pub.publish(pose.position)

    def get_next_waypoint(self, request: NextWaypoint.Request, response: NextWaypoint.Response):
        self.current_path_idx += 1

        if self.current_path_idx >= len(self.path.poses):
            self.current_path_idx = 0
            self.current_lap += 1

            if self.current_lap >= self.n_laps:
                self.race_over_client.call_async(Trigger.Request())

        pose: PoseStamped = self.path.poses[self.current_path_idx]

        response.waypoint.x = pose.pose.position.x
        response.waypoint.y = pose.pose.position.y
        response.success = True

        return response

    def reset_path(self, request: Trigger.Request, response: Trigger.Response):
        self.current_path_idx = 0
        response.success = True
        return response

    def reset_path_count(self, request: Trigger.Request, response: Trigger.Response):
        self.current_lap = 0
        response.success = True
        return response

    def gps_to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        lat0, lon0 = self.origin
        R = 6_371_000.0  # earth radius (m)
        x = (lon - lon0) * (pi / 180.0) * R * cos(lat0 * pi / 180.0)
        y = (lat - lat0) * (pi / 180.0) * R
        return (x, y)


def main(args=None):
    rclpy.init(args=args)
    waypoint_manager_node = WaypointManagerNode()
    try:
        rclpy.spin(waypoint_manager_node)
    finally:
        waypoint_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
