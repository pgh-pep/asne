#!/usr/bin/env python3

from typing import Tuple

import rclpy
from rclpy.node import Node
from math import atan2, pi, sin, cos, sqrt
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


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
        self.declare_parameter("turn_points", 4)

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
        self.desired_wp: Point = None
        self.current_path_idx: int = 0
        self.current_lap: int = 0

        # next waypoint publisher
        desired_waypoint_topic: str = "asne/desired_waypoint"
        self.waypoint_pub = self.create_publisher(Point, desired_waypoint_topic, 10)

        # service to advance to next waypoint
        self.next_waypoint_srv = self.create_service(Trigger, "asne/next_waypoint", self.get_next_waypoint)

        # service to reset path back to first waypoint
        self.reset_path_srv = self.create_service(Trigger, "asne/reset_path", self.reset_path)

        # client to signal race completed
        self.race_over_client = self.create_client(Trigger, "asne/race_completed")

    def generate_path(self) -> Path:
        path: Path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"  # check

        # TODO once recieve more information about the desired path

        return path

    def get_next_waypoint(self, request: Trigger.Request, response: Trigger.Response):
        self.current_path_idx += 1

        if self.current_path_idx >= len(self.path.poses):
            self.current_path_idx = 0
            self.current_lap += 1

            if self.current_lap >= self.n_laps:
                req = Trigger.Request()
                self.race_over_client.call_async(Trigger.Request())

        self.desired_wp = self.path.poses[self.current_path_idx]
        response.success = True
        return response

    def reset_path(self, request: Trigger.Request, response: Trigger.Response):
        self.current_path_idx = 0
        self.desired_wp = self.path.poses[self.current_path_idx]
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
