#!/usr/bin/env python3

import rclpy
from math import pi, atan
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from asne_pkg.pid import PID
from tf_transformations import euler_from_quaternion


class PDMotionController(Node):
    def __init__(self):
        super().__init__("pd_motion_controller")

        self.declare_parameter("heading_kp", 0.1)
        self.declare_parameter("heading_kd", 0.0)
        self.declare_parameter("velocity_kp", 0.5)
        self.declare_parameter("velocity_kd", 0.0)
        self.declare_parameter("pivot_distance", 1.0)
        # TODO: test turning_thresh effect
        self.declare_parameter("turning_thresh", 90 * pi / 180)  # radians

        self.declare_parameter("debug", False)
        self.debug: bool = self.get_parameter("debug").get_parameter_value().bool_value

        heading_kp: float = self.get_parameter("heading_kp").get_parameter_value().double_value
        heading_kd: float = self.get_parameter("heading_kd").get_parameter_value().double_value
        velocity_kp: float = self.get_parameter("velocity_kp").get_parameter_value().double_value
        velocity_kd: float = self.get_parameter("velocity_kd").get_parameter_value().double_value
        self.pivot_distance: float = self.get_parameter("pivot_distance").get_parameter_value().double_value
        self.turning_thresh: float = self.get_parameter("turning_thresh").get_parameter_value().double_value

        # PID
        self.desired_heading: float = 0.0  # rads
        self.desired_velocity: float = 0.0  # m/s
        self.actual_velocity: float = 0.0
        self.actual_heading: float = 0.0
        self.last_cycle_time = self.get_clock().now()  # s

        self.heading_pid = PID(heading_kp, 0, heading_kd)
        self.velocity_pid = PID(velocity_kp, 0, velocity_kd)

        # pub sub
        self.create_subscription(Float64, "/asne/heading/autonomous", self.heading_callback, 10)
        self.create_subscription(Float64, "/asne/velocity/autonomous", self.velocity_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        self.torque_pub = self.create_publisher(Float64, "/asne/torque/autonomous", 10)
        self.servo_pub = self.create_publisher(Float64, "/asne/servo_angle/autonomous", 10)  # radians
        self.omega_pub = self.create_publisher(Float64, "/asne/omega", 10)  # radians

        self.create_timer(0.1, self.update_controls)

    def heading_callback(self, msg: Float64):
        self.desired_heading = msg.data

    def velocity_callback(self, msg: Float64):
        self.desired_velocity = msg.data

    def odom_callback(self, msg: Odometry):
        self.actual_velocity = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.actual_heading = yaw

    def update_controls(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_cycle_time.nanoseconds) * 1e-9
        self.last_cycle_time = now

        if dt <= 0.0:
            return

        # heading pid:
        heading_error = self.desired_heading - self.actual_heading
        if heading_error > pi:
            heading_error -= 2 * pi
        elif heading_error < -pi:
            heading_error += 2 * pi

        # TODO: test slow down proporsationally on heading error:
        # for turning, we apply some factor of slowing
        # based on how large the error is (so proportional)
        turning_slow_factor = max(0.0, 1.0 - (abs(heading_error) / self.turning_thresh))
        adjusted_velocity = self.desired_velocity * turning_slow_factor

        # velocity pid
        velocity_error = adjusted_velocity - self.actual_velocity
        torque = self.velocity_pid.update(velocity_error, dt)

        # calc servo angle
        omega = self.heading_pid.update(heading_error, dt)
        servo_angle = atan((self.pivot_distance * omega) / self.actual_velocity) if self.actual_velocity != 0.0 else 0.0

        self.torque_pub.publish(Float64(data=float(torque)))
        self.servo_pub.publish(Float64(data=float(servo_angle)))
        self.omega_pub.publish(Float64(data=float(omega)))

        if self.debug:
            self.get_logger().info(f"Heading Error:        {heading_error:.3f} rad")
            self.get_logger().info(f"Turning Slow Factor:  {turning_slow_factor:.2f}")
            self.get_logger().info(f"Adjusted Velocity:    {adjusted_velocity:.3f} m/s")
            self.get_logger().info(f"Velocity Error:       {velocity_error:.3f} m/s")
            self.get_logger().info(f"Output Torque:        {torque:.3f}")
            self.get_logger().info(f"Omega:                {omega:.3f} rad/s")
            self.get_logger().info(f"Servo Angle:          {servo_angle:.3f} rad")


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    pd_motion_controller = PDMotionController()
    try:
        rclpy.spin(pd_motion_controller)
    finally:
        pd_motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
