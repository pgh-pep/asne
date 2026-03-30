#!/usr/bin/env python3
# pyright: reportAttributeAccessIssue=false, reportIndexIssue=false

import canopen
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

BAUD_RATE = 115200

# TODO: eventually add a P controller to thrust commands to not accelerate as fast


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        self.declare_parameter("dcf_path", "/home/varun/pep/asne_ws/src/asne/asne_pkg/config/sevcon_working.dcf")
        dcf_path: str = self.get_parameter("dcf_path").get_parameter_value().string_value

        # self.declare_parameter("baud", 115200)
        # self.baud: str = self.get_parameter("dcf_path").value
        # self.network.connect(interface="ixxat", channel=0, bitrate=250000)

        self.network = canopen.Network()
        self.network.connect(channel="can0", bustype="socketcan")
        self.node = self.network.add_node(1, dcf_path)

        self.goal_velocity: float = 0.0
        self.max_velocity: float = 50
        self.deadband: float = 0.05

        self.is_forward = True

        self.velocity_sub = self.create_subscription(Float64, "/asne/velocity/manual", self.velocity_callback, 10)

        self.thrust_cmd_timer = self.create_timer(1, self.thrust_timer_callback)

        self.init_motor()

    def thrust_timer_callback(self):
        # motor_temp = self.node.sdo[0x4600][3].raw
        # self.get_logger().info(f"temp: {motor_temp}Â°C")

        target_torque: float = max(min(self.goal_velocity, self.max_velocity), -self.max_velocity)
        if abs(target_torque) < self.deadband:
            target_torque = 0.0

        # SEVCONFIELD SCALING=0.1
        # SEVCONFIELD UNITS=% of peak
        # ex 1000 = 100% rated torque.
        # ex. 50 -> 5 % of peak torque

        # if forward mode but want to go reverse
        self.get_logger().info(f"torque: {target_torque}")

        if target_torque < 0 and self.is_forward:
            self.set_direction(False)
        # check if reverse mode but want to go forward
        elif target_torque > 0 and not self.is_forward:
            self.set_direction(True)

        self.node.sdo[0x6071].raw = abs(target_torque)

    def set_direction(self, forward: bool):
        self.node.sdo[0x6071].raw = 0  # starting torque = 0

        direction_hex = 0x0400 if forward else 0x0200

        self.node.sdo["controlword"].raw = 0x0006  # Shutdown
        time.sleep(0.1)
        self.node.sdo["controlword"].raw = direction_hex | 0x0007  # Switched On
        time.sleep(0.1)
        self.node.sdo["controlword"].raw = direction_hex | 0x000F  # Enable
        self.get_logger().info(f"set direction to: {'forward' if forward else 'reverse'}")

        self.is_forward = forward

    def init_motor(self):
        self.get_logger().info("BEGINNING SEVCON INITALIZATION!")

        self.node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.2)

        try:
            self.node.sdo[0x6071].raw = 0  # starting torque = 0
        except canopen.SdoAbortedError as e:
            self.get_logger().warn(f"failed to zero torque: {e}")

        self.node.nmt.state = "OPERATIONAL"
        time.sleep(0.5)

        try:
            device_state = int(self.node.sdo["NMT State"].raw)
            self.get_logger().info(f"device state should be operational (0x5110): {hex(device_state)}")
        except Exception as e:
            self.get_logger().warn(f"Could not read 0x5110: {e}")

        # precharge capacistors:
        self.node.sdo[0x5180].raw = 0x0001

        # controlword = 0x6040
        # self.get_logger().info("activation cycle:")
        # self.node.sdo["controlword"].raw = 0x0006  # Shutdown
        # time.sleep(0.1)
        # self.node.sdo["controlword"].raw = 0x0407  # Switched On, forward
        # # NOTE( 0x02xx is reverse)
        # time.sleep(0.1)
        # self.node.sdo["controlword"].raw = 0x040F  # enable forward
        # self.get_logger().info("motor set")
        self.set_direction(True)

        status = int(self.node.sdo[0x6041].raw)
        self.get_logger().info(f"final init status: {hex(status)}")

        try:
            num_faults = int(self.node.sdo[0x5300][1].raw)
            if num_faults > 0:
                self.get_logger().info(f"active fault count (0x5300:01): {num_faults}")
                self.node.sdo[0x5300][2].raw = 0
                active_fault_id = int(self.node.sdo[0x5300][3].raw)
                self.get_logger().info(f"highest priority fault ID: {hex(active_fault_id)}")
            else:
                self.get_logger().info("No active faults detected.")

        except Exception as e:
            self.get_logger().warn(f"ERROR reading faults: {e}")

    def velocity_callback(self, msg: Float64):
        self.goal_velocity = msg.data

    def destroy_node(self):
        self.get_logger().info("\nSHUTTING DOWN")
        self.node.sdo[0x6040].raw = 0x0000  # disable voltage
        self.node.nmt.state = "PRE-OPERATIONAL"
        try:
            device_state = int(self.node.sdo["NMT State"].raw)
            self.get_logger().info(f"device state should be preop(0x5110): {hex(device_state)}")
        except Exception as e:
            self.get_logger().warn(f"ERROR CANT READ STATE (0x5110): {e}")

        super().destroy_node()


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    try:
        rclpy.spin(motor_control_node)
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
