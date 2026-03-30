#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64, String

BAUD_RATE = 115200


class SerialCommNode(Node):
    def __init__(self):
        super().__init__("serial_comm_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.serial_port: str = self.get_parameter("serial_port").get_parameter_value().string_value

        self.declare_parameter("baud", 115200)
        self.baud: int = self.get_parameter("baud").get_parameter_value().integer_value

        self.serial_ESP = serial.Serial(self.serial_port, self.baud, timeout=0.1)
        time.sleep(2.5)

        self.get_logger().info(f"opened serial comms w/ ESP on {self.serial_port} at {self.baud}")

        self.servo_sub = self.create_subscription(Float64, "asne/servo_angle", self.servo_callback, 10)
        self.rc_raw_pub = self.create_publisher(String, "asne/rc/raw_string", 10)

        self.read_timer = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        if self.serial_ESP.in_waiting == 0:
            return

        raw = self.serial_ESP.readline()
        if not raw:
            return

        msg: String = String()
        msg.data = str(raw)[2:][:-5]
        # self.get_logger().info(msg.data)
        self.rc_raw_pub.publish(msg)

    def servo_callback(self, msg: Float64):
        line = f"SA:{msg}"
        self.serial_ESP.write(line.encode("utf-8"))

    def destroy_node(self):
        if self.serial_ESP and self.serial_ESP.is_open:
            self.serial_ESP.close()
            self.get_logger().info("closing ESP serial comms")

        super().destroy_node()


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    serial_comm_node = SerialCommNode()
    try:
        rclpy.spin(serial_comm_node)
    finally:
        serial_comm_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
