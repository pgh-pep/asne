#!/usr/bin/env python3

from typing import Dict
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Bool, String
from asne_interfaces.msg import RCcontroller


BAUD_RATE = 115200


class Channels(Enum):
    Joy_R_LR = 0
    Joy_R_UD = 1
    Joy_L_UD = 2
    Joy_L_LR = 3
    SW_B = 4
    Vr_B = 5
    SW_A = 6
    Vr_A = 7
    CH9 = 8
    CH10 = 9
    CH11 = 10
    CH12 = 11
    CH13 = 12
    CH14 = 13
    CH15 = 14
    CH16 = 15
    FAIL_SAFE = 16
    FRAME_LOST = 17


class RCDecoderNode(Node):
    def __init__(self):
        super().__init__("rc_decoder_node")

        self.rc_raw_sub = self.create_subscription(String, "/asne/rc/raw_string", self.rc_raw_callback, 10)

        self.rc_msg_pub = self.create_publisher(RCcontroller, "/asne/rc/channels", 10)
        self.rc_fail_safe_pub = self.create_publisher(Bool, "/rc/fail_safe", 10)

    def rc_raw_callback(self, msg: String):
        packet = msg.decode("utf-8", errors="ignore").strip()
        channels = self.parse_packet(packet)
        if channels is None:
            return

        self.pub_rc_msg(channels)
        self.pub_fail_safe_msg(channels)

    def parse_packet(self, packet: str) -> Dict[Channels, float] | None:
        # expected packet: "CH1, CH2 ... CH16, failsafe, frame_lost"
        try:
            parsed_pkt = packet.strip().split(",")
            if len(parsed_pkt) != 18:
                self.get_logger().warn(f"SBUS packet has incorrect length: {len(parsed_pkt)}")
                return None

            temp_channels: Dict[Channels, float] = dict()
            for i, channel in enumerate(Channels):
                temp_channels[channel] = float(parsed_pkt[i].strip())

            return temp_channels

        except (ValueError, IndexError) as e:
            print(f"SBUS parse error: {e}")
            return None

    def pub_rc_msg(self, channels: Dict[Channels, float]):
        msg = RCcontroller()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joy_r_lr = channels[Channels.Joy_R_LR]
        msg.joy_r_ud = channels[Channels.Joy_R_UD]
        msg.joy_l_ud = channels[Channels.Joy_L_UD]
        msg.joy_l_lr = channels[Channels.Joy_L_LR]
        msg.sw_b = channels[Channels.SW_B]
        msg.vr_b = channels[Channels.Vr_B]
        msg.sw_a = channels[Channels.SW_A]
        msg.vr_a = channels[Channels.Vr_A]

        self.rc_msg_pub.publish(msg)

    def pub_fail_safe_msg(self, channels: Dict[Channels, float]):
        msg = Bool()
        msg.data = bool(int(channels[Channels.FAIL_SAFE]))

        self.rc_fail_safe_pub.publish(msg)


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    rc_decoder_node = RCDecoderNode()
    try:
        rclpy.spin(rc_decoder_node)
    finally:
        rc_decoder_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
