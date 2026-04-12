#!/usr/bin/env python3

from typing import Dict
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String
from asne_interfaces.msg import State, Estop, RCcontroller
from asne_interfaces.srv import SetState, SetEstop
from rclpy.time import Duration, Time


BAUD_RATE = 115200
TIMEOUT = 3  # s


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


# TODO: three pronged switch distinguishes between manual, auto, and stationary
# two pronged switch is rc estop vs not rc estop


class RCDecoderNode(Node):
    def __init__(self):
        super().__init__("rc_decoder_node")

        # handles failsafe bit
        self.failsafe_since: float | None = None
        self.failsafe_active: bool = False

        # handles disconnect from ESP
        self.last_rc_stamp: Time | None = None

        self.rc_wd_estop_enabled: bool = False
        self.manual_estop_enabled: bool = False

        # switch A has two states -> ESTOP (bottom = -1 = off, top = 1 = on)
        # switch B has three states -> auto (1, top) vs manual (0) vs stationary (-1, bottom)
        self.prev_sw_a: int = -1
        self.prev_sw_b: int = -1

        self.rc_raw_sub = self.create_subscription(String, "/asne/rc/raw_string", self.rc_raw_callback, 10)
        self.rc_msg_pub = self.create_publisher(RCcontroller, "/asne/rc/channels", 10)

        self.set_state_client = self.create_client(SetState, "/asne/set_state")
        self.estop_client = self.create_client(SetEstop, "/asne/set_estop")

        self.rc_watchdog = self.create_timer(0.1, self.rc_watchdog_callback)

    def rc_watchdog_callback(self):
        if self.last_rc_stamp is None:
            return

        now = self.get_clock().now()
        rc_msg_expired = (now - self.last_rc_stamp) > Duration(seconds=TIMEOUT)

        if not self.rc_wd_estop_enabled and (self.failsafe_active or rc_msg_expired):
            self.get_logger().warn("Activating RC failsafe")
            self.rc_wd_estop_enabled = True
            if self.estop_client.service_is_ready():
                req = SetEstop.Request()
                req.type = Estop(type=Estop.RC_ESTOP)
                req.enable = True
                self.estop_client.call_async(req)
            else:
                self.get_logger().error("FAILED to activate RC failsafe")

            return

        # else, disable estop
        if self.rc_wd_estop_enabled and not rc_msg_expired and not self.failsafe_active:
            self.get_logger().info("Disabling RC failsafe")
            self.rc_wd_estop_enabled = False
            if self.estop_client.service_is_ready():
                req = SetEstop.Request()
                req.type = Estop(type=Estop.RC_ESTOP)
                req.enable = False
                self.estop_client.call_async(req)
            else:
                self.get_logger().error("FAILED to disable RC failsafe")

    def rc_raw_callback(self, msg: String):
        self.last_rc_stamp = self.get_clock().now()

        packet = msg.data.strip()
        channels = self.parse_packet(packet)
        if channels is None:
            return

        fail_safe_bit = bool(int(channels[Channels.FAIL_SAFE]))

        # fail_safe bit = any instance where rc controller disconnects:
        if fail_safe_bit:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.failsafe_since is None:
                self.failsafe_since = now
            elif not self.failsafe_active and (now - self.failsafe_since) >= TIMEOUT:
                self.failsafe_active = True

        else:
            self.failsafe_since = None
            if self.failsafe_active:
                self.failsafe_active = False

        self.handle_switches(channels)
        self.pub_rc_msg(channels)

    def parse_packet(self, packet: str) -> Dict[Channels, float] | None:
        # expected packet: "CH1, CH2 ... CH16, failsafe, frame_lost"
        try:
            parsed_pkt = packet.strip().split(",")
            if len(parsed_pkt) != 18:
                self.get_logger().warn(f"SBUS packet has incorrect length: {len(parsed_pkt)}")
                return None

            temp_channels: Dict[Channels, float] = {}
            for i, channel in enumerate(Channels):
                temp_channels[channel] = float(parsed_pkt[i].strip())

            return temp_channels

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"SBUS parse error: {e}")
            return None

    def handle_switches(self, channels: Dict[Channels, float]):
        sw_a = int(channels[Channels.SW_A])
        sw_b = int(channels[Channels.SW_B])
        # switch A has two states -> ESTOP (bottom = -1 = off, top = 1 = on)
        # switch B has three states -> auto (1, top) vs manual (0) vs stationary (-1, bottom)

        if sw_a != self.prev_sw_a:
            self.prev_sw_a = sw_a
            req = SetState.Request()
            match sw_a:
                case -1:
                    self.get_logger().warn("Disabling manual failsafe")
                    self.manual_estop_enabled = False
                    if self.estop_client.service_is_ready():
                        req = SetEstop.Request()
                        req.type = Estop(type=Estop.MANUAL_ESTOP)
                        req.enable = False
                        self.estop_client.call_async(req)
                    else:
                        self.get_logger().error("FAILED to disable manual failsafe")
                case _:
                    self.get_logger().info("Enabling manual failsafe")
                    self.manual_estop_enabled = True
                    if self.estop_client.service_is_ready():
                        req = SetEstop.Request()
                        req.type = Estop(type=Estop.MANUAL_ESTOP)
                        req.enable = True
                        self.estop_client.call_async(req)
                    else:
                        self.get_logger().error("FAILED to enable manual failsafe")

        if sw_b != self.prev_sw_b:
            self.prev_sw_b = sw_b

            # # dont change mode if estop is active
            # if sw_a != -1:
            #     return

            req = SetState.Request()
            match sw_b:
                case -1.0:
                    req.state = State(state=State.STATIONARY)
                case 0.0:
                    req.state = State(state=State.MANUAL)
                case 1.0:
                    req.state = State(state=State.AUTONOMOUS)
                case _:
                    req.state = State(state=State.STATIONARY)

            if self.set_state_client.service_is_ready():
                self.set_state_client.call_async(req)
            else:
                self.get_logger().error("FAILED to manually swtich state")

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
