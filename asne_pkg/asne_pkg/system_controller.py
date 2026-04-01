#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asne_interfaces.msg import State, Estop
from asne_interfaces.srv import SetState, SetEstop
from std_srvs.srv import Trigger


class SystemControllerNode(Node):
    def __init__(self):
        super().__init__("system_controller_node")
        self.current_state: int = State.STATIONARY

        self.gps_estop_enabled: bool = False
        self.rc_estop_enabled: bool = False
        self.manual_estop_enabled: bool = False

        self.gps_init: bool = False

        self.state_pub = self.create_publisher(State, "/asne/state", 10)
        self.set_state_service = self.create_service(SetState, "/asne/set_state", self.set_state)

        self.estop_service = self.create_service(SetEstop, "/asne/estop", self.estop_service_callback)
        self.gps_init_servive = self.create_service(Trigger, "/gps/initialized", self.gps_init_callback)

        self.state_pub_timer = self.create_timer(0.5, self.state_pub_timer)  # type: ignore

    def is_estop_enabled(self) -> bool:
        return self.gps_estop_enabled or self.rc_estop_enabled or self.manual_estop_enabled

    def gps_init_callback(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.gps_init = True
        return response

    # NOTE: only send service when trying to change state
    def set_state(self, request: SetState.Request, response: SetState.Response):
        if self.is_estop_enabled():
            self.get_logger().warn("Ensure following Estops are disabled before switching states: ")
            if self.gps_estop_enabled:
                self.get_logger().warn("    GPS Estop")
            if self.rc_estop_enabled:
                self.get_logger().warn("    RC Estop")
            if self.manual_estop_enabled:
                self.get_logger().warn("    Manual Estop")

            response.success = False
            return response

        next_state: State = request.state
        # self.get_logger().info(f"r: {next_state}")

        match next_state.state:
            case State.MANUAL:
                self.get_logger().info("Entering manual mode")
                self.current_state = State.MANUAL
            case State.AUTONOMOUS:
                if not self.gps_init:
                    self.get_logger().info("No GPS fix, cannot enter autonomous mode")
                else:
                    self.get_logger().info("Entering autonomous mode")
                    self.current_state = State.AUTONOMOUS
                # send reset signal to waypoint generator?
            case State.STATIONARY:
                self.get_logger().info("Entering stationary mode")
                self.current_state = State.STATIONARY
            case _:
                self.get_logger().error("Atemmpting to set to unknown state")

        response.success = True
        return response

    def estop_service_callback(self, request: SetEstop.Request, response: SetEstop.Response) -> SetEstop.Response:
        response.success = True

        match request.type.type:
            case Estop.GPS_ESTOP:
                if request.enable and not self.gps_estop_enabled:
                    self.gps_estop_enabled = True
                    self.get_logger().warn("Enabled GPS Estop")
                elif request.enable and self.gps_estop_enabled:
                    self.get_logger().error("GPS Estop already enabled")
                elif not request.enable and self.gps_estop_enabled:
                    self.gps_estop_enabled = False
                    self.get_logger().info("Disabling GPS Estop")
                elif not request.enable and not self.gps_estop_enabled:
                    self.get_logger().error("GPS Estop already disabled")
                else:
                    self.get_logger().error("Failed to process GPS estop service request")
                    response.success = False

            case Estop.RC_ESTOP:
                if request.enable and not self.rc_estop_enabled:
                    self.rc_estop_enabled = True
                    self.get_logger().warn("Enabled RC Estop")
                elif request.enable and self.rc_estop_enabled:
                    self.get_logger().error("RC Estop already enabled")
                elif not request.enable and self.rc_estop_enabled:
                    self.rc_estop_enabled = False
                    self.get_logger().info("Disabling RC Estop")
                elif not request.enable and not self.rc_estop_enabled:
                    self.get_logger().error("RC Estop already disabled")
                else:
                    self.get_logger().error("Failed to process RC estop service request")
                    response.success = False

            case Estop.MANUAL_ESTOP:
                if request.enable and not self.manual_estop_enabled:
                    self.manual_estop_enabled = True
                    self.get_logger().warn("Enabled manual Estop")
                elif request.enable and self.manual_estop_enabled:
                    self.get_logger().error("manual Estop already enabled")
                elif not request.enable and self.manual_estop_enabled:
                    self.manual_estop_enabled = False
                    self.get_logger().info("Disabling manual Estop")
                elif not request.enable and not self.manual_estop_enabled:
                    self.get_logger().error("manual Estop already disabled")
                else:
                    self.get_logger().error("Failed to process manual estop service request")
                    response.success = False
            case _:
                self.get_logger().error("Atemmpting to modify unknown estop state")
                response.success = False

        return response

    def state_pub_timer(self):
        msg = State()
        msg.state = self.current_state
        self.state_pub.publish(msg)
        self.log()

    def log(self):
        if self.rc_estop_enabled:
            self.get_logger().warn("RC estop enabled")

        if self.gps_estop_enabled:
            self.get_logger().warn("gps estop enabled")

        if self.manual_estop_enabled:
            self.get_logger().warn("manual estop enabled")

        match self.current_state:
            case State.MANUAL:
                self.get_logger().info("Current State: Manual")
            case State.AUTONOMOUS:
                self.get_logger().info("Current State: Autonomous")
            case State.STATIONARY:
                self.get_logger().info("Current State: Stationary")
            case _:
                self.get_logger().warn("Set to unknown state")


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    system_controller_node = SystemControllerNode()
    try:
        rclpy.spin(system_controller_node)
    finally:
        system_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
