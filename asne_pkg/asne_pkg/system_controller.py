#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asne_interfaces.msg import State
from asne_interfaces.srv import SetState
from std_srvs.srv import Trigger


# TODO: Need ESTOP to be a list of faults rather than a state
class SystemController(Node):
    def __init__(self):
        super().__init__("system_controller")
        self.current_state: State = State.STATIONARY

        self.state_pub = self.create_publisher(State, "/asne/state", 10)
        self.set_state_service = self.create_service(SetState, "/asne/set_state", self.set_state)
        self.reset_estop_service = self.create_service(SetState, "/asne/reset_estop", self.reset_estop_callback)

    # only send service when trying to change state
    def set_state(self, request: SetState.Request, response: SetState.Response):
        if self.current_state == State.RC_ESTOP or self.current_state == State.GPS_ESTOP:
            self.get_logger().warn("Deactivate ESTOP before attempting to change state")
            response.success = False
            return response

        next_state: State = request.state
        match next_state:
            case State.MANUAL:
                self.get_logger().info("Entering manual mode")
            case State.AUTONOMOUS:
                self.get_logger().info("Entering autonomous mode")
                # send reset signal to waypoint generator
            case State.STATIONARY:
                self.get_logger().info("Entering stationary mode")
            case State.RC_ESTOP:
                self.get_logger().warn("Activating RC ESTOP")
            case State.GPS_ESTOP:
                self.get_logger().warn("Activating GPS ESTOP")
            case _:
                self.get_logger().info("Set to unknown state")

        response.success = True
        return response

    def reset_estop_callback(self, request: Trigger.Request, response: Trigger.Response):
        if self.current_state not in (State.RC_ESTOP, State.GPS_ESTOP):
            self.current_state = State.STATIONARY
            response.success = True
        else:
            response.success = False

        return response

    def timer_callback(self):
        msg = State()
        msg.state = self.current_state
        self.state_pub(msg)


def main(args=None):
    rclpy.init(args=args)
    system_controller = SystemController()
    try:
        rclpy.spin(system_controller)
    finally:
        system_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
