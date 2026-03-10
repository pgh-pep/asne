#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asne_interfaces.msg import State
from asne_interfaces.srv import SetState


class SystemController(Node):
    def __init__(self):
        super().__init__("system_controller")
        self.current_state: State = State.MANUAL

        self.state_pub = self.create_publisher(State, "/asne/state", 10)
        self.set_state_service = self.create_service(SetState, "/asne/set_state", self.set_state)

    # only send service when trying to change state
    def set_state(self, request: SetState.Request, response: SetState.Response):
        # TODO: check if in estop, if in estop ignore changes until that estop is resolves
        # we need two more services for unlatching each estop
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
