#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from hrim_actuator_gripper_srvs.srv import ControlFinger


class MinimalClient(Node):

    def __init__(self):
        super().__init__('mara_minimal_client')

        # Create a client for service "/hrim_actuation_gripper_000000000004/goal"
        self.client = self.create_client(ControlFinger, "/hrim_actuation_gripper_000000000004/goal")

        # Wait for service to be avaiable before calling it
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Create request with the same type as the service, ControlFinger
        self.req = ControlFinger.Request()

    def send_request(self):
        self.req.goal_linearposition = 0.87
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    node = MinimalClient()

    # Call service and spin
    node.send_request()
    rclpy.spin_until_future_complete(node, node.future)

    # Analyze the result
    if node.future.result() is not None:
        node.get_logger().info('Goal accepted: %d: ' % node.future.result().goal_accepted)
    else:
        node.get_logger().error('Exception while calling service: %r' % node.future.exception())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
