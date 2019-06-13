#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_services_default
from hrim_actuator_gripper_srvs.srv import ControlFinger

# -------- #

rclpy.init(args=None)

# Create Node with name "mara_minimal_client"
node = rclpy.create_node("mara_minimal_client")

# Create a client for service "/hrim_actuation_gripper_000000000004/goal"
client = node.create_client(ControlFinger, "/hrim_actuator_gripper_000000000004/fingercontrol", qos_profile=qos_profile_services_default)

# Create request with the same type as the service, ControlFinger
req = ControlFinger.Request()

# Position range 0 - 0.87 rad
req.goal_angularposition = 0.

# Wait for service to be avaiable before calling it
while not client.wait_for_service(timeout_sec=1.0):
   node.get_logger().info('Service not available, waiting again...')

# Call service and spin
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)

# Analyze the result
if future.result() is not None:
    node.get_logger().info('Goal accepted: %d: ' % future.result().goal_accepted)
else:
    node.get_logger().error('Exception while calling service: %r' % future.exception())


node.destroy_node()
rclpy.shutdown()
