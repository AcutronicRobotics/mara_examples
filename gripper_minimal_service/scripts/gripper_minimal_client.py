#ROS 2.0
from hrim_actuator_gripper_srvs.srv import ControlFinger
import rclpy

rclpy.init(args=None)
node = rclpy.create_node('test_finger_control_service')

cli = node.create_client(ControlFinger, '/hros_actuation_gripper_000000000004/goal')

req = ControlFinger.Request()

req.goal_velocity = 30.0         # velocity range: 30 -  250 mm/s
req.goal_effort = 10.0        # forces range:   10 -  125 N
req.goal_angularposition = 0.87  # position range   0 - 0.87 rad

future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future)
if future.result() is not None:
    node.get_logger().info('Goal accepted: %d: ' % future.result().goal_accepted)
else:
    node.get_logger().error('Exception while calling service: %r' % future.exception())
