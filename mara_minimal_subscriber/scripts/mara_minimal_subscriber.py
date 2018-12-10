#!/usr/bin/python3

# ROS 2.0
import rclpy

# HRIM
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('mara_minimal_subscriber')

    subscription = node.create_subscription(
        GoalRotaryServo, '/hros_actuation_servomotor_000000000001/state', lambda msg: node.get_logger().info('Position:' + str(msg.position)))
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
