#!/usr/bin/python3

# ROS 2.0
import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
# HRIM
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo

rclpy.init(args=None)

node = rclpy.create_node('test_finger_control_service')

publisher = node.create_publisher(GoalRotaryServo, '/hros_actuation_servomotor_000000000001/command', qos_profile=qos_profile_sensor_data)

value = 90 # in degrees

msg = GoalRotaryServo()
msg.position = value * 3.1416/180
msg.velocity = 0.404
msg.control_type = 1
publisher.publish(msg)

rclpy.spin(node)
