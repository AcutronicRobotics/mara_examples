#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import StateRotaryServo

# Function that will be called once a message is published to the topic we are subscribed
def minimal_callback(msg):
    print('Position:', str(msg.position))

# -------- #

rclpy.init(args=None)

# Create Node with name "mara_minimal_subscriber"
node = rclpy.create_node('mara_minimal_subscriber')

# Subscribe to topic "/hrim_actuation_servomotor_000000000001/state_axis1" and link it to "minimal_callback" function
node.create_subscription(StateRotaryServo, '/hrim_actuator_rotaryservo_000000000001/state_axis1', minimal_callback,
    qos_profile=qos_profile_sensor_data) # QoS profile for reading (joint) sensors

# Spin listening to all subscribed topics
rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
