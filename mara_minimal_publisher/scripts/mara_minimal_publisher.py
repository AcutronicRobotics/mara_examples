#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from time import sleep

# -------- #

rclpy.init(args=None)

# Create Node with name "mara_minimal_publisher"
node = rclpy.create_node("mara_minimal_publisher")

# Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
pub = node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_000000000001/goal_axis1', qos_profile=qos_profile_sensor_data)

# Create message with the same type as the topic, GoalRotaryServo
msg = GoalRotaryServo()

# Desired position in degrees
position_deg = 30

# Loop
i = 1 # Loop counter
while rclpy.ok():
    # Fill message content
    msg.position = position_deg * 3.1416/180 # Position to rads
    msg.velocity = 0.4 # Velocity in rads/s
    msg.control_type = 4 # Position and velocity control

    # Publish message!
    pub.publish(msg)

    # Spin not really needed here since we don't have callbacks
    #rclpy.spin_once(node)

    # Sleep 1 second per loop
    sleep(1.)

    # Log
    print("Iteration number", i)
    i += 1


node.destroy_node()
rclpy.shutdown()
