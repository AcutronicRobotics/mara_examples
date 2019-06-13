#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo

class MinimalPublisher(Node):

    def __init__(self):
        # Initialize Node with name "mara_minimal_publisher"
        super().__init__('mara_minimal_publisher')

        # Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
        self.pub_ = self.create_publisher(GoalRotaryServo, '/hrim_actuator_rotaryservo_000000000001/goal_axis1',
                                                qos_profile=qos_profile_sensor_data)

        # Create message with the same type as the topic, GoalRotaryServo
        self.msg = GoalRotaryServo()

        # Create a timer to publish the messages periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0 # Iteration counter


    def timer_callback(self):
        # Fill message content
        position_deg = 30.
        self.msg.position = position_deg * 3.1416/180 # Position to rads
        self.msg.velocity = 0.4                       # Velocity in rads/s
        self.msg.control_type = 4                     # Position and velocity control

        # Publish message!
        self.pub_.publish(self.msg)

        # Log
        self.get_logger().info("Iteration number: {}".format(self.i))
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
