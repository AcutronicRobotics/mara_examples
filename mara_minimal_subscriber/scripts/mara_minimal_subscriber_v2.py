#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import StateRotaryServo

class MinimalSubscriber(Node):

    def __init__(self):
        # Initialize Node with name "mara_minimal_subscriber"
        super().__init__('mara_minimal_subscriber')

        # Subscribe to topic "/hrim_actuation_servomotor_000000000001/state_axis1" and link it to "minimal_callback" function
        self.create_subscription(StateRotaryServo, '/hrim_actuator_rotaryservo_000000000001/state_axis1',
            self.minimal_callback,
            qos_profile=qos_profile_sensor_data) # QoS profile for reading (joint) sensors


    def minimal_callback(self, msg):
        '''
        Function that will be called once a message is published to the topic we are subscribed
        '''
        self.get_logger().info('Position: {}'.format(msg.position))


def main(args=None):
    rclpy.init(args=args)

    node = MinimalSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
