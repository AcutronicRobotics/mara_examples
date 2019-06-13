#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"

// Function that will be called once a message is published to the topic we are subscribed
void minimal_callback(const hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg)
{
  std::cout << "Position: " << msg->position << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create Node with name "mara_minimal_subscriber"
  auto node = rclcpp::Node::make_shared("mara_minimal_subscriber");

  // Subscribe to topic "/hrim_actuation_servomotor_000000000001/state_axis1" and link it to "minimal_callback" function
  auto sub = node->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
    "/hrim_actuator_rotaryservo_000000000001/state_axis1",
    rclcpp::SensorDataQoS(), // QoS profile for reading (joint) sensors
    minimal_callback);

  // Spin listening to all subscribed topics
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
