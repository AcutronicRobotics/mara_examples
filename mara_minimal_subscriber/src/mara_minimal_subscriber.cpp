#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"

void minimalCallback(const hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg)
{
  std::cout << "Position: " << msg->position << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("mara_minimal_subscriber");

  auto sub = node->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
    "/hrim_actuation_servomotor_000000000001/state_axis1",
    minimalCallback,
    rmw_qos_profile_sensor_data); // QoS profile for reading (joint) sensors

  // Spin listening to all subscribed topics
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
