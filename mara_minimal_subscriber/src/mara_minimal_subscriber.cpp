#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("mara_minimal_subscriber")
  {
    subscription_ = this->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
      "/hros_actuation_servomotor_000000000001/state",
      [this](hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Position: %.5f", msg->position);
    });
  }

private:
  rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
