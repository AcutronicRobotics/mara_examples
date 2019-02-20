#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_gripper_msgs/msg/state_gripper.hpp"
#include "hrim_actuator_gripper_msgs/msg/state_finger_gripper.hpp"
#include "rcutils/cmdline_parser.h"

void print_usage()
{
  printf("Usage for gripper_minimal_client app:\n");
  printf("gripper_minimal_subscriber [-t topic] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t node name : Name of the node.\n");
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(std::string node_name)
  : Node("mara_minimal_subscriber")
  {
    subscription_state_gripper_ = this->create_subscription<hrim_actuator_gripper_msgs::msg::StateGripper>(
      node_name + "/state_gripper",
      [this](hrim_actuator_gripper_msgs::msg::StateGripper::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "on_off: %.5f", msg->on_off);
    });

    std::cout << "subscribed to " << node_name + "/state_gripper"  <<  std::endl;
    subscription_state_finger_gripper_ = this->create_subscription<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(
      node_name + "/state_finger_gripper",
      [this](hrim_actuator_gripper_msgs::msg::StateFingerGripper::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Angular position: %.5f\tLinear position: %.5f\n", msg->angular_position, msg->linear_position);
    });
    std::cout << "subscribed to " << node_name + "/state_finger_gripper"  <<  std::endl;

  }

private:
  rclcpp::Subscription<hrim_actuator_gripper_msgs::msg::StateGripper>::SharedPtr subscription_state_gripper_;
  rclcpp::Subscription<hrim_actuator_gripper_msgs::msg::StateFingerGripper>::SharedPtr subscription_state_finger_gripper_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  std::string node_name("/hros_actuation_gripper_000000000004");
  char * cli_option_topic = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option_topic) {
    node_name = std::string(cli_option_topic);
  }

  rclcpp::spin(std::make_shared<MinimalSubscriber>(node_name));
  rclcpp::shutdown();
  return 0;
}
