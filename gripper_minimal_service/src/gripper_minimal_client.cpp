#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "hrim_actuator_gripper_srvs/srv/control_finger.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for gripper_minimal_client app:\n");
  printf("gripper_minimal_client [-p position value] [-t topic] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-p position value : Position of the fingers.\n");
  printf("-t topic : Name of the service.\n");
}

hrim_actuator_gripper_srvs::srv::ControlFinger::Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedPtr client,
  hrim_actuator_gripper_srvs::srv::ControlFinger::Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hros_actuator_gripper_srvs");

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  float position = 0.0;
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-p");
  if (nullptr != cli_option) {
    position = std::atof(cli_option);
  }

  std::string topic("goal");
  char * cli_option_topic = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option_topic) {
    topic = std::string(cli_option_topic);
  }

  auto client = node->create_client<hrim_actuator_gripper_srvs::srv::ControlFinger>(topic);

  auto request = std::make_shared<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>();
  request->goal_angularposition = position;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = send_request(node, client, request);
  if (result) {
    RCLCPP_INFO(node->get_logger(), "Result of add_two_ints: %zd", result->goal_accepted);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}
