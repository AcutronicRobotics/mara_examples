#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_gripper_srvs/srv/control_finger.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  // Create Node with name "mara_minimal_client"
  auto node = rclcpp::Node::make_shared("mara_minimal_client");

  // Create a client for service "/hrim_actuation_gripper_000000000004/goal"
  auto client = node->create_client<hrim_actuator_gripper_srvs::srv::ControlFinger>("/hrim_actuator_gripper_000000000004/fingercontrol");

  // Create request with the same type as the service, ControlFinger
  auto request = std::make_shared<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>();

  // Position range 0 - 0.87 rad
  request->goal_angularposition = 0.;

  // Wait for service to be avaiable before calling it
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  // Call service and spin
  auto future = client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, future);

  // Analyze the result
  auto result = future.get();
  if (result) {
    RCLCPP_INFO(node->get_logger(), "Result of gripper_minimal_client: %d", result->goal_accepted);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}
