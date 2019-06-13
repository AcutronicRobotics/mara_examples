#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_gripper_srvs/srv/control_finger.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node
{
  public:
    MinimalClient(): Node("gripper_minimal_client")
    {
      client = this->create_client<hrim_actuator_gripper_srvs::srv::ControlFinger>(
        "/hrim_actuator_gripper_000000000004/fingercontrol");

      // Wait for service to be avaiable before calling it
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }

      request = std::make_shared<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>();
    }

    void send_request();
    rclcpp::Client<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedFuture future;

  private:
    rclcpp::Client<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedPtr client;
    rclcpp::Client<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedRequest request;
};

void MinimalClient::send_request(){
  request->goal_angularposition = 0.;;
  this->future = client->async_send_request(request);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalClient>();

  node->send_request();
  rclcpp::spin_until_future_complete(node, node->future);

  // Analyze the result
  auto result = node->future.get();
  if (result) {
    RCLCPP_INFO(node->get_logger(), "Result of gripper_minimal_client: %d", result->goal_accepted);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}
