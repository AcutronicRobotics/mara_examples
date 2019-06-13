#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hrim_actuator_rotaryservo_actions/action/goal_joint_trajectory.hpp"

#include <inttypes.h>
#include <memory>

using HRIMJointTrajectory = hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory;
using GoalHandleHRIMJointTrajectory = rclcpp_action::ClientGoalHandle<HRIMJointTrajectory>;

// node as global variable
rclcpp::Node::SharedPtr node = nullptr;

void feedback_callback(
  GoalHandleHRIMJointTrajectory::SharedPtr,
  const std::shared_ptr<const HRIMJointTrajectory::Feedback> feedback)
{
  //
  // ----------- FEEDBACK ----------- //
  //

  double time_from_start = (double)(feedback->actual.time_from_start.sec) +
                           (double)(feedback->actual.time_from_start.nanosec/1e+9);
  RCLCPP_INFO(node->get_logger(), "Current degrees: %.2f\ttime:%.5f",
                                  (double)feedback->actual.positions[0]*180/3.1416,
                                  time_from_start);
  RCLCPP_INFO(node->get_logger(), "-----------------------------");
}


void goal_response_callback(std::shared_future<GoalHandleHRIMJointTrajectory::SharedPtr> goal_handle_future)
{
  //
  // ----------- GOAL RESPONSE ----------- //
  //

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(node->get_logger(), "Goal accepted");
}


void result_callback(const GoalHandleHRIMJointTrajectory::WrappedResult & result)
{
  //
  // ----------- RESULT ----------- //
  //

  switch(result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return;
  }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string action_name("/hrim_actuator_rotaryservo_000000000001/trajectory_axis1");
  printf("Trying to connect with the action %s\n", action_name.c_str());

  // Initialize node and client
  node = rclcpp::Node::make_shared("minimal_action_client");
  auto action_client = rclcpp_action::create_client<HRIMJointTrajectory>(node, action_name);


  //
  // ----------- SEND GOAL ----------- //
  //

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = HRIMJointTrajectory::Goal();
  goal_msg.trajectory.joint_names.push_back("motor1");

  // Define point1 in pos=0 at time=0
  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions.push_back(0);
  point1.velocities.push_back(0);
  point1.time_from_start.sec = 0;
  point1.time_from_start.nanosec = 0;

  // Define point2 in pos=0.2 at time=5
  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.positions.push_back(0.5);
  point2.velocities.push_back(0);
  point2.time_from_start.sec = 5; // velocity 0.5rad/5s = 0.1 rad/s
  point2.time_from_start.nanosec = 0;

  // Define point3 in pos=0 at time=7.5
  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.positions.push_back(0);
  point3.velocities.push_back(0);
  point3.time_from_start.sec = 7; // velocity 0.5rad/(7.5s-5s) = 0.2 rad/s
  point3.time_from_start.nanosec = 5e8;

  // Add points to goal_msg
  goal_msg.trajectory.points.push_back(point1);
  goal_msg.trajectory.points.push_back(point2);
  goal_msg.trajectory.points.push_back(point3);

  // Gather callbacks
  rclcpp_action::Client<HRIMJointTrajectory>::SendGoalOptions options = rclcpp_action::Client<HRIMJointTrajectory>::SendGoalOptions();
  options.feedback_callback = feedback_callback;
  options.goal_response_callback = goal_response_callback;
  options.result_callback = result_callback;

  // Ask server to achieve some goal. Assign feedback, goal_response and result callbacks
  RCLCPP_INFO(node->get_logger(), "Sending goal");
  auto goal_handle_future = action_client->async_send_goal(goal_msg, options);

  //
  // ----------- GOAL SENT ----------- //
  //

  // Spin indefinitely
  rclcpp::spin(node);

  // Shutdown
  action_client.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
