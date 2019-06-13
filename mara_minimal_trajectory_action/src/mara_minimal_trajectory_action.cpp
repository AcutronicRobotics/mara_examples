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
                                        feedback->actual.positions[0]*180/3.1416,
                                        time_from_start);

  // Not working
  // double time_from_start_desired = (double)(feedback->desired.time_from_start.sec) +
  //                                  (double)(feedback->desired.time_from_start.nanosec/1e+9);
  // RCLCPP_INFO(node->get_logger(), "Desired degrees: %.2f\ttime:%.5f",
  //                                 feedback->desired.positions[0]*180/3.1416,
  //                                 time_from_start_desired);
  //
  // double time_from_start_error = (double)(feedback->error.time_from_start.sec) +
  //                                 (double)(feedback->error.time_from_start.nanosec/1e+9);
  // RCLCPP_INFO(node->get_logger(), "Error degrees: %.2f\ttime:%.5f",
  //                                 feedback->error.positions[0]*180/3.1416,
  //                                 time_from_start_error);

  RCLCPP_INFO(node->get_logger(), "-----------------------------");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string action_name("/hrim_actuator_rotaryservo_000000000001/trajectory_axis1");
  printf("Trying to connect with the action %s\n", action_name.c_str());

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

  RCLCPP_INFO(node->get_logger(), "Sending goal");
  // Ask server to achieve some goal. Assign feedback callback
  auto goal_handle_future = action_client->async_send_goal(goal_msg, feedback_callback, true);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "`send_goal` call failed");
    return 1;
  }


  //
  // ----------- GOAL RESPONSE ----------- //
  //

  GoalHandleHRIMJointTrajectory::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return 0;
  }

  //
  // ----------- GET RESULT ----------- //
  //

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");

  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "`get_result` call failed");
    return 1;
  }

  // TODO: Here program is shutdown. Probably because of spin_until_future_complete

  GoalHandleHRIMJointTrajectory::Result result = result_future.get();

  switch(result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return 1;
  }




  action_client.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
