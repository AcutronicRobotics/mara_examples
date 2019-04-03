#include <inttypes.h>
#include <memory>

// ROS 2.0
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcutils/cmdline_parser.h"

// HRIM
#include "hrim_actuator_rotaryservo_actions/action/goal_joint_trajectory.hpp"

rclcpp::Node::SharedPtr g_node = nullptr;

void print_usage()
{
  printf("Usage for gripper_minimal_client app::\n");
  printf("ros2 run mara_minimal_trajectory_action mara_minimal_trajectory_action [-t topic] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t action name : Name of the action.\n");
}

void feedback_callback(
  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr,
  const std::shared_ptr<const hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Feedback> feedback)
{
  double time_from_start = (double)(feedback->actual.time_from_start.sec) +
                           (double)(feedback->actual.time_from_start.nanosec/1e+9);
  double time_from_start_desired = (double)(feedback->desired.time_from_start.sec) +
                                   (double)(feedback->desired.time_from_start.nanosec/1e+9);
  double time_from_start_error = (double)(feedback->error.time_from_start.sec) +
                                 (double)(feedback->error.time_from_start.nanosec/1e+9);

  RCLCPP_INFO(g_node->get_logger(), "Current degrees: %.2f\ttime:%.5f",
                                        feedback->actual.positions[0]*180/3.1416,
                                        time_from_start);
  RCLCPP_INFO(g_node->get_logger(), "Desired degrees: %.2f\ttime:%.5f",
                                        feedback->desired.positions[0]*180/3.1416,
                                        time_from_start_desired);
  RCLCPP_INFO(g_node->get_logger(), "Error degrees: %.2f\ttime:%.5f",
                                        feedback->error.positions[0]*180/3.1416,
                                        time_from_start_error);
  RCLCPP_INFO(g_node->get_logger(), "-----------------------------");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  std::string action_name("/trajectory");
  if (rcutils_cli_option_exist(argv, argv + argc, "-topic")){
    action_name = rcutils_cli_get_option(argv, argv + argc, "-topic");
  }

  printf("Trying to connect with the action %s\n", action_name.c_str());

  g_node = rclcpp::Node::make_shared("minimal_action_client");
  auto action_client = rclcpp_action::create_client<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>(g_node, action_name);

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(g_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal();
  goal_msg.trajectory.joint_names.push_back("motor1");

  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions.push_back(0);
  point1.velocities.push_back(0);
  point1.time_from_start.sec = 0;
  point1.time_from_start.nanosec = 0;
  goal_msg.trajectory.points.push_back(point1);

  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.positions.push_back(1.57);
  point2.velocities.push_back(0);
  point2.time_from_start.sec = 5; // velocity 1.57rad/5s = 0.314 rad/s
  point2.time_from_start.nanosec = 0;
  goal_msg.trajectory.points.push_back(point2);

  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.positions.push_back(0);
  point3.velocities.push_back(0);
  point3.time_from_start.sec = 10; // velocity 1.57rad/(10s-5s) = 0.314 rad/s
  point3.time_from_start.nanosec = 0;
  goal_msg.trajectory.points.push_back(point3);

  RCLCPP_INFO(g_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto goal_handle_future = action_client->async_send_goal(goal_msg, feedback_callback, true);

  if (rclcpp::spin_until_future_complete(g_node, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(g_node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return 0;
  }

  printf("get_status %d\n", goal_handle->get_status());

  bool action_server_is_ready = action_client->action_server_is_ready();
  printf("action_server_is_ready: %d\n", action_server_is_ready);

  std::cout << "get_goal_stamp " << goal_handle->get_goal_stamp().seconds() << std::endl;

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(g_node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(g_node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::Result result = result_future.get();

  switch(result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(g_node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(g_node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(g_node->get_logger(), "Unknown result code");
      return 1;
  }

  action_client.reset();
  g_node.reset();
  rclcpp::shutdown();
  return 0;
}
