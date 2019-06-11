#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create Node with name "mara_minimal_publisher"
  auto node = rclcpp::Node::make_shared("mara_minimal_publisher");

  // Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
  auto pub = node->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>("/hrim_actuator_rotaryservo_000000000001/goal_axis1", rclcpp::SensorDataQoS());

  // Publishing rate of 1 Hz
  rclcpp::WallRate loop_rate(1);

  // Create message with the same type as the topic, <hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>.
  auto msg = std::make_shared<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>();

  // Desired position in degrees
  float position_deg = 30;

  // Loop
  int i = 1; // Loop counter
  while (rclcpp::ok()) {
    // Fill message content
    msg->position = position_deg * 3.1416/180.0; // Position to rads
    msg->velocity = 0.4; // Velocity in rads/s
    msg->control_type = 4; // Position and velocity control

    // Publish message!
    pub->publish(*msg);

    // Spin not really needed here since we don't have callbacks
    rclcpp::spin_some(node);

    // Sleep to mantain the 1 Hz publishing rate
    loop_rate.sleep();

    // Log
    RCLCPP_INFO(node->get_logger(), "Iteration number %d", i);
    i++;
  }

  rclcpp::shutdown();

  return 0;
}
