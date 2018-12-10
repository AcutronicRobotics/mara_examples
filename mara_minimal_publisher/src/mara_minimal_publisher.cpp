#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("mara_minimal_publisher");

  // create a publisher
  auto joint_publisher = node->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>("/hros_actuation_servomotor_000000000001/goal", rmw_qos_profile_sensor_data);

  rclcpp::WallRate loop_rate(1);

  auto msg = std::make_shared<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>();

  float value = 90; // in degrees

  while (rclcpp::ok()) {
    msg->position = value * 3.1416/180.0;
    msg->velocity = 0.404;
    msg->control_type = 1;

    joint_publisher->publish(msg);

    loop_rate.sleep();
  }

  return 0;
}
