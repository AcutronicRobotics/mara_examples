#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher() : Node("mara_minimal_publisher"),
    count_(0)
    {
      // Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
      pub_ = this->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>("/hrim_actuator_rotaryservo_000000000001/goal_axis1", rclcpp::SensorDataQoS());

      // Publishing rate of 1 Hz using a wall timer
      timer_ = this->create_wall_timer(1s, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // Create message with the same type as the topic, <hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>.
      auto msg = std::make_shared<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>();

      // Fill message content
      float position_deg = 30;
      msg->position = position_deg * 3.1416/180.0; // Position to rads
      msg->velocity = 0.4; // Velocity in rads/s
      msg->control_type = 4; // Position and velocity control

      // Publish message!
      pub_->publish(*msg);

      // Log
      RCLCPP_INFO(this->get_logger(), "Iteration number %d", count_);
      count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>::SharedPtr pub_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
