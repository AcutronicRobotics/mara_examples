#include "rclcpp/rclcpp.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("minimal_subscriber")
    {
      sub_ = this->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
        "/hrim_actuator_rotaryservo_000000000001/state_axis1",
        rclcpp::SensorDataQoS(),
        std::bind(&MinimalSubscriber::minimal_callback, this, _1));
    }

  private:
    void minimal_callback(const hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Position: %f", msg->position);
    }

    rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
