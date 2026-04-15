#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>   // std::clamp
#include <memory>

#include "indy_control_cpp/indydcp3.h"

using std::placeholders::_1;

class IndySpeedNode : public rclcpp::Node
{
public:
  IndySpeedNode()
  : Node("indy_speed_node"),
    indy_("192.168.123.15")   // ✅ 여기서 딱 한 번 연결
  {
    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/indy/speed_scale",
      10,
      std::bind(&IndySpeedNode::speedCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Indy Speed Node started");
  }

private:
  void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float scale = msg->data;
    scale = std::clamp(scale, 0.0f, 1.0f);

    unsigned int speed_ratio =
      static_cast<unsigned int>(scale * 100.0f);

    bool is_success = indy_.set_speed_ratio(speed_ratio);

    if (is_success) {
      RCLCPP_INFO(
        this->get_logger(),
        "Speed ratio set to %u%%",
        speed_ratio
      );
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set speed ratio"
      );
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  IndyDCP3 indy_;   // ✅ 로봇 연결은 멤버로 유지
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndySpeedNode>());
  rclcpp::shutdown();
  return 0;
}
