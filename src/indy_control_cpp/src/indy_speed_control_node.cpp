#include <rclcpp/rclcpp.hpp>
#include <algorithm>   // std::clamp
#include <memory>

#include "indy_control_cpp/indydcp3.h"
#include "indy_control_cpp/perf_logger.hpp"
#include "hrc_interfaces/msg/speed_scale_command.hpp"

using std::placeholders::_1;

class IndySpeedNode : public rclcpp::Node
{
public:
  IndySpeedNode()
  : Node("indy_speed_node"),
    indy_("192.168.123.15")   // ✅ 여기서 딱 한 번 연결
  {
    sub_ = this->create_subscription<hrc_interfaces::msg::SpeedScaleCommand>(
      "/indy/speed_scale",
      10,
      std::bind(&IndySpeedNode::speedCallback, this, _1)
    );

    perf_speed_logger_ = std::make_unique<PerfCsvLogger>(
    "/home/robotics/hrc_ws/analysis/realtime_perf/paper_robot_speed_command_perf.csv",
    std::vector<std::string>{
      "t_ros_sec",
      "assessment_id",
      "speed_scale",
      "speed_ratio_percent",
      "command_transport_ms",
      "set_speed_ratio_ms",
      "end_to_end_assessment_to_command_ms",
      "success"
    },
    30
  );

    RCLCPP_INFO(this->get_logger(), "Indy Speed Node started");
  }

private:
  std::unique_ptr<PerfCsvLogger> perf_speed_logger_;

  void speedCallback(
    const hrc_interfaces::msg::SpeedScaleCommand::SharedPtr msg)
  {
    const rclcpp::Time callback_time = this->now();
    float scale = msg->speed_scale;
    scale = std::clamp(scale, 0.0f, 1.0f);

    unsigned int speed_ratio =
      static_cast<unsigned int>(scale * 100.0f);

    const auto t0 = PerfCsvLogger::now();
    bool is_success = indy_.set_speed_ratio(speed_ratio);
    const double command_ms = PerfCsvLogger::msSince(t0);
    const rclcpp::Time command_done_time = this->now();

    double transport_ms = -1.0;
    if (rclcpp::Time(msg->header.stamp).nanoseconds() > 0) {
      transport_ms =
        (callback_time - rclcpp::Time(msg->header.stamp)).nanoseconds() / 1e6;
    }

    double end_to_end_ms = -1.0;
    if (msg->assessment_id > 0 &&
        rclcpp::Time(msg->assessment_start_stamp).nanoseconds() > 0) {
      end_to_end_ms =
        (command_done_time - rclcpp::Time(msg->assessment_start_stamp))
        .nanoseconds() / 1e6;
    }

    if (perf_speed_logger_) {
      perf_speed_logger_->writeRow({
        PerfCsvLogger::toStr(this->now().seconds()),
        std::to_string(msg->assessment_id),
        PerfCsvLogger::toStr(scale),
        std::to_string(speed_ratio),
        PerfCsvLogger::toStr(transport_ms),
        PerfCsvLogger::toStr(command_ms),
        PerfCsvLogger::toStr(end_to_end_ms),
        std::to_string(static_cast<int>(is_success))
      });
    }

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

  rclcpp::Subscription<hrc_interfaces::msg::SpeedScaleCommand>::SharedPtr sub_;
  IndyDCP3 indy_;   // ✅ 로봇 연결은 멤버로 유지
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndySpeedNode>());
  rclcpp::shutdown();
  return 0;
}
