#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "indy_control_cpp/indydcp3.h"

using namespace std::chrono_literals;

class IndyConnectionCheckNode : public rclcpp::Node
{
public:
  IndyConnectionCheckNode()
  : Node("indy_connection_check_node")
  {
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.123.15");
    check_period_sec_ = this->declare_parameter<double>("check_period_sec", 2.0);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Indy Connection Check Node");
    RCLCPP_INFO(this->get_logger(), "robot_ip         : %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "check_period_sec : %.2f", check_period_sec_);
    RCLCPP_INFO(this->get_logger(), "========================================");

    try {
      indy_ = std::make_unique<IndyDCP3>(robot_ip_);
      RCLCPP_INFO(this->get_logger(), "[INIT] IndyDCP3 object created");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[INIT] Failed to create IndyDCP3: %s", e.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "[INIT] Failed to create IndyDCP3: unknown exception");
      return;
    }

    auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(check_period_sec_)
    );

    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&IndyConnectionCheckNode::readAndPrintQ, this)
    );
  }

private:
  void readAndPrintQ()
  {
    if (!indy_) {
      RCLCPP_ERROR(this->get_logger(), "[READ] Indy object is null");
      return;
    }

    Nrmk::IndyFramework::ControlData control_data;
    bool ok = false;

    try {
      ok = indy_->get_robot_data(control_data);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[READ] EXCEPTION during get_robot_data(): %s", e.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "[READ] UNKNOWN EXCEPTION during get_robot_data()");
      return;
    }

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "[READ] Failed to get_robot_data()");
      return;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "[q] ";

    for (int i = 0; i < control_data.q_size(); ++i) {
      oss << "q" << i << ": " << control_data.q(i);
      if (i + 1 < control_data.q_size()) {
        oss << " | ";
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

private:
  std::unique_ptr<IndyDCP3> indy_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_ip_;
  double check_period_sec_{2.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndyConnectionCheckNode>());
  rclcpp::shutdown();
  return 0;
}