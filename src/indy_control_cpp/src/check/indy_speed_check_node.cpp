#include "pinocchio/fwd.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "indy_control_cpp/indydcp3.h"


using namespace std::chrono_literals;

struct LinkInfo
{
  std::string name;
  pinocchio::FrameIndex frame_id;
};

struct LinkVelocitySample
{
  double time_sec{0.0};
  double q_deg[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double qdot_deg_s[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  struct LinkVelocity
  {
    std::string name;
    double v_com_x{0.0};
    double v_com_y{0.0};
    double v_com_z{0.0};
  };

  std::vector<LinkVelocity> link_velocities;
};

class IndyLinkComVelocityLoggerNode : public rclcpp::Node
{
public:
  IndyLinkComVelocityLoggerNode()
  : Node("indy_link_com_velocity_logger_node")
  {
    robot_ip_   = "192.168.123.15";
    urdf_path_  = "/home/robotics/indy_ws/src/urdf_file/indy7.urdf";
    json_path_  = "/home/robotics/indy_ws/link_com_velocity_log.json";

    // -----------------------------
    // Pinocchio model
    // -----------------------------
    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    target_links_ = {
      {"link1", model_.getFrameId("link1", pinocchio::BODY)},
      {"link2", model_.getFrameId("link2", pinocchio::BODY)},
      {"link3", model_.getFrameId("link3", pinocchio::BODY)},
      {"link4", model_.getFrameId("link4", pinocchio::BODY)},
      {"link5", model_.getFrameId("link5", pinocchio::BODY)},
      {"link6", model_.getFrameId("link6", pinocchio::BODY)}
    };

    // -----------------------------
    // DCP3 connection
    // -----------------------------
    try {
      indy_ = std::make_unique<IndyDCP3>(robot_ip_);
      RCLCPP_INFO(this->get_logger(), "[INIT] IndyDCP3 connected: %s", robot_ip_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_FATAL(this->get_logger(), "[INIT] Failed to create IndyDCP3: %s", e.what());
      throw;
    } catch (...) {
      RCLCPP_FATAL(this->get_logger(), "[INIT] Failed to create IndyDCP3: unknown exception");
      throw;
    }

    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "=======================================");
    RCLCPP_INFO(this->get_logger(), "Indy Link CoM Velocity Logger Node");
    RCLCPP_INFO(this->get_logger(), "robot_ip  : %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "urdf_path : %s", urdf_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "json_path : %s", json_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "timer     : 10 ms fixed");
    RCLCPP_INFO(this->get_logger(), "q unit    : deg");
    RCLCPP_INFO(this->get_logger(), "qdot unit : deg/s");
    RCLCPP_INFO(this->get_logger(), "=======================================");

    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&IndyLinkComVelocityLoggerNode::timerCallback, this)
    );
  }

  void saveJson()
  {
    std::ofstream ofs(json_path_);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open json file: %s", json_path_.c_str());
      return;
    }

    ofs << "{\n";
    ofs << "  \"robot_ip\": \"" << robot_ip_ << "\",\n";
    ofs << "  \"urdf_path\": \"" << urdf_path_ << "\",\n";
    ofs << "  \"timer_ms\": 10,\n";
    ofs << "  \"q_unit\": \"deg\",\n";
    ofs << "  \"qdot_unit\": \"deg/s\",\n";
    ofs << "  \"v_com_unit\": \"m/s\",\n";
    ofs << "  \"frame_note\": \"LOCAL_WORLD_ALIGNED (world-aligned, base-aligned in this URDF)\",\n";
    ofs << "  \"sample_count\": " << samples_.size() << ",\n";
    ofs << "  \"samples\": [\n";

    for (size_t i = 0; i < samples_.size(); ++i) {
      const auto &s = samples_[i];

      ofs << "    {\n";
      ofs << "      \"time_sec\": " << std::fixed << std::setprecision(6) << s.time_sec << ",\n";

      ofs << "      \"q_deg\": [";
      for (int j = 0; j < 6; ++j) {
        ofs << s.q_deg[j];
        if (j < 5) ofs << ", ";
      }
      ofs << "],\n";

      ofs << "      \"qdot_deg_s\": [";
      for (int j = 0; j < 6; ++j) {
        ofs << s.qdot_deg_s[j];
        if (j < 5) ofs << ", ";
      }
      ofs << "],\n";

      ofs << "      \"links\": [\n";
      for (size_t k = 0; k < s.link_velocities.size(); ++k) {
        const auto &lv = s.link_velocities[k];
        ofs << "        {\n";
        ofs << "          \"name\": \"" << lv.name << "\",\n";
        ofs << "          \"v_com\": ["
            << lv.v_com_x << ", "
            << lv.v_com_y << ", "
            << lv.v_com_z << "]\n";
        ofs << "        }";
        if (k + 1 < s.link_velocities.size()) ofs << ",";
        ofs << "\n";
      }
      ofs << "      ]\n";

      ofs << "    }";
      if (i + 1 < samples_.size()) ofs << ",";
      ofs << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";
    ofs.close();

    RCLCPP_INFO(this->get_logger(), "Saved %zu samples to %s", samples_.size(), json_path_.c_str());
  }

private:
  static Eigen::Matrix3d skew(const Eigen::Vector3d &r)
  {
    Eigen::Matrix3d S;
    S <<     0.0, -r.z(),  r.y(),
          r.z(),     0.0, -r.x(),
         -r.y(),  r.x(),     0.0;
    return S;
  }

  static Eigen::VectorXd degToRad(const Eigen::VectorXd &x_deg)
  {
    return x_deg * M_PI / 180.0;
  }

  double elapsedSec() const
  {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start_time_).count();
  }

  void readRobotData(Eigen::VectorXd &q_deg, Eigen::VectorXd &qdot_deg_s)
  {
    Nrmk::IndyFramework::ControlData control_data;
    bool ok = indy_->get_robot_data(control_data);

    if (!ok) {
      throw std::runtime_error("Failed to get_robot_data()");
    }

    if (control_data.q_size() != model_.nq) {
      throw std::runtime_error("q_size mismatch between DCP3 and Pinocchio model");
    }

    if (control_data.qdot_size() != model_.nv) {
      throw std::runtime_error("qdot_size mismatch between DCP3 and Pinocchio model");
    }

    q_deg.resize(model_.nq);
    qdot_deg_s.resize(model_.nv);

    for (int i = 0; i < control_data.q_size(); ++i) {
      q_deg[i] = control_data.q(i);
    }

    for (int i = 0; i < control_data.qdot_size(); ++i) {
      qdot_deg_s[i] = control_data.qdot(i);
    }
  }

  void computeAndStoreSample(const Eigen::VectorXd &q_deg,
                             const Eigen::VectorXd &qdot_deg_s)
  {
    const Eigen::VectorXd q_rad    = degToRad(q_deg);
    const Eigen::VectorXd qdot_rad = degToRad(qdot_deg_s);

    pinocchio::computeJointJacobians(model_, *data_, q_rad);
    pinocchio::updateFramePlacements(model_, *data_);

    LinkVelocitySample sample;
    sample.time_sec = elapsedSec();

    for (int i = 0; i < 6; ++i) {
      sample.q_deg[i] = q_deg[i];
      sample.qdot_deg_s[i] = qdot_deg_s[i];
    }

    for (const auto &link : target_links_) {
      const auto &frame = model_.frames[link.frame_id];
      const pinocchio::JointIndex joint_id = frame.parentJoint;
      const auto &inertia = model_.inertias[joint_id];

      const double mass = inertia.mass();
      if (mass < 1e-9) {
        continue;
      }

      const Eigen::Vector3d com_local = inertia.lever();
      const Eigen::Matrix3d R_wf = data_->oMf[link.frame_id].rotation();
      const Eigen::Vector3d r_world = R_wf * com_local;

      Eigen::MatrixXd J_frame(6, model_.nv);
      J_frame.setZero();

      pinocchio::getFrameJacobian(
        model_, *data_, link.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame
      );

      // Pinocchio convention: [linear; angular]
      const Eigen::MatrixXd Jv_o = J_frame.topRows(3);
      const Eigen::MatrixXd Jw   = J_frame.bottomRows(3);

      // CoM point linear Jacobian
      const Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

      const Eigen::Vector3d v_com = Jv_com * qdot_rad;

      LinkVelocitySample::LinkVelocity lv;
      lv.name = link.name;
      lv.v_com_x = v_com.x();
      lv.v_com_y = v_com.y();
      lv.v_com_z = v_com.z();

      sample.link_velocities.push_back(lv);
    }

    samples_.push_back(sample);

    static int print_count = 0;
    if ((print_count++ % 100) == 0) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(6);
      oss << "[sample " << samples_.size() << "] ";

      for (size_t i = 0; i < sample.link_velocities.size(); ++i) {
        const auto &lv = sample.link_velocities[i];
        oss << lv.name << ": ["
            << lv.v_com_x << ", "
            << lv.v_com_y << ", "
            << lv.v_com_z << "]";
        if (i + 1 < sample.link_velocities.size()) oss << " | ";
      }

      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
  }

  void timerCallback()
  {
    try {
      Eigen::VectorXd q_deg;
      Eigen::VectorXd qdot_deg_s;
      readRobotData(q_deg, qdot_deg_s);
      computeAndStoreSample(q_deg, qdot_deg_s);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "timerCallback error: %s", e.what()
      );
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "timerCallback unknown error"
      );
    }
  }

private:
  std::string robot_ip_;
  std::string urdf_path_;
  std::string json_path_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<LinkInfo> target_links_;

  std::unique_ptr<IndyDCP3> indy_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point start_time_;
  std::vector<LinkVelocitySample> samples_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IndyLinkComVelocityLoggerNode>();
  rclcpp::spin(node);

  // Ctrl+C 이후 저장
  node->saveJson();

  rclcpp::shutdown();
  return 0;
}