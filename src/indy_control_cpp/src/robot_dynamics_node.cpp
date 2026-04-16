#include "pinocchio/fwd.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <sstream>
#include <fstream>
#include <vector>
#include <array>

#include "indy_control_cpp/indydcp3.h"
#include <hrc_interfaces/msg/robot_dynamics_state.hpp>

using namespace std::chrono_literals;

struct LinkInfo
{
  std::string name;
  pinocchio::FrameIndex frame_id;
};

struct VelocitySample
{
  double time_sec{0.0};

  double q_deg[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double v_com_x{0.0};
  double v_com_y{0.0};
  double v_com_z{0.0};

  double v_frame_x{0.0};
  double v_frame_y{0.0};
  double v_frame_z{0.0};

  double w_frame_x{0.0};
  double w_frame_y{0.0};
  double w_frame_z{0.0};
};

class IndyRobotMeffNode : public rclcpp::Node
{
public:
  IndyRobotMeffNode()
  : Node("robot_dynamics_node")
  {

    // ================================
    // ROS2 
    // ================================
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    pub_robot_state_ = this->create_publisher<hrc_interfaces::msg::RobotDynamicsState>(
        "/indy/robot_dynamics_state", qos);


    // ================================
    // fixed settings
    // ================================
    robot_ip_  = "192.168.123.15";
    urdf_path_ = "/home/robotics/indy_ws/src/urdf_file/indy7.urdf";


    // ================================
    // Pinocchio model
    // ================================
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

    // ================================
    // gear reflected inertia settings
    // J1,J2 = 121 / J3~J6 = 101
    // rotor inertia: assumed values
    // lambda_i = N_i^2 * Jm_i
    // ================================
    gear_ratio_ << 121.0, 121.0, 101.0, 101.0, 101.0, 101.0;
    motor_rotor_inertia_ << 4.75e-05, 4.75e-05, 2.52e-05, 2.52e-05, 1.41e-05, 1.41e-05;
    lambda_diag_ = gear_ratio_.array().square() * motor_rotor_inertia_.array();

    // ================================
    // DCP3 connection
    // ================================
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

    RCLCPP_INFO(this->get_logger(), "=======================================");
    RCLCPP_INFO(this->get_logger(), "Indy Robot Meff Node");
    RCLCPP_INFO(this->get_logger(), "robot_ip  : %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "urdf_path : %s", urdf_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "timer     : 10 ms fixed");
    RCLCPP_INFO(this->get_logger(), "q input   : deg -> rad");
    RCLCPP_INFO(this->get_logger(), "gear mode : enabled");
    RCLCPP_INFO(this->get_logger(), "=======================================");

    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&IndyRobotMeffNode::timerCallback, this)
    );
  }

private:
  rclcpp::Publisher<hrc_interfaces::msg::RobotDynamicsState>::SharedPtr pub_robot_state_;
  Eigen::Vector3d robot_base_offset_world_{0.0, 0.0, 0.6};

  void publishRobotDynamicsState(
    const std::vector<Eigen::Vector3d> &link_com_positions,
    const std::vector<Eigen::Vector3d> &link_com_velocities)
  {
    if (link_com_positions.size() != target_links_.size() ||
        link_com_velocities.size() != target_links_.size()) {
      RCLCPP_WARN(this->get_logger(),
        "Robot dynamics state size mismatch: pos=%zu vel=%zu target_links=%zu",
        link_com_positions.size(), link_com_velocities.size(), target_links_.size());
      return;
    }


    hrc_interfaces::msg::RobotDynamicsState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";

    msg.link_indices.reserve(target_links_.size());
    msg.link_names.reserve(target_links_.size());
    msg.link_positions.reserve(target_links_.size() * 3);
    msg.link_velocities.reserve(target_links_.size() * 3);

    for (size_t i = 0; i < target_links_.size(); ++i) {
      msg.link_indices.push_back(static_cast<uint32_t>(i));
      msg.link_names.push_back(target_links_[i].name);

      const auto &p = link_com_positions[i];
      msg.link_positions.push_back(static_cast<float>(p.x()));
      msg.link_positions.push_back(static_cast<float>(p.y()));
      msg.link_positions.push_back(static_cast<float>(p.z()));

      const auto &v = link_com_velocities[i];
      msg.link_velocities.push_back(static_cast<float>(v.x()));
      msg.link_velocities.push_back(static_cast<float>(v.y()));
      msg.link_velocities.push_back(static_cast<float>(v.z()));
    }

    pub_robot_state_->publish(msg);
  }

  static Eigen::Matrix3d skew(const Eigen::Vector3d &r)
  {
    Eigen::Matrix3d S;
    S <<     0.0, -r.z(),  r.y(),
          r.z(),     0.0, -r.x(),
         -r.y(),  r.x(),     0.0;
    return S;
  }

  static double effectiveMassAlong(const Eigen::Matrix3d &A,
                                   const Eigen::Vector3d &u_raw,
                                   double eps = 1e-12)
  {
    Eigen::Vector3d u = u_raw.normalized();
    double denom = (u.transpose() * A * u)(0, 0);

    if (std::abs(denom) < eps) {
      return std::numeric_limits<double>::infinity();
    }
    return 1.0 / denom;
  }

  Eigen::VectorXd readQDegFromRobot()
  {
    Nrmk::IndyFramework::ControlData control_data;
    bool ok = indy_->get_robot_data(control_data);

    if (!ok) {
      throw std::runtime_error("Failed to get_robot_data()");
    }

    if (control_data.q_size() != model_.nq) {
      throw std::runtime_error("q_size mismatch between DCP3 and Pinocchio model");
    }

    Eigen::VectorXd q_deg(model_.nq);
    for (int i = 0; i < control_data.q_size(); ++i) {
      q_deg[i] = control_data.q(i);
    }
    return q_deg;
  }

  Eigen::VectorXd readQdotDegFromRobot()
  {
    Nrmk::IndyFramework::ControlData control_data;
    bool ok = indy_->get_robot_data(control_data);

    if (!ok) {
      throw std::runtime_error("Failed to get_robot_data()");
    }

    if (control_data.qdot_size() != model_.nv) {
      throw std::runtime_error("qdot_size mismatch between DCP3 and Pinocchio model");
    }

    Eigen::VectorXd qdot_deg(model_.nv);
    for (int i = 0; i < control_data.qdot_size(); ++i) {
      qdot_deg[i] = control_data.qdot(i);
    }
    return qdot_deg;
  }

  static Eigen::VectorXd degToRad(const Eigen::VectorXd &q_deg)
  {
    return q_deg * M_PI / 180.0;
  }

  void computeMeffWithGear(const Eigen::VectorXd &q_rad,
                           std::vector<double> &meff_x_out)
  {
    pinocchio::computeJointJacobians(model_, *data_, q_rad);
    pinocchio::updateFramePlacements(model_, *data_);

    pinocchio::crba(model_, *data_, q_rad);
    data_->M.triangularView<Eigen::StrictlyLower>() =
        data_->M.transpose().triangularView<Eigen::StrictlyLower>();

    Eigen::MatrixXd M_aug = data_->M;
    M_aug.diagonal() += lambda_diag_;

    Eigen::LDLT<Eigen::MatrixXd> ldlt_aug(M_aug);

    // base/world +X direction
    const Eigen::Vector3d u = Eigen::Vector3d::UnitX();

    meff_x_out.clear();
    meff_x_out.reserve(target_links_.size());

    for (const auto &link : target_links_) {
      const auto &frame = model_.frames[link.frame_id];
      const pinocchio::JointIndex joint_id = frame.parentJoint;
      const auto &inertia = model_.inertias[joint_id];

      const double mass = inertia.mass();
      if (mass < 1e-9) {
        meff_x_out.push_back(0.0);
        continue;
      }

      const Eigen::Vector3d com_local = inertia.lever();
      const Eigen::Matrix3d R_wf = data_->oMf[link.frame_id].rotation();
      const Eigen::Vector3d p_wf = data_->oMf[link.frame_id].translation();

      const Eigen::Vector3d r_world = R_wf * com_local;
      (void)p_wf;

      Eigen::MatrixXd J_frame(6, model_.nv);
      J_frame.setZero();

      pinocchio::getFrameJacobian(
          model_, *data_, link.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

      // Pinocchio convention: [linear; angular]
      Eigen::MatrixXd Jv_o = J_frame.topRows(3);
      Eigen::MatrixXd Jw   = J_frame.bottomRows(3);

      // CoM point Jacobian
      Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

      Eigen::MatrixXd X_aug = ldlt_aug.solve(Jv_com.transpose());
      Eigen::Matrix3d A_aug = Jv_com * X_aug;

      double meff_aug = effectiveMassAlong(A_aug, u);
      meff_x_out.push_back(meff_aug);
    }
  }

  void debugFrameVelocity(const Eigen::VectorXd &q_rad,
                        const Eigen::VectorXd &qdot_rad)
  {
    // tcp 대신 link6 BODY frame 기준으로 디버그
    const pinocchio::FrameIndex frame_id =
        model_.getFrameId("link6", pinocchio::BODY);

    pinocchio::computeJointJacobians(model_, *data_, q_rad);
    pinocchio::updateFramePlacements(model_, *data_);

    Eigen::MatrixXd J_frame(6, model_.nv);
    J_frame.setZero();

    pinocchio::getFrameJacobian(
        model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // Pinocchio convention: [linear; angular]
    Eigen::VectorXd twist = J_frame * qdot_rad;

    Eigen::Vector3d v_frame = twist.head<3>();   // m/s
    Eigen::Vector3d w_frame = twist.tail<3>();   // rad/s

    // CoM point velocity도 같이 계산
    const auto &frame = model_.frames[frame_id];
    const pinocchio::JointIndex joint_id = frame.parentJoint;
    const auto &inertia = model_.inertias[joint_id];

    const Eigen::Vector3d com_local = inertia.lever();
    const Eigen::Matrix3d R_wf = data_->oMf[frame_id].rotation();
    const Eigen::Vector3d r_world = R_wf * com_local;

    Eigen::MatrixXd Jv_o = J_frame.topRows(3);
    Eigen::MatrixXd Jw   = J_frame.bottomRows(3);
    Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

    Eigen::Vector3d v_com = Jv_com * qdot_rad;

    // std::ostringstream oss;
    // oss << std::fixed << std::setprecision(6);
    // oss << "[J*qdot @ link6 BODY, LOCAL_WORLD_ALIGNED] "
    //     << "v_frame[m/s]=[" << v_frame.transpose() << "] | "
    //     << "w_frame[rad/s]=[" << w_frame.transpose() << "] | "
    //     << "v_com[m/s]=[" << v_com.transpose() << "]";
    // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  void timerCallback()
  {
    try {
      Eigen::VectorXd q_deg    = readQDegFromRobot();
      Eigen::VectorXd qdot_deg = readQdotDegFromRobot();

      Eigen::VectorXd q_rad    = degToRad(q_deg);
      Eigen::VectorXd qdot_rad = degToRad(qdot_deg);

      std::vector<double> meff_x;
      computeMeffWithGear(q_rad, meff_x);

      std::vector<Eigen::Vector3d> link_com_positions;
      std::vector<Eigen::Vector3d> link_com_velocities;
      link_com_positions.reserve(target_links_.size());
      link_com_velocities.reserve(target_links_.size());

      pinocchio::computeJointJacobians(model_, *data_, q_rad);
      pinocchio::updateFramePlacements(model_, *data_);

      for (const auto &link : target_links_) {
        const pinocchio::FrameIndex frame_id = link.frame_id;

        const auto &frame = model_.frames[frame_id];
        const pinocchio::JointIndex joint_id = frame.parentJoint;
        const auto &inertia = model_.inertias[joint_id];

        const Eigen::Vector3d com_local = inertia.lever();
        const Eigen::Matrix3d R_wf = data_->oMf[frame_id].rotation();
        const Eigen::Vector3d p_wf = data_->oMf[frame_id].translation();
        const Eigen::Vector3d r_world = R_wf * com_local;

        // Eigen::Vector3d p_com_world = p_wf + r_world;
        Eigen::Vector3d p_com_world = p_wf + r_world + robot_base_offset_world_;

        Eigen::MatrixXd J_frame(6, model_.nv);
        J_frame.setZero();

        pinocchio::getFrameJacobian(
            model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

        Eigen::MatrixXd Jv_o = J_frame.topRows(3);
        Eigen::MatrixXd Jw   = J_frame.bottomRows(3);
        Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

        Eigen::Vector3d v_com = Jv_com * qdot_rad;

        link_com_positions.push_back(p_com_world);
        link_com_velocities.push_back(v_com);
      }

      publishRobotDynamicsState(link_com_positions, link_com_velocities);

      // 계산은 10ms마다, 출력은 10번에 1번
      static int print_count = 0;
      if ((print_count++ % 100) == 0) {

        // std::ostringstream q_oss;
        // q_oss << std::fixed << std::setprecision(3);
        // q_oss << "[q_deg] ";
        // for (int i = 0; i < q_deg.size(); ++i) {
        //   q_oss << "q" << i << ": " << q_deg[i];
        //   if (i + 1 < q_deg.size()) q_oss << " | ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", q_oss.str().c_str());

        // std::ostringstream qdot_oss;
        // qdot_oss << std::fixed << std::setprecision(3);
        // qdot_oss << "[qdot_deg/s assumed] ";
        // for (int i = 0; i < qdot_deg.size(); ++i) {
        //   qdot_oss << "qdot" << i << ": " << qdot_deg[i];
        //   if (i + 1 < qdot_deg.size()) qdot_oss << " | ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", qdot_oss.str().c_str());

        if (link_com_positions.size() > 5) {
          const auto &p6 = link_com_positions[5];
          RCLCPP_INFO(
            this->get_logger(),
            "[link6 world CoM] x=%.3f, y=%.3f, z=%.3f",
            p6.x(), p6.y(), p6.z()
          );
        }
      
        // 유효질량
        // std::ostringstream meff_oss;
        // meff_oss << std::fixed << std::setprecision(3);
        // meff_oss << "[m_eff_with_gear +X] ";
        // for (size_t i = 0; i < target_links_.size(); ++i) {
        //   meff_oss << target_links_[i].name << ": " << meff_x[i];
        //   if (i + 1 < target_links_.size()) meff_oss << " | ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", meff_oss.str().c_str());

        debugFrameVelocity(q_rad, qdot_rad);
      }

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

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<LinkInfo> target_links_;

  Eigen::Matrix<double, 6, 1> gear_ratio_;
  Eigen::Matrix<double, 6, 1> motor_rotor_inertia_;
  Eigen::Matrix<double, 6, 1> lambda_diag_;

  std::unique_ptr<IndyDCP3> indy_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndyRobotMeffNode>());
  rclcpp::shutdown();
  return 0;
}