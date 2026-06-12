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
#include "hrc_interfaces/msg/directional_meff_query.hpp"
#include "hrc_interfaces/msg/directional_meff_result.hpp"
#include "indy_control_cpp/perf_logger.hpp"

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

    sub_robot_query_ =
      this->create_subscription<hrc_interfaces::msg::DirectionalMeffQuery>(
        "/indy/directional_meff_query",
        qos,
        std::bind(&IndyRobotMeffNode::robotQueryCallback, this, std::placeholders::_1));

    pub_robot_result_ =
      this->create_publisher<hrc_interfaces::msg::DirectionalMeffResult>(
        "/indy/directional_meff_result", qos);

    // ==========
    // real time  
    // ==========
        perf_update_logger_ = std::make_unique<PerfCsvLogger>(
      "/home/robotics/hrc_ws/analysis/realtime_perf/paper_robot_dynamics_perf.csv",
      std::vector<std::string>{
        "t_ros_sec",
        "read_robot_state_ms",
        "compute_meff_x_ms",
        "compute_link_state_ms",
        "publish_ms",
        "total_update_ms",
        "nominal_deadline_ms",
        "deadline_miss"
      },
      100
    );

    perf_query_logger_ = std::make_unique<PerfCsvLogger>(
      "/home/robotics/hrc_ws/analysis/realtime_perf/paper_robot_meff_query_perf.csv",
      std::vector<std::string>{
        "t_ros_sec",
        "query_id",
        "target_index",
        "query_compute_ms",
        "valid",
        "nominal_deadline_ms",
        "deadline_miss"
      },
      100
    );


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
  rclcpp::Subscription<hrc_interfaces::msg::DirectionalMeffQuery>::SharedPtr sub_robot_query_;
  rclcpp::Publisher<hrc_interfaces::msg::DirectionalMeffResult>::SharedPtr pub_robot_result_;

  // latest snapshot cache
  Eigen::VectorXd latest_q_rad_;
  Eigen::VectorXd latest_qdot_rad_;
  rclcpp::Time latest_state_stamp_{0, 0, RCL_ROS_TIME};
  bool has_latest_state_{false};
  
  Eigen::Vector3d robot_base_offset_world_{0.0, 0.0, 0.6};

  // realtime
  std::unique_ptr<PerfCsvLogger> perf_update_logger_;
  std::unique_ptr<PerfCsvLogger> perf_query_logger_;


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
    const auto t_start = PerfCsvLogger::now();

    try {
      const auto t_read0 = PerfCsvLogger::now();

      Eigen::VectorXd q_deg    = readQDegFromRobot();
      Eigen::VectorXd qdot_deg = readQdotDegFromRobot();

      const auto t_read1 = PerfCsvLogger::now();

      Eigen::VectorXd q_rad    = degToRad(q_deg);
      Eigen::VectorXd qdot_rad = degToRad(qdot_deg);

      latest_q_rad_ = q_rad;
      latest_qdot_rad_ = qdot_rad;
      latest_state_stamp_ = this->now();
      has_latest_state_ = true;

      const auto t_meff0 = PerfCsvLogger::now();

      std::vector<double> meff_x;
      computeMeffWithGear(q_rad, meff_x);

      const auto t_meff1 = PerfCsvLogger::now();

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

      const auto t_link1 = PerfCsvLogger::now();

      const auto t_pub0 = PerfCsvLogger::now();

      publishRobotDynamicsState(link_com_positions, link_com_velocities);

      const auto t_pub1 = PerfCsvLogger::now();

      const double total_ms = PerfCsvLogger::msBetween(t_start, t_pub1);
      const double deadline_ms = 10.0;

      if (perf_update_logger_) {
        perf_update_logger_->writeRow({
          PerfCsvLogger::toStr(this->now().seconds()),
          PerfCsvLogger::toStr(PerfCsvLogger::msBetween(t_read0, t_read1)),
          PerfCsvLogger::toStr(PerfCsvLogger::msBetween(t_meff0, t_meff1)),
          PerfCsvLogger::toStr(PerfCsvLogger::msBetween(t_meff1, t_link1)),
          PerfCsvLogger::toStr(PerfCsvLogger::msBetween(t_pub0, t_pub1)),
          PerfCsvLogger::toStr(total_ms),
          PerfCsvLogger::toStr(deadline_ms),
          std::to_string(static_cast<int>(total_ms > deadline_ms))
        });
      }

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

        // if (link_com_positions.size() > 5) {
        //   const auto &p6 = link_com_positions[5];
        //   RCLCPP_INFO(
        //     this->get_logger(),
        //     "[link6 world CoM] x=%.3f, y=%.3f, z=%.3f",
        //     p6.x(), p6.y(), p6.z()
        //   );
        // }
      
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

  double computeDirectionalMeffForTarget(
    int target_index,
    const Eigen::Vector3d &u_world,
    const Eigen::VectorXd &q_rad)
  {
    // 최신 q 기준으로 모델 업데이트
    pinocchio::computeJointJacobians(model_, *data_, q_rad);
    pinocchio::updateFramePlacements(model_, *data_);

    const auto &link = target_links_.at(static_cast<size_t>(target_index));
    const pinocchio::FrameIndex frame_id = link.frame_id;

    const auto &frame = model_.frames[frame_id];
    const pinocchio::JointIndex joint_id = frame.parentJoint;
    const auto &inertia = model_.inertias[joint_id];

    const Eigen::Vector3d com_local = inertia.lever();
    const Eigen::Matrix3d R_wf = data_->oMf[frame_id].rotation();
    const Eigen::Vector3d r_world = R_wf * com_local;

    Eigen::MatrixXd J_frame(6, model_.nv);
    J_frame.setZero();

    pinocchio::getFrameJacobian(
      model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // Pinocchio convention: [linear; angular]
    Eigen::MatrixXd Jv_o = J_frame.topRows(3);
    Eigen::MatrixXd Jw   = J_frame.bottomRows(3);

    // frame origin -> CoM point
    Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

    // 여기부터는 네 기존 로봇 meff 계산 로직에 맞춰 연결
    // 핵심은 directional scalar meff = 1 / (u^T Λ^{-1} u)
    //  gear 반영

    pinocchio::crba(model_, *data_, q_rad);
    data_->M.triangularView<Eigen::StrictlyLower>() =
        data_->M.transpose().triangularView<Eigen::StrictlyLower>();

    Eigen::MatrixXd M_aug = data_->M;
    M_aug.diagonal() += lambda_diag_;

    Eigen::LDLT<Eigen::MatrixXd> ldlt_aug(M_aug);

    Eigen::MatrixXd X_aug = ldlt_aug.solve(Jv_com.transpose());
    Eigen::Matrix3d A_aug = Jv_com * X_aug;

    double meff = effectiveMassAlong(A_aug, u_world);
    return meff;
  }

  void robotQueryCallback(
    const hrc_interfaces::msg::DirectionalMeffQuery::SharedPtr msg)
  {
    const auto t_query0 = PerfCsvLogger::now();
    bool query_valid = false;

    hrc_interfaces::msg::DirectionalMeffResult result;
    result.header.stamp = this->now();
    result.header.frame_id = "world";
    result.query_id = msg->query_id;
    result.target_index = msg->target_index;
    result.valid = false;
    result.directional_meff = 0.0f;

    if (!has_latest_state_) {
      RCLCPP_WARN(this->get_logger(),
        "Robot directional query received before state cache is ready");
      pub_robot_result_->publish(result);
      return;
    }

    const int target_index = static_cast<int>(msg->target_index);

    if (target_index < 0 || target_index >= static_cast<int>(target_links_.size())) {
      RCLCPP_WARN(this->get_logger(),
        "Invalid robot target_index: %d (target_links_.size=%zu)",
        target_index, target_links_.size());
      pub_robot_result_->publish(result);
      return;
    }

    Eigen::Vector3d u(
      msg->direction_u.x,
      msg->direction_u.y,
      msg->direction_u.z
    );

    const double norm = u.norm();
    if (norm < 1e-8) {
      RCLCPP_WARN(this->get_logger(),
        "Robot directional query has near-zero direction vector");
      pub_robot_result_->publish(result);
      return;
    }
    u /= norm;

    try {
      const double meff = computeDirectionalMeffForTarget(target_index, u, latest_q_rad_);

      result.valid = true;
      result.directional_meff = static_cast<float>(meff);
      const double query_ms = PerfCsvLogger::msSince(t_query0);
      result.compute_time_ms = static_cast<float>(query_ms);
      pub_robot_result_->publish(result);

      query_valid = true;

      const double deadline_ms = 150.0;

      if (perf_query_logger_) {
        perf_query_logger_->writeRow({
          PerfCsvLogger::toStr(this->now().seconds()),
          std::to_string(msg->query_id),
          std::to_string(msg->target_index),
          PerfCsvLogger::toStr(query_ms),
          std::to_string(static_cast<int>(query_valid)),
          PerfCsvLogger::toStr(deadline_ms),
          std::to_string(static_cast<int>(query_ms > deadline_ms))
        });
      }

      // RCLCPP_INFO(this->get_logger(),
      //   "[robot meff result] qid=%lu target=%d u=(%.3f, %.3f, %.3f) meff=%.3f",
      //   msg->query_id, target_index, u.x(), u.y(), u.z(), meff);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed robot directional meff: qid=%lu target=%d err=%s",
        msg->query_id, target_index, e.what());
      const double query_ms = PerfCsvLogger::msSince(t_query0);
      result.compute_time_ms = static_cast<float>(query_ms);
      pub_robot_result_->publish(result);

      query_valid = true;

      const double deadline_ms = 150.0;

      if (perf_query_logger_) {
        perf_query_logger_->writeRow({
          PerfCsvLogger::toStr(this->now().seconds()),
          std::to_string(msg->query_id),
          std::to_string(msg->target_index),
          PerfCsvLogger::toStr(query_ms),
          std::to_string(static_cast<int>(query_valid)),
          PerfCsvLogger::toStr(deadline_ms),
          std::to_string(static_cast<int>(query_ms > deadline_ms))
        });
      }
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
