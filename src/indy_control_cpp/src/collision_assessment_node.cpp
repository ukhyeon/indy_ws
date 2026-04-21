#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <hrc_interfaces/msg/human_dynamics_state.hpp>
#include <hrc_interfaces/msg/robot_dynamics_state.hpp>
#include <hrc_interfaces/msg/directional_meff_query.hpp>
#include <hrc_interfaces/msg/directional_meff_result.hpp>
#include <hrc_interfaces/msg/collision_state.hpp>
#include "hrc_interfaces/msg/collision_candidates.hpp"
#include <hrc_interfaces/msg/demo_state.hpp>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
using json = nlohmann::json;

class CollisionAssessmentNode : public rclcpp::Node
{
public:
  CollisionAssessmentNode()
  : Node("collision_assessment_node")
  {
    // =========================
    // Parameters
    // =========================
    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 10);
    query_timeout_ms_ = this->declare_parameter<int>("query_timeout_ms", 150);

    iso_json_path_ = this->declare_parameter<std::string>(
      "iso_json_path",
      "/home/robotics/indy_ws/src/indy_control_cpp/ISO15066/iso_15066.json");

    alpha_up_ = this->declare_parameter<double>("alpha_up", 0.10);       // speed up slowly
    alpha_down_ = this->declare_parameter<double>("alpha_down", 0.35);   // slow down faster

    min_speed_scale_ = this->declare_parameter<double>("min_speed_scale", 0.05);
    max_speed_scale_ = this->declare_parameter<double>("max_speed_scale", 1.0);

    if (!loadIsoJson(iso_json_path_)) {
      throw std::runtime_error("Failed to load ISO JSON: " + iso_json_path_);
    }

    //생성자
    initHumanBodyIndexMap();
    initRobotLinkIndexMap();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    // =========================
    // Subscribers
    // =========================
    sub_human_state_ = this->create_subscription<hrc_interfaces::msg::HumanDynamicsState>(
      "/hrc/human_dynamics_state", qos,
      std::bind(&CollisionAssessmentNode::humanStateCallback, this, std::placeholders::_1));

    sub_robot_state_ = this->create_subscription<hrc_interfaces::msg::RobotDynamicsState>(
      "/indy/robot_dynamics_state", qos,
      std::bind(&CollisionAssessmentNode::robotStateCallback, this, std::placeholders::_1));

    sub_human_result_ = this->create_subscription<hrc_interfaces::msg::DirectionalMeffResult>(
      "/hrc/directional_meff_result", qos,
      std::bind(&CollisionAssessmentNode::humanResultCallback, this, std::placeholders::_1));

    sub_robot_result_ = this->create_subscription<hrc_interfaces::msg::DirectionalMeffResult>(
      "/indy/directional_meff_result", qos,
      std::bind(&CollisionAssessmentNode::robotResultCallback, this, std::placeholders::_1));

    // =========================
    // Publishers
    // =========================
    pub_human_query_ = this->create_publisher<hrc_interfaces::msg::DirectionalMeffQuery>(
      "/hrc/directional_meff_query", qos);

    pub_robot_query_ = this->create_publisher<hrc_interfaces::msg::DirectionalMeffQuery>(
      "/indy/directional_meff_query", qos);

    pub_speed_scale_ = this->create_publisher<std_msgs::msg::Float32>(
      "/indy/speed_scale", 10);

    pub_collision_state_ = this->create_publisher<hrc_interfaces::msg::CollisionState>(
      "/collision_state", 10);

    pub_collision_candidates_ =
    this->create_publisher<hrc_interfaces::msg::CollisionCandidates>(
      "/hrc/collision_candidates",
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort()
    );

    pub_demo_state_ = this->create_publisher<hrc_interfaces::msg::DemoState>(
      "/hrc/demo_state", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&CollisionAssessmentNode::timerCallback, this));

    demo_log_path_ = "/home/robotics/hrc_ws/analysis/demo_log/demo_metrics.csv";
    demo_log_ofs_.open(demo_log_path_, std::ios::out | std::ios::trunc);
    
    demo_start_time_ = this->now();
    demo_time_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "collision_assessment_node started");
    RCLCPP_INFO(this->get_logger(), "Loaded ISO JSON: %s", iso_json_path_.c_str());
  }

private:
  struct DemoStateSnapshot
  {
    rclcpp::Time stamp;
    double t_demo_sec{0.0};

    uint32_t selected_human_index{0};
    uint32_t selected_robot_index{0};

    std::string selected_body_name;
    std::string selected_link_name;

    double distance{0.0};
    Eigen::Vector3d direction_u{1.0, 0.0, 0.0};

    double relative_speed_along_u{0.0};
    double robot_speed_along_u_raw{0.0};
    double approach_bias_added{0.0};

    double human_meff{0.0};       // directional meff
    double human_meff_iso{0.0};   // iso fixed m_h_iso
    double robot_meff{0.0};

    double reduced_mass{0.0};     // directional
    double reduced_mass_iso{0.0}; // iso fixed

    double iso_k{0.0};
    double v_allow_dir{0.0};
    double v_allow_iso{0.0};

    double target_speed_scale{1.0};
    double applied_speed_scale{1.0};

    double estimated_collision_force_dir{0.0};
    double estimated_collision_force_iso{0.0};

    double threshold_force{0.0};
    double force_ratio_dir{0.0};
    double force_ratio_iso{0.0};

    std::vector<double> link_speed_magnitudes;
  };


  struct IsoLimit
  {
    double F_max{0.0};    // N
    double m_h_iso{0.0};  // not used in online control, but kept for reference
    double k{0.0};        // kN/m in your existing code convention
  };

  struct DynamicsSnapshot
  {
    rclcpp::Time stamp;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    bool valid{false};
  };

  struct PendingCandidateQuery
  {
    uint64_t query_id{0};

    uint32_t human_index{0};
    uint32_t robot_index{0};

    Eigen::Vector3d u{1.0, 0.0, 0.0};   // robot -> human
    double distance{0.0};

    bool human_result_ready{false};
    bool robot_result_ready{false};

    double human_meff{0.0};
    double robot_meff{0.0};

    rclcpp::Time sent_time;
    rclcpp::Time human_result_time;
    rclcpp::Time robot_result_time;
  };

  struct PendingBatchQuery
  {
    uint64_t batch_id{0};
    rclcpp::Time batch_sent_time;
    std::vector<PendingCandidateQuery> candidates;   // size <= 3
  };

  struct CollisionCandidate
  {
    uint32_t human_index;
    uint32_t robot_index;
    double distance;
    Eigen::Vector3d u_robot_to_human;
  };

private:
  static bool parseFlatXYZ(
    const std::vector<float> &flat,
    std::vector<Eigen::Vector3d> &out)
  {
    if (flat.empty() || (flat.size() % 3) != 0) {
      return false;
    }

    const size_t n = flat.size() / 3;
    out.clear();
    out.reserve(n);

    for (size_t i = 0; i < n; ++i) {
      out.emplace_back(
        static_cast<double>(flat[3 * i + 0]),
        static_cast<double>(flat[3 * i + 1]),
        static_cast<double>(flat[3 * i + 2]));
    }
    return true;
  }

  static geometry_msgs::msg::Vector3 toRosVec(const Eigen::Vector3d &v)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
  }

  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  static double normSafe(const Eigen::Vector3d &v, double eps = 1e-9)
  {
    return std::max(v.norm(), eps);
  }

  static double computeReducedMass(double m_h, double m_r)
  {
    const double eps = 1e-9;
    m_h = std::max(m_h, eps);
    m_r = std::max(m_r, eps);
    return 1.0 / (1.0 / m_h + 1.0 / m_r);
  }

  static double computeVmax(double F_max, double mu, double k_kN_per_m)
  {
    // 기존 Python 코드와 동일
    // k [kN/m] -> [N/m]
    const double k = k_kN_per_m * 1e3;
    const double denom = std::sqrt(std::max(mu * k, 1e-9));
    return F_max / denom;
  }

  bool loadIsoJson(const std::string &path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open ISO JSON: %s", path.c_str());
      return false;
    }

    json j;
    try {
      ifs >> j;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
      return false;
    }

    iso_limits_.clear();

    for (auto it = j.begin(); it != j.end(); ++it) {
      const std::string part = it.key();
      const auto &obj = it.value();

      if (!obj.contains("F_max") || !obj.contains("m_h_iso") || !obj.contains("k")) {
        RCLCPP_ERROR(this->get_logger(), "Missing required fields in ISO JSON for part: %s", part.c_str());
        return false;
      }

      IsoLimit limit;
      limit.F_max = obj.at("F_max").get<double>();
      limit.m_h_iso = obj.at("m_h_iso").get<double>();
      limit.k = obj.at("k").get<double>();

      iso_limits_[part] = limit;
    }

    return true;
  }

  void initHumanBodyIndexMap()
  {
    // SMPL 19-part 기준으로 맞춘 매핑
    // 필요하면 네 실제 body index 정의에 맞게 수정
    human_index_to_part_name_.clear();
    human_index_to_part_name_[0]  = "pelvis";
    human_index_to_part_name_[1]  = "left_thigh";
    human_index_to_part_name_[2]  = "right_thigh";
    human_index_to_part_name_[3]  = "abdomen";
    human_index_to_part_name_[4]  = "left_calf";
    human_index_to_part_name_[5]  = "right_calf";
    human_index_to_part_name_[6]  = "left_foot";        
    human_index_to_part_name_[7]  = "right_foot";       
    human_index_to_part_name_[8]  = "chest";
    human_index_to_part_name_[9]  = "neck";
    human_index_to_part_name_[10] = "left_shoulder";
    human_index_to_part_name_[11] = "right_shoulder";
    human_index_to_part_name_[12] = "head";
    human_index_to_part_name_[13] = "left_upper_arm";
    human_index_to_part_name_[14] = "right_upper_arm";
    human_index_to_part_name_[15] = "left_lower_arm";
    human_index_to_part_name_[16] = "right_lower_arm";
    human_index_to_part_name_[17] = "left_hand";
    human_index_to_part_name_[18] = "right_hand";
  }


  // 충돌 예상 부위와 방향을 토픽으로 날리고 유효질량을 받음
  void publishDirectionalQueries(
    uint64_t query_id,
    uint32_t human_index,
    uint32_t robot_index,
    const Eigen::Vector3d &u_robot_to_human)
  {
    auto now_msg = this->now();

    hrc_interfaces::msg::DirectionalMeffQuery human_q;
    human_q.header.stamp = now_msg;
    human_q.header.frame_id = "world";
    human_q.query_id = query_id;
    human_q.target_index = human_index;
    human_q.direction_u = toRosVec(u_robot_to_human); // human 은 그대로 u
    pub_human_query_->publish(human_q);

    hrc_interfaces::msg::DirectionalMeffQuery robot_q;
    robot_q.header.stamp = now_msg;
    robot_q.header.frame_id = "world";
    robot_q.query_id = query_id;
    robot_q.target_index = robot_index;
    robot_q.direction_u = toRosVec(-u_robot_to_human); // robot 은 -u
    pub_robot_query_->publish(robot_q);

    // 충돌 방향 logger
    RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[query] qid=%lu | h=%u r=%u | u_h=[%.3f %.3f %.3f] | u_r=[%.3f %.3f %.3f]",
    query_id,
    human_index, robot_index,
    u_robot_to_human.x(), u_robot_to_human.y(), u_robot_to_human.z(),
    -u_robot_to_human.x(), -u_robot_to_human.y(), -u_robot_to_human.z());
  }

  // double computeRelativeSpeedAlongU(
  //   uint32_t human_index,
  //   uint32_t robot_index,
  //   const Eigen::Vector3d &u_robot_to_human) const
  // {
  //   if (human_index >= human_state_.velocities.size() ||
  //       robot_index >= robot_state_.velocities.size()) {
  //     return 0.0;
  //   }

  //   // const Eigen::Vector3d v_h = human_state_.velocities[human_index];
  //   const Eigen::Vector3d v_r = robot_state_.velocities[robot_index];

  //   // 사람은 항상 로봇 방향으로 0.3 m/s로 접근한다고 가정
  //   const Eigen::Vector3d v_h = -0.3 * u_robot_to_human.normalized();

  //   // positive -> robot approaching human along u
  //   // return (v_r - v_h).dot(u_robot_to_human);

  //   return v_r.dot(u_robot_to_human); // 임시 디버그, 사람의 속도 0
  // }

  double computeRelativeSpeedAlongU(
    uint32_t human_index,
    uint32_t robot_index,
    const Eigen::Vector3d &u_robot_to_human) const
  {
    if (human_index >= human_state_.velocities.size() ||
        robot_index >= robot_state_.velocities.size()) {
      return 0.0;
    }

    const Eigen::Vector3d u = u_robot_to_human.normalized();
    const Eigen::Vector3d v_r = robot_state_.velocities[robot_index];

    const double vr_along_u = v_r.dot(u);

    // 로봇이 멀어지는 중이면 위험 접근속도 0
    if (vr_along_u <= 0.0) {
      return 0.0;
    }

    // 로봇이 접근할 때만 사람 접근속도 0.3 m/s를 보수적으로 더함
    return vr_along_u + 0.3;
  }

  std::optional<IsoLimit> getIsoLimitForHumanIndex(uint32_t human_index) const
  {
    auto it_name = human_index_to_part_name_.find(human_index);
    if (it_name == human_index_to_part_name_.end()) {
      return std::nullopt;
    }

    auto it_limit = iso_limits_.find(it_name->second);
    if (it_limit == iso_limits_.end()) {
      return std::nullopt;
    }

    return it_limit->second;
  }

  double computeTargetSpeedScale(
    double relative_speed_along_u,
    double human_meff,
    double robot_meff,
    const IsoLimit &iso_limit) const
  {
    // 접근 중이 아니면 full speed 허용
    if (relative_speed_along_u <= 0.0) {
      return max_speed_scale_;
    }

    const double mu = computeReducedMass(human_meff, robot_meff);
    const double v_allow = computeVmax(iso_limit.F_max, mu, iso_limit.k);

    // 현재 접근 속도가 허용속도보다 작으면 scale을 높이는 방향
    // 크면 낮추는 방향
    const double ratio = v_allow / std::max(relative_speed_along_u, 1e-6);

    // 1.0보다 크면 여유 있다는 뜻
    return clamp(ratio, min_speed_scale_, max_speed_scale_);
  }

  double computeEstimatedCollisionForce(
    double relative_speed_along_u,
    double reduced_mass,
    double k_kN_per_m) const
  {
    if (relative_speed_along_u <= 0.0) {
      return 0.0;
    }

    const double k = k_kN_per_m * 1e3; // N/m
    return relative_speed_along_u * std::sqrt(std::max(reduced_mass * k, 1e-9));
  }

  double applyForceLimitGuard(
    double target_speed_scale,
    double estimated_collision_force,
    double threshold_force) const
  {
    const double eps = 1e-6;
    const double ratio = estimated_collision_force / std::max(threshold_force, eps);

    double guarded_scale = target_speed_scale;

    if (ratio > 1.30) {
      guarded_scale = std::min(guarded_scale, 0.05);
    } else if (ratio > 1.15) {
      guarded_scale = std::min(guarded_scale, 0.10);
    } else if (ratio > 1.05) {
      guarded_scale = std::min(guarded_scale, 0.20);
    } else if (ratio > 1.00) {
      guarded_scale = std::min(guarded_scale, 0.35);
    }

    return clamp(guarded_scale, min_speed_scale_, max_speed_scale_);
  }

  // 허용 속도
  // 1차 저역 통과 필터, 현재 값과 목표 값 차이를 보고 
  double updateSpeedScaleSmooth(double target_speed_scale)
  {
    const double alpha = (target_speed_scale < current_speed_scale_) ? alpha_down_ : alpha_up_; // 감속은 빠르게, 가속은 천천히
    current_speed_scale_ =
      current_speed_scale_ + alpha * (target_speed_scale - current_speed_scale_);

    current_speed_scale_ = clamp(current_speed_scale_, min_speed_scale_, max_speed_scale_);
    return current_speed_scale_;
  }

  std::vector<double> computeRobotLinkSpeedMagnitudes() const
  {
    std::vector<double> out;
    out.reserve(robot_state_.velocities.size());

    for (const auto &v : robot_state_.velocities) {
      out.push_back(v.norm());
    }
    return out;
  }

  void publishCollisionState(
    const PendingCandidateQuery &cand,
    double rel_speed,
    double speed_scale)
  {
    hrc_interfaces::msg::CollisionState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";

    msg.human_index = cand.human_index;
    msg.robot_index = cand.robot_index;
    msg.distance = static_cast<float>(cand.distance);
    msg.direction_u = toRosVec(cand.u);
    msg.relative_speed_along_u = static_cast<float>(rel_speed);
    msg.human_meff = static_cast<float>(cand.human_meff);
    msg.robot_meff = static_cast<float>(cand.robot_meff);
    msg.speed_scale = static_cast<float>(speed_scale);

    pub_collision_state_->publish(msg);
  }

  void publishDemoState(const DemoStateSnapshot &snap)
  {
    hrc_interfaces::msg::DemoState msg;
    msg.header.stamp = snap.stamp;
    msg.header.frame_id = "world";

    msg.t_demo_sec = static_cast<float>(snap.t_demo_sec);

    msg.selected_human_index = snap.selected_human_index;
    msg.selected_robot_index = snap.selected_robot_index;

    msg.selected_body_name = snap.selected_body_name;
    msg.selected_link_name = snap.selected_link_name;

    msg.distance = static_cast<float>(snap.distance);
    msg.direction_u = toRosVec(snap.direction_u);

    msg.relative_speed_along_u = static_cast<float>(snap.relative_speed_along_u);
    msg.human_meff = static_cast<float>(snap.human_meff);
    msg.robot_meff = static_cast<float>(snap.robot_meff);
    msg.reduced_mass = static_cast<float>(snap.reduced_mass);

    msg.target_speed_scale = static_cast<float>(snap.target_speed_scale);
    msg.applied_speed_scale = static_cast<float>(snap.applied_speed_scale);

    msg.estimated_collision_force = static_cast<float>(snap.estimated_collision_force_dir);
    msg.threshold_force = static_cast<float>(snap.threshold_force);

    for (double v : snap.link_speed_magnitudes) {
      msg.link_speed_magnitudes.push_back(static_cast<float>(v));
    }

    pub_demo_state_->publish(msg);
  }

  void publishSpeedScale(double scale)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(clamp(scale, min_speed_scale_, max_speed_scale_));
    pub_speed_scale_->publish(msg);
  }

  void writeDemoLogHeaderIfNeeded()
  {
    if (demo_log_header_written_ || !demo_log_ofs_.is_open()) return;

    demo_log_ofs_
      << "t_demo_sec,"
      << "selected_human_index,selected_robot_index,"
      << "selected_link,selected_body,"
      << "u_x,u_y,u_z,"
      << "distance,"
      << "robot_speed_along_u_raw,"
      << "approach_bias_added,"
      << "relative_speed_along_u,"
      << "human_meff_dir,human_meff_iso,robot_meff,"
      << "reduced_mass_dir,reduced_mass_iso,"
      << "iso_k,"
      << "v_allow_dir,v_allow_iso,"
      << "speed_scale_target,speed_scale_applied,"
      << "estimated_collision_force_dir,estimated_collision_force_iso,"
      << "threshold_force,"
      << "force_ratio_dir,force_ratio_iso,"
      << "link1_speed,link2_speed,link3_speed,link4_speed,link5_speed,link6_speed\n";

    demo_log_header_written_ = true;
  }

  void appendDemoLog(const DemoStateSnapshot &snap)
  {
    if (!demo_log_ofs_.is_open()) return;
    writeDemoLogHeaderIfNeeded();

    demo_log_ofs_
      << snap.t_demo_sec << ","
      << snap.selected_human_index << ","
      << snap.selected_robot_index << ","
      << snap.selected_link_name << ","
      << snap.selected_body_name << ","
      << snap.direction_u.x() << ","
      << snap.direction_u.y() << ","
      << snap.direction_u.z() << ","
      << snap.distance << ","
      << snap.robot_speed_along_u_raw << ","
      << snap.approach_bias_added << ","
      << snap.relative_speed_along_u << ","
      << snap.human_meff << ","
      << snap.human_meff_iso << ","
      << snap.robot_meff << ","
      << snap.reduced_mass << ","
      << snap.reduced_mass_iso << ","
      << snap.iso_k << ","
      << snap.v_allow_dir << ","
      << snap.v_allow_iso << ","
      << snap.target_speed_scale << ","
      << snap.applied_speed_scale << ","
      << snap.estimated_collision_force_dir << ","
      << snap.estimated_collision_force_iso << ","
      << snap.threshold_force << ","
      << snap.force_ratio_dir << ","
      << snap.force_ratio_iso;

    for (size_t i = 0; i < 6; ++i) {
      double v = (i < snap.link_speed_magnitudes.size()) ? snap.link_speed_magnitudes[i] : 0.0;
      demo_log_ofs_ << "," << v;
    }
    demo_log_ofs_ << "\n";
  }
  
  // batch 안에서 candidate 찾기
  PendingCandidateQuery* findPendingCandidate(uint64_t query_id)
  {
    if (!pending_query_.has_value()) {
      return nullptr;
    }

    auto &batch = pending_query_.value();
    for (auto &cand : batch.candidates) {
      if (cand.query_id == query_id) {
        return &cand;
      }
    }
    return nullptr;
  }

  //batch 완료 여부 확인
  bool batchAllResultsReady(const PendingBatchQuery &batch) const
  {
    if (batch.candidates.empty()) {
      return false;
    }

    for (const auto &cand : batch.candidates) {
      if (!(cand.human_result_ready && cand.robot_result_ready)) {
        return false;
      }
    }
    return true;
  }
  // batch timeout 확인
  bool pendingExpired(const PendingBatchQuery &batch) const
  {
    const auto dt_ms = (this->now() - batch.batch_sent_time).nanoseconds() / 1e6;
    return dt_ms > static_cast<double>(query_timeout_ms_);
  }

  // 가장 보수적인 candidate 선택 helper
  const PendingCandidateQuery* selectMostConservativeCandidate(
  const PendingBatchQuery &batch,
  std::vector<double> &target_scales,
  std::vector<double> &rel_speeds)
  {
    target_scales.clear();
    rel_speeds.clear();

    if (batch.candidates.empty()) {
      return nullptr;
    }

    const PendingCandidateQuery *best = nullptr;
    double best_target_scale = std::numeric_limits<double>::infinity();

    for (const auto &cand : batch.candidates) {
      const double rel_speed = computeRelativeSpeedAlongU(
        cand.human_index, cand.robot_index, cand.u);

      auto iso_opt = getIsoLimitForHumanIndex(cand.human_index);
      if (!iso_opt.has_value()) {
        return nullptr;  // caller에서 에러 처리
      }

      const double target_scale = computeTargetSpeedScale(
        rel_speed,
        cand.human_meff,
        cand.robot_meff,
        iso_opt.value()
      );

      rel_speeds.push_back(rel_speed);
      target_scales.push_back(target_scale);

      if (target_scale < best_target_scale) {
        best_target_scale = target_scale;
        best = &cand;
      }
    }

    return best;
  }


private:

  void initRobotLinkIndexMap()
  {
    robot_index_to_link_name_.clear();
    robot_index_to_link_name_[0] = "link1";
    robot_index_to_link_name_[1] = "link2";
    robot_index_to_link_name_[2] = "link3";
    robot_index_to_link_name_[3] = "link4";
    robot_index_to_link_name_[4] = "link5";
    robot_index_to_link_name_[5] = "link6";
  }

  // 충돌 제외 부위
  bool isValidRobotCollisionIndex(uint32_t robot_index) const
  {
    // link0, link1 제외
    return !(robot_index == 0 || robot_index == 1);
  }

  bool isValidHumanCollisionIndex(uint32_t human_index) const
  {
    // left_foot(6), right_foot(7) 제외
    return !(human_index == 6 || human_index == 7);
  }

  bool computeTopKCollisionCandidates(
    std::vector<CollisionCandidate> &top_candidates,
    size_t k = 3)
  {
    top_candidates.clear();

    const size_t human_n = human_state_.positions.size();
    const size_t robot_n = robot_state_.positions.size();

    if (!human_state_.valid || !robot_state_.valid || human_n == 0 || robot_n == 0) {
      return false;
    }

    std::vector<CollisionCandidate> all_candidates;
    all_candidates.reserve(human_n);

    for (size_t hi = 0; hi < human_n; ++hi) {
      const uint32_t human_index = static_cast<uint32_t>(hi);

      // ===== human 후보 필터 =====
      if (!isValidHumanCollisionIndex(human_index)) {
        continue;
      }

      double best_dist = std::numeric_limits<double>::infinity();
      uint32_t best_robot = 0;
      Eigen::Vector3d best_u = Eigen::Vector3d::UnitX();
      bool found_valid_robot = false;

      const Eigen::Vector3d &ph = human_state_.positions[hi];

      for (size_t ri = 0; ri < robot_n; ++ri) {
        const uint32_t robot_index = static_cast<uint32_t>(ri);

        // ===== robot 후보 필터 =====
        if (!isValidRobotCollisionIndex(robot_index)) {
          continue;
        }

        const Eigen::Vector3d &pr = robot_state_.positions[ri];
        Eigen::Vector3d d = ph - pr;
        const double dist = d.norm();

        if (dist < best_dist) {
          best_dist = dist;
          best_robot = robot_index;
          found_valid_robot = true;

          if (dist > 1e-9) {
            best_u = d / dist;   // robot -> human
          } else {
            best_u = Eigen::Vector3d::UnitX();
          }
        }
      }

      // 유효한 robot 후보가 하나도 없으면 skip
      if (!found_valid_robot) {
        continue;
      }

      CollisionCandidate c;
      c.human_index = human_index;
      c.robot_index = best_robot;
      c.distance = best_dist;
      c.u_robot_to_human = best_u;
      all_candidates.push_back(c);
    }

    if (all_candidates.empty()) {
      return false;
    }

    std::sort(
      all_candidates.begin(),
      all_candidates.end(),
      [](const CollisionCandidate &a, const CollisionCandidate &b) {
        return a.distance < b.distance;
      });

    const size_t out_n = std::min(k, all_candidates.size());
    top_candidates.assign(all_candidates.begin(), all_candidates.begin() + out_n);
    return !top_candidates.empty();
  }

  void publishRiskOrderedCollisionCandidates(
    const PendingBatchQuery &batch,
    const std::vector<double> &target_scales)
  {
    if (batch.candidates.empty() || batch.candidates.size() != target_scales.size()) {
      return;
    }

    struct RankedItem
    {
      size_t idx;
      double target_scale;
    };

    std::vector<RankedItem> ranked;
    ranked.reserve(batch.candidates.size());

    for (size_t i = 0; i < batch.candidates.size(); ++i) {
      ranked.push_back({i, target_scales[i]});
    }

    // target_scale가 작을수록 더 위험
    std::sort(
      ranked.begin(),
      ranked.end(),
      [](const RankedItem &a, const RankedItem &b) {
        return a.target_scale < b.target_scale;
      });

    hrc_interfaces::msg::CollisionCandidates msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";

    for (const auto &r : ranked) {
      const auto &cand = batch.candidates[r.idx];

      msg.human_indices.push_back(cand.human_index);
      msg.human_names.push_back(human_index_to_part_name_[cand.human_index]);

      msg.robot_indices.push_back(cand.robot_index);

      auto it = robot_index_to_link_name_.find(cand.robot_index);
      if (it != robot_index_to_link_name_.end()) {
        msg.robot_names.push_back(it->second);
      } else {
        msg.robot_names.push_back("unknown");
      }

      msg.distances.push_back(static_cast<float>(cand.distance));
      msg.directions.push_back(toRosVec(cand.u));
    }

    pub_collision_candidates_->publish(msg);
  }

  void publishCollisionCandidates(const std::vector<CollisionCandidate> &cands)
  {
    hrc_interfaces::msg::CollisionCandidates msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";

    for (const auto &c : cands) {
      msg.human_indices.push_back(c.human_index);
      msg.human_names.push_back(human_index_to_part_name_[c.human_index]);

      msg.robot_indices.push_back(c.robot_index);

      auto it = robot_index_to_link_name_.find(c.robot_index);
      if (it != robot_index_to_link_name_.end()) {
        msg.robot_names.push_back(it->second);
      } else {
        msg.robot_names.push_back("unknown");
      }

      msg.distances.push_back(static_cast<float>(c.distance));
      msg.directions.push_back(toRosVec(c.u_robot_to_human));
    }

    pub_collision_candidates_->publish(msg);

  }

  void publishEmptyCollisionCandidates()
  {
    hrc_interfaces::msg::CollisionCandidates msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    pub_collision_candidates_->publish(msg);
  }




  void humanStateCallback(const hrc_interfaces::msg::HumanDynamicsState::SharedPtr msg)
  {
    std::vector<Eigen::Vector3d> pos, vel;

    if (!parseFlatXYZ(msg->body_part_positions, pos)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Invalid human body_part_positions");
      return;
    }

    if (!parseFlatXYZ(msg->body_part_velocities, vel)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Invalid human body_part_velocities");
      return;
    }

    if (pos.size() != vel.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Human state size mismatch");
      return;
    }

    human_state_.positions = std::move(pos);
    human_state_.velocities = std::move(vel);
    human_state_.stamp = msg->header.stamp;
    human_state_.valid = true;
  }

  void robotStateCallback(const hrc_interfaces::msg::RobotDynamicsState::SharedPtr msg)
  {
    std::vector<Eigen::Vector3d> pos, vel;

    if (!parseFlatXYZ(msg->link_positions, pos)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Invalid robot link_positions");
      return;
    }

    if (!parseFlatXYZ(msg->link_velocities, vel)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Invalid robot link_velocities");
      return;
    }

    if (pos.size() != vel.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Robot state size mismatch");
      return;
    }

    robot_state_.positions = std::move(pos);
    robot_state_.velocities = std::move(vel);
    robot_state_.stamp = msg->header.stamp;
    robot_state_.valid = true;
  }

  void humanResultCallback(const hrc_interfaces::msg::DirectionalMeffResult::SharedPtr msg)
  {
    if (!pending_query_.has_value()) {
      return;
    }

    auto *cand = findPendingCandidate(msg->query_id);
    if (cand == nullptr) {
      return;
    }

    if (!msg->valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "Human meff result invalid for query_id=%lu",
        msg->query_id
      );
      return;
    }

    cand->human_meff = msg->directional_meff;
    cand->human_result_ready = true;
    cand->human_result_time = this->now();
  }

  void robotResultCallback(const hrc_interfaces::msg::DirectionalMeffResult::SharedPtr msg)
  {
    if (!pending_query_.has_value()) {
      return;
    }

    auto *cand = findPendingCandidate(msg->query_id);
    if (cand == nullptr) {
      return;
    }

    if (!msg->valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "Robot meff result invalid for query_id=%lu",
        msg->query_id
      );
      return;
    }

    cand->robot_meff = msg->directional_meff;
    cand->robot_result_ready = true;
    cand->robot_result_time = this->now();
  }

  void timerCallback()
  {
    if (!human_state_.valid || !robot_state_.valid) {
      return;
    }

    bool should_publish_speed = false;
    double speed_to_publish = current_speed_scale_;

    // 1) pending batch 처리
    if (pending_query_.has_value()) {
      auto &batch = pending_query_.value();

      if (batchAllResultsReady(batch)) {
        std::vector<double> target_scales;
        std::vector<double> rel_speeds;
        const auto *best_cand = selectMostConservativeCandidate(
          batch, target_scales, rel_speeds);

        if (best_cand == nullptr) {
          speed_to_publish = updateSpeedScaleSmooth(min_speed_scale_);
          should_publish_speed = true;
          pending_query_.reset();
        }
        else {
          publishRiskOrderedCollisionCandidates(batch, target_scales);
          size_t best_idx = 0;
          double best_target_scale = std::numeric_limits<double>::infinity();

          for (size_t i = 0; i < batch.candidates.size(); ++i) {
            if (batch.candidates[i].query_id == best_cand->query_id) {
              best_idx = i;
              best_target_scale = target_scales[i];
              break;
            }
          }

          auto iso_opt = getIsoLimitForHumanIndex(best_cand->human_index);
          if (!iso_opt.has_value()) {
            speed_to_publish = updateSpeedScaleSmooth(min_speed_scale_);
            should_publish_speed = true;
            pending_query_.reset();
            publishSpeedScale(speed_to_publish);
            return;
          }

          DemoStateSnapshot snap;
          snap.stamp = this->now();
          snap.t_demo_sec = (snap.stamp - demo_start_time_).seconds();

          snap.selected_human_index = best_cand->human_index;
          snap.selected_robot_index = best_cand->robot_index;
          snap.selected_body_name = human_index_to_part_name_[best_cand->human_index];
          snap.selected_link_name = robot_index_to_link_name_[best_cand->robot_index];
          snap.distance = best_cand->distance;
          //

          snap.direction_u = best_cand->u;
          snap.relative_speed_along_u = rel_speeds[best_idx];

          const Eigen::Vector3d u_norm = best_cand->u.normalized();
          const Eigen::Vector3d v_r = robot_state_.velocities[best_cand->robot_index];
          const double vr_along_u = v_r.dot(u_norm);

          snap.robot_speed_along_u_raw = vr_along_u;
          snap.approach_bias_added = std::max(0.0, snap.relative_speed_along_u - vr_along_u);

          snap.human_meff = best_cand->human_meff;               // directional meff
          snap.human_meff_iso = iso_opt->m_h_iso;                // fixed ISO mass
          snap.robot_meff = best_cand->robot_meff;

          snap.reduced_mass = computeReducedMass(snap.human_meff, snap.robot_meff);
          snap.reduced_mass_iso = computeReducedMass(snap.human_meff_iso, snap.robot_meff);

          snap.iso_k = iso_opt->k;
          snap.v_allow_dir = computeVmax(
            iso_opt->F_max,
            snap.reduced_mass,
            iso_opt->k
          );
          snap.v_allow_iso = computeVmax(
            iso_opt->F_max,
            snap.reduced_mass_iso,
            iso_opt->k
          );

          snap.target_speed_scale = best_target_scale;
          snap.threshold_force = iso_opt->F_max;

          snap.estimated_collision_force_dir = computeEstimatedCollisionForce(
            snap.relative_speed_along_u,
            snap.reduced_mass,
            iso_opt->k
          );

          snap.estimated_collision_force_iso = computeEstimatedCollisionForce(
            snap.relative_speed_along_u,
            snap.reduced_mass_iso,
            iso_opt->k
          );

          snap.force_ratio_dir =
            snap.threshold_force > 1e-6
              ? snap.estimated_collision_force_dir / snap.threshold_force
              : 0.0;

          snap.force_ratio_iso =
            snap.threshold_force > 1e-6
              ? snap.estimated_collision_force_iso / snap.threshold_force
              : 0.0;
          //


          const double guarded_target_scale = applyForceLimitGuard(
            best_target_scale,
            snap.estimated_collision_force_dir,
            snap.threshold_force
          );

          speed_to_publish = updateSpeedScaleSmooth(guarded_target_scale);
          should_publish_speed = true;

          snap.applied_speed_scale = speed_to_publish;
          snap.link_speed_magnitudes = computeRobotLinkSpeedMagnitudes();

          latest_demo_state_ = snap;
          has_latest_demo_state_ = true;

          publishDemoState(snap);
          appendDemoLog(snap);
          publishCollisionState(*best_cand, rel_speeds[best_idx], speed_to_publish);

          pending_query_.reset();
        }
      }
      else if (pendingExpired(batch)) {
        speed_to_publish = updateSpeedScaleSmooth(min_speed_scale_);
        should_publish_speed = true;
        pending_query_.reset();
      }
      else {
        return;
      }
    }

    // 2) top3 후보 생성
    std::vector<CollisionCandidate> top_candidates;
    if (!computeTopKCollisionCandidates(top_candidates, 3)) {
      if (should_publish_speed) {
        publishSpeedScale(speed_to_publish);
      }
      return;
    }

    const double distance = top_candidates.front().distance;

    const double robot_radius_m = 1.3;
    const double distance_margin = 1.15;
    const double assessment_distance_threshold = robot_radius_m * distance_margin;

    if (distance > assessment_distance_threshold) {
      speed_to_publish = updateSpeedScaleSmooth(1.0);
      should_publish_speed = true;

      publishSpeedScale(speed_to_publish);
      return;
    }

    // publishCollisionCandidates(top_candidates);

    PendingBatchQuery batch;
    batch.batch_id = ++batch_counter_;
    batch.batch_sent_time = this->now();
    batch.candidates.reserve(top_candidates.size());

    for (const auto &c : top_candidates) {
      PendingCandidateQuery cand;
      cand.query_id = ++query_counter_;
      cand.sent_time = this->now();
      cand.human_index = c.human_index;
      cand.robot_index = c.robot_index;
      cand.distance = c.distance;
      cand.u = c.u_robot_to_human;

      publishDirectionalQueries(
        cand.query_id,
        cand.human_index,
        cand.robot_index,
        cand.u
      );

      batch.candidates.push_back(cand);
    }

    pending_query_ = std::move(batch);

    if (should_publish_speed) {
      publishSpeedScale(speed_to_publish);
    }
  }


private:
  // Subscribers
  rclcpp::Subscription<hrc_interfaces::msg::HumanDynamicsState>::SharedPtr sub_human_state_;
  rclcpp::Subscription<hrc_interfaces::msg::RobotDynamicsState>::SharedPtr sub_robot_state_;
  rclcpp::Subscription<hrc_interfaces::msg::DirectionalMeffResult>::SharedPtr sub_human_result_;
  rclcpp::Subscription<hrc_interfaces::msg::DirectionalMeffResult>::SharedPtr sub_robot_result_;

  // Publishers
  rclcpp::Publisher<hrc_interfaces::msg::DirectionalMeffQuery>::SharedPtr pub_human_query_;
  rclcpp::Publisher<hrc_interfaces::msg::DirectionalMeffQuery>::SharedPtr pub_robot_query_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_scale_;
  rclcpp::Publisher<hrc_interfaces::msg::CollisionState>::SharedPtr pub_collision_state_;
  rclcpp::Publisher<hrc_interfaces::msg::CollisionCandidates>::SharedPtr pub_collision_candidates_;
  rclcpp::Publisher<hrc_interfaces::msg::DemoState>::SharedPtr pub_demo_state_;
  std::unordered_map<uint32_t, std::string> robot_index_to_link_name_;


  rclcpp::TimerBase::SharedPtr timer_;

  DynamicsSnapshot human_state_;
  DynamicsSnapshot robot_state_;
  std::optional<PendingBatchQuery> pending_query_; // cadidate 뱔로 query_id를 쓰고
  uint64_t batch_counter_{0}; // 배치 단위

  uint64_t query_counter_{0};
  double current_speed_scale_{1.0};

  // 데모 영상 시간 맞춤
  DemoStateSnapshot latest_demo_state_;
  bool has_latest_demo_state_{false};

  rclcpp::Time demo_start_time_{0, 0, RCL_ROS_TIME};
  bool demo_time_initialized_{false};


  int timer_period_ms_{10};
  int query_timeout_ms_{150};

  double alpha_up_{0.10};
  double alpha_down_{0.35};
  double min_speed_scale_{0.05};
  double max_speed_scale_{1.0};

  std::string iso_json_path_;
  std::unordered_map<std::string, IsoLimit> iso_limits_;
  std::unordered_map<uint32_t, std::string> human_index_to_part_name_;
  std::ofstream demo_log_ofs_;
  std::string demo_log_path_;
  bool demo_log_header_written_{false};

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAssessmentNode>());
  rclcpp::shutdown();
  return 0;
}