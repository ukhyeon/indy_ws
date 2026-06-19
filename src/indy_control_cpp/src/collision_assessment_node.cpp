#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <hrc_interfaces/msg/human_dynamics_state.hpp>
#include <hrc_interfaces/msg/robot_dynamics_state.hpp>
#include <hrc_interfaces/msg/directional_meff_query.hpp>
#include <hrc_interfaces/msg/directional_meff_result.hpp>
#include <hrc_interfaces/msg/collision_state.hpp>
#include "hrc_interfaces/msg/collision_candidates.hpp"
#include <hrc_interfaces/msg/demo_state.hpp>
#include <hrc_interfaces/msg/speed_scale_command.hpp>

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
#include "indy_control_cpp/perf_logger.hpp"

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
    alpha_down_ = this->declare_parameter<double>("alpha_down", 0.28);   // slow down faster

    min_speed_scale_ = this->declare_parameter<double>("min_speed_scale", 0.05);
    max_speed_scale_ = this->declare_parameter<double>("max_speed_scale", 1.0);
    guard_margin_ = this->declare_parameter<double>("guard_margin", 0.4); // 속도 마진 주기
    force_guard_onset_ratio_ =
      this->declare_parameter<double>("force_guard_onset_ratio", 0.70);
    force_guard_cap_at_threshold_ =
      this->declare_parameter<double>("force_guard_cap_at_threshold", 0.30);
    robot_radius_m_ = this->declare_parameter<double>("robot_radius_m", 1.3);
    distance_margin_ = this->declare_parameter<double>("distance_margin", 1.15);
    robot_base_center_world_.x() =
      this->declare_parameter<double>("robot_base_center_x", 0.0);
    robot_base_center_world_.y() =
      this->declare_parameter<double>("robot_base_center_y", 0.0);
    robot_base_center_world_.z() =
      this->declare_parameter<double>("robot_base_center_z", 0.6);

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

    pub_speed_scale_ = this->create_publisher<hrc_interfaces::msg::SpeedScaleCommand>(
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

        perf_collision_logger_ = std::make_unique<PerfCsvLogger>(
      "/home/robotics/hrc_ws/analysis/realtime_perf/paper_collision_assessment_perf.csv",
      std::vector<std::string>{
        "t_ros_sec",
        "assessment_id",
        "human_state_age_ms",
        "robot_state_age_ms",
        "topk_candidate_ms",
        "query_publish_ms",
        "human_meff_latency_ms",
        "robot_meff_latency_ms",
        "batch_query_latency_ms",
        "final_eval_ms",
        "candidate_assessment_latency_ms",
        "speed_publish_ms",
        "total_timer_ms",
        "nominal_deadline_ms",
        "deadline_miss",
        "query_timeout",
        "num_candidates"
      },
      100
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&CollisionAssessmentNode::timerCallback, this));

    demo_log_path_ = "/home/robotics/hrc_ws/analysis/demo_log/demo_metrics.csv";
    demo_log_ofs_.open(demo_log_path_, std::ios::out | std::ios::trunc);
    
    demo_start_time_ = this->now();
    demo_time_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "collision_assessment_node started");
    RCLCPP_INFO(this->get_logger(), "Loaded ISO JSON: %s", iso_json_path_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "PFL activation sphere: center=[%.3f, %.3f, %.3f] m, R_act=%.3f m",
      robot_base_center_world_.x(),
      robot_base_center_world_.y(),
      robot_base_center_world_.z(),
      activationRadius());
  }

private:

  std::unique_ptr<PerfCsvLogger> perf_collision_logger_;

  double ageMsFromStamp(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() <= 0) {
      return -1.0;
    }
    return (this->now() - stamp).nanoseconds() / 1e6;
  }

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

    double target_speed_scale{1.0};         // raw PFL target before force guard
    double command_target_speed_scale{1.0}; // actual smoothing input after force guard
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

  struct ActiveCandidateState
  {
    bool valid{false};

    uint64_t query_id{0};
    uint32_t human_index{0};
    uint32_t robot_index{0};

    std::string body_name;
    std::string link_name;

    double target_scale{1.0};
    double force_ratio{0.0};
    double distance{0.0};

    rclcpp::Time selected_time{0, 0, RCL_ROS_TIME};
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
    // RCLCPP_INFO_THROTTLE(
    // this->get_logger(), *this->get_clock(), 500,
    // "[query] qid=%lu | h=%u r=%u | u_h=[%.3f %.3f %.3f] | u_r=[%.3f %.3f %.3f]",
    // query_id,
    // human_index, robot_index,
    // u_robot_to_human.x(), u_robot_to_human.y(), u_robot_to_human.z(),
    // -u_robot_to_human.x(), -u_robot_to_human.y(), -u_robot_to_human.z());


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
    // return vr_along_u ;
    return vr_along_u;
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

    // -----------------------------
    // 내부 안전 마진 적용
    // 실제 제어는 v_allow보다 더 작은 v_guard 기준으로 감속 시작
    // -----------------------------
    const double v_guard = guard_margin_ * v_allow;

    const double ratio = v_guard / std::max(relative_speed_along_u, 1e-6);

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

  // 불연속
  // double applyForceLimitGuard(
  //   double target_speed_scale,
  //   double estimated_collision_force,
  //   double threshold_force) const
  // {
  //   const double eps = 1e-6;
  //   const double ratio = estimated_collision_force / std::max(threshold_force, eps);

  //   double guarded_scale = target_speed_scale;

  //   if (ratio > 1.30) {
  //     guarded_scale = std::min(guarded_scale, 0.05);
  //   } else if (ratio > 1.15) {
  //     guarded_scale = std::min(guarded_scale, 0.10);
  //   } else if (ratio > 1.05) {
  //     guarded_scale = std::min(guarded_scale, 0.20);
  //   } else if (ratio > 1.00) {
  //     guarded_scale = std::min(guarded_scale, 0.35);
  //   }

  //   return clamp(guarded_scale, min_speed_scale_, max_speed_scale_);
  // }

  double applyForceLimitGuard(
      double target_speed_scale,
      double estimated_collision_force,
      double threshold_force) const
  {
      const double eps = 1e-6;

      const double force_ratio =
          std::max(0.0, estimated_collision_force) /
          std::max(threshold_force, eps);

      // Smooth force-ratio-based cap. These are implementation parameters,
      // not ISO-derived values. Lower onset starts attenuation earlier, while
      // lower cap_at_threshold commands stronger preemptive deceleration.
      const double onset_ratio =
          clamp(force_guard_onset_ratio_, 0.05, 0.99);
      const double cap_at_threshold =
          clamp(force_guard_cap_at_threshold_, min_speed_scale_, max_speed_scale_);

      const double guard_ratio =
          force_ratio / std::max(onset_ratio, eps);

      const double kappa =
          -std::log(cap_at_threshold) /
          std::max(1.0 / onset_ratio - 1.0, eps);

      const double guard_cap =
          (guard_ratio <= 1.0)
              ? 1.0
              : std::exp(-kappa * (guard_ratio - 1.0));

      const double guarded_scale =
          std::min(target_speed_scale, guard_cap);

      return clamp(
          guarded_scale,
          min_speed_scale_,
          max_speed_scale_);
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

  void publishSpeedScale(
    double scale,
    uint64_t assessment_id = 0,
    const rclcpp::Time & assessment_start = rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    hrc_interfaces::msg::SpeedScaleCommand msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.assessment_id = assessment_id;
    msg.assessment_start_stamp = assessment_start;
    msg.speed_scale =
      static_cast<float>(clamp(scale, min_speed_scale_, max_speed_scale_));
    pub_speed_scale_->publish(msg);
  }

  double publishSpeedScaleMeasured(
    double scale,
    uint64_t assessment_id = 0,
    const rclcpp::Time & assessment_start = rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    const auto t0 = PerfCsvLogger::now();
    publishSpeedScale(scale, assessment_id, assessment_start);
    return PerfCsvLogger::msSince(t0);
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
      << "speed_scale_target,command_target_speed_scale,speed_scale_applied,"
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
      << snap.command_target_speed_scale << ","
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
    return !(robot_index == 0 || robot_index == 1 || robot_index ==2 );
  }

  bool isValidHumanCollisionIndex(uint32_t human_index) const
  {
    // left_foot(6), right_foot(7) 제외
    return !(human_index == 6 || human_index == 7);
  }

  double activationRadius() const
  {
    return robot_radius_m_ * distance_margin_;
  }

  bool hasHumanBodyPartInsideActivationSphere(
    double * minimum_distance_from_base = nullptr) const
  {
    double minimum_distance = std::numeric_limits<double>::infinity();
    bool has_valid_body_part = false;
    const double activation_radius = activationRadius();

    for (size_t hi = 0; hi < human_state_.positions.size(); ++hi) {
      const uint32_t human_index = static_cast<uint32_t>(hi);
      if (!isValidHumanCollisionIndex(human_index)) {
        continue;
      }

      has_valid_body_part = true;
      const double distance_from_base =
        (human_state_.positions[hi] - robot_base_center_world_).norm();
      minimum_distance = std::min(minimum_distance, distance_from_base);

      if (distance_from_base <= activation_radius) {
        if (minimum_distance_from_base != nullptr) {
          *minimum_distance_from_base = minimum_distance;
        }
        return true;
      }
    }

    if (minimum_distance_from_base != nullptr) {
      *minimum_distance_from_base =
        has_valid_body_part ? minimum_distance : -1.0;
    }
    return false;
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

  bool isHighPriorityBody(const std::string &body_name) const
  {
    return (body_name == "head" || body_name == "neck");
  }

  bool isArmFamilyBody(const std::string &body_name) const
  {
    return (
      body_name == "left_shoulder"  ||
      body_name == "right_shoulder" ||
      body_name == "left_upper_arm" ||
      body_name == "right_upper_arm" ||
      body_name == "left_lower_arm" ||
      body_name == "right_lower_arm" ||
      body_name == "left_hand"      ||
      body_name == "right_hand"
    );
  }

  double getSwitchMargin(
    const std::string &current_body,
    const std::string &new_body) const
  {
    const bool current_arm = isArmFamilyBody(current_body);
    const bool new_arm = isArmFamilyBody(new_body);

    // 팔 계열 내부 switching은 더 보수적으로
    if (current_arm && new_arm) {
      return switch_margin_arm_family_;
    }

    return switch_margin_default_;
  }

  bool shouldSwitchCandidate(
    const ActiveCandidateState &current_state,
    uint64_t new_query_id,
    uint32_t new_human_index,
    uint32_t new_robot_index,
    const std::string &new_body_name,
    const std::string &new_link_name,
    double new_target_scale,
    double new_force_ratio,
    double new_distance,
    const rclcpp::Time &now_time) const
  {
    // 현재 활성 후보가 없으면 무조건 채택
    if (!current_state.valid) {
      return true;
    }

    // 동일 후보면 그대로 유지
    if (current_state.human_index == new_human_index &&
        current_state.robot_index == new_robot_index) {
      return true;
    }

    const double hold_sec =
      (now_time - current_state.selected_time).seconds();

    // -----------------------------
    // 1) Immediate override
    // -----------------------------

    // 새 후보가 head/neck 이고, 현재보다 조금이라도 더 위험하면 즉시 전환
    if (isHighPriorityBody(new_body_name) &&
        new_target_scale < current_state.target_scale) {
      return true;
    }

    // threshold 초과면 즉시 전환
    if (new_force_ratio > immediate_force_ratio_threshold_) {
      return true;
    }

    // 매우 위험한 scale이면 즉시 전환
    if (new_target_scale < immediate_target_scale_threshold_) {
      return true;
    }

    // -----------------------------
    // 2) Hold time
    // -----------------------------
    // 최근에 막 바꿨으면 기본적으로 유지
    if (hold_sec < candidate_hold_time_sec_) {
      return false;
    }

    // -----------------------------
    // 3) Hysteresis by target scale
    // -----------------------------
    const double margin = getSwitchMargin(current_state.body_name, new_body_name);

    // 새 후보가 충분히 더 위험할 때만 전환
    if (new_target_scale < (current_state.target_scale - margin)) {
      return true;
    }

    return false;
  }

  void updateActiveCandidateState(
    ActiveCandidateState &state,
    uint64_t query_id,
    uint32_t human_index,
    uint32_t robot_index,
    const std::string &body_name,
    const std::string &link_name,
    double target_scale,
    double force_ratio,
    double distance,
    const rclcpp::Time &now_time)
  {
    state.valid = true;
    state.query_id = query_id;
    state.human_index = human_index;
    state.robot_index = robot_index;
    state.body_name = body_name;
    state.link_name = link_name;
    state.target_scale = target_scale;
    state.force_ratio = force_ratio;
    state.distance = distance;
    state.selected_time = now_time;
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
    const auto t_timer0 = PerfCsvLogger::now();

    double human_state_age_ms = -1.0;
    double robot_state_age_ms = -1.0;
    double topk_candidate_ms = 0.0;
    double query_publish_ms = 0.0;
    double human_meff_latency_ms = -1.0;
    double robot_meff_latency_ms = -1.0;
    double batch_query_latency_ms = -1.0;
    double final_eval_ms = 0.0;
    double candidate_assessment_latency_ms = -1.0;
    double speed_publish_ms = 0.0;
    bool query_timeout = false;
    int num_candidates = 0;
    uint64_t completed_assessment_id = 0;
    rclcpp::Time completed_assessment_start{0, 0, RCL_ROS_TIME};


    if (!human_state_.valid || !robot_state_.valid) {
      return;
    }
    human_state_age_ms = ageMsFromStamp(human_state_.stamp);
    robot_state_age_ms = ageMsFromStamp(robot_state_.stamp);

    double minimum_human_base_distance = -1.0;
    const bool pfl_active = hasHumanBodyPartInsideActivationSphere(
      &minimum_human_base_distance);

    if (!pfl_active) {
      pending_query_.reset();
      active_candidate_.valid = false;

      const double speed_to_publish = updateSpeedScaleSmooth(max_speed_scale_);
      const double speed_publish_ms = publishSpeedScaleMeasured(speed_to_publish);

      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "PFL skipped: all valid human body parts are outside activation sphere "
        "(min base distance=%.3f m, R_act=%.3f m)",
        minimum_human_base_distance,
        activationRadius());

      writeCollisionPerfRow(
        0,
        human_state_age_ms,
        robot_state_age_ms,
        0.0,
        0.0,
        -1.0,
        -1.0,
        -1.0,
        0.0,
        -1.0,
        speed_publish_ms,
        PerfCsvLogger::msSince(t_timer0),
        false,
        0
      );
      return;
    }

    bool should_publish_speed = false;
    double speed_to_publish = current_speed_scale_;

    // 1) pending batch 처리
    if (pending_query_.has_value()) {
      auto &batch = pending_query_.value();

      if (batchAllResultsReady(batch)) {

        const auto t_final0 = PerfCsvLogger::now();
        completed_assessment_id = batch.batch_id;
        completed_assessment_start = batch.batch_sent_time;

        batch_query_latency_ms =
          (this->now() - batch.batch_sent_time).nanoseconds() / 1e6;

        human_meff_latency_ms = -1.0;
        robot_meff_latency_ms = -1.0;

        for (const auto & cand : batch.candidates) {
          if (cand.human_result_ready) {
            const double lat =
              (cand.human_result_time - cand.sent_time).nanoseconds() / 1e6;
            human_meff_latency_ms =
              std::max(human_meff_latency_ms, lat);
          }

          if (cand.robot_result_ready) {
            const double lat =
              (cand.robot_result_time - cand.sent_time).nanoseconds() / 1e6;
            robot_meff_latency_ms =
              std::max(robot_meff_latency_ms, lat);
          }
        }


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
          // -----------------------------
          // 1) raw best 후보 정보 찾기
          // -----------------------------
          size_t raw_best_idx = 0;
          double raw_best_target_scale = std::numeric_limits<double>::infinity();

          for (size_t i = 0; i < batch.candidates.size(); ++i) {
            if (batch.candidates[i].query_id == best_cand->query_id) {
              raw_best_idx = i;
              raw_best_target_scale = target_scales[i];
              break;
            }
          }

          const auto now_time = this->now();

          const std::string new_body_name =
            human_index_to_part_name_[best_cand->human_index];
          const std::string new_link_name =
            robot_index_to_link_name_[best_cand->robot_index];

          auto iso_opt_raw = getIsoLimitForHumanIndex(best_cand->human_index);
          if (!iso_opt_raw.has_value()) {
            speed_to_publish = updateSpeedScaleSmooth(min_speed_scale_);
            should_publish_speed = true;
            pending_query_.reset();
            final_eval_ms = PerfCsvLogger::msSince(t_final0);
            candidate_assessment_latency_ms =
              (this->now() - completed_assessment_start).nanoseconds() / 1e6;
            speed_publish_ms = publishSpeedScaleMeasured(
              speed_to_publish,
              completed_assessment_id,
              completed_assessment_start);
            return;
          }

          const double new_reduced_mass =
            computeReducedMass(best_cand->human_meff, best_cand->robot_meff);

          const double new_estimated_force =
            computeEstimatedCollisionForce(
              rel_speeds[raw_best_idx],
              new_reduced_mass,
              iso_opt_raw->k
            );

          const double new_force_ratio =
            iso_opt_raw->F_max > 1e-6
              ? new_estimated_force / iso_opt_raw->F_max
              : 0.0;

          // -----------------------------
          // 2) switch gate
          // -----------------------------
          const bool switch_to_new = shouldSwitchCandidate(
            active_candidate_,
            best_cand->query_id,
            best_cand->human_index,
            best_cand->robot_index,
            new_body_name,
            new_link_name,
            raw_best_target_scale,
            new_force_ratio,
            best_cand->distance,
            now_time
          );

          // -----------------------------
          // 3) 최종 사용할 candidate 결정
          // -----------------------------
          size_t best_idx = raw_best_idx;
          double best_target_scale = raw_best_target_scale;
          const PendingCandidateQuery *chosen_cand = best_cand;

          if (!switch_to_new && active_candidate_.valid) {
            bool found_active_pair = false;

            for (size_t i = 0; i < batch.candidates.size(); ++i) {
              if (batch.candidates[i].human_index == active_candidate_.human_index &&
                  batch.candidates[i].robot_index == active_candidate_.robot_index) {
                best_idx = i;
                best_target_scale = target_scales[i];
                chosen_cand = &batch.candidates[i];
                found_active_pair = true;
                break;
              }
            }

            // 현재 batch에 이전 active pair가 없으면 raw best로 fallback
            if (!found_active_pair) {
              best_idx = raw_best_idx;
              best_target_scale = raw_best_target_scale;
              chosen_cand = best_cand;
            }
          }


          const std::string chosen_body_name =
            human_index_to_part_name_[chosen_cand->human_index];
          const std::string chosen_link_name =
            robot_index_to_link_name_[chosen_cand->robot_index];

          auto iso_opt = getIsoLimitForHumanIndex(chosen_cand->human_index);
          if (!iso_opt.has_value()) {
            speed_to_publish = updateSpeedScaleSmooth(min_speed_scale_);
            should_publish_speed = true;
            pending_query_.reset();
            // publishSpeedScale(speed_to_publish);
            final_eval_ms = PerfCsvLogger::msSince(t_final0);
            candidate_assessment_latency_ms =
              (this->now() - completed_assessment_start).nanoseconds() / 1e6;
            speed_publish_ms = publishSpeedScaleMeasured(
              speed_to_publish,
              completed_assessment_id,
              completed_assessment_start);
            return;
          }

          const double chosen_reduced_mass =
            computeReducedMass(chosen_cand->human_meff, chosen_cand->robot_meff);

          const double chosen_estimated_force =
            computeEstimatedCollisionForce(
              rel_speeds[best_idx],
              chosen_reduced_mass,
              iso_opt->k
            );

          const double chosen_force_ratio =
            iso_opt->F_max > 1e-6
              ? chosen_estimated_force / iso_opt->F_max
              : 0.0;

          
          updateActiveCandidateState(
            active_candidate_,
            chosen_cand->query_id,
            chosen_cand->human_index,
            chosen_cand->robot_index,
            chosen_body_name,
            chosen_link_name,
            best_target_scale,
            chosen_force_ratio,
            chosen_cand->distance,
            now_time
          );
          

          // -----------------------------
          // 4) Demo snapshot
          // -----------------------------
          DemoStateSnapshot snap;
          snap.stamp = this->now();
          snap.t_demo_sec = (snap.stamp - demo_start_time_).seconds();

          snap.selected_human_index = chosen_cand->human_index;
          snap.selected_robot_index = chosen_cand->robot_index;
          snap.selected_body_name = human_index_to_part_name_[chosen_cand->human_index];
          snap.selected_link_name = robot_index_to_link_name_[chosen_cand->robot_index];
          snap.distance = chosen_cand->distance;
          snap.direction_u = chosen_cand->u;
          snap.relative_speed_along_u = rel_speeds[best_idx];

          

          const Eigen::Vector3d u_norm = chosen_cand->u.normalized();
          const Eigen::Vector3d v_r = robot_state_.velocities[chosen_cand->robot_index];
          const double vr_along_u = v_r.dot(u_norm);

          snap.robot_speed_along_u_raw = vr_along_u;
          snap.approach_bias_added = std::max(0.0, snap.relative_speed_along_u - vr_along_u);

          snap.human_meff = chosen_cand->human_meff;
          snap.human_meff_iso = iso_opt->m_h_iso;
          snap.robot_meff = chosen_cand->robot_meff;

          snap.reduced_mass = computeReducedMass(snap.human_meff, snap.robot_meff);
          snap.reduced_mass_iso = computeReducedMass(snap.human_meff_iso, snap.robot_meff);

          snap.iso_k = iso_opt->k;

          const double v_allow_dir_raw =
            computeVmax(iso_opt->F_max, snap.reduced_mass, iso_opt->k);
          const double v_allow_iso_raw =
            computeVmax(iso_opt->F_max, snap.reduced_mass_iso, iso_opt->k);

          // 실제 제어에 쓰는 내부 guard 기준 속도
          snap.v_allow_dir = guard_margin_ * v_allow_dir_raw;
          snap.v_allow_iso = guard_margin_ * v_allow_iso_raw;

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

          const double guarded_target_scale = applyForceLimitGuard(
            best_target_scale,
            snap.estimated_collision_force_dir,
            snap.threshold_force
          );

          snap.command_target_speed_scale = guarded_target_scale;
          speed_to_publish = updateSpeedScaleSmooth(guarded_target_scale);
          should_publish_speed = true;

          snap.applied_speed_scale = speed_to_publish;
          snap.link_speed_magnitudes = computeRobotLinkSpeedMagnitudes();

          latest_demo_state_ = snap;
          has_latest_demo_state_ = true;

          publishDemoState(snap);
          appendDemoLog(snap);
          publishCollisionState(*chosen_cand, rel_speeds[best_idx], speed_to_publish);

          pending_query_.reset();
          
        }

        final_eval_ms = PerfCsvLogger::msSince(t_final0);
        candidate_assessment_latency_ms =
          (this->now() - completed_assessment_start).nanoseconds() / 1e6;
        
      }
      else if (pendingExpired(batch)) {
        query_timeout = true;
        batch_query_latency_ms =
          (this->now() - batch.batch_sent_time).nanoseconds() / 1e6;


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

    const auto t_topk0 = PerfCsvLogger::now();
    const bool has_candidates = computeTopKCollisionCandidates(top_candidates, 3);
    const auto t_topk1 = PerfCsvLogger::now();

    topk_candidate_ms = PerfCsvLogger::msBetween(t_topk0, t_topk1);
    num_candidates = static_cast<int>(top_candidates.size());

    if (!has_candidates) {
      active_candidate_.valid = false;

      if (should_publish_speed) {
        speed_publish_ms = publishSpeedScaleMeasured(
          speed_to_publish,
          completed_assessment_id,
          completed_assessment_start);
      }

      writeCollisionPerfRow(
        completed_assessment_id,
        human_state_age_ms,
        robot_state_age_ms,
        topk_candidate_ms,
        query_publish_ms,
        human_meff_latency_ms,
        robot_meff_latency_ms,
        batch_query_latency_ms,
        final_eval_ms,
        candidate_assessment_latency_ms,
        speed_publish_ms,
        PerfCsvLogger::msSince(t_timer0),
        query_timeout,
        num_candidates
      );

      return;
    }

    // publishCollisionCandidates(top_candidates);

    PendingBatchQuery batch;
    batch.batch_id = ++batch_counter_;
    batch.batch_sent_time = this->now();
    batch.candidates.reserve(top_candidates.size());

    const auto t_query_pub0 = PerfCsvLogger::now();

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

    const auto t_query_pub1 = PerfCsvLogger::now();
    query_publish_ms = PerfCsvLogger::msBetween(t_query_pub0, t_query_pub1);

    pending_query_ = std::move(batch);

    if (should_publish_speed) {
      speed_publish_ms = publishSpeedScaleMeasured(
        speed_to_publish,
        completed_assessment_id,
        completed_assessment_start);
    }

    writeCollisionPerfRow(
      completed_assessment_id,
      human_state_age_ms,
      robot_state_age_ms,
      topk_candidate_ms,
      query_publish_ms,
      human_meff_latency_ms,
      robot_meff_latency_ms,
      batch_query_latency_ms,
      final_eval_ms,
      candidate_assessment_latency_ms,
      speed_publish_ms,
      PerfCsvLogger::msSince(t_timer0),
      query_timeout,
      num_candidates
    );


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
  rclcpp::Publisher<hrc_interfaces::msg::SpeedScaleCommand>::SharedPtr pub_speed_scale_;
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

  ActiveCandidateState active_candidate_;

  double switch_margin_default_{0.08}; // 
  double switch_margin_arm_family_{0.2};
  double candidate_hold_time_sec_{0.30};

  double immediate_force_ratio_threshold_{1.0};
  double immediate_target_scale_threshold_{0.4};
  

  // 데모 영상 시간 맞춤
  DemoStateSnapshot latest_demo_state_;
  bool has_latest_demo_state_{false};

  rclcpp::Time demo_start_time_{0, 0, RCL_ROS_TIME};
  bool demo_time_initialized_{false};


  int timer_period_ms_{10};
  int query_timeout_ms_{150};

  double alpha_up_{0.10};
  double alpha_down_{0.28};
  double min_speed_scale_{0.05};
  double max_speed_scale_{1.0};
  double guard_margin_{0.4};
  double force_guard_onset_ratio_{0.70};
  double force_guard_cap_at_threshold_{0.30};
  double robot_radius_m_{1.3};
  double distance_margin_{1.15};
  Eigen::Vector3d robot_base_center_world_{0.0, 0.0, 0.6};

  std::string iso_json_path_;
  std::unordered_map<std::string, IsoLimit> iso_limits_;
  std::unordered_map<uint32_t, std::string> human_index_to_part_name_;
  std::ofstream demo_log_ofs_;
  std::string demo_log_path_;
  bool demo_log_header_written_{false};


  void writeCollisionPerfRow(
    uint64_t assessment_id,
    double human_state_age_ms,
    double robot_state_age_ms,
    double topk_candidate_ms,
    double query_publish_ms,
    double human_meff_latency_ms,
    double robot_meff_latency_ms,
    double batch_query_latency_ms,
    double final_eval_ms,
    double candidate_assessment_latency_ms,
    double speed_publish_ms,
    double total_timer_ms,
    bool query_timeout,
    int num_candidates)
  {
    const double deadline_ms = static_cast<double>(timer_period_ms_);

    if (!perf_collision_logger_) {
      return;
    }

    perf_collision_logger_->writeRow({
      PerfCsvLogger::toStr(this->now().seconds()),
      std::to_string(assessment_id),
      PerfCsvLogger::toStr(human_state_age_ms),
      PerfCsvLogger::toStr(robot_state_age_ms),
      PerfCsvLogger::toStr(topk_candidate_ms),
      PerfCsvLogger::toStr(query_publish_ms),
      PerfCsvLogger::toStr(human_meff_latency_ms),
      PerfCsvLogger::toStr(robot_meff_latency_ms),
      PerfCsvLogger::toStr(batch_query_latency_ms),
      PerfCsvLogger::toStr(final_eval_ms),
      PerfCsvLogger::toStr(candidate_assessment_latency_ms),
      PerfCsvLogger::toStr(speed_publish_ms),
      PerfCsvLogger::toStr(total_timer_ms),
      PerfCsvLogger::toStr(deadline_ms),
      std::to_string(static_cast<int>(total_timer_ms > deadline_ms)),
      std::to_string(static_cast<int>(query_timeout)),
      std::to_string(num_candidates)
    });
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAssessmentNode>());
  rclcpp::shutdown();
  return 0;
}
