#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <hrc_interfaces/msg/human_dynamics_state.hpp>
#include <hrc_interfaces/msg/robot_dynamics_state.hpp>
#include <hrc_interfaces/msg/directional_meff_query.hpp>
#include <hrc_interfaces/msg/directional_meff_result.hpp>
#include <hrc_interfaces/msg/collision_state.hpp>
#include "hrc_interfaces/msg/collision_candidates.hpp"

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

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&CollisionAssessmentNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "collision_assessment_node started");
    RCLCPP_INFO(this->get_logger(), "Loaded ISO JSON: %s", iso_json_path_.c_str());
  }

private:
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

  struct PendingQuery
  {
    uint64_t query_id{0};
    rclcpp::Time sent_time;
    uint32_t human_index{0};
    uint32_t robot_index{0};
    Eigen::Vector3d u{1.0, 0.0, 0.0}; // robot -> human
    double distance{0.0};

    bool human_result_ready{false};
    bool robot_result_ready{false};

    double human_meff{0.0};
    double robot_meff{0.0};

    rclcpp::Time human_result_time; // 응답 측정용
    rclcpp::Time robot_result_time; // 응답 측정용
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

  bool findClosestPair(
    uint32_t &human_index,
    uint32_t &robot_index,
    double &distance_out,
    Eigen::Vector3d &u_out)
  {
    if (!human_state_.valid || !robot_state_.valid) {
      return false;
    }

    if (human_state_.positions.empty() || robot_state_.positions.empty()) {
      return false;
    }

    double best_dist = std::numeric_limits<double>::infinity();
    uint32_t best_h = 0;
    uint32_t best_r = 0;
    Eigen::Vector3d best_u = Eigen::Vector3d::UnitX();

    for (uint32_t r = 0; r < robot_state_.positions.size(); ++r) {
      for (uint32_t h = 0; h < human_state_.positions.size(); ++h) {
        const Eigen::Vector3d diff = human_state_.positions[h] - robot_state_.positions[r];
        const double dist = diff.norm();

        if (dist < best_dist) {
          best_dist = dist;
          best_h = h;
          best_r = r;
          best_u = diff / normSafe(diff);
        }
      }
    }

    human_index = best_h;
    robot_index = best_r;
    distance_out = best_dist;
    u_out = best_u;
    return std::isfinite(best_dist);
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

  double computeRelativeSpeedAlongU(
    uint32_t human_index,
    uint32_t robot_index,
    const Eigen::Vector3d &u_robot_to_human) const
  {
    if (human_index >= human_state_.velocities.size() ||
        robot_index >= robot_state_.velocities.size()) {
      return 0.0;
    }

    const Eigen::Vector3d v_h = human_state_.velocities[human_index];
    const Eigen::Vector3d v_r = robot_state_.velocities[robot_index];

    // positive -> robot approaching human along u
    return (v_r - v_h).dot(u_robot_to_human);
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

  // 1차 저역 통과 필터, 현재 값과 목표 값 차이를 보고 
  double updateSpeedScaleSmooth(double target_speed_scale)
  {
    const double alpha = (target_speed_scale < current_speed_scale_) ? alpha_down_ : alpha_up_; // 감속은 빠르게, 가속은 천천히
    current_speed_scale_ =
      current_speed_scale_ + alpha * (target_speed_scale - current_speed_scale_);

    current_speed_scale_ = clamp(current_speed_scale_, min_speed_scale_, max_speed_scale_);
    return current_speed_scale_;
  }

  void publishCollisionState(
    const PendingQuery &pq,
    double rel_speed,
    double speed_scale)
  {
    hrc_interfaces::msg::CollisionState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";

    msg.human_index = pq.human_index;
    msg.robot_index = pq.robot_index;
    msg.distance = static_cast<float>(pq.distance);
    msg.direction_u = toRosVec(pq.u);
    msg.relative_speed_along_u = static_cast<float>(rel_speed);
    msg.human_meff = static_cast<float>(pq.human_meff);
    msg.robot_meff = static_cast<float>(pq.robot_meff);
    msg.speed_scale = static_cast<float>(speed_scale);

    pub_collision_state_->publish(msg);
  }

  void publishSpeedScale(double scale)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(clamp(scale, min_speed_scale_, max_speed_scale_));
    pub_speed_scale_->publish(msg);
  }

  bool pendingExpired(const PendingQuery &pq) const
  {
    const auto dt_ms = (this->now() - pq.sent_time).nanoseconds() / 1e6;
    return dt_ms > static_cast<double>(query_timeout_ms_);
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
      double best_dist = std::numeric_limits<double>::infinity();
      uint32_t best_robot = 0;
      Eigen::Vector3d best_u = Eigen::Vector3d::UnitX();

      const Eigen::Vector3d &ph = human_state_.positions[hi];

      for (size_t ri = 0; ri < robot_n; ++ri) {
        const Eigen::Vector3d &pr = robot_state_.positions[ri];
        Eigen::Vector3d d = ph - pr;
        const double dist = d.norm();

        if (dist < best_dist) {
          best_dist = dist;
          best_robot = static_cast<uint32_t>(ri);

          if (dist > 1e-9) {
            best_u = d / dist;   // robot -> human
          } else {
            best_u = Eigen::Vector3d::UnitX();
          }
        }
      }

      CollisionCandidate c;
      c.human_index = static_cast<uint32_t>(hi);
      c.robot_index = best_robot;
      c.distance = best_dist;
      c.u_robot_to_human = best_u;
      all_candidates.push_back(c);
    }

    std::sort(all_candidates.begin(), all_candidates.end(),
              [](const CollisionCandidate &a, const CollisionCandidate &b) {
                return a.distance < b.distance;
              });

    const size_t out_n = std::min(k, all_candidates.size());
    top_candidates.assign(all_candidates.begin(), all_candidates.begin() + out_n);
    return !top_candidates.empty();
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

    auto &pq = pending_query_.value();
    if (msg->query_id != pq.query_id) {
      return;
    }

    if (!msg->valid) {
      RCLCPP_WARN(this->get_logger(), "Human meff result invalid for query_id=%lu", msg->query_id);
      return;
    }

    pq.human_meff = msg->directional_meff;
    pq.human_result_ready = true;
    pq.human_result_time = this->now();
  }

  void robotResultCallback(const hrc_interfaces::msg::DirectionalMeffResult::SharedPtr msg)
  {
    if (!pending_query_.has_value()) {
      return;
    }

    auto &pq = pending_query_.value();
    if (msg->query_id != pq.query_id) {
      return;
    }

    if (!msg->valid) {
      RCLCPP_WARN(this->get_logger(), "Robot meff result invalid for query_id=%lu", msg->query_id);
      return;
    }

    pq.robot_meff = msg->directional_meff;
    pq.robot_result_ready = true;
    pq.robot_result_time = this->now();
  }

  void timerCallback()
  {
    if (!human_state_.valid || !robot_state_.valid) {
      return;
    }

    if (pending_query_.has_value()) {
      auto &pq = pending_query_.value();

      if (pq.human_result_ready && pq.robot_result_ready) {
        const double rel_speed = computeRelativeSpeedAlongU(
          pq.human_index, pq.robot_index, pq.u);
        

        auto iso_opt = getIsoLimitForHumanIndex(pq.human_index);
        if (!iso_opt.has_value()) {

          const std::string part_name = human_index_to_part_name_[pq.human_index];
          // 없다면 에러 나도록 디버그,
          RCLCPP_ERROR(
            this->get_logger(),
            "Missing ISO limit for human part: index=%u name=%s | qid=%lu",
            pq.human_index,
            part_name.c_str(),
            pq.query_id
          );
          
          const double next_scale = updateSpeedScaleSmooth(min_speed_scale_);
          publishSpeedScale(next_scale);
          publishCollisionState(pq, rel_speed, next_scale);
          pending_query_.reset();
        } else {
          const double target_scale = computeTargetSpeedScale(
            rel_speed,
            pq.human_meff,
            pq.robot_meff,
            iso_opt.value()
          );

          const double next_scale = updateSpeedScaleSmooth(target_scale);
          publishSpeedScale(next_scale);
          publishCollisionState(pq, rel_speed, next_scale);

          // 응답 주기 디버그
          const double t_human_ms =
            (pq.human_result_time - pq.sent_time).nanoseconds() / 1e6;
          const double t_robot_ms =
            (pq.robot_result_time - pq.sent_time).nanoseconds() / 1e6;
          const double t_total_ms =
            (this->now() - pq.sent_time).nanoseconds() / 1e6;
          // 응답 주기 디버그
          // RCLCPP_INFO(
          //   this->get_logger(),
          //   "[rt] qid=%lu | human=%.1f ms | robot=%.1f ms | total=%.1f ms",
          //   pq.query_id, t_human_ms, t_robot_ms, t_total_ms);
          
          const std::string part_name = human_index_to_part_name_[pq.human_index];
          RCLCPP_INFO(
            this->get_logger(),
            "[collision] qid=%lu | part=%s | h=%u r=%u | dist=%.3f | vrel=%.3f | "
            "meff_h=%.2f meff_r=%.2f | target=%.2f current=%.2f",
            pq.query_id,
            part_name.c_str(),
            pq.human_index, pq.robot_index,
            pq.distance,
            rel_speed,
            pq.human_meff,
            pq.robot_meff,
            target_scale,
            next_scale
          );

          pending_query_.reset();
        }
        // 여기서 return 하지 않음
      }
      else if (pendingExpired(pq)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Query timeout: qid=%lu (human=%d, robot=%d)",
          pq.query_id,
          pq.human_result_ready ? 1 : 0,
          pq.robot_result_ready ? 1 : 0
        );

        const double next_scale = updateSpeedScaleSmooth(min_speed_scale_);
        publishSpeedScale(next_scale);
        pending_query_.reset();

        // 여기서도 return 하지 않음
      }
      else {
        // 아직 결과 기다리는 중이면 이때만 종료
        return;
      }
    }

    // pending이 없으므로 같은 tick에서 즉시 새 query 생성
    uint32_t human_index = 0;
    uint32_t robot_index = 0;
    double distance = 0.0;
    Eigen::Vector3d u = Eigen::Vector3d::UnitX();



    // pair 를 찾지 못하면 return
    if (!findClosestPair(human_index, robot_index, distance, u)) {
      return;
    }

    // 로봇의 리치보다 거리가 먼지 확인
    const double robot_radius_m = 1.3;
    const double distance_margin = 1.15;
    const double assessment_distance_threshold = robot_radius_m * distance_margin;  // 1.495 m

    // 로봇과 거리가 너무 멀다면 return
    if (distance > assessment_distance_threshold) {
      const double target_scale = 1.0;
      const double next_scale = updateSpeedScaleSmooth(target_scale);
      publishSpeedScale(next_scale);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[far skip] dist=%.3f m > threshold=%.3f m | speed keep %.2f",
        distance, assessment_distance_threshold, next_scale
      );
      return;
    }

    // ===== threshold 안에 들어왔을 때만 top 3 publish =====
    {
      std::vector<CollisionCandidate> top_candidates;
      if (computeTopKCollisionCandidates(top_candidates, 3)) {
        publishCollisionCandidates(top_candidates);
      }
    }
    // ===============================================

    // 여기서 directional query 발행
    PendingQuery pq;
    pq.query_id = ++query_counter_;
    pq.sent_time = this->now();
    pq.human_index = human_index;
    pq.robot_index = robot_index;
    pq.distance = distance;
    pq.u = u;

    publishDirectionalQueries(
      pq.query_id,
      pq.human_index,
      pq.robot_index,
      pq.u
    );

    pending_query_ = pq;
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
  std::unordered_map<uint32_t, std::string> robot_index_to_link_name_;

  rclcpp::TimerBase::SharedPtr timer_;

  DynamicsSnapshot human_state_;
  DynamicsSnapshot robot_state_;
  std::optional<PendingQuery> pending_query_;

  uint64_t query_counter_{0};
  double current_speed_scale_{1.0};

  int timer_period_ms_{10};
  int query_timeout_ms_{150};

  double alpha_up_{0.10};
  double alpha_down_{0.35};
  double min_speed_scale_{0.05};
  double max_speed_scale_{1.0};

  std::string iso_json_path_;
  std::unordered_map<std::string, IsoLimit> iso_limits_;
  std::unordered_map<uint32_t, std::string> human_index_to_part_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAssessmentNode>());
  rclcpp::shutdown();
  return 0;
}