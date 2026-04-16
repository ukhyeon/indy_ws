#include "pinocchio/fwd.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <limits>

struct LinkResult
{
  std::string name;
  double mass{0.0};
  Eigen::Vector3d com_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d com_world{Eigen::Vector3d::Zero()};
  Eigen::MatrixXd Jv_com;
  Eigen::MatrixXd Jw_com;
};

Eigen::Matrix3d skew(const Eigen::Vector3d &r)
{
  Eigen::Matrix3d S;
  S <<     0.0, -r.z(),  r.y(),
        r.z(),     0.0, -r.x(),
       -r.y(),  r.x(),     0.0;
  return S;
}

double effectiveMassAlong(const Eigen::Matrix3d &A,
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

int main()
{
  const std::string urdf_path = "/home/robotics/indy_ws/src/urdf_file/indy7.urdf";

  // MATLAB 출력과 맞춰 보기 위해 link0~link6 사용
  const std::vector<std::string> target_links = {
    "link0", "link1", "link2", "link3", "link4", "link5", "link6"
  };

  // 비교 방향: MATLAB 코드와 동일하게 base/world +X
  const Eigen::Vector3d u = Eigen::Vector3d::UnitX();

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Model name: " << model.name << "\n";
  std::cout << "model.nq: " << model.nq << ", model.nv: " << model.nv << "\n\n";

  // 같은 자세: q = 0
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // ------------------------------------------
  // 1) CRBA로 M(q) 계산
  // ------------------------------------------
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd M_crba = data.M;

  // ------------------------------------------
  // 2) MATLAB sum method와 동일하게 Mq_sum 계산
  // ------------------------------------------
  Eigen::MatrixXd M_sum = Eigen::MatrixXd::Zero(model.nv, model.nv);

  std::vector<LinkResult> link_results;

  for (const auto &link_name : target_links)
  {
    if (!model.existFrame(link_name, pinocchio::BODY)) {
      std::cerr << "[WARN] BODY frame not found: " << link_name << "\n";
      continue;
    }

    const pinocchio::FrameIndex frame_id =
        model.getFrameId(link_name, pinocchio::BODY);

    const pinocchio::Frame &frame = model.frames[frame_id];
    const pinocchio::JointIndex joint_id = frame.parentJoint;

    // Pinocchio inertia attached to this joint/body
    const auto &inertia = model.inertias[joint_id];

    const double mass = inertia.mass();
    const Eigen::Vector3d com_local = inertia.lever();    // body frame 기준 CoM
    const Eigen::Matrix3d I_com_local = inertia.inertia().matrix();

    // link frame 원점 pose (base/world 기준)
    const Eigen::Matrix3d R_wf = data.oMf[frame_id].rotation();
    const Eigen::Vector3d p_wf = data.oMf[frame_id].translation();

    // CoM world 위치
    const Eigen::Vector3d r_world = R_wf * com_local;     // frame origin -> CoM
    const Eigen::Vector3d p_com_world = p_wf + r_world;

    // link frame 원점 기준 Jacobian (축은 world/base 정렬)
    Eigen::MatrixXd J_frame(6, model.nv);
    J_frame.setZero();

    pinocchio::getFrameJacobian(
        model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // Pinocchio convention: [linear; angular]
    Eigen::MatrixXd Jv_o = J_frame.topRows(3);
    Eigen::MatrixXd Jw   = J_frame.bottomRows(3);

    // MATLAB과 같은 point-shift:
    // Jv_com = Jv_o - skew(r_world) * Jw
    Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;
    Eigen::MatrixXd Jw_com = Jw;

    // local CoM inertia -> world
    Eigen::Matrix3d I_world = R_wf * I_com_local * R_wf.transpose();

    // MATLAB sum method과 같은 Mq 누적
    if (mass > 1e-9) {
      M_sum += mass * (Jv_com.transpose() * Jv_com)
             +       (Jw_com.transpose() * I_world * Jw_com);
    }

    LinkResult res;
    res.name = link_name;
    res.mass = mass;
    res.com_local = com_local;
    res.com_world = p_com_world;
    res.Jv_com = Jv_com;
    res.Jw_com = Jw_com;
    link_results.push_back(res);
  }

  // ------------------------------------------
  // 3) M_sum 과 M_crba 비교
  // ------------------------------------------
  double rel_err_M =
      (M_sum - M_crba).norm() / std::max(M_crba.norm(), 1e-12);

  std::cout << "====================================================\n";
  std::cout << "M_sum:\n" << M_sum << "\n\n";
  std::cout << "M_crba:\n" << M_crba << "\n\n";
  std::cout << "Relative Frobenius error ||M_sum - M_crba|| / ||M_crba|| = "
            << rel_err_M << "\n";
  std::cout << "====================================================\n\n";

  // ------------------------------------------
  // 4) 링크별 m_eff 비교
  //    - MATLAB sum method 기반
  //    - CRBA 기반
  // ------------------------------------------
  Eigen::LDLT<Eigen::MatrixXd> ldlt_sum(M_sum);
  Eigen::LDLT<Eigen::MatrixXd> ldlt_crba(M_crba);

  for (const auto &res : link_results)
  {
    if (res.mass < 1e-9) {
      continue;
    }

    // A_sum = Jv_com * M_sum^{-1} * Jv_com^T
    Eigen::MatrixXd X_sum = ldlt_sum.solve(res.Jv_com.transpose());
    Eigen::Matrix3d A_sum = res.Jv_com * X_sum;

    // A_crba = Jv_com * M_crba^{-1} * Jv_com^T
    Eigen::MatrixXd X_crba = ldlt_crba.solve(res.Jv_com.transpose());
    Eigen::Matrix3d A_crba = res.Jv_com * X_crba;

    const double meff_sum  = effectiveMassAlong(A_sum,  u);
    const double meff_crba = effectiveMassAlong(A_crba, u);

    std::cout << "Body: " << std::setw(8) << std::left << res.name
              << " | Mass=" << std::setw(9) << res.mass
              << " | CoM(local)=[" << res.com_local.transpose() << "]"
              << " | CoM(world)=[" << res.com_world.transpose() << "]"
              << " | m_eff_sum="  << meff_sum
              << " | m_eff_crba=" << meff_crba
              << "\n";
  }

  return 0;
}