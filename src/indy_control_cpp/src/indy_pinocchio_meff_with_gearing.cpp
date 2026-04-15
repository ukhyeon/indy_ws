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

  // 링크별 결과 출력 대상
  const std::vector<std::string> target_links = {
    "link0", "link1", "link2", "link3", "link4", "link5", "link6"
  };

  // ------------------------------------------------------------
  // [가정] gear ratio
  // J1, J2 = 121
  // J3~J6 = 101
  //
  // 주의:
  // - J1,J2의 121은 공개 자료 근거 있음
  // - J3~J6 = 101은 실험용 가정
  // ------------------------------------------------------------
  Eigen::Matrix<double, 6, 1> gear_ratio;
  gear_ratio << 121.0, 121.0, 101.0, 101.0, 101.0, 101.0;

  // ------------------------------------------------------------
  // [가정] motor rotor inertia Jm [kg*m^2]
  //
  // !!! 중요 !!!
  // 아래 값은 "예시 가정값"이다.
  // 실제 Indy7의 rotor inertia 공개값을 찾지 못했기 때문에,
  // 다른 motor 를 기준으로 하며 이는 추후 보완할 계획이다.
  //
  // 예: 모든 축 동일 가정
  // ------------------------------------------------------------
  Eigen::Matrix<double, 6, 1> motor_rotor_inertia;
  motor_rotor_inertia << 4.75e-05, 4.75e-05, 2.52e-05, 2.52e-05, 1.41e-05, 1.41e-05;

  // reflected inertia lambda_i = N_i^2 * Jm_i
  Eigen::Matrix<double, 6, 1> lambda_diag =
      gear_ratio.array().square() * motor_rotor_inertia.array();

  // 비교 방향: base/world +X
  const Eigen::Vector3d u = Eigen::Vector3d::UnitX();

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Model name: " << model.name << "\n";
  std::cout << "model.nq: " << model.nq << ", model.nv: " << model.nv << "\n\n";

  // 자세: q = 0
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  // Jacobian / pose 업데이트
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // 기본 rigid-body M(q)
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd M_base = data.M;

  // 기어 reflected inertia 추가
  Eigen::MatrixXd M_aug = M_base;
  M_aug.diagonal() += lambda_diag;

  std::cout << "====================================================\n";
  std::cout << "gear_ratio:\n" << gear_ratio.transpose() << "\n\n";
  std::cout << "motor_rotor_inertia (assumed):\n"
            << motor_rotor_inertia.transpose() << "\n\n";
  std::cout << "lambda_diag = N^2 * Jm:\n"
            << lambda_diag.transpose() << "\n\n";
  std::cout << "M_base:\n" << M_base << "\n\n";
  std::cout << "M_aug = M_base + diag(lambda):\n" << M_aug << "\n";
  std::cout << "====================================================\n\n";

  Eigen::LDLT<Eigen::MatrixXd> ldlt_base(M_base);
  Eigen::LDLT<Eigen::MatrixXd> ldlt_aug(M_aug);

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
    const auto &inertia = model.inertias[joint_id];

    const double mass = inertia.mass();
    if (mass < 1e-9) {
      continue;
    }

    const Eigen::Vector3d com_local = inertia.lever();
    const Eigen::Matrix3d R_wf = data.oMf[frame_id].rotation();
    const Eigen::Vector3d p_wf = data.oMf[frame_id].translation();

    const Eigen::Vector3d r_world = R_wf * com_local;
    const Eigen::Vector3d p_com_world = p_wf + r_world;

    Eigen::MatrixXd J_frame(6, model.nv);
    J_frame.setZero();

    pinocchio::getFrameJacobian(
        model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // Pinocchio convention: [linear; angular]
    Eigen::MatrixXd Jv_o = J_frame.topRows(3);
    Eigen::MatrixXd Jw   = J_frame.bottomRows(3);

    // CoM point Jacobian
    Eigen::MatrixXd Jv_com = Jv_o - skew(r_world) * Jw;

    // baseline A
    Eigen::MatrixXd X_base = ldlt_base.solve(Jv_com.transpose());
    Eigen::Matrix3d A_base = Jv_com * X_base;

    // augmented A
    Eigen::MatrixXd X_aug = ldlt_aug.solve(Jv_com.transpose());
    Eigen::Matrix3d A_aug = Jv_com * X_aug;

    double meff_base = effectiveMassAlong(A_base, u);
    double meff_aug  = effectiveMassAlong(A_aug,  u);

    std::cout << "Body: " << std::setw(8) << std::left << link_name
              << " | Mass=" << std::setw(10) << mass
              << " | CoM(local)=[" << com_local.transpose() << "]"
              << " | CoM(world)=[" << p_com_world.transpose() << "]"
              << " | m_eff_base=" << meff_base
              << " | m_eff_with_gear=" << meff_aug
              << "\n";
  }

  return 0;
}