#include <gtsam/geometry/Pose3.h>
#include <Eigen/SVD>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

gtsam::Pose3 releasedE() {
  Eigen::Matrix3d raw;
  raw << 0.999620, 0.027463, 0.002445,
        -0.027517, 0.999299, 0.025398,
        -0.001746, -0.025456, 0.999674;
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d v = svd.matrixV();
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (u * v.transpose()).determinant();
  const Eigen::Matrix3d rotation = u * correction * v.transpose();
  return gtsam::Pose3(gtsam::Rot3(rotation),
                      gtsam::Point3(0.049258, -0.012500, 0.026946));
}

gtsam::Pose3 releasedS(double quaternion_sign = 1.0) {
  const Eigen::Quaterniond q(
    quaternion_sign * 0.9998828,
    quaternion_sign * -0.0057758,
    quaternion_sign * 0.0022253,
    quaternion_sign * 0.0140019);
  return gtsam::Pose3(gtsam::Rot3(q.normalized()),
                      gtsam::Point3(0.0305, -0.5959, 0.0902));
}

double localDifference(const gtsam::Pose3& first, const gtsam::Pose3& second) {
  return gtsam::Pose3::Logmap(first.between(second)).norm();
}

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  try {
    const gtsam::Pose3 E = releasedE();
    const gtsam::Pose3 S = releasedS();
    const gtsam::Pose3 world_alignment(
      gtsam::Rot3::RzRyRx(0.2, -0.1, 0.3), gtsam::Point3(5.0, -2.0, 1.0));
    const gtsam::Pose3 T_W_I_i(
      gtsam::Rot3::RzRyRx(-0.3, 0.2, 0.1), gtsam::Point3(1.2, -0.7, 0.4));
    const gtsam::Pose3 T_W_I_j(
      gtsam::Rot3::RzRyRx(0.4, -0.25, 0.35), gtsam::Point3(2.1, 0.8, -0.2));

    const auto endpoint_path = [&](const gtsam::Pose3& left) {
      const gtsam::Pose3 T_W_B_i = left * T_W_I_i * E * S.inverse();
      const gtsam::Pose3 T_W_B_j = left * T_W_I_j * E * S.inverse();
      return T_W_B_i.between(T_W_B_j);
    };
    const gtsam::Pose3 Z_I = T_W_I_i.between(T_W_I_j);
    const gtsam::Vector6 payload = gtsam::Pose3::Logmap(Z_I);
    const gtsam::Pose3 decoded_payload = gtsam::Pose3::Expmap(payload);
    const gtsam::Pose3 Z_L = E.inverse() * Z_I * E;
    const gtsam::Pose3 Z_B = S * Z_L * S.inverse();
    const gtsam::Pose3 Z_endpoint = endpoint_path(gtsam::Pose3());

    const double path_difference = localDifference(Z_endpoint, Z_B);
    const double belief_payload_difference = localDifference(Z_I, decoded_payload);
    const double world_difference = localDifference(
      Z_endpoint, endpoint_path(world_alignment));
    const double sign_difference = localDifference(S, releasedS(-1.0));
    const double omit_e_difference = localDifference(
      Z_B, S * Z_I * S.inverse());
    const double omit_s_difference = localDifference(Z_B, Z_L);

    require(path_difference <= 1.0e-12,
            "endpoint and edge-conjugation paths disagree");
    require(belief_payload_difference <= 1.0e-12,
            "G-to-K Logmap payload does not reconstruct endpoint edge");
    require(world_difference <= 1.0e-12,
            "constant left world alignment did not cancel");
    require(sign_difference <= 1.0e-12,
            "quaternion sign changed the transform");
    require(omit_e_difference > 1.0e-6,
            "omitting E unexpectedly preserved the edge");
    require(omit_s_difference > 1.0e-6,
            "omitting S unexpectedly preserved the edge");

    std::cout.precision(17);
    std::cout << "path_difference=" << path_difference << '\n'
              << "belief_payload_difference=" << belief_payload_difference << '\n'
              << "world_alignment_difference=" << world_difference << '\n'
              << "quaternion_sign_difference=" << sign_difference << '\n'
              << "omit_E_difference=" << omit_e_difference << '\n'
              << "omit_S_difference=" << omit_s_difference << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
