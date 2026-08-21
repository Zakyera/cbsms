// SPDX-License-Identifier: MIT
// Offline Stage 3A characterization harness. This file is not part of GLIM.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/scan_dcreg_diagnostics.hpp>
#include <nlohmann/json.hpp>

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using PointVector =
  std::vector<Eigen::Vector4d,
              Eigen::aligned_allocator<Eigen::Vector4d>>;
using CovVector =
  std::vector<Eigen::Matrix4d,
              Eigen::aligned_allocator<Eigen::Matrix4d>>;

struct Scene {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  std::string geometry_class;
  std::string expected_rotation_weak_subspace;
  std::string expected_translation_weak_subspace;
  PointVector target_points;
  PointVector source_points;
  CovVector target_covs;
  CovVector source_covs;
  Vector6d initial_perturbation = Vector6d::Zero();
  bool optimize_from_initial_perturbation = false;
};

Eigen::Matrix4d surfaceCovariance(const Eigen::Vector3d& normal) {
  const Eigen::Vector3d n = normal.normalized();
  const Eigen::Matrix3d tangent =
    Eigen::Matrix3d::Identity() - n * n.transpose();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  covariance.block<3, 3>(0, 0) =
    0.08 * 0.08 * tangent + 0.006 * 0.006 * n * n.transpose();
  return covariance;
}

void appendPatch(Scene* scene,
                 const Eigen::Vector3d& origin,
                 const Eigen::Vector3d& u,
                 const Eigen::Vector3d& v,
                 const Eigen::Vector3d& normal,
                 const int u_count,
                 const int v_count,
                 const double u_extent,
                 const double v_extent) {
  const Eigen::Vector3d unit_u = u.normalized();
  const Eigen::Vector3d unit_v = v.normalized();
  const Eigen::Matrix4d covariance = surfaceCovariance(normal);
  for (int row = 0; row < u_count; ++row) {
    const double alpha =
      u_count == 1 ? 0.0 : -1.0 + 2.0 * row / (u_count - 1.0);
    for (int column = 0; column < v_count; ++column) {
      const double beta =
        v_count == 1 ? 0.0 : -1.0 + 2.0 * column / (v_count - 1.0);
      Eigen::Vector4d point = Eigen::Vector4d::Ones();
      point.head<3>() =
        origin + alpha * u_extent * unit_u + beta * v_extent * unit_v;
      scene->target_points.push_back(point);
      scene->source_points.push_back(point);
      scene->target_covs.push_back(covariance);
      scene->source_covs.push_back(covariance);
    }
  }
}

void appendCylinder(Scene* scene,
                    const int axis_count,
                    const int angle_count,
                    const double half_length,
                    const double radius) {
  for (int axis_index = 0; axis_index < axis_count; ++axis_index) {
    const double x =
      -half_length + 2.0 * half_length * axis_index / (axis_count - 1.0);
    for (int angle_index = 0; angle_index < angle_count; ++angle_index) {
      const double angle =
        2.0 * M_PI * angle_index / static_cast<double>(angle_count);
      const Eigen::Vector3d normal(0.0, std::cos(angle), std::sin(angle));
      Eigen::Vector4d point = Eigen::Vector4d::Ones();
      point.head<3>() = Eigen::Vector3d(x, radius * normal.y(), radius * normal.z());
      scene->target_points.push_back(point);
      scene->source_points.push_back(point);
      scene->target_covs.push_back(surfaceCovariance(normal));
      scene->source_covs.push_back(surfaceCovariance(normal));
    }
  }
}

void appendIrregularPatches(Scene* scene, const int points_per_axis) {
  const std::vector<Eigen::Vector3d> normals = {
    {1.0, 0.1, 0.2},
    {-0.2, 1.0, 0.15},
    {0.1, -0.25, 1.0},
    {0.7, 0.6, 0.35},
    {-0.55, 0.45, 0.7},
    {0.35, -0.8, 0.5},
  };
  const std::vector<Eigen::Vector3d> centers = {
    {3.0, 0.0, 0.5},
    {-2.5, 1.5, -0.5},
    {0.0, -3.0, 1.0},
    {2.0, 2.5, 2.0},
    {-2.0, -2.0, 2.5},
    {1.0, -1.5, -2.0},
  };
  for (std::size_t index = 0; index < normals.size(); ++index) {
    const Eigen::Vector3d normal = normals[index].normalized();
    const Eigen::Vector3d seed =
      std::abs(normal.z()) < 0.8 ? Eigen::Vector3d::UnitZ()
                                  : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d u = normal.cross(seed).normalized();
    const Eigen::Vector3d v = normal.cross(u).normalized();
    appendPatch(scene,
                centers[index],
                u,
                v,
                normal,
                points_per_axis,
                points_per_axis,
                1.3,
                1.0);
  }
}

void applySourceNoise(Scene* scene, const double sigma, const std::uint32_t seed) {
  std::mt19937 generator(seed);
  std::normal_distribution<double> noise(0.0, sigma);
  for (auto& point : scene->source_points) {
    point.x() += noise(generator);
    point.y() += noise(generator);
    point.z() += noise(generator);
  }
}

void keepEveryOtherSourcePoint(Scene* scene) {
  PointVector points;
  CovVector covariances;
  for (std::size_t index = 0; index < scene->source_points.size(); ++index) {
    if (index % 2u == 0u) {
      points.push_back(scene->source_points[index]);
      covariances.push_back(scene->source_covs[index]);
    }
  }
  scene->source_points = std::move(points);
  scene->source_covs = std::move(covariances);
}

std::vector<Scene> makeScenes() {
  std::vector<Scene> scenes;

  Scene plane;
  plane.name = "single_plane";
  plane.geometry_class = "one planar surface";
  plane.expected_rotation_weak_subspace = "rz";
  plane.expected_translation_weak_subspace = "tx,ty";
  appendPatch(&plane,
              Eigen::Vector3d(0.0, 0.0, 3.0),
              Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitY(),
              Eigen::Vector3d::UnitZ(),
              25,
              25,
              4.0,
              4.0);
  scenes.push_back(plane);

  Scene corridor;
  corridor.name = "parallel_planes_corridor";
  corridor.geometry_class = "two parallel vertical planes";
  corridor.expected_rotation_weak_subspace = "rx";
  corridor.expected_translation_weak_subspace = "ty,tz";
  appendPatch(&corridor, {-3.0, 0.0, 0.0}, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 24, 20, 6.0, 2.5);
  appendPatch(&corridor, {3.0, 0.0, 0.0}, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX(), 24, 20, 6.0, 2.5);
  scenes.push_back(corridor);

  Scene tunnel;
  tunnel.name = "tunnel";
  tunnel.geometry_class = "cylindrical tunnel";
  tunnel.expected_rotation_weak_subspace = "rx";
  tunnel.expected_translation_weak_subspace = "tx";
  appendCylinder(&tunnel, 28, 36, 7.0, 3.0);
  scenes.push_back(tunnel);

  Scene corner;
  corner.name = "perpendicular_corner";
  corner.geometry_class = "two perpendicular planes";
  corner.expected_rotation_weak_subspace = "none";
  corner.expected_translation_weak_subspace = "tz";
  appendPatch(&corner, {0.0, 2.5, 0.0}, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 22, 18, 2.5, 2.5);
  appendPatch(&corner, {2.5, 0.0, 0.0}, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), 22, 18, 2.5, 2.5);
  scenes.push_back(corner);

  Scene orthogonal;
  orthogonal.name = "three_orthogonal_surfaces";
  orthogonal.geometry_class = "three approximately orthogonal planes";
  orthogonal.expected_rotation_weak_subspace = "none";
  orthogonal.expected_translation_weak_subspace = "none";
  appendPatch(&orthogonal, {0.0, 2.5, 2.5}, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 18, 18, 2.5, 2.5);
  appendPatch(&orthogonal, {2.5, 0.0, 2.5}, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), 18, 18, 2.5, 2.5);
  appendPatch(&orthogonal, {2.5, 2.5, 0.0}, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), 18, 18, 2.5, 2.5);
  scenes.push_back(orthogonal);

  Scene rich;
  rich.name = "rich_irregular_3d";
  rich.geometry_class = "six irregularly oriented surface patches";
  rich.expected_rotation_weak_subspace = "none";
  rich.expected_translation_weak_subspace = "none";
  appendIrregularPatches(&rich, 13);
  scenes.push_back(rich);

  Scene sparse;
  sparse.name = "sparse_rich_3d";
  sparse.geometry_class = "same rich geometry with sparse support";
  sparse.expected_rotation_weak_subspace = "none";
  sparse.expected_translation_weak_subspace = "none";
  appendIrregularPatches(&sparse, 5);
  scenes.push_back(sparse);

  Scene dense_plane;
  dense_plane.name = "dense_degenerate_plane";
  dense_plane.geometry_class = "dense one-plane geometry";
  dense_plane.expected_rotation_weak_subspace = "rz";
  dense_plane.expected_translation_weak_subspace = "tx,ty";
  appendPatch(&dense_plane, {0.0, 0.0, 3.0}, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), 48, 48, 4.0, 4.0);
  scenes.push_back(dense_plane);

  Scene noisy = rich;
  noisy.name = "rich_3d_noisy";
  noisy.geometry_class = "rich geometry with deterministic 2 cm source noise";
  applySourceNoise(&noisy, 0.02, 271828u);
  noisy.initial_perturbation =
    (Vector6d() << 0.008, -0.006, 0.005, 0.025, -0.018, 0.012).finished();
  noisy.optimize_from_initial_perturbation = true;
  scenes.push_back(noisy);

  Scene low_overlap = rich;
  low_overlap.name = "rich_3d_half_overlap";
  low_overlap.geometry_class = "rich geometry with half source support";
  keepEveryOtherSourcePoint(&low_overlap);
  scenes.push_back(low_overlap);

  Scene large_initial = rich;
  large_initial.name = "rich_3d_large_initial_perturbation";
  large_initial.geometry_class = "rich geometry initialized away from the known solution";
  large_initial.initial_perturbation =
    (Vector6d() << 0.025, -0.020, 0.018, 0.10, -0.07, 0.06).finished();
  large_initial.optimize_from_initial_perturbation = true;
  scenes.push_back(large_initial);

  Scene tilted_plane;
  tilted_plane.name = "tilted_offset_plane";
  tilted_plane.geometry_class = "tilted plane at long range";
  tilted_plane.expected_rotation_weak_subspace = "surface-normal rotation";
  tilted_plane.expected_translation_weak_subspace = "two in-plane translations";
  const Eigen::Vector3d tilted_normal = Eigen::Vector3d(0.4, -0.3, 0.8660254).normalized();
  const Eigen::Vector3d tilted_u = tilted_normal.cross(Eigen::Vector3d::UnitY()).normalized();
  const Eigen::Vector3d tilted_v = tilted_normal.cross(tilted_u).normalized();
  appendPatch(&tilted_plane, {8.0, -3.0, 5.0}, tilted_u, tilted_v, tilted_normal, 25, 25, 4.0, 4.0);
  scenes.push_back(tilted_plane);

  return scenes;
}

Matrix6d extractHessian(const gtsam::NonlinearFactor::shared_ptr& factor,
                        const gtsam::Values& values,
                        const gtsam::Key key) {
  const auto linearized = factor->linearize(values);
  gtsam::GaussianFactorGraph graph;
  graph.push_back(linearized);
  const auto blocks = graph.hessianBlockDiagonal();
  const auto found = blocks.find(key);
  if (found == blocks.end() || found->second.rows() != 6 || found->second.cols() != 6) {
    throw std::runtime_error("unary Hessian block unavailable");
  }
  const Matrix6d raw = found->second.cast<double>();
  return 0.5 * (raw + raw.transpose());
}

nlohmann::json vectorJson(const Eigen::VectorXd& vector) {
  nlohmann::json result = nlohmann::json::array();
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    result.push_back(vector[index]);
  }
  return result;
}

template <typename Derived>
nlohmann::json matrixJson(const Eigen::MatrixBase<Derived>& matrix) {
  nlohmann::json result = nlohmann::json::array();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    nlohmann::json values = nlohmann::json::array();
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      values.push_back(matrix(row, column));
    }
    result.push_back(values);
  }
  return result;
}

nlohmann::json analyzeScene(const Scene& scene) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  auto target = std::make_shared<gtsam_points::PointCloudCPU>();
  target->add_points(scene.target_points);
  target->add_covs(scene.target_covs);
  auto source = std::make_shared<gtsam_points::PointCloudCPU>();
  source->add_points(scene.source_points);
  source->add_covs(scene.source_covs);

  gtsam::NonlinearFactorGraph graph;
  std::vector<
    gtsam_points::shared_ptr<gtsam_points::IntegratedVGICPFactor>> factors;
  for (const double resolution : {0.5, 1.0}) {
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    voxelmap->insert(*target);
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
      gtsam::Pose3(), key, voxelmap, source);
    factor->set_num_threads(1);
    factors.push_back(factor);
    graph.add(factor);
  }

  gtsam::Values initial;
  initial.insert(key, gtsam::Pose3::Expmap(scene.initial_perturbation));
  const double initial_cost = graph.error(initial);

  gtsam_points::LevenbergMarquardtExtParams parameters;
  parameters.setMaxIterations(30);
  parameters.setAbsoluteErrorTol(1.0e-8);
  parameters.setRelativeErrorTol(1.0e-8);
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, initial, parameters);
  const gtsam::Values final_values =
    scene.optimize_from_initial_perturbation ? optimizer.optimize() : initial;
  const double final_cost = graph.error(final_values);

  Matrix6d hessian = Matrix6d::Zero();
  int minimum_inliers = std::numeric_limits<int>::max();
  std::vector<nlohmann::json> per_resolution;
  for (std::size_t index = 0; index < factors.size(); ++index) {
    const Matrix6d factor_hessian = extractHessian(factors[index], final_values, key);
    hessian += factor_hessian;
    const int inliers = factors[index]->num_inliers();
    minimum_inliers = std::min(minimum_inliers, inliers);
    per_resolution.push_back({
      {"resolution_m", index == 0 ? 0.5 : 1.0},
      {"inliers", inliers},
      {"inlier_fraction", factors[index]->inlier_fraction()},
      {"hessian", matrixJson(factor_hessian)},
    });
  }

  glim::ScanDcregDetectionConfig config;
  const auto diagnostics = glim::analyzeScanDcregHessian(hessian, config);
  const auto metrics = glim::computeScanDcregHessianMetrics(
    hessian,
    static_cast<int>(scene.source_points.size()),
    minimum_inliers,
    config);
  Eigen::SelfAdjointEigenSolver<Matrix6d> full_solver(hessian);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_block_solver(
    hessian.block<3, 3>(0, 0));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_block_solver(
    hessian.block<3, 3>(3, 3));

  nlohmann::json points = nlohmann::json::array();
  const std::size_t stride = std::max<std::size_t>(1u, scene.target_points.size() / 500u);
  for (std::size_t index = 0; index < scene.target_points.size(); index += stride) {
    points.push_back({scene.target_points[index].x(),
                      scene.target_points[index].y(),
                      scene.target_points[index].z()});
  }

  const gtsam::Vector6 final_pose =
    gtsam::Pose3::Logmap(final_values.at<gtsam::Pose3>(key));
  return {
    {"name", scene.name},
    {"geometry_class", scene.geometry_class},
    {"expected_rotation_weak_subspace", scene.expected_rotation_weak_subspace},
    {"expected_translation_weak_subspace", scene.expected_translation_weak_subspace},
    {"target_point_count", scene.target_points.size()},
    {"source_point_count", scene.source_points.size()},
    {"point_sample", points},
    {"initial_pose_log", vectorJson(scene.initial_perturbation)},
    {"final_pose_log", vectorJson(final_pose)},
    {"initial_cost", initial_cost},
    {"final_cost", final_cost},
    {"minimum_inlier_count", minimum_inliers},
    {"minimum_inlier_fraction", minimum_inliers / static_cast<double>(scene.source_points.size())},
    {"per_resolution", per_resolution},
    {"hessian", matrixJson(hessian)},
    {"full_hessian_eigenvalues", vectorJson(full_solver.eigenvalues())},
    {"full_hessian_condition", metrics.condition},
    {"full_hessian_minimum_eigenvalue", metrics.minimum_eigenvalue},
    {"full_hessian_rank", metrics.rank},
    {"rotation_block_eigenvalues", vectorJson(rotation_block_solver.eigenvalues())},
    {"translation_block_eigenvalues", vectorJson(translation_block_solver.eigenvalues())},
    {"rotation_schur_eigenvalues", vectorJson(diagnostics.raw_rotation_eigenvalues)},
    {"translation_schur_eigenvalues", vectorJson(diagnostics.raw_translation_eigenvalues)},
    {"rotation_schur_eigenvectors", matrixJson(diagnostics.raw_rotation_eigenvectors)},
    {"translation_schur_eigenvectors", matrixJson(diagnostics.raw_translation_eigenvectors)},
    {"rotation_condition_ratios", vectorJson(diagnostics.rotation_condition_ratios)},
    {"translation_condition_ratios", vectorJson(diagnostics.translation_condition_ratios)},
    {"kappa_rotation", diagnostics.rotation_condition_ratios.maxCoeff()},
    {"kappa_translation", diagnostics.translation_condition_ratios.maxCoeff()},
    {"rotation_absolute_mask", vectorJson(diagnostics.absolute_rotation_degenerate_mask.cast<double>().matrix())},
    {"translation_absolute_mask", vectorJson(diagnostics.absolute_translation_degenerate_mask.cast<double>().matrix())},
    {"rotation_alignment_confidence", vectorJson(diagnostics.rotation_alignment_confidence)},
    {"translation_alignment_confidence", vectorJson(diagnostics.translation_alignment_confidence)},
    {"rotation_cluster_flags", vectorJson(diagnostics.rotation_spectral_cluster_flags.cast<double>().matrix())},
    {"translation_cluster_flags", vectorJson(diagnostics.translation_spectral_cluster_flags.cast<double>().matrix())},
    {"valid", diagnostics.valid},
    {"factorization_ok", diagnostics.factorization_ok},
  };
}

nlohmann::json coupledExample() {
  Matrix6d hessian = Matrix6d::Zero();
  const Eigen::Matrix3d diagonal =
    (Eigen::Vector3d(100.0, 50.0, 20.0)).asDiagonal();
  const Eigen::Matrix3d coupling =
    (Eigen::Vector3d(99.9, 0.0, 0.0)).asDiagonal();
  hessian.block<3, 3>(0, 0) = diagonal;
  hessian.block<3, 3>(3, 3) = diagonal;
  hessian.block<3, 3>(0, 3) = coupling;
  hessian.block<3, 3>(3, 0) = coupling.transpose();
  glim::ScanDcregDetectionConfig config;
  const auto diagnostics = glim::analyzeScanDcregHessian(hessian, config);
  return {
    {"description", "PSD coupled quadratic with individually well-conditioned diagonal blocks"},
    {"hessian", matrixJson(hessian)},
    {"rotation_block_eigenvalues", {20.0, 50.0, 100.0}},
    {"translation_block_eigenvalues", {20.0, 50.0, 100.0}},
    {"rotation_schur_eigenvalues", vectorJson(diagnostics.raw_rotation_eigenvalues)},
    {"translation_schur_eigenvalues", vectorJson(diagnostics.raw_translation_eigenvalues)},
    {"kappa_rotation", diagnostics.rotation_condition_ratios.maxCoeff()},
    {"kappa_translation", diagnostics.translation_condition_ratios.maxCoeff()},
  };
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: dcreg_stage3a_vgicp_harness OUTPUT.json\n";
    return 2;
  }
  try {
    nlohmann::json output = {
      {"schema_version", 1},
      {"seed", 314159},
      {"tangent_ordering", "GTSAM Pose3 local right [rx,ry,rz,tx,ty,tz]"},
      {"vgicp_resolutions_m", {0.5, 1.0}},
      {"dcreg_threshold", 10.0},
      {"scenes", nlohmann::json::array()},
      {"coupled_quadratic", coupledExample()},
    };
    for (const Scene& scene : makeScenes()) {
      std::cerr << "Analyzing " << scene.name << "\n";
      output["scenes"].push_back(analyzeScene(scene));
    }
    std::ofstream stream(argv[1]);
    stream << std::setprecision(17) << output.dump(2) << '\n';
    if (!stream) {
      throw std::runtime_error("failed to write output");
    }
  } catch (const std::exception& exception) {
    std::cerr << "Stage 3A VGICP harness failed: " << exception.what() << '\n';
    return 1;
  }
  return 0;
}
