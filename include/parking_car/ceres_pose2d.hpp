#ifndef CERES_POSE2D_HPP_
#define CERES_POSE2D_HPP_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/solver.h"
// The state for each vertex in the pose graph.
struct Pose2d {
//  Pose2d(double _x, double _y, double _th):x(_x), y(_y), yaw_radians(_th)
//  {}
//  Pose2d()
//  {}


  double x;
  double y;
  double yaw_radians;
};

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint2d {
  int id_begin;
  int id_end;

  double x;
  double y;
  double yaw_radians;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, and yaw.
  Eigen::Matrix3d information;
};


template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
      two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}


class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement.
//
// residual =  information^{1/2} * [  r_a^T * (p_b - p_a) - \hat{p_ab}   ]
//                                 [ Normalize(yaw_b - yaw_a - \hat{yaw_ab}) ]
//
// where r_a is the rotation matrix that rotates a vector represented in frame A
// into the global frame, and Normalize(*) ensures the angles are in the range
// [-pi, pi).
class PoseGraph2dErrorTerm {
 public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians,
                       const Eigen::Matrix3d& sqrt_information)
      : p_ab_(x_ab, y_ab),
        yaw_ab_radians_(yaw_ab_radians),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);

    residuals_map.template head<2>() =
        RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) -
        p_ab_.cast<T>();
    residuals_map(2) = NormalizeAngle(
        (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

    // Scale the residuals by the square root information matrix to account for
    // the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(double x_ab, double y_ab,
                                     double yaw_ab_radians,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1,
                                            1, 1>(new PoseGraph2dErrorTerm(
        x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                              std::map<int, Pose2d>* poses,
                              ceres::Problem* problem) {
  CHECK(poses != NULL);
  CHECK(problem != NULL);
  if (constraints.empty()) {
    std::cout << "No constraints, no problem to optimize.\n";
    return;
  }

  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization* angle_local_parameterization =
      AngleLocalParameterization::Create();

  for (std::vector<Constraint2d>::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const Constraint2d& constraint = *constraints_iter;

    std::map<int, Pose2d>::iterator pose_begin_iter =
        poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    std::map<int, Pose2d>::iterator pose_end_iter =
        poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix3d sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
        constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
    problem->AddResidualBlock(
        cost_function, loss_function, &pose_begin_iter->second.x,
        &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
        &pose_end_iter->second.x, &pose_end_iter->second.y,
        &pose_end_iter->second.yaw_radians);

    problem->SetParameterization(&pose_begin_iter->second.yaw_radians,
                                angle_local_parameterization);
    problem->SetParameterization(&pose_end_iter->second.yaw_radians,
                                angle_local_parameterization);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  std::map<int, Pose2d>::iterator pose_start_iter =
      poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: ID x y yaw_radians.
bool OutputPoses(const std::string& filename,
                 const std::map<int, Pose2d>& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    std::cerr << "Error opening the file: " << filename << '\n';
    return false;
  }
  for (std::map<int, Pose2d>::const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose2d>::value_type& pair = *poses_iter;
    outfile <<  pair.first << " " << pair.second.x << " " << pair.second.y
            << ' ' << pair.second.yaw_radians << '\n';
  }
  return true;
}




#endif;
