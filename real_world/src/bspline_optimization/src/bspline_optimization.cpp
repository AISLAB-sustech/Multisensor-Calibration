#include <bspline_optimization.hpp>
#include <chrono>
#include <fstream>
#include <iostream>

namespace bspline_ekf {

BSplineOptimization::BSplineOptimization() {}

BSplineOptimization::~BSplineOptimization() {}

void BSplineOptimization::optimize(double delta, ExtPara ext_para,
                                   int curve_seg, vector<double> pre_points,
                                   vector<double> &result_points,
                                   vector<SegmentedPathPoint> &path) {
  vector<double> control_points;
  OptimizeData optimize_data;
  OptimizeData *optimize_data_ptr = &optimize_data;
  optimize_data_ptr->pre_points = pre_points;
  optimize_data_ptr->control_points = control_points;
  optimize_data_ptr->ext_para = ext_para;
  if (curve_seg == 1) {
    optimize_data_ptr->control_points.assign(pre_points.begin() + 2,
                                             pre_points.end());
    optimize_data_ptr->pre_points =
        vector<double>{pre_points[0], pre_points[1]};
  } else {
    optimize_data_ptr->control_points.assign(
        pre_points.begin() + curve_seg * 2 + 4, pre_points.end());
    optimize_data_ptr->pre_points.assign(
        pre_points.begin(), pre_points.begin() + curve_seg * 2 + 4);
  }
  nlopt::opt opt(nlopt::LN_COBYLA, optimize_data_ptr->control_points.size());
  std::vector<double> lb(optimize_data_ptr->control_points.size(), 0.0);
  opt.set_lower_bounds(lb);
  std::vector<double> ub(optimize_data_ptr->control_points.size(), 2.0);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(max_negative_min_eigenvalue, optimize_data_ptr);
  opt.set_xtol_rel(1e-2);
  opt.set_xtol_abs(1e-4);
  opt.add_inequality_constraint(BSplineOptimization::constraint_angle,
                                optimize_data_ptr, 1e-8);
  // opt.add_inequality_constraint(BSplineOptimization::constraint_pos1,
  //                               all_points_ptr, 1e-8);
  // opt.add_inequality_constraint(BSplineOptimization::constraint_pos2,
  //                               all_points_ptr, 1e-8);

  double minf;
  try {
    auto tic = std::chrono::high_resolution_clock::now();
    nlopt::result result =
        opt.optimize(optimize_data_ptr->control_points, minf);
    auto toc = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic)
            .count();
    std::cout << "Found minimum at f(";
    for (size_t i = 0; i < optimize_data_ptr->control_points.size(); ++i) {
      std::cout << optimize_data_ptr->control_points[i];
      if (i < optimize_data_ptr->control_points.size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << ") = " << minf << " using " << duration << " ms." << std::endl;
  } catch (std::exception &e) {
    std::cerr << "nlopt failed: " << e.what() << std::endl;
  }

  // return the optimized control points
  result_points = optimize_data_ptr->pre_points;
  result_points.insert(result_points.end(),
                       optimize_data_ptr->control_points.begin(),
                       optimize_data_ptr->control_points.end());

  // return the optimized path
  bspline(result_points, delta, path);
}

double BSplineOptimization::max_negative_min_eigenvalue(
    const std::vector<double> &control_points, std::vector<double> &grad,
    void *optimize_data_ptr) {
  vector<Vector2d> v_set;
  vector<SegmentedPathPoint> path;
  double delta = 0.2;
  if (!optimize_data_ptr) {
    std::cerr << "Error: ext_para_ptr is nullptr." << std::endl;
    return 0;
  }
  OptimizeData *optimize_data = static_cast<OptimizeData *>(optimize_data_ptr);

  vector<double> result_points = optimize_data->pre_points;
  result_points.insert(result_points.end(), control_points.begin(),
                       control_points.end());

  bspline(result_points, delta, path);
  size_t time_step = path.size();
  MatrixXd full_jac = MatrixXd::Zero(8 * time_step - 8, 3 * time_step + 11);
  vector<Eigen::MatrixXd> opt_jac_vec;
  for (int i = 0; i < path.size() - 1; i++) {
    Vector2d x_i = path[i].point;
    double theta_i = path[i].yaw_angle;
    Vector2d x_iplus1 = path[i + 1].point;
    double theta_iplus1 = path[i + 1].yaw_angle;
    MatrixXd T_i;
    jacobian(optimize_data->ext_para, x_i, x_iplus1, theta_i, theta_iplus1,
             T_i);
    // std::cout << "T_i: " << T_i << std::endl;
    opt_jac_vec.push_back(T_i);
  }
  MatrixXd opt_jac = v_merge(opt_jac_vec);
  // std::cout << "opt_jac: " << opt_jac << std::endl;
  full_jac = opt_jac.transpose() * opt_jac;
  // std::cout << "Full_jac: " << full_jac << std::endl;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(full_jac, Eigen::ComputeFullU |
                                                      Eigen::ComputeFullV);
  auto s = svd.singularValues();

  // double tolerance =
  //     1e-10 * std::max(full_jac.cols(), full_jac.rows()) * s.maxCoeff();
  // int rank = 0;
  // for (int i = 0; i < s.size(); ++i) {
  //   if (s(i) > tolerance) {
  //     rank++;
  //   }
  // }
  // std::cout << "Rank of Full_jac: " << rank << std::endl;
  double min_eigenvalue = s.minCoeff();
  return -s.minCoeff();
}

void BSplineOptimization::bspline(const vector<double> &control_points,
                                  double delta,
                                  vector<SegmentedPathPoint> &path) {
  vector<SegmentedPathPoint> temp_path;
  int k = 3;
  Matrix4d M0, M1, M_n_1, M_n, M_else;
  // clang-format off
  M0 << 1, 0, 0, 0,
     -3.0, 3.0, 0, 0,
     3.0, -9.0 / 2.0, 3.0 / 2.0, 0,
     -1, 7.0 / 4.0, -11.0 / 12.0, 1.0 / 6.0;
  M1 << 1.0 / 4.0, 7.0 / 12.0, 1.0 / 6.0, 0, 
  -3.0 / 4.0, 1.0 / 4.0, 1.0 / 2.0, 0,
   3.0 / 4.0, -5.0 / 4.0, 1.0 / 2.0, 0,
    -1.0 / 4.0, 7.0 / 12.0, -1.0 / 2.0, 1.0 / 6.0;
  M_n_1 << 1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0, 0, 
  -1.0 / 2.0, 0, 1.0 / 2.0, 0, 
  1.0 / 2.0, -1, 1.0 / 2.0, 0,
   -1.0 / 6.0, 1.0 / 2.0, -7.0 / 12.0,1.0 / 4.0;
  M_n << 1.0 / 6.0, 7.0 / 12.0, 1.0 / 4.0, 0, 
  -1.0 / 2.0, -1.0 / 4.0, 3.0 / 4.0, 0, 
  1.0 / 2.0, -5.0 / 4.0, 3.0 / 4.0, 0,
   -1.0 / 6.0, 11.0 / 12.0, -7.0 / 4.0, 1.0;
  M_else << 1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0, 0,
   -1.0 / 2.0, 0, 1.0 / 2.0, 0,
      1.0 / 2.0, -1, 1.0 / 2.0, 0,
       -1.0 / 6.0, 1.0 / 2.0, -1.0 / 2.0, 1.0 / 6.0;
  // clang-format on
  for (int segment = 1; segment < control_points.size() / 2 - k + 1;
       segment++) {
    Matrix4d M;
    if (segment == 1)
      M = M0;
    else if (segment == 2)
      M = M1;
    else if (segment == control_points.size() / 2 - k - 1)
      M = M_n_1;
    else if (segment == control_points.size() / 2 - k)
      M = M_n;
    else
      M = M_else;
    Eigen::Matrix<double, 4, 2> control_point;
    for (int i = segment - 1, j = 0; i < segment + k; i++) {
      control_point(j, 0) = control_points[2 * i];
      control_point(j, 1) = control_points[2 * i + 1];
      j++;
    }
    for (double i = 0; i < 1; i += delta) {
      Eigen::Vector2d p =
          (Eigen::Vector4d() << 1, i, std::pow(i, 2), std::pow(i, 3))
              .finished()
              .transpose() *
          M * control_point;
      Eigen::Vector2d v = (Eigen::Vector4d() << 0, 1, 2 * i, 3 * (pow(i, 2)))
                              .finished()
                              .transpose() *
                          M * control_point;
      // std::cout << "v: " << v << std::endl;
      // std::cout << "M: " << M << std::endl;
      // std::cout << "control_point: " << control_point << std::endl;
      // std::cout
      //     << "Eigen::Vector4d: "
      //     << (Eigen::Vector4d() << 0, 1, 2 * i, 3 * (pow(i, 2))).finished()
      //     << std::endl;
      Eigen::Vector2d a =
          (Eigen::Vector4d() << 0, 0, 2, 6 * i).finished().transpose() * M *
          control_point;
      double yaw = atan2f(v(1), v(0));
      // v_set.push_back(v);
      // a_set.push_back(a);
      temp_path.emplace_back(SegmentedPathPoint{p, yaw, segment});
    }
  }
  path = temp_path;
}
MatrixXd BSplineOptimization::h_merge(const vector<MatrixXd> &matrices) {
  int total_cols = 0;
  int rows = matrices[0].rows();
  for (const auto &mat : matrices) {
    total_cols += mat.cols();
  }

  MatrixXd result(rows, total_cols);
  int col_start = 0;
  for (const auto &mat : matrices) {
    result.block(0, col_start, mat.rows(), mat.cols()) = mat;
    col_start += mat.cols();
  }
  return result;
}

MatrixXd BSplineOptimization::v_merge(const vector<MatrixXd> &matrices) {
  int total_rows = 0;
  int cols = matrices[0].cols();
  for (const auto &mat : matrices) {
    total_rows += mat.rows();
  }

  MatrixXd result(total_rows, cols);
  int row_start = 0;
  for (const auto &mat : matrices) {
    result.block(row_start, 0, mat.rows(), mat.cols()) = mat;
    row_start += mat.rows();
  }
  return result;
}

Matrix2d BSplineOptimization::rot_mat(double theta, bool de) {
  Matrix2d matrix;
  if (de) {
    matrix << -sin(theta), -cos(theta), cos(theta), -sin(theta);
  } else {
    matrix << cos(theta), -sin(theta), sin(theta), cos(theta);
  }
  return matrix;
}

void BSplineOptimization::jacobian(const ExtPara &ext_para, const Vector2d &x_i,
                                   const Vector2d &x_iplus1, double theta_i,
                                   double theta_iplus1, MatrixXd &T_i) {
  Vector2d SRC = {ext_para.SRC_x, ext_para.SRC_y};
  Vector2d p_M = {ext_para.M_x, ext_para.M_y};
  Vector2d p_L = {ext_para.L_x, ext_para.L_y};
  double theta_M = ext_para.M_theta;
  double theta_L = ext_para.L_theta;
  Matrix2d R_i = rot_mat(theta_i);
  Matrix2d R_iplus1 = rot_mat(theta_iplus1);
  Vector2d delta_M_i = SRC - (x_i + R_i * p_M);
  Vector2d delta_M_iplus1 = SRC - (x_iplus1 + R_iplus1 * p_M);

  // Matrix2f mic_partial_x =
  //     -(R_i * rot_mat(theta_M).transpose() / std::pow(delta_M_i.norm(), 3)) *
  //     (Matrix2f() << delta_M_i(1) * delta_M_i(1), -delta_M_i(0) *
  //     delta_M_i(1),
  //      -delta_M_i(0) * delta_M_i(1), delta_M_i(0) * delta_M_i(0))
  //         .finished();
  // Vector2d mic_partial_theta =
  //     rot_mat(theta_M).transpose() / pow(delta_M_i.norm(), 3) *
  //     (pow(delta_M_i.norm(), 2) * rot_mat(theta_i, true).transpose() *
  //          (SRC - x_i) +
  //      (R_i.transpose() * (SRC - x_i) - p_M) * delta_M_i.transpose() *
  //          rot_mat(theta_i, true) * p_M);
  // MatrixXd mic_partial = h_merge({mic_partial_x, mic_partial_theta});

  // Matrix2d mic_partial_xplus1 =
  //     -(R_iplus1 * rot_mat(theta_M).transpose() /
  //       pow(delta_M_iplus1.norm(), 3)) *
  //     (Matrix2f() << delta_M_iplus1(1) * delta_M_iplus1(1),
  //      delta_M_iplus1(0) * delta_M_iplus1(1),
  //      delta_M_iplus1(0) * delta_M_iplus1(1),
  //      delta_M_iplus1(0) * delta_M_iplus1(0))
  //         .finished();
  // Vector2d mic_partial_thetaplus1 =
  //     rot_mat(theta_M).transpose() / pow(delta_M_iplus1.norm(), 3) *
  //     (pow(delta_M_iplus1.norm(), 2) * rot_mat(theta_iplus1,
  //     true).transpose() *
  //          (SRC - x_iplus1) +
  //      (R_iplus1.transpose() * (SRC - x_iplus1) - p_M) *
  //          delta_M_iplus1.transpose() * rot_mat(theta_iplus1, true) * p_M);
  // MatrixXd mic_partial_plus1 =
  //     h_merge({mic_partial_xplus1, mic_partial_thetaplus1});

  // Matrix2f pos_partial_x = -(R_i * rot_mat(theta_L)).transpose();
  // Vector2d pos_partial_theta = rot_mat(theta_L).transpose() *
  //                              rot_mat(theta_i, true).transpose() *
  //                              (rot_mat(theta_iplus1) * p_L + x_iplus1 -
  //                              x_i);
  // MatrixXd pos_partial = h_merge({pos_partial_x, pos_partial_theta});
  // Eigen::RowVector3d the_partial_x = {0, 0, 1};

  // Matrix2f pos_partial_xplus1 = (R_i * rot_mat(theta_L)).transpose();
  // Vector2d pos_partial_thetaplus1 =
  //     (R_i * rot_mat(theta_L)).transpose() * rot_mat(theta_iplus1, true) *
  //     p_L;
  // MatrixXd pos_partial_plus1 =
  //     h_merge({pos_partial_xplus1, pos_partial_thetaplus1});
  // Eigen::RowVector3d the_partial_xplus1 = {0, 0, 1};

  T_i = Eigen::MatrixXd::Zero(5, 8);
  // w.r.t MIC parameters
  T_i.block<2, 2>(0, 0) =
      (R_i * rot_mat(theta_M)).transpose() / std::pow(delta_M_i.norm(), 3) *
      (-R_i * std::pow(delta_M_i.norm(), 2) +
       (Matrix2d() << std::pow(delta_M_i(0), 2), delta_M_i(0) * delta_M_i(1),
        delta_M_i(0) * delta_M_i(1), std::pow(delta_M_i(1), 2))
               .finished() *
           R_i);
  T_i.block<2, 1>(0, 2) = rot_mat(theta_M, true).transpose() * R_i.transpose() *
                          delta_M_i / delta_M_i.norm();
  // w.r.t src
  T_i.block<2, 2>(0, 3) =
      (R_i * rot_mat(theta_M)).transpose() / std::pow(delta_M_i.norm(), 3) *
      (Matrix2d() << std::pow(delta_M_i(1), 2), -delta_M_i(0) * delta_M_i(1),
       -delta_M_i(0) * delta_M_i(1), std::pow(delta_M_i(0), 2))
          .finished();
  // w.r.t Lidar parameters
  // std::cout << R_i << std::endl;
  // std::cout << rot_mat(theta_L) << std::endl;
  // std::cout << R_i * rot_mat(theta_L) << std::endl;
  // std::cout << R_iplus1 << std::endl;
  // std::cout << R_iplus1 - R_i << std::endl;
  // std::cout << (R_i * rot_mat(theta_L)).transpose() * (R_iplus1 - R_i)
  //           << std::endl;
  T_i.block<2, 2>(2, 5) =
      (R_i * rot_mat(theta_L)).transpose() * (R_iplus1 - R_i);
  T_i.block<2, 1>(2, 7) = rot_mat(theta_L, true).transpose() * R_i.transpose() *
                          (R_iplus1 * p_L + x_iplus1 - R_i * p_L - x_i);
}

double
BSplineOptimization::constraint_angle(const std::vector<double> &control_points,
                                      std::vector<double> &grad,
                                      void *optimize_data_ptr) {
  static std::ofstream debug_file("yaw_debug.log",
                                  std::ios::out | std::ios::app);
  OptimizeData optimize_data = *static_cast<OptimizeData *>(optimize_data_ptr);
  double delta = 0.3;
  vector<double> result_points = optimize_data.pre_points;
  result_points.insert(result_points.end(),
                       optimize_data.control_points.begin(),
                       optimize_data.control_points.end());
  vector<SegmentedPathPoint> path;
  bspline(result_points, delta, path);
  debug_file << "Control Points: ";
  for (const auto &cp : control_points) {
    debug_file << cp << " ";
  }
  debug_file << "\n";
  vector<double> constraints;
  for (size_t i = 1; i < path.size(); ++i) {
    double yaw_diff = path[i].yaw_angle - path[i - 1].yaw_angle;
    while (yaw_diff > M_PI)
      yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI)
      yaw_diff += 2 * M_PI;
    double constraint_value = std::abs(yaw_diff) - 50.0 / 180.0 * M_PI;
    constraints.push_back(constraint_value);
    debug_file << "Iteration " << i
               << ": yaw_angle[i-1] = " << path[i - 1].yaw_angle
               << ", yaw_angle[i] = " << path[i].yaw_angle
               << ", yaw_diff = " << yaw_diff
               << ", constraint_value = " << constraint_value << std::endl;
  }

  double max_constraint =
      *std::max_element(constraints.begin(), constraints.end());

  debug_file << "Maximum constraint value in this iteration: " << max_constraint
             << std::endl;
  debug_file << "-----------------------------------------------" << std::endl;
  return max_constraint;
}

double
BSplineOptimization::constraint_pos1(const std::vector<double> &control_points,
                                     std::vector<double> &grad, void *data) {
  std::vector<double> constraints;
  for (size_t i = 0; i < control_points.size(); ++i) {
    constraints.push_back(control_points[i]);
  }

  // Return the minimum constraint value (non-positive value indicates a
  // constraint violation)
  return *std::min_element(constraints.begin(), constraints.end());
}

double
BSplineOptimization::constraint_pos2(const std::vector<double> &control_points,
                                     std::vector<double> &grad, void *data) {
  std::vector<double> constraints;
  for (size_t i = 0; i < control_points.size(); ++i) {
    constraints.push_back(2.0 - control_points[i]);
  }

  // Return the minimum constraint value (non-positive value indicates a
  // constraint violation)
  return *std::min_element(constraints.begin(), constraints.end());
}
} // namespace bspline_ekf
