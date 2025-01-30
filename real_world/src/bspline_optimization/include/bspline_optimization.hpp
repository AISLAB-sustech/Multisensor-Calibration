#include <Eigen/Core>
#include <Eigen/Dense>
#include <nlopt.hpp>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::vector;

namespace bspline_ekf {

struct ExtPara {
  double M_x;
  double M_y;
  double M_theta;
  double L_x;
  double L_y;
  double L_theta;
  double SRC_x;
  double SRC_y;
};

struct OptimizeData {
  vector<double> pre_points;
  vector<double> control_points;
  ExtPara ext_para;
};

struct SegmentedPathPoint {
  Vector2d point;
  double yaw_angle;
  int seg;
};

class BSplineOptimization {
public:
  BSplineOptimization();
  ~BSplineOptimization();
  // TODO: combine pre_points and result_points into one vector
  static void optimize(double delta, ExtPara ext_para, int curve_seg,
                       vector<double> pre_points, vector<double> &result_points,
                       vector<SegmentedPathPoint> &path);

private:
  static void bspline(const vector<double> &control_points, double delta,
                      vector<SegmentedPathPoint> &path);
  static void jacobian(const ExtPara &ext_para, const Vector2d &x,
                       const Vector2d &x_iplus1, double theta_i,
                       double theta_iplus1, MatrixXd &T_i);
  static Matrix2d rot_mat(double theta, bool de = false);
  static MatrixXd v_merge(const vector<MatrixXd> &matrices);
  static MatrixXd h_merge(const vector<MatrixXd> &matrices);
  static double
  max_negative_min_eigenvalue(const std::vector<double> &control_points,
                              std::vector<double> &grad, void *ext_para_ptr);
  static double constraint_angle(const std::vector<double> &control_points,
                                 std::vector<double> &grad,
                                 void *pre_point_ptr);
  static double constraint_pos1(const std::vector<double> &control_points,
                                std::vector<double> &grad, void *pre_point_ptr);
  static double constraint_pos2(const std::vector<double> &control_points,
                                std::vector<double> &grad, void *pre_point_ptr);
};

} // namespace bspline_ekf