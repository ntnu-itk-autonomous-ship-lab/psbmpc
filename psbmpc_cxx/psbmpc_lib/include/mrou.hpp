#pragma once

#include "Eigen/Dense"

namespace PSBMPC_LIB {
class MROU {
private:
  double sigma_x;
  double sigma_xy;
  double sigma_y;

  double gamma_x;
  double gamma_y;

  Eigen::Matrix4d Sigma_1;

  double f(const double t) const;

  double g(const double t) const;

  double h(const double t) const;

  double k(const double t) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MROU();

  MROU(const double sigma_x, const double sigma_xy, const double sigma_y,
       const double gamma_x, const double gamma_y);

  Eigen::Vector4d predict_state(const Eigen::Vector4d &xs_old,
                                const Eigen::Vector2d &v, const double t);

  Eigen::Matrix4d predict_covariance(const Eigen::Matrix4d &P_old,
                                     const double t);
};
} // namespace PSBMPC_LIB