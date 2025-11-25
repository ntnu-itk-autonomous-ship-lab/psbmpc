#pragma once

#include <Eigen/Dense>

namespace PSBMPC_LIB {
class KF {
private:
  double t_0, t;

  bool initialized;

  // Predicted and updated state
  Eigen::Vector4d xs_p, xs_upd;

  // Initial, predicted and updated state error covariance
  Eigen::Matrix4d P_0, P_p, P_upd;

  // System matrices (A and C), process noise and measurement noise covariances
  // (Q and R), and identity matrix
  Eigen::Matrix4d A, Q, R, I;
  Eigen::MatrixXd C;
  double q; // Process noise strength

  void set_measurement_matrix(const bool use_velocity_measurements);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KF();

  KF(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0, const double t_0,
     const bool use_velocity_measurements);

  KF(const Eigen::Matrix4d &P_0, const Eigen::Matrix4d &R, const double q,
     const double t_0, const bool use_velocity_measurements);

  KF(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0,
     const Eigen::Matrix4d &R, const double q, const double t_0,
     const bool use_velocity_measurements);

  double get_time() const { return t; };

  Eigen::Vector4d get_predicted_state() const { return xs_p; }
  Eigen::Vector4d get_state() const { return xs_upd; };

  Eigen::Matrix4d get_predicted_covariance() const { return P_p; }
  Eigen::Matrix4d get_covariance() const { return P_upd; };

  void reset(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0,
             const double t_0);

  void predict(const double dt);

  void update(const Eigen::Vector4d &y_m, const double duration_lost,
              const double dt);

  void update(const Eigen::Vector2d &y_m, const double dt,
              const bool dead_reckon);
  void update(const Eigen::Vector4d &y_m, const double dt,
              const bool dead_reckon);
};
} // namespace PSBMPC_LIB