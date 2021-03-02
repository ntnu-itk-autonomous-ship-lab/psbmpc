/**
 * \file   kalman.h
 * \brief  Declares the KalmanFilter class.
 */

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      int id,
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P,
      const Eigen::VectorXd angle_tag 
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();
  

  /** 
  * Create a destructor
  */
  ~KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y, double durationLost);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double durationLost, double dt, const Eigen::MatrixXd A);

  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  Eigen::MatrixXd errorCov() { return P; };
  double time() { return t; };
  int filter_id(){return id;};
  
  /**
  * Compute angle difference
  */
  inline double angle_diff(double a,double b);
  

private:

  // Filter id
  int id;
  
  // Specifies which states are angles
  Eigen::VectorXd angle_tag;
  
  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};
