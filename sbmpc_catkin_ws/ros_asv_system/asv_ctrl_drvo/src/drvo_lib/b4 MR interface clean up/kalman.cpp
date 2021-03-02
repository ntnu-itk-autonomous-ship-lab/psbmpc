/**
 * \file   kalman.cpp
 * \brief  Defines the KalmanFilter class.
 */ 

#include <iostream>
#include <stdexcept>

#include "kalman.h"

KalmanFilter::KalmanFilter(
    int id,
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::VectorXd angle_tag)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n), id(id), angle_tag(angle_tag)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  Eigen::VectorXd y_diff;
  y_diff.resize(n);

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  
  y_diff = y - C*x_hat_new;
  
  // Correct difference for a state that is an angle
  // Only if C is a square matrix. Can be generalized for all C implementations!
  
  if (m==n){
      for (int i=0; i<n; i++){
          if ( angle_tag(i) == 1 ) 
	      y_diff(i) = angle_diff(x_hat_new(i), y(i)); // y - x_hat_new	  
      } 
  }
  
  
  //x_hat_new += K * (y - C*x_hat_new);
  x_hat_new += K * y_diff;
  
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}

inline double KalmanFilter::angle_diff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}


