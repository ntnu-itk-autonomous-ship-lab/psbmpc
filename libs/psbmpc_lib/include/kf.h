/****************************************************************************************
*
*  File name : kf.h
*
*  Function  : Header file for a hardcoded Kalman Filter (KF) to add extra robustness
*              against track loss. See "Autonomous COLREGS compliant decision
*              making using maritime radar tracking and model predictive control". 
*              This is an alternative version of the one created by Giorgio D. Kwame 
*              Minde Kufoalor through the Autosea project
*  
*            ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#ifndef _KF_H_
#define _KF_H_

#include "Eigen/Dense"

class KF{
private:

  int ID;

  double t_0, t;

  bool initialized;

  Eigen::Vector4d xs_p, xs_upd;

  Eigen::Matrix<double, 4, 4> A, C, Q, R, I;

  Eigen::Matrix<double, 4, 4> P_0, P_p, P_upd;

public:

  KF();

  KF(const Eigen::Vector4d& xs_0, const Eigen::Matrix4d& P_0, const int ID, const double dt, const double t_0);

  int get_ID() const { return ID; };

  double get_time() const { return t; };

  Eigen::VectorXd get_state() const { return xs_upd; };

  Eigen::MatrixXd get_covariance() const { return P_upd; };

  void reset(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0, const double t_0);

  void predict(const double dt);

  void update(const Eigen::Vector4d &y_m, const double duration_lost, const double dt);

};

#endif