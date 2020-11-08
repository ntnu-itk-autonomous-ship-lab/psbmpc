/****************************************************************************************
*
*  File name : kf.cuh
*
*  Function  : Header file for a hardcoded Kalman Filter (KF) to add extra robustness
*              against track loss. See "Autonomous COLREGS compliant decision
*              making using maritime radar tracking and model predictive control". 
*              This is an alternative version of the one created by Giorgio D. Kwame 
*              Minde Kufoalor through the Autosea project, modified with .cuh for this 
*              GPU-implementation.
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

#ifndef _KF_CUH_
#define _KF_CUH_

#include "Eigen/Dense"

class KF{
private:

  int ID;

  double t_0, t;

  bool initialized;

  Eigen::Vector4d xs_p, xs_upd;

  Eigen::Matrix4d A, C, Q, R, I;

  Eigen::Matrix4d P_0, P_p, P_upd;

public:

  KF();

  KF(const Eigen::Vector4d& xs_0, const Eigen::Matrix4d& P_0, const int ID, const double dt, const double t_0);

  // Use the constructor below for simulations where the KF is used as a tracking system outside the COLAV algorithm
  // where typically only position measurements of vessels are used.
  KF(
    const Eigen::Vector4d &xs_0, 
    const Eigen::Matrix4d &P_0, 
    const int ID, 
    const double dt, 
    const double t_0, 
    const Eigen::Matrix4d &Q,
    const Eigen::Matrix4d &R);

  int get_ID() const { return ID; };

  double get_time() const { return t; };

  Eigen::Vector4d get_state() const { return xs_upd; };

  Eigen::Matrix4d get_covariance() const { return P_upd; };

  void reset(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0, const double t_0);

  void predict(const double dt);

  void update(const Eigen::Vector4d &y_m, const double duration_lost, const double dt);

};

#endif