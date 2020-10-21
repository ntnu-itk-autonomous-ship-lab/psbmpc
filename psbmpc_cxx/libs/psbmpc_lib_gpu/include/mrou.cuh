/****************************************************************************************
*
*  File name : mrou.cuh
*
*  Function  : Header file for the Mean-Reverting Ornstein-Uhlenbeck process predictor,
*			   modified with .cuh for this GPU-implementation.
*
*  
*	           ---------------------
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

#ifndef _MROU_CUH_
#define _MROU_CUH_

#include "Eigen/Dense"

class MROU{
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

	MROU();

	MROU(const double sigma_x, const double sigma_xy, const double sigma_y, const double gamma_x, const double gamma_y);

	Eigen::Vector4d predict_state(const Eigen::Vector4d &xs_old, const Eigen::Vector2d &v, const double t);

	Eigen::Matrix4d predict_covariance(const Eigen::Matrix4d &P_old, const double t);
};

#endif