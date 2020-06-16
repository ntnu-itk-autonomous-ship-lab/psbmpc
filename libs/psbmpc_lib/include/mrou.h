/****************************************************************************************
*
*  File name : mrou.h
*
*  Function  : Header file for the Mean-Reverting Ornstein-Uhlenbeck process predictor
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

#ifndef _MROU_H_
#define _MROU_H_

#include "Eigen/Dense"

class MROU{
private:

	double sigma_x_;
	double sigma_xy_;
	double sigma_y_;

	double gamma_x_;
	double gamma_y_;


	Eigen::Matrix<double, 4, 4> Sigma_1;

	double f(const double t) const;

	double g(const double t) const;

	double h(const double t) const; 

	double k(const double t) const;

public:

	MROU();

	MROU(const double sigma_x, const double sigma_xy, const double sigma_y, const double gamma_x, const double gamma_y);

	void predict_state(Eigen::VectorXd &xs, const Eigen::Vector2d &v, const double t);

	void predict_covariance(Eigen::MatrixXd &P, const double t);
};


#endif
