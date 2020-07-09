/****************************************************************************************
*
*  File name : test_ownship.cpp
*
*  Function  : Test file for the Kalman Filter (KF) class for PSB-MPC
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


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utilities.h"
#include "kf.h"
#include <iostream>
#include <variant>
#include <vector>
#include "Eigen/StdVector"
#include "Eigen/Core"

#define BUFSIZE 1000000

int main()
{
	Eigen::Matrix<double, 6, 1> xs;
	xs << 0, 0, 0, 6, 0, 0;
	std::cout << "xs = [" << xs(0) << ", " << xs(1) << ", " << xs(2) << ", " << xs(3) << ", " << xs(4) << ", " << xs(5) << "]" << std::endl;

	double t = 0, t_0 = 0, T = 300; double dt = 0.5;

	int n_samples = std::round(T / dt);

	Eigen::VectorXd xs_i(4), xs_i_p, xs_i_upd, y_m(4);
	xs_i << 0, 0, 2, 2;
	y_m << 10, 0, 2, 1;

	Eigen::MatrixXd P_i(4, 4);
	P_i << 10, 0, 0, 0,
			0, 10, 0, 0,
			0, 0, 2, 0,
			0, 0, 0, 2;

	int ID_i = 0;

	KF *kf = new KF(xs_i, P_i, ID_i, dt, t_0);

	double duration_lost = 0;

	kf->predict(dt);
	kf->update(y_m, duration_lost, dt);
	kf->reset(xs_i, P_i, t_0);
	xs_i_upd = kf->get_state();
	std::cout << "xs = [" << xs_i_upd(0) << ", " << xs_i_upd(1) << ", " << xs_i_upd(2) << ", " 
			  << xs_i_upd(3) << "]" << std::endl;
	
	std::cout << "P_i = " << std::endl;
	print_matrix(kf->get_covariance());

	return 0;
}