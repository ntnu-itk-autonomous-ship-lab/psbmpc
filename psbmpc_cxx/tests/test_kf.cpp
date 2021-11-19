/****************************************************************************************
*
*  File name : test_kf.cpp
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

#include "cpu/utilities_cpu.hpp"
#include "kf.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <engine.h>

#define BUFSIZE 1000000

int main()
{
	double t_0(0.0), dt(0.5);

	Eigen::VectorXd xs_i(4), xs_i_p, xs_i_upd, y_m(4);
	xs_i << 0, 0, 2, 2;
	y_m << 10, 0, 2, 1;

	Eigen::MatrixXd P_i(4, 4);
	P_i << 10, 0, 0, 0,
			0, 10, 0, 0,
			0, 0, 2, 0,
			0, 0, 0, 2;

	int ID_i(0);

	std::unique_ptr<PSBMPC_LIB::KF> kf(new PSBMPC_LIB::KF(xs_i, P_i, ID_i, dt, t_0));

	double duration_lost(0.0);

	kf->predict(dt);
	kf->update(y_m, duration_lost, dt);
	xs_i_upd = kf->get_state();
	std::cout << "xs = [" << xs_i_upd(0) << ", " << xs_i_upd(1) << ", " << xs_i_upd(2) << ", " 
			  << xs_i_upd(3) << "]" << std::endl;
	
	std::cout << "P_i = " << std::endl;
	std::cout << kf->get_covariance() << std::endl;

	kf->reset(xs_i, P_i, t_0);
	xs_i_upd = kf->get_state();
	std::cout << "xs = [" << xs_i_upd(0) << ", " << xs_i_upd(1) << ", " << xs_i_upd(2) << ", " 
			  << xs_i_upd(3) << "]" << std::endl;
	
	std::cout << "P_i = " << std::endl;
	std::cout << kf->get_covariance() << std::endl;

	return 0;
}