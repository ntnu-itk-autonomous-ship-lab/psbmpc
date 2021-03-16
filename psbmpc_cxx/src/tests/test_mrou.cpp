/****************************************************************************************
*
*  File name : test_mrou.cpp
*
*  Function  : Test file for the Mean-reverting Ornstein-Uhlenbeck process 
*			   class for PSB-MPC.
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

#include "cpu/utilities_cpu.h"
#include "mrou.h"
#include <iostream>
#include <vector>
#include <memory>
#include "Eigen/Core"
#include "engine.h"

#define BUFSIZE 1000000

int main()
{
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1];
	
	double T = 300; double dt = 0.5;

	int n_samples = std::round(T / dt);

	double sigma_x(0.8), sigma_xy(0),sigma_y(0.8), gamma_x(0.1), gamma_y(0.1);

	Eigen::Vector4d xs_0;
	xs_0 << 0, 0, 2, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
			0, 100, 0, 0,
			0, 0, 0.025, 0,
			0, 0, 0, 0.025;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> v_p;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step
	Eigen::MatrixXd P_p; 

	std::unique_ptr<PSBMPC_LIB::MROU> mrou(new PSBMPC_LIB::MROU(sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y));

	// n_ps = 1
	xs_p.resize(1); xs_p[0].resize(4, n_samples);
	xs_p[0].col(0) = xs_0;
	P_p.resize(16, n_samples);
	P_p.col(0) = PSBMPC_LIB::CPU::flatten(P_0);

	v_p.resize(1); v_p[0].resize(2, n_samples);
	
	int n_ps = xs_p.size();

	Eigen::Vector2d v_0, v;
	v_0 << 4, 0;
	v_p[0].col(0) = v_0;

	Eigen::VectorXd turn_times(1);
	turn_times << 100;

	int tt_count = 0;
	double chi = 0; 
	v = v_0;
	for(int k = 0; k < n_samples; k++)
	{
		if (k == turn_times(tt_count))
		{
			chi = atan2(v(1), v(0));
			v(0) = v.norm() * cos(chi + 30 * M_PI / 180.0);
			v(1) = v.norm() * sin(chi + 30 * M_PI / 180.0);
			if (tt_count < turn_times.size() - 1) tt_count += 1;
		}
		if (k < n_samples - 1)	v_p[0].col(k + 1) = v;
	}
	
	mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *v_traj_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_traj = mxGetPr(traj_mx);
	double *p_v_traj = mxGetPr(v_traj_mx);
	double *p_P_traj = mxGetPr(P_traj_mx);

	double t = 0;
	Eigen::Vector4d xs = xs_0;
	Eigen::Matrix4d P = P_0;
	for (int ps = 0; ps < n_ps; ps++)
	{
		for (int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;
			xs = mrou->predict_state(xs, v_p[ps].col(k), dt);
			P = mrou->predict_covariance(P_0, t);

			if (k < n_samples - 1)
			{
				xs_p[ps].col(k + 1) = xs;
				P_p.col(k + 1) = PSBMPC_LIB::CPU::flatten(P);
			}
		}
	}

	Eigen::Map<Eigen::MatrixXd> map_traj(p_traj, 4, n_samples);
	map_traj = xs_p[0];

	Eigen::Map<Eigen::MatrixXd> map_v_traj(p_v_traj, 2, n_samples);
	map_v_traj = v_p[0];

	Eigen::Map<Eigen::MatrixXd> map_P_traj(p_P_traj, 16, n_samples);
	map_P_traj = P_p;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_mx);
	engPutVariable(ep, "v", v_traj_mx);
	engPutVariable(ep, "P_flat", P_traj_mx);
	engEvalString(ep, "test_mrou_plot");

	printf("%s", buffer);
	mxDestroyArray(traj_mx);
	mxDestroyArray(v_traj_mx);
	mxDestroyArray(P_traj_mx);
	engClose(ep);
	
	return 0;
}