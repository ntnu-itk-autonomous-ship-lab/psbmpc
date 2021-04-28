/****************************************************************************************
*
*  File name : test_obstacle_predictor.cpp
*
*  Function  : Test file for the Mean-reverting Ornstein-Uhlenbeck process 
*			   class for PSB-MPC.
*			   
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#include "cpu/psbmpc_cpu.hpp"
#include <iostream>
#include <vector>
#include <memory>
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

	Eigen::Vector4d xs_i_0;
	xs_i_0 << 0, 0, 6, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
			0, 100, 0, 0,
			0, 0, 0.025, 0,
			0, 0, 0, 0.025;

	Eigen::VectorXd xs_i_aug(9);
	xs_i_aug << xs_i_0, 5, 5, 5, 5, 0;

	Eigen::VectorXd Pr_s(3); Pr_s << 1, 1, 1;
	Pr_s / Pr_s.sum();

	Eigen::VectorXd ownship_state(4); ownship_state << 500, 0, 180 * DEG2RAD, 9.0;

	PSBMPC_LIB::Obstacle_Data<PSBMPC_LIB::Tracked_Obstacle> data;
	data.obstacles.push_back(PSBMPC_LIB::Tracked_Obstacle(xs_i_aug, P_0, Pr_s, false, T, dt));

	PSBMPC_LIB::CPU::PSBMPC psbmpc;
	PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;

	//==============================================
	// Setup and predict using the developed class
	//==============================================
	obstacle_predictor(data, ownship_state, psbmpc);
	//==============================================
	
	std::vector<Eigen::MatrixXd> xs_p = data.obstacles[0].get_trajectories();
	Eigen::MatrixXd P_p = data.obstacles[0].get_trajectory_covariance();
	int n_ps = xs_p.size();

	mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *v_traj_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_traj = mxGetPr(traj_mx);
	double *p_v_traj = mxGetPr(v_traj_mx);
	double *p_P_traj = mxGetPr(P_traj_mx);
	mxArray *ps_mx(nullptr), *k_mx(nullptr);

	engEvalString(ep, "test_obstacle_predictor_plot_init");
	for (int ps = 0; ps < n_ps; ps++)
	{
		ps_mx = mxCreateDoubleScalar(ps + 1);
		engPutVariable(ep, "ps", ps_mx);
		
		Eigen::Map<Eigen::MatrixXd> map_traj(p_traj, 4, n_samples);
		Eigen::Map<Eigen::MatrixXd> map_P_traj(p_P_traj, 16, n_samples);
		
		map_traj = xs_p[ps];
		map_P_traj = P_p;
		
		for (int k = 0; k < n_samples; k++)
		{
			k_mx = mxCreateDoubleScalar(k + 1);

			buffer[BUFSIZE] = '\0';
			engOutputBuffer(ep, buffer, BUFSIZE);

			engPutVariable(ep, "k", k_mx);
			engPutVariable(ep, "X_ps", traj_mx);
			engPutVariable(ep, "v", v_traj_mx);
			engPutVariable(ep, "P_flat", P_traj_mx);
			engEvalString(ep, "test_obstacle_predictor_plot_update");

			printf("%s", buffer);
		}
	}

	mxDestroyArray(traj_mx);
	mxDestroyArray(v_traj_mx);
	mxDestroyArray(P_traj_mx);
	engClose(ep);
	
	return 0;
}