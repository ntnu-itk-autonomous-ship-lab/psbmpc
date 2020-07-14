/****************************************************************************************
*
*  File name : test_ownship.cpp
*
*  Function  : Test file for the Ownship class for PSB-MPC, using Matlab for 
*			   visualization 
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

#include "psbmpc.h"
#include "utilities.h"
#include <iostream>
#include <vector>
#include <string>
#include "Eigen/StdVector"
#include "Eigen/Core"
#include "engine.h"

#define BUFSIZE 1000000

int main(){
/* 	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1]; */

	//*****************************************************************************************************************
	// Simulation setup
	//*****************************************************************************************************************
	double T_sim = 0.5; double dt = 0.5;
	int N = std::round(T_sim / dt);

	//*****************************************************************************************************************
	// Own-ship sim setup
	//*****************************************************************************************************************

	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 9, 0, 0;
	double u_d, chi_d, u_c, chi_c;
	
	Ownship* asv_sim = new Ownship();

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	trajectory.resize(6, N);
	trajectory.col(0) = xs_os_0;

	waypoints.resize(2, 2); 
	//waypoints << 0, 200, 200, 0,    0, 300, 1000,
	//			 0, -50,  -200, -200,  0, 300, 0;
	waypoints << 0, 1000,
				 0, 0;
/* 
	mxArray *traj_os = mxCreateDoubleMatrix(6, N, mxREAL);

	double *ptraj_os = mxGetPr(traj_os); */
	
	//*****************************************************************************************************************
	// Obstacle sim setup
	//*****************************************************************************************************************
	int n_obst = 1;
	std::vector<int> ID(n_obst);

	std::vector<Eigen::VectorXd> xs_i_0(n_obst);
	xs_i_0[0].resize(6);
	xs_i_0[0] << 200, 0, 180 * DEG2RAD, 5, 0, 0;

	// Use constant obstacle uncertainty throughout the simulation, for simplicity
	Eigen::MatrixXd P_0(4, 4);
	P_0 << 25, 0, 0, 0,
	     0, 25, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	double A = 5, B = 5, C = 5, D = 5; 
	
	// Use constant equal intention probability and a priori CC probability  for simplicity
	std::vector<Eigen::VectorXd> Pr_a(n_obst);

	std::vector<double> Pr_CC(n_obst);
	

	bool filter_on = true, colav_on = false;

	// Simulate obstacles using an ownship model
	Ownship* obstacle_sim = new Ownship();

	std::vector<double> u_d_i(n_obst);
	std::vector<double> chi_d_i(n_obst);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_obst);

	std::vector<Eigen::VectorXd> maneuver_times_i(n_obst);

	std::vector<Eigen::Matrix<double, 6, -1>> trajectory_i(n_obst); 
	std::vector<Eigen::Matrix<double, 16, -1>> trajectory_covariances_i(n_obst);
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_obst);

	for (int i = 0; i < n_obst; i++)
	{
		u_d_i[i] = 6.0; chi_d_i[i] = 0.0;

		trajectory_i[i].resize(6, N);
		trajectory_i[i].col(0) = xs_i_0[i];

		trajectory_covariances_i[i].resize(16, N);
		trajectory_covariances_i[i].col(0) = flatten(P_0);

		Pr_a[i].resize(3);
		Pr_a[i] << 1, 1, 1;
		Pr_a[i] = Pr_a[0] / Pr_a[0].sum();

		Pr_CC[i] = 0.9;

		waypoints_i[i].resize(2, 2); 
		waypoints_i[i] << 1000, 0,
					0, 	0;

		offset_sequence_i[i].resize(6);
		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

		maneuver_times_i[i].resize(3);
		maneuver_times_i[i] << 0, 100, 150;

		// Simulate obstacle trajectory independent on the ownship
		obstacle_sim->predict_trajectory(trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], ERK1, LOS, T_sim, dt);
	}
	

/* 	mxArray *traj_i = mxCreateDoubleMatrix(4, N, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, N, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i); */

	//*****************************************************************************************************************
	// PSB-MPC setup
	//*****************************************************************************************************************	
	PSBMPC *psbmpc = new PSBMPC();
	double u_opt, chi_opt;

	Eigen::Matrix<double, 2, -1> predicted_trajectory; 
	mxArray *pred_traj;

	Eigen::Matrix<double,-1,-1> obstacle_status; 				
	Eigen::Matrix<double,-1, 1> colav_status; 

	Eigen::Matrix<double, 9, -1> obstacle_states;
	obstacle_states.resize(9, n_obst);

	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	obstacle_covariances.resize(16, n_obst);

	Eigen::MatrixXd obstacle_intention_probabilities;
	obstacle_intention_probabilities.resize(3, n_obst);

	Eigen::VectorXd obstacle_a_priori_CC_probabilities(n_obst);

	// Not implemented geographical cost yet, so these will not be considered
	Eigen::Matrix<double, 4, -1> static_obstacles;
	static_obstacles.resize(4, n_obst);
    static_obstacles << 50.0, 0.0, 50.0, 2050.0;

	//*****************************************************************************************************************
	// Simulation
	//*****************************************************************************************************************	
	Eigen::Vector4d xs_i_k;

	Eigen::VectorXd xs_aug(9);
	for (int k = 0; k < N; k++)
	{
		// Aquire obstacle information
		for (int i = 0; i < n_obst; i++)
		{
			xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
			xs_i_k.block<2, 1>(2, 0) = rotate_vector_2D(trajectory_i[i].block<2, 1>(3, k), trajectory_i[i](2, k));
			obstacle_states.col(i) << xs_i_k, A, B, C, D, ID[i];

			obstacle_covariances.col(i) = flatten(P_0);

			obstacle_intention_probabilities.col(i) = Pr_a[i];
			obstacle_a_priori_CC_probabilities(i) = Pr_CC[i];
		}

		asv_sim->update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, LOS);

		psbmpc->calculate_optimal_offsets(
			u_opt,
			chi_opt, 
			predicted_trajectory,
			obstacle_status,
			colav_status,
			u_d,
			chi_d,
			waypoints,
			trajectory.col(k),
			obstacle_states,
			obstacle_covariances,
			obstacle_intention_probabilities,
			obstacle_a_priori_CC_probabilities,
			static_obstacles);

		u_c = u_d * u_opt; chi_c = chi_d + chi_opt;

		asv_sim->update_ctrl_input(u_c, chi_d, trajectory.col(k));

		if (k < N - 1) { trajectory.col(k + 1) = asv_sim->predict(trajectory.col(k), dt, ERK1); }

	}
	//*****************************************************************************************************************
	// Send final trajectory data to matlab
	//*****************************************************************************************************************
/* 	Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, N);
	map_traj_os = trajectory;
	
	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, N);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, N);

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_os);
	
	mxDestroyArray(traj_os);

	mxDestroyArray(traj_i);
	mxDestroyArray(P_traj_i);

	engClose(ep);  */

	return 0;
}