/****************************************************************************************
*
*  File name : test_psbmpc.cpp
*
*  Function  : Test file for the Probabilistic Scenario-based Model Predictive Control
*			   using Matlab for visualization 
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

#include "cpu/psbmpc_cpu.hpp"
#include "gpu/psbmpc_gpu.cuh"
#include "cpu/utilities_cpu.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include "Eigen/Dense"
#include "engine.h"


#define BUFSIZE 1000000

int main()
{
//*****************************************************************************************************************
// Simulation setup
//*****************************************************************************************************************
	double T_sim = 150, dt = 0.5;
	int N = std::round(T_sim / dt);

//*****************************************************************************************************************
// Own-ship sim setup
//*****************************************************************************************************************
	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 9, 0, 0;
	double u_d(9.0), chi_d(0.0), u_c(0.0), chi_c(0.0);
	
	PSBMPC_LIB::CPU::Ownship asv_sim;

	Eigen::MatrixXd trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	#if OWNSHIP_TYPE == 0
		trajectory.resize(4, N);
		trajectory.col(0) = xs_os_0.block<4, 1>(0, 0);
	#else
		trajectory.resize(6, N);
		trajectory.col(0) = xs_os_0;
	#endif

	int n_wps_os = 2;
	waypoints.resize(2, n_wps_os); 
	/* waypoints << 0, 200, 200, 400, 600,  300, 500,
				 0, 0,   200, 200,  0,  0, -200; */
	waypoints << 0, 1000,
				 0, 0;
	
//*****************************************************************************************************************
// Obstacle sim setup
//*****************************************************************************************************************
	int n_obst = 1;
	std::vector<int> ID(n_obst);

	std::vector<Eigen::VectorXd> xs_i_0(n_obst);

	// Use constant obstacle uncertainty throughout the simulation, for simplicity
	Eigen::MatrixXd P_0(4, 4);
	P_0 << 25, 0, 0, 0,
	     0, 25, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	double A = 10, B = 10, C = 2, D = 2; 
	
	// Use constant equal intention probability and a priori CC probability  for simplicity
	std::vector<Eigen::VectorXd> Pr_a(n_obst);

	std::vector<double> Pr_CC(n_obst);

	// Simulate obstacles using an ownship model
	PSBMPC_LIB::CPU::Ownship obstacle_sim;

	std::vector<double> u_d_i(n_obst);
	std::vector<double> chi_d_i(n_obst);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_obst);

	std::vector<Eigen::VectorXd> maneuver_times_i(n_obst);

	std::vector<Eigen::MatrixXd> trajectory_i(n_obst); 
	std::vector<Eigen::Matrix<double, 16, -1>> trajectory_covariances_i(n_obst);
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_obst);

	//=====================================================================
	// Matlab engine setup and array setup for the ownship and obstacle, ++
	//=====================================================================
	// Matlab engine setup
 	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1]; 

	mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), N, mxREAL);
	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);

	double *p_traj_os = mxGetPr(traj_os_mx); 
	double *p_wps_os = mxGetPr(wps_os_mx); 

	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_os, 2, n_wps_os);
	map_wps_i = waypoints;

	std::vector<mxArray*> traj_i_mx(n_obst); 
	std::vector<mxArray*> P_traj_i_mx(n_obst); 
	std::vector<mxArray*> wps_i_mx(n_obst);

	double* ptraj_i; 
	double* p_P_traj_i; 
	double* p_wps_i;
	
	
	//=====================================================================
	int n_wps_i(0);

	for (int i = 0; i < n_obst; i++)
	{
		ID[i] = i;

		trajectory_covariances_i[i].resize(16, 1);
		trajectory_covariances_i[i].col(0) = PSBMPC_LIB::CPU::flatten(P_0);

		Pr_a[i].resize(3);
		Pr_a[i] << 0.05, 0.9, 0.05;
		Pr_a[i] = Pr_a[i] / Pr_a[i].sum();
		/* Pr_a[i].resize(1);
		Pr_a[i] << 1; */

		Pr_CC[i] = 1;

		n_wps_i = 2;
		waypoints_i[i].resize(2, n_wps_i); 
		xs_i_0[i].resize(6);
		if (i == 1)
		{
			//xs_i_0[i] << 5000, 0, 180 * DEG2RAD, 6, 0, 0;
			xs_i_0[i] << 300, 150, -90 * DEG2RAD, 5, 0, 0;
			waypoints_i[i] << 	xs_i_0[i](0), 500,
								xs_i_0[i](1), -300;
			u_d_i[i] = 5.0; chi_d_i[i] = -90 * DEG2RAD;
		} 
		else if (i == 2)
		{
			xs_i_0[i] << 500, -300, 90 * DEG2RAD, 5, 0, 0;
			waypoints_i[i] << 	xs_i_0[i](0), 500,
								xs_i_0[i](1), 300;
			u_d_i[i] = 5.0; chi_d_i[i] = 90 * DEG2RAD;
		}
		else
		{
			xs_i_0[i] << 700, 0, 180 * DEG2RAD, 8, 0, 0;
			waypoints_i[i] << 	xs_i_0[i](0), 0,
								xs_i_0[i](1), 0;
			u_d_i[i] = 8.0; chi_d_i[i] = 180 * DEG2RAD;
		}

		#if OWNSHIP_TYPE == 0
			trajectory_i[i].resize(4, N);
			trajectory_i[i].block<2, 1>(0, 0) = xs_i_0[i].block<2, 1>(0, 0);
			trajectory_i[i](2, 0) = xs_i_0[i](2);
			trajectory_i[i](3) = xs_i_0[i].block<2, 1>(3, 0).norm();
		#else
			trajectory_i[i].resize(6, N);
			trajectory_i[i].col(0) = xs_i_0[i];
		#endif
		
		offset_sequence_i[i].resize(6);
		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

		maneuver_times_i[i].resize(3);
		maneuver_times_i[i] << 0, 100, 150;

		// Simulate obstacle trajectory independent on the ownship
		obstacle_sim.predict_trajectory(trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T_sim, dt);
	}
	
//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	PSBMPC_LIB::Obstacle_Manager obstacle_manager;
	PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;
	PSBMPC_LIB::CPU::PSBMPC psbmpc; // change CPU for GPU depending on the version you want to test

	double u_opt(1.0), chi_opt(0.0);

	Eigen::Matrix<double, 2, -1> predicted_trajectory; 

	Eigen::Matrix<double, 9, -1> obstacle_states;
	obstacle_states.resize(9, n_obst);

	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	obstacle_covariances.resize(16, n_obst);

	int n_static_obst = 1;
	Eigen::Matrix<double, 4, -1> static_obstacles;

	// Format of each column: v_0, v_1, where v_0 and v_1 are the (x,y) coordinates of the start and end
	// of the static obstacle no-go zone
	static_obstacles.resize(4, n_static_obst);
    static_obstacles.col(0) << 500.0, 300.0, 1000.0, 50.0;

//*****************************************************************************************************************
// Simulation
//*****************************************************************************************************************	
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	//=========================================================
	// Matlab plot setup
	//=========================================================
	
	for (int i = 0; i < n_obst; i++)
	{
		wps_i_mx[i] = mxCreateDoubleMatrix(2, n_wps_i, mxREAL);
		traj_i_mx[i] = mxCreateDoubleMatrix(trajectory_i[i].rows(), N, mxREAL);
		P_traj_i_mx[i] = mxCreateDoubleMatrix(16, 1, mxREAL);
	}
	
	mxArray *T_sim_mx(nullptr), *n_obst_mx(nullptr), *n_static_obst_mx(nullptr);
	T_sim_mx = mxCreateDoubleScalar(T_sim);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);

	mxArray *pred_traj_mx;
	double *p_pred_traj;

	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, n_static_obst, mxREAL);
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 

	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, n_static_obst);
	map_static_obst = static_obstacles;

	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "WPs", wps_os_mx);

	engEvalString(ep, "init_psbmpc_plotting");
	mxArray *i_mx(nullptr), *k_s_mx(nullptr);

	for (int i = 0; i < n_obst; i++)
	{
		p_wps_i = mxGetPr(wps_i_mx[i]);

		Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i);
		map_wps_i = waypoints_i[i];

		engPutVariable(ep, "WPs_i", wps_i_mx[i]);

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		engEvalString(ep, "init_obstacle_plot");
	}
	//=========================================================
	
	Eigen::Vector4d xs_i_k;
	Eigen::VectorXd xs_aug(9);
	double mean_t(0.0), t(0.0);
	for (int k = 0; k < N; k++)
	{
		t = k * dt;

		// Aquire obstacle information
		for (int i = 0; i < n_obst; i++)
		{
			if (trajectory_i[i].rows() == 4)
			{
				xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
				xs_i_k(2) = trajectory_i[i](3, k) * cos(trajectory_i[i](2, k));
				xs_i_k(3) = trajectory_i[i](3, k) * sin(trajectory_i[i](2, k));
			}
			else
			{
				xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
				xs_i_k.block<2, 1>(2, 0) = PSBMPC_LIB::CPU::rotate_vector_2D(trajectory_i[i].block<2, 1>(3, k), trajectory_i[i](2, k));
			}
			obstacle_states.col(i) << xs_i_k, A, B, C, D, ID[i];

			obstacle_covariances.col(i) = PSBMPC_LIB::CPU::flatten(P_0);
		}

		obstacle_manager(trajectory.col(k), asv_sim.get_length(), obstacle_states, obstacle_covariances, psbmpc);

		obstacle_predictor(obstacle_manager.get_data(), trajectory.col(k), psbmpc);

		asv_sim.update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, PSBMPC_LIB::LOS);

		if (fmod(t, 5) == 0)
		{
			start = std::chrono::system_clock::now();		

			psbmpc.calculate_optimal_offsets(
				u_opt,
				chi_opt, 
				predicted_trajectory,
				u_d,
				chi_d,
				waypoints,
				trajectory.col(k),
				static_obstacles,
				obstacle_manager.get_data());

			end = std::chrono::system_clock::now();
			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

			mean_t = elapsed.count();

			std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;

			//std::cout << "u_d = " << u_d << " | chi_d = " << chi_d << std::endl;

			obstacle_manager.update_obstacle_status(trajectory.col(k));
			obstacle_manager.display_obstacle_information();
		}
		u_c = u_d * u_opt; chi_c = chi_d + chi_opt;
		
		if (k < N - 1) { trajectory.col(k + 1) = asv_sim.predict(trajectory.col(k), u_c, chi_c, dt, PSBMPC_LIB::ERK1); }
		//std::cout << trajectory.col(k).transpose() << std::endl;

		//===========================================
		// Send trajectory data to matlab
		//===========================================
		buffer[BUFSIZE] = '\0';
		engOutputBuffer(ep, buffer, BUFSIZE);

		k_s_mx = mxCreateDoubleScalar(k + 1);
		engPutVariable(ep, "k", k_s_mx);
		
		pred_traj_mx = mxCreateDoubleMatrix(predicted_trajectory.rows(), predicted_trajectory.cols(), mxREAL);
		p_pred_traj = mxGetPr(pred_traj_mx);

		Eigen::Map<Eigen::MatrixXd> map_pred_traj_os(p_pred_traj, 2, predicted_trajectory.cols());
		map_pred_traj_os = predicted_trajectory;

		Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, trajectory.rows(), N);
		map_traj_os = trajectory;

		engPutVariable(ep, "X_pred", pred_traj_mx);
		engPutVariable(ep, "X", traj_os_mx);

		engEvalString(ep, "update_ownship_plot");

		printf("%s", buffer);
		for(int i = 0; i < n_obst; i++)
		{
			ptraj_i = mxGetPr(traj_i_mx[i]);
			p_P_traj_i = mxGetPr(P_traj_i_mx[i]);

			Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, trajectory_i[i].rows(), N);
			Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, 1);
			
			map_traj_i = trajectory_i[i];
			map_P_traj_i = trajectory_covariances_i[i];
			
			engPutVariable(ep, "X_i", traj_i_mx[i]);
			engPutVariable(ep, "P_i", P_traj_i_mx[i]);

			i_mx = mxCreateDoubleScalar(i + 1);
			engPutVariable(ep, "i", i_mx);

			engEvalString(ep, "update_obstacle_plot");

			printf("%s", buffer);
		}
		
		//======================================================
		
	}

	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_os_mx);
	mxDestroyArray(pred_traj_mx);
	mxDestroyArray(i_mx);
	mxDestroyArray(k_s_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(n_obst_mx);
	mxDestroyArray(n_static_obst_mx);
	for (int i = 0; i < n_obst; i++)
	{
		mxDestroyArray(traj_i_mx[i]);
		mxDestroyArray(P_traj_i_mx[i]);
		mxDestroyArray(wps_i_mx[i]);
	}
	engClose(ep);   

	return 0;
}