/****************************************************************************************
*
*  File name : test_psbmpc.cu
*
*  Function  : Test file for the Probabilistic Scenario-based Model Predictive Control
*			   using Matlab for visualization. GPU version
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

#include "psbmpc.cuh"
#include "utilities.h"
#include <iostream>
#include <vector>
#include <chrono>
#include "Eigen/Dense"
#include "engine.h"


#define BUFSIZE 1000000

int main(){
	// Matlab engine setup
 	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1];  */

//*****************************************************************************************************************
// Simulation setup
//*****************************************************************************************************************
	double T_sim = 200; double dt = 0.5;
	int N = std::round(T_sim / dt);

//*****************************************************************************************************************
// Own-ship sim setup
//*****************************************************************************************************************
	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 9, 0, 0;
	double u_d = 9, chi_d, u_c, chi_c;
	
	Ownship asv_sim;

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	trajectory.resize(6, N);
	trajectory.col(0) = xs_os_0;

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
	xs_i_0[0].resize(6);
	xs_i_0[0] << 500, 300, -90 * DEG2RAD, 5, 0, 0;

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

	// Simulate obstacles using an ownship model
	Ownship obstacle_sim;

	std::vector<double> u_d_i(n_obst);
	std::vector<double> chi_d_i(n_obst);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_obst);

	std::vector<Eigen::VectorXd> maneuver_times_i(n_obst);

	std::vector<Eigen::Matrix<double, 6, -1>> trajectory_i(n_obst); 
	std::vector<Eigen::Matrix<double, 16, -1>> trajectory_covariances_i(n_obst);
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_obst);

	//=====================================================================
	// Matlab array setup for the ownship and obstacle, ++
	//=====================================================================
	/* mxArray *traj_os = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);

	double *ptraj_os = mxGetPr(traj_os); 
	double *p_wps_os = mxGetPr(wps_os); 

	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_os, 2, n_wps_os);
	map_wps_i = waypoints;

	std::vector<mxArray*> traj_i(n_obst); 
	std::vector<mxArray*> P_traj_i(n_obst); 
	std::vector<mxArray*> wps_i(n_obst);

	double* ptraj_i; 
	double* p_P_traj_i; 
	double* p_wps_i; */
	int n_wps_i;

	for (int i = 0; i < n_obst; i++)
	{
		ID[i] = i;

		u_d_i[i] = 5.0; chi_d_i[i] = 0.0;

		trajectory_i[i].resize(6, N);
		trajectory_i[i].col(0) = xs_i_0[i];

		trajectory_covariances_i[i].resize(16, 1);
		trajectory_covariances_i[i].col(0) = flatten(P_0);

		Pr_a[i].resize(3);
		Pr_a[i] << 1, 1, 1;
		Pr_a[i] = Pr_a[0] / Pr_a[0].sum();
		/* Pr_a[i].resize(1);
		Pr_a[i] << 1; */

		Pr_CC[i] = 1;

		n_wps_i = 2;
		waypoints_i[i].resize(2, n_wps_i); 
		waypoints_i[i] << 500, 500,
					300, -300;
		
		offset_sequence_i[i].resize(6);
		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

		maneuver_times_i[i].resize(3);
		maneuver_times_i[i] << 0, 100, 150;

		// Simulate obstacle trajectory independent on the ownship
		obstacle_sim.predict_trajectory(trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], ERK1, LOS, T_sim, dt);

		/* wps_i[i] = mxCreateDoubleMatrix(2, n_wps_i, mxREAL);
		traj_i[i] = mxCreateDoubleMatrix(6, N, mxREAL);
		P_traj_i[i] = mxCreateDoubleMatrix(16, 1, mxREAL); */
	}

//*****************************************************************************************************************
// Obstacle Manager setup
//*****************************************************************************************************************	
	std::unique_ptr<Obstacle_Manager> obstacle_manager = std::make_unique<Obstacle_Manager>();
//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	std::unique_ptr<PSBMPC> psbmpc = std::make_unique<PSBMPC>();
	double u_opt, chi_opt;

	Eigen::Matrix<double, 2, -1> predicted_trajectory; 

	Eigen::Matrix<double, 9, -1> obstacle_states;
	obstacle_states.resize(9, n_obst);

	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	obstacle_covariances.resize(16, n_obst);

	Eigen::MatrixXd obstacle_intention_probabilities;
	obstacle_intention_probabilities.resize(3, n_obst);

	Eigen::VectorXd obstacle_a_priori_CC_probabilities(n_obst);

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
	/* mxArray *T_sim_mx, *n_obst_mx, *n_static_obst_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);

	mxArray *pred_traj;
	double *p_pred_traj;

	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, n_static_obst, mxREAL);
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 

	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, n_static_obst);
	map_static_obst = static_obstacles;

	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "WPs", wps_os);

	engEvalString(ep, "init_psbmpc_plotting");
	mxArray *i_mx, *k_s;

	for (int i = 0; i < n_obst; i++)
	{
		p_wps_i = mxGetPr(wps_i[i]);

		Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i);
		map_wps_i = waypoints_i[i];

		engPutVariable(ep, "WPs_i", wps_i[i]);

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		engEvalString(ep, "init_obstacle_plot");
	} */
	//=========================================================
	
	Eigen::Vector4d xs_i_k;
	Eigen::VectorXd xs_aug(9);
	double mean_t = 0, t(0.0);
	for (int k = 0; k < N; k++)
	{
		t = k * dt;

		std::cout << Pr_a[0].transpose() << std::endl;

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

		std::cout << obstacle_intention_probabilities.transpose() << std::endl;

		 obstacle_manager->operator()(
			psbmpc->pars, 
			trajectory.col(k), 
			asv_sim.get_length(),
			obstacle_states, 
			obstacle_covariances, 
			obstacle_intention_probabilities, 
			obstacle_a_priori_CC_probabilities);

		std::cout << Pr_a[0].transpose() << std::endl;

		obstacle_manager->update_obstacle_status(trajectory.col(k));

		std::cout << Pr_a[0].transpose() << std::endl;

		obstacle_manager->display_obstacle_information();

		std::cout << Pr_a[0].transpose() << std::endl;

		/* asv_sim.update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, LOS);

		if (fmod(t, 5) == 0)
		{
			start = std::chrono::system_clock::now();		

			psbmpc->calculate_optimal_offsets(
				u_opt,
				chi_opt, 
				predicted_trajectory,
				u_d,
				chi_d,
				waypoints,
				trajectory.col(k),
				static_obstacles,
				obstacle_manager->get_data());

			end = std::chrono::system_clock::now();
			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

			mean_t = elapsed.count();

			std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;
		
		} 
		u_c = u_d * u_opt; chi_c = chi_d + chi_opt;
		asv_sim.update_ctrl_input(u_c, chi_c, trajectory.col(k));
		
		if (k < N - 1) { trajectory.col(k + 1) = asv_sim.predict(trajectory.col(k), dt, ERK1); }
		 */

		//===========================================
		// Send trajectory data to matlab
		//===========================================
		/* buffer[BUFSIZE] = '\0';
		engOutputBuffer(ep, buffer, BUFSIZE);

		k_s = mxCreateDoubleScalar(k + 1);
		engPutVariable(ep, "k", k_s);
		
		pred_traj = mxCreateDoubleMatrix(predicted_trajectory.rows(), predicted_trajectory.cols(), mxREAL);
		p_pred_traj = mxGetPr(pred_traj);

		Eigen::Map<Eigen::MatrixXd> map_pred_traj_os(p_pred_traj, 2, predicted_trajectory.cols());
		map_pred_traj_os = predicted_trajectory;

		Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, N);
		map_traj_os = trajectory;

		engPutVariable(ep, "X_pred", pred_traj);
		engPutVariable(ep, "X", traj_os);

		engEvalString(ep, "update_ownship_plot");

		for(int i = 0; i < n_obst; i++)
		{
			ptraj_i = mxGetPr(traj_i[i]);
			p_P_traj_i = mxGetPr(P_traj_i[i]);

			Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 6, N);
			Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, 1);
			
			map_traj_i = trajectory_i[i];
			map_P_traj_i = trajectory_covariances_i[i];
			
			engPutVariable(ep, "X_i", traj_i[i]);
			engPutVariable(ep, "P_i", P_traj_i[i]);

			i_mx = mxCreateDoubleScalar(i + 1);
			engPutVariable(ep, "i", i_mx);

			engEvalString(ep, "update_obstacle_plot");
		} */
		//======================================================
		
	}

	/* mxDestroyArray(traj_os);
	mxDestroyArray(wps_os);
	mxDestroyArray(pred_traj);
	for (int i = 0; i < n_obst; i++)
	{
		mxDestroyArray(traj_i[i]);
		mxDestroyArray(P_traj_i[i]);
		mxDestroyArray(wps_i[i]);
	}
	engClose(ep);   */

	return 0;
}