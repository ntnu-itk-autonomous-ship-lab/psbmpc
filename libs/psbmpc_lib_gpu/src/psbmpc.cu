/****************************************************************************************
*
*  File name : psbmpc.cu
*
*  Function  : Class functions for Probabilistic Scenario-based Model Predictive Control
*			   where the GPU is used (hence .cu)
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

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include "utilities.cuh"
#include "cb_cost_functor.cuh"
#include "psbmpc.h"
#include "iostream"
#include "engine.h"

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : Calculate optimal surge and course offsets for PSB-MPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 2, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	Eigen::Matrix<double,-1,-1> &obstacle_status,							// In/out: Various information on obstacles
	Eigen::Matrix<double,-1, 1> &colav_status,								// In/out: status on the COLAV system
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::Matrix<double, 6, 1> &ownship_state, 						// In: Current ship state
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 					// In: Dynamic obstacle states 
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 				// In: Dynamic obstacle covariances
	const Eigen::MatrixXd &obstacle_intention_probabilities,  				// In: Intention probabilitiy information for the obstacles
	const Eigen::VectorXd &obstacle_a_priori_CC_probabilities, 				// In: Information on a priori COLREGS compliance probabilities for the obstacles
	const Eigen::Matrix<double, 4, -1> &static_obstacles					// In: Static obstacle information
	)
{	
	int n_samples = std::round(T / dt);

	trajectory.resize(6, n_samples);
	trajectory.col(0) = ownship_state;

	ownship->determine_active_waypoint_segment(waypoints, ownship_state);

	update_obstacles(obstacle_states, obstacle_covariances, obstacle_intention_probabilities, obstacle_a_priori_CC_probabilities);
	int n_obst = new_obstacles.size();
	int n_static_obst = static_obstacles.cols();

	update_situation_type_and_transitional_variables();

	bool colav_active = determine_colav_active(n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		return;
	}
	
	initialize_prediction();

	for (int i = 0; i < n_obst; i++)
	{
		if (!obstacle_colav_on[i])
		{
			// PSBMPC parameters needed to determine if obstacle breaches COLREGS 
			// (future: implement simple sbmpc class for obstacle which has the "determine COLREGS violation" function)
			new_obstacles[i]->predict_independent_trajectories(T, dt, trajectory.col(0), phi_AH, phi_CR, phi_HO, phi_OT, d_close, d_safe);
		}
	}

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[1000000 + 1]; 
 	mxArray *traj_os = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

	double *ptraj_os = mxGetPr(traj_os); 
	double *p_wps_os = mxGetPr(wps_os); 

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
	map_wps = waypoints;

	mxArray *T_sim, *k_s, *n_ps_mx, *n_obst_mx, *i_mx, *ps_mx;
	T_sim = mxCreateDoubleScalar(T);
	n_ps_mx = mxCreateDoubleScalar(n_ps[0]);
	n_obst_mx = mxCreateDoubleScalar(n_obst);

	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "T_sim", T_sim);
	engPutVariable(ep, "WPs", wps_os);
	engEvalString(ep, "inside_psbmpc_init_plot");

	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);

	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);

 	for(int i = 0; i < n_obst; i++)
	{
		Eigen::MatrixXd P_i_p = new_obstacles[i]->get_trajectory_covariance();
		std::vector<Eigen::MatrixXd> xs_i_p = new_obstacles[i]->get_trajectories();

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		map_P_traj_i = P_i_p;
		engPutVariable(ep, "P_i_flat", P_traj_i);
		for (int ps = 0; ps < n_ps[i]; ps++)
		{
			ps_mx = mxCreateDoubleScalar(ps + 1);
			engPutVariable(ep, "ps", ps_mx);

			map_traj_i = xs_i_p[ps];
			
			engPutVariable(ep, "X_i", traj_i);
			//engEvalString(ep, "inside_psbmpc_obstacle_plot");
		}
	} */
	
	//===============================================================================================================

	//===============================================================================================================
	// Cost evaluation
	//===============================================================================================================
	Eigen::VectorXd HL_0(n_obst); HL_0.setZero();

	op.reset(new CB_Cost_Functor(*this, u_d, chi_d, waypoints, static_obstacles));

	// Allocate device vector for computing CB costs
	thrust::device_vector<double> cb_costs(n_cbs);
	
	// Allocate iterator for passing the index of the control behavior to the kernels
	thrust::counting_iterator<unsigned int> index_iter(0);
	
	// Perform the calculations on the GPU
    thrust::transform(thrust::device, index_iter, index_iter + n_cbs, cb_costs.begin(), *op);

	// Extract minimum cost
	thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
	min_index = min_cost_iter - cb_costs.begin();
	min_cost = cb_costs[min_index];

	// Set the trajectory to the optimal one and assign to the output trajectory
	Eigen::VectorXd offset_sequence(2 * n_M);
	offset_sequence = control_behaviours.col(min_index);
	ownship->predict_trajectory(trajectory, control_behaviours.col(min_index), maneuver_times, u_d, chi_d, waypoints, prediction_method, guidance_method, T, dt);
	assign_optimal_trajectory(predicted_trajectory);
	//===============================================================================================================

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Eigen::Map<Eigen::MatrixXd> map_traj(ptraj_os, 6, n_samples);
	map_traj = trajectory;

	k_s = mxCreateDoubleScalar(n_samples);
	engPutVariable(ep, "k", k_s);

	engPutVariable(ep, "X", traj_os);
	engEvalString(ep, "inside_psbmpc_upd_ownship_plot"); 

	engClose(ep); */
	//===============================================================================================================

	update_obstacle_status(obstacle_status, HL_0);

	u_opt = control_behaviours(0, min_index); 		u_m_last = u_opt;
	chi_opt = control_behaviours(1, min_index); 	chi_m_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < n_M; M++)
	{
		std::cout << control_behaviours(2 * M, min_index) << ", " << control_behaviours(2 * M + 1, min_index) * RAD2DEG;
		if (M < n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	double CF_0 = u_opt * (1 - fabs(chi_opt * RAD2DEG) / 15.0);
	colav_status.resize(2,1);
	colav_status << CF_0, min_cost;
}