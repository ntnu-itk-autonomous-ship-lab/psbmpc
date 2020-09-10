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
#include "psbmpc.h"
#include "cb_cost_functor.cuh"

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
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::Matrix<double, 6, 1> &ownship_state, 						// In: Current ship state
	const Eigen::Matrix<double, 4, -1> &static_obstacles,					// In: Static obstacle information
	Obstacle_Data &data														// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(6, n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = data.new_obstacles.size();
	int n_static_obst = static_obstacles.cols();

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		return;
	}
	
	initialize_prediction(data);

	for (int i = 0; i < n_obst; i++)
	{
		if (!pars.obstacle_colav_on)
		{
			// PSBMPC parameters needed to determine if obstacle breaches COLREGS 
			// (future: implement simple sbmpc class for obstacle which has the "determine COLREGS violation" function)
			data.new_obstacles[i]->predict_independent_trajectories(pars.T, pars.dt, trajectory.col(0), pars.phi_AH, pars.phi_CR, pars.phi_HO, pars.phi_OT, pars.d_close, pars.d_safe);
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
		Eigen::MatrixXd P_i_p = data.new_obstacles[i].get_trajectory_covariance();
		std::vector<Eigen::MatrixXd> xs_i_p = data.new_obstacles[i].get_trajectories();

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

	op.reset(new CB_Cost_Functor(*this, u_d, chi_d, waypoints, static_obstacles, data));

	// Allocate device vector for computing CB costs
	thrust::device_vector<double> cb_costs(pars.n_cbs);
	
	// Allocate iterator for passing the index of the control behavior to the kernels
	thrust::counting_iterator<unsigned int> index_iter(0);
	
	// Perform the calculations on the GPU
    thrust::transform(thrust::device, index_iter, index_iter + pars.n_cbs, cb_costs.begin(), *op);

	// Extract minimum cost
	thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
	min_index = min_cost_iter - cb_costs.begin();
	min_cost = cb_costs[min_index];

	// Set the trajectory to the optimal one and assign to the output trajectory
	ownship.predict_trajectory(trajectory, control_behaviours.col(min_index), maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
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

	u_opt = control_behaviours(0, min_index); 		u_m_last = u_opt;
	chi_opt = control_behaviours(1, min_index); 	chi_m_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << control_behaviours(2 * M, min_index) << ", " << control_behaviours(2 * M + 1, min_index) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl;
}