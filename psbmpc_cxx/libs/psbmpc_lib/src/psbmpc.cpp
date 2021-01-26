/****************************************************************************************
*
*  File name : psbmpc.cpp
*
*  Function  : Class functions for Probabilistic Scenario-based Model Predictive Control
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

#include "utilities.h"
#include "psbmpc.h"

#include "obstacle_manager.h"
#include "ownship.h"
#include "cpe.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include "engine.h"

#define BUFFSIZE 100000

/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC()
{
	offset_sequence_counter.resize(2 * pars.n_M);
	offset_sequence.resize(2 * pars.n_M);
	maneuver_times.resize(pars.n_M);

	cpe = CPE(pars.cpe_method, pars.dt);

	mpc_cost = MPC_Cost<PSBMPC_Parameters>(pars);

	chi_m_last = 0; u_m_last = 1;
}

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : 
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
	Obstacle_Data<Tracked_Obstacle> &data									// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(6, n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = data.obstacles.size();
	int n_static_obst = static_obstacles.cols();

	Eigen::VectorXd opt_offset_sequence(2 * pars.n_M);

	// Predict nominal trajectory first, assign as optimal if no need for
	// COLAV, or use in the prediction initialization
	for (int M = 0; M < pars.n_M; M++)
	{
		offset_sequence(2 * M) = 1.0; offset_sequence(2 * M + 1) = 0.0;
	}
	maneuver_times.setZero();
	ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1.0; 		u_m_last = u_opt;
		chi_opt = 0.0; 	chi_m_last = chi_opt;

		assign_optimal_trajectory(predicted_trajectory);

		return;
	}

	initialize_prediction(data, static_obstacles);

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
 	mxArray *traj_os = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

	double *ptraj_os = mxGetPr(traj_os); 
	double *p_wps_os = mxGetPr(wps_os); 

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
	map_wps = waypoints;

	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, n_static_obst, mxREAL);
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 
	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, n_static_obst);
	map_static_obst = static_obstacles;

	mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_obst_mx, *i_mx, *ps_mx, *n_static_obst_mx;
	dt_sim = mxCreateDoubleScalar(pars.dt);
	T_sim = mxCreateDoubleScalar(pars.T);
	n_ps_mx = mxCreateDoubleScalar(n_ps[0]);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);

	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "dt_sim", dt_sim);
	engPutVariable(ep, "T_sim", T_sim);
	engPutVariable(ep, "WPs", wps_os);
	engEvalString(ep, "inside_psbmpc_init_plot");

	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);
	double *p_P_c_i;

	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);

	std::vector<mxArray*> P_c_i_mx(n_obst);

 	for(int i = 0; i < n_obst; i++)
	{
		P_c_i_mx[i] = mxCreateDoubleMatrix(n_ps[i], n_samples, mxREAL);

		Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
		std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

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
			engEvalString(ep, "inside_psbmpc_obstacle_plot");
		}
	}
	
	//===============================================================================================================
	double cost;
	Eigen::VectorXd cost_i(n_obst);
	Eigen::MatrixXd P_c_i;
	data.HL_0.resize(n_obst); data.HL_0.setZero();
	min_cost = 1e12;
	reset_control_behaviour();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0;
		//std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
		std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;

		ownship.predict_trajectory(
			trajectory, 
			offset_sequence, 
			maneuver_times, 
			u_d, chi_d, 
			waypoints, 
			pars.prediction_method, 
			pars.guidance_method, 
			pars.T, 
			pars.dt);

		if (pars.obstacle_colav_on)
		{ 
			predict_trajectories_jointly(data, static_obstacles, true); 
		}

		for (int i = 0; i < n_obst; i++)
		{
			int p_stepp = 2;
			P_c_i.resize(n_ps[i], n_samples);
			calculate_instantaneous_collision_probabilities(P_c_i, data, i, p_stepp * pars.dt, p_stepp); 

			cost_i(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, offset_sequence, maneuver_times, P_c_i, data, i, ownship.get_length());

			//===============================================================================================================
			// MATLAB PLOTTING FOR DEBUGGING
			//===============================================================================================================
			p_P_c_i = mxGetPr(P_c_i_mx[i]);
			Eigen::Map<Eigen::MatrixXd> map_P_c(p_P_c_i, n_ps[i], n_samples);
			map_P_c = P_c_i;

			i_mx = mxCreateDoubleScalar(i + 1);
			engPutVariable(ep, "i", i_mx);

			engPutVariable(ep, "P_c_i", P_c_i_mx[i]);
			for(int ps = 0; ps < n_ps[i]; ps++)
			{
				ps_mx = mxCreateDoubleScalar(ps + 1);
				engPutVariable(ep, "ps", ps_mx);
				engEvalString(ep, "inside_psbmpc_upd_coll_probs_plot");
			}
			//===============================================================================================================
		}

		cost += cost_i.maxCoeff();

		//cost += calculate_grounding_cost(trajectory, static_obstacles, ownship.get_length());

		cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_m_last, chi_m_last);

		cost += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);


		if (cost < min_cost) 
		{
			min_cost = cost;
			opt_offset_sequence = offset_sequence;

			assign_optimal_trajectory(predicted_trajectory);

			// Assign current optimal hazard level for each obstacle
			for (int i = 0; i < n_obst; i++)
			{
				if (cost_i.sum() > 0)
					data.HL_0(i) = cost_i(i) / cost_i.sum();
			}	
		}
		increment_control_behaviour();

		//===============================================================================================================
		// MATLAB PLOTTING FOR DEBUGGING
		//===============================================================================================================
		Eigen::Map<Eigen::MatrixXd> map_traj(ptraj_os, 6, n_samples);
		map_traj = trajectory;

		k_s = mxCreateDoubleScalar(n_samples);
		engPutVariable(ep, "k", k_s);

		engPutVariable(ep, "X", traj_os);
		engEvalString(ep, "inside_psbmpc_upd_ownship_plot");
		//===============================================================================================================
	}

	u_opt = opt_offset_sequence(0); 	u_m_last = u_opt;
	chi_opt = opt_offset_sequence(1); 	chi_m_last = chi_opt;

	if(u_opt == 0)
	{
		chi_opt = 0; 	chi_m_last = chi_opt;
	} 	

	/* std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl; */

	//std::cout << "Cost at optimum : " << min_cost << std::endl;

	engClose(ep); 
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branch of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::reset_control_behaviour()
{
	offset_sequence_counter.setZero();
	for (int M = 0; M < pars.n_M; M++)
	{
		offset_sequence(2 * M) = pars.u_offsets[M](0);
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](0);
	}
}

/****************************************************************************************
*  Name     : increment_control_behavior
*  Function : Increments the control behavior counter and changes the offset sequence 
*			  accordingly. Backpropagation is used for the incrementation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::increment_control_behaviour()
{
	for (int M = pars.n_M - 1; M > -1; M--)
	{
		// Only increment counter for "leaf node offsets" on each iteration, which are the
		// course offsets in the last maneuver
		if (M == pars.n_M - 1)
		{
			offset_sequence_counter(2 * M + 1) += 1;
		}

		// If one reaches the end of maneuver M's course offsets, reset corresponding
		// counter and increment surge offset counter above
		if (offset_sequence_counter(2 * M + 1) == pars.chi_offsets[M].size())
		{
			offset_sequence_counter(2 * M + 1) = 0;
			offset_sequence_counter(2 * M) += 1;
		}
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](offset_sequence_counter(2 * M + 1));

		// If one reaches the end of maneuver M's surge offsets, reset corresponding
		// counter and increment course offset counter above (if any)
		if (offset_sequence_counter(2 * M) == pars.u_offsets[M].size())
		{
			offset_sequence_counter(2 * M) = 0;
			if (M > 0)
			{
				offset_sequence_counter(2 * M - 1) += 1;
			}
		}
		offset_sequence(2 * M) = pars.u_offsets[M](offset_sequence_counter(2 * M));
	}
}

/****************************************************************************************
*  Name     : initialize_prediction
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation, and predicts
*			  independent obstacle trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_prediction(
	Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const Eigen::Matrix<double, 4, -1> &static_obstacles			// In: Static obstacle information
	)
{
	int n_obst = data.obstacles.size();
	n_ps.resize(n_obst);
	pobstacles.resize(n_obst);
	int n_a(0);
	if (n_obst > 0)
	{
		n_a = data.obstacles[0].get_intention_probabilities().size();
	} 
	
	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	std::vector<Intention> ps_ordering_i;
	Eigen::VectorXd ps_course_changes_i;
	Eigen::VectorXd ps_maneuver_times_i;

	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa, d_AB, v_0;
	Eigen::Vector4d xs_i_0;
	Eigen::Matrix<double, 2, -1> waypoints_i;
	for (int i = 0; i < n_obst; i++)
	{
		xs_i_0 = data.obstacles[i].kf->get_state();
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), xs_i_0);

		if (n_a == 1)
		{
			n_ps[i] = 1;
			ps_ordering_i.resize(1);
			ps_ordering_i[0] = KCC;			
			ps_course_changes_i.resize(1);
			ps_course_changes_i[0] = 0;
			ps_maneuver_times_i.resize(1);
			ps_maneuver_times_i(0) = 0;
		}
		else
		{
			set_up_independent_obstacle_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i, t_cpa(i), data, i);

			pobstacles[i] = Prediction_Obstacle(data.obstacles[i]);
			if (pars.obstacle_colav_on)
			{
				pobstacles[i].set_colav_on(false);

				// Set obstacle waypoints to a straight line out from its current time position 
				// if no future obstacle trajectory is available
				waypoints_i.resize(2, 2);
				xs_i_0 = pobstacles[i].get_initial_state();
				waypoints_i.col(0) = xs_i_0.block<2, 1>(0, 0);
				waypoints_i.col(1) = waypoints_i.col(0) + xs_i_0.block<2, 1>(2, 0) * pars.T;
				pobstacles[i].set_waypoints(waypoints_i);

				n_ps[i] += 1;
			}

			data.obstacles[i].initialize_independent_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i);	

			data.obstacles[i].predict_independent_trajectories<PSBMPC>(pars.T, pars.dt, trajectory.col(0), *this);
		}
	}

	if (pars.obstacle_colav_on)
	{
		predict_trajectories_jointly(data, static_obstacles, false);
	}
	
	//
	prune_obstacle_scenarios(data);

	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(pars.n_M);
	// First avoidance maneuver is always at t0
	maneuver_times.setZero();

	double t_cpa_min, d_safe_i;
	std::vector<bool> maneuvered_by(n_obst);
	int index_closest;
	for (int M = 1; M < pars.n_M; M++)
	{
		// This is the solution so far if n_obst = 0. And also:
		// If a predicted collision occurs with the closest obstacle, avoidance maneuver 
		// M is taken right after the obstacle possibly maneuvers (which will be at t_0 + M * t_ts
		// if the independent obstacle prediction scheme is used), given that t_cpa > t_ts. 
		// If t_cpa < t_ts, the subsequent maneuver is taken at t_0 + M * t_ts + 1 anyways (simplification)
		maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);
		
		// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
		t_cpa_min = 1e10; index_closest = -1;
		for (int i = 0; i < n_obst; i++)
		{
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());
			// For the current avoidance maneuver, determine which obstacle that should be
			// considered, i.e. the closest obstacle that is not already passed (which means
			// that the previous avoidance maneuver happened before CPA with this obstacle)
			if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min)
			{	
				t_cpa_min = t_cpa(i);
				index_closest = i;
			}	
		}

		if (index_closest != -1)
		{
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[index_closest].get_width());
			// If no predicted collision,  avoidance maneuver M with the closest
			// obstacle (that is not passed) is taken at t_cpa_min
			if (d_cpa(index_closest) > d_safe_i)
			{
				//std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
				maneuvered_by[index_closest] = true;
				maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
			}
		}
	}
	
	std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : set_up_independent_obstacle_prediction_variables
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_independent_obstacle_prediction(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const double t_cpa_i, 													// In: Time to Closest Point of Approach for obstacle i wrt own-ship
	const Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const int i 															// In: Index of obstacle in consideration
	)
{
	int turn_count(0), turn_start(0), n_turns(0), course_change_count(0);

	// The out-commented stuff is too ad-hoc to be used.
	/*
	double d_AB, d_AB_prev;
	Eigen::Vector4d xs_i_0 = data.obstacles[i].kf->get_state();
	Eigen::Vector2d v_0, v_i_0;

	d_AB = (trajectory.col(0).block<2, 1>(0, 0) - xs_i_0.block<2, 1>(0, 0)).norm();
	v_0 = rotate_vector_2D(trajectory.col(0).block<2, 1>(3, 0), trajectory(2, 0));
	v_i_0 = xs_i_0.block<2, 1>(2, 0);

	
	 // Alternative obstacle maneuvers (other than the straight line prediction) 
	// are only allowed inside the COLREGS consideration zone, i.e. inside d_close
	while (d_AB > pars.d_close)
	{
		d_AB_prev = d_AB;
		turn_start += 1;
		// calculate new distance between own-ship and obstacle i, given that both keep
		// their course for k * t_ts seconds, k = 1, 2, 3, ...
		d_AB = ((trajectory.col(0).block<2, 1>(0, 0) + v_0 * turn_start * pars.t_ts) - 
				(xs_i_0.block<2, 1>(0, 0) + v_i_0 * turn_start * pars.t_ts)).norm();
		if (d_AB > d_AB_prev)
		{
			turn_start = -1;
			break;
		}
	} */
	if (turn_start >= 0) // and alternative maneuvers are only up until cpa with the own-ship
	{
		n_turns = std::ceil((t_cpa_i - turn_start * pars.t_ts) / pars.t_ts);
	}
	else 							// or no alternative maneuvers at all if the obstacle never enters
	{ 								// the own-ship COLREGS consideration zone
		n_turns = 0;
	}
	n_ps[i] = 1 + 2 * pars.obstacle_course_changes.size() * n_turns;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC;
	ps_maneuver_times_i.resize(n_ps[i]);
	ps_maneuver_times_i[0] = 0;
	ps_course_changes_i.resize(n_ps[i]);
	ps_course_changes_i[0] = 0;
	
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (n_ps[i] - 1) / 2 + 1)
		{
			ps_ordering_i[ps] = SM;

			ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}
		// Port maneuvers
		else
		{
			ps_ordering_i[ps] = PM;

			ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = - pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}	
	}
	std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
	std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : prune_obstacle_scenarios
*  Function : Goes through all generated obstacle prediction scenarios, and selects the
*			  N_r scenarios with highest collision risk for keeping, discarding all others.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::prune_obstacle_scenarios(
	Obstacle_Data<Tracked_Obstacle> &data							// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();

	int p_step = 1;   
	double dt_r = (double)p_step * pars.dt;
	int n_samples = std::round(pars.T / pars.dt);

	Eigen::MatrixXd P_c_i;
	Eigen::VectorXi sorted_ps_indices_i;
	Eigen::VectorXi kept_ps_indices_i;
	Eigen::VectorXd R_c_i, P_c_i_ps, C_i;
	for (int i = 0; i < n_obst; i++)
	{			
		P_c_i.resize(n_ps[i], n_samples); P_c_i_ps.resize(n_ps[i]); 
		C_i.resize(n_ps[i]); R_c_i.resize(n_ps[i]);
		sorted_ps_indices_i.resize(n_ps[i]);
		
		calculate_instantaneous_collision_probabilities(P_c_i, data, i, dt_r, p_step);

		calculate_ps_collision_probabilities(P_c_i_ps, P_c_i, i);

		calculate_ps_collision_consequences(C_i, data, i, dt_r, p_step);

		calculate_ps_collision_risks(R_c_i, sorted_ps_indices_i, C_i, P_c_i_ps, data, i);

		std::cout << sorted_ps_indices_i.transpose() << std::endl;

		// Keep only the n_r prediction scenarios with the highest collision risk
		n_ps[i] = pars.n_r;
		kept_ps_indices_i = sorted_ps_indices_i.block(0, 0, pars.n_r, 1);
		std::cout << kept_ps_indices_i.transpose() << std::endl;
		

		// Sort indices of ps that are to be kept
		std::sort(kept_ps_indices_i.data(), kept_ps_indices_i.data() + pars.n_r);
		std::cout << kept_ps_indices_i.transpose() << std::endl;

		data.obstacles[i].prune_ps(kept_ps_indices_i);
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_probabilities
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated collision probabilities.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_probabilities(
	Eigen::VectorXd &P_c_i_ps,											// In/out: Vector of collision consequences, size n_ps_i x 1
	const Eigen::MatrixXd &P_c_i,										// In: Predicted obstacle collision probabilities for all prediction scenarios, size n_ps[i] x n_samples
	const int i															// In: Index of obstacle
	)
{
	P_c_i_ps.setZero();

	double product(0.0);
	int n_samples = P_c_i.cols();
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k++)
		{
			if (k == 0)	{ product = 1 - P_c_i(ps, k); }
			else		{ product *= (1 - P_c_i(ps, k)); }
		}
		P_c_i_ps(ps) = 1 - product;
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_consequences
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated consequence of collision.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_consequences(
	Eigen::VectorXd &C_i, 												// In/out: Vector of collision consequences, size n_ps_i x 1
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i,														// In: Index of obstacle
	const double dt,													// In: Time step between predicted trajectory samples
	const int p_step													// In: Step between trajectory samples, matches the input prediction time step
	)
{
	C_i.setZero();

	double collision_consequence(0.0), t(0.0), t_cpa(0.0), d_cpa(0.0);

	std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();
	Eigen::MatrixXd xs_i_colav_p;
	if (pars.obstacle_colav_on)
	{
		xs_i_colav_p = pobstacles[i].get_trajectory();
	}

	Eigen::Vector2d p_cpa, v_0_p, v_i_p;

	int n_samples = std::round(pars.T / dt);

	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k += p_step)
		{
			t = k * dt;

			v_0_p = trajectory.block<2, 1>(3, k);
			v_0_p = rotate_vector_2D(v_0_p, trajectory(2, k));

			if (ps == n_ps[i] - 1 && pars.obstacle_colav_on) // Intelligent prediction is the last prediction scenario
			{
				v_i_p(0) = xs_i_colav_p(3, k) * cos(xs_i_colav_p(2, k));
				v_i_p(1) = xs_i_colav_p(3, k) * sin(xs_i_colav_p(2, k));
			}
			else
			{
				v_i_p = xs_i_p[ps].block<2, 1>(2, k);
			}			

			calculate_cpa(p_cpa, t_cpa, d_cpa, trajectory.col(k), xs_i_p[ps].col(k));

			collision_consequence = pow((v_0_p - v_i_p).norm(), 2) * exp(- abs(t - t_cpa));

			if (C_i(ps) < collision_consequence)
			{
				C_i(ps) = collision_consequence;
			}
		}
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_risks
*  Function : Goes through all generated obstacle i prediction scenarios, and calculates
*			  the predicted collision risk.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_risks(
	Eigen::VectorXd &R_c_i,												// In/out: Vector of collision risks, size n_ps_i x 1
	Eigen::VectorXi &indices_i,											// In/out: Vector of indices for the ps, in decending order wrt collision risk, size n_ps_i x 1
	const Eigen::VectorXd &C_i,											// In: Vector of collision consequences, size n_ps_i x 1
	const Eigen::VectorXd &P_c_i_ps,									// In: Vector of collision probabilities, size n_ps_i x 1
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i															// In: Index of obstacle	
	)
{
	R_c_i.setZero();

	double Pr_ps_i_conditional(0.0);
	Eigen::VectorXd Pr_c_i_conditional(n_ps[i]);
	Eigen::VectorXd Pr_a_i = data.obstacles[i].get_intention_probabilities();
	Eigen::VectorXi ps_intention_count_i = data.obstacles[i].get_ps_intention_count();
	int n_a = Pr_a_i.size();

	// The conditional probability of collision with obstacle i is predicted
	// as the product of the conditional probability Pr(ps | a, I) of prediction scenario ps 
	// (here taken as a uniform distribution) weighted by intention probabilities Pr(a | I), and the
	// predicted scenario collision probability Pr(collision | i, s)
	for (int a = 0; a < n_a; a++)
	{
		// Conditional probability of a prediction scenario is taken from a discrete uniform distribution
		// over the number of prediction scenarios which corresponds to intention a, and is
		// weighted by the corresponding intention probability
		Pr_ps_i_conditional += Pr_a_i(a) / (double)ps_intention_count_i(a);
	}
	
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		indices_i[ps] = ps;

		Pr_c_i_conditional(ps) = P_c_i_ps(ps) * Pr_ps_i_conditional;

		R_c_i(ps) = C_i(ps) * Pr_c_i_conditional(ps);
	}

	// Sort vector of ps indices to determine collision risk in sorted order
	std::sort(indices_i.data(), indices_i.data() + n_ps[i], [&](const int index_lhs, const int index_rhs) { return R_c_i(index_lhs) > R_c_i(index_rhs); });
	
	std::ios::fmtflags old_settings = std::cout.flags();
	int old_precision = std::cout.precision(); 
	int cw = 20;
	//std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout << std::fixed << std::setprecision(7);

	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
	std::cout << "Obstacle i = " << i << " prediction scenario information:" << std::endl;
	// ps : Prediction scenario, R: Collision risk, C: Collision consequence,
	// P_c^{i, ps}: Collision probability for that scenario
	// Pr{C^i | ps, I^i}: Conditional collision probability for that scenario, on
	// available information in I^i
	
	std::cout << "ps" << std::setw(cw - 4) << "R" << std::setw(cw - 2) << "C" << std::setw(cw + 6) << "P_c^{i, ps}" << std::setw(cw + 4) << "Pr{C^i | s, I^i}" << std::endl;
	for (int j = 0; j < n_ps[i]; j++)
	{
		std::cout 	<< indices_i[j] << std::setw(cw) << R_c_i(indices_i(j)) << std::setw(cw) << C_i(indices_i(j)) << std::setw(cw) 
					<< P_c_i_ps(indices_i(j)) << std::setw(cw) << Pr_c_i_conditional(indices_i(j)) << std::endl;
	}
	std::cout.flags(old_settings);
	std::cout << std::setprecision(old_precision);
	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
}

/****************************************************************************************
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the obstacles with an active COLAV system,  
*		      considering the fixed current control behaviour, and adds or overwrites
*			  this information to the obstacle data structure
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::predict_trajectories_jointly(
	Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const Eigen::Matrix<double, 4, -1>& static_obstacles,		// In: Static obstacle information
	const bool overwrite										// In: Flag to choose whether or not to add the first intelligent prediction to the data, or overwrite the previous
	)
{
	int n_samples = trajectory.cols();
	int n_obst = pobstacles.size();
	Joint_Prediction_Manager jpm(n_obst); 

	Eigen::VectorXd u_opt_i(n_obst), chi_opt_i(n_obst), u_d_i(n_obst), chi_d_i(n_obst);
	Eigen::Vector4d xs_i_p, xs_i_p_transformed;
	Eigen::VectorXd xs_os_aug_k(7);
	xs_os_aug_k(4) = ownship.get_length();
	xs_os_aug_k(5) = ownship.get_width();
	xs_os_aug_k(6) = n_obst;

	std::vector<Obstacle_Ship> obstacle_ships(n_obst);
	std::vector<Eigen::Matrix<double, 4, -1>> predicted_trajectory_i(n_obst);

	Eigen::Vector4d xs_i_0;
 	for(int i = 0; i < n_obst; i++)
	{
		xs_i_0 = pobstacles[i].get_initial_state();
		u_d_i(i) = xs_i_0.block<2, 1>(2, 0).norm();
		chi_d_i(i) = atan2(xs_i_0(3), xs_i_0(2));

		pobstacles[i].set_intention(KCC);
		/* std::cout << "u_d_i = " << u_d_i(i) << std::endl;
		std::cout << "chi_d_i = " << chi_d_i(i) << std::endl; */
	}

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFFSIZE+1]; 

 	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, static_obstacles.cols(), mxREAL);
	mxArray *dt_sim_mx(nullptr), *T_sim_mx(nullptr), *k_s_mx(nullptr), *d_safe_mx(nullptr), *n_obst_mx(nullptr), *i_mx(nullptr), *n_static_obst_mx(nullptr);

	std::vector<mxArray*> wps_i_mx(n_obst);
	std::vector<mxArray*> traj_i_mx(n_obst);
	std::vector<mxArray*> pred_traj_i_mx(n_obst);

	double *p_traj_os = mxGetPr(traj_os_mx); 
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 
	double *p_wps_i = nullptr; 
	double *p_traj_i = nullptr;
	double *p_pred_traj_i = nullptr;

	int n_wps_i = 2;
	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, 6, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i);
	Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_pred_traj_i(p_pred_traj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, static_obstacles.cols());

	map_traj_os = trajectory;
	map_static_obst = static_obstacles;

	dt_sim_mx = mxCreateDoubleScalar(pars.dt);
	T_sim_mx = mxCreateDoubleScalar(pars.T);
	d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(static_obstacles.cols());

	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "d_safe", d_safe_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engEvalString(ep, "joint_pred_init_plotting");

	Eigen::Vector4d xs_i_0;
 	for(int i = 0; i < n_obst; i++)
	{
		wps_i_mx[i] = mxCreateDoubleMatrix(2, 2, mxREAL);
		traj_i_mx[i] = mxCreateDoubleMatrix(4, n_samples, mxREAL);

		p_wps_i = mxGetPr(wps_i_mx[i]);
		new (&map_wps_i) Eigen::Map<Eigen::MatrixXd>(p_wps_i, 2, n_wps_i);
		map_wps_i = pobstacles[i].get_waypoints();

		engPutVariable(ep, "WPs_i", wps_i_mx[i]);
		engEvalString(ep, "joint_pred_init_obstacle_plot");
	}	 */
	
	//===============================================================================================================
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	double t(0.0), mean_t(0.0), u_c_i(0.0), chi_c_i(0.0), chi_i(0.0);
	Eigen::Vector2d v_os_k;
	for (int k = 0; k < n_samples; k++)
	{	
		t = k * pars.dt;

		v_os_k = trajectory.block<2, 1>(3, k);
		v_os_k = rotate_vector_2D(v_os_k, trajectory(2, k));
		xs_os_aug_k.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, k);
		xs_os_aug_k.block<2, 1>(2, 0) = v_os_k;

		//std::cout << "xs_os_aug_k = " << xs_os_aug_k.transpose() << std::endl;

		// Update Obstacle Data for each prediction obstacle, taking all other obstacles
		// including the ownship into account
		jpm(pars, pobstacles, xs_os_aug_k, k);
			
		for (int i = 0; i < n_obst; i++)
		{
			xs_i_p = pobstacles[i].get_state(k);

			//std::cout << "xs_i_p = " << xs_i_p.transpose() << std::endl;

			// Convert from X_i = [x, y, Vx, Vy] to X_i = [x, y, chi, U]
			xs_i_p_transformed.block<2, 1>(0, 0) = xs_i_p.block<2, 1>(0, 0);
			xs_i_p_transformed(2) = atan2(xs_i_p(3), xs_i_p(2));
			xs_i_p_transformed(3) = xs_i_p.block<2, 1>(2, 0).norm();

			// Determine the intention that obstacle i`s predicted trajectory
			// corresponds to
			chi_i = xs_i_p_transformed(2);
			if (t < 30)
			{
				if (chi_i > 15 * DEG2RAD)								{ pobstacles[i].set_intention(SM); }
				else if (chi_i < -15 * DEG2RAD)							{ pobstacles[i].set_intention(PM); }
			}

			obstacle_ships[i].update_guidance_references(
				u_d_i(i), 
				chi_d_i(i), 
				pobstacles[i].get_waypoints(),
				xs_i_p,
				pars.dt,
				pars.guidance_method);

			if (fmod(t, 5) == 0)
			{
				start = std::chrono::system_clock::now();	

				pobstacles[i].sbmpc->calculate_optimal_offsets(
					u_opt_i(i), 
					chi_opt_i(i), 
					predicted_trajectory_i[i],
					u_d_i(i), 
					chi_d_i(i),
					pobstacles[i].get_waypoints(),
					xs_i_p_transformed,
					static_obstacles,
					jpm.get_data(i),
					k);

				end = std::chrono::system_clock::now();
				elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

				mean_t = elapsed.count();

				//std::cout << "Obstacle_SBMPC time usage : " << mean_t << " milliseconds" << std::endl;

				jpm.update_obstacle_status(i, xs_i_p_transformed, k);
				//jpm.display_obstacle_information(i);
			}
			u_c_i = u_d_i(i) * u_opt_i(i); chi_c_i = chi_d_i(i) + chi_opt_i(i);

			if (k < n_samples - 1)
			{
				xs_i_p_transformed = obstacle_ships[i].predict(xs_i_p_transformed, u_c_i, chi_c_i, pars.dt, pars.prediction_method);
				
				// Convert from X_i = [x, y, chi, U] to X_i = [x, y, Vx, Vy]
				xs_i_p.block<2, 1>(0, 0) = xs_i_p_transformed.block<2, 1>(0, 0);
				xs_i_p(2) = xs_i_p_transformed(3) * cos(xs_i_p_transformed(2));
				xs_i_p(3) = xs_i_p_transformed(3) * sin(xs_i_p_transformed(2));

				pobstacles[i].set_state(xs_i_p, k + 1);
			}
			
		//============================================================================================
		// Send data to matlab for live plotting
		//============================================================================================
			/*
			k_s_mx = mxCreateDoubleScalar(k + 1);
			engPutVariable(ep, "k", k_s_mx);

			buffer[BUFFSIZE] = '\0';
			engOutputBuffer(ep, buffer, BUFFSIZE);
			
			pred_traj_i_mx[i] = mxCreateDoubleMatrix(4, predicted_trajectory_i[i].cols(), mxREAL);

			p_traj_i = mxGetPr(traj_i_mx[i]);
			p_pred_traj_i = mxGetPr(pred_traj_i_mx[i]);

			new (&map_traj_i) Eigen::Map<Eigen::MatrixXd>(p_traj_i, 4, n_samples);
			new (&map_pred_traj_i) Eigen::Map<Eigen::MatrixXd>(p_pred_traj_i, 4, predicted_trajectory_i[i].cols());
			
			map_traj_i = pobstacles[i].get_trajectory();
			map_pred_traj_i = predicted_trajectory_i[i];

			i_mx = mxCreateDoubleScalar(i + 1);

			engPutVariable(ep, "i", i_mx);
			engPutVariable(ep, "X_i", traj_i_mx[i]);			
			engPutVariable(ep, "X_i_pred", pred_traj_i_mx[i]);

			engEvalString(ep, "update_joint_pred_obstacle_plot");

			printf("%s", buffer);	 */
			//============================================================================================				
			
		}
		

		/* engEvalString(ep, "update_joint_pred_ownship_plot"); */
		//============================================================================================
	}

	// Transfer intelligent trajectory information to the obstacle data
	for (int i = 0; i < n_obst; i++)
	{
		data.obstacles[i].add_intelligent_prediction(pobstacles[i], overwrite);
	}
/* 
	engEvalString(ep, "store_joint_pred_ownship_data");

	for (int i = 0; i < n_obst; i++)
	{
		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		pred_traj_i_mx[i] = mxCreateDoubleMatrix(4, predicted_trajectory_i[i].cols(), mxREAL);

		p_traj_i = mxGetPr(traj_i_mx[i]);
		p_pred_traj_i = mxGetPr(pred_traj_i_mx[i]);

		new (&map_traj_i) Eigen::Map<Eigen::MatrixXd>(p_traj_i, 4, n_samples);
		new (&map_pred_traj_i) Eigen::Map<Eigen::MatrixXd>(p_pred_traj_i, 4, predicted_trajectory_i[i].cols());
		
		map_traj_i = pobstacles[i].get_trajectory();
		map_pred_traj_i = predicted_trajectory_i[i];

		engPutVariable(ep, "X_i", traj_i_mx[i]);			
		engPutVariable(ep, "X_i_pred", pred_traj_i_mx[i]);
		engEvalString(ep, "store_joint_pred_obstacle_data");
	}

	//============================================================================================
	// Clean up matlab arrays
	//============================================================================================
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(static_obst_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(dt_sim_mx);
	mxDestroyArray(i_mx);
	mxDestroyArray(d_safe_mx);
	mxDestroyArray(k_s_mx);

	for (int i = 0; i < n_obst; i++)
	{
		mxDestroyArray(wps_i_mx[i]);
		mxDestroyArray(traj_i_mx[i]);
		mxDestroyArray(pred_traj_i_mx[i]);
	}
	engClose(ep); */
}

/****************************************************************************************
*  Name     : find_time_of_passing
*  Function : Finds the time when an obstacle is passed by the own-ship, assuming both 
*			  vessels keeps their current course
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double PSBMPC::find_time_of_passing(
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i 														// In: Index of relevant obstacle
	)
{
	double t_obst_passed(1e12), t, psi_A, d_AB;
	Eigen::VectorXd xs_A = trajectory.col(0);
	Eigen::VectorXd xs_B = data.obstacles[i].kf->get_state();
	Eigen::Vector2d p_A, p_B, v_A, v_B, L_AB;
	p_A(0) = xs_A(0); p_A(1) = xs_A(1); psi_A = xs_A(2);
	v_A(0) = xs_A(3); v_A(1) = xs_A(4); 
	v_A = rotate_vector_2D(v_A, psi_A);
	p_B(0) = xs_B(0); p_B(1) = xs_B(1);
	v_B(0) = xs_B(2); v_B(1) = xs_B(3); 

	bool A_is_overtaken, B_is_overtaken, is_passed;

	int n_samples = pars.T / pars.dt;
	for (int k = 0; k < n_samples; k++)
	{
		t = k * pars.dt;
		p_A = p_A + v_A * t;
		p_B = p_B + v_B * t;

		L_AB = p_B - p_A;
		d_AB = L_AB.norm();
		L_AB = L_AB.normalized();

		A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  		&&
						v_A.norm() > 0.25;

		B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							  		&&
						v_B.norm() > 0.25;

		is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
					!A_is_overtaken) 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
					!B_is_overtaken)) 											&&
					d_AB > pars.d_safe;
		
		if (is_passed) 
		{
			t_obst_passed = t; 
			break;
		}
	}
	return t_obst_passed;
}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the freshly updated obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_colav_active(
	const Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].kf->get_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].kf->get_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;

		// If all obstacles are passed, even though inside colav range,
		// then no need for colav
		if (data.IP_0[i]) 	{ colav_active = false; }
		else 				{ colav_active = true; }
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
}

/****************************************************************************************
*  Name     : calculate_collision_probabilities
*  Function : Estimates collision probabilities for the own-ship and an obstacle i in
*			  consideration. Can use a larger sample time than used in predicting
*			  the vessel trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_instantaneous_collision_probabilities(
	Eigen::MatrixXd &P_c_i,								// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const Obstacle_Data<Tracked_Obstacle> &data,		// In: Dynamic obstacle information
	const int i, 										// In: Index of obstacle
	const double dt, 									// In: Sample time for estimation
	const int p_step                                    // In: Step between trajectory samples, matches the input prediction time step
	)
{
	Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
	std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

	// Increase safety zone by half the max obstacle dimension and ownship length
	double d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());

	int n_samples = P_i_p.cols();
	// Non-optimal temporary row-vector storage solution
	Eigen::Matrix<double, 1, -1> P_c_i_row(n_samples);
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		cpe.estimate_over_trajectories(P_c_i_row, trajectory, xs_i_p[ps], P_i_p, d_safe_i, dt, p_step);
		
		P_c_i.block(ps, 0, 1, P_c_i_row.cols()) = P_c_i_row;
	}		
}

/****************************************************************************************
*  Name     : assign_optimal_trajectory
*  Function : Set the optimal trajectory to the current predicted trajectory
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::assign_optimal_trajectory(
	Eigen::Matrix<double, 2, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = std::round(pars.T / pars.dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (false) //(pars.prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(2, n_samples / pars.p_step);
		for (int k = 0; k < n_samples; k += pars.p_step)
		{
			optimal_trajectory.col(count) = trajectory.block<2, 1>(0, k);
			if (count < std::round(n_samples / pars.p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(2, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 2, n_samples);
	}
}