/****************************************************************************************
*
*  File name : psbmpc_cpu.cpp
*
*  Function  : Class functions for Probabilistic Scenario-based Model Predictive Control
*			   on the CPU
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
#include "cpu/psbmpc_cpu.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include "engine.h"

#define BUFFSIZE 100000

namespace PSBMPC_LIB
{
namespace CPU
{

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

	u_opt_last = 1.0;
	chi_opt_last = 0.0;

	min_cost = 1e12;

	cpe = CPE(pars.cpe_method, pars.dt);

	mpc_cost = MPC_Cost<PSBMPC_Parameters>(pars);

	use_joint_prediction = false;
}

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : W/static obstacles parametrized as polygons
*  Author   : Trym Tengesdal & Tom Daniel Grande
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 2, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::VectorXd &ownship_state, 									// In: Current ship state
	const double V_w,														// In: Estimated wind speed
	const Eigen::Vector2d &wind_direction,									// In: Unit vector in NE describing the estimated wind direction
	const std::vector<polygon_2D> &polygons,								// In: Static obstacles parametrized as polygons
	Obstacle_Data<Tracked_Obstacle> &data									// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);
	
	trajectory.resize(ownship_state.size(), n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = data.obstacles.size();
	int n_static_obst = polygons.size();								

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
		u_opt = 1.0; 		u_opt_last = u_opt;
		chi_opt = 0.0; 		chi_opt_last = chi_opt;

		assign_optimal_trajectory(predicted_trajectory);

		return;
	}
	
	setup_prediction(data);

	// Intelligent prediction with the nominal ownship trajectory before pruning obstacle
	// scenarios
	if (use_joint_prediction)
	{
		//predict_trajectories_jointly(data, polygons, n_static_obst, false);
	}
	
	//prune_obstacle_scenarios(data);
	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	} */
	
 	/* mxArray *traj_os = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

	double *ptraj_os = mxGetPr(traj_os); 
	double *p_wps_os = mxGetPr(wps_os); 

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
	map_wps = waypoints;

	//Make matlab polygons type friendly array:
    Eigen::Matrix<double, -1, 2> polygon_matrix;
    int n_total_vertices = 0;
    BOOST_FOREACH(polygon_2D const &poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			n_total_vertices += 1;
		}
		n_total_vertices += 1;
    }
    polygon_matrix.resize(n_total_vertices, 2); 

    // format polygon_matrix array for matlab plotting
    int pcount = 0; 
    BOOST_FOREACH(polygon_2D const& poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it); // east 
			polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it); // north format for matlab
			
			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		polygon_matrix(pcount, 1) = -1;
		polygon_matrix(pcount, 0) = -1;
		pcount += 1;
    }
    
    mxArray *polygon_matrix_mx = mxCreateDoubleMatrix(n_total_vertices, 2, mxREAL);
    double *p_polygon_matrix = mxGetPr(polygon_matrix_mx);
    Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, n_total_vertices, 2);
	map_polygon_matrix = polygon_matrix;

	engPutVariable(ep, "P", polygon_matrix_mx);

	mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_obst_mx, *i_mx, *ps_mx, *d_safe_mx;
	dt_sim = mxCreateDoubleScalar(pars.dt);
	T_sim = mxCreateDoubleScalar(pars.T);
	n_ps_mx = mxCreateDoubleScalar(n_ps[0]);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
	
	std::cout << waypoints << std::endl;
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "dt_sim", dt_sim);
	engPutVariable(ep, "T_sim", T_sim);
	engPutVariable(ep, "WPs", wps_os);
	engPutVariable(ep, "d_safe", d_safe_mx);
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
	} */ 
	
	// COST PLOTTING
	/* mxArray *total_cost_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
 	mxArray *cost_do_mx = mxCreateDoubleMatrix(n_obst, pars.n_cbs, mxREAL);
	mxArray *cost_colregs_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
	mxArray *max_cost_i_ps_mx = mxCreateDoubleMatrix(n_obst * pars.n_r, pars.n_cbs, mxREAL);
	mxArray *mu_i_ps_mx = mxCreateDoubleMatrix(n_obst * pars.n_r, pars.n_cbs, mxREAL);
	mxArray *cost_so_path_mx = mxCreateDoubleMatrix(2, pars.n_cbs, mxREAL);
	mxArray *n_ps_copy_mx = mxCreateDoubleMatrix(1, n_obst, mxREAL);
	mxArray *cb_matrix_mx = mxCreateDoubleMatrix(2 * pars.n_M, pars.n_cbs, mxREAL);
	mxArray *Pr_s_i_mx = mxCreateDoubleMatrix(n_obst, n_ps[0], mxREAL);
	
	double *ptr_total_cost = mxGetPr(total_cost_mx); 
	double *ptr_cost_do = mxGetPr(cost_do_mx); 
	double *ptr_cost_colregs = mxGetPr(cost_colregs_mx); 
	double *ptr_max_cost_i_ps = mxGetPr(max_cost_i_ps_mx); 
	double *ptr_mu_i_ps = mxGetPr(mu_i_ps_mx); 
	double *ptr_cost_so_path = mxGetPr(cost_so_path_mx); 
	double *ptr_n_ps_copy = mxGetPr(n_ps_copy_mx); 
	double *ptr_cb_matrix = mxGetPr(cb_matrix_mx); 
	double *ptr_Pr_s_i = mxGetPr(Pr_s_i_mx); 

	Eigen::Map<Eigen::MatrixXd> map_total_cost(ptr_total_cost, 1, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_do(ptr_cost_do, n_obst, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_colregs(ptr_cost_colregs, 1, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_max_cost_i_ps(ptr_max_cost_i_ps, n_obst * pars.n_r, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_mu_i_ps(ptr_mu_i_ps, n_obst * pars.n_r, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_so_path(ptr_cost_so_path, 2, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_n_ps(ptr_n_ps_copy, 1, n_obst);
	Eigen::Map<Eigen::MatrixXd> map_cb_matrix(ptr_cb_matrix, 2 * pars.n_M, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_Pr_s_i(ptr_Pr_s_i, n_obst, n_ps[0]);

	mxArray *n_obst_copy_mx = mxCreateDoubleScalar(n_obst), *opt_cb_index_mx(nullptr); */
	//===============================================================================================================

	Eigen::MatrixXd cost_do_matrix(n_obst, pars.n_cbs);
	Eigen::MatrixXd cost_colregs_matrix(1, pars.n_cbs);
	Eigen::MatrixXd max_cost_i_ps_matrix(n_obst * pars.n_r, pars.n_cbs);
	Eigen::MatrixXd mu_i_ps_matrix(n_obst * pars.n_r, pars.n_cbs);
	Eigen::MatrixXd cb_matrix(2 * pars.n_M, pars.n_cbs);
	Eigen::MatrixXd cost_so_path_matrix(2, pars.n_cbs);
	Eigen::MatrixXd total_cost_matrix(1, pars.n_cbs);
	Eigen::MatrixXd n_ps_matrix(1, n_obst);
	Eigen::MatrixXd Pr_s_i_matrix(n_obst, n_ps[0]);
	for (int i = 0; i < n_obst; i++)
	{
		n_ps_matrix(0, i) = n_ps[i];
		for (int ps = 0; ps < n_ps[i]; ps++)
		{
			Pr_s_i_matrix(i, ps) = data.obstacles[i].get_scenario_probabilities()(ps);
		}
	}
	int curr_ps_index(0);
	Eigen::VectorXd max_cost_i_ps, mu_i_ps;
	

	double cost(0.0), h_do(0.0), h_colregs(0.0), h_so(0.0), h_path(0.0);
	Eigen::VectorXd cost_do(n_obst), mu_i(n_obst);
	std::tuple<double, double> tup;
	Eigen::MatrixXd P_c_i;
	data.HL_0.resize(n_obst); data.HL_0.setZero();
	min_cost = 1e12;
	int min_index = 0;
	int thread_count = 1;
	reset_control_behaviour();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0.0; h_do = 0.0; h_colregs = 0.0; h_so = 0.0; h_path = 0.0;
		for (int M = 0; M < pars.n_M; M++)
		{
			cb_matrix(2 * M, cb) = offset_sequence(2 * M);
			cb_matrix(2 * M + 1, cb) = RAD2DEG * offset_sequence(2 * M + 1);
		}
		
		curr_ps_index = 0;
		//std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
		//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
		
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

		for (int i = 0; i < n_obst; i++)
		{
			
			P_c_i.resize(n_ps[i], n_samples); P_c_i.setZero();
			//calculate_collision_probabilities(P_c_i, data, i, p_step_cpe * pars.dt, pars.p_step_cpe); 

			/* tup = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, P_c_i, data, i, ownship.get_length());
			cost_i(i) = std::get<0>(tup);
			mu_i(i) = std::get<1>(tup); */

			tup = mpc_cost.calculate_dynamic_obstacle_cost(max_cost_i_ps, mu_i_ps, trajectory, P_c_i, data, i, ownship.get_length());
			cost_do(i) = std::get<0>(tup);
			mu_i(i) = std::get<1>(tup);
			max_cost_i_ps_matrix.block(curr_ps_index, cb, n_ps[i], 1) = max_cost_i_ps;
			curr_ps_index += n_ps[i];
			
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | mu_i_ps : %.4f | cb : %.1f, %.1f \n", thread_count, i, ps, cb, max_cost_i_ps(ps), mu_i_ps(ps), offset_sequence(0), RAD2DEG * offset_sequence(1));
				thread_count += 1;
			}
			//===============================================================================================================
			// MATLAB PLOTTING FOR DEBUGGING
			//===============================================================================================================
			/* p_P_c_i = mxGetPr(P_c_i_mx[i]);
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
			} */
			//===============================================================================================================
		}

		h_do = cost_do.sum();
		cost_do_matrix.col(cb) = cost_do;
		
		h_colregs = pars.kappa * std::min(1.0, mu_i.sum());
		cost_colregs_matrix(0, cb) = h_colregs;

		h_so = mpc_cost.calculate_grounding_cost(trajectory, polygons, V_w, wind_direction, pars.p_step_grounding);
		cost_so_path_matrix(0, cb) = h_so;

		h_path += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);
		h_path += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);
		cost_so_path_matrix(1, cb) = h_path;
		
		cost = h_do + h_colregs + h_so + h_path;
		total_cost_matrix(cb) = cost;

		if (cost < min_cost) 
		{
			min_cost = cost;
			min_index = cb;
			opt_offset_sequence = offset_sequence;

			assign_optimal_trajectory(predicted_trajectory);

			// Assign current optimal hazard level for each obstacle
			for (int i = 0; i < n_obst; i++)
			{
				if (cost_do.sum() > 0)
					data.HL_0(i) = cost_do(i) / cost_do.sum();
			}	
		}
		increment_control_behaviour();

		//===============================================================================================================
		// MATLAB PLOTTING FOR DEBUGGING
		//===============================================================================================================
		/* Eigen::Map<Eigen::MatrixXd> map_traj(ptraj_os, trajectory.rows(), n_samples);
		map_traj = trajectory;

		k_s = mxCreateDoubleScalar(n_samples);
		engPutVariable(ep, "k", k_s);

		engPutVariable(ep, "X", traj_os);
		engEvalString(ep, "inside_psbmpc_upd_ownship_plot"); */
		//===============================================================================================================
	}
	//==================================================================
	// MATLAB PLOTTING FOR DEBUGGING AND TUNING
	//==================================================================
	/* opt_cb_index_mx = mxCreateDoubleScalar(min_index + 1);
	map_total_cost = total_cost_matrix;
	map_cost_do = cost_do_matrix;
	map_cost_colregs = cost_colregs_matrix;
	map_max_cost_i_ps = max_cost_i_ps_matrix;
	map_mu_i_ps = mu_i_ps_matrix;
	map_cost_so_path = cost_so_path_matrix;
	map_n_ps = n_ps_matrix;
	map_cb_matrix = cb_matrix;
	map_Pr_s_i = Pr_s_i_matrix;

	engPutVariable(ep, "Pr_s_i", Pr_s_i_mx);
	engPutVariable(ep, "total_cost", total_cost_mx);
	engPutVariable(ep, "cost_do", cost_do_mx);
	engPutVariable(ep, "cost_colregs", cost_colregs_mx);
	engPutVariable(ep, "max_cost_i_ps", max_cost_i_ps_mx);
	engPutVariable(ep, "mu_i_ps", mu_i_ps_mx);
	engPutVariable(ep, "cost_so_path", cost_so_path_mx);
	engPutVariable(ep, "n_ps", n_ps_copy_mx);
	engPutVariable(ep, "cb_matrix", cb_matrix_mx);
	engPutVariable(ep, "n_obst", n_obst_copy_mx);
	engPutVariable(ep, "opt_cb_index", opt_cb_index_mx);
	engEvalString(ep, "psbmpc_cost_plotting");

	mxDestroyArray(total_cost_mx);
	mxDestroyArray(cost_do_mx);
	mxDestroyArray(cost_colregs_mx);
	mxDestroyArray(max_cost_i_ps_mx);
	mxDestroyArray(mu_i_ps_mx);
	mxDestroyArray(cost_so_path_mx);
	mxDestroyArray(n_ps_copy_mx);
	mxDestroyArray(cb_matrix_mx);
	mxDestroyArray(n_obst_copy_mx);
	mxDestroyArray(opt_cb_index_mx);
	
	engClose(ep); */
	//==================================================================

	u_opt = opt_offset_sequence(0); 	u_opt_last = u_opt;
	chi_opt = opt_offset_sequence(1); 	chi_opt_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	//std::cout << "Cost at optimum : " << min_cost << std::endl;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
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
	Eigen::VectorXd xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].kf.get_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].kf.get_state()(1) - xs(1);
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
*  Name     : setup_prediction
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation, and predicts
*			  independent obstacle trajectories using the predictor class.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::setup_prediction(
	Obstacle_Data<Tracked_Obstacle> &data							// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();
	n_ps.resize(n_obst);
	for (int i = 0; i < n_obst; i++)
	{
		n_ps[i] = data.obstacles[i].get_scenario_probabilities().size();
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa, v_0;
	Eigen::Vector4d xs_0, xs_i_0;
	if (trajectory.rows() == 4)
	{
		v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
		v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
	}
	else
	{
		v_0(0) = trajectory(3, 0); v_0(1) = trajectory(4, 0);
		v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
	}
	xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
	xs_0(2) = v_0(0); xs_0(3) = v_0(1);
	// First avoidance maneuver is always at t0
	maneuver_times.setZero();

	double t_cpa_min(0.0), d_safe_i(0.0);
	std::vector<bool> maneuvered_by(n_obst);
	int index_closest(-1);
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
			xs_i_0 = data.obstacles[i].kf.get_state();
			CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

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
	
	//std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : prune_obstacle_scenarios
*  Function : Goes through all generated obstacle prediction scenarios, and selects the
*			  N_r scenarios with highest collision risk for keeping, discarding all others.
*		      NOT USED ATM
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::prune_obstacle_scenarios(
	Obstacle_Data<Tracked_Obstacle> &data							// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();

	int p_step = 2;   
	double dt_r = (double)p_step * pars.dt;
	int n_samples = std::round(pars.T / pars.dt);

	int count_joint_pred_scenarios = 0;

	Eigen::MatrixXd P_c_i;
	Eigen::VectorXi risk_sorted_ps_indices_i;
	Eigen::VectorXi kept_ps_indices_i;
	Eigen::VectorXd R_c_i, P_c_i_ps, C_i;
	for (int i = 0; i < n_obst; i++)
	{			
		P_c_i.resize(n_ps[i], n_samples); P_c_i_ps.resize(n_ps[i]); 
		C_i.resize(n_ps[i]); R_c_i.resize(n_ps[i]);
		risk_sorted_ps_indices_i.resize(n_ps[i]);
		
		calculate_collision_probabilities(P_c_i, data, i, dt_r, p_step);

		calculate_ps_collision_probabilities(P_c_i_ps, P_c_i, i);

		calculate_ps_collision_consequences(C_i, data, i, dt_r, p_step);

		calculate_ps_collision_risks(R_c_i, risk_sorted_ps_indices_i, C_i, P_c_i_ps, data, i);

		//std::cout << risk_sorted_ps_indices_i.transpose() << std::endl;

		// Keep only the n_r prediction scenarios with the highest collision risk
		if (n_ps[i] < pars.n_r)
		{
			kept_ps_indices_i = risk_sorted_ps_indices_i;
		}
		else
		{
			kept_ps_indices_i = risk_sorted_ps_indices_i.block(0, 0, pars.n_r, 1);		
		}

		// Sort indices of ps that are to be kept
		std::sort(kept_ps_indices_i.data(), kept_ps_indices_i.data() + kept_ps_indices_i.size());
		//std::cout << kept_ps_indices_i.transpose() << std::endl;

		// For n_a > 1: Joint prediction/intelligent scenario is the last one in the original set
		if (use_joint_prediction && (kept_ps_indices_i(kept_ps_indices_i.size() - 1) == n_ps[i] - 1))
		{
			count_joint_pred_scenarios += 1;
		}

		n_ps[i] = kept_ps_indices_i.size();
		data.obstacles[i].prune_ps(kept_ps_indices_i);
	}

	// For n_a > 1: All intelligent obstacle scenarios are in this case pruned away
	if (!count_joint_pred_scenarios) 
	{
		use_joint_prediction = false;
	}
}

/****************************************************************************************
*  Name     : calculate_collision_probabilities
*  Function : Estimates collision probabilities for the own-ship and an obstacle i in
*			  consideration. Can use a larger sample time than used in predicting
*			  the vessel trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_collision_probabilities(
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
*  Name     : calculate_ps_collision_probabilities
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated collision probabilities. 
*		      NOT USED ATM
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

	//double product(0.0);
	int n_samples = P_c_i.cols();
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k++)
		{
			/* if (k == 0)	{ product = 1 - P_c_i(ps, k); }
			else		{ product *= (1 - P_c_i(ps, k)); } */
			if (P_c_i(ps, k) > P_c_i_ps(ps))
			{
				P_c_i_ps(ps) = P_c_i(ps, k);
			}
		}
		//P_c_i_ps(ps) = 1 - product;
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_consequences
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated consequence of collision.
*		      NOT USED ATM
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
	if (use_joint_prediction)
	{
		xs_i_colav_p = pobstacles[i].get_trajectory();
	}

	Eigen::Vector2d p_cpa, v_0_p, v_i_p;

	int n_samples = xs_i_p[0].cols();

	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k += p_step)
		{
			t = k * dt;

			if (trajectory.rows() == 4)
			{
				v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
				v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
			}
			else
			{
				v_0_p = trajectory.block<2, 1>(3, k);
				v_0_p = CPU::rotate_vector_2D(v_0_p, trajectory(2, k));
			}

			if (ps == n_ps[i] - 1 && use_joint_prediction) // Intelligent prediction is the last prediction scenario
			{
				v_i_p = xs_i_colav_p.block<2, 1>(2, k);
			}
			else
			{
				v_i_p = xs_i_p[ps].block<2, 1>(2, k);
			}			

			calculate_cpa(p_cpa, t_cpa, d_cpa, trajectory.col(k), xs_i_p[ps].col(k));

			collision_consequence = pow((v_0_p - v_i_p).norm(), 2) * exp(- abs(t - t_cpa));
			//collision_consequence = pow((v_0_p - v_i_p).norm(), 2) * exp(- abs(t));

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
*		      NOT USED ATM
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_risks(
	Eigen::VectorXd &R_c_i,												// In/out: Vector of collision risks, size n_ps_i x 1
	Eigen::VectorXi &ps_indices_i,										// In/out: Vector of indices for the ps, in decending order wrt collision risk, size n_ps_i x 1
	const Eigen::VectorXd &C_i,											// In: Vector of collision consequences, size n_ps_i x 1
	const Eigen::VectorXd &P_c_i_ps,									// In: Vector of collision probabilities, size n_ps_i x 1
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i															// In: Index of obstacle	
	)
{
	R_c_i.setZero();

	Eigen::VectorXd Pr_c_i_conditional(n_ps[i]);
	Eigen::VectorXd Pr_s_i = data.obstacles[i].get_scenario_probabilities();
	
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		ps_indices_i[ps] = ps;

		Pr_c_i_conditional(ps) = P_c_i_ps(ps) * Pr_s_i(ps);

		R_c_i(ps) = C_i(ps) * Pr_c_i_conditional(ps);
	}

	// Sort vector of ps indices to determine collision risk in sorted order
	std::sort(ps_indices_i.data(), ps_indices_i.data() + n_ps[i], [&](const int index_lhs, const int index_rhs) { return R_c_i(index_lhs) > R_c_i(index_rhs); });
	
	/* std::ios::fmtflags old_settings = std::cout.flags();
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
		std::cout 	<< ps_indices_i[j] << std::setw(cw) << R_c_i(ps_indices_i(j)) << std::setw(cw) << C_i(ps_indices_i(j)) << std::setw(cw) 
					<< P_c_i_ps(ps_indices_i(j)) << std::setw(cw) << Pr_c_i_conditional(ps_indices_i(j)) << std::endl;
	}
	std::cout.flags(old_settings);
	std::cout << std::setprecision(old_precision);
	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl; */
}

/****************************************************************************************
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the obstacles with an active COLAV system,  
*		      considering the fixed current control behaviour, and adds or overwrites
*			  this information to the obstacle data structure. Static obstacles
*			  parametrized as no-go lines/straight lines.
*		      NOT USED ATM
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::predict_trajectories_jointly(
	Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const Eigen::Matrix<double, 4, -1>& static_obstacles,		// In: Static obstacles parametrized as no-go lines/straight lines
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
	//double mean_t(0.0);

	double t(0.0), u_c_i(0.0), chi_c_i(0.0), chi_i(0.0);
	Eigen::Vector2d v_os_k;
	for (int k = 0; k < n_samples; k++)
	{	
		t = k * pars.dt;

		if (trajectory.rows() == 4)
		{
			v_os_k(0) = trajectory(3, k) * cos(trajectory(2, k));
			v_os_k(1) = trajectory(3, k) * sin(trajectory(2, k));
		}
		else
		{
			v_os_k(0) = trajectory(3, k); 
			v_os_k(1) = trajectory(4, k); 
			v_os_k = rotate_vector_2D(v_os_k, trajectory(2, k));
		}
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
				//start = std::chrono::system_clock::now();	

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

				/* mean_t = elapsed.count();
				std::cout << "Obstacle_SBMPC time usage : " << mean_t << " milliseconds" << std::endl; */

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
		data.obstacles[i].add_intelligent_prediction(&pobstacles[i], overwrite);
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
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the obstacles with an active COLAV system,  
*		      considering the fixed current control behaviour, and adds or overwrites
*			  this information to the obstacle data structure. Static obstacles
*			  parametrized as polygons.
*		      NOT USED ATM
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::predict_trajectories_jointly(
	Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const std::vector<polygon_2D> &polygons,					// In: Static obstacles parametrized as polygons
	const int n_static_obst,									// In: Number of polygons/static obstacles
	const bool overwrite										// In: Flag to choose whether or not to add the first intelligent prediction to the data, or overwrite the previous
	)
{
	Eigen::Matrix<double, 4, -1> static_obstacles(4, 1); static_obstacles.setZero();
	predict_trajectories_jointly(data, static_obstacles, overwrite);
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

}
}