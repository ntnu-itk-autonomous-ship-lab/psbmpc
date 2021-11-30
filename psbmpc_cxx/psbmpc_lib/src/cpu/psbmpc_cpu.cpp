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

#if ENABLE_PSBMPC_DEBUGGING
#include "engine.h"
#define BUFFSIZE 100000
#endif

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
			: u_opt_last(1.0), chi_opt_last(0.0), min_cost(1e12)
		{
			offset_sequence_counter.resize(2 * pars.n_M);
			offset_sequence.resize(2 * pars.n_M);
			maneuver_times.resize(pars.n_M);

			cpe = CPE(pars.cpe_method);

			mpc_cost = MPC_Cost<PSBMPC_Parameters>(pars);
		}

		PSBMPC::PSBMPC(
			const Ownship &ownship,				 // In: Own-ship with specific parameter set
			const CPE &cpe,						 // In: CPE with specific parameter set
			const PSBMPC_Parameters &psbmpc_pars // In: Parameter object to initialize the PSB-MPC
			) : u_opt_last(1.0), chi_opt_last(0.0), min_cost(1e12), ownship(ownship), cpe(cpe), pars(psbmpc_pars), mpc_cost(psbmpc_pars)
		{
			offset_sequence_counter.resize(2 * pars.n_M);
			offset_sequence.resize(2 * pars.n_M);
			maneuver_times.resize(pars.n_M);
		}

		/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : W/static obstacles parametrized as polygons
*  Author   : Trym Tengesdal & Tom Daniel Grande
*  Modified :
*****************************************************************************************/
		void PSBMPC::calculate_optimal_offsets(
			double &u_opt,								   // In/out: Optimal surge offset
			double &chi_opt,							   // In/out: Optimal course offset
			Eigen::MatrixXd &predicted_trajectory,		   // In/out: Predicted optimal ownship trajectory
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Next waypoints
			const Eigen::VectorXd &ownship_state,		   // In: Current ship state
			const double V_w,							   // In: Estimated wind speed
			const Eigen::Vector2d &wind_direction,		   // In: Unit vector in NE describing the estimated wind direction
			const Static_Obstacles &polygons,			   // In: Static obstacles parametrized as polygons
			const Dynamic_Obstacles &obstacles,			   // In: Dynamic obstacle information
			const bool disable							   // In: Disable the COLAV functionality or not
		)
		{

			int n_samples = std::round(pars.T / pars.dt);

			trajectory.resize(ownship_state.size(), n_samples);
			trajectory.col(0) = ownship_state;

			ownship.determine_active_waypoint_segment(waypoints, ownship_state);

			int n_do = obstacles.size();
			int n_so = polygons.size();

			Eigen::VectorXd opt_offset_sequence(2 * pars.n_M);

			// Predict nominal trajectory first, assign as optimal if no need for
			// COLAV, or use in the prediction initialization
			for (int M = 0; M < pars.n_M; M++)
			{
				offset_sequence(2 * M) = 1.0;
				offset_sequence(2 * M + 1) = 0.0;
			}
			maneuver_times.setZero();
			ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

			bool colav_active = determine_colav_active(obstacles, n_so, disable);
			if (!colav_active)
			{
				u_opt = 1.0;
				u_opt_last = u_opt;
				chi_opt = 0.0;
				chi_opt_last = chi_opt;

				assign_optimal_trajectory(predicted_trajectory);

				return;
			}

			for (int i = 0; i < n_do; i++)
			{
				mpc_cost.update_colregs_violation_node(ownship_state, obstacles[i].kf.get_state(), i);
			}

			setup_prediction(obstacles);

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			Engine *ep = engOpen(NULL);
			if (ep == NULL)
			{
				std::cout << "engine start failed!" << std::endl;
			}

			mxArray *init_state_os_mx = mxCreateDoubleMatrix(ownship_state.size(), 1, mxREAL);
			mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
			mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

			double *p_init_state_os = mxGetPr(init_state_os_mx);
			double *p_traj_os = mxGetPr(traj_os_mx);
			double *p_wps_os = mxGetPr(wps_os);

			Eigen::Map<Eigen::VectorXd> map_init_state_os(p_init_state_os, ownship_state.size(), 1);
			Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
			map_init_state_os = ownship_state;
			map_wps = waypoints;

			Eigen::MatrixXd n_ps_matrix(1, n_do);
			int n_ps_max(0);
			for (int i = 0; i < n_do; i++)
			{
				n_ps_matrix(0, i) = n_ps[i];
				if (n_ps_max < n_ps[i])
				{
					n_ps_max = n_ps[i];
				}
			}
			mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_do_mx, *n_so_mx, *i_mx, *ps_mx, *d_safe_mx, *n_patches_mx;
			dt_sim = mxCreateDoubleScalar(pars.dt);
			T_sim = mxCreateDoubleScalar(pars.T);
			n_ps_mx = mxCreateDoubleScalar(n_ps_max);
			n_do_mx = mxCreateDoubleScalar(n_do);
			d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
			n_so_mx = mxCreateDoubleScalar(n_so);
			n_patches_mx = mxCreateDoubleScalar(5);

			engPutVariable(ep, "ownship_state", init_state_os_mx);
			engPutVariable(ep, "n_ps", n_ps_mx);
			engPutVariable(ep, "n_do", n_do_mx);
			engPutVariable(ep, "n_so", n_so_mx);
			engPutVariable(ep, "dt_sim", dt_sim);
			engPutVariable(ep, "T_sim", T_sim);
			engPutVariable(ep, "WPs", wps_os);
			engPutVariable(ep, "d_safe", d_safe_mx);
			engPutVariable(ep, "n_patches", n_patches_mx);
			engEvalString(ep, "inside_psbmpc_init_plot");

			Eigen::Matrix<double, 2, -1> polygon_matrix;
			int n_total_vertices = 0;
			mxArray *polygon_matrix_mx(nullptr);
			mxArray *d_0j_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
			mxArray *j_mx(nullptr);
			double *p_polygon_matrix(nullptr);
			double *p_d_0j = mxGetPr(d_0j_mx);

			Eigen::Map<Eigen::Vector2d> map_d_0j(p_d_0j, 2, 1);
			int pcount;
			Eigen::Vector2d d_0j;
			for (int j = 0; j < n_so; j++)
			{
				j_mx = mxCreateDoubleScalar(j + 1);
				n_total_vertices = 0;
				for (auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
				{
					n_total_vertices += 1;
				}
				polygon_matrix.resize(2, n_total_vertices);
				pcount = 0;
				for (auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
				{
					polygon_matrix(0, pcount) = boost::geometry::get<0>(*it);
					polygon_matrix(1, pcount) = boost::geometry::get<1>(*it);

					pcount += 1;
				}
				d_0j = mpc_cost.distance_to_polygon(ownship_state.block<2, 1>(0, 0), polygons[j]);
				map_d_0j = d_0j;

				polygon_matrix_mx = mxCreateDoubleMatrix(2, n_total_vertices, mxREAL);
				p_polygon_matrix = mxGetPr(polygon_matrix_mx);
				Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, 2, n_total_vertices);
				map_polygon_matrix = polygon_matrix;
				engPutVariable(ep, "j", j_mx);
				engPutVariable(ep, "d_0j", d_0j_mx);
				engPutVariable(ep, "polygon_matrix_j", polygon_matrix_mx);
				engEvalString(ep, "inside_psbmpc_static_obstacle_plot");
			}

			mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
			mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

			double *p_traj_i = mxGetPr(traj_i);
			double *p_P_traj_i = mxGetPr(P_traj_i);
			double *p_P_c_i;

			Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i, 4, n_samples);
			Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);

			std::vector<mxArray *> P_c_i_mx(n_do);

			for (int i = 0; i < n_do; i++)
			{
				P_c_i_mx[i] = mxCreateDoubleMatrix(n_ps[i], n_samples, mxREAL);

				Eigen::MatrixXd P_i_p = obstacles[i].get_trajectory_covariance();
				std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

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

			Eigen::MatrixXd Pr_s_i_matrix(n_do, n_ps_max);
			for (int i = 0; i < n_do; i++)
			{
				for (int ps = 0; ps < n_ps[i]; ps++)
				{
					Pr_s_i_matrix(i, ps) = obstacles[i].get_scenario_probabilities()(ps);
				}
			}
			mxArray *total_cost_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
			mxArray *cost_do_mx = mxCreateDoubleMatrix(n_do, pars.n_cbs, mxREAL);
			mxArray *cost_colregs_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
			mxArray *h_do_i_ps_mx = mxCreateDoubleMatrix(n_do * n_ps_max, pars.n_cbs, mxREAL);
			mxArray *h_so_j_mx = mxCreateDoubleMatrix(n_so, pars.n_cbs, mxREAL);
			mxArray *cost_so_path_mx = mxCreateDoubleMatrix(2, pars.n_cbs, mxREAL);
			mxArray *n_ps_copy_mx = mxCreateDoubleMatrix(1, n_do, mxREAL);
			mxArray *cb_matrix_mx = mxCreateDoubleMatrix(2 * pars.n_M, pars.n_cbs, mxREAL);
			mxArray *Pr_s_i_mx = mxCreateDoubleMatrix(n_do, n_ps_max, mxREAL);

			double *ptr_total_cost = mxGetPr(total_cost_mx);
			double *ptr_cost_do = mxGetPr(cost_do_mx);
			double *ptr_cost_colregs = mxGetPr(cost_colregs_mx);
			double *ptr_h_do_i_ps = mxGetPr(h_do_i_ps_mx);
			double *ptr_h_so_j = mxGetPr(h_so_j_mx);
			double *ptr_cost_so_path = mxGetPr(cost_so_path_mx);
			double *ptr_n_ps_copy = mxGetPr(n_ps_copy_mx);
			double *ptr_cb_matrix = mxGetPr(cb_matrix_mx);
			double *ptr_Pr_s_i = mxGetPr(Pr_s_i_mx);

			Eigen::Map<Eigen::MatrixXd> map_total_cost(ptr_total_cost, 1, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_do(ptr_cost_do, n_do, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_colregs(ptr_cost_colregs, 1, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_h_do_i_ps(ptr_h_do_i_ps, n_do * n_ps_max, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_h_so_j(ptr_h_so_j, n_so, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_so_path(ptr_cost_so_path, 2, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_n_ps(ptr_n_ps_copy, 1, n_do);
			Eigen::Map<Eigen::MatrixXd> map_cb_matrix(ptr_cb_matrix, 2 * pars.n_M, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_Pr_s_i(ptr_Pr_s_i, n_do, n_ps_max);

			mxArray *n_do_copy_mx = mxCreateDoubleScalar(n_do), *n_so_copy_mx = mxCreateDoubleScalar(n_so), *opt_cb_index_mx(nullptr);

			Eigen::MatrixXd cost_do_matrix(n_do, pars.n_cbs);
			Eigen::MatrixXd cost_colregs_matrix(1, pars.n_cbs);
			Eigen::MatrixXd h_do_i_ps_matrix(n_do * n_ps_max, pars.n_cbs);
			Eigen::MatrixXd h_so_j_matrix(n_so, pars.n_cbs);
			Eigen::MatrixXd cb_matrix(2 * pars.n_M, pars.n_cbs);
			Eigen::MatrixXd cost_so_path_matrix(2, pars.n_cbs);
			Eigen::MatrixXd total_cost_matrix(1, pars.n_cbs);

			int curr_ps_index(0);
			int min_index = 0;
			int thread_count = 1;

			if (n_so == 0)
			{
				h_so_j_matrix.resize(1, pars.n_cbs);
				h_so_j_matrix.setZero();
			}
#endif

			double cost(0.0), h_do(0.0), h_colregs(0.0), h_so(0.0), h_path(0.0);
			Eigen::VectorXd h_do_i_ps, h_so_j;
			Eigen::VectorXd cost_do(n_do);
			Eigen::MatrixXd P_c_i;
			min_cost = 1e12;
			reset_control_behaviour();
			for (int cb = 0; cb < pars.n_cbs; cb++)
			{
				cost = 0.0;
				h_do = 0.0;
				h_colregs = 0.0;
				h_so = 0.0;
				h_path = 0.0;

#if ENABLE_PSBMPC_DEBUGGING
				for (int M = 0; M < pars.n_M; M++)
				{
					cb_matrix(2 * M, cb) = offset_sequence(2 * M);
					cb_matrix(2 * M + 1, cb) = RAD2DEG * offset_sequence(2 * M + 1);
				}
				curr_ps_index = 0;
				//std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
				//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
#endif

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

				for (int i = 0; i < n_do; i++)
				{

					P_c_i.resize(n_ps[i], n_samples); //P_c_i.setZero();
					calculate_collision_probabilities(P_c_i, obstacles, i, pars.p_step_do * pars.dt, pars.p_step_do);

#if ENABLE_PSBMPC_DEBUGGING
					cost_do(i) = mpc_cost.calculate_dynamic_obstacle_cost(h_do_i_ps, trajectory, P_c_i, obstacles, i, ownship.get_length());
					h_do_i_ps_matrix.block(curr_ps_index, cb, n_ps[i], 1) = h_do_i_ps;
					curr_ps_index += n_ps[i];
					for (int ps = 0; ps < n_ps[i]; ps++)
					{
						//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f \n", thread_count, i, ps, cb, h_do_i_ps(ps), offset_sequence(0), RAD2DEG * offset_sequence(1));
						thread_count += 1;
					}
#else
					cost_do(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, P_c_i, obstacles, i, ownship.get_length());
#endif

//===============================================================================================================
// MATLAB PLOTTING FOR DEBUGGING
//===============================================================================================================
#if ENABLE_PSBMPC_DEBUGGING
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
#endif
					//===============================================================================================================
				}
				if (n_do > 0)
				{
					h_do = cost_do.sum();
				}

				h_colregs = mpc_cost.calculate_colregs_violation_cost(trajectory, obstacles);

#if ENABLE_PSBMPC_DEBUGGING
				h_so = mpc_cost.calculate_grounding_cost(h_so_j, trajectory, polygons, V_w, wind_direction);
#else
				h_so = mpc_cost.calculate_grounding_cost(trajectory, polygons, V_w, wind_direction);
#endif

				h_path += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);
				h_path += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);

				cost = h_do + h_colregs + h_so + h_path;

#if ENABLE_PSBMPC_DEBUGGING
				if (n_do > 0)
				{
					cost_do_matrix.col(cb) = cost_do;
				}
				cost_colregs_matrix(0, cb) = h_colregs;
				if (n_so > 0)
				{
					h_so_j_matrix.block(0, cb, n_so, 1) = h_so_j;
				}
				cost_so_path_matrix(0, cb) = h_so;
				cost_so_path_matrix(1, cb) = h_path;
				total_cost_matrix(cb) = cost;
				if (cost < min_cost)
				{
					min_index = cb;
				}
#endif

				if (cost < min_cost)
				{
					min_cost = cost;
					opt_offset_sequence = offset_sequence;

					assign_optimal_trajectory(predicted_trajectory);
				}
				increment_control_behaviour();

//===============================================================================================================
// MATLAB PLOTTING FOR DEBUGGING
//===============================================================================================================
#if ENABLE_PSBMPC_DEBUGGING
				/* Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_os, trajectory.rows(), n_samples);
			map_traj = trajectory;

			k_s = mxCreateDoubleScalar(n_samples);
			engPutVariable(ep, "k", k_s);

			engPutVariable(ep, "X", traj_os_mx);
			engEvalString(ep, "inside_psbmpc_upd_ownship_plot"); */
#endif
				//===============================================================================================================
			}
//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			opt_cb_index_mx = mxCreateDoubleScalar(min_index + 1);
			map_total_cost = total_cost_matrix;
			map_cost_colregs = cost_colregs_matrix;
			if (n_so > 0)
			{
				map_h_so_j = h_so_j_matrix;
			}
			if (n_do > 0)
			{
				map_cost_do = cost_do_matrix;
				map_h_do_i_ps = h_do_i_ps_matrix;
				map_Pr_s_i = Pr_s_i_matrix;
			}
			map_cost_so_path = cost_so_path_matrix;
			map_n_ps = n_ps_matrix;
			map_cb_matrix = cb_matrix;

			mxArray *is_gpu_mx = mxCreateDoubleScalar(0);
			engPutVariable(ep, "is_gpu", is_gpu_mx);
			engPutVariable(ep, "h_so_j", h_so_j_mx);
			engPutVariable(ep, "total_cost", total_cost_mx);
			engPutVariable(ep, "cost_do", cost_do_mx);
			engPutVariable(ep, "h_do_i_ps", h_do_i_ps_mx);
			engPutVariable(ep, "Pr_s_i", Pr_s_i_mx);
			engPutVariable(ep, "cost_colregs", cost_colregs_mx);
			engPutVariable(ep, "cost_so_path", cost_so_path_mx);
			engPutVariable(ep, "n_ps", n_ps_copy_mx);
			engPutVariable(ep, "cb_matrix", cb_matrix_mx);
			engPutVariable(ep, "n_do", n_do_copy_mx);
			engPutVariable(ep, "n_so", n_so_copy_mx);
			engPutVariable(ep, "opt_cb_index", opt_cb_index_mx);
			engEvalString(ep, "psbmpc_cost_plotting");

			mxDestroyArray(is_gpu_mx);
			mxDestroyArray(total_cost_mx);
			mxDestroyArray(cost_do_mx);
			mxDestroyArray(cost_colregs_mx);
			mxDestroyArray(h_do_i_ps_mx);
			mxDestroyArray(h_so_j_mx);
			mxDestroyArray(cost_so_path_mx);
			mxDestroyArray(n_ps_copy_mx);
			mxDestroyArray(cb_matrix_mx);
			mxDestroyArray(n_do_copy_mx);
			mxDestroyArray(n_so_copy_mx);
			mxDestroyArray(opt_cb_index_mx);

			engClose(ep);
#endif
			//==================================================================

			u_opt = opt_offset_sequence(0);
			u_opt_last = u_opt;
			chi_opt = opt_offset_sequence(1);
			chi_opt_last = chi_opt;

			std::cout << "Optimal offset sequence : ";
			for (int M = 0; M < pars.n_M; M++)
			{
				std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
				if (M < pars.n_M - 1)
					std::cout << ", ";
			}
			std::cout << std::endl;

			std::cout << "Cost at optimum : " << min_cost << std::endl;
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
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int n_so,						// In: Number of static obstacles
			const bool disable					// In: Disable the COLAV functionality or not
		)
		{
			if (disable)
			{
				return false;
			}
			Eigen::VectorXd xs = trajectory.col(0);
			bool colav_active = false;
			Eigen::Vector2d d_0i;
			for (size_t i = 0; i < obstacles.size(); i++)
			{
				d_0i(0) = obstacles[i].kf.get_state()(0) - xs(0);
				d_0i(1) = obstacles[i].kf.get_state()(1) - xs(1);
				if (d_0i.norm() < pars.d_init)
					colav_active = true;

				// If all obstacles are passed, even though inside colav range,
				// then no need for colav
				if (ship_is_passed_by(xs, obstacles[i].kf.get_state(), pars.d_safe))
				{
					colav_active = false;
				}
				else
				{
					colav_active = true;
				}
			}
			colav_active = colav_active || n_so > 0;

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
			const Dynamic_Obstacles &obstacles // In: Dynamic obstacle information
		)
		{
			int n_do = obstacles.size();
			n_ps.resize(n_do);
			for (int i = 0; i < n_do; i++)
			{
				n_ps[i] = obstacles[i].get_scenario_probabilities().size();
			}
			//***********************************************************************************
			// Own-ship prediction initialization
			//***********************************************************************************
			Eigen::VectorXd t_cpa(n_do), d_cpa(n_do);
			Eigen::Vector2d p_cpa, v_0;
			Eigen::Vector4d xs_0, xs_i_0;
			if (trajectory.rows() == 4)
			{
				v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
				v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
			}
			else
			{
				v_0(0) = trajectory(3, 0);
				v_0(1) = trajectory(4, 0);
				v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
			xs_0(2) = v_0(0);
			xs_0(3) = v_0(1);
			// First avoidance maneuver is always at t0
			maneuver_times.setZero();

			double t_cpa_min(0.0), d_safe_i(0.0);
			std::vector<bool> maneuvered_by(n_do);
			int index_closest(-1);
			for (int M = 1; M < pars.n_M; M++)
			{
				// If a predicted collision occurs with the closest obstacle, avoidance maneuver
				// M is taken right after the obstacle possibly maneuvers (modelled to be at t_0 + M * t_ts
				// given that t_cpa > t_ts. If t_cpa < t_ts or n_do = 0, the subsequent maneuver is taken
				// at t_{M-1} + t_ts + 1 anyways (simplification)
				maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);

				// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
				t_cpa_min = 1e10;
				index_closest = -1;
				for (int i = 0; i < n_do; i++)
				{
					xs_i_0 = obstacles[i].kf.get_state();
					CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

					d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[i].get_length());
					// For the current avoidance maneuver, determine which obstacle that should be
					// considered, i.e. the closest obstacle that is not already passed (which means
					// that the previous avoidance maneuver happened before CPA with this obstacle)
					if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min && t_cpa(i) <= pars.T)
					{
						t_cpa_min = t_cpa(i);
						index_closest = i;
					}
				}

				if (index_closest != -1)
				{
					d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[index_closest].get_width());
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
*  Name     : calculate_collision_probabilities
*  Function : Estimates collision probabilities for the own-ship and an obstacle i in
*			  consideration. Can use a larger sample time than used in predicting
*			  the vessel trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
		void PSBMPC::calculate_collision_probabilities(
			Eigen::MatrixXd &P_c_i,				// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int i,						// In: Index of obstacle
			const double dt,					// In: Sample time for estimation
			const int p_step					// In: Step between trajectory samples, matches the input prediction time step
		)
		{
			Eigen::MatrixXd P_i_p = obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

			// Increase safety zone by half the max obstacle dimension and ownship length
			double d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[i].get_length());

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
			Eigen::MatrixXd &optimal_trajectory // In/out: Optimal PSB-MPC trajectory
		)
		{
			int n_samples = std::round(pars.T / pars.dt);
			// Set current optimal x-y position trajectory, downsample if linear prediction was not used
			if (false) //(pars.prediction_method > Linear)
			{
				int count = 0;
				optimal_trajectory.resize(trajectory.rows(), n_samples / pars.p_step);
				for (int k = 0; k < n_samples; k += pars.p_step)
				{
					optimal_trajectory.col(count) = trajectory.col(k);
					if (count < std::round(n_samples / pars.p_step) - 1)
						count++;
				}
			}
			else
			{
				optimal_trajectory.resize(trajectory.rows(), n_samples);
				optimal_trajectory = trajectory.block(0, 0, trajectory.rows(), n_samples);
			}
		}

	}
}
