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

#include "utilities.h"
#include "psbmpc.cuh"
#include "cb_cost_functor.cuh"

#include <iostream>
#include "engine.h"

/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC() 
	: 
	ownship(Ownship()), pars(PSBMPC_Parameters()), fdata_device_ptr(nullptr), obstacles_device_ptr(nullptr)
{
	u_m_last = 1; chi_m_last = 0;

	map_offset_sequences();

	// Only allocate CPE device memory once, thus changing dt will not be allowed. 
	cudaMalloc((void**)&cpe_device_ptr, sizeof(CPE));
    cuda_check_errors("Malloc of CPE failed.");

	CPE temporary_cpe(pars.cpe_method, pars.dt);

	cudaMemcpy(cpe_device_ptr, &temporary_cpe, sizeof(CPE), cudaMemcpyHostToDevice);
    cuda_check_errors("MemCpy of CPE failed.");
}

/****************************************************************************************
*  Name     : ~PSBMPC
*  Function : Class destructor
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::~PSBMPC() 
{
	fdata_device_ptr = nullptr;
	
	obstacles_device_ptr = nullptr;

	cudaFree(cpe_device_ptr);
	cuda_check_errors("cudaFree of CPE failed.");
};

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
	Obstacle_Data &odata													// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(6, n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = odata.obstacles.size();
	int n_static_obst = static_obstacles.cols();

	bool colav_active = determine_colav_active(odata, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		return;
	}
	
	initialize_prediction(odata);

	for (int i = 0; i < n_obst; i++)
	{
		if (!pars.obstacle_colav_on)
		{
			// PSBMPC parameters needed to determine if obstacle breaches COLREGS 
			// (future: implement simple sbmpc class for obstacle which has the "determine COLREGS violation" function)
			odata.obstacles[i].predict_independent_trajectories(pars.T, pars.dt, trajectory.col(0), pars.phi_AH, pars.phi_CR, pars.phi_HO, pars.phi_OT, pars.d_close, pars.d_safe);
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
			//engEvalString(ep, "inside_psbmpc_obstacle_plot");
		}
	} */
	
	//===============================================================================================================

	//===============================================================================================================
	// Cost evaluation
	//===============================================================================================================
	set_up_temporary_device_memory(u_d, chi_d, waypoints, static_obstacles, odata);
    
	Eigen::VectorXd HL_0(n_obst); HL_0.setZero();
	
	// Allocate device vector for computing CB costs
	thrust::device_vector<double> cb_costs(pars.n_cbs);

	// Allocate iterator for passing the index of the control behavior to the kernels
	thrust::device_vector<unsigned int> cb_index_dvec(pars.n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

	auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), control_behavior_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), control_behavior_dvec.end()));
	
	// Perform the calculations on the GPU
	cb_cost_functor.reset(new CB_Cost_Functor(pars, fdata_device_ptr, obstacles_device_ptr, cpe_device_ptr));
    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), *cb_cost_functor);

	// Extract minimum cost
	thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
	min_index = min_cost_iter - cb_costs.begin();
	min_cost = cb_costs[min_index];

	// Assign optimal offset sequence/control behaviour
	TML::PDMatrix<double, 2 * MAX_N_M, 1> opt_offset_sequence = control_behavior_dvec[min_index];
	Eigen::VectorXd opt_offset_sequence_e;
	TML::assign_tml_object(opt_offset_sequence_e, opt_offset_sequence);

	// Set the trajectory to the optimal one and assign to the output trajectory
	ownship.predict_trajectory(trajectory, opt_offset_sequence_e, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
	assign_optimal_trajectory(predicted_trajectory);

	clear_temporary_device_memory();
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

	u_opt = opt_offset_sequence_e(0); 		u_m_last = u_opt;
	chi_opt = opt_offset_sequence_e(1); 	chi_m_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence_e(2 * M) << ", " << opt_offset_sequence_e(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : map_offset_sequences
*  Function : Maps the currently set surge and course modifications into a matrix of 
*			  offset sequences or control behaviours
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::map_offset_sequences()
{
	Eigen::VectorXd offset_sequence_counter(2 * pars.n_M), offset_sequence(2 * pars.n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);

	TML::PDMatrix<double, 2 * MAX_N_M, 1> tml_offset_sequence(2 * pars.n_M, 1);

	control_behavior_dvec.resize(pars.n_cbs);
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		TML::assign_eigen_object(tml_offset_sequence, offset_sequence);

		control_behavior_dvec[cb] = tml_offset_sequence;

		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
	std::cout << "Number of control behaviours: " << pars.n_cbs << std::endl;
}

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branch of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::reset_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
)
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
void PSBMPC::increment_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
	)
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
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_prediction(
	Obstacle_Data &odata													// In: Dynamic obstacle information
	)
{
	int n_obst = odata.obstacles.size();

	n_ps.resize(n_obst);

	int n_a = 1; // default
	if (n_obst > 0)
	{
		n_a = odata.obstacles[0].get_intention_probabilities().size();
	}
	
	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	int n_turns;
	std::vector<Intention> ps_ordering_i;
	Eigen::VectorXd ps_course_changes_i;
	Eigen::VectorXd ps_weights_i;
	Eigen::VectorXd ps_maneuver_times_i;

	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa;
	for (int i = 0; i < n_obst; i++)
	{
		//Typically three intentions: KCC, SM, PM
		//std::cout << trajectory.col(0).transpose() << std::endl;
		//std::cout << obstacles[i]->kf.get_state() << std::endl;
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), odata.obstacles[i].kf.get_state());
		//std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
		//std::cout << "t_cpa = " << t_cpa(i) << std::endl;
		//std::cout << "d_cpa = " << d_cpa(i) << std::endl;
		if (n_a == 1)
		{
			n_ps[i] = 1;
			ps_ordering_i.resize(1);
			if (!pars.obstacle_colav_on)	{ ps_ordering_i[0] = KCC; } // One intention: KCC for independent obstacle prediction
			else							{ ps_ordering_i[0] = SM;  } // and CC starboard maneuver for dependent obstacle prediction
			
			ps_course_changes_i.resize(1);
			ps_course_changes_i[0] = 0;
			ps_weights_i.resize(1);
			ps_weights_i(0)= 1;
			ps_maneuver_times_i.resize(1);
			ps_maneuver_times_i(0) = 0;
		}
		else
		{
			if (!pars.obstacle_colav_on)
			{
				// Space obstacle maneuvers evenly throughout horizon, depending on CPA configuration
				if (d_cpa(i) > pars.d_safe || (d_cpa(i) <= pars.d_safe && t_cpa(i) > pars.T)) // No predicted collision inside time horizon
				{
					n_turns = std::floor(pars.T / pars.t_ts);
				} 
				else  // Safety zone violation at CPA inside prediction horizon, as d_cpa <= d_safe				
				{
					if (t_cpa(i) > pars.t_ts)	{ n_turns = std::floor(t_cpa(i) / pars.t_ts); }
					else					{ n_turns = 1; }	
				}

				n_ps[i] = 1 + 2 * pars.obstacle_course_changes.size() * n_turns;
				set_up_independent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, n_turns, odata, i);
			}
			else // Set up dependent obstacle prediction scenarios
			{
				n_ps[i] = 3;
				set_up_dependent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, odata, i);
			}	
		}
		odata.obstacles[i].initialize_prediction(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i);		
	}
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
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + odata.obstacles[i].get_length());
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
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + odata.obstacles[index_closest].get_length());
			// If no predicted collision,  avoidance maneuver M with the closest
			// obstacle (that is not passed) is taken at t_cpa_min
			if (d_cpa(index_closest) > d_safe_i)
			{
				std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
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
void PSBMPC::set_up_independent_obstacle_prediction_variables(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_weights_i, 											// In/out: Cost weights of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const int n_turns, 														// In: number of predicted turns for the obstacle 
	const Obstacle_Data &odata,												// In: Dynamic obstacle information
	const int i 															// In: Index of obstacle in consideration
	)
{
	double Pr_CC_i, t_obst_passed;
	int turn_count, course_change_count;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC;
	ps_maneuver_times_i.resize(n_ps[i]);
	ps_maneuver_times_i[0] = 0;
	ps_course_changes_i.resize(n_ps[i]);
	ps_course_changes_i[0] = 0;
	turn_count = 0;
	course_change_count = 0;
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (n_ps[i] - 1) / 2 + 1)
		{
			ps_ordering_i[ps] = SM;

			ps_maneuver_times_i[ps] = turn_count * std::floor(pars.t_ts / pars.dt);

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

			ps_maneuver_times_i[ps] = turn_count * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = - pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}	
	}
	//std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
	std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl;
	// Determine prediction scenario cost weights based on situation type and correct behavior (COLREGS)
	ps_weights_i.resize(n_ps[i]);
	Pr_CC_i = odata.obstacles[i].get_a_priori_CC_probability();
	switch(odata.ST_i_0[i])
	{
		case A : // Outside CC consideration zone
			ps_weights_i(0) = 1;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1;	
			}
			break;
		case B : // OT, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case C : // CR, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case D : // OT, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = Pr_CC_i;
			}
			break;
		case E : // HO, GW
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	

				// Starboard maneuvers, which are CC
				if (ps > 0 && ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;	
				}
			}
			break;
		case F : // CR, GW
			t_obst_passed = find_time_of_passing(odata, i);
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					// Crossing obstacle prediction scenario gets COLREGS compliant weight Pr_CC_i
					// if the course change is COLREGS compliant and happens in time before
					// the own-ship is passed
					if (ps_maneuver_times_i(ps) * pars.dt < t_obst_passed - pars.t_ts)
					{
						ps_weights_i(ps) = Pr_CC_i;
					}							
				} 
			}
			break;
		default :
			std::cout << "This situation type does not exist" << std::endl;
			break;
	}
	ps_weights_i = ps_weights_i / ps_weights_i.sum();
}

/****************************************************************************************
*  Name     : set_up_independent_obstacle_prediction_variables
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_dependent_obstacle_prediction_variables(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_weights_i, 											// In/out: Cost weights of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const Obstacle_Data &odata,												// In: Dynamic obstacle information
	const int i 															// In: Index of obstacle in consideration
	)
{
	double Pr_CC_i;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC;
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard and port maneuvers, respectively
		if (ps < (n_ps[i] - 1) / 2 + 1)		{ ps_ordering_i[ps] = SM; }
		else 								{ ps_ordering_i[ps] = PM; }	
	}
	ps_maneuver_times_i.resize(0);
	ps_course_changes_i.resize(0);

	ps_weights_i.resize(n_ps[i]);
	Pr_CC_i = odata.obstacles[i].get_a_priori_CC_probability();
	switch(odata.ST_i_0[i])
	{
		case A : // Outside CC consideration zone
			ps_weights_i(0) = 1;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1;	
			}
			break;
		case B : // OT, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case C : // CR, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case D : // OT, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = Pr_CC_i;
			}
			break;
		case E : // HO, GW
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
				// Starboard maneuvers, which are CC
				if (ps > 0 && ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;	
				}
			}
			break;
		case F : // CR, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;						
				} 
			}
			break;
		default :
			std::cout << "This situation type does not exist" << std::endl;
			break;
	}
	ps_weights_i = ps_weights_i / ps_weights_i.sum();
}

/****************************************************************************************
*  Name     : find_time_of_passing
*  Function : Finds the time when an obstacle is passed by the own-ship, assuming both 
*			  vessels keeps their current course
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double PSBMPC::find_time_of_passing(
	const Obstacle_Data &odata,											// In: Dynamic obstacle information
	const int i 														// In: Index of relevant obstacle
	)
{
	double t_obst_passed(1e12), t, psi_A, d_AB;
	Eigen::VectorXd xs_A = trajectory.col(0);
	Eigen::VectorXd xs_B = odata.obstacles[i].kf.get_state();
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
*  Function : Uses the dynamic obstacle vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_colav_active(
	const Obstacle_Data &odata,												// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < odata.obstacles.size(); i++)
	{
		d_0i(0) = odata.obstacles[i].kf.get_state()(0) - xs(0);
		d_0i(1) = odata.obstacles[i].kf.get_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;

		// If all obstacles are passed, even though inside colav range,
		// then no need for colav
		if (odata.IP_0[i]) 	{ colav_active = false; }
		else 				{ colav_active = true; }
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
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
	if (pars.prediction_method > Linear)
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

/****************************************************************************************
*  Name     : set_up_cuda_memory_transfer
*  Function : Allocates device memory for data that is required on the GPU for the
*			  PSB-MPC calculations. 
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_temporary_device_memory(
	const double u_d,												// In: Own-ship surge reference
	const double chi_d, 											// In: Own-ship course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,					// In: Own-ship waypoints to follow
	const Eigen::Matrix<double, 4, -1> &static_obstacles,			// In: Static obstacle information
	const Obstacle_Data &odata 										// In: Dynamic obstacle information
	)
{
	int n_obst = odata.obstacles.size();

	std::cout << "CB_Functor_Pars size: " << sizeof(CB_Functor_Pars) << std::endl;
	std::cout << "CB_Functor_Data size: " << sizeof(CB_Functor_Data) << std::endl;
	std::cout << "Ownship size: " << sizeof(Ownship) << std::endl;
	std::cout << "CPE size: " << sizeof(CPE) << std::endl;
	
	size_t limit = 0;

	cudaDeviceSetLimit(cudaLimitStackSize, 100000);
	cuda_check_errors("Setting cudaLimitStackSize failed.");

	cudaDeviceGetLimit(&limit, cudaLimitStackSize);
	cuda_check_errors("Reading cudaLimitStackSize failed.");
	std::cout << "Set device max stack size : " << limit << std::endl;

	/* cudaDeviceGetLimit(&limit, cudaLimitMallocHeapSize);
	cuda_check_errors("Reading cudaLimitMallocHeapSize failed.");
	std::cout << "Device max heap size : " << limit << std::endl; */

	// Allocation of device memory for control behaviour functor data and obstacles
	// CB Functor Data
	cudaMalloc((void**)&fdata_device_ptr, sizeof(CB_Functor_Data));
    cuda_check_errors("Malloc of CB_Functor_Data failed.");

	CB_Functor_Data temporary_fdata(*this, u_d, chi_d, waypoints, static_obstacles, odata);

	cudaMemcpy(fdata_device_ptr, &temporary_fdata, sizeof(CB_Functor_Data), cudaMemcpyHostToDevice);
    cuda_check_errors("MemCpy of CB_Functor_Data failed.");
	
	// Cuda Obstacles
	cudaMalloc((void**)&obstacles_device_ptr, n_obst * sizeof(Cuda_Obstacle));
    cuda_check_errors("Malloc of Cuda_Obstacle's failed.");

	Cuda_Obstacle temporary_transfer_obstacle;
	for (int i = 0; i < n_obst; i++)
	{
		temporary_transfer_obstacle = odata.obstacles[i];

		cudaMemcpy(&obstacles_device_ptr[i], &temporary_transfer_obstacle, sizeof(Cuda_Obstacle), cudaMemcpyHostToDevice);
    	cuda_check_errors("MemCpy of Cuda_Obstacle i failed.");
	}
}

/****************************************************************************************
*  Name     : clear_temporary_device_memory
*  Function : Clears/frees CB Functor and obstacle device memory used in one iteration 
*			  in the PSB-MPC. 
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::clear_temporary_device_memory()
{
	cudaFree(fdata_device_ptr); 
	cuda_check_errors("cudaFree of fdata_device_ptr failed.");
    
	cudaFree(obstacles_device_ptr);
    cuda_check_errors("cudaFree of obstacles_device_ptr failed.");
}