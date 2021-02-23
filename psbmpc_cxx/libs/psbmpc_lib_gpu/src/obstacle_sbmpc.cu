/****************************************************************************************
*
*  File name : obstacle_sbmpc.cu
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

#include <thrust/device_vector.h>
#include "utilities.cuh"
#include "obstacle_sbmpc.cuh"
#include <iostream>

/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle_SBMPC::Obstacle_SBMPC()
{
	offset_sequence_counter.resize(2 * pars.n_M, 1);
	offset_sequence.resize(2 * pars.n_M, 1);
	maneuver_times.resize(pars.n_M, 1);

	mpc_cost = MPC_Cost<Obstacle_SBMPC_Parameters>(pars);

	min_cost = 1e12;
}

__host__ __device__ Obstacle_SBMPC::~Obstacle_SBMPC() = default;

/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle_SBMPC::Obstacle_SBMPC(
	const Obstacle_SBMPC &o_sbmpc 									// In: Obstacle SBMPC to copy
	)
{
	assign_data(o_sbmpc);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle_SBMPC& Obstacle_SBMPC::operator=(
	const Obstacle_SBMPC &rhs 									// In: Rhs Obstacle SBMPC to assign
	)
{
	if (this == &rhs)
	{
		return *this;
	}

	assign_data(rhs);

	return *this = Obstacle_SBMPC(rhs);
}

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_SBMPC::calculate_optimal_offsets(									
	float &u_opt, 															// In/out: Optimal surge offset
	float &chi_opt, 														// In/out: Optimal course offset
	const float u_opt_last,													// In: Previous optimal surge offset
	const float chi_opt_last,												// In: Previous optimal course offset
	const float u_d, 														// In: Surge reference
	const float chi_d, 														// In: Course reference
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,					// In: Next waypoints
	const TML::Vector4f &ownship_state, 									// In: Current ship state
	const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,			// In: Static obstacle information
	const Obstacle_Data_GPU_Friendly &data,									// In: Dynamic obstacle information
	Prediction_Obstacle *obstacles, 										// In: Pointer to list of all prediction obstacles, including the ownship
	const int i_caller,														// In: Index of calling obstacle
	const int k_0 															// In: Index of the current (joint prediction) time t_k0
	)
{
	n_samples = std::round(pars.T / pars.dt);

	n_obst = data.AH_0.size(); // n_obst here includes obstacle i (the calling object)
	n_static_obst = static_obstacles.get_cols();

	cost_i.resize(n_obst, 1);

	if (!determine_colav_active(ownship_state, obstacles, i_caller, n_static_obst, k_0))
	{
		u_opt = 1.0f;
		chi_opt = 0.0f;
		return;
	}

	initialize_prediction(obstacles, i_caller, k_0);

	min_cost = 1e12;
	reset_control_behavior();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0.0f; cost_i.set_zero();

		//===========================================================================
		// Predict and calculate dynamic obstacle cost (and (TBD) grounding cost ) 
		// jointly to save memory
		//===========================================================================
		ownship.initialize_wp_following();

		man_count = 0;
		u_m = 1; u_d_p = u_d;
		chi_m = 0; chi_d_p = chi_d;

		xs_k_p = ownship_state;
		for (int k = 0; k < n_samples; k++)
		{ 	
			// Determine active control behaviour
			if (k == maneuver_times[man_count]){
				u_m = offset_sequence[2 * man_count];
				chi_m = offset_sequence[2 * man_count + 1]; 
				if (man_count < (int)maneuver_times.size() - 1) man_count += 1;
			}  

			// Calculate dynamic obstacle cost and grounding cost
			i_count = 0;
			for (int i = 0; i < n_obst + 1; i++)
			{
				if (i != i_caller)
				{
					xs_i_k_p = obstacles[i].get_predicted_trajectory_sample(k);
					//printf("xs_i_k_p = %.2f, %.2f, %.2f, %.2f\n", xs_i_k_p(0), xs_i_k_p(1), xs_i_k_p(2), xs_i_k_p(3));

					cost_i_k = mpc_cost.calculate_dynamic_obstacle_cost(xs_k_p, xs_i_k_p, k, data, i_count, obstacles[i].get_length(), ownship.get_length(), chi_m);

					if (cost_i(i_count) < cost_i_k)	{ cost_i(i_count) = cost_i_k; }

					i_count += 1;
				}
			}

			cost_g_k = mpc_cost.calculate_grounding_cost(xs_k_p, static_obstacles, ownship.get_length());
			if (cost_g < cost_g_k) { cost_g = cost_g_k; }

			// Predict ownship one step ahead
			ownship.update_guidance_references(u_d_p, chi_d_p, waypoints, xs_k_p, pars.dt, pars.guidance_method);

			xs_k_p = ownship.predict(xs_k_p, u_m * u_d_p , chi_d_p + chi_m, pars.dt, pars.prediction_method);
			
		} 
		cost += cost_i.max_coeff();
		cost += cost_g;

		//===========================================================================
		cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);

		cost += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);

		if (cost < min_cost) 
		{
			min_cost = cost;
			u_opt = offset_sequence(0);
			chi_opt = offset_sequence(1);

		}
		increment_control_behavior();
	}
}

__host__ __device__ void Obstacle_SBMPC::calculate_optimal_offsets(									
	float &u_opt, 															// In/out: Optimal surge offset
	float &chi_opt, 														// In/out: Optimal course offset
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> &predicted_trajectory,			// In/out: Optimal predicted trajectory
	const float u_opt_last,													// In: Previous optimal surge offset
	const float chi_opt_last,												// In: Previous optimal course offset
	const float u_d, 														// In: Surge reference
	const float chi_d, 														// In: Course reference
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,					// In: Next waypoints
	const TML::Vector4f &ownship_state, 									// In: Current ship state
	const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,			// In: Static obstacle information
	const Obstacle_Data_GPU_Friendly &data,									// In: Dynamic obstacle information
	Prediction_Obstacle *obstacles, 										// In: Pointer to list of all prediction obstacles, including the ownship
	const int i_caller,														// In: Index of calling obstacle
	const int k_0 															// In: Index of the current (joint prediction) time t_k0
	)
{
	n_samples = std::round(pars.T / pars.dt); 

	n_obst = data.AH_0.size();
	n_static_obst = static_obstacles.get_cols();

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	cost_i.resize(n_obst, 1);

	if (!determine_colav_active(ownship_state, obstacles, i_caller, n_static_obst, k_0))
	{
		u_opt = 1.0f;
		chi_opt = 0.0f;
		return;
	}

	initialize_prediction(obstacles, i_caller, k_0);

	min_cost = 1e12;
	reset_control_behavior();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0.0f; cost_i.set_zero(); cost_g = 0.0f;

		//printf("offset_sequence = %.2f, %.2f\n", offset_sequence(0), offset_sequence(1));

		//===========================================================================
		// Predict and calculate dynamic obstacle cost (and (TBD) grounding cost ) 
		// jointly to save memory
		//===========================================================================
		ownship.initialize_wp_following();

		man_count = 0;
		u_m = 1; u_d_p = u_d;
		chi_m = 0; chi_d_p = chi_d;

		xs_k_p = ownship_state;
		for (int k = 0; k < n_samples; k++)
		{ 	
			// Determine active control behaviour
			if (k == maneuver_times[man_count]){
				u_m = offset_sequence[2 * man_count];
				chi_m = offset_sequence[2 * man_count + 1]; 
				if (man_count < (int)maneuver_times.size() - 1) man_count += 1;
			}  

			// Calculate dynamic obstacle cost and grounding cost
			i_count = 0;
			for (int i = 0; i < n_obst + 1; i++)
			{
				if (i != i_caller)
				{
					xs_i_k_p = obstacles[i].get_predicted_trajectory_sample(k);
					//printf("xs_i_k_p = %.2f, %.2f, %.2f, %.2f\n", xs_i_k_p(0), xs_i_k_p(1), xs_i_k_p(2), xs_i_k_p(3));

					cost_i_k = mpc_cost.calculate_dynamic_obstacle_cost(xs_k_p, xs_i_k_p, k, data, i_count, obstacles[i].get_length(), ownship.get_length(), chi_m);

					if (cost_i(i_count) < cost_i_k)	{ cost_i(i_count) = cost_i_k; }

					i_count += 1;
				}
			}

			cost_g_k = mpc_cost.calculate_grounding_cost(xs_k_p, static_obstacles, ownship.get_length());
			if (cost_g < cost_g_k) { cost_g = cost_g_k; }

			// Predict ownship one step ahead
			ownship.update_guidance_references(u_d_p, chi_d_p, waypoints, xs_k_p, pars.dt, pars.guidance_method);

			xs_k_p = ownship.predict(xs_k_p, u_m * u_d_p , chi_d_p + chi_m, pars.dt, pars.prediction_method);
			
		} 
		cost += cost_i.max_coeff();
		cost += cost_g;

		//===========================================================================
		cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);

		cost += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);

		if (cost < min_cost) 
		{
			min_cost = cost;
			u_opt = offset_sequence(0);
			chi_opt = offset_sequence(1);

			assign_optimal_trajectory(predicted_trajectory, offset_sequence, waypoints, ownship_state, u_d, chi_d);
		}
		increment_control_behavior();
	}
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : assign_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_SBMPC::assign_data(
	const Obstacle_SBMPC &o_sbmpc 							// In: Obstacle_SBMPC whose data to assign to this
	)
{
	this->offset_sequence_counter = o_sbmpc.offset_sequence_counter;
	this->offset_sequence = o_sbmpc.offset_sequence;
	this->maneuver_times = o_sbmpc.maneuver_times;

	this->min_cost = o_sbmpc.min_cost;

	this->ownship = o_sbmpc.ownship;

	this->pars = o_sbmpc.pars;

	this->mpc_cost = o_sbmpc.mpc_cost;
}

/****************************************************************************************
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_SBMPC::initialize_prediction(
	Prediction_Obstacle *obstacles, 									// In/out: Pointer to list of all prediction obstacles
	const int i_caller, 												// In: Index of calling obstacle
	const int k_0														// In: Index of the current (joint prediction) time t_k0	 
	)
{
	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	for (int i = 0; i < n_obst + 1; i++)
	{
		if (i != i_caller)
		{
			obstacles[i].predict_independent_trajectory(pars.T, pars.dt, k_0);
		}
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(pars.n_M, 1);
	// First avoidance maneuver is always at t0, and n_M = 1 for Obstacle SBMPC
	maneuver_times(0) = 0.0;
	
	//std::cout << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branches of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_SBMPC::reset_control_behavior()
{
	offset_sequence_counter.set_zero();
	for (int M = 0; M < pars.n_M; M++)
	{
		offset_sequence(2 * M) = pars.u_offsets(M, 0);
		offset_sequence(2 * M + 1) = pars.chi_offsets(M, 0);
	}
}

/****************************************************************************************
*  Name     : increment_control_behavior
*  Function : Increments the control behavior counter and changes the offset sequence 
*			  accordingly. Backpropagation is used for the incrementation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_SBMPC::increment_control_behavior()
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
		if (offset_sequence_counter(2 * M + 1) == pars.offsets_size(1, M))
		{
			offset_sequence_counter(2 * M + 1) = 0;
			offset_sequence_counter(2 * M) += 1;
		}
		offset_sequence(2 * M + 1) = pars.chi_offsets(M, offset_sequence_counter(2 * M + 1));

		// If one reaches the end of maneuver M's surge offsets, reset corresponding
		// counter and increment course offset counter above (if any)
		if (offset_sequence_counter(2 * M) == pars.offsets_size(0, M))
		{
			offset_sequence_counter(2 * M) = 0;
			if (M > 0)
			{
				offset_sequence_counter(2 * M - 1) += 1;
			}
		}
		offset_sequence(2 * M) = pars.u_offsets(M, offset_sequence_counter(2 * M));
	}
}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the freshly updated new_obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ bool Obstacle_SBMPC::determine_colav_active(
	const TML::Vector4f &xs_k_0,										// In: State of the ownship at the current time t_k0
	Prediction_Obstacle *obstacles, 									// In: Pointer to list of all prediction obstacles
	const int i_caller, 												// In: Index of calling obstacle
	const int n_static_obst, 											// In: Number of static obstacles
	const int k_0														// In: Index of the current (joint prediction) time t_k0
	)
{
	colav_active = false;
	for (int i = 0; i < n_obst + 1; i++)
	{
		if (i != i_caller)
		{
			xs_i = obstacles[i].get_trajectory_sample(k_0);
			d_0i(0) = xs_i(0) - xs_k_0(0);
			d_0i(1) = xs_i(1) - xs_k_0(1);
			if (d_0i.norm() < pars.d_init) colav_active = true;
		}
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
__host__ __device__ void Obstacle_SBMPC::assign_optimal_trajectory(
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> &optimal_trajectory, 					// In/out: Optimal PSB-MPC trajectory
	TML::PDMatrix<float, 2 * MAX_N_M, 1> &opt_offset_sequence,						// In: Optimal control behaviour to follow
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,							// In: Next waypoints
	const TML::Vector4f &ownship_state,												// In: Current ship state
	const float u_d,																// In: Surge reference
	const float chi_d	 															// In: course reference
	)
{
	n_samples = round(pars.T / pars.dt);
	optimal_trajectory.resize(4, n_samples);

	ownship.initialize_wp_following();

	man_count = 0;
	u_m = 1; u_d_p = u_d;
	chi_m = 0; chi_d_p = chi_d;

	xs_k_p = ownship_state;
	optimal_trajectory.set_col(0, xs_k_p);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = opt_offset_sequence[2 * man_count];
			chi_m = opt_offset_sequence[2 * man_count + 1]; 
			if (man_count < (int)maneuver_times.size() - 1) man_count += 1;
		}  

		ownship.update_guidance_references(u_d_p, chi_d_p, waypoints, xs_k_p, pars.dt, pars.guidance_method);

		xs_k_p = ownship.predict(xs_k_p, u_m * u_d_p , chi_d_p + chi_m, pars.dt, pars.prediction_method);
		
		if (k < n_samples - 1) { optimal_trajectory.set_col(k + 1, xs_k_p); }

		/* printf("opt_traj.col(k) = %.2f, %.2f, %.2f, %.2f\n", 
			optimal_trajectory.get_col(k)(0), 
			optimal_trajectory.get_col(k)(1), 
			optimal_trajectory.get_col(k)(2), 
			optimal_trajectory.get_col(k)(3)); */
	} 
	
}