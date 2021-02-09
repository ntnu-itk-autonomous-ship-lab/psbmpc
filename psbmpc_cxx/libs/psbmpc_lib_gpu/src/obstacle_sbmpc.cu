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
__host__ __device__ Obstacle_SBMPC::Obstacle_SBMPC() :
	pars(SBMPC_Parameters(true))
{
	offset_sequence_counter.resize(2 * pars.n_M);
	offset_sequence.resize(2 * pars.n_M);
	maneuver_times.resize(pars.n_M);

	mpc_cost = MPC_Cost<SBMPC_Parameters>(pars);

	u_m_last = 1.0; chi_m_last = 0.0;

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
	) :
	pars(SBMPC_Parameters(true))
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
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 4, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::Vector4d &ownship_state, 									// In: Current ship state
	const Eigen::Matrix<double, 4, -1> &static_obstacles,					// In: Static obstacle information
	Obstacle_Data<Prediction_Obstacle> &data,								// In/Out: Dynamic obstacle information
	const int k_0 															// In: Index of the current (joint prediction) time t_k0
	)
{
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(4, n_samples);
	trajectory.col(0) = ownship_state;

	int n_obst = data.obstacles.size();
	int n_static_obst = static_obstacles.cols();

	Eigen::VectorXd opt_offset_sequence(2 * pars.n_M), cost_i(n_obst);
	data.HL_0.resize(n_obst); data.HL_0.setZero();

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		
		for (int M = 0; M < pars.n_M; M++)
		{
			opt_offset_sequence(2 * M) = 1.0; opt_offset_sequence(2 * M + 1) = 0.0;
		}
		maneuver_times.setZero();
		ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
		
		assign_optimal_trajectory(predicted_trajectory);

		return;
	}

	initialize_prediction(data, k_0);

	double cost(0.0);
	min_cost = 1e12;
	reset_control_behavior();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0.0;

		//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
		ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

		for (int i = 0; i < n_obst; i++)
		{
			cost_i(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, offset_sequence, maneuver_times, data, i, ownship.get_length());
		}

		cost += cost_i.maxCoeff();
		//std::cout << "cost = " << cost << std::endl;
		//cost += mpc_cost.calculate_grounding_cost(trajectory, static_obstacles, ownship.get_length());

		cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_m_last, chi_m_last);

		//cost += mpc_cost.calculate_chattering_cost(offset_sequence, maneuver_times);
		//std::cout << "cost = " << cost << std::endl;

		if (cost < min_cost) 
		{
			min_cost = cost;
			opt_offset_sequence = offset_sequence;

			assign_optimal_trajectory(predicted_trajectory);

			// Assign current optimal hazard level for each obstacle
			for (int i = 0; i < n_obst; i++)
			{
				data.HL_0(i) = cost_i(i) / cost_i.sum();
			}	
		}
		increment_control_behavior();
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
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl; */
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

	this->u_m_last = o_sbmpc.u_m_last;
	this->chi_m_last = o_sbmpc.chi_m_last;

	this->min_cost = o_sbmpc.min_cost;

	this->ownship = o_sbmpc.ownship;

	this->trajectory = o_sbmpc.trajectory;

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
	Obstacle_Data<Prediction_Obstacle> &data,							// In: Dynamic obstacle information
	const int k_0														// In: Index of the current (joint prediction) time t_k0	 
	)
{
	int n_obst = data.obstacles.size();

	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	for (int i = 0; i < n_obst; i++)
	{
		data.obstacles[i].predict_independent_trajectory(pars.T, pars.dt, k_0);
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(pars.n_M);
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
*  Name     : determine_colav_active
*  Function : Uses the freshly updated new_obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ bool Obstacle_SBMPC::determine_colav_active(
	const Obstacle_Data<Prediction_Obstacle> &data,							// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Vector4d xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].get_initial_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].get_initial_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;
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
	Eigen::Matrix<double, 4, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = round(pars.T / pars.dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (false) //pars.prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(4, n_samples / pars.p_step);
		for (int k = 0; k < n_samples; k += pars.p_step)
		{
			optimal_trajectory.col(count) = trajectory.col(k);
			if (count < round(n_samples / pars.p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(4, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 4, n_samples);
	}
}