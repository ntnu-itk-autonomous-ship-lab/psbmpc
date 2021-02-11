/****************************************************************************************
*
*  File name : obstacle_sbmpc.cuh
*
*  Function  : Header file for Scenario-based Model Predictive Control used by obstacles
*			   in the PSB-MPC predictions.
*
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

#ifndef _OBSTACLE_SBMPC_CUH_
#define _OBSTACLE_SBMPC_CUH_

#include <thrust/device_vector.h>
#include "psbmpc_index.h"
#include "sbmpc_parameters.h"
#include "joint_prediction_manager.cuh"
#include "obstacle_ship.cuh"
#include "mpc_cost.cuh"
#include "tml.cuh"
#include <vector>


class Obstacle_SBMPC
{
private:

	TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

	TML::PDMatrix<int, 2 * MAX_N_M, 1> offset_sequence_counter;

	TML::PDMatrix<float, MAX_N_M, 1> maneuver_times;
	
	float u_m_last;
	float chi_m_last;

	float min_cost;

	Obstacle_Ship ownship;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> trajectory;

	//==============================================
	// Pre-allocated temporaries
	//==============================================
	TML::PDMatrix<float, 2 * MAX_N_M, 1> opt_offset_sequence;
	TML::PDMatrix<float, MAX_N_OBST, 1> cost_i;
	int n_samples, n_obst, n_static_obst, count;

	bool colav_active;

	float cost;

	TML::Vector4f xs;
	TML::Vector2f d_0i;
	//==============================================

	__host__ __device__ void assign_data(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void initialize_prediction(Obstacle_Data<Prediction_Obstacle> &data, const int k_0);

	__host__ __device__ void reset_control_behavior();

	__host__ __device__ void increment_control_behavior();

	__host__ __device__ bool determine_colav_active(const Obstacle_Data<Prediction_Obstacle> &data, const int n_static_obst);

	__host__ __device__ void assign_optimal_trajectory(TML::PDMatrix<float, 4, MAX_N_SAMPLES> &optimal_trajectory);

public:

	SBMPC_Parameters pars;

	MPC_Cost<SBMPC_Parameters> mpc_cost;

	__host__ __device__ Obstacle_SBMPC();

	__host__ __device__ ~Obstacle_SBMPC();

	__host__ __device__ Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void calculate_optimal_offsets(
		float &u_opt, 	
		float &chi_opt, 
		TML::PDMatrix<float, 4, MAX_N_SAMPLES> &predicted_trajectory,
		const float u_d, 
		const float chi_d, 
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
		const TML::Vector4f &ownship_state,
		const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,
		Obstacle_Data<Prediction_Obstacle> &data,
		const int k_0);
};

#endif 