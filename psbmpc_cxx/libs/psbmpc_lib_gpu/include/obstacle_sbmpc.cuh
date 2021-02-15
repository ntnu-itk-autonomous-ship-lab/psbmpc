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

class Obstacle_Data_GPU
{
public:

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	TML::PDMatrix<bool, MAX_N_OBST, 1> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

	// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
	TML::PDMatrix<ST, MAX_N_OBST, 1> ST_0, ST_i_0;

	__host__ __device__ Obstacle_Data_GPU() {}

	__host__ __device__ ~Obstacle_Data_GPU() {}
};

class Obstacle_SBMPC
{
private:

	TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

	TML::PDMatrix<int, 2 * MAX_N_M, 1> offset_sequence_counter;

	TML::PDMatrix<float, MAX_N_M, 1> maneuver_times;

	float min_cost;

	Obstacle_Ship ownship;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> trajectory;

	//==============================================
	// Pre-allocated temporaries
	//==============================================
	TML::PDMatrix<float, MAX_N_OBST, 1> cost_i;
	int n_samples, n_obst, n_static_obst, count;

	bool colav_active;

	float cost;

	TML::Vector4f xs, xs_i;
	TML::Vector2f d_0i;
	//==============================================

	__host__ __device__ void assign_data(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void initialize_prediction(Obstacle_Data<Prediction_Obstacle> &data, const int k_0);
	__device__ void initialize_prediction(
		Prediction_Obstacle *pobstacles,
		const int i_caller, 
		const int k_0);

	__host__ __device__ void reset_control_behavior();

	__host__ __device__ void increment_control_behavior();

	__host__ __device__ bool determine_colav_active(const Obstacle_Data<Prediction_Obstacle> &data, const int n_static_obst, const int k_0);
	__device__ bool determine_colav_active(
		Prediction_Obstacle *pobstacles,
		const int i_caller,
		const int n_static_obst,
		const int k_0);

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
		const float u_opt_last,
		const float chi_opt_last,
		const float u_d, 
		const float chi_d, 
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
		const TML::Vector4f &ownship_state,
		const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,
		Obstacle_Data<Prediction_Obstacle> &data,
		const int k_0);

	__device__ void calculate_optimal_offsets(
		float &u_opt, 	
		float &chi_opt, 
		const float u_opt_last,
		const float chi_opt_last,
		const float u_d, 
		const float chi_d, 
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
		const TML::Vector4f &ownship_state,
		const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,
		const Obstacle_Data_GPU &data,
		Prediction_Obstacle *pobstacles,
		const int i_caller,
		const int k_0);
};

#endif 