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
#include "tml.cuh"
#include <vector>


class Obstacle_SBMPC
{
private:

	TML::PDMatrix<float, 1, 2 * MAX_N_M> offset_sequence;

	TML::PDMatrix<int, 1, MAX_N_M> offset_sequence_counter;

	TML::PDMatrix<float, 1, MAX_N_M> maneuver_times;
	
	float u_m_last;
	float chi_m_last;

	float min_cost;

	Obstacle_Ship ownship;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> trajectory;

	__host__ __device__ void assign_data(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void initialize_prediction(Obstacle_Data<Prediction_Obstacle> &data, const int k_0);

	__host__ __device__ void reset_control_behavior();

	__host__ __device__ void increment_control_behavior();

	__host__ __device__ bool determine_colav_active(const Obstacle_Data<Prediction_Obstacle> &data, const int n_static_obst);

	__host__ __device__ void assign_optimal_trajectory(Eigen::Matrix<double, 4, -1> &optimal_trajectory);

public:

	SBMPC_Parameters pars;

	MPC_Cost<SBMPC_Parameters> mpc_cost;

	__host__ __device__ Obstacle_SBMPC();

	__host__ __device__ ~Obstacle_SBMPC();

	__host__ __device__ Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void calculate_optimal_offsets(
		double &u_opt, 	
		double &chi_opt, 
		Eigen::Matrix<double, 4, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Vector4d &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data<Prediction_Obstacle> &data,
		const int k_0);
};

#endif 