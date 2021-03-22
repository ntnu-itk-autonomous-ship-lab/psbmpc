/****************************************************************************************
*
*  File name : obstacle_sbmpc_gpu.cuh
*
*  Function  : Header file for Scenario-based Model Predictive Control used by obstacles
*			   in the GPU PSB-MPC predictions. 
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

#pragma once

#include "obstacle_sbmpc_parameters_gpu.cuh"
#include "kinematic_ship_models_gpu.cuh"
#include "mpc_cost_gpu.cuh"

#include <thrust/device_vector.h>
#include <vector>

namespace PSBMPC_LIB
{
	namespace GPU
	{	
		class Prediction_Obstacle;
		
		class Obstacle_SBMPC
		{
		private:

			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

			TML::PDMatrix<int, 2 * MAX_N_M, 1> offset_sequence_counter;

			TML::PDMatrix<float, MAX_N_M, 1> maneuver_times;

			float min_cost;

			Obstacle_Ship ownship;

			//==============================================
			// Pre-allocated temporaries
			//==============================================
			TML::Vector4f xs_k_p, xs_i_k_p;

			TML::PDMatrix<float, MAX_N_OBST, 1> cost_i;
			int n_samples, n_obst, n_static_obst, count, man_count, i_count;

			float u_m, u_d_p;
			float chi_m, chi_d_p;

			bool colav_active;

			float cost, cost_g, cost_g_k, cost_i_k;

			TML::Vector4f xs, xs_i;
			TML::Vector2f d_0i;
			//==============================================

			__host__ __device__ void assign_data(const Obstacle_SBMPC &o_sbmpc);

			__host__ __device__ void initialize_prediction(Prediction_Obstacle *pobstacles, const int i_caller, const int k_0);

			__host__ __device__ void reset_control_behavior();

			__host__ __device__ void increment_control_behavior();

			__host__ __device__ bool determine_colav_active(
				const TML::Vector4f &xs_k_0,
				Prediction_Obstacle *pobstacles,
				const int i_caller,
				const int n_static_obst,
				const int k_0);
			//

			__host__ __device__ void assign_optimal_trajectory(
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> &optimal_trajectory, 
				TML::PDMatrix<float, 2 * MAX_N_M, 1> &opt_offset_sequence,
				const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
				const TML::Vector4f &ownship_state, 
				const float u_d, 
				const float chi_d);

		public:

			Obstacle_SBMPC_Parameters pars;

			MPC_Cost<Obstacle_SBMPC_Parameters> mpc_cost;

			__host__ __device__ Obstacle_SBMPC();

			__host__ __device__ ~Obstacle_SBMPC();

			__host__ __device__ Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

			__host__ __device__ Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

			__host__ __device__ void calculate_optimal_offsets(
				float &u_opt, 	
				float &chi_opt, 
				const float u_opt_last,
				const float chi_opt_last,
				const float u_d, 
				const float chi_d, 
				const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
				const TML::Vector4f &ownship_state,
				const TML::PDMatrix<float, 4, MAX_N_OBST> &static_obstacles,
				const Obstacle_Data_GPU_Friendly &data,
				Prediction_Obstacle *pobstacles,
				const int i_caller,
				const int k_0);

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
				const Obstacle_Data_GPU_Friendly &data,
				Prediction_Obstacle *pobstacles,
				const int i_caller,
				const int k_0);
			
		};
	}
}