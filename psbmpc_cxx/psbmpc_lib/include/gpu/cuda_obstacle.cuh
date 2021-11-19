/****************************************************************************************
*
*  File name : cuda_obstacle.cuh
*
*  Function  : Header file for the read-only obstacle class containing independent
*			   prediction scenario trajectory data, used by the PSB-MPC in the
*			   CUDA kernels / GPU threads.
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#include "psbmpc_defines.hpp"
#include "tracked_obstacle.hpp"
#include "tml/tml.cuh"

#include <Eigen/Dense>
#include <thrust/device_vector.h>


namespace PSBMPC_LIB
{	
	namespace GPU
	{
		class Cuda_Obstacle
		{
		private:

			int ID;

			// Obstacle dimension quantifiers, length (l) and width (w)
			float A, B, C, D, l, w;

			float x_offset, y_offset;

			// State and covariance at the current time or predicted time
			TML::Vector4f xs_0;
			TML::Matrix4f P_0;

			// If the tracker-based KF is on, the obstacle is tracked until it dies
			// while the duration lost may be reset if new measurements are aquired
			float duration_tracked, duration_lost;

			// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
			// This is equal for all prediction scenarios including those with active COLAV (using MROU)
			TML::PDMatrix<float, 16, MAX_N_SAMPLES> P_p;  

			// Predicted state for each prediction scenario: size n_ps x n x n_samples, where n = 4
			TML::PDMatrix<float, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_p;

			__host__ void assign_data(const Cuda_Obstacle &co);
			
			__host__ void assign_data(const Tracked_Obstacle &to);
			
		public:

			// Cuda Obstacle`s are only copy constructed
			__host__ Cuda_Obstacle() {};

			__host__ Cuda_Obstacle(const Cuda_Obstacle &co);

			__host__ Cuda_Obstacle(const Tracked_Obstacle &to);

			__host__ Cuda_Obstacle& operator=(const Cuda_Obstacle &rhs);

			__host__ Cuda_Obstacle& operator=(const Tracked_Obstacle &rhs);

			__host__ __device__ inline int get_ID() const { return ID; };

			__host__ __device__ inline float get_length() const { return l; };

			__host__ __device__ inline float get_width() const { return w; };

			__device__ inline float get_duration_lost() const { return duration_lost; }

			__device__ inline float get_duration_tracked() const { return duration_tracked; }	

			__device__ inline TML::PDVector16f get_trajectory_covariance_sample(const int k) { return P_p.get_col(k); }

			__device__ inline TML::PDVector4f get_trajectory_sample(const int ps, const int k) { return xs_p.get_block<4, 1>(4 * ps, k, 4, 1); }
		};
	}
}