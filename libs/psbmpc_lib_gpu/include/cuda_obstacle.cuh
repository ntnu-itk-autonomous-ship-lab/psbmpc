/****************************************************************************************
*
*  File name : cuda_obstacle.cuh
*
*  Function  : Header file for the obstacle class used by the PSB-MPC in the
*			   CUDA kernels / GPU threads.
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


#ifndef _CUDA_OBSTACLE_CUH_
#define _CUDA_OBSTACLE_CUH_

#include "psbmpc_defines.h"
#include <thrust/device_vector.h>
#include "tml.cuh"
#include "obstacle.cuh"
#include "tracked_obstacle.cuh"

//class Obstacle_SBMPC;

class Cuda_Obstacle : public Obstacle
{
private:

	// State and covariance at the current time or predicted time
	TML::Vector4f xs_0;
	TML::Matrix4f P_0;

	int n_ps;

	// Vector of intention probabilities at the current time or last time of update
	TML::PDVector3f Pr_a;

	// A priori COLREGS compliance probability at the current time or last time of update
	float Pr_CC;

	// If the tracker-based KF is on, the obstacle is tracked until it dies
	// while the duration lost may be reset if new measurements are aquired
	float duration_tracked, duration_lost;

	// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
	TML::PDMatrix<bool, MAX_N_PS, 1> mu;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
	// This is equal for all prediction scenarios including those with active COLAV (using MROU). Ad hoc max nr of samples: 2000
	TML::PDMatrix<float, 16, MAX_N_SAMPLES> P_p;  

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	TML::PDMatrix<float, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_p;

	// Weights for the independent prediction scenarios: n_ps x 1
	TML::PDMatrix<float, MAX_N_PS, 1> ps_weights;

	__host__ void assign_data(const Cuda_Obstacle &co);
	
	__host__ void assign_data(const Tracked_Obstacle &to);
	
public:

	__host__ Cuda_Obstacle() {};

	__host__ Cuda_Obstacle(const Cuda_Obstacle &co);

	__host__ Cuda_Obstacle(const Tracked_Obstacle &to);

	__host__ Cuda_Obstacle& operator=(const Cuda_Obstacle &rhs);

	__host__ Cuda_Obstacle& operator=(const Tracked_Obstacle &rhs);

	__device__ inline TML::PDVector3f get_intention_probabilities() const { return Pr_a; }

	__device__ inline float get_a_priori_CC_probability() const { return Pr_CC; }

	__device__ inline float get_duration_lost() const { return duration_lost; }

	__device__ inline float get_duration_tracked() const { return duration_tracked; }

	__device__ inline TML::PDMatrix<bool, MAX_N_PS, 1> get_COLREGS_violation_indicator() const { return mu; }	

	__device__ inline TML::PDMatrix<float, 16, MAX_N_SAMPLES> get_trajectory_covariance() const { return P_p; }

	__device__ inline TML::PDMatrix<float, 16, 1> get_trajectory_covariance_sample(const int k) { return P_p.get_col(k); }

	__device__ inline TML::PDMatrix<float, 4 * MAX_N_PS, MAX_N_SAMPLES> get_trajectories() const { return xs_p; }

	__device__ inline TML::PDMatrix<float, 4, MAX_N_SAMPLES> get_ps_trajectory(const int ps) const
	{ 
		return xs_p.get_block<4, MAX_N_SAMPLES>(4 * ps, 0, 4, xs_p.get_cols()); 
	}

	__device__ inline TML::PDVector4f get_trajectory_sample(const int ps, const int k) { return xs_p.get_block<4, 1>(4 * ps, k, 4, 1); }

	__device__ inline TML::PDMatrix<float, MAX_N_PS, 1> get_ps_weights() const { return ps_weights; }
};

#endif