/****************************************************************************************
*
*  File name : cuda_obstacle.h
*
*  Function  : Header file for the obstacle class used by the PSB-MPC especially in the
*			   CUDA kernels, where raw pointers or custom containers must be used.
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


#ifndef _CUDA_OBSTACLE_H_
#define _CUDA_OBSTACLE_H_

#include <thrust/device_vector.h>
#include "Eigen/Dense"
#include "obstacle.cuh"
#include "tracked_obstacle.h"

class KF;
class MROU;
class Obstacle_SBMPC;

class Cuda_Obstacle : public Obstacle
{
private:

	int n_ps;

	// Vector of intention probabilities at the current time or last time of update
	Eigen::VectorXd Pr_a;

	// A priori COLREGS compliance probability at the current time or last time of update
	double Pr_CC;

	// If the AIS-based KF is on, the obstacle is tracked until it dies
	// while the duration lost may be reset if new measurements are aquired
	double duration_tracked, duration_lost;

	// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
	bool *mu;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
	// This is equal for all prediction scenarios including those with active COLAV (using MROU)
	Eigen::MatrixXd P_p;  

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	Eigen::MatrixXd *xs_p;

	// Mean predicted velocity for the obstacle (MROU): 
	Eigen::Vector2d v_p;

	// Prediction scenario ordering, size n_ps x 1 of intentions
	Intention *ps_ordering;

	// Course change ordering, weights and maneuvering times for the independent prediction scenarios: n_ps x 1
	Eigen::VectorXd ps_course_changes, ps_weights, ps_maneuver_times;
	
public:

	KF *kf;

	MROU *mrou;

	Obstacle_SBMPC *sbmpc;

	__host__ __device__ Cuda_Obstacle() {};

	__host__ __device__ Cuda_Obstacle(const Cuda_Obstacle &co);

	__host__ __device__ Cuda_Obstacle(const Tracked_Obstacle &to);

	__host__ __device__ ~Cuda_Obstacle();

	__host__ __device__ Cuda_Obstacle& operator=(const Cuda_Obstacle &rhs);

	__host__ __device__ Cuda_Obstacle& operator=(const Tracked_Obstacle &rhs);

	__host__ __device__ void clean();

	__device__ bool* get_COLREGS_violation_indicator() const { return mu; };

	__device__ double get_a_priori_CC_probability() const { return Pr_CC; };

	__device__ Eigen::VectorXd get_intention_probabilities() const { return Pr_a; };

	__device__ Eigen::MatrixXd* get_trajectories() const { return xs_p; };

	__device__ Eigen::MatrixXd get_trajectory_covariance() const { return P_p; };
};

#endif