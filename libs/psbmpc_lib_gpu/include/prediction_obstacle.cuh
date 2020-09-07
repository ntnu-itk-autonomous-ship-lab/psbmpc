/****************************************************************************************
*
*  File name : prediction_obstacle.cuh
*
*  Function  : Header file for the predicion obstacle class. Derived simpler variant of  
*			   the obstacle class used in the PSB-MPC, specifically made for the PSB-MPC
*			   predictions with active obstacle COLAV systems.
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


#ifndef _PREDICTION_OBSTACLE_CUH_
#define _PREDICTION_OBSTACLE_CUH_

#include "obstacle.cuh"
#include <thrust/device_vector.h>

class Obstacle_SBMPC;

class Prediction_Obstacle : public Obstacle
{
private:

	// State and covariance at the current time or predicted time (depending on the derived class usage)
	CML::MatrixXd xs_0;
	CML::MatrixXd P_0;

	CML::MatrixXd A_CV;

	// Predicted state trajectory
	CML::MatrixXd xs_p;

	__host__ __device__ void assign_data(const Prediction_Obstacle &po);
	
public:

	//Obstacle_SBMPC *sbmpc;

	__host__ __device__ Prediction_Obstacle() {};

	__host__ __device__ Prediction_Obstacle(
		const CML::MatrixXd &xs_aug,
		const CML::MatrixXd &P,	 
		const bool colav_on, 
		const double T, 
		const double dt);

	__host__ __device__ Prediction_Obstacle(const Prediction_Obstacle &po);

	__host__ __device__ ~Prediction_Obstacle();

	__host__ __device__ Prediction_Obstacle& operator=(const Prediction_Obstacle &po);

	__host__ __device__ void clean();

	__host__ __device__ CML::MatrixXd get_state() const { return xs_0; };

	__host__ __device__ CML::MatrixXd get_trajectory() const { return xs_p; };

	__host__ __device__ void set_trajectory(const CML::MatrixXd &xs_p) { if (colav_on) { this->xs_p = xs_p; }};

	__host__ __device__ void predict_independent_trajectory(const double T, const double dt);

	__host__ __device__ void update(const CML::MatrixXd &xs);
};

#endif