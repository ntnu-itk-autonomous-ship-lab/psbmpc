/****************************************************************************************
*
*  File name : prediction_obstacle.cu
*
*  Function  : Class functions for the derived predicted obstacle class in the PSB-MPC.
*			   Only used in theMPC-predictions for cases where the obstacle has its own 
*			   SB-MPC/COLAV and needs to keep track of its nearby obstacles.
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

#include <thrust/device_vector.h>
#include "prediction_obstacle.cuh"
#include "obstacle_sbmpc.cuh"
#include "utilities.cuh"
#include <iostream>

/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Prediction_Obstacle::Prediction_Obstacle(
	const Eigen::VectorXd& xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID] at the current time
	const Eigen::VectorXd &P, 									// In: Obstacle covariance at the current time
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	Obstacle(xs_aug, P, colav_on)
{
	int n_samples = std::round(T / dt);
	
	A_CV << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	xs_p.resize(4, n_samples);
	xs_p.col(0) = xs_0;
}

/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Prediction_Obstacle::Prediction_Obstacle(
	const Prediction_Obstacle &po 								// In: Obstacle to copy
	) : 
	Obstacle(po)
{
	assign_data(po);
}

/****************************************************************************************
*  Name     : ~Prediction_Obstacle
*  Function : Destructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Prediction_Obstacle::~Prediction_Obstacle()
{
	clean();
}

/****************************************************************************************
*  Name     : operator=
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Prediction_Obstacle& Prediction_Obstacle::operator=(
	const Prediction_Obstacle &rhs 								 // In: Rhs obstacle to assign
	)
{
	if (this == &rhs) 
	{ 
		return *this; 
	}

	assign_data(rhs);

	return *this;
}

/****************************************************************************************
*  Name     : predict_independent_trajectory
*  Function : Predicts the straight line obstacle trajectory for use in other obstacle
*			  SB-MPC predictions (if this obstacle's COLAV is not assumed to be acting).
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Prediction_Obstacle::predict_independent_trajectory(						
	const double T, 											// In: Time horizon
	const double dt 											// In: Time step
	)
{
	int n_samples = std::round(T / dt);
	xs_p.resize(4, n_samples);
	xs_p.col(0) = xs_0;

	A_CV << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	for(int k = 0; k < n_samples; k++)
	{
		if (k < n_samples - 1) 
		{
			xs_p.col(k + 1) = A_CV * xs_p.col(k);
		}
	}
}

/****************************************************************************************
*  Name     : update
*  Function : Updates the predicted obstacle state, at the current prediction time prior 
*			  to a run of the obstacle SB-MPC.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Prediction_Obstacle::update(
	const Eigen::Vector4d &xs 								// In: Predicted obstacle state [x, y, V_x, V_y]
	)
{
	double psi = atan2(xs(3), xs(2));
	xs_0(0) = xs(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs(2);
	xs_0(3) = xs(3);
}

/****************************************************************************************
*  Private functions
*****************************************************************************************/
/****************************************************************************************
*  Name     : assign_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Prediction_Obstacle::assign_data(
	const Prediction_Obstacle &po 							// In: Prediction_Obstacle whose data to assign to *this
	)
{
	this->A_CV = po.A_CV;

	this->xs_p = po.xs_p;

	clean();
	this->sbmpc = new Obstacle_SBMPC(*(po.sbmpc));
}