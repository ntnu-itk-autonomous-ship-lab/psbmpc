/****************************************************************************************
*
*  File name : prediction_obstacle.cpp
*
*  Function  : Class functions for a predicted obstacle in the PSB-MPC. Only used in the
*			   MPC-predictions for cases where the obstacle has its own SB-MPC/COLAV
*			   and needs to keep track of its nearby obstacles.
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


#include "prediction_obstacle.h"
#include "utilities.h"
#include <iostream>

/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::Prediction_Obstacle(
	const Eigen::VectorXd& xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	ID(xs_aug(8)), colav_on(colav_on),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6))
{
	int n_samples = std::round(T / dt);
	resize_trajectory(n_samples);
}

/****************************************************************************************
*  Name     : ~Prediction_Obstacle
*  Function : Class destructor, clears the dynamic kalman filter object
*  Author   : 
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::~Prediction_Obstacle()
{
	delete sbmpc;
}

/****************************************************************************************
*  Name     : predict_independent_trajectories
*  Function : Predicts the straight line obstacle trajectory for use in the obstacle
*			  SB-MPCs
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Prediction_Obstacle::predict_trajectory(						
	const double T, 											// In: Time horizon
	const double dt 											// In: Time step
	)
{
	int n_samples = std::round(T / dt);
	resize_trajectory(n_samples);

	for(int k = 0; k < n_samples; k++)
	{
		
	}
}


/****************************************************************************************
*  Name     : update
*  Function : Updates the predicted obstacle state, at the current time prior to a run of
*			  the predicted obstacle SB-MPC.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Prediction_Obstacle::update(
	const Eigen::Vector4d &xs, 									// In: Predicted obstacle state [x, y, V_x, V_y]
	const bool colav_on										// In: Boolean determining if the prediction obstacle object has an active COLAV system
	)
{
	double psi = atan2(xs(3), xs(2));
	xs_0(0) = xs(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs(2);
	xs_0(3) = xs(3);
}