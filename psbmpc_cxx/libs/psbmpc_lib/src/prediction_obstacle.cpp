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
#include "obstacle_sbmpc.h"
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
	Obstacle(xs_aug, colav_on),
	sbmpc(new Obstacle_SBMPC())
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
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::Prediction_Obstacle(
	const Prediction_Obstacle &po 												// In: Prediction obstacle to copy
	) :
	Obstacle(po)
{
	assign_data(po);
}

Prediction_Obstacle::Prediction_Obstacle(
	const Tracked_Obstacle &to 													// In: Tracked obstacle to copy
	) :
	Obstacle(to)
{
	assign_data(to);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Prediction_Obstacle& Prediction_Obstacle::operator=(
	const Prediction_Obstacle &rhs 										// In: Rhs prediction obstacle to assign
	)
{
	if (this == &rhs)
	{
		return *this;
	}

	assign_data(rhs);

	return *this;
}

Prediction_Obstacle& Prediction_Obstacle::operator=(
	const Tracked_Obstacle &rhs 										// In: Rhs prediction obstacle to assign
	)
{
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
void Prediction_Obstacle::predict_independent_trajectory(						
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
void Prediction_Obstacle::update(
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
	const Prediction_Obstacle &po 													// In: Prediction_Obstacle whose data to assign to *this
	)
{
	this->A_CV = po.A_CV;

	this->xs_p = po.xs_p;

	this->sbmpc.reset(new Obstacle_SBMPC(*(po.sbmpc)));
}

void Prediction_Obstacle::assign_data(
	const Tracked_Obstacle &to 														// In: Tracked_Obstacle whose data to assign to *this
	)
{
	this->xs_p = to.xs_p[0];
}