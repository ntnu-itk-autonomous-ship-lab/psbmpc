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
//#include "obstacle_sbmpc.cuh"
#include "utilities.cuh"
#include <iostream>

/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::Prediction_Obstacle() = default;

Prediction_Obstacle::Prediction_Obstacle(
	const TML::PDMatrix<float, 1, 9>& xs_aug, 					// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	Obstacle(xs_aug, colav_on) //, sbmpc(new Obstacle_SBMPC())
{
	int n_samples = std::round(T / dt);
	
	A_CV = TML::Matrix4f::identity();
	A_CV(0, 2) = dt;
	A_CV(1, 3) = dt; 

	xs_p.resize(4, n_samples);
	xs_p.set_col(0, xs_0);

	xs_k_p.resize(4, n_samples);
	xs_k_p.set_col(0, xs_0);
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

Prediction_Obstacle::~Prediction_Obstacle() = default;

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
	const Tracked_Obstacle &rhs 										// In: Rhs tracked obstacle to assign
	)
{
	assign_data(rhs);

	return *this;
}

/****************************************************************************************
*  Name     : predict_independent_trajectory
*  Function : Predicts the straight line obstacle trajectory at the current predicted time
*			  in the joint prediction, for use in other obstacle
*			  SB-MPC predictions (if this obstacle's COLAV is not assumed to be acting
*			  seen from the predicting obstacle`s point of view).
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Prediction_Obstacle::predict_independent_trajectory(						
	const double T, 											// In: Time horizon
	const double dt, 											// In: Time step
	const int k													// In: Index of the current predicted time
	)
{
	int n_samples = std::round(T / dt);
	xs_k_p.resize(4, n_samples);
	xs_k_p.set_col(0, xs_p.get_col(k));

	A_CV = TML::Matrix4f::identity();
	A_CV(0, 2) = dt;
	A_CV(1, 3) = dt; 

	for(int j = 0; j < n_samples; j++)
	{
		if (j < n_samples - 1) 
		{
			xs_k_p.set_col(j + 1, A_CV * xs_k_p.get_col(j));
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
	const TML::Vector4f &xs, 								// In: Predicted obstacle state [x, y, V_x, V_y]
	const int k												// In: Index of the current predicted time
	)
{
	double psi = atan2(xs(3), xs(2));
	xs_p(0, k) = xs(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_p(1, k) = xs(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_p(2, k) = xs(2);
	xs_p(3, k) = xs(3);
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
	this->ID = po.ID;

	this->colav_on = po.colav_on;

	this->A = po.A; this->B = po.B; this->C = po.C; this->D = po.D;
	this->l = po.l; this->w = po.w;

	this->x_offset = po.x_offset; this->y_offset = po.y_offset;

	this->a_p = po.a_p;

	this->mu = po.mu;

	this->xs_0 = po.xs_0;
	this->P_0 = po.P_0;

	this->A_CV = po.A_CV;

	this->xs_p = po.xs_p;

	this->xs_k_p = po.xs_k_p;

	//this->sbmpc.reset(new Obstacle_SBMPC(*(po.sbmpc)));
}

void Prediction_Obstacle::assign_data(
	const Tracked_Obstacle &to 														// In: Tracked_Obstacle whose data to assign to *this
	)
{
	this->ID = to.ID;

	this->colav_on = to.colav_on;

	this->A = to.A; this->B = to.B; this->C = to.C; this->D = to.D;
	this->l = to.l; this->w = to.w;

	this->x_offset = to.x_offset; this->y_offset = to.y_offset;
	
	TML::assign_eigen_object(this->xs_0, to.xs_0);
	TML::assign_eigen_object(this->P_0, to.P_0);

	A_CV = TML::Matrix4f::identity();
	A_CV(0, 2) = 0.5f;
	A_CV(1, 3) = 0.5f; 

	this->waypoints.resize(2, 2);

	TML::assign_eigen_object(this->xs_p, to.xs_p[0]);

	TML::assign_eigen_object(this->xs_k_p, to.xs_p[0]);

	//this->sbmpc.reset(new Obstacle_SBMPC());
}