/****************************************************************************************
*
*  File name : prediction_obstacle_cpu.cpp
*
*  Function  : Class functions for the CPU used predicted obstacle class in the PSB-MPC.
*			   Only used in the MPC-predictions for cases where the obstacle has its own 
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

#include "cpu/prediction_obstacle_cpu.hpp"
#include "cpu/obstacle_sbmpc_cpu.hpp"
#include <iostream>

namespace PSBMPC_LIB
{
namespace CPU
{
/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::Prediction_Obstacle() = default;

Prediction_Obstacle::Prediction_Obstacle(
	const Eigen::VectorXd& xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	ID(xs_aug(6)),
	A(0.0), B(0.0), C(0), D(0.0),
	l(xs_aug(4)), w(xs_aug(5)), 
	x_offset(0.0), y_offset(0.0), sbmpc(new Obstacle_SBMPC())
{
	int n_samples = std::round(T / dt);

	a_p = KCC;

	mu = false;
	
	A_CV << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	xs_p.resize(4, n_samples);
	xs_p.col(0) = xs_0;

	xs_k_p.resize(4, n_samples);
	xs_k_p.col(0) = xs_0;
}

/****************************************************************************************
*  Name     : Prediction_Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Prediction_Obstacle::Prediction_Obstacle(
	const Prediction_Obstacle &po 												// In: Prediction obstacle to copy
	)
{
	assign_data(po);
}

Prediction_Obstacle::Prediction_Obstacle(
	const Tracked_Obstacle &to 													// In: Tracked obstacle to copy
	)
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
	xs_k_p.col(0) = xs_p.col(k);

	A_CV << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	for(int j = 0; j < n_samples; j++)
	{
		if (j < n_samples - 1) 
		{
			xs_k_p.col(j + 1) = A_CV * xs_k_p.col(j);
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
	const Eigen::Vector4d &xs, 								// In: Predicted obstacle state [x, y, V_x, V_y]
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

	this->A = po.A; this->B = po.B; this->C = po.C; this->D = po.D;
	this->l = po.l; this->w = po.w;

	this->x_offset = po.x_offset; this->y_offset = po.y_offset;

	this->a_p = po.a_p;
	
	this->mu = po.mu;

	this->xs_0 = po.xs_0;

	this->A_CV = po.A_CV;

	this->xs_p = po.xs_p;

	this->xs_k_p = po.xs_k_p;

	this->sbmpc.reset(new Obstacle_SBMPC(*(po.sbmpc)));
}

void Prediction_Obstacle::assign_data(
	const Tracked_Obstacle &to 														// In: Tracked_Obstacle whose data to assign to *this
	)
{
	this->ID = to.ID;

	this->A = to.A; this->B = to.B; this->C = to.C; this->D = to.D;
	this->l = to.l; this->w = to.w;

	this->x_offset = to.x_offset; this->y_offset = to.y_offset;

	this->xs_0 = to.xs_0;

	this->A_CV << 	1, 0, 0.5, 0,
					0, 1, 0, 0.5,
					0, 0, 1, 0,
					0, 0, 0, 1;

	this->waypoints.resize(2, 2);

	this->xs_p = to.xs_p[0];

	this->xs_k_p = to.xs_p[0];

	this->sbmpc.reset(new Obstacle_SBMPC());
}

}
}