/****************************************************************************************
*
*  File name : cuda_obstacle.cpp
*
*  Function  : Cuda obstacle class functions. Derived class of base Obstacle class,
*		  	   used in the PSB-MPC GPU CB_Cost_Functor computation.
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

#include "cuda_obstacle.h"
#include "obstacle_sbmpc.cuh"
#include "utilities.cuh"
#include <iostream> 

/****************************************************************************************
*  Name     : Cuda_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Cuda_Obstacle::Cuda_Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_a,								// In: Obstacle intention probability vector
	const double Pr_CC, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Boolean determining whether KF should be used or not
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	Obstacle(xs_aug, P, colav_on), 
	duration_lost(0.0),
	kf(new KF(xs_0, P_0, ID, dt, 0.0)),
	mrou(new MROU(0.8, 0, 0.8, 0.1, 0.1))
{
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }

	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	xs_p.resize(1);
	xs_p[0].resize(4, n_samples);

	P_p.resize(16, n_samples);
	P_p.col(0) = P;
	
	xs_p[0].col(0) = xs_0;

	if(filter_on) 
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}
}

/****************************************************************************************
*  Name     : Cuda_Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Cuda_Obstacle::Cuda_Obstacle(
	const Cuda_Obstacle &co 													// In: Obstacle to copy
	) : 
	Pr_a(co.Pr_a), 
	Pr_CC(co.Pr_CC),
	duration_tracked(co.duration_tracked), duration_lost(co.duration_lost),
	mu(co.mu),
	P_p(co.P_p), 
	xs_p(co.xs_p),
	v_p(co.v_p),
	ps_ordering(co.ps_ordering),
	ps_course_changes(co.ps_course_changes), ps_weights(co.ps_weights), ps_maneuver_times(co.ps_maneuver_times),
	kf(new KF(*(co.kf))), 
	mrou(new MROU(*(co.mrou))),
	sbmpc(new Obstacle_sbmpc(*(co.sbmpc)))
{}

/****************************************************************************************
*  Name     : ~Cuda_Obstacle
*  Function : Destructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle::~Cuda_Obstacle()
{

}

/****************************************************************************************
*  Name     : operator=
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Cuda_Obstacle &co 										// In: Rhs to assign
	)
{
	if (this == &co) 	{ return *this; }
	if (kf != NULL)		{ delete kf; }
	if (mrou != NULL)	{ delete mrou; }
	if (sbmpc != NULL) 	{ delete sbmpc; }
	return *this = Cuda_Obstacle(co);
}