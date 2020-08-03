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
*  Function : Copy constructor. Overloaded for two derived obstacle types.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Cuda_Obstacle::Cuda_Obstacle(
	const Cuda_Obstacle &co 													// In: Obstacle to copy
	) : 
	Pr_a(co.Pr_a), 
	Pr_CC(co.Pr_CC),
	duration_tracked(co.duration_tracked), duration_lost(co.duration_lost),
	P_p(co.P_p), 
	v_p(co.v_p),
	ps_course_changes(co.ps_course_changes), ps_weights(co.ps_weights), ps_maneuver_times(co.ps_maneuver_times),
	kf(new KF(*(co.kf))), 
	mrou(new MROU(*(co.mrou))),
	sbmpc(new Obstacle_sbmpc(*(co.sbmpc)))
{
	n_ps = co.ps_ordering.size();

	mu = new bool[n_ps];
	xs_p = new Eigen::MatrixXd[n_ps];
	ps_ordering = new Intention[n_ps];

	for (int ps = 0; ps < n_ps; ps++)
	{
		mu[ps] = co.mu[ps];
		xs_p[ps] = co.xs_p[ps];
		ps_ordering[ps] = co.ps_ordering[ps];
	}
}

Cuda_Obstacle::Cuda_Obstacle(
	const Tracked_Obstacle &to 													// In: Obstacle to copy
	) : 
	Pr_a(to.Pr_a), 
	Pr_CC(to.Pr_CC),
	duration_tracked(to.duration_tracked), duration_lost(to.duration_lost),
	P_p(to.P_p), 
	v_p(to.v_p),
	ps_course_changes(to.ps_course_changes), ps_weights(to.ps_weights), ps_maneuver_times(to.ps_maneuver_times),
	kf(new KF(*(to.kf))), 
	mrou(new MROU(*(to.mrou))),
	sbmpc(new Obstacle_sbmpc(*(to.sbmpc)))
{
	n_ps = co.ps_ordering.size();

	mu = new bool[n_ps];
	xs_p = new Eigen::MatrixXd[n_ps];
	ps_ordering = new Intention[n_ps];

	for (int ps = 0; ps < n_ps; ps++)
	{
		mu[ps] = to.mu[ps];
		xs_p[ps] = to.xs_p[ps];
		ps_ordering[ps] = to.ps_ordering[ps];
	}
}

/****************************************************************************************
*  Name     : ~Cuda_Obstacle
*  Function : Destructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle::~Cuda_Obstacle()
{
	delete[] mu;
	delete[] xs_p;
	delete[] ps_ordering;
	delete kf;
	delete mrou;
	delete sbmpc;
}

/****************************************************************************************
*  Name     : operator=
*  Function : Overloaded for two derived obstacle types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Cuda_Obstacle &rhs 										// In: Rhs to assign
	)
{
	if (this == &rhs) 			{ return *this; }
	if (mu != NULL) 			{ delete[] mu; }
	if (xs_p != NULL) 			{ delete[] xs_p; }
	if (ps_ordering != NULL) 	{ delete[] ps_ordering; }
	if (kf != NULL)				{ delete kf; }
	if (mrou != NULL)			{ delete mrou; }
	if (sbmpc != NULL) 			{ delete sbmpc; }

	return *this = Cuda_Obstacle(rhs);
}

__host__ __device__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Tracked_Obstacle &rhs 									// In: Rhs to assign
	)
{
	if (this == &rhs) 			{ return *this; }
	if (mu != NULL) 			{ delete[] mu; }
	if (xs_p != NULL) 			{ delete[] xs_p; }
	if (ps_ordering != NULL) 	{ delete[] ps_ordering; }
	if (kf != NULL)				{ delete kf; }
	if (mrou != NULL)			{ delete mrou; }
	if (sbmpc != NULL) 			{ delete sbmpc; }

	return *this = Cuda_Obstacle(rhs);
}