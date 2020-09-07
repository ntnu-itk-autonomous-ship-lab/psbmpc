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

#include "cuda_obstacle.cuh"
#include "obstacle_sbmpc.cuh"
#include "kf.h"
#include "mrou.h"
#include "utilities.cuh"
#include <iostream> 

/****************************************************************************************
*  Name     : Cuda_Obstacle
*  Function : Copy constructor. Overloaded for two derived obstacle types.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle::Cuda_Obstacle(
	const Cuda_Obstacle &co 													// In: Obstacle to copy
	) : 
	Obstacle(co)
{
	assign_data(co);
}

__host__ __device__ Cuda_Obstacle::Cuda_Obstacle(
	const Tracked_Obstacle &to 													// In: Obstacle to copy
	) : 
	Obstacle(to)
{
	assign_data(to);
}

/****************************************************************************************
*  Name     : ~Cuda_Obstacle
*  Function : Destructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Cuda_Obstacle::~Cuda_Obstacle()
{
	clean();
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
	if (this == &rhs) { return *this; }
	
	clean();

	assign_data(rhs);

	return *this;
}

__host__ __device__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Tracked_Obstacle &rhs 									// In: Rhs to assign
	)
{	
	clean();

	assign_data(rhs);

	return *this;
}

/****************************************************************************************
*  Name     : clean
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void Cuda_Obstacle::clean()
{
	if (xs_p != nullptr) 		{ delete[] xs_p; xs_p = nullptr; }
	if (ps_ordering != nullptr) { delete[] ps_ordering; ps_ordering = nullptr; }
	//if (sbmpc != nullptr) 		{ delete sbmpc; sbmpc = nullptr; }
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
void Cuda_Obstacle::assign_data(
	const Cuda_Obstacle &co 												// In: Cuda_Obstacle whose data to assign to *this
	)
{
	this->n_ps = co.ps_weights.size();

	this->Pr_a = co.Pr_a;

	this->Pr_CC = co.Pr_CC;

	this->duration_tracked = co.duration_tracked; this->duration_lost = co.duration_lost;
	
	this->mu = co.mu;

	this->P_p = co.P_p;
	this->v_p = co.v_p;

	this->ps_course_changes = co.ps_course_changes; this->ps_weights = co.ps_weights; this->ps_maneuver_times = co.ps_maneuver_times;
	
	this->kf = KF(co.kf);

	this->mrou = MROU(co.mrou);

	//this->sbmpc = new Obstacle_SBMPC(*(co.sbmpc));
	
	this->xs_p = new CML::MatrixXd[n_ps];
	this->ps_ordering = new Intention[n_ps];

	for (int ps = 0; ps < n_ps; ps++)
	{
		this->xs_p[ps] = co.xs_p[ps];
		this->ps_ordering[ps] = co.ps_ordering[ps];
	}
}

void Cuda_Obstacle::assign_data(
	const Tracked_Obstacle &to 												// In: Tracked_Obstacle whose data to assign to *this
	)
{
	this->n_ps = to.ps_weights.size();

	CML::assign_eigen_object(this->Pr_a, to.Pr_a);

	this->Pr_CC = to.Pr_CC;

	this->duration_tracked = to.duration_tracked; this->duration_lost = to.duration_lost;
	
	CML::assign_eigen_object(this->P_p, to.P_p);
	CML::assign_eigen_object(this->v_p, to.v_p);

	CML::assign_eigen_object(this->ps_course_changes, to.ps_course_changes); 
	CML::assign_eigen_object(this->ps_weights, to.ps_weights); 
	CML::assign_eigen_object(this->ps_maneuver_times, to.ps_maneuver_times);
	
	this->kf = KF(*(to.kf));

	this->mrou = MROU(*(to.mrou));

	//this->sbmpc = new Obstacle_SBMPC();
	
	this->xs_p = new CML::MatrixXd[n_ps];
	this->ps_ordering = new Intention[n_ps];

	this->mu.resize(n_ps, 1);
	for (int ps = 0; ps < n_ps; ps++)
	{
		this->mu[ps] = to.mu[ps];

		CML::assign_eigen_object(this->xs_p[ps], to.xs_p[ps]);

		this->ps_ordering[ps] = to.ps_ordering[ps];
	}
}