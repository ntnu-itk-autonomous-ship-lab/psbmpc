/****************************************************************************************
*
*  File name : cuda_obstacle.cu
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
//#include "obstacle_sbmpc.cuh"
#include "utilities.cuh"
#include <iostream> 

/****************************************************************************************
*  Name     : Cuda_Obstacle
*  Function : Copy constructor. Overloaded for two derived obstacle types.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ Cuda_Obstacle::Cuda_Obstacle(
	const Cuda_Obstacle &co 													// In: Obstacle to copy
	) : 
	Obstacle(co), xs_p(nullptr)
{
	assign_data(co);
}

__host__ Cuda_Obstacle::Cuda_Obstacle(
	const Tracked_Obstacle &to 													// In: Obstacle to copy
	) : 
	Obstacle(to), xs_p(nullptr)
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
*  Function : Overloaded for two derived obstacle types. NOTE: Should only be used
*			  when the lhs is uninitialized. 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Cuda_Obstacle &rhs 										// In: Rhs to assign
	)
{
	if (this == &rhs) { return *this; }

	assign_data(rhs);

	return *this;
}

__host__ Cuda_Obstacle& Cuda_Obstacle::operator=(
	const Tracked_Obstacle &rhs 									// In: Rhs Tracked_Obstacle whose data to assign
	)
{		
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
	cudaFree(xs_p);

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
__host__ void Cuda_Obstacle::assign_data(
	const Cuda_Obstacle &co 												// In: Cuda_Obstacle whose data to assign to *this
	)
{
	this->ID = co.ID;

	this->colav_on = co.colav_on;

	this->A = co.A; this->B = co.B; this->C = co.C; this->D = co.D;

	this->l = co.l; this->w = co.w;

	this->x_offset = co.x_offset; this->y_offset = co.y_offset;

	this->xs_0 = co.xs_0;
	this->P_0 = co.P_0;

	this->n_ps = co.n_ps;

	this->Pr_a = co.Pr_a;

	this->Pr_CC = co.Pr_CC;

	this->duration_tracked = co.duration_tracked; this->duration_lost = co.duration_lost;
	
	this->mu = co.mu;

	this->P_p = co.P_p;

	this->ps_weights = co.ps_weights;

	//this->sbmpc = new Obstacle_SBMPC(*(co.sbmpc));

	cudaMalloc((void**)&xs_p, n_ps * sizeof(CML::Pseudo_Dynamic_Matrix<double, 4, 2000>));
	cudaCheckErrors("Malloc trajectory inside cuda-to-cuda obstacle assign data failed.");

	for (int ps = 0; ps < n_ps; ps++)
	{
		cudaMemcpy(&xs_p[ps], &co.xs_p[ps], 1, cudaMemcpyHostToDevice);
		cudaCheckErrors("Memcpy of trajectory inside cuda-to-cuda obstacle assign data failed.");
	}
}

__host__ void Cuda_Obstacle::assign_data(
	const Tracked_Obstacle &to 										// In: Tracked_Obstacle ptr whose data to assign to *this
	)
{
	this->ID = to.ID;

	this->colav_on = to.colav_on;

	this->A = to.A; this->B = to.B; this->C = to.C; this->D = to.D;

	this->l = to.l; this->w = to.w;

	this->x_offset = to.x_offset; this->y_offset = to.y_offset;

	CML::assign_eigen_object(this->xs_0, to.xs_0);
	CML::assign_eigen_object(this->P_0, to.P_0);

	this->n_ps = to.ps_weights.size();

	CML::assign_eigen_object(this->Pr_a, to.Pr_a);

	this->Pr_CC = to.Pr_CC;

	this->duration_tracked = to.duration_tracked; this->duration_lost = to.duration_lost;
	
	CML::assign_eigen_object(this->P_p, to.P_p);

	CML::assign_eigen_object(this->ps_weights, to.ps_weights); 

	//this->sbmpc = new Obstacle_SBMPC();

	this->mu.resize(n_ps, 1);
	
	cudaMalloc((void**)&xs_p, n_ps * sizeof(CML::Pseudo_Dynamic_Matrix<double, 4, 2000>));
	cudaCheckErrors("Malloc trajectory inside tracked-to-cuda obstacle assign data failed.");

	CML::Pseudo_Dynamic_Matrix<double, 4, 2000> xs_p_temp, xs_p_temp2;

	for (int ps = 0; ps < n_ps; ps++)
	{
		this->mu[ps] = to.mu[ps];

		CML::assign_eigen_object(xs_p_temp, to.xs_p[ps]);

		cudaMemcpy(&xs_p[ps], &xs_p_temp, sizeof(CML::Pseudo_Dynamic_Matrix<double, 4, 2000>), cudaMemcpyHostToDevice);
		cudaCheckErrors("Memcpy of trajectory inside tracked-to-cuda obstacle assign data failed.");
	}
}