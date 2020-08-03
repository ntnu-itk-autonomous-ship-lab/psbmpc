/****************************************************************************************
*
*  File name : obstacle.cu
*
*  Function  : Base Obstacle class functions. New version of the one created for 
*			   SBMPC by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor through the 
*  			   Autosea project.
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
#include "obstacle.cuh"
#include "obstacle_sbmpc.cuh"

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle::Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const bool colav_on											// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	) : 
	ID(xs_aug(8)), colav_on(colav_on),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6))
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = reshape(P, 4, 4);
}

/****************************************************************************************
*  Name     : Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle::Obstacle(
	const Obstacle &o 											// In: Obstacle to copy
	) : 
	ID(o.ID), 
	colav_on(o.colav_on), 
	A(o.A), B(o.B), C(o.C), D(o.D), 
	l(o.l), w(o.w),
	x_offset(o.x_offset), y_offset(o.y_offset), 
	xs_0(o.xs_0), P_0(o.P_0)
{}

/****************************************************************************************
*  Name     : operator=
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle& Obstacle::operator=(
	const Obstacle &o 											// In: Rhs Obstacle to assign to lhs
	)
{
	if (this == &o)
	{
		return *this;
	}

	return *this = Obstacle(o);
}