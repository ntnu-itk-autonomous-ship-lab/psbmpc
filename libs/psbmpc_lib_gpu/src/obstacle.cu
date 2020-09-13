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
#include "assert.h"

#include "obstacle.cuh"
#include "utilities.cuh"

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle::Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const bool colav_on											// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	) : 
	ID(xs_aug(8)), colav_on(colav_on),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6))
{

}

__host__ __device__ Obstacle::Obstacle(
	const CML::MatrixXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const bool colav_on											// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	) : 
	ID(xs_aug(8)), colav_on(colav_on),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6))
{
	assert(xs_aug.get_cols() == 1);
}