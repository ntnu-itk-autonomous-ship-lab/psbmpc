/****************************************************************************************
*
*  File name : obstacle.cpp
*
*  Function  : Obstacle class functions. Modified version of the one created for SBMPC 
*			   by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor through the 
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

#include "obstacle.h"
#include "utilities.h"

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Eigen::VectorXd &xs_aug, 							// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 								// In: Obstacle covariance
	const bool colav_on										// In: Boolean determining whether the obstacle uses a COLAV system or not in the Obstacle_SBMPC predictions
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

// A, B, C, D, x_offset and y_offset are dont care variables here, meaning xy-symmetrical obstacle
Obstacle::Obstacle(
	const Eigen::VectorXd &xs_aug, 							// In: Obstacle state [x, y, V_x, V_y, l, w, ID]
	const bool colav_on										// In: Boolean determining whether the obstacle uses a COLAV system or not in the Obstacle_SBMPC predictions
	) : 
	ID(xs_aug(6)), colav_on(colav_on),
	A(0.0), B(0.0), C(0), D(0.0),
	l(xs_aug(4)), w(xs_aug(5)), 
	x_offset(0.0), y_offset(0.0)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = Eigen::Matrix4d::Identity();
}

/****************************************************************************************
*  Name     : Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Obstacle &o 													// In: Obstacle to copy
	)
{
	assign_data(o);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle& Obstacle::operator=(
	const Obstacle &rhs 													// In: Rhs Obstacle to assign to lhs
	)
{
	if (this == &rhs)
	{
		return *this;
	}
	
	assign_data(rhs);

	return *this;
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
void Obstacle::assign_data(
	const Obstacle &o 													// In: Obstacle whose data to assign to *this
	)
{
	// Boring non-pointer class member copy
	this->ID = o.ID;

	this->colav_on = o.colav_on;

	this->A = o.A; this->B = o.B; this->C = o.C; this->D = o.D;
	this->l = o.l; this->w = o.w;

	this->x_offset = o.x_offset; this->y_offset = o.y_offset;

	this->xs_0 = o.xs_0;
	this->P_0 = o.P_0;
}