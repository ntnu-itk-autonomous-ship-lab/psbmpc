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
#include "kf.h"
#include "iostream" 

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Eigen::Matrix<double,10,1>& xs, 						// In: Obstacle state [x, y, V_x, V_y, A, B, C, D, id]
	const Eigen::Matrix<double,10,1>& P, 						// In: Obstacle state [x, y, V_x, V_y, A, B, C, D, id]
	const bool filter_on, 										// In: Boolean determining whether KF should be used or not
	const double T, 											// In: Prediction horizon
	const double dt, 											// In: Sampling interval
	) : 
	id(xs(9)), filter_on(filter_on), 
	l(xs(5) + xs(6)), w(xs(7) + xs(8)), 
	x_offset(xs(5) - xs(6)), y_offset(xs(7) - xs(8)),
	duration_tracked(0.0), duration_lost(0.0)
	{

	int n_samp = T / dt;

	x_p.resize(n_samp);
	y_p.resize(n_samp);
	V_x_p.resize(n_samp);
	V_y_p.resize(n_samp);

	double psi = xs(2);
	x_p(0) = xs(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	y_p(0) = xs(1) + x_offset * cos(psi) + y_offset * sin(psi);

	V_x_p(0) = xs(3) * cos(psi) - xs(4) * sin(psi); 
	V_y_p(0) = xs(3) * sin(psi) + xs(4) * cos(psi); 

	Eigen::Vector4d xs_0;
	xs_0 << x_p(0), y_p(0), V_x_p(0), V_y_p(0);

	kf = new KF(xs_0, id, dt, 0.0);

	if (filter_on) {

		kf->update(xs_0, duration_lost, dt);

	}else{

	}

	mrou = new MROU(0.8, 0, 0.8, 0.1, 0.1);

}

/****************************************************************************************
*  Name     : ~Obstacle
*  Function : Class destructor, clears the dynamic kalman filter object
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::~Obstacle(){
	delete kf;
	delete mrou;
}


/****************************************************************************************
*  Name     : predict_independent_trajectories
*  Function : Predicts the obstacle trajectories for scenarios where the obstacle
*			  does not take the own-ship into account 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle::predict_independent_trajectories(
	const double T, 
	const double dt
	)
{

}