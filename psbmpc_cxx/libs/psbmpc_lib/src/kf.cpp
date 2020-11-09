/****************************************************************************************
*
*  File name : kf.cpp
*
*  Function  : Class functions for a "hardcoded" Kalman Filter (KF). Alternative version
*			   of the one created by Giorgio D. Kwame Minde Kufoalor through the Autosea
*		       project
*
*  
*            ---------------------
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

#include "kf.h"
#include <stdexcept>


/****************************************************************************************
*  Name     : KF
*  Function : Class constructors, initializes parameters and variables 
*  Author   : 
*  Modified :
*****************************************************************************************/
KF::KF() : 
	ID(0), t_0(0.0), t(0.0), initialized(false) 
{
	xs_p.setZero();
	xs_upd.setZero();

	I.setIdentity();

	A << 1, 0, 0.5, 0,
		 0, 1, 0, 0.5,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	C << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	Q << .050,  .000,  .001,  .000,  
	     .000,  .050,  .000,  .001,  
	     .001,  .000,  .001,  .000,  
	     .000,  .001,  .000,  .001;     
	     	
	R <<  .2,   .0,   .0,    .0,  
	      .0,   .2,   .0,    .0,  
	      .0,   .0,   .01,   .0,  
	      .0,   .0,   .0,   .01;

	P_0 << .1, .0, .0, .0,  
	     .0, .1, .0, .0,  
	     .0, .0, .1, .0,  
	     .0, .0, .0, .1;

	P_p = P_0;

	P_upd = P_0;
}

KF::KF(
	const Eigen::Vector4d& xs_0, 				// In: Initial filter state
	const Eigen::Matrix4d& P_0,					// In: Initial filter covariance
	const int ID, 								// In: Filter ID
	const double dt, 							// In: Sampling interval
	const double t_0							// In: Initial time
	) : 	
	ID(ID), t_0(t_0), t(t_0), initialized(true), xs_p(xs_0), P_0(P_0)
{
	I.setIdentity();

	A << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	C << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	Q << .050,  .000,  .001,  .000,  
	     .000,  .050,  .000,  .001,  
	     .001,  .000,  .001,  .000,  
	     .000,  .001,  .000,  .001;     
	     	
	R <<  .2,   .0,   .0,    .0,  
	      .0,   .2,   .0,    .0,  
	      .0,   .0,   .01,   .0,  
	      .0,   .0,   .0,   .01;

	xs_upd = xs_p;

	P_p = P_0;
	P_upd = P_0;
}

// Use the constructor below for simulations where the KF is used as a tracking system outside the COLAV algorithm
// where typically only position measurements of vessels are used.
KF::KF(
	const Eigen::Vector4d &xs_0, 				// In: Initial filter state
	const Eigen::Matrix4d &P_0,					// In: Initial filter covariance
	const int ID, 								// In: Filter ID
	const double dt, 							// In: Sampling interval
	const double t_0,							// In: Initial time
	const Eigen::Matrix4d &Q, 					// In: Process noise covariance
	const Eigen::Matrix4d &R					// In: Measurement noise covariance
	) : 	
	ID(ID), t_0(t_0), t(t_0), initialized(true), xs_p(xs_0), P_0(P_0)
{
	I.setIdentity();

	A << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	// In this case, only position measurements
	C << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;

	this->Q = Q;

	this->R = R;

	xs_upd = xs_p;

	P_p = P_0;
	P_upd = P_0;
}

/****************************************************************************************
*  Name     : reset
*  Function : Resets filter to specified time and state, and covariance to P_0
*  Author   : 
*  Modified :
*****************************************************************************************/
 void KF::reset(
 	const Eigen::Vector4d& xs_0,				// In: Initial filter state
	const Eigen::Matrix4d& P_0,					// In: Initial filter covariance
 	const double t_0 							// In: Initial time
 	)
{
 	this->t_0 = t_0;
 	t = t_0;

 	xs_p = xs_0;
 	xs_upd = xs_0;

	this->P_0 = P_0;
	
 	P_p = P_0;
 	P_upd = P_0;

 	initialized = true;
 }

/****************************************************************************************
*  Name     : predict
*  Function : Predicts the estimate dt seconds forward in time using the system model
*  Author   : 
*  Modified :
*****************************************************************************************/
void KF::predict(
	const double dt 							// In: Sampling interval
	)
{
	A << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	xs_p = A * xs_upd; 

	P_p = A * P_upd * A.transpose() + Q;
}

/****************************************************************************************
*  Name     : update
*  Function : Updates the KF prediction with the measurement, and 
*			  propagates the prediction
*  Author   : 
*  Modified :
*****************************************************************************************/
void KF::update(
	const Eigen::Vector4d &y_m, 				// In: Measurement of cartesian position and velocity
	const double duration_lost, 				// In: How long a track on an obstacle has been lost
	const double dt 							// In: Sampling interval
	)
{
	if(!initialized)
    	throw std::runtime_error("Filter is not initialized!");

    if (duration_lost == 0.0){

		Eigen::Matrix<double, 4, 4> K;
		K = P_p * C.transpose() * (C * P_p * C.transpose() + R).inverse(); 

		xs_upd = xs_p + K * (y_m - C * xs_p);

		P_upd = (I - K * C) * P_p;

    } else{

    	xs_upd = xs_p;

    	P_upd = P_p;
    }

	t += dt; // Time used for fault detection (measurement loss)
}

// Use the update function below when the KF is used as a tracking system outside the COLAV algorithm
// where typically only position measurements of vessels are used.
void KF::update(
	const Eigen::Vector2d &y_m, 				// In: Measurement of cartesian position
	const double dt 							// In: Sampling interval
	)
{
	if(!initialized)
    	throw std::runtime_error("Filter is not initialized!");

	Eigen::Matrix<double, 4, 2> K;
	Eigen::Matrix<double, 2, 4> C_2D = C.block<2, 4>(0, 0);
	K = P_p * C_2D.transpose() * (C_2D * P_p * C_2D.transpose() + R.block<2, 2>(0, 0)).inverse(); 

	xs_upd = xs_p + K * (y_m - C_2D * xs_p);

	P_upd = (I - K * C_2D) * P_p;

	predict(dt); 

	t += dt; // Time used for fault detection (measurement loss)
}