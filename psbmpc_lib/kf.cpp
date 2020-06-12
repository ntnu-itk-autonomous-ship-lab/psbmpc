/****************************************************************************************
*
*  File name : kf.h
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
	id(0), n(4), m(4), initialized(0), t_0(0), t(0)
{

	xs_p.setZero();
	xs_upd.setZero();

	I.setIdentity();

	A << 1, 0, 0.5, 0,
		 0, 1, 0, 0.5,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	C <<  	1, 0, 0, 0,
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

}

KF::KF(
	const Eigen::Vector4d &xs_0, 				// In: Initial filter state
	const int id, 								// In: Filter id
	const double dt, 							// In: Sampling interval
	const double t_0							// In: Initial time
	) : 	
	id(id), n(4), m(4), xs_upd(xs_0), initialized(true), t_0(t_0), t(t_0) 
	{

	I.setIdentity();

	A << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	C <<  	1, 0, 0, 0,
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
}

/****************************************************************************************
*  Name     : reset
*  Function : Resets filter to specified time and state, and covariance to P_0
*  Author   : 
*  Modified :
*****************************************************************************************/
 void reset(
 	const Eigen::Vector4d &xs_0,				// In: Initial filter state
 	const double t_0 							// In: Initial time
 	){

 	this->t_0 = t_0;
 	t = t_0;

 	xs_p = xs_0;
 	xs_upd = xs_0;

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
	){

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
	){

	if(!initialized)

    	throw std::runtime_error("Filter is not initialized!");

    if (duration_lost == 0.0){

		K = P_p * C.transpose() * (C * P_p * C.transpose + R).inverse(); 

		xs_upd = xs_p + K * (y_m - C * xs_p);

		P_upd = (I - K * C) * P_p;

    } else{

    	xs_upd = xs_p;

    	P_upd = P_p;
    }

	t += dt; // Time used for fault detection (measurement loss)

}