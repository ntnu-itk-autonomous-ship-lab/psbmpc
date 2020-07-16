/****************************************************************************************
*
*  File name : Obstacle_Model.cpp
*
*  Function  : Class functions for the simple kinematic model used in the obstacle
*			   collision avoidance system predictions.
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

#include "utilities.h"
#include "Obstacle_Model.h"
#include <vector>
#include <string>
#include <iostream>

/****************************************************************************************
*  Name     : Obstacle_Model
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle_Model::Obstacle_Model()
{
	T_chi = 3;
	T_U = 10;

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance
	R_a = 20.0; 			    // WP acceptance radius (20.0)
	LOS_LD = 145.0; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0; 			    // LOS integral gain (0.0)

	wp_counter = 0;				// Determines the active waypoint segment
		
}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts obstacle state xs a number of dt units forward in time with the 
*			  chosen prediction method
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::Vector4d Obstacle_Model::predict(
	const Eigen::Vector4d &xs_old, 									// In: State [x, y, chi, U] to predict forward
	const double U_d, 												// In: Speed over ground (SOG) reference
	const double chi_d, 											// In: Course (COG) reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	Eigen::Vector4d xs_new;

	switch (prediction_method)
	{
		case Linear : 
		{
			// Straight line trajectory with the current heading and surge speed
			xs_new(0) = xs_old(0) + dt * xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(1) + dt * xs_old(3) * sin(xs_old(2));
			xs_new(2) = xs_old(2);
			xs_new(3) = xs_old(3);

			break;
		}
		case ERK1 : 
		{
			// First set xs_new to the continuous time derivative of the model
			xs_new(0) = xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(3) * sin(xs_old(2));
			xs_new(2) = (1 / T_chi) * (chi_d - xs_old(2));
			xs_new(3) = (1 / T_U) * (U_d - xs_old(3));

			// Then use forward euler to obtain new states
			xs_new = xs_old + dt * xs_new;
			break;
		}
		default :
		{
			std::cout << "The prediction method does not exist or is not implemented" << std::endl;
			xs_new.setZero(); 
		}
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the Obstacle_Model trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_Model::predict_trajectory(
	Eigen::Matrix<double, 4, -1>& trajectory, 						// In/out: Own-ship trajectory
	const Eigen::VectorXd offset_sequence, 							// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd maneuver_times,							// In: Time indices for each Obstacle_Model avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Obstacle_Model waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	int n_samples = T / dt;
	
	trajectory.conservativeResize(4, n_samples);

	wp_counter = 0;
	int man_count = 0;
	double u_m = 1, u_d_p = u_d;
	double chi_m = 0, chi_d_p = chi_d;
	Eigen::Vector4d xs = trajectory.col(0);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m = offset_sequence[2 * man_count + 1]; 
			if (man_count < maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs, dt, guidance_method);

		xs = predict(xs, u_m * u_d_p , chi_d_p + chi_m, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.col(k + 1) = xs;
	}
}


/****************************************************************************************
		Private functions
*****************************************************************************************/

/****************************************************************************************
*  Name     : update_guidance_references 
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_Model::update_guidance_references(
	double &u_d,												// Out: Surge reference
	double &chi_d,												// Out: Course reference (set equal to heading reference, compensating for crab angle through LOS_K_i if at all..)
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Vector4d &xs, 									// In: Obstacle state [x, y, chi, U]	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	int n_wps = waypoints.cols();
	double alpha, e, s;
	Eigen::Vector2d d_next_wp;

	
	if (wp_counter < n_wps - 1 && (guidance_method == LOS || guidance_method == WPP))
	{
		// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
		d_next_wp(0) = waypoints(0, wp_counter + 1) - xs(0);
		d_next_wp(1) = waypoints(1, wp_counter + 1) - xs(1);

		alpha = atan2(waypoints(1, wp_counter + 1) - waypoints(1, wp_counter), 
					waypoints(0, wp_counter + 1) - waypoints(0, wp_counter));

		s =   (xs(0) - waypoints(0, wp_counter + 1)) * cos(alpha) + (xs(1) - waypoints(1, wp_counter + 1)) * sin(alpha);
		e = - (xs(0) - waypoints(0, wp_counter + 1)) * sin(alpha) + (xs(1) - waypoints(1, wp_counter + 1)) * cos(alpha);
		// Positive along-track error with respect to the furthest waypoint in the active segment (for cross-track
		// this does not matter) means the segment is passed by, not neccessarily within R_a, which can be the case 
		// when using avoidance maneuvers

		if (d_next_wp.norm() <= R_a || (s > R_a && e <= R_a))
		{
			e_int = 0;
			if (wp_counter < n_wps - 1)	wp_counter += 1;
		} 
	}

	switch (guidance_method)
	{
		case LOS : 
		{
			// Compute path tangential angle
			if (wp_counter == n_wps - 1)
			{
				alpha = atan2(waypoints(1, wp_counter) - waypoints(1, wp_counter - 1), 
							waypoints(0, wp_counter) - waypoints(0, wp_counter - 1));
			}
			else
			{
				alpha = atan2(waypoints(1, wp_counter + 1) - waypoints(1, wp_counter), 
							waypoints(0, wp_counter + 1) - waypoints(0, wp_counter));
			}
			// Compute cross track error and integrate it
			e = - (xs(0) - waypoints(0, wp_counter)) * sin(alpha) + (xs(1) - waypoints(1, wp_counter)) * cos(alpha);
			e_int += e * dt;
			if (e_int >= e_int_max) e_int -= e * dt;

			chi_d = alpha + atan2( - (e + LOS_K_i * e_int), LOS_LD);
			break;
		}
		case WPP :
		{
			// Note that the WPP method will make the own-ship drive in roundabouts
			// around and towards the last waypoint, unless some LOS-element such 
			// as cross-track error and path tangential angle is used
			if (wp_counter == n_wps - 1)
			{
				d_next_wp(0) = waypoints(0, wp_counter) - xs(0);
				d_next_wp(1) = waypoints(1, wp_counter) - xs(1);
			}
			else
			{
				d_next_wp(0) = waypoints(0, wp_counter + 1) - xs(0);
				d_next_wp(1) = waypoints(1, wp_counter + 1) - xs(1);
			}
			chi_d = atan2(d_next_wp(1), d_next_wp(0));	
			break;
		}
		case CH :
		{
			chi_d = xs(2);
			break;
		}
		default : 
		{
			std::cout << "This guidance method does not exist or is not implemented" << std::endl;
			break;
		}
	}
}