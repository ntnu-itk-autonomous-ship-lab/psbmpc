/****************************************************************************************
*
*  File name : kinematic_ship_models_cpu.cpp
*
*  Function  : Class functions for the CPU based kinematic ship model used in the obstacle
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

#include "cpu/utilities_cpu.hpp"
#include "cpu/kinematic_ship_models_cpu.hpp"
#include <vector>
#include <iostream>

namespace PSBMPC_LIB
{
namespace CPU
{
/****************************************************************************************
*  Name     : Kinematic_Ship
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
Kinematic_Ship::Kinematic_Ship()
{
	l = 5.0; // milliAmpere dims
	w = 3.0;
	
	T_U = 1.44;
	T_chi = 0.92; 		// Ad hoc identified time constants for milliAmpere

	/* l = 10.0;
	w = 4.0;
	
	T_U = 10.0;
	T_chi = 8.0; 		// Ad hoc time constants for a 10m long ship */

	// Guidance parameters
	e_int = 0.0;
	e_int_max = 20 * M_PI / 180.0; 	// Maximum integral correction in LOS guidance
	R_a = 5.0; 			   			// WP acceptance radius (5.0 for milliampere, 20.0 for "normal ship")
	LOS_LD = 66.0; 					// LOS lookahead distance (66.0 for milliampere, 200.0 for "normal ship") 
	LOS_K_i = 0.0; 			    	// LOS integral gain (0.0)

	wp_c_0 = 0;	wp_c_p = 0;
		
}

Kinematic_Ship::Kinematic_Ship(
	const double T_U, 												// In: Ship first order speed time constant
	const double T_chi, 											// In: Ship first order course time constant
	const double R_a, 												// In: Ship radius of acceptance parameter in WP following
	const double LOS_LD, 											// In: Ship lookahead distance parameter in LOS WP following
	const double LOS_K_i 											// In: Ship integral gain parameter in LOS WP following
	) : 
	T_U(T_U), T_chi(T_chi), R_a(R_a), LOS_LD(LOS_LD), LOS_K_i(LOS_K_i)
{
	l = 5; // milliAmpere dims
	w = 3;

	/* l = 10.0;
	w = 4.0;*/

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance

	wp_c_0 = 0;	wp_c_p = 0;	
}

/****************************************************************************************
*  Name     : determine_active_waypoint_segment
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinematic_Ship::determine_active_waypoint_segment(
	const Eigen::Matrix<double, 2, -1> &waypoints,  				// In: Waypoints to follow
	const Eigen::Vector4d &xs 										// In: Ownship state
	)	
{
	int n_wps = waypoints.cols();
	Eigen::Vector2d d_0_wp, L_wp_segment, L_0wp;
	bool segment_passed = false;

	if (n_wps <= 2) { wp_c_0 = 0; wp_c_p = 0; return; }

	for (int i = wp_c_0; i < n_wps - 1; i++)
	{
		d_0_wp(0) = waypoints(0, i + 1) - xs(0);
		d_0_wp(1) = waypoints(1, i + 1) - xs(1);

		L_wp_segment(0) = waypoints(0, i + 1) - waypoints(0, i);
		L_wp_segment(1) = waypoints(1, i + 1) - waypoints(1, i);
		L_wp_segment = L_wp_segment.normalized();

		segment_passed = L_wp_segment.dot(d_0_wp.normalized()) < cos(90 * DEG2RAD);

		//(s > R_a && fabs(e) <= R_a))) 	
		if (d_0_wp.norm() <= R_a || segment_passed) { wp_c_0++; std::cout << "Segment " << i << " passed" << std::endl; } 
		else										{ break; }		
		
	}
	wp_c_p = wp_c_0;
}

/****************************************************************************************
*  Name     : update_guidance_references 
*  Function : Two overloads, one general purpose function, and one specialized for LOS
*			  where an artificial cross-track error is applied.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinematic_Ship::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Vector4d &xs, 									// In: Ownship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	// No surge modification
	u_d = u_d;
	
	int n_wps = waypoints.cols();
	double alpha(0.0), e(0.0);
	Eigen::Vector2d d_next_wp, L_wp_segment;
	bool segment_passed = false;
	
	if (wp_c_p < n_wps - 1 && (guidance_method == LOS || guidance_method == WPP))
	{
		// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
		d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
		d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);

		L_wp_segment(0) = waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p);
		L_wp_segment(1) = waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p);
		L_wp_segment = L_wp_segment.normalized();

		segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

		if (d_next_wp.norm() <= R_a || segment_passed) //(s > 0 && e <= R_a))
		{
			e_int = 0;
			wp_c_p ++;
		} 
	}

	switch (guidance_method)
	{
		case LOS : 
			// Compute path tangential angle
			if (wp_c_p == n_wps - 1)
			{
				alpha = atan2(waypoints(1, wp_c_p) - waypoints(1, wp_c_p - 1), 
							waypoints(0, wp_c_p) - waypoints(0, wp_c_p - 1));
			}
			else
			{
				alpha = atan2(waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p), 
							waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p));
			}
			// Compute cross track error and integrate it
			e = - (xs(0) - waypoints(0, wp_c_p)) * sin(alpha) + (xs(1) - waypoints(1, wp_c_p)) * cos(alpha);
			e_int += e * dt;
			if (e_int >= e_int_max) e_int -= e * dt;

			chi_d = alpha + atan2( - (e + LOS_K_i * e_int), LOS_LD);
			break;
		case WPP :
			// Note that the WPP method will make the own-ship drive in roundabouts
			// around and towards the last waypoint, unless some LOS-element such 
			// as cross-track error and path tangential angle is used, or a 
			// forced keep current course is implemented
			if (wp_c_p == n_wps - 1)
			{
				d_next_wp(0) = waypoints(0, wp_c_p) - xs(0);
				d_next_wp(1) = waypoints(1, wp_c_p) - xs(1);
			}
			else
			{
				d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
				d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);
			}
			chi_d = atan2(d_next_wp(1), d_next_wp(0));	
			break;
		case CH :
			chi_d = xs(2);
			break;
		default : 
			// Throw
			break;
	}
}

void Kinematic_Ship::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const double e_m,				 							// In: Modifier to the LOS-guidance cross track error to cause a different path alignment
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Vector4d &xs, 									// In: Ownship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	assert(guidance_method == LOS);

	// No surge modification
	u_d = u_d;
	
	int n_wps = waypoints.cols();
	double alpha(0.0), e(0.0);
	Eigen::Vector2d d_next_wp, L_wp_segment;
	bool segment_passed = false;
	
	if (wp_c_p < n_wps - 1)
	{
		// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
		d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
		d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);

		L_wp_segment(0) = waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p);
		L_wp_segment(1) = waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p);
		L_wp_segment = L_wp_segment.normalized();

		segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

		if (d_next_wp.norm() <= R_a || segment_passed) //(s > 0 && e <= R_a))
		{
			e_int = 0;
			wp_c_p ++;
		} 
	}

	// Compute path tangential angle
	if (wp_c_p == n_wps - 1)
	{
		alpha = atan2(waypoints(1, wp_c_p) - waypoints(1, wp_c_p - 1), 
					waypoints(0, wp_c_p) - waypoints(0, wp_c_p - 1));
	}
	else
	{
		alpha = atan2(waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p), 
					waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p));
	}
	// Compute cross track error and integrate it
	e = - (xs(0) - waypoints(0, wp_c_p)) * sin(alpha) + (xs(1) - waypoints(1, wp_c_p)) * cos(alpha);
	e += e_m; // Add artificial cross track error to cause different path alignment, for obstacle prediction
	e_int += e * dt;
	if (e_int >= e_int_max) e_int -= e * dt;

	chi_d = alpha + atan2( - (e + LOS_K_i * e_int), LOS_LD);
}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts obstacle state xs a number of dt units forward in time with the 
*			  chosen prediction method
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::Vector4d Kinematic_Ship::predict(
	const Eigen::Vector4d &xs_old, 									// In: State [x, y, chi, U] to predict forward
	const double U_d, 												// In: Speed over ground (SOG) reference
	const double chi_d, 											// In: Course (COG) reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	Eigen::Vector4d xs_new;
	double chi_diff = angle_difference_pmpi(chi_d, xs_old(2));

	switch (prediction_method)
	{
		case Linear : 
			// Straight line trajectory with the current heading and surge speed
			xs_new(0) = xs_old(0) + dt * xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(1) + dt * xs_old(3) * sin(xs_old(2));
			xs_new.block<2, 1>(2, 0) = xs_old.block<2, 1>(2, 0);
			break;
		case ERK1 : 
			// First set xs_new to the continuous time derivative of the model
			xs_new(0) = xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(3) * sin(xs_old(2));
			xs_new(2) = (1 / T_chi) * chi_diff;
			xs_new(3) = (1 / T_U) * (U_d - xs_old(3));

			// Then use forward euler to obtain new states
			xs_new = xs_old + dt * xs_new;
			break;
		default :
			// Throw
			xs_new.setZero(); 
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the obstacle ship trajectory for a sequence of avoidance maneuvers
*			  in the offset sequence, or for a cross track modifier to the original path.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinematic_Ship::predict_trajectory(
	Eigen::MatrixXd &trajectory, 									// In/out: Obstacle ship trajectory
	const Eigen::VectorXd &offset_sequence, 						// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd &maneuver_times,							// In: Time indices for each collision avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Obstacle waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	int n_samples = std::round(T / dt);
	
	assert(trajectory.rows() == 4);
	trajectory.conservativeResize(4, n_samples);

	wp_c_p = wp_c_0;

	int man_count = 0;
	double u_m = 1, u_d_p = u_d;
	double chi_m = 0, chi_d_p = chi_d;
	Eigen::Vector4d xs = trajectory.col(0);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m += offset_sequence[2 * man_count + 1]; 
			if (man_count < maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs, dt, guidance_method);

		xs = predict(xs, u_m * u_d_p , chi_d_p + chi_m, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.col(k + 1) = xs;
	}
}

void Kinematic_Ship::predict_trajectory(
	Eigen::MatrixXd &trajectory, 									// In/out: Obstacle ship trajectory
	const double e_m,						 						// In: Modifier to the LOS-guidance cross track error to cause a different path alignment
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Obstacle waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	int n_samples = std::round(T / dt);
	
	assert(trajectory.rows() == 4);
	trajectory.conservativeResize(4, n_samples);

	wp_c_p = wp_c_0;

	double u_d_p = u_d;
	double chi_d_p = chi_d;
	Eigen::Vector4d xs = trajectory.col(0);

	for (int k = 0; k < n_samples; k++)
	{ 
		update_guidance_references(u_d_p, chi_d_p, e_m, waypoints, xs, dt, guidance_method);

		xs = predict(xs, u_d_p, chi_d_p, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.col(k + 1) = xs;
	}
}

/****************************************************************************************
		Private functions
*****************************************************************************************/
}
}