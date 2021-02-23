/****************************************************************************************
*
*  File name : obstacle_ship.cu
*
*  Function  : Class functions for the obstacle ship used in the obstacle
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

#include <thrust/device_vector.h>
#include "utilities.cuh"
#include "obstacle_ship.cuh"
#include <iostream>

/****************************************************************************************
*  Name     : Obstacle_Model
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Obstacle_Ship::Obstacle_Ship()
{
	l = 20;
	w = 4;

	T_chi = 3; 		// Ad hoc parameters, are very dependent on the ship type
	T_U = 10;

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance
	R_a = 20.0; 			    // WP acceptance radius (20.0)
	LOS_LD = 150.0; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0; 			    // LOS integral gain (0.0)

	wp_c_0 = 0;	wp_c_p = 0;
}

Obstacle_Ship::Obstacle_Ship(
	const float T_U, 												// In: Ship first order speed time constant
	const float T_chi, 												// In: Ship first order course time constant
	const float R_a, 												// In: Ship radius of acceptance parameter in WP following
	const float LOS_LD, 											// In: Ship lookahead distance parameter in LOS WP following
	const float LOS_K_i 											// In: Ship integral gain parameter in LOS WP following
	) : 
	T_U(T_U), T_chi(T_chi), R_a(R_a), LOS_LD(LOS_LD), LOS_K_i(LOS_K_i)
{
	l = 20;
	w = 4;

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance

	wp_c_0 = 0;	wp_c_p = 0;	
}

/****************************************************************************************
*  Name     : determine_active_waypoint_segment
*  Function : Two overloads depending on matrix library used.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_Ship::determine_active_waypoint_segment(
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,  			// In: Waypoints to follow
	const TML::Vector4f &xs 										// In: Ownship state
	)	
{
	n_wps = waypoints.get_cols();
	segment_passed = false;

	if (n_wps <= 2) { wp_c_0 = 0; wp_c_p = 0; return; }

	for (int i = wp_c_0; i < n_wps - 1; i++)
	{
		d_next_wp(0) = waypoints(0, i + 1) - xs(0);
		d_next_wp(1) = waypoints(1, i + 1) - xs(1);

		L_wp_segment(0) = waypoints(0, i + 1) - waypoints(0, i);
		L_wp_segment(1) = waypoints(1, i + 1) - waypoints(1, i);
		L_wp_segment = L_wp_segment.normalized();

		segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

		//(s > R_a && fabs(e) <= R_a))) 	
		if (d_next_wp.norm() <= R_a || segment_passed) { wp_c_0++; } 
		else										{ break; }		
	}
	wp_c_p = wp_c_0;
}

__host__ void Obstacle_Ship::determine_active_waypoint_segment(
	const Eigen::Matrix<double, 2, -1> &waypoints,  			// In: Waypoints to follow
	const Eigen::Vector4d &xs 						// In: Ownship state
	)	
{
	TML::PDMatrix<float, 2, MAX_N_WPS> waypoints_copy;
	TML::Vector4f xs_copy;
	TML::assign_eigen_object(waypoints_copy, waypoints);
	TML::assign_eigen_object(xs_copy, xs);

	determine_active_waypoint_segment(waypoints_copy, xs_copy);
}

/****************************************************************************************
*  Name     : update_guidance_references 
*  Function : Two overloads depending on matrix library used.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_Ship::update_guidance_references(
	float &u_d,																	// In/out: Surge reference
	float &chi_d,																// In/out: Course reference 
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,						// In: Waypoints to follow.
	const TML::Vector4f &xs, 													// In: Ownship state	
	const float dt, 															// In: Time step
	const Guidance_Method guidance_method										// In: Type of guidance used	
	)
{
	n_wps = waypoints.get_cols();
	alpha = 0.0f; e = 0.0f;
	segment_passed = false;
	
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
			e_int = 0.0f;
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

__host__ void Obstacle_Ship::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Vector4d &xs, 									// In: Ownship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	TML::Vector4f xs_copy;
	TML::PDMatrix<float, 2, MAX_N_WPS> waypoints_copy;
	TML::assign_eigen_object(waypoints_copy, waypoints); 
	TML::assign_eigen_object(xs_copy, xs); 

	update_guidance_references((float&)u_d, (float&)chi_d, waypoints_copy, xs_copy, (float)dt, guidance_method);
}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts obstacle state xs a number of dt units forward in time with the 
*			  chosen prediction method. Two overloads depending on matrix library used.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ TML::Vector4f Obstacle_Ship::predict(
	const TML::Vector4f &xs_old, 									// In: State [x, y, chi, U] to predict forward
	const double U_d, 												// In: Speed over ground (SOG) reference
	const double chi_d, 											// In: Course (COG) reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	chi_diff = angle_difference_pmpi(chi_d, xs_old(2));

	switch (prediction_method)
	{
		case Linear : 
			// Straight line trajectory with the current heading and surge speed
			xs_new(0) = xs_old(0) + dt * xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(1) + dt * xs_old(3) * sin(xs_old(2));
			xs_new.set_block<2, 1>(2, 0, xs_old.get_block<2, 1>(2, 0));
			break;
		case ERK1 : 
			// First set xs_new to the continuous time derivative of the model
			xs_new(0) = xs_old(3) * cos(xs_old(2));
			xs_new(1) = xs_old(3) * sin(xs_old(2));
			xs_new(2) = (1 / T_chi) * chi_diff;
			xs_new(3) = (1 / T_U) * (U_d - xs_old(3));

			// Then use forward euler to obtain new states
			xs_new *= dt;
			xs_new += xs_old;
			break;
		default :
			// Throw
			xs_new.set_zero(); 
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

__host__ Eigen::Vector4d Obstacle_Ship::predict(
	const Eigen::Vector4d &xs_old, 									// In: State to predict forward
	const double U_d, 												// In: Speed over ground (SOG) reference
	const double chi_d, 											// In: Course (COG) reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	TML::Vector4f xs_old_copy;
	TML::assign_eigen_object(xs_old_copy, xs_old);

	TML::Vector4f result = predict(xs_old_copy, U_d, chi_d, dt, prediction_method);
	Eigen::Vector4d xs_new;
	TML::assign_tml_object(xs_new, result);

	return xs_new;
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the obstacle ship trajectory for a sequence of avoidance maneuvers
*			  in the offset sequence. Two overloads depending on matrix library used.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Obstacle_Ship::predict_trajectory(
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory, 			// In/out: Obstacle ship trajectory
	const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, 	// In: Sequence of offsets in the candidate control behavior
	const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,			// In: Time indices for each Obstacle_Model avoidance maneuver
	const float u_d, 												// In: Surge reference
	const float chi_d, 												// In: Course reference
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, 			// In: Obstacle waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const float T,													// In: Prediction horizon
	const float dt 													// In: Prediction time step
	)
{
	n_samples = T / dt;
	
	trajectory.resize(4, n_samples);

	initialize_wp_following();

	man_count = 0;
	u_m = 1, u_d_p = u_d;
	chi_m = 0, chi_d_p = chi_d;
	xs_p = trajectory.get_col(0);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m = offset_sequence[2 * man_count + 1]; 
			if (man_count < (int)maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs_p, dt, guidance_method);

		xs_p = predict(xs_p, u_m * u_d_p , chi_d_p + chi_m, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.set_col(k + 1, xs_p);
	}
}

__host__ void Obstacle_Ship::predict_trajectory(
	Eigen::Matrix<double, 4, -1> &trajectory, 						// In/out: Own-ship trajectory
	const Eigen::VectorXd &offset_sequence, 						// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd &maneuver_times,							// In: Time indices for each ownship avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Waypoints to follow
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> trajectory_copy;
	TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence_copy;
	TML::PDMatrix<float, MAX_N_M, 1> maneuver_times_copy;
	TML::PDMatrix<float, 2, MAX_N_WPS> waypoints_copy;
	
	TML::assign_eigen_object(trajectory_copy, trajectory); 
	TML::assign_eigen_object(offset_sequence_copy, offset_sequence); 
	TML::assign_eigen_object(maneuver_times_copy, maneuver_times); 
	TML::assign_eigen_object(waypoints_copy, waypoints); 

	predict_trajectory(trajectory_copy, offset_sequence_copy, maneuver_times_copy, u_d, chi_d, waypoints_copy, prediction_method, guidance_method, T, dt);

	TML::assign_tml_object(trajectory, trajectory_copy);
}


/****************************************************************************************
		Private functions
*****************************************************************************************/