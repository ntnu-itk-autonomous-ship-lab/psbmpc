/****************************************************************************************
*
*  File name : ownship.cu
*
*  Function  : Class functions for the ownship. Modified and extended version of the
*			   "Ship_Model" class created for SBMPC by Inger Berge Hagen and Giorgio D. 
*			   Kwame Minde Kufoalor through the Autosea project.
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

#include "utilities.cuh"
#include "ownship.cuh"
#include <thrust/device_vector.h>
#include <vector>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI

/****************************************************************************************
*  Name     : Ownship
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Ownship::Ownship()
{
	tau = Eigen::Vector3d::Zero();

	// Model parameters
	l_r = 4.0; // distance from rudder to CG
	A = 5; // [m]  in reality the length is 14,5 m.
	B = 5; // [m]
	C = 1.5; // [m]
	D = 1.5; // [m]
	l = (A + B);
	w = (C + D);
	calculate_position_offsets();

	m = 3980.0; // [kg]
	I_z = 19703.0; // [kg m2]

	// Added M terms
	X_udot = 0.0;
	Y_vdot = 0.0;
	Y_rdot = 0.0;
	N_vdot = 0.0;
	N_rdot = 0.0;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	X_u	= -50.0;
	Y_v = -200.0;
	Y_r = 0.0;
	N_v = 0.0;
	N_r = -1281.0;//-3224.0;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	X_uu = -135.0;
	Y_vv = -2000.0;
	N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;

	Eigen::Matrix3d M_tot;
	M_tot << m - X_udot, 0, 0,
	        0, m - Y_vdot, -Y_rdot,
	        0, -Y_rdot, I_z - N_rdot;
	M_inv = M_tot.inverse();

	//Force limits
	Fx_min = -6550.0;
	Fx_max = 13100.0;
	Fy_min = -645.0;
	Fy_max = 645.0;

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance
	R_a = 20.0; 			    // WP acceptance radius (20.0)
	LOS_LD = 150.0; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0; 			    // LOS integral gain (0.0)

	wp_c_0 = 0;	wp_c_p = 0;

	// Controller parameters
	Kp_u = 1.0;
	Kp_psi = 5.0;
	Kd_psi = 1.0;
	Kp_r = 8.0;
		
	//Motion limits
	r_max = 0.34 * DEG2RAD; // [rad/s] default max yaw rate
}

/****************************************************************************************
*  Name     : determine_active_waypoint_segment
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::determine_active_waypoint_segment(
	const Eigen::Matrix<double, 2, -1> &waypoints,  			// In: Waypoints to follow
	const Eigen::Matrix<double, 6, 1> &xs 						// In: Ownship state
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
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Matrix<double, 6, 1> &xs, 						// In: Ownship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	int n_wps = waypoints.cols();
	double alpha, e, s;
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
			std::cout << "This guidance method does not exist or is not implemented" << std::endl;
			break;
	}
}

/****************************************************************************************
*  Name     : update_ctrl_input
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::update_ctrl_input(
	const double u_d,										// In: Surge reference
	const double psi_d, 									// In: Heading (taken equal to course reference due to assumed zero crab angle and side slip) reference
	const Eigen::Matrix<double, 6, 1> &xs 					// In: State
	)
{
	update_Cvv(xs.block<3, 1>(3, 0));
	update_Dvv(xs.block<3, 1>(3, 0));

	double Fx = Cvv(0) + Dvv(0) + Kp_u * m * (u_d - xs(3));
	
	double psi_diff = angle_difference_pmpi(psi_d, xs(2));
	double Fy = (Kp_psi * I_z ) * (psi_diff - Kd_psi * xs(5));
    Fy *= 1.0 / l_r;

	// Saturate
	if (Fx < Fx_min)  Fx = Fx_min;
	if (Fx > Fx_max)  Fx = Fx_max;
	if (Fy < Fy_min)  Fy = Fy_min;
	if (Fy > Fy_max)  Fy = Fy_max;

	tau[0] = Fx;
	tau[1] = Fy;
	tau[2] = l_r * Fy;
}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts ownship state xs a number of dt units forward in time with the 
*			  chosen prediction method
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Eigen::Matrix<double, 6, 1> Ownship::predict(
	const Eigen::Matrix<double, 6, 1> &xs_old, 						// In: State to predict forward
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	Eigen::Matrix<double, 6, 1> xs_new;
	Eigen::Vector3d eta, nu;
	eta = xs_old.block<3, 1>(0, 0);
	nu = xs_old.block<3, 1>(3, 0);

	switch (prediction_method)
	{
		case Linear : 
			// Straight line trajectory with the current heading and surge speed
			eta = eta + dt * rotate_vector_3D(nu, eta(2), Yaw);
			nu(0) = nu(0);
			nu(1) = 0;
			nu(2) = 0;
			xs_new.block<3, 1>(0, 0) = eta; 
			xs_new.block<3, 1>(3, 0) = nu;
			break;
		case ERK1 : 
			update_Cvv(nu); 
			update_Dvv(nu);

			eta = eta + dt * rotate_vector_3D(nu, eta(2), Yaw);
			nu  = nu  + dt * M_inv * (- Cvv - Dvv + tau);
			
			xs_new.block<3, 1>(0, 0) = eta; 
			xs_new.block<3, 1>(3, 0) = nu;
			break;
		default :
			std::cout << "The prediction method does not exist or is not implemented" << std::endl;
			xs_new.setZero(); 
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the ownship trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::predict_trajectory(
	Eigen::Matrix<double, 6, -1>& trajectory, 						// In/out: Own-ship trajectory
	const Eigen::VectorXd &offset_sequence, 						// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd &maneuver_times,							// In: Time indices for each ownship avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Ownship waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	int n_samples = T / dt;
	
	trajectory.conservativeResize(6, n_samples);

	wp_c_p = wp_c_0;

	int man_count = 0;
	double u_m = 1, u_d_p = u_d;
	double chi_m = 0, chi_d_p = chi_d;
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m = offset_sequence[2 * man_count + 1]; 
			if (man_count < maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs, dt, guidance_method);

		update_ctrl_input(u_m * u_d_p, chi_m + chi_d_p, xs);

		xs = predict(xs, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.col(k + 1) = xs;
	}
}


/****************************************************************************************
		Private functions
*****************************************************************************************/

/****************************************************************************************
*  Name     : Cvv
*  Function : Calculates the "coriolis vector" for the 3DOF surface vessel based on 
*			  Fossen 2011. Use the equations for C_RB and C_A in Section 7.1
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::update_Cvv(
	const Eigen::Vector3d &nu 									// In: BODY velocity vector nu = [u, v, r]^T				
	)
{
	Cvv(0) = ((Y_vdot - m) * nu(1) + Y_rdot * nu(2)) * nu(2);
	Cvv(1) = (m      - X_udot) * nu(0) * nu(2);
	Cvv(2) = (X_udot - Y_vdot) * nu(0) * nu(1) - Y_rdot * nu(0) * nu(2);
}

/****************************************************************************************
*  Name     : Dvv
*  Function : Calculates the "damping vector" for the 3DOF surface vessel based on 
*			  Fossen 2011
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Ownship::update_Dvv(
	const Eigen::Vector3d &nu 									// In: BODY velocity vector nu = [u, v, r]^T
	)
{
	Dvv(0) = - (X_u + 						+  X_uu * fabs(nu(0))  		  + X_uuu * nu(0) * nu(0)) * nu(0);
	Dvv(1) = - ((Y_v * nu(1) + Y_r * nu(2)) + (Y_vv * fabs(nu(1)) * nu(1) + Y_vvv * nu(1) * nu(1) * nu(1)));
	Dvv(2) = - ((N_v * nu(1) + N_r * nu(2)) + (N_rr * fabs(nu(2)) * nu(2) + N_rrr * nu(2) * nu(2) * nu(2)));
}