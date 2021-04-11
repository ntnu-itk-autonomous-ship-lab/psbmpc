/****************************************************************************************
*
*  File name : kinetic_ship_models_gpu.cu
*
*  Function  : Class functions for the GPU used kinetic ship models. 
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

#include "gpu/utilities_gpu.cuh"
#include "gpu/kinetic_ship_models_gpu.cuh"

#include <thrust/device_vector.h>
#include <vector>
#include <iostream>
#include "assert.h"
#include "stdio.h"

namespace PSBMPC_LIB
{
namespace GPU
{
	
/****************************************************************************************
*  Name     : Kinetic_Ship_Base_3DOF
*  Function : Class constructors
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ Kinetic_Ship_Base_3DOF::Kinetic_Ship_Base_3DOF()
{
	tau.set_zero();
	Cvv.set_zero();
	Dvv.set_zero();

	// Guidance parameters
	e_int = 0;
	e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance
	R_a = 20.0; 			    // WP acceptance radius (20.0)
	LOS_LD = 450.0; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0; 			    // LOS integral gain (0.0)

	wp_c_0 = 0;	wp_c_p = 0;

	// The model parameters are derived class dependent, and therefore set in the derived class constructors
}

/****************************************************************************************
*  Name     : determine_active_waypoint_segment
*  Function : Two overloads depending on matrix library used.
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Kinetic_Ship_Base_3DOF::determine_active_waypoint_segment(
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,  			// In: Waypoints to follow
	const TML::Vector6f &xs 										// In: Ship state
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

__host__ void Kinetic_Ship_Base_3DOF::determine_active_waypoint_segment(
	const Eigen::Matrix<double, 2, -1> &waypoints,  			// In: Waypoints to follow
	const Eigen::Matrix<double, 6, 1> &xs 						// In: Ship state
	)	
{
	TML::PDMatrix<float, 2, MAX_N_WPS> waypoints_copy;
	TML::Vector6f xs_copy;
	TML::assign_eigen_object(waypoints_copy, waypoints);
	TML::assign_eigen_object(xs_copy, xs);

	determine_active_waypoint_segment(waypoints_copy, xs_copy);
}

/****************************************************************************************
*  Name     : update_guidance_references 
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Kinetic_Ship_Base_3DOF::update_guidance_references(
	float &u_d,																	// In/out: Surge reference
	float &chi_d,																// In/out: Course reference 
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,						// In: Waypoints to follow.
	const TML::Vector6f &xs, 													// In: Ship state	
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

__host__ void Kinetic_Ship_Base_3DOF::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Matrix<double, 6, 1> &xs, 						// In: Ship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	TML::Vector6f xs_copy;
	TML::PDMatrix<float, 2, MAX_N_WPS> waypoints_copy;
	TML::assign_eigen_object(waypoints_copy, waypoints); 
	TML::assign_eigen_object(xs_copy, xs); 

	float u_d_copy = (float)u_d, chi_d_copy = (float)chi_d;
	update_guidance_references(u_d_copy, chi_d_copy, waypoints_copy, xs_copy, (float)dt, guidance_method);
	u_d = (double)u_d_copy; chi_d = (double)chi_d_copy;
}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts the ship state xs a number of dt units forward in time with the 
*			  chosen prediction method
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ TML::Vector6f Kinetic_Ship_Base_3DOF::predict(
	const TML::Vector6f &xs_old, 									// In: State to predict forward
	const float dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	eta = xs_old.get_block<3, 1>(0, 0);
	nu = xs_old.get_block<3, 1>(3, 0);

	switch (prediction_method)
	{
		case Linear : 
			// Straight line trajectory with the current heading and surge speed
			eta = eta + dt * rotate_vector_3D(nu, eta(2), Yaw);
			nu(0) = nu(0);
			nu(1) = 0.0f;
			nu(2) = 0.0f;
			xs_new.set_block<3, 1>(0, 0, eta); 
			xs_new.set_block<3, 1>(3, 0, nu);
			break;
		case ERK1 : 
			update_Cvv(nu); 
			update_Dvv(nu);

			eta = eta + dt * rotate_vector_3D(nu, eta(2), Yaw);
			nu  = nu  + dt * M_inv * (- Cvv - Dvv + tau);
			
			xs_new.set_block<3, 1>(0, 0, eta); 
			xs_new.set_block<3, 1>(3, 0, nu);
			break;
		default :
			// Throw
			xs_new.set_zero();
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

__host__ Eigen::Matrix<double, 6, 1> Kinetic_Ship_Base_3DOF::predict(
	const Eigen::Matrix<double, 6, 1> &xs_old, 						// In: State to predict forward
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	TML::Vector6f xs_old_copy;
	TML::assign_eigen_object(xs_old_copy, xs_old);

	TML::Vector6f result = predict(xs_old_copy, dt, prediction_method);
	Eigen::Matrix<double, 6, 1> xs_new;
	TML::assign_tml_object(xs_new, result);

	return xs_new;
}

/****************************************************************************************
		Private functions
*****************************************************************************************/

/****************************************************************************************
*  Name     : Cvv
*  Function : Calculates the "coriolis vector" for the 3DOF surface vessel based on 
*			  Fossen 2011.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinetic_Ship_Base_3DOF::update_Cvv(
	const TML::Vector3f &nu 									// In: BODY velocity vector nu = [u, v, r]^T				
	)
{	
	/* C(nu) = [ 0 		0 		c13(nu)]
			   [ 0 		0 		c23(nu)]
			   [c31(nu) c32(nu) 	0] 
	with 
	c13(nu) = -m21 * u - m22 * v - m23 * r,
	c23(nu) = 	m11 * u + m12 * v + m13 * r
	c31(nu) = -c13(nu), c32(nu) = -c23(nu)	 
	Written out (without temporaries c13 and c23 introduced):  
	*/
	Cvv(0) = - (M(1, 0) * nu(0) + M(1, 1) * nu(1) + M(1, 2) * nu(2)) * nu(2);
	Cvv(1) = (M(0, 0) * nu(0) + M(0, 1) * nu(1) + M(0, 2) * nu(2)) * nu(2);
	Cvv(2) = M(1, 0) * nu(0) * nu(0) 			+ 
			(M(1, 1) - M(0, 0)) * nu(0) * nu(1) - 
			M(0, 1) * nu(1) * nu(1) 			+ 
			M(1, 2) * nu(0) * nu(2) 			- 
			M(0, 2) * nu(1) * nu(2);

	
	/* 
	Alternative requiring more memory
	float c13 = - (M(1, 0) * nu(0) + M(1, 1) * nu(1) + M(1, 2) * nu(2));
	float c23 = (M(0, 0) * nu(0) + M(0, 1) * nu(1) + M(0, 2) * nu(2));
	Cvv(0) = c13 * nu(2);
	Cvv(1) = c23 * nu(2);
	Cvv(2) = -c13 * nu(0) - c23 * nu(1); */
}

/****************************************************************************************
*  Name     : Dvv
*  Function : Calculates the "damping vector" for the 3DOF surface vessel based on 
*			  Fossen 2011
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinetic_Ship_Base_3DOF::update_Dvv(
	const TML::Vector3f &nu 									// In: BODY velocity vector nu = [u, v, r]^T
	)
{
	Dvv(0) = 	- D_l(0, 0) * nu(0) - D_l(0, 1) * nu(1) - D_l(0, 2) * nu(2) 	
			 	- X_uu * fabs(nu(0)) * nu(0) - X_uuu * nu(0) * nu(0) * nu(0);

	Dvv(1) = 	- D_l(1, 0) * nu(0) - D_l(1, 1) * nu(1) - D_l(1, 2) * nu(2)
				- Y_vv * fabs(nu(1)) * nu(1) - Y_rv * fabs(nu(2)) * nu(1) - Y_vr * fabs(nu(1)) * nu(2) - Y_rr * fabs(nu(2)) * nu(2)
				- Y_vvv * nu(1) * nu(1) * nu(1); 

	Dvv(2) = 	- D_l(2, 0) * nu(0) - D_l(2, 1) * nu(1) - D_l(2, 2) * nu(2) 	
				- N_vv * fabs(nu(1)) * nu(1) - N_rv * fabs(nu(2)) * nu(1) - N_vr * fabs(nu(1)) * nu(2) - N_rr * fabs(nu(2)) * nu(2)
				- N_rrr * nu(2) * nu(2) * nu(2);
}




//=======================================================================================
// Telemetron class methods
//=======================================================================================
/****************************************************************************************
*  Name     : Telemetron
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
Telemetron::Telemetron()
{
	tau.set_zero();
	Cvv.set_zero();
	Dvv.set_zero();

	// Guidance parameters
	e_int = 0.0f;
	e_int_max = 20.0f * M_PI / 180.0f; // Maximum integral correction in LOS guidance
	R_a = 20.0f; 			    // WP acceptance radius (20.0)
	LOS_LD = 250.0f; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0f; 			    // LOS integral gain (0.0)

	wp_c_0 = 0;	wp_c_p = 0;

	// Model parameters
	l_r = 4.0f; // [m] distance from rudder to CG
	A = 5.0f; // [m]  in reality the length is 14,5 m.
	B = 5.0f; // [m]
	C = 1.5f; // [m]
	D = 1.5f; // [m]
	l = (A + B);
	w = (C + D);
	calculate_position_offsets();

	m = 3980.0f; // [kg]
	I_z = 19703.0f; // [kg m2]

	TML::Matrix3f M_RB;
	M_RB.set_zero();
	M_RB(0, 0) = m; M_RB(1, 1) = m; M_RB(2, 2) = I_z;

	TML::Matrix3f M_A; M_A.set_zero();
	
	// Total inertia matrix
	M = M_RB + M_A;
	M_inv = M.inverse();

	D_l.set_zero();
	D_l(0, 0) = -50.0f; D_l(1, 1) = -200.0f; D_l(2, 2) = -1281.0f;

	// Nonlinear damping terms
	X_uu = -135.0f;
	Y_vv = -2000.0f; Y_vr = 0.0f, Y_rv = 0.0f, Y_rr = 0.0f;
	N_vv = 0.0f;	N_vr = 0.0f, N_rv = 0.0f, N_rr = 0.0f;
	X_uuu = 0.0f;
	Y_vvv = 0.0f;
	N_rrr = -3224.0f;

	//Force limits
	Fx_min = -6550.0f;
	Fx_max = 13100.0f;
	Fy_min = -645.0f;
	Fy_max = 645.0f;

	// Controller parameters
	Kp_u = 1.0f;
	Kp_psi = 5.0f;
	Kd_psi = 1.0f;
	Kp_r = 8.0f;
		
	//Motion limits
	r_max = 0.34f * DEG2RAD; // [rad/s] default max yaw rate
}

/****************************************************************************************
*  Name     : update_ctrl_input
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ void Telemetron::update_ctrl_input(
	const float u_d,										// In: Surge reference
	const float psi_d, 										// In: Heading (taken equal to course reference due to assumed zero crab angle and side slip) reference
	const TML::Vector6f &xs 								// In: State
	)
{
	update_Cvv(xs.get_block<3, 1>(3, 0));
	update_Dvv(xs.get_block<3, 1>(3, 0));

	Fx = Cvv(0) + Dvv(0) + Kp_u * m * (u_d - xs(3));
	
	psi_diff = angle_difference_pmpi(psi_d, xs(2));
	Fy = (Kp_psi * I_z ) * (psi_diff - Kd_psi * xs(5));
    Fy *= 1.0 / l_r;

	// Saturate
	if (Fx < Fx_min)  Fx = Fx_min;
	if (Fx > Fx_max)  Fx = Fx_max;
	if (Fy < Fy_min)  Fy = Fy_min;
	if (Fy > Fy_max)  Fy = Fy_max;

	tau(0) = Fx;
	tau(1) = Fy;
	tau(2) = l_r * Fy;
}

__host__ void Telemetron::update_ctrl_input(
	const double u_d,										// In: Surge reference
	const double psi_d, 									// In: Heading (taken equal to course reference due to assumed zero crab angle and side slip) reference
	const Eigen::Matrix<double, 6, 1> &xs 					// In: State
	)
{
	TML::Vector6f xs_copy;
	TML::assign_eigen_object(xs_copy, xs);

	update_ctrl_input(u_d, psi_d, xs_copy);
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the ship trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence. Two overloads depending on matrix library used. 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void Telemetron::predict_trajectory(
	TML::PDMatrix<float, 6, MAX_N_SAMPLES> &trajectory, 						// In/out: Ship trajectory
	const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,	 			// In: Sequence of offsets in the candidate control behavior
	const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,						// In: Time indices for each ship avoidance maneuver
	const float u_d, 															// In: Surge reference
	const float chi_d, 															// In: Course reference
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, 						// In: Ship waypoints
	const Prediction_Method prediction_method,									// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 										// In: Type of guidance to be used
	const float T,																// In: Prediction horizon
	const float dt 																// In: Prediction time step
	)
{
	n_samples = round(T / dt);

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

		update_ctrl_input(u_m * u_d_p, chi_m + chi_d_p, xs_p);

		xs_p = predict(xs_p, dt, prediction_method);

		//printf("xs = %f %f %f %f %f %f \n", xs(0), xs(1), xs(2), xs(3), xs(4), xs(5));
		
		if (k < n_samples - 1) 
		{
			trajectory.set_col(k + 1, xs_p);
		}
	}
}

__host__ __device__ void Telemetron::predict_trajectory(
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory, 						// In/out: Ship trajectory with states [x, y, chi, U]^T
	const TML::PDVector6f &ship_state,											// In: Initial ship state [x, y, psi, u, v, r]^T
	const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,	 			// In: Sequence of offsets in the candidate control behavior
	const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,						// In: Time indices for each ship avoidance maneuver
	const float u_d, 															// In: Surge reference
	const float chi_d, 															// In: Course reference
	const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, 						// In: Ship waypoints
	const Prediction_Method prediction_method,									// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 										// In: Type of guidance to be used
	const float T,																// In: Prediction horizon
	const float dt 																// In: Prediction time step
	)
{
	n_samples = round(T / dt);

	initialize_wp_following();

	man_count = 0;
	u_m = 1, u_d_p = u_d;
	chi_m = 0, chi_d_p = chi_d;
	xs_p = ship_state;
	
	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m = offset_sequence[2 * man_count + 1]; 
			if (man_count < (int)maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs_p, dt, guidance_method);

		update_ctrl_input(u_m * u_d_p, chi_m + chi_d_p, xs_p);

		v_p(0) = xs_p(3); v_p(1) = xs_p(4);
		trajectory(0, k) = xs_p(0);
		trajectory(1, k) = xs_p(1);
		trajectory(2, k) = xs_p(2);
		trajectory(3, k) = v_p.norm();

		xs_p = predict(xs_p, dt, prediction_method);

		//printf("xs = %f %f %f %f %f %f \n", xs(0), xs(1), xs(2), xs(3), xs(4), xs(5));
	}
}

__host__ void Telemetron::predict_trajectory(
	Eigen::MatrixXd &trajectory, 									// In/out: Ship trajectory
	const Eigen::VectorXd &offset_sequence, 						// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd &maneuver_times,							// In: Time indices for each ship avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Waypoints to follow
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	
	TML::PDMatrix<float, 6, MAX_N_SAMPLES> trajectory_copy;
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

}
}