/****************************************************************************************
*
*  File name : kinetic_ship_models_cpu.cpp
*
*  Function  : Class functions for the CPU used kinetic ship models: The base class,
*			   Telemetron and MilliAmpere(unfinished). 
*			   
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#include "cpu/utilities_cpu.hpp"
#include "cpu/kinetic_ship_models_cpu.hpp"
#include <vector>
#include <iostream>

namespace PSBMPC_LIB
{
namespace CPU
{
/****************************************************************************************
*  Name     : Ship_Base_3DOF
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
Kinetic_Ship_Base_3DOF::Kinetic_Ship_Base_3DOF()
{
	tau = Eigen::Vector3d::Zero();
	Cvv = Eigen::Vector3d::Zero();
	Dvv = Eigen::Vector3d::Zero();

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
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinetic_Ship_Base_3DOF::determine_active_waypoint_segment(
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
void Kinetic_Ship_Base_3DOF::update_guidance_references(
	double &u_d,												// In/out: Surge reference
	double &chi_d,												// In/out: Course reference 
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Matrix<double, 6, 1> &xs, 						// In: Ownship state	
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	// No surge modification
	u_d = u_d;
	
	int n_wps = waypoints.cols();
	double alpha, e;
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
void Kinetic_Ship_Base_3DOF::update_Cvv(
	const Eigen::Vector3d &nu 									// In: BODY velocity vector nu = [u, v, r]^T				
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
	Cvv(0) = - (M(1, 0) * nu(0) + M(1, 1) * nu(1) + M(1, 2) * nu(2)) * nu(2);
	Cvv(1) = (M(0, 0) * nu(0) + M(0, 1) * nu(1) + M(0, 2) * nu(2)) * nu(2);
	Cvv(2) = M(1, 0) * nu(0) * nu(0) 			+ 
			(M(1, 1) - M(0, 0)) * nu(0) * nu(1) - 
			M(0, 1) * nu(1) * nu(1) 			+ 
			M(1, 2) * nu(0) * nu(2) 			- 
			M(0, 2) * nu(1) * nu(2);
	*/ 
	double c13 = - (M(1, 0) * nu(0) + M(1, 1) * nu(1) + M(1, 2) * nu(2));
	double c23 = (M(0, 0) * nu(0) + M(0, 1) * nu(1) + M(0, 2) * nu(2));
	Cvv(0) = c13 * nu(2);
	Cvv(1) = c23 * nu(2);
	Cvv(2) = -c13 * nu(0) - c23 * nu(1);
}

/****************************************************************************************
*  Name     : Dvv
*  Function : Calculates the "damping vector" for the 3DOF surface vessel based on 
*			  Fossen 2011
*  Author   : 
*  Modified :
*****************************************************************************************/
void Kinetic_Ship_Base_3DOF::update_Dvv(
	const Eigen::Vector3d &nu 									// In: BODY velocity vector nu = [u, v, r]^T
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
	// Model parameters
	l_r = 4.0; // [m] distance from rudder to CG
	A = 5; // [m]  in reality the length is 14,5 m.
	B = 5; // [m]
	C = 1.5; // [m]
	D = 1.5; // [m]
	l = (A + B);
	w = (C + D);
	calculate_position_offsets();

	m = 3980.0; // [kg]
	I_z = 19703.0; // [kg m2]

	Eigen::Matrix3d M_RB;
	M_RB << m, 0.0, 0.0,
			0.0, m, 0.0,
			0.0, 0.0, I_z;

	Eigen::Matrix3d M_A = Eigen::Matrix3d::Zero();
	
	// Total inertia matrix
	M = M_RB + M_A;

	D_l << -50, 0.0, 0.0,
			0.0, -200.0, 0.0,
			0.0, 0.0, -1281.0;

	// Nonlinear damping terms
	X_uu = -135.0;
	Y_vv = -2000.0; Y_vr = 0.0, Y_rv = 0.0, Y_rr = 0.0;
	N_vv = 0.0;	N_vr = 0.0, N_rv = 0.0, N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;

	//Force limits
	Fx_min = -6550.0;
	Fx_max = 13100.0;
	Fy_min = -645.0;
	Fy_max = 645.0;

	// Controller parameters
	Kp_u = 1.0;
	Kp_psi = 5.0;
	Kd_psi = 1.0;
	Kp_r = 8.0;
		
	//Motion limits
	r_max = 0.34 * DEG2RAD; // [rad/s] default max yaw rate
}
/****************************************************************************************
*  Name     : update_ctrl_input
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Telemetron::update_ctrl_input(
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
*			  chosen prediction method. Overloaded depending on if the input vector
*			  is updated or not.
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::Matrix<double, 6, 1> Telemetron::predict(
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
			nu  = nu  + dt * M.inverse() * (- Cvv - Dvv + tau);
			
			xs_new.block<3, 1>(0, 0) = eta; 
			xs_new.block<3, 1>(3, 0) = nu;
			break;
		default :
			// Throw
			xs_new.setZero(); 
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

Eigen::Matrix<double, 6, 1> Telemetron::predict(
	const Eigen::Matrix<double, 6, 1> &xs_old, 						// In: State to predict forward
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{	
	update_ctrl_input(u_d, chi_d, xs_old);

	return predict(xs_old, dt, prediction_method);
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the ownship trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Telemetron::predict_trajectory(
	Eigen::MatrixXd &trajectory, 									// In/out: Own-ship trajectory
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
	int n_samples = T / dt;
	
	assert(trajectory.rows() == 6);
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
			chi_m += offset_sequence[2 * man_count + 1]; 
			if (man_count < maneuver_times.size() - 1) man_count += 1;
		}  

		update_guidance_references(u_d_p, chi_d_p, waypoints, xs, dt, guidance_method);

		update_ctrl_input(u_m * u_d_p, chi_m + chi_d_p, xs);

		xs = predict(xs, dt, prediction_method);
		
		if (k < n_samples - 1) trajectory.col(k + 1) = xs;
	}
}

//=======================================================================================
// MilliAmpere class methods
//=======================================================================================
/****************************************************************************************
*  Name     : Telemetron
*  Function : Class constructor
*  Author   : 
*  Modified :
*****************************************************************************************/
/* MilliAmpere::MilliAmpere()
{
	l_1 = 1.8; l_2 = l_1;
	A = 2.5; 
	B = A; 
	C = 1.5; 
	D = C;
	l = A + B; 
	w = C + D;
	calculate_position_offsets();	
	
	M << 2390.0, 	0.0, 	0.0,
			0.0, 		2448.0, 268,1,
			0.0, 		-23.84, 4862;

	D_l << -106.6, 	0.0, 	0.0,
			0.0,	-29.44,	62.58,
			0.0,	7.34,	-142.7;

	X_uu = -21.39, X_uuu = -37.43;
	Y_vv = -172.9, Y_rv = -1517.0, Y_vr = 488.7, Y_rr = -198.2;
	N_vv = -4.352, N_rv = 437.8, N_vr = -122.0, N_rr = -831.7;
	D_l << -50, 0.0, 0.0,
			0.0, -200.0, 0.0,
			0.0, 0.0, -1281.0;

	// Nonlinear damping terms
	X_uu = -135.0;
	Y_vv = -2000.0; Y_vr = 0.0, Y_rv = 0.0, Y_rr = 0.0;
	N_vv = 0.0;	N_vr = 0.0, N_rv = 0.0, N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;
} */

/****************************************************************************************
*  Name     : update_ctrl_input
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
/* void MilliAmpere::update_ctrl_input(
	const double u_d,										// In: Surge reference
	const double psi_d, 									// In: Heading (taken equal to course reference due to assumed zero crab angle and side slip) reference
	const Eigen::Matrix<double, 6, 1> &xs 					// In: State
	)
{
	update_Cvv(xs.block<3, 1>(3, 0));
	update_Dvv(xs.block<3, 1>(3, 0));

	// Update thruster dynamics


	//double f_1, f_2, X_1, X_2, Y_1, Y_2, Z_1, Z_2;
	// f_1 = map rpm to thrust force for thruster 1
	// f_2 = -||- for thruster 2
	// X_1 = cos(alpha(0)) * f_1;
	// X_2 = cos(alpha(1)) * f_2; 
	// Y_1 = sin(alpha(0)) * f_1;
	// Y_2 = sin(alpha(1)) * f_2; 
	// tau(0) = X_1 + X_2;
	// tau(1) = Y_1 + Y_2;
	// tau(2) = l_1 * Y_1 + l_2 * Y_2;
} */

/****************************************************************************************
*  Name     : predict
*  Function : Predicts ownship state xs a number of dt units forward in time with the 
*			  chosen prediction method and the current input vector.
*  Author   : 
*  Modified :
*****************************************************************************************/
/* Eigen::Matrix<double, 6, 1> MilliAmpere::predict(
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
			nu  = nu  + dt * M.inverse() * (- Cvv - Dvv + tau);
			
			xs_new.block<3, 1>(0, 0) = eta; 
			xs_new.block<3, 1>(3, 0) = nu;
			break;
		default :
			// Throw
			xs_new.setZero(); 
	}
	xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
	return xs_new;
}

Eigen::Matrix<double, 6, 1> MilliAmpere::predict(
	const Eigen::Matrix<double, 6, 1> &xs_old, 						// In: State to predict forward
	const double u_d, 												// In: Surge reference
	const double chi_d, 											// In: Course reference
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{	
	update_ctrl_input(u_d, chi_d, xs_old);

	return predict(xs_old, dt, prediction_method);
} */

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the ownship trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence.
*  Author   : 
*  Modified :
*****************************************************************************************/
/* void MilliAmpere::predict_trajectory(
	Eigen::MatrixXd &trajectory, 									// In/out: Own-ship trajectory
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
	int n_samples = T / dt;
	
	assert(trajectory.cols() == 6);
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
} */

}
}