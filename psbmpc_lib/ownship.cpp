/****************************************************************************************
*
*  File name : ownship.cpp
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


#include "ownship.h"
#include  <iostream>

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
Ownship::Ownship()
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
	LOS_LD = 145.0; 			// LOS lookahead distance (100.0) 
	LOS_K_i = 0.0; 			    // LOS integral gain (0.0)

	wp_counter = 0;				// Determines the active waypoint segment

	// Controller parameters
	Kp_u = 1.0;
	Kp_psi = 5.0;
	Kd_psi = 1.0;
	Kp_r = 8.0;
		
	//Motion limits
	r_max = 0.34 * DEG2RAD; // [rad/s] default max yaw rate
	psi_rf = 0.7; // 1.0 means psi_d must be fully achieved before adapting

}

/****************************************************************************************
*  Name     : predict
*  Function : Predicts ownship state xs a number of dt units forward in time with the 
*			  chosen prediction method
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::VectorXd Ownship::predict(
	const Eigen::Matrix<double, 6, 1> &xs_old, 						// In: State to predict forward
	const double dt, 												// In: Time step
	const Prediction_Method prediction_method 						// In: Method used for prediction
	)
{
	Eigen::Matrix<double, 6, 1> xs_new;
	Eigen::Vector3d eta, nu;
	eta(0) = xs_old(0);
	eta(1) = xs_old(1);
	eta(2) = Utilities::wrap_angle_to_pmpi(xs_old(2));

	nu(0) = xs_old(3);
	nu(1) = xs_old(4);
	nu(2) = xs_old(5);

	switch (prediction_method)
	{
		case Linear : 
		{
			// Straight line trajectory with the current heading and surge speed
			eta = eta + dt * Utilities::rotate_vector_3D(nu, eta(2), Yaw);
			eta(2) = Utilities::wrap_angle_to_pmpi(xs_old(2)); 
			nu(0) = nu(0);
			nu(1) = 0;
			nu(2) = 0;
			xs_new(0) = eta(0); 
			xs_new(1) = eta(1); 
			xs_new(2) = eta(2);
			xs_new(3) = nu(0);  
			xs_new(4) = nu(1);  
			xs_new(5) = nu(2);
		}
		case ERK1 : 
		{
			update_Cvv(nu); 
			update_Dvv(nu);
			eta = eta + dt * Utilities::rotate_vector_3D(nu, eta(2), Yaw);
			nu  = nu  + dt * M_inv * (- Cvv - Dvv + tau);
			xs_new(0) = eta(0); 
			xs_new(1) = eta(1); 
			xs_new(2) = Utilities::wrap_angle_to_pmpi(eta(2));
			xs_new(3) = nu(0);  
			xs_new(4) = nu(1);  
			xs_new(5) = nu(2);
		}
		default :
		{
			std::cout << "The prediction method does not exist or is not implemented" << std::endl;
			xs_new.setZero(); 
		}
	}
}

/****************************************************************************************
*  Name     : predict_trajectory
*  Function : Predicts the ownship trajectory for a sequence of avoidance maneuvers in the 
*			  offset sequence.
*  Author   : 
*  Modified :
*****************************************************************************************/
void Ownship::predict_trajectory(
	Eigen::Matrix<double, 6, -1> &trajectory, 					// In/out: Predicted ownship trajectory
	const Eigen::VectorXd offset_sequence, 							// In: Sequence of offsets in the candidate control behavior
	const Eigen::VectorXd maneuver_times,							// In: Time indices for each ownship avoidance maneuver
	const double u_d, 												// In: Surge reference
	const double psi_d, 											// In: Heading reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 					// In: Ownship waypoints
	const Prediction_Method prediction_method,						// In: Type of prediction method to be used, typically an explicit method
	const Guidance_Method guidance_method, 							// In: Type of guidance to be used
	const double T,													// In: Prediction horizon
	const double dt 												// In: Prediction time step
	)
{
	int n_samples = T / dt;
	
	trajectory.resize(6, n_samples);

	wp_counter = 0;
	int man_count = 0;
	double u_m = 1, u_d_p = u_d;
	double chi_m = 0, psi_d_p = psi_d;
	Eigen::Matrix<double, 6, 1> xs = trajectory.block<6, 1>(0, 0);

	for (int k = 0; k < n_samples; k++)
	{ 
		if (k == maneuver_times[man_count]){
			u_m = offset_sequence[2 * man_count];
			chi_m = offset_sequence[2 * man_count + 1];
			if (man_count < maneuver_times.size()) man_count += 1;
		}  

		update_guidance_references(u_d_p, psi_d_p, waypoints, xs, k, dt, guidance_method);

		update_ctrl_input(u_m * u_d_p, chi_m + psi_d_p, xs);

		xs = predict(xs, dt, prediction_method);
		trajectory.block<6, 1>(0, k) = xs;
	}
}


/****************************************************************************************
		Private functions
*****************************************************************************************/

/****************************************************************************************
*  Name     : calculate_position_offsets
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Ownship::calculate_position_offsets(){
	x_offset = A - B;
	y_offset = D - C;
}

/****************************************************************************************
*  Name     : Cvv
*  Function : Calculates the "coriolis vector" for the 3DOF surface vessel based on 
*			  Fossen 2011. Use the equations for C_RB and C_A in Section 7.1
*  Author   : 
*  Modified :
*****************************************************************************************/
inline void Ownship::update_Cvv(
	const Eigen::Vector3d nu 									// In: BODY velocity vector nu = [u, v, r]^T				
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
inline void Ownship::update_Dvv(
	const Eigen::Vector3d nu 									// In: BODY velocity vector nu = [u, v, r]^T
	)
{
	Dvv(0) = - (X_u + 						+  X_uu * fabs(nu(0))  		  + X_uuu * nu(0) * nu(0)) * nu(0);
	Dvv(1) = - ((Y_v * nu(1) + Y_r * nu(2)) + (Y_vv * fabs(nu(1)) * nu(1) + Y_vvv * nu(1) * nu(1) * nu(1)));
	Dvv(2) = - ((N_v * nu(1) + N_r * nu(2)) + (N_rr * fabs(nu(2)) * nu(2) + N_rrr * nu(2) * nu(2) * nu(2)));
}

/****************************************************************************************
*  Name     : update_guidance_references 
*  Function : Using LOS guidance
*  Author   : 
*  Modified :
*****************************************************************************************/
void Ownship::update_guidance_references(
	double &u_d,												// Out: Surge reference
	double &psi_d,												// Out: Heading reference (set equal to course reference, compensating for crab angle through LOS_K_i if at all..)
	const Eigen::Matrix<double, 2, -1> &waypoints,				// In: Waypoints to follow.
	const Eigen::Matrix<double, 6, 1> &xs, 						// In: Ownship state
	const int k, 												// In: Time step index		
	const double dt, 											// In: Time step
	const Guidance_Method guidance_method						// In: Type of guidance used	
	)
{
	int n_wps = waypoints.cols();
	double alpha, e;

	// Check if last WP segment is passed. If so, hold the current heading/course
	if (wp_counter == n_wps - 1)
	{
		psi_d = xs(2); 
		return;
	}

	// Determine if a switch must be made to the next waypoint segment
	Eigen::Vector2d d_next_wp;
	d_next_wp(0) = waypoints(0, wp_counter + 1) - xs(0);
	d_next_wp(1) = waypoints(1, wp_counter + 1) - xs(1);
	if (d_next_wp.norm() <= R_a)
	{
		e_int = 0;
		wp_counter += 1;
	} 

	switch (guidance_method)
	{
		case LOS : 
		{
			// Compute path tangential angle
			alpha = atan2(waypoints(1, wp_counter + 1) - waypoints(1, wp_counter), 
						  waypoints(0, wp_counter + 1) - waypoints(0, wp_counter));

			//s =   (xs(0) - waypoints(0, wp_counter)) * cos(alpha) + (xs(1) - waypoints(1, wp_counter)) * sin(alpha);
			e = - (xs(0) - waypoints(0, wp_counter)) * sin(alpha) + (xs(1) - waypoints(1, wp_counter)) * cos(alpha);

			e_int += e * dt;
			if (e_int >= e_int_max) e_int -= e * dt;

			psi_d = alpha + atan2( - (e + LOS_K_i * e_int), LOS_LD);
		}
		case WPP :
		{
			d_next_wp(0) = waypoints(0, wp_counter + 1) - xs(0);
			d_next_wp(1) = waypoints(1, wp_counter + 1) - xs(1);
			psi_d = atan2(d_next_wp(1), d_next_wp(0));
		}
		case CH :
		{
			psi_d = xs(2);
		}
		default : 
		{
			std::cout << "This guidance method does not exist or is not implemented" << std::endl;
		}
	}
	psi_d = Utilities::wrap_angle_to_pmpi(psi_d);
}

/****************************************************************************************
*  Name     : update_ctrl_input
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Ownship::update_ctrl_input(
	const double u_d,											// In: Surge reference
	const double psi_d, 										// In: Heading reference
	const Eigen::Matrix<double, 6, 1> &xs 						// In: State
	)
{
	double Fx = Cvv(0) + Dvv(0) + Kp_u * m * (u_d - xs(3));

	double Fy = (Kp_psi * I_z ) * ((psi_d - xs(2)) - Kd_psi * xs(5));
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