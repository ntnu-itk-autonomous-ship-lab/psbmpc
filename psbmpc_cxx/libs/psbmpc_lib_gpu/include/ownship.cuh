/****************************************************************************************
*
*  File name : ownship.cuh
*
*  Function  : Header file for the ownship class. Modified and extended version of the
*			   "Ship_Model" class created for SBMPC by Inger Berge Hagen and Giorgio D. 
*			   Kwame Minde Kufoalor through the Autosea project. Facilitates Guidance,
*			   Navigation and Control (GNC) of a surface vessel 
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


#ifndef _OWNSHIP_CUH_
#define _OWNSHIP_CUH_

#include "psbmpc_defines.h"
#include <thrust/device_vector.h>
#include "tml.cuh"
#include "Eigen/Dense"

// NOTE: If you want standalone use of this module, define the enums Prediction_Method and Guidance_Method below
/* enum Prediction_Method
{
	Linear,													// Linear prediction
	ERK1, 													// Explicit Runge Kutta 1 = Eulers method
	ERK4 													// Explicit Runge Kutta of fourth order, not implemented yet nor needed.
};

enum Guidance_Method 
{
	LOS, 													// Line-of-sight		
	WPP,													// Waypoint-Pursuit
	CH 														// Course Hold
}; */
// Otherwise, for usage with the PSB-MPC, include "psbmpc_parameters.h":
#include "psbmpc_parameters.h"

class Ownship
{
private:

	TML::Vector3f tau;
	TML::Matrix3f M_inv;
	TML::Vector3f Cvv;
	TML::Vector3f Dvv;
	
	// Counter variables to keep track of the active WP segment at the current 
	// time and predicted time
	int wp_c_0, wp_c_p;

	// Model Parameters
	float l_r;
	float m; 	// [kg]
	float I_z; // [kg/m2]

	// Added mass terms
	float X_udot;
	float Y_vdot, Y_rdot;
	float N_vdot, N_rdot;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	float X_u;
	float Y_v, Y_r;
	float N_v, N_r;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	float X_uu;
	float Y_vv;
	float N_rr;
	float X_uuu;
	float Y_vvv;
	float N_rrr;

	//Force limits
	float Fx_min;
	float Fx_max;
	float Fy_min;
	float Fy_max; 

	// Guidance parameters
	float e_int, e_int_max; 
	float R_a;
	float LOS_LD, LOS_K_i;

	// Controller parameters
	float Kp_u;
	float Kp_psi;
	float Kd_psi;
	float Kp_r;
	
	float r_max; 

	float A, B, C, D, l, w;
	float x_offset, y_offset;

	//===================================
	// Pre-allocated temporaries
	int n_samples, n_wps, man_count;
	float u_m, u_d_p, chi_m, chi_d_p, alpha, e;

	TML::Vector2f d_next_wp, L_wp_segment, v_p;
	bool segment_passed;

	TML::Vector6f xs_new, xs_p;
	TML::Vector3f eta, nu;

	float Fx, Fy, psi_diff;
	//===================================

	// Calculates the offsets according to the position of the GPS receiver
	__host__ __device__ inline void calculate_position_offsets() { x_offset = A - B; y_offset = D - C; };

	__host__ __device__ void update_Cvv(const TML::Vector3f &nu);

	__host__ __device__ void update_Dvv(const TML::Vector3f &nu);

public:

	__host__ __device__ Ownship();

	__host__ void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Matrix<double, 6, 1> &xs);

	__host__ __device__ void determine_active_waypoint_segment(const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, const TML::Vector6f &xs);

	__host__ __device__ void update_guidance_references(
		float &u_d, 
		float &chi_d, 
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, 
		const TML::Vector6f &xs,
		const float dt,
		const Guidance_Method guidance_method);

	__host__ void update_guidance_references(
		double &u_d, 
		double &chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 6, 1> &xs,
		const double dt,
		const Guidance_Method guidance_method);

	__host__ __device__ void update_ctrl_input(const float u_d, const float psi_d, const TML::Vector6f &xs);
	
	__host__ void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

	__host__ __device__ inline void initialize_wp_following() { wp_c_p = wp_c_0; }

	__host__ __device__ TML::Vector6f predict(const TML::Vector6f &xs_old, const float dt, const Prediction_Method prediction_method);

	__host__ Eigen::Matrix<double, 6, 1> predict(const Eigen::Matrix<double, 6, 1> &xs_old, const double dt, const Prediction_Method prediction_method);
	
	__host__ __device__ void predict_trajectory(
		TML::PDMatrix<float, 6, MAX_N_SAMPLES> &trajectory,
		const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,
		const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,
		const float u_d,
		const float chi_d,
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const float T,
		const float dt
	);

	__host__ __device__ void predict_trajectory(
		TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
		const TML::Vector6f &ownship_state,
		const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,
		const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,
		const float u_d,
		const float chi_d,
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const float T,
		const float dt
	);

	__host__ void predict_trajectory(
		Eigen::Matrix<double, 6, -1> &trajectory,
		const Eigen::VectorXd &offset_sequence,
		const Eigen::VectorXd &maneuver_times,
		const double u_d,
		const double chi_d,
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const double T,
		const double dt
	);

	__host__ __device__ inline void set_wp_counter(const int wp_c_0) { this->wp_c_0 = wp_c_0; }

	__host__ __device__ inline int get_wp_counter() const { return wp_c_0; }

	__host__ __device__ inline float get_length() const { return l; }

	__host__ __device__ inline float get_width() const { return w; }

};

#endif
