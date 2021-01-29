/****************************************************************************************
*
*  File name : obstacle_ship.cuh
*
*  Function  : Header file for the simple kinematic model based obstacle ship, 
*			   used as base for the obstacle collision avoidance system.
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


#ifndef _OBSTACLE_SHIP_CUH_
#define _OBSTACLE_SHIP_CUH_

#include "psbmpc_defines.h"
#include "tml.cuh"
#include <thrust/device_vector.h>

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

class Obstacle_Ship
{
private:

	// Own-ship state at the predicted time
	TML::Vector4f xs_p;
	
	// Ship length and width
	double l, w;

	// Model parameters
	double T_U, T_chi;

	// Guidance parameters
	double e_int, e_int_max; 
	double R_a;
	double LOS_LD, LOS_K_i;

	// Counter variables to keep track of the active WP segment at the current 
	// time and predicted time
	int wp_c_0, wp_c_p;

	//===================================
	// Pre-allocated temporaries
	int n_samples, n_wps, man_count;
	float u_m, u_d_p, chi_m, chi_d_p, alpha, e;

	TML::Vector2d d_next_wp, L_wp_segment;
	bool segment_passed;

	TML::Vector4f xs_new;

	float chi_diff;
	//===================================

public:

	__host__ __device__ Obstacle_Ship();

	__host__ __device__ inline void initialize_wp_following() { wp_c_p = wp_c_0; }

	__host__ void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Vector4d &xs);

	__host__ __device__ void determine_active_waypoint_segment(const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, const TML::Vector4f &xs);

	__host__ __device__ void update_guidance_references(
		float &u_d, 
		float &chi_d, 
		const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints, 
		const TML::Vector4f &xs,
		const float dt,
		const Guidance_Method guidance_method);

	__host__ void update_guidance_references(
		double &u_d, 
		double &chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Vector4d &xs,
		const double dt,
		const Guidance_Method guidance_method);

	__host__ __device__ TML::Vector4f predict(
		const TML::Vector4f &xs_old, 
		const double U_d,
		const double chi_d,
		const double dt, 
		const Prediction_Method prediction_method);

	__host__ Eigen::Vector4d predict(
		const Eigen::Vector4d &xs_old,
		const double U_d,
		const double chi_d,
		const double dt, 
		const Prediction_Method prediction_method);
	
	__host__ __device__ void predict_trajectory(
		TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
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
		Eigen::Matrix<double, 4, -1> &trajectory,
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
};

#endif