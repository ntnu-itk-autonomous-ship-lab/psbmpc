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

#include "cml.cuh"
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

	// Model parameters
	double T_U, T_chi;

	// Guidance parameters
	double e_int, e_int_max; 
	double R_a;
	double LOS_LD, LOS_K_i;

	// Counter variables to keep track of the active WP segment at the current 
	// time and predicted time
	int wp_c_0, wp_c_p;

public:

	__host__ __device__ Obstacle_Ship();

	__host__ __device__ void determine_active_waypoint_segment(const CML::MatrixXd &waypoints, const CML::MatrixXd &xs);

	__host__ __device__ void update_guidance_references(
		double &u_d, 
		double &chi_d, 
		const CML::MatrixXd &waypoints, 
		const CML::MatrixXd &xs,
		const double dt,
		const Guidance_Method guidance_method);

	__host__ __device__ CML::MatrixXd predict(
		const CML::MatrixXd &xs_old, 
		const double U_d,
		const double chi_d,
		const double dt, 
		const Prediction_Method prediction_method);

	__host__ __device__ void predict_trajectory(
		CML::MatrixXd &trajectory,
		const CML::MatrixXd &offset_sequence,
		const CML::MatrixXd &maneuver_times,
		const double u_d,
		const double chi_d,
		const CML::MatrixXd &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const double T,
		const double dt);

};

#endif