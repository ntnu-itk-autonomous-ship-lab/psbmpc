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


#ifndef _OBSTACLE_SHIP_H_
#define _OBSTACLE_SHIP_H_

#include "Eigen/Dense"
#include <thrust/device_vector.h>

enum Prediction_Method {
	Linear,													// Linear prediction
	ERK1, 													// Explicit Runge Kutta 1 = Eulers method
	ERK4 													// Explicit Runge Kutta of fourth order, not implemented.
};

enum Guidance_Method {
	LOS, 													// Line-of-sight		
	WPP,													// WP-Pursuit
	CH 														// Course Hold
};

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

	__device__ void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Vector4d &xs);

	__device__ void update_guidance_references(
		double &u_d, 
		double &chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Vector4d &xs,
		const double dt,
		const Guidance_Method guidance_method);

	__device__ Eigen::Vector4d predict(
		const Eigen::Vector4d &xs_old, 
		const double U_d,
		const double chi_d,
		const double dt, 
		const Prediction_Method prediction_method);

	__device__ void predict_trajectory(
		Eigen::Matrix<double, 4, -1> &trajectory,
		const Eigen::VectorXd offset_sequence,
		const Eigen::VectorXd maneuver_times,
		const double u_d,
		const double chi_d,
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const double T,
		const double dt);

};

#endif