/****************************************************************************************
*
*  File name : obstacle_ship_cpu.h
*
*  Function  : Header file for the CPU used simple kinematic model based obstacle ship, 
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


#pragma once

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

namespace PSBMPC_LIB
{
	using Obstacle_Ship =
	namespace CPU
	{		
		class Obstacle_Ship
		{
		private:

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

		public:

			Obstacle_Ship();

			Obstacle_Ship(const double T_U, const double  T_chi, const double R_a, const double LOS_LD, const double LOS_K_i);

			double get_length() const { return l; }

			double get_width() const { return w; }

			void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Vector4d &xs);

			void update_guidance_references(
				double &u_d, 
				double &chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const Eigen::Vector4d &xs,
				const double dt,
				const Guidance_Method guidance_method);

			Eigen::Vector4d predict(
				const Eigen::Vector4d &xs_old, 
				const double U_d,
				const double chi_d,
				const double dt, 
				const Prediction_Method prediction_method);

			void predict_trajectory(
				Eigen::Matrix<double, 4, -1> &trajectory,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const double T,
				const double dt);

		};
	}
}