/****************************************************************************************
*
*  File name : kinematic_ship_models_cpu.hpp
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

#include "psbmpc_parameters.hpp"

namespace PSBMPC_LIB
{
	namespace CPU
	{			
		class Kinematic_Ship
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

			Kinematic_Ship();

			Kinematic_Ship(const double T_U, const double  T_chi, const double R_a, const double LOS_LD, const double LOS_K_i);

			double get_length() const { return l; }

			double get_width() const { return w; }

			inline void set_wp_counter(const int wp_c_0) { this->wp_c_0 = wp_c_0; this->wp_c_p = wp_c_0; }

			void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Vector4d &xs);

			void update_guidance_references(
				double &u_d, 
				double &chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const Eigen::Vector4d &xs,
				const double dt,
				const Guidance_Method guidance_method);

			void update_guidance_references(
				double &u_d, 
				double &chi_d, 
				const double e_m,
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const Eigen::Vector4d &xs,
				const double dt);

			Eigen::Vector4d predict(
				const Eigen::Vector4d &xs_old, 
				const double U_d,
				const double chi_d,
				const double dt, 
				const Prediction_Method prediction_method);

			void predict_trajectory(
				Eigen::MatrixXd &trajectory,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const double T,
				const double dt);

			void predict_trajectory(
				Eigen::MatrixXd &trajectory,
				const double e_m,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const double T,
				const double dt);
		};

		// The default ownship is the Kinematic_Ship class
		#if OWNSHIP_TYPE == 0
			using Ownship = Kinematic_Ship;
		#endif

		// The default obstacle ship is the Kinematic_Ship class
		using Obstacle_Ship = Kinematic_Ship;	
	}
}