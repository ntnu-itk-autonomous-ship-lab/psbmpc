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

#include "psbmpc_defines.hpp"
#include "psbmpc_parameters.hpp"

#include <Eigen/Dense>

namespace PSBMPC_LIB
{

	enum Path_Prediction_Shape
	{
		SMOOTH, // Standard implementation of the predicted paths in the PSB-MPC. The generated shapes resembles a U (for one seq. avoidance man.)
		LINEAR  // "Piecewise-linear" prediction geometry. The generated shapes resembles a V (for one seq. avoidance man.)
	};

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

			// The shape of the predicted path for the ship
			Path_Prediction_Shape path_prediction_shape;

		public:
			Kinematic_Ship();

			//Kinematic_Ship(const Kinematic_Ship &other) = default;

			Kinematic_Ship(
				const double l, 
				const double w, 
				const double T_U, 
				const double T_chi, 
				const double R_a, 
				const double LOS_LD, 
				const double LOS_K_i);

			Kinematic_Ship(
				const double l, 
				const double w, 
				const double T_U, 
				const double T_chi, 
				const double R_a, 
				const double LOS_LD, 
				const double LOS_K_i,
				const Path_Prediction_Shape path_prediction_shape);

			Kinematic_Ship(const Kinematic_Ship &other);
			
			inline double get_length() const { return l; };

			inline double get_width() const { return w; };

			inline double get_T_U() const { return T_U; };

			inline double get_T_chi() const { return T_chi; };

			inline double get_R_a() const { return R_a; };

			inline double get_LOS_LD() const { return LOS_LD; };

			inline double get_LOS_K_i() const { return LOS_K_i; };

			inline int get_wp_counter() const { return wp_c_0; };

			inline Path_Prediction_Shape get_path_prediction_shape() const { return path_prediction_shape; };

			inline void set_length(const double l) { this->l = l; };

			inline void set_width(const double w) { this->w = w; };

			inline void set_T_U(const double T_U) { this->T_U = T_U; };

			inline void set_T_chi(const double T_chi) { this->T_chi = T_chi; };

			inline void set_R_a(const double R_a) { this->R_a = R_a; };

			inline void set_LOS_LD(const double LOS_LD) { this->LOS_LD = LOS_LD; };

			inline void set_LOS_K_i(const double LOS_K_i) { this->LOS_K_i = LOS_K_i; };

			inline void set_wp_counter(const int wp_c_0)
			{
				this->wp_c_0 = wp_c_0;
				this->wp_c_p = wp_c_0;
			}

			inline void set_path_prediction_shape(const Path_Prediction_Shape path_prediction_shape) { this->path_prediction_shape = path_prediction_shape; };

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
				double &cross_track_error,
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

			Eigen::Vector2d update_guidance_references_py(
				double &u_d,
				double &chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::Vector4d &xs,
				const double dt,
				const Guidance_Method guidance_method); 

			Eigen::Vector3d update_guidance_references_py(
				double &u_d,
				double &chi_d,
				double cross_track_error,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::Vector4d &xs,
				const double dt,
				const Guidance_Method guidance_method);

			Eigen::Vector2d update_guidance_references_py(
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
				double &max_cross_track_error,
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
				const double T,
				const double dt);

			void predict_trajectory(
				Eigen::MatrixXd &trajectory,
				double &max_cross_track_error,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const Path_Prediction_Shape path_prediction_shape,
				const double T,
				const double dt);

			void predict_trajectory(
				Eigen::MatrixXd &trajectory,
				const double e_m,
				const double chi_m,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Path_Prediction_Shape path_prediction_shape,
				const double T,	
				const double dt);	

			Eigen::MatrixXd predict_trajectory_py(
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

			Eigen::MatrixXd predict_trajectory_py(
				Eigen::MatrixXd &trajectory,
				double &max_cross_track_error,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const double T,
				const double dt);

			Eigen::MatrixXd predict_trajectory_py(
				Eigen::MatrixXd &trajectory,
				const double e_m,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const double T,
				const double dt);
			
			Eigen::MatrixXd predict_trajectory_py(
				Eigen::MatrixXd &trajectory,
				double &max_cross_track_error,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const Path_Prediction_Shape path_prediction_shape,
				const double T,
				const double dt);

			Eigen::MatrixXd predict_trajectory_py(
				Eigen::MatrixXd &trajectory,
				const double e_m,
				const double chi_m,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Prediction_Method prediction_method,
				const Path_Prediction_Shape path_prediction_shape,
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