/****************************************************************************************
*
*  File name : kinematic_ship_models_gpu.cuh
*
*  Function  : Header file for the GPU used simple kinematic model based obstacle ship,
*			   used as base for the obstacle collision avoidance system.
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

#pragma once

#include "psbmpc_defines.hpp"
#include "psbmpc_parameters.hpp"
#include "tml/tml.cuh"

#include <Eigen/Dense>
#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Kinematic_Ship
		{
		private:
			// Ship length and width
			float l, w;

			// Model parameters
			float T_U, T_chi;

			// Guidance parameters
			float e_int, e_int_max;
			float R_a;
			float LOS_LD, LOS_K_i;

			// Counter variables to keep track of the active WP segment at the current
			// time and predicted time
			int wp_c_0, wp_c_p;

			//===================================
			// Pre-allocated temporaries
			int n_samples, n_wps, man_count;
			float u_m, u_d_p, chi_m, chi_d_p, chi_p, U_p, alpha, e, e_k;

			TML::Vector2f d_next_wp, L_wp_segment;
			bool segment_passed, inside_wp_R_a;

			TML::Vector4f xs_p, xs_new;

			float chi_diff;
			//===================================

		public:
			__host__ __device__ Kinematic_Ship();

			__host__ __device__ Kinematic_Ship(const float l, const float w, const float T_U, const float T_chi, const float R_a, const float LOS_LD, const float LOS_K_i);

			__host__ __device__ float get_length() const { return l; }

			__host__ __device__ float get_width() const { return w; }

			__host__ __device__ inline void set_wp_counter(const int wp_c_0)
			{
				this->wp_c_0 = wp_c_0;
				this->wp_c_p = wp_c_0;
			}

			__host__ __device__ inline int get_wp_counter() const { return wp_c_0; }

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

			__host__ __device__ void update_guidance_references(
				float &u_d,
				float &chi_d,
				float &cross_track_error,
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
				const float U_d,
				const float chi_d,
				const float dt,
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
				const float dt);

			__host__ __device__ void predict_trajectory(
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
				const TML::PDVector6f &ship_state,
				const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,
				const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,
				const float u_d,
				const float chi_d,
				const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const float T,
				const float dt);

			__host__ __device__ void predict_trajectory(
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
				float &max_cross_track_error,
				const TML::PDVector6f &ship_state,
				const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence,
				const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times,
				const float u_d,
				const float chi_d,
				const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints,
				const Prediction_Method prediction_method,
				const Guidance_Method guidance_method,
				const float T,
				const float dt);

			__host__ void predict_trajectory(
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
		};

// The default ownship is the simple kinematic_ship class
#if OWNSHIP_TYPE == 0
		using Ownship = Kinematic_Ship;
#endif

		// The default obstacle ship is the Kinematic_Ship class
		using Obstacle_Ship = Kinematic_Ship;
	}
}