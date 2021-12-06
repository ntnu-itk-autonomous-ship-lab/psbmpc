/****************************************************************************************
*
*  File name : kinetic_ship_models_gpu.cuh
*
*  Function  : Header file for the GPU used kinetic ship model(s).
*			   Facilitates Guidance, Navigation and Control (GNC) of a surface vessel
*			   Uses mainly Eigen for matrix functionality.
*
*			   Implements a base Ship class, on which (atm) 2 derived variants are
*			   implemented.
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
		class Kinetic_Ship_Base_3DOF
		{
		protected:
			// Control input vector
			TML::Vector3f tau;

			// Inertia matrix, sum of the rigid body mass matrix M_RB
			// and the added mass matrix M_A
			TML::Matrix3f M, M_inv;

			// Vector results of performing C(v)*v and D(v)*v
			TML::Vector3f Cvv, Dvv;

			// Matrix of linear damping coefficients
			/* 	  {X_u, X_v, X_r}
			D_l = {Y_u, Y_v, Y_r}
				  {N_u, N_v, N_r} */
			TML::Matrix3f D_l;

			// Nonlinear damping terms
			float X_uu, X_uuu;
			float Y_vv, Y_vr, Y_rv, Y_rr, Y_vvv;
			float N_vv, N_vr, N_rv, N_rr, N_rrr;

			// Model parameters
			float A, B, C, D, l, w; // Ship dimension headers, and length/width
			float x_offset, y_offset;

			// Guidance parameters
			float e_int, e_int_max;
			float R_a;
			float LOS_LD, LOS_K_i;

			// Counter variables to keep track of the active WP segment at the current
			// time and predicted time
			int wp_c_0, wp_c_p;

			//Force limits
			float Fx_min;
			float Fx_max;
			float Fy_min;
			float Fy_max;

			//===================================
			// Pre-allocated temporaries
			int n_samples, n_wps, man_count;
			float u_m, u_d_p, chi_m, chi_d_p, alpha, e;

			TML::Vector2f d_next_wp, L_wp_segment, v_p;
			bool segment_passed, inside_wp_R_a;

			TML::Vector6f xs_new, xs_p;
			TML::Vector3f eta, nu;

			float Fx, Fy, psi_diff;
			//===================================

			// Calculates the offsets according to the position of the GPS receiver
			__host__ __device__ inline void calculate_position_offsets() { x_offset = A - B; y_offset = D - C; };

			__host__ __device__ void update_Cvv(const TML::Vector3f &nu);

			__host__ __device__ void update_Dvv(const TML::Vector3f &nu);

		public:

			__host__ __device__ Kinetic_Ship_Base_3DOF();

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

			__host__ __device__ inline void initialize_wp_following() { wp_c_p = wp_c_0; }

			__host__ __device__ TML::Vector6f predict(const TML::Vector6f &xs_old, const float dt, const Prediction_Method prediction_method);

			__host__ Eigen::Matrix<double, 6, 1> predict(const Eigen::Matrix<double, 6, 1> &xs_old, const double dt, const Prediction_Method prediction_method);

			__host__ __device__ inline void set_wp_counter(const int wp_c_0) { this->wp_c_0 = wp_c_0; this->wp_c_p = wp_c_0; }

			__host__ __device__ inline int get_wp_counter() const { return wp_c_0; }

			__host__ __device__ inline float get_length() const { return l; }

			__host__ __device__ inline float get_width() const { return w; }

		};

		class Telemetron : public Kinetic_Ship_Base_3DOF
		{
		private:
			// Specific model parameters used for control
			float m, I_z;
			float l_r; // distance from CG to rudder

			// Controller parameters
			float Kp_u, Kp_psi, Kd_psi, Kp_r;

			float r_max;

            //Force limits
			float Fx_min, Fx_max, Fy_min, Fy_max;

		public:
			Telemetron();

			__host__ __device__ void update_ctrl_input(const float u_d, const float psi_d, const TML::Vector6f &xs);

			__host__ void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

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
				const float dt);

			__host__ __device__ void predict_trajectory(
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
				const TML::PDVector6f &ownship_state,
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

		// Default ownship type is Kinematic_Ship
		#if OWNSHIP_TYPE == 1
			using Ownship = Telemetron;
		#endif
	}
}
