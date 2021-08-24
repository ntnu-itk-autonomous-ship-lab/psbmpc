/****************************************************************************************
*
*  File name : kinetic_ship_models.hpp
*
*  Function  : Header file for the CPU used kinetic ship model(s).
*			   Facilitates Guidance, Navigation and Control (GNC) of a surface vessel
*			   Uses mainly Eigen for matrix functionality.
*  			   
*			   Implements a base Ship class, on which (atm) 2 derived variants are 
*			   implemented.
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

namespace PSBMPC_LIB
{
	namespace CPU
	{
		class Kinetic_Ship_Base_3DOF
		{
		protected:
			// Control input vector
			Eigen::Vector3d tau;

			// Inertia matrix, sum of the rigid body mass matrix M_RB 
			// and the added mass matrix M_A
			Eigen::Matrix3d M;

			// Vector results of performing C(v)*v and D(v)*v
			Eigen::Vector3d Cvv, Dvv;

			// Matrix of linear damping coefficients
			/* 	  {X_u, X_v, X_r}
			D_l = {Y_u, Y_v, Y_r}
				  {N_u, N_v, N_r} */
			Eigen::Matrix3d D_l;

			// Nonlinear damping terms
			double X_uu, X_uuu;
			double Y_vv, Y_vr, Y_rv, Y_rr, Y_vvv;
			double N_vv, N_vr, N_rv, N_rr, N_rrr;

			// Model parameters
			double A, B, C, D, l, w; // Ship dimension headers, and length/width
			double x_offset, y_offset;

			// Guidance parameters
			double e_int, e_int_max; 
			double R_a;
			double LOS_LD, LOS_K_i;

			// Counter variables to keep track of the active WP segment at the current 
			// time and predicted time
			int wp_c_0, wp_c_p;

			// Calculates the offsets according to the position of the GPS receiver
			inline void calculate_position_offsets() { x_offset = A - B; y_offset = D - C; };

			void update_Cvv(const Eigen::Vector3d &nu);

			void update_Dvv(const Eigen::Vector3d &nu);

		public:
			Kinetic_Ship_Base_3DOF();

			void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Matrix<double, 6, 1> &xs);

			void update_guidance_references(
				double &u_d, 
				double &chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const Eigen::Matrix<double, 6, 1> &xs,
				const double dt,
				const Guidance_Method guidance_method);

			inline double get_length() const { return l; }

			inline double get_width() const { return w; }

			inline void set_wp_counter(const int wp_c_0) { this->wp_c_0 = wp_c_0; this->wp_c_p = wp_c_0; }

		};

		class Telemetron : public Kinetic_Ship_Base_3DOF
		{
		private:
			// Specific model parameters used for control
			double m, I_z;
			double l_r; // distance from CG to rudder

			// Controller parameters
			double Kp_u;
			double Kp_psi;
			double Kd_psi;
			double Kp_r;
			
			double r_max; 

            //Force limits
			double Fx_min;
			double Fx_max;
			double Fy_min;
			double Fy_max; 

		public:
			Telemetron();

			void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

			Eigen::Matrix<double, 6, 1> predict(const Eigen::Matrix<double, 6, 1> &xs_old, const double dt, const Prediction_Method prediction_method);

			// Overloaded the predict function to match that of the kinematic ship model,
			// to get an equal interface in the simulations
			Eigen::Matrix<double, 6, 1> predict(
				const Eigen::Matrix<double, 6, 1> &xs_old, 
				const double u_d, 
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
		};

		// 3DOF milliampere class is NOT finished 
		class MilliAmpere : public Kinetic_Ship_Base_3DOF
		{
		private:
			double l_1, l_2; // distance from CG to front (1) and back (2) thrusters (symmetric here)

			double min_rpm, max_rpm, min_thrust, max_thrust;

			Eigen::Vector2d alpha, omega;

			Eigen::Matrix<double, 5, 1> rpm_to_force_polynomial, force_to_rpm_polynomial;

			void update_alpha();

			void update_omega();
		public:
			MilliAmpere();

			void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

			Eigen::Matrix<double, 6, 1> predict(const Eigen::Matrix<double, 6, 1> &xs_old, const double dt, const Prediction_Method prediction_method);
			
			Eigen::Matrix<double, 6, 1> predict(
				const Eigen::Matrix<double, 6, 1> &xs_old, 
				const double u_d, 
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
		};

		// Default ownship type is Kinematic_Ship
		#if OWNSHIP_TYPE == 1
			using Ownship = Telemetron;
		#elif OWNSHIP_TYPE == 2
			using Ownship = MilliAmpere;
		#endif
	}	
}
