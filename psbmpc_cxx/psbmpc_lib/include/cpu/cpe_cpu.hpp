/****************************************************************************************
*
*  File name : cpe_cpu.hpp
*
*  Function  : Header file for the Collision Probability Estimator (CPE).
*			   The module estimates the collision probability wrt all nearby
*			   obstacles. The module assumes that the own-ship uncertainty is negligible
*  			   compared to that of the obstacles. If this is not the case, then a
*			   linear transformationcan be used to "gather" both vessel's 
*			   uncertainty in one RV.
*            ---------------------
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
#include "xoshiro.hpp"

#include <random>
#include <Eigen/Dense>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class CPE;
	}
	
	namespace CPU
	{
		class CPE
		{
		private:

			friend class GPU::CPE;
			
			// Active CPE method
			CPE_Method method;

			// Number of samples drawn
			int n_CE, n_MCSKF;

			// PRNG-related
			// IMPORTANT NOTE: Some compilers might implement this 
			// random device using a PRNG, and if a non-deterministic device is not available
			// => the same sequence is produced every time, this should be checked before
			// real-time testing to ensure proper functionality.
			std::random_device seed;

			xoshiro256plus64 generator;

			std::normal_distribution<double> std_norm_pdf;

			//====================================
			// CE-method parameters, internal states and temporaries
			double sigma_inject, alpha_n, gate, rho;
			int max_it;
			
			bool converged_last;

			Eigen::Vector2d mu_CE_last;
			Eigen::Matrix2d P_CE_last;

			int N_e, e_count;
			Eigen::MatrixXd elite_samples;

			// Temporaries between dashed lines
			//--------------------------------------------
			Eigen::Vector2d mu_CE_prev, mu_CE;
			Eigen::Matrix2d P_CE_prev, P_CE;
			Eigen::Matrix2d P_i_inv;

			double d_0i, var_P_i_largest;
			bool inside_safety_zone, inside_alpha_p_confidence_ellipse;

			Eigen::VectorXd weights, integrand, importance;
			//-------------------------------------------
			//====================================

			//====================================
			// MCSKF4D-method parameters, internal states and temporaries
			double q, r, dt_seg; 
			double P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

			// Temporaries between dashed lines
			//--------------------------------------------
			double y_P_c_i; 
			
			double t_cpa, d_cpa, K;
			Eigen::Vector2d p_os_cpa;

			int n_seg_samples;

			// Speed and course/heading for the vessels along their linear segments
			double U_os_sl, U_i_sl, psi_os_sl, psi_i_sl;

			Eigen::Vector4d xs_os_sl, xs_i_sl;
			Eigen::Matrix4d P_i_sl;

			bool complex_roots;

			Eigen::Vector2d roots, p_i_sample, v_i_sample;
			double d, A, B, C, constant;
			//--------------------------------------------
			//====================================

			// Common internal sample variables
			Eigen::MatrixXd samples;
			Eigen::VectorXd valid;
			
			// Safety zone parameters
			double d_safe;

			// Cholesky decomposition matrix
			Eigen::MatrixXd L;

			//====================================
			// Other pre-allocated temporaries:
			double P_c_est, P_c_CE, y_P_c, sum;
			int n, n_samples, n_samples_traj, n_cols, k_j, k_j_, sample_count;	
			Eigen::MatrixXd Sigma_inv;

			double exp_val, log_val;

			Eigen::MatrixXd xs_os_seg, xs_i_seg, P_i_seg;
			Eigen::Matrix2d P_i_2D;
			Eigen::Vector2d v_os_prev, v_i_prev;
			//====================================
			void assign_data(const CPE &cpe);

			void resize_matrices();

			inline void update_L(const Eigen::MatrixXd &in);

			inline void norm_pdf_log(Eigen::VectorXd &result, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

			inline void generate_norm_dist_samples(const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

			void calculate_roots_2nd_order();

			double produce_MCS_estimate(
				const Eigen::Vector4d &xs_i, 
				const Eigen::Matrix4d &P_i, 
				const Eigen::Vector2d &p_os_cpa,
				const double t_cpa);

			void determine_sample_validity_4D(
				const Eigen::Vector2d &p_os_cpa, 
				const double t_cpa);

			double MCSKF4D_estimation(
				const Eigen::MatrixXd &xs_os,  
				const Eigen::MatrixXd &xs_i, 
				const Eigen::MatrixXd &P_i);	

			void determine_sample_validity_2D(
				const Eigen::Vector2d &p_os);

			void determine_best_performing_samples(
				const Eigen::Vector2d &p_os, 
				const Eigen::Vector2d &p_i, 
				const Eigen::Matrix2d &P_i);

			void update_importance_density();

			double CE_estimation(
				const Eigen::Vector2d &p_os, 
				const Eigen::Vector2d &p_i, 
				const Eigen::Matrix2d &P_i,
				const Eigen::Vector2d &v_os_prev,
				const Eigen::Vector2d &v_i_prev, 
				const double dt);

		public:

			CPE(const CPE_Method cpe_method,
				const int n_CE, 
				const int n_MCSKF, 
				const double alpha_n, 
				const double gate, 
				const double rho, 
				const double max_it,
				const double q,
				const double r);
			CPE(const CPE_Method cpe_method);

			CPE(const CPE &other);

			CPE& operator=(const CPE &rhs);

			void set_method(const CPE_Method cpe_method) { if (cpe_method >= CE && cpe_method <= MCSKF4D) { method = cpe_method;  resize_matrices(); }};

			inline CPE_Method get_method() const { return method; };

			void set_segment_discretization_time(const double dt_seg) { this->dt_seg = dt_seg; };

			double get_segment_discretization_time() const { return dt_seg; };

			void initialize(
				const Eigen::VectorXd &xs_os, 
				const Eigen::Vector4d &xs_i, 
				const double d_safe_i);

			void estimate_over_trajectories(
				Eigen::Matrix<double, 1, -1> &P_c_i,
				const Eigen::MatrixXd &xs_p,
				const Eigen::Matrix<double, 4, -1> &xs_i_p,
				const Eigen::Matrix<double, 16, -1> &P_i_p,
				const double d_safe_i,
				const double dt,
				const int p_step);

			Eigen::Matrix<double, 1, -1> estimate_over_trajectories_py(
				Eigen::Matrix<double, 1, -1> &P_c_i,
				const Eigen::MatrixXd &xs_p,
				const Eigen::Matrix<double, 4, -1> &xs_i_p,
				const Eigen::Matrix<double, 16, -1> &P_i_p,
				const double d_safe_i,
				const double dt,
				const int p_step);
		};
	}
}