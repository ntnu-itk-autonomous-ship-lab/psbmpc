/****************************************************************************************
*
*  File name : cpe.h
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

#ifndef _CPE_H_
#define _CPE_H_

#include "xoshiro.hpp"
#include <Eigen/Dense>

#include <random>

// NOTE: If you want standalone use of this module, define the enum CPE_Method below
/* // See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" and/or 
// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
enum CPE_Method 
{
	CE,														// Consider positional uncertainty only
	MCSKF4D													// Consider uncertainty in both position and velocity along piece-wise linear segments 
}; */
// Otherwise, for usage with the PSB-MPC, include "psbmpc_parameters.h":
#include "psbmpc_parameters.h"

class CPE
{
private:

	// Active CPE method
	CPE_Method method;

	// Number of obstacles to consider in estimation
	int n_obst;

	// Number of samples drawn
	int n_CE, n_MCSKF;

	// PRNG-related
	// IMPORTANT NOTE: Some compilers (Older GCC versions < 9.2) implement this 
	// random device using a PRNG, and if a non-deterministic device is not available
	// => the same sequence is produced every time, this should be checked before
	// real-time testing to ensure proper functionality.
	std::random_device seed;

	xoshiro256plus64 generator;

	std::normal_distribution<double> std_norm_pdf;

	// CE-method parameters and internal states
	double sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	std::vector<Eigen::Vector2d> mu_CE_last;
	std::vector<Eigen::Matrix2d> P_CE_last;

	int N_e, e_count;
	Eigen::MatrixXd elite_samples;

	// MCSKF4D-method parameters and internal states
	double q, r, dt_seg; 
	
	Eigen::VectorXd P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Common internal sample variables
	Eigen::MatrixXd samples;
	Eigen::VectorXd valid;
	
	// Safety zone parameters
	double d_safe;

	// Cholesky decomposition matrix
	Eigen::MatrixXd L;

	void assign_data(const CPE &cpe);

	void resize_matrices();

	inline void update_L(const Eigen::MatrixXd &in);

	inline double calculate_2x2_quadratic_form(const Eigen::Vector2d &x, const Eigen::Matrix2d &A);

	inline void norm_pdf_log(Eigen::VectorXd &result, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	inline void generate_norm_dist_samples(const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	void calculate_roots_2nd_order(Eigen::Vector2d &r, bool &is_complex, const double A, const double B, const double C);

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
		const Eigen::MatrixXd &P_i,
		const int i);	

	void determine_sample_validity_2D(
		const Eigen::Vector2d &p_os);

	void determine_best_performing_samples(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i);

	double CE_estimation(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i,
		const int i);

public:

	CPE(const CPE_Method cpe_method, const int n_CE, const int n_MCSKF, const int n_obst, const double dt);

	CPE(const CPE &cpe);

	CPE& operator=(const CPE &cpe);

	void set_method(const CPE_Method cpe_method) { if (cpe_method >= CE && cpe_method <= MCSKF4D) { method = cpe_method;  resize_matrices(); }};

	void set_number_of_obstacles(const int n_obst);

	double get_segment_discretization_time() const { return dt_seg; };

	void initialize(
		const Eigen::Matrix<double, 6, 1> &xs_os, 
		const Eigen::Vector4d &xs_i, 
		const Eigen::VectorXd &P_i,
		const double d_safe_i, 
		const int i);
	
	double estimate(
		const Eigen::MatrixXd &xs_os,
		const Eigen::MatrixXd &xs_i,
		const Eigen::MatrixXd &P_i,
		const int i);

	void estimate_over_trajectories(
		Eigen::Matrix<double, 1, -1> &P_c_i,
		const Eigen::Matrix<double, 6, -1> &xs_p,
		const Eigen::Matrix<double, 4, -1> &xs_i_p,
		const Eigen::Matrix<double, 16, -1> &P_i_p,
		const double d_safe_i,
		const int i,
		const double dt);
};

#endif