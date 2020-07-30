/****************************************************************************************
*
*  File name : cpe.h
*
*  Function  : Header file for the Collision Probability Estimator (CPE).
*			   The module estimates the collision probability wrt all nearby
*			   obstacles. The module assumes that the own-ship uncertainty is negligible
*  			   compared to that of the obstacles. If this is not the case, then a
*			   linear transformationcan be used to "gather" both vessel's 
*			   uncertainty in one RV
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
#include <thrust/device_vector.h>
#include <Eigen/Dense>

#include <random>

// See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" and/or 
// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
enum CPE_Method 
{
	CE,														// Consider positional uncertainty only
	MCSKF4D													// Consider uncertainty in both position and velocity along piece-wise linear segments 
};

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
	// random device using a PRNG or if a non-deterministic device is not available
	// => the same sequence is produced every time, this should be checked before
	// real-time testing to ensure proper functionality.
	std::random_device seed;

	//std::mt19937_64 generator;
	xoshiro256plus64 generator;

	std::normal_distribution<double> std_norm_pdf;

	// CE-method parameters and states
	double sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	std::vector<Eigen::Vector2d> mu_CE_last;
	std::vector<Eigen::Matrix2d> P_CE_last;

	std::vector<int> N_e, e_count;
	std::vector<Eigen::MatrixXd> elite_samples;

	// MCSKF4D-method parameters and internal states
	double q, r, dt_seg; 
	
	Eigen::VectorXd P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Common internal sample variables
	std::vector<Eigen::MatrixXd> samples;
	std::vector<Eigen::VectorXd> valid;
	
	// Safety zone parameters
	std::vector<double> d_safe;

	// Cholesky decomposition matrix
	Eigen::MatrixXd L;

	__host__ __device__ void resize_matrices();

	__device__ inline void update_L(const Eigen::MatrixXd &in);

	__device__ inline double calculate_2x2_quadratic_form(const Eigen::Vector2d &x, const Eigen::Matrix2d &A);

	__device__ inline void norm_pdf_log(Eigen::VectorXd &result, const Eigen::MatrixXd &samples, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	__device__ inline void generate_norm_dist_samples(Eigen::MatrixXd &samples, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	__device__ void calculate_roots_2nd_order(Eigen::Vector2d &r, bool &is_complex, const double A, const double B, const double C);

	__device__ double produce_MCS_estimate(
		const Eigen::Vector4d &xs_i, 
		const Eigen::Matrix4d &P_i, 
		const Eigen::Vector2d &p_os_cpa,
		const double t_cpa,
		const int i);

	__device__ void determine_sample_validity_4D(
		Eigen::VectorXd &valid, 
		const Eigen::MatrixXd &samples, 
		const Eigen::Vector2d &p_os_cpa, 
		const double t_cpa,
		const int i );

	__device__ double MCSKF4D_estimation(
		const Eigen::MatrixXd &xs_os,  
		const Eigen::MatrixXd &xs_i, 
		const Eigen::MatrixXd &P_i,
		const int i);	

	__device__ void determine_sample_validity_2D(
		Eigen::VectorXd &valid, 
		const Eigen::MatrixXd &samples,
		const Eigen::Vector2d &p_os,
		const int i);

	__device__ void determine_best_performing_samples(
		Eigen::VectorXd &valid, 
		int &N_e, 
		const Eigen::MatrixXd &samples,
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i,
		const int i);

	__device__ double CE_estimation(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i,
		const int i);

public:

	__host__ __device__ CPE(const CPE_Method cpe_method, const int n_CE, const int n_MCSKF, const int n_obst, const double dt);

	__host__ __device__ CPE(const CPE &cpe);

	__host__ __device__ ~CPE();

	__host__ __device__ CPE& operator=(const CPE &cpe);

	__host__ __device__ void set_method(const CPE_Method cpe_method) { method = cpe_method;  resize_matrices(); };

	__device__ void set_safety_zone_radius(const double d_safe, const int i) { this->d_safe[i] = d_safe; };

	__device__ void set_safety_zone_radius(const std::vector<double> d_safe) { this->d_safe = d_safe; };

	__host__ __device__ void set_number_of_obstacles(const int n_obst);

	__device__ void initialize(
		const Eigen::Matrix<double, 6, 1> &xs_os, 
		const Eigen::Vector4d &xs_i, 
		const Eigen::VectorXd &P_i,
		const double d_safe_i, 
		const int i);
	
	__device__ double estimate(
		const Eigen::MatrixXd &xs_os,
		const Eigen::MatrixXd &xs_i,
		const Eigen::MatrixXd &P_i,
		const int i);
};

#endif