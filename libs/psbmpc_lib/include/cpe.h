/****************************************************************************************
*
*  File name : cpe.h
*
*  Function  : Header file for the Collision Probability Estimator (CPE)
*
*  
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

#include <Eigen/Dense>
#include <random>

// See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" or 
// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
enum CPE_Method 
{
	CE,														// Consider positional uncertainty only
	MCSKF4D													// Consider uncertainty in both position and velocity along piece-wise linear segments 
};

class CPE
{
private:

	// Number of samples drawn
	int n_CE, n_MCSKF;

	// PRNG-related
	std::random_device seed;
	std::mt19937 gen{seed()};

	std::normal_distribution<double> std_norm_pdf{0.0, 1.0};

	// CE-method parameters
	double sigma_inject, alpha_n, alpha_p, rho, max_it;
	
	bool converged_last;

	Eigen::Vector2d mu_CE_last, P_CE_last;

	// MCSKF4D-method parameters
	double q, p, dt_seg; 
	
	double P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Safety zone parameters
	double d_safe;

	double norm_pdf_log(const Eigen::VectorXd &xs, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	double produce_MCS_estimate(
		const Eigen::VectorXd &xs, 
		const Eigen::MatrixXd &P, 
		const Eigen::Vector2d &p,
		const double t);

	bool check_sample_validity_4D(const Eigen::MatrixXd samples, const Eigen::Vector2d p_OS, const double t);


public:

	CPE(const int n_CE, const int n_MCSKF, const double d_safe);

	void set_safety_zone_radius(const double d_safe) { this->d_safe = d_safe; };

	void initialize(const Eigen::MatrixXd &xs_A, const Eigen::MatrixXd &P_A, const Eigen::MatrixXd &xs_B, const Eigen::MatrixXd &P_B);

	double estimate(
		const Eigen::MatrixXd &xs_A, 
		const Eigen::MatrixXd &P_A, 
		const Eigen::MatrixXd &xs_B, 
		const Eigen::MatrixXd &P_B,
		const CPE_Method cpe_method);

};

#endif
