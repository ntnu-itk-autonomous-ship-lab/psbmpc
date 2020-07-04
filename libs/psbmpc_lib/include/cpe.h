/****************************************************************************************
*
*  File name : cpe.h
*
*  Function  : Header file for the Collision Probability Estimator (CPE).
*			   The module assumes that the own-ship uncertainty is negligible
*  			   compared to that of the obstacles. If this is not the case, then a
			   linear transformationcan be used to "gather" both vessel's 
			   uncertainty in one RV
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

	// Active CPE method
	CPE_Method method;

	// Number of obstacles to consider in estimation
	int n_obst;

	// Number of samples drawn
	int n_CE, n_MCSKF;

	// PRNG-related
	std::random_device seed;
	// consider other faster generators than the mersenne twister
	std::mt19937 generator;

	std::normal_distribution<double> std_norm_pdf;

	// CE-method parameters
	double sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	// Previous possibly optimal CE density parameters
	std::vector<Eigen::Vector2d> mu_CE_last;
	std::vector<Eigen::Matrix2d> P_CE_last;


	// MCSKF4D-method parameters
	double q, p, dt_seg; 
	
	Eigen::VectorXd P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Safety zone parameters
	double d_safe;

	void norm_pdf_log(Eigen::VectorXd &result, const Eigen::MatrixXd &xs, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	void generate_norm_dist_samples(Eigen::MatrixXd &samples, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	double produce_MCS_estimate(
		const Eigen::Vector4d &xs_i, 
		const Eigen::Matrix4d &P_i, 
		const Eigen::Vector2d &p_os_cpa,
		const double t_cpa);

	bool determine_sample_validity_4D(
		Eigen::VectorXd &valid,
		const Eigen::MatrixXd &samples, 
		const Eigen::Vector2d &p_os_cpa, 
		const double t_cpa);

	double MCSKF4D_estimation(
		const Eigen::MatrixXd &xs_os,  
		const Eigen::VectorXd &xs_i, 
		const Eigen::Matrix4d &P_i,
		const int i);	

	void determine_sample_validity_2D(
		Eigen::VectorXd &valid,
		const Eigen::MatrixXd &samples, 
		const Eigen::Vector2d &p_os);

	Eigen::VectorXd determine_best_performing_samples(
		Eigen::VectorXd &valid, 
		int &N_e,
		const Eigen::MatrixXd &samples, 
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i);

	double CE_estimation(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i,
		const int i);

public:

	CPE(const CPE_Method cpe_method, const int n_CE, const int n_MCSKF, const double d_safe, const double dt);

	void set_method(const CPE_Method cpe_method) { method = cpe_method; };

	void set_safety_zone_radius(const double d_safe) { this->d_safe = d_safe; };

	void set_number_of_obstacles(const int n_obst);

	void initialize(
		const Eigen::Matrix<double, 6, 1> &xs_os, 
		const Eigen::Vector4d &xs_i, 
		const Eigen::Matrix4d &P_i,
		const int i);

	void reset();

	double estimate(
		const Eigen::Matrix<double, 6, 1> &xs_os, 
		const Eigen::Vector4d &xs_i, 
		const Eigen::Matrix4d &P_i,
		const int i);

};

#endif
