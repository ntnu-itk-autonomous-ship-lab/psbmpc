/****************************************************************************************
*
*  File name : cpe.cpp
*
*  Function  : Class functions for the collision probability estimator
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

#include "cpe.h"
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI

/****************************************************************************************
*  Name     : CPE
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
CPE::CPE(
    const int n_CE,                                                 // In: Number of samples for the Cross-Entropy method
    const int n_MCSKF,                                              // In: Number of samples for the Monte Carlo Simulation + Kalman-filtering method
    const double d_safe,                                            // In: Safety zone radius around own-ship
    const double dt                                                 // In: Time step of calling function simulation environment
    ) :
    n_CE(n_CE), n_MCSKF(n_MCSKF), generator(seed()), std_norm_pdf(std::normal_distribution<double>(0, 1)), d_safe(d_safe)
{
    // CE pars
    sigma_inject = 0.9;

    alpha_n = 0.9;
    alpha_p = 0.997;

    rho = 0.9;

    max_it = 10;

    converged_last = false;

    // MCSKF4D pars
    p = 0.001;

    q = 0.003;

    P_c_p = 0; P_c_upd = 0;

    var_P_c_p = 0.3; var_P_c_upd = 0;

    dt_seg = 2 * dt;
}

/****************************************************************************************
*  Name     : initialize
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::initialize(
    const Eigen::MatrixXd &xs_A, 
    const Eigen::MatrixXd &P_A, 
    const Eigen::MatrixXd &xs_B, 
    const Eigen::MatrixXd &P_B
    )
{

}

/****************************************************************************************
*  Name     : estimate
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::estimate(
	const Eigen::MatrixXd &xs_A, 
	const Eigen::MatrixXd &P_A, 
	const Eigen::MatrixXd &xs_B, 
	const Eigen::MatrixXd &P_B,
	const CPE_Method cpe_method
    )
{

}

/****************************************************************************************
*  Name     : 
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/


/****************************************************************************************
	Private functions
****************************************************************************************/

/****************************************************************************************
*  Name     : norm_pdf_log
*  Function : Calculates the value of the multivariate normal distribution (MVN) 
*             for a sample
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::norm_pdf_log(
    const Eigen::VectorXd &xs,                                      // In: State vector value of normal distributed RV
    const Eigen::VectorXd &mu,                                      // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                    // In: Covariance of the MVN
    )
{
    double val;



    return val;
}

/****************************************************************************************
*  Name     : generate_norm_dist_samples
*  Function : Generates samples from the MVN distribution with given mean and covariance
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::generate_norm_dist_samples(
    Eigen::MatrixXd &samples,                                       // In: Samples to fill with generated MVN values, size n x n_samples
    const Eigen::VectorXd &mu,                                      // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                    // In: Covariance of the MVN
    )
{

}

/****************************************************************************************
*  Name     : produce_MCS_estimate
*  Function : Uses Monte Carlo Simulation to produce a collision probability "measurement"
*             for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::produce_MCS_estimate(
	const Eigen::VectorXd &xs, 
	const Eigen::MatrixXd &P, 
	const Eigen::Vector2d &p,
	const double t
    )
{

}

/****************************************************************************************
*  Name     : check_sample_validity_4D
*  Function : Determine if a sample is valid for use in estimation for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool CPE::check_sample_validity_4D(
    const Eigen::MatrixXd samples, 
    const Eigen::Vector2d p_OS, 
    const double t
    )
{

}