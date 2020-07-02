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
    const double d_safe                                             // In: Safety zone radius around own-ship
    ) :
    n_CE(n_CE), n_MCSKF(n_MCSKF), gen(seed()), std_norm_pdf(std::normal_distribution<double>(0, 1)), d_safe(d_safe)
{
    
}

/****************************************************************************************
*  Name     : initialize
*  Function : 
*  Author   : 
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
*  Author   : 
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
*  Function : Calculates the value of the multivariate normal distribution for a sample
*  Author   : 
*  Modified :
*****************************************************************************************/
double CPE::norm_pdf_log(
    const Eigen::VectorXd &xs, 
    const Eigen::VectorXd &mu, 
    const Eigen::MatrixXd &Sigma
    )
{

}
/****************************************************************************************
*  Name     : produce_MCS_estimate
*  Function : Uses Monte Carlo Simulation to produce a collision probability "measurement"
*             for the MCSKF4D method
*  Author   : 
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
*  Author   : 
*  Modified :
*****************************************************************************************/
bool CPE::check_sample_validity_4D(
    const Eigen::MatrixXd samples, 
    const Eigen::Vector2d p_OS, 
    const double t
    )
{

}