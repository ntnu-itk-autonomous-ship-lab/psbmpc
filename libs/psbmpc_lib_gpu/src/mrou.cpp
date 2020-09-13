/****************************************************************************************
*
*  File name : mrou.cpp
*
*  Function  : Implements class functions for the Mean-Reverting Ornstein-Uhlenbeck 
*			   process
*  
*	           ---------------------
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

#include "mrou.h"
#include "math.h"
#include <iostream>


/****************************************************************************************
*  Name     : MROU
*  Function : Class constructor, initializes parameters and variables
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
MROU::MROU() :
	sigma_x(0.8), 						
	sigma_xy(0.0), 
	sigma_y(0.8), 
	gamma_x(0.1), 						
	gamma_y(0.1)
{
	// Constant part of OU predictor covariance
	Sigma_1 << pow(sigma_x, 2) / pow(gamma_x, 3), sigma_xy / (gamma_x * gamma_y), pow(sigma_x, 2) / (2 * pow(gamma_x, 2)), 2 * sigma_xy / gamma_x,

				sigma_xy / (gamma_x * gamma_y), pow(sigma_y, 2) / pow(gamma_y, 3),  2 * sigma_xy / gamma_y, pow(sigma_y, 2) / (2 * pow(gamma_y, 2)),

				pow(sigma_x, 2) / (2 * pow(gamma_x, 2)), 2 * sigma_xy / gamma_y, pow(sigma_x, 2) / gamma_x, 2 * sigma_xy / (gamma_x + gamma_y), 

				2 * sigma_xy / gamma_x, pow(sigma_y, 2) / (2 * pow(gamma_y, 2)), 2 * sigma_xy / (gamma_x + gamma_y), pow(sigma_y, 2) / gamma_y;
}

MROU::MROU(
	const double sigma_x, 						// In: Wiener process standard deviations sigma
	const double sigma_xy, 
	const double sigma_y, 
	const double gamma_x, 						// In: Revertion rate factors gamma
	const double gamma_y
	) :
	sigma_x(sigma_x), 
	sigma_xy(sigma_xy), 
	sigma_y(sigma_y), 
	gamma_x(gamma_x), 
	gamma_y(gamma_y)
{

	// Constant part of OU predictor covariance
	Sigma_1 << pow(sigma_x, 2) / pow(gamma_x, 3), sigma_xy / (gamma_x * gamma_y), pow(sigma_x, 2) / (2 * pow(gamma_x, 2)), 2 * sigma_xy / gamma_x,

				sigma_xy / (gamma_x * gamma_y), pow(sigma_y, 2) / pow(gamma_y, 3),  2 * sigma_xy / gamma_y, pow(sigma_y, 2) / (2 * pow(gamma_y, 2)),

				pow(sigma_x, 2) / (2 * pow(gamma_x, 2)), 2 * sigma_xy / gamma_y, pow(sigma_x, 2) / gamma_x, 2 * sigma_xy / (gamma_x + gamma_y), 

				2 * sigma_xy / gamma_x, pow(sigma_y, 2) / (2 * pow(gamma_y, 2)), 2 * sigma_xy / (gamma_x + gamma_y), pow(sigma_y, 2) / gamma_y;
}

/****************************************************************************************
*  Name     : f, g, h, k
*  Function : Functions used in the covariance calculation
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
double MROU::f(
	const double t 								// In: Prediction time		
	) const 
{
	return 0.5 * (2 * t + 4 * exp(-t) - exp(- 2 * t) - 3);
}

double MROU::g(
	const double t 								// In: Prediction time	
	) const 
{
	return 0.5 * (1 - exp(- 2 * t));
}

double MROU::h(
	const double t 								// In: Prediction time	
	) const 
{
	return t - (1 - exp(- t * gamma_x)) / gamma_x - (1 - exp(- t * gamma_y)) / gamma_y + 
		(1 - exp( - t * (gamma_x + gamma_y))) / (gamma_x + gamma_y);
}

double MROU::k(
	const double t 								// In: Prediction time
	) const 
{
	return exp(- 2 * t) * pow(1 - exp(t), 2);
}

/****************************************************************************************
*  Name     : predict_state
*  Function : Predicts the state t seconds forward in time using the OU process
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::Vector4d MROU::predict_state(
	const Eigen::Vector4d& xs_old, 				// In: Old state
	const Eigen::Vector2d& v, 					// In: Typical mean velocity for the process at the current time
	const double t								// In: Prediction time t	
	)
{
	Eigen::Vector4d xs;
	Eigen::Matrix4d Phi;
	Eigen::Matrix<double, 4, 2> Psi;

	Phi <<  1, 0, (1 - exp( - t * gamma_x)) / gamma_x, 				0,
			0, 1, 				0, 						(1 - exp( - t * gamma_y)) / gamma_y,
			0, 0, 		exp( - t * gamma_x), 						0,
			0, 0, 				0, 							exp( - t * gamma_y);

	Psi <<  t - (1 - exp( - t * gamma_x)) / gamma_x, 					0,
						0, 								t - (1 - exp( - t * gamma_y)) / gamma_y, 
			1 - exp( - t * gamma_x), 									0,
						0,  								1 - exp( - t * gamma_y);

	xs = Phi * xs_old + Psi * v;

	return xs;
}

/****************************************************************************************
*  Name     : predict_covariance
*  Function : Predicts the covariance t seconds forward in time using the OU process
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
Eigen::Matrix4d MROU::predict_covariance(
	const Eigen::Matrix<double, 4, 4> &P_old, 	// In: Old covariance
	const double t 								// In: Prediction time
	)
{
	Eigen::Matrix4d P, Sigma_2;
	Sigma_2 << f(t * gamma_x), h(t), k(t * gamma_x), g(gamma_y * t / 2) / gamma_y - g((gamma_x + gamma_y) * t / 2) / (gamma_x + gamma_y), 

				h(t), f(t * gamma_y), g(gamma_x * t / 2) / gamma_x - g((gamma_x + gamma_y) * t / 2) / (gamma_x + gamma_y), k(t * gamma_y), 

				k(t * gamma_x), g(gamma_x * t / 2) / gamma_x - g((gamma_x + gamma_y) * t / 2) / (gamma_x + gamma_y), g(t * gamma_x), g((gamma_x + gamma_y) * t / 2),

				g(gamma_y * t / 2) / gamma_y - g((gamma_x + gamma_y) * t / 2) / (gamma_x + gamma_y), k(t * gamma_y), g((gamma_x + gamma_y) * t / 2), g(t * gamma_y);

	P = P_old + Sigma_1.cwiseProduct(Sigma_2); 

	return P;
}