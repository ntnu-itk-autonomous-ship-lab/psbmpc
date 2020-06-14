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


/****************************************************************************************
*  Name     : MROU
*  Function : Class constructor, initializes parameters and variables
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
MROU::MROU() :
	sigma_x_(0.8), 						
	sigma_xy_(0), 
	sigma_y_(0.8), 
	gamma_x_(0.1), 						
	gamma_y_(0.1)
{

	// Calculate constant part of OU predictor covariance
	Sigma_1_ << sigma_x_**2 / pow(gamma_x_, 3), sigma_xy_ / (gamma_x_ * gamma_y_), sigma_x_ ** 2 / (2 * gamma_x_ ** 2), 2 * sigma_xy_ / gamma_x_,

				sigma_xy_ / (gamma_x_ * gamma_y_), sigma_y_**2 / pow(gamma_y_, 3),  2 * sigma_xy_ / gamma_y_, sigma_y_ ** 2 / (2 * gamma_y_ ** 2),

				sigma_x_ ** 2 / (2 * gamma_x_ ** 2), 2 * sigma_xy_ / gamma_y_, sigma_x_ ** 2 / gamma_x_, 2 * sigma_xy_ / (gamma_x_ + gamma_y_), 

				2 * sigma_xy_ / gamma_x_, sigma_y_ ** 2 / (2 * gamma_y_ ** 2), 2 * sigma_xy_ / (gamma_x_ + gamma_y_), sigma_y_ ** 2 / gamma_y_;
}

MROU::MROU(
	const double sigma_x, 						// In: Wiener process standard deviations sigma
	const double sigma_xy, 
	const double sigma_y, 
	const double gamma_x, 						// In: Revertion rate factors gamma
	const double gamma_y
	) :
	sigma_x_(sigma_x), 
	sigma_xy_(sigma_xy), 
	sigma_y_(sigma_y), 
	gamma_x_(gamma_x), 
	gamma_y_(gamma_y)
{

	// Calculate constant part of OU predictor covariance
	Sigma_1_ << sigma_x_**2 / pow(gamma_x_, 3), sigma_xy_ / (gamma_x_ * gamma_y_), sigma_x_ ** 2 / (2 * gamma_x_ ** 2), 2 * sigma_xy_ / gamma_x_,

				sigma_xy_ / (gamma_x_ * gamma_y_), sigma_y_**2 / pow(gamma_y_, 3),  2 * sigma_xy_ / gamma_y_, sigma_y_ ** 2 / (2 * gamma_y_ ** 2),

				sigma_x_ ** 2 / (2 * gamma_x_ ** 2), 2 * sigma_xy_ / gamma_y_, sigma_x_ ** 2 / gamma_x_, 2 * sigma_xy_ / (gamma_x_ + gamma_y_), 

				2 * sigma_xy_ / gamma_x_, sigma_y_ ** 2 / (2 * gamma_y_ ** 2), 2 * sigma_xy_ / (gamma_x_ + gamma_y_), sigma_y_ ** 2 / gamma_y_;
}

/****************************************************************************************
*  Name     : f, g, h, k
*  Function : Functions used in the covariance calculation
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
double MROU::f(
	const double t 								// In: Prediction time t = t_k+1 - t_k		
	) const {
	return 0.5 * (2 * t + 4 * exp(-t) - exp(- 2 * t) - 3);
}

double MROU::g(
	const double t 								// In: Prediction time t = t_k+1 - t_k	
	) const {
	return 0.5 * (1 - exp(- 2 * t));
}

double MROU::h(
	const double t 								// In: Prediction time t = t_k+1 - t_k	
	) const {
	return t - (1 - exp(- t * gamma_x_)) / gamma_x_ - (1 - exp(- t * gamma_y_)) / gamma_y_ + 
		(1 - exp( - t * (gamma_x_ + gamma_y_))) / (gamma_x_ + gamma_y_);
}

double MROU::k(
	const double t 								// In: Prediction time t = t_k+1 - t_k	
	) const {
	return exp(- 2 * t) * (1 - exp(t))**2;
}

/****************************************************************************************
*  Name     : predict_state
*  Function : Predicts the state t seconds forward in time using the OU process
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void MROU::predict_state(
	Eigen::VectorXd &xs, 						// In/out:  State to be predicted
	const Eigen::Vector2d &v, 					// In: 		Typical mean velocity for the process
	const double t								// In: 		Prediction time t = t_k+1 - t_k	
	){

	Eigen::Matrix<double, 4, 4> Phi, Psi;

	Phi <<  1, 0, (1 - exp( - t * gamma_x_)) / gamma_x_, 0,
			0, 1, 0, (1 - exp( - t * gamma_y_)) / gamma_y_,
			0, 0, exp( - t * gamma_x_), 0,
			0, 0, 0, exp( - t * gamma_y_);

	Psi <<  t - (1 - exp( - t * gamma_x_)) / gamma_x_, 0,
			0, t - (1 - exp( - t * gamma_y_)) / gamma_y_, 
			- exp( - t * gamma_x_), 0
			0, - exp( - t * gamma_y_);

	xs = Phi * xs + Psi * v;
}

/****************************************************************************************
*  Name     : predict_covariance
*  Function : Predicts the covariance t seconds forward in time using the OU process
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void MROU::predict_covariance(
	Eigen::MatrixXd &P, 						// In/out: Covariance to be predicted
	const double t 								// In: 	   Prediction time t = t_k+1 - t_k
	){

	Sigma_2 << f(t * gamma_x_), h(t), k(t * gamma_x_), g(gamma_y_ * t / 2) / gamma_y_ - g((gamma_x_ + gamma_y_) * t / 2) / (gamma_x_ + gamma_y_), 

				h(t), f(t * gamma_y_), g(gamma_x_ * t / 2) / gamma_x_ - g((gamma_x_ + gamma_y_) * t / 2) / (gamma_x_ + gamma_y_), k(t * gamma_y_), 

				k(t * gamma_x_), g(gamma_x_ * t / 2) / gamma_x_ - g((gamma_x_ + gamma_y_) * t / 2) / (gamma_x_ + gamma_y_), , g(t * gamma_x_), g((gamma_x_ + gamma_y_) * t / 2),

				g(gamma_y_ * t / 2) / gamma_y_ - g((gamma_x_ + gamma_y_) * t / 2) / (gamma_x_ + gamma_y_), k(t * gamma_y_), g((gamma_x_ + gamma_y_) * t / 2), g(t * gamma_y_);


	P = P + Sigma_1_.cwiseProduct(Sigma_2); 
}