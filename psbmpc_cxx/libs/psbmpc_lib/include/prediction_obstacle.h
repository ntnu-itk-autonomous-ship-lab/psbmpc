/****************************************************************************************
*
*  File name : prediction_obstacle.h
*
*  Function  : Header file for the predicion obstacle class. Derived simpler variant of  
*			   the obstacle class, specifically made for the PSB-MPC
*			   predictions with active obstacle COLAV systems.
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


#ifndef _PREDICTION_OBSTACLE_H_
#define _PREDICTION_OBSTACLE_H_

#include "Eigen/Dense"
#include "obstacle.h"
#include <memory>

class Obstacle_SBMPC;

class Prediction_Obstacle : public Obstacle
{
private:

	Eigen::Matrix4d A_CV;

	// Predicted state trajectory
	Eigen::MatrixXd xs_p;

	void assign_data(const Prediction_Obstacle &po);
	
public:

	std::unique_ptr<Obstacle_SBMPC> sbmpc;

	Prediction_Obstacle() {};

	Prediction_Obstacle(const Eigen::VectorXd &xs_aug,
			 const Eigen::VectorXd &P,	 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	Prediction_Obstacle(const Prediction_Obstacle &po);

	Prediction_Obstacle& operator=(const Prediction_Obstacle &rhs);

	Eigen::Vector4d get_state() const { return xs_0; };

	Eigen::MatrixXd get_trajectory() const { return xs_p; };

	void set_trajectory(const Eigen::MatrixXd &xs_p) { if (colav_on) { this->xs_p = xs_p; }};

	void predict_independent_trajectory(const double T, const double dt);

	void update(const Eigen::Vector4d &xs);
};

#endif