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

	// Predicted state trajectory from the current time onwards
	// either from Obstacle_SBMPC or independent prediction
	Eigen::MatrixXd xs_p;

	// In use when using the Obstacle_SBMPC
	Eigen::Matrix<double, 2, -1> waypoints; 

	void assign_data(const Prediction_Obstacle &po);
	void assign_data(const Tracked_Obstacle &to);
	
public:

	std::unique_ptr<Obstacle_SBMPC> sbmpc;

	Prediction_Obstacle() {};

	Prediction_Obstacle(const Eigen::VectorXd &xs_aug,
			 const Eigen::VectorXd &P,	 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	Prediction_Obstacle(const Prediction_Obstacle &po);
	Prediction_Obstacle(const Tracked_Obstacle &to);

	Prediction_Obstacle& operator=(const Prediction_Obstacle &rhs);
	Prediction_Obstacle& operator=(const Tracked_Obstacle &rhs);

	Eigen::Vector4d get_initial_state() const { return xs_0; }; 
	Eigen::Vector4d get_predicted_state(const int k) const { return xs_p.col(k); }; 

	Eigen::MatrixXd get_trajectory() const { return xs_p; };

	void set_predicted_state(const Eigen::Vector4d &xs_k, const int k) { xs_p.col(k) = xs_k; }

	void set_waypoints(const Eigen::Matrix<double, 2, -1> &waypoints) { this->waypoints = waypoints; }

	void predict_independent_trajectory(const double T, const double dt);

	void update(const Eigen::Vector4d &xs);
};

#endif