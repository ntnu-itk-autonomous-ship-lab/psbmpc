/****************************************************************************************
*
*  File name : prediction_obstacle.h
*
*  Function  : Header file for the predicion obstacle class. Simpler variant of the 
*			   obstacle class used in the PSB-MPC, specifically made for the PSB-MPC
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

class Obstacle_SBMPC;

class Prediction_Obstacle 
{
private:

	int ID;

	bool colav_on;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double l, w;

	double x_offset, y_offset;

	Eigen::Matrix4d A;

	// State at the current predicted time
	Eigen::Vector4d xs_0;

	// Predicted state trajectory
	Eigen::MatrixXd xs_p;
	
public:

	Obstacle_SBMPC *sbmpc;

	Prediction_Obstacle(const Eigen::VectorXd &xs_aug, 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	Prediction_Obstacle(const Prediction_Obstacle &po);

	~Prediction_Obstacle();

	Prediction_Obstacle& operator=(const Prediction_Obstacle &po);

	int get_ID() const { return ID; };

	Eigen::Vector4d get_state() const { return xs_0; };

	Eigen::MatrixXd get_trajectory() const { return xs_p; };

	void set_trajectory(const Eigen::MatrixXd &xs_p) { if (colav_on) { this->xs_p = xs_p; }};

	void predict_independent_trajectory(const double T, const double dt);

	void update(const Eigen::Vector4d &xs, const bool colav_on);
};

#endif