/****************************************************************************************
*
*  File name : prediction_obstacle.h
*
*  Function  : Header file for the predicion obstacle class. Simpler variant of the 
*			   obstacle class used in the PSB-MPC.
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

class Prediction_Obstacle {
private:

	int ID;

	bool colav_on;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double A, B, C, D, l, w;

	double x_offset, y_offset;

	// State at the current predicted time
	Eigen::Vector4d xs_0;

	// Predicted state trajectory
	Eigen::MatrixXd xs_p;
	
public:

	Obstacle_SBMPC *sbmpc;

	~Prediction_Obstacle();

	Prediction_Obstacle(const Eigen::VectorXd &xs_aug, 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	int get_ID() const { return ID; };

	void predict_trajectory(const double T, const double dt);

	void resize_trajectory(const int n_samples) { xs_p.resize(4, n_samples); };

	Eigen::MatrixXd get_trajectory() const { return xs_p; };

	void update(
		const Eigen::Vector4d &xs, 
		const bool colav_on,
		const double dt);
};

#endif