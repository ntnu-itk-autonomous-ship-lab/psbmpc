/****************************************************************************************
*
*  File name : obstacle.h
*
*  Function  : Header file for the obstacle class. Modified version of the one created
*			   for SBMPC by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor
*			   through the Autosea project.
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


#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <vector>
#include "Eigen/Dense"

#include "ekf.h"

class Obstacle {
private:

	int id;

	Eigen::VectorXd x_p, y_p, V_x_p, V_y_p;

	// NOTE: Not to be confused with predicted error covariance. This is the trajectory covariance.
	std::vector<Eigen::Matrix4d> P_p;

	Eigen::VectorXd maneuver_times;

	Eigen::Vector4d xs_upd;
	Eigen::Matrix4d P_upd;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double A, B, C, D, l, w;

	double x_offset, y_offset;

	double duration_tracked, duration_lost;

public:

	KF* kf;

	MROU* mrou;

	Obstacle(const Eigen::Vector4d &xs, const std::vector<Eigen::Matrix4d> &P, const bool filter_on, const double t);

	Eigen::VectorXd get_xs() const { return xs_upd; };

	Eigen::Matrix4d get_P() const { return P_upd; };

	void resize_trajectories(const int n_samples);

	void predict_independent_trajectories(const double T, const double dt);

	void increment_duration_tracked(const double dt) { duration_tracked += dt; };

	void increment_duration_lost(const double dt) { duration_lost += dt; };
};

#endif
