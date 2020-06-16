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
#include "mrou.h"
#include "kf.h"

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

	bool filter_on;

public:

	KF* kf;

	MROU* mrou;

	Obstacle(const Eigen::VectorXd &xs_aug, const Eigen::Matrix4d &P, const bool filter_on, const double T, const double dt);

	int get_id() const { return id; };

	Eigen::Vector4d get_xs() const { return xs_upd; };

	Eigen::Matrix4d get_P() const { return P_upd; };

	double get_duration_tracked() const { return duration_tracked; };

	void reset_duration_tracked() { duration_tracked = 0.0; };

	double get_duration_lost() const { return duration_lost; };

	void reset_duration_lost() { duration_lost = 0.0; };

	void increment_duration_tracked(const double dt) { duration_tracked += dt; };

	void increment_duration_lost(const double dt) { duration_lost += dt; };

	void resize_trajectories(const int n_samples);

	void predict_independent_trajectories(const double T, const double dt);
};

#endif