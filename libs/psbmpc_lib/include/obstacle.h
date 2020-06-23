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
#include "utilities.h"

class Obstacle {
private:

	int ID;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> v_p;

	// Predicted covariance for each prediction scenario: n_ps x n*n x n_samples, i.e. the covariance is flattened for each time step
	std::vector<Eigen::MatrixXd> P_p;  

	// Time of alternative maneuvers for the obstacle prediction scenarios
	Eigen::MatrixXd maneuver_times;

	// Predicted obstacle trajectory and covariance when it is behaving intelligently
	Eigen::MatrixXd xs_colav_p, P_colav_p;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double A, B, C, D, l, w;

	double x_offset, y_offset;

	double duration_tracked, duration_lost;

	bool colav_on, filter_on;

public:

	KF* kf;

	MROU* mrou;

	~Obstacle();

	Obstacle(const Eigen::VectorXd &xs_aug, const Eigen::Matrix4d &P, const bool filter_on, const bool colav_on, const double T, const double dt);

	int get_ID() const { return ID; };

	double get_duration_tracked() const { return duration_tracked; };

	void reset_duration_tracked() { duration_tracked = 0.0; };

	double get_duration_lost() const { return duration_lost; };

	void reset_duration_lost() { duration_lost = 0.0; };

	void increment_duration_tracked(const double dt) { duration_tracked += dt; };

	void increment_duration_lost(const double dt) { duration_lost += dt; };

	void resize_trajectories(const int n_samples);

	std::vector<Eigen::MatrixXd> get_independent_trajectories() const { return xs_p; };

	std::vector<Eigen::MatrixXd> get_independent_trajectory_covariances() const { return P_p; };

	void predict_independent_trajectories(const double T, const double dt);

	void set_dependent_trajectory(const Eigen::MatrixXd& xs_colav_p, const Eigen::MatrixXd& P_colav_p) { this->xs_colav_p = xs_colav_p; this->P_colav_p = P_colav_p; };

	Eigen::MatrixXd get_dependent_trajectory() const { return xs_colav_p; };

	Eigen::MatrixXd get_dependent_trajectory_covariance() const { return P_colav_p; };
};

#endif