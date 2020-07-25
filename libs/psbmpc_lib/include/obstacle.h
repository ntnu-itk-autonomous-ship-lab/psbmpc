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

class Obstacle_SBMPC;

enum Intention 
{
	KCC, 					// Keep current course
	SM, 					// Starboard maneuver
	PM 						// Port maneuver
};

class Obstacle {
private:

	int ID;

	bool colav_on, filter_on;

	// Vector of intention probabilities at the current time or last time of update
	Eigen::VectorXd Pr_a;

	// A priori COLREGS compliance probability at the current time or last time of update
	double Pr_CC;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double A, B, C, D, l, w;

	double x_offset, y_offset;

	// If the AIS-based KF is on, the obstacle is tracked until it dies
	// while the duration lost may be reset if new measurements are aquired
	double duration_tracked, duration_lost;

	// State and covariance at the current time or last time of update
	Eigen::Vector4d xs_0;
	Eigen::Matrix4d P_0;

	// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
	std::vector<bool> mu;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): 
	Eigen::Vector2d v_p;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
	// This is equal for all prediction scenarios when using MROU, thus no std::vector needed
	Eigen::MatrixXd P_p;  

	// Prediction scenario ordering, size n_ps x 1 of intentions
	std::vector<Intention> ps_ordering;

	// Course change ordering, weights and maneuvering times for the prediction scenarios: n_ps x 1
	Eigen::VectorXd ps_course_changes, ps_weights, ps_maneuver_times;

	// Predicted obstacle trajectory when it is behaving intelligently
	Eigen::MatrixXd xs_colav_p;
	
public:

	KF *kf;

	MROU *mrou;

	Obstacle_SBMPC *sbmpc;

	~Obstacle();

	Obstacle(const Eigen::VectorXd &xs_aug, 
			 const Eigen::VectorXd &P, 
			 const Eigen::VectorXd Pr_a, 
			 const double Pr_CC,
			 const bool filter_on, 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	int get_ID() const { return ID; };

	std::vector<bool> get_COLREGS_violation_indicator() const { return mu; };

	double get_a_priori_CC_probability() const { return Pr_CC; };

	Eigen::VectorXd get_intention_probabilities() const { return Pr_a; };

	// AIS-based KF related methods
	double get_duration_tracked() const { return duration_tracked; };

	void reset_duration_tracked() { duration_tracked = 0.0; };

	double get_duration_lost() const { return duration_lost; };

	void reset_duration_lost() { duration_lost = 0.0; };

	void increment_duration_tracked(const double dt) { duration_tracked += dt; };

	void increment_duration_lost(const double dt) { duration_lost += dt; };

	// Trajectory prediction related methods
	void resize_trajectories(const int n_samples);

	std::vector<Eigen::MatrixXd> get_independent_trajectories() const { return xs_p; };

	Eigen::MatrixXd get_trajectory_covariance() const { return P_p; };

	void initialize_independent_prediction(	
		const std::vector<Intention> &ps_ordering,
		const Eigen::VectorXd &ps_course_changes,
		const Eigen::VectorXd &ps_weights,
		const Eigen::VectorXd &ps_maneuver_times);

	// Some PSBMPC parameters needed to determine if obstacle breaches COLREGS 
	// (future: implement simple sbmpc class for obstacle which has the "determine COLREGS violation"
	// function and equal parameters for the phi-angles, d_close and d_safe)
	void predict_independent_trajectories(
		const double T, 
		const double dt, 
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const double phi_AH,
		const double phi_CR,
		const double phi_HO,
		const double phi_OT,
		const double d_close,
		const double d_safe);

	// Methods where obstacle COLAV must be activated
	void set_dependent_trajectory(const Eigen::MatrixXd &xs_colav_p) { this->xs_colav_p = xs_colav_p; };

	Eigen::MatrixXd get_dependent_trajectory() const { return xs_colav_p; };

	void update(
		const Eigen::VectorXd &xs_aug, 
		const Eigen::VectorXd &P, 
		const Eigen::VectorXd &Pr_a, 
		const double Pr_CC,
		const bool filter_on,
		const double dt);
};

#endif