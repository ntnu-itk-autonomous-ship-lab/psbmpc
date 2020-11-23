/****************************************************************************************
*
*  File name : tracked_obstacle.cuh
*
*  Function  : Header file for the tracked obstacle class used by the PSB-MPC, slightly
*			   modified for this GPU-implementation
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


#ifndef _TRACKED_OBSTACLE_CUH_
#define _TRACKED_OBSTACLE_CUH_

#include <vector>
#include "Eigen/Dense"

#include "obstacle.cuh"
#include "mrou.cuh"
#include "kf.cuh"

class Cuda_Obstacle;

class Tracked_Obstacle : public Obstacle
{
private:

	// To make transfer of data from cpu to gpu obstacle objects easier
	friend class Cuda_Obstacle;

	// State and covariance at the current time or predicted time
	Eigen::Vector4d xs_0;
	Eigen::Matrix4d P_0;

	// Vector of intention probabilities at the current time or last time of update
	Eigen::VectorXd Pr_a;

	// A priori COLREGS compliance probability at the current time or last time of update
	double Pr_CC;

	// If the KF is on, the obstacle is tracked until it dies
	// while the duration lost may be reset if new measurements are aquired
	double duration_tracked, duration_lost;

	// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
	std::vector<bool> mu;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
	// This is equal for all prediction scenarios including those with active COLAV (using MROU)
	Eigen::MatrixXd P_p;  

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

 	// Mean predicted velocity for the obstacle (MROU): 
	Eigen::Vector2d v_p;

	// Prediction scenario ordering, size n_ps x 1 of intentions
	std::vector<Intention> ps_ordering;

	// Course change ordering, weights and maneuvering times for the independent prediction scenarios: n_ps x 1
	Eigen::VectorXd ps_course_changes, ps_maneuver_times; 
	
public:

	KF kf;

	MROU mrou;

	Tracked_Obstacle() {}

	Tracked_Obstacle(const Eigen::VectorXd &xs_aug, 
			 const Eigen::VectorXd &P, 
			 const Eigen::VectorXd &Pr_a, 
			 const double Pr_CC,
			 const bool filter_on, 
			 const double T, 
			 const double dt);

	inline std::vector<bool> get_COLREGS_violation_indicator() const { return mu; }

	inline double get_a_priori_CC_probability() const { return Pr_CC; }

	inline Eigen::VectorXd get_intention_probabilities() const { return Pr_a; }

	// KF related methods
	inline double get_duration_tracked() const { return duration_tracked; }

	inline void reset_duration_tracked() { duration_tracked = 0.0; }

	inline double get_duration_lost() const { return duration_lost; }

	inline void reset_duration_lost() { duration_lost = 0.0; }

	inline void increment_duration_tracked(const double dt) { duration_tracked += dt; }

	inline void increment_duration_lost(const double dt) { duration_lost += dt; }

	// Trajectory prediction related methods
	void resize_trajectories(const int n_samples);

	inline std::vector<Eigen::MatrixXd> get_trajectories() const { return xs_p; }

	inline Eigen::MatrixXd get_trajectory_covariance() const { return P_p; }

	void initialize_prediction(	
		const std::vector<Intention> &ps_ordering,
		const Eigen::VectorXd &ps_course_changes,
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

	void update(
		const Eigen::VectorXd &xs_aug, 
		const Eigen::VectorXd &P, 
		const Eigen::VectorXd &Pr_a, 
		const double Pr_CC,
		const bool filter_on,
		const double dt);

	void update(
		const bool filter_on,
		const double dt);
};

#endif