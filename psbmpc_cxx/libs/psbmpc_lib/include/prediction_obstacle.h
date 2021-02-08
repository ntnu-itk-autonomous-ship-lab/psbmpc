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
#include "tracked_obstacle.h"
#include <memory>
#include <assert.h>

class Obstacle_SBMPC;

class Prediction_Obstacle : public Obstacle
{
private:

	// Intention of this obstacle when behaving intelligently
	Intention a_p;

	// Indicator of whether or not the prediction obstacle breaches COLREGS in
	// the joint prediction currently considered
	bool mu; 

	Eigen::Matrix4d A_CV;

	// Actual state trajectory from the joint prediction scheme
	// when the obstacle uses its own SB-MPC, used by the PSB-MPC,
	// and the predicted trajectory from the Obstacle_SBMPC
	// at the current predicted time t_k
	Eigen::MatrixXd xs_p, xs_k_p;

	// In use when using the Obstacle_SBMPC
	Eigen::Matrix<double, 2, -1> waypoints; 

	void assign_data(const Prediction_Obstacle &po);
	void assign_data(const Tracked_Obstacle &to);
	
public:

	std::unique_ptr<Obstacle_SBMPC> sbmpc;

	Prediction_Obstacle();

	Prediction_Obstacle(const Eigen::VectorXd &xs_aug,	 
			 const bool colav_on, 
			 const double T, 
			 const double dt);

	Prediction_Obstacle(const Prediction_Obstacle &po);
	Prediction_Obstacle(const Tracked_Obstacle &to);

	~Prediction_Obstacle();

	Prediction_Obstacle& operator=(const Prediction_Obstacle &rhs);
	Prediction_Obstacle& operator=(const Tracked_Obstacle &rhs);

	inline Intention get_intention() const { return a_p; }

	inline bool get_COLREGS_breach_indicator() const { return mu; }

	// State at the current predicted time in the joint predictions
	inline Eigen::Vector4d get_initial_state() const { return xs_0; }; 
	// State at the current predicted time + k time steps in the joint predictions
	inline Eigen::Vector4d get_state(const int k) const { assert(k < xs_p.cols() && k >= 0); return xs_p.col(k); }; 

	inline Eigen::MatrixXd get_trajectory() const { return xs_p; };
	inline Eigen::MatrixXd get_predicted_trajectory() const { return xs_k_p; };

	inline Eigen::Matrix<double, 2, -1> get_waypoints() const { return waypoints; }

	inline void set_intention(const Intention a) {assert(a >= KCC && a <= PM); a_p = a; }

	inline void set_state(const Eigen::Vector4d &xs_k, const int k) { assert(k < xs_p.cols() && k >= 0); xs_p.col(k) = xs_k; }

	inline void set_waypoints(const Eigen::Matrix<double, 2, -1> &waypoints) { this->waypoints = waypoints; }

	void predict_independent_trajectory(const double T, const double dt, const int k);

	void update(const Eigen::Vector4d &xs, const int k);

};

#endif