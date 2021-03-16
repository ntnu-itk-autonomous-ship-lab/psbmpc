/****************************************************************************************
*
*  File name : obstacle_sbmpc.h
*
*  Function  : Header file for Scenario-based Model Predictive Control used by obstacles
*			   in the PSB-MPC predictions.
*
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

#ifndef _OBSTACLE_SBMPC_H_
#define _OBSTACLE_SBMPC_H_


#include "psbmpc_index.h"
#include "sbmpc_parameters.h"
#include "joint_prediction_manager.h"
#include "obstacle_ship.h"
#include "mpc_cost.h"
#include "Eigen/Dense"
#include <vector>
#include <memory>

class Obstacle_SBMPC
{
private:

	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

	double u_m_last;
	double chi_m_last;

	double min_cost;

	Obstacle_Ship ownship;

	Eigen::Matrix<double, 4, -1> trajectory;

	void assign_data(const Obstacle_SBMPC &o_sbmpc);

	void initialize_prediction(Obstacle_Data<Prediction_Obstacle> &data, const int k_0);

	void reset_control_behavior();

	void increment_control_behavior();

	bool determine_colav_active(const Obstacle_Data<Prediction_Obstacle> &data, const int n_static_obst);

	void assign_optimal_trajectory(Eigen::Matrix<double, 4, -1> &optimal_trajectory);

public:

	SBMPC_Parameters pars;

	MPC_Cost<SBMPC_Parameters> mpc_cost;

	Obstacle_SBMPC();

	~Obstacle_SBMPC();

	Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

	Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

	void calculate_optimal_offsets(
		double &u_opt, 	
		double &chi_opt, 
		Eigen::Matrix<double, 4, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Vector4d &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data<Prediction_Obstacle> &data,
		const int k_0);

};

#endif 