/****************************************************************************************
*
*  File name : sbmpc.cuh
*
*  Function  : Header file for the original Scenario-based Model Predictive Control.
*			   
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

#pragma once

#include "psbmpc_index.h"
#include "sbmpc_parameters.h"
#include "ownship.cuh"
#include "obstacle.cuh"
#include "mpc_cost.cuh"

#include "Eigen/Dense"
#include <vector>

class SBMPC
{
private:
	// Control behavior related vectors and the own-ship maneuver times in the prediction horizon
	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

	// Previous optimal offsets/modifications
	double u_opt_last;
	double chi_opt_last;

	// Cost at the optimal solution
	double min_cost;

	// Own-ship predicted trajectory
	Eigen::Matrix<double, 6, -1> trajectory;

	Ownship ownship;

	void reset_control_behaviour();

	void increment_control_behaviour();

	void initialize_prediction(Obstacle_Data<Tracked_Obstacle> &data);

	bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

	//
	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

public:

	SBMPC_Parameters pars;

	MPC_Cost<SBMPC_Parameters> mpc_cost;

	SBMPC();
		
	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data<Tracked_Obstacle> &data);

};

#endif 