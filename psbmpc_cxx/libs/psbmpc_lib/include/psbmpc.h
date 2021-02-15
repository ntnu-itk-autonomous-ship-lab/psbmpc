/****************************************************************************************
*
*  File name : psbmpc.h
*
*  Function  : Header file for Probabilistic Scneario-based Model Predictive Control.
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

#ifndef _PSBMPC_H_
#define _PSBMPC_H_

#include "psbmpc_index.h"
#include "psbmpc_parameters.h"
#include "obstacle_sbmpc.h"
#include "ownship.h"
#include "cpe.h"
#include "mpc_cost.h"

#include "Eigen/Dense"
#include <vector>
#include <memory>	

class PSBMPC
{
private:
	// Amount of prediction scenarios for each obstacle
	std::vector<int> n_ps;

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

	CPE cpe;

	std::vector<Prediction_Obstacle> pobstacles;

	bool use_joint_prediction;

	void reset_control_behaviour();

	void increment_control_behaviour();

	void initialize_prediction(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1> &static_obstacles);

	void set_up_independent_obstacle_prediction(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_maneuver_times,
		const double t_cpa_i,
		const Obstacle_Data<Tracked_Obstacle> &data,
		const int i);

	// Obstacle prediction scenario pruning related methods
	void prune_obstacle_scenarios(Obstacle_Data<Tracked_Obstacle> &data);

	void calculate_instantaneous_collision_probabilities(
		Eigen::MatrixXd &P_c_i, 
		const Obstacle_Data<Tracked_Obstacle> &data, 
		const int i, 
		const double dt, 
		const int p_step);
		
	void calculate_ps_collision_probabilities(Eigen::VectorXd &P_c_i_ps, const Eigen::MatrixXd &P_c_i, const int i);

	void calculate_ps_collision_consequences(Eigen::VectorXd &C_i, const Obstacle_Data<Tracked_Obstacle> &data, const int i, const double dt, const int p_step);

	void calculate_ps_collision_risks(
		Eigen::VectorXd &R_c_i, 
		Eigen::VectorXi &indices_i, 
		const Eigen::VectorXd &C_i, 
		const Eigen::VectorXd &P_c_i_ps, 
		const Obstacle_Data<Tracked_Obstacle> &data, 
		const int i);

	void predict_trajectories_jointly(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1>& static_obstacles, const bool overwrite);

	double find_time_of_passing(const Obstacle_Data<Tracked_Obstacle> &data, const int i);

	bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

public:

	PSBMPC_Parameters pars;

	MPC_Cost<PSBMPC_Parameters> mpc_cost;

	PSBMPC();

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