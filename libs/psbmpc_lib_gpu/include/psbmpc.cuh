/****************************************************************************************
*
*  File name : psbmpc.cuh
*
*  Function  : Header file for Probabilistic Scenario-based Model Predictive Control, 
*			   slightly modified for this GPU-implementation.
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

#ifndef _PSBMPC_CUH_
#define _PSBMPC_CUH_

#include "psbmpc_index.h"
#include "psbmpc_parameters.h"
#include "obstacle_manager.cuh"
#include "obstacle.cuh"
#include "ownship.cuh"
#include "cpe.cuh"

#include "Eigen/Dense"
#include <vector>
#include <memory>

class CB_Cost_Functor;

class PSBMPC
{
private:

	std::vector<int> n_ps;

	Eigen::VectorXd maneuver_times;
	Eigen::MatrixXd control_behaviours;
	
	double u_m_last;
	double chi_m_last;

	double min_cost;
	int min_index;

	Ownship ownship;

	CPE cpe;

	Eigen::Matrix<double, 6, -1> trajectory;

	friend class CB_Cost_Functor;

	void map_offset_sequences();

	void reset_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void increment_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void initialize_prediction(Obstacle_Data &data);

	void set_up_independent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const int n_turns,
		const Obstacle_Data &data,
		const int i);

	void set_up_dependent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const Obstacle_Data &data,
		const int i);

	double find_time_of_passing(const Obstacle_Data &data, const int i);

	bool determine_colav_active(const Obstacle_Data &data, const int n_static_obst);

	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

public:

	PSBMPC_Parameters pars;

	PSBMPC();

	~PSBMPC();

	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data &data);

};

#endif 