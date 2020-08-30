/****************************************************************************************
*
*  File name : obstacle_manager.h
*
*  Function  : Header file for the obstacle management interface and data structure for
*			   keeping information on dynamic obstacles. As of now only
*			   used for dynamic obstacle management.
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

#ifndef _OBSTACLE_MANAGER_H_
#define _OBSTACLE_MANAGER_H_

#include "psbmpc_parameters.h"
#include "tracked_obstacle.h"
#include "Eigen/Dense"
#include <vector>
#include <memory>

class PSBMPC;

enum ST 
{
	A, 														// Non-COLREGS situation	(ST = Ø)
	B, 														// Stand-on in Overtaking 	(ST = OT, SO)
	C, 														// Stand-on in Crossing 	(ST = CR, SO)
	D, 														// Give-way in Overtaking 	(ST = OT, GW)
	E, 														// Give-way in Head-on 		(ST = HO, GW)
	F 														// Give-way in Crossing 	(ST = CR, GW)
};	

struct Obstacle_Data
{
	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	std::vector<bool> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

	// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
	std::vector<ST> ST_0, ST_i_0;

	// Obstacle hazard levels, on a scale from 0 to 1 (output from PSBMPC)
	Eigen::VectorXd HL_0;

	std::vector<std::unique_ptr<Tracked_Obstacle>> old_obstacles;
	std::vector<std::unique_ptr<Tracked_Obstacle>> new_obstacles;

	Eigen::MatrixXd obstacle_status;

	Obstacle_Data() {}

};

class Obstacle_Manager
{
private:

	friend class PSBMPC;

	double T_lost_limit, T_tracked_limit;

	bool obstacle_filter_on;

	std::shared_ptr<Obstacle_Data> data;

	void determine_situation_type(
		ST &st_A,
		ST &st_B,
		const PSBMPC_Parameters &psbmpc_pars,
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

    void update_obstacles(
		const PSBMPC_Parameters &psbmpc_pars,
		const Eigen::Matrix<double, 9, -1>& obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities);

	void update_situation_type_and_transitional_variables(
		const PSBMPC_Parameters &psbmpc_pars, 
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const double ownship_length);

public:

	Obstacle_Manager();

	std::shared_ptr<Obstacle_Data> get_data() const { return data; }

	void update_obstacle_status(const Eigen::Matrix<double, 6, 1> &ownship_state);

	void display_obstacle_information();

	void operator()(
		const PSBMPC_Parameters &psbmpc_pars,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const double ownship_length,
		const Eigen::Matrix<double, 9, -1> &obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities);

};

#endif 