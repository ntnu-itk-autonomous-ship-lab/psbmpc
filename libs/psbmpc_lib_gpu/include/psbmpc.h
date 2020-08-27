/****************************************************************************************
*
*  File name : psbmpc.h
*
*  Function  : Header file for Probabilistic Scenario-based Model Predictive Control.
*			   Brand new extended and improved version of the SB-MPC implemented by  
*			   Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor through the Autosea 
*			   project.
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
#include "ownship.cuh"
#include "tracked_obstacle.h"
#include "cpe.cuh"
#include "Eigen/Dense"
#include <vector>
#include <memory>

class CB_Cost_Functor;

enum ST 
{
	A, 														// Non-COLREGS situation	(ST = Ø)
	B, 														// Stand-on in Overtaking 	(ST = OT, SO)
	C, 														// Stand-on in Crossing 	(ST = CR, SO)
	D, 														// Give-way in Overtaking 	(ST = OT, GW)
	E, 														// Give-way in Head-on 		(ST = HO, GW)
	F 														// Give-way in Crossing 	(ST = CR, GW)
};	



class PSBMPC
{
private:

	std::vector<int> n_ps;


	Eigen::VectorXd maneuver_times;
	
	double u_m_last;
	double chi_m_last;

	double min_cost;
	int min_index;

	std::unique_ptr<Ownship> ownship;

	std::unique_ptr<CPE> cpe;

	Eigen::Matrix<double, 6, -1> trajectory;

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	std::vector<bool> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0; 

	// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
	std::vector<ST> ST_0, ST_i_0;

	std::vector<std::unique_ptr<Tracked_Obstacle>> old_obstacles;
	std::vector<std::unique_ptr<Tracked_Obstacle>> new_obstacles;

	// Thrust functor
	friend class CB_Cost_Functor;
	std::unique_ptr<CB_Cost_Functor> op;

	void map_offset_sequences();

	void reset_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void increment_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void initialize_par_limits();

	void initialize_pars();

	void initialize_prediction();

	void set_up_independent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const int i,
		const int n_turns);

	void set_up_dependent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const int i);

	double find_time_of_passing(const int i);	

	bool determine_colav_active(const int n_static_obst);

	void determine_situation_type(
		ST &st_A,
		ST &st_B,
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

    void update_obstacles(
		const Eigen::Matrix<double, 9, -1>& obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities);

	void update_obstacle_status(Eigen::Matrix<double,-1,-1> &obstacle_status, const Eigen::VectorXd &HL_0);

	void update_situation_type_and_transitional_variables();

public:

	// This object is public to allow online access (have to figure this out yet!)
	PSBMPC_Parameters pars;

	PSBMPC();

	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		Eigen::Matrix<double, -1, -1> &obstacle_status,
		Eigen::Matrix<double, -1, 1> &colav_status,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const Eigen::Matrix<double, 9, -1> &obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities,
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

};

#endif 