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
//#include "cuda_obstacle.cuh"
#include "ownship.cuh"
#include "cpe.cuh"

#include "Eigen/Dense"
#include <vector>
#include <memory>

// Host only due to stderr usage
#define cuda_check_errors(msg) \
    do { \
        cudaError_t __err = cudaGetLastError(); \
        if (__err != cudaSuccess) { \
            fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
                msg, cudaGetErrorString(__err), \
                __FILE__, __LINE__); \
            fprintf(stderr, "*** FAILED - ABORTING\n"); \
            exit(1); \
        } \
    } while (0)

class CB_Cost_Functor;
class CB_Functor_Pars;
class CB_Functor_Data;
class Cuda_Obstacle;

class PSBMPC
{
private:

	std::vector<int> n_ps;

	Eigen::VectorXd maneuver_times;

	thrust::device_vector<TML::PDMatrix<float, 2 * MAX_N_M, 1>> control_behavior_dvec;
	
	double u_opt_last;
	double chi_opt_last;

	double min_cost;
	int min_index;

	Ownship ownship;

	Eigen::Matrix<double, 6, -1> trajectory;

	CPE_CPU cpe_host;

	std::vector<Prediction_Obstacle> pobstacles;

	bool use_joint_prediction;

	//=====================================================
	// Device related objects read/write-ed upon by each
	// GPU thread.
	//=====================================================
	std::unique_ptr<CB_Cost_Functor> cb_cost_functor;

	TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory_device_ptr;

	CB_Functor_Pars *pars_device_ptr;

	friend struct CB_Functor_Data;
	CB_Functor_Data *fdata_device_ptr;

	Cuda_Obstacle *obstacles_device_ptr;
	Prediction_Obstacle *pobstacles_device_ptr;

	CPE_GPU *cpe_device_ptr;
	//=====================================================

	void map_offset_sequences();

	void reset_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void increment_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

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

	void predict_trajectories_jointly(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1>& static_obstacles);

	bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

	void set_up_temporary_device_memory(
		const double u_d,
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		const Obstacle_Data<Tracked_Obstacle> &data);

	void clear_temporary_device_memory();

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
		Obstacle_Data<Tracked_Obstacle> &data);

};

#endif 