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

#pragma once

#include "psbmpc_parameters.h"
#include "cpu/cpe_cpu.h"
#include "cpu/mpc_cost_cpu.h"
#include "cb_cost_functor_structures.cuh"

namespace PSBMPC_LIB
{	
	namespace GPU
	{
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

		class CB_Cost_Functor_1;
		class CB_Cost_Functor_2;
		class CB_Functor_Pars;
		class CB_Functor_Data;
		
		class CPE;
		class Cuda_Obstacle;
		class Obstacle_SBMPC;
		template <typename Parameters> class MPC_Cost;

		class PSBMPC
		{
		private:

			std::vector<int> n_ps;

			Eigen::VectorXd opt_offset_sequence, maneuver_times;

			double u_opt_last;
			double chi_opt_last;

			double min_cost;
			int min_index;

			Ownship ownship;

			Eigen::MatrixXd trajectory;

			CPU::CPE cpe_host;

			std::vector<Prediction_Obstacle> pobstacles;

			bool use_joint_prediction;

			//=====================================================
			// Device related objects read/write-ed upon by each
			// GPU thread.
			//=====================================================
			//Device vector of thread indices/ID's, size n_threads x 1
			thrust::device_vector<unsigned int> thread_index_dvec;

			// Device vector of control behaviours, size n_threads x 1
			thrust::device_vector<TML::PDMatrix<float, 2 * MAX_N_M, 1>> cb_dvec;

			// Device vector of control bevhaviour indices, obstacle indices, obstacle prediction scenario indices,
			// intelligent obstacle prediction scenario indices,  size n_threads x 1
			thrust::device_vector<unsigned int> cb_index_dvec, obstacle_index_dvec, obstacle_ps_index_dvec;
			thrust::device_vector<int> jp_obstacle_ps_index_dvec;

			// Device vector of costs, size n_cbs x 1, consisting of the static obstacle and path related costs for each control behaviour
			thrust::device_vector<float> cb_costs_1_dvec;
			// Device vector of costs, size n_threads x 1. It is the dynamic obstacle cost when the own-ship
			// follows a control behaviour with index cb_index, and a dynamic obstacle with index <obstacle_index>, behaves as in
			// prediction scenario <obstacle_ps_index>. The intention and COLREGS violation indicator for the intelligent prediction scenario is given as the
			// second and third element
			thrust::device_vector<thrust::tuple<float, Intention, bool>> cb_costs_2_dvec;
			
			std::unique_ptr<CB_Cost_Functor_1> cb_cost_functor_1;
			std::unique_ptr<CB_Cost_Functor_2> cb_cost_functor_2;

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory_device_ptr;

			CB_Functor_Pars *pars_device_ptr;

			friend struct CB_Functor_Data;
			CB_Functor_Data *fdata_device_ptr;

			Cuda_Obstacle *obstacles_device_ptr;
			Prediction_Obstacle *pobstacles_device_ptr;

			CPE *cpe_device_ptr;

			Ownship *ownship_device_ptr;

			Obstacle_Ship *obstacle_ship_device_ptr;

			Obstacle_SBMPC *obstacle_sbmpc_device_ptr;

			MPC_Cost<CB_Functor_Pars> *mpc_cost_device_ptr;
			//=====================================================
			bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);
			
			void map_offset_sequences();

			void reset_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

			void increment_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

			void map_thrust_dvecs();

			void find_optimal_control_behaviour(Obstacle_Data<Tracked_Obstacle> &data);

			void initialize_prediction(Obstacle_Data<Tracked_Obstacle> &data);

			void set_up_independent_obstacle_prediction(
				std::vector<Intention> &ps_ordering,
				Eigen::VectorXd &ps_course_changes,
				Eigen::VectorXd &ps_maneuver_times,
				const double t_cpa_i,
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

			void determine_situation_type(
				ST& st_A,
				ST& st_B,
				const TML::Vector2f &v_A,
				const float psi_A,
				const TML::Vector2f &v_B,
				const TML::Vector2f &L_AB,
				const float d_AB);

			void update_conditional_obstacle_data(Obstacle_Data_GPU_Friendly &data, const int i_caller, const int k);

			void predict_trajectories_jointly(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1>& static_obstacles);

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

			CPU::MPC_Cost<PSBMPC_Parameters> mpc_cost;

			PSBMPC();

			~PSBMPC();

			void calculate_optimal_offsets(
				double &u_opt, 
				double &chi_opt, 
				Eigen::Matrix<double, 2, -1> &predicted_trajectory,
				const double u_d, 
				const double chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::VectorXd &ownship_state,
				const Eigen::Matrix<double, 4, -1> &static_obstacles,
				Obstacle_Data<Tracked_Obstacle> &data);

		};
	}
}