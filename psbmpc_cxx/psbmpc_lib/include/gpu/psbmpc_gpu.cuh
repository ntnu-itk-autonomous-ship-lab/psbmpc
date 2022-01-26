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

#include "psbmpc_defines.hpp"
#include "psbmpc_parameters.hpp"
#if OWNSHIP_TYPE == 0
#include "gpu/kinematic_ship_models_gpu.cuh"
#else
#include "gpu/kinetic_ship_models_gpu.cuh"
#endif
#include "gpu/cpe_gpu.cuh"
#include "cpu/mpc_cost_cpu.hpp"
#include "gpu/mpc_cost_gpu.cuh"
#include "gpu/colregs_violation_evaluator.cuh"
#include "cb_cost_functor_structures.cuh"
#include "tml/tml.cuh"

#include <Eigen/Dense>
#include <string>
#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
// Host only due to stderr usage
#define cuda_check_errors(msg)                                 \
	do                                                         \
	{                                                          \
		cudaError_t __err = cudaGetLastError();                \
		if (__err != cudaSuccess)                              \
		{                                                      \
			fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
					msg, cudaGetErrorString(__err),            \
					__FILE__, __LINE__);                       \
			fprintf(stderr, "*** FAILED - ABORTING\n");        \
			exit(1);                                           \
		}                                                      \
	} while (0)

		class CB_Cost_Functor_1;
		class CB_Cost_Functor_2;
		struct CB_Functor_Pars;
		class CB_Functor_Data;
		class Cuda_Obstacle;
		template <typename Parameters>
		class MPC_Cost;

		class PSBMPC
		{
		private:
			Eigen::VectorXd opt_offset_sequence, maneuver_times;

			double u_opt_last;
			double chi_opt_last;

			double min_cost;

			Ownship ownship;

			Eigen::MatrixXd trajectory;

			CPE cpe_gpu;
			COLREGS_Violation_Evaluator colregs_violation_evaluator_gpu;

			//=====================================================
			// Device related objects read/write-ed upon by each
			// GPU thread.
			//=====================================================
			//Device vector of thread indices/ID's for the second cost functor
			thrust::device_vector<int> thread_index_dvec;

			// Device vector of control behaviours
			thrust::device_vector<TML::PDMatrix<float, 2 * MAX_N_M, 1>> cb_dvec;

			// Device vector of control behaviour indices, static obstacle indices, dynamic obstacle indices,
			// dynamic obstacle prediction scenario indices and cpe/colregs_violation_evaluator indices
			thrust::device_vector<int> cb_index_dvec, sobstacle_index_dvec, dobstacle_index_dvec, dobstacle_ps_index_dvec, os_do_ps_pair_index_dvec;

			// Device vector of path related costs for each control behaviour
			thrust::device_vector<float> cb_costs_1_dvec;

			// Device vector of costs, size n_threads x 1. It is the grounding obstacle cost (first tuple element) wrt one static
			// obstacle, and the dynamic obstacle cost (second tuple element) and COLREGS violation cost (third element) when
			// the own-ship  follows a control behaviour with index cb_index, and a dynamic obstacle with index <obstacle_index>,
			// behaves as in prediction scenario <obstacle_ps_index>.
			thrust::device_vector<thrust::tuple<float, float, float>> cb_costs_2_dvec;

			std::unique_ptr<CB_Cost_Functor_1> cb_cost_functor_1;
			std::unique_ptr<CB_Cost_Functor_2> cb_cost_functor_2;

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory_device_ptr;

			CB_Functor_Pars *pars_device_ptr;

			friend class CB_Functor_Data;
			CB_Functor_Data *fdata_device_ptr;

			Cuda_Obstacle *obstacles_device_ptr;

			CPE *cpe_device_ptr;

			Ownship *ownship_device_ptr;

			Basic_Polygon *polygons_device_ptr;

			MPC_Cost<CB_Functor_Pars> *mpc_cost_device_ptr;

			COLREGS_Violation_Evaluator *colregs_violation_evaluators_device_ptr;
			//=====================================================

			void preallocate_device_data();

			bool determine_colav_active(const Dynamic_Obstacles &obstacles, const int n_static_obst, const bool disable);

			void map_offset_sequences();

			void reset_control_behaviour(Eigen::VectorXi &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

			void increment_control_behaviour(Eigen::VectorXi &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

			void map_thrust_dvecs(const Dynamic_Obstacles &obstacles, const Static_Obstacles &polygons);

			void find_optimal_control_behaviour(const Dynamic_Obstacles &obstacles, const Static_Obstacles &polygons);

			void setup_prediction(const Dynamic_Obstacles &obstacles);

			void assign_optimal_trajectory(Eigen::MatrixXd &optimal_trajectory);

			void set_up_temporary_device_memory(
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const double V_w,
				const Eigen::Vector2d &wind_direction,
				const Static_Obstacles &polygons,
				const Dynamic_Obstacles &obstacles);

			void assign_data(const PSBMPC &other);

			void free();

		public:
			PSBMPC_Parameters pars;

			CPU::MPC_Cost<PSBMPC_Parameters> mpc_cost;

			PSBMPC();
			PSBMPC(const Ownship &ownship, const CPU::CPE &cpe, const PSBMPC_Parameters &psbmpc_pars, const CVE_Pars<float> &cve_pars);
			PSBMPC(const PSBMPC &other);

			~PSBMPC();

			PSBMPC &operator=(const PSBMPC &other);

			// Resets previous optimal offsets and predicted own-ship waypoint following
			void reset()
			{
				u_opt_last = 1.0;
				chi_opt_last = 0.0;
				ownship.set_wp_counter(0);
			}

			void calculate_optimal_offsets(
				double &u_opt,
				double &chi_opt,
				Eigen::MatrixXd &predicted_trajectory,
				const double u_d,
				const double chi_d,
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::VectorXd &ownship_state,
				const double V_w,
				const Eigen::Vector2d &wind_direction,
				const Static_Obstacles &polygons,
				const Dynamic_Obstacles &obstacles,
				const bool disable);
		};
	}
}