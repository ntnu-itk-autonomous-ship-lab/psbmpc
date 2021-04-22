/****************************************************************************************
*
*  File name : tracked_obstacle.hpp
*
*  Function  : Header file for the tracked obstacle class used by the PSB-MPC.
*			   Contains dynamic obstacle information from the tracking system, intention 
*			   inference modules etc., managed by the Obstacle_Manager.
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

#include "cpu/utilities_cpu.hpp"
#include "kf.hpp"
#include "mrou.hpp"

#include <vector>
#include <memory>

namespace PSBMPC_LIB
{
	enum Intention 
	{
		KCC, 					// Keep current course
		SM, 					// Starboard maneuver
		PM 						// Port maneuver
	};

	namespace CPU
	{
		class Prediction_Obstacle;
	}
	namespace GPU
	{
		class Cuda_Obstacle;
		class Prediction_Obstacle;
	}

	class Tracked_Obstacle
	{
	private:

		friend class GPU::Cuda_Obstacle;
		friend class CPU::Prediction_Obstacle;
		friend class GPU::Prediction_Obstacle;

		int ID;

		// Obstacle dimension quantifiers, length (l) and width (w)
		double A, B, C, D, l, w;

		double x_offset, y_offset;

		// State and covariance at the current time or predicted time
		Eigen::Vector4d xs_0;
		Eigen::Matrix4d P_0;

		// Vector of intention probabilities at the current time or last time of update
		Eigen::VectorXd Pr_a;

		// A priori COLREGS compliance probability at the current time or last time of update
		double Pr_CC;

		// If the KF is on, the obstacle is tracked until it dies
		// while the duration lost may be reset if new measurements are aquired
		double duration_tracked, duration_lost;

		// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
		std::vector<bool> mu;

		// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
		// This is equal for all prediction scenarios including those with active COLAV (using MROU)
		Eigen::MatrixXd P_p;  

		// Predicted state and MROU mean velocity for each prediction scenario: n_ps x n x n_samples, where n = 4
		std::vector<Eigen::MatrixXd> xs_p, v_ou_p;

		// Prediction scenario ordering, size n_ps x 1 of intentions
		std::vector<Intention> ps_ordering;

		// Number of prediction scenarios corresponding to intention a = 1, 2, 3, ..., n_a. Typically n_a = 3. 
		Eigen::VectorXi ps_intention_count;
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		KF kf;

		MROU mrou;

		Tracked_Obstacle() {}

		Tracked_Obstacle(const Eigen::VectorXd &xs_aug, 
				const Eigen::VectorXd &P, 
				const Eigen::VectorXd &Pr_a, 
				const double Pr_CC,
				const bool filter_on, 
				const double T, 
				const double dt);

		inline int get_ID() const { return ID; };

		inline double get_length() const { return l; };

		inline double get_width() const { return w; };

		inline std::vector<bool> get_COLREGS_violation_indicator() const { return mu; }

		inline double get_a_priori_CC_probability() const { return Pr_CC; }

		inline Eigen::VectorXd get_intention_probabilities() const { return Pr_a; }

		inline std::vector<Intention> get_ps_ordering() const { return ps_ordering; }

		inline Eigen::VectorXi get_ps_intention_count() const { return ps_intention_count; }
		
		// KF related methods
		inline double get_duration_tracked() const { return duration_tracked; }

		inline void reset_duration_tracked() { duration_tracked = 0.0; }

		inline double get_duration_lost() const { return duration_lost; }

		inline void reset_duration_lost() { duration_lost = 0.0; }

		inline void increment_duration_tracked(const double dt) { duration_tracked += dt; }

		inline void increment_duration_lost(const double dt) { duration_lost += dt; }

		inline std::vector<Eigen::MatrixXd> get_trajectories() const { return xs_p; }
		
		inline std::vector<Eigen::MatrixXd> get_mean_velocity_trajectories() const { return v_ou_p; }

		inline Eigen::MatrixXd get_trajectory_covariance() const { return P_p; }

		void setup_prediction(
			const std::vector<Eigen::MatrixXd> &xs_p,	
			const std::vector<Eigen::MatrixXd> &v_ou_p,	
			const Eigen::MatrixXd &P_p,	
			const std::vector<Intention> &ps_ordering);

		void prune_ps(const Eigen::VectorXi &ps_indices);

		void add_intelligent_prediction(const CPU::Prediction_Obstacle *po, const bool overwrite); // only used in the CPU PSBMPC implementation

		void update(
			const Eigen::VectorXd &xs_aug, 
			const Eigen::VectorXd &P, 
			const Eigen::VectorXd &Pr_a, 
			const double Pr_CC,
			const bool filter_on,
			const double dt);

		void update(
			const bool filter_on,
			const double dt);
	};
}