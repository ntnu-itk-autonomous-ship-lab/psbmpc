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

#include <Eigen/Dense>
#include <vector>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Cuda_Obstacle;
	}

	class Tracked_Obstacle
	{
	private:
		friend class GPU::Cuda_Obstacle;

		int ID;

		// Obstacle dimension quantifiers, length (l) and width (w)
		double A, B, C, D, l, w;

		double x_offset, y_offset;

		// State and covariance at the current time or predicted time
		Eigen::Vector4d xs_0;
		Eigen::Matrix4d P_0;

		// Probability that the obstacle will give-way for the own-ship when specified by COLREGS
		double Pr_WGW;

		// Probability that the obstacle will perform a COLREGS compliant evasive maneuver when supposed to
		double Pr_CCEM;

		// Vector of scenario probabilities at the current time or last time of update
		Eigen::VectorXd Pr_s;

		// If the KF is on, the obstacle is tracked until it dies
		// while the duration lost may be reset if new measurements are aquired
		double duration_tracked, duration_lost;

		// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
		// This is equal for all prediction scenarios including those with active COLAV (using MROU)
		Eigen::MatrixXd P_p;

		// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
		std::vector<Eigen::MatrixXd> xs_p, v_ou_p;

		// Waypoints (predicted) for the obstacle, either planned path communicated from the obstacle
		// or straight line path
		Eigen::MatrixXd waypoints;

		void assign_data(const Tracked_Obstacle &other);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		KF kf;

		Tracked_Obstacle() {}

		Tracked_Obstacle(const Eigen::VectorXd &xs_aug,
						 const Eigen::VectorXd &P,
						 const Eigen::VectorXd &Pr_s,
						 const bool filter_on,
						 const double T,
						 const double dt);

		Tracked_Obstacle(const Eigen::VectorXd &xs_aug,
						 const Eigen::VectorXd &P,
						 const bool filter_on,
						 const double T,
						 const double dt);

		Tracked_Obstacle(
			const std::vector<Eigen::MatrixXd> &xs_p,
			const Eigen::MatrixXd &P_p,
			const Eigen::VectorXd &Pr_s,
			const Eigen::Vector4d &dims,
			const double duration_tracked,
			const double duration_lost,
			const int ID);

		Tracked_Obstacle(
			const std::vector<Eigen::MatrixXd> &xs_p,
			const Eigen::MatrixXd &P_p,
			const Eigen::VectorXd &Pr_s,
			const double Pr_WGW,
			const double Pr_CCEM,
			const Eigen::Vector4d &dims,
			const double duration_tracked,
			const double duration_lost,
			const int ID);

		Tracked_Obstacle(const Tracked_Obstacle &other);

		Tracked_Obstacle &operator=(const Tracked_Obstacle &rhs);

		inline int get_ID() const { return this->ID; };

		inline double get_length() const { return this->l; };

		inline double get_width() const { return this->w; };

		inline Eigen::VectorXd get_scenario_probabilities() const { return this->Pr_s; }
		
		// this->Pr_s = Pr_s / Pr_s.sum();
		inline void set_scenario_probabilities(const Eigen::VectorXd &Pr_s) { this->Pr_s = Pr_s / Pr_s.sum(); }

		inline double get_Pr_WGW() const { return this->Pr_WGW; }

		inline double get_Pr_CCEM() const { return this->Pr_CCEM; }

		inline void set_Pr_WGW(const double &Pr_WGW) { this->Pr_WGW = Pr_WGW; }

		inline void set_Pr_CCEM(const double &Pr_CCEM) { this->Pr_CCEM = Pr_CCEM; }

		// KF related methods
		inline void set_duration_tracked(const double duration_tracked) { this->duration_tracked = duration_tracked; }

		inline double get_duration_tracked() const { return this->duration_tracked; }

		inline void reset_duration_tracked() { duration_tracked = 0.0; }

		inline void set_duration_lost(const double duration_lost) { this->duration_lost = duration_lost; }

		inline double get_duration_lost() const { return this->duration_lost; }

		inline void reset_duration_lost() { duration_lost = 0.0; }

		inline void increment_duration_tracked(const double dt) { duration_tracked += dt; }

		inline void increment_duration_lost(const double dt) { duration_lost += dt; }

		inline std::vector<Eigen::MatrixXd> get_trajectories() const { return this->xs_p; }

		inline std::vector<Eigen::MatrixXd> get_mean_velocity_trajectories() const { return this->v_ou_p; }

		inline Eigen::MatrixXd get_trajectory_covariance() const { return this->P_p; }

		inline Eigen::MatrixXd get_waypoints() const { return this->waypoints; }

		inline void set_trajectories(const std::vector<Eigen::MatrixXd> &xs_p) { this->xs_p = xs_p; }

		inline void set_mean_velocity_trajectories(const std::vector<Eigen::MatrixXd> &v_ou_p) { this->v_ou_p = v_ou_p; }

		inline void set_trajectory_covariance(const Eigen::MatrixXd &P_p) { this->P_p = P_p; }

		inline void set_waypoints(const Eigen::MatrixXd &waypoints) { this->waypoints = waypoints; }

		void update(
			const Eigen::VectorXd &xs_aug,
			const Eigen::VectorXd &P,
			const Eigen::VectorXd &Pr_s,
			const bool filter_on,
			const double dt);

		void update(
			const Eigen::VectorXd &xs_aug,
			const Eigen::VectorXd &P,
			const bool filter_on,
			const double dt);

		void update(
			const bool filter_on,
			const double dt);

		inline void predict(const double dt) { kf.predict(dt); }

		inline Eigen::Vector4d get_state() { return kf.get_state(); }

		inline Eigen::Matrix4d get_covariance() { return kf.get_covariance(); }

		inline void reset_KF(const Eigen::Vector4d &xs_0, const Eigen::Matrix4d &P_0, const double t_0) { kf.reset(xs_0, P_0, t_0); }
	};
}