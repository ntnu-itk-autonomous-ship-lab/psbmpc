/****************************************************************************************
*
*  File name : tracked_obstacle.h
*
*  Function  : Header file for the tracked obstacle class used by the PSB-MPC
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


#ifndef _TRACKED_OBSTACLE_H_
#define _TRACKED_OBSTACLE_H_

#include <vector>
#include <memory>
#include "Eigen/Dense"
#include "utilities.h"
#include "obstacle.h"
#include "mrou.h"
#include "kf.h"

class Prediction_Obstacle;
class Tracked_Obstacle : public Obstacle
{
private:

	// To make transfer of data between obstacle objects easier
	friend class Prediction_Obstacle;

	// Vector of intention probabilities at the current time or last time of update
	Eigen::VectorXd Pr_a;

	// A priori COLREGS compliance probability at the current time or last time of update
	double Pr_CC;

	// If the AIS-based KF is on, the obstacle is tracked until it dies
	// while the duration lost may be reset if new measurements are aquired
	double duration_tracked, duration_lost;

	// Indicates whether the obstacle breaches COLREGS in a prediction scenario: n_ps x 1
	std::vector<bool> mu;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
	// This is equal for all prediction scenarios including those with active COLAV (using MROU)
	Eigen::MatrixXd P_p;  

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): 
	Eigen::Vector2d v_p;

	// Prediction scenario ordering, size n_ps x 1 of intentions
	std::vector<Intention> ps_ordering;

	// Course change ordering, weights and maneuvering times for the independent prediction scenarios: n_ps x 1
	Eigen::VectorXd ps_course_changes, ps_maneuver_times;

	void assign_data(const Tracked_Obstacle &to);
	
public:

	std::unique_ptr<KF> kf;

	std::unique_ptr<MROU> mrou;

	Tracked_Obstacle() {}

	Tracked_Obstacle(const Eigen::VectorXd &xs_aug, 
			 const Eigen::VectorXd &P, 
			 const Eigen::VectorXd &Pr_a, 
			 const double Pr_CC,
			 const bool filter_on,  
			 const double T, 
			 const double dt);

	Tracked_Obstacle(const Tracked_Obstacle &to);

	Tracked_Obstacle& operator=(const Tracked_Obstacle &rhs);

	inline std::vector<bool> get_COLREGS_violation_indicator() const { return mu; };

	inline double get_a_priori_CC_probability() const { return Pr_CC; };

	inline Eigen::VectorXd get_intention_probabilities() const { return Pr_a; };

	// KF related methods
	inline double get_duration_tracked() const { return duration_tracked; };

	inline void reset_duration_tracked() { duration_tracked = 0.0; };

	inline double get_duration_lost() const { return duration_lost; };

	inline void reset_duration_lost() { duration_lost = 0.0; };

	inline void increment_duration_tracked(const double dt) { duration_tracked += dt; };

	inline void increment_duration_lost(const double dt) { duration_lost += dt; };

	// Trajectory prediction related methods
	void resize_trajectories(const int n_samples);

	inline std::vector<Eigen::MatrixXd> get_trajectories() const { return xs_p; };

	inline Eigen::MatrixXd get_trajectory_covariance() const { return P_p; };

	void initialize_prediction(	
		const std::vector<Intention> &ps_ordering,
		const Eigen::VectorXd &ps_course_changes,
		const Eigen::VectorXd &ps_maneuver_times);

	/****************************************************************************************
	*  Name     : predict_independent_trajectories
	*  Function : Predicts the obstacle trajectories for scenarios where the obstacle
	*			  does not take the own-ship into account.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class MPC_Type>
	void predict_independent_trajectories(
		const double T, 
		const double dt, 
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const MPC_Type &mpc)
	{
		int n_samples = std::round(T / dt);
		resize_trajectories(n_samples);

		int n_ps = ps_ordering.size();
		mu.resize(n_ps);
		
		Eigen::Matrix<double, 6, 1> ownship_state_sl = ownship_state;
		P_p.col(0) = flatten(kf->get_covariance());

		Eigen::Vector2d v_p_new, v_A, v_B, L_AB;
		double chi_ps, t = 0, psi_A, d_AB;
		bool have_turned;
		for(int ps = 0; ps < n_ps; ps++)
		{
			ownship_state_sl = ownship_state;

			v_p(0) = kf->get_state()(2);
			v_p(1) = kf->get_state()(3);

			xs_p[ps].col(0) = kf->get_state();
			
			have_turned = false;	
			for(int k = 0; k < n_samples; k++)
			{
				t = (k + 1) * dt;

				v_B(0) = ownship_state_sl(3);
				v_B(1) = ownship_state_sl(4);
				v_B = rotate_vector_2D(v_B, ownship_state_sl(2));

				psi_A = atan2(xs_p[ps](4), xs_p[ps](0));
				L_AB = xs_p[ps].block<2, 1>(0, k) - ownship_state_sl.block<2, 1>(0, 0);
				d_AB = L_AB.norm();
				L_AB.normalize();

				if (!mu[ps])
				{
					mu[ps] = mpc.determine_COLREGS_violation(v_A, psi_A, v_B, L_AB, d_AB);
				}
			
				switch (ps_ordering[ps])
				{
					case KCC :	
						break; // Proceed
					case SM :
						if (k == ps_maneuver_times[ps] && !have_turned)
						{
							chi_ps = atan2(v_p(1), v_p(0)); 
							v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
							v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
							v_p = v_p_new;
							have_turned = true;
						}
						break;
					case PM : 
						if (k == ps_maneuver_times[ps] && !have_turned)
						{
							chi_ps = atan2(v_p(1), v_p(0)); 
							v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
							v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
							v_p = v_p_new;
							have_turned = true;
						}
						break;
					default :
						std::cout << "This intention is not valid!" << std::endl;
						break;
				}

				if (k < n_samples - 1)
				{
					xs_p[ps].col(k + 1) = mrou->predict_state(xs_p[ps].col(k), v_p, dt);

					if (ps == 0) P_p.col(k + 1) = flatten(mrou->predict_covariance(P_0, t));

					// Propagate ownship assuming straight line trajectory
					ownship_state_sl.block<2, 1>(0, 0) =  ownship_state_sl.block<2, 1>(0, 0) + 
						dt * rotate_vector_2D(ownship_state_sl.block<2, 1>(3, 0), ownship_state_sl(2, 0));
					ownship_state_sl.block<4, 1>(2, 0) = ownship_state_sl.block<4, 1>(2, 0);
				}
			}
		}
	}

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

#endif