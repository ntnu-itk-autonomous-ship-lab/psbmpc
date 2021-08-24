/****************************************************************************************
*
*  File name : tracked_obstacle.cpp
*
*  Function  : Tracked obstacle class functions.
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

#include "tracked_obstacle.hpp"
#include "cpu/prediction_obstacle_cpu.hpp"
#include "assert.h"
#include <iostream> 

namespace PSBMPC_LIB
{

/****************************************************************************************
*  Name     : Tracked_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Tracked_Obstacle::Tracked_Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_s,								// In: Obstacle scenario probability vector
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) :
	ID(xs_aug(8)),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6)),
	duration_tracked(0.0), duration_lost(0.0)
{
 	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = CPU::reshape(P, 4, 4); 

	this->kf = KF(xs_0, P_0, ID, dt, 0.0);

	this->Pr_s = Pr_s / Pr_s.sum(); 

 	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	this->xs_p.resize(1);
	this->xs_p[0].resize(4, n_samples);
	this->xs_p[0].col(0) = xs_0;

	this->P_p.resize(16, n_samples);
	this->P_p.col(0) = P; 

	if(filter_on) 
	{
		this->kf.update(xs_0, duration_lost, dt);

		this->duration_tracked = kf.get_time();
	}
}

Tracked_Obstacle::Tracked_Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) :
	ID(xs_aug(8)),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6)),
	duration_tracked(0.0), duration_lost(0.0)
{
 	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = CPU::reshape(P, 4, 4); 

	this->kf = KF(xs_0, P_0, ID, dt, 0.0);

 	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	this->xs_p.resize(1);
	this->xs_p[0].resize(4, n_samples);
	this->xs_p[0].col(0) = xs_0;

	this->P_p.resize(16, n_samples);
	this->P_p.col(0) = P; 

	if(filter_on) 
	{
		this->kf.update(xs_0, duration_lost, dt);

		this->duration_tracked = kf.get_time();
	}
}

/****************************************************************************************
*  Name     : prune_ps
*  Function : Removes prediction data for prediction scenarios not in the 
*			  input vector.
*  Modified : Trym Tengesdal
*****************************************************************************************/
void Tracked_Obstacle::prune_ps(
	const Eigen::VectorXi &ps_indices
	)
{
	int n_ps_old = xs_p.size();
	int n_ps_new = ps_indices.size();
	/* std::cout << "n_ps_new = " << n_ps_new << std::endl;
	std::cout << "n_ps = " << n_ps << std::endl;
	std::cout << "n_ps_independent = " << n_ps_independent << std::endl; */
	std::vector<Eigen::MatrixXd> xs_p_copy(n_ps_new);
	std::vector<Intention> ps_ordering_copy(n_ps_new);

	int ps_count = 0;
	for (int ps = 0; ps < n_ps_old; ps++)
	{
		if (ps == ps_indices(ps_count))
		{
			xs_p_copy[ps_count] = xs_p[ps];

			if (ps_count < n_ps_new - 1)
			{
				ps_count += 1;
			}
		}
	}
	xs_p = xs_p_copy; 
}

/****************************************************************************************
*  Name     : add_intelligent_prediction (CPU PSBMPC only)
*  Function : Only used when obstacle predictions with their own COLAV system is enabled.
*			  Adds prediction data for this obstacle`s intelligent prediction to the set
*			  of trajectories. If an intelligent prediction has already been added before,
*			  it is overwritten.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::add_intelligent_prediction(
	const CPU::Prediction_Obstacle *po,					// In: Pointer to prediction obstacle with intelligent prediction information
	const bool overwrite								// In: Flag to choose whether or not to add the first intelligent prediction, or overwrite the previous
	)
{
	if (overwrite)
	{
		xs_p.back() = po->get_trajectory();
	}
	else
	{
		xs_p.push_back(po->get_trajectory());
	}
}

/****************************************************************************************
*  Name     : update
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::update(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_s,								// In: Obstacle scenario probability vector
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double dt 											// In: Prediction time step
	)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = CPU::reshape(P, 4, 4);

	// Depending on if the KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf.update(xs_0, duration_lost, dt);

		duration_tracked = kf.get_time();

		xs_0 = kf.get_state();

		P_0 = kf.get_covariance();
	}
	else
	{ 
		kf.reset(xs_0, P_0, 0.0); 
	}
	
	this->Pr_s = Pr_s / Pr_s.sum(); 
}

void Tracked_Obstacle::update(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double dt 											// In: Prediction time step
	)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = CPU::reshape(P, 4, 4);

	// Depending on if the KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf.update(xs_0, duration_lost, dt);

		duration_tracked = kf.get_time();

		xs_0 = kf.get_state();

		P_0 = kf.get_covariance();
	}
	else
	{ 
		kf.reset(xs_0, P_0, 0.0); 
	}
}

void Tracked_Obstacle::update(
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double dt 											// In: Prediction time step
	)
{
	// Depending on if the KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf.update(xs_0, duration_lost, dt);

		duration_tracked = kf.get_time();

		xs_0 = kf.get_state();

		P_0 = kf.get_covariance();
	}
	else
	{ 
		kf.reset(xs_0, P_0, 0.0); 
	}
}

/****************************************************************************************
*  Private functions
*****************************************************************************************/
}