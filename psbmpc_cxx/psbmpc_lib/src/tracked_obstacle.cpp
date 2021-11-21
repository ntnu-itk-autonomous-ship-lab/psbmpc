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
#include <assert.h>
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

Tracked_Obstacle::Tracked_Obstacle(
	const std::vector<Eigen::MatrixXd> &xs_p, 			// In: Predicted trajectories for the obstacle
	const Eigen::MatrixXd &P_p, 						// In: Predicted covariance trajectory (same for all pred. scenarios) for the obstacle
	const Eigen::VectorXd &Pr_s,						// In: Obstacle scenario probability vector
	const Eigen::Vector4d &dims,						// In: Dimensions A, B, C, D for the obstacle
	const double duration_tracked, 						// In: Duration in seconds that the obstacle has been tracked
	const double duration_lost, 						// In: Duration in seconds that the obstacle has been lost
	const int ID										// In: Obstacle ID		
	) :
	ID(ID),
	A(dims(0)), B(dims(1)), C(dims(2)), D(dims(3)),
	l(dims(0) + dims(1)), w(dims(2) + dims(3)), 
	x_offset(dims(0) - dims(1)), y_offset(dims(3) - dims(2)),
	xs_0(xs_p[0].col(0)), P_0(CPU::reshape(P_p.col(0), 4, 4)),
	Pr_s(Pr_s),
	duration_tracked(duration_tracked), duration_lost(duration_lost),
	P_p(P_p), xs_p(xs_p)
{
	this->kf = KF(xs_0, P_0, ID, 0.0, 0.0); // dt and t_0 are dont care arguments
}

Tracked_Obstacle::Tracked_Obstacle(
	const Tracked_Obstacle &other 			// In: Tracked Obstacle to copy construct
	)
{
	assign_data(other);
}

/****************************************************************************************
*  Name     : operator=
*  Function : 
*  Author   : 
*  Modified : 
*****************************************************************************************/
Tracked_Obstacle& Tracked_Obstacle::operator=(
	const Tracked_Obstacle &rhs
	)
{
	if (this == &rhs)
    {
        return *this;
    }

	assign_data(rhs);

	return *this;
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
/****************************************************************************************
*  Name     : assign_data
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::assign_data(
	const Tracked_Obstacle &other 				// In: Tracked Obstacle to assign data from
	)
{
	this->ID = other.ID;

	this->A = other.A;
	this->B = other.B;
	this->C = other.C;
	this->D = other.D;
	this->l = other.l;
	this->w = other.w;

	this->x_offset = other.x_offset; this->y_offset = other.y_offset;

	this->xs_0 = other.xs_0; this->P_0 = other.P_0;

	this->Pr_s = other.Pr_s;

	this->duration_tracked = other.duration_tracked;
	this->duration_lost = other.duration_lost;

	this->P_p = other.P_p;

	this->xs_p = other.xs_p;
	this->v_ou_p = other.v_ou_p;

	this->waypoints = other.waypoints;

	this->kf = other.kf;
}
}