/****************************************************************************************
*
*  File name : obstacle.cpp
*
*  Function  : Obstacle class functions. Modified version of the one created for SBMPC 
*			   by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor through the 
*  			   Autosea project.
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


#include "obstacle.h"
#include "obstacle_sbmpc.h"
#include "utilities.h"
#include "kf.h"
#include "iostream" 

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_a,								// In: Obstacle intention probability vector
	const double Pr_CC, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Boolean determining whether KF should be used or not
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	ID(xs_aug(8)), colav_on(colav_on),
	A(xs_aug(4)), B(xs_aug(5)), C(xs_aug(6)), D(xs_aug(7)),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6)),
	duration_lost(0.0)
{
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }

	P_0 = reshape(P, 4, 4);

	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	xs_p.resize(1);
	xs_p[0].resize(4, n_samples);

	P_p.resize(16, n_samples);
	P_p.col(0) = P;

	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);
	
	xs_p[0].col(0) = xs_0;

	kf = new KF(xs_0, P_0, ID, dt, 0.0);

	if(filter_on) 
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}

	mrou = new MROU(0.8, 0, 0.8, 0.1, 0.1);

	sbmpc = new Obstacle_SBMPC();
}

/****************************************************************************************
*  Name     : Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Obstacle &o 													// In: Obstacle to copy
	)
{
	// Boring non-pointer class member copy
	this->ID = o.ID;

	this->colav_on = o.colav_on;

	this->Pr_a = o.Pr_a;

	this->Pr_CC = o.Pr_CC;

	this->A = o.A; this->B = o.B; this->C = o.C; this->D = o.D;
	this->l = o.l; this->w = o.w;

	this->x_offset = o.x_offset; this->y_offset = o.y_offset;

	this->duration_tracked = o.duration_tracked;
	this->duration_lost = o.duration_lost;

	this->xs_0 = o.xs_0;
	this->P_0 = o.P_0;

	this->mu = o.mu;

	this->P_p = o.P_p;
	this->xs_p = o.xs_p;
	this->v_p = o.v_p;

	this->ps_ordering = o.ps_ordering;
	this->ps_course_changes = o.ps_course_changes;
	this->ps_weights = o.ps_weights;
	this->ps_maneuver_times = o.ps_maneuver_times;

	// Allocate memory and use copy constructors to perform correct copy of the class pointers
	// such that the pointers do not point to the same objects as "o" does.
	this->kf = new KF(*(o.kf));
	this->mrou = new MROU(*(o.mrou));
	this->sbmpc = new Obstacle_SBMPC(*(o.sbmpc));
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle& Obstacle::operator=(
	const Obstacle &o 													// In: Rhs Obstacle to assign to lhs
	)
{
	if (this == &o)
	{
		return *this;
	}
	// Free memory from old dynamic allocation
	if (kf != NULL) 	{ delete kf; }
	if (mrou != NULL) 	{ delete mrou; }
	if (sbmpc != NULL) 	{ delete sbmpc; }

	return *this = Obstacle(o);
}

/****************************************************************************************
*  Name     : ~Obstacle
*  Function : Class destructor, clears the dynamically allocated objects.
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::~Obstacle()
{
	delete kf;
	delete mrou;
	delete sbmpc;
}

/****************************************************************************************
*  Name     : resize_trajectories
*  Function : Resizes independent trajectories, and also the trajectory covariance
*			  which is the same for all trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::resize_trajectories(const int n_samples)
{
	int n_ps = ps_ordering.size();
	xs_p.resize(n_ps);
	for(int ps = 0; ps < n_ps; ps++)
	{
		xs_p[ps].resize(4, n_samples); 	xs_p[ps].col(0) = kf->get_state();
	}
	P_p.resize(16, n_samples);
	P_p.col(0) 	= flatten(kf->get_covariance());
}

/****************************************************************************************
*  Name     : initialize_prediction
*  Function : Sets up independent or dependent obstacle prediction, depending on if
*		      colav is active or not. For the dependent obstacle prediction,
*			  ps_course_changes and ps_maneuver_times are "dont care" variables, hence
*		      two overloads.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::initialize_prediction(
	const std::vector<Intention> &ps_ordering, 						// In: Prediction scenario ordering
	const Eigen::VectorXd &ps_course_changes, 						// In: Order of alternative maneuvers for the prediction scenarios
	const Eigen::VectorXd &ps_weights,	 							// In: The cost function weights for the prediction scenarios
	const Eigen::VectorXd &ps_maneuver_times 						// In: Time of alternative maneuvers for the prediction scenarios	
	)
{
	this->ps_ordering = ps_ordering;
	
	this->ps_course_changes = ps_course_changes;

	this->ps_weights = ps_weights;

	this->ps_maneuver_times = ps_maneuver_times;
}

/****************************************************************************************
*  Name     : predict_independent_trajectories
*  Function : Predicts the obstacle trajectories for scenarios where the obstacle
*			  does not take the own-ship into account. PSBMPC parameters needed 
* 			  to determine if obstacle breaches cOLREGS (future: implement simple 
*			  sbmpc class for obstacle which has the "determine COLREGS violation" function)
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::predict_independent_trajectories(						
	const double T, 											// In: Time horizon
	const double dt, 											// In: Time step
	const Eigen::Matrix<double, 6, 1> &ownship_state, 			// In: State of own-ship to use for COLREGS penalization calculation
	const double phi_AH,
	const double phi_CR,
	const double phi_HO,
	const double phi_OT,
	const double d_close,
	const double d_safe
	)
{
	int n_samples = std::round(T / dt);
	resize_trajectories(n_samples);

	int n_ps = ps_ordering.size();
	mu.resize(n_ps);
	
	Eigen::Matrix<double, 6, 1> ownship_state_sl = ownship_state;
	P_p.col(0) = flatten(kf->get_covariance());

	Eigen::Vector2d v_p_new;
	double chi_ps, t = 0;
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

			if (!mu[ps])
			{
				//mu[ps] = sbmpc->determine_COLREGS_violation(xs_p[ps].col(k), ownship_state_sl);
				mu[ps] = determine_COLREGS_violation(xs_p[ps].col(k), ownship_state_sl, phi_AH, phi_CR, phi_HO, phi_OT, d_close, d_safe);
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

/****************************************************************************************
*  Name     : update
*  Function : Updates the obstacle state, covariance, intention probabilities, a priori
* 			  COLREGS compliance probability at the current time prior to a run of the
*			  PSB-MPC. Two overloads depending on if obstacle is lost or not, respectively
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::update(
	const bool filter_on, 										// In: Indicator of whether the AIS-KF is active
	const double dt 											// In: Prediction time step
	)
{
	// Depending on if the AIS-based KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();

		xs_0 = kf->get_state();

		P_0 = kf->get_covariance();
	}
	else
	{ 
		kf->reset(xs_0, P_0, 0.0); 
	}
	std::cout << "Inside obstacle update 1: xs_0 = " << xs_0.transpose() << std::endl;
}

void Obstacle::update(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_a,								// In: Obstacle intention probability vector
	const double Pr_CC, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Indicator of whether the AIS-KF is active
	const double dt 											// In: Prediction time step
	)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = reshape(P, 4, 4);

	// Depending on if the AIS-based KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();

		xs_0 = kf->get_state();

		P_0 = kf->get_covariance();
	}
	else
	{ 
		kf->reset(xs_0, P_0, 0.0); 
	}
	std::cout << "Inside obstacle update 2: xs_0 = " << xs_0.transpose() << std::endl;
	
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }
}