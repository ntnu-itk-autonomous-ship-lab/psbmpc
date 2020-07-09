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
	const Eigen::VectorXd& xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::Matrix4d& P, 									// In: Obstacle covariance
	const Eigen::VectorXd Pr_a,									// In: Obstacle intention probability vector
	const double Pr_cc, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Boolean determining whether KF should be used or not
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	ID(xs_aug(8)), P_0(P), filter_on(filter_on), colav_on(colav_on),
	l(xs_aug(4) + xs_aug(5)), w(xs_aug(6) + xs_aug(7)), 
	x_offset(xs_aug(4) - xs_aug(5)), y_offset(xs_aug(7) - xs_aug(6)),
	duration_lost(0.0)
{
	this->Pr_a = Pr_a;

	this->Pr_CC = Pr_CC;

	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	xs_p.resize(1);
	xs_p[0].resize(4, n_samples);

	P_p.resize(1);
	P_p[0].resize(16, n_samples);

	xs_colav_p.resize(4, n_samples);

	P_colav_p.resize(16, n_samples);

	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);
	
	xs_p[0].col(0) = xs_0;

	P_p[0].col(0) = flatten(P_0);

	kf = new KF(xs_0, P_0, ID, dt, 0.0);

	if(filter_on) 
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}

	mrou = new MROU(0.8, 0, 0.8, 0.1, 0.1);
}

/****************************************************************************************
*  Name     : ~Obstacle
*  Function : Class destructor, clears the dynamic kalman filter object
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::~Obstacle()
{
	delete kf;
	delete mrou;
}

/****************************************************************************************
*  Name     : resize_trajectories
*  Function : Resizes trajectory and preserves old values inside new range of samples
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::resize_trajectories(const int n_samples)
{
	int n_ps = ps_ordering.size();
	xs_p.resize(n_ps);
	P_p.resize(n_ps);
	v_p.resize(n_ps);
	for(int ps = 0; ps < n_ps; ps++)
	{
		xs_p[ps].resize(4, n_samples); 	xs_p[ps].col(0) = kf->get_state();
		P_p[ps].resize(16, n_samples); 	P_p[ps].col(0) 	= flatten(kf->get_covariance());
		v_p[ps].resize(2, n_samples); 	v_p[ps].col(0) 	= kf->get_state().block<2, 1>(2, 0);
	}
	xs_colav_p.resize(4, n_samples); 	xs_colav_p.col(0) = kf->get_state();
	P_colav_p.resize(16, n_samples); 	P_colav_p.col(0) = flatten(kf->get_covariance());

}

/****************************************************************************************
*  Name     : initialize_independent_prediction
*  Function : Sets up prediction scenario ordering, maneuvering times for the different
* 			  obstacle alternative maneuvers
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::initialize_independent_prediction(
	const std::vector<Intention> &ps_ordering, 						// In: Prediction scenario ordering
	const Eigen::VectorXd &ps_course_changes, 						// In: Order of alternative maneuvers for the prediction scenarios
	const Eigen::VectorXd &ps_weights,	 							// In: The different course changes employed in the prediction scenarios
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
	Eigen::Matrix4d P_0 = kf->get_covariance();
	double chi_ps, t = 0;
	bool have_turned;
	for(int ps = 0; ps < n_ps; ps++)
	{
		ownship_state_sl = ownship_state;

		v_p[ps](0, 0) = kf->get_state()(2);
		v_p[ps](1, 0) = kf->get_state()(3);
		xs_p[ps].col(0) = kf->get_state();
		P_p[ps].col(0) = flatten(P_0);
		have_turned = false;
		for(int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;

			mu[ps] = determine_COLREGS_violation(xs_p[ps].col(k), ownship_state_sl.col(k), phi_AH, phi_CR, phi_HO, phi_OT, d_close, d_safe);
			switch (ps_ordering[ps])
			{
				case KCC :	// Proceed
				case SM || PM:
					if (k == ps_maneuver_times[ps] && !have_turned)
					{
						chi_ps = atan2(v_p[ps](1, k), v_p[ps](0, k)); 
						v_p[ps](0, k) = v_p[ps].col(k).norm() * cos(chi_ps + ps_course_changes[ps]);
						v_p[ps](1, k) = v_p[ps].col(k).norm() * sin(chi_ps + ps_course_changes[ps]);
						have_turned = true;
					}
					break;
				default :
					std::cout << "This intention is not valid!" << std::endl;
					break;
			}

			if (k < n_samples - 1)
			{
				xs_p[ps].col(k + 1) = mrou->predict_state(xs_p[ps].col(k), v_p[ps].col(k), dt);
				P_p[ps].col(k + 1) = flatten(mrou->predict_covariance(P_0, t));
				v_p[ps].col(k + 1) = v_p[ps].col(k);

				// Propagate ownship assuming straight line trajectory
				ownship_state_sl.block<2, 1>(0, k + 1) =  ownship_state_sl.block<2, 1>(0, k) + 
					dt * rotate_vector_2D(ownship_state_sl.block<2, 1>(3, k), ownship_state_sl(2, k));
				ownship_state_sl.block<4, 1>(2, k + 1) = ownship_state_sl.block<4, 1>(2, k);
			}
		}
	}
}

/****************************************************************************************
*  Name     : update
*  Function : Updates the obstacle state, covariance, intention probabilities, a priori
* 			  COLREGS compliance probability at the current time prior to a run of the
*			  PSB-MPC.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::update(
	const Eigen::VectorXd& xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::Matrix4d& P, 									// In: Obstacle covariance
	const Eigen::VectorXd Pr_a,									// In: Obstacle intention probability vector
	const double Pr_cc, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 											// In: Indicator of whether the AIS-KF is active
	const double dt 											// In: Prediction time step
	)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = P;

	if (filter_on)
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}
	else { kf->reset(xs_0, P_0, 0.0); }
	
	this->Pr_a = Pr_a; 
	
	this->Pr_CC = Pr_CC;
}