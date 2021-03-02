/**
 * \file   drvoAgent.cpp
 * \brief  Defines the DRVO Agent class.
 */

#ifndef DRVO_AGENT_H_
#include "drvoAgent.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef DRVO_KD_TREE_H_
#include "KdTree.h"
#endif

#include "ship_model.h"
#include "iostream" // for test printout!

static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;

const float DRVO_PI = 3.141592653589793f;


	Agent::Agent() 
	{ 
 
		// General parameter defaults:
		T_ = 30.0; //00.0; 		// 400.0, 300.0 (sim)
		T_HD_ = 30.0; 		// 30.0, 15.0
		DT_ = 0.5;		// 5.0, 0.5 (sim) 
    		T_stat_ = T_HD_;	// (50.0)
    		
		MAXDCHI_ = 45.0;	// 45.0 [deg] 90 = 3deg/s * 30s

		D_INIT_ = 1852.0;	// 600 1852 (use 1000 to simulate "obstacle acting first"!)
		D_CLOSE_ = 1000.0;	// 400 1000 D_PROACT
		D_REACT_ = 600.0;	// 200 600
		D_CRIT_ = 400.0;	// 100 400
		D_SAFE_ = 185.2; 	// 40 200 (change obst radius!)

		T_HOLD_ = 60.0; 	//  [s] 
 
		PHI_AH_ = 68.5;		// 15.0, 22.5, 68.5 or more??
		PHI_OT_ = 68.5;		// 68.5
		PHI_HO_ = 22.5;		// 22.5 68.5 89.5 (> is large enough to trigger OT scenario)
		PHI_CR_ = 68.5;		// 68.5 22.5
		G_ = 1.0e3;	        // 1.0e3
		
		OBST_FILTER_ON_ = 0; 	// 1=ON, 0=OFF!

		P_last_ = -1;		// should be the current measurement at start up
		Chi_last_ = -1;		// should be the current measurement at start up		
		hold_last_1 = true;

		cost_ = INFINITY;

		// MPC parameters

		P_ = 1.0; 		// 1.0, 2.0
		Q_ = 4.0;		// 4.0
		K_COLL_ = 0.5;		//   0.5, 1.0, 0.1; 
		KAPPA_ = 3.0;		// 3.0
		
		K_P_ = 10.5;		// 2.5, 10.5 100.5		
		K_DP_ = 0.5;		// 2.0, 0.5 (new cost requires <1)
		
		K_CHI_ = 1.5;		// 1.5
		K_DCHI_ = 0.5;		// new for VO: 0.5, 
		
		K_CHI_SB_ = 1.5;	// new 1.5
		K_CHI_P_ = 1.5;		// new: 100.5, 10.5, 5.5, 2.5, 1.5 (>1.5 red. flexibility to port)
		K_DCHI_SB_ = 0.5;	// 0.9, 0.5 (new cost requires <1) 
		K_DCHI_P_ = 0.5;	// 1.2, 0.9 (new cost requires <1) 

		P_ca_last_ = 1;
		Chi_ca_last_ = 0;
		Chi_ca_last_0 = 0;
		Chi_ca_0.resize(13);
		Chi_ca_0 << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0; // grid around psi_str
		Chi_ca_0 *= DEG2RAD;
		P_ca_.resize(3);
		P_ca_ << 0.0, 0.5, 1.0; // 2 means max speed!
		method = EulerFirstOrder;  //LinearPrediction EulerFirstOrder

		// Agent-specific and environment defaults:
		prefSpeed_ = 5.0f;	// [m/s]
		maxSpeed_ = 15.0f; 	// [m/s] overwritten by u_d in update fxn!
		maxAccel_ = 1.5f; 	// std::numeric_limits<float>::infinity(); 	// 1.5f [m/s²] 
		maxTurnRate_ = 0.0f; 	// (NB!!*** 0.0 disables use of Omega_int and Omega_proj sets) std::numeric_limits<float>::infinity(); 	// 3.0f [deg/s], was 1.5f  
		orientation_ = 0.0f; 	// [rad]
		radius_ = 5.0; 		// [m] asv->getL()/2
		neighborDist_ = D_INIT_; // [m] D_INIT_?
		maxNeighbors_ = 100;	 
		uncertaintyOffset_ = 1.0f; // 0.0f 

		kdTree_ = new KdTree(this); // new KdTree(this), where "this" is this Agent.
		asv = new shipModel(T_,DT_);

	}


	Agent::Agent(const Vector2 &position, const Vector2 &velocity, float prefSpeed, float maxSpeed, float maxAccel, float orientation, float radius, float neighborDist, std::size_t maxNeighbors, float uncertaintyOffset) : position_(position), velocity_(velocity), newVelocity_(velocity), prefSpeed_(prefSpeed), maxSpeed_(maxSpeed), maxAccel_(maxAccel), orientation_(orientation), radius_(radius), neighborDist_(neighborDist), maxNeighbors_(maxNeighbors), uncertaintyOffset_(uncertaintyOffset)
	{ 
		// General parameter defaults:
		T_ = 300.0; 		// 400.0, 300.0 (sim)
		DT_ = 0.5;		// 5.0, 0.5 (sim)

		kdTree_ = new KdTree(this); 
		asv = new shipModel(T_,DT_);
	}


	Agent::~Agent()
	{
		delete kdTree_;
		kdTree_ = NULL;
		delete asv;
		asv = NULL;
	}


	//void Agent::getBestControlInput(double &u_best, double &psi_best, double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state, const Eigen::Matrix<double,-1,10>& obst_states, ofstream &log_asv, ofstream &log_obst_1, ofstream &log_obst_2, ofstream &log_obst_3){
	
	void Agent::getBestControlOffset(double &u_os_best, double &psi_os_best, double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state, const Eigen::Matrix<double,-1,10>& obst_states, const Eigen::Matrix<double,-1,4>& static_obst, const Eigen::Matrix<double,-1,2>& next_waypoints){

		double u_str, psi_str, u_best, psi_best; 
		double cost = INFINITY;
		double cost_i = 0;
		double cost_k;
		Eigen::Vector2d d, v_o, v_s, los_0;
		Eigen::Vector3d v3_o(0,0,0), v3_s(0,0,0), v3_x;
		double dist_k, dist = INFINITY;
		static bool colav_active=false;
		double phi_0; 
		static int sample=1;
		psi_d = normalize_angle(psi_d);

		// update internal agent states
		update(u_d, psi_d, asv_state);
		

		//** Start of ** standard initialization and check of current state or COLREGS behavior
		//=====================================================================================
 
		if (obst_states.rows() == 0){
			u_best = u_d;
			psi_best = psi_d;
			P_ca_last_ = 1;
			Chi_ca_last_ = 0;
			Chi_ca_last_0 = 0;

			return;
		}else{			
			
			// Update list 
			for (int i = 0; i < obst_states.rows(); i++){
				bool obst_exists = false;
				// iterate through old obstacle list
				/**/
				for (int j = 0; j < oldObstacles_.size(); j++){
					
					// does obstacle exist in old obstacle list?
					if( (double)oldObstacles_[j]->id_ == obst_states(i,9)){
						// update object in old list and move it to the new list,
						
						oldObstacles_[j]->updateTrajectory(obst_states.row(i), OBST_FILTER_ON_); // update obstacle
						
						obstacles_.push_back(oldObstacles_[j]); // copy updated obstacle
						
						oldObstacles_.erase(oldObstacles_.begin()+j);	// does not destroy this object, but the pointer to it!
						
						obst_exists = true; 
		
						//std::cout << "Yey!! obstacle exists!" << std::endl; // check test!
						
						break;
					}				
				
				}
				
				if (true && !obst_exists){
					obstacle *obst = new obstacle(obst_states.row(i), T_, DT_, OBST_FILTER_ON_);
					obstacles_.push_back(obst);
					//std::cout << "Ney!! new obstacle!" << std::endl; // check test!
				}
			}
			
			n_obst = obstacles_.size();
			std::cout << "n_obst: " << n_obst << std::endl; // check test!
			
			// delete and clear remaining (terminated) elements in the old list 			
			int n_obst_old = oldObstacles_.size(); 
			std::cout << "n_obst_old: " << n_obst_old << std::endl; // check test!
			for (int k = 0; k < n_obst_old; k++){
				delete(oldObstacles_[k]);
			}
			oldObstacles_.clear();
			
		}

		// compute new behavior only if an obstacle enters the colav range!
		for (int k = 0; k < n_obst; k++){
			d(0) = obstacles_[k]->x_[0] - asv_state(0); 
			d(1) = obstacles_[k]->y_[0] - asv_state(1); 
			dist_k = d.norm();
			if (dist_k < dist) dist = dist_k;
			if (dist < D_INIT_) colav_active=true; 
		}

		// reset colav_active if static
		if (colav_active && dist > D_INIT_+D_SAFE_) colav_active=false; // avoids switching on/off 

		if (colav_active==false){
			u_best = u_d;
			psi_best = psi_d;
			P_ca_last_ = 1;
			Chi_ca_last_ = 0;
			Chi_ca_last_0 = 0;

			for (int k = 0; k < n_obst; k++){ 
				delete(obstacles_[k]);
			}
			obstacles_.clear();
		
			std::cout << "obs! obstacles were cleared " << std::endl; // check test!

			return;
		}

		// for saving current side (starboard/port, ahead/behind) of asv where an obstacle is located
		SB_0.resize(n_obst);
		AH_0.resize(n_obst);

		// for the current transition state of asv
		CRG_0.resize(n_obst);
		OTG_0.resize(n_obst);
		OT_0.resize(n_obst);
		HO_0.resize(n_obst);

		// for the current distance of obstacles from asv
		dist_0.resize(n_obst);
		
		OBS_OUT_OF_WAY_0.resize(n_obst); 	PHI_0.resize(n_obst);

		// current velocity vector of asv
		v_s(0) = asv_state(3);
		v_s(1) = asv_state(4);
		rot2d(normalize_angle(asv_state(2)),v_s);
			
		v3_s(0) = v_s(0); 
		v3_s(1) = v_s(1);
		

		for (int k = 0; k < n_obst; k++){
				
			std::cout << "Obstacle " << obstacles_[k]->id_ << std::endl; // check test!
			std::cout << "==========" << std::endl; // check test!
			
			d(0) = obstacles_[k]->x_[0] - asv_state(0); 
			d(1) = obstacles_[k]->y_[0] - asv_state(1); 
			dist_0(k) = d.norm();

			los_0 = d/dist_0(k);
			std::cout << "dist_0 : " << dist_0(k) << std::endl; // check test!

			phi_0 = atan2(d(1),d(0)) - normalize_angle(asv_state(2)); 
			phi_0 = normalize_angle(phi_0);	PHI_0(k)=phi_0*RAD2DEG;	std::cout << "PHI_0 : " << PHI_0(k) << std::endl; // check test!	

			// obstacle at starboard/port side
			SB_0(k) = phi_0 < 0;
			std::cout << "SB_0 : " << SB_0(k) << std::endl; // check test!

	 		v_o(0) = obstacles_[k]->u_[0];
			v_o(1) = obstacles_[k]->v_[0];
			rot2d(obstacles_[k]->psi_,v_o);
						
			v3_o(0) = v_o(0); 	
			v3_o(1) = v_o(1); 
			

			// obstacle ahead of asv
			AH_0(k) = v_s.dot(los_0) > cos(90.0*DEG2RAD)*v_s.norm();
			std::cout << "AH_0 : " << AH_0(k) << std::endl; // check test!

			// asv overtaking obstacle, new!
			OTG_0(k) = ( v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm() //)
				&& v_o.norm() > 0.05 ) //|| v_o.norm() < 0.05 )  // static obstacle: should be handled by VO or reachability constraint
				//&& v_s.dot(los_0) > cos(PHI_AH_*DEG2RAD)*v_s.norm() // new check, to reduce scope!
				&& v_s.norm() > v_o.norm()
				&& AH_0(k)
				&& dist_0(k) < D_CLOSE_;
			std::cout << "OTG_0 : " << OTG_0(k) << std::endl; // check test!

			// asv being overtaken by obstacle
			OT_0(k) = v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm()
				&& v_s.norm() < v_o.norm()
				//&& v_o.dot(-los_0) > cos(PHI_AH_*DEG2RAD)*v_o.norm() // new check, to reduce scope!
				&& !AH_0(k)
				&& dist_0(k) < D_CLOSE_;
			std::cout << "OT_0 : " << OT_0(k) << std::endl; // check test!

			// New! Obstacle out of way?
			OBS_OUT_OF_WAY_0(k) = ( (v_o.dot(-los_0) < cos(112.5*DEG2RAD)*v_o.norm() // 112.5, obstacle's perspective
					&& !OTG_0(k))
					|| (v_s.dot(los_0) < cos(112.5*DEG2RAD)*v_s.norm() // 95.5, 112.5, ASV's perspective 
					&& !OT_0(k)) ) 
					&& dist_0(k) > D_SAFE_; // 112.5/135.0? check, using obstacle perspective
			std::cout << "OBS_OUT_OF_WAY : " << OBS_OUT_OF_WAY_0(k) << std::endl; // check test!

			// Crossing in progress, new!
			v3_x = v3_o.cross(v3_s);
			std::cout << "v_o x v_s : " << v3_x(2) << std::endl; // check test!
			
			//std::cout << "angle b/n v_o and v_s : " << acos( (v_s.dot(v_o)) / (v_s.norm()*v_o.norm()) ) * RAD2DEG << std::endl; // check test!
			//std::cout << "angle b/n v_o and -los : " << acos( (v_o.dot(-los_0)) / (v_o.norm()) ) * RAD2DEG << std::endl; // check test!
			//std::cout << "angle b/n v_s and los : " << acos( (v_s.dot(los_0)) / (v_s.norm()) ) * RAD2DEG << std::endl; // check test!
			
			CRG_0(k) = v_s.dot(v_o) < cos(22.5*DEG2RAD)*v_s.norm()*v_o.norm() 
				&& !(v_s.dot(los_0) < cos(95.5*DEG2RAD)*v_s.norm() && !SB_0(k) && dist_0(k) > D_CRIT_) // new check, 68.5 -- 95.5 -- 112.5 to reduce scope!
				&& ( (SB_0(k) && v3_x(2) < 0) || (!SB_0(k) && v3_x(2) > 0) ) // change >, < according to ENU or NED!!! 
				&& AH_0(k);
				//&& !OTG_0(k) && !OT_0(k);
				//&& dist_0(k) < D_CLOSE_;
			std::cout << "CRG_0 : " << CRG_0(k) << std::endl; // check test!
			
			
			// Head-on scenario in progress, new!			
			HO_0(k) = v_o.norm() > 0.05
				&& v_s.dot(v_o) < -cos(PHI_HO_*DEG2RAD)*v_s.norm()*v_o.norm()
				&& v_s.dot(los_0) > cos(PHI_AH_*DEG2RAD)*v_s.norm() // new check, to reduce scope!
				&& AH_0(k); 
			std::cout << "HO_0 : " << HO_0(k) << std::endl; // check test!
		}		
		std::cout << "==========" << std::endl; // check test!


		//** End of ** standard initialization and check of current state or COLREGS behavior		
		//===================================================================================


		// build data structure for managing obstacles/neighbors
		kdTree_->build();

		// Compute neighbors from obstacle list and build KD tree 
		computeNeighbors();

		// Compute dynamic weights for sharing responsibility
		computeDynamicWeights();
			 
		// Compute strategic reference velocity 
		computeStrategicVelocity(asv_state, u_d, psi_d, static_obst); // needs dynamic alpha values

		u_str = sqrt((newVelocity_.getX())*(newVelocity_.getX()) + (newVelocity_.getY())*(newVelocity_.getY()));
		psi_str = atan2(newVelocity_.getY(), newVelocity_.getX());

		// apply safe-DRVO solution (may use offset instead) 
		u_best = u_str; 
		psi_best = psi_str;

		// save current best!
		prevVelocity_ = newVelocity_;
		
		
		// data logging
		//=============
		
		double u_os_print = 0;
		if (u_d > 0) u_os_print = u_best/u_d; 
		
		// update speed multiplier and course offset
		u_os_best = u_os_print; psi_os_best = normalize_angle(psi_best - psi_d);
		

		std::cout << "u_str : " << u_str << "  "<< "psi_str : " << psi_str*RAD2DEG << std::endl; // check test!
		//std::cout << "u_os : " << u_best - P_last_ << "  "<< "psi_os : " << (psi_best - Chi_last_)*RAD2DEG << std::endl; // check test!
		std::cout << "u_os : " << u_os_print << "  "<< "psi_os : " << normalize_angle(psi_best - psi_d)*RAD2DEG << std::endl; // check test!


		/*
		
		// write to log files
		log_asv << sample <<","<< asv_state(0) << "," << asv_state(1) << "," << sqrt(asv_state(3)*asv_state(3)+asv_state(4)*asv_state(4)) << "," << asv_state(2)*RAD2DEG << "," << u_d << "," << psi_d*RAD2DEG << "," << u_os_print << "," << normalize_angle(psi_best - psi_d)*RAD2DEG << std::endl;
				
		// logged values are results from filter, not measurements!!!!		
		if (n_obst>0) log_obst_1 <<  sample <<","<< obstacles_[0]->x_[0] <<"," <<obstacles_[0]->y_[0] <<","<< obstacles_[0]->speed_ << "," << obstacles_[0]->psi_in*RAD2DEG << "," << alpha_0(0) << std::endl;
		if (n_obst>1) log_obst_2 <<  sample <<","<< obstacles_[1]->x_[0] <<"," <<obstacles_[1]->y_[0] <<","<< obstacles_[1]->speed_ << "," << obstacles_[1]->psi_in*RAD2DEG << "," << alpha_0(1) << std::endl;
		if (n_obst>2) log_obst_3 <<  sample <<","<< obstacles_[2]->x_[0] <<"," <<obstacles_[2]->y_[0] <<","<< obstacles_[2]->speed_ << "," << obstacles_[2]->psi_in*RAD2DEG << "," << alpha_0(2) << std::endl;
		sample++;
		
		*/	

		P_last_ = u_best;
		Chi_last_ = psi_best;

		P_ca_last_ = u_os_print;//u_os_best; // for MPC as before!
		Chi_ca_last_ = normalize_angle(psi_best - psi_d);
		Chi_ca_last_0 = psi_os_best;

		
		// save current obstacle list
		oldObstacles_.resize(obstacles_.size()); 
		oldObstacles_ = obstacles_; 
			
		//std::cout << "copy successful, number of obtacles are " << obstacles_.size() << std::endl; // check test!	
		
		
		obstacles_.clear();
		
		//std::cout << "clear successful, number of obtacles are " << obstacles_.size() << std::endl; // check test!
			
		
	}



	void Agent::computeDynamicWeights()
	{
		int k = 0;
		alpha_0.resize(n_obst); 
		
		//std::cout << "number of neighbors/obstacles : " << neighbors_.size() << std::endl; // check test!
		//std::cout << "alpha_l.rows/old obstacles : " << alpha_l.rows() << std::endl; // check test!

		for (std::set< std::pair<float, std::size_t> >::const_iterator iter = neighbors_.begin(); iter != neighbors_.end(); ++iter) { //loop through neighbors
		//for (int k = 0; k < n_obst; k++){
			k= iter->second;
			const obstacle *const other = obstacles_[k]; // iter->second is the second member of the pair. i.e. size_t or unsigned int! 
		
			
			// retrieve obstacle's last share of responsibility			
			if (alpha_l.rows() > 0 ){
				for (int l = 0; l < alpha_l.rows(); l++){
					if ( (double)alpha_l(l,0) == (double)other->id_ ){
						if ( alpha_l(l,1)<0.0 || alpha_l(l,1) > 1.0 ) alpha_l(l,1) = 0.0; // init!
						alpha_0(k) = alpha_l(l,1);
					}
				}
				
				std::cout << "alpha_l : " << alpha_0(k) << "  obst_id : " << other->id_ << std::endl; // check test!
			}
			
			// reset for this obstacle
			if ( abs(Chi_ca_last_*RAD2DEG) < 1.0 && P_ca_last_ == 1.0){
				alpha_0(k) = 0.0;
				//std::cout << "Chi_ca_last_ : " << Chi_ca_last_*RAD2DEG << ", 	P_ca_last_: " << P_ca_last_ << std::endl; // check test!
			}
						
			
			// compute ASV's share of responsibility, test for each case ****** NB! for the case where all start from the same alpha, we can simply e.g. set alpha=0.3
			if ( OBS_OUT_OF_WAY_0(k) ){
				alpha_0(k) = 0.0;			
			}else if ( HO_0(k) ){ // may change into CRG P! use smooth transition b/n HO and CRG!
				if (dist_0(k) > D_CLOSE_ && alpha_0(k) < 0.2) alpha_0(k) = 0.2; // indicate and encourage
				if (dist_0(k) <= D_CLOSE_ && dist_0(k) > D_REACT_ && alpha_0(k) < 0.3) alpha_0(k) = 0.3; // adapt to required and hold (0.3 -> 0.5) 
				if (dist_0(k) <= D_REACT_  && dist_0(k) > D_CRIT_ && alpha_0(k) < 0.3) alpha_0(k) = 0.3; // adapt strictly and react (0.3 -> 1.0) 
				if (dist_0(k) <= D_CRIT_ && alpha_0(k) < 0.3) alpha_0(k) = 0.3; // adapt strictly, react and evade (0.3 -> 1.0)
			}else if ( OTG_0(k) || ( (CRG_0(k) && SB_0(k)) ) ){ 
				//if (dist_0(k) > D_CLOSE_ && alpha_0(k) < 0.3) alpha_0(k) = 0.3; // indicate and encourage; adapt (if needed) : 0.3 -> 1.0
				if (dist_0(k) <= D_CLOSE_ && alpha_0(k) < 0.3) alpha_0(k) = 0.3;// adapt strictly to required and hold; react and evade (if needed)
			}else if ( OT_0(k) || ( (CRG_0(k) && !SB_0(k)) ) ){ 
				//if (dist_0(k) > D_CLOSE_ && alpha_0(k) < 0.1) alpha_0(k) = 0.1; // indicate and encourage
				//if (dist_0(k) <= D_CLOSE_ && dist_0(k) > D_REACT_ && alpha_0(k) < 0.1) alpha_0(k) = 0.1; // require and hold
				if (dist_0(k) <= D_REACT_ && dist_0(k) > D_CRIT_ && alpha_0(k) < 0.2) alpha_0(k) = 0.2; // adapt strictly and react (0.1 -> 1.0)
				if (dist_0(k) <= D_CRIT_ && alpha_0(k) < 0.2) alpha_0(k) = 0.2; // adapt strictly, react and evade
			}else{  // no COLREGS scenario is active 
				if (dist_0(k) <= D_CLOSE_ && alpha_0(k) < 0.2)
				alpha_0(k) = 0.2; // allows for adaptation towards RVO (have included "dist_0(k) <= D_CLOSE_ &&" after IROS)
				// should this be zero???
			}

			//std::cout << "alpha_0 : " << alpha_0(k) << std::endl; // check test!
		}		
			
	}

	void Agent::computeNeighbors()
	{
		neighbors_.clear();
		kdTree_->query(this, neighborDist_ * neighborDist_);
	}

	void Agent::computeStrategicVelocity(const Eigen::Matrix<double,6,1>& asv_state, double u_d, double psi_d, const Eigen::Matrix<double,1,4>& static_obst) // needs dynamic alpha values
	{	
		int test_colav_method = 0; // 0=DRVO, 1=RVO, 2=VO
		
		double u_ca, psi_ca;
		Eigen::Vector2d dist, los0, losNd;
		double dist0, distNd;
		Vector2 p0_CPA_asv, p0_CPA_obst, pNd_CPA_asv, pNd_CPA_obst;
		bool safe_candidate = true, obst_right_passage, obst_right_passage_strict, obst_left_passage_strict;
		alpha_l.resize(n_obst,2); 
		psi_d = normalize_angle(psi_d);
		
		int Nd = asv->getNsamp();
		//std::cout << "getNsamp:   " << Nd << std::endl; // check test! 

		/*
		// Geographical restriction related variables
		double gCost, n_geo_samp;               // cost related to geographical constraints
		n_geo_samp = T_stat_/DT_;
		Eigen::Vector2d p0, p1, v0, v1;
		p0 << asv->x[0], asv->y[0];             // start of trajectory
		p1 << asv->x[n_geo_samp],asv->y[n_geo_samp];    // end of trajectory
		int conv = -1; // NED: -1, ENU: 1 , OBS! not tested
		v0 << static_obst[0], static_obst[1];   // start of no-go line
		v1 << static_obst[2], static_obst[3];   // end of no-go line
		// When obstacles are implemented as polygons, isBehind condition can be removed
		double d_geo = distPoint2line(p1, v0, v1);
		bool isGeoConstr = (doIntersect(p0, p1, v0, v1) || isBehind(p1, v0, v1, d_geo));
		*/


		velocityObstacles_.clear(); // velocityObstacles_ are in a std::vector<VelocityObstacle>
		velocityObstacles_.reserve(neighbors_.size()); // neighbors_ are in a std::set< std::pair<float, std::size_t> >

		VelocityObstacle velocityObstacle;

		for (std::set< std::pair<float, std::size_t> >::const_iterator iter = neighbors_.begin(); iter != neighbors_.end(); ++iter) { //loop through neighbors
			
			const obstacle *const other = obstacles_[iter->second]; // iter->second is the second member of the pair. i.e. size_t or unsigned int! 
			bool SB=SB_0(iter->second), AH=AH_0(iter->second), CRG=CRG_0(iter->second), OTG=OTG_0(iter->second), OT=OT_0(iter->second), HO=HO_0(iter->second), OBS_OUT_OF_WAY=OBS_OUT_OF_WAY_0(iter->second); // current colav behavior
			float alpha = (float)alpha_0(iter->second), rho = 0.9; // 0.8 0.9

			const float angle = atan(other->position_ - position_);	// theta: angle of line AB to the horizontal (x) axis
			float openingAngle = 1.57079633; //asin(1)... ASV is colliding with or grazing this obstacle! 
			if (absSq(other->position_ - position_) > sqrd(other->radius_ + radius_)){
				openingAngle = std::asin((other->radius_ + radius_) / abs(other->position_ - position_)); // theta_0: half of VO apex-angle (between left and right edges)
			}else if(!AH) {
				openingAngle = std::asin((other->l - other->w/2.0  + radius_) / abs(other->position_ - position_)); // other->l - other->w/2.0 + 10.0
			}

			//std::cout << "angle:   " << angle << std::endl; // check test!
			//std::cout << "openingAngle:   " << openingAngle*RAD2DEG << std::endl; // check test!

			velocityObstacle.side1_ = Vector2(std::cos(angle - openingAngle), std::sin(angle - openingAngle)); // unit vector of the right (ENU*) side of VO

			velocityObstacle.side2_ = Vector2(std::cos(angle + openingAngle), std::sin(angle + openingAngle)); // unit vector of the left (ENU*) side of VO
			//std::cout << "vo_s2:   " << velocityObstacle.side2_ << std::endl; // check test!

			// option 2: reflect side1 to get side2: s2 = -s1 − 2 * (-s1 * n_AB)n_AB ***works!!
			//velocityObstacle.side2_ = -velocityObstacle.side1_ - 2.0f*(-velocityObstacle.side1_*normalize(other->position_ - position_))*normalize(other->position_ - position_); 

			
			float d = 2.0f * std::sin(openingAngle) * std::cos(openingAngle); //const float d 
			//NB! sin(2*a)=2*sin(a)*cos(a), i.e. sin(apex-angle) ... no problem when dividing by 'd' since sin(a) is never zero! VO apex angle is within (0,pi)

			// if the relative velocity is on the left (ENU*) of the VO center line  
			obst_right_passage = det(other->position_ - position_, velocity_ - other->velocity_) > 0.0f; // (P_B-P_A) x (v_A-v_B) > 0 
			obst_right_passage_strict = det(velocityObstacle.side2_, velocity_ - other->velocity_) >= 0.0f; // (lambda_r) x (v_A-v_B) > 0 (ENU)
			obst_left_passage_strict = det(velocityObstacle.side1_, velocity_ - other->velocity_) <= 0.0f; // (lambda_l) x (v_A-v_B) > 0 (ENU)
			
			//std::cout << "obst_right_passage:    " << obst_right_passage << std::endl; // check test!
			//std::cout << "obst_right_passage_strict:    " << obst_right_passage_strict << std::endl; // check test!
			//std::cout << "obst_left_passage_strict:    " << obst_left_passage_strict << std::endl; // check test!
			
			
			if ( obst_right_passage && (OTG || (OT && dist_0(iter->second) < D_CRIT_)) ) { // (P_B-P_A) x (v_A-v_B) > 0 ... replaced prefVelocity with measured velocity? NO!
			//if (obst_right_passage && !CRG && !HO && (OTG || OT || !AH ) ) { // 

				// is obstacle cooperating? 				
				// adapt if obstacle is not cooperating? 
				if ( (OT && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_  && dist_0(iter->second) > D_CRIT_)  
				     ){ 
				    
					alpha = 1.0f - rho * (1.0f - (float)alpha_0(iter->second)); // react and adapt :alpha_l -> 1.0 	
					std::cout << "Obstacle "<< other->id_ << " not cooperating " << std::endl; // check test!					
				}else
				
				if ( (OTG && !SB && dist_0(iter->second) > D_CLOSE_) || // moved from above!
				     ((!obst_left_passage_strict && !obst_right_passage_strict) && dist_0(iter->second) <= D_REACT_)  ||
				     ( !OT && !OTG && !CRG && !HO && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_) ||
				     ( !OT && !OTG && !CRG && !HO && SB && !obst_right_passage_strict && dist_0(iter->second) <= D_REACT_) 
				    ){
					alpha = 0.5f - rho * (0.5f - (float)alpha_0(iter->second)); // react and adapt :0.5 -> 1.0 (7.0)	
				        if (alpha_0(iter->second) > 0.5) alpha = 0.5; // limit
				        
					std::cout << "Obstacle "<< other->id_ << " not cooperating " << std::endl; // check test!					
				}else
				{
					alpha=alpha_0(iter->second);
				}
				
				
				// test override!!!!
				if (test_colav_method == 1) alpha = 0.5f; // RVO
				if (test_colav_method == 2) alpha = 1.0f; // VO	
								
				std::cout << "Obstacle "<< other->id_  << ", alpha : " << alpha << std::endl; // check test!
				
				const float s = (1.0f - alpha) * det(velocity_ - other->velocity_, velocityObstacle.side2_) / d; // 0.5f (left: ENU*)

				// compute VO apex
				velocityObstacle.apex_ = other->velocity_ + s * velocityObstacle.side1_; // (right: ENU*) *** working OK!!

				// pass this obstacle on the left side
				velocityObstacle.side1_ = normal(position_, other->position_);
				//velocityObstacle.side1_ = -velocityObstacle.side2_; // test** s1=R, s2=L: turn L => keep L change R 

				std::cout << "Check point 1 " << std::endl; // check test!
			}
			else { // the relative velocity is on the right (ENU*) of the VO center line 
			
				// adapt if obstacle is not cooperating? 
				if ( (HO && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_  && dist_0(iter->second) > D_CRIT_)  ||
				     (CRG && SB && !obst_left_passage_strict && dist_0(iter->second) <= D_CLOSE_) ||
				     (CRG && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_  && dist_0(iter->second) > D_CRIT_) ||
				     (OT && SB && !obst_right_passage_strict && dist_0(iter->second) <= D_REACT_  && dist_0(iter->second) > D_CRIT_) ||
				     (OT && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_  && dist_0(iter->second) > D_CRIT_) 
				     ){
				      
					alpha = 1.0f - rho * (1.0f - (float)alpha_0(iter->second)); // react and adapt :0.5 -> 1.0 (7.0)	
					std::cout << "Obstacle "<< other->id_ << " not cooperating " << std::endl; // check test!					
				}else					
				
				if (
				     (OTG && SB && !obst_right_passage && dist_0(iter->second) > D_CLOSE_) || // moved from above!
				     (OTG && !SB && obst_right_passage && dist_0(iter->second) > D_CLOSE_) || // moved from above!
				     (CRG && SB && obst_right_passage && dist_0(iter->second) > D_CLOSE_) ||  // moved from above!					     
				     (HO && !SB && obst_right_passage && dist_0(iter->second) <= D_CLOSE_  && dist_0(iter->second) > D_REACT_) || 
				    //(CRG && !SB && obst_right_passage && dist_0(iter->second) <= D_CLOSE_  && dist_0(iter->second) > D_REACT_) ||
				     ((!obst_left_passage_strict && !obst_right_passage_strict) && dist_0(iter->second) <= D_REACT_)  ||
				     ( !OT && !OTG && !CRG && !HO && !SB && !obst_left_passage_strict && dist_0(iter->second) <= D_REACT_) ||
				     ( !OT && !OTG && !CRG && !HO && SB && !obst_right_passage_strict && dist_0(iter->second) <= D_REACT_) 
				    ){
				      
					alpha = 0.5f - rho * (0.5f - (float)alpha_0(iter->second)); // react and adapt :0.5 -> 1.0 (7.0)	
					if (alpha_0(iter->second) > 0.5f) alpha = 0.5f; // limit
					
					std::cout << "Obstacle "<< other->id_ << " not cooperating " << std::endl; // check test!					
				}
				

				// test override!!!!
				if (test_colav_method == 1) alpha = 0.5f; // RVO
				if (test_colav_method == 2) alpha = 1.0f; // VO
				
				// Obstacle out of way means ignore behavior?
				if (OBS_OUT_OF_WAY) alpha = 0.0f;
				
				
				std::cout << "Obstacle "<< other->id_  << ", alpha : " << alpha << std::endl; // check test!					
				
				const float s = (1.0f - alpha)  * det(velocity_ - other->velocity_, velocityObstacle.side1_) / d; // 0.5f * (right: ENU*)

				// compute VO apex
				velocityObstacle.apex_ = other->velocity_ - s * velocityObstacle.side2_; // (left: ENU) *** working OK, if (-s) is used, else apex remains almost same for all alpha!
				
				// pass this obstacle on the right side
				velocityObstacle.side2_ = -normal(position_, other->position_);
				//velocityObstacle.side2_ = -velocityObstacle.side1_; // test** s1=R, s2=L: turn R => keep R (s1) change L (s2) 
				
				std::cout << "Check point 2 " << std::endl; // check test!
			}


			//std::cout << "vo_s1:   " << velocityObstacle.side1_ << std::endl; // check test!
			//std::cout << "vo_s2:   " << velocityObstacle.side2_ << std::endl; // check test!
			//std::cout << "   " << std::endl; // check test!

			//std::cout << "vo_apex:   " << velocityObstacle.apex_ << std::endl; // check test!
			//std::cout << "   " << std::endl; // check test!

			
			if (!OBS_OUT_OF_WAY || (alpha>0 && !OBS_OUT_OF_WAY) )	
				velocityObstacles_.push_back(velocityObstacle);
						
			// store last share of responsibility
			alpha_l(iter->second, 0) = other->id_; alpha_l(iter->second, 1) = alpha; 
			
			// update alpha_0 for logging and further checks
			alpha_0(iter->second) = alpha;

		}
		
		

		// candidate points in the velocity space, for computing/locating new velocity for this agent ...
		candidates_.clear();

		Candidate candidate;
				
		
		// Velocities in the set Omega_grid
		// Insert grid velocities as candidates if they are feasible, reachable and consistent with the drvo strategy	
		
		float maxAccel = maxAccel_, maxSpeed=maxSpeed_, maxDchi=MAXDCHI_, Chi_d=psi_d;
		int rate_relax_count=1, l=0;
		while(candidates_.empty()){			
			l=0;
			for (int i = 0; i < Chi_ca_0.size(); i++){
				for (int j = 0; j < P_ca_.size(); j++){

					gridVelocity_.setX(u_d*P_ca_[j] * cos( normalize_angle(Chi_d + Chi_ca_0[i]) )); 
					gridVelocity_.setY(u_d*P_ca_[j] * sin( normalize_angle(Chi_d + Chi_ca_0[i]) ));
		
					candidate.velocityObstacle1_ = std::numeric_limits<int>::max() - l;
					candidate.velocityObstacle2_ = std::numeric_limits<int>::max() - l;

					// check for speed limit
					if (absSq(gridVelocity_) < maxSpeed * maxSpeed) {
						candidate.position_ = gridVelocity_;
					}
					else {
						candidate.position_ = maxSpeed * normalize(gridVelocity_);
					}				
				
			
					if ( true && ( (absSq(candidate.position_ - velocity_) < sqrd(maxAccel*T_HD_) && normalize(candidate.position_)*normalize(velocity_) > cos(MAXDCHI_*DEG2RAD)) ) ) {
								
						candidates_.insert(std::make_pair(absSq(prefVelocity_ - candidate.position_), candidate));
				
						//std::cout << "Chi_ca_0: " << Chi_ca_0[i]*RAD2DEG << ",   Chi_d+Chi_ca_0: " << normalize_angle(psi_d + Chi_ca_0[i])*RAD2DEG << std::endl;
					
					}		
						
				
					l++;
				
				}
			} 
			
			if (candidates_.empty()){  
				if (rate_relax_count >= 3) break; // pre-defined behavior enforced, break while loop!!		
								  
				if (rate_relax_count >= 2){ 
					maxSpeed = 0.5*maxSpeed_; // enforce a pre-defined/fixed speed reduction
					Chi_d = orientation_; 	  // grid around current velocity, instead of desired velocity, alarm! 					
					
					std::cout << "Reference velocity is infeasible! Using psi_d=psi_ and 0.5*u_d!" << std::endl;
					
				}				
				
				maxAccel = 2*maxAccel;	// relax acceleration limit			
				maxDchi = 2*maxDchi;	// relax rotation rate limit			
				
				rate_relax_count++;
			}
			
		}
		std::cout << "l-counter: " << l << ",  rate_relax_counter: " << rate_relax_count << std::endl;
		
		


		// identify optimal point among candidate points!
		//===============================================
		
/*		int optimal = -1;
		//bool feasible = false;


		// Option 1: min || v - v^pref ||
		for (std::multimap<float, Candidate>::const_iterator iter = candidates_.begin(); iter != candidates_.end(); ++iter) {
			candidate = iter->second;
			bool valid = true;

			for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j) {
				if (j != candidate.velocityObstacle1_ && j != candidate.velocityObstacle2_ && det(velocityObstacles_[j].side2_, candidate.position_ - velocityObstacles_[j].apex_) < 0.0f && det(velocityObstacles_[j].side1_, candidate.position_ - velocityObstacles_[j].apex_) > 0.0f) {
					valid = false;

					if (j > optimal) {
						optimal = j;
						newVelocity_ = candidate.position_; // newVelocity_
						std::cout << "best 'colliding' solution " << std::endl; // check test!
					}

					break;
				}
			}

			if (valid) {
				newVelocity_ = candidate.position_; // newVelocity_
				//std::cout << "'collision-free' solution " << std::endl; // check test!
				//feasible = true; // NB! the best colliding solution is also feasible since all candidates are checked for feasibility.
				break;
			}
		}
		

 */

		// Option 2: prioritize course and speed differently min q_u*(u - u^pref)^2 + q_chi*(chi - chi^pref)^2 + ... 
		float cost = std::numeric_limits<float>::infinity(), cost_i; 
		double cost_t=0, cost_smax=0, cost_s=0, cost_s1=0, cost_s2=0;
		
		for (std::multimap<float, Candidate>::const_iterator iter = candidates_.begin(); iter != candidates_.end(); ++iter) {
			candidate = iter->second;
			//std::cout << "canditate_vel " << candidate.position_ << std::endl; // check test!						
			bool valid = true;
			cost_t=0; cost_s1=0; cost_s2=0; cost_s=0; cost_smax=0;

			// Verify feasibility w.r.t. the COLREGs-constrained DRVO set.
			for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j) {
				if (j != candidate.velocityObstacle1_ && j != candidate.velocityObstacle2_ && det(velocityObstacles_[j].side2_, candidate.position_ - velocityObstacles_[j].apex_) < 0.0f && det(velocityObstacles_[j].side1_, candidate.position_ - velocityObstacles_[j].apex_) > 0.0f) {
					valid = false;					
					
					break;
				}
			}
			
						
			// compute candidate speed and course
			u_ca = sqrt((candidate.position_.getX())*(candidate.position_.getX()) + (candidate.position_.getY())*(candidate.position_.getY()));
			psi_ca = atan2(candidate.position_.getY(), candidate.position_.getX());
			//std::cout << "Psi_ca: "<< normalize_angle(psi_ca-psi_d)*RAD2DEG <<" =? "<< (normalize_angle_360(psi_ca)-normalize_angle_360(psi_d))*RAD2DEG << ", 	u_ca: " << u_ca << std::endl;
			
			// predict asv position, one-step-ahead
			asv->eulersMethod(asv_state, u_ca, psi_ca); //linearPrediction
			// check position for collision danger 
			for (std::set< std::pair<float, std::size_t> >::const_iterator iter = neighbors_.begin(); iter != neighbors_.end(); ++iter) {
				const obstacle *const other = obstacles_[iter->second];	
				bool SB=SB_0(iter->second), AH=AH_0(iter->second), CRG=CRG_0(iter->second), OTG=OTG_0(iter->second), OT=OT_0(iter->second), HO=HO_0(iter->second), OBS_OUT_OF_WAY=OBS_OUT_OF_WAY_0(iter->second); // current colav behavior
				dist0 = sqrt(absSq(other->position_ - position_)); 
						
				// compute transitional cost for aborting COLREGs 
				//===============================================									
				
				// extract predicted positions at time t_Hd
				cand_predicted_pos_.setX(asv->x[Nd-1]); // 
				cand_predicted_pos_.setY(asv->y[Nd-1]); //
				obst_predicted_pos_.setX(other->x_[Nd-1]); // 
				obst_predicted_pos_.setY(other->y_[Nd-1]); //
				//std::cout << "cand pos at t_Hd: " << cand_predicted_pos_ << ",  obst pos at t_Hd: " << obst_predicted_pos_ << std::endl;
				
				
				// compute t_CPA_t0, d_CPA_t0; t_CPA_tHd, d_CPA_tHd
				float t_CPA_0 = 0.0,  d_CPA_0 = 0.0,  t_CPA_Nd = 0.0,  d_CPA_Nd = 0.0;
				float d = absSq(velocity_ - other->velocity_);
				if (d>0.0){
					t_CPA_0 = ( (other->position_ - position_) * (velocity_ - other->velocity_) ) / d; // or use previous commanded velocity???
				}
				
				d = absSq(candidate.position_ - other->velocity_);
				if (d>0.0){						
					t_CPA_Nd = ( (obst_predicted_pos_ - cand_predicted_pos_) * (candidate.position_ - other->velocity_) ) / d;
				}					
				
				// positions at current CPA, i.e. at t=0
				p0_CPA_asv.setX( position_.getX() + t_CPA_0*velocity_.getX() );
				p0_CPA_asv.setY( position_.getY() + t_CPA_0*velocity_.getY() );
				
				p0_CPA_obst.setX( other->position_.getX() + t_CPA_0*other->velocity_.getX());
				p0_CPA_obst.setY( other->position_.getY() + t_CPA_0*other->velocity_.getY());
			
			
				// positions at CPA computed at t=t_Hd using candidate velocity for asv, obstacle velocity is constant
				pNd_CPA_asv.setX( asv->x[Nd-1] + t_CPA_Nd*candidate.position_.getX() );
				pNd_CPA_asv.setY( asv->y[Nd-1] + t_CPA_Nd*candidate.position_.getY() );
				
				pNd_CPA_obst.setX( other->x_[Nd-1] + t_CPA_Nd*other->velocity_.getX() );
				pNd_CPA_obst.setY( other->y_[Nd-1] + t_CPA_Nd*other->velocity_.getY() );
				
				
				// compute line of sight and passage side at current CPA
				dist(0) = p0_CPA_obst.getX() - p0_CPA_asv.getX(); 
				dist(1) = p0_CPA_obst.getY() - p0_CPA_asv.getY(); 
				d_CPA_0 = dist.norm();
				los0 = dist/d_CPA_0;
				//std::cout << "d_CPA_0 : " << d_CPA_0 << std::endl; // check test!
				
				// compute angle between los and asv course angle
				double phi0 = atan2(dist(1),dist(0)) - normalize_angle(asv->psi[0]); 
				phi0 = normalize_angle(phi0);	
				
				// obstacle at starboard/port side at current CPA
				double SB_t0 = phi0 < 0;
				//std::cout << "SB0 : " << SB0 << std::endl; // check test!
				
				
				// compute line of sight and passage side at CPA computed at t_Hd
				dist(0) = pNd_CPA_obst.getX() - pNd_CPA_asv.getX(); 
				dist(1) = pNd_CPA_obst.getY() - pNd_CPA_asv.getY(); 
				d_CPA_Nd = dist.norm();
				losNd = dist/d_CPA_Nd;
				//std::cout << "dist0 : " << dist0 << std::endl; // check test!
				
				// compute angle between los and asv course angle
				double phiNd = atan2(dist(1),dist(0)) - normalize_angle(asv->psi[0]); // assumed same as asv->psi[Nd]!!!
				phiNd = normalize_angle(phiNd);	
				
				// obstacle at starboard/port side at current CPA
				double SB_tNd = phiNd < 0;
				//std::cout << "SB_tNd : " << SB_tNd << std::endl; // check test!
				
				// Penalize this candidate velocity if SB_t0 != SB_tNd, or rather using commanded/expected side
				if ( ((HO && !SB) || (CRG ) || dist_0(iter->second) < D_CRIT_) && AH && SB_tNd && (!OBS_OUT_OF_WAY || (alpha_0(iter->second)>0 && !OBS_OUT_OF_WAY)) ){ // include COLREGS scenarios! and d_CPA_0 > d_CPA_Nd?
					cost_t = cost_t + 1;	// sum costs for all colav transitions, considering all obstacles					
					//std::cout << "dist_CPA_0: " << dist0 << ", " << "dist_CPA_Nd: "<< d_CPA_Nd <<", "<< "Psi_ca: "<< (psi_ca-psi_d)*RAD2DEG << std::endl;
				}
				
				
				// (Relaxation) penalty for DRVO-constraint voilation or inconsistency
				//=====================================================================
				
				// check for consistency of DRVO strategy 
				/* (NB! we can omit this since it may be restrictive, and it requires a large deviation in position for the predicted side to change)
				if(valid){				
					d = absSq(candidate.position_ - other->velocity_);
					if (d>0.0){
						t_CPA_0 = ( (other->position_ - position_) * (candidate.position_ - other->velocity_) ) / d;
												
						d_CPA_0 = sqrt( absSq( ( position_ + t_CPA_0*candidate.position_) - (other->position_ + t_CPA_0*other->velocity_) ) );
					
						t_CPA_Nd = ( (obst_predicted_pos_ - cand_predicted_pos_) * (candidate.position_ - other->velocity_) ) / d;
											
						d_CPA_Nd = sqrt( absSq( ( cand_predicted_pos_ + t_CPA_Nd*candidate.position_) - (obst_predicted_pos_ + t_CPA_Nd*other->velocity_) ) );
					}				
				
					if ( (d_CPA_Nd + 20.0 < d_CPA_0 && t_CPA_Nd + T_HD_- DT_ > 0)
					     && det(other->position_ - position_, candidate.position_ - other->velocity_) * 
					     det(obst_predicted_pos_ - cand_predicted_pos_, candidate.position_ - other->velocity_) < 0.0f // both determinants must have the same sign!! 
					){
						//std::cout << "candidate velocity (" << u_ca << ", " << psi_ca*RAD2DEG << ") not consistent"<< ", " << sqrt(d_CPA_Nd_sq) << " < " << sqrt(d_CPA_0_sq) << std::endl;
						valid = false; // not used, uncomment to activate penalty for inconsistency!
					}
				}
				*/	
					
				
				if(!valid && ((HO && !SB) || (CRG ) || dist_0(iter->second) < D_CRIT_) && (!OBS_OUT_OF_WAY || (alpha_0(iter->second)>0 && !OBS_OUT_OF_WAY)) ){ // should OBS_OUT_OF_WAY be considered here? Yes, since <!valid> applies to only !OBS_OUT_OF_WAY!
				
					// if the relative velocity is on the left (ENU*) of the VO center line  
					obst_right_passage = det(other->position_ - position_, velocity_ - other->velocity_) > 0.0f; // expected side to pass! (P_B-P_A) x (v_A-v_B) > 0 
					
					if (t_CPA_0<1.0) t_CPA_0=1.0; if (d_CPA_0<1.0) d_CPA_0=1.0; // safe numerics!
					//dist0 = sqrt(absSq(other->position_ - position_)); 
					
					// if v_cand becomes invalid when its d_CPA >= d_min it implies v_cand leads to a wrong passage side, and therefore blocked by COLREGs 
					// or it is blocked by the drvo constraint to ensure a slow return to the original path.   
				
					if (obst_right_passage && (OTG || OT || !AH) && (other->radius_ + radius_ < dist0) && (d_CPA_0 < dist0) ) {
				
						cost_s1 = 1e5 * (1.0/pow(t_CPA_0, P_)) * pow(dist0/d_CPA_0, Q_);
				
					}else if ( (!obst_right_passage || (!OTG && !OT && AH) ) && (d_CPA_0 < other->radius_ + radius_) ) {
					
						//cost_s1 = (1.0/pow(t_CPA_0, P_)) * pow((other->radius_ + radius_)/d_CPA_0, Q_);
						cost_s1 = 1e5 * (1.0/pow(t_CPA_0, P_)) * pow((other->radius_ + radius_)/d_CPA_0, Q_); 
					
					}else{
				
						cost_s1 = 1e10; // High penalty!! 							
					}
					
					//cost_s1 = 1e100; // test High penalty!! 					
				}
				
				
				// Relaxation penalty for velocities that are not reachable
				//=========================================================
				
				
				// Geographical restriction related variables
				
				double gCost=0, n_geo_samp;               // cost related to geographical constraints
				n_geo_samp = T_stat_/DT_;
				Eigen::Vector2d p0, p1, v0, v1;
				p0 << asv->x[0], asv->y[0];             // start of trajectory
				//p1 << asv->x[n_geo_samp],asv->y[n_geo_samp];    // end of trajectory
				p1 << asv->x[n_geo_samp-1],asv->y[n_geo_samp-1];    // end of trajectory
				int conv = -1; // NED: -1, ENU: 1 , OBS! not tested
				v0 << static_obst[0], static_obst[1];   // start of no-go line
				v1 << static_obst[2], static_obst[3];   // end of no-go line
				// When obstacles are implemented as polygons, isBehind condition can be removed
				double d_geo = distPoint2line(p1, v0, v1);
				bool isGeoConstr = (doIntersect(p0, p1, v0, v1) || isBehind(p1, v0, v1, d_geo));
				
				
				
				
				// check position for collision danger 
				double d_CPA=0.0, t=0.0, t0=0.0;
				Eigen::Vector2d v_o, v_s;
				for (int n = 1; n < Nd-1; n+=10){ // using 5s fixed intervals, since actual DT_=0.5 for integration!
			
					t += DT_*10;
					
					dist(0) = other->x_[n] - asv->x[n]; 
					dist(1) = other->y_[n] - asv->y[n];
					d_CPA = dist.norm(); if (d_CPA < 1.0) d_CPA=1.0; // safe numerics!

					v_o(0) = other->u_[n];
					v_o(1) = other->v_[n];
					rot2d(other->psi_,v_o);

					v_s(0) = asv->u[n];
					v_s(1) = asv->v[n];
					rot2d(asv->psi[n],v_s);	
					
					
					// Cost for voilating no-go zone
					
		 			if (isGeoConstr && (t<T_stat_)){
					     p0 << asv->x[n], asv->y[n];
					     d_geo = dist2staticObst(p0, v0, v1);
					     gCost = (1/pow(fabs(t-t0),P_))*pow(D_SAFE_/d_geo,Q_);
					}
						
									

					if (d_CPA < other->radius_ + radius_){	
						double q_coll = 1e5;
						
						if(dist0 >= other->radius_ + radius_){						
							cost_s2 = q_coll * (1.0/pow(fabs(t-t0),P_)) * pow((other->radius_ + radius_)/d_CPA,Q_) * pow((v_s-v_o).norm(),2);
						}else{
							double s_0 = std::abs(u_d - v_o.norm()) + 1.0; // 1.0 ensures ||v_s||<||v_o||
							
							if (HO || CRG || OTG || OT){ 
								cost_s2 = q_coll * (1.0/pow(fabs(t-t0),P_)) * pow((other->radius_ + radius_)/d_CPA,Q_) * (1.0/pow(v_s.norm() - (v_o.norm()+s_0),2));//speed
							}else{
								cost_s2 = q_coll * (1.0/pow(fabs(t-t0),P_)) * pow((other->radius_ + radius_)/d_CPA,Q_); // k_coll or 1?
							}												
						}
													
						//std::cout << "candidate velocity (" << u_ca << ", " << psi_ca*RAD2DEG << ") not reachable" << std::endl;
						//safe_candidate = false; 
						break;	// means only collision time is decisive, not a combination with d_CPA. For a combination find "max cost" in this for-loop!			
					}					
					
				}				
						
				cost_s = cost_s1 + cost_s2 + G_*gCost;
				
				if (cost_s > cost_smax){
					cost_smax = cost_s;  // Maximizing the cost with regards to time, considering all obstacles
				}		
							
			}
			
			/*
			// Cost for voilating no-go zone
 			if (isGeoConstr && (t<T_stat_)){
			     p0 << asv->x[i], asv->y[i];
			     d_geo = dist2staticObst(p0, v0, v1);
			     gCost = (1/pow(fabs(t-t0),P_))*pow(D_SAFE_/d_geo,Q_);
		        }
		        */

		
			// total cost		
			cost_i = K_P_*pow(u_ca-u_d,2) + K_CHI_*pow(normalize_angle(psi_ca-psi_d),2) + K_DP_*pow(u_ca-P_last_,2) + K_DCHI_*pow(normalize_angle(psi_ca-Chi_last_),2) + cost_smax + 1e3*cost_t*cost_t;
								
			// Min cost
			if (cost_i < cost){
				cost = cost_i; 			
				newVelocity_ = candidate.position_;
				//std::cout << "'collision-free' solution, with cost " << cost << std::endl; // check test!
			}									
								
		}
		
		std::cout << "cost " << cost << std::endl; // check test!
		if (cost == std::numeric_limits<float>::infinity()){
			std::cout << "No solution... relax constraints " << std::endl;
			newVelocity_ = prevVelocity_;
		}

	}


	void Agent::insertNeighbor(std::size_t obstNo, float &rangeSq)
	{
		// simulator is env! in next line***** replace with list of obstacles
		const obstacle *const other = obstacles_[obstNo];
		const float distSq = absSq(position_ - other->position_);
					
		if (distSq < rangeSq) {
			if (neighbors_.size() == maxNeighbors_) {
				neighbors_.erase(--neighbors_.end());
			}

			neighbors_.insert(std::make_pair(distSq, obstNo));

			if (neighbors_.size() == maxNeighbors_) {
				rangeSq = (--neighbors_.end())->first;
			}
		}
	}

	// update agent state using a state estimator or sensor measurements
	void Agent::update(double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state)
	{
		Eigen::Vector2d v_2d;

		position_.setX(asv_state(0)); // 0
		position_.setY(asv_state(1)); // 1

		orientation_ = normalize_angle(asv_state(2));

		v_2d(0) = asv_state(3); 
		v_2d(1) = asv_state(4);	  // note that MR may set this state to zero, but OK since we rotate with the orientation in the next few lines!
		
		// reverse speed not allowed, since it can make ASV's own decisions difficult/infeasible, also just due to noise!
		// If reversing during DP, parking, station keeping, etc. rotate the body-coordinates to swap +x with -x axis? 
		if (v_2d(0)<0){
			v_2d(0) = 0.0; 
			v_2d(1) = 0.0;	
		}
		
		rot2d(orientation_,v_2d); // rotate to fixed frame

		velocity_.setX(v_2d(0)); // in fixed frame
		velocity_.setY(v_2d(1)); // in fixed frame

		prefSpeed_ = u_d;
		maxSpeed_ = u_d; // u_d as max makes VO struggle to find collision-free solution in some scenarios! 
		
		psi_d = normalize_angle(psi_d);
		prefVelocity_.setX(u_d*cos(psi_d)); // is this OK? normalization correct?
		prefVelocity_.setY(u_d*sin(psi_d));
		
		if (Chi_last_ == -1 && P_last_ == -1){
			Chi_last_ = psi_d; 
			P_last_ = u_d;
		} 
		
	}


	void Agent::rot2d(double yaw, Eigen::Vector2d &res){
		Eigen::Matrix2d R;
		R << cos(yaw), -sin(yaw),
			 sin(yaw), cos(yaw);
		res = R*res;
	}

	inline double Agent::normalize_angle(double angle)
	{
		while(angle <= -M_PI) angle += 2*M_PI; 
		while (angle > M_PI) angle -= 2*M_PI;
		return angle;
	}
	
	inline double Agent::normalize_angle_360(double angle){
		angle = fmod(angle,2*M_PI);
		if (angle < 0)
		angle += 2*M_PI;
		return angle;
	}
	
	int Agent::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)                            // new 2018
	{
	    int val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
	    
	    if (val == 0) return 0; // colinear
	    return val > 0 ? 1: 2; // clock or counterclock wize
	}

	bool Agent::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)                             // new 2018
	{
	    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
		q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
		return true;
	    return false;
	}

	bool Agent::doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2)    // new 2018
	{
	    // Find the four orientations needed for general and
	    // special cases
	    int o1 = orientation(p1, q1, p2);
	    int o2 = orientation(p1, q1, q2);
	    int o3 = orientation(p2, q2, p1);
	    int o4 = orientation(p2, q2, q1);
	    
	    // General case
	    if (o1 != o2 && o3 != o4)
		return true;
	    
	    // Special Cases
	    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
	    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
	    
	    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
	    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
	    
	    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
	    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
	    
	    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
	    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
	    
	    return false; // Doesn't fall in any of the above cases
	}

	double Agent::distPoint2line(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d q2)                   // new 2018
	{   Eigen::Vector3d a;
	    Eigen::Vector3d b;
	    a << (q1 - q2), 0;
	    b << (p1 - q2), 0;
	    
	    Eigen::Vector3d c = a.cross(b);
	    double d = c.norm()/a.norm();
	    return d;
	}

	bool Agent::isBehind(Eigen::Vector2d p1, Eigen::Vector2d v1, Eigen::Vector2d v2, double dist2Line)         // new 2018
	{
	    Eigen::Vector2d n, n1;
	    n = v2 - v1;
	    n1 << -n[1], n[0];
	    n = n1/n1.norm()*dist2Line;
	    
	    if (onSegment(v1+n, p1, v2+n)) return true;
	    return false;
	}

	double Agent::dist2staticObst(Eigen::Vector2d p1, Eigen::Vector2d v1, Eigen::Vector2d v2){
	    double d2line = distPoint2line(p1, v1, v2);
	    if (isBehind(p1, v1, v2, d2line) || isBehind(p1, v2, v1, d2line)) return d2line;
	    else return std::min((v1-p1).norm(),(v2-p1).norm());
	}


	inline float Agent::sqrd(float scalar)
	{
		return scalar * scalar;
	}


	// External access functions 

	double Agent::getT(){
		return T_;
	}

	double Agent::getDt(){
		return DT_;
	}

	double Agent::getT_stat(){
	    return T_stat_;
	}

	double Agent::getDClose(){
		return D_CLOSE_;
	}

	double Agent::getDSafe(){
		return D_SAFE_;
	}


	double Agent::getPhiAH(){
		return PHI_AH_*RAD2DEG;
	}

	double Agent::getPhiOT(){
		return PHI_OT_*RAD2DEG;
	}

	double Agent::getPhiHO(){
		return PHI_HO_*RAD2DEG;
	}

	double Agent::getPhiCR(){
		return PHI_CR_*RAD2DEG;
	}

	double Agent::getDInit(){
		return D_INIT_;
	}

	double Agent::getG(){
	    	return G_;
	}

	int Agent::getObstFilterStatus(){
		return OBST_FILTER_ON_;
	}	


	// Todo: Add appropriate validity checks for the set functions
	void Agent::setT(double T){
		int n_samp;
		/*
		if(T>60.0){ 
			T_ = T;

			// resize ASV prediction vector using T_/DT_
			n_samp = T_/DT_;
			asv->x.resize(n_samp);
			asv->y.resize(n_samp);
			asv->psi.resize(n_samp);
			asv->u.resize(n_samp);
			asv->v.resize(n_samp);
			asv->r.resize(n_samp);

			// set T_ and n_samp_ in ASV 
			asv->setT(T);
			asv->setNsamp(n_samp);
		}
		*/
	}

	void Agent::setT_stat(double T_stat){
	    T_stat_ = T_stat;
	}

	void Agent::setDt(double dt){
		int n_samp;
		/*
		if(dt>0.0){ 
			DT_ = dt;

			// resize ASV prediction vector using T_/DT_
			n_samp = T_/DT_;
			asv->x.resize(n_samp);
			asv->y.resize(n_samp);
			asv->psi.resize(n_samp);
			asv->u.resize(n_samp);
			asv->v.resize(n_samp);
			asv->r.resize(n_samp);

			// set DT_ and n_samp_ in ASV 
			asv->setDT(DT_);
			asv->setNsamp(n_samp);
		}
		*/
	}

	void Agent::setDClose(double d_close){
		if(d_close>D_SAFE_) D_CLOSE_ = d_close;
	}

	void Agent::setDSafe(double d_safe){
		if(d_safe>20.0) D_SAFE_ = d_safe;
	}


	void Agent::setPhiAH(double phi_AH){
		PHI_AH_ = phi_AH*DEG2RAD;
	}

	void Agent::setPhiOT(double phi_OT){
		PHI_OT_ = phi_OT*DEG2RAD;
	}

	void Agent::setPhiHO(double phi_HO){
		PHI_HO_ = phi_HO*DEG2RAD;
	}

	void Agent::setPhiCR(double phi_CR){
		PHI_CR_ = phi_CR*DEG2RAD;
	}

	void Agent::setDInit(double D_Init){
		if(D_Init>D_SAFE_) D_INIT_ = D_Init;
	}

	void Agent::setG(double G){
	    if (G > 0) G_ = G;
	}

	void Agent::setObstFilterStatus(int obst_filter_on){
		OBST_FILTER_ON_ = obst_filter_on; // should be done at low speed?	
	}

