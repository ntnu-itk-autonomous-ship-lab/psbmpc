/*
 *    This file is part of SB-MPC Library.
 *
 *    SB-MPC -- Scenario-Based MPC for Maritime Collision Avoidance.
 *    Copyright (C) 2016-2019 Inger Berge Hagen, Giorgio D. Kwame Minde Kufoalor, 
 *    NTNU Trondheim.
 *    Developed within the Autosea Project (Sensor fusion and collision avoidance for 
 *    autonomous surface vehicles) under the supervision of Tor Arne Johansen. 
 *    All rights reserved.
 *
 *    SB-MPC Library is software used according to the conditions of the Autosea Consortium.
 *    <https://www.ntnu.edu/autosea>
 */

 
/**
 *    \file   ship_model.cpp
 *    \brief  Defines the shipModel class.
 *    \author Inger Berge Hagen, Giorgio D. K. M. Kufoalor
 */ 


#include "ship_model.h"
#include  <iostream>

static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;


shipModel::shipModel(double T, double dt)
: n_samp_(T/dt)
{
	x.resize(n_samp_);
	y.resize(n_samp_);
	psi.resize(n_samp_);
	u.resize(n_samp_);
	v.resize(n_samp_);
	r.resize(n_samp_);

	tau = Eigen::Vector3d::Zero();

	// Simulation parameters
	T_ = T;
	DT_ = dt;

	// Model parameters
	rudder_d = 4.0; // distance from rudder to CG
	A_ = 5; // [m]  in reality the length is 14,5 m.
	B_ = 5; // [m]
	C_ = 1.5; // [m]
	D_ = 1.5; // [m]
	l = (A_ + B_);
	w = (C_ + D_);
	calculate_position_offsets();
	M = 3980.0; // [kg]
	I_z = 19703.0; // [kg m2]

	// Added M terms
	X_udot = 0.0;
	Y_vdot = 0.0;
	Y_rdot = 0.0;
	N_vdot = 0.0;
	N_rdot = 0.0;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	X_u	= -50.0;
	Y_v = -200.0;
	Y_r = 0.0;
	N_v = 0.0;
	N_r = -1281.0;//-3224.0;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	X_uu = -135.0;
	Y_vv = -2000.0;
	N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;

	Eigen::Matrix3d Mtot;
	Mtot << M - X_udot, 0, 0,
	        0, M-Y_vdot, -Y_rdot,
	        0, -Y_rdot, I_z-N_rdot;
	Minv = Mtot.inverse();

	//Force limits
	Fx_min = -6550.0;
	Fx_max = 13100.0;
	Fy_min = -645.0;
	Fy_max = 645.0;

	// Controller parameters
	Kp_u = 1.0;
	Kp_psi = 5.0;
	Kd_psi = 1.0;
	Kp_r = 8.0;
		
	//Motion limits
	r_max = 0.34 * DEG2RAD; // [rad/s] default max yaw rate
	psi_response_factor = 0.7; // 1.0 means psi_d must be fully achieved before adapting

}


shipModel::~shipModel(){
}


Eigen::VectorXd shipModel::getX(){
	return x;
}

Eigen::VectorXd shipModel::getY(){
	return y;
}

Eigen::VectorXd shipModel::getPsi(){
	return psi;
}

Eigen::VectorXd shipModel::getU(){
	return u;
}

Eigen::VectorXd shipModel::getV(){
	return v;
}

Eigen::VectorXd shipModel::getR(){
	return r;
}

double shipModel::getA(){
	return A_;
}

double shipModel::getB(){
	return B_;
}

double shipModel::getC(){
	return C_;
}

double shipModel::getD(){
	return D_;
}

double shipModel::getL(){
	return l;
}

double shipModel::getW(){
	return w;
}

double shipModel::getT(){
	return T_;
}

double shipModel::getDT(){
	return DT_;	
}

double shipModel::getNsamp(){
	return n_samp_;
}


void  shipModel::setA(double A){
	A_ = A;
}

void shipModel::setB(double B){
	B_ = B;
}

void shipModel::setC(double C){
	C_ = C;
}

void shipModel::setD(double D){
	D_ = D;
}


void shipModel::setT(double T){
	T_ = T;
}

void shipModel::setDT(double DT){
	DT_ = DT;	
}

void shipModel::setNsamp(int n_samp){
	n_samp_ = n_samp;
}


void shipModel::calculate_position_offsets(){
	os_x = A_-B_;
	os_y = D_-C_;
}


void shipModel::eulersMethod(const Eigen::Matrix<double,6,1>& state, double u_d, double psi_d, const Eigen::Matrix<double,-1,2>&waypoints_, double Chi_ca, int course_change_point, int guidance_strategy, double R, double de, double Ki)
{
	Eigen::Vector2d d_s_wp1, los_s_wp1, d_wp0_wp1, los_wp0_wp1;
	int n_Hd = 0; //50, 200; //getHd()/getDT(); // 30/0.5
	
	//int guidance_strategy = 0; // 0=Line-of-sight (LOS), 1= WP-pursiut (WPP), >1= Course-Hold (CH) 
	
	// LOS guidance dynamics variables and parameters
	double psi_d0 = normalize_angle(psi_d-Chi_ca);
	double psi_r; // LOS path course correction 
	double e=0, e_integral=0; // cross-track error and its integral
	double max_integral_corr = M_PI*20.0/180.0; //max integral correction pi*20/180
	Eigen::MatrixXd waypoints(waypoints_.rows(), waypoints_.cols()); waypoints = waypoints_;
		
	
	int init_samp = course_change_point*25/DT_; // assumes course is constant for at least (25s)
	if (guidance_strategy >= 2)  
		init_samp = init_samp*4;
		
	if (course_change_point > 3) init_samp = course_change_point; // TODO:change 3 to n_cp in sb_mpc.cpp 
	
	
	// initial prediction state
	if (init_samp == 0){
		psi(0) = normalize_angle(state(2)); 
		x(0) = state(0) + os_x*cos(psi(0)) - os_y*sin(psi(0));
		y(0) = state(1) + os_x*sin(psi(0)) + os_y*cos(psi(0));
		u(0) = state(3);
		v(0) = state(4);
		r(0) = state(5);
	}
	
	// detect whether waypoints have been switched when init_samp>0, and copy next waypoint	 
	if (init_samp > 0 && guidance_strategy < 2){ 	
		d_s_wp1(0) = waypoints(1,0) - x(init_samp);  
		d_s_wp1(1) = waypoints(1,1) - y(init_samp); 
		los_s_wp1 = d_s_wp1/d_s_wp1.norm();	
		
		d_wp0_wp1(0) = waypoints(1,0) - waypoints(0,0); 
		d_wp0_wp1(1) = waypoints(1,1) - waypoints(0,1); 
		los_wp0_wp1 = d_wp0_wp1/d_wp0_wp1.norm();	
			
		bool leg_passed = los_wp0_wp1.dot(-los_s_wp1) > cos(90*DEG2RAD);		
		
		//std::cout << "leg_passed :  " << leg_passed << std::endl;
			
		// update waypoints for the initial sample 
		if (leg_passed){
			// use next waypoints as current waypoints 
			for (int k=0; k < waypoints_.rows()-1; k++){
				waypoints.row(k) = waypoints_.row(k+1);
			}		
		}	
	}
	

	Eigen::Vector3d temp;
	double r11, r12, r21, r22; // rotation matrix elements		
	

	// compute new course
	double psi_path = atan2(waypoints(1,1) - waypoints(0,1), 
				waypoints(1,0) - waypoints(0,0));

	// initial along-track error using Eq (10.58 or 10.10) in Fossen 2011
	// NB! the same as remaining track distance  
	double along_track_dist0 = fabs((waypoints(1,0) - x(0))*cos(psi_path) + (waypoints(1,1) - y(0))*sin(psi_path));				
	
	// ASV dynamics prediction loop
	
	for (int i = init_samp; i < n_samp_-1; i++){	
				    	
	    // distance of track from wp0 to wp1
		double track_dist = sqrt(pow(waypoints(1,0) - waypoints(0,0),2) + pow(waypoints(1,1) - waypoints(0,1),2));
		
		
		if ( track_dist > R + A_ + B_ ) { // else last WP reached, assume constant course! 
	
			
			// Guidance prediction OPT1: use next wp direction as desired course after t= t_Hd	
			
			//if (1==guidance_strategy && i > n_Hd && (fabs(angle_diff(psi_path, psi_d0)) > 15*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 0.9 means the asv is on the move to the WP2	
			if (1==guidance_strategy && i > n_Hd){ // 0.9 means the asv is on the move to the WP2	
	
				// use pure-pursuit guidance behavior after t= t_Hd?
				d_s_wp1(0) = waypoints(1,0) - x(i); 
				d_s_wp1(1) = waypoints(1,1) - y(i); 
				
				// use course-hold guidance behavior after t= t_Hd?	
				//d_s_wp1(0) = waypoints(1,0) - waypoints(0,0); 
				//d_s_wp1(1) = waypoints(1,1) - waypoints(0,0); 
						
				los_s_wp1 = d_s_wp1/d_s_wp1.norm();
		
				psi_d = atan2(d_s_wp1(1),d_s_wp1(0));
			
				// apply offset
				psi_d = psi_d + Chi_ca;	// approx behavior as input psi_d	
			}
	
	
			// Guidance prediction OPT2: simulate LOS dynamics	
						
			//if (0==guidance_strategy && i > n_Hd && (fabs(angle_diff(psi_path, psi_d0)) > 0*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 15 deg is the grid size for course offsets 
			if (0==guidance_strategy && i > n_Hd){	
		
				// compute cross-track error Eq (10.59 or 10.10) in Fossen 2011
				e = -(x(i) - waypoints(1,0)) * sin(psi_path) 
				    +(y(i) - waypoints(1,1)) * cos(psi_path);
				e_integral += e * DT_;
				
				if (e_integral * Ki > max_integral_corr){
					e_integral -= e * DT_;
				}
					
				psi_r = atan2( -(e + Ki * e_integral) , de);
				psi_d = psi_path + psi_r;
			
				// apply offset
				psi_d = psi_d + Chi_ca;	// approx same behavior as input psi_d		
			}
			
						
			// compute values for handling waypoint switching events
							
			// asv-wp radius
			double asv_wp_radius_sqrd = pow(x(i) - waypoints(1,0),2) + pow(y(i) - waypoints(1,1),2);
			
			// along-track error using Eq (10.58 or 10.10) in Fossen 2011
			// NB! the same as remaining track distance  
			double along_track_dist = fabs((waypoints(1,0) - x(i))*cos(psi_path) + (waypoints(1,1) - y(i))*sin(psi_path));
								
	
			// WP switching criteria
			bool switch_wp = 
				// circle of acceptance: (x_wp1 - x)² + (y_wp1 - y)² < R² 
				asv_wp_radius_sqrd < R*R
				||	
				// progress along path: s_total - s(t) < R or s(t) < R? depends on s(t)
				along_track_dist < R; // track_dist - ?
		
			if (switch_wp){
				// use next waypoints as current waypoints 
				for (int k=0; k < waypoints_.rows()-1; k++){
					waypoints.row(k) = waypoints_.row(k+1);
				}
				
				// reset los integral
				e_integral = 0; 
			
				// compute new course
				psi_path = atan2(waypoints(1,1) - waypoints(0,1), 
						 waypoints(1,0) - waypoints(0,0));	
			}		
		}
		
		
		psi_d = normalize_angle_diff(psi_d, psi(i));	// TODO: check this with the controller input!
		//psi_d = normalize_angle(psi_d);
		

		// compute rotation matrix 
		r11 = cos(psi(i));
		r12 = -sin(psi(i));
		r21 = sin(psi(i));
		r22 = cos(psi(i));

		// Calculate coriolis and damping matrices according to Fossen, 2011 or Stenersen, 2014.
		Cvv(0) = (-M*v(i) + Y_vdot*v(i) + Y_rdot*r(i)) * r(i);
		Cvv(1) = ( M*u(i) - X_udot*u(i)) * r(i);
		Cvv(2) = (( M*v(i) - Y_vdot*v(i) - Y_rdot*r(i) ) * u(i) +
		            ( -M*u(i) + X_udot*u(i)) * v(i));

		Dvv(0) = - (X_u + X_uu*fabs(u(i)) + X_uuu*u(i)*u(i)) * u(i);
		Dvv(1) = - ((Y_v*v(i) + Y_r*r(i)) +
		              (Y_vv*fabs(v(i))*v(i) + Y_vvv*v(i)*v(i)*v(i)));
		Dvv(2) = - ((N_v*v(i) + N_r*r(i)) +
		              (N_rr*fabs(r(i))*r(i) + N_rrr*r(i)*r(i)*r(i)));

		this->updateCtrlInput(u_d, psi_d, i);

		// Integrate system

		x(i+1) = x(i) + DT_*(r11*u(i) + r12*v(i));
		y(i+1) = y(i) + DT_*(r21*u(i) + r22*v(i));
		psi(i+1) = psi(i) + DT_*r(i);
		temp = Minv * (tau - Cvv - Dvv);
		u(i+1) = u(i) + DT_*temp(0);
		v(i+1) = v(i) + DT_*temp(1);
		r(i+1) = r(i) + DT_*temp(2);		
				
		// limit yaw rate 	
		//if (r(i+1) > r_max) r(i+1) = r_max;

		// Keep yaw within [-PI,PI)
		psi(i+1) = normalize_angle(psi(i+1));

	}
}

void shipModel::linearPrediction(const Eigen::Matrix<double,6,1>& state, double u_d, double psi_d, const Eigen::Matrix<double,-1,2>&waypoints_, double Chi_ca, int course_change_point, int guidance_strategy, double R, double de, double Ki){

	Eigen::Vector2d d_s_wp1, los_s_wp1, d_wp0_wp1, los_wp0_wp1;
	int n_Hd = 0; //50, 200; //getHd()/getDT(); // 30/0.5
	
	//int guidance_strategy = 0; // 0=Line-of-sight (LOS), 1= WP-pursiut (WPP), >1= Course-Hold (CH) 
	
	// LOS guidance dynamics variables and parameters
	double psi_d0 = normalize_angle(psi_d-Chi_ca);
	double psi_r; // LOS path course correction 
	double e=0, e_integral=0; // cross-track error and its integral
	double max_integral_corr = M_PI*20.0/180.0; //max integral correction pi*20/180
	Eigen::MatrixXd waypoints(waypoints_.rows(), waypoints_.cols()); waypoints = waypoints_;
	
	
	int init_samp = course_change_point*25/DT_; // assumes course is constant for at least (25s)
	if (guidance_strategy >= 2)  
		init_samp = init_samp*4;
	
	
	// initial prediction state
	if (init_samp == 0){

		psi(0) = normalize_angle(state(2)); // bug fix: use normalized psi // MR interface output sign change
		psi_d = normalize_angle(psi_d); // bug fix: use normalized psi_d
		x(0) = state(0) + os_x*cos(psi(0)) - os_y*sin(psi(0));
		y(0) = state(1) + os_x*sin(psi(0)) + os_y*cos(psi(0));

		/*psi(0) = normalize_angle(psi_d);
		x(0) = state(0) + os_x*cos(psi_d) - os_y*sin(psi_d);
		y(0) = state(1) + os_x*sin(psi_d) + os_y*cos(psi_d);*/
		u(0) = state(3);
		v(0) = state(4);
		r(0) = state(5);	
	}
	
	// detect whether waypoints have been switched when init_samp>0, and copy next waypoint	 
	if (init_samp > 0 && guidance_strategy < 2){ 	
		d_s_wp1(0) = waypoints(1,0) - x(init_samp);  
		d_s_wp1(1) = waypoints(1,1) - y(init_samp); 
		los_s_wp1 = d_s_wp1/d_s_wp1.norm();	
		
		d_wp0_wp1(0) = waypoints(1,0) - waypoints(0,0); 
		d_wp0_wp1(1) = waypoints(1,1) - waypoints(0,1); 
		los_wp0_wp1 = d_wp0_wp1/d_wp0_wp1.norm();	
			
		bool leg_passed = los_wp0_wp1.dot(-los_s_wp1) > cos(90*DEG2RAD);		
		
		//std::cout << "leg_passed :  " << leg_passed << std::endl;
			
		// update waypoints for the initial sample 
		if (leg_passed){
			// use next waypoints as current waypoints 
			for (int k=0; k < waypoints_.rows()-1; k++){
				waypoints.row(k) = waypoints_.row(k+1);
			}		
		}	
	}
	
	double r11, r12, r21, r22;


	// compute new course
	double psi_path = atan2(waypoints(1,1) - waypoints(0,1), 
				waypoints(1,0) - waypoints(0,0));

	// initial along-track error using Eq (10.58 or 10.10) in Fossen 2011
	// NB! the same as remaining track distance  
	double along_track_dist0 = fabs((waypoints(1,0) - x(0))*cos(psi_path) + (waypoints(1,1) - y(0))*sin(psi_path));				
	
	// ASV dynamics prediction loop
	
	for (int i = init_samp; i < n_samp_-1; i++){	
				    	
	    // distance of track from wp0 to wp1
		double track_dist = sqrt(pow(waypoints(1,0) - waypoints(0,0),2) + pow(waypoints(1,1) - waypoints(0,1),2));
		
		
		if ( track_dist > R + A_ + B_ ) { // else last WP reached, assume constant course! 
	
			
			// Guidance prediction OPT1: use next wp direction as desired course after t= t_Hd	
			
			//if (1==guidance_strategy && i > n_Hd && (fabs(angle_diff(psi_path, psi_d0)) > 15*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 0.9 means the asv is on the move to the WP2	
			if (1==guidance_strategy && i > n_Hd){ // 0.9 means the asv is on the move to the WP2	
	
				// use pure-pursuit guidance behavior after t= t_Hd?
				d_s_wp1(0) = waypoints(1,0) - x(i); 
				d_s_wp1(1) = waypoints(1,1) - y(i); 
				
				// use course-hold guidance behavior after t= t_Hd?	
				//d_s_wp1(0) = waypoints(1,0) - waypoints(0,0); 
				//d_s_wp1(1) = waypoints(1,1) - waypoints(0,0); 
						
				los_s_wp1 = d_s_wp1/d_s_wp1.norm();
		
				psi_d = atan2(d_s_wp1(1),d_s_wp1(0));
			
				// apply offset
				psi_d = psi_d + Chi_ca;	// approx behavior as input psi_d	
			}
	
	
			// Guidance prediction OPT2: simulate LOS dynamics	
						
			//if (0==guidance_strategy && i > n_Hd && (fabs(angle_diff(psi_path, psi_d0)) > 0*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 15 deg is the grid size for course offsets 
			if (0==guidance_strategy && i > n_Hd){	
		
				// compute cross-track error Eq (10.59 or 10.10) in Fossen 2011
				e = -(x(i) - waypoints(1,0)) * sin(psi_path) 
				    +(y(i) - waypoints(1,1)) * cos(psi_path);
				e_integral += e * DT_;
				
				if (e_integral * Ki > max_integral_corr){
					e_integral -= e * DT_;
				}
					
				psi_r = atan2( -(e + Ki * e_integral) , de);
				psi_d = psi_path + psi_r;
			
				// apply offset
				psi_d = psi_d + Chi_ca;	// approx same behavior as input psi_d		
			}
			
						
			// compute values for handling waypoint switching events
							
			// asv-wp radius
			double asv_wp_radius_sqrd = pow(x(i) - waypoints(1,0),2) + pow(y(i) - waypoints(1,1),2);
			
			// along-track error using Eq (10.58 or 10.10) in Fossen 2011
			// NB! the same as remaining track distance  
			double along_track_dist = fabs((waypoints(1,0) - x(i))*cos(psi_path) + (waypoints(1,1) - y(i))*sin(psi_path));
								
	
			// WP switching criteria
			bool switch_wp = 
				// circle of acceptance: (x_wp1 - x)² + (y_wp1 - y)² < R² 
				asv_wp_radius_sqrd < R*R
				||	
				// progress along path: s_total - s(t) < R or s(t) < R? depends on s(t)
				along_track_dist < R; // track_dist - ?
		
			if (switch_wp){
				// use next waypoints as current waypoints 
				for (int k=0; k < waypoints_.rows()-1; k++){
					waypoints.row(k) = waypoints_.row(k+1);
				}
				
				// reset los integral
				e_integral = 0; 
			
				// compute new course
				psi_path = atan2(waypoints(1,1) - waypoints(0,1), 
						 waypoints(1,0) - waypoints(0,0));	
			}		
		}


		psi_d = normalize_angle(psi_d);		


		r11 = cos(psi_d);
		r12 = -sin(psi_d);
		r21 = sin(psi_d);
		r22 = cos(psi_d);
		
		x(i+1) = x(i) + DT_*(r11*u(i) + r12*v(i));
		y(i+1) = y(i) + DT_*(r21*u(i) + r22*v(i));
		psi(i+1) = psi_d;
		u(i+1) = u_d;
		v(i+1) = 0;

	}
}


void shipModel::updateCtrlInput(double u_d, double psi_d, int i){
	double Fx = Cvv[0] + Dvv[0] + Kp_u*M*(u_d - u(i));
	double Fy = 0.0;

    Fy = (Kp_psi * I_z ) * ((psi_d - psi(i)) - Kd_psi*r(i));
    Fy *= 1.0 / rudder_d;

	// Saturate
	if (Fx < Fx_min)
	  Fx = Fx_min;
	if (Fx > Fx_max)
	  Fx = Fx_max;

	if (Fy < Fy_min)
	  Fy = Fy_min;
	if (Fy > Fy_max)
	  Fy = Fy_max;

	tau[0] = Fx;
	tau[1] = Fy;
	tau[2] = rudder_d * Fy;
}


double shipModel::normalize_angle(double angle){

	if( isinf(angle)) return angle;

	while(angle <= -M_PI){
		angle += 2*M_PI;
	}

	while (angle > M_PI){
		angle -= 2*M_PI;
	}

	return angle;
}


double shipModel::normalize_angle_diff(double angle, double angle_ref){
	double new_angle;
	double diff = angle_ref - angle;

	if (isinf(angle) || isinf(angle_ref)) return angle;

	// Get angle within 2*PI of angle_ref
	if (diff > 0){
		new_angle = angle +(diff - fmod(diff, 2*M_PI));
	}else{
		new_angle = angle + (diff + fmod(-diff, 2*M_PI));
	}

	// Get angle on side closest to angle_ref
	diff = angle_ref - new_angle;
	if (diff > M_PI){
		new_angle += 2*M_PI;
	}else if (diff < -M_PI){
		new_angle -= 2*M_PI;
	}
	return new_angle;
}


inline double shipModel::angle_diff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}


