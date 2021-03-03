/**
 * \file   obstacle.cpp
 * \brief  Defines the obstacle class.
 */ 


#include "obstacle.h"
#include "kalman.h"
#include "iostream" // for test printout!

#ifndef DRVO_KD_TREE_H_
#include "KdTree.h"
#endif

obstacle::obstacle(const Eigen::Matrix<double,10,1>& state, double T, double dt, int obst_filter_on) // 9->10
: n_samp_(T/dt)
{
	id_ = state(9);
	durationTracked = 0.0;
	durationLost = 0.0;
	
	// check previous ID
	std::cout << "new obst id: " << id_ << std::endl;

	T_ = T;
	dt_ = dt;
	dt_f = dt*10; // Filter sampling time: using 5s fixed intervals, since actual DT_=0.5 for integration/prediction! 

	x_.resize(n_samp_);
	y_.resize(n_samp_);
	u_.resize(n_samp_);
	v_.resize(n_samp_);

	A_ = state(5);
	B_ = state(6);
	C_ = state(7);
	D_ = state(8);

	l = A_ + B_;
	w = C_ + D_;


	// Agent-specific and environment defaults:

	radius_ = 185.2; // [m] use same as D_SAFE: 40 200
	neighborDist_ = 500.0f; // [m] not in use!
	maxNeighbors_ = 100;	// not in use! 
	uncertaintyOffset_ = 0.0f; 


	// Computations

	calculatePosOffsets();

	// Initial prediction state vector
	psi_in = state(2); // for logging
	psi_ = normalize(state(2)); // bug fix: use normalize angle // MR interface output sign change
	x_(0) = state(0) + os_x*cos(psi_) - os_y*sin(psi_);
	y_(0) = state(1) + os_x*sin(psi_) + os_y*cos(psi_);
	u_(0) = state(3);
	v_(0) = state(4);
	
	
	// Current state for DRVO constraints
	position_.setX(state(0)); // 0
	position_.setY(state(1)); // 1

	orientation_ = normalize(state(2)); 	// [rad]

	v_2d(0) = state(3);
	v_2d(1) = state(4);
	rot2d(orientation_,v_2d); // rotate to fixed frame

	velocity_.setX(v_2d(0)); // in fixed frame
	velocity_.setY(v_2d(1)); // in fixed frame

	speed_ = sqrt(state(3)*state(3) + state(4)*state(4));	// [m/s] used only for logging!	


/**/	
	// Before calculating predicted trajectory,
	// use the above position and velocity as measurements, and run them through a Kalman filter

	// Kalman filter parameters
	n = 5; // Number of states : x, y, psi=chi, u, v. 
	m = 5; // Number of measurements

	A.resize(n, n); // System dynamics matrix
	C.resize(m, n); // Output matrix
	Q.resize(n, n); // Process noise covariance
	R.resize(m, m); // Measurement noise covariance
	P.resize(n, n); // Estimate error covariance
	angle_tag.resize(n); // 
	
	x0.resize(n); yM.resize(n); // Initial state and measurement vectors

	// Rotation matrix elements
	psi_ = normalize_angle_360(state(2));
	r11_ = cos(psi_); // use estimate from prevoius state?
	r12_ = -sin(psi_);
	r21_ = sin(psi_);
	r22_ = cos(psi_);
	
	// Discrete LTI/LTV motion
	A <<  1, 0, 0, dt_f*r11_, dt_f*r12_,  
	0, 1, 0, dt_f*r21_, dt_f*r22_,  
	0, 0, 1, 0, 0,  
	0, 0, 0, 1, 0,  
	0, 0, 0, 0, 1;
	
	C <<  1, 0, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, 0, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 1;

	// Covariance matrices
	// the state, psi, is treated as a random constant, and thus not varying with any other state.
	Q << .050,  .000,  .000, .001,  .000,  
	     .000,  .050,  .000, .000,  .001,  
	     .000,  .000,  .010, .000,  .000,  
	     .001,  .000,  .000, .001,  .000,  
	     .000,  .001,  .000, .000,  .001;     
	     	
	R <<  .2,   .0,   .0,   .0,   .0,  
	      .0,   .2,   .0,   .0,   .0,  
	      .0,   .0,   .1,   .0,   .0,  
	      .0,   .0,   .0,   .01,  .0,  
	      .0,   .0,   .0,   .0,   .01;

	P << .1, .0, .0, .0, .0,  
	     .0, .1, .0, .0, .0,  
	     .0, .0, .1, .0, .0,  
	     .0, .0, .0, .1, .0,  
	     .0, .0, .0, .0, .1;
		
	// angle tag: specifies which state is an angle
	angle_tag << 0, 0, 1, 0, 0; 
	     
	// initialize estimates
	x_hat.setZero(); 
	
	/*
	std::cout << "A: \n" << A << std::endl;
	std::cout << "C: \n" << C << std::endl;
	std::cout << "Q: \n" << Q << std::endl;
	std::cout << "R: \n" << R << std::endl;
	std::cout << "P: \n" << P << std::endl;
	std::cout << "\n" << std::endl;
	*/
	
	// Construct the filter
	filter = new KalmanFilter(id_, dt_f, A, C, Q, R, P, angle_tag);	
	
	// Previous heading/estimate is the same as current heading for a new obstacle
	psi_prev = psi_;
	psi_hat_prev = psi_;
	
	// Best guess of initial states	
	x0 << x_(0), y_(0), psi_, u_(0), v_(0); 
	
	filter->init(0.0, x0); // time important?	

	filter->update(x0, durationLost); // measurement yM=x0 in the first time step

	if (obst_filter_on >0){	
	
		// obtain estimated state from filter
		x_hat = filter->state(); 	
		
		// obtain estimated P
		P = filter->errorCov();	
		
		// obtain duration tracked
		durationTracked = filter->time();
		
		// use current state estimate for prediction
		x_(0) = x_hat(0);
		y_(0) = x_hat(1);
		psi_  = normalize(x_hat(2)); 
		u_(0) = x_hat(3);
		v_(0) = x_hat(4);
	
		std::cout << "\n" << std::endl;
		std::cout << "P_hat: \n" << P.transpose() << std::endl;
		std::cout << "x_hat: \n" << x_hat.transpose() << std::endl;
		std::cout << "psi_hat: \n" << psi_*180.0/M_PI << std::endl;
		std::cout << "\n" << std::endl;
	}else{
		psi_ = normalize(state(2));
	}
	
	// use current state estimate for DRVO constraints
	position_.setX(x_(0)); 
	position_.setY(y_(0));  

	orientation_ = psi_; 	// [rad]

	v_2d(0) = u_(0);
	v_2d(1) = v_(0);
	rot2d(orientation_,v_2d); // rotate to fixed frame

	velocity_.setX(v_2d(0)); // in fixed frame
	velocity_.setY(v_2d(1)); // in fixed frame

	speed_ = sqrt(u_(0)*u_(0) + v_(0)*v_(0));	// [m/s] used only for logging!
			
	
	// use estimated heading for rotation?
	r11_ = cos(psi_);
	r12_ = -sin(psi_);
	r21_ = sin(psi_);
	r22_ = cos(psi_); 
		

	calculateTrajectory();
	
	psi_prev = normalize_angle_360(state(2)); // save current psi_ measurement for next time step
	psi_hat_prev = normalize_angle_360(psi_); // save current psi_ estimate for next time step
};

obstacle::~obstacle(){
};

Eigen::VectorXd obstacle::getX(){
	return x_;
}

Eigen::VectorXd obstacle::getY(){
	return y_;
}

Eigen::VectorXd obstacle::getU(){
	return u_;
}

Eigen::VectorXd obstacle::getV(){
	return v_;
}

double obstacle::getPsi(){
	return psi_;
}

double obstacle::getA(){
	return A_;
}

double obstacle::getB(){
	return B_;
}

double obstacle::getC(){
	return C_;
}

double obstacle::getD(){
	return D_;
}

double obstacle::getL(){
	return l;
}

double obstacle::getW(){
	return w;
}

std::size_t obstacle::getId(){
	return id_;
}

void obstacle::calculatePosOffsets(){
	os_x = A_-B_;
	os_y = D_-C_;
}

void obstacle::calculateTrajectory()
{
	for (int i = 1; i < n_samp_;  i++)
	{
		x_(i) = (x_(i-1) + (r11_*u_(i-1) + r12_*v_(i-1))*dt_);
		y_(i) = (y_(i-1) + (r21_*u_(i-1) + r22_*v_(i-1))*dt_);
		u_(i) = (u_(i-1));
		v_(i) = (v_(i-1));
	}

}


void obstacle::updateTrajectory(int obst_filter_on){

	Eigen::VectorXd state_copy(10);
	state_copy << x_hat(0), x_hat(1), x_hat(2), x_hat(3), x_hat(4), A_, B_, C_, D_, id_;
	updateTrajectory(state_copy, obst_filter_on);
}

void obstacle::updateTrajectory(const Eigen::Matrix<double,10,1>& state, int obst_filter_on)
{
	// check previous ID
	//std::cout << "previous id: " << id_ << ", current id: " << state(9) << ", filter id: " << filter->filter_id() << std::endl;
	
	psi_in = state(2); // for logging
	psi_ = normalize(state(2)); // bug fix: use normalize angle // MR interface output sign change
	x_(0) = state(0) + os_x*cos(psi_) - os_y*sin(psi_);
	y_(0) = state(1) + os_x*sin(psi_) + os_y*cos(psi_);
	u_(0) = state(3);
	v_(0) = state(4);
	
	
	if (obst_filter_on >0){
	
		// Rotation matrix elements
		psi_ = normalize_angle_360(state(2));
		r11_ = cos(psi_);
		r12_ = -sin(psi_);
		r21_ = sin(psi_);
		r22_ = cos(psi_);
	
		// update system dynamics matrix
		A << 1, 0, 0, dt_f*r11_, dt_f*r12_,  
		0, 1, 0, dt_f*r21_, dt_f*r22_,  
		0, 0, 1, 0, 0,  
		0, 0, 0, 1, 0,  
		0, 0, 0, 0, 1;	

		//std::cout << "A: \n" << A << std::endl;
		//std::cout << "\n" << std::endl;
		
				
		// Measurements
		yM << x_(0), y_(0), psi_, u_(0), v_(0);
	
		filter->update(yM, durationLost, dt_f, A); // measurement y=x0 in the first time step
	
		// obtain estimated state from filter
		x_hat = filter->state(); 
		
		// obtain estimated P
		P = filter->errorCov();
		
		// obtain duration tracked
		durationTracked = filter->time();
	
		std::cout << "P_hat: \n" << P.transpose() << std::endl;
		std::cout << "x_hat: \n" << x_hat.transpose() << std::endl;
		std::cout << "\n" << std::endl;
		
		// use current state estimate for prediction 
		x_(0) = x_hat(0);
		y_(0) = x_hat(1);
		psi_  = normalize(x_hat(2));
		u_(0) = x_hat(3);
		v_(0) = x_hat(4);
	
		//std::cout << "psi_hat_prev: \n" << psi_hat_prev*180.0/M_PI << std::endl;
		std::cout << "psi_hat: \n" << psi_*180.0/M_PI << std::endl;
		std::cout << "\n" << std::endl;
		
		//std::cout << "P_hat: \n" << filter->errorCov() << std::endl;
		//std::cout << "\n" << std::endl;
	}
	
	// use current state estimate for DRVO constraints
	position_.setX(x_(0)); 
	position_.setY(y_(0));  

	orientation_ = psi_; 	// [rad]

	v_2d(0) = u_(0);
	v_2d(1) = v_(0);
	rot2d(orientation_,v_2d); // rotate to fixed frame

	velocity_.setX(v_2d(0)); // in fixed frame
	velocity_.setY(v_2d(1)); // in fixed frame

	speed_ = sqrt(u_(0)*u_(0) + v_(0)*v_(0));	// [m/s] used only for logging!
	
	
	// Rotation matrix elements. Uses estimated heading if filter is on.
	r11_ = cos(psi_);
	r12_ = -sin(psi_);
	r21_ = sin(psi_);
	r22_ = cos(psi_);

	calculateTrajectory();	
	
	psi_prev = normalize_angle_360(state(2)); // save current psi_ measurement for next time step
	psi_hat_prev = normalize_angle_360(psi_); // save current psi_ estimate for next time step

}
 

inline double obstacle::normalize(double angle)
{
	while(angle <= -M_PI) angle += 2*M_PI; 
	while (angle > M_PI) angle -= 2*M_PI;
	return angle;
}

inline double obstacle::normalize_angle_360(double angle){
	angle = fmod(angle,2*M_PI);
	if (angle < 0)
	angle += 2*M_PI;
	return angle;
}

inline double obstacle::angle_diff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}

void obstacle::rot2d(double yaw, Eigen::Vector2d &res){
	Eigen::Matrix2d R;
	R << cos(yaw), -sin(yaw),
		 sin(yaw), cos(yaw);
	res = R*res;
}

