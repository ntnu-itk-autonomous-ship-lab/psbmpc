/**
 * \file   obstacle.h
 * \brief  Declares the obstacle class.
 */ 

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <vector>
#include "Eigen/Dense"

//#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
//#endif

#include "kalman.h"


class obstacle
{
	public:

	/// Constructor
	obstacle(const Eigen::Matrix<double,10,1>& state, double T, double dt, int obst_filter_on); //9->10

	/// Destructor
	~obstacle();

	Eigen::VectorXd getX();
	Eigen::VectorXd getY();
	Eigen::VectorXd getU();
	Eigen::VectorXd getV();
	double getPsi();
	double getA();
	double getB();
	double getC();
	double getD();
	double getL();
	double getW();
	std::size_t getId();	

	Eigen::VectorXd x_;
	Eigen::VectorXd y_;
	Eigen::VectorXd u_;
	Eigen::VectorXd v_;

	double psi_, psi_in, psi_prev, psi_hat_prev, dPsi_; // dPsi_ = psi_ - psi_prev
	double A_, B_, C_, D_, l, w;
	double os_x, os_y;
	double durationTracked, durationLost;

	// Agent/dynamic obstacle-specific parameters: 
	std::size_t id_;
	Vector2 position_;
	Vector2 velocity_;

	std::size_t maxNeighbors_;
	float neighborDist_;
	float speed_;
	float orientation_;
	float radius_;
	float uncertaintyOffset_;
	
	// Kalman parameters
	int n; // Number of states : x, y, psi=chi, u, v. 
	int m; // Number of measurements

	Eigen::MatrixXd A; // System dynamics matrix
	Eigen::MatrixXd C; // Output matrix
	Eigen::MatrixXd Q; // Process noise covariance
	Eigen::MatrixXd R; // Measurement noise covariance
	Eigen::MatrixXd P; // Estimate error covariance
	
	Eigen::VectorXd x0, yM; // Initial state, measurement
	Eigen::VectorXd x_hat; // Current state estimate
	
	Eigen::VectorXd angle_tag; // identifies angles in the state vector
	
	KalmanFilter* filter;
	
	void updateTrajectory(const Eigen::Matrix<double,10,1>& state, int obst_filter_on);
	void updateTrajectory(int obst_filter_on);
	

	private:

	void calculatePosOffsets();

	void calculateTrajectory();

	inline double normalize(double angle);
	inline double normalize_angle_360(double angle);
	inline double angle_diff(double a,double b);

	void rot2d(double yaw, Eigen::Vector2d &res);

	const int n_samp_;
	double T_;
	double dt_, dt_f;
	Eigen::Vector2d v_2d;

	//Elements of the rotation matrix
	double r11_, r12_, r21_, r22_;


	friend class KdTree;

};

#endif /* OBSTACLE_H_ */
