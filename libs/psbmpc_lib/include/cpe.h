/****************************************************************************************
*
*  File name : cpe.h
*
*  Function  : Header file for the Collision Probability Estimator (CPE)
*
*  
*            ---------------------
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

#ifndef _CPE_H_
#define _CPE_H_

#include <Eigen/Dense>

// See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" or 
// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
enum CPE_Method 
{
	CE,														// Consider positional uncertainty only
	MCSKF4D													// Consider uncertainty in both position and velocity along piece-wise linear segments 
};

class CPE{
private:
  

public:


};

#endif
