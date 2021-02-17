/****************************************************************************************
*
*  File name : joint_prediction_manager.cuh
*
*  Function  : Header file for the prediction management interface used in the PSB-MPC
*			   when considering dynamic obstacles with their own deliberative COLAV
*			   systems.
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

#ifndef _JOINT_PREDICTION_MANAGER_CUH_
#define _JOINT_PREDICTION_MANAGER_CUH_

#include <thrust/device_vector.h>
#include "psbmpc_defines.h"
#include "obstacle_manager.cuh"
#include "Eigen/Dense"
#include <vector>
#include <memory>
#include <string>



class Joint_Prediction_Manager
{
private:

	// Array of strings and precisions for the obstacle status states
	std::vector<std::string> status_str = {"ID", "SOG", "COG", "RB", "RNG", "HL", "IP", "AH", "SB", "HO", "CRG", "OTG", "OT"};
	

public:

	__host__ __device__ Joint_Prediction_Manager(const int n_obst);

	__host__ __device__ ~Joint_Prediction_Manager();

	__host__ __device__ void update_obstacle_status(const int i, const TML::Vector4f &obstacle_i_state, const int k);

	__host__ __device__ void display_obstacle_information(const int i);

};

#endif 