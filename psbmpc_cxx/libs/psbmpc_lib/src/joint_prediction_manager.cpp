/****************************************************************************************
*
*  File name : joint_prediction_manager.cpp
*
*  Function  : Class functions for the obstacle management interface
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

#include "joint_prediction_manager.h"
#include <string>
#include <iostream>
#include <iomanip>

/****************************************************************************************
*  Name     : Class constructor
*  Function : 
*  Author   :
*  Modified :
*****************************************************************************************/
Joint_Prediction_Manager::Joint_Prediction_Manager(
	const int n_obst
	) 
{
	width_arr[0] = 1; 		// Obstacle ID
	width_arr[1] = 1;		// Obstacle SOG
	width_arr[2] = 7; 		// Obstacle COG
	width_arr[3] = 5;		// Obstacle R-BRG
	width_arr[4] = 5; 		// Obstacle RNG
	width_arr[5] = 2;		// Obstacle HL
	width_arr[6] = 2; 		// Obstacle IP
	width_arr[7] = 2;		// Obstacle AH
	width_arr[8] = 2; 		// Obstacle SB
	width_arr[9] = 2;		// Obstacle HO
	width_arr[10] = 2; 		// Obstacle CRG
	width_arr[11] = 2;		// Obstacle OTG
	width_arr[12] = 2; 		// Obstacle OT

	data.resize(n_obst);
}

Joint_Prediction_Manager::~Joint_Prediction_Manager() = default;

/****************************************************************************************
*  Name     : update_obstacle_status
*  Function : Updates various information on each obstacle at the current time
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Joint_Prediction_Manager::update_obstacle_status(
	const int i, 														// In: Index of obstacle asking for a status update
	const Eigen::Vector4d &obstacle_i_state, 							// In: Current predicted time state of obstacle i
	const int k															// In: Index of the current predicted time t_k
	)
{
	int n_obst = data[i].obstacles.size();
	data[i].obstacle_status.resize(13, n_obst);
	double ID_0, RB_0, COG_0, SOG_0; 
	Eigen::Vector2d d_ij;
	Eigen::Vector4d xs_j;

	data[i].HL_0.resize(n_obst); data[i].HL_0.setZero();
	for(int j = 0; j < n_obst; j++)
	{
		xs_j = data[i].obstacles[j].get_state(k);

		ID_0 = data[i].obstacles[j].get_ID();
		
		d_ij = (xs_j.block<2, 1>(0, 0) - obstacle_i_state.block<2, 1>(0, 0));

		COG_0 = atan2(xs_j(3), xs_j(2));

		SOG_0 = xs_j.block<2, 1>(2, 0).norm();

		RB_0 = angle_difference_pmpi(atan2(d_ij(1), d_ij(0)), obstacle_i_state(2));

		data[i].obstacle_status.col(j) << ID_0, 										// Obstacle ID
								  SOG_0, 												// Speed over ground of obstacle
								  wrap_angle_to_02pi(COG_0) * RAD2DEG, 					// Course over ground of obstacle
								  RB_0 * RAD2DEG, 										// Relative bearing
								  d_ij.norm(),											// Range
								  data[i].HL_0[j], 										// Hazard level of obstacle at optimum
								  data[i].IP_0[j], 										// If obstacle is passed by or not 
								  data[i].AH_0[j], 										// If obstacle is ahead or not
								  data[i].S_TC_0[j], 									// If obstacle is starboard or not
								  data[i].H_TC_0[j],									// If obstacle is head on or not
								  data[i].X_TC_0[j],									// If crossing situation or not
								  data[i].O_TC_0[j],									// If ownship overtakes obstacle or not
								  data[i].Q_TC_0[j];									// If obstacle overtakes ownship or not
	}
}

/****************************************************************************************
*  Name     : display_obstacle_information
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Joint_Prediction_Manager::display_obstacle_information(
	const int i 														// In: Index of obstacle asking for information display
	) 			
{
	std::ios::fmtflags old_settings = std::cout.flags();
	int old_precision = std::cout.precision(); 

	//std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout << std::fixed << std::setprecision(0);

	std::cout << "        Obstacle information:" << std::endl;
	//std::cout << "ID   SOG   COG   R-BRG   RNG   HL   IP   AH   SB   HO   CRG   OTG   OT" << std::endl;

	for (int j = 0; j < data[i].obstacle_status.rows(); j++)
	{
		std::cout << std::setw(10) << status_str[j];
	}
	std::cout << "\n";

	for (size_t j = 0; j < data[i].obstacles.size(); j++)
	{
		for (int k = 0; k < data[i].obstacle_status.rows(); k++)
		{
			std::cout << std::setw(10)  << data[i].obstacle_status(k, j);
		}
		std::cout << std::endl;
	}
	std::cout.flags(old_settings);
	std::cout << std::setprecision(old_precision);
}


/****************************************************************************************
	Private functions
****************************************************************************************/