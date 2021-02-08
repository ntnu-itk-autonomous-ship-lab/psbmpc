/****************************************************************************************
*
*  File name : obstacle_manager.cu
*
*  Function  : Class functions for the obstacle management interface, modified with
*			   .cu for this GPU-implementation.
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

#include "utilities.cuh"
#include "obstacle_manager.cuh"
#include <string>
#include <iostream>
#include <iomanip>

/****************************************************************************************
*  Name     : Class constructor
*  Function : 
*  Author   :
*  Modified :
*****************************************************************************************/
Obstacle_Manager::Obstacle_Manager()
{	
	T_lost_limit = 15.0; 	// 15.0 s obstacle no longer relevant after this time
	T_tracked_limit = 15.0; // 15.0 s obstacle still relevant if tracked for so long, choice depends on survival rate

	obstacle_filter_on = false;

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
}

/****************************************************************************************
*  Name     : update_obstacle_status
*  Function : Updates various information on each obstacle at the current time
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::update_obstacle_status(
	const Eigen::Matrix<double, 6, 1> &ownship_state						// In: Current time own-ship state
	)
{
	int n_obst = data.obstacles.size();
	data.obstacle_status.resize(13, n_obst);
	double ID_0, RB_0, COG_0, SOG_0; 
	Eigen::Vector2d d_0i;
	Eigen::Vector4d xs_i;

	data.HL_0.resize(n_obst); data.HL_0.setZero();
	for(int i = 0; i < n_obst; i++)
	{
		xs_i = data.obstacles[i].kf.get_state();

		ID_0 = data.obstacles[i].get_ID();
		
		d_0i = (xs_i.block<2, 1>(0, 0) - ownship_state.block<2, 1>(0, 0));

		COG_0 = atan2(xs_i(3), xs_i(2));

		SOG_0 = xs_i.block<2, 1>(2, 0).norm();

		RB_0 = angle_difference_pmpi(atan2(d_0i(1), d_0i(0)), ownship_state(2));

		data.obstacle_status.col(i) << ID_0, 											// Obstacle ID
								  SOG_0, 												// Speed over ground of obstacle
								  wrap_angle_to_02pi(COG_0) * RAD2DEG, 					// Course over ground of obstacle
								  RB_0 * RAD2DEG, 										// Relative bearing
								  d_0i.norm(),											// Range
								  data.HL_0[i], 										// Hazard level of obstacle at optimum
								  data.IP_0[i], 										// If obstacle is passed by or not 
								  data.AH_0[i], 										// If obstacle is ahead or not
								  data.S_TC_0[i], 										// If obstacle is starboard or not
								  data.H_TC_0[i],										// If obstacle is head on or not
								  data.X_TC_0[i],										// If crossing situation or not
								  data.O_TC_0[i],										// If ownship overtakes obstacle or not
								  data.Q_TC_0[i];										// If obstacle overtakes ownship or not
	}
}

/****************************************************************************************
*  Name     : display_obstacle_information
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::display_obstacle_information() 			
{
	std::ios::fmtflags old_settings = std::cout.flags();
	int old_precision = std::cout.precision(); 

	//std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout << std::fixed << std::setprecision(0);

	std::cout << "        Obstacle information:" << std::endl;
	//std::cout << "ID   SOG   COG   R-BRG   RNG   HL   IP   AH   SB   HO   CRG   OTG   OT" << std::endl;

	for (int j = 0; j < data.obstacle_status.rows(); j++)
	{
		std::cout << std::setw(10) << status_str[j];
	}
	std::cout << "\n";

	for (size_t j = 0; j < data.obstacles.size(); j++)
	{
		for (int k = 0; k < data.obstacle_status.rows(); k++)
		{
			std::cout << std::setw(10)  << data.obstacle_status(k, j);
		}
		std::cout << std::endl;
	}
	std::cout.flags(old_settings);
	std::cout << std::setprecision(old_precision);
}


/****************************************************************************************
	Private functions
****************************************************************************************/