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

#ifndef _JOINT_PREDICTION_MANAGER_H_
#define _JOINT_PREDICTION_MANAGER_H_

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

	

	/****************************************************************************************
	*  Name     : update_obstacles
	*  Function : Takes in new obstacle information and updates the single obstacle data 
	*			  structures for the "intelligent" obstacle i.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class Parameter_Object>
	__host__ __device__ void update_obstacles(
		Prediction_Obstacle *obstacles, 													// In/out: List of Prediction Obstacles
		const int i, 																		// In: Index of obstacle i to update data for				
		const Parameter_Object &mpc_pars,													// In: Parameters of the obstacle manager boss
		const Eigen::Matrix<double, 7, -1> &obstacle_states, 								// In: Other dynamic obstacle states 
		const int k																			// In: Index of the current predicted time t_k																
		) 			
	{
		int n_obst_old = data[i].obstacles.size();
		int n_obst_new = obstacle_states.cols();

		bool obstacle_exist;
		for (int j = 0; j < n_obst_new; j++)
		{
			obstacle_exist = false;
			for (int z = 0; z < n_obst_old; z++)
			{
				if ((double)data[i].obstacles[z].get_ID() == obstacle_states(6, j))
				{
					data[i].obstacles[z].update(obstacle_states.block<4, 1>(0, j), k);

					data[i].new_obstacles.push_back(std::move(data[i].obstacles[z]));

					obstacle_exist = true;

					break;
				}
			}
			if (!obstacle_exist)
			{
				data[i].new_obstacles.push_back(std::move(Prediction_Obstacle(
					obstacle_states.col(j), 
					false,
					mpc_pars.T, 
					mpc_pars.dt)));
			}
		}
		data[i].obstacles = std::move(data[i].new_obstacles);	
	}
	

public:

	__host__ __device__ Joint_Prediction_Manager(const int n_obst);

	__host__ __device__ ~Joint_Prediction_Manager();

	__host__ __device__ void update_obstacle_status(const int i, const TML::Vector4f &obstacle_i_state, const int k);

	__host__ __device__ void display_obstacle_information(const int i);

};

#endif 