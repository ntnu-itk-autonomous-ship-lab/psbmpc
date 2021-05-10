/****************************************************************************************
*
*  File name : sbmpc_parameters.cpp
*
*  Function  : Class function file for the SB-MPC parameter class
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

#include "psbmpc_defines.hpp"
#include "sbmpc_parameters.hpp"
#include "Eigen/Dense"
#include <vector>

namespace PSBMPC_LIB
{

/****************************************************************************************
	Public functions
****************************************************************************************/

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : initialize_pars
*  Function : Sets initial values for SBMPC tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void SBMPC_Parameters::initialize_pars(
	const bool is_obstacle_sbmpc							// In: Boolean flag to determine MPC tuning
	)
{
	if (!is_obstacle_sbmpc) // Tuning for SB-MPC
	{
		n_cbs = 1;
		n_M = 1;

		chi_offsets.resize(n_M);
		u_offsets.resize(n_M);
		for (int M = 0; M < n_M; M++)
		{
			if (M == 0)
			{
				u_offsets[M].resize(3);
				//u_offsets[M] << 1.0;
				u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(13);
				//chi_offsets[M] << 30.0;
				//chi_offsets[M] << -30.0, 0.0, 30.0;
				//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				chi_offsets[M] *= DEG2RAD;
			} 
			else
			{
				u_offsets[M].resize(2);
				//u_offsets[M] << 1.0;
				u_offsets[M] << 1.0, 0.5;
				//u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(7);
				//chi_offsets[M] << 0.0;
				//chi_offsets[M] << -30.0, 0.0, 30.0;
				//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
				//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
				//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				chi_offsets[M] *= DEG2RAD;
			}
			n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
		}

		prediction_method = ERK1;
		guidance_method = LOS;

		T = 110.0;
		dt = 5.0;
		T_static = 60.0;

		p_step = 1;
		if (prediction_method == ERK1)
		{ 
			dt = 0.5; 
			p_step = 10;
		}
		t_ts = 35;

		d_init = 1500;								 
		d_close = 1000;
		d_safe = 50; 							
		K_coll = 0.2;		  					
		phi_AH = 68.5 * DEG2RAD;		 	
		phi_OT = 68.5 * DEG2RAD;		 		 
		phi_HO = 22.5 * DEG2RAD;		 		
		phi_CR = 68.5 * DEG2RAD;	     		
		kappa = 10.0;		  					
		kappa_TC = 20.0;						 
		K_u = 15;		   						 
		K_du = 6;		    					
		K_chi_strb = 1.3;	  					
		K_chi_port =  1.6;	  					
		K_dchi_strb = 0.9;	 			
		K_dchi_port = 1.2;
		K_sgn = 8;
		T_sgn = 4 * t_ts;	  					
		G = 1e3;		         					 
		q = 4.0;
		p = 1.0;
	}
	else // Tuning for Obstacle SB-MPC
	{
		n_cbs = 1;
		n_M = 1;

		chi_offsets.resize(n_M);
		u_offsets.resize(n_M);
		for (int M = 0; M < n_M; M++)
		{
			if (M == 0)
			{
				u_offsets[M].resize(3);
				//u_offsets[M] << 1.0;
				u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(13);
				//chi_offsets[M] << 30.0;
				//chi_offsets[M] << -30.0, 0.0, 30.0;
				//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				chi_offsets[M] *= DEG2RAD;
			} 
			else
			{
				u_offsets[M].resize(2);
				//u_offsets[M] << 1.0;
				u_offsets[M] << 1.0, 0.5;
				//u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(7);
				//chi_offsets[M] << 0.0;
				//chi_offsets[M] << -30.0, 0.0, 30.0;
				//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
				//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
				//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				chi_offsets[M] *= DEG2RAD;
			}
			n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
		}

		prediction_method = ERK1;
		guidance_method = LOS;

		T = 120.0;
		dt = 5.0;
		T_static = 60.0;

		p_step = 1;
		if (prediction_method == ERK1)
		{ 
			dt = 0.5; 
			p_step = 10;
		}
		t_ts = 50;

		d_init = 1500;								 
		d_close = 1000;
		d_safe = 50; 							
		K_coll = 2.5;		  					
		phi_AH = 68.5 * DEG2RAD;		 	
		phi_OT = 68.5 * DEG2RAD;		 		 
		phi_HO = 22.5 * DEG2RAD;		 		
		phi_CR = 68.5 * DEG2RAD;	     		
		kappa = 8.0;		  					
		kappa_TC = 0.0;						 
		K_u = 4;		   						 
		K_du = 2.5;		    					
		K_chi_strb = 1.3;	  					
		K_chi_port =  1.6;	  					
		K_dchi_strb = 0.9;	 			
		K_dchi_port = 1.2;
		K_sgn = 5;
		T_sgn = 4 * t_ts;	  					
		G = 1e3;		         					 
		q = 4.0;
		p = 1.0;
	}
}
}