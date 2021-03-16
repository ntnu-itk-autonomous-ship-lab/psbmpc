/****************************************************************************************
*
*  File name : obstacle_sbmpc_parameters.cu
*
*  Function  : Class function file for the Obstacle SB-MPC parameter class
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

#include "obstacle_sbmpc_parameters.cuh"

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
void Obstacle_SBMPC_Parameters::initialize_pars()
{
	n_cbs = 1;
	n_M = 1;

	chi_offsets.resize(n_M, 13);
	u_offsets.resize(n_M, 2);
	offsets_size.resize(2, n_M);
	for (int M = 0; M < n_M; M++)
	{
		if (M == 0)
		{
			u_offsets(M, 0) = 1.0f; u_offsets(M, 1) = 0.5f; //u_offsets(M, 2) = 0.0f;
			offsets_size(0, M) = 2; 

			chi_offsets(M, 0) = -90.0f * DEG2RAD; 	chi_offsets(M, 1) = -75.0f * DEG2RAD; 	chi_offsets(M, 2) = -60.0f * DEG2RAD; 
			chi_offsets(M, 3) = -45.0f * DEG2RAD;	chi_offsets(M, 4) = -30.0f * DEG2RAD; 	chi_offsets(M, 5) = -15.0f * DEG2RAD; 
			chi_offsets(M, 6) = 0.0f * DEG2RAD;		chi_offsets(M, 7) = 15.0f * DEG2RAD; 	chi_offsets(M, 8) = 30.0f * DEG2RAD; 
			chi_offsets(M, 9) = 45.0f * DEG2RAD; 	chi_offsets(M, 10) = 60.0f * DEG2RAD; 	chi_offsets(M, 11) = 75.0f * DEG2RAD; 
			chi_offsets(M, 12) = 90.0f * DEG2RAD; 
			
			offsets_size(1, M) = 13;
		} 
		else
		{
			u_offsets(M, 0) = 1.0f; u_offsets(M, 1) = 0.5f; //u_offsets(M, 2) = 0.0f;
			offsets_size(0, M) = 2; 

			chi_offsets(M, 0) = -45.0f * DEG2RAD; 	chi_offsets(M, 1) = -30.0f * DEG2RAD; 	chi_offsets(M, 2) = -15.0f * DEG2RAD; 
			chi_offsets(M, 3) = 0.0f * DEG2RAD;		chi_offsets(M, 4) = 15.0f * DEG2RAD; 	chi_offsets(M, 5) = 30.0f * DEG2RAD; 
			chi_offsets(M, 6) = 45.0f * DEG2RAD;

			offsets_size(1, M) = 7;
		}
		n_cbs *= offsets_size(0, M) * offsets_size(1, M);
	}

	prediction_method = ERK1;
	guidance_method = LOS;

	T = 150.0;
	dt = 5.0;
	T_static = 60.0;

	p_step = 1;
	if (prediction_method == ERK1)
	{ 
		dt = 1; 
		p_step = 10;
	}
	t_ts = 50;

	d_init = 1500;								 
	d_close = 1000;
	d_safe = 50; 							
	K_coll = 1.0;		  					
	phi_AH = 68.5 * DEG2RAD;		 	
	phi_OT = 68.5 * DEG2RAD;		 		 
	phi_HO = 22.5 * DEG2RAD;		 		
	phi_CR = 68.5 * DEG2RAD;	     		
	kappa = 3.0;		  					
	kappa_TC = 10.0;						 
	K_u = 10;		   						 
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