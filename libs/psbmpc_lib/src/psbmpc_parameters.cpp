/****************************************************************************************
*
*  File name : psbmpc.h
*
*  Function  : Header file for the PSB-MPC parameter struct.
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

#include "psbmpc_parameters.h"
#include "Eigen/Dense"
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0f
#endif
#ifndef RAD2DEG
#define RAD2DEG 180.0f / M_PI
#endif


/****************************************************************************************
	Public functions
****************************************************************************************/
/****************************************************************************************
*  Name     : get_<type>par
*  Function : Returns parameter with index <index>, "overloaded" for different data types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC_Parameters::get_bpar(
	const int index															// In: Index of parameter to return (Must be of int type)
	) const
{
	switch(index){
		case i_bpar_obstacle_colav_on 				: return obstacle_colav_on; 
		default : 
			// Throw
			return 0;
	}
}

int PSBMPC_Parameters::get_ipar(
	const int index															// In: Index of parameter to return (Must be of int type)
	) const
{
	switch(index){
		case i_ipar_n_M 				: return n_M; 
		default : 
			// Throw
			return 0;
	}
}
	
double PSBMPC_Parameters::get_dpar(
	const int index															// In: Index of parameter to return (Must be of double type)
	) const
{
	switch(index){
		case i_dpar_T 					: return T;
		case i_dpar_T_static 			: return T_static;
		case i_dpar_dt 					: return dt;
		case i_dpar_p_step				: return p_step;
		case i_dpar_t_ts 				: return t_ts;
		case i_dpar_d_safe 				: return d_safe;
		case i_dpar_d_close 			: return d_close;
		case i_dpar_K_coll 				: return K_coll;
		case i_dpar_phi_AH 				: return phi_AH;
		case i_dpar_phi_OT 				: return phi_OT;
		case i_dpar_phi_HO 				: return phi_HO;
		case i_dpar_phi_CR 				: return phi_CR;
		case i_dpar_kappa 				: return kappa;
		case i_dpar_kappa_TC 			: return kappa_TC;
		case i_dpar_K_u 				: return K_u;
		case i_dpar_K_du 				: return K_du;
		case i_dpar_K_chi_strb 			: return K_chi_strb;
		case i_dpar_K_dchi_strb 		: return K_dchi_strb;
		case i_dpar_K_chi_port 			: return K_chi_port;
		case i_dpar_K_dchi_port 		: return K_dchi_port;
		case i_dpar_K_sgn 				: return K_sgn;
		case i_dpar_T_sgn 				: return T_sgn;
		case i_dpar_G					: return G;
		case i_dpar_q					: return q;
		case i_dpar_p					: return p;
		default : 
			// Throw
			return 0.0;
	}
}

std::vector<Eigen::VectorXd> PSBMPC_Parameters::get_opar(
	const int index															// In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
{
	switch (index){
		case i_opar_u_offsets			: return u_offsets;
		case i_opar_chi_offsets 		: return chi_offsets;
		default : 
		{ 
			// Throw
			std::vector<Eigen::VectorXd> bs;
			return bs; 
		}
	}
}

Eigen::VectorXd PSBMPC_Parameters::get_evpar(
	const int index															// In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
{
	switch (index){
		case i_evpar_obstacle_course_changes			: return obstacle_course_changes;
		default : 
		{ 
			// Throw invalid index
			Eigen::VectorXd bs;
			return bs; 
		}
	}
}

/****************************************************************************************
*  Name     : set_par
*  Function : Sets parameter with index <index> to value <value>, given that it is inside
*			  valid limits. Overloaded for different data types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC_Parameters::set_par(
	const int index, 														// In: Index of parameter to set
	const bool value 														// In: Value to set for parameter
	)
{	
	switch(index)
	{
		case i_bpar_obstacle_colav_on 			: obstacle_colav_on = value; break;
		default : 
			// Throw invalid index
			break;
	}	
}

void PSBMPC_Parameters::set_par(
	const int index, 														// In: Index of parameter to set
	const int value 														// In: Value to set for parameter
	)
{
	if (value >= ipar_low[index] && value <= ipar_high[index])
	{	
		switch(index)
		{
			case i_ipar_n_M 				: n_M = value; break;
			default : 
				// Throw
				break;
		}
	}
	else
	{
		// Throw invalid par value
	}
}

void PSBMPC_Parameters::set_par(
	const int index, 														// In: Index of parameter to set
	const double value 														// In: Value to set for parameter
	)
{
	if (value >= dpar_low[index] && value <= dpar_high[index])
	{
		switch(index){
			case i_dpar_T 					: T = value; break;
			case i_dpar_T_static 			: T_static = value; break;
			case i_dpar_dt 					: dt = value; break;
			case i_dpar_p_step 				: p_step = value; break;
			case i_dpar_t_ts 				: t_ts = value; break;
			case i_dpar_d_safe :
				// Limits on d_close and d_init depend on d_safe
				d_safe = value; 
				dpar_low[i_dpar_d_close] = d_safe;
				dpar_low[i_dpar_d_init] = d_safe;
				break;
			case i_dpar_d_close 			: d_close = value; break;
			case i_dpar_d_init 				: d_init = value; break;
			case i_dpar_K_coll 				: K_coll = value; break;
			case i_dpar_phi_AH 				: phi_AH = value; break;
			case i_dpar_phi_OT 				: phi_OT = value; break;
			case i_dpar_phi_HO 				: phi_HO = value; break;
			case i_dpar_phi_CR 				: phi_CR = value; break;
			case i_dpar_kappa 				: kappa = value; break;
			case i_dpar_kappa_TC 			: kappa_TC = value; break;
			case i_dpar_K_u 				: K_u = value; break;
			case i_dpar_K_du 				: K_du = value; break;
			case i_dpar_K_chi_strb 			: K_chi_strb = value; break;
			case i_dpar_K_dchi_strb 		: K_dchi_strb = value; break;
			case i_dpar_K_chi_port 			: K_chi_port = value; break;
			case i_dpar_K_dchi_port 		: K_dchi_port = value; break;
			case i_dpar_K_sgn 				: K_sgn = value; break;
			case i_dpar_T_sgn 				: T_sgn = value; break;
			case i_dpar_G 					: G = value; break;
			case i_dpar_q 					: q = value; break;
			case i_dpar_p 					: p = value; break;
			default : // Throw invalid index
				break;
		}
	}
	else
	{
		// Throw invalid par value
	}
	
}

void PSBMPC_Parameters::set_par(
	const int index,														// In: Index of parameter to set
	const std::vector<Eigen::VectorXd> &value 								// In: Value to set for parameter
	)
{
	if ((int)value.size() == n_M)
	{
		switch (index){
			case i_opar_u_offsets : 
				for (int j = 0; j < n_M; j++){
					if (value[j].size() > 0)
					{
						u_offsets[j] = value[j];
					}
				}
				break;
			case i_opar_chi_offsets : 
				for (int j = 0; j < n_M; j++)
				{
					if (value[j].size() > 0)
					{
						chi_offsets[j] = value[j];
					}
				}
				break;
			default : 
				// Throw invalid index
				break; 
		}
	}
	else
	{
		// Throw invalid value
	}
}

void PSBMPC_Parameters::set_par(
	const int index, 														// In: Index of parameter to set
	const Eigen::VectorXd &value 											// In: Value to set for parameter
	)
{
	if (value.size() > 0)
	{	
		switch(index)
		{
			case i_evpar_obstacle_course_changes 		: obstacle_course_changes = value; break;
			default : 
				// Throw invalid index
				break;
		}
	}
	else
	{
		// Throw invalid par value
	}
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : initialize_par_limits
*  Function : Sets initial low and high limits on tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC_Parameters::initialize_par_limits()
{
	ipar_low.resize(N_IPAR); ipar_high.resize(N_IPAR);
	for (int i = 0; i < N_IPAR; i++)
	{
		ipar_low[i] = 0.0;
		ipar_high[i] = 1e12;
	}
	ipar_low[i_ipar_n_M] = 1; ipar_high[i_ipar_n_M] = 5; 

	//std::cout << "i_par_low = " << ipar_low.transpose() << std::endl;
	//std::cout << "i_par_high = " << ipar_high.transpose() << std::endl;

	dpar_low.resize(N_DPAR); dpar_high.resize(N_DPAR);
	for (int i = 0; i < N_DPAR; i++)
	{
		dpar_low[i] = 0.0;
		dpar_high[i] = 1e12;
	}
	dpar_low[i_dpar_T] = 60.0;
	dpar_low[i_dpar_T_static] = 10.0;
	dpar_low[i_dpar_dt] = 0.001;
	dpar_low[i_dpar_p_step] = 0.001;

	dpar_low[i_dpar_d_safe] = 20.0;
	dpar_low[i_dpar_d_close] = d_safe; 			
	dpar_low[i_dpar_d_init] = d_safe; 			

	dpar_high[i_dpar_K_dchi_strb] = 3.0;
	dpar_high[i_dpar_K_dchi_port] = 3.0;

	dpar_low[i_dpar_phi_AH] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_AH] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_OT] = -180.0 * DEG2RAD;			dpar_high[i_dpar_phi_OT] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_HO] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_HO] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_CR] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_CR] = 180.0 * DEG2RAD;

	//std::cout << "d_par_low = " << dpar_low.transpose() << std::endl;
	//std::cout << "d_par_high = " << dpar_high.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : initialize_pars
*  Function : Sets initial values for PSBMPC tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC_Parameters::initialize_pars()
{
	n_cbs = 1;
	n_M = 2;

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
			//chi_offsets[M] << 0.0;
			//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		} 
		else
		{
			u_offsets[M].resize(2);
			u_offsets[M] << 1.0, 0.5;

			chi_offsets[M].resize(7);
			//chi_offsets[M] << 0.0;
			//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
			chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		}
		n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
	}

	obstacle_course_changes.resize(1);
	obstacle_course_changes << 30 * DEG2RAD; //60 * DEG2RAD, 90 * DEG2RAD;

	cpe_method = CE;
	prediction_method = ERK1;
	guidance_method = LOS;

	T = 100.0; 	      // 400.0, 300.0, 240 (sim/Euler)
	dt = 5.0;		      // 5.0, 0.5 (sim/Euler)
  	T_static = 60.0;		  // (50.0)

	p_step = 1;
	if (prediction_method == ERK1)
	{ 
		dt = 0.5; 
		p_step = 10;
	}
	t_ts = 50;

	d_init = 1500;								 
	d_close = 500;
	d_safe = 50; 							
	K_coll = 1.0;		  					
	phi_AH = 68.5 * DEG2RAD;		 	
	phi_OT = 68.5 * DEG2RAD;		 		 
	phi_HO = 22.5 * DEG2RAD;		 		
	phi_CR = 68.5 * DEG2RAD;	     		
	kappa = 3.0;		  					
	kappa_TC = 100.0;						 
	K_u = 3;		   						 
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

	obstacle_colav_on = false;
}