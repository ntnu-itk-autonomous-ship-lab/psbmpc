/****************************************************************************************
*
*  File name : psbmpc_index.hpp
*
*  Function  : Index file for Probabilistic Scneario-based Model Predictive Control
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

#pragma once

#define N_BPAR						0							// Number of boolean tyoe parameters in PSBMPC
#define N_IPAR						5 							// Number of integer type parameters in PSBMPC
#define N_DPAR						22 							// Number of double type parameters in PSBMPC
#define N_OPAR						2							// Number of offset/control behavior related parameters in PSBMPC

// Indices for parameters of type bool

// Indices for parameters of type int
#define i_ipar_n_M					0                          
#define i_ipar_n_r                  1       
#define i_ipar_p_step 				2       
#define i_ipar_p_step_cpe 			3
#define i_ipar_p_step_grounding		4             

// Indices for parameters of type double
#define i_dpar_T 					0
#define i_dpar_dt 					1

#define i_dpar_t_ts					2

#define i_dpar_d_safe				3
#define i_dpar_d_init				4
#define i_dpar_d_so_relevant    	5

#define i_dpar_K_coll				6

#define i_dpar_kappa_SO				7
#define i_dpar_kappa_GW				8

#define i_dpar_K_u					9
#define i_dpar_K_du					10

#define i_dpar_K_chi_strb			11
#define i_dpar_K_dchi_strb			12
#define i_dpar_K_chi_port			13
#define i_dpar_K_dchi_port			14

#define i_dpar_K_sgn				15
#define i_dpar_T_sgn				16

#define i_dpar_G_1 					17
#define i_dpar_G_2 					18
#define i_dpar_G_3 					19
#define i_dpar_G_4 					20

#define i_dpar_epsilon_rdp          21

// Indices for offset/control behaviour parameters of type std::vector/Eigen::MatrixXd
#define i_opar_u_offsets			0
#define i_opar_chi_offsets			1