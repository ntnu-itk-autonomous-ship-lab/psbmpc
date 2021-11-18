/****************************************************************************************
*
*  File name : sbmpc_index.hpp
*
*  Function  : Index file for Scenario-based Model Predictive Control
*
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#define N_IPAR_SBMPC						4 							// Number of integer type parameters in PSBMPC
#define N_DPAR_SBMPC						29 							// Number of double type parameters in PSBMPC
#define N_OPAR_SBMPC						2							// Number of offset/control behavior related parameters in PSBMPC

// Indices for parameters of type int
#define i_ipar_n_M_SBMPC					0                          
#define i_ipar_n_r_SBMPC                    1       
#define i_ipar_p_step_SBMPC 				2       
#define i_ipar_p_step_grounding_SBMPC		3             

// Indices for parameters of type double
#define i_dpar_T_SBMPC 					    0
#define i_dpar_dt_SBMPC 					1

#define i_dpar_t_ts_SBMPC					2

#define i_dpar_d_safe_SBMPC				    3
#define i_dpar_d_close_SBMPC				4
#define i_dpar_d_init_SBMPC				    5
#define i_dpar_d_so_relevant_SBMPC    	    6

#define i_dpar_K_coll_SBMPC				    7

#define i_dpar_phi_AH_SBMPC				    8
#define i_dpar_phi_OT_SBMPC				    9
#define i_dpar_phi_HO_SBMPC				    10
#define i_dpar_phi_CR_SBMPC				    11

#define i_dpar_kappa_SBMPC				    12
#define i_dpar_kappa_TC_SBMPC				13

#define i_dpar_K_u_SBMPC					14
#define i_dpar_K_du_SBMPC					15

#define i_dpar_K_chi_strb_SBMPC			    16
#define i_dpar_K_dchi_strb_SBMPC			17
#define i_dpar_K_chi_port_SBMPC			    18
#define i_dpar_K_dchi_port_SBMPC			19

#define i_dpar_K_sgn_SBMPC				    20
#define i_dpar_T_sgn_SBMPC				    21

#define i_dpar_q_SBMPC                      22
#define i_dpar_p_SBMPC                      23

#define i_dpar_G_1_SBMPC 					24
#define i_dpar_G_2_SBMPC 					25
#define i_dpar_G_3_SBMPC 					26
#define i_dpar_G_4_SBMPC 					27

#define i_dpar_epsilon_rdp_SBMPC            28

// Indices for offset/control behaviour parameters of type std::vector/Eigen::MatrixXd
#define i_opar_u_offsets_SBMPC			    0
#define i_opar_chi_offsets_SBMPC			1