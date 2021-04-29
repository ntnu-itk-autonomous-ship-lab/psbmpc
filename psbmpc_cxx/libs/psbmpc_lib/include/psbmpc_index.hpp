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

#define N_BPAR						1							// Number of boolean tyoe parameters in PSBMPC
#define N_IPAR						2 							// Number of integer type parameters in PSBMPC
#define N_DPAR						25 							// Number of double type parameters in PSBMPC
#define N_OPAR						2							// Number of offset/control behavior related parameters in PSBMPC
#define N_EVPAR						1							// Number of Eigen::Vector type parameters in PSBMPC

// Indices for parameters of type bool
#define i_bpar_obstacle_colav_on	0

// Indices for parameters of type int
#define i_ipar_n_M					0                          
#define i_ipar_n_r                  1                           

// Indices for parameters of type double
#define i_dpar_T 					0
#define i_dpar_dt 					1
#define i_dpar_p_step 				2

#define i_dpar_t_ts					3

#define i_dpar_d_safe				4
#define i_dpar_d_close				5
#define i_dpar_d_init				6

#define i_dpar_K_coll				7

#define i_dpar_phi_AH				8
#define i_dpar_phi_OT				9
#define i_dpar_phi_HO				10
#define i_dpar_phi_CR				11

#define i_dpar_kappa				12
#define i_dpar_kappa_TC				13

#define i_dpar_K_u					14
#define i_dpar_K_du					15

#define i_dpar_K_chi_strb			16
#define i_dpar_K_dchi_strb			17
#define i_dpar_K_chi_port			18
#define i_dpar_K_dchi_port			19

#define i_dpar_K_sgn				20
#define i_dpar_T_sgn				21

#define i_dpar_G 					22

#define i_dpar_q                    23
#define i_dpar_p                    24

// Indices for offset/control behaviour parameters of type std::vector/Eigen::MatrixXd
#define i_opar_u_offsets			0
#define i_opar_chi_offsets			1

// Indices for offset/control behaviour parameters of type Eigen::VectorXd
#define i_evpar_obstacle_course_changes 0