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

#define N_BPAR 0  // Number of boolean tyoe parameters in PSBMPC
#define N_IPAR 5  // Number of integer type parameters in PSBMPC
#define N_DPAR 20 // Number of double type parameters in PSBMPC
#define N_OPAR 2  // Number of offset/control behavior related parameters in PSBMPC

// Indices for parameters of type bool

// Indices for parameters of type int
#define i_ipar_n_M 0              // Number of sequential avoidance maneuvers considered in MPC
#define i_ipar_n_do_ps 1          // Max number of dynamic obstacle prediction scenarios in the MPC
#define i_ipar_p_step_opt 2       // Step between state samples in optimal trajectory, used to downsample output.
#define i_ipar_p_step_do 3        // Step between state samples in dynamic obstacle cost eval, used for faster cost eval.
#define i_ipar_p_step_grounding 4 // Step between state samples in static obstacle cost eval, used for faster cost eval.

// Indices for parameters of type double
#define i_dpar_T 0  // MPC Prediction horizon
#define i_dpar_dt 1 // MPC Prediction time step

#define i_dpar_t_ts 2 // Time spacing between own-ship trajectories

#define i_dpar_d_safe 3        // Own-ship safety zone radius
#define i_dpar_d_do_relevant 4 // Range for considering dynamic obstacles in the MPC
#define i_dpar_d_so_relevant 5 // Range for considering static ostacles in the MPC

#define i_dpar_K_coll 6 // Collision cost parameter
#define i_dpar_T_coll 7 // Time discounting factor for the chattering cost

#define i_dpar_kappa_SO 8 // COLREGS stand-on penalty parameter
#define i_dpar_kappa_GW 9 // COLREGS giwe-way penalty parameter

#define i_dpar_K_u 10    // Speed change penalty parameter from the current speed reference
#define i_dpar_K_du 11   // Penalty parameter for speed change between current and previous speed modification
#define i_dpar_K_chi 12  // Course modification penalty parameter
#define i_dpar_K_dchi 13 // Penalty parameter for course change between current and previous course modification
#define i_dpar_K_e 14    // Penalty parameter for large predicted cross track error

#define i_dpar_G_1 15 // Grounding cost penalty parameter
#define i_dpar_G_2 16 // Grounding cost penalty parameter connected to non-zero wind speed
#define i_dpar_G_3 17 // Grounding cost distance discount parameter
#define i_dpar_G_4 18 // Grounding cost time discount parameter

#define i_dpar_epsilon_rdp 19 // Ramer-Douglas-Peucker distance threshold parameter

// Indices for offset/control behaviour parameters of type std::vector/Eigen::MatrixXd
#define i_opar_u_offsets 0
#define i_opar_chi_offsets 1