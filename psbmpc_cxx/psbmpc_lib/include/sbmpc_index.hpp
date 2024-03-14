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

#define N_IPAR_SBMPC 4  // Number of integer type parameters in PSBMPC
#define N_DPAR_SBMPC 29 // Number of double type parameters in PSBMPC
#define N_OPAR_SBMPC 2  // Number of offset/control behavior related parameters in PSBMPC

// Indices for parameters of type int
#define i_ipar_n_M_SBMPC 0              // Number of sequential avoidance maneuvers considered in MPC
#define i_ipar_n_do_ps_SBMPC 1          // Max number of dynamic obstacle prediction scenarios in the MPC
#define i_ipar_p_step_opt_SBMPC 2       // Step between state samples in optimal trajectory, used to downsample output.
#define i_ipar_p_step_grounding_SBMPC 3 // Step between state samples in static obstacle cost eval, used for faster cost eval.

// Indices for parameters of type double
#define i_dpar_T_SBMPC 0  // MPC Prediction horizon
#define i_dpar_dt_SBMPC 1 // MPC Prediction time step

#define i_dpar_t_ts_SBMPC 2 // Time spacing between own-ship trajectories

#define i_dpar_d_safe_SBMPC 3        // Own-ship safety zone radius
#define i_dpar_d_close_SBMPC 4       // Range for considering COLREGS violation in the SBMPC
#define i_dpar_d_do_relevant_SBMPC 5 // Range for considering dynamic obstacles in the MPC
#define i_dpar_d_so_relevant_SBMPC 6 // Range for considering static ostacles in the MPC

#define i_dpar_K_coll_SBMPC 7 // Collision cost parameter

#define i_dpar_phi_AH_SBMPC 8  // COLREGS cost parameter for considering a ship to be ahead
#define i_dpar_phi_OT_SBMPC 9  // COLREGS cost parameter for considering a ship to be overtaking
#define i_dpar_phi_HO_SBMPC 10 // COLREGS cost parameter for considering obstacle to be head-on
#define i_dpar_phi_CR_SBMPC 11 // COLREGS cost parameter for considering obstacle to be crossing

#define i_dpar_kappa_SBMPC 12    // COLREGS cost penalty parameter
#define i_dpar_kappa_TC_SBMPC 13 // Transitional cost penalty parameter

#define i_dpar_K_u_SBMPC 14  // Speed change penalty parameter from the current speed reference
#define i_dpar_K_du_SBMPC 15 // Penalty parameter for speed change between current and previous speed modification

#define i_dpar_K_chi_strb_SBMPC 16  // Course modification penalty parameter to starboard
#define i_dpar_K_dchi_strb_SBMPC 17 // Penalty parameter for course change between current (starboard) and previous course modification
#define i_dpar_K_chi_port_SBMPC 18  // Course modification penalty parameter to port
#define i_dpar_K_dchi_port_SBMPC 19 // Penalty parameter for course change between current (port) and previous course modification

#define i_dpar_q_SBMPC 20 // Dynamic obstacle cost ad hoc risk penalty parameter
#define i_dpar_p_SBMPC 21 // Dynamic obstacle cost ad hoc risk penalty parameter

#define i_dpar_G_1_SBMPC 22 // Grounding cost penalty parameter
#define i_dpar_G_2_SBMPC 23 // Grounding cost penalty parameter connected to non-zero wind speed
#define i_dpar_G_3_SBMPC 24 // Grounding cost distance discount parameter
#define i_dpar_G_4_SBMPC 25 // Grounding cost time discount parameter

#define i_dpar_epsilon_rdp_SBMPC 26 // Ramer-Douglas-Peucker distance threshold parameter

// Indices for offset/control behaviour parameters of type std::vector/Eigen::MatrixXd
#define i_opar_u_offsets_SBMPC 0
#define i_opar_chi_offsets_SBMPC 1