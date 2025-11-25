#pragma once

#define N_BPAR 4  // Number of boolean tyoe parameters in PSBMPC
#define N_IPAR 4  // Number of integer type parameters in PSBMPC
#define N_DPAR 22 // Number of double type parameters in PSBMPC
#define N_OPAR                                                                 \
  2 // Number of offset/control behavior related parameters in PSBMPC

// Indices for parameters of type bool

// Indices for parameters of type int
#define i_ipar_n_M                                                             \
  0 // Number of sequential avoidance maneuvers considered in MPC
#define i_ipar_n_do_ps                                                         \
  1 // Max number of dynamic obstacle prediction scenarios in the MPC
#define i_ipar_p_step_do                                                       \
  2 // Step between state samples in dynamic obstacle cost eval, used for faster
    // cost eval.
#define i_ipar_p_step_grounding                                                \
  3 // Step between state samples in static obstacle cost eval, used for faster
    // cost eval.

// Indices for parameters of type double
#define i_dpar_T 0  // MPC Prediction horizon
#define i_dpar_dt 1 // MPC Prediction time step

#define i_dpar_t_ts 2 // Time spacing between own-ship trajectories

#define i_dpar_T_track_loss_threshold                                          \
  3 // Time threshold for considering a target ship to be lost

#define i_dpar_d_safe 4 // Own-ship safety zone radius
#define i_dpar_d_do_relevant                                                   \
  5 // Range for considering dynamic obstacles in the MPC
#define i_dpar_d_so_relevant                                                   \
  6 // Range for considering static ostacles in the MPC

#define i_dpar_K_coll 7 // Collision cost parameter
#define i_dpar_T_coll 8 // Time discounting factor for the chattering cost

#define i_dpar_kappa_SO 9  // COLREGS stand-on (SO) penalty parameter
#define i_dpar_kappa_GW 10 // COLREGS give-way (GW) penalty parameter
#define i_dpar_kappa_RA 11 // COLREGS readily apparent (RA) penalty parameter

#define i_dpar_K_u                                                             \
  12 // Speed change penalty parameter from the current speed reference
#define i_dpar_K_du                                                            \
  13 // Penalty parameter for speed change between current and previous speed
     // modification
#define i_dpar_K_chi 14 // Course modification penalty parameter
#define i_dpar_K_dchi                                                          \
  15 // Penalty parameter for course change between current and previous course
     // modification
#define i_dpar_K_e 16 // Penalty parameter for large predicted cross track error

#define i_dpar_G_1 17 // Grounding cost penalty parameter
#define i_dpar_G_2                                                             \
  18 // Grounding cost penalty parameter connected to non-zero wind speed
#define i_dpar_G_3 19 // Grounding cost distance discount parameter
#define i_dpar_G_4 20 // Grounding cost time discount parameter

#define i_dpar_epsilon_rdp                                                     \
  21 // Ramer-Douglas-Peucker distance threshold parameter

// Indices for offset/control behaviour parameters of type
// std::vector/Eigen::MatrixXd
#define i_opar_u_offsets 0
#define i_opar_chi_offsets 1

// Indices for parameters of type bool (used in the colav-simulator)
#define i_use_intention_model 0
#define i_use_path_pruning_ownship 1
#define i_use_path_pruning_targetship 2
#define i_use_GPU 3