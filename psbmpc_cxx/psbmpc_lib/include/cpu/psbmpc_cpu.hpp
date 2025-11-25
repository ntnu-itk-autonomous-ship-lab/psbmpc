#pragma once

#include "obstacle_predictor.hpp"
#include "psbmpc_defines.hpp"
#include "psbmpc_parameters.hpp"
#if OWNSHIP_TYPE == 0
#include "cpu/kinematic_ship_models_cpu.hpp"
#else
#include "cpu/kinetic_ship_models_cpu.hpp"
#endif
#include "cpu/cpe_cpu.hpp"
#include "cpu/mpc_cost_cpu.hpp"

#include <memory>
#include <vector>

namespace PSBMPC_LIB {
namespace CPU {
// Pybind11 compatability struct
// Struct used to return u_opt, chi_opt and predicted_trajectory to Python
// Used as the return type of the calculate_optimal_offsets_py method
struct optimal_offsets_results_py {
  double u_opt_py;
  double chi_opt_py;
  Eigen::MatrixXd predicted_trajectory_py;
};

class PSBMPC {
private:
  // Amount of prediction scenarios for each obstacle
  std::vector<int> n_ps;

  // Control behavior related vectors and the own-ship maneuver times in the
  // prediction horizon
  Eigen::VectorXd offset_sequence, maneuver_times;
  Eigen::VectorXi offset_sequence_counter;

  // Previous optimal offsets/modifications
  double u_opt_last;
  double chi_opt_last;

  // Cost at the optimal solution
  double min_cost;

  // Own-ship predicted trajectory
  Eigen::MatrixXd trajectory;

  // Pybind11 compatability shared_ptr
  // Shared_ptr used to make the Python KinematicShip object reflect the C++
  // Kinematic_Ship object (and vice versa)
  std::shared_ptr<CPU::Kinematic_Ship> ownship_ptr;

  Ownship ownship;

  CPE cpe;

  bool determine_colav_active(const Dynamic_Obstacles &obstacles,
                              const int n_so, const bool disable);

  void reset_control_behaviour();

  void increment_control_behaviour();

  void setup_prediction(const Dynamic_Obstacles &obstacles);

  void setup_prediction_ptr(const Dynamic_Obstacles &obstacles);

  void calculate_collision_probabilities(Eigen::MatrixXd &P_c_i,
                                         const Dynamic_Obstacles &obstacles,
                                         const int i, const double dt,
                                         const int p_step);

  void calculate_collision_probabilities_ptr(Eigen::MatrixXd &P_c_i,
                                             const Dynamic_Obstacles &obstacles,
                                             const int i, const double dt,
                                             const int p_step);

  void assign_optimal_trajectory(Eigen::MatrixXd &optimal_trajectory);

public:
  PSBMPC_Parameters pars;

  MPC_Cost<PSBMPC_Parameters> mpc_cost;

  PSBMPC();

  PSBMPC(const Ownship &ownship, const CPE &cpe,
         const PSBMPC_Parameters &psbmpc_pars);

  // Pybind11 compatibility overload with shared_ptr
  PSBMPC(const std::shared_ptr<CPU::Kinematic_Ship> &ownship_ptr,
         const CPE &cpe, const PSBMPC_Parameters &psbmpc_pars);

  // Resets previous optimal offsets and predicted own-ship waypoint following
  void reset() {
    u_opt_last = 1.0;
    chi_opt_last = 0.0;
    ownship.set_wp_counter(0);
  }

  // Pybind11 compatability overload which works with shard_ptrs
  void reset_py() {
    u_opt_last = 1.0;
    chi_opt_last = 0.0;
    ownship_ptr->set_wp_counter(0);
  }

  void calculate_optimal_offsets(
      double &u_opt, double &chi_opt, Eigen::MatrixXd &predicted_trajectory,
      const double u_d, const double chi_d,
      const Eigen::Matrix<double, 2, -1> &waypoints,
      const Eigen::VectorXd &ownship_state, const double V_w,
      const Eigen::Vector2d &wind_direction, const Static_Obstacles &polygons,
      const Dynamic_Obstacles &obstacles, const bool disable);

  // Pybind11 compatability overload to return optimal_offsets_results_py
  optimal_offsets_results_py calculate_optimal_offsets_py(
      const double u_d, const double chi_d,
      const Eigen::Matrix<double, 2, -1> &waypoints,
      const Eigen::VectorXd &ownship_state, const double V_w,
      const Eigen::Vector2d &wind_direction,
      const std::vector<Eigen::MatrixXd> &polygons_py,
      const Dynamic_Obstacles &obstacles, const bool new_static_obstacle_data,
      const bool disable);

  // Pybind11 compatability getter to return PSBMPC_Parameters
  const PSBMPC_Parameters get_PSBMPC_Parameters() const {
    PSBMPC_Parameters copyOfPars;
    copyOfPars = pars;

    return copyOfPars;
  }

  // Pybind11/colav simulator compatability method
  Static_Obstacles
  process_list_of_np_polygons(const std::vector<Eigen::MatrixXd> &polygons_py);
};
} // namespace CPU
} // namespace PSBMPC_LIB