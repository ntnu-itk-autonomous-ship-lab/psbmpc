#pragma once

#include "cpu/mpc_cost_cpu.hpp"
#include "sbmpc_parameters.hpp"
#if OWNSHIP_TYPE == 0
#include "cpu/kinematic_ship_models_cpu.hpp"
#else
#include "cpu/kinetic_ship_models_cpu.hpp"
#endif

namespace PSBMPC_LIB {

struct optimal_offsets_results_SBMPC_py {
  double u_opt_py;
  double chi_opt_py;
  Eigen::MatrixXd predicted_trajectory_py;
};

class SBMPC {
private:
  // Control behavior related vectors and the own-ship maneuver times in the
  // prediction horizon
  Eigen::VectorXd offset_sequence, maneuver_times;
  Eigen::VectorXi offset_sequence_counter;

  // Previous optimal offsets/modifications
  double u_opt_last;
  double chi_opt_last;

  double min_cost;

  Eigen::MatrixXd trajectory;

  // Pybind11 compatability shared_ptr
  // Shared_ptr used to make the Python KinematicShip object reflect the C++
  // Kinematic_Ship object (and vice versa)
  std::shared_ptr<CPU::Kinematic_Ship> ownship_ptr;

  CPU::Ownship ownship;

  Transitional_Variables tv;

  void reset_control_behaviour();

  void increment_control_behaviour();

  void update_transitional_variables(const Eigen::VectorXd &ownship_state,
                                     const Dynamic_Obstacles &obstacles);

  void
  update_transitional_variables_ptr_ver(const Eigen::VectorXd &ownship_state,
                                        const Dynamic_Obstacles &obstacles);

  void setup_prediction(const Dynamic_Obstacles &obstacles);

  void setup_prediction_ptr_ver(const Dynamic_Obstacles &obstacles);

  bool determine_colav_active(const Dynamic_Obstacles &obstacles,
                              const int n_static_obst, const bool disable);

  void assign_optimal_trajectory(Eigen::MatrixXd &optimal_trajectory);

public:
  SBMPC_Parameters pars;

  CPU::MPC_Cost<SBMPC_Parameters> mpc_cost;

  SBMPC();

  SBMPC(const CPU::Ownship &ownship, const SBMPC_Parameters &pars);

  // Pybind11 compatibility overload with shared_ptr
  SBMPC(const std::shared_ptr<CPU::Kinematic_Ship> &ownship_ptr,
        const SBMPC_Parameters &pars);

  void calculate_optimal_offsets(
      double &u_opt, double &chi_opt, Eigen::MatrixXd &predicted_trajectory,
      const double u_d, const double chi_d,
      const Eigen::Matrix<double, 2, -1> &waypoints,
      const Eigen::VectorXd &ownship_state,
      const Eigen::Matrix<double, 4, -1> &static_obstacles,
      const Dynamic_Obstacles &obstacles, const bool disable);

  void calculate_optimal_offsets(
      double &u_opt, double &chi_opt, Eigen::MatrixXd &predicted_trajectory,
      const double u_d, const double chi_d,
      const Eigen::Matrix<double, 2, -1> &waypoints,
      const Eigen::VectorXd &ownship_state, const double V_w,
      const Eigen::Vector2d &wind_direction, const Static_Obstacles &polygons,
      const Dynamic_Obstacles &obstacles, const bool disable);

  // Pybind11 compatability overload to return optimal_offsets_results_SBMPC_py
  optimal_offsets_results_SBMPC_py calculate_optimal_offsets_py(
      const double u_d, const double chi_d,
      const Eigen::Matrix<double, 2, -1> &waypoints,
      const Eigen::VectorXd &ownship_state, const double V_w,
      const Eigen::Vector2d &wind_direction,
      const std::vector<Eigen::MatrixXd> &polygons_py,
      const Dynamic_Obstacles &obstacles, const bool new_static_obstacle_data,
      const bool disable);

  // Pybind11/colav simulator compatability method
  Static_Obstacles
  process_list_of_np_polygons(const std::vector<Eigen::MatrixXd> &polygons_py);

  // Pybind11/colav simulator convenience method
  inline const SBMPC_Parameters get_SBMPC_Parameters() const { return pars; }
};
} // namespace PSBMPC_LIB