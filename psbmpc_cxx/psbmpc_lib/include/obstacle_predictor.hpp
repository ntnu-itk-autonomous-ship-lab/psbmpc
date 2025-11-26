#pragma once

#include "cpu/kinematic_ship_models_cpu.hpp"
#include "cpu/utilities_cpu.hpp"
#include "mrou.hpp"
#include "obstacle_manager.hpp"
#include "psbmpc_parameters.hpp"
#include "sbmpc_parameters.hpp"

#include <memory>
#include <vector>

namespace PSBMPC_LIB {
class Obstacle_Predictor {
private:
  // Parameters to determine the number of prediction scenarios for the two
  // prediction methods
  int n_ps_MROU, n_ps_LOS;

  // Max 3sigma cross-track standard deviation for the prediction scenario
  // uncertainty
  double r_ct;

  // Cross track offsets in LOS-prediction for obstacles (used in the SMOOTH
  // prediction)
  Eigen::VectorXd ct_offsets;

  // Course offsets in LOS-prediction for obstacles (used in the LINEAR
  // prediction)
  Eigen::VectorXd chi_offsets;

  // Preset course offsets in LOS-prediction for obstacles (used in the LINEAR
  // prediction, is set by chi_offsets is set by reading chi_offsets_params)
  Eigen::VectorXd chi_offsets_params;

  // Possible course_changes for an obstacle in the MROU prediction
  Eigen::VectorXd course_changes;

  // Actual number of prediction scenarios for each obstacle
  std::vector<int> n_ps;

  // Matrices of prediction scenario course changes and maneuver times for an
  // obstacle i, size n_cc x n_ps
  Eigen::MatrixXd ps_course_changes_i, ps_maneuver_times_i;

  // Prediction scenario trajectory vector and mean velocity trajectory vector
  // for obstacle i
  std::vector<Eigen::MatrixXd> xs_i_p, v_ou_p_i;

  // Prediction scenario trajectory covariance for obstacle i
  Eigen::MatrixXd P_i_p;

  // The shape of the predicted path for the ship
  Path_Prediction_Shape path_prediction_shape;

  /****************************************************************************************
   *  Name     : setup_mrou_obstacle_prediction_variables
   *  Function :
   *  Author   :
   *  Modified :
   *****************************************************************************************/
  template <class MPC_Parameters>
  void setup_mrou_prediction(
      const int i, // In: Index of obstacle in consideration
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    int n_cc(2); // Number of course changes for the obstacle (first at t_0)
    int course_change_count(0);

    n_ps[i] = n_ps_MROU;

    // std::cout << "obst i = " << i << " | t_cpa = " << t_cpa_i << "n_turns = "
    // << n_turns << std::endl;

    ps_maneuver_times_i.resize(n_cc, n_ps[i]);
    ps_course_changes_i.resize(n_cc, n_ps[i]);
    ps_maneuver_times_i(0, 0) = 0;
    ps_maneuver_times_i(1, 0) = 0;
    ps_course_changes_i(0, 0) = 0;
    ps_course_changes_i(1, 0) = 0;
    for (int ps = 1; ps < n_ps[i]; ps++) {
      ps_maneuver_times_i(0, ps) = 0;
      ps_maneuver_times_i(1, ps) = mpc_pars.t_ts;

      // Starboard maneuvers
      if (ps < (n_ps[i] - 1) / 2 + 1) {
        ps_course_changes_i(0, ps) = course_changes(course_change_count);
        ps_course_changes_i(1, ps) = -course_changes(course_change_count);

        if (++course_change_count == course_changes.size()) {
          course_change_count = 0;
        }
      }
      // Port maneuvers
      else {
        ps_course_changes_i(0, ps) = -course_changes(course_change_count);
        ps_course_changes_i(1, ps) = course_changes(course_change_count);

        if (++course_change_count == course_changes.size()) {
          course_change_count = 0;
        }
      }
    }
    std::cout << "Obstacle PS course changes : " << ps_course_changes_i
              << std::endl;
    std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i
              << std::endl;
  }

  /****************************************************************************************
   *  Name     : initialize_independent_mrou_prediction (MROU and LOS)
   *  Function : Sets up independent obstacle prediction.
   *  Author   : Trym Tengesdal
   *  Modified :
   *****************************************************************************************/
  template <class MPC_Parameters>
  void initialize_independent_mrou_prediction(
      const Dynamic_Obstacles
          &obstacles, // In/Out: Dynamic obstacle information
      const int i,    // In: Index of obstacle whose prediction to initialize
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state, either [x, y, psi, u, v, r]^T
                          // or [x, y, chi, U]^T
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    /* Eigen::VectorXd t_cpa, d_cpa;
    Eigen::Vector2d p_cpa; */
    Eigen::Vector2d v_os_0;
    Eigen::Vector4d xs_i_0, xs_0;
    if (ownship_state.rows() == 4) {
      v_os_0(0) = ownship_state(3) * cos(ownship_state(2));
      v_os_0(1) = ownship_state(3) * sin(ownship_state(2));
    } else {
      v_os_0(0) = ownship_state(3);
      v_os_0(1) = ownship_state(4);
      v_os_0 = CPU::rotate_vector_2D(v_os_0, ownship_state(2, 0));
    }
    xs_0.block<2, 1>(0, 0) = ownship_state.block<2, 1>(0, 0);
    xs_0(2) = v_os_0(0);
    xs_0(3) = v_os_0(1);

    xs_i_0 = obstacles[i].kf.get_state();
    /* std::cout << "xs_i_0 = " << xs_i_0.transpose() << std::endl;
    std::cout << "xs_0 = " << xs_0.transpose() << std::endl; */
    // CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);
    /* std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
    std::cout << "t_cpa(i) = " << t_cpa(i) << std::endl;
    std::cout << "d_cpa(i) = " << d_cpa(i)<< std::endl; */

    if (CPU::ship_is_passed_by(xs_0, xs_i_0, mpc_pars.d_safe)) {
      /* std::cout << "Obstacle i = " << i << "is passed => 1 PS only" <<
       * std::endl; */
      ps_course_changes_i.resize(1, 1);
      ps_course_changes_i(0, 0) = 0;
      ps_maneuver_times_i.resize(1, 1);
      ps_maneuver_times_i(0, 0) = 0;
      n_ps[i] = 1;
    } else {
      setup_mrou_prediction(i, mpc_pars);
    }
  }

  template <class MPC_Parameters>
  void initialize_independent_los_prediction(
      const Dynamic_Obstacles
          &obstacles, // In/Out: Dynamic obstacle information
      const int i,    // In: Index of obstacle whose prediction to initialize
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state, either [x, y, psi, u, v, r]^T
                          // or [x, y, chi, U]^T
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    n_ps[i] = n_ps_LOS;

    Eigen::MatrixXd waypoints_i = obstacles[i].get_waypoints();
    Eigen::Vector4d xs_i_0 = obstacles[i].kf.get_state();

    // Either an irrelevant obstacle or too far away to consider
    double d_0i =
        (xs_i_0.block<2, 1>(0, 0) - ownship_state.block<2, 1>(0, 0)).norm();
    // bool is_passed = CPU::ship_is_passed_by(ownship_state, xs_i_0,
    // mpc_pars.d_safe); if (is_passed || d_0i > mpc_pars.d_do_relevant)
    if (d_0i > mpc_pars.d_do_relevant) {
      ct_offsets.resize(1);
      ct_offsets(0) = 0.0;
      n_ps[i] = 1;
      return;
    }

    double alpha(0.0), e(0.0);
    // The obstacle has entered colregs/prediction range if its waypoint matrix
    // is initialized with 2 columns (straight line)
    alpha = atan2(waypoints_i(1, 1) - waypoints_i(1, 0),
                  waypoints_i(0, 1) - waypoints_i(0, 0));

    e = -(xs_i_0(0) - waypoints_i(0, 0)) * sin(alpha) +
        (xs_i_0(1) - waypoints_i(1, 0)) * cos(alpha);

    ct_offsets.resize(n_ps_LOS);
    int multiplier = (n_ps_LOS - 1) / 2;
    for (int ps = 0; ps < n_ps_LOS; ps++) {
      if (ps < (n_ps_LOS - 1) / 2) {
        ct_offsets(ps) = -multiplier * r_ct - e;
        multiplier -= 1;
      } else {
        ct_offsets(ps) = multiplier * r_ct - e;
        multiplier += 1;
      }
    }
  }

  // Pybind11 compatibility overload of initialize_independent_los_prediction
  template <class PSBMPC_Parameters>
  void initialize_independent_los_prediction_py(
      const std::vector<std::shared_ptr<Tracked_Obstacle>>
          &obstacles, // In/Out: Dynamic obstacle information
      const int i,    // In: Index of obstacle whose prediction to initialize
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state, either [x, y, psi, u, v, r]^T
                          // or [x, y, chi, U]^T
      const PSBMPC_Parameters &mpc_pars, // In: Calling PSBMPC parameters
      const Path_Prediction_Shape
          path_prediction_shape // In: The shape of the predicted path for the
                                // ship
  ) {
    n_ps[i] = n_ps_LOS;

    Eigen::MatrixXd waypoints_i = obstacles[i]->get_waypoints();
    Eigen::Vector4d xs_i_0 = obstacles[i]->kf.get_state();

    // Either an irrelevant obstacle or too far away to consider
    double d_0i =
        (xs_i_0.block<2, 1>(0, 0) - ownship_state.block<2, 1>(0, 0)).norm();
    // bool is_passed = CPU::ship_is_passed_by(ownship_state, xs_i_0,
    // mpc_pars.d_safe); if (is_passed || d_0i > mpc_pars.d_do_relevant)
    if (d_0i > mpc_pars.d_do_relevant) {
      ct_offsets.resize(1);
      ct_offsets(0) = 0.0;

      chi_offsets.resize(1);
      chi_offsets(0) = 0.0;

      n_ps[i] = 1;

      return;
    }

    chi_offsets.resize(n_ps_LOS);
    chi_offsets = chi_offsets_params; // Predefined parameters from the init. of
                                      // the Obstacle_Predictor class

    double alpha(0.0), e(0.0);
    // The obstacle has entered colregs/prediction range if its waypoint matrix
    // is initialized with 2 columns (straight line)
    alpha = atan2(waypoints_i(1, 1) - waypoints_i(1, 0),
                  waypoints_i(0, 1) - waypoints_i(0, 0));

    e = -(xs_i_0(0) - waypoints_i(0, 0)) * sin(alpha) +
        (xs_i_0(1) - waypoints_i(1, 0)) * cos(alpha);

    ct_offsets.resize(n_ps_LOS);
    int multiplier = (n_ps_LOS - 1) / 2;
    for (int ps = 0; ps < n_ps_LOS; ps++) {
      if (ps < (n_ps_LOS - 1) / 2) {
        ct_offsets(ps) = -multiplier * r_ct - e;
        multiplier -= 1;
      } else {
        ct_offsets(ps) = multiplier * r_ct - e;
        multiplier += 1;
      }
    }
  }

  /****************************************************************************************
   *  Name     : predict_independent_mrou_trajectories
   *  Function : More refined obstacle prediction with avoidance-like
   * trajectories, including the straight-line trajectory
   * Author   : Trym Tengesdal
   * Modified :
   *****************************************************************************************/
  template <class MPC_Parameters>
  void predict_independent_mrou_trajectories(
      Dynamic_Obstacles &obstacles, // In/Out: Dynamic obstacle information
      const int i, // In: Index of obstacle whose trajectories to predict
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    int n_samples = std::round(mpc_pars.T / mpc_pars.dt);

    Eigen::Matrix4d P_0, P;
    Eigen::Matrix2d P_rot_2D;
    P_i_p.resize(16, n_samples);
    P_0 = obstacles[i].kf.get_covariance();
    P_i_p.col(0) = CPU::flatten(P_0);

    xs_i_p.resize(n_ps[i]);
    v_ou_p_i.resize(n_ps[i]);

    Eigen::Vector2d v_p, v_p_new;
    double chi_ps_ou(0.0), chi_ps(0.0), t(0.0);
    int turn_count(0);
    for (int ps = 0; ps < n_ps[i]; ps++) {
      xs_i_p[ps].resize(4, n_samples);
      xs_i_p[ps].col(0) = obstacles[i].kf.get_state();

      v_p(0) = obstacles[i].kf.get_state()(2);
      v_p(1) = obstacles[i].kf.get_state()(3);
      v_ou_p_i[ps].resize(2, n_samples);

      turn_count = 0;
      for (int k = 0; k < n_samples; k++) {
        t = k * mpc_pars.dt;

        // Starboard/port maneuver velocity rotation
        if (t == ps_maneuver_times_i(turn_count, ps) && ps != 0) {
          chi_ps_ou = atan2(v_p(1), v_p(0));
          v_p_new(0) =
              v_p.norm() * cos(chi_ps_ou + ps_course_changes_i(turn_count, ps));
          v_p_new(1) =
              v_p.norm() * sin(chi_ps_ou + ps_course_changes_i(turn_count, ps));
          v_p = v_p_new;
          if (turn_count < ps_maneuver_times_i.rows() - 1) {
            turn_count += 1;
          }
        }

        v_ou_p_i[ps].col(k) = v_p;
        if (k < n_samples - 1) {
          xs_i_p[ps].col(k + 1) =
              mrou.predict_state(xs_i_p[ps].col(k), v_p, mpc_pars.dt);

          chi_ps = atan2(xs_i_p[ps](3, k + 1), xs_i_p[ps](2, k + 1));
          if (ps == 0) {
            P = mrou.predict_covariance(P_0, t + mpc_pars.dt);
            P_i_p.col(k + 1) = CPU::flatten(P);

            // Add constraint on cross-track variance here
            P_rot_2D = CPU::rotate_matrix_2D(P.block<2, 2>(0, 0), chi_ps);

            if (3 * sqrt(P_rot_2D(1, 1)) > r_ct) {
              // std::cout << P_rot_2D << std::endl;
              P_rot_2D(1, 1) = pow(r_ct, 2) / 3.0;
              // P_i_p.col(k + 1) =
            }
          }
        }
      }
    }
  }

  /****************************************************************************************
   *  Name     : predict_independent_los_trajectories
   *  Function : More refined obstacle prediction with avoidance-like
   * trajectories, including the straight-line trajectory. Version two, using
   * LOS and the kinematic ship model Author   : Trym Tengesdal Modified :
   *****************************************************************************************/
  template <class MPC_Parameters>
  void predict_independent_los_trajectories(
      Dynamic_Obstacles &obstacles, // In/Out: Dynamic obstacle information
      const int i, // In: Index of obstacle whose trajectories to predict
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    int n_samples = std::round(mpc_pars.T / mpc_pars.dt);

    Eigen::MatrixXd trajectory, waypoints = obstacles[i].get_waypoints();
    Eigen::Vector4d xs_i_ps_k;
    Eigen::Matrix4d P_0, P;
    Eigen::Matrix2d P_rot_2D;
    P_i_p.resize(16, n_samples);
    P_0 = obstacles[i].kf.get_covariance();
    P_i_p.col(0) = CPU::flatten(P_0);

    xs_i_p.resize(n_ps[i]);

    CPU::Obstacle_Ship obstacle_ship;

    double chi_ps(0.0), t(0.0);
    for (int ps = 0; ps < n_ps[i]; ps++) {
      xs_i_p[ps].resize(4, n_samples);
      xs_i_p[ps].col(0) = obstacles[i].kf.get_state();

      // Transform obstacle state from [x, y, Vx, Vy]^T to [x, y, chi, U]^T
      xs_i_ps_k.block<2, 1>(0, 0) = xs_i_p[ps].block<2, 1>(0, 0);
      xs_i_ps_k(2) = atan2(xs_i_p[ps](3, 0), xs_i_p[ps](2, 0));
      xs_i_ps_k(3) = xs_i_p[ps].block<2, 1>(2, 0).norm();

      trajectory.resize(4, n_samples);
      trajectory.col(0) = xs_i_ps_k;

      obstacle_ship.predict_trajectory(trajectory, ct_offsets(ps), xs_i_ps_k(3),
                                       xs_i_ps_k(2), waypoints, ERK1,
                                       mpc_pars.T, mpc_pars.dt);

      // Predict covariance using MROU model
      for (int k = 0; k < n_samples; k++) {
        t = k * mpc_pars.dt;

        // Transform obstacle state from [x, y, chi, U]^T to [x, y, Vx, Vy]^T
        xs_i_ps_k.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, k);
        xs_i_ps_k(2) = trajectory(3, k) * cos(trajectory(2, k));
        xs_i_ps_k(3) = trajectory(3, k) * sin(trajectory(2, k));
        xs_i_p[ps].col(k) = xs_i_ps_k;

        if (k < n_samples - 1) {
          chi_ps = trajectory(2, k);
          if (ps == (n_ps[i] - 1) / 2) {
            P = mrou.predict_covariance(P_0, t + mpc_pars.dt);
            /* std::cout << "P_MROU = " << std::endl;
            std::cout << P << std::endl; */

            // Add constraint on cross-track variance here
            P_rot_2D = CPU::rotate_matrix_2D(P.block<2, 2>(0, 0), chi_ps);

            if (3 * sqrt(P_rot_2D(1, 1)) > r_ct) {
              /* std::cout << P_rot_2D << std::endl; */
              P_rot_2D(1, 1) = pow(r_ct, 2) / 3.0;

              /* std::cout << "P_rot after" << std::endl;
              std::cout << P_rot_2D << std::endl; */

              P_rot_2D = CPU::rotate_matrix_2D(P_rot_2D, -chi_ps);

              P.block<2, 2>(0, 0) = P_rot_2D;

              /* std::cout << "P_rot after back rotation" << std::endl;
              std::cout << P_rot_2D << std::endl;
              std::cout << "P_MROU after constraining = " << std::endl;
              std::cout << P << std::endl; */
            }

            P_i_p.col(k + 1) = CPU::flatten(P);
          }
        }
      }
    }
  }

  // Pybind11 compatibility overload of predict_independent_los_trajectories
  template <class PSBMPC_Parameters>
  void predict_independent_los_trajectories_py(
      const std::vector<std::shared_ptr<Tracked_Obstacle>>
          &obstacles, // In/Out: Dynamic obstacle information
      const int i,    // In: Index of obstacle whose trajectories to predict
      const PSBMPC_Parameters &mpc_pars, // In: Calling PSBMPC parameters
      const Path_Prediction_Shape
          path_prediction_shape // In: The shape of the predicted path for the
                                // ship
  ) {
    int n_samples = std::round(mpc_pars.T / mpc_pars.dt);

    Eigen::MatrixXd trajectory, waypoints = obstacles[i]->get_waypoints();
    Eigen::Vector4d xs_i_ps_k;
    Eigen::Matrix4d P_0, P;
    Eigen::Matrix2d P_rot_2D;
    P_i_p.resize(16, n_samples);
    P_0 = obstacles[i]->kf.get_covariance();
    P_i_p.col(0) = CPU::flatten(P_0);

    xs_i_p.resize(n_ps[i]);

    CPU::Obstacle_Ship obstacle_ship;

    double chi_ps(0.0), t(0.0), chi_d_p_linear_pred_shape;
    for (int ps = 0; ps < n_ps[i]; ps++) {
      xs_i_p[ps].resize(4, n_samples);
      xs_i_p[ps].col(0) = obstacles[i]->kf.get_state();

      // Transform obstacle state from [x, y, Vx, Vy]^T to [x, y, chi, U]^T
      xs_i_ps_k.block<2, 1>(0, 0) = xs_i_p[ps].block<2, 1>(0, 0);
      xs_i_ps_k(2) = atan2(xs_i_p[ps](3, 0), xs_i_p[ps](2, 0));
      xs_i_ps_k(3) = xs_i_p[ps].block<2, 1>(2, 0).norm();

      trajectory.resize(4, n_samples);
      trajectory.col(0) = xs_i_ps_k;

      if (ps == 0) {
        chi_d_p_linear_pred_shape = xs_i_ps_k(2);
      }

      if (path_prediction_shape == LINEAR) {
        obstacle_ship.predict_trajectory(
            trajectory, ct_offsets(ps), chi_offsets(ps), xs_i_ps_k(3),
            chi_d_p_linear_pred_shape, waypoints, ERK1, path_prediction_shape,
            mpc_pars.T, mpc_pars.dt);
      } else {
        obstacle_ship.predict_trajectory(
            trajectory, ct_offsets(ps), chi_offsets(ps), xs_i_ps_k(3),
            xs_i_ps_k(2), waypoints, ERK1, path_prediction_shape, mpc_pars.T,
            mpc_pars.dt);
      }

      // Predict covariance using MROU model
      for (int k = 0; k < n_samples; k++) {
        t = k * mpc_pars.dt;

        // Transform obstacle state from [x, y, chi, U]^T to [x, y, Vx, Vy]^T
        xs_i_ps_k.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, k);
        xs_i_ps_k(2) = trajectory(3, k) * cos(trajectory(2, k));
        xs_i_ps_k(3) = trajectory(3, k) * sin(trajectory(2, k));
        xs_i_p[ps].col(k) = xs_i_ps_k;

        if (k < n_samples - 1) {
          chi_ps = trajectory(2, k);
          if (ps == (n_ps[i] - 1) / 2) {
            P = mrou.predict_covariance(P_0, t + mpc_pars.dt);
            /* std::cout << "P_MROU = " << std::endl;
            std::cout << P << std::endl; */

            // Add constraint on cross-track variance here
            P_rot_2D = CPU::rotate_matrix_2D(P.block<2, 2>(0, 0), chi_ps);

            if (3 * sqrt(P_rot_2D(1, 1)) > r_ct) {
              /* std::cout << P_rot_2D << std::endl; */
              P_rot_2D(1, 1) = pow(r_ct, 2) / 3.0;

              /* std::cout << "P_rot after" << std::endl;
              std::cout << P_rot_2D << std::endl; */

              P_rot_2D = CPU::rotate_matrix_2D(P_rot_2D, -chi_ps);

              P.block<2, 2>(0, 0) = P_rot_2D;

              /* std::cout << "P_rot after back rotation" << std::endl;
              std::cout << P_rot_2D << std::endl;
              std::cout << "P_MROU after constraining = " << std::endl;
              std::cout << P << std::endl; */
            }

            P_i_p.col(k + 1) = CPU::flatten(P);
          }
        }
      }
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MROU mrou;

  Obstacle_Predictor()
      //: n_ps_MROU(5), n_ps_LOS(5), r_ct(10.0), mrou(0.01, 0.0, 0.01, 0.1, 0.1)
      : n_ps_MROU(5), n_ps_LOS(5), r_ct(25.0), path_prediction_shape(SMOOTH),
        mrou(0.1, 0.0, 0.1, 0.1, 0.1) {
    if (n_ps_MROU == 3) {
      course_changes.resize(1);
      course_changes << 45 * DEG2RAD;
    } else if (n_ps_MROU == 5) {
      course_changes.resize(2);
      course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
    } else {
      course_changes.resize(3);
      course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
    }
    chi_offsets_params.resize(5);
    chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, 0 * DEG2RAD,
        30 * DEG2RAD, 60 * DEG2RAD;
  }

  Obstacle_Predictor(const PSBMPC_Parameters &pars)
      : n_ps_MROU(pars.n_do_ps), n_ps_LOS(pars.n_do_ps), r_ct(20.0),
        path_prediction_shape(SMOOTH), mrou(0.025, 0.0, 0.025, 0.1, 0.1) {
    if (n_ps_MROU == 3) {
      course_changes.resize(1);
      course_changes << 45 * DEG2RAD;
    } else if (n_ps_MROU == 5) {
      course_changes.resize(2);
      course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
    } else {
      course_changes.resize(3);
      course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
    }

    chi_offsets_params.resize(n_ps_LOS);
    if (pars.n_do_ps == 1) {
      chi_offsets_params << 0;
    } else if (pars.n_do_ps == 2) {
      chi_offsets_params << -5 * DEG2RAD, 5 * DEG2RAD;
    } else if (pars.n_do_ps == 3) {
      chi_offsets_params << -30 * DEG2RAD, 0, 30 * DEG2RAD;
    } else if (pars.n_do_ps == 4) {
      chi_offsets_params << -20 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
          20 * DEG2RAD;
    } else if (pars.n_do_ps == 5) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, 0, 30 * DEG2RAD,
          60 * DEG2RAD;
    } else if (pars.n_do_ps == 6) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -5 * DEG2RAD,
          5 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (pars.n_do_ps == 7) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0,
          15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (pars.n_do_ps == 8) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD,
          -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (pars.n_do_ps == 9) {
      chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
          -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD,
          60 * DEG2RAD;
    } else if (pars.n_do_ps == 10) {
      chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
          -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD,
          45 * DEG2RAD, 60 * DEG2RAD;
    } else if (pars.n_do_ps == 11) {
      chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
          -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD,
          45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
    } else if (pars.n_do_ps == 12) {
      chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
          -30 * DEG2RAD, -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
    } else if (pars.n_do_ps == 13) {
      chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
          -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD, 90 * DEG2RAD;
    } else if (pars.n_do_ps >=
               14) { // No point in predicting paths set to more than 90 degs
                     // (in the LINEAR case), as this means the path points
                     // backwards relative to the obstacle ship's direction
      std::cout
          << "ERROR WARNING: n_do_ps set to larger than 13, n_ps_LOS "
             "overwritten to size 13 (13 is arbitrary can be changed to "
             "another desired value by changing the obstacle predictor code)"
          << std::endl;
      n_ps_LOS = 13;
      chi_offsets_params.resize(n_ps_LOS);
      chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
          -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD, 90 * DEG2RAD;
    }
  }

  Obstacle_Predictor(
      const PSBMPC_Parameters &pars, // In: PSB-MPC Parameters
      const double
          r_ct, // In: Cross-track spacing between obstacle trajectories
      const Path_Prediction_Shape
          path_prediction_shape, // In: The shape of the predicted path for the
                                 // obstacle ship(s)
      const Eigen::VectorXd chi_offsets_degrees // In: Preset course offsets in
                                                // LOS-prediction for obstacles
      )
      : n_ps_MROU(pars.n_do_ps), n_ps_LOS(pars.n_do_ps), r_ct(r_ct),
        path_prediction_shape(path_prediction_shape),
        mrou(0.025, 0.0, 0.025, 0.1, 0.1) {
    if (n_ps_MROU == 3) {
      course_changes.resize(1);
      course_changes << 45 * DEG2RAD;
    } else if (n_ps_MROU == 5) {
      course_changes.resize(2);
      course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
    } else {
      course_changes.resize(3);
      course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
    }

    chi_offsets_params =
        chi_offsets_degrees * DEG2RAD; // Expected input in degrees, expected
                                       // input of size equal to n_do_ps

    if (chi_offsets_params.size() != pars.n_do_ps) {
      if (path_prediction_shape == LINEAR) {
        std::cout
            << "WARNING: Parameter mismatch - Number of elements (angles) in "
               "chi_offsets in targetship_params should equal n_do_ps."
            << std::endl;
        std::cout
            << "Using a predefined chi_offsets of size == n_do_ps, instead of "
               "the chi_offsets defined by the input parameters."
            << std::endl;
      }
      chi_offsets_params.resize(pars.n_do_ps);
      if (pars.n_do_ps == 1) {
        chi_offsets_params << 0;
      } else if (pars.n_do_ps == 2) {
        chi_offsets_params << -5 * DEG2RAD, 5 * DEG2RAD;
      } else if (pars.n_do_ps == 3) {
        chi_offsets_params << -30 * DEG2RAD, 0, 30 * DEG2RAD;
      } else if (pars.n_do_ps == 4) {
        chi_offsets_params << -20 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
            20 * DEG2RAD;
      } else if (pars.n_do_ps == 5) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, 0, 30 * DEG2RAD,
            60 * DEG2RAD;
      } else if (pars.n_do_ps == 6) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -5 * DEG2RAD,
            5 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 7) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0,
            15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 8) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD,
            -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 9) {
        chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
            -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD,
            60 * DEG2RAD;
      } else if (pars.n_do_ps == 10) {
        chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
            -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 11) {
        chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
            -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD,
            45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
      } else if (pars.n_do_ps == 12) {
        chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
            -30 * DEG2RAD, -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
            15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD,
            75 * DEG2RAD;
      } else if (pars.n_do_ps == 13) {
        chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
            -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD,
            90 * DEG2RAD;
      } else if (pars.n_do_ps >=
                 14) { // No point in predicting paths set to more than 90 degs
                       // (in the LINEAR case), as this means the path points
                       // backwards relative to the obstacle ship's direction
        std::cout
            << "ERROR WARNING: n_do_ps was set to larger than 13, n_ps_LOS "
               "overwritten to size 13 (13 is arbitrary can be changed to "
               "another desired value by changing the obstacle predictor code)"
            << std::endl;
        n_ps_LOS = 13;
        chi_offsets_params.resize(n_ps_LOS);
        chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
            -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD,
            90 * DEG2RAD;
      }
    }
  }

  Obstacle_Predictor(
      const SBMPC_Parameters &pars, // In: SB-MPC Parameters
      const double
          r_ct, // In: Cross-track spacing between obstacle trajectories
      const Path_Prediction_Shape
          path_prediction_shape, // In: The shape of the predicted path for the
                                 // obstacle ship(s)
      const Eigen::VectorXd chi_offsets_degrees // In: Preset course offsets in
                                                // LOS-prediction for obstacles
      )
      : n_ps_MROU(pars.n_do_ps), n_ps_LOS(pars.n_do_ps), r_ct(r_ct),
        path_prediction_shape(path_prediction_shape),
        mrou(0.025, 0.0, 0.025, 0.1, 0.1) {
    if (n_ps_MROU == 3) {
      course_changes.resize(1);
      course_changes << 45 * DEG2RAD;
    } else if (n_ps_MROU == 5) {
      course_changes.resize(2);
      course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
    } else {
      course_changes.resize(3);
      course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
    }

    chi_offsets_params =
        chi_offsets_degrees * DEG2RAD; // Expected input in degrees, expected
                                       // input of size equal to n_do_ps
    if (chi_offsets_params.size() != pars.n_do_ps) {
      if (path_prediction_shape == LINEAR) {
        std::cout
            << "WARNING: Parameter mismatch - Number of elements (angles) in "
               "chi_offsets in targetship_params should equal n_do_ps."
            << std::endl;
        std::cout
            << "Using a predefined chi_offsets of size == n_do_ps, instead of "
               "the chi_offsets defined by the input parameters."
            << std::endl;
      }
      chi_offsets_params.resize(pars.n_do_ps);
      if (pars.n_do_ps == 1) {
        chi_offsets_params << 0;
      } else if (pars.n_do_ps == 2) {
        chi_offsets_params << -5 * DEG2RAD, 5 * DEG2RAD;
      } else if (pars.n_do_ps == 3) {
        chi_offsets_params << -30 * DEG2RAD, 0, 30 * DEG2RAD;
      } else if (pars.n_do_ps == 4) {
        chi_offsets_params << -20 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
            20 * DEG2RAD;
      } else if (pars.n_do_ps == 5) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, 0, 30 * DEG2RAD,
            60 * DEG2RAD;
      } else if (pars.n_do_ps == 6) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -5 * DEG2RAD,
            5 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 7) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0,
            15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 8) {
        chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD,
            -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 9) {
        chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
            -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD,
            60 * DEG2RAD;
      } else if (pars.n_do_ps == 10) {
        chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
            -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD;
      } else if (pars.n_do_ps == 11) {
        chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
            -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD,
            45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
      } else if (pars.n_do_ps == 12) {
        chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
            -30 * DEG2RAD, -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
            15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD,
            75 * DEG2RAD;
      } else if (pars.n_do_ps == 13) {
        chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
            -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD,
            90 * DEG2RAD;
      } else if (pars.n_do_ps >=
                 14) { // No point in predicting paths set to more than 90 degs
                       // (in the LINEAR case), as this means the path points
                       // backwards relative to the obstacle ship's direction
        std::cout
            << "ERROR WARNING: n_do_ps was set to larger than 13, n_ps_LOS "
               "overwritten to size 13 (13 is arbitrary can be changed to "
               "another desired value by changing the obstacle predictor code)"
            << std::endl;
        n_ps_LOS = 13;
        chi_offsets_params.resize(n_ps_LOS);
        chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
            -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
            30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD,
            90 * DEG2RAD;
      }
    }
  }

  template <class MPC_Parameters>
  Obstacle_Predictor(
      const double
          r_ct, // In: Cross-track spacing between obstacle trajectories
      const double sigma_x, // In: MROU wiener process noise parameters
      const double sigma_xy, const double sigma_y,
      const double gamma_x, // In: MROU reversion strength parameters
      const double gamma_y,
      const MPC_Parameters
          &mpc_pars // In: Parameters of calling MPC (SB or PSB-MPC)
      )
      : n_ps_MROU(mpc_pars.n_do_ps), n_ps_LOS(mpc_pars.n_do_ps), r_ct(r_ct),
        path_prediction_shape(SMOOTH),
        mrou(sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y) {
    if (n_ps_MROU == 3) {
      course_changes.resize(1);
      course_changes << 45 * DEG2RAD;
    } else if (n_ps_MROU == 5) {
      course_changes.resize(2);
      course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
    } else {
      course_changes.resize(3);
      course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
    }

    chi_offsets_params.resize(n_ps_LOS);
    if (mpc_pars.n_do_ps == 1) {
      chi_offsets_params << 0;
    } else if (mpc_pars.n_do_ps == 2) {
      chi_offsets_params << -5 * DEG2RAD, 5 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 3) {
      chi_offsets_params << -30 * DEG2RAD, 0, 30 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 4) {
      chi_offsets_params << -20 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD,
          20 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 5) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, 0, 30 * DEG2RAD,
          60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 6) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -5 * DEG2RAD,
          5 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 7) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0,
          15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 8) {
      chi_offsets_params << -60 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD,
          -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD, 60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 9) {
      chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
          -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD, 45 * DEG2RAD,
          60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 10) {
      chi_offsets_params << -60 * DEG2RAD, -45 * DEG2RAD, -30 * DEG2RAD,
          -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD, 30 * DEG2RAD,
          45 * DEG2RAD, 60 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 11) {
      chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
          -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD, 30 * DEG2RAD,
          45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 12) {
      chi_offsets_params << -75 * DEG2RAD, -60 * DEG2RAD, -45 * DEG2RAD,
          -30 * DEG2RAD, -15 * DEG2RAD, -5 * DEG2RAD, 5 * DEG2RAD, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD;
    } else if (mpc_pars.n_do_ps == 13) {
      chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
          -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD, 90 * DEG2RAD;
    } else if (mpc_pars.n_do_ps >=
               14) { // No point in predicting paths set to more than 90 degs
                     // (in the LINEAR case), as this means the path points
                     // backwards relative to the obstacle ship's direction
      std::cout
          << "ERROR WARNING: n_do_ps set to larger than 13, n_ps_LOS "
             "overwritten to size 13 (13 is arbitrary can be changed to "
             "another desired value by changing the obstacle predictor code)"
          << std::endl;
      n_ps_LOS = 13;
      chi_offsets_params.resize(n_ps_LOS);
      chi_offsets_params << -90 * DEG2RAD, -75 * DEG2RAD, -60 * DEG2RAD,
          -45 * DEG2RAD, -30 * DEG2RAD, -15 * DEG2RAD, 0, 15 * DEG2RAD,
          30 * DEG2RAD, 45 * DEG2RAD, 60 * DEG2RAD, 75 * DEG2RAD, 90 * DEG2RAD;
    }
  }

  int get_n_ps_i(const int i) const { return n_ps[i]; }

  void set_n_ps_i(const int i, const int n_ps_i) { this->n_ps[i] = n_ps_i; }

  int get_n_ps_LOS() const { return n_ps_LOS; }

  void set_n_ps_LOS(const int n_ps_LOS) { this->n_ps_LOS = n_ps_LOS; }

  int get_n_ps_MROU() const { return n_ps_MROU; }

  void set_n_ps_MROU(const int n_ps_MROU) { this->n_ps_MROU = n_ps_MROU; }

  inline void set_r_ct(const double r_ct) { this->r_ct = r_ct; }

  inline double get_r_ct() const { return r_ct; }

  void set_chi_offsets(const Eigen::VectorXd &chi_offsets_params_degrees) {
    this->chi_offsets_params = chi_offsets_params_degrees * DEG2RAD;
  }

  Eigen::VectorXd get_chi_offsets() const {
    return chi_offsets_params * RAD2DEG;
  }

  inline void
  set_path_prediction_shape(const Path_Prediction_Shape path_prediction_shape) {
    this->path_prediction_shape = path_prediction_shape;
  }

  inline Path_Prediction_Shape get_path_prediction_shape() const {
    return path_prediction_shape;
  }

  /****************************************************************************************
   *  Name     : operator()
   *  Function : Initializes and predicts obstacle trajectories
   *  Author   : Trym Tengesdal
   *  Modified :
   *****************************************************************************************/
  template <class MPC_Parameters>
  void operator()(
      Dynamic_Obstacles &obstacles, // In/Out: Dynamic obstacle information
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state at the current time
      const MPC_Parameters
          &mpc_pars // In: Calling MPC (either PSB-MPC or SB-MPC)
  ) {
    int n_do = obstacles.size();
    n_ps.resize(n_do);
    Eigen::MatrixXd waypoints_i;
    for (int i = 0; i < n_do; i++) {
      // Store the obstacle`s predicted waypoints if not done already
      // (as straight line path if no other info is available)
      if (obstacles[i].get_waypoints().cols() < 2) {
        waypoints_i.resize(2, 2);
        waypoints_i.col(0) = obstacles[i].kf.get_state().block<2, 1>(0, 0);
        waypoints_i.col(1) =
            waypoints_i.col(0) +
            mpc_pars.T * obstacles[i].kf.get_state().block<2, 1>(2, 0);
        obstacles[i].set_waypoints(waypoints_i);
      }

      initialize_independent_los_prediction(obstacles, i, ownship_state,
                                            mpc_pars);

      predict_independent_los_trajectories(obstacles, i, mpc_pars);

      // Transfer obstacles to the tracked obstacle
      obstacles[i].set_trajectories(xs_i_p);
      obstacles[i].set_mean_velocity_trajectories(v_ou_p_i);
      obstacles[i].set_trajectory_covariance(P_i_p);

      // Calculate scenario probabilities using intention model,
      // or just set to be uniform
      Eigen::VectorXd Pr_s_i(n_ps[i]);

      // Uniform
      for (int ps = 0; ps < n_ps[i]; ps++) {
        Pr_s_i(ps) = 1;
      }
      Pr_s_i = Pr_s_i / Pr_s_i.sum();

      // std::cout << "Obstacle i = " << i << "Pr_s_i = " << Pr_s_i.transpose()
      // << std::endl;
      obstacles[i].set_scenario_probabilities(Pr_s_i);
    }
  }

  template <class MPC_Parameters>
  void operator()(
      Dynamic_Obstacles &obstacles, // In/Out: Dynamic obstacle information
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state at the current time
      const MPC_Parameters
          &mpc_pars,      // In: Calling MPC (either PSB-MPC or SB-MPC)
      const bool is_sbmpc // In: SBMPC calling (True) or PSBMPC (False)
  ) {
    int n_do = obstacles.size();
    n_ps.resize(n_do);
    Eigen::MatrixXd waypoints_i;
    for (int i = 0; i < n_do; i++) {
      // Store the obstacle`s predicted waypoints if not done already
      // (as straight line path if no other info is available)
      if (obstacles[i].get_waypoints().cols() < 2) {
        waypoints_i.resize(2, 2);
        waypoints_i.col(0) = obstacles[i].kf.get_state().block<2, 1>(0, 0);
        waypoints_i.col(1) =
            waypoints_i.col(0) +
            mpc_pars.T * obstacles[i].kf.get_state().block<2, 1>(2, 0);
        obstacles[i].set_waypoints(waypoints_i);
      }

      if (is_sbmpc) // Reset waypoints to straight line path at each iteration
      {
        waypoints_i.resize(2, 2);
        waypoints_i.col(0) = obstacles[i].kf.get_state().block<2, 1>(0, 0);
        waypoints_i.col(1) =
            waypoints_i.col(0) +
            mpc_pars.T * obstacles[i].kf.get_state().block<2, 1>(2, 0);
        obstacles[i].set_waypoints(waypoints_i);
      }

      initialize_independent_los_prediction(obstacles, i, ownship_state,
                                            mpc_pars);

      predict_independent_los_trajectories(obstacles, i, mpc_pars);

      // Transfer obstacles to the tracked obstacle
      obstacles[i].set_trajectories(xs_i_p);
      obstacles[i].set_mean_velocity_trajectories(v_ou_p_i);
      obstacles[i].set_trajectory_covariance(P_i_p);

      // Calculate scenario probabilities using intention model,
      // or just set to be uniform
      Eigen::VectorXd Pr_s_i(n_ps[i]);

      // Uniform
      for (int ps = 0; ps < n_ps[i]; ps++) {
        Pr_s_i(ps) = 1;
      }
      Pr_s_i = Pr_s_i / Pr_s_i.sum();

      // std::cout << "Obstacle i = " << i << "Pr_s_i = " << Pr_s_i.transpose()
      // << std::endl;
      obstacles[i].set_scenario_probabilities(Pr_s_i);
    }
  }

  // Pybind11 compatibility overload of operator() with three input arguments
  std::vector<std::shared_ptr<Tracked_Obstacle>> &PSBMPC_operator_py(
      std::vector<std::shared_ptr<Tracked_Obstacle>>
          &obstacles, // In/out from/to Python: Dynamic obstacle information
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state at the current time
      const PSBMPC_Parameters &mpc_pars, // In: Calling PSBMPC parameters
      const Path_Prediction_Shape
          path_prediction_shape // In: The shape of the predicted path for the
                                // ship
  ) {
    int i(0);
    Eigen::MatrixXd waypoints_i;
    int n_do = obstacles.size();
    n_ps.resize(n_do);
    for (auto &obstacle : obstacles) {
      // Store the obstacle`s predicted waypoints as straight line path
      waypoints_i.resize(2, 2);
      waypoints_i.col(0) = obstacle->kf.get_state().block<2, 1>(0, 0);
      waypoints_i.col(1) =
          waypoints_i.col(0) +
          mpc_pars.T * obstacle->kf.get_state().block<2, 1>(2, 0);
      obstacle->set_waypoints(waypoints_i);

      initialize_independent_los_prediction_py(obstacles, i, ownship_state,
                                               mpc_pars, path_prediction_shape);
      predict_independent_los_trajectories_py(obstacles, i, mpc_pars,
                                              path_prediction_shape);

      // Transfer obstacles to the tracked obstacle
      obstacle->set_trajectories(xs_i_p);
      obstacle->set_mean_velocity_trajectories(v_ou_p_i);
      obstacle->set_trajectory_covariance(P_i_p);

      // Calculate scenario probabilities using intention model,
      // or just set to be uniform
      Eigen::VectorXd Pr_s_i(n_ps[i]);

      // Uniform
      for (int ps = 0; ps < n_ps[i]; ps++) {
        Pr_s_i(ps) = 1;
      }
      Pr_s_i = Pr_s_i / Pr_s_i.sum();

      // std::cout << "Obstacle i = " << i << "Pr_s_i = " << Pr_s_i.transpose()
      // << std::endl;
      obstacle->set_scenario_probabilities(Pr_s_i);
      i = i + 1;
    };
    return obstacles;
  }

  // Pybind11 compatibility overload of operator() with three input arguments
  std::vector<std::shared_ptr<Tracked_Obstacle>> &SBMPC_operator_py(
      std::vector<std::shared_ptr<Tracked_Obstacle>>
          &obstacles, // In/out from/to Python: Dynamic obstacle information
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state at the current time
      const SBMPC_Parameters &mpc_pars, // In: Calling SBMPC parameters
      const Path_Prediction_Shape
          path_prediction_shape // In: The shape of the predicted path for the
                                // ship
  ) {
    int i(0);
    Eigen::MatrixXd waypoints_i;
    int n_do = obstacles.size();
    n_ps.resize(n_do);
    for (auto &obstacle : obstacles) {
      // Store the obstacle`s predicted waypoints as straight line path
      // A static var. could be used to change waypoints in a periodic manner
      waypoints_i.resize(2, 2);
      waypoints_i.col(0) = obstacle->kf.get_state().block<2, 1>(0, 0);
      waypoints_i.col(1) =
          waypoints_i.col(0) +
          mpc_pars.T * obstacle->kf.get_state().block<2, 1>(2, 0);
      obstacle->set_waypoints(waypoints_i);

      initialize_independent_los_prediction_py(obstacles, i, ownship_state,
                                               mpc_pars, path_prediction_shape);
      predict_independent_los_trajectories_py(obstacles, i, mpc_pars,
                                              path_prediction_shape);

      // Transfer obstacles to the tracked obstacle
      obstacle->set_trajectories(xs_i_p);
      obstacle->set_mean_velocity_trajectories(v_ou_p_i);
      obstacle->set_trajectory_covariance(P_i_p);

      // Calculate scenario probabilities using intention model,
      // or just set to be uniform
      Eigen::VectorXd Pr_s_i(n_ps[i]);

      // Uniform
      for (int ps = 0; ps < n_ps[i]; ps++) {
        Pr_s_i(ps) = 1;
      }
      Pr_s_i = Pr_s_i / Pr_s_i.sum();

      // std::cout << "Obstacle i = " << i << "Pr_s_i = " << Pr_s_i.transpose()
      // << std::endl;
      obstacle->set_scenario_probabilities(Pr_s_i);
      i = i + 1;
    };
    return obstacles;
  }

  // Pybind11 utility method for updating a single obstacle (only for the
  // PSBMPC)
  std::vector<std::shared_ptr<Tracked_Obstacle>> &single_obstacle_predictor_py(
      std::vector<std::shared_ptr<Tracked_Obstacle>>
          &obstacles,    // In/out from/to Python: Dynamic obstacle information
      const int &obs_id, // In: ID of obstacle to update
      const Eigen::VectorXd
          &ownship_state, // In: Own-ship state at the current time
      const PSBMPC_Parameters &mpc_pars, // In: Calling PSBMPC parameters
      const double
          &r_ct, // In: Cross-track spacing between obstacle trajectories
      const Path_Prediction_Shape
          path_prediction_shape // In: The shape of the predicted path for the
                                // ship
  ) {
    int i(0);
    Eigen::MatrixXd waypoints_i;
    int n_do = obstacles.size();
    n_ps.resize(n_do);
    double r_ct_original = this->r_ct;
    this->r_ct = r_ct;
    for (auto &obstacle : obstacles) {
      if (obstacle->get_ID() == obs_id) {
        // Store the obstacle`s predicted waypoints as straight line path
        // A static var. could be used to change waypoints in a periodic manner
        waypoints_i.resize(2, 2);
        waypoints_i.col(0) = obstacle->kf.get_state().block<2, 1>(0, 0);
        waypoints_i.col(1) =
            waypoints_i.col(0) +
            mpc_pars.T * obstacle->kf.get_state().block<2, 1>(2, 0);
        obstacle->set_waypoints(waypoints_i);

        initialize_independent_los_prediction_py(
            obstacles, i, ownship_state, mpc_pars, path_prediction_shape);
        predict_independent_los_trajectories_py(obstacles, i, mpc_pars,
                                                path_prediction_shape);

        // Transfer obstacles to the tracked obstacle
        obstacle->set_trajectories(xs_i_p);
        obstacle->set_mean_velocity_trajectories(v_ou_p_i);
        obstacle->set_trajectory_covariance(P_i_p);

        // Calculate scenario probabilities using intention model,
        // or just set to be uniform
        Eigen::VectorXd Pr_s_i(n_ps[i]);

        // Uniform
        for (int ps = 0; ps < n_ps[i]; ps++) {
          Pr_s_i(ps) = 1;
        }
        Pr_s_i = Pr_s_i / Pr_s_i.sum();

        // std::cout << "Obstacle i = " << i << "Pr_s_i = " <<
        // Pr_s_i.transpose() << std::endl;
        obstacle->set_scenario_probabilities(Pr_s_i);
        break;
      }
      i = i + 1;
    };
    this->r_ct = r_ct_original;
    return obstacles;
  }
};
} // namespace PSBMPC_LIB