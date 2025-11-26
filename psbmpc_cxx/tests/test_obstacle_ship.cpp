#if USE_GPU_PSBMPC
#include "gpu/kinematic_ship_models_gpu.cuh"
#else
#include "cpu/kinematic_ship_models_cpu.hpp"
#endif
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#define BUFSIZE 1000000
#endif

class ObstacleShipTest : public ::testing::Test {
protected:
  void SetUp() override {
    xs << 0, 0, 0, 15;
    T = 200.0;
    dt = 0.5;
    u_d = 10.0;
    chi_d = 0.0;

    offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1,
        0 * M_PI / 180.0;
    maneuver_times << 0, 100, 150;

    l = 10.0;
    w = 4.0;
    T_U = 10.0;
    T_chi = 7.5;
    R_a = 30.0;
    LOS_LD = 200.0;

#if USE_GPU_PSBMPC
    obstacle_ship = std::make_unique<PSBMPC_LIB::GPU::Obstacle_Ship>(
        l, w, T_U, T_chi, R_a, LOS_LD, 0);
#else
    obstacle_ship = std::make_unique<PSBMPC_LIB::CPU::Obstacle_Ship>(
        l, w, T_U, T_chi, R_a, LOS_LD, 0);
#endif

    n_samples = std::round(T / dt);
    trajectory.resize(4, n_samples);
    trajectory.block<4, 1>(0, 0) << xs;

    n_wps = 4;
    waypoints.resize(2, n_wps);
    waypoints << 0, 1000, 200, 0, 0, 0, -100, -100;
  }

  Eigen::Matrix<double, 4, 1> xs;
  double T;
  double dt;
  double u_d;
  double chi_d;
  Eigen::VectorXd offset_sequence{6};
  Eigen::Vector3d maneuver_times;
  double l, w, T_U, T_chi, R_a, LOS_LD;
  int n_samples;
  int n_wps;
  Eigen::MatrixXd trajectory;
  Eigen::Matrix<double, 2, -1> waypoints;
#if USE_GPU_PSBMPC
  std::unique_ptr<PSBMPC_LIB::GPU::Obstacle_Ship> obstacle_ship;
#else
  std::unique_ptr<PSBMPC_LIB::CPU::Obstacle_Ship> obstacle_ship;
#endif
};

TEST_F(ObstacleShipTest, DetermineActiveWaypointSegment) {
  obstacle_ship->determine_active_waypoint_segment(waypoints,
                                                   trajectory.col(0));

  EXPECT_EQ(trajectory.rows(), 4);
  EXPECT_EQ(trajectory.cols(), n_samples);
}

TEST_F(ObstacleShipTest, PredictTrajectory) {
  obstacle_ship->determine_active_waypoint_segment(waypoints,
                                                   trajectory.col(0));
  obstacle_ship->predict_trajectory(trajectory, offset_sequence, maneuver_times,
                                    u_d, chi_d, waypoints, PSBMPC_LIB::ERK1,
                                    PSBMPC_LIB::LOS, T, dt);

  EXPECT_EQ(trajectory.rows(), 4);
  EXPECT_EQ(trajectory.cols(), n_samples);

  for (int k = 1; k < n_samples; k++) {
    Eigen::VectorXd col = trajectory.col(k);
    EXPECT_FALSE(col.hasNaN());
    EXPECT_FALSE((col.array().isInf()).any());
  }
}

#if ENABLE_TEST_FILE_PLOTTING
TEST_F(ObstacleShipTest, MatlabPlotting) {
  Engine *ep = engOpen(NULL);
  if (ep == NULL) {
    GTEST_SKIP() << "MATLAB engine not available";
    return;
  }

  obstacle_ship->determine_active_waypoint_segment(waypoints,
                                                   trajectory.col(0));
  obstacle_ship->predict_trajectory(trajectory, offset_sequence, maneuver_times,
                                    u_d, chi_d, waypoints, PSBMPC_LIB::ERK1,
                                    PSBMPC_LIB::LOS, T, dt);

  char buffer[BUFSIZE + 1];
  mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
  mxArray *wps_mx = mxCreateDoubleMatrix(2, n_wps, mxREAL);
  mxArray *T_sim_mx = mxCreateDoubleScalar(T);
  mxArray *dt_sim_mx = mxCreateDoubleScalar(dt);

  double *p_traj_mx = mxGetPr(traj_mx);
  double *p_wps_mx = mxGetPr(wps_mx);

  Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_mx, 4, n_samples);
  map_traj = trajectory;

  Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_mx, 2, n_wps);
  map_wps = waypoints;

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engPutVariable(ep, "X", traj_mx);
  engPutVariable(ep, "WPs", wps_mx);
  engPutVariable(ep, "T_sim", T_sim_mx);
  engPutVariable(ep, "dt_sim", dt_sim_mx);

  engEvalString(ep, "test_obstacle_ship_plot");

  mxDestroyArray(traj_mx);
  mxDestroyArray(wps_mx);
  mxDestroyArray(T_sim_mx);
  mxDestroyArray(dt_sim_mx);
  engClose(ep);
}
#endif
