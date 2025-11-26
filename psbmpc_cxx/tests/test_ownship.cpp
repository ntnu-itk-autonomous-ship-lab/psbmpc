#if USE_GPU_PSBMPC
#if OWNSHIP_TYPE == 0
#include "gpu/kinematic_ship_models_gpu.cuh"
#else
#include "gpu/kinetic_ship_models_gpu.cuh"
#endif
#else
#if OWNSHIP_TYPE == 0
#include "cpu/kinematic_ship_models_cpu.hpp"
#else
#include "cpu/kinetic_ship_models_cpu.hpp"
#endif
#endif

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#define BUFSIZE 1000000
#endif

class OwnshipTest : public ::testing::Test {
protected:
  void SetUp() override {
    xs << 0, 0, 0, 15, 0, 0;
    T = 200.0;
    dt = 0.5;
    u_d = 10.0;
    chi_d = 0.0;

    offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1,
        0 * M_PI / 180.0;
    maneuver_times << 0, 100, 150;

#if USE_GPU_PSBMPC
    ownship = std::make_unique<PSBMPC_LIB::GPU::Ownship>();
#else
    ownship = std::make_unique<PSBMPC_LIB::CPU::Ownship>();
#endif

    n_samples = std::round(T / dt);

#if OWNSHIP_TYPE == 0
    trajectory.resize(4, n_samples);
    trajectory.col(0) = xs.block<4, 1>(0, 0);
#else
    trajectory.resize(6, n_samples);
    trajectory.col(0) = xs;
#endif

    waypoints.resize(2, 7);
    waypoints << 0, 200, 200, 0, 0, 300, 1000, 0, -50, -200, -200, 0, 300, 0;
  }

  Eigen::Matrix<double, 6, 1> xs;
  double T;
  double dt;
  double u_d;
  double chi_d;
  Eigen::VectorXd offset_sequence{6};
  Eigen::Vector3d maneuver_times;
  int n_samples;
  Eigen::MatrixXd trajectory;
  Eigen::Matrix<double, 2, -1> waypoints;
#if USE_GPU_PSBMPC
  std::unique_ptr<PSBMPC_LIB::GPU::Ownship> ownship;
#else
  std::unique_ptr<PSBMPC_LIB::CPU::Ownship> ownship;
#endif
};

TEST_F(OwnshipTest, PredictTrajectory) {
  ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d,
                              chi_d, waypoints, PSBMPC_LIB::ERK1,
                              PSBMPC_LIB::LOS, T, dt);

  EXPECT_EQ(trajectory.rows(), OWNSHIP_TYPE == 0 ? 4 : 6);
  EXPECT_EQ(trajectory.cols(), n_samples);

  EXPECT_NEAR((trajectory.col(0) - trajectory.col(0)).norm(), 0.0, 1e-6);

  for (int k = 1; k < n_samples; k++) {
    Eigen::VectorXd col = trajectory.col(k);
    EXPECT_FALSE(col.hasNaN());
    EXPECT_FALSE((col.array().isInf()).any());
  }
}

TEST_F(OwnshipTest, TrajectoryBounds) {
  ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d,
                              chi_d, waypoints, PSBMPC_LIB::ERK1,
                              PSBMPC_LIB::LOS, T, dt);

  double max_distance = 0.0;
  for (int k = 0; k < n_samples; k++) {
    double dist = trajectory.block<2, 1>(0, k).norm();
    if (dist > max_distance)
      max_distance = dist;
  }

  EXPECT_LT(max_distance, 1e6);
}

#if ENABLE_TEST_FILE_PLOTTING
TEST_F(OwnshipTest, MatlabPlotting) {
  Engine *ep = engOpen(NULL);
  if (ep == NULL) {
    GTEST_SKIP() << "MATLAB engine not available";
    return;
  }

  ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d,
                              chi_d, waypoints, PSBMPC_LIB::ERK1,
                              PSBMPC_LIB::LOS, T, dt);

  char buffer[BUFSIZE + 1];
  mxArray *traj_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
  mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);

  double *p_traj_mx = mxGetPr(traj_mx);
  double *p_wps_mx = mxGetPr(wps_mx);

  Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_mx, trajectory.rows(), n_samples);
  map_traj = trajectory;

  Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_mx, 2, 7);
  map_wps = waypoints;

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engPutVariable(ep, "X", traj_mx);
  engPutVariable(ep, "WPs", wps_mx);

  engEvalString(ep, "test_ownship_plot");

  mxDestroyArray(traj_mx);
  mxDestroyArray(wps_mx);
  engClose(ep);
}
#endif