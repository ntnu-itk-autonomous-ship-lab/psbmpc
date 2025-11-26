#include "cpu/utilities_cpu.hpp"
#include "mrou.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#define BUFSIZE 1000000
#endif

class MROUTest : public ::testing::Test {
protected:
  void SetUp() override {
    T = 300.0;
    dt = 0.5;
    n_samples = std::round(T / dt);

    sigma_x = 0.8;
    sigma_xy = 0.0;
    sigma_y = 0.8;
    gamma_x = 0.1;
    gamma_y = 0.1;

    xs_0 << 0, 0, 2, 0;
    P_0 << 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0.025;

    mrou = std::make_unique<PSBMPC_LIB::MROU>(sigma_x, sigma_xy, sigma_y,
                                              gamma_x, gamma_y);

    xs_p.resize(1);
    xs_p[0].resize(4, n_samples);
    xs_p[0].col(0) = xs_0;

    P_p.resize(16, n_samples);
    P_p.col(0) = PSBMPC_LIB::CPU::flatten(P_0);

    v_p.resize(1);
    v_p[0].resize(2, n_samples);

    v_0 << 4, 0;
    v_p[0].col(0) = v_0;

    Eigen::VectorXd turn_times(1);
    turn_times << 100;

    int tt_count = 0;
    double chi = 0;
    Eigen::Vector2d v = v_0;
    for (int k = 0; k < n_samples; k++) {
      if (k == turn_times(tt_count)) {
        chi = atan2(v(1), v(0));
        v(0) = v.norm() * cos(chi + 30 * M_PI / 180.0);
        v(1) = v.norm() * sin(chi + 30 * M_PI / 180.0);
        if (tt_count < turn_times.size() - 1)
          tt_count += 1;
      }
      if (k < n_samples - 1)
        v_p[0].col(k + 1) = v;
    }

    double t = 0;
    Eigen::Vector4d xs = xs_0;
    Eigen::Matrix4d P = P_0;

    for (int k = 0; k < n_samples; k++) {
      t = (k + 1) * dt;
      xs = mrou->predict_state(xs, v_p[0].col(k), dt);
      P = mrou->predict_covariance(P_0, t);

      if (k < n_samples - 1) {
        xs_p[0].col(k + 1) = xs;
        P_p.col(k + 1) = PSBMPC_LIB::CPU::flatten(P);
      }
    }
  }

  double T;
  double dt;
  int n_samples;
  double sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y;
  Eigen::Vector4d xs_0;
  Eigen::Matrix4d P_0;
  Eigen::Vector2d v_0;
  std::unique_ptr<PSBMPC_LIB::MROU> mrou;
  std::vector<Eigen::MatrixXd> xs_p;
  std::vector<Eigen::MatrixXd> v_p;
  Eigen::MatrixXd P_p;
};

TEST_F(MROUTest, PredictState) {
  Eigen::Vector4d xs = xs_0;
  Eigen::Vector2d v;
  v << 4, 0;

  xs = mrou->predict_state(xs, v, dt);

  EXPECT_GT(xs(0), xs_0(0));
  EXPECT_NEAR(xs(1), xs_0(1), 1e-6);
  EXPECT_GT(xs(2), xs_0(2));
  EXPECT_NEAR(xs(3), xs_0(3), 1e-6);
}

TEST_F(MROUTest, PredictCovariance) {
  Eigen::Matrix4d P = mrou->predict_covariance(P_0, dt);

  EXPECT_GT(P.determinant(), 0.0);
  EXPECT_TRUE(P.isApprox(P.transpose()));

  for (int i = 0; i < 4; i++) {
    EXPECT_GE(P(i, i), 0.0);
  }
}

TEST_F(MROUTest, TrajectoryPrediction) {
  EXPECT_EQ(xs_p[0].rows(), 4);
  EXPECT_EQ(xs_p[0].cols(), n_samples);
  EXPECT_EQ(P_p.rows(), 16);
  EXPECT_EQ(P_p.cols(), n_samples);

  EXPECT_NEAR((xs_p[0].col(0) - xs_0).norm(), 0.0, 1e-6);

  for (int k = 1; k < n_samples; k++) {
    EXPECT_GT(xs_p[0](0, k), xs_p[0](0, k - 1) - 1e-6);
  }
}

#if ENABLE_TEST_FILE_PLOTTING
TEST_F(MROUTest, MatlabPlotting) {
  Engine *ep = engOpen(NULL);
  if (ep == NULL) {
    GTEST_SKIP() << "MATLAB engine not available";
    return;
  }

  char buffer[BUFSIZE + 1];

  mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
  mxArray *v_traj_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
  mxArray *P_traj_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

  double *p_traj = mxGetPr(traj_mx);
  double *p_v_traj = mxGetPr(v_traj_mx);
  double *p_P_traj = mxGetPr(P_traj_mx);

  Eigen::Map<Eigen::MatrixXd> map_traj(p_traj, 4, n_samples);
  map_traj = xs_p[0];

  Eigen::Map<Eigen::MatrixXd> map_v_traj(p_v_traj, 2, n_samples);
  map_v_traj = v_p[0];

  Eigen::Map<Eigen::MatrixXd> map_P_traj(p_P_traj, 16, n_samples);
  map_P_traj = P_p;

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engPutVariable(ep, "X", traj_mx);
  engPutVariable(ep, "v", v_traj_mx);
  engPutVariable(ep, "P_flat", P_traj_mx);
  engEvalString(ep, "test_mrou_plot");

  mxDestroyArray(traj_mx);
  mxDestroyArray(v_traj_mx);
  mxDestroyArray(P_traj_mx);
  engClose(ep);
}
#endif