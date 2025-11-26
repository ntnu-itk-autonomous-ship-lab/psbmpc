#include "cpu/utilities_cpu.hpp"
#include "kf.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

class KFTest : public ::testing::Test {
protected:
  void SetUp() override {
    t_0 = 0.0;
    dt = 0.5;
    xs_i << 0, 0, 2, 2;
    y_m << 10, 0, 2, 1;
    P_i << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2;
    kf = std::make_unique<PSBMPC_LIB::KF>(xs_i, P_i, t_0, true);
    duration_lost = 0.0;
  }

  double t_0;
  double dt;
  double duration_lost;
  Eigen::Vector4d xs_i;
  Eigen::Vector4d y_m;
  Eigen::Matrix4d P_i;
  std::unique_ptr<PSBMPC_LIB::KF> kf;
};

TEST_F(KFTest, PredictAndUpdate) {
  kf->predict(dt);
  kf->update(y_m, duration_lost, dt);
  Eigen::Vector4d xs_i_upd = kf->get_state();

  EXPECT_GT(xs_i_upd(0), xs_i(0));
  EXPECT_GT(xs_i_upd(1), xs_i(1));

  Eigen::Matrix4d P_upd = kf->get_covariance();
  EXPECT_GT(P_upd.determinant(), 0.0);
}

TEST_F(KFTest, Reset) {
  kf->predict(dt);
  kf->update(y_m, duration_lost, dt);

  kf->reset(xs_i, P_i, t_0);
  Eigen::Vector4d xs_i_reset = kf->get_state();
  Eigen::Matrix4d P_reset = kf->get_covariance();

  EXPECT_NEAR((xs_i_reset - xs_i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((P_reset - P_i).norm(), 0.0, 1e-6);
}