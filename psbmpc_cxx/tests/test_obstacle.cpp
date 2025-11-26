#include "sbmpc.hpp"
#include "tracked_obstacle.hpp"
#if OWNSHIP_TYPE == 0
#include "cpu/kinematic_ship_models_cpu.hpp"
#else
#include "cpu/kinetic_ship_models_cpu.hpp"
#endif
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#define BUFSIZE 1000000
#endif

class ObstacleTest : public ::testing::Test {
protected:
  void SetUp() override {
    xs_os_0 << 0, 0, 0, 6, 0, 0;
    T = 200.0;
    dt = 0.5;
    n_samples = std::round(T / dt);
    u_d = 6.0;
    chi_d = 0.0;

    offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1,
        0 * M_PI / 180.0;
    maneuver_times << 0, 100, 150;

#if OWNSHIP_TYPE == 0
    trajectory.resize(4, n_samples);
    trajectory.col(0) = xs_os_0.block<4, 1>(0, 0);
#else
    trajectory.resize(6, n_samples);
    trajectory.col(0) = xs_os_0;
#endif

    waypoints.resize(2, 2);
    waypoints << 0, 1000, 0, 0;

    ownship = std::make_unique<PSBMPC_LIB::CPU::Ownship>();

    xs_aug << 1000, 0, -5, 0, 5, 5, 5, 5, 0;
    P << 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0.025;

    Pr_s << 1, 1, 1;
    Pr_s = Pr_s / Pr_s.sum();

    Pr_CCEM = 0.9;
    filter_on = true;

    obstacle = std::make_unique<PSBMPC_LIB::Tracked_Obstacle>(
        xs_aug, PSBMPC_LIB::CPU::flatten(P), Pr_s, filter_on, T, dt);
  }

  Eigen::Matrix<double, 6, 1> xs_os_0;
  double T;
  double dt;
  int n_samples;
  double u_d;
  double chi_d;
  Eigen::VectorXd offset_sequence{6};
  Eigen::Vector3d maneuver_times;
  Eigen::MatrixXd trajectory;
  Eigen::Matrix<double, 2, -1> waypoints;
  std::unique_ptr<PSBMPC_LIB::CPU::Ownship> ownship;

  Eigen::VectorXd xs_aug{9};
  Eigen::MatrixXd P{4, 4};
  Eigen::VectorXd Pr_s{3};
  double Pr_CCEM;
  bool filter_on;
  std::unique_ptr<PSBMPC_LIB::Tracked_Obstacle> obstacle;
};

TEST_F(ObstacleTest, Update) {
  ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d,
                              chi_d, waypoints, PSBMPC_LIB::ERK1,
                              PSBMPC_LIB::LOS, T, dt);

  Eigen::VectorXd xs_aug_new(9);
  xs_aug_new << 900, 0, -6, 0, 5, 5, 5, 5, 0;

  Eigen::MatrixXd P_new(4, 4);
  P_new << 125, 0, 0, 0, 0, 125, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0.05;

  Eigen::VectorXd Pr_s_new(3);
  Pr_s_new << 0.1, 0.5, 0.4;

  obstacle->increment_duration_tracked(dt);
  double duration_before = obstacle->get_duration_tracked();

  obstacle->update(xs_aug_new, PSBMPC_LIB::CPU::flatten(P_new), Pr_s_new,
                   !filter_on, dt);

  EXPECT_GE(obstacle->get_duration_tracked(), duration_before);
  EXPECT_EQ(obstacle->get_Pr_CCEM(), 0.9);

  Eigen::VectorXd Pr_s_result = obstacle->get_scenario_probabilities();
  std::cout << Pr_s_result << std::endl;
  std::cout << Pr_s_new << std::endl;
  EXPECT_TRUE(Pr_s_result.isApprox(Pr_s_new, 1e-6));
}
