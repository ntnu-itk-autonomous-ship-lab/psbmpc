#include "xoshiro.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <gtest/gtest.h>
#include <random>

class PRNGTest : public ::testing::Test {
protected:
  void SetUp() override {
    n_samples = 100000;
    N_trials = 100;
    samples.resize(1, n_samples);
    seed = std::random_device()();
  }

  int n_samples;
  int N_trials;
  unsigned int seed;
  Eigen::MatrixXd samples;
};

TEST_F(PRNGTest, Xoshiro256Plus64) {
  xoshiro256plus64 eng(seed);
  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

  double mean_t = 0.0;
  for (int i = 0; i < N_trials; i++) {
    auto start = std::chrono::system_clock::now();
    for (int j = 0; j < n_samples; j++) {
      samples(0, j) = std_norm_pdf(eng);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    mean_t += elapsed.count();
  }
  mean_t /= N_trials;

  double sample_mean = samples.mean();
  double sample_std =
      std::sqrt((samples.array() - sample_mean).square().mean());

  EXPECT_NEAR(sample_mean, 0.0, 0.1);
  EXPECT_NEAR(sample_std, 1.0, 0.1);
  EXPECT_GE(mean_t, 0.0);
}

TEST_F(PRNGTest, MersenneTwister19937) {
  std::mt19937_64 eng(seed);
  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

  double mean_t = 0.0;
  for (int i = 0; i < N_trials; i++) {
    auto start = std::chrono::system_clock::now();
    for (int j = 0; j < n_samples; j++) {
      samples(0, j) = std_norm_pdf(eng);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    mean_t += elapsed.count();
  }
  mean_t /= N_trials;

  double sample_mean = samples.mean();
  double sample_std =
      std::sqrt((samples.array() - sample_mean).square().mean());

  EXPECT_NEAR(sample_mean, 0.0, 0.1);
  EXPECT_NEAR(sample_std, 1.0, 0.1);
  EXPECT_GE(mean_t, 0.0);
}

TEST_F(PRNGTest, Ranlux48Base) {
  std::ranlux48_base eng(seed);
  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

  double mean_t = 0.0;
  for (int i = 0; i < N_trials; i++) {
    auto start = std::chrono::system_clock::now();
    for (int j = 0; j < n_samples; j++) {
      samples(0, j) = std_norm_pdf(eng);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    mean_t += elapsed.count();
  }
  mean_t /= N_trials;

  double sample_mean = samples.mean();
  double sample_std =
      std::sqrt((samples.array() - sample_mean).square().mean());

  EXPECT_NEAR(sample_mean, 0.0, 0.1);
  EXPECT_NEAR(sample_std, 1.0, 0.1);
  EXPECT_GE(mean_t, 0.0);
}

TEST_F(PRNGTest, MinstdRand) {
  std::minstd_rand eng(seed);
  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

  double mean_t = 0.0;
  for (int i = 0; i < N_trials; i++) {
    auto start = std::chrono::system_clock::now();
    for (int j = 0; j < n_samples; j++) {
      samples(0, j) = std_norm_pdf(eng);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    mean_t += elapsed.count();
  }
  mean_t /= N_trials;

  double sample_mean = samples.mean();
  double sample_std =
      std::sqrt((samples.array() - sample_mean).square().mean());

  EXPECT_NEAR(sample_mean, 0.0, 0.1);
  EXPECT_NEAR(sample_std, 1.0, 0.1);
  EXPECT_GE(mean_t, 0.0);
}