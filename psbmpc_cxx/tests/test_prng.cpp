#include <iostream>
#include <random>
#include <chrono>
#include <Eigen/Dense>

#include "xoshiro.hpp"

int main(void) 
{
  double mean_t(0.0);
  int n_samples(100000), N_trials(1000);
  Eigen::MatrixXd samples(1, n_samples);
  // Xoroshiro256+ 64-bit test

  std::random_device seed;

  xoshiro256plus64 eng1(seed());

  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);
  
  mean_t = 0;
  for (int i = 0; i < N_trials; i++)
  {
    auto start = std::chrono::system_clock::now();


    for (int j = 0; j < n_samples; j++)
    {
      samples(0, j) = std_norm_pdf(eng1);
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= N_trials;

  std::cout << "xoshiro256+-64 time elapsed : " << mean_t << " milliseconds" << std::endl;

  // mersenne-twister 19937 test
  
  std::mt19937_64 eng2(seed());

  mean_t = 0;
  for (int i = 0; i < N_trials; i++)
  {
    auto start = std::chrono::system_clock::now();

    for (int j = 0; j < n_samples; j++)
    {
      samples(0, j) = std_norm_pdf(eng2);
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= N_trials;

  std::cout << "Mersenne-twister 19937-64 time elapsed : " << mean_t << " milliseconds" << std::endl;

  std::ranlux48_base eng3(seed());

  mean_t = 0;
  for (int i = 0; i < N_trials; i++)
  {
    auto start = std::chrono::system_clock::now();

    for (int j = 0; j < n_samples; j++)
    {
      samples(0, j) = std_norm_pdf(eng3);
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= N_trials;

  std::cout << "Subtract with carry 64 time elapsed : " << mean_t << " milliseconds" << std::endl;

  std::minstd_rand eng4(seed());

  mean_t = 0;
  for (int i = 0; i < N_trials; i++)
  {
    auto start = std::chrono::system_clock::now();

    for (int j = 0; j < n_samples; j++)
    {
      samples(0, j) = std_norm_pdf(eng4);
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= N_trials;

  std::cout << "Linear congruential time elapsed : " << mean_t << " milliseconds" << std::endl;
}