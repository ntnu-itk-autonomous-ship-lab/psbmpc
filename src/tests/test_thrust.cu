/****************************************************************************************
*
*  File name : test_thrust.cpp
*
*  Function  : Test file for the thrust library functionality
*			   
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>
#include <functional>
#include <chrono>

#include "xoshiro.hpp"

struct psrngen
{
    __host__ __device__ psrngen(double _a, double _b) : a(_a), b(_b) {}
 
    __host__ __device__ double operator()(const unsigned int n) const
    {
        thrust::default_random_engine rng;
        thrust::normal_distribution<double> norm_dist(a, b);
        rng.discard(n);
        return norm_dist(rng);
    }
    float a, b;
 
};

double do_work(double a, double b, unsigned int n)
{
  thrust::default_random_engine rng;
  thrust::normal_distribution<double> norm_dist(a, b);
  rng.discard(n);
  return norm_dist(rng);
}


int main(void) 
{

  int N = 1000000, N_MC = 100;

  std::function<double(double, double, unsigned int)> f = do_work;

  std::random_device seed;
  xoshiro256plus64 engine(seed());

  std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

  Eigen::VectorXd data_seq(N);

  //===============================================================================================
  // Standard sequential approach
  //===============================================================================================

  double mean_t = 0;

  for (int m = 0; m < N_MC; m++)
  {
    auto start = std::chrono::system_clock::now();

    for (int i = 0; i < N; i++)
    {
      data_seq(i) = std_norm_pdf(engine);
      
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= (double)N_MC;

  std::cout << "Xoshiro256+-64 sequential PRNG time elapsed : " << mean_t << " milliseconds" << std::endl;

  //===============================================================================================
  // GPU approach
  //===============================================================================================
  std::uniform_int_distribution<unsigned int> unif_dist(0, 1000000);

  double seed_thrust = unif_dist(engine);
  thrust::device_vector<double> d_data(N); 
  thrust::host_vector<double> o_data(N); 

  mean_t = 0;
  for (int m = 0; m < N_MC; m++)
  {
    auto start = std::chrono::system_clock::now();

    thrust::counting_iterator<unsigned int> index_sequence_begin(seed_thrust);
    thrust::transform(thrust::device, index_sequence_begin, index_sequence_begin + (N), d_data.begin(), psrngen(0.0, 1.0));

    o_data = d_data;

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    mean_t  += elapsed.count();
  }
  mean_t /= (double)N_MC;

  std::cout << "GPU default PRNG time elapsed : " << mean_t << " milliseconds" << std::endl;

  double mean_seq(0), mean_gpu(0), std_dev_seq(0), std_dev_gpu(0);
  for (int i = 0; i < N; i++)
  {
    mean_seq += data_seq(i);
    mean_gpu += o_data[i];
  }
  mean_seq /= (double)N;
  mean_gpu /= (double)N;
  for (int i = 0; i < N; i++)
  {
    std_dev_seq += pow((data_seq(i) - mean_seq), 2);
    std_dev_gpu += pow((o_data[i] - mean_gpu), 2);
  }
  std_dev_seq /= (double)N;
  std_dev_gpu /= (double)N;
  std_dev_seq = sqrt(std_dev_seq);
  std_dev_gpu = sqrt(std_dev_gpu);

  std::cout << "mean SEQ = " << mean_seq << std::endl;
  std::cout << "std_dev SEQ = " << std_dev_seq << std::endl;
  std::cout << "mean GPU = " << mean_gpu << std::endl;
  std::cout << "std_dev GPU = " << std_dev_gpu << std::endl;

  //force swap to clear
  d_data.clear();
  thrust::device_vector<double>().swap(d_data);

  return 0;
}


