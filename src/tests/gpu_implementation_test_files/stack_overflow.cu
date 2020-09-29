
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>
#include <iostream>
#include <stdio.h>

#include "obstacle_manager.cuh"
#include "cb_cost_functor_structures.cuh"
#include "ownship.cuh"
#include "cpe.cuh"
#include "utilities.h"


#define cudaCheckErrors(msg) \
    do { \
        cudaError_t __err = cudaGetLastError(); \
        if (__err != cudaSuccess) { \
            fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
                msg, cudaGetErrorString(__err), \
                __FILE__, __LINE__); \
            fprintf(stderr, "*** FAILED - ABORTING\n"); \
            exit(1); \
        } \
    } while (0)


class myFunctor
{
private: 

    //CB_Functor_Data *fdata;

    Cuda_Obstacle *obstacles;

    Ownship ownship;

public: 

    __host__ myFunctor() {} 

    __host__ myFunctor(Cuda_Obstacle *obstacles) : obstacles(obstacles) {}

    __host__ __device__ ~myFunctor() { obstacles = nullptr; }

    __device__ double operator()(const thrust::tuple<unsigned int, CML::Pseudo_Dynamic_Matrix<double, 20, 1>> &input) 
    {
        int cb_index = thrust::get<0>(input); 
        CML::MatrixXd offset_sequence = thrust::get<1>(input);

        CPE cpe(CE, 1000, 10, 1, 0.5);

        cpe.seed_prng(cb_index);

        cpe.set_number_of_obstacles(3); 

        double ret = offset_sequence(0) + offset_sequence(2) + obstacles[0].get_a_priori_CC_probability();

        return ret; 
    }
};

int main()
{
    int n_cbs = 1;

    //=================================================================================
    // Cuda_Obstacle setup
    //=================================================================================
    double phi_AH = 68.5 * DEG2RAD;		 	
	double phi_OT = 68.5 * DEG2RAD;		 		 
	double phi_HO = 22.5 * DEG2RAD;		 		
	double phi_CR = 68.5 * DEG2RAD;

    Eigen::VectorXd xs_aug(9), xs_os(6);
	xs_aug << 1000, 0, -5, 0, 5, 5, 5, 5, 0;

	xs_os << 0, 0, 0, 9, 0, 0;

	Eigen::MatrixXd P(4, 4);
	P << 100, 0, 0, 0,
	     0, 100, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	Eigen::VectorXd Pr_a(1);
	Pr_a << 1;
	Pr_a = Pr_a / Pr_a.sum();

	Tracked_Obstacle obstacle(xs_aug, flatten(P), Pr_a, 0.5, false, 1, 0.5);


    std::vector<Intention> ps_ordering(1); ps_ordering[0] = KCC;
    Eigen::VectorXd ps_course_changes(1), ps_maneuver_times(1), ps_weights(1); 
    ps_course_changes << 0; ps_maneuver_times << 0; ps_weights << 1;
    obstacle.initialize_prediction(ps_ordering, ps_course_changes, ps_weights, ps_maneuver_times);
    obstacle.predict_independent_trajectories(1, 0.5, xs_os, phi_AH, phi_CR, phi_HO, phi_OT, 500, 50);

    Cuda_Obstacle transfer = obstacle;
    Cuda_Obstacle *obstacle_ptr;
    CML::Pseudo_Dynamic_Matrix<double, 4, 2000> *xs_p;

    thrust::device_vector<double> cb_costs(n_cbs);

    thrust::device_vector<unsigned int> cb_index_dvec(n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

    thrust::device_vector<CML::Pseudo_Dynamic_Matrix<double, 20, 1>> control_behavior_dvec(n_cbs);
    Eigen::VectorXd constant_cb(4);
    constant_cb << 1, 0, 1, 0;

    CML::Pseudo_Dynamic_Matrix<double, 20, 1> constant_cml_cb(4, 1);
    constant_cml_cb(0) = 1; constant_cml_cb(1) = 0; constant_cml_cb(2) = 1; constant_cml_cb(3) = 0; 

    thrust::fill(control_behavior_dvec.begin(), control_behavior_dvec.end(), constant_cml_cb);
 
    cudaMalloc((void**)&obstacle_ptr, sizeof(Cuda_Obstacle));
    cudaCheckErrors("cudaMalloc1 fail");
    cudaMalloc((void**)&xs_p, transfer.get_num_prediction_scenarios() * sizeof(CML::Pseudo_Dynamic_Matrix<double, 4, 2000>));

    cudaMemcpy(obstacle_ptr, &transfer, 1, cudaMemcpyHostToDevice);
    cudaCheckErrors("cudaMemCpy1 fail");
    
    myFunctor mf(obstacle_ptr);

    auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), control_behavior_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), control_behavior_dvec.end()));

    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), mf);

    cudaFree(obstacle_ptr);
    cudaCheckErrors("cudaFree1 fail");

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;
    return 0;
}