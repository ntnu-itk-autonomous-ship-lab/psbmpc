
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


class My_Class
{
private:
    int *ptr;
public:
    __host__ __device__ My_Class() { ptr = new int[2]; ptr[0] = 0; ptr[1] = 0; }

    __host__ __device__ ~My_Class() { delete[] ptr; }
};


class myFunctor
{
private: 

    //CB_Functor_Data *fdata;

    //int *obstacles;

    Ownship ownship;

public: 

    __host__ myFunctor() {} 

    /* __host__ myFunctor(const CB_Functor_Data &other) 
    {
        cudaMalloc((void**)&fdata, sizeof(CB_Functor_Data));

    } 

    __host__ __device__ ~myFunctor() { cudaFree(fdata); } */

   /*  __host__ myFunctor(const Obstacle_Data &odata, const int *in) 
    {
        cudaMalloc((void**)&obstacles, sizeof(int));
        cudaCheckErrors("cudaMalloc1 fail");

        cudaMemcpy(obstacles, in, 1, cudaMemcpyHostToDevice);
        cudaCheckErrors("cudaMemCpy1 fail");

        //cudaMalloc((void**)&obstacles, odata.obstacles.size() * sizeof(Cuda_Obstacle));

    } 
    __host__ ~myFunctor() { cudaFree(obstacles); cudaCheckErrors("cudaFree1 fail"); } */

    __device__ double operator()(const thrust::tuple<unsigned int, CML::Pseudo_Dynamic_Matrix<double, 20, 1>, My_Class> &input) 
    {
        int cb_index = thrust::get<0>(input); 
        CML::MatrixXd offset_sequence = thrust::get<1>(input);
        My_Class my_class = thrust::get<2>(input);

        CPE cpe(CE, 1000, 10, 1, 0.5);

        cpe.seed_prng(cb_index);

        cpe.set_number_of_obstacles(3); 

        double ret = offset_sequence(0) + offset_sequence(2) + ownship.get_length();

        return ret; 
    }
};

int main()
{
    int n_cbs = 1;

    //=================================================================================
    // CB_Functor_Data setup
    //=================================================================================
    /* Obstacle_Data odata; odata.obstacles.resize(1); 
    odata.ST_0.resize(1); odata.ST_i_0.resize(1);
    odata.AH_0.resize(1); odata.S_TC_0.resize(1);
    odata.S_i_TC_0.resize(1); odata.O_TC_0.resize(1); 
    odata.Q_TC_0.resize(1); odata.IP_0.resize(1);
    odata.H_TC_0.resize(1); odata.X_TC_0.resize(1); */

    /* int n_static_obst = 1;
	Eigen::Matrix<double, 4, -1> static_obstacles;
	static_obstacles.resize(4, n_static_obst);
    static_obstacles.col(0) << 500.0, 300.0, 1000.0, 50.0;

	Eigen::Matrix<double, 2, -1> waypoints;
    int n_wps_os = 2;
	waypoints.resize(2, n_wps_os); 
	waypoints << 0, 1000,
				 0, 0;
    
    PSBMPC psbmpc;

    CB_Functor_Data fdata(psbmpc, 2.0, 0.0, waypoints, static_obstacles, odata); */

    thrust::device_vector<double> cb_costs(n_cbs);

    thrust::device_vector<unsigned int> cb_index_dvec(n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

    thrust::device_vector<CML::Pseudo_Dynamic_Matrix<double, 20, 1>> control_behavior_dvec(n_cbs);
    Eigen::VectorXd constant_cb(4);
    constant_cb << 1, 0, 1, 0;

    CML::Pseudo_Dynamic_Matrix<double, 20, 1> constant_cml_cb(4, 1);
    constant_cml_cb(0) = 1; constant_cml_cb(1) = 0; constant_cml_cb(2) = 1; constant_cml_cb(3) = 0; 

    thrust::fill(control_behavior_dvec.begin(), control_behavior_dvec.end(), constant_cml_cb);

	auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), control_behavior_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), control_behavior_dvec.end()));

    int inp = 5;
    myFunctor mf(odata, &inp);

    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), mf);

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;
    return 0;
}