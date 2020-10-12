
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>
#include <iostream>
#include <stdio.h>

#include "cb_cost_functor_structures.cuh"
#include "psbmpc.cuh"
#include "utilities.h"


class myFunctor
{
private: 

    CB_Functor_Data *fdata;

    Cuda_Obstacle *obstacles;

    Ownship ownship;

public: 

    __host__ myFunctor() {} 

    __host__ myFunctor(CB_Functor_Data *fdata, Cuda_Obstacle *obstacles) : fdata(fdata), obstacles(obstacles) {}

    template <class T>
    __device__ void print(T container)
    {
        int rows = container.get_rows(); 
        int cols = container.get_cols();

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                printf("%f", container(i, j));
                if (j < cols - 1) printf(", ");
            }
            if (i < rows - 1) printf("\n");
        }
    }

    __device__ double operator()(const thrust::tuple<unsigned int, TML::PDMatrix<double, 20, 1>> &input) 
    {
        int cb_index = thrust::get<0>(input); 
        TML::MatrixXd offset_sequence = thrust::get<1>(input);

        CPE cpe(CE, 0.5);

        cpe.seed_prng(cb_index);

        TML::MatrixXd xs_p = obstacles[0].get_ps_trajectory(0);

        # if __CUDA_ARCH__>=200
            //printf("xs_p size: %lu, %lu \n", xs_p.get_rows(), xs_p.get_cols());
        #endif

        TML::Vector4d xs = xs_p.get_block(0, 1, 4, 1);

        double ret = offset_sequence(0) + offset_sequence(2) + fdata->n_obst + xs(0);

        return ret; 
    }
};

int main()
{
    int n_cbs = 2;

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

    //=================================================================================
    // CB_Functor_Data setup
    //=================================================================================
    PSBMPC psbmpc;

	Eigen::Matrix<double, 2, -1> waypoints;

	int n_wps_os = 2;
	waypoints.resize(2, n_wps_os); 
	waypoints << 0, 1000,
				 0, 0;

    int n_static_obst = 1;
	Eigen::Matrix<double, 4, -1> static_obstacles;
	static_obstacles.resize(4, n_static_obst);
    static_obstacles.col(0) << 500.0, 300.0, 1000.0, 50.0;

    Obstacle_Data odata;
    odata.AH_0.resize(1); odata.AH_0[0] = false;
    odata.S_TC_0.resize(1); odata.S_TC_0[0] = false;
    odata.S_i_TC_0.resize(1); odata.S_i_TC_0[0] = false;
    odata.O_TC_0.resize(1); odata.O_TC_0[0] = false;
    odata.Q_TC_0.resize(1); odata.Q_TC_0[0] = false;
    odata.IP_0.resize(1); odata.IP_0[0] = true;
    odata.H_TC_0.resize(1); odata.H_TC_0[0] = false;
    odata.X_TC_0.resize(1); odata.X_TC_0[0] = false;
    odata.ST_0.resize(1); odata.ST_0[0] = B;
    odata.ST_i_0.resize(1); odata.ST_i_0[0] = C;

    odata.obstacles.resize(1); odata.obstacles[0] = obstacle;

    CB_Functor_Data fdata_obj(psbmpc, 5.0, 0.0, waypoints, static_obstacles, odata);

    //=================================================================================
    // Actual cuda n thrust stuff
    //=================================================================================
    CB_Functor_Data *fdata;

    cudaMalloc((void**)&fdata, sizeof(CB_Functor_Data)); 
    cuda_check_errors("cudaMalloc1 fail");

    cudaMemcpy(fdata, &fdata_obj, sizeof(CB_Functor_Data), cudaMemcpyHostToDevice);
    cuda_check_errors("cudaMemCpy1 fail");

    Cuda_Obstacle *obstacles;
    Cuda_Obstacle transfer = obstacle;
    Cuda_Obstacle back;

    cudaMalloc((void**)&obstacles, sizeof(Cuda_Obstacle));
    cuda_check_errors("cudaMalloc2 fail");

    cudaMemcpy(obstacles, &transfer, sizeof(Cuda_Obstacle), cudaMemcpyHostToDevice);
    cuda_check_errors("cudaMemCpy2 fail");

    thrust::device_vector<double> cb_costs(n_cbs);

    thrust::device_vector<unsigned int> cb_index_dvec(n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

    thrust::device_vector<TML::PDMatrix<double, 20, 1>> control_behavior_dvec(n_cbs);
    Eigen::VectorXd constant_cb(4);
    constant_cb << 1, 0, 1, 0;

    TML::PDMatrix<double, 20, 1> constant_TML_cb(4, 1);
    constant_TML_cb(0) = 1; constant_TML_cb(1) = 0; constant_TML_cb(2) = 1; constant_TML_cb(3) = 0; 

    thrust::fill(control_behavior_dvec.begin(), control_behavior_dvec.end(), constant_TML_cb);
 
    myFunctor mf(fdata, obstacles);

    auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), control_behavior_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), control_behavior_dvec.end()));

    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), mf);

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;

    cudaFree(fdata); 
    cuda_check_errors("cudaFree1 fail");

    cudaFree(obstacles);
    cuda_check_errors("cudaFree2 fail");

    return 0;
}