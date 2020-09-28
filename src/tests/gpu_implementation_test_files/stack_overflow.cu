
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>
#include <iostream>

#include "ownship.cuh"
#include "cpe.cuh"


#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

class My_Class
{
private:
    double m; 
    CML::Vector3d mat;

public:
    
    __host__ __device__ My_Class()
    { 
        m = 0;  
        mat.set_zero();
    }

    __host__ __device__ double calc() { return m + mat(0, 0); }
};


class myFunctor
{
private: 
    My_Class my_class;

    Ownship ownship;

public: 

    __host__ __device__ myFunctor() {} 

    __host__ __device__ myFunctor(const My_Class &other) 
    {
        my_class = other;
    }

    __device__ double operator()(const thrust::tuple<unsigned int, CML::Pseudo_Dynamic_Matrix<double, 20, 1>> &input) 
    {
        CML::MatrixXd offset_sequence = thrust::get<1>(input);
        int cb_index = thrust::get<0>(input);  

        CPE cpe(CE, 1000, 10, 1, 0.5);

        //cpe.seed_prng(cb_index);

        //cpe.set_number_of_obstacles(3); 

        double ret = offset_sequence(0) + offset_sequence(2) + ownship.get_length();

        return ret; 
    }
};

int main()
{
    int n_cbs = 1;

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

    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), myFunctor());

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;
    return 0;
}