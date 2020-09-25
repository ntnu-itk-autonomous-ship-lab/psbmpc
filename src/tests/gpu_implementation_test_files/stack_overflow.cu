
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>
#include <iostream>

#include "ownship.cuh"


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
    CML::Static_Matrix<double, 3, 1> mat;

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

    /* __device__ double operator()(const thrust::tuple<unsigned int, My_Class> &input) 
    { 
        int cb = thrust::get<0>(input); 
        My_Class data = thrust::get<1>(input); 

        double ret = data.calc(); 

        return ret; 
    } */

    __device__ double operator()(const unsigned int cb_index) 
    { 
        
        /* CPE cpe(CE, 1000, 10, 1, 0.5);

        cpe.seed_prng(cb_index);

        cpe.set_number_of_obstacles(3); */
        
        double ret = (double)cb_index + ownship.get_length(); 

        return ret; 
    }
};

int main()
{
    int n_cbs = 1000000;

    My_Class my_class;

    thrust::device_vector<double> cb_costs(n_cbs);

    thrust::counting_iterator<unsigned int> index_iter(0);

    thrust::transform(index_iter, index_iter + n_cbs, cb_costs.begin(), myFunctor(my_class));

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;
    return 0;
}