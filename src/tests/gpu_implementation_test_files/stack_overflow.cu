
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>
#include <iostream>

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


namespace CML
{
    //template<class T, class Derived> class Matrix_Base {};
    template <class T, int MaxRows, int MaxCols>
    class SMatrix : public Matrix_Base<T, SMatrix<T, MaxRows, MaxCols>> 
    {
    private:
        size_t n_rows, n_cols; // should be less than MaxRows and MaxCols, respectively

        T data[MaxRows * MaxCols];

        __host__ __device__ void assign_data(const SMatrix &other)
        {
            n_rows = other.n_rows;
            n_cols = other.n_cols;
        
            for (size_t i = 0; i < n_rows; i++)
            {
                for (size_t j = 0; j < n_cols; j++)
                {
                    this->data[n_cols * i + j] = other.data[n_cols * i + j];
                }
            }
        }

    public:
        
        __host__ __device__ SMatrix() = default;

        __host__ __device__ SMatrix(const size_t n_rows, const size_t n_cols) :
        n_rows(n_rows), n_cols(n_cols) {}

        __host__ __device__ SMatrix& operator=(const SMatrix &rhs)
        {
             if (this == &rhs)
             {
                 return *this;
             }
             assign_data(rhs);
             return *this;
        }

        __host__ __device__ T& operator()(const size_t row, const size_t col)
        { 
            return data[n_cols * row + col]; 
        }

        __host__ __device__ void resize(const size_t n_rows, const size_t n_cols)
        {
              assert(n_rows > 0 && n_cols > 0 && n_rows <= MaxRows && n_cols <= MaxCols);
              this->n_rows = n_rows; this->n_cols = n_cols;
        }

        __host__ __device__ void init(const T val)
        {
            for (size_t i = 0; i < n_rows; i++)
            {
                for (size_t j = 0; j < n_cols; j++)
                {
                    data[n_rows * i + j] = val;
                }
            }
        }
    };
};

class My_Class
{
private:
    double m; 
    CML::SMatrix<double, 3, 1> mat;

public:
    
    __host__ __device__ My_Class() : mat(3, 1)
    { 
        m = 0;  
        mat.init(1.0);
    }

    __host__ __device__ My_Class(const size_t max_samples) : mat(3, 1)
    { 
        m = 0;  
        mat.init(1.0);
    }

    __device__ double calc() { return m + mat(0, 0); }
};


class myFunctor
{
private: 
    My_Class my_class;
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
        CPE cpe(CE, 1000, 10, 1, 0.5);

        cpe.seed_prng(cb_index);

        cpe.set_number_of_obstacles(3);
        
        double ret = (double)cb_index; 

        return ret; 
    }
};

int main()
{
    int n_cbs = 10;

    My_Class my_class;

    thrust::device_vector<double> cb_costs(n_cbs);

    // Version of transform using tuples to transfer class into thrust::transform
    /* thrust::device_vector<unsigned int> seq(n_cbs);
    thrust::sequence(seq.begin(), seq.end(), 0);
    for (int i = 0; i < n_cbs; i++)
    {
        std::cout << seq[i] << ", ";
    } 
    std::cout << std::endl;

    thrust::device_vector<My_Class> mcs(n_cbs);
    thrust::fill(mcs.begin(), mcs.end(), my_class);
    
    auto input_begin = 
        thrust::make_zip_iterator(thrust::make_tuple(seq.begin(), mcs.begin()));

    auto input_end = 
        thrust::make_zip_iterator(thrust::make_tuple(seq.end(), mcs.end()));
    
    thrust::transform(thrust::device, input_begin, input_end, cb_costs.begin(), myFunctor()); */

    thrust::counting_iterator<unsigned int> index_iter(0);

    thrust::transform(index_iter, index_iter + n_cbs, cb_costs.begin(), myFunctor(my_class));

    thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    int min_index = min_cost_iter - cb_costs.begin();
    double min_cost = cb_costs[min_index];

    std::cout << "min_index = " << min_index << "   |   min_cost = " << min_cost << std::endl;
    return 0;
}