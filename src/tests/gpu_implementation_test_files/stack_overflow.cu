

#include "cb_cost_functor_structures.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/iterator/counting_iterator.h>

#include <assert.h>

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
    template<class T, class Derived> class Matrix_Base{

    };

    template <class T>
    class Dynamic_Matrix : public Matrix_Base<T, Dynamic_Matrix<T>> 
    {
    private:
        size_t n_rows, n_cols;

        T* data;

        bool is_allocated;

        __host__ __device__ void allocate_data()
        {
              assert(n_rows > 0 && n_cols > 0);
              data = new T[n_rows * n_cols];
              is_allocated = true;
        }

        __host__ __device__ void deallocate_data() { delete[] data; data = nullptr; is_allocated = false; }

        __host__ __device__ void assign_data(const Dynamic_Matrix &other)
        {
             if (!other.is_allocated){ return; }

             n_rows = other.n_rows;
             n_cols = other.n_cols;

             if (!is_allocated){ allocate_data(); }
        
             for (size_t i = 0; i < n_rows; i++)
             {
                  for (size_t j = 0; j < n_cols; j++)
                  {
                        this->data[n_cols * i + j] = other.data[n_cols * i + j];
                  }
             }
        }

    public:
        
        __host__ __device__ Dynamic_Matrix() : n_rows(0), n_cols(0), data(nullptr), is_allocated(false) {}

        __host__ __device__ Dynamic_Matrix(const size_t n_rows, const size_t n_cols) :
        n_rows(n_rows), n_cols(n_cols), data(nullptr), is_allocated(false)
        {
            allocate_data();
        }

        __host__ __device__ Dynamic_Matrix(const Dynamic_Matrix &other):
        data(nullptr), is_allocated(false)
        {
            assign_data(other);
        }

        __host__ __device__ ~Dynamic_Matrix() { deallocate_data(); }

        __host__ __device__ Dynamic_Matrix& operator=(const Dynamic_Matrix &rhs)
        {
             if (this == &rhs)
             {
                 return *this;
             }
             deallocate_data(); 
             assign_data(rhs);
             return *this;
        }

        __host__ __device__ void resize(const size_t n_rows, const size_t n_cols)
        {
              assert(n_rows > 0 && n_cols > 0);
              *this = Dynamic_Matrix<T>(n_rows, n_cols);
        }
    };

    using MatrixXd = Dynamic_Matrix<double>;
};

class My_Class
{
private:

    CML::MatrixXd mat1;

public:
    __host__ __device__ My_Class() { mat1.resize(3, 1); }

    __device__ double calc() { return 2 + 2; }
};


class myFunctor
{
private: 
    //My_Class* my_class;
public: 
    __host__ __device__ myFunctor() {} 

    __host__ __device__ myFunctor(const My_Class &other) 
    {
         //gpuErrchk( cudaMalloc((void **)&my_class, sizeof(My_Class)) );
    }

    //__host__ __device__ ~myFunctor() { gpuErrchk( cudaFree(my_class) ); }

    __device__ double operator()(const unsigned int n, const double cb, const CB_Functor_Data fdata) 
    { 
        double ret = fdata.chi_d; 

        return ret; 
    }
};

int main()
{
    int n_cbs = 10;
    
    CB_Functor_Data fdata;

    thrust::device_vector<unsigned int> seq(n_cbs);
    thrust::sequence(seq.begin(), seq.end());
    thrust::device_vector<double> cb_costs(n_cbs);
    thrust::device_vector<CB_Functor_Data> cb_fdata(n_cbs);
    
    thrust::zip_iterator<thrust::tuple<unsigned int, CB_Functor_Data>> input = 
        thrust::make_zip_iterator(thrust::make_tuple(seq.begin(), cb_fdata.begin()));
    
    thrust::transform(thrust::device, input, input + n_cbs, cb_costs.begin(), myFunctor());

    //thrust::device_vector<double>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
    //int min_index = min_cost_iter - cb_costs.begin();
    //int min_cost = cb_costs[min_index];
    return 0;
}