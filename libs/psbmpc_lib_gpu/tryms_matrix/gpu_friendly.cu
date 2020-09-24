#include <cstdio>
#include <thrust/device_vector.h>
#include <vector>
 
struct dim2
{
    int x, y;
 
    dim2() = default;
 
    __host__ __device__
        dim2(int nrows, int ncols) : x(nrows), y(ncols) {};
 
    __host__ __device__
        bool operator== (const dim2& b) {
            return (x == b.x) && (y == b.y);
        };
};
 
template<typename T>
struct dynamic_matrix
{
    T* data;
    dim2 dims;
 
    dynamic_matrix() = default;
 
    __host__ __device__
        dynamic_matrix(T* p, int nrows, int ncols): data(p), dims(nrows, ncols) {};
 
    __host__ __device__
        T& operator() (int i, int j) { return data[i + j * dims.y]; }; 
 
    __host__ __device__
        const T& operator() (int i, int j) const { return data[i + j * dims.y]; };
 
    __host__ __device__
        dim2 shape() { return dims; };
 
    __host__ __device__
        void print() {
            dim2 d = this->shape();
            printf("Dimensions: %d x %d\n", d.x, d.y);
            for(int i=0; i<d.x; ++i)
                for(int j=0; j<d.y; ++j)
                    printf("%d %d %f\n", i, j, (*this)(i,j));
        };
 
};
 
template<typename T, int nrows, int ncols>
struct static_matrix
{
    T data[nrows * ncols];
 
    static_matrix() = default;
 
    __host__ __device__
    static_matrix(T val) { init(val); };
 
    __host__ __device__
    T& operator() (int i, int j) { return data[i + j * ncols]; }; 
 
    __host__ __device__
    const T& operator() (int i, int j) const { return data[i + j * ncols]; };
 
    __host__ __device__
    void init(T val) {
        for(int i=0; i<nrows; ++i)
            for(int j=0; j<ncols; ++j)
                (*this)(i,j) = val;
    };
 
    __host__ __device__
    dim2 shape() { return dim2(nrows,ncols); };
 
    __host__ __device__
    void print() {
        dim2 d = this->shape();
        printf("Dimensions: %d x %d\n", d.x, d.y);
        for(int i=0; i<d.x; ++i)
            for(int j=0; j<d.y; ++j)
                printf("%d %d %f\n", i, j, (*this)(i,j));
    };
 
};
 
template<typename T>
bool dot(dynamic_matrix<T> &A, dynamic_matrix<T> &B, dynamic_matrix<T> &C)
{
    dim2 Adim = A.shape();
    dim2 Bdim = B.shape();
 
    int m = Adim.x;
    int l = Adim.y;
    int n = Bdim.y;
 
    if (l != Bdim.x) return false;
    for(int i=0; i<m; ++i) {
        for(int j=0; j<n; ++j) {
            T result = T(0);
            for(int k=0; k<l; ++k) {
                result += A(i,k) * B(k,j);
            }
            C(i,j) = result;
        }
    }
    return true;
}
 
template<typename T, int m, int l, int n>
void dot(const static_matrix<T,m,l> &A, const static_matrix<T,l,n> &B, static_matrix<T,m,n> &C)
{
    for(int i=0; i<m; ++i) {
        for(int j=0; j<n; ++j) {
            T result = T(0);
            for(int k=0; k<l; ++k) {
                result += A(i,k) * B(k,j);
            }
            C(i,j) = result;
        }
    }
}
 
typedef static_matrix<float,3,3> fmat3x3;
typedef static_matrix<float,3,1> fmat3x1;
 
int main()
{
    printf("Static matrices\n");
    fmat3x3 A(1.f); 
    A(0,0) = 2.f; A(1,1) = 2.f; A(2,2) = 2.f;
    A.print();
 
    fmat3x1 b(1.f);
    b.print();
 
    fmat3x1 c(0.f);
    c.print();
 
    c = b;
    c.print();
 
    dot(A, b, c);
    c.print();
 
    printf("Dynamic matrices\n");
    thrust::device_vector<float> Av(9,1.f);
    dynamic_matrix<float> Ad(thrust::raw_pointer_cast<float>(Av),3,3);
    Ad(0,0) = 2.f; Ad(1,1) = 2.f; Ad(2,2) = 2.f;
    Ad.print();
 
    std::vector<float> Bv(3,1.f);
    dynamic_matrix<float> Bd(&Bv[0],3,1);
    Bd.print();
 
    std::vector<float> Cv(3,0.f);
    dynamic_matrix<float> Cd(&Cv[0],3,1);
    Cd.print();
 
    dot<float>(Ad, Bd, Cd);
    Cd.print();
 
    return 0;
}