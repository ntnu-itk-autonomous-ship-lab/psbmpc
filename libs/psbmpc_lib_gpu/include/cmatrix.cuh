/****************************************************************************************
*
*  File name : cmatrix.cuh
*
*  Function  : Header file for the matrix (and vector container) used inside the Cuda 
*			   kernels for the PSB-MPC GPU calculations.
*
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

#ifndef _CMATRIX_H_
#define _CMATRIX_H_

#include <thrust/device_vector.h>

template<class T>
class CMatrix
{
private:

	size_t n_rows, n_cols;
	
	T **data;

	__host__ __device__ void allocate_data();

	__host__ __device__ void deallocate_data();

	__host__ __device__ T calculate_determinant_recursive(const CMatrix &sub_matrix, const size_t dim);

	__host__ __device__ void calculate_minor(CMatrix &minor, const size_t row, const size_t col);
	
public:

	__host__ __device__ CMatrix() {}

	__host__ __device__ CMatrix(const size_t n_rows);

	__host__ __device__ CMatrix(const size_t n_rows, const size_t n_cols);

	__host__ __device__ CMatrix(const CMatrix &other);

	__host__ __device__ ~CMatrix();

	__host__ __device__ CMatrix& operator=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator+(const CMatrix &other);

	__host__ __device__ CMatrix& operator+=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator-(const CMatrix &other);

	__host__ __device__ CMatrix& operator-=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator*(const CMatrix &other);
	__host__ __device__ CMatrix operator*(const T &factor);

	__host__ __device__ CMatrix operator/(const T &factor);
	__host__ __device__ CMatrix operator/=(const T &factor);

	__host__ __device__ bool operator==(const CMatrix &rhs);

	__host__ __device__ T& operator()(const size_t row, const size_t col) const { return data[row][col]; }

	__host__ __device__ CMatrix transpose();

	__host__ __device__ T determinant();

	__host__ __device__ CMatrix inverse();

	__host__ __device__ T dot(const CMatrix &other);

	__host__ __device__ CMatrix block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols);

	__host__ __device__ size_t rows() const { return n_rows; }

	__host__ __device__ size_t cols() const { return n_cols; }

	__host__ __device__ T** data() const { return data; }

	__host__ __device__ T max_coeff() const;

	__host__ __device__ T min_coeff() const;

	__host__ __device__ void resize(const size_t n_rows, const size_t n_cols);

	__host__ __device__ void conservative_resize(const size_t n_rows, const size_t n_cols);
};

#endif