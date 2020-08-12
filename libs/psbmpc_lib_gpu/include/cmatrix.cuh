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

#include "Eigen/Dense"
#include <thrust/device_vector.h>

template<class T>
class CMatrix
{
private:

	int n_rows, n_cols;
	
	T **data;

	__host__ __device__ void allocate_data();

	__host__ __device__ void deallocate_data();
	
public:

	__host__ __device__ CMatrix() {}

	__host__ __device__ CMatrix(const int n_rows, const int n_cols);

	__host__ __device__ CMatrix(const CMatrix &other);

	__host__ __device__ ~CMatrix();

	__host__ __device__ CMatrix& operator=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator+(const CMatrix &other);

	__host__ __device__ CMatrix operator-(const CMatrix &other);

	__host__ __device__ CMatrix operator*(const CMatrix &other);
	__host__ __device__ CMatrix operator*(const T &factor);

	__host__ __device__ bool operator==(const CMatrix &rhs);

	__host__ __device__ CMatrix transpose();

	__host__ __device__ T dot(const CMatrix &other);

	__host__ __device__ CMatrix block(const int start_row, const int start_col, const int n_rows, const int n_cols);

	__host__ __device__ T& operator()(const int row, const int col) const { return data[row][col]; }

	__host__ __device__ int rows() const { return n_rows; }

	__host__ __device__ int cols() const { return n_cols; }

	__host__ __device__ T** data() { return data; }

	__host__ __device__ void resize(const int n_rows, const int n_cols);

	__host__ __device__ void conservative_resize(const int n_rows, const int n_cols);
};

#endif