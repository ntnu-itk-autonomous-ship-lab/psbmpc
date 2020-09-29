/****************************************************************************************
*
*  File name : pseudo_dynamic_matrix.cuh
*
*  Function  : Header file for the "dynamic" matrix class used in the Cuda Matrix Library,
*			   which has a compile-time set max number of rows and columns. Used in the
*			   data transfer part from the CPU to the GPU for PSB-MPC. Thus, it only acts
*              as a bridge between the CPU and GPU. Not meant to use for math operations.
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

#ifndef _PSEUDO_DYNAMIC_MATRIX_CUH_
#define _PSEUDO_DYNAMIC_MATRIX_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>

namespace CML
{
	template <class T, class Derived> class Matrix_Base;
	template <class T, size_t Rows, size_t Cols> class Static_Matrix;
	template <class T> class Dynamic_Matrix;

	template <class T, size_t Max_Rows, size_t Max_Cols>
	class Pseudo_Dynamic_Matrix : public Matrix_Base<T, Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>> 
	{
	private:
		size_t n_rows, n_cols; // should be less than MaxRows and MaxCols, respectively

		T data[Max_Rows * Max_Cols];

		__host__ __device__ void assign_data(const Pseudo_Dynamic_Matrix &other);

		template <class U>
		__host__ __device__ void assign_data(const Dynamic_Matrix<U> &other);

		template <class U, size_t Rows, size_t Cols>
		__host__ __device__ void assign_data(const Static_Matrix<U, Rows, Cols> &other);

	public:
		
		__host__ __device__ Pseudo_Dynamic_Matrix() : n_rows(0), n_cols(0) {}

		__host__ __device__ Pseudo_Dynamic_Matrix(const size_t n_rows) : n_rows(n_rows), n_cols(n_rows) {}

		__host__ __device__ Pseudo_Dynamic_Matrix(const size_t n_rows, const size_t n_cols) : n_rows(n_rows), n_cols(n_cols) {}

		__host__ __device__ Pseudo_Dynamic_Matrix(const Pseudo_Dynamic_Matrix &other) { assign_data(other); }

		template <class U>
		__host__ __device__ Pseudo_Dynamic_Matrix(const Dynamic_Matrix<U> &other) { assign_data(other); }

		template <class U, size_t Rows, size_t Cols>
		__host__ __device__ Pseudo_Dynamic_Matrix(const Static_Matrix<U, Rows, Cols> &other) { assign_data(other); }

		__host__ __device__ Pseudo_Dynamic_Matrix& operator=(const Pseudo_Dynamic_Matrix &rhs);

		template<class U>
		__host__ __device__ Pseudo_Dynamic_Matrix& operator=(const Dynamic_Matrix<U> &rhs);

		template<class U, size_t Rows, size_t Cols>
		__host__ __device__ Pseudo_Dynamic_Matrix& operator=(const Static_Matrix<U, Rows, Cols> &rhs);

		template <class U>
		__host__ __device__ inline operator Dynamic_Matrix<U>() const
		{
			Dynamic_Matrix<U> result;
			result = *this;
			return result;
		}

		template<class U, size_t Rows, size_t Cols>
		__host__ __device__ operator Static_Matrix<U, Rows, Cols>() const
		{
			Static_Matrix<U, Rows, Cols> result;
			result = *this;
			return result;
		}

		__host__ __device__ inline size_t get_rows() const { return n_rows; }

		__host__ __device__ inline size_t get_cols() const { return n_cols; }

		__host__ __device__ inline T* get_data() { return data; }

		__host__ __device__ inline size_t size() const 
		{ 
			assert(n_rows == 1 || n_cols == 1); 
			if (n_rows > n_cols) 	{ return n_rows; } 
			else 					{ return n_cols; } 
		}

		__host__ __device__ inline void resize(const size_t n_rows, const size_t n_cols)
		{
			assert(n_rows <= Max_Rows && n_cols <= Max_Cols);

			this->n_rows = n_rows; this->n_cols = n_cols;
		}
		
	};

	/****************************************************************************************
	*  Class member functions
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : operator=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>& Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::operator=(
		const Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols> &rhs 				// In: Right hand side matrix/vector to assign
		)
	{
		if (this == &rhs)
		{
			return *this;
		}
		assign_data(rhs);
		
		return *this;
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	template<class U>
	__host__ __device__ Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>& Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::operator=(
		const Dynamic_Matrix<U> &rhs 											// In: Right hand side matrix/vector to assign
		)
	{
		assign_data(rhs);
		
		return *this;
	}

	/****************************************************************************************
		Private functions
	****************************************************************************************/
	/****************************************************************************************
	*  Name     : assign_data
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::assign_data(
		const Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols> &other 				// In: Matrix whose data to assign to *this;
		)
	{
		assert(other.n_rows <= Max_Rows && other.n_cols <= Max_Cols);
		n_rows = other.n_rows;
		n_cols = other.n_cols;
	
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				this->operator()(i, j) = other(i, j);
			}
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class U>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::assign_data(
		const Dynamic_Matrix<U> &other 																// In: Matrix whose data to assign to *this;
		)
	{
		assert(other.get_rows() <= Max_Rows && other.get_cols() <= Max_Cols);
		n_rows = other.get_rows();
		n_cols = other.get_cols();
	
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				this->operator()(i, j) = other(i, j);
			}
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class U, size_t Rows, size_t Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::assign_data(
		const Static_Matrix<U, Rows, Cols> &other 													// In: Matrix whose data to assign to *this;
		)
	{
		assert(Rows <= Max_Rows && Cols <= Max_Cols);
		n_rows = Rows;
		n_cols = Cols;
	
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				this->operator()(i, j) = other(i, j);
			}
		}
	}
	

	//=========================================================================================================
	// TYPEDEFS
	//=========================================================================================================

}

	

#endif