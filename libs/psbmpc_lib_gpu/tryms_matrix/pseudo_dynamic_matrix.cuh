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

		__host__ __device__ inline operator Dynamic_Matrix<T>() const
		{
			Dynamic_Matrix<T> result;
			result = *this;
			return result;
		}

		template<size_t Rows, size_t Cols>
		__host__ __device__ operator Static_Matrix<T, Rows, Cols>() const
		{
			Static_Matrix<T, Rows, Cols> result;
			result = *this;
			return result;
		}

		__host__ __device__ void set_block(
			const size_t start_row, 
			const size_t start_col, 
			const size_t n_rows, 
			const size_t n_cols, 
			const Dynamic_Matrix<T> &block);

		__host__ __device__ void set_row(const size_t row, const Pseudo_Dynamic_Matrix<T, 1, Max_Cols> &vector);

		__host__ __device__ void set_row(const size_t row, const Dynamic_Matrix<T> &vector);

		template <size_t Cols>
		__host__ __device__ void set_row(const size_t row, const Static_Matrix<T, 1, Cols> &vector);

		__host__ __device__ void set_col(const size_t col, const Pseudo_Dynamic_Matrix<T, Max_Rows, 1> &vector);

		__host__ __device__ void set_col(const size_t col, const Dynamic_Matrix<T> &vector);

		template <size_t Rows>
		__host__ __device__ void set_col(const size_t col, const Static_Matrix<T, Rows, 1> &vector);

		template <size_t New_Max_Rows, size_t New_Max_Cols>
		__host__ __device__ Pseudo_Dynamic_Matrix<T, New_Max_Rows, New_Max_Cols> get_block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

		__host__ __device__ Pseudo_Dynamic_Matrix<T, 1, Max_Cols> get_row(const size_t row) const;

		__host__ __device__ Pseudo_Dynamic_Matrix<T, Max_Rows, 1> get_col(const size_t col) const;

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
	*  Name     : set_block
	*  Function : Sets the n_rows x n_cols block of this object, starting at 
	*			  (start_row, start_col).
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__  void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols, 										// In: Amount of columns
		const Dynamic_Matrix<T> &block 								// In: Block matrix to set
		)
	{
		assert(	n_rows <= Max_Rows && n_cols <= Max_Cols && 
				start_row < Max_Rows && start_col < Max_Cols);

		if (start_row + n_rows > this->n_rows || start_col + n_cols > this->n_cols)
		{
			resize(start_row + n_rows, start_col + n_cols);
		}

		assert(block.get_rows() == n_rows && block.get_cols() == n_cols);

		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				this->operator()(start_row + i, start_col + j) = block(i, j);
			}
		}
	}

	/****************************************************************************************
	*  Name     : set_row
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Pseudo_Dynamic_Matrix<T, 1, Max_Cols> &vector 				// In: Row vector to assign to the row
		)
	{
		assert(vector.get_cols() == this->n_cols && row < this->n_rows);
		for (size_t j = 0; j < this->n_cols; j++)
		{
			this->operator()(row, j) = vector(j);
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Dynamic_Matrix<T> &vector 									// In: Row vector to assign to the row
		)
	{
		assert(vector.get_cols() == this->n_cols && row < this->n_rows);
		for (size_t j = 0; j < this->n_cols; j++)
		{
			this->operator()(row, j) = vector(j);
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <size_t Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Static_Matrix<T, 1, Cols> &vector 							// In: Row vector to assign to the row
		)
	{
		assert(row < n_rows && Cols == n_cols);
		for (size_t j = 0; j < Cols; j++)
		{
			this->operator()(row, j) = vector(j);
		}
	}

	/****************************************************************************************
	*  Name     : set_col
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Pseudo_Dynamic_Matrix<T, Max_Rows, 1> &vector 				// In: Column vector to assign to the column
		)
	{
		assert(vector.get_rows() == this->n_rows && col < this->n_cols);
		for (size_t i = 0; i < this->n_rows; i++)
		{
			this->operator()(i, col) = vector(i);
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Dynamic_Matrix<T> &vector 									// In: Column vector to assign to the column
		)
	{
		assert(vector.get_rows() == this->n_rows && col < this->n_cols);
		for (size_t i = 0; i < this->n_rows; i++)
		{
			this->operator()(i, col) = vector(i);
		}
	}

	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <size_t Rows>
	__host__ __device__ void Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Static_Matrix<T, Rows, 1> &vector 							// In: Column vector to assign to the column
		)
	{
		assert(col < n_cols && Rows == n_rows);
		for (size_t i = 0; i < Rows; i++)
		{
			this->operator()(i, col) = vector(i);
		}
	}

	/****************************************************************************************
	*  Name     : get_block
	*  Function : returns a given block of this object, with upper left reference
	*			  index (start_row, start_col). Effective size of block is n_rows * n_cols,
	*			  but with max size New_Max_Rows * New_Max_Cols
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <size_t New_Max_Rows, size_t New_Max_Cols>
	__host__ __device__ Pseudo_Dynamic_Matrix<T, New_Max_Rows, New_Max_Cols> Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::get_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols 										// In: Amount of columns
		) const
	{
		assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && 
				start_row < this->n_rows && start_col < this->n_cols);

		Pseudo_Dynamic_Matrix<T, New_Max_Rows, New_Max_Cols> result(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				result(i, j) = this->operator()(start_row + i, start_col + j);
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : get_row
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ Pseudo_Dynamic_Matrix<T, 1, Max_Cols> Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::get_row(
		const size_t row											// In: Index of row to fetch
		) const
	{
		assert(row < this->n_rows);

		Pseudo_Dynamic_Matrix<T, 1, Max_Cols> result(1, n_cols);
		for (size_t j = 0; j < n_cols; j++)
		{
			result(0, j) = this->operator()(row, j);
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : get_col
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ Pseudo_Dynamic_Matrix<T, Max_Rows, 1> Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols>::get_col(
		const size_t col											// In: Index of row to fetch
		) const
	{
		assert(col < this->n_cols);

		Pseudo_Dynamic_Matrix<T, Max_Rows, 1> result(n_rows, 1);
		for (size_t i = 0; i < n_rows; i++)
		{
			result(i, 0) = this->operator()(i, col);
		}
		return result;
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