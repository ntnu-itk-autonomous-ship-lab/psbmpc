/****************************************************************************************
*
*  File name : pseudo_dynamic_matrix.cuh
*
*  Function  : Header file for the "dynamic" matrix class used in the Cuda Matrix Library,
*			   which has a compile-time set max number of rows and columns.
*			   Abbreviated as PDMatrix
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
	class PDMatrix : public Matrix_Base<T, PDMatrix<T, Max_Rows, Max_Cols>> 
	{
	private:
		size_t n_rows, n_cols; // should be less than or equal Max_Rows and Max_Cols, respectively

		T data[Max_Rows * Max_Cols];
		
		__host__ __device__ void assign_data(const PDMatrix &other);

		template <class U>
		__host__ __device__ void assign_data(const Dynamic_Matrix<U> &other);

		template <class U, size_t Rows, size_t Cols>
		__host__ __device__ void assign_data(const Static_Matrix<U, Rows, Cols> &other);

	public:
		
		__host__ __device__ PDMatrix() : n_rows(0), n_cols(0) {}

		__host__ __device__ PDMatrix(const size_t n_rows) : n_rows(n_rows), n_cols(n_rows) {}

		__host__ __device__ PDMatrix(const size_t n_rows, const size_t n_cols) : n_rows(n_rows), n_cols(n_cols) {}

		__host__ __device__ PDMatrix(const PDMatrix &other) { assign_data(other); }

		template <class U>
		__host__ __device__ PDMatrix(const Dynamic_Matrix<U> &other) { assign_data(other); }

		template <class U, size_t Rows, size_t Cols>
		__host__ __device__ PDMatrix(const Static_Matrix<U, Rows, Cols> &other) { assign_data(other); }

		__host__ __device__ PDMatrix& operator=(const PDMatrix &rhs){ if (this == &rhs) { return *this; } assign_data(rhs); return *this; }

		template<class U>
		__host__ __device__ PDMatrix& operator=(const Dynamic_Matrix<U> &rhs) { assign_data(rhs); return *this; }

		template<class U, size_t Rows, size_t Cols>
		__host__ __device__ PDMatrix& operator=(const Static_Matrix<U, Rows, Cols> &rhs) { assign_data(rhs); return *this; }

		template<class U, size_t New_Max_Rows, size_t New_Max_Cols>
		__host__ __device__ inline operator PDMatrix<U, New_Max_Rows, New_Max_Cols>() const
		{
			PDMatrix<U, New_Max_Rows, New_Max_Cols> result;
			result = *this;
			return result;
		}

		template <class U>
		__host__ __device__ inline operator Dynamic_Matrix<U>() const
		{
			Dynamic_Matrix<U> result;
			result = *this;
			return result;
		}

		template<class U, size_t Rows, size_t Cols>
		__host__ __device__ inline operator Static_Matrix<U, Rows, Cols>() const
		{
			Static_Matrix<U, Rows, Cols> result;
			result = *this;
			return result;
		}

		template <class Matrix_Type>
		__host__ __device__ PDMatrix operator+(const Matrix_Type &other) const;

		template <class Matrix_Type>
		__host__ __device__ PDMatrix& operator+=(const Matrix_Type &rhs);

		template <class Matrix_Type>
		__host__ __device__ PDMatrix operator-(const Matrix_Type &other) const;

		template <class Matrix_Type>
		__host__ __device__ PDMatrix& operator-=(const Matrix_Type &rhs);

		template <class Matrix_Type>
		__host__ __device__ PDMatrix operator*(const Matrix_Type &other) const;

		__host__ __device__ void transpose();

		__host__ __device__ PDMatrix<T, Max_Cols, Max_Rows> transposed() const;

		template <class Matrix_Type>
		__host__ __device__ Matrix_Type cross(const Matrix_Type &other) const;

		template <class Matrix_Type>
		__host__ __device__ PDMatrix cwise_product(const Matrix_Type &other) const;
		
		__host__ __device__ PDMatrix cwise_mean() const;

		__host__ __device__ PDMatrix rwise_mean() const;

		__host__ __device__ static PDMatrix identity(const size_t n_rows, const size_t n_cols);

		__host__ __device__ static PDMatrix ones(const size_t n_rows, const size_t n_cols);

		template <class Matrix_Type>
		__host__ __device__ void set_block(
			const size_t start_row, 
			const size_t start_col, 
			const size_t n_rows, 
			const size_t n_cols, 
			const Matrix_Type &block);

		template <class Matrix_Type>
		__host__ __device__ void set_row(const size_t row, const Matrix_Type &vector);

		template <class Matrix_Type>
		__host__ __device__ void set_col(const size_t col, const Matrix_Type &vector);

		__host__ __device__ void shift_columns_right();

		template <size_t New_Max_Rows, size_t New_Max_Cols>
		__host__ __device__ PDMatrix<T, New_Max_Rows, New_Max_Cols> get_block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

		__host__ __device__ PDMatrix<T, 1, Max_Cols> get_row(const size_t row) const;

		__host__ __device__ PDMatrix<T, Max_Rows, 1> get_col(const size_t col) const;

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
	*  Global operator functions, to allow for commutativeness
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : operator+ (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator+(const PDMatrix<T, Max_Rows, Max_Cols> &other, const T scalar)
	{
		PDMatrix<T, Max_Rows, Max_Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) += scalar;
			}
		}
		return result;
	}
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator+(const T scalar, const PDMatrix<T, Max_Rows, Max_Cols> &other)	{ return other + scalar; }

	/****************************************************************************************
	*  Name     : operator- (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator-(const PDMatrix<T, Max_Rows, Max_Cols> &other, const T scalar)
	{
		PDMatrix<T, Max_Rows, Max_Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) -= scalar;
			}
		}
		return result;
	}
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator-(const T scalar, const PDMatrix<T, Max_Rows, Max_Cols> &other)	{ return (T)-1 * other + scalar; }

	/****************************************************************************************
	*  Name     : operator* (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator*(const PDMatrix<T, Max_Rows, Max_Cols> &other, const T scalar)
	{
		PDMatrix<T, Max_Rows, Max_Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols(); j++)
			{
				result(i, j) *= scalar;
			}
		}
		return result;
	}
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ inline PDMatrix<T, Max_Rows, Max_Cols> operator*(const T scalar, const PDMatrix<T, Max_Rows, Max_Cols> &other)	{ return other * scalar; }

	/****************************************************************************************
	*  Class member functions
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : operator+
	*  Function : Can add matrices of equal dimension, or row/column vectorts to the lhs, 
	*			  given that the dimension matches
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::operator+(
		const Matrix_Type &other 									// In: Matrix/vector to add by
		) const
	{
		if (n_rows < other.get_rows() || n_cols < other.get_cols())
		{
			return other + *this;
		}

		assert((n_rows == other.get_rows() && n_cols == other.get_cols()) 	|| 
				(n_rows == other.get_rows() && other.get_cols() == 1) 		||
				(n_cols == other.get_cols() && other.get_rows() == 1));

		PDMatrix<T, Max_Rows, Max_Cols> result = *this;
		
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols ; j++)
			{
				if (other.get_rows() == 1)
				{
					result(i, j) += other(0, j);
				} 
				else if(other.get_cols() == 1)
				{
					result(i, j) += other(i, 0);
				} 
				else
				{
					result(i, j) += other(i, j);
				}
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : operator+=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols>& PDMatrix<T, Max_Rows, Max_Cols>::operator+=(
		const Matrix_Type &rhs 										// In: Right hand side matrix/vector to add by
		)
	{
		assert(n_rows == rhs.get_rows() && n_cols == rhs.get_cols());
		
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				this->operator()(i, j) += rhs(i, j);
			}
		}
		return *this;
	}

	/****************************************************************************************
	*  Name     : operator-
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::operator-(
		const Matrix_Type &other 									// In: Matrix/vector to subtract by
		) const
	{
		if (n_rows < other.get_rows() || n_cols < other.get_cols())
		{
			return other - self;
		}

		assert((n_rows == other.get_rows() && n_cols == other.get_cols()) 	|| 
				(n_rows == other.get_rows() && other.get_cols() == 1) 					||
				(n_cols == other.get_cols() && other.get_rows() == 1));

		PDMatrix<T, Max_Rows, Max_Cols> result = *this;

		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols ; j++)
			{
				if (other.get_rows() == 1)
				{
					result(i, j) -= other(0, j);
				} 
				else if(other.get_cols() == 1)
				{
					result(i, j) -= other(i, 0);
				} 
				else
				{
					result(i, j) -= other(i, j);
				}
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : operator-=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols>& PDMatrix<T, Max_Rows, Max_Cols>::operator-=(
		const Matrix_Type &rhs 										// In: Right hand side matrix/vector to subtract by
		)
	{
		assert(n_rows == rhs.get_rows());
		assert(n_cols == rhs.get_cols());

		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols ; j++)
			{
				this->operator()(i, j) -= rhs(i, j);
			}
		}
		return *this;
	}

	/****************************************************************************************
	*  Name     : operator*
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::operator*(
		const Matrix_Type &other 									// In: Matrix/vector to multiply with
		) const
	{	
		// Verify that the matrix product is valid
		assert(n_cols == other.get_rows() && Max_Cols >= other.get_cols());

		PDMatrix<T, Max_Rows, Max_Cols> result(n_rows, other.get_cols());
		for (size_t i = 0 ; i < n_rows; i++)
		{
			for (size_t j = 0; j < other.get_cols(); j++)
			{
				result(i, j) = (T)0;
				for (size_t k = 0; k < n_cols; k++)
				{
					result(i, j) += this->operator()(i, k) * other(k, j);
				}
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : transpose
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::transpose()
	{
		PDMatrix<T, Max_Cols, Max_Rows> result(n_cols, n_rows);
		for (size_t i = 0; i < n_cols; i++)
		{
			for (size_t j = 0; j < n_rows ; j++)
			{
				result(i, j) = this->operator()(j, i);
			}
		}
		*this = result;
	}

	/****************************************************************************************
	*  Name     : transposed
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ PDMatrix<T, Max_Cols, Max_Rows> PDMatrix<T, Max_Rows, Max_Cols>::transposed() const
	{
		PDMatrix<T, Max_Cols, Max_Rows> result(n_cols, n_rows);
		for (size_t i = 0; i < n_cols; i++)
		{
			for (size_t j = 0; j < n_rows ; j++)
			{
				result(i, j) = this->operator()(j, i);
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : cross
	*  Function : 3D row vectors only. Calculates the cross product this x other
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ Matrix_Type PDMatrix<T, Max_Rows, Max_Cols>::cross(
		const Matrix_Type &other 								// In: Matrix/Vector object to perform cross product with
	) const
	{
		// Check that the objects are in fact correct vectors of matching dimension
		assert((n_rows == 3 && n_cols == 1) && (n_rows == other.get_rows() && n_cols == other.get_cols()));
		
		Matrix_Type result(n_rows, 1);
		result(0) = this->operator()(1) * other(2) - this->operator()(2) * other(1);
		result(1) = this->operator()(2) * other(0) - this->operator()(0) * other(2);
		result(2) = this->operator()(0) * other(1) - this->operator()(1) * other(0);
		
		return result;
	}

	/****************************************************************************************
	*  Name     : cwise_product
	*  Function : Basically the dot product between matching column vectors/scalars in each
	*			  matrix/vector object.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::cwise_product(
		const Matrix_Type &other 									// In: Matrix/Vector object to apply columnwise product to
	) const
	{
		assert(n_cols == other.n_cols 				&&
				((n_rows == 1 && other.n_rows > 1) 	||
				(n_rows > 1 && other.n_rows == 1)));
		
		PDMatrix<T, Max_Rows, Max_Cols> result(1, n_cols);
		for (size_t j = 0; j < n_cols; j++)
		{
			result(0, j) = this->get_block(0, j, n_rows, 1).transposed() * other.get_block(0, j, other.n_rows, 1);	
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : cwise_mean
	*  Function : Calculates the mean columnwise.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::cwise_mean() const
	{
		PDMatrix<T, Max_Rows, Max_Cols> result(1, n_cols);
		for (size_t j = 0; j < n_cols; j++)
		{
			result(j) = (T)0;
			for (size_t i = 0; i < n_rows; i++)
			{
				result(j) += this->operator()(i, j);
			}
			result(j) /= (T)n_rows;
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : rwise_mean
	*  Function : Calculates the mean rowwise.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::rwise_mean() const
	{
		PDMatrix<T, Max_Rows, Max_Cols> result(n_rows, 1);
		for (size_t i = 0; i < n_rows; i++)
		{
			result(i) = (T)0;
			for (size_t j = 0; j < n_cols; j++)
			{
				result(i) += this->operator()(i, j);
			}
			result(i) /= (T)n_rows;
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : identity
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::identity(
		const size_t n_rows,  										// In: Amount of matrix rows
		const size_t n_cols 										// In: Amount of matrix columns
		)
	{
		PDMatrix<T, Max_Rows, Max_Cols> result(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				result(i, j) = (T)0;
				if (i == j)
				{
					result(i, j) = (T)1;
				}
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : ones
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ PDMatrix<T, Max_Rows, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::ones(
		const size_t n_rows,  										// In: Amount of matrix rows
		const size_t n_cols 										// In: Amount of matrix columns
		)
	{
		PDMatrix<T, Max_Rows, Max_Cols> result(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				result(i, j) = (T)1;			
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : set_block
	*  Function : Sets the n_rows x n_cols block of this object, starting at 
	*			  (start_row, start_col).
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	template <class Matrix_Type>
	__host__ __device__  void PDMatrix<T, Max_Rows, Max_Cols>::set_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols, 										// In: Amount of columns
		const Matrix_Type &block 									// In: Block matrix to set
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
	template <class Matrix_Type>
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Matrix_Type &vector 											// In: Row vector to assign to the row
		)
	{
		assert(vector.get_cols() == this->n_cols && row < this->n_rows);
		for (size_t j = 0; j < this->n_cols; j++)
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
	template <class Matrix_Type>
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Matrix_Type &vector 											// In: Column vector to assign to the column
		)
	{
		assert(vector.get_rows() == this->n_rows && col < this->n_cols);
		for (size_t i = 0; i < this->n_rows; i++)
		{
			this->operator()(i, col) = vector(i);
		}
	}

	/****************************************************************************************
	*  Name     : shift_columns_right
	*  Function : Has no effect for n_cols = 1.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::shift_columns_right()
	{
		if (this->n_cols == 1)
		{
			return;
		}
		for (int j = this->n_cols - 2; j > -1; j--)
		{
			for (size_t i = 0; i < this->n_rows; i++)
			{
				this->operator()(i, j + 1) = this->operator()(i, j);
			}
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
	__host__ __device__ PDMatrix<T, New_Max_Rows, New_Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::get_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols 										// In: Amount of columns
		) const
	{
		assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && 
				start_row < this->n_rows && start_col < this->n_cols);

		PDMatrix<T, New_Max_Rows, New_Max_Cols> result(n_rows, n_cols);
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
	__host__ __device__ PDMatrix<T, 1, Max_Cols> PDMatrix<T, Max_Rows, Max_Cols>::get_row(
		const size_t row											// In: Index of row to fetch
		) const
	{
		assert(row < this->n_rows);

		PDMatrix<T, 1, Max_Cols> result(1, n_cols);
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
	__host__ __device__ PDMatrix<T, Max_Rows, 1> PDMatrix<T, Max_Rows, Max_Cols>::get_col(
		const size_t col											// In: Index of row to fetch
		) const
	{
		assert(col < this->n_cols);

		PDMatrix<T, Max_Rows, 1> result(n_rows, 1);
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
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::assign_data(
		const PDMatrix<T, Max_Rows, Max_Cols> &other 				// In: Matrix whose data to assign to *this;
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
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::assign_data(
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
	__host__ __device__ void PDMatrix<T, Max_Rows, Max_Cols>::assign_data(
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
	using PDMatrix2d = PDMatrix<double, 2, 2>;
	using PDMatrix3d = PDMatrix<double, 3, 3>;
	using PDMatrix4d = PDMatrix<double, 4, 4>;
	using PDMatrix6d = PDMatrix<double, 6, 6>;

	using PDVector2d = PDMatrix<double, 2, 1>;
	using PDVector3d = PDMatrix<double, 3, 1>;
	using PDVector4d = PDMatrix<double, 4, 1>;
	using PDVector6d = PDMatrix<double, 6, 1>;
}

	

#endif