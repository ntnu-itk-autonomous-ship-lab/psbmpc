/****************************************************************************************
*
*  File name : dynamic_matrix.cuh
*
*  Function  : Header file for the dynamic matrix class used in the Cuda Matrix Library
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

#ifndef _DYNAMIC_MATRIX_CUH_
#define _DYNAMIC_MATRIX_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>

namespace CML
{
	template<class T, class Derived> class Matrix_Base;
	template <class T>
	class Dynamic_Matrix : public Matrix_Base<T, Dynamic_Matrix<T>> 
	{
	private:
		size_t n_rows, n_cols;

		T* data;

		__host__ __device__ void allocate_data();

		__host__ __device__ void deallocate_data();

		__host__ __device__ void assign_data(const Dynamic_Matrix &other);

	public:
		
		__host__ __device__ Dynamic_Matrix() {}

		__host__ __device__ Dynamic_Matrix(const size_t n_rows);

		__host__ __device__ Dynamic_Matrix(const size_t n_rows, const size_t n_cols);

		__host__ __device__ Dynamic_Matrix(const Dynamic_Matrix &other);

		__host__ __device__ ~Dynamic_Matrix();

		__host__ __device__ Dynamic_Matrix& operator=(const Dynamic_Matrix &rhs);

		__host__ __device__ Dynamic_Matrix operator*(const Dynamic_Matrix &other) const;

		__host__ __device__ void transpose();

		__host__ __device__ Dynamic_Matrix transposed() const;

		__host__ __device__ Dynamic_Matrix cross(const Dynamic_Matrix &other) const;

		__host__ __device__ Dynamic_Matrix cwise_product(const Dynamic_Matrix &other) const;

		__host__ __device__ Dynamic_Matrix cwise_mean() const;

		__host__ __device__ Dynamic_Matrix rwise_mean() const;

		__host__ __device__ static Dynamic_Matrix identity(const size_t n_rows, const size_t n_cols);

		__host__ __device__ static Dynamic_Matrix ones(const size_t n_rows, const size_t n_cols);

		__host__ __device__ void set_block(
			const size_t start_row, 
			const size_t start_col, 
			const size_t n_rows, 
			const size_t n_cols, 
			const Dynamic_Matrix &block);

		__host__ __device__ void set_row(const size_t row, const Dynamic_Matrix &vector);

		__host__ __device__ void set_col(const size_t col, const Dynamic_Matrix &vector);

		__host__ __device__ Dynamic_Matrix get_block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

		__host__ __device__ Dynamic_Matrix get_row(const size_t row) const;

		__host__ __device__ Dynamic_Matrix get_col(const size_t col) const;

		__host__ __device__ inline size_t get_rows() const { return n_rows; }

		__host__ __device__ inline size_t get_cols() const { return n_cols; }

		__host__ __device__ inline T* get_data() const { return data; }

		__host__ __device__ inline size_t size() const 
		{ 
			assert(n_rows == 1 || n_cols == 1); 
			if (n_rows > n_cols) 	{ return n_rows; } 
			else 					{ return n_cols; } 
		
		}

		__host__ __device__ void resize(const size_t n_rows, const size_t n_cols);

		__host__ __device__ void conservative_resize(const size_t n_rows, const size_t n_cols);
		
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
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator+(const Dynamic_Matrix<T> &other, const T scalar)
	{
		Dynamic_Matrix<T> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) += scalar;
			}
		}
		return result;
	}
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator+(const T scalar, const Dynamic_Matrix<T> &other)	{ return other + scalar; }

	/****************************************************************************************
	*  Name     : operator- (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator-(const Dynamic_Matrix<T> &other, const T scalar)
	{
		Dynamic_Matrix<T> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) -= scalar;
			}
		}
		return result;
	}
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator-(const T scalar, const Dynamic_Matrix<T> &other)	{ return (T)-1 * other + scalar; }

	/****************************************************************************************
	*  Name     : operator* (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator*(const Dynamic_Matrix<T> &other, const T scalar)
	{
		Dynamic_Matrix<T> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols(); j++)
			{
				result(i, j) *= scalar;
			}
		}
		return result;
	}
	template <class T>
	__host__ __device__ inline Dynamic_Matrix<T> operator*(const T scalar, const Dynamic_Matrix<T> &other)	{ return other * scalar; }

	/****************************************************************************************
	*  Class member functions
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : Dynamic_Matrix
	*  Function : Class constructor, initializes parameters and variables
	*  Method   : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ Dynamic_Matrix<T>::Dynamic_Matrix(
		const size_t n_rows 										// In: Amount of matrix rows
		) :
		n_rows(n_rows), n_cols(n_rows), data(nullptr)
	{
		allocate_data();
	}

	template <class T>
	__host__ __device__ Dynamic_Matrix<T>::Dynamic_Matrix(
		const size_t n_rows,  										// In: Amount of matrix rows
		const size_t n_cols 										// In: New amount of matrix columns
		) :
		n_rows(n_rows), n_cols(n_cols), data(nullptr)
	{
		allocate_data();
	}

	template <class T>
	__host__ __device__ Dynamic_Matrix<T>::Dynamic_Matrix(
		const Dynamic_Matrix<T> &other 									// In: Matrix/vector to copy
		) :
		data(nullptr)
	{
		assign_data(other);
	}

	/****************************************************************************************
	*  Name     : ~Dynamic_Matrix
	*  Function : Class destructor
	*  Method   : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ Dynamic_Matrix<T>::~Dynamic_Matrix()
	{
		deallocate_data();
	}

	/****************************************************************************************
	*  Name     : operator=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ Dynamic_Matrix<T>& Dynamic_Matrix<T>::operator=(
		const Dynamic_Matrix<T> &rhs 									// In: Right hand side matrix/vector to assign
		)
	{
		if (this == &rhs)
		{
			return *this;
		}
		
		deallocate_data(); 

		assign_data(rhs);
		
		return *this;
	}

	/****************************************************************************************
	*  Name     : operator*
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::operator*(
		const Dynamic_Matrix<T> &other 									// In: Matrix/vector to multiply with
		) const
	{	
		// Verify that the matrix product is valid
		assert(n_cols == other.n_rows);

		Dynamic_Matrix<T> result(n_rows, other.n_cols);
		for (size_t i = 0 ; i < n_rows; i++)
		{
			for (size_t j = 0; j < other.n_cols; j++)
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
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::transpose()
	{
		Dynamic_Matrix<T> result(n_cols, n_rows);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::transposed() const
	{
		Dynamic_Matrix result(n_cols, n_rows);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::cross(
		const Dynamic_Matrix<T> &other 								// In: Matrix/Vector object to perform cross product with
	) const
	{
		// Check that the objects are in fact correct vectors of matching dimension
		assert((n_rows == 3 && n_cols == 1) && (n_rows == other.n_rows && n_cols == other.n_cols));
		
		Dynamic_Matrix<T> result(n_rows, 1);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::cwise_product(
		const Dynamic_Matrix<T> &other 								// In: Matrix/Vector object to apply columnwise product to
	) const
	{
		assert(n_cols == other.n_cols 				&&
				((n_rows == 1 && other.n_rows > 1) 	||
				(n_rows > 1 && other.n_rows == 1)));
		
		Dynamic_Matrix<T> result(1, n_cols);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::cwise_mean() const
	{
		Dynamic_Matrix<T> result(1, n_cols);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::rwise_mean() const
	{
		Dynamic_Matrix<T> result(n_rows, 1);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::identity(
		const size_t n_rows,  										// In: Amount of matrix rows
		const size_t n_cols 										// In: Amount of matrix columns
		)
	{
		Dynamic_Matrix<T> result(n_rows, n_cols);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::ones(
		const size_t n_rows,  										// In: Amount of matrix rows
		const size_t n_cols 										// In: Amount of matrix columns
		)
	{
		Dynamic_Matrix<T> result(n_rows, n_cols);
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
	template <class T>
	__host__ __device__  void Dynamic_Matrix<T>::set_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols, 										// In: Amount of columns
		const Dynamic_Matrix<T> &block 								// In: Block matrix to set
		)
	{
		assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && 
				start_row < this->n_rows && start_col < this->n_cols);

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
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::set_row(
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

	/****************************************************************************************
	*  Name     : set_col
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::set_col(
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

	/****************************************************************************************
	*  Name     : get_block
	*  Function : returns the n_rows x n_cols block of this object, with upper left reference
	*			  index (start_row, start_col).
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::get_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const size_t n_rows,  										// In: Amount of rows
		const size_t n_cols 										// In: Amount of columns
		) const
	{

		assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && n_rows > 0 && n_cols > 0 && 
				start_row < n_rows && start_col < n_cols);

		Dynamic_Matrix<T> result(n_rows, n_cols);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::get_row(
		const size_t row											// In: Index of row to fetch
		) const
	{
		assert(row < this->n_rows);

		Dynamic_Matrix<T> result(1, n_cols);
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
	template <class T>
	__host__ __device__ Dynamic_Matrix<T> Dynamic_Matrix<T>::get_col(
		const size_t col											// In: Index of column to fetch
		) const
	{
		assert(col < this->n_cols);

		Dynamic_Matrix<T> result(n_rows, 1);
		for (size_t i = 0; i < n_rows; i++)
		{
				result(i, 0) = this->operator()(i, col);
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : resize
	*  Function : Resizes the Matrix_Base without keeping old data
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::resize(
		const size_t n_rows,  										// In: New amount of rows
		const size_t n_cols 										// In: New amount of columns
		)
	{
		assert(n_rows > 0 && n_cols > 0);

		*this = Dynamic_Matrix<T>(n_rows, n_cols);
	}

	/****************************************************************************************
	*  Name     : conservative_resize
	*  Function : Resizes the Matrix_Base, keeping the old data
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::conservative_resize(
		const size_t n_rows,  										// In: New amount of rows
		const size_t n_cols 										// In: New amount of columns
		)
	{
		assert(n_rows > 0 && n_cols > 0);
		
		Dynamic_Matrix<T> resized(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				resized(i, j) = this->operator()(i, j);
			}
		}
		*this = resized;
	}

	/****************************************************************************************
		Private functions
	****************************************************************************************/
	/****************************************************************************************
	*  Name     : allocate_data
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::allocate_data()
	{
		assert(n_rows > 0 && n_cols > 0);

		data = new T[n_rows * n_cols];
	}

	/****************************************************************************************
	*  Name     : deallocate_data
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::deallocate_data()
	{
		if (data == nullptr)
		{
			return;
		}

		delete[] data;

		data = nullptr;
	}

	/****************************************************************************************
	*  Name     : assign_data
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T>
	__host__ __device__ void Dynamic_Matrix<T>::assign_data(
		const Dynamic_Matrix<T> &other 									// In: Matrix whose data to assign to *this;
		)
	{
		n_rows = other.n_rows;
		n_cols = other.n_cols;

		if (data == nullptr)
		{
			allocate_data();
		}

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
	using MatrixXb = Dynamic_Matrix<bool>;
	using MatrixXi = Dynamic_Matrix<int>;
	using MatrixXf = Dynamic_Matrix<float>;
	using MatrixXd = Dynamic_Matrix<double>;
}

	

#endif