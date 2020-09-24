/****************************************************************************************
*
*  File name : static_matrix.cuh
*
*  Function  : Header file for the fixed-matrices in the Cuda Matrix Library.
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

#ifndef _STATIC_MATRIX_CUH_
#define _STATIC_MATRIX_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>

namespace CML
{
	template<class T, class Derived> class Matrix_Base;
	template <class T, int Rows, int Cols>
	class Static_Matrix : public Matrix_Base<T, Static_Matrix<T, Rows, Cols>> 
	{
	private:
		size_t n_rows = Rows, n_cols = Cols;

		T data[Rows * Cols];

		__host__ __device__ void assign_data(const Static_Matrix &other);

		__host__ __device__ void assign_data(const Static_Matrix &other);

	public:

		__host__ __device__ Static_Matrix(const Static_Matrix &other);

		__host__ __device__ Static_Matrix& operator=(const Static_Matrix &rhs);

		template <class U>
		__host__ __device__ Static_Matrix& operator=(const Dynamic_Matrix<U> &rhs);

		template <class U, int Max_Rows, int Max_Cols>
		__host__ __device__ Static_Matrix& operator=(const Pseudo_Dynamic_Matrix<U, Max_Rows, Max_Cols> &rhs);

		template <class U, int Other_Rows, int Other_Cols>
		__host__ __device__ Static_Matrix operator*(const Static_Matrix<U, Other_Rows, Other_Cols> &other) const;

		__host__ __device__ operator Dynamic_Matrix() const
		{
			Dynamic_Matrix<T> result;
			result = *this;
			return result;
		}
		
		__host__ __device__ operator Pseudo_Dynamic_Matrix() const
		{
			Pseudo_Dynamic_Matrix<T, Rows, Cols> result;
			result = *this;
			return result;
		}

		__host__ __device__ Static_Matrix<T, Cols, Rows> transposed() const;

		__host__ __device__ Static_Matrix<T, Rows, 1> cross(const Static_Matrix &other) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> cwise_product(const Static_Matrix &other) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> cwise_mean() const;

		__host__ __device__ Static_Matrix<T, Rows, 1> rwise_mean() const;

		__host__ __device__ static Static_Matrix identity();

		__host__ __device__ static Static_Matrix ones();

		template<class U, int Block_Rows, int Block_Cols>
		__host__ __device__ void set_block(
			const size_t start_row, 
			const size_t start_col, 
			const Static_Matrix<U, Block_Rows, Block_Cols> &block);

		__host__ __device__ void set_row(const size_t row, const Static_Matrix<T, 1, Cols> &vector);

		__host__ __device__ void set_col(const size_t col, const Static_Matrix<T, Rows, 1> &vector);

		template<class U, int Block_Rows, int Block_Cols>
		__host__ __device__ Static_Matrix<U, Block_Rows, Block_Cols> get_block(const size_t start_row, const size_t start_col) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> get_row(const size_t row) const;

		__host__ __device__ Static_Matrix<T, Rows, 1> get_col(const size_t col) const;

		__host__ __device__ inline size_t get_rows() const { return n_rows; }

		__host__ __device__ inline size_t get_cols() const { return n_cols; }

		__host__ __device__ inline T* get_data() { return data; }

		__host__ __device__ inline size_t size() const 
		{ 
			assert(n_rows == 1 || n_cols == 1); 
			if (n_rows > n_cols) 	{ return n_rows; } 
			else 					{ return n_cols; } 
		
		}
	};

	/****************************************************************************************
	*  Assign general eigen matrix to the CML object
	*****************************************************************************************/

	/****************************************************************************************
	*  Global operator functions, to allow for commutativeness
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : operator+ (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator+(const Static_Matrix<T, Rows, Cols> &other, const T scalar)
	{
		Static_Matrix<T, Rows, Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) += scalar;
			}
		}
		return result;
	}
	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator+(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return other + scalar; }

	/****************************************************************************************
	*  Name     : operator- (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator-(const Static_Matrix<T, Rows, Cols> &other, const T scalar)
	{
		Static_Matrix<T, Rows, Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols() ; j++)
			{
				result(i, j) -= scalar;
			}
		}
		return result;
	}
	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator-(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return (T)-1 * other + scalar; }

	/****************************************************************************************
	*  Name     : operator* (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator*(const Static_Matrix<T, Rows, Cols> &other, const T scalar)
	{
		Static_Matrix<T, Rows, Cols> result = other;
		for (size_t i = 0; i < other.get_rows(); i++)
		{
			for (size_t j = 0; j < other.get_cols(); j++)
			{
				result(i, j) *= scalar;
			}
		}
		return result;
	}

	template <class T, int Rows, int Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator*(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return other * scalar; }

	//*********************************************************************************************************************************************************

	/****************************************************************************************
	*  Derived class member functions
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : Static_Matrix
	*  Function : Class constructor, initializes parameters and variables
	*  Method   : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols>::Static_Matrix(
		const Static_Matrix<T, Rows, Cols> &other 								// In: Matrix/vector to copy
		) : 
		n_rows(other.n_rows), n_cols(other.n_cols)
	{
		assign_data(other);
	}

	/****************************************************************************************
	*  Name     : operator=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator=(
		const Static_Matrix<T, Rows, Cols> &rhs 								// In: Right hand side matrix/vector to assign
		)
	{
		if (this == &rhs)
		{
			return *this;
		}

		assign_data(rhs);
		
		return *this;
	}

	template <class T, int Rows, int Cols>
	template <class U>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator=(
		const Dynamic_Matrix<U> &rhs 								// In: Right hand side matrix/vector to assign
		)
	{
		assign_data(rhs);
		
		return *this;
	}

	template <class T, int Rows, int Cols>
	template <class U, int Max_Rows, int Max_Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator=(
		const Pseudo_Dynamic_Matrix<U, Max_Rows, Max_Cols> &rhs 								// In: Right hand side matrix/vector to assign
		)
	{
		assign_data(rhs);
		
		return *this;
	}

	/****************************************************************************************
	*  Name     : operator*
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	template<class U, int Other_Rows, int Other_Cols>
	__host__ __device__  Static_Matrix<T, Rows, Cols>  Static_Matrix<T, Rows, Cols>::operator*(
		const  Static_Matrix<U, Other_Rows, Other_Cols> &other 							// In: Matrix/vector to multiply with
		) const
	{	
		// Verify that the matrix product is valid
		assert(n_cols == other.n_rows);

		Static_Matrix<T, Rows, Other_Cols> result;
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
	*  Name     : transposed
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Cols, Rows> Static_Matrix<T, Rows, Cols>::transposed() const
	{
		Static_Matrix<T, Cols, Rows> result;
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
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::cross(
		const Static_Matrix<T, Rows, Cols> &other 								// In: Matrix/Vector object to perform cross product with
	) const
	{
		// Check that the objects are in fact correct vectors of matching dimension
		assert((n_rows == 3 && n_cols == 1) && (n_rows == other.n_rows && n_cols == other.n_cols));
		
		Static_Matrix<T, Rows, 1> result;
		result(0) = this->operator()(1) * other(2) - this->operator()(2) * other(1);
		result(1) = this->operator()(2) * other(0) - this->operator()(0) * other(2);
		result(2) = this->operator()(0) * other(1) - this->operator()(1) * other(0);

		if (n_rows == 1)
		{
			result.transpose();
		}
		
		return result;
	}

	/****************************************************************************************
	*  Name     : cwise_product
	*  Function : Basically the dot product between matching column vectors/scalars in each
	*			  matrix/vector object.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::cwise_product(
		const Static_Matrix<T, Rows, Cols> &other 								// In: Matrix/Vector object to apply columnwise product to
	) const
	{
		assert(n_cols == other.n_cols 				&&
				((n_rows == 1 && other.n_rows > 1) 	||
				(n_rows > 1 && other.n_rows == 1)));
		
		Static_Matrix<T, 1, Cols> result;
		for (size_t j = 0; j < n_cols; j++)
		{
			result(0, j) = this->get_block(0, j, n_rows, 1).transpose() * other.get_block(0, j, other.n_rows, 1);	
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : cwise_mean
	*  Function : Calculates the mean columnwise.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::cwise_mean() const
	{
		Static_Matrix<T, 1, Cols> result;
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
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::rwise_mean() const
	{
		Static_Matrix<T, Rows, 1> result;
		for (size_t i = 0; i < n_rows; i++)
		{
			result(i) = (T)0;
			for (size_t j = 0; j < n_cols; j++)
			{
				result(i) += this->operator()(i, j);
			}
			result(i) /= (T)n_rows;
		}
	}

	/****************************************************************************************
	*  Name     : identity
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::identity()
	{
		Static_Matrix<T, Rows, Cols> result;
		for (int i = 0; i < Rows; i++)
		{
			for (int j = 0; j < Cols; j++)
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
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::ones()
	{
		Static_Matrix<T, Rows, Cols> result;
		for (int i = 0; i < Rows; i++)
		{
			for (int j = 0; j < Cols; j++)
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
	template <class T, int Rows, int Cols>
	template <class U, int Block_Rows, int Block_Cols>
	__host__ __device__  void Static_Matrix<T, Rows, Cols>::set_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const Static_Matrix<U, Block_Rows, Block_Cols> &block 		// In: Block matrix to set
		)
	{
		assert(	Block_Rows <= this->n_rows && Block_Cols <= this->n_cols && 
				start_row < this->n_rows && start_col < this->n_cols);

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
	template <class T, int Rows, int Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Static_Matrix<T, 1, Cols> &vector 							// In: Row vector to assign to the row
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
	template <class T, int Rows, int Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Static_Matrix<T, Rows, 1> &vector 							// In: Column vector to assign to the column
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
	*			  index (start_row, start_col). This version for the Static Matrix has not
	*			  yet been fully fixed. 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, int Rows, int Cols>
	template <class U, int Block_Rows, int Block_Cols>
	__host__ __device__ Static_Matrix<U, Block_Rows, Block_Cols> Static_Matrix<T, Rows, Cols>::get_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col	 									// In: Start column of matrix block
		) const
	{
		assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && n_rows > 0 && n_cols > 0 && 
				start_row < n_rows && start_col < n_cols);

		Static_Matrix<U, Block_Rows, Block_Cols> result;
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
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::get_row(
		const size_t row											// In: Index of row to fetch
		) const
	{
		assert(row < this->n_rows);

		Static_Matrix<T, 1, Cols> result;
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
	template <class T, int Rows, int Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::get_col(
		const size_t col											// In: Index of column to fetch
		) const
	{
		assert(col < this->n_cols);

		Static_Matrix<T, Rows, 1> result;
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
	template <class T, int Rows, int Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::assign_data(
		const Static_Matrix<T, Rows, Cols> &other 									// In: Matrix whose data to assign to *this;
		)
	{
		assert(n_rows == other.n_rows && n_cols == other.n_cols);

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
	using Matrix2d = Static_Matrix<double, 2, 2>;
	using Matrix3d = Static_Matrix<double, 3, 3>;
	using Matrix4d = Static_Matrix<double, 4, 4>;

	using Vector2d = Static_Matrix<double, 2, 1>;
	using Vector3d = Static_Matrix<double, 3, 1>;
	using Vector4d = Static_Matrix<double, 4, 1>;
}

#endif