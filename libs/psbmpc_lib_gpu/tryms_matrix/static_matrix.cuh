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

namespace TML
{
	template<class T, class Derived> class Matrix_Base;
	template<class T> class Dynamic_Matrix;
	template<class T, size_t Max_Rows, size_t Max_Cols> class PDMatrix;

	template <class T, size_t Rows, size_t Cols>
	class Static_Matrix : public Matrix_Base<T, Static_Matrix<T, Rows, Cols>> 
	{
	private:

		T data[Rows * Cols];

		__host__ __device__ void assign_data(const Static_Matrix &other);

		template <class U>
		__host__ __device__ void assign_data(const Dynamic_Matrix<U> &other);

		template <class U, size_t Max_Rows, size_t Max_Cols>
		__host__ __device__ void assign_data(const PDMatrix<U, Max_Rows, Max_Cols> &other);

	public:

		__host__ __device__ inline Static_Matrix() {}

		__host__ __device__ inline Static_Matrix(const Static_Matrix &other) {	assign_data(other);	}

		__host__ __device__ Static_Matrix& operator=(const Static_Matrix &rhs);

		template <class U>
		__host__ __device__ Static_Matrix& operator=(const Dynamic_Matrix<U> &rhs);

		template <class U, size_t Max_Rows, size_t Max_Cols>
		__host__ __device__ Static_Matrix& operator=(const PDMatrix<U, Max_Rows, Max_Cols> &rhs);

		template <class U>
		__host__ __device__ inline operator Dynamic_Matrix<U>() const
		{
			Dynamic_Matrix<U> result(Rows, Cols);
			for (size_t i = 0; i < Rows; i++)
			{
				for (size_t j = 0; j < Cols; j++)
				{
					result(i, j) = this->operator()(i, j);
				}
			}
			return result;
		}
		
		/* template <class U, size_t Max_Rows, size_t Max_Cols>
		__host__ __device__ inline operator PDMatrix<U, Max_Rows, Max_Cols>() const
		{
			PDMatrix<U, Max_Rows, Max_Cols> result;
			result = *this;
			return result;
		} */

		/*
		template <class U, size_t Max_Rows, size_t Max_Cols>
		__host__ __device__ Static_Matrix operator+(const PDMatrix<U, Max_Rows, Max_Cols> &other) const;

		 template <class Matrix_Type>
		__host__ __device__ Static_Matrix& operator+=(const Matrix_Type &rhs);

		template <class Matrix_Type>
		__host__ __device__ Static_Matrix operator-(const Matrix_Type &other) const;

		template <class Matrix_Type>
		__host__ __device__ Static_Matrix& operator-=(const Matrix_Type &rhs);  */

		template <class U, size_t Other_Rows, size_t Other_Cols>
		__host__ __device__ Static_Matrix<T, Rows, Other_Cols> operator*(const Static_Matrix<U, Other_Rows, Other_Cols> &other) const;

		__host__ __device__ Static_Matrix<T, Cols, Rows> transposed() const;

		__host__ __device__ Static_Matrix<T, Rows, 1> cross(const Static_Matrix &other) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> cwise_product(const Static_Matrix &other) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> cwise_mean() const;

		__host__ __device__ Static_Matrix<T, Rows, 1> rwise_mean() const;

		__host__ __device__ static Static_Matrix identity();

		__host__ __device__ static Static_Matrix ones();

		template<size_t Block_Rows, size_t Block_Cols>
		__host__ __device__ void set_block(
			const size_t start_row, 
			const size_t start_col, 
			const Static_Matrix<T, Block_Rows, Block_Cols> &block);

		__host__ __device__ void set_row(const size_t row, const Static_Matrix<T, 1, Cols> &vector);

		__host__ __device__ void set_col(const size_t col, const Static_Matrix<T, Rows, 1> &vector);

		template<size_t Block_Rows, size_t Block_Cols>
		__host__ __device__ Static_Matrix<T, Block_Rows, Block_Cols> get_block(const size_t start_row, const size_t start_col) const;

		__host__ __device__ Static_Matrix<T, 1, Cols> get_row(const size_t row) const;

		__host__ __device__ Static_Matrix<T, Rows, 1> get_col(const size_t col) const;

		__host__ __device__ inline size_t get_rows() const { return Rows; }

		__host__ __device__ inline size_t get_cols() const { return Cols; }

		__host__ __device__ inline T* get_data() { return data; }
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
	template <class T, size_t Rows, size_t Cols>
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator+(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return other + scalar; }

	/****************************************************************************************
	*  Name     : operator- (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator-(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return (T)-1 * other + scalar; }

	/****************************************************************************************
	*  Name     : operator* (by scalar)
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
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

	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ inline Static_Matrix<T, Rows, Cols> operator*(const T scalar, const Static_Matrix<T, Rows, Cols> &other) { return other * scalar; }

	//*********************************************************************************************************************************************************

	/****************************************************************************************
	*  Derived class member functions
	*****************************************************************************************/
	/****************************************************************************************
	*  Name     : operator=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
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

	template <class T, size_t Rows, size_t Cols>
	template <class U>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator=(
		const Dynamic_Matrix<U> &rhs 								// In: Right hand side matrix/vector to assign
		)
	{
		assign_data(rhs);
		
		return *this;
	}

	template <class T, size_t Rows, size_t Cols>
	template <class U, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator=(
		const PDMatrix<U, Max_Rows, Max_Cols> &rhs 								// In: Right hand side matrix/vector to assign
		)
	{
		assign_data(rhs);
		
		return *this;
	}

	/****************************************************************************************
	*  Name     : operator+
	*  Function : Can add matrices of equal dimension, or row/column vectorts to the lhs, 
	*			  given that the dimension matches
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	/* template <class T, size_t Rows, size_t Cols>
	template <class U, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::operator+(
		const PDMatrix<U, Max_Rows, Max_Cols> &other 							// In: Matrix/vector to add by
		) const
	{
		if (Rows < other.get_rows() || Cols < other.get_cols())
		{
			return other + *this;
		}

		assert((Rows == other.get_rows() && Cols == other.get_cols()) 	|| 
				(Rows == other.get_rows() && other.get_cols() == 1) 		||
				(Cols == other.get_cols() && other.get_rows() == 1));

		Static_Matrix<T, Rows, Cols> result = *this;
		
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols ; j++)
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
	} */

	/****************************************************************************************
	*  Name     : operator+=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	/* template <class T, size_t Rows, size_t Cols>
	template <class Matrix_Type>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator+=(
		const Matrix_Type &rhs 										// In: Right hand side matrix/vector to add by
		)
	{
		assert(Rows == rhs.get_rows() && Cols == rhs.get_cols());
		
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
			{
				this->operator()(i, j) += rhs(i, j);
			}
		}
		return *this;
	} */

	/****************************************************************************************
	*  Name     : operator-
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	/* template <class T, size_t Rows, size_t Cols>
	template <class Matrix_Type>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::operator-(
		const Matrix_Type &other 									// In: Matrix/vector to subtract by
		) const
	{
		if (Rows < other.get_rows() || Cols < other.get_cols())
		{
			return other - *this;
		}

		assert((Rows == other.get_rows() && Cols == other.get_cols()) 	|| 
				(Rows == other.get_rows() && other.get_cols() == 1) 					||
				(Cols == other.get_cols() && other.get_rows() == 1));

		Static_Matrix<T, Rows, Cols> result = *this;

		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols ; j++)
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
	} */

	/****************************************************************************************
	*  Name     : operator-=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	/* template <class T, size_t Rows, size_t Cols>
	template <class Matrix_Type>
	__host__ __device__ Static_Matrix<T, Rows, Cols>& Static_Matrix<T, Rows, Cols>::operator-=(
		const Matrix_Type &rhs 										// In: Right hand side matrix/vector to subtract by
		)
	{
		assert(Rows == rhs.get_rows());
		assert(Cols == rhs.get_cols());

		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols ; j++)
			{
				this->operator()(i, j) -= rhs(i, j);
			}
		}
		return *this;
	} */

	/****************************************************************************************
	*  Name     : operator*
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	template <class U, size_t Other_Rows, size_t Other_Cols>
	__host__ __device__  Static_Matrix<T, Rows, Other_Cols>  Static_Matrix<T, Rows, Cols>::operator*(
		const  Static_Matrix<U, Other_Rows, Other_Cols> &other 							// In: Matrix/vector to multiply with
		) const
	{	
		// Verify that the matrix product is valid
		assert(Cols == Other_Rows);

		Static_Matrix<T, Rows, Other_Cols> result;
		for (size_t i = 0 ; i < Rows; i++)
		{
			for (size_t j = 0; j < Other_Cols; j++)
			{
				result(i, j) = (T)0;
				for (size_t k = 0; k < Cols; k++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Cols, Rows> Static_Matrix<T, Rows, Cols>::transposed() const
	{
		Static_Matrix<T, Cols, Rows> result;
		for (size_t i = 0; i < Cols; i++)
		{
			for (size_t j = 0; j < Rows ; j++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::cross(
		const Static_Matrix<T, Rows, Cols> &other 								// In: Matrix/Vector object to perform cross product with
	) const
	{
		// Check that the objects are in fact correct vectors of matching dimension
		assert(Rows == 3 && Cols == 1);
		
		Static_Matrix<T, Rows, 1> result;
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::cwise_product(
		const Static_Matrix<T, Rows, Cols> &other 								// In: Matrix/Vector object to apply columnwise product to
	) const
	{		
		Static_Matrix<T, 1, Cols> result;
		for (size_t j = 0; j < Cols; j++)
		{
			result(0, j) = this->get_block<T, Rows, 1>(0, j).dot(other.get_block<T, Rows, 1>(0, j));	
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : cwise_mean
	*  Function : Calculates the mean columnwise.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::cwise_mean() const
	{
		Static_Matrix<T, 1, Cols> result;
		for (size_t j = 0; j < Cols; j++)
		{
			result(j) = (T)0;
			for (size_t i = 0; i < Rows; i++)
			{
				result(j) += this->operator()(i, j);
			}
			result(j) /= (T)Rows;
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : rwise_mean
	*  Function : Calculates the mean rowwise.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::rwise_mean() const
	{
		Static_Matrix<T, Rows, 1> result;
		for (size_t i = 0; i < Rows; i++)
		{
			result(i) = (T)0;
			for (size_t j = 0; j < Cols; j++)
			{
				result(i) += this->operator()(i, j);
			}
			result(i) /= (T)Rows;
		}
	}

	/****************************************************************************************
	*  Name     : identity
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::identity()
	{
		Static_Matrix<T, Rows, Cols> result;
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::ones()
	{
		Static_Matrix<T, Rows, Cols> result;
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
			{
				result(i, j) = (T)1;			
			}
		}
		return result;
	}

	/****************************************************************************************
	*  Name     : set_block
	*  Function : Sets the Block_Rows x Block_Cols block of this object, starting at 
	*			  (start_row, start_col).
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	template <size_t Block_Rows, size_t Block_Cols>
	__host__ __device__  void Static_Matrix<T, Rows, Cols>::set_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col, 									// In: Start column of matrix block
		const Static_Matrix<T, Block_Rows, Block_Cols> &block 		// In: Block matrix to set
		)
	{
		assert(	Block_Rows <= Rows && Block_Cols <= Cols && 
				start_row < Rows && start_col < Cols);

		for (size_t i = 0; i < Block_Rows; i++)
		{
			for (size_t j = 0; j < Block_Cols; j++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::set_row(
		const size_t row,		 											// In: Index of row to assign
		const Static_Matrix<T, 1, Cols> &vector 							// In: Row vector to assign to the row
		)
	{
		assert(row < Rows);
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::set_col(
		const size_t col,		 											// In: Index of column to assign
		const Static_Matrix<T, Rows, 1> &vector 							// In: Column vector to assign to the column
		)
	{
		assert(col < Cols);
		for (size_t i = 0; i < Rows; i++)
		{
			this->operator()(i, col) = vector(i);
		}
	}

	/****************************************************************************************
	*  Name     : get_block
	*  Function : returns the Block_Rows x Block_Cols block of this object, with upper left 
	*			  reference index (start_row, start_col).
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, size_t Rows, size_t Cols>
	template <size_t Block_Rows, size_t Block_Cols>
	__host__ __device__ Static_Matrix<T, Block_Rows, Block_Cols> Static_Matrix<T, Rows, Cols>::get_block(
		const size_t start_row, 									// In: Start row of matrix block
		const size_t start_col	 									// In: Start column of matrix block
		) const
	{
		assert(	Block_Rows <= Rows && Block_Cols <= Cols && 
				start_row < Rows && start_col < Cols);

		Static_Matrix<T, Block_Rows, Block_Cols> result;
		for (size_t i = 0; i < Block_Rows; i++)
		{
			for (size_t j = 0; j < Block_Cols; j++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, 1, Cols> Static_Matrix<T, Rows, Cols>::get_row(
		const size_t row											// In: Index of row to fetch
		) const
	{
		assert(row < Rows);

		Static_Matrix<T, 1, Cols> result;
		for (size_t j = 0; j < Cols; j++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ Static_Matrix<T, Rows, 1> Static_Matrix<T, Rows, Cols>::get_col(
		const size_t col											// In: Index of column to fetch
		) const
	{
		assert(col < Cols);

		Static_Matrix<T, Rows, 1> result;
		for (size_t i = 0; i < Rows; i++)
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
	template <class T, size_t Rows, size_t Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::assign_data(
		const Static_Matrix<T, Rows, Cols> &other 									// In: Matrix whose data to assign to *this;
		)
	{
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
			{
				this->operator()(i, j) = other(i, j);
			}
		}
	}

	template <class T, size_t Rows, size_t Cols>
	template <class U>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::assign_data(
		const Dynamic_Matrix<U> &other 									// In: Matrix whose data to assign to *this;
		)
	{
		assert(Rows == other.get_rows() && Cols == other.get_cols());

		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
			{
				this->operator()(i, j) = other(i, j);
			}
		}
	}

	template <class T, size_t Rows, size_t Cols>
	template <class U, size_t Max_Rows, size_t Max_Cols>
	__host__ __device__ void Static_Matrix<T, Rows, Cols>::assign_data(
		const PDMatrix<U, Max_Rows, Max_Cols> &other 									// In: Matrix whose data to assign to *this;
		)
	{
		assert(Rows == other.get_rows() && Cols == other.get_cols());

		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
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
	using Matrix5d = Static_Matrix<double, 5, 5>;
	using Matrix6d = Static_Matrix<double, 6, 6>;

	using Vector2d = Static_Matrix<double, 2, 1>;
	using Vector3d = Static_Matrix<double, 3, 1>;
	using Vector4d = Static_Matrix<double, 4, 1>;
	using Vector5d = Static_Matrix<double, 5, 1>;
	using Vector6d = Static_Matrix<double, 6, 1>;
}

#endif