/****************************************************************************************
*
*  File name : cmatrix.cuh
*
*  Function  : Header file for the matrix (and vector container) used inside the Cuda 
*			   kernels for the PSB-MPC GPU calculations. Can contain any normal data type
*			   such as double, float, int, bool etc..
*			   Storage is in row-major.
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

#ifndef _CMATRIX_CUH_
#define _CMATRIX_CUH_



#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>

//=========================================================================================================================
// Matrix base that defines common matrix/vector functionality
//=========================================================================================================================
template<class T, class Derived>
class Matrix_Base
{
private:

	__host__ __device__ T calculate_determinant_recursive(const Derived &submatrix) const;

	__host__ __device__ void fill_minor_matrix(Derived &minor_matrix, const Derived &original_matrix, const size_t row, const size_t col) const;
	
public:

	__host__ __device__ Derived operator+(const Derived &other) const;
	__host__ __device__ Derived operator+(const T &scalar) const;

	__host__ __device__ Derived& operator+=(const Derived &rhs);

	__host__ __device__ Derived operator-(const Derived &other) const;
	__host__ __device__ Derived operator-(const T &scalar) const;

	__host__ __device__ Derived& operator-=(const Derived &rhs);

	__host__ __device__ Derived operator*(const T &scalar) const;

	__host__ __device__ Derived& operator*=(const T &scalar);

	__host__ __device__ Derived operator/(const T &scalar) const;

	__host__ __device__ Derived& operator/=(const T &scalar);

	__host__ __device__ bool operator==(const Derived &rhs) const;

	__host__ __device__ T& operator()(const size_t row, const size_t col) const { assert(row < n_rows && col < n_cols); return data[n_rows * row + col]; }

	__host__ __device__ T& operator()(const size_t index) const;

	__host__ __device__ T& operator[](const size_t index) const { return *this(index); }

	__host__ __device__ T determinant() const;

	__host__ __device__ Derived inverse() const;

	__host__ __device__ T dot(const Derived &other) const;

	__host__ __device__ void normalize();

	__host__ __device__ Derived normalized() const;

	__host__ __device__ T norm() const;

	__host__ __device__ Derived cwise_product(const Derived &other) const;

	__host__ __device__ Derived cwise_mean() const;

	__host__ __device__ Derived rwise_mean() const;

	__host__ __device__ Derived exp() const;

	__host__ __device__ Derived log() const;

	__host__ __device__ static Derived identity(const size_t n_rows, const size_t n_cols);

	__host__ __device__ static Derived ones(const size_t n_rows, const size_t n_cols);

	__host__ __device__ void set_block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols, const Derived &block);

	__host__ __device__ void set_row(const size_t row, const Derived &vector);

	__host__ __device__ void set_col(const size_t col, const Derived &vector);

	__host__ __device__ void set_zero();

	__host__ __device__ void set_ones();

	__host__ __device__ void set_all_coeffs(const T coeff);

	__host__ __device__ Derived& get_this() { return (Derived&)*this; }

	__host__ __device__ Derived get_block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

	__host__ __device__ Derived get_row(const size_t row) const;

	__host__ __device__ Derived get_col(const size_t col) const;

	__host__ __device__ size_t get_rows() const { return n_rows; }

	__host__ __device__ size_t get_cols() const { return n_cols; }

	__host__ __device__ T** get_data() const { return data; }

	__host__ __device__ T max_coeff() const;

	__host__ __device__ T min_coeff() const;

	__host__ friend std::ostream& operator<<(std::ostream& os, const Derived<T, Derived> &cm)
	{
		for (size_t i = 0; i< cm.n_rows; i++)
		{
			for (size_t j = 0; j< cm.n_cols; j++)
			{
				os << cm(i, j) << ' '; 
			}
			os << '\n';
		}
		return os;
	}
};

/****************************************************************************************
*  Global operator functions, to allow for commutativeness
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived operator+(const T &scalar, const Derived &other)	{ return other + scalar;}

template <class T, class Derived>
__host__ __device__ Derived operator-(const T &scalar, const Derived &other)	{ return -other + scalar; }

template <class T, class Derived>
__host__ __device__ Derived operator*(const T &scalar, const Derived &other)	{ return other * scalar; }


/****************************************************************************************
*  Name     : operator+
*  Function : Can add matrices of equal dimension, or row/column vectorts to the lhs, 
*			  given that the dimension matches
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator+(
	const Derived &other 									// In: Matrix/vector to add by
	) const
{
	Derived& self = get_this();
	if (self.n_rows < other.n_rows || self.n_cols < other.n_cols)
	{
		return other + self;
	}

	assert((self.n_rows == other.n_rows && self.n_cols == other.n_cols) 	|| 
			(self.n_rows == other.n_rows && other.n_cols == 1) 				||
			(self.n_cols == other.n_cols && other.n_rows == 1));

	Derived result = self;
	
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			if (other.n_rows == 1)
			{
				result(i, j) += other(0, j);
			} 
			else if(other.n_cols == 1)
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

template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator+(
	const T &scalar 										// In: Scalar to add by
	) const
{
	Derived& self = get_this();
	Derived result = self;
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			result(i, j) += scalar;
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
template <class T, class Derived>
__host__ __device__ Derived& Matrix_Base<T, Derived>::operator+=(
	const Derived &rhs 										// In: Right hand side matrix/vector to add by
	)
{
	assert(self.n_rows == rhs.n_rows && self.n_cols == rhs.n_cols);

	Derived& self = get_this();
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			self(i, j) += rhs(i, j);
		}
	}
	return self;
}

/****************************************************************************************
*  Name     : operator-
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator-(
	const Derived &other 									// In: Matrix/vector to subtract by
	) const
{
	Derived& self = get_this();
	if (self.n_rows < other.n_rows || self.n_cols < other.n_cols)
	{
		return other - self;
	}

	assert((self.n_rows == other.n_rows && self.n_cols == other.n_cols) 	|| 
			(self.n_rows == other.n_rows && other.n_cols == 1) 				||
			(self.n_cols == other.n_cols && other.n_rows == 1));

	Derived result = self;

	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			if (other.n_rows == 1)
			{
				result(i, j) -= other(0, j);
			} 
			else if(other.n_cols == 1)
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

template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator-(
	const T &scalar 										// In: Scalar to subtract by
	) const
{
	Derived& self = get_this();
	Derived result = self;
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			result(i, j) -= scalar;
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
template <class T, class Derived>
__host__ __device__ Derived& Matrix_Base<T, Derived>::operator-=(
	const Derived &rhs 										// In: Right hand side matrix/vector to subtract by
	)
{
	assert(self.n_rows == rhs.n_rows);
	assert(self.n_cols == rhs.n_cols);

	Derived& self = get_this();
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			self(i, j) -= rhs(i, j);
		}
	}
	return self;
}

/****************************************************************************************
*  Name     : operator*
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator*(
	const T &scalar 											// In: scalar to multiply with
	) const
{
	Derived& self = get_this();
	Derived result = self;
	for (size_t i = 0 ; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols; j++)
		{
			result(i, j) *= scalar;
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : operator*=
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived& Matrix_Base<T, Derived>::operator*=(
	const T &scalar 											// In: scalar to multiply with
	)
{	
	Derived& self = get_this();
	for (size_t i = 0 ; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols; j++)
		{
			self(i, j) *= scalar;
		}
	}
	return self;
}

/****************************************************************************************
*  Name     : operator/
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::operator/(
	const T &scalar 											// In: scalar to divide by
	) const
{	
	Derived& self = get_this();
	Derived result = self;
	for (size_t i = 0 ; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols; j++)
		{
			result(i, j) /= scalar;
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : operator/=
*  Function : Elementwise division
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived& Matrix_Base<T, Derived>::operator/=(
	const T &scalar 											// In: Right hand side scalar to divide by
	)
{
	Derived& self = get_this();
	for (size_t i = 0 ; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols; j++)
		{
			self(i, j) /= scalar;
		}
	}
	return self;
}

/****************************************************************************************
*  Name     : operator==
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ bool Matrix_Base<T, Derived>::operator==(
	const Derived &rhs 										// In: Right hand side matrix/vector to compare with
	) const
{
	assert(self.n_rows == rhs.n_rows && self.n_cols == rhs.n_cols);

	Derived& self = get_this();
	bool result = true;
	for (size_t i = 0; i < self.n_rows; i++)
	{
		for (size_t j = 0; j < self.n_cols ; j++)
		{
			if (self(i, j) != rhs(i, j))
			{
				result = false;
			}
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : operator()
*  Function : Fetches vector element reference
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T& Matrix_Base<T, Derived>::operator()(
	const size_t index 										// In: Index of element to fetch
	) const
{
	Derived& self = get_this();
	assert(self.n_rows == 1 || self.n_cols == 1 && (index < self.n_cols || index < self.n_rows));

	if (n_rows == 1)
	{
		return self(0, index);
	} 
	else
	{
		return self(index, 0);
	}
}

/****************************************************************************************
*  Name     : determinant
*  Function : Works for square matrices only
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::determinant() const
{
	Derived& self = get_this();
	assert(self.n_rows == self.n_cols);

	if (self.n_rows == 1)
	{
        return self(0, 0);
	}
	if (self.n_rows == 2)
	{
		return self(0, 0) * self(1, 1) - self(0, 1) * self(1, 0);
	}

    T det = 0;
 
	// allocate the cofactor matrix
	Dynamic_Matrix<T> temp_minor(self.n_rows - 1);

    for(size_t i = 0; i < self.n_rows; i++)
    {
        // get minor of element (0,i)
        fill_minor_matrix(self, temp_minor, 0, i);
 
        det += (i % 2 == 1 ? -1.0 : 1.0) * self(0, i) * calculate_determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * self(0, i) * calculate_determinant_recursive(temp_minor);
    }
 
    return det;
}

/****************************************************************************************
*  Name     : inverse
*  Function : Calculates the matrix inverse using the cofactor/minor method
*			  A_inv = C^T / det(A)
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::inverse() const
{
	Derived& self = get_this();
	assert(self.n_rows == self.n_cols && determinant() > T(0));
	
	T det_inv = (T)1 / determinant();

	Derived result = self;

	if (self.n_rows == 1)
	{
		result(0, 0) = det_inv;
		return result;
	}
	if (self.n_rows == 2)
	{
		result(0, 0) = self(1, 1);
		result(0, 1) = - self(0, 1);
		result(1, 0) = - self(1, 0);
		result(1, 1) = self(0, 0);
		result *= det_inv;
		return result;
	}    
 
    Dynamic_Matrix<T> temp_minor(self.n_rows - 1);
 
    for(size_t i = 0; i < self.n_rows; i++)
    {
        for(size_t j = 0; j < self.n_rows; j++)
        {
			// Fill values for minor M_ji
			fill_minor_matrix(self, temp_minor, j, i);

			// Element ij of the inverse is the co-factor C_ji divided by the matrix determinant
			// where C_ij = (-1)^(i + j) * |M_ij|
			
			result(i, j) = ((j + i) % 2 == 1 ? (T)-1 : (T)1) * det_inv * calculate_determinant_recursive(temp_minor);
        }
	}
	return result;
}

/****************************************************************************************
*  Name     : dot
*  Function : Only valid for vectors, i.e. Matrix_Base with either n_rows = 1 or n_cols = 1
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::dot(
	const Derived &other 									// In: Vector to dot with
	) const
{
	Derived& self = get_this();
	assert((self.n_rows == other.n_rows && self.n_cols == other.n_cols) && 
			(self.n_rows == 1 || self.n_cols == 1));

	T result = (T)0;
	if (self.n_rows == 1) 	
	{ 
		for (size_t j = 0; j < self.n_cols; j++)
		{
			result += (*this(0, j)) * other(0, j);
		}
	} 
	else
	{
		for (size_t i = 0; i < self.n_rows; i++)
		{
			result += (*this(i, 0)) * other(i, 0);
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : normalize
*  Function : Vectors only. Normalizes this object
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::normalize()
{
	Derived& self = get_this();
	assert(self.n_rows == 1 || self.n_cols == 1);
	self /= norm();
}

/****************************************************************************************
*  Name     : normalized
*  Function : Vectors only. Returns the normalized vector without modifying this.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::normalized() const
{
	Derived& self = get_this();
	assert(self.n_rows == 1 || self.n_cols == 1);
	
	Derived result = self;
	return result /= norm();
}

/****************************************************************************************
*  Name     : norm
*  Function : Returns the Euclidian (2-norm) in case of vectors, and Frobenius norm in
*			  case of matrices.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::norm() const
{
	Derived& self = get_this();
	T norm = (T)0;
	if (self.n_rows == 1)
	{
		for (size_t j = 0; j < self.n_cols; j++)
		{
			norm += self(0, j) * self(0, j);
		}
	} 
	else if (self.n_cols == 1)
	{
		for (size_t i = 0; i < self.n_rows; i++)
		{
			norm += self(i, 0) * self(i, 0);
		}
	} 
	else
	{
		for (size_t i = 0; i < self.n_rows; i++)
		{
			for (size_t j = 0; j < self.n_cols; j++)
			{
				norm += self(i, j) * self(i, j);
			}
		}
	}
	assert(norm >= 0);
	norm = (T)sqrt(norm);
	return norm;
}

/****************************************************************************************
*  Name     : cwise_product
*  Function : Basically the dot product between matching column vectors/scalars in each
*			  matrix/vector object.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::cwise_product(
	const Derived &other 								// In: Matrix/Vector object to apply columnwise product to
) const
{
	Derived result(1, n_cols);
	for (size_t j = 0; j < n_cols; j++)
	{
		result(0, j) = (T)0;
		for (size_t i = 0; i < n_rows; i++)
		{
			result(0, j) += *this(i, j) * other(i, j);	
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : cwise_mean
*  Function : Calculates the mean columnwise.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::cwise_mean() const
{
	Derived result(1, n_cols);
	for (size_t j = 0; j < n_cols; j++)
	{
		result(j) = (T)0;
		for (size_t i = 0; i < n_rows; i++)
		{
			result(j) += *this(i, j);
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
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::rwise_mean() const
{
	Derived result(n_rows, 1);
	for (size_t i = 0; i < n_rows; i++)
	{
		result(i) = (T)0;
		for (size_t j = 0; j < n_cols; j++)
		{
			result(i) += *this(i, j);
		}
		result(i) /= (T)n_rows;
	}
}

/****************************************************************************************
*  Name     : exp
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::exp() const
{
	Derived result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result(i, j) = exp(result(i, j));		
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : log
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::log() const
{
	Derived result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result(i, j) = log(result(i, j));		
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : identity
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::identity(
	const size_t n_rows,  										// In: Amount of matrix rows
	const size_t n_cols 										// In: Amount of matrix columns
	)
{
	Matrix_Base<T, Derived> result(n_rows, n_cols);
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
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::ones(
	const size_t n_rows,  										// In: Amount of matrix rows
	const size_t n_cols 										// In: Amount of matrix columns
	)
{
	Derived result(n_rows, n_cols);
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
template <class T, class Derived>
__host__ __device__  void Matrix_Base<T, Derived>::set_block(
	const size_t start_row, 									// In: Start row of matrix block
	const size_t start_col, 									// In: Start column of matrix block
	const size_t n_rows,  										// In: Amount of rows
	const size_t n_cols, 										// In: Amount of columns
	const Derived &block 									// In: Block matrix to set
	)
{
	assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && 
			start_row < this->n_rows && start_col < this->n_cols);

	assert(block.get_rows() == n_rows && block.get_cols() == n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			*this(start_row + i, start_col + j) = block(i, j);
		}
	}
}

/****************************************************************************************
*  Name     : set_row
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::set_row(
	const size_t row,		 											// In: Index of row to assign
	const Derived &vector 									// In: Row vector to assign to the row
	)
{
	assert(vector.get_cols() == n_cols && row < n_rows);
	for (size_t j = 0; j < n_cols; j++)
	{
		*this(row, j) = vector(j);
	}
}

/****************************************************************************************
*  Name     : set_col
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::set_col(
	const size_t col,		 											// In: Index of column to assign
	const Derived &vector 									// In: Column vector to assign to the column
	)
{
	assert(vector.get_rows() == n_rows && col < n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		*this(i, col) = vector(i);
	}
}

/****************************************************************************************
*  Name     : set_zero
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::set_zero()
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			*this(i, j) = (T)0;
		}
	}
}

/****************************************************************************************
*  Name     : set_ones
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::set_ones()
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			*this(i, j) = (T)1;
		}
	}
}

/****************************************************************************************
*  Name     : set_all_coeffs
*  Function : Sets all values of the matrix/vector to the coefficient coeff
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::set_all_coeffs(const T coeff)
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			*this(i, j) = coeff;
		}
	}
}

/****************************************************************************************
*  Name     : get_block
*  Function : returns the n_rows x n_cols block of this object, with upper left reference
*			  index (start_row, start_col).
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::get_block(
	const size_t start_row, 									// In: Start row of matrix block
	const size_t start_col, 									// In: Start column of matrix block
	const size_t n_rows,  										// In: Amount of rows
	const size_t n_cols 										// In: Amount of columns
	) const
{
	assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && n_rows > 0 && n_cols > 0 && 
			start_row < n_rows && start_col < n_cols);

	Derived result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result(i, j) = *this(start_row + i, start_col + j);
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
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::get_row(
	const size_t row											// In: Index of row to fetch
	) const
{
	assert(row < this->n_rows);

	Derived result(1, n_cols);
	for (size_t j = 0; j < n_cols; j++)
	{
			result(0, j) = *this(row, j);
	}
	return result;
}

/****************************************************************************************
*  Name     : get_col
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ Derived Matrix_Base<T, Derived>::get_col(
	const size_t col											// In: Index of column to fetch
	) const
{
	assert(col < this->n_cols);

	Derived result(n_rows, 1);
	for (size_t i = 0; i < n_rows; i++)
	{
			result(i, 0) = *this(i, col);
	}
	return result;
}

/****************************************************************************************
*  Name     : max_coeff
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::max_coeff() const
{
	T max = data[0][0];
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			if (max < *this(i, j))
			{
				max = *this(i, j);
			}
		}
	}
	return max;
}

/****************************************************************************************
*  Name     : min_coeff
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::min_coeff() const
{
	T min = data[0][0];
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			if (min > *this(i, j))
			{
				min = *this(i, j);
			}
		}
	}
	return min;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : calculate_determinant_recursive
*  Function : Private function to use when calculating the matrix determinant
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ T Matrix_Base<T, Derived>::calculate_determinant_recursive(
	const Derived &submatrix 									// In: Matrix to calculate determinant of
	) const
{
	T det = (T)0;
	if (submatrix.n_rows == 1)
	{
		det = submatrix(0, 0);
		return det;
	}
	if (submatrix.n_rows == 2)
	{
		det = submatrix(0, 0) * submatrix(1, 1) - submatrix(0, 1) * submatrix.data(1, 0);
		return det;
	}

	Derived temp_minor(submatrix.n_rows - 1);

	for (size_t i = 0; i < submatrix.n_rows; i++)
	{
		fill_minor_matrix(submatrix, temp_minor, 0, i);

		det += (i % 2 == 1 ? (T)-1 : (T)1) * submatrix.data[0][i] * calculate_determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * submatrix[0][i] * calculate_determinant_recursive(temp_minor);
	}

	return det;
}

/****************************************************************************************
*  Name     : fill_minor_matrix
*  Function : Private function to use for calculating the minor M_{row, col} for the input
*			  based on the member data.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, class Derived>
__host__ __device__ void Matrix_Base<T, Derived>::fill_minor_matrix(
	Derived &minor_matrix, 										// In/out: Matrix to fill  as the minor M_{row, col}
	const Derived &original_matrix, 							// In: Original matrix to extract the minor from, of one order higher than the minor matrix
	const size_t row,  											// In: Row index of minor
	const size_t col 											// In: Column index of minor
	) const
{
	assert(original_matrix.n_rows == original_matrix.n_cols);

	// Indicate which col and row is being copied to the minor
    size_t col_count = 0, row_count = 0;
 
    for(size_t i = 0; i < original_matrix.n_rows; i++)
    {
        if (i != row)
        {
            col_count = 0;
            for(size_t j = 0; j < original_matrix.n_rows; j++ )
            {
                if (j != col)
                {
					minor_matrix(row_count, col_count) = original_matrix(i, j);
					
                    col_count++;
                }
            }
            row_count++;
        }
    }
}
/*****************************************************************************************************************/

//=========================================================================================================
// Dynamically sized matrix template
//=========================================================================================================
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

	__host__ __device__ void resize(const size_t n_rows, const size_t n_cols);

	__host__ __device__ void conservative_resize(const size_t n_rows, const size_t n_cols);
    
};

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
				result(i, j) += self(i, k) * other(k, j);
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
			result(i, j) = self(j, i);
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
			result(i, j) = self(j, i);
		}
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
			resized(i, j) = *this(i, j);
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
			*this(i, j) = other(i, j);
		}
	}
}

//=========================================================================================================
// Fixed sized matrix template
//=========================================================================================================
template <class T, int Rows, int Cols>
class Static_Matrix : public Matrix_Base<T, Static_Matrix<T, Rows, Cols>> 
{
private:
	size_t n_rows = Rows, n_cols = Cols;

	T data[Rows * Cols];

	__host__ __device__ void assign_data(const Static_Matrix &other);

public:

    __host__ __device__ Static_Matrix() : n_rows(Rows), n_cols(Cols) {}

	__host__ __device__ Static_Matrix(const Static_Matrix &other);

	__host__ __device__ Static_Matrix& operator=(const Static_Matrix &rhs);

	__host__ __device__ Static_Matrix operator*(const Static_Matrix &other) const;

	__host__ __device__ Static_Matrix transposed() const;
};

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

/****************************************************************************************
*  Name     : operator*
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T, int Rows, int Cols>
__host__ __device__  Static_Matrix<T, Rows, Cols>  Static_Matrix<T, Rows, Cols>::operator*(
	const  Static_Matrix<T, Rows, Cols> &other 							// In: Matrix/vector to multiply with
	) const
{	
	// Verify that the matrix product is valid
	assert(n_cols == other.n_rows);

	Static_Matrix<T, n_rows, other.n_cols> result;
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < other.n_cols; j++)
		{
			result(i, j) = (T)0;
			for (size_t k = 0; k < n_cols; k++)
			{
				result(i, j) += self(i, k) * other(k, j);
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
__host__ __device__ Static_Matrix<T, Rows, Cols> Static_Matrix<T, Rows, Cols>::transposed() const
{
	Static_Matrix<T, Rows, Cols> result;
	for (size_t i = 0; i < n_cols; i++)
	{
		for (size_t j = 0; j < n_rows ; j++)
		{
			result(i, j) = self(j, i);
		}
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
			*this(i, j) = other(i, j);
		}
	}
}



//=========================================================================================================
// TYPEDEFS
//=========================================================================================================
using CMatrixXd = Dynamic_Matrix<double>;
using CMatrix2d = Static_Matrix<double, 2, 2>;
using CMatrix3d = Static_Matrix<double, 3, 3>;
using CMatrix4d = Static_Matrix<double, 4, 4>;

// Column vectors are standard here
using CVector2d = Static_Matrix<double, 2, 1>;
using CVector3d = Static_Matrix<double, 3, 1>;
using CVector4d = Static_Matrix<double, 4, 1>;


#endif