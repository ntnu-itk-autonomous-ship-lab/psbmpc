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

#ifndef _CMATRIX_CUH_
#define _CMATRIX_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>
template<class T>
class CMatrix
{
private:

	size_t n_rows, n_cols;
	
	T **data;

	__host__ __device__ void allocate_data();

	__host__ __device__ void deallocate_data();

	__host__ __device__ void assign_data(const CMatrix &other);

	__host__ __device__ T calculate_determinant_recursive(const CMatrix &submatrix) const;

	__host__ __device__ void fill_minor_matrix(const CMatrix &original_matrix, CMatrix &minor_matrix, const size_t row, const size_t col) const;
	
public:

	__host__ __device__ CMatrix() {}

	__host__ __device__ CMatrix(const size_t n_rows);

	__host__ __device__ CMatrix(const size_t n_rows, const size_t n_cols);

	__host__ __device__ CMatrix(const CMatrix &other);

	__host__ __device__ ~CMatrix();

	__host__ __device__ CMatrix& operator=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator+(const CMatrix &other) const;
	__host__ __device__ CMatrix operator+(const T &scalar) const;

	__host__ __device__ CMatrix& operator+=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator-(const CMatrix &other) const;
	__host__ __device__ CMatrix operator-(const T &scalar) const;

	__host__ __device__ CMatrix& operator-=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator*(const CMatrix &other) const;
	__host__ __device__ CMatrix operator*(const T &scalar) const;

	__host__ __device__ CMatrix& operator*=(const T &scalar);

	__host__ __device__ CMatrix operator/(const T &scalar) const;

	__host__ __device__ CMatrix& operator/=(const T &scalar);

	__host__ __device__ bool operator==(const CMatrix &rhs) const;

	__host__ __device__ T& operator()(const size_t row, const size_t col) const { assert(row < n_rows && col < n_cols) ;return data[row][col]; }

	__host__ __device__ T& operator()(const size_t index) const;

	//__host__ __device__ CMatrix& row(const size_t row) { assert(row < n_rows); return data[row]; }

	__host__ __device__ CMatrix& col(const size_t row);

	__host__ __device__ void transpose();

	__host__ __device__ CMatrix transposed() const;

	__host__ __device__ T determinant() const;

	__host__ __device__ CMatrix inverse() const;

	__host__ __device__ T dot(const CMatrix &other) const;

	__host__ __device__ void normalize();

	__host__ __device__ CMatrix normalized() const;

	__host__ __device__ T norm() const;

	__host__ __device__ CMatrix cwise_product(const CMatrix &other);

	__host__ __device__ CMatrix exp() const;

	__host__ __device__ CMatrix log() const;

	__host__ __device__ CMatrix& block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

	__host__ __device__ CMatrix identity(const size_t n_rows, const size_t, n_cols) const;

	__host__ __device__ void set_zero();

	__host__ __device__ void set_ones();

	__host__ __device__ void set_all_coeffs(const T coeff);

	__host__ __device__ size_t get_rows() const { return n_rows; }

	__host__ __device__ size_t get_cols() const { return n_cols; }

	__host__ __device__ T** get_data() const { return data; }

	__host__ __device__ T max_coeff() const;

	__host__ __device__ T min_coeff() const;

	__host__ __device__ void resize(const size_t n_rows, const size_t n_cols);

	__host__ __device__ void conservative_resize(const size_t n_rows, const size_t n_cols);

	__host__ friend std::ostream& operator<<(std::ostream& os, const CMatrix<T> &cm)
	{
		for (size_t i = 0; i< cm.n_rows; i++)
		{
			for (size_t j = 0; j< cm.n_cols; j++)
			{
				os << cm.data[i][j] << ' '; 
			}
			os << '\n';
		}
		return os;
	}
};

/****************************************************************************************
*  Global operator functions, to allow for commutativeness
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> operator+(const T &scalar, const CMatrix<T> &other){ return other + scalar;}

template <class T>
__host__ __device__ CMatrix<T> operator-(const T &scalar, const CMatrix<T> &other)	{ return -other + scalar; }

template <class T>
__host__ __device__ CMatrix<T> operator*(const T &scalar, const CMatrix<T> &other)	{ return other * scalar; }

/****************************************************************************************
*  Class member functions
*****************************************************************************************/
/****************************************************************************************
*  Name     : CMatrix
*  Function : Class constructor, initializes parameters and variables
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T>::CMatrix(
	const size_t n_rows 										// In: Amount of matrix rows
	) :
	n_rows(n_rows), n_cols(n_rows), data(nullptr)
{
	allocate_data();
}

template <class T>
__host__ __device__ CMatrix<T>::CMatrix(
	const size_t n_rows,  										// In: Amount of matrix rows
	const size_t n_cols 										// In: New amount of matrix columns
	) :
	n_rows(n_rows), n_cols(n_cols), data(nullptr)
{
	allocate_data();
}

template <class T>
__host__ __device__ CMatrix<T>::CMatrix(
	const CMatrix<T> &other 									// In: Matrix/vector to copy
	) :
	data(nullptr)
{
	assign_data(other);
}

/****************************************************************************************
*  Name     : ~CMatrix
*  Function : Class destructor
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T>::~CMatrix()
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
__host__ __device__ CMatrix<T>& CMatrix<T>::operator=(
	const CMatrix<T> &rhs 										// In: Right hand side matrix/vector to assign
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
*  Name     : operator+
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator+(
	const CMatrix<T> &other 									// In: Matrix/vector to add by
	) const
{
	assert(n_rows == other.n_rows);
	assert(n_cols == other.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			result.data[i][j] = this->data[i][j] + other.data[i][j];
		}
	}
	return result;
}

template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator+(
	const T &scalar 										// In: Scalar to add by
	) const
{
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			result.data[i][j] = this->data[i][j] + scalar;
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
template <class T>
__host__ __device__ CMatrix<T>& CMatrix<T>::operator+=(
	const CMatrix<T> &rhs 										// In: Right hand side matrix/vector to add by
	)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			this->data[i][j] += rhs.data[i][j];
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
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator-(
	const CMatrix<T> &other 									// In: Matrix/vector to subtract by
	) const
{
	assert(n_rows == other.n_rows);
	assert(n_cols == other.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			result.data[i][j] = this->data[i][j] - other.data[i][j];
		}
	}
	return result;
}

template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator-(
	const T &scalar 										// In: Scalar to subtract by
	) const
{
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			result.data[i][j] = this->data[i][j] - scalar;
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
template <class T>
__host__ __device__ CMatrix<T>& CMatrix<T>::operator-=(
	const CMatrix<T> &rhs 										// In: Right hand side matrix/vector to subtract by
	)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			this->data[i][j] -= rhs.data[i][j];
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
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator*(
	const CMatrix<T> &other 									// In: Matrix/vector to multiply with
	) const
{	
	// Verify that the matrix product is valid
	assert(n_cols == other.n_rows);

	CMatrix<T> result(n_rows, other.n_cols);
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < other.n_cols; j++)
		{
			result.data[i][j] = 0;
			for (size_t k = 0; k < n_cols; k++)
			{
				result.data[i][j] += this->data[i][k] * other.data[k][j];
			}
		}
	}
	return result;
}

template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator*(
	const T &scalar 											// In: scalar to multiply with
	) const
{
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result.data[i][j] = scalar * this->data[i][j];
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
template <class T>
__host__ __device__ CMatrix<T>& CMatrix<T>::operator*=(
	const T &scalar 											// In: scalar to multiply with
	)
{	
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			this->data[i][j] *= scalar;
		}
	}
	return *this;
}

/****************************************************************************************
*  Name     : operator/
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator/(
	const T &scalar 											// In: scalar to divide by
	) const
{	
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result.data[i][j] = this->data[i][j] / scalar;
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
template <class T>
__host__ __device__ CMatrix<T>& CMatrix<T>::operator/=(
	const T &scalar 											// In: Right hand side scalar to divide by
	)
{
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			this->data[i][j] /= scalar;
		}
	}
	return *this;
}

/****************************************************************************************
*  Name     : operator==
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ bool CMatrix<T>::operator==(
	const CMatrix<T> &rhs 										// In: Right hand side matrix/vector to compare with
	) const
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	bool result = true;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols ; j++)
		{
			if ((*this)(i, j) != rhs(i, j))
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
template <class T>
__host__ __device__ T& CMatrix<T>::operator()(
	const size_t index 										// In: Index of element to fetch
	) const
{
	assert(n_rows == 1 || n_cols == 1);

	if (n_rows == 1)
	{
		assert(index < n_cols);
		return data[0][index];
	} 
	else
	{
		assert(index < n_rows);
		return data[index][0];
	}
	
}

/****************************************************************************************
*  Name     : transpose
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::transpose()
{
	CMatrix<T> result(n_cols, n_rows);
	for (size_t i = 0; i < n_cols; i++)
	{
		for (size_t j = 0; j < n_rows ; j++)
		{
			result.data[i][j] = this->data[j][i];
		}
	}
	*this = result;
}

/****************************************************************************************
*  Name     : transpose
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::transposed() const
{
	CMatrix<T> result(n_cols, n_rows);
	for (size_t i = 0; i < n_cols; i++)
	{
		for (size_t j = 0; j < n_rows ; j++)
		{
			result.data[i][j] = this->data[j][i];
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : determinant
*  Function : Works for square matrices only
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ T CMatrix<T>::determinant() const
{
	assert(n_rows == n_cols);

	if (n_rows == 1)
	{
        return data[0][0];
	}
	if (n_rows == 2)
	{
		return data[0][0] * data[1][1] - data[0][1] * data[1][0];
	}

    T det = 0;
 
	// allocate the cofactor matrix
	CMatrix<T> temp_minor(n_rows - 1);

    for(size_t i = 0; i < n_rows; i++)
    {
        // get minor of element (0,i)
        fill_minor_matrix(*this, temp_minor, 0, i);
 
        det += (i % 2 == 1 ? -1.0 : 1.0) * data[0][i] * calculate_determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * data[0][i] * calculate_determinant_recursive(temp_minor);
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
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::inverse() const
{
	assert(n_rows == n_cols);
	assert(determinant() > T(0));

	T det_inv = (T)1 / determinant();

	CMatrix<T> result(n_rows);

	if (n_rows == 1)
	{
		result.data[0][0] = det_inv;
		return result;
	}
	if (n_rows == 2)
	{
		result.data[0][0] = this->data[1][1];
		result.data[0][1] = - this->data[0][1];
		result.data[1][0] = - this->data[1][0];
		result.data[1][1] = this->data[0][0];
		result *= det_inv;
		return result;
		//result = result * det_inv;
	}    
 
    CMatrix<T> temp_minor(n_rows - 1);
 
    for(size_t i = 0; i < n_rows; i++)
    {
        for(size_t j = 0; j < n_rows; j++)
        {
			// Fill values for minor M_ji
			fill_minor_matrix(*this, temp_minor, j, i);

			// Element ij of the inverse is the co-factor C_ji divided by the matrix determinant
			// where C_ij = (-1)^(i + j) * |M_ij|
			
			result.data[i][j] = ((j + i) % 2 == 1 ? (T)-1 : (T)1) * det_inv * calculate_determinant_recursive(temp_minor);
			
			/* if ((i + j) % 2 == 1)
			{
				result.data[i][j] = - result.data[i][j];
			} */
        }
	}
	return result;
}

/****************************************************************************************
*  Name     : dot
*  Function : Only valid for vectors, i.e. CMatrix with either n_rows = 1 or n_cols = 1
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ T CMatrix<T>::dot(
	const CMatrix<T> &other 									// In: Vector to dot with
	) const
{
	assert((n_rows == other.n_rows && n_cols == other.n_cols) && (n_rows == 1 || n_cols == 1));

	T result = (T)0;
	if (n_rows == 1) 	
	{ 
		for (size_t i = 0; i < n_cols; i++)
		{
			result += this->data[0][i] * other.data[0][i];
		}
	} 
	else
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			result += this->data[i][0] * other.data[i][0];
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
template <class T>
__host__ __device__ void CMatrix<T>::normalize()
{
	assert(n_rows == 1 || n_cols == 1);
	*this /= norm();
}

/****************************************************************************************
*  Name     : normalized
*  Function : Vectors only. Returns the normalized vector without modifying this.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::normalized() const
{
	assert(n_rows == 1 || n_cols == 1);
	
	CMatrix<T> result = *this;
	return result /= norm();
}

/****************************************************************************************
*  Name     : norm
*  Function : Returns the Euclidian (2-norm) in case of vectors, and Frobenius norm in
*			  case of matrices.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ T CMatrix<T>::norm() const
{
	T norm = (T)0;
	if (n_rows == 1)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			norm += data[0][j] * data[0][j];
		}
	} 
	else if (n_cols == 1)
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			norm += data[i][0] * data[i][0];
		}
	} 
	else
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				norm += data[i][j] * data[i][j];
			}
		}
	}
	assert(norm >= 0);
	norm = (T)sqrt(norm);
	return norm;
}

/****************************************************************************************
*  Name     : block
*  Function : returns the n_rows x n_cols block of this object, with upper left reference
*			  index (start_row, start_col).
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T>& CMatrix<T>::block(
	const size_t start_row, 									// In: Start row of matrix block
	const size_t start_col, 									// In: Start column of matrix block
	const size_t n_rows,  										// In: New amount of rows
	const size_t n_cols 										// In: New amount of columns
	) const
{
	assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && n_rows > 0 && n_cols > 0 && 
			start_row < n_rows && start_col < n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = start_row; i < start_row + n_rows; i++)
	{
		for (size_t j = start_col; j < start_col + n_cols; j++)
		{
			result.data[i][j] = this->data[i][j];
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
template <class T>
__host__ __device__ void CMatrix<T>::identity(
	const size_t n_rows,  										// In: Amount of matrix rows
	const size_t n_cols 										// In: New amount of matrix columns
	) const
{
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result.data[i][j] = (T)0;
			if (i == j)
			{
				result.data[i][j] = (T)1;
			}
			
		}
	}
}

/****************************************************************************************
*  Name     : set_zero
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::set_zero()
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			data[i][j] = (T)0;
		}
	}
}

/****************************************************************************************
*  Name     : set_ones
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::set_ones()
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			data[i][j] = (T)1;
		}
	}
}

/****************************************************************************************
*  Name     : set_all_coeffs
*  Function : Sets all values of the matrix/vector to the coefficient coeff
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::set_all_coeffs(const T coeff)
{
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			data[i][j] = coeff;
		}
	}
}

/****************************************************************************************
*  Name     : max_coeff
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ T CMatrix<T>::max_coeff() const
{
	T max = data[0][0];
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			if (max < data[i][j])
			{
				max = data[i][j];
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
template <class T>
__host__ __device__ T CMatrix<T>::min_coeff() const
{
	T min = data[0][0];
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			if (min > data[i][j])
			{
				min = data[i][j];
			}
		}
	}
	return min;
}

/****************************************************************************************
*  Name     : resize
*  Function : Resizes the CMatrix without keeping old data
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::resize(
	const size_t n_rows,  										// In: New amount of rows
	const size_t n_cols 										// In: New amount of columns
	)
{
	assert(n_rows > 0 && n_cols > 0);

	*this = CMatrix<T>(n_rows, n_cols);
}

/****************************************************************************************
*  Name     : conservative_resize
*  Function : Resizes the CMatrix, keeping the old data
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::conservative_resize(
	const size_t n_rows,  										// In: New amount of rows
	const size_t n_cols 										// In: New amount of columns
	)
{
	assert(n_rows > 0 && n_cols > 0);
	
	CMatrix<T> resized(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			resized(i, j) = this->data[i][j];
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
__host__ __device__ void CMatrix<T>::allocate_data()
{
	assert(n_rows > 0 && n_cols > 0);
	data = new T*[n_rows];
	for (size_t i = 0; i < n_rows; i++)
	{
		data[i] = new T[n_cols];
	}
}

/****************************************************************************************
*  Name     : deallocate_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::deallocate_data()
{
	if (data == nullptr)
	{
		return;
	}

	for (size_t i = 0; i < n_rows; i++)
	{
		delete[] data[i];
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
__host__ __device__ void CMatrix<T>::assign_data(
	const CMatrix<T> &other 										// In: Matrix whose data to assign to *this;
	)
{
	n_rows = other.n_rows;
	n_cols = other.n_cols;

	allocate_data();

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			data[i][j] = other.data[i][j];
		}
	}
}

/****************************************************************************************
*  Name     : calculate_determinant_recursive
*  Function : Private function to use when calculating the matrix determinant
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ T CMatrix<T>::calculate_determinant_recursive(
	const CMatrix<T> &submatrix 									// In: Matrix to calculate determinant of
	) const
{
	T det = (T)0;
	if (submatrix.n_rows == 1)
	{
		det = submatrix.data[0][0];
		return det;
	}
	if (submatrix.n_rows == 2)
	{
		det = submatrix.data[0][0] * submatrix.data[1][1] - submatrix.data[0][1] * submatrix.data[1][0];
		return det;
	}

	CMatrix<T> temp_minor(submatrix.n_rows - 1);

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
template <class T>
__host__ __device__ void CMatrix<T>::fill_minor_matrix(
	const CMatrix<T> &original_matrix, 							// In: Original matrix to extract the minor from, of one order higher than the minor matrix
	CMatrix<T> &minor_matrix, 									// In/out: Matrix to fill  as the minor M_{row, col}
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
					minor_matrix.data[row_count][col_count] = original_matrix.data[i][j];
					
                    col_count++;
                }
            }
            row_count++;
        }
    }
}

#endif