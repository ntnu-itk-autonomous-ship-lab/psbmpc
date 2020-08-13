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
template<class T>
class CMatrix
{
private:

	size_t n_rows, n_cols;
	
	T **data;

	__host__ __device__ void allocate_data();

	__host__ __device__ void deallocate_data();

	__host__ __device__ T calculate_determinant_recursive(const CMatrix &submatrix) const;

	__host__ __device__ void calculate_minor_matrix(CMatrix &matrix, const size_t row, const size_t col) const;
	
public:

	__host__ __device__ CMatrix() {}

	__host__ __device__ CMatrix(const size_t n_rows);

	__host__ __device__ CMatrix(const size_t n_rows, const size_t n_cols);

	__host__ __device__ CMatrix(const CMatrix &other);

	__host__ __device__ ~CMatrix();

	__host__ __device__ CMatrix& operator=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator+(const CMatrix &other) const;

	__host__ __device__ CMatrix& operator+=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator-(const CMatrix &other) const;

	__host__ __device__ CMatrix& operator-=(const CMatrix &rhs);

	__host__ __device__ CMatrix operator*(const CMatrix &other) const;
	__host__ __device__ CMatrix operator*(const T &factor) const;

	__host__ __device__ CMatrix& operator*=(const T &factor);

	__host__ __device__ CMatrix operator/(const T &factor) const;

	__host__ __device__ CMatrix& operator/=(const T &factor);

	__host__ __device__ bool operator==(const CMatrix &rhs) const;

	__host__ __device__ T& operator()(const size_t row, const size_t col) const { return data[row][col]; }

	__host__ __device__ CMatrix transpose() const;

	__host__ __device__ T determinant() const;

	__host__ __device__ CMatrix inverse() const;

	__host__ __device__ T dot(const CMatrix &other) const;

	__host__ __device__ CMatrix block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols) const;

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
	n_rows(n_rows), n_cols(n_rows)
{
	allocate_data();
}

template <class T>
__host__ __device__ CMatrix<T>::CMatrix(
	const size_t n_rows,  										// In: Amount of matrix rows
	const size_t n_cols 										// In: New amount of matrix columns
	) :
	n_rows(n_rows), n_cols(n_cols)
{
	allocate_data();
}

template <class T>
__host__ __device__ CMatrix<T>::CMatrix(
	const CMatrix<T> &other 									// In: Matrix/vector to copy
	) :
	n_rows(other.n_rows), n_cols(other.n_cols)
{
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

	n_rows = rhs.n_rows; n_cols = rhs.n_cols;
	allocate_data();

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			data[i][j] = rhs.data[i][j];
		}
	}
	
	return *this;
	//return *this = CMatrix<T>(rhs);
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
			this->data[i][j] = this->data[i][j] + rhs.data[i][j];
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
			for (size_t k = 0; k < n_cols; k++)
			{
				result.data[i][j] += this->data[i][k] * other.data[i][k];
			}
		}
	}
	return result;
}

template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::operator*(
	const T &factor 											// In: Factor to multiply with
	) const
{
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result.data[i][j] = factor * this->data[i][j];
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
	const T &factor 											// In: Factor to multiply with
	)
{	
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			this->data[i][j] *= factor;
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
	const T &factor 											// In: Factor to divide by
	) const
{	
	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			result.data[i][j] = this->data[i][j] / factor;
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
	const T &factor 											// In: Right hand side factor to divide by
	)
{
	for (size_t i = 0 ; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			this->data[i][j] /= factor;
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
*  Name     : transpose
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::transpose() const
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
        calculate_minor_matrix(temp_minor, 0, i);
 
        det += (i % 2 == 1 ? -1.0 : 1.0) * data[0][i] * calculate_determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * data[0][i] * calculate_determinant_recursive(temp_minor);
    }
 
    return det;
}

/****************************************************************************************
*  Name     : inverse
*  Function : Works for square matrices only
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
			// Fill values for minor M_ij
			calculate_minor_matrix(temp_minor, i, j);

			// Element ij of the inverse is the co-factor C_ij of this matrix
			// where C_ij = (-1)^(i + j) * |M_ij|
			
			result.data[i][j] = det_inv * calculate_determinant_recursive(temp_minor);
			
			if ((i + j) % 2 == 1)
			{
				result.data[i][j] = - result.data[i][j];
			}
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
	T result = 0;

	assert((n_rows == other.n_rows && n_cols == other.n_cols) && (n_rows == 1 || n_cols == 1));

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
*  Name     : block
*  Function : returns the n_rows x n_cols block of this object, with upper left reference
*			  index (start_row, start_col).
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ CMatrix<T> CMatrix<T>::block(
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
	if (submatrix.n_rows == 1)
	{
		return submatrix.data[0][0];
	}
	if (submatrix.n_rows == 2)
	{
		return submatrix.data[0][0] * submatrix.data[1][1] - submatrix.data[0][1] * submatrix.data[1][0];
	}

	T det = 0;

	CMatrix<T> temp_minor(submatrix.n_rows - 1);

	for (size_t i = 0; i < submatrix.n_rows; i++)
	{
		calculate_minor_matrix(temp_minor, 0, i);

		det += (i % 2 == 1 ? (T)-1 : (T)1) * submatrix.data[0][i] * calculate_determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * submatrix[0][i] * calculate_determinant_recursive(temp_minor);
	}

	return det;
}

/****************************************************************************************
*  Name     : calculate_minor_matrix
*  Function : Private function to use for calculating the minor M_{row, col} for the input
*			  based on the member data.
*  Author   : 
*  Modified :
*****************************************************************************************/
template <class T>
__host__ __device__ void CMatrix<T>::calculate_minor_matrix(
	CMatrix<T> &matrix, 										// In/out: Matrix to fill  as the minor M_{row, col}
	const size_t row,  											// In: Row index of minor
	const size_t col 											// In: Column index of minor
	) const
{
	assert(matrix.n_rows == matrix.n_cols);

	// Indicate which col and row is being copied to the minor
    size_t col_count = 0, row_count = 0;
 
    for(size_t i = 0; i < matrix.n_rows; i++)
    {
        if (i != row)
        {
            col_count = 0;
            for(size_t j = 0; j < matrix.n_rows; j++ )
            {
                if (j != col)
                {
					matrix.data[row_count][col_count] = data[i][j];
					
                    col_count++;
                }
            }
            row_count++;
        }
    }
}

#endif