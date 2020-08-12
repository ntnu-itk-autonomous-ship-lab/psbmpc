/****************************************************************************************
*
*  File name : cmatrix.cu
*
*  Function  : Implements class functions Cuda matrix container.
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

#include "cmatrix.h"
#include <assert.h>

/****************************************************************************************
*  Name     : CMatrix
*  Function : Class constructor, initializes parameters and variables
*  Method   : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ CMatrix<T>::CMatrix(const int n_rows) :
	n_rows(n_rows), n_cols(n_rows)
{
	allocate_data();
}

__host__ __device__ CMatrix<T>::CMatrix(const int n_rows, const int n_cols) :
	n_rows(n_rows), n_cols(n_cols)
{
	allocate_data();
}

__host__ __device__ CMatrix<T>::CMatrix(const CMatrix<T> &cm) :
	n_rows(cm.n_rows), n_cols(cm.n_cols)
{
	allocate_data();
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols; j++)
		{
			data[i][j] = cm.data[i][j];
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
__host__ __device__ CMatrix<T>& CMatrix<T>::operator=(const CMatrix<T> &rhs)
{
	if (this == &rhs)
	{
		return *this;
	}
	deallocate_data();

	return *this = CMatrix(rhs);
}

/****************************************************************************************
*  Name     : operator+
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ CMatrix<T> CMatrix<T>::operator+(const CMatrix<T> &other)
{
	assert(n_rows == other.n_rows);
	assert(n_cols == other.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
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
__host__ __device__ CMatrix<T>& CMatrix<T>::operator+=(const CMatrix<T> &rhs)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
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
__host__ __device__ CMatrix<T> CMatrix<T>::operator-(const CMatrix<T> &other)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
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
__host__ __device__ CMatrix<T>& CMatrix<T>::operator-=(const CMatrix<T> &rhs)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
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
__host__ __device__ CMatrix<T> CMatrix<T>::operator*(const CMatrix<T> &other)
{	
	// Verify that the matrix product is valid
	assert(n_cols == other.n_rows);

	CMatrix<T> result(n_rows, other.n_cols);
	for (int i = 0 ; i < n_rows; i++)
	{
		for (int j = 0; j < other.n_cols; j++)
		{
			for (int k = 0; k < n_cols)
			{
				result.data[i][j] += this->data[i][k] * other.data[i][k];
			}
		}
	}
	return result;
}

__host__ __device__ CMatrix<T> CMatrix<T>::operator*(const T &factor)
{
	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0 ; i < n_rows; i++)
	{
		for (int j = 0; j < rhs.n_cols; j++)
		{
			result.data[i][j] = factor * this->data[i][j];
		}
	}
	return result;
}

/****************************************************************************************
*  Name     : operator/
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ CMatrix<T> CMatrix<T>::operator/(const T &factor)
{	
	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0 ; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols; j++)
		{
			result.data[i][j] = this->data[i][k] / factor;
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
__host__ __device__ CMatrix<T>& CMatrix<T>::operator/=(const T &factor)
{
	for (int i = 0 ; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols; j++)
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
__host__ __device__ bool CMatrix<T>::operator==(const CMatrix<T> &rhs)
{
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	bool result = true;
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
		{
			if (*this(i, j) != rhs(i, j))
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
__host__ __device__ CMatrix CMatrix<T>::transpose()
{
	CMatrix<T> result(n_cols, n_rows);

	for (int i = 0; i < n_cols; i++)
	{
		for (int j = 0; j < n_rows ; j++)
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
__host__ __device__ T CMatrix<T>::determinant()
{
	assert(n_rows == n_cols);

	int dim = n_rows;
    if (dim == 1 )
        return data[0][0];
	
	if (dim == 2)
		return data[0][0] * data[1][1] - data[0][1] * data[1][0];

    T det = 0;
 
	// allocate the cofactor matrix
	CMatrix temp_minor(n_rows - 1);

    for(int i = 0; i < dim; i++)
    {
        // get minor of element (0,i)
        minor(temp_minor, 0, i , dim);
 
        det += (i % 2 == 1 ? -1.0 : 1.0) * data[0][i] * determinant_recursive(minor);
        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
    }
 
    return det;
}
}

/****************************************************************************************
*  Name     : inverse
*  Function : Works for square matrices only
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ CMatrix<T> CMatrix<T>::inverse()
{
	assert(n_rows == n_cols);

	if (n_rows == 1)
	{
		return 1.0 / data[0][0];
	}

	CMatrix result(n_rows);

	T det = (T)1 / determinant();

	if (n_rows == 2)
	{
		result.data[0][0] = this->data[1][1];
		result.data[0][1] = - this->data[0][1];
		result.data[1][0] = - this->data[1][0];
		result.data[1][1] = this->data[0][0];
		result /= det;
		return result;
		//result = result / det;
	}    
 
    float *temp = new float[(order-1)*(order-1)];
    CMatrix minor(n_rows - 1);
 
    for(size_t i = 0; i < n_rows; i++)
    {
        for(size_t j = 0; j < n_rows; j++)
        {
            // get the co-factor C_ij of this matrix
			minor(minor, i, j);
			
			result.data[i][j] = det * determinant(minor, n_rows - 1);
			
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
__host__ __device__ T CMatrix<T>::dot(const CMatrix<T> &other)
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
__host__ __device__ CMatrix<T> CMatrix<T>::block(const size_t start_row, const size_t start_col, const size_t n_rows, const size_t n_cols)
{
	assert(	n_rows <= this->n_rows && n_cols <= this->n_cols && n_rows > 0 && n_cols > 0 && 
			start_row < n_rows && start_col < n_cols && start_row >= 0 && start_col >= 0);

	CMatrix<T> result(n_rows, n_cols);
	for (size_t i = start_row; i < start_row + n_rows; i++)
	{
		for (size_t j = start_col; j < start_col + n_cols)
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
__host__ __device__ T CMatrix<T>::max_coeff()
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
__host__ __device__ T CMatrix<T>::min_coeff()
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
__host__ __device__ void CMatrix<T>::resize(const int n_rows, const int n_cols)
{
	assert(n_rows > 0 && n_cols > 0);
	
	*this = CMatrix(n_rows, n_cols);
}

/****************************************************************************************
*  Name     : conservative_resize
*  Function : Resizes the CMatrix, keeping the old data
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void CMatrix<T>::conservative_resize(const size_t n_rows, const size_t n_cols)
{
	assert(n_rows > 0 && n_cols > 0);
	
	CMatrix resized(n_rows, n_cols);
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
__host__ __device__ void CMatrix<T>::deallocate_data()
{
	if (data == nullptr)
		return;

	for (int i = 0; i < n_rows; i++)
	{
		delete[] data[i];
	}
	delete[] data;
}

/****************************************************************************************
*  Name     : calculate_determinant_recursive
*  Function : Private function to use when calculating the matrix determinant
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ T calculate_determinant_recursive(const CMatrix &sub_matrix)
{
	if (sub_matrix.n_rows == 1)
	{
		return sub_matrix.data[0][0];
	}

	T det = 0;

	CMatrix temp_minor;
	for (size_t i = 0; i < sub_matrix.n_rows; i++)
	{
		minor(temp_minor, 0, i);

		det += (i % 2 == 1 ? -1.0 : 1.0) * sub_matrix[0][i] * determinant_recursive(temp_minor);
        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
	}
}

/****************************************************************************************
*  Name     : calculate_minor
*  Function : Private function to use for calculating the minor M_{row, col}
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void calculate_minor(CMatrix &minor, const size_t row, const size_t col)
{
	assert(minor.rows() == minor.cols());

	// Indicate which col and row is being copied to the minor
    size_t col_count = 0, row_count = 0;
 
    for(size_t i = 0; i < minor.n_rows; i++)
    {
        if (i != row)
        {
            col_count = 0;
            for(size_t j = 0; j < minor.n_rows; j++ )
            {
                if (j != col)
                {
					minor.data[row_count][col_count) = data[i][j];
					
                    col_count++;
                }
            }
            row_count++;
        }
    }
}
	