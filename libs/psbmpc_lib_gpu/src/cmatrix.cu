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
	assert(n_rows == rhs.n_rows);
	assert(n_cols == rhs.n_cols);

	CMatrix<T> result(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++)
	{
		for (int j = 0; j < n_cols ; j++)
		{
			result.data[i][j] = this->data[i][j] + rhs.data[i][j];
		}
	}
	return result;
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
			result.data[i][j] = this->data[i][j] - rhs.data[i][j];
		}
	}
	return result;
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
	assert(n_cols == rhs.n_rows);

	CMatrix<T> result(n_rows, rhs.n_cols);
	for (int i = 0 ; i < n_rows; i++)
	{
		for (int j = 0; j < rhs.n_cols; j++)
		{
			for (int k = 0; k < n_cols)
			{
				result.data[i][j] += this->data[i][k] * rhs.data[i][k];
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
		for (int i = 0; i < n_cols; i++)
		{
			result += this->data[0][i] * other.data[0][i];
		}
	} 
	else
	{
		for (int i = 0; i < n_rows; i++)
		{
			result += this->data[i][0] * other.data[i][0];
		}
	}
	return result;
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
	data = new T*[n_rows];
	for (int i = 0; i < n_rows; i++)
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