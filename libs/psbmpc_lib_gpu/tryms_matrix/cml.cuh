/****************************************************************************************
*
*  File name : cml.cuh
*
*  Function  : Header file for a simple Cuda Matrix Library (CML)
*			   used inside the Cuda kernels for the PSB-MPC GPU calculations. Can contain 
*			   any normal data type such as double, float, int, bool etc. 
*			   The implementation is based on CRTP. Storage is row-major.
*
*  			   A Matrix_Base is here defined which is commonly used by the dynamic and 
*			   static matrix classes. 
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

#ifndef _CML_CUH_
#define _CML_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>

#include "dynamic_matrix.cuh"
#include "static_matrix.cuh"
#include "eigen_interface.cuh"

namespace CML
{
	//=========================================================================================================================
	// Matrix base that defines common matrix/vector functionality
	//=========================================================================================================================
	template<class T, class Derived>
	class Matrix_Base
	{
	private:

		__host__ __device__ T calculate_determinant_recursive(const Dynamic_Matrix<T> &submatrix) const;

		__host__ __device__ void fill_minor_matrix(Dynamic_Matrix<T> &minor_matrix, const Derived &original_matrix, const size_t row, const size_t col) const;
		
	public:

		__host__ __device__ Derived operator+(const Derived &other) const;

		__host__ __device__ Derived& operator+=(const Derived &rhs);

		__host__ __device__ Derived operator-(const Derived &other) const;

		__host__ __device__ Derived& operator-=(const Derived &rhs);

		__host__ __device__ Derived& operator*=(const T scalar);

		__host__ __device__ Derived operator/(const T scalar) const;

		__host__ __device__ Derived& operator/=(const T scalar);

		__host__ __device__ bool operator==(const Derived &rhs) const;

		__host__ __device__ T& operator()(const size_t index) const;

		__host__ __device__ inline T& operator()(const size_t row, const size_t col) const
		{
			Derived& self = get_this();
			assert(row < self.get_rows() && col < self.get_cols()); 

			return (T&) self.get_data()[self.get_cols() * row + col]; 
		}

		__host__ __device__ inline T& operator[](const size_t index) const { Derived& self = get_this(); return self(index); }

		__host__ __device__ inline operator T() const { Derived& self = get_this(); return self(0, 0); }

		__host__ __device__ T determinant() const;

		__host__ __device__ Derived inverse() const;

		__host__ __device__ T dot(const Derived &other) const;

		__host__ __device__ void normalize();

		__host__ __device__ Derived normalized() const;

		__host__ __device__ T norm() const;

		__host__ __device__ Derived exp() const;

		__host__ __device__ Derived log() const;

		__host__ __device__ void set_zero();

		__host__ __device__ void set_ones();

		__host__ __device__ void set_all_coeffs(const T coeff);

		__host__ __device__ inline Derived& get_this() const { return (Derived&)*this; }

		__host__ __device__ T max_coeff() const;

		__host__ __device__ T min_coeff() const;

		__host__ friend inline std::ostream& operator<<(std::ostream& os, const Derived &other)
		{
			for (size_t i = 0; i < other.get_rows(); i++)
			{
				for (size_t j = 0; j< other.get_cols(); j++)
				{
					os << other(i, j) << ' '; 
				}
				os << '\n';
			}
			return os;
		}
	};

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
		if (self.get_rows() < other.get_rows() || self.get_cols() < other.get_cols())
		{
			return other + self;
		}

		assert((self.get_rows() == other.get_rows() && self.get_cols() == other.get_cols()) 	|| 
				(self.get_rows() == other.get_rows() && other.get_cols() == 1) 					||
				(self.get_cols() == other.get_cols() && other.get_rows() == 1));

		Derived result = self;
		
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols() ; j++)
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
	template <class T, class Derived>
	__host__ __device__ Derived& Matrix_Base<T, Derived>::operator+=(
		const Derived &rhs 										// In: Right hand side matrix/vector to add by
		)
	{
		Derived& self = get_this();
		assert(self.get_rows() == rhs.get_rows() && self.get_cols() == rhs.get_cols());
		
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
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
		if (self.get_rows() < other.get_rows() || self.get_cols() < other.get_cols())
		{
			return other - self;
		}

		assert((self.get_rows() == other.get_rows() && self.get_cols() == other.get_cols()) 	|| 
				(self.get_rows() == other.get_rows() && other.get_cols() == 1) 					||
				(self.get_cols() == other.get_cols() && other.get_rows() == 1));

		Derived result = self;

		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols() ; j++)
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
	template <class T, class Derived>
	__host__ __device__ Derived& Matrix_Base<T, Derived>::operator-=(
		const Derived &rhs 										// In: Right hand side matrix/vector to subtract by
		)
	{
		Derived& self = get_this();
		assert(self.get_rows() == rhs.get_rows());
		assert(self.get_cols() == rhs.get_cols());

		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols() ; j++)
			{
				self(i, j) -= rhs(i, j);
			}
		}
		return self;
	}

	/****************************************************************************************
	*  Name     : operator*=
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, class Derived>
	__host__ __device__ Derived& Matrix_Base<T, Derived>::operator*=(
		const T scalar 											// In: scalar to multiply with
		)
	{	
		Derived& self = get_this();
		for (size_t i = 0 ; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
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
		const T scalar 											// In: scalar to divide by
		) const
	{	
		Derived& self = get_this();
		Derived result = self;
		for (size_t i = 0 ; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
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
		const T scalar 											// In: Right hand side scalar to divide by
		)
	{
		Derived& self = get_this();
		for (size_t i = 0 ; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
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
		Derived& self = get_this();
		assert(self.get_rows() == rhs.get_rows() && self.get_cols() == rhs.get_cols());

		bool result = true;
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols() ; j++)
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
	*  Name     : operator() with one parameter
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
		assert((self.get_rows() == 1 || self.get_cols() == 1) && (index < self.get_cols() || index < self.get_rows()));

		if (self.get_rows() == 1)
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
		assert(self.get_rows() == self.get_cols());

		if (self.get_rows() == 1)
		{
			return self(0, 0);
		}
		if (self.get_rows() == 2)
		{
			return self(0, 0) * self(1, 1) - self(0, 1) * self(1, 0);
		}

		T det = 0;
	
		// allocate the cofactor matrix
		Dynamic_Matrix<T> temp_minor(self.get_rows() - 1);

		for(size_t i = 0; i < self.get_rows(); i++)
		{
			// get minor of element (0,i)
			fill_minor_matrix(temp_minor, self, 0, i);
	
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
		const Derived& self = get_this();
		assert(self.get_rows() == self.get_cols() && determinant() > T(0));
		
		T det_inv = (T)1 / determinant();

		Derived result = self;

		if (self.get_rows() == 1)
		{
			result(0, 0) = det_inv;
			return result;
		}
		if (self.get_rows() == 2)
		{
			result(0, 0) = self(1, 1);
			result(0, 1) = - self(0, 1);
			result(1, 0) = - self(1, 0);
			result(1, 1) = self(0, 0);
			result *= det_inv;
			return result;
		}    
	
		Dynamic_Matrix<T> temp_minor(self.get_rows() - 1);
	
		for(size_t i = 0; i < self.get_rows(); i++)
		{
			for(size_t j = 0; j < self.get_rows(); j++)
			{
				// Fill values for minor M_ji
				fill_minor_matrix(temp_minor, self, j, i);

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
		assert((self.get_rows() == other.get_rows() && self.get_cols() == other.get_cols()) && 
				(self.get_rows() == 1 || self.get_cols() == 1));

		T result = (T)0;
		if (self.get_rows() == 1) 	
		{ 
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				result += self(0, j) * other(0, j);
			}
		} 
		else
		{
			for (size_t i = 0; i < self.get_rows(); i++)
			{
				result += self(i, 0) * other(i, 0);
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
		assert(self.get_rows() == 1 || self.get_cols() == 1);
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
		assert(self.get_rows() == 1 || self.get_cols() == 1);
		
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
		if (self.get_rows() == 1)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				norm += self(0, j) * self(0, j);
			}
		} 
		else if (self.get_cols() == 1)
		{
			for (size_t i = 0; i < self.get_rows(); i++)
			{
				norm += self(i, 0) * self(i, 0);
			}
		} 
		else
		{
			for (size_t i = 0; i < self.get_rows(); i++)
			{
				for (size_t j = 0; j < self.get_cols(); j++)
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
	*  Name     : exp
	*  Function : 
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template <class T, class Derived>
	__host__ __device__ Derived Matrix_Base<T, Derived>::exp() const
	{
		Derived& self = get_this();
		Derived result = self;
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
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
		Derived& self = get_this();
		Derived result = self;
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				result(i, j) = log(result(i, j));		
			}
		}
		return result;
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
		Derived& self = get_this();
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				self(i, j) = (T)0;
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
		Derived& self = get_this();
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				self(i, j) = (T)1;
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
		Derived& self = get_this();
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				self(i, j) = coeff;
			}
		}
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
		Derived& self = get_this();
		T max = self(0, 0);
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				if (max < self(i, j))
				{
					max = self(i, j);
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
		Derived& self = get_this();
		T min = self(0, 0);
		for (size_t i = 0; i < self.get_rows(); i++)
		{
			for (size_t j = 0; j < self.get_cols(); j++)
			{
				if (min > self(i, j))
				{
					min = self(i, j);
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
		const Dynamic_Matrix<T> &submatrix 									// In: Matrix to calculate determinant of
		) const
	{
		T det = (T)0;
		if (submatrix.get_rows() == 1)
		{
			det = submatrix(0, 0);
			return det;
		}
		if (submatrix.get_rows() == 2)
		{
			det = submatrix(0, 0) * submatrix(1, 1) - submatrix(0, 1) * submatrix(1, 0);
			return det;
		}

		Dynamic_Matrix<T> temp_minor(submatrix.get_rows() - 1);

		for (size_t i = 0; i < submatrix.get_rows(); i++)
		{
			fill_minor_matrix(temp_minor, submatrix, 0, i);

			det += (i % 2 == 1 ? (T)-1 : (T)1) * submatrix(0, i) * calculate_determinant_recursive(temp_minor);
			//det += pow( -1.0, i ) * submatrix(0, i) * calculate_determinant_recursive(temp_minor);
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
		Dynamic_Matrix<T> &minor_matrix, 							// In/out: Matrix to fill  as the minor M_{row, col}
		const Derived &original_matrix, 							// In: Original matrix to extract the minor from, of one order higher than the minor matrix
		const size_t row,  											// In: Row index of minor
		const size_t col 											// In: Column index of minor
		) const
	{
		assert(original_matrix.get_rows() == original_matrix.get_cols());

		// Indicate which col and row is being copied to the minor
		size_t col_count = 0, row_count = 0;
	
		for(size_t i = 0; i < original_matrix.get_rows(); i++)
		{
			if (i != row)
			{
				col_count = 0;
				for(size_t j = 0; j < original_matrix.get_cols(); j++)
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
}

#endif