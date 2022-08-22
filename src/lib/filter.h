//! filter.h
//! template Header File
//! structs and fcns to create and use Kalman filter
/*! $Id: filter.h,v 1.2 2005/06/06 18:53:08 dave Exp $ */

/*

	This filter is formulated to compensate for the stochastic data
	of a sensor.  The formulation is an observer of the internal state
	of the sensor and the initial state transition matrix will be identity

*/

//! Includes
#include <stdlib.h>

// linear algebra routines
#include <cblas.h>	// BLAS Level 1 2 3 
#include <clapack.h>	// Matrix inversions and other higher level Linear algebra functions
#include "matrix.h"	// for Identity matrix fcns

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FILTER_H
#define FILTER_H


//! Defines

#define DATA_TYPE_LENGTH	36

//! Data structs


/*!
 
	First version- discrete-time stochastic Kalman filter 
	Uses linear estimator with measurement and update steps.
	Recursive a priori formulation
	This formulation can be sized to meet the sensor transducer array.
	Given that the system is a sensor, the following assumptions are made:
	1. control input array is nil, therefore B matrix is nil.
	2. each sensor assumed linear stochastic
	3. G matrix is identity
	
	
	Matrix operations are conducted by two external libraries
	
	CBLAS 	= C-wrapped Basic Linear Algebra Subroutines in Fortran 77 (vector and matrix multiply)
	CLAPACK	= C-wrapped Linear Algebra Package (matrix inversions)
	
	The array order used by BLAS is X(i) = x_i or the ith element (from 0,...,n-1)
	
	The matrix order used is 2D arrays ans the arrangement of rows and columns is 
	Rows x Columns : 
	
	X (1,3) is Row1 Column3
	Row--->
	C
	o
	l
	|
	|
	|
	\/
	
	The first row is read in first and then the second etc. until (n-1th)
	
	Some BLAS and LAPACK functions read from and write to the A matrix so that it 
	should be a copy.  Various functions return important values within the altered
	A matrix (dgesv from LAPACK returns the LU factorization for the A matrix)
	
*/


//! data struct Kalman filter
typedef struct
{

	//! array members

	//! number elements  dimension of the matrix
	int	num_elements;
				
	//! internal state array of conditional means for transducers
	//! a posteriori state estimate
	double *x_hat;		
	//! a priori state estimate
	double *x_hat_;
	//! measured data of the transducers
	double *y;		
	// estimated delta y measurement
	double *y_hat;
	// prefactor to final estimate of y
	double *y_hat_;
	//! control input vector u
	double *u;	
	//!  table of pivots for matrix multiplication
	int *pivot_table;		

	// computation members
	//! Ax 	element
	double *Ax;
	//! Bu element
	double *Bu;
	
	//! matrix members
	double *A;		//! state transition matrix
	double *B;		//! control input matrix
	double *C;		//! output matrix
	double *G;		//! noise input matrix
	double *K;		//! Kalman gain matrix	
	double *P;		//! state covariance matrix
	double *Q;		//! process/system covariance matrix
	double *R;		//! measurement noise covariance matrix
	
	// computation matrix members - stored elements for use in computation
	//! holds the CPC element
	double *CPC;	
	//! Identity Matrix of sufficient size
	double *I;		
	//!                       
	//! holds inv(CPtrans(C) + R)
	double *CPC_R;
	//! holds Ptrans(C)
	double *PC;
	//! holds Qtrans(G)
	double *QG;
	//! holds GQtrans(G)
	double *GQG;	
	//! holds temporary value during P (covariance) computation
	double *CP;
	// ! holds AK matrix factor during Estimate recursion
	double *AK; 
	
} k_filter; 


//! Functions 

//! Constructors - create data structs
k_filter * CreateKFilter( void );	//! creates and returns dynamic memory

//! Init Fcns
//! create the sub elements of the kalman filter and zero them
int InitKFilter ( k_filter *out, int size_matrices );
//! create the sub elements of the kalman filter with 
//! assigned measurement and filtered arrays
int InitKFilter2 ( k_filter *out, int size_matrices, double * ptr_measurement, double * ptr_filtered );
//! Destructors

//! Zero Fcns - zero the elements
//! zero the elements of a pre-existing kalman filter
int ZeroKFilter ( k_filter *out);

//! Get/Set Functions - resets specific values into the data struct

//! sets the measured values in the y array
int SetKFilterMeasured (  k_filter *out, double * array, size_t size_array );
//! sets the measured values in the A matrix
int SetKFilterAMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the values in the B matrix
int SetKFilterBMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the values in the C matrix
int SetKFilterCMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the measured values in the noise input matrix
int SetKFilterGMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the values in the Kalman gain matrix
int SetKFilterKMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the measured values in the P matrix
int SetKFilterPMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the values in the Q matrix
int SetKFilterQMatrix (  k_filter *out, double *array, size_t size_array );
//! sets the values in the R matrix
int SetKFilterRMatrix (  k_filter *out, double *array, size_t size_array );

//! Update Fcns - updates matrix/array with current values

//! Compute Fcns

//! compute the kalman gain matrix
int ComputeKFilterKMatrix(  k_filter *out );
//! compute the covariance matrix 
int ComputeKFilterPMatrix(  k_filter *out );
//! compute the predictive state estimate
int ComputeKFilterAPrioriEstimate(  k_filter *out );
//! compute the current state estimate
int ComputeKFilterAPosterioriEstimate(  k_filter *out );

#endif  //! define FILTER_H

#ifdef __cplusplus
} /*! matches extern "C" for C++ */
#endif
