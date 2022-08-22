//! filter.c
//!
//! DRE 2005/06/16
//!
//! Kalman filter Functions
/* $Id: filter.c,v 1.2 2005/06/06 18:53:08 dave Exp $ */


/*

	This c file holds the functions for the basic Kalman filter to be 
	used inside the localization sensor data structs.  This filter is
	an observer which has a very simple state transition matrix (identity)
	so essentially it filters transducers within a single sensor.
	
	The first version is based on a linear stochastic system model
	and a discrete time recursive a priori formulation of the Kalman filter.

	BLAS and LAPACK are used to perform the linear algebra operations.
	
	x_hat_ is x_hat a priori 			(predictive estimate)
	x_hat  is x_hat a posteriori	(actual estimate)

*/


#ifdef __cplusplus
extern "C" {
#endif

#include "filter.h"


//!-------------------------------------------------------
//! CONSTRUCTORS
//!-------------------------------------------------------
k_filter * CreateKFilter( void )
{
	
	//! assign dynamic memory
	return( (k_filter *) malloc(sizeof(k_filter)));

}//! end CreateKFilter

//!-------------------------------------------------------
//! Init Fcns - dynamically create and clear
//!-------------------------------------------------------
//! create the sub elements of the kalman filter
int InitKFilter ( k_filter *out, int size_matrices )
{
	//! create the sub elements of the kalman filter
	int i,j;
	
	//! allocate size to the data structr
	out->num_elements = size_matrices;
	
	//! create an appropriate array size for x_hat and y
	
	//! create x_hat
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->x_hat = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create x_hat_
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->x_hat_ = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
		
	//! create y
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->y = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// y_hat
	//! create y_hat
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->y_hat = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	// y_hat_
	//! create y_hat
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->y_hat_ = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}	
	//! pivot_table
	//! create pivot_table
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->pivot_table = (int *) malloc( size_matrices * sizeof( int ));		
	}
	else
	{
		return -1;
	}
	// u
	//! create u
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->u = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	
	// Ax
	//! create Ax
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Ax = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

		// Bu
	//! create Bu
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Bu = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
																	
	//! create memory for each transducer row in matrices

	//! create A matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->A = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create B matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->B = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create C matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->C = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	//! create G matrix - create identity matrix for computation
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->G = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	
	//! create K matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->K = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	//! create P matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->P = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create Q matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Q = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create R matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->R = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create CPC matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CPC = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}


	//! create I matrix - create identity matrix for computation
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->I = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// CPC_R
	//! create CPC_R matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CPC_R = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	
	//  PC
	//! create PC matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->PC = (double *) malloc( size_matrices * size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// QG
	//! create GQG matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->QG = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// GQG
	//! create GQG matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->GQG = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// CP
	//! create CP matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CP = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// AK
	//! create AK matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->AK = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// zero values
	ZeroKFilter ( out );
			
	
	return 0;
}

/*
	InitKFilter2 allows a Kalman filter to be inserted into another object and apply
	filtering to the data elements within it.  It requires a data input, ptr_measurement
	and a data output, ptr_filtered to place the  filter output values.
*/
int InitKFilter2 ( k_filter *out, int size_matrices, double * ptr_measurement, double * ptr_filtered )
{
	//! create the sub elements of the kalman filter with 
	//! assigned measurement and filtered arrays
	int i,j;
	
	// y will aim at ptr_measurement for meassured values
	// x_hat will aim at filterd for the filtered measurement
	
	//! allocate size to the data structr
	out->num_elements = size_matrices;
	
	//! create an appropriate array size for x_hat and y
	
	//! aim  x_hat_ at filtered output
	out->x_hat = ptr_filtered;

	//! create x_hat_
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->x_hat_ = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
		
	//! aim y at data measurement input
	out->y = ptr_measurement;
	
	// y_hat
	//! create y_hat
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->y_hat = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	// y_hat_
	//! create y_hat
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->y_hat_ = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}	
	//! pivot_table
	//! create pivot_table
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->pivot_table = (int *) malloc( size_matrices * sizeof( int ));		
	}
	else
	{
		return -1;
	}
	// u
	//! create u
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->u = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	
	
	// Ax
	//! create Ax
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Ax = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	

		// Bu
	//! create Bu
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Bu = (double *) malloc( size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
																	
	//! create memory for each transducer row in matrices

	//! create A matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->A = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create B matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->B = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create C matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->C = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	//! create G matrix - create identity matrix for computation
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->G = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	
	//! create K matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->K = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	//! create P matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->P = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create Q matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->Q = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create R matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->R = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	//! create CPC matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CPC = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}


	//! create I matrix - create identity matrix for computation
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->I = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// CPC_R
	//! create CPC_R matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CPC_R = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	
	//  PC
	//! create PC matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->PC = (double *) malloc( size_matrices * size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// QG
	//! create GQG matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->QG = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// GQG
	//! create GQG matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->GQG = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// CP
	//! create CP matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->CP = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}
	
	// AK
	//! create AK matrix
	if ( (size_matrices) > 0 )//! so long as there are transducers set
	{
		//! create array pointers
		out->AK = (double *) malloc( size_matrices *size_matrices * sizeof( double ));		
	}
	else
	{
		return -1;
	}

	// zero values
	ZeroKFilter ( out );
			
	
	return 0;
}

//!-------------------------------------------------------
//! Zero Fcns
//!-------------------------------------------------------
//! zero the elements of a pre-existing kalman filter
int ZeroKFilter ( k_filter *out)
{
	int i, j;
	//! zero x_hat
/*	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->x_hat[i] = 0.0;		
	}	
*/
	//! zero x_hat_
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->x_hat_[i] = 0.0;		
	}
	//! zero y
/*	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->y[i] = 0.0;		
	}	
*/
	//! zero y_hat
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->y_hat[i] = 0.0;		
	}	
	//! zero y_hat_
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->y_hat_[i] = 0.0;		
	}		
	//! zero u
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->u[i] = 0.0;		
	}		
	//! zero pivot_table
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->pivot_table[i] = 1;		
	}
	//! zero Ax
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->Ax[i] = 0.0;		
	}		
	
	//! zero Bu
	for (i = 0; i < out->num_elements; i++ )
	{
		//! zero entry
		out->Bu[i] = 0.0;		
	}	
	//! zero A matrix
	//! identity for A matrix
	SetIdentityMatrix ( out->A, out->num_elements );
	//! identity for B matrix
	SetIdentityMatrix ( out->B, out->num_elements );
	//! identity for C matrix
	SetIdentityMatrix ( out->C, out->num_elements );

	//! zero G matrix
	//! identity for G matrix
	SetIdentityMatrix ( out->G, out->num_elements );
	//! zero Kalman gain matrix
	//! identity for K matrix
	SetIdentityMatrix ( out->K, out->num_elements );	
	//! zero P matrix
/*	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->P[i + out->num_elements*j] = 0.0;		
			}
	}
*/	
	SetIdentityMatrix ( out->P, out->num_elements );	
	//! zero Q matrix
/*	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->Q[i + out->num_elements*j] = 0.0;		
			}
	}*/
	SetIdentityMatrix ( out->Q, out->num_elements );	
	//! zero R matrix
	//! identity for R matrix
	SetIdentityMatrix ( out->R, out->num_elements );	
	//! zero CPC matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->CPC[i + out->num_elements*j] = 0.0;		
			}
	}	
	//! zero I matrix
	//! identity for I matrix
	SetIdentityMatrix ( out->I, out->num_elements );
	
	//! zero CPC_R matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->CPC_R[i + out->num_elements*j] = 0.0;		
			}
	}			

	//! zero PC matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->PC[i + out->num_elements*j] = 0.0;		
			}
	}
	// QG
	//! zero QG matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->GQG[i + out->num_elements*j] = 0.0;		
			}
	}		
	//! zero GQG matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->GQG[i + out->num_elements*j] = 0.0;		
			}
	}	
	//! zero CP matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->CP[i + out->num_elements*j] = 0.0;		
			}
	}
	//! zero AK matrix
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				out->AK[i + out->num_elements*j] = 0.0;		
			}
	}


	return 0;
}

	
//!-------------------------------------------------------
//! Get/Set Fcns
//!-------------------------------------------------------
//! sets the measured values in the y array
int SetKFilterMeasured (  k_filter *out, double * array, size_t size_array )
{
	int i;

	if ( size_array == ( out->num_elements * sizeof(double) ))// if the proper size
	{
		for (i = 0; i < out->num_elements; i++ )
		{
			// copy the value over to y array
			out->y[i] = array[i];
		
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the measured values in the A matrix
int SetKFilterAMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->A[i + out->num_elements*j] = *( ptr_double );
				//printf(" %f ", out->A[i + out->num_elements*j] );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the measured values in the B matrix
int SetKFilterBMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->B[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}

//! sets the measured values in the C matrix
int SetKFilterCMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->C[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the values in the noise input matrix
int SetKFilterGMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	
	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->G[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the measured values in the Kalman gain  matrix
int SetKFilterKMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->K[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the measured values in the P matrix
int SetKFilterPMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->P[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the measured values in the Q matrix
int SetKFilterQMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;
	
	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->Q[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}
//! sets the values in the R matrix
int SetKFilterRMatrix (  k_filter *out, double *array, size_t size_array )
{
	int i,j;
	double *ptr_double;

	
	// test for proper data size (no mem leaks)
	if ( size_array == ( out->num_elements * out->num_elements * sizeof(double) ))// if the proper size
	{
		//assign double pointer to array
		ptr_double = (double *)array;
		
		for (i = 0; i < out->num_elements; i++ )
		{
			for (j = 0; j < out->num_elements; j++ )
			{
				// copy the value over to K matrix
				out->R[i + out->num_elements*j] = *( ptr_double );
				// increment pointer
				ptr_double++;
			}
		}
	
	}
	else
	{
		return -1;
	}

	return 0;
}


//!-------------------------------------------------------
//! Update Fcns
//!-------------------------------------------------------



//!-------------------------------------------------------
//! Transform Fcns
//!-------------------------------------------------------



//!-------------------------------------------------------
//! Compute Fcns
//!-------------------------------------------------------


/*
		ComputeKFilterKMatrix computes the Kalman gain from the
		covariance and output matrices.
		
		Equations from Lewis, Applied Optimal Control & Estimation, 1992
		
		This function uses the previous state of elements and matrices generated in the
		other computation equations.
			
*/
int ComputeKFilterKMatrix(  k_filter *out )
{
	// This fcn uses the BLAS and LAPACK fcns for the major
	// linear algebra operations
	// scalar factors for BLAS
	double alpha = 1.0; // include matrix
	double beta	 = 0.0; // exclude matrix
	int i,j;
	// need a spare matrix as a temprary data storage - 
	// two options - generic spare inside each sensor or
	// dynamically allocate a new one each iteration.
	
	// compute P trans(C)
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasTrans,
								/* 	m x n */(const int)out->num_elements,	(const int)out->num_elements,
								/* 	k */(const int)out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->P, 
								/*  leading dimension of A */(const int)out->num_elements,
								/*  matrix B */(const double *)out->C,
								/*  leading dimension of B */(const int)out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  Matrix C */(double *)out->PC,
								/*  leading dimension of C */(const int)out->num_elements);
	

	
	// compute CP trans(C)
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */(const int)out->num_elements,	(const int)out->num_elements,
								/* 	k */(const int)out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->C, 
								/*  leading dimension of A */(const int)out->num_elements,
								/*  matrix B */(const double *)out->PC,
								/*  leading dimension of B */(const int)out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  Matrix C */(double *)out->CPC,
								/*  leading dimension of C */(const int)out->num_elements);

	// add (R + CP trans(C))
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */(const int)out->num_elements,	(const int)out->num_elements,
								/* 	k */(const int)out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->R, 
								/*  leading dimension of A */(const int)out->num_elements,
								/*  matrix B */(const double *)out->I,
								/*  leading dimension of B */(const int)out->num_elements,
								/*  scaling factor for C */(const double)alpha,
								/*  matrix C */(double *)out->CPC,
								/*  leading dimension of C */(const int)out->num_elements);	

	
	// copy contents of CP trans(C) to CPC_R as clapack_dgesv stores factorization
	// to matrix A
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */(const int)out->num_elements,	(const int)out->num_elements,
								/* 	k */(const int)out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->CPC, 
								/*  leading dimension of A */(const int)out->num_elements,
								/*  matrix B */(const double *)out->I,
								/*  leading dimension of B */(const int)out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->CPC_R,
								/*  leading dimension of C */(const int)out->num_elements);	
								
	// make CPC into identity matrix
	SetIdentityMatrix ( out->CPC, out->num_elements );

	//! inverse ( R + CP trans(C) )
	//! Operation required to be CblasColMajor to return the inverted matrix in the 
	//! Values in pivot table don't matter, they are replaced by the fcn.	
  clapack_dgesv ( /* 	which leading dimension to consider row or col */CblasColMajor, 
									/*  size of matrices  */(const int)out->num_elements, 
									/*  NRHS must be as great as M or n*/ (const int)out->num_elements,
									/*  matrix A */(double *)(out->CPC_R), 
									/*  leading dimension of A */(const int)out->num_elements,
									/* 	pivot table for columns */(int *)out->pivot_table,
									/*  matrix B */(double *)(out->CPC),
									/*  leading dimension of B */(const int)out->num_elements);
/*									
	printf ( "CPC \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				printf( "%f \n", out->CPC[i + out->num_elements*j]);		
			}
	}	
	printf ( "CPC_R \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				printf( "%f \n", out->CPC_R[i + out->num_elements*j]);		
			}
	}	
*/	
	// CPC now contains inv( R + CP trans(C) )
	// PC holds PC	
	  
	// Finally compute K gain matrix and store in K
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */(const int)out->num_elements,		(const int)out->num_elements,
								/* 	k */(const int)out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->PC, 
								/*  leading dimension of A */(const int)out->num_elements,
								/*  matrix B */(const double *)out->CPC,
								/*  leading dimension of B */(const int)out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->K,
								/*  leading dimension of C */(const int)out->num_elements);	

/*	printf ( "K \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				printf( "%f \n", out->K[i + out->num_elements*j]);		
			}
	}			
*/

	return 0;
}

/*

	KFilterPMatrix computes the updated covariance matrix from previous 
	estimates

	Equations from Lewis, Applied Optimal Control & Estimation, 1992
	
	This function uses the previous state of elements and matrices generated in the
	other computation equations.
	
	
*/
int ComputeKFilterPMatrix(  k_filter *out )
{

	// computes the covariance matrix
	// scalar factors for BLAS
	double alpha = 1.0; // include matrix
	double beta	 = 0.0; // exclude matrix
	int i,j;


	// cblas_dgemm cannot be used with beta = 0 and apply the matrix C as either A or B
	// within the computation.  It appears that cblas_dgemm compute beta*c first and this alters
	// the result before A*B can be computed
	// To solve the above problem, I copy the contents of the previous factor into 
	// temporary storage in an unneeded matrix to compute the result properly.  
	// In most cases, CPC_R is used in this function as temporary storage.

	// compute Qtrans(G)
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->Q, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->G,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->GQG,
								/*  leading dimension of C */out->num_elements);	
	
									
	// copy GQG into QG
	CopyMatrix( out->GQG, out->QG, out->num_elements );
	
	// compute GQtrans(G)
	/* Compute C = alpha*AB + beta*C */
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->G, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->QG,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->GQG,
								/*  leading dimension of C */out->num_elements);

	
	// compute CP
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->C, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->P,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);	


	// copy CP into CPC_R and use as CP
	CopyMatrix( out->CP, out->CPC_R, out->num_elements );


	// compute PC inv( R + CP trans(C) ) * CP and store in CP
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->K, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->CPC_R,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);	

	// compute (P - Ptrans(C)*(inv( R + CP trans(C) ) * CP))	and store in	CP																
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->P, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->I,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)-1.0*alpha,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);
										
	// copy CP into CPC_R and use as CP
	CopyMatrix( out->CP, out->CPC_R, out->num_elements );
																						
	// compute (P - Ptrans(C)*(inv( R + CP trans(C) ) * CP))*trans(A)	and store in	CP	
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->CPC_R, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->A,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);
	
	// copy CP into CPC_R and use as CP
	CopyMatrix( out->CP, out->CPC_R, out->num_elements );

	// compute A*(P - Ptrans(C)*(inv( R + CP trans(C) ) * CP))*trans(A)	and store in	CP
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->A, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->CPC_R,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);
								
	// add  A*(P - Ptrans(C)*(inv( R + CP trans(C) ) * CP))*trans(A) + GQtrans(G) and store in	CP
  cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */out->num_elements,		out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->GQG, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->I,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)alpha,
								/*  matrix C */(double *)out->CP,
								/*  leading dimension of C */out->num_elements);				

	
	// store result CP in P
	CopyMatrix( out->CP, out->P, out->num_elements );		
	
/*	printf ( "P \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				printf( "%f \n", out->P[i + out->num_elements*j]);		
			}
	}			
*/	
		
	return 0;
}

/*
	
	ComputeKFilterEstimate updates the estimated state of the system 
	based on the update and measurement equations of the Kalman filter

	Equations from Lewis, Applied Optimal Control & Estimation, 1992
	
	This function uses the previous state of elements and matrices generated in the
	other computation equations.	
	
*/
int ComputeKFilterAPrioriEstimate( k_filter *out )
{



	// x_hat_ is x_hat a priori
	// x_hat  is x_hat a posteriori
	// scalar factors for BLAS
	double alpha = 1.0; // include matrix
	double beta	 = 0.0; // exclude matrix
	int i,j;
	
	// cblas_dgemm cannot be used with beta = 0 and apply the matrix C as either A or B
	// within the computation.  It appears that cblas_dgemm compute beta*c first and this alters
	// the result before A*B can be computed
	// To solve the above problem, I copy the contents of the previous factor into 
	// temporary storage in an unneeded matrix to compute the result properly.  
	// In most cases, CPC_R is used in this function as temporary storage for dgemm.
	// dgemv has the same problem, using x_hat_ as the copy storage for x_hat
	
	
	// copy x_hat into x_hat_ (previous estimate from last time step)
	for ( i = 0; i < out->num_elements; i++) 
	{
			out->x_hat_[i] = out->x_hat[i];
	}
			
	// compute ( Cx_hat_(k)) and store in a posteriori x_hat
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */2,		2,
//                /*  scaling factor for A */(const double)-1.0*alpha,
//								/*  matrix A */(const double *)out->C, 
//								/*  leading dimension of A */2,
//								/*  vector x */(const double *)out->x_hat_,
//								/*  increment of x */2,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->x_hat,
//								/*  increment of y */1);
	// WORKAROUND 
	MultiplyMatrixVector ( out->C ,  out->x_hat_ , out->x_hat , out->num_elements  );
	
	// compute y - C*x_hat and store in x_hat
	for ( i = 0; i < out->num_elements; i++) 
	{
			out->x_hat[i] = out->y[i] - out->x_hat_[i];
	}

	// compute AK and store in AK
	cblas_dgemm ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
								/*  Transpose A or not */	CblasNoTrans, 
								/*  Transpose B or not */ CblasNoTrans,
								/* 	m x n */(const int) out->num_elements,	(const int)out->num_elements,
								/* 	k */out->num_elements,
                /*  scaling factor for A */(const double)alpha,
								/*  matrix A */(const double *)out->A, 
								/*  leading dimension of A */out->num_elements,
								/*  matrix B */(const double *)out->K,
								/*  leading dimension of B */out->num_elements,
								/*  scaling factor for C */(const double)beta,
								/*  matrix C */(double *)out->AK,
								/*  leading dimension of C */out->num_elements);
/*	printf ( "AK \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			for (j = 0; j < out->num_elements; j++ )
			{
				//! zero entry
				printf( "%f \n", out->AK[i + out->num_elements*j]);		
			}
	}			
*/
	// copy x_hat into Ax for temp storage
	for ( i = 0; i < out->num_elements; i++) 
	{
			out->Ax[i] = out->x_hat[i];
	}

														
	// compute AK(y - Cx_hat_(k)) and store in x_hat
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */(const int) out->num_elements,	(const int)out->num_elements,
//               /*  scaling factor for A */(const double)alpha,
//								/*  matrix A */(const double *)out->AK, 
//								/*  leading dimension of A */(const int)out->num_elements,
//								/*  vector x */(const double *)out->x_hat_,
//								/*  increment of x */(const int)out->num_elements,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->x_hat,
//								/*  increment of y */(const int)out->num_elements);
	// WORKAROUND 
	MultiplyMatrixVector ( out->AK ,  out->Ax , out->x_hat , out->num_elements  );
		
								
	// compute Ax_hat_(k) and store in Ax
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */(const int)out->num_elements,	(const int)out->num_elements,
//                /*  scaling factor for A */(const double)alpha,
//								/*  matrix A */(const double *)out->A, 
//								/*  leading dimension of A */(const int)out->num_elements,
//								/*  vector x */(const double *)out->x_hat_,
//								/*  increment of x */(const int)out->num_elements,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->Ax,
//								/*  increment of y */(const int)out->num_elements);	
	// WORKAROUND 
	MultiplyMatrixVector ( out->A ,  out->x_hat_ , out->Ax , out->num_elements  );
	
		
	// compute Bu and store in Bu
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */2,		2,
//                /*  scaling factor for A */(const double)alpha,
//								/*  matrix A */(const double *)out->B, 
//								/*  leading dimension of A */2,
//								/*  vector x */(const double *)out->u,
//								/*  increment of x */2,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->Bu,
//								/*  increment of y */(const int)out->num_elements);	
	// WORKAROUND 
	MultiplyMatrixVector ( out->B ,  out->u, out->Bu , out->num_elements  );											
								
	// add Ax and Bu and store in Ax
	for (i = 0; i < out->num_elements; i++)							
	{
			out->Ax[i] += out->Bu[i];
	}
	// compute x_hat_(k+1) and store in x_hat
	for (i = 0; i < out->num_elements; i++)							
	{
			out->x_hat[i] += out->Ax[i];
	}	
	
	

	return 0;
}
/*
	
	ComputeKFilterEstimate updates the estimated state of the system 
	based on the update and measurement equations of the Kalman filter

	Equations from Lewis, Applied Optimal Control & Estimation, 1992
	
	This function uses the previous state of elements and matrices generated in the
	other computation equations.	
	
*/
int ComputeKFilterAPosterioriEstimate(  k_filter *out )
{


	// x_hat_ is x_hat a priori
	// x_hat  is x_hat a posteriori
	// scalar factors for BLAS
	double alpha = 1.0; // include matrix
	double beta	 = 0.0; // exclude matrix
	int i,j;

	// compute the a posteriori estimate of the state vector
	
	// compute Cx_hat(k+1)) and store in y_hat
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */2,		2,
//               /*  scaling factor for A */(const double)-1.0*alpha,
//								/*  matrix A */(const double *)out->C, 
//								/*  leading dimension of A */2,
//								/*  vector x */(const double *)out->x_hat,
//								/*  increment of x */2,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->y_hat,
//								/*  increment of y */2);	
	// WORKAROUND 
	MultiplyMatrixVector ( out->C ,  out->x_hat, out->y_hat , out->num_elements  );								
	// difference y and y_hat
	for (i = 0; i < out->num_elements; i++)							
	{
			out->y_hat[i] -= out->y[i];
	}	
	// copy y_hat into y_hat_
	for (i = 0; i < out->num_elements; i++)							
	{
			out->y_hat_[i] = out->y_hat[i];
	}		
	// compute K(y(k+1) - Cx_hat(k+1)) and store in y_hat
	/* Compute y = alpha*(Ax) + beta*(y) */
//  cblas_dgemv ( /* 	which leading dimension to consider row or col */CblasRowMajor, 
//								/*  Transpose A or not */	CblasNoTrans, 
//								/* 	m x n */2,		2,
//                /*  scaling factor for A */(const double)1.0*alpha,
//								/*  matrix A */(const double *)out->K, 
//								/*  leading dimension of A */2,
//								/*  vector x */(const double *)out->y_hat_,
//								/*  increment of x */2,
//								/*  scaling factor for y */(const double)beta,
//								/*  vector y */(double *)out->y_hat,
//								/*  increment of y */1);	
	// WORKAROUND 
	MultiplyMatrixVector ( out->K ,  out->y_hat_, out->y_hat , out->num_elements  );	
	
	// compute x_hat(k+1) = x_hat(k+1) + K(y(k+1) - Cx_hat(k+1)) and store in x_hat
	for (i = 0; i < out->num_elements; i++)							
	{
			out->x_hat[i] += out->y_hat[i];
	}	 

/*	printf ( "x_hat= \n");								
	for (i = 0; i < out->num_elements; i++ )
	{
			//! zero entry
			printf( "%f \n", out->x_hat[i]);		

	}
*/
	return 0;
}
/*
	
	ComputeKFilter is the over-arching Kalman filter computation function
	
*/
int ComputeKFilter( k_filter *out )
{
	// This is the meta algorithm that controls the computation of the Kalman filter 
	// current time step
	
	// compute K gain
	ComputeKFilterKMatrix( out );

	// compute estimate recursion / predictive estimate
	ComputeKFilterAPrioriEstimate( out );
	
	// compute  Covariance Recursion
	ComputeKFilterPMatrix( out );
	
	// compute state vector estimate
	ComputeKFilterAPosterioriEstimate( out );
	
	
	return 0;
}


#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
