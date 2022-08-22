// matrix.c
//
// DRE 2005/05/18
//
// matrix Functions
/*
	
	This .C file holds matrix-relatd operations that are generic to the use of matrices

*/
/* $Id: localize.c,v 1.3 2005/06/28 03:30:49 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

// 




//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------
//! Set matrix values to an identity matrix
//! out is a pointer to a single dimension array acting as a n x n matrix
//! size is the dimension of n
int SetIdentityMatrix (double * out, int size)
{

	//! out is a pointer to a single dimension array acting as a n x n matrix
	//! size is the dimension of n
	int i,j;
	
	// row iteration
	for (i = 0; i < size; i++)
	{
		//col iteration
		for (j = 0; j < size; j++)
		{
				if ( i == j )// main diagonal
				{
					out[i + size*j]	= 1.0;
				}
				else
				{
					out[i + size*j]	= 0.0;					
				}
		
		}
	}


	return 0;
}
//! copy contents from in into out assuming equal size
int CopyMatrix(double *in, double *out, int size)
{

	//! out is a pointer to a single dimension array acting as a n x n matrix
	//! in is a pointer to a single dimension array acting as a n x n matrix	
	//! size is the dimension of n
	int i,j;
	
	// row iteration
	for (i = 0; i < size; i++)
	{
		//col iteration
		for (j = 0; j < size; j++)
		{
			// copy cell contents
			out[i + size*j]	= in[i + size*j];
		}
	}


	return 0;
}

// multiply matrix by vector and store in out vector
// assumes n x n matrix and n x 1 vectors
int MultiplyMatrixVector (double *in_matrix, double *in_vector, double *out_vector, int n  )
{
	int i, j, k;
	double temp_row;// temp storage
	double temp_int;// temp intermediate storage
	
	
	// row iterator
	for (i = 0; i < n; i ++)
	{
		// col iterator
		for ( j = 0; j < n; j++)
		{
			// clear storage
			temp_row = 0.0;
			for ( k = 0; k < n; k++)
			{
				// add intermediates			
				temp_row += in_matrix[i+ n*k] * in_vector[k];
			}
			// store in output array
			out_vector[i] = temp_row;
		}
	
	}
	
	
	return 0;
}



#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
