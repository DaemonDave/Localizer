// matrix.h
// marix function Header File
// structs and fcns to create and use state_vectors
/* $Id: matrix.h,v 1.3 2005/06/28 03:30:49 dave Exp $ */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MATRIX_H
#define MATRIX_H


//-------------------------------------------------------
// Fcn Declarations
//-------------------------------------------------------
//! Set matrix values to an identity matrix
int SetIdentityMatrix (double * out, int size);
//! copy contents from in into out assuming equal size
int CopyMatrix(double *in, double *out, int size);
// simple matrix vector multiplication
int MultiplyMatrixVector (double *in_matrix, double *in_vector, double *out_vector, int n  );

#endif  // define MATRIX_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
