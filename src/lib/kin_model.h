// kin_model.h
// Kinematic Model Header File
// structs and fcns to create and use kinematic model abstracts
/* $Id: kin_model.h,v 1.3 2005/06/06 18:53:07 dave Exp $ */


#include <stdlib.h>	// malloc
#ifndef POSEMATH_H
#include "posemath.h"
#endif //POSEMATH_H  			// PmCartesian and PmQuaternion data structs and fcns

#ifndef STATE_VECTOR_H
#include "state_vector.h"
#endif		// state_vector and velocity_matrix data and fcns

#ifdef __cplusplus
extern "C" {
#endif

#ifndef KIN_MODEL_H
#define KIN_MODEL_H




// matrix defines
#define ROW_SIZE		3
#define COL_SIZE		3
#define ROW_SIZE_U 	4


// DATA STRUCTS



typedef struct 
{

	double  entry [ROW_SIZE][COL_SIZE];

} rot_matrix;// rotation matrix based on quaternions

typedef struct 
{

	double  entry[ROW_SIZE][COL_SIZE];

} skew_matrix;// skew symmetric matrix 

typedef struct
{
	
	double entry[ROW_SIZE_U][COL_SIZE];

} U_matrix;// quaternion Jacobian submatrix


typedef struct
{

	rot_matrix R;	// Rotation matrix applied to linear velocities
	U_matrix   U;	// U matrix applied to angular velocities
	
} Jacobian;




// FUNCTIONS
// Constructors - create data structs

skew_matrix * km_CreateSkewMatrix ( void );			// create memory and assign
rot_matrix *	km_CreateRotMatrix ( void );			// create memory and assign
U_matrix *	km_CreateUMatrix ( void );					// create memory and assign
Jacobian * km_CreateJacobian ( void );					// create memory and assign

// Zero fcns - zero the elements

int km_ZeroSkewMatrix ( skew_matrix *out );
int km_ZeroRotMatrix ( rot_matrix *out );
int km_ZeroUMatrix ( U_matrix *out );
int km_ZeroJacobian ( Jacobian *out );

// Get/Set Functions - resets specific values into the data struct
// Update Fcns - updates matrix/array with current values
int km_UpdateSkewMatrix ( PmQuaternion q, skew_matrix *out );				// fcn places quaternion entries in matrix 
int	km_UpdateRotMatrix ( PmQuaternion q, rot_matrix *out );					// creates rotation matrix based on quaternions
int	km_UpdateUMatrix ( PmQuaternion q, U_matrix *out );							// creates U matrix based on quaternions
int km_UpdateJacobian ( PmQuaternion q, Jacobian *out  ); 					// constructs Jacobian from primitives


// Compute fcns
// individual sub functions compute portions of the Jacobian matrix
int km_MultRotMat_StateVec( rot_matrix R, vel_matrix v, state_vector *out );
int km_MultUMat_StateVec( U_matrix U, vel_matrix v, state_vector *out); 
int km_ComputeStateVector( Jacobian J, vel_matrix v, state_vector *out);

// Kinematic Model Functions
int km_ComputeKinematicModel ( Jacobian *J, vel_matrix v, double delta_t, state_vector *delta, state_vector *out  );




#endif

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif




