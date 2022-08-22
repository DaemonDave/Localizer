// state_vector.h
// state_vector Header File
// structs and fcns to create and use state_vectors
/* $Id: state_vector.h,v 1.5 2005/06/10 03:38:44 dave Exp $ */

// Includes
#ifndef POSEMATH_H
#include "posemath.h"
#endif //POSEMATH_H


#ifdef __cplusplus
extern "C" {
#endif

#ifndef STATE_VECTOR_H
#define STATE_VECTOR_H


// Defines
#define VEL_ARRAY		6

// numerical constants
#define APPROACHING_ONE  0.99999999999999999999999999999999999



// Data structs

typedef struct
{
	double vel[VEL_ARRAY];	// Linear velocities 0=Vx  1=Vy 2=Vz 
									  			// Anguluar velocities 3=omega_x 4=omega_y 5=omega_z
} vel_matrix;							// velocity matrix of a rigid body.
  


// first attempt at state vector model, using basic elements
typedef struct
{
 PmCartesian  loc;			// Cartesian coords as location
 PmQuaternion orient; 	// quaternion orientation

} state_vector;

// Functions 

// Constructors - create data structs
state_vector *	km_CreateStateVector ( void );	// create memory and assign
vel_matrix *	km_CreateVelMatrix ( void );			// create memory and assign

// Destructors

// Zero Fcns - zero the elements
int km_ZeroStateVector ( state_vector *out );
int km_ZeroVelocityMatrix ( vel_matrix *out );

// Get/Set Functions - resets specific values into the data struct
int km_SetStateVector( double x, double y, double z, double qs, double qx, double qy, double qz, state_vector *out );
int km_SetStateVector2( state_vector in, state_vector *out );// state vector version
int km_SetVelMatrix( double vx, double vy, double vz, double omega_x, double omega_y, double omega_z, vel_matrix *out);
int km_SetVelMatrix2( vel_matrix in, vel_matrix *out);// vel matrix version

// Copy Fcns
int CopyStateVector ( state_vector *in, state_vector *out);// copy contents from in into out

// Update Fcns - updates matrix/array with current values
int	km_UpdateStateVector ( state_vector delta, double delta_t, state_vector *out );	// adds delta to pose


// Compute Fcns
int	WeighStateVectors ( state_vector in, double W, state_vector *out );	// computes blending of two state vectors 
int	WeighStateVectors2 ( state_vector in, state_vector in2,  double W, state_vector *out );// blends state vectors 1 and 2 into 3
int	WeighVelocityMatrices ( vel_matrix in, vel_matrix in2,  double W, vel_matrix *out );// weigh vel matrices
int IntegrateAcceleration (double acc, double delta_time,  double * out);  // simple uniform acceleration eqn  // compute 


#endif  // define STATE_VECTOR_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
