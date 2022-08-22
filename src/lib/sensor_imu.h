// sensor_imu.h
/*

	This header file outlines the specific implementations for the various sensor types 
	within the localization filter apparatus.  This is intended to encapsulate the imu
	-specific settings and functions not unlike an inherited class in C++

*/
/* $Id: sensor_imu.h,v 1.4 2005/06/10 03:38:44 dave Exp $ */

#ifndef KIN_MODEL_H
#include "kin_model.h"
#endif

#ifndef STATE_VECTOR_H
#include "state_vector.h"
#endif

#ifndef POSEMATH_H
#include "posemath.h"
#endif

#ifndef TRANSDUCER_H
#include "transducer.h"
#endif

#ifndef SENSOR_H
#include "sensor.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H


// imu data IDL struct

typedef struct 
{
	//! Seconds of the time value.
  long sec;
  //! Microseconds of the time value.
  long usec;
}TimeIDL ;

/*
		transducer values 16
		
	0: quaternion[0]
	1: quaternion[1]
	2: quaternion[2]
	3: quaternion[4]
	4: magfield[0]
	5: magfield[1]
	6: magfield[2]
	7: accel[0]   x component
	8: accel[1]   y component
	9: accel[2]   z component
	10:angrate[0] x component
	11:angrate[1] y component
	12:angrate[2] z component
	13:angle[0]
	14:angle[1]
	15:angle[2]

*/


typedef struct 
{
 	//!Structure containing time the IMU was sampled
	TimeIDL time;

	/*!	4 component quaternion which describes the orientation of the IMU with respect
  	 	to the fixed earth coordinate system. The earth fixed coordinate
			system has X pointing North, Y pointing East, and Z pointing down. */
	double quaternion[4];

  /*!	Vector (X, Y and Z components) quantifying the direction and
			magnitude of the instantaneously measured magnetic field that
			the IMU is exposed expressed in terms of the IMU's local coordinate system.*/
  double magfield[3];

	/*!	Vector (X, Y and Z components) quantifying the direction and
			magnitude of the instantaneously measured acceleration expressed in terms of the IMU
			local coordinate system.*/            
	double accel[3];

  /*!	Vector (X, Y and Z components) quantifying the rate of rotation of
			the IMU expressed in terms of the IMU local coordinate system.*/                
  double angrate[3];

  /*!	Vector (X, Y and Z components) quantifying the roll, pitch and yaw
			of the IMU expressed in terms of the IMU local coordinate system.*/
  double angle[3];

  /*!	The 3x3 orientation matrix describes the orientation of the IMU
			expressed in terms of the IMU local coordinate system.*/
  double orientmatrix[3][3];
	
}ImuIDL;




// FCN Declarations 
// imu statevector generation function
int ImuGenerateStateVector( void *in, state_vector *out);  
	
// imu vel matrix generation function
int ImuGenerateVelMatrix( void *in, vel_matrix * out);  
	
// imu update fcn for data received					
int	ImuUpdateSensor( size_t  size_data, void *in, void *out);



	
#endif


#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
