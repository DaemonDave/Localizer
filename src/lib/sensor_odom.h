// sensor_odom.h
/*

	This header file outlines the specific implementations for the various sensor types 
	within the localization filter apparatus.  This is intended to encapsulate the odom
	-specific settings and functions not unlike an inherited class in C++

*/
/* $Id: sensor_odom.h,v 1.5 2005/06/10 18:32:55 dave Exp $ */



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

#ifndef SENSOR_ODOM_H
#define SENSOR_ODOM_H

#define METRES_IN_KM			1000.0
#define SECONDS_IN_HOUR		3600.0 

// Data struct


/*
		transducer values
		
	0: leftDistance
	1: rightDistance
	2: leftSpeed
	3: rightSpeed

*/
typedef struct 
{	
	//!Distance the left Wheel has travelled since startup (Meters)
	double leftDistance;

	//!Distance the left Wheel has travelled since startup (Meters)
	double rightDistance;

	//!Speed of the left Wheel (KM/HR) -- need to be converted to m/s
	double leftSpeed;

	//!Speed of the right Wheel (KM/HR) -- need to be converted to m/s
	double rightSpeed;
}WheelDataIDL;



// FCN Declarations 
// odom statevector generation function
int OdomGenerateStateVector( void *in, state_vector *out);  
	
// odom vel matrix generation function
int OdomGenerateVelMatrix( void *in, vel_matrix * out);  
	
// odom update fcn for data received					
int	OdomUpdateSensor( size_t  size_data, void *in, void *out);



#endif  // define SENSOR_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
