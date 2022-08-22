//!localize.h
//!localize Header File
//!structs and fcns to create and use Localize
/* $Id: localize.h,v 1.2 2005/06/10 03:38:44 dave Exp $ */

//!Includes
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

#ifndef SENSOR_GPS_H
#include "sensor_gps.h"
#endif

#ifndef SENSOR_IMU_H
#include "sensor_imu.h"
#endif

#ifndef SENSOR_ODOM_H
#include "sensor_odom.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

#ifndef LOCALIZE_H
#define LOCALIZE_H


//!Defines

#define DATA_TYPE_LENGTH	36

#define GPS_SENSOR 				0
#define IMU_SENSOR				1
#define	ODOM_SENSOR				2

//!Data structs
//!predefined sensor types

//! localize data struct
typedef struct
{
	//!Sensors involved are external objects
	//!that are pointed to from this one.
	//!This allows the sensors to be changed and 
	//!initiated without altering this struct
	sensor *ptr_gps;
	
	sensor *ptr_odom;
	
	sensor *ptr_imu;
	
	//!Jacobian 
	Jacobian jacob;
	Jacobian *ptr_jacob;
	
	//!state vectors	
	state_vector  fused_state;
	state_vector *ptr_fused_state;
	
	//!sensor state vectors
	state_vector  gps_state;
	state_vector  imu_state;
	state_vector  odom_state;
	
	state_vector  fused_delta_state;
	state_vector  previous_fused_state;
	state_vector *ptr_fused_delta_state;	
	
	//!velocity matrix
	vel_matrix 		fused_vel_matrix;
	vel_matrix   *ptr_fused_vel_matrix;
	
	//!sensor velocity matrices
	vel_matrix  gps_velocity;
	vel_matrix  imu_velocity;
	vel_matrix  odom_velocity;	
	
	
	//! previous time update in seconds
	unsigned long 	pt_sec;		
	//! previous time update in microseconds			
	unsigned long 	pt_usec;				
	//! last update in seconds
	unsigned long 	t_sec;	
	//! last update in microseconds					
	unsigned long 	t_usec;					
	//! time difference between updates
	double delta_time;	
	
} localize;


//!Functions 

//!Constructors - create data structs
localize * CreateLocalize( void );	//!creates and returns dynamic memory

//!Destructors

//!Init Fcns
int InitLocalize( localize *out ); //!inits the sub-members of the Localize data struct

//!Zero Fcns - zero the elements
int ZeroLocalize( localize *in ); 

//!Get/Set Functions - resets specific values into the data struct
int SetCurrentLocalize( state_vector in,  localize *out ); //!adjusts the state vector and updates the Jacobian Matrix
state_vector GetCurrentLocalize( localize *in );//!returns the current state vector


//!Update Fcns - updates the sensors with latest data and updates predictions
int UpdateLocalizeData( int sensor,  localize *in, size_t size_data, void *data  );//!updates the external sensors data
int UpdateLocalize( localize *in );//!updates the Localize 
int	UpdateLocalizeTime2(localize *out);//! use computer time to update the timestamp

//!Compute Fcns
//!compute the state vector of attached sensors
int ComputeSensorStateVector( int sensor,  localize *in); 
//!compute the vel matrix of attached sensors
int ComputeSensorVelMatrix( int sensor,  localize *in);

//!Fuse the state vectors of attached sensors
int FuseSensorStateVector(  localize *in); 
//!Fuse the vel matrices of attached sensors
int FuseSensorVelMatrix(  localize *in);

//! Output Fcn - output to external functions

//!outputs the state vector as it stands
state_vector *OutputLocalizeStateVector ( localize *in );	
//!Converts the  UTM E and UTM N state vector elements to Lat and Lon and returns elevation unchanged 
//!used to get at the current lat and lon without needing to convert the values external to this 
//!function.
int OutputLatLonElev( localize *in, double *lat, double *lon, double *elev); 

#endif  //!define LOCALIZE_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
