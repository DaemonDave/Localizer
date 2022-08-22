// localize.c
//
// DRE 2005/05/18
//
// localize Functions
/*
	This fcn is the container struct for the lower sensor, transducer, Jacobian, and 
	state vector structs.  It is the lowest functional element to be used in 
	the function of gaining a current state of the system.

*/
/* $Id: localize.c,v 1.2 2005/06/10 03:38:44 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

// fcn header
#include "localize.h"

// data headers for various sensor types
#include "gps.h"
#include "imu.h"
#include "odom.h"


//-------------------------------------------------------
// CONSTRUCTORS
//-------------------------------------------------------
localize * CreateLocalize( void )
{
	
	// assign dynamic memory
	return( (localize *) malloc(sizeof(localize)));

}// end CreateLocalize

//-------------------------------------------------------
// Destructors
//-------------------------------------------------------




//-------------------------------------------------------
// Init Fcns
//-------------------------------------------------------
int InitLocalize( localize *out )
{
	// inits the sub-members of the Localize data struct
	
	// aim pointers at the sub-elements
	out->ptr_jacob 								= &(out->jacob);// Jacobian Matrix
	out->ptr_fused_state					= &(out->fused_state);  			// the output state vector
	out->ptr_fused_delta_state		= &(out->fused_delta_state);	// delta state vector
	out->ptr_fused_vel_matrix 		= &(out->fused_vel_matrix);		// velocity matrix from relative sensors
	
	// init the state vectors
	km_ZeroStateVector ( &(out->gps_state));					
	km_ZeroStateVector ( &(out->imu_state));	
	km_ZeroStateVector ( &(out->odom_state));	
	
	// init the velocity matrices
  km_ZeroVelocityMatrix ( &(out->gps_velocity) );
  km_ZeroVelocityMatrix ( &(out->imu_velocity) );
  km_ZeroVelocityMatrix ( &(out->odom_velocity) );		
										
	// aim sensor pointer at external sensors
	out->ptr_gps   								= &(gps);	
	out->ptr_odom									= &(odom);
	out->ptr_imu									= &(imu);
	
	
	// initialize the internal data structs to zero state
	
	// zero the state vector
	km_ZeroStateVector ( out->ptr_fused_state );
	// zero the delta state vector
	km_ZeroStateVector ( out->ptr_fused_delta_state );
	// zero the previous state vector	
	km_ZeroStateVector ( &(out->previous_fused_state) );	
	// zero the velocity matrix
	km_ZeroVelocityMatrix (	out->ptr_fused_vel_matrix );
	
	// set the Jacobian to the current state vector pointing north
	km_UpdateJacobian ( (out->ptr_fused_state->orient), out->ptr_jacob  );
	
	// initialize the sensors
	
	// init the gps - assuming the struct was created and static elements are set
	// still need to allocate the dynamic memory for arrays
	InitSensor ( out->ptr_gps->transducers,  out->ptr_gps );
	
	// init the imu
	InitSensor ( out->ptr_imu->transducers,  out->ptr_imu );	

	// init the odometry
	InitSensor ( out->ptr_odom->transducers,  out->ptr_odom );			
	
	// init time stamps - two passes through update time
	UpdateLocalizeTime2 ( out );	
	UpdateLocalizeTime2 ( out );		
	
	
	return 0;
}

//-------------------------------------------------------
// Zero Fcns
//-------------------------------------------------------
int ZeroLocalize( localize *in)
{
	// zeros the elements within Localize
	// assumes it is initialized

	// zero the state vector
	km_ZeroStateVector ( in->ptr_fused_state );
	// zero the delta state vector
	km_ZeroStateVector ( in->ptr_fused_delta_state );	
	// zero the velocity matrix
	km_ZeroVelocityMatrix (	in->ptr_fused_vel_matrix );
	
	// set the Jacobian to the current state vector pointing north
	km_UpdateJacobian ( (in->ptr_fused_state->orient), in->ptr_jacob  );	
	
	
	return 0;
}// end ZeroLocalize

	
//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------
// This is used to change the state of the localization 
// model drastically like when the vehicle has a GPS
// update or the user overrides the current Localize
// UpdateLocalize should be used for dead reckoning
int SetCurrentLocalize( state_vector in,  localize *out )
{
	// adjusts the state vector and updates the Jacobian Matrix
	
	// set the state vector to the new value
	km_SetStateVector2( in, out->ptr_fused_state );
	
	// set the Jacobian to the current state vector 
	km_UpdateJacobian ( (out->ptr_fused_state->orient), out->ptr_jacob  );	

	return 0;
}

// GetCurrentLocalize can be used externally to 
// retrieve the current state vector of the Localize 
state_vector GetCurrentLocalize( localize *in )
{
	// returns the current state vector
	return *(in->ptr_fused_state );	
	
}// end GetCurrentLocalize

//-------------------------------------------------------
// Update Fcns
//-------------------------------------------------------
int UpdateLocalizeData( int sensor,  localize *in, size_t size_data, void *data  )
{
	// updates the external sensors data
	
	// this fcn will be a stop gap until the full fcn list is 
	// finally worked out
	switch (sensor)
	{
		case GPS_SENSOR:	
		{
			// send data to the gps
			UpdateSensorData( (in->ptr_gps), size_data, data );
			break;
		}
		case IMU_SENSOR:	
		{
			// send data to the imu
			UpdateSensorData( (in->ptr_imu), size_data, data );
			break;
		}	
		case ODOM_SENSOR:	
		{
			// send data to the odom
			UpdateSensorData( (in->ptr_odom), size_data, data );
			break;
		}	
	}
	
}
int UpdateLocalize( localize *in )
{
	// updates the Localize 
	// this fcn calls all sensors and updates their data
	
	return 0;
}

//! Bug fix- update time fcn to use computer time for everything
//! The IDLs do not all have timestamps, nor do some of the sensors
//! and so I will use the computer time for the moment to make realistic 
//! if not completely accurate computations.
int UpdateLocalizeTime2 (localize *out )
{
	//! get time from computer and substitute it into the current time stamp
	//! and then compute the latest delta time for computation.
	struct timeval   computer_time;
	
	gettimeofday( &computer_time, 0 );//! timezone not implemented under linux, needs to be zero
	
	//! update the time data from sequential measurements
	(out->pt_sec) 	= (out->t_sec);
	(out->pt_usec) 	= (out->t_usec);
	
	//! update new time from computer
	out->t_sec			= ( unsigned long )computer_time.tv_sec;
	out->t_usec 		= ( unsigned long )computer_time.tv_usec;
	
	//! compute delta time step adding microseconds to seconds
	out->delta_time  = ((double)out->t_sec - (double)out->pt_sec ) + (MICROSECOND_CONVERSION)*((double)out->t_usec - (double)out->pt_usec);
	//printf("%e \n", out->delta_time );

	return 0;
}//! end UpdateSensorTime2


//-------------------------------------------------------
// Transform Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Compute Fcns
//-------------------------------------------------------
// compute the state vector of attached sensors
int ComputeSensorStateVector( int sensor,  localize *in )
{
	// compute the state vector of attached sensors
	switch (sensor)
	{
		case GPS_SENSOR:	
		{
			// force GPS to compute internal state vector
			// 	int (*GenerateStateVector)( void *in, state_vector *out);
			in->ptr_gps->GenerateStateVector( (void *)in->ptr_gps, &(in->ptr_gps->sv) );
			// place the computed state vector within Localize
			CopyStateVector ( &(in->ptr_gps->sv), &(in->gps_state));
			break;
		}
		case IMU_SENSOR:	
		{
			// force GPS to compute internal state vector
			in->ptr_imu->GenerateStateVector( (void *)in->ptr_imu, &(in->ptr_imu->sv) );
			// place the computed state vector within Localize
			CopyStateVector ( &(in->ptr_imu->sv), &(in->imu_state));
			break;
		}	
		case ODOM_SENSOR:	
		{
			// force GPS to compute internal state vector
			in->ptr_odom->GenerateStateVector( (void *)in->ptr_odom, &(in->ptr_odom->sv) );
			// place the computed state vector within Localize
			CopyStateVector ( &(in->ptr_odom->sv), &(in->odom_state));			
			break;
		}	
		
	}
	
	return 0;
}
 
// compute the vel matrix of a single attached sensor
int ComputeSensorVelMatrix( int sensor,  localize *in)
{
	// compute the velocity matrix of attached sensors
	switch (sensor)
	{
		case GPS_SENSOR:	
		{
			// force GPS to compute internal state vector
			in->ptr_gps->GenerateVelMatrix( (void *)in->ptr_gps, &(in->ptr_gps->vm) );
			// place the computed state vector within Localize
			km_SetVelMatrix2( (in->ptr_gps->vm),&(in->gps_velocity));// vel matrix version
			break;
		}
		case IMU_SENSOR:	
		{
			// force GPS to compute internal state vector
			in->ptr_imu->GenerateVelMatrix( (void *)in->ptr_imu, &(in->ptr_imu->vm) );
			// place the computed state vector within Localize
			km_SetVelMatrix2( (in->ptr_imu->vm), &(in->imu_velocity));
			break;
		}	
		case ODOM_SENSOR:	
		{
			// force GPS to compute internal state vector
			in->ptr_odom->GenerateVelMatrix( (void *)in->ptr_odom, &(in->ptr_odom->vm) );
			// place the computed state vector within Localize
			km_SetVelMatrix2( (in->ptr_odom->vm), &(in->odom_velocity));			
			break;
		}	
	}

	return 0;
}

int ComputeUpdatedStateVector ( localize *in, double delta_t )
{
	// call low-level function to perform update

	// first version uses the update state vector
	km_UpdateStateVector (in->fused_delta_state, delta_t, in->ptr_fused_state );
	
	
	
	return 0;
}

// Fuse the state vectors of attached sensors
int FuseSensorStateVector( localize *in )
{
	// fuses the respective estimates from the various 
	// sensors into a fused output

	// first version just send back GPS version
	//*(in->ptr_fused_state) = (in->gps_state);
	
	// The second version attempts to reduce the
	// "snapback problem" that would occur from a bad
	// predicted value of an absolute sensor that 
	// wrongfully negates velocity and acceleration from the previous
	// timestep.
	
	// If first iteration, take the value of GPS reading as a start to
	// begin the computation.  This speeds up the convergence of a solution
	// copy to previous state
	CopyStateVector ( in->ptr_fused_state, &(in->previous_fused_state));// copy contents from in into out
			
	// first version just send back GPS state 
	CopyStateVector ( &(in->ptr_gps->sv), in->ptr_fused_state);// copy contents from in into out

	
	//printf("FuseStateVector:%e %e %e\n", in->ptr_gps->sv.loc.x, in->ptr_gps->sv.loc.y, in->ptr_gps->sv.loc.z );	
	
	// copy imu heading into fused state quaternions
	in->ptr_fused_state->orient.s = in->ptr_imu->sv.orient.s;
	in->ptr_fused_state->orient.x = in->ptr_imu->sv.orient.x;
	in->ptr_fused_state->orient.y = in->ptr_imu->sv.orient.y;
	in->ptr_fused_state->orient.z = in->ptr_imu->sv.orient.z;				
/*		// Weigh the old vector with the 
		// new estimate.
		
		// copy current to previous state
		CopyStateVector ( in->ptr_fused_state, &(in->previous_fused_state));// copy contents from in into out
		
		// weigh GPS vector with into fused state vector
		WeighStateVectors ( in->gps_state, 0.75, in->ptr_fused_state );	
		
		// This is where additional absolute measurements may be applied to the 
		// state of the system while fused together.  The next step will be to analyze the 
		// other sensors and find a way to weight the inputs.
		
	}
	else // not first data point and no new measurement
	{
*/

	return 0;
}
// Fuse the vel matrices of attached sensors
int FuseSensorVelMatrix( localize *in )
{
	// fuses the respective estimates from the various 
	// sensors into a fused output

	// first version just send back GPS version
	//*(in->ptr_fused_vel_matrix) = (in->imu_velocity);
	
	
	// second version
	// average Vx components
	in->ptr_fused_vel_matrix->vel[0] = in->imu_velocity.vel[0];//0.75*(in->imu_velocity.vel[0]) + 0.25*(in->odom_velocity.vel[0]);
	printf("fused velocity: %e   imu:%e  odom:%e \n", in->ptr_fused_vel_matrix->vel[0], (in->imu_velocity.vel[0]), (in->odom_velocity.vel[0])  );
	// Use IMU component for Vx
	in->ptr_fused_vel_matrix->vel[1] = 1.0*in->imu_velocity.vel[1]; 
	// Use IMU component for Vz
	in->ptr_fused_vel_matrix->vel[2] = 1.0*in->imu_velocity.vel[2]; 
	// Use IMU component for Omegax
	in->ptr_fused_vel_matrix->vel[3] = 1.0*in->imu_velocity.vel[3];	
	// Use IMU component for Omegay
	in->ptr_fused_vel_matrix->vel[4] = 1.0*in->imu_velocity.vel[4];
	// Use IMU component for Omegaz
	in->ptr_fused_vel_matrix->vel[5] = 1.0*in->imu_velocity.vel[5];	

	return 0;
}
int ComputeLocalize (localize *in )
{
	// macro fcn to compute the Localize
	
	// First Algorithm 
	
	// Compute Sensor Absolute State Vectors
	//printf("[%e %e %e ][ %f %f %f %f]\n", in->ptr_fused_state->loc.x, in->ptr_fused_state->loc.y, in->ptr_fused_state->loc.z, in->ptr_fused_state->orient.s, in->ptr_fused_state->orient.x, in->ptr_fused_state->orient.y, in->ptr_fused_state->orient.z );
	// compute gps
	ComputeSensorStateVector( GPS_SENSOR, in );
	
	// compute imu
	ComputeSensorStateVector( IMU_SENSOR, in );
	
	// compute odom
	ComputeSensorStateVector( ODOM_SENSOR, in );
		
	// Fuse the absolute state vector with current state vector
	// and alter the absolute Localize directly
	FuseSensorStateVector( in );

				
	// Compute the relative delta state vector
	
	// First compute velocity matrices
	// then transform using Jacobian into delta state vector
	
	// compute gps
	ComputeSensorVelMatrix( GPS_SENSOR, in );
	
	// compute imu
	ComputeSensorVelMatrix( IMU_SENSOR, in );
	
	// compute odom
	ComputeSensorVelMatrix( ODOM_SENSOR, in );
	
	// Fuse velocity matrices
	FuseSensorVelMatrix( in );
	
	// update the time since last computation 
	UpdateLocalizeTime2 ( in );	
	
	// Transform fused velocity matrix into delta state vector using kinematic model
	// and 
	// Fuse the absolute state vector with current state vector
	// km_ComputeKinematicModel also updates the *out state vector based on the model
	// in->delta_time
	km_ComputeKinematicModel ( in->ptr_jacob, in->fused_vel_matrix, in->delta_time, in->ptr_fused_delta_state, in->ptr_fused_state );
	
	
	

	return 0;
}


//-------------------------------------------------------
// Output Fcns
//-------------------------------------------------------

//  Output Fcn - output to external functions
state_vector *OutputLocalizeStateVector ( localize *in )
{
	
	;
	// simple output for now
	return (in->ptr_fused_state);
}


int OutputLatLonElev( localize *in, double *lat, double *lon, double *elev)
{
	// this fcn converts the state vector UTM_E and UTM_N and 
	
	// convert and place within lat and lon as decimal degrees
	// void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone, double* Lat,  double* Long );
	UTMtoLL(23, (const double)(in->ptr_fused_state->loc.y), (const double)(in->ptr_fused_state->loc.x), (const char*)(in->ptr_gps->gen_string), lat,  lon );
	// return current elevation from MSL as measured from GPS
	*elev = in->ptr_fused_state->loc.z;

	return 0;
}



#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
