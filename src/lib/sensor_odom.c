// sensor_odom.c
//
// DRE 2005/05/18
//
// sensor_odom Functions
/* $Id: sensor_odom.c,v 1.4 2005/06/10 03:38:44 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_odom.h"
/*
		transducer values
		
	0: leftDistance
	1: rightDistance
	2: leftSpeed
	3: rightSpeed
	

*/

//-------------------------------------------------------
// CONSTRUCTORS
//-------------------------------------------------------


//-------------------------------------------------------
// Zero Fcns
//-------------------------------------------------------


	
//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Update Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Transform Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Compute Fcns
//-------------------------------------------------------
// Odom statevector generation function
int OdomGenerateStateVector( void *in, state_vector *out)
{
	// very simple implementation at first
	sensor *ptr;
	
	// type cast and assign pointer to input 
	ptr = (sensor *)in;
	
	// compute filter update
	ComputeKFilter( ptr->filter );
	
	// simply transfer the values for UTM and altitude over
	out->loc.x = 0.0; 	// utm easting
	out->loc.y = 0.0; // utm northing
	out->loc.z = 0.0; // altitude from MSL
	
	// future: compute quaternions from heading - now: leave zero
	out->orient.s = 0.0;
	out->orient.x = 0.0;
	out->orient.y = 0.0;
	out->orient.z = 0.0;

	return 0;
}
	
// Odom statevector generation function
int OdomGenerateVelMatrix( void *in, vel_matrix * out)
{

	sensor *ptr;
	
	// type cast and assign pointer to input
	ptr = (sensor *)in;
	
	// Ackermann drive so use one wheel velocity as the composite Vx
	//out->vel[0] = ptr->array[3]->value;// only returns right wheel 
	
	// current: use filtered velocity 
	out->vel[0] = ptr->filtered[3];// only returns right wheel
	
	// Future version should fuse right wheel velocity with 
	// delta position from left and right.
	// data should derive x component of velocity from wheel angle
	 
	out->vel[1] = 0.0;
	out->vel[2] = 0.0;	
	out->vel[3] = 0.0;	
	out->vel[4] = 0.0;	
	out->vel[5] = 0.0;

	return 0;
}
	
// Odom update fcn for data received					
int	OdomUpdateSensor( size_t  size_data, void *in, void *out)
{

	// this fcn extracts the data from the sent data packet 
	// for processing in other fcns
	WheelDataIDL *ptr;  //ptr to data
	
	sensor *ptr_output;
	int i;
	
	
	// confirm we have the right packet size
	if (sizeof( WheelDataIDL ) == size_data )
	{
		// aim specific pointer at the incoming data
		ptr = (WheelDataIDL *)in;
		
		ptr_output = (sensor *)out;
		// assign values to sensor transducers
		ptr_output->array[0]->value = ptr->leftDistance;
		ptr_output->array[1]->value = ptr->rightDistance;
		ptr_output->array[2]->value = ptr->leftSpeed * METRES_IN_KM / SECONDS_IN_HOUR ;// speed converted to metres/second
		ptr_output->array[3]->value = ptr->rightSpeed * METRES_IN_KM / SECONDS_IN_HOUR;// from km / hr

		//update time from computer for now
		//UpdateSensorTime (ptr_output, (unsigned long)(ptr->time_sec), (unsigned long)(ptr->time_usec) );  
		UpdateSensorTime2 ( ptr_output );		
		
		
		// increase population of data 
		for (i = 0; i < ptr_output->transducers; i++)  
		{
			// increase the populations
			ptr_output->array[i]->population++;
		}
		
	}
	else
	{
		// fail to update sensor data
		return -1;
	}
	return 0;
}


#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
