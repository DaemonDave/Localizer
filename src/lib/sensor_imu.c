// sensor_imu.c
//
// DRE 2005/05/18
//
// sensor_imu Functions
/* $Id: sensor_imu.c,v 1.4 2005/06/10 03:38:44 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_imu.h"


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
// imu statevector generation function
int ImuGenerateStateVector( void *in, state_vector *out)
{
	// very simple implementation at first
	sensor *ptr;
	
	// type cast and assign pointer to input 
	ptr = (sensor *)in;
	
	
	// compute filter update
	ComputeKFilter( ptr->filter );
	
	// simply transfer the values for UTM and altitude over
	out->loc.x = 0.0; // utm easting
	out->loc.y = 0.0; // utm northing
	out->loc.z = 0.0; // altitude from MSL
	
	// original:  use given values
	out->orient.s = ptr->array[0]->value;// quaternion[0]
	out->orient.x = ptr->array[1]->value;// quaternion[1]
	out->orient.y = ptr->array[2]->value;// quaternion[2]
	out->orient.z = ptr->array[3]->value;// quaternion[3]
	
	// current: use filtered values

	//out->orient.s = ptr->filtered[0];// quaternion[0]
	//out->orient.x = ptr->filtered[1];// quaternion[1]
	//out->orient.y = ptr->filtered[2];// quaternion[2]
	//out->orient.z = ptr->filtered[3];// quaternion[3]	
	
	// future:  compute quaternions from heading	
	
	return 0;
}
	
// imu statevector generation function
int ImuGenerateVelMatrix( void *in, vel_matrix * out)
{
	sensor *ptr;// pointer to the sensors own struct
	
	double* ptr_dbl;
	// type cast and assign pointer to input
	ptr = (sensor *)in;
 
	// Compute Velocity Matrix components
	ptr_dbl = (double *)(out->vel); // point at out	
	
	// compute x velocity component from acceleration Ax
	//IntegrateAcceleration (ptr->array[7]->value, ptr->delta_time, (ptr_dbl) );
	//out->vel[0] = (out->vel[0]) + (ptr->filtered[7])*ptr->delta_time;
	out->vel[0] = out->vel[0] + (ptr->array[7]->value)*(ptr->delta_time);
	printf("delta_time:%e  \n", ptr->delta_time );
	//printf("Vx:%e  filtered:%e delta_time:%e  \n",out->vel[0], ptr->filtered[7], ptr->delta_time );
	//ptr_dbl++;
	// compute y velocity component from acceleration Ay
	//IntegrateAcceleration (ptr->array[8]->value, ptr->delta_time,  (ptr_dbl) );
	// fix IntegrateAcceleration
	out->vel[1] = (out->vel[1]) + (ptr->filtered[8])*(ptr->delta_time);
			
	// compute z velocity component from acceleration Az
	out->vel[2] = (out->vel[2]) + (ptr->filtered[9])*(ptr->delta_time);
		
	// transfer omega x velocity component from angular rate omega x
	out->vel[3] = (ptr->filtered[10]);
	
	// transfer omega y velocity component from angular rate omega y
	out->vel[4] = (ptr->filtered[11]);

	// transfer omega z velocity component from angular rate omega z
	out->vel[5] = (ptr->filtered[12]);		

	return 0;
}
	
// imu update fcn for data received					
int	ImuUpdateSensor( size_t  size_data, void *in, void *out)
{
	// this fcn extracts the data from the sent data packet 
	// for processing in other fcns
	ImuIDL *ptr;  //ptr to data
	sensor *ptr_output;
	int i;

	// confirm we have the right packet size
	if (sizeof( ImuIDL ) == size_data )
	{
		// aim specific pointer at the incoming data
		ptr = (ImuIDL *)in;
		ptr_output = (sensor *)out;
		
		//update time from computer for now
		//UpdateSensorTime (ptr_output, (unsigned long)(ptr->time_sec), (unsigned long)(ptr->time_usec) );  
		UpdateSensorTime2 ( ptr_output );
				
		// assign values to sensor transducers
		ptr_output->array[0]->value = ptr->quaternion[0];
		ptr_output->array[1]->value = ptr->quaternion[1];
		ptr_output->array[2]->value = ptr->quaternion[2];
		ptr_output->array[3]->value = ptr->quaternion[3];
		ptr_output->array[4]->value = ptr->magfield[0];
		ptr_output->array[5]->value = ptr->magfield[1];
		ptr_output->array[6]->value = ptr->magfield[2];		
		ptr_output->array[7]->value = ptr->accel[0];
		ptr_output->array[8]->value = ptr->accel[1];
		ptr_output->array[9]->value = ptr->accel[2];
		ptr_output->array[10]->value = ptr->angrate[0];
		ptr_output->array[11]->value = ptr->angrate[1];
		ptr_output->array[12]->value = ptr->angrate[2];										
		ptr_output->array[13]->value = ptr->angle[0];
		ptr_output->array[14]->value = ptr->angle[1];
		ptr_output->array[15]->value = ptr->angle[2];
						

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
