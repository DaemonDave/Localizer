// sensor_gps.c
//
// DRE 2005/05/18
//
// sensor_gps Functions
/* $Id: sensor_gps.c,v 1.4 2005/06/10 03:38:44 dave Exp $ */

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_gps.h"


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
/*
	transducer variables for GPS
	0: latitude
	1: longitude
	2: altitude
	3: utm_e
	4: utm_n
	5: hdop
	6: utm_zone converted using LatLonConversion.c
	
*/
// GPS statevector generation function
int GPSGenerateStateVector( void *in, state_vector *out)
{
	// very simple implementation at first
	sensor *ptr;
	
	// type cast and assign pointer to input 
	ptr = (sensor *)in;
	
	// compute filter update
	ComputeKFilter( ptr->filter );
	
	// simply transfer the values for UTM and altitude over
	out->loc.x = ptr->array[3]->value; // utm easting
	out->loc.y = ptr->array[4]->value; // utm northing
	out->loc.z = ptr->array[2]->value; // altitude from MSL
	
	// updated for filtering, use the filtered values
	// as the output to state vector and velocity matrix
	//out->loc.x = ptr->filtered[3]; // utm easting
	//out->loc.y = ptr->filtered[4]; // utm northing
	//out->loc.z = ptr->filtered[2]; // altitude from MSL	
	
//	printf("%e %e %e\n", ptr->filtered[3], ptr->filtered[4], ptr->filtered[2]  );
	
	// future: compute quaternions from heading - now: leave zero
	out->orient.s = 0.0;
	out->orient.x = 0.0;
	out->orient.y = 0.0;
	out->orient.z = 0.0;

	// pmRpyQuatConvert(PmRpy rpy, PmQuaternion * q)

	return 0;
}
	
// GPS statevector generation function
int GPSGenerateVelMatrix( void *in, vel_matrix * out)
{
	sensor *ptr;
	
	// type cast and assign pointer to input
	ptr = (sensor *)in;
	
	// for now implement as zero
	out->vel[0] = 0.0;
	out->vel[1] = 0.0;
	out->vel[2] = 0.0;	
	out->vel[3] = 0.0;	
	out->vel[4] = 0.0;	
	out->vel[5] = 0.0;	
	
	return 0;
}
	
// GPS update fcn for data received		
// this fcn is only concerned with placing the correct data 
// within the prescribed transducers for specificity			
int	GPSUpdateSensor( size_t  size_data, void *in, void *out)
{
	// this fcn extracts the data from the sent data packet 
	// for processing in other fcns
	GpsIDL 	*ptr;  //ptr to data
	sensor 	*ptr_output;// pointer to output data 
	
	// for conversion of UTM
	double 	convert_UTM_E;			// local converted UTM E
	double 	convert_UTM_N;			// local converted UTM N
															// use gen_string to copy the UTM zone into
	int i;
	

	// confirm we have the right packet size
	if (sizeof( GpsIDL ) == size_data )
	{
		// aim specific pointer at the incoming data
		ptr = (GpsIDL *)in;
		
		ptr_output = (sensor *)out;
		
		//update time from computer for now
		//UpdateSensorTime (ptr_output, (unsigned long)(ptr->time_sec), (unsigned long)(ptr->time_usec) );  
		UpdateSensorTime2 ( ptr_output );
		
		// assign values to sensor transducers
		ptr_output->array[0]->value = ptr->latitude;
		ptr_output->array[1]->value = ptr->longitude;
		ptr_output->array[2]->value = ptr->altitude;
		//printf("%e %e %e\n", ptr->latitude, ptr->longitude, ptr->altitude  );	
			
		// using LatLong-UTMconversion.h	Conversion routines to compute UTM Eastings and Northings
		// convert longitude into UTM Easting
		// void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, double *UTMNorthing, double *UTMEasting, char* UTMZone);
		LLtoUTM( 23, (const double) (ptr->latitude), (const double)(ptr->longitude), &(convert_UTM_N), &(convert_UTM_E), (ptr_output->gen_string));
		ptr_output->array[3]->value = convert_UTM_E;
		ptr_output->array[4]->value = convert_UTM_N;
		ptr_output->array[5]->value = ptr->hdop;
		
		//printf("%e %e %e\n", ptr_output->array[3]->value, ptr_output->array[4]->value,ptr_output->array[5]->value );	
	
		// debugging conversion
		printf("gps utm_e:%f gps utm_n:%f | converted utm_e:%f converted utm_n:%f\n", ptr->utm_e, ptr->utm_n, convert_UTM_E, convert_UTM_N  );
		
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
