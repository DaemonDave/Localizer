//! sensor.h
//! SENSOR Header File
//! structs and fcns to create and use sensors
/* $Id: sensor.h,v 1.5 2005/06/10 15:10:59 dave Exp $ */

//! Includes
#include <sys/time.h>			//! for gettimeofday fcn


#ifndef TRANSDUCER_H
#include "transducer.h"
#endif

#ifndef KIN_MODEL_H				//! For Jacobian and Matrix Manipulation code
#include "kin_model.h"
#endif

#ifndef STATE_VECTOR_H
#include "state_vector.h"
#endif


#ifndef POSEMATH_H				//! For quaternion conversion fcns for various sensors.
#include "posemath.h"			//! GPS and IMU require pmRpyQuatConvert 
#endif //localizeMATH_H

#include "filter.h"



#ifdef __cplusplus
extern "C" {
#endif



#ifndef SENSOR_H
#define SENSOR_H



// Defines

#define DATA_TYPE_LENGTH				36
#define MICROSECOND_CONVERSION  1.0/1000000.0   //! used for converting time to floating point value
#define STRING_SIZE							60							//! size of generic strings within sensor struct



/*! 
	sensor_types - Types of sensor that may be the 
 	device under inspection as a container of transducers.
 	This is a pseudo construct, since multiple readings
 	may be derived from the same sensor representing
 	multiple devices.
*/
//! sensor type data struct
typedef struct 
{
	//! array of names of sensor types
	char sensor_type[DATA_TYPE_LENGTH];
	
}sensor_types;

//! a data structure to cover the types of data and the 
//! resultant variation in the handling of updates and computation
static sensor_types  sensor_list[50] = 
{

	{"gps"},							//! absolute 
	{"odometer"},					//! relative
	{"imu"}								//! relative
	
	 
};

/*! 
	Sensor data struct - the basis of the localizer data device:
	
	It contains a number of transducers as an array allowing operations 
	on any amount of the data received from a device for use in formulating
	a solution.  As a container, it controls the transducer objects.  As an
	object, it also stores/computes collective data types
*/
typedef struct
{

	//! MEMBERS
	
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
	//! number of transducers
	int		transducers;			
	//! positive indicates that the sensor received new data				
	int		updated;			
	//! array of the sensors				
	transducer ** array;			
	//! measurements of the transducers			
	double		 ** measurement;
	//! filtered values post filtering
	double     * filtered;
	//! Kalman filter for tranducer array
	k_filter *	filter;
	
	//! each sensor may have some contribution to the state vector and
	//! the velocity matrix so each will posess one to contribute.
	//! absolute contribution of the sensor
	state_vector  		sv;
	//! relative contribution of the sensor
	vel_matrix				vm; 
	//! generic string for use by the sensor
	char gen_string[STRING_SIZE];		
	//! generic string2 for use by the sensor		
	char gen_string2[STRING_SIZE];		
	
	//! METHODS

	//! unique fcns to generate the individual absolute and relative 
	//!
	//! Ptr to individual statevector generation function
	int (*GenerateStateVector)( void *in, state_vector *out);
	//! *in points to a sensor 
	//!   
	//! Ptr to individual statevector generation function
	int (*GenerateVelMatrix)( void *in, vel_matrix * out);  
	//! *in points to a sensor 
	//! 	
	//! Ptr to individual update fcn for data received	
	//! UpdateSensor is only concerned with placing the correct data 
	//! within the prescribed transducers for specificity				
	int	(*UpdateSensor)( size_t  size_data, void *in, void *out );		
	//! *in points to a data packet of unknown type 
	//! *out points to	the sensor matching data packet																													
	//!
} sensor;


//! Functions 

//! Constructors - create data structs
sensor * CreateSensor( void );	//! creates and returns dynamic memory

//! Init Fcns
int InitSensor(int num_transducers, sensor *out);//! inits a sensor with the defined number of sensors

//! Destructors

//! Zero Fcns - zero the elements
int ZeroSensor ( sensor *out );				//! zero all elements
int ZeroMeasurement ( sensor *out );	//! zeros values in transducers

//! Get/Set Functions - resets specific values into the data struct
int SetSensorTransducer( double svalue, double spredicted, double svariance, int snumber,  sensor * out);

//! Update Fcns - updates matrix/array with current values
int UpdateSensorTransducerValue ( double svalue, int snumber, sensor * out);
int UpdateSensorData ( sensor * in, size_t size_data, void *data ); //! updates the entire transducer
int UpdateSensorPredicted ( sensor * out);  //! iterates above fcn to handle entire transducer array

int UpdateSensorTime (sensor *out, unsigned long time_sec, unsigned long time_usec);//! update time elements from data
int	UpdateSensorTime2(sensor *out);//! use computer time to update the timestamp
//! Compute Fcns



#endif  //! define SENSOR_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
