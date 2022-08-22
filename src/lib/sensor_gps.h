// sensor_gps.h
/*

	This header file outlines the specific implementations for the various sensor types 
	within the localization filter apparatus.  This is intended to encapsulate the GPS
	-specific settings and functions not unlike an inherited class in C++

*/
/* $Id: sensor_gps.h,v 1.3 2005/06/10 03:38:44 dave Exp $ */

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

#ifndef LATLONGCONV		// used for lat lon conversion functions
#include "LatLong-UTMconversion.h"
#endif 

#ifndef CONSTANTS_H
#include "constants.h"		// used for the defined ellipsoid for UTM conversion  Ellipsoid 23 = WG-84
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SENSOR_GPS_H
#define SENSOR_GPS_H




// GPS sensor device

/*
	gen_string is used to store UTM zone returned from Lat Lon conversion
	
*/

// GPS data IDL struct

typedef struct 
{
  //!UTC time (seconds since the epoch) 
  double utc_time;

  //Latitude and longitude (degrees).  Latitudes are positive for
  //North, negative for South.  Longitudes are positive for East, negative for West. 
  //!Latitude (degrees).  Latitudes are positive for north, negative for south
  double latitude;

  //!Longitude (degrees). Logitudes are positive for east, negative for west. 
  double longitude;

  //!Altitude (meters).  Positive is above sea-level, negative is below.
  double altitude;

  //!UTM easting (meters). 
  double utm_e;

  //!UTM northing (meters). 
  double utm_n;

  //!Horizontal dilution of precision. 
  double hdop;

  //!Horizontal error (meters). 
  double err_horz;

  //!Vertical error (meters). 
  double err_vert;

  //!Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix 
  long quality;

  //!Number of satellites in view. 
  long sat_count;

  //!Time since epoch seconds
  long time_sec;

  //!Time since epoch micro seconds
  long time_usec;
                
  //!GPS Sokkia solution status
  unsigned long sol_status;
                
  //!GPS Sokkia velocity and position type
  char pos_type[70];
  char vel_type[70];
                
  //!GPS Sokkia BESTPOS mode standard deviations
  double latitudestandarddeviation;
  double longitudestandarddeviation;
  double altitudestandarddeviation;

  //GPS Sokkia BESTVEL mode velocities (m/s) (horizontal speed overground, 
  //actual direction of motion over ground and vertical speed) and latency 
  //in the velocity time tag (sec).
  double hor_speed;
  double direction_motion;
  double vert_speed;
  float latency;
	
	
}GpsIDL;

// FCN Declarations 
// GPS statevector generation function
int GPSGenerateStateVector( void *in, state_vector *out);  
	
// GPS vel matrix generation function
int GPSGenerateVelMatrix( void *in, vel_matrix * out);  
	
// GPS update fcn for data received					
int	GPSUpdateSensor( size_t  size_data, void *in, void *out);





#endif  // define SENSOR_GPS_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif



