/* $Id: GPSIDLtest.h,v 1.4 2005/06/10 03:38:43 dave Exp $ */

GpsIDL testGPSIDL =
{
  //!UTC time (seconds since the epoch) 
  1.7e12,
  //Latitude and longitude (degrees).  Latitudes are positive for
  //North, negative for South.  Longitudes are positive for East, negative for West. 
	
	// testing with Medicine Hat Airport
  //!Latitude (degrees).  Latitudes are positive for north, negative for south
  50.0664,

  //!Longitude (degrees). Longitudes are positive for east, negative for west. 
  -110.7174,

  //!Altitude (meters).  Positive is above sea-level, negative is below.
  750.360000,

  //!UTM easting (meters). 
  10145477.99999,

  //!UTM northing (meters). 
  5068989.6666,

  //!Horizontal dilution of precision. 
  3.4,

  //!Horizontal error (meters). 
  1.2,

  //!Vertical error (meters). 
  1.6,

  //!Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix 
  1,

  //!Number of satellites in view. 
  28,

  //!Time since epoch seconds
  32000000,

  //!Time since epoch micro seconds
  3500,
                
  //!GPS Sokkia solution status
  850000,
                
  //!GPS Sokkia velocity and position type
  "pose type\n",
  "vel type\n",
                
  //!GPS Sokkia BESTPOS mode standard deviations
  0.00001,
  0.25252525252,
  0.33333333333,

  //GPS Sokkia BESTVEL mode velocities (m/s) (horizontal speed overground, 
  //actual direction of motion over ground and vertical speed) and latency 
  //in the velocity time tag (sec).
  1.5,
  100.8,
  50.0,
  0.0354
	
	
};

