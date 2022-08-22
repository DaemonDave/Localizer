// scripted sensor type gps
/* $Id: gps.h,v 1.4 2005/06/10 15:10:59 dave Exp $ */
sensor gps = 
{
	// pt_sec
	0,
	// pt_usec
	0,
	// t_sec
	0,
	//t_usec
	0,
	// deltatime
	0.0,
	// number of transducers
	7,
	/*
	
	0: latitude
	1: longitude
	2: altitude
	3: utm_e
	4: utm_n
	5: hdop
	6: utm_zone converted using LatLonConversion.c
	
	*/
	// updated
	0,	
	// array of the sensors
	NULL,
	//! measurements of the transducers
	NULL,
	//! filtered values post filtering
	NULL,
	//! Kalman filter for tranducer array
	NULL,	
	// state_vector
	{ 0.0,0.0,0.0,1.0,0.0,0.0,0.0},
	// vel_matrix
	{ 0.0,0.0,0.0,0.0,0.0,0.0},	
	// General String
	"nil",
	// General String 2
	"nil",	
	// MEMBER PTRS
	// generate state vector
	GPSGenerateStateVector,
	// generate velocitymatrix
	GPSGenerateVelMatrix,
	// update from data
	GPSUpdateSensor,

};
