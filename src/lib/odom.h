// scripted sensor type odom
/* $Id: odom.h,v 1.5 2005/06/10 15:10:59 dave Exp $ */
sensor odom = 
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
	/*
		transducer values
		
	0: leftDistance
	1: rightDistance
	2: leftSpeed
	3: rightSpeed
	*/
	4,
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
	// General String2
	"nil",	
	// MEMBER PTRS
	// generate state vector
	OdomGenerateStateVector,
	// generate velocitymatrix
	OdomGenerateVelMatrix,
	// update from data
	OdomUpdateSensor,

};
