// scripted sensor type imu
/* $Id: imu.h,v 1.5 2005/06/10 15:10:59 dave Exp $ */
sensor imu = 
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
		transducer values 16
		
	0: quaternion[0]
	1: quaternion[1]
	2: quaternion[2]
	3: quaternion[3]
	4: magfield[0]
	5: magfield[1]
	6: magfield[2]
	7: accel[0]
	8: accel[1]
	9: accel[2]
	10:angrate[0]
	11:angrate[1]
	12:angrate[2]
	13:angle[0]
	14:angle[1]
	15:angle[2]

	*/	
	16,
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
	ImuGenerateStateVector,
	// generate velocitymatrix
	ImuGenerateVelMatrix,
	// update from data
	ImuUpdateSensor,

};
