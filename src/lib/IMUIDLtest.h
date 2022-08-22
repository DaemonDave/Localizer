/* $Id: IMUIDLtest.h,v 1.2 2005/06/10 18:32:55 dave Exp $ */

ImuIDL testIMUIDL =
{
 	//!Structure containing time the IMU was sampled
	{0, 500000},

	/*!	4 component quaternion which describes the orientation of the IMU with respect
  	 	to the fixed earth coordinate system. The earth fixed coordinate
			system has X pointing North, Y pointing East, and Z pointing down. */
	0.717,
	0.0,
	0.0,
	0.717,

  /*!	Vector (X, Y and Z components) quantifying the direction and
			magnitude of the instantaneously measured magnetic field that
			the IMU is exposed expressed in terms of the IMU's local coordinate system.*/

	-44.3, 	// x
	2.1,		// y
	3.2,		// z
	/*!	Vector (X, Y and Z components) quantifying the direction and
			magnitude of the instantaneously measured acceleration expressed in terms of the IMU
			local coordinate system.*/            
	20.1,
	0.00,
	0.00,

  /*!	Vector (X, Y and Z components) quantifying the rate of rotation of
			the IMU expressed in terms of the IMU local coordinate system.*/                
	0.0,  // x
	0.0,  // y
	0.5,  // z
  /*!	Vector (X, Y and Z components) quantifying the roll, pitch and yaw
			of the IMU expressed in terms of the IMU local coordinate system.*/
	60.0, // roll
	30.0, // pitch
	10.0, // yaw

  /*!	The 3x3 orientation matrix describes the orientation of the IMU
			expressed in terms of the IMU local coordinate system.*/
  1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0 
	
};


