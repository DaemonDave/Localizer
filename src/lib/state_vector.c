// state_vector.c
//
// DRE 2005/06/01
//
// state_vector Functions
/* $Id: state_vector.c,v 1.4 2005/06/10 03:38:44 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

#include "state_vector.h"


//-------------------------------------------------------
// CONSTRUCTORS
//-------------------------------------------------------
state_vector * km_CreateStateVector ( void )
{
	
	// assign dynamic memory
	return( (state_vector *) malloc(sizeof(state_vector)));

}// end km_CreateSkewMatrix

vel_matrix *	km_CreateVelMatrix ( void )
{
	
	// assign dynamic memory
	return (vel_matrix *) malloc(  sizeof(vel_matrix));


}// end km_CreateVelMatrix


//-------------------------------------------------------
// Zero Fcns
//-------------------------------------------------------
int km_ZeroStateVector ( state_vector *out )
{
	// zero pmCartesian
	out->loc.x = 0.0;
	out->loc.y = 0.0;
	out->loc.z = 0.0;
	
	
	// zero pmQuaternion to pointing north
	// this is a valid unit quaternion 
	// which is the equivalent "zero" for this data type
	out->orient.s = 1.0;
	out->orient.x = 0.0;
	out->orient.y = 0.0;
	out->orient.z = 0.0;
	
	return 0;
	
}

int km_ZeroVelocityMatrix ( vel_matrix *out )
{
	int i;
	
	for (i = 0; i < VEL_ARRAY; i++ )	
	{
		out->vel[i] = 0.0;
	}
	return 0;
	
}
	
//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------
int km_SetStateVector( double x, double y, double z, double qs, double qx, double qy, double qz, state_vector *out )
{
	// sets the individual values of the data struct
	// set pmCartesian
	out->loc.x = x;
	out->loc.y = y;
	out->loc.z = z;
	
	// set pmQuaternion
	out->orient.s = qs;
	out->orient.x = qx;
	out->orient.y = qy;
	out->orient.z = qz;

	return 0;
}
int km_SetStateVector2( state_vector in, state_vector *out )
{
	// state vector version
	// sets the individual values of the data struct
	// set pmCartesian
	out->loc.x = in.loc.x;
	out->loc.y = in.loc.y;
	out->loc.z = in.loc.z;
	
	
	// set pmQuaternion
	out->orient.s = in.orient.s;
	out->orient.x = in.orient.x;
	out->orient.y = in.orient.y;
	out->orient.z = in.orient.z;	
	
	return 0;
		
}

int km_SetVelMatrix( double vx, double vy, double vz, double omega_x, double omega_y, double omega_z, vel_matrix *out)
{
	// sets the individual values of the data struct
	out->vel[0] = vx;
	out->vel[1] = vy;	
	out->vel[2] = vz;	
	out->vel[3] = omega_x;	
	out->vel[4] = omega_y;	
	out->vel[5] = omega_z;	
	
	return 0;

}// end km_SetVelMatrix
int km_SetVelMatrix2( vel_matrix in, vel_matrix *out)
{
	// sets the individual values of the data struct
	out->vel[0] = in.vel[0];
	out->vel[1] = in.vel[1];	
	out->vel[2] = in.vel[2];	
	out->vel[3] = in.vel[3];	
	out->vel[4] = in.vel[4];	
	out->vel[5] = in.vel[5];	
	
	return 0;

}// end km_SetVelMatrix

//-------------------------------------------------------
// Copy Fcns
//-------------------------------------------------------
int CopyStateVector ( state_vector *in, state_vector *out)
{
	// copy contents from in into out
	// state vector version
	// sets the individual values of the data struct
	// set pmCartesian
	out->loc.x = in->loc.x;
	out->loc.y = in->loc.y;
	out->loc.z = in->loc.z;
	
	
	// set pmQuaternion
	out->orient.s = in->orient.s;
	out->orient.x = in->orient.x;
	out->orient.y = in->orient.y;
	out->orient.z = in->orient.z;

	return 0;
}



//-------------------------------------------------------
// Update Fcns
//-------------------------------------------------------
int	km_UpdateStateVector ( state_vector delta, double delta_t, state_vector *out )
{
	// adds delta to pose  X = X + Delta_X*Delta_t
	// improved with a time delta which is forced to be positive
	// sets the individual values of the data struct
	
	// make sure delta_t is positive
	if ( delta_t  < 0.0) 
	{
		// make positive
		delta_t = abs(delta_t);
	}
	
	// update pmCartesian
	out->loc.x += delta.loc.x*delta_t;
	out->loc.y += delta.loc.y*delta_t;
	out->loc.z += delta.loc.z*delta_t;
	
	
	// update pmQuaternion
	out->orient.s += delta.orient.s*delta_t;
	
	out->orient.x += delta.orient.x*delta_t;
	
	// test for quaternion rollover conditions
	if  (out->orient.x >= 1.00000)
	{
		// rollover to negative value
		out->orient.x = -APPROACHING_ONE;	
	}
	if ( out->orient.x <= -1.00000)
	{
		// rollover to positive value
		out->orient.x = APPROACHING_ONE;	
	}
	
	out->orient.y += delta.orient.y*delta_t;
	if  (out->orient.y >= 1.00000)
	{
		// rollover to negative value
		out->orient.y = -APPROACHING_ONE;	
	}
	if ( out->orient.y <= -1.00000)
	{
		// rollover to positive value
		out->orient.y = APPROACHING_ONE;	
	}
		
	out->orient.z += delta.orient.z*delta_t;	
	
	if  (out->orient.z >= 1.00000)
	{
		// rollover to negative value
		out->orient.z = -APPROACHING_ONE;	
	}
	if ( out->orient.z <= -1.00000)
	{
		// rollover to positive value
		out->orient.z = APPROACHING_ONE;	
	}
	
	// renormalize the quaternions to remain unitary
	pmQuatNorm( (out->orient), &(out->orient));
	
	
	return 0;	
}


//-------------------------------------------------------
// Transform Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Compute Fcns
//-------------------------------------------------------
int	WeighStateVectors ( state_vector in, double W, state_vector *out )
{
	// computes blending of two state vectors
	// multiply in by W factor and add to (1-W)out
	
	
	// weigh location component
	out->loc.x = (W)*(in.loc.x) + (1-W)*(out->loc.x);
	out->loc.y = (W)*(in.loc.y) + (1-W)*(out->loc.y);
	out->loc.z = (W)*(in.loc.z) + (1-W)*(out->loc.z);

	// weigh orientation component
	out->orient.s = (W)*(in.orient.s) + (1-W)*(out->orient.s);
	out->orient.x = (W)*(in.orient.x) + (1-W)*(out->orient.x);
	out->orient.y = (W)*(in.orient.y) + (1-W)*(out->orient.y);
	out->orient.z = (W)*(in.orient.z) + (1-W)*(out->orient.z);			

		
	return 0;
}

int	WeighStateVectors2 ( state_vector in, state_vector in2,  double W, state_vector *out )
{
	// computes blending of two state vectors
	// out = W*in + (1-W)in2	
	
	// weigh location component
	out->loc.x = (W)*(in.loc.x) + (1-W)*(in2.loc.x);
	out->loc.y = (W)*(in.loc.y) + (1-W)*(in2.loc.y);
	out->loc.z = (W)*(in.loc.z) + (1-W)*(in2.loc.z);

	// weigh orientation component
	out->orient.s = (W)*(in.orient.s) + (1-W)*(in2.orient.s);
	out->orient.x = (W)*(in.orient.x) + (1-W)*(in2.orient.x);
	out->orient.y = (W)*(in.orient.y) + (1-W)*(in2.orient.y);
	out->orient.z = (W)*(in.orient.z) + (1-W)*(in2.orient.z);			

		
	return 0;
}

int	WeighVelocityMatrices ( vel_matrix in, vel_matrix in2,  double W, vel_matrix *out )
{
	// computes blending of two state vectors
	// out = W*in + (1-W)in2	
	int i;
	
	for ( i = 0;i < 6; i++)
	{
	// weigh velocity component
		out->vel[i] = (W)*(in.vel[i]) + (1-W)*(in2.vel[i]);
	}
	

		
	return 0;
}


int IntegrateAcceleration (double acc, double delta_time,  double * out)
{
	// compute velocity from accelerations
	
	// need to incorporate the particle acceleration approximation for the 
	// equation of velocity integration.  Taken from Klamkin 95/Weissstein
	
	// Using uniform acceleration equation here for now.

	// Vf = out  
	// Vf = Vi +a*t = (out(t-1) + a*t )
	
	// compute and place in output velocity
	*out = *out + (acc)*delta_time;
	
	return 0;	
}

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
