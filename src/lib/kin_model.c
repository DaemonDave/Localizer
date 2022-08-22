// kin_model.c
//
// DRE 2005/05/18
//
// Kinematic Model Functions
/* $Id: kin_model.c,v 1.3 2005/06/06 18:53:07 dave Exp $ */

#ifndef KIN_MODEL_H
#include "kin_model.h"
#endif


//-------------------------------------------------------
// CONSTRUCTORS
//-------------------------------------------------------


// Constructors
skew_matrix * km_CreateSkewMatrix ( void )
{
	
	// assign dynamic memory
	return( (skew_matrix *)malloc(sizeof(skew_matrix)));

}// end km_CreateSkewMatrix

rot_matrix *	km_CreateRotMatrix ( void )
{
	
	// assign dynamic memory
	return(  (rot_matrix *) malloc(sizeof(rot_matrix)));

} // end km_CreateRotMatrix

U_matrix *	km_CreateUMatrix ( void )
{

	// assign dynamic memory
	return( (U_matrix *) malloc(sizeof(U_matrix)));
	

}// end km_CreateUMatrix



Jacobian * km_CreateJacobian ( void )
{
	
	// assign dynamic memory
	return((Jacobian *)malloc(sizeof(Jacobian)));
	

}// end km_CreateJacobian

// 
//-------------------------------------------------------
// Zero Fcns
//-------------------------------------------------------


int km_ZeroSkewMatrix ( skew_matrix *out )
{
	int i,j;// iterators
	
	// iterate and clear
	for (i = 0; i < 3; i ++)
	{
	
		for (j = 0; j < 3; j ++)
		{
			out->entry[i][j] = 0.0;
		}
	}

	return 0;
}// end km_ZeroSkewMatrix

int km_ZeroRotMatrix ( rot_matrix *out )
{

	int i,j;// iterators
	
	// iterate and clear
	for (i = 0; i < 3; i ++)
	{
	
		for (j = 0; j < 3; j ++)
		{
			out->entry[i][j] = 0.0;
		}
	}
	
	return 0;
}// end km_ZeroRotMatrix

int km_ZeroUMatrix ( U_matrix *out )
{
	int i,j;// iterators
	
	// iterate and clear
	for (i = 0; i < 3; i ++)
	{
	
		for (j = 0; j < 4; j ++)
		{
			out->entry[i][j] = 0.0;
		}
	}

	return 0;
}// end km_ZeroUMatrix

int km_ZeroJacobian ( Jacobian *out )
{

	// clear Rotation Matrix
	km_ZeroRotMatrix ( &(out->R) );
	
	// clear U Matrix
	km_ZeroUMatrix ( &(out->U) );
	
	
	return 0;
	
	
}// end km_ZeroJacobian

//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------


//-------------------------------------------------------
// Update Fcns
//-------------------------------------------------------

int  km_UpdateSkewMatrix ( PmQuaternion q, skew_matrix *out )
{
	//fcn places quaternion entries in matrix 
	// main diagonal zeros
	out->entry[0][0] = 0.0;
	out->entry[1][1] = 0.0;
	out->entry[2][2] = 0.0;
	
	// Row 1
	out->entry[0][1] = -1*q.z;
	out->entry[0][2] = q.y;	
	
	// Row 2
	out->entry[1][0] = q.z;	
	out->entry[1][2] = -1*q.x;		
	
	// Row 3
	out->entry[2][0] = -1*q.y;	
	out->entry[2][1] = q.x;	

	return 0;
}// end km_MakeSkewMatrix

int	km_UpdateRotMatrix ( PmQuaternion q, rot_matrix *out )
{
	// creates rotation matrix based on quaternions
	// Row 1
	out->entry[0][0] = ( (q.s)*(q.s) + (q.x)*(q.x) - (q.y)*(q.y) - (q.z)*(q.z)  );
	out->entry[0][1] = 2*( (q.x*q.y) - (q.s*q.z));
	out->entry[0][2] = 2*( (q.x*q.z) + (q.s*q.y));
	// Row 1
	out->entry[1][0] = 2*( (q.y*q.x) + (q.s*q.z));	
	out->entry[1][1] = ( (q.s)*(q.s) - (q.x)*(q.x) + (q.y)*(q.y) - (q.z)*(q.z)  );
	out->entry[1][2] = 2*( (q.y*q.z) - (q.s*q.x));	
	
	// Row 3	
	out->entry[2][0] = 2*( (q.z*q.x) - (q.s*q.y));
	out->entry[2][1] = 2*( (q.z*q.y) + (q.s*q.x));		
	out->entry[2][2] = ( (q.s)*(q.s) - (q.x)*(q.x) - (q.y)*(q.y) + (q.z)*(q.z)  );		
	
	
	return 0;
}// end km_MakeRotMatrix

int	km_UpdateUMatrix ( PmQuaternion q, U_matrix *out )
{
	// creates U matrix based on quaternions
	// Row 1
	out->entry[0][0] = -1 * q.x;	
	out->entry[0][1] = -1 * q.y;	
	out->entry[0][2] = -1 * q.z;	

	// Row 2
	out->entry[1][0] = q.s;	
	out->entry[1][1] = -1 * q.z;		
	out->entry[1][2] = q.y;		
	
	// Row 3
	out->entry[2][0] = q.z;		
	out->entry[2][1] = q.s;
	out->entry[2][2] = -1 * q.x;
	
	// Row 4
	out->entry[3][0] = -1 * q.y;			
	out->entry[3][1] = q.x;	
	out->entry[3][2] = q.s;	
	
	return 0;
		
}// end km_MakeUMatrix

int km_UpdateJacobian ( PmQuaternion q, Jacobian *out  )
{
	// constructs Jacobian from primitives
	// create rotation matrix portion
	km_UpdateRotMatrix (  q, &(out->R) );
	// create U matrix portion
	km_UpdateUMatrix (  q, &(out->U) );
	
}// end km_UpdateJacobian



//-------------------------------------------------------
// Transform Fcns
//-------------------------------------------------------

int km_MultRotMat_StateVec( rot_matrix R, vel_matrix v, state_vector *out )
{
	// compute elements depending on linear velocity
	// multiply elements and store inside state vector
	int i,j;
	double temp = 0.0;
	i = 0;
	
	// compute dx/dt
	
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += R.entry[i][j] * v.vel[j];
	}
	// store temp as dx/dt 
	out->loc.x = temp;
	temp = 0.0;
	// compute dy/dt
	i ++;
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += R.entry[i][j] * v.vel[j];
	}
	// store temp as dy/dt 
	out->loc.y = temp;
	temp = 0.0;
	// compute dz/dt
	i ++;
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += R.entry[i][j] * v.vel[j];
	}
	// store temp as dz/dt 
	out->loc.z = temp;	
	temp = 0.0;

	return 0;
}// end km_MultRotMat_StateVec

int km_MultUMat_StateVec( U_matrix U, vel_matrix v, state_vector *out)
{
	// compute elements depending on angular velocity
	// multiply elements and store inside state vector
	int i,j;
	double temp = 0.0;
	i = 0;
	
	// compute dq.s/dt
	
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += 0.5*( U.entry[i][j] * v.vel[j+3]);
	}
	// store temp as dq.s/dt 
	out->orient.s = temp;
	
	temp = 0.0;
	// compute dq.x/dt
	i ++;
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += 0.5*(U.entry[i][j] * v.vel[j+3]);
	}
	// store temp as dq.x/dt 
	out->orient.x = temp;
	temp = 0.0;
	// compute dq.y/dt
	i ++;
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += 0.5*( U.entry[i][j] * v.vel[j+3]);
	}
	// store temp as dq.y/dt 
	out->orient.y = temp;	
	temp = 0.0;
	// compute dq.z/dt
	i ++;
	// iterate through multipled elements
	for (j = 0; j < COL_SIZE; j ++)
	{
		temp += 0.5*( U.entry[i][j] * v.vel[j+3]);
	}
	// store temp as dz/dt 
	out->orient.z = temp;	
	temp = 0.0;
		
	return 0;
}// end km_MultUMat_StateVec

int km_ComputeStateVector( Jacobian J, vel_matrix v, state_vector *out)
{
	// computer position delta  
	km_MultRotMat_StateVec( J.R, v, out );
	
	// compute orientation delta
	km_MultUMat_StateVec( J.U, v, out);
	
	return 0;
}// end km_ComputeStateVector

//-------------------------------------------------------
// Kinematic Model Fcns
//-------------------------------------------------------
int km_ComputeKinematicModel ( Jacobian *J, vel_matrix v, double delta_t, state_vector *delta, state_vector *out  )
{
	
	// this fcn takes the input from the Jacobian and the velocity matrix
	// and computes the delta state change.  It then updates the pose state
	// vector with the changes and then uses the updates quaternion of the state vector
	// to update the Jacobian matrix.
	
	// computing state vector and place in delta
	km_ComputeStateVector( *(J), v, delta);

	// update the current state vector
	km_UpdateStateVector ( *(delta), delta_t, out );

	// update R Matrix
	km_UpdateRotMatrix ( (out->orient), &(J->R));

	// update U Matrix
	km_UpdateUMatrix ( (out->orient), &(J->U) );

	return 0;
}// end km_ApplyKinematicModel


