// transducer.c
// transducer Function File
// structs and fcns to create and use transducer abstracts
/* $Id: transducer.c,v 1.2 2005/06/06 18:53:08 dave Exp $ */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRANSDUCER_H
#include "transducer.h"
#endif


//-------------------------------------------------------
// CONSTRUCTORS
//-------------------------------------------------------
transducer * CreateTransducer( void )
{
	// assigns and returns dynamic memory

	// assign dynamic memory
	return( (transducer *)malloc(sizeof(transducer)));

}// end km_CreateSkewMatrix


//-------------------------------------------------------
// Zero Fcns
//-------------------------------------------------------
int ZeroTransducer ( transducer *out )
{
	// clear the char string
	snprintf (out->type, (size_t)DATA_TYPE_LENGTH, " ");  	
	// reset the ID number
	out->ID = 0;				 								
	
	out->value			= 0.0;								// current value as sensed
	out->previous		= 0.0;								// previous value
	out->variance		= 0.0;								// variance of the data 
	out->innovation = 0.0;								// noise component
	out->predicted	= 0.0;								// predicted value at previous time 
	out->population	= 0.0;								// current population of data points seen
	

	return 0;
}

int ZeroTransducerValues ( transducer *out )
{
  // clears only the numerical variables
	// leaves the type and ID intact

	out->value			= 0.0;								// current value as sensed
	out->previous		= 0.0;								// previous value
	out->variance		= 0.0;								// variance of the data 
	out->innovation = 0.0;								// noise component
	out->predicted	= 0.0;								// predicted value at previous time 
	out->population	= 0.0;								// current population of data points seen	
			

	// 
	return 0;	
}			
//-------------------------------------------------------
// Get/Set Fcns
//-------------------------------------------------------
int SetTransducer( char *newtype, int newID, double newvalue, double newprevious, double newinnovation, double newvariance, double newpredicted, double newcertainty, double newprobability, transducer *out )
{
	// set the values within the data struct
	
	// make a unique char string for this sensor
	snprintf(out->type, (size_t)DATA_TYPE_LENGTH , "%c-%s", newID, newtype);
		
	// set numerical values
	out->ID 				= newID;
	out->value 			= newvalue;
	out->previous 	= newprevious;
	out->innovation	= newinnovation;
	out->variance 	= newvariance;
	out->predicted 	= newpredicted;
	out->certainty  = newcertainty;
	out->probability= newprobability;
	

	return 0;
}


int SetTransducerType (char *newtype, int newID, transducer *out )
{
	// sets only the physical phenomenon
	
	// make a unique char string for this sensor
	snprintf(out->type, (size_t)DATA_TYPE_LENGTH , "%c-%s", newID, newtype);
	


	return 0;
}

double GetTransducerValue( transducer *out )
{
	// gets specific value

	return(out->value);
}

double GetTransducerPredicted( transducer *out )
{
	// gets specific value

	return(out->predicted);
}

//-------------------------------------------------------
// Update Fcns
//-------------------------------------------------------
int UpdateTransducerValue(double updatevalue, transducer *out )
{
	// updates the sensed value
	
	// store old value in previous
	out->previous = out->value;
	
	// enter new sensed value
	out->value = updatevalue;

	// increase population of values seen
	out->population++;

	return 0;
}

int UpdateTransducerValue2( transducer *out )
{
	// updates the sensed value with predicted without data
	
	// enter new sensed value
	out->value = out->predicted;

	// increase population of values seen
	//out->population++;

	return 0;
}// end UpdateTransducerValue2

int UpdateTransducerPrevious( transducer *out )
{
	// updates the sensed value with predicted without data
	
	// store old value in previous
	out->previous = out->value;

	// increase population of values seen
	//out->population++;

	return 0;
}// end UpdateTransducerValue2

int UpdateTransducerPredicted(double W1, double W2, transducer *out )
{
	// updates the predicted value
	// W1 is the weight of the previous value
	// W2 is the weight of the current value
	// store old value in previous
	if (out->population)// if there is more than zero data measurements
	{
		out->predicted = W1*(out->predicted) + W2*(out->value);
	}
	else// else first measurement then simply assign
	{
		out->predicted =(out->value);
	}

	return 0;
}

int UpdateTransducerValueWithPredicted(double W1, double W2, transducer *out )
{
	// inserts a weighted prediction within the value
	// W1 is the weight of the previous value
	// W2 is the weight of the current value
	// store old value in previous
	out->value = W1*(out->predicted) + W2*(out->value);
	

	return 0;
}



//-------------------------------------------------------
// Transform Fcns
//-------------------------------------------------------



//-------------------------------------------------------
// Compute Fcns
//-------------------------------------------------------
int ComputeTransducerInnovation( transducer *out  )
{
	// compute noise form difference between predicted and actual reading
	out->innovation = out->value - out->predicted;
	


	return 0;
}





#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif

