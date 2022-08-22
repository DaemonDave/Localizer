// transducer.h
// transducer Header File
// structs and fcns to create and use transducer abstracts
/* $Id: transducer.h,v 1.2 2005/06/06 18:53:08 dave Exp $ */


// Includes

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRANSDUCER_H
#define TRANSDUCER_H


// defines

#define DATA_TYPE_LENGTH	36




// data structs


// Types of physical phenomena that may be the 
// data under inspection by the transducer in question.
// This is a pseudo construct, since multiple readings
// may be derived from the same transducer representing
// multiple values.
typedef struct 
{
	char phenomenon[DATA_TYPE_LENGTH];
	
}transducer_types;

// a data structure to cover the types of data and the 
// resultant variation in the handling of updates and computation
static transducer_types  transducer_list[50] = 
{

	{"distance"},					// measure in metres
	{"velocity"},					// metres /second
	{"acceleration"},			// metres / second*second
	{"angle"},						// angle in radians
	{"temperature"},			// degrees in K/C
	{"current"},					// Amperes
	{"voltage"}						// Volts
 
};// end struct 

// this data struct transducer
// is a first implementation subject to 
// improvement upon re-use.  The main 
// objective is to capture the value of the 
// data along with the data statistics in a 
// single entity.  


typedef struct
{
	// Identification data

	char			type[DATA_TYPE_LENGTH];  	// describing the type of data ()
	int				ID;				 								// Identify the data type : acceleration etc.

	// transducer data
	
	// The following values will be single values at the beginning  
	// and then expanded to include arrays.  Stores all values as doubles
	// even though some may require  
	
	double		value;										// current value as sensed
	double		previous;									// previous value
	double		variance;									// variance of the data 
	double		innovation;								// estimated noise component [ measurement - predicted]
	double		predicted;								// predicted value at previous time 
	double		population;								// current population of data points seen
	double		certainty;								// current certainty
	double		probability;							// current probability
	
} transducer;


// functions 

// Constructors - create data structs
transducer * CreateTransducer( void );	// creates and returns dynamic memory

// Destructors


// Zero Fcns - zero the elements
int ZeroTransducer ( transducer *out );
int ZeroTransducerValues ( transducer *out );  // clears only the numerical variables


// Get/Set Functions - resets specific values into the data struct
int SetTransducer( char *newtype, int newID, double newvalue, double newprevious, double newinnovation, double newvariance, double newpredicted, double newcertainty, double newprobability, transducer *out );
int SetTransducerType (char *newtype, int newID, transducer *out );// sets only the physical phenomenon

// simple get functions
double GetTransducerValue( transducer *out );				// gets specific value
double GetTransducerPredicted( transducer *out );		// gets predicted value

// Update Fcns - updates matrix/array with current values
int UpdateTransducerValue(double updatevalue, transducer *out );// updates the sensed value
int UpdateTransducerValue2( transducer *out ); // updates the value without data
int UpdateTransducerPrevious( transducer *out ); // update the previous value with the current value
int UpdateTransducerPredicted(double W1, double W2, transducer *out );// updates the predicted value as a linear weighting
int UpdateTransducerValueWithPredicted(double W1, double W2, transducer *out );// inserts a weighted prediction within the value

// Compute Fcns
int ComputeTransducerInnovation( transducer *out  );


#endif  // define TRANSDUCER_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
