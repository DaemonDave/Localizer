// template.h
// template Header File
// structs and fcns to create and use _______________
/* $Id: template.h,v 1.2 2005/06/06 18:53:08 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef TEMPLATE_H
#define TEMPLATE_H

// Includes



// Defines

#define DATA_TYPE_LENGTH	36

// Data structs

  


typedef struct
{
	double test;
} example;


// Functions 

// Constructors - create data structs
example * CreateExample( void );	// creates and returns dynamic memory

// Destructors

// Zero Fcns - zero the elements


// Get/Set Functions - resets specific values into the data struct

// Update Fcns - updates matrix/array with current values

// Compute Fcns


#endif  // define TEMPLATE_H

#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
