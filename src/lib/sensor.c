//! sensor.c
//!
//! DRE 2005/05/30
//!
//! sensor Functions
/* $Id: sensor.c,v 1.4 2005/06/10 03:38:44 dave Exp $ */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef SENSOR_H
#include "sensor.h"
#endif


//!-------------------------------------------------------
//! CONSTRUCTORS  - assign memory only
//!-------------------------------------------------------
sensor * CreateSensor( void )
{
	//! creates only the container sensor struct
		
	//! assign dynamic memory
	return( (sensor *)malloc( sizeof(sensor)) );

}//! end CreateExample


//!-------------------------------------------------------
//! Init Fcns
//!-------------------------------------------------------
int InitSensor (int num_transducers, sensor *out)
{
	//! inits a sensor with the defined number of sensors
	int i,j;
	
	//! clear time and previous time
	out->pt_sec 	= 0;
	out->pt_usec 	= 0;

	out->t_sec 		= 0;
	out->t_usec 	= 0;	
	
	//! clear delta t
	out->delta_time = 0.0;
	
	//! assign the number of transducers
	if (num_transducers != -1 )
	{
		//!if not pre-assigned then update the number
		out->transducers = num_transducers;
	}
	//! create memory for each transducer pointer
	//! assign memory for address of transducer
	if ( (out->transducers) > 0 )//! so long as there are transducers set
	{
		out->array = (transducer **) malloc( num_transducers * sizeof( transducer *));		
	}
	else
	{
		return -1;
	}
	
	//! create memory for each transducer
	for (i = 0; i < num_transducers; i++ )
	{
		//! create transducer type		
		out->array[i] = CreateTransducer();		
	}
	
	//! zero all transducers
	for (i = 0; i < num_transducers; i++ )
	{
		ZeroTransducer ( out->array[i] );	
	}
	
	//! create measurement array to point to measured values
	out->measurement = ( double ** )malloc( num_transducers * sizeof( double * ));	
	
	//! aim measurements at transducer values
	for (i = 0; i < num_transducers; i++ )
	{
		//! create a row for each sensor
		out->measurement[i] = &(out->array[i]->value);		
	}	
	
	//! create measurement array to point to measured values
	out->filtered = ( double * )malloc( num_transducers * sizeof( double ));	
	
	//! clear filtered 
	for (i = 0; i < num_transducers; i++ )
	{
		//! zero values
		out->filtered[i] = 0.0;		
	}	
	
	//! zero the data indicator
	out->updated = 0;
	
	//! NULL ptr for all fcn pointers
	//!out->GenerateStateVector = NULL;
	//!out->GenerateVelMatrix = NULL;
	//!out->UpdateSensor = NULL;
	
	//!  create Kalman filter
	out->filter = CreateKFilter();
	
	//! init filter with number of transducers
	//! aim measurement and filtered
	InitKFilter2 ( out->filter , num_transducers, *out->measurement, out->filtered  );
	
	//! zero sensor
	ZeroSensor ( out );
	
	return 0;
}

//!-------------------------------------------------------
//! Zero Fcns
//!-------------------------------------------------------
int ZeroSensor( sensor *out )
{
	int i,j;
	//! zeros all elements of the sensor
	
	//! clear time and previous time
	out->pt_sec 	= 0;
	out->pt_usec 	= 0;

	out->t_sec 		= 0;
	out->t_usec 	= 0;
	
	//! clear delta t
	out->delta_time = 0.0;
	
	//! iterate through all transducers and zero elements
	for (i = 0; i < (out->transducers); i++)
	{
		ZeroTransducer ( out->array[i] );		
	}
	
		//! clear filtered 
	for (i = 0; i < out->transducers; i++ )
	{
		//! zero values
		out->filtered[i] = 0.0;		
	}
	
	//! zeros values 
	for (i = 0; i < out->transducers; i++)
	{
		//! create a row for each sensor
		*out->measurement[i] = 0.0;			

	}	
	
	//! NULL ptr for all fcn pointers
	//out->GenerateStateVector = NULL;
	//out->GenerateVelMatrix = NULL;
	//out->UpdateSensor = NULL;
	
	//! zero filter
	ZeroKFilter ( out->filter );
	
	// Fix the time update bug by updating time now for each sensor
	// This shoud remove some instances of the "infinite time bug"
	// infinite time bug - large velocity or acceleration 
	// based on a huge delta time at the initialization of the filter
	// call update time once to set current time
	UpdateSensorTime2 ( out );
	// call update time again to cancel large delta time after two successive 
	// function calls in rapid succession
	UpdateSensorTime2 ( out );
	
	return 0;
}


int ZeroMeasurement ( sensor *out )
{
	int i,j;
	
	//! error checking: no assigned covariances
	if (out->transducers > 0)
	{
		//! zeros values 
		for (i = 0; i < out->transducers; i++)
		{
			//! create a row for each sensor
			*out->measurement[i] = 0.0;			

		}
	}
	else
	{
		return -1; //! no covariance matrix
	}
	
	return 0;
}
//!-------------------------------------------------------
//! Get/Set Fcns
//!-------------------------------------------------------
int SetSensorTransducer ( double svalue, double spredicted, double svariance, int snumber,  sensor * out)
{
	//! set values
	out->array[snumber]->value 			= svalue;
	out->array[snumber]->predicted 	= spredicted;
	out->array[snumber]->variance 	= svariance;

	return 0;
}


//!-------------------------------------------------------
//! Update Fcns
//!-------------------------------------------------------
int UpdateSensorTransducerValue ( double svalue, int snumber, sensor * out)
{
	//! set value for sensed phenomena
	out->array[snumber]->value = svalue;
	
	//! force update of state of transducer
	UpdateTransducerValue(svalue, out->array[snumber] )	;
	

	return 0;
}

int UpdateSensorData( sensor * out, size_t size_data, void *data )
{
	//! Updates the sensor with or without
	//! data by either updating the transducer values or  
	//! using the predicted values inserted into the value
	int i;
	
	//! if the size of data is not zero, then pass on data
	if ( size_data > (size_t)0)
	{
		//! update the data within transducers
		out->updated = 1; //! there is new data
		//! update previous data in transducers with current value
		for (i = 0; i < out->transducers; i++)
		{
			UpdateTransducerPrevious( out->array[i] );
		}
		//! Generic call to the pointer for fcn to update data 
		out->UpdateSensor( size_data, data, out);
		
		//! compute innovation from prediction vs. value
		for (i = 0; i < out->transducers; i++)
		{
			ComputeTransducerInnovation(  out->array[i] );
		}	
		//! compute prediction from prediction and value
		for (i = 0; i < out->transducers; i++)
		{
			UpdateTransducerPredicted( 0.5, 0.5, out->array[i] );
		}		
		
	}
	else //! no data
	{
		//! update with predicted values in the transducer data
		out->updated = 0;//! indicate no new data
		//! update previous data in transducers with current value
		for (i = 0; i < out->transducers; i++)
		{
			UpdateTransducerPrevious( out->array[i] );
		}
		//! need to update time stamp
		UpdateSensorTime2(out);
		//! update data in transducers with predicted value
		for (i = 0; i < out->transducers; i++)
		{
			UpdateTransducerValueWithPredicted(0.9, 0.1,  out->array[i] );
		}

		//! compute innovation from prediction vs. value
		for (i = 0; i < out->transducers; i++)
		{
			ComputeTransducerInnovation(  out->array[i] );
		}	
		//! compute prediction from prediction and value
		for (i = 0; i < out->transducers; i++)
		{
			UpdateTransducerPredicted( 0.5, 0.5, out->array[i] );
		}			
		
	}

	return 0;	
}

int UpdateSensorTransducerPredicted ( int snumber, sensor * out)
{
	//! force update of predicted value of transducer	
	UpdateTransducerPredicted(0.5, 0.5, (transducer *)out->array[snumber] );

	return 0;
}

int UpdateSensorPredicted ( sensor * out)
{
	//! performs an update to all attached transducers
	//! of their predicted values
	int i;
	
	//! iterate through all transducers and update predicted values
	for (i = 0; i < out->transducers; i++)
	{
		//! iterate through attached transducers
		UpdateSensorTransducerPredicted ( i, out );
	}
	
	
	
	return 0;
}

int UpdateSensorTime (sensor *out, unsigned long time_sec, unsigned long time_usec)
{
	
	//! update the time data from sequential measurements
	(out->pt_sec) 	= (out->t_sec);
	(out->pt_usec) 	= (out->t_usec);
	
	//! update new time
	out->t_sec			= time_sec;
	out->t_usec 		= time_usec;
	
	//! compute delta time step adding microseconds to seconds
	out->delta_time  = ((double)out->t_sec - (double)out->pt_sec ) + (MICROSECOND_CONVERSION)*((double)out->t_usec - (double)out->pt_usec);
	

	return 0;
}//! end UpdateSensorTime

//! Bug fix- update time fcn to use computer time for everything
//! The IDLs do not all have timestamps, nor do some of the sensors
//! and so I will use the computer time for the moment to make realistic 
//! if not completely accurate computations.
int UpdateSensorTime2 (sensor *out )
{
	//! get time from computer and substitute it into the current time stamp
	//! and then compute the latest delta time for computation.
	struct timeval   computer_time;
	
	gettimeofday( &computer_time, 0 );//! timezone not implemented under linux, needs to be zero
	
	//! update the time data from sequential measurements
	(out->pt_sec) 	= (out->t_sec);
	(out->pt_usec) 	= (out->t_usec);
	
	//! update new time from computer
	out->t_sec			= ( unsigned long )computer_time.tv_sec;
	out->t_usec 		= ( unsigned long )computer_time.tv_usec;
	
	//! compute delta time step adding microseconds to seconds
	out->delta_time  = ((double)out->t_sec - (double)out->pt_sec ) + (MICROSECOND_CONVERSION)*((double)out->t_usec - (double)out->pt_usec);
	//printf("%e \n", out->delta_time );	

	return 0;
}//! end UpdateSensorTime2

//!-------------------------------------------------------
//! Transform Fcns
//!-------------------------------------------------------



//!-------------------------------------------------------
//! Compute Fcns
//!-------------------------------------------------------



#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif
