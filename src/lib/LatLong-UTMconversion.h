//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions
/* $Id: LatLong-UTMconversion.h,v 1.2 2005/06/06 18:53:07 dave Exp $ */
#include <string.h>

#ifndef LATLONGCONV
#define LATLONGCONV

void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, double *UTMNorthing, double *UTMEasting, char* UTMZone);
void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone, double* Lat,  double* Long );
char UTMLetterDesignator(double Lat);
void LLtoSwissGrid(const double Lat, const double Long, double *SwissNorthing, double *SwissEasting);
void SwissGridtoLL(const double SwissNorthing, const double SwissEasting, double *Lat, double *Long);

// convert the DMS number to a decimal verion
int  ConvertDMS_Decimal ( double degrees, double minutes, double seconds, double *out );

/*

class Ellipsoid
{
public:
	Ellipsoid(){};
	Ellipsoid(int Id, char* name, double radius, double ecc)
	{
		id = Id; ellipsoidName = name; 
		EquatorialRadius = radius; eccentricitySquared = ecc;
	}

	int id;
	char* ellipsoidName;
	double EquatorialRadius; 
	double eccentricitySquared;  

};

*/

#endif
