/*
 * geodesy.h
 *
 *  Created on: Mar 31, 2017
 *      Author: Sam Reed
 */

#include "ogr_spatialref.h"
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

#ifndef GEODESY_H_
#define GEODESY_H_


class Geodesy
{
public:
	// Constructor
	Geodesy() {Initialise(0, 0); UTM_Zone=0; };
	Geodesy(double LatOrigin, double LonOrigin) {Initialise(LatOrigin, LonOrigin); };
	~Geodesy() {};

	void Initialise(double Lat_Origin, double Lon_Origin);

	void LatLong2UTM(double Lat, double Lon, double &x, double &y);
	void UTM2LatLong(double x, double y, double &Lat, double &Lon);

	void LatLong2LocalUTM(double Lat, double Lon, double &x, double &y) {LatLong2UTM(Lat, Lon, x, y); x-=x_origin; y-=y_origin; };
	void LocalUTM2LatLong(double x, double y, double &Lat, double &Lon) {x+=x_origin; y+=y_origin; UTM2LatLong(x, y, Lat, Lon); };
	
	int getUTMZone(double longitude) {return 1+(longitude+180.0)/6.0; };

private:
	double LatOrigin, LonOrigin;
	double x_origin, y_origin;
	int UTM_Zone;
	OGRSpatialReference UTM, LatLong;
};

#endif /* GEODESY_H_ */
