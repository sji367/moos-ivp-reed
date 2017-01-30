/*
 * point.h
 *
 *  Created on: Jan 20, 2017
 *      Author: sji36
 */

#include "OA.h"
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

using namespace std;

#ifndef POINT_H_
#define POINT_H_

class Point : public OA
{
public:
	// Default Constructor
	Point();

	// Overloaded Constructor
	Point(double X, double Y);

	// Destructor
	~Point() {};

	// Sets the cost of the "V" vertex
	void calcCost(double v_length, double speed, double maxutil);

	// Calculates the cost and distance of the vertex
	void calcLocation(double ASV_x, double ASV_y, double v_length,
		double speed, double maxutil) {OA::calcDist(ASV_x, ASV_y); calcCost(v_length, speed, maxutil);}

private:
	// Member Variables
};



#endif /* POINT_H_ */
