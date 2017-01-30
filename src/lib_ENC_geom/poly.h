/*
 * poly.h
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include "OA.h"
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

using namespace std;

#ifndef POLY_H_
#define POLY_H_

class Poly : public OA
{
public:
	// Default Constructor
	Poly();

	// Overloaded Constructor
	Poly(double X, double Y);

	// Destructor
	~Poly() {};

	// Calculates the slope and y-intercept of the "V" vertex
	//	***Use Right for max angle and Left for min angle***
	void setMB_Right_pnt(double cost, double angle);
	void setMB_Left_pnt(double cost, double angle);

	// Sets the cost of the "V" vertex
	void calcCost(double v_length, double speed, double maxutil);

	// Calculates the cost and distance of the vertex
	void calcLocation(double ASV_x, double ASV_y, double v_length,
		double speed, double maxutil) {OA::calcDist(ASV_x, ASV_y); calcCost(v_length, speed, maxutil);}

	// Outputs the slope of the "V" vertex of the obstacle
	double getM() {return m;};

	// Outputs the y-intercept of the "V" vertex of the obstacle
	double getB() {return b;};

protected:
	// Member Variables
	double m,b;
};



#endif /* POLY_H_ */

