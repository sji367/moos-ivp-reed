/*
 * poly.h
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include "point.h"
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

using namespace std;

#ifndef POLY_H_
#define POLY_H_

class Poly : public Point
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
	
	// Sets the angle of the vertex
	void setAngle(double Angle, double boundary);

	// Sets the t_lvl and type for the vertex
	void setStatics(int Ref_Frame, int T_Lvl, string Obs_Type);

	// Outputs the slope of the "V" vertex of the obstacle
	double getM() {return m;};

	// Outputs the y-intercept of the "V" vertex of the obstacle
	double getB() {return b;};
	
	// Outputs the reference frame of the obstacle
	double getRefFrame() {return ref_frame;};

protected:
	// Member Variables
	double m,b;
	int ref_frame;
};



#endif /* POLY_H_ */

