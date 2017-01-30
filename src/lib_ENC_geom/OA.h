/*
 * OA.h
 *
 *  Created on: Jan 20, 2017
 *      Author: sji36
 */

#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

using namespace std;

#ifndef OA_H_
#define OA_H_
class OA
{
public:
	// Default Constructor
	OA();

	// Overloaded Constructor
	// @param - X and Y position of an obstacle
	OA(double, double);

	// Destructor
	~OA() {};

	// Sets the X, Y position of the vertex
	void setXY(double X, double Y){x=X;y=Y;};

	// Sets the reference frame, t_lvl, and type for the point
	void setStatics(int Ref_Frame, int t_lvl, string Obs_Type);

	// Sets the angle of the vertex
	void setAngle(double Angle){ang=convert_ref_frame(Angle);};

	// Sets the distance of the vertex
	void calcDist(double ASV_x, double ASV_y);

	// Outputs the distance from the vertex of the obstacle to the ASV
	double getDist() {return dist;};

	// Outputs the cost of the vertex of the obstacle
	double getCost() {return cost;};

	// Outputs the angle of the vertex of the obstacle
	double getAngle() {return ang;};

	// Outputs the X position of the vertex of the obstacle
	double getX() {return x;};

	// Outputs the Y position of the vertex of the obstacle
	double getY() {return y;};

	// Outputs the threat level of the obstacle
	double getTLvL() {return t_lvl;};

	// Outputs the obstacle type of the obstacle
	string getObsType() {return obs_type;};

	// Outputs the reference frame of the obstacle
	double getRefFrame() {return ref_frame;};

	// Converts the angle to the right domain and then returns the corrected value
	//	case 1: // Domain [0, 360]
	//	case 2: // Domain [-90, 270]
	//	case 3: // Domain [-180, 180]
	//	case 4: // Domain [-270, 90]
	double convert_ref_frame(double Angle);

protected:
	// Member Variables
	double ang, cost, dist;
	double x,y;
	int t_lvl, ref_frame;
	string obs_type; 
};


#endif /* OA_H_ */
