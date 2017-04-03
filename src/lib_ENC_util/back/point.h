/*
 * point.h
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

#ifndef POINT_H_
#define POINT_H_

class Point
{
public:
	// Default Constructor
	Point();

	// Overloaded Constructor
	// @param - X and Y position of an obstacle
	Point(double X, double Y);

	// Destructor
	~Point() {};

	// Sets the X, Y position of the vertex
	void setXY(double X, double Y){x=X;y=Y;};

	// Sets the t_lvl and type for the vertex
	void setStatics(int t_lvl, string Obs_Type);

	// Sets the angle of the vertex
	void setAngle(double Angle){ang=Angle;};

	// Sets the distance of the vertex
	void calcDist(double ASV_x, double ASV_y);

	// Sets the cost of the vertex
	void calcCost(double v_length, double speed, double maxutil);

	// Calculates the cost and distance of the vertex
	void calcLocation(double ASV_x, double ASV_y, double v_length,
		double speed, double maxutil) {calcDist(ASV_x, ASV_y); calcCost(v_length, speed, maxutil);}

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


protected:
	// Member Variables
	double ang, cost, dist;
	double x,y;
	int t_lvl;
	string obs_type; 
};



#endif /* POINT_H_ */
