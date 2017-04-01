/*
 * L84.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip> // setprecision
#include <stdlib.h> // atof
#include "geodesy.h" // GDAL Geodesy

using namespace std;

#ifndef L84_H_
#define L84_H_

class L84
{
public:
	// Constructors/Deconstructors
	L84() {dfLatOrigin = 0; dfLongOrigin = 0; };
	L84(string filename, string WPT, double LatOrigin, double LongOrigin) {WPTs = WPT; L84FileName = filename+".L84"; dfLatOrigin = LatOrigin; dfLongOrigin = LongOrigin; };
	~L84() {};

	// Output the list of waypoints to a L84 file
	void writeL84();
private:
	string L84FileName, WPTs;

	Geodesy m_Geodesy;
	double dfLatOrigin, dfLongOrigin;
	
};

vector<string> split(const string& s, char c);

#endif /* L84_H_ */
