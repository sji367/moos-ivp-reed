/*
 * crit_pts_poly.h
 *
 *  Created on: Feb 9, 2017
 *      Author: sji36
 */

#include <iostream>  // for cout
#include <numeric>   // for adjacent_difference
#include <vector>    // for vector
#include <stdlib.h>  // for rand
#include <algorithm> // for all_of

#define MAX_NUM 360

using namespace std;

#ifndef ENVELOPE_H_
#define ENVELOPE_H_

class Envelope
{
public:
	// Default Constructor
        Envelope() {min_ang=0; max_ang =0;}

	// Default Constructor
        ~Envelope() {}

	// Prints a 2D vector
	void print2D_vect(vector<vector<double>> vect);

	// Returns the index of the vector that is more than 5
	double morethan5(vector<double> vect);

	// If gap is 5 or greater interpolate between the points, overwise
	//  store the angle. Each row consists of:
	//      [angle, vertex_index, interpolation_index]
	void store_angle(double ang, double prev_ang, double vertex_index,
			 vector<vector<double>>& vertex);

	// Function to help build the rows for the vertices 2D vector
	//     [angle, vertex_index, interpolation_index]
	vector<double> build_row(double ang, double vertex_index, double interp_index);

	// Determine the adjacent difference for first column of the 2D vertex
	void adj_diff2D(vector<vector<double>>,vector<double>&);

	// Calculate the envelope (points of min and max angle)
	void calcEnvelope(vector<vector<double>>);

	// Outputs the critical point: Minimum Angle
        double GetMinAng() {return min_ang;}

	// Outputs the critical point: Maximum Angle
        double GetMaxAng() {return max_ang;}

	// Outputs the critical point: Minimum Angle
        double GetMinAngIndex() {return ind_min_ang;}

	// Outputs the critical point: Maximum Angle
        double GetMaxAngIndex() {return ind_max_ang;}
private:
	// Member Variables
	double min_ang, max_ang, ind_min_ang, ind_max_ang;
	
};

#endif  /* ENVELOPE_H_ */

// Driver function to sort the 2D vector on basis of a particular column
bool sortcol(const vector<double>& v1, const vector<double>& v2);
