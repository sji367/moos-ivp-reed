/*
 * astar.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <iostream>
#include "MBUtils.h"
#include <string>
#include <ctime>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort
#include <queue> // for priority_queue
#include "strtk.hpp" // for csv parsing

using namespace std;

#ifndef ASTAR_H_
#define ASTAR_H_

class Node
{
public:
	// Constuctors/Deconstructor
	Node() {xPos=0; yPos=0; current_cost=0; priority=0; };
	Node(int x, int y, double prev_cost, double new_priority) {xPos=x; yPos=y; current_cost=prev_cost; priority=new_priority; };
	~Node() {};

	// Outputs the X position of the node
	int getX() {return xPos; };

	// Outputs the Y position of the node
	int getY() {return yPos; };

	// Outputs the current cost
	double getCost() {return current_cost; };

	// Outputs the current estimated priority
	double getPriority() {return priority; };
	double getPriority() const {return priority; };

	// Set the new cost to go to the node (10*distance as an int)
	void calcCost(int dx, int dy) {current_cost += calcDistance(dx,dy); };
	
	// Estimate the remaining cost to go to the destination 
	int estimateRemainingCost(int xDest, int yDest) {return calcDistance(xPos-xDest, yPos-yDest); };
	
	// Update the priority
	void updatePriority(int xDest, int yDest) {priority = current_cost + estimateRemainingCost(xDest, yDest); };

	// Distance fomula
	int calcDistance(int dx, int dy) {return static_cast<int>(sqrt(dx*dx+dy*dy)*10); };
	
protected:
	int xPos, yPos;
	int current_cost, priority;
};


class A_Star
{
public:
	// Constuctors/Deconstructor
	A_Star();
	A_Star(int x1, int y1, int x2, int y2, int connecting_dist);
	A_Star(int connecting_dist);
	~A_Star() {};

	// These functions set the start and finsh coordinates
	void setStartX(int x) {xStart = x; };
	void setStartY(int y) {yStart = y; };
	void setFinishX(int x) {xFinish = x; };
	void setFinishY(int y) {yFinish = y; };
	void setStart(int x, int y) {setStartX(x); setStartY(y); };
	void setFinish(int x, int y) {setFinishX(x); setFinishY(y); };
	void setStartFinish(int x1, int y1, int x2, int y2)  {setStart(x1,y1); setFinish(x2, y2); };

	void setGridXYBounds(int xmin, int xmax, int ymin, int ymax) {x_min = xmin; x_max = xmax; y_min = ymin; y_max = ymax; };

	// Number of Neighboors one wants to investigate from each cell. A larger
        //    number of nodes means that the path can be alligned in more directions.
	//
        //    Connecting_Distance=1-> Path can  be alligned along 8 different direction.
        //    Connecting_Distance=2-> Path can be alligned along 16 different direction.
        //    Connecting_Distance=3-> Path can be alligned along 32 different direction.
        //    Connecting_Distance=4-> Path can be alligned along 56 different direction.
        //    ETC......
	void NeighborsMask(int Connecting_Dist);

	//Generate the path from finish to start by following the directions
        //    in the direction map.
	vector<int> ReconstructPath(vector<vector<int>> direction_map);

	// This function runs A* search. It outputs the generated path as an string
        //    where each movement is stored as a number corrisponding to the
        //    index of the neighbor mask.
	vector<int> AStar_Search();

	// This function runs A* and prints out the result
	void runA_Star(bool print_meta);
	void runA_Star() {runA_Star(true); };
	
	// This function marks the route on the map
	string markRoute(vector<int> route);

	//This function prints out the map to std output where:
        //        '.' = space
        //        '0' = Obstacle
        //        'S' = Start
        //        'F' = Finish
        //        'X' = New Waypoint
        //        '@' = route (no new waypoint)
	void printMap(vector<int> route, double total_time, bool print_meta);
	void printMap(vector<int> route, double total_time) {printMap(route, total_time, true); };

	// Set Occupancy Grid
	void setMap(vector<vector<int>> MAP) {Map=MAP; n=MAP.size(); m=MAP[0].size(); setGridXYBounds(0,0,m,n); };

	// Build the default map (Simple Map with a cross down the middle)
	void build_default_map(int N, int M, int config);

	// Use your own map from a csv file that contains the grid
	void build_map(string filename);
	void build_map(string filename, int x_min, int x_max, int y_min, int y_max); // Builds master map and then only uses subset of map for A*

	// Use a subset of the map for the A* search
	void subsetMap(int xmin, int xmax, int ymin, int ymax);

	// Outputs either dx or dy
	vector<int> get_dx() {return dx; };
	vector<int> get_dy() {return dy; };

	// Gets Finish position
	int getFinishX() {return xFinish; };
	int getFinishY() {return yFinish; };

	// Get the parent of the node
	int getParent(int i) {return (num_directions-i-1)%num_directions; };
	int getNumDir() {return num_directions; };

	// Check to see if the extened path is valid
	bool extendedPathValid(int i, int x, int y);

protected:
	int xStart, yStart, xFinish, yFinish, n, m, num_directions;
	int x_min, x_max, y_min, y_max;
	vector<int> dx, dy;
	vector<vector<int>> Map;
	vector<vector<int>> FullMap;
	bool default_map;
};

bool operator<(Node& lhs, Node& rhs);

#endif /* ASTAR_H_ */
