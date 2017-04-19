/*
 * astar.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <iostream>
#include <string>
#include <ctime>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort
#include <queue> // for priority_queue
#include "strtk.hpp" // for csv parsing
//#include "MBUtils.h"
#include <iostream>
#include <fstream>
#include "L84.h"

using namespace std;

#ifndef ASTAR_H_
#define ASTAR_H_

class Vessel_Dimensions
{
public:
	Vessel_Dimensions() {set_ship_meta(0, 0, 0, 0);};
	Vessel_Dimensions(double Length, double Width, double Draft, double turn_radius) {set_ship_meta(Length, Width, Draft, turn_radius); };
	~Vessel_Dimensions() {};

	// Get Vessel dimensions from user
	void getVesselMeta();
	void set_ship_meta(double Length, double Width, double Draft, double turn_radius) {TurnRadius=turn_radius; length=Length; width=Width, draft=Draft; };

	// Output the private variables (vessel meta data)
	double getTurnRadius() {return TurnRadius; };
	double getLength() {return length; };
	double getWidth() {return width; };
	double getDraft() {return draft; };

private:
	double TurnRadius, length, width, draft;
};

class Node
{
public:
	// Constuctors/Deconstructor
	Node() {xPos=0; yPos=0; current_cost=0; priority=0; depth=0; ShipMeta=Vessel_Dimensions(); };
	Node(int x, int y, int Depth, int prev_cost, int new_priority) {xPos=x; yPos=y; depth=Depth; current_cost=prev_cost; priority=new_priority; };
	Node(int x, int y, int Depth, int prev_cost, int new_priority, Vessel_Dimensions meta) {xPos=x; yPos=y; depth=Depth; current_cost=prev_cost; priority=new_priority; setShipMeta(meta); };
	~Node() {};

	// Outputs the X position of the node
	int getX() {return xPos; };

	// Outputs the Y position of the node
	int getY() {return yPos; };

	// Outputs the current cost
	double getCost() {return current_cost; };

	int getDepth() {return depth; };

	// Outputs the current estimated priority
	double getPriority() {return priority; };
	double getPriority() const {return priority; };

	// Set the new cost to go to the node 
	void calcCost(int dx, int dy) {current_cost += calcDistance(dx,dy); }; // (10*distance as an int)
	void calcCost(int dx, int dy, int depth_thresh) {current_cost += calcDistance(dx,dy) + depthCost(depth_thresh); }; // (10*distance as an int) + cummilative depth
	void calcCost(int dx, int dy, int old_depth, double speed) {int dist=calcDistance(dx,dy); current_cost += dist + time2shoreCost(old_depth, speed, dist); };// (10*distance as an int) + time2crash
	
	int calcMinDepth();
	int depthCost(int depth_threshold);
	int time2shoreCost(int old_depth, double speed, int dist);
	
	// Estimate the remaining cost to go to the destination 
	int estimateRemainingCost(int xDest, int yDest) {return calcDistance(xPos-xDest, yPos-yDest); };
	
	// Update the priority
	void updatePriority(int xDest, int yDest) {priority = current_cost + estimateRemainingCost(xDest, yDest); };

	// Distance fomula
	int calcDistance(int dx, int dy) {return static_cast<int>(sqrt(dx*dx+dy*dy)*10); };

	void setShipMeta(Vessel_Dimensions meta) {ShipMeta = Vessel_Dimensions(meta.getLength(), meta.getWidth(), meta.getDraft(), meta.getTurnRadius()); };

protected:
	int xPos, yPos;
	int current_cost, priority;
	int depth;
	Vessel_Dimensions ShipMeta;
};


class A_Star
{
public:
	// Constuctors/Deconstructor
	A_Star();
	A_Star(int connecting_dist);
	A_Star(int x1, int y1, int x2, int y2, int depthCutoff, int connecting_dist);
	A_Star(int x1, int y1, int x2, int y2, int depthCutoff, double gridSize, double TopX, double TopY, int connecting_dist);
	~A_Star() {};

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
        string ReconstructPath(vector<vector<int> > direction_map);

	// Check to see if the extened path is valid
	bool extendedPathValid(int i, int x, int y);

	// This function runs A* search. It outputs the generated path as a string
	string AStar_Search();

	// This function runs A* and prints out the result
	bool runA_Star(bool yes_print, bool MOOS_WPT, bool L84_WPT, string filename, double LatOrigin, double LongOrigin);
        bool runA_Star(bool yes_print, bool L84_WPT, string filename, double LatOrigin, double LongOrigin) {return runA_Star(yes_print,false,L84_WPT, filename, LatOrigin,LongOrigin); };
        bool runA_Star(bool yes_print, bool MOOS_WPT, string filename) {return runA_Star(yes_print,MOOS_WPT,false,filename, 0,0); };
        bool runA_Star(bool yes_print) {return runA_Star(yes_print,false,false, "", 0,0); };
        bool runA_Star() {return runA_Star(true,false,false, "", 0,0); };
	
	// This function marks the route on the map
	string markRoute(vector<int> route);

	//This function prints out the map to std output where:
        //        '.' = space
        //        '0' = Obstacle
        //        'S' = Start
        //        'F' = Finish
        //        'X' = New Waypoint
        //        '@' = route (no new waypoint)
	void printMap(string route, double total_time);

	// Build the default map (Simple Map with a cross down the middle)
	void build_default_map(int N, int M, int config);

	// Use your own map from a csv file that contains the grid
	void build_map(string filename);
	void build_map(string filename, int x_min, int x_max, int y_min, int y_max); // Builds master map and then only uses subset of map for A*

	// Set Occupancy Grid
        void setMap(vector<vector<int> > MAP) {Map=MAP; FullMap=MAP; n=MAP.size(); m=MAP[0].size(); setGridXYBounds(0,m,0,n); };

	// Use a subset of the map for the A* search
	void subsetMap(int xmin, int xmax, int ymin, int ymax);

	// Outputs either dx or dy
	vector<int> get_dx() {return dx; };
	vector<int> get_dy() {return dy; };
	
	// Gets Finish position
	int getStartX() {return xStart; };
	int getStartY() {return yStart; };

	// Gets Finish position
	int getFinishX() {return xFinish; };
	int getFinishY() {return yFinish; };

	// Get the parent of the node
	int getParent(int i) {return (num_directions-i-1)%num_directions; };
	int getNumDir() {return num_directions; };

	// Get the start/finish waypoint warnings
	bool getStartValid() {return valid_start; };
	bool getFinishValid() {return valid_finish; };

	// These functions set the start and finsh coordinates (in the grid world)
	void setStart_Grid(int x, int y) {xStart = x; yStart =y; };
	void setFinish_Grid(int x, int y) {xFinish = x; yFinish = y; };
	void setStartFinish_Grid(int x1, int y1, int x2, int y2)  {setStart_Grid(x1,y1); setFinish_Grid(x2, y2); };

	// These functions set the start and finsh coordinates (in the grid world)
	void setStart(double x, double y) {int gridX,gridY; xy2grid(x,y,gridX,gridY); xStart = gridX; yStart = gridY; };
	void setFinish(double x, double y) {int gridX,gridY; xy2grid(x,y,gridX,gridY); xFinish = gridX; yFinish = gridY; };
	void setStartFinish(double x1, double y1, double x2, double y2)  {setStart(x1,y1); setFinish(x2, y2); };

	// Check to see if the start/finish waypoints are valid
	void checkStart();
	void checkFinish();
	void checkStartFinish() {checkStart(); checkFinish(); };

	// Set the depth cutoff (in centimeters) for being classified as an obstacle
	void setDepthCutoff(int depthCutoff) {depth_cutoff = depthCutoff; };

	// Set the tide
	void setTide(double Tide) {tide=Tide; setDepthCutoff(depth_cutoff+Tide); };

	// Sets the bounds of the grid (used when runing A* on a subset of the overall map)
	void setGridXYBounds(int xmin, int xmax, int ymin, int ymax) {x_min = xmin; x_max = xmax; y_min = ymin; y_max = ymax; };

	// Sets the metadata for conversions to/from the grid world 
	void setConversionMeta(double gridSize, double topX, double topY) {grid_size=gridSize; xTop = topX; yTop = topY; };

	// Return Vessel dimensions
	Vessel_Dimensions getVesselMeta() {return ShipMeta; };
	void setShipMeta(Vessel_Dimensions meta) {ShipMeta = Vessel_Dimensions(meta.getLength(), meta.getWidth(), meta.getDraft(), meta.getTurnRadius()); setDepthCutoff(calcMinDepth()); };
	int calcMinDepth();

	// Set desired speed
	void setDesiredSpeed();
	void setDesiredSpeed(double Speed) {desired_speed=Speed; };
	double getDesiredSpeed() {return desired_speed; };

	// Output .moos file
	void buildMOOSFile(string filename, string WPTs);

	// Functions to convert to/from the grid world
	void xy2grid(double x, double y, int &gridX, int &gridY) {gridX = (x-xTop-grid_size/2.0)/grid_size- x_min; gridY = (y-yTop-grid_size/2.0)/grid_size- y_min; };
	void grid2xy(double &x, double &y, int gridX, int gridY) {x = (gridX+x_min)*grid_size+ xTop+grid_size/2.0; y = (gridY+y_min)*grid_size+ yTop+grid_size/2.0; };
        
protected:
	bool valid_start, valid_finish;
	int xStart, yStart, xFinish, yFinish;
	int n, m, num_directions;
	double grid_size, xTop, yTop;
	int x_min, x_max, y_min, y_max;
	double tide;
	double desired_speed;
	Vessel_Dimensions ShipMeta;
	int depth_cutoff; //in Centimeters
	vector<int> dx, dy;
        vector<vector<int> > Map;
        vector<vector<int> > FullMap;
};

bool operator<(Node& lhs, Node& rhs);

#endif /* ASTAR_H_ */
