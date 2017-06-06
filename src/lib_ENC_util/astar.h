/*
 * astar.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
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
#include <stdio.h>

using namespace std;

#ifndef ASTAR_H_
#define ASTAR_H_


/* --------------------------------------------------------------------------
This class is used to store the information on the vessel's metadata including
the ship's length, width, draft and turning radius. 
--------------------------------------------------------------------------- */
class Vessel_Dimensions
{
public:
        Vessel_Dimensions() {set_ship_meta(0, 0, 0, 0);}
        Vessel_Dimensions(double Length, double Width, double Draft, double turn_radius) {set_ship_meta(Length, Width, Draft, turn_radius); }
        ~Vessel_Dimensions() {}

	// Get Vessel dimensions from user
	void getVesselMeta();
        void set_ship_meta(double Length, double Width, double Draft, double turn_radius) {TurnRadius=turn_radius; length=Length; width=Width, draft=Draft; }

	// Output the private variables (vessel meta data)
        double getTurnRadius() {return TurnRadius; }
        double getLength() {return length; }
        double getWidth() {return width; }
        double getDraft() {return draft; }

private:
	double TurnRadius, length, width, draft;
};


/* --------------------------------------------------------------------------
This class is used in A* to store the information on the nodes of the graph.
The cost for each node can be determined with 3 different methods:
	-minimizing the distance traveled
	-combination of the minimum distance traveled and maximizing the
		depth under the ship
	-combination of the minimum distance traveled and maximizing the 
		time to shore
--------------------------------------------------------------------------- */
class Node
{
public:
	// Constuctors/Deconstructor
        Node() {xPos=0; yPos=0; current_cost=0; priority=0; depth=0; ShipMeta=Vessel_Dimensions(); }
        Node(int x, int y, int Depth, int prev_cost, double Grid_size) {xPos=x; yPos=y; depth=Depth/100.0; current_cost=prev_cost; priority=0; grid_size = Grid_size; ShipMeta=Vessel_Dimensions();}
        Node(int x, int y, int Depth, int prev_cost, double Grid_size, Vessel_Dimensions meta) {xPos=x; yPos=y; depth=Depth/100.0; current_cost=prev_cost; priority=0; grid_size = Grid_size; setShipMeta(meta); }
        ~Node() {}

	// Outputs the X position of the node
        int getX() {return xPos; }

	// Outputs the Y position of the node
        int getY() {return yPos; }

	// Outputs the current cost
        double getCost() {return current_cost; }

        int getDepth() {return depth; }

	// Outputs the current estimated priority
        double getPriority1() {return priority; }
        double getPriority() const {return priority; }

        // Set the new cost to go to the node where the distance is stored as an int
        void calcCost(int dx, int dy) {current_cost += calcDistance(dx,dy); } // cumlative distance traveled
        // cumlative distance traveled + cummilative depth (in cm)
        void calcCost_depth(int dx, int dy) {double dist = calcDistance(dx,dy); current_cost += dist + depthCost(dist); }
        // cumlative distance traveled + time2crash
        void calcCost(int dx, int dy, int old_depth, double speed) {double dist=calcDistance(dx,dy); current_cost += dist + time2shoreCost(old_depth, speed, dist); }
	
	// Calculates the depth classified as an obstacle by A* as 300*draft (aka depth in cm)
	int calcMinDepth();
        int depthCost(double dist); // Calculates the cost of traveling through the cells depth
        double time2shoreCost(int old_depth, double speed, int dist); // Calculates the cost based on the approximate time to crashing into the shore
	
	// Estimate the remaining cost to go to the destination (heuristic - straight line distance)
        int estimateRemainingCost(int xDest, int yDest) {return calcDistance(xPos-xDest, yPos-yDest); }
	
	// Update the priority
        void updatePriority(int xDest, int yDest) {priority = current_cost + estimateRemainingCost(xDest, yDest); }

	// Distance fomula
        double calcDistance(int dx, int dy) {return sqrt(dx*dx+dy*dy)*grid_size; }

        void setShipMeta(Vessel_Dimensions meta) {ShipMeta = Vessel_Dimensions(meta.getLength(), meta.getWidth(), meta.getDraft(), meta.getTurnRadius()); }

protected:
	int xPos, yPos;
        double current_cost, priority, grid_size;
        double depth;
	Vessel_Dimensions ShipMeta;
};

/* --------------------------------------------------------------------------
This class actually runs the A* graph search algorithm. There are a multitude
of different options however in the typical use, the constructor is called 
which allows the user to set how how many neighbors to investagate, the depth
that is classified as an obstacle by A*, the start and finish coordinates, and 
a few other options. Then the map is built using a preloaded csv file, a simple
default map of an cross, or from an ENC (STILL NEEDS TO BE DONE). The start
and finish positions can be inputed at any time and are in either the grid 
coordinate system or in local UTM. Then the user can either run A* from a set
of commands:
	-AStar_Search: runs the search algorithm without checking if the 
		waypoints are valid and outputs the A* generated waypoints 
		in a comma seperated string in the grid coordinate system
	-runA_Star: Runs A* after checking the waypoints to see if they are
		valid. Once A* has been run, the function can then be set to
		output different types of files:
			-Print to STD output
			-Output a .bhv MOOS file
			-Output a L84 file
		If the user wants to generate a new MOOS and/or L84 file, a
		base filename must be given.  
--------------------------------------------------------------------------- */
class A_Star
{
public:
	// Constuctors/Deconstructor
	A_Star();
	A_Star(int connecting_dist);
        A_Star(double gridSize, double TopX, double TopY, int connecting_dist);
	A_Star(int x1, int y1, int x2, int y2, int depthCutoff, int connecting_dist);
	A_Star(int x1, int y1, int x2, int y2, int depthCutoff, double gridSize, double TopX, double TopY, int connecting_dist);
        ~A_Star() {}

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
        bool extendedPathValid(int i, int wptX, int wptY, int &depth_cost);

	// This function runs A* search. It outputs the generated WPTs as a comma seperated string
	string AStar_Search();

	// This function runs A* and prints out the result
	bool runA_Star(bool yes_print, bool MOOS_WPT, bool L84_WPT, string filename, double LatOrigin, double LongOrigin);
        bool runA_Star(bool yes_print, bool L84_WPT, string filename, double LatOrigin, double LongOrigin) {return runA_Star(yes_print,false,L84_WPT, filename, LatOrigin,LongOrigin); }// Boolean for std out and L84
        bool runA_Star(bool yes_print, bool MOOS_WPT, string filename) {return runA_Star(yes_print,MOOS_WPT,false,filename, 0,0); } // Boolean for std out and MOOS
        bool runA_Star(bool yes_print) {return runA_Star(yes_print,false,false, "", 0,0); } // Boolean for std out
        bool runA_Star() {return runA_Star(true,false,false, "", 0,0); }// just to std out
	
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
        void setMap(vector<vector<int> > MAP) {Map=MAP; FullMap=MAP; cout << "set Map" <<endl; n=MAP.size(); m=MAP[0].size(); setGridXYBounds(0,m,0,n); }

	// Use a subset of the map for the A* search
	void subsetMap(int xmin, int xmax, int ymin, int ymax);

	// Outputs either dx or dy
        vector<int> get_dx() {return dx; }
        vector<int> get_dy() {return dy; }
	
	// Gets Finish position
        int getStartX() {return xStart; }
        int getStartY() {return yStart; }

	// Gets Finish position
        int getFinishX() {return xFinish; }
        int getFinishY() {return yFinish; }

	// Get the parent of the node
        int getParent(int i) {return (num_directions-i-1)%num_directions; }
        int getNumDir() {return num_directions; }

	// Get the start/finish waypoint warnings
        bool getStartValid() {return valid_start; }
        bool getFinishValid() {return valid_finish; }

	// These functions set the start and finsh coordinates (in the grid world)
        void setStart_Grid(int x, int y) {xStart = x; yStart =y; }
        void setFinish_Grid(int x, int y) {xFinish = x; yFinish = y; }
        void setStartFinish_Grid(int x1, int y1, int x2, int y2)  {setStart_Grid(x1,y1); setFinish_Grid(x2, y2); }

	// These functions set the start and finsh coordinates (in the grid world)
        void setStart(double x, double y) {int gridX,gridY; xy2grid(x,y,gridX,gridY); xStart = gridX; yStart = gridY; }
        void setFinish(double x, double y) {int gridX,gridY; xy2grid(x,y,gridX,gridY); xFinish = gridX; yFinish = gridY; }
        void setStartFinish(double x1, double y1, double x2, double y2)  {setStart(x1,y1); setFinish(x2, y2); }

	// Check to see if the start/finish waypoints are valid
	void checkStart();
	void checkFinish();
        void checkStartFinish() {checkStart(); checkFinish(); }

	// Set the depth cutoff (in centimeters) for being classified as an obstacle
        void setDepthCutoff(int depthCutoff) {depth_cutoff = depthCutoff; }
        int getDepthCutoff() {return depth_cutoff; }
        int calcDepthCost(int wptX,int wptY, int i);

	// Set the tide
        void setTide(double Tide) {tide=Tide; setDepthCutoff(depth_cutoff+Tide); }

	// Sets the bounds of the grid (used when runing A* on a subset of the overall map)
        void setGridXYBounds(int xmin, int xmax, int ymin, int ymax) {x_min = xmin; x_max = xmax; y_min = ymin; y_max = ymax; }

	// Sets the metadata for conversions to/from the grid world 
        void setConversionMeta(double gridSize, double topX, double topY) {grid_size=gridSize; xTop = topX; yTop = topY; }

	// Return Vessel dimensions
        Vessel_Dimensions getVesselMeta() {return ShipMeta; }
        void setShipMeta(Vessel_Dimensions meta) {ShipMeta = Vessel_Dimensions(meta.getLength(), meta.getWidth(), meta.getDraft(), meta.getTurnRadius()); setDepthCutoff(calcMinDepth()); }
	int calcMinDepth();

	// Set desired speed
	void setDesiredSpeed();
        void setDesiredSpeed(double Speed) {desired_speed=Speed; }
        double getDesiredSpeed() {return desired_speed; }

	// Output .moos file
	void buildMOOSFile(string filename, string WPTs);

	// Functions to convert to/from the grid world
        void xy2grid(double x, double y, int &gridX, int &gridY) {gridX = (x-xTop-grid_size/2.0)/grid_size- x_min; gridY = (y-yTop-grid_size/2.0)/grid_size- y_min; }
        void grid2xy(int gridX, int gridY, double &x, double &y) {x = (gridX+x_min)*grid_size+ xTop+grid_size/2.0; y = (gridY+y_min)*grid_size+ yTop+grid_size/2.0; }

	void getNM() {cout << Map.size() << ", " << Map[0].size() << endl; }
        
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
	vector<vector<int> > Map2print;
};

bool operator<(Node& lhs, Node& rhs);

#endif /* ASTAR_H_ */
