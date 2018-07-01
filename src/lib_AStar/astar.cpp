/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH CCOM                                        */
/*    FILE: a_star.cpp                                      */
/*    DATE: 3/21/17                                         */
/************************************************************/

#include "astar.h"

double Node::calcMinDepth()
{
  double draft =ShipMeta.getDraft();
  if (draft > .33)
    return draft*3;
  else
    return 1;
}

// Depth threshold is also in meters and is 20 times the minimum depth
double Node::depthCost(double dist)
{
    double depth_threshold = 15;
    double depth_cost = 0;
    if (depth < depth_threshold)
        depth_cost = costMultiplier*dist*(depth_threshold-depth);

    return depth_cost;
}

double Node::time2shoreCost(double old_depth, double speed, double dist)
{
  // Calculate the time to shore
  double time_theshold, seafloor_slope, time2crash;
  double min_depth;
  double weight = 1;
  
  min_depth = calcMinDepth();
  time_theshold = 2*ShipMeta.getTurnRadius()*speed;
  seafloor_slope = (1.0*(old_depth-depth))/dist; //cm depth per meter spatially
  if (seafloor_slope <= 0)
    return 0;
  else
    {
      time2crash = (depth-min_depth)/(seafloor_slope*speed);
      
      if (time_theshold > time2crash){
	//cout << time2crash << endl;
	return weight*(time_theshold - time2crash);
      }
      else
	return 0;
    }
}

A_Star::A_Star()
{
    valid_start = true;
    valid_finish = true;
    startSet = false;
    finishSet = false;
    xTop = -4470; // value for the US5NH02M ENC
    yTop = -12439; // value for the US5NH02M ENC
    setStart_Grid(0,0);
    setFinish_Grid(0,0);
    depth_cutoff = 3; // Depths of more than 3 meter below MLLW are considered obstacles
    tide = 0;
    grid_size=5;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(8); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = 0.25;
}

A_Star::A_Star(int connecting_dist)
{
    startSet = false;
    finishSet = false;
    xTop = -4470; // value for the US5NH02M ENC
    yTop = -12439; // value for the US5NH02M ENC
    setStart_Grid(0,0);
    setFinish_Grid(0,0);
    depth_cutoff = 1; // Depths of more than 1 meter below MLLW are considered obstacles
    tide = 0;
    grid_size=5;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = 0.25;
}

A_Star::A_Star(double gridSize, double depthCutoff, double CostMultiplier, int connecting_dist)
{
    startSet = false;
    finishSet = false;
    xTop = 0;
    yTop = 0;
    setStart_Grid(0,0);
    setFinish_Grid(0,0);
    depth_cutoff = depthCutoff; // Depths of more than depthCutoff meters below MLLW are considered obstacles
    tide = 0;
    grid_size=gridSize;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = CostMultiplier;
}

A_Star::A_Star(double gridSize, double depthCutoff, int connecting_dist)
{
    startSet = false;
    finishSet = false;
    xTop = 0;
    yTop = 0;
    setStart_Grid(0,0);
    setFinish_Grid(0,0);
    depth_cutoff = depthCutoff; // Depths of more than depthCutoff meters below MLLW are considered obstacles
    tide = 0;
    grid_size=gridSize;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = 0.25;
}

A_Star::A_Star(int x1, int y1, int x2, int y2, double depthCutoff, int connecting_dist)
{
    xTop = -4470;
    yTop = -12439;
    startSet = false;
    finishSet = false;
    setStart_UTM(x1,y1);
    setFinish_UTM(x2,y2);
    depth_cutoff = depthCutoff; // Depths of more than this are considered obstacles (in cm)
    tide = 0;
    grid_size=5;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = 0.25;
}

A_Star::A_Star(int x1, int y1, int x2, int y2, double depthCutoff, double gridSize, double TopX, double TopY, int connecting_dist)
{
    xTop = TopX;
    yTop = TopY;
    startSet = false;
    finishSet = false;
    setStart_UTM(x1,y1);
    setFinish_UTM(x2,y2);
    depth_cutoff = depthCutoff; // Depths of more than this are considered obstacles (in cm)
    tide = 0;
    grid_size = gridSize;
    setGridXYBounds(0,0,0,0);
    NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
    setSubsetVars(false, 0, 0, 0, 0);
    costMultiplier = 0.25;
}

/*
Number of Neighboors one wants to investigate from each cell. A larger
   number of nodes means that the path can be alligned in more directions.

   Connecting_Distance=1-> Path can  be alligned along 8 different direction.
   Connecting_Distance=2-> Path can be alligned along 16 different direction.
   Connecting_Distance=3-> Path can be alligned along 32 different direction.
   Connecting_Distance=4-> Path can be alligned along 56 different direction.
   ETC......
*/
void A_Star::NeighborsMask(int Connecting_Dist)
{
  int twice = 2*Connecting_Dist;
  int Mid = Connecting_Dist;
  int r_size = 2*Connecting_Dist+1;

  // Remove the old values for dx and dy
  dx.clear();
  dy.clear();

  // Mask of the desired neighbors
  vector<vector<int>> new_neighbors (r_size, vector<int>(r_size,1));

  // Remove the positions that are the same directions
  for (int i=0; i<Connecting_Dist-1; i++)
    {
      new_neighbors[i][i]=0;
      new_neighbors[twice-i][i]=0;
      new_neighbors[i][twice-i]=0;
      new_neighbors[twice-i][twice-i]=0;
      new_neighbors[Mid][i]=0;
      new_neighbors[Mid][twice-i]=0;
      new_neighbors[i][Mid]=0;
      new_neighbors[twice-i][Mid]=0;
    }
  new_neighbors[Mid][Mid]=0;
        
  // Find the locations of the mask for the new neighbors
  for (int i=0; i<r_size; i++)
    {
      for (int j=0; j<r_size; j++)
	{
	  if (new_neighbors[i][j] == 1)
	    {
	      dy.push_back(i-Connecting_Dist);
	      dx.push_back(j-Connecting_Dist);
	    }
	}
    }

  // Number of directions that the algorithm will search
  num_directions = dx.size();
}

/*
Generate the path from finish to start by following the directions
in the direction map.
            
Input:
  direction_map - Map of how to traverse back to the start. Contains 
    the index of dx/dy.
        
Output:
  path - indices of dx/dy that relate to the planned path from the 
         start to the finish
*/
string A_Star::ReconstructPath(vector<vector<int> > direction_map)
{
  vector<int> path;
  bool first_time = true;
  int x,y,j;
  x = xFinish;
  y = yFinish;
 
  while (!((x == xStart) && (y == yStart)))
    {
      j = direction_map[x][y];
      if (first_time)
	{
	  path.push_back(getParent(j));
	  first_time=false;
	}   
      else
	path.insert(path.begin(), getParent(j));
      x += dx[j];
      y += dy[j];
    }
  return markRoute(path);
}


/* Check to see if the extended path runs through any obstacles. It also calculates the cost to
 * travel to the cell.
 *
 *   Inputs -
 *      i - index of the cell that we are trying to determine if it is valid
 *      wptX - current X location
 *      wptY - current Y location
 *      ave_depth - value of the average detpth of all cells that were traveled through
 *
 *   Output -
 *      Boolean describing if the extended path determined by the function is
 *          drives through an obstacle
 */
bool A_Star::extendedPathValid(int i, int wptX, int wptY, double &ave_depth)
{
    double x,y, m, b;
    int X,Y;
    int floor_x, ceil_x, floor_y, ceil_y;

    int dX = abs(dx[i]);
    int dY = abs(dy[i]);

    double cummulative_cost = 0;
    double intermidate = 0;

    int num_points = 5; // number of data points per cell
    int total_points = 0;
    double df_total_points = 0;

    if (max(dX,dY)==1)
    // If the node is a Moore Neighboor, check the cell on either side of desired cell
    {
        // If either cell next to the desired cell is an obstacle, the path is not valid
        if ((getMapValue(wptX, wptY+dy[i]) < depth_cutoff) || (getMapValue(wptX+dx[i], wptY) < depth_cutoff))
            return false; // Path is invalid)

        ave_depth = getMapValue(wptX+dx[i], wptY+dy[i]); // depth in grid cells
    }
    else
    // Otherwise check all cells in the path between the two cells
    {
        // calculate the slope and y-intersect of the line between the two points
        m = (1.0*(dy[i]))/(1.0*(dx[i]));
        b = wptY - wptX*m;

        // Segment the grid in the larger dimension
        if (dX < dY)
        {
            total_points = num_points*dY;
            df_total_points = total_points*1.0;

            // Cycle through all segmented grid nodes to check for path validity
            //  determine the total water depth traveled through
            for (int j=1; j<=total_points; j++)
            {
                intermidate = (1.0*dY)*(j*1.0)/(total_points*1.0);

                // Calculate the current y grid node
                if (signbit(dy[i]))// Returns true if negative
                    y = wptY-intermidate;
                else
                    y = wptY+intermidate;

                // Calculate the x gride node
                x = (y-b)/m;

                // Store the grid node's X,Y position as ints
                X= static_cast<int>(round(x));
                Y= static_cast<int>(round(y));

                cummulative_cost += getMapValue(X,Y);

                // If either the grid node or the next closest node on the line in the x direction
                //  is an obstacle, the path is not valid
                floor_x = int(floor(x));
                ceil_x = int(ceil(x));
                if ((getMapValue(floor_x, Y) < depth_cutoff) || (getMapValue(ceil_x, Y) < depth_cutoff))
                {
                    return false; // Path is invalid
                }
            }
        }
        else
        {
            total_points = num_points*dX;
            df_total_points = total_points*1.0;

            // Cycle through all segmented grid nodes to check for path validity
            //  determine the total water depth traveled through
            for (int j=1; j<total_points; j++)
            {
                intermidate = (1.0*dX)*(j*1.0)/(df_total_points);
                // Calculate the x gride node
                if (signbit(dx[i]))// Returns true if negative
                    x = wptX-intermidate;
                else
                    x = wptX+intermidate;

                // Calculate the y gride node
                y= m*x+b;

                // Store the grid node's X,Y position as ints
                X= static_cast<int>(round(x));
                Y= static_cast<int>(round(y));

                cummulative_cost += getMapValue(X,Y);

                // If either the grid node or the next closest node on the line in the x direction
                //  is an obstacle, the path is not valid
                floor_y = int(floor(y));
                ceil_y = int(ceil(y));
                if ((getMapValue(X, floor_y) < depth_cutoff) || (getMapValue(X, ceil_y) < depth_cutoff))
                {
                    return false; // Path is invalid
                }
            }
        }
        ave_depth =cummulative_cost/(df_total_points); // average depth in grid cells
    }

    return true; // Path is valid so return true
}

/* Check to see if the extended path runs through any obstacles. It also calculates the cost to
 * travel to the cell. This method computes the aveage by using a linear approximation of the
 * two cells that the line passes through.
 *
 *   Inputs -
 *      i - index of the cell that we are trying to determine if it is valid
 *      wptX - current X location
 *      wptY - current Y location
 *      ave_depth - value of the average detpth of all cells that were traveled through
 *
 *   Output -
 *      Boolean describing if the extended path determined by the function is
 *          drives through an obstacle
 */
bool A_Star::extendedPathValid_aveDepth(int i, int wptX, int wptY, double &ave_depth)
{
    double intermediate, alpha, beta;

    int x1,y1,x2,y2;
    int accumDepth=0;
    int absX = abs(dx[i]);
    int absY = abs(dy[i]);
    int farestPoint_in_XY = max(absX, absY);
    int signX = posORneg(dx[i]);
    int signY = posORneg(dy[i]);

    // Check to see if it is not a Moore Neighboor. If it is then return the depth of the desired cell.
    if (farestPoint_in_XY==1)
        accumDepth = getMapValue(wptY+dx[i], wptY+dy[i]);
    // Otherwise calculate the average depth of the cells that a line would pass between the waypoints.
    else
    {
        for (int index=1; index<=farestPoint_in_XY; index++)
        {
            if (absY<absX)
            {
                intermediate = (1.0*dy[i])/(1.0*dx[i])*(1.0*index);
                x1 = index*signX;
                x2 = x1;
                // Make sure that you use the correct sign for 'Y's if dx and dy have the same sign
                if (signX==signY)
                    y1 = static_cast<int>(intermediate)*signY;
                else
                    y1 = static_cast<int>(intermediate);
                y2 = y1+signY;
            }
            else
            {
                intermediate = (1.0*dx[i])/(1.0*dy[i])*(1.0*index);
                // Make sure that you use the correct sign for the 'X's if dx and dy have the same sign
                if (signX==signY)
                    x1 = static_cast<int>(intermediate)*signX;
                else
                    x1 = static_cast<int>(intermediate);
                x2 = x1+signX;
                y1 = index*signY;
                y2 = y1;
            }
            beta = abs(fmod(intermediate, 1.0));
            alpha = 1-beta;

            // Make sure than neither of the two cells that you are passing through have a
            if ((getMapValue(wptX+x1, wptY+y1) < depth_cutoff)||getMapValue(wptX+x2,wptY+y2) < depth_cutoff)
                return false;

            // If you are not at the last cell, then average the two cells and store that value. Otherwise store the value for the final cell.
            if (index!=farestPoint_in_XY)
                accumDepth += alpha*getMapValue(wptX+x1, wptY+y1) + beta*getMapValue(wptX+x2,wptY+y2);
            else
                accumDepth += alpha*getMapValue(wptX+x1, wptY+y1);
        }
    }
    ave_depth = accumDepth/farestPoint_in_XY;
    return true;
}

bool A_Star::runA_Star(bool yes_print, bool MOOS_WPT, bool L84_WPT, bool depthBasedAStar, string filename, double LatOrigin, double LongOrigin)
{
    bool found_path;

    clock_t start;
    char command = 'Y';
    double total_time;
    cout << "Running A*\n";

    // Print a warning if the depth cutoff is lower than NOAAs
    if (depth_cutoff<400)
    {
        cout << "WARNING: Depth cutoff is set lower than the NOAA's mapping threshold of 4 meters(" << depth_cutoff << " meters). Do you want to continue? (Y/N)" << endl;
        //cin >>command;

        // Check to see if the user wants to continue with the threshold lower depth cutoff
        if (toupper(command) != 'Y')
            exit(1);
    }

    checkStartFinish();
    if ((getStartValid()) && (getFinishValid()))
    {
        cout << "Valid Start/Finish" << endl;
        start = clock();
        Route = AStar_Search(depthBasedAStar);
        total_time = (clock() - start)*0.001;
    }
    else
        cout << "Invalid Start/Finish" << endl;

    found_path = !(Route.empty());
    // Only print the map if a route was found
    if (found_path)
    {
        cout << Route << endl;
        if (yes_print)
            printMap(Route, total_time);
        if (MOOS_WPT)
            buildMOOSFile(filename, Route);
        if (L84_WPT)
        {
            L84 newfile = L84(filename, Route, LatOrigin, LongOrigin);
            newfile.writeL84();
        }
    }
    else
        cout << "No path found!" << endl;
    return found_path;
}

void A_Star::buildMOOSFile(string filename, string WPTs)
{
  ofstream MOOS;
  MOOS.open (filename+".bhv");
  // Build the bhv file
  string BHV_file;
  BHV_file = "Behavior = BHV_Waypoint\n{\n";
  BHV_file += "  name      = waypt_survey\n";
  BHV_file += "  pwt       = 100\n";
  BHV_file += "  condition = RETURN = false\n";
  BHV_file += "  condition = DEPLOY = true\n";
  BHV_file += "  endflag   = RETURN = true\n";
  BHV_file += "  idleflag  = WPTING = idle\n";
  BHV_file += "  runflag   = WPTING = running\n";
  BHV_file += "  endflag   = WPTING = end\n";
  BHV_file += "  inactiveflag = WPTING = inactive\n";
  BHV_file += "  UPDATES   = WPT_UPDATE\n";
  BHV_file += "  perpetual = true\n";
  BHV_file += "  lead = 8\n";
  BHV_file += "  lead_damper = 1\n";
  BHV_file += "  speed ="+to_string(getDesiredSpeed()) + "\n";
  BHV_file += "  capture_radius = 5.0\n";
  BHV_file += "  slip_radius = 5.0\n";
  BHV_file += "  repeat = 1\n";
  MOOS << BHV_file;
  
  vector<string> vect_WPTs;
  char delimiter = ',';
  vect_WPTs = split(WPTs, delimiter);
  MOOS << "  points = pts={";
  for (int i=0; i<vect_WPTs.size(); i++)
    {
      MOOS << vect_WPTs[i];
      if (i%2 == 0)
	MOOS << ",";
      else
	MOOS << ":";
    }
  MOOS << "}\n";
  MOOS << "}\n";
  MOOS.close();
}

// This function runs A*. It outputs the generated path as a string to stdout
string A_Star::AStar_Search(bool depthBasedAStar)
{
    //FILE *myfile;
    //myfile = fopen("Explored.txt", "w");

    int x_size, y_size;
    if (mapSubsetFlag)
    {
        x_size = subsetXmax-subsetXmin;
        y_size = subsetYmax-subsetYmin;
    }
    else
    {
        x_size = n;
        y_size = m;
    }

    vector<vector<int>> direction_map (x_size, vector<int>(y_size,0));
    vector<vector<double>> open_nodes_map (x_size, vector<double>(y_size,0));
    vector<vector<int>> closed_nodes_map (x_size, vector<int>(y_size,0));
    int cntr =0;
    double depth_cost=0;

    int x,y, new_x, new_y;
    Node n0, child;
    priority_queue<Node, vector<Node>, less<Node>> frontier;

    // Start node
    n0 = Node(xStart, yStart, getMapValue(xStart, yStart), 0,grid_size, costMultiplier);
    n0.updatePriority(xFinish,yFinish);
    n0.setShipMeta(ShipMeta);
    frontier.push(n0);

    // Mark start node on map
    open_nodes_map[xStart][yStart] = n0.getPriority();
    cout << "depth cutoff " << depth_cutoff << endl;
    //double c,p;
    // Run A*
    while (!frontier.empty())
    {
      // A* explores from the highest priority node in the frontier
      n0 = frontier.top();
      //n0.updatePriority(xFinish,yFinish);
      frontier.pop(); // Remove the node from the frontier
      x = n0.getX();
      y = n0.getY();

      // If it is a new node, explore the node's neighbors
      if (closed_nodes_map[x][y] != 1)
      {
        cntr++;
        open_nodes_map[x][y] = 0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y] = 1;
        // Quit searching when you reach the goal state
        if ((x == xFinish) && (y == yFinish))
        {
          // Generate the path from finish to start by following the
          //  directions.
          //fclose(myfile);
          return ReconstructPath(direction_map);
        }
        for (int i = 0; i<num_directions; i++)
          {
            new_x = x+dx[i];
            new_y = y+dy[i];
            // Place the node in the frontier if the neighbor is within the
            //  map dimensions, not an obstacle, and not closed.
            if ((new_x >= 0)&&(new_x < x_size)&&(new_y >= 0)&&(new_y < y_size)&&
                (getMapValue(new_x, new_y) > depth_cutoff)&&
                (closed_nodes_map[new_x][new_y]!=1))
              {
                // Check to see if the extended path goes through obstacles
                if (extendedPathValid(i, x, y, depth_cost))
                  {
                    // Build the new node
                    child = Node(new_x, new_y, depth_cost,
                                 n0.getCost(), grid_size, costMultiplier);
                    child.setShipMeta(ShipMeta);
                    //child.calcCost(dx[i], dy[i], n0.getDepth(), getDesiredSpeed());

                    // if depthBasedAStar is set to true then include depth into the cost of the node
                    //  otherwise run normal A*
                    if (depthBasedAStar)
                        child.calcCost_depth(dx[i], dy[i]);
                    else
                        child.calcCost(dx[i], dy[i]);

                    //c = child.getCost();
                    child.updatePriority(xFinish, yFinish);
                    //p = child.getPriority1();

                    //fprintf(myfile, "%d\t%d\t%d\t%d\t%0.3f\t%0.3f\n", cntr,i,new_x, new_y, c, p);
                    // If the child node is not in the open list or the
                    //  current priority is better than the stored one,
                    //  add it to the frontier
                    if ((open_nodes_map[new_x][new_y] == 0) ||
                        (open_nodes_map[new_x][new_y] > child.getPriority()))
                      {
                        open_nodes_map[new_x][new_y] = child.getPriority();
                        // If the node has been explored, the old value will
                        //  still be in the frontier but it will have a lower
                        //  priority (aka higher cost) and should not be
                        //  popped. To deal with this obscure case the popped
                        //  value from the frontier will be checked to see if
                        //  it has been previously opened and if it has then
                        //  it will be skipped.
                        frontier.push(child);
                        // Mark the direction of the parent node
                        direction_map[new_x][new_y] = getParent(i);
                      }
                  }
              }
          }

        }
    }
    cout << "No path found." << endl;
    //fclose(myfile);
    return "";

}

double A_Star::calcDepthCost(int wptX,int wptY, int i)
{
    double m,b, x,y;
    int X,Y;

    double cummulative_cost = 0;
    int num_points = 5;
    int total_pnts = 0;

    int dX = abs(dx[i]);
    int dY = abs(dy[i]);

    // Only run the interpolation if the cell is not a Moore Neighbor
    if (max(dX, dY)>1)
    {
        m = (1.0*(dy[i]))/(1.0*(dx[i]));
        b = wptY - wptX*m;

        if (dX < dY)
        {
            total_pnts = num_points*dY;
            for (int j=1; j<=total_pnts; j++)
            {
                if (signbit(dy[i]))// Returns true if negative
                    y = wptY-j/num_points;
                else
                    y = wptY+j/num_points;

                x = (1.0*(y-b))/m;
                X= static_cast<int>(x);
                Y=static_cast<int>(y);
                cummulative_cost += getMapValue(X,Y);
            }
        }
        else
        {
            total_pnts = num_points*dX;
            for (int j=1; j<total_pnts; j++)
            {
                if (signbit(dx[i]))// Returns true if negative
                    x = wptX-j/num_points;
                else
                    x = wptX+j/num_points;
                y= m*x+b;
                X= static_cast<int>(x);
                Y=static_cast<int>(y);
                cummulative_cost += getMapValue(X,Y);
            }
        }

        return cummulative_cost/(total_pnts); // average depth in grid cells
    }
    else
        return getMapValue(wptX+dx[i], wptY+dy[i]); // depth in grid cells


}

// This function marks the route on the map
string A_Star::markRoute(vector<int> route)
{
    int x, y, direction, route_len, x_size, y_size;
    double localX = 0;
    double localY = 0;
    string WPT;
    route_len = route.size();
    if (mapSubsetFlag)
    {
        x_size = subsetXmax-subsetXmin;
        y_size = subsetYmax-subsetYmin;
    }
    else
    {
        x_size = n;
        y_size = m;
    }

    // Initialize the print map 2d vector
    Map2print.clear();
    Map2print.resize(x_size, vector<int>(y_size, 0));

    // Simplify the map for printing
    for (int x_index=0; x_index<n; x_index++)
    {
        for (int y_index=0; y_index<m; y_index++)
        {
            if (getMapValue(x_index,y_index)>depth_cutoff)
                Map2print[x_index][y_index]=0;
            else
                Map2print[x_index][y_index]=1;
        }
    }

    // New waypoints are when placed the heading changes
    if (route_len > 0)
    {
        x = xStart;
        y = yStart;

        grid2xy(x, y, localX, localY);

        cout << "N: " << x_size << ", M:" << y_size << endl;

        // Mark Start
        Map2print[x][y] = 2;
        WPT=to_string(localX)+","+to_string(localY)+",";
        cout << "["+to_string(x+x_min+1) <<","+to_string(FullMap.size()-(y+y_min))+"; ";
        for (int i=0; i<route_len-1; i++)
        {
            direction = route[i];
            x += dx[direction];
            y += dy[direction];
            grid2xy(x, y, localX, localY);
            if (i<route_len-1)
            {
                // If you are continueing with on the same path,
                //  don't store the new waypoint
                if (route[i]==route[i+1])
                    Map2print[x][y] = 3;
                else
                {
                    Map2print[x][y] = 4;
                    WPT+= to_string(localX)+","+to_string(localY)+",";
                    cout << to_string(x+x_min+1) <<","+to_string(FullMap.size()-(y+y_min))+"; ";
                }
            }
        }
        // Mark finish
        direction = route[route_len-1];
        x += dx[direction];
        y += dy[direction];
        grid2xy(x, y, localX, localY);
        Map2print[x][y] = 5;
        WPT += to_string(localX)+","+to_string(localY);
        cout << to_string(x+x_min+1) <<","+to_string(y+y_min+1)+"];" << endl;
    }
    return WPT;
}

/*
This function prints out the map to std output where:
        '.' = space
        '0' = Obstacle
        'S' = Start
        'F' = Finish
        'X' = New Waypoint
        '@' = route (no new waypoint)
*/
void A_Star::printMap(string WPTs, double total_time)
{
  int xy, x_size, y_size;
  if (mapSubsetFlag)
  {
      x_size = subsetXmax-subsetXmin;
      y_size = subsetYmax-subsetYmin;
  }
  else
  {
      x_size = n;
      y_size = m;
  }

  // Print Meta Data
  cout << "Map Size (X,Y): " <<  x_size << ", " << y_size << endl;
  cout << "Start: " << xStart << ", " << yStart << endl;
  cout << "Finish: " << xFinish << ", "  << yFinish << endl;
  cout << "Time to generate the route: " << total_time<< " (ms)" << endl;
  cout << "Number of directions searched: " << getNumDir() << endl;
  cout << WPTs << endl;
        
  // Display the map with the route
  cout << "Map: \n";
  for (int y=y_size-1; y>=0; y--)
    {
      for (int x=0; x<x_size; x++)
	{
          xy = Map2print[x][y];
	  if (xy == 0)
	    cout << ". "; // space
	  else if (xy == 1)
	    cout << "O "; // obstacle
	  else if (xy == 2)
	    cout << "S "; // start
	  else if (xy == 3)
	    cout << "@ "; // route
	  else if (xy == 4)
	    cout << "X "; // Waypoint
	  else if (xy == 5)
	    cout << "F "; // finish
          else
              cout << xy << " ";
	}
      cout << endl;
    }
}

// Build the default map
void A_Star::build_default_map(int N, int M, int config)
{
    // Now build the map
    FullMap.clear();
    FullMap.resize(N, vector<double>(M,110));
    vector<vector<int>> start_finish;

    // Place an cross obstacle in the middle
    for (int x = M/8; x<7*M/8; x++)
        FullMap[N/2][x] = 0;
    for (int y = N/8; y<7*N/8; y++)
        FullMap[y][M/2] = 0;

    // Different Start/Finish Configurations
    start_finish.push_back({0, 0, N - 1, M - 1});
    start_finish.push_back({0, M - 1, N - 1, 0});
    start_finish.push_back({N / 2 - 1, M / 2 - 1, N / 2 + 1, M / 2 + 1});
    start_finish.push_back({N / 2 - 1, M / 2 + 1, N / 2 + 1, M / 2 - 1});
    start_finish.push_back({N / 2 - 1, 0, N / 2 + 1, M - 1});
    start_finish.push_back({N / 2 + 1, M - 1, N / 2 - 1, 0});
    start_finish.push_back({0, M / 2 - 1, N - 1, M / 2 + 1});
    start_finish.push_back({N - 1, M / 2 + 1, 0, M / 2 - 1});

    // Pick a configuration
    config= config%8; // Make sure the value is between 0 and 7
    xStart = start_finish[config][0];
    yStart = start_finish[config][1];
    xFinish = start_finish[config][2];
    yFinish = start_finish[config][3];

    // Set the derived map
    setMapMeta();
    // Sets the grid and local coordinate system as the same system
    setConversionMeta(1, 0, 0);
}

// Use your own map from a csv file that contains the grid and define the
//  coordinate dimensions of a subset of the map to be used for A*.
void A_Star::buildMapFromCSV(string filename, int xMin, int xMax, int yMin, int yMax)
{
    strtk::token_grid::options option;
    option.column_delimiters = ",";

    // Read in the file
    strtk::token_grid grid(filename, option);

    // Initialize the 2D vector
    strtk::token_grid::row_type row = grid.row(0);
    FullMap.clear();
    FullMap.resize(grid.row_count()-1, vector<double>(row.size()-1,0));

    for(size_t i = 1; i < grid.row_count(); ++i)
    {
        row = grid.row(i);
        for(size_t j = 1; j < row.size(); ++j)
        {
            FullMap[i-1][j-1] = row.get<double>(j);
        }
    }
    cout << "Map loaded" << endl;

    // Subset the map
    //subsetMap(xMin, xMax, yMin, yMax);
    setSubsetVars(true, xMin, yMin, xMax, yMax);
}

// Use your own map from a csv file that contains the grid
void A_Star::buildMapFromCSV(string filename)
{
    strtk::token_grid::options option;
    option.column_delimiters = ",";

    // Read in the file
    strtk::token_grid grid(filename, option);

    // Initialize the 2D vector
    strtk::token_grid::row_type row = grid.row(0);
    FullMap.clear();
    FullMap.resize(grid.row_count()-1, vector<double>(row.size()-1));

    for(size_t i = 1; i < grid.row_count(); ++i)
    {
        row = grid.row(i);
        for(size_t j = 1; j < row.size(); ++j)
        {
            FullMap[i-1][j-1] = row.get<int>(j);
        }
    }
    cout << "Map loaded" << endl;

    // Set the new map
    setMapMeta();
}

/*
 * // Define the coordinate dimensions of a subset of the map to be used for A*.
void A_Star::subsetMap(int xmin, int xmax, int ymin, int ymax)
{
    // Set the bounds of the map (in grid coordinate system)
    setGridXYBounds(xmin, xmax, ymin, ymax);

    n = y_max - y_min;
    m = x_max - x_min;

    // Fill the map
    Map.clear();
    Map.resize(n, vector<double> (m));
    for (int y=0; y<n; ++y)
    {
        for (int x=0; x<m; ++x)
        {
            Map[y][x] = FullMap[y+y_min][x+x_min];
        }
    }

    xStart-= xmin;
    yStart-= ymin;
    xFinish-= xmin;
    yFinish-= ymin;

}
*/

// This function takes the two given indices for the of the map and returns the value in the map. If there has been a defined subset for
//  the map, then the function returns the correct value as long as it is inside the subset. If it is not then the undefined value (-10)
//  is returned and error message is printed on the std out.
double A_Star::getMapValue(int xIndex, int yIndex)
{
    //cout << mapSubsetFlag << " " << xIndex << " " << yIndex << endl;
    // If there is no subset the value at the inputted index if it is inside the Map
    if (!mapSubsetFlag)
    {
        if ((xIndex>x_min)&&(yIndex>y_min)&&(xIndex<x_max)&&(yIndex<y_max))
        {
            //cout << xIndex << ", " << yIndex << " --> " << FullMap[xIndex][yIndex] << endl;
            return FullMap[xIndex][yIndex];
        }

        // If the indices are outside of the map, then an undefined value (-10) is returned and error message
        //  is printed on the std out.
        else
        {
            if ((xIndex>x_max)||(xIndex<x_min))
                cout << "ERROR: The inputted X index (" <<xIndex<<") was out of spec. Please make sure to input valid indices [" << x_min << " " << x_max <<"]. (Now returning -10)" << endl;
            if ((yIndex>y_max)||(yIndex<y_min))
                cout << "ERROR: The inputted Y index (" <<yIndex<<") was out of spec. Please make sure to input valid indices [" << y_min << " " << y_max <<"]. (Now returning -10)" << endl;

            return -10;
        }
    }
    else
    {
        if ((xIndex>0)&&(yIndex>0)&&(xIndex+subsetXmin<subsetXmax)&&(yIndex+subsetYmin<subsetYmax))
        {
            return FullMap[xIndex+subsetXmin][yIndex+subsetYmin];
        }

        // If the indices are outside of the map subset, then an undefined value (-10) is returned and error message
        //  is printed on the std out.
        else
        {
            if ((xIndex>0)||(xIndex+subsetXmin<subsetXmax))
                cout << "ERROR: The inputted X index was out of spec. Please make sure to input valid subset indices. (Now returning -10)" << endl;
            if ((yIndex>0)||(yIndex+subsetYmin<subsetYmax))
                cout << "ERROR: The inputted Y index was out of spec. Please make sure to input valid subset indices. (Now returning -10)" << endl;

            return -10;
        }
    }
}

void A_Star::setSubsetVars(bool flag, int XMin, int YMin, int XMax, int YMax)
{
    mapSubsetFlag = flag;
    subsetXmin = XMin;
    subsetXmax = XMax;
    subsetYmin = YMin;
    subsetYmax = YMax;
    resetStartFinish_subset();
}

void A_Star::resetStartFinish_subset()
{
    // If the subset has been set, adjust the start and finish grid cell to the subset coordinate system
    if (mapSubsetFlag)
    {
        if (startSet)
        {
            cout << "re-inializing start to subset." << endl;
            xStart -= subsetXmin;
            yStart -= subsetYmin;
            // Reset the flag so this doesnt occur more than once
            startSet = false;
        }
        if (finishSet)
        {
            cout << "re-inializing finish position to subset." << endl;
            xFinish -= subsetXmin;
            yFinish -= subsetYmin;
            // Reset the flag so this doesnt occur more than once
            finishSet = false;
        }
    }
}

void A_Star::setMapFromTiff(string tiff_filename)
{
    int nXSize, nYSize;
    double x,y;
    GDALDataset  *poDataset;
    GDALRasterBand *poRasterBand;
    vector<double> adfGeoTransform, RasterData;

    // Preallocate the size of the affine transformation coefficients.
    adfGeoTransform.resize(6);

    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen( tiff_filename.c_str(), GA_ReadOnly );
    poDataset->GetGeoTransform(adfGeoTransform.data());

    poRasterBand = poDataset -> GetRasterBand(1);

    nXSize = poRasterBand->GetXSize(); // width
    nYSize = poRasterBand->GetYSize(); // height

    // Resize the vector to fit the raster dataset
    RasterData.resize(nXSize*nYSize);

    // Read the raster into a 1D row-major vector where it is organized in left to right,top to bottom pixel order
    if( poRasterBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, RasterData.data(), nXSize, nYSize, GDT_Float64, 0, 0 ) != CE_None )
    {
        printf( "Failed to access raster data.\n" );
        GDALClose(poDataset);
        exit( 1 );
    }

    GDALClose(poDataset);

    /* The GeoTransforms gives you how to go from pixel space (P,L) to projected coordinates (Xp,Yp)
     *  [0] - Upper Left X (ulx)
     *  [1] - X Resolution (xres)
     *  [2] - X Skew (xskew)
     *  [3] - Upper Left Y (uly)
     *  [4] - Y Skew (yskew)
     *  [5] - Y Resolution (yres)
     *  Xp = ulx + P*xres + L*xskew;
     *  Yp = uly + P*yskew + L*yres;
     */
    xTop = adfGeoTransform[0];
    x_min = static_cast<int>(adfGeoTransform[2]);
    y_min = static_cast<int>(adfGeoTransform[4]);

    // Since we need to transpose the coordinates, we need to store the TIFF's lower y coordinate not upper y coordinate
    yTop = adfGeoTransform[3]+(1.0*nYSize)*adfGeoTransform[5];

    // Now flip the Y axis
    FullMap.clear();
    FullMap.resize(nXSize, vector<double> (nYSize,0));
    // RasterData is stored left to right top to bottom
    for (int rasterIndex=0; rasterIndex<RasterData.size(); rasterIndex++)
    {
        x = rasterIndex%nXSize; // Store left to right
        y = (nYSize-1)-(rasterIndex-x)/nXSize; // Convert from top to bottom -> bottom to top (i.e. Transpose)
        FullMap[x][y] = RasterData[rasterIndex];
    }

    n=nXSize;
    m=nYSize;
    x_max = n;
    y_max = m;

//    for (int x1=0; x1<nXSize; x1++)
//    {
//        for (int y1= 0; y<nYSize; y1++)
//        {
//            cout << x1 << ", " << y1 << endl;//S << " --> " << getMapValue(x1,y1) << endl;
//            //double temp = getMapValue(x,y);
//        }
//    }
}

void A_Star::row_major_to_2D(int index, double &gridX, double &gridY, int numCols)
{
    gridX = index%numCols;
    gridY = (index-gridX)/numCols;
}

void A_Star::checkStart()
{
    if (getMapValue(xStart, yStart) < depth_cutoff)
    {
        cout << "Invalid Start position" << endl;
        valid_start = false;
    }
    else
        valid_start = true;
    //cout << xStart << ", " << yStart << " --> " << getMapValue(xStart, yStart) << " " << getMapValue(yStart, xStart) << endl;

}

void A_Star::checkFinish()
{
    if (getMapValue(xFinish, yFinish) < depth_cutoff)
    {
        cout << "Invalid finish position" << endl;
        valid_finish = false;
    }
    else
        valid_finish = true;
}

void Vessel_Dimensions::getVesselMeta()
{
    double length = 0;
    double width = 0;
    double draft = 0;
    int mobility=0;
    double turn_radius = 0;
    string know_TR;

    cout << "What is the vessel length? (in meters) ";
    cin >> length;

    cout << endl << "What is the vessel width? (in meters) ";
    cin >> width;

    cout << endl << "What is the vessel draft? (in meters) ";
    cin >> draft;

    cout << endl << "Do you know the vessel turn radius? (Y/N) ";
    cin >> know_TR;

    if ((know_TR == "Y") || (know_TR == "YES")||(know_TR == "y")||(know_TR == "yes"))
    {
        cout << endl << "What is the vessel turn radius? (in meters) ";
        cin >> turn_radius;
    }
    else
    {
        cout << endl << "How would you rate the mobility on a scale from 1 to 5? ";
        cin >> mobility;
        turn_radius = (6-mobility)*length;
    }
    set_ship_meta(turn_radius, length, width, draft);
}

void A_Star::setDesiredSpeed()
{
    cout << "What is the desired speed for this waypoint? (in knots)";
    cin >> desired_speed;

    desired_speed *= 0.514444; // Knots to m/s
}

double A_Star::calcMinDepth()
{
    double depth_thresh;
    double draft =ShipMeta.getDraft();
    if (draft > .33)
        depth_thresh = draft*3;
    else
        depth_thresh = 1;
    return depth_thresh;
}

// How we are sorting the frontier priority queue 
bool operator<(const Node& lhs, const Node& rhs)
{
  return lhs.getPriority() > rhs.getPriority();
}
