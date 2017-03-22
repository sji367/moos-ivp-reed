/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH CCOM                                        */
/*    FILE: a_star.cpp                                      */
/*    DATE: 3/21/17                                         */
/************************************************************/

#include "astar.h"

A_Star::A_Star()
{
  xStart = 0;
  yStart = 0;
  xFinish = 0;
  yFinish = 0;
  NeighborsMask(2); // Set dx, dy, and num_directions
}

A_Star::A_Star(int connecting_dist)
{
  xStart = 0;
  yStart = 0;
  xFinish = 0;
  yFinish = 0;
  NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
}

A_Star::A_Star(int x1, int y1, int x2, int y2, int connecting_dist)
{
  xStart = x1;
  yStart = y1;
  xFinish = x2;
  yFinish = y2;
  NeighborsMask(connecting_dist); // Set dx, dy, and num_directions
}

void A_Star::NeighborsMask(int Connecting_Dist)
{
  int twice = 2*Connecting_Dist;
  int Mid = Connecting_Dist;
  int r_size = 2*Connecting_Dist+1;
  
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
vector<int> A_Star::ReconstructPath(vector<vector<int>> direction_map)
{
  vector<int> path;
  bool first_time = true;
  int x,y,j;
  x = xFinish;
  y = yFinish;
 
  while (!((x == xStart) && (y == yStart)))
    {
      j = direction_map[y][x];
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
  return path;
}

// Check to see if the extended path runs through any obstacles
bool A_Star::extendedPathValid(int i, int x, int y)
{
  bool Flag = true;
  double temp = 1;
  int JumpCells, YPOS, XPOS;
  
  if ((abs(dx[i])>1)||(abs(dy[i])>1))
    {
      // Need to check that the path does not pass an object
      JumpCells=2*max(abs(dx[i]),abs(dy[i]))-1;
      for (int K=1; K<JumpCells; K++)
	{
	  YPOS=int(round(K*temp*dy[i]/JumpCells));
	  XPOS=int(round(K*temp*dx[i]/JumpCells));
	  // This is actual check to see if the intermediate grid cells
	  //  intersect an obstacle
	  if (Map[y+YPOS][x+XPOS]==1)
	    Flag=false;
	}
    }
  return Flag;
}

void A_Star::runA_Star(bool meta)
{
  vector<int> route;
  clock_t start = clock();
  route = AStar_Search();
  clock_t total_time = clock() - start;
  printMap(route, total_time, meta);
}

// This function runs A*. It outputs the generated path as an string
//    where each movement is stored as a number corrisponding to the
//    index of the neighbor mask.
vector<int> A_Star::AStar_Search()
{
  vector<vector<int>> direction_map (n, vector<int>(m,0));
  vector<vector<int>> open_nodes_map (n, vector<int>(m,0));
  vector<vector<int>> closed_nodes_map (n, vector<int>(m,0));
  
  vector<int> path, no_path;
  int x,y, new_x, new_y;
  Node n0, child;
  
  priority_queue<Node, vector<Node>, less<Node>> frontier;
  //int pqi =0; // priority queue index
  
  n0 = Node(xStart, yStart, 0,0); // Start node
  n0.updatePriority(xFinish,yFinish);
  frontier.push(n0);
  open_nodes_map[yStart][xStart] = n0.getPriority(); // Mark start node on map
  while (!frontier.empty())
    {
      n0 = frontier.top();
      n0.updatePriority(xFinish,yFinish);
      frontier.pop(); // Remove the node from the frontier
      x = n0.getX();
      y = n0.getY();
      //cout << "Node: " << x << ", " << y  << endl;
      if (closed_nodes_map[y][x] != 1){
	open_nodes_map[y][x] = 0;
	// mark it on the closed nodes map
	closed_nodes_map[y][x] = 1;
	// Quit searching when you reach the goal state
	if ((x == xFinish) && (y == yFinish))
	  // Generate the path from finish to start by following the 
	  //  directions.
	  return ReconstructPath(direction_map);
	for (int i = 0; i<num_directions; i++)
	  {
	    new_x = x+dx[i];
	    new_y = y+dy[i];
	    if ((new_x >= 0)&&(new_x < n)&&(new_y >= 0)&&(new_y < m)&&
		(Map[new_y][new_x] != 1)&&(closed_nodes_map[new_y][new_x]!=1))
	      {
		if (extendedPathValid(i, x, y))
		  {
		    child = Node(new_x, new_y, n0.getCost(), n0.getPriority());
		    child.calcCost(dx[i], dy[i]);
		    child.updatePriority(xFinish, yFinish);
		    //cout << " New Pos " << i <<": " << new_x << ", " << new_y << ", P: " << child.getPriority() << endl;
		    // If the child node is not in the open list, add it to
		    //  the list
		    if ((open_nodes_map[new_y][new_x] == 0) ||
			(open_nodes_map[new_y][new_x] > child.getPriority()))
		      {
			open_nodes_map[new_y][new_x] = child.getPriority();
			// If the node has been explored, the old value will
			//  still be in the frontier but it will have a lower 
			//  priority (aka higher cost) and should not be  
			//  popped. To deal with this obscure case the popped 
			//  value from the frontier will be checked to see if 
			//  it has been previously opened and if it has then 
			//  it will be skipped.
			frontier.push(child);
			// Mark the direction of the parent node 
			direction_map[new_y][new_x] = getParent(i);
		      }
		    /*
		      else if (open_nodes_map[new_y][new_x] > child.getPriority())
		      {
		      // Update the priority info
		      open_nodes_map[new_y][new_x] = child.getPriority();
		      // Update the parent direction info
		      direction_map[new_y][new_x] = getParent(i);
		      
		      // Replace the node by emptying one pq to the other
		      //  one except the node to be replaced will be ignored
		      //  and the new node will be pushed in instead
		      while ((frontier[pqi].top().getX() != new_x) &&
		      (frontier[pqi].top().gety() != new_y))
		      {
		      frontier[1-pqi].push(pq[pqi][0]);
		      frontier[pqi].pop();
		      }
		      frontier[pqi].pop(); // remove the wanted node
		      // Empty the larger size priority queue to the
		      //  smaller one
		      if (frontier[pqi].size() > frontier[1-pqi].size())
		      pqi = 1-pqi;
		      while (!frontier[pqi].empty())
		      {
		      frontier[1-pqi].push(pq[pqi].top());
		      frontier[pqi].pop(pq[pqi]);
		      }
		      pqi = 1-pqi;
		      frontier[pqi].push(child); // add the better node instead
		    */
		  }
	      }
	  }
	  
      }
    }
  cout << "No path found." << endl;
  return no_path;

}

// This function marks the route on the map
string A_Star::markRoute(vector<int> route)
{
  int x, y, direction, route_len;
  string WPT;
  route_len = route.size();
  if (route_len > 0)
    {
      x = xStart;
      y = yStart;
      Map[y][x] = 2;
      WPT="("+to_string(y)+", "+to_string(x)+")";
      cout << "Route: ";
      for (int i=0; i<route_len; i++)
	{
	  cout << route[i] << " ";
	  direction = route[i];
	  x += dx[direction];
	  y += dy[direction];
	  if (i<route_len-1)
	    {
	      // If you are continueing with on the same path
	      //  don't store the new waypoint
	      if (route[i]==route[i+1])
		Map[y][x] = 3;
	      else
		{
		  Map[y][x] = 4;
		  WPT+=", ("+to_string(y)+", "+to_string(x)+")";
		}
	    }
	}
      cout << endl;
      Map[y][x] = 5;
      WPT += ",("+to_string(y)+", "+to_string(x)+")";
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
void A_Star::printMap(vector<int> route, int total_time, bool print_meta)
{
  int xy;
  string WPTs;
  if (route.size()>0)
    WPTs = markRoute(route);
  if (print_meta)
    {
      cout << "Map Size (X,Y): " <<  m << ", " << n << endl;
      cout << "Start: " << xStart << ", " << yStart << endl;
      cout << "Finish: " << xFinish << ", "  << yFinish << endl;
      cout << "Time to generate the route: " << total_time<< " (ms)" << endl;
      cout << WPTs << endl;
    }
        
  // display the map with the route
  cout << "Map: \n";
  for (int y=0; y<n; y++)
    {
      for (int x=0; x<m; x++)
	{
	  xy = Map[y][x];
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
	}
      cout << endl;
    }
}

// Build the default map
void A_Star::default_map(int N, int M, int config)
{
  // Set size of map
  n=N;
  m=M;

  // Now build the map
  vector<vector<int>> MAP (N, vector<int>(M,0));
  vector<vector<int>> start_finish;
  
  // Place an cross obstacle in the middle
  for (int x = M/8; x<7*M/8; x++)
    MAP[N/2][x] = 1;
  for (int y = N/8; y<7*N/8; y++)
    MAP[y][M/2] = 1;

  start_finish.push_back({0, 0, N - 1, M - 1});
  start_finish.push_back({0, M - 1, N - 1, 0});
  start_finish.push_back({N / 2 - 1, M / 2 - 1, N / 2 + 1, M / 2 + 1});
  start_finish.push_back({N / 2 - 1, M / 2 + 1, N / 2 + 1, M / 2 - 1});
  start_finish.push_back({N / 2 - 1, 0, N / 2 + 1, M - 1});
  start_finish.push_back({N / 2 + 1, M - 1, N / 2 - 1, 0});
  start_finish.push_back({0, M / 2 - 1, N - 1, M / 2 + 1});
  start_finish.push_back({N - 1, M / 2 + 1, 0, M / 2 - 1});

  config= config%8;
  xStart = start_finish[config][0];
  yStart = start_finish[config][1];
  xFinish = start_finish[config][2];
  yFinish = start_finish[config][3];

  // Set the derived map
  setMap(MAP);
}

bool operator<(const Node& lhs, const Node& rhs)
{
  return lhs.getPriority() > rhs.getPriority();
}
