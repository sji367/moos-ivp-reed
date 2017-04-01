/*
 * t.cpp
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <iostream>
#include <algorithm>
#include <astar.h>
#include <cmath>

using namespace std;


void print_all(A_Star &astar);

int main()
{
  bool flag;
  A_Star astar = A_Star(8);
  int gridX, gridY;
  double x,y;
  Vessel_Dimensions ShipMeta = Vessel_Dimensions(1.8,0.9,0.3,5.4);
  //ShipMeta.getVesselMeta();
  astar.setDesiredSpeed(4*0.514444);
  astar.setShipMeta(ShipMeta);
  //astar.build_default_map(10,10,3);

  // Big map
  astar.setStartFinish_Grid(16,169,29,0);
  astar.build_map("src/ENCs/Shape/grid/grid.csv",880,980,2300,2500);
  
  // Small map
  //astar.setStartFinish_Grid(2,7,26,14);
  //astar.build_map("src/ENCs/Shape/grid/grid.csv",887,917,2460,2490);

  // Build .L84 file
  //flag = astar.runA_Star(true, true, "test",43.071959194444446, -70.711610833333339);

  // Build .moos file
  flag = astar.runA_Star(true, true, "/missions/portsmouth/test");

  if (flag)
    cout << "Check the waypoints" << endl;
  
  return 0;
}

void print_all(A_Star &astar)
{
  for (int i =0; i<8; i++)
    {
      astar.build_default_map(30,30,i);
      astar.runA_Star();
      cout << "%-----------------------------------------------------%" << endl;
    }
}
