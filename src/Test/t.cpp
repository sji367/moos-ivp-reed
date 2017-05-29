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
#include "gridinterp.h"


using namespace std;

int main()
{
    int grid_size = 5;
    int buffer_size = 5;
    double LatOrigin  = 43.071959194444446;
    double LongOrigin = -70.711610833333339;
    Grid_Interp grid = Grid_Interp("/home/sji367/moos-ivp/moos-ivp-reed/", "/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs/US5NH02M/US5NH02M.000", grid_size, buffer_size, LatOrigin, LongOrigin);
    grid.Run(true);

    printf("minX: %0.3f, minY: %0.3f, maxX: %0.3f, maxY: %0.3f\n", grid.getMinX(), grid.getMinY(), grid.getMaxX(), grid.getMaxY());
    /*
    A_Star astar = A_Star(grid_size, grid.getMaxX(), grid.getMaxY(), 1);

    astar.setMap(grid.transposeMap());
    astar.setStartFinish_Grid(896,2507,920,2432);
    astar.subsetMap(870,970, 2400, 2600);
    astar.getNM();
    astar.runA_Star(true);
*/

    return 0;

}

/*

  bool flag;
  A_Star astar = A_Star(2);
  int gridX, gridY;
  double x,y;
  Vessel_Dimensions ShipMeta = Vessel_Dimensions(1.8,0.9,0.3,5.4);
  //ShipMeta.getVesselMeta();
  astar.setDesiredSpeed(4*0.514444);
  astar.setShipMeta(ShipMeta);
  //astar.build_default_map(10,10,3);
  
  // Big map

  astar.build_map("valsisland.csv");
  astar.setStartFinish_Grid(1,1,33,33);
  flag = astar.runA_Star(true);
  astar.setStartFinish_Grid(1,1,65,65);
  flag = astar.runA_Star(true);
  astar.setStartFinish_Grid(1,1,99,99);
  flag = astar.runA_Star(true);
/*
  // Small map
  //astar.setStartFinish_Grid(892,2472,890,2488);//(0.0,-25.0,-0.0,-55);
  astar.setStartFinish_Grid(880+45,103+2400,908,2403);//(0.0,-25.0,-10.0,-55);
  astar.build_map("src/ENCs/Shape/grid/grid.csv",880,980,2400,2600);

  astar.getNM();
  printf("Start: %i, %i\t Finish: %i, %i\n", astar.getStartX(), astar.getStartY(), astar.getFinishX(), astar.getFinishY());
  flag = astar.runA_Star(true);

  //astar.setStartFinish_Grid(12,72,28,3);
  //flag = astar.runA_Star(true);

  //cout << astar.getDepthCutoff() << endl;


  astar.NeighborsMask(2);
  flag = astar.runA_Star(true);

  astar.NeighborsMask(3);
  flag = astar.runA_Star(true);

  // Build .L84 file
  //flag = astar.runA_Star(true, true, "test",43.071959194444446, -70.711610833333339);

  // Build .moos file
  // WPT1
  //astar.setStartFinish(0.0,-25.0,-10.0,55);
  //flag = astar.runA_Star(true);//, true, "missions/Portsmouth/mission2");
  

  // WPT2
  astar.setStartFinish(-10.0,55,-27, 57);
  flag = astar.runA_Star(true);
  if (flag)
    cout << "Check the waypoints" << endl;
  
  // WPT3
  //astar.setStartFinish(-27, 57, 0.0,-25.0);
  flag = astar.runA_Star(true);
  if (flag)
    cout << "Check the waypoints" << endl;

	
  return 0;
}

*/
