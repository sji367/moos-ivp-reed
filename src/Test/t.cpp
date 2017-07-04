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
#include "geodesy.h"
#include <ctime>


using namespace std;

int main()
{
    int grid_size = 5;
    int buffer_size = 5;
    double LatOrigin  = 43.071959194444446;
    double LongOrigin = -70.711610833333339;
    bool flag;
    clock_t start;
    double lat, lon;
    double t=0;


    bool build_layer = false;

    double lat1, lat2, lat3, lat4;
    double lon1, lon2, lon3, lon4;

    // Build the gridded ENC
    Grid_Interp grid = Grid_Interp("/home/sji367/moos-ivp/moos-ivp-reed/", "/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs/US5NH02M/US5NH02M.000", grid_size, buffer_size,3.564-0.829, LatOrigin, LongOrigin);
    grid.Run(false);//(true, true); // Boleans are t/f build a .csv or .mat files for each raster


    // Get bounds of the raster
    Geodesy geod = Geodesy(LatOrigin, LongOrigin);
    /*
    geod.LocalUTM2LatLong(grid.getMinX(), grid.getMinY(), lat, lon);
    printf("lat: %0.8f, long: %0.8f\n",lat, lon);

    geod.LocalUTM2LatLong(grid.getMinX(), grid.getMinY(), lat1, lon1);
    geod.LocalUTM2LatLong(grid.getMinX(), grid.getMaxY(), lat2, lon2);
    geod.LocalUTM2LatLong(grid.getMaxX(), grid.getMinY(), lat3, lon3);
    geod.LocalUTM2LatLong(grid.getMaxX(), grid.getMaxY(), lat4, lon4);

    printf("lat_north = %0.7f\nlat_south = %0.7f\nlon_east  = %0.7f\nlon_west  = %0.7f\n", (lat1+lat3)/2, (lat2+lat4)/2, (lon1+lon3)/2, (lon2+lon4)/2);
    printf("\nbot lat: %0.6f, bot lon: %0.6f, top lat: %0.6f, bot lon: %0.6f\n", lat1, lon1, lat2, lon2);
    printf("bot lat: %0.6f, top lon: %0.6f, top lat: %0.6f, top lon: %0.6f\n", lat3, lon3, lat4, lon4);
    */

    // Run A*
    A_Star astar= A_Star(grid_size, grid.getMinX(), grid.getMinY(), 8);

    astar.setMap(grid.transposeMap());
    start = clock();

    //astar.set
    //astar.setStartFinish_Grid(895,2479,896,2502);

    astar.setStartFinish_Grid(895,2478,2700, 855); //Out to the isle of shoals

    //astar.setStartFinish_Grid(896,2507,920,2432);
    //astar.setStartFinish_Grid(895,2478,920,2432); // Pier to other side of the peninsula
    //astar.setStartFinish(2.5,-47.5,-40,85); // One side of the pier to the other
    //astar.subsetMap(870,970, 2350, 2550);
    //astar.setStartFinish_Grid(24,128, 50,0);

    //astar.setStartFinish_Grid(894,2478,264,2661); // Up the channel
    //astar.subsetMap(800,1000, 2400, 2600);
    astar.runA_Star(false);
    t = (clock()-start)*0.001;

    cout << "Time: " << t << " (ms)" << endl;


    // Build a shape file that holds the information on the path
    if (build_layer)
    {
        // Build the datasource and layer that will hold independant points
        string Path = "line.shp";
        OGRLayer *layer;
        GDALAllRegister();
        const char *pszDriverName = "ESRI Shapefile";
        GDALDriver *poDriver;
        poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
        GDALDataset *ds = poDriver->Create(Path.c_str(), 0, 0, 0, GDT_Unknown, NULL );

        // Set the Spatial Reference
        OGRSpatialReference oSRS;
        char *pszSRS_WKT = NULL;
        oSRS = geod.getUTM();
        oSRS.exportToWkt( &pszSRS_WKT );
        ds->SetProjection( pszSRS_WKT );
        CPLFree( pszSRS_WKT );

        // Check to see if the datasource and layer are valid
        if( ds == NULL )
        {
            printf( "Creation of output file failed.\n" );
            exit( 1 );
        }
        // Create the layers
        layer = ds->CreateLayer( "line", NULL, wkbLineString25D, NULL );
        if( layer == NULL )
        {
            printf( "Layer creation failed.\n" );
            exit( 1 );
        }

        OGRFeature *new_feat;
        OGRGeometry *geom;
        OGRLineString *line = (OGRLineString *)OGRGeometryFactory::createGeometry(wkbLineString25D);
        OGRFeatureDefn *poFDefn = layer->GetLayerDefn();
        new_feat =  OGRFeature::CreateFeature(poFDefn);

        // Parse the waypoint string
        vector<string> vect_WPTs;
        char delimiter = ',';
        vect_WPTs = split(astar.getRoute(), delimiter);

        // Add the waypoints to a OGRLineString
        for (int i = 0; i<vect_WPTs.size(); i+=2)
        {
            line->addPoint(atof(vect_WPTs[i].c_str())+geod.getXOrigin(), atof(vect_WPTs[i+1].c_str())+geod.getYOrigin());
        }
        new_feat->SetGeometry(line);

        // Build the new feature
        if( layer->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in shapefile.\n" );
            exit( 1 );
        }

        GDALClose(ds);
    }

/*
    astar = A_Star(8);
    astar.setStartFinish_Grid(50,0, 24,128);
    astar.runA_Star(true);


    astar.build_map("valsisland.csv");
    astar.setStartFinish_Grid(1,1,33,33);
    flag = astar.runA_Star(true);
    astar.setStartFinish_Grid(33,33, 1,1);
    flag = astar.runA_Star(true);

    astar.setStartFinish_Grid(65,65, 1,1);
    flag = astar.runA_Star(true);
    astar.setStartFinish_Grid(1,1,65,65);
    flag = astar.runA_Star(true);

    astar.setStartFinish_Grid(99,99, 1,1);
    flag = astar.runA_Star(true);
    astar.setStartFinish_Grid(1,1,99,99);
    flag = astar.runA_Star(true);


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
