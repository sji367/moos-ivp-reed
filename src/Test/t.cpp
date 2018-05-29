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
#include<ENC_Rasterize.h>
#include "ENC_Polygonize.h"


using namespace std;

int main()
{
    int grid_size = 5000;
    int buffer_size = 100;
    double LatOrigin  = 43.071959194444446;
    double LongOrigin = -70.711610833333339;
    bool flag;
    clock_t start;
    double lat, lon;
    double t=0;

    bool build_layer = true; //false; //
    string chartname = "US5NH02M";// "US3EC10M";// "US4MA14M";// "US6MA03M";//

    double lat1, lat2, lat3, lat4;
    double lon1, lon2, lon3, lon4;

    // Get bounds of the raster
    //Geodesy geod = Geodesy(LatOrigin, LongOrigin);
    // Build the gridded ENC
    Grid_Interp grid = Grid_Interp("/home/sreed/moos-ivp/moos-ivp-reed/", chartname, grid_size, buffer_size, 3.564-0.829, false, false);
    grid.Run(false);//(true, true); // Boleans are t/f build a .csv or .mat files for each raster

    //ENC_Rasterize raster= ENC_Rasterize("/home/sreed/moos-ivp/moos-ivp-reed/", "src/ENCs/Grid/depth.shp", 356180.4070, 4757790.1787, 372748.4070, 4783833.1787);
    //raster.rasterize("test.tiff", 5, "Depth");
    //ENC_Polygonize poly = ENC_Polygonize("/home/sreed/moos-ivp/moos-ivp-reed/", "new.tiff", "raster.shp",geod,2);
    //poly.runWithGrid();



    //geod.LocalUTM2LatLong(grid.getMinX(), grid.getMinY(), lat, lon);
    //printf("x: %0.8f, y: %0.8f; lat: %0.8f, long: %0.8f\n", grid.getMinX(), grid.getMinY(),lat, lon);

//    geod.LocalUTM2LatLong(grid.getMinX(), grid.getMinY(), lat1, lon1);
//    geod.LocalUTM2LatLong(grid.getMinX(), grid.getMaxY(), lat2, lon2);
//    geod.LocalUTM2LatLong(grid.getMaxX(), grid.getMinY(), lat3, lon3);
//    geod.LocalUTM2LatLong(grid.getMaxX(), grid.getMaxY(), lat4, lon4);
//    printf("lat_north = %0.7f\nlat_south = %0.7f\nlon_east  = %0.7f\nlon_west  = %0.7f\n", (lat1+lat3)/2, (lat2+lat4)/2, (lon1+lon3)/2, (lon2+lon4)/2);
//    printf("\nbot lat: %0.6f, bot lon: %0.6f, top lat: %0.6f, bot lon: %0.6f\n", lat1, lon1, lat2, lon2);
//    printf("bot lat: %0.6f, top lon: %0.6f, top lat: %0.6f, top lon: %0.6f\n", lat3, lon3, lat4, lon4);

    //cout << "Min: " << grid.getMinX() << ", " <<grid.getMinY() << "\tMax: " << grid.getMaxX() << ", " << grid.getMaxY() << endl;

    //vector<vector <int> > Map = grid.transposeMap();
    //cout << Map[0].size() << " " << Map.size() << endl;
    //src/ENCs/Grid/rawInterp_
/*
    // Run A*
    A_Star astar= A_Star(6);//grid_size, 0, 0, 6);
    astar.setMapFromTiff("US5NH02M.tiff");

    astar.setStartFinish_LatLong(43.07020,-70.70389,43.05824,-70.67716);
    double utmX, utmY;

//    astar.LatLong2UTM(43.071959194444446,-70.711610833333339, utmX, utmY);
//    cout << utmX << "," << utmY << endl;

    //astar.setMap(grid.transposeMap());
    start = clock();

    //astar.set
    //astar.setStartFinish_Grid(895,2479,896,2502);

    //astar.setStartFinish_Grid(895,2478,2700, 855); //Out to the isle of shoals

    //astar.setStartFinish_Grid(896,2507,920,2432);
    //astar.setStartFinish_Grid(895,2478,920,2432); // Pier to other side of the peninsula
    //astar.setStartFinish(2.5,-47.5,-40,85); // One side of the pier to the other
    //astar.subsetMap(870,970, 2350, 2550);
    //astar.setStartFinish_Grid(24,128, 50,0);

    //astar.setStartFinish_Grid(894,2478,264,2661); // Up the channel
    //astar.subsetMap(800,1000, 2400, 2600);

    astar.runA_Star(false);

    //cout << endl <<endl <<endl <<endl;
    //astar.runA_Star(true, false);
    t = (clock()-start)*1e-6;

    printf("Time: %0.3f (sec)\n", t);


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

//        // Set the Spatial Reference
//        OGRSpatialReference oSRS;
//        char *pszSRS_WKT = NULL;
//        oSRS = geod.getUTM();
//        oSRS.exportToWkt( &pszSRS_WKT );
//        ds->SetProjection( pszSRS_WKT );
//        CPLFree( pszSRS_WKT );

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
            line->addPoint(atof(vect_WPTs[i].c_str()), atof(vect_WPTs[i+1].c_str()));
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
    //*/

    return 0;

}
