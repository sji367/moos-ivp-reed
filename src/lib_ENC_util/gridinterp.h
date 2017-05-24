#include <vector>
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort
#include "gdal_frmts.h" // for GDAL/OGR
#include <ogrsf_frmts.h> // OGRLayer
#include <ogr_geometry.h> // OGRPoint, OGRGeometry etc
#include <ogr_feature.h> // OGRFeature
#include "geodesy.h"
#include "gdal_alg.h"

using namespace std;

#ifndef GRIDINTERP_H
#define GRIDINTERP_H

class Grid_Interp
{
public:
    Grid_Interp();
    Grid_Interp(string ENC_Filename, double Grid_size, double buffer_dist, Geodesy Geod);
    Grid_Interp(string ENC_Filename, double Grid_size, double buffer_dist, double lat, double lon);
    ~Grid_Interp() {}

    // Initialize Geodesy
    void initGeodesy(Geodesy Geod) { geod = Geodesy(Geod.getLatOrigin(), Geod.getLonOrigin()); }

    void Run();

    // Get the minimum and maximum x/y values (in local UTM) of the ENC
    void getENC_MinMax(GDALDataset* ds);

    // Functions to deal with build the delaunay polygon
    void rasterizeLayer(OGRLayer* layer, string layerName);
    void multipointFeat(OGRFeature* feat, OGRGeometry* geom);
    void pointFeat(OGRFeature* feat, OGRGeometry* geom, string layerName);
    void polygonFeat(OGRFeature* feat, OGRGeometry* geom, string layerName);
    void lineFeat(OGRFeature* feat, OGRGeometry* geom, string layerName);

    // Returns the interpolated map
    vector<vector<int> > getGriddedMap() {return Map; }

    // Functions for converting between the grid, local UTM, and Lat/Long
    void UTM2grid(double x, double y, int &gridX, int &gridY) { xy2grid(x+geod.getXOrigin(),y+geod.getYOrigin(), gridX, gridY);}
    void xy2grid(double x, double y, int &gridX, int &gridY);
    void grid2xy(int gridX, int gridY, double &x, double &y);
    void LatLong2Grid(double lat, double lon, int &gridX, int &gridY);
    void Grid2LatLong(int gridX, int gridY, double &lat, double &lon);


private:
    Geodesy geod;
    string ENC_filename;
    vector<double> X,Y,depth;
    vector<vector<int> > Map;
    double minX, minY, maxX, maxY;
    int grid_size;
    double buffer_size;
};

#endif // GRIDINTERP_H

