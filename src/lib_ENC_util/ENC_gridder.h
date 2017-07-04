/*
 * ENC_gridder.h
 *
 *  Created on: May 17, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <iostream>
#include <string>
#include <algorithm> // for max_element and sort
#include <ogrsf_frmts.h> // OGRLayer
#include "geodesy.h"

using namespace std;

#ifndef ENC_GRIDDER_H_
#define ENC_GRIDDER_H_

class ENC_gridder
{
public:
    // Constructor/deconstuctor
    ENC_gridder();
    ENC_gridder(string ENC_Filename, double Grid_size, double buffer_dist, double lat, double lon);
    ENC_gridder(string ENC_Filename, double Grid_size, double buffer_dist, Geodesy Geod);
    ~ENC_gridder() {}

    // Initialize Geodesy
    void initGeodesy(Geodesy Geod) { geod = Geodesy(Geod.getLatOrigin(), Geod.getLonOrigin()); }
    
    void buildDelaunayPoly();

    // Get the minimum and maximum x/y values (in local UTM) of the ENC
    void getENC_MinMax(GDALDataset* ds);

    // Functions to deal with build the delaunay polygon 
    void rasterizeLayer(OGRLayer* layer, string layerName);
    void multipointFeat(OGRFeature* feat, OGRGeometry* geom);
    void pointFeat(OGRFeature* feat, OGRGeometry* geom);
    void polygonFeat(OGRFeature* feat, OGRGeometry* geom, string layerName);
    void lineFeat(OGRFeature* feat, OGRGeometry* geom, string layerName);
    

    void makeGrid() {};
    void run() {};

    // Functions for converting between the grid, local UTM, and Lat/Long
    void xy2grid(double x, double y, int &gridX, int &gridY);
    void grid2xy(int gridX, int gridY, double &x, double &y);
    void LatLong2Grid(double lat, double lon, int &gridX, int &gridY);
    void Grid2LatLong(int gridX, int gridY, double &lat, double &lon);

private:
    Geodesy geod;
    OGRPolygon *delaunayPoly, *delaunayLand_poly;
    OGRLinearRing *delaunayRing, *delaunayLand_ring;
    string ENC_filename;
    vector<vector<int> > Map;
    double minX, minY, maxX, maxY;
    double grid_size;
    double buffer_size;

};

#endif /*ENC_GRIDDER_H_*/
