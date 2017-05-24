/*
 * gridENC.h
 *
 *  Created on: April 21, 2017
 *      Author: Sam Reed
 */

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

using namespace std;

#ifndef GRIDENC_H_
#define GRIDENC_H_

class Grid_ENC
{
public:
    // Constructor/deconstuctor
    Grid_ENC();
    Grid_ENC(string ENC_Filename, int Grid_size, double buffer_dist, Geodesy Geod);
    ~Grid_ENC();

    // Initialize Geodesy
    void initGeodesy(Geodesy Geod) { geod = Geodesy(Geod.getLatOrigin(), Geod.getLonOrigin()); }

    // Grab the sounding, contour line, and land area layers and get the XYZ locations of each vertex
    void MakeBaseMap(int segment_size);

    // Function to rasterize the layer and add each point to the vector Map
    void rasterizePoints(OGRLayer * Layer);
    void rasterizeMultiPoints(OGRLayer * Layer);
    void rasterizePoly(OGRLayer * Layer, double segment_size);
    void rasterizeLine(OGRLayer * Layer, double segment_size);

    // Interpolate the map using linterp library
    void makeMap();

    // Returns the interpolated map
    vector<vector<int> > getGriddedMap() {return Map; }

    // Functions for converting between the grid, local UTM, and Lat/Long
    void xy2grid(double x, double y, int &gridX, int &gridY);
    void grid2xy(int gridX, int gridY, double &x, double &y);
    void LatLong2Grid(double lat, double lon, int &gridX, int &gridY);
    void Grid2LatLong(int gridX, int gridY, double &lat, double &lon);

    void EnvelopeInUTM(OGREnvelope* env, double &x_min, double &y_min, double &x_max, double &y_max);
    void EnvelopeInUTM(OGRLayer* layer);

    // Put in functions to make a minimum segment length in meters for polygons/lines in lat/lon
    OGRLineString* SegmentLine_LatLon(OGRLineString* line, double segment_dist);
    OGRPolygon* SegmentPoly_LatLon(OGRPolygon* poly, double segment_dist);

    // Put in functions to make a minimum segment length in meters for polygons/lines in lat/lon
    OGRPolygon* BufferPoly_LatLon(OGRPolygon* poly, double buffer_dist);

    double dist(int x1, int y1, int x2, int y2);

private:
    Geodesy geod;
    vector<OGRLayer*> desired_layers;
    string ENC_filename;
    vector<vector<int> > data;
    vector<vector<int> > Map;
    double minX, minY, maxX, maxY;
    int grid_size;
    double buffer_size;

};

class ENC_grid
{
public:
    // Constructor/deconstuctor
    ENC_grid();
    ENC_grid(string ENC_Filename, double Grid_size, double buffer_dist, double lat, double lon);
    ENC_grid(string ENC_Filename, double Grid_size, double buffer_dist, Geodesy Geod);
    ~ENC_grid() {}

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

    void makeGrid(OGRGeometry *geom);
    void run();
    double calc_dist_sq(int x1, int y1, int x2, int y2);

    // Functions for converting between the grid, local UTM, and Lat/Long
    void UTM2grid(double x, double y, int &gridX, int &gridY) { xy2grid(x+geod.getXOrigin(),y+geod.getYOrigin(), gridX, gridY);}
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

#endif /*GRIDENC_H_*/
