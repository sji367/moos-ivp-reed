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
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef K::Point_3   Point;

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
    void delaunayTri(Delaunay & tri);
    void makeMap();

    // Returns the interpolated map
    vector<vector<int> > getGriddedMap() {return Map; }

    // Functions for converting between the grid, local UTM, and Lat/Long
    void xy2grid(double x, double y, int &gridX, int &gridY);
    void grid2xy(int gridX, int gridY, double &x, double &y);
    void LatLong2Grid(double lat, double lon, int &gridX, int &gridY);
    void Grid2LatLong(int gridX, int gridY, double &lat, double &lon);

    void EnvelopeInUTM(OGREnvelope *env, double &x_min, double & y_min, double &x_max, double &y_max);

    // Put in functions to make a minimum segment length in meters for polygons/lines in lat/lon
    OGRLineString* SegmentLine_LatLon(OGRLineString* line, double segment_dist);
    OGRPolygon* SegmentPoly_LatLon(OGRPolygon* poly, double segment_dist);

    // Put in functions to make a minimum segment length in meters for polygons/lines in lat/lon
    OGRPolygon* BufferPoly_LatLon(OGRPolygon* poly, double buffer_dist);

    double dist(int x1, int y1, int x2, int y2);

private:
    Geodesy geod;
    vector<OGRLayer*> desired_layers;
    unsigned cntr;
    string ENC_filename;
    vector<std::pair<Point, unsigned> > data;
    vector<vector<int> > Map;
    double minX, minY, maxX, maxY;
    int grid_size;
    double buffer_size;

};

#endif /*GRIDENC_H_*/
