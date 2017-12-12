#ifndef POLYINSIDE_H
#define POLYINSIDE_H
#define PI 3.14159265

#include <vector>
#include <string>
#include <cmath> // for pow and sqrt
#include <math.h>
#include <algorithm> // for min_element and max_element
#include "geodesy.h" // Conversion between lat/lon and UTM
#include "ogrsf_frmts.h" // GDAL
#include "ogr_spatialref.h"
#include <iostream>

using namespace std;

class polyInside
{
public:
    polyInside(double ASV_x, double ASV_y);
    polyInside() {polyInside(0,0); }
    ~polyInside() {}

    // This function determines if the ASV is inside of the current polygon, and if the ASV
    //  is inside the polygon, if determines the angle to get out of the polygon.
    bool determineIfInside(OGRGeometry *polyGeom);

    void storeMinAng_MinDist(OGRPolygon *poly);

    // This function picks the angle to steer towards by looking at all of the polygons that the
    //  ASV is inside and returns the angle to the closest vertex. It also clears the both vectors.
    double pickAngle2SteerTowards();

    // Returns if the ASV was inside any polygons
    bool getIf_anyPolys_Inside() {return inside; }

    double calcDist2ASV(double x, double y) { sqrt(pow(x-m_ASV_x,2)+pow(y-m_ASV_y,2));}

    double ang4MOOS(double oldAngle);

private:
    bool inside;
    double m_ASV_x, m_ASV_y;
    vector<double> minDist;
    vector<double> angle2minDist;
    OGRPoint *ASV_Location;
};

#endif // POLYINSIDE_H
