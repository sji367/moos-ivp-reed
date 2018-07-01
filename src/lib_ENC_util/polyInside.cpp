#include "polyInside.h"

// Constructor - needs the ASV's current location as two doubles (x,y)
polyInside::polyInside(double ASV_x, double ASV_y)
{
    inside = false;
    ASV_Location = (OGRPoint *)OGRGeometryFactory::createGeometry(wkbPoint);
    ASV_Location->setX(ASV_x);
    ASV_Location->setY(ASV_y);
    m_ASV_x = ASV_x;
    m_ASV_y = ASV_y;
}

// This function determines if the ASV is inside of the current polygon, and if the ASV
//  is inside the polygon, if determines the angle to get out of the polygon.
bool polyInside::determineIfInside(OGRGeometry *polyGeom)
{
    OGRPolygon *poly;
    OGRMultiPolygon *multipoly;
    bool thisPoly_Inside;
    string geomName;

    // Check to see if the ASV is inside or touches the polygon
    thisPoly_Inside = (ASV_Location->Within(polyGeom))||(ASV_Location->Touches(polyGeom));
    inside = inside||thisPoly_Inside;

    if (thisPoly_Inside)
    {
        geomName =polyGeom->getGeometryName();
        if (geomName == "MULTIPOLYGON")
        {
            multipoly = (OGRMultiPolygon *) polyGeom;
            cout << geomName<< ": "<< multipoly->getNumGeometries() << endl;
            for (int i=0; i<multipoly->getNumGeometries(); i++)
            {
                poly = (OGRPolygon *) multipoly->getGeometryRef(i);
                if ((ASV_Location->Within(poly))||(ASV_Location->Touches(poly)))
                {
                    cout << "inside" << endl;
                    storeMinAng_MinDist(poly);
                }
            }
        }
        else
        {
            poly = (OGRPolygon *) polyGeom;
            if (thisPoly_Inside)
                storeMinAng_MinDist(poly);
        }
    }

    return thisPoly_Inside;
}

void polyInside::storeMinAng_MinDist(OGRPolygon *poly)
{
    OGRLinearRing *ring;
    double current_minDist = 99999;
    double angle2poly, x,y, distance2vertex;

    ring = poly->getExteriorRing();

    // Find which vertex is the closest and store that angle in MOOS coordinates
    for (int i=0; i<ring->getNumPoints(); i++)
    {
        x = ring->getX(i);
        y = ring->getY(i);
        distance2vertex = calcDist2ASV(x,y);
        // Store the angle in MOOS coordinates
        if (distance2vertex<current_minDist)
        {
            angle2poly = ang4MOOS(atan2(y-m_ASV_y,x-m_ASV_x)* 180 / PI); //atan2(x,y)* 180 / PI;//
            current_minDist = distance2vertex;
        }
    }
    angle2minDist.push_back(angle2poly);
    minDist.push_back(current_minDist);
}

// This function picks the angle to steer towards by looking at all of the polygons that the
//  ASV is inside and returns the angle to the closest vertex. It also clears the both vectors.
double polyInside::pickAngle2SteerTowards()
{
    // Get the angle to the closest vertex for any polygon that the ASV is within or touching
    int index = distance(minDist.begin(), min_element(minDist.begin(), minDist.end()));
    //cout << "index " << index << ",\t" << angle2minDist.size() << endl;
    double angle = angle2minDist[index];
    //cout << angle << endl;

    // Clear the vectors
    angle2minDist.clear(); minDist.clear();

    return angle;

}

double polyInside::ang4MOOS(double oldAngle)
{
    double newAngle= 90-oldAngle;
    if (newAngle<0)
        return newAngle+360;
    else
        return newAngle;
}

