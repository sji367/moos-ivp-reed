#include "poly_AngularSweep.h"

/*
 * This is the constructor for the polyAngularSweep class
 *      Inputs:
 *          x's - ASV X position
 *          y's - ASV Y position
 *          length - ASV's length
 *          searchDistance - How far away are we searching
 *          util - vector containing the utility for each 5 degree segment
 *          debug - flag to create a shapefile containing all intersecting lines
 */
polyAngularSweep::polyAngularSweep(double x, double y, double length, double searchDist, bool debug)
{
    m_ASV_x = x;
    m_ASV_y = y;
    m_ASV_length = length;
    t_lvl = 4; //initialize it to 4, however it will be set inside the findIntersection function
    m_search_dist = searchDist;
    ASV_Location =(OGRPoint *)OGRGeometryFactory::createGeometry(wkbPoint);

    // Build the point for the position
    ASV_Location->setX(m_ASV_x);
    ASV_Location->setY(m_ASV_y);
    ASV_Location->setZ(0);

    // Initialize the polyInside object
    checkInside = polyInside(m_ASV_x, m_ASV_y);

    Debug = debug;

    // Make 360%swath size = 0
    swathSize = 5;

    // Resize the utility vector to 72 (360/5) and fill it with 100s (max_util)
    resetUtilVector();

    if (debug)
        buildLineLayer();
    //cout << "end" <<endl;
}

/*
 * This function takes in a geometry describing a polygon (geomPoly) and its threat level
 * and determines all of the posible intersections using a swathSize degree sweeps.
 */
void polyAngularSweep::findIntersections(OGRGeometry *geomPoly, double threat_level)
{
    OGRLineString *line;
    double newX,newY;
    int index=0;
    OGRGeometry *linePolyintersect, *boxPolyIntersect;

    t_lvl = threat_level;

    boxPolyIntersect = search_area_poly->Intersection(geomPoly);

    // Only try to find the intersection if the ASV is NOT inside/touching the polygon
    if (!(checkInside.determineIfInside(boxPolyIntersect)))
    {
        for (int cur_angle=0; cur_angle<360; cur_angle+=swathSize)
        {
            // Build the line extending out from the ASV's location to each 5 degree
            line =(OGRLineString *)OGRGeometryFactory::createGeometry(wkbLineString);
            line->addPoint(ASV_Location);

            getOtherVertex(cur_angle, newX, newY);
            line->addPoint(newX,newY, 0);

            // Check to see if the line intersects the polygon. If it does, store the utility
            //  of the closest vertex into a vector
            if (line->Intersects(boxPolyIntersect))
            {
                linePolyintersect = line->Intersection(boxPolyIntersect);
                storeUtil(linePolyintersect, index);
            }
            // Increment the counter for the index of the vector
            index++;
        }
    }
    else
        distance2poly.push_back(-1);
}

/* This function stores the distance to the polygon (on the 5 degree arc) and
 *  also stores the utility of that direction. If the debug flag is set, then it
 *  also stores the intersection of the 5 degrees arcs between the ASV and the polygon.
 */
void polyAngularSweep::storeUtil(OGRGeometry *intersect_geom, int index)
{
    double minDist = ASV_Location->Distance(intersect_geom);
    double utility = calcUtil(minDist);

    // Store the line into a shape file if debugging is set to true
    if (Debug)
    {
        OGRMultiLineString *multiline;
        OGRLineString *line;
        OGRFeature *new_feat;
        OGRFeatureDefn *poFDefn;
        poFDefn = LineLayer->GetLayerDefn();

        string geomName =intersect_geom->getGeometryName();
        if (geomName == "MULTILINESTRING")
        {
            multiline = (OGRMultiLineString*) intersect_geom;
            for (int i=0; i<multiline->getNumGeometries(); i++)
            {
                line = (OGRLineString *) multiline->getGeometryRef(i);
                new_feat =  OGRFeature::CreateFeature(poFDefn);
                new_feat->SetField("Distance", minDist);
                new_feat->SetField("Utility", utility);
                new_feat->SetField("Angle", index*5);

                new_feat->SetGeometry(line);
                // Build the new feature
                if( LineLayer->CreateFeature( new_feat ) != OGRERR_NONE )
                {
                    printf( "Failed to create feature in shapefile.\n" );
                    exit( 1 );
                }
            }
        }
        else
        {
            line = (OGRLineString *) intersect_geom;
            new_feat =  OGRFeature::CreateFeature(poFDefn);
            new_feat->SetField("Distance", minDist);
            new_feat->SetField("Utility", utility);
            new_feat->SetField("Angle", index*5);

            new_feat->SetGeometry(line);
            // Build the new feature
            if( LineLayer->CreateFeature( new_feat ) != OGRERR_NONE )
            {
                printf( "Failed to create feature in shapefile.\n" );
                exit( 1 );
            }
        }
    }

    // Only store the utility if less than the one that is currently stored in the function
    if (utility < util[index])
        util[index] = utility;

    distance2poly.push_back(minDist);
}

double polyAngularSweep::getMinDist()
{
    if (distance2poly.size() >0)
    {
        minDist = *min_element(distance2poly.begin(), distance2poly.end());
        distance2poly.clear();
        return minDist;
    }
}


// This function converts the inputed distance to a utilty.
double polyAngularSweep::calcUtil(double dist)
{
    double utility = 2.5*pow(2,dist/m_ASV_length)/pow(t_lvl,2);

    if (utility > 100)
        utility = 100;

    return utility;
}

void polyAngularSweep::getOtherVertex(double angle, double &x, double &y)
{
    // In MOOS angles
    x = m_search_dist*sin(angle/180*PI)+m_ASV_x;
    y = m_search_dist*cos(angle/180*PI)+m_ASV_y;
}

string polyAngularSweep::getUtilVect_asString()
{
    string utilString;
    int windowSize = 5;


    // Use a moving average filter to extend angularly the penality for driving toward
    // polygons by (windowSize+1)/2*swathSize, which is 15 degrees in the default config.
    movingAverageFilter(windowSize);

    // Check to see if the ASV is touching or within a polygon. If it is, reset the utility
    //  function to get out of the polygon as fast as posible.
    resetUtilityIfInside();

    // cycle through the vector and store a interger version of the utility
    for(auto i : util)
        utilString += to_string(static_cast<int>(round(i))) +",";

    // Remove the last comma
    utilString.pop_back();

    return utilString;
}

void polyAngularSweep::build_search_poly()
{
    OGRLinearRing *poRing;
    OGRLineString *line;
    OGRFeature *new_feat;
    OGRFeatureDefn *poFDefn;

    double x1, x2, x3, x4, y1, y2, y3, y4;
    double theta = 45+fmod(0,90);
    double add_sin = m_search_dist*sin(theta*PI/180);
    double add_cos = m_search_dist*cos(theta*PI/180);

    x1 = m_ASV_x+add_sin;
    x2 = m_ASV_x-add_cos;
    x3 = m_ASV_x-add_sin;
    x4 = m_ASV_x+add_cos;

    y1 = m_ASV_y+add_cos;
    y2 = m_ASV_y+add_sin;
    y3 = m_ASV_y-add_cos;
    y4 = m_ASV_y-add_sin;

    search_area_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
    poRing = ( OGRLinearRing * ) OGRGeometryFactory::createGeometry(wkbLinearRing);
    poRing->addPoint(x1, y1);
    poRing->addPoint(x2, y2);
    poRing->addPoint(x3, y3);
    poRing->addPoint(x4, y4);
    poRing->closeRings();
    search_area_poly->addRing(poRing);
    search_area_poly->closeRings();

    // Add the search box to the shapefile
    if (Debug)
    {
        poFDefn = LineLayer->GetLayerDefn();
        line =(OGRLineString *)OGRGeometryFactory::createGeometry(wkbLineString);
        line->addPoint(x1, y1);
        line->addPoint(x2, y2);
        line->addPoint(x3, y3);
        line->addPoint(x4, y4);
        line->addPoint(x1, y1);
        new_feat =  OGRFeature::CreateFeature(poFDefn);
        new_feat->SetField("Distance", 0);
        new_feat->SetField("Utility", 0);
        new_feat->SetField("Angle", 0);
        new_feat->SetGeometry(line);

        if( LineLayer->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in shapefile.\n" );
            exit( 1 );
        }
    }
}

void polyAngularSweep::buildLineLayer()
{
    GDALAllRegister();
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );

    DS = poDriver->Create( "src/ENCs/Shape/Line.shp", 0, 0, 0, GDT_Unknown, NULL );
    if( DS == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    LineLayer = DS->CreateLayer( "Line", NULL, wkbLineString, NULL );
    if( LineLayer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    OGRFieldDefn oField_dist( "Distance", OFTInteger);
    if( LineLayer->CreateField( &oField_dist ) != OGRERR_NONE )
    {
        printf( "Creating distance field failed.\n" );
        exit( 1 );
    }
    OGRFieldDefn oField_Util( "Utility", OFTInteger);
    if( LineLayer->CreateField( &oField_Util ) != OGRERR_NONE )
    {
        printf( "Creating utility field failed.\n" );
        exit( 1 );
    }
    OGRFieldDefn oField_Angle( "Angle", OFTInteger);
    if( LineLayer->CreateField( &oField_Angle ) != OGRERR_NONE )
    {
        printf( "Creating angle field failed.\n" );
        exit( 1 );
    }

}

double polyAngularSweep::ang4MOOS(double oldAngle)
{
    double newAng = 90-oldAngle;
    if (newAng<0)
        newAng+=360;

    return newAng;

}

// This function Check to see if the ASV is touching or within a polygon. If it is, reset the
//  utility vector to get out of the closest polygon (that it is inside of) as fast as posible.
void polyAngularSweep::resetUtilityIfInside()
{
    double angle;
    int index;
    double fillRemainder = 10;
    double fill5deg_off = 100;
    if (checkInside.getIf_anyPolys_Inside())
    {
        // Reset the utility
        fill (util.begin(),util.end(),fillRemainder);

        // Calculate the angle and convert it to a index of the vector
        angle = checkInside.pickAngle2SteerTowards();
        //cout << "angle: " << angle << endl;
        index = static_cast<int>(round(angle/5));

        util[index] = 100;

        // Update the 20 degree swath of high utility (Utility falls off linearly +/- 10 degrees away
        //  from the fastest way out of the polygon
        if (index == 0)
        {
            util[index+1] = fill5deg_off;
            util[71]= fill5deg_off;
        }
        else if(index == 71)
        {
            util[0]= fill5deg_off;
            util[index-1] = fill5deg_off;
        }
        else
        {
            util[index+1] = fill5deg_off;
            util[index-1] = fill5deg_off;
        }
    }
}

// This function runs a moving average over a odd numvered size window. (If the
//  window size is not odd, then one will be added to the window size). Once the
//  moving average has been calculated, it is stored as the current utility only
//  if it less than the value currently stored in the utility vector.
void polyAngularSweep::movingAverageFilter(int windowSize)
{
    int plusMinus = (windowSize-1)/2;
    int utilSize = util.size();
    vector<double> localUtil = util;
    int curIndex = 0;
    double cumSum, movingAve;
    // Window size must be a odd number. Therefore if it is an even number then
    //  add one to the window size.
    if (windowSize%2==0)
        windowSize++;

    // Cycle through the utility vector and run a moving average filter.
    for (int index = 0; index<utilSize; index++)
    {
        cumSum = 0;
        for (int windowIndex = -plusMinus; windowIndex<=plusMinus; windowIndex++)
        {
            // Deal with the wrap around
            curIndex = index+windowIndex;
            if (curIndex < 0)
                curIndex += utilSize;
            else if (curIndex>=utilSize)
                curIndex -= utilSize;

            cumSum += localUtil[curIndex];
        }

        // Store the average utility of the window if the utility is less than
        //  the value that was previously stored.
        movingAve = cumSum/(windowSize*1.0);
        if (util[index]>movingAve)
            util[index]=movingAve;
    }
}
