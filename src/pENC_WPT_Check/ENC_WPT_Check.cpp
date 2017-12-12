/************************************************************/
/*    NAME: Sam Reed                                              */
/*    ORGN: MIT                                             */
/*    FILE: ENC_WPT_Check.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include "ENC_WPT_Check.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ENC_WPT_Check::ENC_WPT_Check()
{
    m_timewarp   = 1;
    buffer_dist = 25;
    ASV_x =0;
    ASV_y = 0;
    WPT_x =0;
    WPT_y =0;
    prev_WPT_x =0;
    prev_WPT_y =0;
    first_iteration = true;
    first_position = true;
    check_within = false;
    check_intersection = false;
    skipped = false;
    wpt_index = 0;
    m_ENC_INT = false;
    openned = false;
    curr_WPT = (OGRPoint*) OGRGeometryFactory::createGeometry(wkbPoint);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ENC_WPT_Check::OnNewMail(MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;

    string name;
    for(p=NewMail.begin(); p!=NewMail.end(); p++)
    {
        CMOOSMsg &msg = *p;
        name = msg.GetName();
        if (name == "NAV_X")
            vect_x.push_back(msg.GetDouble());
        else if (name == "NAV_Y")
            vect_y.push_back(msg.GetDouble());
        else if (name == "ENC_INIT")
            m_ENC_INT = (msg.GetString()=="true");
        else if (name== "WPT_INDEX")
            wpt_index=int(msg.GetDouble());
        else if (name=="Next_WPT")
            vect_nextWPT.push_back(msg.GetString());
    }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ENC_WPT_Check::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);
	
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ENC_WPT_Check::Iterate()
{
    if (m_ENC_INT)
    {        if (!openned)
            openned = openLayers();

        // Find and store the first position
        if ((vect_x.size()>0)&&(vect_y.size()>0)&&(first_position))
        {

            // Store and clear the variables
            ASV_x = vect_x.back();
            ASV_y = vect_y.back();
            vect_x.clear(); vect_y.clear();
            first_position = false;
        }
        else
        {
            //When there is a new waypoint, we want to check to see if the next
            //   waypoint is valid and if the planned path intersects any polygon.
            //   If the waypoint is NOT valid, then we will post a flag and
            //   calculate the intersection between the polygon and the planned
            //   path.
            if (vect_nextWPT.size()>0)
            {
                cout << "Got new WPT: "+vect_nextWPT.back() << endl;
                WPT_Valid(vect_nextWPT.back());
                vect_nextWPT.clear();
            }

            // Check to see if there is a new position and if there is check to see
            //  if the waypoint needs to be skipped.
            if ((vect_x.size()>0)&&(vect_y.size()>0)&&(!first_position))
            {
                // Store and clear the variables
                ASV_x = vect_x.back();
                ASV_y = vect_y.back();
                vect_x.clear(); vect_y.clear();

                //If the waypoint is within a polygon, then switch to the next
                //   waypoint when it is comes within the buffer distance
                WPT_skip();
            }
        }

    }
    return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ENC_WPT_Check::OnStartUp()
{
    list<string> sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams))
    {
        list<string>::iterator p;
        for(p=sParams.begin(); p!=sParams.end(); p++)
        {
            string original_line = *p;
            string param = stripBlankEnds(toupper(biteString(*p, '=')));
            string value = stripBlankEnds(*p);

            if(param == "BUFFER_DIST")
                buffer_dist = atoi(value.c_str());
        }
    }

    m_timewarp = GetMOOSTimeWarp();

    RegisterVariables();
    return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void ENC_WPT_Check::RegisterVariables()
{
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("ENC_INIT",0);
    Register("WPT_INDEX", 0);
    Register("Next_WPT", 0);
}

bool ENC_WPT_Check::openLayers()
{
    GDALAllRegister();
    // Open the data source so that we can use them later in the iterate loop
    ds_poly = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Poly.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds_poly == NULL )
    {
        printf( "Poly datasource open failed.\n" );
        exit( 1 );
    }

    // Open the layer
    layer = ds_poly -> GetLayerByName( "Poly" );
    if (layer == NULL)
    {
        cout << "Opening poly layer failed." << endl;
        exit( 1 );
    }

    return true;
}

//If the waypoint is within a polygon, then switch to the next
//  waypoint when it is comes within the buffer distance.
void ENC_WPT_Check::WPT_skip()
{
    double dist2interpt =0;
    if (check_within)
    {
        for(int k=0; k<X_intersection.size(); k++)
        {
            dist2interpt = sqrt(pow(X_intersection[k]-ASV_x,2) + pow(Y_intersection[k]-ASV_y,2));

            if (dist2interpt <= buffer_dist)
            {
                // Increment to the next waypoint
                cout << "Skiping " << wpt_index << ", now navigating towards " << wpt_index << "." << endl;
                Notify("WPT_UPDATE", "currix="+to_string(wpt_index+1));
                check_within = false;
                prev_WPT_x = ASV_x;
                prev_WPT_y = ASV_y;
                skipped = true;

                // Clear the pink dots on pMarineViewer
                for (auto i:polyIntersect_Index)
                    Notify("VIEW_POINT", "x=1,y=1,active=false,label=intercept_"+to_string(i));
            }
        }

    }
}

/*
There are two cases for setting the previous waypoint:
    1) On the first iteration there is no previous waypoint.
        Therefore, we will set the current position of the ASV as
        the previous waypoint

    2) Afterwards, there is a previous waypoint so we will just set
        the previous waypoint as the current value of WPT_x or
        WPT_y. If we skipped a waypoint, the previous waypoint has
        been previously set in the WPT_skip function.
Inputs:
    ASV_x - Current x position of the ASV
    ASV_y - Current y position of the ASV
*/
void ENC_WPT_Check::Set_Prev_WPT()
{
    // On the first iteration there is no previous waypoint. Therefore,
    //   we will set the current position of the ASV as previous
    //   waypoint.
    if (first_iteration)
    {
        prev_WPT_x = ASV_x;
        prev_WPT_y = ASV_y;
        first_iteration = false;
    }

    // Afterwards, there is a previous waypoint so we will just set the
    //   previous waypoint as the current value of WPT_x or WPT_y. If
    //   we skipped a waypoint, the previous waypoint has been previously
    //   set in an earlier function.
    else
    {
        if (skipped)
            skipped = false;
        else
        {
            // Set the current position to the ASV as the x,y coordinate
            //   of the previous waypoint
            prev_WPT_x = WPT_x;
            prev_WPT_y = WPT_y;
        }
    }
}

/*
This function sets the current waypoint and then determines the
    straight line path between the two points.

Inputs:
    WPT - String from MOOS that gives the x,y position of the current
            waypoint and is in the form of WPT_x,WPT_y

Outputs:
    WPT_poly - OGR polygon describing the straight line path with a
            small buffer
    curr_WPT - OGR point describing the current waypoint
*/
OGRLineString* ENC_WPT_Check::BuildWPT(string WPT, OGRPolygon **WPT_poly)
{
    OGRPoint *nextWPT, *prev_WPT;
    OGRLineString *WPT_line;
    OGRGeometry *geom;
    double poly_buff = 5;


    // Parse waypoint string. It is in the format: x,y
    vector<string> wpt_split = parseString(WPT, ',');
    WPT_x = atof(wpt_split[0].c_str());
    WPT_y = atof(wpt_split[1].c_str());

    // Post a polygon to pMarineViewer to show the location of the
    //   current waypoint.
    string print_WPT = "format=radial,x="+wpt_split[0]+",y="+wpt_split[1]+",radius=5,pts=4,edge_size=5,vertex_size=2,edge_color=gold,label=WPT";
    Notify("VIEW_POLYGON", print_WPT);

    // Make GDAL point for the current and previous waypoints
    nextWPT = (OGRPoint*) OGRGeometryFactory::createGeometry(wkbPoint);
    nextWPT->setX(WPT_x); nextWPT->setY(WPT_y);
    curr_WPT = nextWPT;

    prev_WPT = (OGRPoint*) OGRGeometryFactory::createGeometry(wkbPoint);
    prev_WPT->setX(prev_WPT_x); prev_WPT->setY(prev_WPT_y);

    // Make a GDAL Line from the waypoints
    WPT_line = (OGRLineString*) OGRGeometryFactory::createGeometry(wkbLineString);
    WPT_line->addPoint(curr_WPT);
    WPT_line->addPoint(prev_WPT);

    // Make GDAL polygon encapsulating the current and previous
    //  waypoints with a buffer around it
    geom = WPT_line->Buffer(poly_buff);
    *WPT_poly = (OGRPolygon*) geom;
    return WPT_line;
}

void ENC_WPT_Check::WPT_Valid(string WPT)
{
    OGRLineString *WPT_line, *line;
    OGRFeature *feat;
    OGRGeometry *geom, *newGeom;
    OGRPolygon *WPT_poly;
    double x_int, y_int;
    string intercept_pt;
    int i=0;

    check_intersection = false;
    check_within = false;

    // Clear vectors
    X_intersection.clear();
    Y_intersection.clear();
    polyIntersect_Index.clear();

    Set_Prev_WPT();
    WPT_line = BuildWPT(WPT, &WPT_poly);

    layer->ResetReading();
    feat= layer->GetNextFeature();
    while(feat)
    {
        geom = feat->GetGeometryRef();
        if (curr_WPT->Within(geom))
        {
            newGeom = WPT_line->Intersection(geom);
            line = (OGRLineString*)newGeom;

            // The returned line should only have two points: the intersection and the WPT
            if (line->getNumPoints()==2)
            {
                check_within = true;
                // Get the intersection
                x_int = line->getX(1); y_int = line->getY(1);

                X_intersection.push_back(x_int);
                Y_intersection.push_back(y_int);
                polyIntersect_Index.push_back(i);

                // Print it to pMarineViewer
                intercept_pt = "x="+to_string(x_int) + ",y="+to_string(y_int)+",vertex_size=6,vertex_color=pink,active=true,label=intercept_"+to_string(i);
                Notify("VIEW_POINT", intercept_pt);
            }
            else
                cout << "Should have an intersection.." << endl;

            // Check to see if the planned path will intersect the
            //   polygon obstacle
            if (WPT_poly->Intersect(geom))
                check_intersection = true;
        }
        feat = layer->GetNextFeature();
        i++; // Increase counter for the number of polygons

    }


    // Update flags and print out status message
    Notify("INVALID_WPT", check_within);
    Notify("INVALID_PATH", check_intersection);
}
