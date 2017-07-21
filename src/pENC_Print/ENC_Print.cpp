/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: ENC_Print.cpp                                   */
/*    DATE: June 2017                                       */
/************************************************************/

#include "ENC_Print.h"

//---------------------------------------------------------
// Constructor

ENC_Print::ENC_Print()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_tide = 1.5;
  m_MHW_Offset = 2.735; // Value for Fort Point
  m_ASV_draft = 1;

  // Values for the boundary of the portsmouth tiff
  N_lat = 43.07511878;
  E_long = -70.68395689;
  S_lat = 43.05780589;
  W_long = -70.72434189;

  first_print = true;
  m_ENC_INT = false;
  openned = false;
  print_all = false;
  UTM = false;
}

//---------------------------------------------------------
// Destructor

ENC_Print::~ENC_Print()
{
    GDALClose(Point_Layer);
    GDALClose(Poly_Layer);
    GDALClose(Line_Layer);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ENC_Print::OnNewMail(MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;
    string name;
    for(p=NewMail.begin(); p!=NewMail.end(); p++)
    {
        CMOOSMsg &msg = *p;
        name = msg.GetName();
        if (name == "Current_Tide")
        {
            if (msg.IsString())
                vect_tide.push_back(atof(msg.GetString().c_str()));
            else
                vect_tide.push_back(msg.GetDouble());
        }
        else if (name == "MHW_Offset")
            m_MHW_Offset = atof(msg.GetString().c_str());
        else if (name == "ENC_INIT")
            m_ENC_INT = (msg.GetString()=="true");
    }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ENC_Print::OnConnectToServer()
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

bool ENC_Print::Iterate()
{
    if (m_ENC_INT)
    {
        if (!openned)
        {
            openned = openLayers();
            cout << "Opened Files. Starting to print..." << endl;
            first_print = true;
            printPoints();
            printPolygons();
        }

        else if (vect_tide.size()>0)
        {
            first_print = false;
            m_tide = vect_tide.back();
            vect_tide.clear();
            cout << "Tide: " << m_tide << endl;
            printPoints();
            printPolygons();
        }
    }

    return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ENC_Print::OnStartUp()
{
    double dfLatOrigin,dfLongOrigin;
    string sVal;
    list<string> sParams;

    // Parse the parameters from the .moos file
    m_MissionReader.EnableVerbatimQuoting(false);
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams))
    {
        list<string>::iterator p;
        for(p=sParams.begin(); p!=sParams.end(); p++)
        {
            string original_line = *p;
            string param = stripBlankEnds(toupper(biteString(*p, '=')));
            string value = stripBlankEnds(*p);

            if(param == "MHW_OffSET")
                m_MHW_Offset = atof(value.c_str());

            else if (param == "ASV_DRAFT")
                m_ASV_draft = atof(value.c_str());
            else if ((param == "N_LAT")||(param == "N_Y"))
                N_lat = atof(value.c_str());
            else if ((param == "S_LAT")||(param == "S_Y"))
                S_lat = atof(value.c_str());
            else if ((param == "E_LONG")||(param == "E_X"))
                E_long = atof(value.c_str());
            else if ((param == "W_LONG")||(param == "W_X"))
                W_long = atof(value.c_str());
            else if (param == "PRINT_ALL")
                print_all = value=="true";
            else if (param == "UNIT")
                UTM = toupper(value) == "UTM";
        }
    }

    // Get the lat/long origin from the mission file
    if (m_MissionReader.GetValue("LatOrigin", sVal))
        dfLatOrigin = atof(sVal.c_str());
    else
    {
        MOOSTrace("LatOrigin not set - FAIL\n");
        return false;
    }

    if (m_MissionReader.GetValue("LongOrigin", sVal))
        dfLongOrigin = atof(sVal.c_str());
    else
    {
        MOOSTrace("LongOrigin not set - FAIL\n");
        return false;
    }

    // Initialize the geodesy object
    geod.Initialise(dfLatOrigin, dfLongOrigin);

    m_timewarp = GetMOOSTimeWarp();

    RegisterVariables();
    return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void ENC_Print::RegisterVariables()
{
    Register("Current_Tide", 0);
    Register("MHW_Offset", 0);
    Register("ENC_INIT", 0);
}

//---------------------------------------------------------
// Procedure: calc_WL_depth
/*
  This function updates the Water Level attribute with the current
  predicted tide. This is shoal biased. The values used in these
  calculations are from NOAA's Nautical Chart User Manual:

  http://portal.survey.ntua.gr/main/labs/carto/academic/persons/bnakos_site_nafp/documentation/noaa_chart_users_manual.pdf

  Inputs:
  WL - Water level for the obstacle
  (qualitative depth measurement)

  Outputs:
  WL_depth - current depth in with respect to the given WL
*/
double ENC_Print::calc_WL_depth(double WL)
{
    double WL_depth = 0;
    double feet2meters = 0.3048;
    if (WL == 2)
        // At least 1foot above MHW. Being shoal biased, we will take the
        // object's "charted" depth as 1 foot above MHW
        WL_depth = m_tide-(1*feet2meters+m_MHW_Offset);
    else if (WL == 3)
        // At least 1 foot below MLLW. Being shoal biased, we will take the
        //  object's "charted" depth as 1 foot below MLLW
        WL_depth = m_tide+1*feet2meters;
    else if (WL == 4)
        // The range for these attributes 1 foot below MLLW and 1 foot above MHW
        //   Therefore, we will be shoal biased and take 1 foot above MHW as the
        //   object's "charted" depth.
        WL_depth = m_tide-(1*feet2meters+m_MHW_Offset);
    else if (WL == 5)
        // The range for these attributes 1 foot below MLLW and 1 foot above
        //   MLLW. Therefore, we will be shoal biased and take 1 foot above MLLW
        //   as the object's "charted" depth.
        WL_depth = m_tide-1*feet2meters;
    else
        // All other Water levels (1, 6, and 7) don't have a quantitative
        //   descriptions. Therefore we will set it to 0.
        WL_depth = 0;

    return WL_depth;
}

//---------------------------------------------------------
// Procedure: calc_t_lvl
/*
  This function uses the water level and depth attributes for a
  feature and calculates the threat level for that obstacle.

  Inputs:
  depth - Recorded depth for the obstacle (quantitative depth measurement)
  WL - Water level for the obstacle (qualitative depth measurement)

  Outputs:
  t_lvl - Calculated threat level
*/
int ENC_Print::calc_t_lvl(double depth, double WL, string LayerName)
{
    int t_lvl = 0;
    double WL_depth = 0;
    double current_depth = 9999;

    // If it is Land set threat level to 5
    if ((LayerName == "LNDARE")||(LayerName == "DYKCON")||(LayerName == "PONTON")||(LayerName == "COALNE"))
        t_lvl = 5;
    else if (LayerName == "LIGHTS")
        t_lvl = -2;
    // If it is a Buoy or Beacon set threat level to 3
    else if ((LayerName == "BOYISD")||(LayerName == "BOYSPP")||(LayerName == "BOYSAW")||(LayerName == "BOYLAT")||(LayerName == "BCNSPP")||(LayerName == "BCNLAT"))
        t_lvl = 3;
    else if (LayerName == "LNDMRK")
        t_lvl = -1;
    else
    {
        if (depth == 9999)
            current_depth = 9999;
        else
            current_depth = depth+m_tide;

        // Neither the WL or depth are recorded - this is bad
        if ((current_depth == 9999) && (WL == 0))
        {
            cout << "No depth or WL -> Threat Level will be set to 4." << endl;
            t_lvl = 4;
            depth=-2;
        }
        // No Charted Depth
        else if (current_depth == 9999)
        {
            // If there is no depth, use the Water Level attribute to
            //   calculate the threat level. There is no quanitative
            //   description of qualitative WL attribute for IDs 1, 6 and 7.
            //   Therefore, they will print a warning and set the threat
            //   level to 4.
            if ((WL == 1)||(WL == 6)||(WL == 7))
            {
                cout << "Unknown Description of Water Level: " << WL <<
                ", Threat Level will be set to 4." << endl;
                t_lvl = 4;
            }
            else
            {
                depth = calc_WL_depth(WL);
                t_lvl = threat_level(calc_WL_depth(WL));
            }
        }
        // If WL is unknown, use current detpth
        else if (WL == 0)
        t_lvl = threat_level(current_depth);
        // If we have both the WL and the depth, use the depth measurement
        else
        {
          WL_depth = calc_WL_depth(WL);
          t_lvl = threat_level(current_depth);
        }
    }
  return t_lvl;
}

//---------------------------------------------------------
// Procedure: threat_level
/*
  This function uses a depth for a feature (Determined by a sounding
  or realative to the WL attribute) and calculates the threat level
  for that obstacle.

  Inputs:
  depth - Recorded depth for the obstacle (actual or relative to WL)

  Outputs:
  t_lvl - Calculated threat level
*/
int ENC_Print::threat_level(double depth)
{
    int t_lvl = 0;
    // Above the water surface (plus buffer)
    if (depth<=m_ASV_draft*3)
        t_lvl = 4;
    // Near the water surface (plus buffer)
    else if (depth< m_ASV_draft*4)
        t_lvl = 3;
    else if (depth < m_ASV_draft*5)
        t_lvl = 2;
    else if ((depth >=m_ASV_draft*5) && (depth <= m_ASV_draft*7))
        t_lvl = 1;
    // Obstacle is deep
    else
        t_lvl = 0;

  return t_lvl;
}

bool ENC_Print::openLayers()
{
    GDALAllRegister();
    // Open the data sources so that we can use them later in the iterate loop
    ds_pnt = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
    ds_poly = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Poly.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
    ds_line = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Line.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );

    // Check if it worked
    if( ds_pnt == NULL )
    {
        printf( "Point datasource open failed.\n" );
        exit( 1 );
    }
    if( ds_poly == NULL )
    {
        printf( "Poly datasource open failed.\n" );
        exit( 1 );
    }
    if( ds_line == NULL )
    {
        printf( "Line datasource open failed.\n" );
        exit( 1 );
    }

    // Open the layers
    Point_Layer = ds_pnt -> GetLayerByName( "Point" );
    Poly_Layer = ds_poly -> GetLayerByName( "Poly" );
    Line_Layer = ds_line -> GetLayerByName( "Line" );

    if (Point_Layer == NULL)
    {
        cout << "Opening point layer failed." << endl;
        exit( 1 );
    }
    if (Poly_Layer == NULL)
    {
        cout << "Opening poly layer failed." << endl;
        exit( 1 );
    }
    if (Line_Layer == NULL)
    {
        cout << "Opening line layer failed." << endl;
        exit( 1 );
    }

    if (!print_all)
    {
        // Build spatial filter
        OGRPolygon *print_area_filter = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
        OGRLinearRing *ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);

        double W_x, N_y, E_x, S_y;

        if (!UTM)
        {
            // Convert corner positions from lat/long to local UTM
            geod.LatLong2LocalUTM(N_lat, W_long, W_x, N_y);
            geod.LatLong2LocalUTM(S_lat, E_long, E_x, S_y);
        }
        else
        {
            N_y = N_lat;
            S_y = S_lat;
            W_x = W_long;
            E_x = E_long;
        }

        // Build the filter polygon
        ring->addPoint(W_x, N_y);
        ring->addPoint(E_x, N_y);
        ring->addPoint(E_x, S_y);
        ring->addPoint(W_x, S_y);
        ring->addPoint(W_x, N_y);
        ring->closeRings();
        print_area_filter->addRing(ring);
        print_area_filter->closeRings();

        //string s_poly_str = "pts={"+to_string((int)W_x)+","+ to_string((int)N_y)+":"+ to_string((int)E_x)+","+to_string((int)N_y)+":"+ to_string((int)E_x)+","+to_string((int)S_y)+":" +to_string((int)W_x)+","+to_string((int)S_y)+"}";
        //s_poly_str+=",label=Search2,edge_size=10,vertex_size=1,edge_color=blue,active=true";
        //Notify("VIEW_POLYGON", s_poly_str);

        // Set Spatial Filters
        Point_Layer->SetSpatialFilter(print_area_filter);
        Poly_Layer->SetSpatialFilter(print_area_filter);
        Line_Layer->SetSpatialFilter(print_area_filter);
        m_filter = print_area_filter;
    }

    return true;
}

/*
 * Print the points from the ENC that are in the area defined by the
 *  print_area function.
*/
void ENC_Print::printPoints()
{
    OGRPoint *point;
    OGRFeature *feature;
    OGRGeometry *geom;

    int i =0;
    int MLLW_t_lvl, WL, t_lvl, depth;
    string obs_type, color, label, print_pnt, vsize, location, active;

    Point_Layer->ResetReading();
    feature = Point_Layer->GetNextFeature();
    while(feature)
    {
        MLLW_t_lvl = feature->GetFieldAsDouble(0); // Get threat level (@ MLLW)
        WL = feature->GetFieldAsDouble(1);
        depth = feature->GetFieldAsDouble(2);
        obs_type = feature->GetFieldAsString(3);
        t_lvl = calc_t_lvl(depth, WL, obs_type);

        if ((first_print) || (MLLW_t_lvl != t_lvl))
        {
            geom = feature->GetGeometryRef();
            point = (OGRPoint *)geom;
            location = "x="+to_string(point->getX())+",y="+to_string(point->getY())+",";

            setColor(t_lvl, color);
            color = "vertex_color="+color;

            label = ",label="+obs_type+"_"+to_string(i);
            vsize =  "vertex_size=10,";
            //*
            if (t_lvl == 0)
                active = "active=false";
            else
                active = "active=true";
            print_pnt = location+vsize+color+active+label;
            // Print the point
            Notify("VIEW_POINT", print_pnt);
        }
        feature = Point_Layer->GetNextFeature();
        i += 1;
    }
}

/*
 * Print the polygons from the ENC that are in the area defined by the
 *  print_area function.
*/
void ENC_Print::printPolygons()
{
    OGRPolygon *poly;
    OGRLinearRing *ring;
    OGRFeature *feature;
    OGRGeometry *geom;

    int i =0;
    int MLLW_t_lvl, WL, t_lvl, depth, numPoints;
    string obs_type, color, label, print_poly, sizes, vertex, active;

    Poly_Layer->ResetReading();
    feature = Poly_Layer->GetNextFeature();

    while(feature)
    {
        geom = feature->GetGeometryRef();
        geom = geom->Intersection(m_filter);
        MLLW_t_lvl = feature->GetFieldAsDouble(0); // Get threat level (@ MLLW)
        WL = feature->GetFieldAsDouble(1);
        depth = feature->GetFieldAsDouble(2);
        obs_type = feature->GetFieldAsString(3);
        t_lvl = calc_t_lvl(depth, WL, obs_type);

        if ((first_print) || (MLLW_t_lvl != t_lvl))
        {
            poly = (OGRPolygon *)geom;

            ring = poly->getExteriorRing();
            if (ring)
            {
                color = "";
                // Determine how many vertices there are in the polygon
                numPoints = ring->getNumPoints();

                // String to hold the vertices
                vertex = "pts={";

                // Cycle through the vertices and store them as a string
                for (int j=0; j<numPoints; j++)
                {
                    vertex += to_string(ring->getX(j)) + ","+ to_string(ring->getY(j));
                    if (j!=numPoints-1)
                        vertex += ":";
                }
                vertex += "},";

                // Set the edge and vertex color
                setColor(t_lvl, color);
                color = "vertex_color="+color + "edge_color="+color;

                label = ",label="+obs_type+"_"+to_string(i);
                sizes =  "vertex_size=2.5,edge_size=2,";

                if (t_lvl == 0)
                    active = "active=false";
                else
                    active = "active=true";
                print_poly = vertex+sizes+color+active+label;
                // Print the point
                Notify("VIEW_SEGLIST", print_poly);
            }
        }
        feature = Poly_Layer->GetNextFeature();
        i++;
    }

}

void ENC_Print::setColor(int t_lvl, string &color)
{
    // Change the Color of the point based on the Threat Level
    if (t_lvl == 5) //Coast Line
        color = "black,";
    else if (t_lvl == 4)
        color = "red,";
    else if (t_lvl == 3)
        color = "darkorange,";
    else if (t_lvl == 2)
        color = "gold,";
    else if (t_lvl == 1)
        color = "greenyellow,";
    else if (t_lvl == 0)
        color = "green,";
    else if (t_lvl == -1) // Landmark
        color = "violet,";
    else if (t_lvl == -2) // LIGHTS
        color = "cornflowerblue,";
    else
        cout << "ERROR --> Threat Level " << t_lvl << " does not exist!" << endl;
}

