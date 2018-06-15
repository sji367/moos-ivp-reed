/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH CCOM                                        */
/*    FILE: ENC_Contact.cpp                                 */
/*    DATE: 11/22/16                                        */
/************************************************************/

#include "ENC_Contact.h"

using namespace std;
//---------------------------------------------------------
// Constructor

ENC_Contact::ENC_Contact()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_MHW_Offset = 2.735; // Nominal value for Fort Point
  m_tide = 0.65;
  m_ASV_length = 1.8;//4;
  m_ASV_width = 1;
  m_ASV_draft = 1;
  m_search_dist = 50;
  m_min_depth = 1.0;
  m_ENC = "US5NH02M";
  m_speed = 0;
  geod = Geodesy();
  m_max_pnts = 0;
  m_max_poly = 0;
  m_segmentation_dist = 3;
  m_buffer_size = 5;
  m_simplifyPolys = true;
  m_SearchDistSet = false;
  prev_dist = 10*m_ASV_length;
  prev2PolyDist= 10*m_ASV_length;
  bareSteerage = 0.5;
  origDesSpeed = 1;
  newSpeedSet_flag = false;
  prevPolyDist = 10*m_ASV_length;
}

ENC_Contact::~ENC_Contact()
{
    GDALClose( DS_pnt );
    GDALClose( DS_poly );

    // Delete the vector of pointers
    for (OGRPoint* obj: newPoint)
        delete obj;
    newPoint.clear();

    for (OGRPolygon* obj2: newPoly)
        delete obj2;
    newPoly.clear();
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ENC_Contact::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  string name, color;
  double distance;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    name   = msg.GetName();
    if (name == "NAV_X")
        vect_x.push_back(msg.GetDouble());
    else if (name == "ASV_Length")
        m_ASV_length = msg.GetDouble();
    else if (name == "NAV_Y")
        vect_y.push_back(msg.GetDouble());
    else if(name == "NAV_HEADING")
        vect_head.push_back(msg.GetDouble());
    else if (name=="NAV_SPEED")
        m_speed = msg.GetDouble();
    else if (name == "Current_Tide")
        vect_tide.push_back(atof(msg.GetString().c_str()));
    else if (name == "MHW_Offset")
        m_MHW_Offset = atof(msg.GetString().c_str());
    else if (name == "NEW_POINTS")
        parseNewPoint(msg.GetString());
    else if (name == "NEW_POLYS")
        parseNewPoly(msg.GetString());
    else if (name == "CYCLE_INDEX")
    {
        // Print and store the closest the ASV got to any of the obstacles
        if (dist2obstacle.size()>0)
        {
            // Calculate the minimum distance to the obstacle
            distance = *min_element(dist2obstacle.begin(), dist2obstacle.end());
            cout << distance << endl;
            Notify("MIN_DIST", distance);
        }
        // Increase the threat level for the obstacle after each iteration
        if (msg.GetDouble()>0)
        {
            if (fmod(msg.GetDouble(),5)==0)
            {
                    Notify("UPDATE_POLYS", "RESET");
                    Notify("UPDATE_PTS", "RESET");
            }
            else
            {
                Notify("UPDATE_POLYS", "INCREASE");
                Notify("UPDATE_PTS", "INCREASE");
            }

        }
        dist2obstacle.clear();
        cout << "Index: " << msg.GetDouble() << endl;
    }
    else if (name == "UPDATE_PTS")
    {
        if (pointTLvl.size() > 0)
        {
            for (int i=0; i<pointTLvl.size(); i++)
            {
                // Update the threat level
                if(msg.GetString()=="INCREASE")
                    pointTLvl[i]++;
                else if(msg.GetString()=="DECREASE")
                    pointTLvl[i]--;
                else if(msg.GetString()=="RESET")
                    pointTLvl[i]=1;

                // Make sure it is in the domain [-2 to 5]
                if (pointTLvl[i] < -2)
                    pointTLvl[i] = -2;
                else if (pointTLvl[i] > 5)
                    pointTLvl[i] = 5;

                // Update the color on pMarineViewer
                setColor(pointTLvl[i], color);
                Notify("VIEW_POINT", "x="+to_string(newPoint[i]->getX()) +",y="+ to_string(newPoint[i]->getY())+",vertex_size=8,vertex_color="+color+",label="+pointLabels[i]);

            }
        }
    }
    else if (name == "UPDATE_POLYS")
    {
        if (polyTLvl.size() > 0)
        {
            for (int i=0; i<polyTLvl.size(); i++)
            {
                // Update the threat level
                if(msg.GetString()=="INCREASE")
                    polyTLvl[i]++;
                else if(msg.GetString()=="DECREASE")
                    polyTLvl[i]--;
                else if(msg.GetString()=="RESET")
                    polyTLvl[i]=1;

                // Make sure it is in the domain [-2 to 5]
                if (polyTLvl[i] < -2)
                    polyTLvl[i] = -2;
                else if (polyTLvl[i] > 5)
                    polyTLvl[i] = 5;

                // Update the color on pMarineViewer
                setColor(polyTLvl[i], color);
                MOOS_polygons[i].set_param("vertex_color",color);
                MOOS_polygons[i].set_param("edge_color",color);
                Notify("VIEW_POLYGON",MOOS_polygons[i].get_spec("label"));
            }
        }
    }
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ENC_Contact::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ENC_Contact::Iterate()
{
    if (m_iterations == 0)
        Notify("ENC_INIT", "true");

    if (vect_head.size() > 0)
    {
        m_ASV_head = vect_head.back();
        vect_head.clear();
    }

    if ((vect_x.size() > 0) && (vect_y.size() > 0))
    {
        // Get new values for the new X, Y and heading of the ASV
        m_ASV_x = vect_x.back();
        m_ASV_y = vect_y.back();

        vect_x.clear();
        vect_y.clear();

        // Clear old tlvl vectors
        vect_TLvl.clear();

        // need to add a global point, polygon and line geometry
        build_search_poly();
        filter_feats();
        publish_points();
        publish_poly_360();

        // Update the lead parameter and desired speed if necessary
        if (vect_TLvl.size()>0)
            Update_Lead_Param();
        else
            Notify("WPT_UPDATE", "lead=8");

        UpdateSpeed();

    }
    if (vect_tide.size() > 0 )
    {
        m_tide = vect_tide.back();
        vect_tide.clear();
    }
    m_iterations++;
    return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
bool ENC_Contact::OnStartUp()
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

            if(param == "ENCS")
                m_ENC = value;
            else if(param == "MHW_OffSET")
                m_MHW_Offset = atof(value.c_str());
            else if(param == "SEARCH_DIST")
            {
                // If the search distance is set, then use the user suppplied distance. Otherwise use 10*(vessel length).
                m_SearchDistSet = true;
                m_search_dist = atof(value.c_str());
            }
            else if(param == "SEGMENTATION_DIST")
                m_segmentation_dist = atof(value.c_str());
            else if(param == "BUFFER_DIST")
                m_buffer_size = atof(value.c_str());
            else if(param == "MIN_DEPTH")
                m_min_depth = atof(value.c_str());
            else if(param == "UNION")
                m_simplifyPolys = tolower(value)=="true";
            else if(param  == "DESIRED_SPEED")
                origDesSpeed = atof(value.c_str());

            // ASV Configuration
            else if (param == "ASV_LENGTH")
            {
                m_ASV_length = atof(value.c_str());
                prev_dist = 10*m_ASV_length;
                prevPolyDist = 10*m_ASV_length;
                prev2PolyDist= 10*m_ASV_length;
            }

            else if (param == "ASV_WIDTH")
                m_ASV_width = atof(value.c_str());

            else if (param == "ASV_DRAFT")
                m_ASV_draft = atof(value.c_str());

            else if (param == "BARE_STEERAGE")
                bareSteerage = atof(value.c_str());
        }
    }

    string sVal;

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

    if (!m_Geodesy.Initialise(dfLatOrigin, dfLongOrigin))
    {
        MOOSTrace("Geodesy Init failed - FAIL\n");
        return false;
    }

    geod.Initialise(dfLatOrigin, dfLongOrigin);

    m_timewarp = GetMOOSTimeWarp();
    RegisterVariables();

    // Build the ENC Layers
    BuildLayers();
    cout << "Initialized" << endl;

    return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void ENC_Contact::RegisterVariables()
{
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("ASV_Length",0);
    Register("NAV_HEADING", 0);
    Register("NAV_SPEED", 0);
    Register("Current_Tide",0);
    Register("MHW_Offset",0);
    Register("NEW_POINTS",0);
    Register("NEW_POLYS",0);
    Register("UPDATE_PTS", 0);
    Register("UPDATE_POLYS", 0);
    Register("CYCLE_INDEX", 0);
}

void ENC_Contact::setColor(int t_lvl, string &color)
{
    // Change the Color of the point based on the Threat Level
    if (t_lvl == 5) //Coast Line
        color = "black";
    else if (t_lvl == 4)
        color = "red";
    else if (t_lvl == 3)
        color = "darkorange";
    else if (t_lvl == 2)
        color = "gold";
    else if (t_lvl == 1)
        color = "greenyellow";
    else if (t_lvl == 0)
        color = "green";
    else if (t_lvl == -1) // Landmark
        color = "violet";
    else if (t_lvl == -2) // LIGHTS
        color = "cornflowerblue";
    else
        cout << "ERROR --> Threat Level " << t_lvl << " does not exist!" << endl;
}

// Function for adding a new point to avoid. These messages will be passed by pMarineViewer
//  and will contain a string in the format: x=$(XPOS),y=$(YPOS),$(t_lvl),$(label)
//  This string is parsed and an OGRPoint object is created. The points can be removed if
//  pointSting contains the value "CLEARALL"
void ENC_Contact::parseNewPoint(string pointString)
{
    // The string is in the format:
    //  x=$(XPOS),y=$(YPOS),$(t_lvl),$(label)
    vector<string> parsed, x, y;

    // Remove all points from the
    if (toupper(pointString) == "CLEARALL")
    {
        // Remove the point from pMarineViewer
        for(auto i:pointLabels)
            Notify("VIEW_POINT", "x=1,y=1, active=false, label="+i);
        // Delete the pointer and clear the vector holding the points
        for (OGRPoint* obj: newPoint)
            delete obj;
        newPoint.clear();
        pointTLvl.clear();
        pointLabels.clear();

        // Remove the points on pMarineViewer
    }
    else
    {
        // Parse the string
        parsed = parseString(pointString,",");
        if (parsed.size() == 4)
        {
            x = parseString(parsed[0],"=");
            y = parseString(parsed[1],"=");

            cout << parsed[0] << ", " << parsed[1] << endl;

            // Add the point to the vector
            newPoint.push_back(new OGRPoint(atof(x[1].c_str()), atof(y[1].c_str())));
            pointTLvl.push_back(atof(parsed[2].c_str()));
            pointLabels.push_back(parsed[3]);
        }
        else
            cout << "ERROR --> New Point: " << pointString << endl;
    }
}

// Function for adding a new polygon to avoid. These messages will be passed by pMarineViewer
//  and will contain a string in the format of an XYPolygon.This string is parsed and an OGRPoint
//  object is created. The points can be removed if polySting contains the value "CLEARALL"
void ENC_Contact::parseNewPoly(string polyString)
{
    // The string is in the format of VIEW_POLY, where the label gives
    //  the threat level (label=!$(t_lvl)!$(XPOS)!$(YPOS))
    vector<string> parseT_lvl;

    if (toupper(polyString) == "CLEARALL")
    {
        // Delete the pointer and clear the vector holding the polygons
        for (OGRPolygon* obj: newPoly)
            delete obj;
        newPoly.clear();
        polyTLvl.clear();

        // Remove the polygon from pMarineViewer
        for(auto i:MOOS_polygons)
            Notify("VIEW_POLYGON",i.get_spec("active=false"));
        MOOS_polygons.clear();
    }
    else
    {
        // Parse the string to build an XYPolygon
        XYPolygon MOOS_poly = string2Poly(polyString);
        MOOS_polygons.push_back(MOOS_poly);

        // parse the specs to get the threat level of the polygon
        parseT_lvl = parseString(MOOS_poly.get_spec("label"),"!");
        int t_lvl = atoi(parseT_lvl[1].c_str());
        polyTLvl.push_back(t_lvl);

        // Build the new OGRPolygon object
        OGRPolygon *poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
        OGRGeometry *geomBuff;
        OGRPolygon *polyBuff = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
        OGRLinearRing *ring = (OGRLinearRing*) OGRGeometryFactory::createGeometry(wkbLinearRing);

        // Add all of the vertices to the ring that decribes the polygon
        for (unsigned int i = 0; i<MOOS_poly.size(); i++)
        {
            ring->addPoint(MOOS_poly.get_vx(i), MOOS_poly.get_vy(i)+0.5);
            //cout << MOOS_poly.get_vx(i) << ", " << MOOS_poly.get_vy(i) <<"; ";
        }
        cout << endl;
        ring->closeRings();
        poly->addRing(ring);
        poly->closeRings();

        // Segmentize the polygon to m_segmentation_dist meters between the vertices and store it
        poly->segmentize(m_segmentation_dist);
        //geomBuff=poly->Buffer(calcBuffer(t_lvl));
        //polyBuff=(OGRPolygon*)geomBuff;
        ring = poly->getExteriorRing();//Buff->getExteriorRing();
        for (unsigned int i = 0; i<ring->getNumPoints(); i++)
        {
            cout << ring->getX(i) << ", " << ring->getY(i) <<"; ";
        }
        cout << endl;
        newPoly.push_back(poly);//Buff);
    }
}

//---------------------------------------------------------
// Procedure: category_lights
/*
  This function takes in an index relating to a category of the light 
  and converts it to a human readable string.
        
  Inputs: 
  Index - Index describing which category the light is a part of
            
  Outputs:
  A string that holds the information from the category of light 
  attribute from the ENC
*/
string ENC_Contact::category_lights(int index)
{
  switch(index) // Field # 12
    {
    case 1:
      return "Directional Function";
    case 4:
      return "Leading";
    case 5:
      return "Aero";
    case 6:
      return "Air Obstruction";
    case 7:
      return "Fog Detector";
    case 8:
      return "Flood";
    case 9:
      return "Strip";
    case 10:
      return "Subsidiary";
    case 11:
      return "Spot";
    case 12:
      return "Front";
    case 13:
      return "Rear";
    case 14:
      return "Lower";
    case 15:
      return "Upper";
    case 16:
      return "Moire Effect";
    case 17:
      return "Emergency";
    case 18:
      return "Bearing";
    case 19:
      return "Horizontally Disposed";
    case 20:
      return "Vertically Disposed";
    default:
      return "Marine";
    }
}

//---------------------------------------------------------
// Procedure: category_landmark
/*
  This function takes in an index relating to a category of the  
  landmark and converts it to a human readable string.
        
  Inputs: 
  Index - Index describing which category the landmark is a part of
            
  Outputs:
  A string that holds the information from the category of landmark 
  attribute from the ENC
*/
string ENC_Contact::category_landmark(int index)
{
  switch(index) // Field # 11
    {
    case 1:
      return "Cairn";
    case 2:
      return "Cemetery";
    case 3:
      return "Chimney";
    case 4:
      return "Dish Aerial";
    case 5:
      return "Flagstaff";
    case 6:
      return "Flare Stack";
    case 7:
      return "Mast";
    case 8:
      return "Mast";
    case 9:
      return "Windsock";
    case 10:
      return "Monument";
    case 11:
      return "Memorial Plaque";
    case 12:
      return "Obelisk";
    case 13:
      return "Statue";
    case 14:
      return "Cross";
    case 15:
      return "Dome";
    case 16:
      return "Radar Scanner";
    case 17:
      return "Tower";
    case 18:
      return "Windmill";
    case 19:
      return "Windmotor";
    case 20:
      return "Spire";
    default:
      return "Unknown Landmark";
    }
}

//---------------------------------------------------------
// Procedure: category_silo
string ENC_Contact::category_silo(int index)
{
  /*
    This function takes in an index relating to a category of the silo 
    and converts it to a human readable string.
        
    Inputs: 
    Index - Index describing which category the silo is a part of
            
    Outputs:
    A string that holds the information from the category of silo 
    attribute from the ENC
  */
  switch(index) // field # 12
    {
    case 1:
      return "Silo";
    case 2:
      return "Tank";
    case 3:
      return "Grain Elevator";
    case 4:
      return "Water Tower";
    default:
      return "Unknown Silo";
    }
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
double ENC_Contact::calc_WL_depth(double WL)
{
  double WL_depth = 0;
  double feet2meters = 0.3048;
  if (WL == 2)
      // At least 1 foot above MHW. Being shoal biased, we will take the
      // object's "charted" depth as 2 foot above MHW
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
int ENC_Contact::calc_t_lvl(double &depth, double WL, string LayerName)
{
  int t_lvl = 0;
  double WL_depth = 0;
  double current_depth = 9999;
  
  // If it is Land set threat level to 5
  if ((LayerName == "LNDARE")||(LayerName == "DYKCON")||(LayerName == "PONTON")||(LayerName == "COALNE"))
  {
    t_lvl = 5;
    depth = round(-m_MHW_Offset-2*100)/100; // Make sure it only has a precision of two
  }
  else if (LayerName == "GRID")
      t_lvl = 4;
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
      depth = current_depth;
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
	  else {
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
int ENC_Contact::threat_level(double depth)
{
  int t_lvl = 0;
  // Above the water surface (plus buffer)
  if (depth<=m_ASV_draft*3.0)
    t_lvl = 4;
  // Near the water surface (plus buffer)
  else if (depth< m_ASV_draft*4.0)
    t_lvl = 3;
  else if (depth < m_ASV_draft*5.0)
    t_lvl = 2;
  else if (depth <= m_ASV_draft*6.0)
    t_lvl = 1;
  // Obstacle is deep
  else
    t_lvl = 0;

  return t_lvl;
}

//---------------------------------------------------------
// Procedure:
/*
  Create a OGR layer for the point obstacles, polygon obstacles and  
  the line obstacles. Then open them so that can be used in the rest
  of the application.
        
  Outputs (these are not returned, but defined as self.):
   Point_Layer - OGR layer that holds all the information from the 
    ENC that have point geometry
   Poly_Layer - OGR layer that holds all the information from the 
    ENC that have polygon geometry
*/
void ENC_Contact::BuildLayers()
{
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    GDALDataset *ds_pnt, *ds_poly, *ds_ENC, *ds_grid;
    OGRLayer *PointLayer, *PolyLayer, *GridLayer;
    string ENC_filename;


    // Build the grid and interp. Then make a binary grid (based on the desired minimum depth)and polygonize it
    ENC_Polygonize polygonize = ENC_Polygonize("../../", "PostProcess"+m_ENC +".tiff", "raster.shp",dfLatOrigin, dfLongOrigin, m_min_depth);
    polygonize.runWithGrid(m_ENC, 5, calcBuffer(4), m_MHW_Offset, true);

    string gridFilename = "../../src/ENCs/Grid/raster.shp";

    ds_grid = (GDALDataset*) GDALOpenEx( gridFilename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds_grid == NULL )
    {
        printf( "Open grid shp file failed.\n" );
        exit( 1 );
    }

    // Create the shapefile
    ds_pnt = poDriver->Create( "../../src/ENCs/Shape/Point.shp", 0, 0, 0, GDT_Unknown, NULL );
    if( ds_pnt == NULL )
      {
	printf( "Creation of output file failed.\n" );
	exit( 1 );
      }
    ds_poly = poDriver->Create( "../../src/ENCs/Shape/Poly.shp", 0, 0, 0, GDT_Unknown, NULL );
    if( ds_poly == NULL )
      {
	printf( "Creation of output file failed.\n" );
	exit( 1 );
      }
    // Create the layers (point and polygon)
    PointLayer = ds_pnt->CreateLayer( "Point", NULL, wkbPoint, NULL );
    if( PointLayer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    PolyLayer = ds_poly->CreateLayer( "Poly", NULL, wkbPolygon, NULL );
    if( PointLayer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    
    // Create the fields for the layer
    OGRFieldDefn oField_tlvl( "T_lvl", OFTInteger);
    OGRFieldDefn oField_WL( "WL", OFTReal);
    OGRFieldDefn oField_depth( "Depth", OFTReal);
    OGRFieldDefn oField_type( "Type", OFTString);
    OGRFieldDefn oField_cat( "Cat", OFTString);
    OGRFieldDefn oField_visual( "Visual", OFTInteger);
    oField_type.SetWidth(6);
    oField_cat.SetWidth(25);
    // Make the point layer fields
    if( PointLayer->CreateField( &oField_tlvl ) != OGRERR_NONE )
    {
        printf( "Creating Threat Level field failed.\n" );
        exit( 1 );
    }
    if( PointLayer->CreateField( &oField_WL ) != OGRERR_NONE )
    {
        printf( "Creating WL field failed.\n" );
        exit( 1 );
    }
    if(PointLayer->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    if( PointLayer->CreateField( &oField_type ) != OGRERR_NONE )
    {
        printf( "Creating Type field failed.\n" );
        exit( 1 );
    }    
    if( PointLayer->CreateField( &oField_cat ) != OGRERR_NONE )
    {
        printf( "Creating Cat field failed.\n" );
        exit( 1 );
    }
    if( PointLayer->CreateField( &oField_visual ) != OGRERR_NONE )
    {
        printf( "Creating visual field failed.\n" );
        exit( 1 );
    }
    // Make the polygon layer fields
    if( PolyLayer->CreateField( &oField_tlvl ) != OGRERR_NONE )
    {
        printf( "Creating Threat Level field failed.\n" );
        exit( 1 );
    }
    if( PolyLayer->CreateField( &oField_WL ) != OGRERR_NONE )
    {
        printf( "Creating WL field failed.\n" );
        exit( 1 );
    }
    if( PolyLayer->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    if( PolyLayer->CreateField( &oField_type ) != OGRERR_NONE )
    {
        printf( "Creating Type field failed.\n" );
        exit( 1 );
    }
    if( PolyLayer->CreateField( &oField_cat ) != OGRERR_NONE )
    {
        printf( "Creating Cat field failed.\n" );
        exit( 1 );
    }
    if( PolyLayer->CreateField( &oField_visual ) != OGRERR_NONE )
    {
        printf( "Creating visual field failed.\n" );
        exit( 1 );
    }

    int i = 0;
    // Get the ENC
    ENC_filename= "../../src/ENCs/"+m_ENC+"/"+m_ENC+".000";
    ds_ENC = (GDALDataset*) GDALOpenEx( ENC_filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds_ENC == NULL )
      {
        printf( "Open failed.\n" );
        exit( 1 );
      }
    else
      cout << "Opened "<< m_ENC << endl;

    // Points only
    //LayerMultiPoint(ds_ENC->GetLayerByName("SOUNDG"), PointLayer, "SOUNDG"); // Soundings
    //ENC_Converter(ds_ENC->GetLayerByName("LNDMRK"), PointLayer, PolyLayer, "LNDMRK"); // Landmarks
    //ENC_Converter(ds_ENC->GetLayerByName("SILTNK"), PointLayer, PolyLayer, "SILTNK"); // Silos/Tanks
    ENC_Converter(ds_ENC->GetLayerByName("LIGHTS"), PointLayer, PolyLayer, "LIGHTS"); // Lights
    ENC_Converter(ds_ENC->GetLayerByName("BOYSPP"), PointLayer, PolyLayer, "BOYSPP"); // Special Purpose Buoy
    ENC_Converter(ds_ENC->GetLayerByName("BOYISD"), PointLayer, PolyLayer, "BOYISD"); // Isolated Danger Buoy
    ENC_Converter(ds_ENC->GetLayerByName("BOYSAW"), PointLayer, PolyLayer, "BOYSAW"); // Safe water Buoy
    ENC_Converter(ds_ENC->GetLayerByName("BOYLAT"), PointLayer, PolyLayer, "BOYLAT"); // Lateral Buoy
    ENC_Converter(ds_ENC->GetLayerByName("BCNSPP"), PointLayer, PolyLayer, "BCNSPP"); // Special Purpose Beacon
    ENC_Converter(ds_ENC->GetLayerByName("BCNLAT"), PointLayer, PolyLayer, "BCNLAT"); // Lateral Beacon
    ENC_Converter(ds_ENC->GetLayerByName("UWTROC"), PointLayer, PolyLayer, "UWTROC"); // Underwater Rocks
    ENC_Converter(ds_ENC->GetLayerByName("WATTUR"), PointLayer, PolyLayer, "WATTUR"); // Water Turbulence
    ENC_Converter(ds_ENC->GetLayerByName("WEDKLP"), PointLayer, PolyLayer, "WEDKLP"); // Weeds/Kelp
    ENC_Converter(ds_ENC->GetLayerByName("TOPMAR"), PointLayer, PolyLayer, "TOPMAR"); // Top Mark
    ENC_Converter(ds_ENC->GetLayerByName("DAYMAR"), PointLayer, PolyLayer, "DAYMAR"); // Day Mark
    ENC_Converter(ds_ENC->GetLayerByName("PILPNT"), PointLayer, PolyLayer, "PILPNT"); // Piles

    // Other Types
    ENC_Converter(ds_ENC->GetLayerByName("LNDARE"), PointLayer, PolyLayer, "LNDARE"); // Land
    ENC_Converter(ds_ENC->GetLayerByName("PONTON"), PointLayer, PolyLayer, "PONTON"); // Pontoons
    ENC_Converter(ds_ENC->GetLayerByName("FLODOC"), PointLayer, PolyLayer, "FLODOC"); // Floating Docks
    ENC_Converter(ds_ENC->GetLayerByName("DYKCON"), PointLayer, PolyLayer, "DYKCON"); // Dykes
    ENC_Converter(ds_ENC->GetLayerByName("WRECKS"), PointLayer, PolyLayer, "WRECKS"); // Wrecks
    ENC_Converter(ds_ENC->GetLayerByName("OBSTRN"), PointLayer, PolyLayer, "OBSTRN"); // Obstructions

    // Depth area and depth contours give the same infomation except the areas are polygons and
    //  contours are line segments
    //ENC_Converter(ds_ENC->GetLayerByName("DEPARE"), PointLayer, PolyLayer, "DEPARE");  // Depth Areas
    //ENC_Converter(ds_ENC->GetLayerByName("DEPCNT"), PointLayer, PolyLayer, "DEPCNT");  // Depth contours
    StoreShallowPolys(ds_grid->GetLayer(0), PolyLayer);
    // close the data sources - need this to save the new files
    GDALClose( ds_ENC );
    GDALClose( ds_pnt );
    GDALClose( ds_poly );
    GDALClose(ds_grid);


    //*/

    // Reopen the data source so that we can use them later in the iterate loop
    DS_pnt = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
    DS_poly = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Poly.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
      
    // Check if it worked
    if( DS_pnt == NULL )
    {
	printf( "Open failed.\n" );
	exit( 1 );
    }
    if( DS_poly == NULL )
    {
	printf( "Open failed.\n" );
	exit( 1 );
    }

    // Open the layers
    Point_Layer = DS_pnt -> GetLayerByName( "Point" );
    Poly_Layer = DS_poly -> GetLayerByName( "Poly" );

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
}

void ENC_Contact::StoreShallowPolys(OGRLayer *layer, OGRLayer *PolyLayer)
{
    OGRFeature *feat, *new_feat;
    OGRFeatureDefn *poFDefn;
    OGRGeometry *geom;
    OGRPolygon *poly, *UTM_poly;
    OGRLinearRing *ring, *UTM_ring;//, *innerRing, *UTM_innerRing;
    double x,y;
    int binaryType;

    layer->ResetReading();
    layer->SetAttributeFilter("Depth=1");
    feat = layer->GetNextFeature();
    while(feat)
    {
        binaryType = feat->GetFieldAsDouble("Depth");
        if (binaryType == 1);
        {
            geom = feat->GetGeometryRef();
            poly = ( OGRPolygon * )geom;
            //geom = geom->Buffer(calcBuffer(4));

            // Build the new UTM ring and polygon
            UTM_ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
            UTM_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);

            /*
            innerRing=poly->getInteriorRing(0);
            if (innerRing)
            {
                for (int k=0; k<ring->getNumPoints(); k++)
                {
                    x = innerRing->getX(k)-geod.getXOrigin();
                    y = innerRing->getY(k)-geod.getYOrigin();
                    UTM_innerRing->addPoint(x,y,0);
                }
            }
            UTM_innerRing->closeRings();
            UTM_poly->addRing(UTM_innerRing);
            */

            ring = poly->getExteriorRing();
            for (int j=0; j<ring->getNumPoints(); j++)
            {
                x = ring->getX(j)-geod.getXOrigin();
                y = ring->getY(j)-geod.getYOrigin();
                UTM_ring->addPoint(x,y,0);
            }

            // Build the UTM polygon from the ring
            UTM_ring->closeRings();
            UTM_poly->addRing(UTM_ring);
            UTM_poly->closeRings();

            UTM_poly->segmentize(m_segmentation_dist);

            // Add the polygon to the inputted layer
            poFDefn = PolyLayer->GetLayerDefn();
            new_feat =  OGRFeature::CreateFeature(poFDefn);
            new_feat->SetField("Depth", 0);
            new_feat->SetField("WL", 0);
            new_feat->SetField("T_lvl", 4);
            new_feat->SetField("Type", "GRID");
            new_feat->SetGeometry(UTM_poly);

            // Build the new feature
            if( PolyLayer->CreateFeature( new_feat ) != OGRERR_NONE )
            {
                printf( "Failed to create feature in shapefile.\n" );
                exit( 1 );
            }
        }
        feat = layer->GetNextFeature();
    }
}

//---------------------------------------------------------
// Procedure:LayerMultiPoint
/*
  Adds the features from the inputed multipoints layer to a point 
  layer.
        
  Inputs:
  LayerName_mp - Name of the multipoint layer
*/
void ENC_Contact::LayerMultiPoint(OGRLayer *layer_mp, OGRLayer *PointLayer, string LayerName_mp)
{
  OGRFeature *feat_mp, *new_feat;
  OGRFeatureDefn *feat_def;
  OGRGeometry *geom, *poPointGeometry;
  OGRPoint *poPoint, pt;
  OGRMultiPoint *poMultipoint;
  double depth = 9999;
  int num_geom = 0;
  int WL = 0;
  double x,y, lat, lon;
  if (layer_mp != NULL)
      {
	PointLayer->ResetReading();
	layer_mp->ResetReading();
	feat_def = PointLayer->GetLayerDefn();
	while( (feat_mp = layer_mp->GetNextFeature()) != NULL )
	  {
	    geom = feat_mp->GetGeometryRef();
	    poMultipoint = ( OGRMultiPoint * )geom;
	    num_geom = poMultipoint->getNumGeometries();
	    for(int iPnt = 0; iPnt < num_geom; iPnt++ )
	      {
		
		depth = 9999;
		// Make the point
		poPointGeometry = poMultipoint ->getGeometryRef(iPnt);
		poPoint = ( OGRPoint * )poPointGeometry;
		
		// Get the x,y, and depth in UTM
		lon = poPoint->getX();
		lat = poPoint->getY();
		m_Geodesy.LatLong2LocalUTM(lat,lon,y,x);
		depth = poPoint->getZ();
		
		pt.setX(x);
		pt.setY(y);
		
		new_feat =  OGRFeature::CreateFeature(feat_def);
		new_feat->SetField("Depth", depth);
		new_feat->SetField("T_lvl", calc_t_lvl(depth, WL, LayerName_mp));
		new_feat->SetField("WL", 0);
		new_feat->SetField("Type", LayerName_mp.c_str());
		new_feat->SetGeometry( &pt );

		if( PointLayer->CreateFeature( new_feat ) != OGRERR_NONE )
		  {
		    printf( "Failed to create feature in shapefile.\n" );
		    exit( 1 );
		  }

		OGRFeature::DestroyFeature( new_feat );
	      }
	  }
      }
  else
    cout << "Layer "<< LayerName_mp << " did not open correctly" << endl;
}

//---------------------------------------------------------
// Procedure: ENC_Converter
/*
  This function converts the inputed layer from the ENC to a 
  shapefile layer if the layer is in the ENC. If the inputted layer 
  is not in the ENC, the layer is skipped and the function prints a 
  warning to the user.
*/
void ENC_Contact::ENC_Converter(OGRLayer *Layer_ENC, OGRLayer *PointLayer, OGRLayer *PolyLayer, string LayerName)
{
    OGRFeature *poFeature, *new_feat;
    OGRFeatureDefn *poFDefn, *poFDefn_ENC;
    OGRFieldDefn *poFieldDefn;
    OGRGeometry *geom, *UTM_geom, *buff_geom;
    OGRPoint *poPoint,*vertex, pt;
    OGRPolygon *poPoly, *UTM_poly, *buff_poly;
    OGRLineString *poLine, *UTM_line;
    OGRLinearRing *ring, *UTM_ring;

    double x,y, lat,lon;
    double WL;
    int vis;
    int i=0;
    double t_lvl = 0;
    double depth = 9999;
    int iField = 0;
    string name = "";
    string cat;
    string a_filter = "DRVAL2<"+to_string(m_min_depth);
    string depth_str;

    if (Layer_ENC != NULL)
    {
        if (LayerName=="DEPARE")
        {
            // Only store depth areas of with a maximum depth of less than 1
            Layer_ENC->SetAttributeFilter(a_filter.c_str());
        }
        poFDefn_ENC = Layer_ENC->GetLayerDefn();
        Layer_ENC->ResetReading();

        while( (poFeature = Layer_ENC->GetNextFeature()) != NULL )
        {
            geom = poFeature->GetGeometryRef();

            t_lvl = 10;
            WL = 0;
            vis = 0;
            cat = "";
            for( iField = 0; iField < poFDefn_ENC->GetFieldCount(); iField++ )
            {
                poFieldDefn = poFDefn_ENC->GetFieldDefn(iField);
                name = poFieldDefn->GetNameRef();
                if (name == "WATLEV"){
                    WL = poFeature->GetFieldAsDouble(iField);
            }
            else if ((name == "VALSOU")||(name == "VALDCO"))
            {
                depth_str = poFeature->GetFieldAsString(iField);
                if (!(depth_str.empty()))
                    depth = poFeature->GetFieldAsDouble(iField);
                else
                    depth = 9999;
            }
            else if (name == "CONVIS")
                vis = 2-((int)round(poFeature->GetFieldAsDouble(iField)));
            else if (name == "CATLIT")
                cat = category_lights((int)round(poFeature->GetFieldAsDouble(iField)));
            else if (name == "CATLMK")
                cat = category_landmark((int)round(poFeature->GetFieldAsDouble(iField)));
            else if (name == "CATSIL")
                cat = category_silo((int)round(poFeature->GetFieldAsDouble(iField)));
            }
	  
            if (LayerName=="DEPCNT")
            {
                depth_str = poFeature->GetFieldAsString("VALDCO");
                if (!(depth_str.empty()))
                    depth = poFeature->GetFieldAsDouble("VALDCO"); // Contour's depth
                else
                    depth = 9999;
            }
            if (LayerName=="DEPARE")
            {
                depth_str = poFeature->GetFieldAsString("DRVAL1");
                if (!(depth_str.empty()))
                    depth = poFeature->GetFieldAsDouble("DRVAL1"); // Minimum depth of the depth area
                else
                    depth = 9999;
            }
	  
            t_lvl = calc_t_lvl(depth, WL, LayerName);

            if (geom ->getGeometryType()  == wkbPoint)
            {
                // Set attributes of the new feature
                poFDefn = PointLayer->GetLayerDefn();
                new_feat =  OGRFeature::CreateFeature(poFDefn);

                new_feat->SetField("Depth", depth);
                new_feat->SetField("WL", WL);
                new_feat->SetField("T_lvl", t_lvl);
                new_feat->SetField("Type", LayerName.c_str());
                new_feat->SetField("Cat", cat.c_str());
                new_feat->SetField("Visual", vis);

                // Get the old point from the ENC
                poPoint = ( OGRPoint * )geom;

                // Convert lat/long to UTM
                lon = poPoint->getX();
                lat = poPoint->getY();
                m_Geodesy.LatLong2LocalUTM(lat,lon,y,x);

                // Make the new point in UTM
                pt.setX(x);
                pt.setY(y);
                new_feat->SetGeometry( &pt);

                // Build the new feature
                if( PointLayer->CreateFeature( new_feat ) != OGRERR_NONE )
                {
                    printf( "Failed to create feature in shapefile.\n" );
                    exit( 1 );
                }
            }
            else if(geom ->getGeometryType() == wkbPolygon)
            {
                // Set attributes of the new feature
                poFDefn = PolyLayer->GetLayerDefn();
                new_feat =  OGRFeature::CreateFeature(poFDefn);
                new_feat->SetField("Depth", depth);
                new_feat->SetField("WL", WL);
                new_feat->SetField("T_lvl", t_lvl);
                new_feat->SetField("Type", LayerName.c_str());
                new_feat->SetField("Cat", cat.c_str());
                new_feat->SetField("Visual", vis);

                poPoly = ( OGRPolygon * )geom;
                ring = poPoly->getExteriorRing();
                // Build the new UTM ring and polygon
                UTM_ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
                UTM_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
                for (int j=0; j<ring->getNumPoints(); j++)
                {
                    //x = ring->getX(j)-geod.getXOrigin();
                    //y = ring->getY(j)-geod.getYOrigin();
                    lon = ring->getX(j);
                    lat = ring->getY(j);

                    m_Geodesy.LatLong2LocalUTM(lat,lon,y,x);
                    UTM_ring->addPoint(x,y,0);
                }
                // Build the UTM polygon from the ring
                UTM_ring->closeRings();
                UTM_poly->addRing(UTM_ring);
                UTM_poly->closeRings();

                // Buffer and segmentize the polygon
                buff_geom = UTM_poly->Buffer(calcBuffer(t_lvl));
                buff_poly = ( OGRPolygon * )buff_geom;
                buff_poly = check4Union(buff_poly, PolyLayer, depth, LayerName);
                buff_poly->segmentize(m_segmentation_dist);

                new_feat->SetGeometry(buff_poly);

                // Build the new feature
                if( PolyLayer->CreateFeature( new_feat ) != OGRERR_NONE )
                {
                    printf( "Failed to create feature in shapefile.\n" );
                    exit( 1 );
                }
            }
            else if(geom ->getGeometryType() == wkbLineString)
            {
                // Set attributes of the new feature
                poFDefn = PolyLayer->GetLayerDefn();
                new_feat =  OGRFeature::CreateFeature(poFDefn);
                new_feat->SetField("Depth", depth);
                new_feat->SetField("WL", WL);
                new_feat->SetField("T_lvl", t_lvl);
                new_feat->SetField("Type", LayerName.c_str());
                new_feat->SetField("Cat", cat.c_str());
                new_feat->SetField("Visual", vis);

                poLine = ( OGRLineString * )geom;
                UTM_line = (OGRLineString *)OGRGeometryFactory::createGeometry(wkbLineString);
                // Make all lines in UTM not lat/long
                for (int j=0; j<poLine->getNumPoints(); j++)
                {
                    // Convert to UTM
                    lon = poLine->getX(j);
                    lat = poLine->getY(j);
                    m_Geodesy.LatLong2LocalUTM(lat,lon,y,x);

                    UTM_line->addPoint(x,y,0);
                }

                // Buffer the polygon
                buff_geom = UTM_line->Buffer(calcBuffer(t_lvl));
                buff_poly = ( OGRPolygon * )buff_geom;

                // Check to see if you can make unions with other polys
                buff_poly = check4Union(buff_poly, PolyLayer, depth, LayerName);

                // Segment the polygons
                buff_poly->segmentize(m_segmentation_dist);

                // Build the new feature as a polygon
                new_feat->SetGeometry(buff_poly);
                if( PolyLayer->CreateFeature( new_feat ) != OGRERR_NONE )
                {
                    printf( "Failed to create feature in shapefile.\n" );
                    exit( 1 );
                }
            }
	  
            OGRFeature::DestroyFeature( new_feat );
	  
        }
    }
    else
        cout << "Layer " << LayerName << " is not in the ENC." << endl;
}

// Simplifies the layers so the objects are not overlapping
OGRPolygon* ENC_Contact::check4Union(OGRPolygon *poly, OGRLayer *PolyLayer, double depth, string obs_type)
{
    OGRFeature *feat;
    OGRGeometry *geom, *newGeom;
    int FID;
    string filter;

    filter= "Depth="+to_string(depth);//"Type='"+obs_type+"'";

    PolyLayer->SetSpatialFilter(poly);
    PolyLayer->SetAttributeFilter(filter.c_str());
    feat = PolyLayer->GetNextFeature();
    PolyLayer->ResetReading();
    //cout << depth << " union " << PolyLayer->GetFeatureCount() << endl;
    while(feat)
    {
        //if (feat->GetFieldAsDouble("Depth") == depth)
        //{
        FID = feat->GetFID();
        geom = feat->GetGeometryRef();
        newGeom = poly->Union(geom);
        poly=(OGRPolygon*) newGeom;
        PolyLayer->DeleteFeature(FID);
        feat = PolyLayer->GetNextFeature();
        //}
    }

    return poly;
}

void ENC_Contact::build_search_poly()
{
    /*
    This function builds the polygon which represents the area that
    the ASV is going to search the ENC for potential threats. The shape
    of the search area is a square that is centered on the ASV's
    current X,Ylocation. It also prints the search area polygon to
    pMarnine Viewer.

    Outputs:
    search_area_poly - OGR Polygon that describes the area in which we
    want to search the input layer for objects
    */
    OGRLinearRing *poRing;

    // If the search distance is not set, then set the search distance by 10*(vessel length)
    if (!m_SearchDistSet)
        m_search_dist = 10*m_ASV_length;

    double x1, x2, x3, x4, y1, y2, y3, y4;
    double theta = 45+fmod(m_ASV_head,90);
    double add_sin = m_search_dist*sqrt(2)*sin(theta*PI/180);
    double add_cos = m_search_dist*sqrt(2)*cos(theta*PI/180);

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

    string s_poly_str = "pts={"+to_string((int)x1)+","+ to_string((int)y1)+":"+ to_string((int)x2)+","+to_string((int)y2)+":"+ to_string((int)x3)+","+to_string((int)y3)+":" +to_string((int)x4)+","+to_string((int)y4)+"}";

    s_poly_str+=",label=Search,edge_size=10,vertex_size=1,edge_color=red,active=true";

    Notify("VIEW_POLYGON", s_poly_str);
}

void ENC_Contact::filter_feats()
{
  // Remove old spatial filter
  Point_Layer->SetSpatialFilter(NULL);
  Poly_Layer->SetSpatialFilter(NULL);
  
  // Filter Data
  Point_Layer->SetSpatialFilter(search_area_poly);
  Poly_Layer->SetSpatialFilter(search_area_poly);
  //Point_Layer->SetAttributeFilter("t_lvl>0");
  //Poly_Layer->SetAttributeFilter("t_lvl>0");
}

void ENC_Contact::publish_points()
{
  /* 
     This function filters the point layer and then cycles through that 
     layer of point obstacles from the ENC to highlight them in
     pMarnineViewer and publish the x,y position, threat level and 
     obstacle type to the MOOSDB. At the end, it checks to see if any of 
     the highlighted obstacles are no longer within the search and if 
     there are any wrongly highlighted obstacles, it removes them.
  */

    OGRFeature *poFeature;
    OGRGeometry *geom;
    OGRPoint *poPoint;

    int num_obs=0;
    string obs_info, remove_highlight, obs_type, obstacles;

    double WL = 0;
    double depth = 0;
    double t_lvl = 0;
    Point_Layer->ResetReading();
    //Notify("Other",Point_Layer->GetFeatureCount() );
    while( (poFeature = Point_Layer->GetNextFeature()) != NULL )
    {

        geom = poFeature->GetGeometryRef();
        poPoint = ( OGRPoint * )geom;

        WL = poFeature->GetFieldAsDouble(1);
        depth = poFeature->GetFieldAsDouble(2);
        obs_type =poFeature->GetFieldAsString(3);

        t_lvl = calc_t_lvl(depth, WL, obs_type);

        if (t_lvl > 0)
            buildPointHighlight(poPoint, num_obs,obs_info,t_lvl,obs_type);
    }
    Notify("Other", to_string(t_lvl)+", "+to_string(depth) +", "+ to_string(m_ASV_draft));
    getNewPointVertex(num_obs, obs_info);

    // Output to the MOOSDB a list of obstacles
    //  ASV_X,ASV_Y,ASV_Head # of Obstacles : x=x_obs,y=y_obs,t_lvl,type ! x=x_obs,y=y_obs,t_lvl,type ! ...
    obstacles = to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(m_ASV_head)+":"+to_string(num_obs)+":"+obs_info;
    Notify("Obstacles", obstacles);

    // Determine if a new polygon was used
    if (m_max_pnts < num_obs)
        m_max_pnts = num_obs;

    // Remove highlighted point obstacles (shown as polygons) from
    //  pMarineViewer if they are outside of the search area
    for (int i=num_obs; i<m_max_pnts; i++)
    {
        remove_highlight = "format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=hpnt_"+to_string(i);
        Notify("VIEW_POLYGON", remove_highlight);
    }
}

void ENC_Contact::getNewPointVertex(int &num_obs, string &point_info)
{
    string obs_type = "newPoint";
    // Cycle through all of the new polygons to determine if any of them are in the search area
    for(int i=0; i<newPoint.size(); i++)
    {
        // If they are in the search area, find the closest vertex and the maximum angular extent
        if ((newPoint[i]->Within(search_area_poly))&&(pointTLvl[i]>0))
        {
            buildPointHighlight(newPoint[i], num_obs, point_info, pointTLvl[i], obs_type);
        }
    }
}

void ENC_Contact::buildPointHighlight(OGRPoint *poPoint, int &num_obs, string &point_info, int t_lvl, string obs_type)
{
    double x = 0;
    double y = 0;
    string pos, highlight;
    double dist;

    x = poPoint->getX();
    y = poPoint->getY();
    pos = "x="+to_string(x)+",y="+to_string(y);
    dist = sqrt(pow(x-m_ASV_x,2)+pow(y-m_ASV_y,2));
    dist2obstacle.push_back(dist);
    d_dist2obstacle.push_back(dist);

    // Calculate the distance and store the cost for updating the lead parameter
    //double dist = sqrt(pow(x-m_ASV_x, 2) + pow(y-m_ASV_y, 2));
    //calcCost(t_lvl, dist);
    vect_TLvl.push_back(t_lvl);

    highlight = "format=radial,"+pos+",radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label=hpnt_"+to_string(num_obs);
    // Highlight the point on pMarnineViewer
    Notify("VIEW_POLYGON", highlight);

    // Store the values of the position, threat level and obstacle typeX
    if (num_obs != 0)
      point_info += "!";
    num_obs ++;

    point_info += pos+","+to_string(t_lvl)+","+obs_type;
}
/*
//This function determines which obstacles with polygon geometry from
//  the ENC are within the search area.
void ENC_Contact::publish_poly()
{
    OGRFeature *poFeature;
    OGRGeometry *geom;
    OGRPolygon *poPoly;

    int num_obs=0;
    string obs_type = "";
    string vertices = "";
    string poly_info = "";
    string Poly_Obs = "";
    string pt_1,pt_2, pt_3;

    int i =0;

    double WL = 0;
    double depth = 0;
    double t_lvl = 0;

    prev_crit_pts.resize(Poly_Layer->GetFeatureCount()+newPoly.size());

    Poly_Layer->ResetReading();
    while( (poFeature = Poly_Layer->GetNextFeature()) != NULL )
    {
        // Calculate the current threat level
        WL = poFeature->GetFieldAsDouble(1);
        depth = poFeature->GetFieldAsDouble(2);
        obs_type =poFeature->GetFieldAsString(3);
        t_lvl = calc_t_lvl(depth,WL,obs_type);

        if (t_lvl>0)
        {
            geom = poFeature->GetGeometryRef();
            // Find the intersection of the polygon and the search area
            poPoly = (OGRPolygon*) geom;

            vertices = find_crit_pts(poPoly, num_obs, t_lvl, i);
            vertices = to_string(t_lvl)+","+obs_type+"@"+vertices;

            if (num_obs >0)
                poly_info += "!";

            poly_info += vertices;

            // Incriment the counter
            num_obs++;
        }
        i++;
    }

    // Check the polygons added from pMarineViewer to see if they are inside of the search area
    getNewPolyVertex(num_obs, poly_info);

    Poly_Obs = to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(m_ASV_head)+":"+to_string(num_obs);

    if (num_obs>0)
    {
        Poly_Obs = Poly_Obs+":"+poly_info;
    }

    Notify("Poly_Obs", Poly_Obs);

    if (m_max_poly < num_obs)
        m_max_poly = num_obs;

    for (int ii = num_obs; ii<m_max_poly; ii++)
    {
        pt_1 = "x=10000,y=10000,vertex_color=white,active=false,label=minAng_"+to_string(ii);
        pt_2 = "x=10000,y=10000,vertex_color=mediumblue,active=false,label=minDist_"+to_string(ii);
        pt_3 = "x=10000,y=10000,vertex_color=white,active=false,label=maxAng_"+to_string(ii);
        Notify("VIEW_POINT", pt_1);
        Notify("VIEW_POINT", pt_2);
        Notify("VIEW_POINT", pt_3);
    }
}
*/
/*
void ENC_Contact::getNewPolyVertex(int &num_obs, string &poly_info)
{
    string obs_type = "newPoly";
    string vertices;
    OGRGeometry *geomBuff;
    OGRPolygon *polyBuff;
    int numPolys = Poly_Layer->GetFeatureCount();
    // Cycle through all of the new polygons to determine if any of them are in the search area
    for(int i=0; i<newPoly.size(); i++)
    {
        geomBuff=newPoly[i]->Buffer(calcBuffer(polyTLvl[i]));
        polyBuff=(OGRPolygon*)geomBuff;
        Notify("Buffer", calcBuffer(polyTLvl[i]));
        // If they are in the search area, find the closest vertex and the maximum angular extent
        if ((polyBuff->Intersects(search_area_poly))&&(polyTLvl[i]>0))
        {
            vertices = find_crit_pts(polyBuff, num_obs, polyTLvl[i], i+numPolys);
            vertices = to_string(polyTLvl[i])+","+obs_type+"@"+vertices;
            if (num_obs >0)
                poly_info += "!";
            poly_info += vertices;
            num_obs++;
        }
    }
}
*/
/*
string ENC_Contact::find_crit_pts(OGRPolygon *poPolygon, int num_obs, int t_lvl, int ii)
{
    OGRPoint *ASV_pos;
    OGRLinearRing *poRing = ( OGRLinearRing * ) OGRGeometryFactory::createGeometry(wkbLinearRing);
    poRing = poPolygon->getExteriorRing();

    int pnt_cnt =poRing->getNumPoints();
    int min_ang_index, max_ang_index, min_dist_index;
    int i;

    double min_ang, max_ang, min_dist;
    double x, y, lat, lon, ang, prev_ang, first_ang;

    string inside = "0";

    string crit_pts = "";
    string pt_1,pt_2, pt_3;
    string ref_frame = "";

    vector<double> vert_x, vert_y;
    if (poRing != NULL)
    {
        vector<vector<double>> angle2poly;
        vector<double>  dist2poly;
        Envelope poly_envs = Envelope();
        for (i=0; i<pnt_cnt; i++)
        {
            x = poRing->getX(i);
            y = poRing->getY(i);

            vert_x.push_back(x);
            vert_y.push_back(y);

            dist2poly.push_back(sqrt(pow((m_ASV_x-x),2)+pow((m_ASV_y-y),2)));
            ang = relAng(m_ASV_x, m_ASV_y,x,y);

            if (i>0)
                poly_envs.store_angle(ang,prev_ang,(double)i,angle2poly);
            else
                first_ang = ang;

            prev_ang = ang;
        }
        // The angle of the first vertex should be compared to the angle
        //  of the last vertex.
        poly_envs.store_angle(first_ang,prev_ang,0,angle2poly);

        // Determine the minimum distance
        min_dist = *min_element(dist2poly.begin(), dist2poly.end());
        dist2obstacle.push_back(min_dist);
        d_dist2obstacle.push_back(min_dist);

        // Store the maximum cost (which is the minimum distance) for updating the lead parameter
        //calcCost(t_lvl, min_dist);
        vect_TLvl.push_back(t_lvl);

        // Deterimine which vertex is the min distance
        for (int i=0; i<pnt_cnt; i++)
        {
            if (min_dist ==dist2poly.at(i))
                min_dist_index = i;
        }

        // Set the position of the ASV
        ASV_pos = ( OGRPoint * ) OGRGeometryFactory::createGeometry(wkbPoint25D);
        ASV_pos->setX(m_ASV_x);
        ASV_pos->setY(m_ASV_y);

        // Check to see if you are inside of a polygon.
        //    If you are go towards the closest vertex
        if (ASV_pos->Within(poPolygon))
        {
            // If the closest vertex is the first vertex, then the max angle is
            // the last vertex in the ring
            if (min_dist_index ==0)
                min_ang_index = pnt_cnt-1;
            else
                min_ang_index = min_dist_index-1;

            // If the closest vertex is the last vertex, then the max angle is
            // the first vertex in the ring
            if (min_dist_index==pnt_cnt-1)
                max_ang_index = 0;
            else
                max_ang_index = min_dist_index+1;

            // set min/max angles
            min_ang = angle2poly[min_ang_index][0];
            max_ang = angle2poly[max_ang_index][0];
            inside = "1";
        }
        else
        {
            // Determine the minimum and maximum angles
            poly_envs.calcEnvelope(angle2poly);

            min_ang = poly_envs.GetMinAng();
            max_ang = poly_envs.GetMaxAng();

            // Get their index
            min_ang_index = poly_envs.GetMinAngIndex();
            max_ang_index = poly_envs.GetMaxAngIndex();
            inside = "0";
        }

        // Deal with the cross over point (0/360 boundary)
        if (min_ang < max_ang)
            ref_frame = "0";
        else
            ref_frame = "1";

        // Make string representations for each x,y points for ease of use
        string x_ang_min, y_ang_min, x_dist_min, y_dist_min, x_ang_max, y_ang_max;

        x_dist_min = to_string(vert_x.at(min_dist_index));
        y_dist_min = to_string(vert_y.at(min_dist_index));

        // Angles Wraped 360
        if (min_ang == -1)
        {
            cout << "No Bounds" << endl;
            x_ang_min = prev_crit_pts[ii][0];
            y_ang_min = prev_crit_pts[ii][1];
            x_ang_max = prev_crit_pts[ii][2];
            y_ang_max = prev_crit_pts[ii][3];
        }
        else
        {
            x_ang_min = to_string(vert_x.at(min_ang_index));
            y_ang_min = to_string(vert_y.at(min_ang_index));

            x_ang_max = to_string(vert_x.at(max_ang_index));
            y_ang_max = to_string(vert_y.at(max_ang_index));
            prev_crit_pts[ii] = {x_ang_min, y_ang_min, x_ang_max, y_ang_max};
            //cout << prev_crit_pts[ii][0] << " " << prev_crit_pts[ii][1]<< " " << prev_crit_pts[ii][2]<< " " << prev_crit_pts[ii][3] << endl;
        }

        // Post hints to pMarineViewer about the critical points
        pt_1 = "x="+x_ang_min+",y="+y_ang_min+",vertex_color=white,vertex_size=7,label=minAng_"+to_string(num_obs);
        pt_2 = "x="+x_dist_min+",y="+y_dist_min+",vertex_color=mediumblue,vertex_size=7,label=minDist_"+to_string(num_obs);
        pt_3 = "x="+x_ang_max+",y="+y_ang_max+",vertex_color=white,vertex_size=7,label=maxAng_"+to_string(num_obs);

        Notify("VIEW_POINT", pt_1);
        Notify("VIEW_POINT", pt_2);
        Notify("VIEW_POINT", pt_3);

        crit_pts = inside+"," +ref_frame +","+ x_ang_min +","+ y_ang_min +","+ x_dist_min +","+ y_dist_min +","+ x_ang_max +","+ y_ang_max;
    }
    else
    {
        // Post an error if the ring is empty
        cout << "Ring is EMPTY!!!!" << endl;
    }
    return crit_pts;
}
*/

void ENC_Contact::publish_poly_360()
{
    OGRFeature *feat;
    OGRGeometry *geom;

    int t_lvl;
    vector<double> util;
    double polyDist;
    int headingBias_Flag = 0;
    bool addedPolys_Inside_SearchArea =false;

    string Obstacles, angSweep_str;
    string position = to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(m_ASV_head)+"!";

    polyAngularSweep angSweep = polyAngularSweep(m_ASV_x, m_ASV_y, m_ASV_length, m_search_dist, false);
    angSweep.resetUtilVector();

    if ((Poly_Layer->GetFeatureCount()>0)||(newPoly.size() >0))
    {
        angSweep.setSearchPoly(search_area_poly);


        Poly_Layer->ResetReading();
        while( (feat = Poly_Layer->GetNextFeature()) != NULL )
        {
            geom = feat->GetGeometryRef();
            t_lvl = feat->GetFieldAsDouble("T_Lvl");
            vect_TLvl.push_back(t_lvl);
            angSweep.findIntersections(geom, t_lvl);
        }

        if (newPoly.size() >0)
            addedPolys_Inside_SearchArea = getNewPolyVertex(angSweep);

        // Output the ASV's position and the utility for each 5 degree sector and store it in MOOS
        if ((Poly_Layer->GetFeatureCount()>0)||(addedPolys_Inside_SearchArea))
        {
            // Store minimum distances to the polygons
            polyDist = angSweep.getMinDist();
            d_dist2obstacle.push_back(polyDist); // for each iteraton (for setting speed and lead param)
            dist2obstacle.push_back(polyDist); // for each cycle index

            // Set a flag for setting a heading bias toward the ASV's current heading the utility based
            //  if the ASV is within 3*ASV_Length and it is getting closer to the obstacle over 2 iterations
            set_headingBias_Flag(polyDist, headingBias_Flag);

            angSweep_str = angSweep.getUtilVect_asString();
            Obstacles = position+angSweep_str+"!"+to_string(headingBias_Flag)+"!"+to_string(angSweep.getSwathSize());
            Notify("Full_360_polys", Obstacles);

            // These are for the heading bias flag
            prev2PolyDist = prevPolyDist;
            prevPolyDist = polyDist;
        }
        else
        {
            prevPolyDist= 10*m_ASV_length;
            prev2PolyDist = 10*m_ASV_length;
        }
    }
    else
    {
        prevPolyDist= 10*m_ASV_length;
        prev2PolyDist = 10*m_ASV_length;
    }
}

bool ENC_Contact::getNewPolyVertex(polyAngularSweep &angSweep)
{
    OGRGeometry *geomBuff;
    OGRPolygon *polyBuff;
    bool polyInsideSearchArea = false;

    // Cycle through all of the new polygons to determine if any of them are in the search area
    for(int i=0; i<newPoly.size(); i++)
    {
        geomBuff=newPoly[i]->Buffer(calcBuffer(polyTLvl[i]));
        polyBuff=(OGRPolygon*)geomBuff;

        // If they are in the search area, find the closest vertex and the maximum angular extent
        if ((polyBuff->Intersects(search_area_poly))&&(polyTLvl[i]>0))
        {
            polyInsideSearchArea = true;
            angSweep.findIntersections(polyBuff, polyTLvl[i]);
            vect_TLvl.push_back(polyTLvl[i]);
        }
    }
    return polyInsideSearchArea;
}

// Set a flag for setting a heading bias toward the ASV's current heading the utility based
//  if the ASV is within 3*ASV_Length and it is getting closer to the obstacle over 2 iterations
void ENC_Contact::set_headingBias_Flag(double polyDist, int &headingBias_Flag)
{
    if ((((polyDist< 3*m_ASV_length)&&(polyDist <prevPolyDist)))&&(polyDist!=-1)&&(prevPolyDist!=-1))//(polyDist< m_ASV_length)||
         headingBias_Flag = 1;
    else
         headingBias_Flag = 0;

    Notify("FLAG", headingBias_Flag);
}

void ENC_Contact::calcCost(double t_lvl, double dist)
{
    // make sure you cannot divide by zero
    if (dist==0)
        max_cost.push_back(0);
    else
        max_cost.push_back(pow(t_lvl*m_ASV_length*m_speed/(dist),2)*50);
}

// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
void ENC_Contact::Update_Lead_Param()
{
    double lead, safety_dist;
    int t_lvl = *max_element(vect_TLvl.begin(), vect_TLvl.end());
    double dist = *min_element(d_dist2obstacle.begin(), d_dist2obstacle.end());
    d_dist2obstacle.clear();
    if (t_lvl <= 0)
        lead= 8;
    else
    {
        safety_dist = 3*m_ASV_length+0.5*t_lvl;// m_ASV_length/2*(1+t_lvl);//
        if ((dist > prev_dist)&&(dist>safety_dist))
            lead = 8;
        else
            lead = t_lvl*20;

        Notify("ClosestObstacle", dist);
    }

    Notify("WPT_UPDATE", "lead="+to_string(lead));
    prev_dist = dist;
}

//-------------------------------------------------------------
// This function updates the waypoint desired speed if the ASV is close
//  to an obstacle. If it is, then the ASV will slow down. If the ASV is
//  within a boat length it slows down to the minimum bare steerage.
//  Once the ASV is no gets farther away it speeds up again.
void ENC_Contact::UpdateSpeed()
{
    double newSpeed;
    if ((prev_dist < m_ASV_length)||(prev_dist==-1))
    {
        newSpeedSet_flag = true;
        Notify("WPT_UPDATE", "speed="+to_string(bareSteerage));
    }
    else if (prev_dist < 2*m_ASV_length)
    {
        newSpeed = prev_dist/m_ASV_length*bareSteerage;
        if (newSpeed<origDesSpeed)
        {
            Notify("WPT_UPDATE", "speed="+to_string(newSpeed));
            newSpeedSet_flag = true;
        }
    }
    else if(newSpeedSet_flag)
    {
        newSpeedSet_flag = false;
        Notify("WPT_UPDATE", "speed="+to_string(origDesSpeed));
    }
}

//-------------------------------------------------------------
// Procedure: relAng
//   Purpose: Returns relative angle of pt B to pt A. Treats A
//            as the center.
//
//                   0
//                   |
//                   |
//         270 ----- A ----- 90      
//                   |
//                   |
//                  180

double ENC_Contact::relAng(double xa, double ya, double xb, double yb)
{ 
  if((xa==xb)&&(ya==yb))
    return(0);

  double w   = 0;
  double sop = 0;

  if(xa < xb) {
    if(ya==yb)  
      return(90.0);
    else
      w = 90.0;
  }
  else if(xa > xb) {
    if(ya==yb)  
      return(270.0);
    else
      w = 270.0;
  }

  if(ya < yb) {
    if(xa == xb) 
      return(0.0);
    if(xb > xa) 
      sop = -1.0;
    else 
      sop =  1.0;
  }
  else if(yb < ya) {
    if(xa == xb) 
      return(180);
    if(xb >  xa) 
      sop =  1.0;
    else 
      sop = -1.0;
  }

  double ydiff = yb-ya;
  double xdiff = xb-xa;
  if(ydiff<0) ydiff = ydiff * -1.0;
  if(xdiff<0) xdiff = xdiff * -1.0;

  double avalPI = atan(ydiff/xdiff)*180.0/M_PI;
  double retVal = (avalPI * sop) + w;

  retVal = fmod(retVal, 360.0);
  if (retVal<0.0)
    retVal += 360.0;

  return(retVal);
}
