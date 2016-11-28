/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH CCOM                                        */
/*    FILE: ENC_Contact.cpp                                 */
/*    DATE: 11/22/16                                        */
/************************************************************/

#include <iterator>
#include <vector>
#include <cmath>
#include <string>
#include "MBUtils.h"
#include "ENC_Contact.h"
#include "ogrsf_frmts.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ENC_Contact::ENC_Contact()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_MHW_Offset = 0;
  m_tide = 0;
  m_ASV_length = 4;
  m_ASV_width = 1;
  m_ASV_draft = 1;
  m_ENC = "US5NH02M";
  
  BuildLayers();

  cout << "Initialized" << endl;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ENC_Contact::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  string name = "";
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    name   = msg.GetName();
    if (name == "NAV_X")
      vect_x.push_back(msg.GetDouble());
    else if (name == "NAV_Y")
      vect_y.push_back(msg.GetDouble());
    else if(name == "NAV_HEADING")
      vect_head.push_back(msg.GetDouble());
    else if (name == "Current_Tide")
      vect_tide.push_back(atof(msg.GetString().c_str()));
    else if (name == "MHW_Offset")
      m_MHW_Offset = atof(msg.GetString().c_str());
    
#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ENC_Contact::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
  // m_MissionReader.GetConfigurationParam("LongOrigin", <string>);
   // m_Comms.Register("VARNAME", 0);
  
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ENC_Contact::Iterate()
{
  if (m_iterations ==0)
    {
      cout << m_iterations << endl;
      // Open the data source
      DS_pnt = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
      DS_poly = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Poly.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
      DS_line = (GDALDataset*) GDALOpenEx( "../../src/ENCs/Shape/Line.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
      
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
      if( DS_line == NULL )
	{
	  printf( "Open failed.\n" );
	  exit( 1 );
	}

      // Open the layers
      Point_Layer = DS_pnt -> GetLayerByName( "Point" );
      Poly_Layer = DS_poly -> GetLayerByName( "Poly" );
      Line_Layer = DS_line -> GetLayerByName( "Line" );

      if (Point_Layer == NULL)
      {
	cout << "Opening point layer failed." << endl;
      }
      if (Poly_Layer == NULL)
      {
	cout << "Opening poly layer failed." << endl;
      }
      if (Line_Layer == NULL)
      {
	cout << "Opening line layer failed." << endl;
      }

      // Get some geometries
      /*
      OGRFeature *poFeature = Point_Layer->GetNextFeature();
      geom_pnt = poFeature->GetGeometryRef();
      poFeature = Poly_Layer->GetNextFeature();
      geom_poly = poFeature->GetGeometryRef();
      poFeature = Line_Layer->GetNextFeature();
      geom_line = poFeature->GetGeometryRef();
      */
    }
  
  // While loop
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
      
      //cout << "X: " << m_ASV_x << ", Y: " << m_ASV_y << ", Head: " << m_ASV_head << endl;
      vect_x.clear();
      vect_y.clear();

      // need to add a global point, polygon and line geometry
      //build_search_poly(50, geom_poly);
      filter_feats(50);
      publish_points();
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
  double dfLatOrigin,dfLongOrigin;
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if (value != "default")
	{
	  if(param == "ENCS") {
	    m_ENC = value;
	  }
	  else if(param == "MHW_OffSET") {
	    m_MHW_Offset = atof(value.c_str());
	  }
	  // ASV Configuration
	  else if (param == "ASV_LENGTH")
	    m_ASV_length = atof(value.c_str());
	  
	  else if (param == "ASV_WIDTH")
	    m_ASV_width = atof(value.c_str());
	  
	  else if (param == "ASV_DRAFT")
	    m_ASV_draft = atof(value.c_str());
	}
    }
  }

  string sVal;

  if (m_MissionReader.GetValue("LatOrigin", sVal)) {
    dfLatOrigin = atof(sVal.c_str());
  } else {
    MOOSTrace("LatOrigin not set - FAIL\n");

    return false;

  }

  if (m_MissionReader.GetValue("LongOrigin", sVal)) {
    dfLongOrigin = atof(sVal.c_str());
  } else {
    MOOSTrace("LongOrigin not set - FAIL\n");

    return false;
  }

  if (!m_Geodesy.Initialise(dfLatOrigin, dfLongOrigin)) {
    MOOSTrace("Geodesy Init failed - FAIL\n");

    return false;
  }
  
  m_timewarp = GetMOOSTimeWarp();

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void ENC_Contact::RegisterVariables()
{
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("Current_Tide",0);
  //Register("MHW_Offset",0);
  //Register("ENCs",0);
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
  double feet2meters = 0;
  if (WL == 2)
      // At least 2 feet above MHW. Being shoal biased, we will take the 
      // object's "charted" depth as 2 feet above MHW
      WL_depth = m_tide-(2*feet2meters+m_MHW_Offset);
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
int ENC_Contact::calc_t_lvl(double depth, double WL, const char LayerName[7])
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
  else if ((LayerName == "BOYISD")||(LayerName == "BOYSPP")||(LayerName == "BOYSAW")||(LayerName == "BOYLAT")||(LayerName == "BCNSP")||(LayerName == "BCNLAT"))
    t_lvl = 3;
  else if (LayerName == "LNDMRK")
    t_lvl = -1;
  else
    {
      /*
      // Deal with the cases where the attributes are empty
      if (WL == NULL)
	WL = 0;
      else
	WL_depth = calc_WL_depth(WL);
      */
      if (depth == 9999)
	current_depth = 9999;
      else
	current_depth = depth+m_tide;
            
      // Neither the WL or depth are recorded - this is bad
      if ((current_depth == 9999) && (WL == 0))
	{
	  cout << "FAILED, Threat Level will be set to 4." << endl;
	  t_lvl = 4;
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
	      cout << "Unknown Description of Water Level: " << WL << ", Threat Level will be set to 4." << endl;
	      t_lvl = 4;
	    }
	  else
	    t_lvl = threat_level(WL_depth);
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
  // Above the water surface
  if (depth<=0)
    t_lvl = 4;
  // Near the water surface
  else if (depth< 1)
    t_lvl = 3;
  // Obstacle below surface
  else if (depth >= 1)
    {
      // 1<=depth<2 
      if (depth < 2)
	t_lvl = 2;
      // 2<=depth<4
      else if ((depth >=2) && (depth <= 4))
	t_lvl = 1;
      // Obstacle is deep (depth > 4m)
      else
	t_lvl = 0;
    }
  
  return t_lvl;
}

//---------------------------------------------------------
// Procedure:
/*
  Create a OGR layer for the point obstacles, polygon obstacles and  
  the line obstacles.
        
  Outputs (these are not returned, but defined as self.):
  ENC_point_layer - OGR layer that holds all the information from the 
  ENC that have point geometry
  ENC_poly_layer - OGR layer that holds all the information from the 
  ENC that have polygon geometry
  ENC_line_layer - OGR layer that holds all the information from the 
  ENC that have line geometry
*/
void ENC_Contact::BuildLayers()
{
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    GDALDataset *ds_pnt, *ds_poly, *ds_line, *ds_ENC;
    OGRLayer *PointLayer, *PolyLayer, *LineLayer;
    // Get the ENC
    ds_ENC = (GDALDataset*) GDALOpenEx( "../../src/ENCs/US5NH02M/US5NH02M.000", GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds_ENC == NULL )
      {
	printf( "Open failed.\n" );
	exit( 1 );
      }
    else
      printf("It opened.\n");
  
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
    ds_line = poDriver->Create( "../../src/ENCs/Shape/Line.shp", 0, 0, 0, GDT_Unknown, NULL );
    if( ds_line == NULL )
      {
	printf( "Creation of output file failed.\n" );
	exit( 1 );
      }
    
    // Create the layers (point, polygon, and lines)
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
    LineLayer = ds_line->CreateLayer( "Line", NULL, wkbLineString, NULL );
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
    oField_type.SetWidth(25);
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
    // Make the line layer
    if( LineLayer->CreateField( &oField_tlvl ) != OGRERR_NONE )
    {
        printf( "Creating Threat Level field failed.\n" );
        exit( 1 );
    }
    if( LineLayer->CreateField( &oField_WL ) != OGRERR_NONE )
    {
        printf( "Creating WL field failed.\n" );
        exit( 1 );
    }
    if( LineLayer->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    if( LineLayer->CreateField( &oField_type ) != OGRERR_NONE )
    {
        printf( "Creating Type field failed.\n" );
        exit( 1 );
    }
    if( LineLayer->CreateField( &oField_cat ) != OGRERR_NONE )
    {
        printf( "Creating Cat field failed.\n" );
        exit( 1 );
    }
    if( LineLayer->CreateField( &oField_visual ) != OGRERR_NONE )
    {
        printf( "Creating visual field failed.\n" );
        exit( 1 );
    }
    // Points only
    LayerMultiPoint(ds_ENC->GetLayerByName("SOUNDG"), PointLayer, "SOUNDG");
    ENC_Converter(ds_ENC->GetLayerByName("UWTROC"), PointLayer, PolyLayer, LineLayer, "UWTROC");
    ENC_Converter(ds_ENC->GetLayerByName("LIGHTS"), PointLayer, PolyLayer, LineLayer, "LIGHTS");
    ENC_Converter(ds_ENC->GetLayerByName("BOYSPP"), PointLayer, PolyLayer, LineLayer, "BOYSPP");
    ENC_Converter(ds_ENC->GetLayerByName("BOYISD"), PointLayer, PolyLayer, LineLayer, "BOYISD");
    ENC_Converter(ds_ENC->GetLayerByName("BOYSAW"), PointLayer, PolyLayer, LineLayer, "BOYSAW");
    ENC_Converter(ds_ENC->GetLayerByName("BOYLAT"), PointLayer, PolyLayer, LineLayer, "BOYLAT");
    ENC_Converter(ds_ENC->GetLayerByName("BCNSPP"), PointLayer, PolyLayer, LineLayer, "BCNSPP");
    ENC_Converter(ds_ENC->GetLayerByName("BCNLAT"), PointLayer, PolyLayer, LineLayer, "BCNLAT");
    
    // Other Types
    ENC_Converter(ds_ENC->GetLayerByName("LNDARE"), PointLayer, PolyLayer, LineLayer, "LNDARE");
    ENC_Converter(ds_ENC->GetLayerByName("PONTON"), PointLayer, PolyLayer, LineLayer, "PONTON");
    ENC_Converter(ds_ENC->GetLayerByName("DEPCNT"), PointLayer, PolyLayer, LineLayer, "DEPCNT");
    ENC_Converter(ds_ENC->GetLayerByName("DYKCON"), PointLayer, PolyLayer, LineLayer, "DYKCON");
    ENC_Converter(ds_ENC->GetLayerByName("LNDMRK"), PointLayer, PolyLayer, LineLayer, "LNDMRK");
    ENC_Converter(ds_ENC->GetLayerByName("SILTNK"), PointLayer, PolyLayer, LineLayer, "SILTNK");
    ENC_Converter(ds_ENC->GetLayerByName("WRECKS"), PointLayer, PolyLayer, LineLayer, "WRECKS");
    
    // close the data sources
    GDALClose( ds_ENC );
    GDALClose( ds_pnt );
    GDALClose( ds_poly );
    GDALClose( ds_line );
}

//---------------------------------------------------------
// Procedure:LayerMultiPoint
/*
  Adds the features from the inputed multipoints layer to a point 
  layer.
        
  Inputs:
  LayerName_mp - Name of the multipoint layer
*/
void ENC_Contact::LayerMultiPoint(OGRLayer *layer_mp, OGRLayer *PointLayer, const char LayerName_mp[7])
{
  OGRFeature *feat_mp, *new_feat;
  OGRFeatureDefn *feat_def;
  OGRGeometry *geom, *poPointGeometry;
  OGRPoint *poPoint, pt;
  OGRMultiPoint *poMultipoint;
  double depth = 9999;
  int num_geom = 0;
  int WL = 0;
  double x,y;
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
		
		// Get the x,y, and depth
		x = poPoint->getX();
		y = poPoint->getY();
		depth = poPoint->getZ();
		
		pt.setX(x);//poPoint->getX());
		pt.setY(y);//poPoint->getY());
		
		new_feat =  OGRFeature::CreateFeature(feat_def);
		new_feat->SetField("Depth", depth);
		new_feat->SetField("T_lvl", calc_t_lvl(depth, WL, LayerName_mp));
		new_feat->SetField("WL", 0);
		new_feat->SetField("Type", LayerName_mp);
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
void ENC_Contact::ENC_Converter(OGRLayer *Layer_ENC, OGRLayer *PointLayer, OGRLayer *PolyLayer, OGRLayer *LineLayer, const char LayerName[7])
{
  OGRFeature *poFeature, *new_feat;
  OGRFeatureDefn *poFDefn, *poFDefn_ENC;
  OGRFieldDefn *poFieldDefn;
  OGRGeometry *geom;
  OGRPoint *poPoint, pt;
  OGRPolygon *poPoly, poly;
  OGRLineString *poLine;
    
  double WL;
  int vis;
  double t_lvl = 0;
  double depth = 9999;
  int iField = 0;
  string name = "";
  string cat;
  
  if (Layer_ENC != NULL)
    {
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
		  // Reset depth to 9999
		  depth = 9999;
		  depth = poFeature->GetFieldAsDouble(iField);
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
	  t_lvl = calc_t_lvl(depth, WL, LayerName);

	  if (geom ->getGeometryType()  == wkbPoint)
	    {
	      // Set attributes of the new feature
	      poFDefn = PointLayer->GetLayerDefn();
	      new_feat =  OGRFeature::CreateFeature(poFDefn);
	      
	      new_feat->SetField("Depth", depth);
	      new_feat->SetField("WL", WL);
	      new_feat->SetField("T_lvl", t_lvl);
	      new_feat->SetField("Type", LayerName);
	      new_feat->SetField("Cat", cat.c_str());
	      new_feat->SetField("Visual", vis);
	      //*/
	      // Make the point
	      poPoint = ( OGRPoint * )geom;
		
	      // Get the x,y
	      pt.setX(poPoint->getX());
	      pt.setY(poPoint->getY());
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
	      new_feat->SetField("Type", LayerName);
	      new_feat->SetField("Cat", cat.c_str());
	      new_feat->SetField("Visual", vis);
	      
	      poPoly = ( OGRPolygon * )geom;
	      new_feat->SetGeometry(poPoly);
	      
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
	      poFDefn = LineLayer->GetLayerDefn();
	      new_feat =  OGRFeature::CreateFeature(poFDefn);
	      new_feat->SetField("Depth", depth);
	      new_feat->SetField("WL", WL);
	      new_feat->SetField("T_lvl", t_lvl);
	      new_feat->SetField("Type", LayerName);
	      new_feat->SetField("Cat", cat.c_str());
	      new_feat->SetField("Visual", vis);
	      
	      
	      poLine = ( OGRLineString * )geom;
	      new_feat->SetGeometry( poLine);
	  
	      // Build the new feature
	      if( LineLayer->CreateFeature( new_feat ) != OGRERR_NONE )
		{
		  printf( "Failed to create feature in shapefile.\n" );
		  exit( 1 );
		}
	      
	    }
	  
	  OGRFeature::DestroyFeature( new_feat );
	  
	}
    }
  else
    {
      cout << "Layer " << LayerName << " is not in the ENC." << endl;
      exit(1);
    }
}

void ENC_Contact::build_search_poly(double search_dist, OGRGeometry *geom)
{
  /*
    This function builds the polygon which represents the area that
    the ASV is going to search the ENC for potential threats. The shape 
    of the search area is a square that is centered on the ASV's 
    current X,Ylocation. It also prints the search area polygon to 
    pMarnine Viewer.
        
    Outputs:
    search_area_poly - OGR Polygon that describes the area in which we 
    want to seach the input layer for objects
  */
  OGRLinearRing *poRing;
  //OGRPoint *pt;
  
  double lon1, lon2, lon3, lon4, lat1, lat2, lat3, lat4;
  double x1, x2, x3, x4, y1, y2, y3, y4;
  double theta = 45-fmod(m_ASV_head,90);
  double add_sin = search_dist*sqrt(2)*sin(theta*PI/180);
  double add_cos = search_dist*sqrt(2)*cos(theta*PI/180);

  cout << "ANG: "<< theta <<" SIN: " << add_sin << ", COS: " << add_cos<< endl;
  
  x1 = m_ASV_x+add_sin;
  x2 = m_ASV_x-add_cos;
  x3 = m_ASV_x-add_sin;
  x4 = m_ASV_x+add_cos;
  
  y1 = m_ASV_y+add_cos;
  y2 = m_ASV_y+add_sin;
  y3 = m_ASV_y-add_cos;
  y4 = m_ASV_y-add_sin;
  
  
  // Convert the positions for the vertices of the polygon to lat/long
  m_Geodesy.LocalGrid2LatLong(x1, y1, lat1, lon1);
  m_Geodesy.LocalGrid2LatLong(x2, y2, lat2, lon2);
  m_Geodesy.LocalGrid2LatLong(x3, y3, lat3, lon3);
  m_Geodesy.LocalGrid2LatLong(x4, y4, lat4, lon4);
  
  cout << "before1" << endl;
  search_area_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
  poRing = ( OGRLinearRing * ) OGRGeometryFactory::createGeometry(wkbLinearRing);
  cout << "before2" << endl;
  poRing->addPoint(lon2, lat2);
  cout << "after" <<endl;
  //poRing->addPoint(lon2, lat2);
  //poRing->addPoint(lon3, lat3);
  //poRing->addPoint(lon4, lat4);
  //poRing->addPoint(lon1, lat1);
  cout << "after" <<endl;
  //poRing->closeRings();
  search_area_poly->addRing(poRing);
  
  
  //search_area_poly->closeRings();
  
  
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
  string obs_pos = "";
  string highlight = "";
  string remove_highlight = "";
  string pos = "";
  string obs_type = "";
  string obstacles = "";
  
  double lat = 0;
  double lon = 0;
  double x = 0;
  double y = 0;

  double WL = 0;
  double depth = 0;
  double t_lvl = 0;
  

  while( (poFeature = Point_Layer->GetNextFeature()) != NULL )
    {
      geom = poFeature->GetGeometryRef();
      poPoint = ( OGRPoint * )geom;
		
      // Get the lat and long position of the point and convert it to UTM 
      lon = poPoint->getX();
      lat = poPoint->getY();

      m_Geodesy.LatLong2LocalUTM(lat,lon,y,x);
      pos = "x="+to_string(x)+",y="+to_string(y);
      cout << pos << endl;

      highlight = "format=radial,"+pos+",radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label="+to_string(num_obs);

      Notify("VIEW_POLYGON", highlight);

      if (num_obs != 0)
	obs_pos += "!";
      num_obs ++;

      WL = poFeature->GetFieldAsDouble(1);
      depth = poFeature->GetFieldAsDouble(2);
      obs_type =poFeature->GetFieldAsString(3);
      
      t_lvl = calc_t_lvl(depth, WL, obs_type.c_str());
      obs_pos += pos+","+to_string(t_lvl)+","+obs_type; 
    }
  // Output to the MOOSDB a list of obstacles
  //  ASV_X,ASV_Y,Heading : # of Obstacles : x=x_obs,y=y_obs,t_lvl,type ! x=x_obs,y=y_obs,t_lvl,type ! ...
  obstacles = to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(m_ASV_head)+":"+to_string(num_obs)+":"+obs_pos;
  Notify("Obstacles", obstacles);

  // Determine if a new polygon was used
  if (max_pnts < num_obs)
    max_pnts = num_obs;
  
  // Remove highlighted point obstacles (shown as polygons) from 
  //  pMarineViewer if they are outside of the search area
  for (int i=num_obs; i<max_pnts; i++)
    {
      remove_highlight = "format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label="+to_string(i);
      Notify("VIEW_POLYGON", remove_highlight);
    }
}
void ENC_Contact::filter_feats(double search_dist)
{
  double lon1, lon2, lon3, lon4, lat1, lat2, lat3, lat4;
  double x1, x2, x3, x4, y1, y2, y3, y4;
  double lon_min, lon_max, lat_min, lat_max;
  double theta = 45-fmod(m_ASV_head,90);
  double add_sin = search_dist*sqrt(2)*sin(theta*PI/180);
  double add_cos = search_dist*sqrt(2)*cos(theta*PI/180);
  
  
  x1 = m_ASV_x+add_sin;
  x2 = m_ASV_x-add_cos;
  x3 = m_ASV_x-add_sin;
  x4 = m_ASV_x+add_cos;
  
  y1 = m_ASV_y+add_cos;
  y2 = m_ASV_y+add_sin;
  y3 = m_ASV_y-add_cos;
  y4 = m_ASV_y-add_sin;
    
  // Convert the positions for the vertices of the polygon to lat/long
  m_Geodesy.LocalGrid2LatLong(x1, y1, lat1, lon1);
  m_Geodesy.LocalGrid2LatLong(x2, y2, lat2, lon2);
  m_Geodesy.LocalGrid2LatLong(x3, y3, lat3, lon3);
  m_Geodesy.LocalGrid2LatLong(x4, y4, lat4, lon4);
  
  double lon[4] = {lon1,lon2,lon3,lon4};
  double lat[4] = {lat1,lat2,lat3,lat4};
  
  lon_min = *min_element(lon,lon+4);
  lon_max = *max_element(lon,lon+4);
  lat_min = *min_element(lat,lat+4);
  lat_max = *max_element(lat,lat+4);
  
  // Remove old spatial filter
  Point_Layer->SetSpatialFilter(NULL);
  Poly_Layer->SetSpatialFilter(NULL);

  // Remove old attribute filter
  Point_Layer->SetAttributeFilter(NULL);
  Poly_Layer->SetAttributeFilter(NULL);
  //cout << "start: " << Point_Layer->GetFeatureCount() << endl;
  //cout << "Min: " << lon_min <<"," <<lat_min << " Max: "<< lon_max <<"," <<lat_max <<endl;
  // Filter Data
  Point_Layer->SetSpatialFilterRect(lon_min,lat_min,lon_max,lat_max);
  Poly_Layer->SetSpatialFilterRect(lon_min,lat_min,lon_max,lat_max);
  //cout << "end: " << Point_Layer->GetFeatureCount() << endl;
  
  Point_Layer->SetAttributeFilter("T_lvl>0");
  Poly_Layer->SetAttributeFilter("T_lvl>0");
  Line_Layer->SetAttributeFilter("T_lvl>0");
  //cout << "end2: " << Point_Layer->GetFeatureCount() << endl;
  string s_poly_str = "pts={"+to_string((int)x1)+","+ to_string((int)y1)+":"+ to_string((int)x2)+","+to_string((int)y2)+":"+ to_string((int)x3)+","+to_string((int)y3)+":" +to_string((int)x4)+","+to_string((int)y4)+"},label=Search,edge_size=10,vertex_size=1,edge_color=red,active=true";
  
  Notify("VIEW_POLYGON", s_poly_str);
}
void ENC_Contact::publish_poly()
{
  /*
    This function determines which obstacles with polygon geometry from
    the ENC are within the search area. 
  */
  OGRFeature *poFeature;
  OGRGeometry *geom;
  OGRPoint *poPoint;
  OGRPolygon *poPoly;
  OGRLinearRing *poRing;
  
  int num_obs=0;
  string obs_pos = "";
  string obs_type = "";
  string vertices = "";
  string poly_info = "";
  string pt_1,pt_2, pt_3;
  
  double lat = 0;
  double lon = 0;
  double x = 0;
  double y = 0;

  double WL = 0;
  double depth = 0;
  double t_lvl = 0;

  while( (poFeature = Poly_Layer->GetNextFeature()) != NULL )
    {
      geom = poFeature->GetGeometryRef();
      poPoly = (OGRPolygon)* geom;

      // Find the intersection of the polygon and the search area
      //poRing = geom->GetGeometryRef(0);
      find_crit_pts(poPoly);
      
      // Calculate the current threat level
      WL = poFeature->GetFieldAsDouble(1);
      depth = poFeature->GetFieldAsDouble(2);
      obs_type =poFeature->GetFieldAsString(3);
      t_lvl = calc_t_lvl(depth,WL,obs_type);
      
      vertices = to_string(t_lvl)+","+to_string(obs_type)+"@"+vertices;
      if (num_obs >0)
	poly_info += "!";
      poly_info += vertices;

      // Incriment the counter
      num_obs++;
    }
  if (num_obs>0)
    poly_info = to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(M-ASV_head)+","+to_string(num_obs)+poly_info;
  if (max_poly < num_obs)
    max_poly = num_obs;

  for (int ii = num_obs; ii<max_poly; ii++)
    {
      pt_1 = "x=1,y=1,vertex_color=white,active=false,label=pt1_"+to_string(ii);
      Notify("VIEW_POINT", pt_1);
      pt_2 = "x=1,y=1,vertex_color=white,active=false,label=pt2_"+to_string(ii);
      Notify("VIEW_POINT", pt_2);
      pt_3 = "x=1,y=1,vertex_color=mediumblue,active=false,label=pt3_"+to_string(ii);
      Notify("VIEW_POINT", pt_3);
    }
}
string ENC_Contact::find_crit_pts(OGRPolygon *poPolygon)
{
  OGRLinearRing *poRing;
  
  if (poRing != NULL)
	{
	  
	}
      else
	{

	}
}
