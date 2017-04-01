/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: AStar.cpp                                       */
/*    DATE: April 2017                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "AStar.h"

using namespace std;

//---------------------------------------------------------
// Constructor

AStar::AStar()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_tide = 0;
  
  // Initialize with 368 directions searched
  astar = A_Star(10); 
  m_mapfile="../../src/ENCs/Shape/grid/grid.csv";

  // Grid locations for a subset of the graph
  m_subset = false;
  m_subset_xmin = 0;
  m_subset_xmax = 0;
  m_subset_ymin = 0;
  m_subset_ymax = 0;

  // Grid meta data (how to convert back to the local UTM)
  m_grid_size=5;
  m_xTop = -4459; // value for the US5NH02M ENC
  m_yTop = -12386; // value for the US5NH02M ENC
  astar.setConversionMeta(m_grid_size, m_xTop, m_yTop);
}

//---------------------------------------------------------
// Destructor

AStar::~AStar()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool AStar::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  string name;
  
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    name   = msg.GetName();
    if (name == "Current_Tide")
      vect_tide.push_back(atof(msg.GetString().c_str()));
    else if(name == "New_WPT")
      vect_new_wpt.push_back(msg.GetString());
    
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

bool AStar::OnConnectToServer()
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

bool AStar::Iterate()
{
  if (vect_new_wpt.size() > 0)
    {
      // Convert the new waypoint directions into multiple start/end positions
      // Need to determine how to set if you repeat or not
    }
      
  if (vect_tide.size() > 0 )
    {
      m_tide = vect_tide.back();
      vect_tide.clear();
    }  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AStar::OnStartUp()
{
  list<string> sParams;
  vector<string> info, coordinates;
  
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "MAPFILE")
	m_mapfile=value;

      if((param == "MIN_Z")||(param == "DEPTH_CUTOFF"))
	astar.setDepthCutoff(atoi(value.c_str()));
      
      else if((param == "CONNECTING_DIST")||(param == "CONNECTING_DISTANCE"))
	astar.NeighborsMask(atoi(value.c_str()));
      
      else if((param == "GRID_DATA")||(param == "GRIDDATA"))
	{
	  info = parseString(value, ',');
	  if (info.size() == 3)
	    {
	      m_grid_size = atof(info[0].c_str());
	      m_xTop = atof(info[1].c_str());
	      m_yTop = atof(info[2].c_str());
	      astar.setConversionMeta(m_grid_size, m_xTop, m_yTop);
	    }
	  else
	    return false;
	}
      else if((param == "GRID_SIZE")||(param == "GRIDSIZE"))
	{
	  m_grid_size=atof(value.c_str());
	  astar.setConversionMeta(m_grid_size, m_xTop, m_yTop);
	}
      else if((param == "XMIN")||(param == "X_MIN")||(param == "XTOP"))
	{
	  m_xTop=atof(value.c_str());
	  astar.setConversionMeta(m_grid_size, m_xTop, m_yTop);
	}
      else if((param == "YMIN")||(param == "Y_MIN")||(param == "YTOP"))
	{
	  m_yTop=atof(value.c_str());
	  astar.setConversionMeta(m_grid_size, m_xTop, m_yTop);
	}
      else if(param == "SUBSET")
	{
	  info = parseString(value, ',');
	  if (info.size() == 4)
	    {
	      m_subset = true;
	      m_subset_xmin = atof(info[0].c_str());
	      m_subset_xmax = atof(info[1].c_str());
	      m_subset_ymin = atof(info[2].c_str());
	      m_subset_ymax = atof(info[3].c_str());
	    }
	  else
	    return false;
	}
      else if((param == "polygon") || (param == "points")) {
	XYSegList new_seglist = string2SegList(value);
	if(new_seglist.size() == 0)
	  return false;
      }
      else if(param == "point") {
	coordinates = parseString(value, ',');
	if (coordinates.size() == 2)
	  {
	    new_wpt_x.push_back(atof(info[0].c_str()));
	    new_wpt_y.push_back(atof(info[1].c_str()));
	  }
	else
	  return false;
	    
      }
	
    }
  }
  
  m_timewarp = GetMOOSTimeWarp();

  if (m_subset)
    astar.build_map(m_mapfile, m_subset_xmin, m_subset_xmax,
		    m_subset_ymin, m_subset_ymax);
  else
    astar.build_map(m_mapfile);
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void AStar::RegisterVariables()
{
  Register("Current_Tide",0);
  Register("New_WPT",0);
}

