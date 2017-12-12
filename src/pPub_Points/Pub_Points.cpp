/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: Pub_Points.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "Pub_Points.h"
#include <math.h>

using namespace std;

//---------------------------------------------------------
// Constructor

Pub_Points::Pub_Points()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_ASV_x = 0;
  m_ASV_y = 0;
  m_ASV_head = 0;
}

//---------------------------------------------------------
// Destructor

Pub_Points::~Pub_Points()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Pub_Points::OnNewMail(MOOSMSG_LIST &NewMail)
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
    else if (name == "NEW_POINTS")
      vect_pts.push_back(msg.GetString());
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Pub_Points::OnConnectToServer()
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

bool Pub_Points::Iterate()
{
  string new_pt;
  vector <string> parsed, x, y;
  double pt_x,pt_y;

  if (vect_head.size() > 0)
    {
      m_ASV_head = vect_head.back();
      vect_head.clear();
    } 
      
  if ((vect_y.size()>0) && (vect_y.size()>0))
    {
       m_ASV_x = vect_x.back();
       m_ASV_y = vect_y.back();
      
       vect_x.clear();
       vect_y.clear();
    }
  if (vect_pts.size()>0)
    {
      for (int i=0; i<vect_pts.size(); i++)
	{
	  new_pt = vect_pts.back();
	  parsed = parseString(new_pt,",");
	  x = parseString(parsed[0],"=");
	  y = parseString(parsed[1],"=");
	  pt_x = atof(x[1].c_str());
	  pt_y = atof(y[1].c_str());
	  if (dist(pt_x, pt_y, m_ASV_x, m_ASV_y)<75)
	    pt.push_back(new_pt);
	}
      vect_pts.clear();
    }
  
  for (int ii = 0;ii<pt.size(); ii++)
    Notify("Obstacles",to_string(m_ASV_x)+","+to_string(m_ASV_y)+","+to_string(m_ASV_head)+":"+to_string(pt.size()) +":"+ pt[ii]);
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Pub_Points::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "FOO") {
        //handled
      }
      else if(param == "BAR") {
        //handled
      }
    }
  }
  
  m_timewarp = GetMOOSTimeWarp();

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void Pub_Points::RegisterVariables()
{
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NEW_POINTS",0);
}

double Pub_Points::dist(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}
