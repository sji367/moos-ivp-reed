/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: AStar.h                                         */
/*    DATE: March 2017                                      */
/************************************************************/

#ifndef AStar_HEADER
#define AStar_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "astar.h"
#include "XYFormatUtilsSegl.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "WaypointEngine.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
using namespace std;

class AStar : public CMOOSApp
{
 public:
   AStar();
   ~AStar();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
   double       m_timewarp;
   unsigned int m_iterations;

	string new_WPT, m_mapfile;
	vector<double> original_WPTs;
	A_Star astar;
	double m_grid_size, m_xTop, m_yTop;
	double m_tide;
	bool m_subset, m_map_initialized;
	int m_subset_xmin,m_subset_xmax,m_subset_ymin,m_subset_ymax;

	vector<double> vect_tide;
	vector<string> vect_new_wpt;
	vector<double> new_wpt_x,new_wpt_y ;

	XYSegList waypoint;
	
};

#endif 
