/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: Pub_Points.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef Pub_Points_HEADER
#define Pub_Points_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include <vector>
#include <string>


class Pub_Points : public CMOOSApp
{
 public:
   Pub_Points();
   ~Pub_Points();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
	double dist(double, double, double, double);

 private: // Configuration variables

 private: // State variables
   unsigned int m_iterations;
   double       m_timewarp;
	double m_ASV_x, m_ASV_y, m_ASV_head;
	std::vector <double> vect_x, vect_y, vect_head;
	std::vector <std::string> vect_pts,pt;
};

#endif 
