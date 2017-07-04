/************************************************************/
/*    NAME: Sam Reed                                              */
/*    ORGN: MIT                                             */
/*    FILE: ENC_WPT_Check.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef ENC_WPT_Check_HEADER
#define ENC_WPT_Check_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MBUtils.h"
#include <string>
#include <iterator>
#include <vector>
#include <cmath>
#include "ogrsf_frmts.h" // for GDAL/OGR

using namespace std;

class ENC_WPT_Check : public CMOOSApp
{
    public:
        ENC_WPT_Check();
        ~ENC_WPT_Check() {}

    protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();

        // Open ENC Shapefiles
        bool openLayers();

        // If the waypoint is within a polygon, then switch to the next
        //  waypoint when it is comes within the buffer distance.
        void WPT_skip();

        // This function sets the position of the previous waypoint
        void Set_Prev_WPT();

        // This function sets the current waypoint and then cycles through the
        //  polygon obstacles and checks to see if the the current waypoint is within
        //  or the straight line path (with a small buffer) intersects any obstacles.
        void WPT_Valid(string WPT);

        // This function sets the current waypoint and then determines the
        //  straight line path between the two points (the returned value and WPT_poly).
        OGRLineString *BuildWPT(string WPT, OGRPolygon **WPT_poly);


    private: // Configuration variables

    private: // State variables
        double       m_timewarp;
        string ENC_name;
        GDALDataset *ds_poly;
        OGRLayer *layer;
        OGRPoint *curr_WPT;
        double buffer_dist;
        vector<double> vect_x, vect_y;
        vector<string> vect_nextWPT;
        double ASV_x, ASV_y, WPT_x, WPT_y, prev_WPT_x, prev_WPT_y;
        bool first_iteration, first_position, check_within, check_intersection, skipped, m_ENC_INT, openned;
        int wpt_index;
        vector<double> X_intersection, Y_intersection;
        vector<int> polyIntersect_Index;
};

#endif 
