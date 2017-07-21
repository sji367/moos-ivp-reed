/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: ENC_Contact.h                                   */
/*    DATE:                                                 */
/************************************************************/

#ifndef ENC_Contact_HEADER
#define ENC_Contact_HEADER
#define PI 3.14159265

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "XYPolygon.h"
#include "XYFormatUtilsPoly.h" // For converting strings to XYPolygons
#include "MBUtils.h"
#include <vector>
#include <string>
#include <iterator>
#include <cmath> // for pow and sqrt
#include <algorithm> // for min_element and max_element
#include "geodesy.h" // Conversion between lat/lon and UTM
#include "ogrsf_frmts.h" // GDAL
#include "envelope.h"

using namespace std;

class ENC_Contact : public CMOOSApp
{
 public:
       ENC_Contact();
       ~ENC_Contact();

 protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();
	
        // Parsers for different catagories
        string category_lights(int index);
        string category_landmark(int index);
        string category_silo(int index);

        // Functions to calculate the threat level
	double calc_WL_depth(double WL);
	int calc_t_lvl(double &depth, double WL, string LayerName);
	int threat_level(double depth);

        // Functions for building ENC_DB (shapefiles) from the ENC
	void BuildLayers();
	void ENC_Converter(OGRLayer *Layer_ENC, OGRLayer *PointLayer, OGRLayer *PolyLayer, OGRLayer *LineLayer, string LayerName);
	void LayerMultiPoint (OGRLayer *layer_mp, OGRLayer *Point_Layer, string LayerName_mp);

        OGRPolygon *check4Union(OGRPolygon* poly, OGRLayer *PolyLayer, double depth, string obs_type);

        // Function to build the search area polygon and filter ENC_DB to only include
        //  features from that area.
	void build_search_poly();
        void filter_feats();

        // Avoiding points (from ENCs and new ones from pMarineViewer)
	void publish_points();
        void buildPointHighlight(OGRPoint *poPoint, int &num_obs, string &point_info, int t_lvl, string obs_type);

        // Avoiding points (from ENCs and new ones from pMarineViewer)
        string find_crit_pts(OGRPolygon *poPolygon, int num_obs, int t_lvl);
	void publish_poly();

	double relAng(double xa, double ya, double xb, double yb);
        double calc_dist(double x1, double y1, double x2, double y2) {return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2)); }

        // Adding points/polygons to avoid to a vector
        void parseNewPoint(string pointString);
        void parseNewPoly(string polyString);

        // Finding the critical vertices from the new points/polygons
        void getNewPolyVertex(int &num_obs, string &poly_info);
        void getNewPointVertex(int &num_obs, string &point_info);

        void setColor(int t_lvl, string &color);

        void calcCost(double t_lvl, double dist);
        void Update_Lead_Param();

	CMOOSGeodesy m_Geodesy;

 private: // Configuration variables

 private: // State variables
        unsigned int m_iterations;
        double       m_timewarp;
        int m_max_pnts, m_max_poly;
        vector <double> vect_x, vect_y, vect_head, vect_tide, max_cost;

        // GDAL
        OGRLayer *Point_Layer, *Poly_Layer, *Line_Layer;
        GDALDataset *DS_pnt, *DS_poly, *DS_line;
        OGRPolygon*  search_area_poly;

        double m_ASV_x, m_ASV_y, m_ASV_head, m_ASV_length, m_ASV_width, m_ASV_draft;
        double m_MHW_Offset, m_tide;
        double m_search_dist, m_min_depth;
        double m_segmentation_dist, m_buffer_size;
        bool m_first_run;
        string m_ENC;

        bool m_simplifyPolys;

        // Adding points/polygons to avoid
        vector<int> pointTLvl, polyTLvl, vect_TLvl;
        vector<string> pointLabels;
        vector<XYPolygon> MOOS_polygons;
        vector<OGRPoint*> newPoint;
        vector<OGRPolygon*> newPoly;

        // Variables for Update_Lead_Param
        vector<double> dist2obstacle, d_dist2obstacle;


        double m_speed;
        Geodesy geod;
};

#endif 
