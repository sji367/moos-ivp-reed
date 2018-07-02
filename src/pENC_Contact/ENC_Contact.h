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
#include "ogr_spatialref.h"
//#include "envelope.h"
#include "ENC_Polygonize.h"
#include "poly_AngularSweep.h"
#include "polyInside.h"
#include <unistd.h>

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
        void initGeodesy(bool LatLongInMOOSFile);
        void setENC_Scale(GDALDataset *ds);
        void ENC_Converter(OGRLayer *Layer_ENC,OGRLayer *PolyLayer, string LayerName);

        // This function determines what scale it should use for buffering the points to 2 mm at chart scale.
        //  Basically it check if there are subsets in the ENC and if there are, it then checks to see if the point
        //  is inside that subset. If the point is inside a subset it uses the smallest scale for buffering.
        double pointGetChartScale(OGRPoint pt);

	void LayerMultiPoint (OGRLayer *layer_mp, OGRLayer *Point_Layer, string LayerName_mp);
        void StoreShallowPolys(OGRLayer *layer, OGRLayer *PolyLayer);

        OGRPolygon *check4Union(OGRPolygon* poly, OGRLayer *PolyLayer, double depth, string obs_type);

        // Function to build the search area polygon and filter ENC_DB to only include
        //  features from that area.
	void build_search_poly();
        void filter_feats();

        // Avoiding points (from ENCs and new ones from pMarineViewer)
	void publish_points();
        void buildPointHighlight(OGRPoint *poPoint, int &num_obs, string &point_info, int t_lvl, string obs_type);

        // Avoiding points (from ENCs and new ones from pMarineViewer)
        //string find_crit_pts(OGRPolygon *poPolygon, int num_obs, int t_lvl, int ii);
        void publish_poly_360();

	double relAng(double xa, double ya, double xb, double yb);
        double calc_dist(double x1, double y1, double x2, double y2) {return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2)); }

        // Adding points/polygons to avoid to a vector
        void parseNewPoint(string pointString);
        void parseNewPoly(string polyString);

        // Finding the critical vertices from the new points/polygons
        bool getNewPolyVertex(polyAngularSweep &angSweep);
        //void getNewPolyVertex(int &num_obs, string &poly_info);
        void getNewPointVertex(int &num_obs, string &point_info);

        void setColor(int t_lvl, string &color);

        void calcCost(double t_lvl, double dist);
        void Update_Lead_Param();
        void UpdateSpeed();

        double calcBuffer(int t_lvl) {return sqrt(pow(m_ASV_length,2)+pow(m_ASV_width,2))/2.0*(1+.4*t_lvl);}

	CMOOSGeodesy m_Geodesy;


        void set_headingBias_Flag(double polyDist, int &headingBias_Flag);

 private: // Configuration variables

 private: // State variables
        unsigned int m_iterations;
        double       m_timewarp;
        int m_max_pnts, m_max_poly;
        vector <double> vect_x, vect_y, vect_head, vect_tide, max_cost;

        // GDAL
        OGRLayer *Point_Layer, *Poly_Layer;
        GDALDataset *DS_pnt, *DS_poly;
        OGRPolygon*  search_area_poly;

        double m_ASV_x, m_ASV_y, m_ASV_head, m_ASV_length, m_ASV_width, m_ASV_draft;
        double dfLatOrigin,dfLongOrigin;
        double m_MHW_Offset, m_tide;
        double m_search_dist, m_min_depth;
        double m_segmentation_dist, m_buffer_size;
        bool m_first_run;
        string m_ENC;
        string OutfileDIR;

        // ENC Scale objects
        double ENC_Scale;
        vector<OGRPolygon*> scaleSubsets_poly;
        vector<double> SubsetScale;

        bool m_simplifyPolys, m_SearchDistSet, geodSet;

        // Adding points/polygons to avoid
        vector<int> pointTLvl, polyTLvl, vect_TLvl;
        vector<string> pointLabels;
        vector<XYPolygon> MOOS_polygons;
        vector<OGRPoint*> newPoint;
        vector<OGRPolygon*> newPoly;

        // Variables for Update_Lead_Param
        vector<double> dist2obstacle, d_dist2obstacle;
        double prev_dist, prevPolyDist, prev2PolyDist;

        vector<vector<string>> prev_crit_pts;

        bool newSpeedSet_flag;
        double bareSteerage, origDesSpeed;

        double m_speed;
        Geodesy geod;
};

#endif 
