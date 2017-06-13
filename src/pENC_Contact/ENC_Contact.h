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
#include "ogrsf_frmts.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <vector>
#include <string>
#include "geodesy.h"

using namespace std;

class ENC_Contact : public CMOOSApp
{
 public:
       ENC_Contact();
       ~ENC_Contact() {}

 protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();
	
	std::string category_lights(int index);
	std::string category_landmark(int index);
	std::string category_silo(int index);
	double calc_WL_depth(double WL);
	int calc_t_lvl(double &depth, double WL, string LayerName);
	int threat_level(double depth);
	void BuildLayers();
	void ENC_Converter(OGRLayer *Layer_ENC, OGRLayer *PointLayer, OGRLayer *PolyLayer, OGRLayer *LineLayer, string LayerName);
	void LayerMultiPoint (OGRLayer *layer_mp, OGRLayer *Point_Layer, string LayerName_mp);
	void build_search_poly();
	void publish_points();
	void filter_feats();
	string find_crit_pts(OGRPolygon *poPolygon, int num_obs);
	void publish_poly();
	double relAng(double xa, double ya, double xb, double yb);
        double calc_dist(double x1, double y1, double x2, double y2) {return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2)); }

	CMOOSGeodesy m_Geodesy;

 private: // Configuration variables

 private: // State variables
        unsigned int m_iterations;
        double       m_timewarp;
        int m_max_pnts, m_max_poly;
        std::vector <double> vect_x, vect_y, vect_head, vect_tide;

        // GDAL
        OGRLayer *Point_Layer, *Poly_Layer, *Line_Layer;
        GDALDataset *DS_pnt, *DS_poly, *DS_line;
        OGRPolygon*  search_area_poly;

        double m_ASV_x, m_ASV_y, m_ASV_head, m_ASV_length, m_ASV_width, m_ASV_draft;
        double m_MHW_Offset, m_tide;
        double m_search_dist, m_max_avoid_dist;
        bool m_first_run;
        std::string m_ENC;

        double m_speed;
        Geodesy geod;
};

#endif 
