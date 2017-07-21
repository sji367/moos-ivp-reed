/************************************************************/
/*    NAME: Sam Reed                                              */
/*    ORGN: MIT                                             */
/*    FILE: ENC_Print.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef ENC_Print_HEADER
#define ENC_Print_HEADER

#include "MBUtils.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include <vector>
#include <iterator>
#include "ogrsf_frmts.h"
#include "geodesy.h"
#include <unistd.h>
#include <string>

using namespace std;

class ENC_Print : public CMOOSApp
{
    public:
        ENC_Print();
        ~ENC_Print();

    protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();

        // Functions for calculating Threat Level
        double calc_WL_depth(double WL);
        int calc_t_lvl(double depth, double WL, string LayerName);
        int threat_level(double depth);

        // Open Layers and filter them
        bool openLayers();

        // Utility for setting the color of the object
        void setColor(int t_lvl, string &color);

        void printPoints();
        void printPolygons();


    private: // Configuration variables

    private: // State variables
        unsigned int m_iterations;
        double       m_timewarp;
        double m_tide, m_MHW_Offset, m_ASV_draft;
        double N_lat, W_long, S_lat, E_long;
        vector<double> vect_tide;
        Geodesy geod;
        GDALDataset *ds_pnt, *ds_poly, *ds_line;
        OGRLayer *Point_Layer, *Poly_Layer, *Line_Layer;
        OGRPolygon *m_filter;
        bool first_print, m_ENC_INT, openned, print_all, UTM;
};

#endif 
