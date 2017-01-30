/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA_poly.h                                   */
/*    DATE: June 2016                                       */
/************************************************************/

#ifndef A_HEADER
#define A_HEADER

#include <string>
#include <tuple>
#include "IvPBehavior.h"
#include "AOF.h"
#include "../../../MOOS_V10.0.3_May1215/MOOSCore/Core/libMOOS/include/MOOS/Compatibility/Core/MOOSGenLib/ProcessConfigReader.h"

struct poly_attributes{
	double ang;
	double cost;
	double dist;
	double m;
	double b;
	double x;
	double y;
};

struct poly_obs {
	int t_lvl;
	string obs_type;
	int ref_frame;
	poly_attributes min_ang, max_ang, min_dist;
};

class BHV_OA_poly : public IvPBehavior {
public:
  BHV_OA_poly(IvPDomain);
  ~BHV_OA_poly() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete() {};
  void         onCompleteState() {};
  void         onIdleState() {};
  void         onHelmStart() {};
  void         postConfigStatus() {};
  void         onRunToIdleState() {};
  void         onIdleToRunState() {};
  IvPFunction* onRunState();
  

protected: // Local Utility functions
	IvPFunction* buildZAIC_Vector();
	double Calc_Cost(int t_lvl, double dist);
	void Update_Lead_Param(double max_cost);
	double calc_dist2ASV(double x, double y);
	tuple<double, double> calc_m_b(poly_attributes left_pt, poly_attributes right_pt, int ref_frame);
	double convert_ref_frame(double ang, int ref_frame);
	double calc_RelAngle(double x, double y);

protected: // Configuration parameters

protected: // State variables
  string m_obstacles, m_obs_info, m_WPT;
  double m_ASV_x, m_ASV_y, m_ASV_head, m_speed, m_v_length, m_maxutil;
  int m_num_obs, m_WPT_x, m_WPT_y;
	//poly_obs m_obstacle;
	
};

      // Initialize the stucture holding the information on the obstacle
      poly_obs obstacle;
      


#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_OA_poly(domain);}
}
#endif
