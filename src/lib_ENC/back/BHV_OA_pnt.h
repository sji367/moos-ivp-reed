/*
 * BHV_OA_pnt.h
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include <string>
#include <vector>
#include "IvPBehavior.h"
#include "AOF.h"
#include "point.h"

using namespace std;

#ifndef BHV_OA_PNT_H_
#define BHV_OA_PNT_H_

class BHV_OA_pnt : public IvPBehavior {
public:
  BHV_OA_pnt(IvPDomain);
  ~BHV_OA_pnt() {};

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
	void getPoint(string, Point&, vector<double>&);
	void calcBuffer(double& buffer_width, double cost);
	void calcGaussWindow(double (&OA_util)[360], Point&);
	double calc_Gaussian(double x, double mu, double sigma, double amplitude);
	IvPFunction* setIVP_domain_range(double OA_util[360]);
	void Update_Lead_Param(vector<double> vect_max_cost);

protected: // State variables
  string m_obstacles, m_obs_info, m_WPT;
  double m_ASV_x, m_ASV_y, m_ASV_head, m_speed, m_v_length, m_maxutil;
  int m_num_obs, m_WPT_x, m_WPT_y;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_OA_pnt(domain);}
}
#endif /* BHV_OA_PNT_H_ */
