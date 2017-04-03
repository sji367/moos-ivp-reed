/*
 * BHV_OA_poly.h
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include <string>
#include <vector>
#include "IvPBehavior.h"
#include "AOF.h"
#include "poly.h"

using namespace std;

#ifndef BHV_OA_POLY_H_
#define BHV_OA_POLY_H_

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
	void getVertices(string, Poly&, Poly&, Poly&, vector<double>&);
	void getVertices(int i,string info, Poly& min_angle, Poly& min_dist, Poly& max_angle, vector<double>& max_cost);
	double calcBuffer(double cost);
	void calcVShape(double buffer_width, double (&OA_util)[360], Poly, Poly, Poly);
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
  {return new BHV_OA_poly(domain);}
}

#endif /* BHV_OA_POLY_H_ */
