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
  ~BHV_OA_poly() {}

  bool         setParam(std::string, std::string);
  void         onSetParamComplete() {}
  void         onCompleteState() {}
  void         onIdleState() {}
  void         onHelmStart() {}
  void         postConfigStatus() {}
  void         onRunToIdleState() {}
  void         onIdleToRunState() {}
  IvPFunction* onRunState();


protected: // Local Utility functions
	IvPFunction* buildZAIC_Vector();
        void getVertices(int, string, Poly&, Poly&, Poly&, vector<double>&, bool &inside);
	double calcBuffer(double cost);
        void calcVShape(double buffer_width, vector<double> &OA_util, Poly, Poly, Poly, bool &inside);
        IvPFunction* setIVP_domain_range(vector<double> &OA_util);
	void Update_Lead_Param(vector<double> vect_max_cost);

        void interpolate(vector<double> &angles, vector<double> &cost, vector<double> &dist, vector<double> &OA_util);
        void setOA_util(int ang, double util, vector<double> &OA_util);
        void calc_m_b(int x1, double y1, int x2, double y2, double &m, double &b);
        double calcCost(double t_lvl, double dist);
        double calcDist2ASV(double x, double y) {return sqrt(pow(m_ASV_x-x, 2)+pow(m_ASV_y-y, 2)); }
        double calc_Gaussian(double x, double mu, double sigma, double amplitude);
        void gaussianAroundDesHead(vector<double> &OA_util, double amplitude);

protected: // State variables
  string m_obstacles, m_obs_info, m_WPT;
  double m_ASV_x, m_ASV_y, m_ASV_head, m_speed, m_v_length, m_maxutil, m_Desired_head;
  int m_num_obs, m_WPT_x, m_WPT_y;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_OA_poly(domain);}
}

#endif /* BHV_OA_POLY_H_ */
