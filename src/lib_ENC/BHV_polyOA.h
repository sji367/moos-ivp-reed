/************************************************************/
/*    NAME: Sam Reed                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_polyOA.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef polyOA_HEADER
#define polyOA_HEADER

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include "IvPBehavior.h"
#include "ZAIC_Vector.h"
#include "math.h"

using namespace std;

class BHV_polyOA : public IvPBehavior {
public:
    BHV_polyOA(IvPDomain);
    ~BHV_polyOA() {}

    bool         setParam(std::string, std::string);

    //---------------------------------------------------------------
    // Procedure: onSetParamComplete()
    //   Purpose: Invoked once after all parameters have been handled.
    //            Good place to ensure all required params have are set.
    //            Or any inter-param relationships like a<b
    void         onSetParamComplete() {}


    void         onCompleteState() {}

    //---------------------------------------------------------------
    // Procedure: onIdleState()
    //   Purpose: Invoked on each helm iteration if conditions not met.
    void         onIdleState() {}

    //---------------------------------------------------------------
    // Procedure: onHelmStart()
    //   Purpose: Invoked once upon helm start, even if this behavior
    //            is a template and not spawned at startup
    void         onHelmStart() {}

    //---------------------------------------------------------------
    // Procedure: postConfigStatus()
    //   Purpose: Invoked each time a param is dynamically changed
    void         postConfigStatus() {}

    //---------------------------------------------------------------
    // Procedure: onRunToIdleState()
    //   Purpose: Invoked once upon each transition from run to idle state
    void         onRunToIdleState() {}

    //---------------------------------------------------------------
    // Procedure: onIdleToRunState()
    //   Purpose: Invoked once upon each transition from idle to run state
    void         onIdleToRunState() {}

    //---------------------------------------------------------------
    // Procedure: onRunState()
    //   Purpose: Invoked each iteration when run conditions have been met.
    IvPFunction* onRunState();

protected: // Local Utility functions
    IvPFunction* buildZAIC_Vector();
    double interpLin(double m, double b, double angle);
    void calc_m_b(double x1, double x2, double y1, double y2, double &m, double &b);
    double setHeadingBias(double utility, double angle);

protected: // State variables
    bool inside, minDist_Flag;
    double bias_Heading;
    vector<string> utility;
    int swathSize;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_polyOA(domain);}
}
#endif
