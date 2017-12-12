/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: BHV_polyOA.cpp                                  */
/*    DATE: Oct 31, 2017                                    */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_polyOA.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_polyOA::BHV_polyOA(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "OA_poly");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("Full_360_polys, NAV_HEADING, DESIRED_HEADING");

  // Set global variables
  bias_Heading = 0;
  inside = false;
  minDist_Flag = false;
  swathSize = 5;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_polyOA::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_polyOA::onRunState()
{
    vector<string> polyInfo;
    string utilityString;
    double ASV_heading, des_heading;

    // Part 1: Build the IvP function
    IvPFunction *ipf = 0;

    // Part 2: Get information from the InfoBuffer
    bool ok1, ok2, ok3;
    utilityString = getBufferStringVal("Full_360_polys", ok1);
    bias_Heading = getBufferDoubleVal("NAV_HEADING", ok2);
    des_heading = getBufferDoubleVal("DESIRED_HEADING", ok3);

    if (ok1&&ok2)
    {
        polyInfo = parseString(utilityString, '!');

        if (polyInfo.size() == 4)
        {
            postMessage("minDist_Flag", minDist_Flag);
            utility = parseString(polyInfo[1],",");
            minDist_Flag = stoi(polyInfo[2].c_str())==1;
            swathSize = atoi(polyInfo[3].c_str());
            postMessage("SWATH", swathSize);
            ipf = buildZAIC_Vector();
        }
        else
            cout << "DEBUG " << "Should not be here..." << endl;
    }

    // Part N: Prior to returning the IvP function, apply the priority wt
    // Actual weight applied may be some value different than the configured
    // m_priority_wt, depending on the behavior author's insite.
    if(ipf)
        ipf->setPWT(m_priority_wt);

    return(ipf);
}

IvPFunction* BHV_polyOA::buildZAIC_Vector()
{
    IvPFunction *ivp_function = 0;
    double angle, util, m, b;
    //double prev_util = atof(utility[360/swathSize-1].c_str());
    //double prev_ang = -swathSize;
    double next_angle, next_util;

    // Declare which variable the domain is
    ZAIC_Vector head_zaic_v(m_domain, "course");

    // Used for the ZAIC_Vector function
    vector<double> domain_vals, range_vals;

    // Set the values for the angle (domain) and utility (range)
    for (int index = 0; index<utility.size(); index++)
    {
        angle = index*swathSize;
        next_angle = (index+1)*swathSize;
        util = atof(utility[index].c_str());

        // make sure to set the angle correctly for wraping
        if (index<utility.size()-1)
            next_util = atof(utility[index+1].c_str());
        else
            next_angle = (index+1)*swathSize;

        calc_m_b(angle, next_angle,util, next_util, m, b);
        // Linearly interpolate between the 5 degree heading rays. (Only nessisary due to a MOOS bug)
        for (int j=0; j<swathSize; j++)
        {
            angle++;
            domain_vals.push_back(angle);
            if (j==0)
                range_vals.push_back(setHeadingBias(util, angle));
            else
                range_vals.push_back(setHeadingBias(interpLin(m,b,angle), angle));
        }
    }

    double minUtil = round(*min_element(range_vals.begin(), range_vals.end()));
    if (minUtil<100)
    {
        postMessage("minUtil", minUtil);
        // Set the ZAIC domain and range
        head_zaic_v.setDomainVals(domain_vals);
        head_zaic_v.setRangeVals(range_vals);


        // Extract the IvP function
        ivp_function = head_zaic_v.extractIvPFunction();
    }


    /* Debugging MOOS Bug
    if (ivp_function!=0)
    {

        if (!(ivp_function->freeOfNan()))
        {
            cout << "DEBUG ";
            for (int i = 0; i<73; i++)
                cout << domain_vals[i] << ", " << range_vals[i] << "; ";
            cout << endl;
        }
    }
    */

    // Clear the domain and range values
    domain_vals.clear();
    range_vals.clear();

    return(ivp_function);
}

void BHV_polyOA::calc_m_b(double x1, double x2, double y1, double y2, double &m, double &b)
{
    if (x1==x2)
        m = 9999;
    else
        m = (y2-y1)/(x2-x1);

    b = y1-m*x1;
}

// This function set the heading bias for the current heading if you have hit the minimum distance (3*vessel_length)
double  BHV_polyOA::setHeadingBias(double utility, double angle)
{
    double alpha = 2;
    double realitive_heading=0;
    if (minDist_Flag)
    {
        // Realitive heading accounts for the cross over point at 360/0
        realitive_heading = abs(angle-bias_Heading);
        if (realitive_heading>180)
            realitive_heading-=360;

        return utility*(1-abs(realitive_heading)/(180*alpha));
    }
    else
    {
        return utility;
    }

}

double BHV_polyOA::interpLin(double m, double b, double angle)
{
    double util = (m*angle+b);
    if (util>100)
        util=100;
    return util;
}
