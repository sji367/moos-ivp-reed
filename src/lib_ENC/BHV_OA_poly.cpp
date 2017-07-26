/*
 * BHV_OA_poly.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include "BHV_OA_poly.h"

#ifdef _WIN32
#   define _USE_MATH_DEFINES
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif

#include <cstdlib>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

// MOOS Libraries
#include "XYPolygon.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_Vector.h"
#include "BHV_OA_poly.h"
//---------------------------------------------------------------
// Constructor

BHV_OA_poly::BHV_OA_poly(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "ENC_OA_poly");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  //   Next_WPT --> Published by the Waypoint BHV
  //   Poly_Obs --> Published by ENC_Search
  addInfoVars("Next_WPT, Poly_Obs, NAV_SPEED, NAV_X, NAV_Y, NAV_HEADING, DESIRED_HEADING");

  m_maxutil = 100;
  m_v_length = 4;
  m_Desired_head = 0;
  m_Safety_Dist = 2.5;
  m_SafetyDistSet = false;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_OA_poly::setParam(string param, string val)
{
    // Convert the parameter to lower case for more general matching
    param = tolower(param);

    // Get the numerical value of the param argument for convenience once
    double double_val = atof(val.c_str());

    // ASV Length
    if((param == "vehicle_size") || (param == "vehicle_length" ) || (param == "asv_length" ) && (isNumber(val)))
    {
        m_v_length = double_val;
        postMessage("ASV_Length", double_val);
        return(true);
    }
    else if (param == "saftey_distance")
    {
        // If this is not set, then it will default to 3*(Vessel Length)
        m_SafetyDistSet = true;
        m_Safety_Dist = double_val;
        return true;
    }
    // If not handled above, then just return false;
    return(false);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_OA_poly::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;

  // Part 2a: Get information from the InfoBuffer
  bool ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8;
  m_obstacles = getBufferStringVal("Poly_Obs", ok1);
  m_WPT = getBufferStringVal("Next_WPT", ok2);
  m_speed = getBufferDoubleVal("NAV_SPEED", ok3);
  m_ASV_x = getBufferDoubleVal("NAV_X", ok4);
  m_ASV_y = getBufferDoubleVal("NAV_Y", ok5);
  m_ASV_head = getBufferDoubleVal("NAV_HEADING", ok6);
  m_Desired_head = getBufferDoubleVal("DESIRED_HEADING", ok7);

  vector<string> temp_WPT, result, ASV_info;

  // Check if there are new obstacles and speed and if there are, make a new IvPfunction
  if(!ok1)
    {
      postWMessage("No new obstacles info buffer.");
      return(0);
    }
  else if(!ok3)
    {
      postWMessage("Speed is not being defined.");
      return(0);
    }
  else if (!ok4 && !ok5)
    {
      postWMessage("ASV position is not being defined.");
      return(0);
    }
  else if (!ok6)
    {
      postWMessage("Heading is not being defined.");
      return(0);
    }
  else if (!ok7)
  {
      postWMessage("Desired Heading not yet defined.");
      return(0);
  }
  else
    {
      // Part 2b: Parse the obstacle information collected in the previous step
      // Parse the Waypoint Information
      if (ok2 and m_WPT!="first_point")
	{
	  temp_WPT = parseString(m_WPT, ',');
	  m_WPT_x = stoi(temp_WPT[0]);
	  m_WPT_y = stoi(temp_WPT[1]);
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   # of Obstacles:t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!...
      result = parseString(m_obstacles, ':');

      // Parse the number of obstacles
      m_num_obs = stoi(result[1]);

      // Store the information on the obstacle if there are the right amount of parts in the Poly_obstacle string
      if (result.size()==3)
	{
	  m_obs_info = result[2];
	  ipf = buildZAIC_Vector();
	}

    }
  // Part 3: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

/***************************************************************************/
/* This function parses the information on the polygon obstacles and then  */
/*   uses that information to create an IvPFunction using the ZAIC Vector  */
/*   tools. This function takes into account all polygons in the search    */
/*   area and does a linear interpolation between the minimum angle,       */
/*   maximum cost, and maximum angle points.                               */
/***************************************************************************/

IvPFunction *BHV_OA_poly::buildZAIC_Vector()
{
  IvPFunction *ivp_function = 0;
  vector<Poly> min_ang, max_ang, min_dist;
  vector<double> max_cost;
  vector<string> info;
  double buffer_width = 0;
  bool inside;

  // To start with, fill an array with maxiumum utility
  vector<double> OA_util (360, m_maxutil);

  // Clear all vectors
  min_ang.clear();
  min_dist.clear();
  max_ang.clear();
  info.clear();
  max_cost.clear();

  //  Then separate the obstacles from one another
  info = parseString(m_obs_info, '!');
  for (int i=0; i<info.size(); i++)
    {
      // Make new polygon vertices
      Poly polygon_min_ang = Poly();
      Poly polygon_min_dist = Poly();
      Poly polygon_max_ang = Poly();

      // Push them to the back of the vector
      min_ang.push_back(polygon_min_ang);
      min_dist.push_back(polygon_min_dist);
      max_ang.push_back(polygon_max_ang);

      getVertices(i, info[i], min_ang[i], min_dist[i], max_ang[i], max_cost, inside);
      buffer_width = calcBuffer(min_dist[i].getCost());
      calcVShape(buffer_width, OA_util, min_ang[i], min_dist[i], max_ang[i], inside);
    }
  ivp_function  = setIVP_domain_range(OA_util);
  return ivp_function;
}


void BHV_OA_poly::getVertices(int i,string info, Poly& min_angle, Poly& min_dist, Poly& max_angle, vector<double>& max_cost, bool &inside)
{
  // Vectors holding the parsed information on the obstacles
  vector<string> poly_info, poly_gen_info, poly_ang_info;

  // Deals with the 0/360 cross over. If the angles of the "V" cross over
  //  the 0/360 boundary, then the domain will be
  //    [-buffer_0_360, 360-buffer_0_360]
  double buffer_0_360;

  // Parse the individual obstacles
  poly_info = parseString(info, '@');

  // General information on obstacle
  //    Type of obstacle and threat level
  poly_gen_info = parseString(poly_info[0], ',');

  // Information on the angles
  //    minimum angle, angle of minimum distance, maximum angle
  poly_ang_info = parseString(poly_info[1], ',');

  // Store if the ASV is inside a polygon
  postMessage("INSIDE", poly_ang_info[0] == "1");
  inside = (poly_ang_info[0] == "1");

  // Build new objects holding the information for "V" shaped function
  min_angle.setXY(strtod(poly_ang_info[2].c_str(), NULL),strtod(poly_ang_info[3].c_str(), NULL));
  min_dist.setXY(strtod(poly_ang_info[4].c_str(), NULL),strtod(poly_ang_info[5].c_str(), NULL));
  max_angle.setXY(strtod(poly_ang_info[6].c_str(), NULL),strtod(poly_ang_info[7].c_str(), NULL));
  postMessage("MAX_ANGLE", to_string(max_angle.getX())+","+ to_string(max_angle.getY()));

  // Set the reference frame, threat level and the obstacle type
  min_angle.setStatics(0, stoi(poly_gen_info[0]), poly_gen_info[1]);
  min_dist.setStatics(0, stoi(poly_gen_info[0]), poly_gen_info[1]);
  max_angle.setStatics(0, stoi(poly_gen_info[0]), poly_gen_info[1]);

  // Determine the maximum extent angle, which will be used as a buffer
  buffer_0_360 = relAng(m_ASV_x, m_ASV_y, max_angle.getX(), max_angle.getY());
  
  // Calculate and set the angle for the critical points
  min_angle.setAngle(relAng(m_ASV_x, m_ASV_y, min_angle.getX(), min_angle.getY()), buffer_0_360, inside);
  min_dist.setAngle(relAng(m_ASV_x, m_ASV_y, min_dist.getX(), min_dist.getY()), buffer_0_360, inside);
  max_angle.setAngle(relAng(m_ASV_x, m_ASV_y, max_angle.getX(), max_angle.getY()), buffer_0_360, inside);

  // Calculate and set the cost and distance of the the critical points "V"
  min_angle.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);
  min_dist.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);
  max_angle.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);

  // Calculate and set the slope and y-intercept of the the critical points "V"
  min_angle.setMB_Left_pnt(min_dist.getCost(), min_dist.getAngle());
  max_angle.setMB_Right_pnt(min_dist.getCost(), min_dist.getAngle());

  //Store the maximum cost
  max_cost.push_back(min_dist.getCost());
}

// The buffer distance is to make sure that the ASV avoids the obstacle
//  with some buffer
//  Starts at 20 degrees and if the cost is bigger than 70, then the
//  buffer is increased to a maximum of 100
double BHV_OA_poly::calcBuffer(double cost)
{
  double temp_buff = 0;

  double buffer_width = 20;

  // If the maximum cost is greater than 100, increase the size of the
  //  safety buffer
  if (cost > 100)
    {
      temp_buff = floor(4*cost/m_maxutil);
      if (temp_buff >60)
        temp_buff =60;
      buffer_width += temp_buff;
    }

   postMessage("Buffer_Width", buffer_width);

  return buffer_width;
}

void BHV_OA_poly::calcVShape(double buffer_width, vector<double> &OA_util, Poly min_angle, Poly min_dist, Poly max_angle, bool &inside)
{
    int buff_high, buff_low;
    double calculated_cost, actual_cost;

    int safety_buff = 90;
    int cur_ang = 0;
    int t_lvl = min_dist.getTLvL();

    double utility = 0;
    postMessage("Angles",to_string(min_angle.getAngle())+","+to_string(min_dist.getAngle())+","+to_string(max_angle.getAngle()));
    postMessage("Angles1",to_string(min_angle.getAngle()-buffer_width)+", "+to_string(min_dist.getAngle())+", "+to_string(max_angle.getAngle()+buffer_width));
    postMessage("Util1",to_string(min_angle.getCost())+","+to_string(min_dist.getCost())+","+to_string(max_angle.getCost()));
    postMessage("DIST1", to_string(min_angle.getDist())+", "+to_string(min_dist.getDist())+", "+to_string(max_angle.getDist()));

    // If the ASV is inside of the obstacle, then try to leave it A.S.A.P.
    if (inside)
    {
        double start_ang, end_ang;
        string Case;
        // 2 cases:
        //  1) min_ang > max_ang
        //  2) min_ang < max_ang

        // Case 1: (min_ang > max_ang)
        if (min_angle.getAngle()>max_angle.getAngle())
        {
            start_ang = min_angle.getAngle();
            end_ang = max_angle.getAngle()+360;
            Case = ", 1";
        }
        // Case 2: (min_ang < max_ang)
        else
        {
            start_ang = min_angle.getAngle();
            end_ang = max_angle.getAngle();
            Case = ", 2";
        }

        postMessage("RANGES", to_string(start_ang)+", "+to_string(end_ang)+Case);
        for (double i=start_ang; i<end_ang; i++)
        {
            // Make sure the angle is in the domain[0,360]
            cur_ang = int(round(fmod(i,360)));
            if (cur_ang<0)
                cur_ang+360;
            if (utility<OA_util[cur_ang])
                OA_util[cur_ang]=utility;
        }
    }
    else
    {
        // If you are close to the obstacle place a safety buffer of atleast +/- 90 degrees around the closest point
        //    only if the ASV is not inside another polygon
        if (!m_SafetyDistSet)
            m_Safety_Dist = 3*m_v_length;
        double safety_distance = m_Safety_Dist+.5*t_lvl;
        postMessage("DIST", min_dist.getDist());
        if (min_dist.getDist() <= (safety_distance))
        {
            // This function initializes the utility to a gaussian centered at the previous desired heading.
            //  and only is called when the ASV is close to an obstacle. This makes it more difficult for
            //  the ASV to make large changes in heading which can allow the ASV to crash into the obstacle
            //  that is being avoided
            gaussianAroundDesHead(OA_util, m_maxutil);

            /*
            // Increase the NoGo region as the ASV gets closer to the obstacle
            if (min_dist.getDist()<safety_distance/2)
            {
                safety += (safety_distance/2-min_dist.getDist())*10;
                postMessage("SAFETY", (safety_distance/2-min_dist.getDist())*10);
            }
            */
            for (int i = 0; i<safety_buff; i++)
            {
                // Update the current buffer angles (Domain = [-360,360])
                buff_low = (int)floor(fmod(min_dist.getAngle()-i,360));
                buff_high = (int)floor(fmod(min_dist.getAngle()+i,360));

                // Make sure the angles are not negative (Domain = [0,360])
                if (buff_low < 0)
                    buff_low += 360;
                if (buff_high < 0)
                    buff_high += 360;

                // Set the OA Utility function
                if (utility<OA_util[buff_low])
                    OA_util[buff_low] = utility;
                if (utility<OA_util[buff_high])
                    OA_util[buff_high] = utility;
            }
        }
        // Make sure the minimum angle and maximum angle (when accounting for the buffer width) does not wrap
        //if (min_angle.getAngle()-buffer_width)

        // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> min angle to max cost
        // Makes the first half of the "V Shaped" penalty function
        for (double x1 = min_angle.getAngle()-buffer_width; x1<=min_dist.getAngle(); x1++)
        {
            // Update the current angle (Domain = [-360,360])
            cur_ang = (int)floor(fmod(x1,360));

            // Make sure it is not negative (Domain = [0,360])
            if (cur_ang<0)
            cur_ang+=360;

            // Deal with slope being inf
            //  (if max angle = min dist angle) --> set the cost to the cost
            //       of the mininum distance point.
            if (min_angle.getM() == 999)
                calculated_cost = min_dist.getCost();
            else
                calculated_cost = (min_angle.getM()*x1+min_angle.getB());

            // Set a maximum threshold on the cost.
            if (calculated_cost > m_maxutil)
                actual_cost = m_maxutil;
            else if (calculated_cost < 0)
                actual_cost = 0;
            else
                actual_cost = calculated_cost;

            utility = m_maxutil-actual_cost;

            // If the current utility value (the one that was just calculated)
            //  is less than the previously stored value, store the new one
            if (utility<OA_util[cur_ang])
                OA_util[cur_ang]=floor(utility);
        }

        // This calculates the utility and stores that value if it is less than
        //  the current utility for all obstacles --> max cost to max angle
        // Makes the first half of the "V Shaped" penalty function
        for (double x2 = min_dist.getAngle(); x2<=max_angle.getAngle()+buffer_width; x2++)
        {
            // Update the current angle  (Domain = [-360,360])
            cur_ang = (int)floor(fmod(x2,360));

            // Make sure it is not negative (Domain = [0,360])
            if (cur_ang < 0)
            cur_ang += 360;

            // Deal with slope being inf
            //  (if max angle = min dist angle) --> set the cost to the cost
            //       of the mininum distance point.
            if (max_angle.getM() == 999)
            calculated_cost = min_dist.getCost();
            else
            calculated_cost = (max_angle.getM()*x2+max_angle.getB());

            // Set a maximum threshold on the cost.
            if (calculated_cost < 0)
                actual_cost = 0;
            else if (calculated_cost > m_maxutil)
                actual_cost = m_maxutil;
            else
                actual_cost = calculated_cost;

            utility = m_maxutil-actual_cost;

            // If the current utility value (the one that was just calculated)
            //  is less than the previously stored value, store the new one
            if (utility<OA_util[cur_ang])
                OA_util[cur_ang]=floor(utility);
        }
    }
}


IvPFunction* BHV_OA_poly::setIVP_domain_range(vector<double> &OA_util)
{
  IvPFunction *ivp_function;

  // Declare which variable the domain is
  ZAIC_Vector head_zaic_v(m_domain, "course");

  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;
  int range_min;

  domain_vals.push_back(0);
  range_vals.push_back((int)floor((OA_util[359]+OA_util[1])/2));
  // Set the values for the angle (domain) and utility (range)
  for (int iii = 1; iii<360; iii++)
    {
      domain_vals.push_back(iii);
      range_vals.push_back(int(OA_util[iii]));
    }
  // Set 360 degrees
  domain_vals.push_back(360);
  range_vals.push_back((int)floor((OA_util[359]+OA_util[1])/2));

  range_min = *min_element(range_vals.begin(), range_vals.end());
  postMessage("Min_Util", range_min);

  // Set the ZAIC domain and range
  head_zaic_v.setDomainVals(domain_vals);
  head_zaic_v.setRangeVals(range_vals);

  // Clear the domain and range values
  domain_vals.clear();
  range_vals.clear();

  // Extract the IvP function
  ivp_function = head_zaic_v.extractIvPFunction();

  return(ivp_function);
}

// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
void BHV_OA_poly::Update_Lead_Param(vector<double> vect_max_cost)
{
  double lead;
  double max_cost = *max_element(vect_max_cost.begin(), vect_max_cost.end());;
  // Set the lead waypoint parameter to a high number (150) if cost > 75
  if (max_cost > 75)
    lead=150;

  // Increases linearly between 8 and 100 as the cost increases
  else if (max_cost > 14)
    lead = 2*(max_cost-14)+8;

  // If cost is small (>= 14) keep the nominal lead value
  else
    lead=8;

  postMessage("WPT_UPDATE", "lead="+to_string(lead));
}

// This function initializes the utility to a gaussian centered at the previous desired heading.
//  and only is called when the ASV is close to an obstacle. This makes it more difficult for
//  the ASV to make large changes in heading which can allow the ASV to crash into the obstacle
//  that is being avoided
void BHV_OA_poly::gaussianAroundDesHead(vector<double> &OA_util, double amplitude)
{
    double sigma = 153; // Puts the point 180 degrees from the previous heading at approximately half of the amplitude
    double gauss_util;
    int cur_ang;
    for (int i=0; i<OA_util.size(); i++)
    {
        // Account for the 0/360 cross over
        if ((m_Desired_head<180)&&(i>m_Desired_head+180))
            cur_ang = i-360;
        else if ((m_Desired_head>180)&&(i<m_Desired_head-180))
            cur_ang = i+360;
        else
            cur_ang =i;

        gauss_util = calc_Gaussian(m_Desired_head,cur_ang,sigma,amplitude);
        if (gauss_util<OA_util[i])
            OA_util[i] =gauss_util;
    }
}

double BHV_OA_poly::calc_Gaussian(double x, double mu, double sigma, double amplitude)
{
    return pow(M_E, -(pow((x - mu),2)/(2*(sigma * sigma))))*amplitude;
}





