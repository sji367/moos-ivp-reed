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
  addInfoVars("Next_WPT, Poly_Obs, NAV_SPEED, NAV_X, NAV_Y, NAV_HEADING, ASV_length");

  m_maxutil = 100;
  m_v_length = 4;
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
  if((param == "vehicle_length")||(param == "vehicle_size")) {
    m_v_length = double_val;
    return(true);
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
  bool ok1, ok2, ok3, ok4, ok5, ok6, ok7;
  m_obstacles = getBufferStringVal("Poly_Obs", ok1);
  m_WPT = getBufferStringVal("Next_WPT", ok2);
  m_speed = getBufferDoubleVal("NAV_SPEED", ok3);
  m_ASV_x = getBufferDoubleVal("NAV_X", ok4);
  m_ASV_y = getBufferDoubleVal("NAV_Y", ok5);
  m_ASV_head = getBufferDoubleVal("NAV_HEADING", ok6);
  m_v_length = strtod(getBufferStringVal("ASV_length", ok7).c_str(), NULL);

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
  else
    {
      if (!ok7)
	postWMessage("ASV length is not defined. Will use default of 4 meters.");

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

/*
IvPFunction *BHV_OA_poly::buildZAIC_Vector()
{
    IvPFunction *ivp_function = 0;
    vector<double> max_cost, costs, angles, dist;
    vector<string> info;
    double buffer_width = 0;

    double x, y, vertexCost;

    // To start with, fill an array with maxiumum utility
    vector<double> OA_util(360,m_maxutil);
    //fill(OA_util,OA_util+360, m_maxutil);


    // Clear all vectors
    info.clear();
    max_cost.clear();

    //  Then separate the obstacles from one another
    info = parseString(m_obs_info, '!');

    // Vectors holding the parsed information on the obstacles
    vector<string> poly_info, poly_gen_info, poly_vertex_info;

    int min_dist_index;

    for (int i=0; i<info.size(); i++)
    {
        // Clear the vector strings
        poly_info.clear(); poly_gen_info.clear(); poly_vertex_info.clear();
        // Clear the double strings
        costs.clear(); angles.clear(); dist.clear();

        // Parse the individual obstacles
        poly_info = parseString(info[i], '@');

        // General information on obstacle
        //    Type of obstacle and threat level
        poly_gen_info = parseString(poly_info[0], ',');

        for (int j=1; j<poly_info.size(); j++)
        {
            // Angle and range for the vertices
            poly_vertex_info = parseString(poly_info[j], ',');
            if (poly_vertex_info.size() == 2)
            {
                x = atof(poly_vertex_info[0].c_str());
                y = atof(poly_vertex_info[1].c_str());

                // Calculate the vertex's cost
                //  Params - obstacles threat level (1) and distance from ASV (2)
                dist.push_back(calcDist2ASV(x,y));
                vertexCost = calcCost(atof(poly_gen_info[0].c_str()), dist.back());
                costs.push_back(vertexCost);
                max_cost.push_back(vertexCost);

                angles.push_back(relAng(m_ASV_x, m_ASV_y,x,y));
            }
        }
        if (angles.size()>0)
            interpolate(angles, costs, dist, OA_util);
    }
    if (max_cost.size() >0)
        Update_Lead_Param(max_cost);
    return setIVP_domain_range(OA_util);
}

double BHV_OA_poly::calcCost(double t_lvl, double dist)
{
    double cost = 0;
    // make sure you cannot divide by zero
    if (m_speed==0)
        cost=0;
    else
    {
        cost = pow((t_lvl/dist*m_v_length/m_speed*4.5),2)*m_maxutil;

        // Bound the cost
        if (cost>m_maxutil)
            cost = m_maxutil;
        else if (cost <0)
            cost = 0;
    }

    return cost;

}

void BHV_OA_poly::interpolate(vector<double> &angles, vector<double> &cost, vector<double> &dist, vector<double> &OA_util)
{
    double m,b;
    int ang1,ang2;
    int pos_ang;

    double util_interp, util1, util2;

    // If the ASV is close to the obstacle set a 180 degree buffer around the closest vertex
    int buff_low, buff_high;
    int safety = 120;
    int min_dist_index = min_element(dist.begin(), dist.end()) - dist.begin();
    if (dist[min_dist_index] <10)
    {
        for (int i = 0; i<safety; i++)
        {
            // Update the current buffer angles (Domain = [-360,360])
            buff_low = (int)floor(fmod(angles[min_dist_index]-i,360));
            buff_high = (int)floor(fmod(angles[min_dist_index]+i,360));

            // Make sure the angles are not negative (Domain = [0,360])
            if (buff_low < 0)
              buff_low += 360;
            if (buff_high < 0)
              buff_high += 360;

            // Set the OA Utility function
            OA_util[buff_low] = 0;
            OA_util[buff_high] = 0;
        }
    }
    else
    //{
        for (int i=0; i<angles.size()-1; i++)
        {
            ang1 = static_cast<int>(round(angles[i]));
            ang2 = static_cast<int>(round(angles[i+1]));

            util1 = m_maxutil - cost[i];
            util2 = m_maxutil - cost[i+1];

            // if the angles are the same store the lower of the two costs
            if (ang1 == ang2)
                setOA_util(ang1, min(util1, util2), OA_util);
            else if(abs(ang1-ang2)==1)
            {
                setOA_util(ang1, util1, OA_util);
                setOA_util(ang2, util2, OA_util);
            }

            else
            {
                // make sure that the angle doesnt cross 360/0 boundary
                if (abs(ang1-ang2)>200)
                {
                    if (ang1<ang2)
                        ang2 -= 360;
                    else if (ang1<ang2)
                        ang1 -= 360;
                }
                // Interpolate between the angles of the two vertices
                calc_m_b(ang1, util1, ang2, util2, m, b);
                if (ang1<ang2)
                {
                    for(int cur_ang=ang1; cur_ang<ang2; cur_ang++)
                    {
                        // Make sure that the angle used as the index is a positive number between the range [0,360)
                        pos_ang = cur_ang%360;
                        if (pos_ang<0)
                            pos_ang+=360;
                        util_interp = m*(1.0*cur_ang)+b;
                        setOA_util(pos_ang, util_interp, OA_util);
                    }
                }
                else
                {
                    for(int cur_ang=ang2; cur_ang<ang1; cur_ang++)
                    {
                        // Make sure that the angle used as the index is a positive number between the range [0,360)
                        pos_ang = cur_ang%360;
                        if (pos_ang<0)
                            pos_ang+=360;
                        util_interp = m*(1.0*cur_ang)+b;
                        setOA_util(pos_ang, util_interp, OA_util);
                    }
                }
            }

        }
    //}
}

void BHV_OA_poly::setOA_util(int ang, double util, vector<double> &OA_util)
{
    if (OA_util[ang] > util)
    {
        postMessage("interp", to_string(ang) +"( " + to_string(static_cast<int>(util))+ ")");
        OA_util[ang] = util;
    }
}

void BHV_OA_poly::calc_m_b(int x1, double y1, int x2, double y2, double &m, double &b)
{
    if (x1 != x2)
    {
        m = (y2 - y1)/(1.0*(x2 - x1));
        b = (y1) - (m*(1.0*x1));
    }
    else
    {
        // If the slope is infinite, make m and b 999
        m = 999;
        b = 999;
    }
}
*/
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

  // To start with, fill an array with maxiumum utility
  vector<double> OA_util (360, m_maxutil);
  //fill(OA_util,OA_util+360, m_maxutil);

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

      getVertices(i, info[i], min_ang[i], min_dist[i], max_ang[i], max_cost);
      
      buffer_width = calcBuffer(min_dist[i].getCost());
      
      calcVShape(buffer_width, OA_util, min_ang[i], min_dist[i], max_ang[i]);
    }
  Update_Lead_Param(max_cost);
  return setIVP_domain_range(OA_util);
}


void BHV_OA_poly::getVertices(int i,string info, Poly& min_angle, Poly& min_dist, Poly& max_angle, vector<double>& max_cost)
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

  // Build new objects holding the information for "V" shaped function
  min_angle.setXY(strtod(poly_ang_info[1].c_str(), NULL),strtod(poly_ang_info[2].c_str(), NULL));
  min_dist.setXY(strtod(poly_ang_info[3].c_str(), NULL),strtod(poly_ang_info[4].c_str(), NULL));
  max_angle.setXY(strtod(poly_ang_info[5].c_str(), NULL),strtod(poly_ang_info[6].c_str(), NULL));

  // Set the reference frame, threat level and the obstacle type
  min_angle.setStatics(stoi(poly_ang_info[0]), stoi(poly_gen_info[0]), poly_gen_info[1]);
  min_dist.setStatics(stoi(poly_ang_info[0]), stoi(poly_gen_info[0]), poly_gen_info[1]);
  max_angle.setStatics(stoi(poly_ang_info[0]), stoi(poly_gen_info[0]), poly_gen_info[1]);

  // Determine the 0/360 buffer
  buffer_0_360 = relAng(m_ASV_x, m_ASV_y, max_angle.getX(), max_angle.getY());

  postMessage("Min", buffer_0_360);
  
  // Calculate and set the angle for the critical points
  min_angle.setAngle(relAng(m_ASV_x, m_ASV_y, min_angle.getX(), min_angle.getY()),buffer_0_360);
  min_dist.setAngle(relAng(m_ASV_x, m_ASV_y, min_dist.getX(), min_dist.getY()),buffer_0_360);
  max_angle.setAngle(relAng(m_ASV_x, m_ASV_y, max_angle.getX(), max_angle.getY()),0);//buffer_0_360);

  postMessage("Angles", to_string(i)+": "+to_string(min_angle.getAngle())+", "+to_string(min_dist.getAngle())+", "+to_string(max_angle.getAngle()));

  // Calculate and set the cost and distance of the the critical points "V"
  min_angle.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);
  min_dist.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);
  min_dist.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);

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
  if (cost > 70)
    {
      temp_buff = floor(pow(2*cost/m_maxutil,3));
      if (temp_buff >100)
	temp_buff =100;
      buffer_width += temp_buff;
      postMessage("buffer_w", buffer_width);
    }

  return buffer_width;
}

void BHV_OA_poly::calcVShape(double buffer_width, vector<double> &OA_util, Poly min_angle, Poly min_dist, Poly max_angle)
{
  int buff_high, buff_low;
  double calculated_cost, actual_cost, utility;
  int safety = 90;
  int cur_ang = 0;
  // If you are close to the obstacle place a safety buffer of atleast +/- 90 degrees around the closest point
  if (min_dist.getDist() < 10)
    {
      postMessage("DIST", min_dist.getDist());
      for (int i = 0; i<safety; i++)
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
	  OA_util[buff_low] = 0;
	  OA_util[buff_high] = 0;
	}
    }

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


IvPFunction* BHV_OA_poly::setIVP_domain_range(vector<double> &OA_util)
{
  IvPFunction *ivp_function;

  // Declare which variable the domain is
  ZAIC_Vector head_zaic_v(m_domain, "course");

  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;

  int range_min;

  // Set the values for the angle (domain) and utility (range)
  for (int iii = 0; iii<360; iii++)
    {
      domain_vals.push_back(iii);
      range_vals.push_back((int)floor(OA_util[iii]));
    }

  range_min = *min_element(range_vals.begin(), range_vals.end());

  // Make sure to include the last point
  //domain_vals.push_back(iii+1); range_vals.push_back((int)floor(OA_util[iii+1]));

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




