/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA_poly.cpp                                 */
/*    DATE: June 2016                                       */
/************************************************************/

#ifdef _WIN32
#   define _USE_MATH_DEFINES
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif
 
#include <iterator>
#include <cstdlib>
#include <vector>
#include <sstream> // For stringstream
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

// MOOS Libraries
#include "XYPolygon.h"
#include "OF_Coupler.h"
#include "OF_Reflector.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_Vector.h"
#include "BHV_OA_poly.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_OA_poly::BHV_OA_poly(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "ENC_OA_poly");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  //   Next_WPT --> Published by the Waypoint BHV
  //   Poly_Obs --> Published by ENC_Search
  addInfoVars("Next_WPT, Poly_Obs, NAV_SPEED, NAV_X, NAV_Y, NAV_HEAD");

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
  m_ASV_head = getBufferDoubleVal("NAV_HEAD", ok6);
  m_v_length = getBufferDoubleVal("ASV_length", ok7);

  vector<string> temp_WPT, result, ASV_info;
  
  // Check if there are new obstacles and speed and if there are, make a new IvPfunction
  if(!ok1) {
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
	  m_WPT_x = (int)floor(strtod(temp_WPT[0].c_str(), NULL));
	  m_WPT_y = (int)floor(strtod(temp_WPT[1].c_str(), NULL));
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   ASV_X,ASV_Y,heading:# of Obstacles:t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!...
      result = parseString(m_obstacles, ':');
      
      /*
      // Parse ASV info
      ASV_info = parseString(result[0], ',');
  
      // Convert strings to doubles
      m_ASV_x = strtod(ASV_info[0].c_str(), NULL);
      m_ASV_y = strtod(ASV_info[1].c_str(), NULL);
      m_ASV_head = strtod(ASV_info[2].c_str(), NULL);
      */
      
      // Parse the number of obstacles
      m_num_obs = (int)floor(strtod(result[1].c_str(), NULL));
      
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
  if(ipf){
    ipf->setPWT(m_priority_wt);
  }

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
  
  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  
  // Declare which variable the domain is
  ZAIC_Vector head_zaic_v(m_domain, "course");

  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;
  
  // Fill the array with maxiumum utility
  double OA_util[360];
  fill(OA_util,OA_util+360, m_maxutil);

  // Holds the maximum cost for each obstacle which will be used in adjusting the lead parameter
  vector <double> max_cost;

  // Vectors holding the parsed information on the obstacles
  vector<string> poly_info, poly_gen_info, poly_ang_info, poly_dist_info;

  // Array holding the distance information (during the conversion to a double)
  double d[3];

  double utility, actual_cost, calculated_cost;
  int cur_ang;
  string x,y;
  
  double pnt_x, pnt_y;

  bool Debug = false;

  // The buffer distance is to make sure that the ASV avoids the obstacle with some buffer 
  int buffer_width, temp_buff;
  
  for (unsigned int i=0;i<m_num_obs; i++)
    {
      // Initialize vectors by clearing them
      poly_info.clear();
      poly_gen_info.clear();
      poly_ang_info.clear();
      poly_dist_info.clear();
      
      // Initialize the stucture holding the information on the obstacle
      poly_obs obstacle;

      // Parse the individual obstacles
      poly_info = parseString(info[i], '@');
      
      // Initialize attributes variable
      poly_attributes min_a, max_a, min_d;

      // General information on obstacle
      //    Type of obstacle and threat level
      poly_gen_info = parseString(poly_info[0], ',');
	  
      // Type of obstacle and threat level
      obstacle.t_lvl = (int)floor(strtod(poly_gen_info[0].c_str(), NULL));
      obstacle.obs_type = poly_gen_info[1];

      // Information on the angles
      //    minimum angle, maximum angle, angle of minimum distance
      poly_ang_info = parseString(poly_info[1], ',');
      
      // min_angle
      pnt_x = (strtod(poly_ang_info[4].c_str(), NULL));
      pnt_y = (strtod(poly_ang_info[5].c_str(), NULL));
      obstacle.min_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt_x, pnt_y));
      // Max angle
      pnt_x = (strtod(poly_ang_info[0].c_str(), NULL));
      pnt_y = (strtod(poly_ang_info[1].c_str(), NULL));
      obstacle.max_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt_x, pnt_y));
      // Max Cost
      pnt_x = (strtod(poly_ang_info[2].c_str(), NULL));
      pnt_y = (strtod(poly_ang_info[3].c_str(), NULL));
      obstacle.min_dist.ang = (relAng(m_ASV_x, m_ASV_y, pnt_x, pnt_y));
      
      // Information on the distance
      //    minimum angle distance, maximum angle distance, minimum distance
      poly_dist_info = parseString(poly_info[2], ',');
      
      // Convert distances to a double and check to see if they are less than 1
      for (int ii = 0; ii<3; ii++)
	{
	  d[ii] = (strtod(poly_dist_info[ii].c_str(), NULL));
	  if (d[ii]<1)
	    d[ii] = 1;
	}
      obstacle.min_ang.dist = d[0];
      obstacle.max_ang.dist = d[1];
      obstacle.min_dist.dist = d[2];

      // Determine the cost for each angle that you have information on
      obstacle.min_ang.cost = Calc_Cost(obstacle.t_lvl,obstacle.min_ang.dist);
      obstacle.max_ang.cost = Calc_Cost(obstacle.t_lvl,obstacle.max_ang.dist);
      obstacle.min_dist.cost = Calc_Cost(obstacle.t_lvl,obstacle.min_dist.dist);

      // Calculate the slope and y intercepts
      // Deal with slope being inf --> set it = to large number
      if (obstacle.min_ang.ang == obstacle.min_dist.ang)
	{
	  obstacle.min_ang.m = 999;
	}
      else
	obstacle.min_ang.m = (obstacle.min_ang.cost-obstacle.min_dist.cost)/(obstacle.min_ang.ang-obstacle.min_dist.ang);
      // Deal with slope being inf --> set it = to large number
      if (obstacle.max_ang.ang == obstacle.min_dist.ang)
	{
	  obstacle.max_ang.m = 999;
	}
      else
	obstacle.max_ang.m = (obstacle.min_dist.cost-obstacle.max_ang.cost)/(obstacle.min_dist.ang-obstacle.max_ang.ang);

      obstacle.min_ang.b = obstacle.min_ang.cost - obstacle.min_ang.m*obstacle.min_ang.ang;
      obstacle.max_ang.b = obstacle.max_ang.cost - obstacle.max_ang.m*obstacle.max_ang.ang;
      
      // Store the maximum cost for the obstacle so that it can be used later to adjust the lead parameter 
      max_cost.push_back(obstacle.min_dist.cost);

      // The buffer distance is to make sure that the ASV avoids the obstacle with some buffer
      buffer_width = 20;
      // If the maximum cost is greater than 100, increase the size of the saftey buffer
      if (obstacle.min_dist.cost > 100)
	{
	  temp_buff = floor(pow(2*obstacle.min_dist.cost/m_maxutil,2));
	  if (temp_buff >100)
	    temp_buff =100;
	  buffer_width += temp_buff;
	  postMessage("buffer_w", buffer_width);
	}

      // Debugging
      if Debug
	{
	  postMessage("Angles", doubleToString(obstacle.min_ang.ang+buffer_width)+", "+ doubleToString(obstacle.min_dist.ang)+", " +doubleToString(obstacle.max_ang.ang+buffer_width));
	  postMessage("Angles_no_buff", doubleToString(obstacle.min_ang.ang)+", "+ doubleToString(obstacle.min_dist.ang)+", " +doubleToString(obstacle.max_ang.ang));
	  postMessage("cost", doubleToString(obstacle.min_ang.cost)+", "+ doubleToString(obstacle.min_dist.cost)+", " +doubleToString(obstacle.max_ang.cost));
	}
      
      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> min angle to max cost
      // Makes the first half of the "V Shaped" penalty function
      for (double x1 = obstacle.min_ang.ang-buffer_width; x1<=obstacle.min_dist.ang; x1++)
	{
	  // Deal with slope being inf (if maximum angle = mininum distance angle) --> set the cost to the cost of the mininum distance point. 
	  if (obstacle.min_ang.m == 999)
	    calculated_cost = obstacle.min_dist.cost;
	  else
	    calculated_cost = (obstacle.min_ang.m*x1+obstacle.min_ang.b);

	  // Set a maximum threshold on the cost.
	  if (calculated_cost > m_maxutil)
	      actual_cost = m_maxutil;
	  else if (cost < 0)
	    calculated_cost = 0;
	  else
	    {
	      actual_cost = calculated_cost;
	    }
	  utility = m_maxutil-actual_cost;	  

	  // Update the current angle
	  cur_ang = (int)floor((int)x1%360);
	  if (cur_ang<0)
	    cur_ang+=360;

	  // If the current utility value (the one that was just calculated) is less than the previously stored value, store the new one
	  if (utility<OA_util[cur_ang])
	    OA_util[cur_ang]=floor(utility);
	}

      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> max cost to max angle
      // Makes the first half of the "V Shaped" penalty function
      for (double x2 = obstacle.min_dist.ang; x2<=obstacle.max_ang.ang+buffer_width; x2++)
	{
	  // Deal with slope being inf (if maximum angle = mininum distance angle) --> set the cost to the cost of the mininum distance point.
	  if (obstacle.max_ang.m == 999)
	    calculated_cost = obstacle.min_dist.cost;
	  else
	    calculated_cost = (obstacle.max_ang.m*x2+obstacle.max_ang.b);

	  // Set a maximum threshold on the cost. 
	  if (calculated_cost < 0)
	    actual_cost = 0;
	  else if (cost > m_maxutil)
	    actual_cost = m_maxutil;
	  else
	    actual_cost = calculated_cost;
	  
	  utility = m_maxutil-actual_cost;
	  
	  // Update the current angle (Domain = [-360,360])
	  cur_ang = (int)floor((int)x2%360);
	  
	  // Make sure it is not negative (Domain = [0,360])
	  if (cur_ang < 0)
	    cur_ang += 360;

	  // If the current utility value (the one that was just calculated) is less than the previously stored value, store the new one
	  if (utility<OA_util[cur_ang])
	    OA_util[cur_ang]=floor(utility);
	}
    }

  // Store the first value
  int iii = 0;
  domain_vals.push_back(iii); range_vals.push_back((int)floor(OA_util[iii]));

  // Set the values for the angle (domain) and utility (range)
  for (iii = 1; iii<359; iii++)
    {
      if (OA_util[iii] != (OA_util[iii-1]))
	{
	  domain_vals.push_back(iii);
	  range_vals.push_back((int)floor(OA_util[iii]));
	}
    }
  // Make sure to include the last point
  domain_vals.push_back(iii+1); range_vals.push_back((int)floor(OA_util[iii+1]));

  // Set the ZAIC domain and range
  head_zaic_v.setDomainVals(domain_vals);
  head_zaic_v.setRangeVals(range_vals);

  // Clear the domain and range values
  domain_vals.clear();
  range_vals.clear();
    
  // Extract the IvP function
  ivp_function = head_zaic_v.extractIvPFunction();
  
  // Find the maximum value in the max_cost vector, which holds the cost of
  //  the minimum distance point for each obstacle
  double maximum_cost_value = *max_element(max_cost.begin(), max_cost.end());
  postMessage("Max_Cost", maximum_cost_value);

  // The lead parameter sets the distance from the perpendicular intersection
  //  of the ASV's current location and the trackline that the waypoint
  //  behavior steers toward.
  Update_Lead_Param(maximum_cost_value)
  
  return(ivp_function);
}

// This function calcuates the cost of the individual points
double BHV_OA_poly::Calc_Cost(int t_lvl, double dist)
{
  return pow((t_lvl/dist*m_v_length/m_speed*4.5),3)*m_maxutil;
}

// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
void BHV_OA_poly::Update_Lead_Param(double max_cost)
{
  double lead;
  
  // Set the lead waypoint parameter to a high number (150) if cost > 75
  if (maximum_value > 75)
    lead=150;
  
  else if (maximum_value > 14)
    // Increases linearly between 8 and 100 as the cost increases
    lead = 2*(maximum_value-14)+8;
  
  // If cost is small (>= 14) keep the nominal lead value    
  else 
    lead=8;

  postMessage("WPT_UPDATE", "lead="+doubleToString(lead));
}
