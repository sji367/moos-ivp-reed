/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA.cpp                                      */
/*    DATE: June 2016                                       */
/*                                                          */
/* This application uses the information from the ENC search*/
/*   program to output a gausian depression in the IvP      */
/*   function using the ZAIC tool based upon the point      */
/*   obstacles.                                             */
/************************************************************/

#ifdef _WIN32
#   define _USE_MATH_DEFINES
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif

#include <iterator>
#include <cstdlib>
#include <vector>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

// MOOS Libraries
//#include "OF_Coupler.h" // Reflector Tools
//#include "OF_Reflector.h" // Reflector Tools
//#include "AOF_Gauss.h" // Reflector application
//#include "ZAIC_PEAK.h" // ZAIC Peak version
#include "XYPolygon.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_Vector.h"
#include "BHV_OA.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_OA::BHV_OA(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "ENC_OA");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("Next_WPT, Obstacles, NAV_SPEED, NAV_X, NAV_Y, NAV_HEADING, ASV_length");

  // Initialize Globals
  m_v_length = 4;
  m_maxutil = 100;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_OA::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "vehicle_size"|| param == "vehicle_length") && isNumber(val)) {
    m_v_length = double_val;
    return(true);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_OA::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  
  // Part 1a: Get information from the InfoBuffer
  vector<string> temp_WPT, result, ASV_info;
  
  bool ok1, ok2, ok3, ok4, ok5, ok6, ok7;
  m_obstacles = getBufferStringVal("Obstacles", ok1);
  m_WPT = getBufferStringVal("Next_WPT", ok2);
  m_speed = getBufferDoubleVal("NAV_SPEED", ok3);
  m_ASV_x = getBufferDoubleVal("NAV_X", ok4);
  m_ASV_y = getBufferDoubleVal("NAV_Y", ok5);
  m_ASV_head = getBufferDoubleVal("NAV_HEADING", ok6);
  m_v_length = getBufferDoubleVal("ASV_length", ok7);
  
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
      
      // Part 1b: Parse the obstacle information collected in the previous step
      // Parse the Waypoint Information
      if (ok2)
	{
	  temp_WPT = parseString(m_WPT, ',');
	  m_WPT_x = (int)floor(strtod(temp_WPT[0].c_str(), NULL));
	  m_WPT_y = (int)floor(strtod(temp_WPT[1].c_str(), NULL));
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   ASV_X,ASV_Y,heading:# of Obstacles:x,y,t_lvl,type!x,y,t_lvl,type!...
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

      // Store the information on the obstacle
      if (result.size()==3)
	{
	  m_obs_info = result[2];
	  ipf = buildZAIC_Vector();
	}      
    }
  // Part 2: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf){
    ipf->setPWT(m_priority_wt);
  }

  return(ipf);
}

/***************************************************************************/
/* This function parses the obstacles and then uses that information to    */
/*   create an IvPFunction using the ZAIC Vector tools. This function is   */
/*   the most functional out of the three because it uses a Gaussian       */
/*   distribution to describe how the utility falls off as a function of   */
/*   angle and it takes into account all obstacles in the search area.     */
/***************************************************************************/
IvPFunction *BHV_OA::buildZAIC_Vector()
{
  IvPFunction *ivp_function = 0;
  double max_cost, dist, util;
  string obs_type;
  int obs_t_lvl;
  vector<double> cost,obs_x, obs_y;
  vector<string> info, x, y, ind_obs_info;
  vector<int> ang;
  string str, poly, obs_pos;
  
  // For loop iterator
  //  (Sets each value of the gaussian depression for each obstacle)
  int k; 

  // Information on the obstacle
  int width, cur_ang;
  double amplitude, sigma;
  
  // This is the utility for the OA procedure for a particular
  // angle in the gaussian window which describes how the cost
  // falls off from directly towards the obstacle  
  double utility;
  double penalty; // maximum_utility - calc_utility
  
  // To start with, fill an array with maxiumum utility
  double OA_util[360];
  fill(OA_util,OA_util+360, m_maxutil);
  
  //  First seperate the obstacles from one another
  info = parseString(m_obs_info, '!');
  for (unsigned int i=0;i<info.size(); i++)
    {
      // Clear the vectors
      ind_obs_info.clear();
      x.clear();
      y.clear();
      
      // Parse the individual obstacles
      ind_obs_info = parseString(info[i], ',');

      // Convert the strings to doubles and ints
      x = parseString(ind_obs_info[0], '=');
      obs_x.push_back(strtod(x[1].c_str(), NULL)); // Get rid of the 'x='
      y = parseString(ind_obs_info[1], '=');
      obs_y.push_back(strtod(y[1].c_str(), NULL)); // Get rid of the 'y='
      obs_t_lvl = (int)floor(strtod(ind_obs_info[2].c_str(), NULL));
    
      // Type of obstacle
      obs_type = ind_obs_info[3];

      // Calculate the angle and cost for the obstacle
      ang.push_back(relAng(m_ASV_x, m_ASV_y, obs_x[i], obs_y[i]));
      dist = sqrt(pow(m_ASV_x-obs_x[i],2) +pow(m_ASV_y-obs_y[i],2));

      // Make sure you dont divide by zero - if the distance to the object is 
      //  less than 1, set it equal to 1
      if (dist <1)
	dist = 1;
      cost.push_back(Calc_Cost(obs_t_lvl,dist));

      // If the cost is greater than 0, then calculate the gaussian depression
      if (cost[i] > 0)
	{
	  // Set a limit for the cost as well as set sigma and the width 
	  if (cost[i] > m_maxutil) 
	    {
	      amplitude = m_maxutil;
	      width = (int)floor(20*cost[i]/m_maxutil);
	      sigma = 8+width/8;
	    }
	  else
	    {
	      amplitude = cost[i];
	      width = 20;
	      sigma = 8;
	    }

	  // This calculates the utility of the Gaussian window
	  // function and stores that value if it is less than the
	  // current utility for all obstacles 
	  for (k = 0; k< (2*width+1); k++)
	    {
	      // Calculate the angle that will be used in the gaussian window
	      cur_ang = (int)floor((int)(ang[i]-(width)+k)%360);
	      if (cur_ang < 0)
		cur_ang += 360;

	      // Calculate the Gaussian Depression Penalty Fuction
	      penalty = Calc_Gaussian(cur_ang, ang[i], sigma, amplitude);
	      utility = m_maxutil - penalty;

	      // If the current utility value is less than the one for
	      // the gaussian window then store the one for the
	      // Gaussian window 
	      if (utility<OA_util[cur_ang])
		OA_util[cur_ang]=utility;
	    }
	}
    }
  
  max_cost = *max_element(cost.begin(), cost.end());
  postMessage("Max_cost", doubleToString(max_cost));
  
  ZAIC_Vector head_zaic_v(m_domain, "course");
  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;

  // for loop iterator (set domain and range for ZAIC)
  int ii; 
  if (max_cost != 0)
    {
      // Set the values for the angle (domain) and utility (range)
      domain_vals.push_back(ii); range_vals.push_back(OA_util[ii]);
      for (ii = 1; ii<359; ii++)
	{
	  //if (OA_util[ii] != OA_util[ii-1])
	  //{
	      domain_vals.push_back(ii);
	      range_vals.push_back(OA_util[ii]);
	      //}
	}
      domain_vals.push_back(ii+1); range_vals.push_back(OA_util[ii+1]);

      head_zaic_v.setDomainVals(domain_vals);
      head_zaic_v.setRangeVals(range_vals);

      // Clear the domain and range values
      domain_vals.clear();
      range_vals.clear();

      // Extract the IvP function
      ivp_function = head_zaic_v.extractIvPFunction();
      
      // The lead parameter sets the distance from the perpendicular 
      //  intersection of the ASV's current location and the trackline that 
      //  the waypoint behavior steers toward.
      Update_Lead_Param(max_cost);
    }
  return(ivp_function);
}

// This function calcuates the cost of the individual points
//  Domain of the returned answer is [0, m_maxutil]
double BHV_OA::Calc_Cost(int t_lvl, double dist)
{
  return pow((t_lvl/dist*m_v_length/m_speed*4.5),3)*m_maxutil;
}

// This function calcuates a 1D gaussian given a point, mean, and standard
//   deviation. (M_E is e=2.71828183)
double BHV_OA::Calc_Gaussian(double x, double mu, double sigma, double amplitude)
{
  return pow(M_E, -(pow((x - mu),2)/(2*(sigma * sigma))))*amplitude;
}

// The lead parameter sets the distance from the perpendicular intersection
//  of the ASV's current location and the trackline that the waypoint
//  behavior steers toward.
void BHV_OA::Update_Lead_Param(double max_cost)
{
  double lead;
  
  // Set the lead waypoint parameter to a high number (150) if cost > 75
  if (max_cost > 75)
    lead=150;
  
  else if (max_cost > 14)
    // Increases linearly between 8 and 100 as the cost increases
    lead = 2*(max_cost-14)+8;
  
  // If cost is small (>= 14) keep the nominal lead value    
  else 
    lead=8;

  postMessage("WPT_UPDATE", "lead="+doubleToString(lead));
}
