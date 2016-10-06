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
#include <sstream> // For stringstream
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
  addInfoVars("Next_WPT, Obstacles, NAV_SPEED, NAV_X, NAV_Y, NAV_HEAD");

  // Initialize Globals
  m_v_size = 4;
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
  
  if((param == "vehicle_size") && isNumber(val)) {
    m_v_size = atof(val.c_str());
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

IvPFunction* BHV_OA::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  
  // Part 1a: Get information from the InfoBuffer
  bool ok1, ok2, ok3;
  m_obstacles = getBufferStringVal("Obstacles", ok1);
  m_WPT = getBufferStringVal("Next_WPT", ok2);
  m_speed = getBufferDoubleVal("NAV_SPEED", ok3);
  
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
  else
    {
      // Part 1b: Parse the obstacle information collected in the previous step
      // Parse the Waypoint Information
      if (ok2)
	{
	  vector<string> temp_WPT = parseString(m_WPT, ',');
	  m_WPT_x = (int)floor(strtod(temp_WPT[0].c_str(), NULL));
	  m_WPT_y = (int)floor(strtod(temp_WPT[1].c_str(), NULL));
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   ASV_X,ASV_Y,heading:# of Obstacles:x,y,t_lvl,type!x,y,t_lvl,type!...
      vector<string> result = parseString(m_obstacles, ':');

      // Parse ASV info
      vector<string> ASV_info = parseString(result[0], ',');
  
      // Convert strings to doubles
      m_ASV_x = strtod(ASV_info[0].c_str(), NULL);
      m_ASV_y = strtod(ASV_info[1].c_str(), NULL);
      m_ASV_head = strtod(ASV_info[2].c_str(), NULL);

      // Parse the number of obstacles
      m_num_obs = (int)floor(strtod(result[1].c_str(), NULL));

      // Store the information on the obstacle
      if (result.size()==3)
	{
	  m_obs_info = result[2];
	  //ipf = buildIvPFunction(); 
	  //ipf = buildFunctionWithZAIC();
	  ipf = buildZAIC_Vector();
	}
      /*
      // This piece is only nessisary for the buildIvPFunction()
      else
	postMessage("VIEW_POLYGON", "x=5000,y=5000,format=radial,radius=30,pts=3,edge_color=darkviolet,label=obs");
      */
      
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
  vector<int> ang;
  string str, poly, obs_pos;
  
  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  for (unsigned int i=0;i<info.size(); i++)
    {
      // Parse the individual obstacles
      vector<string> ind_obs_info = parseString(info[i], ',');

      // Convert the strings to doubles and ints
      vector<string> x = parseString(ind_obs_info[0], '=');
      obs_x.push_back(strtod(x[1].c_str(), NULL)); // Get rid of the 'x='
      vector<string> y = parseString(ind_obs_info[1], '=');
      obs_y.push_back(strtod(y[1].c_str(), NULL)); // Get rid of the 'y='
      obs_t_lvl = (int)floor(strtod(ind_obs_info[2].c_str(), NULL));
    
      // Type of obstacle
      obs_type = ind_obs_info[3];

      // Calculate the angle and cost for the obstacle
      ang.push_back(relAng(m_ASV_x, m_ASV_y, obs_x[i], obs_y[i]));
      dist = sqrt(pow(m_ASV_x-obs_x[i],2) +pow(m_ASV_y-obs_y[i],2));

      // Make sure you dont divide by zero - if it is less than 1, set it 
      //  equal to 1
      if (dist <1)
	dist = 1;
      cost.push_back(Calc_Cost(obs_t_lvl,dist));
    
    }

  ZAIC_Vector head_zaic_v(m_domain, "course");
  
  max_cost = *max_element(cost.begin(), cost.end());
  postMessage("Max_cost", doubleToString(max_cost));
  double lead;
  // Remove the lead waypoint parameter if cost > .5
  if (max_cost > .56)
    postMessage("WPT_UPDATE", "lead=50");
  else if (max_cost > .14)
    {
      // Should increase linearly between 8 and 50 as the cost increases
      lead = (max_cost-.14)*100+8; 
      postMessage("WPT_UPDATE", "lead="+doubleToString(lead));
    }
  else
    postMessage("WPT_UPDATE", "lead=8");
  
  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;
  
  // Fill the array with maxiumum utility
  double OA_util[360];
  fill(OA_util,OA_util+360, m_maxutil);

  // Information on the obstacle
  int width, cur_ang;

  double amplitude, sigma;
  if (max_cost != 0)
    {
      for (unsigned int ii=0;ii<cost.size(); ii++)
	{
	  
	  // Set a limit for the cost as well as set sigma and the width 
	  if (cost[ii] > m_maxutil) 
	    {
	      amplitude = m_maxutil;
	      width = (int)floor(20*cost[ii]/m_maxutil);
	      sigma = 8+width/8;
	    }
	  else
	    {
	      amplitude = cost[ii];
	      width = 20;
	      sigma = 8;
	    }

	  // This is the utility for the OA procedure for a particular
	  // angle in the gaussian window which describes how the cost
	  // falls off from directly towards the obstacle  
	  double utility;

	  double penalty;

	  // This calculates the utility of the Gaussian window
	  // function and stores that value if it is less than the
	  // current utility for all obstacles 
	  for (int k = 0; k< (2*width+1); k++)
	    {
	      // Calculate the angle that will be used in the gaussian window
	      cur_ang = ang[ii]-(width)+k;
	      if (cur_ang < 0)
		cur_ang += 360;
	      if (cur_ang >= 360)
		cur_ang += -360;

	      // Calculate the Gaussian Depression Penalty Fuction
	      penalty = Calc_Gaussian(cur_ang, ang[ii], sigma, amplitude);
	      utility = m_maxutil - penalty;

	      // If the current utility value is less than the one for
	      // the gaussian window then store the one for the
	      // Gaussian window 
	      if (utility<OA_util[cur_ang])
		  OA_util[cur_ang]=utility;
	    }
	}
      
      // Set the values for the angle (domain) and utility (range)
      int iii=0;
      domain_vals.push_back(iii); range_vals.push_back(OA_util[iii]);
      for (iii = 1; iii<360; iii++)
	{
	  //if (OA_util[iii] != OA_util[iii-1])
	  // {
	      domain_vals.push_back(iii);
	      range_vals.push_back(OA_util[iii]);
	      // }
	}

      head_zaic_v.setDomainVals(domain_vals);
      head_zaic_v.setRangeVals(range_vals);

      // Clear the domain and range values
      domain_vals.clear();
      range_vals.clear();

      // Extract the IvP function
      ivp_function = head_zaic_v.extractIvPFunction();
      
    }
  return(ivp_function);
}

// This function calcuates the cost of the individual points
double BHV_OA::Calc_Cost(int t_lvl, double dist)
{
  return pow((t_lvl/dist*m_v_size/m_speed*4.5),3)*m_maxutil;
}

// This function calcuates a 1D gaussian given a point, mean, and standard
//   deviation.
double BHV_OA::Calc_Gaussian(double x, double mu, double sigma, double amplitude)
{
  return pow(M_E, -(pow((x - mu),2)/(2*(sigma * sigma))))*amplitude;
}


/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/* This function parses the obstacles and then uses that information to    */
/*   create an IvPFunction using the ZAIC  tools. This function is         */
/*   functional, however it only takes in account only the obstacle with   */
/*   highest cost in the search area. Therefore it only really works when  */
/*   there is only one obstacle in the search zone. Also utility decreases */
/*   linearly with angle.
/***************************************************************************/
/*
IvPFunction *BHV_OA::buildFunctionWithZAIC()
{
  double max_cost, dist, util;
  string obs_type;
  int obs_t_lvl;
  vector<double> cost,obs_x, obs_y;
  vector<int> ang;
  stringstream ss, ss1, ss2, ss3, ss4, ss5;
  string str, poly, obs_pos;
  // This is a constant multiplier for the cost function that sets the
  // prohibition zone such that the utility function is zero when the
  // cost is greater than this constant. The radius of the prohibition
  // zone will be directly proportional to the threat level of the
  // object (t_lvl*multipler)  
  double multiplier; 
  // These are the values need to create a ZAIC
  int summit, peakwidth, basewidth, summitdelta, minutil, maxutil;

  basewidth = 20; // Arbitrarly picked 10 degrees on each side
  peakwidth = 180-basewidth;
  summitdelta = 1;
  maxutil = 100; 

  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  for (unsigned int i=0;i<info.size(); i++){
    // Parse the individual obstacles
    vector<string> ind_obs_info = parseString(info[i], ',');

    // Convert the strings to doubles and ints
    vector<string> x = parseString(ind_obs_info[0], '=');
    obs_x.push_back(strtod(x[1].c_str(), NULL)); // Get rid of the 'x='
    vector<string> y = parseString(ind_obs_info[1], '=');
    obs_y.push_back(strtod(y[1].c_str(), NULL)); // Get rid of the 'y='
    obs_t_lvl = (int)floor(strtod(ind_obs_info[2].c_str(), NULL));
    
    // Type of obstacle
    obs_type = ind_obs_info[3];

    // Calculate the angle and cost for the obstacle
    ang.push_back(relAng(m_ASV_x, m_ASV_y, obs_x[i], obs_y[i]));
    dist = sqrt(pow(m_ASV_x-obs_x[i],2) +pow(m_ASV_y-obs_y[i],2));

    // Make sure you dont divide by zero - if it is less than 1, set it 
    //  equal to 1
    if (dist <1)
      dist = 1;
    cost.push_back(obs_t_lvl/dist);
  }

  // Need to set this so that it is a function of the size and the current speed of the vessel
  double v_size = 4;
  multiplier = v_size/m_speed*4.5; 

  ZAIC_PEAK head_zaic(m_domain, "course");
  
  poly = ",format=radial,radius=30,pts=3,edge_color=hotpink,label=obs";
  max_cost = *max_element(cost.begin(), cost.end());
  
  // If the maximum cost is zero, then we dont want to create a ZAIC
  //   function describing the utility function
  if (max_cost != 0)
    {
      // ZAIC Components: summit, peakwidth, basewidth, summitdelta, 
      //   minutil, and maxutil  
      for (unsigned int ii=0;ii<cost.size(); ii++)
	{
	  // If the cost is the maximum cost, then calculate the utility of traveling in that direction
	  if (cost[ii]==max_cost)
	    {
	      // Set a maximum cost (aka if the ASV has gone into the prohibition zone). If this is the case increase the width of the utility penalty (basewidth).
	      if (cost[ii] > 1/multiplier) 
		{
		  basewidth = (int)floor(20*(multiplier*cost[ii])); 
		  peakwidth = 180-basewidth;
		  cost[ii] = 1/multiplier;
		}
	      else
		{
		  basewidth = 20; // Arbitrarly picked 10 degrees on each side
		  peakwidth = 180-basewidth;
		}
	      // Post the information on the Basewidth to the MOOSDB
	      ss1.str(string());
	      ss2.str(string());
	      ss1 << max_cost*multiplier;
	      ss2 << basewidth;
	      postMessage("Basewidth", "BW: "+ss2.str()+" Cost: "+ss1.str());

	      // Post the information on the obstacle to the MOOSDB
	      minutil = (int)floor((1-cost[ii]*multiplier)*maxutil);
	      ss1.str(string());
	      ss2.str(string());
	      ss1 << (ang[ii]);
	      ss2 << (minutil);
	      str =  "Angle: " + ss1.str() + " Utility: " + ss2.str();
	      postMessage("ENC_OA", str);

	      // Create an objective function using MOOS's ZAIC
	      summit = (ang[ii]+180)%360;
	      head_zaic.setParams(summit, peakwidth, basewidth, summitdelta, minutil, maxutil);

	      // Post a polygon to the MOOSDB showing which obstacle is being avoided
	      ss1.str(string());
	      ss2.str(string());
	      ss1 << obs_x[ii];
	      ss2 << obs_y[ii];
	      obs_pos = "x="+ss1.str()+",y="+ss2.str();
	      postMessage("VIEW_POLYGON", obs_pos+poly);
	    }
	}
    }

  
  head_zaic.setValueWrap(true); // Wrap around the heading axis
  head_zaic.setSummitInsist(true); // It combines the multiple utility functions by adding them together 
  
  // Set the IvP Function to take the maximum value if there is overlap
  bool take_the_max = true;

  IvPFunction *ipf = 0;
  if(head_zaic.stateOK()){
    ipf = head_zaic.extractIvPFunction(take_the_max);
  }
  else
    // Post warnings (if there are any) to the MOOSDB
    postWMessage(head_zaic.getWarnings());
  
  return(ipf);
}
*/

/***************************************************************************/
/* This function parses the obstacles and then uses that information to    */
/*   create an IvPFunction using the reflector tools. This function is not */
/*   functional and causes the helm to crash when any obstacles are found. */
/***************************************************************************/
/*
IvPFunction *BHV_OA::buildIvPFunction()
{
  IvPFunction *ipf = 0;
  double max_cost, dist, util;
  string obs_type;
  int obs_t_lvl;
  vector<double> cost,obs_x, obs_y;
  vector<int> ang;
  stringstream ss, ss1, ss2, ss3, ss4, ss5;
  string str, poly, obs_pos;
  // This is a constant multiplier for the cost function that sets the prohibition zone such that the utility function is zero when the cost is greater than this constant. The radius of the prohibition zone will be directly proportional to the threat level of the object (t_lvl*multipler) 
  double multiplier; 
  int  maxutil= 100; // Need to look at the Waypoint behavior to see what their maximum utility is

  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  for (unsigned int i=0;i<info.size(); i++){
    // Parse the individual obstacles
    vector<string> ind_obs_info = parseString(info[i], ',');

    // Convert the strings to doubles and ints
    vector<string> x = parseString(ind_obs_info[0], '=');
    obs_x.push_back(strtod(x[1].c_str(), NULL)); // Get rid of the 'x='
    vector<string> y = parseString(ind_obs_info[1], '=');
    obs_y.push_back(strtod(y[1].c_str(), NULL)); // Get rid of the 'y='
    obs_t_lvl = (int)floor(strtod(ind_obs_info[2].c_str(), NULL));
    
    // Type of obstacle
    obs_type = ind_obs_info[3];

    // Calculate the angle and cost for the obstacle
    ang.push_back(relAng(m_ASV_x, m_ASV_y, obs_x[i], obs_y[i]));
    dist = sqrt(pow(m_ASV_x-obs_x[i],2) +pow(m_ASV_y-obs_y[i],2));

    // Make sure you dont divide by zero - if it is less than 1, set it 
    //  equal to 1
    if (dist <1)
      dist = 1;
    cost.push_back(obs_t_lvl/dist);
  }
  multiplier = 4.5; // need to set this so that it is a function of the size and the current speed of the vessel
  AOF_Gauss aof_g(m_domain);
  /*
    AOF_Gauss aof(m_domain);
  
  
    max_cost = *max_element(cost.begin(), cost.end());
    string center, sigma, amp;

    // If the maximum cost is zero, then we dont want to create a ZAIC
    //   function describing the utility function
    if (max_cost != 0)
    {
    center.clear();
    sigma.clear();
    amp.clear();
    double sig;
    for (int ii =0; ii<m_num_obs; ii++)
    {
    stringstream ss_cent, ss_sigma, ss_amp;
    if (cost[ii] > 1/multiplier)
    {
    sig = multiplier*cost[ii];
    cost[ii] = 1/multiplier;
    }
    else
    sig = 1;
    ss_cent << ang[ii];
    ss_amp << cost[ii];
    ss_sigma << sig;
    sigma += ss_cent.str();
    center += ss_cent.str();
    amp += ss_amp.str();
    if (ii != m_num_obs-1)
    {
    sigma += ",";
    center += ",";
    amp += ",";
    }
    }
    
    bool ok = true;
    ss << m_num_obs;
      
      
    // Step 1 - Create the AOF instance and set parameters
    ok = ok && aof_g.setParam("center", center);
    //ok = ok && aof_g.setParam("sigma", sigma);
    //ok = ok && aof_g.setParam("amp", amp);
      
    if (ok)
    postWMessage(sigma);
      
    // Step 2 - Create the Reflector instance given the AOF
    if(ok)
    {
    postWMessage("Ok");
      
    OF_Reflector reflector(&aof);
  
    // Step 3 - Build and Extract the IvP Function
    int  amt_created = reflector.create(500,0);
    ipf = reflector.extractIvPFunction();
    if (ipf == 0)
    postWMessage("ERROR --> Size of xcent != Size of sigma != Size of range");
      
    }
    else
    postWMessage("ERROR --> Check out the Parameters");
  //
  // }
  
  return(ipf);

}
*/
