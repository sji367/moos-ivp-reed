/*
 * BHV_OA_pnt.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

// MOOS Libraries
#include "BHV_OA_pnt.h"
#include "XYPolygon.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_Vector.h"

// Normal C++ Libraries
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort
#include <string>
#include <vector>

//---------------------------------------------------------------
// Constructor

BHV_OA_pnt::BHV_OA_pnt(IvPDomain gdomain) :
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

bool BHV_OA_pnt::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if((param == "vehicle_size"|| param == "vehicle_length") && isNumber(val))
    {
      m_v_length = double_val;
      return(true);
    }
  
  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_OA_pnt::onRunState()
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
  cout << "!abc check "<< endl;
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
      
      // Parse the number of obstacles
      m_num_obs = (int)floor(strtod(result[0].c_str(), NULL));
      
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
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

/***************************************************************************/
/* This function parses the obstacles and then uses that information to    */
/*   create an IvPFunction using the ZAIC Vector tools. This function is   */
/*   the most functional out of the three because it uses a Gaussian       */
/*   distribution to describe how the utility falls off as a function of   */
/*   angle and it takes into account all obstacles in the search area.     */
/***************************************************************************/
IvPFunction *BHV_OA_pnt::buildZAIC_Vector()
{
  vector<Point> obstacle;
  vector<double> max_cost;
  vector<string> info;

  // To start with, fill an array with maxiumum utility
  double OA_util[360];
  fill(OA_util,OA_util+360, m_maxutil);

  // Clear all vectors
  obstacle.clear();
  info.clear();
  max_cost.clear();
  
  // Then separate the obstacles from one another
  info = parseString(m_obs_info, '!');
  
  for (int i=0; i<info.size(); i++)
    {
      Point object = Point();
      cout << "!abc construct" << endl;
      obstacle.push_back(object);
      
      getPoint(info[i], obstacle[i], max_cost);
      cout << "!abc getPoint" << endl;
      if (obstacle[i].getCost()>0){
	calcGaussWindow(OA_util, obstacle[i]);
	cout << "!abc calc window" << endl;
      }
    }

  Update_Lead_Param(max_cost);
  cout << "!abc return" << endl;
  return setIVP_domain_range(OA_util);
}

void BHV_OA_pnt::getPoint(string info, Point& Obstacle, vector<double>& max_cost)
{
  // Vectors holding the parsed information on the obstacles
  vector<string> x, y, ind_obs_info;;

  // Parse the individual obstacles
  ind_obs_info = parseString(info, ',');

  // Get rid of the 'x=' and 'y='
  x = parseString(ind_obs_info[0], '=');
  y = parseString(ind_obs_info[1], '=');
  Obstacle.setXY(strtod(x[1].c_str(), NULL),strtod(y[1].c_str(), NULL));
  
  // Set the reference frame (1 for all points), threat level and the obstacle type
  Obstacle.setStatics(1, (int)floor(strtod(ind_obs_info[2].c_str(), NULL)), ind_obs_info[3]);
  
  // Calculate and set the angle for the obstacle
  Obstacle.setAngle(relAng(m_ASV_x, m_ASV_y, Obstacle.getX(), Obstacle.getY()));
  
  // Calculate and set the cost and distance of the point obstacle
  Obstacle.calcLocation(m_ASV_x, m_ASV_y, m_v_length, m_speed, m_maxutil);
  
  //Store the maximum cost
  max_cost.push_back(Obstacle.getCost());
}

void BHV_OA_pnt::calcGaussWindow(double (&OA_util)[360], Point & Obstacle)
{
  double amplitude, width, sigma;
  int cur_ang= 0;
  double utility = 0;

  if (Obstacle.getCost() > m_maxutil)
    {
      amplitude = m_maxutil;
      width = (int)floor(20*Obstacle.getCost()/m_maxutil);
      sigma = 8+width/8;
    }
  else
    {
      amplitude = Obstacle.getCost();
      width = 20;
      sigma = 8;
    }

  // This calculates the utility of the Gaussian window
  // function and stores that value if it is less than the
  // current utility for all obstacles
  for (int k = 0; k< (2*width+1); k++)
    {
      // Calculate the angle that will be used in the Gaussian window
      cur_ang = (int)floor((int)(Obstacle.getAngle()-(width)+k)%360);
      if (cur_ang < 0)
	cur_ang += 360;

      // Calculate the Gaussian Depression Penalty Fuction
      utility = m_maxutil - calc_Gaussian(cur_ang, Obstacle.getAngle(), sigma, amplitude);

      // If the current utility value is less than the one for
      // the gaussian window then store the one for the
      // Gaussian window
      if (utility<OA_util[cur_ang])
	OA_util[cur_ang]=utility;
    }
}

IvPFunction* BHV_OA_pnt::setIVP_domain_range(double OA_util[360])
{
  IvPFunction *ivp_function;

  // Declare which variable the domain is
  ZAIC_Vector head_zaic_v(m_domain, "course");

  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;

  // Set the values for the angle (domain) and utility (range)
  for (int iii = 0; iii<360; iii++)
    {
      domain_vals.push_back(iii);
      range_vals.push_back((int)floor(OA_util[iii]));
    }
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
void BHV_OA_pnt::Update_Lead_Param(vector<double> vect_max_cost)
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

double BHV_OA_pnt::calc_Gaussian(double x, double mu, double sigma, double amplitude)
{
  return pow(M_E, -(pow((x - mu),2)/(2*(sigma * sigma))))*amplitude;
}



