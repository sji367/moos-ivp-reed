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
#include <tuple> // To use make_tuple and tie

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
	  m_WPT_x = stoi(temp_WPT[0]);
	  m_WPT_y = stoi(temp_WPT[1]);
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   # of Obstacles:t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist!...
      result = parseString(m_obstacles, ':');
      
      // Parse the number of obstacles
      m_num_obs = stoi(result[0]);
      
      // Store the information on the obstacle if there are the right amount of parts in the Poly_obstacle string
      if (result.size()==2)
	{
	  m_obs_info = result[1];
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
  fill_n(OA_util,360, m_maxutil);

  // Holds the maximum cost for each obstacle which will be used in adjusting the lead parameter
  vector <double> max_cost;

  // Vectors holding the parsed information on the obstacles
  vector<string> poly_info, poly_gen_info, poly_ang_info;

  // Array holding the distance information (during the conversion to a double)
  double d[3] = {0};

  double utility, actual_cost, calculated_cost;
  int cur_ang;
  string x,y;

  string Ang = "";
  string Ang1 = "";
  string COST = "";
  
  double pnt_x, pnt_y;

  bool Debug = true;
  int safety = 85;
  int buff_low = 0;
  int buff_high = 0;
  string cur_ang_str= "";
  int cntr = 0;
  // The buffer distance is to make sure that the ASV avoids the obstacle with some buffer 
  int buffer_width, temp_buff;
  
  for (unsigned int i=0;i<m_num_obs; i++)
    {
      // Initialize vectors by clearing them
      poly_info.clear();
      poly_gen_info.clear();
      poly_ang_info.clear();
      
      // Initialize the stucture holding the information on the obstacle
      poly_obs obstacle;

      // Parse the individual obstacles
      poly_info = parseString(info[i], '@');

      // General information on obstacle
      //    Type of obstacle and threat level
      poly_gen_info = parseString(poly_info[0], ',');
	  
      // Type of obstacle and threat level
      obstacle.t_lvl = stoi(poly_gen_info[0]);
      obstacle.obs_type = poly_gen_info[1];

      // Information on the angles
      //    minimum angle, angle of minimum distance, maximum angle
      poly_ang_info = parseString(poly_info[1], ',');
      
      // Reference_frame
      obstacle.ref_frame = stoi(poly_ang_info[0]);
      postMessage("Ref_frame", obstacle.ref_frame);

      // Min Angle
      obstacle.min_ang.x = (strtod(poly_ang_info[1].c_str(), NULL));
      obstacle.min_ang.y = (strtod(poly_ang_info[2].c_str(), NULL));
      obstacle.min_ang.ang = relAng(m_ASV_x, m_ASV_y, obstacle.min_ang.x,obstacle.min_ang.y);
      obstacle.min_ang.ang = convert_ref_frame(obstacle.min_ang.ang, obstacle.ref_frame);
      obstacle.min_ang.dist = calc_dist2ASV(obstacle.min_ang.x, obstacle.min_ang.y);

      // Min Distance
      obstacle.min_dist.x = (strtod(poly_ang_info[3].c_str(), NULL));
      obstacle.min_dist.y = (strtod(poly_ang_info[4].c_str(), NULL));
      obstacle.min_dist.ang = relAng(m_ASV_x, m_ASV_y, obstacle.min_dist.x, obstacle.min_dist.y);
      obstacle.min_dist.ang = convert_ref_frame(obstacle.min_dist.ang, obstacle.ref_frame);
      obstacle.min_dist.dist = calc_dist2ASV(obstacle.min_dist.x, obstacle.min_dist.y);

      // Max Angle
      obstacle.max_ang.x = (strtod(poly_ang_info[5].c_str(), NULL));
      obstacle.max_ang.y = (strtod(poly_ang_info[6].c_str(), NULL));
      obstacle.max_ang.ang = relAng(m_ASV_x, m_ASV_y, obstacle.max_ang.x, obstacle.max_ang.y);
      obstacle.max_ang.ang = convert_ref_frame(obstacle.max_ang.ang, obstacle.ref_frame);
      obstacle.max_ang.dist = calc_dist2ASV(obstacle.max_ang.x, obstacle.max_ang.y);

      // Determine the cost for each angle that you have information on
      obstacle.min_ang.cost = Calc_Cost(obstacle.t_lvl,obstacle.min_ang.dist);
      obstacle.max_ang.cost = Calc_Cost(obstacle.t_lvl,obstacle.max_ang.dist);
      obstacle.min_dist.cost = Calc_Cost(obstacle.t_lvl,obstacle.min_dist.dist);

      // Calculate the slope and y intercepts
      // Min angle
      tie(obstacle.min_ang.m, obstacle.min_ang.b) = calc_m_b(obstacle.min_ang, obstacle.min_dist, obstacle.ref_frame);
      // Max Angle
      tie(obstacle.max_ang.m, obstacle.max_ang.b) = calc_m_b(obstacle.min_dist, obstacle.max_ang, obstacle.ref_frame);

      // Store the maximum cost for the obstacle so that it can be used later
      //  to adjust the lead parameter 
      max_cost.push_back(obstacle.min_dist.cost);

      // The buffer distance is to make sure that the ASV avoids the obstacle 
      //  with some buffer
      buffer_width = 20;
      // If the maximum cost is greater than 100, increase the size of the 
      //  saftey buffer
      if (obstacle.min_dist.cost > 70)
	{
	  temp_buff = floor(pow(2*obstacle.min_dist.cost/m_maxutil,3));
	  if (temp_buff >100)
	    temp_buff =100;
	  buffer_width += temp_buff;
	  postMessage("buffer_w", buffer_width);
	  /*
	  // make the slope more gradual
	  obstacle.max_ang.ang += buffer_width/10;
	  obstacle.min_ang.ang += -buffer_width/10;
	  // Recalculate the slope and y intercepts
	  // Min angle
	  tie(obstacle.min_ang.m, obstacle.min_ang.b) = calc_m_b(obstacle.min_ang, obstacle.min_dist);
	  // Max Angle
	  tie(obstacle.max_ang.m, obstacle.max_ang.b) = calc_m_b(obstacle.min_dist, obstacle.max_ang);
	  //buffer_width = 120;
	  */
	}

      // Debugging
      if (Debug)
	{
	  //Ang = doubleToString(obstacle.min_ang.ang-buffer_width)+","+ doubleToString(obstacle.min_dist.ang)+"," +doubleToString(obstacle.max_ang.ang+buffer_width);
	  Ang1 = doubleToString(convert_ref_frame(obstacle.min_ang.ang, obstacle.ref_frame))+","+ doubleToString(convert_ref_frame(obstacle.min_dist.ang, obstacle.ref_frame))+"," +doubleToString(convert_ref_frame(obstacle.max_ang.ang, obstacle.ref_frame));  
	  postMessage("Angles", Ang1);
	  
	  COST = doubleToString(obstacle.min_ang.cost)+","+ doubleToString(obstacle.min_dist.cost)+"," +doubleToString(obstacle.max_ang.cost);
	  postMessage("cost", COST);

	  Ang = doubleToString(obstacle.min_ang.ang)+","+ doubleToString(obstacle.min_dist.ang)+"," +doubleToString(obstacle.max_ang.ang);
	  postMessage("Angles_no_buff", Ang);
	  
	  cout << "!Ang_Cost," << i <<"," << to_string(obstacle.ref_frame) << "," << Ang << "," << Ang1 << endl;
	}

      // If you are close to the obstacle place a safety buffer of atleast +/- 75 degrees around the closest point
      if (obstacle.min_dist.dist < 10)
	{
	  for (int i = 1; i<safety; i++)
	    {
	      // Update the current buffer angles (Domain = [-360,360])
	      buff_low = (int)floor(fmod(obstacle.min_dist.ang-i,360));
	      buff_high = (int)floor(fmod(obstacle.min_dist.ang+i,360));
	      
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
      for (double x1 = obstacle.min_ang.ang-buffer_width; x1<=obstacle.min_dist.ang; x1++)
	{
	  // Update the current angle (Domain = [-360,360])
	  cur_ang = (int)floor(fmod(x1,360));

	  // Make sure it is not negative (Domain = [0,360])
	  if (cur_ang<0)
	    cur_ang+=360;

	  // Deal with slope being inf 
	  //  (if max angle = min dist angle) --> set the cost to the cost
	  //       of the mininum distance point. 
	  if (obstacle.min_ang.m == 999)
	    calculated_cost = obstacle.min_dist.cost;
	  else
	    calculated_cost = (obstacle.min_ang.m*x1+obstacle.min_ang.b);

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
	    {
	      OA_util[cur_ang]=floor(utility);
	    }
	}
      // This calculates the utility and stores that value if it is less than
      //  the current utility for all obstacles --> max cost to max angle
      // Makes the first half of the "V Shaped" penalty function
      for (double x2 = obstacle.min_dist.ang; x2<=obstacle.max_ang.ang+buffer_width; x2++)
	{
	  // Update the current angle  (Domain = [-360,360])
	  cur_ang = (int)floor(fmod(x2,360));
	  
	  // Make sure it is not negative (Domain = [0,360])
	  if (cur_ang < 0)
	    cur_ang += 360;

	  // Deal with slope being inf 
	  //  (if max angle = min dist angle) --> set the cost to the cost
	  //       of the mininum distance point. 
	  if (obstacle.max_ang.m == 999)
	    calculated_cost = obstacle.min_dist.cost;
	  else
	    calculated_cost = (obstacle.max_ang.m*x2+obstacle.max_ang.b);

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
	    {
	      OA_util[cur_ang]=floor(utility);
	    }
	}
    }
  // Store the first value
  int iii = 0;
  //domain_vals.push_back(iii); range_vals.push_back((int)floor(OA_util[iii]));
  string ut = "";
  // Set the values for the angle (domain) and utility (range)
  for (iii = 0; iii<360; iii++)
    {
      ///if (OA_util[iii] != (OA_util[iii-1]))
      //{
      domain_vals.push_back(iii);
      range_vals.push_back((int)floor(OA_util[iii]));
      ut += to_string((int)floor(OA_util[iii]))+",";
	  //}
    }
  //cout << "!IvP," << ut << endl;
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
  
  // Find the maximum value in the max_cost vector, which holds the cost of
  //  the minimum distance point for each obstacle
  double maximum_cost_value = *max_element(max_cost.begin(), max_cost.end());
  postMessage("Max_Cost", maximum_cost_value);

  // The lead parameter sets the distance from the perpendicular intersection
  //  of the ASV's current location and the trackline that the waypoint
  //  behavior steers toward.
  Update_Lead_Param(maximum_cost_value);
  
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
// This function calcuates the distance to the vertex from the ASV's current
//  location. If the distance is less than one, this function returns a 
//  distance of 1.
double BHV_OA_poly::calc_dist2ASV(double x, double y)
{
  double dist = sqrt(pow(m_ASV_x-x,2) +pow(m_ASV_y-y,2));
  if (dist<1)
    dist=1;
  return dist;
}
tuple<double, double> BHV_OA_poly::calc_m_b(poly_attributes left_pt, poly_attributes right_pt, int ref_frame)
{
  double m,b;
  if (left_pt.ang != right_pt.ang)
    {        
      m = (left_pt.cost - right_pt.cost)/(left_pt.ang - right_pt.ang);
      b = (left_pt.cost) - (m*left_pt.ang);
    }
  else
    {
      // If the slope is inf, make m and b 999
      m = 999;
      b = 999;
    }
  return make_tuple(m,b);
}

double BHV_OA_poly::convert_ref_frame(double ang, int ref_frame)
{
  double cur_ang = 0;
  // Update the current angle (Domain = [-360,360])
  cur_ang = fmod(ang,360);

  // Make sure it is not negative (Domain = [0,360]) aka reference frame 1
  if (cur_ang<0)
    cur_ang+=360;
  
  switch(ref_frame)
    {
    case 1: // Domain [0, 360]
      // it is already in case 1
      break;
      
    case 2: // Domain [-90, 270]
      if (cur_ang > 270)
	cur_ang += -360;
      break;

    case 3: // Domain [-180, 180]
      if (cur_ang > 180)
	cur_ang += -360;
      break;
      
    case 4: // Domain [-270, 90]
      if (cur_ang > 90)
	cur_ang += -360;
      break;
      
    default:
      postWMessage("Invalid Reference Frame: "+to_string(ref_frame));
      break;
    }
  return cur_ang;
}

double BHV_OA_poly::calc_RelAngle(double x, double y)
{
  double angle = atan2((y-m_ASV_y),(x-m_ASV_x));
  return -(angle-90);
}
