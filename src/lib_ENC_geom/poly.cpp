/*
 * poly.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include "poly.h"

// Initialize Member Variables
Poly::Poly()
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = 0;
  y = 0;
  m = 0;
  b = 0;
  ref_frame = 0;
  obs_type = "";
  t_lvl = 0;
}

// Initialize Member Variables
Poly::Poly(double X, double Y)
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = X;
  y = Y;
  m = 0;
  b = 0;
  ref_frame = 0;
  obs_type = "";
  t_lvl = 0;
}

void Poly::calcCost(double v_length, double speed, double maxutil)
{
  cost = pow((t_lvl/dist*v_length/speed*4.5),3)*maxutil;
}

// Calculates the slope and y-intercept of the "V" vertex
//	***Use Right for max angle and Left for min angle***
void Poly::setMB_Right_pnt(double center_pnt_cost, double center_pnt_angle)
{
  if (center_pnt_angle != ang)
    {
      m = (center_pnt_cost - cost)/(center_pnt_angle - ang);
      b = (center_pnt_cost) - (m*center_pnt_angle);
    }
  else
    {
      // If the slope is infinite, make m and b 999
      m = 999;
      b = 999;
    }
}

void Poly::setMB_Left_pnt(double center_pnt_cost, double center_pnt_angle)
{
  if (center_pnt_angle != ang)
    {
      m = (cost - center_pnt_cost)/(ang- center_pnt_angle);
      b = (cost) - (m*ang);
    }
  else
    {
      // If the slope is infinite, make m and b 999
      m = 999;
      b = 999;
    }
}





