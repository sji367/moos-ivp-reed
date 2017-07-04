/*
 * poly.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: sji367
 */

#include "poly.h"

// Initialize Member Variables

// Constructor
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

// Makes sure that the maximum angle is actually bigger than the minimum angle
void Poly::setAngle(double Angle, double boundary, bool inside)
{
  if ((Angle<=boundary)||inside)
    ang = Angle;
  else
    ang = Angle-360;
  /*
  if ((ref_frame == 0)||(boundary==0))
    ang = Angle;
  else
    {
      ang = fmod(360+(Angle-boundary),360);
      // Make sure it is in the domain [0,360]
      if (ang<0)
	ang += 360;
    }
  */
}


void Poly::setStatics(int Ref_Frame, int T_Lvl, string Obs_Type)
{
  ref_frame = Ref_Frame;
  obs_type=Obs_Type;
  t_lvl = T_Lvl;
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





