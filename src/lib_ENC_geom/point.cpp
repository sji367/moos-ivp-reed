/*
 * point.cpp
 * Created by sji36
 * Jan 20, 2017
 *
 */

#include "point.h"

// Initialize Member Variables
Point::Point()
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = 0;
  y = 0;
  ref_frame = 0;
  obs_type = "";
  t_lvl = 0;
}

// Initialize Member Variables
Point::Point(double X, double Y)
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = X;
  y = Y;
  ref_frame = 0;
  obs_type = "";
  t_lvl = 0;
}

void Point::calcCost(double v_length, double speed, double maxutil)
{
  cost = pow((t_lvl/dist*v_length/speed*4.5),3)*maxutil;
}


