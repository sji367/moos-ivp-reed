/*
 * point.cpp
 * Created by sji36
 * Jan 20, 2017
 *
 */

#include "point.h"

// Initialize Member Variables

// Constructor
Point::Point()
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = 0;
  y = 0;
  obs_type = "";
  t_lvl = 0;
}

Point::Point(double X, double Y)
{
  ang = 0;
  cost = 0;
  dist = 1;
  x = X;
  y = Y;
  obs_type = "";
  t_lvl = 0;
}

void Point::setStatics(int T_Lvl, string Obs_Type)
{
  obs_type=Obs_Type;
  t_lvl = T_Lvl;
}

// This function calculates the distance to the vertex from the ASV's current
//  location. If the distance is less than one, this function returns a
//  distance of 1.
void Point::calcDist(double ASV_x, double ASV_y)
{
  dist = sqrt(pow(ASV_x-x,2) +pow(ASV_y-y,2));
  cout << "!DIST: " << dist << " X: " << x << " Y:" << y << endl;
  if (dist<1)
    dist=1;
}

void Point::calcCost(double v_length, double speed, double maxutil)
{
    // make sure you cannot divide by zero
    if (speed==0)
        cost=0;
    else
        cost = 2*pow((t_lvl*v_length/(dist)),2)*maxutil;
        //cost = exp(dist/t_lvl)*(1/v_length);

    /*
    if (cost <2.5)
        cost = 0;
    else if (cost > 100)
        cost = 100;
        */
}

