/*
 * OA.cpp
 * Created by sji36
 * Jan 20, 2017
 *
 */

#include "OA.h"

// Initialize Member Variables
OA::OA(double X, double Y)
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

OA::OA()
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

// This function calculates the distance to the vertex from the ASV's current
//  location. If the distance is less than one, this function returns a
//  distance of 1.
void OA::calcDist(double ASV_x, double ASV_y)
{
  dist = sqrt(pow(ASV_x-x,2) +pow(ASV_y-y,2));
  if (dist<1)
    dist=1;
}

void OA::setStatics(int Ref_Frame, int T_Lvl, string Obs_Type)
{
  ref_frame = Ref_Frame;
  obs_type=Obs_Type;
  t_lvl = T_Lvl;
}

// Converts the angle to the right domain and then returns the corrected value
//	case 1: // Domain [0, 360]
//	case 2: // Domain [-90, 270]
//	case 3: // Domain [-180, 180]
//	case 4: // Domain [-270, 90]
double OA::convert_ref_frame(double Angle)
{
  double cur_ang = 0;
  // Update the current angle (Domain = [-360,360])
  cur_ang = fmod(Angle,360);

  // Make sure it is not negative (Domain = [0,360]) aka reference frame 1
  if (cur_ang<0)
    cur_ang+=360;
  switch(ref_frame)
    {
    case 1: // Domain [0, 360]
      // it is already in case 1
      break;

    case 2: // Domain [-180, 90]
      if (cur_ang > 180)
	cur_ang += -360;
      break;

    case 3: // Domain [-270, 90]
      if (cur_ang > 90)
	cur_ang += -360;
      break;

    case 4: // Domain [-90, 270]
      if (cur_ang > 270)
	cur_ang += -360;
      break;

    default:
      //postWMessage("Invalid Reference Frame");
      break;
    }
  return cur_ang;
}
