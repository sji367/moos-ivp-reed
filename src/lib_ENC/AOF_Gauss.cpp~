/*****************************************************************/
/*    NAME: Sam Reed                                             */
/*    ORGN: UNH CCOM/JHC                                         */
/*    FILE: AOF_Gauss.cpp                                        */
/*    DATE: June 6th 2016                                        */
/*                                                               */
/*****************************************************************/

#ifdef _WIN32
#   define _USE_MATH_DEFINES
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif

#include "AOF_Gauss.h"
#include <cmath>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
using namespace std;
/*
//----------------------------------------------------------
// Procedure: Constructor

AOF_Gauss::AOF_Gauss(IvPDomain g_domain) : AOF(g_domain)
{
  m_xcent="0";
  m_sigma="1";
  m_range="100";
}
*/
//----------------------------------------------------------------
// Procedure: setParam
 
bool AOF_Gauss::setParam(string param, string value)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  if(param == "xcent" or "x" or "center")
    m_xcent = value;
  else if(param == "sigma")
    m_sigma = value;
  else if(param == "range" or "amplitude" or "amp")
    m_range = value;
  else
    return(false);
  return(true);
}

//----------------------------------------------------------------
// Procedure: evalPoint

double AOF_Gauss::evalPoint(vector<double> point) 
{
  
  double cent, sigma, range;

  // Parse the m_xcent string
  vector<string> temp_cent = parseString(m_xcent, ',');

  // Parse the m_sigma string
  vector<string> temp_sigma = parseString(m_sigma, ',');

  // Parse the m_range string
  vector<string> temp_range = parseString(m_range, ',');

  // This is the minimum utility after cycling though each temp value
  double util =0;

  // If the parsed vectors are the same length then continue, otherwise break
  if (temp_sigma.size() == temp_sigma.size() and temp_sigma.size() == temp_range.size())
    {
      double xval = extract("course", point);

      // This value is the temperary value for the gaussian function
      double temp;
      // This should be the maximum value after cycling though each temp value
      double max;

      // Cycles through the parsed sigmas and calculates the combined gaussian function
      for (int i= 0;i<temp_sigma.size(); i++)
	{
	  // Convert the parsed string to a double
	  cent = atof(temp_cent[i].c_str());
	  sigma = atof(temp_sigma[i].c_str());
	  range = atof(temp_range[i].c_str());

	  // Calculation of the Gaussian where M_E is e (2.71727...)
	  temp = pow(M_E, -(pow((xval - cent),2)/(2*(sigma * sigma))));
	  // Check to see if this is the maximum value, if it is translate it into a utility
	  if (temp > max or i == 0)
	    {
	      max = temp;
	      util = (1-max) * range;
	    }
	}
      // Return the gaussian where the maximum cost has been kept
      return(util);
    }
  // If the sizes of xcent and sigma are not the same post an error and break
  else
    {
      //postWMessage("ERROR --> Size of xcent != Size of sigma != Size of range");
      return 0;
    }
  //*/
}




