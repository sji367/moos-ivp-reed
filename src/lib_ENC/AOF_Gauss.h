/*****************************************************************/
/*    NAME: Sam Reed                                             */
/*    ORGN: UNH CCOM/JHC                                         */
/*    FILE: AOF_Gauss.cpp                                        */
/*    DATE: June 2016                                            */
/*                                                               */
/*****************************************************************/

#ifndef AOF_GAUSS_HEADER
#define AOF_GAUSS_HEADER

#include <vector>
#include <string>
#include "AOF.h"
#include "IvPDomain.h"

using namespace std;

class AOF_Gauss: public AOF {
 public:
  AOF_Gauss(IvPDomain domain) : AOF(domain)
		{m_xcent="0"; m_sigma="1"; m_range="100";}
  ~AOF_Gauss() {}
  
 public:
  double evalPoint(std::vector<double>);
  //bool   setParam(std::string, std::string);
	bool   setParam(std::string, std::string);

private:
  std::string m_xcent, m_sigma; 
  std::string m_range; // Amplitude of Gaussian
};

#endif













