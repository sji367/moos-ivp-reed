/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: L84.cpp                                         */
/*    DATE: April 2017                                      */
/************************************************************/

#include "L84.h"

void L84::writeL84()
{
  // Initialize Geodesy to convert the WPTs to lat/long
  m_Geodesy.Initialise(dfLatOrigin, dfLongOrigin);\
  
  double lat, lon;// place holder for lat/long conversion
  unsigned int  line_number =1;
  ofstream L84file;
  L84file.open (L84FileName); // Open the new file
  vector<string> vect_WPTs;
  char delimiter = ',';
  vect_WPTs = split(WPTs, delimiter); // split the string by ','s

  // Start writing the L84 files
  int num_lines = (vect_WPTs.size())/4;
  L84file << "LNS "<< num_lines <<"\n";
  for (int i=0; i<vect_WPTs.size()-2; i+=2)
    {
      // convert the first point from local UTM to Lat/Long
      m_Geodesy.LocalUTM2LatLong(atof(vect_WPTs[i].c_str()), atof(vect_WPTs[i+1].c_str()), lat, lon);
      L84file << "LIN 2\n";
      L84file << "PTS " << lat<< setprecision(10)<< " " << lon<< setprecision(10)<<"\n";

      // convert the next point from local UTM to Lat/Long
      m_Geodesy.LocalUTM2LatLong(atof(vect_WPTs[i+2].c_str()), atof(vect_WPTs[i+3].c_str()), lat, lon);
      L84file << "PTS " << lat<< setprecision(10)<< " " << lon<< setprecision(10)<<"\n";
      L84file << "LNN "<< line_number << "\n";
      L84file << "EOL\n";
      line_number++;
    }
  L84file << "\n";
  
  L84file.close();
}


vector<string> split(const string& s, char c)
{
  vector<string> v;
  unsigned int i = 0;
  unsigned int j = s.find(c);
  while (j < s.length()) {
    v.push_back(s.substr(i, j - i));
    i = ++j;
    j = s.find(c, j);
    if (j >= s.length()) {
      v.push_back(s.substr(i, s.length()));
      break;
    }
  }
  return v;
}
