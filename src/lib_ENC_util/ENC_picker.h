#include <map>
#include <vector>
#include <iostream>
#include <string>
#include <geodesy.h> // Lat/Long to UTM
#include "strtk.hpp" // for csv parsing
#include "ogrsf_frmts.h" // for gdal
#include "dir_walk.h" // for walking through directories 

using namespace std;

#ifndef ENC_PICKER_H_
#define ENC_PICKER_H_

class ENC_Picker
{
public:
	// Constructor and deconstructor
	ENC_Picker();
	ENC_Picker(double latOrigin,double lonOrigin);
	ENC_Picker(double latOrigin, double lonOrigin, string root_dir, string csvfilename);
	ENC_Picker(double latOrigin, double lonOrigin, string root_dir);
	~ENC_Picker() {};

	// Sets the lat/longoOrigin
	void setOrigin(double latOrigin,double lonOrigin) {LatOrigin=latOrigin; LonOrigin=lonOrigin; point= new OGRPoint(lonOrigin,latOrigin); };

	// Walk the directory and store the ENC datasource. 
	void build_ENC_outlines();

	// Create the map container of RNC chart names and data
	void parse_csv();
	
	// This function choses the ENC that should be run and returns the 
        //    chart scale and the path to the chart.
	void pick_ENC(string &chart_name, int &chart_scale);
        

private:
	double LatOrigin, LonOrigin;
	OGRPoint* point;
	vector<OGRPolygon*> ENC_Outline;
	vector<string> chart, ENC_Names;
	map<string, int> RNC;
	string root_directory, csvfile;
};
void split(vector<string>& parsed, const string& s, char c);

#endif /* ENC_PICKER_H_ */


