/*
 * test.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: sji36
 */

#include "ogrsf_frmts.h" // for gdal/ogr
#include <iostream>
#include <vector>
#include <algorithm> // for min_element and max_element

using namespace std;

int main()
{
  GDALAllRegister();

  OGRGeometry *geom, *geom2;
  OGRLineString *line, *line2;
  OGRLayer *Line_Layer, *LL1, *LL2;
  OGRFeature *feat, *feat2;
  OGRFeatureDefn *feat_def;

  GDALDataset *ds;
  vector <double> distance2line;
  vector <double> minDist;

  ds = (GDALDataset*) GDALOpenEx( "Line.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
  //ds_line = (GDALDataset*) GDALOpenEx( , GDAL_OF_VECTOR, NULL, NULL, NULL );

  if (ds == NULL)
    cout << "Opening ds failed." << endl;
  else
    {
      Line_Layer = ds -> GetLayerByName("Line");
      Line_Layer->SetAttributeFilter("Type=DEPCNT");
      LL1 = Line_Layer; LL2 = Line_Layer;
      feat_def = Line_Layer->GetLayerDefn();
      
      if (Line_Layer == NULL)
	cout << "Opening line layer failed." << endl;
      else
	cout << "Opened correctly, # feat: "<< Line_Layer->GetFeatureCount() << endl;
      int i = 0;
      LL1->ResetReading();
      feat = LL1->GetNextFeature();
      while(feat)
	{
	  i++;
	  
          geom = feat->GetGeometryRef();
          line = (OGRLineString *)geom;

	  // Preset/ reset some variables
	  distance2line.clear();
	  LL2->ResetReading();
	  feat2 = LL2->GetNextFeature();
    	  while(feat2)
	    {
	      // If they are not the same feature, compare the distances
	      if (feat->GetFID() != feat2->GetFID())
		{
		  geom2 = feat2->GetGeometryRef();
		  line2 = (OGRLineString *)geom2;
		  distance2line.push_back(line->Distance(line2));
		  //cout << line->Distance(line2);
		}
	      feat2 = LL2->GetNextFeature();
	    }
	  minDist.push_back(*max_element(distance2line.begin(),distance2line.end()));
	  cout << i <<": Distance: " << minDist[i] << endl;
	}
    }
  return 0;
}
