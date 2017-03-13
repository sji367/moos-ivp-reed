
#include <string>
#include <iostream>
#include "ogrsf_frmts.h" // for gdal/ogr
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

using namespace std;

void LayerMultiPoint(OGRLayer *layer_mp, OGRLayer *PointLayer, string LayerName_mp);

void BuildLayers();

void ENC_Converter(OGRLayer *Layer_ENC, OGRLayer *PointLayer, OGRLayer *PolyLayer, OGRLayer *LineLayer, string LayerName);


OGRLayer *Point_Layer, *Poly_Layer, *Line_Layer; 
GDALDataset *DS_pnt, *DS_poly, *DS_line;
OGRPolygon*  search_area_poly;
CMOOSGeodesy m_Geodesy;
