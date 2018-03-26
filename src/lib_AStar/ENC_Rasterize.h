#ifndef ENC_RASTERIZE_H
#define ENC_RASTERIZE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <cmath>
#include <gdal.h>
#include "gdal_alg.h"
#include "gdal_frmts.h" // for GDAL/OGR
#include <ogrsf_frmts.h> // OGRLayer
#include "gdal_priv.h" // for rasterio
#include "gdal_utils.h" // GDALRasterize
#include "cpl_conv.h" // for CPLMalloc()
#include "geodesy.h"

#include "cpl_string.h"
//#include "/home/sreed/gdal/apps/gdal_rasterize_lib.cpp"

using namespace std;

class ENC_Rasterize
{
public:
    ENC_Rasterize(string MOOSPath, string filename, char * minX, char * minY, char * maxX, char * maxY) {}
    ENC_Rasterize(string MOOSPath, string filename);
    ~ENC_Rasterize() {GDALClose(ds);}

    void rasterize(string rasterpath, double grid_size, char *attribute);
    char* string2CharStar(string str);
    char* double2CharStar(double value);

private:
    string rasterfile, MOOS_Path, InputFileName;
    GDALDataset *ds;
    char *xMin, *yMin, *xMax, *yMax;


};

#endif // ENC_RASTERIZE_H
