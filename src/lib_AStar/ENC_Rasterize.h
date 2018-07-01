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

using namespace std;

class ENC_Rasterize
{
public:
    ENC_Rasterize(string MOOSPath, string filename, double minX, double minY, double maxX, double maxY);
    ENC_Rasterize(string MOOSPath, string filename);
    ~ENC_Rasterize() {GDALClose(ds_shp);}

    void rasterize(string rasterfilename, double grid_size, char *attribute, char *dtype);
    char* double2charStar(double input);

private:
    string MOOS_Path, InputFileName;
    GDALDataset *ds_shp;
    double XMin, YMin, XMax, YMax;
    bool georef;


};

#endif // ENC_RASTERIZE_H
