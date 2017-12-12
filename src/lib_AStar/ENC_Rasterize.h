#ifndef ENC_RASTERIZE_H
#define ENC_RASTERIZE_H

#include <string>
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
#include "gridinterp.h"
#include "gdal_utils.h"

class ENC_Rasterize
{
public:
    ENC_Rasterize(string filename, string MOOSPath);
    ~ENC_Rasterize() {GDALClose(ds);}

    void run(int nXSize, int nYSize, string georef_extent);

private:
    string rasterfile, MOOS_Path;
    GDALDataset *ds;
    OGRLayer *layer;
    OGRFeatureDefn *feat_def;


};

#endif // ENC_RASTERIZE_H
