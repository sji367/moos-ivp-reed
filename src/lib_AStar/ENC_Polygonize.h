#ifndef ENC_Polygonize_H
#define ENC_Polygonize_H

#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <gdal.h>
#include "gdal_alg.h"
#include "gdal_frmts.h" // for GDAL/OGR
#include <ogrsf_frmts.h> // OGRLayer
#include "gdal_priv.h" // for rasterio
#include "cpl_conv.h" // for CPLMalloc()
#include "geodesy.h"
#include "gridinterp.h"


using namespace std;

class ENC_Polygonize
{
public:
    // Constructors\Deconstructor
    ENC_Polygonize(string MOOS_path, string tiffPath, string outfilePath, Geodesy Geod, double min_depth) {landZ = 5; closed = false; MOOS_PATH=MOOS_path; tiff_path=MOOS_path+"src/ENCs/Grid/"+tiffPath; outfile_path = MOOS_path+"src/ENCs/Grid/"+outfilePath; geod=Geod; minDepth =min_depth; buildSHP();}
    ENC_Polygonize(string MOOS_path, string tiffPath, string outfilePath, double LatOrigin, double LonOrigin, double min_depth) {landZ = 5; closed = false; MOOS_PATH=MOOS_path; tiff_path=MOOS_path+"src/ENCs/Grid/"+tiffPath; outfile_path = MOOS_path+"src/ENCs/Grid/"+outfilePath; geod=Geodesy(LatOrigin, LonOrigin); minDepth =min_depth; buildSHP();}
    ENC_Polygonize();
    ~ENC_Polygonize() {if (!closed){closeDS();}}

    // Initialize the shape file
    void buildSHP();

    // Actually create the polygons from the grid
    void polygonize();

    // Parse the tiff and make the binary grid
    string makeBinaryGrid();

    //void StoreShallowPolys(OGRLayer *PolyLayer);

    void closeDS() {GDALClose(ds); closed=true;}

    // Read and write functions for rasters
    void getRasterData(string filename, int &nXSize, int &nYSize, vector<float> &RasterData, vector<double> &adfGeoTransform);
    void writeRasterData(string filename, int nXSize, int nYSize, vector<int>&RasterData, vector<double> &adfGeoTransform);

    // Build the binary grid before making a binary grid and polygonizing it
    void runWithGrid(string ENC_Name, double grid_size, double buffer_size, double MHW_offset, bool SimpleGrid);
    void runWithGrid(string ENC_Name, double grid_size, double buffer_size, double MHW_offset) {runWithGrid(ENC_Name, grid_size, buffer_size, MHW_offset, false);}
    void runWithGrid() {runWithGrid("US5NH02M", 5,5, 3.564-0.829, false);}

    double getLandx() {return landZ; }



private:
    string MOOS_PATH, tiff_path, outfile_path;
    GDALDataset *ds;
    bool closed;
    OGRLayer *layer;
    OGRFeatureDefn *feat_def;
    Geodesy geod;
    double minDepth;
    double landZ;
    OGRSpatialReference oSRS;

};

#endif // ENC_Polygonize_H
