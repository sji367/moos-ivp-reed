#include "ENC_Rasterize.h"

ENC_Rasterize::ENC_Rasterize(string filename, string MOOSPath)
{
    GDALAllRegister();
    string full_Filename = MOOS_Path+"src/ENCs/Grid/" +filename;
    ds = (GDALDataset*) GDALOpenEx( full_Filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    layer = ds->GetLayer(0);



}

void ENC_Rasterize::run(int nXSize, int nYSize, string georef_extent)
{
    GDALDataset *poDataset;//, DS;
    //GDALRasterBand *poBand;
    //OGRSpatialReference oSRS;
    GDALDriver *poDriver;
    //char **opt = NULL;
    char *pszSRS_WKT = NULL;
    char **papszOptions = NULL;
    const char *pszFormat = "GTiff";
    double grid_size = 5;

    string full_Filename = MOOS_Path+"src/ENCs/Grid/new2.tiff";

    // Strings to hold the data for the input/output filenames for gdal_rasterize
    //string full_inFilename = MOOS_Path+"src/ENCs/Grid/" +infilename;
    //string full_outFilename = MOOS_Path+"src/ENCs/Grid/" +outfilename;
    //string filenames = full_inFilename + " " + full_outFilename;

    // String to hold the data for the georeferenced extents for gdal_rasterize
    //string georef_extent = "-te "+to_string(minX+geod.getXOrigin())+ " "+to_string(minY+geod.getYOrigin())+ " "
    //        +to_string(maxX+geod.getXOrigin())+ " "+to_string(maxY+geod.getYOrigin())+ " ";

    // String for all the options for gdal_rasterize
    //string option = "-a_nodata -1000 -at -a Depth -tr " + to_string(grid_size)+ " " +
    //        to_string(grid_size)+ " -a_srs EPSG:2219 -ot Int32 "; //+ georef_extent;// + filenames;
    //char **option = "-a_nodata -1000 -at -a Depth -tr 5 5 -a_srs EPSG:2219 -ot Int32 ";

    //const char *opt1 = option.c_str();
    //const char **opt = *opt1;
    GDALRasterizeOptionsForBinary *binOp;// = NULL;
    //GDALRasterizeOptions *options = GDALRasterizeOptionsNew(option,binOp);

    // Open file for writing
    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    poDataset = poDriver->Create( full_Filename.c_str(), nXSize, nYSize, 1, GDT_Float32, papszOptions );
    GDALRasterize(NULL, poDataset, ds, NULL, NULL);
    //GDALRasterizeOptionsFree(options);
    //GDALClose(DS);
    GDALClose(poDataset);
}
