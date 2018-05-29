#include "ENC_Rasterize.h"

ENC_Rasterize::ENC_Rasterize(string MOOSPath, string filename, double minX, double minY, double maxX, double maxY)
{
    MOOS_Path = MOOSPath;
    GDALAllRegister();
    InputFileName = filename;
    string fullSHP_Filename = MOOS_Path+filename;
    ds_shp = (GDALDataset*) GDALOpenEx( fullSHP_Filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    XMin = (minX);
    XMax = (maxX);
    YMin = (minY);
    YMax = (maxY);
    georef = true;
}

ENC_Rasterize::ENC_Rasterize(string MOOSPath, string filename)
{
    MOOS_Path = MOOSPath;
    GDALAllRegister();
    InputFileName = filename;
    string fullSHP_Filename = MOOS_Path+filename;
    ds_shp = (GDALDataset*) GDALOpenEx( fullSHP_Filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    XMin = 0;
    XMax = 0;
    YMin = 0;
    YMax = 0;
    georef = false;
}

/* This function takes in the desired grid size and the path for the output tiff file. Currently all
 *  options are hard coded, except for the resolution and the burn value attribute
 *
 *      Inputs:
 *          grid_size - desired resolution of the raster
 *          rasterfile - path of the newly raster file (not including the MOOS_Path)
 *          attribute - attribute of the shp file that should be used as z
 *          dtype - data type for the outputed tiff
 */
void ENC_Rasterize::rasterize(string rasterpath, double grid_size, char* attribute, char* dtype)
{
    GDALDatasetH ds_tiff; // dataset for the tiff (Will return this. Need to be able to close it so it can save.)
    char *resolution_str, *xMin, *yMin, *xMax, *yMax;
    //string rasterpath = MOOS_Path+"src/ENCs/Grid/"+rasterfilename;

    // Convert the grid size and grid bounds code to a char*.
    // Allocate the size of the char*
    resolution_str = (char*)malloc(sizeof(grid_size)+1);
    xMin = (char*)malloc(sizeof(XMin)+1);
    yMin = (char*)malloc(sizeof(YMin)+1);
    xMax = (char*)malloc(sizeof(XMax)+1);
    yMax = (char*)malloc(sizeof(YMax)+1);

    // Convert the double into a char*
    sprintf(resolution_str,"%f",grid_size);
    sprintf(xMin,"%f",XMin);
    sprintf(yMin,"%f",YMin);
    sprintf(xMax,"%f",XMax);
    sprintf(yMax,"%f",YMax);


    // Create the list of options. Use the same as found on the command line gdal_rasterize utility
    //  http://www.gdal.org/gdal_rasterize.html
    char *options[] = {
                            "-at", // All touched (all pixels touched by lines or polygons will be
                                   //   updated, not just those whose center point is within the polygon)
                            "-a_nodata", "-10", // Set the value to be used if there is no data
                            "-tr", resolution_str, resolution_str, // Set the X and Y resolution (grid size)
                            "-ot", dtype, // Set the output type (in this case we are using double)
                            "-a", attribute,  // Set which attribute you want to use to determines the raster's value
                            "-te", xMin, yMin, xMax, yMax, // Set the X,Y bounds of the raster (i.e. touched extent)
                            nullptr // Need to add a null pointer to the end to tell the options that you are finished
                       };

    GDALRasterizeOptions *Options = GDALRasterizeOptionsNew(options, NULL);

    cout << "Starting to rasterize " << InputFileName << " as " << rasterpath << "." << endl;
    //cout << rasterpath << endl;

    ds_tiff = GDALRasterize(rasterpath.c_str(), NULL, ds_shp, Options, NULL);

    cout << "\t" << rasterpath << " was sucessfully rasterized." << endl << endl;

    GDALRasterizeOptionsFree(Options);
    GDALClose(ds_tiff);
}

// This function takes the inputted double, converts it to a char* and returns it.
char* ENC_Rasterize::double2charStar(double input)
{
    // Allocate the size of the char*
    char* output = (char*)malloc(sizeof(input)+1);

    // Convert to Char*
    sprintf(output,"%f",input);

    return output;
}
