#include "ENC_Rasterize.h"

ENC_Rasterize::ENC_Rasterize(string MOOSPath, string filename)//, char *minX, char *minY, char *maxX, char *maxY)
{
    MOOS_Path = MOOSPath;
    GDALAllRegister();
    InputFileName = filename;
    string full_Filename = MOOS_Path+filename;
    ds = (GDALDataset*) GDALOpenEx( full_Filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
//    xMin = minX;//string2CharStar(minX);
//    xMax = maxX;//string2CharStar(maxX);
//    yMin = minY;//string2CharStar(minY);
//    yMax = maxY;//string2CharStar(maxY);
}

/* This function takes in the desired grid size and the path for the output tiff file. Currently all
 *  options are hard coded, except for the resolution and the burn value attribute
 *
 *      Inputs:
 *          grid_size - desired resolution of the raster
 *          rasterfile - path of the newly raster file (not including the MOOS_Path)
 *          attribute - attribute of the shp file that should be used as z
 */
void ENC_Rasterize::rasterize(string rasterpath, double grid_size, char* attribute)
{
    char *resolution_str;

    cout << "Starting to rasterize " << InputFileName << " as " << rasterpath << "." << endl;

    // Convert the grid size and EPSG code to a char*.
    resolution_str = (char*)malloc(sizeof(grid_size)+1);
    sprintf(resolution_str,"%f",grid_size);

    // Create the list of options. Use the same as found on the command line gdal_rasterize utility
    //  http://www.gdal.org/gdal_rasterize.html
    char *options[] = {
                            "-at", // All touched (all pixels touched by lines or polygons will be
                                   //   updated, not just those whose center point is within the polygon)
                            "-a_nodata", "-10", // Set the value to be used if there is no data
                            "-tr", resolution_str, resolution_str, // Set the X and Y resolution (grid size)
                            "-ot", "Float64", // Set the output type (in this case we are using double)
                            "-a", attribute // Set which attribute you want to use to determines the raster's value
                            //"-te", xMin, yMin, xMax, yMax // Set the X,Y bounds of the raster (i.e. touched extent)
                       };

    // cout << CSLCount(options) << endl;
    GDALRasterizeOptions *Options = GDALRasterizeOptionsNew(options, NULL);

    // Create the new raster in the location given in rasterfile
    rasterfile = MOOS_Path+rasterpath;
    GDALRasterize(rasterfile.c_str(), NULL, ds, Options, NULL);

    GDALRasterizeOptionsFree(Options);

    cout << rasterpath << " was sucessfully rasterized." << endl;
}

char* ENC_Rasterize::string2CharStar(string str)
{
//    char *output = new char[input.size() + 1];
//    copy(input.begin(), input.end(), output);
//    output[input.size()] = '\0';

    vector<char> v(str.begin(), str.end());
    char* output = &v[0];
    return output;
}

char* ENC_Rasterize::double2CharStar(double value)
{
    char* str;
    str = (char*)malloc(sizeof(value)+1);
    sprintf(str,"%f",value);
    return str;
}
