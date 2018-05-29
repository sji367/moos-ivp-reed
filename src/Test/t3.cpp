#include "ENC_Polygonize.h"
#include "ogrsf_frmts.h" // GDAL
#include "geodesy.h"
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow

using namespace std;

double calcBuffer(int t_lvl, double m_ASV_length, double m_ASV_width) {return sqrt(pow(m_ASV_length,2)+pow(m_ASV_width,2))/2.0*(1+.4*t_lvl);}
double crossOver(double minA, double maxA, double buffer_width);

int main(int argc, char* argv[])
{

    double LatOrigin  = 43.071959194444446;
    double LongOrigin = -70.711610833333339;
    double m_MHW_Offset = 3;
    Geodesy geod = Geodesy(LatOrigin, LongOrigin);
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    GDALDataset *ds_grid;
    OGRLayer *GridLayer;
    double buff_size = calcBuffer(4, 1.8, 1.0);
    double grid_size = 5;

    //Grid_Interp(string MOOS_path, string ENC_Filename, double buffer_dist, double MHW_offset, double lat, double lon, bool SimpleGrid);
    string ENC_Name = "US3EC10M"; //"US5NH02M";
    Grid_Interp grid = Grid_Interp("", ENC_Name, grid_size, 2.735, LatOrigin, LongOrigin, false);
    grid.Run(false);

    return 0;
    /*
    // Build the grid and interp. Then make a binary grid (based on the desired minimum depth)and polygonize it
    ENC_Polygonize polygonize = ENC_Polygonize("", "PostProcess"+ ENC_Name +".tiff", "raster.shp",geod, m_MHW_Offset);
    polygonize.runWithGrid("US5NH02M",grid_size, buff_size, m_MHW_Offset, true);//US3EC10M
    polygonize.closeDS();

    string gridFilename = "src/ENCs/Grid/raster.shp";

    ds_grid = (GDALDataset*) GDALOpenEx( gridFilename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds_grid == NULL )
    {
        printf( "Open grid shp file failed.\n" );
        exit( 1 );
    }
    return 0;

    if (argc != 3)
    {
        printf("%s %s %s\n", argv[0], argv[1], argv[2]);
        return 1;
    }
    else
    {
        crossOver(atof(argv[1]),atof(argv[2]),20);
        return 0;
    }
*/
}

double crossOver(double minA, double maxA, double buffer_width)
{
    double min_angle_rawBuffered, min_angle_Buffered, max_angle_rawBuffered, max_angle_Buffered, BW;
    double minAngleGap = 4;
    double angleGap_buffered, angleGap_raw;

    //cout << fmod(minA-buffer_width,360) << "\t" << fmod(maxA+buffer_width,360) << endl;

    angleGap_raw = 360-abs(fmod(maxA,360)-fmod(minA,360));
    if (angleGap_raw >minAngleGap)
    {
        min_angle_rawBuffered = minA-buffer_width;
        max_angle_rawBuffered = maxA+buffer_width;
        min_angle_Buffered = fmod(min_angle_rawBuffered, 360);
        max_angle_Buffered = fmod(max_angle_rawBuffered, 360);
        angleGap_buffered = 360-(max_angle_Buffered-min_angle_Buffered);
        /*
        if (((min_angle_rawBuffered <360)&&(min_angle_rawBuffered >= 0)) && ((max_angle_rawBuffered <360)&&(max_angle_rawBuffered >= 0)))
        {
            angleGap_buffered = 360-(max_angle_Buffered-min_angle_Buffered);
            cout << "none" << endl;
        }
        else
        {
            // Case where both angles wrapped with the buffer
            if (!((min_angle_rawBuffered <360)&&(min_angle_rawBuffered >= 0)) && !((max_angle_rawBuffered <360)&&(max_angle_rawBuffered >= 0)))
            {
                angleGap_buffered = 360-(min_angle_Buffered-max_angle_Buffered);
                cout << "both" << endl;
            }

            // Case that only one of the angles is wrapped with the buffer
            else
            {
                angleGap_buffered = min_angle_Buffered-max_angle_Buffered;
                cout << "one" << endl;
            }
        }
        */
        // If the gap after the angles are buffered is less than 4, then
        //  make the gap 4 degrees.
        if ((angleGap_buffered<minAngleGap)||(angleGap_buffered>angleGap_raw))
        {
            BW = (angleGap_raw-minAngleGap)/2;
            if ((BW<buffer_width)&&(BW>0))
                buffer_width = BW;
        }
    }
    else
    {
        // The gap in the maximum angular extent is more than 30 degrees.
        //  Therefore remove the buffer completely.
        buffer_width = 0;
    }
    return buffer_width;
}
