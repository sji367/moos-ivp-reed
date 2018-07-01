#include "poly_AngularSweep.h"
#include <vector>
#include <string>
#include <cmath> // for pow and sqrt
#include <algorithm> // for min_element and max_element
#include "geodesy.h" // Conversion between lat/lon and UTM
#include "ogrsf_frmts.h" // GDAL
#include <iostream>
#include <ctime>

using namespace std;
void movingAverageFilter(int windowSize, vector<double> &util);
OGRPolygon* searchArea(double m_ASV_x, double m_ASV_y, double m_ASV_head, double m_search_dist);

int main()
{
    GDALAllRegister();
    GDALDataset *ds;
    OGRLayer *layer;
    OGRFeature *feat;
    OGRGeometry *geom;

    double x = 47;//-23;//20.5311;
    double y = 52;//-65;//72;//-17.2426;
    double head = 187;
    double length = 1.8;
    double search = length*10*sqrt(2);
    double t_lvl = 5;

    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );

    string Filename = "src/ENCs/Shape/Poly.shp";

    ds= (GDALDataset*) GDALOpenEx( Filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds == NULL )
    {
        printf( "Open grid shp file failed.\n" );
        exit( 1 );
    }

    layer = ds->GetLayerByName("poly");

    //layer->SetSpatialFilterRect(-105,-138,94,20);
    layer->SetSpatialFilter(searchArea(x,y,head, search));
    polyAngularSweep angSweep = polyAngularSweep(x,y, length, search, 59, true);
    angSweep.build_search_poly();

    clock_t begin = clock();
    clock_t end;
    feat = layer->GetNextFeature();
    while(feat)
    {
        geom = feat->GetGeometryRef();
        t_lvl = feat->GetFieldAsDouble("T_Lvl");
        angSweep.findIntersections(geom, t_lvl);
        feat = layer->GetNextFeature();
    }
    cout << angSweep.getUtilVect_asString() << endl;
    end = clock();
    cout << double(end - begin) / CLOCKS_PER_SEC << endl;

    return 0;
}

// This function runs a moving average over a odd numvered size window. (If the
//  window size is not odd, then one will be added to the window size). Once the
//  moving average has been calculated, it is stored as the current utility only
//  if it less than the value currently stored in the utility vector.
void movingAverageFilter(int windowSize, vector<double> &util)
{
    int plusMinus = (windowSize-1)/2;
    int utilSize = util.size();
    vector<double> tempUtil = util;
    int curIndex = 0;
    double cumSum, movingAve;
    // Window size must be a odd number. Therefore if it is an even number then
    //  add one to the window size.
    if (windowSize%2==0)
        windowSize++;

    // Cycle through the utility vector and run a moving average filter.
    for (int index = 0; index<utilSize; index++)
    {
        cumSum = 0;
        for (int windowIndex = -plusMinus; windowIndex<=plusMinus; windowIndex++)
        {
            // Deal with the wrap around
            curIndex = index+windowIndex;
            if (curIndex < 0)
                curIndex += utilSize;
            else if (curIndex>=utilSize)
                curIndex -= utilSize;

            cumSum += tempUtil[curIndex];
        }

        // Store the average utility of the window if the utility is less than
        //  the value that was previously stored.
        movingAve = cumSum/(windowSize*1.0);
        if (util[index]>movingAve)
            util[index]=movingAve;
    }
}


OGRPolygon* searchArea(double m_ASV_x, double m_ASV_y, double m_ASV_head, double m_search_dist)
{
    OGRPolygon* search_area_poly;
    OGRLinearRing *poRing;

    double x1, x2, x3, x4, y1, y2, y3, y4;
    double theta = 45+fmod(m_ASV_head,90);
    double add_sin = m_search_dist*sqrt(2)*sin(theta*PI/180);
    double add_cos = m_search_dist*sqrt(2)*cos(theta*PI/180);

    x1 = m_ASV_x+add_sin;
    x2 = m_ASV_x-add_cos;
    x3 = m_ASV_x-add_sin;
    x4 = m_ASV_x+add_cos;

    y1 = m_ASV_y+add_cos;
    y2 = m_ASV_y+add_sin;
    y3 = m_ASV_y-add_cos;
    y4 = m_ASV_y-add_sin;

    search_area_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
    poRing = ( OGRLinearRing * ) OGRGeometryFactory::createGeometry(wkbLinearRing);
    poRing->addPoint(x1, y1);
    poRing->addPoint(x2, y2);
    poRing->addPoint(x3, y3);
    poRing->addPoint(x4, y4);
    poRing->closeRings();
    search_area_poly->addRing(poRing);
    search_area_poly->closeRings();
    return search_area_poly;
}

/*

int main()
{
    vector<double> util;
    util.resize(10);
    for (int i=0; i<10; i++)
    {
        if ((2<i)&&(i<=7))
        {
            util[i]=0;
        }
        else
            util[i] = 100;
    }
    cout << "Before:\t";
    for (auto i:util)
        cout << i << ",\t";
    cout << endl;

    movingAverageFilter(5,util);

    cout << "After:\t";
    for (auto i:util)
        cout << i << ",\t";
    cout << endl;
    return 0;
}
*/
