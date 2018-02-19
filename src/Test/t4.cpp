#include "ogrsf_frmts.h" // GDAL
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    GDALAllRegister();
    GDALDataset *ds;
    OGRLayer *layer;
    OGRFeature *feat;
    int cntr = 0;

    string filename = "src/ENCs/US5NH02M/US5NH02M.000";

    ds = (GDALDataset*) GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds == NULL )
    {
        printf( "Open grid shp file failed.\n" );
        exit( 1 );
    }

    layer = ds->GetLayerByName("M_CSCL");
    feat = layer->GetNextFeature();
    while(feat)
    {
        cout << cntr<< ": " <<feat->GetFieldAsInteger("CSCALE") << endl;
        feat = layer->GetNextFeature();
        cntr ++;
    }

    return 0;
}
