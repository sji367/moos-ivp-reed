#include "astar.cpp"
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <ctime>
#include <boost/filesystem.hpp>
#include "gdal_frmts.h" // for GDAL/OGR

namespace fs = boost::filesystem;

using namespace std;

void buildExtraLayer(string filename, string route);

int main(int argc, char **argv)
{
    A_Star astar;
    int opt,d, g,c, w;
    bool f = false;
    bool o = false;
    bool startX = false;
    bool startY = false;
    bool endX = false;
    bool endY = false;
    double startLat, startLong, endLat, endLong;
    int connectingDist = 6;
    double gridSize = 5;
    double weight = 0.11;
    double depthCutoff = 1;
    string filename, outfile;
    if (argc>1)
    {
        while ((opt = getopt(argc,argv,"x:y:X:Y:f:g:o:d:c:w:h")) != EOF)
        {
            switch(opt)
            {
            case 'x':
            {
                startX = true;
                startLat= atof(optarg);
                cout << "Start Lat " << startLat << endl;
                break;
            }
            case 'y':
            {
                startY = true;
                startLong= atof(optarg);
                cout << "Start Long " << startLong<< endl;
                break;
            }
            case 'X':
            {
                endX = true;
                endLat = atof(optarg);
                cout << "Finish Lat " << endLat << endl;
                break;
            }
            case 'Y':
            {
                endY = true;
                endLong = atof(optarg);
                cout << "Finsh Long " << endLong << endl;
                break;
            }
            case 'f':
            {
                f=true;
                // Find the ENC on my computer (hard coded)
                filename = optarg;
                if(!fs::exists(filename))
                {
                    cout << "Cannot find file: " << filename <<endl;
                    return 1;
                }
                cout << "Tiff Filename:\t " << filename << endl;
                break;
            }

            case 'g':
            {
                g = 1;
                gridSize = atof(optarg);
                cout <<"Grid Resolution:\t "<< gridSize <<endl;
                break;
            }

            case 'o':
            {
                o = true;
                outfile = optarg;
                cout << "Output Filename:\t " << outfile << endl;
                break;
            }

            case 'd':
            {
                d=1;
                depthCutoff = atof(optarg);
                cout << "Depth Cutoff:\t" << depthCutoff << endl;
                break;
            }

            case 'c':
            {
                c = 1;
                connectingDist = atoi(optarg);
                cout <<"Connecting Dist:\t "<< connectingDist <<endl;
                break;
            }

            case 'w':
            {
                w = 1;
                weight = atof(optarg);
                cout <<"Depth Cost Weight:\t "<< weight <<endl;
                break;
            }
            case 'h':
            {
                fprintf(stderr,"You need to atleast put in the start and finish point (in Lat/Long) as well as the tiff file.\n   USAGE: A_Star -x startLat -y startLon -X endLat -Y endLon -f Tiff_FILENAME -g Grid_Resolution -o output_FILENAME -c connecting_dist -w depthCost_weight\n");
                return -1;
            }

            default:
                return 1;
            }
        }

        if((startX)&&(startY)&&(endX)&&(endY)&&(f))
        {
            astar = A_Star(gridSize, depthCutoff, weight, connectingDist);
            cout << "\tnum directions: "<<astar.getNumDir() << endl;
            astar.setMapFromTiff(filename);
            astar.setStartFinish_LatLong(startLat, startLong, endLat, endLong);
            astar.runA_Star(false);

            if (o)
                buildExtraLayer(outfile, astar.getRoute());

        }
        else
            fprintf(stderr,"You need to atleast put in the start and finish point (in Lat/Long) as well as the tiff file.\n   USAGE: A_Star -x startLat -y startLon -X endLat -Y endLon -f Tiff_FILENAME -g Grid_Resolution -o output_FILENAME -c connecting_dist -w depthCost_weight\n");
    }
    else
        fprintf(stderr,"You need to atleast put in the start and finish point (in Lat/Long) as well as the tiff file.\n   USAGE: A_Star -x startLat -y startLon -X endLat -Y endLon -f Tiff_FILENAME -g Grid_Resolution -o output_FILENAME -c connecting_dist -w depthCost_weight\n");

    return 0;
}

void buildExtraLayer(string filename, string route)
{
    // Build the datasource and layer that will hold independant points
    OGRLayer *layer;
    GDALAllRegister();
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    GDALDataset *ds = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL );

//        // Set the Spatial Reference
//        OGRSpatialReference oSRS;
//        char *pszSRS_WKT = NULL;
//        oSRS = geod.getUTM();
//        oSRS.exportToWkt( &pszSRS_WKT );
//        ds->SetProjection( pszSRS_WKT );
//        CPLFree( pszSRS_WKT );

    // Check to see if the datasource and layer are valid
    if( ds == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    // Create the layers
    layer = ds->CreateLayer( "line", NULL, wkbLineString25D, NULL );
    if( layer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }

    OGRFeature *new_feat;
    OGRLineString *line = (OGRLineString *)OGRGeometryFactory::createGeometry(wkbLineString25D);
    OGRFeatureDefn *poFDefn = layer->GetLayerDefn();
    new_feat =  OGRFeature::CreateFeature(poFDefn);

    // Parse the waypoint string
    vector<string> vect_WPTs;
    char delimiter = ',';
    vect_WPTs = split(route, delimiter);

    // Add the waypoints to a OGRLineString
    for (int i = 0; i<vect_WPTs.size(); i+=2)
    {
        line->addPoint(atof(vect_WPTs[i].c_str()), atof(vect_WPTs[i+1].c_str()));
    }
    new_feat->SetGeometry(line);

    // Build the new feature
    if( layer->CreateFeature( new_feat ) != OGRERR_NONE )
    {
        printf( "Failed to create feature in shapefile.\n" );
        exit( 1 );
    }

    GDALClose(ds);
}
