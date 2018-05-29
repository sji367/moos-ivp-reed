#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <ctime>

#include "gridENC.h"

using namespace std;
namespace fs = boost::filesystem;

int main(int argc, char **argv){

    int opt,f,b,r,s;
    //fs::path ENCfilename = "";
    int buffer_dist = 0;
    double grid_size = -1;// if this doesnt get set, the gridding process will automatically pick the appropriate grid size
    double search_radius = -1;
    bool simpleGrid = false;
    bool CATZOC_poly = false;
    string ENCfilename, ENCname;

    if (argc>1)
    {
        while ((opt = getopt(argc,argv,"f:b:s:r::h")) != EOF)
        {
            switch(opt)
            {
            case 'f':
            {
                f = 1;
                // Find the ENC on my computer (hard coded)
                ENCname = optarg;
                ENCfilename = "/home/sreed/moos-ivp/moos-ivp-reed/src/ENCs/"+ENCname+"/"+ENCname+".000";
                if(!fs::exists(ENCfilename))
                {
                    cout << "Cannot find file: " << ENCfilename <<endl;
                    return 1;
                }

                cout <<"ENCFilename: "<< ENCname <<endl;
                break;
            }

            case 'b':
            {
                b = 1;
                buffer_dist = atoi(optarg);
                cout <<"Buffer     : "<< buffer_dist <<endl;
                break;
            }
            case 's':
            {
                s = 1;
                search_radius = atoi(optarg);
                cout << "Search Radius: "<< search_radius <<endl ;
                break;
            }
            case 'r':
            {
                r = 1;
                grid_size = atoi(optarg);
                cout << "Resolution : "<< grid_size <<endl ;
                break;
            }

            case 'h':
            {
                fprintf(stderr,"   USAGE: ENC_Grid -f ENC_FILENAME -b buffer -s search_radius -r grid_resolution\n");
            }

            default:
                return 1;
            }
        }
        // This is mean high water which is used to estimate depths of features whose features are not specified.
        // I'm not sure how to guess MHW programmatically.
        // FIX: Guess MHW programmatically.
        double MHW = 2.7;

        // Build the gridded ENC
        // This formulation creates a directory into which all temporaray and final output files are put.
        fs::path outputGriddir = fs::path(".");
        outputGriddir /= fs::path(ENCname);

        // Create the output directory.
        if(boost::filesystem::create_directory(outputGriddir)) {
               std::cout << "Success" << "\n";
        }

        // simpleGrid == False: Grids on land, soundings, contours, rocks, masked by rocks, wrecks
        // and obstructions, floating docks, pontoons, depth areas, land and outline.
        // simpleGrid == True: Grids only on land, soundings, contours, masked by depth areas, land and outline.
        // simpleGrid provides a grid based only on soundings for ENC_contact (reactive boat driving) as
        // these other components are handled separately. (the simple grid is used to create an adhoc set of contours
        // for reactive ship driving to prevent going aground.
        GridENC grid = GridENC(outputGriddir.string() + '/',
                                       ENCname,
                                       buffer_dist,
                                       grid_size,
                                       search_radius,
                                       MHW,
                                       simpleGrid,
                                       CATZOC_poly);


        // I don't think csv or .mat options are available.
        grid.Run(false);//(true, true); // Boleans are t/f build a .csv or .mat files for each raster

        return 0;
    }
    else
    {
        fprintf(stderr,"   USAGE: ENC_Grid -f ENC_FILENAME -b buffer -s search_radius -r grid_resolution\n");
    }
}

