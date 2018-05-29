#include "ogrsf_frmts.h" // GDAL
#include <iostream>
#include <string>
#include <vector>

using namespace std;
void fnc(int dX, int dY);

int main(int argc, char* argv[])
{
    /*
    GDALAllRegister();
    GDALDataset *ds;

    string filename = "src/ENCs/US3EC09M/US3EC09M.000";

    ds = (GDALDataset*) GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( ds == NULL )
    {
        printf( "Open grid shp file failed.\n" );
        exit( 1 );
    }

    int Chart_Scale = ds->GetLayerByName("DSID")->GetFeature(0)->GetFieldAsInteger("DSPM_CSCL");

    cout << Chart_Scale << endl;
    */
    fnc(5,2);
    return 0;
}

void fnc(int dX, int dY)
{
    int wptX = 0;
    int wptY =0;
    int ceil_y, floor_y, X, Y;
    int total_points = 5*dX;
    double df_total_points = total_points*1.0;
    double intermidate, x, y;
    double m = (1.0*dY)/(dX*1.0);
    int depth_cutoff = 1;
    vector<vector<int> > Map;
    Map.resize(6, vector<int> (4,0));


    Map[0][0] = 1;
    Map[1][0] = 1;
    Map[2][0] = 1;
    Map[3][0] = 1;
    Map[0][1] = 1;
    Map[1][1] = 1;
    Map[2][1] = 1;
    Map[3][1] = 1;
    Map[0][2] = 1;
    Map[3][5] = 1;
    Map[2][5] = 1;

    // Cycle through all segmented grid nodes to check for path validity
    //  determine the total water depth traveled through
    for (int j=1; j<total_points; j++)
    {
        intermidate = (j*1.0)/(df_total_points);
        // Calculate the x gride node
        if (signbit(dX))// Returns true if negative
            x = wptX-intermidate;
        else
            x = wptX+intermidate;

        // Calculate the y gride node
        y= m*x;

        // Store the grid node's X,Y position as ints
        X= static_cast<int>(round(x));
        Y= static_cast<int>(round(y));

        // If either the grid node or the next closest node on the line in the x direction
        //  is an obstacle, the path is not valid
        floor_y = int(floor(y));
        ceil_y = int(ceil(y));
        if ((Map[floor_y][X] < depth_cutoff) || (Map[ceil_y][X] < depth_cutoff))
        {
            cout << x <<"," << y <<"\t" <<X <<"," << Y << " BAD!"<< endl;
            return; // Path is invalid
        }
        else
            cout << x <<"," << y <<"\t" << X <<"," << Y << " Good!"<<endl;
    }
}
