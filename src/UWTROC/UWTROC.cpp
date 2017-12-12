#include <vector>
#include <iostream>
#include <string>
#include "ogrsf_frmts.h" // for gdal
#include "dir_walk.h"

int main()
{
    GDALAllRegister();

    GDALDataset *ds_ENC;
    OGRLayer *layer;
    OGRFeature *feat;
    string valsou;
    double numRocks = 0;
    double noSounding = 0;


    vector<string> path, allENC_names;
    fs::path root = "/home/sreed/moos-ivp/moos-ivp-reed/src/ENCs/";

    // Store the paths to each ENC in the given directory
    Dir_Walk walk = Dir_Walk(root, ".000");
    walk.find_all_ext(path, allENC_names);

    for (int i =0; i<path.size(); i++)
    {
        ds_ENC = (GDALDataset*) GDALOpenEx( path[i].c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
        // Check if it worked
        if(!ds_ENC)
            cout <<"Opening " << allENC_names[i] <<" failed." << endl;
        else
        {
            layer = ds_ENC->GetLayerByName("UWTROC");
            if (layer)
            {
                layer->ResetReading();
                feat = layer->GetNextFeature();
                numRocks += layer->GetFeatureCount();
                while(feat)
                {
                    // Count the number of empty sounding values
                    valsou = feat->GetFieldAsString("VALSOU");
                    if (valsou.empty())
                        noSounding++;
                    feat = layer->GetNextFeature();
                }
            }
            GDALClose(ds_ENC);
        }
    }
    double percent = noSounding/numRocks*100;
    printf("%.0f / %.0f =  %.4f\n", noSounding, numRocks, percent);
    return 0;
}

