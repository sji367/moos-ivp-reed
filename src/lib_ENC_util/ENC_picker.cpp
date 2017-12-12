#include <ENC_picker.h>

ENC_Picker::ENC_Picker()
{
  RNC.clear();
  setOrigin(0,0);
  root_directory = "src/ENCs/";
  csvfile = root_directory+"ENCList.csv";
  cntr = 0;
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = "src/ENCs/";
  csvfile = root_directory+"ENCList.csv";
  cntr = 0;
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin, string root_dir)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = root_dir;
  csvfile = root_directory+"ENCList.csv";
  cntr = 0;
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin, string root_dir, string csvfilename)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = root_dir;
  csvfile = root_directory+"/"+csvfilename;
  cntr = 0;
}

void ENC_Picker::parse_csv()
{
  // Make sure that the RNC map is clear
  RNC.clear();
  
  strtk::token_grid::options option;
  option.column_delimiters = ",";

  // Read in the file
  strtk::token_grid grid(csvfile, option);
  strtk::token_grid::row_type row;

  // Skip the headers and any chart that does not have an associated scale
  for(size_t i = 1; i < grid.row_count(); ++i)
    {
      row = grid.row(i);
      if (row.get<string>(1)!="nan")
	{
	  RNC[row.get<string>(0)] = row.get<int>(1);
	}
    }
}

void ENC_Picker::build_ENC_outlines()
{
    GDALAllRegister();

    // Check to see if the outline.shp file has been built
    string outline_fileName = root_directory+"outline.shp";
    outline_ds = (GDALDataset*) GDALOpenEx(outline_fileName.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if (outline_ds)
        outline_layer = outline_ds->GetLayer(0);

    // Otherwise build the necessary shapefile
    else
    {
        cout << "Building new shp file" << endl;
        GDALDataset *ds_ENC, *ds_shp;
        OGRLayer *layer, *layer_shp;
        OGRFeature *feat, *new_feat;
        OGRGeometry *geom;
        OGRPolygon *newPoly;

        // Build the dataset
        const char *pszDriverName = "ESRI Shapefile";
        GDALDriver *poDriver;
        poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );

        // Build the shapefile
        ds_shp = poDriver->Create(outline_fileName.c_str(), 0, 0, 0, GDT_Unknown, NULL );
        if( ds_shp == NULL )
        {
            printf( "Creation of output file failed.\n" );
            exit( 1 );
        }

        // Create the layer
        layer_shp = ds_shp->CreateLayer( "Point", NULL, wkbPolygon, NULL );
        if( layer_shp == NULL )
        {
            printf( "Layer creation failed.\n" );
            exit( 1 );
        }

        // Create the fields for the layer
        OGRFieldDefn oField_scale( "Scale", OFTInteger);
        OGRFieldDefn oField_RNC( "RNC_Name", OFTString);
        OGRFieldDefn oField_ENC( "ENC_Name", OFTString);
        oField_ENC.SetWidth(25);
        oField_RNC.SetWidth(25);

        OGRFeatureDefn *poFDefn = layer_shp->GetLayerDefn();

        if( layer_shp->CreateField( &oField_scale ) != OGRERR_NONE )
        {
            printf( "Creating ENC Scale field failed.\n" );
            exit( 1 );
        }
        if( layer_shp->CreateField( &oField_RNC ) != OGRERR_NONE )
        {
            printf( "Creating ENC Name field failed.\n" );
            exit( 1 );
        }
        if( layer_shp->CreateField( &oField_ENC ) != OGRERR_NONE )
        {
            printf( "Creating ENC Name field failed.\n" );
            exit( 1 );
        }


        string source_indication;
        vector<string> path, allENC_names, temp_split, temp_split2;
        fs::path root = root_directory;

        Dir_Walk walk = Dir_Walk(root, ".000");
        walk.find_all_ext(path, allENC_names);

        for (int i =0; i<path.size(); i++)
        {
            // Clear the vectors
            temp_split.clear(); temp_split2.clear();

            ds_ENC = (GDALDataset*) GDALOpenEx( path[i].c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
            // Check if it worked
            if(!ds_ENC)
                cout <<"Opening " << allENC_names[i] <<" failed." << endl;
            else
            {
                layer = ds_ENC->GetLayerByName("M_NPUB");
                if (layer)
                {
                    layer->ResetReading();
                    feat = layer->GetNextFeature();
                    source_indication = feat->GetFieldAsString("SORIND");
                    // Parse the information to get the RNC chart number
                    //  In the format:
                    //    Country, Authority, Source, ID-Code
                    if (!source_indication.empty())
                    {
                        split(temp_split, source_indication, ',');
                        if (temp_split.size() == 4)
                        {
                            split(temp_split2, temp_split.back(), ' ');
                            if (temp_split2.size() == 2)
                            {
                                if ((temp_split2[0] == "chart")||(temp_split2[0] == "Chart")||(temp_split2[0] == "CharT") )
                                {
                                    // Build the outline polyon
                                    geom = feat->GetGeometryRef();
                                    newPoly = (OGRPolygon *) geom;

                                    // Build the new feature
                                    new_feat =  OGRFeature::CreateFeature(poFDefn);
                                    new_feat->SetField("RNC_Name", temp_split2.back().c_str());
                                    new_feat->SetField("ENC_Name", allENC_names[i].c_str());
                                    new_feat->SetGeometry(newPoly);

                                    if( layer_shp->CreateFeature( new_feat ) != OGRERR_NONE )
                                    {
                                        printf( "Failed to create feature in shapefile.\n" );
                                        exit( 1 );
                                    }
                                    OGRFeature::DestroyFeature( new_feat );
                                }
                            }
                        }
                    }
                }
                GDALClose(ds_ENC);
            }
        }
        GDALClose(ds_shp);
        outline_ds = (GDALDataset*) GDALOpenEx(outline_fileName.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
        outline_layer = outline_ds->GetLayerByName("outline");
        cout << "Outline shape file built." << endl;
    }
}

void ENC_Picker::pick_ENC(string &chart_name, string &RNC_name, int &chart_scale)
{
    string ENCs_RNC;
    OGRFeature *feat;
    OGRGeometry *geom;
    int temp_scale;
    chart_scale = -1;

    // Get outlines of the ENCs and parse the csv file that relates the RNCs
    //  with scales
    parse_csv();

    build_ENC_outlines();
    outline_layer->ResetReading();
    feat = outline_layer->GetNextFeature();
    while(feat)
    {
        geom = feat->GetGeometryRef();
        if (point->Within(geom))
        {
            ENCs_RNC= feat->GetFieldAsString("RNC_Name");
            map<string,int>::iterator search;
            search = RNC.find(ENCs_RNC);
            // If the origin lies within the ENC and its a smaller scale, then
            //  store the ENC's name and scale
            if(search != RNC.end())
            {
                temp_scale = search->second;
                if ((temp_scale < chart_scale)||(chart_scale==-1))
                {
                  RNC_name = search->first;
                  chart_name = feat->GetFieldAsString("ENC_Name");
                  chart_scale = temp_scale;
                }
            }
            else
                cout << ENCs_RNC << " not found" << endl;
        }
        feat = outline_layer->GetNextFeature();
    }
}

void split(vector<string>& parsed, const string& s, const char c)
{
  unsigned int i = 0;
  unsigned int j = s.find(c);
  while (j < s.length()) {
    parsed.push_back(s.substr(i, j - i));
    i = ++j;
    j = s.find(c, j);
    if (j >= s.length()) {
      parsed.push_back(s.substr(i, s.length()));
      break;
    }
  }
}
