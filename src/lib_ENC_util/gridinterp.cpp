#include "gridinterp.h"

Grid_Interp::Grid_Interp()
{
  geod = Geodesy();
  grid_size = 5;
  buffer_size = 5;
  MOOS_Path = "/home/sji367/moos-ivp/moos-ivp-reed/";
  ENC_filename = "/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs/US5NH02M/US5NH02M.000";
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

Grid_Interp::Grid_Interp(string MOOS_path, string ENC_Filename, double Grid_size, double buffer_dist, Geodesy Geod)
{
  initGeodesy(Geod);
  grid_size = Grid_size;
  ENC_filename = ENC_Filename;
  MOOS_Path = MOOS_path;
  buffer_size = buffer_dist;
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

Grid_Interp::Grid_Interp(string MOOS_path, string ENC_Filename, double Grid_size, double buffer_dist, double lat, double lon)
{
  geod = Geodesy(lat, lon);
  grid_size = Grid_size;
  ENC_filename = ENC_Filename;
  MOOS_Path = MOOS_path;
  buffer_size = buffer_dist;
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

void Grid_Interp::buildLayers()
{
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    OGRFieldDefn oField_depth( "Depth", OFTReal);
    OGRFieldDefn oField_inside( "Inside_ENC", OFTReal);

    // Build the datasource and layer that will hold the wrecks and land area polygons
    string polyPath = MOOS_Path+"src/ENCs/Grid/polygon.shp";
    ds_poly = poDriver->Create(polyPath.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( ds_poly == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    // Create the layers
    layer_poly = ds_poly->CreateLayer( "poly", NULL, wkbPolygon25D, NULL );
    if( layer_poly == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    // Create the field
    if(layer_poly->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    feat_def_poly = layer_poly->GetLayerDefn();



    // Build the datasource layer that will hold the depth area polygons
    string depthPath = MOOS_Path+"src/ENCs/Grid/depth.shp";
    ds_depth = poDriver->Create(depthPath.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( ds_depth == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    // Create the layer
    layer_depth = ds_depth->CreateLayer( "depth", NULL, wkbPolygon25D, NULL );
    if( layer_depth == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    // Create the field
    if(layer_depth->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    feat_def_depth = layer_depth->GetLayerDefn();


    // Build the datasource and layer that will hold the outline of the ENC
    string outlinePath = MOOS_Path+"src/ENCs/Grid/outline.shp";
    ds_outline = poDriver->Create(outlinePath.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( ds_poly == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    // Create the layers
    layer_outline = ds_outline->CreateLayer( "outline", NULL, wkbPolygon25D, NULL );
    if( layer_outline == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    // Create the field
    if(layer_outline->CreateField( &oField_inside ) != OGRERR_NONE )
    {
        printf( "Creating Inside_ENC field failed.\n" );
        exit( 1 );
    }
    feat_def_outline = layer_outline->GetLayerDefn();
}

// This function creates a grid from the soundings, land areas, rocks, wrecks, depth areas
//  and depth contours and store it as a 2D vector of ints where the depth is stored in cm.
void Grid_Interp::Run(bool csv)
{
    clock_t start = clock();
    GDALAllRegister();
    GDALDataset* ds;

    buildLayers();

    ds = (GDALDataset*) GDALOpenEx( ENC_filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );

    getENC_MinMax(ds);

    vector<int> poly_rasterdata, depth_area_rasterdata, ENC_outline_rasterdata;
    int poly_extentX, poly_extentY, depth_extentX, depth_extentY, outline_extentX, outline_extentY;

    // Add the vertices from the soundings, land areas, rocks, wrecks, and depth contours
    //  to vectors for gridding
    layer2XYZ(ds->GetLayerByName("SOUNDG"), "SOUNDG");
    layer2XYZ(ds->GetLayerByName("UWTROC"), "UWTROC");
    layer2XYZ(ds->GetLayerByName("WRECKS"), "WRECKS");
    layer2XYZ(ds->GetLayerByName("PONTON"), "PONTON");
    layer2XYZ(ds->GetLayerByName("DEPCNT"), "DEPCNT");
    layer2XYZ(ds->GetLayerByName("M_COVR"), "M_COVR");
    layer2XYZ(ds->GetLayerByName("LNDARE"), "LNDARE");
    layer2XYZ(ds->GetLayerByName("DEPARE"), "DEPARE");
    layer2XYZ(ds->GetLayerByName("FLODOC"), "FLODOC");

    // Save the rasters
    GDALClose(ds_depth);
    GDALClose(ds_poly);
    GDALClose(ds_outline);

    // Rasterize the polygon and depth area layers
    rasterizeSHP("polygon.tiff","polygon.shp", "Depth");
    rasterizeSHP("depth_area.tiff","depth.shp", "Depth");
    rasterizeSHP("outline.tiff", "outline.shp", "Inside_ENC");

    // Store the rasterized data as 1D vectors
    getRasterData("polygon.tiff", poly_extentX, poly_extentY, poly_rasterdata);
    getRasterData("depth_area.tiff", depth_extentX, depth_extentY, depth_area_rasterdata);
    getRasterData("outline.tiff", outline_extentX, outline_extentY, ENC_outline_rasterdata);

    // Grid the data
    GDALGridLinearOptions options;
    options.dfRadius = 0;
    options.dfNoDataValue = -1000;

    int x_res = int(round((maxX - minX)/grid_size));
    int y_res = int(round((maxY - minY)/grid_size));


    griddedData.resize(y_res*x_res);


    Map.clear();
    Map.resize(x_res, vector<int> (y_res,0));
    //Map.resize(y_res,x_res);

    /*
    OGRSpatialReference UTM = geod.getUTM();
    // Build vector of layers
    vector<OGRLayer*> layers;
    layers.push_back(land);

    // Set spatial Reference
    char    *spat_ref = NULL;
    UTM.exportToWkt(&spat_ref);

    // Set the rasterize options
    char** opt = nullptr;
    opt = CSLSetNameValue(opt, "ALL_TOUCHED", "TRUE");

    // Build the geotransformation matrix of the target raster
    vector<double> geoTransform = {minX, grid_size, 0, maxY, 0, -grid_size};
    if (GDALRasterizeLayersBuf(Map.data(), x_res, y_res, GDT_Int32, 0, 0, 1, (OGRLayerH*)&layers[0], spat_ref, geoTransform.data(), NULL, NULL, -500, opt, NULL, NULL ) == CE_Failure)
    */

    cout << "Starting to grid data" << endl;
    printf("Data points: %d, Grid: %dx%d\n", static_cast<int>(X.size()), y_res,x_res);

    //printf("X:%d, Y:%d, Z:%d\n", static_cast<int>(X.size()), static_cast<int>(Y.size()), static_cast<int>(depth.size()));
    // Grid the data
    if (GDALGridCreate(GGA_Linear, &options, static_cast<int>(X.size()), X.data(), Y.data(), depth.data(), minX, maxX, minY, maxY, x_res, y_res, GDT_Int32, griddedData.data(), NULL, NULL) == CE_Failure)
    {
        cout << "Gridding Failed" << endl;
        exit(1);
    }

    cout << "Gridded" << endl;

    // Combine the 3 1D vectors to one 2D vector
    updateMap(poly_rasterdata, depth_area_rasterdata, ENC_outline_rasterdata, x_res, y_res);

    if (csv)
        write2csv(poly_rasterdata, depth_area_rasterdata, x_res, y_res, true);

    clock_t end = clock();
    double total_time = double(end - start) / CLOCKS_PER_SEC;
    cout << "Elapsed Time: " << total_time << " seconds." << endl;
}

vector<vector<int> > Grid_Interp::transposeMap()
{
    vector <vector<int> > t_map (Map[0].size(), vector<int> (Map.size(),0));

    for (int i =0; i<Map.size(); i++)
        for (int j=0; j<Map[0].size(); j++)
        {
            t_map[j][i] = Map[i][j];
        }
    return t_map;
}

// Build the 2D map from the 4 1D vectors
void Grid_Interp::updateMap(vector<int> &poly_data, vector<int> &depth_data, vector<int> &outline_data, int x_res, int y_res)
{
    int x, y;
    int rasterIndex = 0;

    // Make sure that the size of all of the 1D vectors are the same
    if ((poly_data.size() == depth_data.size()) && (poly_data.size() == griddedData.size()) && (poly_data.size() == outline_data.size()))
    {
        for (int i =0; i<griddedData.size(); i++)
        {
            rasterIndex2gridIndex(i,rasterIndex, x_res, y_res);
            row_major2grid(i, x, y, x_res);

            // Make sure that the datapoint is inside of the ENC
            if (outline_data[rasterIndex] == 1)
            {
                // Check to see if the index is not land.
                if (poly_data[rasterIndex] != -500)
                {
                    // If it is not land make sure that is not below the minimum depth given by the depth area
                    if((depth_data[rasterIndex] != -1000) && (depth_data[rasterIndex] > griddedData[i]))
                    {
                        Map[y][x] = depth_data[rasterIndex];
                    }
                    else
                        Map[y][x] = griddedData[i];
                }
                else
                    Map[y][x] =poly_data[rasterIndex];
            }
            else
                Map[y][x] = outline_data[rasterIndex];
        }
    }
    else
    {
        printf("Poly: %d, Depth: %d, Grid:%d, ENC Outline:%d\n", poly_data.size(), depth_data.size(), griddedData.size(), outline_data.size());
        cout << "The vectors are the wrong size. Exiting...\n";
        exit(1);
    }
}

// Save the 1D vectors (polygon, depth area, pre-processed interpolated grid) and the 2D
//  post-processed interpolated grid as csv files
void Grid_Interp::write2csv(vector<int> &poly_data, vector<int> &depth_data, int x_res, int y_res, bool mat)
{
    ofstream combined, grid, poly, DA;
    int x,y;
    int prev_x = 0;
    int rasterIndex = 0;

    string postcsv= MOOS_Path+"post.csv";
    string precsv= MOOS_Path+"pre.csv";
    string polycsv= MOOS_Path+"poly.csv";
    string DAcsv= MOOS_Path+"DA.csv";


    // File to write data to csv format
    combined.open(postcsv.c_str());
    grid.open(precsv.c_str());
    poly.open(polycsv.c_str());
    DA.open(DAcsv.c_str());

    cout << "Writing data to CSV files." << endl;

    // Make sure that the size of all of the 1D vectors are the same
    if ((poly_data.size() == depth_data.size()) && (poly_data.size() == griddedData.size()))
    {
        for (int i =0; i<griddedData.size(); i++)
        {
            rasterIndex2gridIndex(i,rasterIndex, x_res, y_res);
            row_major2grid(i, x, y, x_res);

            if (x!=prev_x)
            {
                grid << "\n";
                poly << "\n";
                DA << "\n";
                combined << "\n";
            }

            // Store the data in a csv
            combined << Map[y][x]<< ",";
            grid << griddedData[i] << ",";
            poly << poly_data[i] << ",";
            DA << depth_data[i] << ",";
            prev_x =x;
        }
        combined.close();
        grid.close();
        poly.close();
        DA.close();

        // Write these files to .mat
        if (mat)
        {
            cout << "Writing data to .mat files." << endl;
            writeMat("post");
            writeMat("pre");
            writeMat("poly");
            writeMat("DA");
        }
    }
    else
    {
        printf("Poly: %d, Depth: %d, Grid:%d\n", poly_data.size(), depth_data.size(), griddedData.size());
        cout << "The vectors are the wrong size. Exiting...\n";
        exit(1);
    }
}

void Grid_Interp::writeMat(string filename)
{
    string toMat = "python " + MOOS_Path+"scripts/csv2mat.py " + MOOS_Path+filename+".csv " + MOOS_Path+filename+".mat";
    cout << toMat << endl;
    system(toMat.c_str());
}

// Get the minimum and maximum x/y values (in local UTM) of the ENC
void Grid_Interp::getENC_MinMax(GDALDataset* ds)
{
    OGRLayer* layer;
    OGRFeature* feat;
    OGRLinearRing* ring;
    OGRGeometry* geom;
    OGRPolygon* coverage;

    vector<double> x,y;
    double UTM_x, UTM_y;
    double remainder;

    layer = ds->GetLayerByName("M_COVR");
    layer->ResetReading();
    feat = layer->GetNextFeature();

    // Cycle through the points and store all x and y values of the layer
    for (int i=0; i<layer->GetFeatureCount(); i++)
    {
        if (feat->GetFieldAsDouble("CATCOV") == 1)
        {
            geom = feat->GetGeometryRef();
            coverage = ( OGRPolygon * )geom;
            ring = coverage->getExteriorRing();
            for (int j =0;  j<ring->getNumPoints(); j++)
            {
                geod.LatLong2LocalUTM(ring->getY(j), ring->getX(j), UTM_x, UTM_y);
                x.push_back(UTM_x);
                y.push_back(UTM_y);
            }
        }
            feat = layer->GetNextFeature();
    }

    // Find the min and max of the X and Y values
    minX = *min_element(x.begin(), x.end());
    maxX = *max_element(x.begin(), x.end());
    minY = *min_element(y.begin(), y.end());
    maxY = *max_element(y.begin(), y.end());

    // Make sure each one of min/max values are divisable by grid_size
    remainder = fmod(minX, grid_size);
    if (remainder != 0)
        minX -= remainder+grid_size;

    remainder = fmod(minY, grid_size);
    if (remainder != 0)
        minY -= remainder+grid_size;

    remainder = fmod(maxX, grid_size);
    if (remainder != 0)
        maxX += grid_size - remainder;

    remainder = fmod(maxY, grid_size);
    if (remainder != 0)
        maxY += grid_size - remainder;

}

// This function takes in an OGRLayer and sorts out which function it should
//  use based upon the geometry type of the feature for interpolation
void Grid_Interp::layer2XYZ(OGRLayer* layer, string layerName)
{
  OGRFeature* feat;
  OGRGeometry* geom;
  string geom_name;

  if (layer != NULL)
    {
      layer->ResetReading();
      //cout << layerName << " has " << layer->GetFeatureCount() << " features." << endl;
      while( (feat = layer->GetNextFeature()) != NULL )
        {
          geom = feat->GetGeometryRef();
          geom_name = geom->getGeometryName();
          if (geom_name == "MULTIPOINT")
            multipointFeat(feat, geom);
          else if (geom_name == "POLYGON")
            polygonFeat(feat, geom, layerName);
          else if (geom_name == "LINESTRING")
            lineFeat(feat, geom, layerName);
          else if (geom_name == "POINT")
            pointFeat(feat, geom, layerName);
        }
    }
  else
    cout << layerName << " is not in the ENC!" << endl;
}

// This function converts the mulitpoints into Local UTM, and
//  stores the vertices for interpolation
void Grid_Interp::multipointFeat(OGRFeature* feat, OGRGeometry* geom)
{
    OGRGeometry* poPointGeometry;
    OGRMultiPoint *poMultipoint;
    OGRPoint * poPoint;

    double lat,lon, x, y, z;

    // Cycle through all points in the multipoint and store the xyz location of each point
    geom = feat->GetGeometryRef();
    poMultipoint = ( OGRMultiPoint * )geom;
    for(int iPnt = 0; iPnt < poMultipoint->getNumGeometries(); iPnt++ )
      {
        // Make the point
        poPointGeometry = poMultipoint ->getGeometryRef(iPnt);
        poPoint = ( OGRPoint * )poPointGeometry;
        // Get the location in the grid coordinate system
        lon = poPoint->getX();
        lat = poPoint->getY();
        geod.LatLong2LocalUTM(lat,lon, x,y);

        // Get depth in cm
        z= poPoint->getZ()*100;
        if (!z)
            return;
        else
            depth.push_back(z);

        X.push_back(x);
        Y.push_back(y);
      }
}

// This function converts the lines into Local UTM, segements the lines, and
//  stores the vertices for interpolation
void Grid_Interp::lineFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRLineString *UTM_line;
    OGRGeometry *geom_UTM;

    double x,y,z;

    geom_UTM = geod.LatLong2UTM(geom);
    UTM_line = ( OGRLineString * )geom_UTM;

    UTM_line->segmentize(grid_size);

    if (layerName=="DEPCNT")
        z = feat->GetFieldAsDouble("VALDCO")*100;
    else if ((layerName =="PONTON")||(layerName =="FLODOC"))
        z = -500;
    else
    {
        cout << "Unknown Line Layer: " << layerName << endl;
        return;
    }

    // Get the location in the grid coordinate system
    for (int j=0; j<UTM_line->getNumPoints(); j++)
    {
        x = UTM_line->getX(j)-geod.getXOrigin();// convert to local UTM
        y = UTM_line->getY(j)-geod.getYOrigin();// convert to local UTM

        // Store the XYZ of the vertices
        X.push_back(x);
        Y.push_back(y);
        depth.push_back(z); // in cm
    }

}

void Grid_Interp::pointFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRPoint * poPoint;
    double lat,lon, x,y, z;

    if (layerName == "LNDARE")
    {
        z = -500;
        depth.push_back(z);
    }
    else if ((layerName == "UWTROC")||(layerName == "WRECKS"))
    {
        z = feat->GetFieldAsDouble("VALSOU");
        if (!z)
        {
            //cout << layerName << " feature has no sounding" << endl;
            return;
        }
        depth.push_back(z*100);
    }
    else
    {
        cout << "Unknown point layer: " << layerName << endl;
        return;
    }

    poPoint = poPoint = ( OGRPoint * )geom;
    // Get the location in the grid coordinate system
    lon = poPoint->getX();
    lat = poPoint->getY();
    geod.LatLong2LocalUTM(lat, lon, x,y);

    // Store the XYZ of the vertices
    X.push_back(x);
    Y.push_back(y);

    // Place point into the layer
    //buildPoint(x, y, z*100);
}

// Store the polygon as shapefiles for rasterization and store the vertices for interpolation
void Grid_Interp::polygonFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRPolygon *UTM_poly;
    OGRGeometry *geom_UTM;
    OGRFeature *new_feat;

    double z;

    geom_UTM = geod.LatLong2UTM(geom);

    UTM_poly = ( OGRPolygon * )geom_UTM;

    // Build the layer for the area covered by the ENC
    if (layerName == "M_COVR")
    {
        if (feat->GetFieldAsDouble("CATCOV") == 1)
        {
            UTM_poly->segmentize(grid_size);
            new_feat =  OGRFeature::CreateFeature(feat_def_outline);
            new_feat->SetField("Inside_ENC", 1);
            new_feat->SetGeometry(UTM_poly);

            // Build the new feature
            if( layer_outline->CreateFeature( new_feat ) != OGRERR_NONE )
            {
                printf( "Failed to create feature in polygon shapefile.\n" );
                exit( 1 );
            }
        }
    }

    UTM_poly->Buffer(buffer_size);
    UTM_poly->segmentize(grid_size);

    if (layerName == "LNDARE")
    {
        z = -500;
        new_feat =  OGRFeature::CreateFeature(feat_def_poly);
        new_feat->SetField("Depth", z);
        new_feat->SetGeometry(UTM_poly);

        // Build the new feature
        if( layer_poly->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in polygon shapefile.\n" );
            exit( 1 );
        }
        // To limit the amount of points stored, only store the vertices of the
        //  buffered polygon.
        store_vertices(UTM_poly,z);
    }
    else if (layerName == "WRECKS")
    {
        z = feat->GetFieldAsDouble("VALSOU")*100;
        if (!z)
            return;
        new_feat =  OGRFeature::CreateFeature(feat_def_poly);
        new_feat->SetField("Depth", z);
        new_feat->SetGeometry(UTM_poly);

        // Build the new feature
        if( layer_poly->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in polygon shapefile.\n" );
            exit( 1 );
        }
    }
    else if (layerName == "DEPARE")
    {
        z = feat->GetFieldAsDouble("DRVAL1")*100; // Minimum depth in range
        if (!z)
            return;
        new_feat =  OGRFeature::CreateFeature(feat_def_depth);

        new_feat->SetField("Depth", z);
        new_feat->SetGeometry(UTM_poly);

        // Build the new feature
        if( layer_depth->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in depth shapefile.\n" );
            exit( 1 );
        }
    }
    else if ((layerName == "PONTON")||(layerName =="FLODOC"))
    {
        z = -500;
        new_feat =  OGRFeature::CreateFeature(feat_def_poly);
        new_feat->SetField("Depth", z);
        new_feat->SetGeometry(UTM_poly);

        // Build the new feature
        if( layer_poly->CreateFeature( new_feat ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in polygon shapefile.\n" );
            exit( 1 );
        }
    }
    else
    {
        if (layerName != "M_COVR")
        {
            cout << "Unknown Polygon layer: " << layerName << endl;
            return;
        }
    }
}

// This function stores the vertices of a polygon (that is in UTM) for interpolation
void Grid_Interp::store_vertices(OGRPolygon *UTM_Poly, double z)
{
    OGRLinearRing *ring;

    ring = UTM_Poly->getExteriorRing();

    for (int j = 0 ; j< ring->getNumPoints(); j++)
    {
        X.push_back(ring->getX(j)-geod.getXOrigin());// convert to local UTM
        Y.push_back(ring->getY(j)-geod.getYOrigin());// convert to local UTM
        depth.push_back(z);
    }

}

void Grid_Interp::getRasterData(string filename, int &nXSize, int &nYSize, vector<int>&RasterData)
{
    GDALDataset  *poDataset;
    GDALRasterBand *poRasterBand;
    string full_Filename = MOOS_Path+"src/ENCs/Grid/" +filename;

    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen( full_Filename.c_str(), GA_ReadOnly );

    poRasterBand = poDataset -> GetRasterBand(1);

    nXSize = poRasterBand->GetXSize(); // width
    nYSize = poRasterBand->GetYSize(); // height

    // Resize the vector to fit the raster dataset
    RasterData.resize(nXSize*nYSize);

    // Read the raster into a 1D row-major vector where it is organized in left to right,top to bottom pixel order
    if( poRasterBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, RasterData.data(), nXSize, nYSize, GDT_Int32, 0, 0 ) != CE_None )
    {
        printf( "Failed to access raster data.\n" );
        GDALClose(poDataset);
        exit( 1 );
    }

    GDALClose(poDataset);
}

// Cycle through the raster data and store the xyz coordinates to be used in the interpolation
void Grid_Interp::raster2XYZ(vector<int>& RasterData, int nXSize)
{
    double x,y;

    // Store the Polygon data before gridding
    for (int i=0; i<RasterData.size(); i++)
    {
        // Convert the coordinates from 1D to 2D then grid to local_UTM
        row_major_to_2D(i, x, y, nXSize);
        X.push_back(x);
        Y.push_back(y);
        depth.push_back(RasterData[i]);
    }
}

// Rasterize the shapefile using the commandline function gdal_rasterize
void Grid_Interp::rasterizeSHP(string outfilename, string infilename, string attribute)
{
    // Strings to hold the data for the input/output filenames for gdal_rasterize
    string full_inFilename = MOOS_Path+"src/ENCs/Grid/" +infilename;
    string full_outFilename = MOOS_Path+"src/ENCs/Grid/" +outfilename;
    string filenames = full_inFilename + " " + full_outFilename;

    // String to hold the data for the georeferenced extents for gdal_rasterize
    string georef_extent = "-te "+to_string(minX+geod.getXOrigin())+ " "+to_string(minY+geod.getYOrigin())+ " "
            +to_string(maxX+geod.getXOrigin())+ " "+to_string(maxY+geod.getYOrigin())+ " ";

    // String for all the options for gdal_rasterize
    string options = "-a_nodata -1000 -at -a " + attribute + " -tr " + to_string(grid_size)+ " " +
            to_string(grid_size)+ " -a_srs EPSG:2219 -ot Int32 " + georef_extent + filenames;

    string rasterize = "gdal_rasterize " + options;

    cout << "Rasterizing " << outfilename << endl;

    system(rasterize.c_str());
}

void Grid_Interp::xy2grid(double x, double y, int &gridX, int &gridY)
{
  // Converts the local UTM to the grid coordinates.
  gridX = int(round((x-minX-grid_size/2.0)/grid_size));
  gridY = int(round((y-minY-grid_size/2.0)/grid_size));
}

void Grid_Interp::grid2xy(int gridX, int gridY, double &x, double &y)
{
  // Converts the grid coordinates to the local UTM.
  x = int(round(gridX*grid_size+ minX+ grid_size/2.0));
  y = int(round(gridY*grid_size+ minY+ grid_size/2.0));
}

void Grid_Interp::LatLong2Grid(double lat, double lon, int &gridX, int &gridY)
{
    double x,y;
    geod.LatLong2LocalUTM(lat,lon, x,y);
    xy2grid(x,y, gridX, gridY);
}


void Grid_Interp::Grid2LatLong(int gridX, int gridY, double &lat, double &lon)
{
    double x,y;
    grid2xy(gridX, gridY, x,y);
    geod.LocalUTM2LatLong(x,y, lat,lon);
}

void Grid_Interp::row_major_to_2D(int index, double &x, double &y, int numCols)
{
    int gridX, gridY;
    gridY = index%numCols;
    gridX = (index-gridY)/numCols;
    grid2xy(gridX, gridY, x,y);
}

void Grid_Interp::row_major2grid(int index, int &gridX, int &gridY, int numCols)
{
    gridY = index%numCols;
    gridX = (index-gridY)/numCols;
}

void Grid_Interp::rasterIndex2gridIndex(int rasterIndex, int &gridIndex, int x_res, int y_res)
{
    int x,y;

    row_major2grid(rasterIndex, x,y, x_res);

    // Raster indexing is top to bottom and the grid indexing is bottom to top
    gridIndex = (y_res-1 - x)*x_res +y;
}

