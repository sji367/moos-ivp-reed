#include "gridinterp.h"

Grid_Interp::Grid_Interp()
{
  geod = Geodesy();
  grid_size = 5;
  buffer_size = 5;
  MOOS_Path = "/home/sji367/moos-ivp/moos-ivp-reed";
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

void Grid_Interp::buildLayer()
{
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    string gridPath = MOOS_Path+"src/ENCs/Shape/grid/grid.shp";
    DS = poDriver->Create(gridPath.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( DS == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }
    // Create the layers (point, polygon, and lines)
    Layer = DS->CreateLayer( "Point", NULL, wkbPoint25D, NULL );
    if( Layer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }

    // Create the field
    OGRFieldDefn oField_depth( "Depth", OFTReal);
    if(Layer->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    feat_def = Layer->GetLayerDefn();

}

void Grid_Interp::buildPoint(double x,  double y, double depth)
{
    OGRPoint *newPoint;
    OGRFeature *newFeat;

    newPoint = (OGRPoint *)OGRGeometryFactory::createGeometry(wkbPoint25D);
    newPoint->setX(x);
    newPoint->setY(y);
    newPoint->setZ(depth);

    newFeat =  OGRFeature::CreateFeature(feat_def);
    newFeat->SetField("Depth", depth);
    newFeat->SetGeometry(newPoint);

    if( Layer->CreateFeature( newFeat ) != OGRERR_NONE )
    {
        printf( "Failed to create feature in shapefile.\n" );
        exit( 1 );
    }

    OGRFeature::DestroyFeature( newFeat );
}


void Grid_Interp::Run()
{
    GDALAllRegister();
    GDALDataset* ds;
    OGRSpatialReference UTM = geod.getUTM();

    buildLayer();

    ds = (GDALDataset*) GDALOpenEx( ENC_filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );

    getENC_MinMax(ds);
    OGRLayer *land = ds->GetLayerByName("LNDARE");

    // Add the vertices from the soundings, land areas and depth contours
    //  to the delaunay objects

    layer2XYZ(ds->GetLayerByName("SOUNDG"), "SOUNDG");
    layer2XYZ(ds->GetLayerByName("UWTROC"), "UWTROC");
    cout << "Point" << endl;
    layer2XYZ(ds->GetLayerByName("DEPCNT"), "DEPCNT");
    cout << "Line" << endl;
    layer2XYZ(land, "LNDARE");
    cout << "Poly" << endl;

    // Grid the data
    GDALGridLinearOptions options;
    options.dfRadius = 0;
    options.dfNoDataValue = -1500;

    int x_res = int(round((maxX - minX)/300));
    int y_res = int(round((maxY - minY)/300));

    Map.resize(y_res*x_res);

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
    vector<double> geoTransform = {minX, 300, 0, maxY, 0, 300};

    printf("%0.2f, %0.2f, %0.2f\n", X[0], Y[0], depth[0]);
    //if (GDALGridCreate(GGA_Linear, &options, static_cast<int>(X.size()), X.data(), Y.data(), depth.data(), minX, maxX, minY, maxY, x_res, y_res, GDT_Int32, Map.data(), NULL, NULL) == CE_Failure)
    if (GDALRasterizeLayersBuf(Map.data(), x_res, y_res, GDT_Int32, 4, 0, 1, (OGRLayerH*)&layers[0], spat_ref, geoTransform.data(), NULL, NULL, -500, opt, NULL, NULL ) == CE_Failure)
        cout << "Gridding Failed" << endl;
    else
        cout << "It WORKED!!!" << endl;

    for (int i=0; i<Map.size(); i++)
        cout << i << " " << Map[i] << endl;

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
}

void Grid_Interp::layer2XYZ(OGRLayer* layer, string layerName)
{
  OGRFeature* feat;
  OGRGeometry* geom;
  string geom_name;

  if (layer != NULL)
    {
      layer->ResetReading();
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
    cout << layer->GetName() << " is not in the ENC!" << endl;
}

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

        X.push_back(x);
        Y.push_back(y);

        // Get depth in cm
        z= poPoint->getZ()*100;
        depth.push_back(z);

        // Place point into the layer
        buildPoint(x,y, z);
      }
}

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

    // Get the location in the grid coordinate system
    for (int j=0; j<UTM_line->getNumPoints(); j++)
    {
        x = UTM_line->getX(j)-geod.getXOrigin();// convert to local UTM
        y = UTM_line->getY(j)-geod.getYOrigin();// convert to local UTM

        // Store the XYZ of the vertices
        X.push_back(x);
        Y.push_back(y);
        depth.push_back(z); // in cm

        // Place point into the layer
        buildPoint(x, y, z);
    }

}

void Grid_Interp::polygonFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRPolygon *UTM_poly;
    OGRLinearRing *ring;
    OGRGeometry *geom_UTM;

    double x,y,z;

    geom_UTM = geod.LatLong2UTM(geom);
    UTM_poly = ( OGRPolygon * )geom_UTM;

    UTM_poly->Buffer(buffer_size);
    UTM_poly->segmentize(grid_size);

    ring = UTM_poly->getExteriorRing();

    if (layerName == "LNDARE")
        z = -500;

    for (int j=0; j<ring->getNumPoints(); j++)
    {
        x = ring->getX(j)-geod.getXOrigin();// convert to local UTM
        y = ring->getY(j)-geod.getYOrigin();// convert to local UTM

        // Store the XYZ of the vertices
        X.push_back(x);
        Y.push_back(y);
        depth.push_back(z);

        // Place point into the layer
        buildPoint(x, y, z);
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
    else if (layerName == "UWTROC")
    {
        z = feat->GetFieldAsDouble("VALSOU");
        if (z == NULL)
            return;
        depth.push_back(z*100);
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
    buildPoint(x, y, z*100);
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
