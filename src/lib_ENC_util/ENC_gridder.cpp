#include "gridENC.h"

ENC_grid::ENC_grid()
{
  geod = Geodesy();
  grid_size = 5;
  buffer_size = 5;
  ENC_filename = "/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs/US5NH02M/US5NH02M.000";
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

ENC_grid::ENC_grid(string ENC_Filename, double Grid_size, double buffer_dist, Geodesy Geod)
{
  initGeodesy(Geod);
  grid_size = Grid_size;
  ENC_filename = ENC_Filename;
  buffer_size = buffer_dist;
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

ENC_grid::ENC_grid(string ENC_Filename, double Grid_size, double buffer_dist, double lat, double lon)
{
  geod = Geodesy(lat, lon);
  grid_size = Grid_size;
  ENC_filename = ENC_Filename;
  buffer_size = buffer_dist;
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;
}

void ENC_grid::run()
{
    OGRGeometry *geom, *geom2;
    OGRGeometryCollection  *collection;
    OGRPolygon *poly;
    OGRLinearRing *ring;
    buildDelaunayPoly();
    cout << "Starting Delaunay Triangulation" << endl;
    geom = delaunayPoly->DelaunayTriangulation(-1, 0);
    cout << "End of Delaunay Triangulation" << endl;

    cout << geom->getGeometryName() << endl;
    collection = (OGRGeometryCollection *)geom;

    if (collection ==NULL)
        cout << "empty collection" << endl;
    else
    {
        cout << collection->getNumGeometries() << endl;
        for (int i = 0; i<collection->getNumGeometries(); i++)
        {
            geom2 = collection->getGeometryRef(i);

            if (geom2 ==NULL)
                cout << "empty geom" << endl;
            else
            {
                if (i%10000==0)
                    cout << i << endl;
                makeGrid(geom2);
            }
        }
    }


}

// Use inverse distance weighting (Power of 2)
void ENC_grid::makeGrid(OGRGeometry *geom)
{
    OGRPolygon *poly;
    OGRLinearRing *ring;
    OGRPoint *point;

    vector<int> x,y,z;
    vector<double> distance;
    int grid_X,grid_Y, Z;
    x.clear(); y.clear(); z.clear();

    poly = (OGRPolygon *) geom;
    ring = poly->getExteriorRing();
    for (int i = 0; i<ring->getNumPoints(); i++)
    {
        UTM2grid(ring->getX(i), ring->getY(i), grid_X,grid_Y);
        x.push_back(grid_X);
        y.push_back(grid_Y);
        z.push_back(ring->getZ(i));
        //cout << ring->getX(i)-geod.getXOrigin() << ", " << ring->getY(i)-geod.getYOrigin() << "," << ring->getZ(i)<< endl;
    }

    minX = *min_element(x.begin(), x.end());
    maxX = *max_element(x.begin(), x.end());

    minY = *min_element(y.begin(), y.end());
    maxY = *max_element(y.begin(), y.end());

    // Now cycle though the points to get
    for(int gridX=minX; gridX<= maxX; gridX++)
    {
        for (int gridY=minY; gridY<= maxY; gridY++)
        {
            point = (OGRPoint*) OGRGeometryFactory::createGeometry(wkbPoint);
            point->setX(gridX);
            point->setY(gridY);

            if (point->Intersects(poly))//||point->Touches(poly))
            {
                distance.clear();
                // Calculate the distance to each vertex
                distance.push_back(calc_dist_sq(x[0],y[0], gridX,gridY));
                distance.push_back(calc_dist_sq(x[1],y[1], gridX,gridY));
                distance.push_back(calc_dist_sq(x[2],y[2], gridX,gridY));
                if((distance[0]!=0) && (distance[1]!=0) && (distance[2]!=0))
                {
                    Z = int(round((1.0*z[0]/distance[0]+ 1.0*z[1]/distance[1] +1.0*z[2]/distance[2])/(1.0/distance[0]+1.0/distance[1]+1.0/distance[2])));
                    Map[gridY][gridX]= Z;
                }
            }
        }
    }
}

// Distance squared
double ENC_grid::calc_dist_sq(int x1, int y1, int x2, int y2)
{
    return pow(x1-x2, 2)+pow(y1-y2, 2);
}

// Get the minimum and maximum x/y values (in local UTM) of the ENC
void ENC_grid::getENC_MinMax(GDALDataset* ds)
{
  OGRLayer* layer;
  OGRFeature* feat;
  OGRLinearRing* ring;
  OGRGeometry* geom;
  OGRPolygon* coverage;

  vector<double> x,y;
  double UTM_x, UTM_y;
  int gridX, gridY;

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

  xy2grid(maxX, maxY, gridX, gridY);
  vector<vector<int> > MAP(gridY, std::vector<int>(gridX, -1500));
  Map = MAP;
}

void ENC_grid::buildDelaunayPoly()
{
    GDALAllRegister();
    GDALDataset* ds;
    OGRSpatialReference UTM = geod.getUTM();

    ds = (GDALDataset*) GDALOpenEx( ENC_filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );

    getENC_MinMax(ds);

    // Create the rings and polygons for GDAL Delaunay Triangulation
    delaunayRing = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
    delaunayPoly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);

    delaunayLand_ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
    delaunayLand_poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);

    // Set the Spatial Reference to UTM
    delaunayLand_ring->assignSpatialReference(&UTM);
    delaunayLand_poly->assignSpatialReference(&UTM);

    delaunayRing->assignSpatialReference(&UTM);
    delaunayPoly->assignSpatialReference(&UTM);


    // Add the vertices from the soundings, land areas and depth contours
    //  to the delaunay objects
    rasterizeLayer(ds->GetLayerByName("SOUNDG"), "SOUNDG");
    cout << "Point" << endl;
    rasterizeLayer(ds->GetLayerByName("DEPCNT"), "DEPCNT");
    cout << "Line" << endl;
    rasterizeLayer(ds->GetLayerByName("LNDARE"), "LNDARE");
    cout << "Poly" << endl;
    delaunayRing->closeRings();
    delaunayPoly->addRing(delaunayRing);
    delaunayPoly->closeRings();
}

void ENC_grid::rasterizeLayer(OGRLayer* layer, string layerName)
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
	    pointFeat(feat, geom);
	}
    }
  else
    cout << layer->GetName() << " is not in the ENC!" << endl;
}

void ENC_grid::multipointFeat(OGRFeature* feat, OGRGeometry* geom)
{
    OGRGeometry* poPointGeometry;
    OGRMultiPoint *poMultipoint;
    OGRPoint * poPoint;

    double lat,lon, x, y, z;
    int gridX, gridY, depth;
    
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
        geod.LatLong2LocalUTM(lon,lat, x,y);
        xy2grid(x, y, gridX, gridY);

	// Get depth as int in cm
	z = poPoint->getZ();
	depth = int(round(z*100));

        delaunayRing->addPoint(x,y,depth);
      }
}

void ENC_grid::lineFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRLineString *UTM_line;
    OGRGeometry *geom_UTM;

    double x, y, z;
    int gridX, gridY, depth;
    vector<int> xyz;

    geom_UTM = geod.LatLong2UTM(geom);
    UTM_line = ( OGRLineString * )geom_UTM;

    UTM_line->segmentize(grid_size/2);

    if (layerName=="DEPCNT")
        z = feat->GetFieldAsDouble("VALDCO");

    // Get the location in the grid coordinate system
    for (int j=0; j<UTM_line->getNumPoints(); j++)
    {
        // Convert to grid
        x = UTM_line->getX(j);
        y = UTM_line->getY(j);
        xy2grid(x,y,gridX, gridY);

        // Get depth as int in cm
        depth = int(round(z*100));
        delaunayRing->addPoint(x,y,depth);
    }
       
}

void ENC_grid::polygonFeat(OGRFeature* feat, OGRGeometry* geom, string layerName)
{
    OGRPolygon *UTM_poly;
    OGRLinearRing *ring;
    OGRGeometry *geom_UTM;

    double x,y;
    int gridX, gridY, depth;

    geom_UTM = geod.LatLong2UTM(geom);
    UTM_poly = ( OGRPolygon * )geom_UTM;

    UTM_poly->Buffer(buffer_size);
    UTM_poly->segmentize(grid_size/2);

    ring = UTM_poly->getExteriorRing();

    if (layerName == "LNDARE")
        depth = -500;
  
    for (int j=0; j<ring->getNumPoints(); j++)
    {
        x = ring->getX(j);
        y = ring->getY(j);

        xy2grid(x, y, gridX, gridY);

        delaunayRing->addPoint(x,y,depth);
    }
}

void ENC_grid::pointFeat(OGRFeature* feat, OGRGeometry* geom)
{
    OGRPoint * poPoint;
    double lat,lon, x,y, z;
    int gridX, gridY, depth;

    poPoint = poPoint = ( OGRPoint * )geom;
    // Get the location in the grid coordinate system
    lon = poPoint->getX();
    lat = poPoint->getY();
    geod.LatLong2LocalUTM(lon,lat, x,y);
    xy2grid(x, y, gridX, gridY);

    // Get depth as int in cm
    z = poPoint->getZ();
    depth = int(round(z*100));
    delaunayRing->addPoint(x,y,depth);
}

void ENC_grid::xy2grid(double x, double y, int &gridX, int &gridY)
{
  // Converts the local UTM to the grid coordinates.
  gridX = int(round((x-minX-grid_size/2.0)/grid_size));
  gridY = int(round((y-minY-grid_size/2.0)/grid_size));
}

void ENC_grid::grid2xy(int gridX, int gridY, double &x, double &y)
{
  // Converts the grid coordinates to the local UTM.
  x = int(round(gridX*grid_size+ minX+ grid_size/2.0));
  y = int(round(gridY*grid_size+ minY+ grid_size/2.0));
}

void ENC_grid::LatLong2Grid(double lat, double lon, int &gridX, int &gridY)
{
    double x,y;
    geod.LatLong2LocalUTM(lat,lon, x,y);
    xy2grid(x,y, gridX, gridY);
}


void ENC_grid::Grid2LatLong(int gridX, int gridY, double &lat, double &lon)
{
    double x,y;
    grid2xy(gridX, gridY, x,y);
    geod.LocalUTM2LatLong(x,y, lat,lon);
}
