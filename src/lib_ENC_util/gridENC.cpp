/*
 * gridENC.cpp
 *
 *  Created on: April 21, 2017
 *      Author: Sam Reed
 */

#include "gridENC.h"

Grid_ENC::Grid_ENC()
{
    geod = Geodesy();
    minX = 0;
    minY = 0;
    maxX = 0;
    maxY = 0;
    grid_size = 5;
    buffer_size = 5;
    ENC_filename = "/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs/US5NH02M/US5NH02M.000";
}

Grid_ENC::Grid_ENC(string ENC_Filename, int Grid_size, double buffer_dist, Geodesy Geod)
{
    initGeodesy(Geod);
    minX = 0;
    minY = 0;
    maxX = 0;
    maxY = 0;
    grid_size = Grid_size;
    ENC_filename = ENC_Filename;
    buffer_size = buffer_dist;
}


Grid_ENC::~Grid_ENC()
{
    // Delete the allocation of the pointer to the vector
    for (OGRLayer* obj : desired_layers)
        delete obj;
    desired_layers.clear();
}

void Grid_ENC::MakeBaseMap(int segment_size)
{
    GDALAllRegister();

    GDALDataset* ds;
    OGRLayer* layer;
    //OGREnvelope* ENC_envelope;

    cout << ENC_filename << endl;

    ds = (GDALDataset*) GDALOpenEx( ENC_filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    cout << "ds" << endl;
    layer = ds->GetLayerByName("M_COVR");
    cout << "layer" << endl;
    //cout << layer->GetExtent(ENC_envelope) << endl; // get the min/max lat and long

    //cout << "Envelope" << endl;
    EnvelopeInUTM(layer);//, minX, minY, maxX, maxY);

    cout << "Envelope" << endl;
    rasterizeMultiPoints(ds->GetLayerByName("SOUNDG"));
    cout << "Point" << endl;
    rasterizePoly(ds->GetLayerByName("LNDARE"), segment_size);
    cout << "Poly" << endl;
    rasterizeLine(ds->GetLayerByName("DEPCNT"), segment_size);
    cout << "Line" << endl;
}

void Grid_ENC::EnvelopeInUTM(OGRLayer* layer)//, double &x_min, double &y_min, double &x_max, double &y_max)
{
	OGRFeature* feat;
	OGRLinearRing* ring;
	OGRGeometry* geom;
	OGRPolygon* coverage;

	vector<double> x,y;
	double UTM_x, UTM_y;

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

void Grid_ENC::EnvelopeInUTM(OGREnvelope* env, double &x_min, double &y_min, double &x_max, double &y_max)
{
    // Initialize x and y vectors to be 4x1 with 0s
    vector<double> x(4,0);
    vector<double> y(4,0);

    // Store the min and max x and y
    geod.LatLong2LocalUTM(env->MinY,env->MinX, x[0], y[0]);
    geod.LatLong2LocalUTM(env->MinY,env->MaxX, x[1], y[1]);
    geod.LatLong2LocalUTM(env->MaxY,env->MinX, x[2], y[2]);
    geod.LatLong2LocalUTM(env->MaxY,env->MaxX, x[3], y[3]);

    // Find the min and max of the X and Y values
    x_min = *min_element(x.begin(), x.end());
    y_min = *min_element(y.begin(), y.end());
    x_max = *max_element(x.begin(), x.end());
    y_max = *max_element(y.begin(), y.end());
}

void Grid_ENC::rasterizeMultiPoints(OGRLayer *Layer)
{
    OGRFeature* feat;
    OGRGeometry* geom, *poPointGeometry;
    OGRMultiPoint *poMultipoint;
    OGRPoint * poPoint;

    double lat,lon, z;
    //vector<int> xyz;
    int gridX, gridY, depth;

    Layer->ResetReading();
    if (Layer != NULL)
    {
        // Cycle through all points in the multipoint and store the xyz location of each point
        while( (feat = Layer->GetNextFeature()) != NULL )
        {
            geom = feat->GetGeometryRef();
            poMultipoint = ( OGRMultiPoint * )geom;
            for(int iPnt = 0; iPnt < poMultipoint->getNumGeometries(); iPnt++ )
            {
                //xyz.clear();
                // Make the point
                poPointGeometry = poMultipoint ->getGeometryRef(iPnt);
                poPoint = ( OGRPoint * )poPointGeometry;
                // Get the location in the grid coordinate system
                lon = poPoint->getX();
                lat = poPoint->getY();
                LatLong2Grid(lat,lon,gridX, gridY);

                // Get depth as int in cm
                z = poPoint->getZ();
                depth = int(round(z*100));

                data.push_back({gridX, gridY, depth});
            }
        }
    }
    else
        cout << Layer->GetName() << " is not in the ENC!" << endl;
}

void Grid_ENC::rasterizePoints(OGRLayer *Layer)
{
    OGRFeature* feat;
    OGRPoint * poPoint;
    OGRGeometry *geom;

    double lat,lon, z;
    int gridX, gridY, depth;
    //vector<int> xyz;

    Layer->ResetReading();
    if (Layer != NULL)
    {
        // Cycle though all points in the layer and store all xyz values
        while( (feat = Layer->GetNextFeature()) != NULL )
        {
            geom = feat->GetGeometryRef();
            poPoint = poPoint = ( OGRPoint * )geom;
            // Get the location in the grid coordinate system
            lon = poPoint->getX();
            lat = poPoint->getY();
            LatLong2Grid(lat,lon,gridX, gridY);

            // Get depth as int in cm
            z = poPoint->getZ();
            depth = int(round(z*100));

            //xyz.insert(xyz.end(), { gridX, gridY, depth });
            data.push_back({gridX, gridY, depth});
        }
    }
    else
        cout << Layer->GetName() << " is not in the ENC!" << endl;
}

void Grid_ENC::rasterizeLine(OGRLayer *Layer, double segment_size)
{
    OGRFeature* feat;
    OGRGeometry *geom;
    OGRLineString *poLine;
    string layerName;

    double lat,lon, z;
    int gridX, gridY, depth;
    vector<int> xyz;

    layerName = Layer->GetName();

    Layer->ResetReading();
    if (Layer != NULL)
    {
        // Cycle though all lines in the layer and store all xyz locations and values
        //  for each vertex in the lines
        while( (feat = Layer->GetNextFeature()) != NULL )
        {
            feat->GetGeometryRef();
            poLine = ( OGRLineString * )geom;
            SegmentLine_LatLon(poLine, segment_size);

            if (layerName=="DEPCNT")
              z = feat->GetFieldAsDouble("VALDCO");

            // Get the location in the grid coordinate system
            for (int j=0; j<poLine->getNumPoints(); j++)
            {
                //xyz.clear();
                // Convert to grid
                lon = poLine->getX(j);
                lat = poLine->getY(j);
                LatLong2Grid(lat,lon,gridX, gridY);

                // Get depth as int in cm
                //z = poLine->getZ(j);
                depth = int(round(z*100));

                data.push_back({gridX, gridY, depth});
            }
        }
    }
    else
        cout << layerName << " is not in the ENC!" << endl;
}

void Grid_ENC::rasterizePoly(OGRLayer * Layer, double segment_size)
{
    OGRFeature* feat;
    OGRGeometry *geom;
    OGRPoint *poPoint;
    OGRPolygon *poPoly;
    OGRLinearRing *ring;
    //OGREnvelope *env;

    string layerName;
    double lat,lon, z;
    int gridX, gridY, depth;
    vector<int> x,y, xyz;

    //double xmin, ymin, xmax, ymax;
    int gridXmin, gridYmin, gridXmax, gridYmax;
    int cntr = 0;
    string geom_name;

    layerName = Layer->GetName();
    Layer->ResetReading();
    if (Layer != NULL)
    {
        // Cycle though all polygons in the layer and store all xyz locations and values
        //  for each vertex in the polygon
        while( (feat = Layer->GetNextFeature()) != NULL )
        {
            cout << cntr << " out of " << Layer->GetFeatureCount() << endl;
            cntr++;
            geom = feat->GetGeometryRef();
			
            if (Layer->GetName()=="LNDARE")
            	z = -5; // Set land depth to 5 meters above the surface
            else
                z = feat->GetFieldAsDouble("VALSOU"); // otherwise use the sounding value

            geom_name = geom->getGeometryName();
            if (geom_name == "POINT")
            {
                poPoint = ( OGRPoint * )geom;
                lon = poPoint->getX();
                lat = poPoint->getY();

                LatLong2Grid(lat,lon,gridX, gridY);

                // Get depth as int in cm
                depth = int(round(z*100));

                data.push_back({gridX, gridY, depth});
            }
            else if (geom_name == "POLYGON")
            {
                cout << "inside" << endl;
                poPoly = ( OGRPolygon * )geom;

                poPoly = BufferPoly_LatLon(poPoly, buffer_size);
                cout << "inside2" << endl;
                poPoly = SegmentPoly_LatLon(poPoly,segment_size);
                cout << "inside3" << endl;
                ring = poPoly->getExteriorRing();

                cout << ring->getNumPoints() << endl;

                // Store the exterior ring
                for (int j=0; j<ring->getNumPoints(); j++)
                {
                    lon = ring->getX(j);
                    lat = ring->getY(j);

                    LatLong2Grid(lat,lon,gridX, gridY);

                    // Get depth as int in cm
                    depth = int(round(z*100));

                    data.push_back({gridX, gridY, depth});
                    x.push_back(gridX);
                    y.push_back(gridY);
                }
                /*
                // Get the envelope of the polygon
                gridXmin = *min_element(x.begin(), x.end());
                gridYmin = *min_element(y.begin(), y.end());
                gridXmax = *max_element(x.begin(), x.end());
                gridYmax = *max_element(y.begin(), y.end());
                cout << "X: " <<gridXmin << ", " << gridXmax<< "\t Y: " << gridYmin << ", " << gridYmax << endl;
                // Now we need all of the points inside of the polygon
                for (int y = gridYmin; y<gridYmax; y++)
                {
                    for (int x = gridXmin; x<gridXmax; x++)
                    {
                        OGRPoint *point;
                        Grid2LatLong(x,y, lat,lon);
                        point = (OGRPoint *)OGRGeometryFactory::createGeometry(wkbPoint25D);
                        point->setX(lon);
                        point->setY(lat);
                        if (point->Within(poPoly))
                        {
                            data.push_back(Point(gridX, gridY, depth));
                        }
                        OGRPoint::D
                    }
                }
                */
            }
        }
    }
    else
        cout << layerName << " is not in the ENC!" << endl;
}

void Grid_ENC::xy2grid(double x, double y, int &gridX, int &gridY)
{
    // Converts the local UTM to the grid coordinates.
    gridX = int(round((x-minX-grid_size/2.0)/grid_size));
    gridY = int(round((y-minY-grid_size/2.0)/grid_size));
}

void Grid_ENC::grid2xy(int gridX, int gridY, double &x, double &y)
{
        // Converts the grid coordinates to the local UTM.
        x = int(round(gridX*grid_size+ minX+ grid_size/2.0));
        y = int(round(gridY*grid_size+ minY+ grid_size/2.0));
}

void Grid_ENC::LatLong2Grid(double lat, double lon, int &gridX, int &gridY)
{
    double x,y;
    geod.LatLong2LocalUTM(lat,lon, x,y);
    xy2grid(x,y, gridX, gridY);
}


void Grid_ENC::Grid2LatLong(int gridX, int gridY, double &lat, double &lon)
{
    double x,y;
    grid2xy(gridX, gridY, x,y);
    geod.LocalUTM2LatLong(x,y, lat,lon);
}

// This functions allows the user to chose a minimum segment distance in meters
//  when the polygon vertices are in lat/long
OGRPolygon* Grid_ENC::SegmentPoly_LatLon(OGRPolygon* poly, double segment_dist)
{
    double x,y, newLat, newLon, diffLat, diffLon, lat, lon;
    OGRPoint *poPoint;

    // Get the centroid of the Polygon
    poly->Centroid(poPoint);
    lon = poPoint->getX();
    lat = poPoint->getY();

    // Find the difference between the lat/long and segment+lat/long
    geod.LatLong2LocalUTM(lat, lon, x, y);
    geod.LocalUTM2LatLong(x+segment_dist, y+segment_dist, newLat, newLon);
    diffLat = abs(newLat - lat);
    diffLon = abs(newLon - lon);

    // Choose the minimum segment the polygon as the minimum of either the difference
    //  between the lat/long and segment+lat/long
    if (diffLat < diffLon)
        poly->segmentize(diffLat);
    else
        poly->segmentize(diffLon);
}

// This functions allows the user to chose a minimum segment distance in meters
//  when the line vertices are in lat/long
OGRLineString* Grid_ENC::SegmentLine_LatLon(OGRLineString* line, double segment_dist)
{
    double x,y, newLat, newLon, diffLat, diffLon, lat, lon;
    OGRPoint *poPoint;

    // Get the centroid of the Line
    line->Centroid(poPoint);
    lon = poPoint->getX();
    lat = poPoint->getY();

    // Find the difference between the lat/long and segment+lat/long
    geod.LatLong2LocalUTM(lat, lon, x, y);
    geod.LocalUTM2LatLong(x+segment_dist, y+segment_dist, newLat, newLon);
    diffLat = abs(newLat - lat);
    diffLon = abs(newLon - lon);

    // Choose the minimum segment the line as the minimum of either the difference
    //  between the lat/long and segment+lat/long
    if (diffLat < diffLon)
        line->segmentize(diffLat);
    else
        line->segmentize(diffLon);
}


// This functions allows the user to chose a buffer distance in meters when the
//  polygon vertices are in lat/long
OGRPolygon* Grid_ENC::BufferPoly_LatLon(OGRPolygon* poly, double buffer_dist)
{
    double x,y, newLat, newLon, diffLat, diffLon, lat, lon;
    OGRPoint *poPoint;

    // Get the centroid of the Polygon
    poly->Centroid(poPoint);
    lon = poPoint->getX();
    lat = poPoint->getY();

    // Find the difference between the lat/long and buffer_dist+lat/long
    geod.LatLong2LocalUTM(lat, lon, x, y);
    geod.LocalUTM2LatLong(x+buffer_dist, y+buffer_dist, newLat, newLon);
    diffLat = abs(newLat - lat);
    diffLon = abs(newLon - lon);

    // Choose the buffer distance of the polygon as the maximum of either the difference
    //  between the lat/long and segment+lat/long
    if (diffLat > diffLon)
        poly->Buffer(diffLat);
    else
        poly->Buffer(diffLon);
}

// Using this format:
//  http://stackoverflow.com/questions/19850354/cgal-retrieve-vertex-index-after-delaunay-triangulation
void Grid_ENC::makeMap()
{
/*
  OGRPolygon *poly;
  OGRLinearRing *ring;
  OGRPoint *point;
  OGREnvelope *env;
  vector<int> x,y,z;
  vector<double> d;
  double Z;

  int minX, maxX, minY, maxY;
  
  Delaunay dt(data.begin(), data.end());

  for( Delaunay::Finite_faces_iterator fi = dt.finite_faces_begin(); fi != dt.finite_faces_end(); fi++)
    {
      x.clear(); y.clear(); z.clear();

      for(int i=0; i<3; i++)
	{
	  x.push_back(fi->vertex(i)->point().hx());
	  y.push_back(fi->vertex(i)->point().hy());
	  z.push_back(fi->vertex(i)->point().hz());

	  Map[y[i]][x[i]] = z[i]*10;
	}
      // Build the polygon of the Delaunay triangle
      poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
      ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
      ring->addPoint(x[0],y[0]);
      ring->addPoint(x[1],y[1]);
      ring->addPoint(x[2],y[2]);
      ring->addPoint(x[0],y[0]);

      ring->closeRings();
      poly->addRing(ring);
      poly->closeRings();

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
		  d.clear();
		  // Calculate the distance to each vertex
		  d.push_back(dist(x[0],y[0], gridX,gridY));
		  d.push_back(dist(x[1],y[1], gridX,gridY));
		  d.push_back(dist(x[2],y[2], gridX,gridY));
		  if((d[0]!=0) && (d[1]!=0) && (d[2]!=0))
		    {
		      Z = int(round((1.0*z[0]/d[0]+ 1.0*z[1]/d[1] +1.0*z[2]/d[2])/(1.0/d[0]+1.0/d[1]+1.0/d[2])*10));
		      Map[gridY][gridX]= Z;
		    }
		}
	    }
	}
    }
*/
}

double Grid_ENC::dist(int x1, int y1, int x2, int y2)
{
  return (pow((x1-x2),2)+pow((y1-y2),2));
}

