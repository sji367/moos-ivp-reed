#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <fstream>
#include <vector>
//#include "gdal_frmts.h" // for GDAL/OGR
#include "ogrsf_frmts.h" // for gdal/ogr
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef K::Point_3   Point;

double dist(int x1, int y1, int x2, int y2);

using namespace std;
int main()
/*
{
  vector<vector<int> > Map(10, vector<int>(10, 0));
  std::ifstream in("terrain.cin");
  std::istream_iterator<Point> begin(in);
  std::istream_iterator<Point> end;
  Delaunay dt(begin, end);
  OGRPolygon *poly;
  OGRRing *ring;
  OGRPolygon *point;
  OGREnvelope *env;
  vector<int> x,y,z;
  vector<double> d;
  double Z;
  
  cout << dt.number_of_vertices() << endl;
  for( Delaunay::Finite_faces_iterator fi = dt.finite_faces_begin(); fi != dt.finite_faces_end(); fi++)
    {
      x.clear(); y.clear; z.clear();
      x = {fi->vertex(0)->point().hx(), fi->vertex(1)->point().hx(), fi->vertex(2)->point().hx()};
      y = {fi->vertex(0)->point().hy(), fi->vertex(1)->point().hy(), fi->vertex(2)->point().hy()};
      z = {fi->vertex(0)->point().hz(), fi->vertex(1)->point().hz(), fi->vertex(2)->point().hz()};
	
      // Build the polygon of the Delaunay triangle
      poly = (OGRPolygon*) OGRGeometryFactory::createGeometry(wkbPolygon);
      ring = (OGRLinearRing *) OGRGeometryFactory::createGeometry(wkbLinearRing);
      ring->addPoint(x[0],y[0]);
      ring->addPoint(x[1],y[1]);
      ring->addPoint(x[2],y[2]);
	
      ring->closeRings();
      poly->addRing(ring);
      poly->closeRings();
      poly->getEnvelope(env);
      // Now cycle though the points to get 
      for(int gridX=env->MinX(); gridX<= env->MaxX(); gridX++)
	{
	  for (int gridY=env->MinY(); gridY<= env->MaxY(); gridY++)
	    {
	      point = (OGRPoint*) OGRGeometryFactory::createGeometry(wkbPoint);
	      point->setX(gridX);
	      point->setY(gridY);
	      if (point->within(poly))
		{
		  // Calculate the distance to each vertex
		  d.push_back(dist(x[0],y[0], gridX,gridY));
		  d.push_back(dist(x[1],y[1], gridX,gridY));
		  d.push_back(dist(x[2],y[2], gridX,gridY));
		  Z = (1.0*z[0]/d[0]+ 1.0*z[1]/d[1] +1.0*z[2]/d[2])/(1.0/d[0]+1.0/d[1]+1.0/d[2]);
		  Map[y][x]= int(round(Z));
		}
	    }
	}
    }
  for (int i = 0; i<Map.size(); i++){
    for (int j=0; j<Map[0].size(); j++)
      {
	cout << Map[i][j] << ", ";
      }
    cout << endl;
  }
  /*
  for( Delaunay::Finite_faces_iterator fi = dt.finite_faces_begin(); fi != dt.finite_faces_end(); fi++)
    {
      cout<< fi->vertex(0)->point().hx() << "," << fi->vertex(0)->point().hy() << "," << fi->vertex(0)->point().hz()<<";" << endl;
      cout<< fi->vertex(1)->point().hx() << "," << fi->vertex(1)->point().hy() << "," << fi->vertex(1)->point().hz()<<";" << endl;
      cout<< fi->vertex(2)->point().hx() << "," << fi->vertex(2)->point().hy() << "," << fi->vertex(2)->point().hz()<<";" << endl;
    }
  
  return 0;
}

*/
{
    vector<vector<int> > Map(10, vector<int>(10, -15));
    OGRPolygon *poly;
    OGRLinearRing *ring;
    OGRPoint *point;
    vector<int> x,y,z;
    vector<double> d;
    int Z;

    vector<Point> XYZ;
    XYZ.push_back(Point(1, 2, 3));
    XYZ.push_back(Point(4, 5, 6));
    XYZ.push_back(Point(7, 8, 9));
    XYZ.push_back(Point(9, 6, 7));
    XYZ.push_back(Point(3, 5, 6));
    XYZ.push_back(Point(5, 6, 3));
    XYZ.push_back(Point(4, 3, 8));
    XYZ.push_back(Point(5, 5, 8));
    XYZ.push_back(Point(7, 4, 2));

    
    //std::ifstream in("/home/sji367/moos-ivp/moos-ivp-reed/src/lib_ENC_util/terrain.cin");
    //std::istream_iterator<Point> begin(in);
    //std::istream_iterator<Point> end;
    Delaunay dt;
    dt.insert(XYZ.begin(), XYZ.end());
    Delaunay::Finite_faces_iterator fi = dt.finite_faces_begin();
    cout << "#: " << dt.number_of_vertices() << endl;
      
    int minX, maxX, maxY, minY;

    for( Delaunay::Finite_faces_iterator fi = dt.finite_faces_begin(); fi != dt.finite_faces_end(); fi++)
      {
        x.clear(); y.clear(); z.clear();

        for(int i=0; i<3; i++)
        {
	  
            x.push_back(fi->vertex(i)->point().hx());
            y.push_back(fi->vertex(i)->point().hy());
            z.push_back(fi->vertex(i)->point().hz());
	    cout << x[i] << endl;
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
    for (unsigned i = 0; i<Map.size(); i++){
      for (unsigned j=0; j<Map[0].size(); j++)
        {
      cout << Map[i][j] << ",\t";
        }
      cout << endl;
    }
}

double dist(int x1, int y1, int x2, int y2)
{
    return (pow((x1-x2),2)+pow((y1-y2),2));
}
