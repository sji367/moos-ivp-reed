#include "geodesy.h"

void Geodesy::Initialise(double Lat_Origin, double Lon_Origin)
{
  LatOrigin =Lat_Origin;
  LonOrigin=Lon_Origin;
  UTM_Zone=getUTMZone(Lon_Origin);

  cout << UTM_Zone << endl;
  
  UTM.SetWellKnownGeogCS("WGS84");
  UTM.SetUTM(UTM_Zone, signbit(Lat_Origin));
  
  LatLong.SetWellKnownGeogCS("WGS84");
  cout << "set\n";
  LatLong2UTM(LatOrigin, LonOrigin, x_origin, y_origin);
  cout << "Origin: " << y_origin << ", " << x_origin << endl;
}

void Geodesy::LatLong2UTM(double Lat, double Lon, double &x, double &y)
{
  OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(&LatLong, &UTM);
  bool reprojected = coordTrans->Transform(1, &Lon, &Lat);
  x=Lon;
  y=Lat;
}

void Geodesy::UTM2LatLong(double x, double y, double &Lat, double &Lon)
{
  OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(&UTM, &LatLong);
  bool reprojected = coordTrans->Transform(1, &x, &y);
  Lon = x;
  Lat = y;
}
