# -*- coding: utf-8 -*-
"""
    This application takes the information from the ENC that has previously  
been sorted into shapefiles by object geometry type and prints it to 
pMarnineViewer.

@author: Sam Reed
"""
# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class MOOS_comms(object):
    """ This class id for MOOS communications. It has 2 parts:
          1. Initialize comms
              a. Set the Timewarp
              b. Register for variables
              c. Connect to the server
          2. Get new values.
    """
    
    def __init__(self):
        # Open a communication link to MOOS
        self.comms = pymoos.comms()

    def Register_Vars(self):
        """ This function registers for the updates of the VIEW_POINT and 
            VIEW_SEGLIST at the rate of which it is being outputed from MOOS.
        """
        self.comms.register('VIEW_POINT', 0)
        self.comms.register('VIEW_SEGLIST', 0)
        return True
        
    def Set_time_warp(self, timewarp):
        if timewarp>1:
            # Time Warp and Scaling factor constant
            time_warp = timewarp
            scaling_factor = 0.04*time_warp    
            
            # Set the timewarp and scale factor
            pymoos.set_moos_timewarp(time_warp)
            self.comms.set_comms_control_timewarp_scale_factor(scaling_factor)
        
    def Initialize(self):
        """ This function registers for the current X,Y, and heading and then
            connects to the server.
        """
        # Set timewarp        
        self.Set_time_warp(1)
        
        # Register for desired variables
        self.comms.set_on_connect_callback(self.Register_Vars)
        
        # Connect to the server
        self.comms.run('localhost',9000,'Print')
        
    def Get_mail(self):
        """ We don't need to get mail. """
        pass

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#       
class Print_ENC(object):
    """ This application takes the information from the ENC that has previously  
        been sorted into shapefiles by object geometry type and prints it to 
        pMarnineViewer.
    """
    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    def __init__(self, LatOrigin=43.071959194444446, 
                 LongOrigin=-70.711610833333339,
                 ENC_filename='../../src/ENCs/US5NH02M/US5NH02M.000', 
                 filename_pnt='../../src/ENCs/US5NH02M/Shape/point.shp', 
                 filename_poly='../../src/ENCs/US5NH02M/Shape/poly.shp', 
                 filename_line='../../src/ENCs/US5NH02M/Shape/line.shp'):
        """ Initialize varibles. 
            
        Inputs:
            LatOrigin - Latitude origin for the pyproj conversion to UTM
            LongOrigin - Longitude origin for the pyproj conversion to UTM
            ENC_filename - path to the ENC file
            filename_* - path to the shapefiles which are organized by geometry
                         type 
        """
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        self.ENC_filename = ENC_filename
        self.filename_pnt = filename_pnt
        self.filename_poly = filename_poly
        self.filename_line = filename_line
        self.print_area_filter = ogr.Geometry(ogr.wkbPolygon)
    
    def LonLat2MOOSxy(self, lon, lat):
        """ This function converts Longitude and Latitude to MOOS X and Y
        
        Inputs:
            lat - Latitude to be converted to UTM
            lon - Longitude to be converted to UTM
            
        Output:
            x - Corresponding X location in UTM
            y - Corresponding Y location in UTM
        """
        x,y = self.LonLat2UTM(lon, lat)
        x += -self.x_origin
        y += -self.y_origin
        return x,y  
            
    def MOOSxy2LonLat(self, x, y):
        """ This function converts MOOS X,Y to Longitude and Latitude
        
        Inputs:
            x - X location in UTM coordinates
            y - Y location in UTM coordinates
            
        Output:
            lat - Corresponding latitude location
            lon - Corresponding longitude location
        """
        lat,lon = self.LonLat2UTM(x+self.x_origin, y+self.y_origin, inverse=True)
        return lat,lon
        
    def define_print_area(self, N_lat = 43.07511878, E_long = -70.68395689, 
                          S_lat = 43.05780589, W_long = -70.72434189):
        """ Define the area that we want to print.
            
        Inputs:
            N_lat - Northern latitude boundary
            E_long - Eastern longitude boundary
            S_lat - Southern latitude boundary
            W_long - Western longitude boundary
        """
        # Start by creating the baseline for the search area polygon 
        ring = ogr.Geometry(ogr.wkbLinearRing)
        
        # Build a ring of the points fot the search area polygon
        ring.AddPoint(W_long, N_lat)
        ring.AddPoint(E_long, N_lat)
        ring.AddPoint(E_long, S_lat)
        ring.AddPoint(W_long, S_lat)
        ring.AddPoint(W_long, N_lat)
        self.print_area_filter.AddGeometry(ring) # Add the ring to the previously created polygon
        
    def print_points(self, comms):
        """ Print the points from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            comms - Communication link to MOOS
        """
        # Get the driver and open the point file
        driver = ogr.GetDriverByName('ESRI Shapefile')
        ds = driver.Open(self.filename_pnt, 0)
        
        # There is only one layer in each file and we are just opening it to see how
        #   features there are in the layer
        layer = ds.GetLayer()
        #print 'Num of Features in File: %d' %layer.GetFeatureCount()
        
        # Filter the layer to only include features with Threat level > 0
        layer.SetSpatialFilter(self.print_area_filter) 
        
        feature = layer.GetNextFeature()
        while feature:    
            geom = feature.GetGeometryRef()
            t_lvl = feature.GetField(0) # Get the Threat Level for that feature
            time.sleep(.1) # Don't make it faster or it wont print all of the points
    
            # Convert the MOOS x,y position to Lat/Long and store it
            new_x, new_y = self.LonLat2MOOSxy (geom.GetX(), geom.GetY())
            location = 'x='+str(new_x)+',y='+str(new_y)+','
            
            # Change the Color of the point based on the Threat Level
            if t_lvl == 5:
                color = 'vertex_color=black,'
            elif t_lvl == 4:
                color = 'vertex_color=red,'
            elif t_lvl == 3:
                color = 'vertex_color=darkorange,'
            elif t_lvl == 2:
                color = 'vertex_color=gold,'
            elif t_lvl == 1:
                color = 'vertex_color=greenyellow,'
            elif t_lvl == 0:
                color = 'vertex_color=green,'
            elif t_lvl == -1: # Landmark
                color = 'vertex_color=violet'
            elif t_lvl == -2: # LIGHTS
                color = 'vertex_color=cornflowerblue,'
            
            size =  'vertex_size=10,'   
            m = location+size+color
            
            # Print the point
            comms.notify('VIEW_POINT', m)
            feature = layer.GetNextFeature()
    
    def print_polygons(self, comms):
        """ Print the polygons from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            comms - Communication link to MOOS
        """
        # Get the driver and open the point file
        driver = ogr.GetDriverByName('ESRI Shapefile')
        ds = driver.Open(self.filename_poly, 0)
        
        # There is only one layer in each file and we are just opening it to see how
        #   features there are in the layer
        layer = ds.GetLayer()
        
        # Filter the layer to only include features with Threat level > 0
        layer.SetSpatialFilter(self.print_area_filter) 
        
        feature = layer.GetFeature(0)
        while feature:
            time.sleep(.01)
            geom = feature.GetGeometryRef() # Polygon from shapefile
            
            # Get the interesection of the polygon from the shapefile and the
            #   outline of tiff from pMarnineViewer
            intersection_poly = geom.Intersection(self.print_area_filter) 
            
            # Get the ring of that intersection polygon
            p_ring = intersection_poly.GetGeometryRef(0) 
            
            if p_ring:
                # Determine how many vertices there are in the polygon
                points = p_ring.GetPointCount()
                vertex = 'pts={' # String to hold the vertices
                # Cycle through the vertices and store them as a string
                for p1 in xrange(points):
                    lon, lat, z = p_ring.GetPoint(p1)
                    p_x,p_y = self.LonLat2MOOSxy (lon, lat)
                    vertex += str(p_x) + ','+ str(p_y)
                    if (p1!=points-1):
                        vertex += ':'
                t_lvl = feature.GetField(0) # Get threat level
                
                # Change the Color of the point based on the Threat Level
                if t_lvl == 5:
                    color = 'edge_color=black, vertex_color=black'
                if t_lvl == 4:
                    color = 'edge_color=red,vertex_color=red'
                elif t_lvl == 3:
                    color = 'edge_color=darkorange,vertex_color=darkorange'
                elif t_lvl == 2:
                    color = 'edge_color=gold,vertex_color=gold'
                elif t_lvl == 1:
                    color = 'edge_color=greenyellow,vertex_color=greenyellow'
                elif t_lvl == 0:
                    color = 'edge_color=green,vertex_color=green'
                elif t_lvl == -1: # Landmark
                    color = 'edge_color=violet,vertex_color=violet'
                elif t_lvl == -2: # LIGHTS
                    color = 'edge_color=cornflowerblue,vertex_color=cornflowerblue'
                  
                if points != 0:
                    comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color)
            feature = layer.GetNextFeature()
            
    def poly_line_intersect(self, poly, line):
        """ This function determines if each point along a line is within a 
            polygon and if it is it outputs that line. If the line intersects 
            the polygon multiple times, then the points between the two 
            intersections are also considered within the polygon so that it 
            doesn't connect two points that should not be connected.
            
        Inputs:
            poly - The area that we want to bound the line to
            line - The line that the function is working with
        
        Outputs:
            new_line - The line that is bounded within the desired polygon
        """
        # Create the new line
        new_line = ogr.Geometry(ogr.wkbLineString)
        outside = ogr.Geometry(ogr.wkbLineString)
        pnt = ogr.Geometry(ogr.wkbPoint)
        
        # Temp variables for if there are more than one intersection
        out = False
        first_inside = False
        
        # Cycle through the points along the line to determine if they fall within
        #   the polygon
        for i in range(line.GetPointCount()):
            x,y,z = line.GetPoint(i)
            pnt.AddPoint(x,y)
            # If the point is within the polygon save it
            if pnt.Within(poly):
                # Variable that states if you had the first point inside or not
                first_inside = True 
                # If the last point was inside the polygon, save it without other 
                #   processing            
                if out == False: 
                    new_line.AddPoint(x,y)
                # If the previous point was outside the polygon then add the rest 
                #   of the original line to the new line
                else: 
                    for ii in range(outside.GetPointCount()):
                        x,y,z = outside.GetPoint(ii)
                        new_line.AddPoint(x,y)
                    outside = ogr.Geometry(ogr.wkbLineString)
                    out = False
            # If the  point is not within the polygon then add to a line describing
            #   the line outside of the polygon in case it intersects more than 
            #   once
            else: 
                if out == False:
                    out = True
                if first_inside == True:
                    outside.AddPoint(x,y)
        return new_line    
        
    def print_lines(self, comms):
        """ Print the lines from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            comms - Communication link to MOOS
        """
        # Get the driver and open the point file
        driver = ogr.GetDriverByName('ESRI Shapefile')
        ds = driver.Open(self.filename_poly, 0)
    
        # There is only one layer in each file and we want to open it
        layer = ds.GetLayer()
        
        # Filter the line layer to only include features within the area in 
        #   pMarnineViewer
        layer.SetSpatialFilter(self.print_area_filter)
        feature = layer.GetNextFeature()
        while feature:
            time.sleep(.01)
            line = feature.GetGeometryRef() # line from shapefile
            # Get the interesection of the line from the shapefile and the
            #   outline of tiff from pMarnineViewer
            intersection_line = self.poly_line_intersect(self.print_area_filter, line)
    
            points = intersection_line.GetPointCount()
    
            vertex = 'pts={' # String to hold the vertices
            # Cycle through the vertices and store them as a string
            for p2 in xrange(points):
                lon, lat, z = intersection_line.GetPoint(p2)
                p_x,p_y = self.LonLat2MOOSxy (lon, lat)
                vertex += str(p_x) + ','+ str(p_y)
                if (p2!=points-1):
                    vertex += ':'
            t_lvl = feature.GetField(0) # Get threat level
            
            # Change the Color of the point based on the Threat Level
            if t_lvl == 5:
                color = 'edge_color=black, vertex_color=black'
            if t_lvl == 4:
                color = 'edge_color=red,vertex_color=red'
            elif t_lvl == 3:
                color = 'edge_color=darkorange,vertex_color=darkorange'
            elif t_lvl == 2:
                color = 'edge_color=gold,vertex_color=gold'
            elif t_lvl == 1:
                color = 'edge_color=greenyellow,vertex_color=greenyellow'
            elif t_lvl == 0:
                color = 'edge_color=green,vertex_color=green'
            elif t_lvl == -1: # Landmark
                color = 'edge_color=violet,vertex_color=violet'
            elif t_lvl == -2: # LIGHTS
                color = 'edge_color=cornflowerblue,vertex_color=cornflowerblue'
            if points != 0:
                comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color)
            feature = layer.GetNextFeature()
            
    def print_all(self):
        """ This function prints all of the objects that are within the desired
            area to be printed to the pMarnineViewer.        
        """
        MOOS = MOOS_comms()
        MOOS.Initialize()
        self.define_print_area()
        self.print_points(MOOS.comms)
        self.print_polygons(MOOS.comms)
#        self.print_lines(MOOS.comms)

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
e = Print_ENC()
e.print_all()