# -*- coding: utf-8 -*-
"""
Created on Fri Apr 15 13:14:36 2016

@author: mapper
"""
# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

import csv

#==============================================================================
# Initialize Globals
#==============================================================================
comms = pymoos.comms()
### FIX: Generalize this to the utm zone of LatOrigin/LongOrigin ###
LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')

LatOrigin  = 43.071959194444446
LongOrigin = -70.711610833333339 
x_origin,y_origin = LonLat2UTM(LongOrigin, LatOrigin)

#==============================================================================
# Convert MOOS x,y to Longitude and Latitude
#==============================================================================
def MOOSxy2LonLat(x, y):
    lat,lon = LonLat2UTM(x+x_origin, y+y_origin, inverse=True)
    return lat,lon
    
#==============================================================================
# Convert Longitude and Latitude to MOOS x,y
#==============================================================================
def LonLat2MOOSxy(lon, lat):
    x,y = LonLat2UTM(lon, lat)
    x += -x_origin
    y += -y_origin
    return x,y 
    
#=============================================================================#
# This code determines if each point along a line is within a polygon and if it
#   is it outputs that line. If the line intersects the polygon multiple times,
#   then the points between the two intersections are also considered within 
#   the polygon so that it doesn't connect two points that should not be
#   connected.
#=============================================================================#
def poly_line_intersect(poly, line):
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
    
#=============================================================================#
# This program uses the information from the ENC Database and prints out the 
#   obstacles onto the pMarineViewer. The obstacles are printed as points with
#   the different colors relating to the threat level of the obstacle.
#=============================================================================#
def on_connect():
    comms.register('VIEW_POLYGON',0)
    comms.register('VIEW_POINT',0)
    return True

def main():
#    # Time Warp and Scaling factor constant
#    time_warp = 2
#    scaling_factor = 0.04*time_warp    
#    
#    # Set the timewarp and scale factor
#    pymoos.set_moos_timewarp(time_warp)
#    comms.set_comms_control_timewarp_scale_factor(scaling_factor)
    
    comms.set_on_connect_callback(on_connect);
    comms.run('localhost',9000,'print1')
    
    # Start by creating the baseline for the search area polygon 
    ring = ogr.Geometry(ogr.wkbLinearRing)
    poly_filter = ogr.Geometry(ogr.wkbPolygon)
    
#    # Landmarks
#    lon1, lat1 = MOOSxy2LonLat(5719,-9353)
#    lon2, lat2 = MOOSxy2LonLat(8807,-11539)    
    
#    # Poly obstacles 2
    lon1, lat1 = MOOSxy2LonLat(1606,-1184)
    lon2, lat2 = MOOSxy2LonLat(2076,-1462)
#    
#    lon1, lat1 = MOOSxy2LonLat(-4,-1138)
#    lon2, lat2 = MOOSxy2LonLat(116,-1204)

#    lon1, lat1 = MOOSxy2LonLat(1392,-713)
#    lon2, lat2 = MOOSxy2LonLat(1789,-1021)
    
#    lon1, lat1 = MOOSxy2LonLat(1021,-835)
#    lon2, lat2 = MOOSxy2LonLat(1291,-1119)
#    N_lat = lat1
#    E_long = lon1
#    S_lat = lat2
#    W_long = lon2
    
    N_lat = 43.07511878
    E_long = -70.68395689
    S_lat = 43.05780589
    W_long = -70.72434189
#    
    # Build a ring of the points fot the search area polygon
    ring.AddPoint(W_long, N_lat)
    ring.AddPoint(E_long, N_lat)
    ring.AddPoint(E_long, S_lat)
    ring.AddPoint(W_long, S_lat)
    ring.AddPoint(W_long, N_lat)
    poly_filter.AddGeometry(ring) # Add the ring to the previously created polygon
     
    #easily mark all of the output and input files
    file_pnt = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_pnt2.shp'
    file_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'
    file_line = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_line2.shp'
    
###############################################################################
##########                  Print the Point Obstacles                ##########
###############################################################################
    # Get the driver and open the point file
    driver = ogr.GetDriverByName('ESRI Shapefile')
    ds = driver.Open(file_pnt, 0)
    
    # There is only one layer in each file and we are just opening it to see how
    #   features there are in the layer
    layer = ds.GetLayer()
    #print 'Num of Features in File: %d' %layer.GetFeatureCount()
    
    # Filter the layer to only include features with Threat level > 0
    layer.SetSpatialFilter(poly_filter) 
#    layer.SetAttributeFilter("T_lvl != '0'")
#    layer.SetAttributeFilter("Type = LIGHTS")
    print '# of Features in Filter: %d' %layer.GetFeatureCount()
    
    feature = layer.GetNextFeature()
    cnt = 1
    while feature:    
        geom = feature.GetGeometryRef()
        t_lvl = feature.GetField(0) # Get the Threat Level for that feature
        time.sleep(.1) # Don't make it faster or it wont print all of the points

        # Convert the MOOS x,y position to Lat/Long and store it
        new_x, new_y = LonLat2MOOSxy (geom.GetX(), geom.GetY())
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
        cnt = 1+cnt
        
        # Print the point
        comms.notify('VIEW_POINT', m)
        feature = layer.GetNextFeature()
        
###############################################################################
##########               Print the Polygons Obstacles                ##########
###############################################################################
    ds = driver.Open(file_poly, 0)
    outfile_mp = file('poly2.csv', 'wb')
    writer_mp = csv.writer(outfile_mp, delimiter=',', quoting=csv.QUOTE_MINIMAL)
    # There is only one layer in each file and we want to open it
    layer = ds.GetLayer()
    
    # Filter the layer to only include features within the area in 
    #   pMarnineViewer
    layer.SetSpatialFilter(poly_filter)
#    print 'Num of Features in Filter: %d' %layer.GetFeatureCount()
    feature = layer.GetFeature(0)
    while feature:
        time.sleep(.01)
        geom = feature.GetGeometryRef() # Polygon from shapefile
        # Get the interesection of the polygon from the shapefile and the
        #   outline of tiff from pMarnineViewer
        intersection_poly = geom.Intersection(poly_filter) 
        
        # Get the ring of that intersection polygon
        p_ring = intersection_poly.GetGeometryRef(0) 
        
#        print feature.GetField(0)
        if p_ring:
            # Determine how many vertices there are in the polygon
            points = p_ring.GetPointCount()
            vertex = 'pts={' # String to hold the vertices
            # Cycle through the vertices and store them as a string
            for p1 in xrange(points):
                xy = []
                lon, lat, z = p_ring.GetPoint(p1)
                p_x,p_y = LonLat2MOOSxy (lon, lat)
                vertex += str(p_x) + ','+ str(p_y)
                xy.append(p_x)
                xy.append(p_y)
                writer_mp.writerow(xy)
                if (p1!=points-1):
                    vertex += ':'
            t_lvl = feature.GetField(0) # Get threat level
            writer_mp.writerow([0,0])
            # Change the Color of the point based on the Threat Level
            if t_lvl == 5:
                color = 'edge_color=black, vertex_color=black'
            if t_lvl == 4:
                color = 'edge_color=red,vertex_color=red'
            elif t_lvl == 3:
                color = 'edge_color=orange,vertex_color=darkorange'
            elif t_lvl == 2:
                color = 'edge_color=yellow,vertex_color=yellow'
            elif t_lvl == 1:
                color = 'edge_color=blue,vertex_color=yellowgreen'
            elif t_lvl == 0:
                color = 'edge_color=green,vertex_color=green'
              
            if points != 0:
                comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color)
        feature = layer.GetNextFeature()
    outfile_mp.close()
###############################################################################
##########                 Print the Line Obstacles                  ##########
###############################################################################
    ds = driver.Open(file_line, 0)
    
    # There is only one layer in each file and we want to open it
    layer = ds.GetLayer()
    
    # Filter the line layer to only include features within the area in 
    #   pMarnineViewer
    layer.SetSpatialFilter(poly_filter)
    feature = layer.GetNextFeature()
    while feature:
        time.sleep(.01)
        line = feature.GetGeometryRef() # line from shapefile
        # Get the interesection of the line from the shapefile and the
        #   outline of tiff from pMarnineViewer
#        intersection_line = line.Intersection(poly_filter)
        intersection_line = poly_line_intersect(poly_filter, line)

        points = intersection_line.GetPointCount()

        vertex = 'pts={' # String to hold the vertices
        # Cycle through the vertices and store them as a string
        for p2 in xrange(points):
            lon, lat, z = intersection_line.GetPoint(p2)
            p_x,p_y = LonLat2MOOSxy (lon, lat)
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
            color = 'edge_color=orange,vertex_color=darkorange'
        elif t_lvl == 2:
            color = 'edge_color=yellow,vertex_color=yellow'
        elif t_lvl == 1:
            color = 'edge_color=blue,vertex_color=yellowgreen'
        elif t_lvl == 0:
            color = 'edge_color=green,vertex_color=green'
#        if points != 0:
#            comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color)
#            print pymoos.time()-t
        feature = layer.GetNextFeature()
        
if __name__ == "__main__":
    main()