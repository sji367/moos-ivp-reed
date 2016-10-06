# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 12:51:02 2016

@author: mapper
"""

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# Numpy is a useful tool for mathematical operations 
import numpy as np

comms = pymoos.comms()

#==============================================================================
## Initialize some global variables
#==============================================================================
# Calculate the origin 
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

#==============================================================================
## Register for updates of the MOOS variables NAV_X and NAV_Y once every second
#==============================================================================
def on_connect():
    comms.register('NAV_X', 0)
    comms.register('NAV_Y', 0)
    comms.register('NAV_HEADING', 0)
    return True
    
#==============================================================================
# To convert the calculated angle to the one that relates to the one that MOOS
#   outputs, you have to use the formula: MOOS_ang = -(calc_ang-90)
#==============================================================================
def ang4MOOS(angle):
    return -(angle-90)

#==============================================================================
# This calculates the intersection of a polygon and the search polygon and 
#   outputs all vetices that are within the search area as well as the ones 
#   that conect two different segments of a concave polygon that are within the
#   search area.
#==============================================================================
def poly_intersect(search_poly, poly):
    # Initialize the necessary geometries
    pnt = ogr.Geometry(ogr.wkbPoint)
    outside = ogr.Geometry(ogr.wkbLineString)
    ring = ogr.Geometry(ogr.wkbLinearRing)
    
    ring = poly.GetGeometryRef(0)    
    
    # Temp variables for if there are more than one intersection
    out = False
    first_inside = False
    
    # Get the threat level and type of the obstacle and store it
    poly_info = str(poly.GetField(0))+','+ str(poly.GetField(1))
    
    # Initialize strings and counter
    obs_poly_str = ''
    poly_str = ''
    num_pnts = 0

    # Cycle through the points along the line to determine if they fall within
    #   the polygon
    for i in range(ring.GetPointCount()):
        x,y,z = ring.GetPoint(i)
        pnt.AddPoint(x,y)
        # If the point is within the polygon save it
        if pnt.Within(search_poly):
            # Variable that states if you had the first point inside or not
            first_inside = True 
            # If the last point was inside the polygon, save it without other 
            #   processing            
            if out == False: 
                poly_str+= str(x)+','+str(y)+poly_info
                num_pnts +=1
                
            # If the previous point was outside the polygon then add the info  
            #   on the original line to the string
            else: 
                for ii in range(outside.GetPointCount()):
                    x,y,z = outside.GetPoint(ii)
                    poly_str+= str(x)+','+str(y)+poly_info
                    num_pnts +=1
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
    
    # Add in the number of vertices that are in the search area
    obs_poly_str = str(num_pnts)+':'+poly_str
    return obs_poly_str
    
#==============================================================================
# Function to determine what the important information should be outputed for
#   each polygon in the search area. This function only works if there are 
#   vertices of the polygon in the search area.
#==============================================================================
def polygon(ASV_X, ASV_Y, heading, feature, intersect, TF_intersect, cntr):
    if (TF_intersect):
        geom = intersect.Buffer(0.00003)
    else:
        geom = feature.GetGeometryRef()
        geom = geom.Buffer(0.00003)
    
    # Initialize variables
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    x_small = 9999
    y_small = 9999
    
    min_angle = 360
    max_angle = -360
    min_dist = 9999 
#    min_dist_angle = 999
    
    d_min = 0
    d_max = 0
    min_dist = 9999
    
    ring = geom.GetGeometryRef(0)
    num_points =  ring.GetPointCount()
    flag = True
    # Cycle through the to find the matching x and y coordinates
    for i in range (0, num_points):
        lon = ring.GetX(i)
        lat = ring.GetY(i)
        ptx, pty = LonLat2MOOSxy(lon, lat)

        d = np.sqrt(np.square(ASV_X-ptx)+np.square(ASV_Y-pty))
        angle = np.arctan2(ASV_Y-pty, ASV_X-ptx)*180/np.pi      
        
        # There is one major issue with finding the minimum and maximum angles.
        #   It is that when the angle switches from 360 to 0. This will lead to
        #   incorrect angles being saved as the minimum and maximum. Therefore
        #   when the angle is in the right half plane we will use -180 to 180 
        #   and when the angle is in the left half plane, we will use 0 to 360.
        
        # Check to see if the first point is in the right or left hand plane.
        #   From this, it will set if the angle will range from 0 to 360 or 
        #   -180 to 180.
        if i==0:
            # Case 1: Right half plane --> flag = false
            if (-90 <= angle < 90):
                flag = False
            # Case 2: Left half plane --> flag = true
            else:
                flag = True
                
        # If it is Case 2, then have it go from 0 to 360. Otherwise the angle
        #   will range from -180 to 180
        if flag:
            angle = np.mod(angle, 360)
            
        # Determine if the angle is the minimum angle
        if angle < min_angle:
            min_angle = angle
            d_min_ang = d
            x1 = ptx
            y1 = pty
        
        # Determine if the angle is the maximum angle
        if angle > max_angle:
            max_angle = angle
            d_max_ang = d
            x2 = ptx
            y2 = pty
        
        # Check if this point is the points match the envelope and to find  
        #   the x and y coordinates is the shortest distance
        if min_dist > d:
            min_dist = d
#            print "distance: %f" %d
#            min_dist_angle = angle
            x_small = ptx
            y_small = pty
        
    # Post these points to pMarineViewer
    pt1 = 'x='+str(x1)+',y='+str(y1)+',vertex_size=6.5,vertex_color=white,active=true,label=pt1_'+str(cntr)
    comms.notify('VIEW_POINT', pt1)
    time.sleep(.001)
    pt2 = 'x='+str(x2)+',y='+str(y2)+',vertex_size=6.5,vertex_color=white,active=true,label=pt2_'+str(cntr)
    comms.notify('VIEW_POINT', pt2)
    pt3 = 'x='+str(x_small)+',y='+str(y_small)+',vertex_size=6.5,vertex_color=mediumblue,active=true,label=pt3_'+str(cntr)
    comms.notify('VIEW_POINT', pt3)
    
    # Find other useful info on the obstacle
    t_lvl = feature.GetField(0)
    obs_type = feature.GetField(1)
    
    if num_points>0:
    # t_lvl,type @ min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y @ min_ang_dist,max_ang_dist,min_dist
        poly = str(t_lvl)+','+str(obs_type)+'@'+str(x1)+','+str(y1)+','+str(x_small)+','+str(y_small)+','+str(x2)+','+str(y2)+'@'+str(d_min_ang)+','+str(d_max_ang)+','+str(min_dist)
#        +str(np.mod(ang4MOOS(max_angle), 360))+','+str(np.mod(ang4MOOS(min_angle), 360))+','+str(np.mod(ang4MOOS(min_dist_angle), 360))
    else:
        poly = "No points"    
    
    return poly
            
#==============================================================================
# This program uses the X and Y cooridinates from the ASV and filters out all 
#   of the points from the ENC database that are in a predetermined search
#   radius. It then outputs information needed for obstacle avoidance to the
#   MOOSDB as a string.
#==============================================================================
def main():
#    # Time Warp and Scaling factor constant
#    time_warp = 2
#    scaling_factor = 0.04*time_warp
#    
#    # Set the timewarp and scale factor
#    pymoos.set_moos_timewarp(time_warp)
#    comms.set_comms_control_timewarp_scale_factor(scaling_factor)
    
    #easily mark all of the output and input files
    file_pnt = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_pnt2.shp'  
    file_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'  
    
    # Get the driver and open the point and polygon files
    driver = ogr.GetDriverByName('ESRI Shapefile')
    ds = driver.Open(file_pnt, 0)
    ds_poly = driver.Open(file_poly, 0) # open Polygon File
    
    # There is only one layer in each file
    layer = ds.GetLayer()
    layer_poly = ds_poly.GetLayer()
    
    # Register for desired variables
    comms.set_on_connect_callback(on_connect)
    
    # Connect to the server
    comms.run('localhost',9000,'Search')
    
    NAV_X, NAV_Y, NAV_HEAD = [],[],[]
    X = Y = 0
    max_cntr = 1
    max_cntr_poly = 1
    while True:
        NAV_X, NAV_Y, NAV_HEAD = [],[],[]
        time.sleep(.001)
        ## Update the values of the ASV position (x,y) - they are in meters
        info = comms.fetch()
        # Store all values of the ASV's position
        for x in info:
            if x.is_double():
#                print x.name()+ ': ' +str(x.double())
                if x.name()=='NAV_X':
                    NAV_X.append(x.double())
                elif x.name()=='NAV_Y':
                    NAV_Y.append(x.double())
                elif x.name()=='NAV_HEADING':
                    NAV_HEAD.append(x.double())
        ## If there is a new position of the ASV, then filter the data and 
        #   highlight the ones in the search area
        # Check to see if either of the list are empty
        if len(NAV_X) != 0 and len(NAV_Y)!= 0 and len(NAV_HEAD)!=0:
            # Clear previous values from ring and poly and remove the old
            #   spatial filter
            layer.SetSpatialFilter(None)
            layer_poly.SetSpatialFilter(None)
            ring_filter = None
            poly_filter = None
                
            # Create the baseline for the search area polygon 
            ring_filter = ogr.Geometry(ogr.wkbLinearRing)
            poly_filter = ogr.Geometry(ogr.wkbPolygon)
            
            # Store most recent value of X, Y, and Heading
            X = NAV_X[NAV_X.__len__()-1]
            Y = NAV_Y[NAV_Y.__len__()-1]
            heading = NAV_HEAD[NAV_HEAD.__len__()-1]
            NAV_X, NAV_Y, NAV_HEAD = [],[],[]
            cor_head = ang4MOOS(heading)# corrected heading to convert to normal 
            
            # Search area should be a function of the vessel size and current 
            #   speed
            search_dis = 75
            
            # Calculate the correct corrections to have the search area spin 
            #   rotate with respect to the ASV's heading
            theta = 45-np.mod(cor_head,90) # Degrees
            add_sin = search_dis*np.sqrt(2)*np.sin(theta*np.pi/180)
            add_cos = search_dis*np.sqrt(2)*np.cos(theta*np.pi/180)
            
            # Convert the positions to latitude and longitude
            pt1_lon,pt1_lat = MOOSxy2LonLat(X+add_sin, Y+add_cos)
            pt2_lon,pt2_lat = MOOSxy2LonLat(X-add_cos, Y+add_sin)
            pt3_lon,pt3_lat = MOOSxy2LonLat(X-add_sin, Y-add_cos)
            pt4_lon,pt4_lat = MOOSxy2LonLat(X+add_cos, Y-add_sin)
            
            # Build a ring of the points for the search area polygon
            ring_filter.AddPoint(pt1_lon, pt1_lat)
            ring_filter.AddPoint(pt2_lon, pt2_lat)
            ring_filter.AddPoint(pt3_lon, pt3_lat)
            ring_filter.AddPoint(pt4_lon, pt4_lat)
            ring_filter.CloseRings()
            poly_filter.AddGeometry(ring_filter) # Add the ring to the previously created polygon         
            
            # Show the Search Radius Polygon on pMarnineViewer
            s_poly_vert = 'pts={'+str(X+add_sin)+','+ str(Y+add_cos)+':'+ str(X-add_cos)+','+str(Y+add_sin)+':'+ str(X-add_sin)+','+str(Y-add_cos)+':' +str(X+add_cos)+','+str(Y-add_sin)+'},'
            comms.notify('VIEW_POLYGON', s_poly_vert+'label=Search,edge_size=10,vertex_size=1,edge_color=red',)
            
            # Filter out data to determine the search area and print out the 
            #   number of potential hazards in the imediate vacinity 
            layer.SetSpatialFilter(poly_filter)
            layer.SetAttributeFilter("t_lvl>0")
            layer_poly.SetSpatialFilter(poly_filter)
            layer_poly.SetAttributeFilter("t_lvl>0")
            feature = layer.GetNextFeature()
            feature_poly = layer_poly.GetNextFeature()
            print 'Number of Hazards (point): %d' %layer.GetFeatureCount()
            print 'Number of Hazards (polygon): %d \n' %layer_poly.GetFeatureCount()
            
            # This counter is used to count the number of features in the 
            #   search radius. This is then used to determine if any of the old
            #   hazard polygons need to be removed.
            counter = 1
            counter_poly = 1
            # Counter to determine how many obstacles that are above threat 
            #   level 0 in the search radius
            num_obs = 0
            
            # Intialize the obstacle string
            obs_pos = ''
            
###############################################################################
##########             Cycle through the point obstacles             ##########
###############################################################################
            # Highlight all point features in search radius in pMarineViewer 
            #   and store them into a MOOS variable called "Obstacles"
            feature = layer.GetNextFeature() 
            while feature:
                time.sleep(.001)
                geom_point = feature.GetGeometryRef()
                new_x, new_y = LonLat2MOOSxy(geom_point.GetX(), geom_point.GetY())
                
                pos = 'x='+str(new_x)+',y='+str(new_y)
                poly_search = 'format=radial,'+pos+',radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label='+str(counter)
                comms.notify('VIEW_POLYGON', poly_search)
                                
                # Store information for the obstacle to be used later to post to the MOOSDB
                # x,y,t_lvl,type 
                if feature.GetField(0) !=0:
                    # if it isnt the first obstacle put a ! at the end
                    if num_obs!=0:
                        obs_pos += '!'
                    num_obs += 1
                    obs_pos += pos+','+ str(feature.GetField(0))+','+ str(feature.GetField(1))

                # Go to the next feature and increment the counter
                feature = layer.GetNextFeature() 
                counter += 1
            # Output to the MOOSDB a list of obstacles
            #   ASV_X,ASV_Y : # of Obstacles : x,y,t_lvl,type : x,y,t_lvl,type : ...
            obstacles = str(X)+','+str(Y)+','+str(heading)+':'+str(num_obs)+':'+obs_pos
            comms.notify('Obstacles', obstacles)            
            # Determine if a new polygon was used
            if max_cntr < counter:
                max_cntr = counter  
                
            # Remove highlighted point obstacles (shown as polygons) from 
            #   pMarineViewer
            for i in range(counter, max_cntr):
                time.sleep(.002)
                poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label='+str(i)
                comms.notify('VIEW_POLYGON', poly)
                
###############################################################################
##########                 Cycle through the polygons                ##########
###############################################################################
            # Initialize the polygon strings
            poly_str = ''
            poly_info = ''
            point = ogr.Geometry(ogr.wkbPoint)
            point.AddPoint(-70.718287193807441, 43.081495831972077)
#            feature_poly = layer_poly.GetNextFeature()
            while feature_poly:
                geom_poly = feature_poly.GetGeometryRef() # Polygon from shapefile
                # Get the interesection of the polygon from the shapefile and
                #   the outline of tiff from pMarnineViewer
                intersection_poly = geom_poly.Intersection(poly_filter) 
                
                # Get the ring of that intersection polygon
                p_ring = intersection_poly.GetGeometryRef(0) 
                
#                c = geom_poly.Centroid()                
#                print c
                # There are two potential cases - There are vertices of the 
                #   polygon within the search area and There are no vertices of
                #   the polygon within the search area.
                
                # Case 1: There are vertices within the search area therefore 
                #   we will give the polygon string function the intersection
                #   of the polygon and the search area
                if p_ring:
                    # FIX THIS - INVALID PROCEDURE
                    if point.Within(geom_poly):
                        poly_str = "No points"
                    else:
                        poly_str = polygon(X, Y, heading, feature_poly, intersection_poly, True, counter_poly)
                    
                # Case 2: there are no points in the the search window, 
                #   therefore we are temperarily giving the entire polygon to 
                #   the polygon function.
                else:
#                    if point.Within(geom_poly):
#                        poly_str = "No points"
#                    else:
                    poly_str = polygon(X, Y, heading, feature_poly, False, False, counter_poly)
   
                # If it is not the first polygon, then add a '!' to the end of 
                #   the  string.
                if poly_str!="No points":
                    if counter_poly==1:
                        poly_info = poly_str
                    elif counter_poly>1:
                        poly_info += '!'
                        poly_info += poly_str
                    # Increment counter
                    counter_poly += 1
                    
                # Go to the next polygon
                feature_poly = layer_poly.GetNextFeature()
                

            # Post an update if there are polygon obstacles
            if (layer_poly.GetFeatureCount()>0):
                poly_obs = str(X)+','+str(Y)+','+str(heading)+':'+str(counter_poly-1)+':'+poly_info
                comms.notify('Poly_Obs', poly_obs)  
                
            # Determine if a new polygon was used
            if max_cntr_poly < counter_poly:
                max_cntr_poly = counter_poly    
                
            # Remove highlighted line/polygon obstacles (shown as seglist) from 
            #   pMarineViewer
            for ii in range(counter_poly, max_cntr_poly+1):
                time.sleep(.002)
                pt_1 = 'x=1000,y=1000,vertex_size=8,vertex_color=white,active=false,label=pt1_'+str(ii)
                comms.notify('VIEW_POINT', pt_1)
                time.sleep(.002)
                pt_2 = 'x=1,y=1,vertex_size=8,vertex_color=white,active=false,label=pt2_'+str(ii)
                comms.notify('VIEW_POINT', pt_2)
                time.sleep(.002)
                pt_3 = 'x=1,y=1,vertex_size=8,vertex_color=white,active=false,label=pt3_'+str(ii)
                comms.notify('VIEW_POINT', pt_3)
            
#==============================================================================
        # MOOS freaks out when nothing is posted to the DB so post this dummy
        #   variable to avoid this problem if nothing was posted during the l
        #   last cycle
        else:
            comms.notify('dummy_var','')

if __name__ == "__main__":
    main()        