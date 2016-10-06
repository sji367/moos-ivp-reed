# -*- coding: utf-8 -*-
"""
Created on Tue Jun 21 13:58:06 2016

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

# FIX: Generalize this to the utm zone of LatOrigin/LongOrigin
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
    comms.register('NAV_X',0)
    comms.register('NAV_Y',0)
    comms.register('NAV_HEADING',0)
    comms.register('Next_WPT', 0)
    comms.register('WPT_INDEX', 0)
    comms.register('Buffer_dist', 0)
    return True

#==============================================================================
# When given two sets of points, this function calculates the slope (m) and 
#   y-intercept (b).
#==============================================================================  
def slopeintersept(x1,y1,x2,y2):
    if x2 != x1:
        
        m = float(y2-y1)/float(x2-x1)
        b = float(y1) - float(m*x1)
        return m, b
    else:
        return 'vert_line', ''

#==============================================================================
# When given 4 sets of points, this function calculates the point at which the
#   two lines that they represent intercept.
#============================================================================== 
def intercept(x1,y1, x2,y2, x3,y3, x4,y4):
    # Find slope intercept form for the first line
    if x2 != x1:
        m1, b1 = slopeintersept(x1,y1,x2,y2)
        inf_slope_flag1 = False
    else:
        inf_slope_flag1 = True
        
    # Find slope intercept form for the second line
    if x4 != x3:
        m2, b2 = slopeintersept(x3,y3,x4,y4)
        inf_slope_flag2 = False
    else:
        inf_slope_flag2 = True
        
    # Four Cases: 
    #   slope != inf for both, 
    #   slope != inf for fist line but slope = inf the second
    #   slope = inf for fist line but slope != inf the second
    #   slope = inf for both
    if inf_slope_flag1 == False and inf_slope_flag2 == False:
        if m1 == m2:
            return 'Same', ''
        x_int = (b2-b1)/(m1-m2)
        y_int = (m1*x_int) + b1
        
    elif inf_slope_flag1 == False and inf_slope_flag2:
        x_int = x3
        y_int = (m1*x_int) + b1
        
    elif inf_slope_flag1 and inf_slope_flag2 == False:
        x_int = x1
        y_int = (m2*x_int) + b2
    
#    print 'y = %f * x + %f, y = %f * x + %f'%(m1, b1, m2, b2)    
    
    if inf_slope_flag2 and inf_slope_flag1:
        if x4 == x1:
            return 'Same', ''
        else:
            return 'None', ''
    else:
#        print 'x: %f, y: %f'%(x_int,y_int)
        
        # Check to see if the calculated intercept lies on the line segments
        if x1>x2:
            pt1_x_small = x2
            pt1_x_big = x1
        else:
            pt1_x_small = x1
            pt1_x_big = x2
        
        if x3>x4:
            pt2_x_small = x4
            pt2_x_big = x3
        else:
            pt2_x_small = x3
            pt2_x_big = x4
        
        if y1>y2:
            pt1_y_small = y2
            pt1_y_big = y1
        else:
            pt1_y_small = y1
            pt1_y_big = y2
        
        if y3>y4:
            pt2_y_small = y4
            pt2_y_big = y3
        else:
            pt2_y_small = y3
            pt2_y_big = y4
        
        if pt1_x_small <= x_int <= pt1_x_big and pt2_x_small <= x_int <= pt2_x_big and pt1_y_small <= y_int <= pt1_y_big and pt2_y_small <= y_int <= pt2_y_big:
#            print '%f, %f' %(x_int, y_int)
            return x_int, y_int
        else:
            return 'None', ''

#==============================================================================
# When given 2 sets of x,y coordinates and a geometry of a GDAL polygon, this  
#   function interpolates between the polygon vertices to determine the
#   intersection of the line and the polygon vertices.
#==============================================================================
def PolyLineIntercept(geom, WPT_x, WPT_y,prev_WPT_x,prev_WPT_y, debug): 
    if debug:
        print "WPT_x: %0.3f" % WPT_x
        print "WPT_y: %0.3f" % WPT_y
        print "prev_WPT_x: %0.3f" % prev_WPT_x
        print "prev_WPT_y: %0.3f" % prev_WPT_y
        
    # Find crossing point
    ring = geom.GetGeometryRef(0)
    for j in range(ring.GetPointCount()):
        pt_lon, pt_lat,z = ring.GetPoint(j)
        pt_x, pt_y = LonLat2MOOSxy(pt_lon, pt_lat)
        # If it is the first point, set the previous point to the 
        if j == 0:
            prev_lat, prev_lon,z = ring.GetPoint(ring.GetPointCount()-1)
            prev_X, prev_Y = LonLat2MOOSxy(prev_lat, prev_lon)
            
        x_int, y_int = intercept(WPT_x, WPT_y,prev_WPT_x,prev_WPT_y, pt_x, pt_y, prev_X, prev_Y)
        
        # If there is a valid intersection return it
        if ((x_int != 'None' or x_int != 'Same')and y_int != ''):
            dist = np.sqrt(np.square(x_int-WPT_x) + np.square(y_int-WPT_y))
            return x_int, y_int, dist
        
        # Store previous value                        
        prev_X = pt_x
        prev_Y = pt_y
    return 'failed', '', ''

#==============================================================================
# When given a set of x,y (UTM=1) or lat/long (UTM=0) coordinates and desired 
#   buffer distance in meters, this function returns the buffer in lat/long.
#==============================================================================
def latlongBuffer(x, y, des_buffer, UTM):
    if UTM == 1:
        orig_lon, orig_lat = MOOSxy2LonLat(x,y)
    else:
        orig_lon = x
        orig_lat = y
        
    # Convert to lat/long
    lon, lat= LonLat2UTM(x+des_buffer, y+des_buffer, inverse=True)
    
    # Calculate lat/long buffer
    buf_lon = lon - orig_lon
    buf_lat = lat - orig_lat
    
    # Find max
    if (buf_lon > buf_lat):
        buf_max = buf_lon
    else:
        buf_max = buf_lat
    
    return buf_max
     
#==============================================================================
# This program uses the X and Y cooridinates from the ASV and filters out all 
#   of the points from the ENC database that are in a predetermined search
#   radius. It then outputs information need for obstacle avoidance to the
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
    
    file_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'  
    
    # Get the driver and open the point file first
    driver = ogr.GetDriverByName('ESRI Shapefile')
    ds = driver.Open(file_poly, 0)
    
    # There is only one layer in each file and we are just opening it to see how
    #   features there are in the layer
    layer = ds.GetLayer()
    
    # Register for desired variables
    comms.set_on_connect_callback(on_connect);
    comms.run('localhost',9000,'check')
    NAV_X, NAV_Y, NAV_HEAD, Next_WPT, buff, WPT_INDEX = [],[],[],[],[],[]
    
    # Initialize Variables
    first_iteration = True
    buffer_dist = 20 
    wpt_index = 0
    check_within = False
    skiped = False
    first_pos = True
    x_int, y_int = [],[]
    d_inside =0
    
    WPT_x = 0
    WPT_y = 0
    
    debug = False
    
    while True:
        time.sleep(.001)
        ## Update the values of the ASV position, heading and next waypoint
        info = comms.fetch()

        NAV_X, NAV_Y, NAV_HEAD, Next_WPT, buff, WPT_INDEX = [],[],[],[],[],[]
 
        # Store all values of the ASV's position and the information on the 
        #   next waypoint
        for x in info:
            if x.is_double():
                if x.name()=='NAV_X':
                    NAV_X.append(x.double())
                elif x.name()=='NAV_Y':
                    NAV_Y.append(x.double())
                elif x.name()=='NAV_HEADING':
                    NAV_HEAD.append(x.double())
                elif x.name()=='WPT_INDEX':
                    WPT_INDEX.append(x.double())
            elif x.is_string():
                if x.name() == 'Next_WPT':
                    Next_WPT.append(x.string())
                elif (x.name() == 'Buffer_dist'):
                    buff.append(x.string())
                    
        # Check to see if the buffer has changed
        if len(buff)  != 0:
            buffer_dist = float(buff[buff.__len__()-1])

        # Check to see if there is a new waypoint index
        if len(WPT_INDEX) != 0:
            wpt_index = WPT_INDEX[WPT_INDEX.__len__()-1]
            
        # Check to see if there is a new position
        if len(NAV_X) != 0 and len(NAV_Y)!= 0 and len(NAV_HEAD)!=0 and first_pos:
            # Return the most recent information on the ASV's state
            ASV_x = NAV_X[NAV_X.__len__()-1]
            ASV_y = NAV_Y[NAV_Y.__len__()-1]
            ASV_head = NAV_HEAD[NAV_HEAD.__len__()-1]
            first_pos = False
        
        # When there is a new waypoint, we want to check to see if the next 
        #   waypoint is valid and if the planned path intersects any polygon.
        #   If the waypoint is NOT valid, then we will post a flag and  
        #   calculate the intersection between the polygon and the planned
        #   path.
        if len(Next_WPT) != 0:
            check_intersection = False
            check_within = False
            x_int, y_int = [],[]
            
            # On the first iteration there is no previous waypoint. Therefore,
            #   we will set the current position of the ASV as previous 
            #   waypoint.
            if first_iteration:
                prev_WPT_x = ASV_x
                prev_WPT_y = ASV_y
                    
                first_iteration = False
                
            # Afterwards, there is a previous waypoint so we will just set the
            #   previous waypoint as the current value of WPT_x or WPT_y. If 
            #   we skiped a waypoint, the previous waypoint has been previously
            #   set in an earlier function.
            else:
                if skiped:
                    skiped = False
                else:
                    # Set the current position to the ASV as the x,y coordinate
                    #   of the previous waypoint
                    prev_WPT_x = WPT_x
                    prev_WPT_y = WPT_y
                        
            # Return the waypoint --> x,y
            WPT = Next_WPT[Next_WPT.__len__()-1]            
            # Parse waypoint string. It is in the format --> x,y
            WPT_x, WPT_y = WPT.split(",")
                    
            # Convert the string to a float
            WPT_x = float(WPT_x)
            WPT_y = float(WPT_y)
            
            # Post a polygon to pMarineViewer to show what is the current 
            #   waypoint.
            print_WPT = 'format=radial,x='+str(WPT_x)+',y='+str(WPT_y)+',radius=5,pts=4,edge_size=5,vertex_size=2,edge_color=gold,label=WPT'
            comms.notify('VIEW_POLYGON', print_WPT)
                     
            # Convert the previous and current waypoints into lat and long
            WPT_long, WPT_lat  = MOOSxy2LonLat(WPT_x, WPT_y)
            prev_WPT_long, prev_WPT_lat  = MOOSxy2LonLat(prev_WPT_x, prev_WPT_y)                
            
            # Make GDAL point for the current waypoint
            WPT_pnt = ogr.Geometry(ogr.wkbPoint)
            WPT_pnt.AddPoint(WPT_long, WPT_lat)
            
            # Make GDAL point for the previous waypoint
            prev_WPT_pnt = ogr.Geometry(ogr.wkbPoint)
            prev_WPT_pnt.AddPoint(prev_WPT_long, prev_WPT_lat)
            
            # Make GDAL line string for the current and previous waypoints
            WPT_line = ogr.Geometry(ogr.wkbLineString)
            WPT_line.AddPoint(WPT_long, WPT_lat)
            WPT_line.AddPoint(prev_WPT_long, prev_WPT_lat)
            
            ## Make GDAL polygon encapsulating the current and previous 
            #   waypoints with a buffer around it
            WPT_poly = ogr.Geometry(ogr.wkbPolygon)
#            WPT_ring = ogr.Geometry(ogr.wkbLinearRing)
            
            # Add a buffer around the waypoint
            Buffer = 0.0001#10
#            buf_max = latlongBuffer(WPT_long, WPT_lat, Buffer, 0)
            WPT_poly = WPT_line.Buffer(Buffer)
#            print '%f, %f'%(buf_lon, buf_lat)
#                
#            # Allow the polygon to rotate and be correct
#            angle = np.arctan2(WPT_lat-prev_WPT_lat, WPT_long-prev_WPT_long)
#            add_x = buf_max*np.sin(angle)
#            add_y = buf_max*np.cos(angle)
#            
#            # Add points to the ring
#            WPT_ring.AddPoint(WPT_long-add_x, WPT_lat+add_y)
#            WPT_ring.AddPoint(WPT_long+add_x, WPT_lat-add_y)
#            WPT_ring.AddPoint(prev_WPT_long+add_x, prev_WPT_lat-add_y)
#            WPT_ring.AddPoint(prev_WPT_long-add_x, prev_WPT_lat+add_y)
#            WPT_ring.CloseRings()
#            WPT_poly.AddGeometry(WPT_ring)
#            
#            # Show the flag polygon in pMarineViewer
#            pt1,pt2 = LonLat2UTM(WPT_long-add_x, WPT_lat+add_y)
#            pt3,pt4 = LonLat2UTM(WPT_long+add_x, WPT_lat-add_y)
#            pt5,pt6 = LonLat2UTM(prev_WPT_long+add_x, prev_WPT_lat-add_y)
#            pt7,pt8 = LonLat2UTM(prev_WPT_long-add_x, prev_WPT_lat+add_y)
#                
#            poly = 'pts={'+str(pt1-x_origin)+','+str(pt2-y_origin)+':'+str(pt3-x_origin)+','+str(pt4-y_origin)+':'+str(pt5-x_origin)+','+str(pt6-y_origin)+':'+str(pt7-x_origin)+','+str(pt8-y_origin)+'},label=check,edge_color=blue'
#            comms.notify('VIEW_POLYGON',poly)              
                
            ## Check to see if the next waypoint/path is valid
            for i in range (0,layer.GetFeatureCount()):
                # Get the info on the feature
                feature = layer.GetFeature(i)
                geom = feature.GetGeometryRef()
                
                # Check to see if the next waypoint is within the polygon
                #   obstacle. If it is, get the intersection between the 
                #   polygon and the planned path.
                if WPT_pnt.Within(geom):
                    # Get the X and Y intercepts
                    cur_x_int, cur_y_int, d_inside = PolyLineIntercept(geom, WPT_x, WPT_y,prev_WPT_x,prev_WPT_y, debug)
                    check_within = True
                    
                    print str(cur_x_int) +', '+str(cur_y_int)
#                    if isinstance(cur_x_int, float):
                    comms.notify('VIEW_POINT','x='+str(cur_x_int) +','+'y='+str(cur_y_int)+',vertex_size=6.5,vertex_color=pink,active=true,label=int')
                    x_int.append(cur_x_int)
                    y_int.append(cur_y_int)
                    
                # Check to see if the planned path will intersect the 
                #   polygon obstacle
                if WPT_poly.Intersect(geom):
                    check_intersection = True
                
            # Update flags and print out status message
            comms.notify('INVALID_WPT', str(check_within))
            if check_within:
                print "ERROR: Next Waypoint is NOT Valid"
            else:
                print "Next Waypoint is Valid"
                        
            comms.notify('INVALID_PATH', str(check_intersection))     
            if check_intersection:
                print "ERROR: Planned path intersects obstacles"
            else:
                print "Planned path is  Valid"
                    
        #======================================================================
        # MOOS freaks out when nothing is posted to the DB so post this dummy
        #   variable to avoid this problem if nothing was posted during the
        #   last cycle.
        #======================================================================
        else:
            comms.notify('dummy_var','')
        
        # Check to see if there is a new position and if there is check to see
        # if the waypoint needs to be skipped.
        if len(NAV_X) != 0 and len(NAV_Y)!= 0 and len(NAV_HEAD)!=0 and not first_pos:
            # Return the most recent information on the ASV's state
            ASV_x = NAV_X[NAV_X.__len__()-1]
            ASV_y = NAV_Y[NAV_Y.__len__()-1]
            ASV_head = NAV_HEAD[NAV_HEAD.__len__()-1]
            
            # If the waypoint is within a polygon, then switch to the next
            #   waypoint when it is comes within the buffer distance
            if check_within:
                for k in range(len(x_int)):
                    dist = np.sqrt(np.square(x_int[k]-ASV_x) + np.square(y_int[k]-ASV_y))
                    if dist <= buffer_dist:
                        # Increment to the next waypoint 
                        comms.notify('WPT_UPDATE', 'currix='+str(int(wpt_index)+1))
                        comms.notify('VIEW_POINT','x=1,y=1,active=false,label=int')
                        print 'Skiped WPT %i, going to %i' % (wpt_index, wpt_index+1)
                        check_within = False
                        prev_WPT_x = ASV_x
                        prev_WPT_y = ASV_y
                        skiped = True
    
if __name__ == "__main__":
    main()   