# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 15:34:58 2016

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

#==============================================================================
## Initialize some global variables
#==============================================================================

# Initialize the MOOS connection
comms = pymoos.comms()

# Calculate the origin 
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

#==============================================================================
## Register for updates of the MOOS variables NAV_X and NAV_Y once every second
#==============================================================================
def on_connect():
    comms.register('NAV_X', 1)
    comms.register('NAV_Y', 1)
    comms.register('NAV_HEADING', 1)
    return True
    
#==============================================================================
# To convert the calculated angle to the one that relates to the one that MOOS
#   outputs, you have to use the formula: MOOS_ang = -(calc_ang-90)
#==============================================================================
def ang4MOOS(angle):
    return -(angle-90)
    
    
# Converts the number stored in Category of Landmark to a string
def category_landmark(feat, name):
    if name == 'LNDMRK':
        index = str(feat.GetField(11))
        if index == '1':
            return 'Cairn'
        elif index =='2':
            return 'Cemetery'
        elif index =='3':
            return 'Chimney'
        elif index =='4':
            return 'Dish Aerial'
        elif index =='5':
            return 'Flagstaff'
        elif index =='6':
            return 'Flare Stack'
        elif index == '7':
            return 'Mast'
        elif index == '8':
            return 'Windsock'
        elif index =='9':
            return 'Monument'
        elif index == '10':
            return 'Column'
        elif index == '11':
            return 'Memorial Plaque'
        elif index == '12':
            return 'Obelisk'
        elif index == '13':
            return 'Statue'
        elif index == '14':
            return 'Cross'
        elif index == '15':
            return 'Dome'
        elif index == '16':
            return 'Radar Scanner'
        elif index == '17':
            return 'Tower'
        elif index == '18':
            return 'Windmill'
        elif index == '19':
            return 'Windmotor'
        elif index =='20':
            return 'Spire'
        elif index =='21':
            return 'Large On Land Rock'
        else:
            return 'Unknown Landmark'
    elif name == 'SILTNK':
        index = str(feat.GetField(12))
        if index == '1':
            return 'Silo'
        elif index =='2':
            return 'Tank'
        elif index =='3':
            return 'Grain Elevator'
        elif index =='4':
            return 'Water Tower'

# Converts the number stored in Category of Special Purpose Beacon/Buoy to 
#   string
def category_SPP(feat):
    index = str(feat.GetField(12))
    if index == '1':
        return 'Firing Danger Area'
    elif index =='2':
        return 'Target'
    elif index =='3':
        return 'Marker Ship'
    elif index =='4':
        return 'Degaussing Range'
    elif index =='5':
        return 'Barge'
    elif index =='6':
        return 'Cable'
    elif index == '7':
        return 'Spoil Ground'
    elif index == '8':
        return 'Outfall'
    elif index =='9':
        return 'ODAS'
    elif index == '10':
        return 'Recording'
    elif index == '11':
        return 'Seaplane Anchorage'
    elif index == '12':
        return 'Recreation'
    elif index == '13':
        return 'Private'
    elif index == '14':
        return 'Mooring'
    elif index == '15':
        return 'LANDBY'
    elif index == '16':
        return 'Leading'
    elif index == '17':
        return 'Measured Distance'
    elif index == '18':
        return 'Notice'
    elif index == '19':
        return 'TSS'
    elif index =='20':
        return 'Ancoring Prohibited'
    elif index =='21':
        return 'Berthing Prohibited'
    elif index =='22':
        return 'Overtaking Prohibited'
    elif index =='23':
        return 'Two-way Traffic Prohibited'
    elif index =='24':
        return 'Reduced Wake'
    elif index =='25':
        return 'Speed Limit'
    elif index =='26':
        return 'Stop'
    elif index =='27':
        return 'General Warning'
    elif index =='28':
        return 'Sound Ships Siren'
    elif index =='29':
        return 'Restricted Vertical Clearance'
    elif index =='30':
        return 'Maximum Vessels Draught Mark'
    elif index =='31':
        return 'Restricted Horizontal Clearance'
    elif index =='32':
        return 'Strong Current Warning'
    elif index =='33':
        return 'Beartging Permitted'
    elif index =='34':
        return 'Overhead Power Cable'
    elif index =='35':
        return 'Channel Edge Gradient'
    elif index =='36':
        return 'Telephone'
    elif index =='37':
        return 'Ferry Crossing'
    elif index =='39':
        return 'Pipeline'
    elif index =='40':
        return 'Anchorage'
    elif index =='41':
        return 'Clearing'
    elif index =='42':
        return 'Control'
    elif index =='43':
        return 'Diving'
    elif index =='44':
        return 'Refuge'
    elif index =='45':
        return 'Foul Ground'
    elif index =='46':
        return 'Yachting'
    elif index =='47':
        return 'Heliport'
    elif index =='48':
        return 'GPS'
    elif index =='49':
        return 'Seaplane Landing'
    elif index =='50':
        return 'Entry Prohibited'
    elif index =='51':
        return 'Work in Progress'
    elif index =='53':
        return 'Wellhead'
    elif index =='54':
        return 'Channel Seperation'
    elif index =='55':
        return 'Marine Farm'
    elif index =='56':
        return 'Artificial Reef'
    else:
        return 'Special Purpose'

# Converts the number stored in Category of Lateral Beacon/Buoy to string    
def category_LAT(feat):
    index = str(feat.GetField(12))
    if index == '1':
        return 'Port-hand'
    elif index =='2':
        return 'Starboard-hand'
    elif index =='3':
        return 'Perferred Channel to Starboard'
    elif index =='4':
        return 'Perferred Channel to Port'
    else:
        return 'Lateral'

# Converts the number stored in Category of Cardinal Beacon/Buoy to string      
def category_CAR(feat):
    index = str(feat.GetField(12))
    if index == '1':
        return 'North'
    elif index =='2':
        return 'East'
    elif index =='3':
        return 'South'
    elif index =='4':
        return 'West'
    else:
        return 'Cardinal'

# Converts the number stored in Color to string
def obj_color(index):
    index = str(index)
    if index == '1':
        return 'White'
    elif index =='2':
        return 'Black'
    elif index =='3':
        return 'Red'
    elif index =='4':
        return 'Green'
    elif index =='5':
        return 'Blue'
    elif index =='6':
        return 'Yellow'
    elif index == '7':
        return 'Grey'
    elif index == '8':
        return 'Brown'
    elif index =='9':
        return 'Amber'
    elif index == '10':
        return 'Violet'
    elif index == '11':
        return 'Orange'
    elif index == '12':
        return 'Magenta'
    elif index == '13':
        return 'Pink'
    else:
        return 'Unknown Color'
        
# Converts the number stored in Category of Lights to string    
def category_lights(feat):
    index = str(feat.GetField(12))
    if index == '1':
        return 'Directional Function'
#        #Index 2 and 3 are not used
#    elif index =='2':
#        return ''
#    elif index =='3':
#        return ''
    elif index =='4':
        return 'Leading'
    elif index =='5':
        return 'Aero'
    elif index =='6':
        return 'Air Obstruction'
    elif index == '7':
        return 'Fog Detector'
    elif index == '8':
        return 'Flood'
    elif index =='9':
        return 'Strip'
    elif index == '10':
        return 'Subsidiary'
    elif index == '11':
        return 'Spot'
    elif index == '12':
        return 'Front'
    elif index == '13':
        return 'Rear'
    elif index == '14':
        return 'Lower'
    elif index == '15':
        return 'Upper'
    elif index == '16':
        return 'Moire Effect'
    elif index == '17':
        return 'Emergency'
    elif index == '18':
        return 'Bearing'
    elif index == '19':
        return 'Horizontally Disposed'
    elif index =='20':
        return 'Vertically Disposed'
    else:
        return 'Marine'

# Converts the number stored in Category of Wreck to string  
def category_wreck(feat):
    index = str(feat.GetField(11))
    if index == '1':
       return 'Non-Dangerous Wreck'
    elif index =='2':
        return 'Dangerous Wreck'
    elif index =='3':
        return 'Distributed Remains of Wreck'
    elif index =='4':
        return 'Wreck Showing Mast(s)'
    elif index =='5':
        return 'Wreck Showing Any Portionof Hull or Superstructure'
    else:
        return 'Wreck'

# Converts the number stored in Water Level Attribute to string     
def water_level(index):
    index = str(index)
    if index == '1':
        return 'is Partly Submerged at High Water'
    elif index =='2':
        return 'is Alyways Dry'
    elif index =='3':
        return 'is Always Underwater/Submerged'
    elif index =='4':
        return 'Covers and Uncovers'
    elif index =='5':
        return 'is Awash'
    elif index =='6':
        return 'is Subject to Inundation or Floating'
    elif index == '7':
        return 'is Floating'
    else:
        return 'is Unknown'

# Converts the information from the ENC on naviagational aids to something 
#   humans can understand
def category_nav_aid(feat, name):
    # BOYISD --> 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
    # BOYLAT --> 11 - Buoy Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern 
    # BOYSPP --> 11 - Buoy Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern
    # BOYSAW --> 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
    # BCNLAT --> 11 - Beacon Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
    # BCNSPP --> 11 - Beacon Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
    # LIGHTS --> 11- Category of Light, 16 - Height, 23 - Light Orientation
    if name == 'LIGHTS':
        nav_aid = category_lights(feat)
        nav_aid += ' Light'
        
    elif name == 'BOYSPP':
        nav_aid = obj_color(feat.GetField(13)) + ' '
        nav_aid += category_SPP(feat)
        nav_aid += ' Buoy'
        
    elif name == 'BOYISD':
        nav_aid = obj_color(feat.GetField(12)) + ' '
        nav_aid += 'Isolated Danger Buoy'
        
    elif name == 'BOYSAW':
        nav_aid = obj_color(feat.GetField(12)) + ' '
        nav_aid += 'Safe Water Bouy'
        
    elif name == 'BOYLAT':
        nav_aid = obj_color(feat.GetField(13)) + ' '
        nav_aid += category_LAT(feat)
        nav_aid += ' Buoy'
        
    elif name == 'BCNSPP':
        nav_aid = obj_color(feat.GetField(13)) + ' '
        nav_aid += category_SPP(feat)
        nav_aid += ' Buoy'
        
    elif name == 'BCNLAT':
        nav_aid = obj_color(feat.GetField(13)) + ' '
        nav_aid += category_LAT(feat)
        nav_aid += ' Buoy'
        
    else:
        nav_aid = 'Unknown Navigational Aid'
        
    return nav_aid

# Converts the information from the ENC on underwater objects to something 
#   humans can understand
def category_underwater(feat, name):
    # UWTROC --> 20 - Value of sounding, 22 - WL
    # WRECKS --> 11 - Catagory of wreck, 22 - Value of the Sounding, 26 - WL
    if name == 'UWTROC':
        obj_type = 'Rock'
        z = feat.GetField(20)
        if z is None:
            obj_type += ' that '+water_level(str(feat.GetField(22)))
#            print water_level(feat.GetField(22))
        else:
            obj_type += ' @ '+str(z)+'m (MLLW)'
        
    elif name == 'WRECKS':
        obj_type = category_wreck(feat)
        z = feat.GetField(20)
        if z is None:
            obj_type += ' that is ' + water_level(str(feat.GetField(26)))
        else:
            obj_type += ' @ '+str(z)+'m  (MLLW)'
        
    else:
        obj_type = 'Unknown Underwater Obstacle'
    
    return obj_type
    
# Function to build the filter
#   UTM_in:  0 --> input is in Lat/Long
#            1 --> input is in UTM
#   Output is the Lat/Long filter polygon
# It assumes that the sensor heading is defined as counterclockwise = positive
def build_filter(max_dist, in1, in2, ASV_head, sensor_FoV_ang, sensor_head, UTM_in, f_type):
    # Get both Lat/long and UTM
    if UTM_in == 0:
        lon = in1
        lat = in2
        x,y = LonLat2MOOSxy(lon, lat)
    else:
        x = in1
        y = in2
        lon,lat = MOOSxy2LonLat(x, y)
        
    # Make sure that the heading is from 0-360
    ASV_head = np.mod(ASV_head, 360)
    
    # Build polygon for pMarineViewer
    hypot = max_dist/np.cos(sensor_FoV_ang)
    ang1 = sensor_head+ASV_head+sensor_FoV_ang
    ang2 = sensor_head+ASV_head-sensor_FoV_ang
    
    # Find the sensor FoV polygon as it rotates
    A_x = x
    A_y = y
    B_x = x+hypot*np.cos(ang1*np.pi/180)
    B_y = y+hypot*np.sin(ang1*np.pi/180)
    C_x = x+hypot*np.cos(ang2*np.pi/180)
    C_y = y+hypot*np.sin(ang2*np.pi/180)
    
    # Print it to pMarnineViewer
    print 'ASV: ' + str(ASV_head)+', A: ' + str(ang1)+', B:' + str(ang2)
#    if f_type == 'Underwater':
    color = 'edge_color=blue'
#    elif f_type == 'Landmark':
#        color = 'edge_color=salmon'
#    elif f_type == 'Nav_aid':
#        color = 'edge_color=seagreen'
    FoV_Filter = 'pts={'+str(A_x)+','+str(A_y)+':'+str(B_x)+','+str(B_y)+':'+str(C_x)+','+str(C_y)+'},label=Sensor_FoV_'+f_type+','+color
    
    time.sleep(.001)
    comms.notify('VIEW_POLYGON', FoV_Filter, pymoos.time()) 
    
    # Initialize filers
    ring_filter = ogr.Geometry(ogr.wkbLinearRing)
    poly_filter = ogr.Geometry(ogr.wkbPolygon)
    
    # Build Lat/Long filter
    B_lon,B_lat = MOOSxy2LonLat(B_x, B_y)
    C_lon,C_lat = MOOSxy2LonLat(C_x, C_y)
    ring_filter.AddPoint(lon,lat)
    ring_filter.AddPoint(B_lon,B_lat)
    ring_filter.AddPoint(C_lon,C_lat)
    ring_filter.CloseRings()
    poly_filter.AddGeometry(ring_filter)
    
    return poly_filter
    


#==============================================================================
# This program uses the X and Y cooridinates from the ASV and filters out all 
#   of the points from the ENC database that are in sensor's field of view. It
#   then outputs information on the obstacles to the MOOSDB as a string.
#==============================================================================
def main(): 
#    # Time Warp and Scaling factor constant
#    time_warp = 2
#    scaling_factor = 0.04*time_warp    
#    
#    # Set the timewarp and scale factor
#    pymoos.set_moos_timewarp(time_warp)
#    comms.set_comms_control_timewarp_scale_factor(scaling_factor)
    
    # Paths to all of the ENC files
    s57filename =  "/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/US5NH02M.000"
    
    # Open the S57 file
    ds = ogr.Open(s57filename)
    
    # The layers that we care about are:
    Nav_aids =  ['LIGHTS', 'BOYSPP', 'BOYISD', 'BOYSAW', 'BOYLAT', 'BCNSPP', 'BCNLAT']
    Landmarks = ['LNDMRK', 'SILTNK']
    Underwater = ['UWTROC', 'WRECKS']
    
    # Register for desired variables
    comms.set_on_connect_callback(on_connect);
    comms.run('localhost',9000,'Sensor_FoV')
    NAV_X, NAV_Y, NAV_HEAD  = [],[],[]
    
    # Initialize some constants
    landmark_max_dist = 3000
    nav_aid_max_dist = 1500
    underwater_max_dist = 500 #  Somewhere between 250m and 500m
    sensor_head = 0#15 # degrees
    sensor_FoV_ang = 45
    
    
    
    # This loop checks to see if there is any mail and if there is, it sorts 
    #   it. If there is a new position and heading of the ASV, then it 
    #   determines if there are any objects from the ENC that are within the 
    #   field of view of a sensor. Currently, this is split into 3 different
    #   categories of obstacles: Navigational Aids, Landmarks, and General
    #   Underwater.
    while True:
        time.sleep(.001)
        ## Update the values of the ASV position (x,y) - they are in meters
        info = comms.fetch()
        
        # Store all values of the ASV's position
        for x in info:
            if x.is_double():
                if x.name()=='NAV_X':
                    NAV_X.append(x.double())
                elif x.name()=='NAV_Y':
                    NAV_Y.append(x.double())
                elif x.name()=='NAV_HEADING':
                    NAV_HEAD.append(x.double())
        
        ## If there is a new position of the ASV, then filter the data and 
        #   highlight the ones in the search area
        if len(NAV_X) != 0 and len(NAV_Y)!= 0:
            # Clear previous values from ring and poly
            poly_filter = None
        
            # Store most recent value of X, Y, and Heading
            X = NAV_X[NAV_X.__len__()-1]
            Y = NAV_Y[NAV_Y.__len__()-1]
            
            if len(NAV_HEAD)!=0:
                heading = NAV_HEAD[NAV_HEAD.__len__()-1]
                
            NAV_X, NAV_Y, NAV_HEAD = [],[],[]
            cor_head = ang4MOOS(heading)# corrected heading to convert to normal 
            
            ## Landmarks  
            # Build the spatical filter
            poly_filter = build_filter(landmark_max_dist, X, Y, cor_head, sensor_FoV_ang, -sensor_head, 1, 'Landmark')
            
            obj1, dist, xpos_l, ypos_l = [],[],[],[]
            landmark_objects = ''
            # Cycle through the Landmarks
            for i in range(len(Landmarks)):
                layer = ds.GetLayerByName(Landmarks[i])
                # Set spatial filter
                layer.SetSpatialFilter(poly_filter)
                
                # print all 
                feat = layer.GetNextFeature()
                while feat:
                    geom = feat.GetGeometryRef()
                    geom_name = geom.GetGeometryName()
                    if geom_name == 'LINESTRING' or geom_name == 'POLYGON':
                        centroid = geom.Centroid()
                        pt_lon = centroid.GetX()
                        pt_lat = centroid.GetY()
                    else:
                        pt_lon = geom.GetX()
                        pt_lat = geom.GetY()
                    obj_type = category_landmark(feat,Landmarks[i])
                    pt_x, pt_y = LonLat2MOOSxy(pt_lon,pt_lat)
                    
                    xpos_l.append(pt_x)
                    ypos_l.append(pt_y)
                    # Calc dist to Landmark
                    d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                    print 'x = %f, y = %f, dist = %f' %(pt_x, pt_y, d)
                    dist.append(d)
#                    print "dist1: %f"%d
                    obj1.append(obj_type + ',Long:' + str(pt_lon)+ ',Lat:' + str(pt_lat)+ ',' +str(d))
                    feat = layer.GetNextFeature()
                        
            # Only output out the closest 5 landmarks
            cntr = 0  
            num_landmarks = len(dist)
            
            if len(dist)>5:
                # Sort the landmarks by distance away from the ASV and only
                #   store the closest 5
                for ii in (sorted(range(num_landmarks), key=lambda j: dist[j])[:5]):
                    landmark_objects += obj1[ii]
                    time.sleep(.002)
                    print 'x='+str(xpos_l[ii])+', y='+str(ypos_l[ii]) + ',dist='+str(dist[ii])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_l[ii])+',y='+str(ypos_l[ii])+',radius=30,pts=5,edge_color=pink,label=landmark_'+str(cntr))
                    if cntr < 4:
                        landmark_objects += '!'
                    cntr += 1
            else:
                for ij in range(num_landmarks):
                    landmark_objects += obj1[ij]
                    time.sleep(.002)
                    print 'x='+str(xpos_l[ij])+', y='+str(ypos_l[ij])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_l[ij])+',y='+str(ypos_l[ij])+',radius=30,pts=5,edge_color=pink,label=landmark_'+str(ij))
                    if ij < num_landmarks-1:
                        landmark_objects += '!'
                for j in range(num_landmarks, 5):
                    time.sleep(.002)
                    poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=landmark_'+str(j)
                    comms.notify('VIEW_POLYGON', poly)
            if landmark_objects != '':
                print landmark_objects
                comms.notify('Landmarks', landmark_objects)
            layer.SetSpatialFilter(None)
            print ''
            
            
            ## Navigational Aids
            # Build the spatical filter
            poly_filter = build_filter(nav_aid_max_dist, X, Y, cor_head, sensor_FoV_ang, sensor_head, 1, 'Nav_aid')
            
            obj2, dist, xpos_na, ypos_na = [],[],[],[]
            nav_aid_objects = ''
            # Cycle through the Navigational aids
            for i in range(len(Nav_aids)):
                layer = ds.GetLayerByName(Nav_aids[i])
                # Set spatial filter
                layer.SetSpatialFilter(poly_filter)
                
                # print all 
                feat = layer.GetNextFeature()
                while feat:
                    geom = feat.GetGeometryRef()
                    geom_name = geom.GetGeometryName()
                    if geom_name == 'LINESTRING' or geom_name == 'POLYGON':
                        centroid = geom.Centroid()
                        pt_lon = centroid.GetX()
                        pt_lat = centroid.GetY()
                    else:
                        pt_lon = geom.GetX()
                        pt_lat = geom.GetY()
                        
                    obj_type = category_nav_aid(feat, Nav_aids[i])
                    pt_x, pt_y = LonLat2MOOSxy(pt_lon,pt_lat)
                    
                    xpos_na.append(pt_x)
                    ypos_na.append(pt_y)
                    # Calc dist to Landmark
                    d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                    print 'x = %f, y = %f, dist = %f' %(pt_x, pt_y, d)
                    
#                    print "dist2: %f"%d
                    dist.append(d)
                    obj2.append(obj_type + ',Long:' + str(pt_lon)+ ',Lat:' + str(pt_lat)+ ',' +str(d))
                    feat = layer.GetNextFeature()

            # Only output the closest 5 Nav_Aids
            cntr = 0
            num_nav_aid = len(dist)
            if num_nav_aid>5:
                # Sort the Navigational Aids by distance away from the ASV and 
                #   only store the closest 5
                for ii in (sorted(range(num_nav_aid), key=lambda j: dist[j])[:5]):
                    nav_aid_objects += obj2[ii]
                    time.sleep(.002)
                    print 'x='+str(xpos_na[ii])+', y='+str(ypos_na[ii])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_na[ii])+',y='+str(ypos_na[ii])+',radius=30,pts=5,edge_color=darkorange,label=nav_aid_'+str(cntr))
                    if cntr < 4:
                        nav_aid_objects += '!'
                    cntr += 1
            else:
                for ij in range(num_nav_aid):
                    nav_aid_objects += obj2[ij]
                    time.sleep(.002)
                    print 'x='+str(xpos_na[ij])+', y='+str(ypos_na[ij])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_na[ij])+',y='+str(ypos_na[ij])+',radius=30,pts=5,edge_color=darkorange,label=nav_aid_'+str(ij))
                    if ij < num_nav_aid-1:
                        nav_aid_objects += '!'
                for j in range(num_nav_aid, 5):
                    time.sleep(.002)
                    poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=nav_aid_'+str(j)
                    comms.notify('VIEW_POLYGON', poly)
            if nav_aid_objects != '':
                print nav_aid_objects
                comms.notify('Nav_Aids', nav_aid_objects)
            layer.SetSpatialFilter(None)
            print ''                    
            
            
            ## Underwater Objects
            # Build the spatical filter
            poly_filter = build_filter(underwater_max_dist, X, Y, cor_head, sensor_FoV_ang, 0, 1, 'Underwater')
            
            obj3, dist, xpos_u, ypos_u = [],[],[],[]
            underwater_objects = ''
            # Cycle through the Underwater Objects
            for i in range(len(Underwater)):
                layer = ds.GetLayerByName(Underwater[i])
                # Set spatial filter
                layer.SetSpatialFilter(poly_filter)
                
                # print all 
                feat = layer.GetNextFeature()
#                print Underwater[i] + ' Objects: '+str(layer.GetFeatureCount())
                while feat:
                    geom = feat.GetGeometryRef()
                    geom_name = geom.GetGeometryName()
                    if geom_name == 'LINESTRING' or geom_name == 'POLYGON':
                        centroid = geom.Centroid()
                        pt_lon = centroid.GetX()
                        pt_lat = centroid.GetY()
                    else:
                        pt_lon = geom.GetX()
                        pt_lat = geom.GetY()
                    obj_type = category_underwater(feat, Underwater[i])
                    pt_x, pt_y = LonLat2MOOSxy(pt_lon,pt_lat)
                    
                    xpos_u.append(pt_x)
                    ypos_u.append(pt_y)
                    # Calc dist to Landmark
                    d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                    
                    print 'x = %f, y = %f, dist = %f' %(pt_x, pt_y, d)
                    dist.append(d)
                    obj3.append(obj_type + ',Long:' + str(pt_lon)+ ',Lat:' + str(pt_lat)+ ',' +str(d))
                    feat = layer.GetNextFeature()
                    
            # Only output the closest 5 Underwater Objects
            cntr = 0
            num_underwater = len(dist)
            if num_underwater>5:
                # Sort the Underwater Objects by distance away from the ASV and 
                #   only store the closest 5
                for ii in (sorted(range(num_underwater), key=lambda j: dist[j])[:5]):
                    underwater_objects += obj3[ii]
                    print 'x='+str(xpos_u[ii])+', y='+str(ypos_u[ii])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_u[ii])+',y='+str(ypos_u[ii])+',radius=30,pts=5,edge_color=deepskyblue ,label=underwater_'+str(cntr))                  
                    time.sleep(.002)
                    if cntr < 4:
                        underwater_objects += '!'
                    cntr += 1
            else:
                for ij in range(num_underwater):
                    underwater_objects += obj3[ij]
                    time.sleep(.002)
                    print 'x='+str(xpos_u[ij])+', y='+str(ypos_u[ij])
                    comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_u[ij])+',y='+str(ypos_u[ij])+',radius=30,pts=5,edge_color=deepskyblue ,label=underwater_'+str(ij))
                    if ij < num_underwater-1:
                        underwater_objects += '!'
                for j in range(num_underwater, 5):
                    time.sleep(.002)
                    poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=underwater_'+str(j)
                    comms.notify('VIEW_POLYGON', poly)
            if underwater_objects != '':
                print underwater_objects
                comms.notify('Underwater_Objects', underwater_objects)
            layer.SetSpatialFilter(None)
            print ''                     
                    
#==============================================================================
        # MOOS freaks out when nothing is posted to the DB so post this dummy
        #   variable to avoid this problem if nothing was posted during the l
        #   last cycle
        else:
            comms.notify('dummy_var','')

if __name__ == "__main__":
    main()   

