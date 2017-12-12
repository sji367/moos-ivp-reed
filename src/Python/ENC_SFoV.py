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

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class MOOS_comms(object):
    """ This class id for MOOS communications. It has 2 parts:
          1. Initialize comms
              a. Register for variables
              b. Connect to the server
          2. Get new values.
    """
    
    def __init__(self):
        # Open a communication link to MOOS
        self.comms = pymoos.comms()
        # Placeholder for the list of varables that we want
        self.NAV_X = []
        self.NAV_Y = []
        self.NAV_HEAD = []

    def Register_Vars(self):
        """ This function registers for the updates of the X,Y and heading at 
            the rate of which it is being outputed from MOOS.
        """
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('NAV_HEADING', 0)
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
        self.comms.run('localhost',9000,'SFoV1')
        
    def Get_mail(self):
        """ When called, this function fetches the new values for the ASV 
            position in meters (x,y) and heading and stores them into lists.
        """
        
        # Fetch the values of the ASV position in meters (x,y) and heading
        info = self.comms.fetch()
        
        # Store all values of the ASV's position
        for x in info:
            if x.is_double():
                if x.name()=='NAV_X':
                    self.NAV_X.append(x.double())
                elif x.name()=='NAV_Y':
                    self.NAV_Y.append(x.double())
                elif x.name()=='NAV_HEADING':
                    self.NAV_HEAD.append(x.double())
                    
#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class ENC_SFoV(object):
    """ This program uses the X and Y cooridinates from the ASV and filters out 
        all of the points from the ENC database that are in sensor's field of 
        view. It then outputs information on the obstacles to the MOOSDB as a 
        string. 
    """
    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')    
    
    def __init__(self, sensor_head=0, sensor_max_dist=500, sensor_FoV_ang=45,
                 obs_type=['landmark', 'nav_aid'], LatOrigin=43.071959194444446, 
                 LongOrigin=-70.711610833333339,
                 ENC_filename='../../src/ENCs/US5NH02M/US5NH02M.000'):
        """ Initialize varibles. 
            
            Inputs:
                sensor_head - heading of the sensor in the ship refernce frame
                sensor_max_dist - Maximum range of the sensor
                sensor_FoV_ang - Maximum angular range of the sensor
                obs_type - List of the type of obstacles that the sensor is 
                        looking for (landmark, nav_aid, underwater)
                        NOTE: This must be a list
                LatOrigin - Latitude origin for the pyproj conversion to UTM
                LongOrigin - Longitude origin for the pyproj conversion to UTM
                ENC_filename - path to the ENC file
        """
        self.sensor_head = sensor_head
        self.sensor_max_dist = sensor_max_dist
        self.sensor_FoV_ang = sensor_FoV_ang
        self.obs_type = [x.lower() for x in obs_type] # makes it lowercase
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        self.ds = ogr.Open(ENC_filename)
        
    def ang4MOOS(self, angle):
        """ This fucntion converts a typical angle to one that is 0 degrees at
        North
        """
        return -(angle-90)
        
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
        
    def category_landmark(self, feat, name):
        """ This function converts the number stored in category of landmark 
            attribute in the ENC to a string
            
        Inputs:
            feat - Feature for which we want to find the category of landmark
            name - ENC Layer name
            
        Outputs:
            Category of landmark
        """
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
    def category_SPP(self, feat):
        """ This function converts the number stored in category of Special 
            Purpose Beacon/Buoy attribute in the ENC to a string.
            
        Inputs:
            feat - Feature for which we want to find the category of special 
                Purpose Beacon/Buoy
            
        Outputs:
            Category of Special Purpose Buoy/Beacon
        """
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
    def category_LAT(self, feat):
        """ This function converts the number stored in category of Lateral 
            Beacon/Buoy attribute in the ENC to a string.
            
        Inputs:
            feat - Feature for which we want to find the category of Lateral 
                Beacon/Buoy
            
        Outputs:
            Category of Lateral Buoy/Beacon
        """
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
    def category_CAR(self, feat):
        """ This function converts the number stored in category of Cardinal 
            Beacon/Buoy attribute in the ENC to a string.
            
        Inputs:
            feat - Feature for which we want to find the category of Cardinal 
                Beacon/Buoy
            
        Outputs:
            Category of Cardinal Buoy/Beacon
        """
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
    def obj_color(self, index):
        """ This function converts the number stored in color attribute in the
            ENC to a string.
            
        Inputs:
            index - Index for which we want to find the color
            
        Outputs:
            Color defined in the ENC for the object
        """
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
    def category_lights(self, feat):
        """ This function converts the number stored in category of light
            attribute in the ENC to a string.
            
        Inputs:
            feat - Feature for which we want to find the category of light
            
        Outputs:
            Category of light
        """
        index = str(feat.GetField(12))
        if index == '1':
            return 'Directional Function'
        #Index 2 and 3 are not used
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
    def category_wreck(self, feat):
        """ This function converts the number stored in category of wreck
            attribute in the ENC to a string.
            
        Inputs:
            feat - Feature for which we want to find the category of wreck
            
        Outputs:
            Category of wreck
        """
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
    def water_level(self, index):
        """ This function converts the number stored in Water Level attribute 
            in the ENC to a string.
            
        Inputs:
            index - Index in the water level attribue
            
        Outputs:
            Water Level for the desired object
        """
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
    
    def category_nav_aid(self, feat, name):
        """ This function converts the information from the ENC on 
            naviagational aids to something humans can understand.
            
        Inputs:
            feat - Feature for which we want to find the category of 
                    naviagational aid
            name - ENC Layer name
            
        Output:
            nav_aid - Readable information on naviagational aids
        """
        # BOYISD --> 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
        # BOYLAT --> 11 - Buoy Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern 
        # BOYSPP --> 11 - Buoy Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern
        # BOYSAW --> 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
        # BCNLAT --> 11 - Beacon Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
        # BCNSPP --> 11 - Beacon Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
        # LIGHTS --> 11- Category of Light, 16 - Height, 23 - Light Orientation
        if name == 'LIGHTS':
            nav_aid = self.category_lights(feat)
            nav_aid += ' Light'
            
        elif name == 'BOYSPP':
            nav_aid = self.obj_color(feat.GetField(13)) + ' '
            nav_aid += self.category_SPP(feat)
            nav_aid += ' Buoy'
            
        elif name == 'BOYISD':
            nav_aid = self.obj_color(feat.GetField(12)) + ' '
            nav_aid += 'Isolated Danger Buoy'
            
        elif name == 'BOYSAW':
            nav_aid = self.obj_color(feat.GetField(12)) + ' '
            nav_aid += 'Safe Water Bouy'
            
        elif name == 'BOYLAT':
            nav_aid = self.obj_color(feat.GetField(13)) + ' '
            nav_aid += self.category_LAT(feat)
            nav_aid += ' Buoy'
            
        elif name == 'BCNSPP':
            nav_aid = self.obj_color(feat.GetField(13)) + ' '
            nav_aid += self.category_SPP(feat)
            nav_aid += ' Buoy'
            
        elif name == 'BCNLAT':
            nav_aid = self.obj_color(feat.GetField(13)) + ' '
            nav_aid += self.category_LAT(feat)
            nav_aid += ' Buoy'
            
        else:
            nav_aid = 'Unknown Navigational Aid'
            
        return nav_aid
    
    def category_underwater(self, feat, name):
        # UWTROC --> 20 - Value of sounding, 22 - WL
        # WRECKS --> 11 - Catagory of wreck, 22 - Value of the Sounding, 26 - WL
        """ This function converts the information from the ENC on underwater 
            objects to something humans can understand.
            
        Inputs:
            feat - Feature for which we want to find the category of underwater
                    objects
            name - ENC Layer name
            
        Output:
            obj_type - Readable information on underwater objects
        """
        if name == 'UWTROC':
            obj_type = 'Rock'
            z = feat.GetField(20)
            if z is None:
                obj_type += ' that '+self.water_level(str(feat.GetField(22)))
            else:
                obj_type += ' @ '+str(z)+'m (MLLW)'
            
        elif name == 'WRECKS':
            obj_type = self.category_wreck(feat)
            z = feat.GetField(20)
            if z is None:
                obj_type += ' that is ' + self.water_level(str(feat.GetField(26)))
            else:
                obj_type += ' @ '+str(z)+'m  (MLLW)'
            
        else:
            obj_type = 'Unknown Underwater Obstacle'
        
        return obj_type
        
    def build_filter(self, x, y, ASV_head, sensor_type, comms):
        """ This function builds the OGR polygon that corresponds to the field 
            of view of the sensor and prints it to pMarnineViewer
            
        Inputs:
            x - Current x position of the ASV
            y - Current y position of the ASV
            ASV_head - Current heading of the ASV
            sensor_type - Type of obstacles that the sensor can "see" 
                    (Landmarks, Nav_aids, Underwater)
            comms - Communication link to MOOS 
            
        Ouputs:
            poly_filter - polygon that corresponds to the field of view of the 
                    sensor
        """
        # Get both Lat/long
        lon,lat = self.MOOSxy2LonLat(x, y)
            
        # Make sure that the heading is from 0-360
        ASV_head = np.mod(self.ang4MOOS(ASV_head), 360)
        
        # Build polygon for pMarineViewer
        hypot = self.sensor_max_dist/np.cos(self.sensor_FoV_ang)
        ang1 = self.sensor_head+ASV_head+self.sensor_FoV_ang
        ang2 = self.sensor_head+ASV_head-self.sensor_FoV_ang
        
        # Find the sensor FoV polygon as it rotates
        A_x = x
        A_y = y
        B_x = x+hypot*np.cos(ang1*np.pi/180)
        B_y = y+hypot*np.sin(ang1*np.pi/180)
        C_x = x+hypot*np.cos(ang2*np.pi/180)
        C_y = y+hypot*np.sin(ang2*np.pi/180)
        
        # Print it to pMarnineViewer
        if sensor_type == 'Underwater':
            color = 'edge_color=blue'
        elif sensor_type == 'Landmark':
            color = 'edge_color=salmon'
        elif sensor_type == 'Nav_aid':
            color = 'edge_color=red'
            
        FoV_Filter = 'pts={'+str(A_x)+','+str(A_y)+':'+str(B_x)+','+str(B_y)+\
            ':'+str(C_x)+','+str(C_y)+'},label=Sensor_FoV_'+sensor_type+','+color
            
        comms.notify('VIEW_POLYGON', FoV_Filter, pymoos.time()) 
        
        # Initialize filers
        ring_filter = ogr.Geometry(ogr.wkbLinearRing)
        poly_filter = ogr.Geometry(ogr.wkbPolygon)
        
        # Build Lat/Long filter
        B_lon,B_lat = self.MOOSxy2LonLat(B_x, B_y)
        C_lon,C_lat = self.MOOSxy2LonLat(C_x, C_y)
        ring_filter.AddPoint(lon,lat)
        ring_filter.AddPoint(B_lon,B_lat)
        ring_filter.AddPoint(C_lon,C_lat)
        ring_filter.CloseRings()
        poly_filter.AddGeometry(ring_filter)
        
        return poly_filter
        
    def Landmarks(self, X, Y, heading, layers, comms):
        """ This function searchs the ENCs for the 5 closest landmarks in the 
            sensors field of view, highlights them in pMarnineViewer, and then 
            posts the position and current distance to each landamark to the
            MOOSDB.
            
        Inputs:
            x - Current x position of the ASV
            y - Current y position of the ASV
            heading - Current heading of the ASV
            layers - The layers in the ENC that will be searched for landmarks
            comms - Communication link to MOOS
        """ 
        # Build the spatical filter
        poly_filter = self.build_filter(X, Y, heading, 'Landmark', comms)
        
        obj1, dist, xpos_l, ypos_l = [],[],[],[]
        landmark_objects = ''        
        
        for i in range(len(layers)):
            layer = self.ds.GetLayerByName(layers[i])
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
                obj_type = self.category_landmark(feat,layers[i])
                pt_x, pt_y = self.LonLat2MOOSxy(pt_lon,pt_lat)
                
                xpos_l.append(pt_x)
                ypos_l.append(pt_y)
                
                # Calc dist to Landmark
                d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                dist.append(d)
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
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_l[ii])+'\
                    ,y='+str(ypos_l[ii])+',radius=30,pts=5,edge_color=pink,label=landmark_'+str(cntr))
                if cntr < 4:
                    landmark_objects += '!'
                cntr += 1
        else:
            for ij in range(num_landmarks):
                landmark_objects += obj1[ij]
                time.sleep(.002)
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_l[ij])+\
                    ',y='+str(ypos_l[ij])+',radius=30,pts=5,edge_color=pink,label=landmark_'+str(ij))
                if ij < num_landmarks-1:
                    landmark_objects += '!'
            for j in range(num_landmarks, 5):
                time.sleep(.002)
                poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=landmark_'+str(j)
                comms.notify('VIEW_POLYGON', poly)
        if landmark_objects != '':
            comms.notify('Landmarks', landmark_objects)
        layer.SetSpatialFilter(None)
    
    def Underwater_Objs(self, X, Y, heading, layers, comms):
        """ This function searchs the ENCs for the 5 closest underwater objects 
            in the sensors field of view, highlights them in pMarnineViewer, 
            and then posts the position and current distance to each underwater  
            objects to the MOOSDB.
            
        Inputs:
            x - Current x position of the ASV
            y - Current y position of the ASV
            heading - Current heading of the ASV
            layers - The layers in the ENC that will be searched for underwater
                    objects
            comms - Communication link to MOOS
        """ 
        ## Underwater Objects
        # Build the spatical filter
        poly_filter = self. build_filter(X, Y, heading, 'Underwater', comms)
        
        obj3, dist, xpos_u, ypos_u = [],[],[],[]
        underwater_objects = ''
        # Cycle through the Underwater Objects
        for i in range(len(layers)):
            layer = self.ds.GetLayerByName(layers[i])
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
                obj_type = self.category_underwater(feat, layers[i])
                pt_x, pt_y = self.LonLat2MOOSxy(pt_lon,pt_lat)
                
                xpos_u.append(pt_x)
                ypos_u.append(pt_y)
                # Calc dist to Landmark
                d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                
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
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_u[ii])+\
                    ',y='+str(ypos_u[ii])+',radius=30,pts=5,edge_color=deepskyblue ,label=underwater_'+str(cntr))                  
                time.sleep(.002)
                if cntr < 4:
                    underwater_objects += '!'
                cntr += 1
        else:
            for ij in range(num_underwater):
                underwater_objects += obj3[ij]
                time.sleep(.002)
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_u[ij])+'\
                    ,y='+str(ypos_u[ij])+',radius=30,pts=5,edge_color=deepskyblue ,label=underwater_'+str(ij))
                if ij < num_underwater-1:
                    underwater_objects += '!'
            for j in range(num_underwater, 5):
                time.sleep(.002)
                poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=underwater_'+str(j)
                comms.notify('VIEW_POLYGON', poly)
        if underwater_objects != '':
            comms.notify('Underwater_Objects', underwater_objects)
        layer.SetSpatialFilter(None)
    
    def Nav_Aids(self, X, Y, heading, layers, comms):
        """ This function searchs the ENCs for the 5 closest navigational aids 
            in the sensors field of view, highlights them in pMarnineViewer, 
            and then posts the position and current distance to each   
            navigational aids to the MOOSDB.
            
        Inputs:
            x - Current x position of the ASV
            y - Current y position of the ASV
            heading - Current heading of the ASV
            layers - The layers in the ENC that will be searched for 
                    navigational aids
            comms - Communication link to MOOS
        """ 
        ## Navigational Aids
        # Build the spatical filter
        poly_filter = self.build_filter(X, Y, heading, 'Nav_aid', comms)
        
        obj2, dist, xpos_na, ypos_na = [],[],[],[]
        nav_aid_objects = ''
        # Cycle through the Navigational aids
        for i in range(len(layers)):
            layer = self.ds.GetLayerByName(layers[i])
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
                    
                obj_type = self.category_nav_aid(feat, layers[i])
                pt_x, pt_y = self.LonLat2MOOSxy(pt_lon,pt_lat)
                
                if layers[i] == 'BOYLAT':
                    print pt_x, pt_y
                
                xpos_na.append(pt_x)
                ypos_na.append(pt_y)
                # Calc dist to Landmark
                d = np.sqrt(np.power(pt_x-X,2)+np.power(pt_y-Y,2))
                
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
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_na[ii])+\
                    ',y='+str(ypos_na[ii])+',radius=30,pts=5,edge_color=darkorange,label=nav_aid_'+str(cntr))
                if cntr < 4:
                    nav_aid_objects += '!'
                cntr += 1
        else:
            for ij in range(num_nav_aid):
                nav_aid_objects += obj2[ij]
                time.sleep(.002)
                comms.notify('VIEW_POLYGON', 'format=radial,x='+str(xpos_na[ij])+\
                    ',y='+str(ypos_na[ij])+',radius=30,pts=5,edge_color=darkorange,label=nav_aid_'+str(ij))
                if ij < num_nav_aid-1:
                    nav_aid_objects += '!'
            for j in range(num_nav_aid, 5):
                time.sleep(.002)
                poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label=nav_aid_'+str(j)
                comms.notify('VIEW_POLYGON', poly)
        if nav_aid_objects != '':
            comms.notify('Nav_Aids', nav_aid_objects)
        layer.SetSpatialFilter(None)
    
    def sensor(self, X, Y, heading, comms):
        """ This chooses the correct function to determine what is in the 
            sensor field of view based on the previously declared obstacles 
            that can be "seen" by the sensor.
            
        Inputs:
            x - Current x position of the ASV
            y - Current y position of the ASV
            heading - Current heading of the ASV
            comms - Communication link to MOOS
        """
        
        Landmarks_layers = ['LNDMRK', 'SILTNK']
        Underwater_layers = ['UWTROC', 'WRECKS']
        Nav_aids_layers =  ['LIGHTS', 'BOYSPP', 'BOYISD', 'BOYSAW', 'BOYLAT', 'BCNSPP', 'BCNLAT']
        
        for i in range(len(self.obs_type)):
            if (self.obs_type[i] == "landmark"):
                self.Landmarks(X, Y, heading, Landmarks_layers, comms)
            elif (self.obs_type[i] == "underwater"):   
                self.Underwater_Objs(X, Y, heading, Underwater_layers, comms)
            if (self.obs_type[i] == "nav_aid"):
                self.Nav_Aids(X, Y, heading, Nav_aids_layers, comms)
    
    def run(self):
        """ """
        # Initialize the MOOS connection
        MOOS = MOOS_comms()
        MOOS.Initialize()
        
        # Run the actual loop
        while(True):
            # Get the new values for the ASV's current position (x, y) and 
            #   heading.
            time.sleep(.001)
            MOOS.Get_mail()
            
            # If there is a new position and heading for the ASV, then filter  
            #   the data and highlight the ones in the search area
            if len(MOOS.NAV_X) != 0 and len(MOOS.NAV_Y)!= 0 and len(MOOS.NAV_HEAD)!=0:
                # Get most recent value of position and heading and then reset 
                #   the lists
                X = MOOS.NAV_X[-1]
                Y = MOOS.NAV_Y[-1]
                heading = MOOS.NAV_HEAD[-1]
                MOOS.NAV_X, MOOS.NAV_Y, MOOS.NAV_HEAD = [],[],[]
                self.sensor(X, Y, heading, MOOS.comms)
                
            # MOOS freaks out when nothing is posted to the MOOSDB so post this 
            #   variable to avoid this problem if nothing was posted during the
            #   last cycle
            else:
                MOOS.comms.notify('dummy_var','')

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#               
SFoV = ENC_SFoV()
SFoV.run()