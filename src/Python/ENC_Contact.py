# -*- coding: utf-8 -*-
"""
Created on Thu Oct  6 16:47:16 2016

@author: mapper
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
    This application has two main parts and initialization and a infinate run 
loop. In the initialization, the program takes the information from the ENCs 
and stores it to disk. Once that has happened, it then runs the infinate loop.
In the infinate loop, a search polygon is built and the objects from the ENC 
that are within the search area are then published to the MOOSDB.
"""

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

# Used to check if the file already exists
from os import path

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# Numpy is a useful tool for mathematical operations 
import numpy as np


class MOOS(object):
    """ This class id for MOOS communications. It has 3 parts:
          1. Initialize comms
              a. Register for variables
              b. Connect to the server
          2. Get new values.
    """
#    def __init__(self):
#    """ Open a communication link to MOOS and initialize the list for X, Y, and
#    heading."""
#        self.comms = pymoos.comms()
#    
#        # Placeholder for the list of varables that we want
#        self.NAV_X, self.NAV_Y, self.NAV_HEAD = [],[],[]
    
    # Open a communication link to MOOS
    comms = pymoos.comms()
    
    # Placeholder for the list of varables that we want
    NAV_X, NAV_Y, NAV_HEAD = [],[],[]

    def Register_Vars(self):
        """ This function registers for the updates of the X,Y and heading at 
        the rate of which it is being outputed from MOOS. """
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('NAV_HEADING', 0)
        return True
        
    def Initialize_MOOS_comms(self):
        """ This function registers for the current X,Y, and heading and then
        connects to the server"""
        
        # Register for desired variables
        self.comms.set_on_connect_callback(self.Register_Vars())
        
        # Connect to the server
        self.comms.run('localhost',9000,'Contact')
        
    def Get_mail(self):
        ## Update the values of the ASV position in meters (x,y) and heading
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

class ENC_OA(object):
    """This application has two main parts: initialization and an infinate  
        run loop. In the initialization, the program takes the information from 
        the ENCs and stores it to disk. Once that has happened, it then runs 
        the infinate loop. In the infinate loop, a search polygon is built and 
        the objects from the ENC that are within the search area are then 
        published to the MOOSDB."""

    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    def __init__(self, LatOrigin=43.071959194444446, LongOrigin=-70.711610833333339):
        """ Initialize varibles. """
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        self.search_area_poly = ogr.Geometry(ogr.wkbPolygon)
        
    
    def ang4MOOS(self, angle):
        """ This fucntion converts a typical angle to one that is 0 degrees at
        North"""
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
        
    def calc_t_lvl(self, WL, depth):
        """ This function uses the water level and depth attributes for a  
            feature and calculates the threat level for that obstacle.
            
            Inputs:
                depth - Recorded depth for the obstacle
                        (quantitative depth measurement)
                WL - Water level for the obstacle 
                        (qualitative depth measurement)
                
            Outputs:
                threat level - Calculated threat level"""
        # Obstacle Partly submerged at high water, Always Dry, Awash, Floating, or 0<=Z<=3
        if (WL == 1 or WL == 2 or WL == 5 or WL == 7 or depth <= 0):
            return 4
        # Obstacle Covers and uncovers, Subject to inundation or flooding, or 0<Z<3
        elif (WL == 4 or WL == 6 or depth < 1.5):
            return 3
        # Obstacle is alway below surface
        elif (WL == 3):
            # 3<=Z<6 or depth is unknown (9999)
            if (depth < 3 or depth == 9999):
                return 2
            # 6<=Z<10
            elif (depth >=3 and depth <= 5):
                return 1
            # Z > 10
            else:
                return 0
                
    # Converts the number stored in Category of Lights to string    
    def category_lights(self, feat):
        """ This function takes in a light feature and converts the number 
        stored in category attribute to string.
        
        Inputs: 
            feat - Feature from the light layer of an ENC
            
        Outputs:
            A string that holds the information from the category of lights 
                attribute from the ENC
        """
        
        index = str(feat.GetField(12))
        if index == '1':
            return 'Directional Function'
        # Index 2 and 3 are not used
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
        if index == '10':
            return 'Subsidiary'
        if index == '11':
            return 'Spot'
        if index == '12':
            return 'Front'
        if index == '13':
            return 'Rear'
        if index == '14':
            return 'Lower'
        if index == '15':
            return 'Upper'
        if index == '16':
            return 'Moire Effect'
        if index == '17':
            return 'Emergency'
        if index == '18':
            return 'Bearing'
        if index == '19':
            return 'Horizontally Disposed'
        elif index =='20':
            return 'Vertically Disposed'
        else:
            return 'Marine'

    # Converts the number stored in category of Landmark or Silo/Tank to a string
    def category_landmark(self, feat, name):
        """ This function takes in a landmark or Silo/Tank feature and converts
        the number stored in category attribute to string.
        
        Inputs: 
            feat - Feature from the light layer of an ENC
            name - Layer name (either Landmark or Silo/Tank)
            
        Outputs:
            A string that holds the information from the category of landmark 
                attribute from the ENC
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
            else:
                return 'Unknown'
    
    def BuildLayers(self, ENC_filename='../ENCs/US5NH02M/US5NH02M.000', filename_pnt='../ENCs/US5NH02M/Shape/point.shp', filename_poly='../ENCs/US5NH02M/Shape/poly.shp', filename_line='../ENCs/US5NH02M/Shape/line.shp'):
        """ Create a OGR layer for the point obstacles, polygon obstacles and  
        the line obstacles.
        
        Inputs:
            ENC_filename - path to ENC file
            out_filename - path to the file needed so that we can reorganize 
                            the ENC to a shapefile
        
        Outputs (these are not returned, but defined as self.):
            ENC_point_layer - OGR layer that holds all the information from the 
                                ENC that have point geometry
            ENC_poly_layer - OGR layer that holds all the information from the 
                                ENC that have polygon geometry
            ENC_line_layer - OGR layer that holds all the information from the 
                                ENC that have line geometry:
            ds - Data source for the ENC 
        """
        # Get the S-57 driver and open the ENC
        self.ds = ogr.Open(ENC_filename)
        
        # Use the Shapefile driver for gdal
        shape_driver = ogr.GetDriverByName('Esri Shapefile')
        
        ## Point Output File
        # If there already is a file with that name, delete it
        if path.exists(filename_pnt):
            shape_driver.DeleteDataSource(filename_pnt)
        
        # create an output file
        ds_pnt = shape_driver.CreateDataSource(filename_pnt)
        
        ## Polygon Output File
        # If there already is a file with that name, delete it
        if path.exists(filename_poly):
            shape_driver.DeleteDataSource(filename_poly)
        
        # create an output file
        ds_poly = shape_driver.CreateDataSource(filename_poly)
        
        ## Line Output File
        # If there already is a file with that name, delete it
        if path.exists(filename_pnt):
            shape_driver.DeleteDataSource(filename_line)
        
        # create an output file
        ds_line = shape_driver.CreateDataSource(filename_line)
        
        ## Buld the layers of obstacles seperated by geometry type
        # Create a layer for the point obstacles, polygon obstacles and the 
        #   line obstacles.
        self.ENC_point_layer = ds_pnt.CreateLayer('Points', None, ogr.wkbPoint)
        self.ENC_poly_layer = ds_poly.CreateLayer('Poly', None, ogr.wkbPolygon)
        self.ENC_line_layer = ds_line.CreateLayer('Line', None, ogr.wkbLineString)
        
        # Add the Threat Level and Type Attributes
        self.ENC_point_layer.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
        self.ENC_point_layer.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
        self.ENC_point_layer.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
        self.ENC_point_layer.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
        
        # Add the Threat Level and Type Attributes
        self.ENC_poly_layer.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
        self.ENC_poly_layer.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
        self.ENC_poly_layer.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
        self.ENC_poly_layer.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
        
        # Add the Threat Level and Type Attributes
        self.ENC_line_layer.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
        self.ENC_line_layer.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
        self.ENC_line_layer.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
        self.ENC_line_layer.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
        
    # Layers that are Multipoints --> SOUNDG
    def LayerMultiPoint (self, LayerName_mp):
        """ Adds the features from the inputed multipoints layer to a point 
        layer.
        
        Inputs:
            LayerName_mp - Name of the multipoint layer
        """
        
        layer_mp = self.ds.GetLayerByName(LayerName_mp)
        feat_mp = layer_mp.GetNextFeature()
        
        while feat_mp is not None:
            multi_geom = feat_mp.GetGeometryRef()
            for iPnt in range(multi_geom.GetGeometryCount()):
                # Create point
                point = ogr.Geometry(ogr.wkbPoint)
                pnt = multi_geom.GetGeometryRef(iPnt)
                point.SetPoint_2D(0,pnt.GetX(),pnt.GetY())
    
                # Create a new feature (attribute and geometry)
                defn_pnt = self.ENC_point_layer.GetLayerDefn()
                new_feat = ogr.Feature(defn_pnt)
                new_feat.SetField('T_lvl', self.calc_t_lvl(pnt.GetZ(),0))
                new_feat.SetField('Type', LayerName_mp)
        
                # Make a geometry for the new feature from Shapely object
                new_feat.SetGeometry(point)
    
                self.ENC_point_layer.CreateFeature(new_feat) 
            feat_mp = layer_mp.GetNextFeature()
    
            
    # Converts the ENC File to 3 different shape files    
    def ENC_Converter(self, LayerName):
        """ """        
        layer = self.ds.GetLayerByName(LayerName)
        layer_def = layer.GetLayerDefn() # needed to find the name of the fields
        feat = layer.GetNextFeature()
        
        while feat is not None:
            geom = feat.GetGeometryRef() # Needed for determining the geom type and to export the object to wkt
            geo_name = geom.GetGeometryName()    
            
            # Export this feature to WTK for easy conversion to Shapefile
            wkt = geom.ExportToWkt()
        
            ## Initialize WL, depth, and threat level
            WL = 0
            depth = 9999
            t_lvl = 0
            
            ## Cycle through the fields and store the ones that are useful
            for i in range(feat.GetFieldCount()):
                name = layer_def.GetFieldDefn(i).GetName()
                if name == 'WATLEV':
                    WL = int(feat.GetField(i))
                elif name == 'VALSOU':
                    depth = feat.GetField(i)
                    if depth is None:
                        depth = 9999
                elif name == 'VALDCO': # value of the depth contour
                    depth = feat.GetField(i)
                    if depth is None:
                        depth = 9999
            
            ## Update Threat Level
            # If WL has been updated use the function calc_t_lvl to calculate threat level
            if WL != 0 or LayerName == 'DEPCNT':
                t_lvl =  self.calc_t_lvl(WL, depth)
            # If it is Land set threat level to 5
            elif LayerName == 'LNDARE' or LayerName == 'DYKCON' or LayerName == 'PONTON' or LayerName == 'COALNE':
                t_lvl = 5
            elif LayerName == 'LIGHTS':
                t_lvl = -2
    #            print str(geom.GetX())+', '+str(geom.GetY())
    #            print feat.GetField(35)
            # If it is a Buoy, Light or Beacon set threat level to 3
            elif LayerName == 'BOYISD' or LayerName == 'BOYSPP' or LayerName == 'BOYSAW' or LayerName == 'BOYLAT' or LayerName == 'BCNSPP' or LayerName == 'BCNLAT':
                t_lvl = 3
            elif LayerName == 'LNDMRK':
                t_lvl = -1
                
            out_layer = None
            # Choose the correct layer for the 
            if geo_name == 'POINT':
                out_layer = self.ENC_point_layer
                obj = ogr.Geometry(ogr.wkbPoint)
            elif geo_name == 'POLYGON':
                out_layer = self.ENC_poly_layer
                obj = ogr.Geometry(ogr.wkbPolygon)
            elif geo_name == 'LINESTRING':
                out_layer = self.ENC_line_layer
                obj = ogr.Geometry(ogr.wkbLineString)
                
            # Create a new feature (attribute and geometry)
            defn = out_layer.GetLayerDefn()
            new_feat = ogr.Feature(defn)
            new_feat.SetField('T_lvl', t_lvl)
            new_feat.SetField('Type', LayerName)
            
            if LayerName == 'LNDMRK':
                new_feat.SetField('Cat', self.category_landmark(feat,LayerName)) # Category - store as string and not a number
                new_feat.SetField('Visual', 2-feat.GetField(16)) # Visually Conspicuous (Y - store as 1, N - store as 0)
            elif LayerName == 'SILTNK':
                new_feat.SetField('Cat', self.category_landmark(feat,LayerName)) # Category - store as string and not a #
                new_feat.SetField('Visual', 2-feat.GetField(17)) # Visually Conspicuous (Y - store as 1, N - store as 0)
            elif LayerName == 'LIGHTS':
                self.category_lights(feat)
            # Make a geometry from wkt object
            obj = ogr.CreateGeometryFromWkt(wkt)
            new_feat.SetGeometry(obj)
    
            # Output the new feature to the correct layer
            if geo_name == 'POINT':
                self.ENC_point_layer.CreateFeature(new_feat)
                
            elif geo_name == 'POLYGON':
                self.ENC_poly_layer.CreateFeature(new_feat)
                
            elif geo_name == 'LINESTRING':
                self.ENC_line_layer.CreateFeature(new_feat)
            
            # Get the next feature
            feat = layer.GetNextFeature()

    
    def read_ENC(self, ENC_filename='../ENCs/US5NH02M/US5NH02M.000', filename_pnt='../ENCs/US5NH02M/Shape/point.shp', filename_poly='../ENCs/US5NH02M/Shape/poly.shp', filename_line='../ENCs/US5NH02M/Shape/line.shp'):
        """ This converts the ENC into OGR layers by geometry types. This 
        information will then be used by the all other functions in this 
        class.
        
        Inputs:
            ENC_filename - path to the ENC file
            filename_* - path to the file needed so that we can reorganize 
                            the ENC to shapefiles by geometry type 
            
        Outputs:
            ENC_point_layer - OGR layer that holds all the information from the 
                                ENC that have point geometry
            ENC_poly_layer - OGR layer that holds all the information from the 
                                ENC that have polygon geometry
            ENC_line_layer - OGR layer that holds all the information from the 
                                ENC that have line geometry
        """
        
        # Create a OGR layer for the point obstacles, polygon obstacles and  
        #   the line obstacles. 
        self.BuildLayers(ENC_filename, filename_pnt, filename_poly, filename_line)
        
        # Layers that are only points --> UWTROC, LIGHTS, BOYSPP, BOYISD, 
        #   BOYSAW, BOYLAT, BCNSPP, BCNLAT
        self.ENC_Converter('UWTROC')
        self.ENC_Converter('LIGHTS')
        self.ENC_Converter('BOYSPP')
        self.ENC_Converter('BOYISD')
        self.ENC_Converter('BOYSAW')
        self.ENC_Converter('BOYLAT')
        self.ENC_Converter('BCNSPP')
        self.ENC_Converter('BCNLAT')
        
        # Add the features from the SOUNDG multipoint layer to the point layer
        self.LayerMultiPoint('SOUNDG')
        
        # Layers that have multiple types --> WRECKS, OBSTRN, LNDARE, DEPARE, 
        #   PONTON, DEPCNT, DYKCON
        self.ENC_Converter('WRECKS')
        self.ENC_Converter('LNDARE')
        self.ENC_Converter('PONTON')
        self.ENC_Converter('DEPCNT')
        self.ENC_Converter('DYKCON')
        self.ENC_Converter('LNDMRK')
        self.ENC_Converter('SILTNK')
        
        
        return self.ENC_point_layer, self.ENC_poly_layer, self.ENC_line_layer
        
    def build_search_poly(self, X, Y, heading, search_dist):
        """ This function builds the polygon which represents the area that
        the ASV is going to search the ENC for potential threats. The shape of
        the search area is a square that is centered on the ASV's current X,Y
        location. It also prints the search area polygon to pMarnine Viewer.
        
        Inputs: 
            X - Current X position of the ASV
            Y - Current Y position of the ASV
            heading - Current heading of the ASV
            search_dist - length of each side of the seach polygon
        
        Outputs:
            search_area_poly - OGR Polygon that describes the area in which we 
                            want to seach the input layer for objects
        """
        # Clear previous values from ring and poly and remove the old
        #   spatial filter
        self.layer_pnt.SetSpatialFilter(None)
        self.layer_poly.SetSpatialFilter(None)
        search_area_ring = None
        self.search_area_poly = None
            
        # Create the baseline for the search area polygon 
        search_area_ring = ogr.Geometry(ogr.wkbLinearRing)
        self.search_area_poly = ogr.Geometry(ogr.wkbPolygon)
        
        # Calculate the correct corrections to have the search area spin 
        #   rotate with respect to the ASV's heading
        theta = 45-np.mod(self.ang4MOOS(heading), 90) # Degrees
        add_sin = search_dist*np.sqrt(2)*np.sin(theta*np.pi/180)
        add_cos = search_dist*np.sqrt(2)*np.cos(theta*np.pi/180)
        
        # Convert the positions to latitude and longitude
        pt1_lon,pt1_lat = self.MOOSxy2LonLat(X+add_sin, Y+add_cos)
        pt2_lon,pt2_lat = self.MOOSxy2LonLat(X-add_cos, Y+add_sin)
        pt3_lon,pt3_lat = self.MOOSxy2LonLat(X-add_sin, Y-add_cos)
        pt4_lon,pt4_lat = self.MOOSxy2LonLat(X+add_cos, Y-add_sin)
        
        # Build a ring of the points for the search area polygon
        search_area_ring.AddPoint(pt1_lon, pt1_lat)
        search_area_ring.AddPoint(pt2_lon, pt2_lat)
        search_area_ring.AddPoint(pt3_lon, pt3_lat)
        search_area_ring.AddPoint(pt4_lon, pt4_lat)
        search_area_ring.CloseRings()
        self.search_area_poly.AddGeometry(search_area_ring) # Add the ring to the previously created polygon         
        
        # Show the Search Radius Polygon on pMarnineViewer
        s_poly_vert = 'pts={'+str(X+add_sin)+','+ str(Y+add_cos)+':'+ str(X-add_cos)+','+str(Y+add_sin)+':'+ str(X-add_sin)+','+str(Y-add_cos)+':' +str(X+add_cos)+','+str(Y-add_sin)+'},'
        self.comms.notify('VIEW_POLYGON', s_poly_vert+'label=Search,edge_size=10,vertex_size=1,edge_color=red',)
        
        
    def filter_features(self, input_layer, search_area):
        """ This function uses OGR's filter commands to limit the objects in 
        the OGR layer to the search area.
        
        Inputs:
            input_layer - Layer from the read_ENC function that is to be 
                            filtered to only contain objects from the search 
                            area
            seach_area  - OGR Polygon that describes the area in which we want
                            to seach the input layer for objects
            
        Outputs:
            filted_layer  - New OGR layer that consists of the objects from the 
                        passed layer that reside within the search area.
        """
        filted_layer = input_layer
        
        # Filter out data to determine the search area
        filted_layer.SetSpatialFilter(input_layer)
        filted_layer.SetAttributeFilter("t_lvl>0")
        
        return filted_layer
        
    def publish_points(self, X, Y, heading):
        """ This function filters the point layer and then cycles through that 
            layer of point obstacles from the ENC to highlight them in
            pMarnineViewer and publish the x,y position, threat level and 
            obstacle type to the MOOSDB. At the end, it checks to see if any of 
            the highlighted obstacles are no longer within the search and if 
            there are any wrongly highlighted obstacles, it removes them."""
        
            
        # This counter is used to count the number of features in the 
        #   search radius. This is then used to determine if any of the old
        #   hazard polygons need to be removed.
        counter = 1
        
        # Counter to determine how many obstacles that are above threat 
        #   level 0 in the search radius
        num_obs = 0
        
        # Intialize the obstacle string
        obs_pos = ''
        
        # Counter to remove the highlighted obstacles from pMarnineViewer that
        #   are not within the search area.
        max_cntr = 1
        
        # Filter the layer that holds the point obstacles
        layer_point = self.filter_features(self.layer_pnt, self.search_area_poly)
        
        # Highlight all point features in search radius in pMarineViewer 
        #   and store them into a MOOS variable called "Obstacles"
        feature = layer_point.GetNextFeature() 
        while feature:
            time.sleep(.001)
            geom_point = feature.GetGeometryRef()
            new_x, new_y = self.LonLat2MOOSxy(geom_point.GetX(), geom_point.GetY())
            
            pos = 'x='+str(new_x)+',y='+str(new_y)
            highlight_obs = 'format=radial,'+pos+',radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label='+str(counter)
            self.comms.notify('VIEW_POLYGON', highlight_obs)
            
            # Store information for the obstacle to be used later to post to the MOOSDB
            # x,y,t_lvl,type 
            if feature.GetField(0) !=0:
                # if it isnt the first obstacle put a ! at the end
                if num_obs!=0:
                    obs_pos += '!'
                num_obs += 1
                obs_pos += pos+','+ str(feature.GetField(0))+','+ str(feature.GetField(1))

            # Go to the next feature and increment the counter
            feature = layer_point.GetNextFeature() 
            counter += 1
            
        # Output to the MOOSDB a list of obstacles
        #   ASV_X,ASV_Y : # of Obstacles : x,y,t_lvl,type : x,y,t_lvl,type : ...
        obstacles = str(X)+','+str(Y)+','+str(heading)+':'+str(num_obs)+':'+obs_pos
        self.comms.notify('Obstacles', obstacles)     
        
        # Determine if a new polygon was used
        if max_cntr < counter:
            max_cntr = counter  
            
        # Remove highlighted point obstacles (shown as polygons) from 
        #   pMarineViewer if they are outside of the search area
        for i in range(counter, max_cntr):
            time.sleep(.002)
            remove_highlight = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label='+str(i)
            self.comms.notify('VIEW_POLYGON', remove_highlight)
            
    def publish_poly(self):
        """ """
    def verify_poly(self):
        """ """
    def run(self):
        """ """
    def initialize(self):
        """ """
        self.read_ENC()

        
e = ENC_OA()
e.BuildLayers()
e.LayerMultiPoint('SOUNDG')
#e.ENC_Converter('UWTROC')
