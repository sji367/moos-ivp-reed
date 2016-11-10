# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 20:09:11 2016

@author: mapper
"""

import json

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
              a. Set the Timewarp
              b. Register for variables
              c. Connect to the server
          2. Get new values.
    """
    
    def __init__(self):
        # Open a communication link to MOOS
        self.comms = pymoos.comms()

    def Register_Vars(self):
        """ We dont need to register for variables.
        """
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
        self.comms.run('localhost',9000,'np_json')
        
    def Get_mail(self):
        """ We don't need to get mail. """
        pass

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#       
class np_json(object):
    """ This application takes the information from the ENC that has previously  
        been sorted into shapefiles by object geometry type and prints it to 
        pMarnineViewer.
    """
    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    def __init__(self, LatOrigin=43.071959194444446, 
                 LongOrigin=-70.711610833333339,
                 filename_pnt='../../src/ENCs/US5NH02M/Shape/point.shp', 
                 filename_poly='../../src/ENCs/US5NH02M/Shape/poly.shp', 
                 filename_line='../../src/ENCs/US5NH02M/Shape/line.shp'):
        """ Initialize varibles. 
            
        Inputs:
            LatOrigin - Latitude origin for the pyproj conversion to UTM
            LongOrigin - Longitude origin for the pyproj conversion to UTM
            filename_* - path to the shapefiles which are organized by geometry
                         type 
        """
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        
        # Find the point, polygon and line layers
        self.Get_layers(filename_pnt, filename_poly, filename_line)        
        
        # Open a communication link to MOOS
#        self.init_MOOS()
    
    def init_MOOS(self):
        """ Initializes the communication link to MOOS. 
        
        Ouputs:
            self.comms - Communication link to MOOS
        """
        MOOS = MOOS_comms()
        MOOS.Initialize()
        # Need this time to connect to MOOS
        time.sleep(.11)
        self.comms = MOOS.comms
        
    def Get_layers(self, filename_pnt, filename_poly, filename_line):
        """ This function defines the layers for the points, polygon and lines.
            It also can filter the layers spatially and to not contain objects 
            with threat level 0.
        
        Inputs:
            filename_* - path to the shapefiles which are organized by geometry
                         type 
        
        Outputs:
            self.ENC_point_layer- OGR layer that holds all the information from
                                 the ENC that have point geometry
            self.ENC_poly_layer - OGR layer that holds all the information from
                                 the ENC that have polygon geometry
            self.ENC_line_layer - OGR layer that holds all the information from 
                                 the ENC that have line geometry
        """
        driver = ogr.GetDriverByName('ESRI Shapefile')
        # Get the Datasourse
        self.ds_pnt = driver.Open(filename_pnt, 0)
        self.ds_poly = driver.Open(filename_poly, 0)
        self.ds_line = driver.Open(filename_line, 0)
        
        self.ENC_point_layer = self.ds_pnt.GetLayer()
        self.ENC_poly_layer = self.ds_poly.GetLayer()
        self.ENC_line_layer = self.ds_line.GetLayer()
        
    
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
        
    def ang4MOOS(self, angle):
        """ This fucntion converts a typical angle to one that is 0 degrees at
        North
        """
        return np.mod(-(angle-90),360)
    
#    def convert2np_wtk(self, feat):
#        """ This function takes in a ogr feature and converts its vertices to a
#            2D numpy array 
#        
#        Inputs:
#            feat - Feature that we want its vertices
#            
#        Outputs:
#            array - Numpy array of the feature's vertices
#        """
#        wkt = feat.GetGeometryRef().ExportToWkt()
#        s = wkt.split('((')[1].split(',')
#        s[-1]=s[-1].split('))')[0]
#        array = np.empty((len(s),2))
#        for i in range(len(s)):
#            temp = s[i].split(' ')
#            array[i] = np.asarray(temp, dtype=np.float32)
#            
#        return array
    
    def convert2np(self, feat):
        """ This function takes in a ogr feature and converts its vertices to a
            2D numpy array 
        
        Inputs:
            feat - Feature that we want its vertices
            
        Outputs:
            np_array - Numpy array of the feature's vertices
        """
        name = feat.GetGeometryRef().GetGeometryName()   
        
        # Get json representation of the feature
        feat_json = json.loads(feat.ExportToJson())
        
        # Convert json representation to list of vertices
        if name == 'POLYGON':
            vertices = feat_json["geometry"]['coordinates'][0]
        else:
            vertices = feat_json["geometry"]['coordinates']
        
        # Convert that list to a numpy array and return it
        np_array = np.asarray(vertices)
        
        return np_array
        
    def find_crit_pnts(self, feat, ASV_X, ASV_Y):
        """ This function finds the maximum angular extent of the obstacle with
            respect to the ASV and the vertex of the obstacle that is closest 
            to the ASV.
            
        Inputs:
            feat - OGR feature of the obstacle
            ASV_X - X position of the ASV
            ASV_Y - Y position of the ASV
            
        Ouputs:
            min_ang - Minimum angle with respect to the ASV
            max_ang - 
            min_dist - 
        """
        # Get the vertices of the polygon
        vertices_latlon = self.convert2np(feat)
        
        # Convert the vertices from lat/long to X/Y
        vertices_x, vertices_y = self.LonLat2MOOSxy(vertices_latlon[:,0],vertices_latlon[:,1])
        
        # Convert the position of the ASV to a matrix
        pos = np.ones_like(vertices_x)
        ASV_x = ASV_X*pos
        ASV_y = ASV_Y*pos
        
        # Calculte the distance and angle from the ASV to the polygon
        dist2poly = np.sqrt(np.square(ASV_x-vertices_x)+np.square(ASV_y-vertices_y))
        angle2poly = np.arctan2(ASV_y-vertices_y, ASV_x-vertices_x)*180/np.pi
        
        # Determine the minimum distance, minimum angle, and maximum angle
        index_min_dist = np.argmin(dist2poly)
        index_min_ang = np.argmin(angle2poly)
        index_max_ang = np.argmax(angle2poly)
        
        # There is one major issue with finding the minimum and maximum angles.
        #   It is that when the angle switches from 360 to 0. This will lead to
        #   incorrect angles being saved as the minimum and maximum. Therefore
        #   if any of the any of the angles are near the cross over point, 
        #   use the domain [0,360] instead of [-180, 180]
        if ((angle2poly[index_min_ang] >150) or (angle2poly[index_max_ang] < -150)):
            angle2poly = np.mod(angle2poly, 360)
            index_min_ang = np.argmin(angle2poly)
            index_max_ang = np.argmax(angle2poly)
        
        min_dist = [vertices_x[index_min_dist], vertices_y[index_min_dist], dist2poly[index_min_dist], angle2poly[index_min_dist]]
        min_ang = [vertices_x[index_min_ang], vertices_y[index_min_ang], dist2poly[index_min_ang], angle2poly[index_min_ang]]
        max_ang = [vertices_x[index_max_ang], vertices_y[index_max_ang], dist2poly[index_max_ang], angle2poly[index_max_ang]]
        
        return min_ang, max_ang, min_dist
        
if __name__ == '__main__':
    test = np_json()


