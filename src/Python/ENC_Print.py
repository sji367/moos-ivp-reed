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

from scipy.constants import foot as feet2meters


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
        self.tide = []
        self.MHW_offset = []
#        self.ENCs = []
        self.origin_lat = []
        self.origin_lon = []

    def Register_Vars(self):
        """ Register for the current tide and the offset between MHW and MLLW.
        """
        self.comms.register('Current_Tide', 0)
        self.comms.register('MHW_Offset', 0)
        self.comms.register('LatOrigin', 0)
        self.comms.register('LonOrigin', 0)
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
        self.comms.run('localhost',9000,'Print_ENC')
        
    def Get_mail(self):
        """ Get the most recent value for the tide. """
        info = self.comms.fetch()
        
        # Store all values of the tide
        for x in info:
            if x.name() == 'Current_Tide':
                self.tide.append(x.string())
            elif x.name() == 'MHW_Offset':
                self.MHW_offset.append(x.string()) 
            elif x.name()=='LatOrigin':
                self.origin_lat.append(x.string())
            elif x.name()=='LongOrigin':
                self.origin_lon.append(x.string())

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#       
class Print_ENC(object):
    """ This application takes the information from the ENC that has previously  
        been sorted into shapefiles by object geometry type and prints it to 
        pMarnineViewer.
    """
    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    def __init__(self, LatOrigin=43.071959194444446, print_t_lvl0=True,
                 LongOrigin=-70.711610833333339,  print_all_feat=False,
                 filename_pnt='../../src/ENCs/Shape/Point.shp', 
                 filename_poly='../../src/ENCs/Shape/Poly.shp', 
                 filename_line='../../src/ENCs/Shape/Line.shp'):
        """ Initialize varibles. 
            
        Inputs:
            LatOrigin - Latitude origin for the pyproj conversion to UTM
            LongOrigin - Longitude origin for the pyproj conversion to UTM
            print_t_lvl0 - Boolean for printing things that are threat level 0 
                            True - Print object that are threat level 0
                            False - Don't print object that are threat level 0
            print_all_feat - Boolean for determining if we want to limit 
                            spatially the objects printed to pMarineViewer
            filename_* - path to the shapefiles which are organized by geometry
                         type 
        """
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.print_area_filter = ogr.Geometry(ogr.wkbPolygon)
        self.first_print = True
        
        self.print_t_lvl0 = print_t_lvl0
        self.print_all_feat = print_all_feat 

        self.filename_pnt = filename_pnt
        self.filename_poly = filename_poly
        self.filename_line = filename_line
        
        # Open a communication link to MOOS
        self.init_MOOS()
        self.tide = 0
#        print '{} \n {} \n {}'.format(self.filename_pnt, self.filename_poly, self.filename_line)
        # Find the point, polygon and line layers
        self.Get_layers() 
    
    def init_MOOS(self):
        """ Initializes the communication link to MOOS. 
        
        Ouputs:
            self.comms - Communication link to MOOS
        """
        self.MOOS = MOOS_comms()
        self.MOOS.Initialize()
        # Need this time to connect to MOOS
        time.sleep(.25)
        self.MOOS.Get_mail()
        print 'Len: {}'.format(len(self.MOOS.MHW_offset))
        if (len(self.MOOS.MHW_offset) != 0):
            MLLW = self.MOOS.MHW_offset[-1]
            self.MHW_Offset = float(MLLW)
            self.MOOS.MHW_Offset = []
        
        # Get the Lat/Long Origin
        if (len(self.MOOS.origin_lat) != 0 and len(self.MOOS.origin_lon) != 0): 
            origin_lat = self.MOOS.origin_lat[-1]
            origin_lon = self.MOOS.origin_lon[-1]
            self.LatOrigin = float(origin_lat)
            self.LongOrigin = float(origin_lon)
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        
        
    def Get_layers(self):
        """ This function defines the layers for the points, polygon and lines.
            It also can filter the layers spatially and to not contain objects 
            with threat level 0.
        
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
        self.ds_pnt = driver.Open(self.filename_pnt, 0)
        self.ds_poly = driver.Open(self.filename_poly, 0)
        self.ds_line = driver.Open(self.filename_line, 0)
        
        self.ENC_point_layer = self.ds_pnt.GetLayer()
        self.ENC_poly_layer = self.ds_poly.GetLayer()
        self.ENC_line_layer = self.ds_line.GetLayer()
        
        self.filter_feat()
            
    def filter_feat(self):
        """ This function filters the layers and reset them so that they start 
            on the first layer when feature.GetNextFeature() is called.
            
        Inputs:
            self.print_t_lvl0 - Boolean for printing things that are t_lvl 0 
                            True - Print object that are threat level 0
                            False - Don't print object that are threat level 0
            self.print_all_feat - Boolean for determining if we want to limit 
                            spatially the objects printed to pMarineViewer
        """
        self.ENC_point_layer.ResetReading()
        self.ENC_poly_layer.ResetReading()
        self.ENC_line_layer.ResetReading()
        
        # Remove the old spatial filter
        self.ENC_point_layer.SetSpatialFilter(None)
        self.ENC_poly_layer.SetSpatialFilter(None)
        self.ENC_line_layer.SetSpatialFilter(None)
        
        # Remove the old attribute filter
        self.ENC_point_layer.SetAttributeFilter(None)
        self.ENC_poly_layer.SetAttributeFilter(None)
        self.ENC_line_layer.SetAttributeFilter(None)
        
        if not self.print_t_lvl0:
            self.ENC_point_layer.SetAttributeFilter("t_lvl>0")
            self.ENC_poly_layer.SetAttributeFilter("t_lvl>0")
            self.ENC_line_layer.SetAttributeFilter("t_lvl>0")
            
        # Filter the layers to only include features within the area in 
        #   pMarnineViewer - Need to do this to reset the layers to start at 
        #   first layer at feature.GetNextFeature().
        self.define_print_area()
        self.ENC_point_layer.SetSpatialFilter(self.print_area_filter)
        self.ENC_poly_layer.SetSpatialFilter(self.print_area_filter)
        self.ENC_line_layer.SetSpatialFilter(self.print_area_filter)
        
        if self.print_all_feat:  
            self.ENC_point_layer.SetSpatialFilter(None)
            self.ENC_poly_layer.SetSpatialFilter(None)
            self.ENC_line_layer.SetSpatialFilter(None)
            
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
        
    def calc_t_lvl(self, depth, WL, LayerName):
        """ This function uses the water level and depth attributes for a  
            feature and calculates the threat level for that obstacle.
            
        Inputs:
            depth - Recorded depth for the obstacle
                    (quantitative depth measurement)
            WL - Water level for the obstacle 
                    (qualitative depth measurement)
            LayerName - Name of the layer for the object that was inputted
            
        Outputs:
            t_lvl - Calculated threat level
        """    
        # If it is Land set threat level to 5
        if LayerName == 'LNDARE' or LayerName == 'DYKCON' or LayerName == 'PONTON' or LayerName == 'COALNE':
            t_lvl = 5
        elif LayerName == 'LIGHTS':
            t_lvl = -2
        # If it is a Buoy or Beacon set threat level to 3
        elif LayerName == 'BOYISD' or LayerName == 'BOYSPP' or LayerName == 'BOYSAW' or LayerName == 'BOYLAT' or LayerName == 'BCNSPP' or LayerName == 'BCNLAT':
            t_lvl = 3
        elif LayerName == 'LNDMRK':
            t_lvl = -1
        else:
            # Deal with the cases where the attributes are empty
            if WL == None:
                WL = 0
            else:
                WL_depth = self.calc_WL_depth(WL)
            if depth == None or depth == 9999.0:
                current_depth = 9999.0
            else:
                current_depth = depth+self.tide    
            # Neither the WL or depth are recorded - this is bad
            if ((current_depth == 9999.0) and (WL == 0)):
                print 'FAILED, Threat Level will be set to 4'
                t_lvl = 4
            # No Charted Depth
            elif current_depth == 9999.0:
                # If there is no depth, use the Water Level attribute to 
                #   calculate the threat level. There is no quanitative 
                #   description of qualitative WL attribute for IDs 1, 6 and 7.
                #   Therefore, they will print a warning and set the threat
                #   level to 4.
                if WL in [1,6,7]:
                    print 'Unknown Description of Water Level: {}, Threat Level will be set to 4.'.format(WL)
                    t_lvl = 4
                t_lvl = self.threat_level(WL_depth)
            # WL is unknown
            elif WL == 0:
                t_lvl = self.threat_level(current_depth)  
            # If we have both the WL and the depth, use the depth measurement
            else:
#                # Go with the recorded depth unless it is more than a meter off
#                if ((current_depth-WL_depth) < -1):
#                    z = current_depth
#                else:
#                    z = WL_depth
                WL_t_lvl = self.threat_level(WL_depth)
                t_lvl = self.threat_level(current_depth)
#                if WL_t_lvl != t_lvl:
#                print 'WL:{} z:{}'.format(WL_t_lvl, t_lvl)
#        print t_lvl
        return t_lvl
              
    def threat_level(self, depth):
        """ This function uses a depth for a feature (Determined by a sounding 
            or realative to the WL attribute) and calculates the threat level 
            for that obstacle.
            
        Inputs:
            depth - Recorded depth for the obstacle (actual or relative to WL)
            
        Outputs:
            t_lvl - Calculated threat level
        """
        # Obstacle Partly submerged at high water, Always Dry, Awash, Floating, or 0>=Z
        if (depth<=0.0):
            t_lvl = 4
        # Obstacle Covers and uncovers, Subject to inundation or flooding, or 0<Z<1
        elif (depth< 1.0):
            t_lvl = 3
        # Obstacle is alway below surface
        elif (depth >= 1.0):
            # 1<=Z<2
            if (depth < 2.0):
                t_lvl = 2
            # 2<=Z<4
            elif (depth >=2.0 and depth <= 4.0):
                t_lvl = 1
            # Z > 4
            else:
                t_lvl = 0
                
        return t_lvl
        
    def calc_WL_depth(self, WL):
        """ This function updates the Water Level attribute with the current 
            predicted tide. This is shoal biased. The values used in these  
            calculations are from NOAA's Nautical Chart User Manual:
                
            http://portal.survey.ntua.gr/main/labs/carto/academic/persons/bnakos_site_nafp/documentation/noaa_chart_users_manual.pdf
            
        Inputs:
            WL - Water level for the obstacle 
                    (qualitative depth measurement)
            
        Outputs:
            WL_depth - current depth in relation to the 
        """
        WL = float(WL)
        if WL == 2:
            # At least 2 feet above MHW. Being shoal biased, we will take the 
            #   object's "charted" depth as 2 feet above MHW
            WL_depth = self.tide-(2.0*feet2meters+self.MHW_Offset)
            
        elif WL == 3:
            # At least 1 foot below MLLW. Being shoal biased, we will take the 
            #   object's "charted" depth as 1 foot below MLLW
            WL_depth = self.tide+1.0*feet2meters
            
        elif WL == 4:
            # The range for these attributes 1 foot below MLLW and 1 foot above MHW
            #   Therefore, we will be shoal biased and take 1 foot above MHW as the
            #   object's "charted" depth.
            WL_depth = self.tide-(1.0*feet2meters+self.MHW_Offset)
            
        elif WL == 5:
            # The range for these attributes 1 foot below MLLW and 1 foot above 
            #   MLLW. Therefore, we will be shoal biased and take 1 foot above MLLW
            #   as the object's "charted" depth.
            WL_depth = self.tide-1.0*feet2meters
        elif WL == 0:
            # Make the threat level completely dependant on the sounding depth
            WL_depth = 99.0
        else:
            # All other Water levels (1, 6, and 7) don't have a quantitative 
            #   descriptions. Therefore we will set it to 0.
            WL_depth = 0.0
            
        return WL_depth
    
    def define_print_area(self, N_lat = 43.07511878, E_long = -70.68395689, 
                          S_lat = 43.05780589, W_long = -70.72434189):
        """ Define the area that we want to print.
            
        Inputs:
            N_lat - Northern latitude boundary
            E_long - Eastern longitude boundary
            S_lat - Southern latitude boundary
            W_long - Western longitude boundary
        """
#        # White Island
#        E_long, N_lat = self.MOOSxy2LonLat(1606,-1184)
#        W_long, S_lat = self.MOOSxy2LonLat(2076,-1462)    
        
#        # Landmarks
#        E_long, N_lat = self.MOOSxy2LonLat(5719,-9353)
#        W_long, S_lat = self.MOOSxy2LonLat(8807,-11539) 
        
#        # Through and Island
#        E_long, N_lat = self.MOOSxy2LonLat(1021,-835)
#        W_long, S_lat = self.MOOSxy2LonLat(1291,-1119)        
          
        # Start by creating the baseline for the search area polygon 
        ring = ogr.Geometry(ogr.wkbLinearRing)
        
        # Build a ring of the points fot the search area polygon
        ring.AddPoint(W_long, N_lat)
        ring.AddPoint(E_long, N_lat)
        ring.AddPoint(E_long, S_lat)
        ring.AddPoint(W_long, S_lat)
        ring.AddPoint(W_long, N_lat)
        self.print_area_filter.AddGeometry(ring) # Add the ring to the previously created polygon
        
    def print_points(self):
        """ Print the points from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            self.comms - Communication link to MOOS
        """
        i = 0
        layer = self.ENC_point_layer
        print layer.GetFeatureCount()
        feature = layer.GetNextFeature()
        while feature:
            time.sleep(.0001)
            
            MLLW_t_lvl = feature.GetField(0) # Get threat level (@ MLLW)
            WL = feature.GetField(1)
            depth = feature.GetField(2)
            obs_type = feature.GetField(3)
            t_lvl = self.calc_t_lvl(depth, WL, obs_type)
            
            if self.first_print or MLLW_t_lvl != t_lvl:
                geom = feature.GetGeometryRef()
                
                # Convert the MOOS x,y position to Lat/Long and store it
                new_x, new_y = self.LonLat2MOOSxy (geom.GetX(), geom.GetY())
                location = 'x={},y={},'.format(new_x, new_y)     
                
                if not self.first_print:
                    print 'Point -  Old:{}, New{} --> x={},y={}'.format(MLLW_t_lvl, t_lvl,new_x, new_y)
                
                # Change the Color of the point based on the Threat Level
                if t_lvl == 5: #Coast Line
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
                
                label = ',label=point_{}'.format(i)
                size =  'vertex_size=10,'
                if t_lvl == 0:
                    active = ',active=false'
                else:
                    active = ',active=true'
                print_pnt = location+size+color+active+label
                # Print the point
                self.MOOS.comms.notify('VIEW_POINT', print_pnt)
            feature = layer.GetNextFeature()
            i += 1
    
    def print_polygons(self):
        """ Print the polygons from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            self.comms - Communication link to MOOS
        """
        i = 0
        layer = self.ENC_poly_layer
        
        feature = layer.GetNextFeature()
        while feature:
            time.sleep(.0001)
        
            MLLW_t_lvl = feature.GetField(0) # Get threat level (@ MLLW)
            WL = feature.GetField(1)
            depth = feature.GetField(2)
            obs_type = feature.GetField(3)
            t_lvl = self.calc_t_lvl(depth, WL, obs_type) # Calculate the threat level with respect to the current tide
            
            if self.first_print or MLLW_t_lvl != t_lvl:
                geom = feature.GetGeometryRef() # Polygon from shapefile
                
                if not self.first_print:
                    print 'Poly -  Old:{}, New{}'.format(MLLW_t_lvl, t_lvl)
                
                if not self.print_all_feat:
                    # Get the interesection of the polygon from the shapefile and the
                    #   outline of tiff from pMarnineViewer
                    intersection_poly = geom.Intersection(self.print_area_filter)
                else:
                    intersection_poly = geom
                
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
                    
                    vertex += '}'
                    label = 'label=poly_{}'.format(i)
                    # Change the Color of the point based on the Threat Level
                    if t_lvl == 5:
                        color = 'edge_color=black,vertex_color=black'
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
                      
                    if points != 0 and t_lvl != 0:
                        poly_info = '{},vertex_size=2.5,edge_size=2,{},{}'.format(vertex, color, label)
                        self.MOOS.comms.notify('VIEW_SEGLIST', poly_info)
            i += 1
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
        
    def print_lines(self):
        """ Print the lines from the ENC that are in the area defined by the
            print_area function.
            
        Inputs:
            self.comms - Communication link to MOOS
        """
        i = 0
        layer = self.ENC_line_layer
        feature = layer.GetNextFeature()
        while feature:
            time.sleep(.0001)
            
            MLLW_t_lvl = feature.GetField(0) # Get threat level
            WL = feature.GetField(1)
            depth = feature.GetField(2)
            obs_type = feature.GetField(3)
            t_lvl = self.calc_t_lvl(depth, WL, obs_type)
            
            if self.first_print or MLLW_t_lvl != t_lvl:
                line = feature.GetGeometryRef() # line from shapefile
                
                if not self.first_print:
                    print 'Line -  Old:{}, New{}'.format(MLLW_t_lvl, t_lvl)
                
                if not self.print_all_feat:
                    # Get the interesection of the line from the shapefile and the
                    #   outline of tiff from pMarnineViewer
                    line = self.poly_line_intersect(self.print_area_filter, feature.GetGeometryRef())
        
                points = line.GetPointCount()
                label = 'label=line_{}'.format(i)
                vertex = 'pts={' # String to hold the vertices
                # Cycle through the vertices and store them as a string
                for p2 in xrange(points):
                    lon, lat, z = line.GetPoint(p2)
                    p_x,p_y = self.LonLat2MOOSxy (lon, lat)
                    pos = '{},{}'.format(p_x, p_y)
                    vertex += pos
                    if (p2!=points-1):
                        vertex += ':'
                        
                vertex += '}'
                # Change the Color of the point based on the Threat Level
                if t_lvl == 5:
                    color = 'edge_color=black,vertex_color=black'
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
                if points != 0 and t_lvl != 0:
                    line_info = '{},vertex_size=2.5,edge_size=2,{},{}'.format(vertex, color,label)
                    self.MOOS.comms.notify('VIEW_SEGLIST', line_info)
            i += 1
            feature = layer.GetNextFeature()
            
    def print_all(self):
        """ This function prints all of the objects that are within the desired
            area to be printed to the pMarnineViewer.        
        """
        #self.print_points()
        self.print_polygons()
        #self.print_lines()
        self.first_print = False
        while(True):
            self.MOOS.Get_mail()
            if (len(self.MOOS.MHW_offset)):
                MLLW = self.MOOS.MHW_offset[-1]
                try:
                    self.MHW_Offset = float(MLLW)
                except ValueError:
                    print '{}'.format(MLLW)
                print 'Offset: {}'.format(self.MHW_Offset)
                self.MOOS.MHW_offset=[]

            if (len(self.MOOS.tide)!=0):
                # Get the current tide
                TIDE = self.MOOS.tide[-1]
                try:
                    self.tide = float(TIDE)
                except ValueError:
                    print '{}'.format(TIDE)
                print 'Tide: {}'.format(self.tide)
                self.MOOS.tide = []
                self.filter_feat()
                #self.print_points()
                self.print_polygons()
                #self.print_lines()

                

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
e = Print_ENC()
e.print_all()