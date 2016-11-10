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
        self.Next_WPT = []
        self.WPT_INDEX = []
        self.buff = []
        self.origin_lat = []
        self.origin_lon = []

    def Register_Vars(self):
        """ This function registers for the updates of the X,Y and heading at 
            the rate of which it is being outputed from MOOS.
        """
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('NAV_HEADING', 0)
        self.comms.register('Next_WPT', 0)
        self.comms.register('WPT_INDEX', 0)
        self.comms.register('Buffer_dist', 0)
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
        self.comms.run('localhost',9000,'WPT_Check1')
        
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
                elif x.name()=='WPT_INDEX':
                    self.WPT_INDEX.append(x.double())
            elif x.is_string():
                if x.name() == 'Next_WPT':
                    self.Next_WPT.append(x.string())
                elif (x.name() == 'Buffer_dist'):
                    self.buff.append(x.string())
                elif x.name()=='LatOrigin':
                    self.origin_lat.append(x.string())
                elif x.name()=='LongOrigin':
                    self.origin_lon.append(x.string())

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class WPT_Check_ENC(object):
    """This application has two main parts: initialization and an infinate  
        run loop. In the initialization, the program takes the information from 
        the ENCs and stores it to disk. Once that has happened, it then runs 
        the infinate loop. In the infinate loop, a search polygon is built and 
        the objects from the ENC that are within the search area are then 
        published to the MOOSDB.
    """

    # Define which UTM zone we are in
    LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    def __init__(self, buffer_dist=25, LatOrigin=43.071959194444446, 
                 LongOrigin=-70.711610833333339, debug = True,
                 filename_poly='../../src/ENCs/Shape/poly.shp',):
        """ Initialize varibles. 
            
            Inputs:
                search_dist - length of each side of the seach polygon
                LatOrigin - Latitude origin for the pyproj conversion to UTM
                LongOrigin - Longitude origin for the pyproj conversion to UTM
                ENC_filename - path to the ENC file
                filename_* - path to the file needed so that we can reorganize 
                            the ENC to shapefiles by geometry type 
        """
        self.LatOrigin = LatOrigin
        self.LongOrigin = LongOrigin
        self.buffer_dist = buffer_dist
        self.filename_poly = filename_poly
        self.debug = debug
    
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
    
    #==============================================================================
    # When given two sets of points, this function calculates the slope (m) and 
    #   y-intercept (b).
    #==============================================================================  
    def slopeintersept(self, x1,y1,x2,y2):
        """ This function calculates the slope and y intercept when given 2 
            (X,Y) points.
            
        Inputs:
            x1 - x position of point 1
            y1 - y position of point 1
            x2 - x position of point 2
            y2 - y position of point 2
            
        Outputs:
            m - slope of line
            b - y intercept of line
            """
        if x2 != x1:
            
            m = float(y2-y1)/float(x2-x1)
            b = float(y1) - float(m*x1)
            return m, b
        else:
            return 'vert_line', ''

    def intercept(self, x1,y1, x2,y2, x3,y3, x4,y4):
        """ When given 4 sets of points, this function calculates the point at 
            which the two lines that they represent intercept.
            
        Inputs:
            x1 - x position of point 1
            y1 - y position of point 1
            x2 - x position of point 2
            y2 - y position of point 2
            x3 - x position of point 3
            y3 - y position of point 3
            x4 - x position of point 4
            y4 - y position of point 4
            
        Outputs:
            x_int - x position of the intercept of the two lines
            y_int - y position of the intercept of the two lines
        """
        # Find slope intercept form for the first line
        if x2 != x1:
            m1, b1 = self.slopeintersept(x1,y1,x2,y2)
            inf_slope_flag1 = False
        else:
            inf_slope_flag1 = True
            
        # Find slope intercept form for the second line
        if x4 != x3:
            m2, b2 = self.slopeintersept(x3,y3,x4,y4)
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
        
        if inf_slope_flag2 and inf_slope_flag1:
            if x4 == x1:
                return 'Same', ''
            else:
                return 'None', ''
        else:            
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
            
            if ((pt1_x_small <= x_int <= pt1_x_big) and (pt2_x_small <= x_int <= pt2_x_big)\
                    and (pt1_y_small <= y_int <= pt1_y_big) and (pt2_y_small <= y_int <= pt2_y_big)):
                return x_int, y_int
            else:
                return 'None', ''


    def PolyLineIntercept(self, geom):
        """ When given 2 sets of x,y coordinates and a geometry of a GDAL   
            polygon, this function interpolates between the polygon vertices to 
            determine the intersection of the line and the polygon vertices and
            the distance from the intersection and the current waypoint.
            
        Inputs:
            geom - geometry of the polygon
            
        Outputs:
            x_int - X position of the intercept of the current path and the 2 
                        vertices of the polygon
            y_int - Y position of the intercept of the current path and the 2 
                        vertices of the polygon 
            dist - Distance from the intersection and the current waypoint
        """
        if self.debug:
            print "WPT_x: %0.3f" % self.WPT_x
            print "WPT_y: %0.3f" % self.WPT_y
            print "prev_WPT_x: %0.3f" % self.prev_WPT_x
            print "prev_WPT_y: %0.3f" % self.prev_WPT_y
            
        # Find crossing point
        ring = geom.GetGeometryRef(0)
        for j in range(ring.GetPointCount()):
            pt_lon, pt_lat,z = ring.GetPoint(j)
            pt_x, pt_y = self.LonLat2MOOSxy(pt_lon, pt_lat)
            # If it is the first point, set the previous point to the 
            if j == 0:
                prev_lat, prev_lon,z = ring.GetPoint(ring.GetPointCount()-1)
                prev_X, prev_Y = self.LonLat2MOOSxy(prev_lat, prev_lon)
                
            x_int, y_int = self.intercept(self.WPT_x, self.WPT_y,self.prev_WPT_x,self.prev_WPT_y, pt_x, pt_y, prev_X, prev_Y)
                
            # If there is a valid intersection return it
            if ((x_int != 'None' or x_int != 'Same')and y_int != ''):
                dist = np.sqrt(np.square(x_int-self.WPT_x) + np.square(y_int-self.WPT_y))
                return x_int, y_int, dist
            
            # Store previous value                        
            prev_X = pt_x
            prev_Y = pt_y
        return 'failed', '', ''
        
    def WPT_skip(self, ASV_x, ASV_y, comms):
        """ If the waypoint is within a polygon, then switch to the next
            waypoint when it is comes within the buffer distance.
            
        Inputs:
            ASV_x - Current x position of the ASV
            ASV_y - Current y position of the ASV
            
        Output:
            self.skipped - Flag stating that a waypoint has been skipped
        """
#        ASV_pos = ogr.Geometry(ogr.wkbPoint)
#        ASV_pos.SetPoint_2D(0, ASV_x, ASV_y)
#        if self.check_within:
#            for k in range(len(self.intercept_poly)):
#                poly = self.intercept_poly[k]
#                print 'b4 dist {}'.format(len(self.intercept_poly))
#                dist2interpt = poly.Distance(ASV_pos)
#                print dist2interpt
#                comms.notify('Dist', str(dist2interpt))
            
        # If the waypoint is within a polygon, then switch to the next
        #   waypoint when it is comes within the buffer distance
        if self.check_within:
            for k in range(len(self.x_int)):
                if not isinstance(ASV_x, basestring):
                    dist2interpt = np.sqrt(np.square(self.x_int[k]-ASV_x) + np.square(self.y_int[k]-ASV_y))
                else:
                    print "INTERSECTION CODE FAILED, SKIPPING TO NEXT WPT"
                    dist2interpt = 0
                    
                if dist2interpt <= self.buffer_dist:
                    # Increment to the next waypoint 
                    comms.notify('WPT_UPDATE', 'currix='+str(int(self.wpt_index)+1))
                    comms.notify('VIEW_POINT','x=1,y=1,active=false,label=int')
                    if self.debug:
                        print 'Skipped WPT %i, going to %i' % (self.wpt_index, self.wpt_index+1)
                    self.check_within = False
                    self.prev_WPT_x = ASV_x
                    self.prev_WPT_y = ASV_y
                    self.skipped = True
                    
        return self.skipped
    
    def Set_Prev_WPT(self, ASV_x, ASV_y):
        """ There are two cases for setting the previous waypoint:
                1) On the first iteration there is no previous waypoint.
                    Therefore, we will set the current position of the ASV as 
                    the previous waypoint
           
                2) Afterwards, there is a previous waypoint so we will just set
                    the previous waypoint as the current value of WPT_x or 
                    WPT_y. If we skipped a waypoint, the previous waypoint has 
                    been previously set in the WPT_skip function.
        Inputs:
            ASV_x - Current x position of the ASV
            ASV_y - Current y position of the ASV
        """
        # On the first iteration there is no previous waypoint. Therefore,
        #   we will set the current position of the ASV as previous 
        #   waypoint.                        
        if self.first_iteration:
            self.prev_WPT_x = ASV_x
            self.prev_WPT_y = ASV_y
            
            self.first_iteration = False
            
        # Afterwards, there is a previous waypoint so we will just set the
        #   previous waypoint as the current value of WPT_x or WPT_y. If 
        #   we skipped a waypoint, the previous waypoint has been previously
        #   set in an earlier function.
        else:
            if self.skipped:
                self.skipped = False
            else:
                # Set the current position to the ASV as the x,y coordinate
                #   of the previous waypoint
                self.prev_WPT_x = self.WPT_x
                self.prev_WPT_y = self.WPT_y
    
    def WPT_build(self, WPT, poly_buff, comms):
        """ This function sets the current waypoint and then determines the 
            straight line path between the two points.
            
        Inputs:
            WPT - String from MOOS that gives the x,y position of the current 
                    waypoint and is in the form of WPT_x,WPT_y
            poly_buff - Buffer from the straight line path for the returned 
                    line plan polygon (in Lat/Long)
            comms - Communication link to MOOS
            
        Outputs:
            WPT_poly - OGR polygon describing the straight line path with a 
                    small buffer
            curr_WPT - OGR point describing the current waypoint
        """
        if self.debug:
            print WPT        
        
        # Parse waypoint string. It is in the format --> x,y
        self.WPT_x, self.WPT_y = WPT.split(",")
                
        # Convert the string to a float
        self.WPT_x = float(self.WPT_x)
        self.WPT_y = float(self.WPT_y)
        
        # Post a polygon to pMarineViewer to show the location of the  
        #   current waypoint.
        print_WPT = 'format=radial,x='+str(self.WPT_x)+',y='+str(self.WPT_y)+\
            ',radius=5,pts=4,edge_size=5,vertex_size=2,edge_color=gold,label=WPT'
        comms.notify('VIEW_POLYGON', print_WPT)
                 
        # Convert the previous and current waypoints into lat and long
        WPT_long, WPT_lat  = self.MOOSxy2LonLat(self.WPT_x, self.WPT_y)
        prev_WPT_long, prev_WPT_lat  = self.MOOSxy2LonLat(self.prev_WPT_x, self.prev_WPT_y)                
        
        # Make GDAL point for the current waypoint
        curr_WPT = ogr.Geometry(ogr.wkbPoint)
        curr_WPT.AddPoint(WPT_long, WPT_lat)
        
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
        
        WPT_poly = WPT_line.Buffer(poly_buff)
        
        return WPT_poly, curr_WPT
    
    def WPT_Valid(self, ASV_x, ASV_y, WPT, comms):
        """ This function sets the current waypoint and then cycles through the
            polygon obstacles and checks to see if the the current waypoint or 
            straight line path (with a small buffer) intersects any obstacles.
            
            If it does, then the this function sets the flag:
                check_within = true (current waypoint is within a polygon) and/or
                check_intersection = true (straight line path intersects a polygon)
            
        Inputs:
            ASV_x - Current x position of the ASV
            ASV_y - Current y position of the ASV
            WPT - String from MOOS that gives the x,y position of the current 
                    waypoint and is in the form of WPT_x,WPT_y
            comms - Communication link to MOOS
        Outputs:
            check_within - Flag that shows if the current waypoint is within a 
                    polygon
            check_intersection - Flag that shows if the straight line path 
                    intersects a polygon
        """
        self.check_intersection = False
        self.check_within = False
        self.x_int, self.y_int, self.intercept_poly = [],[],[]
        
        # Set Previous Waypoint
        self.Set_Prev_WPT(ASV_x, ASV_y)        
        
        # Set Current Waypoint
        WPT_poly, curr_WPT = self.WPT_build(WPT, 0.0001, comms)
        
        ## Check to see if the next waypoint/path is valid
        for i in range (0,self.layer.GetFeatureCount()):
            # Get the info on the feature
            feature = self.layer.GetFeature(i)
            geom = feature.GetGeometryRef()
            
            # Check to see if the next waypoint is within the polygon
            #   obstacle. If it is, get the intersection between the 
            #   polygon and the planned path.
            if curr_WPT.Within(geom):
                self.intercept_poly.append(geom)
                # Get the X and Y intercepts
                cur_x_int, cur_y_int, d_inside = self.PolyLineIntercept(geom)
                self.check_within = True
                pos = 'x={},y={}'.format(cur_x_int,cur_y_int)
                print_intersect_pnt =  pos+',vertex_size=6,vertex_color=pink,active=true,label=int'
                comms.notify('VIEW_POINT', print_intersect_pnt)
                self.x_int.append(cur_x_int)
                self.y_int.append(cur_y_int)
                
            # Check to see if the planned path will intersect the 
            #   polygon obstacle
            if WPT_poly.Intersect(geom):
                self.check_intersection = True
                
        # Update flags and print out status message
        comms.notify('INVALID_WPT', str(self.check_within))
        comms.notify('INVALID_PATH', str(self.check_intersection)) 
        
        if self.debug:
            if self.check_within:
                print "ERROR: Next Waypoint is NOT Valid"
            else:
                print "Next Waypoint is Valid"
            
            if self.check_intersection:
                print "ERROR: Planned path intersects obstacles"
            else:
                print "Planned path is  Valid"
        
        return self.check_within, self.check_intersection
                
    def Initialize(self):
        """ This initializes the comminications with MOOS which allows 
            everythingelse to use "comms"  which will be used for communicating
            with theMOOSDB and the lists of "new mail". Also, this function 
            opens the file with the polygon obstacles and places the layer that
            holds the information on the obstacles into self.
        
        Initialize Variables:
            self.layer - Polygon obstacle layer
            self.first_iteration - Boolean flag to get the store the first 
                    position  
            check_within - Flag that shows if the current waypoint is within a 
                    polygon
            check_intersection - Flag that shows if the straight line path 
                    intersects a polygon  
            self.skipped - Boolean flag that shows if we have skipped a waypoint
            self.x_int, self.y_int - List describing the x or y intersection 
                    between the planned path and a polygon if the waypoint if 
                    inside a polygon
            self.wpt_index - Counter for the index of the current waypoint
        
        Output:
            MOOS - Object to be able to utilize the MOOS comms.
            
        """
        # Start connection to MOOS
        MOOS = MOOS_comms()
        MOOS.Initialize()
        time.sleep(.11)
        
        # Open the file with the polygon obstacles
        driver = ogr.GetDriverByName('ESRI Shapefile')
        self.ds = driver.Open(self.filename_poly, 0)
        
        # There is only one layer in the file 
        self.layer = self.ds.GetLayer()
        
        # Initialize other variables
        self.first_iteration = True
        self.check_within = False
        self.check_intersection = False
        self.skipped = False
        self.first_pos = True
        self.x_int, self.y_int = [],[]
        self.wpt_index = 0
        
        MOOS.Get_mail()
        # Get the Lat/Long Origin
        if (len(MOOS.origin_lat) != 0 and len(MOOS.origin_lon) != 0): 
            origin_lat = self.MOOS.origin_lat[-1]
            origin_lon = self.MOOS.origin_lon[-1]
            self.LatOrigin = float(origin_lat)
            self.LongOrigin = float(origin_lon)
        self.x_origin, self.y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)
        
        if self.debug:
            print "Init"
            
        return MOOS
        
    def run(self):
        """ This program uses the X and Y cooridinates from the ASV and filters 
            out all of the points from the ENC database that are in a 
            predetermined search radius. It then outputs information need for 
            obstacle avoidance to the MOOSDB as a string. 
        """
        MOOS = self.Initialize()
        
        while(True):
            time.sleep(.001)
            MOOS.Get_mail()
            # Check to see if the buffer has changed
            if len(MOOS.buff)  != 0:
                self.buffer_dist = float(MOOS.buff[-1])
                MOOS.buff = []
                
            # Check to see if there is a new waypoint index
            if len(MOOS.WPT_INDEX) != 0:
                self.wpt_index = MOOS.WPT_INDEX[-1]
                MOOS.WPT_INDEX = []
                
            # Find and store the first position
            if len(MOOS.NAV_X) != 0 and len(MOOS.NAV_Y)!= 0 and len(MOOS.NAV_HEAD)!=0 and self.first_pos:
                # Return the most recent information on the ASV's state
                ASV_x = MOOS.NAV_X[-1]
                ASV_y = MOOS.NAV_Y[-1]
                ASV_head = MOOS.NAV_HEAD[-1]
                MOOS.NAV_X, MOOS.NAV_Y, MOOS.NAV_HEAD = [],[],[]
                self.first_pos = False
        
            # When there is a new waypoint, we want to check to see if the next 
            #   waypoint is valid and if the planned path intersects any polygon.
            #   If the waypoint is NOT valid, then we will post a flag and  
            #   calculate the intersection between the polygon and the planned
            #   path.
            if len(MOOS.Next_WPT) != 0:
                # Get the next waypoint --> x,y
                WPT = MOOS.Next_WPT[-1]
                MOOS.Next_WPT = []
                
                ## Check to see if the next waypoint/path is valid
                self.WPT_Valid(ASV_x, ASV_y, WPT, MOOS.comms)
                     
            #======================================================================
            # MOOS freaks out when nothing is posted to the DB so post this dummy
            #   variable to avoid this problem if nothing was posted during the
            #   last cycle.
            #======================================================================
            else:
                MOOS.comms.notify('dummy_var','')
        
            # Check to see if there is a new position and if there is check to see
            # if the waypoint needs to be skipped.
            if len(MOOS.NAV_X) != 0 and len(MOOS.NAV_Y)!= 0 and len(MOOS.NAV_HEAD)!=0 and not self.first_pos:
                # Return the most recent information on the ASV's state
                ASV_x = MOOS.NAV_X[-1]
                ASV_y = MOOS.NAV_Y[-1]
                ASV_head = MOOS.NAV_HEAD[-1]
                MOOS.NAV_X, MOOS.NAV_Y, MOOS.NAV_HEAD = [],[],[]
                # If the waypoint is within a polygon, then switch to the next
                #   waypoint when it is comes within the buffer distance
                self.WPT_skip(ASV_x, ASV_y, MOOS.comms)
    
if __name__ == "__main__":
    wpt_check = WPT_Check_ENC()
    wpt_check.run()