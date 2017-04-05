# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 21:23:50 2017

@author: sji367
"""

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

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
        self.X = []
        self.Y = []
        self.Heading = []
        self.new_pts = []

    def Register_Vars(self):
        """ Register for the current tide and the offset between MHW and MLLW.
        """
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('NAV_HEADING', 0)
        self.comms.register('NEW_POINTS', 0)
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
        self.comms.run('localhost',9000,'Pub_Points')
        
    def Get_mail(self):
        """ Get the most recent value for the tide. """
        info = self.comms.fetch()
        
        # Store all values of the tide
        for x in info:
            if x.name() == 'NAV_X':
                self.X.append(x.double())
            elif x.name() == 'NAV_Y':
                self.Y.append(x.double()) 
            elif x.name()=='NAV_HEADING':
                self.Heading.append(x.double())
            elif x.name()=='NEW_POINTS':
                self.new_pts.append(x.string())

class pub_pnts(object):
    def __init__(self):
        self.MOOS = MOOS_comms()
        self.MOOS.Initialize()
        print 'init'
        # Need this time to connect to MOOS
        time.sleep(.25)
        self.MOOS.Get_mail()
        self.run()
        
    def dist(self, x1,y1, x2,y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)
        
    def run(self):
        x = 0
        y = 0
        head = 0
        new_pt = []
        pt = []
        while(True):
            self.MOOS.Get_mail()
            if (len(self.MOOS.X)>0 and len(self.MOOS.Y)>0 and len(self.MOOS.Heading)>0):
                x= self.MOOS.X[-1]
                y= self.MOOS.Y[-1]
                head= self.MOOS.Heading[-1]
#                print x,y,head
                
                self.MOOS.X=[]
                self.MOOS.Y = []
                self.MOOS.Heading = []
                
            if (len(self.MOOS.new_pts)>0):
                new_pt.append(self.MOOS.new_pts[-1])
                self.MOOS.new_pts = []

            for i in range(len(new_pt)):
#                print new_pt[i]
                parsed = new_pt[i].split(',')
#                print parsed
                pt_x = float(parsed[0].split('=')[1])
                pt_y = float(parsed[1].split('=')[1])
                if self.dist(pt_x, pt_y, x,y)<75:
                    pt.append('{},{},{}:1:{}'.format(x, y, head, new_pt[i]))
            for ii in range(len(pt)): 
                self.MOOS.comms.notify('Obstacles',pt[ii])
                
            time.sleep(.2)

a = pub_pnts()