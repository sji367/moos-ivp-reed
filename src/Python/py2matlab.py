# -*- coding: utf-8 -*-
"""
Created on Wed Nov 16 18:56:15 2016

@author: mapper
"""

from scipy.io import savemat

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

import sys

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
        self.All_ang = []
        self.util = []
        self.X = []
        self.Y = []
        self.cost = []

    def Register_Vars(self):
        """ Register for the current tide and the offset between MHW and MLLW.
        """
        self.comms.register('ang', 0)
        self.comms.register('OA_util', 0)
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('cost', 0)
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
        self.comms.run('localhost',9000,'Matlab')
        
    def Get_mail(self):
        """ Get the most recent value for the tide. """
        info = self.comms.fetch()
        
        # Store all values of the tide
        for x in info:
            if x.name() == 'ang':
                self.All_ang.append(x.double())
            elif x.name() == 'OA_util':
                self.util.append(x.string()) 
            elif x.name() == 'cost':
                self.cost.append(x.double()) 
            elif x.name() == 'NAV_X':
                self.X.append(x.double())
            elif x.name() == 'NAV_Y':
                self.Y.append(x.double())

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class convert2matlab(object):
    """ """
    def __init__(self):
        self.util = []
        self.ang = []
        self.X = []
        self.Y = []
        self.cost = []
    
    def run(self):
        MOOS = MOOS_comms()
        MOOS.Initialize()
        time.sleep(.25)
        try:
            while(True):
                MOOS.Get_mail()
#                if len(MOOS.All_ang) != 0:
##                    print 'new ang value'
#                    for i in range(len(MOOS.All_ang)):
#                        angle_raw = MOOS.All_ang.pop(0)
##                        angle = angle_raw.split(',')
#                        self.ang.append(angle_raw)
#                
#                if len(MOOS.util) != 0:
#                    print 'new util value:'
#                    for i in range(len(MOOS.util)):
#                        utility_raw = MOOS.util.pop(0)
##                        utility = utility_raw.split(',')
#                        self.util.append(utility_raw)
#                        
#                if len(MOOS.cost) != 0:
##                    print 'new cost value'
#                    for i in range(len(MOOS.cost)):
#                        self.cost.append(MOOS.cost.pop(0))
#                        
                if len(MOOS.X) != 0:
                    print 'new X value'
                    for i in range(len(MOOS.X)):
                        self.X.append(MOOS.X.pop(0))
                
                if len(MOOS.Y) != 0:
                    print 'new Y value'
                    for i in range(len(MOOS.Y)):
                        self.Y.append(MOOS.Y.pop(0))
                
        except KeyboardInterrupt:
            print 'end'
            savemat('nav.mat', mdict={'X': self.X, 'Y' : self.Y})
            print 'saved'
#            sys.exit()
            
        
if __name__ == '__main__':
    mat = convert2matlab()
    mat.run()

