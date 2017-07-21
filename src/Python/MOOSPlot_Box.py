#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import time
import pymoos
from sys import argv

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#
class MOOS_comms(object):
    """ This class id for MOOS communications. It has 2 parts:
          1. Initialize comms
              a. Set the Timewarp
              b. Connect to the server
          2. Get new values.
    """
    
    def __init__(self):
        # Open a communication link to MOOS
        self.comms = pymoos.comms()
        self.X = []
        self.Y = []
        self.Index =[]
        self.New_Pnt = []
        self.New_Poly = []

    def Register_Vars(self):
        """ Register for the ID for the Tide Station.
        """
        self.comms.register('NAV_X', 0)
        self.comms.register('NAV_Y', 0)
        self.comms.register('NEW_POINTS', 0)
        self.comms.register('NEW_POLYS', 0)
        self.comms.register('CYCLE_INDEX', 0)
        
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
        self.comms.run('localhost',9000,'Plot_WPT')
        
    def Get_mail(self):
        """ Get the new value for the name of the tide station """
        # Fetch the name of the tide station
        info = self.comms.fetch()
        
        # Store the tide station's name
        for x in info:
            if x.is_double():
                if x.name()=='NAV_X':
                    self.X.append(x.double())
                elif x.name()=='NAV_Y':
                    self.Y.append(x.double())
                elif x.name()=='CYCLE_INDEX':
                    self.Index.append(x.double())
            else:
                if x.name()=='NEW_POLYS':
                    self.New_Poly.append(x.string())
                elif x.name()=='NEW_POINTS':
                    self.New_Pnt.append(x.string())
    

class plot_MOOS(object):
    def __init__(self, wptX, wptY):
        self.needsInit = True
        self.plots = {}
        
        # Open a communication link to MOOS
        self.init_MOOS()
        
        self.data_items= []
        self.data_items.append([])
        self.data_items.append([])
        self.data_items.append([])
        self.data_items.append([])
        self.data_items.append([])
        
        self.start = time.time()
        
        self.wptX = wptX
        self.wptY = wptY
        
        
    def init_MOOS(self):
        """ Initializes the communication link to MOOS. 
        
        Ouputs:
            self.comms - Communication link to MOOS
            MOOS - Object to access the MOOSDB
        """
        self.MOOS = MOOS_comms()
        self.MOOS.Initialize()
        # Need this time to connect to MOOS
        time.sleep(.25)
        
        self.comms = self.MOOS.comms
        
    def updatePlot(self, pl, x, y, clear=False):
        if clear:
            plt.savefig('plot.pdf')
            pl.set_xdata([])
            pl.set_ydata([])
        else:
            pl.set_xdata(np.append(pl.get_xdata(), x))
            pl.set_ydata(np.append(pl.get_ydata(), y))
        
    def run(self):
        while True:
            self.MOOS.Get_mail()
            
            if len(self.MOOS.X) == 0 and len(self.MOOS.Y) == 0:
                gotData=False
            else:
                gotData=True

            if not self.needsInit and len(self.MOOS.New_Pnt)!=0:
                # x=$(XPOS),y=$(YPOS),$(t_lvl),$(label)
                parse = self.MOOS.New_Pnt[-1].split(',')
                if len(parse) == 4:
                    ptX = parse[0].split('=')[1]
                    ptY = parse[1].split('=')[1]
                    self.plots[2], = plt.plot(ptX, ptY, 'm*', markersize =20)
                self.MOOS.New_Pnt = []
                
            
            if len(self.MOOS.X) != 0:
                x=(self.MOOS.X[-1])
                self.MOOS.X = []            
            
            if  len(self.MOOS.Y)!= 0:
                y=(self.MOOS.Y[-1])
                self.MOOS.Y = []
                
            if  len(self.MOOS.Index)!= 0:
#                clear_plot=True
                if not self.needsInit:
                    self.updatePlot(self.plots[1],0,0,True)
                    self.MOOS.Index = []
                    
            if self.needsInit:
                if gotData:
                    self.plots[0], = plt.plot(self.wptX, self.wptY, '--', linewidth=4)
                    self.plots[1], = plt.plot(x,y, 'r', linewidth=2)
                    self.needsInit = False
                    
            else:
                if gotData:
                    self.updatePlot(self.plots[1], x,y)
                    
            plt.pause(0.1)
            fig = plt.gca()
            fig.set_title('Simple Waypoint Mission')
            fig.set_xlim([min(self.wptX)-10, max(self.wptX)+10])
            fig.set_ylim([min(self.wptY)-20, max(self.wptY)+20])
            

if __name__ == '__main__':
    p = plot_MOOS([-100, 100, 100, -100, -100], [210, 210, 200, 200, 210])
    p.run()
#    if len(argv)>1:
#        tide = tide_prediction(argv[1])
#    else:
#        tide = tide_prediction()
#    tide.run()