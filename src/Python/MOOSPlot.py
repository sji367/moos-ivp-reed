#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import time
import pymoos

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
        self.Des_Heading = []
        self.Rudder = []
        self.Heading =[]

    def Register_Vars(self):
        """ Register for the ID for the Tide Station.
        """
        self.comms.register('DESIRED_HEADING', 0)
        self.comms.register('DESIRED_RUDDER', 0)
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
        self.comms.run('localhost',9000,'RT_Plot')
        
    def Get_mail(self):
        """ Get the new value for the name of the tide station """
        # Fetch the name of the tide station
        info = self.comms.fetch()
        
        # Store the tide station's name
        for x in info:
            if x.is_double():
#                print x.name(), x.double()
                if x.name()=='DESIRED_HEADING':
                    self.Des_Heading.append(x.double())
                elif x.name()=='DESIRED_RUDDER':
                    self.Rudder.append(x.double())
                elif x.name()=='NAV_HEADING':
                    self.Heading.append(x.double())
    

class plot_MOOS(object):
    def __init__(self):
        self.needsInit = True
        self.plots = {}
        
        # Open a communication link to MOOS
        self.init_MOOS()
        
        self.data_items= []
        self.data_items.append([])
        self.data_items.append([])
        self.data_items.append([])
        
        self.Label = ['Heading', 'Desired Heading', 'Desired Rudder']
        self.start = time.time()
        
        
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
        
    def updatePlot(self, pl,data,ts):
        pl.set_xdata(np.append(pl.get_xdata(), ts))
        pl.set_ydata(np.append(pl.get_ydata(), data))
    
    def getItem(self, data,path):
        if len(path) == 1:
            return data[path[0]]
        return self.getItem(data[path[0]],path[1:])
        
    def run(self):
        gotData=[False]*3
        while True:
            self.MOOS.Get_mail()
            
            if len(self.MOOS.Heading)!=0:
                self.data_items[2]=(self.MOOS.Heading[-1])
                self.MOOS.Heading = []
                gotData[0]=True
            else:
                gotData[0]=False
                
            if len(self.MOOS.Des_Heading) != 0:
                self.data_items[3]=(self.MOOS.Des_Heading[-1])
                self.MOOS.Des_Heading = []
                gotData[1]=True
            else:
                gotData[1]=False
                
            if len(self.MOOS.Rudder)!= 0:
                self.data_items[4]=(self.MOOS.Rudder[-1])
                self.MOOS.Rudder = []
                gotData[2]=True
            else:
                gotData[2]=False
                
            now = time.time()-self.start
            if self.needsInit:
                for di in range(len(self.data_items)):
                    if gotData[di]:
                        self.plots[di], = plt.plot(now, self.data_items[di],label=self.Label[di])
#                        print self.data_items[di]
                plt.legend()
                self.needsInit = False
            else:
                for di in range(len(self.data_items)):
                    if gotData[di]:
                        self.updatePlot(self.plots[di], self.data_items[di], now)
#                        print self.data_items[di]
            plt.pause(0.3)                        
            axes = plt.gca()
            if (now <100.0):
                axes.set_xlim([0, 100])
            else:
                axes.set_xlim([now-100.0, now])
                
            axes.set_ylim([-50, 400])

if __name__ == '__main__':
    p = plot_MOOS()
    p.run()