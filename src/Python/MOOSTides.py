# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 09:40:06 2016

@author: mapper
"""

from datetime import datetime as dt
from pytides.tide import Tide
import pytides.constituent as cons
import numpy as np

from parse_txt import parse_file

import pymoos

# Used for delays
import time

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
        self.comms.run('localhost',9000,'Tides')
        
    def Get_mail(self):
        """ We don't need to get mail. """
        pass
    
class tide_prediction(object):
    """ This publishes a tidal prediction based off of NOAA's tidal harmonic 
    """
    def __init__(self, tide_station_name='Fort_Point', publish_rate=.1):
        # Rate of the tidal prediction (Hz)
        self.publish_rate = publish_rate
        
        # Open a communication link to MOOS
        self.init_MOOS()
        
        # Determine which station was inputed - to make easier, we are converting 
        #   all inputs to a lowercase string
        ts = str(tide_station_name).lower()
        if (ts == 'fort_point' or ts == 'fort point' or ts == '8423898'):
            self.tide_station = 'Fort_Point'
        elif (ts == 'wells' or ts == 'wells, me' or ts == '8419317'):
            self.tide_station = 'Wells'
        elif (ts == 'boston' or ts == 'boston, ma' or ts == '8443970'):
            self.tide_station = 'Boston'
        else:
            print 'ERROR: Invalid Tide Station. Create a new file for {}.'.format(tide_station_name)
            
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
        
    def get_model(self):
        """ This function gives a model of the predicted tides. To get tides at  
            a specific time, use the command:
                tide.at([datetime(year,month,day,hour,minute)])
            
        Output:
            self.tide - Model of the tides using NOAA's Harmonic Constituents 
        """     
        station_filename = '../Tides/{}/{}.txt'.format(self.tide_station, self.tide_station)
        
        station_info = parse_file(station_filename)
    
        # These are the NOAA constituents, in the order presented on their website.
        constituents = [c for c in cons.noaa if c != cons._Z0]
        
        # Phases and amplitudes (relative to GMT and in degrees and meters)
        published_amplitudes = station_info[0][1]
        published_phases = station_info[1][1]
        
        # We can add a constant offset (e.g. for a different datum, we will use
        #   relative to MLLW):
#       MTL = station_info[4][1]
        MSL = station_info[5][1]
        MLLW = station_info[6][1]
        offset = MSL - MLLW
        constituents.append(cons._Z0)
        published_phases.append(0)
        published_amplitudes.append(offset)
        
        # Build the model.
        assert(len(constituents) == len(published_phases) == len(published_amplitudes))
        model = np.zeros(len(constituents), dtype = Tide.dtype)
        model['constituent'] = constituents
        model['amplitude'] = published_amplitudes
        model['phase'] = published_phases
        
        self.tide = Tide(model = model, radians = False)

    def get_curr_tide(self, comms):
        """ This function returns the current tide in relation to MLLW and 
            publishes to the MOOSDB.
        
        Output:
            current_tide - Current tide compared to MLLW
        """
        current_time = dt.now()
        current_tide = self.tide.at([current_time])
        comms.notify('Current_Tide', str(current_tide[0]))
        return current_tide[0]
        
    def run(self):
        """ This function publishes the current tide compared to MLLW to the 
            MOOSDB at the previously defined rate in Hz
        """
        self.get_model()
        while(True):
            tide = self.get_curr_tide(MOOS.comms)
            print tide
            time.sleep(1/self.publish_rate)

if __name__ == '__main__':
    tide = tide_prediction()
    tide.run()
    
    
    
    
