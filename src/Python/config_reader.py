# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 16:22:36 2016

@author: Sam Reed
"""

# Python-MOOS Bridge
import pymoos

# Function used for parsing text files like the config file
from parse_txt import parse_file

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
        self.comms.run('localhost',9000,'Config_file')
        
    def Get_mail(self):
        """ We don't need to get mail. """
        pass

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#  
class config_file(object):
    """ """
    def __init__(self, config_filename='../config_files/config.txt'):
        # Open a communication link to MOOS
        self.config_filename = config_filename
        
        # Defining the defaults configuration
        self.default_Unit_System = 'metric'
        self.default_ASV_length = '4.0'
        self.default_ASV_width = '1.0'
        self.default_ASV_height = '2.0'
        self.default_ASV_draft = '1.0'
        self.default_ASV_turning_radius = '1.0'
        self.default_ASV_min_speed = '0.0'
        self.default_ASV_max_speed = '2.5722' # Meters/sec (5 knots)
        self.default_ENCs = 'US5NH02'
        self.default_tide_station_ID = 'Fort_Point'
        self.default_boat_IP = '192.168.1.202'
        self.default_shore_IP= '192.168.1.204'
        self.default_camera_IP= '192.168.1.7'
        
    def parse_config(self, comms):
        """ This function takes the default input from a configuration file,  
            parses it and then post it to the MOOSDB as a string.
            
        Inputs:
            comms - Communication link to MOOS
        """
        list_config = parse_file(self.config_filename)
        for i in range(len(list_config)):
            param = list_config[i][0].lower()
            data = list_config[i][1]
            # ASV Physical Dimensions
            if (param=='unit_system'):
                Unit_System = data
                if Unit_System.lower() == 'default':
                    Unit_System = self.default_Unit_System
                comms.notify('Unit_System', '{}'.format(Unit_System))
            elif (param=='length'):
                ASV_length = data
                if ASV_length.lower() == 'default':
                    ASV_length = self.default_ASV_length
                comms.notify('ASV_length', '{}'.format(ASV_length))
            elif (param=='width'):
                ASV_width = data
                if ASV_width.lower() == 'default':
                    ASV_width = self.default_ASV_width
                comms.notify('ASV_width', '{}'.format(ASV_width))
            elif (param=='height'):
                ASV_height = data
                if ASV_height.lower() == 'default':
                    ASV_height = self.default_ASV_height
                comms.notify('ASV_height', '{}'.format(ASV_height))
            elif (param=='draft'):
                ASV_draft = data
                if ASV_draft.lower() == 'default':
                     ASV_draft= self.default_ASV_draft
                comms.notify('ASV_draft', '{}'.format(ASV_draft))
            
            # ASV Constraints
            elif (param=='turning_radius'):
                ASV_turning_radius = data
                if ASV_turning_radius.lower() == 'default':
                    ASV_turning_radius = self.default_ASV_turning_radius
                comms.notify('ASV_turning_radius', '{}'.format(ASV_turning_radius))
            elif (param=='min_speed'):
                ASV_min_speed = data
                if ASV_min_speed.lower() == 'default':
                    ASV_min_speed = self.default_ASV_min_speed
                comms.notify('ASV_min_speed', '{}'.format(ASV_min_speed))
            elif (param=='max_speed'):
                ASV_max_speed = data
                if ASV_max_speed.lower() == 'default':
                    ASV_max_speed = self.default_ASV_max_speed
                comms.notify('ASV_max_speed', '{}'.format(ASV_max_speed))
                
            # ENCs and Tide Station
            elif (param=='enc_name'):
                ENCs = data
                if ENCs.lower() == 'default':
                    ENCs = self.default_ENCs
                comms.notify('ENCs', '{}'.format(ENCs))
            elif (param=='tide_station_id'):
                tide_station_ID = data
                if tide_station_ID.lower() == 'default':
                    tide_station_ID = self.default_tide_station_ID
                comms.notify('Tide_station_ID', '{}'.format(tide_station_ID))
            
            # Networking
            elif (param=='boat_ip'):
                boat_IP = data
                if boat_IP.lower() == 'default':
                    boat_IP = self.default_boat_IP
                comms.notify('ASV_IP', '{}'.format(boat_IP))
            elif (param=='shore_ip'):
                shore_IP = data
                if shore_IP.lower() == 'default':
                    shore_IP = self.default_shore_IP
                comms.notify('shore_IP', '{}'.format(shore_IP))
            elif (param=='camera_ip'):
                camera_IP = data
                if camera_IP.lower() == 'default':
                    camera_IP = self.default_camera_IP
                comms.notify('camera_IP', '{}'.format(camera_IP))
            else:
                print '{} was not expected and will be ignored'.format(list_config[i][0])
        
MOOS = MOOS_comms()   
cf = config_file()
cf.parse_config(MOOS.comms)
    