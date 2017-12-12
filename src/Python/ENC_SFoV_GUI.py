# -*- coding: utf-8 -*-
"""
Created on Mon Jul 11 14:44:22 2016

@author: mapper
"""
# Python-MOOS Bridge
import pymoos

# GUI program
import Tkinter as tk

def Draw():
    global text_r0_c0, text_r0_c1, text_r0_c2, text_r0_c3
    
    # Landmark Globals for Grid
    global text_r1_c0, text_r1_c1, text_r1_c2, text_r1_c3
    global text_r2_c0, text_r2_c1, text_r2_c2, text_r2_c3
    global text_r3_c0, text_r3_c1, text_r3_c2, text_r3_c3
    global text_r4_c0, text_r4_c1, text_r4_c2, text_r4_c3
    global text_r5_c0, text_r5_c1, text_r5_c2, text_r5_c3
    
    # Nav_Aids Globals for Grid
    global text_r7_c0, text_r7_c1, text_r7_c2, text_r7_c3
    global text_r8_c0, text_r8_c1, text_r8_c2, text_r8_c3
    global text_r9_c0, text_r9_c1, text_r9_c2, text_r9_c3
    global text_r10_c0, text_r10_c1, text_r10_c2, text_r10_c3
    global text_r11_c0, text_r11_c1, text_r11_c2, text_r11_c3
    
    # Underwater Globals for Grid
    global text_r13_c0, text_r13_c1, text_r13_c2, text_r13_c3
    global text_r14_c0, text_r14_c1, text_r14_c2, text_r14_c3
    global text_r15_c0, text_r15_c1, text_r15_c2, text_r15_c3
    global text_r16_c0, text_r16_c1, text_r16_c2, text_r16_c3
    global text_r17_c0, text_r17_c1, text_r17_c2, text_r17_c3
    
    # Define and put Labels in correct place - Titles
    text_r0_c0 = tk.Label(text="  Obstacle Type  ", font = "Helvetica 12 bold").grid(row=0,column=0)
    text_r0_c1 = tk.Label(text='  Latitude  ', font = "Helvetica 12 bold").grid(row=0,column=1)
    text_r0_c2 = tk.Label(text='  Longitude  ', font = "Helvetica 12 bold").grid(row=0,column=2)
    text_r0_c3 = tk.Label(text='  Distance (m)  ', font = "Helvetica 12 bold").grid(row=0,column=3)
    
    # Add a space between the two sections
    text_space1_c0 = tk.Label(text='')
    text_space1_c1 = tk.Label(text='')
    text_space1_c2 = tk.Label(text='')
    text_space1_c3 = tk.Label(text='')
    
    text_space2_c0 = tk.Label(text='')
    text_space2_c1 = tk.Label(text='')
    text_space2_c2 = tk.Label(text='')
    text_space2_c3 = tk.Label(text='')
    
    text_space1_c0.grid(row=6,column=0)
    text_space1_c1.grid(row=6,column=1)
    text_space1_c2.grid(row=6,column=2)
    text_space1_c3.grid(row=6,column=3)
    
    text_space2_c0.grid(row=12,column=0)
    text_space2_c1.grid(row=12,column=1)
    text_space2_c2.grid(row=12,column=2)
    text_space2_c3.grid(row=12,column=3)
    
    ##-----------------------------Landmarks--------------------------------## 
    text_r1_c0 = tk.Label(text='')
    text_r1_c1 = tk.Label(text='')
    text_r1_c2 = tk.Label(text='')
    text_r1_c3 = tk.Label(text='')
    
    text_r2_c0 = tk.Label(text='')
    text_r2_c1 = tk.Label(text='')
    text_r2_c2 = tk.Label(text='')
    text_r2_c3 = tk.Label(text='')
    
    text_r3_c0 = tk.Label(text='')
    text_r3_c1 = tk.Label(text='')
    text_r3_c2 = tk.Label(text='')
    text_r3_c3 = tk.Label(text='')
    
    text_r4_c0 = tk.Label(text='')
    text_r4_c1 = tk.Label(text='')
    text_r4_c2 = tk.Label(text='')
    text_r4_c3 = tk.Label(text='')
    
    text_r5_c0 = tk.Label(text='')
    text_r5_c1 = tk.Label(text='')
    text_r5_c2 = tk.Label(text='')
    text_r5_c3 = tk.Label(text='')
    
    text_space1_c0.grid(row=6,column=0)
    text_space1_c1.grid(row=6,column=1)
    text_space1_c2.grid(row=6,column=2)
    text_space1_c3.grid(row=6,column=3)
    
    text_r1_c0.grid(row=1,column=0)
    text_r1_c1.grid(row=1,column=1)
    text_r1_c2.grid(row=1,column=2)
    text_r1_c3.grid(row=1,column=3)
    
    text_r2_c0.grid(row=2,column=0)
    text_r2_c1.grid(row=2,column=1)
    text_r2_c2.grid(row=2,column=2)
    text_r2_c3.grid(row=2,column=3)
    
    text_r3_c0.grid(row=3,column=0)
    text_r3_c1.grid(row=3,column=1)
    text_r3_c2.grid(row=3,column=2)
    text_r3_c3.grid(row=3,column=3)
    
    text_r4_c0.grid(row=4,column=0)
    text_r4_c1.grid(row=4,column=1)
    text_r4_c2.grid(row=4,column=2)
    text_r4_c3.grid(row=4,column=3)
    
    text_r5_c0.grid(row=5,column=0)
    text_r5_c1.grid(row=5,column=1)
    text_r5_c2.grid(row=5,column=2)
    text_r5_c3.grid(row=5,column=3)
    
    ##---------------------------Navigational Aids---------------------------##     
    text_r7_c0 = tk.Label(text='')
    text_r7_c1 = tk.Label(text='')
    text_r7_c2 = tk.Label(text='')
    text_r7_c3 = tk.Label(text='')
    
    text_r8_c0 = tk.Label(text='')
    text_r8_c1 = tk.Label(text='')
    text_r8_c2 = tk.Label(text='')
    text_r8_c3 = tk.Label(text='')
    
    text_r9_c0 = tk.Label(text='')
    text_r9_c1 = tk.Label(text='')
    text_r9_c2 = tk.Label(text='')
    text_r9_c3 = tk.Label(text='')
    
    text_r10_c0 = tk.Label(text='')
    text_r10_c1 = tk.Label(text='')
    text_r10_c2 = tk.Label(text='')
    text_r10_c3 = tk.Label(text='')
    
    text_r11_c0 = tk.Label(text='')
    text_r11_c1 = tk.Label(text='')
    text_r11_c2 = tk.Label(text='')
    text_r11_c3 = tk.Label(text='')
    
    text_r7_c0.grid(row=7,column=0)
    text_r7_c1.grid(row=7,column=1)
    text_r7_c2.grid(row=7,column=2)
    text_r7_c3.grid(row=7,column=3)
    
    text_r8_c0.grid(row=8,column=0)
    text_r8_c1.grid(row=8,column=1)
    text_r8_c2.grid(row=8,column=2)
    text_r8_c3.grid(row=8,column=3)

    text_r9_c0.grid(row=9,column=0)
    text_r9_c1.grid(row=9,column=1)
    text_r9_c2.grid(row=9,column=2)
    text_r9_c3.grid(row=9,column=3)
    
    text_r10_c0.grid(row=10,column=0)
    text_r10_c1.grid(row=10,column=1)
    text_r10_c2.grid(row=10,column=2)
    text_r10_c3.grid(row=10,column=3)
    
    text_r11_c0.grid(row=11,column=0)
    text_r11_c1.grid(row=11,column=1)
    text_r11_c2.grid(row=11,column=2)
    text_r11_c3.grid(row=11,column=3)
    
    ##---------------------------Underwater Objects--------------------------##
    text_r13_c0 = tk.Label(text='')
    text_r13_c1 = tk.Label(text='')
    text_r13_c2 = tk.Label(text='')
    text_r13_c3 = tk.Label(text='')
    
    text_r14_c0 = tk.Label(text='')
    text_r14_c1 = tk.Label(text='')
    text_r14_c2 = tk.Label(text='')
    text_r14_c3 = tk.Label(text='')
    
    text_r15_c0 = tk.Label(text='')
    text_r15_c1 = tk.Label(text='')
    text_r15_c2 = tk.Label(text='')
    text_r15_c3 = tk.Label(text='')
    
    text_r16_c0 = tk.Label(text='')
    text_r16_c1 = tk.Label(text='')
    text_r16_c2 = tk.Label(text='')
    text_r16_c3 = tk.Label(text='')
    
    text_r17_c0 = tk.Label(text='')
    text_r17_c1 = tk.Label(text='')
    text_r17_c2 = tk.Label(text='')
    text_r17_c3 = tk.Label(text='')
    
    text_r13_c0.grid(row=13,column=0)
    text_r13_c1.grid(row=13,column=1)
    text_r13_c2.grid(row=13,column=2)
    text_r13_c3.grid(row=13,column=3)
    
    text_r14_c0.grid(row=14,column=0)
    text_r14_c1.grid(row=14,column=1)
    text_r14_c2.grid(row=14,column=2)
    text_r14_c3.grid(row=14,column=3)
    
    text_r15_c0.grid(row=15,column=0)
    text_r15_c1.grid(row=15,column=1)
    text_r15_c2.grid(row=15,column=2)
    text_r15_c3.grid(row=15,column=3)
    
    text_r16_c0.grid(row=16,column=0)
    text_r16_c1.grid(row=16,column=1)
    text_r16_c2.grid(row=16,column=2)
    text_r16_c3.grid(row=16,column=3)
    
    text_r17_c0.grid(row=17,column=0)
    text_r17_c1.grid(row=17,column=1)
    text_r17_c2.grid(row=17,column=2)
    text_r17_c3.grid(row=17,column=3)
    
##---------------------------------------------------------------------------##
# This function updates the configuration labels for the Landmarks from the 
#    MOOS Variables passed as an input
##---------------------------------------------------------------------------##    
def config_Landmarks(landmark):
    # Landmark Globals for Grid
    global text_r1_c0, text_r1_c1, text_r1_c2, text_r1_c3
    global text_r2_c0, text_r2_c1, text_r2_c2, text_r2_c3
    global text_r3_c0, text_r3_c1, text_r3_c2, text_r3_c3
    global text_r4_c0, text_r4_c1, text_r4_c2, text_r4_c3
    global text_r5_c0, text_r5_c1, text_r5_c2, text_r5_c3    
    
    # Parse Landmark string for type, lat, long and distance
    L = landmark.split('!')
    for i in range(5):
        if i <len(L) and landmark != '':
            TYPE,LAT,LON,DIST = L[i].split(',')
            letters,lat = LAT.split(':')
            letters,lon = LON.split(':')
            exec "TYPE_%s=TYPE" % (i+1)
            exec "lat_%s=lat" % (i+1)
            exec "lon_%s=lon" % (i+1)
            exec "DIST_%s=DIST" % (i+1)
        else:
            exec "TYPE_%s=''" % (i+1)
            exec "lat_%s=''" % (i+1)
            exec "lon_%s=''" % (i+1)
            exec "DIST_%s=''" % (i+1)
    
    # Update the labels text
    text_r1_c0.configure(text=TYPE_1)
    text_r1_c1.configure(text=lat_1)
    text_r1_c2.configure(text=lon_1)
    text_r1_c3.configure(text=DIST_1)
    
    text_r2_c0.configure(text=TYPE_2)
    text_r2_c1.configure(text=lat_2)
    text_r2_c2.configure(text=lon_2)
    text_r2_c3.configure(text=DIST_2)
    
    text_r3_c0.configure(text=TYPE_3)
    text_r3_c1.configure(text=lat_3)
    text_r3_c2.configure(text=lon_3)
    text_r3_c3.configure(text=DIST_3)
    
    text_r4_c0.configure(text=TYPE_4)
    text_r4_c1.configure(text=lat_4)
    text_r4_c2.configure(text=lon_4)
    text_r4_c3.configure(text=DIST_4)
    
    text_r5_c0.configure(text=TYPE_5)
    text_r5_c1.configure(text=lat_5)
    text_r5_c2.configure(text=lon_5)
    text_r5_c3.configure(text=DIST_5)

##---------------------------------------------------------------------------##
# This function updates the configuration labels for the Navigational Aid 
#   obstacles from the MOOS Variables passed as an input
##---------------------------------------------------------------------------##   
def config_Nav_Aids(nav_aids):
    # Nav_Aids Globals for Grid
    global text_r7_c0, text_r7_c1, text_r7_c2, text_r7_c3
    global text_r8_c0, text_r8_c1, text_r8_c2, text_r8_c3
    global text_r9_c0, text_r9_c1, text_r9_c2, text_r9_c3
    global text_r10_c0, text_r10_c1, text_r10_c2, text_r10_c3
    global text_r11_c0, text_r11_c1, text_r11_c2, text_r11_c3
    
    # Parse Nav_Aids string for type, lat, long and distance
    NA = nav_aids.split('!')
    for i in range(5):
        if i <len(NA) and nav_aids != '':
            TYPE,LAT,LON,DIST = NA[i].split(',')
            letters,lat = LAT.split(':')
            letters,lon = LON.split(':')
            exec "TYPE_%s=TYPE" % (i+1)
            exec "lat_%s=lat" % (i+1)
            exec "lon_%s=lon" % (i+1)
            exec "DIST_%s=DIST" % (i+1)
        else:
            exec "TYPE_%s=''" % (i+1)
            exec "lat_%s=''" % (i+1)
            exec "lon_%s=''" % (i+1)
            exec "DIST_%s=''" % (i+1)
    
    # Update the labels text
    text_r7_c0.configure(text=TYPE_1)
    text_r7_c1.configure(text=lat_1)
    text_r7_c2.configure(text=lon_1)
    text_r7_c3.configure(text=DIST_1)
    
    text_r8_c0.configure(text=TYPE_2)
    text_r8_c1.configure(text=lat_2)
    text_r8_c2.configure(text=lon_2)
    text_r8_c3.configure(text=DIST_2)
    
    text_r9_c0.configure(text=TYPE_3)
    text_r9_c1.configure(text=lat_3)
    text_r9_c2.configure(text=lon_3)
    text_r9_c3.configure(text=DIST_3)
    
    text_r10_c0.configure(text=TYPE_4)
    text_r10_c1.configure(text=lat_4)
    text_r10_c2.configure(text=lon_4)
    text_r10_c3.configure(text=DIST_4)
    
    text_r11_c0.configure(text=TYPE_5)
    text_r11_c1.configure(text=lat_5)
    text_r11_c2.configure(text=lon_5)
    text_r11_c3.configure(text=DIST_5)

##---------------------------------------------------------------------------##
# This function updates the configuration labels for the underwater obstacles
#   from the MOOS Variables passed as an input
##---------------------------------------------------------------------------##   
def config_Underwater(underwater):
    # Underwater Globals for Grid
    
    # Underwater Globals for Grid
    global text_r13_c0, text_r13_c1, text_r13_c2, text_r13_c3
    global text_r14_c0, text_r14_c1, text_r14_c2, text_r14_c3
    global text_r15_c0, text_r15_c1, text_r15_c2, text_r15_c3
    global text_r16_c0, text_r16_c1, text_r16_c2, text_r16_c3
    global text_r17_c0, text_r17_c1, text_r17_c2, text_r17_c3  
    
    # Parse Nav_Aids string for type, lat, long and distance
    UW = underwater.split('!')
    for i in range(5):
        if i <len(UW) and underwater != '':
            TYPE,LAT,LON,DIST = UW[i].split(',')
            letters,lat = LAT.split(':')
            letters,lon = LON.split(':')
            exec "TYPE_%s=TYPE" % (i+1)
            exec "lat_%s=lat" % (i+1)
            exec "lon_%s=lon" % (i+1)
            exec "DIST_%s=DIST" % (i+1)
        else:
            exec "TYPE_%s=''" % (i+1)
            exec "lat_%s=''" % (i+1)
            exec "lon_%s=''" % (i+1)
            exec "DIST_%s=''" % (i+1)
    
    # Update the labels text
    text_r13_c0.configure(text=TYPE_1)
    text_r13_c1.configure(text=lat_1)
    text_r13_c2.configure(text=lon_1)
    text_r13_c3.configure(text=DIST_1)
    
    text_r14_c0.configure(text=TYPE_2)
    text_r14_c1.configure(text=lat_2)
    text_r14_c2.configure(text=lon_2)
    text_r14_c3.configure(text=DIST_2)
    
    text_r15_c0.configure(text=TYPE_3)
    text_r15_c1.configure(text=lat_3)
    text_r15_c2.configure(text=lon_3)
    text_r15_c3.configure(text=DIST_3)
    
    text_r16_c0.configure(text=TYPE_4)
    text_r16_c1.configure(text=lat_4)
    text_r16_c2.configure(text=lon_4)
    text_r16_c3.configure(text=DIST_4)
    
    text_r17_c0.configure(text=TYPE_5)
    text_r17_c1.configure(text=lat_5)
    text_r17_c2.configure(text=lon_5)
    text_r17_c3.configure(text=DIST_5)
    
##---------------------------------------------------------------------------##
# This function updates the labels from the MOOS Variables and communicates
#   with MOOS
##---------------------------------------------------------------------------##
def Refresher():
    global MOOS
    
    # Fetch the new values from the MOOSDB
    MOOS.Get_mail()
    
    landmark = ''
    nav_aids = ''
    underwater = ''
    
    # If there is a new Landmark string, get the newest value
    if len(MOOS.LANDMARKS) != 0:
        landmark = MOOS.LANDMARKS[-1]
        MOOS.LANDMARKS = []
    
    # If there is a new Nav_Aid string, get the newest value 
    if len(MOOS.NAV_AIDS)!= 0:
        nav_aids = MOOS.NAV_AIDS[-1]
        MOOS.NAV_AIDS = []
    
    # If there is a new Underwater string, get the newest value
    if len(MOOS.UNDERWATER)!= 0:
        underwater = MOOS.UNDERWATER[-1]
        MOOS.UNDERWATER = []
    
    # Update the Landmarks labels from the new MOOS values or the null value if
    #   there was no new value posted
    config_Landmarks(landmark) 
    
    # Update the Naviagational Aids labels from the new MOOS values or the null 
    #   value if there was no new value posted
    config_Nav_Aids(nav_aids)
    
    # Update the Underwater labels from the new MOOS values or the null value 
    #   if there was no new value posted
    config_Underwater(underwater)
    
    # Update the labels every second
    root.after(1000, Refresher)

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
        # Initialize the MOOS lists
        self.LANDMARKS = []
        self.NAV_AIDS  = []
        self.UNDERWATER = []

    def Register_Vars(self):
        """ This function registers for the updates of the X,Y and heading at 
            the rate of which it is being outputed from MOOS.
        """
        self.comms.register('Landmarks', 0)
        self.comms.register('Nav_Aids', 0)
        self.comms.register('Underwater_Objects', 0)
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
        self.comms.run('localhost',9000,'GUI1')
        
    def Get_mail(self):
        """ When called, this function fetches the new values for the objects
            in the sensor field of view.
        """
        
        # Fetch the values of the ASV position in meters (x,y) and heading
        info = self.comms.fetch()
        
        # Store all values of the objects sensor field of view
        for x in info:
            if x.is_string():
                if x.name()=='Landmarks':
                    self.LANDMARKS.append(x.string())
                elif x.name()=='Nav_Aids':
                    self.NAV_AIDS.append(x.string())
                elif x.name()=='Underwater_Objects':
                    self.UNDERWATER.append(x.string())

#-----------------------------------------------------------------------------#
#-----------------------------------------------------------------------------#



##---------------------------------------------------------------------------##
# Register for updates of the MOOS variables Landmarks and Nav_Aids once every
#   second
##---------------------------------------------------------------------------##
#def on_connect():
#    comms.register('Landmarks', 0)
#    comms.register('Nav_Aids', 0)
#    comms.register('Underwater_Objects', 0)
#    return True
#    
## Initialize the MOOS connection and communication
#comms = pymoos.comms()
#comms.set_on_connect_callback(on_connect);
#comms.run('localhost',9000,'Camera')
#
## Declare the MOOS lists as GLOBALS
#global LANDMARKS
#global NAV_AIDS
#global UNDERWATER
#
## Initialize the MOOS lists
#LANDMARKS = []
#NAV_AIDS  = []
#UNDERWATER = []

global MOOS

# Initialize MOOS
MOOS = MOOS_comms()
MOOS.Initialize()

# Run the GUI - it updates with new MOOS Variables every second
root=tk.Tk()
root.geometry("800x300+300+300")
Draw()
Refresher()
root.mainloop()


