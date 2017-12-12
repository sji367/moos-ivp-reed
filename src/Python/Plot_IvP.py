#!/usr/bin/env python

"""
Created on Sun Jul 23 17:13:57 2017

@author: sji367
"""

import numpy as np
from matplotlib import pyplot as plt
import pymoos
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
        self.WPT = []
        self.poly = []
        self.point =[]
        self.heading = []
        self.angles = []
        self.util = []
        self.buffer_width = []
        

    def Register_Vars(self):
        """ Register for the ID for the Tide Station.
        """
        self.comms.register('BHV_IPF', 0)
        self.comms.register('DESIRED_HEADING', 0)
        self.comms.register('Angles', 0)
        self.comms.register('Util1', 0)
        self.comms.register('Buffer_Width', 0)
        
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
        self.comms.run('localhost',9000,'Plot_IvP')
        
    def Get_mail(self):
        """ Get the new value for the name of the tide station """
        # Fetch the name of the tide station
        info = self.comms.fetch()
        
        # Store the tide station's name
        for x in info:
            if x.name()=='BHV_IPF':
                parse = (x.string()).split(',')[1].split('^')[0]
                if parse == 'waypt_survey':
                    self.WPT.append(x.string())
                elif parse == 'poly_OA': 
                    self.poly.append(x.string())
                elif parse == 'pnt_OA':
                    self.point.append(x.string())
                    
            elif x.name()=='DESIRED_HEADING':
                self.heading.append(x.double())
                
            elif x.name() == 'Angles':
                self.angles.append(x.string())
            elif x.name() == 'Util1':
                self.util.append(x.string())
            elif x.name() == 'Buffer_Width':
                self.buffer_width.append(x.double())
                


class plotIvP(object):
    def __init__(self, plotOAStars= False):
        self.nums2ave = 7
        self.r_wpt = np.zeros([self.nums2ave])
        self.des_head = 0.0
        self.plot={}
        self.needsInit = True
        self.theta = np.deg2rad(range(360))
        self.BW = 20
        self.plotOAStars= plotOAStars
        
        self.init_MOOS()
        
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
        
    def ang4MOOS(self, angle):
        return (90-angle)%360
        
    def MOOSAngle(self, angle):
        return (angle-90*np.ones_like(angle))%360
    
    def parseWPT(self, IPFstring):
        
        # Parse the WPT IFP string 
        parsed = IPFstring.split(',')
        pieces = int(parsed[8])
#        speed_num = int(parsed[12].split(';')[-1])
        WPT = np.array(parsed[17:], dtype=np.float)
        wpt = np.reshape(WPT, [pieces,7])
        utility = np.zeros([360,self.nums2ave])
        
        # Create a heading only IvP function (for simplicity) by averaging the
        #   3 values on either side of the desired speed to get one utility value
        #   for each heading
        for i in range(pieces):
            for j in range(int(wpt[i,0]),int(wpt[i,1])+1): # heading
                ang = self.ang4MOOS(j)
                for  speed_index in range(int(wpt[i,2]),int( wpt[i,3])+1):# speed
                    if 17<=speed_index <=23:
                        utility[ang,speed_index-17] = wpt[i,4]*j+wpt[i,5]*speed_index+wpt[i,6]
                        
        return np.sum(utility,axis=1)/self.nums2ave
                        
    def parseOA(self, OAstring, rawStringList):
        # Parse the OA string
        skip=1
        parsed = OAstring.split(',')
        
        # When the number of lines that the IvP function is on is greater than 1
        #   we need to add the data from the additional lines to the first
        while int(parsed[3])>1:
            if len(rawStringList)-(1+skip)>=0:
                parsed2 = rawStringList[len(rawStringList)-(1+skip)].split(',')
                skip+=1
                parsed.extend(parsed2[4:])
            else:
                return np.zeros([360])
            
        pieces = int(parsed[8])
        oa = np.array(parsed[16:], dtype=np.float)
        OA = np.reshape(oa, [pieces,4])
        utility = np.zeros([360])
        
        # Create the IvP function for the OA behavior
        for i in range(pieces):
            for j in range(int(OA[i,0]),int(OA[i,1])+1): # heading
                ang = self.ang4MOOS(j)
                utility[ang]= (OA[i,2]*j+OA[i,3])
                
        return utility
        
    def updatePlot(self, pl, x, y, polar=True,clear=False):
        if clear:
            plt.savefig('plot.pdf')
            pl.set_xdata([])
            pl.set_ydata([])
        else:
            if polar:
                pl.set_xdata(np.multiply(y,np.cos(x)))
                pl.set_ydata(np.multiply(y,np.sin(x)))
            else:
                pl.set_xdata(x)
                pl.set_ydata(y)
        
    def plotIvP(self, utility_wpt, utility_pnt, utility_poly, utility_total):
        if self.gotData:
            # Clear the old plot
            if not self.needsInit:
                fig= plt.gca()
                fig.clear()
            else:
                self.needsInit = False
            
            self.plot[0], = plt.polar(self.theta, utility_wpt, 'r', linewidth=4, label='WPT IvP')
#            self.plot[1], = plt.polar(self.theta, utility_pnt, 'k', linewidth=4, label='Point_OA IvP')
            self.plot[2], = plt.polar(self.theta, utility_poly, 'b', linewidth=4, label='PolyOA IvP')
            self.plot[3], = plt.polar(self.theta, utility_total, 'g', linewidth=4, label='ALL IvPs')
            self.plot[4], = plt.polar(np.deg2rad(self.des_head), utility_total[int(self.des_head)], 'm*', markersize =20, label='Desired\nHeading')
            plt.legend()
    
    def run(self):
        while True:            
            self.MOOS.Get_mail()
            util_poly = np.zeros(360)
            util_pnt = np.zeros(360)
            util_WPT = np.zeros(360)
            
            active_BHV = 0
            
            # Only update plot if there is a new IvP Function
            if (len(self.MOOS.poly)!=0 or len(self.MOOS.WPT)!=0):
                self.gotData = True
            else:
                self.gotData = False
            
            # Parse the Polygon OA IvP Function
            if len(self.MOOS.poly)!=0:
                util_poly = self.parseOA(self.MOOS.poly[-1], self.MOOS.poly)
                self.MOOS.poly=[]
                active_BHV += 1
                
            # Parse the WPT IvP Function
            if len(self.MOOS.WPT)!=0:
                util_WPT = self.parseWPT(self.MOOS.WPT[-1])
                self.MOOS.WPT=[]
                active_BHV += 1
                
                if len(self.MOOS.heading)!=0:
                    # Get the most recent desired heading
                    self.des_head = self.ang4MOOS(self.MOOS.heading[-1])
#                    print self.des_head 
                    self.MOOS.heading = []
                    
            # Parse the Point OA IvP Function
            if len(self.MOOS.point)!=0:
                util_pnt = self.parseOA(self.MOOS.point[-1], self.MOOS.point)
                self.MOOS.point=[]
                active_BHV += 1
                
            utility_total = util_poly+util_WPT+util_pnt
            
                
            # Parse the utility for the outer bounds of the IvP Function
            if len(self.MOOS.util)!=0:
                parse_util = self.MOOS.util[-1].split(',')
                self.MOOS.util=[]
                
                util = np.array(parse_util, dtype=np.float)
                Utility = 100*np.ones_like(util)-util
                Utility[Utility<0] = 10
                gotUtil = True
            else:
                gotUtil = False
                
            # Parse the Buffer Width
            if len(self.MOOS.buffer_width)!=0:
                self.BW = self.MOOS.buffer_width[-1]
                self.MOOS.buffer_width = []
                
            # Parse the Angles
            if len(self.MOOS.angles)!=0:
                parse_ang = self.MOOS.angles[-1].split(',')
                self.MOOS.angles=[]
                angles = np.deg2rad(self.ang4MOOS(np.array(parse_ang, dtype=np.float))+np.array([self.BW, 0, -self.BW]))
            
            # Make sure all of the data is on the same scale
            if active_BHV >0:
                self.plotIvP(util_WPT, util_pnt, util_poly, utility_total/active_BHV)
            else:
                fig= plt.gca()
                fig.clear()
                
            if gotUtil and self.plotOAStars:
                self.plot[5], = plt.plot(angles, Utility, 'b*', markersize =10)
            fig = plt.gca(projection='polar')
            fig.set_rlim(110)
            plt.legend(bbox_to_anchor=(1.34, 1.14), bbox_transform=fig.transAxes)
            #fig.set_title('Util: {}, Buffer: {}'.format(U,self.BW))
            plt.pause(.1)
            
if __name__ == '__main__':
    p = plotIvP()
    p.run()    
    
#        fig = plt.gca()
#        fig.set_title('Simple Waypoint Mission')
#        fig.set_xlim([min(self.wptX)-10, max(self.wptX)+10])
#        fig.set_ylim([min(self.wptY)-20, max(self.wptY)+20])
        


#test2 = 'P,poly_OA^3183,1,1,H,12,3183:poly_OA,1,48,1,100,D,course;0;359;360,G,359,F,0,2,0,79.,3,5,0,80.,6,8,0,81.,9,11,0,82.,12,14,0,83.,15,17,0,84.,18,20,0,85.,21,24,0,86.,25,27,0,87.,28,31,0,88.,32,34,0,89.,35,38,0,90.,39,42,0,91.,43,46,0,92.,47,51,0,93.,52,55,0,94.,56,61,0,95.,62,67,0,96.,68,74,0,97.,75,83,0,98.,84,104,0,99.,105,106,-1.,205.,107,120,0,99.,121,299,0,0,300,301,1.,-245.,302,303,1.,-246.,304,305,0,57.,306,307,0,58.,308,310,0,59.,311,312,0,60.,313,315,0,61.,316,317,0,62.,318,320,0,63.,321,322,0,64.,323,325,0,65.,326,328,0,66.,329,330,0,67.,331,333,0,68.,334,335,0,69.,336,338,0,70.,339,340,0,71.,341,343,0,72.,344,346,0,73.,347,348,0,74.,349,351,0,75.,352,354,0,76.,355,357,0,77.,358,359,0.5,-101'
#test = 'P,waypt_survey^3205,1,1,H,17,3205:waypt_survey,2,28,1,100,D,course;0;359;360:speed;0;6;41,G,359,40,F,0,74,0,10,0.2739,3.9474,20.9992,0,74,11,20,0.2739,1.0526,49.9465,0,74,21,30,0.2739,-1.0526,92.0518,0,74,31,40,0.2739,-3.9474,178.8939,75,102,0,10,0.3097,3.9474,18.3441,75,102,11,20,0.3097,1.0526,47.2915,75,102,21,30,0.3097,-1.0526,89.3968,75,102,31,40,0.3097,-3.9474,176.2389,103,104,0,10,-0.1667,3.9474,67.1742,103,104,11,20,-0.1667,1.0526,96.1216,103,104,21,30,-0.1667,-1.0526,138.2268,103,104,31,40,-0.1667,-3.9474,225.069,105,134,0,10,-0.3097,3.9474,82.0819,105,134,11,20,-0.3097,1.0526,111.0292,105,134,21,30,-0.3097,-1.0526,153.1345,105,134,31,40,-0.3097,-3.9474,239.9766,135,282,0,10,-0.2739,3.9474,77.2831,135,282,11,20,-0.2739,1.0526,106.2304,135,282,21,30,-0.2739,-1.0526,148.3357,135,282,31,40,-0.2739,-3.9474,235.1778,283,284,0,10,0.2025,3.9474,-57.299,283,284,11,20,0.2025,1.0526,-28.3516,283,284,21,30,0.2025,-1.0526,13.7536,283,284,31,40,0.2025,-3.9474,100.5957,285,359,0,10,0.2739,3.9474,-77.6156,285,359,11,20,0.2739,1.0526,-48.6682,285,359,21,30,0.2739,-1.0526,-6.563,285,359,31,40,0.2739,-3.9474,80.2791'
##parsed = test.split(',')
##pieces = int(parsed[8])
##speed_num = int(parsed[12].split(';')[-1])
##start = 17
##
###w = '0,114,0,13,0.2791,0,6.8112,0,114,14,20,0.2791,6.0976,-78.1888,0,114,21,41,0.2791,0.4878,36.8112,115,144,0,13,0.3155,0,2.6476,115,144,14,20,0.3155,6.0976,-82.3524,115,144,21,41,0.3155,0.4878,32.6476,145,155,0,13,0.1699,0,23.6701,145,155,14,20,0.1699,6.0976,-61.3299,145,155,21,41,0.1699,0.4878,53.6701,156,174,0,13,-0.3155,0,99.064,156,174,14,20,-0.3155,6.0976,14.064,156,174,21,41,-0.3155,0.4878,129.064,175,324,0,13,-0.2791,0,92.7164,175,324,14,20,-0.2791,6.0976,7.7164,175,324,21,41,-0.2791,0.4878,122.7164,325,335,0,13,-0.2063,0,69.1009,325,335,14,20,-0.2063,6.0976,-15.8991,325,335,21,41,-0.2063,0.4878,99.1009,336,359,0,13,0.2791,0,-93.655,336,359,14,20,0.2791,6.0976,-178.655,336,359,21,41,0.2791,0.4878,-63.655'
###wpt = np.array(w.split(','), dtype=np.float)
##wpt = np.array(parsed[start:], dtype=np.float)
##wpt2 = np.reshape(wpt, [pieces,7])
##
###num_pts = 360*10
###X = np.zeros([num_pts])
###Y = np.zeros([num_pts])
##z_wpt = np.zeros([360,8])
##
##cntr = 0
##
##for i in range(pieces):
##    for j in range(int(wpt2[i,0]),int(wpt2[i,1])+1): # heading
###        ang = np.deg2rad((90-j)%360)
##        ang = (90-j)%360
###        print ang
##        for  speed_index in range(int(wpt2[i,2]),int( wpt2[i,3])+1):# speed
##            if 17<=speed_index <=23:
###                X[cntr] = speed_index*np.cos(ang)
###                Y[cntr] = speed_index*np.cos(ang)
###                z_wpt[cntr] = wpt2[i,4]*j+wpt2[i,5]*speed_index+wpt2[i,6]
###                cntr+=1
##                z_wpt[ang,speed_index-17] = wpt2[i,4]*j+wpt2[i,5]*speed_index+wpt2[i,6]
#
#p = plotIvP()
#r=p.parseWPT(test) 
#r+= p.parseOA(test2)
#
#print np.argmax(r)
#
#theta = np.deg2rad(range(360))
##r = np.sum(z_wpt,axis=1)/7
#        
##plot = {}
#fig = plt.figure()
#
#ax = fig.gca(projection='polar')
#ax.plot(theta, r)
#plt.show()
##ax = fig.gca(projection='3d')
##X, Y = np.meshgrid(X, Y)
##ax.plot_surface(X,Y,z_wpt)
##plt.show()



        