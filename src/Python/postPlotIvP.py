#!/usr/bin/env python
"""
Created on Sun Nov 12 11:37:07 2017

@author: sreed
"""

import numpy as np
from matplotlib import pyplot as plt
import time

class plot_IvP(object):
    def __init__(self, plotOAStars= False):
        self.nums2ave = 7
        self.r_wpt = np.zeros([self.nums2ave])
        self.des_head = 0.0
        self.plot={}
        self.needsInit = True
        self.theta = np.deg2rad(range(-180,180))
        self.theta= np.rad2deg(self.theta)
        self.BW = 20
        self.plotOAStars= plotOAStars
        path = '../../missions/portsmouth/POLY_14_11_2017_____21_12_05/POLY_14_11_2017_____21_12_05_alvtmp/'
        self.poly = readALOG(path+'BHV_IPF_poly_OA.klog')
        self.WPT = readALOG(path+'BHV_IPF_waypt_survey.klog')
        self.desHead = readALOG(path+'DESIRED_HEADING.klog', True)
        self.s = 235
        self.end = 451
        self.gotData = True
        
    def plotPost(self):
        # Run a while loop to plot all desired timeseries
        while(True):
            new_input = raw_input('\nWhich one do you want to plot? (type E to exit)?\n')
            
            if new_input.upper() == 'E':
                exit(-1)
            else:
                self.plotIndex(int(new_input))
            
            plt.pause(1)
        
    def plotIndex(self,index):
        util_poly = np.zeros(360)
        util_WPT = np.zeros(360)
        active_BHV = 2
        
        util_WPT = self.parseWPT(self.WPT.IPF[index+self.s])
        util_poly = self.parseOA(self.poly.IPF[index], self.poly.IPF)
        self.des_head = self.ang4MOOS(self.desHead.head[index+self.s+6])
        
        utility_total = (util_poly+util_WPT)/active_BHV

        self.plotIvP(util_WPT, np.zeros(360), util_poly, utility_total)
        
    def ang4MOOS(self, angle):
        return (90-angle)%360
        
    def MOOSAngle(self, angle):
        return (angle-90*np.ones_like(angle))%360
    
    def parseWPT(self, IPFstring):
        # Parse the WPT IFP string
        des_speed = 1
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
                    if des_speed*10-3<=speed_index <=des_speed*10+3:
                        utility[ang,speed_index-des_speed*10-3] = wpt[i,4]*j+wpt[i,5]*speed_index+wpt[i,6]
                        
        wpt_util = np.sum(utility,axis=1)/self.nums2ave
        
        return wpt_util/np.max(wpt_util)*100.0
                        
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
#            np.deg2rad
            
            if (self.des_head<180):
                plot_head = (self.des_head)
            else:
                plot_head = self.des_head -360
            self.plot[4], = plt.plot(plot_head, utility_total[int(self.des_head)], 'm*', markersize =20, label='Desired\nHeading')
            utility_wpt = np.roll(utility_wpt,180)
            utility_poly = np.roll(utility_poly,180)
            utility_total = np.roll(utility_total,180)
            self.plot[0], = plt.plot(self.theta, utility_wpt, 'r', linewidth=4, label='WPT IvP')
#            self.plot[1], = plt.polar(self.theta, utility_pnt, 'k', linewidth=4, label='Point_OA IvP')
            self.plot[2], = plt.plot(self.theta, utility_poly, 'b', linewidth=4, label='PolyOA IvP')
            self.plot[3], = plt.plot(self.theta, utility_total, 'g', linewidth=4, label='ALL IvPs')
#            self.plot[4], = plt.plot((self.des_head)-180, utility_total[int(self.des_head)], 'm*', markersize =20, label='Desired\nHeading')
#            plt.legend()
    def runPost(self):
        for i in range(self.e-self.s):
            util_poly = np.zeros(360)
            util_WPT = np.zeros(360)
            active_BHV = 0
            
            
    def run(self):
        while True:
            util_poly = np.zeros(360)
            util_pnt = np.zeros(360)
            util_WPT = np.zeros(360)
            
            active_BHV = 0
            
            # Only update plot if there is a new IvP Function
            if (len(self.poly.IPF)!=0 or len(self.WPT.IPF)!=0):
                self.gotData = True
            else:
                self.gotData = False
            
            # Parse the Polygon OA IvP Function
            if len(self.poly)!=0:
                util_poly = self.parseOA(self.poly.IPF[-1], self.poly.IPF)
#                self.poly=[]
                active_BHV += 1
                
            # Parse the WPT IvP Function
            if len(self.WPT)!=0:
                util_WPT = self.parseWPT(self.WPT[-1])
#                self.WPT=[]
                active_BHV += 1
                
                if len(self.MOOS.heading)!=0:
                    # Get the most recent desired heading
                    self.des_head = self.ang4MOOS(self.heading[-1])
#                    print self.des_head 
#                    self.heading = []
                    
            # Parse the Point OA IvP Function
            if len(self.MOOS.point)!=0:
                util_pnt = self.parseOA(self.point[-1], self.point)
                self.point=[]
                active_BHV += 1
                
            utility_total = util_poly+util_WPT+util_pnt
            
            # Make sure all of the data is on the same scale
            if active_BHV >0:
                self.plotIvP(util_WPT, util_pnt, util_poly, utility_total/active_BHV)
            else:
                fig= plt.gca()
                fig.clear()
                
            fig = plt.gca(projection='polar')
            fig.set_rlim(110)
            plt.legend(bbox_to_anchor=(1.34, 1.14), bbox_transform=fig.transAxes)
            #fig.set_title('Util: {}, Buffer: {}'.format(U,self.BW))
            plt.pause(.1)
            
class readALOG(object):
    def __init__(self, filename, heading=False):
        self.filename= filename
        self.T = []
        if heading:
            self.head = []
            self.parseHeading()
        else:
            self.IPF=[]
            self.parseIVP()
        
    def parseIVP(self):
        with open(self.filename) as f:
            for line in f:
                cells = line.split(' ')
                self.T.append(float(cells[0]))
                self.IPF.append(cells[-2])
                
    def parseHeading(self):
        with open(self.filename) as f:
            for line in f:
                cells = line.split(' ')
                self.T.append(float(cells[0]))
                for i in range(1,len(cells)):
                    try:
                        self.head.append(float(cells[i]))
                        break
                    except ValueError:
                        pass
                
if __name__ == '__main__':
    p = plot_IvP()
    p.plotPost()