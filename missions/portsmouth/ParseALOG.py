# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 15:49:07 2017

@author: sreed
"""

from matplotlib import pyplot as plt
import numpy as np
from scipy.io import savemat

folder = '/home/sreed/moos-ivp/moos-ivp-reed/missions/portsmouth/POLY_24_10_2017_____16_58_20'
filename = '{}/p2.alog'.format(folder)
with open(filename) as f:
    polys = []
    cntr =-1
    for line in f:
        cntr +=1
        xy = []
        xy_string = line.split('{')[-1].split('}')[0].split(':')
        for i in range(len(xy_string)):
            temp = xy_string[i].split(',')
            if len(temp) == 2:
                x=float(temp[0])
                y=float(temp[1])
                if (-150<=x<=350) and (-300<=y<=150):
                    xy.append(temp)
            else:
                print temp
        XY = np.array(xy, dtype=np.float)
        polys.append(XY)
        
mat_dict = {}
for ii in range(len(polys)):
    plt.plot(polys[ii][:,0], polys[ii][:,1])
    if ii<2:
         mat_dict['Land{}'.format(ii)] = polys[ii]
    else:
         mat_dict['Shallow{}'.format(ii)] = polys[ii]
   
savemat('../../polys2.mat', mat_dict)
#sed -i 1,5d p2.alog