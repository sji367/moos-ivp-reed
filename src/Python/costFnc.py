#!/usr/bin/env python
"""
Spyder Editor

This is a temporary script file.
"""

import matplotlib.pyplot as plt
import numpy as np

def origCost(t_lvl, ASV_Length, distance):
    return 2*np.power((t_lvl*ASV_Length)/distance,2)
    
def newCost(t_lvl, ASV_Length, distance):
    return np.exp(1.1*(distance-ASV_Length)/(1.0*ASV_Length))/(t_lvl**3)
#    return np.power(2.7182818284590451,(1.1*distance/(1.0*ASV_Length))/(t_lvl**4))
    
def newCost2(t_lvl, dist):
    c = np.zeros_like(dist)
    for i in range(len(dist)):
        c[i] = 2.5*np.power(2,dist[i])/np.power(t_lvl,2)#7*np.power((dist[i])-1,3)/(t_lvl**2.5)
    return c

l = 1.8
n = 101.0
d = np.linspace(0,l*10,n)
d2 = np.linspace(0,10,n)
threat = np.linspace(1,5,5)

#c1 = np.zeros([5,50])
#c2 = np.zeros([5,n])
for i in range(5):
#    c1[i,:] = origCost(threat[i],1.8, d)
#    c2[i,:] = newCost2(threat[i], d)
    c = origCost(threat[i],l, d)
    c2 = newCost(threat[i],l, d)
    c3 = newCost2(threat[i],d2)
    c[c>100] = 100
    c = np.ones_like(c)*100-c
    c2[c2>100] = 100
    c3[c3>100] = 100
    plt.plot(d2,c3)

plt.plot([3, 3], [0,100])
plt.xlabel('Distance from Obstacle (ASV Lengths)')
plt.ylabel('Utility')
plt.title('Utility Function for Any Size ASV')
plt.show()

#def gauss(x, mu, sigma, amplitude):
#    return 100-np.exp(-np.square(x-mu)/(2*np.power(sigma,2)))*amplitude
#    
#g = []
#for i in range(-20,21):
#    g.append(gauss(float(i),0.0,8.0,100.0-30.0))
#
#plt.plot(np.linspace(-20,20,41), g)
#plt.show()

