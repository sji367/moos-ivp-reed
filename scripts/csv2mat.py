#!/usr/local/bin/python

# -*- coding: utf-8 -*-
"""
Created on Sat May 27 15:51:16 2017

@author: sji367
"""

import sys
import numpy as np
import scipy.io

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s input.csv output.mat" % sys.argv[0]
        sys.exit(-1)
    
    data_name = sys.argv[1].split('/')[-1]
    data_name = data_name.split('.')[0]
    my_data = np.genfromtxt(sys.argv[1], delimiter=',')
    # Remove the last column as the previous command creates an column of nans 
    #  at the end.
    fixeddata = np.delete(my_data, -1, 1) 
    scipy.io.savemat(sys.argv[2], {data_name:fixeddata})