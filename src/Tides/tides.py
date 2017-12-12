# -*- coding: utf-8 -*-
"""
Created on Sun Oct 23 08:43:51 2016

@author: mapper
"""
import sys
from datetime import datetime as dt
from pytides.tide import Tide
import pytides.constituent as cons
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0,'../Python/')
from parse_txt import parse_file

# To output to .mat
from scipy.io import savemat

def tidal_model(tide_station_name='Fort_Point'):
    """ This function gives a model of the predicted tides. To get tides at a 
        specific time, use the command:
            tide.at([datetime(year,month,day,hour,minute)])
        
    Input:
        tide_station_name - Name of the tide station for which we want the 
                    tidal model
    Output:
        tide_model - Model of the tides using NOAA's Harmonic Constituents 
    """
    # Determine which station was inputed - to make easier, we are converting 
    #   all inputs to a lowercase string
    ts = str(tide_station_name).lower()
    if (ts == 'fort_point' or ts == 'fort point' or ts == '8423898'):
        tide_station = 'Fort_Point'
    elif (ts == 'wells' or ts == 'wells, me' or ts == '8419317'):
        tide_station = 'Wells'
    elif (ts == 'boston' or ts == 'boston, ma' or ts == '8443970'):
        tide_station = 'Boston'
    else:
        print 'ERROR: Invalid Tide Station. Create a new file for {}.'.format(tide_station_name)
        return None
        
    station_filename = '{}/{}.txt'.format(tide_station,tide_station)
    
    station_info = parse_file(station_filename)

    # These are the NOAA constituents, in the order presented on their website.
    constituents = [c for c in cons.noaa if c != cons._Z0]
    
    # Phases and amplitudes (relative to GMT and in degrees and meters)
    published_amplitudes = station_info[0][1]
    published_phases = station_info[1][1]
    
    # We can add a constant offset (e.g. for a different datum, we will use
    #   relative to MLLW):
#    MTL = station_info[4][1]
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
    
    tide = Tide(model = model, radians = False)
    
    return tide

def prediction(tide, tide_station_name, start_date=dt(2016,10,23)):
    """ Predicts the tide for a week from the given start date (in GMT) and 
        prints out a plot of the Pytide tide model vs NOAA tide model vs Actual
        Tide.
        
    Inputs:
        tide - Model for the predicted tides from pytides using NOAA's
            Harmonic Constituents
        tide_station_name - Name of the tide station for which we want the 
                    tidal model
        start_date - date that you want the first measurement from
                format: datetime(year,month,day,hour,minute)
    """
    if tide:
        ##Prepare a list of datetimes, each 6 minutes apart, for a week.
        prediction_t0 = start_date
        hours = 0.1*np.arange(7 * 24 * 10)
        times = Tide._times(prediction_t0, hours)
        
        ##Prepare NOAA's results
        noaa_verified = []
        noaa_predicted = []
        
        with open('{}/actual.txt'.format(tide_station_name)) as f:
            for line in f:
                # temp stores the line we can store the first value as a float
                temp = line.split()
                noaa_verified.append(float(temp[0]))
          
        with open('{}/predict.txt'.format(tide_station_name)) as f1:
            for line in f1:
                # temp stores the line we can store the first value as a float
                temp = line.split()
                noaa_predicted.append(float(temp[0]))
        
        my_prediction = tide.at(times)
        fig, ax = plt.subplots( nrows=1, ncols=1 )  # create figure & 1 axis
        plt.plot(hours, my_prediction, label="Pytides")
        plt.plot(hours, noaa_predicted, label="NOAA Prediction")
        plt.plot(hours, noaa_verified, label="NOAA Verified")
        plt.legend()
        plt.title('Comparison of Pytides and NOAA predictions for {}'.format(tide_station_name))
        plt.xlabel('Hours since {} (GMT)'.format(prediction_t0))
        plt.ylabel('Meters')
        plt.savefig('{}/{}_Tides_2016_Oct23.png'.format(tide_station_name,tide_station_name))
        return my_prediction, noaa_predicted, noaa_verified, hours
        
    else:
        print 'ERROR: Invalid Tide Station. Create a new file for {}.'.format(tide_station_name)
    
tide_station_name='Fort_Point'
tide_FP = tidal_model(tide_station_name)
pytide_pt_FP, NOAA_pt_FP, at_FP, t = prediction(tide_FP, tide_station_name)
# Save as a .mat file for better plots
savemat('FP_Tides_2016_Oct23.mat', mdict={'Pytides': pytide_pt_FP, 'NOAA': NOAA_pt_FP, 'Actual': at_FP, 'Time' : t})

tide_station_name='Wells'
tide_Wells = tidal_model(tide_station_name)
pytide_pt_Wells, NOAA_pt_Wells, at_Wells, t = prediction(tide_Wells, tide_station_name)
# Save as a .mat file for better plots
savemat('WELLS_Tides_2016_Oct23.mat', mdict={'Pytides': pytide_pt_Wells, 'NOAA': NOAA_pt_Wells, 'Actual': at_Wells, 'Time' : t})

tide_station_name='Boston'
tide_Boston = tidal_model(tide_station_name)
pytide_pt_Boston, NOAA_pt_Boston, at_Boston, t = prediction(tide_Boston, tide_station_name)
# Save as a .mat file for better plots
savemat('BOS_Tides_2016_Oct23.mat', mdict={'Pytides': pytide_pt_Boston, 'NOAA': NOAA_pt_Boston, 'Actual': at_Boston, 'Time' : t})