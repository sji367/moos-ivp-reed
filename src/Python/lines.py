# -*- coding: utf-8 -*-
"""
Created on Sat Jan 28 12:09:05 2017

@author: sji367
"""

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

from ENC_Contact import Search_ENC

e = Search_ENC()
m = e.Initialize()

line_layer = e.ENC_line_layer
pnt_layer = e.ENC_point_layer

pnt_layer.SetAttributeFilter("Type=SOUNDG")
line_layer.SetAttributeFilter("Type=DEPCNT")

line_layer.ResetReading()

for l in range(line_layer.GetFeatureCount()):
    line = line_layer.GetNextFeature()
    geom_line = line.GetGeometryRef()
    dist = 99999
    sounding = 0
    print 'type: {} Z: {}'.format(line.GetFieldAsString(3), line.GetFieldAsDouble(2))
    pnt_layer.ResetReading()
#    for i in range(pnt_layer.GetFeatureCount()):
#        pnt = pnt_layer.GetNextFeature()
#        geom_pnt = pnt.GetGeometryRef()
#        new_dist = geom_line.Distance(geom_pnt)
#        if (new_dist < dist):
#            dist = new_dist
#            sounding = pnt.GetFieldAsDouble(2)
#    print 'Line {}: Z: {}, dist {}, Sounding {}'.format(l,line.GetFieldAsDouble(2), dist, sounding)