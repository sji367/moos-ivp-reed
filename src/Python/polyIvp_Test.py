# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 15:20:04 2017

@author: sreed
"""


from osgeo import ogr

ds = ogr.Open('../../src/ENCs/Shape/Poly.shp')
layer = ds.GetLayerByName("poly")

pnt = ogr.Geometry(ogr.wkbPoint)
layer.SetSpatialFilterRect(-105,-138,94,20)
#pnt.SetX(0)
#pnt.SetY(0)

cntr = 0
layer.ResetReading()
feat = layer.GetNextFeature()
feat = layer.GetNextFeature()
while(feat):
    geom = feat.GetGeometryRef()
    area = geom.GetArea()
    print int(round(area/25)), feat.GetFID()
    feat = layer.GetNextFeature()
    cntr += 1