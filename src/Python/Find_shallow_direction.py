# -*- coding: utf-8 -*-
"""
Created on Tue Dec 27 13:17:58 2016

@author: sji367
"""

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

import numpy as np

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 


# Define which UTM zone we are in
LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
LatOrigin=43.071959194444446
LongOrigin=-70.711610833333339
x_origin, y_origin = LonLat2UTM(LongOrigin, LatOrigin)

def ang4MOOS(angle, array = False):
    """ This fucntion converts a typical angle to one that is 0 degrees at
    North
    
    Input:
        angle - Angle or angles to be converted
        array - Boolean if the angle is an array
        
    Outputs:
        The angle(s) in MOOS's coordinate system
    """
    if array:
        return np.mod(-(angle - np.ones_like(angle)*90),360)
    else:
        return np.mod(-(angle-90), 360)
        
def LonLat2MOOSxy(lon, lat):
    """ This function converts Longitude and Latitude to MOOS X and Y
    
    Inputs:
        lat - Latitude to be converted to UTM
        lon - Longitude to be converted to UTM
        
    Output:
        x - Corresponding X location in UTM
        y - Corresponding Y location in UTM
    """
    x,y = LonLat2UTM(lon, lat)
    x += -x_origin
    y += -y_origin
    return x,y  
        
def MOOSxy2LonLat(x, y):
    """ This function converts MOOS X,Y to Longitude and Latitude
    
    Inputs:
        x - X location in UTM coordinates
        y - Y location in UTM coordinates
        
    Output:
        lat - Corresponding latitude location
        lon - Corresponding longitude location
    """
    lat,lon = LonLat2UTM(x+x_origin, y+y_origin, inverse=True)
    return lat,lon


def dist(x1, y1, x2, y2):
    return np.sqrt(np.square(x1-np.ones_like(x1)*x2)+np.square(y1-np.ones_like(y1)*y2))
    

#-----------------------------------------------------------------------------#
#                            Line Code                                        #
#-----------------------------------------------------------------------------#
filename_pnt='../../src/ENCs/Shape/Point.shp'
filename_poly='../../src/ENCs/Shape/Poly.shp'
filename_line='../../src/ENCs/Shape/Line.shp'

driver = ogr.GetDriverByName('ESRI Shapefile')

# Get the Datasourse
ds_pnt = driver.Open(filename_pnt, 0)
ds_poly = driver.Open(filename_poly, 0)
ds_line = driver.Open(filename_line, 0)

ENC_point_layer = ds_pnt.GetLayer()
ENC_poly_layer = ds_poly.GetLayer()
ENC_line_layer = ds_line.GetLayer()

# Isolate the Soundings
ENC_point_layer.SetAttributeFilter("Type='SOUNDG'")
ENC_line_layer.SetAttributeFilter("Type='DEPCNT'")

soundings = ENC_point_layer

ENC_poly_layer.ResetReading()
soundings.ResetReading()
ENC_line_layer.ResetReading()

# Store X,Y, Z of the soundings
sound_x = []
sound_y = []
sound_z = []
sound_index = []
for j in range(soundings.GetFeatureCount()):
    feat_pnt = soundings.GetNextFeature()
    geom_point = feat_pnt.GetGeometryRef()
    x, y = LonLat2MOOSxy(geom_point.GetX(), geom_point.GetY())
    sound_x.append(x)
    sound_y.append(y)
    sound_z.append(feat_pnt.GetFieldAsDouble("Depth"))
    sound_index.append(j)
  
min_dist_vertex = []
min_dist2contour = []
sounding_index = []
closest_sounding_depth = []
contour_line_index = []
z_contour= []
deep = []
sounding_direction = []
result = []
cntr=0
Reverse = []
feat_line = ENC_line_layer.GetNextFeature()
iii=0
while(iii<2):#feat_line):
    iii += 1
    soundings.ResetReading()
    soundings.SetAttributeFilter(None)
    min_dist_vertex = []
    vertex_index = []
    same_depth=[]
    geom_line = feat_line.GetGeometryRef()
    num_vertices = geom_line.GetPointCount()
    
    z = feat_line.GetFieldAsDouble("Depth")
    z_contour.append(z)
    
    # Get the feature ID for all soundings with the same depth as the contour
    #  so that we can remove them later.
    soundings.SetAttributeFilter('Depth='+str(z))
    for ii in range(soundings.GetFeatureCount()):
        feat = soundings.GetNextFeature()
        same_depth.append(feat.GetFID())
        
    # Remove all of the soundings with the same depth as the contour
    new_sound_x = np.delete(sound_x, same_depth)
    new_sound_y = np.delete(sound_y, same_depth)
    new_sound_z = np.delete(sound_y, same_depth)  
    new_sound_index = np.delete(sound_index, same_depth)

    # Calculate the min distance to each vertex    
    for i in range(num_vertices):
        # GetPoint returns a tuple not a Geometry
        pt = geom_line.GetPoint(i)
        vertex_x, vertex_y = LonLat2MOOSxy(pt[0],pt[1])
        
        # Calculate the distance to each sounding from the vertex pt and store
        #  the smallest value
        distance = dist(new_sound_x,new_sound_y,vertex_x,vertex_y)
        
        min_dist_vertex.append(np.min(distance)) # Distance from the vertex of the contour to the nearest sounding
        vertex_index.append(np.argmin(distance)) # Index of the nearest sounding to the current vertex
    # Store the smallest distance to a sounding for the contour
    min_dist2contour.append(np.min(min_dist_vertex)) # Distance from the contour to the nearest sounding
    contour_line_index.append(np.argmin(min_dist_vertex)) # Vertex of the contour that is closest to the nearest sounding
    sounding_index.append(vertex_index[np.argmin(min_dist_vertex)]) # Index of the closest sounding 
    closest_sounding_depth.append(new_sound_z[sounding_index[-1]]) # Depth of the closest sounding
    
    ## Determine which direction points toward shallow water
    close_pt = geom_line.GetPoint(contour_line_index[cntr])
    close_vertex_x, close_vertex_y = LonLat2MOOSxy(close_pt[0],close_pt[1])
    num_vertices = geom_line.GetPointCount()

    # If the closest vertex is the first vertex in the contour, then the  
    #   previous vertex is the next vertex
    if contour_line_index[cntr]==0:
        Reverse.append(False) # Normalized angle to sounding with next vertex
        other_pt = geom_line.GetPoint(1)
    else:
        Reverse.append(True) # Normalized angle to sounding with next vertex
        other_pt = geom_line.GetPoint(contour_line_index[cntr]-1)
    other_vertex_x, other_vertex_y = LonLat2MOOSxy(other_pt[0],other_pt[1])
    
    # Calculate the angle to the sounding from the closest vertex along with 
    #  the angle to the next vertex
    angle2sounding = np.rad2deg(np.arctan2(close_vertex_y-sound_y[sounding_index[cntr]], close_vertex_x-sound_x[sounding_index[cntr]]))
    angle2next_pt = np.rad2deg(np.arctan2(close_vertex_y-other_vertex_y, close_vertex_x-other_vertex_x))
    
    # Determine if the contour line (True) or sounding (False) is deeper
    contour_deeper = z_contour[cntr]>closest_sounding_depth[cntr]
    if contour_deeper:
        deep.append('C')
    else:
        deep.append('S')
        
    # Print error message if the depths for the contour and sounding are equal 
    if z_contour[cntr]==closest_sounding_depth[cntr]:
        print 'ERROR --> Depths are equal for contour {}'.format(cntr)
        
    # Normalize the angle with respect to the next vertex
    normalized_ang = (angle2next_pt-angle2sounding)%360
    
    # Make sure the sounding is not in the left half plane
    # RHP = True, LHP = False
    contour_right_of_sounding = not 90<=normalized_ang<270
    if contour_right_of_sounding:
        sounding_direction.append('R')
    else:
        sounding_direction.append('L')
#    print cntr
#    print 'Sound: {}, Next: {}, Norm: {}'.format(angle2sounding, angle2next_pt, normalized_ang)        
#    print 'depth --> contour: {}, sounding: {}'.format(z_contour[cntr], closest_sounding_depth[cntr])
    
    deepH20right = contour_right_of_sounding != contour_deeper
    if (deepH20right):
#        print 'DEEP WATER --> Right\n'
        result.append('R')
    else:
#        print 'DEEP WATER --> Left\n'
        result.append('L')
    feat_line = ENC_line_layer.GetNextFeature()
    cntr += 1

        