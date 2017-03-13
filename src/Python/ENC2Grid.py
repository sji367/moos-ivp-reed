# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:24:18 2017

@author: sji367
"""
import numpy as np
import ogr, gdal
from scipy.interpolate import griddata
from scipy.io import savemat
from time import time

class ENC_Grid():
    def __init__(self, filename_point= '../../src/ENCs/Shape/grid/Point.shp',
         filename_poly= '../../src/ENCs/Shape/grid/Poly.shp',
         filename_line= '../../src/ENCs/Shape/grid/Line.shp', grid_size = 20):
        
        self.filename_point = filename_point
        self.filename_poly = filename_poly
        self.filename_line = filename_line
        self.grid_size = grid_size
        
        # Build the list of the point, polygon and line shapefiles
        self.BuildLayers()
        
        # Maximum extent of the grid
        extent = self.multiLayer_envelope()
        self.x_min = extent[0]
        self.x_max = extent[1]
        self.y_min = extent[2]
        self.y_max = extent[3]
        
        # For timers
        self.start = time()
        
    def shp2array(self, source_layer, NoData_value = 255, burn_depth=False):
        """ This function rasterizes the inputed shapefile layer. To do so, it  
            uses the predefinded grid size (from __init__) and maximum extent 
            of all the layers. All sections of the grid will be filled with the
            either the depth information or ones.
            
            Inputs:
                source_layer - GDAL layer that we are rasterizing
                NoData_value - Value used if there is no data 
                    (in our case, we dont have this)
                burn_depth - Boolean that if True burns the Depth attribute to 
                    the raster and if false burns 1 
                
            Outputs:
                array - rasterized layer stored in a numpy array
        """        
        # Create the destination data source
        x_res = int((self.x_max - self.x_min) / self.grid_size)
        y_res = int((self.y_max - self.y_min) / self.grid_size)
        target_ds = gdal.GetDriverByName('MEM').Create('', x_res, y_res, gdal.GDT_Byte)
        target_ds.SetGeoTransform((self.x_min, self.grid_size, 0, self.y_max, 0, -self.grid_size))
        band = target_ds.GetRasterBand(1)
        band.Fill(np.nan)
        band.SetNoDataValue(NoData_value)
        
        # Rasterize
        if burn_depth:
            gdal.RasterizeLayer(target_ds, [1], source_layer, burn_values=[1], options=["ATTRIBUTE=Depth",'ALL_TOUCHED=True'])
        else:
            gdal.RasterizeLayer(target_ds, [1], source_layer, burn_values=[1], options=['ALL_TOUCHED=True'])
            
        # Read as array
        return band.ReadAsArray()
        
    def numRowsCols(self, array):
        """ Returns the number of rows and columns of a 2D matrix"""
        return len(array),len(array[0])
    
    def multiLayer_envelope(self):
        """ This function gets the maximum envelope for all of the layers. 
        
            Output:
                extent - [x_min, x_max, y_min, y_max]
        """
        # Set up the matrices
        x_min = np.zeros([len(self.layers)])
        x_max = np.zeros([len(self.layers)])
        y_min = np.zeros([len(self.layers)])
        y_max = np.zeros([len(self.layers)])
        
        # Fill the matrices with the envelopes of each layer
        for i in range(len(self.layers)):
            x_min[i], x_max[i], y_min[i], y_max[i] = self.layers[i].GetExtent()
            
        extent = [min(x_min), max(x_max), min(y_min), max(y_max)]
        
        # Make the envelope whole numbers
        for i in range(4):
            if extent[i] > 0:
                extent[i] = np.ceil(extent[i])
            else:
                extent[i] = np.floor(extent[i])
                
        return extent
 
    def point2XYZ(self, layer):
        """ This function takes in a GDAL point layer and outputs a numpy array
            of the XYZ data.
            
            Input:
                layer - GDAL point layer used to find the XYZ coordinates
                
            Output:
                matrix - 2D matrix (N x 3) of the raw XYZ data from the inputed
                    layer
        """
        layer.ResetReading()
        matrix = []
        # Store the XYZ data
        for i in range(layer.GetFeatureCount()):
            feat = layer.GetNextFeature()
            geom = feat.GetGeometryRef()
            x=geom.GetX()
            y=geom.GetY()
            z=feat.GetFieldAsDouble("Depth")
            matrix.append([x,y,z])
            
        return np.array(matrix)
        
    def line2XYZ(self, layer, max_segment=10):
        """ This function takes in a GDAL line layer (depth contour), 
            segmentizes each line andthen outputs a numpy array of the XYZ data 
            of each vertex.
            
            Input:
                layer - GDAL line layer used to find the XYZ coordinates
                
            Output:
                matrix - 2D matrix (N x 3) of the raw XYZ data from the inputed
                    layer
        """
        layer.ResetReading()
        matrix = []
        for i in range(layer.GetFeatureCount()):
            feat = layer.GetNextFeature()
            geom = feat.GetGeometryRef()
            # Segmentize the line so that the entire contour is taken into 
            #  account in the grid
            geom.Segmentize(max_segment)
            # Cycle through each point in the segmentized line and store the 
            #   XYZ coordinates of each vertex
            for iPnt in range(geom.GetPointCount()):
                x,y,z = geom.GetPoint(iPnt)
                z = feat.GetFieldAsDouble("Depth")
                matrix.append([x,y,z])
        return np.array(matrix)
        
    def poly2XYZ(self, layer, land_z = -10):
        """ This function takes in a GDAL polygon layer (land area) and outputs
             a numpy array of the XYZ data.
            
            Input:
                layer - Point Layer used to find the XYZ coordinates
                land_z - Depth value stored for land
                
            Output:
                matrix - 2D matrix (N x 3) of the raw XYZ data from the inputed
                    layer
        """
        # Rasterize the polygon layer (aka grid it)
        array_poly=self.shp2array(layer)
        # The non-zero points are the polygons
        no0=np.nonzero(array_poly)
        matrix = []
        # Now cycle through these points and store the XY data along with the 
        #   default land height data given by land_z
        for i in range(len(no0[0])):
            x = no0[1][i]*self.grid_size+12.5+self.x_min
            y = self.y_max-(no0[0][i]*self.grid_size+12.5)
            matrix.append([x,y,land_z])
        
        return np.array(matrix)
        
    def grid_data(self, rawXYZ, method = 'linear', Remove_nan=True, nan_value = -15):
        """ This function grids the raw XYZ data and does a linear 
                interpolation on the grid. 
            
            Inputs:
                rawXYZ - raw data points from the ENC to be gridded (N x 3 matrix)
                method - linear interpolation method 
                    (linear, cubic and )
                Remove_nan - If set, the NANs will be replaced with the value 
                    stored in nan_value
                nan_value - The value that the NANs will be replaced with if 
                    Remove_nan boolean is set
            
            Outputs:
                interp - Interpolated grid of the raw data (rawXYZ)
        """
        # Determine how many points should be in the grid
        numX = int((self.x_max-self.x_min)/self.grid_size)
        numY = int((self.y_max-self.y_min)/self.grid_size)
        
        # Since this is most likely not going to be a whole number find the
        #   upper bound    
        UB_x = numX*self.grid_size+self.x_min
        UB_y = numY*self.grid_size+self.y_min
        
        # Build grid
        x = np.linspace(self.x_min, UB_x, numX+1)
        y = np.linspace(self.y_min, UB_y, numY+1)
        X, Y = np.meshgrid(x,y)
        
        # Interpolate the data
        interp = griddata((rawXYZ.T[0], rawXYZ.T[1]), rawXYZ.T[2], (X, Y), method=method)
        
        # Remove all NANS
        if Remove_nan:
            n, m = self.numRowsCols(X)
            for i in range(n):
                for j in range(m):
                    if np.isnan(interp[i][j]):
                        interp[i][j] = nan_value
        return interp
    
    def BuildLayers(self):
        """ This function builds a list of layers 
                (soundings, contours and land areas)
        """
        self.layers=[] 
        self.ds_pnt = ogr.Open(self.filename_point)
        self.ds_poly = ogr.Open(self.filename_poly) 
        self.ds_line = ogr.Open(self.filename_line)
        self.layers.append(self.ds_pnt.GetLayer())
        self.layers.append(self.ds_poly.GetLayer())
        self.layers.append(self.ds_line.GetLayer())
    
    def Build_Grid(self):
        """ In this function, the data from the ENC (soundings, contours and 
                land areas) is put into a grid and interpolated.
        
            Outputs:
                raw - raw XYZ 2D array with all soundings, contours and land 
                        areas
                interp - interpolated grid of the raw XYZ data from the 
                        soundings, contours and land areas
        """
        # Get the raw data
        point = self.point2XYZ(self.layers[0])
        poly = self.poly2XYZ(self.layers[1])
        line = self.line2XYZ(self.layers[2])
        
        # Combine the raw data into 1 array
        raw = np.concatenate((point,poly,line))
        
        # Grid and interpolate the raw depth data
        interp = self.grid_data(raw)
        return  raw,interp

GRID = ENC_Grid() 
raw,interp = GRID.Build_Grid()
end = time()
print 'Total Time: {}'.format(end-GRID.start)
savemat('../../src/ENCs/Shape/grid/grid.mat', {'raw':raw,'interp':interp})  

# Get the Z values
#l.SetAttributeFilter(None)#"Type='SOUNDG'")
#z=[]
#for j in range(3):
#    l=layers[j]
#    l.ResetReading()
#    for i in range(l.GetFeatureCount()):
#        f = l.GetNextFeature()
#        g = f.GetGeometryRef()
#        wkt = g.ExportToWkt()    
#        z.append(f.GetFieldAsDouble("Depth"))
#
## Define pixel_size and NoData value of new raster
#pixel_size = 25
#NoData_value = 255
#
#array_pnt= shp2array(layers[0], extent)
##array_poly=shp2array(layers[1], extent)
#array_line=shp2array(layers[2], extent)
#
##array = make1array(array_pnt, array_poly, array_line)
#
##savemat('../../../../array.mat', {'point':array_pnt,'poly':array_poly, 'line':array_line, 'all':array})
#
#
#
#
##feat = layers[2].GetNextFeature()
##while(feat):
#   
#     
#matrix = poly2pnt(layers[1], extent)
#savemat('../../../../newer.mat', {'xy':matrix})  

#def make1array(array1, array2, array3):
#    # Convert all of the arrays to the same size
#    Row1, Col1 = self.numRowsCols(array1)
#    Row2, Col2 = numRowsCols(array2)
#    Row3, Col3 = numRowsCols(array3)
#    
#    maxRow = max([Row1, Row2, Row3])
#    maxCol = max([Col1, Col2, Col3])
#    
#    if maxRow > Row1:
#        array1 = np.concatenate((array1,np.zeros([maxRow-Row1,Col1])),axis=0)
#    if maxRow > Row2:
#        array2 = np.concatenate((array2,np.zeros([maxRow-Row2,Col2])),axis=0)
#    if maxRow > Row3:
#        array3 = np.concatenate((array3,np.zeros([maxRow-Row3,Col3])),axis=0)
#    if maxRow > Col1:
#        array1 = np.concatenate((array1,np.zeros([maxRow,maxCol-Col1])),axis=1)
#    if maxRow > Col2:
#        array2 = np.concatenate((array2,np.zeros([maxRow,maxCol-Col2])),axis=1)
#    if maxRow > Col3:
#        array3 = np.concatenate((array3,np.zeros([maxRow,maxCol-Col3])),axis=1)
#        
#    return np.maximum(np.maximum(array1,array2),array3)
    