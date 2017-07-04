# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:24:18 2017

@author: sji367
"""
import sys
import numpy as np
import ogr, gdal
from scipy.interpolate import griddata
from scipy.io import savemat
from time import time
from pandas import DataFrame

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# Define which UTM zone we are in



class ENC_Grid():
    def __init__(self, filename_point= '../../src/ENCs/Shape/grid/Point.shp',
         filename_poly= '../../src/ENCs/Shape/grid/Poly.shp',
         filename_line= '../../src/ENCs/Shape/grid/Line.shp', grid_size = 5):
        
        self.filename_point = filename_point
        self.filename_poly = filename_poly
        self.filename_line = filename_line
        self.grid_size = grid_size
        
        # Build the list of the point, polygon and line shapefiles
        self.BuildLayers()
        
        # Maximum extent of the grid
        self.extent = self.multiLayer_envelope()
        self.x_min = self.extent[0]
        self.x_max = self.extent[1]
        self.y_min = self.extent[2]
        self.y_max = self.extent[3]
        
        # To convert things to UTM
        self.LonLat2UTM = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
        self.LatOrigin=43.071959194444446
        self.LongOrigin=-70.711610833333339
        self.x_origin, y_origin = self.LonLat2UTM(self.LongOrigin, self.LatOrigin)        
        
        # For timers
        self.start = time()
        
    def LonLat2MOOSxy(self, lon, lat):
        """ This function converts Longitude and Latitude to MOOS X and Y
        
        Inputs:
            lat - Latitude to be converted to UTM
            lon - Longitude to be converted to UTM
            
        Output:
            x - Corresponding X location in UTM
            y - Corresponding Y location in UTM
        """
        x,y = self.LonLat2UTM(lon, lat)
        x += -self.x_origin
        y += -self.y_origin
        return x,y 
        
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
            matrix.append([x,y,self.convertZ2int(z)])
            
        return np.array(matrix)
        
    def line2XYZ(self, layer):
        """ This function takes in a GDAL line layer (depth contour), and 
            outputs a numpy array of the XYZ data of each vertex.
            
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
            # Cycle through each point in the segmentized line and store the 
            #   XYZ coordinates of each vertex
            for iPnt in range(geom.GetPointCount()):
                x,y,z = geom.GetPoint(iPnt)
                z = feat.GetFieldAsDouble("Depth")
                matrix.append([x,y,self.convertZ2int(z)])
        return np.array(matrix)
        
    def poly2XYZ(self, layer, land_z = -5):
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
        no0=np.nonzero(array_poly) # 
        matrix = []
        z = self.convertZ2int(land_z)
        # Now cycle through these points and store the XY data along with the 
        #   default land height data given by land_z
        for i in range(len(no0[0])):
            gridX = int(no0[1][i])
            gridY = int(no0[0][i])
            x = gridX*self.grid_size+self.grid_size/2+self.x_min
            y = self.y_max-(gridY*self.grid_size+self.grid_size/2)
            matrix.append([x,y,z])
        
        return np.array(matrix)
        
    def convertZ2int(self, z):
        return (z)*100
        
    def grid_data(self, rawXYZ, method = 'linear', Remove_nan=True, nan_value = -10):
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
        if Remove_nan:
            interp = griddata((rawXYZ.T[0], rawXYZ.T[1]), rawXYZ.T[2], (X, Y), method=method, fill_value=self.convertZ2int(nan_value))
        else:
            interp = griddata((rawXYZ.T[0], rawXYZ.T[1]), rawXYZ.T[2], (X, Y), method=method)

        interp.astype(int)
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
        print 'Points: {}'.format(time()-self.start)
        poly = self.poly2XYZ(self.layers[1])
        print 'Poly: {}'.format(time()-self.start)
        line = self.line2XYZ(self.layers[2])
        print 'Lines: {}'.format(time()-self.start)
        
        # Combine the raw data into 1 array
        raw = np.concatenate((point,poly,line))
        
        # Grid and interpolate the raw depth data
        interp = self.add_point_obs(self.grid_data(raw))
        return  raw,interp
        
    def xy2grid(self, x,y):
        """ Converts the local UTM to the grid coordinates. """
        gridX = (x-self.x_min-self.grid_size/2.0)/self.grid_size
        gridY = (y-self.y_min-self.grid_size/2.0)/self.grid_size
    
        return gridX,gridY
        
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local UTM. """
#        print '{} {}'.format(gridX, gridY)
        x = gridX*self.grid_size+self.x_min+self.grid_size/2.0
        y = gridY*self.grid_size+self.y_min+self.grid_size/2.0      
        
        return x,y
        
    def ENC_Outline(self):
        """ Gets an outline of the ENC Coverage and stores it to a .mat 
            file.
        """
        ## Store ENC Coverage
        ENC_filename='../../src/ENCs/US5NH02M/US5NH02M.000'
        ds = ogr.Open(ENC_filename)
        coverage = ds.GetLayerByName("M_COVR")
        coverage.ResetReading()
        area = []
        for i in range(coverage.GetFeatureCount()):
            feat = coverage.GetNextFeature()
            if feat.GetFieldAsDouble('CATCOV') == 1:
                geom = feat.GetGeometryRef()
                ring = geom.GetGeometryRef(0)
                for j in range (ring.GetPointCount()):
                    x,y = self.LonLat2MOOSxy(ring.GetX(j), ring.GetY(j))
                    x2,y2 = self.xy2grid(x,y)
                    area.append([x2,y2])
        savemat('../../src/ENCs/Shape/grid/outline.mat', {'outline':np.array(area)})
        
    def add_point_obs(self, interp):
        """ This adds the other points that have depth (ie not bouys, beacons, 
            lights, landmarks or soundings) to the grid. """
        ds = ogr.Open('../../src/ENCs/Shape/Point.shp')
        layer = ds.GetLayerByName('Point')
        layer.SetAttributeFilter("Type!='SOUNDG'")
        layer.ResetReading()
        feat = layer.GetNextFeature()
        while (feat):
            geom = feat.GetGeometryRef()
            x,y = geom.GetPoint_2D()
            grid_x, grid_y = self.xy2grid(x,y)
            depth = feat.GetFieldAsDouble('Depth')
            if depth != 9999:
                new_depth = self.convertZ2int(depth)
                interp[int(np.floor(grid_y))][int(grid_x)] = new_depth
                interp[int(grid_y)][int(np.round(grid_x))]=new_depth
                interp[int(np.round(grid_y))][int(grid_x)]=new_depth
                interp[int(np.round(grid_y))][int(np.round(grid_x))]=new_depth
            feat = layer.GetNextFeature()
            
        return interp


def main(argv):
    # Set the grid size if given
    if len(argv)>=1:
        GRID = ENC_Grid(grid_size=float(argv[0]))
    else:
        print 'Assume grid size is 5 meters'
        GRID = ENC_Grid()

    raw,interp = GRID.Build_Grid()   
    end = time()
    print 'Total Elapsed Time: {}'.format(end-GRID.start)
    # Set the type of output file    
    if len(argv)>1:
        output = str(argv[1]).upper()
    else:
        output = 'CSV'
    print 'Saving to {} file(s).'.format(output)
    
    # Save the data as either a .mat file or .csv file
    if output == 'MATLAB' or output == 'MAT':
        savemat('../../src/ENCs/Shape/grid/grid.mat', {'interp':interp})
    elif output == 'CSV':
        df = DataFrame(interp, dtype=int)
        df.to_csv("../../src/ENCs/Shape/grid/grid.csv")
        return df
    elif output == 'BOTH':
        savemat('../../src/ENCs/Shape/grid/grid.mat', {'interp':interp})
        df = DataFrame(interp, dtype=int)
        df.to_csv("../../src/ENCs/Shape/grid/grid.csv")
    elif output== 'NONE':
        pass
    else:
        print 'Invalid output file type. Use Matlab/CSV/Both/None.'
    
    return GRID

if __name__ == '__main__':
    if len(sys.argv) <= 3:
        if (sys.argv) > 1:
            grid = main(sys.argv[1:])
        else:
            grid = main()
    else:
        print('Usage: ENC2Grid.py grid_size Matlab/CSV/Both')
        print('       ENC2Grid.py grid_size')



















    